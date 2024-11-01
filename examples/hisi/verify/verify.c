/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2015 Intel Corporation.
 *   All rights reserved.
 */

#include "spdk/stdinc.h"

#include "spdk/hisi.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/queue.h"
#include "spdk/string.h"
#include "spdk/util.h"

#define SRC_BUFFER_SIZE (512*1024)

struct user_config {
	int queue_depth;
	int time_in_sec;
	char *core_mask;
};

struct hisi_device {
	struct spdk_hisi_chan *hisi;
	TAILQ_ENTRY(hisi_device) tailq;
};

static TAILQ_HEAD(, hisi_device) g_devices = TAILQ_HEAD_INITIALIZER(g_devices);
static struct hisi_device *g_next_device;

static struct user_config g_user_config;

struct thread_entry {
	struct spdk_hisi_chan *chan;
	uint64_t xfer_completed;
	uint64_t xfer_failed;
	uint64_t current_queue_depth;
	unsigned lcore_id;
	bool is_draining;
	bool init_failed;
	struct spdk_mempool *data_pool;
	struct spdk_mempool *task_pool;
};

struct hisi_task {
	struct thread_entry *thread_entry;
	void *buffer;
	int len;
	void *src;
	void *dst;
};

static __thread unsigned int seed = 0;

static unsigned char *g_src;

static void submit_single_xfer(struct hisi_task *hisi_task);

static void
construct_user_config(struct user_config *self)
{
	self->queue_depth = 32;
	self->time_in_sec = 10;
	self->core_mask = "0x1";
}

static void
dump_user_config(struct user_config *self)
{
	printf("User configuration:\n");
	printf("Run time:       %u seconds\n", self->time_in_sec);
	printf("Core mask:      %s\n", self->core_mask);
	printf("Queue depth:    %u\n", self->queue_depth);
}

static void
hisi_exit(void)
{
	struct hisi_device *dev;

	while (!TAILQ_EMPTY(&g_devices)) {
		dev = TAILQ_FIRST(&g_devices);
		TAILQ_REMOVE(&g_devices, dev, tailq);
		if (dev->hisi) {
			spdk_hisi_detach(dev->hisi);
		}
		free(dev);
	}
}
static void
prepare_hisi_task(struct thread_entry *thread_entry, struct hisi_task *hisi_task)
{
	int len;
	uintptr_t src_offset;
	uintptr_t dst_offset;

	src_offset = rand_r(&seed) % SRC_BUFFER_SIZE;
	len = rand_r(&seed) % (SRC_BUFFER_SIZE - src_offset);
	dst_offset = rand_r(&seed) % (SRC_BUFFER_SIZE - len);

	memset(hisi_task->buffer, 0, SRC_BUFFER_SIZE);
	hisi_task->src = (void *)((uintptr_t)g_src + src_offset);
	hisi_task->dst = (void *)((uintptr_t)hisi_task->buffer + dst_offset);
	hisi_task->len = len;
	hisi_task->thread_entry = thread_entry;
}

static void
hisi_done(void *cb_arg)
{
	struct hisi_task *hisi_task = (struct hisi_task *)cb_arg;
	struct thread_entry *thread_entry = hisi_task->thread_entry;

	if (memcmp(hisi_task->src, hisi_task->dst, hisi_task->len)) {
		thread_entry->xfer_failed++;
	} else {
		thread_entry->xfer_completed++;
	}

	thread_entry->current_queue_depth--;
	if (thread_entry->is_draining) {
		spdk_mempool_put(thread_entry->data_pool, hisi_task->buffer);
		spdk_mempool_put(thread_entry->task_pool, hisi_task);
	} else {
		prepare_hisi_task(thread_entry, hisi_task);
		submit_single_xfer(hisi_task);
	}
}

static bool
probe_cb(void *cb_ctx, struct spdk_pci_device *pci_dev)
{
	printf(" Found matching device at %04x:%02x:%02x.%x "
	       "vendor:0x%04x device:0x%04x\n",
	       spdk_pci_device_get_domain(pci_dev),
	       spdk_pci_device_get_bus(pci_dev), spdk_pci_device_get_dev(pci_dev),
	       spdk_pci_device_get_func(pci_dev),
	       spdk_pci_device_get_vendor_id(pci_dev), spdk_pci_device_get_device_id(pci_dev));

	return true;
}

static void
attach_cb(void *cb_ctx, struct spdk_pci_device *pci_dev, struct spdk_hisi_chan *hisi)
{
	struct hisi_device *dev;

	dev = malloc(sizeof(*dev));
	if (dev == NULL) {
		printf("Failed to allocate device struct\n");
		return;
	}
	memset(dev, 0, sizeof(*dev));

	dev->hisi = hisi;
	TAILQ_INSERT_TAIL(&g_devices, dev, tailq);
}

static int
hisi_init(void)
{
	if (spdk_hisi_probe(NULL, probe_cb, attach_cb) != 0) {
		fprintf(stderr, "hisi_probe() failed\n");
		return 1;
	}

	return 0;
}

static void
usage(char *program_name)
{
	printf("%s options\n", program_name);
	printf("\t[-h help message]\n");
	printf("\t[-c core mask for distributing I/O submission/completion work]\n");
	printf("\t[-t time in seconds]\n");
	printf("\t[-q queue depth]\n");
}

static int
parse_args(int argc, char **argv)
{
	int op;

	construct_user_config(&g_user_config);
	while ((op = getopt(argc, argv, "c:ht:q:")) != -1) {
		switch (op) {
		case 't':
			g_user_config.time_in_sec = spdk_strtol(optarg, 10);
			break;
		case 'c':
			g_user_config.core_mask = optarg;
			break;
		case 'q':
			g_user_config.queue_depth = spdk_strtol(optarg, 10);
			break;
		case 'h':
			usage(argv[0]);
			exit(0);
		default:
			usage(argv[0]);
			return 1;
		}
	}
	if (g_user_config.time_in_sec <= 0 || !g_user_config.core_mask ||
	    g_user_config.queue_depth <= 0) {
		usage(argv[0]);
		return 1;
	}

	return 0;
}

static void
drain_xfers(struct thread_entry *thread_entry)
{
	while (thread_entry->current_queue_depth > 0) {
		spdk_hisi_process_events(thread_entry->chan);
	}
}

static void
submit_single_xfer(struct hisi_task *hisi_task)
{
	spdk_hisi_submit_copy(hisi_task->thread_entry->chan, hisi_task, hisi_done,
					hisi_task->dst, hisi_task->src, hisi_task->len);
	hisi_task->thread_entry->current_queue_depth++;
	printf("submitted depth %u\n", hisi_task->thread_entry->current_queue_depth);
}

static void
submit_xfers(struct thread_entry *thread_entry, uint64_t queue_depth)
{
	while (queue_depth-- > 0) {
		struct hisi_task *hisi_task = NULL;
		hisi_task = spdk_mempool_get(thread_entry->task_pool);
		assert(hisi_task != NULL);
		hisi_task->buffer = spdk_mempool_get(thread_entry->data_pool);
		assert(hisi_task->buffer != NULL);

		prepare_hisi_task(thread_entry, hisi_task);
		submit_single_xfer(hisi_task);
	}
}

static int
work_fn(void *arg)
{
	uint64_t tsc_end;
	char buf_pool_name[20], task_pool_name[20];
	struct thread_entry *t = (struct thread_entry *)arg;

	if (!t->chan) {
		return 1;
	}

	t->lcore_id = spdk_env_get_current_core();

	snprintf(buf_pool_name, sizeof(buf_pool_name), "buf_pool_%u", t->lcore_id);
	snprintf(task_pool_name, sizeof(task_pool_name), "task_pool_%u", t->lcore_id);
	t->data_pool = spdk_mempool_create(buf_pool_name, g_user_config.queue_depth, SRC_BUFFER_SIZE,
					   SPDK_MEMPOOL_DEFAULT_CACHE_SIZE,
					   SPDK_ENV_SOCKET_ID_ANY);
	t->task_pool = spdk_mempool_create(task_pool_name, g_user_config.queue_depth,
					   sizeof(struct hisi_task),
					   SPDK_MEMPOOL_DEFAULT_CACHE_SIZE,
					   SPDK_ENV_SOCKET_ID_ANY);
	if (!t->data_pool || !t->task_pool) {
		fprintf(stderr, "Could not allocate buffer pool.\n");
		t->init_failed = true;
		return 1;
	}

	tsc_end = spdk_get_ticks() + g_user_config.time_in_sec * spdk_get_ticks_hz();

	submit_xfers(t, g_user_config.queue_depth);
	while (spdk_get_ticks() < tsc_end) {
		spdk_hisi_process_events(t->chan);
	}

	t->is_draining = true;
	drain_xfers(t);

	return 0;
}


static int
init_src_buffer(void)
{
	int i;

	g_src = spdk_dma_zmalloc(SRC_BUFFER_SIZE, 512, NULL);
	if (g_src == NULL) {
		fprintf(stderr, "Allocate src buffer failed\n");
		return 1;
	}

	for (i = 0; i < SRC_BUFFER_SIZE / 4; i++) {
		memset((g_src + (4 * i)), i, 4);
	}

	return 0;
}

static int
init(void)
{
	struct spdk_env_opts opts;

	spdk_env_opts_init(&opts);
	opts.name = "verify";
	opts.core_mask = g_user_config.core_mask;
	opts.iova_mode = "va";
	spdk_log_set_level(SPDK_LOG_DEBUG);
	spdk_log_open(NULL);
	if (spdk_env_init(&opts) < 0) {
		fprintf(stderr, "Unable to initialize SPDK env\n");
		return 1;
	}

	if (init_src_buffer() != 0) {
		fprintf(stderr, "Could not init src buffer\n");
		return 1;
	}
	if (hisi_init() != 0) {
		fprintf(stderr, "Could not init hisi\n");
		return 1;
	}

	return 0;
}

static int
dump_result(struct thread_entry *threads, uint32_t num_threads)
{
	uint32_t i;
	uint64_t total_completed = 0;
	uint64_t total_failed = 0;

	for (i = 0; i < num_threads; i++) {
		struct thread_entry *t = &threads[i];

		if (!t->chan) {
			continue;
		}

		if (t->init_failed) {
			total_failed++;
			continue;
		}

		total_completed += t->xfer_completed;
		total_failed += t->xfer_failed;
		if (total_completed || total_failed)
			printf("lcore = %d, copy success = %" PRIu64 ", copy failed = %" PRIu64 "\n",
			       t->lcore_id, t->xfer_completed, t->xfer_failed);
	}
	return total_failed ? 1 : 0;
}

static struct spdk_hisi_chan *
get_next_chan(void)
{
	struct spdk_hisi_chan *chan;

	if (g_next_device == NULL) {
		fprintf(stderr, "Not enough hisi channels found. Check that hisi channels are bound\n");
		fprintf(stderr, "to uio_pci_generic or vfio-pci.  scripts/setup.sh can help with this.\n");
		return NULL;
	}

	chan = g_next_device->hisi;

	g_next_device = TAILQ_NEXT(g_next_device, tailq);

	return chan;
}

static uint32_t
get_max_core(void)
{
	uint32_t i;
	uint32_t max_core = 0;

	SPDK_ENV_FOREACH_CORE(i) {
		if (i > max_core) {
			max_core = i;
		}
	}

	return max_core;
}

int
main(int argc, char **argv)
{
	uint32_t i, current_core;
	struct thread_entry *threads;
	uint32_t num_threads;
	int rc;

	if (parse_args(argc, argv) != 0) {
		return 1;
	}

	if (init() != 0) {
		return 1;
	}

	dump_user_config(&g_user_config);

	g_next_device = TAILQ_FIRST(&g_devices);

	num_threads = get_max_core() + 1;
	threads = calloc(num_threads, sizeof(*threads));
	if (!threads) {
		fprintf(stderr, "Thread memory allocation failed\n");
		rc = 1;
		goto cleanup;
	}

	current_core = spdk_env_get_current_core();
	SPDK_ENV_FOREACH_CORE(i) {
		if (i != current_core) {
			threads[i].chan = get_next_chan();
			spdk_env_thread_launch_pinned(i, work_fn, &threads[i]);
		}
	}

	threads[current_core].chan = get_next_chan();
	if (work_fn(&threads[current_core]) != 0) {
		rc = 1;
		goto cleanup;
	}

	spdk_env_thread_wait_all();
	rc = dump_result(threads, num_threads);

cleanup:
	spdk_dma_free(g_src);
	hisi_exit();
	free(threads);

	spdk_env_fini();
	return rc;
}
