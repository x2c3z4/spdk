/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2015 Intel Corporation.
 *   All rights reserved.
 */

#include "spdk/stdinc.h"

#include "hisi_internal.h"

#include "spdk/env.h"
#include "spdk/util.h"
#include "spdk/memory.h"

#include "spdk/log.h"
#include "spdk/likely.h"

#define hisi_roundup(x, y) ((((x) + ((y) - 1)) / (y)) * (y))
#define hisi_cacheline_roundup(x) hisi_roundup(x, 128)

struct hisi_driver {
	pthread_mutex_t			lock;
	TAILQ_HEAD(, spdk_hisi_chan)	attached_chans;
};


static uint8_t
hisi_dma_reg_layout(uint8_t revision)
{
	if (revision == HISI_DMA_REVISION_HIP08B)
		return HISI_DMA_REG_LAYOUT_HIP08;
	else if (revision >= HISI_DMA_REVISION_HIP09A)
		return HISI_DMA_REG_LAYOUT_HIP09;
	else
		return HISI_DMA_REG_LAYOUT_INVALID;
}

static int
hisi_dma_check_revision(struct spdk_pci_device *pci_dev,
			uint8_t *out_revision)
{
	uint8_t revision;
	int ret;

	ret = spdk_pci_device_cfg_read8(pci_dev, &revision, HISI_DMA_PCI_REVISION_ID_REG);
	if (ret) {
		SPDK_ERRLOG("read PCI revision failed!");
		return -EINVAL;
	}
	if (hisi_dma_reg_layout(revision) == HISI_DMA_REG_LAYOUT_INVALID) {
		SPDK_ERRLOG("revision: 0x%x not supported!", revision);
		return -EINVAL;
	}

	*out_revision = revision;
	return 0;
}

static uint32_t
hisi_dma_queue_base(struct hisi_dma_dev *hw)
{
	if (hw->reg_layout == HISI_DMA_REG_LAYOUT_HIP08)
		return HISI_DMA_HIP08_QUEUE_BASE;
	else if (hw->reg_layout == HISI_DMA_REG_LAYOUT_HIP09)
		return HISI_DMA_HIP09_QUEUE_BASE;
	else
		return 0;
}

static volatile void *
hisi_dma_queue_regaddr(struct hisi_dma_dev *hw, uint32_t qoff)
{
	uint32_t off = hisi_dma_queue_base(hw) +
			hw->queue_id * HISI_DMA_QUEUE_REGION_SIZE + qoff;
	return (volatile void *)((char *)hw->io_base + off);
}

static void
hisi_dma_write_reg(void *base, uint32_t off, uint32_t val)
{
	spdk_mmio_write_4((volatile void *)((char *)base + off), val);
}

static void
hisi_dma_write_dev(struct hisi_dma_dev *hw, uint32_t off, uint32_t val)
{
	hisi_dma_write_reg(hw->io_base, off, val);
}

static void
hisi_dma_write_queue(struct hisi_dma_dev *hw, uint32_t qoff, uint32_t val)
{
	uint32_t off = hisi_dma_queue_base(hw) +
			hw->queue_id * HISI_DMA_QUEUE_REGION_SIZE + qoff;
	hisi_dma_write_dev(hw, off, val);
}

static uint32_t
hisi_dma_read_reg(void *base, uint32_t off)
{
	return spdk_mmio_read_4((volatile void *)((char *)base + off));
}

uint32_t hisi_dma_read_dev(struct hisi_dma_dev *hw, uint32_t off)
{
	return hisi_dma_read_reg(hw->io_base, off);
}

uint32_t hisi_dma_read_queue(struct hisi_dma_dev *hw, uint32_t qoff)
{
	uint32_t off = hisi_dma_queue_base(hw) +
			hw->queue_id * HISI_DMA_QUEUE_REGION_SIZE + qoff;
	return hisi_dma_read_dev(hw, off);
}

static void
hisi_dma_update_bit(struct hisi_dma_dev *hw, uint32_t off, uint32_t pos,
		    bool set)
{
	uint32_t tmp = hisi_dma_read_dev(hw, off);
	uint32_t mask = 1u << pos;
	tmp = set ? tmp | mask : tmp & ~mask;
	hisi_dma_write_dev(hw, off, tmp);
}

static void
hisi_dma_update_queue_bit(struct hisi_dma_dev *hw, uint32_t qoff, uint32_t pos,
			  bool set)
{
	uint32_t tmp = hisi_dma_read_queue(hw, qoff);
	uint32_t mask = 1u << pos;
	tmp = set ? tmp | mask : tmp & ~mask;
	hisi_dma_write_queue(hw, qoff, tmp);
}

static void
hisi_dma_update_queue_mbit(struct hisi_dma_dev *hw, uint32_t qoff,
			   uint32_t mask, bool set)
{
	uint32_t tmp = hisi_dma_read_queue(hw, qoff);
	tmp = set ? tmp | mask : tmp & ~mask;
	hisi_dma_write_queue(hw, qoff, tmp);
}

static struct hisi_driver g_hisi_driver = {
	.lock = PTHREAD_MUTEX_INITIALIZER,
	.attached_chans = TAILQ_HEAD_INITIALIZER(g_hisi_driver.attached_chans),
};

static int
hisi_dma_get_status(struct hisi_dma_dev* hw)
{
	uint32_t val;

	val = hisi_dma_read_queue(hw, HISI_DMA_QUEUE_FSM_REG);
	val = FIELD_GET(HISI_DMA_QUEUE_FSM_STS_M, val);
	if (val > HISI_DMA_STATE_CPL) {
		SPDK_ERRLOG("bad status: 0x%d\n", val);
		hisi_dma_dump(hw);
		abort();
		return -1;
	}
	return val;
}

static int
hisi_map_pci_bar(struct spdk_hisi_chan *hisi)
{
	int rc;
	void *addr;
	uint64_t phys_addr, size;

	rc = spdk_pci_device_map_bar(hisi->device, 2, &addr, &phys_addr, &size);
	if (rc != 0 || addr == NULL) {
		SPDK_ERRLOG("pci_device_map_range failed with error code %d\n",
			    rc);
		return -1;
	}

	hisi->hw.io_base = addr;

	return 0;
}

static int
hisi_unmap_pci_bar(struct spdk_hisi_chan *hisi)
{
	int rc = 0;
	void *addr = (void *)hisi->hw.io_base;

	if (addr) {
		rc = spdk_pci_device_unmap_bar(hisi->device, 0, addr);
	}
	return rc;
}

static inline uint32_t
hisi_get_ring_space(struct hisi_dma_dev *hw)
{
	if (((hw->sq_tail + 1) & hw->sq_depth_mask) == hw->sq_head) {
		hw->qfulls++;
		return 0;
	}
	return 1;
}

static uint32_t
hisi_get_ring_index(struct hisi_dma_dev *hw, uint32_t index)
{
	return index & hw->sq_depth_mask;
}

static void
hisi_get_ring_entry(struct spdk_hisi_chan *hisi, uint32_t index,
		    struct hisi_descriptor **desc,
		    struct hisi_dma_sqe **hw_desc)
{
	struct hisi_dma_dev *hw = &hisi->hw;
	uint32_t i = hisi_get_ring_index(hw, index);

	*desc = &hisi->ring[i];
	*hw_desc = &hw->sqe[i];
}

static void
hisi_submit_single(struct hisi_dma_dev *hw)
{
	hw->sq_tail = (hw->sq_tail + 1) & hw->sq_depth_mask;
	// write sq doorbell
	spdk_mmio_write_4(hw->sq_tail_reg, hw->sq_tail);
}

static struct hisi_descriptor *
hisi_prep_copy(struct spdk_hisi_chan *hisi, uint64_t dst,
	       uint64_t src, uint32_t len)
{
	struct hisi_descriptor *desc;
	struct hisi_dma_sqe *sqe;

	struct hisi_dma_dev *hw =&hisi->hw;

	if (hisi_get_ring_space(hw) == 0) {
		return NULL;
	}

	hisi_get_ring_entry(hisi, hw->sq_tail, &desc, &sqe);

	sqe->dw0 = SQE_OPCODE_M2M;
	sqe->dw1 = 0;
	sqe->dw2 = 0;
	sqe->length = len;
	sqe->src_addr = src;
	sqe->dst_addr = dst;

	desc->callback_fn = NULL;
	desc->callback_arg = NULL;

	hisi_submit_single(hw);

	return desc;
}

static int is_hisi_run(struct hisi_dma_dev *hw) {
	uint32_t val = hisi_dma_read_queue(hw, HISI_DMA_QUEUE_FSM_REG);
	return !!(FIELD_GET(HISI_DMA_QUEUE_FSM_STS_M, val) == HISI_DMA_STATE_RUN);
}

static int is_hisi_idle(struct hisi_dma_dev *hw) {
	uint32_t val = hisi_dma_read_queue(hw, HISI_DMA_QUEUE_FSM_REG);
	return !!(FIELD_GET(HISI_DMA_QUEUE_FSM_STS_M, val) == HISI_DMA_STATE_IDLE);
}

static int
hisi_reset_hw(struct spdk_hisi_chan *hisi)
{
	struct hisi_dma_dev *hw = &hisi->hw;
	int timeout;

	SPDK_NOTICELOG("begin hisi_reset_hw\n");
	hisi_dma_update_queue_bit(hw, HISI_DMA_QUEUE_CTRL0_REG,
				  HISI_DMA_QUEUE_CTRL0_PAUSE_B, true);
	hisi_dma_update_queue_bit(hw, HISI_DMA_QUEUE_CTRL0_REG,
				  HISI_DMA_QUEUE_CTRL0_EN_B, false);

	timeout = 10; /* in milliseconds */
	while (is_hisi_run(hw)) {
		spdk_delay_us(100);
		timeout--;
		if (timeout == 0) {
			SPDK_ERRLOG("disable dma timed out\n");
			return -1;
		}
	}

	hisi_dma_update_queue_bit(hw, HISI_DMA_QUEUE_CTRL1_REG,
				  HISI_DMA_QUEUE_CTRL1_RESET_B, true);
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_SQ_TAIL_REG, 0);
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_CQ_HEAD_REG, 0);
	hisi_dma_update_queue_bit(hw, HISI_DMA_QUEUE_CTRL0_REG,
				  HISI_DMA_QUEUE_CTRL0_PAUSE_B, false);

	timeout = 10; /* in milliseconds */
	while (!is_hisi_idle(hw)) {
		spdk_delay_us(100);
		timeout--;
		if (timeout == 0) {
			SPDK_ERRLOG("reset dma timed out\n");
			return -1;
		}
	}

	SPDK_NOTICELOG("end hisi_reset_hw\n");
	return 0;
}

static int
hisi_process_channel_events(struct spdk_hisi_chan *hisi)
{
	struct hisi_dma_dev *hw = &hisi->hw;
	uint16_t sq_head = hw->sq_head;
	uint16_t cpl_num, i;

	volatile struct hisi_dma_cqe *cqe;
	uint16_t csq_head = hw->cq_sq_head;
	uint16_t cq_head = hw->cq_head;
	uint16_t count = 0;
	uint64_t misc;
	struct hisi_descriptor *desc = NULL;

	while (count < hw->cq_depth) {
		cqe = &hw->cqe[cq_head];
		misc = cqe->misc;
		if (FIELD_GET(CQE_VALID_B, misc) != hw->cqe_vld)
			break;

		csq_head = FIELD_GET(CQE_SQ_HEAD_MASK, misc);
		if (spdk_unlikely(csq_head > hw->sq_depth_mask)) {
			/**
			 * Defensive programming to prevent overflow of the
			 * status array indexed by csq_head. Only error logs
			 * are used for prompting.
			 */
			SPDK_ERRLOG("invalid csq_head:%u!\n", csq_head);
			count = 0;
			break;
		}
		if (spdk_unlikely(misc & CQE_STATUS_MASK))
			hw->status[csq_head] = FIELD_GET(CQE_STATUS_MASK,
							 misc);

		count++;
		cq_head++;
		if (cq_head == hw->cq_depth) {
			hw->cqe_vld = !hw->cqe_vld;
			cq_head = 0;
		}
	}

	if (count == 0)
		return 0;

	hw->cq_head = cq_head;
	hw->cq_sq_head = (csq_head + 1) & hw->sq_depth_mask;
	hw->cqs_completed += count;
	if (hw->cqs_completed >= HISI_DMA_CQ_RESERVED) {
		spdk_mmio_write_4(hw->cq_head_reg, hw->cq_head);
		hw->cqs_completed = 0;
	}

	if (hw->cq_sq_head >= hw->sq_head)
		cpl_num = hw->cq_sq_head - hw->sq_head;
	else
		cpl_num = hw->sq_depth_mask + 1 - hw->sq_head + hw->cq_sq_head;

	SPDK_NOTICELOG("cpl_num: %d\n", cpl_num);
	for (i = 0; i < cpl_num; i++) {
		if (hw->status[sq_head]) {
			break;
		}
		desc = &hisi->ring[sq_head];
		if (desc->callback_fn) {
			desc->callback_fn(desc->callback_arg);
		}
		sq_head = (sq_head + 1) & hw->sq_depth_mask;
	}

	if (i > 0) {
		hw->sq_head = sq_head;
		hw->completed += i;
		SPDK_NOTICELOG("completed %ld\n", hw->completed);
	}

	return i;
}

static void
hisi_dma_init_hw(struct hisi_dma_dev *hw)
{
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_SQ_BASE_L_REG,
			     lower_32_bits(hw->sqe_iova));
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_SQ_BASE_H_REG,
			     upper_32_bits(hw->sqe_iova));
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_CQ_BASE_L_REG,
			     lower_32_bits(hw->cqe_iova));
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_CQ_BASE_H_REG,
			     upper_32_bits(hw->cqe_iova));
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_SQ_DEPTH_REG,
			     hw->sq_depth_mask);
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_CQ_DEPTH_REG, hw->cq_depth - 1);
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_SQ_TAIL_REG, 0);
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_CQ_HEAD_REG, 0);
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_ERR_INT_NUM0_REG, 0);
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_ERR_INT_NUM1_REG, 0);
	hisi_dma_write_queue(hw, HISI_DMA_QUEUE_ERR_INT_NUM2_REG, 0);

	if (hw->reg_layout == HISI_DMA_REG_LAYOUT_HIP08) {
		hisi_dma_write_queue(hw, HISI_DMA_HIP08_QUEUE_ERR_INT_NUM3_REG,
				     0);
		hisi_dma_write_queue(hw, HISI_DMA_HIP08_QUEUE_ERR_INT_NUM4_REG,
				     0);
		hisi_dma_write_queue(hw, HISI_DMA_HIP08_QUEUE_ERR_INT_NUM5_REG,
				     0);
		hisi_dma_write_queue(hw, HISI_DMA_HIP08_QUEUE_ERR_INT_NUM6_REG,
				     0);
		hisi_dma_update_queue_bit(hw, HISI_DMA_QUEUE_CTRL0_REG,
				HISI_DMA_HIP08_QUEUE_CTRL0_ERR_ABORT_B, false);
		hisi_dma_update_queue_mbit(hw, HISI_DMA_QUEUE_INT_STATUS_REG,
				HISI_DMA_HIP08_QUEUE_INT_MASK_M, true);
		hisi_dma_update_queue_mbit(hw, HISI_DMA_QUEUE_INT_MASK_REG,
				HISI_DMA_HIP08_QUEUE_INT_MASK_M, true);
	} else if (hw->reg_layout == HISI_DMA_REG_LAYOUT_HIP09) {
		hisi_dma_update_queue_mbit(hw, HISI_DMA_QUEUE_CTRL0_REG,
				HISI_DMA_HIP09_QUEUE_CTRL0_ERR_ABORT_M, false);
		hisi_dma_update_queue_mbit(hw, HISI_DMA_QUEUE_INT_STATUS_REG,
				HISI_DMA_HIP09_QUEUE_INT_MASK_M, true);
		hisi_dma_update_queue_mbit(hw, HISI_DMA_QUEUE_INT_MASK_REG,
				HISI_DMA_HIP09_QUEUE_INT_MASK_M, true);
		hisi_dma_update_queue_mbit(hw,
				HISI_DMA_HIP09_QUEUE_ERR_INT_STATUS_REG,
				HISI_DMA_HIP09_QUEUE_ERR_INT_MASK_M, true);
		hisi_dma_update_queue_mbit(hw,
				HISI_DMA_HIP09_QUEUE_ERR_INT_MASK_REG,
				HISI_DMA_HIP09_QUEUE_ERR_INT_MASK_M, true);
		hisi_dma_update_queue_bit(hw, HISI_DMA_QUEUE_CTRL1_REG,
				HISI_DMA_HIP09_QUEUE_CTRL1_VA_ENABLE_B, true);
		hisi_dma_update_bit(hw,
				HISI_DMA_HIP09_QUEUE_CFG_REG(hw->queue_id),
				HISI_DMA_HIP09_QUEUE_CFG_LINK_DOWN_MASK_B,
				true);
	}
}

static int
hisi_channel_setup(struct spdk_hisi_chan *hisi)
{
	uint8_t revision;
	uint32_t sq_size;
	uint32_t cq_size;
	uint32_t status_size;
	uint32_t total_size;
	struct hisi_dma_dev *hw =&hisi->hw;

	int ret = hisi_dma_check_revision(hisi->device, &revision);
	if (ret)
		return ret;
	SPDK_NOTICELOG("read PCI revision: 0x%x", revision);

	// io_base, bar 2
	if (hisi_map_pci_bar(hisi) != 0) {
		SPDK_ERRLOG("hisi_map_pci_bar() failed\n");
		return -1;
	}

	SPDK_NOTICELOG("hisi_map_pci_bar() done\n");

	if (revision == HISI_DMA_REVISION_HIP08B)
		hisi_dma_update_bit(hw, HISI_DMA_HIP08_MODE_REG, HISI_DMA_HIP08_MODE_SEL_B, true);

	// TODO: support multi queues.
	hw->queue_id = 0;
	hw->revision = revision;
	hw->reg_layout = hisi_dma_reg_layout(revision);
	hw->sq_tail_reg = hisi_dma_queue_regaddr(hw, HISI_DMA_QUEUE_SQ_TAIL_REG);
	hw->cq_head_reg = hisi_dma_queue_regaddr(hw, HISI_DMA_QUEUE_CQ_HEAD_REG);

	ret = hisi_reset_hw(hisi);
	if (ret) {
		SPDK_ERRLOG("hisi_reset_hw() failed\n");
		return ret;
	}

	/* Reset the dmadev to a known state, include:
	 *   1) zero iomem, also include status fields.
	 *   2) init hardware register.
	 *   3) init index values to zero.
	 *   4) init running statistics.
	 */
	hw->ring_size = HISI_DMA_MAX_DESC_NUM /HISI_DMA_MAX_HW_QUEUES;
	sq_size = sizeof(struct hisi_dma_sqe) * hw->ring_size;
	cq_size = sizeof(struct hisi_dma_cqe) * (hw->ring_size + HISI_DMA_CQ_RESERVED);
	status_size = sizeof(uint16_t) * hw->ring_size;

	SPDK_NOTICELOG("ring_size: %u, sq_size: %u, cq_size: %u, status_size: %u\n", hw->ring_size, sq_size, cq_size, status_size);

	sq_size = hisi_cacheline_roundup(sq_size);
	cq_size = hisi_cacheline_roundup(cq_size);
	status_size = hisi_cacheline_roundup(status_size);
	total_size = sq_size + cq_size + status_size;

	hisi->ring = calloc(hw->ring_size, sizeof(struct hisi_descriptor));
	if (!hisi->ring) {
		return -ENOMEM;
	}

	hw->iomz = spdk_zmalloc(total_size, 128, NULL, SPDK_ENV_LCORE_ID_ANY, SPDK_MALLOC_DMA);
	if (!hw->iomz) {
		SPDK_ERRLOG("alloc hisi memory failed");
		return -ENOMEM;
	}

	hw->iomz_sz = total_size;
	hw->iova = (uint64_t)hw->iomz;
	hw->sqe = (struct hisi_dma_sqe *)hw->iomz;
	hw->cqe = (struct hisi_dma_cqe *)((char *)hw->iomz + sq_size);
	hw->status = (void *)((char *)hw->iomz + sq_size + cq_size);
	hw->sqe_iova = hw->iova;
	hw->cqe_iova = hw->iova + sq_size;
	hw->sq_depth_mask = hw->ring_size - 1;
	hw->cq_depth = hw->ring_size + HISI_DMA_CQ_RESERVED;
	memset(hw->iomz, 0, hw->iomz_sz);

	SPDK_NOTICELOG("iova: 0x%lx, sqe_iova: 0x%lx, cqe_iova: 0x%lx, sq_depth_mask: 0x%x, cq_depth: 0x%x, sqe: %p, cqe: %p, status: %p\n",
		       hw->iova, hw->sqe_iova, hw->cqe_iova, hw->sq_depth_mask, hw->cq_depth, hw->sqe, hw->cqe, hw->status);
	hisi_dma_init_hw(hw);
	hw->sq_head = 0;
	hw->sq_tail = 0;
	hw->cq_sq_head = 0;
	hw->cq_head = 0;
	hw->cqs_completed = 0;
	hw->cqe_vld = 1;
	hw->submitted = 0;
	hw->completed = 0;
	hw->errors = 0;
	hw->qfulls = 0;

	hisi_dma_update_queue_bit(hw, HISI_DMA_QUEUE_CTRL0_REG,
				  HISI_DMA_QUEUE_CTRL0_EN_B, true);

	return 0;
}

static void hisi_channel_destruct(struct spdk_hisi_chan *hisi) {
	struct hisi_dma_dev *hw = &hisi->hw;
	hisi_unmap_pci_bar(hisi);

	if (hisi->ring) {
		free(hisi->ring);
		hisi->ring = NULL;
	}

	if (hw->iomz) {
		spdk_free(hw->iomz);
		hw->iomz = NULL;
	}
}

/* Caller must hold g_hisi_driver.lock */
static struct spdk_hisi_chan *
hisi_attach(struct spdk_pci_device *device)
{
	struct spdk_hisi_chan *hisi;
	uint32_t cmd_reg;

	hisi = calloc(1, sizeof(struct spdk_hisi_chan));
	if (hisi == NULL) {
		return NULL;
	}

	/* Enable PCI busmaster. */
	spdk_pci_device_cfg_read32(device, &cmd_reg, 4);
	cmd_reg |= 0x4;
	spdk_pci_device_cfg_write32(device, cmd_reg, 4);

	hisi->device = device;

	if (hisi_channel_setup(hisi) != 0) {
		hisi_channel_destruct(hisi);
		free(hisi);
		return NULL;
	}

	hisi_dma_dump(&hisi->hw);
	return hisi;
}

struct hisi_enum_ctx {
	spdk_hisi_probe_cb probe_cb;
	spdk_hisi_attach_cb attach_cb;
	void *cb_ctx;
};

/* This function must only be called while holding g_hisi_driver.lock */
static int
hisi_enum_cb(void *ctx, struct spdk_pci_device *pci_dev)
{
	struct hisi_enum_ctx *enum_ctx = ctx;
	struct spdk_hisi_chan *hisi;

	/* Verify that this device is not already attached */
	TAILQ_FOREACH(hisi, &g_hisi_driver.attached_chans, tailq) {
		/*
		 * NOTE: This assumes that the PCI abstraction layer will use the same device handle
		 *  across enumerations; we could compare by BDF instead if this is not true.
		 */
		if (pci_dev == hisi->device) {
			return 0;
		}
	}

	if (enum_ctx->probe_cb(enum_ctx->cb_ctx, pci_dev)) {
		/*
		 * Since I/OAT init is relatively quick, just perform the full init during probing.
		 *  If this turns out to be a bottleneck later, this can be changed to work like
		 *  NVMe with a list of devices to initialize in parallel.
		 */
		hisi = hisi_attach(pci_dev);
		if (hisi == NULL) {
			SPDK_ERRLOG("hisi_attach() failed\n");
			return -1;
		}

		TAILQ_INSERT_TAIL(&g_hisi_driver.attached_chans, hisi, tailq);

		enum_ctx->attach_cb(enum_ctx->cb_ctx, pci_dev, hisi);
	}

	return 0;
}

int
spdk_hisi_probe(void *cb_ctx, spdk_hisi_probe_cb probe_cb, spdk_hisi_attach_cb attach_cb)
{
	int rc;
	struct hisi_enum_ctx enum_ctx;

	pthread_mutex_lock(&g_hisi_driver.lock);

	enum_ctx.probe_cb = probe_cb;
	enum_ctx.attach_cb = attach_cb;
	enum_ctx.cb_ctx = cb_ctx;

	rc = spdk_pci_enumerate(spdk_pci_hisi_get_driver(), hisi_enum_cb, &enum_ctx);

	pthread_mutex_unlock(&g_hisi_driver.lock);

	return rc;
}

void
spdk_hisi_detach(struct spdk_hisi_chan *hisi)
{
	struct hisi_driver	*driver = &g_hisi_driver;

	/* hisi should be in the free list (not registered to a thread)
	 * when calling hisi_detach().
	 */
	pthread_mutex_lock(&driver->lock);
	TAILQ_REMOVE(&driver->attached_chans, hisi, tailq);
	pthread_mutex_unlock(&driver->lock);

	hisi_channel_destruct(hisi);
	free(hisi);
}

int
spdk_hisi_submit_copy(struct spdk_hisi_chan *hisi, void *cb_arg, spdk_hisi_req_cb cb_fn,
		     void *dst, const void *src, uint64_t nbytes)
{
	struct hisi_descriptor	*last_desc = NULL;
	uint64_t	remaining, op_size;
	uint64_t	vdst, vsrc;
	uint64_t	pdst_addr, psrc_addr, dst_len, src_len;

	vdst = (uint64_t)dst;
	vsrc = (uint64_t)src;

	remaining = nbytes;
	while (remaining) {
		src_len = dst_len = remaining;

		psrc_addr = spdk_vtophys((void *)vsrc, &src_len);
		if (psrc_addr == SPDK_VTOPHYS_ERROR) {
			return -EINVAL;
		}
		pdst_addr = spdk_vtophys((void *)vdst, &dst_len);
		if (pdst_addr == SPDK_VTOPHYS_ERROR) {
			return -EINVAL;
		}

		op_size = spdk_min(dst_len, src_len);
		op_size = spdk_min(op_size, 4ULL<<20/* 4MB */);
		remaining -= op_size;

		last_desc = hisi_prep_copy(hisi, pdst_addr, psrc_addr, op_size);

		if (remaining == 0 || last_desc == NULL) {
			break;
		}

		vsrc += op_size;
		vdst += op_size;
	}

	if (last_desc) {
		last_desc->callback_fn = cb_fn;
		last_desc->callback_arg = cb_arg;
	} else {
		/*
		 * Ran out of descriptors in the ring - reset head to leave things as they were
		 * in case we managed to fill out any descriptors.
		 */
		return -ENOMEM;
	}

	return 0;
}

int
spdk_hisi_process_events(struct spdk_hisi_chan *hisi)
{
	return hisi_process_channel_events(hisi);
}

SPDK_LOG_REGISTER_COMPONENT(hisi)
