/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2015 Intel Corporation.
 *   All rights reserved.
 */

#ifndef __HISI_INTERNAL_H__
#define __HISI_INTERNAL_H__

#include "spdk/stdinc.h"

#include "spdk/hisi.h"
#include "spdk/hisi_spec.h"
#include "spdk/queue.h"
#include "spdk/mmio.h"

struct hisi_descriptor {
	spdk_hisi_req_cb	callback_fn;
	void			*callback_arg;
};

/* One of these per allocated PCI device. */
struct spdk_hisi_chan {
	/* Opaque handle to upper layer */
	struct spdk_pci_device		*device;
	struct hisi_dma_dev hw;
	struct hisi_descriptor	*ring;

	/* tailq entry for attached_chans */
	TAILQ_ENTRY(spdk_hisi_chan)	tailq;
};

uint32_t hisi_dma_read_dev(struct hisi_dma_dev *hw, uint32_t off);
uint32_t hisi_dma_read_queue(struct hisi_dma_dev *hw, uint32_t qoff);
int hisi_dma_dump(struct hisi_dma_dev *hw);
#endif /* __HISI_INTERNAL_H__ */
