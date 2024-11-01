/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2016 Intel Corporation.
 *   All rights reserved.
 */

#include "env_internal.h"

#include "spdk/pci_ids.h"

#define SPDK_HISI_PCI_DEVICE(DEVICE_ID) SPDK_PCI_DEVICE(SPDK_PCI_VID_HUAWEI, DEVICE_ID)
static struct spdk_pci_id hisi_driver_id[] = {
	{SPDK_HISI_PCI_DEVICE(PCI_DEVICE_ID_HUAWEI_HISIDMA)},
	{ .vendor_id = 0, /* sentinel */ },
};

struct spdk_pci_driver *
spdk_pci_hisi_get_driver(void)
{
	return spdk_pci_get_driver("hisi");
}

SPDK_PCI_DRIVER_REGISTER(hisi, hisi_driver_id, SPDK_PCI_DRIVER_NEED_MAPPING);
