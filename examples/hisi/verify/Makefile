#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2015 Intel Corporation.
#  All rights reserved.
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

APP = hisi_verify

C_SRCS := verify.c

SPDK_LIB_LIST = hisi util

include $(SPDK_ROOT_DIR)/mk/spdk.app.mk
