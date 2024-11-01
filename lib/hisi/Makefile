#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2024 SmartX Corporation.
#  All rights reserved.
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

SO_VER := 5
SO_MINOR := 0

C_SRCS = hisi.c hisi_debug.c
LIBNAME = hisi

SPDK_MAP_FILE = $(abspath $(CURDIR)/spdk_hisi.map)

include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk
