/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2022 Intel Corporation.
 *   All rights reserved.
 */

#include "accel_hisi.h"

#include "spdk/rpc.h"
#include "spdk/util.h"
#include "spdk/event.h"

static void
rpc_hisi_scan_accel_module(struct spdk_jsonrpc_request *request,
			   const struct spdk_json_val *params)
{
	if (params != NULL) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INVALID_PARAMS,
						 "hisi_scan_accel_module requires no parameters");
		return;
	}

	SPDK_NOTICELOG("Enabling hisi\n");
	accel_hisi_enable_probe();

	spdk_jsonrpc_send_bool_response(request, true);
}
SPDK_RPC_REGISTER("hisi_scan_accel_module", rpc_hisi_scan_accel_module, SPDK_RPC_STARTUP)
SPDK_RPC_REGISTER_ALIAS_DEPRECATED(hisi_scan_accel_module, hisi_scan_accel_engine)
