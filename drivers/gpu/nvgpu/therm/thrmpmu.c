/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include "gk20a/gk20a.h"
#include "include/bios.h"
#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobjgrp_e32.h"
#include "pmuif/gpmuifboardobj.h"
#include "thrmpmu.h"
#include "pmuif/gpmuiftherm.h"

struct therm_pmucmdhandler_params {
	struct nv_pmu_therm_rpc *prpccall;
	u32 success;
};

static void therm_pmucmdhandler(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	struct therm_pmucmdhandler_params *phandlerparams =
		(struct therm_pmucmdhandler_params *)param;

	if (msg->msg.therm.msg_type != NV_PMU_THERM_MSG_ID_RPC) {
		gk20a_err(dev_from_gk20a(g),
			"unknow msg %x",
			msg->msg.pmgr.msg_type);
		return;
	}

	if (!phandlerparams->prpccall->b_supported)
		gk20a_err(dev_from_gk20a(g),
			"RPC msg %x failed",
			msg->msg.pmgr.msg_type);
	else
		phandlerparams->success = 1;
}

u32 therm_send_pmgr_tables_to_pmu(struct gk20a *g)
{
	u32 status = 0;
	struct boardobjgrp *pboardobjgrp = NULL;

	if (!BOARDOBJGRP_IS_EMPTY(&g->therm_pmu.therm_deviceobjs.super.super)) {
		pboardobjgrp = &g->therm_pmu.therm_deviceobjs.super.super;
		status = pboardobjgrp->pmuinithandle(g, pboardobjgrp);
		if (status) {
			gk20a_err(dev_from_gk20a(g),
				"therm_send_pmgr_tables_to_pmu - therm_device failed %x",
				status);
			goto exit;
		}
	}

	if (!BOARDOBJGRP_IS_EMPTY(
			&g->therm_pmu.therm_channelobjs.super.super)) {
		pboardobjgrp = &g->therm_pmu.therm_channelobjs.super.super;
		status = pboardobjgrp->pmuinithandle(g, pboardobjgrp);
		if (status) {
			gk20a_err(dev_from_gk20a(g),
				"therm_send_pmgr_tables_to_pmu - therm_channel failed %x",
				status);
			goto exit;
		}
	}

exit:
	return status;
}

u32 therm_set_warn_temp_limit(struct gk20a *g)
{
	u32 status;
	u32 seqdesc = 0;
	struct pmu_cmd cmd = { {0} };
	struct pmu_msg msg = { {0} };
	struct pmu_payload payload = { {0} };
	struct nv_pmu_therm_rpc rpccall = {0};
	struct therm_pmucmdhandler_params handlerparams = {0};

	rpccall.function = NV_PMU_THERM_RPC_ID_SLCT_EVENT_TEMP_TH_SET;
	rpccall.params.slct_event_temp_th_set.event_id =
		NV_PMU_THERM_EVENT_THERMAL_1;
	rpccall.params.slct_event_temp_th_set.temp_threshold = g->curr_warn_temp;
	rpccall.b_supported = 0;

	cmd.hdr.unit_id = PMU_UNIT_THERM;
	cmd.hdr.size = ((u32)sizeof(struct nv_pmu_therm_cmd) +
			(u32)sizeof(struct pmu_hdr));
	cmd.cmd.therm.cmd_type = NV_PMU_THERM_CMD_ID_RPC;

	msg.hdr.size = sizeof(struct pmu_msg);

	payload.in.buf = (u8 *)&rpccall;
	payload.in.size = (u32)sizeof(struct nv_pmu_therm_rpc);
	payload.in.fb_size = PMU_CMD_SUBMIT_PAYLOAD_PARAMS_FB_SIZE_UNUSED;
	payload.in.offset = NV_PMU_THERM_CMD_RPC_ALLOC_OFFSET;

	payload.out.buf = (u8 *)&rpccall;
	payload.out.size = (u32)sizeof(struct nv_pmu_therm_rpc);
	payload.out.fb_size = PMU_CMD_SUBMIT_PAYLOAD_PARAMS_FB_SIZE_UNUSED;
	payload.out.offset = NV_PMU_CLK_MSG_RPC_ALLOC_OFFSET;

	/* Setup the handler params to communicate back results.*/
	handlerparams.success = 0;
	handlerparams.prpccall = &rpccall;

	status = gk20a_pmu_cmd_post(g, &cmd, NULL, &payload,
				PMU_COMMAND_QUEUE_LPQ,
				therm_pmucmdhandler,
				(void *)&handlerparams,
				&seqdesc, ~0);
	if (status) {
		gk20a_err(dev_from_gk20a(g),
			"unable to post pmgr cmd for unit %x cmd id %x size %x",
			cmd.hdr.unit_id, cmd.cmd.therm.cmd_type, cmd.hdr.size);
		goto exit;
	}

	pmu_wait_message_cond(&g->pmu,
			gk20a_get_gr_idle_timeout(g),
			&handlerparams.success, 1);

	if (handlerparams.success == 0) {
		gk20a_err(dev_from_gk20a(g), "could not process cmd\n");
		status = -ETIMEDOUT;
		goto exit;
	}

exit:
	return status;
}

