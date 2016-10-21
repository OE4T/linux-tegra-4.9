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
