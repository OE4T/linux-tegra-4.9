/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

static void gk20a_falcon_ops(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	struct nvgpu_falcon_version_ops *flcn_vops = &flcn->flcn_vops;

	flcn_ops->reset = NULL;
	flcn_ops->enable_irq = NULL;
	flcn_ops->fbif_transcfg = NULL;
	flcn_ops->read_hwcfg = NULL;
	flcn_ops->write_hwcfg = NULL;
	flcn_ops->copy_from_dmem = NULL;
	flcn_ops->copy_to_dmem = NULL;
	flcn_ops->dma_copy = NULL;
	flcn_ops->mailbox_read = NULL;
	flcn_ops->mailbox_write = NULL;
	flcn_ops->get_unit_status = NULL;
	flcn_ops->dump_falcon_stats = NULL;

	flcn_vops->start_cpu_secure = NULL;
	flcn_vops->write_dmatrfbase = NULL;
}

static void gk20a_falcon_hal_sw_init(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;


	switch (flcn->flcn_id) {
	case FALCON_ID_PMU:
		flcn->flcn_base = FALCON_PWR_BASE;
		break;
	case FALCON_ID_SEC2:
		flcn->flcn_base = FALCON_SEC_BASE;
	break;
	case FALCON_ID_FECS:
		flcn->flcn_base = FALCON_FECS_BASE;
	break;
	case FALCON_ID_GPCCS:
		flcn->flcn_base = FALCON_GPCCS_BASE;
	break;
	default:
		nvgpu_err(g, "Invalid flcn request");
		break;
	}

	nvgpu_mutex_init(&flcn->copy_lock);

	gk20a_falcon_ops(flcn);
}

void gk20a_falcon_init_hal(struct gpu_ops *gops)
{
	gops->falcon.falcon_hal_sw_init = gk20a_falcon_hal_sw_init;
}
