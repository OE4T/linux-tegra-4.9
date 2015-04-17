/*
 * GP10B PMU
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>	/* for udelay */
#include "gk20a/gk20a.h"
#include "gk20a/pmu_gk20a.h"
#include "gm20b/acr_gm20b.h"
#include "gm20b/pmu_gm20b.h"

#include "pmu_gp10b.h"

/*!
 * Structure/object which single register write need to be done during PG init
 * sequence to set PROD values.
 */
struct pg_init_sequence_list {
	u32 regaddr;
	u32 writeval;
};

/* PROD settings for ELPG sequencing registers*/
static struct pg_init_sequence_list _pginitseq_gp10b[] = {
		{0x0010ab10, 0x0000868B} ,
		{0x0010e118, 0x8590848F} ,
		{0x0010e000, 0} ,
		{0x0010e06c, 0x000000A3} ,
		{0x0010e06c, 0x000000A0} ,
		{0x0010e06c, 0x00000095} ,
		{0x0010e06c, 0x000000A6} ,
		{0x0010e06c, 0x0000008C} ,
		{0x0010e06c, 0x00000080} ,
		{0x0010e06c, 0x00000081} ,
		{0x0010e06c, 0x00000087} ,
		{0x0010e06c, 0x00000088} ,
		{0x0010e06c, 0x0000008D} ,
		{0x0010e06c, 0x000000A00} ,
		{0x0010e06c, 0x000000A01} ,
		{0x0010e06c, 0x000000A02} ,
		{0x0010e06c, 0x000000A03} ,
		{0x0010e06c, 0x000000A04} ,
		{0x0010e06c, 0x000000A05} ,
		{0x0010e06c, 0x000000A06} ,
		{0x0010e06c, 0x000000A07} ,
		{0x0010e06c, 0x000000A08} ,
		{0x0010e06c, 0x000000A09} ,
		{0x0010e06c, 0x000000950} ,
		{0x0010e06c, 0x000000951} ,
		{0x0010e06c, 0x000000952} ,
		{0x0010e06c, 0x000000953} ,
		{0x0010e06c, 0x000000954} ,
		{0x0010e06c, 0x000000955} ,
		{0x0010e06c, 0x000000956} ,
		{0x0010e06c, 0x000000957} ,
		{0x0010ab14, 0x00000000} ,
		{0x0010e024, 0x00000000} ,
		{0x0010e028, 0x00000000} ,
		{0x0010e11c, 0x00000000} ,
		{0x0010ab1c, 0x140B0B55} ,
		{0x0010e020, 0x0E262655} ,
		{0x0010e124, 0x25101055} ,
		{0x0010ab20, 0x89abcdef} ,
		{0x0010ab24, 0x00000000} ,
		{0x0010e02c, 0x89abcdef} ,
		{0x0010e030, 0x00000000} ,
		{0x0010e128, 0x89abcdef} ,
		{0x0010e12c, 0x00000000} ,
		{0x0010ab28, 0x75555555} ,
		{0x0010ab2c, 0x70000000} ,
		{0x0010e034, 0x75555555} ,
		{0x0010e038, 0x70000000} ,
		{0x0010e130, 0x75555555} ,
		{0x0010e134, 0x70000000} ,
		{0x0010ab30, 0x00000000} ,
		{0x0010ab34, 0x00000001} ,
		{0x00020004, 0x00000000} ,
		{0x0010e138, 0x00000000} ,
		{0x0010e040, 0x00000000} ,
		{0x0010e168, 0x00000000} ,
		{0x0010e114, 0x0000A5A4} ,
		{0x0010e110, 0x00000000} ,
		{0x0010e10c, 0x8590848F} ,
		{0x0010e05c, 0x00000000} ,
		{0x0010e044, 0x00000000} ,
		{0x0010a644, 0x0000868B} ,
		{0x0010a648, 0x00000000 } ,
		{0x0010a64c, 0x00829493 } ,
		{0x0010a650, 0x00000000} ,
		{0x0010e000, 0} ,
		{0x0010e068, 0x000000A3} ,
		{0x0010e068, 0x000000A0} ,
		{0x0010e068, 0x00000095} ,
		{0x0010e068, 0x000000A6} ,
		{0x0010e068, 0x0000008C} ,
		{0x0010e068, 0x00000080} ,
		{0x0010e068, 0x00000081} ,
		{0x0010e068, 0x00000087} ,
		{0x0010e068, 0x00000088} ,
		{0x0010e068, 0x0000008D} ,
		{0x0010e068, 0x000000A00} ,
		{0x0010e068, 0x000000A01} ,
		{0x0010e068, 0x000000A02} ,
		{0x0010e068, 0x000000A03} ,
		{0x0010e068, 0x000000A04} ,
		{0x0010e068, 0x000000A05} ,
		{0x0010e068, 0x000000A06} ,
		{0x0010e068, 0x000000A07} ,
		{0x0010e068, 0x000000A08} ,
		{0x0010e068, 0x000000A09} ,
		{0x0010e068, 0x000000950} ,
		{0x0010e068, 0x000000951} ,
		{0x0010e068, 0x000000952} ,
		{0x0010e068, 0x000000953} ,
		{0x0010e068, 0x000000954} ,
		{0x0010e068, 0x000000955} ,
		{0x0010e068, 0x000000956} ,
		{0x0010e068, 0x000000957} ,
		{0x0010e000, 0} ,
		{0x0010e004, 0x0000008E},
};

static int gp10b_pmu_setup_elpg(struct gk20a *g)
{
	int ret = 0;
	u32 reg_writes;
	u32 index;

	gk20a_dbg_fn("");

	if (g->elpg_enabled) {
		reg_writes = ((sizeof(_pginitseq_gp10b) /
				sizeof((_pginitseq_gp10b)[0])));
		/* Initialize registers with production values*/
		for (index = 0; index < reg_writes; index++) {
			gk20a_writel(g, _pginitseq_gp10b[index].regaddr,
				_pginitseq_gp10b[index].writeval);
		}
	}

	gk20a_dbg_fn("done");
	return ret;
}

void gp10b_init_pmu_ops(struct gpu_ops *gops)
{
	if (gops->privsecurity) {
		gm20b_init_secure_pmu(gops);
		gops->pmu.init_wpr_region = gm20b_pmu_init_acr;
	} else {
		gk20a_init_pmu_ops(gops);
		gops->pmu.init_wpr_region = NULL;
	}
	gops->pmu.pmu_setup_elpg = gp10b_pmu_setup_elpg;
	gops->pmu.lspmuwprinitdone = false;
	gops->pmu.fecsbootstrapdone = false;
}
