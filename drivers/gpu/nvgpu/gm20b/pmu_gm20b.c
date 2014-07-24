/*
 * GM20B PMU
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#include "acr_gm20b.h"
#include "pmu_gm20b.h"

/*!
 * Structure/object which single register write need to be done during PG init
 * sequence to set PROD values.
 */
struct pg_init_sequence_list {
	u32 regaddr;
	u32 writeval;
};


/* PROD settings for ELPG sequencing registers*/
static struct pg_init_sequence_list _pginitseq_gm20b[] = {
		{ 0x0010ab10, 0x8180},
		{ 0x0010e118, 0x81818080},
		{ 0x0010e068, 0},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000082},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000082},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000082},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000082},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000082},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000082},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000082},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010ab14, 0x00000000},
		{ 0x0010ab18, 0x00000000},
		{ 0x0010e024, 0x00000000},
		{ 0x0010e028, 0x00000000},
		{ 0x0010e11c, 0x00000000},
		{ 0x0010e120, 0x00000000},
		{ 0x0010ab1c, 0x00010011},
		{ 0x0010e020, 0x001C0011},
		{ 0x0010e124, 0x00030011},
		{ 0x0010ab20, 0xfedcba98},
		{ 0x0010ab24, 0x00000000},
		{ 0x0010e02c, 0xfedcba98},
		{ 0x0010e030, 0x00000000},
		{ 0x0010e128, 0xfedcba98},
		{ 0x0010e12c, 0x00000000},
		{ 0x0010ab28, 0x71111111},
		{ 0x0010ab2c, 0x70000000},
		{ 0x0010e034, 0x71111111},
		{ 0x0010e038, 0x70000000},
		{ 0x0010e130, 0x71111111},
		{ 0x0010e134, 0x70000000},
		{ 0x0010ab30, 0x00000000},
		{ 0x0010ab34, 0x00000001},
		{ 0x00020004, 0x00000000},
		{ 0x0010e138, 0x00000000},
		{ 0x0010e040, 0x00000000},
};

int gm20b_pmu_setup_elpg(struct gk20a *g)
{
	int ret = 0;
	u32 reg_writes;
	u32 index;

	gk20a_dbg_fn("");

	if (g->elpg_enabled) {
		reg_writes = ((sizeof(_pginitseq_gm20b) /
				sizeof((_pginitseq_gm20b)[0])));
		/* Initialize registers with production values*/
		for (index = 0; index < reg_writes; index++) {
			gk20a_writel(g, _pginitseq_gm20b[index].regaddr,
				_pginitseq_gm20b[index].writeval);
		}
	}

	gk20a_dbg_fn("done");
	return ret;
}

void gm20b_init_pmu_ops(struct gpu_ops *gops)
{
#ifdef CONFIG_TEGRA_ACR
	gm20b_init_secure_pmu(gops);
#else
	gk20a_init_pmu_ops(gops);
#endif
	gops->pmu.pmu_setup_elpg = gm20b_pmu_setup_elpg;
}
