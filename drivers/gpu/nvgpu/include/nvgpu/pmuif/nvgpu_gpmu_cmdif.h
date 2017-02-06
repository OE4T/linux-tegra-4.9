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
#ifndef _NVGPUGPMUCMDIF_H_
#define _NVGPUGPMUCMDIF_H_

#include <nvgpu/flcnif_cmn.h>
#include "gpmuif_cmn.h"
#include "gpmuif_pmu.h"
#include "gpmuif_ap.h"
#include "gpmuif_pg.h"
#include "gpmuif_perfmon.h"
#include "gpmuif_acr.h"
#include "gpmuifboardobj.h"
#include "gpmuifclk.h"
#include "gpmuifperf.h"
#include "gpmuifperfvfe.h"
#include "gpmuifpmgr.h"
#include "gpmuifvolt.h"
#include "gpmuiftherm.h"
#include "gpmuifthermsensor.h"
#include "gpmuifseq.h"

struct pmu_cmd {
	struct pmu_hdr hdr;
	union {
		struct pmu_perfmon_cmd perfmon;
		struct pmu_pg_cmd pg;
		struct pmu_zbc_cmd zbc;
		struct pmu_acr_cmd acr;
		struct nv_pmu_boardobj_cmd boardobj;
		struct nv_pmu_perf_cmd perf;
		struct nv_pmu_volt_cmd volt;
		struct nv_pmu_clk_cmd clk;
		struct nv_pmu_pmgr_cmd pmgr;
		struct nv_pmu_therm_cmd therm;
	} cmd;
};

struct pmu_msg {
	struct pmu_hdr hdr;
	union {
		struct pmu_init_msg init;
		struct pmu_perfmon_msg perfmon;
		struct pmu_pg_msg pg;
		struct pmu_rc_msg rc;
		struct pmu_acr_msg acr;
		struct nv_pmu_boardobj_msg boardobj;
		struct nv_pmu_perf_msg perf;
		struct nv_pmu_volt_msg volt;
		struct nv_pmu_clk_msg clk;
		struct nv_pmu_pmgr_msg pmgr;
		struct nv_pmu_therm_msg therm;
	} msg;
};

#define PMU_UNIT_REWIND		(0x00)
#define PMU_UNIT_PG			(0x03)
#define PMU_UNIT_INIT		(0x07)
#define PMU_UNIT_ACR		(0x0A)
#define PMU_UNIT_PERFMON_T18X	(0x11)
#define PMU_UNIT_PERFMON	(0x12)
#define PMU_UNIT_PERF		(0x13)
#define PMU_UNIT_RC			(0x1F)
#define PMU_UNIT_FECS_MEM_OVERRIDE	(0x1E)
#define PMU_UNIT_CLK	(0x0D)
#define PMU_UNIT_THERM	(0x14)
#define PMU_UNIT_PMGR	(0x18)
#define PMU_UNIT_VOLT	(0x0E)

#define PMU_UNIT_END		(0x23)

#define PMU_UNIT_TEST_START	(0xFE)
#define PMU_UNIT_END_SIM	(0xFF)
#define PMU_UNIT_TEST_END	(0xFF)

#define PMU_UNIT_ID_IS_VALID(id)		\
		(((id) < PMU_UNIT_END) || ((id) >= PMU_UNIT_TEST_START))

#endif /* _NVGPUGPMUCMDIF_H_*/
