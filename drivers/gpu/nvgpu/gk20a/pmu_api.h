/*
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __PMU_API_H__
#define __PMU_API_H__

#include <nvgpu/flcnif_cmn.h>
#include "pmuif/gpmuif_pg_rppg.h"

/* FECS mem override command*/

#define PMU_LRF_TEX_LTC_DRAM_CMD_ID_EN_DIS   0

/*!
 * Enable/Disable FECS error feature
 */
struct pmu_cmd_lrf_tex_ltc_dram_en_dis {
	/*Command type must be first*/
	u8  cmd_type;
	/*unit bitmask*/
	u8  en_dis_mask;
};

struct pmu_lrf_tex_ltc_dram_cmd {
	union {
		u8 cmd_type;
		struct pmu_cmd_lrf_tex_ltc_dram_en_dis en_dis;
	};
};

/*  FECS mem override messages*/
#define PMU_LRF_TEX_LTC_DRAM_MSG_ID_EN_DIS    0

struct pmu_msg_lrf_tex_ltc_dram_en_dis {
	/*!
	 * Must be at start
	 */
	u8 msg_type;
	u8 en_fail_mask;
	u8 dis_fail_mask;
	u32 pmu_status;
};

struct pmu_lrf_tex_ltc_dram_msg {
	union {
		u8 msg_type;
		struct pmu_msg_lrf_tex_ltc_dram_en_dis en_dis;
	};
};

#endif /*__PMU_API_H__*/
