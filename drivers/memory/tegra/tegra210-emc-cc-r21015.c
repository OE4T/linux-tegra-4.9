/*
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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <soc/tegra/tegra_emc.h>

#include "tegra210-emc-reg.h"

#define DVFS_CLOCK_CHANGE_VERSION	21018
#define EMC_PRELOCK_VERSION		2101

static void __update_emc_alt_timing(struct emc_table *current_timing)
{
	struct emc_table *current_table, *alt_timing;
	int i;

	if (!tegra_emc_table_derated)
		return;

	current_table = emc_get_table(dram_over_temp_state);
	i = current_timing - current_table;

	BUG_ON(i < 0 || i > tegra_emc_table_size);

	if (dram_over_temp_state == TEGRA_DRAM_OVER_TEMP_THROTTLE)
		alt_timing = &tegra_emc_table_normal[i];
	else
		alt_timing = &tegra_emc_table_derated[i];

	__emc_copy_table_params(current_timing, alt_timing,
				EMC_COPY_TABLE_PARAM_PERIODIC_FIELDS);
}

static void __reset_dram_clktree_values(struct emc_table *table)
{
	#define __RESET_CLKTREE(TBL, C, D, U)				\
	TBL->current_dram_clktree_c ## C ## d ## D ## u ## U =		\
	TBL->trained_dram_clktree_c ## C ## d ## D ## u ## U

	__RESET_CLKTREE(table, 0, 0, 0);
	__RESET_CLKTREE(table, 0, 0, 1);
	__RESET_CLKTREE(table, 1, 0, 0);
	__RESET_CLKTREE(table, 1, 0, 1);
	__RESET_CLKTREE(table, 1, 1, 0);
	__RESET_CLKTREE(table, 1, 1, 1);
}

static inline u32 actual_osc_clocks(u32 in)
{
	if (in < 0x40)
		return in * 16;
	else if (in < 0x80)
		return 2048;
	else if (in < 0xc0)
		return 4096;
	else
		return 8192;
}

static u32 update_clock_tree_delay(struct emc_table *last_timing,
				   struct emc_table *next_timing,
				   u32 dram_dev_num, u32 channel_mode)
{
	u32 mrr_req = 0, mrr_data = 0;
	u32 temp0_0 = 0, temp0_1 = 0, temp1_0 = 0, temp1_1 = 0;
	s32 tdel = 0, tmdel = 0, adel = 0;
	u32 cval;
	u32 last_timing_rate_mhz = last_timing->rate / 1000;
	u32 next_timing_rate_mhz = next_timing->rate / 1000;

	mrr_req = (2 << EMC_MRR_DEV_SEL_SHIFT) |
		  (19 << EMC_MRR_MA_SHIFT);
	emc_writel(mrr_req, EMC_MRR);

	WARN(wait_for_update(EMC_EMC_STATUS,
			     EMC_EMC_STATUS_MRR_DIVLD, 1, REG_EMC),
	     "Timed out waiting for MRR 19 (ch=0)\n");
	if (channel_mode == DUAL_CHANNEL)
		WARN(wait_for_update(EMC_EMC_STATUS,
				     EMC_EMC_STATUS_MRR_DIVLD, 1, REG_EMC1),
		     "Timed out waiting for MRR 19 (ch=1)\n");

	mrr_data = (emc_readl(EMC_MRR) & EMC_MRR_DATA_MASK) <<
		   EMC_MRR_DATA_SHIFT;

	temp0_0 = (mrr_data & 0xff) << 8;
	temp0_1 = mrr_data & 0xff00;

	if (channel_mode == DUAL_CHANNEL) {
		mrr_data = (emc1_readl(EMC_MRR) & EMC_MRR_DATA_MASK) <<
			   EMC_MRR_DATA_SHIFT;
		temp1_0 = (mrr_data & 0xff) << 8;
		temp1_1 = mrr_data & 0xff00;
	}

	mrr_req = (mrr_req & ~EMC_MRR_MA_MASK) | (18 << EMC_MRR_MA_SHIFT);
	emc_writel(mrr_req, EMC_MRR);

	WARN(wait_for_update(EMC_EMC_STATUS,
			     EMC_EMC_STATUS_MRR_DIVLD, 1, REG_EMC),
	     "Timed out waiting for MRR 18 (ch=0)\n");
	if (channel_mode == DUAL_CHANNEL)
		WARN(wait_for_update(EMC_EMC_STATUS,
				     EMC_EMC_STATUS_MRR_DIVLD, 1, REG_EMC1),
		     "Timed out waiting for MRR 18 (ch=1)\n");

	mrr_data = (emc_readl(EMC_MRR) & EMC_MRR_DATA_MASK) <<
		   EMC_MRR_DATA_SHIFT;

	temp0_0 |= mrr_data & 0xff;
	temp0_1 |= (mrr_data & 0xff00) >> 8;

	if (channel_mode == DUAL_CHANNEL) {
		mrr_data = (emc1_readl(EMC_MRR) & EMC_MRR_DATA_MASK) <<
			   EMC_MRR_DATA_SHIFT;
		temp1_0 |= (mrr_data & 0xff);
		temp1_1 |= (mrr_data & 0xff00) >> 8;
	}

	cval = (1000000 * actual_osc_clocks(last_timing->run_clocks)) /
	       (last_timing_rate_mhz * 2 * temp0_0);
	tdel = next_timing->current_dram_clktree_c0d0u0 - cval;
	tmdel = (tdel < 0) ? -1 * tdel : tdel;
	adel = tmdel;

	if (tmdel * 128 * next_timing_rate_mhz / 1000000 >
	    next_timing->tree_margin)
		next_timing->current_dram_clktree_c0d0u0 = cval;

	cval = (1000000 * actual_osc_clocks(last_timing->run_clocks)) /
	       (last_timing_rate_mhz * 2 * temp0_1);
	tdel = next_timing->current_dram_clktree_c0d0u1 - cval;
	tmdel = (tdel < 0) ? -1 * tdel : tdel;

	if (tmdel > adel)
		adel = tmdel;

	if (tmdel * 128 * next_timing_rate_mhz / 1000000 >
	    next_timing->tree_margin)
		next_timing->current_dram_clktree_c0d0u1 = cval;

	if (channel_mode == DUAL_CHANNEL) {
		cval = (1000000 * actual_osc_clocks(last_timing->run_clocks)) /
		       (last_timing_rate_mhz * 2 * temp1_0);
		tdel = next_timing->current_dram_clktree_c1d0u0 - cval;
		tmdel = (tdel < 0) ? -1 * tdel : tdel;
		if (tmdel > adel)
			adel = tmdel;

		if (tmdel * 128 * next_timing_rate_mhz / 1000000 >
		    next_timing->tree_margin)
			next_timing->current_dram_clktree_c1d0u0 = cval;


		cval = (1000000 * actual_osc_clocks(last_timing->run_clocks)) /
		      (last_timing_rate_mhz * 2 * temp1_1);
		tdel = next_timing->current_dram_clktree_c1d0u1 - cval;
		tmdel = (tdel < 0) ? -1 * tdel : tdel;

		if (tmdel > adel)
			adel = tmdel;

		if (tmdel * 128 * next_timing_rate_mhz / 1000000 >
		    next_timing->tree_margin)
			next_timing->current_dram_clktree_c1d0u1 = cval;

	}

	if (dram_dev_num != TWO_RANK)
		goto done;

	mrr_req = (1 << EMC_MRR_DEV_SEL_SHIFT) |
		  (19 << EMC_MRR_MA_SHIFT);
	emc_writel(mrr_req, EMC_MRR);

	WARN(wait_for_update(EMC_EMC_STATUS,
			     EMC_EMC_STATUS_MRR_DIVLD, 1, REG_EMC),
	     "Timed out waiting for MRR 19 (ch=0)\n");
	if (channel_mode == DUAL_CHANNEL)
		WARN(wait_for_update(EMC_EMC_STATUS,
				     EMC_EMC_STATUS_MRR_DIVLD, 1, REG_EMC1),
		     "Timed out waiting for MRR 19 (ch=1)\n");

	mrr_data = (emc_readl(EMC_MRR) & EMC_MRR_DATA_MASK) <<
		   EMC_MRR_DATA_SHIFT;

	temp0_0 = (mrr_data & 0xff) << 8;
	temp0_1 = mrr_data & 0xff00;

	if (channel_mode == DUAL_CHANNEL) {
		mrr_data = (emc1_readl(EMC_MRR) & EMC_MRR_DATA_MASK) <<
			   EMC_MRR_DATA_SHIFT;
		temp1_0 = (mrr_data & 0xff) << 8;
		temp1_1 = mrr_data & 0xff00;
	}

	mrr_req = (mrr_req & ~EMC_MRR_MA_MASK) | (18 << EMC_MRR_MA_SHIFT);
	emc_writel(mrr_req, EMC_MRR);

	WARN(wait_for_update(EMC_EMC_STATUS,
			     EMC_EMC_STATUS_MRR_DIVLD, 1, REG_EMC),
	     "Timed out waiting for MRR 18 (ch=0)\n");
	if (channel_mode == DUAL_CHANNEL)
		WARN(wait_for_update(EMC_EMC_STATUS,
				     EMC_EMC_STATUS_MRR_DIVLD, 1, REG_EMC1),
		     "Timed out waiting for MRR 18 (ch=1)\n");

	mrr_data = (emc_readl(EMC_MRR) & EMC_MRR_DATA_MASK) <<
		   EMC_MRR_DATA_SHIFT;

	temp0_0 |= mrr_data & 0xff;
	temp0_1 |= (mrr_data & 0xff00) >> 8;

	if (channel_mode == DUAL_CHANNEL) {
		mrr_data = (emc1_readl(EMC_MRR) & EMC_MRR_DATA_MASK) <<
			   EMC_MRR_DATA_SHIFT;
		temp1_0 |= (mrr_data & 0xff);
		temp1_1 |= (mrr_data & 0xff00) >> 8;
	}

	cval = (1000000 * actual_osc_clocks(last_timing->run_clocks)) /
	       (last_timing_rate_mhz * 2 * temp0_0);
	tdel = next_timing->current_dram_clktree_c0d1u0 - cval;
	tmdel = (tdel < 0) ? -1 * tdel : tdel;
	if (tmdel > adel)
		adel = tmdel;

	if (tmdel * 128 * next_timing_rate_mhz / 1000000 >
	    next_timing->tree_margin)
		next_timing->current_dram_clktree_c0d1u0 = cval;

	cval = (1000000 * actual_osc_clocks(last_timing->run_clocks)) /
	       (last_timing_rate_mhz * 2 * temp0_1);
	tdel = next_timing->current_dram_clktree_c0d1u1 - cval;
	tmdel = (tdel < 0) ? -1 * tdel : tdel;
	if (tmdel > adel)
		adel = tmdel;

	if (tmdel * 128 * next_timing_rate_mhz / 1000000 >
	    next_timing->tree_margin)
		next_timing->current_dram_clktree_c0d1u1 = cval;

	if (channel_mode == DUAL_CHANNEL) {
		cval = (1000000 * actual_osc_clocks(last_timing->run_clocks)) /
		       (last_timing_rate_mhz * 2 * temp1_0);
		tdel = next_timing->current_dram_clktree_c1d1u0 - cval;
		tmdel = (tdel < 0) ? -1 * tdel : tdel;
		if (tmdel > adel)
			adel = tmdel;

		if (tmdel * 128 * next_timing_rate_mhz / 1000000 >
		    next_timing->tree_margin)
			next_timing->current_dram_clktree_c1d1u0 = cval;

		cval = (1000000 * actual_osc_clocks(last_timing->run_clocks)) /
		       (last_timing_rate_mhz * 2 * temp1_1);
		tdel = next_timing->current_dram_clktree_c1d1u1 - cval;
		tmdel = (tdel < 0) ? -1 * tdel : tdel;
		if (tmdel > adel)
			adel = tmdel;

		if (tmdel * 128 * next_timing_rate_mhz / 1000000 >
		    next_timing->tree_margin)
			next_timing->current_dram_clktree_c1d1u1 = cval;
	}

done:
	return adel;
}

static void start_periodic_compensation(void)
{
	u32 mpc_req = 0x4b;

	emc_writel(mpc_req, EMC_MPC);
	mpc_req = emc_readl(EMC_MPC);
}

#define EMC0_EMC_CMD_BRLSHFT_0_INDEX	0
#define EMC1_EMC_CMD_BRLSHFT_1_INDEX	1
#define EMC0_EMC_DATA_BRLSHFT_0_INDEX	2
#define EMC1_EMC_DATA_BRLSHFT_0_INDEX	3
#define EMC0_EMC_DATA_BRLSHFT_1_INDEX	4
#define EMC1_EMC_DATA_BRLSHFT_1_INDEX	5
#define EMC0_EMC_QUSE_BRLSHFT_0_INDEX	6
#define EMC1_EMC_QUSE_BRLSHFT_1_INDEX	7
#define EMC0_EMC_QUSE_BRLSHFT_2_INDEX	8
#define EMC1_EMC_QUSE_BRLSHFT_3_INDEX	9


#define TRIM_REG(chan, rank, reg, byte)					\
	((EMC_PMACRO_OB_DDLL_LONG_DQ_RANK ## rank ## _ ## reg ##	\
	  _OB_DDLL_LONG_DQ_RANK ## rank ## _BYTE ## byte ## _MASK &	\
	  next_timing->trim_regs[EMC_PMACRO_OB_DDLL_LONG_DQ_RANK ##	\
				 rank ## _ ## reg ## _INDEX]) >>	\
	 EMC_PMACRO_OB_DDLL_LONG_DQ_RANK ## rank ## _ ## reg ##		\
	 _OB_DDLL_LONG_DQ_RANK ## rank ## _BYTE ## byte ## _SHIFT)	\
	+								\
	(((EMC_DATA_BRLSHFT_ ## rank ## _RANK ## rank ## _BYTE ##	\
	   byte ## _DATA_BRLSHFT_MASK &					\
	   next_timing->trim_perch_regs[EMC ## chan ##			\
			      _EMC_DATA_BRLSHFT_ ## rank ## _INDEX]) >>	\
	  EMC_DATA_BRLSHFT_ ## rank ## _RANK ## rank ## _BYTE ##	\
	  byte ## _DATA_BRLSHFT_SHIFT) * 64)

#define CALC_TEMP(rank, reg, byte1, byte2, n)				\
	((new[n] << EMC_PMACRO_OB_DDLL_LONG_DQ_RANK ## rank ## _ ##	\
	  reg ## _OB_DDLL_LONG_DQ_RANK ## rank ## _BYTE ## byte1 ## _SHIFT) & \
	 EMC_PMACRO_OB_DDLL_LONG_DQ_RANK ## rank ## _ ## reg ##		\
	 _OB_DDLL_LONG_DQ_RANK ## rank ## _BYTE ## byte1 ## _MASK)	\
	|								\
	((new[n + 1] << EMC_PMACRO_OB_DDLL_LONG_DQ_RANK ## rank ## _ ##	\
	  reg ## _OB_DDLL_LONG_DQ_RANK ## rank ## _BYTE ## byte2 ## _SHIFT) & \
	 EMC_PMACRO_OB_DDLL_LONG_DQ_RANK ## rank ## _ ## reg ##		\
	 _OB_DDLL_LONG_DQ_RANK ## rank ## _BYTE ## byte2 ## _MASK)	\

static u32 apply_periodic_compensation_trimmer(
		struct emc_table *next_timing, u32 offset)
{
	u32 i, temp = 0;
	u32 next_timing_rate_mhz = next_timing->rate / 1000;
	s32 tree_delta[4];
	s32 tree_delta_taps[4];
	s32 new[] = {
		TRIM_REG(0, 0, 0, 0),
		TRIM_REG(0, 0, 0, 1),
		TRIM_REG(0, 0, 1, 2),
		TRIM_REG(0, 0, 1, 3),

		TRIM_REG(1, 0, 2, 4),
		TRIM_REG(1, 0, 2, 5),
		TRIM_REG(1, 0, 3, 6),
		TRIM_REG(1, 0, 3, 7),

		TRIM_REG(0, 1, 0, 0),
		TRIM_REG(0, 1, 0, 1),
		TRIM_REG(0, 1, 1, 2),
		TRIM_REG(0, 1, 1, 3),

		TRIM_REG(1, 1, 2, 4),
		TRIM_REG(1, 1, 2, 5),
		TRIM_REG(1, 1, 3, 6),
		TRIM_REG(1, 1, 3, 7)
	};

	switch (offset) {
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0:
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1:
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2:
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3:
	case EMC_DATA_BRLSHFT_0:
		tree_delta[0] = 128 *
				(next_timing->current_dram_clktree_c0d0u0 -
				 next_timing->trained_dram_clktree_c0d0u0);
		tree_delta[1] = 128 *
				(next_timing->current_dram_clktree_c0d0u1 -
				 next_timing->trained_dram_clktree_c0d0u1);
		tree_delta[2] = 128 *
				(next_timing->current_dram_clktree_c1d0u0 -
				 next_timing->trained_dram_clktree_c1d0u0);
		tree_delta[3] = 128 *
				(next_timing->current_dram_clktree_c1d0u1 -
				 next_timing->trained_dram_clktree_c1d0u1);

		tree_delta_taps[0] = (tree_delta[0] *
				     (s32)next_timing_rate_mhz) / 1000000;
		tree_delta_taps[1] = (tree_delta[1] *
				     (s32)next_timing_rate_mhz) / 1000000;
		tree_delta_taps[2] = (tree_delta[2] *
				     (s32)next_timing_rate_mhz) / 1000000;
		tree_delta_taps[3] = (tree_delta[3] *
				     (s32)next_timing_rate_mhz) / 1000000;

		for (i = 0; i < 4; i++) {
			if ((tree_delta_taps[i] > next_timing->tree_margin) ||
			    (tree_delta_taps[i] <
			    (-1 * next_timing->tree_margin))) {
				new[i * 2] = new[i * 2] + tree_delta_taps[i];
				new[i * 2 + 1] = new[i * 2 + 1] +
						 tree_delta_taps[i];
			}
		}

		if (offset == EMC_DATA_BRLSHFT_0) {
			for (i = 0; i < 8; i++)
				new[i] = new[i] / 64;
		} else {
			for (i = 0; i < 8; i++)
				new[i] = new[i] % 64;
		}
		break;

	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0:
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1:
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2:
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3:
	case EMC_DATA_BRLSHFT_1:
		tree_delta[0] = 128 *
				(next_timing->current_dram_clktree_c0d1u0 -
				 next_timing->trained_dram_clktree_c0d1u0);
		tree_delta[1] = 128 *
				(next_timing->current_dram_clktree_c0d1u1 -
				 next_timing->trained_dram_clktree_c0d1u1);
		tree_delta[2] = 128 *
				(next_timing->current_dram_clktree_c1d1u0 -
				 next_timing->trained_dram_clktree_c1d1u0);
		tree_delta[3] = 128 *
				(next_timing->current_dram_clktree_c1d1u1 -
				 next_timing->trained_dram_clktree_c1d1u1);

		tree_delta_taps[0] = (tree_delta[0] *
				     (s32)next_timing_rate_mhz) / 1000000;
		tree_delta_taps[1] = (tree_delta[1] *
				     (s32)next_timing_rate_mhz) / 1000000;
		tree_delta_taps[2] = (tree_delta[2] *
				     (s32)next_timing_rate_mhz) / 1000000;
		tree_delta_taps[3] = (tree_delta[3] *
				     (s32)next_timing_rate_mhz) / 1000000;

		for (i = 0; i < 4; i++) {
			if ((tree_delta_taps[i] > next_timing->tree_margin) ||
			    (tree_delta_taps[i] <
			     (-1 * next_timing->tree_margin))) {
				new[8 + i * 2] = new[8 + i * 2] +
						 tree_delta_taps[i];
				new[8 + i * 2 + 1] = new[8 + i * 2 + 1] +
						     tree_delta_taps[i];
			}
		}

		if (offset == EMC_DATA_BRLSHFT_1) {
			for (i = 0; i < 8; i++)
				new[i + 8] = new[i + 8] / 64;
		} else {
			for (i = 0; i < 8; i++)
				new[i + 8] = new[i + 8] % 64;
		}
		break;
	}

	switch (offset) {
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0:
		temp = CALC_TEMP(0, 0, 0, 1, 0);
		break;
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1:
		temp = CALC_TEMP(0, 1, 2, 3, 2);
		break;
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2:
		temp = CALC_TEMP(0, 2, 4, 5, 4);
		break;
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3:
		temp = CALC_TEMP(0, 3, 6, 7, 6);
		break;
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0:
		temp = CALC_TEMP(1, 0, 0, 1, 8);
		break;
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1:
		temp = CALC_TEMP(1, 1, 2, 3, 10);
		break;
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2:
		temp = CALC_TEMP(1, 2, 4, 5, 12);
		break;
	case EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3:
		temp = CALC_TEMP(1, 3, 6, 7, 14);
		break;
	case EMC_DATA_BRLSHFT_0:
		temp = ((new[0] <<
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE0_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE0_DATA_BRLSHFT_MASK) |
		       ((new[1] <<
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE1_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE1_DATA_BRLSHFT_MASK) |
		       ((new[2] <<
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE2_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE2_DATA_BRLSHFT_MASK) |
		       ((new[3] <<
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE3_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE3_DATA_BRLSHFT_MASK) |
		       ((new[4] <<
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE4_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE4_DATA_BRLSHFT_MASK) |
		       ((new[5] <<
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE5_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE5_DATA_BRLSHFT_MASK) |
		       ((new[6] <<
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE6_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE6_DATA_BRLSHFT_MASK) |
		       ((new[7] <<
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE7_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_0_RANK0_BYTE7_DATA_BRLSHFT_MASK);
		break;
	case EMC_DATA_BRLSHFT_1:
		temp = ((new[8] <<
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE0_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE0_DATA_BRLSHFT_MASK) |
		       ((new[9] <<
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE1_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE1_DATA_BRLSHFT_MASK) |
		       ((new[10] <<
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE2_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE2_DATA_BRLSHFT_MASK) |
		       ((new[11] <<
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE3_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE3_DATA_BRLSHFT_MASK) |
		       ((new[12] <<
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE4_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE4_DATA_BRLSHFT_MASK) |
		       ((new[13] <<
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE5_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE5_DATA_BRLSHFT_MASK) |
		       ((new[14] <<
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE6_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE6_DATA_BRLSHFT_MASK) |
		       ((new[15] <<
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE7_DATA_BRLSHFT_SHIFT) &
			 EMC_DATA_BRLSHFT_1_RANK1_BYTE7_DATA_BRLSHFT_MASK);
		break;
	default:
		break;
	}

	return temp;
}

u32 __do_periodic_emc_compensation_r21015(struct emc_table *current_timing)
{
	u32 dram_dev_num;
	u32 channel_mode;
	u32 emc_cfg, emc_cfg_o;
	u32 emc_dbg_o;
	u32 del, i;
	u32 list[] = {
		EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0,
		EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1,
		EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2,
		EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3,
		EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0,
		EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1,
		EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2,
		EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3,
		EMC_DATA_BRLSHFT_0,
		EMC_DATA_BRLSHFT_1
	};
	u32 items = ARRAY_SIZE(list);

	if (current_timing->periodic_training) {
		emc_dbg_o = emc_readl(EMC_DBG);
		emc_cfg_o = emc_readl(EMC_CFG);
		emc_cfg = emc_cfg_o & ~(EMC_CFG_DYN_SELF_REF |
					EMC_CFG_DRAM_ACPD |
					EMC_CFG_DRAM_CLKSTOP_PD |
					EMC_CFG_DRAM_CLKSTOP_PD);

		channel_mode = !!(current_timing->
				  burst_regs[EMC_FBIO_CFG7_INDEX] &
				 (1 << 2));
		dram_dev_num = 1 + (mc_readl(MC_EMEM_ADR_CFG) & 0x1);

		emc_writel(emc_cfg, EMC_CFG);
		emc_timing_update(channel_mode);

		wait_for_update(EMC_EMC_STATUS,
				EMC_EMC_STATUS_DRAM_IN_POWERDOWN_MASK,
				0, REG_EMC);
		if (channel_mode)
			wait_for_update(EMC_EMC_STATUS,
					EMC_EMC_STATUS_DRAM_IN_POWERDOWN_MASK,
					0, REG_EMC1);

		wait_for_update(EMC_EMC_STATUS,
				EMC_EMC_STATUS_DRAM_IN_SELF_REFRESH_MASK,
				0, REG_EMC);
		if (channel_mode)
			wait_for_update(EMC_EMC_STATUS,
				      EMC_EMC_STATUS_DRAM_IN_SELF_REFRESH_MASK,
				      0, REG_EMC1);

		start_periodic_compensation();

		udelay((actual_osc_clocks(current_timing->run_clocks) * 1000) /
		       current_timing->rate + 1);

		del = update_clock_tree_delay(current_timing, current_timing,
					      dram_dev_num, channel_mode);

		if (current_timing->tree_margin <
		    ((del * 128 * (current_timing->rate / 1000)) / 1000000)) {
			for (i = 0; i < items; i++) {
				u32 tmp = apply_periodic_compensation_trimmer(
						      current_timing, list[i]);
				emc_writel(tmp, list[i]);
			}
		}

		emc_writel(emc_cfg_o, EMC_CFG);

		emc_timing_update(channel_mode);

		__update_emc_alt_timing(current_timing);
	}

	return 0;
}

static u32 do_dvfs_power_ramp_down(u32 clk, int flip_backward,
				   struct emc_table *last_timing,
				   struct emc_table *next_timing)
{
	u32 ramp_down_wait = 0;
	u32 pmacro_cmd_pad;
	u32 pmacro_dq_pad;
	u32 pmacro_rfu1;
	u32 pmacro_cfg5;
	u32 pmacro_common_tx;
	u32 seq_wait;

	if (flip_backward) {
		pmacro_cmd_pad   = next_timing->
			burst_regs[EMC_PMACRO_CMD_PAD_TX_CTRL_INDEX];
		pmacro_dq_pad    = next_timing->
			burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];
		pmacro_rfu1      = next_timing->
			burst_regs[EMC_PMACRO_BRICK_CTRL_RFU1_INDEX];
		pmacro_cfg5      = next_timing->
			burst_regs[EMC_FBIO_CFG5_INDEX];
		pmacro_common_tx = next_timing->
			burst_regs[EMC_PMACRO_COMMON_PAD_TX_CTRL_INDEX];
	} else {
		pmacro_cmd_pad   = last_timing->
			burst_regs[EMC_PMACRO_CMD_PAD_TX_CTRL_INDEX];
		pmacro_dq_pad    = last_timing->
			burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];
		pmacro_rfu1      = last_timing->
			burst_regs[EMC_PMACRO_BRICK_CTRL_RFU1_INDEX];
		pmacro_cfg5      = last_timing->
			burst_regs[EMC_FBIO_CFG5_INDEX];
		pmacro_common_tx = last_timing->
			burst_regs[EMC_PMACRO_COMMON_PAD_TX_CTRL_INDEX];
	}

	pmacro_cmd_pad |= EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQ_TX_DRVFORCEON;

	ccfifo_writel(pmacro_cmd_pad, EMC_PMACRO_CMD_PAD_TX_CTRL, 0);
	ccfifo_writel(pmacro_cfg5 | EMC_FBIO_CFG5_CMD_TX_DIS, EMC_FBIO_CFG5,
		      12);
	ramp_down_wait = 12 * clk;

	seq_wait = (100000 / clk) + 1;

	if (clk < (1000000 / DVFS_FGCG_HIGH_SPEED_THRESHOLD)) {
		if (clk < (1000000 / IOBRICK_DCC_THRESHOLD)) {
			pmacro_cmd_pad &=
				~(EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQ_TX_E_DCC |
				  EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_CMD_TX_E_DCC);
			pmacro_cmd_pad |=
				EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSP_TX_E_DCC |
				EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSN_TX_E_DCC;
			ccfifo_writel(pmacro_cmd_pad,
				      EMC_PMACRO_CMD_PAD_TX_CTRL, seq_wait);
			ramp_down_wait += 100000;

			pmacro_dq_pad &=
			      ~(EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQ_TX_E_DCC |
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_CMD_TX_E_DCC);
			pmacro_dq_pad |=
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSP_TX_E_DCC |
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSN_TX_E_DCC;
			ccfifo_writel(pmacro_dq_pad,
				      EMC_PMACRO_DATA_PAD_TX_CTRL, 0);
			ccfifo_writel(pmacro_rfu1 & ~0x01120112,
				      EMC_PMACRO_BRICK_CTRL_RFU1, 0);
		} else {
			ccfifo_writel(pmacro_rfu1 & ~0x01120112,
				      EMC_PMACRO_BRICK_CTRL_RFU1, seq_wait);
			ramp_down_wait += 100000;
		}

		ccfifo_writel(pmacro_rfu1 & ~0x01bf01bf,
			      EMC_PMACRO_BRICK_CTRL_RFU1, seq_wait);
		ramp_down_wait += 100000;

		if (clk < (1000000 / IOBRICK_DCC_THRESHOLD)) {
			pmacro_cmd_pad &=
				~(EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQ_TX_E_DCC |
				  EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_CMD_TX_E_DCC |
				  EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSP_TX_E_DCC |
				  EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSN_TX_E_DCC);
			ccfifo_writel(pmacro_cmd_pad,
				      EMC_PMACRO_CMD_PAD_TX_CTRL, seq_wait);
			ramp_down_wait += 100000;

			pmacro_dq_pad &=
			      ~(EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQ_TX_E_DCC |
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_CMD_TX_E_DCC |
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSP_TX_E_DCC |
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSN_TX_E_DCC);
			ccfifo_writel(pmacro_dq_pad,
				      EMC_PMACRO_DATA_PAD_TX_CTRL, 0);
			ccfifo_writel(pmacro_rfu1 & ~0x07ff07ff,
				      EMC_PMACRO_BRICK_CTRL_RFU1, 0);
		} else {
			ccfifo_writel(pmacro_rfu1 & ~0x07ff07ff,
				      EMC_PMACRO_BRICK_CTRL_RFU1, seq_wait);
			ramp_down_wait += 100000;
		}
	} else {
		ccfifo_writel(pmacro_rfu1 & ~0xffff07ff,
			      EMC_PMACRO_BRICK_CTRL_RFU1, seq_wait + 19);
		ramp_down_wait += 100000 + (20 * clk);
	}

	if (clk < (1000000 / DVFS_FGCG_MID_SPEED_THRESHOLD)) {
		ramp_down_wait += 100000;
		ccfifo_writel(pmacro_common_tx & ~0x5,
			      EMC_PMACRO_COMMON_PAD_TX_CTRL, seq_wait);
		ramp_down_wait += 100000;
		ccfifo_writel(pmacro_common_tx & ~0xf,
			      EMC_PMACRO_COMMON_PAD_TX_CTRL, seq_wait);
		ramp_down_wait += 100000;
		ccfifo_writel(0, 0, seq_wait);
		ramp_down_wait += 100000;
	} else {
		ccfifo_writel(pmacro_common_tx & ~0xf,
			      EMC_PMACRO_COMMON_PAD_TX_CTRL, seq_wait);
	}

	return ramp_down_wait;
}

noinline u32 do_dvfs_power_ramp_up(u32 clk, int flip_backward,
				   struct emc_table *last_timing,
				   struct emc_table *next_timing)
{
	u32 pmacro_cmd_pad;
	u32 pmacro_dq_pad;
	u32 pmacro_rfu1;
	u32 pmacro_cfg5;
	u32 pmacro_common_tx;
	u32 ramp_up_wait = 0;

	if (flip_backward) {
		pmacro_cmd_pad   = last_timing->
			burst_regs[EMC_PMACRO_CMD_PAD_TX_CTRL_INDEX];
		pmacro_dq_pad    = last_timing->
			burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];
		pmacro_rfu1      = last_timing->
			burst_regs[EMC_PMACRO_BRICK_CTRL_RFU1_INDEX];
		pmacro_cfg5      = last_timing->burst_regs[EMC_FBIO_CFG5_INDEX];
		pmacro_common_tx = last_timing->
			burst_regs[EMC_PMACRO_COMMON_PAD_TX_CTRL_INDEX];
	} else {
		pmacro_cmd_pad   = next_timing->
			burst_regs[EMC_PMACRO_CMD_PAD_TX_CTRL_INDEX];
		pmacro_dq_pad    = next_timing->
			burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];
		pmacro_rfu1      = next_timing->
			burst_regs[EMC_PMACRO_BRICK_CTRL_RFU1_INDEX];
		pmacro_cfg5      = next_timing->burst_regs[EMC_FBIO_CFG5_INDEX];
		pmacro_common_tx = next_timing->
			burst_regs[EMC_PMACRO_COMMON_PAD_TX_CTRL_INDEX];
	}
	pmacro_cmd_pad |= EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQ_TX_DRVFORCEON;

	if (clk < 1000000 / DVFS_FGCG_MID_SPEED_THRESHOLD) {
		ccfifo_writel(pmacro_common_tx & 0xa,
			      EMC_PMACRO_COMMON_PAD_TX_CTRL, 0);
		ccfifo_writel(pmacro_common_tx & 0xf,
			      EMC_PMACRO_COMMON_PAD_TX_CTRL,
			      (100000 / clk) + 1);
		ramp_up_wait += 100000;
	} else {
		ccfifo_writel(pmacro_common_tx | 0x8,
			      EMC_PMACRO_COMMON_PAD_TX_CTRL, 0);
	}

	if (clk < 1000000 / DVFS_FGCG_HIGH_SPEED_THRESHOLD) {
		if (clk < 1000000 / IOBRICK_DCC_THRESHOLD) {
			pmacro_cmd_pad |=
				EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSP_TX_E_DCC |
				EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSN_TX_E_DCC;
			pmacro_cmd_pad &=
				~(EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQ_TX_E_DCC |
				  EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_CMD_TX_E_DCC);
			ccfifo_writel(pmacro_cmd_pad,
				      EMC_PMACRO_CMD_PAD_TX_CTRL,
				      (100000 / clk) + 1);
			ramp_up_wait += 100000;

			pmacro_dq_pad |=
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSP_TX_E_DCC |
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSN_TX_E_DCC;
			pmacro_dq_pad &=
			       ~(EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQ_TX_E_DCC |
				 EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_CMD_TX_E_DCC);
			ccfifo_writel(pmacro_dq_pad,
				      EMC_PMACRO_DATA_PAD_TX_CTRL, 0);
			ccfifo_writel(pmacro_rfu1 & 0xfe40fe40,
				      EMC_PMACRO_BRICK_CTRL_RFU1, 0);
		} else {
			ccfifo_writel(pmacro_rfu1 & 0xfe40fe40,
				      EMC_PMACRO_BRICK_CTRL_RFU1,
				      (100000 / clk) + 1);
			ramp_up_wait += 100000;
		}

		ccfifo_writel(pmacro_rfu1 & 0xfeedfeed,
			      EMC_PMACRO_BRICK_CTRL_RFU1, (100000 / clk) + 1);
		ramp_up_wait += 100000;

		if (clk < 1000000 / IOBRICK_DCC_THRESHOLD) {
			pmacro_cmd_pad |=
				EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSP_TX_E_DCC |
				EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSN_TX_E_DCC |
				EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQ_TX_E_DCC |
				EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_CMD_TX_E_DCC;
			ccfifo_writel(pmacro_cmd_pad,
				      EMC_PMACRO_CMD_PAD_TX_CTRL,
				      (100000 / clk) + 1);
			ramp_up_wait += 100000;

			pmacro_dq_pad |=
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSP_TX_E_DCC |
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSN_TX_E_DCC |
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQ_TX_E_DCC |
				EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_CMD_TX_E_DCC;
			ccfifo_writel(pmacro_dq_pad,
				      EMC_PMACRO_DATA_PAD_TX_CTRL, 0);
			ccfifo_writel(pmacro_rfu1,
				      EMC_PMACRO_BRICK_CTRL_RFU1, 0);
		} else {
			ccfifo_writel(pmacro_rfu1,
				      EMC_PMACRO_BRICK_CTRL_RFU1,
				      (100000 / clk) + 1);
			ramp_up_wait += 100000;
		}

		ccfifo_writel(pmacro_cfg5 & ~EMC_FBIO_CFG5_CMD_TX_DIS,
			      EMC_FBIO_CFG5, (100000 / clk) + 10);
		ramp_up_wait += 100000 + (10 * clk);
	} else if (clk < 1000000 / DVFS_FGCG_MID_SPEED_THRESHOLD) {
		ccfifo_writel(pmacro_rfu1 | 0x06000600,
			      EMC_PMACRO_BRICK_CTRL_RFU1, (100000 / clk) + 1);
		ccfifo_writel(pmacro_cfg5 & ~EMC_FBIO_CFG5_CMD_TX_DIS,
			      EMC_FBIO_CFG5, (100000 / clk) + 10);
		ramp_up_wait += 100000 + 10 * clk;
	} else {
		ccfifo_writel(pmacro_rfu1 | 0x00000600,
			      EMC_PMACRO_BRICK_CTRL_RFU1, 0);
		ccfifo_writel(pmacro_cfg5 & ~EMC_FBIO_CFG5_CMD_TX_DIS,
			      EMC_FBIO_CFG5, 12);
		ramp_up_wait += 12 * clk;
	}

	pmacro_cmd_pad &= ~EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQ_TX_DRVFORCEON;
	ccfifo_writel(pmacro_cmd_pad, EMC_PMACRO_CMD_PAD_TX_CTRL, 5);

	return ramp_up_wait;
}

static void change_dll_src(struct emc_table *next_timing,
			     u32 clksrc)
{
	u32 out_enb_x;
	u32 dll_setting = next_timing->dll_clk_src;
	u32 emc_clk_src;
	u32 emc_clk_div;

	out_enb_x = 0;
	emc_clk_src = (clksrc & EMC_CLK_EMC_2X_CLK_SRC_MASK) >>
		       EMC_CLK_EMC_2X_CLK_SRC_SHIFT;
	emc_clk_div = (clksrc & EMC_CLK_EMC_2X_CLK_DIVISOR_MASK) >>
		       EMC_CLK_EMC_2X_CLK_DIVISOR_SHIFT;

	dll_setting &= ~(DLL_CLK_EMC_DLL_CLK_SRC_MASK |
			 DLL_CLK_EMC_DLL_CLK_DIVISOR_MASK);
	dll_setting |= emc_clk_src << DLL_CLK_EMC_DLL_CLK_SRC_SHIFT;
	dll_setting |= emc_clk_div << DLL_CLK_EMC_DLL_CLK_DIVISOR_SHIFT;

	dll_setting &= ~DLL_CLK_EMC_DLL_DDLL_CLK_SEL_MASK;
	if (emc_clk_src == EMC_CLK_SOURCE_PLLMB_LJ)
		dll_setting |= (PLLM_VCOB <<
				DLL_CLK_EMC_DLL_DDLL_CLK_SEL_SHIFT);
	else if (emc_clk_src == EMC_CLK_SOURCE_PLLM_LJ)
		dll_setting |= (PLLM_VCOA <<
				DLL_CLK_EMC_DLL_DDLL_CLK_SEL_SHIFT);
	else
		dll_setting |= (EMC_DLL_SWITCH_OUT <<
				DLL_CLK_EMC_DLL_DDLL_CLK_SEL_SHIFT);

	writel(dll_setting, clk_base + CLK_RST_CONTROLLER_CLK_SOURCE_EMC_DLL);

	if (next_timing->clk_out_enb_x_0_clk_enb_emc_dll) {
		writel(CLK_OUT_ENB_X_CLK_ENB_EMC_DLL,
		       clk_base + CLK_RST_CONTROLLER_CLK_OUT_ENB_X_SET);
	} else {
		writel(CLK_OUT_ENB_X_CLK_ENB_EMC_DLL,
		       clk_base + CLK_RST_CONTROLLER_CLK_OUT_ENB_X_CLR);
	}
}

static u32 dll_prelock(struct emc_table *next_timing,
			 int dvfs_with_training, u32 clksrc)
{
	u32 emc_dig_dll_status;
	u32 dll_locked;
	u32 dll_out;
	u32 emc_cfg_dig_dll;
	u32 emc_dll_cfg_0;
	u32 emc_dll_cfg_1;
	u32 ddllcal_ctrl_start_trim_val;
	u32 dll_en;
	u32 dual_channel_lpddr4_case;
	u32 dll_priv_updated;

	dual_channel_lpddr4_case =
		      !!(emc_readl(EMC_FBIO_CFG7) & EMC_FBIO_CFG7_CH1_ENABLE) &
		      !!(emc_readl(EMC_FBIO_CFG7) & EMC_FBIO_CFG7_CH0_ENABLE);

	emc_dig_dll_status = 0;
	dll_locked = 0;
	dll_out = 0;
	emc_cfg_dig_dll = 0;
	emc_dll_cfg_0 = 0;
	emc_dll_cfg_1 = 0;
	ddllcal_ctrl_start_trim_val = 0;
	dll_en = 0;

	emc_cfg_dig_dll = emc_readl(EMC_CFG_DIG_DLL) &
			  ~EMC_CFG_DIG_DLL_CFG_DLL_LOCK_LIMIT_MASK;
	emc_cfg_dig_dll |= (3 << EMC_CFG_DIG_DLL_CFG_DLL_LOCK_LIMIT_SHIFT);
	emc_cfg_dig_dll &= ~EMC_CFG_DIG_DLL_CFG_DLL_EN;
	emc_cfg_dig_dll &= ~EMC_CFG_DIG_DLL_CFG_DLL_MODE_MASK;
	emc_cfg_dig_dll |= (3 << EMC_CFG_DIG_DLL_CFG_DLL_MODE_SHIFT);
	emc_cfg_dig_dll |= EMC_CFG_DIG_DLL_CFG_DLL_STALL_ALL_TRAFFIC;
	emc_cfg_dig_dll &= ~EMC_CFG_DIG_DLL_CFG_DLL_STALL_RW_UNTIL_LOCK;
	emc_cfg_dig_dll &= ~EMC_CFG_DIG_DLL_CFG_DLL_STALL_ALL_UNTIL_LOCK;

	emc_writel(emc_cfg_dig_dll, EMC_CFG_DIG_DLL);
	emc_writel(1, EMC_TIMING_CONTROL);

	wait_for_update(EMC_EMC_STATUS,
			EMC_EMC_STATUS_TIMING_UPDATE_STALLED, 0, REG_EMC);
	if (dual_channel_lpddr4_case)
		wait_for_update(EMC_EMC_STATUS,
				EMC_EMC_STATUS_TIMING_UPDATE_STALLED,
				0, REG_EMC1);

	do {
		emc_cfg_dig_dll = emc_readl(EMC_CFG_DIG_DLL);
		dll_en = emc_cfg_dig_dll & EMC_CFG_DIG_DLL_CFG_DLL_EN;
	} while (dll_en == 1);

	if (dual_channel_lpddr4_case) {
		do {
			emc_cfg_dig_dll = emc1_readl(EMC_CFG_DIG_DLL);
			dll_en = emc_cfg_dig_dll & EMC_CFG_DIG_DLL_CFG_DLL_EN;
		} while (dll_en == 1);
	}

	emc_dll_cfg_0 = next_timing->burst_regs[EMC_DLL_CFG_0_INDEX];

	emc_writel(emc_dll_cfg_0, EMC_DLL_CFG_0);

	if (next_timing->rate >= 400000 && next_timing->rate < 600000)
		ddllcal_ctrl_start_trim_val = 150;
	else if (next_timing->rate >= 600000 && next_timing->rate < 800000)
		ddllcal_ctrl_start_trim_val = 100;
	else if (next_timing->rate >= 800000 && next_timing->rate < 1000000)
		ddllcal_ctrl_start_trim_val = 70;
	else if (next_timing->rate >= 1000000 && next_timing->rate < 1200000)
		ddllcal_ctrl_start_trim_val = 30;
	else
		ddllcal_ctrl_start_trim_val = 20;

	emc_dll_cfg_1 = emc_readl(EMC_DLL_CFG_1);
	emc_dll_cfg_1 &= EMC_DLL_CFG_1_DDLLCAL_CTRL_START_TRIM_MASK;
	emc_dll_cfg_1 |= ddllcal_ctrl_start_trim_val;
	emc_writel(emc_dll_cfg_1, EMC_DLL_CFG_1);

	change_dll_src(next_timing, clksrc);

	emc_cfg_dig_dll = emc_readl(EMC_CFG_DIG_DLL);
	emc_cfg_dig_dll |= EMC_CFG_DIG_DLL_CFG_DLL_EN;
	emc_writel(emc_cfg_dig_dll, EMC_CFG_DIG_DLL);

	emc_timing_update(dual_channel_lpddr4_case ?
			  DUAL_CHANNEL : SINGLE_CHANNEL);

	do {
		emc_cfg_dig_dll = emc_readl(EMC_CFG_DIG_DLL);
		dll_en = emc_cfg_dig_dll & EMC_CFG_DIG_DLL_CFG_DLL_EN;
	} while (dll_en == 0);

	if (dual_channel_lpddr4_case) {
		do {
			emc_cfg_dig_dll = emc1_readl(EMC_CFG_DIG_DLL);
			dll_en = emc_cfg_dig_dll & EMC_CFG_DIG_DLL_CFG_DLL_EN;
		} while (dll_en == 0);
	}

	do {
		emc_dig_dll_status = emc_readl(EMC_DIG_DLL_STATUS);
		dll_locked = emc_dig_dll_status & EMC_DIG_DLL_STATUS_DLL_LOCK;
		dll_priv_updated = emc_dig_dll_status &
				   EMC_DIG_DLL_STATUS_DLL_PRIV_UPDATED;
	} while (!dll_locked || !dll_priv_updated);

	emc_dig_dll_status = emc_readl(EMC_DIG_DLL_STATUS);
	return emc_dig_dll_status & EMC_DIG_DLL_STATUS_DLL_OUT_MASK;
}

static void dll_disable(int channel_mode)
{
	u32 emc_cfg_dig_dll;

	emc_cfg_dig_dll = emc_readl(EMC_CFG_DIG_DLL);
	emc_cfg_dig_dll &= ~EMC_CFG_DIG_DLL_CFG_DLL_EN;
	emc_writel(emc_cfg_dig_dll, EMC_CFG_DIG_DLL);
	emc_timing_update(channel_mode);

	wait_for_update(EMC_CFG_DIG_DLL, EMC_CFG_DIG_DLL_CFG_DLL_EN,
			0, REG_EMC);
	if (channel_mode == DUAL_CHANNEL)
		wait_for_update(EMC_CFG_DIG_DLL,
				EMC_CFG_DIG_DLL_CFG_DLL_EN, 0, REG_EMC1);
}

void emc_set_clock_r21015(struct emc_table *next_timing,
			  struct emc_table *last_timing,
			  int training, u32 clksrc)
{
	struct emc_table *fake_timing;

	u32 i, tmp;

	u32 cya_allow_ref_cc = 0, ref_b4_sref_en = 0, cya_issue_pc_ref = 0;

	u32 zqcal_before_cc_cutoff = 2400;
	u32 ref_delay_mult;
	u32 ref_delay;
	s32 zq_latch_dvfs_wait_time;
	s32 t_zqcal_lpddr4_fc_adj;
	u32 t_fc_lpddr4 = 1000 * next_timing->dram_timings[T_FC_LPDDR4];
	u32 t_zqcal_lpddr4 = 1000000;

	u32 dram_type, dram_dev_num, shared_zq_resistor;
	u32 channel_mode;
	u32 is_lpddr3;

	u32 emc_cfg, emc_sel_dpd_ctrl, emc_cfg_reg;

	u32 emc_dbg;
	u32 emc_zcal_interval;
	u32 emc_zcal_wait_cnt_old;
	u32 emc_zcal_wait_cnt_new;
	u32 emc_dbg_active;
	u32 zq_op;
	u32 zcal_wait_time_clocks;
	u32 zcal_wait_time_ps;

	u32 emc_auto_cal_config;
	u32 auto_cal_en;

	u32 mr13_catr_enable;

	u32 ramp_up_wait = 0, ramp_down_wait = 0;

	u32 source_clock_period;
	u32 destination_clock_period;

	u32 emc_dbg_o;
	u32 emc_cfg_pipe_clk_o;
	u32 emc_pin_o;

	u32 mr13_flip_fspwr;
	u32 mr13_flip_fspop;

	u32 opt_zcal_en_cc;
	u32 opt_do_sw_qrst = 1;
	u32 opt_dvfs_mode;
	u32 opt_dll_mode;
	u32 opt_cc_short_zcal = 1;
	u32 opt_short_zcal = 1;
	u32 save_restore_clkstop_pd = 1;

	u32 prelock_dll_en = 0, dll_out;

	int next_push, next_dq_e_ivref, next_dqs_e_ivref;

	u32 opt_war_200024907;
	u32 zq_wait_long;
	u32 zq_wait_short;

	u32 bg_regulator_switch_complete_wait_clks;
	u32 bg_regulator_mode_change;
	u32 enable_bglp_regulator;
	u32 enable_bg_regulator;

	u32 t_rtm;
	u32 rp_war;
	u32 r2p_war;
	u32 trpab_war;
	s32 n_rtp;
	u32 delta_twatm;
	u32 w2p_war;
	u32 t_rpst;

	u32 mrw_req;
	u32 adel = 0, compensate_trimmer_applicable = 0;
	u32 next_timing_rate_mhz = next_timing->rate / 1000;

	static u32 fsp_for_next_freq;

	fake_timing = get_timing_from_freq(last_timing->rate);

	fsp_for_next_freq = !fsp_for_next_freq;

	dram_type = emc_readl(EMC_FBIO_CFG5) &
		    EMC_FBIO_CFG5_DRAM_TYPE_MASK >>
		    EMC_FBIO_CFG5_DRAM_TYPE_SHIFT;
	shared_zq_resistor = last_timing->burst_regs[EMC_ZCAL_WAIT_CNT_INDEX] &
			     1 << 31;
	channel_mode = !!(last_timing->burst_regs[EMC_FBIO_CFG7_INDEX] &
			  1 << 2);
	opt_zcal_en_cc = (next_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX] &&
			  !last_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX]) ||
			  dram_type == DRAM_TYPE_LPDDR4;
	opt_dll_mode = (dram_type == DRAM_TYPE_DDR3) ?
		       get_dll_state(next_timing) : DLL_OFF;
	is_lpddr3 = (dram_type == DRAM_TYPE_LPDDR2) &&
		    next_timing->burst_regs[EMC_FBIO_CFG5_INDEX] & 1 << 25;
	opt_war_200024907 = (dram_type == DRAM_TYPE_LPDDR4);
	opt_dvfs_mode = MAN_SR;
	dram_dev_num = (mc_readl(MC_EMEM_ADR_CFG) & 0x1) + 1;

	emc_cfg_reg = emc_readl(EMC_CFG);
	emc_auto_cal_config = emc_readl(EMC_AUTO_CAL_CONFIG);

	source_clock_period = 1000000000 / last_timing->rate;
	destination_clock_period = 1000000000 / next_timing->rate;

	t_zqcal_lpddr4_fc_adj = (source_clock_period > zqcal_before_cc_cutoff) ?
				t_zqcal_lpddr4 / destination_clock_period :
				(t_zqcal_lpddr4 - t_fc_lpddr4) /
				destination_clock_period;
	emc_dbg_o = emc_readl(EMC_DBG);
	emc_pin_o = emc_readl(EMC_PIN);
	emc_cfg_pipe_clk_o = emc_readl(EMC_CFG_PIPE_CLK);
	emc_dbg = emc_dbg_o;

	emc_cfg = next_timing->burst_regs[EMC_CFG_INDEX];
	emc_cfg &= ~(EMC_CFG_DYN_SELF_REF | EMC_CFG_DRAM_ACPD |
		     EMC_CFG_DRAM_CLKSTOP_SR | EMC_CFG_DRAM_CLKSTOP_PD);
	emc_sel_dpd_ctrl = next_timing->emc_sel_dpd_ctrl;
	emc_sel_dpd_ctrl &= ~(EMC_SEL_DPD_CTRL_CLK_SEL_DPD_EN |
			      EMC_SEL_DPD_CTRL_CA_SEL_DPD_EN |
			      EMC_SEL_DPD_CTRL_RESET_SEL_DPD_EN |
			      EMC_SEL_DPD_CTRL_ODT_SEL_DPD_EN |
			      EMC_SEL_DPD_CTRL_DATA_SEL_DPD_EN);

	tmp = emc_readl(EMC_CFG_DIG_DLL);
	tmp &= ~EMC_CFG_DIG_DLL_CFG_DLL_EN;
	emc_writel(tmp, EMC_CFG_DIG_DLL);

	emc_timing_update(channel_mode);
	wait_for_update(EMC_CFG_DIG_DLL,
			EMC_CFG_DIG_DLL_CFG_DLL_EN, 0, REG_EMC);
	if (channel_mode)
		wait_for_update(EMC_CFG_DIG_DLL,
				EMC_CFG_DIG_DLL_CFG_DLL_EN, 0, REG_EMC1);

	emc_auto_cal_config = next_timing->emc_auto_cal_config;
	auto_cal_en = emc_auto_cal_config & EMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
	emc_auto_cal_config &= ~EMC_AUTO_CAL_CONFIG_AUTO_CAL_START;
	emc_auto_cal_config |=  EMC_AUTO_CAL_CONFIG_AUTO_CAL_MEASURE_STALL;
	emc_auto_cal_config |=  EMC_AUTO_CAL_CONFIG_AUTO_CAL_UPDATE_STALL;
	emc_auto_cal_config |=  auto_cal_en;
	emc_writel(emc_auto_cal_config, EMC_AUTO_CAL_CONFIG);
	emc_readl(EMC_AUTO_CAL_CONFIG);

	emc_set_shadow_bypass(ACTIVE);
	emc_writel(emc_cfg, EMC_CFG);
	emc_writel(emc_sel_dpd_ctrl, EMC_SEL_DPD_CTRL);
	emc_set_shadow_bypass(ASSEMBLY);

	if (next_timing->periodic_training) {
		__reset_dram_clktree_values(next_timing);

		wait_for_update(EMC_EMC_STATUS,
				EMC_EMC_STATUS_DRAM_IN_POWERDOWN_MASK,
				0, REG_EMC);
		if (channel_mode)
			wait_for_update(EMC_EMC_STATUS,
				EMC_EMC_STATUS_DRAM_IN_POWERDOWN_MASK,
				0, REG_EMC1);

		wait_for_update(EMC_EMC_STATUS,
				EMC_EMC_STATUS_DRAM_IN_SELF_REFRESH_MASK,
				0, REG_EMC);
		if (channel_mode)
			wait_for_update(EMC_EMC_STATUS,
				      EMC_EMC_STATUS_DRAM_IN_SELF_REFRESH_MASK,
				      0, REG_EMC1);

		start_periodic_compensation();

		udelay(((1000 * actual_osc_clocks(last_timing->run_clocks)) /
			last_timing->rate) + 2);
		adel = update_clock_tree_delay(fake_timing, next_timing,
					       dram_dev_num, channel_mode);
		compensate_trimmer_applicable =
			next_timing->periodic_training &&
			((adel * 128 * next_timing_rate_mhz) / 1000000) >
			next_timing->tree_margin;
	}

	if (opt_war_200024907) {
		n_rtp = 16;
		if (source_clock_period >= 1000000/1866)
			n_rtp = 14;
		if (source_clock_period >= 1000000/1600)
			n_rtp = 12;
		if (source_clock_period >= 1000000/1333)
			n_rtp = 10;
		if (source_clock_period >= 1000000/1066)
			n_rtp = 8;

		delta_twatm = max_t(u32, div_o3(7500, source_clock_period), 8);

		t_rpst = ((last_timing->emc_mrw & 0x80) >> 7);
		t_rtm = fake_timing->dram_timings[RL] +
		       div_o3(3600, source_clock_period) +
		       max_t(u32, div_o3(7500, source_clock_period), 8) +
		       t_rpst + 1 + n_rtp;

		if (last_timing->burst_regs[EMC_RP_INDEX] < t_rtm) {
			if (t_rtm > (last_timing->burst_regs[EMC_R2P_INDEX] +
			    last_timing->burst_regs[EMC_RP_INDEX])) {
				r2p_war = t_rtm -
					 last_timing->burst_regs[EMC_RP_INDEX];
				rp_war = last_timing->burst_regs[EMC_RP_INDEX];
				trpab_war =
				       last_timing->burst_regs[EMC_TRPAB_INDEX];
				if (r2p_war > 63) {
					rp_war = r2p_war +
						last_timing->burst_regs
						[EMC_RP_INDEX] - 63;
					if (trpab_war < rp_war)
						trpab_war = rp_war;
					r2p_war = 63;
				}
			} else {
				r2p_war = last_timing->
					  burst_regs[EMC_R2P_INDEX];
				rp_war = last_timing->burst_regs[EMC_RP_INDEX];
				trpab_war =
				       last_timing->burst_regs[EMC_TRPAB_INDEX];
			}

			if (rp_war < delta_twatm) {
				w2p_war = last_timing->burst_regs[EMC_W2P_INDEX]
					+ delta_twatm - rp_war;
				if (w2p_war > 63) {
					rp_war = rp_war + w2p_war - 63;
					if (trpab_war < rp_war)
						trpab_war = rp_war;
					w2p_war = 63;
				}
			} else {
				w2p_war =
					last_timing->burst_regs[EMC_W2P_INDEX];
			}

			if ((last_timing->burst_regs[EMC_W2P_INDEX] ^
			     w2p_war) ||
			    (last_timing->burst_regs[EMC_R2P_INDEX] ^
			     r2p_war) ||
			    (last_timing->burst_regs[EMC_RP_INDEX] ^
			     rp_war) ||
			    (last_timing->burst_regs[EMC_TRPAB_INDEX] ^
			     trpab_war)) {
				emc_writel(rp_war, EMC_RP);
				emc_writel(r2p_war, EMC_R2P);
				emc_writel(w2p_war, EMC_W2P);
				emc_writel(trpab_war, EMC_TRPAB);
			}
			emc_timing_update(DUAL_CHANNEL);
		}
	}

	emc_writel(EMC_INTSTATUS_CLKCHANGE_COMPLETE, EMC_INTSTATUS);
	emc_set_shadow_bypass(ACTIVE);
	emc_writel(emc_cfg, EMC_CFG);
	emc_writel(emc_sel_dpd_ctrl, EMC_SEL_DPD_CTRL);
	emc_writel(emc_cfg_pipe_clk_o | EMC_CFG_PIPE_CLK_CLK_ALWAYS_ON,
		   EMC_CFG_PIPE_CLK);
	emc_writel(next_timing->emc_fdpd_ctrl_cmd_no_ramp &
		   ~EMC_FDPD_CTRL_CMD_NO_RAMP_CMD_DPD_NO_RAMP_ENABLE,
		   EMC_FDPD_CTRL_CMD_NO_RAMP);

	bg_regulator_mode_change =
		((next_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
		  EMC_PMACRO_BG_BIAS_CTRL_0_BGLP_E_PWRD) ^
		 (last_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
		  EMC_PMACRO_BG_BIAS_CTRL_0_BGLP_E_PWRD)) ||
		((next_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
		  EMC_PMACRO_BG_BIAS_CTRL_0_BG_E_PWRD) ^
		 (last_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
		  EMC_PMACRO_BG_BIAS_CTRL_0_BG_E_PWRD));
	enable_bglp_regulator =
		(next_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
		 EMC_PMACRO_BG_BIAS_CTRL_0_BGLP_E_PWRD) == 0;
	enable_bg_regulator =
		(next_timing->burst_regs[EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
		 EMC_PMACRO_BG_BIAS_CTRL_0_BG_E_PWRD) == 0;

	if (bg_regulator_mode_change) {
		if (enable_bg_regulator)
			emc_writel(last_timing->burst_regs
				   [EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
				   ~EMC_PMACRO_BG_BIAS_CTRL_0_BG_E_PWRD,
				   EMC_PMACRO_BG_BIAS_CTRL_0);
		else
			emc_writel(last_timing->burst_regs
				   [EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
				   ~EMC_PMACRO_BG_BIAS_CTRL_0_BGLP_E_PWRD,
				   EMC_PMACRO_BG_BIAS_CTRL_0);

	}

	if ((((last_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] &
	       EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQ_E_IVREF) == 0) &&
	     ((next_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] &
	       EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQ_E_IVREF) == 1)) ||
	    (((last_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] &
	       EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQS_E_IVREF) == 0) &&
	     ((next_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX] &
	       EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQS_E_IVREF) == 1))) {
		u32 pad_tx_ctrl =
		    next_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];
		u32 last_pad_tx_ctrl =
		    last_timing->burst_regs[EMC_PMACRO_DATA_PAD_TX_CTRL_INDEX];

		next_dqs_e_ivref = pad_tx_ctrl &
				  EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQS_E_IVREF;
		next_dq_e_ivref = pad_tx_ctrl &
				  EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQ_E_IVREF;
		next_push = (last_pad_tx_ctrl &
			     ~EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQ_E_IVREF &
			     ~EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQS_E_IVREF) |
			     next_dq_e_ivref | next_dqs_e_ivref;
		emc_writel(next_push, EMC_PMACRO_DATA_PAD_TX_CTRL);
		udelay(1);
	} else if (bg_regulator_mode_change) {
		udelay(1);
	}

	emc_set_shadow_bypass(ASSEMBLY);

	if (next_timing->burst_regs[EMC_CFG_DIG_DLL_INDEX] &
	    EMC_CFG_DIG_DLL_CFG_DLL_EN) {
		dll_out = dll_prelock(next_timing, 0, clksrc);
		prelock_dll_en = 1;
	} else {
		dll_disable(channel_mode);
	}

	emc_set_shadow_bypass(ACTIVE);
	emc_writel(next_timing->emc_auto_cal_config2, EMC_AUTO_CAL_CONFIG2);
	emc_writel(next_timing->emc_auto_cal_config3, EMC_AUTO_CAL_CONFIG3);
	emc_writel(next_timing->emc_auto_cal_config4, EMC_AUTO_CAL_CONFIG4);
	emc_writel(next_timing->emc_auto_cal_config5, EMC_AUTO_CAL_CONFIG5);
	emc_writel(next_timing->emc_auto_cal_config6, EMC_AUTO_CAL_CONFIG6);
	emc_writel(next_timing->emc_auto_cal_config7, EMC_AUTO_CAL_CONFIG7);
	emc_writel(next_timing->emc_auto_cal_config8, EMC_AUTO_CAL_CONFIG8);
	emc_set_shadow_bypass(ASSEMBLY);

	emc_auto_cal_config |= (EMC_AUTO_CAL_CONFIG_AUTO_CAL_COMPUTE_START |
				auto_cal_en);
	emc_writel(emc_auto_cal_config, EMC_AUTO_CAL_CONFIG);

	if (source_clock_period > 50000 && dram_type == DRAM_TYPE_LPDDR4)
		ccfifo_writel(1, EMC_SELF_REF, 0);
	else
		emc_writel(next_timing->emc_cfg_2, EMC_CFG_2);

	emc_zcal_interval = 0;
	emc_zcal_wait_cnt_old =
		last_timing->burst_regs[EMC_ZCAL_WAIT_CNT_INDEX];
	emc_zcal_wait_cnt_new =
		next_timing->burst_regs[EMC_ZCAL_WAIT_CNT_INDEX];
	emc_zcal_wait_cnt_old &= ~EMC_ZCAL_WAIT_CNT_ZCAL_WAIT_CNT_MASK;
	emc_zcal_wait_cnt_new &= ~EMC_ZCAL_WAIT_CNT_ZCAL_WAIT_CNT_MASK;

	if (dram_type == DRAM_TYPE_LPDDR4)
		zq_wait_long = max((u32)1,
				   div_o3(1000000, destination_clock_period));
	else if (dram_type == DRAM_TYPE_LPDDR2 || is_lpddr3)
		zq_wait_long = max(next_timing->min_mrs_wait,
				 div_o3(360000, destination_clock_period)) + 4;
	else if (dram_type == DRAM_TYPE_DDR3)
		zq_wait_long = max((u32)256,
				 div_o3(320000, destination_clock_period) + 2);
	else
		zq_wait_long = 0;

	if (dram_type == DRAM_TYPE_LPDDR2 || is_lpddr3)
		zq_wait_short = max(max(next_timing->min_mrs_wait, (u32)6),
				  div_o3(90000, destination_clock_period)) + 4;
	else if (dram_type == DRAM_TYPE_DDR3)
		zq_wait_short = max((u32)64,
				  div_o3(80000, destination_clock_period)) + 2;
	else
		zq_wait_short = 0;

	if (!fsp_for_next_freq) {
		mr13_flip_fspwr = (next_timing->emc_mrw3 & 0xffffff3f) | 0x80;
		mr13_flip_fspop = (next_timing->emc_mrw3 & 0xffffff3f) | 0x00;
	} else {
		mr13_flip_fspwr = (next_timing->emc_mrw3 & 0xffffff3f) | 0x40;
		mr13_flip_fspop = (next_timing->emc_mrw3 & 0xffffff3f) | 0xc0;
	}

	mr13_catr_enable = (mr13_flip_fspwr & 0xFFFFFFFE) | 0x01;
	if (dram_dev_num == TWO_RANK)
		mr13_catr_enable =
				(mr13_catr_enable & 0x3fffffff) | 0x80000000;

	if (dram_type == DRAM_TYPE_LPDDR4) {
		emc_writel(mr13_flip_fspwr, EMC_MRW3);
		emc_writel(next_timing->emc_mrw, EMC_MRW);
		emc_writel(next_timing->emc_mrw2, EMC_MRW2);
	}

	for (i = 0; i < next_timing->num_burst; i++) {
		u32 var;
		u32 wval;

		if (!burst_regs_off[i])
			continue;

		var = burst_regs_off[i];
		wval = next_timing->burst_regs[i];

		if (dram_type != DRAM_TYPE_LPDDR4 &&
		    (var == EMC_MRW6      || var == EMC_MRW7 ||
		     var == EMC_MRW8      || var == EMC_MRW9 ||
		     var == EMC_MRW10     || var == EMC_MRW11 ||
		     var == EMC_MRW12     || var == EMC_MRW13 ||
		     var == EMC_MRW14     || var == EMC_MRW15 ||
		     var == EMC_TRAINING_CTRL))
			continue;

		if (var == EMC_CFG) {
			wval &= ~EMC_CFG_DRAM_ACPD;
			wval &= ~EMC_CFG_DYN_SELF_REF;
			if (dram_type == DRAM_TYPE_LPDDR4) {
				wval &= ~EMC_CFG_DRAM_CLKSTOP_SR;
				wval &= ~EMC_CFG_DRAM_CLKSTOP_PD;
			}
		} else if (var == EMC_MRS_WAIT_CNT &&
			   dram_type == DRAM_TYPE_LPDDR2 &&
			   opt_zcal_en_cc &&
			   !opt_cc_short_zcal &&
			   opt_short_zcal) {
			wval = (wval &
			      ~(EMC_MRS_WAIT_CNT_SHORT_WAIT_MASK <<
				EMC_MRS_WAIT_CNT_SHORT_WAIT_SHIFT)) |
			      ((zq_wait_long &
				EMC_MRS_WAIT_CNT_SHORT_WAIT_MASK) <<
				EMC_MRS_WAIT_CNT_SHORT_WAIT_SHIFT);
		} else if (var == EMC_ZCAL_WAIT_CNT &&
			   dram_type == DRAM_TYPE_DDR3 &&
			   opt_zcal_en_cc &&
			   !opt_cc_short_zcal &&
			   opt_short_zcal) {
			wval = (wval &
			      ~(EMC_ZCAL_WAIT_CNT_ZCAL_WAIT_CNT_MASK <<
				EMC_ZCAL_WAIT_CNT_ZCAL_WAIT_CNT_SHIFT)) |
			      ((zq_wait_long &
				EMC_ZCAL_WAIT_CNT_ZCAL_WAIT_CNT_MASK) <<
				EMC_MRS_WAIT_CNT_SHORT_WAIT_SHIFT);
		} else if (var == EMC_ZCAL_INTERVAL && opt_zcal_en_cc) {
			wval = 0;
		} else if (var == EMC_PMACRO_AUTOCAL_CFG_COMMON) {
			wval |= EMC_PMACRO_AUTOCAL_CFG_COMMON_E_CAL_BYPASS_DVFS;
		} else if (var == EMC_PMACRO_DATA_PAD_TX_CTRL) {
			wval &=
			     ~(EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSP_TX_E_DCC |
			       EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQSN_TX_E_DCC |
			       EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_DQ_TX_E_DCC |
			       EMC_PMACRO_DATA_PAD_TX_CTRL_DATA_CMD_TX_E_DCC);
		} else if (var == EMC_PMACRO_CMD_PAD_TX_CTRL) {
			wval |= EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQ_TX_DRVFORCEON;
			wval &= ~(EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSP_TX_E_DCC |
				  EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQSN_TX_E_DCC |
				  EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_DQ_TX_E_DCC |
				  EMC_PMACRO_CMD_PAD_TX_CTRL_CMD_CMD_TX_E_DCC);
		} else if (var == EMC_PMACRO_BRICK_CTRL_RFU1) {
			wval &= 0xf800f800;
		} else if (var == EMC_PMACRO_COMMON_PAD_TX_CTRL) {
			wval &= 0xfffffff0;
		}

		emc_writel(wval, var);
	}

	set_over_temp_timing(next_timing, dram_over_temp_state);

	if (dram_type == DRAM_TYPE_LPDDR4) {
		mrw_req = (23 << EMC_MRW_MRW_MA_SHIFT) |
			  (next_timing->run_clocks & EMC_MRW_MRW_OP_MASK);
		emc_writel(mrw_req, EMC_MRW);
	}

	for (i = 0; i < next_timing->num_burst_per_ch; i++) {
		if (!burst_regs_per_ch_off[i])
			continue;

		if (dram_type != DRAM_TYPE_LPDDR4 &&
			(burst_regs_per_ch_off[i] == EMC_MRW6 ||
			burst_regs_per_ch_off[i] == EMC_MRW7 ||
			burst_regs_per_ch_off[i] == EMC_MRW8 ||
			burst_regs_per_ch_off[i] == EMC_MRW9 ||
			burst_regs_per_ch_off[i] == EMC_MRW10 ||
			burst_regs_per_ch_off[i] == EMC_MRW11 ||
			burst_regs_per_ch_off[i] == EMC_MRW12 ||
			burst_regs_per_ch_off[i] == EMC_MRW13 ||
			burst_regs_per_ch_off[i] == EMC_MRW14 ||
			burst_regs_per_ch_off[i] == EMC_MRW15))
			continue;

		if (channel_mode != DUAL_CHANNEL &&
			burst_regs_per_ch_type[i] >= REG_EMC1)
			continue;

		emc_writel_per_ch(next_timing->burst_reg_per_ch[i],
			burst_regs_per_ch_type[i], burst_regs_per_ch_off[i]);
	}

	for (i = 0; i < next_timing->vref_num; i++) {
		if (!vref_regs_per_ch_off[i])
			continue;

		if (channel_mode != DUAL_CHANNEL &&
			vref_regs_per_ch_type[i] >= REG_EMC1)
			continue;

		emc_writel_per_ch(next_timing->vref_perch_regs[i],
				  vref_regs_per_ch_type[i],
				  vref_regs_per_ch_off[i]);
	}

	for (i = 0; i < next_timing->num_trim; i++) {
		u32 trim_reg;

		if (!trim_regs_off[i])
			continue;

		trim_reg = trim_regs_off[i];
		if (compensate_trimmer_applicable &&
		   (trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3 ||
		    trim_reg == EMC_DATA_BRLSHFT_0 ||
		    trim_reg == EMC_DATA_BRLSHFT_1)) {
			u32 reg = apply_periodic_compensation_trimmer(
							next_timing, trim_reg);
			emc_writel(reg, trim_regs_off[i]);
		} else {
			emc_writel(next_timing->trim_regs[i],
				   trim_regs_off[i]);
		}
	}

	for (i = 0; i < next_timing->num_trim_per_ch; i++) {
		u32 trim_reg;

		if (!trim_regs_per_ch_off[i])
			continue;

		if (channel_mode != DUAL_CHANNEL &&
		    trim_regs_per_ch_type[i] >= REG_EMC1)
			continue;

		trim_reg = trim_regs_per_ch_off[i];
		if (compensate_trimmer_applicable &&
		   (trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2 ||
		    trim_reg == EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3 ||
		    trim_reg == EMC_DATA_BRLSHFT_0 ||
		    trim_reg == EMC_DATA_BRLSHFT_1)) {
			u32 reg = apply_periodic_compensation_trimmer(
							next_timing, trim_reg);
			emc_writel_per_ch(reg, trim_regs_per_ch_type[i],
					  trim_regs_per_ch_off[i]);
		} else {
			emc_writel_per_ch(next_timing->trim_perch_regs[i],
					  trim_regs_per_ch_type[i],
					  trim_regs_per_ch_off[i]);
		}
	}

	for (i = 0; i < next_timing->num_mc_regs; i++)
		mc_writel(next_timing->burst_mc_regs[i], burst_mc_regs_off[i]);

	if (next_timing->rate < last_timing->rate) {
		for (i = 0; i < next_timing->num_up_down; i++) {
			mc_writel(next_timing->la_scale_regs[i],
				  la_scale_regs_off[i]);
		}
	}

	wmb();

	if (dram_type == DRAM_TYPE_LPDDR4) {
		emc_writel(emc_zcal_interval, EMC_ZCAL_INTERVAL);
		emc_writel(emc_zcal_wait_cnt_new, EMC_ZCAL_WAIT_CNT);

		emc_dbg |= (EMC_DBG_WRITE_MUX_ACTIVE |
			    EMC_DBG_WRITE_ACTIVE_ONLY);

		emc_writel(emc_dbg, EMC_DBG);
		emc_writel(emc_zcal_interval, EMC_ZCAL_INTERVAL);
		emc_writel(emc_dbg_o, EMC_DBG);
	}

	if (opt_dvfs_mode == MAN_SR || dram_type == DRAM_TYPE_LPDDR4) {
		if (dram_type == DRAM_TYPE_LPDDR4)
			ccfifo_writel(0x101, EMC_SELF_REF, 0);
		else
			ccfifo_writel(0x1, EMC_SELF_REF, 0);

		if (dram_type == DRAM_TYPE_LPDDR4 &&
		    source_clock_period <= zqcal_before_cc_cutoff) {
			ccfifo_writel(mr13_flip_fspwr ^ 0x40, EMC_MRW3, 0);
			ccfifo_writel(
				(next_timing->burst_regs[EMC_MRW6_INDEX] &
				0xFFFF3F3F) |
				(last_timing->burst_regs[EMC_MRW6_INDEX] &
				0x0000C0C0), EMC_MRW6, 0);
			ccfifo_writel(
				(next_timing->burst_regs[EMC_MRW14_INDEX] &
				 0xFFFF0707) |
				(last_timing->burst_regs[EMC_MRW14_INDEX] &
				 0x00003838), EMC_MRW14, 0);

			if (dram_dev_num == TWO_RANK) {
				ccfifo_writel(
				     (next_timing->burst_regs[EMC_MRW7_INDEX] &
				      0xFFFF3F3F) |
				     (last_timing->burst_regs[EMC_MRW7_INDEX] &
				      0x0000C0C0), EMC_MRW7, 0);
				ccfifo_writel(
				     (next_timing->burst_regs[EMC_MRW15_INDEX] &
				      0xFFFF0707) |
				     (last_timing->burst_regs[EMC_MRW15_INDEX] &
				      0x00003838), EMC_MRW15, 0);
			}
			if (opt_zcal_en_cc) {
				if (dram_dev_num == ONE_RANK)
					ccfifo_writel(
						2 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
						EMC_ZQ_CAL_ZQ_CAL_CMD,
						EMC_ZQ_CAL, 0);
				else if (shared_zq_resistor)
					ccfifo_writel(
						2 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
						EMC_ZQ_CAL_ZQ_CAL_CMD,
						EMC_ZQ_CAL, 0);
				else
					ccfifo_writel(EMC_ZQ_CAL_ZQ_CAL_CMD,
						      EMC_ZQ_CAL, 0);
			}
		}
	}

	emc_dbg = emc_dbg_o;
	if (dram_type == DRAM_TYPE_LPDDR4) {
		ccfifo_writel(mr13_flip_fspop | 0x8, EMC_MRW3,
			     (1000 * fake_timing->dram_timings[T_RP]) /
			      source_clock_period);
		ccfifo_writel(0, 0, t_fc_lpddr4 / source_clock_period);
	}

	if (dram_type == DRAM_TYPE_LPDDR4 || opt_dvfs_mode != MAN_SR) {
		u32 t = 30 + (cya_allow_ref_cc ?
			(4000 * fake_timing->dram_timings[T_RFC]) +
			((1000 * fake_timing->dram_timings[T_RP]) /
			source_clock_period) : 0);

		ccfifo_writel(emc_pin_o & ~(EMC_PIN_PIN_CKE_PER_DEV |
					    EMC_PIN_PIN_CKEB |
					    EMC_PIN_PIN_CKE),
			      EMC_PIN, t);
	}

	ref_delay_mult = 1;
	ref_b4_sref_en = 0;
	cya_issue_pc_ref = 0;

	ref_delay_mult += ref_b4_sref_en   ? 1 : 0;
	ref_delay_mult += cya_allow_ref_cc ? 1 : 0;
	ref_delay_mult += cya_issue_pc_ref ? 1 : 0;
	ref_delay = ref_delay_mult *
		    ((1000 * fake_timing->dram_timings[T_RP] /
		      source_clock_period) +
		     (1000 * fake_timing->dram_timings[T_RFC] /
		      source_clock_period)) + 20;

	ccfifo_writel(0x0, EMC_CFG_SYNC,
		      dram_type == DRAM_TYPE_LPDDR4 ? 0 : ref_delay);

	emc_dbg_active = emc_dbg | (EMC_DBG_WRITE_MUX_ACTIVE |
				    EMC_DBG_WRITE_ACTIVE_ONLY);
	ccfifo_writel(emc_dbg_active, EMC_DBG, 0);

	ramp_down_wait = do_dvfs_power_ramp_down(source_clock_period, 0,
						 last_timing, next_timing);

	ccfifo_writel(1, EMC_STALL_THEN_EXE_AFTER_CLKCHANGE, 0);
	emc_dbg_active &= ~EMC_DBG_WRITE_ACTIVE_ONLY;
	ccfifo_writel(emc_dbg_active, EMC_DBG, 0);

	ramp_up_wait = do_dvfs_power_ramp_up(destination_clock_period, 0,
					     last_timing, next_timing);
	ccfifo_writel(emc_dbg, EMC_DBG, 0);

	if (dram_type == DRAM_TYPE_LPDDR4) {
		u32 r = emc_pin_o | EMC_PIN_PIN_CKE;
		if (dram_dev_num == TWO_RANK)
			ccfifo_writel(r | EMC_PIN_PIN_CKEB |
				      EMC_PIN_PIN_CKE_PER_DEV,
				      EMC_PIN, 0);
		else
			ccfifo_writel(r & ~(EMC_PIN_PIN_CKEB |
					    EMC_PIN_PIN_CKE_PER_DEV),
				      EMC_PIN, 0);
	}

	if (source_clock_period <= zqcal_before_cc_cutoff) {
		s32 t = (s32)(ramp_up_wait + ramp_down_wait) /
			(s32)destination_clock_period;
		zq_latch_dvfs_wait_time = (s32)t_zqcal_lpddr4_fc_adj - t;
	} else {
		zq_latch_dvfs_wait_time = t_zqcal_lpddr4_fc_adj -
			div_o3(1000 * next_timing->dram_timings[T_PDEX],
			       destination_clock_period);
	}

	if (dram_type == DRAM_TYPE_LPDDR4 && opt_zcal_en_cc) {
		if (dram_dev_num == ONE_RANK) {
			if (source_clock_period > zqcal_before_cc_cutoff)
				ccfifo_writel(2 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
					  EMC_ZQ_CAL_ZQ_CAL_CMD,
					  EMC_ZQ_CAL,
					  div_o3(1000 *
					  next_timing->dram_timings[T_PDEX],
					  destination_clock_period));
			ccfifo_writel((mr13_flip_fspop & 0xFFFFFFF7) |
				       0x0C000000, EMC_MRW3,
				       div_o3(1000 *
					 next_timing->dram_timings[T_PDEX],
					 destination_clock_period));
			ccfifo_writel(0, EMC_SELF_REF, 0);
			ccfifo_writel(0, EMC_REF, 0);
			ccfifo_writel(2 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
				      EMC_ZQ_CAL_ZQ_LATCH_CMD, EMC_ZQ_CAL,
				      max_t(s32, 0, zq_latch_dvfs_wait_time));
		} else if (shared_zq_resistor) {
			if (source_clock_period > zqcal_before_cc_cutoff)
				ccfifo_writel(2 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
					  EMC_ZQ_CAL_ZQ_CAL_CMD,
					  EMC_ZQ_CAL,
					  div_o3(1000 *
					  next_timing->dram_timings[T_PDEX],
					  destination_clock_period));
			ccfifo_writel(2 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
				      EMC_ZQ_CAL_ZQ_LATCH_CMD, EMC_ZQ_CAL,
				      max_t(s32, 0, zq_latch_dvfs_wait_time) +
				      div_o3(1000 *
					 next_timing->dram_timings[T_PDEX],
					 destination_clock_period));
			ccfifo_writel(1 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
				      EMC_ZQ_CAL_ZQ_LATCH_CMD, EMC_ZQ_CAL, 0);
			ccfifo_writel((mr13_flip_fspop & 0xfffffff7) |
				       0x0c000000, EMC_MRW3, 0);
			ccfifo_writel(0, EMC_SELF_REF, 0);
			ccfifo_writel(0, EMC_REF, 0);
			ccfifo_writel(1 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
				      EMC_ZQ_CAL_ZQ_LATCH_CMD, EMC_ZQ_CAL,
				      t_zqcal_lpddr4 /
				      destination_clock_period);
		} else {
			if (source_clock_period > zqcal_before_cc_cutoff) {
				ccfifo_writel(EMC_ZQ_CAL_ZQ_CAL_CMD, EMC_ZQ_CAL,
					 div_o3(1000 *
					 next_timing->dram_timings[T_PDEX],
					 destination_clock_period));
			}

			ccfifo_writel((mr13_flip_fspop & 0xfffffff7) |
				       0x0c000000, EMC_MRW3,
				       div_o3(1000 *
					 next_timing->dram_timings[T_PDEX],
					 destination_clock_period));
			ccfifo_writel(0, EMC_SELF_REF, 0);
			ccfifo_writel(0, EMC_REF, 0);
			ccfifo_writel(EMC_ZQ_CAL_ZQ_LATCH_CMD, EMC_ZQ_CAL,
				      max_t(s32, 0, zq_latch_dvfs_wait_time));
		}
	}

	ccfifo_writel(0, 0, 10);

	if (opt_dvfs_mode == MAN_SR && dram_type != DRAM_TYPE_LPDDR4)
		ccfifo_writel(0, EMC_SELF_REF, 0);

	if (dram_type == DRAM_TYPE_LPDDR2) {
		ccfifo_writel(next_timing->emc_mrw2, EMC_MRW2, 0);
		ccfifo_writel(next_timing->emc_mrw,  EMC_MRW,  0);
		if (is_lpddr3)
			ccfifo_writel(next_timing->emc_mrw4, EMC_MRW4, 0);
	} else if (dram_type == DRAM_TYPE_DDR3) {
		if (opt_dll_mode == DLL_ON)
			ccfifo_writel(next_timing->emc_emrs &
				      ~EMC_EMRS_USE_EMRS_LONG_CNT,
				      EMC_EMRS, 0);
		ccfifo_writel(next_timing->emc_emrs2 &
			      ~EMC_EMRS2_USE_EMRS2_LONG_CNT, EMC_EMRS2, 0);
		ccfifo_writel(next_timing->emc_mrs |
			      EMC_EMRS_USE_EMRS_LONG_CNT, EMC_MRS, 0);
	}

	if (opt_zcal_en_cc) {
		if (dram_type == DRAM_TYPE_LPDDR2) {
			u32 r;

			zq_op = opt_cc_short_zcal  ? 0x56 : 0xAB;
			zcal_wait_time_ps = opt_cc_short_zcal  ? 90000 : 360000;
			zcal_wait_time_clocks = div_o3(zcal_wait_time_ps,
						     destination_clock_period);
			r = zcal_wait_time_clocks <<
			    EMC_MRS_WAIT_CNT2_MRS_EXT2_WAIT_CNT_SHIFT |
			    zcal_wait_time_clocks <<
			    EMC_MRS_WAIT_CNT2_MRS_EXT1_WAIT_CNT_SHIFT;
			ccfifo_writel(r, EMC_MRS_WAIT_CNT2, 0);
			ccfifo_writel(2 << EMC_MRW_MRW_DEV_SELECTN_SHIFT |
				      EMC_MRW_USE_MRW_EXT_CNT |
				      10 << EMC_MRW_MRW_MA_SHIFT |
				      zq_op << EMC_MRW_MRW_OP_SHIFT,
				      EMC_MRW, 0);
			if (dram_dev_num == TWO_RANK) {
				r = 1 << EMC_MRW_MRW_DEV_SELECTN_SHIFT |
				    EMC_MRW_USE_MRW_EXT_CNT |
				    10 << EMC_MRW_MRW_MA_SHIFT |
				    zq_op << EMC_MRW_MRW_OP_SHIFT;
				ccfifo_writel(r, EMC_MRW, 0);
			}
		} else if (dram_type == DRAM_TYPE_DDR3) {
			zq_op = opt_cc_short_zcal ? 0 : EMC_ZQ_CAL_LONG;
			ccfifo_writel(zq_op | 2 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
				      EMC_ZQ_CAL_ZQ_CAL_CMD, EMC_ZQ_CAL, 0);
			if (dram_dev_num == TWO_RANK)
				ccfifo_writel(zq_op |
					      1 << EMC_ZQ_CAL_DEV_SEL_SHIFT |
					      EMC_ZQ_CAL_ZQ_CAL_CMD,
					      EMC_ZQ_CAL, 0);
		}
	}

	if (bg_regulator_mode_change) {
		emc_set_shadow_bypass(ACTIVE);
		bg_regulator_switch_complete_wait_clks =
			    ramp_up_wait > 1250000 ? 0 :
			    (1250000 - ramp_up_wait) / destination_clock_period;
		ccfifo_writel(next_timing->burst_regs
			      [EMC_PMACRO_BG_BIAS_CTRL_0_INDEX],
			      EMC_PMACRO_BG_BIAS_CTRL_0,
			      bg_regulator_switch_complete_wait_clks);
		emc_set_shadow_bypass(ASSEMBLY);
	}

	if (dram_type != DRAM_TYPE_LPDDR4)
		ccfifo_writel(0, EMC_REF, 0);

	if (opt_do_sw_qrst) {
		ccfifo_writel(1, EMC_ISSUE_QRST, 0);
		ccfifo_writel(0, EMC_ISSUE_QRST, 2);
	}

	if (save_restore_clkstop_pd || opt_zcal_en_cc) {
		ccfifo_writel(emc_dbg_o | EMC_DBG_WRITE_MUX_ACTIVE, EMC_DBG, 0);
		if (opt_zcal_en_cc && dram_type != DRAM_TYPE_LPDDR4)
			ccfifo_writel(next_timing->
				      burst_regs[EMC_ZCAL_INTERVAL_INDEX],
				      EMC_ZCAL_INTERVAL, 0);

		if (save_restore_clkstop_pd)
			ccfifo_writel(next_timing->burst_regs[EMC_CFG_INDEX] &
				      ~EMC_CFG_DYN_SELF_REF, EMC_CFG, 0);
		ccfifo_writel(emc_dbg_o, EMC_DBG, 0);
	}

	ccfifo_writel(emc_cfg_pipe_clk_o, EMC_CFG_PIPE_CLK, 0);

	if (bg_regulator_mode_change) {
		if (enable_bg_regulator)
			emc_writel(next_timing->burst_regs
				   [EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
				   ~EMC_PMACRO_BG_BIAS_CTRL_0_BGLP_E_PWRD,
				   EMC_PMACRO_BG_BIAS_CTRL_0);
		else
			emc_writel(next_timing->burst_regs
				   [EMC_PMACRO_BG_BIAS_CTRL_0_INDEX] &
				   ~EMC_PMACRO_BG_BIAS_CTRL_0_BG_E_PWRD,
				   EMC_PMACRO_BG_BIAS_CTRL_0);
	}

	tmp = emc_readl(EMC_CFG_DIG_DLL);
	tmp |= EMC_CFG_DIG_DLL_CFG_DLL_STALL_ALL_TRAFFIC;
	tmp &= ~EMC_CFG_DIG_DLL_CFG_DLL_STALL_RW_UNTIL_LOCK;
	tmp &= ~EMC_CFG_DIG_DLL_CFG_DLL_STALL_ALL_UNTIL_LOCK;
	tmp &= ~EMC_CFG_DIG_DLL_CFG_DLL_EN;
	tmp = (tmp & ~EMC_CFG_DIG_DLL_CFG_DLL_MODE_MASK) |
	      (2 << EMC_CFG_DIG_DLL_CFG_DLL_MODE_SHIFT);
	emc_writel(tmp, EMC_CFG_DIG_DLL);

	do_clock_change(clksrc);

	if (next_timing->rate > last_timing->rate) {
		for (i = 0; i < next_timing->num_up_down; i++)
			mc_writel(next_timing->la_scale_regs[i],
				  la_scale_regs_off[i]);
		emc_timing_update(0);
	}

	if (dram_type == DRAM_TYPE_LPDDR4) {
		emc_set_shadow_bypass(ACTIVE);
		emc_writel(next_timing->burst_regs[EMC_ZCAL_WAIT_CNT_INDEX],
			   EMC_ZCAL_WAIT_CNT);
		emc_writel(next_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX],
			   EMC_ZCAL_INTERVAL);
		emc_set_shadow_bypass(ASSEMBLY);
	}

	if (dram_type != DRAM_TYPE_LPDDR4 && opt_zcal_en_cc &&
	    !opt_short_zcal && opt_cc_short_zcal) {
		udelay(2);

		emc_set_shadow_bypass(ACTIVE);
		if (dram_type == DRAM_TYPE_LPDDR2)
			emc_writel(next_timing->
				   burst_regs[EMC_MRS_WAIT_CNT_INDEX],
				   EMC_MRS_WAIT_CNT);
		else if (dram_type == DRAM_TYPE_DDR3)
			emc_writel(next_timing->
				   burst_regs[EMC_ZCAL_WAIT_CNT_INDEX],
				   EMC_ZCAL_WAIT_CNT);
		emc_set_shadow_bypass(ASSEMBLY);
	}

	emc_set_shadow_bypass(ACTIVE);
	emc_writel(next_timing->burst_regs[EMC_CFG_INDEX], EMC_CFG);
	emc_set_shadow_bypass(ASSEMBLY);
	emc_writel(next_timing->emc_fdpd_ctrl_cmd_no_ramp,
		   EMC_FDPD_CTRL_CMD_NO_RAMP);
	emc_writel(next_timing->emc_sel_dpd_ctrl, EMC_SEL_DPD_CTRL);

	emc_set_shadow_bypass(ACTIVE);
	emc_writel(next_timing->burst_regs[EMC_PMACRO_AUTOCAL_CFG_COMMON_INDEX],
		   EMC_PMACRO_AUTOCAL_CFG_COMMON);
	emc_set_shadow_bypass(ASSEMBLY);

	emc_writel(EMC_PMACRO_CFG_PM_GLOBAL_0_DISABLE_CFG_BYTE0 |
		   EMC_PMACRO_CFG_PM_GLOBAL_0_DISABLE_CFG_BYTE1 |
		   EMC_PMACRO_CFG_PM_GLOBAL_0_DISABLE_CFG_BYTE2 |
		   EMC_PMACRO_CFG_PM_GLOBAL_0_DISABLE_CFG_BYTE3 |
		   EMC_PMACRO_CFG_PM_GLOBAL_0_DISABLE_CFG_BYTE4 |
		   EMC_PMACRO_CFG_PM_GLOBAL_0_DISABLE_CFG_BYTE5 |
		   EMC_PMACRO_CFG_PM_GLOBAL_0_DISABLE_CFG_BYTE6 |
		   EMC_PMACRO_CFG_PM_GLOBAL_0_DISABLE_CFG_BYTE7,
		   EMC_PMACRO_CFG_PM_GLOBAL_0);
	emc_writel(EMC_PMACRO_TRAINING_CTRL_0_CH0_TRAINING_E_WRPTR,
		   EMC_PMACRO_TRAINING_CTRL_0);
	emc_writel(EMC_PMACRO_TRAINING_CTRL_1_CH1_TRAINING_E_WRPTR,
		   EMC_PMACRO_TRAINING_CTRL_1);
	emc_writel(0, EMC_PMACRO_CFG_PM_GLOBAL_0);

	if (next_timing->burst_regs[EMC_CFG_DIG_DLL_INDEX] &
	    EMC_CFG_DIG_DLL_CFG_DLL_EN) {
		tmp = emc_readl(EMC_CFG_DIG_DLL);
		tmp |=  EMC_CFG_DIG_DLL_CFG_DLL_STALL_ALL_TRAFFIC;
		tmp |=  EMC_CFG_DIG_DLL_CFG_DLL_EN;
		tmp &= ~EMC_CFG_DIG_DLL_CFG_DLL_STALL_RW_UNTIL_LOCK;
		tmp &= ~EMC_CFG_DIG_DLL_CFG_DLL_STALL_ALL_UNTIL_LOCK;
		tmp = (tmp & ~EMC_CFG_DIG_DLL_CFG_DLL_MODE_MASK) |
		      (2 << EMC_CFG_DIG_DLL_CFG_DLL_MODE_SHIFT);
		emc_writel(tmp, EMC_CFG_DIG_DLL);
		emc_timing_update(channel_mode);
	}

	emc_auto_cal_config = next_timing->emc_auto_cal_config;
	emc_writel(emc_auto_cal_config, EMC_AUTO_CAL_CONFIG);

	__update_emc_alt_timing(next_timing);
}
