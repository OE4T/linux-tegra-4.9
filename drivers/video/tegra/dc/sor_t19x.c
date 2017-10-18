/*
 * drivers/video/tegra/dc/sor_t19x.c
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "dc_priv.h"
#include "edid.h"
#include "hdmi2.0.h"
#include "hw_nvdisp_nvdisp.h"
#include "nvdisp.h"
#include "sor.h"
#include "sor_t19x.h"
#include "sor_regs_t19x.h"
#include "dp.h"
#include "dp_t19x.h"

static const struct tegra_dc_sor_link_speed link_speed_table_t19x[] = {
	[TEGRA_DC_SOR_LINK_SPEED_G1_62] = {
		.prod_prop = "prod_c_rbr",
		.max_link_bw = 1620,
		.link_rate = 6,
	},
	[TEGRA_DC_SOR_LINK_SPEED_G2_7] = {
		.prod_prop = "prod_c_hbr",
		.max_link_bw = 2700,
		.link_rate = 10,
	},
	[TEGRA_DC_SOR_LINK_SPEED_G5_4] = {
		.prod_prop = "prod_c_hbr2",
		.max_link_bw = 5400,
		.link_rate = 20,
	},
	[TEGRA_DC_SOR_LINK_SPEED_G8_1] = {
		.prod_prop = "prod_c_hbr3",
		.max_link_bw = 8100,
		.link_rate = 30,
	},
};

static const struct tegra_dc_dp_training_pattern training_pattern_table_t19x[] =
{
	[TEGRA_DC_DP_TRAINING_PATTERN_DISABLE] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_NONE,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_NOPATTERN
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_1] = {
		.chan_coding = true,
		.scrambling = false,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_TP1,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_TRAINING1
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_2] = {
		.chan_coding = true,
		.scrambling = false,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_TP2,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_TRAINING2
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_3] = {
		.chan_coding = true,
		.scrambling = false,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_TP3,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_TRAINING3
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_D102] = {
		.chan_coding = true,
		.scrambling = false,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_D102
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_SBLERRRATE] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_SBLERRRATE
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_PRBS7] = {
		.chan_coding = false,
		.scrambling = false,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_PRBS7
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_CSTM] = {
		.chan_coding = false,
		.scrambling = false,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_CSTM
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_HBR2_COMPLIANCE] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_HBR2_COMPLIANCE
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_4] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_TP4,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_TRAINING4
	},
};

/* "Quirks" is a deliberate misnomer, so that we could potentially use this
 * function to add more Xavier specific init overrides here
 */
inline void tegra_sor_init_quirks_t19x(struct tegra_dc_sor_data *sor)
{
	sor->link_speeds = link_speed_table_t19x;
	sor->num_link_speeds = ARRAY_SIZE(link_speed_table_t19x);

	sor->training_patterns = training_pattern_table_t19x;
	sor->num_training_patterns = ARRAY_SIZE(training_pattern_table_t19x);
}

inline u32 nv_sor_head_state0_t19x(u32 i)
{
	return NV_HEAD_STATE0_T19X(i);
}

inline u32 nv_sor_head_state1_t19x(u32 i)
{
	return NV_HEAD_STATE1_T19X(i);
}

inline u32 nv_sor_head_state2_t19x(u32 i)
{
	return NV_HEAD_STATE2_T19X(i);
}

inline u32 nv_sor_head_state3_t19x(u32 i)
{
	return NV_HEAD_STATE3_T19X(i);
}

inline u32 nv_sor_head_state4_t19x(u32 i)
{
	return NV_HEAD_STATE4_T19X(i);
}

inline u32 nv_sor_head_state5_t19x(u32 i)
{
	return NV_HEAD_STATE5_T19X(i);
}

inline u32 nv_sor_pll0_t19x(void)
{
	return NV_SOR_PLL0_T19X;
}

inline u32 nv_sor_pll1_t19x(void)
{
	return NV_SOR_PLL1_T19X;
}

inline u32 nv_sor_pll2_t19x(void)
{
	return NV_SOR_PLL2_T19X;
}

inline u32 nv_sor_pll3_t19x(void)
{
	return NV_SOR_PLL3_T19X;
}

inline u32 nv_sor_pll4_t19x(void)
{
	return NV_SOR_PLL4_T19X;
}

inline u32 nv_sor_dp_padctl_t19x(u32 i)
{
	return NV_SOR_DP_PADCTL_T19X(i);
}

inline u32 nv_sor_dp_misc1_override_t19x(void)
{
	return NV_SOR_DP_MISC1_OVERRIDE_T19X;
}

inline u32 nv_sor_dp_misc1_bit6_t19x(void)
{
	return NV_SOR_DP_MISC1_BIT6_T19X;
}

inline u32 nv_sor_dp_int_enable_t19x(void)
{
	return NV_SOR_DP_INT_ENABLE_T19X;
}

/* This function either blocks or unblocks the SOR AFIFO. */
inline void tegra_sor_clk_switch_setup_t19x(struct tegra_dc_sor_data *sor,
					bool unblock)
{
	tegra_sor_write_field(sor, NV_SOR_DP_LINKCTL(sor->portnum),
			NV_SOR_DP_LINKCTL_ASYNC_FIFO_BLOCK_MASK,
			(unblock ? NV_SOR_DP_LINKCTL_ASYNC_FIFO_BLOCK_NO :
			NV_SOR_DP_LINKCTL_ASYNC_FIFO_BLOCK_YES));
}

inline void tegra_sor_program_fpga_clk_mux_t19x(
					struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;
	u32 reg_val = 0;

	if (dc->out->type == TEGRA_DC_OUT_HDMI) {
		reg_val |= NV_SOR_FPGA_CLK_SEL_FPGA_PCLK_MUX_SEL_HDMI;
		if (tegra_dc_is_yuv420_8bpc(&dc->mode))
			reg_val |= NV_SOR_FPGA_CLK_SEL_FPGA_HDMI420_SEL_ENABLE;
	} else if (dc->out->type == TEGRA_DC_OUT_DP ||
			dc->out->type == TEGRA_DC_OUT_FAKE_DP) {
		reg_val |= NV_SOR_FPGA_CLK_SEL_FPGA_PCLK_MUX_SEL_DP;
	} else {
		return;
	}

	tegra_sor_writel(sor, NV_SOR_FPGA_CLK_SEL, reg_val);
}

inline void tegra_sor_set_clk_rate_t19x(struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;
	int yuv_flag = dc->mode.vmode & FB_VMODE_YUV_MASK;
	long rate = dc->mode.pclk;

	if (!dc->yuv_bypass) {
		if ((IS_RGB(yuv_flag) && (yuv_flag == FB_VMODE_Y36)) ||
			(yuv_flag == (FB_VMODE_Y444 | FB_VMODE_Y36))) {
			/* 3:2 pclk:orclk */
			rate = rate >> 1;
			rate = rate * 3;
		} else if (tegra_dc_is_yuv420_8bpc(&dc->mode)) {
			/* 2:1 pclk:orclk */
			rate = rate >> 1;
		}
	}

	clk_set_rate(sor->sor_clk, rate);
}

u32 tegra_sor_get_pixel_depth_t19x(struct tegra_dc *dc)
{
	int yuv_flag = dc->mode.vmode & FB_VMODE_YUV_MASK;
	int yuv_bypass_mode = dc->mode.vmode & FB_VMODE_BYPASS;
	u32 pixel_depth = 0;

	if (dc->out->type == TEGRA_DC_OUT_HDMI && !yuv_bypass_mode) {
		if (tegra_dc_is_yuv420_8bpc(&dc->mode)) {
			pixel_depth = NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_12_420 |
				NV_SOR_STATE1_ASY_CHROMA_V_DECIMATE_ENABLE;
		} else if (yuv_flag & FB_VMODE_Y422) {
			if (yuv_flag & FB_VMODE_Y24)
				pixel_depth =
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_16_422;
			else if (yuv_flag & FB_VMODE_Y30)
				pixel_depth =
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_20_422;
			else if (yuv_flag & FB_VMODE_Y36)
				pixel_depth =
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_24_422;
		} else {
			if (yuv_flag & FB_VMODE_Y24)
				pixel_depth =
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_24_444;
			else if (yuv_flag & FB_VMODE_Y30)
				pixel_depth =
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_30_444;
			else if (yuv_flag & FB_VMODE_Y36)
				pixel_depth =
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_36_444;
		}
	} else {
		pixel_depth = (dc->out->depth > 18 || !dc->out->depth) ?
			NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_24_444 :
			NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_18_444;
	}

	return pixel_depth;
}

inline u32 tegra_sor_get_adjusted_hblank_t19x(struct tegra_dc *dc,
					u32 hblank_end)
{
	if (dc->yuv_bypass || dc->out->type != TEGRA_DC_OUT_HDMI)
		return hblank_end;

	if (tegra_dc_is_yuv420_8bpc(&dc->mode))
		hblank_end = hblank_end / 2;

	return hblank_end;
}

