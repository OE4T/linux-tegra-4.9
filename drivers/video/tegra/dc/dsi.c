/*
 * drivers/video/tegra/dc/dsi.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <mach/clk.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/nvhost.h>
#include <../gpio-names.h>

#include "dc_reg.h"
#include "dc_priv.h"
#include "dsi_regs.h"
#include "dsi.h"

#define DSI_USE_SYNC_POINTS 0


#define DSI_MODULE_NOT_INIT		0x0
#define DSI_MODULE_INIT			0x1

#define DSI_LPHS_NOT_INIT		0x0
#define DSI_LPHS_IN_LP_MODE		0x1
#define DSI_LPHS_IN_HS_MODE		0x2

#define DSI_VIDEO_TYPE_NOT_INIT		0x0
#define DSI_VIDEO_TYPE_VIDEO_MODE	0x1
#define DSI_VIDEO_TYPE_CMD_MODE		0x2

#define DSI_DRIVEN_MODE_NOT_INIT	0x0
#define DSI_DRIVEN_MODE_DC		0x1
#define DSI_DRIVEN_MODE_HOST		0x2

#define DSI_PHYCLK_OUT_DIS		0x0
#define DSI_PHYCLK_OUT_EN		0x1

#define DSI_PHYCLK_NOT_INIT		0x0
#define DSI_PHYCLK_CONTINUOUS		0x1
#define DSI_PHYCLK_TX_ONLY		0x2

#define DSI_CLK_BURST_NOT_INIT		0x0
#define DSI_CLK_BURST_NONE_BURST	0x1
#define DSI_CLK_BURST_BURST_MODE	0x2

struct dsi_status {
	unsigned	init:2;

	unsigned	lphs:2;

	unsigned	vtype:2;
	unsigned	driven:2;

	unsigned	clk_out:2;
	unsigned	clk_mode:2;
	unsigned	clk_burst:2;
};

/* source of video data */
enum{
	TEGRA_DSI_DRIVEN_BY_DC,
	TEGRA_DSI_DRIVEN_BY_HOST,
};

struct tegra_dc_dsi_data {
	struct tegra_dc	*dc;
	void __iomem	*base;
	struct resource	*base_res;

	struct clk		*dc_clk;
	struct clk		*dsi_clk;

	struct mutex	lock;

	/* data from board info */
	struct tegra_dsi_out info;

	struct dsi_status	status;

	u8		driven_mode;
	u8		controller_index;

	u8		pixel_scaler_mul;
	u8		pixel_scaler_div;

	u32		default_pixel_clk_khz;
	u32		default_hs_clk_khz;

	u32		target_hs_clk_khz;
	u32		target_lp_clk_khz;

	u16		current_bit_clk_ns;
	u32		current_dsi_clk_khz;

	u32		dsi_control_val;
};

const u32 dsi_pkt_seq_reg[NUMOF_PKT_SEQ] = {
	DSI_PKT_SEQ_0_LO,
	DSI_PKT_SEQ_0_HI,
	DSI_PKT_SEQ_1_LO,
	DSI_PKT_SEQ_1_HI,
	DSI_PKT_SEQ_2_LO,
	DSI_PKT_SEQ_2_HI,
	DSI_PKT_SEQ_3_LO,
	DSI_PKT_SEQ_3_HI,
	DSI_PKT_SEQ_4_LO,
	DSI_PKT_SEQ_4_HI,
	DSI_PKT_SEQ_5_LO,
	DSI_PKT_SEQ_5_HI,
};

const u32 dsi_pkt_seq_video_non_burst_syne[NUMOF_PKT_SEQ] = {
	PKT_ID0(CMD_VS) | PKT_LEN0(0) | PKT_ID1(CMD_EOT) | PKT_LEN1(0) | PKT_LP,
	0,
	PKT_ID0(CMD_VE) | PKT_LEN0(0) | PKT_ID1(CMD_EOT) | PKT_LEN1(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_EOT) | PKT_LEN1(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(1) |
		PKT_ID2(CMD_HE) | PKT_LEN2(0),
	PKT_ID3(CMD_BLNK) | PKT_LEN3(2) | PKT_ID4(CMD_RGB) | PKT_LEN4(3) |
		PKT_ID5(CMD_BLNK) | PKT_LEN5(4),
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_EOT) | PKT_LEN1(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(1) |
		PKT_ID2(CMD_HE) | PKT_LEN2(0),
	PKT_ID3(CMD_BLNK) | PKT_LEN3(2) | PKT_ID4(CMD_RGB) | PKT_LEN4(3) |
		PKT_ID5(CMD_BLNK) | PKT_LEN5(4),
};

const u32 dsi_pkt_seq_video_non_burst[NUMOF_PKT_SEQ] = {
	PKT_ID0(CMD_VS) | PKT_LEN0(0) | PKT_ID1(CMD_EOT) | PKT_LEN1(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_EOT) | PKT_LEN1(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_EOT) | PKT_LEN1(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(2) |
		PKT_ID2(CMD_RGB) | PKT_LEN2(3),
	PKT_ID3(CMD_BLNK) | PKT_LEN3(4),
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_EOT) | PKT_LEN1(0) | PKT_LP,
	0,
	PKT_ID0(CMD_HS) | PKT_LEN0(0) | PKT_ID1(CMD_BLNK) | PKT_LEN1(2) |
		PKT_ID2(CMD_RGB) | PKT_LEN2(3),
	PKT_ID3(CMD_BLNK) | PKT_LEN3(4),
};

static const u32 dsi_pkt_seq_video_burst[NUMOF_PKT_SEQ] = {
	PKT_ID0(CMD_NULL) | PKT_LEN0(4) | PKT_ID1(CMD_VS) | PKT_LEN1(0) |
		PKT_ID2(CMD_EOT) | PKT_LEN2(0) | PKT_LP,
	0,
	PKT_ID0(CMD_NULL) | PKT_LEN0(4) | PKT_ID1(CMD_HS) | PKT_LEN1(0) |
		PKT_ID2(CMD_EOT) | PKT_LEN2(0) | PKT_LP,
	0,
	PKT_ID0(CMD_NULL) | PKT_LEN0(4) | PKT_ID1(CMD_HS) | PKT_LEN1(0) |
		PKT_ID2(CMD_EOT) | PKT_LEN2(0) | PKT_LP,
	PKT_ID0(CMD_BLNK) | PKT_LEN0(4) | PKT_ID1(CMD_HS) | PKT_LEN1(0) |
		PKT_ID2(CMD_BLNK) | PKT_LEN2(2),
	PKT_ID3(CMD_RGB) | PKT_LEN3(3) | PKT_ID4(CMD_EOT) | PKT_LEN4(0),
	PKT_ID0(CMD_NULL) | PKT_LEN0(4) | PKT_ID1(CMD_HS) | PKT_LEN1(0) |
		PKT_ID2(CMD_EOT) | PKT_LEN2(0) | PKT_LP,
	0,
	PKT_ID0(CMD_BLNK) | PKT_LEN0(4) | PKT_ID1(CMD_HS) | PKT_LEN1(0) |
		PKT_ID2(CMD_BLNK) | PKT_LEN2(2),
	PKT_ID3(CMD_RGB) | PKT_LEN3(3) | PKT_ID4(CMD_EOT) | PKT_LEN4(0),
};

/* TODO: verify with hw about this format */
const u32 dsi_pkt_seq_cmd_mode [NUMOF_PKT_SEQ] = {
	0,
	0,
	0,
	0,
	0,
	0,
	PKT_ID0(CMD_LONGW) | PKT_LEN0(3) | PKT_ID1(CMD_EOT) | PKT_LEN1(7),
	0,
	0,
	0,
	PKT_ID0(CMD_LONGW) | PKT_LEN0(3) | PKT_ID1(CMD_EOT) | PKT_LEN1(7),
	0,
};

const u32 init_reg[] = {
	DSI_WR_DATA,
	DSI_INT_ENABLE,
	DSI_INT_STATUS,
	DSI_INT_MASK,
	DSI_INIT_SEQ_DATA_0,
	DSI_INIT_SEQ_DATA_1,
	DSI_INIT_SEQ_DATA_2,
	DSI_INIT_SEQ_DATA_3,
	DSI_DCS_CMDS,
	DSI_PKT_SEQ_0_LO,
	DSI_PKT_SEQ_1_LO,
	DSI_PKT_SEQ_2_LO,
	DSI_PKT_SEQ_3_LO,
	DSI_PKT_SEQ_4_LO,
	DSI_PKT_SEQ_5_LO,
	DSI_PKT_SEQ_0_HI,
	DSI_PKT_SEQ_1_HI,
	DSI_PKT_SEQ_2_HI,
	DSI_PKT_SEQ_3_HI,
	DSI_PKT_SEQ_4_HI,
	DSI_PKT_SEQ_5_HI,
	DSI_CONTROL,
	DSI_HOST_DSI_CONTROL,
	DSI_PAD_CONTROL,
	DSI_PAD_CONTROL_CD,
	DSI_SOL_DELAY,
	DSI_MAX_THRESHOLD,
	DSI_TRIGGER,
	DSI_TX_CRC,
	DSI_INIT_SEQ_CONTROL,
	DSI_PKT_LEN_0_1,
	DSI_PKT_LEN_2_3,
	DSI_PKT_LEN_4_5,
	DSI_PKT_LEN_6_7,
};

static inline unsigned long tegra_dsi_readl(struct tegra_dc_dsi_data *dsi,
									u32 reg)
{
	return readl(dsi->base + reg * 4);
}

static inline void tegra_dsi_writel(struct tegra_dc_dsi_data *dsi,u32 val,
									u32 reg)
{
	writel(val, dsi->base + reg * 4);
}

static u32 tegra_dsi_get_hs_clk_rate(struct tegra_dc_dsi_data *dsi)
{
	u32 dsi_clock_rate_khz;

	switch (dsi->info.video_burst_mode) {
	case TEGRA_DSI_VIDEO_BURST_MODE_LOW_SPEED:
	case TEGRA_DSI_VIDEO_BURST_MODE_MEDIUM_SPEED:
	case TEGRA_DSI_VIDEO_BURST_MODE_FAST_SPEED:
	case TEGRA_DSI_VIDEO_BURST_MODE_FASTEST_SPEED:
		/* TODO: implement algo for these speed rate */

	case TEGRA_DSI_VIDEO_BURST_MODE_MANUAL:
		if (dsi->info.burst_mode_freq_khz) {
			dsi_clock_rate_khz = dsi->info.burst_mode_freq_khz;
			break;
		}
	case TEGRA_DSI_VIDEO_NONE_BURST_MODE:
	case TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END:
	case TEGRA_DSI_VIDEO_BURST_MODE_LOWEST_SPEED:
	default:
		dsi_clock_rate_khz = dsi->default_hs_clk_khz;
		break;
	}

	return dsi_clock_rate_khz;
}

static u32 tegra_dsi_get_lp_clk_rate(struct tegra_dc_dsi_data *dsi)
{
	u32 dsi_clock_rate_khz;

	if (dsi->info.enable_hs_clock_on_lp_cmd_mode)
		if (dsi->info.hs_clk_in_lp_cmd_mode_freq_khz)
			dsi_clock_rate_khz =
				dsi->info.hs_clk_in_lp_cmd_mode_freq_khz;
		else
			dsi_clock_rate_khz = tegra_dsi_get_hs_clk_rate(dsi);
	else
		dsi_clock_rate_khz = dsi->info.lp_cmd_mode_freq_khz;

	return dsi_clock_rate_khz;
}

static void tegra_dsi_init_sw(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	u32 h_width_pixels;
	u32 v_width_lines;
	u32 pixel_clk_hz;
	u32 byte_clk_hz;

	switch (dsi->info.pixel_format) {
		case TEGRA_DSI_PIXEL_FORMAT_16BIT_P:
			/* 2 bytes per pixel */
			dsi->pixel_scaler_mul = 2;
			dsi->pixel_scaler_div = 1;
			break;
		case TEGRA_DSI_PIXEL_FORMAT_18BIT_P:
			/* 2.25 bytes per pixel */
			dsi->pixel_scaler_mul = 9;
			dsi->pixel_scaler_div = 4;
			break;
		case TEGRA_DSI_PIXEL_FORMAT_18BIT_NP:
		case TEGRA_DSI_PIXEL_FORMAT_24BIT_P:
			/* 3 bytes per pixel */
			dsi->pixel_scaler_mul = 3;
			dsi->pixel_scaler_div = 1;
			break;
		default:
			break;
	}

	h_width_pixels = dc->mode.h_back_porch + dc->mode.h_front_porch +
			dc->mode.h_sync_width + dc->mode.h_active;
	v_width_lines = dc->mode.v_back_porch + dc->mode.v_front_porch +
			dc->mode.v_sync_width + dc->mode.v_active;

	/* The slowest pixel rate that is required */
	/*   for the given display timing 	*/
	pixel_clk_hz = h_width_pixels * v_width_lines * dsi->info.refresh_rate;

	/* Pixel byte rate on DSI interface */
	byte_clk_hz	= (pixel_clk_hz * dsi->pixel_scaler_mul) /
			(dsi->pixel_scaler_div * dsi->info.n_data_lanes);

	dsi->default_pixel_clk_khz = pixel_clk_hz / 1000;

	printk("dsi: default pixel rate %d khz\n", dsi->default_pixel_clk_khz);

	/*
	 * Pixel bit rate on DSI. Since DSI interface is double data rate (
	 * transferring data on both rising and falling edge of clk), div by 2
	 * to get the actual clock rate.
	 */
	dsi->default_hs_clk_khz =
			(byte_clk_hz * NUMOF_BIT_PER_BYTE) / (1000 * 2);

	dsi->dsi_control_val =
			DSI_CONTROL_VIRTUAL_CHANNEL(dsi->info.virtual_channel) |
			DSI_CONTROL_NUM_DATA_LANES(dsi->info.n_data_lanes - 1) |
			DSI_CONTROL_VID_SOURCE(dsi->controller_index) |
			DSI_CONTROL_DATA_FORMAT(dsi->info.pixel_format);

	dsi->target_lp_clk_khz = tegra_dsi_get_lp_clk_rate(dsi);
	dsi->target_hs_clk_khz = tegra_dsi_get_hs_clk_rate(dsi);

	/*
	 * Force video clock to be continuous mode if
	 * enable_hs_clock_on_lp_cmd_mode is set
	 */
	if (dsi->info.enable_hs_clock_on_lp_cmd_mode) {
		if (dsi->info.video_clock_mode != TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS)
			printk("Force to clock continuous mode\n");

		dsi->info.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS;
	}

}

static void tegra_dsi_get_phy_timing(struct tegra_dc_dsi_data *dsi,
				struct dsi_phy_timing_inclk *phy_timing_clk,
				u32 clk_ns)
{

	phy_timing_clk->t_hsdexit = dsi->info.phy_timing.t_hsdexit_ns ?
			(dsi->info.phy_timing.t_hsdexit_ns / clk_ns) :
			(T_HSEXIT_DEFAULT(clk_ns));

	phy_timing_clk->t_hstrail = dsi->info.phy_timing.t_hstrail_ns ?
			(dsi->info.phy_timing.t_hstrail_ns / clk_ns) :
			(T_HSTRAIL_DEFAULT(clk_ns));

	phy_timing_clk->t_datzero = dsi->info.phy_timing.t_datzero_ns ?
			(dsi->info.phy_timing.t_datzero_ns / clk_ns) :
			(T_DATZERO_DEFAULT(clk_ns));

	phy_timing_clk->t_hsprepr = dsi->info.phy_timing.t_hsprepr_ns ?
			(dsi->info.phy_timing.t_hsprepr_ns / clk_ns) :
			(T_HSPREPR_DEFAULT(clk_ns));

	phy_timing_clk->t_clktrail = dsi->info.phy_timing.t_clktrail_ns ?
				(dsi->info.phy_timing.t_clktrail_ns / clk_ns) :
				(T_CLKTRAIL_DEFAULT(clk_ns));

	phy_timing_clk->t_clkpost = dsi->info.phy_timing.t_clkpost_ns ?
				(dsi->info.phy_timing.t_clkpost_ns / clk_ns) :
				(T_CLKPOST_DEFAULT(clk_ns));

	phy_timing_clk->t_clkzero = dsi->info.phy_timing.t_clkzero_ns ?
				(dsi->info.phy_timing.t_clkzero_ns / clk_ns) :
				(T_CLKZERO_DEFAULT(clk_ns));

	phy_timing_clk->t_tlpx = dsi->info.phy_timing.t_tlpx_ns ?
				(dsi->info.phy_timing.t_tlpx_ns / clk_ns) :
				(T_TLPX_DEFAULT(clk_ns));

	phy_timing_clk->t_clkpre = T_CLKPRE_DEFAULT(clk_ns);
	phy_timing_clk->t_clkprepare = T_CLKPREPARE_DEFAULT(clk_ns);
	phy_timing_clk->t_wakeup = T_WAKEUP_DEFAULT(clk_ns);

	phy_timing_clk->t_taget = 5 * phy_timing_clk->t_tlpx;
	phy_timing_clk->t_tasure = 2 * phy_timing_clk->t_tlpx;
	phy_timing_clk->t_tago = 4 * phy_timing_clk->t_tlpx;
}

static void tegra_dsi_set_phy_timing(struct tegra_dc_dsi_data *dsi)
{
	u32 val;
	struct dsi_phy_timing_inclk	phy_timing;

	tegra_dsi_get_phy_timing(dsi, &phy_timing, dsi->current_bit_clk_ns);

	val = DSI_PHY_TIMING_0_THSDEXIT(phy_timing.t_hsdexit) |
			DSI_PHY_TIMING_0_THSTRAIL(phy_timing.t_hstrail) |
			DSI_PHY_TIMING_0_TDATZERO(phy_timing.t_datzero) |
			DSI_PHY_TIMING_0_THSPREPR(phy_timing.t_hsprepr);
	tegra_dsi_writel(dsi, val, DSI_PHY_TIMING_0);

	val = DSI_PHY_TIMING_1_TCLKTRAIL(phy_timing.t_clktrail) |
			DSI_PHY_TIMING_1_TCLKPOST(phy_timing.t_clkpost) |
			DSI_PHY_TIMING_1_TCLKZERO(phy_timing.t_clkzero) |
			DSI_PHY_TIMING_1_TTLPX(phy_timing.t_tlpx);
	tegra_dsi_writel(dsi, val, DSI_PHY_TIMING_1);

	val = DSI_PHY_TIMING_2_TCLKPREPARE(phy_timing.t_clkprepare) |
                DSI_PHY_TIMING_2_TCLKPRE(phy_timing.t_clkpre) |
			DSI_PHY_TIMING_2_TWAKEUP(phy_timing.t_wakeup);
	tegra_dsi_writel(dsi, val, DSI_PHY_TIMING_2);

	val = DSI_BTA_TIMING_TTAGET(phy_timing.t_taget) |
			DSI_BTA_TIMING_TTASURE(phy_timing.t_tasure) |
			DSI_BTA_TIMING_TTAGO(phy_timing.t_tago);
	tegra_dsi_writel(dsi, val, DSI_BTA_TIMING);
}

static u32 tegra_dsi_sol_delay_burst(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	u32 dsi_to_pixel_clk_ratio;
	u32 temp;
	u32 temp1;
	u32 mipi_clk_adj_kHz;
	u32 sol_delay;
	struct tegra_dc_mode *dc_modes = &dc->mode;

	/* Get Fdsi/Fpixel ration (note: Fdsi si in bit format) */
	dsi_to_pixel_clk_ratio = (dsi->current_dsi_clk_khz * 2 +
		dsi->default_pixel_clk_khz - 1) / dsi->default_pixel_clk_khz;

	/* Convert Fdsi to byte format */
	dsi_to_pixel_clk_ratio *= 1000/8;

	/* Multiplying by 1000 so that we don't loose the fraction part */
	temp = dc_modes->h_active * 1000;
	temp1 = dc_modes->h_active + dc_modes->h_back_porch +
			dc_modes->h_sync_width;

	sol_delay = temp1 * dsi_to_pixel_clk_ratio -
			temp * dsi->pixel_scaler_mul /
			(dsi->pixel_scaler_div * dsi->info.n_data_lanes);

	/* Do rounding on sol delay */
	sol_delay = (sol_delay + 1000 - 1)/1000;

	/* TODO:
	 * 1. find out the correct sol fifo depth to use
	 * 2. verify with hw about the clamping function
	 */
	if (sol_delay > (480 * 4)) {
		sol_delay = (480 * 4);
		mipi_clk_adj_kHz = sol_delay +
			(dc_modes->h_active * dsi->pixel_scaler_mul) /
			(dsi->info.n_data_lanes * dsi->pixel_scaler_div);

		mipi_clk_adj_kHz *= (dsi->default_pixel_clk_khz / temp1);

		mipi_clk_adj_kHz *= 4;
	}

	dsi->target_hs_clk_khz = mipi_clk_adj_kHz;

	return sol_delay;
}

static void tegra_dsi_set_sol_delay(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	u32 sol_delay;

	if (dsi->info.video_burst_mode == TEGRA_DSI_VIDEO_NONE_BURST_MODE ||
		dsi->info.video_burst_mode ==
				TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END) {
		sol_delay = NUMOF_BIT_PER_BYTE * dsi->pixel_scaler_mul /
			(dsi->pixel_scaler_div * dsi->info.n_data_lanes);
		dsi->status.clk_burst = DSI_CLK_BURST_NONE_BURST;
	} else {
		sol_delay = tegra_dsi_sol_delay_burst(dc, dsi);
		dsi->status.clk_burst = DSI_CLK_BURST_BURST_MODE;
	}

	tegra_dsi_writel(dsi, DSI_SOL_DELAY_SOL_DELAY(sol_delay),
								DSI_SOL_DELAY);
}

static void tegra_dsi_set_timeout(struct tegra_dc_dsi_data *dsi)
{
	u32 val;
	u32 bytes_per_frame;
	u32 timeout = 0;

	/* TODO: verify the following eq */
	bytes_per_frame = dsi->current_dsi_clk_khz * 1000 * 2 /
						(dsi->info.refresh_rate * 8);
	timeout = bytes_per_frame / DSI_CYCLE_COUNTER_VALUE;
	timeout = (timeout + DSI_HTX_TO_MARGIN) & 0xffff;

	val = DSI_TIMEOUT_0_LRXH_TO(DSI_LRXH_TO_VALUE) |
			DSI_TIMEOUT_0_HTX_TO(timeout);
	tegra_dsi_writel(dsi, val, DSI_TIMEOUT_0);

	if (dsi->info.panel_reset_timeout_msec)
		timeout = (dsi->info.panel_reset_timeout_msec * 1000*1000)
					/ dsi->current_bit_clk_ns;
	else
		timeout = DSI_PR_TO_VALUE;

	val = DSI_TIMEOUT_1_PR_TO(timeout) |
		DSI_TIMEOUT_1_TA_TO(DSI_TA_TO_VALUE);
	tegra_dsi_writel(dsi, val, DSI_TIMEOUT_1);

	val = DSI_TO_TALLY_P_RESET_STATUS(IN_RESET) |
		DSI_TO_TALLY_TA_TALLY(DSI_TA_TALLY_VALUE)|
		DSI_TO_TALLY_LRXH_TALLY(DSI_LRXH_TALLY_VALUE)|
		DSI_TO_TALLY_HTX_TALLY(DSI_HTX_TALLY_VALUE);
	tegra_dsi_writel(dsi, val, DSI_TO_TALLY);
}

static void tegra_dsi_setup_video_mode_pkt_length(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	u32 val;
	u32 hact_pkt_len;
	u32 hsa_pkt_len;
	u32 hbp_pkt_len;
	u32 hfp_pkt_len;

	hact_pkt_len = dc->mode.h_active * dsi->pixel_scaler_mul /
							dsi->pixel_scaler_div;
	hsa_pkt_len = dc->mode.h_sync_width * dsi->pixel_scaler_mul /
							dsi->pixel_scaler_div;
	hbp_pkt_len = dc->mode.h_back_porch * dsi->pixel_scaler_mul /
							dsi->pixel_scaler_div;
	hfp_pkt_len = dc->mode.h_front_porch * dsi->pixel_scaler_mul /
							dsi->pixel_scaler_div;

	if (dsi->info.video_burst_mode !=
				TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END)
		hbp_pkt_len += hsa_pkt_len;

	hsa_pkt_len -= DSI_HSYNC_BLNK_PKT_OVERHEAD;
	hbp_pkt_len -= DSI_HBACK_PORCH_PKT_OVERHEAD;
	hfp_pkt_len -= DSI_HFRONT_PORCH_PKT_OVERHEAD;

	val = DSI_PKT_LEN_0_1_LENGTH_0(0) | DSI_PKT_LEN_0_1_LENGTH_1(hsa_pkt_len);
	tegra_dsi_writel(dsi, val, DSI_PKT_LEN_0_1);

	val = DSI_PKT_LEN_2_3_LENGTH_2(hbp_pkt_len) |
			DSI_PKT_LEN_2_3_LENGTH_3(hact_pkt_len);
	tegra_dsi_writel(dsi, val, DSI_PKT_LEN_2_3);

	val = DSI_PKT_LEN_4_5_LENGTH_4(hfp_pkt_len) | DSI_PKT_LEN_4_5_LENGTH_5(0);
	tegra_dsi_writel(dsi, val, DSI_PKT_LEN_4_5);

	val = DSI_PKT_LEN_6_7_LENGTH_6(0) | DSI_PKT_LEN_6_7_LENGTH_7(0);
	tegra_dsi_writel(dsi, val, DSI_PKT_LEN_6_7);
}

static void tegra_dsi_setup_cmd_mode_pkt_length(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	unsigned long	val;
	unsigned long	act_bytes;

	act_bytes = dc->mode.h_active * dsi->pixel_scaler_mul /
			dsi->pixel_scaler_div + 1;

	val = DSI_PKT_LEN_0_1_LENGTH_0(0) | DSI_PKT_LEN_0_1_LENGTH_1(0);
	tegra_dsi_writel(dsi, val, DSI_PKT_LEN_0_1);

	val = DSI_PKT_LEN_2_3_LENGTH_2(0) | DSI_PKT_LEN_2_3_LENGTH_3(act_bytes);
	tegra_dsi_writel(dsi, val, DSI_PKT_LEN_2_3);

	val = DSI_PKT_LEN_4_5_LENGTH_4(0) | DSI_PKT_LEN_4_5_LENGTH_5(act_bytes);
	tegra_dsi_writel(dsi, val, DSI_PKT_LEN_4_5);

	val = DSI_PKT_LEN_6_7_LENGTH_6(0) | DSI_PKT_LEN_6_7_LENGTH_7(0x0f0f);
	tegra_dsi_writel(dsi, val, DSI_PKT_LEN_6_7);
}

static void tegra_dsi_set_pkt_length(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	if (dsi->driven_mode == TEGRA_DSI_DRIVEN_BY_HOST)
		return;

	if (dsi->info.video_data_type == TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE)
		tegra_dsi_setup_video_mode_pkt_length(dc, dsi);
	else
		tegra_dsi_setup_cmd_mode_pkt_length(dc, dsi);
}

static void tegra_dsi_set_pkt_seq(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	const u32 *pkt_seq;
	u32 rgb_info;
	u32 pkt_seq_3_5_rgb_lo;
	u32 pkt_seq_3_5_rgb_hi;
	u32	val;
	u32 reg;
	u8  i;

	if (dsi->driven_mode == TEGRA_DSI_DRIVEN_BY_HOST)
		return;

	switch(dsi->info.pixel_format) {
	case TEGRA_DSI_PIXEL_FORMAT_16BIT_P:
		rgb_info = CMD_RGB_16BPP;
		break;
	case TEGRA_DSI_PIXEL_FORMAT_18BIT_P:
		rgb_info = CMD_RGB_18BPP;
		break;
	case TEGRA_DSI_PIXEL_FORMAT_18BIT_NP:
		rgb_info = CMD_RGB_18BPPNP;
		break;
	case TEGRA_DSI_PIXEL_FORMAT_24BIT_P:
	default:
		rgb_info = CMD_RGB_24BPP;
		break;
	}

	pkt_seq_3_5_rgb_lo = 0;
	pkt_seq_3_5_rgb_hi = 0;
	if (dsi->info.video_data_type == TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE)
		pkt_seq = dsi_pkt_seq_cmd_mode;
	else {
		switch (dsi->info.video_burst_mode) {
		case TEGRA_DSI_VIDEO_BURST_MODE_LOWEST_SPEED:
		case TEGRA_DSI_VIDEO_BURST_MODE_LOW_SPEED:
		case TEGRA_DSI_VIDEO_BURST_MODE_MEDIUM_SPEED:
		case TEGRA_DSI_VIDEO_BURST_MODE_FAST_SPEED:
		case TEGRA_DSI_VIDEO_BURST_MODE_FASTEST_SPEED:
		case TEGRA_DSI_VIDEO_BURST_MODE_MANUAL:
			pkt_seq_3_5_rgb_hi = DSI_PKT_SEQ_3_HI_PKT_33_ID(rgb_info);
			pkt_seq = dsi_pkt_seq_video_burst;
			break;
		case TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END:
			pkt_seq_3_5_rgb_hi = DSI_PKT_SEQ_3_HI_PKT_34_ID(rgb_info);
			pkt_seq = dsi_pkt_seq_video_non_burst_syne;
			break;
		case TEGRA_DSI_VIDEO_NONE_BURST_MODE:
		default:
			pkt_seq_3_5_rgb_lo = DSI_PKT_SEQ_3_LO_PKT_32_ID(rgb_info);
			pkt_seq = dsi_pkt_seq_video_non_burst;
			break;
		}
	}

	for (i = 0; i < NUMOF_PKT_SEQ; i++) {
		val = pkt_seq[i];
		reg = dsi_pkt_seq_reg[i];
		if ((reg == DSI_PKT_SEQ_3_LO) || (reg == DSI_PKT_SEQ_5_LO))
			val |= pkt_seq_3_5_rgb_lo;
		if ((reg == DSI_PKT_SEQ_3_HI) || (reg == DSI_PKT_SEQ_5_HI))
			val |= pkt_seq_3_5_rgb_hi;
		tegra_dsi_writel(dsi, val, reg);
	}
}

static void tegra_dsi_stop_dc_stream(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	/*
	 * TODO: It is possible that we are in the middle of video stream,
	 * Add code to wait for vsync and then stop DC from sending data to dsi
	 */
	tegra_dc_writel(dc, 0, DC_DISP_DISP_WIN_OPTIONS);
}

static void tegra_dsi_start_dc_stream(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	u32 val;
	tegra_dc_writel(dc, DSI_ENABLE, DC_DISP_DISP_WIN_OPTIONS);

	/* TODO: clean up */
	val = PIN_INPUT_LSPI_INPUT_EN;
	tegra_dc_writel(dc, val, DC_COM_PIN_INPUT_ENABLE3);

	val = PIN_OUTPUT_LSPI_OUTPUT_DIS;
	tegra_dc_writel(dc, val, DC_COM_PIN_OUTPUT_ENABLE3);

	tegra_dc_writel(dc, PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE |
			PW4_ENABLE | PM0_ENABLE | PM1_ENABLE,
			DC_CMD_DISPLAY_POWER_CONTROL);

	val = MSF_POLARITY_HIGH | MSF_ENABLE | MSF_LSPI;
	tegra_dc_writel(dc, val, DC_CMD_DISPLAY_COMMAND_OPTION0);


	/* TODO: using continuous video mode for now */
	/* if (dsi->info.panel_has_frame_buffer) {*/
	if (0) {
		tegra_dc_writel(dc, DISP_CTRL_MODE_NC_DISPLAY, DC_CMD_DISPLAY_COMMAND);
		tegra_dc_writel(dc, GENERAL_UPDATE, DC_CMD_STATE_CONTROL);
		val = GENERAL_ACT_REQ | NC_HOST_TRIG;
		tegra_dc_writel(dc, val, DC_CMD_STATE_CONTROL);
	} else {
		tegra_dc_writel(dc, DISP_CTRL_MODE_C_DISPLAY, DC_CMD_DISPLAY_COMMAND);
		tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
		tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	}
}

static void tegra_dsi_set_dc_clk(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	u32 shift_clk_div;
	u32 val;

	if (dsi->info.video_burst_mode == TEGRA_DSI_VIDEO_NONE_BURST_MODE ||
		dsi->info.video_burst_mode ==
				TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END)
		shift_clk_div = NUMOF_BIT_PER_BYTE * dsi->pixel_scaler_mul /
			(dsi->pixel_scaler_div * dsi->info.n_data_lanes) - 2;
	else
		shift_clk_div = (dsi->current_dsi_clk_khz * 2 +
						dsi->default_hs_clk_khz - 1) /
						(dsi->default_hs_clk_khz) - 2;

#ifdef CONFIG_TEGRA_FPGA_PLATFORM
	shift_clk_div = 1;
#endif

	/* TODO: find out if PCD3 option is required */
	val = PIXEL_CLK_DIVIDER_PCD1 | SHIFT_CLK_DIVIDER(shift_clk_div);
	tegra_dc_writel(dc, val, DC_DISP_DISP_CLOCK_CONTROL);

	clk_enable(dsi->dc_clk);
}

static void tegra_dsi_set_dsi_clk(struct tegra_dc *dc,
					struct tegra_dc_dsi_data *dsi, u32 clk)
{
	u32 rm;

	rm = clk % 1000;
	if (rm != 0)
		clk -= rm;

	clk *= 2; 	/* Value for PLLD routine is required to be twice as */
                        /* the desired clock rate */

	dc->mode.pclk = clk*1000;
	tegra_dc_setup_clk(dc, dsi->dsi_clk);
	clk_enable(dsi->dsi_clk);
	tegra_periph_reset_deassert(dsi->dsi_clk);

	dsi->current_dsi_clk_khz = clk_get_rate(dsi->dsi_clk) / 1000;

	dsi->current_bit_clk_ns =  1000*1000 / (dsi->current_dsi_clk_khz * 2);
}

static void tegra_dsi_hs_clk_out_enable(struct tegra_dc_dsi_data *dsi)
{
	u32 val;

	val = tegra_dsi_readl(dsi, DSI_CONTROL);
	val &= ~DSI_CONTROL_HS_CLK_CTRL(1);

	if (dsi->info.video_clock_mode == TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS) {
		val |= DSI_CONTROL_HS_CLK_CTRL(CONTINUOUS);
		dsi->status.clk_mode = DSI_PHYCLK_CONTINUOUS;
	} else {
		val |= DSI_CONTROL_HS_CLK_CTRL(TX_ONLY);
		dsi->status.clk_mode = DSI_PHYCLK_TX_ONLY;
	}
	tegra_dsi_writel(dsi, val, DSI_CONTROL);

	val = tegra_dsi_readl(dsi, DSI_HOST_DSI_CONTROL);
	val &= ~DSI_HOST_DSI_CONTROL_HIGH_SPEED_TRANS(1);
	val |= DSI_HOST_DSI_CONTROL_HIGH_SPEED_TRANS(TEGRA_DSI_HIGH);
	tegra_dsi_writel(dsi, val, DSI_HOST_DSI_CONTROL);

	dsi->status.clk_out = DSI_PHYCLK_OUT_EN;
}

static void tegra_dsi_hs_clk_out_enable_in_lp(struct tegra_dc_dsi_data *dsi)
{
	u32 val;
	tegra_dsi_hs_clk_out_enable(dsi);

	val = tegra_dsi_readl(dsi, DSI_HOST_DSI_CONTROL);
	val &= ~DSI_HOST_DSI_CONTROL_HIGH_SPEED_TRANS(1);
	val |= DSI_HOST_DSI_CONTROL_HIGH_SPEED_TRANS(TEGRA_DSI_LOW);
	tegra_dsi_writel(dsi, val, DSI_HOST_DSI_CONTROL);
}

static void tegra_dsi_hs_clk_out_disable(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	u32 val;

	if (dsi->status.driven == DSI_DRIVEN_MODE_DC)
		tegra_dsi_stop_dc_stream(dc, dsi);

	val = tegra_dsi_readl(dsi, DSI_CONTROL);
	val &= ~DSI_CONTROL_HS_CLK_CTRL(1);
	val |= DSI_CONTROL_HS_CLK_CTRL(TX_ONLY);
	tegra_dsi_writel(dsi, val, DSI_CONTROL);

	/* TODO: issue a cmd */

	val = tegra_dsi_readl(dsi, DSI_HOST_DSI_CONTROL);
	val &= ~DSI_HOST_DSI_CONTROL_HIGH_SPEED_TRANS(1);
	val |= DSI_HOST_DSI_CONTROL_HIGH_SPEED_TRANS(TEGRA_DSI_LOW);
	tegra_dsi_writel(dsi, val, DSI_HOST_DSI_CONTROL);

	dsi->status.clk_mode = DSI_PHYCLK_NOT_INIT;
	dsi->status.clk_out = DSI_PHYCLK_OUT_DIS;
}

static void tegra_dsi_set_control_reg_lp(struct tegra_dc_dsi_data *dsi)
{
	u32 dsi_control;
	u32 host_dsi_control;
	u32 max_threshold;

	dsi_control = dsi->dsi_control_val | DSI_CTRL_HOST_DRIVEN;
	host_dsi_control = HOST_DSI_CTRL_COMMON |
			HOST_DSI_CTRL_HOST_DRIVEN |
			DSI_HOST_DSI_CONTROL_HIGH_SPEED_TRANS(TEGRA_DSI_LOW);
	max_threshold = DSI_MAX_THRESHOLD_MAX_THRESHOLD(DSI_HOST_FIFO_DEPTH);

	tegra_dsi_writel(dsi, max_threshold, DSI_MAX_THRESHOLD);
	tegra_dsi_writel(dsi, dsi_control, DSI_CONTROL);
	tegra_dsi_writel(dsi, host_dsi_control, DSI_HOST_DSI_CONTROL);

	dsi->status.driven = DSI_DRIVEN_MODE_HOST;
	dsi->status.clk_burst = DSI_CLK_BURST_NOT_INIT;
	dsi->status.vtype = DSI_VIDEO_TYPE_NOT_INIT;
}

static void tegra_dsi_set_control_reg_hs(struct tegra_dc_dsi_data *dsi)
{
	u32 dsi_control;
	u32 host_dsi_control;
	u32 max_threshold;
	u32 dcs_cmd;

	dsi_control = dsi->dsi_control_val;
	host_dsi_control = HOST_DSI_CTRL_COMMON;
	max_threshold = 0;
	dcs_cmd = 0;

	if (dsi->driven_mode == TEGRA_DSI_DRIVEN_BY_HOST) {
		dsi_control |= DSI_CTRL_HOST_DRIVEN;
		host_dsi_control |= HOST_DSI_CTRL_HOST_DRIVEN;
		max_threshold = DSI_MAX_THRESHOLD_MAX_THRESHOLD(DSI_HOST_FIFO_DEPTH);
		dsi->status.driven = DSI_DRIVEN_MODE_HOST;
	} else {
		dsi_control |= DSI_CTRL_DC_DRIVEN;
		host_dsi_control |= HOST_DSI_CTRL_DC_DRIVEN;
		max_threshold = DSI_MAX_THRESHOLD_MAX_THRESHOLD(DSI_VIDEO_FIFO_DEPTH);
		dsi->status.driven = DSI_DRIVEN_MODE_DC;
	}

	if (dsi->info.video_data_type == TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE) {
		dsi_control |= DSI_CTRL_CMD_MODE;
		host_dsi_control |= HOST_DSI_CTRL_CMD_MODE;
		dcs_cmd = DSI_DCS_CMDS_LT5_DCS_CMD(DSI_WRITE_MEMORY_START)|
				DSI_DCS_CMDS_LT3_DCS_CMD(DSI_WRITE_MEMORY_CONTINUE);
		dsi->status.vtype = DSI_VIDEO_TYPE_CMD_MODE;

	} else {
		dsi_control |= DSI_CTRL_VIDEO_MODE;
		host_dsi_control |= HOST_DSI_CTRL_VIDEO_MODE;
		dsi->status.vtype = DSI_VIDEO_TYPE_VIDEO_MODE;
	}

	tegra_dsi_writel(dsi, max_threshold, DSI_MAX_THRESHOLD);
	tegra_dsi_writel(dsi, dcs_cmd, DSI_DCS_CMDS);
	tegra_dsi_writel(dsi, dsi_control, DSI_CONTROL);
	tegra_dsi_writel(dsi, host_dsi_control, DSI_HOST_DSI_CONTROL);
}

static int tegra_dsi_init_hw(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	u32 val;
	u32 i;
	int err;

	tegra_dsi_set_dsi_clk(dc, dsi, dsi->target_lp_clk_khz);

	/* TODO: only need to change the timing for bta */
	tegra_dsi_set_phy_timing(dsi);

	err = gpio_request(TEGRA_GPIO_PJ1, "DSI TE");
	if (err < 0)
		goto fail;

	err = gpio_direction_input(TEGRA_GPIO_PJ1);
	if (err < 0) {
		gpio_free(TEGRA_GPIO_PJ1);
		goto fail;
	}
	tegra_gpio_enable(TEGRA_GPIO_PJ1);

	if (dsi->status.driven == DSI_DRIVEN_MODE_DC)
		tegra_dsi_stop_dc_stream(dc, dsi);

	/* Initializing DSI registers */
	for (i = 0; i < ARRAY_SIZE(init_reg); i++) {
		tegra_dsi_writel(dsi, 0, init_reg[i]);
	}
	tegra_dsi_writel(dsi, dsi->dsi_control_val, DSI_CONTROL);

	val = DSI_PAD_CONTROL_PAD_PDIO(0) |
			DSI_PAD_CONTROL_PAD_PDIO_CLK(0) |
			DSI_PAD_CONTROL_PAD_PULLDN_ENAB(TEGRA_DSI_DISABLE);
	tegra_dsi_writel(dsi, val, DSI_PAD_CONTROL);

	val = DSI_POWER_CONTROL_LEG_DSI_ENABLE(TEGRA_DSI_ENABLE);
	tegra_dsi_writel(dsi, val, DSI_POWER_CONTROL);

	while (tegra_dsi_readl(dsi, DSI_POWER_CONTROL) != val) {
		tegra_dsi_writel(dsi, val, DSI_POWER_CONTROL);
	}

	dsi->status.init = DSI_MODULE_INIT;
	dsi->status.lphs = DSI_LPHS_NOT_INIT;
	dsi->status.vtype = DSI_VIDEO_TYPE_NOT_INIT;
	dsi->status.driven = DSI_DRIVEN_MODE_NOT_INIT;
	dsi->status.clk_out = DSI_PHYCLK_OUT_DIS;
	dsi->status.clk_mode = DSI_PHYCLK_NOT_INIT;
	dsi->status.clk_burst = DSI_CLK_BURST_NOT_INIT;
fail:
	return err;
}

static int tegra_dsi_set_to_lp_mode(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	int 	err;

	if (dsi->status.init != DSI_MODULE_INIT) {
		err = -EPERM;
		goto fail;
	}

	if (dsi->status.lphs == DSI_LPHS_IN_LP_MODE)
		goto success;

	if (dsi->status.driven == DSI_DRIVEN_MODE_DC)
		tegra_dsi_stop_dc_stream(dc, dsi);

	/* disable/enable hs clock according to enable_hs_clock_on_lp_cmd_mode */
	if ((dsi->status.clk_out == DSI_PHYCLK_OUT_EN) &&
		(!dsi->info.enable_hs_clock_on_lp_cmd_mode))
		tegra_dsi_hs_clk_out_disable(dc, dsi);

	if (dsi->current_dsi_clk_khz != dsi->target_lp_clk_khz){
		tegra_dsi_set_dsi_clk(dc, dsi, dsi->target_lp_clk_khz);
		tegra_dsi_set_timeout(dsi);
	}

	tegra_dsi_set_control_reg_lp(dsi);

	if ((dsi->status.clk_out == DSI_PHYCLK_OUT_DIS) &&
		(dsi->info.enable_hs_clock_on_lp_cmd_mode))
		tegra_dsi_hs_clk_out_enable_in_lp(dsi);

success:
	dsi->status.lphs = DSI_LPHS_IN_LP_MODE;
	err = 0;
fail:
	return err;
}

static int tegra_dsi_set_to_hs_mode(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	int err;

	if (dsi->status.init != DSI_MODULE_INIT) {
		err = -EPERM;
		goto fail;
	}

	if (dsi->status.driven == DSI_DRIVEN_MODE_DC)
		tegra_dsi_stop_dc_stream(dc, dsi);

	if ((dsi->status.clk_out == DSI_PHYCLK_OUT_EN) &&
		(!dsi->info.enable_hs_clock_on_lp_cmd_mode))
		tegra_dsi_hs_clk_out_disable(dc, dsi);

	if (dsi->current_dsi_clk_khz != dsi->target_hs_clk_khz) {
		tegra_dsi_set_dsi_clk(dc, dsi, dsi->target_hs_clk_khz);
		tegra_dsi_set_timeout(dsi);
	}

	tegra_dsi_set_phy_timing(dsi);

	if (dsi->driven_mode == TEGRA_DSI_DRIVEN_BY_DC){
		tegra_dsi_set_pkt_seq(dc, dsi);
		tegra_dsi_set_pkt_length(dc, dsi);
		tegra_dsi_set_sol_delay(dc, dsi);
		tegra_dsi_set_dc_clk(dc, dsi);
	}

	tegra_dsi_set_control_reg_hs(dsi);

	if (dsi->status.clk_out == DSI_PHYCLK_OUT_DIS)
		tegra_dsi_hs_clk_out_enable(dsi);

	dsi->status.lphs = DSI_LPHS_IN_HS_MODE;
	err = 0;
fail:
	return err;
}

static bool tegra_dsi_is_controller_idle(struct tegra_dc_dsi_data *dsi)
{
	u32 timeout = 0;
	bool retVal;

	retVal = false;
	while (timeout <= DSI_MAX_COMMAND_DELAY_USEC) {
		if (!tegra_dsi_readl(dsi, DSI_TRIGGER)) {
			retVal = true;
			break;
		}
		udelay(DSI_COMMAND_DELAY_STEPS_USEC);
		timeout += DSI_COMMAND_DELAY_STEPS_USEC;
	}

	return retVal;
}

static bool tegra_dsi_host_trigger(struct tegra_dc_dsi_data *dsi)
{
	bool status;

	status = false;
	if (tegra_dsi_readl(dsi, DSI_TRIGGER))
		goto fail;

	tegra_dsi_writel(dsi, DSI_TRIGGER_HOST_TRIGGER(TEGRA_DSI_ENABLE),
								DSI_TRIGGER);

#if DSI_USE_SYNC_POINTS
	/* TODO: Implement sync point */
#else
	status = tegra_dsi_is_controller_idle(dsi);
#endif

fail:
	return status;
}

static int tegra_dsi_read_data(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	/* TODO: implement DSI read */
	return ENXIO;
}

static int tegra_dsi_write_data(struct tegra_dc *dc,
					struct tegra_dc_dsi_data *dsi,
					u8* pdata, u8 data_id, u16 data_len)
{
	bool switch_back_to_hs_mode;
	bool switch_back_to_dc_mode;
	u32 val;
	u8 *pval;
	int	err;
	u8  virtua_channel;

	err = 0;
	switch_back_to_hs_mode = false;
	switch_back_to_dc_mode = false;

	if ((dsi->status.init != DSI_MODULE_INIT) ||
		(dsi->status.lphs == DSI_LPHS_NOT_INIT)) {
		err = -EPERM;
		goto fail;
	}

	if (!tegra_dsi_is_controller_idle(dsi)) {
		err = -EBUSY;
		goto fail;
	}

	err = 0;

	if (dsi->status.lphs == DSI_LPHS_IN_HS_MODE) {
		if (dsi->info.hs_cmd_mode_supported) {
			if (dsi->status.driven == DSI_DRIVEN_MODE_DC) {
				dsi->driven_mode = TEGRA_DSI_DRIVEN_BY_HOST;
				tegra_dsi_set_to_hs_mode(dc, dsi);
				switch_back_to_dc_mode = true;
			}
		} else {
			tegra_dsi_set_to_lp_mode(dc, dsi);
			switch_back_to_hs_mode = true;
		}
	}

	virtua_channel = dsi->info.virtual_channel << DSI_VIR_CHANNEL_BIT_POSITION;

	/* always use hw for ecc */
	val = (virtua_channel | data_id) << 0 |
			data_len << 8;
	tegra_dsi_writel(dsi, val, DSI_WR_DATA);

	/* if pdata != NULL, pkt type is long pkt */
	if (pdata != NULL) {
		while (data_len) {
			if (data_len >= 4) {
				val = ((u32*) pdata)[0];
				data_len -= 4;
				pdata += 4;
			} else {
				val = 0;
				pval = (u8*) &val;
				do
					*pval++ = *pdata++;
				while(--data_len);
			}
			tegra_dsi_writel(dsi, val, DSI_WR_DATA);
		}
	}

	if (!tegra_dsi_host_trigger(dsi))
		err = -EIO;

	if (switch_back_to_dc_mode)
		dsi->driven_mode = TEGRA_DSI_DRIVEN_BY_DC;
	if (switch_back_to_dc_mode || switch_back_to_hs_mode)
		tegra_dsi_set_to_hs_mode(dc, dsi);

fail:
	return err;
}

static int tegra_dsi_init_panel(struct tegra_dc *dc,
						struct tegra_dc_dsi_data *dsi)
{
	u32	i;
	int err;

	err = 0;
	for (i = 0; i < dsi->info.n_init_cmd; i++) {
		struct tegra_dsi_cmd *cur_cmd;
		cur_cmd = &dsi->info.dsi_init_cmd[i];

		if (cur_cmd->cmd_type == TEGRA_DSI_DELAY_MS)
			mdelay(cur_cmd->sp_len_dly.delay_ms);
		else {
			err = tegra_dsi_write_data(dc, dsi,
						cur_cmd->pdata,
						cur_cmd->data_id,
						cur_cmd->sp_len_dly.data_len);
			if (err < 0)
				break;
		}
	}
	return err;
}

static void tegra_dc_dsi_enable(struct tegra_dc *dc)
{
	struct tegra_dc_dsi_data *dsi = tegra_dc_get_outdata(dc);
	int err;

	tegra_dc_io_start(dc);
	mutex_lock(&dsi->lock);

	err = tegra_dsi_init_hw(dc, dsi);
	if (err < 0) {
		dev_err(&dc->ndev->dev, "dsi: not able to init dsi hardware\n");
		return;
	}

	err = tegra_dsi_set_to_lp_mode(dc, dsi);
	if (err < 0) {
		dev_err(&dc->ndev->dev, "dsi: not able to set to lp mode\n");
		return;
	}

	err = tegra_dsi_init_panel(dc, dsi);
	if (err < 0) {
		dev_err(&dc->ndev->dev, "dsi: error while sending dsi cmd\n");
		return;
	}

	err = tegra_dsi_set_to_hs_mode(dc, dsi);
	if (err < 0) {
			dev_err(&dc->ndev->dev,
					"dsi: not able to set to hs mode\n");
		return;
	}

	if (dsi->status.driven == DSI_DRIVEN_MODE_DC) {
		tegra_dsi_start_dc_stream(dc, dsi);
	}

	mutex_unlock(&dsi->lock);
	tegra_dc_io_end(dc);
}

static void _tegra_dc_dsi_init(struct tegra_dc *dc)
{
	struct tegra_dc_dsi_data *dsi = tegra_dc_get_outdata(dc);

	tegra_dsi_init_sw(dc, dsi);
	/* TODO: Configure the CSI pad configuration */
}

static int tegra_dc_dsi_cp_init_cmd(struct tegra_dsi_cmd* src,
					struct tegra_dsi_cmd* dst, u16 n_cmd)
{
	u16 i;
	u16 len;

	memcpy(dst, src, sizeof(*dst) * n_cmd);

	for (i = 0; i < n_cmd; i++)
		if (src[i].pdata) {
			len = sizeof(*src[i].pdata) * src[i].sp_len_dly.data_len;
			dst[i].pdata = kzalloc(len, GFP_KERNEL);
			if (!dst[i].pdata)
				goto free_cmd_pdata;
			memcpy(dst[i].pdata, src[i].pdata, len);
		}

	return 0;

free_cmd_pdata:
	for (--i; i >=0; i--)
		if (dst[i].pdata)
			kfree(dst[i].pdata);
	return -ENOMEM;
}

static int tegra_dc_dsi_cp_info(struct tegra_dc_dsi_data* dsi,
						struct tegra_dsi_out* p_dsi)
{
	struct tegra_dsi_cmd* pcmd;
	int err;

	pcmd = kzalloc(sizeof(*pcmd) * p_dsi->n_init_cmd, GFP_KERNEL);
	if (!pcmd)
		return -ENOMEM;

	if (p_dsi->n_data_lanes > MAX_DSI_DATA_LANES) {
		err = -EINVAL;
		goto err_free_pcmd;
	}

	memcpy(&dsi->info, p_dsi, sizeof(dsi->info));

	err = tegra_dc_dsi_cp_init_cmd(p_dsi->dsi_init_cmd,
						pcmd, p_dsi->n_init_cmd);
	if (err < 0)
		goto err_free_pcmd;

	dsi->info.dsi_init_cmd = pcmd;

	if (!dsi->info.panel_reset_timeout_msec)
		dsi->info.panel_reset_timeout_msec = DEFAULT_PANEL_RESET_TIMEOUT;

	if (!dsi->info.panel_buffer_size_byte)
		dsi->info.panel_buffer_size_byte = DEFAULT_PANEL_BUFFER_BYTE;

	if (!dsi->info.max_panel_freq_khz)
		dsi->info.max_panel_freq_khz = DEFAULT_MAX_DSI_PHY_CLK_KHZ;

	if (!dsi->info.lp_cmd_mode_freq_khz)
		dsi->info.lp_cmd_mode_freq_khz = DEFAULT_LP_CMD_MODE_CLK_KHZ;

	dsi->controller_index = 0;

	/* host mode is for testing only*/
	dsi->driven_mode = TEGRA_DSI_DRIVEN_BY_DC;

	return 0;

err_free_pcmd:
	kfree(pcmd);
	return err;
}

static int tegra_dc_dsi_init(struct tegra_dc *dc)
{
	struct tegra_dc_dsi_data *dsi;
	struct resource *res;
	struct resource *base_res;
	void __iomem *base;
	struct clk *dc_clk = NULL;
	struct clk *dsi_clk = NULL;
	int err;

	err = 0;

	dsi = kzalloc(sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	res = nvhost_get_resource_byname(dc->ndev, IORESOURCE_MEM,
					"dsi_regs");
	if (!res) {
		dev_err(&dc->ndev->dev, "dsi: no mem resource\n");
		err = -ENOENT;
		goto err_free_dsi;
	}

	base_res = request_mem_region(res->start, resource_size(res),
				dc->ndev->name);
	if (!base_res) {
		dev_err(&dc->ndev->dev, "dsi: request_mem_region failed\n");
		err = -EBUSY;
		goto err_free_dsi;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&dc->ndev->dev, "dsi: registers can't be mapped\n");
		err = -EBUSY;
		goto err_release_regs;
	}

	dsi_clk = clk_get(&dc->ndev->dev, "dsia");
	if (IS_ERR_OR_NULL(dsi_clk)) {
		dev_err(&dc->ndev->dev, "dsi: can't get clock\n");
		err = -EBUSY;
		goto err_release_regs;
	}

	dc_clk = clk_get_sys(dev_name(&dc->ndev->dev), NULL);
	if (IS_ERR_OR_NULL(dc_clk)) {
		dev_err(&dc->ndev->dev, "dsi: dc clock %s unavailable\n",
			dev_name(&dc->ndev->dev));
		err = -EBUSY;
		goto err_clk_put;
	}

	if (!dc->pdata->default_out->dsi) {
		dev_err(&dc->ndev->dev, "dsi: dsi data not available\n");
		goto err_dsi_data;
	}

	err = tegra_dc_dsi_cp_info(dsi, dc->pdata->default_out->dsi);
	if (err < 0)
		goto err_dsi_data;

	mutex_init(&dsi->lock);
	dsi->dc = dc;
	dsi->base = base;
	dsi->base_res = base_res;
	dsi->dc_clk = dc_clk;
	dsi->dsi_clk = dsi_clk;

	tegra_dc_set_outdata(dc, dsi);
	_tegra_dc_dsi_init(dc);

	return 0;

err_dsi_data:
err_clk_put:
	clk_put(dsi->dsi_clk);
err_release_regs:
	release_resource(base_res);
err_free_dsi:
	kfree(dsi);

	return err;
}

static void tegra_dc_dsi_destroy(struct tegra_dc *dc)
{
	struct tegra_dc_dsi_data *dsi = tegra_dc_get_outdata(dc);
	u16 i;
	u32 val;

	mutex_lock(&dsi->lock);

	/* free up the pdata*/
	for(i = 0; i < dsi->info.n_init_cmd; i++){
		if(dsi->info.dsi_init_cmd[i].pdata)
			kfree(dsi->info.dsi_init_cmd[i].pdata);
	}
	kfree(dsi->info.dsi_init_cmd);

	/* Disable dc stream*/
	if(dsi->status.driven == DSI_DRIVEN_MODE_DC)
		tegra_dsi_stop_dc_stream(dc, dsi);

	/* Disable dsi phy clock*/
	if(dsi->status.clk_out == DSI_PHYCLK_OUT_EN)
		tegra_dsi_hs_clk_out_disable(dc, dsi);

	val = DSI_POWER_CONTROL_LEG_DSI_ENABLE(TEGRA_DSI_DISABLE);
	tegra_dsi_writel(dsi, val, DSI_POWER_CONTROL);

	iounmap(dsi->base);
	release_resource(dsi->base_res);

	clk_put(dsi->dc_clk);
	clk_put(dsi->dsi_clk);

	mutex_unlock(&dsi->lock);

	mutex_destroy(&dsi->lock);
	kfree(dsi);
}

static void tegra_dc_dsi_disable(struct tegra_dc *dc)
{
	struct tegra_dc_dsi_data *dsi = tegra_dc_get_outdata(dc);

	mutex_lock(&dsi->lock);

	if (dsi->status.driven == DSI_DRIVEN_MODE_DC)
		tegra_dsi_stop_dc_stream(dc, dsi);

	if (dsi->status.clk_out == DSI_PHYCLK_OUT_EN)
		tegra_dsi_hs_clk_out_disable(dc, dsi);

	mutex_unlock(&dsi->lock);

	dev_err(&dc->ndev->dev, "dsi: disable\n");
}

struct tegra_dc_out_ops tegra_dc_dsi_ops = {
	.init = tegra_dc_dsi_init,
	.destroy = tegra_dc_dsi_destroy,
	.enable = tegra_dc_dsi_enable,
	.disable = tegra_dc_dsi_disable,
};
