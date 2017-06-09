/*
 * dc.h: Headers required for tegra dc and dependent modules.
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *
 * Copyright (c) 2010-2017, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __MACH_TEGRA_DC_H
#define __MACH_TEGRA_DC_H

#include <linux/pm.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/isomgr.h>
#include <drm/drm_fixed.h>
#include <linux/nvhost.h>
#include <video/tegra_dc_ext_kernel.h>
#include <video/tegra_dc_ext.h>
#include "dc_extras.h"

#define DEFAULT_FPGA_FREQ_KHZ	160000

#define TEGRA_DC_EXT_FLIP_MAX_WINDOW 6
#if defined(CONFIG_TEGRA_NVDISPLAY)
#define GAIN_TABLE_MAX_ENTRIES	32
#define ADAPTATION_FACTOR_MAX_LEVELS	51
#endif
extern atomic_t sd_brightness;

extern struct fb_videomode tegra_dc_vga_mode;

enum {
	TEGRA_HPD_STATE_FORCE_DEASSERT = -1,
	TEGRA_HPD_STATE_NORMAL = 0,
	TEGRA_HPD_STATE_FORCE_ASSERT = 1,
};

/* DSI pixel data format */
enum {
	TEGRA_DSI_PIXEL_FORMAT_16BIT_P,
	TEGRA_DSI_PIXEL_FORMAT_18BIT_P,
	TEGRA_DSI_PIXEL_FORMAT_18BIT_NP,
	TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	TEGRA_DSI_PIXEL_FORMAT_8BIT_DSC,
	TEGRA_DSI_PIXEL_FORMAT_12BIT_DSC,
	TEGRA_DSI_PIXEL_FORMAT_16BIT_DSC,
};

/* DSI virtual channel number */
enum {
	TEGRA_DSI_VIRTUAL_CHANNEL_0,
	TEGRA_DSI_VIRTUAL_CHANNEL_1,
	TEGRA_DSI_VIRTUAL_CHANNEL_2,
	TEGRA_DSI_VIRTUAL_CHANNEL_3,
};

/* DSI transmit method for video data */
enum {
	TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
};

/* DSI HS clock mode */
enum {
	TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
};

/* DSI burst mode setting in video mode. Each mode is assigned with a
 * fixed value. The rationale behind this is to avoid change of these
 * values, since the calculation of dsi clock depends on them. */
enum {
	TEGRA_DSI_VIDEO_NONE_BURST_MODE = 0,
	TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END = 1,
	TEGRA_DSI_VIDEO_BURST_MODE_LOWEST_SPEED = 2,
	TEGRA_DSI_VIDEO_BURST_MODE_LOW_SPEED = 3,
	TEGRA_DSI_VIDEO_BURST_MODE_MEDIUM_SPEED = 4,
	TEGRA_DSI_VIDEO_BURST_MODE_FAST_SPEED = 5,
	TEGRA_DSI_VIDEO_BURST_MODE_FASTEST_SPEED = 6,
};

enum {
	TEGRA_DSI_GANGED_SYMMETRIC_LEFT_RIGHT = 1,
	TEGRA_DSI_GANGED_SYMMETRIC_EVEN_ODD = 2,
	TEGRA_DSI_GANGED_SYMMETRIC_LEFT_RIGHT_OVERLAP = 3,
};

/* Split Link Type */
enum {
	TEGRA_DSI_SPLIT_LINK_A_B = 1,
	TEGRA_DSI_SPLIT_LINK_C_D = 2,
	TEGRA_DSI_SPLIT_LINK_A_B_C_D = 3,
};

enum {
	TEGRA_DSI_PACKET_CMD,
	TEGRA_DSI_DELAY_MS,
	TEGRA_DSI_GPIO_SET,
	TEGRA_DSI_SEND_FRAME,
	TEGRA_DSI_PACKET_VIDEO_VBLANK_CMD,
	TEGRA_DSI_DELAY_US,
};
enum {
	TEGRA_DSI_LINK0,
	TEGRA_DSI_LINK1,
};

struct tegra_dsi_cmd {
	u8	cmd_type;
	u8	data_id;
	union {
		u16 data_len;
		u16 delay_ms;
		u16 delay_us;
		unsigned gpio;
		u16 frame_cnt;
		struct {
			u8 data0;
			u8 data1;
		} sp;
	} sp_len_dly;
	u8	*pdata;
	u8   link_id;
	bool	club_cmd;
};

#define CMD_CLUBBED				true
#define CMD_NOT_CLUBBED				false

#define DSI_GENERIC_LONG_WRITE			0x29
#define DSI_DCS_LONG_WRITE			0x39
#define DSI_GENERIC_SHORT_WRITE_1_PARAMS	0x13
#define DSI_GENERIC_SHORT_WRITE_2_PARAMS	0x23
#define DSI_DCS_WRITE_0_PARAM			0x05
#define DSI_DCS_WRITE_1_PARAM			0x15

#define DSI_DCS_SET_ADDR_MODE			0x36
#define DSI_DCS_EXIT_SLEEP_MODE			0x11
#define DSI_DCS_ENTER_SLEEP_MODE		0x10
#define DSI_DCS_SET_DISPLAY_ON			0x29
#define DSI_DCS_SET_DISPLAY_OFF			0x28
#define DSI_DCS_SET_TEARING_EFFECT_OFF		0x34
#define DSI_DCS_SET_TEARING_EFFECT_ON		0x35
#define DSI_DCS_NO_OP				0x0
#define DSI_NULL_PKT_NO_DATA			0x9
#define DSI_BLANKING_PKT_NO_DATA		0x19

#define IS_DSI_SHORT_PKT(cmd)	((cmd.data_id == DSI_DCS_WRITE_0_PARAM) ||\
			(cmd.data_id == DSI_DCS_WRITE_1_PARAM) ||\
			(cmd.data_id == DSI_GENERIC_SHORT_WRITE_1_PARAMS) ||\
			(cmd.data_id == DSI_GENERIC_SHORT_WRITE_2_PARAMS))

#define _DSI_CMD_SHORT(di, p0, p1, lnk_id, _cmd_type, club)	{ \
					.cmd_type = _cmd_type, \
					.data_id = di, \
					.sp_len_dly.sp.data0 = p0, \
					.sp_len_dly.sp.data1 = p1, \
					.link_id = lnk_id, \
					.club_cmd = club,\
					}

#define DSI_CMD_VBLANK_SHORT(di, p0, p1, club) \
			_DSI_CMD_SHORT(di, p0, p1, TEGRA_DSI_LINK0,\
				TEGRA_DSI_PACKET_VIDEO_VBLANK_CMD, club)

#define DSI_CMD_SHORT_LINK(di, p0, p1, lnk_id) \
			_DSI_CMD_SHORT(di, p0, p1, lnk_id,\
				TEGRA_DSI_PACKET_CMD, CMD_NOT_CLUBBED)

#define DSI_CMD_SHORT(di, p0, p1)	\
			DSI_CMD_SHORT_LINK(di, p0, p1, TEGRA_DSI_LINK0)

#define DSI_DLY_MS(ms)	{ \
			.cmd_type = TEGRA_DSI_DELAY_MS, \
			.sp_len_dly.delay_ms = ms, \
			}

#define DSI_GPIO_SET(rst_gpio, on)	{ \
					.cmd_type = TEGRA_DSI_GPIO_SET, \
					.data_id = on, \
					.sp_len_dly.gpio = rst_gpio, \
					}

#define _DSI_CMD_LONG(di, ptr, lnk_id, _cmd_type)	{ \
				.cmd_type = _cmd_type, \
				.data_id = di, \
				.sp_len_dly.data_len = ARRAY_SIZE(ptr), \
				.pdata = ptr, \
				.link_id = lnk_id, \
				}

#define DSI_CMD_VBLANK_LONG(di, ptr)	\
		_DSI_CMD_LONG(di, ptr, TEGRA_DSI_LINK0, \
					TEGRA_DSI_PACKET_VIDEO_VBLANK_CMD)

#define DSI_CMD_LONG_LINK(di, ptr, lnk_id)	\
		_DSI_CMD_LONG(di, ptr, lnk_id, TEGRA_DSI_PACKET_CMD)

#define DSI_CMD_LONG(di, ptr)	\
			DSI_CMD_LONG_LINK(di, ptr, TEGRA_DSI_LINK0)

#define _DSI_CMD_LONG_SIZE(di, ptr, size, lnk_id, _cmd_type)	{ \
				.cmd_type = _cmd_type, \
				.data_id = di, \
				.sp_len_dly.data_len = size, \
				.pdata = ptr, \
				.link_id = lnk_id, \
				}

#define DSI_CMD_LONG_SIZE(di, ptr, size)	\
	_DSI_CMD_LONG_SIZE(di, ptr, size, TEGRA_DSI_LINK0, TEGRA_DSI_PACKET_CMD)

#define DSI_SEND_FRAME(cnt)	{ \
			.cmd_type = TEGRA_DSI_SEND_FRAME, \
			.sp_len_dly.frame_cnt = cnt, \
			}

struct dsi_phy_timing_ns {
	u16		t_hsdexit_ns;
	u16		t_hstrail_ns;
	u16		t_datzero_ns;
	u16		t_hsprepare_ns;

	u16		t_clktrail_ns;
	u16		t_clkpost_ns;
	u16		t_clkzero_ns;
	u16		t_tlpx_ns;

	u16		t_clkprepare_ns;
	u16		t_clkpre_ns;
	u16		t_wakeup_ns;

	u16		t_taget_ns;
	u16		t_tasure_ns;
	u16		t_tago_ns;
};

enum {
	CMD_VS		= 0x01,
	CMD_VE		= 0x11,

	CMD_HS		= 0x21,
	CMD_HE		= 0x31,

	CMD_EOT		= 0x08,
	CMD_NULL	= 0x09,
	CMD_SHORTW	= 0x15,
	CMD_BLNK	= 0x19,
	CMD_LONGW	= 0x39,

	CMD_RGB		= 0x00,
	CMD_RGB_16BPP	= 0x0E,
	CMD_RGB_18BPP	= 0x1E,
	CMD_RGB_18BPPNP = 0x2E,
	CMD_RGB_24BPP	= 0x3E,

	CMD_CMPR_PIXEL_STREAM	 = 0x0B,
	CMD_DSC_WRITE_START	= 0x2C,
	CMD_DSC_WRITE_CONT	= 0x3C,
};

enum {
	TEGRA_DSI_DISABLE,
	TEGRA_DSI_ENABLE,
};

#define PKT_ID0(id)	((((id) & 0x3f) << 3) | \
			(((TEGRA_DSI_ENABLE) & 0x1) << 9))
#define PKT_LEN0(len)	(((len) & 0x7) << 0)
#define PKT_ID1(id)	((((id) & 0x3f) << 13) | \
			(((TEGRA_DSI_ENABLE) & 0x1) << 19))
#define PKT_LEN1(len)	(((len) & 0x7) << 10)
#define PKT_ID2(id)	((((id) & 0x3f) << 23) | \
			(((TEGRA_DSI_ENABLE) & 0x1) << 29))
#define PKT_LEN2(len)	(((len) & 0x7) << 20)
#define PKT_ID3(id)	((((id) & 0x3f) << 3) | \
			(((TEGRA_DSI_ENABLE) & 0x1) << 9))
#define PKT_LEN3(len)	(((len) & 0x7) << 0)
#define PKT_ID4(id)	((((id) & 0x3f) << 13) | \
			(((TEGRA_DSI_ENABLE) & 0x1) << 19))
#define PKT_LEN4(len)	(((len) & 0x7) << 10)
#define PKT_ID5(id)	((((id) & 0x3f) << 23) | \
			(((TEGRA_DSI_ENABLE) & 0x1) << 29))
#define PKT_LEN5(len)	(((len) & 0x7) << 20)
#define PKT_LP		(((TEGRA_DSI_ENABLE) & 0x1) << 30)
#define NUMOF_PKT_SEQ	12

enum {
	DSI_VS_0 = 0x0,
	DSI_VS_1 = 0x1,
};

enum {
#ifndef CONFIG_TEGRA_NVDISPLAY
	DSI_INSTANCE_0,
	DSI_INSTANCE_1,
#else
/* T186 has 4 controllers. DSI-A and DSI-C are the main controllers
   needed for ganged mode */
	DSI_INSTANCE_0,
	DSI_INSTANCE_1 = 2,
#endif
};

/* Aggressiveness level of DSI suspend. The higher, the more aggressive. */
#define DSI_NO_SUSPEND			0
#define DSI_HOST_SUSPEND_LV0		1
#define DSI_HOST_SUSPEND_LV1		2
#define DSI_HOST_SUSPEND_LV2		3

/*
 * DPD (deep power down) mode for dsi pads.
 *  - Available for DSI_VS1 and later.
 *  - Available for Non-DSI_GANGED.
 * Usually, DSIC/DSID pads' DPD for DSI_INSTANCE_0 and
   DSI/DSIB pads' DPD for DSI_INSTANCE_1 can be enabled, respectively,
 * but in SW, sometimes pins from one pad can be used by
 * more than one module, so it may be dependent on board design.
 * 0:A
 * 1:B
 * 2:C
 * 3:D
 */
#define DSI_DPD_EN(i)		(1 << i)

struct tegra_dsi_board_info {
	u32 platform_boardid;
	u32 platform_boardversion;
	u32 display_boardid;
	u32 display_boardversion;
};
struct tegra_dsi_out {
	u8		n_data_lanes;			/* required */
	u8		pixel_format;			/* required */
	u8		refresh_rate;			/* required */
	u8		rated_refresh_rate;
	u8		panel_reset;			/* required */
	u8		virtual_channel;		/* required */
	u8		dsi_instance;
	u16		dsi_panel_rst_gpio;
	u16		dsi_panel_bl_en_gpio;
	u16		dsi_panel_bl_pwm_gpio;
	u16		even_odd_split_width;
	u8		controller_vs;

	bool		panel_has_frame_buffer;	/* required*/

	/* Deprecated. Use DSI_SEND_FRAME panel command instead. */
	bool		panel_send_dc_frames;

	struct tegra_dsi_cmd	*dsi_init_cmd;		/* required */
	u16		n_init_cmd;			/* required */

	struct tegra_dsi_cmd	*dsi_early_suspend_cmd;
	u16		n_early_suspend_cmd;

	struct tegra_dsi_cmd	*dsi_late_resume_cmd;
	u16		n_late_resume_cmd;

	struct tegra_dsi_cmd	*dsi_postvideo_cmd;
	u16		n_postvideo_cmd;

	struct tegra_dsi_cmd	*dsi_suspend_cmd;	/* required */
	u16		n_suspend_cmd;			/* required */

	u8		video_data_type;		/* required */
	u8		video_clock_mode;
	u8		video_burst_mode;
	u8		ganged_type;
	u16		ganged_overlap;
	bool		ganged_swap_links;
	bool		ganged_write_to_all_links;
	u8		split_link_type;

	u8		suspend_aggr;

	u16		panel_buffer_size_byte;
	u16		panel_reset_timeout_msec;

	bool		hs_cmd_mode_supported;
	bool		hs_cmd_mode_on_blank_supported;
	bool		enable_hs_clock_on_lp_cmd_mode;
	bool		drm_override_disable;
	bool		no_pkt_seq_eot; /* 1st generation panel may not
					 * support eot. Don't set it for
					 * most panels. */
	bool		skip_dsi_pkt_header;
	bool		te_polarity_low;
	bool		power_saving_suspend;
	bool		suspend_stop_stream_late;
	bool		dsi2lvds_bridge_enable;
	bool		dsi2edp_bridge_enable;

	u32		max_panel_freq_khz;
	u32		lp_cmd_mode_freq_khz;
	u32		lp_read_cmd_mode_freq_khz;
	u32		hs_clk_in_lp_cmd_mode_freq_khz;
	u32		burst_mode_freq_khz;
	u32		fpga_freq_khz;

	u32		te_gpio;

	u32		dpd_dsi_pads;

	const u32		*pkt_seq;

	struct dsi_phy_timing_ns phy_timing;

	u8		*bl_name;

	bool		lp00_pre_panel_wakeup;
	bool		ulpm_not_supported;
	struct tegra_dsi_board_info	boardinfo;
	bool		use_video_host_fifo_for_cmd;
	bool		dsi_csi_loopback;
	bool		set_max_timeout;
	u32		refresh_rate_adj;
};

struct tegra_panel_reg {
	struct regulator *avdd_lcd;
	struct regulator *avee_lcd;
	struct regulator *vddi_lcd;
};

enum {
	TEGRA_DC_STEREO_MODE_2D,
	TEGRA_DC_STEREO_MODE_3D
};

enum {
	TEGRA_DC_STEREO_LANDSCAPE,
	TEGRA_DC_STEREO_PORTRAIT
};

struct tegra_stereo_out {
	int  mode_2d_3d;
	int  orientation;

	void (*set_mode)(int mode);
	void (*set_orientation)(int orientation);
};

struct tegra_dc_mode {
	int	pclk;
	int	rated_pclk;
	int	h_ref_to_sync;
	int	v_ref_to_sync;
	int	h_sync_width;
	int	v_sync_width;
	int	h_back_porch;
	int	v_back_porch;
	int	h_active;
	int	v_active;
	int	h_front_porch;
	int	v_front_porch;
	int	stereo_mode;
	u32	flags;
	u8	avi_m;
	u32	vmode;
};

struct tegra_dc_mode_metadata {
	u64 line_in_nsec; /* Line duration in nsec */
	int vtotal_lines; /* # of lines to vsync */
	int vblank_lines; /* # of lines till vblank intr; vtotal-fp */
};

#define TEGRA_DC_MODE_FLAG_NEG_V_SYNC	(1 << 0)
#define TEGRA_DC_MODE_FLAG_NEG_H_SYNC	(1 << 1)
#define TEGRA_DC_MODE_FLAG_NEG_DE		(1 << 2)

#define TEGRA_DC_MODE_AVI_M_NO_DATA	0x0
#define TEGRA_DC_MODE_AVI_M_4_3	0x1
#define TEGRA_DC_MODE_AVI_M_16_9	0x2
#define TEGRA_DC_MODE_AVI_M_64_27	0x3	/* dummy, no avi m support */
#define TEGRA_DC_MODE_AVI_M_256_135	0x4	/* dummy, no avi m support */

enum {
	TEGRA_DC_OUT_RGB,
	TEGRA_DC_OUT_HDMI,
	TEGRA_DC_OUT_DSI,
	TEGRA_DC_OUT_DP,
	TEGRA_DC_OUT_LVDS,
	TEGRA_DC_OUT_NVSR_DP,
	TEGRA_DC_OUT_FAKE_DP,
	TEGRA_DC_OUT_FAKE_DSIA,
	TEGRA_DC_OUT_FAKE_DSIB,
	TEGRA_DC_OUT_FAKE_DSI_GANGED,
	TEGRA_DC_OUT_NULL,
	TEGRA_DC_OUT_MAX /* Keep this always as last enum */
};

struct tegra_dc_out_pin {
	int	name;
	int	pol;
};

enum {
	TEGRA_DC_OUT_PIN_DATA_ENABLE,
	TEGRA_DC_OUT_PIN_H_SYNC,
	TEGRA_DC_OUT_PIN_V_SYNC,
	TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
};

enum {
	TEGRA_DC_OUT_PIN_POL_LOW,
	TEGRA_DC_OUT_PIN_POL_HIGH,
};

enum {
	TEGRA_DC_UNDEFINED_DITHER = 0,
	TEGRA_DC_DISABLE_DITHER,
	TEGRA_DC_ORDERED_DITHER,
	TEGRA_DC_ERRDIFF_DITHER,
	TEGRA_DC_TEMPORAL_DITHER,
	TEGRA_DC_ERRACC_DITHER,
};

typedef u8 tegra_dc_bl_output[256];
typedef u8 *p_tegra_dc_bl_output;

struct tegra_dc_sd_blp {
	u16 time_constant;
	u8 step;
};

struct tegra_dc_sd_fc {
	u8 time_limit;
	u8 threshold;
};

struct tegra_dc_sd_rgb {
	u8 r;
	u8 g;
	u8 b;
};

struct tegra_dc_sd_agg_priorities {
	u8 pri_lvl;
	u8 agg[4];
};

struct tegra_dc_sd_window {
	u16 h_position;
	u16 v_position;
	u16 h_size;
	u16 v_size;
};

struct tegra_dc_sd_settings {
#ifdef CONFIG_TEGRA_NVDISPLAY
	bool update_sd;
	unsigned upper_bound;
	unsigned lower_bound;
	unsigned num_over_saturated_pixels;
	unsigned over_saturated_bin;
	unsigned last_over_saturated_bin;
	unsigned *gain_table;
	unsigned gain_luts_parsed;
	unsigned pixel_gain_tables[ADAPTATION_FACTOR_MAX_LEVELS]
		[GAIN_TABLE_MAX_ENTRIES];
	unsigned backlight_table[ADAPTATION_FACTOR_MAX_LEVELS];
	unsigned *current_gain_table;
	unsigned *phase_backlight_table;
	unsigned new_backlight;
	unsigned old_backlight;
	unsigned last_phase_step;
	unsigned phase_in_steps;
	int backlight_adjust_steps;
	u8 sw_update_delay;
	int frame_runner;
#endif
	unsigned enable;
	u8 turn_off_brightness;
	u8 turn_on_brightness;
	unsigned enable_int;
	bool use_auto_pwm;
	u8 hw_update_delay;
	u8 aggressiveness;
	short bin_width;
	u8 phase_in_settings;
	u8 phase_in_adjustments;
	u8 cmd;
	u8 final_agg;
	u16 cur_agg_step;
	u16 phase_settings_step;
	u16 phase_adj_step;
	u16 num_phase_in_steps;

	struct tegra_dc_sd_agg_priorities agg_priorities;

	bool use_vid_luma;
	struct tegra_dc_sd_rgb coeff;

	bool k_limit_enable;
	u16 k_limit;

	bool sd_window_enable;
	struct tegra_dc_sd_window sd_window;

	bool soft_clipping_enable;
	u8 soft_clipping_threshold;

	bool smooth_k_enable;
	u16 smooth_k_incr;

	bool sd_proc_control;
	bool soft_clipping_correction;
	bool use_vpulse2;

	struct tegra_dc_sd_fc fc;
	struct tegra_dc_sd_blp blp;
	u8 bltf[4][4][4];
	struct tegra_dc_sd_rgb lut[4][9];

	atomic_t *sd_brightness;
	char *bl_device_name;
	struct backlight_device *bl_device;

	u8 bias0;
};

enum {
	NO_CMD = 0x0,
	ENABLE = 0x1,
	DISABLE = 0x2,
	PHASE_IN = 0x4,
	AGG_CHG = 0x8,
};

enum {
	TEGRA_PIN_OUT_CONFIG_SEL_LHP0_LD21,
	TEGRA_PIN_OUT_CONFIG_SEL_LHP1_LD18,
	TEGRA_PIN_OUT_CONFIG_SEL_LHP2_LD19,
	TEGRA_PIN_OUT_CONFIG_SEL_LVP0_LVP0_Out,
	TEGRA_PIN_OUT_CONFIG_SEL_LVP1_LD20,

	TEGRA_PIN_OUT_CONFIG_SEL_LM1_M1,
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_LD21,
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_PM1,

	TEGRA_PIN_OUT_CONFIG_SEL_LDI_LD22,
	TEGRA_PIN_OUT_CONFIG_SEL_LPP_LD23,
	TEGRA_PIN_OUT_CONFIG_SEL_LDC_SDC,
	TEGRA_PIN_OUT_CONFIG_SEL_LSPI_DE,
};

/* this is the old name. provided for compatibility with old board files. */
#define dcc_bus ddc_bus

struct tegra_vrr {
	s32	capability;
	s32	enable;
	s32	lastenable;
	s32	flip;
	s32	pclk;
	s32	vrr_min_fps;
	s32	vrr_max_fps;
	s32	v_front_porch_max;
	s32	v_front_porch_min;
	s32	vfp_extend;
	s32	vfp_shrink;
	s32	v_front_porch;
	s32	v_back_porch;

	s64	curr_flip_us;
	s64	last_flip_us;
	s32	flip_count;
	s32	flip_interval_us;
	s32	frame_len_max;
	s32	frame_len_min;
	s32	frame_len_fluct;
	s32	line_width;
	s32	lines_per_frame_common;

	s32	frame_type;
	s32	frame_count;
	s32	v_count;
	s32	last_v_cnt;
	s32	curr_v_cnt;
	s64     last_frame_us;
	s64     curr_frame_us;
	s64     fe_time_us;
	s32	frame_delta_us;
	s32	frame_interval_us;
	s32	even_frame_us;
	s32	odd_frame_us;

	s32	max_adj_pct;
	s32	max_flip_pct;
	s32	max_dcb;
	s32	max_inc_pct;

	s32	dcb;
	s32	frame_avg_pct;
	s32	fluct_avg_pct;

	s32	fe_intr_req;
	s32	db_tolerance;
	s32	frame2flip_us;
	s32	adjust_vfp;
	s32	adjust_db;
	u32	db_correct_cap;
	u32	db_hist_cap;
	s32	vfp;
	s32	insert_frame;
	s32	g_vrr_session_id;
	s32	nvdisp_direct_drive;

	/* Must be kept in order */
	u8	keynum;
	u8	serial[9];
	u8	challenge[32];
	u8	digest[32];
	u8	challenge_src;
};

struct tegra_dc_out {
	int				type;
	unsigned			flags;
	unsigned			hdcp_policy;

	/* size in mm */
	unsigned			h_size;
	unsigned			v_size;

	int				ddc_bus;
	int				hotplug_gpio;
	int				hotplug_state; /* TEGRA_HPD_STATE_* */
	const char			*parent_clk;
	const char			*parent_clk_backup;

	unsigned			max_pixclock;
	unsigned			order;
	unsigned			align;
	unsigned			depth;
	unsigned			dither;

	struct tegra_dc_mode		*modes;
	int				n_modes;

	struct tegra_dsi_out		*dsi;
	struct tegra_hdmi_out		*hdmi_out;
	struct tegra_dp_out		*dp_out;
	struct tegra_stereo_out		*stereo;
	struct tegra_vrr		*vrr;

	unsigned			height; /* mm */
	unsigned			width; /* mm */
	unsigned			rotation; /* degrees */

	struct tegra_dc_out_pin		*out_pins;
	unsigned			n_out_pins;

	struct tegra_dc_sd_settings	*sd_settings;

	/* DSI link compression parameters */
	u32		slice_height;
	u32		slice_width;
	u8		num_of_slices;
	u8		dsc_bpp;
	bool		en_block_pred;
	bool		dsc_en;
	bool		dual_dsc_en;

	u8			*out_sel_configs;
	unsigned		n_out_sel_configs;
	int			user_needs_vblank;
	struct completion	user_vblank_comp;

	bool				is_ext_dp_panel;

	int	(*enable)(struct device *);
	int	(*postpoweron)(struct device *);
	int	(*prepoweroff)(void);
	int	(*disable)(struct device *);

	int	(*hotplug_init)(struct device *);
	int	(*postsuspend)(void);
	void	(*hotplug_report)(bool);
};

/* bits for tegra_dc_out.flags */
#define TEGRA_DC_OUT_HOTPLUG_HIGH		(0 << 1)
#define TEGRA_DC_OUT_HOTPLUG_LOW		(1 << 1)
#define TEGRA_DC_OUT_HOTPLUG_MASK		(1 << 1)
#define TEGRA_DC_OUT_NVHDCP_POLICY_ALWAYS_ON	(0 << 2)
#define TEGRA_DC_OUT_NVHDCP_POLICY_ON_DEMAND	(1 << 2)
#define TEGRA_DC_OUT_NVHDCP_POLICY_MASK		(1 << 2)
#define TEGRA_DC_OUT_CONTINUOUS_MODE		(0 << 3)
#define TEGRA_DC_OUT_ONE_SHOT_MODE		(1 << 3)
#define TEGRA_DC_OUT_N_SHOT_MODE		(1 << 4)
#define TEGRA_DC_OUT_ONE_SHOT_LP_MODE		(1 << 5)
#define TEGRA_DC_OUT_INITIALIZED_MODE		(1 << 6)
/* Makes hotplug GPIO a LP0 wakeup source */
#define TEGRA_DC_OUT_HOTPLUG_WAKE_LP0		(1 << 7)
#define TEGRA_DC_OUT_NVSR_MODE			(1 << 8)

#define TEGRA_DC_HDCP_POLICY_ALWAYS_ON	0
#define TEGRA_DC_HDCP_POLICY_ON_DEMAND	1
#define TEGRA_DC_HDCP_POLICY_ALWAYS_OFF	2

#define TEGRA_DC_ALIGN_MSB		0
#define TEGRA_DC_ALIGN_LSB		1

#define TEGRA_DC_ORDER_RED_BLUE		0
#define TEGRA_DC_ORDER_BLUE_RED		1

/* Errands use the interrupts */
#define V_BLANK_FLIP		0
#define V_BLANK_NVSD		1
#define V_BLANK_USER		2
#define V_BLANK_IMP		3

#define V_PULSE2_FLIP		0
#define V_PULSE2_NVSD		1

struct tegra_dc_cmu_csc {
	u16 krr;
	u16 kgr;
	u16 kbr;
	u16 krg;
	u16 kgg;
	u16 kbg;
	u16 krb;
	u16 kgb;
	u16 kbb;
};

#if defined(CONFIG_TEGRA_DC_CMU_V2)
struct tegra_dc_cmu {
	u64			rgb[1025];
#if defined(CONFIG_TEGRA_CSC_V2)
	struct tegra_dc_csc_v2	panel_csc;
#endif
};
#else
struct tegra_dc_cmu {
	u16 lut1[256];
	struct tegra_dc_cmu_csc csc;
	u8 lut2[960];
};
#endif

struct tegra_dc_hdr {
	bool		enabled;
	u32		eotf;
	u32		static_metadata_id;
	u8		static_metadata[24];
};

struct frame_lock_info {
	bool frame_lock_enable;
	bool job_pending;
	bool check_for_error;
	wait_queue_head_t win_upd_reqs;

};

#define TEGRA_WIN_PPFLAG_CP_ENABLE	(1 << 0) /* enable RGB color lut */
#define TEGRA_WIN_PPFLAG_CP_FBOVERRIDE	(1 << 1) /* override fbdev color lut */

struct tegra_fb_data {
	int		win;

	int		xres;
	int		yres;
	int		bits_per_pixel; /* -1 means autodetect */
	size_t		fbmem_size;

	unsigned long	flags;
};

#define TEGRA_FB_FLIP_ON_PROBE		(1 << 0)

struct tegra_dc_platform_data {
	unsigned long		flags;
	unsigned long		emc_clk_rate;
	struct tegra_dc_out	*default_out;
	struct tegra_fb_data	*fb;
	unsigned long		low_v_win;

#if defined(CONFIG_TEGRA_DC_CMU) || defined(CONFIG_TEGRA_DC_CMU_V2)
	bool			cmu_enable;
	struct tegra_dc_cmu	*cmu;
	struct tegra_dc_cmu	*cmu_adbRGB;
	int			default_clr_space;
#endif
	unsigned long		ctrl_num;
	unsigned long		win_mask;
	struct device_node	*conn_np; /* DSI, SOR0, SOR1, etc. */
	struct device_node	*panel_np; /* dp-display, hdmi-display etc. */
	struct device_node	*def_out_np; /* disp-default-out */
	bool			frame_lock_enable;
};

struct tegra_dc_bw_data {
	u32	total_bw;
	u32	avail_bw;
	u32	resvd_bw;
};

#define TEGRA_DC_FLAG_ENABLED		(1 << 0)
#define TEGRA_DC_FLAG_SET_EARLY_MODE		(1 << 1)

int tegra_dc_get_stride(struct tegra_dc *dc, unsigned win);
struct tegra_dc_win *tegra_dc_get_window(struct tegra_dc *dc, unsigned win);
bool tegra_dc_get_connected(struct tegra_dc *);
bool tegra_dc_hpd(struct tegra_dc *dc);

bool tegra_dc_has_vsync(struct tegra_dc *dc);
int tegra_dc_vsync_enable(struct tegra_dc *dc);
void tegra_dc_vsync_disable(struct tegra_dc *dc);
int tegra_dc_wait_for_vsync(struct tegra_dc *dc);
void tegra_dc_blank_wins(struct tegra_dc *dc, unsigned windows);
int tegra_dc_restore(struct tegra_dc *dc);

void tegra_dc_enable(struct tegra_dc *dc);
void tegra_dc_disable(struct tegra_dc *dc);
int tegra_dc_set_default_videomode(struct tegra_dc *dc);


u32 tegra_dc_get_syncpt_id(struct tegra_dc *dc, int i);
u32 tegra_dc_incr_syncpt_max(struct tegra_dc *dc, int i);
void tegra_dc_incr_syncpt_min(struct tegra_dc *dc, int i, u32 val);
struct sync_fence *tegra_dc_create_fence(struct tegra_dc *dc, int i, u32 val);

/* needed for tegra/dc/ext/ */

int __init tegra_dc_ext_module_init(void);
void __exit tegra_dc_ext_module_exit(void);

struct tegra_dc_ext *tegra_dc_ext_register(struct platform_device *ndev,
					   struct tegra_dc *dc);
void tegra_dc_ext_unregister(struct tegra_dc_ext *dc_ext);

void tegra_dc_ext_enable(struct tegra_dc_ext *dc_ext);
int tegra_dc_ext_disable(struct tegra_dc_ext *dc_ext);
int tegra_dc_ext_restore(struct tegra_dc_ext *dc_ext);

int tegra_dc_ext_process_hotplug(int output);
int tegra_dc_ext_process_vblank(int output, u64 timestamp);
int tegra_dc_ext_process_modechange(int output);
int tegra_dc_ext_process_bandwidth_renegotiate(int output,
					struct tegra_dc_bw_data *bw);
bool tegra_dc_ext_is_userspace_active(void);
int tegra_dc_ext_get_scanline(struct tegra_dc_ext *dc_ext);
/* finish dc/ext */

/* needed for tegra_fb.h merge */
struct tegra_fb_info;
struct resource;

#if IS_ENABLED(CONFIG_FB_TEGRA)
struct tegra_fb_info *tegra_fb_register(struct platform_device *ndev,
					struct tegra_dc *dc,
					struct tegra_fb_data *fb_data,
					struct resource *fb_mem,
					void *virt_addr);
void tegra_fb_unregister(struct tegra_fb_info *fb_info);
void tegra_fb_pan_display_reset(struct tegra_fb_info *fb_info);
void tegra_fb_update_monspecs(struct tegra_fb_info *fb_info,
			      struct fb_monspecs *specs,
			      bool (*mode_filter)(const struct tegra_dc *dc,
						  struct fb_videomode *mode));
void tegra_fb_update_fix(struct tegra_fb_info *fb_info,
				struct fb_monspecs *specs);
struct fb_var_screeninfo *tegra_fb_get_var(struct tegra_fb_info *fb_info);
int tegra_fb_create_sysfs(struct device *dev);
void tegra_fb_remove_sysfs(struct device *dev);
int tegra_fb_set_var(struct tegra_dc *dc, struct fb_var_screeninfo *var);
int tegra_fb_update_modelist(struct tegra_dc *dc, int fblistindex);
struct tegra_dc_win *tegra_fb_get_win(struct tegra_fb_info *tegra_fb);
struct tegra_dc_win *tegra_fb_get_blank_win(struct tegra_fb_info *tegra_fb);
int tegra_fb_set_win_index(struct tegra_dc *dc, unsigned long win_mask);
struct fb_videomode *tegra_fb_get_mode(struct tegra_dc *dc);
#else
static inline struct tegra_fb_info *tegra_fb_register(
	struct platform_device *ndev, struct tegra_dc *dc,
	struct tegra_fb_data *fb_data, struct resource *fb_mem)
{
	return NULL;
}

static inline void tegra_fb_unregister(struct tegra_fb_info *fb_info)
{
}

static inline void tegra_fb_pan_display_reset(struct tegra_fb_info *fb_info)
{
}

static inline void tegra_fb_update_monspecs(struct tegra_fb_info *fb_info,
					    struct fb_monspecs *specs,
				bool (*mode_filter)(struct fb_videomode *mode))
{
}

static inline void tegra_fb_update_fix(struct tegra_fb_info *fb_info,
					struct fb_monspecs *specs)
{
}

static struct fb_var_screeninfo *tegra_fb_get_var(struct tegra_fb_info *fb_info)
{
	return NULL;
}

static inline int tegra_fb_create_sysfs(struct device *dev)
{
	return -ENOENT;
}

static inline void tegra_fb_remove_sysfs(struct device *dev)
{
}

static inline struct tegra_dc_win *tegra_fb_get_win(
				struct tegra_fb_info *tegra_fb)
{
	return NULL;
}

static inline struct tegra_dc_win *tegra_fb_get_blank_win(
				struct tegra_fb_info *tegra_fb)
{
	return NULL;

}

static inline struct fb_videomode *tegra_fb_get_mode(struct tegra_dc *dc)
{
	return NULL;
}
static int tegra_fb_set_win_index(struct tegra_dc *dc, unsigned long win_mask)
{
	return 0;
}
#endif
/* finish merge */

/* tegra_dc_update_windows and tegra_dc_sync_windows do not support windows
 * with differenct dcs in one call
 * dirty_rect is u16[4]: xoff, yoff, width, height
 */
int tegra_dc_update_windows(struct tegra_dc_win *windows[], int n,
	u16 *dirty_rect, bool wait_for_vblank, bool lock_flip);
int tegra_dc_sync_windows(struct tegra_dc_win *windows[], int n);
void tegra_dc_disable_window(struct tegra_dc *dc, unsigned win);
int tegra_dc_attach_win(struct tegra_dc *dc, unsigned idx);
int tegra_dc_dettach_win(struct tegra_dc *dc, unsigned idx);
int tegra_dc_config_frame_end_intr(struct tegra_dc *dc, bool enable);
bool tegra_dc_is_within_n_vsync(struct tegra_dc *dc, s64 ts);
bool tegra_dc_does_vsync_separate(struct tegra_dc *dc, s64 new_ts, s64 old_ts);

int tegra_dc_set_mode(struct tegra_dc *dc, const struct tegra_dc_mode *mode);
struct fb_videomode;
int tegra_dc_to_fb_videomode(struct fb_videomode *fbmode,
	const struct tegra_dc_mode *mode);
int tegra_dc_set_fb_mode(struct tegra_dc *dc, const struct fb_videomode *fbmode,
	bool stereo_mode);
int tegra_dc_set_fbcon_boot_mode(struct tegra_dc *dc);

unsigned tegra_dc_get_out_height(const struct tegra_dc *dc);
unsigned tegra_dc_get_out_width(const struct tegra_dc *dc);
unsigned tegra_dc_get_out_max_pixclock(const struct tegra_dc *dc);
#ifdef CONFIG_TEGRA_NVDISPLAY
void tegra_sd_check_prism_thresh(struct device *dev, int brightness);
void tegra_sd_enbl_dsbl_prism(struct device *dev, bool status);
#else
void nvsd_check_prism_thresh(struct device *dev, int brightness);
void nvsd_enbl_dsbl_prism(struct device *dev, bool status);
#endif

/* PM0 and PM1 signal control */
#define TEGRA_PWM_PM0 0
#define TEGRA_PWM_PM1 1

struct tegra_dc_pwm_params {
	int which_pwm;
	int gpio_conf_to_sfio;
	unsigned int period;
	unsigned int clk_div;
	unsigned int clk_select;
	unsigned int duty_cycle;
};

void tegra_dc_config_pwm(struct tegra_dc *dc, struct tegra_dc_pwm_params *cfg);

int tegra_dsi_send_panel_short_cmd(struct tegra_dc *dc, u8 *pdata, u8 data_len);

#if defined(CONFIG_TEGRA_CSC_V2)
int tegra_nvdisp_update_csc(struct tegra_dc *dc, int win_index);
#else
int tegra_dc_update_csc(struct tegra_dc *dc, int win_index);
#endif

int tegra_dc_update_lut(struct tegra_dc *dc, int win_index, int fboveride);

/*
 * In order to get a dc's current EDID, first call tegra_dc_get_edid() from an
 * interruptible context.  The returned value (if non-NULL) points to a
 * snapshot of the current state; after copying data from it, call
 * tegra_dc_put_edid() on that pointer.  Do not dereference anything through
 * that pointer after calling tegra_dc_put_edid().
 */
struct tegra_dc_edid {
	size_t		len;
	u8		buf[0];
};
struct tegra_dc_edid *tegra_dc_get_edid(struct tegra_dc *dc);
void tegra_dc_put_edid(struct tegra_dc_edid *edid);

int tegra_dc_get_head(const struct tegra_dc *dc);
int tegra_dc_get_out(const struct tegra_dc *dc);
int tegra_dc_get_source_physical_address(u8 *phy_address);

bool tegra_is_bl_display_initialized(int instance);

static inline void find_dc_node(struct device_node **dc1_node,
	struct device_node **dc2_node)
{
	pr_err("%s: function is unimplemented\n", __func__);
}

static inline struct tegra_dc *tegra_get_dc_from_dev(struct device *dev)
{
	return platform_get_drvdata(container_of(dev,
				struct platform_device, dev));
}

void tegra_get_fb_resource(struct resource *fb_res, int instance);
unsigned tegra_dc_out_flags_from_dev(struct device *dev);
bool tegra_dc_initialized(struct device *dev);
bool tegra_dc_is_ext_dp_panel(const struct tegra_dc *dc);

struct tegra_hdmi_out {
	bool hdmi2fpd_bridge_enable;
	bool hdmi2gmsl_bridge_enable;
	bool hdmi2dsi_bridge_enable;
};

enum {
	DRIVE_CURRENT_L0 = 0,
	DRIVE_CURRENT_L1 = 1,
	DRIVE_CURRENT_L2 = 2,
	DRIVE_CURRENT_L3 = 3,
};

enum {
	PRE_EMPHASIS_L0 = 0,
	PRE_EMPHASIS_L1 = 1,
	PRE_EMPHASIS_L2 = 2,
	PRE_EMPHASIS_L3 = 3,
};

enum {
	POST_CURSOR2_L0 = 0,
	POST_CURSOR2_L1 = 1,
	POST_CURSOR2_L2 = 2,
	POST_CURSOR2_L3 = 3,
};

enum {
	SOR_LINK_SPEED_G1_62 = 6,
	SOR_LINK_SPEED_G2_7 = 10,
	SOR_LINK_SPEED_G5_4 = 20,
	SOR_LINK_SPEED_LVDS = 7,
};

struct tegra_dc_dp_lt_settings {
	u32 drive_current[4]; /* Entry for each lane */
	u32 lane_preemphasis[4]; /* Entry for each lane */
	u32 post_cursor[4]; /* Entry for each lane */
	u32 tx_pu;
	u32 load_adj;
};

enum {
	DP_VS = 0,
	DP_PE = 1,
	DP_PC = 2,
	DP_TX_PU = 3,
};

struct tegra_dc_lt_data {
	u32 data[4][4][4];
	const char *name;
};

struct tegra_dp_out {
	struct tegra_dc_dp_lt_settings *lt_settings;
	int n_lt_settings;
	bool tx_pu_disable;
	bool enhanced_framing_disable;
	int lanes;
	u8 link_bw;
	bool hdmi2fpd_bridge_enable;
	int edp2lvds_i2c_bus_no;
	bool edp2lvds_bridge_enable;
	struct tegra_dc_lt_data *lt_data;
	int n_lt_data;
};

/*
 * On T18x, isohub is the only memclient that's relevant to display.
 * The following structs keep track of its isoclient info.
 */
#if defined(CONFIG_TEGRA_NVDISPLAY) && defined(CONFIG_TEGRA_ISOMGR)
struct nvdisp_bandwidth_config {
	u32 iso_bw;		/* KB/s */
	u32 total_bw;		/* KB/s */
	u32 emc_la_floor;	/* Hz */
	u32 hubclk;		/* Hz */
};

struct nvdisp_isoclient_bw_info {
	tegra_isomgr_handle		isomgr_handle;
	struct tegra_bwmgr_client	*bwmgr_handle;

	struct nvdisp_bandwidth_config	max_config;
	struct nvdisp_bandwidth_config	cur_config;
	struct nvdisp_bandwidth_config	usr_config;

	u32				available_bw;		/* KB/s */
	u32				reserved_bw;		/* KB/s */

	u32				emc_at_res_bw;		/* Hz */
	u32				hubclk_at_res_bw;	/* Hz */
};
#endif

/* Timestamp in nsec in TSC timebase */
u64 tegra_dc_get_tsc_time(void);

#if defined(CONFIG_TEGRA_NVDISPLAY)
void tegra_dc_crc_deinit(struct tegra_dc *dc);
void tegra_dc_crc_reset(struct tegra_dc *dc);

/* APIs related to CRC IOCTLs */
long tegra_dc_crc_enable(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg);
long tegra_dc_crc_disable(struct tegra_dc *dc,
			  struct tegra_dc_ext_crc_arg *arg);
long tegra_dc_crc_get(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg);
#else
static inline void tegra_dc_crc_deinit(struct tegra_dc *dc)
{
}

static inline void tegra_dc_crc_reset(struct tegra_dc *dc)
{
}

static inline long tegra_dc_crc_enable(struct tegra_dc *dc,
				       struct tegra_dc_ext_crc_arg *arg)
{
	return 0;
}

static inline long tegra_dc_crc_disable(struct tegra_dc *dc,
					struct tegra_dc_ext_crc_arg *arg)
{
	return 0;
}

static inline long tegra_dc_crc_get(struct tegra_dc *dc,
				    struct tegra_dc_ext_crc_arg *arg)
{
	return 0;
}
#endif

#endif
