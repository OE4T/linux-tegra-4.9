/*
 * drivers/video/tegra/dc/hdmi2.0.h
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DRIVERS_VIDEO_TEGRA_DC_HDMI2_0_H__
#define __DRIVERS_VIDEO_TEGRA_DC_HDMI2_0_H__

#define HDMI_HPD_DEBOUNCE_DELAY_MS	(100)
#define HDMI_SCDC_MONITOR_TIMEOUT_MS	(5000)

/* SCDC block */
#define HDMI_SCDC_TMDS_CONFIG_OFFSET	(0x20)
#define HDMI_SCDC_TMDS_CONFIG_SCRAMBLING_EN	(1)
#define HDMI_SCDC_TMDS_CONFIG_SCRAMBLING_DIS	(1)
#define HDMI_SCDC_TMDS_CONFIG_BIT_CLK_RATIO_10	(0 << 1)
#define HDMI_SCDC_TMDS_CONFIG_BIT_CLK_RATIO_40	(1 << 1)

enum {
	HDMI_INFOFRAME_TYPE_VENDOR = 0x81,
	HDMI_INFOFRAME_TYPE_AVI = 0x82,
	HDMI_INFOFRAME_TYPE_SPD = 0x83,
	HDMI_INFOFRAME_TYPE_AUDIO = 0x84,
	HDMI_INFOFRAME_TYPE_MPEG_SRC = 0x85,
};

enum {
	HDMI_INFOFRAME_VS_VENDOR = 0x1,
	HDMI_INFOFRAME_VS_AVI = 0x2,
	HDMI_INFOFRAME_VS_SPD = 0x1,
	HDMI_INFOFRAME_VS_AUDIO = 0x1,
	HDMI_INFOFRAME_VS_MPEG_SRC = 0x1,
};

/* excluding checksum and header bytes */
enum {
	HDMI_INFOFRAME_LEN_VENDOR, /* vendor specific */
	HDMI_INFOFRAME_LEN_AVI = 13,
	HDMI_INFOFRAME_LEN_SPD = 25,
	HDMI_INFOFRAME_LEN_AUDIO = 10,
	HDMI_INFOFRAME_LEN_MPEG_SRC = 10,
};

enum {
	HDMI_AVI_SCAN_NO_INFO = 0x0,
	HDMI_AVI_OVERSCAN = 0x1,
	HDMI_AVI_UNDERSCAN = 0x2,
};

enum {
	HDMI_AVI_BAR_INVALID = 0x0,
	HDMI_AVI_VERT_BAR_VALID = 0x1,
	HDMI_AVI_HOR_BAR_VALID = 0x2,
	HDMI_AVI_VERT_HOR_BAR_VALID = 0x3,
};

enum {
	HDMI_AVI_ACTIVE_FORMAT_INVALID = 0x0,
	HDMI_AVI_ACTIVE_FORMAT_VALID = 0x1,
};

enum {
	HDMI_AVI_RGB = 0x0,
	HDMI_AVI_YCC_422 = 0x1,
	HDMI_AVI_YCC_444 = 0x2,
	HDMI_AVI_YCC_420 = 0x3,
};

enum {
	HDMI_AVI_ACTIVE_FORMAT_SAME = 0x8,
	HDMI_AVI_ACTIVE_FORMAT_4_3_CENTER = 0x9,
	HDMI_AVI_ACTIVE_FORMAT_16_9_CENTER = 0xa,
	HDMI_AVI_ACTIVE_FORMAT_14_9_CENTER = 0xb,
};

enum {
	HDMI_AVI_ASPECT_RATIO_NO_DATA = 0x0,
	HDMI_AVI_ASPECT_RATIO_4_3 = 0x1,
	HDMI_AVI_ASPECT_RATIO_16_9 = 0x2,
};

enum {
	HDMI_AVI_COLORIMETRY_DEFAULT = 0x0,
	HDMI_AVI_COLORIMETRY_SMPTE170M_ITU601 = 0x1,
	HDMI_AVI_COLORIMETRY_ITU709 = 0x2,
	HDMI_AVI_COLORIMETRY_EXTENDED_VALID = 0x3,
};

enum {
	HDMI_AVI_SCALING_UNKNOWN = 0x0,
	HDMI_AVI_SCALING_HOR = 0x1,
	HDMI_AVI_SCALING_VERT = 0x2,
	HDMI_AVI_SCALING_VERT_HOR = 0x3,
};

enum {
	HDMI_AVI_RGB_QUANT_DEFAULT = 0x0,
	HDMI_AVI_RGB_QUANT_LIMITED = 0x1,
	HDMI_AVI_RGB_QUANT_FULL = 0x2,
};

enum {
	HDMI_AVI_EXT_COLORIMETRY_INVALID = 0x0,
	HDMI_AVI_EXT_COLORIMETRY_xvYCC601 = 0x0,
	HDMI_AVI_EXT_COLORIMETRY_xvYCC709 = 0x1,
	HDMI_AVI_EXT_COLORIMETRY_sYCC601 = 0x2,
	HDMI_AVI_EXT_COLORIMETRY_ADOBE_YCC601 = 0x3,
	HDMI_AVI_EXT_COLORIMETRY_ADOBE_RGB = 0x4,
	HDMI_AVI_EXT_COLORIMETRY_BT2020_CYCC = 0x5,
	HDMI_AVI_EXT_COLORIMETRY_BT2020_YCC_RGB = 0x6,
};

enum {
	HDMI_AVI_IT_CONTENT_FALSE = 0x0,
	HDMI_AVI_IT_CONTENT_TRUE = 0x0,
};

enum {
	HDMI_AVI_NO_PIX_REPEAT = 0x0,
};

enum {
	HDMI_AVI_IT_CONTENT_NONE = 0x0,
	HDMI_AVI_IT_CONTENT_GRAPHICS = 0x0,
	HDMI_AVI_IT_CONTENT_PHOTO = 0x1,
	HDMI_AVI_IT_CONTENT_CINEMA = 0x2,
	HDMI_AVI_IT_CONTENT_GAME = 0x3,
};

enum {
	HDMI_AVI_YCC_QUANT_NONE = 0x0,
	HDMI_AVI_YCC_QUANT_LIMITED = 0x0,
	HDMI_AVI_YCC_QUANT_FULL = 0x1,
};

/* all fields little endian */
struct hdmi_avi_infoframe {
	/* PB0 */
	u32 csum:8;	/* checksum */

	/* PB1 */
	u32 scan:2;	/* scan information */
	u32 bar_valid:2;	/* bar info data valid */
	u32 act_fmt_valid:1;	/* active info present */
	u32 rgb_ycc:2;	/* RGB or YCbCr */
	u32 res1:1;	/* reserved */

	/* PB2 */
	u32 act_format:4;	/* active format aspect ratio */
	u32 aspect_ratio:2;	/* picture aspect ratio */
	u32 colorimetry:2;	/* colorimetry */

	/* PB3 */
	u32 scaling:2;	/* non-uniform picture scaling */
	u32 rgb_quant:2;	/* rgb quantization range */
	u32 ext_colorimetry:3;	/* extended colorimetry */
	u32 it_content:1;	/* it content */

	/* PB4 */
	u32 video_format:7; /* video format id code */
	u32 res4:1;	/* reserved */

	/* PB5 */
	u32 pix_rep:4;	/* pixel repetition factor */
	u32 it_content_type:2;	/* it content type */
	u32 ycc_quant:2;	/* YCbCr quantization range */

	/* PB6-7 */
	u32 top_bar_end_line_low_byte:8;
	u32 reg_hole1:8;
	u32 top_bar_end_line_high_byte:8;

	/* PB8-9 */
	u32 bot_bar_start_line_low_byte:8;
	u32 bot_bar_start_line_high_byte:8;

	/* PB10-11 */
	u32 left_bar_end_pixel_low_byte:8;
	u32 left_bar_end_pixel_high_byte:8;

	/* PB12-13 */
	u32 right_bar_start_pixel_low_byte:8;
	u32 right_bar_start_pixel_high_byte:8;

	u32 reg_hole2:8;
} __packed;

enum {
	HDMI_AUDIO_CHANNEL_CNT_STREAM,	/* refer to audio stream header */
	HDMI_AUDIO_CHANNEL_CNT_2,
	HDMI_AUDIO_CHANNEL_CNT_3,
	HDMI_AUDIO_CHANNEL_CNT_4,
	HDMI_AUDIO_CHANNEL_CNT_5,
	HDMI_AUDIO_CHANNEL_CNT_6,
	HDMI_AUDIO_CHANNEL_CNT_7,
	HDMI_AUDIO_CHANNEL_CNT_8,
};

/* all fields little endian */
struct hdmi_audio_infoframe {
	/* PB0 */
	u32 csum:8;	/* checksum */

	/* PB1 */
	u32 channel_cnt:3;
	u32 res1:1;	/* reserved */
	u32	coding_type:4;	/* coding type */

	/* PB2 */
	u32 sample_size:2;
	u32 sample_freq:3;
	u32 res2:3;	/* reserved */

	/* PB3 */
	u32 res3:8; /* reserved */

	/* PB4 */
	u32 channel_alloc:8;	/* channel/speaker allocation */

	/* PB5 */
	u32 low_freq_effect_level:2;	/* low freq effect playback level */
	u32 res4:1;	/* reserved */
	u32	level_sft_val:4;	/* level shift value */
	u32 downmix_inhibit:1;

	u32 reg_hole1:16;
} __packed;

#define HDMI_LICENSING_LLC_OUI	(0x000c03)

enum {
	HDMI_VENDOR_VIDEO_FORMAT_NONE,
	HDMI_VENDOR_VIDEO_FORMAT_EXTENDED,
	HDMI_VENDOR_VIDEO_FORMAT_3D,
};

/* all fields little endian */
struct hdmi_vendor_infoframe {
	/* PB0 */
	u32 csum:8;

	/* PB1, PB2, PB3 */
	u32 oui:24;	/* organizationally unique identifier */

	/* PB4 */
	u32 res1:5;
	u32 video_format:3;

	/* PB5 */
	union {
		u32 extended_vic:8;
		u32 res2:4;
		u32 format_3d:4;
	} __packed;

	/* PB6 */
	u32 res3:4;
	u32 ext_data_3d:4;
} __packed;

enum {
	TEGRA_HDMI_SAFE_CLK = 1,
	TEGRA_HDMI_BRICK_CLK = 2,
};

struct tegra_hdmi {
	struct tegra_dc *dc;
	struct tegra_hdmi_out *pdata;
	struct tegra_dc_sor_data *sor;
	struct hdmi_avi_infoframe avi;
	bool enabled;
	int clock_refcount;

	bool dvi;

	u32 clk_type;

	struct tegra_edid_hdmi_eld eld;
	bool eld_valid;

	struct fb_monspecs mon_spec;
	bool mon_spec_valid;

	struct tegra_edid *edid;
	struct i2c_client *ddc_i2c_client;
	struct mutex ddc_lock;

	struct i2c_client *scdc_i2c_client;
	struct delayed_work scdc_work;

	struct hdmi_audio_infoframe audio;
	bool null_sample_inject;
	u32 audio_freq;
	struct clk *hda_clk;
	struct clk *hda2codec_clk;
	struct clk *hda2hdmi_clk;
#ifdef CONFIG_SWITCH
	struct switch_dev hpd_switch;
	struct switch_dev audio_switch;
#endif

	struct hdmi_vendor_infoframe vsi;

	struct tegra_nvhdcp *nvhdcp;

	struct delayed_work  hpd_worker;
	struct mutex hpd_lock;

	int ddc_i2c_original_rate;
	int irq;
	struct tegra_prod_list *prod_list;
	int ddc_refcount;
	bool device_shutdown;
};

#define HDMI_ELD_BUF 96

/* eld field indexes */
enum {
	HDMI_ELD_VER = 0,
	HDMI_ELD_BASELINE_ELD_LEN = 2,
	HDMI_ELD_CEA_EDID_VER_MNL = 4,
	HDMI_ELD_SAD_CNT_CON_TYPE_S_AI_S_HDCP = 5,
	HDMI_ELD_AUDIO_SYNC_DELAY = 6,
	HDMI_ELD_RLRC_FLRC_RC_RLR_FC_LFE_FLR = 7,
	HDMI_ELD_PORT_ID = 8, /* 8 bytes */
	HDMI_ELD_MANUFACTURER_NAME = 16, /* 2 bytes */
	HDMI_ELD_PRODUCT_CODE = 18, /* 2 bytes */
	HDMI_ELD_MONITOR_NAME_STR = 20, /* MNL bytes */
	HDMI_ELD_CEA_SAD, /* SAD_CNT * 3 bytes, index depends on MNL */
};

/* bpp without alpha */
enum {
	TEGRA_HDMI_BPP_UNKNOWN = 0,
	TEGRA_HDMI_BPP_24 = 4,
	TEGRA_HDMI_BPP_30 = 5,
	TEGRA_HDMI_BPP_36 = 6,
	TEGRA_HDMI_BPP_48 = 7,
};

#endif
