/*
 * drivers/video/tegra/dc/hdmi2.0.h
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All rights reserved.
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
	HDMI_AVI_ACTIVE_FORMAT_SAME = 0x8,
	HDMI_AVI_ACTIVE_FORMAT_4_3_CENTER = 0x9,
	HDMI_AVI_ACTIVE_FORMAT_16_9_CENTER = 0xa,
	HDMI_AVI_ACTIVE_FORMAT_14_9_CENTER = 0xb,
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
	u32 content_type:2;	/* content type*/
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

struct tegra_hdmi {
	struct tegra_dc *dc;
	struct tegra_hdmi_out *pdata;
	bool enabled;
	struct tegra_dc_sor_data *sor;
	struct hdmi_avi_infoframe avi;

	struct tegra_edid_hdmi_eld eld;
	bool eld_valid;

	struct tegra_edid *edid;
	struct i2c_client *ddc_i2c_client;

	struct hdmi_audio_infoframe audio;
	bool null_sample_inject;
	u32 audio_freq;
	struct clk *hda_clk;
	struct clk *hda2codec_clk;
	struct clk *hda2hdmi_clk;

	struct tegra_nvhdcp *nvhdcp;

	struct work_struct hpd_worker;
	bool hpd_in_progress;
	struct mutex hpd_lock;
	struct {
		u32 dc_enable:1;
		u32 hdmi_host_enable:1;
		u32 edid_eld_read:1;
		u32 pix_stream_on:1;
	} hpd_state_status;
};

#endif
