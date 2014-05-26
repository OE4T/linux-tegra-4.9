/*
 * drivers/video/tegra/dc/hdmi2.0.c
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/clk/tegra.h>

#include <mach/dc.h>
#include <linux/nvhost.h>

#include "dc_reg.h"
#include "dc_priv.h"
#include "hdmi2.0.h"
#include "sor.h"
#include "sor_regs.h"

static int tegra_dc_hdmi_init(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi;
	int err;

	hdmi = devm_kzalloc(&dc->ndev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->sor = tegra_dc_sor_init(dc, NULL);
	if (IS_ERR_OR_NULL(hdmi->sor)) {
		err = PTR_ERR(hdmi->sor);
		goto fail;
	}

	hdmi->pdata = dc->pdata->default_out->hdmi_out;
	hdmi->dc = dc;
	hdmi->enabled = false;

	tegra_dc_set_outdata(dc, hdmi);

	return 0;

fail:
	devm_kfree(&dc->ndev->dev, hdmi);
	return err;
}

static void tegra_dc_hdmi_destroy(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	tegra_dc_sor_destroy(hdmi->sor);
	devm_kfree(&dc->ndev->dev, hdmi);
}

static void tegra_hdmi_config(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;
	struct tegra_dc *dc = hdmi->dc;

	tegra_sor_write_field(sor, NV_SOR_INPUT_CONTROL,
			NV_SOR_INPUT_CONTROL_ARM_VIDEO_RANGE_LIMITED |
			NV_SOR_INPUT_CONTROL_HDMI_SRC_SELECT_DISPLAYB,
			NV_SOR_INPUT_CONTROL_ARM_VIDEO_RANGE_LIMITED |
			NV_SOR_INPUT_CONTROL_HDMI_SRC_SELECT_DISPLAYB);

	/* TODO: fix hardcoding */
	tegra_sor_writel(sor, NV_SOR_REFCLK, 6912);
	tegra_sor_writel(sor, NV_SOR_HDMI_CTRL, 0x40020038);

	tegra_dc_writel(dc, 0x180, DC_DISP_H_PULSE2_CONTROL);
	tegra_dc_writel(dc, 0x790071, DC_DISP_H_PULSE2_POSITION_A);
	tegra_dc_writel(dc, 0x1000, DC_DISP_DISP_SIGNAL_OPTIONS0);
}

static void tegra_hdmi_infoframe_pkt_write(struct tegra_hdmi *hdmi,
						u32 header_reg, u8 pkt_type,
						u8 pkt_vs, u8 pkt_len,
						void *reg_payload,
						u32 reg_payload_len)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;
	u32 val;
	u32 *data = reg_payload;
	u32 data_reg = header_reg + 1;

	val = NV_SOR_HDMI_INFOFRAME_HEADER_TYPE(pkt_type) |
		NV_SOR_HDMI_INFOFRAME_HEADER_VERSION(pkt_vs) |
		NV_SOR_HDMI_INFOFRAME_HEADER_LEN(pkt_len);
	tegra_sor_writel(sor, header_reg, val);

	for (val = 0; val < reg_payload_len; val += 4, data_reg++, data++)
		tegra_sor_writel(sor, data_reg, *data);
}

static void tegra_hdmi_avi_infoframe_update(struct tegra_hdmi *hdmi)
{
	struct hdmi_avi_infoframe *avi = &hdmi->avi;

	avi->act_fmt_valid = 1;
	avi->act_format = HDMI_AVI_ACTIVE_FORMAT_SAME;

	/* TODO: read from edid */
	avi->video_format = 0x3;
}

static void tegra_hdmi_avi_infoframe(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;

	/* disable avi infoframe before configuring */
	tegra_sor_writel(sor, NV_SOR_HDMI_AVI_INFOFRAME_CTRL, 0);

	tegra_hdmi_avi_infoframe_update(hdmi);

	tegra_hdmi_infoframe_pkt_write(hdmi, NV_SOR_HDMI_AVI_INFOFRAME_HEADER,
					HDMI_INFOFRAME_TYPE_AVI,
					HDMI_INFOFRAME_VS_AVI,
					HDMI_INFOFRAME_LEN_AVI,
					&hdmi->avi, sizeof(hdmi->avi));

	/* Send infoframe every frame, checksum hw generated */
	tegra_sor_writel(sor, NV_SOR_HDMI_AVI_INFOFRAME_CTRL,
		NV_SOR_HDMI_AVI_INFOFRAME_CTRL_ENABLE_YES |
		NV_SOR_HDMI_AVI_INFOFRAME_CTRL_OTHER_DISABLE |
		NV_SOR_HDMI_AVI_INFOFRAME_CTRL_SINGLE_DISABLE |
		NV_SOR_HDMI_AVI_INFOFRAME_CTRL_CHECKSUM_ENABLE);
}

static void tegra_hdmi_audio_infoframe(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;
	int i;

	tegra_sor_writel(sor, NV_SOR_HDMI_ACR_CTRL, 0x0);
	tegra_sor_writel(sor, NV_SOR_HDMI_AUDIO_INFOFRAME_HEADER, 0xa0184);
	tegra_sor_writel(sor, NV_SOR_HDMI_AUDIO_INFOFRAME_SUBPACK0_LOW, 0x100);
	tegra_sor_writel(sor, NV_SOR_HDMI_AUDIO_INFOFRAME_SUBPACK0_HIGH, 0x00);
	tegra_sor_writel(sor, NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL, 0x201);

	for (i = 0; i < 7; i++)
		tegra_sor_writel(sor,
				NV_SOR_HDMI_ACR_0320_SUBPACK_HIGH + (i * 2),
				0x80001800);

	tegra_sor_writel(sor, NV_SOR_HDMI_SPARE, 0x10001);
	tegra_sor_writel(sor, NV_SOR_AUDIO_CTRL, 0x21000);

	for (i = 0; i <= 7; i++)
		tegra_sor_writel(sor, NV_SOR_AUDIO_AVAL_0320 + i, 0x5dc0);
}

static void tegra_hdmi_config_lanes(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;

	tegra_sor_writel(sor, NV_SOR_LANE_DRIVE_CURRENT(sor->portnum),
			0xffffffff);
	tegra_sor_writel(sor, NV_SOR_PR(sor->portnum), 0x0);
}

static void tegra_dc_hdmi_enable(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
	struct tegra_dc_sor_data *sor = hdmi->sor;

	clk_prepare_enable(sor->sor_clk);
	tegra_dc_io_start(dc);

	tegra_sor_config_safe_clk(sor);
	clk_prepare_enable(sor->safe_clk);

	tegra_sor_hdmi_pad_power_up(sor);

	tegra_hdmi_config_lanes(hdmi);
	tegra_sor_power_lanes(sor, 4, true);

	tegra_sor_config_hdmi_clk(sor);

	tegra_dc_sor_set_internal_panel(sor, false);
	tegra_hdmi_config(hdmi);
	tegra_hdmi_avi_infoframe(hdmi);
	tegra_hdmi_audio_infoframe(hdmi);

	tegra_dc_sor_attach(sor);
	tegra_dc_io_end(dc);
}

static long tegra_dc_hdmi_setup_clk(struct tegra_dc *dc, struct clk *clk)
{
	return tegra_dc_pclk_round_rate(dc, dc->mode.pclk);
}

static void tegra_dc_hdmi_disable(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
	struct tegra_dc_sor_data *sor = hdmi->sor;

	clk_disable_unprepare(sor->safe_clk);
	clk_disable_unprepare(sor->sor_clk);
}

struct tegra_dc_out_ops tegra_dc_hdmi2_0_ops = {
	.init = tegra_dc_hdmi_init,
	.destroy = tegra_dc_hdmi_destroy,
	.enable = tegra_dc_hdmi_enable,
	.disable = tegra_dc_hdmi_disable,
	.setup_clk = tegra_dc_hdmi_setup_clk,
};
