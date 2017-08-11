/*
 * hda_dc.h: hd audio dc driver.
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
 * Author: Rahul Mittal <rmittal@nvidia.com>
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_HDA_DC_H__
#define __DRIVERS_VIDEO_TEGRA_DC_HDA_DC_H__

enum {
	SINK_HDMI = 0,
	SINK_DP = 1,
};

struct tegra_dc_hda_data {
	bool valid; /* set to true only after tegra_hda_set_data */
	int dev_id;
	struct tegra_dc_sor_data *sor;
	struct tegra_dc *dc;
	struct tegra_edid_hdmi_eld *eld;
	int sink;
	bool null_sample_inject;
	bool *eld_valid;
	bool *enabled;
	u32 audio_freq;
	struct clk *pll_p_clk;
	struct clk *hda_clk;
	struct clk *hda2codec_clk;
	struct clk *hda2hdmi_clk;
	struct clk *maud_clk;
	void *client_data;
};

void *tegra_hda_set_data(struct tegra_dc *dc, void *data, int sink);
void tegra_hda_reset_data(void *hda_handle);
int tegra_hdmi_setup_hda_presence(int sor_num);
int tegra_hda_get_dev_id(struct tegra_dc_sor_data *sor);

#endif
