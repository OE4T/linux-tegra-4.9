/*
 * drivers/video/tegra/dc/nvdisp/nvdisp_t19x.c
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
#include "nvdisp.h"
#include "hw_nvdisp_nvdisp.h"

static struct tegra_dc_pd_clk_info t19x_disp_pd0_clk_info[] = {
	{
		.name = "nvdisplayhub",
		.clk = NULL,
	},
	{
		.name = "nvdisplay_disp",
		.clk = NULL,
	},
	{
		.name = "nvdisplay_p0",
		.clk = NULL,
	},
};

static struct tegra_dc_pd_clk_info t19x_disp_pd1_clk_info[] = {
	{
		.name = "nvdisplay_p1",
		.clk = NULL,
	},
};

static struct tegra_dc_pd_clk_info t19x_disp_pd2_clk_info[] = {
	{
		.name = "nvdisplay_p2",
		.clk = NULL,
	},
	{
		.name = "nvdisplay_p3",
		.clk = NULL,
	},
};

/*
 * NOTE: Keep the following power domains ordered according to their head owner.
 */
static struct tegra_dc_pd_info t19x_disp_pd_info[] = {
	/* Head0 power domain */
	{
		.of_id = {
			{ .compatible = "nvidia,tegra194-disa-pd", },
			{},
		},
		.pg_id = -1,
		.head_owner = 0,
		.head_mask = 0x1,	/* Head(s):	0 */
		.win_mask = 0x1,	/* Window(s):	0 */
		.domain_clks = t19x_disp_pd0_clk_info,
		.nclks = ARRAY_SIZE(t19x_disp_pd0_clk_info),
		.ref_cnt = 0,
	},
	/* Head1 power domain */
	{
		.of_id = {
			{ .compatible = "nvidia,tegra194-disb-pd", },
			{},
		},
		.pg_id = -1,
		.head_owner = 1,
		.head_mask = 0x2,	/* Head(s):	1 */
		.win_mask = 0x6,	/* Window(s):	1,2 */
		.domain_clks = t19x_disp_pd1_clk_info,
		.nclks = ARRAY_SIZE(t19x_disp_pd1_clk_info),
		.ref_cnt = 0,
	},
	/* Head2 power domain */
	{
		.of_id = {
			{ .compatible = "nvidia,tegra194-disc-pd", },
			{},
		},
		.pg_id = -1,
		.head_owner = 2,
		.head_mask = 0xc,	/* Head(s):	2,3 */
		.win_mask = 0x38,	/* Window(s):	3,4,5 */
		.domain_clks = t19x_disp_pd2_clk_info,
		.nclks = ARRAY_SIZE(t19x_disp_pd2_clk_info),
		.ref_cnt = 0,
	},
};

static struct tegra_dc_pd_table t19x_disp_pd_table = {
	.pd_entries = t19x_disp_pd_info,
	.npd = ARRAY_SIZE(t19x_disp_pd_info),
};

int tegra_nvdisp_set_control_t19x(struct tegra_dc *dc)
{
	u32 reg, protocol;

	if (dc->out->type == TEGRA_DC_OUT_HDMI) {

		/* sor1 in the function name is irrelevant */
		protocol = nvdisp_sor1_control_protocol_tmdsa_f();
	} else if ((dc->out->type == TEGRA_DC_OUT_DP) ||
		   (dc->out->type == TEGRA_DC_OUT_NVSR_DP) ||
		   (dc->out->type == TEGRA_DC_OUT_FAKE_DP)) {

		/* sor in the function name is irrelevant */
		protocol = nvdisp_sor_control_protocol_dpa_f();
	} else {
		dev_err(&dc->ndev->dev, "%s: unsupported out_type=%d\n",
				__func__, dc->out->type);
		return -EINVAL;
	}

	switch (dc->out_ops->get_connector_instance(dc)) {
	case 0:
		reg = nvdisp_sor_control_r();
		break;
	case 1:
		reg = nvdisp_sor1_control_r();
		break;
	case 2:
		reg = 0x00000405; /* todo: add t19x hw_reg headerfile */
		break;
	case 3:
		reg = 0x00000406; /* todo: add t19x hw_reg headerfile */
		break;
	default:
		pr_err("%s: invalid sor_num:%d\n", __func__,
				dc->out_ops->get_connector_instance(dc));
		return -ENODEV;
	}

	tegra_dc_writel(dc, protocol, reg);
	tegra_dc_enable_general_act(dc);

	return 0;
}

void tegra_dc_enable_sor_t19x(struct tegra_dc *dc, int sor_num, bool enable)
{
	u32 enb;
	u32 reg_val = tegra_dc_readl(dc, nvdisp_win_options_r());

	/* todo: add t19x hw_reg headerfile */
	switch (sor_num) {
	case 0:
		enb = nvdisp_win_options_sor_set_sor_enable_f();
		break;
	case 1:
		enb = nvdisp_win_options_sor1_set_sor1_enable_f();
		break;
	case 2:
		enb = BIT(28); /* todo: add t19x hw_reg headerfile */
		break;
	case 3:
		enb = BIT(29); /* todo: add t19x hw_reg headerfile */
		break;
	default:
		pr_err("%s: invalid sor_num:%d\n", __func__, sor_num);
		return;
	}

	reg_val = enable ? reg_val | enb : reg_val & ~enb;
	tegra_dc_writel(dc, reg_val, nvdisp_win_options_r());
}

void tegra_dc_populate_t19x_hw_data(struct tegra_dc_hw_data *hw_data)
{
	if (!hw_data)
		return;

	hw_data->nheads = 4;
	hw_data->nwins = 6;
	hw_data->nsors = 4;
	hw_data->pd_table = &t19x_disp_pd_table;
	hw_data->valid = true;
	hw_data->version = TEGRA_DC_HW_T19x;
}
