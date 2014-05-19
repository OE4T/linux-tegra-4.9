/*
 * kernel/drivers/video/tegra/dc/fake_panel.c
 *
 * Copyright (c) 2014-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <generated/mach-types.h>
#include "fake_panel.h"

#define TEGRA_DSI_GANGED_MODE	0

#define DSI_PANEL_RESET		1

#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
static struct resource all_disp1_resources[] = {
	{
		/* keep fbmem as first variable in array for
		 * easy replacement
		 */
		.name	= "fbmem",
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "ganged_dsia_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "ganged_dsib_regs",
		.start	= TEGRA_DSIB_BASE,
		.end	= TEGRA_DSIB_BASE + TEGRA_DSIB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* init with dispa reg base*/
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "mipi_cal",
		.start	= TEGRA_MIPI_CAL_BASE,
		.end	= TEGRA_MIPI_CAL_BASE + TEGRA_MIPI_CAL_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sor0",
		.start  = TEGRA_SOR_BASE,
		.end    = TEGRA_SOR_BASE + TEGRA_SOR_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "sor1",
		.start  = TEGRA_SOR1_BASE,
		.end    = TEGRA_SOR1_BASE + TEGRA_SOR1_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "dpaux",
		.start  = TEGRA_DPAUX_BASE,
		.end    = TEGRA_DPAUX_BASE + TEGRA_DPAUX_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name	= "irq_dp",
		.start	= INT_DPAUX,
		.end	= INT_DPAUX,
		.flags	= IORESOURCE_IRQ,
	},

};
#endif

static int tegra_dc_add_fakedisp_resources(struct platform_device *ndev)
{
	/* Copy the existing fbmem resources locally
	 * and replace the existing resource pointer
	 * with local array
	 */
	struct resource *resources  = ndev->resource;
	int i;
	for (i = 0; i < ndev->num_resources; i++) {
		struct resource *r = &resources[i];
		if (resource_type(r) == IORESOURCE_MEM &&
			!strcmp(r->name, "fbmem")) {
			if (!strcmp(all_disp1_resources[0].name, "fbmem")) {
				all_disp1_resources[0].flags = r->flags;
				all_disp1_resources[0].start = r->start;
				all_disp1_resources[0].end  = r->end;
			} else
				pr_info("Error - First variable is not fbmem\n");
		}
	}
	ndev->resource = all_disp1_resources;
	ndev->num_resources = ARRAY_SIZE(all_disp1_resources);

	return 0;
}

static struct tegra_dsi_out dsi_fake_panel_pdata = {
	.controller_vs = DSI_VS_1,
	.n_data_lanes = 4,

#if DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
	.ganged_type = TEGRA_DSI_GANGED_SYMMETRIC_LEFT_RIGHT,
	.suspend_aggr = DSI_HOST_SUSPEND_LV2,
	.refresh_rate = 61,
	.rated_refresh_rate = 60,
	.te_polarity_low = true,
#else
	/*.ganged_type = TEGRA_DSI_GANGED_SYMMETRIC_EVEN_ODD,*/
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,
	.refresh_rate = 60,
#endif

	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,

};

static struct tegra_dc_mode dsi_fake_panel_modes[] = {
	{
#if DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE
		.pclk = 294264000, /* @61Hz*/
		.h_ref_to_sync = 0,

		/* dc constraint, min 1*/
		.v_ref_to_sync = 1,

		.h_sync_width = 32,

		/* dc constraint, min 1*/
		.v_sync_width = 1,

		.h_back_porch = 80,

		/* panel constraint, send frame after TE deassert*/
		.v_back_porch = 5,

		.h_active = 2560,
		.v_active = 1600,
		.h_front_porch = 328,

		/* dc constraint, min v_ref_to_sync + 1*/
		.v_front_porch = 2,
#else
		.pclk = 154700000, /* @60Hz*/
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 2,
		.h_back_porch = 32,
		.v_back_porch = 16,
		.h_active = 1920,
		.v_active = 1200,
		.h_front_porch = 120,
		.v_front_porch = 17,
#endif
	},
};

int tegra_dc_init_fake_panel_link_cfg(struct tegra_dc_dp_link_config *cfg)
{
	/*
	 * Currently fake the values for testing - same as eDp
	 * will need to add a method to update as needed
	 */
	cfg->max_lane_count = 4;
	cfg->tps3_supported = false;
	cfg->support_enhanced_framing = true;
	cfg->downspread = true;
	cfg->support_fast_lt = true;
	cfg->aux_rd_interval = 0;
	cfg->alt_scramber_reset_cap = true;
	cfg->only_enhanced_framing = true;
	cfg->edp_cap = true;
	cfg->max_link_bw = 20;
	cfg->scramble_ena = 0;
	cfg->lt_data_valid = 0;

	return 0;
}

int tegra_dc_init_fakedp_panel(struct tegra_dc *dc)
{
	struct tegra_dc_out *dc_out = dc->out;
	/* Set the needed resources */
	tegra_dc_add_fakedisp_resources(dc->ndev);
	dc_out->align = TEGRA_DC_ALIGN_MSB,
	dc_out->order = TEGRA_DC_ORDER_RED_BLUE,
	dc_out->flags = TEGRA_DC_OUT_CONTINUOUS_MODE;
	dc_out->out_pins = NULL;
	dc_out->n_out_pins = 0;
	dc_out->depth = 18;
	dc_out->parent_clk = "pll_d_out0";
	dc_out->enable = NULL;
	dc_out->disable = NULL;
	dc_out->postsuspend = NULL;
	dc_out->hotplug_gpio = -1;
	dc_out->postpoweron = NULL;

	return 0;
}

int tegra_dc_init_fakedsi_panel(struct tegra_dc *dc)
{
	struct tegra_dc_out *dc_out = dc->out;
	/* Set the needed resources */
	tegra_dc_add_fakedisp_resources(dc->ndev);
	dc_out->dsi = &dsi_fake_panel_pdata;
	dc_out->parent_clk = "pll_d_out0";
	dc_out->modes = dsi_fake_panel_modes;
	dc_out->n_modes = ARRAY_SIZE(dsi_fake_panel_modes);
	dc_out->enable = NULL;
	dc_out->postpoweron = NULL;
	dc_out->disable = NULL;
	dc_out->postsuspend	= NULL;
	dc_out->width = 217;
	dc_out->height = 135;
	dc_out->flags = TEGRA_DC_OUT_CONTINUOUS_MODE;

	return 0;
}

