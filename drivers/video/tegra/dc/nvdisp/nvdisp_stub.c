/*
 * drivers/video/tegra/dc/nvdisplay/nvdis_stub.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All rights reserved.
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

/* Define stub functions to allow compilation */

#include <linux/types.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/clk/tegra.h>
#include <linux/of.h>

#include "mach/latency_allowance.h"
#include "mach/io_dpd.h"
#include "mach/dc.h"
#include "board.h"
#include "board-panel.h"

const struct disp_client *tegra_la_disp_clients_info;
atomic_t sd_brightness = ATOMIC_INIT(255);
EXPORT_SYMBOL(sd_brightness);

bool tegra_powergate_is_powered(int id)
{
	return true;
}

int tegra_is_clk_enabled(struct clk *c)
{
	return 1;
}

/* Update this after carve out is defined */
void tegra_get_fb_resource(struct resource *fb_res)
{
	fb_res->start = 0;
	fb_res->end = 0;
}

void tegra_get_fb2_resource(struct resource *fb2_res)
{
	fb2_res->start = 0;
	fb2_res->end = 0;
}

int tegra_dvfs_set_rate(struct clk *c, unsigned long rate)
{
	return 0;
}

int tegra_powergate_partition(int id)
{
	return 0;
}

int tegra_unpowergate_partition(int id)
{
	return 0;
}

unsigned int tegra_emc_bw_to_freq_req(unsigned int bw_kbps)
{
	return 0;
}

unsigned int tegra_emc_freq_req_to_bw(unsigned int freq_khz)
{
	return 0;
}

u32 tegra_get_dvfs_clk_change_latency_nsec(unsigned long emc_freq_khz)
{
	return 0;
}

int tegra_set_disp_latency_allowance(enum tegra_la_id id,
	unsigned long emc_freq_hz,
	unsigned int bw_mbps,
	struct dc_to_la_params disp_params)
{
	return 0;
}

int tegra_mc_get_tiled_memory_bandwidth_multiplier(void)
{
	return 0;
}

int tegra_dvfs_use_alt_freqs_on_clk(struct clk *c, bool use_alt_freq)
{
	return 0;
}

struct la_to_dc_params tegra_get_la_to_dc_params(void)
{
	static struct la_to_dc_params la;
	return la;
}

void tegra_io_dpd_enable(struct tegra_io_dpd *hnd)
{
}

void tegra_io_dpd_disable(struct tegra_io_dpd *hnd)
{
}


static void tegra_panel_register_ops(struct tegra_dc_out *dc_out,
				struct tegra_panel_ops *p_ops)
{
	BUG_ON(!dc_out);

	dc_out->enable = p_ops->enable;
	dc_out->postpoweron = p_ops->postpoweron;
	dc_out->prepoweroff = p_ops->prepoweroff;
	dc_out->disable = p_ops->disable;
	dc_out->hotplug_init = p_ops->hotplug_init;
	dc_out->postsuspend = p_ops->postsuspend;
	dc_out->hotplug_report = p_ops->hotplug_report;
}

extern struct tegra_panel_ops panel_sim_ops;

/* Clone from board-panel.c */
struct device_node *tegra_primary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct tegra_dc_out *dc_out = NULL;
	/* struct board_info display_board; */

	/* tegra_get_display_board_info(&display_board); */

	if (pdata)
		dc_out = pdata->default_out;

	if (pdata && dc_out)
		tegra_panel_register_ops(dc_out, &panel_sim_ops);
	np_panel = of_find_compatible_node(NULL, NULL, "nvidia,sim-panel");

	if (!np_panel)
		pr_err("Could not find node nvidia,sim-panel\n");
	return of_device_is_available(np_panel) ? np_panel : NULL;
}

struct device_node *tegra_secondary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct device_node *np_hdmi =
		of_find_node_by_path(HDMI_NODE);
	struct tegra_dc_out *dc_out = NULL;

	if (pdata)
		dc_out = pdata->default_out;

	np_panel =
		of_get_child_by_name(np_hdmi, "hdmi-display");

	return of_device_is_available(np_panel) ? np_panel : NULL;
}


