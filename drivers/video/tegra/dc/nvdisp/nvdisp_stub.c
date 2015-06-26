/*
 * drivers/video/tegra/dc/nvdisplay/nvdis_stub.c
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/tegra-soc.h>

#include <linux/platform/tegra/latency_allowance.h>

#include "mach/io_dpd.h"
#include "mach/dc.h"
#include "board.h"
#include "board-panel.h"

const struct disp_client *tegra_la_disp_clients_info;
atomic_t sd_brightness = ATOMIC_INIT(255);
EXPORT_SYMBOL(sd_brightness);

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

int tegra_dvfs_use_alt_freqs_on_clk(struct clk *c, bool use_alt_freq)
{
	return 0;
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

/* Fix T18x kernel-only build - Cloned from common.c */
/* returns true if bl initialized the display */
bool tegra_is_bl_display_initialized(int instance)
{
	return false;
}

/* Clone from board-panel.c */
struct device_node *tegra_primary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct tegra_dc_out *dc_out = NULL;
	struct device_node *np_sor =
			of_find_node_by_path(SOR_NODE);

	if (pdata)
		dc_out = pdata->default_out;

	if (pdata && dc_out)
		tegra_panel_register_ops(dc_out, &panel_sim_ops);

	if (tegra_platform_is_silicon()) {
		/* Take from new DTS file */
	} else if (tegra_platform_is_fpga()) {
		/* Using Hdmi in sor 0 node */
		np_panel =
			of_get_child_by_name(np_sor, "hdmi-display");
	} else if (tegra_platform_is_unit_fpga())
		np_panel = of_get_child_by_name(np_sor, "dp-ufpga-panel");
	else if (tegra_platform_is_linsim())
		np_panel = of_get_child_by_name(np_sor, "panel-nvidia-sim");

	if (!np_panel)
		pr_err("Could not find node sim-panel\n");
	return of_device_is_available(np_panel) ? np_panel : NULL;
}

struct device_node *tegra_secondary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct tegra_dc_out *dc_out = NULL;

	struct device_node *np_hdmi =
			of_find_node_by_path(HDMI_NODE);
	if (pdata)
		dc_out = pdata->default_out;

#if 0 /* DSI */
	struct device_node *np_dsi =
			of_find_node_by_path(DSI_NODE);
	if (pdata && dc_out)
		tegra_panel_register_ops(dc_out, &panel_sim_ops);

	np_panel = of_get_child_by_name(np_dsi, "panel-s-wqxga-10-1");
#endif

	if (tegra_platform_is_unit_fpga()) {
		np_panel =
			of_get_child_by_name(np_hdmi, "hdmi-display");
	}
	else if (tegra_platform_is_linsim()) {
		if (pdata && dc_out)
			tegra_panel_register_ops(dc_out, &panel_sim_ops);

		np_panel = of_get_child_by_name(np_hdmi, "panel-nvidia-sim");
	}

	if (!np_panel)
		pr_err("Could not find right panel\n");

	return of_device_is_available(np_panel) ? np_panel : NULL;
}

struct device_node *tegra_tertiary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct tegra_dc_out *dc_out = NULL;
	struct device_node *np_hdmi =
			of_find_node_by_path(HDMI_NODE);

	if (pdata)
		dc_out = pdata->default_out;

	np_panel = of_get_child_by_name(np_hdmi, "hdmi-display");
	if (!np_panel)
		pr_err("Could not find node nvidia,dsi-panel\n");

	return of_device_is_available(np_panel) ? np_panel : NULL;
}
