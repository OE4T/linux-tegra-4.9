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
#include <linux/of_gpio.h>

#include <linux/platform/tegra/latency_allowance.h>

#include "mach/io_dpd.h"
#include "mach/dc.h"
#include "board.h"
#include "board-panel.h"
#include "panel-s-wuxga-8-0.c"
#include "panel-s-edp-uhdtv-15-6.c"

const struct disp_client *tegra_la_disp_clients_info;
atomic_t sd_brightness = ATOMIC_INIT(255);
EXPORT_SYMBOL(sd_brightness);

void nvsd_check_prism_thresh(struct device *dev, int brightness)
{
	pr_info("%s no support added \n", __func__);
}
int tegra_is_clk_enabled(struct clk *c)
{
	dump_stack();
	pr_info(" WARNING!!! OBSOLETE FUNCTION CALL!!! \
			DON'T USE %s FUNCTION \n", __func__);
	return 0;
}
EXPORT_SYMBOL(tegra_is_clk_enabled);

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

int tegra_panel_gpio_get_dt(const char *comp_str,
				struct tegra_panel_of *panel)
{
	int cnt = 0;
	char *label = NULL;
	int err = 0;
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, comp_str);

	/*
	 * If gpios are already populated, just return.
	 */
	if (panel->panel_gpio_populated)
		return 0;

	if (!node) {
		pr_info("%s panel dt support not available\n", comp_str);
		err = -ENOENT;
		goto fail;
	}

	panel->panel_gpio[TEGRA_GPIO_RESET] =
		of_get_named_gpio(node, "nvidia,panel-rst-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_PANEL_EN] =
		of_get_named_gpio(node, "nvidia,panel-en-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BL_ENABLE] =
		of_get_named_gpio(node, "nvidia,panel-bl-en-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_PWM] =
		of_get_named_gpio(node, "nvidia,panel-bl-pwm-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BRIDGE_EN_0] =
		of_get_named_gpio(node, "nvidia,panel-bridge-en-0-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BRIDGE_EN_1] =
		of_get_named_gpio(node, "nvidia,panel-bridge-en-1-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BRIDGE_REFCLK_EN] =
		of_get_named_gpio(node,
			"nvidia,panel-bridge-refclk-en-gpio", 0);

	for (cnt = 0; cnt < TEGRA_N_GPIO_PANEL; cnt++) {
		if (gpio_is_valid(panel->panel_gpio[cnt])) {
			switch (cnt) {
			case TEGRA_GPIO_RESET:
				label = "tegra-panel-reset";
				break;
			case TEGRA_GPIO_PANEL_EN:
				label = "tegra-panel-en";
				break;
			case TEGRA_GPIO_BL_ENABLE:
				label = "tegra-panel-bl-enable";
				break;
			case TEGRA_GPIO_PWM:
				label = "tegra-panel-pwm";
				break;
			case TEGRA_GPIO_BRIDGE_EN_0:
				label = "tegra-panel-bridge-en-0";
				break;
			case TEGRA_GPIO_BRIDGE_EN_1:
				label = "tegra-panel-bridge-en-1";
				break;
			case TEGRA_GPIO_BRIDGE_REFCLK_EN:
				label = "tegra-panel-bridge-refclk-en";
				break;
			default:
				pr_err("tegra panel no gpio entry\n");
			}
			if (label) {
				gpio_request(panel->panel_gpio[cnt],
					label);
				label = NULL;
			}
		}
	}
	if (gpio_is_valid(panel->panel_gpio[TEGRA_GPIO_PWM]))
		gpio_free(panel->panel_gpio[TEGRA_GPIO_PWM]);
	panel->panel_gpio_populated = true;
fail:
	of_node_put(node);
	return err;
}

int tegra_panel_check_regulator_dt_support(const char *comp_str,
				struct tegra_panel_of *panel)
{
	int err = 0;
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, comp_str);

	if (!node) {
		pr_info("%s panel dt support not available\n", comp_str);
		err = -ENOENT;
	}

	panel->en_vmm_vpp_i2c_config =
		of_property_read_bool(node, "nvidia,en-vmm-vpp-with-i2c-config");

	return err;
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
	struct device_node *np_primary;

	if (pdata)
		dc_out = pdata->default_out;

	if (tegra_platform_is_silicon()) {

		/* DSI */
		/*np_primary = of_find_node_by_path(DSI_NODE);*/
		np_primary = of_find_node_by_path(dc_or_node_names[0]);
		if (of_device_is_available(np_primary)) {
			/* SHARP 19x12 panel is being used */
			np_panel = of_get_child_by_name(np_primary,
				"panel-s-wuxga-8-0");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
					&dsi_s_wuxga_8_0_ops);
			/* P2393 DSI2DP Bridge */
			if (!of_device_is_available(np_panel))
				np_panel = of_get_child_by_name(np_primary,
					"panel-dsi-1080p-p2393");
		}
		/* HDMI */
		if (!of_device_is_available(np_panel)) {
			/*np_primary = of_find_node_by_path(HDMI_NODE);*/
			if (of_device_is_available(np_primary))
				np_panel = of_get_child_by_name(np_primary,
					"hdmi-display");
		}
		/* DP */
		if (!of_device_is_available(np_panel)) {
			/*np_primary = of_find_node_by_path(SOR_NODE);*/
			if (of_device_is_available(np_primary))
				np_panel = of_get_child_by_name(np_primary,
					"dp-display");
			if (!of_device_is_available(np_panel)) {
				np_panel = of_get_child_by_name(np_primary,
					"panel-s-edp-uhdtv-15-6");
				if (of_device_is_available(np_panel) && dc_out)
					tegra_panel_register_ops(dc_out,
					&edp_s_uhdtv_15_6_ops);
			}
		}
	} else {/* for linsim or no display panel case */
		/*  use fake dp or fake dsi */
		np_primary = of_find_node_by_path(SOR_NODE);

		if (dc_out)
			tegra_panel_register_ops(dc_out, &panel_sim_ops);

		np_panel = of_get_child_by_name(np_primary, "panel-nvidia-sim");
	}

	if (!np_panel)
		pr_err("Could not find panel for primary node\n");

	return of_device_is_available(np_panel) ? np_panel : NULL;
}

struct device_node *tegra_secondary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct tegra_dc_out *dc_out = NULL;
	struct device_node *np_secondary;

	if (pdata)
		dc_out = pdata->default_out;

	if (tegra_platform_is_silicon()) {
		/* HDMI */
		np_secondary = of_find_node_by_path(dc_or_node_names[1]);
		/*np_secondary = of_find_node_by_path(HDMI_NODE);*/
		if (of_device_is_available(np_secondary))
			np_panel = of_get_child_by_name(np_secondary,
				"hdmi-display");
		/* eDP */
		if (!of_device_is_available(np_panel)) {
			/*np_secondary = of_find_node_by_path(SOR_NODE);*/
			if (of_device_is_available(np_secondary))
				np_panel = of_get_child_by_name(np_secondary,
					"dp-display");
			if (!of_device_is_available(np_panel))
				np_panel = of_get_child_by_name(np_secondary,
					"panel-s-edp-uhdtv-15-6");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
					&edp_s_uhdtv_15_6_ops);
		}
	} else { /* for linsim or no display panel case */

		np_secondary = of_find_node_by_path(SOR_NODE);

		if (dc_out)
			tegra_panel_register_ops(dc_out, &panel_sim_ops);

		np_panel = of_get_child_by_name(np_secondary, "panel-nvidia-sim");
	}

	if (!np_panel)
		pr_err("Could not find panel for secondary node\n");

	return of_device_is_available(np_panel) ? np_panel : NULL;
}

struct device_node *tegra_tertiary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct tegra_dc_out *dc_out = NULL;
	struct device_node *np_tertiary;

	if (pdata)
		dc_out = pdata->default_out;

	if (tegra_platform_is_silicon()) {
		/* eDp panel */
		np_tertiary = of_find_node_by_path(dc_or_node_names[2]);
		/*np_tertiary = of_find_node_by_path(SOR_NODE);*/

		if (dc_out)
			tegra_panel_register_ops(dc_out, &edp_s_uhdtv_15_6_ops);

		np_panel = of_get_child_by_name(np_tertiary, "panel-s-edp-uhdtv-15-6");

	} else {
		np_tertiary = of_find_node_by_path(HDMI_NODE);
		np_panel = of_get_child_by_name(np_tertiary, "hdmi-display");
	}

	if (!np_panel)
		pr_err("Could not find panel for tertiary node\n");

	return (of_device_is_available(np_panel) ? np_panel : NULL);
}
