/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <mach/dc.h>

#include "board-panel.h"
#include "iomap.h"

atomic_t sd_brightness = ATOMIC_INIT(255);
EXPORT_SYMBOL(sd_brightness);

void tegra_dsi_resources_init(u8 dsi_instance,
			struct resource *resources, int n_resources)
{
	int i;
	for (i = 0; i < n_resources; i++) {
		struct resource *r = &resources[i];
		if (resource_type(r) == IORESOURCE_MEM &&
			!strcmp(r->name, "dsi_regs")) {
			switch (dsi_instance) {
			case DSI_INSTANCE_0:
				r->start = TEGRA_DSI_BASE;
				r->end = TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1;
				break;
			case DSI_INSTANCE_1:
			default:
				r->start = TEGRA_DSIB_BASE;
				r->end = TEGRA_DSIB_BASE + TEGRA_DSIB_SIZE - 1;
				break;
			}
		}
		if (resource_type(r) == IORESOURCE_MEM &&
			!strcmp(r->name, "ganged_dsia_regs")) {
			r->start = TEGRA_DSI_BASE;
			r->end = TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1;
		}
		if (resource_type(r) == IORESOURCE_MEM &&
			!strcmp(r->name, "ganged_dsib_regs")) {
			r->start = TEGRA_DSIB_BASE;
			r->end = TEGRA_DSIB_BASE + TEGRA_DSIB_SIZE - 1;
		}
	}
}

void tegra_dsi_update_init_cmd_gpio_rst(
	struct tegra_dc_out *dsi_disp1_out)
{
	int i;
	for (i = 0; i < dsi_disp1_out->dsi->n_init_cmd; i++) {
		if (dsi_disp1_out->dsi->dsi_init_cmd[i].cmd_type ==
					TEGRA_DSI_GPIO_SET)
			dsi_disp1_out->dsi->dsi_init_cmd[i].sp_len_dly.gpio
				= dsi_disp1_out->dsi->dsi_panel_rst_gpio;
	}
}

int tegra_panel_reset(struct tegra_panel_of *panel, unsigned int delay_ms)
{
	int gpio = panel->panel_gpio[TEGRA_GPIO_RESET];

	if (!gpio_is_valid(gpio))
		return -ENOENT;

	gpio_set_value(gpio, 1);
	usleep_range(1000, 5000);
	gpio_set_value(gpio, 0);
	usleep_range(1000, 5000);
	gpio_set_value(gpio, 1);
	msleep(delay_ms);

	return 0;
}

int tegra_panel_gpio_get_dt(const char *comp_str,
				struct tegra_panel_of *panel)
{
	int cnt = 0;
	char *label;
	const char *node_status;
	int err = 0;
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, comp_str);

	/*
	 * All panels need reset. If reset is populated,
	 * we have requested all gpios.
	 */
	if (gpio_is_valid(panel->panel_gpio[TEGRA_GPIO_RESET]))
		return 0;

	if (!node) {
		pr_info("%s panel dt support not available\n", comp_str);
		err = -ENOENT;
		goto fail;
	}

	of_property_read_string(node, "status", &node_status);
	if (strcmp(node_status, "okay")) {
		pr_info("%s panel dt support disabled\n", comp_str);
		err = -ENOENT;
		goto fail;
	}

	panel->panel_gpio[TEGRA_GPIO_RESET] =
		of_get_named_gpio(node, "nvidia,dsi-panel-rst-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BL_ENABLE] =
		of_get_named_gpio(node, "nvidia,dsi-panel-bl-en-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_PWM] =
		of_get_named_gpio(node, "nvidia,dsi-panel-bl-pwm-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_TE] =
		of_get_named_gpio(node, "nvidia,te-gpio", 0);

	for (cnt = 0; cnt < TEGRA_N_GPIO_PANEL; cnt++) {
		if (gpio_is_valid(panel->panel_gpio[cnt])) {
			switch (cnt) {
			case TEGRA_GPIO_RESET:
				label = "tegra-panel-reset";
				break;
			case TEGRA_GPIO_BL_ENABLE:
				label = "tegra-panel-bl-enable";
				break;
			case TEGRA_GPIO_PWM:
				label = "tegra-panel-pwm";
				break;
			case TEGRA_GPIO_TE:
				label = "tegra-panel-te";
				break;
			default:
				pr_err("tegra panel no gpio entry\n");
			}
			gpio_request(panel->panel_gpio[cnt], label);
		}
	}
fail:
	of_node_put(node);
	return err;
}
