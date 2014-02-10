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

#include <mach/dc.h>

#include "board-panel.h"
#include "iomap.h"

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

