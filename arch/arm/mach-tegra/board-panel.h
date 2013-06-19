/*
 * arch/arm/mach-tegra/board-panel.h
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __MACH_TEGRA_BOARD_PANEL_H
#define __MACH_TEGRA_BOARD_PANEL_H

#include <linux/platform_device.h>
#include "tegra-board-id.h"

struct tegra_panel {
	void (*init_sd_settings)(struct tegra_dc_sd_settings *);
	void (*init_dc_out)(struct tegra_dc_out *);
	void (*init_fb_data)(struct tegra_fb_data *);
	void (*init_cmu_data)(struct tegra_dc_platform_data *);
	void (*set_disp_device)(struct platform_device *);
	int (*register_bl_dev)(void);
	int (*register_i2c_bridge)(void);
};

enum {
	TEGRA_GPIO_RESET,
	TEGRA_GPIO_BL_ENABLE,
	TEGRA_GPIO_PWM,
	TEGRA_GPIO_TE,
	TEGRA_N_GPIO_PANEL, /* add new gpio above this entry */
};

/* tegra_panel_of will replace tegra_panel once we completely move to DT */
struct tegra_panel_of {
	int panel_gpio[TEGRA_N_GPIO_PANEL];
};
static struct tegra_panel_of __maybe_unused panel_of = {
	.panel_gpio = {-1, -1, -1, -1},
};

extern atomic_t sd_brightness;
extern struct tegra_panel dsi_p_wuxga_10_1;
extern struct tegra_panel dsi_a_1080p_11_6;
extern struct tegra_panel dsi_s_wqxga_10_1;
extern struct tegra_panel dsi_a_1080p_14_0;
extern struct tegra_panel edp_a_1080p_14_0;

void tegra_dsi_resources_init(u8 dsi_instance,
			struct resource *resources, int n_resources);

void tegra_dsi_update_init_cmd_gpio_rst(struct tegra_dc_out *dsi_disp1_out);

int tegra_panel_gpio_get_dt(const char *comp_str,
				struct tegra_panel_of *panel);

int tegra_panel_reset(struct tegra_panel_of *panel, unsigned int delay_ms);
#endif /* __MACH_TEGRA_BOARD_PANEL_H */
