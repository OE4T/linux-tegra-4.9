/*
 * drivers/video/tegra/dc/nvdisplay/nvdisp_stub.c
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION, All rights reserved.
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
#include <soc/tegra/chip-id.h>
#include <linux/of_gpio.h>
#include <linux/backlight.h>
#include <linux/iommu.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>

#include <linux/platform/tegra/latency_allowance.h>

#include "dc.h"
#include "board-panel.h"
#include "dc_priv.h"
#include <linux/platform_data/lp855x.h>
#include <soc/tegra/common.h>

__weak const struct disp_client *tegra_la_disp_clients_info;

int tegra_is_clk_enabled(struct clk *c)
{
	dump_stack();
	pr_info(" WARNING!!! OBSOLETE FUNCTION CALL!!! \
			DON'T USE %s FUNCTION \n", __func__);
	return 0;
}
EXPORT_SYMBOL(tegra_is_clk_enabled);

static int tegra_t210ref_i2c_notifier_call(struct notifier_block *nb,
	unsigned long event, void *data)
{
	struct backlight_device_brightness_info *smartdim_info = data;
	struct device *dev = smartdim_info->dev;
	if (dev->of_node) {
		if (of_device_is_compatible(dev->of_node,
			"ti,lp8557")) {
			smartdim_info->brightness = tegra_bl_notify(dev,
				smartdim_info->brightness);
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block i2c_nb = {
	.notifier_call = tegra_t210ref_i2c_notifier_call,
};

int nvdisp_register_backlight_notifier(struct tegra_dc *dc)
{
	if (dc->out->sd_settings && !dc->out->sd_settings->bl_device &&
		dc->out->sd_settings->bl_device_name) {
		char *bl_device_name = dc->out->sd_settings->bl_device_name;
		struct backlight_device *bl_device =
			get_backlight_device_by_name(bl_device_name);
		if (bl_device)
			backlight_device_register_notifier(bl_device, &i2c_nb);
	}
	return 0;
}

static int disp_fb_linear_set(void)
{
#if defined(CONFIG_OF_TEGRA_IOMMU_SMMU)
	tegra_fb_linear_set(NULL);
#endif
	return 0;
}
arch_initcall(disp_fb_linear_set);

