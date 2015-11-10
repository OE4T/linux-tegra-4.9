/*
 * drivers/video/tegra/dc/dsi_padctrl.c
 *
 * Copyright (c) 2015, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/reset.h>
#include <mach/dc.h>

#include "dsi_padctrl_regs.h"
#include "dc_priv_defs.h"
#include "dc_priv.h"
#include "dsi.h"

#define DSI_PADCTRL_INSTANCE	4
#define DSI_MAX_INSTANCES	4

static int dsi_padctrl_pwr_down_regs[DSI_MAX_INSTANCES] = {
	DSI_PADCTRL_A_LANES_PWR_DOWN,
	DSI_PADCTRL_B_LANES_PWR_DOWN,
	DSI_PADCTRL_C_LANES_PWR_DOWN,
	DSI_PADCTRL_D_LANES_PWR_DOWN,
};

static int dsi_padctrl_pull_down_regs[DSI_MAX_INSTANCES] = {
	DSI_PADCTRL_A_PULL_DOWN,
	DSI_PADCTRL_B_PULL_DOWN,
	DSI_PADCTRL_C_PULL_DOWN,
	DSI_PADCTRL_D_PULL_DOWN,
};

static inline void tegra_dsi_padctrl_write(struct tegra_dsi_padctrl *dsi_padctrl,
	int val, int reg)
{
	writel(val, dsi_padctrl->base_addr + (reg * 4));
}

static inline unsigned long tegra_dsi_padctrl_read(struct tegra_dsi_padctrl *dsi_padctrl,
	int reg)
{
	return readl(dsi_padctrl->base_addr + (reg * 4));
}

static void tegra_dsi_padctrl_reset(struct tegra_dsi_padctrl *dsi_padctrl)
{
	if (!dsi_padctrl || !dsi_padctrl->reset) {
		pr_err("dsi padctl: Invalid dsi padctrl reset handle\n");
		return;
	}

	reset_control_reset(dsi_padctrl->reset);
}

void tegra_dsi_padctrl_enable(struct tegra_dsi_padctrl *dsi_padctrl)
{
	int val;
	u8 i;

	/* Clear all pwr downs for all controllers */
	for (i = 0; i < ARRAY_SIZE(dsi_padctrl_pwr_down_regs); i++) {
		val = tegra_dsi_padctrl_read(dsi_padctrl,
			dsi_padctrl_pwr_down_regs[i]);
		val &= ~(DSI_PADCTRL_PWR_DOWN_PD_CLK_EN |
			DSI_PADCTRL_PWR_DOWN_PD_IO_0_EN |
			DSI_PADCTRL_PWR_DOWN_PD_IO_1_EN);
		tegra_dsi_padctrl_write(dsi_padctrl, val,
			dsi_padctrl_pwr_down_regs[i]);
	}

	/* Clear all pull downs for all controllers */
	for (i = 0; i < ARRAY_SIZE(dsi_padctrl_pull_down_regs); i++) {
		val = tegra_dsi_padctrl_read(dsi_padctrl,
			dsi_padctrl_pull_down_regs[i]);
		val &= ~(DSI_PADCTRL_E_PULL_DWN_PD_CLK_EN |
			DSI_PADCTRL_E_PULL_DWN_PD_IO_0_EN |
			DSI_PADCTRL_E_PULL_DWN_PD_IO_1_EN);
		tegra_dsi_padctrl_write(dsi_padctrl, val,
			dsi_padctrl_pull_down_regs[i]);
	}
}

struct tegra_dsi_padctrl *tegra_dsi_padctrl_init(struct tegra_dc *dc)
{
	struct tegra_dsi_padctrl *dsi_padctrl;
	struct resource *res;
	struct resource padctrl_res;
	struct device_node *np_dsi = of_find_node_by_path(DSI_NODE);
	int err;

	/* Padctrl module doesn't exist on fpga */
	if (tegra_platform_is_linsim() || tegra_platform_is_fpga()) {
		return NULL;
	}

	dsi_padctrl = devm_kzalloc(&dc->ndev->dev, sizeof(struct tegra_dsi_padctrl),
		GFP_KERNEL);
        if (!dsi_padctrl) {
                dev_err(&dc->ndev->dev, "dsi padctl: memory allocation failed\n");
                err = -ENOMEM;
		goto fail;
        }

	/*
	 * DSI pad control module is listed in dt immediately after DSI
	 * instances. Use DSI_PADCTRL_INSTANCE to get the resource for
	 * dsi pad control module.
	 */
	if (np_dsi && of_device_is_available(np_dsi)) {
		err = of_address_to_resource(np_dsi, DSI_PADCTRL_INSTANCE,
			&padctrl_res);
		if (err) {
			dev_err(&dc->ndev->dev, "dsi padctl: no mem res\n");
			goto fail;
		}
	}

	res = &padctrl_res;
	dsi_padctrl->base_res = request_mem_region(res->start,
		resource_size(res), dc->ndev->name);
	if (!dsi_padctrl->base_res) {
		dev_err(&dc->ndev->dev,
			"dsi patctl: request_mem_region failed\n");
		err = -EBUSY;
		goto fail;
	}

	dsi_padctrl->base_addr = ioremap(res->start, resource_size(res));
	if (!dsi_padctrl->base_addr) {
		dev_err(&dc->ndev->dev,
			"dsi patctl: Failed to map registers\n");
		err = -EINVAL;
		goto fail;
	}

	dsi_padctrl->reset = of_reset_control_get(np_dsi, "dsi_padctrl");
	if (IS_ERR_OR_NULL(dsi_padctrl->reset)) {
		dev_err(&dc->ndev->dev, "dsi padctl: Failed to get reset\n");
		err = PTR_ERR(dsi_padctrl->reset);
		goto fail;
	}

	/* Reset dsi padctrl module */
	tegra_dsi_padctrl_reset(dsi_padctrl);

	return dsi_padctrl;
fail:
	dev_err(&dc->ndev->dev, "dsi pactrl init failed %d\n", err);
	return ERR_PTR(err);
}

void tegra_dsi_padctrl_shutdown(struct tegra_dc *dc)
{
	struct tegra_dc_dsi_data *dsi = tegra_dc_get_outdata(dc);
	struct tegra_dsi_padctrl *dsi_padctrl;
	int val;
	u8 i;

	dsi_padctrl = dsi->pad_ctrl;

	/* Enable all pwr downs for all controllers */
	val = 0;
	for (i = 0; i < ARRAY_SIZE(dsi_padctrl_pwr_down_regs); i++) {
		val |= (DSI_PADCTRL_PWR_DOWN_PD_CLK_EN |
			DSI_PADCTRL_PWR_DOWN_PD_IO_0_EN |
			DSI_PADCTRL_PWR_DOWN_PD_IO_1_EN);
		tegra_dsi_padctrl_write(dsi_padctrl, val,
			dsi_padctrl_pwr_down_regs[i]);
	} 

	/* Clear all pull downs for all controllers */
	val = 0;
	for (i = 0; i < ARRAY_SIZE(dsi_padctrl_pull_down_regs); i++) {
		val |= (DSI_PADCTRL_E_PULL_DWN_PD_CLK_EN |
			DSI_PADCTRL_E_PULL_DWN_PD_IO_0_EN |
			DSI_PADCTRL_E_PULL_DWN_PD_IO_1_EN);
		tegra_dsi_padctrl_write(dsi_padctrl, val,
			dsi_padctrl_pull_down_regs[i]);
	}

	iounmap(dsi_padctrl->base_addr);
	release_mem_region(dsi_padctrl->base_res->start,
		resource_size(dsi_padctrl->base_res));
}
