/*
 * drivers/video/tegra/dc/mipi_cal.c
 *
 * Copyright (c) 2012, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/ioport.h>
#include <linux/gfp.h>
#include "dc_priv.h"
#include "mipi_cal.h"
#include "mipi_cal_regs.h"

/* must be called with clocks enabled */
void tegra_mipi_cal_init_hw(struct tegra_mipi_cal *mipi_cal)
{
	unsigned cnt = MIPI_CAL_MIPI_CAL_CTRL_0;

	mutex_lock(&mipi_cal->lock);

	for (; cnt <= MIPI_CAL_MIPI_BIAS_PAD_CFG2_0; cnt += 4)
		tegra_mipi_cal_write(mipi_cal, 0, cnt);

	mutex_unlock(&mipi_cal->lock);
}
EXPORT_SYMBOL(tegra_mipi_cal_init_hw);

struct tegra_mipi_cal *tegra_mipi_cal_init_sw(struct tegra_dc *dc)
{
	struct tegra_mipi_cal *mipi_cal;
	struct resource *res;
	struct clk *clk;
	void __iomem *base;
	int err = 0;

	mipi_cal = devm_kzalloc(&dc->ndev->dev, sizeof(*mipi_cal), GFP_KERNEL);
	if (!mipi_cal) {
		dev_err(&dc->ndev->dev, "mipi_cal: memory allocation fail\n");
		err = -ENOMEM;
		goto fail;
	}

	res = nvhost_get_resource_byname(dc->ndev, IORESOURCE_MEM, "mipi_cal");
	if (!res) {
		dev_err(&dc->ndev->dev, "mipi_cal: no entry in resource\n");
		err = -ENOENT;
		goto fail_free_mipi_cal;
	}

	base = devm_request_and_ioremap(&dc->ndev->dev, res);
	if (!base) {
		dev_err(&dc->ndev->dev, "mipi_cal: bus to virtual mapping failed\n");
		err = -EBUSY;
		goto fail_free_res;
	}

	clk = clk_get_sys("mipi-cal", NULL);
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&dc->ndev->dev, "mipi_cal: clk get failed\n");
		err = PTR_ERR(clk);
		goto fail_free_map;
	}

	mutex_init(&mipi_cal->lock);
	mipi_cal->dc = dc;
	mipi_cal->res = res;
	mipi_cal->base = base;
	mipi_cal->clk = clk;
	mipi_cal->power_on = false;

	return mipi_cal;

fail_free_map:
	devm_iounmap(&dc->ndev->dev, base);
	devm_release_mem_region(&dc->ndev->dev, res->start, resource_size(res));
fail_free_res:
	release_resource(res);
fail_free_mipi_cal:
	devm_kfree(&dc->ndev->dev, mipi_cal);
fail:
	return ERR_PTR(err);
}
EXPORT_SYMBOL(tegra_mipi_cal_init_sw);

