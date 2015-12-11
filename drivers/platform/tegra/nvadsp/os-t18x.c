/*
 * Copyright (C) 2015, NVIDIA Corporation. All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/tegra_nvadsp.h>
#include <linux/tegra-hsp.h>
#include <linux/irqchip/tegra-agic.h>

static void nvadsp_dbell_handler(int master, void *data)
{
	struct platform_device *pdev = data;
	struct device *dev = &pdev->dev;

	dev_info(dev, "APE DBELL handler (master:%d)\n", master);
}


int nvadsp_os_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	ret = tegra_agic_route_interrupt(INT_ATKE_TMR0,
					 TEGRA_AGIC_ADSP);
	if (ret) {
		dev_err(dev, "failed to atke interrupt\n");
		goto end;
	}

	ret = tegra_agic_route_interrupt(INT_ATKE_TMR1,
			TEGRA_AGIC_ADSP);
	if (ret) {
		dev_err(dev, "failed to atke interrupt\n");
		goto end;
	}

	ret = tegra_agic_route_interrupt(INT_SHSP2APE_DB,
					 TEGRA_AGIC_ADSP);
	if (ret) {
		dev_err(dev, "failed to INT_SHSP2APE_DB interrupt\n");
		goto end;
	}

	ret = tegra_hsp_db_add_handler(HSP_MASTER_APE,
				       nvadsp_dbell_handler, pdev);
	if (ret) {
		dev_err(dev, "failed to add HSP_MASTER_APE DB handler\n");
		goto end;
	}
 end:
	return ret;
}
