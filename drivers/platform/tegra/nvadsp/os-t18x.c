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
#include <linux/irqchip/tegra-agic.h>

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
 end:
	return ret;
}
