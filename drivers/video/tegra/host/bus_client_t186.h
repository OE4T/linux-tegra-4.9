/*
 * Copyright (c) 2015, NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_BUS_CLIENT_T186_H
#define __NVHOST_BUS_CLIENT_T186_H

#include "bus_client.h"

void host1x_hypervisor_writel(struct platform_device *pdev, u32 r, u32 v)
{
	void __iomem *aperture = get_aperture(pdev, 1);

	if (aperture) {
		nvhost_dbg(dbg_reg, " d=%s r=0x%x v=0x%x", pdev->name, r, v);
		writel(v, aperture + r);
	}
}

u32 host1x_hypervisor_readl(struct platform_device *pdev, u32 r)
{
	void __iomem *aperture = get_aperture(pdev, 1);
	u32 v = 0;

	if (aperture) {
		nvhost_dbg(dbg_reg, " d=%s r=0x%x", pdev->name, r);
		v = readl(aperture + r);
		nvhost_dbg(dbg_reg, " d=%s r=0x%x v=0x%x", pdev->name, r, v);
	}

	return v;
}

#endif /* __NVHOST_BUS_CLIENT_T186_H */
