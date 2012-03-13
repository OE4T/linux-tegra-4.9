/*
 * drivers/video/tegra/host/bus_client.c
 *
 * Tegra Graphics Host Client Module
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#include "bus_client.h"
#include "dev.h"
#include <linux/string.h>

void nvhost_read_module_regs(struct nvhost_device *ndev,
			u32 offset, int count, u32 *values)
{
	void __iomem *p = ndev->aperture + offset;

	nvhost_module_busy(ndev);
	while (count--) {
		*(values++) = readl(p);
		p += 4;
	}
	rmb();
	nvhost_module_idle(ndev);
}

void nvhost_write_module_regs(struct nvhost_device *ndev,
			u32 offset, int count, const u32 *values)
{
	void __iomem *p = ndev->aperture + offset;

	nvhost_module_busy(ndev);
	while (count--) {
		writel(*(values++), p);
		p += 4;
	}
	wmb();
	nvhost_module_idle(ndev);
}
