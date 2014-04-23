/*
 * Tegra NVENC Module Support
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/types.h>
#include <linux/nvhost.h>
#include "nvenc.h"
#include "flcn/flcn.h"

int nvhost_nvenc_t210_finalize_poweron(struct platform_device *dev)
{
	host1x_writel(dev, 0x117c, 0x18004);
	host1x_writel(dev, 0x2200, 0x800040);
	host1x_writel(dev, 0x2204, 0x10000000);
	host1x_writel(dev, 0x2208, 0x0);

	return nvhost_flcn_boot(dev);
}
