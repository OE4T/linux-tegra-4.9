/*
 * Tegra Graphics Host Chip support module
 *
 * Copyright (c) 2012-2014, NVIDIA Corporation.  All rights reserved.
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

#include <linux/errno.h>
#include <linux/types.h>
#include <linux/bug.h>
#include <linux/slab.h>
#include <linux/tegra-soc.h>

#include "chip_support.h"
#include "t124/t124.h"

struct nvhost_chip_support *nvhost_chip_ops;

struct nvhost_chip_support *nvhost_get_chip_ops(void)
{
	return nvhost_chip_ops;
}

int nvhost_init_chip_support(struct nvhost_master *host)
{
	int err = 0;

	if (nvhost_chip_ops == NULL) {
		nvhost_chip_ops = kzalloc(sizeof(*nvhost_chip_ops), GFP_KERNEL);
		if (nvhost_chip_ops == NULL) {
			pr_err("%s: Cannot allocate nvhost_chip_support\n",
				__func__);
			return 0;
		}
	}

	switch (tegra_get_chipid()) {
	case TEGRA_CHIPID_TEGRA12:
		nvhost_chip_ops->soc_name = "tegra12x";
		err = nvhost_init_t124_support(host, nvhost_chip_ops);
		break;

	case TEGRA_CHIPID_TEGRA13:
		nvhost_chip_ops->soc_name = "tegra13x";
		err = nvhost_init_t124_support(host, nvhost_chip_ops);
		break;

	default:
		err = -ENODEV;
	}

	return err;
}
