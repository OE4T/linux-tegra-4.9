/*
 * drivers/video/tegra/host/t124/as_t124.c
 *
 * Tegra Graphics Host Address Space Support for T124 Architecture Chips
 *
 * Copyright (c) 2010-2013, NVIDIA CORPORATION.  All rights reserved.
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
#include "dev.h"
#include "nvhost_as.h"
#include "t124.h"
#include "gk20a/gk20a.h"



static int t124_as_init(struct nvhost_master *host, struct nvhost_as *as)
{
#if defined(CONFIG_TEGRA_GK20A)
	if (is_gk20a_module(as->ch->dev))
		as->ops = &gk20a_as_moduleops;
#endif
	return 0;
}

int nvhost_init_t124_as_support(struct nvhost_chip_support *op)
{
	nvhost_dbg_fn("");

	op->as.init = t124_as_init;
	return 0;
}
