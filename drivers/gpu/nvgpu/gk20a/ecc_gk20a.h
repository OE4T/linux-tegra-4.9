/*
 * GK20A ECC
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef ECC_GK20A_H
#define ECC_GK20A_H

#include <uapi/linux/nvgpu.h>

struct gk20a_ecc_stat {
	char **names;
	u32 *counters;
	struct hlist_node hash_node;
};

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "ecc_t18x.h"
#endif
#ifdef CONFIG_TEGRA_19x_GPU
#include "ecc_t19x.h"
#endif

struct ecc_gk20a {
	/* Stats per engine */
	struct {
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
		struct ecc_gr_t18x t18x;
#endif
#ifdef CONFIG_TEGRA_19x_GPU
		struct ecc_gr_t19x t19x;
#endif
	} gr;
};

#endif /*__ECC_GK20A_H__*/
