/*
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

#ifndef _PLATFORM_GP10B_TEGRA_H_
#define _PLATFORM_GP10B_TEGRA_H_

#include "gp10b/gr_gp10b.h"

int gr_gp10b_ecc_stat_create(struct device *dev,
				int is_l2,
				char *ecc_stat_name,
				struct gr_gp10b_ecc_stat *ecc_stat,
				struct device_attribute *dev_attr_array);

void gr_gp10b_ecc_stat_remove(struct device *dev,
				int is_l2,
				struct gr_gp10b_ecc_stat *ecc_stat,
				struct device_attribute *dev_attr_array);

int gp10b_tegra_remove(struct device *dev);

#endif
