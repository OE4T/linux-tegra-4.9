/*
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

#ifndef _TEGRA_PRODS_H
#define _TEGRA_PRODS_H

struct tegra_prod_list;

int tegra_prod_set_list(void __iomem **base,
		struct tegra_prod_list *tegra_prod_list);

int tegra_prod_set_boot_init(void __iomem **base,
		struct tegra_prod_list *tegra_prod_list);

int tegra_prod_set_by_name(void __iomem **base, const char *name,
		struct tegra_prod_list *tegra_prod_list);

struct tegra_prod_list *tegra_prod_init(const struct device_node *np);

struct tegra_prod_list *tegra_prod_get(struct device *dev, const char *name);

int tegra_prod_release(struct tegra_prod_list **tegra_prod_list);
#endif
