/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION. All rights reserved.
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

#ifndef _LINUX_TEGRA_AST_H
#define _LINUX_TEGRA_AST_H

struct device;
struct device_node;

struct tegra_ast_region_info {
	u8  enabled;
	u8  lock;
	u8  snoop;
	u8  non_secure;

	u8  ns_passthru;
	u8  carveout_id;
	u8  carveout_al;
	u8  vpr_rd;

	u8  vpr_wr;
	u8  vpr_passthru;
	u8  vm_index;
	u8  physical;

	u8  stream_id;
	u8  stream_id_enabled;
	u8  pad[2];

	u64 slave;
	u64 mask;
	u64 master;
	u32 control;
};

int tegra_ast_map(struct device *, const char *name, unsigned count,
			void __iomem *[]);
void tegra_ast_unmap(struct device *, unsigned count, void __iomem *const []);

int tegra_ast_region_enable(unsigned count, void __iomem *const [],
				u32 region, u32 slave_base, u32 mask,
				u64 master_base, u32 stream_id);
void tegra_ast_region_disable(unsigned count, void __iomem *const [],
				u32 region);

void tegra_ast_get_region_info(void __iomem *base,
			u32 region, struct tegra_ast_region_info *info);

#endif
