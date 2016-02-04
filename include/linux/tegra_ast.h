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

struct tegra_ast;

struct tegra_ast *tegra_ast_add_ref(const struct device_node *client_np,
			const char *ast_prop_name, u32 ast_prop_index);

int tegra_ast_del_ref(void);

int tegra_ast_set_streamid(struct tegra_ast *ast,
		u32 vmindex, u32 stream_id);

int tegra_ast_region_enable(struct tegra_ast *ast, u32 region,
		u32 slave_base, u32 mask, u64 master_base);

int tegra_ast_region_disable(struct tegra_ast *ast, u32 region);

#endif
