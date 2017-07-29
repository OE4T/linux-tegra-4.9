/*
 *
 * Tegra GV11B GPU Driver Register Ops
 *
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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
#ifndef __REGOPS_GV11B_H_
#define __REGOPS_GV11B_H_

const struct regop_offset_range *gv11b_get_global_whitelist_ranges(void);
int gv11b_get_global_whitelist_ranges_count(void);
const struct regop_offset_range *gv11b_get_context_whitelist_ranges(void);
int gv11b_get_context_whitelist_ranges_count(void);
const u32 *gv11b_get_runcontrol_whitelist(void);
int gv11b_get_runcontrol_whitelist_count(void);
const struct regop_offset_range *gv11b_get_runcontrol_whitelist_ranges(void);
int gv11b_get_runcontrol_whitelist_ranges_count(void);
const u32 *gv11b_get_qctl_whitelist(void);
int gv11b_get_qctl_whitelist_count(void);
const struct regop_offset_range *gv11b_get_qctl_whitelist_ranges(void);
int gv11b_get_qctl_whitelist_ranges_count(void);
int gv11b_apply_smpc_war(struct dbg_session_gk20a *dbg_s);

#endif /* __REGOPS_GV11B_H_ */
