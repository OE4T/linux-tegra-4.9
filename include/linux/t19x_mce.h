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

#ifndef _LINUX_TEGRA_MCE_H
#define _LINUX_TEGRA_MCE_H

#define NVG_STAT_MAX_ENTRIES	10
#define MCE_STAT_ID_SHIFT	16UL

struct cstats_info {
	char	*name; /* name of the cstats */
	int	id;   /* NVG id */
	int	units;/* No of cores/clusters/cluster groups */
};

int t19x_mce_enter_cstate(u32 state, u32 wake_time);
int t19x_mce_update_cstate_info(u32 cluster, u32 ccplex,
	u32 system, u8 force, u32 wake_mask, bool valid);
int t19x_mce_update_crossover_time(u32 type, u32 time);
int t19x_mce_read_cstate_stats(u32 state, u32 *stats);
int t19x_mce_write_cstate_stats(u32 state, u32 stats);
int t19x_mce_is_sc7_allowed(u32 state, u32 wake, u32 *allowed);
int t19x_mce_online_core(int cpu);
int t19x_mce_cc3_ctrl(u32 ndiv, u8 enable);
int t19x_mce_echo_data(u32 data, int *matched);
int t19x_mce_read_versions(u32 *major, u32 *minor);
int t19x_mce_enum_features(u64 *features);

#endif
