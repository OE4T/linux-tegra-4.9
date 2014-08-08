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

#ifndef _LINUX_TEGRA_MCE_H
#define _LINUX_TEGRA_MCE_H

enum {
	ARI_STATUS_PENDING = 1,
	ARI_STATUS_ONGOING = 3,
	ARI_STATUS_NONE,
};

int ari_get_status(u32 *request);
int ari_get_response(u64 *data);
int ari_request(u32 events, u32 request, u64 data);
int ari_cancel(u32 request);

int tegra_ari_enter_cstate(u32 state, u32 wake);
int tegra_ari_update_cluster_cstate(u32 cluster_state, u32 ccplex_state);
int tegra_ari_update_crossover_time(u32 type, u32 time);
int tegra_ari_cstate_stats(int state, u64 *stats);

#endif
