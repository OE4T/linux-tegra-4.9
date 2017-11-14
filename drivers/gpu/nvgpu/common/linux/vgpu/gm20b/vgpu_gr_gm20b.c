/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"
#include "gk20a/css_gr_gk20a.h"
#include "common/linux/vgpu/css_vgpu.h"
#include "vgpu_gr_gm20b.h"

void vgpu_gr_gm20b_init_cyclestats(struct gk20a *g)
{
#if defined(CONFIG_GK20A_CYCLE_STATS)
	bool snapshots_supported = true;

	/* cyclestats not supported on vgpu */
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_CYCLE_STATS, false);

	g->gr.max_css_buffer_size = vgpu_css_get_buffer_size(g);

	/* snapshots not supported if the buffer size is 0 */
	if (g->gr.max_css_buffer_size == 0)
		snapshots_supported = false;

	__nvgpu_set_enabled(g, NVGPU_SUPPORT_CYCLE_STATS_SNAPSHOT,
							snapshots_supported);
#endif
}

int vgpu_gm20b_init_fs_state(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_index, gpc_index;
	u32 sm_id = 0;

	gk20a_dbg_fn("");

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		for (tpc_index = 0; tpc_index < gr->gpc_tpc_count[gpc_index];
								tpc_index++) {
			g->gr.sm_to_cluster[sm_id].tpc_index = tpc_index;
			g->gr.sm_to_cluster[sm_id].gpc_index = gpc_index;

			sm_id++;
		}
	}

	gr->no_of_sm = sm_id;
	return 0;
}
