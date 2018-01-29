/*
 * Virtualized GPU L2
 *
 * Copyright (c) 2014-2018 NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "vgpu.h"
#include "ltc_vgpu.h"

int vgpu_determine_L2_size_bytes(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	return priv->constants.l2_size;
}

int vgpu_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	u32 max_comptag_lines = 0;
	int err;

	gk20a_dbg_fn("");

	gr->cacheline_size = priv->constants.cacheline_size;
	gr->comptags_per_cacheline = priv->constants.comptags_per_cacheline;
	gr->slices_per_ltc = priv->constants.slices_per_ltc;
	max_comptag_lines = priv->constants.comptag_lines;

	if (max_comptag_lines < 2)
		return -ENXIO;

	err = gk20a_comptag_allocator_init(g, &gr->comp_tags, max_comptag_lines);
	if (err)
		return err;

	return 0;
}

void vgpu_ltc_init_fs_state(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	g->ltc_count = priv->constants.ltc_count;
}
