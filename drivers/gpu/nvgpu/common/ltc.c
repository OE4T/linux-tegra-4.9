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

#include <nvgpu/ltc.h>

#include "gk20a/gk20a.h"

int nvgpu_init_ltc_support(struct gk20a *g)
{
	nvgpu_spinlock_init(&g->ltc_enabled_lock);

	g->mm.ltc_enabled_current = true;
	g->mm.ltc_enabled_target = true;

	if (g->ops.ltc.init_fs_state)
		g->ops.ltc.init_fs_state(g);

	return 0;
}

void nvgpu_ltc_sync_enabled(struct gk20a *g)
{
	nvgpu_spinlock_acquire(&g->ltc_enabled_lock);
	if (g->mm.ltc_enabled_current != g->mm.ltc_enabled_target) {
		g->ops.ltc.set_enabled(g, g->mm.ltc_enabled_target);
		g->mm.ltc_enabled_current = g->mm.ltc_enabled_target;
	}
	nvgpu_spinlock_release(&g->ltc_enabled_lock);
}
