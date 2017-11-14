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

#ifndef _CSS_VGPU_H_
#define _CSS_VGPU_H_

#include <nvgpu/types.h>

struct gr_gk20a;
struct channel_gk20a;
struct gk20a_cs_snapshot_client;

void vgpu_css_release_snapshot_buffer(struct gr_gk20a *gr);
int vgpu_css_flush_snapshots(struct channel_gk20a *ch,
			u32 *pending, bool *hw_overflow);
int vgpu_css_detach(struct channel_gk20a *ch,
		struct gk20a_cs_snapshot_client *cs_client);
int vgpu_css_enable_snapshot_buffer(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *cs_client);
u32 vgpu_css_get_buffer_size(struct gk20a *g);
#endif
