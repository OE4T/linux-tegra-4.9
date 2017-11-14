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

#ifndef _LTC_VGPU_H_
#define _LTC_VGPU_H_

struct gk20a;
struct gr_gk20a;

int vgpu_determine_L2_size_bytes(struct gk20a *g);
int vgpu_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr);
void vgpu_ltc_init_fs_state(struct gk20a *g);

#endif
