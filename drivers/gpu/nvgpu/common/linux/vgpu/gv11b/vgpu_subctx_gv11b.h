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

#ifndef _VGPU_SUBCTX_GV11B_H_
#define _VGPU_SUBCTX_GV11B_H_

struct channel_gk20a;

int vgpu_gv11b_alloc_subctx_header(struct channel_gk20a *c);
void vgpu_gv11b_free_subctx_header(struct channel_gk20a *c);

#endif
