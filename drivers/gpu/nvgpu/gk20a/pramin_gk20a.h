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

#ifndef __PRAMIN_GK20A_H__
#define __PRAMIN_GK20A_H__

struct gk20a;
struct nvgpu_mem;
struct nvgpu_mem_sgl;

u32 gk20a_pramin_enter(struct gk20a *g, struct nvgpu_mem *mem,
		       struct nvgpu_sgt *sgt, void *sgl, u32 w);
void gk20a_pramin_exit(struct gk20a *g, struct nvgpu_mem *mem,
		       void *sgl);
#endif
