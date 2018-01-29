/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _MM_VGPU_H_
#define _MM_VGPU_H_

struct nvgpu_mem;
struct channel_gk20a;
struct vm_gk20a_mapping_batch;
struct gk20a_as_share;

void vgpu_locked_gmmu_unmap(struct vm_gk20a *vm,
				u64 vaddr,
				u64 size,
				int pgsz_idx,
				bool va_allocated,
				int rw_flag,
				bool sparse,
				struct vm_gk20a_mapping_batch *batch);
int vgpu_vm_bind_channel(struct gk20a_as_share *as_share,
				struct channel_gk20a *ch);
int vgpu_mm_fb_flush(struct gk20a *g);
void vgpu_mm_l2_invalidate(struct gk20a *g);
void vgpu_mm_l2_flush(struct gk20a *g, bool invalidate);
void vgpu_mm_tlb_invalidate(struct gk20a *g, struct nvgpu_mem *pdb);
void vgpu_mm_mmu_set_debug_mode(struct gk20a *g, bool enable);
#endif
