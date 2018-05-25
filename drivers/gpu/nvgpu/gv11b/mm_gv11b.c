/*
 * GV11B MMU
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/mm.h>
#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"

#include "gp10b/mm_gp10b.h"
#include "gp10b/mc_gp10b.h"

#include "mm_gv11b.h"
#include "fb_gv11b.h"
#include "subctx_gv11b.h"

#include <nvgpu/hw/gv11b/hw_fb_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gmmu_gv11b.h>

#define NVGPU_L3_ALLOC_BIT	BIT(36)

bool gv11b_mm_is_bar1_supported(struct gk20a *g)
{
	return false;
}

void gv11b_init_inst_block(struct nvgpu_mem *inst_block,
		struct vm_gk20a *vm, u32 big_page_size)
{
	struct gk20a *g = gk20a_from_vm(vm);

	nvgpu_log_info(g, "inst block phys = 0x%llx, kv = 0x%p",
		nvgpu_inst_block_addr(g, inst_block), inst_block->cpu_va);

	g->ops.mm.init_pdb(g, inst_block, vm);

	if (big_page_size && g->ops.mm.set_big_page_size)
		g->ops.mm.set_big_page_size(g, inst_block, big_page_size);

	gv11b_init_subcontext_pdb(vm, inst_block);
}

bool gv11b_mm_mmu_fault_pending(struct gk20a *g)
{
	return gv11b_fb_mmu_fault_pending(g);
}

void gv11b_mm_fault_info_mem_destroy(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	nvgpu_mutex_acquire(&g->mm.hub_isr_mutex);

	g->ops.fb.disable_hub_intr(g, STALL_REG_INDEX, HUB_INTR_TYPE_OTHER |
			 HUB_INTR_TYPE_NONREPLAY | HUB_INTR_TYPE_REPLAY);

	nvgpu_kfree(g, g->mm.fault_info[FAULT_TYPE_OTHER_AND_NONREPLAY]);

	g->mm.fault_info[FAULT_TYPE_OTHER_AND_NONREPLAY] = NULL;
	g->mm.fault_info[FAULT_TYPE_REPLAY] = NULL;

	nvgpu_mutex_release(&g->mm.hub_isr_mutex);
	nvgpu_mutex_destroy(&g->mm.hub_isr_mutex);
}

static int gv11b_mm_mmu_fault_info_buf_init(struct gk20a *g,
			 u32 *hub_intr_types)
{
	struct mmu_fault_info *fault_info_mem;

	if (g->mm.fault_info[FAULT_TYPE_OTHER_AND_NONREPLAY] != NULL &&
		g->mm.fault_info[FAULT_TYPE_REPLAY] != NULL) {
		*hub_intr_types |= HUB_INTR_TYPE_OTHER;
		return 0;
	}

	fault_info_mem = nvgpu_kzalloc(g, sizeof(struct mmu_fault_info) *
						FAULT_TYPE_NUM);
	if (!fault_info_mem) {
		nvgpu_log_info(g, "failed to alloc shadow fault info");
		return -ENOMEM;
	}
	/* shadow buffer for copying mmu fault info */
	g->mm.fault_info[FAULT_TYPE_OTHER_AND_NONREPLAY] =
		 &fault_info_mem[FAULT_TYPE_OTHER_AND_NONREPLAY];

	g->mm.fault_info[FAULT_TYPE_REPLAY] =
		 &fault_info_mem[FAULT_TYPE_REPLAY];

	*hub_intr_types |= HUB_INTR_TYPE_OTHER;
	return 0;
}

static void gv11b_mm_mmu_hw_fault_buf_init(struct gk20a *g,
			 u32 *hub_intr_types)
{
	struct vm_gk20a *vm = g->mm.bar2.vm;
	int err = 0;
	size_t fb_size;

	/* Max entries take care of 1 entry used for full detection */
	fb_size = (g->ops.fifo.get_num_fifos(g) + 1) *
				 gmmu_fault_buf_size_v();

	if (!nvgpu_mem_is_valid(
		&g->mm.hw_fault_buf[FAULT_TYPE_OTHER_AND_NONREPLAY])) {

		err = nvgpu_dma_alloc_map_sys(vm, fb_size,
			&g->mm.hw_fault_buf[FAULT_TYPE_OTHER_AND_NONREPLAY]);
		if (err) {
			nvgpu_err(g,
			"Error in hw mmu fault buf [0] alloc in bar2 vm ");
			/* Fault will be snapped in pri reg but not in buffer */
			return;
		}
	}

	*hub_intr_types |= HUB_INTR_TYPE_NONREPLAY;

	if (!nvgpu_mem_is_valid(
		&g->mm.hw_fault_buf[FAULT_TYPE_REPLAY])) {
		err = nvgpu_dma_alloc_map_sys(vm, fb_size,
				&g->mm.hw_fault_buf[FAULT_TYPE_REPLAY]);
		if (err) {
			nvgpu_err(g,
			"Error in hw mmu fault buf [1] alloc in bar2 vm ");
			/* Fault will be snapped in pri reg but not in buffer */
			return;
		}
	}

	*hub_intr_types |= HUB_INTR_TYPE_REPLAY;
}

static void gv11b_mm_mmu_hw_fault_buf_deinit(struct gk20a *g)
{
	struct vm_gk20a *vm = g->mm.bar2.vm;

	nvgpu_log_fn(g, " ");

	g->ops.fb.disable_hub_intr(g, STALL_REG_INDEX, HUB_INTR_TYPE_NONREPLAY |
					 HUB_INTR_TYPE_REPLAY);

	g->mm.hub_intr_types &= (~(HUB_INTR_TYPE_NONREPLAY |
				 HUB_INTR_TYPE_REPLAY));

	if ((gv11b_fb_is_fault_buf_enabled(g, NONREPLAY_REG_INDEX))) {
		gv11b_fb_fault_buf_set_state_hw(g, NONREPLAY_REG_INDEX,
						 FAULT_BUF_DISABLED);
	}

	if ((gv11b_fb_is_fault_buf_enabled(g, REPLAY_REG_INDEX))) {
		gv11b_fb_fault_buf_set_state_hw(g, REPLAY_REG_INDEX,
						 FAULT_BUF_DISABLED);
	}

	if (nvgpu_mem_is_valid(
		    &g->mm.hw_fault_buf[FAULT_TYPE_OTHER_AND_NONREPLAY]))
		nvgpu_dma_unmap_free(vm,
			 &g->mm.hw_fault_buf[FAULT_TYPE_OTHER_AND_NONREPLAY]);
	if (nvgpu_mem_is_valid(&g->mm.hw_fault_buf[FAULT_TYPE_REPLAY]))
		nvgpu_dma_unmap_free(vm,
			 &g->mm.hw_fault_buf[FAULT_TYPE_REPLAY]);
}

void gv11b_mm_remove_bar2_vm(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;

	nvgpu_log_fn(g, " ");

	gv11b_mm_mmu_hw_fault_buf_deinit(g);

	nvgpu_free_inst_block(g, &mm->bar2.inst_block);
	nvgpu_vm_put(mm->bar2.vm);
}

static void gv11b_mm_mmu_fault_setup_hw(struct gk20a *g)
{
	if (nvgpu_mem_is_valid(
			&g->mm.hw_fault_buf[FAULT_TYPE_OTHER_AND_NONREPLAY]))
		gv11b_fb_fault_buf_configure_hw(g, NONREPLAY_REG_INDEX);
	if (nvgpu_mem_is_valid(&g->mm.hw_fault_buf[FAULT_TYPE_REPLAY]))
		gv11b_fb_fault_buf_configure_hw(g, REPLAY_REG_INDEX);
}

static int gv11b_mm_mmu_fault_setup_sw(struct gk20a *g)
{
	int err;

	nvgpu_log_fn(g, " ");

	nvgpu_mutex_init(&g->mm.hub_isr_mutex);

	g->mm.hub_intr_types = HUB_INTR_TYPE_ECC_UNCORRECTED;

	err = gv11b_mm_mmu_fault_info_buf_init(g, &g->mm.hub_intr_types);

	if (!err)
		gv11b_mm_mmu_hw_fault_buf_init(g, &g->mm.hub_intr_types);

	return err;
}

int gv11b_init_mm_setup_hw(struct gk20a *g)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	g->ops.fb.set_mmu_page_size(g);
	g->ops.fb.init_hw(g);

	err = g->ops.bus.bar2_bind(g, &g->mm.bar2.inst_block);
	if (err)
		return err;

	if (gk20a_mm_fb_flush(g) || gk20a_mm_fb_flush(g))
		return -EBUSY;

	err = gv11b_mm_mmu_fault_setup_sw(g);
	if (!err)
		gv11b_mm_mmu_fault_setup_hw(g);

	nvgpu_log_fn(g, "end");

	return err;
}

void gv11b_mm_l2_flush(struct gk20a *g, bool invalidate)
{
	nvgpu_log(g, gpu_dbg_fn, "gv11b_mm_l2_flush");

	g->ops.mm.fb_flush(g);
	gk20a_mm_l2_flush(g, invalidate);
	g->ops.mm.fb_flush(g);
}

/*
 * On Volta the GPU determines whether to do L3 allocation for a mapping by
 * checking bit 36 of the phsyical address. So if a mapping should allocte lines
 * in the L3 this bit must be set.
 */
u64 gv11b_gpu_phys_addr(struct gk20a *g,
			       struct nvgpu_gmmu_attrs *attrs, u64 phys)
{
	if (attrs && attrs->l3_alloc)
		return phys | NVGPU_L3_ALLOC_BIT;

	return phys;
}
