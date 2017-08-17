/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/scatterlist.h>

#include <nvgpu/vidmem.h>
#include <nvgpu/page_allocator.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"

void nvgpu_vidmem_destroy(struct gk20a *g)
{
	if (nvgpu_alloc_initialized(&g->mm.vidmem.allocator))
		nvgpu_alloc_destroy(&g->mm.vidmem.allocator);
}

int nvgpu_vidmem_clear_all(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct gk20a_fence *gk20a_fence_out = NULL;
	u64 region2_base = 0;
	int err = 0;

	if (mm->vidmem.ce_ctx_id == (u32)~0)
		return -EINVAL;

	err = gk20a_ce_execute_ops(g,
			mm->vidmem.ce_ctx_id,
			0,
			mm->vidmem.base,
			mm->vidmem.bootstrap_base - mm->vidmem.base,
			0x00000000,
			NVGPU_CE_DST_LOCATION_LOCAL_FB,
			NVGPU_CE_MEMSET,
			NULL,
			0,
			NULL);
	if (err) {
		nvgpu_err(g,
			"Failed to clear vidmem region 1 : %d", err);
		return err;
	}

	region2_base = mm->vidmem.bootstrap_base + mm->vidmem.bootstrap_size;

	err = gk20a_ce_execute_ops(g,
			mm->vidmem.ce_ctx_id,
			0,
			region2_base,
			mm->vidmem.size - region2_base,
			0x00000000,
			NVGPU_CE_DST_LOCATION_LOCAL_FB,
			NVGPU_CE_MEMSET,
			NULL,
			0,
			&gk20a_fence_out);
	if (err) {
		nvgpu_err(g,
			"Failed to clear vidmem region 2 : %d", err);
		return err;
	}

	if (gk20a_fence_out) {
		struct nvgpu_timeout timeout;

		nvgpu_timeout_init(g, &timeout,
				   gk20a_get_gr_idle_timeout(g),
				   NVGPU_TIMER_CPU_TIMER);

		do {
			err = gk20a_fence_wait(g, gk20a_fence_out,
					       gk20a_get_gr_idle_timeout(g));
		} while (err == -ERESTARTSYS &&
			 !nvgpu_timeout_expired(&timeout));

		gk20a_fence_put(gk20a_fence_out);
		if (err) {
			nvgpu_err(g,
				"fence wait failed for CE execute ops");
			return err;
		}
	}

	mm->vidmem.cleared = true;

	return 0;
}

int nvgpu_vidmem_init(struct mm_gk20a *mm)
{
	struct gk20a *g = mm->g;
	size_t size = g->ops.mm.get_vidmem_size ?
		g->ops.mm.get_vidmem_size(g) : 0;
	u64 bootstrap_base, bootstrap_size, base;
	u64 default_page_size = SZ_64K;
	int err;

	static struct nvgpu_alloc_carveout wpr_co =
		NVGPU_CARVEOUT("wpr-region", 0, SZ_16M);

	if (!size)
		return 0;

	wpr_co.base = size - SZ_256M;
	bootstrap_base = wpr_co.base;
	bootstrap_size = SZ_16M;
	base = default_page_size;

	/*
	 * Bootstrap allocator for use before the CE is initialized (CE
	 * initialization requires vidmem but we want to use the CE to zero
	 * out vidmem before allocating it...
	 */
	err = nvgpu_page_allocator_init(g, &g->mm.vidmem.bootstrap_allocator,
					"vidmem-bootstrap",
					bootstrap_base, bootstrap_size,
					SZ_4K, 0);

	err = nvgpu_page_allocator_init(g, &g->mm.vidmem.allocator,
					"vidmem",
					base, size - base,
					default_page_size,
					GPU_ALLOC_4K_VIDMEM_PAGES);
	if (err) {
		nvgpu_err(g, "Failed to register vidmem for size %zu: %d",
				size, err);
		return err;
	}

	/* Reserve bootstrap region in vidmem allocator */
	nvgpu_alloc_reserve_carveout(&g->mm.vidmem.allocator, &wpr_co);

	mm->vidmem.base = base;
	mm->vidmem.size = size - base;
	mm->vidmem.bootstrap_base = bootstrap_base;
	mm->vidmem.bootstrap_size = bootstrap_size;

	nvgpu_mutex_init(&mm->vidmem.first_clear_mutex);

	INIT_WORK(&mm->vidmem.clear_mem_worker, nvgpu_vidmem_clear_mem_worker);
	nvgpu_atomic64_set(&mm->vidmem.bytes_pending, 0);
	nvgpu_init_list_node(&mm->vidmem.clear_list_head);
	nvgpu_mutex_init(&mm->vidmem.clear_list_mutex);

	gk20a_dbg_info("registered vidmem: %zu MB", size / SZ_1M);

	return 0;
}

int nvgpu_vidmem_get_space(struct gk20a *g, u64 *space)
{
	struct nvgpu_allocator *allocator = &g->mm.vidmem.allocator;

	gk20a_dbg_fn("");

	if (!nvgpu_alloc_initialized(allocator))
		return -ENOSYS;

	nvgpu_mutex_acquire(&g->mm.vidmem.clear_list_mutex);
	*space = nvgpu_alloc_space(allocator) +
		nvgpu_atomic64_read(&g->mm.vidmem.bytes_pending);
	nvgpu_mutex_release(&g->mm.vidmem.clear_list_mutex);
	return 0;
}

int nvgpu_vidmem_clear(struct gk20a *g, struct nvgpu_mem *mem)
{
	struct gk20a_fence *gk20a_fence_out = NULL;
	struct gk20a_fence *gk20a_last_fence = NULL;
	struct nvgpu_page_alloc *alloc = NULL;
	void *sgl = NULL;
	int err = 0;

	if (g->mm.vidmem.ce_ctx_id == (u32)~0)
		return -EINVAL;

	alloc = nvgpu_vidmem_get_page_alloc(mem->priv.sgt->sgl);

	nvgpu_sgt_for_each_sgl(sgl, &alloc->sgt) {
		if (gk20a_last_fence)
			gk20a_fence_put(gk20a_last_fence);

		err = gk20a_ce_execute_ops(g,
			g->mm.vidmem.ce_ctx_id,
			0,
			nvgpu_sgt_get_phys(&alloc->sgt, sgl),
			nvgpu_sgt_get_length(&alloc->sgt, sgl),
			0x00000000,
			NVGPU_CE_DST_LOCATION_LOCAL_FB,
			NVGPU_CE_MEMSET,
			NULL,
			0,
			&gk20a_fence_out);

		if (err) {
			nvgpu_err(g,
				"Failed gk20a_ce_execute_ops[%d]", err);
			return err;
		}

		gk20a_last_fence = gk20a_fence_out;
	}

	if (gk20a_last_fence) {
		struct nvgpu_timeout timeout;

		nvgpu_timeout_init(g, &timeout,
				   gk20a_get_gr_idle_timeout(g),
				   NVGPU_TIMER_CPU_TIMER);

		do {
			err = gk20a_fence_wait(g, gk20a_last_fence,
					       gk20a_get_gr_idle_timeout(g));
		} while (err == -ERESTARTSYS &&
			 !nvgpu_timeout_expired(&timeout));

		gk20a_fence_put(gk20a_last_fence);
		if (err)
			nvgpu_err(g,
				"fence wait failed for CE execute ops");
	}

	return err;
}

struct nvgpu_mem *nvgpu_vidmem_get_pending_alloc(struct mm_gk20a *mm)
{
	struct nvgpu_mem *mem = NULL;

	nvgpu_mutex_acquire(&mm->vidmem.clear_list_mutex);
	if (!nvgpu_list_empty(&mm->vidmem.clear_list_head)) {
		mem = nvgpu_list_first_entry(&mm->vidmem.clear_list_head,
				nvgpu_mem, clear_list_entry);
		nvgpu_list_del(&mem->clear_list_entry);
	}
	nvgpu_mutex_release(&mm->vidmem.clear_list_mutex);

	return mem;
}
