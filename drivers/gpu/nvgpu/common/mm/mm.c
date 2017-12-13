/*
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

#include <nvgpu/mm.h>
#include <nvgpu/vm.h>
#include <nvgpu/dma.h>
#include <nvgpu/vm_area.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/vidmem.h>
#include <nvgpu/semaphore.h>
#include <nvgpu/pramin.h>
#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"

/*
 * Attempt to find a reserved memory area to determine PTE size for the passed
 * mapping. If no reserved area can be found use small pages.
 */
enum gmmu_pgsz_gk20a __get_pte_size_fixed_map(struct vm_gk20a *vm,
					      u64 base, u64 size)
{
	struct nvgpu_vm_area *vm_area;

	vm_area = nvgpu_vm_area_find(vm, base);
	if (!vm_area)
		return gmmu_page_size_small;

	return vm_area->pgsz_idx;
}

/*
 * This is for when the address space does not support unified address spaces.
 */
static enum gmmu_pgsz_gk20a __get_pte_size_split_addr(struct vm_gk20a *vm,
					       u64 base, u64 size)
{
	if (!base) {
		if (size >= vm->gmmu_page_sizes[gmmu_page_size_big])
			return gmmu_page_size_big;
		return gmmu_page_size_small;
	} else {
		if (base < __nv_gmmu_va_small_page_limit())
			return gmmu_page_size_small;
		else
			return gmmu_page_size_big;
	}
}

/*
 * This determines the PTE size for a given alloc. Used by both the GVA space
 * allocator and the mm core code so that agreement can be reached on how to
 * map allocations.
 *
 * The page size of a buffer is this:
 *
 *   o  If the VM doesn't support large pages then obviously small pages
 *      must be used.
 *   o  If the base address is non-zero (fixed address map):
 *      - Attempt to find a reserved memory area and use the page size
 *        based on that.
 *      - If no reserved page size is available, default to small pages.
 *   o  If the base is zero:
 *      - If the size is larger than or equal to the big page size, use big
 *        pages.
 *      - Otherwise use small pages.
 */
enum gmmu_pgsz_gk20a __get_pte_size(struct vm_gk20a *vm, u64 base, u64 size)
{
	struct gk20a *g = gk20a_from_vm(vm);

	if (!vm->big_pages)
		return gmmu_page_size_small;

	if (!nvgpu_is_enabled(g, NVGPU_MM_UNIFY_ADDRESS_SPACES))
		return __get_pte_size_split_addr(vm, base, size);

	if (base)
		return __get_pte_size_fixed_map(vm, base, size);

	if (size >= vm->gmmu_page_sizes[gmmu_page_size_big])
		return gmmu_page_size_big;
	return gmmu_page_size_small;
}

int nvgpu_mm_suspend(struct gk20a *g)
{
	nvgpu_log_info(g, "MM suspend running...");

	nvgpu_vidmem_thread_pause_sync(&g->mm);

	g->ops.mm.cbc_clean(g);
	g->ops.mm.l2_flush(g, false);

	nvgpu_log_info(g, "MM suspend done!");

	return 0;
}

u64 nvgpu_inst_block_addr(struct gk20a *g, struct nvgpu_mem *inst_block)
{
	if (g->mm.has_physical_mode)
		return nvgpu_mem_get_phys_addr(g, inst_block);
	else
		return nvgpu_mem_get_addr(g, inst_block);
}

void nvgpu_free_inst_block(struct gk20a *g, struct nvgpu_mem *inst_block)
{
	if (nvgpu_mem_is_valid(inst_block))
		nvgpu_dma_free(g, inst_block);
}

static int nvgpu_alloc_sysmem_flush(struct gk20a *g)
{
	return nvgpu_dma_alloc_sys(g, SZ_4K, &g->mm.sysmem_flush);
}

static void nvgpu_remove_mm_ce_support(struct mm_gk20a *mm)
{
	struct gk20a *g = gk20a_from_mm(mm);

	if (mm->vidmem.ce_ctx_id != (u32)~0)
		gk20a_ce_delete_context_priv(g, mm->vidmem.ce_ctx_id);

	mm->vidmem.ce_ctx_id = (u32)~0;

	nvgpu_vm_put(mm->ce.vm);
}

static void nvgpu_remove_mm_support(struct mm_gk20a *mm)
{
	struct gk20a *g = gk20a_from_mm(mm);

	if (g->ops.mm.fault_info_mem_destroy)
		g->ops.mm.fault_info_mem_destroy(g);

	if (g->ops.mm.remove_bar2_vm)
		g->ops.mm.remove_bar2_vm(g);

	if (g->ops.mm.is_bar1_supported(g)) {
		nvgpu_free_inst_block(g, &mm->bar1.inst_block);
		nvgpu_vm_put(mm->bar1.vm);
	}

	nvgpu_free_inst_block(g, &mm->pmu.inst_block);
	nvgpu_free_inst_block(g, &mm->hwpm.inst_block);
	nvgpu_vm_put(mm->pmu.vm);

	if (g->has_cde)
		nvgpu_vm_put(mm->cde.vm);

	nvgpu_semaphore_sea_destroy(g);
	nvgpu_vidmem_destroy(g);
	nvgpu_pd_cache_fini(g);
}

/* pmu vm, share channel_vm interfaces */
static int nvgpu_init_system_vm(struct mm_gk20a *mm)
{
	int err;
	struct gk20a *g = gk20a_from_mm(mm);
	struct nvgpu_mem *inst_block = &mm->pmu.inst_block;
	u32 big_page_size = g->ops.mm.get_default_big_page_size();
	u32 low_hole, aperture_size;

	/*
	 * No user region - so we will pass that as zero sized.
	 */
	low_hole = SZ_4K * 16;
	aperture_size = GK20A_PMU_VA_SIZE * 2;

	mm->pmu.aperture_size = GK20A_PMU_VA_SIZE;
	nvgpu_log_info(g, "pmu vm size = 0x%x", mm->pmu.aperture_size);

	mm->pmu.vm = nvgpu_vm_init(g, big_page_size,
				   low_hole,
				   aperture_size - low_hole,
				   aperture_size,
				   true,
				   false,
				   "system");
	if (!mm->pmu.vm)
		return -ENOMEM;

	err = g->ops.mm.alloc_inst_block(g, inst_block);
	if (err)
		goto clean_up_vm;
	g->ops.mm.init_inst_block(inst_block, mm->pmu.vm, big_page_size);

	return 0;

clean_up_vm:
	nvgpu_vm_put(mm->pmu.vm);
	return err;
}

static int nvgpu_init_hwpm(struct mm_gk20a *mm)
{
	int err;
	struct gk20a *g = gk20a_from_mm(mm);
	struct nvgpu_mem *inst_block = &mm->hwpm.inst_block;

	err = g->ops.mm.alloc_inst_block(g, inst_block);
	if (err)
		return err;
	g->ops.mm.init_inst_block(inst_block, mm->pmu.vm, 0);

	return 0;
}

static int nvgpu_init_cde_vm(struct mm_gk20a *mm)
{
	struct gk20a *g = gk20a_from_mm(mm);
	u32 big_page_size = g->ops.mm.get_default_big_page_size();

	mm->cde.vm = nvgpu_vm_init(g, big_page_size,
				   big_page_size << 10,
				   NV_MM_DEFAULT_KERNEL_SIZE,
				   NV_MM_DEFAULT_KERNEL_SIZE + NV_MM_DEFAULT_USER_SIZE,
				   false, false, "cde");
	if (!mm->cde.vm)
		return -ENOMEM;
	return 0;
}

static int nvgpu_init_ce_vm(struct mm_gk20a *mm)
{
	struct gk20a *g = gk20a_from_mm(mm);
	u32 big_page_size = g->ops.mm.get_default_big_page_size();

	mm->ce.vm = nvgpu_vm_init(g, big_page_size,
				  big_page_size << 10,
				  NV_MM_DEFAULT_KERNEL_SIZE,
				  NV_MM_DEFAULT_KERNEL_SIZE + NV_MM_DEFAULT_USER_SIZE,
				  false, false, "ce");
	if (!mm->ce.vm)
		return -ENOMEM;
	return 0;
}

void nvgpu_init_mm_ce_context(struct gk20a *g)
{
#if defined(CONFIG_GK20A_VIDMEM)
	if (g->mm.vidmem.size && (g->mm.vidmem.ce_ctx_id == (u32)~0)) {
		g->mm.vidmem.ce_ctx_id =
			gk20a_ce_create_context(g,
				gk20a_fifo_get_fast_ce_runlist_id(g),
				-1,
				-1);

		if (g->mm.vidmem.ce_ctx_id == (u32)~0)
			nvgpu_err(g,
				"Failed to allocate CE context for vidmem page clearing support");
	}
#endif
}

static int nvgpu_init_mm_reset_enable_hw(struct gk20a *g)
{
	if (g->ops.fb.reset)
		g->ops.fb.reset(g);

	if (g->ops.clock_gating.slcg_fb_load_gating_prod)
		g->ops.clock_gating.slcg_fb_load_gating_prod(g,
				g->slcg_enabled);
	if (g->ops.clock_gating.slcg_ltc_load_gating_prod)
		g->ops.clock_gating.slcg_ltc_load_gating_prod(g,
				g->slcg_enabled);
	if (g->ops.clock_gating.blcg_fb_load_gating_prod)
		g->ops.clock_gating.blcg_fb_load_gating_prod(g,
				g->blcg_enabled);
	if (g->ops.clock_gating.blcg_ltc_load_gating_prod)
		g->ops.clock_gating.blcg_ltc_load_gating_prod(g,
				g->blcg_enabled);

	if (g->ops.fb.init_fs_state)
		g->ops.fb.init_fs_state(g);

	return 0;
}

static int nvgpu_init_bar1_vm(struct mm_gk20a *mm)
{
	int err;
	struct gk20a *g = gk20a_from_mm(mm);
	struct nvgpu_mem *inst_block = &mm->bar1.inst_block;
	u32 big_page_size = g->ops.mm.get_default_big_page_size();

	mm->bar1.aperture_size = bar1_aperture_size_mb_gk20a() << 20;
	nvgpu_log_info(g, "bar1 vm size = 0x%x", mm->bar1.aperture_size);
	mm->bar1.vm = nvgpu_vm_init(g,
				    big_page_size,
				    SZ_64K,
				    mm->bar1.aperture_size - SZ_64K,
				    mm->bar1.aperture_size,
				    true, false,
				    "bar1");
	if (!mm->bar1.vm)
		return -ENOMEM;

	err = g->ops.mm.alloc_inst_block(g, inst_block);
	if (err)
		goto clean_up_vm;
	g->ops.mm.init_inst_block(inst_block, mm->bar1.vm, big_page_size);

	return 0;

clean_up_vm:
	nvgpu_vm_put(mm->bar1.vm);
	return err;
}

static int nvgpu_init_mm_setup_sw(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	int err;

	if (mm->sw_ready) {
		nvgpu_log_info(g, "skip init");
		return 0;
	}

	mm->g = g;
	nvgpu_mutex_init(&mm->l2_op_lock);

	/*TBD: make channel vm size configurable */
	mm->channel.user_size = NV_MM_DEFAULT_USER_SIZE -
		NV_MM_DEFAULT_KERNEL_SIZE;
	mm->channel.kernel_size = NV_MM_DEFAULT_KERNEL_SIZE;

	nvgpu_log_info(g, "channel vm size: user %dMB  kernel %dMB",
		   (int)(mm->channel.user_size >> 20),
		   (int)(mm->channel.kernel_size >> 20));

	nvgpu_init_pramin(mm);

	mm->vidmem.ce_ctx_id = (u32)~0;

	err = nvgpu_vidmem_init(mm);
	if (err)
		return err;

	/*
	 * this requires fixed allocations in vidmem which must be
	 * allocated before all other buffers
	 */
	if (g->ops.pmu.alloc_blob_space
			&& !nvgpu_is_enabled(g, NVGPU_MM_UNIFIED_MEMORY)) {
		err = g->ops.pmu.alloc_blob_space(g, 0, &g->acr.ucode_blob);
		if (err)
			return err;
	}

	err = nvgpu_alloc_sysmem_flush(g);
	if (err)
		return err;

	if (g->ops.mm.is_bar1_supported(g)) {
		err = nvgpu_init_bar1_vm(mm);
		if (err)
			return err;
	}
	if (g->ops.mm.init_bar2_vm) {
		err = g->ops.mm.init_bar2_vm(g);
		if (err)
			return err;
	}
	err = nvgpu_init_system_vm(mm);
	if (err)
		return err;

	err = nvgpu_init_hwpm(mm);
	if (err)
		return err;

	if (g->has_cde) {
		err = nvgpu_init_cde_vm(mm);
			if (err)
				return err;
	}

	err = nvgpu_init_ce_vm(mm);
	if (err)
		return err;

	mm->remove_support = nvgpu_remove_mm_support;
	mm->remove_ce_support = nvgpu_remove_mm_ce_support;

	mm->sw_ready = true;

	return 0;
}

int nvgpu_init_mm_support(struct gk20a *g)
{
	u32 err;

	err = nvgpu_init_mm_reset_enable_hw(g);
	if (err)
		return err;

	err = nvgpu_init_mm_setup_sw(g);
	if (err)
		return err;

	if (g->ops.mm.init_mm_setup_hw)
		err = g->ops.mm.init_mm_setup_hw(g);

	return err;
}

u32 nvgpu_mm_get_default_big_page_size(struct gk20a *g)
{
	u32 big_page_size;

	big_page_size = g->ops.mm.get_default_big_page_size();

	if (g->mm.disable_bigpage)
		big_page_size = 0;

	return big_page_size;
}

u32 nvgpu_mm_get_available_big_page_sizes(struct gk20a *g)
{
	u32 available_big_page_sizes = 0;

	if (!g->mm.disable_bigpage) {
		available_big_page_sizes =
			g->ops.mm.get_default_big_page_size();
		if (g->ops.mm.get_big_page_sizes)
			available_big_page_sizes |= g->ops.mm.get_big_page_sizes();
	}

	return available_big_page_sizes;
}
