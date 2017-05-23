/*
 * GK20A memory management
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/scatterlist.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/dma-attrs.h>
#include <linux/lcm.h>
#include <linux/platform/tegra/tegra_fd.h>
#include <uapi/linux/nvgpu.h>
#include <trace/events/gk20a.h>

#include <nvgpu/vm.h>
#include <nvgpu/vm_area.h>
#include <nvgpu/dma.h>
#include <nvgpu/kmem.h>
#include <nvgpu/timers.h>
#include <nvgpu/pramin.h>
#include <nvgpu/list.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/allocator.h>
#include <nvgpu/semaphore.h>
#include <nvgpu/page_allocator.h>
#include <nvgpu/log.h>
#include <nvgpu/bug.h>
#include <nvgpu/log2.h>
#include <nvgpu/enabled.h>

#include <nvgpu/linux/dma.h>

#include "gk20a.h"
#include "platform_gk20a.h"
#include "mm_gk20a.h"
#include "fence_gk20a.h"
#include "kind_gk20a.h"
#include "bus_gk20a.h"
#include "common/linux/os_linux.h"

#include <nvgpu/hw/gk20a/hw_gmmu_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ram_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pram_gk20a.h>
#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_bus_gk20a.h>
#include <nvgpu/hw/gk20a/hw_flush_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ltc_gk20a.h>

/*
 * Necessary while transitioning to less coupled code. Will be removed once
 * all the common APIs no longers have Linux stuff in them.
 */
#include "common/linux/vm_priv.h"

#if defined(CONFIG_GK20A_VIDMEM)
static void gk20a_vidmem_clear_mem_worker(struct work_struct *work);
#endif

void set_vidmem_page_alloc(struct scatterlist *sgl, u64 addr)
{
	/* set bit 0 to indicate vidmem allocation */
	sg_dma_address(sgl) = (addr | 1ULL);
}

bool is_vidmem_page_alloc(u64 addr)
{
	return !!(addr & 1ULL);
}

struct nvgpu_page_alloc *get_vidmem_page_alloc(struct scatterlist *sgl)
{
	u64 addr;

	addr = sg_dma_address(sgl);

	if (is_vidmem_page_alloc(addr))
		addr = addr & ~1ULL;
	else
		WARN_ON(1);

	return (struct nvgpu_page_alloc *)(uintptr_t)addr;
}

/*
 * GPU mapping life cycle
 * ======================
 *
 * Kernel mappings
 * ---------------
 *
 * Kernel mappings are created through vm.map(..., false):
 *
 *  - Mappings to the same allocations are reused and refcounted.
 *  - This path does not support deferred unmapping (i.e. kernel must wait for
 *    all hw operations on the buffer to complete before unmapping).
 *  - References to dmabuf are owned and managed by the (kernel) clients of
 *    the gk20a_vm layer.
 *
 *
 * User space mappings
 * -------------------
 *
 * User space mappings are created through as.map_buffer -> vm.map(..., true):
 *
 *  - Mappings to the same allocations are reused and refcounted.
 *  - This path supports deferred unmapping (i.e. we delay the actual unmapping
 *    until all hw operations have completed).
 *  - References to dmabuf are owned and managed by the vm_gk20a
 *    layer itself. vm.map acquires these refs, and sets
 *    mapped_buffer->own_mem_ref to record that we must release the refs when we
 *    actually unmap.
 *
 */

static int __must_check gk20a_init_system_vm(struct mm_gk20a *mm);
static int __must_check gk20a_init_bar1_vm(struct mm_gk20a *mm);
static int __must_check gk20a_init_hwpm(struct mm_gk20a *mm);
static int __must_check gk20a_init_cde_vm(struct mm_gk20a *mm);
static int __must_check gk20a_init_ce_vm(struct mm_gk20a *mm);

static struct gk20a *gk20a_vidmem_buf_owner(struct dma_buf *dmabuf);

struct gk20a_dmabuf_priv {
	struct nvgpu_mutex lock;

	struct gk20a *g;

	struct gk20a_comptag_allocator *comptag_allocator;
	struct gk20a_comptags comptags;

	struct dma_buf_attachment *attach;
	struct sg_table *sgt;

	int pin_count;

	struct nvgpu_list_node states;

	u64 buffer_id;
};

struct gk20a_vidmem_buf {
	struct gk20a *g;
	struct nvgpu_mem *mem;
	struct dma_buf *dmabuf;
	void *dmabuf_priv;
	void (*dmabuf_priv_delete)(void *);
};

static int gk20a_comptaglines_alloc(struct gk20a_comptag_allocator *allocator,
		u32 *offset, u32 len)
{
	unsigned long addr;
	int err = 0;

	nvgpu_mutex_acquire(&allocator->lock);
	addr = bitmap_find_next_zero_area(allocator->bitmap, allocator->size,
			0, len, 0);
	if (addr < allocator->size) {
		/* number zero is reserved; bitmap base is 1 */
		*offset = 1 + addr;
		bitmap_set(allocator->bitmap, addr, len);
	} else {
		err = -ENOMEM;
	}
	nvgpu_mutex_release(&allocator->lock);

	return err;
}

static void gk20a_comptaglines_free(struct gk20a_comptag_allocator *allocator,
		u32 offset, u32 len)
{
	/* number zero is reserved; bitmap base is 1 */
	u32 addr = offset - 1;
	WARN_ON(offset == 0);
	WARN_ON(addr > allocator->size);
	WARN_ON(addr + len > allocator->size);

	nvgpu_mutex_acquire(&allocator->lock);
	bitmap_clear(allocator->bitmap, addr, len);
	nvgpu_mutex_release(&allocator->lock);
}

static void gk20a_mm_delete_priv(void *_priv)
{
	struct gk20a_buffer_state *s, *s_tmp;
	struct gk20a_dmabuf_priv *priv = _priv;
	struct gk20a *g;

	if (!priv)
		return;

	g = priv->g;

	if (priv->comptags.lines) {
		BUG_ON(!priv->comptag_allocator);
		gk20a_comptaglines_free(priv->comptag_allocator,
				priv->comptags.offset,
				priv->comptags.allocated_lines);
	}

	/* Free buffer states */
	nvgpu_list_for_each_entry_safe(s, s_tmp, &priv->states,
				gk20a_buffer_state, list) {
		gk20a_fence_put(s->fence);
		nvgpu_list_del(&s->list);
		nvgpu_kfree(g, s);
	}

	nvgpu_kfree(g, priv);
}

struct sg_table *gk20a_mm_pin(struct device *dev, struct dma_buf *dmabuf)
{
	struct gk20a_dmabuf_priv *priv;

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (WARN_ON(!priv))
		return ERR_PTR(-EINVAL);

	nvgpu_mutex_acquire(&priv->lock);

	if (priv->pin_count == 0) {
		priv->attach = dma_buf_attach(dmabuf, dev);
		if (IS_ERR(priv->attach)) {
			nvgpu_mutex_release(&priv->lock);
			return (struct sg_table *)priv->attach;
		}

		priv->sgt = dma_buf_map_attachment(priv->attach,
						   DMA_BIDIRECTIONAL);
		if (IS_ERR(priv->sgt)) {
			dma_buf_detach(dmabuf, priv->attach);
			nvgpu_mutex_release(&priv->lock);
			return priv->sgt;
		}
	}

	priv->pin_count++;
	nvgpu_mutex_release(&priv->lock);
	return priv->sgt;
}

void gk20a_mm_unpin(struct device *dev, struct dma_buf *dmabuf,
		    struct sg_table *sgt)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(dmabuf, dev);
	dma_addr_t dma_addr;

	if (IS_ERR(priv) || !priv)
		return;

	nvgpu_mutex_acquire(&priv->lock);
	WARN_ON(priv->sgt != sgt);
	priv->pin_count--;
	WARN_ON(priv->pin_count < 0);
	dma_addr = sg_dma_address(priv->sgt->sgl);
	if (priv->pin_count == 0) {
		dma_buf_unmap_attachment(priv->attach, priv->sgt,
					 DMA_BIDIRECTIONAL);
		dma_buf_detach(dmabuf, priv->attach);
	}
	nvgpu_mutex_release(&priv->lock);
}

void gk20a_get_comptags(struct device *dev, struct dma_buf *dmabuf,
			struct gk20a_comptags *comptags)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(dmabuf, dev);

	if (!comptags)
		return;

	if (!priv) {
		memset(comptags, 0, sizeof(*comptags));
		return;
	}

	*comptags = priv->comptags;
}

int gk20a_alloc_comptags(struct gk20a *g,
			 struct device *dev,
			 struct dma_buf *dmabuf,
			 struct gk20a_comptag_allocator *allocator,
			 u32 lines, bool user_mappable,
			 u64 *ctag_map_win_size,
			 u32 *ctag_map_win_ctagline)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(dmabuf, dev);
	u32 ctaglines_allocsize;
	u32 ctagline_align;
	u32 offset;
	u32 alignment_lines;
	const u32 aggregate_cacheline_sz =
		g->gr.cacheline_size * g->gr.slices_per_ltc *
		g->ltc_count;
	const u32 small_pgsz = 4096;
	int err;

	if (!priv)
		return -ENOSYS;

	if (!lines)
		return -EINVAL;

	if (!user_mappable) {
		ctaglines_allocsize = lines;
		ctagline_align = 1;
	} else {
		/*
		 * For security, align the allocation on a page, and reserve
		 * whole pages. Unfortunately, we cannot ask the allocator to
		 * align here, since compbits per cacheline is not always a
		 * power of two. So, we just have to allocate enough extra that
		 * we're guaranteed to find a ctagline inside the allocation so
		 * that: 1) it is the first ctagline in a cacheline that starts
		 * at a page boundary, and 2) we can add enough overallocation
		 * that the ctaglines of the succeeding allocation are on
		 * different page than ours.
		 */

		ctagline_align =
			(lcm(aggregate_cacheline_sz, small_pgsz) /
			 aggregate_cacheline_sz) *
			g->gr.comptags_per_cacheline;

		ctaglines_allocsize =
			/* for alignment */
			ctagline_align +

			/* lines rounded up to cachelines */
			DIV_ROUND_UP(lines, g->gr.comptags_per_cacheline) *
			g->gr.comptags_per_cacheline +

			/* trail-padding */
			DIV_ROUND_UP(aggregate_cacheline_sz, small_pgsz) *
			g->gr.comptags_per_cacheline;

		if (ctaglines_allocsize < lines)
			return -EINVAL; /* integer overflow */
	}

	/* store the allocator so we can use it when we free the ctags */
	priv->comptag_allocator = allocator;
	err = gk20a_comptaglines_alloc(allocator, &offset,
			       ctaglines_allocsize);
	if (err)
		return err;

	/*
	 * offset needs to be at the start of a page/cacheline boundary;
	 * prune the preceding ctaglines that were allocated for alignment.
	 */
	alignment_lines =
		DIV_ROUND_UP(offset, ctagline_align) * ctagline_align - offset;
	if (alignment_lines) {
		gk20a_comptaglines_free(allocator, offset, alignment_lines);
		offset += alignment_lines;
		ctaglines_allocsize -= alignment_lines;
	}

	/*
	 * check if we can prune the trailing, too; we just need to reserve
	 * whole pages and ctagcachelines.
	 */
	if (user_mappable) {
		u32 needed_cachelines =
			DIV_ROUND_UP(lines, g->gr.comptags_per_cacheline);
		u32 needed_bytes = round_up(needed_cachelines *
					    aggregate_cacheline_sz,
					    small_pgsz);
		u32 first_unneeded_cacheline =
			DIV_ROUND_UP(needed_bytes, aggregate_cacheline_sz);
		u32 needed_ctaglines = first_unneeded_cacheline *
			g->gr.comptags_per_cacheline;
		u64 win_size;

		if (needed_ctaglines < ctaglines_allocsize) {
			gk20a_comptaglines_free(allocator,
				offset + needed_ctaglines,
				ctaglines_allocsize - needed_ctaglines);
			ctaglines_allocsize = needed_ctaglines;
		}

		*ctag_map_win_ctagline = offset;
		win_size =
			DIV_ROUND_UP(lines, g->gr.comptags_per_cacheline) *
			aggregate_cacheline_sz;

		*ctag_map_win_size = round_up(win_size, small_pgsz);
	}

	priv->comptags.offset = offset;
	priv->comptags.lines = lines;
	priv->comptags.allocated_lines = ctaglines_allocsize;
	priv->comptags.user_mappable = user_mappable;

	return 0;
}




static int gk20a_init_mm_reset_enable_hw(struct gk20a *g)
{
	gk20a_dbg_fn("");
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

static void gk20a_vidmem_destroy(struct gk20a *g)
{
#if defined(CONFIG_GK20A_VIDMEM)
	if (nvgpu_alloc_initialized(&g->mm.vidmem.allocator))
		nvgpu_alloc_destroy(&g->mm.vidmem.allocator);
#endif
}

static void gk20a_remove_mm_ce_support(struct mm_gk20a *mm)
{
	struct gk20a *g = gk20a_from_mm(mm);

	if (mm->vidmem.ce_ctx_id != (u32)~0)
		gk20a_ce_delete_context_priv(g, mm->vidmem.ce_ctx_id);

	mm->vidmem.ce_ctx_id = (u32)~0;

	nvgpu_vm_put(mm->ce.vm);
}

static void gk20a_remove_mm_support(struct mm_gk20a *mm)
{
	struct gk20a *g = gk20a_from_mm(mm);

	if (g->ops.mm.fault_info_mem_destroy)
		g->ops.mm.fault_info_mem_destroy(g);

	if (g->ops.mm.remove_bar2_vm)
		g->ops.mm.remove_bar2_vm(g);

	if (g->ops.mm.is_bar1_supported(g)) {
		gk20a_free_inst_block(g, &mm->bar1.inst_block);
		nvgpu_vm_put(mm->bar1.vm);
	}

	gk20a_free_inst_block(g, &mm->pmu.inst_block);
	gk20a_free_inst_block(g, &mm->hwpm.inst_block);
	nvgpu_vm_put(mm->pmu.vm);
	nvgpu_vm_put(mm->cde.vm);

	gk20a_semaphore_sea_destroy(g);
	gk20a_vidmem_destroy(g);
	nvgpu_pd_cache_fini(g);
}

static int gk20a_alloc_sysmem_flush(struct gk20a *g)
{
	return nvgpu_dma_alloc_sys(g, SZ_4K, &g->mm.sysmem_flush);
}

#if defined(CONFIG_GK20A_VIDMEM)
static int gk20a_vidmem_clear_all(struct gk20a *g)
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
#endif

static int gk20a_init_vidmem(struct mm_gk20a *mm)
{
#if defined(CONFIG_GK20A_VIDMEM)
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

	INIT_WORK(&mm->vidmem.clear_mem_worker, gk20a_vidmem_clear_mem_worker);
	atomic64_set(&mm->vidmem.bytes_pending, 0);
	nvgpu_init_list_node(&mm->vidmem.clear_list_head);
	nvgpu_mutex_init(&mm->vidmem.clear_list_mutex);

	gk20a_dbg_info("registered vidmem: %zu MB", size / SZ_1M);

#endif
	return 0;
}

int gk20a_init_mm_setup_sw(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	int err;

	gk20a_dbg_fn("");

	if (mm->sw_ready) {
		gk20a_dbg_fn("skip init");
		return 0;
	}

	mm->g = g;
	nvgpu_mutex_init(&mm->l2_op_lock);

	/*TBD: make channel vm size configurable */
	mm->channel.user_size = NV_MM_DEFAULT_USER_SIZE -
		NV_MM_DEFAULT_KERNEL_SIZE;
	mm->channel.kernel_size = NV_MM_DEFAULT_KERNEL_SIZE;

	gk20a_dbg_info("channel vm size: user %dMB  kernel %dMB",
		       (int)(mm->channel.user_size >> 20),
		       (int)(mm->channel.kernel_size >> 20));

	nvgpu_init_pramin(mm);

	mm->vidmem.ce_ctx_id = (u32)~0;

	err = gk20a_init_vidmem(mm);
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

	err = gk20a_alloc_sysmem_flush(g);
	if (err)
		return err;

	if (g->ops.mm.is_bar1_supported(g)) {
		err = gk20a_init_bar1_vm(mm);
		if (err)
			return err;
	}
	if (g->ops.mm.init_bar2_vm) {
		err = g->ops.mm.init_bar2_vm(g);
		if (err)
			return err;
	}
	err = gk20a_init_system_vm(mm);
	if (err)
		return err;

	err = gk20a_init_hwpm(mm);
	if (err)
		return err;

	err = gk20a_init_cde_vm(mm);
	if (err)
		return err;

	err = gk20a_init_ce_vm(mm);
	if (err)
		return err;

	mm->remove_support = gk20a_remove_mm_support;
	mm->remove_ce_support = gk20a_remove_mm_ce_support;

	mm->sw_ready = true;

	gk20a_dbg_fn("done");
	return 0;
}

/* make sure gk20a_init_mm_support is called before */
int gk20a_init_mm_setup_hw(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	int err;

	gk20a_dbg_fn("");

	g->ops.fb.set_mmu_page_size(g);
	if (g->ops.fb.set_use_full_comp_tag_line)
		mm->use_full_comp_tag_line =
			g->ops.fb.set_use_full_comp_tag_line(g);

	g->ops.fb.init_hw(g);

	if (g->ops.bus.bar1_bind)
		g->ops.bus.bar1_bind(g, &mm->bar1.inst_block);

	if (g->ops.mm.init_bar2_mm_hw_setup) {
		err = g->ops.mm.init_bar2_mm_hw_setup(g);
		if (err)
			return err;
	}

	if (gk20a_mm_fb_flush(g) || gk20a_mm_fb_flush(g))
		return -EBUSY;

	gk20a_dbg_fn("done");
	return 0;
}

int gk20a_init_mm_support(struct gk20a *g)
{
	u32 err;

	err = gk20a_init_mm_reset_enable_hw(g);
	if (err)
		return err;

	err = gk20a_init_mm_setup_sw(g);
	if (err)
		return err;

	if (g->ops.mm.init_mm_setup_hw)
		err = g->ops.mm.init_mm_setup_hw(g);

	return err;
}

void gk20a_init_mm_ce_context(struct gk20a *g)
{
#if defined(CONFIG_GK20A_VIDMEM)
	if (g->mm.vidmem.size && (g->mm.vidmem.ce_ctx_id == (u32)~0)) {
		g->mm.vidmem.ce_ctx_id =
			gk20a_ce_create_context_with_cb(g,
				gk20a_fifo_get_fast_ce_runlist_id(g),
				-1,
				-1,
				-1,
				NULL);

		if (g->mm.vidmem.ce_ctx_id == (u32)~0)
			nvgpu_err(g,
				"Failed to allocate CE context for vidmem page clearing support");
	}
#endif
}

int gk20a_mm_pde_coverage_bit_count(struct vm_gk20a *vm)
{
	return vm->mmu_levels[0].lo_bit[0];
}

int nvgpu_vm_get_buffers(struct vm_gk20a *vm,
			 struct nvgpu_mapped_buf ***mapped_buffers,
			 int *num_buffers)
{
	struct nvgpu_mapped_buf *mapped_buffer;
	struct nvgpu_mapped_buf **buffer_list;
	struct nvgpu_rbtree_node *node = NULL;
	int i = 0;

	if (vm->userspace_managed) {
		*mapped_buffers = NULL;
		*num_buffers = 0;
		return 0;
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	buffer_list = nvgpu_big_zalloc(vm->mm->g, sizeof(*buffer_list) *
				       vm->num_user_mapped_buffers);
	if (!buffer_list) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		return -ENOMEM;
	}

	nvgpu_rbtree_enum_start(0, &node, vm->mapped_buffers);
	while (node) {
		mapped_buffer = mapped_buffer_from_rbtree_node(node);
		if (mapped_buffer->user_mapped) {
			buffer_list[i] = mapped_buffer;
			kref_get(&mapped_buffer->ref);
			i++;
		}
		nvgpu_rbtree_enum_next(&node, node);
	}

	BUG_ON(i != vm->num_user_mapped_buffers);

	*num_buffers = vm->num_user_mapped_buffers;
	*mapped_buffers = buffer_list;

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return 0;
}

void gk20a_vm_unmap_locked_kref(struct kref *ref)
{
	struct nvgpu_mapped_buf *mapped_buffer =
		container_of(ref, struct nvgpu_mapped_buf, ref);
	nvgpu_vm_unmap_locked(mapped_buffer, mapped_buffer->vm->kref_put_batch);
}

void nvgpu_vm_put_buffers(struct vm_gk20a *vm,
				 struct nvgpu_mapped_buf **mapped_buffers,
				 int num_buffers)
{
	int i;
	struct vm_gk20a_mapping_batch batch;

	if (num_buffers == 0)
		return;

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	nvgpu_vm_mapping_batch_start(&batch);
	vm->kref_put_batch = &batch;

	for (i = 0; i < num_buffers; ++i)
		kref_put(&mapped_buffers[i]->ref,
			 gk20a_vm_unmap_locked_kref);

	vm->kref_put_batch = NULL;
	nvgpu_vm_mapping_batch_finish_locked(vm, &batch);
	nvgpu_mutex_release(&vm->update_gmmu_lock);

	nvgpu_big_free(vm->mm->g, mapped_buffers);
}

static void nvgpu_vm_unmap_user(struct vm_gk20a *vm, u64 offset,
				struct vm_gk20a_mapping_batch *batch)
{
	struct gk20a *g = vm->mm->g;
	struct nvgpu_mapped_buf *mapped_buffer;

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	mapped_buffer = __nvgpu_vm_find_mapped_buf(vm, offset);
	if (!mapped_buffer) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		nvgpu_err(g, "invalid addr to unmap 0x%llx", offset);
		return;
	}

	if (mapped_buffer->flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET) {
		struct nvgpu_timeout timeout;

		nvgpu_mutex_release(&vm->update_gmmu_lock);

		nvgpu_timeout_init(vm->mm->g, &timeout, 10000,
				   NVGPU_TIMER_RETRY_TIMER);
		do {
			if (atomic_read(&mapped_buffer->ref.refcount) == 1)
				break;
			nvgpu_udelay(5);
		} while (!nvgpu_timeout_expired_msg(&timeout,
					    "sync-unmap failed on 0x%llx"));

		nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	}

	if (mapped_buffer->user_mapped == 0) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		nvgpu_err(g, "addr already unmapped from user 0x%llx", offset);
		return;
	}

	mapped_buffer->user_mapped--;
	if (mapped_buffer->user_mapped == 0)
		vm->num_user_mapped_buffers--;

	vm->kref_put_batch = batch;
	kref_put(&mapped_buffer->ref, gk20a_vm_unmap_locked_kref);
	vm->kref_put_batch = NULL;

	nvgpu_mutex_release(&vm->update_gmmu_lock);
}

int setup_buffer_kind_and_compression(struct vm_gk20a *vm,
				      u32 flags,
				      struct buffer_attrs *bfr,
				      enum gmmu_pgsz_gk20a pgsz_idx)
{
	bool kind_compressible;
	struct gk20a *g = gk20a_from_vm(vm);
	int ctag_granularity = g->ops.fb.compression_page_size(g);

	if (unlikely(bfr->kind_v == gmmu_pte_kind_invalid_v()))
		bfr->kind_v = gmmu_pte_kind_pitch_v();

	if (unlikely(!gk20a_kind_is_supported(bfr->kind_v))) {
		nvgpu_err(g, "kind 0x%x not supported", bfr->kind_v);
		return -EINVAL;
	}

	bfr->uc_kind_v = gmmu_pte_kind_invalid_v();
	/* find a suitable uncompressed kind if it becomes necessary later */
	kind_compressible = gk20a_kind_is_compressible(bfr->kind_v);
	if (kind_compressible) {
		bfr->uc_kind_v = gk20a_get_uncompressed_kind(bfr->kind_v);
		if (unlikely(bfr->uc_kind_v == gmmu_pte_kind_invalid_v())) {
			/* shouldn't happen, but it is worth cross-checking */
			nvgpu_err(g, "comptag kind 0x%x can't be"
				   " downgraded to uncompressed kind",
				   bfr->kind_v);
			return -EINVAL;
		}
	}
	/* comptags only supported for suitable kinds, 128KB pagesize */
	if (kind_compressible &&
	    vm->gmmu_page_sizes[pgsz_idx] < g->ops.fb.compressible_page_size(g)) {
		/* it is safe to fall back to uncompressed as
		   functionality is not harmed */
		bfr->kind_v = bfr->uc_kind_v;
		kind_compressible = false;
	}
	if (kind_compressible)
		bfr->ctag_lines = DIV_ROUND_UP_ULL(bfr->size, ctag_granularity);
	else
		bfr->ctag_lines = 0;

	return 0;
}

enum nvgpu_aperture gk20a_dmabuf_aperture(struct gk20a *g,
					  struct dma_buf *dmabuf)
{
	struct gk20a *buf_owner = gk20a_vidmem_buf_owner(dmabuf);
	bool unified_memory = nvgpu_is_enabled(g, NVGPU_MM_UNIFIED_MEMORY);

	if (buf_owner == NULL) {
		/* Not nvgpu-allocated, assume system memory */
		return APERTURE_SYSMEM;
	} else if (WARN_ON(buf_owner == g && unified_memory)) {
		/* Looks like our video memory, but this gpu doesn't support
		 * it. Warn about a bug and bail out */
		nvgpu_warn(g,
			"dmabuf is our vidmem but we don't have local vidmem");
		return APERTURE_INVALID;
	} else if (buf_owner != g) {
		/* Someone else's vidmem */
		return APERTURE_INVALID;
	} else {
		/* Yay, buf_owner == g */
		return APERTURE_VIDMEM;
	}
}

#if defined(CONFIG_GK20A_VIDMEM)
static struct sg_table *gk20a_vidbuf_map_dma_buf(
	struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	struct gk20a_vidmem_buf *buf = attach->dmabuf->priv;

	return buf->mem->priv.sgt;
}

static void gk20a_vidbuf_unmap_dma_buf(struct dma_buf_attachment *attach,
				       struct sg_table *sgt,
				       enum dma_data_direction dir)
{
}

static void gk20a_vidbuf_release(struct dma_buf *dmabuf)
{
	struct gk20a_vidmem_buf *buf = dmabuf->priv;

	gk20a_dbg_fn("");

	if (buf->dmabuf_priv)
		buf->dmabuf_priv_delete(buf->dmabuf_priv);

	nvgpu_dma_free(buf->g, buf->mem);
	nvgpu_kfree(buf->g, buf);
}

static void *gk20a_vidbuf_kmap(struct dma_buf *dmabuf, unsigned long page_num)
{
	WARN_ON("Not supported");
	return NULL;
}

static void *gk20a_vidbuf_kmap_atomic(struct dma_buf *dmabuf,
				      unsigned long page_num)
{
	WARN_ON("Not supported");
	return NULL;
}

static int gk20a_vidbuf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static int gk20a_vidbuf_set_private(struct dma_buf *dmabuf,
		struct device *dev, void *priv, void (*delete)(void *priv))
{
	struct gk20a_vidmem_buf *buf = dmabuf->priv;

	buf->dmabuf_priv = priv;
	buf->dmabuf_priv_delete = delete;

	return 0;
}

static void *gk20a_vidbuf_get_private(struct dma_buf *dmabuf,
		struct device *dev)
{
	struct gk20a_vidmem_buf *buf = dmabuf->priv;

	return buf->dmabuf_priv;
}

static const struct dma_buf_ops gk20a_vidbuf_ops = {
	.map_dma_buf      = gk20a_vidbuf_map_dma_buf,
	.unmap_dma_buf    = gk20a_vidbuf_unmap_dma_buf,
	.release          = gk20a_vidbuf_release,
	.kmap_atomic      = gk20a_vidbuf_kmap_atomic,
	.kmap             = gk20a_vidbuf_kmap,
	.mmap             = gk20a_vidbuf_mmap,
	.set_drvdata      = gk20a_vidbuf_set_private,
	.get_drvdata      = gk20a_vidbuf_get_private,
};

static struct dma_buf *gk20a_vidbuf_export(struct gk20a_vidmem_buf *buf)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.priv = buf;
	exp_info.ops = &gk20a_vidbuf_ops;
	exp_info.size = buf->mem->size;
	exp_info.flags = O_RDWR;

	return dma_buf_export(&exp_info);
#else
	return dma_buf_export(buf, &gk20a_vidbuf_ops, buf->mem->size,
			O_RDWR, NULL);
#endif
}
#endif

static struct gk20a *gk20a_vidmem_buf_owner(struct dma_buf *dmabuf)
{
#if defined(CONFIG_GK20A_VIDMEM)
	struct gk20a_vidmem_buf *buf = dmabuf->priv;

	if (dmabuf->ops != &gk20a_vidbuf_ops)
		return NULL;

	return buf->g;
#else
	return NULL;
#endif
}

int gk20a_vidmem_buf_alloc(struct gk20a *g, size_t bytes)
{
#if defined(CONFIG_GK20A_VIDMEM)
	struct gk20a_vidmem_buf *buf;
	int err = 0, fd;

	gk20a_dbg_fn("");

	buf = nvgpu_kzalloc(g, sizeof(*buf));
	if (!buf)
		return -ENOMEM;

	buf->g = g;

	if (!g->mm.vidmem.cleared) {
		nvgpu_mutex_acquire(&g->mm.vidmem.first_clear_mutex);
		if (!g->mm.vidmem.cleared) {
			err = gk20a_vidmem_clear_all(g);
			if (err) {
				nvgpu_err(g,
				          "failed to clear whole vidmem");
				goto err_kfree;
			}
		}
		nvgpu_mutex_release(&g->mm.vidmem.first_clear_mutex);
	}

	buf->mem = nvgpu_kzalloc(g, sizeof(struct nvgpu_mem));
	if (!buf->mem)
		goto err_kfree;

	buf->mem->mem_flags |= NVGPU_MEM_FLAG_USER_MEM;

	err = nvgpu_dma_alloc_vid(g, bytes, buf->mem);
	if (err)
		goto err_memfree;

	buf->dmabuf = gk20a_vidbuf_export(buf);
	if (IS_ERR(buf->dmabuf)) {
		err = PTR_ERR(buf->dmabuf);
		goto err_bfree;
	}

	fd = tegra_alloc_fd(current->files, 1024, O_RDWR);
	if (fd < 0) {
		/* ->release frees what we have done */
		dma_buf_put(buf->dmabuf);
		return fd;
	}

	/* fclose() on this drops one ref, freeing the dma buf */
	fd_install(fd, buf->dmabuf->file);

	return fd;

err_bfree:
	nvgpu_dma_free(g, buf->mem);
err_memfree:
	nvgpu_kfree(g, buf->mem);
err_kfree:
	nvgpu_kfree(g, buf);
	return err;
#else
	return -ENOSYS;
#endif
}

int gk20a_vidmem_get_space(struct gk20a *g, u64 *space)
{
#if defined(CONFIG_GK20A_VIDMEM)
	struct nvgpu_allocator *allocator = &g->mm.vidmem.allocator;

	gk20a_dbg_fn("");

	if (!nvgpu_alloc_initialized(allocator))
		return -ENOSYS;

	nvgpu_mutex_acquire(&g->mm.vidmem.clear_list_mutex);
	*space = nvgpu_alloc_space(allocator) +
		atomic64_read(&g->mm.vidmem.bytes_pending);
	nvgpu_mutex_release(&g->mm.vidmem.clear_list_mutex);
	return 0;
#else
	return -ENOSYS;
#endif
}

int gk20a_vidbuf_access_memory(struct gk20a *g, struct dma_buf *dmabuf,
		void *buffer, u64 offset, u64 size, u32 cmd)
{
#if defined(CONFIG_GK20A_VIDMEM)
	struct gk20a_vidmem_buf *vidmem_buf;
	struct nvgpu_mem *mem;
	int err = 0;

	if (gk20a_dmabuf_aperture(g, dmabuf) != APERTURE_VIDMEM)
		return -EINVAL;

	vidmem_buf = dmabuf->priv;
	mem = vidmem_buf->mem;

	switch (cmd) {
	case NVGPU_DBG_GPU_IOCTL_ACCESS_FB_MEMORY_CMD_READ:
		nvgpu_mem_rd_n(g, mem, offset, buffer, size);
		break;

	case NVGPU_DBG_GPU_IOCTL_ACCESS_FB_MEMORY_CMD_WRITE:
		nvgpu_mem_wr_n(g, mem, offset, buffer, size);
		break;

	default:
		err = -EINVAL;
	}

	return err;
#else
	return -ENOSYS;
#endif
}

int nvgpu_vm_get_compbits_info(struct vm_gk20a *vm,
			       u64 mapping_gva,
			       u64 *compbits_win_size,
			       u32 *compbits_win_ctagline,
			       u32 *mapping_ctagline,
			       u32 *flags)
{
	struct nvgpu_mapped_buf *mapped_buffer;
	struct gk20a *g = vm->mm->g;

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	mapped_buffer = __nvgpu_vm_find_mapped_buf(vm, mapping_gva);

	if (!mapped_buffer || !mapped_buffer->user_mapped)
	{
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		nvgpu_err(g, "%s: bad offset 0x%llx", __func__, mapping_gva);
		return -EFAULT;
	}

	*compbits_win_size = 0;
	*compbits_win_ctagline = 0;
	*mapping_ctagline = 0;
	*flags = 0;

	if (mapped_buffer->ctag_offset)
		*flags |= NVGPU_AS_GET_BUFFER_COMPBITS_INFO_FLAGS_HAS_COMPBITS;

	if (mapped_buffer->ctags_mappable)
	{
		*flags |= NVGPU_AS_GET_BUFFER_COMPBITS_INFO_FLAGS_MAPPABLE;
		*compbits_win_size = mapped_buffer->ctag_map_win_size;
		*compbits_win_ctagline = mapped_buffer->ctag_map_win_ctagline;
		*mapping_ctagline = mapped_buffer->ctag_offset;
	}

	nvgpu_mutex_release(&vm->update_gmmu_lock);
	return 0;
}


int nvgpu_vm_map_compbits(struct vm_gk20a *vm,
			  u64 mapping_gva,
			  u64 *compbits_win_gva,
			  u64 *mapping_iova,
			  u32 flags)
{
	struct nvgpu_mapped_buf *mapped_buffer;
	struct gk20a *g = gk20a_from_vm(vm);
	const bool fixed_mapping =
		(flags & NVGPU_AS_MAP_BUFFER_COMPBITS_FLAGS_FIXED_OFFSET) != 0;

	if (vm->userspace_managed && !fixed_mapping) {
		nvgpu_err(g,
			  "%s: non-fixed-offset mapping is not available on userspace managed address spaces",
			  __func__);
		return -EFAULT;
	}

	if (fixed_mapping && !vm->userspace_managed) {
		nvgpu_err(g,
			  "%s: fixed-offset mapping is available only on userspace managed address spaces",
			  __func__);
		return -EFAULT;
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	mapped_buffer =	__nvgpu_vm_find_mapped_buf(vm, mapping_gva);

	if (!mapped_buffer || !mapped_buffer->user_mapped) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		nvgpu_err(g, "%s: bad offset 0x%llx", __func__, mapping_gva);
		return -EFAULT;
	}

	if (!mapped_buffer->ctags_mappable) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		nvgpu_err(g, "%s: comptags not mappable, offset 0x%llx",
			  __func__, mapping_gva);
		return -EFAULT;
	}

	if (!mapped_buffer->ctag_map_win_addr) {
		const u32 small_pgsz_index = 0; /* small pages, 4K */
		const u32 aggregate_cacheline_sz =
			g->gr.cacheline_size * g->gr.slices_per_ltc *
			g->ltc_count;

		/* first aggregate cacheline to map */
		u32 cacheline_start; /* inclusive */

		/* offset of the start cacheline (will be page aligned) */
		u64 cacheline_offset_start;

		if (!mapped_buffer->ctag_map_win_size) {
			nvgpu_mutex_release(&vm->update_gmmu_lock);
			nvgpu_err(g,
				  "%s: mapping 0x%llx does not have "
				  "mappable comptags",
				  __func__, mapping_gva);
			return -EFAULT;
		}

		cacheline_start = mapped_buffer->ctag_offset /
			g->gr.comptags_per_cacheline;
		cacheline_offset_start =
			(u64)cacheline_start * aggregate_cacheline_sz;

		if (fixed_mapping) {
			struct buffer_attrs bfr;
			int err;
			struct nvgpu_vm_area *vm_area = NULL;

			memset(&bfr, 0, sizeof(bfr));

			bfr.pgsz_idx = small_pgsz_index;

			err = nvgpu_vm_area_validate_buffer(
				vm, *compbits_win_gva, mapped_buffer->ctag_map_win_size,
				bfr.pgsz_idx, &vm_area);

			if (err) {
				nvgpu_mutex_release(&vm->update_gmmu_lock);
				return err;
			}

			if (vm_area) {
				/* this would create a dangling GPU VA
				 * pointer if the space is freed
				 * before before the buffer is
				 * unmapped */
				nvgpu_mutex_release(&vm->update_gmmu_lock);
				nvgpu_err(g,
					  "%s: comptags cannot be mapped into allocated space",
					  __func__);
				return -EINVAL;
			}
		}

		mapped_buffer->ctag_map_win_addr =
			g->ops.mm.gmmu_map(
				vm,
				!fixed_mapping ? 0 : *compbits_win_gva, /* va */
				g->gr.compbit_store.mem.priv.sgt,
				cacheline_offset_start, /* sg offset */
				mapped_buffer->ctag_map_win_size, /* size */
				small_pgsz_index,
				0, /* kind */
				0, /* ctag_offset */
				NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_TRUE,
				gk20a_mem_flag_read_only,
				false, /* clear_ctags */
				false, /* sparse */
				false, /* priv */
				NULL,  /* mapping_batch handle */
				g->gr.compbit_store.mem.aperture);

		if (!mapped_buffer->ctag_map_win_addr) {
			nvgpu_mutex_release(&vm->update_gmmu_lock);
			nvgpu_err(g,
				  "%s: failed to map comptags for mapping 0x%llx",
				  __func__, mapping_gva);
			return -ENOMEM;
		}
	} else if (fixed_mapping && *compbits_win_gva &&
		   mapped_buffer->ctag_map_win_addr != *compbits_win_gva) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		nvgpu_err(g,
			  "%s: re-requesting comptags map into mismatching address. buffer offset 0x"
			  "%llx, existing comptag map at 0x%llx, requested remap 0x%llx",
			  __func__, mapping_gva,
			  mapped_buffer->ctag_map_win_addr, *compbits_win_gva);
		return -EINVAL;
	}

	*mapping_iova = gk20a_mm_iova_addr(g, mapped_buffer->sgt->sgl, 0);
	*compbits_win_gva = mapped_buffer->ctag_map_win_addr;

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return 0;
}

#if defined(CONFIG_GK20A_VIDMEM)
static int gk20a_gmmu_clear_vidmem_mem(struct gk20a *g, struct nvgpu_mem *mem)
{
	struct gk20a_fence *gk20a_fence_out = NULL;
	struct gk20a_fence *gk20a_last_fence = NULL;
	struct nvgpu_page_alloc *alloc = NULL;
	struct page_alloc_chunk *chunk = NULL;
	int err = 0;

	if (g->mm.vidmem.ce_ctx_id == (u32)~0)
		return -EINVAL;

	alloc = get_vidmem_page_alloc(mem->priv.sgt->sgl);

	nvgpu_list_for_each_entry(chunk, &alloc->alloc_chunks,
				  page_alloc_chunk, list_entry) {
		if (gk20a_last_fence)
			gk20a_fence_put(gk20a_last_fence);

		err = gk20a_ce_execute_ops(g,
			g->mm.vidmem.ce_ctx_id,
			0,
			chunk->base,
			chunk->length,
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
#endif

/*
 * If mem is in VIDMEM, return base address in vidmem
 * else return IOVA address for SYSMEM
 */
u64 nvgpu_mem_get_base_addr(struct gk20a *g, struct nvgpu_mem *mem,
			    u32 flags)
{
	struct nvgpu_page_alloc *alloc;
	u64 addr;

	if (mem->aperture == APERTURE_VIDMEM) {
		alloc = get_vidmem_page_alloc(mem->priv.sgt->sgl);

		/* This API should not be used with > 1 chunks */
		WARN_ON(alloc->nr_chunks != 1);

		addr = alloc->base;
	} else {
		addr = g->ops.mm.get_iova_addr(g, mem->priv.sgt->sgl, flags);
	}

	return addr;
}

#if defined(CONFIG_GK20A_VIDMEM)
static struct nvgpu_mem *get_pending_mem_desc(struct mm_gk20a *mm)
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

static void gk20a_vidmem_clear_mem_worker(struct work_struct *work)
{
	struct mm_gk20a *mm = container_of(work, struct mm_gk20a,
					vidmem.clear_mem_worker);
	struct gk20a *g = mm->g;
	struct nvgpu_mem *mem;

	while ((mem = get_pending_mem_desc(mm)) != NULL) {
		gk20a_gmmu_clear_vidmem_mem(g, mem);
		nvgpu_free(mem->allocator,
			   (u64)get_vidmem_page_alloc(mem->priv.sgt->sgl));
		nvgpu_free_sgtable(g, &mem->priv.sgt);

		WARN_ON(atomic64_sub_return(mem->size,
					&g->mm.vidmem.bytes_pending) < 0);
		mem->size = 0;
		mem->aperture = APERTURE_INVALID;

		nvgpu_kfree(g, mem);
	}
}
#endif

dma_addr_t gk20a_mm_gpuva_to_iova_base(struct vm_gk20a *vm, u64 gpu_vaddr)
{
	struct nvgpu_mapped_buf *buffer;
	dma_addr_t addr = 0;
	struct gk20a *g = gk20a_from_vm(vm);

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	buffer = __nvgpu_vm_find_mapped_buf(vm, gpu_vaddr);
	if (buffer)
		addr = g->ops.mm.get_iova_addr(g, buffer->sgt->sgl,
				buffer->flags);
	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return addr;
}

u64 gk20a_mm_smmu_vaddr_translate(struct gk20a *g, dma_addr_t iova)
{
	/* ensure it is not vidmem allocation */
	WARN_ON(is_vidmem_page_alloc((u64)iova));

	if (device_is_iommuable(dev_from_gk20a(g)) &&
			g->ops.mm.get_physical_addr_bits)
		return iova | 1ULL << g->ops.mm.get_physical_addr_bits(g);

	return iova;
}

u64 gk20a_mm_iova_addr(struct gk20a *g, struct scatterlist *sgl,
		u32 flags)
{
	if (!device_is_iommuable(dev_from_gk20a(g)))
		return sg_phys(sgl);

	if (sg_dma_address(sgl) == 0)
		return sg_phys(sgl);

	if (sg_dma_address(sgl) == DMA_ERROR_CODE)
		return 0;

	return gk20a_mm_smmu_vaddr_translate(g, sg_dma_address(sgl));
}

/* for gk20a the "video memory" apertures here are misnomers. */
static inline u32 big_valid_pde0_bits(struct gk20a *g,
				      struct nvgpu_gmmu_pd *pd, u64 addr)
{
	u32 pde0_bits =
		nvgpu_aperture_mask(g, pd->mem,
		  gmmu_pde_aperture_big_sys_mem_ncoh_f(),
		  gmmu_pde_aperture_big_video_memory_f()) |
		gmmu_pde_address_big_sys_f(
			   (u32)(addr >> gmmu_pde_address_shift_v()));

	return pde0_bits;
}

static inline u32 small_valid_pde1_bits(struct gk20a *g,
					struct nvgpu_gmmu_pd *pd, u64 addr)
{
	u32 pde1_bits =
		nvgpu_aperture_mask(g, pd->mem,
		  gmmu_pde_aperture_small_sys_mem_ncoh_f(),
		  gmmu_pde_aperture_small_video_memory_f()) |
		gmmu_pde_vol_small_true_f() | /* tbd: why? */
		gmmu_pde_address_small_sys_f(
			   (u32)(addr >> gmmu_pde_address_shift_v()));

	return pde1_bits;
}

static void update_gmmu_pde_locked(struct vm_gk20a *vm,
				   const struct gk20a_mmu_level *l,
				   struct nvgpu_gmmu_pd *pd,
				   u32 pd_idx,
				   u64 virt_addr,
				   u64 phys_addr,
				   struct nvgpu_gmmu_attrs *attrs)
{
	struct gk20a *g = gk20a_from_vm(vm);
	bool small_valid, big_valid;
	u32 pd_offset = pd_offset_from_index(l, pd_idx);
	u32 pde_v[2] = {0, 0};

	small_valid = attrs->pgsz == gmmu_page_size_small;
	big_valid   = attrs->pgsz == gmmu_page_size_big;

	pde_v[0] = gmmu_pde_size_full_f();
	pde_v[0] |= big_valid ?
		big_valid_pde0_bits(g, pd, phys_addr) :
		gmmu_pde_aperture_big_invalid_f();

	pde_v[1] |= (small_valid ? small_valid_pde1_bits(g, pd, phys_addr) :
		     (gmmu_pde_aperture_small_invalid_f() |
		      gmmu_pde_vol_small_false_f()))
		|
		(big_valid ? (gmmu_pde_vol_big_true_f()) :
		 gmmu_pde_vol_big_false_f());

	pte_dbg(g, attrs,
		"PDE: i=%-4u size=%-2u offs=%-4u pgsz: %c%c | "
		"GPU %#-12llx  phys %#-12llx "
		"[0x%08x, 0x%08x]",
		pd_idx, l->entry_size, pd_offset,
		small_valid ? 'S' : '-',
		big_valid ?   'B' : '-',
		virt_addr, phys_addr,
		pde_v[1], pde_v[0]);

	pd_write(g, &vm->pdb, pd_offset + 0, pde_v[0]);
	pd_write(g, &vm->pdb, pd_offset + 1, pde_v[1]);
}

static void __update_pte_sparse(u32 *pte_w)
{
	pte_w[0]  = gmmu_pte_valid_false_f();
	pte_w[1] |= gmmu_pte_vol_true_f();
}

static void __update_pte(struct vm_gk20a *vm,
			 u32 *pte_w,
			 u64 phys_addr,
			 struct nvgpu_gmmu_attrs *attrs)
{
	struct gk20a *g = gk20a_from_vm(vm);
	u32 page_size = vm->gmmu_page_sizes[attrs->pgsz];
	u32 pte_valid = attrs->valid ?
		gmmu_pte_valid_true_f() :
		gmmu_pte_valid_false_f();
	u32 phys_shifted = phys_addr >> gmmu_pte_address_shift_v();
	u32 addr = attrs->aperture == APERTURE_SYSMEM ?
		gmmu_pte_address_sys_f(phys_shifted) :
		gmmu_pte_address_vid_f(phys_shifted);
	int ctag_shift = ilog2(g->ops.fb.compression_page_size(g));

	pte_w[0] = pte_valid | addr;

	if (attrs->priv)
		pte_w[0] |= gmmu_pte_privilege_true_f();

	pte_w[1] = __nvgpu_aperture_mask(g, attrs->aperture,
					 gmmu_pte_aperture_sys_mem_ncoh_f(),
					 gmmu_pte_aperture_video_memory_f()) |
		gmmu_pte_kind_f(attrs->kind_v) |
		gmmu_pte_comptagline_f((u32)(attrs->ctag >> ctag_shift));

	if (attrs->ctag && vm->mm->use_full_comp_tag_line &&
	    phys_addr & 0x10000)
		pte_w[1] |= gmmu_pte_comptagline_f(
			1 << (gmmu_pte_comptagline_s() - 1));

	if (attrs->rw_flag == gk20a_mem_flag_read_only) {
		pte_w[0] |= gmmu_pte_read_only_true_f();
		pte_w[1] |= gmmu_pte_write_disable_true_f();
	} else if (attrs->rw_flag == gk20a_mem_flag_write_only) {
		pte_w[1] |= gmmu_pte_read_disable_true_f();
	}

	if (!attrs->cacheable)
		pte_w[1] |= gmmu_pte_vol_true_f();

	if (attrs->ctag)
		attrs->ctag += page_size;
}

static void update_gmmu_pte_locked(struct vm_gk20a *vm,
				   const struct gk20a_mmu_level *l,
				   struct nvgpu_gmmu_pd *pd,
				   u32 pd_idx,
				   u64 virt_addr,
				   u64 phys_addr,
				   struct nvgpu_gmmu_attrs *attrs)
{
	struct gk20a *g = gk20a_from_vm(vm);
	u32 page_size  = vm->gmmu_page_sizes[attrs->pgsz];
	u32 pd_offset = pd_offset_from_index(l, pd_idx);
	u32 pte_w[2] = {0, 0};
	int ctag_shift = ilog2(g->ops.fb.compression_page_size(g));

	if (phys_addr)
		__update_pte(vm, pte_w, phys_addr, attrs);
	else if (attrs->sparse)
		__update_pte_sparse(pte_w);

	pte_dbg(g, attrs,
		"PTE: i=%-4u size=%-2u offs=%-4u | "
		"GPU %#-12llx  phys %#-12llx "
		"pgsz: %3dkb perm=%-2s kind=%#02x APT=%-6s %c%c%c%c%c "
		"ctag=0x%08x "
		"[0x%08x, 0x%08x]",
		pd_idx, l->entry_size, pd_offset,
		virt_addr, phys_addr,
		page_size >> 10,
		nvgpu_gmmu_perm_str(attrs->rw_flag),
		attrs->kind_v,
		nvgpu_aperture_str(attrs->aperture),
		attrs->cacheable ? 'C' : 'v',
		attrs->sparse    ? 'S' : '-',
		attrs->priv      ? 'P' : '-',
		attrs->coherent  ? 'c' : '-',
		attrs->valid     ? 'V' : '-',
		(u32)attrs->ctag >> ctag_shift,
		pte_w[1], pte_w[0]);

	pd_write(g, pd, pd_offset + 0, pte_w[0]);
	pd_write(g, pd, pd_offset + 1, pte_w[1]);
}

/* NOTE! mapped_buffers lock must be held */
void nvgpu_vm_unmap_locked(struct nvgpu_mapped_buf *mapped_buffer,
			   struct vm_gk20a_mapping_batch *batch)
{
	struct vm_gk20a *vm = mapped_buffer->vm;
	struct gk20a *g = vm->mm->g;

	if (mapped_buffer->ctag_map_win_addr) {
		/* unmap compbits */

		g->ops.mm.gmmu_unmap(vm,
				     mapped_buffer->ctag_map_win_addr,
				     mapped_buffer->ctag_map_win_size,
				     0,       /* page size 4k */
				     true,    /* va allocated */
				     gk20a_mem_flag_none,
				     false,   /* not sparse */
				     batch);  /* batch handle */
	}

	g->ops.mm.gmmu_unmap(vm,
		mapped_buffer->addr,
		mapped_buffer->size,
		mapped_buffer->pgsz_idx,
		mapped_buffer->va_allocated,
		gk20a_mem_flag_none,
		mapped_buffer->vm_area ?
		  mapped_buffer->vm_area->sparse : false,
		batch);

	gk20a_mm_unpin(dev_from_vm(vm), mapped_buffer->dmabuf,
		       mapped_buffer->sgt);

	/* remove from mapped buffer tree and remove list, free */
	nvgpu_remove_mapped_buf(vm, mapped_buffer);
	if (!nvgpu_list_empty(&mapped_buffer->buffer_list))
		nvgpu_list_del(&mapped_buffer->buffer_list);

	/* keep track of mapped buffers */
	if (mapped_buffer->user_mapped)
		vm->num_user_mapped_buffers--;

	if (mapped_buffer->own_mem_ref)
		dma_buf_put(mapped_buffer->dmabuf);

	nvgpu_kfree(g, mapped_buffer);

	return;
}

const struct gk20a_mmu_level gk20a_mm_levels_64k[] = {
	{.hi_bit = {NV_GMMU_VA_RANGE-1, NV_GMMU_VA_RANGE-1},
	 .lo_bit = {26, 26},
	 .update_entry = update_gmmu_pde_locked,
	 .entry_size = 8},
	{.hi_bit = {25, 25},
	 .lo_bit = {12, 16},
	 .update_entry = update_gmmu_pte_locked,
	 .entry_size = 8},
	{.update_entry = NULL}
};

const struct gk20a_mmu_level gk20a_mm_levels_128k[] = {
	{.hi_bit = {NV_GMMU_VA_RANGE-1, NV_GMMU_VA_RANGE-1},
	 .lo_bit = {27, 27},
	 .update_entry = update_gmmu_pde_locked,
	 .entry_size = 8},
	{.hi_bit = {26, 26},
	 .lo_bit = {12, 17},
	 .update_entry = update_gmmu_pte_locked,
	 .entry_size = 8},
	{.update_entry = NULL}
};

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

int __gk20a_vm_bind_channel(struct vm_gk20a *vm, struct channel_gk20a *ch)
{
	int err = 0;

	gk20a_dbg_fn("");

	nvgpu_vm_get(vm);
	ch->vm = vm;
	err = channel_gk20a_commit_va(ch);
	if (err)
		ch->vm = NULL;

	nvgpu_log(gk20a_from_vm(vm), gpu_dbg_map, "Binding ch=%d -> VM:%s",
		  ch->chid, vm->name);

	return err;
}

int gk20a_vm_bind_channel(struct gk20a_as_share *as_share,
			  struct channel_gk20a *ch)
{
	return __gk20a_vm_bind_channel(as_share->vm, ch);
}

int gk20a_dmabuf_alloc_drvdata(struct dma_buf *dmabuf, struct device *dev)
{
	struct gk20a *g = gk20a_get_platform(dev)->g;
	struct gk20a_dmabuf_priv *priv;
	static u64 priv_count = 0;

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (likely(priv))
		return 0;

	nvgpu_mutex_acquire(&g->mm.priv_lock);
	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (priv)
		goto priv_exist_or_err;

	priv = nvgpu_kzalloc(g, sizeof(*priv));
	if (!priv) {
		priv = ERR_PTR(-ENOMEM);
		goto priv_exist_or_err;
	}

	nvgpu_mutex_init(&priv->lock);
	nvgpu_init_list_node(&priv->states);
	priv->buffer_id = ++priv_count;
	priv->g = g;
	dma_buf_set_drvdata(dmabuf, dev, priv, gk20a_mm_delete_priv);

priv_exist_or_err:
	nvgpu_mutex_release(&g->mm.priv_lock);
	if (IS_ERR(priv))
		return -ENOMEM;

	return 0;
}

int gk20a_dmabuf_get_state(struct dma_buf *dmabuf, struct gk20a *g,
			   u64 offset, struct gk20a_buffer_state **state)
{
	int err = 0;
	struct gk20a_dmabuf_priv *priv;
	struct gk20a_buffer_state *s;
	struct device *dev = dev_from_gk20a(g);

	if (WARN_ON(offset >= (u64)dmabuf->size))
		return -EINVAL;

	err = gk20a_dmabuf_alloc_drvdata(dmabuf, dev);
	if (err)
		return err;

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (WARN_ON(!priv))
		return -ENOSYS;

	nvgpu_mutex_acquire(&priv->lock);

	nvgpu_list_for_each_entry(s, &priv->states, gk20a_buffer_state, list)
		if (s->offset == offset)
			goto out;

	/* State not found, create state. */
	s = nvgpu_kzalloc(g, sizeof(*s));
	if (!s) {
		err = -ENOMEM;
		goto out;
	}

	s->offset = offset;
	nvgpu_init_list_node(&s->list);
	nvgpu_mutex_init(&s->lock);
	nvgpu_list_add_tail(&s->list, &priv->states);

out:
	nvgpu_mutex_release(&priv->lock);
	if (!err)
		*state = s;
	return err;


}

int nvgpu_vm_map_buffer(struct vm_gk20a *vm,
			int dmabuf_fd,
			u64 *offset_align,
			u32 flags, /*NVGPU_AS_MAP_BUFFER_FLAGS_*/
			int kind,
			u64 buffer_offset,
			u64 mapping_size,
			struct vm_gk20a_mapping_batch *batch)
{
	int err = 0;
	struct dma_buf *dmabuf;
	u64 ret_va;

	gk20a_dbg_fn("");

	/* get ref to the mem handle (released on unmap_locked) */
	dmabuf = dma_buf_get(dmabuf_fd);
	if (IS_ERR(dmabuf)) {
		nvgpu_warn(gk20a_from_vm(vm), "%s: fd %d is not a dmabuf",
			 __func__, dmabuf_fd);
		return PTR_ERR(dmabuf);
	}

	/* verify that we're not overflowing the buffer, i.e.
	 * (buffer_offset + mapping_size)> dmabuf->size.
	 *
	 * Since buffer_offset + mapping_size could overflow, first check
	 * that mapping size < dmabuf_size, at which point we can subtract
	 * mapping_size from both sides for the final comparison.
	 */
	if ((mapping_size > dmabuf->size) ||
			(buffer_offset > (dmabuf->size - mapping_size))) {
		nvgpu_err(gk20a_from_vm(vm),
			"buf size %llx < (offset(%llx) + map_size(%llx))\n",
			(u64)dmabuf->size, buffer_offset, mapping_size);
		return -EINVAL;
	}

	err = gk20a_dmabuf_alloc_drvdata(dmabuf, dev_from_vm(vm));
	if (err) {
		dma_buf_put(dmabuf);
		return err;
	}

	ret_va = nvgpu_vm_map(vm, dmabuf, *offset_align,
			flags, kind, true,
			gk20a_mem_flag_none,
			buffer_offset,
			mapping_size,
			batch);

	*offset_align = ret_va;
	if (!ret_va) {
		dma_buf_put(dmabuf);
		err = -EINVAL;
	}

	return err;
}

int nvgpu_vm_unmap_buffer(struct vm_gk20a *vm, u64 offset,
			  struct vm_gk20a_mapping_batch *batch)
{
	gk20a_dbg_fn("");

	nvgpu_vm_unmap_user(vm, offset, batch);
	return 0;
}

int gk20a_alloc_inst_block(struct gk20a *g, struct nvgpu_mem *inst_block)
{
	int err;

	gk20a_dbg_fn("");

	err = nvgpu_dma_alloc(g, ram_in_alloc_size_v(), inst_block);
	if (err) {
		nvgpu_err(g, "%s: memory allocation failed", __func__);
		return err;
	}

	gk20a_dbg_fn("done");
	return 0;
}

void gk20a_free_inst_block(struct gk20a *g, struct nvgpu_mem *inst_block)
{
	if (inst_block->size)
		nvgpu_dma_free(g, inst_block);
}

u64 gk20a_mm_inst_block_addr(struct gk20a *g, struct nvgpu_mem *inst_block)
{
	u64 addr;
	if (g->mm.has_physical_mode)
		addr = gk20a_mem_phys(inst_block);
	else
		addr = nvgpu_mem_get_base_addr(g, inst_block, 0);

	return addr;
}

static int gk20a_init_bar1_vm(struct mm_gk20a *mm)
{
	int err;
	struct gk20a *g = gk20a_from_mm(mm);
	struct nvgpu_mem *inst_block = &mm->bar1.inst_block;
	u32 big_page_size = g->ops.mm.get_default_big_page_size();

	mm->bar1.aperture_size = bar1_aperture_size_mb_gk20a() << 20;
	gk20a_dbg_info("bar1 vm size = 0x%x", mm->bar1.aperture_size);
	mm->bar1.vm = nvgpu_vm_init(g,
				    big_page_size,
				    SZ_4K,
				    mm->bar1.aperture_size - SZ_4K,
				    mm->bar1.aperture_size,
				    true, false,
				    "bar1");
	if (!mm->bar1.vm)
		return -ENOMEM;

	err = gk20a_alloc_inst_block(g, inst_block);
	if (err)
		goto clean_up_vm;
	g->ops.mm.init_inst_block(inst_block, mm->bar1.vm, big_page_size);

	return 0;

clean_up_vm:
	nvgpu_vm_put(mm->bar1.vm);
	return err;
}

/* pmu vm, share channel_vm interfaces */
static int gk20a_init_system_vm(struct mm_gk20a *mm)
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
	gk20a_dbg_info("pmu vm size = 0x%x", mm->pmu.aperture_size);

	mm->pmu.vm = nvgpu_vm_init(g, big_page_size,
				   low_hole,
				   aperture_size - low_hole,
				   aperture_size,
				   true,
				   false,
				   "system");
	if (!mm->pmu.vm)
		return -ENOMEM;

	err = gk20a_alloc_inst_block(g, inst_block);
	if (err)
		goto clean_up_vm;
	g->ops.mm.init_inst_block(inst_block, mm->pmu.vm, big_page_size);

	return 0;

clean_up_vm:
	nvgpu_vm_put(mm->pmu.vm);
	return err;
}

static int gk20a_init_hwpm(struct mm_gk20a *mm)
{
	int err;
	struct gk20a *g = gk20a_from_mm(mm);
	struct nvgpu_mem *inst_block = &mm->hwpm.inst_block;

	err = gk20a_alloc_inst_block(g, inst_block);
	if (err)
		return err;
	g->ops.mm.init_inst_block(inst_block, mm->pmu.vm, 0);

	return 0;
}

static int gk20a_init_cde_vm(struct mm_gk20a *mm)
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

static int gk20a_init_ce_vm(struct mm_gk20a *mm)
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

void gk20a_mm_init_pdb(struct gk20a *g, struct nvgpu_mem *inst_block,
		struct vm_gk20a *vm)
{
	u64 pdb_addr = nvgpu_mem_get_base_addr(g, vm->pdb.mem, 0);
	u32 pdb_addr_lo = u64_lo32(pdb_addr >> ram_in_base_shift_v());
	u32 pdb_addr_hi = u64_hi32(pdb_addr);

	gk20a_dbg_info("pde pa=0x%llx", pdb_addr);

	nvgpu_mem_wr32(g, inst_block, ram_in_page_dir_base_lo_w(),
		nvgpu_aperture_mask(g, vm->pdb.mem,
		  ram_in_page_dir_base_target_sys_mem_ncoh_f(),
		  ram_in_page_dir_base_target_vid_mem_f()) |
		ram_in_page_dir_base_vol_true_f() |
		ram_in_page_dir_base_lo_f(pdb_addr_lo));

	nvgpu_mem_wr32(g, inst_block, ram_in_page_dir_base_hi_w(),
		ram_in_page_dir_base_hi_f(pdb_addr_hi));
}

void gk20a_init_inst_block(struct nvgpu_mem *inst_block, struct vm_gk20a *vm,
		u32 big_page_size)
{
	struct gk20a *g = gk20a_from_vm(vm);

	gk20a_dbg_info("inst block phys = 0x%llx, kv = 0x%p",
		gk20a_mm_inst_block_addr(g, inst_block), inst_block->cpu_va);

	g->ops.mm.init_pdb(g, inst_block, vm);

	nvgpu_mem_wr32(g, inst_block, ram_in_adr_limit_lo_w(),
		u64_lo32(vm->va_limit - 1) & ~0xfff);

	nvgpu_mem_wr32(g, inst_block, ram_in_adr_limit_hi_w(),
		ram_in_adr_limit_hi_f(u64_hi32(vm->va_limit - 1)));

	if (big_page_size && g->ops.mm.set_big_page_size)
		g->ops.mm.set_big_page_size(g, inst_block, big_page_size);
}

int gk20a_mm_fb_flush(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	u32 data;
	int ret = 0;
	struct nvgpu_timeout timeout;

	gk20a_dbg_fn("");

	gk20a_busy_noresume(g);
	if (!g->power_on) {
		gk20a_idle_nosuspend(g);
		return 0;
	}

	nvgpu_timeout_init(g, &timeout, 100, NVGPU_TIMER_RETRY_TIMER);

	nvgpu_mutex_acquire(&mm->l2_op_lock);

	/* Make sure all previous writes are committed to the L2. There's no
	   guarantee that writes are to DRAM. This will be a sysmembar internal
	   to the L2. */

	trace_gk20a_mm_fb_flush(g->name);

	gk20a_writel(g, flush_fb_flush_r(),
		flush_fb_flush_pending_busy_f());

	do {
		data = gk20a_readl(g, flush_fb_flush_r());

		if (flush_fb_flush_outstanding_v(data) ==
			flush_fb_flush_outstanding_true_v() ||
		    flush_fb_flush_pending_v(data) ==
			flush_fb_flush_pending_busy_v()) {
				gk20a_dbg_info("fb_flush 0x%x", data);
				nvgpu_udelay(5);
		} else
			break;
	} while (!nvgpu_timeout_expired(&timeout));

	if (nvgpu_timeout_peek_expired(&timeout)) {
		if (g->ops.fb.dump_vpr_wpr_info)
			g->ops.fb.dump_vpr_wpr_info(g);
		ret = -EBUSY;
	}

	trace_gk20a_mm_fb_flush_done(g->name);

	nvgpu_mutex_release(&mm->l2_op_lock);

	gk20a_idle_nosuspend(g);

	return ret;
}

static void gk20a_mm_l2_invalidate_locked(struct gk20a *g)
{
	u32 data;
	struct nvgpu_timeout timeout;

	trace_gk20a_mm_l2_invalidate(g->name);

	nvgpu_timeout_init(g, &timeout, 200, NVGPU_TIMER_RETRY_TIMER);

	/* Invalidate any clean lines from the L2 so subsequent reads go to
	   DRAM. Dirty lines are not affected by this operation. */
	gk20a_writel(g, flush_l2_system_invalidate_r(),
		flush_l2_system_invalidate_pending_busy_f());

	do {
		data = gk20a_readl(g, flush_l2_system_invalidate_r());

		if (flush_l2_system_invalidate_outstanding_v(data) ==
			flush_l2_system_invalidate_outstanding_true_v() ||
		    flush_l2_system_invalidate_pending_v(data) ==
			flush_l2_system_invalidate_pending_busy_v()) {
				gk20a_dbg_info("l2_system_invalidate 0x%x",
						data);
				nvgpu_udelay(5);
		} else
			break;
	} while (!nvgpu_timeout_expired(&timeout));

	if (nvgpu_timeout_peek_expired(&timeout))
		nvgpu_warn(g, "l2_system_invalidate too many retries");

	trace_gk20a_mm_l2_invalidate_done(g->name);
}

void gk20a_mm_l2_invalidate(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	gk20a_busy_noresume(g);
	if (g->power_on) {
		nvgpu_mutex_acquire(&mm->l2_op_lock);
		gk20a_mm_l2_invalidate_locked(g);
		nvgpu_mutex_release(&mm->l2_op_lock);
	}
	gk20a_idle_nosuspend(g);
}

void gk20a_mm_l2_flush(struct gk20a *g, bool invalidate)
{
	struct mm_gk20a *mm = &g->mm;
	u32 data;
	struct nvgpu_timeout timeout;

	gk20a_dbg_fn("");

	gk20a_busy_noresume(g);
	if (!g->power_on)
		goto hw_was_off;

	nvgpu_timeout_init(g, &timeout, 2000, NVGPU_TIMER_RETRY_TIMER);

	nvgpu_mutex_acquire(&mm->l2_op_lock);

	trace_gk20a_mm_l2_flush(g->name);

	/* Flush all dirty lines from the L2 to DRAM. Lines are left in the L2
	   as clean, so subsequent reads might hit in the L2. */
	gk20a_writel(g, flush_l2_flush_dirty_r(),
		flush_l2_flush_dirty_pending_busy_f());

	do {
		data = gk20a_readl(g, flush_l2_flush_dirty_r());

		if (flush_l2_flush_dirty_outstanding_v(data) ==
			flush_l2_flush_dirty_outstanding_true_v() ||
		    flush_l2_flush_dirty_pending_v(data) ==
			flush_l2_flush_dirty_pending_busy_v()) {
				gk20a_dbg_info("l2_flush_dirty 0x%x", data);
				nvgpu_udelay(5);
		} else
			break;
	} while (!nvgpu_timeout_expired_msg(&timeout,
					 "l2_flush_dirty too many retries"));

	trace_gk20a_mm_l2_flush_done(g->name);

	if (invalidate)
		gk20a_mm_l2_invalidate_locked(g);

	nvgpu_mutex_release(&mm->l2_op_lock);

hw_was_off:
	gk20a_idle_nosuspend(g);
}

void gk20a_mm_cbc_clean(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	u32 data;
	struct nvgpu_timeout timeout;

	gk20a_dbg_fn("");

	gk20a_busy_noresume(g);
	if (!g->power_on)
		goto hw_was_off;

	nvgpu_timeout_init(g, &timeout, 200, NVGPU_TIMER_RETRY_TIMER);

	nvgpu_mutex_acquire(&mm->l2_op_lock);

	/* Flush all dirty lines from the CBC to L2 */
	gk20a_writel(g, flush_l2_clean_comptags_r(),
		flush_l2_clean_comptags_pending_busy_f());

	do {
		data = gk20a_readl(g, flush_l2_clean_comptags_r());

		if (flush_l2_clean_comptags_outstanding_v(data) ==
			flush_l2_clean_comptags_outstanding_true_v() ||
		    flush_l2_clean_comptags_pending_v(data) ==
			flush_l2_clean_comptags_pending_busy_v()) {
				gk20a_dbg_info("l2_clean_comptags 0x%x", data);
				nvgpu_udelay(5);
		} else
			break;
	} while (!nvgpu_timeout_expired_msg(&timeout,
					 "l2_clean_comptags too many retries"));

	nvgpu_mutex_release(&mm->l2_op_lock);

hw_was_off:
	gk20a_idle_nosuspend(g);
}

int nvgpu_vm_find_buf(struct vm_gk20a *vm, u64 gpu_va,
		      struct dma_buf **dmabuf,
		      u64 *offset)
{
	struct nvgpu_mapped_buf *mapped_buffer;

	gk20a_dbg_fn("gpu_va=0x%llx", gpu_va);

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	mapped_buffer = __nvgpu_vm_find_mapped_buf_range(vm, gpu_va);
	if (!mapped_buffer) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		return -EINVAL;
	}

	*dmabuf = mapped_buffer->dmabuf;
	*offset = gpu_va - mapped_buffer->addr;

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return 0;
}

int gk20a_mm_suspend(struct gk20a *g)
{
	gk20a_dbg_fn("");

#if defined(CONFIG_GK20A_VIDMEM)
	cancel_work_sync(&g->mm.vidmem.clear_mem_worker);
#endif

	g->ops.mm.cbc_clean(g);
	g->ops.mm.l2_flush(g, false);

	gk20a_dbg_fn("done");
	return 0;
}

u32 gk20a_mm_get_physical_addr_bits(struct gk20a *g)
{
	return 34;
}

const struct gk20a_mmu_level *gk20a_mm_get_mmu_levels(struct gk20a *g,
						      u32 big_page_size)
{
	return (big_page_size == SZ_64K) ?
		 gk20a_mm_levels_64k : gk20a_mm_levels_128k;
}

int gk20a_mm_get_buffer_info(struct device *dev, int dmabuf_fd,
			     u64 *buffer_id, u64 *buffer_len)
{
	struct dma_buf *dmabuf;
	struct gk20a_dmabuf_priv *priv;
	int err = 0;

	dmabuf = dma_buf_get(dmabuf_fd);
	if (IS_ERR(dmabuf)) {
		dev_warn(dev, "%s: fd %d is not a dmabuf", __func__, dmabuf_fd);
		return PTR_ERR(dmabuf);
	}

	err = gk20a_dmabuf_alloc_drvdata(dmabuf, dev);
	if (err) {
		dev_warn(dev, "Failed to allocate dmabuf drvdata (err = %d)",
			 err);
		goto clean_up;
	}

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (likely(priv)) {
		*buffer_id = priv->buffer_id;
		*buffer_len = dmabuf->size;
	}

clean_up:
	dma_buf_put(dmabuf);
	return err;
}
