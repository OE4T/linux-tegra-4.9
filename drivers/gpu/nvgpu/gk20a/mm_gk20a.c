/*
 * GK20A memory management
 *
 * Copyright (c) 2011-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/nvhost.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/nvmap.h>
#include <linux/tegra-soc.h>
#include <linux/vmalloc.h>
#include <linux/dma-buf.h>
#include <linux/lcm.h>
#include <uapi/linux/nvgpu.h>
#include <trace/events/gk20a.h>

#include "gk20a.h"
#include "mm_gk20a.h"
#include "fence_gk20a.h"
#include "hw_gmmu_gk20a.h"
#include "hw_fb_gk20a.h"
#include "hw_bus_gk20a.h"
#include "hw_ram_gk20a.h"
#include "hw_mc_gk20a.h"
#include "hw_flush_gk20a.h"
#include "hw_ltc_gk20a.h"

#include "kind_gk20a.h"
#include "semaphore_gk20a.h"

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

static inline int vm_aspace_id(struct vm_gk20a *vm)
{
	/* -1 is bar1 or pmu, etc. */
	return vm->as_share ? vm->as_share->id : -1;
}
static inline u32 hi32(u64 f)
{
	return (u32)(f >> 32);
}
static inline u32 lo32(u64 f)
{
	return (u32)(f & 0xffffffff);
}

static struct mapped_buffer_node *find_mapped_buffer_locked(
					struct rb_root *root, u64 addr);
static struct mapped_buffer_node *find_mapped_buffer_reverse_locked(
				struct rb_root *root, struct dma_buf *dmabuf,
				u32 kind);
static int update_gmmu_ptes_locked(struct vm_gk20a *vm,
				   enum gmmu_pgsz_gk20a pgsz_idx,
				   struct sg_table *sgt, u64 buffer_offset,
				   u64 first_vaddr, u64 last_vaddr,
				   u8 kind_v, u32 ctag_offset, bool cacheable,
				   bool umapped_pte, int rw_flag,
				   bool sparse,
				   bool priv);
static int __must_check gk20a_init_system_vm(struct mm_gk20a *mm);
static int __must_check gk20a_init_bar1_vm(struct mm_gk20a *mm);
static int __must_check gk20a_init_hwpm(struct mm_gk20a *mm);
static int __must_check gk20a_init_cde_vm(struct mm_gk20a *mm);


struct gk20a_dmabuf_priv {
	struct mutex lock;

	struct gk20a_comptag_allocator *comptag_allocator;
	struct gk20a_comptags comptags;

	struct dma_buf_attachment *attach;
	struct sg_table *sgt;

	int pin_count;

	struct list_head states;

	u64 buffer_id;
};

static void gk20a_vm_remove_support_nofree(struct vm_gk20a *vm);

static int gk20a_comptaglines_alloc(struct gk20a_comptag_allocator *allocator,
		u32 *offset, u32 len)
{
	unsigned long addr;
	int err = 0;

	mutex_lock(&allocator->lock);
	addr = bitmap_find_next_zero_area(allocator->bitmap, allocator->size,
			0, len, 0);
	if (addr < allocator->size) {
		/* number zero is reserved; bitmap base is 1 */
		*offset = 1 + addr;
		bitmap_set(allocator->bitmap, addr, len);
	} else {
		err = -ENOMEM;
	}
	mutex_unlock(&allocator->lock);

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

	mutex_lock(&allocator->lock);
	bitmap_clear(allocator->bitmap, addr, len);
	mutex_unlock(&allocator->lock);
}

static void gk20a_mm_delete_priv(void *_priv)
{
	struct gk20a_buffer_state *s, *s_tmp;
	struct gk20a_dmabuf_priv *priv = _priv;
	if (!priv)
		return;

	if (priv->comptags.lines) {
		BUG_ON(!priv->comptag_allocator);
		gk20a_comptaglines_free(priv->comptag_allocator,
				priv->comptags.offset,
				priv->comptags.allocated_lines);
	}

	/* Free buffer states */
	list_for_each_entry_safe(s, s_tmp, &priv->states, list) {
		gk20a_fence_put(s->fence);
		list_del(&s->list);
		kfree(s);
	}

	kfree(priv);
}

struct sg_table *gk20a_mm_pin(struct device *dev, struct dma_buf *dmabuf)
{
	struct gk20a_dmabuf_priv *priv;

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (WARN_ON(!priv))
		return ERR_PTR(-EINVAL);

	mutex_lock(&priv->lock);

	if (priv->pin_count == 0) {
		priv->attach = dma_buf_attach(dmabuf, dev);
		if (IS_ERR(priv->attach)) {
			mutex_unlock(&priv->lock);
			return (struct sg_table *)priv->attach;
		}

		priv->sgt = dma_buf_map_attachment(priv->attach,
						   DMA_BIDIRECTIONAL);
		if (IS_ERR(priv->sgt)) {
			dma_buf_detach(dmabuf, priv->attach);
			mutex_unlock(&priv->lock);
			return priv->sgt;
		}
	}

	priv->pin_count++;
	mutex_unlock(&priv->lock);
	return priv->sgt;
}

void gk20a_mm_unpin(struct device *dev, struct dma_buf *dmabuf,
		    struct sg_table *sgt)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(dmabuf, dev);
	dma_addr_t dma_addr;

	if (IS_ERR(priv) || !priv)
		return;

	mutex_lock(&priv->lock);
	WARN_ON(priv->sgt != sgt);
	priv->pin_count--;
	WARN_ON(priv->pin_count < 0);
	dma_addr = sg_dma_address(priv->sgt->sgl);
	if (priv->pin_count == 0) {
		dma_buf_unmap_attachment(priv->attach, priv->sgt,
					 DMA_BIDIRECTIONAL);
		dma_buf_detach(dmabuf, priv->attach);
	}
	mutex_unlock(&priv->lock);
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

static int gk20a_alloc_comptags(struct gk20a *g,
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

void gk20a_remove_vm(struct vm_gk20a *vm, struct mem_desc *inst_block)
{
	struct gk20a *g = vm->mm->g;

	gk20a_dbg_fn("");

	gk20a_free_inst_block(g, inst_block);
	gk20a_vm_remove_support_nofree(vm);
}

static void gk20a_remove_mm_support(struct mm_gk20a *mm)
{
	struct gk20a *g = gk20a_from_mm(mm);

	if (g->ops.mm.remove_bar2_vm)
		g->ops.mm.remove_bar2_vm(g);
	gk20a_remove_vm(&mm->bar1.vm, &mm->bar1.inst_block);
	gk20a_remove_vm(&mm->pmu.vm, &mm->pmu.inst_block);
	gk20a_free_inst_block(gk20a_from_mm(mm), &mm->hwpm.inst_block);
	gk20a_vm_remove_support_nofree(&mm->cde.vm);
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
	mutex_init(&mm->l2_op_lock);

	/*TBD: make channel vm size configurable */
	mm->channel.user_size = NV_MM_DEFAULT_USER_SIZE;
	mm->channel.kernel_size = NV_MM_DEFAULT_KERNEL_SIZE;

	gk20a_dbg_info("channel vm size: user %dMB  kernel %dMB",
		       (int)(mm->channel.user_size >> 20),
		       (int)(mm->channel.kernel_size >> 20));

	err = gk20a_init_bar1_vm(mm);
	if (err)
		return err;

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

	/* set vm_alloc_share op here as gk20a_as_alloc_share needs it */
	g->ops.mm.vm_alloc_share = gk20a_vm_alloc_share;
	mm->remove_support = gk20a_remove_mm_support;
	mm->sw_ready = true;

	gk20a_dbg_fn("done");
	return 0;
}

/* make sure gk20a_init_mm_support is called before */
int gk20a_init_mm_setup_hw(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct mem_desc *inst_block = &mm->bar1.inst_block;
	phys_addr_t inst_pa = gk20a_mem_phys(inst_block);
	int err;

	gk20a_dbg_fn("");

	g->ops.fb.set_mmu_page_size(g);
	if (g->ops.fb.set_use_full_comp_tag_line)
		mm->use_full_comp_tag_line =
			g->ops.fb.set_use_full_comp_tag_line(g);


	inst_pa = (u32)(inst_pa >> bar1_instance_block_shift_gk20a());
	gk20a_dbg_info("bar1 inst block ptr: 0x%08x",  (u32)inst_pa);

	gk20a_writel(g, bus_bar1_block_r(),
		     bus_bar1_block_target_vid_mem_f() |
		     bus_bar1_block_mode_virtual_f() |
		     bus_bar1_block_ptr_f(inst_pa));

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

static int alloc_gmmu_phys_pages(struct vm_gk20a *vm, u32 order,
				 struct gk20a_mm_entry *entry)
{
	u32 num_pages = 1 << order;
	u32 len = num_pages * PAGE_SIZE;
	int err;
	struct page *pages;

	gk20a_dbg_fn("");

	pages = alloc_pages(GFP_KERNEL, order);
	if (!pages) {
		gk20a_dbg(gpu_dbg_pte, "alloc_pages failed\n");
		goto err_out;
	}
	entry->sgt = kzalloc(sizeof(*entry->sgt), GFP_KERNEL);
	if (!entry->sgt) {
		gk20a_dbg(gpu_dbg_pte, "cannot allocate sg table");
		goto err_alloced;
	}
	err = sg_alloc_table(entry->sgt, 1, GFP_KERNEL);
	if (err) {
		gk20a_dbg(gpu_dbg_pte, "sg_alloc_table failed\n");
		goto err_sg_table;
	}
	sg_set_page(entry->sgt->sgl, pages, len, 0);
	entry->cpu_va = page_address(pages);
	memset(entry->cpu_va, 0, len);
	entry->size = len;
	FLUSH_CPU_DCACHE(entry->cpu_va, sg_phys(entry->sgt->sgl), len);

	return 0;

err_sg_table:
	kfree(entry->sgt);
err_alloced:
	__free_pages(pages, order);
err_out:
	return -ENOMEM;
}

static void free_gmmu_phys_pages(struct vm_gk20a *vm,
			    struct gk20a_mm_entry *entry)
{
	gk20a_dbg_fn("");
	free_pages((unsigned long)entry->cpu_va, get_order(entry->size));
	entry->cpu_va = NULL;

	sg_free_table(entry->sgt);
	kfree(entry->sgt);
	entry->sgt = NULL;
}

static int map_gmmu_phys_pages(struct gk20a_mm_entry *entry)
{
	FLUSH_CPU_DCACHE(entry->cpu_va,
			 sg_phys(entry->sgt->sgl),
			 entry->sgt->sgl->length);
	return 0;
}

static void unmap_gmmu_phys_pages(struct gk20a_mm_entry *entry)
{
	FLUSH_CPU_DCACHE(entry->cpu_va,
			 sg_phys(entry->sgt->sgl),
			 entry->sgt->sgl->length);
}

static int alloc_gmmu_pages(struct vm_gk20a *vm, u32 order,
			    struct gk20a_mm_entry *entry)
{
	struct device *d = dev_from_vm(vm);
	u32 num_pages = 1 << order;
	u32 len = num_pages * PAGE_SIZE;
	dma_addr_t iova;
	DEFINE_DMA_ATTRS(attrs);
	void *cpuva;
	int err = 0;

	gk20a_dbg_fn("");

	if (tegra_platform_is_linsim())
		return alloc_gmmu_phys_pages(vm, order, entry);

	entry->size = len;

	/*
	 * On arm32 we're limited by vmalloc space, so we do not map pages by
	 * default.
	 */
	if (IS_ENABLED(CONFIG_ARM64)) {
		cpuva = dma_zalloc_coherent(d, len, &iova, GFP_KERNEL);
		if (!cpuva) {
			gk20a_err(d, "memory allocation failed\n");
			goto err_out;
		}

		err = gk20a_get_sgtable(d, &entry->sgt, cpuva, iova, len);
		if (err) {
			gk20a_err(d, "sgt allocation failed\n");
			goto err_free;
		}

		entry->cpu_va = cpuva;
	} else {
		struct page **pages;

		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
		pages = dma_alloc_attrs(d, len, &iova, GFP_KERNEL, &attrs);
		if (!pages) {
			gk20a_err(d, "memory allocation failed\n");
			goto err_out;
		}

		err = gk20a_get_sgtable_from_pages(d, &entry->sgt, pages,
					iova, len);
		if (err) {
			gk20a_err(d, "sgt allocation failed\n");
			goto err_free;
		}

		entry->pages = pages;
	}

	return 0;

err_free:
	if (IS_ENABLED(CONFIG_ARM64)) {
		dma_free_coherent(d, len, entry->cpu_va, iova);
		cpuva = NULL;
	} else {
		dma_free_attrs(d, len, entry->pages, iova, &attrs);
		entry->pages = NULL;
	}
	iova = 0;
err_out:
	return -ENOMEM;
}

void free_gmmu_pages(struct vm_gk20a *vm,
		     struct gk20a_mm_entry *entry)
{
	struct device *d = dev_from_vm(vm);
	u64 iova;
	DEFINE_DMA_ATTRS(attrs);

	gk20a_dbg_fn("");

	if (!entry->sgt)
		return;

	if (tegra_platform_is_linsim()) {
		free_gmmu_phys_pages(vm, entry);
		return;
	}

	iova = sg_dma_address(entry->sgt->sgl);

	gk20a_free_sgtable(&entry->sgt);

	/*
	 * On arm32 we're limited by vmalloc space, so we do not map pages by
	 * default.
	 */
	if (IS_ENABLED(CONFIG_ARM64)) {
		dma_free_coherent(d, entry->size, entry->cpu_va, iova);
		entry->cpu_va = NULL;
	} else {
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
		dma_free_attrs(d, entry->size, entry->pages, iova, &attrs);
		entry->pages = NULL;
	}
	entry->size = 0;
	entry->sgt = NULL;
}

int map_gmmu_pages(struct gk20a_mm_entry *entry)
{
	int count = PAGE_ALIGN(entry->size) >> PAGE_SHIFT;
	struct page **pages;
	gk20a_dbg_fn("");

	if (tegra_platform_is_linsim())
		return map_gmmu_phys_pages(entry);

	if (IS_ENABLED(CONFIG_ARM64)) {
		FLUSH_CPU_DCACHE(entry->cpu_va,
				 sg_phys(entry->sgt->sgl),
				 entry->size);
	} else {
		pages = entry->pages;
		entry->cpu_va = vmap(pages, count, 0,
				     pgprot_writecombine(PAGE_KERNEL));
		if (!entry->cpu_va)
			return -ENOMEM;
	}

	return 0;
}

void unmap_gmmu_pages(struct gk20a_mm_entry *entry)
{
	gk20a_dbg_fn("");

	if (tegra_platform_is_linsim()) {
		unmap_gmmu_phys_pages(entry);
		return;
	}

	if (IS_ENABLED(CONFIG_ARM64)) {
		FLUSH_CPU_DCACHE(entry->cpu_va,
				 sg_phys(entry->sgt->sgl),
				 entry->size);
	} else {
		vunmap(entry->cpu_va);
		entry->cpu_va = NULL;
	}
}

/* allocate a phys contig region big enough for a full
 * sized gmmu page table for the given gmmu_page_size.
 * the whole range is zeroed so it's "invalid"/will fault
 */

static int gk20a_zalloc_gmmu_page_table(struct vm_gk20a *vm,
				 enum gmmu_pgsz_gk20a pgsz_idx,
				 const struct gk20a_mmu_level *l,
				 struct gk20a_mm_entry *entry)
{
	int err;
	int order;
	struct gk20a *g = gk20a_from_vm(vm);

	gk20a_dbg_fn("");

	/* allocate enough pages for the table */
	order = l->hi_bit[pgsz_idx] - l->lo_bit[pgsz_idx] + 1;
	order += ilog2(l->entry_size);
	order -= PAGE_SHIFT;
	order = max(0, order);

	err = alloc_gmmu_pages(vm, order, entry);
	gk20a_dbg(gpu_dbg_pte, "entry = 0x%p, addr=%08llx, size %d",
		  entry, g->ops.mm.get_iova_addr(g, entry->sgt->sgl, 0),
		  order);
	if (err)
		return err;
	entry->pgsz = pgsz_idx;

	return err;
}

int gk20a_mm_pde_coverage_bit_count(struct vm_gk20a *vm)
{
	return vm->mmu_levels[0].lo_bit[0];
}

/* given address range (inclusive) determine the pdes crossed */
void pde_range_from_vaddr_range(struct vm_gk20a *vm,
					      u64 addr_lo, u64 addr_hi,
					      u32 *pde_lo, u32 *pde_hi)
{
	int pde_shift = gk20a_mm_pde_coverage_bit_count(vm);

	*pde_lo = (u32)(addr_lo >> pde_shift);
	*pde_hi = (u32)(addr_hi >> pde_shift);
	gk20a_dbg(gpu_dbg_pte, "addr_lo=0x%llx addr_hi=0x%llx pde_ss=%d",
		   addr_lo, addr_hi, pde_shift);
	gk20a_dbg(gpu_dbg_pte, "pde_lo=%d pde_hi=%d",
		   *pde_lo, *pde_hi);
}

u32 *pde_from_index(struct vm_gk20a *vm, u32 i)
{
	return (u32 *) (((u8 *)vm->pdb.cpu_va) + i*gmmu_pde__size_v());
}

u32 pte_index_from_vaddr(struct vm_gk20a *vm,
				       u64 addr, enum gmmu_pgsz_gk20a pgsz_idx)
{
	u32 ret;
	/* mask off pde part */
	addr = addr & ((1ULL << gk20a_mm_pde_coverage_bit_count(vm)) - 1ULL);

	/* shift over to get pte index. note assumption that pte index
	 * doesn't leak over into the high 32b */
	ret = (u32)(addr >> ilog2(vm->gmmu_page_sizes[pgsz_idx]));

	gk20a_dbg(gpu_dbg_pte, "addr=0x%llx pte_i=0x%x", addr, ret);
	return ret;
}

static struct vm_reserved_va_node *addr_to_reservation(struct vm_gk20a *vm,
						       u64 addr)
{
	struct vm_reserved_va_node *va_node;
	list_for_each_entry(va_node, &vm->reserved_va_list, reserved_va_list)
		if (addr >= va_node->vaddr_start &&
		    addr < (u64)va_node->vaddr_start + (u64)va_node->size)
			return va_node;

	return NULL;
}

int gk20a_vm_get_buffers(struct vm_gk20a *vm,
			 struct mapped_buffer_node ***mapped_buffers,
			 int *num_buffers)
{
	struct mapped_buffer_node *mapped_buffer;
	struct mapped_buffer_node **buffer_list;
	struct rb_node *node;
	int i = 0;

	if (vm->userspace_managed) {
		*mapped_buffers = NULL;
		*num_buffers = 0;
		return 0;
	}

	mutex_lock(&vm->update_gmmu_lock);

	buffer_list = nvgpu_alloc(sizeof(*buffer_list) *
			      vm->num_user_mapped_buffers, true);
	if (!buffer_list) {
		mutex_unlock(&vm->update_gmmu_lock);
		return -ENOMEM;
	}

	node = rb_first(&vm->mapped_buffers);
	while (node) {
		mapped_buffer =
			container_of(node, struct mapped_buffer_node, node);
		if (mapped_buffer->user_mapped) {
			buffer_list[i] = mapped_buffer;
			kref_get(&mapped_buffer->ref);
			i++;
		}
		node = rb_next(&mapped_buffer->node);
	}

	BUG_ON(i != vm->num_user_mapped_buffers);

	*num_buffers = vm->num_user_mapped_buffers;
	*mapped_buffers = buffer_list;

	mutex_unlock(&vm->update_gmmu_lock);

	return 0;
}

static void gk20a_vm_unmap_locked_kref(struct kref *ref)
{
	struct mapped_buffer_node *mapped_buffer =
		container_of(ref, struct mapped_buffer_node, ref);
	gk20a_vm_unmap_locked(mapped_buffer, mapped_buffer->vm->kref_put_batch);
}

void gk20a_vm_mapping_batch_start(struct vm_gk20a_mapping_batch *mapping_batch)
{
	memset(mapping_batch, 0, sizeof(*mapping_batch));
	mapping_batch->gpu_l2_flushed = false;
	mapping_batch->need_tlb_invalidate = false;
}

void gk20a_vm_mapping_batch_finish_locked(
	struct vm_gk20a *vm, struct vm_gk20a_mapping_batch *mapping_batch)
{
	 /* hanging kref_put batch pointer? */
	WARN_ON(vm->kref_put_batch == mapping_batch);

	if (mapping_batch->need_tlb_invalidate) {
		struct gk20a *g = gk20a_from_vm(vm);
		g->ops.mm.tlb_invalidate(vm);
	}
}

void gk20a_vm_mapping_batch_finish(struct vm_gk20a *vm,
				   struct vm_gk20a_mapping_batch *mapping_batch)
{
	mutex_lock(&vm->update_gmmu_lock);
	gk20a_vm_mapping_batch_finish_locked(vm, mapping_batch);
	mutex_unlock(&vm->update_gmmu_lock);
}

void gk20a_vm_put_buffers(struct vm_gk20a *vm,
				 struct mapped_buffer_node **mapped_buffers,
				 int num_buffers)
{
	int i;
	struct vm_gk20a_mapping_batch batch;

	if (num_buffers == 0)
		return;

	mutex_lock(&vm->update_gmmu_lock);
	gk20a_vm_mapping_batch_start(&batch);
	vm->kref_put_batch = &batch;

	for (i = 0; i < num_buffers; ++i)
		kref_put(&mapped_buffers[i]->ref,
			 gk20a_vm_unmap_locked_kref);

	vm->kref_put_batch = NULL;
	gk20a_vm_mapping_batch_finish_locked(vm, &batch);
	mutex_unlock(&vm->update_gmmu_lock);

	nvgpu_free(mapped_buffers);
}

static void gk20a_vm_unmap_user(struct vm_gk20a *vm, u64 offset,
				struct vm_gk20a_mapping_batch *batch)
{
	struct device *d = dev_from_vm(vm);
	int retries = 10000; /* 50 ms */
	struct mapped_buffer_node *mapped_buffer;

	mutex_lock(&vm->update_gmmu_lock);

	mapped_buffer = find_mapped_buffer_locked(&vm->mapped_buffers, offset);
	if (!mapped_buffer) {
		mutex_unlock(&vm->update_gmmu_lock);
		gk20a_err(d, "invalid addr to unmap 0x%llx", offset);
		return;
	}

	if (mapped_buffer->flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET) {
		mutex_unlock(&vm->update_gmmu_lock);

		while (retries >= 0 || !tegra_platform_is_silicon()) {
			if (atomic_read(&mapped_buffer->ref.refcount) == 1)
				break;
			retries--;
			udelay(5);
		}
		if (retries < 0 && tegra_platform_is_silicon())
			gk20a_err(d, "sync-unmap failed on 0x%llx",
								offset);
		mutex_lock(&vm->update_gmmu_lock);
	}

	if (mapped_buffer->user_mapped == 0) {
		mutex_unlock(&vm->update_gmmu_lock);
		gk20a_err(d, "addr already unmapped from user 0x%llx", offset);
		return;
	}

	mapped_buffer->user_mapped--;
	if (mapped_buffer->user_mapped == 0)
		vm->num_user_mapped_buffers--;

	vm->kref_put_batch = batch;
	kref_put(&mapped_buffer->ref, gk20a_vm_unmap_locked_kref);
	vm->kref_put_batch = NULL;

	mutex_unlock(&vm->update_gmmu_lock);
}

u64 gk20a_vm_alloc_va(struct vm_gk20a *vm,
		      u64 size,
		      enum gmmu_pgsz_gk20a gmmu_pgsz_idx)

{
	struct gk20a_allocator *vma = &vm->vma[gmmu_pgsz_idx];
	u64 offset;
	u64 gmmu_page_size = vm->gmmu_page_sizes[gmmu_pgsz_idx];

	if (gmmu_pgsz_idx >= gmmu_nr_page_sizes) {
		dev_warn(dev_from_vm(vm),
			 "invalid page size requested in gk20a vm alloc");
		return 0;
	}

	if ((gmmu_pgsz_idx == gmmu_page_size_big) && !vm->big_pages) {
		dev_warn(dev_from_vm(vm),
			 "unsupportd page size requested");
		return 0;

	}

	/* Be certain we round up to gmmu_page_size if needed */
	size = (size + ((u64)gmmu_page_size - 1)) & ~((u64)gmmu_page_size - 1);
	gk20a_dbg_info("size=0x%llx @ pgsz=%dKB", size,
			vm->gmmu_page_sizes[gmmu_pgsz_idx]>>10);

	offset = gk20a_balloc(vma, size);
	if (!offset) {
		gk20a_err(dev_from_vm(vm),
			  "%s oom: sz=0x%llx", vma->name, size);
		return 0;
	}

	gk20a_dbg_fn("%s found addr: 0x%llx", vma->name, offset);
	return offset;
}

int gk20a_vm_free_va(struct vm_gk20a *vm,
		     u64 offset, u64 size,
		     enum gmmu_pgsz_gk20a pgsz_idx)
{
	struct gk20a_allocator *vma = &vm->vma[pgsz_idx];

	gk20a_dbg_info("%s free addr=0x%llx, size=0x%llx",
			vma->name, offset, size);
	gk20a_bfree(vma, offset);

	return 0;
}

static int insert_mapped_buffer(struct rb_root *root,
				struct mapped_buffer_node *mapped_buffer)
{
	struct rb_node **new_node = &(root->rb_node), *parent = NULL;

	/* Figure out where to put new node */
	while (*new_node) {
		struct mapped_buffer_node *cmp_with =
			container_of(*new_node, struct mapped_buffer_node,
				     node);

		parent = *new_node;

		if (cmp_with->addr > mapped_buffer->addr) /* u64 cmp */
			new_node = &((*new_node)->rb_left);
		else if (cmp_with->addr != mapped_buffer->addr) /* u64 cmp */
			new_node = &((*new_node)->rb_right);
		else
			return -EINVAL; /* no fair dup'ing */
	}

	/* Add new node and rebalance tree. */
	rb_link_node(&mapped_buffer->node, parent, new_node);
	rb_insert_color(&mapped_buffer->node, root);

	return 0;
}

static struct mapped_buffer_node *find_mapped_buffer_reverse_locked(
				struct rb_root *root, struct dma_buf *dmabuf,
				u32 kind)
{
	struct rb_node *node = rb_first(root);
	while (node) {
		struct mapped_buffer_node *mapped_buffer =
			container_of(node, struct mapped_buffer_node, node);
		if (mapped_buffer->dmabuf == dmabuf &&
		    kind == mapped_buffer->kind)
			return mapped_buffer;
		node = rb_next(&mapped_buffer->node);
	}
	return NULL;
}

static struct mapped_buffer_node *find_mapped_buffer_locked(
					struct rb_root *root, u64 addr)
{

	struct rb_node *node = root->rb_node;
	while (node) {
		struct mapped_buffer_node *mapped_buffer =
			container_of(node, struct mapped_buffer_node, node);
		if (mapped_buffer->addr > addr) /* u64 cmp */
			node = node->rb_left;
		else if (mapped_buffer->addr != addr) /* u64 cmp */
			node = node->rb_right;
		else
			return mapped_buffer;
	}
	return NULL;
}

static struct mapped_buffer_node *find_mapped_buffer_range_locked(
					struct rb_root *root, u64 addr)
{
	struct rb_node *node = root->rb_node;
	while (node) {
		struct mapped_buffer_node *m =
			container_of(node, struct mapped_buffer_node, node);
		if (m->addr <= addr && m->addr + m->size > addr)
			return m;
		else if (m->addr > addr) /* u64 cmp */
			node = node->rb_left;
		else
			node = node->rb_right;
	}
	return NULL;
}

/* find the first mapped buffer with GPU VA less than addr */
static struct mapped_buffer_node *find_mapped_buffer_less_than_locked(
	struct rb_root *root, u64 addr)
{
	struct rb_node *node = root->rb_node;
	struct mapped_buffer_node *ret = NULL;

	while (node) {
		struct mapped_buffer_node *mapped_buffer =
			container_of(node, struct mapped_buffer_node, node);
		if (mapped_buffer->addr >= addr)
			node = node->rb_left;
		else {
			ret = mapped_buffer;
			node = node->rb_right;
		}
	}

	return ret;
}

#define BFR_ATTRS (sizeof(nvmap_bfr_param)/sizeof(nvmap_bfr_param[0]))

struct buffer_attrs {
	struct sg_table *sgt;
	u64 size;
	u64 align;
	u32 ctag_offset;
	u32 ctag_lines;
	u32 ctag_allocated_lines;
	int pgsz_idx;
	u8 kind_v;
	u8 uc_kind_v;
	bool ctag_user_mappable;
};

static void gmmu_select_page_size(struct vm_gk20a *vm,
				  struct buffer_attrs *bfr)
{
	int i;
	/*  choose the biggest first (top->bottom) */
	for (i = gmmu_page_size_kernel - 1; i >= 0; i--)
		if (!((vm->gmmu_page_sizes[i] - 1) & bfr->align)) {
			bfr->pgsz_idx = i;
			break;
		}
}

static int setup_buffer_kind_and_compression(struct vm_gk20a *vm,
					     u32 flags,
					     struct buffer_attrs *bfr,
					     enum gmmu_pgsz_gk20a pgsz_idx)
{
	bool kind_compressible;
	struct gk20a *g = gk20a_from_vm(vm);
	struct device *d = dev_from_gk20a(g);
	int ctag_granularity = g->ops.fb.compression_page_size(g);

	if (unlikely(bfr->kind_v == gmmu_pte_kind_invalid_v()))
		bfr->kind_v = gmmu_pte_kind_pitch_v();

	if (unlikely(!gk20a_kind_is_supported(bfr->kind_v))) {
		gk20a_err(d, "kind 0x%x not supported", bfr->kind_v);
		return -EINVAL;
	}

	bfr->uc_kind_v = gmmu_pte_kind_invalid_v();
	/* find a suitable uncompressed kind if it becomes necessary later */
	kind_compressible = gk20a_kind_is_compressible(bfr->kind_v);
	if (kind_compressible) {
		bfr->uc_kind_v = gk20a_get_uncompressed_kind(bfr->kind_v);
		if (unlikely(bfr->uc_kind_v == gmmu_pte_kind_invalid_v())) {
			/* shouldn't happen, but it is worth cross-checking */
			gk20a_err(d, "comptag kind 0x%x can't be"
				   " downgraded to uncompressed kind",
				   bfr->kind_v);
			return -EINVAL;
		}
	}
	/* comptags only supported for suitable kinds, 128KB pagesize */
	if (kind_compressible &&
	    vm->gmmu_page_sizes[pgsz_idx] < g->ops.fb.compressible_page_size(g)) {
		/*
		gk20a_warn(d, "comptags specified"
		" but pagesize being used doesn't support it");*/
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

static int validate_fixed_buffer(struct vm_gk20a *vm,
				 struct buffer_attrs *bfr,
				 u64 map_offset, u64 map_size,
				 struct vm_reserved_va_node **pva_node)
{
	struct device *dev = dev_from_vm(vm);
	struct vm_reserved_va_node *va_node;
	struct mapped_buffer_node *buffer;
	u64 map_end = map_offset + map_size;

	/* can wrap around with insane map_size; zero is disallowed too */
	if (map_end <= map_offset) {
		gk20a_warn(dev, "fixed offset mapping with invalid map_size");
		return -EINVAL;
	}

	if (map_offset & (vm->gmmu_page_sizes[bfr->pgsz_idx] - 1)) {
		gk20a_err(dev, "map offset must be buffer page size aligned 0x%llx",
			  map_offset);
		return -EINVAL;
	}

	/* Find the space reservation, but it's ok to have none for
	 * userspace-managed address spaces */
	va_node = addr_to_reservation(vm, map_offset);
	if (!va_node && !vm->userspace_managed) {
		gk20a_warn(dev, "fixed offset mapping without space allocation");
		return -EINVAL;
	}

	/* Mapped area should fit inside va, if there's one */
	if (va_node && map_end > va_node->vaddr_start + va_node->size) {
		gk20a_warn(dev, "fixed offset mapping size overflows va node");
		return -EINVAL;
	}

	/* check that this mapping does not collide with existing
	 * mappings by checking the buffer with the highest GPU VA
	 * that is less than our buffer end */
	buffer = find_mapped_buffer_less_than_locked(
		&vm->mapped_buffers, map_offset + map_size);
	if (buffer && buffer->addr + buffer->size > map_offset) {
		gk20a_warn(dev, "overlapping buffer map requested");
		return -EINVAL;
	}

	*pva_node = va_node;

	return 0;
}

u64 gk20a_locked_gmmu_map(struct vm_gk20a *vm,
			u64 map_offset,
			struct sg_table *sgt,
			u64 buffer_offset,
			u64 size,
			int pgsz_idx,
			u8 kind_v,
			u32 ctag_offset,
			u32 flags,
			int rw_flag,
			bool clear_ctags,
			bool sparse,
			bool priv,
			struct vm_gk20a_mapping_batch *batch)
{
	int err = 0;
	bool allocated = false;
	struct device *d = dev_from_vm(vm);
	struct gk20a *g = gk20a_from_vm(vm);
	int ctag_granularity = g->ops.fb.compression_page_size(g);
	u32 ctag_lines = DIV_ROUND_UP_ULL(size, ctag_granularity);

	if (clear_ctags && ctag_offset) {
		/* init/clear the ctag buffer */
		g->ops.ltc.cbc_ctrl(g, gk20a_cbc_op_clear,
				ctag_offset, ctag_offset + ctag_lines - 1);
	}

	/* Allocate (or validate when map_offset != 0) the virtual address. */
	if (!map_offset) {
		map_offset = gk20a_vm_alloc_va(vm, size,
					  pgsz_idx);
		if (!map_offset) {
			gk20a_err(d, "failed to allocate va space");
			err = -ENOMEM;
			goto fail_alloc;
		}
		allocated = true;
	}

	gk20a_dbg(gpu_dbg_map,
	   "as=%d pgsz=%d "
	   "kind=0x%x flags=0x%x "
	   "ctags=%d start=%d gv=0x%x,%08x -> 0x%x,%08x -> 0x%x,%08x + %llx",
	   vm_aspace_id(vm), pgsz_idx,
	   kind_v, flags,
	   ctag_lines, ctag_offset,
	   hi32(map_offset), lo32(map_offset),
	   hi32((u64)sg_dma_address(sgt->sgl)),
	   lo32((u64)sg_dma_address(sgt->sgl)),
	   hi32((u64)sg_phys(sgt->sgl)),
	   lo32((u64)sg_phys(sgt->sgl)),
	   buffer_offset);

	err = update_gmmu_ptes_locked(vm, pgsz_idx,
				      sgt,
				      buffer_offset,
				      map_offset, map_offset + size,
				      kind_v,
				      ctag_offset,
				      flags &
				      NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_TRUE,
				      flags &
				      NVGPU_AS_MAP_BUFFER_FLAGS_UNMAPPED_PTE,
				      rw_flag,
				      sparse,
				      priv);
	if (err) {
		gk20a_err(d, "failed to update ptes on map");
		goto fail_validate;
	}

	if (!batch)
		g->ops.mm.tlb_invalidate(vm);
	else
		batch->need_tlb_invalidate = true;

	return map_offset;
fail_validate:
	if (allocated)
		gk20a_vm_free_va(vm, map_offset, size, pgsz_idx);
fail_alloc:
	gk20a_err(d, "%s: failed with err=%d\n", __func__, err);
	return 0;
}

void gk20a_locked_gmmu_unmap(struct vm_gk20a *vm,
			u64 vaddr,
			u64 size,
			int pgsz_idx,
			bool va_allocated,
			int rw_flag,
			bool sparse,
			struct vm_gk20a_mapping_batch *batch)
{
	int err = 0;
	struct gk20a *g = gk20a_from_vm(vm);

	if (va_allocated) {
		err = gk20a_vm_free_va(vm, vaddr, size, pgsz_idx);
		if (err) {
			dev_err(dev_from_vm(vm),
				"failed to free va");
			return;
		}
	}

	/* unmap here needs to know the page size we assigned at mapping */
	err = update_gmmu_ptes_locked(vm,
				pgsz_idx,
				NULL, /* n/a for unmap */
				0,
				vaddr,
				vaddr + size,
				0, 0, false /* n/a for unmap */,
				false, rw_flag,
				sparse, 0);
	if (err)
		dev_err(dev_from_vm(vm),
			"failed to update gmmu ptes on unmap");

	/* flush l2 so any dirty lines are written out *now*.
	 *  also as we could potentially be switching this buffer
	 * from nonvolatile (l2 cacheable) to volatile (l2 non-cacheable) at
	 * some point in the future we need to invalidate l2.  e.g. switching
	 * from a render buffer unmap (here) to later using the same memory
	 * for gmmu ptes.  note the positioning of this relative to any smmu
	 * unmapping (below). */

	if (!batch) {
		gk20a_mm_l2_flush(g, true);
		g->ops.mm.tlb_invalidate(vm);
	} else {
		if (!batch->gpu_l2_flushed) {
			gk20a_mm_l2_flush(g, true);
			batch->gpu_l2_flushed = true;
		}
		batch->need_tlb_invalidate = true;
	}
}

static u64 gk20a_vm_map_duplicate_locked(struct vm_gk20a *vm,
					 struct dma_buf *dmabuf,
					 u64 offset_align,
					 u32 flags,
					 int kind,
					 struct sg_table **sgt,
					 bool user_mapped,
					 int rw_flag)
{
	struct mapped_buffer_node *mapped_buffer = NULL;

	mapped_buffer =
		find_mapped_buffer_reverse_locked(&vm->mapped_buffers,
						  dmabuf, kind);
	if (!mapped_buffer)
		return 0;

	if (mapped_buffer->flags != flags)
		return 0;

	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET &&
	    mapped_buffer->addr != offset_align)
		return 0;

	BUG_ON(mapped_buffer->vm != vm);

	/* mark the buffer as used */
	if (user_mapped) {
		if (mapped_buffer->user_mapped == 0)
			vm->num_user_mapped_buffers++;
		mapped_buffer->user_mapped++;

		/* If the mapping comes from user space, we own
		 * the handle ref. Since we reuse an
		 * existing mapping here, we need to give back those
		 * refs once in order not to leak.
		 */
		if (mapped_buffer->own_mem_ref)
			dma_buf_put(mapped_buffer->dmabuf);
		else
			mapped_buffer->own_mem_ref = true;
	}
	kref_get(&mapped_buffer->ref);

	gk20a_dbg(gpu_dbg_map,
		   "reusing as=%d pgsz=%d flags=0x%x ctags=%d "
		   "start=%d gv=0x%x,%08x -> 0x%x,%08x -> 0x%x,%08x "
		   "own_mem_ref=%d user_mapped=%d",
		   vm_aspace_id(vm), mapped_buffer->pgsz_idx,
		   mapped_buffer->flags,
		   mapped_buffer->ctag_lines,
		   mapped_buffer->ctag_offset,
		   hi32(mapped_buffer->addr), lo32(mapped_buffer->addr),
		   hi32((u64)sg_dma_address(mapped_buffer->sgt->sgl)),
		   lo32((u64)sg_dma_address(mapped_buffer->sgt->sgl)),
		   hi32((u64)sg_phys(mapped_buffer->sgt->sgl)),
		   lo32((u64)sg_phys(mapped_buffer->sgt->sgl)),
		   mapped_buffer->own_mem_ref, user_mapped);

	if (sgt)
		*sgt = mapped_buffer->sgt;
	return mapped_buffer->addr;
}

u64 gk20a_vm_map(struct vm_gk20a *vm,
			struct dma_buf *dmabuf,
			u64 offset_align,
			u32 flags /*NVGPU_AS_MAP_BUFFER_FLAGS_*/,
			int kind,
			struct sg_table **sgt,
			bool user_mapped,
			int rw_flag,
			u64 buffer_offset,
			u64 mapping_size,
			struct vm_gk20a_mapping_batch *batch)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct gk20a_comptag_allocator *ctag_allocator = &g->gr.comp_tags;
	struct device *d = dev_from_vm(vm);
	struct mapped_buffer_node *mapped_buffer = NULL;
	bool inserted = false, va_allocated = false;
	u32 gmmu_page_size = 0;
	u64 map_offset = 0;
	int err = 0;
	struct buffer_attrs bfr = {NULL};
	struct gk20a_comptags comptags;
	bool clear_ctags = false;
	struct scatterlist *sgl;
	u64 buf_addr;
	u64 ctag_map_win_size = 0;
	u32 ctag_map_win_ctagline = 0;
	u32 ctag_offset;
	struct vm_reserved_va_node *va_node = NULL;

	if (user_mapped && vm->userspace_managed &&
	    !(flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET)) {
		gk20a_err(d,
			  "%s: non-fixed-offset mapping not available on userspace managed address spaces",
			  __func__);
		return -EFAULT;
	}

	mutex_lock(&vm->update_gmmu_lock);

	/* check if this buffer is already mapped */
	if (!vm->userspace_managed) {
		map_offset = gk20a_vm_map_duplicate_locked(
			vm, dmabuf, offset_align,
			flags, kind, sgt,
			user_mapped, rw_flag);
		if (map_offset) {
			mutex_unlock(&vm->update_gmmu_lock);
			return map_offset;
		}
	}

	/* pin buffer to get phys/iovmm addr */
	bfr.sgt = gk20a_mm_pin(d, dmabuf);
	if (IS_ERR(bfr.sgt)) {
		/* Falling back to physical is actually possible
		 * here in many cases if we use 4K phys pages in the
		 * gmmu.  However we have some regions which require
		 * contig regions to work properly (either phys-contig
		 * or contig through smmu io_vaspace).  Until we can
		 * track the difference between those two cases we have
		 * to fail the mapping when we run out of SMMU space.
		 */
		gk20a_warn(d, "oom allocating tracking buffer");
		goto clean_up;
	}

	if (sgt)
		*sgt = bfr.sgt;

	bfr.kind_v = kind;
	bfr.size = dmabuf->size;
	sgl = bfr.sgt->sgl;
	buf_addr = (u64)sg_dma_address(bfr.sgt->sgl);
	if (g->mm.bypass_smmu || buf_addr == DMA_ERROR_CODE || !buf_addr) {
		while (sgl) {
			u64 align;

			buf_addr = (u64)sg_phys(sgl);

			align = 1ULL << __ffs(buf_addr | (u64)sgl->length);
			if (bfr.align)
				bfr.align = min_t(u64, align, bfr.align);
			else
				bfr.align = align;
			sgl = sg_next(sgl);
		}
	} else
		bfr.align = 1ULL << __ffs(buf_addr);

	bfr.pgsz_idx = -1;
	mapping_size = mapping_size ? mapping_size : bfr.size;

	if (vm->big_pages)
		gmmu_select_page_size(vm, &bfr);
	else
		bfr.pgsz_idx = gmmu_page_size_small;

	/* If FIX_OFFSET is set, pgsz is determined at address allocation
	 * time. The alignment at address alloc time must be the same as
	 * the alignment determined by gmmu_select_page_size().
	 */
	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET) {
		int pgsz_idx =
			__nv_gmmu_va_is_big_page_region(vm, offset_align) ?
			gmmu_page_size_big : gmmu_page_size_small;
		if (pgsz_idx > bfr.pgsz_idx) {
			gk20a_err(d, "%llx buffer pgsz %d, VA pgsz %d",
				  offset_align, bfr.pgsz_idx, pgsz_idx);
			err = -EINVAL;
			goto clean_up;
		}
		bfr.pgsz_idx = min(bfr.pgsz_idx, pgsz_idx);
	}

	/* validate/adjust bfr attributes */
	if (unlikely(bfr.pgsz_idx == -1)) {
		gk20a_err(d, "unsupported page size detected");
		goto clean_up;
	}

	if (unlikely(bfr.pgsz_idx < gmmu_page_size_small ||
		     bfr.pgsz_idx > gmmu_page_size_big)) {
		BUG_ON(1);
		err = -EINVAL;
		goto clean_up;
	}
	gmmu_page_size = vm->gmmu_page_sizes[bfr.pgsz_idx];

	/* Check if we should use a fixed offset for mapping this buffer */

	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET)  {
		err = validate_fixed_buffer(vm, &bfr,
					    offset_align, mapping_size,
					    &va_node);
		if (err)
			goto clean_up;

		map_offset = offset_align;
		va_allocated = false;
	} else
		va_allocated = true;

	if (sgt)
		*sgt = bfr.sgt;

	err = setup_buffer_kind_and_compression(vm, flags, &bfr, bfr.pgsz_idx);
	if (unlikely(err)) {
		gk20a_err(d, "failure setting up kind and compression");
		goto clean_up;
	}

	/* bar1 and pmu vm don't need ctag */
	if (!vm->enable_ctag)
		bfr.ctag_lines = 0;

	gk20a_get_comptags(d, dmabuf, &comptags);

	/* ensure alignment to compression page size if compression enabled */
	if (bfr.ctag_offset)
		mapping_size = ALIGN(mapping_size,
				     g->ops.fb.compression_page_size(g));

	if (bfr.ctag_lines && !comptags.lines) {
		const bool user_mappable =
			!!(flags & NVGPU_AS_MAP_BUFFER_FLAGS_MAPPABLE_COMPBITS);

		/* allocate compression resources if needed */
		err = gk20a_alloc_comptags(g, d, dmabuf, ctag_allocator,
					   bfr.ctag_lines, user_mappable,
					   &ctag_map_win_size,
					   &ctag_map_win_ctagline);
		if (err) {
			/* ok to fall back here if we ran out */
			/* TBD: we can partially alloc ctags as well... */
			bfr.kind_v = bfr.uc_kind_v;
		} else {
			gk20a_get_comptags(d, dmabuf, &comptags);
			clear_ctags = true;

			if (comptags.lines < comptags.allocated_lines) {
				/* clear tail-padding comptags */
				u32 ctagmin = comptags.offset + comptags.lines;
				u32 ctagmax = comptags.offset +
					comptags.allocated_lines - 1;

				g->ops.ltc.cbc_ctrl(g, gk20a_cbc_op_clear,
						    ctagmin, ctagmax);
			}
		}
	}

	/* store the comptag info */
	bfr.ctag_offset = comptags.offset;
	bfr.ctag_lines = comptags.lines;
	bfr.ctag_allocated_lines = comptags.allocated_lines;
	bfr.ctag_user_mappable = comptags.user_mappable;

	/*
	 * Calculate comptag index for this mapping. Differs in
	 * case of partial mapping.
	 */
	ctag_offset = comptags.offset;
	if (ctag_offset)
		ctag_offset += buffer_offset /
			       g->ops.fb.compression_page_size(g);

	/* update gmmu ptes */
	map_offset = g->ops.mm.gmmu_map(vm, map_offset,
					bfr.sgt,
					buffer_offset, /* sg offset */
					mapping_size,
					bfr.pgsz_idx,
					bfr.kind_v,
					ctag_offset,
					flags, rw_flag,
					clear_ctags,
					false,
					false,
					batch);
	if (!map_offset)
		goto clean_up;

	gk20a_dbg(gpu_dbg_map,
	   "as=%d pgsz=%d "
	   "kind=0x%x kind_uc=0x%x flags=0x%x "
	   "ctags=%d start=%d ctags_allocated=%d ctags_mappable=%d gv=0x%x,%08x -> 0x%x,%08x -> 0x%x,%08x",
	   vm_aspace_id(vm), gmmu_page_size,
	   bfr.kind_v, bfr.uc_kind_v, flags,
	   bfr.ctag_lines, bfr.ctag_offset,
	   bfr.ctag_allocated_lines, bfr.ctag_user_mappable,
	   hi32(map_offset), lo32(map_offset),
	   hi32((u64)sg_dma_address(bfr.sgt->sgl)),
	   lo32((u64)sg_dma_address(bfr.sgt->sgl)),
	   hi32((u64)sg_phys(bfr.sgt->sgl)),
	   lo32((u64)sg_phys(bfr.sgt->sgl)));

#if defined(NVHOST_DEBUG)
	{
		int i;
		struct scatterlist *sg = NULL;
		gk20a_dbg(gpu_dbg_pte, "for_each_sg(bfr.sgt->sgl, sg, bfr.sgt->nents, i)");
		for_each_sg(bfr.sgt->sgl, sg, bfr.sgt->nents, i ) {
			u64 da = sg_dma_address(sg);
			u64 pa = sg_phys(sg);
			u64 len = sg->length;
			gk20a_dbg(gpu_dbg_pte, "i=%d pa=0x%x,%08x da=0x%x,%08x len=0x%x,%08x",
				   i, hi32(pa), lo32(pa), hi32(da), lo32(da),
				   hi32(len), lo32(len));
		}
	}
#endif

	/* keep track of the buffer for unmapping */
	/* TBD: check for multiple mapping of same buffer */
	mapped_buffer = kzalloc(sizeof(*mapped_buffer), GFP_KERNEL);
	if (!mapped_buffer) {
		gk20a_warn(d, "oom allocating tracking buffer");
		goto clean_up;
	}
	mapped_buffer->dmabuf      = dmabuf;
	mapped_buffer->sgt         = bfr.sgt;
	mapped_buffer->addr        = map_offset;
	mapped_buffer->size        = mapping_size;
	mapped_buffer->pgsz_idx    = bfr.pgsz_idx;
	mapped_buffer->ctag_offset = bfr.ctag_offset;
	mapped_buffer->ctag_lines  = bfr.ctag_lines;
	mapped_buffer->ctag_allocated_lines = bfr.ctag_allocated_lines;
	mapped_buffer->ctags_mappable = bfr.ctag_user_mappable;
	mapped_buffer->ctag_map_win_size = ctag_map_win_size;
	mapped_buffer->ctag_map_win_ctagline = ctag_map_win_ctagline;
	mapped_buffer->vm          = vm;
	mapped_buffer->flags       = flags;
	mapped_buffer->kind        = kind;
	mapped_buffer->va_allocated = va_allocated;
	mapped_buffer->user_mapped = user_mapped ? 1 : 0;
	mapped_buffer->own_mem_ref = user_mapped;
	INIT_LIST_HEAD(&mapped_buffer->unmap_list);
	INIT_LIST_HEAD(&mapped_buffer->va_buffers_list);
	kref_init(&mapped_buffer->ref);

	err = insert_mapped_buffer(&vm->mapped_buffers, mapped_buffer);
	if (err) {
		gk20a_err(d, "failed to insert into mapped buffer tree");
		goto clean_up;
	}
	inserted = true;
	if (user_mapped)
		vm->num_user_mapped_buffers++;

	gk20a_dbg_info("allocated va @ 0x%llx", map_offset);

	if (va_node) {
		list_add_tail(&mapped_buffer->va_buffers_list,
			      &va_node->va_buffers_list);
		mapped_buffer->va_node = va_node;
	}

	mutex_unlock(&vm->update_gmmu_lock);

	return map_offset;

clean_up:
	if (inserted) {
		rb_erase(&mapped_buffer->node, &vm->mapped_buffers);
		if (user_mapped)
			vm->num_user_mapped_buffers--;
	}
	kfree(mapped_buffer);
	if (va_allocated)
		gk20a_vm_free_va(vm, map_offset, bfr.size, bfr.pgsz_idx);
	if (!IS_ERR(bfr.sgt))
		gk20a_mm_unpin(d, dmabuf, bfr.sgt);

	mutex_unlock(&vm->update_gmmu_lock);
	gk20a_dbg_info("err=%d\n", err);
	return 0;
}

int gk20a_vm_get_compbits_info(struct vm_gk20a *vm,
			       u64 mapping_gva,
			       u64 *compbits_win_size,
			       u32 *compbits_win_ctagline,
			       u32 *mapping_ctagline,
			       u32 *flags)
{
	struct mapped_buffer_node *mapped_buffer;
	struct device *d = dev_from_vm(vm);

	mutex_lock(&vm->update_gmmu_lock);

	mapped_buffer = find_mapped_buffer_locked(&vm->mapped_buffers, mapping_gva);

	if (!mapped_buffer || !mapped_buffer->user_mapped)
	{
		mutex_unlock(&vm->update_gmmu_lock);
		gk20a_err(d, "%s: bad offset 0x%llx", __func__, mapping_gva);
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

	mutex_unlock(&vm->update_gmmu_lock);
	return 0;
}


int gk20a_vm_map_compbits(struct vm_gk20a *vm,
			  u64 mapping_gva,
			  u64 *compbits_win_gva,
			  u64 *mapping_iova,
			  u32 flags)
{
	struct mapped_buffer_node *mapped_buffer;
	struct gk20a *g = gk20a_from_vm(vm);
	struct device *d = dev_from_vm(vm);
	const bool fixed_mapping =
		(flags & NVGPU_AS_MAP_BUFFER_COMPBITS_FLAGS_FIXED_OFFSET) != 0;

	if (vm->userspace_managed && !fixed_mapping) {
		gk20a_err(d,
			  "%s: non-fixed-offset mapping is not available on userspace managed address spaces",
			  __func__);
		return -EFAULT;
	}

	if (fixed_mapping && !vm->userspace_managed) {
		gk20a_err(d,
			  "%s: fixed-offset mapping is available only on userspace managed address spaces",
			  __func__);
		return -EFAULT;
	}

	mutex_lock(&vm->update_gmmu_lock);

	mapped_buffer =
		find_mapped_buffer_locked(&vm->mapped_buffers, mapping_gva);

	if (!mapped_buffer || !mapped_buffer->user_mapped) {
		mutex_unlock(&vm->update_gmmu_lock);
		gk20a_err(d, "%s: bad offset 0x%llx", __func__, mapping_gva);
		return -EFAULT;
	}

	if (!mapped_buffer->ctags_mappable) {
		mutex_unlock(&vm->update_gmmu_lock);
		gk20a_err(d, "%s: comptags not mappable, offset 0x%llx",
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
			mutex_unlock(&vm->update_gmmu_lock);
			gk20a_err(d,
				  "%s: mapping 0x%llx does not have "
				  "mappable comptags",
				  __func__, mapping_gva);
			return -EFAULT;
		}

		cacheline_start = mapped_buffer->ctag_offset /
			g->gr.comptags_per_cacheline;
		cacheline_offset_start =
			cacheline_start * aggregate_cacheline_sz;

		if (fixed_mapping) {
			struct buffer_attrs bfr;
			int err;
			struct vm_reserved_va_node *va_node = NULL;

			memset(&bfr, 0, sizeof(bfr));

			bfr.pgsz_idx = small_pgsz_index;

			err = validate_fixed_buffer(
				vm, &bfr, *compbits_win_gva,
				mapped_buffer->ctag_map_win_size, &va_node);

			if (err) {
				mutex_unlock(&vm->update_gmmu_lock);
				return err;
			}

			if (va_node) {
				/* this would create a dangling GPU VA
				 * pointer if the space is freed
				 * before before the buffer is
				 * unmapped */
				mutex_unlock(&vm->update_gmmu_lock);
				gk20a_err(d,
					  "%s: comptags cannot be mapped into allocated space",
					  __func__);
				return -EINVAL;
			}
		}

		mapped_buffer->ctag_map_win_addr =
			g->ops.mm.gmmu_map(
				vm,
				!fixed_mapping ? 0 : *compbits_win_gva, /* va */
				g->gr.compbit_store.mem.sgt,
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
				NULL); /* mapping_batch handle */

		if (!mapped_buffer->ctag_map_win_addr) {
			mutex_unlock(&vm->update_gmmu_lock);
			gk20a_err(d,
				  "%s: failed to map comptags for mapping 0x%llx",
				  __func__, mapping_gva);
			return -ENOMEM;
		}
	} else if (fixed_mapping && *compbits_win_gva &&
		   mapped_buffer->ctag_map_win_addr != *compbits_win_gva) {
		mutex_unlock(&vm->update_gmmu_lock);
		gk20a_err(d,
			  "%s: re-requesting comptags map into mismatching address. buffer offset 0x"
			  "%llx, existing comptag map at 0x%llx, requested remap 0x%llx",
			  __func__, mapping_gva,
			  mapped_buffer->ctag_map_win_addr, *compbits_win_gva);
		return -EINVAL;
	}

	*mapping_iova = gk20a_mm_iova_addr(g, mapped_buffer->sgt->sgl, 0);
	*compbits_win_gva = mapped_buffer->ctag_map_win_addr;

	mutex_unlock(&vm->update_gmmu_lock);

	return 0;
}

u64 gk20a_gmmu_map(struct vm_gk20a *vm,
		struct sg_table **sgt,
		u64 size,
		u32 flags,
		int rw_flag,
		bool priv)
{
	struct gk20a *g = gk20a_from_vm(vm);
	u64 vaddr;

	mutex_lock(&vm->update_gmmu_lock);
	vaddr = g->ops.mm.gmmu_map(vm, 0, /* already mapped? - No */
				*sgt, /* sg table */
				0, /* sg offset */
				size,
				gmmu_page_size_kernel,
				0, /* kind */
				0, /* ctag_offset */
				flags, rw_flag,
				false, /* clear_ctags */
				false, /* sparse */
				priv, /* priv */
				NULL); /* mapping_batch handle */
	mutex_unlock(&vm->update_gmmu_lock);
	if (!vaddr) {
		gk20a_err(dev_from_vm(vm), "failed to allocate va space");
		return 0;
	}

	return vaddr;
}

int gk20a_gmmu_alloc(struct gk20a *g, size_t size, struct mem_desc *mem)
{
	return gk20a_gmmu_alloc_attr(g, 0, size, mem);
}

int gk20a_gmmu_alloc_attr(struct gk20a *g, enum dma_attr attr, size_t size, struct mem_desc *mem)
{
	struct device *d = dev_from_gk20a(g);
	int err;
	dma_addr_t iova;

	gk20a_dbg_fn("");

	if (attr) {
		DEFINE_DMA_ATTRS(attrs);
		dma_set_attr(attr, &attrs);
		if (attr == DMA_ATTR_NO_KERNEL_MAPPING) {
			mem->pages = dma_alloc_attrs(d,
					size, &iova, GFP_KERNEL, &attrs);
			if (!mem->pages)
				return -ENOMEM;
		} else {
			mem->cpu_va = dma_alloc_attrs(d,
					size, &iova, GFP_KERNEL, &attrs);
			if (!mem->cpu_va)
				return -ENOMEM;
		}
	} else {
		mem->cpu_va = dma_alloc_coherent(d, size, &iova, GFP_KERNEL);
		if (!mem->cpu_va)
			return -ENOMEM;
	}

	if (attr == DMA_ATTR_NO_KERNEL_MAPPING)
		err = gk20a_get_sgtable_from_pages(d, &mem->sgt, mem->pages,
						   iova, size);
	else {
		err = gk20a_get_sgtable(d, &mem->sgt, mem->cpu_va, iova, size);
		memset(mem->cpu_va, 0, size);
	}
	if (err)
		goto fail_free;

	mem->size = size;

	gk20a_dbg_fn("done");

	return 0;

fail_free:
	dma_free_coherent(d, size, mem->cpu_va, iova);
	mem->cpu_va = NULL;
	mem->sgt = NULL;
	return err;
}

void gk20a_gmmu_free_attr(struct gk20a *g, enum dma_attr attr,
			  struct mem_desc *mem)
{
	struct device *d = dev_from_gk20a(g);

	if (mem->cpu_va || mem->pages) {
		if (attr) {
			DEFINE_DMA_ATTRS(attrs);
			dma_set_attr(attr, &attrs);
			if (attr == DMA_ATTR_NO_KERNEL_MAPPING) {
				if (mem->pages)
					dma_free_attrs(d, mem->size, mem->pages,
						sg_dma_address(mem->sgt->sgl),
						&attrs);
			} else {
				if (mem->cpu_va)
					dma_free_attrs(d, mem->size,
						mem->cpu_va,
						sg_dma_address(mem->sgt->sgl),
						&attrs);
			}
		} else {
			if (mem->cpu_va)
				dma_free_coherent(d, mem->size, mem->cpu_va,
						sg_dma_address(mem->sgt->sgl));
		}
		mem->cpu_va = NULL;
		mem->pages = NULL;
	}

	if (mem->sgt)
		gk20a_free_sgtable(&mem->sgt);
}

void gk20a_gmmu_free(struct gk20a *g, struct mem_desc *mem)
{
	return gk20a_gmmu_free_attr(g, 0, mem);
}

int gk20a_gmmu_alloc_map(struct vm_gk20a *vm, size_t size, struct mem_desc *mem)
{
	return gk20a_gmmu_alloc_map_attr(vm, 0, size, mem);
}

int gk20a_gmmu_alloc_map_attr(struct vm_gk20a *vm,
			 enum dma_attr attr, size_t size, struct mem_desc *mem)
{
	int err = gk20a_gmmu_alloc_attr(vm->mm->g, attr, size, mem);

	if (err)
		return err;

	mem->gpu_va = gk20a_gmmu_map(vm, &mem->sgt, size, 0,
				     gk20a_mem_flag_none, false);
	if (!mem->gpu_va) {
		err = -ENOMEM;
		goto fail_free;
	}

	return 0;

fail_free:
	gk20a_gmmu_free(vm->mm->g, mem);
	return err;
}

void gk20a_gmmu_unmap_free(struct vm_gk20a *vm, struct mem_desc *mem)
{
	if (mem->gpu_va)
		gk20a_gmmu_unmap(vm, mem->gpu_va, mem->size, gk20a_mem_flag_none);
	mem->gpu_va = 0;

	gk20a_gmmu_free(vm->mm->g, mem);
}

dma_addr_t gk20a_mm_gpuva_to_iova_base(struct vm_gk20a *vm, u64 gpu_vaddr)
{
	struct mapped_buffer_node *buffer;
	dma_addr_t addr = 0;
	struct gk20a *g = gk20a_from_vm(vm);

	mutex_lock(&vm->update_gmmu_lock);
	buffer = find_mapped_buffer_locked(&vm->mapped_buffers, gpu_vaddr);
	if (buffer)
		addr = g->ops.mm.get_iova_addr(g, buffer->sgt->sgl,
				buffer->flags);
	mutex_unlock(&vm->update_gmmu_lock);

	return addr;
}

void gk20a_gmmu_unmap(struct vm_gk20a *vm,
		u64 vaddr,
		u64 size,
		int rw_flag)
{
	struct gk20a *g = gk20a_from_vm(vm);

	mutex_lock(&vm->update_gmmu_lock);
	g->ops.mm.gmmu_unmap(vm,
			vaddr,
			size,
			gmmu_page_size_kernel,
			true, /*va_allocated */
			rw_flag,
			false,
			NULL);
	mutex_unlock(&vm->update_gmmu_lock);
}

phys_addr_t gk20a_get_phys_from_iova(struct device *d,
				u64 dma_addr)
{
	phys_addr_t phys;
	u64 iova;

	struct dma_iommu_mapping *mapping = to_dma_iommu_mapping(d);
	if (!mapping)
		return dma_addr;

	iova = dma_addr & PAGE_MASK;
	phys = iommu_iova_to_phys(mapping->domain, iova);
	return phys;
}

/* get sg_table from already allocated buffer */
int gk20a_get_sgtable(struct device *d, struct sg_table **sgt,
			void *cpuva, u64 iova,
			size_t size)
{
	int err = 0;
	*sgt = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!(*sgt)) {
		dev_err(d, "failed to allocate memory\n");
		err = -ENOMEM;
		goto fail;
	}
	err = dma_get_sgtable(d, *sgt,
			cpuva, iova,
			size);
	if (err) {
		dev_err(d, "failed to create sg table\n");
		goto fail;
	}
	sg_dma_address((*sgt)->sgl) = iova;

	return 0;
 fail:
	if (*sgt) {
		kfree(*sgt);
		*sgt = NULL;
	}
	return err;
}

int gk20a_get_sgtable_from_pages(struct device *d, struct sg_table **sgt,
			struct page **pages, u64 iova,
			size_t size)
{
	int err = 0;
	*sgt = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!(*sgt)) {
		dev_err(d, "failed to allocate memory\n");
		err = -ENOMEM;
		goto fail;
	}
	err = sg_alloc_table_from_pages(*sgt, pages,
			DIV_ROUND_UP(size, 4096), 0, size, GFP_KERNEL);
	if (err) {
		dev_err(d, "failed to allocate sg_table\n");
		goto fail;
	}
	sg_dma_address((*sgt)->sgl) = iova;

	return 0;
 fail:
	if (*sgt) {
		kfree(*sgt);
		*sgt = NULL;
	}
	return err;
}

void gk20a_free_sgtable(struct sg_table **sgt)
{
	sg_free_table(*sgt);
	kfree(*sgt);
	*sgt = NULL;
}

u64 gk20a_mm_smmu_vaddr_translate(struct gk20a *g, dma_addr_t iova)
{
	if (!device_is_iommuable(dev_from_gk20a(g)))
		return iova;
	else
		return iova | 1ULL << g->ops.mm.get_physical_addr_bits(g);
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
static inline u32 big_valid_pde0_bits(u64 pte_addr)
{
	u32 pde0_bits =
		gmmu_pde_aperture_big_video_memory_f() |
		gmmu_pde_address_big_sys_f(
			   (u32)(pte_addr >> gmmu_pde_address_shift_v()));
	return  pde0_bits;
}

static inline u32 small_valid_pde1_bits(u64 pte_addr)
{
	u32 pde1_bits =
		gmmu_pde_aperture_small_video_memory_f() |
		gmmu_pde_vol_small_true_f() | /* tbd: why? */
		gmmu_pde_address_small_sys_f(
			   (u32)(pte_addr >> gmmu_pde_address_shift_v()));
	return pde1_bits;
}

/* Given the current state of the ptes associated with a pde,
   determine value and write it out.  There's no checking
   here to determine whether or not a change was actually
   made.  So, superfluous updates will cause unnecessary
   pde invalidations.
*/
static int update_gmmu_pde_locked(struct vm_gk20a *vm,
			   struct gk20a_mm_entry *pte,
			   u32 i, u32 gmmu_pgsz_idx,
			   struct scatterlist **sgl,
			   u64 *offset,
			   u64 *iova,
			   u32 kind_v, u64 *ctag,
			   bool cacheable, bool unammped_pte,
			   int rw_flag, bool sparse, bool priv)
{
	struct gk20a *g = gk20a_from_vm(vm);
	bool small_valid, big_valid;
	u64 pte_addr_small = 0, pte_addr_big = 0;
	struct gk20a_mm_entry *entry = vm->pdb.entries + i;
	u32 pde_v[2] = {0, 0};
	u32 *pde;

	gk20a_dbg_fn("");

	small_valid = entry->size && entry->pgsz == gmmu_page_size_small;
	big_valid   = entry->size && entry->pgsz == gmmu_page_size_big;

	if (small_valid)
		pte_addr_small = g->ops.mm.get_iova_addr(g, entry->sgt->sgl, 0);

	if (big_valid)
		pte_addr_big = g->ops.mm.get_iova_addr(g, entry->sgt->sgl, 0);

	pde_v[0] = gmmu_pde_size_full_f();
	pde_v[0] |= big_valid ? big_valid_pde0_bits(pte_addr_big) :
		(gmmu_pde_aperture_big_invalid_f());

	pde_v[1] |= (small_valid ?
		     small_valid_pde1_bits(pte_addr_small) :
		     (gmmu_pde_aperture_small_invalid_f() |
		      gmmu_pde_vol_small_false_f()))
		    |
		    (big_valid ? (gmmu_pde_vol_big_true_f()) :
		     gmmu_pde_vol_big_false_f());

	pde = pde_from_index(vm, i);

	gk20a_mem_wr32(pde, 0, pde_v[0]);
	gk20a_mem_wr32(pde, 1, pde_v[1]);

	gk20a_dbg(gpu_dbg_pte, "pde:%d,sz=%d = 0x%x,0x%08x",
		  i, gmmu_pgsz_idx, pde_v[1], pde_v[0]);
	return 0;
}

static int update_gmmu_pte_locked(struct vm_gk20a *vm,
			   struct gk20a_mm_entry *pte,
			   u32 i, u32 gmmu_pgsz_idx,
			   struct scatterlist **sgl,
			   u64 *offset,
			   u64 *iova,
			   u32 kind_v, u64 *ctag,
			   bool cacheable, bool unmapped_pte,
			   int rw_flag, bool sparse, bool priv)
{
	struct gk20a *g = gk20a_from_vm(vm);
	u64 ctag_granularity = g->ops.fb.compression_page_size(g);
	u32 page_size  = vm->gmmu_page_sizes[gmmu_pgsz_idx];
	u32 pte_w[2] = {0, 0}; /* invalid pte */

	if (*iova) {
		if (unmapped_pte)
			pte_w[0] = gmmu_pte_valid_false_f() |
				gmmu_pte_address_sys_f(*iova
				>> gmmu_pte_address_shift_v());
		else
			pte_w[0] = gmmu_pte_valid_true_f() |
				gmmu_pte_address_sys_f(*iova
				>> gmmu_pte_address_shift_v());

		if (priv)
			pte_w[0] |= gmmu_pte_privilege_true_f();

		pte_w[1] = gmmu_pte_aperture_video_memory_f() |
			gmmu_pte_kind_f(kind_v) |
			gmmu_pte_comptagline_f((u32)(*ctag / ctag_granularity));

		if (vm->mm->use_full_comp_tag_line && *iova & 0x10000) {
			pte_w[1] |= gmmu_pte_comptagline_f(
					1 << (gmmu_pte_comptagline_s() - 1));
		}

		if (rw_flag == gk20a_mem_flag_read_only) {
			pte_w[0] |= gmmu_pte_read_only_true_f();
			pte_w[1] |=
				gmmu_pte_write_disable_true_f();
		} else if (rw_flag ==
			   gk20a_mem_flag_write_only) {
			pte_w[1] |=
				gmmu_pte_read_disable_true_f();
		}
		if (!unmapped_pte) {
			if (!cacheable)
				pte_w[1] |=
					gmmu_pte_vol_true_f();
		} else {
			/* Store cacheable value behind
			 * gmmu_pte_write_disable_true_f */
			if (!cacheable)
				pte_w[1] |=
				gmmu_pte_write_disable_true_f();
		}

		gk20a_dbg(gpu_dbg_pte,
			"pte=%d iova=0x%llx kind=%d ctag=%d vol=%d [0x%08x, 0x%08x]",
			   i, *iova,
			   kind_v, (u32)(*ctag / ctag_granularity), !cacheable,
			   pte_w[1], pte_w[0]);

		if (*ctag)
			*ctag += page_size;
	} else if (sparse) {
		pte_w[0] = gmmu_pte_valid_false_f();
		pte_w[1] |= gmmu_pte_vol_true_f();
	} else {
		gk20a_dbg(gpu_dbg_pte, "pte_cur=%d [0x0,0x0]", i);
	}

	gk20a_mem_wr32(pte->cpu_va + i*8, 0, pte_w[0]);
	gk20a_mem_wr32(pte->cpu_va + i*8, 1, pte_w[1]);

	if (*iova) {
		*iova += page_size;
		*offset += page_size;
		if (*sgl && *offset + page_size > (*sgl)->length) {
			u64 new_iova;
			*sgl = sg_next(*sgl);
			if (*sgl) {
				new_iova = sg_phys(*sgl);
				gk20a_dbg(gpu_dbg_pte, "chunk address %llx, size %d",
					  new_iova, (*sgl)->length);
				if (new_iova) {
					*offset = 0;
					*iova = new_iova;
				}
			}
		}
	}

	return 0;
}

static int update_gmmu_level_locked(struct vm_gk20a *vm,
				    struct gk20a_mm_entry *pte,
				    enum gmmu_pgsz_gk20a pgsz_idx,
				    struct scatterlist **sgl,
				    u64 *offset,
				    u64 *iova,
				    u64 gpu_va, u64 gpu_end,
				    u8 kind_v, u64 *ctag,
				    bool cacheable, bool unmapped_pte,
				    int rw_flag,
				    bool sparse,
				    int lvl,
				    bool priv)
{
	const struct gk20a_mmu_level *l = &vm->mmu_levels[lvl];
	const struct gk20a_mmu_level *next_l = &vm->mmu_levels[lvl+1];
	int err = 0;
	u32 pde_i;
	u64 pde_size = 1ULL << (u64)l->lo_bit[pgsz_idx];

	gk20a_dbg_fn("");

	pde_i = (gpu_va & ((1ULL << ((u64)l->hi_bit[pgsz_idx]+1)) - 1ULL))
		>> (u64)l->lo_bit[pgsz_idx];

	gk20a_dbg(gpu_dbg_pte, "size_idx=%d, l: %d, [%llx,%llx], iova=%llx",
		  pgsz_idx, lvl, gpu_va, gpu_end-1, *iova);

	while (gpu_va < gpu_end) {
		struct gk20a_mm_entry *next_pte = NULL;
		u64 next = min((gpu_va + pde_size) & ~(pde_size-1), gpu_end);

		/* Allocate next level */
		if (next_l->update_entry) {
			if (!pte->entries) {
				int num_entries =
					1 <<
					 (l->hi_bit[pgsz_idx]
					  - l->lo_bit[pgsz_idx] + 1);
				pte->entries =
					vzalloc(sizeof(struct gk20a_mm_entry) *
						num_entries);
				if (!pte->entries)
					return -ENOMEM;
				pte->pgsz = pgsz_idx;
				pte->num_entries = num_entries;
			}
			next_pte = pte->entries + pde_i;

			if (!next_pte->size) {
				err = gk20a_zalloc_gmmu_page_table(vm,
					pgsz_idx, next_l, next_pte);
				if (err)
					return err;
			}
		}

		err = l->update_entry(vm, pte, pde_i, pgsz_idx,
				sgl, offset, iova,
				kind_v, ctag, cacheable, unmapped_pte,
				rw_flag, sparse, priv);
		if (err)
			return err;

		if (next_l->update_entry) {
			/* get cpu access to the ptes */
			err = map_gmmu_pages(next_pte);
			if (err) {
				gk20a_err(dev_from_vm(vm),
					   "couldn't map ptes for update as=%d",
					   vm_aspace_id(vm));
				return err;
			}
			err = update_gmmu_level_locked(vm, next_pte,
				pgsz_idx,
				sgl,
				offset,
				iova,
				gpu_va,
				next,
				kind_v, ctag, cacheable, unmapped_pte,
				rw_flag, sparse, lvl+1, priv);
			unmap_gmmu_pages(next_pte);

			if (err)
				return err;
		}

		pde_i++;
		gpu_va = next;
	}

	gk20a_dbg_fn("done");

	return 0;
}

static int update_gmmu_ptes_locked(struct vm_gk20a *vm,
				   enum gmmu_pgsz_gk20a pgsz_idx,
				   struct sg_table *sgt,
				   u64 buffer_offset,
				   u64 gpu_va, u64 gpu_end,
				   u8 kind_v, u32 ctag_offset,
				   bool cacheable, bool unmapped_pte,
				   int rw_flag,
				   bool sparse,
				   bool priv)
{
	struct gk20a *g = gk20a_from_vm(vm);
	int ctag_granularity = g->ops.fb.compression_page_size(g);
	u64 ctag = (u64)ctag_offset * (u64)ctag_granularity;
	u64 iova = 0;
	u64 space_to_skip = buffer_offset;
	u32 page_size  = vm->gmmu_page_sizes[pgsz_idx];
	int err;
	struct scatterlist *sgl = NULL;

	gk20a_dbg(gpu_dbg_pte, "size_idx=%d, iova=%llx, buffer offset %lld, nents %d",
		   pgsz_idx,
		   sgt ? g->ops.mm.get_iova_addr(vm->mm->g, sgt->sgl, 0)
		       : 0ULL,
		   buffer_offset,
		   sgt ? sgt->nents : 0);

	/* note: here we need to map kernel to small, since the
	 * low-level mmu code assumes 0 is small and 1 is big pages */
	if (pgsz_idx == gmmu_page_size_kernel)
		pgsz_idx = gmmu_page_size_small;

	if (space_to_skip & (page_size - 1))
		return -EINVAL;

	if (sgt) {
		iova = g->ops.mm.get_iova_addr(vm->mm->g, sgt->sgl, 0);
		if (!vm->mm->bypass_smmu && iova) {
			iova += space_to_skip;
		} else {
			sgl = sgt->sgl;

			gk20a_dbg(gpu_dbg_pte, "chunk address %llx, size %d",
					(u64)sg_phys(sgl),
					sgl->length);
			while (space_to_skip && sgl &&
			       space_to_skip + page_size > sgl->length) {
				space_to_skip -= sgl->length;
				sgl = sg_next(sgl);
				gk20a_dbg(gpu_dbg_pte, "chunk address %llx, size %d",
						(u64)sg_phys(sgl),
						sgl->length);
			}
			iova = sg_phys(sgl) + space_to_skip;
		}
	}

	gk20a_dbg(gpu_dbg_map, "size_idx=%d, gpu_va=[%llx,%llx], iova=%llx",
			pgsz_idx, gpu_va, gpu_end-1, iova);
	err = map_gmmu_pages(&vm->pdb);
	if (err) {
		gk20a_err(dev_from_vm(vm),
			   "couldn't map ptes for update as=%d",
			   vm_aspace_id(vm));
		return err;
	}
	err = update_gmmu_level_locked(vm, &vm->pdb, pgsz_idx,
			&sgl,
			&space_to_skip,
			&iova,
			gpu_va, gpu_end,
			kind_v, &ctag,
			cacheable, unmapped_pte, rw_flag, sparse, 0, priv);
	unmap_gmmu_pages(&vm->pdb);

	smp_mb();

	gk20a_dbg_fn("done");

	return err;
}

/* NOTE! mapped_buffers lock must be held */
void gk20a_vm_unmap_locked(struct mapped_buffer_node *mapped_buffer,
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
		mapped_buffer->va_node ?
		  mapped_buffer->va_node->sparse : false,
		batch);

	gk20a_dbg(gpu_dbg_map, "as=%d pgsz=%d gv=0x%x,%08x own_mem_ref=%d",
		   vm_aspace_id(vm),
		   vm->gmmu_page_sizes[mapped_buffer->pgsz_idx],
		   hi32(mapped_buffer->addr), lo32(mapped_buffer->addr),
		   mapped_buffer->own_mem_ref);

	gk20a_mm_unpin(dev_from_vm(vm), mapped_buffer->dmabuf,
		       mapped_buffer->sgt);

	/* remove from mapped buffer tree and remove list, free */
	rb_erase(&mapped_buffer->node, &vm->mapped_buffers);
	if (!list_empty(&mapped_buffer->va_buffers_list))
		list_del(&mapped_buffer->va_buffers_list);

	/* keep track of mapped buffers */
	if (mapped_buffer->user_mapped)
		vm->num_user_mapped_buffers--;

	if (mapped_buffer->own_mem_ref)
		dma_buf_put(mapped_buffer->dmabuf);

	kfree(mapped_buffer);

	return;
}

void gk20a_vm_unmap(struct vm_gk20a *vm, u64 offset)
{
	struct device *d = dev_from_vm(vm);
	struct mapped_buffer_node *mapped_buffer;

	mutex_lock(&vm->update_gmmu_lock);
	mapped_buffer = find_mapped_buffer_locked(&vm->mapped_buffers, offset);
	if (!mapped_buffer) {
		mutex_unlock(&vm->update_gmmu_lock);
		gk20a_err(d, "invalid addr to unmap 0x%llx", offset);
		return;
	}

	kref_put(&mapped_buffer->ref, gk20a_vm_unmap_locked_kref);
	mutex_unlock(&vm->update_gmmu_lock);
}

static void gk20a_vm_free_entries(struct vm_gk20a *vm,
				  struct gk20a_mm_entry *parent,
				  int level)
{
	int i;

	if (parent->entries)
		for (i = 0; i < parent->num_entries; i++)
			gk20a_vm_free_entries(vm, &parent->entries[i], level+1);

	if (parent->size)
		free_gmmu_pages(vm, parent);
	vfree(parent->entries);
	parent->entries = NULL;
}

static void gk20a_vm_remove_support_nofree(struct vm_gk20a *vm)
{
	struct mapped_buffer_node *mapped_buffer;
	struct vm_reserved_va_node *va_node, *va_node_tmp;
	struct rb_node *node;

	gk20a_dbg_fn("");
	mutex_lock(&vm->update_gmmu_lock);

	/* TBD: add a flag here for the unmap code to recognize teardown
	 * and short-circuit any otherwise expensive operations. */

	node = rb_first(&vm->mapped_buffers);
	while (node) {
		mapped_buffer =
			container_of(node, struct mapped_buffer_node, node);
		gk20a_vm_unmap_locked(mapped_buffer, NULL);
		node = rb_first(&vm->mapped_buffers);
	}

	/* destroy remaining reserved memory areas */
	list_for_each_entry_safe(va_node, va_node_tmp, &vm->reserved_va_list,
		reserved_va_list) {
		list_del(&va_node->reserved_va_list);
		kfree(va_node);
	}

	gk20a_deinit_vm(vm);

	mutex_unlock(&vm->update_gmmu_lock);
}

void gk20a_vm_remove_support(struct vm_gk20a *vm)
{
	gk20a_vm_remove_support_nofree(vm);
	/* vm is not used anymore. release it. */
	kfree(vm);
}

static void gk20a_vm_remove_support_kref(struct kref *ref)
{
	struct vm_gk20a *vm = container_of(ref, struct vm_gk20a, ref);
	struct gk20a *g = gk20a_from_vm(vm);
	g->ops.mm.vm_remove(vm);
}

void gk20a_vm_get(struct vm_gk20a *vm)
{
	kref_get(&vm->ref);
}

void gk20a_vm_put(struct vm_gk20a *vm)
{
	kref_put(&vm->ref, gk20a_vm_remove_support_kref);
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

int gk20a_init_vm(struct mm_gk20a *mm,
		struct vm_gk20a *vm,
		u32 big_page_size,
		u64 low_hole,
		u64 kernel_reserved,
		u64 aperture_size,
		bool big_pages,
		bool userspace_managed,
		char *name)
{
	int err, i;
	char alloc_name[32];
	u64 small_vma_start, small_vma_limit, large_vma_start, large_vma_limit,
		kernel_vma_start, kernel_vma_limit;
	u32 pde_lo, pde_hi;

	/* note: this must match gmmu_pgsz_gk20a enum */
	u32 gmmu_page_sizes[gmmu_nr_page_sizes] = { SZ_4K, big_page_size, SZ_4K };

	WARN_ON(kernel_reserved + low_hole > aperture_size);
	if (kernel_reserved > aperture_size)
		return -ENOMEM;

	vm->mm = mm;

	vm->va_start = low_hole;
	vm->va_limit = aperture_size;
	vm->big_pages = big_pages;

	vm->big_page_size = gmmu_page_sizes[gmmu_page_size_big];

	vm->userspace_managed = userspace_managed;

	vm->mmu_levels = vm->mm->g->ops.mm.get_mmu_levels(vm->mm->g,
			vm->big_page_size);

	for (i = 0; i < gmmu_nr_page_sizes; i++)
		vm->gmmu_page_sizes[i] = gmmu_page_sizes[i];

	gk20a_dbg_info("small page-size (%dKB)",
			vm->gmmu_page_sizes[gmmu_page_size_small] >> 10);

	gk20a_dbg_info("big page-size (%dKB)",
			vm->gmmu_page_sizes[gmmu_page_size_big] >> 10);

	gk20a_dbg_info("kernel page-size (%dKB)",
			vm->gmmu_page_sizes[gmmu_page_size_kernel] >> 10);

	pde_range_from_vaddr_range(vm,
				   0, vm->va_limit-1,
				   &pde_lo, &pde_hi);
	vm->pdb.entries = vzalloc(sizeof(struct gk20a_mm_entry) *
			(pde_hi + 1));
	vm->pdb.num_entries = pde_hi + 1;

	if (!vm->pdb.entries)
		return -ENOMEM;

	gk20a_dbg_info("init space for %s va_limit=0x%llx num_pdes=%d",
		   name, vm->va_limit, pde_hi + 1);

	/* allocate the page table directory */
	err = gk20a_zalloc_gmmu_page_table(vm, 0, &vm->mmu_levels[0], &vm->pdb);
	if (err)
		goto clean_up_pdes;

	/* setup vma limits */
	small_vma_start = low_hole;

	if (big_pages) {
		/* First 16GB of the address space goes towards small
		 * pages. What ever remains is allocated to large
		 * pages. */
		small_vma_limit = __nv_gmmu_va_small_page_limit();
		large_vma_start = small_vma_limit;
		large_vma_limit = vm->va_limit - kernel_reserved;
	} else {
		small_vma_limit = vm->va_limit - kernel_reserved;
		large_vma_start = 0;
		large_vma_limit = 0;
	}

	kernel_vma_start = vm->va_limit - kernel_reserved;
	kernel_vma_limit = vm->va_limit;

	gk20a_dbg_info(
		"small_vma=[0x%llx,0x%llx) large_vma=[0x%llx,0x%llx) kernel_vma=[0x%llx,0x%llx)\n",
		small_vma_start, small_vma_limit,
		large_vma_start, large_vma_limit,
		kernel_vma_start, kernel_vma_limit);

	/* check that starts do not exceed limits */
	WARN_ON(small_vma_start > small_vma_limit);
	WARN_ON(large_vma_start > large_vma_limit);
	/* kernel_vma must also be non-zero */
	WARN_ON(kernel_vma_start >= kernel_vma_limit);

	if (small_vma_start > small_vma_limit ||
	    large_vma_start > large_vma_limit ||
	    kernel_vma_start >= kernel_vma_limit) {
		err = -EINVAL;
		goto clean_up_pdes;
	}

	if (small_vma_start < small_vma_limit) {
		snprintf(alloc_name, sizeof(alloc_name), "gk20a_%s-%dKB", name,
			 vm->gmmu_page_sizes[gmmu_page_size_small] >> 10);
		err = __gk20a_allocator_init(&vm->vma[gmmu_page_size_small],
					     vm, alloc_name,
					     small_vma_start,
					     small_vma_limit - small_vma_start,
					     SZ_4K,
					     GPU_BALLOC_MAX_ORDER,
					     GPU_BALLOC_GVA_SPACE);
		if (err)
			goto clean_up_ptes;
	}

	if (large_vma_start < large_vma_limit) {
		snprintf(alloc_name, sizeof(alloc_name), "gk20a_%s-%dKB",
			 name, vm->gmmu_page_sizes[gmmu_page_size_big] >> 10);
		err = __gk20a_allocator_init(&vm->vma[gmmu_page_size_big],
					     vm, alloc_name,
					     large_vma_start,
					     large_vma_limit - large_vma_start,
					     big_page_size,
					     GPU_BALLOC_MAX_ORDER,
					     GPU_BALLOC_GVA_SPACE);
		if (err)
			goto clean_up_small_allocator;
	}

	snprintf(alloc_name, sizeof(alloc_name), "gk20a_%s-%dKB-sys",
		 name, vm->gmmu_page_sizes[gmmu_page_size_kernel] >> 10);
	/*
	 * kernel reserved VMA is at the end of the aperture
	 */
	err = __gk20a_allocator_init(&vm->vma[gmmu_page_size_kernel],
				     vm, alloc_name,
				     kernel_vma_start,
				     kernel_vma_limit - kernel_vma_start,
				     SZ_4K,
				     GPU_BALLOC_MAX_ORDER,
				     GPU_BALLOC_GVA_SPACE);
	if (err)
		goto clean_up_big_allocator;

	vm->mapped_buffers = RB_ROOT;

	mutex_init(&vm->update_gmmu_lock);
	kref_init(&vm->ref);
	INIT_LIST_HEAD(&vm->reserved_va_list);

	return 0;

clean_up_big_allocator:
	if (large_vma_start < large_vma_limit)
		gk20a_allocator_destroy(&vm->vma[gmmu_page_size_big]);
clean_up_small_allocator:
	if (small_vma_start < small_vma_limit)
		gk20a_allocator_destroy(&vm->vma[gmmu_page_size_small]);
clean_up_ptes:
	free_gmmu_pages(vm, &vm->pdb);
clean_up_pdes:
	vfree(vm->pdb.entries);
	return err;
}

/* address space interfaces for the gk20a module */
int gk20a_vm_alloc_share(struct gk20a_as_share *as_share, u32 big_page_size,
			 u32 flags)
{
	struct gk20a_as *as = as_share->as;
	struct gk20a *g = gk20a_from_as(as);
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm;
	char name[32];
	int err;
	const bool userspace_managed =
		(flags & NVGPU_GPU_IOCTL_ALLOC_AS_FLAGS_USERSPACE_MANAGED) != 0;

	gk20a_dbg_fn("");

	if (big_page_size == 0) {
		big_page_size =
			gk20a_get_platform(g->dev)->default_big_page_size;
	} else {
		if (!is_power_of_2(big_page_size))
			return -EINVAL;

		if (!(big_page_size & g->gpu_characteristics.available_big_page_sizes))
			return -EINVAL;
	}

	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return -ENOMEM;

	as_share->vm = vm;
	vm->as_share = as_share;
	vm->enable_ctag = true;

	snprintf(name, sizeof(name), "gk20a_as_%d", as_share->id);

	err = gk20a_init_vm(mm, vm, big_page_size, big_page_size << 10,
			    mm->channel.kernel_size,
			    mm->channel.user_size + mm->channel.kernel_size,
			    !mm->disable_bigpage, userspace_managed, name);

	return err;
}

int gk20a_vm_release_share(struct gk20a_as_share *as_share)
{
	struct vm_gk20a *vm = as_share->vm;

	gk20a_dbg_fn("");

	vm->as_share = NULL;

	/* put as reference to vm */
	gk20a_vm_put(vm);

	as_share->vm = NULL;

	return 0;
}


int gk20a_vm_alloc_space(struct gk20a_as_share *as_share,
			 struct nvgpu_as_alloc_space_args *args)

{
	int err = -ENOMEM;
	int pgsz_idx = gmmu_page_size_small;
	struct gk20a_allocator *vma;
	struct vm_gk20a *vm = as_share->vm;
	struct gk20a *g = vm->mm->g;
	struct vm_reserved_va_node *va_node;
	u64 vaddr_start = 0;
	int page_sizes = gmmu_nr_page_sizes;

	gk20a_dbg_fn("flags=0x%x pgsz=0x%x nr_pages=0x%x o/a=0x%llx",
			args->flags, args->page_size, args->pages,
			args->o_a.offset);

	if (!vm->big_pages)
		page_sizes--;

	for (; pgsz_idx < page_sizes; pgsz_idx++) {
		if (vm->gmmu_page_sizes[pgsz_idx] == args->page_size)
			break;
	}

	if (pgsz_idx >= page_sizes) {
		err = -EINVAL;
		goto clean_up;
	}

	va_node = kzalloc(sizeof(*va_node), GFP_KERNEL);
	if (!va_node) {
		err = -ENOMEM;
		goto clean_up;
	}

	vma = &vm->vma[pgsz_idx];
	if (args->flags & NVGPU_AS_ALLOC_SPACE_FLAGS_FIXED_OFFSET)
		vaddr_start = gk20a_balloc_fixed(vma, args->o_a.offset,
						 (u64)args->pages *
						 (u64)args->page_size);
	else
		vaddr_start = gk20a_balloc(vma,
					   (u64)args->pages *
					   (u64)args->page_size);

	if (!vaddr_start) {
		kfree(va_node);
		goto clean_up;
	}

	va_node->vaddr_start = vaddr_start;
	va_node->size = (u64)args->page_size * (u64)args->pages;
	va_node->pgsz_idx = pgsz_idx;
	INIT_LIST_HEAD(&va_node->va_buffers_list);
	INIT_LIST_HEAD(&va_node->reserved_va_list);

	mutex_lock(&vm->update_gmmu_lock);

	/* mark that we need to use sparse mappings here */
	if (args->flags & NVGPU_AS_ALLOC_SPACE_FLAGS_SPARSE) {
		u64 map_offset = g->ops.mm.gmmu_map(vm, vaddr_start,
					 NULL,
					 0,
					 va_node->size,
					 pgsz_idx,
					 0,
					 0,
					 args->flags,
					 gk20a_mem_flag_none,
					 false,
					 true,
					 false,
					 NULL);
		if (!map_offset) {
			mutex_unlock(&vm->update_gmmu_lock);
			gk20a_bfree(vma, vaddr_start);
			kfree(va_node);
			goto clean_up;
		}

		va_node->sparse = true;
	}
	list_add_tail(&va_node->reserved_va_list, &vm->reserved_va_list);

	mutex_unlock(&vm->update_gmmu_lock);

	args->o_a.offset = vaddr_start;
	err = 0;

clean_up:
	return err;
}

int gk20a_vm_free_space(struct gk20a_as_share *as_share,
			struct nvgpu_as_free_space_args *args)
{
	int err = -ENOMEM;
	int pgsz_idx;
	struct gk20a_allocator *vma;
	struct vm_gk20a *vm = as_share->vm;
	struct vm_reserved_va_node *va_node;
	struct gk20a *g = gk20a_from_vm(vm);

	gk20a_dbg_fn("pgsz=0x%x nr_pages=0x%x o/a=0x%llx", args->page_size,
			args->pages, args->offset);

	/* determine pagesz idx */
	pgsz_idx = __nv_gmmu_va_is_big_page_region(vm, args->offset) ?
			gmmu_page_size_big : gmmu_page_size_small;

	vma = &vm->vma[pgsz_idx];
	gk20a_bfree(vma, args->offset);

	mutex_lock(&vm->update_gmmu_lock);
	va_node = addr_to_reservation(vm, args->offset);
	if (va_node) {
		struct mapped_buffer_node *buffer, *n;

		/* Decrement the ref count on all buffers in this va_node. This
		 * allows userspace to let the kernel free mappings that are
		 * only used by this va_node. */
		list_for_each_entry_safe(buffer, n,
			&va_node->va_buffers_list, va_buffers_list) {
			list_del_init(&buffer->va_buffers_list);
			kref_put(&buffer->ref, gk20a_vm_unmap_locked_kref);
		}

		list_del(&va_node->reserved_va_list);

		/* if this was a sparse mapping, free the va */
		if (va_node->sparse)
			g->ops.mm.gmmu_unmap(vm,
					va_node->vaddr_start,
					va_node->size,
					va_node->pgsz_idx,
					true,
					gk20a_mem_flag_none,
					true,
					NULL);
		kfree(va_node);
	}
	mutex_unlock(&vm->update_gmmu_lock);
	err = 0;

	return err;
}

int gk20a_vm_bind_channel(struct gk20a_as_share *as_share,
			  struct channel_gk20a *ch)
{
	int err = 0;
	struct vm_gk20a *vm = as_share->vm;

	gk20a_dbg_fn("");

	ch->vm = vm;
	err = channel_gk20a_commit_va(ch);
	if (err)
		ch->vm = NULL;

	return err;
}

int gk20a_dmabuf_alloc_drvdata(struct dma_buf *dmabuf, struct device *dev)
{
	struct gk20a_dmabuf_priv *priv;
	static DEFINE_MUTEX(priv_lock);
	static u64 priv_count = 0;

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (likely(priv))
		return 0;

	mutex_lock(&priv_lock);
	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (priv)
		goto priv_exist_or_err;
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		priv = ERR_PTR(-ENOMEM);
		goto priv_exist_or_err;
	}
	mutex_init(&priv->lock);
	INIT_LIST_HEAD(&priv->states);
	priv->buffer_id = ++priv_count;
	dma_buf_set_drvdata(dmabuf, dev, priv, gk20a_mm_delete_priv);
priv_exist_or_err:
	mutex_unlock(&priv_lock);
	if (IS_ERR(priv))
		return -ENOMEM;

	return 0;
}

int gk20a_dmabuf_get_state(struct dma_buf *dmabuf, struct device *dev,
			   u64 offset, struct gk20a_buffer_state **state)
{
	int err = 0;
	struct gk20a_dmabuf_priv *priv;
	struct gk20a_buffer_state *s;

	if (WARN_ON(offset >= (u64)dmabuf->size))
		return -EINVAL;

	err = gk20a_dmabuf_alloc_drvdata(dmabuf, dev);
	if (err)
		return err;

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (WARN_ON(!priv))
		return -ENOSYS;

	mutex_lock(&priv->lock);

	list_for_each_entry(s, &priv->states, list)
		if (s->offset == offset)
			goto out;

	/* State not found, create state. */
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		err = -ENOMEM;
		goto out;
	}

	s->offset = offset;
	INIT_LIST_HEAD(&s->list);
	mutex_init(&s->lock);
	list_add_tail(&s->list, &priv->states);

out:
	mutex_unlock(&priv->lock);
	if (!err)
		*state = s;
	return err;


}

int gk20a_vm_map_buffer(struct vm_gk20a *vm,
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
		dev_warn(dev_from_vm(vm), "%s: fd %d is not a dmabuf",
			 __func__, dmabuf_fd);
		return PTR_ERR(dmabuf);
	}

	err = gk20a_dmabuf_alloc_drvdata(dmabuf, dev_from_vm(vm));
	if (err) {
		dma_buf_put(dmabuf);
		return err;
	}

	ret_va = gk20a_vm_map(vm, dmabuf, *offset_align,
			flags, kind, NULL, true,
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

int gk20a_vm_unmap_buffer(struct vm_gk20a *vm, u64 offset,
			  struct vm_gk20a_mapping_batch *batch)
{
	gk20a_dbg_fn("");

	gk20a_vm_unmap_user(vm, offset, batch);
	return 0;
}

void gk20a_deinit_vm(struct vm_gk20a *vm)
{
	gk20a_allocator_destroy(&vm->vma[gmmu_page_size_kernel]);
	if (vm->vma[gmmu_page_size_big].init)
		gk20a_allocator_destroy(&vm->vma[gmmu_page_size_big]);
	if (vm->vma[gmmu_page_size_small].init)
		gk20a_allocator_destroy(&vm->vma[gmmu_page_size_small]);

	gk20a_vm_free_entries(vm, &vm->pdb, 0);
}

int gk20a_alloc_inst_block(struct gk20a *g, struct mem_desc *inst_block)
{
	struct device *dev = dev_from_gk20a(g);
	int err;

	gk20a_dbg_fn("");

	err = gk20a_gmmu_alloc(g, ram_in_alloc_size_v(), inst_block);
	if (err) {
		gk20a_err(dev, "%s: memory allocation failed\n", __func__);
		return err;
	}

	gk20a_dbg_fn("done");
	return 0;
}

void gk20a_free_inst_block(struct gk20a *g, struct mem_desc *inst_block)
{
	if (inst_block->cpu_va)
		gk20a_gmmu_free(g, inst_block);
}

static int gk20a_init_bar1_vm(struct mm_gk20a *mm)
{
	int err;
	struct vm_gk20a *vm = &mm->bar1.vm;
	struct gk20a *g = gk20a_from_mm(mm);
	struct mem_desc *inst_block = &mm->bar1.inst_block;
	u32 big_page_size = gk20a_get_platform(g->dev)->default_big_page_size;

	mm->bar1.aperture_size = bar1_aperture_size_mb_gk20a() << 20;
	gk20a_dbg_info("bar1 vm size = 0x%x", mm->bar1.aperture_size);
	gk20a_init_vm(mm, vm, big_page_size, SZ_4K,
		      mm->bar1.aperture_size - SZ_4K,
		      mm->bar1.aperture_size, false, false, "bar1");

	err = gk20a_alloc_inst_block(g, inst_block);
	if (err)
		goto clean_up_va;
	gk20a_init_inst_block(inst_block, vm, big_page_size);

	return 0;

clean_up_va:
	gk20a_deinit_vm(vm);
	return err;
}

/* pmu vm, share channel_vm interfaces */
static int gk20a_init_system_vm(struct mm_gk20a *mm)
{
	int err;
	struct vm_gk20a *vm = &mm->pmu.vm;
	struct gk20a *g = gk20a_from_mm(mm);
	struct mem_desc *inst_block = &mm->pmu.inst_block;
	u32 big_page_size = gk20a_get_platform(g->dev)->default_big_page_size;

	mm->pmu.aperture_size = GK20A_PMU_VA_SIZE;
	gk20a_dbg_info("pmu vm size = 0x%x", mm->pmu.aperture_size);

	gk20a_init_vm(mm, vm, big_page_size,
		      SZ_4K * 16, GK20A_PMU_VA_SIZE,
		      GK20A_PMU_VA_SIZE * 2, false, false,
		      "system");

	err = gk20a_alloc_inst_block(g, inst_block);
	if (err)
		goto clean_up_va;
	gk20a_init_inst_block(inst_block, vm, big_page_size);

	return 0;

clean_up_va:
	gk20a_deinit_vm(vm);
	return err;
}

static int gk20a_init_hwpm(struct mm_gk20a *mm)
{
	int err;
	struct vm_gk20a *vm = &mm->pmu.vm;
	struct gk20a *g = gk20a_from_mm(mm);
	struct mem_desc *inst_block = &mm->hwpm.inst_block;

	err = gk20a_alloc_inst_block(g, inst_block);
	if (err)
		return err;
	gk20a_init_inst_block(inst_block, vm, 0);

	return 0;
}

static int gk20a_init_cde_vm(struct mm_gk20a *mm)
{
	struct vm_gk20a *vm = &mm->cde.vm;
	struct gk20a *g = gk20a_from_mm(mm);
	u32 big_page_size = gk20a_get_platform(g->dev)->default_big_page_size;

	return gk20a_init_vm(mm, vm, big_page_size,
			SZ_4K * 16,
			NV_MM_DEFAULT_KERNEL_SIZE,
			NV_MM_DEFAULT_KERNEL_SIZE + NV_MM_DEFAULT_USER_SIZE,
			false, false, "cde");
}

void gk20a_mm_init_pdb(struct gk20a *g, void *inst_ptr, u64 pdb_addr)
{
	u32 pdb_addr_lo = u64_lo32(pdb_addr >> ram_in_base_shift_v());
	u32 pdb_addr_hi = u64_hi32(pdb_addr);

	gk20a_mem_wr32(inst_ptr, ram_in_page_dir_base_lo_w(),
		ram_in_page_dir_base_target_vid_mem_f() |
		ram_in_page_dir_base_vol_true_f() |
		ram_in_page_dir_base_lo_f(pdb_addr_lo));

	gk20a_mem_wr32(inst_ptr, ram_in_page_dir_base_hi_w(),
		ram_in_page_dir_base_hi_f(pdb_addr_hi));
}

void gk20a_init_inst_block(struct mem_desc *inst_block, struct vm_gk20a *vm,
		u32 big_page_size)
{
	struct gk20a *g = gk20a_from_vm(vm);
	u64 pde_addr = g->ops.mm.get_iova_addr(g, vm->pdb.sgt->sgl, 0);
	phys_addr_t inst_pa = gk20a_mem_phys(inst_block);
	void *inst_ptr = inst_block->cpu_va;

	gk20a_dbg_info("inst block phys = 0x%llx, kv = 0x%p",
		(u64)inst_pa, inst_ptr);

	gk20a_dbg_info("pde pa=0x%llx", (u64)pde_addr);

	g->ops.mm.init_pdb(g, inst_ptr, pde_addr);

	gk20a_mem_wr32(inst_ptr, ram_in_adr_limit_lo_w(),
		u64_lo32(vm->va_limit - 1) & ~0xfff);

	gk20a_mem_wr32(inst_ptr, ram_in_adr_limit_hi_w(),
		ram_in_adr_limit_hi_f(u64_hi32(vm->va_limit - 1)));

	if (big_page_size && g->ops.mm.set_big_page_size)
		g->ops.mm.set_big_page_size(g, inst_ptr, big_page_size);
}

int gk20a_mm_fb_flush(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	u32 data;
	s32 retry = 100;
	int ret = 0;

	gk20a_dbg_fn("");

	gk20a_busy_noresume(g->dev);
	if (!g->power_on) {
		pm_runtime_put_noidle(&g->dev->dev);
		return 0;
	}

	mutex_lock(&mm->l2_op_lock);

	/* Make sure all previous writes are committed to the L2. There's no
	   guarantee that writes are to DRAM. This will be a sysmembar internal
	   to the L2. */

	trace_gk20a_mm_fb_flush(g->dev->name);

	gk20a_writel(g, flush_fb_flush_r(),
		flush_fb_flush_pending_busy_f());

	do {
		data = gk20a_readl(g, flush_fb_flush_r());

		if (flush_fb_flush_outstanding_v(data) ==
			flush_fb_flush_outstanding_true_v() ||
		    flush_fb_flush_pending_v(data) ==
			flush_fb_flush_pending_busy_v()) {
				gk20a_dbg_info("fb_flush 0x%x", data);
				retry--;
				udelay(5);
		} else
			break;
	} while (retry >= 0 || !tegra_platform_is_silicon());

	if (tegra_platform_is_silicon() && retry < 0) {
		gk20a_warn(dev_from_gk20a(g),
			"fb_flush too many retries");
		if (g->ops.fb.dump_vpr_wpr_info)
			g->ops.fb.dump_vpr_wpr_info(g);
		ret = -EBUSY;
	}

	trace_gk20a_mm_fb_flush_done(g->dev->name);

	mutex_unlock(&mm->l2_op_lock);

	pm_runtime_put_noidle(&g->dev->dev);

	return ret;
}

static void gk20a_mm_l2_invalidate_locked(struct gk20a *g)
{
	u32 data;
	s32 retry = 200;

	trace_gk20a_mm_l2_invalidate(g->dev->name);

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
				retry--;
				udelay(5);
		} else
			break;
	} while (retry >= 0 || !tegra_platform_is_silicon());

	if (tegra_platform_is_silicon() && retry < 0)
		gk20a_warn(dev_from_gk20a(g),
			"l2_system_invalidate too many retries");

	trace_gk20a_mm_l2_invalidate_done(g->dev->name);
}

void gk20a_mm_l2_invalidate(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	gk20a_busy_noresume(g->dev);
	if (g->power_on) {
		mutex_lock(&mm->l2_op_lock);
		gk20a_mm_l2_invalidate_locked(g);
		mutex_unlock(&mm->l2_op_lock);
	}
	pm_runtime_put_noidle(&g->dev->dev);
}

void gk20a_mm_l2_flush(struct gk20a *g, bool invalidate)
{
	struct mm_gk20a *mm = &g->mm;
	u32 data;
	s32 retry = 200;

	gk20a_dbg_fn("");

	gk20a_busy_noresume(g->dev);
	if (!g->power_on)
		goto hw_was_off;

	mutex_lock(&mm->l2_op_lock);

	trace_gk20a_mm_l2_flush(g->dev->name);

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
				retry--;
				udelay(5);
		} else
			break;
	} while (retry >= 0 || !tegra_platform_is_silicon());

	if (tegra_platform_is_silicon() && retry < 0)
		gk20a_warn(dev_from_gk20a(g),
			"l2_flush_dirty too many retries");

	trace_gk20a_mm_l2_flush_done(g->dev->name);

	if (invalidate)
		gk20a_mm_l2_invalidate_locked(g);

	mutex_unlock(&mm->l2_op_lock);

hw_was_off:
	pm_runtime_put_noidle(&g->dev->dev);
}

void gk20a_mm_cbc_clean(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	u32 data;
	s32 retry = 200;

	gk20a_dbg_fn("");

	gk20a_busy_noresume(g->dev);
	if (!g->power_on)
		goto hw_was_off;

	mutex_lock(&mm->l2_op_lock);

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
				retry--;
				udelay(5);
		} else
			break;
	} while (retry >= 0 || !tegra_platform_is_silicon());

	if (tegra_platform_is_silicon() && retry < 0)
		gk20a_warn(dev_from_gk20a(g),
			"l2_clean_comptags too many retries");

	mutex_unlock(&mm->l2_op_lock);

hw_was_off:
	pm_runtime_put_noidle(&g->dev->dev);
}

int gk20a_vm_find_buffer(struct vm_gk20a *vm, u64 gpu_va,
			 struct dma_buf **dmabuf,
			 u64 *offset)
{
	struct mapped_buffer_node *mapped_buffer;

	gk20a_dbg_fn("gpu_va=0x%llx", gpu_va);

	mutex_lock(&vm->update_gmmu_lock);

	mapped_buffer = find_mapped_buffer_range_locked(&vm->mapped_buffers,
							gpu_va);
	if (!mapped_buffer) {
		mutex_unlock(&vm->update_gmmu_lock);
		return -EINVAL;
	}

	*dmabuf = mapped_buffer->dmabuf;
	*offset = gpu_va - mapped_buffer->addr;

	mutex_unlock(&vm->update_gmmu_lock);

	return 0;
}

void gk20a_mm_tlb_invalidate(struct vm_gk20a *vm)
{
	struct gk20a *g = gk20a_from_vm(vm);
	u32 addr_lo = u64_lo32(g->ops.mm.get_iova_addr(vm->mm->g,
						  vm->pdb.sgt->sgl, 0) >> 12);
	u32 data;
	s32 retry = 2000;
	static DEFINE_MUTEX(tlb_lock);

	gk20a_dbg_fn("");

	/* pagetables are considered sw states which are preserved after
	   prepare_poweroff. When gk20a deinit releases those pagetables,
	   common code in vm unmap path calls tlb invalidate that touches
	   hw. Use the power_on flag to skip tlb invalidation when gpu
	   power is turned off */

	if (!g->power_on)
		return;

	mutex_lock(&tlb_lock);

	trace_gk20a_mm_tlb_invalidate(g->dev->name);

	do {
		data = gk20a_readl(g, fb_mmu_ctrl_r());
		if (fb_mmu_ctrl_pri_fifo_space_v(data) != 0)
			break;
		udelay(2);
		retry--;
	} while (retry >= 0 || !tegra_platform_is_silicon());

	if (tegra_platform_is_silicon() && retry < 0) {
		gk20a_warn(dev_from_gk20a(g),
			"wait mmu fifo space too many retries");
		goto out;
	}

	gk20a_writel(g, fb_mmu_invalidate_pdb_r(),
		fb_mmu_invalidate_pdb_addr_f(addr_lo) |
		fb_mmu_invalidate_pdb_aperture_vid_mem_f());

	gk20a_writel(g, fb_mmu_invalidate_r(),
		fb_mmu_invalidate_all_va_true_f() |
		fb_mmu_invalidate_trigger_true_f());

	do {
		data = gk20a_readl(g, fb_mmu_ctrl_r());
		if (fb_mmu_ctrl_pri_fifo_empty_v(data) !=
			fb_mmu_ctrl_pri_fifo_empty_false_f())
			break;
		retry--;
		udelay(2);
	} while (retry >= 0 || !tegra_platform_is_silicon());

	if (tegra_platform_is_silicon() && retry < 0)
		gk20a_warn(dev_from_gk20a(g),
			"mmu invalidate too many retries");

	trace_gk20a_mm_tlb_invalidate_done(g->dev->name);

out:
	mutex_unlock(&tlb_lock);
}

int gk20a_mm_suspend(struct gk20a *g)
{
	gk20a_dbg_fn("");

	g->ops.mm.cbc_clean(g);
	g->ops.mm.l2_flush(g, false);

	gk20a_dbg_fn("done");
	return 0;
}

bool gk20a_mm_mmu_debug_mode_enabled(struct gk20a *g)
{
	u32 debug_ctrl = gk20a_readl(g, fb_mmu_debug_ctrl_r());
	return fb_mmu_debug_ctrl_debug_v(debug_ctrl) ==
		fb_mmu_debug_ctrl_debug_enabled_v();
}

static void gk20a_mm_mmu_set_debug_mode(struct gk20a *g, bool enable)
{
	u32 reg_val, debug_ctrl;

	reg_val = gk20a_readl(g, fb_mmu_debug_ctrl_r());
	if (enable) {
		debug_ctrl = fb_mmu_debug_ctrl_debug_enabled_f();
		g->mmu_debug_ctrl = true;
	} else {
		debug_ctrl = fb_mmu_debug_ctrl_debug_disabled_f();
		g->mmu_debug_ctrl = false;
	}

	reg_val = set_field(reg_val,
				fb_mmu_debug_ctrl_debug_m(), debug_ctrl);
	gk20a_writel(g, fb_mmu_debug_ctrl_r(), reg_val);
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

void gk20a_init_mm(struct gpu_ops *gops)
{
	gops->mm.is_debug_mode_enabled = gk20a_mm_mmu_debug_mode_enabled;
	gops->mm.set_debug_mode = gk20a_mm_mmu_set_debug_mode;
	gops->mm.gmmu_map = gk20a_locked_gmmu_map;
	gops->mm.gmmu_unmap = gk20a_locked_gmmu_unmap;
	gops->mm.vm_remove = gk20a_vm_remove_support;
	gops->mm.vm_alloc_share = gk20a_vm_alloc_share;
	gops->mm.vm_bind_channel = gk20a_vm_bind_channel;
	gops->mm.fb_flush = gk20a_mm_fb_flush;
	gops->mm.l2_invalidate = gk20a_mm_l2_invalidate;
	gops->mm.l2_flush = gk20a_mm_l2_flush;
	gops->mm.cbc_clean = gk20a_mm_cbc_clean;
	gops->mm.tlb_invalidate = gk20a_mm_tlb_invalidate;
	gops->mm.get_iova_addr = gk20a_mm_iova_addr;
	gops->mm.get_physical_addr_bits = gk20a_mm_get_physical_addr_bits;
	gops->mm.get_mmu_levels = gk20a_mm_get_mmu_levels;
	gops->mm.init_pdb = gk20a_mm_init_pdb;
	gops->mm.init_mm_setup_hw = gk20a_init_mm_setup_hw;
}

