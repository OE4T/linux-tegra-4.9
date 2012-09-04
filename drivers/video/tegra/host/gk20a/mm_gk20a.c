/*
 * drivers/video/tegra/host/gk20a/mm_gk20a.c
 *
 * GK20A memory management
 *
 * Copyright (c) 2011, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>

#include "../../nvmap/nvmap.h"
#include "../../nvmap/nvmap_ioctl.h"

#include "../dev.h"
#include "../nvhost_as.h"
#include "gk20a.h"
#include "mm_gk20a.h"
#include "hw_gmmu_gk20a.h"
#include "hw_fb_gk20a.h"
#include "hw_bus_gk20a.h"
#include "hw_ram_gk20a.h"
#include "hw_mc_gk20a.h"
#include "hw_flush_gk20a.h"

#include "kind_gk20a.h"

/* we support 2 page sizes, define indexing based upon that */
static inline int gmmu_page_size_idx(u32 ps)
{
	return (ps) != 4096; /* 4K:=0, all else := 1 */
}
static const u32 gmmu_page_sizes[2] = { 0x1000, 0x20000 }; /* 4KB and 128KB */
static const u32 gmmu_page_shift[2] = { 12, 17 };
static const u64 gmmu_page_offset_mask[2] = { 0xfffLL, 0x1ffffLL };
static const u64 gmmu_page_mask[2] = { ~0xfffLL, ~0x1ffffLL };

static int gk20a_init_mm_reset_enable_hw(struct gk20a *g)
{
	u32 pmc_enable;

	pmc_enable = gk20a_readl(g, mc_enable_r());
	pmc_enable &= ~mc_enable_pfb_enabled_f();
	pmc_enable &= ~mc_enable_l2_enabled_f();
	pmc_enable &= ~mc_enable_ce2_enabled_f();
	pmc_enable &= ~mc_enable_xbar_enabled_f();
	pmc_enable &= ~mc_enable_hub_enabled_f();
	gk20a_writel(g, mc_enable_r(), pmc_enable);

	pmc_enable = gk20a_readl(g, mc_enable_r());
	pmc_enable |= mc_enable_pfb_enabled_f();
	pmc_enable |= mc_enable_l2_enabled_f();
	pmc_enable |= mc_enable_ce2_enabled_f();
	pmc_enable |= mc_enable_xbar_enabled_f();
	pmc_enable |= mc_enable_hub_enabled_f();
	gk20a_writel(g, mc_enable_r(), pmc_enable);
	gk20a_readl(g, mc_enable_r());

	nvhost_dbg_fn("done");
	return 0;
}

static int gk20a_init_mm_setup_sw(struct gk20a *g, bool reinit)
{
	struct mm_gk20a *mm = &g->mm;
	int i;

	nvhost_dbg_fn("");

	if (reinit) {
		nvhost_dbg_fn("skip init");
		return 0;
	}

	mm->g = g;
	mm->big_page_size = (128 << 10);
	mm->pde_stride    = mm->big_page_size << 10;
	mm->pde_stride_shift = ilog2(mm->pde_stride);
	BUG_ON(mm->pde_stride_shift > 31); /* we have assumptions about this */

	for (i = 0; i < ARRAY_SIZE(gmmu_page_sizes); i++) {

		u32 num_ptes, pte_space, num_pages;

		/* assuming "full" page tables */
		num_ptes = mm->pde_stride / gmmu_page_sizes[i];

		pte_space = num_ptes * gmmu_pte__size_v();
		/* allocate whole pages */
		pte_space = roundup(pte_space, PAGE_SIZE);

		num_pages = pte_space / PAGE_SIZE;
		/* make sure "order" is viable */
		BUG_ON(!is_power_of_2(num_pages));

		mm->page_table_sizing[i].num_ptes = num_ptes;
		mm->page_table_sizing[i].order = ilog2(num_pages);
	}

	/* we only have 256MB of sysmem to play with anyway*/
	/* TBD: fixme */
	mm->channel.size = mm->pde_stride * 2;
	nvhost_dbg_info("channel vm size = 0x%llx", mm->channel.size);

	gk20a_init_bar1_vm(mm);

	gk20a_init_uncompressed_kind_map();
	gk20a_init_kind_attr();

	nvhost_dbg_fn("done");
	return 0;
}

/* make sure gk20a_init_mm_support is called before */
static int gk20a_init_mm_setup_hw(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct inst_desc *inst_block = &mm->bar1.inst_block;
	phys_addr_t inst_pa = inst_block->cpu_pa;

	nvhost_dbg_fn("");

	/* set large page size in fb
	 * note this is very early on, can we defer it ? */
	{
		u32 fb_mmu_ctrl = gk20a_readl(g, fb_mmu_ctrl_r());
		fb_mmu_ctrl = (fb_mmu_ctrl & ~fb_mmu_ctrl_vm_pg_size_f(~0x0)) |
			fb_mmu_ctrl_vm_pg_size_128kb_f();
		gk20a_writel(g, fb_mmu_ctrl_r(), fb_mmu_ctrl);
	}

	inst_pa = (u32)(inst_pa >> bar1_instance_block_shift_gk20a());
	nvhost_dbg_info("bar1 inst block ptr: 0x%08x",  (u32)inst_pa);

	/* this is very early in init... can we defer this? */
	{
		gk20a_writel(g, bus_bar1_block_r(),
			     bus_bar1_block_target_vid_mem_f() |
			     bus_bar1_block_mode_virtual_f() |
			     bus_bar1_block_ptr_f(inst_pa));
	}

	nvhost_dbg_fn("done");
	return 0;
}

int gk20a_init_mm_support(struct gk20a *g, bool reinit)
{
	u32 err;

	err = gk20a_init_mm_reset_enable_hw(g);
	if (err)
		return err;

	err = gk20a_init_mm_setup_sw(g, reinit);
	if (err)
		return err;

	err = gk20a_init_mm_setup_hw(g);
	if (err)
		return err;

	return err;
}

#ifdef CONFIG_TEGRA_SIMULATION_SPLIT_MEM
static int alloc_gmmu_nvmap_pages(struct vm_gk20a *vm, u32 order, phys_addr_t *pa, void **handle)
{
	struct mem_mgr *client = mem_mgr_from_vm(vm);
	struct mem_handle *r;
	phys_addr_t phys;
	u32 num_pages = 1 << order;
	u32 len = num_pages * PAGE_SIZE;
	u32 *va;

	nvhost_dbg_fn("");

	r = mem_op().alloc(client, len,
			DEFAULT_NVMAP_ALLOC_ALIGNMENT,
			DEFAULT_NVMAP_ALLOC_FLAGS,
			NVMAP_HEAP_CARVEOUT_GENERIC);
	if (IS_ERR_OR_NULL(r)) {
		nvhost_dbg(dbg_pte, "nvmap_alloc failed\n");
		goto err_out;
	}
	va = mem_op().mmap(r);
	if (IS_ERR_OR_NULL(va)) {
		nvhost_dbg(dbg_pte, "nvmap_mmap failed\n");
		goto err_alloced;
	}
	phys = mem_op().pin(client, r);
	if (IS_ERR_OR_NULL((void *)pa)) {
		nvhost_dbg(dbg_pte, "nvmap_pin failed\n");
		goto err_alloced;
	}

	memset(va, 0, len);
	mem_op().munmap(r, va);
	*pa = phys;
	*handle = (void *)r;

	return 0;

err_alloced:
	mem_op().put(client, r);
err_out:
	return -ENOMEM;
}
#endif

static int alloc_gmmu_sysmem_pages(struct vm_gk20a *vm, u32 order, phys_addr_t *pa, void **handle)
{
	struct page *pte_page;

	nvhost_dbg_fn("");

	pte_page = alloc_pages(GFP_KERNEL | __GFP_ZERO, order);
	if (!pte_page)
		return -ENOMEM;

	*pa = page_to_phys(pte_page);
	*handle = pte_page;
	return 0;
}

static int alloc_gmmu_pages(struct vm_gk20a *vm, u32 order, phys_addr_t *pa, void **handle)
{

	nvhost_dbg_fn("");

#ifdef CONFIG_TEGRA_SIMULATION_SPLIT_MEM
	if (tegra_split_mem_active())
		return alloc_gmmu_nvmap_pages(vm, order, pa, handle);
	else
#endif
		return alloc_gmmu_sysmem_pages(vm, order, pa, handle);
}

static void free_gmmu_pages(struct vm_gk20a *vm, void *handle, u32 order)
{

	nvhost_dbg_fn("");

#ifdef CONFIG_TEGRA_SIMULATION_SPLIT_MEM
	if (tegra_split_mem_active()) {
		struct mem_mgr *client = mem_mgr_from_vm(vm);
		mem_op().put(client, handle);
	} else
#endif
		__free_pages((struct page *)handle, order);
}

static int map_gmmu_pages(void *handle, void **va)
{
	u32 *tmp_va;

	nvhost_dbg_fn("");

#ifdef CONFIG_TEGRA_SIMULATION_SPLIT_MEM
	if (tegra_split_mem_active()) {
		struct mem_handle *r = handle;

		tmp_va = mem_op().mmap(r);
		if (IS_ERR_OR_NULL(tmp_va))
			goto err_out;
	} else
#endif
	{
		tmp_va = kmap((struct page *)handle);
		if (!tmp_va)
			goto err_out;
	}
	*va = tmp_va;
	return 0;

err_out:
	return -ENOMEM;
}

static void unmap_gmmu_pages(void *handle, u32 *va)
{

	nvhost_dbg_fn("");

#ifdef CONFIG_TEGRA_SIMULATION_SPLIT_MEM
	if (tegra_split_mem_active()) {
		struct mem_handle *r = handle;
		mem_op().munmap(r, va);
	} else
#endif
		kunmap((struct page *)handle);
}

static int update_gmmu_ptes(struct vm_gk20a *vm, u32 page_size_idx,
			u64 bfr_addr, u64 first_vaddr, u64 last_vaddr,
			u8 kind_v, u32 ctag_offset, bool cachable);

/* allocate a phys contig region big enough for a full
 * sized gmmu page table for the given gmmu_page_size.
 * the whole range is zeroed so it's "invalid"/will fault
 */

static int zalloc_gmmu_page_table_gk20a(struct vm_gk20a *vm,
				       u32 gmmu_page_size,
				       struct page_table_gk20a *pte,
				       phys_addr_t *pa)
{
	int err, page_size_idx;
	u32 pte_order;
	phys_addr_t phys;
	void *handle;

	nvhost_dbg_fn("");

	/* allocate enough pages for the table */
	page_size_idx = gmmu_page_size_idx(gmmu_page_size);
	pte_order = vm->mm->page_table_sizing[page_size_idx].order;

	err = alloc_gmmu_pages(vm, pte_order, &phys, &handle);
	if (err)
		return err;

	nvhost_dbg(dbg_pte, "pte = 0x%p, phys = 0x%x", pte, (unsigned int)phys);

	*pa = phys;
	pte->ref = handle;

	return 0;
}

/* given address range (inclusive) determine the pdes crossed */
static inline void pde_range_from_vaddr_range(struct vm_gk20a *vm,
					      u64 addr_lo, u64 addr_hi,
					      u32 *pde_lo, u32 *pde_hi)
{
	*pde_lo = (u32)(addr_lo >> vm->mm->pde_stride_shift);
	*pde_hi = (u32)(addr_hi >> vm->mm->pde_stride_shift);
	nvhost_dbg(dbg_pte, "addr_lo=0x%llx addr_hi=0x%llx pde_ss=%d",
		   addr_lo, addr_hi, vm->mm->pde_stride_shift);
	nvhost_dbg(dbg_pte, "pde_lo=%d pde_hi=%d",
		   *pde_lo, *pde_hi);
}

static inline u32 *pde_from_index(struct vm_gk20a *vm, u32 i)
{
	return (u32 *) (((u8 *)vm->pdes.kv) + i*gmmu_pde__size_v());
}

static inline u32 pte_index_from_vaddr(struct vm_gk20a *vm,
				       u64 addr, u32 page_size_idx)
{
	static const u32 page_shift[2] = {12, 17};
	u32 ret;
	/* mask off pde part */
	addr = addr & ((((u64)1) << vm->mm->pde_stride_shift) - ((u64)1));
	/* shift over to get pte index. note assumption that pte index
	 * doesn't leak over into the high 32b */
	ret = (u32)(addr >> page_shift[page_size_idx]);

	nvhost_dbg(dbg_pte, "addr=0x%llx pte_i=0x%x", addr, ret);
	return ret;
}

static inline void pte_space_page_offset_from_index(u32 i, u32 *pte_page,
						    u32 *pte_offset)
{
	/* ptes are 8B regardless of pagesize */
	/* pte space pages are 4KB. so 512 ptes per 4KB page*/
	*pte_page = i >> 9;

	/* this offset is a pte offset, not a byte offset */
	*pte_offset = i & ((1<<9)-1);

	nvhost_dbg(dbg_pte, "i=0x%x pte_page=0x%x pte_offset=0x%x",
		   i, *pte_page, *pte_offset);
}

/*
 * given a pde index/page table number make sure it has
 * backing store and if not go ahead allocate it and
 * record it in the appropriate pde
 */

static int validate_gmmu_page_table_gk20a(struct vm_gk20a *vm,
					  u32 i,
					  u32 gmmu_page_size)
{
	u32 err;
	phys_addr_t pte_addr;
	phys_addr_t dbg_addr;
	struct page_table_gk20a *pte = vm->pdes.ptes + i;
	u32 *pde;

	nvhost_dbg_fn("");
	nvhost_dbg(dbg_pte, "i = %d", i);

	if (unlikely(gmmu_page_size != 4096 &&
		     gmmu_page_size != 128*1024))
		return -EINVAL;

	/* if it's already in place it's valid */
	/* tbd: unless it's set to the other pagesize... */
	if (pte->ref)
		return 0;

	err = zalloc_gmmu_page_table_gk20a(vm, gmmu_page_size, pte, &pte_addr);
	if (err)
		return err;

	dbg_addr = (u32)pte_addr;
	pte_addr >>= gmmu_pde_address_shift_v();
	pde = pde_from_index(vm, i);

	if (gmmu_page_size == 4096) {
		/* "small" gmmu page size */
		nvhost_dbg(dbg_pte, "4KB ptes @ 0x%x", dbg_addr);
		mem_wr32(pde, 0, (gmmu_pde_size_full_f() |
				  gmmu_pde_aperture_big_invalid_f()));
		mem_wr32(pde, 1, (gmmu_pde_aperture_small_video_memory_f() |
				  gmmu_pde_vol_small_true_f() |
				  gmmu_pde_address_small_sys_f((u32)pte_addr)));
	} else {
		/* "large" gmmu page size */
		nvhost_dbg(dbg_pte, "128KB ptes @ 0x%x", dbg_addr);
		mem_wr32(pde, 0, (gmmu_pde_size_full_f() |
				  gmmu_pde_aperture_big_video_memory_f() |
				  gmmu_pde_address_big_sys_f((u32)pte_addr)));
		mem_wr32(pde, 1, (gmmu_pde_aperture_small_invalid_f() |
				  gmmu_pde_vol_big_true_f()));
	}

	vm->pdes.dirty = true;

	smp_mb();
	/* !!! FIXME : Bug 963594 !!!
	   Volatile pde/pte doesn't look working.
	   Flush everything (slowest) for now. */
	gk20a_mm_fb_flush(vm->mm->g);
	gk20a_mm_l2_flush(vm->mm->g, true);
	gk20a_mm_tlb_invalidate(vm->mm->g, vm);

	return 0;
}

static u64 gk20a_channel_vm_alloc_va(struct vm_gk20a *vm,
				     u64 size,
				     u32 gmmu_page_size)
{
	struct nvhost_allocator *vma = &vm->vma;
	int err;
	u64 offset;
	u32 gmmu_page_size_shift = ilog2(gmmu_page_size);
	u32 start_page_nr = 0, num_pages;
	u32 i, pde_lo, pde_hi;

	/* be certain we round up to gmmu_page_size if needed */
	size = (size + ((u64)gmmu_page_size - 1)) & ~((u64)gmmu_page_size - 1);

	nvhost_dbg_info("size=0x%llx", size);

	num_pages = size >> gmmu_page_size_shift;
	err = vma->alloc(vma, &start_page_nr, num_pages);

	offset = (u64)start_page_nr << gmmu_page_size_shift;
	if (err) {
		nvhost_err(dev_from_vm(vm),
			   "oom: sz=0x%llx", size);
		return 0;
	}

	pde_range_from_vaddr_range(vm,
				   offset, offset + size - 1,
				   &pde_lo, &pde_hi);

	for (i = pde_lo; i <= pde_hi; i++) {

		err = validate_gmmu_page_table_gk20a(vm, i, gmmu_page_size);

		/* mark the pages valid, with correct phys addr */
		if (err) {
			nvhost_err(dev_from_vm(vm),
				   "failed to validate page table %d: %d",
				   i, err);
			return 0;
		}
	}

	nvhost_dbg_fn("ret=0x%llx", offset);

	return offset;
}

static void gk20a_channel_vm_free_va(struct vm_gk20a *vm, u64 offset,
				     u64 size, u32 gmmu_page_size)
{
	struct nvhost_allocator *vma = &vm->vma;
	u32 gmmu_page_size_shift = ilog2(gmmu_page_size);
	u32 start_page_nr = 0, num_pages;
	int err;

	nvhost_dbg_info("offset=0x%llx, size=0x%llx", offset, size);

	start_page_nr = (u32)(offset >> gmmu_page_size_shift);
	num_pages = (u32)((size + gmmu_page_size - 1) >> gmmu_page_size_shift);

	err = vma->free(vma, start_page_nr, num_pages);
	if (err) {
		nvhost_err(dev_from_vm(vm),
			   "oom: offset=0x%llx, sz=0x%llx",
			   offset, size);
	}
}

static int insert_mapped_buffer(struct rb_root *root,
				struct mapped_buffer_node *mapped_buffer)
{
	struct rb_node **new_node = &(root->rb_node), *parent = NULL;

	/* Figure out where to put new node */
	while (*new_node) {
		struct mapped_buffer_node *cmp_with =
			container_of(*new_node, struct mapped_buffer_node, node);

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

static struct mapped_buffer_node *find_mapped_buffer(struct rb_root *root,
						     u64 addr)
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
	return 0;
}

/* convenience setup for nvmap buffer attr queries */
struct bfr_attr_query {
	int err;
	u32 v;
};
static u32 nvmap_bfr_param[] = {
#define BFR_SIZE   0
	NVMAP_HANDLE_PARAM_SIZE,
#define BFR_ALIGN  1
	NVMAP_HANDLE_PARAM_ALIGNMENT,
#define BFR_HEAP   2
	NVMAP_HANDLE_PARAM_HEAP,
#define BFR_KIND   3
	NVMAP_HANDLE_PARAM_KIND,
};
#define BFR_ATTRS (sizeof(nvmap_bfr_param)/sizeof(nvmap_bfr_param[0]))

struct buffer_attrs {
	phys_addr_t addr;
	u64 size;
	u64 align;
	u32 ctag_offset;
	u32 ctag_lines;
	int contig;
	int iovmm_mapped;
	int page_size_idx; /* largest gmmu page size which fits the buffer's attributes */
	u8 kind_v;
	u8 uc_kind_v;
};

static int setup_buffer_size_and_align(struct mem_mgr *memmgr,
				       struct mem_handle *hdl,
				       struct buffer_attrs *bfr,
				       struct bfr_attr_query *query)
{
	int i;
	/* buffer allocation size and alignment must be a multiple
	   of one of the supported page sizes.*/
	bfr->size = query[BFR_SIZE].v;
	bfr->align = query[BFR_ALIGN].v;
	bfr->page_size_idx = -1;

	for (i = 1; i >= 0; i--) /*  choose the biggest first (top->bottom) */
		if (!(gmmu_page_offset_mask[i] & bfr->align)) {
			/* would like to add this too but nvmap returns the
			 * original requested size not the allocated size.
			 * (!(gmmu_page_offset_mask[i] & bfr->size)) */
			bfr->page_size_idx = i;
			break;
		}

	if (unlikely(0 > bfr->page_size_idx)) {
		nvhost_err(0, "unsupported nvmap buffer allocation "
			   "alignment, size: 0x%llx, 0x%llx\n",
			   bfr->align, bfr->size);
		return -EINVAL;
	}
	switch (query[BFR_HEAP].v) {
	case NVMAP_HEAP_SYSMEM:
		/* sysmem, contig
		 * Fall through to carveout...
		 * TBD: Need nvmap support for scattered sysmem allocs
		 * w/o mapping through smmu.
		 */

	case NVMAP_HEAP_CARVEOUT_GENERIC:
		/* carveout sysmem, contig */
		bfr->contig = 1;
		bfr->iovmm_mapped = 0;
		break;

	case NVMAP_HEAP_IOVMM:
		/* sysmem, iovmm/smmu mapped */
		bfr->contig = 1;
		bfr->iovmm_mapped = 1;
		break;
	default:
		nvhost_err(0, "unsupported nvmap buffer heap: 0x%x\n",
			   query[BFR_HEAP].v);
		return -EINVAL;
	}

	bfr->kind_v = query[BFR_KIND].v;

	return 0;
}


static int setup_buffer_kind_and_compression(u32 flags,
					     u32 kind,
					     struct buffer_attrs *bfr,
					     u32 gmmu_page_size)
{
	bool kind_compressible;

	/* This flag (which comes from map_buffer ioctl) is for override now.
	   It will be removed when all clients which use it have been
	   changed to specify kind in the nvmap buffer alloc. */
	if (flags & NVHOST_MAP_BUFFER_FLAGS_KIND_SPECIFIED)
		bfr->kind_v = kind;

	if (unlikely(bfr->kind_v == gmmu_pte_kind_invalid_v()))
		bfr->kind_v = gmmu_pte_kind_pitch_v();

	if (unlikely(!gk20a_kind_is_supported(bfr->kind_v))) {
		nvhost_err(0, "kind 0x%x not supported", bfr->kind_v);
		return EINVAL;
	}

	bfr->uc_kind_v = gmmu_pte_kind_invalid_v();
	/* find a suitable uncompressed kind if it becomes necessary later */
	kind_compressible = gk20a_kind_is_compressible(bfr->kind_v);
	if (kind_compressible) {
		bfr->uc_kind_v = gk20a_get_uncompressed_kind(bfr->kind_v);
		if (unlikely(bfr->uc_kind_v == gmmu_pte_kind_invalid_v())) {
			/* shouldn't happen, but it is worth cross-checking */
			nvhost_err(0, "comptag kind 0x%x can't be"
				   " downgraded to uncompressed kind",
				   bfr->kind_v);
			return -EINVAL;
		}
	}
	/* comptags only supported for suitable kinds, 128KB pagesize */
	if (unlikely(kind_compressible && (gmmu_page_size != 128*1024))) {
		nvhost_warn(0, "comptags specified"
			    " but pagesize being used doesn't support it");
		/* it is safe to fall back to uncompressed as
		   functionality is not harmed */
		bfr->kind_v = bfr->uc_kind_v;
		kind_compressible = false;
	}
	if (kind_compressible)
		bfr->ctag_lines = bfr->size >> COMP_TAG_LINE_SIZE_SHIFT;
	else
		bfr->ctag_lines = 0;

	return 0;
}


static u64 gk20a_vm_map(struct vm_gk20a *vm,
			struct mem_mgr *memmgr,
			struct mem_handle *r,
			u64 offset_align,
			u32 flags /*NVHOST_MAP_BUFFER_FLAGS_*/,
			u32 kind)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct nvhost_allocator *ctag_allocator = &g->gr.comp_tags;
	struct device *d = &g->dev->dev;
	struct mapped_buffer_node *mapped_buffer = 0;
	bool inserted = false, va_allocated = false;
	u32 gmmu_page_size = 0;
	u64 map_offset = 0;
	int attr, err = 0;
	struct buffer_attrs bfr;
	struct bfr_attr_query query[BFR_ATTRS];

	/* query bfr attributes: size, align, heap, kind */
	for (attr = 0; attr < BFR_ATTRS; attr++) {
		query[attr].err =
			mem_op().get_param(memmgr, r,
					nvmap_bfr_param[attr],
					&query[attr].v);
		if (unlikely(query[attr].err != 0)) {
			nvhost_err(d,
				   "failed to get nvmap buffer param %d: %d\n",
				   nvmap_bfr_param[attr],
				   query[attr].err);
			return query[attr].err;
		}
	}

	/* validate/adjust bfr attributes */

	err = setup_buffer_size_and_align(memmgr, r, &bfr, query);
	if (unlikely(err))
		goto clean_up;

	/* if specified the map offset needs to be gmmu page aligned */
	if (flags & NVHOST_MAP_BUFFER_FLAGS_OFFSET) {
		map_offset = offset_align;
		if (map_offset & gmmu_page_offset_mask[bfr.page_size_idx]) {
			nvhost_err(d, "unsupported buffer map offset 0x%llx",
				   map_offset);
			err = -EINVAL;
			goto clean_up;
		}
	}

	gmmu_page_size = gmmu_page_sizes[bfr.page_size_idx];

	err = setup_buffer_kind_and_compression(flags, kind,
						&bfr, gmmu_page_size);
	if (unlikely(err)) {
		nvhost_err(d, "failure setting up kind and compression");
		goto clean_up;
	}

	/* allocate compression resources if needed */
	if (bfr.ctag_lines) {
		err = ctag_allocator->alloc(ctag_allocator, &bfr.ctag_offset,
					    bfr.ctag_lines);
		/* ok to fall back here if we ran out */
		/* TBD: we can partially alloc ctags as well... */
		if (err) {
			bfr.ctag_lines = bfr.ctag_offset = 0;
			bfr.kind_v = bfr.uc_kind_v;
		}
	}

	/* init/clear the ctag buffer */
	if (bfr.ctag_lines)
		gk20a_gr_clear_comptags(g,
					bfr.ctag_offset,
					bfr.ctag_offset + bfr.ctag_lines - 1);


	/* TBD: need to get nvmap to assign the correct asid */
	/* until then there's no point even trying */
	if (bfr.iovmm_mapped) {
		nvhost_err(d, "iovmm remapping is unsupported at this time");
		err = -EINVAL;
		goto clean_up;
	}

	/* Allocate (or validate when map_offset != 0) the virtual address. */
	if (!map_offset) {
		map_offset = vm->alloc_va(vm, bfr.size,
					  gmmu_page_size);
		if (!map_offset) {
			nvhost_err(d, "failed to allocate va space");
			err = -ENOMEM;
			goto clean_up;
		}
		va_allocated = true;
	} else {
		/* TODO: validate that the offset has been previosuly allocated ||
		 * allocate here to keep track */
		nvhost_warn(d, "fixed offset mapping isn't safe yet!");
		nvhost_warn(d, "other mappings may collide!");
	}

	/* pin buffer to get phys/iovmm addr */
	bfr.addr = mem_op().pin(memmgr, r);

	nvhost_dbg_info("nvmap pinned buffer @ 0x%x", bfr.addr);
	nvhost_dbg_fn("r=%p, map_offset=0x%llx, contig=%d "
		      "iovmm_mapped=%d kind=0x%x kind_uc=0x%x flags=0x%x",
		      r, map_offset, bfr.contig, bfr.iovmm_mapped,
		      bfr.kind_v, bfr.uc_kind_v, flags);
	nvhost_dbg_info("comptag size=%d start=%d for 0x%x",
			bfr.ctag_lines, bfr.ctag_offset, bfr.addr);


	/* keep track of the buffer for unmapping */
	/* TBD: check for multiple mapping of same buffer */
	mapped_buffer = kzalloc(GFP_KERNEL, sizeof(*mapped_buffer));
	if (!mapped_buffer) {
		nvhost_warn(d, "oom allocating tracking buffer");
		goto clean_up;
	}
	mapped_buffer->memmgr     = memmgr;
	mapped_buffer->handle_ref = r;
	mapped_buffer->addr       = map_offset;
	mapped_buffer->size       = bfr.size;
	mapped_buffer->page_size  = gmmu_page_size;
	mapped_buffer->ctag_offset = bfr.ctag_offset;
	mapped_buffer->ctag_lines  = bfr.ctag_lines;

	err = insert_mapped_buffer(&vm->mapped_buffers, mapped_buffer);
	if (err) {
		nvhost_err(d, "failed to insert into mapped buffer tree");
		goto clean_up;
	}
	inserted = true;

	nvhost_dbg_info("allocated va @ 0x%llx", map_offset);

	err = update_gmmu_ptes(vm, bfr.page_size_idx,
			       bfr.addr,
			       map_offset, map_offset + bfr.size - 1,
			       bfr.kind_v,
			       bfr.ctag_offset,
			       !(flags & NVHOST_MAP_BUFFER_FLAGS_CACHABLE_FALSE));
	if (err) {
		nvhost_err(d, "failed to update ptes on map");
		goto clean_up;
	}

	return map_offset;

clean_up:
	if (inserted)
		rb_erase(&mapped_buffer->node, &vm->mapped_buffers);
	kfree(mapped_buffer);
	if (va_allocated)
		vm->free_va(vm, map_offset, bfr.size, gmmu_page_size);
	if (bfr.ctag_lines)
		ctag_allocator->free(ctag_allocator,
				     bfr.ctag_offset,
				     bfr.ctag_lines);

	nvhost_dbg_info("err=%d\n", err);
	return 0;
}

static int update_gmmu_ptes(struct vm_gk20a *vm, u32 page_size_idx,
		       u64 bfr_addr, u64 first_vaddr, u64 last_vaddr,
		       u8 kind_v, u32 ctag_offset, bool cachable)
{
	int err;
	u32 pde_lo, pde_hi, pde_i, cur_page;
	u32 pte_w[2] = {0, 0}; /* invalid pte */
	u32 ctag = ctag_offset;
	u32 ctag_ptes, ctag_pte_cnt;

	pde_range_from_vaddr_range(vm, first_vaddr, last_vaddr,
				   &pde_lo, &pde_hi);

	nvhost_dbg(dbg_pte, "pde_lo=%d, pde_hi=%d", pde_lo, pde_hi);

	if (gmmu_page_sizes[page_size_idx] == 4096) {
		cur_page = (u32)(bfr_addr >> 12);
		ctag_ptes = COMP_TAG_LINE_SIZE >> 12;
	} else {
		cur_page = (u32)(bfr_addr >> 17);
		ctag_ptes = COMP_TAG_LINE_SIZE >> 17;
	}

	ctag_pte_cnt = 0;

	for (pde_i = pde_lo; pde_i <= pde_hi; pde_i++) {
		u32 pte_lo, pte_hi;
		u32 pte_cur;
		u32 pte_space_page_cur, pte_space_offset_cur;
		u32 pte_space_page, pte_space_page_offset;
		u32 *pde;
		void *pte_kv_cur;

		u32 pte_pfn0 = 0; 		/* Keep compiler quiet */

		struct page_table_gk20a *pte = vm->pdes.ptes + pde_i;

		if (pde_i == pde_lo)
			pte_lo = pte_index_from_vaddr(vm, first_vaddr,
						      page_size_idx);
		else
			pte_lo = 0;

		if ((pde_i != pde_hi) && (pde_hi != pde_lo))
			pte_hi = vm->mm->page_table_sizing[page_size_idx].num_ptes - 1;
		else
			pte_hi = pte_index_from_vaddr(vm, last_vaddr,
						      page_size_idx);

		/* need to worry about crossing pages when accessing the ptes */
		pte_space_page_offset_from_index(pte_lo, &pte_space_page_cur,
						 &pte_space_offset_cur);
#ifdef CONFIG_TEGRA_SIMULATION_SPLIT_MEM
		if (tegra_split_mem_active()) {
			err = map_gmmu_pages(pte->ref, &pte_kv_cur);
		} else
#endif
		{
			pte_pfn0 = page_to_pfn((struct page *)pte->ref);
			err = map_gmmu_pages((void *)pfn_to_page(pte_pfn0 +
								 pte_space_page_cur),
								 &pte_kv_cur);
		}

		if (err) {
			nvhost_err(dev_from_vm(vm),
				   "couldn't map ptes for update");
			goto clean_up;
		}

		nvhost_dbg(dbg_pte, "pte_lo=%d, pte_hi=%d", pte_lo, pte_hi);
		for (pte_cur = pte_lo; pte_cur <= pte_hi; pte_cur++) {

#ifdef CONFIG_TEGRA_SIMULATION_SPLIT_MEM
			if (tegra_split_mem_active()) {
				pte_space_page_offset = pte_cur;
			} else
#endif
			{
				pte_space_page_offset_from_index(pte_cur,
								&pte_space_page,
								&pte_space_page_offset);
				if (unlikely(pte_space_page != pte_space_page_cur)) {
					unmap_gmmu_pages(pfn_to_page(pte_pfn0 +
							 pte_space_page_cur),
							 pte_kv_cur);
					pte_space_page_cur = pte_space_page;
					err = map_gmmu_pages(pfn_to_page(pte_pfn0 +
							     pte_space_page_cur),
							     &pte_kv_cur);
					if (err) {
						nvhost_err(dev_from_vm(vm),
							   "couldn't map ptes for update");
						goto clean_up;
					}
				}
			}
			if (ctag) {
				ctag_pte_cnt++;
				if (ctag_pte_cnt > ctag_ptes) {
					ctag++;
					ctag_pte_cnt = 0;
				}
			}

			nvhost_dbg(dbg_pte, "pte_cur=%d cur_page=0x%x kind=%d ctag=%d",
				pte_cur, cur_page, kind_v, ctag);

			if (likely(bfr_addr != 0)) {
				pte_w[0] = gmmu_pte_valid_true_f() |
					gmmu_pte_address_sys_f(cur_page);
				pte_w[1] = gmmu_pte_aperture_video_memory_f() |
					gmmu_pte_kind_f(kind_v) |
					gmmu_pte_comptagline_f(ctag);
				if (!cachable)
					pte_w[1] |= gmmu_pte_vol_true_f();
				cur_page++;
				pte->ref_cnt++;
			} else
				pte->ref_cnt--;

			nvhost_dbg(dbg_pte, "pte[0]=0x%x pte[1]=0x%x", pte_w[0], pte_w[1]);
			mem_wr32(pte_kv_cur + pte_space_page_offset*8, 0,
				 pte_w[0]);
			mem_wr32(pte_kv_cur + pte_space_page_offset*8, 1,
				 pte_w[1]);
		}

#ifdef CONFIG_TEGRA_SIMULATION_SPLIT_MEM
		if (tegra_split_mem_active())
			unmap_gmmu_pages(pte->ref, pte_kv_cur);
		else
#endif
			unmap_gmmu_pages(pfn_to_page(pte_pfn0 + pte_space_page_cur),
					 pte_kv_cur);

		if (pte->ref_cnt == 0) {
			/* invalidate pde */
			pde = pde_from_index(vm, pde_i);
			mem_wr32(pde, 0, gmmu_pde_aperture_big_invalid_f());
			mem_wr32(pde, 1, gmmu_pde_aperture_small_invalid_f());
			vm->pdes.dirty = true;
			/* free page table */
			free_gmmu_pages(vm, pte->ref,
					vm->mm->page_table_sizing[page_size_idx].order);
			pte->ref = NULL;
		}
	}

	smp_mb();
	/* !!! FIXME : Bug 963594 !!!
	   Volatile pde/pte doesn't look working.
	   Flush everything (slowest) for now. */
	gk20a_mm_fb_flush(vm->mm->g);
	gk20a_mm_l2_flush(vm->mm->g, true);
	gk20a_mm_tlb_invalidate(vm->mm->g, vm);

	return 0;

clean_up:
	/*TBD: potentially rewrite above to pre-map everything it needs to
	 * as that's the only way it can fail */
	return err;

}

static u64 gk20a_channel_vm_map(struct vm_gk20a *vm,
				struct mem_mgr *nvmap,
				struct mem_handle *r,
				u64 offset_align,
				u32 flags /*NVHOST_MAP_BUFFER_FLAGS_*/,
				u32 kind)
{
	nvhost_dbg_fn("");
	return gk20a_vm_map(vm, nvmap, r, offset_align, flags, kind);
}

static void gk20a_channel_vm_unmap(struct vm_gk20a *vm,
			 u64 offset)
{
	struct mapped_buffer_node *mapped_buffer;
	struct gk20a *g = gk20a_from_vm(vm);
	struct nvhost_allocator *comp_tags = &g->gr.comp_tags;
	int err = 0;

	nvhost_dbg_fn("offset=0x%llx", offset);

	mapped_buffer = find_mapped_buffer(&vm->mapped_buffers, offset);
	if (!mapped_buffer) {
		nvhost_dbg(dbg_err, "invalid addr to unmap 0x%llx", offset);
		return;
	}

	vm->free_va(vm, mapped_buffer->addr, mapped_buffer->size,
		    mapped_buffer->page_size);

	comp_tags->free(comp_tags,
		mapped_buffer->ctag_offset, mapped_buffer->ctag_lines);

	err = update_gmmu_ptes(vm,
			       gmmu_page_size_idx(mapped_buffer->page_size),
			       0, /* n/a for unmap */
			       mapped_buffer->addr,
			       mapped_buffer->addr + mapped_buffer->size - 1,
			       0, 0, false /* n/a for unmap */);
	if (err) {
		nvhost_dbg(dbg_err, "failed to update ptes on unmap");
		goto clean_up;
	}

	mem_op().unpin(mapped_buffer->memmgr,
		    mapped_buffer->handle_ref);

	/* remove from mapped buffer tree, free */
	rb_erase(&mapped_buffer->node, &vm->mapped_buffers);
	kfree(mapped_buffer);

	return;

clean_up:

	return;
}

void gk20a_channel_vm_remove_support(struct vm_gk20a *vm)
{
	nvhost_dbg_fn("");
}

/* address space interfaces for the gk20a module */
static int gk20a_as_alloc_share(struct nvhost_as_share *as_share)
{
	struct nvhost_as *as = as_share->as;
	struct nvhost_device *host_dev = as->ch->dev;
	struct gk20a *gk20a = get_gk20a(host_dev);
	struct mm_gk20a *mm = &gk20a->mm;
	struct vm_gk20a *vm;
	char name[32];
	int err;

	nvhost_dbg_fn("");

	vm = kzalloc(GFP_KERNEL, sizeof(*vm));
	if (!vm)
		return -ENOMEM;

	as_share->priv = (void *)vm;

	vm->mm = mm;
	vm->as_share = as_share;

	vm->va_start  = 0; /* we have a one page hole though so zeros fault*/
	vm->va_limit  = mm->channel.size;

	{
		u32 pde_lo, pde_hi;
		pde_range_from_vaddr_range(vm,
					   0, vm->va_limit-1,
					   &pde_lo, &pde_hi);
		vm->pdes.num_pdes = pde_hi + 1;
	}

	vm->pdes.ptes = kzalloc(GFP_KERNEL,
				sizeof(struct page_table_gk20a) *
				vm->pdes.num_pdes);
	if (!vm->pdes.ptes)
		return -ENOMEM;

	nvhost_dbg_info("init space for va_limit=0x%llx num_pdes=%d",
		   vm->va_limit, vm->pdes.num_pdes);

	/* allocate the page table directory */
	err = alloc_gmmu_pages(vm, 0, &vm->pdes.phys, &vm->pdes.ref);
	if (err) {
		return -ENOMEM;
	}
	err = map_gmmu_pages(vm->pdes.ref, &vm->pdes.kv);
	if (err) {
		free_gmmu_pages(vm, vm->pdes.ref, 0);
		return -ENOMEM;
	}
	nvhost_dbg(dbg_pte, "pdes.kv = 0x%p, pdes.phys = 0x%x",
			vm->pdes.kv, vm->pdes.phys);

	/* we could release vm->pdes.kv but it's only one page... */

	/* alloc in 4K granularity */
	snprintf(name, sizeof(name), "gk20a_as_%d", as_share->id);
	nvhost_allocator_init(&vm->vma, name,
		1, mm->channel.size >> 12, 1);

	vm->mapped_buffers = RB_ROOT;

	vm->alloc_va = gk20a_channel_vm_alloc_va;
	vm->free_va =  gk20a_channel_vm_free_va;
	vm->map =  gk20a_channel_vm_map;
	vm->unmap =  gk20a_channel_vm_unmap;
	vm->remove_support = gk20a_channel_vm_remove_support;

	return 0;
}


static int gk20a_as_release_share(struct nvhost_as_share *as_share)
{
	struct vm_gk20a *vm = (struct vm_gk20a *)as_share->priv;
	struct mapped_buffer_node *mapped_buffer;
	struct rb_node *node;
	int err = 0;

	nvhost_dbg_fn("");

	node = rb_first(&vm->mapped_buffers);
	while (node) {
		mapped_buffer =
			container_of(node, struct mapped_buffer_node, node);
		vm->unmap(vm, mapped_buffer->addr);
		node = rb_first(&vm->mapped_buffers);
	}

	unmap_gmmu_pages(vm->pdes.ref, vm->pdes.kv);
	free_gmmu_pages(vm, vm->pdes.ref, 0);
	kfree(vm->pdes.ptes);
	nvhost_allocator_destroy(&vm->vma);

	return err;
}

static int gk20a_as_alloc_space(struct nvhost_as_share *as_share,
				struct nvhost_as_alloc_space_args *args)

{	int err = -ENOMEM;
	/*struct vm_gk20a *vm = (struct vm_gk20a *)as_share->priv;*/

	nvhost_dbg_fn("");
	return err;
}

static int gk20a_as_free_space(struct nvhost_as_share *as_share,
			       struct nvhost_as_free_space_args *args)
{
	int err = -ENOMEM;
	/*struct vm_gk20a *vm = (struct vm_gk20a *)as_share->priv;*/
	nvhost_dbg_fn("");

	return err;

}

static int gk20a_as_bind_hwctx(struct nvhost_as_share *as_share,
			       struct nvhost_hwctx *hwctx)
{
	int err = 0;
	struct vm_gk20a *vm = (struct vm_gk20a *)as_share->priv;
	struct channel_gk20a *c = hwctx->priv;

	nvhost_dbg_fn("");

	c->vm = vm;
	err = channel_gk20a_commit_va(c);
	if (err)
		c->vm = 0;

	return err;
}

static int gk20a_as_map_buffer(struct nvhost_as_share *as_share,
			       struct mem_mgr *nvmap,
			       struct mem_handle *r,
			       u64 *offset_align,
			       u32 flags /*NVHOST_AS_MAP_BUFFER_FLAGS_*/)
{
	int err = 0;
	struct vm_gk20a *vm = (struct vm_gk20a *)as_share->priv;
	u64 ret_va;

	nvhost_dbg_fn("");

	ret_va = vm->map(vm, nvmap, r, *offset_align, flags, 0/*no kind here, to be removed*/);
	*offset_align = ret_va;
	if (!ret_va)
		err = -EINVAL;

	return err;

}

static int gk20a_as_unmap_buffer(struct nvhost_as_share *as_share,
				 u64 offset)
{
	int err = 0;
	struct vm_gk20a *vm = (struct vm_gk20a *)as_share->priv;

	nvhost_dbg_fn("");

	vm->unmap(vm, offset);

	return err;
}


const struct nvhost_as_moduleops gk20a_as_moduleops = {
	.alloc_share   = gk20a_as_alloc_share,
	.release_share = gk20a_as_release_share,
	.alloc_space   = gk20a_as_alloc_space,
	.free_space    = gk20a_as_free_space,
	.bind_hwctx    = gk20a_as_bind_hwctx,
	.map_buffer    = gk20a_as_map_buffer,
	.unmap_buffer  = gk20a_as_unmap_buffer,
};

/* bar1 vm interfaces */

u64 gk20a_bar1_vm_alloc_va(struct vm_gk20a *vm,
			   u64 size,
			   u32 bar1_page_size)
{
	struct nvhost_allocator *vma = &vm->vma;
	int err;
	u64 offset;
	u32 shift = ilog2(bar1_page_size);
	u32 start_page_nr = 0, num_pages;
	u32 i, pde_lo, pde_hi;

	BUG_ON(bar1_page_size != 4096); /* always 4KB ?*/;

	/* be certain we round up to gmmu_page_size if needed */
	size = (size + ((u64)bar1_page_size - 1)) & ~((u64)bar1_page_size - 1);

	nvhost_dbg_info("size=0x%llx", size);

	num_pages = size >> shift;
	err = vma->alloc(vma, &start_page_nr, num_pages);
	offset = (u64)start_page_nr << shift;
	if (err != 0) {
		nvhost_err(dev_from_vm(vm), "oom: sz=0x%llx", size);
		return 0;
	}

	pde_range_from_vaddr_range(vm,
				   offset, offset + size - 1,
				   &pde_lo, &pde_hi);

	for (i = pde_lo; i <= pde_hi; i++) {

		err = validate_gmmu_page_table_gk20a(vm, i, bar1_page_size);

		/* mark the pages valid, with correct phys adddr */
		if (err) {
			nvhost_err(dev_from_vm(vm),
				   "failed to validate bar1 page table %d:"
				   " %d", i, err);
			return 0;
		}
	}

	nvhost_dbg_fn("ret=0x%llx", offset);

	return offset;
}

void gk20a_bar1_vm_free_va(struct vm_gk20a *vm, u64 offset, u64 size, u32 page_size)
{
	nvhost_dbg_fn("");
}

u64 gk20a_bar1_vm_map(struct vm_gk20a *vm,
		      struct mem_mgr *nvmap,
		      struct mem_handle *r,
		      u64 offset_align,
		      u32 flags/*NVHOST_MAP_BUFFER_FLAGS_*/,
		      u32 kind)
{
	nvhost_dbg_fn("");
	return gk20a_vm_map(vm, nvmap, r, offset_align, flags, kind);
}

void gk20a_bar1_vm_unmap(struct vm_gk20a *vm,
			u64 offset)
{
	nvhost_dbg_fn("");
}

void gk20a_bar1_vm_remove_support(struct vm_gk20a *vm)
{
	nvhost_dbg_fn("");
	nvhost_allocator_destroy(&vm->vma);
}

int gk20a_init_bar1_vm(struct mm_gk20a *mm)
{
	int err;
	struct mem_mgr *nvmap = mem_mgr_from_mm(mm);
	phys_addr_t inst_pa;
	void *inst_ptr;
	struct vm_gk20a *vm = &mm->bar1.vm;
	struct inst_desc *inst_block = &mm->bar1.inst_block;
	u32 pde_addr_lo;
	u32 pde_addr_hi;

	vm->mm = mm;

	mm->bar1.aperture_size = bar1_aperture_size_mb_gk20a() << 20;

	nvhost_dbg_info("bar1 vm size = 0x%x", mm->bar1.aperture_size);

	vm->va_start  = mm->pde_stride * 1;
	vm->va_limit  = mm->bar1.aperture_size;

	{
		u32 pde_lo, pde_hi;
		pde_range_from_vaddr_range(vm,
					   0, vm->va_limit-1,
					   &pde_lo, &pde_hi);
		vm->pdes.num_pdes = pde_hi + 1;
	}

	vm->pdes.ptes = kzalloc(GFP_KERNEL,
				sizeof(struct page_table_gk20a) *
				vm->pdes.num_pdes);
	if (!vm->pdes.ptes)
		return -ENOMEM;

	nvhost_dbg_info("init space for bar1 va_limit=0x%llx num_pdes=%d",
		   vm->va_limit, vm->pdes.num_pdes);


	/* allocate the page table directory */
	err = alloc_gmmu_pages(vm, 0, &vm->pdes.phys, &vm->pdes.ref);
	if (err)
		goto clean_up;

	err = map_gmmu_pages(vm->pdes.ref, &vm->pdes.kv);
	if (err) {
		free_gmmu_pages(vm, vm->pdes.ref, 0);
		goto clean_up;
	}
	nvhost_dbg(dbg_pte, "bar 1 pdes.kv = 0x%p, pdes.phys = 0x%x",
			vm->pdes.kv, vm->pdes.phys);
	/* we could release vm->pdes.kv but it's only one page... */

	pde_addr_lo = u64_lo32(vm->pdes.phys) >> 12;
	pde_addr_hi = u64_hi32(vm->pdes.phys);

	nvhost_dbg_info("pde pa=0x%x pde_addr_lo=0x%x pde_addr_hi=0x%x",
		   vm->pdes.phys, pde_addr_lo, pde_addr_hi);

	/* allocate instance mem for bar1 */
	inst_block->mem.size = ram_in_alloc_size_v();
	inst_block->mem.ref =
		mem_op().alloc(nvmap, inst_block->mem.size,
			    DEFAULT_NVMAP_ALLOC_ALIGNMENT,
			    DEFAULT_NVMAP_ALLOC_FLAGS,
			    NVMAP_HEAP_CARVEOUT_GENERIC);

	if (IS_ERR(inst_block->mem.ref)) {
		inst_block->mem.ref = 0;
		err = -ENOMEM;
		goto clean_up;
	}

	inst_block->cpu_pa = inst_pa =
		mem_op().pin(nvmap, inst_block->mem.ref);

	/* IS_ERR throws a warning here (expecting void *) */
	if (inst_pa == -EINVAL || inst_pa == -EINTR) {
		inst_pa = 0;
		err = (int)inst_pa;
		goto clean_up;
	}
	inst_ptr = mem_op().mmap(inst_block->mem.ref);
	if (IS_ERR(inst_ptr)) {
		return -ENOMEM;
		goto clean_up;
	}

	nvhost_dbg_info("bar1 inst block physical phys = 0x%08x, kv = 0x%p",
		   inst_pa, inst_ptr);

	memset(inst_ptr, 0, ram_fc_size_val_v());

	mem_wr32(inst_ptr, ram_in_page_dir_base_lo_w(),
		ram_in_page_dir_base_target_vid_mem_f() |
		ram_in_page_dir_base_vol_true_f() |
		ram_in_page_dir_base_lo_f(pde_addr_lo));

	mem_wr32(inst_ptr, ram_in_page_dir_base_hi_w(),
		ram_in_page_dir_base_hi_f(pde_addr_hi));

	mem_wr32(inst_ptr, ram_in_adr_limit_lo_w(),
		 u64_lo32(vm->va_limit) | 0xFFF);

	mem_wr32(inst_ptr, ram_in_adr_limit_hi_w(),
		ram_in_adr_limit_hi_f(u64_hi32(vm->va_limit)));

	mem_op().munmap(inst_block->mem.ref, inst_ptr);

	nvhost_dbg_info("bar1 inst block ptr: %08x",  (u32)inst_pa);
	nvhost_allocator_init(&vm->vma, "gk20a_bar1",
		1, (vm->va_limit >> 12) - 1, 1);

	vm->mapped_buffers = RB_ROOT;

	vm->alloc_va = gk20a_bar1_vm_alloc_va;
	vm->free_va =  gk20a_bar1_vm_free_va;
	vm->map =  gk20a_bar1_vm_map;
	vm->unmap =  gk20a_bar1_vm_unmap;
	vm->remove_support = gk20a_channel_vm_remove_support;

	return 0;

clean_up:
	/* free, etc */
	return err;
}

/* pmu vm, share channel_vm interfaces */
int gk20a_init_pmu_vm(struct mm_gk20a *mm)
{
	int err;
	struct mem_mgr *nvmap = mem_mgr_from_mm(mm);
	phys_addr_t inst_pa;
	void *inst_ptr;
	struct vm_gk20a *vm = &mm->pmu.vm;
	struct inst_desc *inst_block = &mm->pmu.inst_block;
	u32 pde_addr_lo;
	u32 pde_addr_hi;

	vm->mm = mm;

	mm->pmu.aperture_size = GK20A_PMU_VA_SIZE;

	nvhost_dbg_info("pmu vm size = 0x%x", mm->pmu.aperture_size);

	vm->va_start  = GK20A_PMU_VA_START;
	vm->va_limit  = vm->va_start + mm->pmu.aperture_size;

	{
		u32 pde_lo, pde_hi;
		pde_range_from_vaddr_range(vm,
					   0, vm->va_limit-1,
					   &pde_lo, &pde_hi);
		vm->pdes.num_pdes = pde_hi + 1;
	}

	vm->pdes.ptes = kzalloc(GFP_KERNEL,
				sizeof(struct page_table_gk20a) *
				vm->pdes.num_pdes);
	if (!vm->pdes.ptes)
		return -ENOMEM;

	nvhost_dbg_info("init space for pmu va_limit=0x%llx num_pdes=%d",
		   vm->va_limit, vm->pdes.num_pdes);

	/* allocate the page table directory */
	err = alloc_gmmu_pages(vm, 0, &vm->pdes.phys, &vm->pdes.ref);
	if (err)
		goto clean_up;

	err = map_gmmu_pages(vm->pdes.ref, &vm->pdes.kv);
	if (err) {
		free_gmmu_pages(vm, vm->pdes.ref, 0);
		goto clean_up;
	}
	nvhost_dbg_info("pmu pdes phys @ 0x%x",
		   vm->pdes.phys);
	/* we could release vm->pdes.kv but it's only one page... */

	pde_addr_lo = u64_lo32(vm->pdes.phys) >> 12;
	pde_addr_hi = u64_hi32(vm->pdes.phys);

	nvhost_dbg_info("pde pa=0x%x pde_addr_lo=0x%x pde_addr_hi=0x%x",
		   vm->pdes.phys, pde_addr_lo, pde_addr_hi);

	/* allocate instance mem for pmu */
	inst_block->mem.size = GK20A_PMU_INST_SIZE;
	inst_block->mem.ref =
		mem_op().alloc(nvmap, inst_block->mem.size,
			    DEFAULT_NVMAP_ALLOC_ALIGNMENT,
			    DEFAULT_NVMAP_ALLOC_FLAGS,
			    NVMAP_HEAP_CARVEOUT_GENERIC);

	if (IS_ERR(inst_block->mem.ref)) {
		inst_block->mem.ref = 0;
		err = -ENOMEM;
		goto clean_up;
	}

	inst_block->cpu_pa = inst_pa =
		mem_op().pin(nvmap, inst_block->mem.ref);

	/* IS_ERR throws a warning here (expecting void *) */
	if (inst_pa == -EINVAL || inst_pa == -EINTR) {
		inst_pa = 0;
		err = (int)inst_pa;
		goto clean_up;
	}
	nvhost_dbg_info("pmu inst block physical addr: 0x%08x",
		   inst_pa);

	inst_ptr = mem_op().mmap(inst_block->mem.ref);
	if (IS_ERR(inst_ptr)) {
		return -ENOMEM;
		goto clean_up;
	}

	memset(inst_ptr, 0, GK20A_PMU_INST_SIZE);

	mem_wr32(inst_ptr, ram_in_page_dir_base_lo_w(),
		ram_in_page_dir_base_target_vid_mem_f() |
		ram_in_page_dir_base_vol_true_f() |
		ram_in_page_dir_base_lo_f(pde_addr_lo));

	mem_wr32(inst_ptr, ram_in_page_dir_base_hi_w(),
		ram_in_page_dir_base_hi_f(pde_addr_hi));

	mem_wr32(inst_ptr, ram_in_adr_limit_lo_w(),
		 u64_lo32(vm->va_limit) | 0xFFF);

	mem_wr32(inst_ptr, ram_in_adr_limit_hi_w(),
		ram_in_adr_limit_hi_f(u64_hi32(vm->va_limit)));

	mem_op().munmap(inst_block->mem.ref, inst_ptr);

	nvhost_allocator_init(&vm->vma, "gk20a_pmu",
		(vm->va_start >> 12), (vm->va_limit >> 12) - 1, 1);

	vm->mapped_buffers = RB_ROOT;

	vm->alloc_va	    = gk20a_channel_vm_alloc_va;
	vm->free_va	    = gk20a_channel_vm_free_va;
	vm->map		    = gk20a_channel_vm_map;
	vm->unmap	    = gk20a_channel_vm_unmap;
	vm->remove_support  = gk20a_channel_vm_remove_support;

	return 0;

clean_up:
	/* free, etc */
	return err;
}

void gk20a_mm_fb_flush(struct gk20a *g)
{
	u32 data;
	s32 retry = 100;

	/* Make sure all previous writes are committed to the L2. There's no
	   guarantee that writes are to DRAM. This will be a sysmembar internal
	   to the L2. */
	gk20a_writel(g, flush_fb_flush_r(),
		flush_fb_flush_pending_busy_f());

	do {
		data = gk20a_readl(g, flush_fb_flush_r());

		if (flush_fb_flush_outstanding_v(data) ==
			flush_fb_flush_outstanding_true_v() ||
		    flush_fb_flush_pending_v(data) ==
			flush_fb_flush_pending_busy_v()) {
				nvhost_dbg_info("fb_flush 0x%x", data);
				retry--;
				udelay(20);
		} else
			break;
	}
	while (retry >= 0);

	if (retry < 0)
		nvhost_warn(dev_from_gk20a(g),
			"fb_flush too many retries");
}

void gk20a_mm_l2_flush(struct gk20a *g, bool invalidate)
{
	u32 data;
	s32 retry = 200;

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
				nvhost_dbg_info("l2_flush_dirty 0x%x", data);
				retry--;
				udelay(20);
		} else
			break;
	}
	while (retry >= 0);

	if (retry < 0)
		nvhost_warn(dev_from_gk20a(g),
			"l2_flush_dirty too many retries");

	if (!invalidate)
		return;

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
				nvhost_dbg_info("l2_system_invalidate 0x%x", data);
				retry--;
				udelay(20);
		} else
			break;
	}
	while (retry >= 0);

	if (retry < 0)
		nvhost_warn(dev_from_gk20a(g),
			"l2_system_invalidate too many retries");
}

void gk20a_mm_tlb_invalidate(struct gk20a *g, struct vm_gk20a *vm)
{
	u32 addr_lo = u64_lo32(vm->pdes.phys) >> 12;
	u32 data;
	s32 retry = 200;

	do {
		data = gk20a_readl(g, fb_mmu_ctrl_r());
		if (fb_mmu_ctrl_pri_fifo_space_v(data) != 0)
			break;
		udelay(20);
		retry--;
	} while (retry >= 0);

	if (retry < 0)
		nvhost_warn(dev_from_gk20a(g),
			"wait mmu fifo space too many retries");

	gk20a_writel(g, fb_mmu_invalidate_pdb_r(),
		fb_mmu_invalidate_pdb_addr_f(addr_lo) |
		fb_mmu_invalidate_pdb_aperture_vid_mem_f());

	gk20a_writel(g, fb_mmu_invalidate_r(),
		fb_mmu_invalidate_all_pdb_true_f() |
		fb_mmu_invalidate_all_va_true_f() |
		fb_mmu_invalidate_trigger_true_f());

	do {
		data = gk20a_readl(g, fb_mmu_ctrl_r());
		if (fb_mmu_ctrl_pri_fifo_empty_v(data) !=
			fb_mmu_ctrl_pri_fifo_empty_false_f())
			break;
		retry--;
		udelay(20);
	} while (retry >= 0);

	if (retry < 0)
		nvhost_warn(dev_from_gk20a(g),
			"mmu invalidate too many retries");
}
