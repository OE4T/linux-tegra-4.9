/*
 * drivers/video/tegra/host/gk20a/mm_gk20a.h
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
#ifndef __MM_GK20A_H__
#define __MM_GK20A_H__

#include "../nvhost_allocator.h"

struct mem_desc {
	struct mem_handle *ref;
	u32 size;
};

struct mem_desc_sub {
	u32 offset;
	u32 size;
};

struct gpfifo_desc {
	struct mem_desc mem;
	u32 entry_num;

	u32 get;
	u32 put;

	bool wrap;

	struct gpfifo *cpu_va;
	u64 gpu_va;
};

struct mmu_desc {
	struct mem_desc mem;
	phys_addr_t cpu_pa;
};

struct inst_desc {
	struct mem_desc mem;
	phys_addr_t cpu_pa;
};

struct userd_desc {
	struct mem_desc mem;
	void *cpu_va;
	phys_addr_t cpu_pa;
	u64 gpu_va;
};

struct patch_desc {
	struct mem_desc mem;
	u64 gpu_va;
	u32 data_count;
};

struct pmu_mem_desc {
	struct mem_desc mem;
	phys_addr_t cpu_pa;
	u64 pmu_va;
};

struct zcull_ctx_desc {
	struct mem_desc mem;
	u64 gpu_va;
	u32 ctx_attr;
	u32 ctx_sw_mode;
};

struct pm_ctx_desc {
	struct mem_desc mem;
	u64 gpu_va;
	u32 ctx_attr;
	u32 ctx_sw_mode;
};

struct gr_ctx_desc {
	struct mem_desc mem;
	u64 gpu_va;
};

struct compbit_store_desc {
	struct mem_desc mem;
	phys_addr_t base_pa;
	u32 alignment;
};

struct page_table_gk20a {
	/* backing for */
	/* Either a *page or a *mem_handle */
	void *ref;
	/* track mapping cnt on this page table */
	u32 ref_cnt;
};

struct page_directory_gk20a {
	/* backing for */
	u32 num_pdes;
	void *kv;
	phys_addr_t phys;
	/* Either a *page or a *mem_handle */
	void *ref;
	bool dirty;

	struct page_table_gk20a *ptes;
};

struct mapped_buffer_node {
	struct rb_node node;
	u64 addr;
	u64 size;
	struct mem_mgr *memmgr;
	struct mem_handle *handle_ref;
	u32 page_size;
	u32 ctag_offset;
	u32 ctag_lines;
};

struct vm_gk20a {
	struct mm_gk20a *mm;
	struct nvhost_as_share *as_share; /* as_share this represents */

	u64 va_start;
	u64 va_limit;

	struct page_directory_gk20a pdes;

	struct nvhost_allocator vma; /* page interval allocator */
	struct rb_root mapped_buffers;

	u64 (*alloc_va)(struct vm_gk20a *vm,
			u64 size,
			u32 page_size);

	void (*free_va)(struct vm_gk20a *vm, u64 offset, u64 size, u32 page_size);

	u64 (*map)(struct vm_gk20a *vm,
		   struct mem_mgr *memmgr,
		   struct mem_handle *r,
		   u64 offset_align,
		   u32 flags /*NVHOST_MAP_BUFFER_FLAGS_*/,
		   u32 kind);

	void (*unmap)(struct vm_gk20a *vm,
		      u64 offset);

	void (*remove_support)(struct vm_gk20a *vm);
};

struct gk20a;
struct channel_gk20a;

int gk20a_init_mm_support(struct gk20a *g, bool reinit);
int gk20a_init_bar1_vm(struct mm_gk20a *mm);
int gk20a_init_pmu_vm(struct mm_gk20a *mm);

void gk20a_mm_fb_flush(struct gk20a *g);
void gk20a_mm_l2_flush(struct gk20a *g, bool invalidate);
void gk20a_mm_tlb_invalidate(struct gk20a *g, struct vm_gk20a *vm);

struct mm_gk20a {
	struct gk20a *g;

	u32 big_page_size;
	u32 pde_stride;
	u32 pde_stride_shift;

	struct {
		u32 order;
		u32 num_ptes;
	} page_table_sizing[2];


	struct {
		u64 size;
	} channel;

	struct {
		u32 aperture_size;
		struct vm_gk20a vm;
		struct inst_desc inst_block;
	} bar1;

	struct {
		u32 aperture_size;
		struct vm_gk20a vm;
		struct inst_desc inst_block;
	} pmu;
};
int gk20a_mm_init(struct mm_gk20a *mm);

#define gk20a_from_mm(mm) ((mm)->g)
#define gk20a_from_vm(vm) ((vm)->mm->g)

#define mem_mgr_from_mm(mm) (gk20a_from_mm(mm)->host->memmgr)
#define mem_mgr_from_vm(vm) (gk20a_from_vm(vm)->host->memmgr)
#define dev_from_vm(vm) dev_from_gk20a(vm->mm->g)

#define DEFAULT_NVMAP_ALLOC_FLAGS (NVMAP_HANDLE_UNCACHEABLE)
#define DEFAULT_NVMAP_ALLOC_ALIGNMENT (4*1024)

static inline int bar1_aperture_size_mb_gk20a(void)
{
	return 128; /*TBD read this from fuses?*/
}
/* max address bits */
static inline int max_physaddr_bits_gk20a(void)
{
	return 40;/*"old" sys physaddr, meaningful? */
}
static inline int max_vid_physaddr_bits_gk20a(void)
{
	/* "vid phys" is asid/smmu phys?,
	 * i.e. is this the real sys physaddr? */
	return 37;
}
static inline int max_vaddr_bits_gk20a(void)
{
	return 40; /* chopped for area? */
}

#if 0 /*related to addr bits above, concern below TBD on which is accurate */
#define bar1_instance_block_shift_gk20a() (max_physaddr_bits_gk20a() -\
					   bus_bar1_block_ptr_s())
#else
#define bar1_instance_block_shift_gk20a() bus_bar1_block_ptr_shift_v()
#endif

#endif /*_MM_GK20A_H_ */
