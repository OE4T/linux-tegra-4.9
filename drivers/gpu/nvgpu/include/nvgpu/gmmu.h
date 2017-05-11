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

#ifndef __NVGPU_GMMU_H__
#define __NVGPU_GMMU_H__

#include <nvgpu/types.h>
#include <nvgpu/nvgpu_mem.h>

struct scatterlist;

/*
 * This is the GMMU API visible to blocks outside of the GMMU. Basically this
 * API supports all the different types of mappings that might be done in the
 * GMMU.
 */

struct vm_gk20a;
struct nvgpu_mem;

enum gmmu_pgsz_gk20a {
	gmmu_page_size_small  = 0,
	gmmu_page_size_big    = 1,
	gmmu_page_size_kernel = 2,
	gmmu_nr_page_sizes    = 3,
};

enum gk20a_mem_rw_flag {
	gk20a_mem_flag_none = 0,	/* RW */
	gk20a_mem_flag_read_only = 1,	/* RO */
	gk20a_mem_flag_write_only = 2,	/* WO */
};

/*
 * GMMU page directory. This is the kernel's tracking of a list of PDEs or PTEs
 * in the GMMU.
 */
struct nvgpu_gmmu_pd {
	/*
	 * DMA memory describing the PTEs or PTEs.
	 */
	struct nvgpu_mem	 mem;

	/*
	 * List of pointers to the next level of page tables. Does not
	 * need to be populated when this PD is pointing to PTEs.
	 */
	struct nvgpu_gmmu_pd	*entries;
	int			 num_entries;
};

/*
 * Reduce the number of arguments getting passed through the various levels of
 * GMMU mapping functions.
 *
 * The following fields are set statically and do not change throughout
 * mapping call:
 *
 *   pgsz:        Index into the page size table.
 *   kind_v:      Kind attributes for mapping.
 *   cacheable:   Cacheability of the mapping.
 *   rw_flag:     Flag from enum gk20a_mem_rw_flag
 *   sparse:      Set if the mapping should be sparse.
 *   priv:        Privilidged mapping.
 *   valid:       Set if the PTE should be marked valid.
 *   aperture:    VIDMEM or SYSMEM.
 *   debug:       When set print debugging info.
 *
 * These fields are dynamically updated as necessary during the map:
 *
 *   ctag:        Comptag line in the comptag cache;
 *                updated every time we write a PTE.
 */
struct nvgpu_gmmu_attrs {
	u32			 pgsz;
	u32			 kind_v;
	u64			 ctag;
	bool			 cacheable;
	int			 rw_flag;
	bool			 sparse;
	bool			 priv;
	bool			 valid;
	enum nvgpu_aperture	 aperture;
	bool			 debug;
};

struct gk20a_mmu_level {
	int hi_bit[2];
	int lo_bit[2];

	/*
	 * Build map from virt_addr -> phys_addr.
	 */
	void (*update_entry)(struct vm_gk20a *vm,
			     const struct gk20a_mmu_level *l,
			     struct nvgpu_gmmu_pd *pd,
			     u32 pd_idx,
			     u64 phys_addr,
			     u64 virt_addr,
			     struct nvgpu_gmmu_attrs *attrs);
	u32 entry_size;
};

static inline const char *nvgpu_gmmu_perm_str(enum gk20a_mem_rw_flag p)
{
	switch (p) {
	case gk20a_mem_flag_none:
		return "RW";
	case gk20a_mem_flag_write_only:
		return "WO";
	case gk20a_mem_flag_read_only:
		return "RO";
	default:
		return "??";
	}
}

int nvgpu_gmmu_init_page_table(struct vm_gk20a *vm);

/**
 * nvgpu_gmmu_map - Map memory into the GMMU.
 *
 * Kernel space.
 */
u64 nvgpu_gmmu_map(struct vm_gk20a *vm,
		   struct nvgpu_mem *mem,
		   u64 size,
		   u32 flags,
		   int rw_flag,
		   bool priv,
		   enum nvgpu_aperture aperture);

/**
 * nvgpu_gmmu_map_fixed - Map memory into the GMMU.
 *
 * Kernel space.
 */
u64 nvgpu_gmmu_map_fixed(struct vm_gk20a *vm,
			 struct nvgpu_mem *mem,
			 u64 addr,
			 u64 size,
			 u32 flags,
			 int rw_flag,
			 bool priv,
			 enum nvgpu_aperture aperture);

/**
 * nvgpu_gmmu_unmap - Unmap a buffer.
 *
 * Kernel space.
 */
void nvgpu_gmmu_unmap(struct vm_gk20a *vm,
		      struct nvgpu_mem *mem,
		      u64 gpu_va);

void nvgpu_free_gmmu_pages(struct vm_gk20a *vm,
		     struct nvgpu_gmmu_pd *entry);

/*
 * Some useful routines that are shared across chips.
 */
static inline u32 pd_offset_from_index(const struct gk20a_mmu_level *l,
				       u32 pd_idx)
{
	return (pd_idx * l->entry_size) / sizeof(u32);
}

static inline void pd_write(struct gk20a *g, struct nvgpu_gmmu_pd *pd,
			    size_t w, size_t data)
{
	nvgpu_mem_wr32(g, &pd->mem, w, data);
}


/*
 * Internal debugging routines. Probably not something you want to use.
 */
#define pte_dbg(g, attrs, fmt, args...)					\
	do {								\
		if (attrs && attrs->debug)				\
			nvgpu_info(g, fmt, ##args);			\
		else							\
			nvgpu_log(g, gpu_dbg_pte, fmt, ##args);		\
	} while (0)

#endif
