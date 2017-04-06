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

#ifndef __NVGPU_NVGPU_MEM_H__
#define __NVGPU_NVGPU_MEM_H__

#include <nvgpu/types.h>
#include <nvgpu/list.h>

#include <nvgpu/linux/nvgpu_mem.h>

struct page;
struct sg_table;

struct gk20a;
struct nvgpu_allocator;

/*
 * Real location of a buffer - nvgpu_aperture_mask() will deduce what will be
 * told to the gpu about the aperture, but this flag designates where the
 * memory actually was allocated from.
 */
enum nvgpu_aperture {
	APERTURE_INVALID, /* unallocated or N/A */
	APERTURE_SYSMEM,
	APERTURE_VIDMEM
};

struct nvgpu_mem {
	/*
	 * Populated for all nvgpu_mem structs - vidmem or system.
	 */
	enum nvgpu_aperture			 aperture;
	size_t					 size;
	u64					 gpu_va;
	bool					 skip_wmb;

	/*
	 * Only populated for a sysmem allocation.
	 */
	void					*cpu_va;

	/*
	 * Fields only populated for vidmem allocations.
	 */
	bool					 fixed;
	bool					 user_mem;
	struct nvgpu_allocator			*allocator;
	struct nvgpu_list_node			 clear_list_entry;

	/*
	 * This is defined by the system specific header. It can be empty if
	 * there's no system specific stuff for a given system.
	 */
	struct nvgpu_mem_priv			 priv;
};

static inline struct nvgpu_mem *
nvgpu_mem_from_clear_list_entry(struct nvgpu_list_node *node)
{
	return (struct nvgpu_mem *)
		((uintptr_t)node - offsetof(struct nvgpu_mem,
					    clear_list_entry));
};

static inline const char *nvgpu_aperture_str(enum nvgpu_aperture aperture)
{
	switch (aperture) {
		case APERTURE_INVALID: return "invalid";
		case APERTURE_SYSMEM:  return "sysmem";
		case APERTURE_VIDMEM:  return "vidmem";
	};
	return "UNKNOWN";
}

/*
 * Buffer accessors - wrap between begin() and end() if there is no permanent
 * kernel mapping for this buffer.
 */

int nvgpu_mem_begin(struct gk20a *g, struct nvgpu_mem *mem);
/* nop for null mem, like with free() or vunmap() */
void nvgpu_mem_end(struct gk20a *g, struct nvgpu_mem *mem);

/* word-indexed offset */
u32 nvgpu_mem_rd32(struct gk20a *g, struct nvgpu_mem *mem, u32 w);
/* byte offset (32b-aligned) */
u32 nvgpu_mem_rd(struct gk20a *g, struct nvgpu_mem *mem, u32 offset);
/* memcpy to cpu, offset and size in bytes (32b-aligned) */
void nvgpu_mem_rd_n(struct gk20a *g, struct nvgpu_mem *mem, u32 offset,
		void *dest, u32 size);

/* word-indexed offset */
void nvgpu_mem_wr32(struct gk20a *g, struct nvgpu_mem *mem, u32 w, u32 data);
/* byte offset (32b-aligned) */
void nvgpu_mem_wr(struct gk20a *g, struct nvgpu_mem *mem, u32 offset, u32 data);
/* memcpy from cpu, offset and size in bytes (32b-aligned) */
void nvgpu_mem_wr_n(struct gk20a *g, struct nvgpu_mem *mem, u32 offset,
		void *src, u32 size);
/* size and offset in bytes (32b-aligned), filled with the constant byte c */
void nvgpu_memset(struct gk20a *g, struct nvgpu_mem *mem, u32 offset,
		u32 c, u32 size);

u32 __nvgpu_aperture_mask(struct gk20a *g, enum nvgpu_aperture aperture,
		u32 sysmem_mask, u32 vidmem_mask);
u32 nvgpu_aperture_mask(struct gk20a *g, struct nvgpu_mem *mem,
		u32 sysmem_mask, u32 vidmem_mask);

#endif
