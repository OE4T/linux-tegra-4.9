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

#ifndef __NVGPU_MEM_DESC_H__
#define __NVGPU_MEM_DESC_H__

#include <linux/types.h>

#include <nvgpu/list.h>

struct page;
struct sg_table;

struct gk20a;
struct nvgpu_allocator;

/*
 * Real location of a buffer - gk20a_aperture_mask() will deduce what will be
 * told to the gpu about the aperture, but this flag designates where the
 * memory actually was allocated from.
 */
enum gk20a_aperture {
	APERTURE_INVALID, /* unallocated or N/A */
	APERTURE_SYSMEM,
	APERTURE_VIDMEM
};

struct mem_desc {
	void *cpu_va; /* sysmem only */
	struct page **pages; /* sysmem only */
	struct sg_table *sgt;
	enum gk20a_aperture aperture;
	size_t size;
	u64 gpu_va;
	bool fixed; /* vidmem only */
	bool user_mem; /* vidmem only */
	struct nvgpu_allocator *allocator; /* vidmem only */
	struct nvgpu_list_node clear_list_entry; /* vidmem only */
	bool skip_wmb;
	unsigned long flags;
};

static inline struct mem_desc *
mem_desc_from_clear_list_entry(struct nvgpu_list_node *node)
{
	return (struct mem_desc *)
		((uintptr_t)node - offsetof(struct mem_desc, clear_list_entry));
};

struct mem_desc_sub {
	u32 offset;
	u32 size;
};

static inline const char *gk20a_aperture_str(enum gk20a_aperture aperture)
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

int gk20a_mem_begin(struct gk20a *g, struct mem_desc *mem);
/* nop for null mem, like with free() or vunmap() */
void gk20a_mem_end(struct gk20a *g, struct mem_desc *mem);

/* word-indexed offset */
u32 gk20a_mem_rd32(struct gk20a *g, struct mem_desc *mem, u32 w);
/* byte offset (32b-aligned) */
u32 gk20a_mem_rd(struct gk20a *g, struct mem_desc *mem, u32 offset);
/* memcpy to cpu, offset and size in bytes (32b-aligned) */
void gk20a_mem_rd_n(struct gk20a *g, struct mem_desc *mem, u32 offset,
		void *dest, u32 size);

/* word-indexed offset */
void gk20a_mem_wr32(struct gk20a *g, struct mem_desc *mem, u32 w, u32 data);
/* byte offset (32b-aligned) */
void gk20a_mem_wr(struct gk20a *g, struct mem_desc *mem, u32 offset, u32 data);
/* memcpy from cpu, offset and size in bytes (32b-aligned) */
void gk20a_mem_wr_n(struct gk20a *g, struct mem_desc *mem, u32 offset,
		void *src, u32 size);
/* size and offset in bytes (32b-aligned), filled with the constant byte c */
void gk20a_memset(struct gk20a *g, struct mem_desc *mem, u32 offset,
		u32 c, u32 size);

u32 __gk20a_aperture_mask(struct gk20a *g, enum gk20a_aperture aperture,
		u32 sysmem_mask, u32 vidmem_mask);
u32 gk20a_aperture_mask(struct gk20a *g, struct mem_desc *mem,
		u32 sysmem_mask, u32 vidmem_mask);

#endif
