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

#include <nvgpu/mem_desc.h>
#include <nvgpu/page_allocator.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"

u32 __nvgpu_aperture_mask(struct gk20a *g, enum nvgpu_aperture aperture,
		u32 sysmem_mask, u32 vidmem_mask)
{
	switch (aperture) {
	case APERTURE_SYSMEM:
		/* sysmem for dgpus; some igpus consider system memory vidmem */
		return g->mm.vidmem_is_vidmem ? sysmem_mask : vidmem_mask;
	case APERTURE_VIDMEM:
		/* for dgpus only */
		return vidmem_mask;
	case APERTURE_INVALID:
		WARN_ON("Bad aperture");
	}
	return 0;
}

u32 nvgpu_aperture_mask(struct gk20a *g, struct mem_desc *mem,
		u32 sysmem_mask, u32 vidmem_mask)
{
	return __nvgpu_aperture_mask(g, mem->aperture,
			sysmem_mask, vidmem_mask);
}

int nvgpu_mem_begin(struct gk20a *g, struct mem_desc *mem)
{
	void *cpu_va;

	if (mem->aperture != APERTURE_SYSMEM || g->mm.force_pramin)
		return 0;

	if (WARN_ON(mem->cpu_va)) {
		gk20a_warn(dev_from_gk20a(g), "nested %s", __func__);
		return -EBUSY;
	}

	cpu_va = vmap(mem->pages,
			PAGE_ALIGN(mem->size) >> PAGE_SHIFT,
			0, pgprot_writecombine(PAGE_KERNEL));

	if (WARN_ON(!cpu_va))
		return -ENOMEM;

	mem->cpu_va = cpu_va;
	return 0;
}

void nvgpu_mem_end(struct gk20a *g, struct mem_desc *mem)
{
	if (mem->aperture != APERTURE_SYSMEM || g->mm.force_pramin)
		return;

	vunmap(mem->cpu_va);
	mem->cpu_va = NULL;
}

u32 nvgpu_mem_rd32(struct gk20a *g, struct mem_desc *mem, u32 w)
{
	u32 data = 0;

	if (mem->aperture == APERTURE_SYSMEM && !g->mm.force_pramin) {
		u32 *ptr = mem->cpu_va;

		WARN_ON(!ptr);
		data = ptr[w];
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
		gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr + w, data);
#endif
	} else if (mem->aperture == APERTURE_VIDMEM || g->mm.force_pramin) {
		u32 value;
		u32 *p = &value;

		nvgpu_pramin_access_batched(g, mem, w * sizeof(u32),
				sizeof(u32), pramin_access_batch_rd_n, &p);

		data = value;

	} else {
		WARN_ON("Accessing unallocated mem_desc");
	}

	return data;
}

u32 nvgpu_mem_rd(struct gk20a *g, struct mem_desc *mem, u32 offset)
{
	WARN_ON(offset & 3);
	return nvgpu_mem_rd32(g, mem, offset / sizeof(u32));
}

void nvgpu_mem_rd_n(struct gk20a *g, struct mem_desc *mem,
		u32 offset, void *dest, u32 size)
{
	WARN_ON(offset & 3);
	WARN_ON(size & 3);

	if (mem->aperture == APERTURE_SYSMEM && !g->mm.force_pramin) {
		u8 *src = (u8 *)mem->cpu_va + offset;

		WARN_ON(!mem->cpu_va);
		memcpy(dest, src, size);
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
		if (size)
			gk20a_dbg(gpu_dbg_mem, " %p = 0x%x ... [%d bytes]",
					src, *dest, size);
#endif
	} else if (mem->aperture == APERTURE_VIDMEM || g->mm.force_pramin) {
		u32 *dest_u32 = dest;

		nvgpu_pramin_access_batched(g, mem, offset, size,
				pramin_access_batch_rd_n, &dest_u32);
	} else {
		WARN_ON("Accessing unallocated mem_desc");
	}
}

void nvgpu_mem_wr32(struct gk20a *g, struct mem_desc *mem, u32 w, u32 data)
{
	if (mem->aperture == APERTURE_SYSMEM && !g->mm.force_pramin) {
		u32 *ptr = mem->cpu_va;

		WARN_ON(!ptr);
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
		gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr + w, data);
#endif
		ptr[w] = data;
	} else if (mem->aperture == APERTURE_VIDMEM || g->mm.force_pramin) {
		u32 value = data;
		u32 *p = &value;

		nvgpu_pramin_access_batched(g, mem, w * sizeof(u32),
				sizeof(u32), pramin_access_batch_wr_n, &p);
		if (!mem->skip_wmb)
			wmb();
	} else {
		WARN_ON("Accessing unallocated mem_desc");
	}
}

void nvgpu_mem_wr(struct gk20a *g, struct mem_desc *mem, u32 offset, u32 data)
{
	WARN_ON(offset & 3);
	nvgpu_mem_wr32(g, mem, offset / sizeof(u32), data);
}

void nvgpu_mem_wr_n(struct gk20a *g, struct mem_desc *mem, u32 offset,
		void *src, u32 size)
{
	WARN_ON(offset & 3);
	WARN_ON(size & 3);

	if (mem->aperture == APERTURE_SYSMEM && !g->mm.force_pramin) {
		u8 *dest = (u8 *)mem->cpu_va + offset;

		WARN_ON(!mem->cpu_va);
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
		if (size)
			gk20a_dbg(gpu_dbg_mem, " %p = 0x%x ... [%d bytes]",
					dest, *src, size);
#endif
		memcpy(dest, src, size);
	} else if (mem->aperture == APERTURE_VIDMEM || g->mm.force_pramin) {
		u32 *src_u32 = src;

		nvgpu_pramin_access_batched(g, mem, offset, size,
				pramin_access_batch_wr_n, &src_u32);
		if (!mem->skip_wmb)
			wmb();
	} else {
		WARN_ON("Accessing unallocated mem_desc");
	}
}

void nvgpu_memset(struct gk20a *g, struct mem_desc *mem, u32 offset,
		u32 c, u32 size)
{
	WARN_ON(offset & 3);
	WARN_ON(size & 3);
	WARN_ON(c & ~0xff);

	c &= 0xff;

	if (mem->aperture == APERTURE_SYSMEM && !g->mm.force_pramin) {
		u8 *dest = (u8 *)mem->cpu_va + offset;

		WARN_ON(!mem->cpu_va);
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
		if (size)
			gk20a_dbg(gpu_dbg_mem, " %p = 0x%x [times %d]",
				dest, c, size);
#endif
		memset(dest, c, size);
	} else if (mem->aperture == APERTURE_VIDMEM || g->mm.force_pramin) {
		u32 repeat_value = c | (c << 8) | (c << 16) | (c << 24);
		u32 *p = &repeat_value;

		nvgpu_pramin_access_batched(g, mem, offset, size,
				pramin_access_batch_set, &p);
		if (!mem->skip_wmb)
			wmb();
	} else {
		WARN_ON("Accessing unallocated mem_desc");
	}
}
