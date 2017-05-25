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

#include <nvgpu/pramin.h>
#include <nvgpu/page_allocator.h>

#include "gk20a/gk20a.h"

/*
 * Flip this to force all gk20a_mem* accesses via PRAMIN from the start of the
 * boot, even for buffers that would work via cpu_va. In runtime, the flag is
 * in debugfs, called "force_pramin".
 */
#define GK20A_FORCE_PRAMIN_DEFAULT false

void pramin_access_batch_rd_n(struct gk20a *g, u32 start, u32 words, u32 **arg)
{
	u32 r = start, *dest_u32 = *arg;

	if (!g->regs) {
		__gk20a_warn_on_no_regs();
		return;
	}

	while (words--) {
		*dest_u32++ = gk20a_readl(g, r);
		r += sizeof(u32);
	}

	*arg = dest_u32;
}

void pramin_access_batch_wr_n(struct gk20a *g, u32 start, u32 words, u32 **arg)
{
	u32 r = start, *src_u32 = *arg;

	if (!g->regs) {
		__gk20a_warn_on_no_regs();
		return;
	}

	while (words--) {
		writel_relaxed(*src_u32++, g->regs + r);
		r += sizeof(u32);
	}

	*arg = src_u32;
}

void pramin_access_batch_set(struct gk20a *g, u32 start, u32 words, u32 **arg)
{
	u32 r = start, repeat = **arg;

	if (!g->regs) {
		__gk20a_warn_on_no_regs();
		return;
	}

	while (words--) {
		writel_relaxed(repeat, g->regs + r);
		r += sizeof(u32);
	}
}

/*
 * The PRAMIN range is 1 MB, must change base addr if a buffer crosses that.
 * This same loop is used for read/write/memset. Offset and size in bytes.
 * One call to "loop" is done per range, with "arg" supplied.
 */
void nvgpu_pramin_access_batched(struct gk20a *g, struct nvgpu_mem *mem,
		u32 offset, u32 size, pramin_access_batch_fn loop, u32 **arg)
{
	struct nvgpu_page_alloc *alloc = NULL;
	struct nvgpu_mem_sgl *sgl;
	u32 byteoff, start_reg, until_end, n;

	alloc = get_vidmem_page_alloc(mem->priv.sgt->sgl);
	sgl = alloc->sgl;
	while (sgl) {
		if (offset >= nvgpu_mem_sgl_length(sgl)) {
			offset -= nvgpu_mem_sgl_length(sgl);
			sgl = sgl->next;
		} else {
			break;
		}
	}

	while (size) {
		u32 sgl_len = (u32)nvgpu_mem_sgl_length(sgl);

		byteoff = g->ops.pramin.enter(g, mem, sgl,
					      offset / sizeof(u32));
		start_reg = g->ops.pramin.data032_r(byteoff / sizeof(u32));
		until_end = SZ_1M - (byteoff & (SZ_1M - 1));

		n = min3(size, until_end, (u32)(sgl_len - offset));

		loop(g, start_reg, n / sizeof(u32), arg);

		/* read back to synchronize accesses */
		gk20a_readl(g, start_reg);
		g->ops.pramin.exit(g, mem, sgl);

		size -= n;

		if (n == (sgl_len - offset)) {
			sgl = nvgpu_mem_sgl_next(sgl);
			offset = 0;
		} else {
			offset += n;
		}
	}
}

void nvgpu_init_pramin(struct mm_gk20a *mm)
{
	mm->pramin_window = 0;
	nvgpu_spinlock_init(&mm->pramin_window_lock);
	mm->force_pramin = GK20A_FORCE_PRAMIN_DEFAULT;
}
