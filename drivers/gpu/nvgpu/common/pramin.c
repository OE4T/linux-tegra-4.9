/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
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

#include <nvgpu/pramin.h>
#include <nvgpu/page_allocator.h>
#include <nvgpu/enabled.h>

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
	struct nvgpu_sgt *sgt;
	void *sgl;
	u32 byteoff, start_reg, until_end, n;

	/*
	 * TODO: Vidmem is not accesible through pramin on shutdown path.
	 * driver should be refactored to prevent this from happening, but for
	 * now it is ok just to ignore the writes
	 */
	if (!g->regs && nvgpu_is_enabled(g, NVGPU_DRIVER_IS_DYING))
		return;

	alloc = get_vidmem_page_alloc(mem->priv.sgt->sgl);
	sgt = &alloc->sgt;
	for (sgl = sgt->sgl; sgl; sgl = nvgpu_sgt_get_next(sgt, sgl)) {
		if (offset >= nvgpu_sgt_get_length(sgt, sgl))
			offset -= nvgpu_sgt_get_length(sgt, sgl);
		else
			break;
	}

	while (size) {
		u32 sgl_len = (u32)nvgpu_sgt_get_length(sgt, sgl);

		byteoff = g->ops.pramin.enter(g, mem, sgt, sgl,
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
			sgl = nvgpu_sgt_get_next(sgt, sgl);
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
