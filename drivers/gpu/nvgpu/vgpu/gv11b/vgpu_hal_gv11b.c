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

#include <gk20a/gk20a.h>
#include <gv11b/hal_gv11b.h>
#include <vgpu/vgpu.h>
#include <vgpu/vgpu_t19x.h>
#include <vgpu/gp10b/vgpu_mm_gp10b.h>

#include "vgpu_gr_gv11b.h"
#include "vgpu_fifo_gv11b.h"

int vgpu_gv11b_init_hal(struct gk20a *g)
{
	int err;

	gk20a_dbg_fn("");

	err = gv11b_init_hal(g);
	if (err)
		return err;

	vgpu_init_hal_common(g);
	vgpu_gp10b_init_mm_ops(&g->ops);

	vgpu_gv11b_init_gr_ops(&g->ops);
	vgpu_gv11b_init_fifo_ops(&g->ops);

	return 0;
}
