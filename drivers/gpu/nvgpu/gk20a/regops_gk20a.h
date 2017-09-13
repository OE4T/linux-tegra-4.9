/*
 * Tegra GK20A GPU Debugger Driver Register Ops
 *
 * Copyright (c) 2013-2017, NVIDIA CORPORATION. All rights reserved.
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
#ifndef REGOPS_GK20A_H
#define REGOPS_GK20A_H

#include <uapi/linux/nvgpu.h>

struct regop_offset_range {
	u32 base:24;
	u32 count:8;
};

int exec_regops_gk20a(struct dbg_session_gk20a *dbg_s,
		      struct nvgpu_dbg_gpu_reg_op *ops,
		      u64 num_ops);

/* turn seriously unwieldy names -> something shorter */
#define REGOP(x) NVGPU_DBG_GPU_REG_OP_##x

static inline bool reg_op_is_gr_ctx(u8 type)
{
	return  type == REGOP(TYPE_GR_CTX) ||
		type == REGOP(TYPE_GR_CTX_TPC) ||
		type == REGOP(TYPE_GR_CTX_SM) ||
		type == REGOP(TYPE_GR_CTX_CROP) ||
		type == REGOP(TYPE_GR_CTX_ZROP) ||
		type == REGOP(TYPE_GR_CTX_QUAD);
}
static inline bool reg_op_is_read(u8 op)
{
	return  op == REGOP(READ_32) ||
		op == REGOP(READ_64) ;
}

bool is_bar0_global_offset_whitelisted_gk20a(struct gk20a *g, u32 offset);

#endif /* REGOPS_GK20A_H */
