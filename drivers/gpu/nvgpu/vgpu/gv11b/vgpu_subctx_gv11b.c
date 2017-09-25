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
#include <vgpu/vgpu.h>
#include <linux/tegra_vgpu.h>

int vgpu_gv11b_alloc_subctx_header(struct channel_gk20a *c)
{
	struct ctx_header_desc *ctx = &c->ch_ctx.ctx_header;
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_alloc_ctx_header_params *p =
				&msg.params.t19x.alloc_ctx_header;
	struct gr_gk20a *gr = &c->g->gr;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_ALLOC_CTX_HEADER;
	msg.handle = vgpu_get_handle(c->g);
	p->ch_handle = c->virt_ctx;
	p->ctx_header_va = __nvgpu_vm_alloc_va(c->vm,
				gr->ctx_vars.golden_image_size,
				gmmu_page_size_kernel);
	if (!p->ctx_header_va) {
		nvgpu_err(c->g, "alloc va failed for ctx_header");
		return -ENOMEM;
	}
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (unlikely(err)) {
		nvgpu_err(c->g, "alloc ctx_header failed err %d", err);
		__nvgpu_vm_free_va(c->vm, p->ctx_header_va,
			gmmu_page_size_kernel);
		return err;
	}
	ctx->mem.gpu_va = p->ctx_header_va;

	return err;
}

void vgpu_gv11b_free_subctx_header(struct channel_gk20a *c)
{
	struct ctx_header_desc *ctx = &c->ch_ctx.ctx_header;
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_free_ctx_header_params *p =
				&msg.params.t19x.free_ctx_header;
	int err;

	if (ctx->mem.gpu_va) {
		msg.cmd = TEGRA_VGPU_CMD_FREE_CTX_HEADER;
		msg.handle = vgpu_get_handle(c->g);
		p->ch_handle = c->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		err = err ? err : msg.ret;
		if (unlikely(err))
			nvgpu_err(c->g, "free ctx_header failed err %d", err);
		__nvgpu_vm_free_va(c->vm, ctx->mem.gpu_va,
				gmmu_page_size_kernel);
		ctx->mem.gpu_va = 0;
	}
}
