/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "common/linux/vgpu/vgpu.h"
#include <nvgpu/vgpu/tegra_vgpu.h>
#include <nvgpu/hw/gv11b/hw_ctxsw_prog_gv11b.h>

int vgpu_gv11b_alloc_subctx_header(struct channel_gk20a *c)
{
	struct ctx_header_desc *ctx = &c->ctx_header;
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_alloc_ctx_header_params *p =
				&msg.params.alloc_ctx_header;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_ALLOC_CTX_HEADER;
	msg.handle = vgpu_get_handle(c->g);
	p->ch_handle = c->virt_ctx;
	p->ctx_header_va = __nvgpu_vm_alloc_va(c->vm,
				ctxsw_prog_fecs_header_v(),
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
	struct ctx_header_desc *ctx = &c->ctx_header;
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_free_ctx_header_params *p =
				&msg.params.free_ctx_header;
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
