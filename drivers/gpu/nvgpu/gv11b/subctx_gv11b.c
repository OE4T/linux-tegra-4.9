/*
 * Volta GPU series Subcontext
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program.
 */

#include "gk20a/gk20a.h"

#include "gv11b/subctx_gv11b.h"

#include <nvgpu/dma.h>
#include <nvgpu/log.h>

#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ctxsw_prog_gv11b.h>

static void gv11b_init_subcontext_pdb(struct channel_gk20a *c,
				struct nvgpu_mem *inst_block);

void gv11b_free_subctx_header(struct channel_gk20a *c)
{
	struct ctx_header_desc *ctx = &c->ch_ctx.ctx_header;
	struct gk20a *g = c->g;

	gk20a_dbg_fn("");

	if (ctx->mem.gpu_va) {
		gk20a_gmmu_unmap(c->vm, ctx->mem.gpu_va,
			ctx->mem.size, gk20a_mem_flag_none);

		nvgpu_dma_free(g, &ctx->mem);
	}
}

int gv11b_alloc_subctx_header(struct channel_gk20a *c)
{
	struct ctx_header_desc *ctx = &c->ch_ctx.ctx_header;
	struct gk20a *g = c->g;
	struct gr_gk20a *gr = &g->gr;
	int ret = 0;

	gk20a_dbg_fn("");

	if (ctx->mem.gpu_va == 0) {
		ret = nvgpu_dma_alloc_flags_sys(g,
				NVGPU_DMA_NO_KERNEL_MAPPING,
				gr->ctx_vars.golden_image_size,
				&ctx->mem);
		if (ret) {
			nvgpu_err(g, "failed to allocate sub ctx header");
			return ret;
		}
		ctx->mem.gpu_va = gk20a_gmmu_map(c->vm,
					&ctx->mem.sgt,
					ctx->mem.size,
					NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_TRUE,
					gk20a_mem_flag_none, true,
					ctx->mem.aperture);
		if (!ctx->mem.gpu_va) {
			nvgpu_err(g, "failed to map ctx header");
			nvgpu_dma_free(g, &ctx->mem);
			return -ENOMEM;
		}
		/* Now clear the buffer */
		if (nvgpu_mem_begin(g, &ctx->mem))
			return -ENOMEM;

		nvgpu_memset(g, &ctx->mem, 0, 0, ctx->mem.size);
		nvgpu_mem_end(g, &ctx->mem);

		gv11b_init_subcontext_pdb(c, &c->inst_block);

	}
	return ret;
}

static void gv11b_init_subcontext_pdb(struct channel_gk20a *c,
				struct nvgpu_mem *inst_block)
{
	struct gk20a *g = c->g;
	struct vm_gk20a *vm;
	u64 pdb_addr, pdb_addr_lo, pdb_addr_hi;
	u32 format_word;
	u32 lo, hi;

	gk20a_dbg_fn("");
	/* load main pdb as veid0 pdb also */
	vm = c->vm;
	pdb_addr = g->ops.mm.get_iova_addr(g, vm->pdb.mem.sgt->sgl, 0);
	pdb_addr_lo = u64_lo32(pdb_addr >> ram_in_base_shift_v());
	pdb_addr_hi = u64_hi32(pdb_addr);
	format_word = ram_in_sc_page_dir_base_target_f(
		ram_in_sc_page_dir_base_target_sys_mem_ncoh_v(), 0) |
		ram_in_sc_page_dir_base_vol_f(
		ram_in_sc_page_dir_base_vol_true_v(), 0) |
		ram_in_sc_page_dir_base_fault_replay_tex_f(0, 0) |
		ram_in_sc_page_dir_base_fault_replay_gcc_f(0, 0) |
		ram_in_sc_use_ver2_pt_format_f(1, 0) |
		ram_in_sc_big_page_size_f(1, 0) |
		ram_in_sc_page_dir_base_lo_0_f(pdb_addr_lo);
	lo = ram_in_sc_page_dir_base_vol_0_w();
	hi = ram_in_sc_page_dir_base_hi_0_w();
	nvgpu_mem_wr32(g, inst_block, lo, format_word);
	nvgpu_mem_wr32(g, inst_block, hi, pdb_addr_hi);

	/* make subcontext0 address space to valid */
	/* TODO fix proper hw register definations */
	nvgpu_mem_wr32(g, inst_block, 166, 0x1);
	nvgpu_mem_wr32(g, inst_block, 167, 0);
	nvgpu_mem_wr32(g, inst_block, ram_in_engine_wfi_veid_w(),
			ram_in_engine_wfi_veid_f(0));

}

int gv11b_update_subctx_header(struct channel_gk20a *c, u64 gpu_va)
{
	struct ctx_header_desc *ctx = &c->ch_ctx.ctx_header;
	struct nvgpu_mem *gr_mem;
	struct gk20a *g = c->g;
	int ret = 0;
	u32 addr_lo, addr_hi;

	addr_lo = u64_lo32(gpu_va);
	addr_hi = u64_hi32(gpu_va);

	gr_mem = &ctx->mem;
	g->ops.mm.l2_flush(g, true);
	if (nvgpu_mem_begin(g, gr_mem))
		return -ENOMEM;

	nvgpu_mem_wr(g, gr_mem,
		ctxsw_prog_main_image_context_buffer_ptr_hi_o(), addr_hi);
	nvgpu_mem_wr(g, gr_mem,
		ctxsw_prog_main_image_context_buffer_ptr_o(), addr_lo);
	nvgpu_mem_end(g, gr_mem);
	return ret;
}
