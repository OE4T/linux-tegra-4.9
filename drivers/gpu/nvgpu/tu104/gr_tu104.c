<<<<<<< HEAD   (bbef4c gpu: nvgpu: initialize masks for the perfmon counters 3)
=======
/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/types.h>
#include <nvgpu/soc.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>
#include <nvgpu/netlist.h>

#include "gk20a/gr_gk20a.h"
#include "gk20a/gr_pri_gk20a.h"

#include "gp10b/gr_gp10b.h"

#include "gv11b/gr_gv11b.h"

#include "tu104/gr_tu104.h"

#include <nvgpu/hw/tu104/hw_gr_tu104.h>

bool gr_tu104_is_valid_class(struct gk20a *g, u32 class_num)
{
	nvgpu_speculation_barrier();
	switch (class_num) {
	case TURING_CHANNEL_GPFIFO_A:
	case TURING_A:
	case TURING_COMPUTE_A:
	case TURING_DMA_COPY_A:
		return true;
	default:
		break;
	}

	return gr_gv11b_is_valid_class(g, class_num);
};

bool gr_tu104_is_valid_gfx_class(struct gk20a *g, u32 class_num)
{
	nvgpu_speculation_barrier();
	switch (class_num) {
	case TURING_A:
		return true;
	default:
		break;
	}

	return gr_gv11b_is_valid_gfx_class(g, class_num);
}

bool gr_tu104_is_valid_compute_class(struct gk20a *g, u32 class_num)
{
	nvgpu_speculation_barrier();
	switch (class_num) {
	case TURING_COMPUTE_A:
		return true;
	default:
		break;
	}

	return gr_gv11b_is_valid_compute_class(g, class_num);
}

int gr_tu104_init_sw_bundle64(struct gk20a *g)
{
	u32 i;
	u32 last_bundle_data_lo = 0;
	u32 last_bundle_data_hi = 0;
	int err = 0;
	struct netlist_av64_list *sw_bundle64_init =
			&g->netlist_vars->sw_bundle64_init;

	for (i = 0U; i < sw_bundle64_init->count; i++) {
		if (i == 0U ||
		   (last_bundle_data_lo != sw_bundle64_init->l[i].value_lo) ||
		   (last_bundle_data_hi != sw_bundle64_init->l[i].value_hi)) {
			nvgpu_writel(g, gr_pipe_bundle_data_r(),
				sw_bundle64_init->l[i].value_lo);
			nvgpu_writel(g, gr_pipe_bundle_data_hi_r(),
				sw_bundle64_init->l[i].value_hi);

			last_bundle_data_lo = sw_bundle64_init->l[i].value_lo;
			last_bundle_data_hi = sw_bundle64_init->l[i].value_hi;
		}

		nvgpu_writel(g, gr_pipe_bundle_address_r(),
			sw_bundle64_init->l[i].addr);

		if (gr_pipe_bundle_address_value_v(sw_bundle64_init->l[i].addr)
				== GR_GO_IDLE_BUNDLE) {
			err = gr_gk20a_wait_idle(g,
				gk20a_get_gr_idle_timeout(g),
				GR_IDLE_CHECK_DEFAULT);
		} else if (nvgpu_platform_is_silicon(g)) {
			err = gr_gk20a_wait_fe_idle(g,
				gk20a_get_gr_idle_timeout(g),
				GR_IDLE_CHECK_DEFAULT);
		}
		if (err != 0) {
			break;
		}
	}

	return err;
}

int gr_tu104_alloc_global_ctx_buffers(struct gk20a *g)
{
	int err;
	struct gr_gk20a *gr = &g->gr;
	u32 rtv_circular_buffer_size;

	nvgpu_log_fn(g, " ");

	rtv_circular_buffer_size =
		(gr_scc_rm_rtv_cb_size_div_256b_default_f() +
			gr_scc_rm_rtv_cb_size_div_256b_db_adder_f()) *
		gr_scc_bundle_cb_size_div_256b_byte_granularity_v();
	nvgpu_log_info(g, "rtv_circular_buffer_size : %u",
		rtv_circular_buffer_size);

	err = gk20a_gr_alloc_ctx_buffer(g,
			&gr->global_ctx_buffer[RTV_CIRCULAR_BUFFER],
			rtv_circular_buffer_size);
	if (err != 0) {
		return err;
	}

	err = gr_gk20a_alloc_global_ctx_buffers(g);
	if (err != 0) {
		goto clean_up;
	}

	return 0;

clean_up:
	nvgpu_err(g, "fail");
	gk20a_gr_destroy_ctx_buffer(g,
		&gr->global_ctx_buffer[RTV_CIRCULAR_BUFFER]);

	return err;
}

int gr_tu104_map_global_ctx_buffers(struct gk20a *g, struct vm_gk20a *vm,
		struct nvgpu_gr_ctx *gr_ctx, bool vpr)
{
	int err;
	u64 *g_bfr_va;
	u64 *g_bfr_size;
	int *g_bfr_index;
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_mem *mem;
	u64 gpu_va;

	nvgpu_log_fn(g, " ");

	g_bfr_va = gr_ctx->global_ctx_buffer_va;
	g_bfr_size = gr_ctx->global_ctx_buffer_size;
	g_bfr_index = gr_ctx->global_ctx_buffer_index;

	/* RTV circular buffer */
	mem = &gr->global_ctx_buffer[RTV_CIRCULAR_BUFFER].mem;
	gpu_va = nvgpu_gmmu_map(vm, mem, mem->size, 0,
			gk20a_mem_flag_none, true, mem->aperture);
	if (gpu_va == 0ULL) {
		return -ENOMEM;
	}

	g_bfr_va[RTV_CIRCULAR_BUFFER_VA] = gpu_va;
	g_bfr_size[RTV_CIRCULAR_BUFFER_VA] = mem->size;
	g_bfr_index[RTV_CIRCULAR_BUFFER_VA] = RTV_CIRCULAR_BUFFER;

	err = gr_gk20a_map_global_ctx_buffers(g, vm, gr_ctx, vpr);
	if (err != 0) {
		goto clean_up;
	}

	return 0;

clean_up:
	nvgpu_err(g, "fail");
	nvgpu_gmmu_unmap(vm, mem, gpu_va);

	return err;
}

static void gr_tu104_commit_rtv_circular_buffer(struct gk20a *g,
	struct nvgpu_gr_ctx *gr_ctx,
	u64 addr, u32 size, u32 gfxpAddSize, bool patch)
{
	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_scc_rm_rtv_cb_base_r(),
		gr_scc_rm_rtv_cb_base_addr_39_8_f(addr), patch);
	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_scc_rm_rtv_cb_size_r(),
		gr_scc_rm_rtv_cb_size_div_256b_f(size), patch);
	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_gcc_rm_rtv_cb_base_r(),
		gr_gpcs_gcc_rm_rtv_cb_base_addr_39_8_f(addr), patch);
	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_scc_rm_gfxp_reserve_r(),
		gr_scc_rm_gfxp_reserve_rtv_cb_size_div_256b_f(gfxpAddSize),
		patch);
}

int gr_tu104_commit_global_ctx_buffers(struct gk20a *g,
			struct nvgpu_gr_ctx *gr_ctx, bool patch)
{
	int err;
	u64 addr;
	u32 size;
	u32 gfxpaddsize = 0;

	nvgpu_log_fn(g, " ");

	err = gr_gk20a_commit_global_ctx_buffers(g, gr_ctx, patch);
	if (err != 0) {
		return err;
	}

	if (patch) {
		int err;
		err = gr_gk20a_ctx_patch_write_begin(g, gr_ctx, false);
		if (err != 0) {
			return err;
		}
	}

	/* RTV circular buffer */
	addr = gr_ctx->global_ctx_buffer_va[RTV_CIRCULAR_BUFFER_VA] >>
		U64(gr_scc_rm_rtv_cb_base_addr_39_8_align_bits_f());

	size = (gr_scc_rm_rtv_cb_size_div_256b_default_f() +
			gr_scc_rm_rtv_cb_size_div_256b_db_adder_f());

	gr_tu104_commit_rtv_circular_buffer(g, gr_ctx, addr, size,
						gfxpaddsize, patch);

	if (patch) {
		gr_gk20a_ctx_patch_write_end(g, gr_ctx, false);
	}

	return 0;
}

int gr_tu104_alloc_gfxp_rtv_cb(struct gk20a *g,
		  struct nvgpu_gr_ctx *gr_ctx, struct vm_gk20a *vm)
{
	int err;
	u32 rtv_cb_size;

	nvgpu_log_fn(g, " ");

	rtv_cb_size =
		(gr_scc_rm_rtv_cb_size_div_256b_default_f() +
		gr_scc_rm_rtv_cb_size_div_256b_db_adder_f() +
		gr_scc_rm_rtv_cb_size_div_256b_gfxp_adder_f()) *
		gr_scc_rm_rtv_cb_size_div_256b_byte_granularity_v();

	err = gr_gp10b_alloc_buffer(vm,
				rtv_cb_size,
				&gr_ctx->gfxp_rtvcb_ctxsw_buffer);

	return err;
}

void gr_tu104_commit_gfxp_rtv_cb(struct gk20a *g,
		  struct nvgpu_gr_ctx *gr_ctx, bool patch)
{
	u64 addr;
	u32 rtv_cb_size;
	u32 gfxp_addr_size;

	nvgpu_log_fn(g, " ");

	rtv_cb_size =
		(gr_scc_rm_rtv_cb_size_div_256b_default_f() +
		gr_scc_rm_rtv_cb_size_div_256b_db_adder_f() +
		gr_scc_rm_rtv_cb_size_div_256b_gfxp_adder_f());
	gfxp_addr_size = gr_scc_rm_rtv_cb_size_div_256b_gfxp_adder_f();

	/* GFXP RTV circular buffer */
	addr = (u64)(u64_lo32(gr_ctx->gfxp_rtvcb_ctxsw_buffer.gpu_va) >>
	       gr_scc_rm_rtv_cb_base_addr_39_8_align_bits_f()) |
	       (u64)(u64_hi32(gr_ctx->gfxp_rtvcb_ctxsw_buffer.gpu_va) <<
		(32U - gr_scc_rm_rtv_cb_base_addr_39_8_align_bits_f()));


	gr_tu104_commit_rtv_circular_buffer(g, gr_ctx, addr,
						rtv_cb_size,
						gfxp_addr_size,
						patch);
}

void gr_tu104_bundle_cb_defaults(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	gr->bundle_cb_default_size =
		gr_scc_bundle_cb_size_div_256b__prod_v();
	gr->min_gpm_fifo_depth =
		gr_pd_ab_dist_cfg2_state_limit_min_gpm_fifo_depths_v();
	gr->bundle_cb_token_limit =
		gr_pd_ab_dist_cfg2_token_limit_init_v();
}

void gr_tu104_cb_size_default(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	if (gr->attrib_cb_default_size == 0U) {
		gr->attrib_cb_default_size =
			gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v();
	}
	gr->alpha_cb_default_size =
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_default_v();
	gr->attrib_cb_gfxp_default_size =
		gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v();
	gr->attrib_cb_gfxp_size =
		gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v();
}

void gr_tu104_free_gr_ctx(struct gk20a *g,
			  struct vm_gk20a *vm, struct nvgpu_gr_ctx *gr_ctx)
{
	nvgpu_log_fn(g, " ");

	if (gr_ctx != NULL) {
		nvgpu_dma_unmap_free(vm, &gr_ctx->gfxp_rtvcb_ctxsw_buffer);
	}

	gr_gk20a_free_gr_ctx(g, vm, gr_ctx);
}

void gr_tu104_enable_gpc_exceptions(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_mask;

	gk20a_writel(g, gr_gpcs_tpcs_tpccs_tpc_exception_en_r(),
			gr_gpcs_tpcs_tpccs_tpc_exception_en_sm_enabled_f());

	tpc_mask =
		gr_gpcs_gpccs_gpc_exception_en_tpc_f((1 << gr->max_tpc_per_gpc_count) - 1);

	gk20a_writel(g, gr_gpcs_gpccs_gpc_exception_en_r(),
		(tpc_mask | gr_gpcs_gpccs_gpc_exception_en_gcc_f(1) |
			    gr_gpcs_gpccs_gpc_exception_en_gpccs_f(1) |
			    gr_gpcs_gpccs_gpc_exception_en_gpcmmu_f(1)));
}

int gr_tu104_get_offset_in_gpccs_segment(struct gk20a *g,
					enum ctxsw_addr_type addr_type,
					u32 num_tpcs,
					u32 num_ppcs,
					u32 reg_list_ppc_count,
					u32 *__offset_in_segment)
{
	u32 offset_in_segment = 0;
	u32 num_pes_per_gpc = nvgpu_get_litter_value(g,
				GPU_LIT_NUM_PES_PER_GPC);

	if (addr_type == CTXSW_ADDR_TYPE_TPC) {
		/*
		 * reg = g->netlist_vars->ctxsw_regs.tpc.l;
		 * offset_in_segment = 0;
		 */
	} else if (addr_type == CTXSW_ADDR_TYPE_PPC) {
		/*
		 * The ucode stores TPC data before PPC data.
		 * Advance offset past TPC data to PPC data.
		 */
		offset_in_segment =
			((g->netlist_vars->ctxsw_regs.tpc.count *
				num_tpcs) << 2);
	} else if (addr_type == CTXSW_ADDR_TYPE_GPC) {
		/*
		 * The ucode stores TPC/PPC data before GPC data.
		 * Advance offset past TPC/PPC data to GPC data.
		 *
		 * Note 1 PES_PER_GPC case
		 */
		if (num_pes_per_gpc > 1U) {
			offset_in_segment =
				(((g->netlist_vars->ctxsw_regs.tpc.count *
					num_tpcs) << 2) +
				((reg_list_ppc_count * num_ppcs) << 2));
		} else {
			offset_in_segment =
				((g->netlist_vars->ctxsw_regs.tpc.count *
					num_tpcs) << 2);
		}
	} else if ((addr_type == CTXSW_ADDR_TYPE_EGPC) ||
			(addr_type == CTXSW_ADDR_TYPE_ETPC)) {
		if (num_pes_per_gpc > 1U) {
			offset_in_segment =
				((g->netlist_vars->ctxsw_regs.tpc.count *
					num_tpcs) << 2) +
				((reg_list_ppc_count * num_ppcs) << 2) +
				(g->netlist_vars->ctxsw_regs.gpc.count << 2);
		} else {
			offset_in_segment =
				((g->netlist_vars->ctxsw_regs.tpc.count *
					num_tpcs) << 2) +
				(g->netlist_vars->ctxsw_regs.gpc.count << 2);
		}

		/* aligned to next 256 byte */
		offset_in_segment = ALIGN(offset_in_segment, 256);

		nvgpu_log(g, gpu_dbg_info | gpu_dbg_gpu_dbg,
			"egpc etpc offset_in_segment 0x%#08x",
			offset_in_segment);
	} else {
		nvgpu_log_fn(g, "Unknown address type.");
		return -EINVAL;
	}

	*__offset_in_segment = offset_in_segment;
	return 0;
}

static void gr_tu104_set_sm_disp_ctrl(struct gk20a *g, u32 data)
{
	u32 reg_val;

	nvgpu_log_fn(g, " ");

	reg_val = nvgpu_readl(g, gr_gpcs_tpcs_sm_disp_ctrl_r());

	if ((data & NVC5C0_SET_SM_DISP_CTRL_COMPUTE_SHADER_QUAD_MASK)
		       == NVC5C0_SET_SM_DISP_CTRL_COMPUTE_SHADER_QUAD_DISABLE) {
		reg_val = set_field(reg_val,
			gr_gpcs_tpcs_sm_disp_ctrl_compute_shader_quad_m(),
			gr_gpcs_tpcs_sm_disp_ctrl_compute_shader_quad_disable_f()
			);
	} else if ((data & NVC5C0_SET_SM_DISP_CTRL_COMPUTE_SHADER_QUAD_MASK)
			== NVC5C0_SET_SM_DISP_CTRL_COMPUTE_SHADER_QUAD_ENABLE) {
		reg_val = set_field(reg_val,
			gr_gpcs_tpcs_sm_disp_ctrl_compute_shader_quad_m(),
			gr_gpcs_tpcs_sm_disp_ctrl_compute_shader_quad_enable_f()
			);
	}

	nvgpu_writel(g, gr_gpcs_tpcs_sm_disp_ctrl_r(), reg_val);
}

int gr_tu104_handle_sw_method(struct gk20a *g, u32 addr,
			      u32 class_num, u32 offset, u32 data)
{
	nvgpu_log_fn(g, " ");

	if (class_num == TURING_COMPUTE_A) {
		switch (offset << 2) {
		case NVC5C0_SET_SHADER_EXCEPTIONS:
			gv11b_gr_set_shader_exceptions(g, data);
			break;
		case NVC5C0_SET_SKEDCHECK:
			gr_gv11b_set_skedcheck(g, data);
			break;
		case NVC5C0_SET_SM_DISP_CTRL:
			gr_tu104_set_sm_disp_ctrl(g, data);
			break;
		case NVC5C0_SET_SHADER_CUT_COLLECTOR:
			gr_gv11b_set_shader_cut_collector(g, data);
			break;
		default:
			goto fail;
		}
	}

	if (class_num == TURING_A) {
		switch (offset << 2) {
		case NVC597_SET_SHADER_EXCEPTIONS:
			gv11b_gr_set_shader_exceptions(g, data);
			break;
		case NVC597_SET_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_circular_buffer_size(g, data);
			break;
		case NVC597_SET_ALPHA_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_alpha_circular_buffer_size(g, data);
			break;
		case NVC597_SET_GO_IDLE_TIMEOUT:
			gr_gv11b_set_go_idle_timeout(g, data);
			break;
		case NVC097_SET_COALESCE_BUFFER_SIZE:
			gr_gv11b_set_coalesce_buffer_size(g, data);
			break;
		case NVC597_SET_TEX_IN_DBG:
			gr_gv11b_set_tex_in_dbg(g, data);
			break;
		case NVC597_SET_SKEDCHECK:
			gr_gv11b_set_skedcheck(g, data);
			break;
		case NVC597_SET_BES_CROP_DEBUG3:
			g->ops.gr.set_bes_crop_debug3(g, data);
			break;
		case NVC597_SET_BES_CROP_DEBUG4:
			g->ops.gr.set_bes_crop_debug4(g, data);
			break;
		case NVC597_SET_SM_DISP_CTRL:
			gr_tu104_set_sm_disp_ctrl(g, data);
			break;
		case NVC597_SET_SHADER_CUT_COLLECTOR:
			gr_gv11b_set_shader_cut_collector(g, data);
			break;
		default:
			goto fail;
		}
	}
	return 0;

fail:
	return -EINVAL;
}

void gr_tu104_init_sm_dsm_reg_info(void)
{
	return;
}

void gr_tu104_get_sm_dsm_perf_ctrl_regs(struct gk20a *g,
				        u32 *num_sm_dsm_perf_ctrl_regs,
				        u32 **sm_dsm_perf_ctrl_regs,
				        u32 *ctrl_register_stride)
{
	*num_sm_dsm_perf_ctrl_regs = 0;
	*sm_dsm_perf_ctrl_regs = NULL;
	*ctrl_register_stride = 0;
}
>>>>>>> CHANGE (f0762e gpu: nvgpu: add speculative barrier)
