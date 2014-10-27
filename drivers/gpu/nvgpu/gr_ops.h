#ifndef _NVGPU_GR_OPS_H_
#define _NVGPU_GR_OPS_H_

#include <linux/types.h>
#include <linux/printk.h>

/* TBD: rename these.  s/gk20a/nvgpu/g s/gpu/nvgpu/g*/
struct gk20a;
struct channel_ctx_gk20a;
struct channel_gk20a;
struct gr_gk20a;
struct gk20a_ctxsw_ucode_segments;
struct nvgpu_alloc_obj_ctx_args;
struct nvgpu_free_obj_ctx_args;
struct gr_zcull_info;

typedef int (*gr_init_fs_state_fn)(struct gk20a *g);
typedef void (*gr_access_smpc_reg_fn)(struct gk20a *g, u32 quad, u32 offset);
typedef void (*gr_bundle_cb_defaults_fn)(struct gk20a *g);
typedef void (*gr_cb_size_default_fn)(struct gk20a *g);
typedef int (*gr_calc_global_ctx_buffer_size_fn)(struct gk20a *g);
typedef void (*gr_commit_global_attrib_cb_fn)(struct gk20a *g,
					struct channel_ctx_gk20a *ch_ctx,
					u64 addr, bool patch);
typedef void (*gr_commit_global_bundle_cb_fn)(struct gk20a *g,
					struct channel_ctx_gk20a *ch_ctx,
					u64 addr, u64 size, bool patch);
typedef int (*gr_commit_global_cb_manager_fn)(struct gk20a *g,
					struct channel_gk20a *ch,
					bool patch);
typedef void (*gr_commit_global_pagepool_fn)(struct gk20a *g,
				       struct channel_ctx_gk20a *ch_ctx,
				       u64 addr, u32 size, bool patch);
typedef void (*gr_init_gpc_mmu_fn)(struct gk20a *g);
typedef int (*gr_handle_sw_method_fn)(struct gk20a *g, u32 addr,
				u32 class_num, u32 offset, u32 data);
typedef void (*gr_set_alpha_circular_buffer_size_fn)(struct gk20a *g,
					       u32 data);
typedef void (*gr_set_circular_buffer_size_fn)(struct gk20a *g, u32 data);
typedef void (*gr_enable_hww_exceptions_fn)(struct gk20a *g);
typedef bool (*gr_is_valid_class_fn)(struct gk20a *g, u32 class_num);
typedef void (*gr_get_sm_dsm_perf_regs_fn)(struct gk20a *g,
				     u32 *num_sm_dsm_perf_regs,
				     u32 **sm_dsm_perf_regs,
				     u32 *perf_register_stride);
typedef void (*gr_get_sm_dsm_perf_ctrl_regs_fn)(struct gk20a *g,
					  u32 *num_sm_dsm_perf_regs,
					  u32 **sm_dsm_perf_regs,
					  u32 *perf_register_stride);
typedef void (*gr_set_hww_esr_report_mask_fn)(struct gk20a *g);
typedef int (*gr_setup_alpha_beta_tables_fn)(struct gk20a *g,
				       struct gr_gk20a *gr);
typedef int (*gr_falcon_load_ucode_fn)(struct gk20a *g,
				 u64 addr_base,
				 struct gk20a_ctxsw_ucode_segments *segments,
				 u32 reg_offset);
typedef int (*gr_load_ctxsw_ucode_fn)(struct gk20a *g);
typedef u32 (*gr_get_gpc_tpc_mask_fn)(struct gk20a *g, u32 gpc_index);
typedef void (*gr_free_channel_ctx_fn)(struct channel_gk20a *c);
typedef int (*gr_alloc_obj_ctx_fn)(struct channel_gk20a  *c,
			     struct nvgpu_alloc_obj_ctx_args *args);
typedef int (*gr_free_obj_ctx_fn)(struct channel_gk20a  *c,
			    struct nvgpu_free_obj_ctx_args *args);
typedef int (*gr_bind_ctxsw_zcull_fn)(struct gk20a *g, struct gr_gk20a *gr,
				struct channel_gk20a *c, u64 zcull_va,
				u32 mode);
typedef int (*gr_get_zcull_info_fn)(struct gk20a *g, struct gr_gk20a *gr,
			      struct gr_zcull_info *zcull_params);

#define __op_decl(X) gr_##X##_fn X

struct gpu_gr_ops {
	__op_decl(init_fs_state);
	__op_decl(access_smpc_reg);
	__op_decl(bundle_cb_defaults);
	__op_decl(cb_size_default);
	__op_decl(calc_global_ctx_buffer_size);
	__op_decl(commit_global_attrib_cb);
	__op_decl(commit_global_bundle_cb);
	__op_decl(commit_global_cb_manager);
	__op_decl(commit_global_pagepool);
	__op_decl(init_gpc_mmu);
	__op_decl(handle_sw_method);
	__op_decl(set_alpha_circular_buffer_size);
	__op_decl(set_circular_buffer_size);
	__op_decl(enable_hww_exceptions);
	__op_decl(is_valid_class);
	__op_decl(get_sm_dsm_perf_regs);
	__op_decl(get_sm_dsm_perf_ctrl_regs);
	__op_decl(set_hww_esr_report_mask);
	__op_decl(setup_alpha_beta_tables);
	__op_decl(falcon_load_ucode);
	__op_decl(load_ctxsw_ucode);
	__op_decl(get_gpc_tpc_mask);
	__op_decl(free_channel_ctx);
	__op_decl(alloc_obj_ctx);
	__op_decl(free_obj_ctx);
	__op_decl(bind_ctxsw_zcull);
	__op_decl(get_zcull_info);
};
#undef __op_decl

#endif
