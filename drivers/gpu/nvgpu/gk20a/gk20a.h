/*
 * GK20A Graphics
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef GK20A_H
#define GK20A_H

struct gk20a;
struct fifo_gk20a;
struct channel_gk20a;
struct gr_gk20a;
struct sim_gk20a;
struct gk20a_ctxsw_ucode_segments;
struct gk20a_fecs_trace;
struct gk20a_ctxsw_trace;
struct acr_desc;
struct nvgpu_mem_alloc_tracker;
struct dbg_profiler_object_data;

#include <linux/sched.h>
#include <nvgpu/lock.h>
#include <linux/nvgpu.h>
#include <linux/irqreturn.h>
#include <soc/tegra/chip-id.h>
#include <linux/version.h>
#include <linux/atomic.h>
#include <linux/cdev.h>

#include "../../../arch/arm/mach-tegra/iomap.h"

#include <nvgpu/as.h>
#include <nvgpu/log.h>
#include <nvgpu/pramin.h>
#include <nvgpu/acr/nvgpu_acr.h>

#include "clk_gk20a.h"
#include "ce2_gk20a.h"
#include "fifo_gk20a.h"
#include "tsg_gk20a.h"
#include "gr_gk20a.h"
#include "sim_gk20a.h"
#include "pmu_gk20a.h"
#include "priv_ring_gk20a.h"
#include "therm_gk20a.h"
#include "platform_gk20a.h"
#include "gm20b/acr_gm20b.h"
#include "cde_gk20a.h"
#include "debug_gk20a.h"
#include "sched_gk20a.h"
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "clk/clk.h"
#include "clk/clk_arb.h"
#include "perf/perf.h"
#include "pmgr/pmgr.h"
#include "therm/thrm.h"
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
#define WRITE_ONCE(x, val) \
			x = val
#endif

struct page_alloc_chunk;

/* PTIMER_REF_FREQ_HZ corresponds to a period of 32 nanoseconds.
    32 ns is the resolution of ptimer. */
#define PTIMER_REF_FREQ_HZ                      31250000

struct cooling_device_gk20a {
	struct thermal_cooling_device *gk20a_cooling_dev;
	unsigned int gk20a_freq_state;
	unsigned int gk20a_freq_table_size;
	struct gk20a *g;
};

#ifdef CONFIG_DEBUG_FS
struct railgate_stats {
	unsigned long last_rail_gate_start;
	unsigned long last_rail_gate_complete;
	unsigned long last_rail_ungate_start;
	unsigned long last_rail_ungate_complete;
	unsigned long total_rail_gate_time_ms;
	unsigned long total_rail_ungate_time_ms;
	unsigned long railgating_cycle_count;
};
#endif

enum gk20a_cbc_op {
	gk20a_cbc_op_clear,
	gk20a_cbc_op_clean,
	gk20a_cbc_op_invalidate,
};

#define MC_INTR_UNIT_DISABLE	false
#define MC_INTR_UNIT_ENABLE		true

#define GPU_LIT_NUM_GPCS	0
#define GPU_LIT_NUM_PES_PER_GPC 1
#define GPU_LIT_NUM_ZCULL_BANKS 2
#define GPU_LIT_NUM_TPC_PER_GPC 3
#define GPU_LIT_NUM_SM_PER_TPC  4
#define GPU_LIT_NUM_FBPS	5
#define GPU_LIT_GPC_BASE	6
#define GPU_LIT_GPC_STRIDE	7
#define GPU_LIT_GPC_SHARED_BASE 8
#define GPU_LIT_TPC_IN_GPC_BASE 9
#define GPU_LIT_TPC_IN_GPC_STRIDE 10
#define GPU_LIT_TPC_IN_GPC_SHARED_BASE 11
#define GPU_LIT_PPC_IN_GPC_BASE	12
#define GPU_LIT_PPC_IN_GPC_STRIDE 13
#define GPU_LIT_PPC_IN_GPC_SHARED_BASE 14
#define GPU_LIT_ROP_BASE	15
#define GPU_LIT_ROP_STRIDE	16
#define GPU_LIT_ROP_SHARED_BASE 17
#define GPU_LIT_HOST_NUM_ENGINES 18
#define GPU_LIT_HOST_NUM_PBDMA	19
#define GPU_LIT_LTC_STRIDE	20
#define GPU_LIT_LTS_STRIDE	21
#define GPU_LIT_NUM_FBPAS	22
#define GPU_LIT_FBPA_STRIDE	23
#define GPU_LIT_FBPA_BASE	24
#define GPU_LIT_FBPA_SHARED_BASE 25

#define nvgpu_get_litter_value(g, v) (g)->ops.get_litter_value((g), v)

struct gpu_ops {
	struct {
		int (*determine_L2_size_bytes)(struct gk20a *gk20a);
		int (*init_comptags)(struct gk20a *g, struct gr_gk20a *gr);
		int (*cbc_ctrl)(struct gk20a *g, enum gk20a_cbc_op op,
				u32 min, u32 max);
		void (*set_zbc_color_entry)(struct gk20a *g,
					    struct zbc_entry *color_val,
					    u32 index);
		void (*set_zbc_depth_entry)(struct gk20a *g,
					    struct zbc_entry *depth_val,
					    u32 index);
		void (*set_zbc_s_entry)(struct gk20a *g,
					    struct zbc_entry *s_val,
					    u32 index);
		void (*init_cbc)(struct gk20a *g, struct gr_gk20a *gr);
		void (*sync_debugfs)(struct gk20a *g);
		void (*init_fs_state)(struct gk20a *g);
		void (*isr)(struct gk20a *g);
		u32 (*cbc_fix_config)(struct gk20a *g, int base);
		void (*flush)(struct gk20a *g);
	} ltc;
	struct {
		void (*isr_stall)(struct gk20a *g, u32 inst_id, u32 pri_base);
		int (*isr_nonstall)(struct gk20a *g, u32 inst_id, u32 pri_base);
	} ce2;
	struct {
		int (*init_fs_state)(struct gk20a *g);
		int (*init_preemption_state)(struct gk20a *g);
		void (*access_smpc_reg)(struct gk20a *g, u32 quad, u32 offset);
		void (*bundle_cb_defaults)(struct gk20a *g);
		void (*cb_size_default)(struct gk20a *g);
		int (*calc_global_ctx_buffer_size)(struct gk20a *g);
		void (*commit_global_attrib_cb)(struct gk20a *g,
						struct channel_ctx_gk20a *ch_ctx,
						u64 addr, bool patch);
		void (*commit_global_bundle_cb)(struct gk20a *g,
						struct channel_ctx_gk20a *ch_ctx,
						u64 addr, u64 size, bool patch);
		int (*commit_global_cb_manager)(struct gk20a *g,
						struct channel_gk20a *ch,
						bool patch);
		void (*commit_global_pagepool)(struct gk20a *g,
					       struct channel_ctx_gk20a *ch_ctx,
					       u64 addr, u32 size, bool patch);
		void (*init_gpc_mmu)(struct gk20a *g);
		int (*handle_sw_method)(struct gk20a *g, u32 addr,
					 u32 class_num, u32 offset, u32 data);
		void (*set_alpha_circular_buffer_size)(struct gk20a *g,
					               u32 data);
		void (*set_circular_buffer_size)(struct gk20a *g, u32 data);
		void (*enable_hww_exceptions)(struct gk20a *g);
		bool (*is_valid_class)(struct gk20a *g, u32 class_num);
		void (*get_sm_dsm_perf_regs)(struct gk20a *g,
						  u32 *num_sm_dsm_perf_regs,
						  u32 **sm_dsm_perf_regs,
						  u32 *perf_register_stride);
		void (*get_sm_dsm_perf_ctrl_regs)(struct gk20a *g,
						  u32 *num_sm_dsm_perf_regs,
						  u32 **sm_dsm_perf_regs,
						  u32 *perf_register_stride);
		void (*set_hww_esr_report_mask)(struct gk20a *g);
		int (*setup_alpha_beta_tables)(struct gk20a *g,
					      struct gr_gk20a *gr);
		int (*falcon_load_ucode)(struct gk20a *g,
				u64 addr_base,
				struct gk20a_ctxsw_ucode_segments *segments,
				u32 reg_offset);
		int (*load_ctxsw_ucode)(struct gk20a *g);
		u32 (*get_gpc_tpc_mask)(struct gk20a *g, u32 gpc_index);
		void (*set_gpc_tpc_mask)(struct gk20a *g, u32 gpc_index);
		void (*free_channel_ctx)(struct channel_gk20a *c);
		int (*alloc_obj_ctx)(struct channel_gk20a  *c,
				struct nvgpu_alloc_obj_ctx_args *args);
		int (*bind_ctxsw_zcull)(struct gk20a *g, struct gr_gk20a *gr,
				struct channel_gk20a *c, u64 zcull_va,
				u32 mode);
		int (*get_zcull_info)(struct gk20a *g, struct gr_gk20a *gr,
				struct gr_zcull_info *zcull_params);
		bool (*is_tpc_addr)(struct gk20a *g, u32 addr);
		u32 (*get_tpc_num)(struct gk20a *g, u32 addr);
		bool (*is_ltcs_ltss_addr)(struct gk20a *g, u32 addr);
		bool (*is_ltcn_ltss_addr)(struct gk20a *g, u32 addr);
		bool (*get_lts_in_ltc_shared_base)(void);
		void (*split_lts_broadcast_addr)(struct gk20a *g, u32 addr,
					u32 *priv_addr_table,
					u32 *priv_addr_table_index);
		void (*split_ltc_broadcast_addr)(struct gk20a *g, u32 addr,
					u32 *priv_addr_table,
					u32 *priv_addr_table_index);
		void (*detect_sm_arch)(struct gk20a *g);
		int (*add_zbc_color)(struct gk20a *g, struct gr_gk20a *gr,
				  struct zbc_entry *color_val, u32 index);
		int (*add_zbc_depth)(struct gk20a *g, struct gr_gk20a *gr,
				  struct zbc_entry *depth_val, u32 index);
		int (*add_zbc_s)(struct gk20a *g, struct gr_gk20a *gr,
				  struct zbc_entry *s_val, u32 index);
		int (*zbc_set_table)(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_entry *zbc_val);
		int (*zbc_query_table)(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_query_params *query_params);
		int (*zbc_s_query_table)(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_query_params *query_params);
		int (*load_zbc_s_default_tbl)(struct gk20a *g,
					 struct gr_gk20a *gr);
		int (*load_zbc_s_tbl)(struct gk20a *g,
					 struct gr_gk20a *gr);
		void (*pmu_save_zbc)(struct gk20a *g, u32 entries);
		int (*add_zbc)(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_entry *zbc_val);
		bool (*add_zbc_type_s)(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_entry *zbc_val, int *ret_val);
		u32 (*pagepool_default_size)(struct gk20a *g);
		int (*init_ctx_state)(struct gk20a *g);
		int (*alloc_gr_ctx)(struct gk20a *g,
			  struct gr_ctx_desc **__gr_ctx, struct vm_gk20a *vm,
			  u32 class, u32 padding);
		void (*free_gr_ctx)(struct gk20a *g,
			  struct vm_gk20a *vm,
			  struct gr_ctx_desc *gr_ctx);
		void (*update_ctxsw_preemption_mode)(struct gk20a *g,
				struct channel_ctx_gk20a *ch_ctx,
				struct mem_desc *mem);
		int (*update_smpc_ctxsw_mode)(struct gk20a *g,
				struct channel_gk20a *c,
				bool enable);
		int (*update_hwpm_ctxsw_mode)(struct gk20a *g,
				struct channel_gk20a *c,
				bool enable);
		int (*dump_gr_regs)(struct gk20a *g,
				struct gk20a_debug_output *o);
		int (*update_pc_sampling)(struct channel_gk20a *ch,
					   bool enable);
		u32 (*get_max_fbps_count)(struct gk20a *g);
		u32 (*get_fbp_en_mask)(struct gk20a *g);
		u32 (*get_max_ltc_per_fbp)(struct gk20a *g);
		u32 (*get_max_lts_per_ltc)(struct gk20a *g);
		u32* (*get_rop_l2_en_mask)(struct gk20a *g);
		void (*init_sm_dsm_reg_info)(void);
		int (*wait_empty)(struct gk20a *g, unsigned long duration_ms,
				  u32 expect_delay);
		void (*init_cyclestats)(struct gk20a *g);
		void (*enable_cde_in_fecs)(struct gk20a *g,
				struct mem_desc *mem);
		int (*set_sm_debug_mode)(struct gk20a *g, struct channel_gk20a *ch,
					u64 sms, bool enable);
		void (*bpt_reg_info)(struct gk20a *g,
				struct warpstate *w_state);
		void (*get_access_map)(struct gk20a *g,
				      u32 **whitelist, int *num_entries);
		int (*handle_fecs_error)(struct gk20a *g,
				struct channel_gk20a *ch,
				struct gr_gk20a_isr_data *isr_data);
		int (*pre_process_sm_exception)(struct gk20a *g,
				u32 gpc, u32 tpc, u32 global_esr, u32 warp_esr,
				bool sm_debugger_attached,
				struct channel_gk20a *fault_ch,
				bool *early_exit, bool *ignore_debugger);
		u32 (*mask_hww_warp_esr)(u32 hww_warp_esr);
		int (*handle_sm_exception)(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr);
		int (*handle_tex_exception)(struct gk20a *g, u32 gpc, u32 tpc,
						bool *post_event);
		void (*enable_gpc_exceptions)(struct gk20a *g);
		void (*create_gr_sysfs)(struct device *dev);
		u32 (*get_lrf_tex_ltc_dram_override)(struct gk20a *g);
		int (*record_sm_error_state)(struct gk20a *g,
				u32 gpc, u32 tpc);
		int (*update_sm_error_state)(struct gk20a *g,
				struct channel_gk20a *ch, u32 sm_id,
				struct nvgpu_dbg_gpu_sm_error_state_record *
								sm_error_state);
		int (*clear_sm_error_state)(struct gk20a *g,
				struct channel_gk20a *ch, u32 sm_id);
		int (*suspend_contexts)(struct gk20a *g,
				struct dbg_session_gk20a *dbg_s,
				int *ctx_resident_ch_fd);
		int (*resume_contexts)(struct gk20a *g,
				struct dbg_session_gk20a *dbg_s,
				int *ctx_resident_ch_fd);
		int (*set_preemption_mode)(struct channel_gk20a *ch,
				u32 graphics_preempt_mode,
				u32 compute_preempt_mode);
		int (*get_preemption_mode_flags)(struct gk20a *g,
		       struct nvgpu_preemption_modes_rec *preemption_modes_rec);
		int (*set_ctxsw_preemption_mode)(struct gk20a *g,
				struct gr_ctx_desc *gr_ctx,
				struct vm_gk20a *vm, u32 class,
				u32 graphics_preempt_mode,
				u32 compute_preempt_mode);
		int (*set_boosted_ctx)(struct channel_gk20a *ch, bool boost);
		void (*update_boosted_ctx)(struct gk20a *g,
					   struct mem_desc *mem,
					   struct gr_ctx_desc *gr_ctx);
		int (*fuse_override)(struct gk20a *g);
		void (*init_sm_id_table)(struct gk20a *g);
		int (*load_smid_config)(struct gk20a *g);
		void (*program_sm_id_numbering)(struct gk20a *g,
						u32 gpc, u32 tpc, u32 smid);
		void (*program_active_tpc_counts)(struct gk20a *g, u32 gpc);
		int  (*setup_rop_mapping)(struct gk20a *g, struct gr_gk20a *gr);
		int (*init_sw_veid_bundle)(struct gk20a *g);
		void (*program_zcull_mapping)(struct gk20a *g,
				u32 zcull_alloc_num, u32 *zcull_map_tiles);
		int (*commit_global_timeslice)(struct gk20a *g,
					struct channel_gk20a *c, bool patch);
		int (*commit_inst)(struct channel_gk20a *c, u64 gpu_va);
		void (*restore_context_header)(struct gk20a *g, struct mem_desc *ctxheader);
		void (*write_zcull_ptr)(struct gk20a *g,
					struct mem_desc *mem, u64 gpu_va);
		void (*write_pm_ptr)(struct gk20a *g,
					struct mem_desc *mem, u64 gpu_va);
		void (*init_elcg_mode)(struct gk20a *g, u32 mode, u32 engine);
		void (*load_tpc_mask)(struct gk20a *g);
		int (*inval_icache)(struct gk20a *g, struct channel_gk20a *ch);
		int (*trigger_suspend)(struct gk20a *g);
		int (*wait_for_pause)(struct gk20a *g, struct warpstate *w_state);
		int (*resume_from_pause)(struct gk20a *g);
		int (*clear_sm_errors)(struct gk20a *g);
		u32 (*tpc_enabled_exceptions)(struct gk20a *g);
	} gr;
	struct {
		void (*init_hw)(struct gk20a *g);
		void (*init_fs_state)(struct gk20a *g);
		void (*reset)(struct gk20a *g);
		void (*init_uncompressed_kind_map)(struct gk20a *g);
		void (*init_kind_attr)(struct gk20a *g);
		void (*set_mmu_page_size)(struct gk20a *g);
		bool (*set_use_full_comp_tag_line)(struct gk20a *g);
		unsigned int (*compression_page_size)(struct gk20a *g);
		unsigned int (*compressible_page_size)(struct gk20a *g);
		void (*dump_vpr_wpr_info)(struct gk20a *g);
		int (*vpr_info_fetch)(struct gk20a *g);
		bool (*is_debug_mode_enabled)(struct gk20a *g);
		void (*set_debug_mode)(struct gk20a *g, bool enable);
		void (*tlb_invalidate)(struct gk20a *g, struct mem_desc *pdb);
		void (*hub_isr)(struct gk20a *g);
	} fb;
	struct {
		void (*slcg_bus_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_ce2_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_chiplet_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_ctxsw_firmware_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_fb_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_fifo_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_gr_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_ltc_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_perf_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_priring_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_pmu_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_therm_load_gating_prod)(struct gk20a *g, bool prod);
		void (*slcg_xbar_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_bus_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_ce_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_ctxsw_firmware_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_fb_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_fifo_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_gr_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_ltc_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_pwr_csb_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_pmu_load_gating_prod)(struct gk20a *g, bool prod);
		void (*blcg_xbar_load_gating_prod)(struct gk20a *g, bool prod);
		void (*pg_gr_load_gating_prod)(struct gk20a *g, bool prod);
	} clock_gating;
	struct {
		int (*init_fifo_setup_hw)(struct gk20a *g);
		void (*bind_channel)(struct channel_gk20a *ch_gk20a);
		void (*unbind_channel)(struct channel_gk20a *ch_gk20a);
		void (*disable_channel)(struct channel_gk20a *ch);
		void (*enable_channel)(struct channel_gk20a *ch);
		int (*alloc_inst)(struct gk20a *g, struct channel_gk20a *ch);
		void (*free_inst)(struct gk20a *g, struct channel_gk20a *ch);
		int (*setup_ramfc)(struct channel_gk20a *c, u64 gpfifo_base,
				u32 gpfifo_entries,
				unsigned long acquire_timeout,
				u32 flags);
		int (*resetup_ramfc)(struct channel_gk20a *c);
		int (*preempt_channel)(struct gk20a *g, u32 hw_chid);
		int (*preempt_tsg)(struct gk20a *g, u32 tsgid);
		int (*update_runlist)(struct gk20a *g, u32 runlist_id,
				u32 hw_chid, bool add,
				bool wait_for_finish);
		void (*trigger_mmu_fault)(struct gk20a *g,
				unsigned long engine_ids);
		void (*apply_pb_timeout)(struct gk20a *g);
		int (*wait_engine_idle)(struct gk20a *g);
		u32 (*get_num_fifos)(struct gk20a *g);
		u32 (*get_pbdma_signature)(struct gk20a *g);
		int (*channel_set_priority)(struct channel_gk20a *ch, u32 priority);
		int (*set_runlist_interleave)(struct gk20a *g, u32 id,
					bool is_tsg, u32 runlist_id,
					u32 new_level);
		int (*channel_set_timeslice)(struct channel_gk20a *ch,
					u32 timeslice);
		int (*tsg_set_timeslice)(struct tsg_gk20a *tsg, u32 timeslice);
		int (*force_reset_ch)(struct channel_gk20a *ch,
					u32 err_code, bool verbose);
		int (*engine_enum_from_type)(struct gk20a *g, u32 engine_type,
					u32 *inst_id);
		void (*device_info_data_parse)(struct gk20a *g,
					u32 table_entry, u32 *inst_id,
					u32 *pri_base, u32 *fault_id);
		u32 (*device_info_fault_id)(u32 table_entry);
		int (*tsg_bind_channel)(struct tsg_gk20a *tsg,
				struct channel_gk20a *ch);
		int (*tsg_unbind_channel)(struct channel_gk20a *ch);
		int (*tsg_open)(struct tsg_gk20a *tsg);
		u32 (*eng_runlist_base_size)(void);
		int (*init_engine_info)(struct fifo_gk20a *f);
		u32 (*runlist_entry_size)(void);
		void (*get_tsg_runlist_entry)(struct tsg_gk20a *tsg,
					u32 *runlist);
		void (*get_ch_runlist_entry)(struct channel_gk20a *ch,
					u32 *runlist);
		u32 (*userd_gp_get)(struct gk20a *g, struct channel_gk20a *ch);
		void (*userd_gp_put)(struct gk20a *g, struct channel_gk20a *ch);
		void (*free_channel_ctx_header)(struct channel_gk20a *ch);
		bool (*is_fault_engine_subid_gpc)(struct gk20a *g,
					 u32 engine_subid);
		void (*dump_pbdma_status)(struct gk20a *g,
				struct gk20a_debug_output *o);
		void (*dump_eng_status)(struct gk20a *g,
				struct gk20a_debug_output *o);
		void (*dump_channel_status_ramfc)(struct gk20a *g,
				struct gk20a_debug_output *o, u32 hw_chid,
				struct ch_state *ch_state);
		u32 (*intr_0_error_mask)(struct gk20a *g);
		int (*is_preempt_pending)(struct gk20a *g, u32 id,
			unsigned int id_type, unsigned int timeout_rc_type);
		int (*preempt_ch_tsg)(struct gk20a *g, u32 id,
			unsigned int id_type, unsigned int timeout_rc_type);
		void (*init_pbdma_intr_descs)(struct fifo_gk20a *f);
		int (*reset_enable_hw)(struct gk20a *g);
		int (*setup_userd)(struct channel_gk20a *c);
		u32 (*pbdma_acquire_val)(u64 timeout);
		void (*teardown_ch_tsg)(struct gk20a *g, u32 act_eng_bitmask,
			u32 id, unsigned int id_type, unsigned int rc_type,
			 struct mmu_fault_info *mmfault);
	} fifo;
	struct pmu_v {
		/*used for change of enum zbc update cmd id from ver 0 to ver1*/
		u32 cmd_id_zbc_table_update;
		u32 (*get_pmu_cmdline_args_size)(struct pmu_gk20a *pmu);
		void (*set_pmu_cmdline_args_cpu_freq)(struct pmu_gk20a *pmu,
			u32 freq);
		void (*set_pmu_cmdline_args_trace_size)(struct pmu_gk20a *pmu,
			u32 size);
		void (*set_pmu_cmdline_args_trace_dma_base)(
				struct pmu_gk20a *pmu);
		void (*set_pmu_cmdline_args_trace_dma_idx)(
			struct pmu_gk20a *pmu, u32 idx);
		void * (*get_pmu_cmdline_args_ptr)(struct pmu_gk20a *pmu);
		u32 (*get_pmu_allocation_struct_size)(struct pmu_gk20a *pmu);
		void (*set_pmu_allocation_ptr)(struct pmu_gk20a *pmu,
				void **pmu_alloc_ptr, void *assign_ptr);
		void (*pmu_allocation_set_dmem_size)(struct pmu_gk20a *pmu,
				void *pmu_alloc_ptr, u16 size);
		u16 (*pmu_allocation_get_dmem_size)(struct pmu_gk20a *pmu,
				void *pmu_alloc_ptr);
		u32 (*pmu_allocation_get_dmem_offset)(struct pmu_gk20a *pmu,
				void *pmu_alloc_ptr);
		u32 * (*pmu_allocation_get_dmem_offset_addr)(
				struct pmu_gk20a *pmu, void *pmu_alloc_ptr);
		void (*pmu_allocation_set_dmem_offset)(struct pmu_gk20a *pmu,
				void *pmu_alloc_ptr, u32 offset);
		void * (*pmu_allocation_get_fb_addr)(
				struct pmu_gk20a *pmu, void *pmu_alloc_ptr);
		u32 (*pmu_allocation_get_fb_size)(
				struct pmu_gk20a *pmu, void *pmu_alloc_ptr);
		void (*get_pmu_init_msg_pmu_queue_params)(
				struct pmu_queue *queue, u32 id,
				void *pmu_init_msg);
		void *(*get_pmu_msg_pmu_init_msg_ptr)(
				struct pmu_init_msg *init);
		u16 (*get_pmu_init_msg_pmu_sw_mg_off)(
			union pmu_init_msg_pmu *init_msg);
		u16 (*get_pmu_init_msg_pmu_sw_mg_size)(
			union pmu_init_msg_pmu *init_msg);
		u32 (*get_pmu_perfmon_cmd_start_size)(void);
		int (*get_perfmon_cmd_start_offsetofvar)(
				enum pmu_perfmon_cmd_start_fields field);
		void (*perfmon_start_set_cmd_type)(struct pmu_perfmon_cmd *pc,
				u8 value);
		void (*perfmon_start_set_group_id)(struct pmu_perfmon_cmd *pc,
				u8 value);
		void (*perfmon_start_set_state_id)(struct pmu_perfmon_cmd *pc,
				u8 value);
		void (*perfmon_start_set_flags)(struct pmu_perfmon_cmd *pc,
				u8 value);
		u8 (*perfmon_start_get_flags)(struct pmu_perfmon_cmd *pc);
		u32 (*get_pmu_perfmon_cmd_init_size)(void);
		int (*get_perfmon_cmd_init_offsetofvar)(
				enum pmu_perfmon_cmd_start_fields field);
		void (*perfmon_cmd_init_set_sample_buffer)(
				struct pmu_perfmon_cmd *pc, u16 value);
		void (*perfmon_cmd_init_set_dec_cnt)(
				struct pmu_perfmon_cmd *pc, u8 value);
		void (*perfmon_cmd_init_set_base_cnt_id)(
				struct pmu_perfmon_cmd *pc, u8 value);
		void (*perfmon_cmd_init_set_samp_period_us)(
				struct pmu_perfmon_cmd *pc, u32 value);
		void (*perfmon_cmd_init_set_num_cnt)(struct pmu_perfmon_cmd *pc,
				u8 value);
		void (*perfmon_cmd_init_set_mov_avg)(struct pmu_perfmon_cmd *pc,
				u8 value);
		void *(*get_pmu_seq_in_a_ptr)(
				struct pmu_sequence *seq);
		void *(*get_pmu_seq_out_a_ptr)(
				struct pmu_sequence *seq);
		void (*set_pmu_cmdline_args_secure_mode)(struct pmu_gk20a *pmu,
			u32 val);
		u32 (*get_perfmon_cntr_sz)(struct pmu_gk20a *pmu);
		void * (*get_perfmon_cntr_ptr)(struct pmu_gk20a *pmu);
		void (*set_perfmon_cntr_ut)(struct pmu_gk20a *pmu, u16 ut);
		void (*set_perfmon_cntr_lt)(struct pmu_gk20a *pmu, u16 lt);
		void (*set_perfmon_cntr_valid)(struct pmu_gk20a *pmu, u8 val);
		void (*set_perfmon_cntr_index)(struct pmu_gk20a *pmu, u8 val);
		void (*set_perfmon_cntr_group_id)(struct pmu_gk20a *pmu,
				u8 gid);

		u8 (*pg_cmd_eng_buf_load_size)(struct pmu_pg_cmd *pg);
		void (*pg_cmd_eng_buf_load_set_cmd_type)(struct pmu_pg_cmd *pg,
				u8 value);
		void (*pg_cmd_eng_buf_load_set_engine_id)(struct pmu_pg_cmd *pg,
				u8 value);
		void (*pg_cmd_eng_buf_load_set_buf_idx)(struct pmu_pg_cmd *pg,
				u8 value);
		void (*pg_cmd_eng_buf_load_set_pad)(struct pmu_pg_cmd *pg,
				u8 value);
		void (*pg_cmd_eng_buf_load_set_buf_size)(struct pmu_pg_cmd *pg,
				u16 value);
		void (*pg_cmd_eng_buf_load_set_dma_base)(struct pmu_pg_cmd *pg,
				u32 value);
		void (*pg_cmd_eng_buf_load_set_dma_offset)(struct pmu_pg_cmd *pg,
				u8 value);
		void (*pg_cmd_eng_buf_load_set_dma_idx)(struct pmu_pg_cmd *pg,
				u8 value);
	} pmu_ver;
	struct {
		int (*get_netlist_name)(struct gk20a *g, int index, char *name);
		bool (*is_fw_defined)(void);
		bool use_dma_for_fw_bootstrap;
	} gr_ctx;
	struct {
		int (*init)(struct gk20a *g);
		int (*max_entries)(struct gk20a *,
			struct nvgpu_ctxsw_trace_filter *);
		int (*flush)(struct gk20a *g);
		int (*poll)(struct gk20a *g);
		int (*enable)(struct gk20a *g);
		int (*disable)(struct gk20a *g);
		bool (*is_enabled)(struct gk20a *g);
		int (*reset)(struct gk20a *g);
		int (*bind_channel)(struct gk20a *, struct channel_gk20a *);
		int (*unbind_channel)(struct gk20a *, struct channel_gk20a *);
		int (*deinit)(struct gk20a *g);
		int (*alloc_user_buffer)(struct gk20a *g,
					void **buf, size_t *size);
		int (*free_user_buffer)(struct gk20a *g);
		int (*mmap_user_buffer)(struct gk20a *g,
					struct vm_area_struct *vma);
		int (*set_filter)(struct gk20a *g,
			struct nvgpu_ctxsw_trace_filter *filter);
	} fecs_trace;
	struct {
		bool (*support_sparse)(struct gk20a *g);
		u64 (*gmmu_map)(struct vm_gk20a *vm,
				u64 map_offset,
				struct sg_table *sgt,
				u64 buffer_offset,
				u64 size,
				int pgsz_idx,
				u8 kind_v,
				u32 ctag_offset,
				u32 flags,
				int rw_flag,
				bool clear_ctags,
				bool sparse,
				bool priv,
				struct vm_gk20a_mapping_batch *batch,
				enum nvgpu_aperture aperture);
		void (*gmmu_unmap)(struct vm_gk20a *vm,
				u64 vaddr,
				u64 size,
				int pgsz_idx,
				bool va_allocated,
				int rw_flag,
				bool sparse,
				struct vm_gk20a_mapping_batch *batch);
		void (*vm_remove)(struct vm_gk20a *vm);
		int (*vm_alloc_share)(struct gk20a_as_share *as_share,
				      u32 big_page_size, u32 flags);
		int (*vm_bind_channel)(struct gk20a_as_share *as_share,
				struct channel_gk20a *ch);
		int (*fb_flush)(struct gk20a *g);
		void (*l2_invalidate)(struct gk20a *g);
		void (*l2_flush)(struct gk20a *g, bool invalidate);
		void (*cbc_clean)(struct gk20a *g);
		void (*set_big_page_size)(struct gk20a *g,
					  struct mem_desc *mem, int size);
		u32 (*get_big_page_sizes)(void);
		u32 (*get_physical_addr_bits)(struct gk20a *g);
		int (*init_mm_setup_hw)(struct gk20a *g);
		bool (*is_bar1_supported)(struct gk20a *g);
		int (*init_bar2_vm)(struct gk20a *g);
		int (*init_bar2_mm_hw_setup)(struct gk20a *g);
		void (*remove_bar2_vm)(struct gk20a *g);
		const struct gk20a_mmu_level *
			(*get_mmu_levels)(struct gk20a *g, u32 big_page_size);
		void (*init_pdb)(struct gk20a *g, struct mem_desc *inst_block,
				struct vm_gk20a *vm);
		u64 (*get_iova_addr)(struct gk20a *g, struct scatterlist *sgl,
					 u32 flags);
		size_t (*get_vidmem_size)(struct gk20a *g);
		void (*init_inst_block)(struct mem_desc *inst_block,
				struct vm_gk20a *vm, u32 big_page_size);
		bool (*mmu_fault_pending)(struct gk20a *g);
	} mm;
	struct {
		u32 (*enter)(struct gk20a *g, struct mem_desc *mem,
			     struct page_alloc_chunk *chunk, u32 w);
		void (*exit)(struct gk20a *g, struct mem_desc *mem,
			     struct page_alloc_chunk *chunk);
		u32 (*data032_r)(u32 i);
	} pramin;
	struct {
		int (*init_therm_setup_hw)(struct gk20a *g);
		int (*elcg_init_idle_filters)(struct gk20a *g);
		void (*therm_debugfs_init)(struct gk20a *g);
		int (*get_internal_sensor_curr_temp)(struct gk20a *g, u32 *temp_f24_8);
		void (*get_internal_sensor_limits)(s32 *max_24_8,
							s32 *min_24_8);
		u32 (*configure_therm_alert)(struct gk20a *g, s32 curr_warn_temp);
	} therm;
	struct {
		bool (*is_pmu_supported)(struct gk20a *g);
		int (*prepare_ucode)(struct gk20a *g);
		int (*pmu_setup_hw_and_bootstrap)(struct gk20a *g);
		int (*pmu_nsbootstrap)(struct pmu_gk20a *pmu);
		int (*pmu_setup_elpg)(struct gk20a *g);
		u32 (*pmu_get_queue_head)(u32 i);
		u32 (*pmu_get_queue_head_size)(void);
		u32 (*pmu_get_queue_tail_size)(void);
		u32 (*pmu_get_queue_tail)(u32 i);
		int (*init_wpr_region)(struct gk20a *g);
		int (*load_lsfalcon_ucode)(struct gk20a *g, u32 falconidmask);
		void (*write_dmatrfbase)(struct gk20a *g, u32 addr);
		void (*pmu_elpg_statistics)(struct gk20a *g, u32 pg_engine_id,
			struct pmu_pg_stats_data *pg_stat_data);
		int (*pmu_pg_init_param)(struct gk20a *g, u32 pg_engine_id);
		u32 (*pmu_pg_supported_engines_list)(struct gk20a *g);
		u32 (*pmu_pg_engines_feature_list)(struct gk20a *g,
			u32 pg_engine_id);
		bool (*pmu_is_lpwr_feature_supported)(struct gk20a *g,
			u32 feature_id);
		int (*pmu_lpwr_enable_pg)(struct gk20a *g, bool pstate_lock);
		int (*pmu_lpwr_disable_pg)(struct gk20a *g, bool pstate_lock);
		u32 (*pmu_pg_param_post_init)(struct gk20a *g);
		int (*send_lrf_tex_ltc_dram_overide_en_dis_cmd)
			(struct gk20a *g, u32 mask);
		void (*dump_secure_fuses)(struct gk20a *g);
		int (*reset)(struct gk20a *g);
		int (*falcon_wait_for_halt)(struct gk20a *g,
			unsigned int timeout);
		int (*falcon_clear_halt_interrupt_status)(struct gk20a *g,
			unsigned int timeout);
		int (*init_falcon_setup_hw)(struct gk20a *g,
			void *desc, u32 bl_sz);
		bool (*is_lazy_bootstrap)(u32 falcon_id);
		bool (*is_priv_load)(u32 falcon_id);
		void (*get_wpr)(struct gk20a *g, struct wpr_carveout_info *inf);
		int (*alloc_blob_space)(struct gk20a *g,
				size_t size, struct mem_desc *mem);
		int (*pmu_populate_loader_cfg)(struct gk20a *g,
			void *lsfm,	u32 *p_bl_gen_desc_size);
		int (*flcn_populate_bl_dmem_desc)(struct gk20a *g,
			void *lsfm,	u32 *p_bl_gen_desc_size, u32 falconid);
		int (*mclk_init)(struct gk20a *g);
		void (*mclk_deinit)(struct gk20a *g);
		u32  lspmuwprinitdone;
		u32  lsfloadedfalconid;
		bool fecsbootstrapdone;
	} pmu;
	struct {
		void (*disable_slowboot)(struct gk20a *g);
		int (*init_clk_support)(struct gk20a *g);
		int (*suspend_clk_support)(struct gk20a *g);
		u32 (*get_crystal_clk_hz)(struct gk20a *g);
		unsigned long (*measure_freq)(struct gk20a *g, u32 api_domain);
		unsigned long (*get_rate)(struct gk20a *g, u32 api_domain);
		int (*set_rate)(struct gk20a *g, u32 api_domain, unsigned long rate);
	} clk;
	struct {
		u32 (*get_arbiter_clk_domains)(struct gk20a *g);
		int (*get_arbiter_clk_range)(struct gk20a *g, u32 api_domain,
				u16 *min_mhz, u16 *max_mhz);
		int (*get_arbiter_clk_default)(struct gk20a *g, u32 api_domain,
				u16 *default_mhz);
		/* This function is inherently unsafe to call while
		 *  arbiter is running arbiter must be blocked
		 *  before calling this function */
		int (*get_current_pstate)(struct gk20a *g);
	} clk_arb;
	struct {
		int (*handle_pmu_perf_event)(struct gk20a *g, void *pmu_msg);
	} perf;
	bool privsecurity;
	bool securegpccs;
	bool pmupstate;
	struct {
		const struct regop_offset_range* (
				*get_global_whitelist_ranges)(void);
		int (*get_global_whitelist_ranges_count)(void);
		const struct regop_offset_range* (
				*get_context_whitelist_ranges)(void);
		int (*get_context_whitelist_ranges_count)(void);
		const u32* (*get_runcontrol_whitelist)(void);
		int (*get_runcontrol_whitelist_count)(void);
		const struct regop_offset_range* (
				*get_runcontrol_whitelist_ranges)(void);
		int (*get_runcontrol_whitelist_ranges_count)(void);
		const u32* (*get_qctl_whitelist)(void);
		int (*get_qctl_whitelist_count)(void);
		const struct regop_offset_range* (
				*get_qctl_whitelist_ranges)(void);
		int (*get_qctl_whitelist_ranges_count)(void);
		int (*apply_smpc_war)(struct dbg_session_gk20a *dbg_s);
	} regops;
	struct {
		void (*intr_enable)(struct gk20a *g);
		void (*intr_unit_config)(struct gk20a *g,
				bool enable, bool is_stalling, u32 unit);
		irqreturn_t (*isr_stall)(struct gk20a *g);
		irqreturn_t (*isr_nonstall)(struct gk20a *g);
		irqreturn_t (*isr_thread_stall)(struct gk20a *g);
		void (*isr_thread_nonstall)(struct gk20a *g, u32 intr);
		void (*isr_nonstall_cb)(struct work_struct *work);
		bool (*is_intr_hub_pending)(struct gk20a *g, u32 mc_intr);
		u32 intr_mask_restore[4];
		void (*enable)(struct gk20a *g, u32 units);
		void (*disable)(struct gk20a *g, u32 units);
		void (*reset)(struct gk20a *g, u32 units);
		u32 (*boot_0)(struct gk20a *g, u32 *arch, u32 *impl, u32 *rev);
	} mc;
	struct {
		void (*show_dump)(struct gk20a *g,
				struct gk20a_debug_output *o);
	} debug;
	struct {
		int (*exec_reg_ops)(struct dbg_session_gk20a *dbg_s,
			    struct nvgpu_dbg_gpu_reg_op *ops,
			    u64 num_ops);
		int (*dbg_set_powergate)(struct dbg_session_gk20a *dbg_s,
					u32 mode);
		bool (*check_and_set_global_reservation)(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
		bool (*check_and_set_context_reservation)(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
		void (*release_profiler_reservation)(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
	} dbg_session_ops;
	struct {
		void (*get_program_numbers)(struct gk20a *g,
					    u32 block_height_log2,
					    int *hprog, int *vprog);
		bool (*need_scatter_buffer)(struct gk20a *g);
		int (*populate_scatter_buffer)(struct gk20a *g,
					       struct sg_table *sgt,
					       size_t surface_size,
					       void *scatter_buffer_ptr,
					       size_t scatter_buffer_size);
	} cde;

	int (*get_litter_value)(struct gk20a *g, int value);
	int (*chip_init_gpu_characteristics)(struct gk20a *g);

	struct {
		void (*init_hw)(struct gk20a *g);
		void (*isr)(struct gk20a *g);
		int (*read_ptimer)(struct gk20a *g, u64 *value);
		int (*bar1_bind)(struct gk20a *g, struct mem_desc *bar1_inst);
	} bus;

	int (*bios_init)(struct gk20a *g);

#if defined(CONFIG_GK20A_CYCLE_STATS)
	struct {
		int (*enable_snapshot)(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *client);
		void (*disable_snapshot)(struct gr_gk20a *gr);
		int (*check_data_available)(struct channel_gk20a *ch,
						u32 *pending,
						bool *hw_overflow);
		void (*set_handled_snapshots)(struct gk20a *g, u32 num);
		u32 (*allocate_perfmon_ids)(struct gk20a_cs_snapshot *data,
				       u32 count);
		u32 (*release_perfmon_ids)(struct gk20a_cs_snapshot *data,
				      u32 start,
				      u32 count);
		int (*detach_snapshot)(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *client);
	} css;
#endif
	struct {
		int (*sw_init)(struct device *dev);
		int (*get_speed)(struct gk20a *g, u32 *xve_link_speed);
		int (*set_speed)(struct gk20a *g, u32 xve_link_speed);
		void (*available_speeds)(struct gk20a *g, u32 *speed_mask);
		u32 (*xve_readl)(struct gk20a *g, u32 reg);
		void (*xve_writel)(struct gk20a *g, u32 reg, u32 val);
		void (*disable_aspm)(struct gk20a *g);
		void (*reset_gpu)(struct gk20a *g);
#if defined(CONFIG_PCI_MSI)
		void (*rearm_msi)(struct gk20a *g);
#endif
		void (*enable_shadow_rom)(struct gk20a *g);
		void (*disable_shadow_rom)(struct gk20a *g);
	} xve;
};

struct nvgpu_bios_ucode {
	u8 *bootloader;
	u32 bootloader_phys_base;
	u32 bootloader_size;
	u8 *ucode;
	u32 phys_base;
	u32 size;
	u8 *dmem;
	u32 dmem_phys_base;
	u32 dmem_size;
	u32 code_entry_point;
};

struct nvgpu_bios {
	u8 *data;
	size_t size;

	struct nvgpu_bios_ucode devinit;
	struct nvgpu_bios_ucode preos;

	u8 *devinit_tables;
	u32 devinit_tables_size;
	u8 *bootscripts;
	u32 bootscripts_size;

	u8 mem_strap_data_count;
	u16 mem_strap_xlat_tbl_ptr;

	u32 condition_table_ptr;

	u32 devinit_tables_phys_base;
	u32 devinit_script_phys_base;

	struct bit_token *perf_token;
	struct bit_token *clock_token;
	struct bit_token *virt_token;
	u32 expansion_rom_offset;
};

struct gk20a {
	struct device *dev;
	struct platform_device *host1x_dev;

	atomic_t usage_count;
	int driver_is_dying;

	atomic_t nonstall_ops;
	struct work_struct nonstall_fn_work;
	struct workqueue_struct *nonstall_work_queue;

	bool is_fmodel;

	struct kref refcount;

	struct resource *reg_mem;
	void __iomem *regs;
	void __iomem *regs_saved;

	struct resource *bar1_mem;
	void __iomem *bar1;
	void __iomem *bar1_saved;

	const char *name;

	bool gpu_reset_done;
	bool power_on;
	bool suspended;

	u32 log_mask;
	u32 log_trace;

	struct rw_semaphore busy_lock;

	struct clk_gk20a clk;
	struct fifo_gk20a fifo;
	struct gr_gk20a gr;
	struct sim_gk20a sim;
	struct mm_gk20a mm;
	struct pmu_gk20a pmu;
	struct acr_desc acr;
	struct cooling_device_gk20a gk20a_cdev;
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	struct clk_pmupstate clk_pmu;
	struct perf_pmupstate perf_pmu;
	struct pmgr_pmupstate pmgr_pmu;
	struct therm_pmupstate therm_pmu;
#endif

#ifdef CONFIG_DEBUG_FS
	struct railgate_stats pstats;
#endif
	u32 gr_idle_timeout_default;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
	u32 timeouts_enabled;
#else
	bool timeouts_enabled;
#endif

	struct nvgpu_mutex poweroff_lock;

	/* Channel priorities */
	u32 timeslice_low_priority_us;
	u32 timeslice_medium_priority_us;
	u32 timeslice_high_priority_us;
	u32 min_timeslice_us;
	u32 max_timeslice_us;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
	u32 runlist_interleave;
#else
	bool runlist_interleave;
#endif

	bool slcg_enabled;
	bool blcg_enabled;
	bool elcg_enabled;
	bool elpg_enabled;
	bool aelpg_enabled;
	bool mscg_enabled;
	bool forced_idle;
	bool forced_reset;
	bool allow_all;

	u32 emc3d_ratio;

#ifdef CONFIG_DEBUG_FS
	struct nvgpu_spinlock debugfs_lock;
	struct dentry *debugfs_ltc_enabled;
	struct dentry *debugfs_timeouts_enabled;
	struct dentry *debugfs_gr_idle_timeout_default;
	struct dentry *debugfs_bypass_smmu;
	struct dentry *debugfs_disable_bigpage;
	struct dentry *debugfs_gr_default_attrib_cb_size;

	struct dentry *debugfs_timeslice_low_priority_us;
	struct dentry *debugfs_timeslice_medium_priority_us;
	struct dentry *debugfs_timeslice_high_priority_us;
	struct dentry *debugfs_runlist_interleave;
	struct dentry *debugfs_allocators;
	struct dentry *debugfs_xve;
	struct dentry *debugfs_kmem;
#endif
	struct gk20a_ctxsw_ucode_info ctxsw_ucode_info;

	/*
	 * A group of semaphore pools. One for each channel.
	 */
	struct nvgpu_semaphore_sea *sema_sea;

	/* List of pending SW semaphore waits. */
	struct list_head pending_sema_waits;
	struct nvgpu_raw_spinlock pending_sema_waits_lock;

	/* held while manipulating # of debug/profiler sessions present */
	/* also prevents debug sessions from attaching until released */
	struct nvgpu_mutex dbg_sessions_lock;
	int dbg_powergating_disabled_refcount; /*refcount for pg disable */
	int dbg_timeout_disabled_refcount; /*refcount for timeout disable */

	/* must have dbg_sessions_lock before use */
	struct nvgpu_dbg_gpu_reg_op *dbg_regops_tmp_buf;
	u32 dbg_regops_tmp_buf_ops;

	/* For profiler reservations */
	struct list_head profiler_objects;
	bool global_profiler_reservation_held;
	int profiler_reservation_count;

	void (*remove_support)(struct gk20a *);

	u64 pg_ingating_time_us;
	u64 pg_ungating_time_us;
	u32 pg_gating_cnt;

	struct nvgpu_spinlock mc_enable_lock;

	struct nvgpu_gpu_characteristics gpu_characteristics;

	struct {
		struct cdev cdev;
		struct device *node;
	} channel;

	struct gk20a_as as;

	struct {
		struct cdev cdev;
		struct device *node;
	} ctrl;

	struct {
		struct cdev cdev;
		struct device *node;
	} as_dev;

	struct {
		struct cdev cdev;
		struct device *node;
	} dbg;

	struct {
		struct cdev cdev;
		struct device *node;
	} prof;

	struct {
		struct cdev cdev;
		struct device *node;
	} tsg;

	struct {
		struct cdev cdev;
		struct device *node;
	} ctxsw;

	struct {
		struct cdev cdev;
		struct device *node;
	} sched;

	struct nvgpu_mutex client_lock;
	int client_refcount; /* open channels and ctrl nodes */

	dev_t cdev_region;

	struct gpu_ops ops;

	int irqs_enabled;
	int irq_stall; /* can be same as irq_nonstall in case of PCI */
	int irq_nonstall;
	u32 max_ltc_count;
	u32 ltc_count;

	atomic_t hw_irq_stall_count;
	atomic_t hw_irq_nonstall_count;

	atomic_t sw_irq_stall_last_handled;
	wait_queue_head_t sw_irq_stall_last_handled_wq;

	atomic_t sw_irq_nonstall_last_handled;
	wait_queue_head_t sw_irq_nonstall_last_handled_wq;

	struct gk20a_channel_worker {
		struct task_struct *poll_task;
		atomic_t put;
		wait_queue_head_t wq;
		struct list_head items;
		struct nvgpu_spinlock items_lock;
	} channel_worker;

	struct devfreq *devfreq;

	struct gk20a_scale_profile *scale_profile;
	unsigned long last_freq;

	struct gk20a_ctxsw_trace *ctxsw_trace;
	struct gk20a_fecs_trace *fecs_trace;

	struct gk20a_sched_ctrl sched_ctrl;

	struct device_dma_parameters dma_parms;

	struct gk20a_cde_app cde_app;
	bool mmu_debug_ctrl;

	u32 tpc_fs_mask_user;

	struct nvgpu_bios bios;
	struct debugfs_blob_wrapper bios_blob;

	struct nvgpu_clk_arb *clk_arb;

	struct gk20a_ce_app ce_app;

	/* PCI device identifier */
	u16 pci_vendor_id, pci_device_id;
	u16 pci_subsystem_vendor_id, pci_subsystem_device_id;
	u16 pci_class;
	u8 pci_revision;

	/* PCIe power states. */
	bool xve_l0s;
	bool xve_l1;

	/* Current warning temp in sfxp24.8 */
	s32 curr_warn_temp;

#if defined(CONFIG_PCI_MSI)
	/* Check if msi is enabled */
	bool msi_enabled;
#endif
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	struct nvgpu_mem_alloc_tracker *vmallocs;
	struct nvgpu_mem_alloc_tracker *kmallocs;
#endif
};

static inline unsigned long gk20a_get_gr_idle_timeout(struct gk20a *g)
{
	return g->timeouts_enabled ?
		g->gr_idle_timeout_default : MAX_SCHEDULE_TIMEOUT;
}

static inline struct gk20a *get_gk20a(struct device *dev)
{
	return gk20a_get_platform(dev)->g;
}

enum BAR0_DEBUG_OPERATION {
	BARO_ZERO_NOP = 0,
	OP_END = 'DONE',
	BAR0_READ32 = '0R32',
	BAR0_WRITE32 = '0W32',
};

struct share_buffer_head {
	enum BAR0_DEBUG_OPERATION operation;
/* size of the operation item */
	u32 size;
	u32 completed;
	u32 failed;
	u64 context;
	u64 completion_callback;
};

struct gk20a_cyclestate_buffer_elem {
	struct share_buffer_head	head;
/* in */
	u64 p_data;
	u64 p_done;
	u32 offset_bar0;
	u16 first_bit;
	u16 last_bit;
/* out */
/* keep 64 bits to be consistent */
	u64 data;
};

/* operations that will need to be executed on non stall workqueue */
enum gk20a_nonstall_ops {
	gk20a_nonstall_ops_wakeup_semaphore = BIT(0), /* wake up semaphore */
	gk20a_nonstall_ops_post_events = BIT(1),
};

void gk20a_init_clk_ops(struct gpu_ops *gops);

/* register accessors */
int gk20a_lockout_registers(struct gk20a *g);
int gk20a_restore_registers(struct gk20a *g);

void __nvgpu_check_gpu_state(struct gk20a *g);
void __gk20a_warn_on_no_regs(void);

static inline void gk20a_writel(struct gk20a *g, u32 r, u32 v)
{
	if (unlikely(!g->regs)) {
		__gk20a_warn_on_no_regs();
		gk20a_dbg(gpu_dbg_reg, "r=0x%x v=0x%x (failed)", r, v);
	} else {
		writel_relaxed(v, g->regs + r);
		wmb();
		gk20a_dbg(gpu_dbg_reg, "r=0x%x v=0x%x", r, v);
	}
}
static inline u32 gk20a_readl(struct gk20a *g, u32 r)
{

	u32 v = 0xffffffff;

	if (unlikely(!g->regs)) {
		__gk20a_warn_on_no_regs();
		gk20a_dbg(gpu_dbg_reg, "r=0x%x v=0x%x (failed)", r, v);
	} else {
		v = readl(g->regs + r);
		if (v == 0xffffffff)
			__nvgpu_check_gpu_state(g);
		gk20a_dbg(gpu_dbg_reg, "r=0x%x v=0x%x", r, v);
	}

	return v;
}
static inline void gk20a_writel_check(struct gk20a *g, u32 r, u32 v)
{
	if (unlikely(!g->regs)) {
		__gk20a_warn_on_no_regs();
		gk20a_dbg(gpu_dbg_reg, "r=0x%x v=0x%x (failed)", r, v);
	} else {
		wmb();
		do {
			writel_relaxed(v, g->regs + r);
		} while (readl(g->regs + r) != v);
		gk20a_dbg(gpu_dbg_reg, "r=0x%x v=0x%x", r, v);
	}
}

static inline void gk20a_bar1_writel(struct gk20a *g, u32 b, u32 v)
{
	if (unlikely(!g->bar1)) {
		__gk20a_warn_on_no_regs();
		gk20a_dbg(gpu_dbg_reg, "b=0x%x v=0x%x (failed)", b, v);
	} else {
		wmb();
		writel_relaxed(v, g->bar1 + b);
		gk20a_dbg(gpu_dbg_reg, "b=0x%x v=0x%x", b, v);
	}
}

static inline u32 gk20a_bar1_readl(struct gk20a *g, u32 b)
{
	u32 v = 0xffffffff;

	if (unlikely(!g->bar1)) {
		__gk20a_warn_on_no_regs();
		gk20a_dbg(gpu_dbg_reg, "b=0x%x v=0x%x (failed)", b, v);
	} else {
		v = readl(g->bar1 + b);
		gk20a_dbg(gpu_dbg_reg, "b=0x%x v=0x%x", b, v);
	}

	return v;
}

/* convenience */
static inline struct device *dev_from_gk20a(struct gk20a *g)
{
	return g->dev;
}
static inline struct gk20a *gk20a_from_dev(struct device *dev)
{
	if (!dev)
		return NULL;

	return ((struct gk20a_platform *)dev_get_drvdata(dev))->g;
}
static inline struct gk20a *gk20a_from_as(struct gk20a_as *as)
{
	return container_of(as, struct gk20a, as);
}
static inline struct gk20a *gk20a_from_pmu(struct pmu_gk20a *pmu)
{
	return container_of(pmu, struct gk20a, pmu);
}

static inline u32 u64_hi32(u64 n)
{
	return (u32)((n >> 32) & ~(u32)0);
}

static inline u32 u64_lo32(u64 n)
{
	return (u32)(n & ~(u32)0);
}

static inline u64 hi32_lo32_to_u64(u32 hi, u32 lo)
{
	return  (((u64)hi) << 32) | (u64)lo;
}

static inline u32 set_field(u32 val, u32 mask, u32 field)
{
	return ((val & ~mask) | field);
}

static inline u32 get_field(u32 reg, u32 mask)
{
	return (reg & mask);
}

/* invalidate channel lookup tlb */
static inline void gk20a_gr_flush_channel_tlb(struct gr_gk20a *gr)
{
	nvgpu_spinlock_acquire(&gr->ch_tlb_lock);
	memset(gr->chid_tlb, 0,
		sizeof(struct gr_channel_map_tlb_entry) *
		GR_CHANNEL_MAP_TLB_SIZE);
	nvgpu_spinlock_release(&gr->ch_tlb_lock);
}

/* classes that the device supports */
/* TBD: get these from an open-sourced SDK? */
enum {
	KEPLER_C                  = 0xA297,
	FERMI_TWOD_A              = 0x902D,
	KEPLER_COMPUTE_A          = 0xA0C0,
	KEPLER_INLINE_TO_MEMORY_A = 0xA040,
	KEPLER_DMA_COPY_A         = 0xA0B5,
	KEPLER_CHANNEL_GPFIFO_C   = 0xA26F,
};

static inline bool gk20a_gpu_is_virtual(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);

	return platform->virtual_dev;
}

static inline int support_gk20a_pmu(struct device *dev)
{
	if (IS_ENABLED(CONFIG_GK20A_PMU)) {
		/* gPMU is not supported for vgpu */
		return !gk20a_gpu_is_virtual(dev);
	}

	return 0;
}

void gk20a_create_sysfs(struct device *dev);
void gk20a_remove_sysfs(struct device *dev);

#define GK20A_BAR0_IORESOURCE_MEM 0
#define GK20A_BAR1_IORESOURCE_MEM 1
#define GK20A_SIM_IORESOURCE_MEM 2

void gk20a_busy_noresume(struct device *dev);
void gk20a_idle_nosuspend(struct device *dev);
int __must_check gk20a_busy(struct gk20a *g);
void gk20a_idle(struct gk20a *g);
int gk20a_do_idle(void);
int gk20a_do_unidle(void);
int __gk20a_do_idle(struct device *dev, bool force_reset);
int __gk20a_do_unidle(struct device *dev);

void gk20a_driver_start_unload(struct gk20a *g);
int gk20a_wait_for_idle(struct device *dev);

#define NVGPU_GPU_ARCHITECTURE_SHIFT 4

/* constructs unique and compact GPUID from nvgpu_gpu_characteristics
 * arch/impl fields */
#define GK20A_GPUID(arch, impl) ((u32) ((arch) | (impl)))

#define GK20A_GPUID_GK20A \
	GK20A_GPUID(NVGPU_GPU_ARCH_GK100, NVGPU_GPU_IMPL_GK20A)
#define GK20A_GPUID_GM20B \
	GK20A_GPUID(NVGPU_GPU_ARCH_GM200, NVGPU_GPU_IMPL_GM20B)
#define NVGPU_GPUID_GP10B \
	GK20A_GPUID(NVGPU_GPU_ARCH_GP100, NVGPU_GPU_IMPL_GP10B)
#define NVGPU_GPUID_GP104 \
	GK20A_GPUID(NVGPU_GPU_ARCH_GP100, NVGPU_GPU_IMPL_GP104)
#define NVGPU_GPUID_GP106 \
	GK20A_GPUID(NVGPU_GPU_ARCH_GP100, NVGPU_GPU_IMPL_GP106)

int gk20a_init_gpu_characteristics(struct gk20a *g);

int gk20a_user_init(struct device *dev, const char *interface_name,
		    struct class *class);
void gk20a_user_deinit(struct device *dev, struct class *class);

static inline u32 ptimer_scalingfactor10x(u32 ptimer_src_freq)
{
	return (u32)(((u64)(PTIMER_REF_FREQ_HZ * 10)) / ptimer_src_freq);
}
static inline u32 scale_ptimer(u32 timeout , u32 scale10x)
{
	if (((timeout*10) % scale10x) >= (scale10x/2))
		return ((timeout * 10) / scale10x) + 1;
	else
		return (timeout * 10) / scale10x;
}

extern struct class nvgpu_class;

#define INTERFACE_NAME "nvhost%s-gpu"

int gk20a_pm_init(struct device *dev);
int gk20a_pm_finalize_poweron(struct device *dev);
void gk20a_remove_support(struct gk20a *g);

static inline struct tsg_gk20a *tsg_gk20a_from_ch(struct channel_gk20a *ch)
{
	struct tsg_gk20a *tsg = NULL;

	if (gk20a_is_channel_marked_as_tsg(ch))
	{
		struct gk20a *g = ch->g;
		struct fifo_gk20a *f = &g->fifo;
		tsg = &f->tsg[ch->tsgid];
	}

	return tsg;
}

static inline void gk20a_channel_trace_sched_param(
	void (*trace)(int chid, int tsgid, pid_t pid, u32 timeslice,
		u32 timeout, const char *interleave,
		const char *graphics_preempt_mode,
		const char *compute_preempt_mode),
	struct channel_gk20a *ch)
{
	(trace)(ch->hw_chid, ch->tsgid, ch->pid,
		gk20a_is_channel_marked_as_tsg(ch) ?
			tsg_gk20a_from_ch(ch)->timeslice_us : ch->timeslice_us,
		ch->timeout_ms_max,
		gk20a_fifo_interleave_level_name(ch->interleave_level),
		gr_gk20a_graphics_preempt_mode_name(ch->ch_ctx.gr_ctx ?
			ch->ch_ctx.gr_ctx->graphics_preempt_mode : 0),
		gr_gk20a_compute_preempt_mode_name(ch->ch_ctx.gr_ctx ?
			ch->ch_ctx.gr_ctx->compute_preempt_mode : 0));
}

void nvgpu_wait_for_deferred_interrupts(struct gk20a *g);

struct gk20a * __must_check gk20a_get(struct gk20a *g);
void gk20a_put(struct gk20a *g);

#ifdef CONFIG_DEBUG_FS
int gk20a_railgating_debugfs_init(struct device *dev);
#endif

int gk20a_secure_page_alloc(struct device *dev);
void __iomem *gk20a_ioremap_resource(struct platform_device *dev, int i,
					    struct resource **out);

#endif /* GK20A_H */
