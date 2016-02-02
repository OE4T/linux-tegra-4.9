/*
 * GK20A Graphics
 *
 * Copyright (c) 2011-2016, NVIDIA CORPORATION.  All rights reserved.
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
struct acr_gm20b;

#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/nvgpu.h>
#include <linux/irqreturn.h>
#include <linux/tegra-soc.h>

#include "../../../arch/arm/mach-tegra/iomap.h"

#include "as_gk20a.h"
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

/* PTIMER_REF_FREQ_HZ corresponds to a period of 32 nanoseconds.
    32 ns is the resolution of ptimer. */
#define PTIMER_REF_FREQ_HZ                      31250000

#define MAX_INTERLEAVED_CHANNELS                32

struct cooling_device_gk20a {
	struct thermal_cooling_device *gk20a_cooling_dev;
	unsigned int gk20a_freq_state;
	unsigned int gk20a_freq_table_size;
	struct gk20a *g;
};

enum gk20a_cbc_op {
	gk20a_cbc_op_clear,
	gk20a_cbc_op_clean,
	gk20a_cbc_op_invalidate,
};

#define MC_INTR_UNIT_DISABLE	false
#define MC_INTR_UNIT_ENABLE		true

struct gpu_ops {
	struct {
		int (*determine_L2_size_bytes)(struct gk20a *gk20a);
		void (*set_max_ways_evict_last)(struct gk20a *g, u32 max_ways);
		int (*init_comptags)(struct gk20a *g, struct gr_gk20a *gr);
		int (*cbc_ctrl)(struct gk20a *g, enum gk20a_cbc_op op,
				u32 min, u32 max);
		void (*set_zbc_color_entry)(struct gk20a *g,
					    struct zbc_entry *color_val,
					    u32 index);
		void (*set_zbc_depth_entry)(struct gk20a *g,
					    struct zbc_entry *depth_val,
					    u32 index);
		void (*init_cbc)(struct gk20a *g, struct gr_gk20a *gr);
		void (*sync_debugfs)(struct gk20a *g);
		void (*init_fs_state)(struct gk20a *g);
		void (*elpg_flush)(struct gk20a *g);
		void (*isr)(struct gk20a *g);
		u32 (*cbc_fix_config)(struct gk20a *g, int base);
		void (*flush)(struct gk20a *g);
	} ltc;
	struct {
		void (*isr_stall)(struct gk20a *g);
		void (*isr_nonstall)(struct gk20a *g);
	} ce2;
	struct {
		int (*init_fs_state)(struct gk20a *g);
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
		int (*free_obj_ctx)(struct channel_gk20a  *c,
				struct nvgpu_free_obj_ctx_args *args);
		int (*bind_ctxsw_zcull)(struct gk20a *g, struct gr_gk20a *gr,
				struct channel_gk20a *c, u64 zcull_va,
				u32 mode);
		int (*get_zcull_info)(struct gk20a *g, struct gr_gk20a *gr,
				struct gr_zcull_info *zcull_params);
		bool (*is_tpc_addr)(u32 addr);
		u32 (*get_tpc_num)(u32 addr);
		void (*detect_sm_arch)(struct gk20a *g);
		int (*add_zbc_color)(struct gk20a *g, struct gr_gk20a *gr,
				  struct zbc_entry *color_val, u32 index);
		int (*add_zbc_depth)(struct gk20a *g, struct gr_gk20a *gr,
				  struct zbc_entry *depth_val, u32 index);
		int (*zbc_set_table)(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_entry *zbc_val);
		int (*zbc_query_table)(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_query_params *query_params);
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
				void *ctx_ptr);
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
		int (*wait_empty)(struct gk20a *g, unsigned long end_jiffies,
		       u32 expect_delay);
		void (*init_cyclestats)(struct gk20a *g);
		void (*enable_cde_in_fecs)(void *ctx_ptr);
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
			bool *post_event, struct channel_gk20a *fault_ch);
		int (*handle_tex_exception)(struct gk20a *g, u32 gpc, u32 tpc,
						bool *post_event);
		void (*create_gr_sysfs)(struct platform_device *dev);
	} gr;
	const char *name;
	struct {
		void (*init_fs_state)(struct gk20a *g);
		void (*reset)(struct gk20a *g);
		void (*init_uncompressed_kind_map)(struct gk20a *g);
		void (*init_kind_attr)(struct gk20a *g);
		void (*set_mmu_page_size)(struct gk20a *g);
		bool (*set_use_full_comp_tag_line)(struct gk20a *g);
		int (*compression_page_size)(struct gk20a *g);
		int (*compressible_page_size)(struct gk20a *g);
		void (*dump_vpr_wpr_info)(struct gk20a *g);
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
		void (*bind_channel)(struct channel_gk20a *ch_gk20a);
		void (*unbind_channel)(struct channel_gk20a *ch_gk20a);
		void (*disable_channel)(struct channel_gk20a *ch);
		void (*enable_channel)(struct channel_gk20a *ch);
		int (*alloc_inst)(struct gk20a *g, struct channel_gk20a *ch);
		void (*free_inst)(struct gk20a *g, struct channel_gk20a *ch);
		int (*setup_ramfc)(struct channel_gk20a *c, u64 gpfifo_base,
				u32 gpfifo_entries, u32 flags);
		int (*resetup_ramfc)(struct channel_gk20a *c);
		int (*preempt_channel)(struct gk20a *g, u32 hw_chid);
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
		int (*get_netlist_name)(int index, char *name);
		bool (*is_fw_defined)(void);
		bool use_dma_for_fw_bootstrap;
	} gr_ctx;
	struct {
		bool (*support_sparse)(struct gk20a *g);
		bool (*is_debug_mode_enabled)(struct gk20a *g);
		void (*set_debug_mode)(struct gk20a *g, bool enable);
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
				struct vm_gk20a_mapping_batch *batch);
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
		void (*tlb_invalidate)(struct vm_gk20a *vm);
		void (*set_big_page_size)(struct gk20a *g,
					  void *inst_ptr, int size);
		u32 (*get_big_page_sizes)(void);
		u32 (*get_physical_addr_bits)(struct gk20a *g);
		int (*init_mm_setup_hw)(struct gk20a *g);
		int (*init_bar2_vm)(struct gk20a *g);
		int (*init_bar2_mm_hw_setup)(struct gk20a *g);
		void (*remove_bar2_vm)(struct gk20a *g);
		const struct gk20a_mmu_level *
			(*get_mmu_levels)(struct gk20a *g, u32 big_page_size);
		void (*init_pdb)(struct gk20a *g, void *inst_ptr, u64 pdb_addr);
		u64 (*get_iova_addr)(struct gk20a *g, struct scatterlist *sgl,
					 u32 flags);
	} mm;
	struct {
		int (*init_therm_setup_hw)(struct gk20a *g);
		int (*update_therm_gate_ctrl)(struct gk20a *g);
	} therm;
	struct {
		int (*prepare_ucode)(struct gk20a *g);
		int (*pmu_setup_hw_and_bootstrap)(struct gk20a *g);
		int (*pmu_nsbootstrap)(struct pmu_gk20a *pmu);
		int (*pmu_setup_elpg)(struct gk20a *g);
		int (*init_wpr_region)(struct gk20a *g);
		int (*load_lsfalcon_ucode)(struct gk20a *g, u32 falconidmask);
		void (*write_dmatrfbase)(struct gk20a *g, u32 addr);
		void (*pmu_elpg_statistics)(struct gk20a *g,
			u32 *ingating_time, u32 *ungating_time,
			u32 *gating_cnt);
		int (*pmu_pg_grinit_param)(struct gk20a *g,
			u8 grfeaturemask);
		u32  lspmuwprinitdone;
		u32  lsfloadedfalconid;
		bool fecsbootstrapdone;
	} pmu;
	struct {
		void (*disable_slowboot)(struct gk20a *g);
		int (*init_clk_support)(struct gk20a *g);
		int (*suspend_clk_support)(struct gk20a *g);
	} clk;
	bool privsecurity;
	bool securegpccs;
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
		irqreturn_t (*isr_thread_nonstall)(struct gk20a *g);
		u32 intr_mask_restore[4];
	} mc;
	struct {
		void (*show_dump)(struct gk20a *g,
				struct gk20a_debug_output *o);
	} debug;
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
};

struct gk20a {
	struct platform_device *dev;
	struct platform_device *host1x_dev;

	struct resource *reg_mem;
	void __iomem *regs;
	void __iomem *regs_saved;

	struct resource *bar1_mem;
	void __iomem *bar1;
	void __iomem *bar1_saved;

	bool power_on;

	struct rw_semaphore busy_lock;

	struct clk_gk20a clk;
	struct fifo_gk20a fifo;
	struct gr_gk20a gr;
	struct sim_gk20a sim;
	struct mm_gk20a mm;
	struct pmu_gk20a pmu;
	struct acr_gm20b acr;
	struct cooling_device_gk20a gk20a_cdev;

	/* Save pmu fw here so that it lives cross suspend/resume.
	   pmu suspend destroys all pmu sw/hw states. Loading pmu
	   fw in resume crashes when the resume is from sys_exit. */
	const struct firmware *pmu_fw;

	u32 gr_idle_timeout_default;
	u32 timeouts_enabled;

	struct mutex ch_wdt_lock;

	struct mutex poweroff_lock;

	/* Channel priorities */
	u32 timeslice_low_priority_us;
	u32 timeslice_medium_priority_us;
	u32 timeslice_high_priority_us;
	u32 interleave_high_priority;

	struct mutex interleave_lock;
	u32 num_interleaved_channels;

	bool slcg_enabled;
	bool blcg_enabled;
	bool elcg_enabled;
	bool elpg_enabled;
	bool aelpg_enabled;
	bool forced_idle;
	bool forced_reset;
	bool allow_all;

	u32 emc3d_ratio;

#ifdef CONFIG_DEBUG_FS
	spinlock_t debugfs_lock;
	struct dentry *debugfs_ltc_enabled;
	struct dentry *debugfs_timeouts_enabled;
	struct dentry *debugfs_gr_idle_timeout_default;
	struct dentry *debugfs_bypass_smmu;
	struct dentry *debugfs_disable_bigpage;
	struct dentry *debugfs_gr_default_attrib_cb_size;

	struct dentry *debugfs_timeslice_low_priority_us;
	struct dentry *debugfs_timeslice_medium_priority_us;
	struct dentry *debugfs_timeslice_high_priority_us;
	struct dentry *debugfs_interleave_high_priority;

#endif
	struct gk20a_ctxsw_ucode_info ctxsw_ucode_info;

	/* held while manipulating # of debug/profiler sessions present */
	/* also prevents debug sessions from attaching until released */
	struct mutex dbg_sessions_lock;
	int dbg_sessions; /* number attached */
	int dbg_powergating_disabled_refcount; /*refcount for pg disable */
	int dbg_timeout_disabled_refcount; /*refcount for timeout disable */

	void (*remove_support)(struct platform_device *);

	u64 pg_ingating_time_us;
	u64 pg_ungating_time_us;
	u32 pg_gating_cnt;

	spinlock_t mc_enable_lock;

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
	} dbg;

	struct {
		struct cdev cdev;
		struct device *node;
	} prof;

	struct {
		struct cdev cdev;
		struct device *node;
	} tsg;

	struct mutex client_lock;
	int client_refcount; /* open channels and ctrl nodes */

	dev_t cdev_region;
	struct class *class;

	struct gpu_ops ops;

	int irq_stall;
	int irq_nonstall;
	u32 max_ltc_count;
	u32 ltc_count;

	atomic_t hw_irq_stall_count;
	atomic_t hw_irq_nonstall_count;

	atomic_t sw_irq_stall_last_handled;
	wait_queue_head_t sw_irq_stall_last_handled_wq;

	atomic_t sw_irq_nonstall_last_handled;
	wait_queue_head_t sw_irq_nonstall_last_handled_wq;

	struct devfreq *devfreq;

	struct gk20a_scale_profile *scale_profile;

	struct device_dma_parameters dma_parms;

	struct gk20a_cde_app cde_app;
	bool mmu_debug_ctrl;
};

static inline unsigned long gk20a_get_gr_idle_timeout(struct gk20a *g)
{
	return g->timeouts_enabled ?
		g->gr_idle_timeout_default : MAX_SCHEDULE_TIMEOUT;
}

static inline struct gk20a *get_gk20a(struct platform_device *dev)
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

struct gk20a_domain_data {
	struct generic_pm_domain gpd;
	struct gk20a *gk20a;
};

/* debug accessories */

#ifdef CONFIG_DEBUG_FS
    /* debug info, default is compiled-in but effectively disabled (0 mask) */
    #define GK20A_DEBUG
    /*e.g: echo 1 > /d/gk20a.0/dbg_mask */
    #define GK20A_DEFAULT_DBG_MASK 0
#else
    /* manually enable and turn it on the mask */
    /*#define NVGPU_DEBUG*/
    #define GK20A_DEFAULT_DBG_MASK (dbg_info)
#endif

enum gk20a_dbg_categories {
	gpu_dbg_info    = BIT(0),  /* lightly verbose info */
	gpu_dbg_fn      = BIT(2),  /* fn name tracing */
	gpu_dbg_reg     = BIT(3),  /* register accesses, very verbose */
	gpu_dbg_pte     = BIT(4),  /* gmmu ptes */
	gpu_dbg_intr    = BIT(5),  /* interrupts */
	gpu_dbg_pmu     = BIT(6),  /* gk20a pmu */
	gpu_dbg_clk     = BIT(7),  /* gk20a clk */
	gpu_dbg_map     = BIT(8),  /* mem mappings */
	gpu_dbg_gpu_dbg = BIT(9),  /* gpu debugger/profiler */
	gpu_dbg_cde     = BIT(10), /* cde info messages */
	gpu_dbg_cde_ctx = BIT(11), /* cde context usage messages */
	gpu_dbg_mem     = BIT(31), /* memory accesses, very verbose */
};

#if defined(GK20A_DEBUG)
extern u32 gk20a_dbg_mask;
extern u32 gk20a_dbg_ftrace;
#define gk20a_dbg(dbg_mask, format, arg...)				\
do {									\
	if (unlikely((dbg_mask) & gk20a_dbg_mask)) {		\
		if (gk20a_dbg_ftrace)					\
			trace_printk(format "\n", ##arg);		\
		else							\
			pr_info("gk20a %s: " format "\n",		\
					__func__, ##arg);		\
	}								\
} while (0)

#else /* GK20A_DEBUG */
#define gk20a_dbg(dbg_mask, format, arg...)				\
do {									\
	if (0)								\
		pr_info("gk20a %s: " format "\n", __func__, ##arg);\
} while (0)

#endif

#define gk20a_err(d, fmt, arg...) \
	dev_err(d, "%s: " fmt "\n", __func__, ##arg)

#define gk20a_warn(d, fmt, arg...) \
	dev_warn(d, "%s: " fmt "\n", __func__, ##arg)

#define gk20a_dbg_fn(fmt, arg...) \
	gk20a_dbg(gpu_dbg_fn, fmt, ##arg)

#define gk20a_dbg_info(fmt, arg...) \
	gk20a_dbg(gpu_dbg_info, fmt, ##arg)

/* mem access with dbg_mem logging */
static inline u8 gk20a_mem_rd08(void *ptr, int b)
{
	u8 _b = ((const u8 *)ptr)[b];
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u8)*b, _b);
#endif
	return _b;
}
static inline u16 gk20a_mem_rd16(void *ptr, int s)
{
	u16 _s = ((const u16 *)ptr)[s];
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u16)*s, _s);
#endif
	return _s;
}
static inline u32 gk20a_mem_rd32(void *ptr, int w)
{
	u32 _w = ((const u32 *)ptr)[w];
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr + sizeof(u32)*w, _w);
#endif
	return _w;
}
static inline void gk20a_mem_wr08(void *ptr, int b, u8 data)
{
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u8)*b, data);
#endif
	((u8 *)ptr)[b] = data;
}
static inline void gk20a_mem_wr16(void *ptr, int s, u16 data)
{
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u16)*s, data);
#endif
	((u16 *)ptr)[s] = data;
}
static inline void gk20a_mem_wr32(void *ptr, int w, u32 data)
{
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u32)*w, data);
#endif
	((u32 *)ptr)[w] = data;
}

void gk20a_init_clk_ops(struct gpu_ops *gops);

/* register accessors */
int gk20a_lockout_registers(struct gk20a *g);
int gk20a_restore_registers(struct gk20a *g);

static inline void gk20a_writel(struct gk20a *g, u32 r, u32 v)
{
	gk20a_dbg(gpu_dbg_reg, " r=0x%x v=0x%x", r, v);
	wmb();
	writel_relaxed(v, g->regs + r);
}
static inline u32 gk20a_readl(struct gk20a *g, u32 r)
{
	u32 v = readl(g->regs + r);
	gk20a_dbg(gpu_dbg_reg, " r=0x%x v=0x%x", r, v);
	return v;
}
static inline void gk20a_writel_check(struct gk20a *g, u32 r, u32 v)
{
	gk20a_dbg(gpu_dbg_reg, " r=0x%x v=0x%x", r, v);
	wmb();
	do {
		writel_relaxed(v, g->regs + r);
	} while (readl(g->regs + r) != v);
}

static inline void gk20a_bar1_writel(struct gk20a *g, u32 b, u32 v)
{
	gk20a_dbg(gpu_dbg_reg, " b=0x%x v=0x%x", b, v);
	wmb();
	writel_relaxed(v, g->bar1 + b);
}

static inline u32 gk20a_bar1_readl(struct gk20a *g, u32 b)
{
	u32 v = readl(g->bar1 + b);
	gk20a_dbg(gpu_dbg_reg, " b=0x%x v=0x%x", b, v);
	return v;
}

/* convenience */
static inline struct device *dev_from_gk20a(struct gk20a *g)
{
	return &g->dev->dev;
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

static inline u32 set_field(u32 val, u32 mask, u32 field)
{
	return ((val & ~mask) | field);
}

/* invalidate channel lookup tlb */
static inline void gk20a_gr_flush_channel_tlb(struct gr_gk20a *gr)
{
	spin_lock(&gr->ch_tlb_lock);
	memset(gr->chid_tlb, 0,
		sizeof(struct gr_channel_map_tlb_entry) *
		GR_CHANNEL_MAP_TLB_SIZE);
	spin_unlock(&gr->ch_tlb_lock);
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

static inline bool gk20a_gpu_is_virtual(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	return platform->virtual_dev;
}

static inline int support_gk20a_pmu(struct platform_device *dev)
{
	if (IS_ENABLED(CONFIG_GK20A_PMU)) {
		/* gPMU is not supported for vgpu */
		return !gk20a_gpu_is_virtual(dev);
	}

	return 0;
}

void gk20a_create_sysfs(struct platform_device *dev);
void gk20a_remove_sysfs(struct device *dev);

#define GK20A_BAR0_IORESOURCE_MEM 0
#define GK20A_BAR1_IORESOURCE_MEM 1
#define GK20A_SIM_IORESOURCE_MEM 2

void gk20a_busy_noresume(struct platform_device *pdev);
int __must_check gk20a_busy(struct platform_device *pdev);
void gk20a_idle(struct platform_device *pdev);
void gk20a_disable(struct gk20a *g, u32 units);
void gk20a_enable(struct gk20a *g, u32 units);
void gk20a_reset(struct gk20a *g, u32 units);
int gk20a_do_idle(void);
int gk20a_do_unidle(void);
int __gk20a_do_idle(struct platform_device *pdev, bool force_reset);
int __gk20a_do_unidle(struct platform_device *pdev);

const struct firmware *
gk20a_request_firmware(struct gk20a *g, const char *fw_name);

#define NVGPU_GPU_ARCHITECTURE_SHIFT 4

/* constructs unique and compact GPUID from nvgpu_gpu_characteristics
 * arch/impl fields */
#define GK20A_GPUID(arch, impl) ((u32) ((arch) | (impl)))

#define GK20A_GPUID_GK20A \
	GK20A_GPUID(NVGPU_GPU_ARCH_GK100, NVGPU_GPU_IMPL_GK20A)

#define GK20A_GPUID_GM20B \
	GK20A_GPUID(NVGPU_GPU_ARCH_GM200, NVGPU_GPU_IMPL_GM20B)

int gk20a_init_gpu_characteristics(struct gk20a *g);

void gk20a_pbus_isr(struct gk20a *g);

int gk20a_user_init(struct platform_device *dev);
void gk20a_user_deinit(struct platform_device *dev);

extern void gk20a_debug_dump_device(struct platform_device *pdev);

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
#endif /* GK20A_H */
