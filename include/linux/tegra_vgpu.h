/*
 * Tegra GPU Virtualization Interfaces to Server
 *
 * Copyright (c) 2014-2016, NVIDIA Corporation. All rights reserved.
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

#ifndef __TEGRA_VGPU_H
#define __TEGRA_VGPU_H

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include <linux/tegra_vgpu_t18x.h>
#endif

enum {
	TEGRA_VGPU_MODULE_GPU = 0,
};

enum {
	/* Needs to follow last entry in TEGRA_VHOST_QUEUE_* list,
	 * in tegra_vhost.h
	 */
	TEGRA_VGPU_QUEUE_CMD = 3,
	TEGRA_VGPU_QUEUE_INTR
};

enum {
	TEGRA_VGPU_CMD_CONNECT = 0,
	TEGRA_VGPU_CMD_DISCONNECT,
	TEGRA_VGPU_CMD_ABORT,
	TEGRA_VGPU_CMD_CHANNEL_ALLOC_HWCTX,
	TEGRA_VGPU_CMD_CHANNEL_FREE_HWCTX,
	TEGRA_VGPU_CMD_GET_ATTRIBUTE,
	TEGRA_VGPU_CMD_MAP_BAR1,
	TEGRA_VGPU_CMD_AS_ALLOC_SHARE,
	TEGRA_VGPU_CMD_AS_BIND_SHARE,
	TEGRA_VGPU_CMD_AS_FREE_SHARE,
	TEGRA_VGPU_CMD_AS_MAP,
	TEGRA_VGPU_CMD_AS_UNMAP,
	TEGRA_VGPU_CMD_AS_INVALIDATE,
	TEGRA_VGPU_CMD_CHANNEL_BIND,
	TEGRA_VGPU_CMD_CHANNEL_UNBIND,
	TEGRA_VGPU_CMD_CHANNEL_DISABLE,
	TEGRA_VGPU_CMD_CHANNEL_PREEMPT,
	TEGRA_VGPU_CMD_CHANNEL_SETUP_RAMFC,
	TEGRA_VGPU_CMD_CHANNEL_ALLOC_GR_CTX,
	TEGRA_VGPU_CMD_CHANNEL_FREE_GR_CTX,
	TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_CTX,
	TEGRA_VGPU_CMD_CHANNEL_ALLOC_GR_PATCH_CTX,
	TEGRA_VGPU_CMD_CHANNEL_FREE_GR_PATCH_CTX,
	TEGRA_VGPU_CMD_CHANNEL_MAP_GR_GLOBAL_CTX,
	TEGRA_VGPU_CMD_CHANNEL_UNMAP_GR_GLOBAL_CTX,
	TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_GLOBAL_CTX,
	TEGRA_VGPU_CMD_CHANNEL_LOAD_GR_GOLDEN_CTX,
	TEGRA_VGPU_CMD_CHANNEL_BIND_ZCULL,
	TEGRA_VGPU_CMD_CACHE_MAINT,
	TEGRA_VGPU_CMD_SUBMIT_RUNLIST,
	TEGRA_VGPU_CMD_GET_ZCULL_INFO,
	TEGRA_VGPU_CMD_ZBC_SET_TABLE,
	TEGRA_VGPU_CMD_ZBC_QUERY_TABLE,
	TEGRA_VGPU_CMD_AS_MAP_EX,
	TEGRA_VGPU_CMD_CHANNEL_BIND_GR_CTXSW_BUFFERS,
	TEGRA_VGPU_CMD_SET_MMU_DEBUG_MODE,
	TEGRA_VGPU_CMD_SET_SM_DEBUG_MODE,
	TEGRA_VGPU_CMD_REG_OPS,
	TEGRA_VGPU_CMD_CHANNEL_SET_PRIORITY,
	TEGRA_VGPU_CMD_CHANNEL_SET_RUNLIST_INTERLEAVE,
	TEGRA_VGPU_CMD_CHANNEL_SET_TIMESLICE,
	TEGRA_VGPU_CMD_FECS_TRACE_ENABLE,
	TEGRA_VGPU_CMD_FECS_TRACE_DISABLE,
	TEGRA_VGPU_CMD_FECS_TRACE_POLL,
	TEGRA_VGPU_CMD_FECS_TRACE_SET_FILTER,
	TEGRA_VGPU_CMD_CHANNEL_SET_SMPC_CTXSW_MODE,
	TEGRA_VGPU_CMD_CHANNEL_SET_HWPM_CTXSW_MODE,
	TEGRA_VGPU_CMD_CHANNEL_FREE_HWPM_CTX,
};

struct tegra_vgpu_connect_params {
	u32 module;
	u64 handle;
};

struct tegra_vgpu_channel_hwctx_params {
	u32 id;
	u64 pid;
	u64 handle;
};

enum {
	TEGRA_VGPU_ATTRIB_NUM_CHANNELS = 0,
	TEGRA_VGPU_ATTRIB_GOLDEN_CTX_SIZE,
	TEGRA_VGPU_ATTRIB_ZCULL_CTX_SIZE,
	TEGRA_VGPU_ATTRIB_COMPTAG_LINES,
	TEGRA_VGPU_ATTRIB_GPC_COUNT,
	TEGRA_VGPU_ATTRIB_MAX_TPC_PER_GPC_COUNT,
	TEGRA_VGPU_ATTRIB_MAX_TPC_COUNT,
	TEGRA_VGPU_ATTRIB_PMC_BOOT_0,
	TEGRA_VGPU_ATTRIB_L2_SIZE,
	TEGRA_VGPU_ATTRIB_GPC0_TPC0_SM_ARCH,
	TEGRA_VGPU_ATTRIB_NUM_FBPS,
	TEGRA_VGPU_ATTRIB_FBP_EN_MASK,
	TEGRA_VGPU_ATTRIB_MAX_LTC_PER_FBP,
	TEGRA_VGPU_ATTRIB_MAX_LTS_PER_LTC,
	TEGRA_VGPU_ATTRIB_GPC0_TPC_MASK,
	TEGRA_VGPU_ATTRIB_CACHELINE_SIZE,
	TEGRA_VGPU_ATTRIB_COMPTAGS_PER_CACHELINE,
	TEGRA_VGPU_ATTRIB_SLICES_PER_LTC,
	TEGRA_VGPU_ATTRIB_LTC_COUNT,
	TEGRA_VGPU_ATTRIB_TPC_COUNT
};

struct tegra_vgpu_attrib_params {
	u32 attrib;
	u32 value;
};

struct tegra_vgpu_as_share_params {
	u64 size;
	u64 handle;
	u32 big_page_size;
};

struct tegra_vgpu_as_bind_share_params {
	u64 as_handle;
	u64 chan_handle;
};

enum {
	TEGRA_VGPU_MAP_PROT_NONE = 0,
	TEGRA_VGPU_MAP_PROT_READ_ONLY,
	TEGRA_VGPU_MAP_PROT_WRITE_ONLY
};

struct tegra_vgpu_as_map_params {
	u64 handle;
	u64 addr;
	u64 gpu_va;
	u64 size;
	u8 pgsz_idx;
	u8 iova;
	u8 kind;
	u8 cacheable;
	u8 clear_ctags;
	u8 prot;
	u32 ctag_offset;
};

struct tegra_vgpu_as_map_ex_params {
	u64 handle;
	u64 gpu_va;
	u64 size;
	u32 mem_desc_count;
	u8 pgsz_idx;
	u8 iova;
	u8 kind;
	u8 cacheable;
	u8 clear_ctags;
	u8 prot;
	u32 ctag_offset;
};

struct tegra_vgpu_mem_desc {
	u64 addr;
	u64 length;
};

struct tegra_vgpu_as_invalidate_params {
	u64 handle;
};

struct tegra_vgpu_channel_config_params {
	u64 handle;
};

struct tegra_vgpu_ramfc_params {
	u64 handle;
	u64 gpfifo_va;
	u32 num_entries;
	u64 userd_addr;
	u8 iova;
};

struct tegra_vgpu_gr_ctx_params {
	u64 handle;
	u64 gr_ctx_va;
	u64 patch_ctx_va;
	u64 cb_va;
	u64 attr_va;
	u64 page_pool_va;
	u64 priv_access_map_va;
	u32 class_num;
};

struct tegra_vgpu_zcull_bind_params {
	u64 handle;
	u64 zcull_va;
	u32 mode;
};

enum {
	TEGRA_VGPU_L2_MAINT_FLUSH = 0,
	TEGRA_VGPU_L2_MAINT_INV,
	TEGRA_VGPU_L2_MAINT_FLUSH_INV,
	TEGRA_VGPU_FB_FLUSH
};

struct tegra_vgpu_cache_maint_params {
	u8 op;
};

struct tegra_vgpu_runlist_params {
	u8 runlist_id;
	u32 num_entries;
};

struct tegra_vgpu_golden_ctx_params {
	u32 size;
};

struct tegra_vgpu_zcull_info_params {
	u32 width_align_pixels;
	u32 height_align_pixels;
	u32 pixel_squares_by_aliquots;
	u32 aliquot_total;
	u32 region_byte_multiplier;
	u32 region_header_size;
	u32 subregion_header_size;
	u32 subregion_width_align_pixels;
	u32 subregion_height_align_pixels;
	u32 subregion_count;
};

#define TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE		4
#define TEGRA_VGPU_ZBC_TYPE_INVALID		0
#define TEGRA_VGPU_ZBC_TYPE_COLOR		1
#define TEGRA_VGPU_ZBC_TYPE_DEPTH		2

struct tegra_vgpu_zbc_set_table_params {
	u32 color_ds[TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE];
	u32 color_l2[TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE];
	u32 depth;
	u32 format;
	u32 type;     /* color or depth */
};

struct tegra_vgpu_zbc_query_table_params {
	u32 color_ds[TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE];
	u32 color_l2[TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE];
	u32 depth;
	u32 ref_cnt;
	u32 format;
	u32 type;             /* color or depth */
	u32 index_size;       /* [out] size, [in] index */
};

#define TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_MAX 4

struct tegra_vgpu_gr_bind_ctxsw_buffers_params {
	u64 handle;
	u64 gpu_va[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_MAX];
	u64 size[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_MAX];
	u32 mode;
};

struct tegra_vgpu_mmu_debug_mode {
	u32 enable;
};

struct tegra_vgpu_sm_debug_mode {
	u64 handle;
	u64 sms;
	u32 enable;
};

struct tegra_vgpu_reg_op {
	u8 op;
	u8 type;
	u8 status;
	u8 quad;
	u32 group_mask;
	u32 sub_group_mask;
	u32 offset;
	u32 value_lo;
	u32 value_hi;
	u32 and_n_mask_lo;
	u32 and_n_mask_hi;
};

struct tegra_vgpu_reg_ops_params {
	u64 handle;
	u64 num_ops;
	u32 is_profiler;
};

struct tegra_vgpu_channel_priority_params {
	u64 handle;
	u32 priority;
};

/* level follows nvgpu.h definitions */
struct tegra_vgpu_channel_runlist_interleave_params {
	u64 handle;
	u32 level;
};

struct tegra_vgpu_channel_timeslice_params {
	u64 handle;
	u32 timeslice_us;
};

#define TEGRA_VGPU_FECS_TRACE_FILTER_SIZE 256
struct tegra_vgpu_fecs_trace_filter {
	u64 tag_bits[(TEGRA_VGPU_FECS_TRACE_FILTER_SIZE + 63) / 64];
};

enum {
	TEGRA_VGPU_CTXSW_MODE_NO_CTXSW = 0,
	TEGRA_VGPU_CTXSW_MODE_CTXSW,
};

struct tegra_vgpu_channel_set_ctxsw_mode {
	u64 handle;
	u32 mode;
};

struct tegra_vgpu_channel_free_hwpm_ctx {
	u64 handle;
};

struct tegra_vgpu_cmd_msg {
	u32 cmd;
	int ret;
	u64 handle;
	union {
		struct tegra_vgpu_connect_params connect;
		struct tegra_vgpu_channel_hwctx_params channel_hwctx;
		struct tegra_vgpu_attrib_params attrib;
		struct tegra_vgpu_as_share_params as_share;
		struct tegra_vgpu_as_bind_share_params as_bind_share;
		struct tegra_vgpu_as_map_params as_map;
		struct tegra_vgpu_as_map_ex_params as_map_ex;
		struct tegra_vgpu_as_invalidate_params as_invalidate;
		struct tegra_vgpu_channel_config_params channel_config;
		struct tegra_vgpu_ramfc_params ramfc;
		struct tegra_vgpu_gr_ctx_params gr_ctx;
		struct tegra_vgpu_zcull_bind_params zcull_bind;
		struct tegra_vgpu_cache_maint_params cache_maint;
		struct tegra_vgpu_runlist_params runlist;
		struct tegra_vgpu_golden_ctx_params golden_ctx;
		struct tegra_vgpu_zcull_info_params zcull_info;
		struct tegra_vgpu_zbc_set_table_params zbc_set_table;
		struct tegra_vgpu_zbc_query_table_params zbc_query_table;
		struct tegra_vgpu_gr_bind_ctxsw_buffers_params gr_bind_ctxsw_buffers;
		struct tegra_vgpu_mmu_debug_mode mmu_debug_mode;
		struct tegra_vgpu_sm_debug_mode sm_debug_mode;
		struct tegra_vgpu_reg_ops_params reg_ops;
		struct tegra_vgpu_channel_priority_params channel_priority;
		struct tegra_vgpu_channel_runlist_interleave_params channel_interleave;
		struct tegra_vgpu_channel_timeslice_params channel_timeslice;
		struct tegra_vgpu_fecs_trace_filter fecs_trace_filter;
		struct tegra_vgpu_channel_set_ctxsw_mode set_ctxsw_mode;
		struct tegra_vgpu_channel_free_hwpm_ctx free_hwpm_ctx;
		char padding[192];
	} params;
};

enum {
	TEGRA_VGPU_GR_INTR_NOTIFY = 0,
	TEGRA_VGPU_GR_INTR_SEMAPHORE_TIMEOUT,
	TEGRA_VGPU_GR_INTR_ILLEGAL_NOTIFY,
	TEGRA_VGPU_GR_INTR_ILLEGAL_METHOD,
	TEGRA_VGPU_GR_INTR_ILLEGAL_CLASS,
	TEGRA_VGPU_GR_INTR_FECS_ERROR,
	TEGRA_VGPU_GR_INTR_CLASS_ERROR,
	TEGRA_VGPU_GR_INTR_FIRMWARE_METHOD,
	TEGRA_VGPU_GR_INTR_EXCEPTION,
	TEGRA_VGPU_GR_INTR_SEMAPHORE,
	TEGRA_VGPU_FIFO_INTR_PBDMA,
	TEGRA_VGPU_FIFO_INTR_CTXSW_TIMEOUT,
	TEGRA_VGPU_FIFO_INTR_MMU_FAULT,
	TEGRA_VGPU_GR_NONSTALL_INTR_SEMAPHORE,
	TEGRA_VGPU_FIFO_NONSTALL_INTR_CHANNEL,
	TEGRA_VGPU_CE2_NONSTALL_INTR_NONBLOCKPIPE,
	TEGRA_VGPU_GR_INTR_SM_EXCEPTION
};

struct tegra_vgpu_gr_intr_info {
	u32 type;
	u32 chid;
};

struct tegra_vgpu_gr_nonstall_intr_info {
	u32 type;
};

struct tegra_vgpu_fifo_intr_info {
	u32 type;
	u32 chid;
};

struct tegra_vgpu_fifo_nonstall_intr_info {
	u32 type;
};

struct tegra_vgpu_ce2_nonstall_intr_info {
	u32 type;
};

enum {
	TEGRA_VGPU_FECS_TRACE_DATA_UPDATE = 0
};

struct tegra_vgpu_fecs_trace_event_info {
	u32 type;
};

enum {

	TEGRA_VGPU_INTR_GR = 0,
	TEGRA_VGPU_INTR_FIFO,
	TEGRA_VGPU_INTR_CE2,
	TEGRA_VGPU_NONSTALL_INTR_GR,
	TEGRA_VGPU_NONSTALL_INTR_FIFO,
	TEGRA_VGPU_NONSTALL_INTR_CE2
};

enum {
	TEGRA_VGPU_EVENT_INTR = 0,
	TEGRA_VGPU_EVENT_ABORT,
	TEGRA_VGPU_EVENT_FECS_TRACE
};

struct tegra_vgpu_intr_msg {
	unsigned int event;
	u32 unit;
	union {
		struct tegra_vgpu_gr_intr_info gr_intr;
		struct tegra_vgpu_gr_nonstall_intr_info gr_nonstall_intr;
		struct tegra_vgpu_fifo_intr_info fifo_intr;
		struct tegra_vgpu_fifo_nonstall_intr_info fifo_nonstall_intr;
		struct tegra_vgpu_ce2_nonstall_intr_info ce2_nonstall_intr;
		struct tegra_vgpu_fecs_trace_event_info fecs_trace;
		char padding[32];
	} info;
};

#define TEGRA_VGPU_QUEUE_SIZES	\
	512,			\
	sizeof(struct tegra_vgpu_intr_msg)

#endif
