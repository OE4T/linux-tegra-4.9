/*
 * Tegra GPU Virtualization Interfaces to Server
 *
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
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

enum {
	TEGRA_VGPU_MODULE_GPU = 0,
};

enum {
	TEGRA_VGPU_QUEUE_CMD = 2,
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
	TEGRA_VGPU_CMD_GET_ZCULL_INFO
};

struct tegra_vgpu_connect_params {
	u32 module;
	u64 handle;
};

struct tegra_vgpu_channel_hwctx_params {
	u32 id;
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
	TEGRA_VGPU_ATTRIB_GPC0_TPC0_SM_ARCH
};

struct tegra_vgpu_attrib_params {
	u32 attrib;
	u32 value;
};

struct tegra_vgpu_as_share_params {
	u64 size;
	u64 handle;
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
		struct tegra_vgpu_as_invalidate_params as_invalidate;
		struct tegra_vgpu_channel_config_params channel_config;
		struct tegra_vgpu_ramfc_params ramfc;
		struct tegra_vgpu_gr_ctx_params gr_ctx;
		struct tegra_vgpu_zcull_bind_params zcull_bind;
		struct tegra_vgpu_cache_maint_params cache_maint;
		struct tegra_vgpu_runlist_params runlist;
		struct tegra_vgpu_golden_ctx_params golden_ctx;
		struct tegra_vgpu_zcull_info_params zcull_info;
	} params;
};

enum {
	TEGRA_VGPU_GR_INTR_NOTIFY = 0
};

struct tegra_vgpu_gr_intr_info {
	u32 type;
	u32 chid;
};

enum {
	TEGRA_VGPU_INTR_GR = 0
};

enum {
	TEGRA_VGPU_EVENT_INTR = 0,
	TEGRA_VGPU_EVENT_ABORT
};

struct tegra_vgpu_intr_msg {
	unsigned int event;
	u32 unit;
	union {
		struct tegra_vgpu_gr_intr_info gr_intr;
	} info;
};

#define TEGRA_VGPU_QUEUE_SIZES	\
	512,			\
	sizeof(struct tegra_vgpu_intr_msg)

#endif
