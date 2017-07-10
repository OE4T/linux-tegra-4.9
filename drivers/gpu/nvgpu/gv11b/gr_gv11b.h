/*
 * GV11B GPU GR
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
 */

#ifndef _NVGPU_GR_GV11B_H_
#define _NVGPU_GR_GV11B_H_

#define GV11B_ZBC_TYPE_STENCIL            T19X_ZBC
#define ZBC_STENCIL_CLEAR_FMT_INVAILD     0
#define ZBC_STENCIL_CLEAR_FMT_U8          1

struct zbc_s_table {
	u32 stencil;
	u32 format;
	u32 ref_cnt;
};

struct gpu_ops;

enum {
	VOLTA_CHANNEL_GPFIFO_A  = 0xC36F,
	VOLTA_A                 = 0xC397,
	VOLTA_COMPUTE_A         = 0xC3C0,
	VOLTA_DMA_COPY_A        = 0xC3B5,
};

#define NVC397_SET_SHADER_EXCEPTIONS		0x1528
#define NVC397_SET_CIRCULAR_BUFFER_SIZE 	0x1280
#define NVC397_SET_ALPHA_CIRCULAR_BUFFER_SIZE 	0x02dc
#define NVC397_SET_GO_IDLE_TIMEOUT 		0x022c
#define NVC397_SET_TEX_IN_DBG			0x10bc
#define NVC397_SET_SKEDCHECK			0x10c0
#define NVC397_SET_BES_CROP_DEBUG3		0x10c4

#define NVC397_SET_TEX_IN_DBG_TSL1_RVCH_INVALIDATE		0x1
#define NVC397_SET_TEX_IN_DBG_SM_L1TAG_CTRL_CACHE_SURFACE_LD	0x2
#define NVC397_SET_TEX_IN_DBG_SM_L1TAG_CTRL_CACHE_SURFACE_ST	0x4

#define NVC397_SET_SKEDCHECK_18_MASK				0x3
#define NVC397_SET_SKEDCHECK_18_DEFAULT				0x0
#define NVC397_SET_SKEDCHECK_18_DISABLE				0x1
#define NVC397_SET_SKEDCHECK_18_ENABLE				0x2

#define NVC3C0_SET_SKEDCHECK			0x23c

#define NVA297_SET_SHADER_EXCEPTIONS_ENABLE_FALSE 0

void gv11b_init_gr(struct gpu_ops *ops);
int gr_gv11b_alloc_buffer(struct vm_gk20a *vm, size_t size,
                        struct nvgpu_mem *mem);
/*zcull*/
void gr_gv11b_program_zcull_mapping(struct gk20a *g, u32 zcull_num_entries,
					u32 *zcull_map_tiles);
void gr_gv11b_create_sysfs(struct device *dev);
#endif
