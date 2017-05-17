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

struct gr_t19x {
	struct {
		struct gr_gp10b_ecc_stat sm_l1_tag_corrected_err_count;
		struct gr_gp10b_ecc_stat sm_l1_tag_uncorrected_err_count;
		struct gr_gp10b_ecc_stat sm_cbu_corrected_err_count;
		struct gr_gp10b_ecc_stat sm_cbu_uncorrected_err_count;
		struct gr_gp10b_ecc_stat sm_l1_data_corrected_err_count;
		struct gr_gp10b_ecc_stat sm_l1_data_uncorrected_err_count;
		struct gr_gp10b_ecc_stat sm_icache_corrected_err_count;
		struct gr_gp10b_ecc_stat sm_icache_uncorrected_err_count;
	} ecc_stats;
};

#define NVC397_SET_SHADER_EXCEPTIONS		0x1528
#define NVC397_SET_CIRCULAR_BUFFER_SIZE 	0x1280
#define NVC397_SET_ALPHA_CIRCULAR_BUFFER_SIZE 	0x02dc
#define NVC397_SET_GO_IDLE_TIMEOUT 		0x022c

#define NVA297_SET_SHADER_EXCEPTIONS_ENABLE_FALSE 0

void gv11b_init_gr(struct gpu_ops *ops);
int gr_gv11b_alloc_buffer(struct vm_gk20a *vm, size_t size,
                        struct nvgpu_mem *mem);
/*zcull*/
void gr_gv11b_program_zcull_mapping(struct gk20a *g, u32 zcull_num_entries,
					u32 *zcull_map_tiles);
void gr_gv11b_create_sysfs(struct device *dev);
#endif
