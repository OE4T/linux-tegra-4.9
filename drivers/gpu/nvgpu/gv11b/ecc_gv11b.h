/*
 * GV11B GPU ECC
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

#ifndef _NVGPU_ECC_GV11B_H_
#define _NVGPU_ECC_GV11B_H_

struct ecc_gr_t19x {
	struct gk20a_ecc_stat sm_l1_tag_corrected_err_count;
	struct gk20a_ecc_stat sm_l1_tag_uncorrected_err_count;
	struct gk20a_ecc_stat sm_cbu_corrected_err_count;
	struct gk20a_ecc_stat sm_cbu_uncorrected_err_count;
	struct gk20a_ecc_stat sm_l1_data_corrected_err_count;
	struct gk20a_ecc_stat sm_l1_data_uncorrected_err_count;
	struct gk20a_ecc_stat sm_icache_corrected_err_count;
	struct gk20a_ecc_stat sm_icache_uncorrected_err_count;
	struct gk20a_ecc_stat gcc_l15_corrected_err_count;
	struct gk20a_ecc_stat gcc_l15_uncorrected_err_count;
	struct gk20a_ecc_stat fecs_corrected_err_count;
	struct gk20a_ecc_stat fecs_uncorrected_err_count;
	struct gk20a_ecc_stat gpccs_corrected_err_count;
	struct gk20a_ecc_stat gpccs_uncorrected_err_count;
};

#endif
