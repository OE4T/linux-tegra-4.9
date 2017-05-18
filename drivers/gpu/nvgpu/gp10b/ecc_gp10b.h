/*
 * GP10B ECC
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVGPU_ECC_GP10B_H_
#define _NVGPU_ECC_GP10B_H_

#include <linux/version.h>

struct ecc_gr_t18x {
	struct gk20a_ecc_stat sm_lrf_single_err_count;
	struct gk20a_ecc_stat sm_lrf_double_err_count;

	struct gk20a_ecc_stat sm_shm_sec_count;
	struct gk20a_ecc_stat sm_shm_sed_count;
	struct gk20a_ecc_stat sm_shm_ded_count;

	struct gk20a_ecc_stat tex_total_sec_pipe0_count;
	struct gk20a_ecc_stat tex_total_ded_pipe0_count;
	struct gk20a_ecc_stat tex_unique_sec_pipe0_count;
	struct gk20a_ecc_stat tex_unique_ded_pipe0_count;
	struct gk20a_ecc_stat tex_total_sec_pipe1_count;
	struct gk20a_ecc_stat tex_total_ded_pipe1_count;
	struct gk20a_ecc_stat tex_unique_sec_pipe1_count;
	struct gk20a_ecc_stat tex_unique_ded_pipe1_count;

	struct gk20a_ecc_stat l2_sec_count;
	struct gk20a_ecc_stat l2_ded_count;
};
#endif
