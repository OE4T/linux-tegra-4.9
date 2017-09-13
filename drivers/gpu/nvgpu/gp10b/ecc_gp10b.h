/*
 * GP10B ECC
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVGPU_ECC_GP10B_H_
#define _NVGPU_ECC_GP10B_H_

struct gk20a_ecc_stat;

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
