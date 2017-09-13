/*
 * GP10B Graphics
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"

#include "gp10b.h"

#include <nvgpu/hw/gp10b/hw_fuse_gp10b.h>
#include <nvgpu/hw/gp10b/hw_gr_gp10b.h>

static u64 gp10b_detect_ecc_enabled_units(struct gk20a *g)
{
	u64 ecc_enabled_units = 0;
	u32 opt_ecc_en = gk20a_readl(g, fuse_opt_ecc_en_r());
	u32 opt_feature_fuses_override_disable =
			gk20a_readl(g,
				fuse_opt_feature_fuses_override_disable_r());
	u32 fecs_feature_override_ecc =
				gk20a_readl(g,
					gr_fecs_feature_override_ecc_r());

	if (opt_feature_fuses_override_disable) {
		if (opt_ecc_en)
			ecc_enabled_units = NVGPU_GPU_FLAGS_ALL_ECC_ENABLED;
		else
			ecc_enabled_units = 0;
	} else {
		/* SM LRF */
		if (gr_fecs_feature_override_ecc_sm_lrf_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_sm_lrf_v(
						fecs_feature_override_ecc)) {
				ecc_enabled_units |=
					NVGPU_GPU_FLAGS_ECC_ENABLED_SM_LRF;
			}
		} else {
			if (opt_ecc_en) {
				ecc_enabled_units |=
					NVGPU_GPU_FLAGS_ECC_ENABLED_SM_LRF;
			}
		}

		/* SM SHM */
		if (gr_fecs_feature_override_ecc_sm_shm_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_sm_shm_v(
						fecs_feature_override_ecc)) {
				ecc_enabled_units |=
					NVGPU_GPU_FLAGS_ECC_ENABLED_SM_SHM;
			}
		} else {
			if (opt_ecc_en) {
				ecc_enabled_units |=
					NVGPU_GPU_FLAGS_ECC_ENABLED_SM_SHM;
			}
		}

		/* TEX */
		if (gr_fecs_feature_override_ecc_tex_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_tex_v(
						fecs_feature_override_ecc)) {
				ecc_enabled_units |=
					NVGPU_GPU_FLAGS_ECC_ENABLED_TEX;
			}
		} else {
			if (opt_ecc_en) {
				ecc_enabled_units |=
					NVGPU_GPU_FLAGS_ECC_ENABLED_TEX;
			}
		}

		/* LTC */
		if (gr_fecs_feature_override_ecc_ltc_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_ltc_v(
						fecs_feature_override_ecc)) {
				ecc_enabled_units |=
					NVGPU_GPU_FLAGS_ECC_ENABLED_LTC;
			}
		} else {
			if (opt_ecc_en) {
				ecc_enabled_units |=
					NVGPU_GPU_FLAGS_ECC_ENABLED_LTC;
			}
		}
	}

	return ecc_enabled_units;
}

int gp10b_init_gpu_characteristics(struct gk20a *g)
{
	gk20a_init_gpu_characteristics(g);
	g->gpu_characteristics.flags |= gp10b_detect_ecc_enabled_units(g);
	g->gpu_characteristics.flags |=
		NVGPU_GPU_FLAGS_SUPPORT_RESCHEDULE_RUNLIST;
	return 0;
}
