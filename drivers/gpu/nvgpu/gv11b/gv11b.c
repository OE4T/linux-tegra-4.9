/*
 * GV11B Graphics
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"
#include "gp10b/gp10b.h"

#include "gv11b/gv11b.h"
#include <nvgpu/hw/gv11b/hw_fuse_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gr_gv11b.h>

void gv11b_detect_ecc_enabled_units(struct gk20a *g)
{
	u32 opt_ecc_en = gk20a_readl(g, fuse_opt_ecc_en_r());
	u32 opt_feature_fuses_override_disable =
			gk20a_readl(g,
				fuse_opt_feature_fuses_override_disable_r());
	u32 fecs_feature_override_ecc =
			gk20a_readl(g,
				gr_fecs_feature_override_ecc_r());

	if (opt_feature_fuses_override_disable) {
		if (opt_ecc_en) {
			__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_LRF, true);
			__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_DATA, true);
			__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_TAG, true);
			__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_ICACHE, true);
			__nvgpu_set_enabled(g, NVGPU_ECC_ENABLED_LTC, true);
			__nvgpu_set_enabled(g, NVGPU_ECC_ENABLED_SM_CBU, true);
		}
	} else {
		/* SM LRF */
		if (gr_fecs_feature_override_ecc_sm_lrf_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_sm_lrf_v(
						fecs_feature_override_ecc)) {
				__nvgpu_set_enabled(g,
						NVGPU_ECC_ENABLED_SM_LRF, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
						NVGPU_ECC_ENABLED_SM_LRF, true);
			}
		}
		/* SM L1 DATA*/
		if (gr_fecs_feature_override_ecc_sm_l1_data_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_sm_l1_data_v(
						fecs_feature_override_ecc)) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_DATA, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_DATA, true);
			}
		}
		/* SM L1 TAG*/
		if (gr_fecs_feature_override_ecc_sm_l1_tag_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_sm_l1_tag_v(
						fecs_feature_override_ecc)) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_TAG, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_TAG, true);
			}
		}
		/* SM ICACHE*/
		if (gr_fecs_feature_override_ecc_1_sm_l0_icache_override_v(
						fecs_feature_override_ecc) &&
			gr_fecs_feature_override_ecc_1_sm_l1_icache_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_1_sm_l0_icache_v(
						fecs_feature_override_ecc) &&
				gr_fecs_feature_override_ecc_1_sm_l1_icache_v(
						fecs_feature_override_ecc)) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_ICACHE, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_ICACHE, true);
			}
		}
		/* LTC */
		if (gr_fecs_feature_override_ecc_ltc_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_ltc_v(
					fecs_feature_override_ecc)) {
				__nvgpu_set_enabled(g,
						NVGPU_ECC_ENABLED_LTC, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
						NVGPU_ECC_ENABLED_LTC, true);
			}
		}
		/* SM CBU */
		if (gr_fecs_feature_override_ecc_sm_cbu_override_v(
						fecs_feature_override_ecc)) {
			if (gr_fecs_feature_override_ecc_sm_cbu_v(
						fecs_feature_override_ecc)) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_CBU, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_CBU, true);
			}
		}
	}
}



int gv11b_init_gpu_characteristics(struct gk20a *g)
{
	gk20a_init_gpu_characteristics(g);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_TSG_SUBCONTEXTS, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_IO_COHERENCE, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_SCG, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_SYNCPOINT_ADDRESS, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_USER_SYNCPOINT, true);

	return 0;
}
