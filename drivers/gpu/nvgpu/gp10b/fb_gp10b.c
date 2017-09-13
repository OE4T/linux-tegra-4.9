/*
 * GP10B FB
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include "gm20b/fb_gm20b.h"
#include "gk20a/kind_gk20a.h"
#include "fb_gp10b.h"

#include <nvgpu/hw/gp10b/hw_gmmu_gp10b.h>

noinline_for_stack void gp10b_init_uncompressed_kind_map(void)
{
	int i;

	for (i = 0; i < 256; i++)
		gk20a_uc_kind_map[i] = gmmu_pte_kind_invalid_v();

	/* From gp10b */
	gk20a_uc_kind_map[gmmu_pte_kind_z16_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms2_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms4_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms8_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms16_2cz_v()] =
		gmmu_pte_kind_z16_v();

	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms4_4cbra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms4_4cbra_v()] =
		gmmu_pte_kind_generic_16bx2_v();

	/* From gm20b */
	gk20a_uc_kind_map[gmmu_pte_kind_s8_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_s8_2s_v()] =
		gmmu_pte_kind_s8_v();

	/* From gk20a */
	gk20a_uc_kind_map[gmmu_pte_kind_z16_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms2_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms4_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms8_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_2z_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms2_2z_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms4_2z_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z16_ms8_2z_v()] =
		gmmu_pte_kind_z16_v();

	gk20a_uc_kind_map[gmmu_pte_kind_s8z24_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_s8z24_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_s8z24_ms2_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_s8z24_ms4_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_s8z24_ms8_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_s8z24_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_s8z24_ms2_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_s8z24_ms4_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_s8z24_ms8_2cs_v()] =
		gmmu_pte_kind_s8z24_v();

	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms4_vc4_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms4_vc4_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms4_vc4_2czv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms4_vc4_2zv_v()] =
		gmmu_pte_kind_v8z24_ms4_vc4_v();

	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms8_vc8_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms8_vc8_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms8_vc8_2czv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms8_vc8_2zv_v()] =
		gmmu_pte_kind_v8z24_ms8_vc8_v();

	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms4_vc12_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms4_vc12_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms4_vc12_2czv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms4_vc12_2zv_v()] =
		gmmu_pte_kind_v8z24_ms4_vc12_v();

	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms8_vc24_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms8_vc24_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms8_vc24_2czv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_v8z24_ms8_vc24_2zv_v()] =
		gmmu_pte_kind_v8z24_ms8_vc24_v();

	gk20a_uc_kind_map[gmmu_pte_kind_z24s8_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24s8_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24s8_ms2_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24s8_ms4_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24s8_ms8_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24s8_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24s8_ms2_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24s8_ms4_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24s8_ms8_2cz_v()] =
		gmmu_pte_kind_z24s8_v();

	gk20a_uc_kind_map[gmmu_pte_kind_zf32_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_ms2_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_ms4_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_ms8_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_ms2_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_ms4_2cz_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_ms8_2cz_v()] =
		gmmu_pte_kind_zf32_v();

	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_2cszv_v()] =
		gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_v();

	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_2cszv_v()] =
		gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_v();

	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_2cszv_v()] =
		gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_v();

	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_2cszv_v()] =
		gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_v();

	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_2cszv_v()] =
		gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_v();

	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_2cszv_v()] =
		gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_v();

	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_2cszv_v()] =
		gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_v();

	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_2cszv_v()] =
		gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_v();

	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x24s8_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x24s8_2cszv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x24s8_ms2_2cszv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x24s8_ms4_2cszv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x24s8_ms8_2cszv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x24s8_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x24s8_ms2_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x24s8_ms4_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_zf32_x24s8_ms8_2cs_v()] =
		gmmu_pte_kind_zf32_x24s8_v();

	gk20a_uc_kind_map[gmmu_pte_kind_c32_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_2cba_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_2cra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_2bra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms2_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms2_2cra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms4_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms4_2cbr_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms4_2cba_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms4_2cra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms4_2bra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms8_ms16_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms8_ms16_2cra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_2cbr_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_2cba_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_2cra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_2bra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms2_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms2_2cra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms4_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms4_2cbr_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms4_2cba_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms4_2cra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms4_2bra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms8_ms16_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms8_ms16_2cra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c128_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c128_2cr_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c128_ms2_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c128_ms2_2cr_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c128_ms4_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c128_ms4_2cr_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c128_ms8_ms16_2c_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c128_ms8_ms16_2cr_v()] =
		gmmu_pte_kind_generic_16bx2_v();

	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms4_vc4_2czv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms4_vc4_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms4_vc4_2zv_v()] =
		gmmu_pte_kind_z24v8_ms4_vc4_v();

	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms4_vc12_2czv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms4_vc12_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms4_vc12_2zv_v()] =
		gmmu_pte_kind_z24v8_ms4_vc12_v();

	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms8_vc8_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms8_vc8_2czv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms8_vc8_2zv_v()] =
		gmmu_pte_kind_z24v8_ms8_vc8_v();

	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms8_vc24_2cs_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms8_vc24_2czv_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_z24v8_ms8_vc24_2zv_v()] =
		gmmu_pte_kind_z24v8_ms8_vc24_v();

	gk20a_uc_kind_map[gmmu_pte_kind_x8c24_v()] =
		gmmu_pte_kind_x8c24_v();
}

static noinline_for_stack bool gp10b_kind_supported(u8 k)
{
	return /* From gp10b */
		(k >= gmmu_pte_kind_z16_2cz_v() &&
			k <= gmmu_pte_kind_z16_ms8_2cz_v())
		|| k == gmmu_pte_kind_z16_ms16_2cz_v()
		|| k == gmmu_pte_kind_c32_ms4_4cbra_v()
		|| k == gmmu_pte_kind_c64_ms4_4cbra_v()
		/* From gm20b */
		|| (k == gmmu_pte_kind_smsked_message_v())
		|| (k >= gmmu_pte_kind_s8_v() &&
		    k <= gmmu_pte_kind_s8_2s_v())
		/* From gk20a */
		|| gk20a_kind_work_creation(k)
		|| (k == gmmu_pte_kind_invalid_v())
		|| (k == gmmu_pte_kind_pitch_v())
		|| (k >= gmmu_pte_kind_z16_v() &&
			k <= gmmu_pte_kind_z16_ms8_2c_v())
		|| (k >= gmmu_pte_kind_z16_2z_v() &&
			k <= gmmu_pte_kind_z16_ms8_2z_v())
		|| (k == gmmu_pte_kind_s8z24_v())
		|| (k >= gmmu_pte_kind_s8z24_2cz_v() &&
			k <= gmmu_pte_kind_s8z24_ms8_2cz_v())
		|| (k >= gmmu_pte_kind_v8z24_ms4_vc12_v() &&
			k <= gmmu_pte_kind_v8z24_ms8_vc24_v())
		|| (k >= gmmu_pte_kind_v8z24_ms4_vc12_2czv_v() &&
			k <= gmmu_pte_kind_v8z24_ms8_vc24_2zv_v())
		|| (k == gmmu_pte_kind_z24s8_v())
		|| (k >= gmmu_pte_kind_z24s8_2cz_v() &&
			k <= gmmu_pte_kind_z24s8_ms8_2cz_v())
		|| (k == gmmu_pte_kind_zf32_v())
		|| (k >= gmmu_pte_kind_zf32_2cz_v() &&
			k <= gmmu_pte_kind_zf32_ms8_2cz_v())
		|| (k >= gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_v() &&
			k <= gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_v())
		|| (k >= gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_2cszv_v() &&
			k <= gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_v())
		|| (k >= gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_2cszv_v() &&
			k <= gmmu_pte_kind_zf32_x24s8_v())
		|| (k >= gmmu_pte_kind_zf32_x24s8_2cszv_v() &&
			k <= gmmu_pte_kind_zf32_x24s8_ms8_2cszv_v())
		|| (k == gmmu_pte_kind_generic_16bx2_v())
		|| (k == gmmu_pte_kind_c32_2c_v())
		|| (k == gmmu_pte_kind_c32_2cra_v())
		|| (k == gmmu_pte_kind_c32_ms2_2c_v())
		|| (k == gmmu_pte_kind_c32_ms2_2cra_v())
		|| (k >= gmmu_pte_kind_c32_ms4_2c_v() &&
			k <= gmmu_pte_kind_c32_ms4_2cbr_v())
		|| (k >= gmmu_pte_kind_c32_ms4_2cra_v() &&
			k <= gmmu_pte_kind_c64_2c_v())
		|| (k == gmmu_pte_kind_c64_2cra_v())
		|| (k == gmmu_pte_kind_c64_ms2_2c_v())
		|| (k == gmmu_pte_kind_c64_ms2_2cra_v())
		|| (k >= gmmu_pte_kind_c64_ms4_2c_v() &&
			k <= gmmu_pte_kind_c64_ms4_2cbr_v())
		|| (k >= gmmu_pte_kind_c64_ms4_2cra_v() &&
			k <= gmmu_pte_kind_c128_ms8_ms16_2cr_v())
		|| (k == gmmu_pte_kind_pitch_no_swizzle_v());
}

static noinline_for_stack bool gp10b_kind_z(u8 k)
{
	return /* From gp10b */
		(k >= gmmu_pte_kind_z16_2cz_v() &&
			k <= gmmu_pte_kind_z16_ms8_2cz_v())
		|| (k == gmmu_pte_kind_z16_ms16_2cz_v())
		/* From gm20b */
		|| (k >= gmmu_pte_kind_s8_v() &&
			k <= gmmu_pte_kind_s8_2s_v())
		/* From gk20a */
		|| (k >= gmmu_pte_kind_z16_v() &&
			k <= gmmu_pte_kind_v8z24_ms8_vc24_v())
		|| (k >= gmmu_pte_kind_v8z24_ms4_vc12_1zv_v() &&
			k <= gmmu_pte_kind_v8z24_ms8_vc24_2cs_v())
		|| (k >= gmmu_pte_kind_v8z24_ms4_vc12_2czv_v() &&
			k <= gmmu_pte_kind_z24v8_ms8_vc24_v())
		|| (k >= gmmu_pte_kind_z24v8_ms4_vc12_1zv_v() &&
			k <= gmmu_pte_kind_z24v8_ms8_vc24_2cs_v())
		|| (k >= gmmu_pte_kind_z24v8_ms4_vc12_2czv_v() &&
			k <= gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_1cs_v())
		|| (k >= gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_1zv_v() &&
			k <= gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_1cs_v())
		|| (k >= gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_1zv_v() &&
			k <= gmmu_pte_kind_zf32_x24s8_ms16_1cs_v());
}

static noinline_for_stack bool gp10b_kind_c(u8 k)
{
	return gk20a_kind_work_creation(k)
		|| (k == gmmu_pte_kind_pitch_v())
		|| (k == gmmu_pte_kind_generic_16bx2_v())
		|| (k >= gmmu_pte_kind_c32_2c_v() &&
			k <= gmmu_pte_kind_c32_ms2_2cbr_v())
		|| (k == gmmu_pte_kind_c32_ms2_2cra_v())
		|| (k >= gmmu_pte_kind_c32_ms4_2c_v() &&
			k <= gmmu_pte_kind_c64_ms2_2cbr_v())
		|| (k == gmmu_pte_kind_c64_ms2_2cra_v())
		|| (k >= gmmu_pte_kind_c64_ms4_2c_v() &&
			k <= gmmu_pte_kind_pitch_no_swizzle_v());
}

static noinline_for_stack bool gp10b_kind_compressible(u8 k)
{
	return /* From gp10b */
		(k >= gmmu_pte_kind_z16_2cz_v() &&
			k <= gmmu_pte_kind_z16_ms8_2cz_v())
		|| (k == gmmu_pte_kind_z16_ms16_2cz_v())
		|| (k >= gmmu_pte_kind_z16_4cz_v() &&
			k <= gmmu_pte_kind_z16_ms16_4cz_v())
		|| (k == gmmu_pte_kind_c32_ms4_4cbra_v())
		|| (k == gmmu_pte_kind_c64_ms4_4cbra_v())
		/* From gm20b */
		|| (k >= gmmu_pte_kind_s8_v() &&
			k <= gmmu_pte_kind_s8_2s_v())
		/* From gk20a */
		|| (k >= gmmu_pte_kind_z16_2c_v() &&
			k <= gmmu_pte_kind_z16_ms16_4cz_v())
		|| (k >= gmmu_pte_kind_s8z24_1z_v() &&
			k <= gmmu_pte_kind_s8z24_ms16_4cszv_v())
		|| (k >= gmmu_pte_kind_v8z24_ms4_vc12_1zv_v() &&
			k <= gmmu_pte_kind_v8z24_ms8_vc24_2cs_v())
		|| (k >= gmmu_pte_kind_v8z24_ms4_vc12_2czv_v() &&
			k <= gmmu_pte_kind_v8z24_ms8_vc24_4cszv_v())
		|| (k >= gmmu_pte_kind_z24s8_1z_v() &&
			k <= gmmu_pte_kind_z24s8_ms16_4cszv_v())
		|| (k >= gmmu_pte_kind_z24v8_ms4_vc12_1zv_v() &&
			k <= gmmu_pte_kind_z24v8_ms8_vc24_2cs_v())
		|| (k >= gmmu_pte_kind_z24v8_ms4_vc12_2czv_v() &&
			k <= gmmu_pte_kind_z24v8_ms8_vc24_4cszv_v())
		|| (k >= gmmu_pte_kind_zf32_1z_v() &&
			k <= gmmu_pte_kind_zf32_ms16_2cz_v())
		|| (k >= gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_1cs_v() &&
			k <= gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_1cs_v())
		|| (k >= gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_1zv_v() &&
			k <= gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_2cszv_v())
		|| (k >= gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_1cs_v() &&
			k <= gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_1cs_v())
		|| (k >= gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_1zv_v() &&
			k <= gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_2cszv_v())
		|| (k >= gmmu_pte_kind_zf32_x24s8_1cs_v() &&
			k <= gmmu_pte_kind_zf32_x24s8_ms16_1cs_v())
		|| (k >= gmmu_pte_kind_zf32_x24s8_2cszv_v() &&
			k <= gmmu_pte_kind_c32_ms2_2cbr_v())
		|| (k == gmmu_pte_kind_c32_ms2_2cra_v())
		|| (k >= gmmu_pte_kind_c32_ms4_2c_v() &&
			k <= gmmu_pte_kind_c64_ms2_2cbr_v())
		|| (k == gmmu_pte_kind_c64_ms2_2cra_v())
		|| (k >= gmmu_pte_kind_c64_ms4_2c_v() &&
			k <= gmmu_pte_kind_c128_ms8_ms16_2cr_v());
}

static noinline_for_stack bool gp10b_kind_zbc(u8 k)
{
	return /* From gp10b */
		(k >= gmmu_pte_kind_z16_2cz_v() &&
			k <= gmmu_pte_kind_z16_ms8_2cz_v())
		|| (k == gmmu_pte_kind_z16_ms16_2cz_v())
		|| (k == gmmu_pte_kind_c32_ms4_4cbra_v())
		|| (k == gmmu_pte_kind_c64_ms4_4cbra_v())
		/* From gm20b */
		|| (k >= gmmu_pte_kind_s8_v() &&
			k <= gmmu_pte_kind_s8_2s_v())
		/* From gk20a */
		|| (k >= gmmu_pte_kind_z16_2c_v() &&
			k <= gmmu_pte_kind_z16_ms16_2c_v())
		|| (k >= gmmu_pte_kind_z16_4cz_v() &&
			k <= gmmu_pte_kind_z16_ms16_4cz_v())
		|| (k >= gmmu_pte_kind_s8z24_2cz_v() &&
			k <= gmmu_pte_kind_s8z24_ms16_4cszv_v())
		|| (k >= gmmu_pte_kind_v8z24_ms4_vc12_2cs_v() &&
			k <= gmmu_pte_kind_v8z24_ms8_vc24_2cs_v())
		|| (k >= gmmu_pte_kind_v8z24_ms4_vc12_2czv_v() &&
			k <= gmmu_pte_kind_v8z24_ms8_vc24_2czv_v())
		|| (k >= gmmu_pte_kind_v8z24_ms4_vc12_4cszv_v() &&
			k <= gmmu_pte_kind_v8z24_ms8_vc24_4cszv_v())
		|| (k >= gmmu_pte_kind_z24s8_2cs_v() &&
			k <= gmmu_pte_kind_z24s8_ms16_4cszv_v())
		|| (k >= gmmu_pte_kind_z24v8_ms4_vc12_2cs_v() &&
			k <= gmmu_pte_kind_z24v8_ms8_vc24_2cs_v())
		|| (k >= gmmu_pte_kind_z24v8_ms4_vc12_2czv_v() &&
			k <= gmmu_pte_kind_z24v8_ms8_vc24_2czv_v())
		|| (k >= gmmu_pte_kind_z24v8_ms4_vc12_4cszv_v() &&
			k <= gmmu_pte_kind_z24v8_ms8_vc24_4cszv_v())
		|| (k >= gmmu_pte_kind_zf32_2cs_v() &&
			k <= gmmu_pte_kind_zf32_ms16_2cz_v())
		|| (k >= gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_1cs_v() &&
			k <= gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_1cs_v())
		|| (k >= gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_1czv_v() &&
			k <= gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_2cszv_v())
		|| (k >= gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_1cs_v() &&
			k <= gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_1cs_v())
		|| (k >= gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_1czv_v() &&
			k <= gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_2cszv_v())
		|| (k >= gmmu_pte_kind_zf32_x24s8_1cs_v() &&
			k <= gmmu_pte_kind_zf32_x24s8_ms16_1cs_v())
		|| (k >= gmmu_pte_kind_zf32_x24s8_2cszv_v() &&
			k <= gmmu_pte_kind_c32_2cra_v())
		|| (k >= gmmu_pte_kind_c32_ms2_2c_v() &&
			k <= gmmu_pte_kind_c32_ms2_2cbr_v())
		|| (k == gmmu_pte_kind_c32_ms2_2cra_v())
		|| (k >= gmmu_pte_kind_c32_ms4_2c_v() &&
			k <= gmmu_pte_kind_c32_ms4_2cra_v())
		|| (k >= gmmu_pte_kind_c32_ms8_ms16_2c_v() &&
			k <= gmmu_pte_kind_c64_2cra_v())
		|| (k >= gmmu_pte_kind_c64_ms2_2c_v() &&
			k <= gmmu_pte_kind_c64_ms2_2cbr_v())
		|| (k == gmmu_pte_kind_c64_ms2_2cra_v())
		|| (k >= gmmu_pte_kind_c64_ms4_2c_v() &&
			k <= gmmu_pte_kind_c64_ms4_2cra_v())
		|| (k >= gmmu_pte_kind_c64_ms8_ms16_2c_v() &&
			k <= gmmu_pte_kind_c128_ms8_ms16_2cr_v());
}

void gp10b_init_kind_attr(void)
{
	u16 k;

	for (k = 0; k < NV_KIND_ATTR_SIZE; k++) {
		if (gp10b_kind_supported((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_SUPPORTED;
		if (gp10b_kind_compressible((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_COMPRESSIBLE;
		if (gp10b_kind_z((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_Z;
		if (gp10b_kind_c((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_C;
		if (gp10b_kind_zbc((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_ZBC;
	}
}

unsigned int gp10b_fb_compression_page_size(struct gk20a *g)
{
	return SZ_64K;
}

unsigned int gp10b_fb_compressible_page_size(struct gk20a *g)
{
	return SZ_4K;
}
