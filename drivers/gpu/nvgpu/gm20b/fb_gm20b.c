/*
 * GM20B GPC MMU
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "gk20a/kind_gk20a.h"
#include "gk20a/fb_gk20a.h"
#include "gm20b/fb_gm20b.h"

#include <nvgpu/hw/gm20b/hw_fb_gm20b.h>
#include <nvgpu/hw/gm20b/hw_top_gm20b.h>
#include <nvgpu/hw/gm20b/hw_gmmu_gm20b.h>
#include <nvgpu/hw/gm20b/hw_gr_gm20b.h>

#define VPR_INFO_FETCH_WAIT	(5)

static void fb_gm20b_init_fs_state(struct gk20a *g)
{
	gk20a_dbg_info("initialize gm20b fb");

	gk20a_writel(g, fb_fbhub_num_active_ltcs_r(),
			g->ltc_count);
}

noinline_for_stack void gm20b_init_uncompressed_kind_map(void)
{
	int i;

	for (i = 0; i < 256; i++)
		gk20a_uc_kind_map[i] = gmmu_pte_kind_invalid_v();

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

static noinline_for_stack bool gm20b_kind_supported(u8 k)
{
	return /* From gm20b */
		(k == gmmu_pte_kind_smsked_message_v())
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

static noinline_for_stack bool gm20b_kind_z(u8 k)
{
	return /* From gm20b */
		(k >= gmmu_pte_kind_s8_v() &&
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

static noinline_for_stack bool gm20b_kind_c(u8 k)
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

static noinline_for_stack bool gm20b_kind_compressible(u8 k)
{
	return /* From gm20b */
		(k >= gmmu_pte_kind_s8_v() &&
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

static noinline_for_stack bool gm20b_kind_zbc(u8 k)
{
	return /* From gm20b */
		(k >= gmmu_pte_kind_s8_v() &&
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

void gm20b_init_kind_attr(void)
{
	u16 k;

	for (k = 0; k < NV_KIND_ATTR_SIZE; k++) {
		gk20a_kind_attr[k] = 0;
		if (gm20b_kind_supported((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_SUPPORTED;
		if (gm20b_kind_compressible((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_COMPRESSIBLE;
		if (gm20b_kind_z((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_Z;
		if (gm20b_kind_c((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_C;
		if (gm20b_kind_zbc((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_ZBC;
	}
}

static void gm20b_fb_set_mmu_page_size(struct gk20a *g)
{
	/* set large page size in fb */
	u32 fb_mmu_ctrl = gk20a_readl(g, fb_mmu_ctrl_r());
	fb_mmu_ctrl |= fb_mmu_ctrl_use_pdb_big_page_size_true_f();
	gk20a_writel(g, fb_mmu_ctrl_r(), fb_mmu_ctrl);
}

static bool gm20b_fb_set_use_full_comp_tag_line(struct gk20a *g)
{
	/* set large page size in fb */
	u32 fb_mmu_ctrl = gk20a_readl(g, fb_mmu_ctrl_r());
	fb_mmu_ctrl |= fb_mmu_ctrl_use_full_comp_tag_line_true_f();
	gk20a_writel(g, fb_mmu_ctrl_r(), fb_mmu_ctrl);

	return true;
}

static unsigned int gm20b_fb_compression_page_size(struct gk20a *g)
{
	return SZ_128K;
}

static unsigned int gm20b_fb_compressible_page_size(struct gk20a *g)
{
	return SZ_64K;
}

static void gm20b_fb_dump_vpr_wpr_info(struct gk20a *g)
{
	u32 val;

	/* print vpr and wpr info */
	val = gk20a_readl(g, fb_mmu_vpr_info_r());
	val &= ~0x3;
	val |= fb_mmu_vpr_info_index_addr_lo_v();
	gk20a_writel(g, fb_mmu_vpr_info_r(), val);
	nvgpu_err(g, "VPR: %08x %08x %08x %08x",
		gk20a_readl(g, fb_mmu_vpr_info_r()),
		gk20a_readl(g, fb_mmu_vpr_info_r()),
		gk20a_readl(g, fb_mmu_vpr_info_r()),
		gk20a_readl(g, fb_mmu_vpr_info_r()));

	val = gk20a_readl(g, fb_mmu_wpr_info_r());
	val &= ~0xf;
	val |= (fb_mmu_wpr_info_index_allow_read_v());
	gk20a_writel(g, fb_mmu_wpr_info_r(), val);
	nvgpu_err(g, "WPR: %08x %08x %08x %08x %08x %08x",
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()));

}

static int gm20b_fb_vpr_info_fetch_wait(struct gk20a *g,
					    unsigned int msec)
{
	struct nvgpu_timeout timeout;

	nvgpu_timeout_init(g, &timeout, msec, NVGPU_TIMER_CPU_TIMER);

	do {
		u32 val;

		val = gk20a_readl(g, fb_mmu_vpr_info_r());
		if (fb_mmu_vpr_info_fetch_v(val) ==
		    fb_mmu_vpr_info_fetch_false_v())
			return 0;

	} while (!nvgpu_timeout_expired(&timeout));

	return -ETIMEDOUT;
}

static int gm20b_fb_vpr_info_fetch(struct gk20a *g)
{
	if (gm20b_fb_vpr_info_fetch_wait(g, VPR_INFO_FETCH_WAIT)) {
		return -ETIME;
	}

	gk20a_writel(g, fb_mmu_vpr_info_r(),
			fb_mmu_vpr_info_fetch_true_v());

	return gm20b_fb_vpr_info_fetch_wait(g, VPR_INFO_FETCH_WAIT);
}

static bool gm20b_fb_debug_mode_enabled(struct gk20a *g)
{
	u32 debug_ctrl = gk20a_readl(g, gr_gpcs_pri_mmu_debug_ctrl_r());
	return gr_gpcs_pri_mmu_debug_ctrl_debug_v(debug_ctrl) ==
		gr_gpcs_pri_mmu_debug_ctrl_debug_enabled_v();
}

static void gm20b_fb_set_debug_mode(struct gk20a *g, bool enable)
{
	u32 reg_val, fb_debug_ctrl, gpc_debug_ctrl;

	if (enable) {
		fb_debug_ctrl = fb_mmu_debug_ctrl_debug_enabled_f();
		gpc_debug_ctrl = gr_gpcs_pri_mmu_debug_ctrl_debug_enabled_f();
		g->mmu_debug_ctrl = true;
	} else {
		fb_debug_ctrl = fb_mmu_debug_ctrl_debug_disabled_f();
		gpc_debug_ctrl = gr_gpcs_pri_mmu_debug_ctrl_debug_disabled_f();
		g->mmu_debug_ctrl = false;
	}

	reg_val = gk20a_readl(g, fb_mmu_debug_ctrl_r());
	reg_val = set_field(reg_val,
			fb_mmu_debug_ctrl_debug_m(), fb_debug_ctrl);
	gk20a_writel(g, fb_mmu_debug_ctrl_r(), reg_val);

	reg_val = gk20a_readl(g, gr_gpcs_pri_mmu_debug_ctrl_r());
	reg_val = set_field(reg_val,
			gr_gpcs_pri_mmu_debug_ctrl_debug_m(), gpc_debug_ctrl);
	gk20a_writel(g, gr_gpcs_pri_mmu_debug_ctrl_r(), reg_val);
}

void gm20b_init_fb(struct gpu_ops *gops)
{
	gops->fb.reset = fb_gk20a_reset;
	gops->fb.init_hw = gk20a_fb_init_hw;
	gops->fb.init_fs_state = fb_gm20b_init_fs_state;
	gops->fb.set_mmu_page_size = gm20b_fb_set_mmu_page_size;
	gops->fb.set_use_full_comp_tag_line = gm20b_fb_set_use_full_comp_tag_line;
	gops->fb.compression_page_size = gm20b_fb_compression_page_size;
	gops->fb.compressible_page_size = gm20b_fb_compressible_page_size;
	gops->fb.vpr_info_fetch = gm20b_fb_vpr_info_fetch;
	gops->fb.dump_vpr_wpr_info = gm20b_fb_dump_vpr_wpr_info;
	gops->fb.is_debug_mode_enabled = gm20b_fb_debug_mode_enabled;
	gops->fb.set_debug_mode = gm20b_fb_set_debug_mode;
	gops->fb.tlb_invalidate = gk20a_fb_tlb_invalidate;
	gm20b_init_uncompressed_kind_map();
	gm20b_init_kind_attr();
}
