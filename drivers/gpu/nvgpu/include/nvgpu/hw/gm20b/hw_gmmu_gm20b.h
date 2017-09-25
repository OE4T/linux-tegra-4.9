/*
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
/*
 * Function naming determines intended use:
 *
 *     <x>_r(void) : Returns the offset for register <x>.
 *
 *     <x>_o(void) : Returns the offset for element <x>.
 *
 *     <x>_w(void) : Returns the word offset for word (4 byte) element <x>.
 *
 *     <x>_<y>_s(void) : Returns size of field <y> of register <x> in bits.
 *
 *     <x>_<y>_f(u32 v) : Returns a value based on 'v' which has been shifted
 *         and masked to place it at field <y> of register <x>.  This value
 *         can be |'d with others to produce a full register value for
 *         register <x>.
 *
 *     <x>_<y>_m(void) : Returns a mask for field <y> of register <x>.  This
 *         value can be ~'d and then &'d to clear the value of field <y> for
 *         register <x>.
 *
 *     <x>_<y>_<z>_f(void) : Returns the constant value <z> after being shifted
 *         to place it at field <y> of register <x>.  This value can be |'d
 *         with others to produce a full register value for <x>.
 *
 *     <x>_<y>_v(u32 r) : Returns the value of field <y> from a full register
 *         <x> value 'r' after being shifted to place its LSB at bit 0.
 *         This value is suitable for direct comparison with other unshifted
 *         values appropriate for use in field <y> of register <x>.
 *
 *     <x>_<y>_<z>_v(void) : Returns the constant value for <z> defined for
 *         field <y> of register <x>.  This value is suitable for direct
 *         comparison with unshifted values appropriate for use in field <y>
 *         of register <x>.
 */
#ifndef _hw_gmmu_gm20b_h_
#define _hw_gmmu_gm20b_h_

static inline u32 gmmu_pde_aperture_big_w(void)
{
	return 0U;
}
static inline u32 gmmu_pde_aperture_big_invalid_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_pde_aperture_big_video_memory_f(void)
{
	return 0x1U;
}
static inline u32 gmmu_pde_aperture_big_sys_mem_coh_f(void)
{
	return 0x2U;
}
static inline u32 gmmu_pde_aperture_big_sys_mem_ncoh_f(void)
{
	return 0x3U;
}
static inline u32 gmmu_pde_size_w(void)
{
	return 0U;
}
static inline u32 gmmu_pde_size_full_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_pde_address_big_sys_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 gmmu_pde_address_big_sys_w(void)
{
	return 0U;
}
static inline u32 gmmu_pde_aperture_small_w(void)
{
	return 1U;
}
static inline u32 gmmu_pde_aperture_small_invalid_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_pde_aperture_small_video_memory_f(void)
{
	return 0x1U;
}
static inline u32 gmmu_pde_aperture_small_sys_mem_coh_f(void)
{
	return 0x2U;
}
static inline u32 gmmu_pde_aperture_small_sys_mem_ncoh_f(void)
{
	return 0x3U;
}
static inline u32 gmmu_pde_vol_small_w(void)
{
	return 1U;
}
static inline u32 gmmu_pde_vol_small_true_f(void)
{
	return 0x4U;
}
static inline u32 gmmu_pde_vol_small_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_pde_vol_big_w(void)
{
	return 1U;
}
static inline u32 gmmu_pde_vol_big_true_f(void)
{
	return 0x8U;
}
static inline u32 gmmu_pde_vol_big_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_pde_address_small_sys_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 gmmu_pde_address_small_sys_w(void)
{
	return 1U;
}
static inline u32 gmmu_pde_address_shift_v(void)
{
	return 0x0000000cU;
}
static inline u32 gmmu_pde__size_v(void)
{
	return 0x00000008U;
}
static inline u32 gmmu_pte__size_v(void)
{
	return 0x00000008U;
}
static inline u32 gmmu_pte_valid_w(void)
{
	return 0U;
}
static inline u32 gmmu_pte_valid_true_f(void)
{
	return 0x1U;
}
static inline u32 gmmu_pte_valid_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_pte_privilege_w(void)
{
	return 0U;
}
static inline u32 gmmu_pte_privilege_true_f(void)
{
	return 0x2U;
}
static inline u32 gmmu_pte_privilege_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_pte_address_sys_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 gmmu_pte_address_sys_w(void)
{
	return 0U;
}
static inline u32 gmmu_pte_address_vid_f(u32 v)
{
	return (v & 0x1ffffffU) << 4U;
}
static inline u32 gmmu_pte_address_vid_w(void)
{
	return 0U;
}
static inline u32 gmmu_pte_vol_w(void)
{
	return 1U;
}
static inline u32 gmmu_pte_vol_true_f(void)
{
	return 0x1U;
}
static inline u32 gmmu_pte_vol_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_pte_aperture_w(void)
{
	return 1U;
}
static inline u32 gmmu_pte_aperture_video_memory_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_pte_aperture_sys_mem_coh_f(void)
{
	return 0x4U;
}
static inline u32 gmmu_pte_aperture_sys_mem_ncoh_f(void)
{
	return 0x6U;
}
static inline u32 gmmu_pte_read_only_w(void)
{
	return 0U;
}
static inline u32 gmmu_pte_read_only_true_f(void)
{
	return 0x4U;
}
static inline u32 gmmu_pte_write_disable_w(void)
{
	return 1U;
}
static inline u32 gmmu_pte_write_disable_true_f(void)
{
	return 0x80000000U;
}
static inline u32 gmmu_pte_read_disable_w(void)
{
	return 1U;
}
static inline u32 gmmu_pte_read_disable_true_f(void)
{
	return 0x40000000U;
}
static inline u32 gmmu_pte_comptagline_s(void)
{
	return 17U;
}
static inline u32 gmmu_pte_comptagline_f(u32 v)
{
	return (v & 0x1ffffU) << 12U;
}
static inline u32 gmmu_pte_comptagline_w(void)
{
	return 1U;
}
static inline u32 gmmu_pte_address_shift_v(void)
{
	return 0x0000000cU;
}
static inline u32 gmmu_pte_kind_f(u32 v)
{
	return (v & 0xffU) << 4U;
}
static inline u32 gmmu_pte_kind_w(void)
{
	return 1U;
}
static inline u32 gmmu_pte_kind_invalid_v(void)
{
	return 0x000000ffU;
}
static inline u32 gmmu_pte_kind_pitch_v(void)
{
	return 0x00000000U;
}
static inline u32 gmmu_pte_kind_z16_v(void)
{
	return 0x00000001U;
}
static inline u32 gmmu_pte_kind_z16_2c_v(void)
{
	return 0x00000002U;
}
static inline u32 gmmu_pte_kind_z16_ms2_2c_v(void)
{
	return 0x00000003U;
}
static inline u32 gmmu_pte_kind_z16_ms4_2c_v(void)
{
	return 0x00000004U;
}
static inline u32 gmmu_pte_kind_z16_ms8_2c_v(void)
{
	return 0x00000005U;
}
static inline u32 gmmu_pte_kind_z16_ms16_2c_v(void)
{
	return 0x00000006U;
}
static inline u32 gmmu_pte_kind_z16_2z_v(void)
{
	return 0x00000007U;
}
static inline u32 gmmu_pte_kind_z16_ms2_2z_v(void)
{
	return 0x00000008U;
}
static inline u32 gmmu_pte_kind_z16_ms4_2z_v(void)
{
	return 0x00000009U;
}
static inline u32 gmmu_pte_kind_z16_ms8_2z_v(void)
{
	return 0x0000000aU;
}
static inline u32 gmmu_pte_kind_z16_ms16_2z_v(void)
{
	return 0x0000000bU;
}
static inline u32 gmmu_pte_kind_z16_4cz_v(void)
{
	return 0x0000000cU;
}
static inline u32 gmmu_pte_kind_z16_ms2_4cz_v(void)
{
	return 0x0000000dU;
}
static inline u32 gmmu_pte_kind_z16_ms4_4cz_v(void)
{
	return 0x0000000eU;
}
static inline u32 gmmu_pte_kind_z16_ms8_4cz_v(void)
{
	return 0x0000000fU;
}
static inline u32 gmmu_pte_kind_z16_ms16_4cz_v(void)
{
	return 0x00000010U;
}
static inline u32 gmmu_pte_kind_s8z24_v(void)
{
	return 0x00000011U;
}
static inline u32 gmmu_pte_kind_s8z24_1z_v(void)
{
	return 0x00000012U;
}
static inline u32 gmmu_pte_kind_s8z24_ms2_1z_v(void)
{
	return 0x00000013U;
}
static inline u32 gmmu_pte_kind_s8z24_ms4_1z_v(void)
{
	return 0x00000014U;
}
static inline u32 gmmu_pte_kind_s8z24_ms8_1z_v(void)
{
	return 0x00000015U;
}
static inline u32 gmmu_pte_kind_s8z24_ms16_1z_v(void)
{
	return 0x00000016U;
}
static inline u32 gmmu_pte_kind_s8z24_2cz_v(void)
{
	return 0x00000017U;
}
static inline u32 gmmu_pte_kind_s8z24_ms2_2cz_v(void)
{
	return 0x00000018U;
}
static inline u32 gmmu_pte_kind_s8z24_ms4_2cz_v(void)
{
	return 0x00000019U;
}
static inline u32 gmmu_pte_kind_s8z24_ms8_2cz_v(void)
{
	return 0x0000001aU;
}
static inline u32 gmmu_pte_kind_s8z24_ms16_2cz_v(void)
{
	return 0x0000001bU;
}
static inline u32 gmmu_pte_kind_s8z24_2cs_v(void)
{
	return 0x0000001cU;
}
static inline u32 gmmu_pte_kind_s8z24_ms2_2cs_v(void)
{
	return 0x0000001dU;
}
static inline u32 gmmu_pte_kind_s8z24_ms4_2cs_v(void)
{
	return 0x0000001eU;
}
static inline u32 gmmu_pte_kind_s8z24_ms8_2cs_v(void)
{
	return 0x0000001fU;
}
static inline u32 gmmu_pte_kind_s8z24_ms16_2cs_v(void)
{
	return 0x00000020U;
}
static inline u32 gmmu_pte_kind_s8z24_4cszv_v(void)
{
	return 0x00000021U;
}
static inline u32 gmmu_pte_kind_s8z24_ms2_4cszv_v(void)
{
	return 0x00000022U;
}
static inline u32 gmmu_pte_kind_s8z24_ms4_4cszv_v(void)
{
	return 0x00000023U;
}
static inline u32 gmmu_pte_kind_s8z24_ms8_4cszv_v(void)
{
	return 0x00000024U;
}
static inline u32 gmmu_pte_kind_s8z24_ms16_4cszv_v(void)
{
	return 0x00000025U;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc12_v(void)
{
	return 0x00000026U;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc4_v(void)
{
	return 0x00000027U;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc8_v(void)
{
	return 0x00000028U;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc24_v(void)
{
	return 0x00000029U;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc12_1zv_v(void)
{
	return 0x0000002eU;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc4_1zv_v(void)
{
	return 0x0000002fU;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc8_1zv_v(void)
{
	return 0x00000030U;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc24_1zv_v(void)
{
	return 0x00000031U;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc12_2cs_v(void)
{
	return 0x00000032U;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc4_2cs_v(void)
{
	return 0x00000033U;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc8_2cs_v(void)
{
	return 0x00000034U;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc24_2cs_v(void)
{
	return 0x00000035U;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc12_2czv_v(void)
{
	return 0x0000003aU;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc4_2czv_v(void)
{
	return 0x0000003bU;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc8_2czv_v(void)
{
	return 0x0000003cU;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc24_2czv_v(void)
{
	return 0x0000003dU;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc12_2zv_v(void)
{
	return 0x0000003eU;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc4_2zv_v(void)
{
	return 0x0000003fU;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc8_2zv_v(void)
{
	return 0x00000040U;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc24_2zv_v(void)
{
	return 0x00000041U;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc12_4cszv_v(void)
{
	return 0x00000042U;
}
static inline u32 gmmu_pte_kind_v8z24_ms4_vc4_4cszv_v(void)
{
	return 0x00000043U;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc8_4cszv_v(void)
{
	return 0x00000044U;
}
static inline u32 gmmu_pte_kind_v8z24_ms8_vc24_4cszv_v(void)
{
	return 0x00000045U;
}
static inline u32 gmmu_pte_kind_z24s8_v(void)
{
	return 0x00000046U;
}
static inline u32 gmmu_pte_kind_z24s8_1z_v(void)
{
	return 0x00000047U;
}
static inline u32 gmmu_pte_kind_z24s8_ms2_1z_v(void)
{
	return 0x00000048U;
}
static inline u32 gmmu_pte_kind_z24s8_ms4_1z_v(void)
{
	return 0x00000049U;
}
static inline u32 gmmu_pte_kind_z24s8_ms8_1z_v(void)
{
	return 0x0000004aU;
}
static inline u32 gmmu_pte_kind_z24s8_ms16_1z_v(void)
{
	return 0x0000004bU;
}
static inline u32 gmmu_pte_kind_z24s8_2cs_v(void)
{
	return 0x0000004cU;
}
static inline u32 gmmu_pte_kind_z24s8_ms2_2cs_v(void)
{
	return 0x0000004dU;
}
static inline u32 gmmu_pte_kind_z24s8_ms4_2cs_v(void)
{
	return 0x0000004eU;
}
static inline u32 gmmu_pte_kind_z24s8_ms8_2cs_v(void)
{
	return 0x0000004fU;
}
static inline u32 gmmu_pte_kind_z24s8_ms16_2cs_v(void)
{
	return 0x00000050U;
}
static inline u32 gmmu_pte_kind_z24s8_2cz_v(void)
{
	return 0x00000051U;
}
static inline u32 gmmu_pte_kind_z24s8_ms2_2cz_v(void)
{
	return 0x00000052U;
}
static inline u32 gmmu_pte_kind_z24s8_ms4_2cz_v(void)
{
	return 0x00000053U;
}
static inline u32 gmmu_pte_kind_z24s8_ms8_2cz_v(void)
{
	return 0x00000054U;
}
static inline u32 gmmu_pte_kind_z24s8_ms16_2cz_v(void)
{
	return 0x00000055U;
}
static inline u32 gmmu_pte_kind_z24s8_4cszv_v(void)
{
	return 0x00000056U;
}
static inline u32 gmmu_pte_kind_z24s8_ms2_4cszv_v(void)
{
	return 0x00000057U;
}
static inline u32 gmmu_pte_kind_z24s8_ms4_4cszv_v(void)
{
	return 0x00000058U;
}
static inline u32 gmmu_pte_kind_z24s8_ms8_4cszv_v(void)
{
	return 0x00000059U;
}
static inline u32 gmmu_pte_kind_z24s8_ms16_4cszv_v(void)
{
	return 0x0000005aU;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc12_v(void)
{
	return 0x0000005bU;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc4_v(void)
{
	return 0x0000005cU;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc8_v(void)
{
	return 0x0000005dU;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc24_v(void)
{
	return 0x0000005eU;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc12_1zv_v(void)
{
	return 0x00000063U;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc4_1zv_v(void)
{
	return 0x00000064U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc8_1zv_v(void)
{
	return 0x00000065U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc24_1zv_v(void)
{
	return 0x00000066U;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc12_2cs_v(void)
{
	return 0x00000067U;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc4_2cs_v(void)
{
	return 0x00000068U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc8_2cs_v(void)
{
	return 0x00000069U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc24_2cs_v(void)
{
	return 0x0000006aU;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc12_2czv_v(void)
{
	return 0x0000006fU;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc4_2czv_v(void)
{
	return 0x00000070U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc8_2czv_v(void)
{
	return 0x00000071U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc24_2czv_v(void)
{
	return 0x00000072U;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc12_2zv_v(void)
{
	return 0x00000073U;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc4_2zv_v(void)
{
	return 0x00000074U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc8_2zv_v(void)
{
	return 0x00000075U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc24_2zv_v(void)
{
	return 0x00000076U;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc12_4cszv_v(void)
{
	return 0x00000077U;
}
static inline u32 gmmu_pte_kind_z24v8_ms4_vc4_4cszv_v(void)
{
	return 0x00000078U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc8_4cszv_v(void)
{
	return 0x00000079U;
}
static inline u32 gmmu_pte_kind_z24v8_ms8_vc24_4cszv_v(void)
{
	return 0x0000007aU;
}
static inline u32 gmmu_pte_kind_zf32_v(void)
{
	return 0x0000007bU;
}
static inline u32 gmmu_pte_kind_zf32_1z_v(void)
{
	return 0x0000007cU;
}
static inline u32 gmmu_pte_kind_zf32_ms2_1z_v(void)
{
	return 0x0000007dU;
}
static inline u32 gmmu_pte_kind_zf32_ms4_1z_v(void)
{
	return 0x0000007eU;
}
static inline u32 gmmu_pte_kind_zf32_ms8_1z_v(void)
{
	return 0x0000007fU;
}
static inline u32 gmmu_pte_kind_zf32_ms16_1z_v(void)
{
	return 0x00000080U;
}
static inline u32 gmmu_pte_kind_zf32_2cs_v(void)
{
	return 0x00000081U;
}
static inline u32 gmmu_pte_kind_zf32_ms2_2cs_v(void)
{
	return 0x00000082U;
}
static inline u32 gmmu_pte_kind_zf32_ms4_2cs_v(void)
{
	return 0x00000083U;
}
static inline u32 gmmu_pte_kind_zf32_ms8_2cs_v(void)
{
	return 0x00000084U;
}
static inline u32 gmmu_pte_kind_zf32_ms16_2cs_v(void)
{
	return 0x00000085U;
}
static inline u32 gmmu_pte_kind_zf32_2cz_v(void)
{
	return 0x00000086U;
}
static inline u32 gmmu_pte_kind_zf32_ms2_2cz_v(void)
{
	return 0x00000087U;
}
static inline u32 gmmu_pte_kind_zf32_ms4_2cz_v(void)
{
	return 0x00000088U;
}
static inline u32 gmmu_pte_kind_zf32_ms8_2cz_v(void)
{
	return 0x00000089U;
}
static inline u32 gmmu_pte_kind_zf32_ms16_2cz_v(void)
{
	return 0x0000008aU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_v(void)
{
	return 0x0000008bU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_v(void)
{
	return 0x0000008cU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_v(void)
{
	return 0x0000008dU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_v(void)
{
	return 0x0000008eU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_1cs_v(void)
{
	return 0x0000008fU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_1cs_v(void)
{
	return 0x00000090U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_1cs_v(void)
{
	return 0x00000091U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_1cs_v(void)
{
	return 0x00000092U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_1zv_v(void)
{
	return 0x00000097U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_1zv_v(void)
{
	return 0x00000098U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_1zv_v(void)
{
	return 0x00000099U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_1zv_v(void)
{
	return 0x0000009aU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_1czv_v(void)
{
	return 0x0000009bU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_1czv_v(void)
{
	return 0x0000009cU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_1czv_v(void)
{
	return 0x0000009dU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_1czv_v(void)
{
	return 0x0000009eU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_2cs_v(void)
{
	return 0x0000009fU;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_2cs_v(void)
{
	return 0x000000a0U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_2cs_v(void)
{
	return 0x000000a1U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_2cs_v(void)
{
	return 0x000000a2U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc12_2cszv_v(void)
{
	return 0x000000a3U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms4_vc4_2cszv_v(void)
{
	return 0x000000a4U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc8_2cszv_v(void)
{
	return 0x000000a5U;
}
static inline u32 gmmu_pte_kind_x8z24_x16v8s8_ms8_vc24_2cszv_v(void)
{
	return 0x000000a6U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_v(void)
{
	return 0x000000a7U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_v(void)
{
	return 0x000000a8U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_v(void)
{
	return 0x000000a9U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_v(void)
{
	return 0x000000aaU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_1cs_v(void)
{
	return 0x000000abU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_1cs_v(void)
{
	return 0x000000acU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_1cs_v(void)
{
	return 0x000000adU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_1cs_v(void)
{
	return 0x000000aeU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_1zv_v(void)
{
	return 0x000000b3U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_1zv_v(void)
{
	return 0x000000b4U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_1zv_v(void)
{
	return 0x000000b5U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_1zv_v(void)
{
	return 0x000000b6U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_1czv_v(void)
{
	return 0x000000b7U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_1czv_v(void)
{
	return 0x000000b8U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_1czv_v(void)
{
	return 0x000000b9U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_1czv_v(void)
{
	return 0x000000baU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_2cs_v(void)
{
	return 0x000000bbU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_2cs_v(void)
{
	return 0x000000bcU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_2cs_v(void)
{
	return 0x000000bdU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_2cs_v(void)
{
	return 0x000000beU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc12_2cszv_v(void)
{
	return 0x000000bfU;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms4_vc4_2cszv_v(void)
{
	return 0x000000c0U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc8_2cszv_v(void)
{
	return 0x000000c1U;
}
static inline u32 gmmu_pte_kind_zf32_x16v8s8_ms8_vc24_2cszv_v(void)
{
	return 0x000000c2U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_v(void)
{
	return 0x000000c3U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_1cs_v(void)
{
	return 0x000000c4U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms2_1cs_v(void)
{
	return 0x000000c5U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms4_1cs_v(void)
{
	return 0x000000c6U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms8_1cs_v(void)
{
	return 0x000000c7U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms16_1cs_v(void)
{
	return 0x000000c8U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_2cszv_v(void)
{
	return 0x000000ceU;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms2_2cszv_v(void)
{
	return 0x000000cfU;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms4_2cszv_v(void)
{
	return 0x000000d0U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms8_2cszv_v(void)
{
	return 0x000000d1U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms16_2cszv_v(void)
{
	return 0x000000d2U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_2cs_v(void)
{
	return 0x000000d3U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms2_2cs_v(void)
{
	return 0x000000d4U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms4_2cs_v(void)
{
	return 0x000000d5U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms8_2cs_v(void)
{
	return 0x000000d6U;
}
static inline u32 gmmu_pte_kind_zf32_x24s8_ms16_2cs_v(void)
{
	return 0x000000d7U;
}
static inline u32 gmmu_pte_kind_generic_16bx2_v(void)
{
	return 0x000000feU;
}
static inline u32 gmmu_pte_kind_c32_2c_v(void)
{
	return 0x000000d8U;
}
static inline u32 gmmu_pte_kind_c32_2cbr_v(void)
{
	return 0x000000d9U;
}
static inline u32 gmmu_pte_kind_c32_2cba_v(void)
{
	return 0x000000daU;
}
static inline u32 gmmu_pte_kind_c32_2cra_v(void)
{
	return 0x000000dbU;
}
static inline u32 gmmu_pte_kind_c32_2bra_v(void)
{
	return 0x000000dcU;
}
static inline u32 gmmu_pte_kind_c32_ms2_2c_v(void)
{
	return 0x000000ddU;
}
static inline u32 gmmu_pte_kind_c32_ms2_2cbr_v(void)
{
	return 0x000000deU;
}
static inline u32 gmmu_pte_kind_c32_ms2_2cra_v(void)
{
	return 0x000000ccU;
}
static inline u32 gmmu_pte_kind_c32_ms4_2c_v(void)
{
	return 0x000000dfU;
}
static inline u32 gmmu_pte_kind_c32_ms4_2cbr_v(void)
{
	return 0x000000e0U;
}
static inline u32 gmmu_pte_kind_c32_ms4_2cba_v(void)
{
	return 0x000000e1U;
}
static inline u32 gmmu_pte_kind_c32_ms4_2cra_v(void)
{
	return 0x000000e2U;
}
static inline u32 gmmu_pte_kind_c32_ms4_2bra_v(void)
{
	return 0x000000e3U;
}
static inline u32 gmmu_pte_kind_c32_ms8_ms16_2c_v(void)
{
	return 0x000000e4U;
}
static inline u32 gmmu_pte_kind_c32_ms8_ms16_2cra_v(void)
{
	return 0x000000e5U;
}
static inline u32 gmmu_pte_kind_c64_2c_v(void)
{
	return 0x000000e6U;
}
static inline u32 gmmu_pte_kind_c64_2cbr_v(void)
{
	return 0x000000e7U;
}
static inline u32 gmmu_pte_kind_c64_2cba_v(void)
{
	return 0x000000e8U;
}
static inline u32 gmmu_pte_kind_c64_2cra_v(void)
{
	return 0x000000e9U;
}
static inline u32 gmmu_pte_kind_c64_2bra_v(void)
{
	return 0x000000eaU;
}
static inline u32 gmmu_pte_kind_c64_ms2_2c_v(void)
{
	return 0x000000ebU;
}
static inline u32 gmmu_pte_kind_c64_ms2_2cbr_v(void)
{
	return 0x000000ecU;
}
static inline u32 gmmu_pte_kind_c64_ms2_2cra_v(void)
{
	return 0x000000cdU;
}
static inline u32 gmmu_pte_kind_c64_ms4_2c_v(void)
{
	return 0x000000edU;
}
static inline u32 gmmu_pte_kind_c64_ms4_2cbr_v(void)
{
	return 0x000000eeU;
}
static inline u32 gmmu_pte_kind_c64_ms4_2cba_v(void)
{
	return 0x000000efU;
}
static inline u32 gmmu_pte_kind_c64_ms4_2cra_v(void)
{
	return 0x000000f0U;
}
static inline u32 gmmu_pte_kind_c64_ms4_2bra_v(void)
{
	return 0x000000f1U;
}
static inline u32 gmmu_pte_kind_c64_ms8_ms16_2c_v(void)
{
	return 0x000000f2U;
}
static inline u32 gmmu_pte_kind_c64_ms8_ms16_2cra_v(void)
{
	return 0x000000f3U;
}
static inline u32 gmmu_pte_kind_c128_2c_v(void)
{
	return 0x000000f4U;
}
static inline u32 gmmu_pte_kind_c128_2cr_v(void)
{
	return 0x000000f5U;
}
static inline u32 gmmu_pte_kind_c128_ms2_2c_v(void)
{
	return 0x000000f6U;
}
static inline u32 gmmu_pte_kind_c128_ms2_2cr_v(void)
{
	return 0x000000f7U;
}
static inline u32 gmmu_pte_kind_c128_ms4_2c_v(void)
{
	return 0x000000f8U;
}
static inline u32 gmmu_pte_kind_c128_ms4_2cr_v(void)
{
	return 0x000000f9U;
}
static inline u32 gmmu_pte_kind_c128_ms8_ms16_2c_v(void)
{
	return 0x000000faU;
}
static inline u32 gmmu_pte_kind_c128_ms8_ms16_2cr_v(void)
{
	return 0x000000fbU;
}
static inline u32 gmmu_pte_kind_x8c24_v(void)
{
	return 0x000000fcU;
}
static inline u32 gmmu_pte_kind_pitch_no_swizzle_v(void)
{
	return 0x000000fdU;
}
static inline u32 gmmu_pte_kind_smsked_message_v(void)
{
	return 0x000000caU;
}
static inline u32 gmmu_pte_kind_smhost_message_v(void)
{
	return 0x000000cbU;
}
static inline u32 gmmu_pte_kind_s8_v(void)
{
	return 0x0000002aU;
}
static inline u32 gmmu_pte_kind_s8_2s_v(void)
{
	return 0x0000002bU;
}
#endif
