/*
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
#ifndef _hw_trim_gk20a_h_
#define _hw_trim_gk20a_h_

static inline u32 trim_sys_gpcpll_cfg_r(void)
{
	return 0x00137000;
}
static inline u32 trim_sys_gpcpll_cfg_enable_m(void)
{
	return 0x1 << 0;
}
static inline u32 trim_sys_gpcpll_cfg_enable_v(u32 r)
{
	return (r >> 0) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_enable_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_enable_yes_f(void)
{
	return 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_m(void)
{
	return 0x1 << 1;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_v(u32 r)
{
	return (r >> 1) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_power_on_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_m(void)
{
	return 0x1 << 4;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_power_on_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_power_off_f(void)
{
	return 0x10;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_v(u32 r)
{
	return (r >> 17) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_true_f(void)
{
	return 0x20000;
}
static inline u32 trim_sys_gpcpll_coeff_r(void)
{
	return 0x00137004;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_s(void)
{
	return 8;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_f(u32 v)
{
	return (v & 0xff) << 0;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_m(void)
{
	return 0xff << 0;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_v(u32 r)
{
	return (r >> 0) & 0xff;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_s(void)
{
	return 8;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_f(u32 v)
{
	return (v & 0xff) << 8;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_m(void)
{
	return 0xff << 8;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_v(u32 r)
{
	return (r >> 8) & 0xff;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_r(u32 i)
{
	return 0x00134124 + i*512;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_noofipclks_f(u32 v)
{
	return (v & 0x3fff) << 0;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_write_en_asserted_f(void)
{
	return 0x10000;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_enable_asserted_f(void)
{
	return 0x100000;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_reset_asserted_f(void)
{
	return 0x1000000;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cnt_r(u32 i)
{
	return 0x00134128 + i*512;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cnt_value_v(u32 r)
{
	return (r >> 0) & 0xfffff;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_min_v(void)
{
	return 0x00000008;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_min_f(void)
{
	return 0x800;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_max_v(void)
{
	return 0x000000FF;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_max_f(void)
{
	return 0xff00;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_s(void)
{
	return 6;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_f(u32 v)
{
	return (v & 0x3f) << 16;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_m(void)
{
	return 0x3f << 16;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_v(u32 r)
{
	return (r >> 16) & 0x3f;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_init_f(void)
{
	return 0x10000;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_min_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_min_f(void)
{
	return 0x10000;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_max_v(void)
{
	return 0x0000003F;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_max_f(void)
{
	return 0x3f0000;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_f(u32 v)
{
	return (v & 0x1) << 30;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_m(void)
{
	return 0x1 << 30;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_v(u32 r)
{
	return (r >> 30) & 0x1;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_cya_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_m(void)
{
	return 0x1 << 31;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_coeff_clamp_ndiv_override_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg2_r(void)
{
	return 0x0013700c;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_s(void)
{
	return 8;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_f(u32 v)
{
	return (v & 0xff) << 16;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_m(void)
{
	return 0xff << 16;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_v(u32 r)
{
	return (r >> 16) & 0xff;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_f(u32 v)
{
	return (v & 0x1) << 16;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_m(void)
{
	return 0x1 << 16;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_v(u32 r)
{
	return (r >> 16) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_internal_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_internal_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_external_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_calibrate_m_n_external_f(void)
{
	return 0x10000;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_cml_div_reset_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_cml_div_reset_f(u32 v)
{
	return (v & 0x1) << 17;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_cml_div_reset_m(void)
{
	return 0x1 << 17;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_cml_div_reset_v(u32 r)
{
	return (r >> 17) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_cml_div_reset_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_cml_div_reset_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_loop_filter_ctl_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_loop_filter_ctl_f(u32 v)
{
	return (v & 0x1) << 18;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_loop_filter_ctl_m(void)
{
	return 0x1 << 18;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_loop_filter_ctl_v(u32 r)
{
	return (r >> 18) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_loop_filter_ctl_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_loop_filter_ctl_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_internal_cal_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_internal_cal_f(u32 v)
{
	return (v & 0x1) << 19;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_internal_cal_m(void)
{
	return 0x1 << 19;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_internal_cal_v(u32 r)
{
	return (r >> 19) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_internal_cal_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_internal_cal_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_unused_s(void)
{
	return 3;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_unused_f(u32 v)
{
	return (v & 0x7) << 20;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_unused_m(void)
{
	return 0x7 << 20;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_unused_v(u32 r)
{
	return (r >> 20) & 0x7;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_unused_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_unused_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_bypass_pdiv_counter_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_bypass_pdiv_counter_f(u32 v)
{
	return (v & 0x1) << 23;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_bypass_pdiv_counter_m(void)
{
	return 0x1 << 23;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_bypass_pdiv_counter_v(u32 r)
{
	return (r >> 23) & 0x1;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_bypass_pdiv_counter_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_cfg2_setup2_bypass_pdiv_counter_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_cfg2_pll_stepa_s(void)
{
	return 8;
}
static inline u32 trim_sys_gpcpll_cfg2_pll_stepa_f(u32 v)
{
	return (v & 0xff) << 24;
}
static inline u32 trim_sys_gpcpll_cfg2_pll_stepa_m(void)
{
	return 0xff << 24;
}
static inline u32 trim_sys_gpcpll_cfg2_pll_stepa_v(u32 r)
{
	return (r >> 24) & 0xff;
}
static inline u32 trim_sys_gpcpll_cfg2_pll_stepa_init_v(void)
{
	return 0x00000014;
}
static inline u32 trim_sys_gpcpll_cfg2_pll_stepa_init_f(void)
{
	return 0x14000000;
}
static inline u32 trim_sys_gpcpll_cfg3_r(void)
{
	return 0x00137018;
}
static inline u32 trim_sys_gpcpll_cfg3_pll_stepb_s(void)
{
	return 8;
}
static inline u32 trim_sys_gpcpll_cfg3_pll_stepb_f(u32 v)
{
	return (v & 0xff) << 16;
}
static inline u32 trim_sys_gpcpll_cfg3_pll_stepb_m(void)
{
	return 0xff << 16;
}
static inline u32 trim_sys_gpcpll_cfg3_pll_stepb_v(u32 r)
{
	return (r >> 16) & 0xff;
}
static inline u32 trim_sys_gpcpll_cfg3_pll_stepb_init_v(void)
{
	return 0x00000004;
}
static inline u32 trim_sys_gpcpll_cfg3_pll_stepb_init_f(void)
{
	return 0x40000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_r(void)
{
	return 0x0013701c;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_s(void)
{
	return 8;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_f(u32 v)
{
	return (v & 0xff) << 0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_m(void)
{
	return 0xff << 0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_v(u32 r)
{
	return (r >> 0) & 0xff;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_init_v(void)
{
	return 0x0000002A;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_init_f(void)
{
	return 0x2a;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_min_v(void)
{
	return 0x00000008;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_min_f(void)
{
	return 0x8;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_max_v(void)
{
	return 0x000000FF;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_lo_max_f(void)
{
	return 0xff;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_s(void)
{
	return 8;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_f(u32 v)
{
	return (v & 0xff) << 8;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_m(void)
{
	return 0xff << 8;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_v(u32 r)
{
	return (r >> 8) & 0xff;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_init_v(void)
{
	return 0x00000037;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_init_f(void)
{
	return 0x3700;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_min_v(void)
{
	return 0x00000008;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_min_f(void)
{
	return 0x800;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_max_v(void)
{
	return 0x000000FF;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_ndiv_mid_max_f(void)
{
	return 0xff00;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_s(void)
{
	return 6;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_f(u32 v)
{
	return (v & 0x3f) << 16;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_m(void)
{
	return 0x3f << 16;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_v(u32 r)
{
	return (r >> 16) & 0x3f;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_min_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_min_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_max_v(void)
{
	return 0x0000003F;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_lo2mid_max_f(void)
{
	return 0x3f0000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_f(u32 v)
{
	return (v & 0x1) << 22;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_m(void)
{
	return 0x1 << 22;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_v(u32 r)
{
	return (r >> 22) & 0x1;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_no_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_yes_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_yes_f(void)
{
	return 0x400000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_s(void)
{
	return 6;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_f(u32 v)
{
	return (v & 0x3f) << 24;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_m(void)
{
	return 0x3f << 24;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_v(u32 r)
{
	return (r >> 24) & 0x3f;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_init_f(void)
{
	return 0x1000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_min_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_min_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_max_v(void)
{
	return 0x0000003F;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_step_size_mid2hi_max_f(void)
{
	return 0x3f000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_f(u32 v)
{
	return (v & 0x1) << 30;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_m(void)
{
	return 0x1 << 30;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_v(u32 r)
{
	return (r >> 30) & 0x1;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_init_f(void)
{
	return 0x40000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_pll_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_pll_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_hw_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_hw_mid2hi_hw_f(void)
{
	return 0x40000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_m(void)
{
	return 0x1 << 31;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_no_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_yes_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_yes_f(void)
{
	return 0x80000000;
}
static inline u32 trim_sys_sel_vco_r(void)
{
	return 0x00137100;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_s(void)
{
	return 1;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_f(u32 v)
{
	return (v & 0x1) << 0;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_m(void)
{
	return 0x1 << 0;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_v(u32 r)
{
	return (r >> 0) & 0x1;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_bypass_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_bypass_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_vco_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_vco_f(void)
{
	return 0x1;
}
static inline u32 trim_sys_gpc2clk_out_r(void)
{
	return 0x00137250;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_s(void)
{
	return 6;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_f(u32 v)
{
	return (v & 0x3f) << 0;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_m(void)
{
	return 0x3f << 0;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_v(u32 r)
{
	return (r >> 0) & 0x3f;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_init_v(void)
{
	return 0x0000003C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_init_f(void)
{
	return 0x3c;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by1_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by1_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by1p5_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by1p5_f(void)
{
	return 0x1;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by2_v(void)
{
	return 0x00000002;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by2_f(void)
{
	return 0x2;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by2p5_v(void)
{
	return 0x00000003;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by2p5_f(void)
{
	return 0x3;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by3_v(void)
{
	return 0x00000004;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by3_f(void)
{
	return 0x4;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by3p5_v(void)
{
	return 0x00000005;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by3p5_f(void)
{
	return 0x5;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by4_v(void)
{
	return 0x00000006;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by4_f(void)
{
	return 0x6;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by4p5_v(void)
{
	return 0x00000007;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by4p5_f(void)
{
	return 0x7;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by5_v(void)
{
	return 0x00000008;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by5_f(void)
{
	return 0x8;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by5p5_v(void)
{
	return 0x00000009;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by5p5_f(void)
{
	return 0x9;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by6_v(void)
{
	return 0x0000000A;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by6_f(void)
{
	return 0xa;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by6p5_v(void)
{
	return 0x0000000B;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by6p5_f(void)
{
	return 0xb;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by7_v(void)
{
	return 0x0000000C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by7_f(void)
{
	return 0xc;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by7p5_v(void)
{
	return 0x0000000D;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by7p5_f(void)
{
	return 0xd;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by8_v(void)
{
	return 0x0000000E;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by8_f(void)
{
	return 0xe;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by8p5_v(void)
{
	return 0x0000000F;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by8p5_f(void)
{
	return 0xf;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by9_v(void)
{
	return 0x00000010;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by9_f(void)
{
	return 0x10;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by9p5_v(void)
{
	return 0x00000011;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by9p5_f(void)
{
	return 0x11;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by10_v(void)
{
	return 0x00000012;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by10_f(void)
{
	return 0x12;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by10p5_v(void)
{
	return 0x00000013;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by10p5_f(void)
{
	return 0x13;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by11_v(void)
{
	return 0x00000014;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by11_f(void)
{
	return 0x14;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by11p5_v(void)
{
	return 0x00000015;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by11p5_f(void)
{
	return 0x15;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by12_v(void)
{
	return 0x00000016;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by12_f(void)
{
	return 0x16;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by12p5_v(void)
{
	return 0x00000017;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by12p5_f(void)
{
	return 0x17;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by13_v(void)
{
	return 0x00000018;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by13_f(void)
{
	return 0x18;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by13p5_v(void)
{
	return 0x00000019;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by13p5_f(void)
{
	return 0x19;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by14_v(void)
{
	return 0x0000001A;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by14_f(void)
{
	return 0x1a;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by14p5_v(void)
{
	return 0x0000001B;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by14p5_f(void)
{
	return 0x1b;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by15_v(void)
{
	return 0x0000001C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by15_f(void)
{
	return 0x1c;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by15p5_v(void)
{
	return 0x0000001D;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by15p5_f(void)
{
	return 0x1d;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by16_v(void)
{
	return 0x0000001E;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by16_f(void)
{
	return 0x1e;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by16p5_v(void)
{
	return 0x0000001F;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by16p5_f(void)
{
	return 0x1f;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by17_v(void)
{
	return 0x00000020;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by17_f(void)
{
	return 0x20;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by18_v(void)
{
	return 0x00000022;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by18_f(void)
{
	return 0x22;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by19_v(void)
{
	return 0x00000024;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by19_f(void)
{
	return 0x24;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by20_v(void)
{
	return 0x00000026;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by20_f(void)
{
	return 0x26;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by21_v(void)
{
	return 0x00000028;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by21_f(void)
{
	return 0x28;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by22_v(void)
{
	return 0x0000002A;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by22_f(void)
{
	return 0x2a;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by23_v(void)
{
	return 0x0000002C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by23_f(void)
{
	return 0x2c;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by24_v(void)
{
	return 0x0000002E;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by24_f(void)
{
	return 0x2e;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by25_v(void)
{
	return 0x00000030;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by25_f(void)
{
	return 0x30;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by26_v(void)
{
	return 0x00000032;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by26_f(void)
{
	return 0x32;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by27_v(void)
{
	return 0x00000034;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by27_f(void)
{
	return 0x34;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by28_v(void)
{
	return 0x00000036;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by28_f(void)
{
	return 0x36;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by29_v(void)
{
	return 0x00000038;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by29_f(void)
{
	return 0x38;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by30_v(void)
{
	return 0x0000003A;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by30_f(void)
{
	return 0x3a;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by31_v(void)
{
	return 0x0000003C;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by31_f(void)
{
	return 0x3c;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_s(void)
{
	return 6;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_f(u32 v)
{
	return (v & 0x3f) << 8;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_m(void)
{
	return 0x3f << 8;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_v(u32 r)
{
	return (r >> 8) & 0x3f;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by1_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by1_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by1p5_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by1p5_f(void)
{
	return 0x100;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by2_v(void)
{
	return 0x00000002;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by2_f(void)
{
	return 0x200;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by2p5_v(void)
{
	return 0x00000003;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by2p5_f(void)
{
	return 0x300;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by3_v(void)
{
	return 0x00000004;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by3_f(void)
{
	return 0x400;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by3p5_v(void)
{
	return 0x00000005;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by3p5_f(void)
{
	return 0x500;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by4_v(void)
{
	return 0x00000006;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by4_f(void)
{
	return 0x600;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by4p5_v(void)
{
	return 0x00000007;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by4p5_f(void)
{
	return 0x700;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by5_v(void)
{
	return 0x00000008;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by5_f(void)
{
	return 0x800;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by5p5_v(void)
{
	return 0x00000009;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by5p5_f(void)
{
	return 0x900;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by6_v(void)
{
	return 0x0000000A;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by6_f(void)
{
	return 0xa00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by6p5_v(void)
{
	return 0x0000000B;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by6p5_f(void)
{
	return 0xb00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by7_v(void)
{
	return 0x0000000C;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by7_f(void)
{
	return 0xc00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by7p5_v(void)
{
	return 0x0000000D;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by7p5_f(void)
{
	return 0xd00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by8_v(void)
{
	return 0x0000000E;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by8_f(void)
{
	return 0xe00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by8p5_v(void)
{
	return 0x0000000F;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by8p5_f(void)
{
	return 0xf00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by9_v(void)
{
	return 0x00000010;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by9_f(void)
{
	return 0x1000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by9p5_v(void)
{
	return 0x00000011;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by9p5_f(void)
{
	return 0x1100;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by10_v(void)
{
	return 0x00000012;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by10_f(void)
{
	return 0x1200;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by10p5_v(void)
{
	return 0x00000013;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by10p5_f(void)
{
	return 0x1300;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by11_v(void)
{
	return 0x00000014;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by11_f(void)
{
	return 0x1400;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by11p5_v(void)
{
	return 0x00000015;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by11p5_f(void)
{
	return 0x1500;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by12_v(void)
{
	return 0x00000016;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by12_f(void)
{
	return 0x1600;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by12p5_v(void)
{
	return 0x00000017;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by12p5_f(void)
{
	return 0x1700;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by13_v(void)
{
	return 0x00000018;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by13_f(void)
{
	return 0x1800;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by13p5_v(void)
{
	return 0x00000019;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by13p5_f(void)
{
	return 0x1900;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by14_v(void)
{
	return 0x0000001A;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by14_f(void)
{
	return 0x1a00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by14p5_v(void)
{
	return 0x0000001B;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by14p5_f(void)
{
	return 0x1b00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by15_v(void)
{
	return 0x0000001C;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by15_f(void)
{
	return 0x1c00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by15p5_v(void)
{
	return 0x0000001D;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by15p5_f(void)
{
	return 0x1d00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by16_v(void)
{
	return 0x0000001E;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by16_f(void)
{
	return 0x1e00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by16p5_v(void)
{
	return 0x0000001F;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by16p5_f(void)
{
	return 0x1f00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by17_v(void)
{
	return 0x00000020;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by17_f(void)
{
	return 0x2000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by18_v(void)
{
	return 0x00000022;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by18_f(void)
{
	return 0x2200;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by19_v(void)
{
	return 0x00000024;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by19_f(void)
{
	return 0x2400;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by20_v(void)
{
	return 0x00000026;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by20_f(void)
{
	return 0x2600;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by21_v(void)
{
	return 0x00000028;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by21_f(void)
{
	return 0x2800;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by22_v(void)
{
	return 0x0000002A;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by22_f(void)
{
	return 0x2a00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by23_v(void)
{
	return 0x0000002C;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by23_f(void)
{
	return 0x2c00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by24_v(void)
{
	return 0x0000002E;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by24_f(void)
{
	return 0x2e00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by25_v(void)
{
	return 0x00000030;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by25_f(void)
{
	return 0x3000;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by26_v(void)
{
	return 0x00000032;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by26_f(void)
{
	return 0x3200;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by27_v(void)
{
	return 0x00000034;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by27_f(void)
{
	return 0x3400;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by28_v(void)
{
	return 0x00000036;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by28_f(void)
{
	return 0x3600;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by29_v(void)
{
	return 0x00000038;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by29_f(void)
{
	return 0x3800;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by30_v(void)
{
	return 0x0000003A;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by30_f(void)
{
	return 0x3a00;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by31_v(void)
{
	return 0x0000003C;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by31_f(void)
{
	return 0x3c00;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_s(void)
{
	return 2;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_f(u32 v)
{
	return (v & 0x3) << 20;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_m(void)
{
	return 0x3 << 20;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_v(u32 r)
{
	return (r >> 20) & 0x3;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_load_cnt_init_f(void)
{
	return 0x100000;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_f(u32 v)
{
	return (v & 0x1) << 24;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_m(void)
{
	return 0x1 << 24;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_v(u32 r)
{
	return (r >> 24) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_gateclkdly_init_f(void)
{
	return 0x1000000;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_f(u32 v)
{
	return (v & 0x1) << 25;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_m(void)
{
	return 0x1 << 25;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_v(u32 r)
{
	return (r >> 25) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_no_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_yes_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating_yes_f(void)
{
	return 0x2000000;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating__prod_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_input_clock_gating__prod_f(void)
{
	return 0x2000000;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_f(u32 v)
{
	return (v & 0x1) << 26;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_m(void)
{
	return 0x1 << 26;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_v(u32 r)
{
	return (r >> 26) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_no_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_yes_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating_yes_f(void)
{
	return 0x4000000;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating__prod_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_int_clock_gating__prod_f(void)
{
	return 0x4000000;
}
static inline u32 trim_sys_gpc2clk_out_gclks_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_gclks_f(u32 v)
{
	return (v & 0x1) << 27;
}
static inline u32 trim_sys_gpc2clk_out_gclks_m(void)
{
	return 0x1 << 27;
}
static inline u32 trim_sys_gpc2clk_out_gclks_v(u32 r)
{
	return (r >> 27) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_gclks_init_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_gclks_init_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_gclks_no_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_gclks_no_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_gclks_yes_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_gclks_yes_f(void)
{
	return 0x8000000;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_s(void)
{
	return 1;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_m(void)
{
	return 0x1 << 31;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_init_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_init_f(void)
{
	return 0x80000000;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_indiv1_mode_v(void)
{
	return 0x00000000;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_indiv1_mode_f(void)
{
	return 0x0;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_indiv4_mode_v(void)
{
	return 0x00000001;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_indiv4_mode_f(void)
{
	return 0x80000000;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_r(void)
{
	return 0x001328a0;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_gpcpll_ndiv_sync_s(void)
{
	return 8;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_gpcpll_ndiv_sync_f(u32 v)
{
	return (v & 0xff) << 0;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_gpcpll_ndiv_sync_m(void)
{
	return 0xff << 0;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_gpcpll_ndiv_sync_v(u32 r)
{
	return (r >> 0) & 0xff;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_ndiv_sm_state_s(void)
{
	return 4;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_ndiv_sm_state_f(u32 v)
{
	return (v & 0xf) << 8;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_ndiv_sm_state_m(void)
{
	return 0xf << 8;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_ndiv_sm_state_v(u32 r)
{
	return (r >> 8) & 0xf;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_fpdiv_sm_state_s(void)
{
	return 3;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_fpdiv_sm_state_f(u32 v)
{
	return (v & 0x7) << 13;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_fpdiv_sm_state_m(void)
{
	return 0x7 << 13;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_fpdiv_sm_state_v(u32 r)
{
	return (r >> 13) & 0x7;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_ndiv_state_s(void)
{
	return 3;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_ndiv_state_f(u32 v)
{
	return (v & 0x7) << 16;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_ndiv_state_m(void)
{
	return 0x7 << 16;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_ndiv_state_v(u32 r)
{
	return (r >> 16) & 0x7;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_en_dynramp_pll_s(void)
{
	return 1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_en_dynramp_pll_f(u32 v)
{
	return (v & 0x1) << 19;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_en_dynramp_pll_m(void)
{
	return 0x1 << 19;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_en_dynramp_pll_v(u32 r)
{
	return (r >> 19) & 0x1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_using_ndiv_lo_s(void)
{
	return 1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_using_ndiv_lo_f(u32 v)
{
	return (v & 0x1) << 20;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_using_ndiv_lo_m(void)
{
	return 0x1 << 20;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_using_ndiv_lo_v(u32 r)
{
	return (r >> 20) & 0x1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_switch_to_ndiv_lo_s(void)
{
	return 1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_switch_to_ndiv_lo_f(u32 v)
{
	return (v & 0x1) << 21;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_switch_to_ndiv_lo_m(void)
{
	return 0x1 << 21;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_switch_to_ndiv_lo_v(u32 r)
{
	return (r >> 21) & 0x1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_switch_to_ndiv_hi_s(void)
{
	return 1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_switch_to_ndiv_hi_f(u32 v)
{
	return (v & 0x1) << 22;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_switch_to_ndiv_hi_m(void)
{
	return 0x1 << 22;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_switch_to_ndiv_hi_v(u32 r)
{
	return (r >> 22) & 0x1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_fpdiv_ack_s(void)
{
	return 1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_fpdiv_ack_f(u32 v)
{
	return (v & 0x1) << 23;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_fpdiv_ack_m(void)
{
	return 0x1 << 23;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_fpdiv_ack_v(u32 r)
{
	return (r >> 23) & 0x1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_pll_dynramp_done_synced_s(void)
{
	return 1;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_pll_dynramp_done_synced_f(u32 v)
{
	return (v & 0x1) << 24;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_pll_dynramp_done_synced_m(void)
{
	return 0x1 << 24;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_pll_dynramp_done_synced_v(u32 r)
{
	return (r >> 24) & 0x1;
}

#endif /* __hw_trim_gk20a_h__ */
