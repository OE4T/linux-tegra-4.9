/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_host1x5_sync_h_
#define _hw_host1x5_sync_h_

static inline u32 host1x_sync_intstatus_r(void)
{
	return 0x1c;
}
static inline u32 host1x_sync_intstatus_ip_read_int_v(u32 r)
{
	return (r >> 0) & 0x1;
}
static inline u32 host1x_sync_intstatus_ip_write_int_v(u32 r)
{
	return (r >> 1) & 0x1;
}
static inline u32 host1x_sync_intmask_r(void)
{
	return 0x30;
}
static inline u32 host1x_sync_intgmask_r(void)
{
	return 0x44;
}
static inline u32 host1x_sync_intc0mask_r(void)
{
	return 0x4;
}
static inline u32 host1x_sync_cmdproc_stat_r(void)
{
	return 0x38;
}
static inline u32 host1x_sync_cmdproc_stop_r(void)
{
	return 0x48;
}
static inline u32 host1x_sync_ch_teardown_r(void)
{
	return 0x4c;
}
static inline u32 host1x_sync_usec_clk_r(void)
{
	return 0x2524;
}
static inline u32 host1x_sync_ctxsw_timeout_cfg_r(void)
{
	return 0x2528;
}
static inline u32 host1x_sync_ip_busy_timeout_r(void)
{
	return 0x2530;
}
static inline u32 host1x_sync_ip_read_timeout_addr_r(void)
{
	return 0x2534;
}
static inline u32 host1x_sync_ip_write_timeout_addr_r(void)
{
	return 0x253c;
}
static inline u32 host1x_sync_mlock_0_r(void)
{
	return 0x2cc0;
}
static inline u32 host1x_sync_mlock_owner_0_r(void)
{
	return 0x2d00;
}
static inline u32 host1x_sync_mlock_owner_0_mlock_owner_chid_0_v(u32 r)
{
	return (r >> 8) & 0x3f;
}
static inline u32 host1x_sync_mlock_owner_0_mlock_cpu_owns_0_v(u32 r)
{
	return (r >> 1) & 0x1;
}
static inline u32 host1x_sync_mlock_owner_0_mlock_ch_owns_0_v(u32 r)
{
	return (r >> 0) & 0x1;
}
static inline u32 host1x_sync_syncpt_base_0_r(void)
{
	return 0x18000;
}
static inline u32 host1x_sync_cfpeek_ctrl_r(void)
{
	return 0x261c;
}
static inline u32 host1x_sync_cfpeek_ctrl_cfpeek_addr_f(u32 v)
{
	return (v & 0xfff) << 0;
}
static inline u32 host1x_sync_cfpeek_ctrl_cfpeek_addr_v(u32 r)
{
	return (r >> 0) & 0xfff;
}
static inline u32 host1x_sync_cfpeek_ctrl_cfpeek_channr_f(u32 v)
{
	return (v & 0x3f) << 16;
}
static inline u32 host1x_sync_cfpeek_ctrl_cfpeek_channr_v(u32 r)
{
	return (r >> 16) & 0x3f;
}
static inline u32 host1x_sync_cfpeek_ctrl_cfpeek_ena_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 host1x_sync_cfpeek_read_r(void)
{
	return 0x2620;
}
static inline u32 host1x_sync_cfpeek_ptrs_r(void)
{
	return 0x2624;
}
static inline u32 host1x_sync_cfpeek_ptrs_cf_rd_ptr_v(u32 r)
{
	return (r >> 0) & 0xfff;
}
static inline u32 host1x_sync_cfpeek_ptrs_cf_wr_ptr_v(u32 r)
{
	return (r >> 16) & 0xfff;
}
static inline u32 host1x_sync_cf0_setup_r(void)
{
	return 0x2868;
}
static inline u32 host1x_sync_cf0_setup_cf0_base_v(u32 r)
{
	return (r >> 0) & 0xfff;
}
static inline u32 host1x_sync_cf0_setup_cf0_limit_v(u32 r)
{
	return (r >> 16) & 0xfff;
}
static inline u32 host1x_sync_cbread_r(void)
{
	return 0x28;
}
static inline u32 host1x_sync_cboffset_r(void)
{
	return 0x30;
}
static inline u32 host1x_sync_cbclass_r(void)
{
	return 0x34;
}
static inline u32 host1x_sync_syncpt_thresh_cpu0_int_status_r(void)
{
	return 0x54;
}
static inline u32 host1x_sync_syncpt_thresh_cpu1_int_status_r(void)
{
	return 0xb8;
}
static inline u32 host1x_sync_syncpt_thresh_int_disable_r(void)
{
	return 0x1e4;
}
static inline u32 host1x_sync_syncpt_thresh_int_enable_cpu0_r(void)
{
	return 0x248;
}
static inline u32 host1x_sync_syncpt_intgmask_r(void)
{
	return 0x50;
}
static inline u32 host1x_sync_syncpt_cpu_incr_r(void)
{
	return 0x310;
}
static inline u32 host1x_sync_syncpt_0_r(void)
{
	return 0x18080;
}
static inline u32 host1x_sync_syncpt_int_thresh_0_r(void)
{
	return 0x18a00;
}
#endif
