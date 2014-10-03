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
#ifndef _hw_win_nvdisp_h_
#define _hw_win_nvdisp_h_

static inline u32 win_base_addr_dc_winc_r(void)
{
	return 0x00000500;
}
static inline u32 win_base_addr_dc_win_r(void)
{
	return 0x00000700;
}
static inline u32 win_base_addr_dc_winbuf_r(void)
{
	return 0x00000800;
}
static inline u32 win_base_addr_dc_a_winc_r(void)
{
	return 0x00000a00;
}
static inline u32 win_base_addr_dc_a_win_r(void)
{
	return 0x00000b80;
}
static inline u32 win_base_addr_dc_a_winbuf_r(void)
{
	return 0x00000bc0;
}
static inline u32 win_base_addr_dc_b_winc_r(void)
{
	return 0x00000d00;
}
static inline u32 win_base_addr_dc_b_win_r(void)
{
	return 0x00000e80;
}
static inline u32 win_base_addr_dc_b_winbuf_r(void)
{
	return 0x00000ec0;
}
static inline u32 win_base_addr_dc_c_winc_r(void)
{
	return 0x00001000;
}
static inline u32 win_base_addr_dc_c_win_r(void)
{
	return 0x00001180;
}
static inline u32 win_base_addr_dc_c_winbuf_r(void)
{
	return 0x000011c0;
}
static inline u32 win_base_addr_dc_d_winc_r(void)
{
	return 0x00001300;
}
static inline u32 win_base_addr_dc_d_win_r(void)
{
	return 0x00001480;
}
static inline u32 win_base_addr_dc_d_winbuf_r(void)
{
	return 0x000014c0;
}
static inline u32 win_base_addr_dc_e_winc_r(void)
{
	return 0x00001600;
}
static inline u32 win_base_addr_dc_e_win_r(void)
{
	return 0x00001780;
}
static inline u32 win_base_addr_dc_e_winbuf_r(void)
{
	return 0x000017c0;
}
static inline u32 win_base_addr_dc_f_winc_r(void)
{
	return 0x00001900;
}
static inline u32 win_base_addr_dc_f_win_r(void)
{
	return 0x00001a80;
}
static inline u32 win_base_addr_dc_f_winbuf_r(void)
{
	return 0x00001ac0;
}
#endif
