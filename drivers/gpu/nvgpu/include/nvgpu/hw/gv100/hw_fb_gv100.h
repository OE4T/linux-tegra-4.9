/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_fb_gv100_h_
#define _hw_fb_gv100_h_

static inline u32 fb_fbhub_num_active_ltcs_r(void)
{
	return 0x00100800;
}
static inline u32 fb_mmu_ctrl_r(void)
{
	return 0x00100c80;
}
static inline u32 fb_mmu_ctrl_vm_pg_size_f(u32 v)
{
	return (v & 0x1) << 0;
}
static inline u32 fb_mmu_ctrl_vm_pg_size_128kb_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_ctrl_vm_pg_size_64kb_f(void)
{
	return 0x1;
}
static inline u32 fb_mmu_ctrl_pri_fifo_empty_v(u32 r)
{
	return (r >> 15) & 0x1;
}
static inline u32 fb_mmu_ctrl_pri_fifo_empty_false_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_ctrl_pri_fifo_space_v(u32 r)
{
	return (r >> 16) & 0xff;
}
static inline u32 fb_mmu_ctrl_use_pdb_big_page_size_v(u32 r)
{
	return (r >> 11) & 0x1;
}
static inline u32 fb_mmu_ctrl_use_pdb_big_page_size_true_f(void)
{
	return 0x800;
}
static inline u32 fb_mmu_ctrl_use_pdb_big_page_size_false_f(void)
{
	return 0x0;
}
static inline u32 fb_priv_mmu_phy_secure_r(void)
{
	return 0x00100ce4;
}
static inline u32 fb_mmu_invalidate_pdb_r(void)
{
	return 0x00100cb8;
}
static inline u32 fb_mmu_invalidate_pdb_aperture_vid_mem_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_invalidate_pdb_aperture_sys_mem_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_invalidate_pdb_addr_f(u32 v)
{
	return (v & 0xfffffff) << 4;
}
static inline u32 fb_mmu_invalidate_r(void)
{
	return 0x00100cbc;
}
static inline u32 fb_mmu_invalidate_all_va_true_f(void)
{
	return 0x1;
}
static inline u32 fb_mmu_invalidate_all_pdb_true_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_s(void)
{
	return 1;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_f(u32 v)
{
	return (v & 0x1) << 2;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_m(void)
{
	return 0x1 << 2;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_v(u32 r)
{
	return (r >> 2) & 0x1;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_true_f(void)
{
	return 0x4;
}
static inline u32 fb_mmu_invalidate_replay_s(void)
{
	return 3;
}
static inline u32 fb_mmu_invalidate_replay_f(u32 v)
{
	return (v & 0x7) << 3;
}
static inline u32 fb_mmu_invalidate_replay_m(void)
{
	return 0x7 << 3;
}
static inline u32 fb_mmu_invalidate_replay_v(u32 r)
{
	return (r >> 3) & 0x7;
}
static inline u32 fb_mmu_invalidate_replay_none_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_invalidate_replay_start_f(void)
{
	return 0x8;
}
static inline u32 fb_mmu_invalidate_replay_start_ack_all_f(void)
{
	return 0x10;
}
static inline u32 fb_mmu_invalidate_replay_cancel_global_f(void)
{
	return 0x20;
}
static inline u32 fb_mmu_invalidate_sys_membar_s(void)
{
	return 1;
}
static inline u32 fb_mmu_invalidate_sys_membar_f(u32 v)
{
	return (v & 0x1) << 6;
}
static inline u32 fb_mmu_invalidate_sys_membar_m(void)
{
	return 0x1 << 6;
}
static inline u32 fb_mmu_invalidate_sys_membar_v(u32 r)
{
	return (r >> 6) & 0x1;
}
static inline u32 fb_mmu_invalidate_sys_membar_true_f(void)
{
	return 0x40;
}
static inline u32 fb_mmu_invalidate_ack_s(void)
{
	return 2;
}
static inline u32 fb_mmu_invalidate_ack_f(u32 v)
{
	return (v & 0x3) << 7;
}
static inline u32 fb_mmu_invalidate_ack_m(void)
{
	return 0x3 << 7;
}
static inline u32 fb_mmu_invalidate_ack_v(u32 r)
{
	return (r >> 7) & 0x3;
}
static inline u32 fb_mmu_invalidate_ack_ack_none_required_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_invalidate_ack_ack_intranode_f(void)
{
	return 0x100;
}
static inline u32 fb_mmu_invalidate_ack_ack_globally_f(void)
{
	return 0x80;
}
static inline u32 fb_mmu_invalidate_cancel_client_id_s(void)
{
	return 6;
}
static inline u32 fb_mmu_invalidate_cancel_client_id_f(u32 v)
{
	return (v & 0x3f) << 9;
}
static inline u32 fb_mmu_invalidate_cancel_client_id_m(void)
{
	return 0x3f << 9;
}
static inline u32 fb_mmu_invalidate_cancel_client_id_v(u32 r)
{
	return (r >> 9) & 0x3f;
}
static inline u32 fb_mmu_invalidate_cancel_gpc_id_s(void)
{
	return 5;
}
static inline u32 fb_mmu_invalidate_cancel_gpc_id_f(u32 v)
{
	return (v & 0x1f) << 15;
}
static inline u32 fb_mmu_invalidate_cancel_gpc_id_m(void)
{
	return 0x1f << 15;
}
static inline u32 fb_mmu_invalidate_cancel_gpc_id_v(u32 r)
{
	return (r >> 15) & 0x1f;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_s(void)
{
	return 1;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_f(u32 v)
{
	return (v & 0x1) << 20;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_m(void)
{
	return 0x1 << 20;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_v(u32 r)
{
	return (r >> 20) & 0x1;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_gpc_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_hub_f(void)
{
	return 0x100000;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_s(void)
{
	return 3;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_f(u32 v)
{
	return (v & 0x7) << 24;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_m(void)
{
	return 0x7 << 24;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_v(u32 r)
{
	return (r >> 24) & 0x7;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_all_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_pte_only_f(void)
{
	return 0x1000000;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde0_f(void)
{
	return 0x2000000;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde1_f(void)
{
	return 0x3000000;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde2_f(void)
{
	return 0x4000000;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde3_f(void)
{
	return 0x5000000;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde4_f(void)
{
	return 0x6000000;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde5_f(void)
{
	return 0x7000000;
}
static inline u32 fb_mmu_invalidate_trigger_s(void)
{
	return 1;
}
static inline u32 fb_mmu_invalidate_trigger_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 fb_mmu_invalidate_trigger_m(void)
{
	return 0x1 << 31;
}
static inline u32 fb_mmu_invalidate_trigger_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 fb_mmu_invalidate_trigger_true_f(void)
{
	return 0x80000000;
}
static inline u32 fb_mmu_debug_wr_r(void)
{
	return 0x00100cc8;
}
static inline u32 fb_mmu_debug_wr_aperture_s(void)
{
	return 2;
}
static inline u32 fb_mmu_debug_wr_aperture_f(u32 v)
{
	return (v & 0x3) << 0;
}
static inline u32 fb_mmu_debug_wr_aperture_m(void)
{
	return 0x3 << 0;
}
static inline u32 fb_mmu_debug_wr_aperture_v(u32 r)
{
	return (r >> 0) & 0x3;
}
static inline u32 fb_mmu_debug_wr_aperture_vid_mem_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_debug_wr_aperture_sys_mem_coh_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_debug_wr_aperture_sys_mem_ncoh_f(void)
{
	return 0x3;
}
static inline u32 fb_mmu_debug_wr_vol_false_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_debug_wr_vol_true_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_debug_wr_vol_true_f(void)
{
	return 0x4;
}
static inline u32 fb_mmu_debug_wr_addr_f(u32 v)
{
	return (v & 0xfffffff) << 4;
}
static inline u32 fb_mmu_debug_wr_addr_alignment_v(void)
{
	return 0x0000000c;
}
static inline u32 fb_mmu_debug_rd_r(void)
{
	return 0x00100ccc;
}
static inline u32 fb_mmu_debug_rd_aperture_vid_mem_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_debug_rd_aperture_sys_mem_coh_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_debug_rd_aperture_sys_mem_ncoh_f(void)
{
	return 0x3;
}
static inline u32 fb_mmu_debug_rd_vol_false_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_debug_rd_addr_f(u32 v)
{
	return (v & 0xfffffff) << 4;
}
static inline u32 fb_mmu_debug_rd_addr_alignment_v(void)
{
	return 0x0000000c;
}
static inline u32 fb_mmu_debug_ctrl_r(void)
{
	return 0x00100cc4;
}
static inline u32 fb_mmu_debug_ctrl_debug_v(u32 r)
{
	return (r >> 16) & 0x1;
}
static inline u32 fb_mmu_debug_ctrl_debug_m(void)
{
	return 0x1 << 16;
}
static inline u32 fb_mmu_debug_ctrl_debug_enabled_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_debug_ctrl_debug_disabled_v(void)
{
	return 0x00000000;
}
static inline u32 fb_mmu_vpr_info_r(void)
{
	return 0x00100cd0;
}
static inline u32 fb_mmu_vpr_info_fetch_v(u32 r)
{
	return (r >> 2) & 0x1;
}
static inline u32 fb_mmu_vpr_info_fetch_false_v(void)
{
	return 0x00000000;
}
static inline u32 fb_mmu_vpr_info_fetch_true_v(void)
{
	return 0x00000001;
}
static inline u32 fb_niso_flush_sysmem_addr_r(void)
{
	return 0x00100c10;
}
static inline u32 fb_niso_intr_r(void)
{
	return 0x00100a20;
}
static inline u32 fb_niso_intr_hub_access_counter_notify_m(void)
{
	return 0x1 << 0;
}
static inline u32 fb_niso_intr_hub_access_counter_notify_pending_f(void)
{
	return 0x1;
}
static inline u32 fb_niso_intr_hub_access_counter_error_m(void)
{
	return 0x1 << 1;
}
static inline u32 fb_niso_intr_hub_access_counter_error_pending_f(void)
{
	return 0x2;
}
static inline u32 fb_niso_intr_mmu_replayable_fault_notify_m(void)
{
	return 0x1 << 27;
}
static inline u32 fb_niso_intr_mmu_replayable_fault_notify_pending_f(void)
{
	return 0x8000000;
}
static inline u32 fb_niso_intr_mmu_replayable_fault_overflow_m(void)
{
	return 0x1 << 28;
}
static inline u32 fb_niso_intr_mmu_replayable_fault_overflow_pending_f(void)
{
	return 0x10000000;
}
static inline u32 fb_niso_intr_mmu_nonreplayable_fault_notify_m(void)
{
	return 0x1 << 29;
}
static inline u32 fb_niso_intr_mmu_nonreplayable_fault_notify_pending_f(void)
{
	return 0x20000000;
}
static inline u32 fb_niso_intr_mmu_nonreplayable_fault_overflow_m(void)
{
	return 0x1 << 30;
}
static inline u32 fb_niso_intr_mmu_nonreplayable_fault_overflow_pending_f(void)
{
	return 0x40000000;
}
static inline u32 fb_niso_intr_mmu_other_fault_notify_m(void)
{
	return 0x1 << 31;
}
static inline u32 fb_niso_intr_mmu_other_fault_notify_pending_f(void)
{
	return 0x80000000;
}
static inline u32 fb_niso_intr_en_r(u32 i)
{
	return 0x00100a24 + i*4;
}
static inline u32 fb_niso_intr_en__size_1_v(void)
{
	return 0x00000002;
}
static inline u32 fb_niso_intr_en_hub_access_counter_notify_f(u32 v)
{
	return (v & 0x1) << 0;
}
static inline u32 fb_niso_intr_en_hub_access_counter_notify_enabled_f(void)
{
	return 0x1;
}
static inline u32 fb_niso_intr_en_hub_access_counter_error_f(u32 v)
{
	return (v & 0x1) << 1;
}
static inline u32 fb_niso_intr_en_hub_access_counter_error_enabled_f(void)
{
	return 0x2;
}
static inline u32 fb_niso_intr_en_mmu_replayable_fault_notify_f(u32 v)
{
	return (v & 0x1) << 27;
}
static inline u32 fb_niso_intr_en_mmu_replayable_fault_notify_enabled_f(void)
{
	return 0x8000000;
}
static inline u32 fb_niso_intr_en_mmu_replayable_fault_overflow_f(u32 v)
{
	return (v & 0x1) << 28;
}
static inline u32 fb_niso_intr_en_mmu_replayable_fault_overflow_enabled_f(void)
{
	return 0x10000000;
}
static inline u32 fb_niso_intr_en_mmu_nonreplayable_fault_notify_f(u32 v)
{
	return (v & 0x1) << 29;
}
static inline u32 fb_niso_intr_en_mmu_nonreplayable_fault_notify_enabled_f(void)
{
	return 0x20000000;
}
static inline u32 fb_niso_intr_en_mmu_nonreplayable_fault_overflow_f(u32 v)
{
	return (v & 0x1) << 30;
}
static inline u32 fb_niso_intr_en_mmu_nonreplayable_fault_overflow_enabled_f(void)
{
	return 0x40000000;
}
static inline u32 fb_niso_intr_en_mmu_other_fault_notify_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 fb_niso_intr_en_mmu_other_fault_notify_enabled_f(void)
{
	return 0x80000000;
}
static inline u32 fb_niso_intr_en_set_r(u32 i)
{
	return 0x00100a2c + i*4;
}
static inline u32 fb_niso_intr_en_set__size_1_v(void)
{
	return 0x00000002;
}
static inline u32 fb_niso_intr_en_set_hub_access_counter_notify_m(void)
{
	return 0x1 << 0;
}
static inline u32 fb_niso_intr_en_set_hub_access_counter_notify_set_f(void)
{
	return 0x1;
}
static inline u32 fb_niso_intr_en_set_hub_access_counter_error_m(void)
{
	return 0x1 << 1;
}
static inline u32 fb_niso_intr_en_set_hub_access_counter_error_set_f(void)
{
	return 0x2;
}
static inline u32 fb_niso_intr_en_set_mmu_replayable_fault_notify_m(void)
{
	return 0x1 << 27;
}
static inline u32 fb_niso_intr_en_set_mmu_replayable_fault_notify_set_f(void)
{
	return 0x8000000;
}
static inline u32 fb_niso_intr_en_set_mmu_replayable_fault_overflow_m(void)
{
	return 0x1 << 28;
}
static inline u32 fb_niso_intr_en_set_mmu_replayable_fault_overflow_set_f(void)
{
	return 0x10000000;
}
static inline u32 fb_niso_intr_en_set_mmu_nonreplayable_fault_notify_m(void)
{
	return 0x1 << 29;
}
static inline u32 fb_niso_intr_en_set_mmu_nonreplayable_fault_notify_set_f(void)
{
	return 0x20000000;
}
static inline u32 fb_niso_intr_en_set_mmu_nonreplayable_fault_overflow_m(void)
{
	return 0x1 << 30;
}
static inline u32 fb_niso_intr_en_set_mmu_nonreplayable_fault_overflow_set_f(void)
{
	return 0x40000000;
}
static inline u32 fb_niso_intr_en_set_mmu_other_fault_notify_m(void)
{
	return 0x1 << 31;
}
static inline u32 fb_niso_intr_en_set_mmu_other_fault_notify_set_f(void)
{
	return 0x80000000;
}
static inline u32 fb_niso_intr_en_clr_r(u32 i)
{
	return 0x00100a34 + i*4;
}
static inline u32 fb_niso_intr_en_clr__size_1_v(void)
{
	return 0x00000002;
}
static inline u32 fb_niso_intr_en_clr_hub_access_counter_notify_m(void)
{
	return 0x1 << 0;
}
static inline u32 fb_niso_intr_en_clr_hub_access_counter_notify_set_f(void)
{
	return 0x1;
}
static inline u32 fb_niso_intr_en_clr_hub_access_counter_error_m(void)
{
	return 0x1 << 1;
}
static inline u32 fb_niso_intr_en_clr_hub_access_counter_error_set_f(void)
{
	return 0x2;
}
static inline u32 fb_niso_intr_en_clr_mmu_replayable_fault_notify_m(void)
{
	return 0x1 << 27;
}
static inline u32 fb_niso_intr_en_clr_mmu_replayable_fault_notify_set_f(void)
{
	return 0x8000000;
}
static inline u32 fb_niso_intr_en_clr_mmu_replayable_fault_overflow_m(void)
{
	return 0x1 << 28;
}
static inline u32 fb_niso_intr_en_clr_mmu_replayable_fault_overflow_set_f(void)
{
	return 0x10000000;
}
static inline u32 fb_niso_intr_en_clr_mmu_nonreplayable_fault_notify_m(void)
{
	return 0x1 << 29;
}
static inline u32 fb_niso_intr_en_clr_mmu_nonreplayable_fault_notify_set_f(void)
{
	return 0x20000000;
}
static inline u32 fb_niso_intr_en_clr_mmu_nonreplayable_fault_overflow_m(void)
{
	return 0x1 << 30;
}
static inline u32 fb_niso_intr_en_clr_mmu_nonreplayable_fault_overflow_set_f(void)
{
	return 0x40000000;
}
static inline u32 fb_niso_intr_en_clr_mmu_other_fault_notify_m(void)
{
	return 0x1 << 31;
}
static inline u32 fb_niso_intr_en_clr_mmu_other_fault_notify_set_f(void)
{
	return 0x80000000;
}
static inline u32 fb_niso_intr_en_clr_mmu_non_replay_fault_buffer_v(void)
{
	return 0x00000000;
}
static inline u32 fb_niso_intr_en_clr_mmu_replay_fault_buffer_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_buffer_lo_r(u32 i)
{
	return 0x00100e24 + i*20;
}
static inline u32 fb_mmu_fault_buffer_lo__size_1_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_f(u32 v)
{
	return (v & 0x1) << 0;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_v(u32 r)
{
	return (r >> 0) & 0x1;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_virtual_v(void)
{
	return 0x00000000;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_virtual_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_physical_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_physical_f(void)
{
	return 0x1;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_f(u32 v)
{
	return (v & 0x3) << 1;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_v(u32 r)
{
	return (r >> 1) & 0x3;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_sys_coh_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_sys_coh_f(void)
{
	return 0x4;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_sys_nocoh_v(void)
{
	return 0x00000003;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_sys_nocoh_f(void)
{
	return 0x6;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_vol_f(u32 v)
{
	return (v & 0x1) << 3;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_vol_v(u32 r)
{
	return (r >> 3) & 0x1;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_f(u32 v)
{
	return (v & 0xfffff) << 12;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_v(u32 r)
{
	return (r >> 12) & 0xfffff;
}
static inline u32 fb_mmu_fault_buffer_hi_r(u32 i)
{
	return 0x00100e28 + i*20;
}
static inline u32 fb_mmu_fault_buffer_hi__size_1_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_fault_buffer_hi_addr_f(u32 v)
{
	return (v & 0xffffffff) << 0;
}
static inline u32 fb_mmu_fault_buffer_hi_addr_v(u32 r)
{
	return (r >> 0) & 0xffffffff;
}
static inline u32 fb_mmu_fault_buffer_get_r(u32 i)
{
	return 0x00100e2c + i*20;
}
static inline u32 fb_mmu_fault_buffer_get__size_1_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_fault_buffer_get_ptr_f(u32 v)
{
	return (v & 0xfffff) << 0;
}
static inline u32 fb_mmu_fault_buffer_get_ptr_m(void)
{
	return 0xfffff << 0;
}
static inline u32 fb_mmu_fault_buffer_get_ptr_v(u32 r)
{
	return (r >> 0) & 0xfffff;
}
static inline u32 fb_mmu_fault_buffer_get_getptr_corrupted_f(u32 v)
{
	return (v & 0x1) << 30;
}
static inline u32 fb_mmu_fault_buffer_get_getptr_corrupted_m(void)
{
	return 0x1 << 30;
}
static inline u32 fb_mmu_fault_buffer_get_getptr_corrupted_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_buffer_get_getptr_corrupted_clear_f(void)
{
	return 0x40000000;
}
static inline u32 fb_mmu_fault_buffer_get_overflow_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 fb_mmu_fault_buffer_get_overflow_m(void)
{
	return 0x1 << 31;
}
static inline u32 fb_mmu_fault_buffer_get_overflow_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_buffer_get_overflow_clear_f(void)
{
	return 0x80000000;
}
static inline u32 fb_mmu_fault_buffer_put_r(u32 i)
{
	return 0x00100e30 + i*20;
}
static inline u32 fb_mmu_fault_buffer_put__size_1_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_fault_buffer_put_ptr_f(u32 v)
{
	return (v & 0xfffff) << 0;
}
static inline u32 fb_mmu_fault_buffer_put_ptr_v(u32 r)
{
	return (r >> 0) & 0xfffff;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_f(u32 v)
{
	return (v & 0x1) << 30;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_v(u32 r)
{
	return (r >> 30) & 0x1;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_yes_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_yes_f(void)
{
	return 0x40000000;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_no_v(void)
{
	return 0x00000000;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_no_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_fault_buffer_put_overflow_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 fb_mmu_fault_buffer_put_overflow_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 fb_mmu_fault_buffer_put_overflow_yes_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_buffer_put_overflow_yes_f(void)
{
	return 0x80000000;
}
static inline u32 fb_mmu_fault_buffer_size_r(u32 i)
{
	return 0x00100e34 + i*20;
}
static inline u32 fb_mmu_fault_buffer_size__size_1_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_fault_buffer_size_val_f(u32 v)
{
	return (v & 0xfffff) << 0;
}
static inline u32 fb_mmu_fault_buffer_size_val_v(u32 r)
{
	return (r >> 0) & 0xfffff;
}
static inline u32 fb_mmu_fault_buffer_size_overflow_intr_f(u32 v)
{
	return (v & 0x1) << 29;
}
static inline u32 fb_mmu_fault_buffer_size_overflow_intr_v(u32 r)
{
	return (r >> 29) & 0x1;
}
static inline u32 fb_mmu_fault_buffer_size_overflow_intr_enable_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_buffer_size_overflow_intr_enable_f(void)
{
	return 0x20000000;
}
static inline u32 fb_mmu_fault_buffer_size_set_default_f(u32 v)
{
	return (v & 0x1) << 30;
}
static inline u32 fb_mmu_fault_buffer_size_set_default_v(u32 r)
{
	return (r >> 30) & 0x1;
}
static inline u32 fb_mmu_fault_buffer_size_set_default_yes_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_buffer_size_set_default_yes_f(void)
{
	return 0x40000000;
}
static inline u32 fb_mmu_fault_buffer_size_enable_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 fb_mmu_fault_buffer_size_enable_m(void)
{
	return 0x1 << 31;
}
static inline u32 fb_mmu_fault_buffer_size_enable_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 fb_mmu_fault_buffer_size_enable_true_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_buffer_size_enable_true_f(void)
{
	return 0x80000000;
}
static inline u32 fb_mmu_fault_addr_lo_r(void)
{
	return 0x00100e4c;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_f(u32 v)
{
	return (v & 0x3) << 0;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_v(u32 r)
{
	return (r >> 0) & 0x3;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_sys_coh_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_sys_coh_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_sys_nocoh_v(void)
{
	return 0x00000003;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_sys_nocoh_f(void)
{
	return 0x3;
}
static inline u32 fb_mmu_fault_addr_lo_addr_f(u32 v)
{
	return (v & 0xfffff) << 12;
}
static inline u32 fb_mmu_fault_addr_lo_addr_v(u32 r)
{
	return (r >> 12) & 0xfffff;
}
static inline u32 fb_mmu_fault_addr_hi_r(void)
{
	return 0x00100e50;
}
static inline u32 fb_mmu_fault_addr_hi_addr_f(u32 v)
{
	return (v & 0xffffffff) << 0;
}
static inline u32 fb_mmu_fault_addr_hi_addr_v(u32 r)
{
	return (r >> 0) & 0xffffffff;
}
static inline u32 fb_mmu_fault_inst_lo_r(void)
{
	return 0x00100e54;
}
static inline u32 fb_mmu_fault_inst_lo_engine_id_v(u32 r)
{
	return (r >> 0) & 0x1ff;
}
static inline u32 fb_mmu_fault_inst_lo_aperture_v(u32 r)
{
	return (r >> 10) & 0x3;
}
static inline u32 fb_mmu_fault_inst_lo_aperture_sys_coh_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_fault_inst_lo_aperture_sys_nocoh_v(void)
{
	return 0x00000003;
}
static inline u32 fb_mmu_fault_inst_lo_addr_f(u32 v)
{
	return (v & 0xfffff) << 12;
}
static inline u32 fb_mmu_fault_inst_lo_addr_v(u32 r)
{
	return (r >> 12) & 0xfffff;
}
static inline u32 fb_mmu_fault_inst_hi_r(void)
{
	return 0x00100e58;
}
static inline u32 fb_mmu_fault_inst_hi_addr_v(u32 r)
{
	return (r >> 0) & 0xffffffff;
}
static inline u32 fb_mmu_fault_info_r(void)
{
	return 0x00100e5c;
}
static inline u32 fb_mmu_fault_info_fault_type_v(u32 r)
{
	return (r >> 0) & 0x1f;
}
static inline u32 fb_mmu_fault_info_replayable_fault_v(u32 r)
{
	return (r >> 7) & 0x1;
}
static inline u32 fb_mmu_fault_info_client_v(u32 r)
{
	return (r >> 8) & 0x7f;
}
static inline u32 fb_mmu_fault_info_access_type_v(u32 r)
{
	return (r >> 16) & 0xf;
}
static inline u32 fb_mmu_fault_info_client_type_v(u32 r)
{
	return (r >> 20) & 0x1;
}
static inline u32 fb_mmu_fault_info_gpc_id_v(u32 r)
{
	return (r >> 24) & 0x1f;
}
static inline u32 fb_mmu_fault_info_protected_mode_v(u32 r)
{
	return (r >> 29) & 0x1;
}
static inline u32 fb_mmu_fault_info_replayable_fault_en_v(u32 r)
{
	return (r >> 30) & 0x1;
}
static inline u32 fb_mmu_fault_info_valid_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 fb_mmu_fault_status_r(void)
{
	return 0x00100e60;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_m(void)
{
	return 0x1 << 0;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_set_f(void)
{
	return 0x1;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_clear_f(void)
{
	return 0x1;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_m(void)
{
	return 0x1 << 1;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_set_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_clear_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_m(void)
{
	return 0x1 << 2;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_set_f(void)
{
	return 0x4;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_clear_f(void)
{
	return 0x4;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_m(void)
{
	return 0x1 << 3;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_set_f(void)
{
	return 0x8;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_clear_f(void)
{
	return 0x8;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_m(void)
{
	return 0x1 << 4;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_set_f(void)
{
	return 0x10;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_clear_f(void)
{
	return 0x10;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_m(void)
{
	return 0x1 << 5;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_set_f(void)
{
	return 0x20;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_clear_f(void)
{
	return 0x20;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_m(void)
{
	return 0x1 << 6;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_set_f(void)
{
	return 0x40;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_clear_f(void)
{
	return 0x40;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_m(void)
{
	return 0x1 << 7;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_set_f(void)
{
	return 0x80;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_clear_f(void)
{
	return 0x80;
}
static inline u32 fb_mmu_fault_status_replayable_m(void)
{
	return 0x1 << 8;
}
static inline u32 fb_mmu_fault_status_replayable_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_replayable_set_f(void)
{
	return 0x100;
}
static inline u32 fb_mmu_fault_status_replayable_reset_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_fault_status_non_replayable_m(void)
{
	return 0x1 << 9;
}
static inline u32 fb_mmu_fault_status_non_replayable_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_non_replayable_set_f(void)
{
	return 0x200;
}
static inline u32 fb_mmu_fault_status_non_replayable_reset_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_fault_status_replayable_error_m(void)
{
	return 0x1 << 10;
}
static inline u32 fb_mmu_fault_status_replayable_error_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_replayable_error_set_f(void)
{
	return 0x400;
}
static inline u32 fb_mmu_fault_status_replayable_error_reset_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_fault_status_non_replayable_error_m(void)
{
	return 0x1 << 11;
}
static inline u32 fb_mmu_fault_status_non_replayable_error_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_non_replayable_error_set_f(void)
{
	return 0x800;
}
static inline u32 fb_mmu_fault_status_non_replayable_error_reset_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_fault_status_replayable_overflow_m(void)
{
	return 0x1 << 12;
}
static inline u32 fb_mmu_fault_status_replayable_overflow_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_replayable_overflow_set_f(void)
{
	return 0x1000;
}
static inline u32 fb_mmu_fault_status_replayable_overflow_reset_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_fault_status_non_replayable_overflow_m(void)
{
	return 0x1 << 13;
}
static inline u32 fb_mmu_fault_status_non_replayable_overflow_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_non_replayable_overflow_set_f(void)
{
	return 0x2000;
}
static inline u32 fb_mmu_fault_status_non_replayable_overflow_reset_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_fault_status_replayable_getptr_corrupted_m(void)
{
	return 0x1 << 14;
}
static inline u32 fb_mmu_fault_status_replayable_getptr_corrupted_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_replayable_getptr_corrupted_set_f(void)
{
	return 0x4000;
}
static inline u32 fb_mmu_fault_status_non_replayable_getptr_corrupted_m(void)
{
	return 0x1 << 15;
}
static inline u32 fb_mmu_fault_status_non_replayable_getptr_corrupted_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_non_replayable_getptr_corrupted_set_f(void)
{
	return 0x8000;
}
static inline u32 fb_mmu_fault_status_busy_m(void)
{
	return 0x1 << 30;
}
static inline u32 fb_mmu_fault_status_busy_true_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_busy_true_f(void)
{
	return 0x40000000;
}
static inline u32 fb_mmu_fault_status_valid_m(void)
{
	return 0x1 << 31;
}
static inline u32 fb_mmu_fault_status_valid_set_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_valid_set_f(void)
{
	return 0x80000000;
}
static inline u32 fb_mmu_fault_status_valid_clear_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_fault_status_valid_clear_f(void)
{
	return 0x80000000;
}
static inline u32 fb_mmu_local_memory_range_r(void)
{
	return 0x00100ce0;
}
static inline u32 fb_mmu_local_memory_range_lower_scale_v(u32 r)
{
	return (r >> 0) & 0xf;
}
static inline u32 fb_mmu_local_memory_range_lower_mag_v(u32 r)
{
	return (r >> 4) & 0x3f;
}
static inline u32 fb_mmu_local_memory_range_ecc_mode_v(u32 r)
{
	return (r >> 30) & 0x1;
}
static inline u32 fb_niso_scrub_status_r(void)
{
	return 0x00100b20;
}
static inline u32 fb_niso_scrub_status_flag_v(u32 r)
{
	return (r >> 0) & 0x1;
}
static inline u32 fb_mmu_priv_level_mask_r(void)
{
	return 0x00100cdc;
}
static inline u32 fb_mmu_priv_level_mask_write_violation_m(void)
{
	return 0x1 << 7;
}
#endif
