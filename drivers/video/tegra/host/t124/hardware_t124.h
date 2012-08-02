/*
 * drivers/video/tegra/host/t124/hardware_t124.h
 *
 * Tegra T124 HOST1X Register Definitions
 *
 * Copyright (c) 2011, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __NVHOST_HARDWARE_T124_H
#define __NVHOST_HARDWARE_T124_H

#include "host1x/hw_host1x04_sync.h"
#include "host1x/hw_host1x04_uclass.h"
#include "host1x/hw_host1x04_channel.h"

#define NV_HOST1X_CHANNELS	9
#define NV_HOST1X_SYNC_MLOCK_NUM 16

/* sync registers */
#define NV_HOST1X_SYNCPT_NB_PTS 192
#define NV_HOST1X_SYNCPT_NB_BASES 64
#define NV_HOST1X_NB_MLOCKS 16
#define HOST1X_CHANNEL_SYNC_REG_BASE 0x3000
#define NV_HOST1X_CHANNEL_MAP_SIZE_BYTES 16384

/* Faked for now until the tool can handle mobile register def files*/

static inline u32 host1x_sync_intmask_0_r(void) { return 0x04; }
static inline u32 host1x_sync_intc0mask_0_r(void) { return 0x08; }
static inline u32 host1x_sync_hintstatus_0_r(void) { return 0x20; }
static inline u32 host1x_sync_hintmask_0_r(void) { return 0x24; }
static inline u32 host1x_sync_hintstatus_ext_0_r(void) { return 0x28; }
static inline u32 host1x_sync_hintmask_ext_0_r(void) { return 0x2c; }
static inline u32 host1x_sync_syncpt_thresh_cpu0_int_status_0_r(void) { return 0x40; }
static inline u32 host1x_sync_syncpt_thresh_cpu0_int_status_1_0_r(void) { return 0x44; } /* thru ... */
static inline u32 host1x_sync_syncpt_thresh_cpu0_int_status_5_0_r(void) { return 0x54; }
static inline u32 host1x_sync_syncpt_thresh_cpu1_int_status_0_r(void) { return 0x58; }
static inline u32 host1x_sync_syncpt_thresh_cpu1_int_status_1_0_r(void) { return 0x5c; } /* thru ... */
static inline u32 host1x_sync_syncpt_thresh_cpu1_int_status_5_0_r(void) { return 0x6c; }
static inline u32 host1x_sync_syncpt_thresh_int_disable_0_r(void) { return 0xa0; }
static inline u32 host1x_sync_syncpt_thresh_int_disable_1_0_r(void) { return 0xa4; } /* thru ... */
static inline u32 host1x_sync_syncpt_thresh_int_disable_5_0_r(void) { return 0xb4; }
static inline u32 host1x_sync_syncpt_thresh_int_enable_cpu0_0_r(void) { return 0xb8; }
static inline u32 host1x_sync_syncpt_thresh_int_enable_cpu0_1_0_r(void) { return 0xbc; } /* thru ... */
static inline u32 host1x_sync_syncpt_thresh_int_enable_cpu0_5_0_r(void) { return 0xcc; }
static inline u32 host1x_sync_usec_clk_0_r(void) { return 0x244; }
static inline u32 host1x_sync_ctxsw_timeout_cfg_0_r(void) { return 0x248; }
static inline u32 host1x_sync_ip_busy_timeout_0_r(void) { return 0x25c; }
static inline u32 host1x_sync_ip_read_timeout_addr_0_r(void) { return 0x260; }
static inline u32 host1x_sync_ip_write_timeout_addr_0_r(void) { return 0x264; }
static inline u32 host1x_sync_mlock_0_0_r(void) { return 0x380; }
static inline u32 host1x_sync_mlock_1_0_r(void) { return 0x384; } /* thru ... */
static inline u32 host1x_sync_mlock_15_0_r(void) { return 0x3bc; }
static inline u32 host1x_sync_mlock_owner_0_0_r(void) { return 0x400; }
static inline u32 host1x_sync_mlock_owner_1_0_r(void) { return 0x404; } /* thru ... */
static inline u32 host1x_sync_mlock_owner_15_0_r(void) { return 0x43c; }
static inline u32 host1x_sync_syncpt_0_0_r(void) { return 0x500; }
static inline u32 host1x_sync_syncpt_1_0_r(void) { return 0x504; } /* thru ... */
static inline u32 host1x_sync_syncpt_191_0_r(void) { return 0x7fc; }
static inline u32 host1x_sync_syncpt_int_thresh_0_0_r(void) { return 0x800; }
static inline u32 host1x_sync_syncpt_int_thresh_1_0_r(void) { return 0x804; } /* thru ... */
static inline u32 host1x_sync_syncpt_int_thresh_191_0_r(void) { return 0xafc; }
static inline u32 host1x_sync_syncpt_base_0_0_r(void) { return 0xb00; }
static inline u32 host1x_sync_syncpt_base_1_0_r(void) { return 0xb04; } /* thru... */
static inline u32 host1x_sync_syncpt_base_63_0_r(void) { return 0xbfc; }
static inline u32 host1x_sync_syncpt_cpu_incr_0_0_r(void) { return 0xc00; }
static inline u32 host1x_sync_syncpt_cpu_incr_1_0_r(void) { return 0xc04; } /* thru ... */
static inline u32 host1x_sync_syncpt_cpu_incr_5_0_r(void) { return 0xc14; }

/* Switch to bitfield defs when the tools work */
static inline bool nvhost_sync_hintstatus_ext_ip_read_int(u32 reg)
{
	return (reg & BIT(30)) != 0;
}

static inline bool nvhost_sync_hintstatus_ext_ip_write_int(u32 reg)
{
	return (reg & BIT(31)) != 0;
}

static inline bool nvhost_sync_mlock_owner_ch_owns(u32 reg)
{
	return (reg & BIT(0)) != 0;
}

static inline bool nvhost_sync_mlock_owner_cpu_owns(u32 reg)
{
	return (reg & BIT(1)) != 0;
}

static inline unsigned int nvhost_sync_mlock_owner_owner_chid(u32 reg)
{
	return (reg >> 8) & 0xf;
}

/* Generic support */
static inline u32 nvhost_class_host_wait_syncpt(
	unsigned indx, unsigned threshold)
{
	return (indx << 24) | (threshold & 0xffffff);
}

static inline u32 nvhost_class_host_load_syncpt_base(
	unsigned indx, unsigned threshold)
{
	return host1x_uclass_wait_syncpt_indx_f(indx)
		| host1x_uclass_wait_syncpt_thresh_f(threshold);
}

static inline u32 nvhost_class_host_wait_syncpt_base(
	unsigned indx, unsigned base_indx, unsigned offset)
{
	return host1x_uclass_wait_syncpt_base_indx_f(indx)
		| host1x_uclass_wait_syncpt_base_base_indx_f(base_indx)
		| host1x_uclass_wait_syncpt_base_offset_f(offset);
}

static inline u32 nvhost_class_host_incr_syncpt_base(
	unsigned base_indx, unsigned offset)
{
	return host1x_uclass_incr_syncpt_base_base_indx_f(base_indx)
		| host1x_uclass_incr_syncpt_base_offset_f(offset);
}

static inline u32 nvhost_class_host_incr_syncpt(
	unsigned cond, unsigned indx)
{
	return host1x_uclass_incr_syncpt_cond_f(cond)
		| host1x_uclass_incr_syncpt_indx_f(indx);
}

enum {
	NV_HOST_MODULE_HOST1X = 0,
	NV_HOST_MODULE_MPE = 1,
	NV_HOST_MODULE_GR3D = 6
};

static inline u32 nvhost_class_host_indoff_reg_write(
	unsigned mod_id, unsigned offset, bool auto_inc)
{
	u32 v = host1x_uclass_indoff_indbe_f(0xf)
		| host1x_uclass_indoff_indmodid_f(mod_id)
		| host1x_uclass_indoff_indroffset_f(offset);
	if (auto_inc)
		v |= host1x_uclass_indoff_autoinc_f(1);
	return v;
}

static inline u32 nvhost_class_host_indoff_reg_read(
	unsigned mod_id, unsigned offset, bool auto_inc)
{
	u32 v = host1x_uclass_indoff_indmodid_f(mod_id)
		| host1x_uclass_indoff_indroffset_f(offset)
		| host1x_uclass_indoff_rwn_read_v();
	if (auto_inc)
		v |= host1x_uclass_indoff_autoinc_f(1);
	return v;
}

/* cdma opcodes */
static inline u32 nvhost_opcode_setclass(
	unsigned class_id, unsigned offset, unsigned mask)
{
	return (0 << 28) | (offset << 16) | (class_id << 6) | mask;
}

static inline u32 nvhost_opcode_incr(unsigned offset, unsigned count)
{
	return (1 << 28) | (offset << 16) | count;
}

static inline u32 nvhost_opcode_nonincr(unsigned offset, unsigned count)
{
	return (2 << 28) | (offset << 16) | count;
}

static inline u32 nvhost_opcode_mask(unsigned offset, unsigned mask)
{
	return (3 << 28) | (offset << 16) | mask;
}

static inline u32 nvhost_opcode_imm(unsigned offset, unsigned value)
{
	return (4 << 28) | (offset << 16) | value;
}

static inline u32 nvhost_opcode_imm_incr_syncpt(unsigned cond, unsigned indx)
{
	return nvhost_opcode_imm(host1x_uclass_incr_syncpt_r(),
		nvhost_class_host_incr_syncpt(cond, indx));
}

static inline u32 nvhost_opcode_restart(unsigned address)
{
	return (5 << 28) | (address >> 4);
}

static inline u32 nvhost_opcode_gather(unsigned count)
{
	return (6 << 28) | count;
}

static inline u32 nvhost_opcode_gather_nonincr(unsigned offset,	unsigned count)
{
	return (6 << 28) | (offset << 16) | BIT(15) | count;
}

static inline u32 nvhost_opcode_gather_incr(unsigned offset, unsigned count)
{
	return (6 << 28) | (offset << 16) | BIT(15) | BIT(14) | count;
}

#define NVHOST_OPCODE_NOOP nvhost_opcode_nonincr(0, 0)

static inline u32 nvhost_mask2(unsigned x, unsigned y)
{
	return 1 | (1 << (y - x));
}

#endif /* __NVHOST_HARDWARE_T124_H */
