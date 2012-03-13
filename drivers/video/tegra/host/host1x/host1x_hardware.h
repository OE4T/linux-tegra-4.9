/*
 * drivers/video/tegra/host/host1x/host1x_hardware.h
 *
 * Tegra Graphics Host Register Offsets
 *
 * Copyright (c) 2010-2012 NVIDIA Corporation.
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

#ifndef __NVHOST_HOST1X_HOST1X_HARDWARE_H
#define __NVHOST_HOST1X_HOST1X_HARDWARE_H

#include <linux/types.h>
#include <linux/bitops.h>

/* class ids */
enum {
	NV_HOST1X_CLASS_ID = 0x1,
	NV_VIDEO_ENCODE_MPEG_CLASS_ID = 0x20,
	NV_GRAPHICS_3D_CLASS_ID = 0x60
};


/* channel registers */
#define NV_HOST1X_CHANNELS 8
#define NV_HOST1X_CHANNEL0_BASE 0
#define NV_HOST1X_CHANNEL_MAP_SIZE_BYTES 16384
#define NV_HOST1X_SYNC_MLOCK_NUM 16

#define HOST1X_VAL(reg, field, regdata) \
	((regdata >> HOST1X_##reg##_##field##_SHIFT) \
			& HOST1X_##reg##_##field##_MASK)
#define HOST1X_CREATE(reg, field, data) \
	((data & HOST1X_##reg##_##field##_MASK) \
			<< HOST1X_##reg##_##field##_SHIFT) \

#define HOST1X_CHANNEL_FIFOSTAT		0x00
#define HOST1X_CHANNEL_FIFOSTAT_CFEMPTY_SHIFT 10
#define HOST1X_CHANNEL_FIFOSTAT_CFEMPTY_MASK 0x1
#define HOST1X_CHANNEL_FIFOSTAT_OUTFENTRIES_SHIFT 24
#define HOST1X_CHANNEL_FIFOSTAT_OUTFENTRIES_MASK 0x1f
#define HOST1X_CHANNEL_INDDATA		0x0c
#define HOST1X_CHANNEL_DMASTART		0x14
#define HOST1X_CHANNEL_DMAPUT		0x18
#define HOST1X_CHANNEL_DMAGET		0x1c
#define HOST1X_CHANNEL_DMAEND		0x20
#define HOST1X_CHANNEL_DMACTRL		0x24
#define HOST1X_CHANNEL_DMACTRL_DMASTOP_SHIFT 0
#define HOST1X_CHANNEL_DMACTRL_DMASTOP_MASK 0x1
#define HOST1X_CHANNEL_DMACTRL_DMAGETRST_SHIFT 1
#define HOST1X_CHANNEL_DMACTRL_DMAGETRST_MASK 0x1
#define HOST1X_CHANNEL_DMACTRL_DMAINITGET_SHIFT 2
#define HOST1X_CHANNEL_DMACTRL_DMAINITGET_MASK 0x1

#define HOST1X_CHANNEL_SYNC_REG_BASE	0x3000

#define HOST1X_SYNC_INTMASK		0x4
#define HOST1X_SYNC_INTC0MASK		0x8
#define HOST1X_SYNC_HINTSTATUS		0x20
#define HOST1X_SYNC_HINTMASK		0x24
#define HOST1X_SYNC_HINTSTATUS_EXT	0x28
#define HOST1X_SYNC_HINTSTATUS_EXT_IP_READ_INT_SHIFT 30
#define HOST1X_SYNC_HINTSTATUS_EXT_IP_READ_INT_MASK 0x1
#define HOST1X_SYNC_HINTSTATUS_EXT_IP_WRITE_INT_SHIFT 31
#define HOST1X_SYNC_HINTSTATUS_EXT_IP_WRITE_INT_MASK 0x1
#define HOST1X_SYNC_HINTMASK_EXT	0x2c
#define HOST1X_SYNC_SYNCPT_THRESH_CPU0_INT_STATUS 0x40
#define HOST1X_SYNC_SYNCPT_THRESH_CPU1_INT_STATUS 0x48
#define HOST1X_SYNC_SYNCPT_THRESH_INT_DISABLE 0x60
#define HOST1X_SYNC_SYNCPT_THRESH_INT_ENABLE_CPU0 0x68
#define HOST1X_SYNC_CF0_SETUP		0x80
#define HOST1X_SYNC_CF0_SETUP_BASE_SHIFT 0
#define HOST1X_SYNC_CF0_SETUP_BASE_MASK	0x1ff
#define HOST1X_SYNC_CF0_SETUP_LIMIT_SHIFT 16
#define HOST1X_SYNC_CF0_SETUP_LIMIT_MASK 0x1ff
#define HOST1X_SYNC_CFx_SETUP(x)	(HOST1X_SYNC_CF0_SETUP + (4 * (x)))

#define HOST1X_SYNC_CMDPROC_STOP	0xac
#define HOST1X_SYNC_CH_TEARDOWN		0xb0
#define HOST1X_SYNC_USEC_CLK		0x1a4
#define HOST1X_SYNC_CTXSW_TIMEOUT_CFG	0x1a8
#define HOST1X_SYNC_IP_BUSY_TIMEOUT	0x1bc
#define HOST1X_SYNC_IP_READ_TIMEOUT_ADDR 0x1c0
#define HOST1X_SYNC_IP_WRITE_TIMEOUT_ADDR 0x1c4
#define HOST1X_SYNC_MLOCK_0		0x2c0
#define HOST1X_SYNC_MLOCK_OWNER_0	0x340
#define HOST1X_SYNC_MLOCK_OWNER_0_CHID_SHIFT 8
#define HOST1X_SYNC_MLOCK_OWNER_0_CHID_MASK 0xf
#define HOST1X_SYNC_MLOCK_OWNER_0_CPU_OWNS_SHIFT 1
#define HOST1X_SYNC_MLOCK_OWNER_0_CPU_OWNS_MASK 0x1
#define HOST1X_SYNC_MLOCK_OWNER_0_CH_OWNS_SHIFT 0
#define HOST1X_SYNC_MLOCK_OWNER_0_CH_OWNS_MASK 0x1
#define HOST1X_SYNC_SYNCPT_0		0x400
#define HOST1X_SYNC_SYNCPT_INT_THRESH_0	0x500

#define HOST1X_SYNC_SYNCPT_BASE_0	0x600
#define HOST1X_SYNC_SYNCPT_BASE_0_BASE_SHIFT 0
#define HOST1X_SYNC_SYNCPT_BASE_0_BASE_MASK 0xffff
#define HOST1X_SYNC_SYNCPT_BASE_x(x)	(HOST1X_SYNC_SYNCPT_BASE_0 + (4 * (x)))

#define HOST1X_SYNC_SYNCPT_CPU_INCR	0x700

#define HOST1X_SYNC_CBREAD_0		0x720
#define HOST1X_SYNC_CBREAD_x(x)		(HOST1X_SYNC_CBREAD_0 + (4 * (x)))
#define HOST1X_SYNC_CFPEEK_CTRL		0x74c
#define HOST1X_SYNC_CFPEEK_CTRL_ADDR_SHIFT 0
#define HOST1X_SYNC_CFPEEK_CTRL_ADDR_MASK 0x1ff
#define HOST1X_SYNC_CFPEEK_CTRL_CHANNR_SHIFT 16
#define HOST1X_SYNC_CFPEEK_CTRL_CHANNR_MASK 0x7
#define HOST1X_SYNC_CFPEEK_CTRL_ENA_SHIFT 31
#define HOST1X_SYNC_CFPEEK_CTRL_ENA_MASK 0x1
#define HOST1X_SYNC_CFPEEK_READ		0x750
#define HOST1X_SYNC_CFPEEK_PTRS		0x754
#define HOST1X_SYNC_CFPEEK_PTRS_CF_RD_PTR_SHIFT 0
#define HOST1X_SYNC_CFPEEK_PTRS_CF_RD_PTR_MASK 0x1ff
#define HOST1X_SYNC_CFPEEK_PTRS_CF_WR_PTR_SHIFT 16
#define HOST1X_SYNC_CFPEEK_PTRS_CF_WR_PTR_MASK 0x1ff
#define HOST1X_SYNC_CBSTAT_0		0x758
#define HOST1X_SYNC_CBSTAT_0_CBOFFSET0_SHIFT 0
#define HOST1X_SYNC_CBSTAT_0_CBOFFSET0_MASK 0xffff
#define HOST1X_SYNC_CBSTAT_0_CBCLASS0_SHIFT 16
#define HOST1X_SYNC_CBSTAT_0_CBCLASS0_MASK 0xffff
#define HOST1X_SYNC_CBSTAT_x(x)		(HOST1X_SYNC_CBSTAT_0 + (4 * (x)))

/* sync registers */
#define NV_HOST1X_SYNCPT_NB_PTS 32
#define NV_HOST1X_SYNCPT_NB_BASES 8
#define NV_HOST1X_NB_MLOCKS 16

/* host class methods */
enum {
	NV_CLASS_HOST_INCR_SYNCPT = 0x0,
	NV_CLASS_HOST_WAIT_SYNCPT = 0x8,
	NV_CLASS_HOST_WAIT_SYNCPT_BASE = 0x9,
	NV_CLASS_HOST_LOAD_SYNCPT_BASE = 0xb,
	NV_CLASS_HOST_INCR_SYNCPT_BASE = 0xc,
	NV_CLASS_HOST_INDOFF = 0x2d,
	NV_CLASS_HOST_INDDATA = 0x2e
};
/*  sync point conditionals */
enum {
	NV_SYNCPT_IMMEDIATE = 0x0,
	NV_SYNCPT_OP_DONE = 0x1,
	NV_SYNCPT_RD_DONE = 0x2,
	NV_SYNCPT_REG_WR_SAFE = 0x3,
};

static inline u32 nvhost_class_host_wait_syncpt(
	unsigned indx, unsigned threshold)
{
	return (indx << 24) | (threshold & 0xffffff);
}

static inline u32 nvhost_class_host_load_syncpt_base(
	unsigned indx, unsigned threshold)
{
	return (indx << 24) | (threshold & 0xffffff);
}

static inline u32 nvhost_class_host_wait_syncpt_base(
	unsigned indx, unsigned base_indx, unsigned offset)
{
	return (indx << 24) | (base_indx << 16) | offset;
}

static inline u32 nvhost_class_host_incr_syncpt_base(
	unsigned base_indx, unsigned offset)
{
	return (base_indx << 24) | offset;
}

static inline u32 nvhost_class_host_incr_syncpt(
	unsigned cond, unsigned indx)
{
	return (cond << 8) | indx;
}

enum {
	NV_HOST_MODULE_HOST1X = 0,
	NV_HOST_MODULE_MPE = 1,
	NV_HOST_MODULE_GR3D = 6
};

static inline u32 nvhost_class_host_indoff_reg_write(
	unsigned mod_id, unsigned offset, bool auto_inc)
{
	u32 v = (0xf << 28) | (mod_id << 18) | (offset << 2);
	if (auto_inc)
		v |= BIT(27);
	return v;
}

static inline u32 nvhost_class_host_indoff_reg_read(
	unsigned mod_id, unsigned offset, bool auto_inc)
{
	u32 v = (mod_id << 18) | (offset << 2) | 1;
	if (auto_inc)
		v |= BIT(27);
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
	return nvhost_opcode_imm(NV_CLASS_HOST_INCR_SYNCPT,
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

#endif
