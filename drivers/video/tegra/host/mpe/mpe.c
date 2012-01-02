/*
 * drivers/video/tegra/host/mpe/mpe.c
 *
 * Tegra Graphics Host MPE
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "nvhost_hwctx.h"
#include "dev.h"
#include "host1x/host1x_hardware.h"
#include "host1x/host1x_channel.h"
#include "host1x/host1x_syncpt.h"
#include "t20/t20.h"
#include <linux/slab.h>

enum {
	HWCTX_REGINFO_NORMAL = 0,
	HWCTX_REGINFO_STASH,
	HWCTX_REGINFO_CALCULATE,
	HWCTX_REGINFO_WRITEBACK
};

const struct hwctx_reginfo ctxsave_regs_mpe[] = {
	HWCTX_REGINFO(0x124,  1, STASH),
	HWCTX_REGINFO(0x123,  1, STASH),
	HWCTX_REGINFO(0x103,  1, STASH),
	HWCTX_REGINFO(0x074,  1, STASH),
	HWCTX_REGINFO(0x021,  1, NORMAL),
	HWCTX_REGINFO(0x020,  1, STASH),
	HWCTX_REGINFO(0x024,  2, NORMAL),
	HWCTX_REGINFO(0x0e6,  1, NORMAL),
	HWCTX_REGINFO(0x3fc,  1, NORMAL),
	HWCTX_REGINFO(0x3d0,  1, NORMAL),
	HWCTX_REGINFO(0x3d4,  1, NORMAL),
	HWCTX_REGINFO(0x013,  1, NORMAL),
	HWCTX_REGINFO(0x022,  1, NORMAL),
	HWCTX_REGINFO(0x030,  4, NORMAL),
	HWCTX_REGINFO(0x023,  1, NORMAL),
	HWCTX_REGINFO(0x070,  1, NORMAL),
	HWCTX_REGINFO(0x0a0,  9, NORMAL),
	HWCTX_REGINFO(0x071,  1, NORMAL),
	HWCTX_REGINFO(0x100,  4, NORMAL),
	HWCTX_REGINFO(0x104,  2, NORMAL),
	HWCTX_REGINFO(0x108,  9, NORMAL),
	HWCTX_REGINFO(0x112,  2, NORMAL),
	HWCTX_REGINFO(0x114,  1, STASH),
	HWCTX_REGINFO(0x014,  1, NORMAL),
	HWCTX_REGINFO(0x072,  1, NORMAL),
	HWCTX_REGINFO(0x200,  1, NORMAL),
	HWCTX_REGINFO(0x0d1,  1, NORMAL),
	HWCTX_REGINFO(0x0d0,  1, NORMAL),
	HWCTX_REGINFO(0x0c0,  1, NORMAL),
	HWCTX_REGINFO(0x0c3,  2, NORMAL),
	HWCTX_REGINFO(0x0d2,  1, NORMAL),
	HWCTX_REGINFO(0x0d8,  1, NORMAL),
	HWCTX_REGINFO(0x0e0,  2, NORMAL),
	HWCTX_REGINFO(0x07f,  2, NORMAL),
	HWCTX_REGINFO(0x084,  8, NORMAL),
	HWCTX_REGINFO(0x0d3,  1, NORMAL),
	HWCTX_REGINFO(0x040, 13, NORMAL),
	HWCTX_REGINFO(0x050,  6, NORMAL),
	HWCTX_REGINFO(0x058,  1, NORMAL),
	HWCTX_REGINFO(0x057,  1, NORMAL),
	HWCTX_REGINFO(0x111,  1, NORMAL),
	HWCTX_REGINFO(0x130,  3, NORMAL),
	HWCTX_REGINFO(0x201,  1, NORMAL),
	HWCTX_REGINFO(0x068,  2, NORMAL),
	HWCTX_REGINFO(0x08c,  1, NORMAL),
	HWCTX_REGINFO(0x0cf,  1, NORMAL),
	HWCTX_REGINFO(0x082,  2, NORMAL),
	HWCTX_REGINFO(0x075,  1, NORMAL),
	HWCTX_REGINFO(0x0e8,  1, NORMAL),
	HWCTX_REGINFO(0x056,  1, NORMAL),
	HWCTX_REGINFO(0x057,  1, NORMAL),
	HWCTX_REGINFO(0x073,  1, CALCULATE),
	HWCTX_REGINFO(0x074,  1, NORMAL),
	HWCTX_REGINFO(0x075,  1, NORMAL),
	HWCTX_REGINFO(0x076,  1, STASH),
	HWCTX_REGINFO(0x11a,  9, NORMAL),
	HWCTX_REGINFO(0x123,  1, NORMAL),
	HWCTX_REGINFO(0x124,  1, NORMAL),
	HWCTX_REGINFO(0x12a,  5, NORMAL),
	HWCTX_REGINFO(0x12f,  1, STASH),
	HWCTX_REGINFO(0x125,  2, NORMAL),
	HWCTX_REGINFO(0x034,  1, NORMAL),
	HWCTX_REGINFO(0x133,  2, NORMAL),
	HWCTX_REGINFO(0x127,  1, NORMAL),
	HWCTX_REGINFO(0x106,  1, WRITEBACK),
	HWCTX_REGINFO(0x107,  1, WRITEBACK)
};

#define NR_STASHES 8
#define NR_WRITEBACKS 2

#define RC_RAM_LOAD_CMD 0x115
#define RC_RAM_LOAD_DATA 0x116
#define RC_RAM_READ_CMD 0x128
#define RC_RAM_READ_DATA 0x129
#define RC_RAM_SIZE 692

#define IRFR_RAM_LOAD_CMD 0xc5
#define IRFR_RAM_LOAD_DATA 0xc6
#define IRFR_RAM_READ_CMD 0xcd
#define IRFR_RAM_READ_DATA 0xce
#define IRFR_RAM_SIZE 408

struct mpe_save_info {
	u32 in[NR_STASHES];
	u32 out[NR_WRITEBACKS];
	unsigned in_pos;
	unsigned out_pos;
	u32 h264_mode;
};


/*** restore ***/

static unsigned int restore_size;

static void restore_begin(u32 *ptr)
{
	/* set class to host */
	ptr[0] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_INCR_SYNCPT_BASE, 1);
	/* increment sync point base */
	ptr[1] = nvhost_class_host_incr_syncpt_base(NVWAITBASE_MPE, 1);
	/* set class to MPE */
	ptr[2] = nvhost_opcode_setclass(NV_VIDEO_ENCODE_MPEG_CLASS_ID, 0, 0);
}
#define RESTORE_BEGIN_SIZE 3

static void restore_ram(u32 *ptr, unsigned words,
			unsigned cmd_reg, unsigned data_reg)
{
	ptr[0] = nvhost_opcode_imm(cmd_reg, words);
	ptr[1] = nvhost_opcode_nonincr(data_reg, words);
}
#define RESTORE_RAM_SIZE 2

static void restore_end(u32 *ptr)
{
	/* syncpt increment to track restore gather. */
	ptr[0] = nvhost_opcode_imm_incr_syncpt(NV_SYNCPT_OP_DONE,
			NVSYNCPT_MPE);
}
#define RESTORE_END_SIZE 1

static u32 *setup_restore_regs(u32 *ptr,
			const struct hwctx_reginfo *regs,
			unsigned int nr_regs)
{
	const struct hwctx_reginfo *rend = regs + nr_regs;

	for ( ; regs != rend; ++regs) {
		u32 offset = regs->offset;
		u32 count = regs->count;
		*ptr++ = nvhost_opcode_incr(offset, count);
		ptr += count;
	}
	return ptr;
}

static u32 *setup_restore_ram(u32 *ptr, unsigned words,
			unsigned cmd_reg, unsigned data_reg)
{
	restore_ram(ptr, words, cmd_reg, data_reg);
	return ptr + (RESTORE_RAM_SIZE + words);
}

static void setup_restore(u32 *ptr)
{
	restore_begin(ptr);
	ptr += RESTORE_BEGIN_SIZE;

	ptr = setup_restore_regs(ptr, ctxsave_regs_mpe,
				ARRAY_SIZE(ctxsave_regs_mpe));

	ptr = setup_restore_ram(ptr, RC_RAM_SIZE,
			RC_RAM_LOAD_CMD, RC_RAM_LOAD_DATA);

	ptr = setup_restore_ram(ptr, IRFR_RAM_SIZE,
			IRFR_RAM_LOAD_CMD, IRFR_RAM_LOAD_DATA);

	restore_end(ptr);

	wmb();
}


/*** save ***/

/* the same context save command sequence is used for all contexts. */
static struct nvmap_handle_ref *save_buf;
static phys_addr_t save_phys;
static unsigned int save_size;

struct save_info {
	u32 *ptr;
	unsigned int save_count;
	unsigned int restore_count;
};

static void __init save_begin(u32 *ptr)
{
	/* MPE: when done, increment syncpt to base+1 */
	ptr[0] = nvhost_opcode_setclass(NV_VIDEO_ENCODE_MPEG_CLASS_ID, 0, 0);
	ptr[1] = nvhost_opcode_imm_incr_syncpt(NV_SYNCPT_OP_DONE,
			NVSYNCPT_MPE);
	/* host: wait for syncpt base+1 */
	ptr[2] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_WAIT_SYNCPT_BASE, 1);
	ptr[3] = nvhost_class_host_wait_syncpt_base(NVSYNCPT_MPE,
						NVWAITBASE_MPE, 1);
	/* host: signal context read thread to start reading */
	ptr[4] = nvhost_opcode_imm_incr_syncpt(NV_SYNCPT_IMMEDIATE,
			NVSYNCPT_MPE);
}
#define SAVE_BEGIN_SIZE 5

static void __init save_direct(u32 *ptr, u32 start_reg, u32 count)
{
	ptr[0] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_INDOFF, 1);
	ptr[1] = nvhost_class_host_indoff_reg_read(NV_HOST_MODULE_MPE,
						start_reg, true);
	ptr[2] = nvhost_opcode_nonincr(NV_CLASS_HOST_INDDATA, count);
}
#define SAVE_DIRECT_SIZE 3

static void __init save_set_ram_cmd(u32 *ptr, u32 cmd_reg, u32 count)
{
	ptr[0] = nvhost_opcode_setclass(NV_VIDEO_ENCODE_MPEG_CLASS_ID,
					cmd_reg, 1);
	ptr[1] = count;
}
#define SAVE_SET_RAM_CMD_SIZE 2

static void __init save_read_ram_data_nasty(u32 *ptr, u32 data_reg)
{
	ptr[0] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_INDOFF, 1);
	ptr[1] = nvhost_class_host_indoff_reg_read(NV_HOST_MODULE_MPE,
						data_reg, false);
	ptr[2] = nvhost_opcode_imm(NV_CLASS_HOST_INDDATA, 0);
	/* write junk data to avoid 'cached problem with register memory' */
	ptr[3] = nvhost_opcode_setclass(NV_VIDEO_ENCODE_MPEG_CLASS_ID,
					data_reg, 1);
	ptr[4] = 0x99;
}
#define SAVE_READ_RAM_DATA_NASTY_SIZE 5

static void __init save_end(u32 *ptr)
{
	/* Wait for context read service to finish (cpu incr 3) */
	ptr[0] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_WAIT_SYNCPT_BASE, 1);
	ptr[1] = nvhost_class_host_wait_syncpt_base(NVSYNCPT_MPE,
						NVWAITBASE_MPE, 3);
	/* Advance syncpoint base */
	ptr[2] = nvhost_opcode_nonincr(NV_CLASS_HOST_INCR_SYNCPT_BASE, 1);
	ptr[3] = nvhost_class_host_incr_syncpt_base(NVWAITBASE_MPE, 3);
	/* set class back to the unit */
	ptr[4] = nvhost_opcode_setclass(NV_VIDEO_ENCODE_MPEG_CLASS_ID, 0, 0);
}
#define SAVE_END_SIZE 5

static void __init setup_save_regs(struct save_info *info,
			const struct hwctx_reginfo *regs,
			unsigned int nr_regs)
{
	const struct hwctx_reginfo *rend = regs + nr_regs;
	u32 *ptr = info->ptr;
	unsigned int save_count = info->save_count;
	unsigned int restore_count = info->restore_count;

	for ( ; regs != rend; ++regs) {
		u32 offset = regs->offset;
		u32 count = regs->count;
		if (regs->type != HWCTX_REGINFO_WRITEBACK) {
			if (ptr) {
				save_direct(ptr, offset, count);
				ptr += SAVE_DIRECT_SIZE;
				memset(ptr, 0, count * 4);
				ptr += count;
			}
			save_count += (SAVE_DIRECT_SIZE + count);
		}
		restore_count += (1 + count);
	}

	info->ptr = ptr;
	info->save_count = save_count;
	info->restore_count = restore_count;
}

static void __init setup_save_ram_nasty(struct save_info *info,	unsigned words,
					unsigned cmd_reg, unsigned data_reg)
{
	u32 *ptr = info->ptr;
	unsigned int save_count = info->save_count;
	unsigned int restore_count = info->restore_count;
	unsigned i;

	if (ptr) {
		save_set_ram_cmd(ptr, cmd_reg, words);
		ptr += SAVE_SET_RAM_CMD_SIZE;
		for (i = words; i; --i) {
			save_read_ram_data_nasty(ptr, data_reg);
			ptr += SAVE_READ_RAM_DATA_NASTY_SIZE;
		}
	}

	save_count += SAVE_SET_RAM_CMD_SIZE;
	save_count += words * SAVE_READ_RAM_DATA_NASTY_SIZE;
	restore_count += (RESTORE_RAM_SIZE + words);

	info->ptr = ptr;
	info->save_count = save_count;
	info->restore_count = restore_count;
}

static void __init setup_save(u32 *ptr)
{
	struct save_info info = {
		ptr,
		SAVE_BEGIN_SIZE,
		RESTORE_BEGIN_SIZE
	};

	if (info.ptr) {
		save_begin(info.ptr);
		info.ptr += SAVE_BEGIN_SIZE;
	}

	setup_save_regs(&info, ctxsave_regs_mpe,
			ARRAY_SIZE(ctxsave_regs_mpe));

	setup_save_ram_nasty(&info, RC_RAM_SIZE,
			RC_RAM_READ_CMD, RC_RAM_READ_DATA);

	setup_save_ram_nasty(&info, IRFR_RAM_SIZE,
			IRFR_RAM_READ_CMD, IRFR_RAM_READ_DATA);

	if (info.ptr) {
		save_end(info.ptr);
		info.ptr += SAVE_END_SIZE;
	}

	wmb();

	save_size = info.save_count + SAVE_END_SIZE;
	restore_size = info.restore_count + RESTORE_END_SIZE;
}


static u32 calculate_mpe(u32 word, struct mpe_save_info *msi)
{
	u32 buffer_full_read = msi->in[0] & 0x01ffffff;
	u32 byte_len = msi->in[1];
	u32 drain = (msi->in[2] >> 2) & 0x007fffff;
	u32 rep_frame = msi->in[3] & 0x0000ffff;
	u32 h264_mode = (msi->in[4] >> 11) & 1;
	int new_buffer_full;

	if (h264_mode)
		byte_len >>= 3;
	new_buffer_full = buffer_full_read + byte_len - (drain * 4);
	msi->out[0] = max(0, new_buffer_full);
	msi->out[1] = rep_frame;
	if (rep_frame == 0)
		word &= 0xffff0000;
	return word;
}

static u32 *save_regs(u32 *ptr, unsigned int *pending,
		struct nvhost_channel *channel,
		const struct hwctx_reginfo *regs,
		unsigned int nr_regs,
		struct mpe_save_info *msi)
{
	const struct hwctx_reginfo *rend = regs + nr_regs;

	for ( ; regs != rend; ++regs) {
		u32 count = regs->count;
		++ptr; /* restore incr */
		if (regs->type == HWCTX_REGINFO_NORMAL) {
			host1x_drain_read_fifo(channel->aperture,
						ptr, count, pending);
			ptr += count;
		} else {
			u32 word;
			if (regs->type == HWCTX_REGINFO_WRITEBACK) {
				BUG_ON(msi->out_pos >= NR_WRITEBACKS);
				word = msi->out[msi->out_pos++];
			} else {
				host1x_drain_read_fifo(channel->aperture,
							&word, 1, pending);
				if (regs->type == HWCTX_REGINFO_STASH) {
					BUG_ON(msi->in_pos >= NR_STASHES);
					msi->in[msi->in_pos++] = word;
				} else {
					word = calculate_mpe(word, msi);
				}
			}
			*ptr++ = word;
		}
	}
	return ptr;
}

static u32 *save_ram(u32 *ptr, unsigned int *pending,
		struct nvhost_channel *channel,
		unsigned words,	unsigned cmd_reg, unsigned data_reg)
{
	int err = 0;
	ptr += RESTORE_RAM_SIZE;
	err = host1x_drain_read_fifo(channel->aperture, ptr, words, pending);
	WARN_ON(err);
	return ptr + words;
}


/*** ctxmpe ***/

static struct nvhost_hwctx *ctxmpe_alloc(struct nvhost_channel *ch)
{
	struct nvmap_client *nvmap = nvhost_get_host(ch->dev)->nvmap;
	struct nvhost_hwctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;
	ctx->restore = nvmap_alloc(nvmap, restore_size * 4, 32,
				NVMAP_HANDLE_WRITE_COMBINE);
	if (IS_ERR_OR_NULL(ctx->restore)) {
		kfree(ctx);
		return NULL;
	}

	ctx->restore_virt = nvmap_mmap(ctx->restore);
	if (!ctx->restore_virt) {
		nvmap_free(nvmap, ctx->restore);
		kfree(ctx);
		return NULL;
	}

	kref_init(&ctx->ref);
	ctx->channel = ch;
	ctx->valid = false;
	ctx->save = save_buf;
	ctx->save_incrs = 3;
	ctx->save_thresh = 2;
	ctx->restore_phys = nvmap_pin(nvmap, ctx->restore);
	ctx->restore_size = restore_size;
	ctx->restore_incrs = 1;

	setup_restore(ctx->restore_virt);

	return ctx;
}

static void ctxmpe_get(struct nvhost_hwctx *ctx)
{
	kref_get(&ctx->ref);
}

static void ctxmpe_free(struct kref *ref)
{
	struct nvhost_hwctx *ctx = container_of(ref, struct nvhost_hwctx, ref);
	struct nvmap_client *nvmap = nvhost_get_host(ctx->channel->dev)->nvmap;

	if (ctx->restore_virt)
		nvmap_munmap(ctx->restore, ctx->restore_virt);
	nvmap_unpin(nvmap, ctx->restore);
	nvmap_free(nvmap, ctx->restore);
	kfree(ctx);
}

static void ctxmpe_put(struct nvhost_hwctx *ctx)
{
	kref_put(&ctx->ref, ctxmpe_free);
}

static void ctxmpe_save_push(struct nvhost_cdma *cdma, struct nvhost_hwctx *ctx)
{
	nvhost_cdma_push(cdma,
			nvhost_opcode_gather(save_size),
			save_phys);
}

static void ctxmpe_save_service(struct nvhost_hwctx *ctx)
{
	u32 *ptr = (u32 *)ctx->restore_virt + RESTORE_BEGIN_SIZE;
	unsigned int pending = 0;
	struct mpe_save_info msi;

	msi.in_pos = 0;
	msi.out_pos = 0;

	ptr = save_regs(ptr, &pending, ctx->channel,
			ctxsave_regs_mpe, ARRAY_SIZE(ctxsave_regs_mpe), &msi);

	ptr = save_ram(ptr, &pending, ctx->channel,
		RC_RAM_SIZE, RC_RAM_READ_CMD, RC_RAM_READ_DATA);

	ptr = save_ram(ptr, &pending, ctx->channel,
		IRFR_RAM_SIZE, IRFR_RAM_READ_CMD, IRFR_RAM_READ_DATA);

	wmb();
	nvhost_syncpt_cpu_incr(&nvhost_get_host(ctx->channel->dev)->syncpt,
			NVSYNCPT_MPE);
}

int __init nvhost_mpe_ctxhandler_init(struct nvhost_hwctx_handler *h)
{
	struct nvhost_channel *ch;
	struct nvmap_client *nvmap;
	u32 *save_ptr;

	ch = container_of(h, struct nvhost_channel, ctxhandler);
	nvmap = nvhost_get_host(ch->dev)->nvmap;

	setup_save(NULL);

	save_buf = nvmap_alloc(nvmap, save_size * 4, 32,
				NVMAP_HANDLE_WRITE_COMBINE);
	if (IS_ERR(save_buf)) {
		int err = PTR_ERR(save_buf);
		save_buf = NULL;
		return err;
	}

	save_ptr = nvmap_mmap(save_buf);
	if (!save_ptr) {
		nvmap_free(nvmap, save_buf);
		save_buf = NULL;
		return -ENOMEM;
	}

	save_phys = nvmap_pin(nvmap, save_buf);

	setup_save(save_ptr);

	h->alloc = ctxmpe_alloc;
	h->save_push = ctxmpe_save_push;
	h->save_service = ctxmpe_save_service;
	h->get = ctxmpe_get;
	h->put = ctxmpe_put;

	return 0;
}

int nvhost_mpe_prepare_power_off(struct nvhost_device *dev)
{
	return host1x_save_context(dev, NVSYNCPT_MPE);
}
