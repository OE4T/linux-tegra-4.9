/*
 * drivers/video/tegra/host/gr3d/gr3d_t20.c
 *
 * Tegra Graphics Host 3D for Tegra2
 *
 * Copyright (c) 2010-2013, NVIDIA Corporation.
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

#include "nvhost_hwctx.h"
#include "nvhost_channel.h"
#include "host1x/host1x.h"
#include "host1x/host1x01_hardware.h"
#include "gr3d.h"
#include "chip_support.h"
#include "nvhost_memmgr.h"
#include "nvhost_job.h"
#include "nvhost_acm.h"
#include "class_ids.h"

#include <linux/slab.h>
#include <linux/scatterlist.h>

static const struct hwctx_reginfo ctxsave_regs_3d_global[] = {
	HWCTX_REGINFO(0xe00,    4, DIRECT),
	HWCTX_REGINFO(0xe05,   30, DIRECT),
	HWCTX_REGINFO(0xe25,    2, DIRECT),
	HWCTX_REGINFO(0xe28,    2, DIRECT),
	HWCTX_REGINFO(0x001,    2, DIRECT),
	HWCTX_REGINFO(0x00c,   10, DIRECT),
	HWCTX_REGINFO(0x100,   34, DIRECT),
	HWCTX_REGINFO(0x124,    2, DIRECT),
	HWCTX_REGINFO(0x200,    5, DIRECT),
	HWCTX_REGINFO(0x205, 1024, INDIRECT),
	HWCTX_REGINFO(0x207, 1024, INDIRECT),
	HWCTX_REGINFO(0x209,    1, DIRECT),
	HWCTX_REGINFO(0x300,   64, DIRECT),
	HWCTX_REGINFO(0x343,   25, DIRECT),
	HWCTX_REGINFO(0x363,    2, DIRECT),
	HWCTX_REGINFO(0x400,   16, DIRECT),
	HWCTX_REGINFO(0x411,    1, DIRECT),
	HWCTX_REGINFO(0x500,    4, DIRECT),
	HWCTX_REGINFO(0x520,   32, DIRECT),
	HWCTX_REGINFO(0x540,   64, INDIRECT),
	HWCTX_REGINFO(0x600,   16, INDIRECT_4X),
	HWCTX_REGINFO(0x603,  128, INDIRECT),
	HWCTX_REGINFO(0x608,    4, DIRECT),
	HWCTX_REGINFO(0x60e,    1, DIRECT),
	HWCTX_REGINFO(0x700,   64, INDIRECT),
	HWCTX_REGINFO(0x710,   50, DIRECT),
	HWCTX_REGINFO(0x800,   16, INDIRECT_4X),
	HWCTX_REGINFO(0x803,  512, INDIRECT),
	HWCTX_REGINFO(0x805,   64, INDIRECT),
	HWCTX_REGINFO(0x820,   32, DIRECT),
	HWCTX_REGINFO(0x900,   64, INDIRECT),
	HWCTX_REGINFO(0x902,    2, DIRECT),
	HWCTX_REGINFO(0xa02,   10, DIRECT),
	HWCTX_REGINFO(0xe04,    1, DIRECT),
	HWCTX_REGINFO(0xe2a,    1, DIRECT),
};

/* the same context save command sequence is used for all contexts. */
#define SAVE_BEGIN_V0_SIZE 5
#define SAVE_DIRECT_V0_SIZE 3
#define SAVE_INDIRECT_V0_SIZE 5
#define SAVE_END_V0_SIZE 5
#define SAVE_INCRS 3
#define SAVE_THRESH_OFFSET 1
#define RESTORE_BEGIN_SIZE 4
#define RESTORE_DIRECT_SIZE 1
#define RESTORE_INDIRECT_SIZE 2
#define RESTORE_END_SIZE 1

struct save_info {
	u32 *ptr;
	unsigned int save_count;
	unsigned int restore_count;
	unsigned int save_incrs;
	unsigned int restore_incrs;
};

static u32 *setup_restore_regs_v0(u32 *ptr,
			const struct hwctx_reginfo *regs,
			unsigned int nr_regs)
{
	const struct hwctx_reginfo *rend = regs + nr_regs;

	for ( ; regs != rend; ++regs) {
		u32 offset = regs->offset;
		u32 count = regs->count;
		u32 indoff = offset + 1;
		switch (regs->type) {
		case HWCTX_REGINFO_DIRECT:
			nvhost_3dctx_restore_direct(ptr, offset, count);
			ptr += RESTORE_DIRECT_SIZE;
			break;
		case HWCTX_REGINFO_INDIRECT_4X:
			++indoff;
			/* fall through */
		case HWCTX_REGINFO_INDIRECT:
			nvhost_3dctx_restore_indirect(ptr,
					offset, 0, indoff, count);
			ptr += RESTORE_INDIRECT_SIZE;
			break;
		}
		ptr += count;
	}
	return ptr;
}

static void setup_restore_v0(struct host1x_hwctx_handler *h, u32 *ptr)
{
	nvhost_3dctx_restore_begin(h, ptr);
	ptr += RESTORE_BEGIN_SIZE;

	ptr = setup_restore_regs_v0(ptr,
			ctxsave_regs_3d_global,
			ARRAY_SIZE(ctxsave_regs_3d_global));

	nvhost_3dctx_restore_end(h, ptr);

	wmb();
}

/*** v0 saver ***/

static void save_push_v0(struct nvhost_hwctx *nctx, struct nvhost_cdma *cdma)
{
	struct host1x_hwctx *ctx = to_host1x_hwctx(nctx);
	struct host1x_hwctx_handler *p = host1x_hwctx_handler(ctx);

	nvhost_cdma_push_gather(cdma,
			nvhost_get_host(nctx->channel->dev)->memmgr,
			p->save_buf,
			0,
			nvhost_opcode_gather(p->save_size),
			p->save_phys);
}

static void save_begin_v0(struct host1x_hwctx_handler *h, u32 *ptr)
{
	/* 3d: when done, increment syncpt to base+1 */
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
	ptr[1] = nvhost_opcode_imm_incr_syncpt(
			host1x_uclass_incr_syncpt_cond_op_done_v(),
			h->h.syncpt); /*  incr 1 */
	/* host: wait for syncpt base+1 */
	ptr[2] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					host1x_uclass_wait_syncpt_base_r(), 1);
	ptr[3] = nvhost_class_host_wait_syncpt_base(h->h.syncpt,
						h->h.waitbase, 1);
	/* host: signal context read thread to start reading */
	ptr[4] = nvhost_opcode_imm_incr_syncpt(
			host1x_uclass_incr_syncpt_cond_immediate_v(),
			h->h.syncpt); /* incr 2 */
}

static void save_direct_v0(u32 *ptr, u32 start_reg, u32 count)
{
	ptr[0] = nvhost_opcode_nonincr(host1x_uclass_indoff_r(), 1);
	ptr[1] = nvhost_class_host_indoff_reg_read(
			host1x_uclass_indoff_indmodid_gr3d_v(),
			start_reg, true);
	ptr[2] = nvhost_opcode_nonincr(host1x_uclass_inddata_r(), count);
}

static void save_indirect_v0(u32 *ptr, u32 offset_reg, u32 offset,
			u32 data_reg, u32 count)
{
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID,
					offset_reg, 1);
	ptr[1] = offset;
	ptr[2] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					host1x_uclass_indoff_r(), 1);
	ptr[3] = nvhost_class_host_indoff_reg_read(
			host1x_uclass_indoff_indmodid_gr3d_v(),
			data_reg, false);
	ptr[4] = nvhost_opcode_nonincr(host1x_uclass_inddata_r(), count);
}

static void save_end_v0(struct host1x_hwctx_handler *h, u32 *ptr)
{
	/* Wait for context read service to finish (cpu incr 3) */
	ptr[0] = nvhost_opcode_nonincr(host1x_uclass_wait_syncpt_base_r(), 1);
	ptr[1] = nvhost_class_host_wait_syncpt_base(h->h.syncpt,
			h->h.waitbase, h->save_incrs);
	/* Advance syncpoint base */
	ptr[2] = nvhost_opcode_nonincr(host1x_uclass_incr_syncpt_base_r(), 1);
	ptr[3] = nvhost_class_host_incr_syncpt_base(h->h.waitbase,
			h->save_incrs);
	/* set class back to the unit */
	ptr[4] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
}

static u32 *save_regs_v0(u32 *ptr, unsigned int *pending,
			struct nvhost_channel *ch,
			const struct hwctx_reginfo *regs,
			unsigned int nr_regs)
{
	const struct hwctx_reginfo *rend = regs + nr_regs;
	int drain_result = 0;

	for ( ; regs != rend; ++regs) {
		u32 count = regs->count;
		switch (regs->type) {
		case HWCTX_REGINFO_DIRECT:
			ptr += RESTORE_DIRECT_SIZE;
			break;
		case HWCTX_REGINFO_INDIRECT:
		case HWCTX_REGINFO_INDIRECT_4X:
			ptr += RESTORE_INDIRECT_SIZE;
			break;
		}
		drain_result = nvhost_channel_drain_read_fifo(ch,
			ptr, count, pending);
		WARN_ON(drain_result < 0);
		ptr += count;
	}
	return ptr;
}

/*** save ***/

static void setup_save_regs(struct save_info *info,
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
		u32 indoff = offset + 1;
		switch (regs->type) {
		case HWCTX_REGINFO_DIRECT:
			if (ptr) {
				save_direct_v0(ptr, offset, count);
				ptr += SAVE_DIRECT_V0_SIZE;
			}
			save_count += SAVE_DIRECT_V0_SIZE;
			restore_count += RESTORE_DIRECT_SIZE;
			break;
		case HWCTX_REGINFO_INDIRECT_4X:
			++indoff;
			/* fall through */
		case HWCTX_REGINFO_INDIRECT:
			if (ptr) {
				save_indirect_v0(ptr, offset, 0,
						indoff, count);
				ptr += SAVE_INDIRECT_V0_SIZE;
			}
			save_count += SAVE_INDIRECT_V0_SIZE;
			restore_count += RESTORE_INDIRECT_SIZE;
			break;
		}
		if (ptr) {
			/* SAVE cases only: reserve room for incoming data */
			u32 k = 0;
			/*
			 * Create a signature pattern for indirect data (which
			 * will be overwritten by true incoming data) for
			 * better deducing where we are in a long command
			 * sequence, when given only a FIFO snapshot for debug
			 * purposes.
			*/
			for (k = 0; k < count; k++)
				*(ptr + k) = 0xd000d000 | (offset << 16) | k;
			ptr += count;
		}
		save_count += count;
		restore_count += count;
	}

	info->ptr = ptr;
	info->save_count = save_count;
	info->restore_count = restore_count;
}

static void setup_save(struct host1x_hwctx_handler *h, u32 *ptr)
{
	struct save_info info = {
		ptr,
		SAVE_BEGIN_V0_SIZE,
		RESTORE_BEGIN_SIZE,
		SAVE_INCRS,
		1
	};

	if (info.ptr) {
		save_begin_v0(h, info.ptr);
		info.ptr += SAVE_BEGIN_V0_SIZE;
	}

	/* save regs */
	setup_save_regs(&info,
			ctxsave_regs_3d_global,
			ARRAY_SIZE(ctxsave_regs_3d_global));

	if (info.ptr) {
		save_end_v0(h, info.ptr);
		info.ptr += SAVE_END_V0_SIZE;
	}

	wmb();

	h->save_size = info.save_count + SAVE_END_V0_SIZE;
	h->restore_size = info.restore_count + RESTORE_END_SIZE;
	h->save_incrs = info.save_incrs;
	h->h.save_thresh = h->save_incrs - SAVE_THRESH_OFFSET;
	h->restore_incrs = info.restore_incrs;
}



/*** ctx3d ***/

static struct nvhost_hwctx *ctx3d_alloc_v0(struct nvhost_hwctx_handler *h,
		struct nvhost_channel *ch)
{
	struct host1x_hwctx_handler *p = to_host1x_hwctx_handler(h);
	struct host1x_hwctx *ctx =
		nvhost_3dctx_alloc_common(p, ch, true);
	if (ctx) {
		setup_restore_v0(p, ctx->restore_virt);
		return &ctx->hwctx;
	} else
		return NULL;
}

static void ctx3d_save_service(struct nvhost_hwctx *nctx)
{
	struct host1x_hwctx *ctx = to_host1x_hwctx(nctx);

	u32 *ptr = (u32 *)ctx->restore_virt + RESTORE_BEGIN_SIZE;
	unsigned int pending = 0;

	ptr = save_regs_v0(ptr, &pending, nctx->channel,
			ctxsave_regs_3d_global,
			ARRAY_SIZE(ctxsave_regs_3d_global));

	wmb();
	nvhost_syncpt_cpu_incr(&nvhost_get_host(nctx->channel->dev)->syncpt,
			ctx->hwctx.h->syncpt);
}

struct nvhost_hwctx_handler *nvhost_gr3d_t20_ctxhandler_init(
		u32 syncpt, u32 waitbase,
		struct nvhost_channel *ch)
{
	struct mem_mgr *memmgr;
	u32 *save_ptr;
	struct host1x_hwctx_handler *p;

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;
	memmgr = nvhost_get_host(ch->dev)->memmgr;

	p->h.syncpt = syncpt;
	p->h.waitbase = waitbase;

	setup_save(p, NULL);

	p->save_buf = nvhost_memmgr_alloc(memmgr,
			p->save_size * sizeof(u32), 32,
			mem_mgr_flag_write_combine);
	if (IS_ERR(p->save_buf))
		goto fail_alloc;

	save_ptr = nvhost_memmgr_mmap(p->save_buf);
	if (!save_ptr)
		goto fail_mmap;

	p->save_sgt = nvhost_memmgr_pin(memmgr, p->save_buf);
	if (IS_ERR(p->save_sgt))
		goto fail_pin;
	p->save_phys = sg_dma_address(p->save_sgt->sgl);

	setup_save(p, save_ptr);

	nvhost_memmgr_munmap(p->save_buf, save_ptr);

	p->save_slots = 1;
	p->h.alloc = ctx3d_alloc_v0;
	p->h.save_push = save_push_v0;
	p->h.restore_push = nvhost_3dctx_restore_push;
	p->h.save_service = ctx3d_save_service;
	p->h.get = nvhost_3dctx_get;
	p->h.put = nvhost_3dctx_put;

	return &p->h;

fail_pin:
	nvhost_memmgr_munmap(p->save_buf, save_ptr);
fail_mmap:
	nvhost_memmgr_put(memmgr, p->save_buf);
fail_alloc:
	kfree(p);
	return NULL;
}

int nvhost_gr3d_t20_read_reg(struct platform_device *dev,
	struct nvhost_channel *channel,
	struct nvhost_hwctx *hwctx,
	u32 offset,
	u32 *value)
{
	struct host1x_hwctx_handler *h = to_host1x_hwctx_handler(hwctx->h);
	u32 syncpt_incrs = 4;
	unsigned int pending = 0;
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	void *ref;
	void *read_waiter = NULL;
	struct nvhost_job *job;
	int err;
	struct mem_handle *mem = NULL;
	u32 *cmdbuf_ptr = NULL;
	struct mem_mgr *memmgr = hwctx->memmgr;
	u32 opcodes[] = {
		/* Switch to 3D - wait for it to complete what it was doing */
		nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0),
		nvhost_opcode_imm_incr_syncpt(
				host1x_uclass_incr_syncpt_cond_op_done_v(),
				h->h.syncpt),
		nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				host1x_uclass_wait_syncpt_base_r(), 1),
		nvhost_class_host_wait_syncpt_base(h->h.syncpt,
				h->h.waitbase, 1),
		/*  Tell 3D to send register value to FIFO */
		nvhost_opcode_nonincr(host1x_uclass_indoff_r(), 1),
		nvhost_class_host_indoff_reg_read(
				host1x_uclass_indoff_indmodid_gr3d_v(),
				offset, false),
		nvhost_opcode_imm(host1x_uclass_inddata_r(), 0),
		/*  Increment syncpt to indicate that FIFO can be read */
		nvhost_opcode_imm_incr_syncpt(
				host1x_uclass_incr_syncpt_cond_immediate_v(),
				h->h.syncpt),
		/*  Wait for value to be read from FIFO */
		nvhost_opcode_nonincr(host1x_uclass_wait_syncpt_base_r(), 1),
		nvhost_class_host_wait_syncpt_base(h->h.syncpt,
				h->h.waitbase, 3),
		/*  Indicate submit complete */
		nvhost_opcode_nonincr(host1x_uclass_incr_syncpt_base_r(), 1),
		nvhost_class_host_incr_syncpt_base(h->h.waitbase, 4),
		nvhost_opcode_imm_incr_syncpt(
				host1x_uclass_incr_syncpt_cond_immediate_v(),
				h->h.syncpt),
	};

	mem = nvhost_memmgr_alloc(memmgr, sizeof(opcodes),
			32, mem_mgr_flag_uncacheable);
	if (IS_ERR(mem))
		return PTR_ERR(mem);

	cmdbuf_ptr = nvhost_memmgr_mmap(mem);
	if (!cmdbuf_ptr) {
		err = -ENOMEM;
		goto done;
	}

	read_waiter = nvhost_intr_alloc_waiter();
	if (!read_waiter) {
		err = -ENOMEM;
		goto done;
	}

	job = nvhost_job_alloc(channel, hwctx, 1, 0, 0, 1, memmgr);
	if (!job) {
		err = -ENOMEM;
		goto done;
	}

	job->hwctx_syncpt_idx = 0;
	job->sp->id = h->h.syncpt;
	job->sp->waitbase = h->h.waitbase;
	job->sp->incrs = syncpt_incrs;
	job->num_syncpts = 1;
	job->serialize = 1;
	memcpy(cmdbuf_ptr, opcodes, sizeof(opcodes));

	/* Submit job */
	nvhost_job_add_gather(job, nvhost_memmgr_handle_to_id(mem),
			ARRAY_SIZE(opcodes), 0);

	err = nvhost_job_pin(job, &nvhost_get_host(dev)->syncpt);
	if (err)
		goto done;

	err = nvhost_channel_submit(job);
	if (err)
		goto done;

	/* Wait for FIFO to be ready */
	err = nvhost_intr_add_action(&nvhost_get_host(dev)->intr,
			h->h.syncpt, job->sp->fence - 2,
			NVHOST_INTR_ACTION_WAKEUP, &wq,
			read_waiter,
			&ref);
	read_waiter = NULL;
	WARN(err, "Failed to set wakeup interrupt");
	wait_event(wq,
		nvhost_syncpt_is_expired(&nvhost_get_host(dev)->syncpt,
				h->h.syncpt, job->sp->fence - 2));
	nvhost_intr_put_ref(&nvhost_get_host(dev)->intr, h->h.syncpt,
			ref);

	/* Read the register value from FIFO */
	err = nvhost_channel_drain_read_fifo(channel, value, 1, &pending);

	/* Indicate we've read the value */
	nvhost_syncpt_cpu_incr(&nvhost_get_host(dev)->syncpt,
			h->h.syncpt);

	nvhost_job_put(job);
	job = NULL;

done:
	kfree(read_waiter);
	if (cmdbuf_ptr)
		nvhost_memmgr_munmap(mem, cmdbuf_ptr);
	if (mem)
		nvhost_memmgr_put(memmgr, mem);
	return err;
}

