/*
 * drivers/video/tegra/host/host1x/channel_host1x.c
 *
 * Tegra Graphics Host Channel
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

#include "nvhost_channel.h"
#include "dev.h"
#include "nvhost_hwctx.h"
#include <trace/events/nvhost.h>
#include <linux/slab.h>

#include "host1x_syncpt.h"
#include "host1x_channel.h"
#include "host1x_hardware.h"
#include "nvhost_intr.h"

#define NV_FIFO_READ_TIMEOUT 200000

static void sync_waitbases(struct nvhost_channel *ch, u32 syncpt_val)
{
	unsigned long waitbase;
	unsigned long int waitbase_mask = ch->desc->waitbases;
	if (ch->desc->waitbasesync) {
		waitbase = find_first_bit(&waitbase_mask, BITS_PER_LONG);
		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				NV_CLASS_HOST_LOAD_SYNCPT_BASE,
				1),
				nvhost_class_host_load_syncpt_base(waitbase,
						syncpt_val));
	}
}

int host1x_channel_submit(struct nvhost_job *job)
{
	struct nvhost_hwctx *hwctx_to_save = NULL;
	struct nvhost_channel *channel = job->ch;
	struct nvhost_syncpt *sp = &job->ch->dev->syncpt;
	u32 user_syncpt_incrs = job->syncpt_incrs;
	bool need_restore = false;
	u32 syncval;
	int err;
	void *ctxrestore_waiter = NULL;
	void *ctxsave_waiter, *completed_waiter;

	if (job->hwctx && job->hwctx->has_timedout)
		return -ETIMEDOUT;

	ctxsave_waiter = nvhost_intr_alloc_waiter();
	completed_waiter = nvhost_intr_alloc_waiter();
	if (!ctxsave_waiter || !completed_waiter) {
		err = -ENOMEM;
		goto done;
	}

	/* keep module powered */
	nvhost_module_busy(&channel->mod);
	if (channel->mod.desc->busy)
		channel->mod.desc->busy(&channel->mod);

	/* before error checks, return current max */
	job->syncpt_end = nvhost_syncpt_read_max(sp, job->syncpt_id);

	/* get submit lock */
	err = mutex_lock_interruptible(&channel->submitlock);
	if (err) {
		nvhost_module_idle(&channel->mod);
		goto done;
	}

	/* If we are going to need a restore, allocate a waiter for it */
	if (channel->cur_ctx != job->hwctx && job->hwctx && job->hwctx->valid) {
		ctxrestore_waiter = nvhost_intr_alloc_waiter();
		if (!ctxrestore_waiter) {
			mutex_unlock(&channel->submitlock);
			nvhost_module_idle(&channel->mod);
			err = -ENOMEM;
			goto done;
		}
		need_restore = true;
	}

	/* remove stale waits */
	if (job->num_waitchk) {
		err = nvhost_syncpt_wait_check(sp,
					       job->nvmap,
					       job->waitchk_mask,
					       job->waitchk,
					       job->num_waitchk);
		if (err) {
			dev_warn(&channel->dev->pdev->dev,
				 "nvhost_syncpt_wait_check failed: %d\n", err);
			mutex_unlock(&channel->submitlock);
			nvhost_module_idle(&channel->mod);
			goto done;
		}
	}

	/* begin a CDMA submit */
	err = nvhost_cdma_begin(&channel->cdma, job);
	if (err) {
		mutex_unlock(&channel->submitlock);
		nvhost_module_idle(&channel->mod);
		goto done;
	}

	sync_waitbases(channel, job->syncpt_end);

	/* context switch */
	if (channel->cur_ctx != job->hwctx) {
		trace_nvhost_channel_context_switch(channel->desc->name,
		  channel->cur_ctx, job->hwctx);
		hwctx_to_save = channel->cur_ctx;
		if (hwctx_to_save &&
			hwctx_to_save->has_timedout) {
			hwctx_to_save = NULL;
			dev_dbg(&channel->dev->pdev->dev,
				"%s: skip save of timed out context (0x%p)\n",
				__func__, channel->cur_ctx);
		}
		if (hwctx_to_save) {
			job->syncpt_incrs += hwctx_to_save->save_incrs;
			hwctx_to_save->valid = true;
			channel->ctxhandler.get(hwctx_to_save);
		}
		channel->cur_ctx = job->hwctx;
		if (need_restore)
			job->syncpt_incrs += channel->cur_ctx->restore_incrs;
	}

	/* get absolute sync value */
	if (BIT(job->syncpt_id) & sp->client_managed)
		syncval = nvhost_syncpt_set_max(sp,
				job->syncpt_id, job->syncpt_incrs);
	else
		syncval = nvhost_syncpt_incr_max(sp,
				job->syncpt_id, job->syncpt_incrs);

	job->syncpt_end = syncval;

	/* push save buffer (pre-gather setup depends on unit) */
	if (hwctx_to_save)
		channel->ctxhandler.save_push(&channel->cdma, hwctx_to_save);

	/* gather restore buffer */
	if (need_restore) {
		nvhost_cdma_push_gather(&channel->cdma,
			channel->dev->nvmap,
			nvmap_ref_to_handle(channel->cur_ctx->restore),
			nvhost_opcode_gather(channel->cur_ctx->restore_size),
			channel->cur_ctx->restore_phys);
		channel->ctxhandler.get(channel->cur_ctx);
	}

	/* add a setclass for modules that require it (unless ctxsw added it) */
	if (!hwctx_to_save && !need_restore && channel->desc->class)
		nvhost_cdma_push(&channel->cdma,
			nvhost_opcode_setclass(channel->desc->class, 0, 0),
			NVHOST_OPCODE_NOOP);

	if (job->null_kickoff) {
		int incr;
		u32 op_incr;

		/* TODO ideally we'd also perform host waits here */

		/* push increments that correspond to nulled out commands */
		op_incr = nvhost_opcode_imm(0, 0x100 | job->syncpt_id);
		for (incr = 0; incr < (user_syncpt_incrs >> 1); incr++)
			nvhost_cdma_push(&channel->cdma, op_incr, op_incr);
		if (user_syncpt_incrs & 1)
			nvhost_cdma_push(&channel->cdma,
					op_incr, NVHOST_OPCODE_NOOP);

		/* for 3d, waitbase needs to be incremented after each submit */
		if (channel->desc->class == NV_GRAPHICS_3D_CLASS_ID)
			nvhost_cdma_push(&channel->cdma,
					nvhost_opcode_setclass(
						NV_HOST1X_CLASS_ID,
						NV_CLASS_HOST_INCR_SYNCPT_BASE,
						1),
					nvhost_class_host_incr_syncpt_base(
						NVWAITBASE_3D,
						user_syncpt_incrs));
	} else {
		/* push user gathers */
		int i = 0;
		for ( ; i < job->num_gathers; i++) {
			u32 op1 = nvhost_opcode_gather(job->gathers[i].words);
			u32 op2 = job->gathers[i].mem;
			nvhost_cdma_push_gather(&channel->cdma,
					job->nvmap, job->unpins[i/2],
					op1, op2);
		}
	}

	/* end CDMA submit & stash pinned hMems into sync queue */
	nvhost_cdma_end(&channel->cdma, job);

	trace_nvhost_channel_submitted(channel->desc->name,
			syncval - job->syncpt_incrs, syncval);

	/*
	 * schedule a context save interrupt (to drain the host FIFO
	 * if necessary, and to release the restore buffer)
	 */
	if (hwctx_to_save) {
		err = nvhost_intr_add_action(&channel->dev->intr,
			job->syncpt_id,
			syncval - job->syncpt_incrs
				+ hwctx_to_save->save_thresh,
			NVHOST_INTR_ACTION_CTXSAVE, hwctx_to_save,
			ctxsave_waiter,
			NULL);
		ctxsave_waiter = NULL;
		WARN(err, "Failed to set ctx save interrupt");
	}

	if (need_restore) {
		BUG_ON(!ctxrestore_waiter);
		err = nvhost_intr_add_action(&channel->dev->intr,
			job->syncpt_id,
			syncval - user_syncpt_incrs,
			NVHOST_INTR_ACTION_CTXRESTORE, channel->cur_ctx,
			ctxrestore_waiter,
			NULL);
		ctxrestore_waiter = NULL;
		WARN(err, "Failed to set ctx restore interrupt");
	}

	/* schedule a submit complete interrupt */
	err = nvhost_intr_add_action(&channel->dev->intr, job->syncpt_id,
			syncval,
			NVHOST_INTR_ACTION_SUBMIT_COMPLETE, channel,
			completed_waiter,
			NULL);
	completed_waiter = NULL;
	WARN(err, "Failed to set submit complete interrupt");

	mutex_unlock(&channel->submitlock);

done:
	kfree(ctxrestore_waiter);
	kfree(ctxsave_waiter);
	kfree(completed_waiter);
	return err;
}

int host1x_channel_read_3d_reg(
	struct nvhost_channel *channel,
	struct nvhost_hwctx *hwctx,
	u32 offset,
	u32 *value)
{
	struct nvhost_hwctx *hwctx_to_save = NULL;
	bool need_restore = false;
	u32 syncpt_incrs = 4;
	unsigned int pending = 0;
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	void *ref;
	void *ctx_waiter, *read_waiter, *completed_waiter;
	struct nvhost_job *job;
	u32 syncval;
	int err;

	if (hwctx && hwctx->has_timedout)
		return -ETIMEDOUT;

	ctx_waiter = nvhost_intr_alloc_waiter();
	read_waiter = nvhost_intr_alloc_waiter();
	completed_waiter = nvhost_intr_alloc_waiter();
	if (!ctx_waiter || !read_waiter || !completed_waiter) {
		err = -ENOMEM;
		goto done;
	}

	job = nvhost_job_alloc(channel, hwctx,
			NULL,
			channel->dev->nvmap, 0, 0);
	if (!job) {
		err = -ENOMEM;
		goto done;
	}

	/* keep module powered */
	nvhost_module_busy(&channel->mod);

	/* get submit lock */
	err = mutex_lock_interruptible(&channel->submitlock);
	if (err) {
		nvhost_module_idle(&channel->mod);
		return err;
	}

	/* context switch */
	if (channel->cur_ctx != hwctx) {
		hwctx_to_save = channel->cur_ctx;
		if (hwctx_to_save) {
			syncpt_incrs += hwctx_to_save->save_incrs;
			hwctx_to_save->valid = true;
			channel->ctxhandler.get(hwctx_to_save);
		}
		channel->cur_ctx = hwctx;
		if (channel->cur_ctx && channel->cur_ctx->valid) {
			need_restore = true;
			syncpt_incrs += channel->cur_ctx->restore_incrs;
		}
	}

	syncval = nvhost_syncpt_incr_max(&channel->dev->syncpt,
		NVSYNCPT_3D, syncpt_incrs);

	job->syncpt_id = NVSYNCPT_3D;
	job->syncpt_incrs = syncpt_incrs;
	job->syncpt_end = syncval;

	/* begin a CDMA submit */
	nvhost_cdma_begin(&channel->cdma, job);

	/* push save buffer (pre-gather setup depends on unit) */
	if (hwctx_to_save)
		channel->ctxhandler.save_push(&channel->cdma, hwctx_to_save);

	/* gather restore buffer */
	if (need_restore)
		nvhost_cdma_push(&channel->cdma,
			nvhost_opcode_gather(channel->cur_ctx->restore_size),
			channel->cur_ctx->restore_phys);

	/* Switch to 3D - wait for it to complete what it was doing */
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0),
		nvhost_opcode_imm_incr_syncpt(NV_SYNCPT_OP_DONE, NVSYNCPT_3D));
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
			NV_CLASS_HOST_WAIT_SYNCPT_BASE, 1),
		nvhost_class_host_wait_syncpt_base(NVSYNCPT_3D,
			NVWAITBASE_3D, 1));
	/*  Tell 3D to send register value to FIFO */
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_nonincr(NV_CLASS_HOST_INDOFF, 1),
		nvhost_class_host_indoff_reg_read(NV_HOST_MODULE_GR3D,
			offset, false));
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_imm(NV_CLASS_HOST_INDDATA, 0),
		NVHOST_OPCODE_NOOP);
	/*  Increment syncpt to indicate that FIFO can be read */
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_imm_incr_syncpt(NV_SYNCPT_IMMEDIATE,
			NVSYNCPT_3D),
		NVHOST_OPCODE_NOOP);
	/*  Wait for value to be read from FIFO */
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_nonincr(NV_CLASS_HOST_WAIT_SYNCPT_BASE, 1),
		nvhost_class_host_wait_syncpt_base(NVSYNCPT_3D,
			NVWAITBASE_3D, 3));
	/*  Indicate submit complete */
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_nonincr(NV_CLASS_HOST_INCR_SYNCPT_BASE, 1),
		nvhost_class_host_incr_syncpt_base(NVWAITBASE_3D, 4));
	nvhost_cdma_push(&channel->cdma,
		NVHOST_OPCODE_NOOP,
		nvhost_opcode_imm_incr_syncpt(NV_SYNCPT_IMMEDIATE,
			NVSYNCPT_3D));

	/* end CDMA submit  */
	nvhost_cdma_end(&channel->cdma, job);
	nvhost_job_put(job);
	job = NULL;

	/*
	 * schedule a context save interrupt (to drain the host FIFO
	 * if necessary, and to release the restore buffer)
	 */
	if (hwctx_to_save) {
		err = nvhost_intr_add_action(&channel->dev->intr, NVSYNCPT_3D,
			syncval - syncpt_incrs + hwctx_to_save->save_incrs - 1,
			NVHOST_INTR_ACTION_CTXSAVE, hwctx_to_save,
			ctx_waiter,
			NULL);
		ctx_waiter = NULL;
		WARN(err, "Failed to set context save interrupt");
	}

	/* Wait for FIFO to be ready */
	err = nvhost_intr_add_action(&channel->dev->intr, NVSYNCPT_3D,
			syncval - 2,
			NVHOST_INTR_ACTION_WAKEUP, &wq,
			read_waiter,
			&ref);
	read_waiter = NULL;
	WARN(err, "Failed to set wakeup interrupt");
	wait_event(wq,
		nvhost_syncpt_min_cmp(&channel->dev->syncpt,
				NVSYNCPT_3D, syncval - 2));
	nvhost_intr_put_ref(&channel->dev->intr, ref);

	/* Read the register value from FIFO */
	err = host1x_drain_read_fifo(channel->aperture,
		value, 1, &pending);

	/* Indicate we've read the value */
	nvhost_syncpt_cpu_incr(&channel->dev->syncpt, NVSYNCPT_3D);

	/* Schedule a submit complete interrupt */
	err = nvhost_intr_add_action(&channel->dev->intr, NVSYNCPT_3D, syncval,
			NVHOST_INTR_ACTION_SUBMIT_COMPLETE, channel,
			completed_waiter, NULL);
	completed_waiter = NULL;
	WARN(err, "Failed to set submit complete interrupt");

	mutex_unlock(&channel->submitlock);

done:
	kfree(ctx_waiter);
	kfree(read_waiter);
	kfree(completed_waiter);
	return err;
}


int host1x_drain_read_fifo(void __iomem *chan_regs,
	u32 *ptr, unsigned int count, unsigned int *pending)
{
	unsigned int entries = *pending;
	unsigned long timeout = jiffies + NV_FIFO_READ_TIMEOUT;
	while (count) {
		unsigned int num;

		while (!entries && time_before(jiffies, timeout)) {
			/* query host for number of entries in fifo */
			entries = HOST1X_VAL(CHANNEL_FIFOSTAT, OUTFENTRIES,
				readl(chan_regs + HOST1X_CHANNEL_FIFOSTAT));
			if (!entries)
				cpu_relax();
		}

		/*  timeout -> return error */
		if (!entries)
			return -EIO;

		num = min(entries, count);
		entries -= num;
		count -= num;

		while (num & ~0x3) {
			u32 arr[4];
			arr[0] = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
			arr[1] = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
			arr[2] = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
			arr[3] = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
			memcpy(ptr, arr, 4*sizeof(u32));
			ptr += 4;
			num -= 4;
		}
		while (num--)
			*ptr++ = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
	}
	*pending = entries;

	return 0;
}

int host1x_save_context(struct nvhost_module *mod, u32 syncpt_id)
{
	struct nvhost_channel *ch =
			container_of(mod, struct nvhost_channel, mod);
	struct nvhost_hwctx *hwctx_to_save;
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	u32 syncpt_incrs, syncpt_val;
	int err = 0;
	void *ref;
	void *ctx_waiter = NULL, *wakeup_waiter = NULL;
	struct nvhost_job *job;

	ctx_waiter = nvhost_intr_alloc_waiter();
	wakeup_waiter = nvhost_intr_alloc_waiter();
	if (!ctx_waiter || !wakeup_waiter) {
		err = -ENOMEM;
		goto done;
	}

	if (mod->desc->busy)
		mod->desc->busy(mod);

	mutex_lock(&ch->submitlock);
	hwctx_to_save = ch->cur_ctx;
	if (!hwctx_to_save) {
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	job = nvhost_job_alloc(ch, hwctx_to_save,
			NULL,
			ch->dev->nvmap, 0, 0);
	if (IS_ERR_OR_NULL(job)) {
		err = PTR_ERR(job);
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	hwctx_to_save->valid = true;
	ch->ctxhandler.get(hwctx_to_save);
	ch->cur_ctx = NULL;

	syncpt_incrs = hwctx_to_save->save_incrs;
	syncpt_val = nvhost_syncpt_incr_max(&ch->dev->syncpt,
					syncpt_id, syncpt_incrs);

	job->syncpt_id = syncpt_id;
	job->syncpt_incrs = syncpt_incrs;
	job->syncpt_end = syncpt_val;

	err = nvhost_cdma_begin(&ch->cdma, job);
	if (err) {
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	ch->ctxhandler.save_push(&ch->cdma, hwctx_to_save);
	nvhost_cdma_end(&ch->cdma, job);
	nvhost_job_put(job);
	job = NULL;

	err = nvhost_intr_add_action(&ch->dev->intr, syncpt_id,
			syncpt_val - syncpt_incrs + hwctx_to_save->save_thresh,
			NVHOST_INTR_ACTION_CTXSAVE, hwctx_to_save,
			ctx_waiter,
			NULL);
	ctx_waiter = NULL;
	WARN(err, "Failed to set context save interrupt");

	err = nvhost_intr_add_action(&ch->dev->intr, syncpt_id, syncpt_val,
			NVHOST_INTR_ACTION_WAKEUP, &wq,
			wakeup_waiter,
			&ref);
	wakeup_waiter = NULL;
	WARN(err, "Failed to set wakeup interrupt");
	wait_event(wq,
		nvhost_syncpt_min_cmp(&ch->dev->syncpt,
				syncpt_id, syncpt_val));

	nvhost_intr_put_ref(&ch->dev->intr, ref);

	nvhost_cdma_update(&ch->cdma);

	mutex_unlock(&ch->submitlock);

done:
	kfree(ctx_waiter);
	kfree(wakeup_waiter);
	return err;
}
