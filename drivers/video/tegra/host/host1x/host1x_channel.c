/*
 * drivers/video/tegra/host/host1x/channel_host1x.c
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#include "nvhost_channel.h"
#include "dev.h"
#include "nvhost_hwctx.h"
#include <trace/events/nvhost.h>
#include <linux/slab.h>

#include "host1x_syncpt.h"
#include "host1x_channel.h"
#include "host1x_hardware.h"
#include "host1x_hwctx.h"
#include "nvhost_intr.h"

#define NV_FIFO_READ_TIMEOUT 200000

static void sync_waitbases(struct nvhost_channel *ch, u32 syncpt_val)
{
	unsigned long waitbase;
	unsigned long int waitbase_mask = ch->dev->waitbases;
	if (ch->dev->waitbasesync) {
		waitbase = find_first_bit(&waitbase_mask, BITS_PER_LONG);
		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				NV_CLASS_HOST_LOAD_SYNCPT_BASE,
				1),
				nvhost_class_host_load_syncpt_base(waitbase,
						syncpt_val));
	}
}

static void *pre_submit_ctxsave(struct nvhost_job *job,
		struct nvhost_hwctx *cur_ctx)
{
	struct nvhost_channel *ch = job->ch;
	void *ctxsave_waiter = NULL;

	/* Is a save needed? */
	if (!cur_ctx || ch->cur_ctx == job->hwctx)
		return NULL;

	if (cur_ctx->has_timedout) {
		dev_dbg(&ch->dev->dev,
			"%s: skip save of timed out context (0x%p)\n",
			__func__, ch->cur_ctx);

		return NULL;
	}

	/* Allocate save waiter if needed */
	if (ch->ctxhandler->save_service) {
		ctxsave_waiter = nvhost_intr_alloc_waiter();
		if (!ctxsave_waiter)
			return ERR_PTR(-ENOMEM);
	}

	return ctxsave_waiter;
}

static void submit_ctxsave(struct nvhost_job *job, void *ctxsave_waiter,
		struct nvhost_hwctx *cur_ctx)
{
	struct nvhost_master *host = nvhost_get_host(job->ch->dev);
	struct nvhost_channel *ch = job->ch;
	u32 syncval;
	int err;
	u32 save_thresh = 0;

	/* Is a save needed? */
	if (!cur_ctx || cur_ctx == job->hwctx || cur_ctx->has_timedout)
		return;

	/* Retrieve save threshold if we have a waiter */
	if (ctxsave_waiter)
		save_thresh =
			nvhost_syncpt_read_max(&host->syncpt, job->syncpt_id)
			+ to_host1x_hwctx(cur_ctx)->save_thresh;

	/* Adjust the syncpoint max */
	job->syncpt_incrs += to_host1x_hwctx(cur_ctx)->save_incrs;
	syncval = nvhost_syncpt_incr_max(&host->syncpt,
			job->syncpt_id,
			to_host1x_hwctx(cur_ctx)->save_incrs);

	/* Send the save to channel */
	cur_ctx->valid = true;
	ch->ctxhandler->save_push(cur_ctx, &ch->cdma);
	nvhost_job_get_hwctx(job, cur_ctx);

	/* Notify save service */
	if (ctxsave_waiter) {
		err = nvhost_intr_add_action(&host->intr,
			job->syncpt_id,
			save_thresh,
			NVHOST_INTR_ACTION_CTXSAVE, cur_ctx,
			ctxsave_waiter,
			NULL);
		ctxsave_waiter = NULL;
		WARN(err, "Failed to set ctx save interrupt");
	}

	trace_nvhost_channel_context_save(ch->dev->name, cur_ctx);
}

static void submit_ctxrestore(struct nvhost_job *job)
{
	struct nvhost_master *host = nvhost_get_host(job->ch->dev);
	struct nvhost_channel *ch = job->ch;
	u32 syncval;
	struct host1x_hwctx *ctx =
		job->hwctx ? to_host1x_hwctx(job->hwctx) : NULL;

	/* First check if we have a valid context to restore */
	if(ch->cur_ctx == job->hwctx || !job->hwctx || !job->hwctx->valid)
		return;

	/* Increment syncpt max */
	job->syncpt_incrs += ctx->restore_incrs;
	syncval = nvhost_syncpt_incr_max(&host->syncpt,
			job->syncpt_id,
			ctx->restore_incrs);

	/* Send restore buffer to channel */
	nvhost_cdma_push_gather(&ch->cdma,
		host->nvmap,
		nvmap_ref_to_handle(ctx->restore),
		nvhost_opcode_gather(ctx->restore_size),
		ctx->restore_phys);

	trace_nvhost_channel_context_restore(ch->dev->name, &ctx->hwctx);
}

void submit_nullkickoff(struct nvhost_job *job, int user_syncpt_incrs)
{
	struct nvhost_channel *ch = job->ch;
	int incr;
	u32 op_incr;

	/* push increments that correspond to nulled out commands */
	op_incr = nvhost_opcode_imm_incr_syncpt(NV_SYNCPT_OP_DONE,
			job->syncpt_id);
	for (incr = 0; incr < (user_syncpt_incrs >> 1); incr++)
		nvhost_cdma_push(&ch->cdma, op_incr, op_incr);
	if (user_syncpt_incrs & 1)
		nvhost_cdma_push(&ch->cdma, op_incr, NVHOST_OPCODE_NOOP);

	/* for 3d, waitbase needs to be incremented after each submit */
	if (ch->dev->class == NV_GRAPHICS_3D_CLASS_ID) {
		u32 waitbase = to_host1x_hwctx_handler(job->hwctx->h)->waitbase;
		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(
				NV_HOST1X_CLASS_ID,
				NV_CLASS_HOST_INCR_SYNCPT_BASE,
				1),
			nvhost_class_host_incr_syncpt_base(
				waitbase,
				user_syncpt_incrs));
	}
}

void submit_gathers(struct nvhost_job *job)
{
	/* push user gathers */
	int i = 0;
	for ( ; i < job->num_gathers; i++) {
		u32 op1 = nvhost_opcode_gather(job->gathers[i].words);
		u32 op2 = job->gathers[i].mem;
		nvhost_cdma_push_gather(&job->ch->cdma,
				job->nvmap, job->unpins[i/2],
				op1, op2);
	}
}

int host1x_channel_submit(struct nvhost_job *job)
{
	struct nvhost_channel *ch = job->ch;
	struct nvhost_syncpt *sp = &nvhost_get_host(job->ch->dev)->syncpt;
	u32 user_syncpt_incrs = job->syncpt_incrs;
	u32 prev_max = 0;
	u32 syncval;
	int err;
	void *completed_waiter = NULL, *ctxsave_waiter = NULL;

	/* Bail out on timed out contexts */
	if (job->hwctx && job->hwctx->has_timedout)
		return -ETIMEDOUT;

	/* Turn on the client module and host1x */
	nvhost_module_busy(ch->dev);
	if (ch->dev->busy)
		ch->dev->busy(ch->dev);

	/* before error checks, return current max */
	prev_max = job->syncpt_end =
		nvhost_syncpt_read_max(sp, job->syncpt_id);

	/* get submit lock */
	err = mutex_lock_interruptible(&ch->submitlock);
	if (err) {
		nvhost_module_idle(ch->dev);
		goto error;
	}

	/* Do the needed allocations */
	ctxsave_waiter = pre_submit_ctxsave(job, ch->cur_ctx);
	if (IS_ERR(ctxsave_waiter)) {
		err = PTR_ERR(ctxsave_waiter);
		nvhost_module_idle(ch->dev);
		mutex_unlock(&ch->submitlock);
		goto error;
	}

	completed_waiter = nvhost_intr_alloc_waiter();
	if (!completed_waiter) {
		nvhost_module_idle(ch->dev);
		mutex_unlock(&ch->submitlock);
		err = -ENOMEM;
		goto error;
	}

	/* remove stale waits */
	if (job->num_waitchk) {
		err = nvhost_syncpt_wait_check(sp,
					       job->nvmap,
					       job->waitchk_mask,
					       job->waitchk,
					       job->num_waitchk);
		if (err) {
			dev_warn(&ch->dev->dev,
				 "nvhost_syncpt_wait_check failed: %d\n", err);
			mutex_unlock(&ch->submitlock);
			nvhost_module_idle(ch->dev);
			goto error;
		}
	}

	/* begin a CDMA submit */
	err = nvhost_cdma_begin(&ch->cdma, job);
	if (err) {
		mutex_unlock(&ch->submitlock);
		nvhost_module_idle(ch->dev);
		goto error;
	}

	submit_ctxsave(job, ctxsave_waiter, ch->cur_ctx);
	submit_ctxrestore(job);
	ch->cur_ctx = job->hwctx;

	syncval = nvhost_syncpt_incr_max(sp,
			job->syncpt_id, user_syncpt_incrs);

	job->syncpt_end = syncval;

	/* add a setclass for modules that require it */
	if (ch->dev->class)
		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(ch->dev->class, 0, 0),
			NVHOST_OPCODE_NOOP);

	if (job->null_kickoff)
		submit_nullkickoff(job, user_syncpt_incrs);
	else
		submit_gathers(job);

	sync_waitbases(ch, job->syncpt_end);

	/* end CDMA submit & stash pinned hMems into sync queue */
	nvhost_cdma_end(&ch->cdma, job);

	trace_nvhost_channel_submitted(ch->dev->name,
			prev_max, syncval);

	/* schedule a submit complete interrupt */
	err = nvhost_intr_add_action(&nvhost_get_host(ch->dev)->intr,
			job->syncpt_id, syncval,
			NVHOST_INTR_ACTION_SUBMIT_COMPLETE, ch,
			completed_waiter,
			NULL);
	completed_waiter = NULL;
	WARN(err, "Failed to set submit complete interrupt");

	mutex_unlock(&ch->submitlock);

	return 0;

error:
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
	struct host1x_hwctx *hwctx_to_save = NULL;
	struct nvhost_hwctx_handler *h = hwctx->h;
	struct host1x_hwctx_handler *p = to_host1x_hwctx_handler(h);
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
			nvhost_get_host(channel->dev)->nvmap, 0, 0);
	if (!job) {
		err = -ENOMEM;
		goto done;
	}

	/* keep module powered */
	nvhost_module_busy(channel->dev);

	/* get submit lock */
	err = mutex_lock_interruptible(&channel->submitlock);
	if (err) {
		nvhost_module_idle(channel->dev);
		return err;
	}

	/* context switch */
	if (channel->cur_ctx != hwctx) {
		hwctx_to_save = channel->cur_ctx ?
			to_host1x_hwctx(channel->cur_ctx) : NULL;
		if (hwctx_to_save) {
			syncpt_incrs += hwctx_to_save->save_incrs;
			hwctx_to_save->hwctx.valid = true;
			channel->ctxhandler->get(&hwctx_to_save->hwctx);
		}
		channel->cur_ctx = hwctx;
		if (channel->cur_ctx && channel->cur_ctx->valid) {
			need_restore = true;
			syncpt_incrs += to_host1x_hwctx(channel->cur_ctx)
				->restore_incrs;
		}
	}

	syncval = nvhost_syncpt_incr_max(&nvhost_get_host(channel->dev)->syncpt,
		p->syncpt, syncpt_incrs);

	job->syncpt_id = p->syncpt;
	job->syncpt_incrs = syncpt_incrs;
	job->syncpt_end = syncval;

	/* begin a CDMA submit */
	nvhost_cdma_begin(&channel->cdma, job);

	/* push save buffer (pre-gather setup depends on unit) */
	if (hwctx_to_save)
		h->save_push(&hwctx_to_save->hwctx, &channel->cdma);

	/* gather restore buffer */
	if (need_restore)
		nvhost_cdma_push(&channel->cdma,
			nvhost_opcode_gather(to_host1x_hwctx(channel->cur_ctx)
				->restore_size),
			to_host1x_hwctx(channel->cur_ctx)->restore_phys);

	/* Switch to 3D - wait for it to complete what it was doing */
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0),
		nvhost_opcode_imm_incr_syncpt(NV_SYNCPT_OP_DONE,
			p->syncpt));
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
			NV_CLASS_HOST_WAIT_SYNCPT_BASE, 1),
		nvhost_class_host_wait_syncpt_base(p->syncpt,
			p->waitbase, 1));
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
			p->syncpt),
		NVHOST_OPCODE_NOOP);
	/*  Wait for value to be read from FIFO */
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_nonincr(NV_CLASS_HOST_WAIT_SYNCPT_BASE, 1),
		nvhost_class_host_wait_syncpt_base(p->syncpt,
			p->waitbase, 3));
	/*  Indicate submit complete */
	nvhost_cdma_push(&channel->cdma,
		nvhost_opcode_nonincr(NV_CLASS_HOST_INCR_SYNCPT_BASE, 1),
		nvhost_class_host_incr_syncpt_base(p->waitbase, 4));
	nvhost_cdma_push(&channel->cdma,
		NVHOST_OPCODE_NOOP,
		nvhost_opcode_imm_incr_syncpt(NV_SYNCPT_IMMEDIATE,
			p->syncpt));

	/* end CDMA submit  */
	nvhost_cdma_end(&channel->cdma, job);
	nvhost_job_put(job);
	job = NULL;

	/*
	 * schedule a context save interrupt (to drain the host FIFO
	 * if necessary, and to release the restore buffer)
	 */
	if (hwctx_to_save) {
		err = nvhost_intr_add_action(
			&nvhost_get_host(channel->dev)->intr,
			p->syncpt,
			syncval - syncpt_incrs
				+ hwctx_to_save->save_incrs
				- 1,
			NVHOST_INTR_ACTION_CTXSAVE, hwctx_to_save,
			ctx_waiter,
			NULL);
		ctx_waiter = NULL;
		WARN(err, "Failed to set context save interrupt");
	}

	/* Wait for FIFO to be ready */
	err = nvhost_intr_add_action(&nvhost_get_host(channel->dev)->intr,
			p->syncpt, syncval - 2,
			NVHOST_INTR_ACTION_WAKEUP, &wq,
			read_waiter,
			&ref);
	read_waiter = NULL;
	WARN(err, "Failed to set wakeup interrupt");
	wait_event(wq,
		nvhost_syncpt_is_expired(&nvhost_get_host(channel->dev)->syncpt,
				p->syncpt, syncval - 2));
	nvhost_intr_put_ref(&nvhost_get_host(channel->dev)->intr, ref);

	/* Read the register value from FIFO */
	err = host1x_drain_read_fifo(channel->aperture,
		value, 1, &pending);

	/* Indicate we've read the value */
	nvhost_syncpt_cpu_incr(&nvhost_get_host(channel->dev)->syncpt,
			p->syncpt);

	/* Schedule a submit complete interrupt */
	err = nvhost_intr_add_action(&nvhost_get_host(channel->dev)->intr,
			p->syncpt, syncval,
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

int host1x_save_context(struct nvhost_device *dev, u32 syncpt_id)
{
	struct nvhost_channel *ch = dev->channel;
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

	if (dev->busy)
		dev->busy(dev);

	mutex_lock(&ch->submitlock);
	hwctx_to_save = ch->cur_ctx;
	if (!hwctx_to_save) {
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	job = nvhost_job_alloc(ch, hwctx_to_save,
			NULL,
			nvhost_get_host(ch->dev)->nvmap, 0, 0);
	if (IS_ERR_OR_NULL(job)) {
		err = PTR_ERR(job);
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	hwctx_to_save->valid = true;
	ch->ctxhandler->get(hwctx_to_save);
	ch->cur_ctx = NULL;

	syncpt_incrs = to_host1x_hwctx(hwctx_to_save)->save_incrs;
	syncpt_val = nvhost_syncpt_incr_max(&nvhost_get_host(ch->dev)->syncpt,
					syncpt_id, syncpt_incrs);

	job->syncpt_id = syncpt_id;
	job->syncpt_incrs = syncpt_incrs;
	job->syncpt_end = syncpt_val;

	err = nvhost_cdma_begin(&ch->cdma, job);
	if (err) {
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	ch->ctxhandler->save_push(hwctx_to_save, &ch->cdma);
	nvhost_cdma_end(&ch->cdma, job);
	nvhost_job_put(job);
	job = NULL;

	err = nvhost_intr_add_action(&nvhost_get_host(ch->dev)->intr, syncpt_id,
			syncpt_val - syncpt_incrs +
				to_host1x_hwctx(hwctx_to_save)->save_thresh,
			NVHOST_INTR_ACTION_CTXSAVE, hwctx_to_save,
			ctx_waiter,
			NULL);
	ctx_waiter = NULL;
	WARN(err, "Failed to set context save interrupt");

	err = nvhost_intr_add_action(&nvhost_get_host(ch->dev)->intr,
			syncpt_id, syncpt_val,
			NVHOST_INTR_ACTION_WAKEUP, &wq,
			wakeup_waiter,
			&ref);
	wakeup_waiter = NULL;
	WARN(err, "Failed to set wakeup interrupt");
	wait_event(wq,
		nvhost_syncpt_is_expired(&nvhost_get_host(ch->dev)->syncpt,
				syncpt_id, syncpt_val));

	nvhost_intr_put_ref(&nvhost_get_host(ch->dev)->intr, ref);

	nvhost_cdma_update(&ch->cdma);

	mutex_unlock(&ch->submitlock);

done:
	kfree(ctx_waiter);
	kfree(wakeup_waiter);
	return err;
}
