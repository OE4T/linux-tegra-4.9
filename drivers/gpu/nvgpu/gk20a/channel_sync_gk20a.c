/*
 * GK20A Channel Synchronization Abstraction
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <nvgpu/semaphore.h>
#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/list.h>
#include <nvgpu/nvhost.h>

#include "channel_sync_gk20a.h"
#include "gk20a.h"
#include "fence_gk20a.h"
#include "sync_gk20a.h"
#include "mm_gk20a.h"

#ifdef CONFIG_SYNC
#include "../drivers/staging/android/sync.h"
#endif

#ifdef CONFIG_TEGRA_GK20A_NVHOST

struct gk20a_channel_syncpt {
	struct gk20a_channel_sync ops;
	struct channel_gk20a *c;
	struct nvgpu_nvhost_dev *nvhost_dev;
	u32 id;
	struct nvgpu_mem syncpt_buf;
};

static int gk20a_channel_syncpt_wait_syncpt(struct gk20a_channel_sync *s,
		u32 id, u32 thresh, struct priv_cmd_entry *wait_cmd,
		struct gk20a_fence *fence)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	struct channel_gk20a *c = sp->c;
	int err = 0;

	if (!nvgpu_nvhost_syncpt_is_valid_pt_ext(sp->nvhost_dev, id)) {
		nvgpu_warn(c->g, "invalid wait id in gpfifo submit, elided");
		return 0;
	}

	if (nvgpu_nvhost_syncpt_is_expired_ext(sp->nvhost_dev, id, thresh))
		return 0;

	err = gk20a_channel_alloc_priv_cmdbuf(c,
			c->g->ops.fifo.get_syncpt_wait_cmd_size(), wait_cmd);
	if (err) {
		nvgpu_err(c->g,
				"not enough priv cmd buffer space");
		return err;
	}

	nvgpu_log(c->g, gpu_dbg_info, "sp->id %d gpu va %llx",
					id, sp->syncpt_buf.gpu_va);
	c->g->ops.fifo.add_syncpt_wait_cmd(c->g, wait_cmd, 0, id,
						thresh, sp->syncpt_buf.gpu_va);

	return 0;
}

static int gk20a_channel_syncpt_wait_fd(struct gk20a_channel_sync *s, int fd,
		       struct priv_cmd_entry *wait_cmd,
		       struct gk20a_fence *fence)
{
#ifdef CONFIG_SYNC
	int i;
	int num_wait_cmds;
	struct sync_fence *sync_fence;
	struct sync_pt *pt;
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	struct channel_gk20a *c = sp->c;
	u32 wait_id;
	int err = 0;
	u32 wait_cmd_size = 0;

	sync_fence = nvgpu_nvhost_sync_fdget(fd);
	if (!sync_fence)
		return -EINVAL;

	/* validate syncpt ids */
	for (i = 0; i < sync_fence->num_fences; i++) {
		pt = sync_pt_from_fence(sync_fence->cbs[i].sync_pt);
		wait_id = nvgpu_nvhost_sync_pt_id(pt);
		if (!wait_id || !nvgpu_nvhost_syncpt_is_valid_pt_ext(
					sp->nvhost_dev, wait_id)) {
			sync_fence_put(sync_fence);
			return -EINVAL;
		}
	}

	num_wait_cmds = nvgpu_nvhost_sync_num_pts(sync_fence);
	if (num_wait_cmds == 0) {
		sync_fence_put(sync_fence);
		return 0;
	}
	wait_cmd_size = c->g->ops.fifo.get_syncpt_wait_cmd_size();
	err = gk20a_channel_alloc_priv_cmdbuf(c,
		wait_cmd_size * num_wait_cmds,
		wait_cmd);
	if (err) {
		nvgpu_err(c->g,
				"not enough priv cmd buffer space");
		sync_fence_put(sync_fence);
		return err;
	}

	i = 0;
	for (i = 0; i < sync_fence->num_fences; i++) {
		struct fence *f = sync_fence->cbs[i].sync_pt;
		struct sync_pt *pt = sync_pt_from_fence(f);
		u32 wait_id = nvgpu_nvhost_sync_pt_id(pt);
		u32 wait_value = nvgpu_nvhost_sync_pt_thresh(pt);

		if (nvgpu_nvhost_syncpt_is_expired_ext(sp->nvhost_dev,
				wait_id, wait_value)) {
			nvgpu_memset(c->g, wait_cmd->mem,
			(wait_cmd->off + i * wait_cmd_size) * sizeof(u32),
				0, wait_cmd_size * sizeof(u32));
		} else {
			nvgpu_log(c->g, gpu_dbg_info, "sp->id %d gpu va %llx",
					wait_id, sp->syncpt_buf.gpu_va);
			c->g->ops.fifo.add_syncpt_wait_cmd(c->g, wait_cmd,
				i * wait_cmd_size, wait_id, wait_value,
				sp->syncpt_buf.gpu_va);
		}
	}

	WARN_ON(i != num_wait_cmds);
	sync_fence_put(sync_fence);

	return 0;
#else
	return -ENODEV;
#endif
}

static void gk20a_channel_syncpt_update(void *priv, int nr_completed)
{
	struct channel_gk20a *ch = priv;

	gk20a_channel_update(ch);

	/* note: channel_get() is in __gk20a_channel_syncpt_incr() */
	gk20a_channel_put(ch);
}

static int __gk20a_channel_syncpt_incr(struct gk20a_channel_sync *s,
				       bool wfi_cmd,
				       bool register_irq,
				       struct priv_cmd_entry *incr_cmd,
				       struct gk20a_fence *fence,
				       bool need_sync_fence)
{
	u32 thresh;
	int err;
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	struct channel_gk20a *c = sp->c;

	err = gk20a_channel_alloc_priv_cmdbuf(c,
			c->g->ops.fifo.get_syncpt_incr_cmd_size(wfi_cmd),
			incr_cmd);
	if (err)
		return err;

	nvgpu_log(c->g, gpu_dbg_info, "sp->id %d gpu va %llx",
				sp->id, sp->syncpt_buf.gpu_va);
	c->g->ops.fifo.add_syncpt_incr_cmd(c->g, wfi_cmd,
			incr_cmd, sp->id, sp->syncpt_buf.gpu_va);

	thresh = nvgpu_nvhost_syncpt_incr_max_ext(sp->nvhost_dev, sp->id, 2);

	if (register_irq) {
		struct channel_gk20a *referenced = gk20a_channel_get(c);

		WARN_ON(!referenced);

		if (referenced) {
			/* note: channel_put() is in
			 * gk20a_channel_syncpt_update() */

			err = nvgpu_nvhost_intr_register_notifier(
				sp->nvhost_dev,
				sp->id, thresh,
				gk20a_channel_syncpt_update, c);
			if (err)
				gk20a_channel_put(referenced);

			/* Adding interrupt action should
			 * never fail. A proper error handling
			 * here would require us to decrement
			 * the syncpt max back to its original
			 * value. */
			WARN(err,
			     "failed to set submit complete interrupt");
		}
	}

	err = gk20a_fence_from_syncpt(fence, sp->nvhost_dev, sp->id, thresh,
					 wfi_cmd, need_sync_fence);
	if (err)
		goto clean_up_priv_cmd;

	return 0;

clean_up_priv_cmd:
	gk20a_free_priv_cmdbuf(c, incr_cmd);
	return err;
}

static int gk20a_channel_syncpt_incr_wfi(struct gk20a_channel_sync *s,
				  struct priv_cmd_entry *entry,
				  struct gk20a_fence *fence)
{
	return __gk20a_channel_syncpt_incr(s,
			true /* wfi */,
			false /* no irq handler */,
			entry, fence, true);
}

static int gk20a_channel_syncpt_incr(struct gk20a_channel_sync *s,
			      struct priv_cmd_entry *entry,
			      struct gk20a_fence *fence,
			      bool need_sync_fence,
			      bool register_irq)
{
	/* Don't put wfi cmd to this one since we're not returning
	 * a fence to user space. */
	return __gk20a_channel_syncpt_incr(s,
			false /* no wfi */,
			register_irq /* register irq */,
			entry, fence, need_sync_fence);
}

static int gk20a_channel_syncpt_incr_user(struct gk20a_channel_sync *s,
				   int wait_fence_fd,
				   struct priv_cmd_entry *entry,
				   struct gk20a_fence *fence,
				   bool wfi,
				   bool need_sync_fence,
				   bool register_irq)
{
	/* Need to do 'wfi + host incr' since we return the fence
	 * to user space. */
	return __gk20a_channel_syncpt_incr(s,
			wfi,
			register_irq /* register irq */,
			entry, fence, need_sync_fence);
}

static void gk20a_channel_syncpt_set_min_eq_max(struct gk20a_channel_sync *s)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	nvgpu_nvhost_syncpt_set_min_eq_max_ext(sp->nvhost_dev, sp->id);
}

static void gk20a_channel_syncpt_signal_timeline(
		struct gk20a_channel_sync *s)
{
	/* Nothing to do. */
}

static int gk20a_channel_syncpt_id(struct gk20a_channel_sync *s)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	return sp->id;
}

static void gk20a_channel_syncpt_destroy(struct gk20a_channel_sync *s)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);


	sp->c->g->ops.fifo.free_syncpt_buf(sp->c, &sp->syncpt_buf);

	nvgpu_nvhost_syncpt_set_min_eq_max_ext(sp->nvhost_dev, sp->id);
	nvgpu_nvhost_syncpt_put_ref_ext(sp->nvhost_dev, sp->id);
	nvgpu_kfree(sp->c->g, sp);
}

static struct gk20a_channel_sync *
gk20a_channel_syncpt_create(struct channel_gk20a *c)
{
	struct gk20a_channel_syncpt *sp;
	char syncpt_name[32];

	sp = nvgpu_kzalloc(c->g, sizeof(*sp));
	if (!sp)
		return NULL;

	sp->c = c;
	sp->nvhost_dev = c->g->nvhost_dev;

	snprintf(syncpt_name, sizeof(syncpt_name),
		"%s_%d", c->g->name, c->chid);

	sp->id = nvgpu_nvhost_get_syncpt_host_managed(sp->nvhost_dev,
						c->chid, syncpt_name);
	if (!sp->id) {
		nvgpu_kfree(c->g, sp);
		nvgpu_err(c->g, "failed to get free syncpt");
		return NULL;
	}

	sp->c->g->ops.fifo.alloc_syncpt_buf(sp->c, sp->id,
				&sp->syncpt_buf);

	nvgpu_nvhost_syncpt_set_min_eq_max_ext(sp->nvhost_dev, sp->id);

	nvgpu_atomic_set(&sp->ops.refcount, 0);
	sp->ops.wait_syncpt		= gk20a_channel_syncpt_wait_syncpt;
	sp->ops.wait_fd			= gk20a_channel_syncpt_wait_fd;
	sp->ops.incr			= gk20a_channel_syncpt_incr;
	sp->ops.incr_wfi		= gk20a_channel_syncpt_incr_wfi;
	sp->ops.incr_user		= gk20a_channel_syncpt_incr_user;
	sp->ops.set_min_eq_max		= gk20a_channel_syncpt_set_min_eq_max;
	sp->ops.signal_timeline		= gk20a_channel_syncpt_signal_timeline;
	sp->ops.syncpt_id		= gk20a_channel_syncpt_id;
	sp->ops.destroy			= gk20a_channel_syncpt_destroy;

	return &sp->ops;
}
#endif /* CONFIG_TEGRA_GK20A_NVHOST */

struct gk20a_channel_semaphore {
	struct gk20a_channel_sync ops;
	struct channel_gk20a *c;

	/* A semaphore pool owned by this channel. */
	struct nvgpu_semaphore_pool *pool;

	/* A sync timeline that advances when gpu completes work. */
	struct sync_timeline *timeline;
};

#ifdef CONFIG_SYNC
struct wait_fence_work {
	struct sync_fence_waiter waiter;
	struct sync_fence *fence;
	struct channel_gk20a *ch;
	struct nvgpu_semaphore *sema;
	struct gk20a *g;
	struct nvgpu_list_node entry;
};

static inline struct wait_fence_work *
wait_fence_work_from_entry(struct nvgpu_list_node *node)
{
	return (struct wait_fence_work *)
		((uintptr_t)node - offsetof(struct wait_fence_work, entry));
};

/*
 * Keep track of all the pending waits on semaphores that exist for a GPU. This
 * has to be done because the waits on fences backed by semaphores are
 * asynchronous so it's impossible to otherwise know when they will fire. During
 * driver cleanup this list can be checked and all existing waits can be
 * canceled.
 */
static void gk20a_add_pending_sema_wait(struct gk20a *g,
					struct wait_fence_work *work)
{
	nvgpu_raw_spinlock_acquire(&g->pending_sema_waits_lock);
	nvgpu_list_add(&work->entry, &g->pending_sema_waits);
	nvgpu_raw_spinlock_release(&g->pending_sema_waits_lock);
}

/*
 * Copy the list head from the pending wait list to the passed list and
 * then delete the entire pending list.
 */
static void gk20a_start_sema_wait_cancel(struct gk20a *g,
					 struct nvgpu_list_node *list)
{
	nvgpu_raw_spinlock_acquire(&g->pending_sema_waits_lock);
	nvgpu_list_replace_init(&g->pending_sema_waits, list);
	nvgpu_raw_spinlock_release(&g->pending_sema_waits_lock);
}

/*
 * During shutdown this should be called to make sure that any pending sema
 * waits are canceled. This is a fairly delicate and tricky bit of code. Here's
 * how it works.
 *
 * Every time a semaphore wait is initiated in SW the wait_fence_work struct is
 * added to the pending_sema_waits list. When the semaphore launcher code runs
 * it checks the pending_sema_waits list. If this list is non-empty that means
 * that the wait_fence_work struct must be present and can be removed.
 *
 * When the driver shuts down one of the steps is to cancel pending sema waits.
 * To do this the entire list of pending sema waits is removed (and stored in a
 * separate local list). So now, if the semaphore launcher code runs it will see
 * that the pending_sema_waits list is empty and knows that it no longer owns
 * the wait_fence_work struct.
 */
void gk20a_channel_cancel_pending_sema_waits(struct gk20a *g)
{
	struct wait_fence_work *work;
	struct nvgpu_list_node local_pending_sema_waits;

	gk20a_start_sema_wait_cancel(g, &local_pending_sema_waits);

	while (!nvgpu_list_empty(&local_pending_sema_waits)) {
		int ret;

		work = nvgpu_list_first_entry(&local_pending_sema_waits,
					wait_fence_work,
					entry);

		nvgpu_list_del(&work->entry);

		/*
		 * Only nvgpu_kfree() work if the cancel is successful.
		 * Otherwise it's in use by the
		 * gk20a_channel_semaphore_launcher() code.
		 */
		ret = sync_fence_cancel_async(work->fence, &work->waiter);
		if (ret == 0)
			nvgpu_kfree(g, work);
	}
}

static void gk20a_channel_semaphore_launcher(
		struct sync_fence *fence,
		struct sync_fence_waiter *waiter)
{
	int err;
	struct wait_fence_work *w =
		container_of(waiter, struct wait_fence_work, waiter);
	struct gk20a *g = w->g;

	/*
	 * This spinlock must protect a _very_ small critical section -
	 * otherwise it's possible that the deterministic submit path suffers.
	 */
	nvgpu_raw_spinlock_acquire(&g->pending_sema_waits_lock);
	if (!nvgpu_list_empty(&g->pending_sema_waits))
		nvgpu_list_del(&w->entry);
	nvgpu_raw_spinlock_release(&g->pending_sema_waits_lock);

	gk20a_dbg_info("waiting for pre fence %p '%s'",
			fence, fence->name);
	err = sync_fence_wait(fence, -1);
	if (err < 0)
		nvgpu_err(g, "error waiting pre-fence: %d", err);

	gk20a_dbg_info(
		  "wait completed (%d) for fence %p '%s', triggering gpu work",
		  err, fence, fence->name);
	sync_fence_put(fence);
	nvgpu_semaphore_release(w->sema);
	nvgpu_semaphore_put(w->sema);
	nvgpu_kfree(g, w);
}
#endif

static void add_sema_cmd(struct gk20a *g, struct channel_gk20a *c,
			 struct nvgpu_semaphore *s, struct priv_cmd_entry *cmd,
			 int cmd_size, bool acquire, bool wfi)
{
	int ch = c->chid;
	u32 ob, off = cmd->off;
	u64 va;

	ob = off;

	/*
	 * RO for acquire (since we just need to read the mem) and RW for
	 * release since we will need to write back to the semaphore memory.
	 */
	va = acquire ? nvgpu_semaphore_gpu_ro_va(s) :
		       nvgpu_semaphore_gpu_rw_va(s);

	/*
	 * If the op is not an acquire (so therefor a release) we should
	 * incr the underlying sema next_value.
	 */
	if (!acquire)
		nvgpu_semaphore_incr(s);

	/* semaphore_a */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010004);
	/* offset_upper */
	nvgpu_mem_wr32(g, cmd->mem, off++, (va >> 32) & 0xff);
	/* semaphore_b */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010005);
	/* offset */
	nvgpu_mem_wr32(g, cmd->mem, off++, va & 0xffffffff);

	if (acquire) {
		/* semaphore_c */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010006);
		/* payload */
		nvgpu_mem_wr32(g, cmd->mem, off++,
			       nvgpu_semaphore_get_value(s));
		/* semaphore_d */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010007);
		/* operation: acq_geq, switch_en */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x4 | (0x1 << 12));
	} else {
		/* semaphore_c */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010006);
		/* payload */
		nvgpu_mem_wr32(g, cmd->mem, off++,
			       nvgpu_semaphore_get_value(s));
		/* semaphore_d */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010007);
		/* operation: release, wfi */
		nvgpu_mem_wr32(g, cmd->mem, off++,
				0x2 | ((wfi ? 0x0 : 0x1) << 20));
		/* non_stall_int */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010008);
		/* ignored */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0);
	}

	if (acquire)
		gpu_sema_verbose_dbg(g, "(A) c=%d ACQ_GE %-4u owner=%-3d"
				     "va=0x%llx cmd_mem=0x%llx b=0x%llx off=%u",
				     ch, nvgpu_semaphore_get_value(s),
				     s->hw_sema->ch->chid, va, cmd->gva,
				     cmd->mem->gpu_va, ob);
	else
		gpu_sema_verbose_dbg(g, "(R) c=%d INCR %u (%u) va=0x%llx "
				     "cmd_mem=0x%llx b=0x%llx off=%u",
				     ch, nvgpu_semaphore_get_value(s),
				     nvgpu_semaphore_read(s), va, cmd->gva,
				     cmd->mem->gpu_va, ob);
}

static int gk20a_channel_semaphore_wait_syncpt(
		struct gk20a_channel_sync *s, u32 id,
		u32 thresh, struct priv_cmd_entry *entry,
		struct gk20a_fence *fence)
{
	struct gk20a_channel_semaphore *sema =
		container_of(s, struct gk20a_channel_semaphore, ops);
	struct gk20a *g = sema->c->g;
	nvgpu_err(g, "trying to use syncpoint synchronization");
	return -ENODEV;
}

#ifdef CONFIG_SYNC
/*
 * Attempt a fast path for waiting on a sync_fence. Basically if the passed
 * sync_fence is backed by a nvgpu_semaphore then there's no reason to go
 * through the rigmarole of setting up a separate semaphore which waits on an
 * interrupt from the GPU and then triggers a worker thread to execute a SW
 * based semaphore release. Instead just have the GPU wait on the same semaphore
 * that is going to be incremented by the GPU.
 *
 * This function returns 2 possible values: -ENODEV or 0 on success. In the case
 * of -ENODEV the fastpath cannot be taken due to the fence not being backed by
 * a GPU semaphore.
 */
static int __semaphore_wait_fd_fast_path(struct channel_gk20a *c,
					 struct sync_fence *fence,
					 struct priv_cmd_entry *wait_cmd,
					 struct nvgpu_semaphore **fp_sema)
{
	struct nvgpu_semaphore *sema;
	int err;

	if (!gk20a_is_sema_backed_sync_fence(fence))
		return -ENODEV;

	sema = gk20a_sync_fence_get_sema(fence);

	/*
	 * If there's no underlying sema then that means the underlying sema has
	 * already signaled.
	 */
	if (!sema) {
		*fp_sema = NULL;
		return 0;
	}

	err = gk20a_channel_alloc_priv_cmdbuf(c, 8, wait_cmd);
	if (err)
		return err;

	nvgpu_semaphore_get(sema);
	BUG_ON(!nvgpu_atomic_read(&sema->value));
	add_sema_cmd(c->g, c, sema, wait_cmd, 8, true, false);

	/*
	 * Make sure that gk20a_channel_semaphore_wait_fd() can create another
	 * fence with the underlying semaphore.
	 */
	*fp_sema = sema;

	return 0;
}
#endif

static int gk20a_channel_semaphore_wait_fd(
		struct gk20a_channel_sync *s, int fd,
		struct priv_cmd_entry *entry,
		struct gk20a_fence *fence)
{
	struct gk20a_channel_semaphore *sema =
		container_of(s, struct gk20a_channel_semaphore, ops);
	struct channel_gk20a *c = sema->c;
#ifdef CONFIG_SYNC
	struct nvgpu_semaphore *fp_sema;
	struct sync_fence *sync_fence;
	struct priv_cmd_entry *wait_cmd = entry;
	struct wait_fence_work *w = NULL;
	int err, ret, status;

	sync_fence = gk20a_sync_fence_fdget(fd);
	if (!sync_fence)
		return -EINVAL;

	ret = __semaphore_wait_fd_fast_path(c, sync_fence, wait_cmd, &fp_sema);
	if (ret == 0) {
		if (fp_sema) {
			err = gk20a_fence_from_semaphore(c->g, fence,
					sema->timeline,
					fp_sema,
					&c->semaphore_wq,
					false, false);
			if (err) {
				nvgpu_semaphore_put(fp_sema);
				goto clean_up_priv_cmd;
			}
		} else
			/*
			 * Init an empty fence. It will instantly return
			 * from gk20a_fence_wait().
			 */
			gk20a_init_fence(fence, NULL, NULL, false);

		sync_fence_put(sync_fence);
		goto skip_slow_path;
	}

	/* If the fence has signaled there is no reason to wait on it. */
	status = atomic_read(&sync_fence->status);
	if (status == 0) {
		sync_fence_put(sync_fence);
		goto skip_slow_path;
	}

	err = gk20a_channel_alloc_priv_cmdbuf(c, 8, wait_cmd);
	if (err) {
		nvgpu_err(c->g,
				"not enough priv cmd buffer space");
		goto clean_up_sync_fence;
	}

	w = nvgpu_kzalloc(c->g, sizeof(*w));
	if (!w) {
		err = -ENOMEM;
		goto clean_up_priv_cmd;
	}

	sync_fence_waiter_init(&w->waiter, gk20a_channel_semaphore_launcher);
	w->fence = sync_fence;
	w->g = c->g;
	w->ch = c;
	w->sema = nvgpu_semaphore_alloc(c);
	if (!w->sema) {
		nvgpu_err(c->g, "ran out of semaphores");
		err = -ENOMEM;
		goto clean_up_worker;
	}

	/* worker takes one reference */
	nvgpu_semaphore_get(w->sema);
	nvgpu_semaphore_incr(w->sema);

	/* GPU unblocked when the semaphore value increments. */
	add_sema_cmd(c->g, c, w->sema, wait_cmd, 8, true, false);

	/*
	 *  We need to create the fence before adding the waiter to ensure
	 *  that we properly clean up in the event the sync_fence has
	 *  already signaled
	 */
	err = gk20a_fence_from_semaphore(c->g, fence, sema->timeline, w->sema,
			&c->semaphore_wq, false, false);
	if (err)
		goto clean_up_sema;

	ret = sync_fence_wait_async(sync_fence, &w->waiter);
	gk20a_add_pending_sema_wait(c->g, w);

	/*
	 * If the sync_fence has already signaled then the above async_wait
	 * will never trigger. This causes the semaphore release op to never
	 * happen which, in turn, hangs the GPU. That's bad. So let's just
	 * do the nvgpu_semaphore_release() right now.
	 */
	if (ret == 1) {
		sync_fence_put(sync_fence);
		nvgpu_semaphore_release(w->sema);
		nvgpu_semaphore_put(w->sema);
	}

skip_slow_path:
	return 0;

clean_up_sema:
	/*
	 * Release the refs to the semaphore, including
	 * the one for the worker since it will never run.
	 */
	nvgpu_semaphore_put(w->sema);
	nvgpu_semaphore_put(w->sema);
clean_up_worker:
	nvgpu_kfree(c->g, w);
clean_up_priv_cmd:
	gk20a_free_priv_cmdbuf(c, entry);
clean_up_sync_fence:
	sync_fence_put(sync_fence);
	return err;
#else
	nvgpu_err(c->g,
		  "trying to use sync fds with CONFIG_SYNC disabled");
	return -ENODEV;
#endif
}

static int __gk20a_channel_semaphore_incr(
		struct gk20a_channel_sync *s, bool wfi_cmd,
		struct priv_cmd_entry *incr_cmd,
		struct gk20a_fence *fence,
		bool need_sync_fence)
{
	int incr_cmd_size;
	struct gk20a_channel_semaphore *sp =
		container_of(s, struct gk20a_channel_semaphore, ops);
	struct channel_gk20a *c = sp->c;
	struct nvgpu_semaphore *semaphore;
	int err = 0;

	semaphore = nvgpu_semaphore_alloc(c);
	if (!semaphore) {
		nvgpu_err(c->g,
				"ran out of semaphores");
		return -ENOMEM;
	}

	incr_cmd_size = 10;
	err = gk20a_channel_alloc_priv_cmdbuf(c, incr_cmd_size, incr_cmd);
	if (err) {
		nvgpu_err(c->g,
				"not enough priv cmd buffer space");
		goto clean_up_sema;
	}

	/* Release the completion semaphore. */
	add_sema_cmd(c->g, c, semaphore, incr_cmd, 14, false, wfi_cmd);

	err = gk20a_fence_from_semaphore(c->g, fence,
			sp->timeline, semaphore,
			&c->semaphore_wq,
			wfi_cmd,
			need_sync_fence);
	if (err)
		goto clean_up_sema;

	return 0;

clean_up_sema:
	nvgpu_semaphore_put(semaphore);
	return err;
}

static int gk20a_channel_semaphore_incr_wfi(
		struct gk20a_channel_sync *s,
		struct priv_cmd_entry *entry,
		struct gk20a_fence *fence)
{
	return __gk20a_channel_semaphore_incr(s,
			true /* wfi */,
			entry, fence, true);
}

static int gk20a_channel_semaphore_incr(
		struct gk20a_channel_sync *s,
		struct priv_cmd_entry *entry,
		struct gk20a_fence *fence,
		bool need_sync_fence,
		bool register_irq)
{
	/* Don't put wfi cmd to this one since we're not returning
	 * a fence to user space. */
	return __gk20a_channel_semaphore_incr(s,
			false /* no wfi */,
			entry, fence, need_sync_fence);
}

static int gk20a_channel_semaphore_incr_user(
		struct gk20a_channel_sync *s,
		int wait_fence_fd,
		struct priv_cmd_entry *entry,
		struct gk20a_fence *fence,
		bool wfi,
		bool need_sync_fence,
		bool register_irq)
{
#ifdef CONFIG_SYNC
	int err;

	err = __gk20a_channel_semaphore_incr(s, wfi, entry, fence,
			need_sync_fence);
	if (err)
		return err;

	return 0;
#else
	struct gk20a_channel_semaphore *sema =
		container_of(s, struct gk20a_channel_semaphore, ops);
	nvgpu_err(sema->c->g,
		  "trying to use sync fds with CONFIG_SYNC disabled");
	return -ENODEV;
#endif
}

static void gk20a_channel_semaphore_set_min_eq_max(struct gk20a_channel_sync *s)
{
	/* Nothing to do. */
}

static void gk20a_channel_semaphore_signal_timeline(
		struct gk20a_channel_sync *s)
{
	struct gk20a_channel_semaphore *sp =
		container_of(s, struct gk20a_channel_semaphore, ops);
	gk20a_sync_timeline_signal(sp->timeline);
}

static int gk20a_channel_semaphore_syncpt_id(struct gk20a_channel_sync *s)
{
	return -EINVAL;
}

static void gk20a_channel_semaphore_destroy(struct gk20a_channel_sync *s)
{
	struct gk20a_channel_semaphore *sema =
		container_of(s, struct gk20a_channel_semaphore, ops);
	if (sema->timeline)
		gk20a_sync_timeline_destroy(sema->timeline);

	/* The sema pool is cleaned up by the VM destroy. */
	sema->pool = NULL;

	nvgpu_kfree(sema->c->g, sema);
}

static struct gk20a_channel_sync *
gk20a_channel_semaphore_create(struct channel_gk20a *c)
{
	int asid = -1;
	struct gk20a_channel_semaphore *sema;
	char pool_name[20];

	if (WARN_ON(!c->vm))
		return NULL;

	sema = nvgpu_kzalloc(c->g, sizeof(*sema));
	if (!sema)
		return NULL;
	sema->c = c;

	if (c->vm->as_share)
		asid = c->vm->as_share->id;

	sprintf(pool_name, "semaphore_pool-%d", c->chid);
	sema->pool = c->vm->sema_pool;

#ifdef CONFIG_SYNC
	sema->timeline = gk20a_sync_timeline_create(
			"gk20a_ch%d_as%d", c->chid, asid);
	if (!sema->timeline) {
		gk20a_channel_semaphore_destroy(&sema->ops);
		return NULL;
	}
#endif
	nvgpu_atomic_set(&sema->ops.refcount, 0);
	sema->ops.wait_syncpt	= gk20a_channel_semaphore_wait_syncpt;
	sema->ops.wait_fd	= gk20a_channel_semaphore_wait_fd;
	sema->ops.incr		= gk20a_channel_semaphore_incr;
	sema->ops.incr_wfi	= gk20a_channel_semaphore_incr_wfi;
	sema->ops.incr_user	= gk20a_channel_semaphore_incr_user;
	sema->ops.set_min_eq_max = gk20a_channel_semaphore_set_min_eq_max;
	sema->ops.signal_timeline = gk20a_channel_semaphore_signal_timeline;
	sema->ops.syncpt_id	= gk20a_channel_semaphore_syncpt_id;
	sema->ops.destroy	= gk20a_channel_semaphore_destroy;

	return &sema->ops;
}

void gk20a_channel_sync_destroy(struct gk20a_channel_sync *sync)
{
	sync->destroy(sync);
}

struct gk20a_channel_sync *gk20a_channel_sync_create(struct channel_gk20a *c)
{
#ifdef CONFIG_TEGRA_GK20A_NVHOST
	if (gk20a_platform_has_syncpoints(c->g))
		return gk20a_channel_syncpt_create(c);
#endif
	return gk20a_channel_semaphore_create(c);
}

bool gk20a_channel_sync_needs_sync_framework(struct gk20a *g)
{
	return !gk20a_platform_has_syncpoints(g);
}
