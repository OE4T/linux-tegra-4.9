/*
 * drivers/video/tegra/host/nvhost_syncpt.c
 *
 * Tegra Graphics Host Syncpoints
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

#include <linux/nvhost_ioctl.h>
#include <linux/platform_device.h>
#include "nvhost_syncpt.h"
#include "dev.h"

#define MAX_STUCK_CHECK_COUNT 15

/**
 * Resets syncpoint and waitbase values to sw shadows
 */
void nvhost_syncpt_reset(struct nvhost_syncpt *sp)
{
	u32 i;
	BUG_ON(!(syncpt_op(sp).reset && syncpt_op(sp).reset_wait_base));

	for (i = 0; i < sp->nb_pts; i++)
		syncpt_op(sp).reset(sp, i);
	for (i = 0; i < sp->nb_bases; i++)
		syncpt_op(sp).reset_wait_base(sp, i);
	wmb();
}

/**
 * Updates sw shadow state for client managed registers
 */
void nvhost_syncpt_save(struct nvhost_syncpt *sp)
{
	u32 i;
	BUG_ON(!(syncpt_op(sp).update_min && syncpt_op(sp).read_wait_base));

	for (i = 0; i < sp->nb_pts; i++) {
		if (client_managed(i))
			syncpt_op(sp).update_min(sp, i);
		else
			BUG_ON(!nvhost_syncpt_min_eq_max(sp, i));
	}

	for (i = 0; i < sp->nb_bases; i++)
		syncpt_op(sp).read_wait_base(sp, i);
}

/**
 * Updates the last value read from hardware.
 */
u32 nvhost_syncpt_update_min(struct nvhost_syncpt *sp, u32 id)
{
	BUG_ON(!syncpt_op(sp).update_min);

	return syncpt_op(sp).update_min(sp, id);
}

/**
 * Get the current syncpoint value
 */
u32 nvhost_syncpt_read(struct nvhost_syncpt *sp, u32 id)
{
	u32 val;
	BUG_ON(!syncpt_op(sp).update_min);
	nvhost_module_busy(syncpt_to_dev(sp)->dev);
	val = syncpt_op(sp).update_min(sp, id);
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
	return val;
}

/**
 * Get the current syncpoint base
 */
u32 nvhost_syncpt_read_wait_base(struct nvhost_syncpt *sp, u32 id)
{
	u32 val;
	BUG_ON(!syncpt_op(sp).read_wait_base);
	nvhost_module_busy(syncpt_to_dev(sp)->dev);
	syncpt_op(sp).read_wait_base(sp, id);
	val = sp->base_val[id];
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
	return val;
}

/**
 * Write a cpu syncpoint increment to the hardware, without touching
 * the cache. Caller is responsible for host being powered.
 */
void nvhost_syncpt_cpu_incr(struct nvhost_syncpt *sp, u32 id)
{
	BUG_ON(!syncpt_op(sp).cpu_incr);
	syncpt_op(sp).cpu_incr(sp, id);
}

/**
 * Increment syncpoint value from cpu, updating cache
 */
void nvhost_syncpt_incr(struct nvhost_syncpt *sp, u32 id)
{
	if (client_managed(id))
		nvhost_syncpt_incr_max(sp, id, 1);
	nvhost_module_busy(syncpt_to_dev(sp)->dev);
	nvhost_syncpt_cpu_incr(sp, id);
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
}

/**
 * Main entrypoint for syncpoint value waits.
 */
int nvhost_syncpt_wait_timeout(struct nvhost_syncpt *sp, u32 id,
			u32 thresh, u32 timeout, u32 *value)
{
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	void *ref;
	void *waiter;
	int err = 0, check_count = 0, low_timeout = 0;
	u32 val;

	if (value)
		*value = 0;

	/* first check cache */
	if (nvhost_syncpt_is_expired(sp, id, thresh)) {
		if (value)
			*value = nvhost_syncpt_read_min(sp, id);
		return 0;
	}

	/* keep host alive */
	nvhost_module_busy(syncpt_to_dev(sp)->dev);

	/* try to read from register */
	val = syncpt_op(sp).update_min(sp, id);
	if (nvhost_syncpt_is_expired(sp, id, thresh)) {
		if (value)
			*value = val;
		goto done;
	}

	if (!timeout) {
		err = -EAGAIN;
		goto done;
	}

	/* schedule a wakeup when the syncpoint value is reached */
	waiter = nvhost_intr_alloc_waiter();
	if (!waiter) {
		err = -ENOMEM;
		goto done;
	}

	err = nvhost_intr_add_action(&(syncpt_to_dev(sp)->intr), id, thresh,
				NVHOST_INTR_ACTION_WAKEUP_INTERRUPTIBLE, &wq,
				waiter,
				&ref);
	if (err)
		goto done;

	err = -EAGAIN;
	/* Caller-specified timeout may be impractically low */
	if (timeout < SYNCPT_CHECK_PERIOD)
		low_timeout = timeout;

	/* wait for the syncpoint, or timeout, or signal */
	while (timeout) {
		u32 check = min_t(u32, SYNCPT_CHECK_PERIOD, timeout);
		int remain = wait_event_interruptible_timeout(wq,
				nvhost_syncpt_is_expired(sp, id, thresh),
				check);
		if (remain > 0) {
			if (value)
				*value = nvhost_syncpt_read_min(sp, id);
			err = 0;
			break;
		}
		if (remain < 0) {
			err = remain;
			break;
		}
		if (timeout != NVHOST_NO_TIMEOUT)
			timeout -= check;
		if (timeout) {
			dev_warn(&syncpt_to_dev(sp)->dev->dev,
				"%s: syncpoint id %d (%s) stuck waiting %d, timeout=%d\n",
				 current->comm, id, syncpt_op(sp).name(sp, id),
				 thresh, timeout);
			syncpt_op(sp).debug(sp);
			if (check_count > MAX_STUCK_CHECK_COUNT) {
				if (low_timeout) {
					dev_warn(&syncpt_to_dev(sp)->dev->dev,
						"is timeout %d too low?\n",
						low_timeout);
				}
				nvhost_debug_dump(syncpt_to_dev(sp));
				BUG();
			}
			check_count++;
		}
	}
	nvhost_intr_put_ref(&(syncpt_to_dev(sp)->intr), ref);

done:
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
	return err;
}

/**
 * Returns true if syncpoint is expired, false if we may need to wait
 */
bool nvhost_syncpt_is_expired(
	struct nvhost_syncpt *sp,
	u32 id,
	u32 thresh)
{
	u32 current_val;
	u32 future_val;
	smp_rmb();
	current_val = (u32)atomic_read(&sp->min_val[id]);
	future_val = (u32)atomic_read(&sp->max_val[id]);

	/* Note the use of unsigned arithmetic here (mod 1<<32).
	 *
	 * c = current_val = min_val	= the current value of the syncpoint.
	 * t = thresh			= the value we are checking
	 * f = future_val  = max_val	= the value c will reach when all
	 *			   	  outstanding increments have completed.
	 *
	 * Note that c always chases f until it reaches f.
	 *
	 * Dtf = (f - t)
	 * Dtc = (c - t)
	 *
	 *  Consider all cases:
	 *
	 *	A) .....c..t..f.....	Dtf < Dtc	need to wait
	 *	B) .....c.....f..t..	Dtf > Dtc	expired
	 *	C) ..t..c.....f.....	Dtf > Dtc	expired	   (Dct very large)
	 *
	 *  Any case where f==c: always expired (for any t).  	Dtf == Dcf
	 *  Any case where t==c: always expired (for any f).  	Dtf >= Dtc (because Dtc==0)
	 *  Any case where t==f!=c: always wait.	 	Dtf <  Dtc (because Dtf==0,
	 *							Dtc!=0)
	 *
	 *  Other cases:
	 *
	 *	A) .....t..f..c.....	Dtf < Dtc	need to wait
	 *	A) .....f..c..t.....	Dtf < Dtc	need to wait
	 *	A) .....f..t..c.....	Dtf > Dtc	expired
	 *
	 *   So:
	 *	   Dtf >= Dtc implies EXPIRED	(return true)
	 *	   Dtf <  Dtc implies WAIT	(return false)
	 *
	 * Note: If t is expired then we *cannot* wait on it. We would wait
	 * forever (hang the system).
	 *
	 * Note: do NOT get clever and remove the -thresh from both sides. It
	 * is NOT the same.
	 *
	 * If future valueis zero, we have a client managed sync point. In that
	 * case we do a direct comparison.
	 */
	if (!client_managed(id))
		return future_val - thresh >= current_val - thresh;
	else
		return (s32)(current_val - thresh) >= 0;
}

void nvhost_syncpt_debug(struct nvhost_syncpt *sp)
{
	syncpt_op(sp).debug(sp);
}

int nvhost_mutex_try_lock(struct nvhost_syncpt *sp, int idx)
{
	struct nvhost_master *host = syncpt_to_dev(sp);
	u32 reg;

	nvhost_module_busy(host->dev);
	reg = syncpt_op(sp).mutex_try_lock(sp, idx);
	if (reg) {
		nvhost_module_idle(host->dev);
		return -EBUSY;
	}
	atomic_inc(&sp->lock_counts[idx]);
	return 0;
}

void nvhost_mutex_unlock(struct nvhost_syncpt *sp, int idx)
{
	syncpt_op(sp).mutex_unlock(sp, idx);
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
	atomic_dec(&sp->lock_counts[idx]);
}

/* check for old WAITs to be removed (avoiding a wrap) */
int nvhost_syncpt_wait_check(struct nvhost_syncpt *sp,
			     struct nvmap_client *nvmap,
			     u32 waitchk_mask,
			     struct nvhost_waitchk *wait,
			     int num_waitchk)
{
	return syncpt_op(sp).wait_check(sp, nvmap,
			waitchk_mask, wait, num_waitchk);
}
