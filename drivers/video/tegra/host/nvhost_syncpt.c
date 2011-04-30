/*
 * drivers/video/tegra/host/nvhost_syncpt.c
 *
 * Tegra Graphics Host Syncpoints
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include "nvhost_syncpt.h"
#include "dev.h"

#define client_managed(id) (BIT(id) & NVSYNCPTS_CLIENT_MANAGED)
#define syncpt_to_dev(sp) container_of(sp, struct nvhost_master, syncpt)
#define SYNCPT_CHECK_PERIOD 2*HZ

static bool check_max(struct nvhost_syncpt *sp, u32 id, u32 real)
{
	u32 max;
	if (client_managed(id))
		return true;
	smp_rmb();
	max = (u32)atomic_read(&sp->max_val[id]);
	return ((s32)(max - real) >= 0);
}

/**
 * Write the current syncpoint value back to hw.
 */
static void reset_syncpt(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);
	int min;
	smp_rmb();
	min = atomic_read(&sp->min_val[id]);
	writel(min, dev->sync_aperture + (HOST1X_SYNC_SYNCPT_0 + id * 4));
}

/**
 * Write the current waitbase value back to hw.
 */
static void reset_syncpt_wait_base(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);
	writel(sp->base_val[id],
		dev->sync_aperture + (HOST1X_SYNC_SYNCPT_BASE_0 + id * 4));
}

/**
 * Read waitbase value from hw.
 */
static void read_syncpt_wait_base(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);
	sp->base_val[id] = readl(dev->sync_aperture +
				(HOST1X_SYNC_SYNCPT_BASE_0 + id * 4));
}

/**
 * Resets syncpoint and waitbase values to sw shadows
 */
void nvhost_syncpt_reset(struct nvhost_syncpt *sp)
{
	u32 i;
	for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++)
		reset_syncpt(sp, i);
	for (i = 0; i < NV_HOST1X_SYNCPT_NB_BASES; i++)
		reset_syncpt_wait_base(sp, i);
	wmb();
}

/**
 * Updates sw shadow state for client managed registers
 */
void nvhost_syncpt_save(struct nvhost_syncpt *sp)
{
	u32 i;

	for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++) {
		if (client_managed(i))
			nvhost_syncpt_update_min(sp, i);
		else
			BUG_ON(!nvhost_syncpt_min_eq_max(sp, i));
	}

	for (i = 0; i < NV_HOST1X_SYNCPT_NB_BASES; i++)
		read_syncpt_wait_base(sp, i);
}

/**
 * Updates the last value read from hardware.
 */
u32 nvhost_syncpt_update_min(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);
	void __iomem *sync_regs = dev->sync_aperture;
	u32 old, live;

	do {
		smp_rmb();
		old = (u32)atomic_read(&sp->min_val[id]);
		live = readl(sync_regs + (HOST1X_SYNC_SYNCPT_0 + id * 4));
	} while ((u32)atomic_cmpxchg(&sp->min_val[id], old, live) != old);

	BUG_ON(!check_max(sp, id, live));

	return live;
}

/**
 * Get the current syncpoint value
 */
u32 nvhost_syncpt_read(struct nvhost_syncpt *sp, u32 id)
{
	u32 val;

	nvhost_module_busy(&syncpt_to_dev(sp)->mod);
	val = nvhost_syncpt_update_min(sp, id);
	nvhost_module_idle(&syncpt_to_dev(sp)->mod);
	return val;
}

/**
 * Write a cpu syncpoint increment to the hardware, without touching
 * the cache. Caller is responsible for host being powered.
 */
void nvhost_syncpt_cpu_incr(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);
	BUG_ON(!nvhost_module_powered(&dev->mod));
	BUG_ON(!client_managed(id) && nvhost_syncpt_min_eq_max(sp, id));
	writel(BIT(id), dev->sync_aperture + HOST1X_SYNC_SYNCPT_CPU_INCR);
	wmb();
}

/**
 * Increment syncpoint value from cpu, updating cache
 */
void nvhost_syncpt_incr(struct nvhost_syncpt *sp, u32 id)
{
	nvhost_syncpt_incr_max(sp, id, 1);
	nvhost_module_busy(&syncpt_to_dev(sp)->mod);
	nvhost_syncpt_cpu_incr(sp, id);
	nvhost_module_idle(&syncpt_to_dev(sp)->mod);
}

/**
 * Main entrypoint for syncpoint value waits.
 */
int nvhost_syncpt_wait_timeout(struct nvhost_syncpt *sp, u32 id,
			u32 thresh, u32 timeout, u32 *value)
{
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	void *ref;
	int err = 0;

	if (value)
		*value = 0;

	BUG_ON(!check_max(sp, id, thresh));

	/* first check cache */
	if (nvhost_syncpt_min_cmp(sp, id, thresh)) {
		if (value)
			*value = nvhost_syncpt_read_min(sp, id);
		return 0;
	}

	/* keep host alive */
	nvhost_module_busy(&syncpt_to_dev(sp)->mod);

	if (client_managed(id) || !nvhost_syncpt_min_eq_max(sp, id)) {
		/* try to read from register */
		u32 val = nvhost_syncpt_update_min(sp, id);
		if ((s32)(val - thresh) >= 0) {
			if (value)
				*value = val;
			goto done;
		}
	}

	if (!timeout) {
		err = -EAGAIN;
		goto done;
	}

	/* schedule a wakeup when the syncpoint value is reached */
	err = nvhost_intr_add_action(&(syncpt_to_dev(sp)->intr), id, thresh,
				NVHOST_INTR_ACTION_WAKEUP_INTERRUPTIBLE, &wq, &ref);
	if (err)
		goto done;

	err = -EAGAIN;
	/* wait for the syncpoint, or timeout, or signal */
	while (timeout) {
		u32 check = min_t(u32, SYNCPT_CHECK_PERIOD, timeout);
		int remain = wait_event_interruptible_timeout(wq,
						nvhost_syncpt_min_cmp(sp, id, thresh),
						check);
		if (remain > 0 || nvhost_syncpt_min_cmp(sp, id, thresh)) {
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
			dev_warn(&syncpt_to_dev(sp)->pdev->dev,
				"syncpoint id %d (%s) stuck waiting %d\n",
				id, nvhost_syncpt_name(id), thresh);
			nvhost_syncpt_debug(sp);
		}
	}
	nvhost_intr_put_ref(&(syncpt_to_dev(sp)->intr), ref);

done:
	nvhost_module_idle(&syncpt_to_dev(sp)->mod);
	return err;
}

static const char *s_syncpt_names[32] = {
	"gfx_host", "", "", "", "", "", "", "", "", "", "",
	"csi_vi_0", "csi_vi_1", "vi_isp_0", "vi_isp_1", "vi_isp_2", "vi_isp_3", "vi_isp_4",
	"2d_0", "2d_1",
	"", "",
	"3d", "mpe", "disp0", "disp1", "vblank0", "vblank1", "mpe_ebm_eof", "mpe_wr_safe",
	"2d_tinyblt", "dsi"
};

const char *nvhost_syncpt_name(u32 id)
{
	BUG_ON(id > ARRAY_SIZE(s_syncpt_names));
	return s_syncpt_names[id];
}

void nvhost_syncpt_debug(struct nvhost_syncpt *sp)
{
	u32 i;
	for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++) {
		u32 max = nvhost_syncpt_read_max(sp, i);
		if (!max)
			continue;
		dev_info(&syncpt_to_dev(sp)->pdev->dev,
			"id %d (%s) min %d max %d\n",
			i, nvhost_syncpt_name(i),
			nvhost_syncpt_update_min(sp, i), max);

	}
}

/* returns true, if a <= b < c using wrapping comparison */
static inline bool nvhost_syncpt_is_between(u32 a, u32 b, u32 c)
{
	return b-a < c-a;
}

/* returns true, if syncpt >= threshold (mod 1 << 32) */
static bool nvhost_syncpt_wrapping_comparison(u32 syncpt, u32 threshold)
{
	return nvhost_syncpt_is_between(threshold, syncpt,
					(1UL<<31UL)+threshold);
}

/* check for old WAITs to be removed (avoiding a wrap) */
int nvhost_syncpt_wait_check(struct nvmap_client *nvmap,
			struct nvhost_syncpt *sp, u32 waitchk_mask,
			struct nvhost_waitchk *wait,
			struct nvhost_waitchk *waitend)
{
	u32 idx;
	int err = 0;

	/* get current syncpt values */
	for (idx = 0; idx < NV_HOST1X_SYNCPT_NB_PTS; idx++) {
		if (BIT(idx) & waitchk_mask)
			nvhost_syncpt_update_min(sp, idx);
	}

	BUG_ON(!wait && !waitend);

	/* compare syncpt vs wait threshold */
	while (wait != waitend) {
		u32 syncpt, override;

		BUG_ON(wait->syncpt_id > NV_HOST1X_SYNCPT_NB_PTS);

		syncpt = atomic_read(&sp->min_val[wait->syncpt_id]);
		if (nvhost_syncpt_wrapping_comparison(syncpt, wait->thresh)) {
			/*
			 * NULL an already satisfied WAIT_SYNCPT host method,
			 * by patching its args in the command stream. The
			 * method data is changed to reference a reserved
			 * (never given out or incr) NVSYNCPT_GRAPHICS_HOST
			 * syncpt with a matching threshold value of 0, so
			 * is guaranteed to be popped by the host HW.
			 */
			dev_dbg(&syncpt_to_dev(sp)->pdev->dev,
			    "drop WAIT id %d (%s) thresh 0x%x, syncpt 0x%x\n",
			    wait->syncpt_id,
			    nvhost_syncpt_name(wait->syncpt_id),
			    wait->thresh, syncpt);

			/* patch the wait */
			override = nvhost_class_host_wait_syncpt(
					NVSYNCPT_GRAPHICS_HOST, 0);
			err = nvmap_patch_word(nvmap,
					(struct nvmap_handle *)wait->mem,
					wait->offset, override);
			if (err)
				break;
		}
		wait++;
	}
	return err;
}
