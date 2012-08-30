/*
 * drivers/video/tegra/host/t124/syncpt_t124.c
 *
 * Tegra Graphics Host Syncpoints for T124
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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

#include <linux/io.h>
#include "dev.h"
#include "nvhost_syncpt.h"
#include "nvhost_acm.h"

#include "t124.h"
#include "hardware_t124.h"
#include "syncpt_t124.h"

#include "chip_support.h"

#include "gk20a/gk20a.h"

/**
 * Write the current syncpoint value back to hw.
 */
static void t124_syncpt_reset(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);
	int min = nvhost_syncpt_read_min(sp, id);
	nvhost_dbg_fn("");
	writel(min, dev->sync_aperture + (host1x_sync_syncpt_0_0_r() + id * 4));
}

/**
 * Write the current waitbase value back to hw.
 */
static void t124_syncpt_reset_wait_base(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);

	nvhost_dbg_fn("");
	writel(sp->base_val[id],
		dev->sync_aperture + (host1x_sync_syncpt_base_0_0_r() +
				      id * 4));
}

/**
 * Read waitbase value from hw.
 */
static void t124_syncpt_read_wait_base(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);

	nvhost_dbg_fn("");
	sp->base_val[id] = readl(dev->sync_aperture +
				 (host1x_sync_syncpt_base_0_0_r() +
				  id * 4));
}

/**
 * Updates the last value read from hardware.
 * (was nvhost_syncpt_update_min)
 */
static u32 t124_syncpt_update_min(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);
	void __iomem *sync_regs = dev->sync_aperture;
	u32 old, live;

	nvhost_dbg_fn("");
	do {
		old = nvhost_syncpt_read_min(sp, id);
		live = readl(sync_regs + (host1x_sync_syncpt_0_0_r() + id * 4));
	} while ((u32)atomic_cmpxchg(&sp->min_val[id], old, live) != old);

	if (!nvhost_syncpt_check_max(sp, id, live))
		dev_err(&syncpt_to_dev(sp)->dev->dev,
				"%s failed: id=%u\n",
				__func__,
				id);

	return live;
}

/**
 * Write a cpu syncpoint increment to the hardware, without touching
 * the cache. Caller is responsible for host being powered.
 */
static void t124_syncpt_cpu_incr(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_master *dev = syncpt_to_dev(sp);
	u32 reg_offset = id / 32;

	nvhost_dbg_fn("");
	BUG_ON(!nvhost_module_powered(dev->dev));
	if (!nvhost_syncpt_client_managed(sp, id) && nvhost_syncpt_min_eq_max(sp, id)) {
		dev_err(&syncpt_to_dev(sp)->dev->dev,
			"Trying to increment syncpoint id %d beyond max\n",
			id);
		nvhost_debug_dump(syncpt_to_dev(sp));
		return;
	}

	writel(BIT(id & (32 - 1)), dev->sync_aperture +
	       host1x_sync_syncpt_cpu_incr_0_0_r() + reg_offset * 4);
	wmb();
}

/* remove a wait pointed to by patch_addr */
static int t124_syncpt_patch_wait(struct nvhost_syncpt *sp,
        void *patch_addr)
{
    u32 override = nvhost_class_host_wait_syncpt(
            NVSYNCPT_GRAPHICS_HOST, 0);
    __raw_writel(override, patch_addr);
    return 0;
}

static const char *s_syncpt_names[NV_HOST1X_SYNCPT_NB_PTS] = {
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
	"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""
};

static const char *t124_syncpt_name(struct nvhost_syncpt *sp, u32 id)
{
	struct host1x_device_info *info = &syncpt_to_dev(sp)->info;

	BUG_ON(id > ARRAY_SIZE(s_syncpt_names));
	return (id >= info->nb_pts) ? NULL : info->syncpt_names[id];
}

static void t124_syncpt_debug(struct nvhost_syncpt *sp)
{
	nvhost_dbg_fn("");
}

static int t124_syncpt_mutex_try_lock(struct nvhost_syncpt *sp,
		unsigned int idx)
{
	void __iomem *sync_regs = syncpt_to_dev(sp)->sync_aperture;
	/* mlock registers returns 0 when the lock is aquired.
	 * writing 0 clears the lock. */
	return !!readl(sync_regs + (host1x_sync_mlock_0_0_r() + idx * 4));
}

static void t124_syncpt_mutex_unlock(struct nvhost_syncpt *sp,
		unsigned int idx)
{
	void __iomem *sync_regs = syncpt_to_dev(sp)->sync_aperture;

	writel(0, sync_regs + (host1x_sync_mlock_0_0_r() + idx * 4));
}

int nvhost_init_t124_syncpt_support(struct nvhost_master *host,
		struct nvhost_chip_support *op)
{
	host->info.nb_pts	= NV_HOST1X_SYNCPT_NB_PTS;
	host->info.nb_mlocks	= NV_HOST1X_SYNC_MLOCK_NUM;
	host->info.nb_bases	= NV_HOST1X_SYNCPT_NB_BASES;
	host->info.syncpt_names	= s_syncpt_names;
	host->info.client_managed = NVSYNCPTS_CLIENT_MANAGED;

	host->sync_aperture = host->aperture + HOST1X_CHANNEL_SYNC_REG_BASE;

	op->syncpt.reset = t124_syncpt_reset;
	op->syncpt.reset_wait_base = t124_syncpt_reset_wait_base;
	op->syncpt.read_wait_base = t124_syncpt_read_wait_base;
	op->syncpt.update_min = t124_syncpt_update_min;
	op->syncpt.cpu_incr = t124_syncpt_cpu_incr;
	op->syncpt.patch_wait = t124_syncpt_patch_wait;
	op->syncpt.debug = t124_syncpt_debug;
	op->syncpt.name = t124_syncpt_name;
	op->syncpt.mutex_try_lock = t124_syncpt_mutex_try_lock;
	op->syncpt.mutex_unlock = t124_syncpt_mutex_unlock;

	gk20a_device.syncpt_base = NVSYNCPT_GK20A_BASE;

	s_syncpt_names[NVSYNCPT_CSI_VI_0]	= "csi_vi_0";
	s_syncpt_names[NVSYNCPT_CSI_VI_1]	= "csi_vi_1";
	s_syncpt_names[NVSYNCPT_VI_ISP_0]	= "vi_isp_0";
	s_syncpt_names[NVSYNCPT_VI_ISP_1]	= "vi_isp_1";
	s_syncpt_names[NVSYNCPT_VI_ISP_2]	= "vi_isp_2";
	s_syncpt_names[NVSYNCPT_VI_ISP_3]	= "vi_isp_3";
	s_syncpt_names[NVSYNCPT_VI_ISP_4]	= "vi_isp_4";
	s_syncpt_names[NVSYNCPT_3D]		= "3d";
	s_syncpt_names[NVSYNCPT_MPE]		= "mpe";
	s_syncpt_names[NVSYNCPT_MPE_EBM_EOF]	= "mpe_ebm_eof";
	s_syncpt_names[NVSYNCPT_MPE_WR_SAFE]	= "mpe_wr_safe";
	s_syncpt_names[NVSYNCPT_VIC]		= "vic";
	s_syncpt_names[NVSYNCPT_TSEC]		= "tsec";
	s_syncpt_names[NVSYNCPT_DISP0_A]	= "disp0";
	s_syncpt_names[NVSYNCPT_DISP1_A]	= "disp1";
	s_syncpt_names[NVSYNCPT_AVP_0]		= "avp";
	s_syncpt_names[NVSYNCPT_DISP0_B]	= "disp0b";
	s_syncpt_names[NVSYNCPT_DISP1_B]	= "disp1b";
	s_syncpt_names[NVSYNCPT_DISP0_C]	= "disp0c";
	s_syncpt_names[NVSYNCPT_DISP1_C]	= "disp0c";
	s_syncpt_names[NVSYNCPT_VBLANK0]	= "vblank0";
	s_syncpt_names[NVSYNCPT_VBLANK1]	= "vblank1";
	s_syncpt_names[NVSYNCPT_DSI]		= "dsi";

	return 0;
}
