/*
 * Tegra Graphics Init for T194 Architecture Chips
 *
 * Copyright (c) 2016, NVIDIA Corporation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/tegra-soc.h>
#include <linux/platform/tegra/emc_bwmgr.h>

#include <linux/platform/tegra/tegra18_kfuse.h>

#include "dev.h"
#include "class_ids.h"
#include "class_ids_t194.h"

#include "t194.h"
#include "host1x/host1x.h"
#include "tsec/tsec.h"
#include "flcn/flcn.h"
#include "nvdec/nvdec.h"
#include "hardware_t194.h"

#include "chip_support.h"

#include "streamid_regs.c"

#define HOST_EMC_FLOOR 204000000
#define HOST_NVDEC_EMC_FLOOR 102000000

/*
 * TODO: Move following functions to the corresponding files under
 * kernel-3.18 once kernel-t19x gets merged there. Until that
 * happens we can keep these here to avoid extensive amount of
 * added infra
 */

static inline u32 flcn_thi_sec(void)
{
	return 0x00000038;
}

static inline u32 flcn_thi_sec_ch_lock(void)
{
	return (1 << 8);
}

static int nvhost_tsec_t194_finalize_poweron(struct platform_device *dev)
{
	/* Disable access to non-THI registers through channel */
	host1x_writel(dev, flcn_thi_sec(), flcn_thi_sec_ch_lock());

	return nvhost_tsec_finalize_poweron(dev);
}

static int nvhost_flcn_t194_finalize_poweron(struct platform_device *dev)
{
	/* Disable access to non-THI registers through channel */
	host1x_writel(dev, flcn_thi_sec(), flcn_thi_sec_ch_lock());

	return nvhost_flcn_finalize_poweron(dev);
}

static int nvhost_nvdec_t194_finalize_poweron(struct platform_device *dev)
{
	int ret;

	ret = tegra_kfuse_enable_sensing();
	if (ret)
		return ret;

	/* Disable access to non-THI registers through channel */
	host1x_writel(dev, flcn_thi_sec(), flcn_thi_sec_ch_lock());

	ret = nvhost_nvdec_finalize_poweron(dev);
	if (ret)
		tegra_kfuse_disable_sensing();

	return ret;
}

static int nvhost_nvdec_t194_prepare_poweroff(struct platform_device *dev)
{
	tegra_kfuse_disable_sensing();

	return 0;
}

static struct host1x_device_info host1x04_info = {
	.nb_channels	= T194_NVHOST_NUMCHANNELS,
	.ch_base	= 0,
	.ch_limit	= T194_NVHOST_NUMCHANNELS,
	.nb_mlocks	= NV_HOST1X_NB_MLOCKS,
	.initialize_chip_support = nvhost_init_t194_support,
	.nb_hw_pts	= NV_HOST1X_SYNCPT_NB_PTS,
	.nb_pts		= NV_HOST1X_SYNCPT_NB_PTS,
	.pts_base	= 0,
	.pts_limit	= NV_HOST1X_SYNCPT_NB_PTS,
	.syncpt_policy	= SYNCPT_PER_CHANNEL_INSTANCE,
	.channel_policy	= MAP_CHANNEL_ON_SUBMIT,
	.firmware_area_size = SZ_1M,
	.nb_actmons = true,
};

struct nvhost_device_data t19_host1x_info = {
	.clocks			= {
		{"host1x", UINT_MAX},
		{"actmon", UINT_MAX}
	},
	NVHOST_MODULE_NO_POWERGATE_ID,
	.powergate_delay        = 50,
	.private_data		= &host1x04_info,
	.finalize_poweron = nvhost_host1x_finalize_poweron,
	.prepare_poweroff = nvhost_host1x_prepare_poweroff,
	.isolate_contexts	= true,
};

static struct host1x_device_info host1xb04_info = {
	.nb_channels	= T194_NVHOST_NUMCHANNELS,
	.ch_base	= 0,
	.ch_limit	= T194_NVHOST_NUMCHANNELS,
	.nb_mlocks	= NV_HOST1X_NB_MLOCKS,
	.initialize_chip_support = nvhost_init_t194_support,
	.nb_hw_pts	= NV_HOST1X_SYNCPT_NB_PTS,
	.nb_pts		= NV_HOST1X_SYNCPT_NB_PTS,
	.pts_base	= 0,
	.pts_limit	= NV_HOST1X_SYNCPT_NB_PTS,
	.syncpt_policy	= SYNCPT_PER_CHANNEL_INSTANCE,
	.channel_policy	= MAP_CHANNEL_ON_SUBMIT,
};

struct nvhost_device_data t19_host1xb_info = {
	.clocks			= {
		{"host1x", UINT_MAX},
		{"actmon", UINT_MAX}
	},
	NVHOST_MODULE_NO_POWERGATE_ID,
	.private_data		= &host1xb04_info,
};

struct nvhost_device_data t19_msenc_info = {
	.version		= NVHOST_ENCODE_FLCN_VER(6, 1),
	.devfs_name		= "msenc",
	.class			= NV_VIDEO_ENCODE_NVENC_CLASS_ID,
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_NVENC},
	.powergate_delay        = 500,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.clocks			= {
		{"nvenc", UINT_MAX},
		{"emc", HOST_EMC_FLOOR,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_SHARED_BW}
	},
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_flcn_t194_finalize_poweron,
	.moduleid		= NVHOST_MODULE_MSENC,
	.num_channels		= true,
	.firmware_name		= "nvhost_nvenc061.fw",
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x1844,
	.transcfg_val		= 0x20,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_MSENC,
};

struct nvhost_device_data t19_nvdec_info = {
	.version		= NVHOST_ENCODE_NVDEC_VER(3, 0),
	.devfs_name		= "nvdec",
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_NVDEC},
	.class			= NV_NVDEC_CLASS_ID,
	.powergate_delay        = 500,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.clocks			= {
		{"nvdec", UINT_MAX},
		{"kfuse", 0, 0},
		{"emc", HOST_NVDEC_EMC_FLOOR,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_FLOOR}
	},
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_nvdec_t194_finalize_poweron,
	.prepare_poweroff	= nvhost_nvdec_t194_prepare_poweroff,
	.moduleid		= NVHOST_MODULE_NVDEC,
	.ctrl_ops		= &tegra_nvdec_ctrl_ops,
	.num_channels		= true,
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x2c44,
	.transcfg_val		= 0x20,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_NVDEC,
};

struct nvhost_device_data t19_nvjpg_info = {
	.version		= NVHOST_ENCODE_FLCN_VER(1, 1),
	.devfs_name		= "nvjpg",
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_NVJPG},
	.class			= NV_NVJPG_CLASS_ID,
	.powergate_delay        = 500,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.keepalive		= true,
	.moduleid		= NVHOST_MODULE_TSEC,
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_tsec_t194_finalize_poweron,
	.prepare_poweroff	= nvhost_tsec_prepare_poweroff,
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x1644,
	.transcfg_val		= 0x20,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_TSEC,
};

struct nvhost_device_data t19_tsecb_info = {
	.num_channels		= true,
	.devfs_name		= "tsecb",
	.version		= NVHOST_ENCODE_TSEC_VER(1, 0),
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_TSECB},
	.class			= NV_TSECB_CLASS_ID,
	.clocks			= {
		{"tsecb", UINT_MAX},
		{"emc", HOST_EMC_FLOOR,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_FLOOR}
	},
	NVHOST_MODULE_NO_POWERGATE_ID,
	.powergate_delay        = 500,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.keepalive		= true,
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_tsec_t194_finalize_poweron,
	.prepare_poweroff	= nvhost_tsec_prepare_poweroff,
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x1644,
	.transcfg_val		= 0x20,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_TSECB,
};

struct nvhost_device_data t19_vic_info = {
	.num_channels		= true,
	.devfs_name		= "vic",
	.clocks			= {
		{"vic", UINT_MAX, 0},
		{"emc", UINT_MAX, NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_SHARED_BW},
	},
	.version		= NVHOST_ENCODE_FLCN_VER(4, 0),
	.powergate_delay        = 500,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid		= NVHOST_MODULE_VIC,
	.poweron_reset		= true,
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_VIC},
	.class			= NV_GRAPHICS_VIC_CLASS_ID,
	.finalize_poweron	= nvhost_flcn_t194_finalize_poweron,
	.init_class_context	= nvhost_vic_init_context,
	.firmware_name		= "vic04_ucode.bin",
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x2044,
	.transcfg_val		= 0x20,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_VIC,
};

#include "host1x/host1x_channel_t186.c"

static void t194_set_nvhost_chanops(struct nvhost_channel *ch)
{
	if (!ch)
		return;

	ch->ops = host1x_channel_ops;

	/* Disable gather filter in simulator */
	if (tegra_platform_is_linsim())
		ch->ops.init_gather_filter = NULL;
}

int nvhost_init_t194_channel_support(struct nvhost_master *host,
				     struct nvhost_chip_support *op)
{
	op->nvhost_dev.set_nvhost_chanops = t194_set_nvhost_chanops;

	return 0;
}

static void t194_remove_support(struct nvhost_chip_support *op)
{
	kfree(op->priv);
	op->priv = NULL;
}

static void t194_init_regs(struct platform_device *pdev, bool prod)
{
	struct nvhost_streamid_mapping *map_regs = t19x_host1x_streamid_mapping;

	/* simulator cannot handle following writes - skip them */
	if (tegra_platform_is_linsim())
		return;

	while (map_regs->host1x_offset) {
		host1x_hypervisor_writel(pdev,
					 map_regs->host1x_offset,
					 map_regs->client_offset);
		host1x_hypervisor_writel(pdev,
					 map_regs->host1x_offset + sizeof(u32),
					 map_regs->client_limit);
		map_regs++;
	}
}

#include "host1x/host1x_cdma_t186.c"
#include "host1x/host1x_syncpt.c"
#include "host1x/host1x_syncpt_prot_t186.c"
#include "host1x/host1x_intr_t186.c"
#include "host1x/host1x_debug_t186.c"
#include "host1x/host1x_vm_t186.c"
#include "host1x/host1x_actmon_t186.c"

int nvhost_init_t194_support(struct nvhost_master *host,
			     struct nvhost_chip_support *op)
{
	int err;

	op->soc_name = "tegra19x";

	/* create a symlink for host1x if it is not under platform bus or
	 * it has been created with different name */

	if ((host->dev->dev.parent != &platform_bus) ||
	    !strcmp(dev_name(&host->dev->dev), "host1x")) {
		err = sysfs_create_link_nowarn(&platform_bus.kobj,
					&host->dev->dev.kobj,
					"host1x");
		if (err) {
			err = sysfs_create_link(&platform_bus.kobj,
						&host->dev->dev.kobj,
						dev_name(&host->dev->dev));
			if (err)
				dev_warn(&host->dev->dev, "could not create sysfs links\n");
		}
	}

	/* don't worry about cleaning up on failure... "remove" does it. */
	err = nvhost_init_t194_channel_support(host, op);
	if (err)
		return err;

	op->cdma = host1x_cdma_ops;
	op->push_buffer = host1x_pushbuffer_ops;
	op->debug = host1x_debug_ops;

	host->sync_aperture = host->aperture;
	op->syncpt = host1x_syncpt_ops;
	op->intr = host1x_intr_ops;
	op->vm = host1x_vm_ops;
	op->actmon = host1x_actmon_ops;
	op->nvhost_dev.load_gating_regs = t194_init_regs;

	/* WAR to bugs 200094901 and 200082771: enable protection
	 * only on silicon/emulation */

	if (!tegra_platform_is_linsim()) {
		op->syncpt.reset = t186_syncpt_reset;
		op->syncpt.mark_used = t186_syncpt_mark_used;
		op->syncpt.mark_unused = t186_syncpt_mark_unused;
	}
	op->syncpt.mutex_owner = t186_syncpt_mutex_owner;

	op->remove_support = t194_remove_support;

	return 0;
}
