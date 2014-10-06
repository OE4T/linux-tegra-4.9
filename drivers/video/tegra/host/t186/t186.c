/*
 * Tegra Graphics Init for T186 Architecture Chips
 *
 * Copyright (c) 2014, NVIDIA Corporation.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/io.h>

#include "dev.h"
#include "class_ids.h"

#include "t186.h"
#include "host1x/host1x.h"
#include "flcn/flcn.h"
#include "nvdec/nvdec.h"
#include "hardware_t186.h"

#include "chip_support.h"

static struct host1x_device_info host1x04_info = {
	.nb_channels	= T186_NVHOST_NUMCHANNELS,
	.nb_pts		= NV_HOST1X_SYNCPT_NB_PTS,
	.nb_mlocks	= NV_HOST1X_NB_MLOCKS,
	.initialize_chip_support = nvhost_init_t186_support,
	.pts_base	= 0,
	.pts_limit	= NV_HOST1X_SYNCPT_NB_PTS,
};

struct nvhost_device_data t18_host1x_info = {
	.clocks		= {{"host1x", UINT_MAX}, {"actmon", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	.private_data	= &host1x04_info,
};

struct nvhost_device_data t18_msenc_info = {
	.version		= NVHOST_ENCODE_FLCN_VER(5, 0),
	.class			= NV_VIDEO_ENCODE_NVENC_CLASS_ID,
#ifdef TEGRA_POWERGATE_NVENC
	.powergate_ids		= { TEGRA_POWERGATE_NVENC, -1 },
#else
	NVHOST_MODULE_NO_POWERGATE_IDS,
#endif
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.clocks			= {{"msenc", UINT_MAX, 0},
				   {"emc", UINT_MAX} },
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_flcn_finalize_poweron,
	.moduleid		= NVHOST_MODULE_MSENC,
	.num_channels		= 1,
	.firmware_name		= "nvhost_nvenc050.fw",
};

struct nvhost_device_data t18_nvdec_info = {
	.version		= NVHOST_ENCODE_NVDEC_VER(2, 0),
	.class			= NV_NVDEC_CLASS_ID,
#ifdef TEGRA_POWERGATE_NVDEC
	.powergate_ids		= { TEGRA_POWERGATE_NVDEC, -1 },
#else
	NVHOST_MODULE_NO_POWERGATE_IDS,
#endif
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.clocks			= {{"nvdec", UINT_MAX, 0},
				   {"emc", UINT_MAX} },
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_nvdec_finalize_poweron,
	.moduleid		= NVHOST_MODULE_NVDEC,
	.ctrl_ops		= &tegra_nvdec_ctrl_ops,
	.num_channels		= 1,
	.actmon_enabled		= true,
};

struct nvhost_device_data t18_nvjpg_info = {
	.version		= NVHOST_ENCODE_FLCN_VER(1, 0),
	.class			= NV_NVJPG_CLASS_ID,
#ifdef TEGRA_POWERGATE_NVJPG
	.powergate_ids		= { TEGRA_POWERGATE_NVJPG, -1 },
#else
	NVHOST_MODULE_NO_POWERGATE_IDS,
#endif
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.powergate_delay	= 500,
	.can_powergate		= true,
	.clocks			= { {"nvjpg", UINT_MAX, 0},
				    {"emc", UINT_MAX} },
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_flcn_finalize_poweron,
	.moduleid		= NVHOST_MODULE_NVJPG,
	.num_channels		= 1,
	.firmware_name		= "nvhost_nvjpg010.fw",
};

struct nvhost_device_data t18_vic_info = {
	.num_channels		= 1,
	.modulemutexes		= {NVMODMUTEX_VIC},
	.clocks			= {{"vic03", UINT_MAX, 0},
				   {"emc", UINT_MAX,
				   NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER},
				   {"vic_floor", 0,
				   NVHOST_MODULE_ID_CBUS_FLOOR},
				   {"emc_shared", 0,
				   NVHOST_MODULE_ID_EMC_SHARED}, {} },
	.version		= NVHOST_ENCODE_FLCN_VER(4, 0),
#ifdef TEGRA_POWERGATE_VIC
	.powergate_ids	= { TEGRA_POWERGATE_VIC, -1 },
#else
	NVHOST_MODULE_NO_POWERGATE_IDS,
#endif
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid		= NVHOST_MODULE_VIC,
	.poweron_reset		= true,
	.class			= NV_GRAPHICS_VIC_CLASS_ID,
	.alloc_hwctx_handler	= nvhost_alloc_hwctx_handler,
	.prepare_poweroff	= nvhost_vic_prepare_poweroff,
	.finalize_poweron	= nvhost_vic_finalize_poweron,
	.firmware_name		= "vic04_ucode.bin",
};

#include "host1x/host1x_channel.c"

static void t186_set_nvhost_chanops(struct nvhost_channel *ch)
{
	if (ch)
		ch->ops = host1x_channel_ops;
}

int nvhost_init_t186_channel_support(struct nvhost_master *host,
				     struct nvhost_chip_support *op)
{
	op->nvhost_dev.set_nvhost_chanops = t186_set_nvhost_chanops;

	return 0;
}

static void t186_remove_support(struct nvhost_chip_support *op)
{
	kfree(op->priv);
	op->priv = 0;
}

#include "host1x/host1x_cdma_t186.c"
#include "host1x/host1x_syncpt.c"
#include "host1x/host1x_intr_t186.c"
#include "host1x/host1x_debug_t186.c"

int nvhost_init_t186_support(struct nvhost_master *host,
			     struct nvhost_chip_support *op)
{
	int err;

	op->soc_name = "tegra18x";

	/* create a symlink for host1x if it is not under platform bus or
	 * it has been created with different name */

	if ((host->dev->dev.parent != &platform_bus) ||
	    !strcmp(dev_name(&host->dev->dev), "host1x")) {
		err = sysfs_create_link(&platform_bus.kobj,
					&host->dev->dev.kobj,
					"host1x");
		if (err)
			dev_warn(&host->dev->dev, "could not create sysfs links\n");
	}

	/* don't worry about cleaning up on failure... "remove" does it. */
	err = nvhost_init_t186_channel_support(host, op);
	if (err)
		return err;

	op->cdma = host1x_cdma_ops;
	op->push_buffer = host1x_pushbuffer_ops;
	op->debug = host1x_debug_ops;

	host->sync_aperture = host->aperture;
	op->syncpt = host1x_syncpt_ops;
	op->intr = host1x_intr_ops;

	op->remove_support = t186_remove_support;

	return 0;
}
