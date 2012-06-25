/*
 * drivers/video/tegra/host/gr3d/gr3d.c
 *
 * Tegra Graphics Host 3D
 *
 * Copyright (c) 2012 NVIDIA Corporation.
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
#include <linux/export.h>
#include <linux/module.h>

#include "t20/t20.h"
#include "host1x/host1x_channel.h"
#include "host1x/host1x_hardware.h"
#include "host1x/host1x_syncpt.h"
#include "nvhost_hwctx.h"
#include "dev.h"
#include "gr3d.h"
#include "gr3d_t20.h"
#include "gr3d_t30.h"
#include "gr3d_t114.h"
#include "scale3d.h"
#include "bus_client.h"
#include "nvhost_channel.h"
#include "nvhost_memmgr.h"
#include "chip_support.h"

#include <mach/hardware.h>

#ifndef TEGRA_POWERGATE_3D1
#define TEGRA_POWERGATE_3D1	-1
#endif

void nvhost_3dctx_restore_begin(struct host1x_hwctx_handler *p, u32 *ptr)
{
	/* set class to host */
	ptr[0] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_INCR_SYNCPT_BASE, 1);
	/* increment sync point base */
	ptr[1] = nvhost_class_host_incr_syncpt_base(p->waitbase,
			p->restore_incrs);
	/* set class to 3D */
	ptr[2] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
	/* program PSEQ_QUAD_ID */
	ptr[3] = nvhost_opcode_imm(AR3D_PSEQ_QUAD_ID, 0);
}

void nvhost_3dctx_restore_direct(u32 *ptr, u32 start_reg, u32 count)
{
	ptr[0] = nvhost_opcode_incr(start_reg, count);
}

void nvhost_3dctx_restore_indirect(u32 *ptr, u32 offset_reg, u32 offset,
			u32 data_reg, u32 count)
{
	ptr[0] = nvhost_opcode_imm(offset_reg, offset);
	ptr[1] = nvhost_opcode_nonincr(data_reg, count);
}

void nvhost_3dctx_restore_end(struct host1x_hwctx_handler *p, u32 *ptr)
{
	/* syncpt increment to track restore gather. */
	ptr[0] = nvhost_opcode_imm_incr_syncpt(
			NV_SYNCPT_OP_DONE, p->syncpt);
}

/*** ctx3d ***/
struct host1x_hwctx *nvhost_3dctx_alloc_common(struct host1x_hwctx_handler *p,
		struct nvhost_channel *ch, bool map_restore)
{
	struct mem_mgr *memmgr = nvhost_get_host(ch->dev)->memmgr;
	struct host1x_hwctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;
	ctx->restore = mem_op().alloc(memmgr, p->restore_size * 4, 32,
		map_restore ? mem_mgr_flag_write_combine
			    : mem_mgr_flag_uncacheable);
	if (IS_ERR_OR_NULL(ctx->restore))
		goto fail;

	if (map_restore) {
		ctx->restore_virt = mem_op().mmap(ctx->restore);
		if (!ctx->restore_virt)
			goto fail;
	} else
		ctx->restore_virt = NULL;

	kref_init(&ctx->hwctx.ref);
	ctx->hwctx.h = &p->h;
	ctx->hwctx.channel = ch;
	ctx->hwctx.valid = false;
	ctx->save_incrs = p->save_incrs;
	ctx->save_thresh = p->save_thresh;
	ctx->save_slots = p->save_slots;
	ctx->restore_phys = mem_op().pin(memmgr, ctx->restore);
	if (IS_ERR_VALUE(ctx->restore_phys))
		goto fail;

	ctx->restore_size = p->restore_size;
	ctx->restore_incrs = p->restore_incrs;
	return ctx;

fail:
	if (map_restore && ctx->restore_virt) {
		mem_op().munmap(ctx->restore, ctx->restore_virt);
		ctx->restore_virt = NULL;
	}
	mem_op().put(memmgr, ctx->restore);
	ctx->restore = NULL;
	kfree(ctx);
	return NULL;
}

void nvhost_3dctx_get(struct nvhost_hwctx *ctx)
{
	kref_get(&ctx->ref);
}

void nvhost_3dctx_free(struct kref *ref)
{
	struct nvhost_hwctx *nctx = container_of(ref, struct nvhost_hwctx, ref);
	struct host1x_hwctx *ctx = to_host1x_hwctx(nctx);
	struct mem_mgr *memmgr = nvhost_get_host(nctx->channel->dev)->memmgr;

	if (ctx->restore_virt) {
		mem_op().munmap(ctx->restore, ctx->restore_virt);
		ctx->restore_virt = NULL;
	}
	mem_op().unpin(memmgr, ctx->restore);
	ctx->restore_phys = 0;
	mem_op().put(memmgr, ctx->restore);
	ctx->restore = NULL;
	kfree(ctx);
}

void nvhost_3dctx_put(struct nvhost_hwctx *ctx)
{
	kref_put(&ctx->ref, nvhost_3dctx_free);
}

int nvhost_gr3d_prepare_power_off(struct nvhost_device *dev)
{
	return host1x_save_context(dev, NVSYNCPT_3D);
}

enum gr3d_ip_ver {
	gr3d_01,
	gr3d_02,
	gr3d_03,
};

struct gr3d_desc {
	void (*finalize_poweron)(struct nvhost_device *dev);
	void (*busy)(struct nvhost_device *);
	void (*idle)(struct nvhost_device *);
	void (*suspend_ndev)(struct nvhost_device *);
	void (*init)(struct nvhost_device *dev);
	void (*deinit)(struct nvhost_device *dev);
	int (*prepare_poweroff)(struct nvhost_device *dev);
	struct nvhost_hwctx_handler *(*alloc_hwctx_handler)(u32 syncpt,
			u32 waitbase, struct nvhost_channel *ch);
};

static const struct gr3d_desc gr3d[] = {
	[gr3d_01] = {
		.finalize_poweron = NULL,
		.busy = NULL,
		.idle = NULL,
		.suspend_ndev = NULL,
		.init = NULL,
		.deinit = NULL,
		.prepare_poweroff = nvhost_gr3d_prepare_power_off,
		.alloc_hwctx_handler = nvhost_gr3d_t20_ctxhandler_init,
	},
	[gr3d_02] = {
		.finalize_poweron = NULL,
		.busy = nvhost_scale3d_notify_busy,
		.idle = nvhost_scale3d_notify_idle,
		.suspend_ndev = nvhost_scale3d_suspend,
		.init = nvhost_scale3d_init,
		.deinit = nvhost_scale3d_deinit,
		.prepare_poweroff = nvhost_gr3d_prepare_power_off,
		.alloc_hwctx_handler = nvhost_gr3d_t30_ctxhandler_init,
	},
	[gr3d_03] = {
		.finalize_poweron = NULL,
		.busy = nvhost_scale3d_notify_busy,
		.idle = nvhost_scale3d_notify_idle,
		.suspend_ndev = nvhost_scale3d_suspend,
		.init = nvhost_scale3d_init,
		.deinit = nvhost_scale3d_deinit,
		.prepare_poweroff = nvhost_gr3d_prepare_power_off,
		.alloc_hwctx_handler = nvhost_gr3d_t114_ctxhandler_init,
	},
};

static struct nvhost_device_id gr3d_id[] = {
	{ "gr3d01", gr3d_01 },
	{ "gr3d02", gr3d_02 },
	{ "gr3d03", gr3d_03 },
	{ },
};

MODULE_DEVICE_TABLE(nvhost, gr3d_id);

static int gr3d_probe(struct nvhost_device *dev,
	struct nvhost_device_id *id_table)
{
	int index = 0;
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);

	index = id_table->driver_data;

	drv->finalize_poweron		= gr3d[index].finalize_poweron;
	drv->busy			= gr3d[index].busy;
	drv->idle			= gr3d[index].idle;
	drv->suspend_ndev		= gr3d[index].suspend_ndev;
	drv->init			= gr3d[index].init;
	drv->deinit			= gr3d[index].deinit;
	drv->prepare_poweroff		= gr3d[index].prepare_poweroff;
	drv->alloc_hwctx_handler	= gr3d[index].alloc_hwctx_handler;

	/* reset device name so that consistent device name can be
	 * found in clock tree */
	dev->name = "gr3d";

	return nvhost_client_device_init(dev);
}

static int __exit gr3d_remove(struct nvhost_device *dev)
{
	/* Add clean-up */
	return 0;
}

static int gr3d_suspend(struct nvhost_device *dev, pm_message_t state)
{
	return nvhost_client_device_suspend(dev);
}

static int gr3d_resume(struct nvhost_device *dev)
{
	dev_info(&dev->dev, "resuming\n");
	return 0;
}

struct nvhost_device *gr3d_device;

static struct nvhost_driver gr3d_driver = {
	.probe = gr3d_probe,
	.remove = __exit_p(gr3d_remove),
#ifdef CONFIG_PM
	.suspend = gr3d_suspend,
	.resume = gr3d_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "gr3d",
	},
	.id_table = gr3d_id,
};

static int __init gr3d_init(void)
{
	int err;

	gr3d_device = nvhost_get_device("gr3d");
	if (!gr3d_device)
		return -ENXIO;

	err = nvhost_device_register(gr3d_device);
	if (err) {
		pr_err("Could not register 3D device\n");
		return err;
	}

	return nvhost_driver_register(&gr3d_driver);
}

static void __exit gr3d_exit(void)
{
	nvhost_driver_unregister(&gr3d_driver);
}

module_init(gr3d_init);
module_exit(gr3d_exit);
