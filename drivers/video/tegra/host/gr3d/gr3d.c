/*
 * drivers/video/tegra/host/gr3d/gr3d.c
 *
 * Tegra Graphics Host 3D
 *
 * Copyright (c) 2012-2013 NVIDIA Corporation.
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
#include <linux/scatterlist.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/pm.h>

#include <mach/pm_domains.h>
#include <mach/gpufuse.h>

#include "t20/t20.h"
#include "t30/t30.h"
#include "t114/t114.h"
#include "host1x/host1x01_hardware.h"
#include "nvhost_hwctx.h"
#include "nvhost_acm.h"
#include "dev.h"
#include "gr3d.h"
#include "gr3d_t20.h"
#include "gr3d_t30.h"
#include "gr3d_t114.h"
#include "scale3d_actmon.h"
#include "scale3d.h"
#include "bus_client.h"
#include "nvhost_channel.h"
#include "nvhost_memmgr.h"
#include "chip_support.h"
#include "pod_scaling.h"
#include "class_ids.h"

void nvhost_3dctx_restore_begin(struct host1x_hwctx_handler *p, u32 *ptr)
{
	/* set class to host */
	ptr[0] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					host1x_uclass_incr_syncpt_base_r(), 1);
	/* increment sync point base */
	ptr[1] = nvhost_class_host_incr_syncpt_base(p->h.waitbase,
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
		host1x_uclass_incr_syncpt_cond_op_done_v(), p->h.syncpt);
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
	ctx->restore = nvhost_memmgr_alloc(memmgr, p->restore_size * 4, 32,
		map_restore ? mem_mgr_flag_write_combine
			    : mem_mgr_flag_uncacheable);
	if (IS_ERR(ctx->restore))
		goto fail_alloc;

	if (map_restore) {
		ctx->restore_virt = nvhost_memmgr_mmap(ctx->restore);
		if (!ctx->restore_virt)
			goto fail_mmap;
	} else
		ctx->restore_virt = NULL;

	ctx->restore_sgt = nvhost_memmgr_pin(memmgr, ctx->restore);
	if (IS_ERR(ctx->restore_sgt))
		goto fail_pin;
	ctx->restore_phys = sg_dma_address(ctx->restore_sgt->sgl);

	kref_init(&ctx->hwctx.ref);
	ctx->hwctx.h = &p->h;
	ctx->hwctx.channel = ch;
	ctx->hwctx.valid = false;
	ctx->hwctx.save_incrs = p->save_incrs;
	ctx->hwctx.save_thresh = p->h.save_thresh;
	ctx->hwctx.save_slots = p->save_slots;

	ctx->restore_size = p->restore_size;
	ctx->hwctx.restore_incrs = p->restore_incrs;
	return ctx;

fail_pin:
	if (map_restore)
		nvhost_memmgr_munmap(ctx->restore, ctx->restore_virt);
fail_mmap:
	nvhost_memmgr_put(memmgr, ctx->restore);
fail_alloc:
	kfree(ctx);
	return NULL;
}

void nvhost_3dctx_restore_push(struct nvhost_hwctx *nctx,
		struct nvhost_cdma *cdma)
{
	struct host1x_hwctx *ctx = to_host1x_hwctx(nctx);
	nvhost_cdma_push_gather(cdma,
		ctx->hwctx.memmgr,
		ctx->restore,
		0,
		nvhost_opcode_gather(ctx->restore_size),
		ctx->restore_phys);
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

	if (ctx->restore_virt)
		nvhost_memmgr_munmap(ctx->restore, ctx->restore_virt);

	nvhost_memmgr_unpin(memmgr, ctx->restore, ctx->restore_sgt);
	nvhost_memmgr_put(memmgr, ctx->restore);
	kfree(ctx);
}

void nvhost_3dctx_put(struct nvhost_hwctx *ctx)
{
	kref_put(&ctx->ref, nvhost_3dctx_free);
}

int nvhost_gr3d_prepare_power_off(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	return nvhost_channel_save_context(pdata->channel);
}

static struct of_device_id tegra_gr3d_of_match[] = {
	{ .compatible = "nvidia,tegra20-gr3d",
		.data = (struct nvhost_device_data *)&t20_gr3d_info },
	{ .compatible = "nvidia,tegra30-gr3d",
		.data = (struct nvhost_device_data *)&t30_gr3d_info },
	{ .compatible = "nvidia,tegra114-gr3d",
		.data = (struct nvhost_device_data *)&t11_gr3d_info },
	{ },
};

struct gr3d_pm_domain {
	struct platform_device *dev;
	struct generic_pm_domain pd;
};

static int gr3d_unpowergate(struct generic_pm_domain *domain)
{
	struct gr3d_pm_domain *gr3d_pd;

	gr3d_pd = container_of(domain, struct gr3d_pm_domain, pd);
	return nvhost_module_power_on(gr3d_pd->dev);
}

static int gr3d_powergate(struct generic_pm_domain *domain)
{
	struct gr3d_pm_domain *gr3d_pd;

	gr3d_pd = container_of(domain, struct gr3d_pm_domain, pd);
	return nvhost_module_power_off(gr3d_pd->dev);
}

static int gr3d_enable_clock(struct device *dev)
{
	return nvhost_module_enable_clk(to_platform_device(dev));
}

static int gr3d_disable_clock(struct device *dev)
{
	return nvhost_module_disable_clk(to_platform_device(dev));
}

static int gr3d_save_context(struct device *dev)
{
	struct platform_device *pdev;
	struct nvhost_device_data *pdata;

	pdev = to_platform_device(dev);
	if (!pdev)
		return -EINVAL;

	pdata = platform_get_drvdata(pdev);
	if (!pdata)
		return -EINVAL;

	if (pdata->prepare_poweroff)
		pdata->prepare_poweroff(pdev);

	return 0;
}

static int gr3d_restore_context(struct device *dev)
{
	struct platform_device *pdev;
	struct nvhost_device_data *pdata;

	pdev = to_platform_device(dev);
	if (!pdev)
		return -EINVAL;

	pdata = platform_get_drvdata(pdev);
	if (!pdata)
		return -EINVAL;

	if (pdata->finalize_poweron)
		pdata->finalize_poweron(pdev);

	return 0;
}

static int gr3d_suspend(struct device *dev)
{
	return nvhost_client_device_suspend(to_platform_device(dev));
}

static int gr3d_resume(struct device *dev)
{
	dev_info(dev, "resuming\n");
	return 0;
}

static struct gr3d_pm_domain gr3d_pd = {
	.pd = {
		.name = "gr3d",
		.power_off = gr3d_powergate,
		.power_on = gr3d_unpowergate,
		.dev_ops = {
			.start = gr3d_enable_clock,
			.stop = gr3d_disable_clock,
		},
	},
};

static int gr3d_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_gr3d_of_match, &dev->dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);
	nvhost_module_init(dev);

	gr3d_pd.dev = dev;
	err = nvhost_module_add_domain(&gr3d_pd.pd, dev);

	/* overwrite save/restore fptrs set by pm_genpd_init */
	gr3d_pd.pd.dev_ops.save_state = gr3d_save_context;
	gr3d_pd.pd.dev_ops.restore_state = gr3d_restore_context;
	gr3d_pd.pd.domain.ops.suspend = gr3d_suspend;
	gr3d_pd.pd.domain.ops.resume = gr3d_resume;

	pm_runtime_set_autosuspend_delay(&dev->dev, pdata->clockgate_delay);
	pm_runtime_use_autosuspend(&dev->dev);
	pm_runtime_enable(&dev->dev);

	pm_runtime_get_sync(&dev->dev);
	err = nvhost_client_device_init(dev);
	pm_runtime_put(&dev->dev);
	if (err)
		return err;

	return err;
}

static int __exit gr3d_remove(struct platform_device *dev)
{
	/* Add clean-up */
	return 0;
}

static struct platform_driver gr3d_driver = {
	.probe = gr3d_probe,
	.remove = __exit_p(gr3d_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "gr3d",
#ifdef CONFIG_OF
		.of_match_table = tegra_gr3d_of_match,
#endif
	},
};

static int __init gr3d_init(void)
{
	return platform_driver_register(&gr3d_driver);
}

static void __exit gr3d_exit(void)
{
	platform_driver_unregister(&gr3d_driver);
}

module_init(gr3d_init);
module_exit(gr3d_exit);
