/*
 * drivers/video/tegra/host/gr3d/gr3d.c
 *
 * Tegra Graphics Host 3D
 *
 * Copyright (c) 2012-2013 NVIDIA Corporation.  All rights reserved.
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
#include <linux/dma-buf.h>
#include <linux/syscalls.h>

#include <mach/pm_domains.h>
#include <mach/gpufuse.h>

#include "t114/t114.h"
#include "t148/t148.h"
#include "host1x/host1x01_hardware.h"
#include "nvhost_hwctx.h"
#include "nvhost_acm.h"
#include "dev.h"
#include "gr3d.h"
#include "gr3d_t114.h"
#include "scale3d.h"
#include "bus_client.h"
#include "nvhost_channel.h"
#include "nvhost_memmgr.h"
#include "chip_support.h"
#include "pod_scaling.h"
#include "class_ids.h"
#include "nvhost_job.h"

int nvhost_gr3d_read_reg(
	struct platform_device *dev,
	struct nvhost_channel *channel,
	struct nvhost_hwctx *hwctx,
	u32 offset,
	u32 *value)
{
	struct host1x_hwctx_handler *h = to_host1x_hwctx_handler(hwctx->h);
	u32 syncpt_incrs = 1;
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	void *ref;
	void *read_waiter = NULL;
	struct nvhost_job *job;
	int err;
	struct mem_handle *mem = NULL;
	u32 *mem_ptr = NULL;
	u32 *cmdbuf_ptr = NULL;
	struct sg_table *mem_sgt = NULL;
	struct mem_mgr *memmgr = hwctx->memmgr;
	ulong user_id;
	u32 opcodes[] = {
		/* Switch to 3D - set up output to memory */
		nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0),
		nvhost_opcode_imm(AR3D_GLOBAL_MEMORY_OUTPUT_READS, 1),
		nvhost_opcode_nonincr(AR3D_DW_MEMORY_OUTPUT_ADDRESS, 1),
		0xdeadbeef,
		/* Get host1x to request a register read */
		nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				host1x_uclass_indoff_r(), 1),
		nvhost_class_host_indoff_reg_read(
				host1x_uclass_indoff_indmodid_gr3d_v(),
				offset, false),
		nvhost_opcode_imm(host1x_uclass_inddata_r(), 0),
		/* send reg reads back to host */
		nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0),
		nvhost_opcode_imm(AR3D_GLOBAL_MEMORY_OUTPUT_READS, 0),
		/* Finalize with syncpt increment */
		nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				host1x_uclass_incr_syncpt_base_r(), 1),
		nvhost_class_host_incr_syncpt_base(h->h.waitbase,
				1),
		nvhost_opcode_imm_incr_syncpt(
				host1x_uclass_incr_syncpt_cond_immediate_v(),
				h->h.syncpt),
	};

	/* 12 slots for gather, and one slot for storing the result value */
	mem = nvhost_memmgr_alloc(memmgr, sizeof(opcodes)+4,
			32, mem_mgr_flag_uncacheable, 0);
	if (IS_ERR(mem))
		return PTR_ERR(mem);

	mem_ptr = nvhost_memmgr_mmap(mem);
	if (!mem_ptr) {
		err = -ENOMEM;
		goto done;
	}
	cmdbuf_ptr = mem_ptr + 1;

	mem_sgt = nvhost_memmgr_pin(memmgr, mem, &channel->dev->dev,
							mem_flag_none);
	if (IS_ERR(mem_sgt)) {
		err = -ENOMEM;
		mem_sgt = NULL;
		goto done;
	}
	/* Set address of target memory slot to the stream */
	opcodes[3] = nvhost_memmgr_dma_addr(mem_sgt);

	read_waiter = nvhost_intr_alloc_waiter();
	if (!read_waiter) {
		err = -ENOMEM;
		goto done;
	}

	job = nvhost_job_alloc(channel, hwctx, 1, 0, 0, 1, memmgr);
	if (!job) {
		err = -ENOMEM;
		goto done;
	}

	job->hwctx_syncpt_idx = 0;
	job->sp->id = h->h.syncpt;
	job->sp->waitbase = h->h.waitbase;
	job->sp->incrs = syncpt_incrs;
	job->num_syncpts = 1;
	job->serialize = 1;
	memcpy(cmdbuf_ptr, opcodes, sizeof(opcodes));

#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	get_dma_buf((struct dma_buf *)mem);
	user_id = dma_buf_fd((struct dma_buf *)mem, O_CLOEXEC);
#else
	user_id = nvhost_memmgr_handle_to_id(mem);
#endif

	/* Submit job */
	nvhost_job_add_gather(job, user_id,
			ARRAY_SIZE(opcodes), 4, 0);

	err = nvhost_job_pin(job, &nvhost_get_host(dev)->syncpt);
	if (err)
		goto done;

#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	sys_close(user_id);
#endif
	err = nvhost_channel_submit(job);
	if (err)
		goto done;

	/* Wait for read to be ready */
	err = nvhost_intr_add_action(&nvhost_get_host(dev)->intr,
			h->h.syncpt, job->sp->fence,
			NVHOST_INTR_ACTION_WAKEUP, &wq,
			read_waiter,
			&ref);
	read_waiter = NULL;
	WARN(err, "Failed to set wakeup interrupt");
	wait_event(wq,
		nvhost_syncpt_is_expired(&nvhost_get_host(dev)->syncpt,
				h->h.syncpt, job->sp->fence));
	nvhost_job_put(job);
	job = NULL;
	nvhost_intr_put_ref(&nvhost_get_host(dev)->intr, h->h.syncpt,
			ref);

	*value = *mem_ptr;

done:
	kfree(read_waiter);
	if (mem_ptr)
		nvhost_memmgr_munmap(mem, mem_ptr);
	if (mem_sgt)
		nvhost_memmgr_unpin(memmgr, mem, &channel->dev->dev, mem_sgt);
	if (mem)
		nvhost_memmgr_put(memmgr, mem);
	return err;
}
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
		struct nvhost_channel *ch, bool mem_flag)
{
	struct host1x_hwctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	if (mem_flag)
		ctx->cpuva = dma_alloc_writecombine(&ch->dev->dev,
						p->restore_size * 4,
						&ctx->iova,
						GFP_KERNEL);
	else
		ctx->cpuva = dma_alloc_coherent(&ch->dev->dev,
						p->restore_size * 4,
						&ctx->iova,
						GFP_KERNEL);

	if (!ctx->cpuva) {
		dev_err(&ch->dev->dev, "memory allocation failed\n");
		goto fail;
	}

	kref_init(&ctx->hwctx.ref);
	ctx->hwctx.h = &p->h;
	ctx->hwctx.channel = ch;
	ctx->hwctx.valid = false;
	ctx->hwctx.save_incrs = p->save_incrs;
	ctx->hwctx.save_thresh = p->h.save_thresh;
	ctx->hwctx.save_slots = p->save_slots;

	ctx->restore_size = p->restore_size;
	ctx->hwctx.restore_incrs = p->restore_incrs;
	ctx->mem_flag = mem_flag;
	return ctx;

fail:
	kfree(ctx);
	return NULL;
}

void nvhost_3dctx_restore_push(struct nvhost_hwctx *nctx,
		struct nvhost_cdma *cdma)
{
	struct host1x_hwctx *ctx = to_host1x_hwctx(nctx);
	_nvhost_cdma_push_gather(cdma,
		ctx->cpuva,
		ctx->iova,
		0,
		nvhost_opcode_gather(ctx->restore_size),
		ctx->iova);
}

void nvhost_3dctx_get(struct nvhost_hwctx *ctx)
{
	kref_get(&ctx->ref);
}

void nvhost_3dctx_free(struct kref *ref)
{
	struct nvhost_hwctx *nctx = container_of(ref, struct nvhost_hwctx, ref);
	struct host1x_hwctx *ctx = to_host1x_hwctx(nctx);

	if (ctx->cpuva) {
		if (ctx->mem_flag)
			dma_free_writecombine(&nctx->channel->dev->dev,
					ctx->restore_size * 4,
					ctx->cpuva,
					ctx->iova);
		else
			dma_free_coherent(&nctx->channel->dev->dev,
					ctx->restore_size * 4,
					ctx->cpuva,
					ctx->iova);

		ctx->cpuva = NULL;
		ctx->iova = 0;
	}

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
#ifdef TEGRA_11X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra114-gr3d",
		.data = (struct nvhost_device_data *)&t11_gr3d_info },
#endif
#ifdef TEGRA_14X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra148-gr3d",
		.data = (struct nvhost_device_data *)&t14_gr3d_info },
#endif
	{ },
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

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	nvhost_module_init(dev);

#ifdef CONFIG_PM_GENERIC_DOMAINS
	pdata->pd.name = "gr3d";

	err = nvhost_module_add_domain(&pdata->pd, dev);
#endif

	err = nvhost_client_device_init(dev);

	return err;
}

static int __exit gr3d_remove(struct platform_device *dev)
{
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_put(&dev->dev);
	pm_runtime_disable(&dev->dev);
#else
	nvhost_module_disable_clk(&dev->dev);
#endif
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
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
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
