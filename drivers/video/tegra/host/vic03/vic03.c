/*
 * drivers/video/tegra/host/vic/vic03.c
 *
 * Tegra VIC03 Module Support
 *
 * Copyright (c) 2011-2013, NVIDIA Corporation.
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

#include <linux/slab.h>         /* for kzalloc */
#include <asm/byteorder.h>      /* for parsing ucode image wrt endianness */
#include <linux/delay.h>	/* for udelay */
#include <linux/export.h>
#include <linux/scatterlist.h>
#include <linux/nvmap.h>

#include "dev.h"
#include "class_ids.h"
#include "bus_client.h"
#include "nvhost_as.h"

#include "host1x/host1x_hwctx.h"

#include "vic03.h"
#include "hw_flcn_vic03.h"
#include "hw_tfbif_vic03.h"

#include "t124/hardware_t124.h" /* for nvhost opcodes*/


#include "../../../../../arch/arm/mach-tegra/iomap.h"

static struct resource vic03_resources[] = {
{
	.name = "base",
	.start = TEGRA_VIC_BASE,
	.end = TEGRA_VIC_BASE + TEGRA_VIC_SIZE - 1,
	.flags = IORESOURCE_MEM,
},
};

struct nvhost_device_data vic03_info = {
	/*.syncpts       = BIT(NVSYNCPT_VIC),*/
	/*.modulemutexes = BIT(NVMODMUTEX_VIC),*/
	.clocks = {{"vic03", UINT_MAX}, {"emc", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VIC,
	.alloc_hwctx_handler = nvhost_vic03_alloc_hwctx_handler,

};

struct platform_device tegra_vic03_device = {
	.name	       = "vic03",
	.num_resources = 1,
	.resource      = vic03_resources,
	.dev           = {
		.platform_data = &vic03_info,
	},
};
static inline struct vic03 *get_vic03(struct platform_device *dev)
{
	return (struct vic03 *)nvhost_get_private_data(dev);
}
static inline void set_vic03(struct platform_device *dev, struct vic03 *vic03)
{
	nvhost_set_private_data(dev, vic03);
}

#define VIC_IDLE_TIMEOUT_DEFAULT	10000	/* 10 milliseconds */
#define VIC_IDLE_CHECK_PERIOD	10		/* 10 usec */
static int vic03_flcn_wait_idle(struct platform_device *dev,
				u32 *timeout)
{
	struct vic03 *v = get_vic03(dev);

	nvhost_dbg_fn("");

	if (!*timeout)
		*timeout = VIC_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, VIC_IDLE_CHECK_PERIOD, *timeout);
		u32 w = vic03_readl(v, flcn_idlestate_r());

		if (!w) {
			nvhost_dbg_fn("done");
			return 0;
		}
		udelay(VIC_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout);

	dev_err(&dev->dev, "vic03 flcn idle timeout");

	return -1;
}

static int vic03_flcn_dma_wait_idle(struct platform_device *dev, u32 *timeout)
{
	struct vic03 *v = get_vic03(dev);
	nvhost_dbg_fn("");

	if (!*timeout)
		*timeout = VIC_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, VIC_IDLE_CHECK_PERIOD, *timeout);
		u32 dmatrfcmd = vic03_readl(v, flcn_dmatrfcmd_r());
		u32 idle_v = flcn_dmatrfcmd_idle_v(dmatrfcmd);

		if (flcn_dmatrfcmd_idle_true_v() == idle_v) {
			nvhost_dbg_fn("done");
			return 0;
		}
		udelay(VIC_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout);

	dev_err(&dev->dev, "vic03 dma idle timeout");

	return -1;
}


static int vic03_flcn_dma_pa_to_internal_256b(struct platform_device *dev,
					      phys_addr_t pa,
					      u32 internal_offset,
					      bool imem)
{
	struct vic03 *v = get_vic03(dev);

	u32 cmd = flcn_dmatrfcmd_size_256b_f();
	u32 pa_offset =  flcn_dmatrffboffs_offs_f(pa);
	u32 i_offset = flcn_dmatrfmoffs_offs_f(internal_offset);
	u32 timeout = 0; /* default*/

	if (imem)
		cmd |= flcn_dmatrfcmd_imem_true_f();

	vic03_writel(v, flcn_dmatrfmoffs_r(), i_offset);
	vic03_writel(v, flcn_dmatrffboffs_r(), pa_offset);
	vic03_writel(v, flcn_dmatrfcmd_r(), cmd);

	return vic03_flcn_dma_wait_idle(dev, &timeout);

}

static int vic03_setup_ucode_image(struct platform_device *dev,
				   u32 *ucode_ptr,
				   const struct firmware *ucode_fw)
{
	struct vic03 *v = get_vic03(dev);
	/* image data is little endian. */
	struct ucode_v1_vic03 ucode;
	int w;

	/* copy the whole thing taking into account endianness */
	for (w = 0; w < ucode_fw->size/sizeof(u32); w++)
		ucode_ptr[w] = le32_to_cpu(((u32 *)ucode_fw->data)[w]);

	ucode.bin_header = (struct ucode_bin_header_v1_vic03 *)ucode_ptr;
	/* endian problems would show up right here */
	if (ucode.bin_header->bin_magic != 0x10de) {
		dev_err(&dev->dev,
			   "failed to get vic03 firmware magic");
		return -EINVAL;
	}
	if (ucode.bin_header->bin_ver != 1) {
		dev_err(&dev->dev,
			   "unsupported firmware version");
		return -ENOENT;
	}
	/* shouldn't be bigger than what firmware thinks */
	if (ucode.bin_header->bin_size > ucode_fw->size) {
		dev_err(&dev->dev,
			   "ucode image size inconsistency");
		return -EINVAL;
	}

	nvhost_dbg_info("vic03 ucode bin header: magic:0x%x ver:%d size:%d",
			ucode.bin_header->bin_magic,
			ucode.bin_header->bin_ver,
			ucode.bin_header->bin_size);
	nvhost_dbg_info("vic03 ucode bin header: os bin (header,data) offset size: 0x%x, 0x%x %d",
			ucode.bin_header->os_bin_header_offset,
			ucode.bin_header->os_bin_data_offset,
			ucode.bin_header->os_bin_size);
	nvhost_dbg_info("vic03 ucode bin header: fce bin (header,data) offset size: 0x%x, 0x%x %d",
			ucode.bin_header->fce_bin_header_offset,
			ucode.bin_header->fce_bin_data_offset,
			ucode.bin_header->fce_bin_size);

	ucode.os_header = (struct ucode_os_header_v1_vic03 *)
		(((void *)ucode_ptr) + ucode.bin_header->os_bin_header_offset);

	nvhost_dbg_info("vic03 os ucode header: os code (offset,size): 0x%x, 0x%x",
			ucode.os_header->os_code_offset,
			ucode.os_header->os_code_size);
	nvhost_dbg_info("vic03 os ucode header: os data (offset,size): 0x%x, 0x%x",
			ucode.os_header->os_data_offset,
			ucode.os_header->os_data_size);
	nvhost_dbg_info("vic03 os ucode header: num apps: %d", ucode.os_header->num_apps);

	ucode.fce_header = (struct ucode_fce_header_v1_vic03 *)
		(((void *)ucode_ptr) + ucode.bin_header->fce_bin_header_offset);

	nvhost_dbg_info("vic03 fce ucode header: offset, buffer_size, size: 0x%x 0x%x 0x%x",
			ucode.fce_header->fce_ucode_offset,
			ucode.fce_header->fce_ucode_buffer_size,
			ucode.fce_header->fce_ucode_size);

	v->ucode.os.size = ucode.bin_header->os_bin_size;
	v->ucode.os.bin_data_offset = ucode.bin_header->os_bin_data_offset;
	v->ucode.os.code_offset = ucode.os_header->os_code_offset;
	v->ucode.os.data_offset = ucode.os_header->os_data_offset;
	v->ucode.os.data_size   = ucode.os_header->os_data_size;

	v->ucode.fce.size        = ucode.fce_header->fce_ucode_size;
	v->ucode.fce.data_offset = ucode.bin_header->fce_bin_data_offset;

	return 0;
}

static int vic03_read_ucode(struct platform_device *dev)
{
	struct vic03 *v = get_vic03(dev);
	struct mem_mgr *nvmap_c = v->host->memmgr;
	const struct firmware *ucode_fw;
	void *ucode_ptr;
	int err;

	ucode_fw = nvhost_client_request_firmware(dev, VIC03_UCODE_FW_NAME);
	if (IS_ERR_OR_NULL(ucode_fw)) {
		nvhost_dbg_fn("request firmware failed");
		dev_err(&dev->dev, "failed to get vic03 firmware\n");
		err = -ENOENT;
		return err;
	}

	/* allocate pages for ucode */
	v->ucode.mem_r = mem_op().alloc(nvmap_c,
				     roundup(ucode_fw->size, PAGE_SIZE),
				     PAGE_SIZE, mem_mgr_flag_uncacheable,
				     0);
	if (IS_ERR_OR_NULL(v->ucode.mem_r)) {
		nvhost_dbg_fn("nvmap alloc failed");
		err = -ENOMEM;
		goto clean_up;
	}


	ucode_ptr = mem_op().mmap(v->ucode.mem_r);
	if (!ucode_ptr) {
		nvhost_dbg_fn("nvmap mmap failed");
		err = -ENOMEM;
		goto clean_up;
	}

	err = vic03_setup_ucode_image(dev, ucode_ptr, ucode_fw);
	if (err) {
		dev_err(&dev->dev, "failed to parse firmware image\n");
		return err;
	}

	v->ucode.valid = true;

	mem_op().munmap(v->ucode.mem_r, ucode_ptr);

 clean_up:
	release_firmware(ucode_fw);
	return err;
}

static int vic03_boot(struct platform_device *dev)
{
	struct vic03 *v = get_vic03(dev);
	u32 fifoctrl, timeout;
	u32 offset;
	int err = 0;

	vic03_writel(v, flcn_dmactl_r(), 0);

	/* FIXME : disable clock gating, remove when clock gating problem is resolved from MCCIF*/
	fifoctrl = vic03_readl(v, tfbif_mccif_fifoctrl_r());

	fifoctrl |= tfbif_mccif_fifoctrl_rclk_override_enable_f() |
		tfbif_mccif_fifoctrl_wclk_override_enable_f();
	vic03_writel(v, tfbif_mccif_fifoctrl_r(), fifoctrl);

	vic03_writel(v, flcn_dmatrfbase_r(), (v->ucode.pa + v->ucode.os.bin_data_offset) >> 8);

	for (offset = 0; offset < v->ucode.os.data_size; offset += 256)
		vic03_flcn_dma_pa_to_internal_256b(dev,
					   v->ucode.os.data_offset + offset,
					   offset, false);

	vic03_flcn_dma_pa_to_internal_256b(dev, v->ucode.os.code_offset,
					   0, true);

	/* setup falcon interrupts and enable interface */
	vic03_writel(v, flcn_irqmset_r(), (flcn_irqmset_ext_f(0xff)    |
					   flcn_irqmset_swgen1_set_f() |
					   flcn_irqmset_swgen0_set_f() |
					   flcn_irqmset_exterr_set_f() |
					   flcn_irqmset_halt_set_f()   |
					   flcn_irqmset_wdtmr_set_f()));
	vic03_writel(v, flcn_irqdest_r(), (flcn_irqdest_host_ext_f(0xff) |
					   flcn_irqdest_host_swgen1_host_f() |
					   flcn_irqdest_host_swgen0_host_f() |
					   flcn_irqdest_host_exterr_host_f() |
					   flcn_irqdest_host_halt_host_f()));
	vic03_writel(v, flcn_itfen_r(), (flcn_itfen_mthden_enable_f() |
					flcn_itfen_ctxen_enable_f()));

	/* boot falcon */
	vic03_writel(v, flcn_bootvec_r(), flcn_bootvec_vec_f(0));
	vic03_writel(v, flcn_cpuctl_r(), flcn_cpuctl_startcpu_true_f());

	timeout = 0; /* default */

	err = vic03_flcn_wait_idle(dev, &timeout);
	if (err != 0) {
		dev_err(&dev->dev, "boot failed due to timeout");
		return err;
	}

	return 0;
}

void nvhost_vic03_init(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);
	struct vic03 *v = get_vic03(dev);

	nvhost_dbg_fn("in dev:%p v:%p", dev, v);

	if (!v) {
		nvhost_dbg_fn("allocating vic03 support");
		v = kzalloc(sizeof(*v), GFP_KERNEL);
		if (!v) {
			dev_err(&dev->dev, "couldn't alloc vic03 support");
			err = -ENOMEM;
			goto clean_up;
		}
		set_vic03(dev, v);
	}
	nvhost_dbg_fn("primed dev:%p v:%p", dev, v);

	v->host = nvhost_get_host(dev);
	v->regs = pdata->aperture[0];

	if (!v->ucode.valid)
		err = vic03_read_ucode(dev);

	if (err || !v->ucode.valid) {
		nvhost_err(&dev->dev, "ucode image is not valid");
		return;
	}

	v->ucode.sgt = mem_op().pin(v->host->memmgr, v->ucode.mem_r);
	if (IS_ERR_OR_NULL(v->ucode.sgt)){
		nvhost_err(&dev->dev, "nvmap pin failed for ucode");
		goto clean_up;
	}
	v->ucode.pa = sg_dma_address(v->ucode.sgt->sgl);

	if (!pdata->can_powergate) {
		nvhost_module_busy(dev);
		err = vic03_boot(dev);
		nvhost_module_idle(dev);
	}

	if (err)
		goto clean_up;

	return /*0*/;

 clean_up:
	nvhost_err(&dev->dev, "failed");
	mem_op().unpin(nvhost_get_host(dev)->memmgr, v->ucode.mem_r,
		       v->ucode.sgt);
	return /*err*/;


}

void nvhost_vic03_deinit(struct platform_device *dev)
{

	struct vic03 *v = get_vic03(dev);
	/* unpin, free ucode memory */
	if (v->ucode.mem_r) {
		mem_op().unpin(v->host->memmgr, v->ucode.mem_r, v->ucode.sgt);
		mem_op().put(v->host->memmgr, v->ucode.mem_r);
		v->ucode.mem_r = 0;
	}
	/* free mappings to registers, etc*/
	if (v->regs) {
		iounmap(v->regs);
		v->regs = 0;
	}

	/* zap, free */
	set_vic03(dev,0);
	kfree(v);
}

static struct nvhost_hwctx *vic03_alloc_hwctx(struct nvhost_hwctx_handler *h,
		struct nvhost_channel *ch)
{
	struct host1x_hwctx_handler *p = to_host1x_hwctx_handler(h);

	struct vic03 *v = get_vic03(ch->dev);
	struct mem_mgr *nvmap = nvhost_get_host(ch->dev)->memmgr;
	struct host1x_hwctx *ctx;
	bool map_restore = true;
	u32 *ptr;
	u32 syncpt = nvhost_get_devdata(ch->dev)->syncpts[0];
	u32 nvhost_vic03_restore_size = 11; /* number of words written below */

	nvhost_dbg_fn("");

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	ctx->restore = mem_op().alloc(nvmap,
				   nvhost_vic03_restore_size * 4, 32,
				   map_restore ? mem_mgr_flag_write_combine
				      : mem_mgr_flag_uncacheable,
				   0);
	if (IS_ERR_OR_NULL(ctx->restore))
		goto fail_alloc;

	if (map_restore) {
		ctx->restore_virt = mem_op().mmap(ctx->restore);
		if (IS_ERR_OR_NULL(ctx->restore_virt))
			goto fail_mmap;
	} else
		ctx->restore_virt = NULL;

	ptr = ctx->restore_virt;

	/* set class to vic */
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_VIC_CLASS_ID, 0, 0);

	/* set app id, fce ucode size, offset */
	ptr[1] = nvhost_opcode_incr(VIC_UCLASS_METHOD_OFFSET, 2);
	ptr[2] = NVA0B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID  >> 2;
	ptr[3] = 1;

	ptr[4] = nvhost_opcode_incr(VIC_UCLASS_METHOD_OFFSET, 2);
	ptr[5] = NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_SIZE >> 2;
	ptr[6] = v->ucode.fce.size;

	ptr[7] = nvhost_opcode_incr(VIC_UCLASS_METHOD_OFFSET, 2);
	ptr[8] = NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_OFFSET >> 2;
	ptr[9] = (v->ucode.pa + v->ucode.fce.data_offset) >> 8;

	/* syncpt increment to track restore gather. */
	ptr[10] = nvhost_opcode_imm_incr_syncpt(
			host1x_uclass_incr_syncpt_cond_op_done_v(),
			syncpt);

	kref_init(&ctx->hwctx.ref);
	ctx->hwctx.h = &p->h;
	ctx->hwctx.channel = ch;
	ctx->hwctx.valid = true; /* this is a preconditioning sequence... */
	ctx->hwctx.save_incrs = 0;
	ctx->hwctx.save_thresh = 0;
	ctx->hwctx.save_slots = 0;

	ctx->restore_sgt = mem_op().pin(nvmap, ctx->restore);
	if (IS_ERR_VALUE(ctx->restore_phys))
		goto fail_pin;
	ctx->restore_phys = sg_dma_address(ctx->restore_sgt->sgl);

	ctx->restore_size = nvhost_vic03_restore_size;
	ctx->hwctx.restore_incrs = 1;

	return &ctx->hwctx;

 fail_pin:
	if (map_restore)
		mem_op().munmap(ctx->restore, ctx->restore_virt);
 fail_mmap:
	mem_op().put(nvmap, ctx->restore);
 fail_alloc:
	kfree(ctx);
	return NULL;
}

static void vic03_free_hwctx(struct kref *ref)
{
	struct nvhost_hwctx *nctx = container_of(ref, struct nvhost_hwctx, ref);
	struct host1x_hwctx *ctx = to_host1x_hwctx(nctx);
	struct mem_mgr *nvmap =
		nvhost_get_host(nctx->channel->dev)->memmgr;

	if (ctx->restore_virt) {
		mem_op().munmap(ctx->restore, ctx->restore_virt);
		ctx->restore_virt = NULL;
	}
	mem_op().unpin(nvmap, ctx->restore, ctx->restore_sgt);
	ctx->restore_phys = 0;
	mem_op().put(nvmap, ctx->restore);
	ctx->restore = NULL;
	kfree(ctx);
}

static void vic03_get_hwctx (struct nvhost_hwctx *ctx)
{
	nvhost_dbg_fn("");
	kref_get(&ctx->ref);
}
static void vic03_put_hwctx (struct nvhost_hwctx *ctx)
{
	nvhost_dbg_fn("");
	kref_put(&ctx->ref, vic03_free_hwctx);
}
static void vic03_save_push_hwctx ( struct nvhost_hwctx *ctx, struct nvhost_cdma *cdma)
{
	nvhost_dbg_fn("");
}


struct nvhost_hwctx_handler * nvhost_vic03_alloc_hwctx_handler(
      u32 syncpt, u32 waitbase,
      struct nvhost_channel *ch)
{

	struct host1x_hwctx_handler *p;

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	p->h.syncpt = syncpt;
	p->h.waitbase = waitbase;

	p->h.alloc = vic03_alloc_hwctx;
	p->h.get   = vic03_get_hwctx;
	p->h.put   = vic03_put_hwctx;
	p->h.save_push = vic03_save_push_hwctx;
	p->h.save_service = NULL;

	return &p->h;
}


void nvhost_vic03_finalize_poweron(struct platform_device *dev)
{
	vic03_boot(dev);
}

static int __devinit vic03_probe(struct platform_device *dev)
{
	int err;
	struct nvhost_device_data *pdata =
		(struct nvhost_device_data *)dev->dev.platform_data;

	nvhost_dbg_fn("dev:%p pdata:%p", dev, pdata);

	pdata->init                = nvhost_vic03_init;
	pdata->deinit              = nvhost_vic03_deinit;
	pdata->alloc_hwctx_handler = nvhost_vic03_alloc_hwctx_handler;
	pdata->finalize_poweron    = nvhost_vic03_finalize_poweron;

	pdata->pdev = dev;

	platform_set_drvdata(dev, pdata);
	/*nvhost_set_private_data(dev, vic03);*/

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	err = nvhost_client_device_init(dev);
	if (err) {
		nvhost_dbg_fn("failed to init client device for %s",
			      dev->name);
		return err;
	}
	err = nvhost_as_init_device(dev);
	if (err) {
		nvhost_dbg_fn("failed to init client address space"
			      " device for %s", dev->name);
		return err;
	}
	return 0;
}

static int __exit vic03_remove(struct platform_device *dev)
{
	/* Add clean-up */
	return 0;
}

#ifdef CONFIG_PM
static int vic03_suspend(struct platform_device *dev, pm_message_t state)
{
	return nvhost_client_device_suspend(dev);
}

static int vic03_resume(struct platform_device *dev)
{
	return 0;
}
#endif

static struct platform_driver vic03_driver = {
	.probe = vic03_probe,
	.remove = __exit_p(vic03_remove),

#ifdef CONFIG_PM
	.suspend = vic03_suspend,
	.resume = vic03_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "vic03",
	}
};

static int __init vic03_init(void)
{
	return platform_driver_register(&vic03_driver);
}

static void __exit vic03_exit(void)
{
	platform_driver_unregister(&vic03_driver);
}

module_init(vic03_init);
module_exit(vic03_exit);
