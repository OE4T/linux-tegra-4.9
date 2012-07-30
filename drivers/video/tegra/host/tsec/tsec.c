/*
 * drivers/video/tegra/host/tsec/tsec.c
 *
 * Tegra TSEC Module Support
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/slab.h>         /* for kzalloc */
#include <linux/firmware.h>
#include <linux/module.h>
#include <asm/byteorder.h>      /* for parsing ucode image wrt endianness */
#include <linux/delay.h>	/* for udelay */
#include "dev.h"
#include "tsec.h"
#include "hw_tsec.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "chip_support.h"
#include "nvhost_memmgr.h"

#define TSEC_IDLE_TIMEOUT_DEFAULT	10000	/* 10 milliseconds */
#define TSEC_IDLE_CHECK_PERIOD		10	/* 10 usec */

#define get_tsec(ndev) ((struct tsec *)(ndev)->dev.platform_data)
#define set_tsec(ndev, f) ((ndev)->dev.platform_data = f)

/* caller is responsible for freeing */
static char *tsec_get_fw_name(struct nvhost_device *dev)
{
	char *fw_name;
	u8 maj, min;

	/*note size here is a little over...*/
	fw_name = kzalloc(32, GFP_KERNEL);
	if (!fw_name)
		return NULL;

	decode_tsec_ver(dev->version, &maj, &min);
	if (maj == 1) {
		/* there are no minor versions so far for maj==1 */
		sprintf(fw_name, "nvhost_tsec.fw");
	}
	else
		return NULL;

	dev_info(&dev->dev, "fw name:%s\n", fw_name);

	return fw_name;
}


static int tsec_dma_wait_idle(struct nvhost_device *dev, u32 *timeout)
{
	if (!*timeout)
		*timeout = TSEC_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, TSEC_IDLE_CHECK_PERIOD, *timeout);
		u32 dmatrfcmd = nvhost_device_readl(dev, tsec_dmatrfcmd_r());
		u32 idle_v = tsec_dmatrfcmd_idle_v(dmatrfcmd);

		if (tsec_dmatrfcmd_idle_true_v() == idle_v)
			return 0;

		udelay(TSEC_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout);

	dev_err(&dev->dev, "dma idle timeout");

	return -1;
}

static int tsec_dma_pa_to_internal_256b(struct nvhost_device *dev,
		phys_addr_t pa, u32 internal_offset, bool imem)
{
	u32 cmd = tsec_dmatrfcmd_size_256b_f();
	u32 pa_offset =  tsec_dmatrffboffs_offs_f(pa);
	u32 i_offset = tsec_dmatrfmoffs_offs_f(internal_offset);
	u32 timeout = 0; /* default*/

	if (imem)
		cmd |= tsec_dmatrfcmd_imem_true_f();

	nvhost_device_writel(dev, tsec_dmatrfmoffs_r(), i_offset);
	nvhost_device_writel(dev, tsec_dmatrffboffs_r(), pa_offset);
	nvhost_device_writel(dev, tsec_dmatrfcmd_r(), cmd);

	return tsec_dma_wait_idle(dev, &timeout);

}

static int tsec_wait_idle(struct nvhost_device *dev, u32 *timeout)
{
	if (!*timeout)
		*timeout = TSEC_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, TSEC_IDLE_CHECK_PERIOD, *timeout);
		u32 w = nvhost_device_readl(dev, tsec_idlestate_r());

		if (!w)
			return 0;
		udelay(TSEC_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout);

	return -1;
}

int tsec_boot(struct nvhost_device *dev)
{
	u32 timeout;
	u32 offset;
	int err = 0;
	struct tsec *m = get_tsec(dev);

	nvhost_device_writel(dev, tsec_dmactl_r(), 0);
	nvhost_device_writel(dev, tsec_dmatrfbase_r(),
			(m->pa + m->os.bin_data_offset) >> 8);

	for (offset = 0; offset < m->os.data_size; offset += 256)
		tsec_dma_pa_to_internal_256b(dev,
					   m->os.data_offset + offset,
					   offset, false);

	tsec_dma_pa_to_internal_256b(dev, m->os.code_offset, 0, true);

	/* setup tsec interrupts and enable interface */
	nvhost_device_writel(dev, tsec_irqmset_r(),
			(tsec_irqmset_ext_f(0xff) |
				tsec_irqmset_swgen1_set_f() |
				tsec_irqmset_swgen0_set_f() |
				tsec_irqmset_exterr_set_f() |
				tsec_irqmset_halt_set_f()   |
				tsec_irqmset_wdtmr_set_f()));
	nvhost_device_writel(dev, tsec_irqdest_r(),
			(tsec_irqdest_host_ext_f(0xff) |
				tsec_irqdest_host_swgen1_host_f() |
				tsec_irqdest_host_swgen0_host_f() |
				tsec_irqdest_host_exterr_host_f() |
				tsec_irqdest_host_halt_host_f()));
	nvhost_device_writel(dev, tsec_itfen_r(),
			(tsec_itfen_mthden_enable_f() |
				tsec_itfen_ctxen_enable_f()));

	/* boot tsec */
	nvhost_device_writel(dev, tsec_bootvec_r(), tsec_bootvec_vec_f(0));
	nvhost_device_writel(dev, tsec_cpuctl_r(),
			tsec_cpuctl_startcpu_true_f());

	timeout = 0; /* default */

	err = tsec_wait_idle(dev, &timeout);
	if (err != 0) {
		dev_err(&dev->dev, "boot failed due to timeout");
		return err;
	}

	return 0;
}

static int tsec_setup_ucode_image(struct nvhost_device *dev,
		u32 *ucode_ptr,
		const struct firmware *ucode_fw)
{
	struct tsec *m = get_tsec(dev);
	/* image data is little endian. */
	struct tsec_ucode_v1 ucode;
	int w;

	/* copy the whole thing taking into account endianness */
	for (w = 0; w < ucode_fw->size / sizeof(u32); w++)
		ucode_ptr[w] = le32_to_cpu(((u32 *)ucode_fw->data)[w]);

	ucode.bin_header = (struct tsec_ucode_bin_header_v1 *)ucode_ptr;
	/* endian problems would show up right here */
	if (ucode.bin_header->bin_magic != 0x10de) {
		dev_err(&dev->dev,
			   "failed to get firmware magic");
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

	dev_dbg(&dev->dev,
		"ucode bin header: magic:0x%x ver:%d size:%d",
		ucode.bin_header->bin_magic,
		ucode.bin_header->bin_ver,
		ucode.bin_header->bin_size);
	dev_dbg(&dev->dev,
		"ucode bin header: os bin (header,data) offset size: 0x%x, 0x%x %d",
		ucode.bin_header->os_bin_header_offset,
		ucode.bin_header->os_bin_data_offset,
		ucode.bin_header->os_bin_size);
	ucode.os_header = (struct tsec_ucode_os_header_v1 *)
		(((void *)ucode_ptr) + ucode.bin_header->os_bin_header_offset);

	dev_dbg(&dev->dev,
		"os ucode header: os code (offset,size): 0x%x, 0x%x",
		ucode.os_header->os_code_offset,
		ucode.os_header->os_code_size);
	dev_dbg(&dev->dev,
		"os ucode header: os data (offset,size): 0x%x, 0x%x",
		ucode.os_header->os_data_offset,
		ucode.os_header->os_data_size);
	dev_dbg(&dev->dev,
		"os ucode header: num apps: %d",
		ucode.os_header->num_apps);

	m->os.size = ucode.bin_header->os_bin_size;
	m->os.bin_data_offset = ucode.bin_header->os_bin_data_offset;
	m->os.code_offset = ucode.os_header->os_code_offset;
	m->os.data_offset = ucode.os_header->os_data_offset;
	m->os.data_size   = ucode.os_header->os_data_size;

	return 0;
}

int tsec_read_ucode(struct nvhost_device *dev, const char *fw_name)
{
	struct tsec *m = get_tsec(dev);
	const struct firmware *ucode_fw;
	void *ucode_ptr;
	int err;

	err = request_firmware(&ucode_fw, fw_name, &dev->dev);
	if (err) {
		dev_err(&dev->dev, "failed to get tsec firmware\n");
		return err;
	}

	/* allocate pages for ucode */
	m->mem_r = mem_op().alloc(nvhost_get_host(dev)->memmgr,
				     roundup(ucode_fw->size, PAGE_SIZE),
				     PAGE_SIZE, mem_mgr_flag_uncacheable);
	if (IS_ERR_OR_NULL(m->mem_r)) {
		dev_err(&dev->dev, "nvmap alloc failed");
		err = -ENOMEM;
		goto clean_up;
	}

	ucode_ptr = mem_op().mmap(m->mem_r);
	if (!ucode_ptr) {
		dev_err(&dev->dev, "nvmap mmap failed");
		err = -ENOMEM;
		goto clean_up;
	}

	err = tsec_setup_ucode_image(dev, ucode_ptr, ucode_fw);
	if (err) {
		dev_err(&dev->dev, "failed to parse firmware image\n");
		return err;
	}

	m->valid = true;

	mem_op().munmap(m->mem_r, ucode_ptr);

 clean_up:
	release_firmware(ucode_fw);
	return err;
}

void nvhost_tsec_init(struct nvhost_device *dev)
{
	int err = 0;
	struct tsec *m;
	char *fw_name;

	fw_name = tsec_get_fw_name(dev);
	if (!fw_name) {
		dev_err(&dev->dev, "couldn't determine firmware name");
		return;
	}

	m = kzalloc(sizeof(struct tsec), GFP_KERNEL);
	if (!m) {
		dev_err(&dev->dev, "couldn't alloc ucode");
		kfree(fw_name);
		return;
	}
	set_tsec(dev, m);

	err = tsec_read_ucode(dev, fw_name);
	kfree(fw_name);
	fw_name = 0;

	if (err || !m->valid) {
		kfree(m);
		dev_err(&dev->dev, "ucode not valid");
		return;
	}

	m->pa = mem_op().pin(nvhost_get_host(dev)->memmgr, m->mem_r);
	if (m->pa == -EINVAL || m->pa == -EINTR) {
		dev_err(&dev->dev, "nvmap pin failed for ucode");
		err = -EINVAL;
		kfree(m);
		goto clean_up;
	}


	nvhost_module_busy(dev);
	tsec_boot(dev);
	nvhost_module_idle(dev);
	return;

 clean_up:
	dev_err(&dev->dev, "failed");
	mem_op().unpin(nvhost_get_host(dev)->memmgr, m->mem_r);
}

void nvhost_tsec_deinit(struct nvhost_device *dev)
{
	struct tsec *m = get_tsec(dev);

	/* unpin, free ucode memory */
	if (m->mem_r) {
		mem_op().unpin(nvhost_get_host(dev)->memmgr, m->mem_r);
		mem_op().put(nvhost_get_host(dev)->memmgr, m->mem_r);
		m->mem_r = 0;
	}
}

void nvhost_tsec_finalize_poweron(struct nvhost_device *dev)
{
	tsec_boot(dev);
}

static int tsec_probe(struct nvhost_device *dev,
		struct nvhost_device_id *id_table)
{
	int err;
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);

	drv->init = nvhost_tsec_init;
	drv->deinit = nvhost_tsec_deinit;

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	return nvhost_client_device_init(dev);
}

static int __exit tsec_remove(struct nvhost_device *dev)
{
	/* Add clean-up */
	return 0;
}

#ifdef CONFIG_PM
static int tsec_suspend(struct nvhost_device *dev, pm_message_t state)
{
	return nvhost_client_device_suspend(dev);
}

static int tsec_resume(struct nvhost_device *dev)
{
	dev_info(&dev->dev, "resuming\n");
	return 0;
}
#endif

static struct nvhost_driver tsec_driver = {
	.probe = tsec_probe,
	.remove = __exit_p(tsec_remove),
#ifdef CONFIG_PM
	.suspend = tsec_suspend,
	.resume = tsec_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "tsec",
	}
};

static int __init tsec_init(void)
{
	return nvhost_driver_register(&tsec_driver);
}

static void __exit tsec_exit(void)
{
	nvhost_driver_unregister(&tsec_driver);
}

module_init(tsec_init);
module_exit(tsec_exit);
