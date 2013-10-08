/*
 * drivers/video/tegra/host/nvjpg/nvjpg.c
 *
 * Tegra NVJPG Module Support
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <mach/clk.h>
#include <asm/byteorder.h>      /* for parsing ucode image wrt endianness */
#include <linux/delay.h>	/* for udelay */
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <mach/pm_domains.h>

#include "dev.h"
#include "nvjpg.h"
#include "hw_nvjpg.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_scale.h"
#include "chip_support.h"
#include "nvhost_memmgr.h"
#include "t210/t210.h"

#define NVJPG_IDLE_TIMEOUT_DEFAULT	10000	/* 10 milliseconds */
#define NVJPG_IDLE_CHECK_PERIOD		10	/* 10 usec */

#define get_nvjpg(ndev) ((struct nvjpg *)(ndev)->dev.platform_data)
#define set_nvjpg(ndev, f) ((ndev)->dev.platform_data = f)

/* caller is responsible for freeing */
static char *nvjpg_get_fw_name(struct platform_device *dev)
{
	char *fw_name;
	u8 maj, min;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	/* note size here is a little over...*/
	fw_name = kzalloc(32, GFP_KERNEL);
	if (!fw_name)
		return NULL;

	decode_nvjpg_ver(pdata->version, &maj, &min);
	sprintf(fw_name, "nvhost_nvjpg0%d%d.fw", maj, min);
	dev_info(&dev->dev, "fw name:%s\n", fw_name);

	return fw_name;
}

static int nvjpg_dma_wait_idle(struct platform_device *dev, u32 *timeout)
{
	if (!*timeout)
		*timeout = NVJPG_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, NVJPG_IDLE_CHECK_PERIOD, *timeout);
		u32 dmatrfcmd = nvhost_device_readl(dev, nvjpg_dmatrfcmd_r());
		u32 idle_v = nvjpg_dmatrfcmd_idle_v(dmatrfcmd);

		if (nvjpg_dmatrfcmd_idle_true_v() == idle_v)
			return 0;

		udelay(NVJPG_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout);

	dev_err(&dev->dev, "dma idle timeout");

	return -1;
}

static int nvjpg_dma_pa_to_internal_256b(struct platform_device *dev,
		u32 offset, u32 internal_offset, bool imem)
{
	u32 cmd = nvjpg_dmatrfcmd_size_256b_f();
	u32 pa_offset =  nvjpg_dmatrffboffs_offs_f(offset);
	u32 i_offset = nvjpg_dmatrfmoffs_offs_f(internal_offset);
	u32 timeout = 0; /* default*/

	if (imem)
		cmd |= nvjpg_dmatrfcmd_imem_true_f();

	nvhost_device_writel(dev, nvjpg_dmatrfmoffs_r(), i_offset);
	nvhost_device_writel(dev, nvjpg_dmatrffboffs_r(), pa_offset);
	nvhost_device_writel(dev, nvjpg_dmatrfcmd_r(), cmd);

	return nvjpg_dma_wait_idle(dev, &timeout);

}

static int nvjpg_wait_idle(struct platform_device *dev, u32 *timeout)
{
	if (!*timeout)
		*timeout = NVJPG_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, NVJPG_IDLE_CHECK_PERIOD, *timeout);
		u32 w = nvhost_device_readl(dev, nvjpg_idlestate_r());

		if (!w)
			return 0;
		udelay(NVJPG_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout);

	return -1;
}

int nvjpg_boot(struct platform_device *dev)
{
	u32 timeout;
	u32 offset;
	int err = 0;
	struct nvjpg *m = get_nvjpg(dev);

	/* check if firmware is loaded or not */
	if (!m || !m->valid)
		return -ENOMEDIUM;

	nvhost_device_writel(dev, nvjpg_dmactl_r(), 0);
	nvhost_device_writel(dev, nvjpg_dmatrfbase_r(),
		(sg_dma_address(m->pa->sgl) + m->os.bin_data_offset) >> 8);

	for (offset = 0; offset < m->os.data_size; offset += 256)
		nvjpg_dma_pa_to_internal_256b(dev,
					   m->os.data_offset + offset,
					   offset, false);

	nvjpg_dma_pa_to_internal_256b(dev, m->os.code_offset, 0, true);

	/* setup nvjpg interrupts and enable interface */
	nvhost_device_writel(dev, nvjpg_irqmset_r(),
			(nvjpg_irqmset_ext_f(0xff) |
				nvjpg_irqmset_swgen1_set_f() |
				nvjpg_irqmset_swgen0_set_f() |
				nvjpg_irqmset_exterr_set_f() |
				nvjpg_irqmset_halt_set_f()   |
				nvjpg_irqmset_wdtmr_set_f()));
	nvhost_device_writel(dev, nvjpg_irqdest_r(),
			(nvjpg_irqdest_host_ext_f(0xff) |
				nvjpg_irqdest_host_swgen1_host_f() |
				nvjpg_irqdest_host_swgen0_host_f() |
				nvjpg_irqdest_host_exterr_host_f() |
				nvjpg_irqdest_host_halt_host_f()));
	nvhost_device_writel(dev, nvjpg_itfen_r(),
			(nvjpg_itfen_mthden_enable_f() |
				nvjpg_itfen_ctxen_enable_f()));

	/* boot nvjpg */
	nvhost_device_writel(dev, nvjpg_bootvec_r(), nvjpg_bootvec_vec_f(0));
	nvhost_device_writel(dev, nvjpg_cpuctl_r(),
			nvjpg_cpuctl_startcpu_true_f());

	timeout = 0; /* default */

	err = nvjpg_wait_idle(dev, &timeout);
	if (err != 0) {
		dev_err(&dev->dev, "boot failed due to timeout");
		return err;
	}

	return 0;
}

static int nvjpg_setup_ucode_image(struct platform_device *dev,
		u32 *ucode_ptr,
		const struct firmware *ucode_fw)
{
	struct nvjpg *m = get_nvjpg(dev);
	/* image data is little endian. */
	struct nvjpg_ucode_v1 ucode;
	int w;

	/* copy the whole thing taking into account endianness */
	for (w = 0; w < ucode_fw->size / sizeof(u32); w++)
		ucode_ptr[w] = le32_to_cpu(((u32 *)ucode_fw->data)[w]);

	ucode.bin_header = (struct nvjpg_ucode_bin_header_v1 *)ucode_ptr;
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
	ucode.os_header = (struct nvjpg_ucode_os_header_v1 *)
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

int nvjpg_read_ucode(struct platform_device *dev, const char *fw_name)
{
	struct nvjpg *m = get_nvjpg(dev);
	const struct firmware *ucode_fw;
	int err;

	ucode_fw  = nvhost_client_request_firmware(dev, fw_name);
	if (!ucode_fw) {
		dev_err(&dev->dev, "failed to get nvjpg firmware\n");
		err = -ENOENT;
		return err;
	}

	/* allocate pages for ucode */
	m->mem_r = nvhost_memmgr_alloc(nvhost_get_host(dev)->memmgr,
				     roundup(ucode_fw->size, PAGE_SIZE),
				     PAGE_SIZE, mem_mgr_flag_uncacheable, 0);
	if (IS_ERR(m->mem_r)) {
		dev_err(&dev->dev, "nvmap alloc failed");
		err = PTR_ERR(m->mem_r);
		goto clean_up;
	}

	m->pa = nvhost_memmgr_pin(nvhost_get_host(dev)->memmgr, m->mem_r,
			&dev->dev, mem_flag_read_only);
	if (IS_ERR(m->pa)) {
		dev_err(&dev->dev, "nvmap pin failed for ucode");
		err = PTR_ERR(m->pa);
		m->pa = NULL;
		goto clean_up;
	}

	m->mapped = nvhost_memmgr_mmap(m->mem_r);
	if (IS_ERR_OR_NULL(m->mapped)) {
		dev_err(&dev->dev, "nvmap mmap failed");
		err = -ENOMEM;
		goto clean_up;
	}

	err = nvjpg_setup_ucode_image(dev, (u32 *)m->mapped, ucode_fw);
	if (err) {
		dev_err(&dev->dev, "failed to parse firmware image\n");
		return err;
	}

	m->valid = true;

	release_firmware(ucode_fw);

	return 0;

clean_up:
	if (m->mapped) {
		nvhost_memmgr_munmap(m->mem_r, (u32 *)m->mapped);
		m->mapped = NULL;
	}
	if (m->pa) {
		nvhost_memmgr_unpin(nvhost_get_host(dev)->memmgr, m->mem_r,
				&dev->dev, m->pa);
		m->pa = NULL;
	}
	if (m->mem_r) {
		nvhost_memmgr_put(nvhost_get_host(dev)->memmgr, m->mem_r);
		m->mem_r = NULL;
	}
	release_firmware(ucode_fw);
	return err;
}

int nvhost_nvjpg_init(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	int err = 0;
	struct nvjpg *m;
	char *fw_name;

	fw_name = nvjpg_get_fw_name(dev);
	if (!fw_name) {
		dev_err(&dev->dev, "couldn't determine firmware name");
		return -EINVAL;
	}

	m = kzalloc(sizeof(struct nvjpg), GFP_KERNEL);
	if (!m) {
		dev_err(&dev->dev, "couldn't alloc ucode");
		kfree(fw_name);
		return -ENOMEM;
	}
	set_nvjpg(dev, m);

	err = nvjpg_read_ucode(dev, fw_name);
	kfree(fw_name);
	fw_name = 0;

	if (err || !m->valid) {
		dev_err(&dev->dev, "ucode not valid");
		goto clean_up;
	}

	nvhost_module_busy(dev);
	nvjpg_boot(dev);
	nvhost_module_idle(dev);

	if (pdata->scaling_init)
		nvhost_scale_hw_init(dev);

	return 0;

clean_up:
	dev_err(&dev->dev, "failed");
	return err;
}

void nvhost_nvjpg_deinit(struct platform_device *dev)
{
	struct nvjpg *m = get_nvjpg(dev);
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	if (pdata->scaling_init)
		nvhost_scale_hw_deinit(dev);

	if (!m)
		return;

	/* unpin, free ucode memory */
	if (m->mapped) {
		nvhost_memmgr_munmap(m->mem_r, m->mapped);
		m->mapped = NULL;
	}
	if (m->pa) {
		nvhost_memmgr_unpin(nvhost_get_host(dev)->memmgr, m->mem_r,
			&dev->dev, m->pa);
		m->pa = NULL;
	}
	if (m->mem_r) {
		nvhost_memmgr_put(nvhost_get_host(dev)->memmgr, m->mem_r);
		m->mem_r = NULL;
	}
	kfree(m);
	set_nvjpg(dev, NULL);
	m->valid = false;
}

int nvhost_nvjpg_finalize_poweron(struct platform_device *dev)
{
	return nvjpg_boot(dev);
}

static struct of_device_id tegra_nvjpg_of_match[] = {
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra210-nvjpg",
		.data = (struct nvhost_device_data *)&t21_nvjpg_info },
#endif
	{ },
};

static int nvjpg_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_nvjpg_of_match, &dev->dev);
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
	pdata->init = nvhost_nvjpg_init;
	pdata->deinit = nvhost_nvjpg_deinit;
	pdata->finalize_poweron = nvhost_nvjpg_finalize_poweron;

	mutex_init(&pdata->lock);

	platform_set_drvdata(dev, pdata);
	dev->dev.platform_data = NULL;

	/* get the module clocks to sane state */
	nvhost_module_init(dev);

#ifdef CONFIG_PM_GENERIC_DOMAINS
	pdata->pd.name = "nvjpg";

	/* add module power domain and also add its domain
	 * as sub-domain of MC domain */
	err = nvhost_module_add_domain(&pdata->pd, dev);
#endif

	/* enable runtime pm. this is needed now since we need to call
	 * _get_sync/_put during boot-up to ensure MC domain is ON */
#ifdef CONFIG_PM_RUNTIME
	if (pdata->clockgate_delay) {
		pm_runtime_set_autosuspend_delay(&dev->dev,
			pdata->clockgate_delay);
		pm_runtime_use_autosuspend(&dev->dev);
	}
	pm_runtime_enable(&dev->dev);
	pm_runtime_get_sync(&dev->dev);
#else
	nvhost_module_enable_clk(&dev->dev);
#endif

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	err = nvhost_client_device_init(dev);

#ifdef CONFIG_PM_RUNTIME
	if (pdata->clockgate_delay)
		pm_runtime_put_sync_autosuspend(&dev->dev);
	else
		pm_runtime_put(&dev->dev);
	if (err)
		return err;
#endif

	return 0;
}

static int __exit nvjpg_remove(struct platform_device *dev)
{
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_put(&dev->dev);
	pm_runtime_disable(&dev->dev);
#else
	nvhost_module_disable_clk(&dev->dev);
#endif
	return 0;
}

static struct platform_driver nvjpg_driver = {
	.probe = nvjpg_probe,
	.remove = __exit_p(nvjpg_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvjpg",
#ifdef CONFIG_OF
		.of_match_table = tegra_nvjpg_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	}
};

static int __init nvjpg_init(void)
{
	return platform_driver_register(&nvjpg_driver);
}

static void __exit nvjpg_exit(void)
{
	platform_driver_unregister(&nvjpg_driver);
}

module_init(nvjpg_init);
module_exit(nvjpg_exit);
