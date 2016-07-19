/*
 * PVA driver for T194
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#include <linux/export.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/iommu.h>

#include "nvhost_syncpt_unit_interface.h"
#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t194/t194.h"
#include "nvhost_queue.h"
#include "pva_queue.h"
#include "pva.h"
#include "pva_regs.h"

/* Map PVA-A and PVA-B to respective configuration items in nvhost */
static struct of_device_id tegra_pva_of_match[] = {
	{
		.name = "pva0",
		.compatible = "nvidia,tegra194-pva",
		.data = (struct nvhost_device_data *)&t19_pva0_info },
	{
		.name = "pva1",
		.compatible = "nvidia,tegra194-pva",
		.data = (struct nvhost_device_data *)&t19_pva1_info },
	{ },
};

#define EVP_REG_NUM 8
static u32 pva_get_evp_reg(u32 index)
{
	u32 evp_reg[EVP_REG_NUM] = {
		evp_reset_addr_r(),
		evp_undef_addr_r(),
		evp_swi_addr_r(),
		evp_prefetch_abort_addr_r(),
		evp_data_abort_addr_r(),
		evp_rsvd_addr_r(),
		evp_irq_addr_r(),
		evp_fiq_addr_r()
	};

	return evp_reg[index];
}

#define R5_USER_SEGREG_OFFSET 0x40000000
static int pva_init_fw(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva *pva = pdata->private_data;
	struct pva_fw *fw_info = &pva->fw_info;
	u32 *ucode_ptr = fw_info->mapped;
	int err = 0, w;
	int timeout;

	nvhost_dbg_fn("");

	/* Set the Ucode Header address for R5 */
	/* Program user seg subtracting the offset */
	host1x_writel(pdev, cfg_r5user_lsegreg_r(),
		PVA_LOW32((fw_info->phys - R5_USER_SEGREG_OFFSET)));
	host1x_writel(pdev, cfg_r5user_usegreg_r(),
		PVA_EXTRACT64((fw_info->phys - R5_USER_SEGREG_OFFSET),
					39, 32, u32));

	/* check the type of segments and their offset and address */
	for (w = 0; w < fw_info->hdr->nsegments; w++) {
		struct pva_ucode_seg *useg = (struct pva_ucode_seg *)
			((void *)ucode_ptr + PVA_UCODE_SEG_HDR_LENGTH
				+ (PVA_UCODE_SEG_HDR_LENGTH * w));

		switch (useg->type) {

		case PVA_UCODE_SEG_EVP:
		{
			/* First 32 bytes of the EVP payload are zeros.
			 * so skip first 32 bytes
			 */
			u32 i;
			u32 *evpmem =  (u32 *)((void *)ucode_ptr
						+ useg->offset + 32);
			for (i = 0; i < EVP_REG_NUM; i++)
				host1x_writel(pdev,
					pva_get_evp_reg(i), evpmem[i]);
		}
		break;
		case PVA_UCODE_SEG_R5:
		{
			/* R5 segment - Program PRIV1 segment regs*/
			/* Subracting PRIV1 start for R5PRIV1 address*/
			u64 seg_addr = (fw_info->phys - useg->addr);

			host1x_writel(pdev, cfg_priv_ar1_start_r(),
						useg->addr);
			host1x_writel(pdev, cfg_priv_ar1_end_r(),
				useg->addr + useg->size);
			host1x_writel(pdev, cfg_priv_ar1_lsegreg_r(),
				PVA_LOW32((seg_addr + useg->offset)));
			host1x_writel(pdev, cfg_priv_ar1_usegreg_r(),
				PVA_EXTRACT64((seg_addr + useg->offset),
						39, 32, u32));
		}
		break;

		}
	}

	/* TODO: Add stream ID */

	/* Indicate the OS is waiting for PVA ready Interrupt */
	pva->mailbox_status = PVA_MBOX_STATUS_WFI;
	host1x_writel(pdev, hsp_ss0_set_r(), PVA_BOOT_INT);

	/* Take R5 out of reset */
	host1x_writel(pdev, proc_cpuhalt_r(),
		proc_cpuhalt_ncpuhalt_f(proc_cpuhalt_ncpuhalt_done_v()));

	nvhost_dbg_fn("Waiting for PVA to be READY");

	/* Wait PVA to report itself as ready */
	timeout = wait_event_interruptible_timeout(pva->mailbox_waitqueue,
		pva->mailbox_status == PVA_MBOX_STATUS_DONE,
		msecs_to_jiffies(10000));
	if (timeout <= 0)
		err = -ETIMEDOUT;

	pva->mailbox_status = PVA_MBOX_STATUS_INVALID;

	nvhost_dbg_fn("PVA boot returned: %d", err);

	return err;
}

static int pva_free_fw(struct platform_device *pdev, struct pva *pva)
{
	struct pva_fw *fw_info = &pva->fw_info;

	if (fw_info->mapped)
		dma_free_attrs(&pdev->dev, fw_info->size, fw_info->mapped,
			fw_info->phys, &fw_info->attrs);

	memset(fw_info, 0, sizeof(struct pva_fw));

	return 0;
}

static int pva_read_ucode(struct platform_device *pdev,
		const char *fw_name, struct pva_fw *fw_info)
{
	int err = 0, w;
	u32 *ucode_ptr;
	const struct firmware *ucode_fw;

	nvhost_dbg_fn("");

	init_dma_attrs(&fw_info->attrs);

	ucode_fw = nvhost_client_request_firmware(pdev, fw_name);
	if (!ucode_fw) {
		nvhost_dbg_fn("pva firmware request failed");
		dev_err(&pdev->dev,
			"Failed to load the %s firmware\n", fw_name);
		err = -ENOENT;
		return err;
	}

	fw_info->size = ucode_fw->size;
	dma_set_attr(DMA_ATTR_READ_ONLY, &fw_info->attrs);

	/* Allocate the Memory area to load the firmware */
	fw_info->mapped = dma_alloc_attrs(&pdev->dev, fw_info->size,
		&fw_info->phys, GFP_KERNEL, &fw_info->attrs);

	if (!fw_info->mapped) {
		err = -ENOMEM;
		goto clean_up;
	}

	ucode_ptr = fw_info->mapped;

	/* copy the whole thing taking into account endianness */
	for (w = 0; w < ucode_fw->size/sizeof(u32); w++)
		ucode_ptr[w] = le32_to_cpu(((u32 *)ucode_fw->data)[w]);

	/* set the header location accordingly */
	fw_info->hdr = (struct pva_ucode_hdr *)ucode_ptr;

	/* check for the magic number  and header version*/
	if ((fw_info->hdr->magic != PVA_HDR_MAGIC) &&
		(fw_info->hdr->hdr_version != PVA_HDR_VERSION)) {
		dev_err(&pdev->dev, "Wrong PVA uCode header magic/version\n");
		err = -EINVAL;
	}

clean_up:
	release_firmware(ucode_fw);
	return err;
}

static int pva_load_fw(struct platform_device *pdev)
{
	int err = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva *pva = pdata->private_data;

	err = pva_read_ucode(pdev, pdata->firmware_name, &pva->fw_info);
	if (err < 0)
		goto load_fw_err;

	return err;

load_fw_err:
	pva_free_fw(pdev, pva);
	return err;
}

int pva_finalize_poweron(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva *pva = pdata->private_data;
	int err = 0;

	/* Enable LIC_INTERRUPT line for HSP1 */
	host1x_writel(pva->pdev, sec_lic_intr_enable_r(),
		sec_lic_intr_enable_hsp_f(SEC_LIC_INTR_HSP1));

	enable_irq(pva->irq);

	err = pva_load_fw(pdev);
	if (err < 0)
		goto err_poweron;

	err = pva_init_fw(pdev);
	if (err < 0)
		goto err_poweron;

	return err;

err_poweron:
	disable_irq(pva->irq);
	return err;
}

int pva_prepare_poweroff(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva *pva = pdata->private_data;

	disable_irq(pva->irq);

	pva_free_fw(pdev, pva);

	return 0;
}

static int pva_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *pdata;
	const struct of_device_id *match;
	struct pva *pva;
	int err = 0;

	nvhost_dbg_fn("%s", __func__);

	match = of_match_device(tegra_pva_of_match, dev);
	pdata = (struct nvhost_device_data *)match->data;

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(dev, "no platform data\n");
		err = -ENODATA;
		goto err_get_pdata;
	}

	pva = devm_kzalloc(dev, sizeof(*pva), GFP_KERNEL);
	if (!pva) {
		err = -ENOMEM;
		goto err_alloc_pva;
	}

	/* Initialize PVA private data */
	pva->pdev = pdev;

	/* Initialize nvhost specific data */
	pdata->pdev = pdev;
	mutex_init(&pdata->lock);
	pdata->private_data = pva;
	platform_set_drvdata(pdev, pdata);
	init_waitqueue_head(&pva->mailbox_waitqueue);
	mutex_init(&pva->mailbox_mutex);

	/* Map MMIO range to kernel space */
	err = nvhost_client_device_get_resources(pdev);
	if (err < 0)
		goto err_get_resources;

	/* Get clocks */
	err = nvhost_module_init(pdev);
	if (err < 0)
		goto err_module_init;

#ifdef CONFIG_PM_GENERIC_DOMAINS
	/* Initialize Linux power domain */
	err = nvhost_module_add_domain(&pdata->pd, pdev);
	if (err < 0)
		goto err_add_domain;
#endif

	/*
	 * Add this to nvhost device list, initialize scaling,
	 * setup memory management for the device, create dev nodes
	 */
	err = nvhost_client_device_init(pdev);
	if (err < 0)
		goto err_client_device_init;

	pva->pool = nvhost_queue_init(pdev, &pva_queue_ops,
					MAX_PVA_QUEUE_COUNT);
	if (IS_ERR(pva->pool)) {
		err = PTR_ERR(pva->pool);
		goto err_queue_init;
	}

	err = pva_register_isr(pdev);
	if (err < 0)
		goto err_isr_init;

	err = nvhost_syncpt_unit_interface_init(pdev);
	if (err)
		goto err_mss_init;

	return 0;

err_mss_init:
err_isr_init:
	nvhost_queue_deinit(pva->pool);
err_queue_init:
	nvhost_client_device_release(pdev);
err_client_device_init:
#ifdef CONFIG_PM_GENERIC_DOMAINS
err_add_domain:
#endif
	nvhost_module_deinit(pdev);
err_module_init:
err_get_resources:
	devm_kfree(dev, pva);
err_alloc_pva:
err_get_pdata:

	return err;
}

static int __exit pva_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva *pva = pdata->private_data;

	nvhost_queue_deinit(pva->pool);
	nvhost_client_device_release(pdev);
	free_irq(pva->irq, pdata);

	return 0;
}

static struct platform_driver pva_driver = {
	.probe = pva_probe,
	.remove = __exit_p(pva_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "pva",
#ifdef CONFIG_OF
		.of_match_table = tegra_pva_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

static struct of_device_id tegra_pva_domain_match[] = {
	{
		.compatible = "nvidia,tegra194-cv-pd",
		.data = (struct nvhost_device_data *)&t19_pva0_info
	},
};

/* Register PVA power domains and driver */
static int __init pva_init(void)
{
	int ret;

	ret = nvhost_domain_init(tegra_pva_domain_match);
	if (ret)
		return ret;

	return platform_driver_register(&pva_driver);
}

static void __exit pva_exit(void)
{
	platform_driver_unregister(&pva_driver);
}

module_init(pva_init);
module_exit(pva_exit);
