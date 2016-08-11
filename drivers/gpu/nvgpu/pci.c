/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include "pci.h"
#include "gk20a/gk20a.h"
#include "gk20a/platform_gk20a.h"

#define PCI_INTERFACE_NAME "card-%s%%s"

static int nvgpu_pci_tegra_probe(struct device *dev)
{
	return 0;
}

static int nvgpu_pci_tegra_remove(struct device *dev)
{
	return 0;
}

static bool nvgpu_pci_tegra_is_railgated(struct device *pdev)
{
	return false;
}

static struct gk20a_platform nvgpu_pci_device = {
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay		= 500,
	.can_railgate		= false,
	.can_elpg = false,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,

	.default_big_page_size	= SZ_64K,

	.ch_wdt_timeout_ms = 7000,
	.disable_bigpage = true,

	.has_ce = true,
};

static struct pci_device_id nvgpu_pci_table[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, PCI_ANY_ID),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
	},
	{}
};

static irqreturn_t nvgpu_pci_isr(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;
	irqreturn_t ret_stall;
	irqreturn_t ret_nonstall;

	ret_stall = g->ops.mc.isr_stall(g);
	ret_nonstall = g->ops.mc.isr_nonstall(g);

	return (ret_stall == IRQ_NONE && ret_nonstall == IRQ_NONE) ?
		IRQ_NONE : IRQ_WAKE_THREAD;
}

static irqreturn_t nvgpu_pci_intr_thread(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;

	g->ops.mc.isr_thread_stall(g);
	g->ops.mc.isr_thread_nonstall(g);

	return IRQ_HANDLED;
}

static int nvgpu_pci_init_support(struct pci_dev *pdev)
{
	int err = 0;
	struct gk20a *g = get_gk20a(&pdev->dev);

	g->regs = ioremap(pci_resource_start(pdev, 0),
			  pci_resource_len(pdev, 0));
	if (IS_ERR(g->regs)) {
		gk20a_err(dev_from_gk20a(g), "failed to remap gk20a registers");
		err = PTR_ERR(g->regs);
		goto fail;
	}

	g->bar1 = ioremap(pci_resource_start(pdev, 1),
			  pci_resource_len(pdev, 1));
	if (IS_ERR(g->bar1)) {
		gk20a_err(dev_from_gk20a(g), "failed to remap gk20a bar1");
		err = PTR_ERR(g->bar1);
		goto fail;
	}

	g->regs_saved = g->regs;
	g->bar1_saved = g->bar1;

	mutex_init(&g->dbg_sessions_lock);
	mutex_init(&g->client_lock);
	mutex_init(&g->ch_wdt_lock);
	mutex_init(&g->poweroff_lock);

	g->remove_support = gk20a_remove_support;
	return 0;

 fail:
	gk20a_remove_support(&pdev->dev);
	return err;
}

static char *nvgpu_pci_devnode(struct device *dev, umode_t *mode)
{
	*mode = S_IRUGO | S_IWUGO;
	return kasprintf(GFP_KERNEL, "nvgpu-pci/%s", dev_name(dev));
}

static struct class nvgpu_pci_class = {
	.owner = THIS_MODULE,
	.name = "nvidia-pci-gpu",
	.devnode = nvgpu_pci_devnode,
};

#ifdef CONFIG_PM
static int nvgpu_pci_pm_runtime_resume(struct device *dev)
{
	return gk20a_pm_finalize_poweron(dev);
}

static int nvgpu_pci_pm_runtime_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops nvgpu_pci_pm_ops = {
	.runtime_resume = nvgpu_pci_pm_runtime_resume,
	.runtime_suspend = nvgpu_pci_pm_runtime_suspend,
	.resume = nvgpu_pci_pm_runtime_resume,
	.suspend = nvgpu_pci_pm_runtime_suspend,
};
#endif

static int nvgpu_pci_pm_init(struct device *dev)
{
#ifdef CONFIG_PM
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	if (!platform->can_railgate) {
		pm_runtime_disable(dev);
	} else {
		if (platform->railgate_delay)
			pm_runtime_set_autosuspend_delay(dev,
				platform->railgate_delay);

		/*
		 * Runtime PM for PCI devices is disabled by default,
		 * so we need to enable it first
		 */
		pm_runtime_use_autosuspend(dev);
		pm_runtime_put_noidle(dev);
		pm_runtime_allow(dev);
	}
#endif
	return 0;
}

static int nvgpu_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *pent)
{
	struct gk20a_platform *platform = &nvgpu_pci_device;
	struct gk20a *g;
	int err;
	char *nodefmt;

	pci_set_drvdata(pdev, platform);

	g = kzalloc(sizeof(struct gk20a), GFP_KERNEL);
	if (!g) {
		gk20a_err(&pdev->dev, "couldn't allocate gk20a support");
		return -ENOMEM;
	}

	init_waitqueue_head(&g->sw_irq_stall_last_handled_wq);
	init_waitqueue_head(&g->sw_irq_nonstall_last_handled_wq);

	platform->g = g;
	g->dev = &pdev->dev;

	err = pci_enable_device(pdev);
	if (err)
		return err;
	pci_set_master(pdev);

	g->irq_stall = pdev->irq;
	g->irq_nonstall = pdev->irq;
	if (g->irq_stall < 0)
		return -ENXIO;
	err = devm_request_threaded_irq(&pdev->dev,
			g->irq_stall,
			nvgpu_pci_isr,
			nvgpu_pci_intr_thread,
			IRQF_SHARED, "nvgpu", g);
	if (err) {
		gk20a_err(&pdev->dev,
			"failed to request irq @ %d", g->irq_stall);
		return err;
	}
	disable_irq(g->irq_stall);

	if (strchr(dev_name(&pdev->dev), '%')) {
		gk20a_err(&pdev->dev, "illegal character in device name");
		return -EINVAL;
	}

	nodefmt = kasprintf(GFP_KERNEL, PCI_INTERFACE_NAME, dev_name(&pdev->dev));
	if (!nodefmt)
		return -ENOMEM;

	err = gk20a_user_init(&pdev->dev, nodefmt, &nvgpu_pci_class);
	kfree(nodefmt);
	nodefmt = NULL;
	if (err)
		return err;

	err = nvgpu_pci_init_support(pdev);
	if (err)
		return err;

	init_rwsem(&g->busy_lock);
	mutex_init(&platform->railgate_lock);

	spin_lock_init(&g->mc_enable_lock);

	gk20a_debug_init(&pdev->dev);

	/* Initialize the platform interface. */
	err = platform->probe(&pdev->dev);
	if (err) {
		gk20a_err(&pdev->dev, "platform probe failed");
		return err;
	}

	/* Set DMA parameters to allow larger sgt lists */
	pdev->dev.dma_parms = &g->dma_parms;
	dma_set_max_seg_size(&pdev->dev, UINT_MAX);

	g->gr_idle_timeout_default =
			CONFIG_GK20A_DEFAULT_TIMEOUT;
	if (tegra_platform_is_silicon())
		g->timeouts_enabled = true;

	g->runlist_interleave = true;

	g->timeslice_low_priority_us = 1300;
	g->timeslice_medium_priority_us = 2600;
	g->timeslice_high_priority_us = 5200;

	gk20a_create_sysfs(&pdev->dev);

	g->mm.has_physical_mode = false;
	g->mm.vidmem_is_vidmem = true;
#ifdef CONFIG_DEBUG_FS
	g->mm.ltc_enabled = true;
	g->mm.ltc_enabled_debug = true;
#endif
	g->mm.bypass_smmu = platform->bypass_smmu;
	g->mm.disable_bigpage = platform->disable_bigpage;

	gk20a_init_gr(g);

	err = nvgpu_pci_pm_init(&pdev->dev);
	if (err) {
		gk20a_err(&pdev->dev, "pm init failed");
		return err;
	}

	return 0;
}

static void nvgpu_pci_remove(struct pci_dev *pdev)
{
	struct gk20a_platform *platform = gk20a_get_platform(&pdev->dev);
	struct gk20a *g = get_gk20a(&pdev->dev);

	if (g->remove_support)
		g->remove_support(g->dev);

	gk20a_user_deinit(g->dev, &nvgpu_pci_class);

	debugfs_remove_recursive(platform->debugfs);
	debugfs_remove_recursive(platform->debugfs_alias);

	gk20a_remove_sysfs(g->dev);

	if (platform->remove)
		platform->remove(g->dev);

	kfree(g);
}

static struct pci_driver nvgpu_pci_driver = {
	.name = "nvgpu",
	.id_table = nvgpu_pci_table,
	.probe = nvgpu_pci_probe,
	.remove = nvgpu_pci_remove,
#ifdef CONFIG_PM
	.driver.pm = &nvgpu_pci_pm_ops,
#endif
};

int __init nvgpu_pci_init(void)
{
	int ret;

	ret = class_register(&nvgpu_pci_class);
	if (ret)
		return ret;

	return pci_register_driver(&nvgpu_pci_driver);
}

void __exit nvgpu_pci_exit(void)
{
	pci_unregister_driver(&nvgpu_pci_driver);
	class_unregister(&nvgpu_pci_class);
}
