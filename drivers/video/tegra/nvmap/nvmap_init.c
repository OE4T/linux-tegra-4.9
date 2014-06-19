/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/of.h>

#include <linux/nvmap.h>

#include "nvmap_priv.h"
#include "iomap.h"
#include "board.h"
#include "common.h"

static struct nvmap_platform_carveout nvmap_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		.dma_dev	= &tegra_iram_dev,
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,
		.size		= 0,
		.dma_dev	= &tegra_generic_dev,
	},
	[2] = {
		.name		= "vpr",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_VPR,
		.base		= 0,
		.size		= 0,
		.dma_dev	= &tegra_vpr_dev,
	},
};

static struct nvmap_platform_data nvmap_data = {
	.carveouts	= nvmap_carveouts,
	.nr_carveouts	= ARRAY_SIZE(nvmap_carveouts),
};

struct platform_device nvmap_platform_device  = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &nvmap_data,
	},
};

struct platform_device *nvmap_get_platform_dev(void)
{
	return &nvmap_platform_device;
}

int __init nvmap_init(void)
{
	int err = 0;
#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	struct dma_declare_info vpr_dma_info;
	struct dma_declare_info generic_dma_info;
#endif

#ifdef CONFIG_TEGRA_NVMAP
	nvmap_carveouts[1].base = tegra_carveout_start;
	nvmap_carveouts[1].size = tegra_carveout_size;
	nvmap_carveouts[1].dma_dev = &tegra_generic_dev;
	nvmap_carveouts[2].base = tegra_vpr_start;
	nvmap_carveouts[2].size = tegra_vpr_size;
	nvmap_carveouts[2].dma_dev = &tegra_vpr_dev;

#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	generic_dma_info.name = "generic";
	generic_dma_info.base = tegra_carveout_start;
	generic_dma_info.size = tegra_carveout_size;
	generic_dma_info.resize = false;
	generic_dma_info.cma_dev = NULL;

	vpr_dma_info.name = "vpr";
	vpr_dma_info.base = tegra_vpr_start;
	vpr_dma_info.size = SZ_32M;
	vpr_dma_info.resize = true;
	vpr_dma_info.cma_dev = &tegra_vpr_cma_dev;
	vpr_dma_info.notifier.ops = &vpr_dev_ops;

	carveout_linear_set(&tegra_generic_cma_dev);
	nvmap_carveouts[1].cma_dev = &tegra_generic_cma_dev;
	nvmap_carveouts[1].resize = false;
	carveout_linear_set(&tegra_vpr_cma_dev);
	nvmap_carveouts[2].cma_dev = &tegra_vpr_cma_dev;
	nvmap_carveouts[2].resize = true;

	if (tegra_carveout_size) {
		err = dma_declare_coherent_resizable_cma_memory(
				&tegra_generic_dev, &generic_dma_info);
		if (err) {
			pr_err("Generic coherent memory declaration failed\n");
			return err;
		}
	}
	if (tegra_vpr_size) {
		err = dma_declare_coherent_resizable_cma_memory(
				&tegra_vpr_dev, &vpr_dma_info);
		if (err) {
			pr_err("VPR coherent memory declaration failed\n");
			return err;
		}
	}
#endif

	err = platform_device_register(&nvmap_platform_device);
#endif
	return err;
}
arch_initcall(nvmap_init);

static int nvmap_remove(struct platform_device *pdev)
{
	struct nvmap_device *dev = platform_get_drvdata(pdev);
	struct rb_node *n;
	struct nvmap_handle *h;
	int i;

	misc_deregister(&dev->dev_user);

	while ((n = rb_first(&dev->handles))) {
		h = rb_entry(n, struct nvmap_handle, node);
		rb_erase(&h->node, &dev->handles);
		kfree(h);
	}

	for (i = 0; i < dev->nr_carveouts; i++) {
		struct nvmap_carveout_node *node = &dev->heaps[i];
		nvmap_heap_destroy(node->carveout);
	}
	kfree(dev->heaps);

	kfree(dev);
	nvmap_dev = NULL;
	return 0;
}

static int nvmap_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nvmap_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver nvmap_driver = {
	.probe		= nvmap_probe,
	.remove		= nvmap_remove,
	.suspend	= nvmap_suspend,
	.resume		= nvmap_resume,

	.driver = {
		.name	= "tegra-nvmap",
		.owner	= THIS_MODULE,
	},
};

static int __init nvmap_init_driver(void)
{
	int e;

	nvmap_dev = NULL;

	e = nvmap_heap_init();
	if (e)
		goto fail;

	e = platform_driver_register(&nvmap_driver);
	if (e) {
		nvmap_heap_deinit();
		goto fail;
	}

fail:
	return e;
}
fs_initcall(nvmap_init_driver);

static void __exit nvmap_exit_driver(void)
{
	platform_driver_unregister(&nvmap_driver);
	nvmap_heap_deinit();
	nvmap_dev = NULL;
}
module_exit(nvmap_exit_driver);
