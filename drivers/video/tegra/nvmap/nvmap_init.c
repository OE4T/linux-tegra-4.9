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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <linux/nvmap.h>

#include "nvmap_priv.h"
#include "iomap.h"
#include "board.h"
#include <linux/platform/tegra/common.h>

phys_addr_t __weak tegra_carveout_start;
phys_addr_t __weak  tegra_carveout_size;
phys_addr_t __weak tegra_vpr_start;
phys_addr_t __weak tegra_vpr_size;
bool __weak tegra_vpr_resize;

struct device __weak tegra_generic_dev;
struct device __weak tegra_vpr_dev;
struct device __weak tegra_iram_dev;
struct device __weak tegra_generic_cma_dev;
struct device __weak tegra_vpr_cma_dev;
struct dma_resize_notifier_ops __weak vpr_dev_ops;

static const struct of_device_id nvmap_of_ids[] = {
	{ .compatible = "nvidia,carveouts" },
	{ }
};

/*
 * Order of this must match nvmap_carveouts;
 */
static char *nvmap_carveout_names[] = {
	"iram",
	"generic",
	"vpr"
};

static struct nvmap_platform_carveout nvmap_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= 0,
		.size		= 0,
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

/*
 * In case there is no DT entry.
 */
struct platform_device nvmap_platform_device  = {
	.name	= "tegra-carveouts",
	.id	= -1,
	.dev	= {
		.platform_data = &nvmap_data,
	},
};

/*
 * @data must be at least a 2 element array of u64's.
 */
static int nvmap_populate_carveout(const void *data,
				   struct nvmap_platform_carveout *co)
{
	__be64 *co_data = (__be64 *)data;
	u64 base, size;

	base = be64_to_cpup(co_data);
	size = be64_to_cpup(co_data + 1);

	/*
	 * If base and size are 0, then assume the CO is not being populated.
	 */
	if (base == 0 && size == 0)
		return -ENODEV;

	if (size == 0)
		return -EINVAL;

	co->base = (phys_addr_t)base;
	co->size = (size_t)size;

	pr_info("Populating %s\n", co->name);
	pr_info("  base = 0x%08lx size = 0x%x\n",
		(unsigned long)base, (unsigned int)size);

	return 0;
}

static int __nvmap_init_dt(struct platform_device *pdev)
{
	const void *prop;
	int prop_len, i;

	if (!of_match_device(nvmap_of_ids, &pdev->dev)) {
		pr_err("Missing DT entry!\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(nvmap_carveout_names); i++) {
		prop = of_get_property(pdev->dev.of_node,
				       nvmap_carveout_names[i], &prop_len);
		if (!prop || prop_len != 4 * sizeof(u32)) {
			pr_err("Missing carveout for %s!\n",
			       nvmap_carveout_names[i]);
			continue;
		}
		nvmap_populate_carveout(prop, &nvmap_carveouts[i]);
	}

	pdev->dev.platform_data = &nvmap_data;

	return 0;
}

/*
 * This requires proper kernel arguments to have been passed.
 */
static int __nvmap_init_legacy(void)
{
	/* IRAM. */
	nvmap_carveouts[0].base = TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE;
	nvmap_carveouts[0].size = TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE;

	/* Carveout. */
	nvmap_carveouts[1].base = tegra_carveout_start;
	nvmap_carveouts[1].size = tegra_carveout_size;

	/* VPR */
	nvmap_carveouts[2].base = tegra_vpr_start;
	nvmap_carveouts[2].size = tegra_vpr_size;

	if (tegra_vpr_resize) {
		nvmap_carveouts[1].cma_dev = &tegra_generic_cma_dev;
		nvmap_carveouts[2].cma_dev = &tegra_vpr_cma_dev;
	}

	return 0;
}

/*
 * Fills in the platform data either from the device tree or with the
 * legacy path.
 */
int nvmap_init(struct platform_device *pdev)
{
	int err;
	struct dma_declare_info vpr_dma_info;
	struct dma_declare_info generic_dma_info;

	if (pdev->dev.platform_data) {
		err = __nvmap_init_legacy();
		if (err)
			return err;
	}

	if (pdev->dev.of_node) {
		err = __nvmap_init_dt(pdev);
		if (err)
			return err;
	}

	if (!tegra_vpr_resize)
		goto end;
	generic_dma_info.name = "generic";
	generic_dma_info.size = 0;
	generic_dma_info.cma_dev = nvmap_carveouts[1].cma_dev;

	vpr_dma_info.name = "vpr";
	vpr_dma_info.size = SZ_32M;
	vpr_dma_info.cma_dev = nvmap_carveouts[2].cma_dev;
	vpr_dma_info.notifier.ops = &vpr_dev_ops;

	if (nvmap_carveouts[1].size) {
		err = dma_declare_coherent_resizable_cma_memory(
				&tegra_generic_dev, &generic_dma_info);
		if (err) {
			pr_err("Generic coherent memory declaration failed\n");
			return err;
		}
	}
	if (nvmap_carveouts[2].size) {
		err = dma_declare_coherent_resizable_cma_memory(
				&tegra_vpr_dev, &vpr_dma_info);
		if (err) {
			pr_err("VPR coherent memory declaration failed\n");
			return err;
		}
	}

end:
	return err;
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
		.name	= "tegra-carveouts",
		.owner	= THIS_MODULE,
		.of_match_table = nvmap_of_ids,
	},
};

static int __init nvmap_init_driver(void)
{
	int e = 0;
	struct device_node *dnode;

	nvmap_dev = NULL;

	/* Pick DT vs legacy loading. */
	dnode = of_find_compatible_node(NULL, NULL, "nvidia,carveouts");
	if (!dnode)
		e = platform_device_register(&nvmap_platform_device);
	else
		of_node_put(dnode);

	if (e)
		goto fail;

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
