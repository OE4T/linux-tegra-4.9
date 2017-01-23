/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/init.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/tegra-ivc-bus.h>

#define NV(p) "nvidia," #p

struct tegra_rce_rm {
	struct {
		size_t size;
		void *base;
		dma_addr_t dma;
	} scratch;
};

static int tegra_rce_rm_remove(struct platform_device *pdev)
{
	struct tegra_rce_rm *rm = platform_get_drvdata(pdev);

	dma_free_coherent(&pdev->dev, rm->scratch.size,
		rm->scratch.base, rm->scratch.dma);

	return 0;
}

static int tegra_rce_rm_probe(struct platform_device *pdev)
{
	struct tegra_rce_rm *rm;
	struct device *dev = &pdev->dev;
	int ret;
	void *base;
	uint32_t size;

	rm = devm_kzalloc(dev, sizeof(*rm), GFP_KERNEL);
	if (rm == NULL)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, NV(scratch-size), &size);
	if (ret != 0)
		size = 1024 * 1024;

	base = dma_alloc_coherent(dev, size, &rm->scratch.dma,
			GFP_KERNEL | __GFP_ZERO);
	if (IS_ERR(base))
		return PTR_ERR(base);

	rm->scratch.base = base;
	rm->scratch.size = size;
	dev_info(dev, "allocated %u bytes of scratch\n", size);

	return 0;
}

static const struct of_device_id tegra_rce_rm_of_match[] = {
	{
		.compatible = NV(tegra194-rce-rm),
	},
	{ },
};

static struct platform_driver tegra_rce_rm_driver = {
	.driver = {
		.name	= "tegra194-rce-rm",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_rce_rm_of_match),
	},
	.probe = tegra_rce_rm_probe,
	.remove = tegra_rce_rm_remove,
};
module_platform_driver(tegra_rce_rm_driver);

MODULE_DESCRIPTION("CAMERA RTCPU RM driver");
MODULE_AUTHOR("Pekka Pessi <ppessi@nvidia.com>");
MODULE_LICENSE("GPL v2");
