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
		struct sg_table sgt;
	} scratch;
	struct device *vi_dev;
};

static int tegra_rce_rm_remove(struct platform_device *pdev)
{
	struct tegra_rce_rm *rm = platform_get_drvdata(pdev);
	struct sg_table *sgt = &rm->scratch.sgt;

	if (sgt->sgl) {
		dma_unmap_sg(rm->vi_dev, sgt->sgl, sgt->orig_nents,
			DMA_FROM_DEVICE);
		sg_free_table(sgt);
	}

	dma_free_coherent(&pdev->dev, rm->scratch.size,
		rm->scratch.base, rm->scratch.dma);

	put_device(rm->vi_dev);

	return 0;
}

static int tegra_rce_rm_probe(struct platform_device *pdev)
{
	struct tegra_rce_rm *rm;
	struct device *dev = &pdev->dev;
	struct device_node *vi_np;
	struct platform_device *vi_pdev = NULL;
	int ret;
	void *base;
	uint32_t size;
	dma_addr_t dma;

	rm = devm_kzalloc(dev, sizeof(*rm), GFP_KERNEL);
	if (rm == NULL)
		return -ENOMEM;

	vi_np = of_parse_phandle(dev->of_node, "nvidia,vi-device", 0);
	if (vi_np != NULL) {
		vi_pdev = of_find_device_by_node(vi_np);
		of_node_put(vi_np);

		if (vi_pdev != NULL)
			rm->vi_dev = &vi_pdev->dev;
		else
			dev_WARN(dev, "no vi device\n");
	} else {
		dev_WARN(dev, "missing %s handle\n", "nvidia,vi-device");
	}

	ret = of_property_read_u32(dev->of_node, NV(scratch-size), &size);
	if (ret != 0)
		size = 1024 * 1024;

	base = dma_alloc_coherent(dev, size, &dma, GFP_KERNEL | __GFP_ZERO);
	if (IS_ERR(base))
		return PTR_ERR(base);

	rm->scratch.base = base;
	rm->scratch.size = size;
	rm->scratch.dma = dma;
	dev_info(dev, "allocated scratch at 0x%llx..0x%llx\n",
		(u64)dma, (u64)dma + size);

	if (rm->vi_dev) {
		struct sg_table *sgt = &rm->scratch.sgt;
		int ret = dma_get_sgtable(rm->vi_dev, sgt, base, dma, size);
		if (ret < 0)
			goto error;
		if (!dma_map_sg(rm->vi_dev, sgt->sgl, sgt->orig_nents,
					DMA_FROM_DEVICE)) {
			dev_err(dev, "failed to map vi scratch at 0x%llx\n",
				(u64)dma);
			sg_free_table(sgt);
			ret = -ENXIO;
			goto error;
		}
		dev_info(dev, "mapped vi scratch at 0x%llx\n", (u64)dma);
	}

	return 0;

error:
	dma_free_coherent(dev, size, base, dma);
	return ret;
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
