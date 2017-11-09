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
#include <linux/tegra-rce-rm.h>
#include <linux/dma-mapping.h>
#include <linux/dma-attrs.h>

#define NV(p) "nvidia," #p

struct tegra_rce_rm {
	struct {
		size_t size;
		void *base;
		dma_addr_t dma;
	} scratch;
	struct device_dma_parameters dma_parms;
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
	dma_addr_t dma;
	u32 scratch_area[2];

	rm = devm_kzalloc(dev, sizeof(*rm), GFP_KERNEL);
	if (rm == NULL)
		return -ENOMEM;

	dev->dma_parms = &rm->dma_parms;
	dma_set_max_seg_size(dev, UINT_MAX);

	ret = of_property_read_u32_array(dev->of_node, NV(scratch-area),
					scratch_area, ARRAY_SIZE(scratch_area));
	if (ret) {
		dev_warn(dev, "%s: scratch-area property not defined",
			__func__);
		scratch_area[0] = 0x80000000;
		scratch_area[1] = 0x00600000;
	}

	dma = scratch_area[0];
	base = get_dma_ops(dev)->alloc(dev, scratch_area[1], &dma,
				GFP_KERNEL | __GFP_ZERO, 0);

	if (IS_ERR(base)) {
		kfree(rm);
		return PTR_ERR(base);
	}
	if (dma != scratch_area[0]) {
		dev_err(dev, "%s: incorrect scratch area mapping", __func__);
		dma_free_coherent(dev, scratch_area[1], base, dma);
		kfree(rm);
		return -ENOMEM;
	}

	rm->scratch.base = base;
	rm->scratch.size = scratch_area[1];
	rm->scratch.dma = dma;

	dev_info(dev, "allocated scratch at 0x%llx..0x%llx\n",
		(u64)dma, (u64)dma + scratch_area[1]);

	platform_set_drvdata(pdev, rm);
	return 0;
}

int rce_rm_map_carveout_for_device(struct platform_device *pdev,
				struct device *dev, struct sg_table *sgt)
{
	struct tegra_rce_rm *rm = platform_get_drvdata(pdev);
	int err = 0;
	dma_addr_t map_addr;
	struct scatterlist *s = sgt->sgl;
	int i = 0;
	int mapped_count = 0;
	unsigned int size = 0;
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, __DMA_ATTR(attrs));

	if (!rm->scratch.base)
		return -ENOMEM;

	err = dma_get_sgtable(dev, sgt, rm->scratch.base,
			rm->scratch.dma, rm->scratch.size);
	if (err < 0)
		return err;

	/* Map the pages in the sg to the scratch IOVA */
	map_addr = rm->scratch.dma;
	for_each_sg(sgt->sgl, s, sgt->nents, i) {
		dma_addr_t ret;
		phys_addr_t phys = page_to_phys(sg_page(s)) + s->offset;
		ret = get_dma_ops(dev)->map_at(dev, map_addr, phys, s->length,
				DMA_FROM_DEVICE, __DMA_ATTR(attrs));
		if (ret == DMA_ERROR_CODE) {
			mapped_count = i;
			goto unmap;
		}

		s->dma_address = DMA_ERROR_CODE;
		s->dma_length  = 0;

		size     += s->length;
		map_addr += s->length;
	}
	sgt->sgl->dma_address = rm->scratch.dma;
	sgt->sgl->dma_length  = size;

	pdev->dev.dma_parms = NULL;

	return 0;

unmap:
	/* Unmap all of the pages we mapped so far */
	dma_unmap_page(dev, rm->scratch.dma, size, DMA_FROM_DEVICE);
	return -ENOMEM;
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
