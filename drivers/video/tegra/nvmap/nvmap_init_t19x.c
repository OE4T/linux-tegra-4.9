/*
 * drivers/video/tegra/nvmap/nvmap_init_t19x.c
 *
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

#define pr_fmt(fmt)	"nvmap: %s() " fmt, __func__

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/nvmap_t19x.h>

#include "nvmap_priv.h"

bool nvmap_version_t19x;

int nvmap_register_cvsram_carveout(struct device *dma_dev,
		phys_addr_t base, size_t size)
{
	struct nvmap_platform_carveout cvsram = {
		.name = "cvsram",
		.usage_mask = NVMAP_HEAP_CARVEOUT_CVSRAM,
		.disable_dynamic_dma_map = true,
		.no_cpu_access = true,
	};

	if (!base || !size || (base != PAGE_ALIGN(base)) ||
	    (size != PAGE_ALIGN(size)))
		return -EINVAL;
	cvsram.base = base;
	cvsram.size = size;

	if (!dma_dev)
		cvsram.dma_dev = dma_dev;
	return nvmap_create_carveout(&cvsram);
}
EXPORT_SYMBOL(nvmap_register_cvsram_carveout);

static struct nvmap_platform_carveout gosmem = {
	.name = "gosmem",
	.usage_mask = NVMAP_HEAP_CARVEOUT_GOS,
	.no_cpu_access = true,
};

static struct cv_dev_info *cvdev_info;
static int count = 0;

static void nvmap_gosmem_device_release(struct reserved_mem *rmem,
		struct device *dev)
{
	int i;
	struct reserved_mem_ops *rmem_ops =
		(struct reserved_mem_ops *)rmem->ops;

	for (i = 0; i < count; i++)
		of_node_put(cvdev_info[i].np);
	kfree(cvdev_info);
	rmem_ops->device_release(rmem, dev);
}

static int __init nvmap_gosmem_device_init(struct reserved_mem *rmem,
		struct device *dev)
{
	struct of_phandle_iter iter;
	struct device_node *np;
	DEFINE_DMA_ATTRS(attrs);
	phys_addr_t pa;
	int ret, i, idx, bytes;
	struct reserved_mem_ops *rmem_ops =
		(struct reserved_mem_ops *)rmem->ops;

	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, __DMA_ATTR(attrs));

	np = of_find_compatible_node(NULL, NULL, rmem->name);
	if (!np) {
		pr_err("Can't find the node using compatible\n");
		return -ENODEV;
	}

	of_property_for_each_phandle_with_args(iter, np, "cvdevs",
			NULL, 0)
		count++;

	(void)dma_alloc_attrs(gosmem.dma_dev, count * SZ_4K,
				&pa, DMA_MEMORY_NOMAP, __DMA_ATTR(attrs));
	if (dma_mapping_error(dev, pa)) {
		pr_err("Failed to allocate from Gos mem carveout\n");
		return -ENOMEM;
	}

	bytes = sizeof(*cvdev_info) * count;
	bytes += sizeof(struct sg_table) * count * count;
	cvdev_info = kmalloc(bytes, GFP_KERNEL);
	if (!cvdev_info) {
		pr_err("kmalloc failed. No memory!!!\n");
		ret = -ENOMEM;
		goto unmap_dma;
	}

	idx = 0;
	of_property_for_each_phandle_with_args(iter, np, "cvdevs",
			NULL, 0) {
		struct device_node *temp = device_node_from_iter(iter);

		cvdev_info[idx].np = of_node_get(temp);
		if (!cvdev_info[idx].np)
			continue;
		cvdev_info[idx].count = count;
		cvdev_info[idx].idx = idx;
		cvdev_info[idx].sgt =
			(struct sg_table *)(cvdev_info + count);
		cvdev_info[idx].sgt += idx * count;

		for (i = 0; i < count; i++) {
			struct sg_table *sgt = cvdev_info[idx].sgt + i;

			sg_set_buf(sgt->sgl,
				phys_to_virt(pa + SZ_4K * idx), SZ_4K);
		}
		idx++;
	}
	ret = rmem_ops->device_init(rmem, dev);
	if (ret)
		goto free;
	return ret;
free:
	kfree(cvdev_info);
unmap_dma:
	dma_free_attrs(gosmem.dma_dev, SZ_4K, NULL, pa, __DMA_ATTR(attrs));
	return ret;
}

static struct reserved_mem_ops gosmem_rmem_ops = {
	.device_init = nvmap_gosmem_device_init,
	.device_release = nvmap_gosmem_device_release,
};

int __init nvmap_gosmem_setup(struct reserved_mem *rmem)
{
	int ret;

	rmem->priv = &gosmem;
	ret = nvmap_co_setup(rmem);
	if (ret)
		return ret;

	rmem->priv = (struct reserved_mem_ops *)rmem->ops;
	rmem->ops = &gosmem_rmem_ops;
	return 0;
}
RESERVEDMEM_OF_DECLARE(nvmap_co, "nvidia,gosmem", nvmap_gosmem_setup);

static int nvmap_gosmem_notifier(struct notifier_block *nb,
		unsigned long event, void *_dev)
{
	struct device *dev = _dev;
	int ents, i;

	if ((event != BUS_NOTIFY_BOUND_DRIVER) &&
		(event != BUS_NOTIFY_UNBIND_DRIVER))
		return NOTIFY_DONE;

	if ((event == BUS_NOTIFY_BOUND_DRIVER) &&
		nvmap_dev && (dev == nvmap_dev->dev_user.parent)) {
		struct of_device_id nvmap_t19x_of_ids[] = {
			{.compatible = "nvidia,tegra194-carveouts"},
			{ }
		};

		/*
		 * user space IOCTL and dmabuf ops happen much later in boot
		 * flow. So, setting the version here to ensure all of those
		 * callbacks can safely query the proper version of nvmap
		 */
		if (of_match_node((struct of_device_id *)&nvmap_t19x_of_ids,
				dev->of_node))
			nvmap_version_t19x = 1;
		return NOTIFY_DONE;
	}

	for (i = 0; i < count; i++)
		if (cvdev_info[i].np == dev->of_node)
			goto map_gosmem;
	return NOTIFY_DONE;
map_gosmem:
	for (i = 0; i < count; i++) {
		DEFINE_DMA_ATTRS(attrs);
		enum dma_data_direction dir;

		dir = DMA_BIDIRECTIONAL;
		dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, __DMA_ATTR(attrs));
		dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, __DMA_ATTR(attrs));
		if (cvdev_info[i].np != dev->of_node) {
			dma_set_attr(DMA_ATTR_READ_ONLY, __DMA_ATTR(attrs));
			dir = DMA_TO_DEVICE;
		}

		switch (event) {
		case BUS_NOTIFY_BOUND_DRIVER:
			ents = dma_map_sg_attrs(dev, cvdev_info[i].sgt->sgl,
					cvdev_info[i].sgt->nents, dir, __DMA_ATTR(attrs));
			if (ents != 1) {
				pr_err("mapping gosmem chunk %d for %s failed\n",
					i, dev_name(dev));
				return NOTIFY_DONE;
			}
			break;
		case BUS_NOTIFY_UNBIND_DRIVER:
			dma_unmap_sg_attrs(dev, cvdev_info[i].sgt->sgl,
					cvdev_info[i].sgt->nents, dir, __DMA_ATTR(attrs));
		default:
			return NOTIFY_DONE;
		};
	}
	return NOTIFY_DONE;
}

static struct notifier_block nvmap_gosmem_nb = {
	.notifier_call = nvmap_gosmem_notifier,
};

static int nvmap_t19x_init(void)
{
	return bus_register_notifier(&platform_bus_type,
			&nvmap_gosmem_nb);
}
core_initcall(nvmap_t19x_init);

struct cv_dev_info *nvmap_fetch_cv_dev_info(struct device *dev)
{
	int i;

	for (i = 0; i < count; i++)
		if (cvdev_info[i].np == dev->of_node)
			return &cvdev_info[i];
	return NULL;
}
