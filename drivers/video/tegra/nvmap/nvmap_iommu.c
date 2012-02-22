/*
 * IOMMU backend support for NVMAP
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

#include <linux/slab.h>
#include <mach/iomap.h>
#include <mach/iovmm.h>

#include "nvmap.h"

struct tegra_iovmm_area *tegra_iommu_create_vm(struct device *dev,
			       dma_addr_t req, size_t size, pgprot_t prot)
{
	struct tegra_iovmm_area *area;
	dma_addr_t iova;

	area = kmalloc(sizeof(*area), GFP_KERNEL);
	if (!area)
		return NULL;

	if (!req)
		req = DMA_ANON_ADDR;

	iova = arm_iommu_alloc_iova_at(dev, req, size);
	if (iova == DMA_ERROR_CODE)
		goto err_out;
	area->iovm_start = iova;
	area->iovm_length = size;
	area->pgprot = prot;
	area->dev = dev;
	return area;

err_out:
	kfree(area);
	return NULL;
}

void tegra_iommu_free_vm(struct tegra_iovmm_area *area)
{
	int i;
	size_t count = area->iovm_length >> PAGE_SHIFT;

	for (i = 0; i < count; i++) {
		dma_addr_t iova;

		iova = area->iovm_start + i * PAGE_SIZE;
		dma_unmap_page(area->dev, iova, PAGE_SIZE, DMA_NONE);
	}
	kfree(area);
}

struct tegra_iovmm_client *tegra_iommu_alloc_client(struct device *dev)
{
	struct dma_iommu_mapping *map;
	struct tegra_iovmm_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return NULL;

	map = arm_iommu_create_mapping(&platform_bus_type,
		       TEGRA_IOMMU_BASE, TEGRA_IOMMU_SIZE, 0);
	if (IS_ERR(map))
		goto err_map;

	if (arm_iommu_attach_device(dev, map))
		goto err_attach;
	client->dev = dev;
	return client;

err_attach:
	arm_iommu_release_mapping(map);
err_map:
	kfree(client);
	return NULL;
}

void tegra_iommu_free_client(struct tegra_iovmm_client *client)
{
	arm_iommu_release_mapping(client->dev->archdata.mapping);
	kfree(client);
}
