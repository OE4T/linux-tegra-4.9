/*
 * util.c: Utility functions for tegradc ext interface.
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION, All rights reserved.
 *
 * Author: Robert Morell <rmorell@nvidia.com>
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
 */

#include <linux/err.h>
#include <linux/types.h>
#include <linux/dma-buf.h>

#include "../dc.h"
#include "tegra_dc_ext_priv.h"


int tegra_dc_ext_pin_window(struct tegra_dc_ext_user *user, u32 fd,
			    struct tegra_dc_dmabuf **dc_buf,
			    dma_addr_t *phys_addr, u32 map_buffer_flag)
{
	struct tegra_dc_ext *ext = user->ext;
	struct tegra_dc_dmabuf *dc_dmabuf;
	dma_addr_t dma_addr;

	*dc_buf = NULL;
	*phys_addr = -1;
	if (!fd)
		return 0;

	dc_dmabuf = kzalloc(sizeof(*dc_dmabuf), GFP_KERNEL);
	if (!dc_dmabuf)
		return -ENOMEM;

	dc_dmabuf->buf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dc_dmabuf->buf))
		goto buf_fail;

	dc_dmabuf->attach = dma_buf_attach(dc_dmabuf->buf, ext->dev->parent);
	if (IS_ERR_OR_NULL(dc_dmabuf->attach))
		goto attach_fail;

	dc_dmabuf->sgt = dma_buf_map_attachment(dc_dmabuf->attach,
		map_buffer_flag);
	if (IS_ERR_OR_NULL(dc_dmabuf->sgt))
		goto sgt_fail;

	if (!device_is_iommuable(ext->dev->parent) &&
			sg_nents(dc_dmabuf->sgt->sgl) > 1) {
		dev_err(ext->dev->parent,
			"Cannot use non-contiguous buffer w/ IOMMU disabled\n");
		goto iommu_fail;
	}

	dma_addr = sg_dma_address(dc_dmabuf->sgt->sgl);
	if (dma_addr)
		*phys_addr = dma_addr;
	else
		*phys_addr = sg_phys(dc_dmabuf->sgt->sgl);

	*dc_buf = dc_dmabuf;

	return 0;
iommu_fail:
	dma_buf_unmap_attachment(dc_dmabuf->attach, dc_dmabuf->sgt,
		DMA_TO_DEVICE);
sgt_fail:
	dma_buf_detach(dc_dmabuf->buf, dc_dmabuf->attach);
attach_fail:
	dma_buf_put(dc_dmabuf->buf);
buf_fail:
	kfree(dc_dmabuf);
	return -ENOMEM;
}
