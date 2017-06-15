/*
 * Tegra capture common operations
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Sudhir Vyas <svyas@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/nvhost.h>
#include <linux/scatterlist.h>
#include <media/capture_common.h>
#include <media/mc_common.h>

struct surface_t {
	uint32_t offset;
	uint32_t offset_hi;
};

int capture_common_pin_memory(struct device *dev,
		uint32_t mem, struct capture_common_buf *unpin_data)
{
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	int err = 0;

	buf = dma_buf_get(mem);
	if (IS_ERR(buf)) {
		err = PTR_ERR(buf);
		goto fail;
	}

	attach = dma_buf_attach(buf, dev);
	if (IS_ERR(attach)) {
		err = PTR_ERR(attach);
		goto fail;
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		err = PTR_ERR(sgt);
		goto fail;
	}

	if (sg_dma_address(sgt->sgl) == 0)
		sg_dma_address(sgt->sgl) = sg_phys(sgt->sgl);

	unpin_data->iova = sg_dma_address(sgt->sgl);
	unpin_data->buf = buf;
	unpin_data->attach = attach;
	unpin_data->sgt = sgt;

	return 0;

fail:
	capture_common_unpin_memory(unpin_data);
	return err;
}

void capture_common_unpin_memory(struct capture_common_buf *unpin_data)
{
	if (unpin_data->sgt != NULL)
		dma_buf_unmap_attachment(unpin_data->attach, unpin_data->sgt,
				DMA_BIDIRECTIONAL);
	if (unpin_data->attach != NULL)
		dma_buf_detach(unpin_data->buf, unpin_data->attach);
	if (unpin_data->buf != NULL)
		dma_buf_put(unpin_data->buf);

	unpin_data->sgt = NULL;
	unpin_data->attach = NULL;
	unpin_data->buf = NULL;
	unpin_data->iova = 0;
}

int capture_common_request_pin_and_reloc(struct capture_common_pin_req *req)
{
	uint32_t num_relocs = req->relocs->num_relocs;
	void *reloc_page_addr = NULL;
	int last_page = -1;
	dma_addr_t surface_phys_addr = 0;
	int i;
	int err = 0;

	dev_dbg(req->dev, "%s: relocating %u addresses", __func__, num_relocs);

	for (i = 0; i < num_relocs; i++) {
		uint32_t reloc_relative = req->relocs->reloc_relatives[i];
		uint32_t reloc_offset = req->request_offset + reloc_relative;

		uint64_t surface_raw;
		struct surface_t *surface;
		uint32_t mem;
		uint32_t target_offset;
		dma_addr_t target_phys_addr;

		dev_dbg(req->dev,
			"%s: idx:%i reloc:%u reloc_offset:%u", __func__,
			i, reloc_relative, reloc_offset);

		/* locate page of request in capture desc reloc is on */
		if (last_page != reloc_offset >> PAGE_SHIFT) {
			if (reloc_page_addr != NULL)
				dma_buf_kunmap(req->requests->buf,
					last_page, reloc_page_addr);

			reloc_page_addr = dma_buf_kmap(req->requests->buf,
						reloc_offset >> PAGE_SHIFT);
			last_page = reloc_offset >> PAGE_SHIFT;

			if (unlikely(reloc_page_addr == NULL)) {
				dev_err(req->dev,
					"%s: couldn't map request\n", __func__);
				goto fail;
			}
		}

		/* read surf offset and mem handle from request descr */
		surface_raw = __raw_readq(
			(void __iomem *)(reloc_page_addr +
			(reloc_offset & ~PAGE_MASK)));
		surface = (struct surface_t *)&surface_raw;
		target_offset = surface->offset;
		mem = surface->offset_hi;

		dev_dbg(req->dev, "%s: hmem:0x%x offset:0x%x\n",
				__func__,
				mem, target_offset);

		if (!mem) {
			dev_err(req->dev,
					"%s: invalid mem handle\n", __func__);
			goto fail;
		}

		if (mem == req->requests_mem) {
			target_phys_addr = req->requests_dev->iova +
					req->request_offset + target_offset;
		} else {
			err = capture_common_pin_memory(req->dev, mem,
				&req->unpins->data[req->unpins->num_unpins]);
			if (err < 0) {
				dev_info(req->dev,
					"%s: pin memory failed pin count %d\n",
					__func__, req->unpins->num_unpins);
				goto fail;
			}
			surface_phys_addr = req->unpins->data[i].iova;
			target_phys_addr = surface_phys_addr + target_offset;

			req->unpins->num_unpins++;
		}

		dev_dbg(req->dev,
			"%s: surface addr 0x%lx at desc 0x%lx\n",
			__func__, (unsigned long)target_phys_addr,
			(unsigned long)reloc_page_addr +
			(reloc_offset & ~PAGE_MASK));

		/* write relocated physical address to request descr */
		__raw_writeq(
			target_phys_addr,
			(void __iomem *)(reloc_page_addr +
				(reloc_offset & ~PAGE_MASK)));

		dma_sync_single_range_for_device(req->rtcpu_dev,
			    req->requests->iova, req->request_offset,
			    req->request_size, DMA_TO_DEVICE);
	}

	return 0;

fail:
	if (reloc_page_addr != NULL)
		dma_buf_kunmap(req->requests->buf, last_page,
			reloc_page_addr);

	for (i = 0; i < req->unpins->num_unpins; i++)
		capture_common_unpin_memory(&req->unpins->data[i]);

	return err;
}
