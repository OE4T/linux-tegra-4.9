/*
 * Tegra Graphics Host Hardware Context Interface
 *
 * Copyright (c) 2013-2014, NVIDIA Corporation.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/nvhost_ioctl.h>
#include <linux/dma-buf.h>
#include "user_hwctx.h"
#include "nvhost_cdma.h"
#include "nvhost_channel.h"
#include "host1x/host1x.h"
#include "host1x/host1x01_hardware.h"

static void user_hwctx_save_push(struct nvhost_hwctx *nctx,
		struct nvhost_cdma *cdma)
{
	struct user_hwctx *ctx = to_user_hwctx(nctx);
	void *cpuva = dma_buf_vmap(ctx->save_buf);
	dma_addr_t iova = sg_dma_address(ctx->save_sgt->sgl);

	nvhost_cdma_push_gather(cdma,
			cpuva,
			iova,
			ctx->save_offset,
			nvhost_opcode_gather(ctx->save_size),
			iova);

	dma_buf_vunmap(ctx->save_buf, cpuva);
}

static void user_hwctx_restore_push(struct nvhost_hwctx *nctx,
		struct nvhost_cdma *cdma)
{
	struct user_hwctx *ctx = to_user_hwctx(nctx);
	void *cpuva = dma_buf_vmap(ctx->restore_buf);
	dma_addr_t iova = sg_dma_address(ctx->restore_sgt->sgl);

	nvhost_cdma_push_gather(cdma,
			cpuva,
			iova,
			ctx->restore_offset,
			nvhost_opcode_gather(ctx->restore_size),
			iova);

	dma_buf_vunmap(ctx->restore_buf, cpuva);
}

static void user_hwctx_free(struct kref *ref)
{
	struct nvhost_hwctx *hwctx =
		container_of(ref, struct nvhost_hwctx, ref);
	struct user_hwctx *uhwctx = to_user_hwctx(hwctx);

	user_ctxhandler_free(hwctx->h);

	if (uhwctx->save_sgt) {
		dma_buf_unmap_attachment(uhwctx->save_attach, uhwctx->save_sgt,
							DMA_BIDIRECTIONAL);
		dma_buf_detach(uhwctx->save_buf, uhwctx->save_attach);
		dma_buf_put(uhwctx->save_buf);
	}

	if (uhwctx->restore_sgt) {
		dma_buf_unmap_attachment(uhwctx->restore_attach,
				uhwctx->restore_sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(uhwctx->restore_buf, uhwctx->restore_attach);
		dma_buf_put(uhwctx->restore_buf);
	}

	kfree(uhwctx);
}

static void user_hwctx_put(struct nvhost_hwctx *ctx)
{
	kref_put(&ctx->ref, user_hwctx_free);
}

static void user_hwctx_get(struct nvhost_hwctx *ctx)
{
	kref_get(&ctx->ref);
}

static struct nvhost_hwctx *user_hwctx_alloc(struct nvhost_hwctx_handler *h,
		struct nvhost_channel *ch)
{
	struct user_hwctx *hwctx;

	hwctx = kzalloc(sizeof(*hwctx), GFP_KERNEL);

	if (!hwctx)
		return NULL;

	kref_init(&hwctx->hwctx.ref);
	hwctx->hwctx.h = h;
	hwctx->hwctx.channel = ch;
	hwctx->hwctx.valid = false;
	hwctx->hwctx.save_slots = 1;

	return &hwctx->hwctx;
}

int user_hwctx_set_save(struct user_hwctx *ctx,
		ulong mem, u32 offset, u32 words, struct nvhost_reloc *reloc)
{
	void *page_addr;

	/* First the restore buffer is set, then the save buffer */
	if (!ctx->restore_buf || !ctx->restore_sgt)
		return -EINVAL;

	ctx->save_buf = dma_buf_get(mem);
	if (IS_ERR_OR_NULL(ctx->save_buf))
		return -ENOMEM;

	ctx->save_attach = dma_buf_attach(ctx->save_buf,
					&ctx->hwctx.channel->dev->dev);
	if (IS_ERR_OR_NULL(ctx->save_attach))
		goto err_buf_put;

	ctx->save_sgt = dma_buf_map_attachment(ctx->save_attach,
						DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(ctx->save_sgt))
		goto err_buf_detach;

	ctx->save_offset = offset;
	ctx->save_size = words;

	/* Patch restore buffer address into save buffer */
	page_addr = dma_buf_kmap(ctx->save_buf,
			reloc->cmdbuf_offset >> PAGE_SHIFT);
	if (!page_addr)
		goto err_buf_unmap;

	__raw_writel(sg_dma_address(ctx->restore_sgt->sgl) + offset,
			page_addr + (reloc->cmdbuf_offset & ~PAGE_MASK));
	dma_buf_kunmap(ctx->save_buf,
			reloc->cmdbuf_offset >> PAGE_SHIFT,
			page_addr);

	return 0;

 err_buf_unmap:
	dma_buf_unmap_attachment(ctx->save_attach, ctx->save_sgt,
						DMA_BIDIRECTIONAL);
 err_buf_detach:
	dma_buf_detach(ctx->save_buf, ctx->save_attach);
 err_buf_put:
	dma_buf_put(ctx->save_buf);

	return -ENOMEM;
}

int user_hwctx_set_restore(struct user_hwctx *ctx,
		ulong mem, u32 offset, u32 words)
{
	ctx->restore_buf = dma_buf_get(mem);
	if (IS_ERR_OR_NULL(ctx->restore_buf))
		return -ENOMEM;

	ctx->restore_attach = dma_buf_attach(ctx->restore_buf,
					&ctx->hwctx.channel->dev->dev);
	if (IS_ERR_OR_NULL(ctx->restore_attach))
		goto err_buf_put;

	ctx->restore_sgt = dma_buf_map_attachment(ctx->restore_attach,
						DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(ctx->restore_sgt))
		goto err_buf_detach;

	ctx->restore_offset = offset;
	ctx->restore_size = words;

	return 0;

 err_buf_detach:
	dma_buf_detach(ctx->restore_buf, ctx->restore_attach);
 err_buf_put:
	dma_buf_put(ctx->restore_buf);

	return -ENOMEM;
}

struct nvhost_hwctx_handler *user_ctxhandler_init(u32 syncpt,
		u32 waitbase, struct nvhost_channel *ch)
{
	struct user_hwctx_handler *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	p->h.syncpt = syncpt;
	p->h.waitbase = waitbase;

	p->h.alloc = user_hwctx_alloc;
	p->h.save_push = user_hwctx_save_push;
	p->h.restore_push = user_hwctx_restore_push;
	p->h.get = user_hwctx_get;
	p->h.put = user_hwctx_put;

	return &p->h;
}

void user_ctxhandler_free(struct nvhost_hwctx_handler *h)
{
	kfree(to_user_hwctx_handler(h));
}
