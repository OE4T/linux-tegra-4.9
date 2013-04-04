/*
 * drivers/video/tegra/host/nvmap.c
 *
 * Tegra Graphics Host Nvmap support
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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
#include <linux/nvmap.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/err.h>
#include <trace/events/nvhost.h>
#include "nvmap.h"
#include "nvhost_job.h"
#include "chip_support.h"

#define FLAG_NVHOST_MAPPED 1
#define FLAG_NVMAP_MAPPED 2
#define FLAG_CARVEOUT 3

struct nvhost_nvmap_data {
	struct nvmap_client *client;
	struct nvmap_handle_ref *ref;
	struct sg_table *sgt;
	size_t len;
	struct device *dev;
	int pin_count;
	int flags;
	struct mutex lock;
};

struct mem_mgr *nvhost_nvmap_alloc_mgr(void)
{
	return (struct mem_mgr *)nvmap_create_client(nvmap_dev, "nvhost");
}

void nvhost_nvmap_put_mgr(struct mem_mgr *mgr)
{
	nvmap_client_put((struct nvmap_client *)mgr);
}

struct mem_mgr *nvhost_nvmap_get_mgr(struct mem_mgr *mgr)
{
	return (struct mem_mgr *)nvmap_client_get((struct nvmap_client *)mgr);
}

struct mem_mgr *nvhost_nvmap_get_mgr_file(int fd)
{
	return (struct mem_mgr *)nvmap_client_get_file(fd);
}

struct mem_handle *nvhost_nvmap_alloc(struct mem_mgr *mgr,
		size_t size, size_t align, int flags, unsigned int heap_mask)
{
	return (struct mem_handle *)nvmap_alloc((struct nvmap_client *)mgr,
			size, align, flags, heap_mask);
}

void nvhost_nvmap_put(struct mem_mgr *mgr, struct mem_handle *handle)
{
	if (!handle)
		return;
	_nvmap_free((struct nvmap_client *)mgr,
			(struct nvmap_handle_ref *)handle);
}

static void unmap_pages(struct device *dev, struct sg_table *sgt, size_t len)
{
	DEFINE_DMA_ATTRS(attrs);
	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_unmap_sg_attrs(dev, sgt->sgl, sgt->nents, 0, &attrs);
}

void delete_priv(void *_priv)
{
	struct nvhost_nvmap_data *priv = _priv;
	if (priv->sgt) {
		if (priv->flags & BIT(FLAG_NVHOST_MAPPED))
			unmap_pages(priv->dev, priv->sgt, priv->len);
		nvmap_free_sg_table(priv->client, priv->ref, priv->sgt);
	}
	kfree(priv);
}

struct sg_table *nvhost_nvmap_pin(struct mem_mgr *mgr,
		struct mem_handle *handle,
		struct device *dev)
{
	struct nvmap_handle_ref *ref = (struct nvmap_handle_ref *)handle;
	struct nvmap_client *nvmap = (struct nvmap_client *)mgr;
	struct nvhost_nvmap_data *priv = nvmap_get_nvhost_private(ref);
	struct sg_table *sgt;
	int ret;

	if (!priv) {
		u64 size = 0, addr = 0;

		ret = nvmap_get_handle_param(nvmap, ref,
				NVMAP_HANDLE_PARAM_SIZE, &size);
		if (ret)
			return ERR_PTR(ret);

		ret = nvmap_get_handle_param(nvmap, ref,
				NVMAP_HANDLE_PARAM_BASE, &addr);
		if (ret)
			return ERR_PTR(ret);

		priv = kzalloc(sizeof(*priv), GFP_KERNEL);
		if (!priv)
			return ERR_PTR(-ENOMEM);

		mutex_init(&priv->lock);
		priv->client = nvmap;
		priv->ref = ref;
		priv->sgt = nvmap_sg_table(nvmap, ref);
		priv->dev = dev;
		priv->len = size;
		if (IS_ERR(priv->sgt)) {
			kfree(priv);
			return priv->sgt;
		}
		nvmap_set_nvhost_private(ref, priv, delete_priv);

		if (!IS_ERR_VALUE(addr))
			priv->flags |= BIT(FLAG_NVMAP_MAPPED);
	}

	mutex_lock(&priv->lock);
	sgt = priv->sgt;

	if (priv->flags & BIT(FLAG_NVMAP_MAPPED) &&
			sg_dma_address(sgt->sgl) == 0) {
		dma_addr_t addr = 0;
		int err = nvmap_pin(priv->client, ref, &addr);
		if (err)
			return ERR_PTR(err);

		sg_dma_address(sgt->sgl) = addr;
	} else if (priv->pin_count == 0 && sg_dma_address(sgt->sgl) == 0) {
		int ents;
		DEFINE_DMA_ATTRS(attrs);
		dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);

		ents = dma_map_sg_attrs(dev, sgt->sgl, sgt->nents, 0, &attrs);
		if (!ents) {
			mutex_unlock(&priv->lock);
			return ERR_PTR(-ENOMEM);
		}
		priv->flags |= BIT(FLAG_NVHOST_MAPPED);
	}
	trace_nvhost_nvmap_pin(dev_name(dev),
			ref, priv->len, sg_dma_address(sgt->sgl));

	priv->pin_count++;
	mutex_unlock(&priv->lock);
	nvmap_flush_deferred_cache(nvmap, ref);
	return sgt;
}

void nvhost_nvmap_unpin(struct mem_mgr *mgr, struct mem_handle *handle,
		struct device *dev, struct sg_table *sgt)
{
	struct nvmap_handle_ref *ref = (struct nvmap_handle_ref *)handle;
	struct nvhost_nvmap_data *priv = nvmap_get_nvhost_private(ref);
	if (!priv)
		return;

	mutex_lock(&priv->lock);
	priv->pin_count--;
	WARN_ON(priv->pin_count < 0);
	mutex_unlock(&priv->lock);
	trace_nvhost_nvmap_unpin(dev_name(dev),
			ref, priv->len, sg_dma_address(sgt->sgl));
}

void *nvhost_nvmap_mmap(struct mem_handle *handle)
{
	return nvmap_mmap((struct nvmap_handle_ref *)handle);
}

void nvhost_nvmap_munmap(struct mem_handle *handle, void *addr)
{
	nvmap_munmap((struct nvmap_handle_ref *)handle, addr);
}

void *nvhost_nvmap_kmap(struct mem_handle *handle, unsigned int pagenum)
{
	return nvmap_kmap((struct nvmap_handle_ref *)handle, pagenum);
}

void nvhost_nvmap_kunmap(struct mem_handle *handle, unsigned int pagenum,
		void *addr)
{
	nvmap_kunmap((struct nvmap_handle_ref *)handle, pagenum, addr);
}

struct mem_handle *nvhost_nvmap_get(struct mem_mgr *mgr,
		ulong id, struct platform_device *dev)
{
	return (struct mem_handle *)
		nvmap_duplicate_handle_user_id((struct nvmap_client *)mgr, id);
}

int nvhost_nvmap_get_param(struct mem_mgr *mgr, struct mem_handle *handle,
		u32 param, u64 *result)
{
	return nvmap_get_handle_param((struct nvmap_client *)mgr,
			(struct nvmap_handle_ref *)handle,
			param, result);
}
