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
#include "nvmap.h"

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
		size_t size, size_t align, int flags)
{
	return (struct mem_handle *)nvmap_alloc((struct nvmap_client *)mgr,
			size, align, flags, 0);
}

void nvhost_nvmap_put(struct mem_mgr *mgr, struct mem_handle *handle)
{
	nvmap_free((struct nvmap_client *)mgr,
			(struct nvmap_handle_ref *)handle);
}

static struct scatterlist *sg_kmalloc(unsigned int nents, gfp_t gfp_mask)
{
	return (struct scatterlist *)gfp_mask;
}

struct sg_table *nvhost_nvmap_pin(struct mem_mgr *mgr,
		struct mem_handle *handle)
{
	int err = 0;
	dma_addr_t ret = 0;
	struct sg_table *sgt = kmalloc(sizeof(*sgt) + sizeof(*sgt->sgl),
			GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	err = __sg_alloc_table(sgt, 1, 1, (gfp_t)(sgt+1), sg_kmalloc);
	if (err)
		return ERR_PTR(err);

	ret = nvmap_pin((struct nvmap_client *)mgr,
			(struct nvmap_handle_ref *)handle);
	if (IS_ERR_VALUE(ret)) {
		kfree(sgt);
		return ERR_PTR(ret);
	}
	sg_dma_address(sgt->sgl) = ret;

	return sgt;
}

void nvhost_nvmap_unpin(struct mem_mgr *mgr,
		struct mem_handle *handle, struct sg_table *sgt)
{
	kfree(sgt);

	return nvmap_unpin((struct nvmap_client *)mgr,
			(struct nvmap_handle_ref *)handle);
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
		u32 id, struct nvhost_device *dev)
{
	return (struct mem_handle *)
		nvmap_duplicate_handle_id((struct nvmap_client *)mgr, id);
}

