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
#include "nvhost_job.h"
#include "chip_support.h"

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

static struct scatterlist *sg_kmalloc(unsigned int nents, gfp_t gfp_mask)
{
#ifdef CONFIG_COMPAT
	return (struct scatterlist *)(((uintptr_t)gfp_mask)|PAGE_OFFSET);
#else
	return (struct scatterlist *)((uintptr_t)gfp_mask);
#endif
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

	err = __sg_alloc_table(sgt, 1, 1, (gfp_t)((uintptr_t)(sgt+1)), sg_kmalloc);
	if (err) {
		kfree(sgt);
		return ERR_PTR(err);
	}

	err = nvmap_pin((struct nvmap_client *)mgr,
			(struct nvmap_handle_ref *)handle, &ret);
	if (err) {
		kfree(sgt);
		return ERR_PTR(err);
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

int nvhost_nvmap_pin_array_ids(struct mem_mgr *mgr,
		long unsigned *ids,
		u32 id_type_mask,
		u32 id_type,
		u32 count,
		struct nvhost_job_unpin *unpin_data,
		dma_addr_t *phys_addr)
{
	int i;
	int result = 0;
	struct nvmap_handle **unique_handles;
	struct nvmap_handle_ref **unique_handle_refs;
	void *ptrs = kmalloc(sizeof(void *) * count * 2,
			GFP_KERNEL);

	if (!ptrs)
		return -ENOMEM;

	unique_handles = (struct nvmap_handle **) ptrs;
	unique_handle_refs = (struct nvmap_handle_ref **)
			&unique_handles[count];

	result = nvmap_pin_array((struct nvmap_client *)mgr,
		    (long unsigned *)ids, id_type_mask, id_type, count,
		    unique_handles,
		    unique_handle_refs);

	if (result < 0)
		goto fail;

	WARN_ON(result > count);

	for (i = 0; i < result; i++)
		unpin_data[i].h = (struct mem_handle *)unique_handle_refs[i];

	for (i = 0; i < count; i++) {
		if ((ids[i] & id_type_mask) == id_type)
			phys_addr[i] = (dma_addr_t)nvmap_get_addr_from_user_id(
								ids[i]);
	}

fail:
	kfree(ptrs);
	return result;
}

struct mem_handle *nvhost_nvmap_get(struct mem_mgr *mgr,
		ulong id, struct platform_device *dev)
{
	return (struct mem_handle *)
		nvmap_duplicate_handle_user_id((struct nvmap_client *)mgr, id);
}

int nvhost_nvmap_get_param(struct mem_mgr *mgr, struct mem_handle *handle,
		u32 param, u32 *result)
{
	return nvmap_get_handle_param_u32((struct nvmap_client *)mgr,
			nvmap_ref_to_handle((struct nvmap_handle_ref *)handle),
			param, result);
}

int nvhost_init_nvmap_support(struct nvhost_chip_support *chip)
{
	chip->mem.alloc_mgr = nvhost_nvmap_alloc_mgr;
	chip->mem.put_mgr = nvhost_nvmap_put_mgr;
	chip->mem.get_mgr = nvhost_nvmap_get_mgr;
	chip->mem.get_mgr_file = nvhost_nvmap_get_mgr_file;
	chip->mem.alloc = nvhost_nvmap_alloc;
	chip->mem.put = nvhost_nvmap_put;
	chip->mem.get = nvhost_nvmap_get;
	chip->mem.pin = nvhost_nvmap_pin;
	chip->mem.unpin = nvhost_nvmap_unpin;
	chip->mem.mmap = nvhost_nvmap_mmap;
	chip->mem.munmap = nvhost_nvmap_munmap;
	chip->mem.get_param = nvhost_nvmap_get_param;

	return 0;
}
