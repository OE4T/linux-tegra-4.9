/*
 * drivers/video/tegra/host/nvhost_memmgr.c
 *
 * Tegra Graphics Host Memory Management Abstraction
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

#include <linux/kernel.h>
#include <linux/err.h>

#include "nvhost_memmgr.h"
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
#include "nvmap.h"
#endif
#include "chip_support.h"

struct mem_mgr *nvhost_memmgr_alloc_mgr(void)
{
	struct mem_mgr *mgr = NULL;
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	mgr = nvhost_nvmap_alloc_mgr();
#endif

	return mgr;
}

void nvhost_memmgr_put_mgr(struct mem_mgr *mgr)
{
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	nvhost_nvmap_put_mgr(mgr);
#endif
}

struct mem_mgr *nvhost_memmgr_get_mgr(struct mem_mgr *_mgr)
{
	struct mem_mgr *mgr = NULL;
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	mgr = nvhost_nvmap_get_mgr(_mgr);
#endif

	return mgr;
}

struct mem_mgr *nvhost_memmgr_get_mgr_file(int fd)
{
	struct mem_mgr *mgr = NULL;
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	mgr = nvhost_nvmap_get_mgr_file(fd);
#endif

	return mgr;
}

struct mem_handle *nvhost_memmgr_alloc(struct mem_mgr *mgr,
		size_t size, size_t align, int flags)
{
	struct mem_handle *h = NULL;
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	h = nvhost_nvmap_alloc(mgr, size, align, flags);
#endif

	return h;
}

struct mem_handle *nvhost_memmgr_get(struct mem_mgr *mgr,
		u32 id, struct nvhost_device *dev)
{
	struct mem_handle *h = NULL;

	switch (nvhost_memmgr_type(id)) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		h = (struct mem_handle *) nvhost_nvmap_get(mgr, id, dev);
		break;
#endif
	default:
		break;
	}

	return h;
}

void nvhost_memmgr_put(struct mem_mgr *mgr, struct mem_handle *handle)
{
	switch (nvhost_memmgr_type((u32)handle)) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		nvhost_nvmap_put(mgr, handle);
		break;
#endif
	default:
		break;
	}
}

struct sg_table *nvhost_memmgr_pin(struct mem_mgr *mgr,
		struct mem_handle *handle)
{
	switch (nvhost_memmgr_type((u32)handle)) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		return nvhost_nvmap_pin(mgr, handle);
		break;
#endif
	default:
		return 0;
		break;
	}
}

void nvhost_memmgr_unpin(struct mem_mgr *mgr,
		struct mem_handle *handle, struct sg_table *sgt)
{
	switch (nvhost_memmgr_type((u32)handle)) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		nvhost_nvmap_unpin(mgr, handle, sgt);
		break;
#endif
	default:
		break;
	}
}

void *nvhost_memmgr_mmap(struct mem_handle *handle)
{
	switch (nvhost_memmgr_type((u32)handle)) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		return nvhost_nvmap_mmap(handle);
		break;
#endif
	default:
		return 0;
		break;
	}
}

void nvhost_memmgr_munmap(struct mem_handle *handle, void *addr)
{
	switch (nvhost_memmgr_type((u32)handle)) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		nvhost_nvmap_munmap(handle, addr);
		break;
#endif
	default:
		break;
	}
}

static const struct nvhost_mem_ops mem_ops = {
	.alloc_mgr = nvhost_memmgr_alloc_mgr,
	.put_mgr = nvhost_memmgr_put_mgr,
	.get_mgr = nvhost_memmgr_get_mgr,
	.get_mgr_file = nvhost_memmgr_get_mgr_file,
	.alloc = nvhost_memmgr_alloc,
	.get = nvhost_memmgr_get,
	.put = nvhost_memmgr_put,
	.pin = nvhost_memmgr_pin,
	.unpin = nvhost_memmgr_unpin,
	.mmap = nvhost_memmgr_mmap,
	.munmap = nvhost_memmgr_munmap,
};

int nvhost_memmgr_init(struct nvhost_chip_support *chip)
{
	chip->mem = mem_ops;
	return 0;
}
