/*
 * drivers/video/tegra/host/nvhost_memmgr.c
 *
 * Tegra Graphics Host Memory Management Abstraction
 *
 * Copyright (c) 2012-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/bug.h>
#include <linux/platform_device.h>

#include "nvhost_memmgr.h"
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
#include "nvmap.h"
#include <linux/nvmap.h>
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

struct mem_handle *nvhost_memmgr_get(struct mem_mgr *mgr,
		ulong id, struct platform_device *dev)
{
	struct mem_handle *h = ERR_PTR(-EINVAL);

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
	switch (nvhost_memmgr_type((u32)((uintptr_t)handle))) {
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
		struct mem_handle *handle, struct device *dev, int rw_flag)
{
	switch (nvhost_memmgr_type((u32)((uintptr_t)handle))) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		return nvhost_nvmap_pin(mgr, handle, dev, rw_flag);
		break;
#endif
	default:
		return 0;
		break;
	}
}

void nvhost_memmgr_unpin(struct mem_mgr *mgr,
		struct mem_handle *handle, struct device *dev,
		struct sg_table *sgt)
{
	switch (nvhost_memmgr_type((u32)((uintptr_t)handle))) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		nvhost_nvmap_unpin(mgr, handle, dev, sgt);
		break;
#endif
	default:
		break;
	}
}

void *nvhost_memmgr_mmap(struct mem_handle *handle)
{
	switch (nvhost_memmgr_type((u32)((uintptr_t)handle))) {
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
	switch (nvhost_memmgr_type((u32)((uintptr_t)handle))) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		nvhost_nvmap_munmap(handle, addr);
		break;
#endif
	default:
		break;
	}
}

void *nvhost_memmgr_kmap(struct mem_handle *handle, unsigned int pagenum)
{
	switch (nvhost_memmgr_type((u32)((uintptr_t)handle))) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		return nvhost_nvmap_kmap(handle, pagenum);
		break;
#endif
	default:
		return 0;
		break;
	}
}

void nvhost_memmgr_kunmap(struct mem_handle *handle, unsigned int pagenum,
		void *addr)
{
	switch (nvhost_memmgr_type((u32)((uintptr_t)handle))) {
#ifdef CONFIG_TEGRA_GRHOST_USE_NVMAP
	case mem_mgr_type_nvmap:
		nvhost_nvmap_kunmap(handle, pagenum, addr);
		break;
#endif
	default:
		break;
	}
}

int nvhost_memmgr_init(struct nvhost_chip_support *chip)
{
	return 0;
}

