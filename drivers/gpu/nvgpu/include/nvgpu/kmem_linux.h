/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVGPU_KMEM_LINUX_H__
#define __NVGPU_KMEM_LINUX_H__

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <asm/page.h>

struct gk20a;
struct device;

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
void *__nvgpu_track_vmalloc(struct gk20a *g, unsigned long size,
			    unsigned long ip);
void *__nvgpu_track_vzalloc(struct gk20a *g, unsigned long size,
			    unsigned long ip);
void *__nvgpu_track_kmalloc(struct gk20a *g, size_t size, unsigned long ip);
void *__nvgpu_track_kzalloc(struct gk20a *g, size_t size, unsigned long ip);
void *__nvgpu_track_kcalloc(struct gk20a *g, size_t n, size_t size,
			    unsigned long ip);
void  __nvgpu_track_vfree(struct gk20a *g, void *addr);
void  __nvgpu_track_kfree(struct gk20a *g, void *addr);

void nvgpu_kmem_debugfs_init(struct device *dev);
#else
static inline void nvgpu_kmem_debugfs_init(struct device *dev)
{
}
#endif

/**
 * DOC: Linux pass through kmem implementation.
 *
 * These are the Linux implementations of the various kmem functions defined by
 * nvgpu. This should not be included directly - instead include <nvgpu/kmem.h>.
 */

static inline void *__nvgpu_kmalloc(struct gk20a *g, unsigned long size,
				    unsigned long ip)
{
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	return __nvgpu_track_vmalloc(g, size, ip);
#else
	return kmalloc(size, GFP_KERNEL);
#endif
}

static inline void *__nvgpu_kzalloc(struct gk20a *g, size_t size,
				    unsigned long ip)
{
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	return __nvgpu_track_kzalloc(g, size, ip);
#else
	return kzalloc(size, GFP_KERNEL);
#endif
}

static inline void *__nvgpu_kcalloc(struct gk20a *g, size_t n, size_t size,
				    unsigned long ip)
{
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	return __nvgpu_track_kcalloc(g, n, size, ip);
#else
	return kcalloc(n, size, GFP_KERNEL);
#endif
}

static inline void *__nvgpu_vmalloc(struct gk20a *g, unsigned long size,
				    unsigned long ip)
{
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	return __nvgpu_track_vmalloc(g, size, ip);
#else
	return vmalloc(size);
#endif
}

static inline void *__nvgpu_vzalloc(struct gk20a *g, unsigned long size,
				    unsigned long ip)
{
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	return __nvgpu_track_vzalloc(g, size, ip);
#else
	return vzalloc(size);
#endif
}

static inline void __nvgpu_kfree(struct gk20a *g, void *addr)
{
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	__nvgpu_track_kfree(g, addr);
#else
	kfree(addr);
#endif
}

static inline void __nvgpu_vfree(struct gk20a *g, void *addr)
{
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	__nvgpu_track_vfree(g, addr);
#else
	vfree(addr);
#endif
}

#endif
