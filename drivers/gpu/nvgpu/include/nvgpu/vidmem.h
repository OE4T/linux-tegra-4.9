/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __NVGPU_VIDMEM_H__
#define __NVGPU_VIDMEM_H__

#include <nvgpu/types.h>
#include <nvgpu/errno.h>

struct scatterlist;
struct dma_buf;
struct work_struct;

struct gk20a;
struct mm_gk20a;
struct nvgpu_mem;

struct nvgpu_vidmem_buf {
	struct gk20a *g;
	struct nvgpu_mem *mem;
	struct dma_buf *dmabuf;
	void *dmabuf_priv;
	void (*dmabuf_priv_delete)(void *);
};

#if defined(CONFIG_GK20A_VIDMEM)

struct nvgpu_page_alloc *nvgpu_vidmem_get_page_alloc(struct scatterlist *sgl);
void nvgpu_vidmem_set_page_alloc(struct scatterlist *sgl, u64 addr);
bool nvgpu_addr_is_vidmem_page_alloc(u64 addr);
int nvgpu_vidmem_buf_alloc(struct gk20a *g, size_t bytes);
int nvgpu_vidmem_get_space(struct gk20a *g, u64 *space);

struct nvgpu_mem *nvgpu_vidmem_get_pending_alloc(struct mm_gk20a *mm);

void nvgpu_vidmem_destroy(struct gk20a *g);
int nvgpu_vidmem_init(struct mm_gk20a *mm);
int nvgpu_vidmem_clear_all(struct gk20a *g);

void nvgpu_vidmem_clear_mem_worker(struct work_struct *work);
int nvgpu_vidmem_clear(struct gk20a *g, struct nvgpu_mem *mem);

/*
 * Will need to be moved later on once we have the Linux vidmem.h file.
 */
struct gk20a *nvgpu_vidmem_buf_owner(struct dma_buf *dmabuf);
int nvgpu_vidmem_buf_access_memory(struct gk20a *g, struct dma_buf *dmabuf,
		void *buffer, u64 offset, u64 size, u32 cmd);

#else /* !defined(CONFIG_GK20A_VIDMEM) */

/*
 * When VIDMEM support is not present this interface is used.
 */

static inline struct nvgpu_page_alloc *
nvgpu_vidmem_get_page_alloc(struct scatterlist *sgl)
{
	return NULL;
}

static inline void nvgpu_vidmem_set_page_alloc(struct scatterlist *sgl, u64 addr)
{
}

static inline bool nvgpu_addr_is_vidmem_page_alloc(u64 addr)
{
	return false;
}

static inline int nvgpu_vidmem_buf_alloc(struct gk20a *g, size_t bytes)
{
	return -ENOSYS;
}
static inline int nvgpu_vidmem_get_space(struct gk20a *g, u64 *space)
{
	return -ENOSYS;
}

static inline struct nvgpu_mem *nvgpu_vidmem_get_pending_alloc(struct mm_gk20a *mm)
{
	return NULL;
}

static inline void nvgpu_vidmem_destroy(struct gk20a *g)
{
}

static inline int nvgpu_vidmem_init(struct mm_gk20a *mm)
{
	return 0;
}

static inline int nvgpu_vidmem_clear_all(struct gk20a *g)
{
	return -ENOSYS;
}

static inline int nvgpu_vidmem_clear(struct gk20a *g,
					      struct nvgpu_mem *mem)
{
	return -ENOSYS;
}

static inline struct gk20a *nvgpu_vidmem_buf_owner(struct dma_buf *dmabuf)
{
	return NULL;
}

static inline int nvgpu_vidmem_buf_access_memory(struct gk20a *g,
					     struct dma_buf *dmabuf,
					     void *buffer, u64 offset,
					     u64 size, u32 cmd)
{
	return -ENOSYS;
}

#endif /* !defined(CONFIG_GK20A_VIDMEM) */

#endif /* __NVGPU_VIDMEM_H__ */
