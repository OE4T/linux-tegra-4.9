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

#include <linux/dma-buf.h>

#include <linux/platform/tegra/tegra_fd.h>

#include <nvgpu/dma.h>
#include <nvgpu/enabled.h>
#include <nvgpu/vidmem.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/page_allocator.h>

#include <nvgpu/linux/dma.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"

#include "vm_priv.h"

void set_vidmem_page_alloc(struct scatterlist *sgl, u64 addr)
{
	/* set bit 0 to indicate vidmem allocation */
	sg_dma_address(sgl) = (addr | 1ULL);
}

bool is_vidmem_page_alloc(u64 addr)
{
	return !!(addr & 1ULL);
}

struct nvgpu_page_alloc *get_vidmem_page_alloc(struct scatterlist *sgl)
{
	u64 addr;

	addr = sg_dma_address(sgl);

	if (is_vidmem_page_alloc(addr))
		addr = addr & ~1ULL;
	else
		WARN_ON(1);

	return (struct nvgpu_page_alloc *)(uintptr_t)addr;
}

static struct sg_table *gk20a_vidbuf_map_dma_buf(
	struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	struct gk20a_vidmem_buf *buf = attach->dmabuf->priv;

	return buf->mem->priv.sgt;
}

static void gk20a_vidbuf_unmap_dma_buf(struct dma_buf_attachment *attach,
				       struct sg_table *sgt,
				       enum dma_data_direction dir)
{
}

static void gk20a_vidbuf_release(struct dma_buf *dmabuf)
{
	struct gk20a_vidmem_buf *buf = dmabuf->priv;

	gk20a_dbg_fn("");

	if (buf->dmabuf_priv)
		buf->dmabuf_priv_delete(buf->dmabuf_priv);

	nvgpu_dma_free(buf->g, buf->mem);
	nvgpu_kfree(buf->g, buf);
}

static void *gk20a_vidbuf_kmap(struct dma_buf *dmabuf, unsigned long page_num)
{
	WARN_ON("Not supported");
	return NULL;
}

static void *gk20a_vidbuf_kmap_atomic(struct dma_buf *dmabuf,
				      unsigned long page_num)
{
	WARN_ON("Not supported");
	return NULL;
}

static int gk20a_vidbuf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static int gk20a_vidbuf_set_private(struct dma_buf *dmabuf,
		struct device *dev, void *priv, void (*delete)(void *priv))
{
	struct gk20a_vidmem_buf *buf = dmabuf->priv;

	buf->dmabuf_priv = priv;
	buf->dmabuf_priv_delete = delete;

	return 0;
}

static void *gk20a_vidbuf_get_private(struct dma_buf *dmabuf,
		struct device *dev)
{
	struct gk20a_vidmem_buf *buf = dmabuf->priv;

	return buf->dmabuf_priv;
}

static const struct dma_buf_ops gk20a_vidbuf_ops = {
	.map_dma_buf      = gk20a_vidbuf_map_dma_buf,
	.unmap_dma_buf    = gk20a_vidbuf_unmap_dma_buf,
	.release          = gk20a_vidbuf_release,
	.kmap_atomic      = gk20a_vidbuf_kmap_atomic,
	.kmap             = gk20a_vidbuf_kmap,
	.mmap             = gk20a_vidbuf_mmap,
	.set_drvdata      = gk20a_vidbuf_set_private,
	.get_drvdata      = gk20a_vidbuf_get_private,
};

static struct dma_buf *gk20a_vidbuf_export(struct gk20a_vidmem_buf *buf)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.priv = buf;
	exp_info.ops = &gk20a_vidbuf_ops;
	exp_info.size = buf->mem->size;
	exp_info.flags = O_RDWR;

	return dma_buf_export(&exp_info);
}

struct gk20a *gk20a_vidmem_buf_owner(struct dma_buf *dmabuf)
{
	struct gk20a_vidmem_buf *buf = dmabuf->priv;

	if (dmabuf->ops != &gk20a_vidbuf_ops)
		return NULL;

	return buf->g;
}

int gk20a_vidmem_buf_alloc(struct gk20a *g, size_t bytes)
{
	struct gk20a_vidmem_buf *buf;
	int err = 0, fd;

	gk20a_dbg_fn("");

	buf = nvgpu_kzalloc(g, sizeof(*buf));
	if (!buf)
		return -ENOMEM;

	buf->g = g;

	if (!g->mm.vidmem.cleared) {
		nvgpu_mutex_acquire(&g->mm.vidmem.first_clear_mutex);
		if (!g->mm.vidmem.cleared) {
			err = gk20a_vidmem_clear_all(g);
			if (err) {
				nvgpu_err(g,
				          "failed to clear whole vidmem");
				goto err_kfree;
			}
		}
		nvgpu_mutex_release(&g->mm.vidmem.first_clear_mutex);
	}

	buf->mem = nvgpu_kzalloc(g, sizeof(struct nvgpu_mem));
	if (!buf->mem)
		goto err_kfree;

	buf->mem->mem_flags |= NVGPU_MEM_FLAG_USER_MEM;

	err = nvgpu_dma_alloc_vid(g, bytes, buf->mem);
	if (err)
		goto err_memfree;

	buf->dmabuf = gk20a_vidbuf_export(buf);
	if (IS_ERR(buf->dmabuf)) {
		err = PTR_ERR(buf->dmabuf);
		goto err_bfree;
	}

	fd = tegra_alloc_fd(current->files, 1024, O_RDWR);
	if (fd < 0) {
		/* ->release frees what we have done */
		dma_buf_put(buf->dmabuf);
		return fd;
	}

	/* fclose() on this drops one ref, freeing the dma buf */
	fd_install(fd, buf->dmabuf->file);

	return fd;

err_bfree:
	nvgpu_dma_free(g, buf->mem);
err_memfree:
	nvgpu_kfree(g, buf->mem);
err_kfree:
	nvgpu_kfree(g, buf);
	return err;
}

int gk20a_vidbuf_access_memory(struct gk20a *g, struct dma_buf *dmabuf,
		void *buffer, u64 offset, u64 size, u32 cmd)
{
	struct gk20a_vidmem_buf *vidmem_buf;
	struct nvgpu_mem *mem;
	int err = 0;

	if (gk20a_dmabuf_aperture(g, dmabuf) != APERTURE_VIDMEM)
		return -EINVAL;

	vidmem_buf = dmabuf->priv;
	mem = vidmem_buf->mem;

	switch (cmd) {
	case NVGPU_DBG_GPU_IOCTL_ACCESS_FB_MEMORY_CMD_READ:
		nvgpu_mem_rd_n(g, mem, offset, buffer, size);
		break;

	case NVGPU_DBG_GPU_IOCTL_ACCESS_FB_MEMORY_CMD_WRITE:
		nvgpu_mem_wr_n(g, mem, offset, buffer, size);
		break;

	default:
		err = -EINVAL;
	}

	return err;
}

void gk20a_vidmem_clear_mem_worker(struct work_struct *work)
{
	struct mm_gk20a *mm = container_of(work, struct mm_gk20a,
					vidmem.clear_mem_worker);
	struct gk20a *g = mm->g;
	struct nvgpu_mem *mem;

	while ((mem = get_pending_mem_desc(mm)) != NULL) {
		gk20a_gmmu_clear_vidmem_mem(g, mem);
		nvgpu_free(mem->allocator,
			   (u64)get_vidmem_page_alloc(mem->priv.sgt->sgl));
		nvgpu_free_sgtable(g, &mem->priv.sgt);

		WARN_ON(nvgpu_atomic64_sub_return(mem->aligned_size,
					&g->mm.vidmem.bytes_pending) < 0);
		mem->size = 0;
		mem->aperture = APERTURE_INVALID;

		nvgpu_kfree(g, mem);
	}
}
