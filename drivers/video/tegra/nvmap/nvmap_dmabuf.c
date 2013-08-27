/*
 * dma_buf exporter for nvmap
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/export.h>
#include <linux/nvmap.h>
#include <linux/dma-buf.h>

#include "nvmap_priv.h"
#include "nvmap_ioctl.h"

struct nvmap_handle_info {
	struct nvmap_handle *handle;
};

static int nvmap_dmabuf_attach(struct dma_buf *dmabuf, struct device *dev,
			       struct dma_buf_attachment *attach)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	dev_dbg(dev, "%s() 0x%p\n", __func__, info->handle);
	return 0;
}

static void nvmap_dmabuf_detach(struct dma_buf *dmabuf,
				struct dma_buf_attachment *attach)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	dev_dbg(attach->dev, "%s() 0x%p\n", __func__, info->handle);
}

static struct sg_table *nvmap_dmabuf_map_dma_buf(
	struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = attach->dmabuf->priv;
	int err;
	struct sg_table *sgt;
	dma_addr_t addr;

	sgt = __nvmap_sg_table(NULL, info->handle);
	if (IS_ERR(sgt))
		return sgt;

	err = __nvmap_pin(NULL, info->handle, &addr);
	if (err)
		goto err_pin;

	sg_dma_address(sgt->sgl) = addr;
	sg_dma_len(sgt->sgl) = info->handle->size;

	dev_dbg(attach->dev, "%s() 0x%p\n", __func__, info->handle);
	return sgt;

err_pin:
	__nvmap_free_sg_table(NULL, info->handle, sgt);
	return ERR_PTR(err);
}

static void nvmap_dmabuf_unmap_dma_buf(struct dma_buf_attachment *attach,
				       struct sg_table *sgt,
				       enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = attach->dmabuf->priv;

	__nvmap_unpin(NULL, info->handle);
	__nvmap_free_sg_table(NULL, info->handle, sgt);

	dev_dbg(attach->dev, "%s() 0x%p\n", __func__, info->handle);
}

static void nvmap_dmabuf_release(struct dma_buf *dmabuf)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s() 0x%p\n", __func__, info->handle);

	nvmap_handle_put(info->handle);
	info->handle->dmabuf = NULL;
	kfree(info);
}

static void *nvmap_dmabuf_kmap(struct dma_buf *dmabuf, unsigned long page_num)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s() 0x%p\n", __func__, info->handle);
	return __nvmap_kmap(info->handle, page_num);
}

static void nvmap_dmabuf_kunmap(struct dma_buf *dmabuf,
		unsigned long page_num, void *addr)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s() 0x%p\n", __func__, info->handle);
	__nvmap_kunmap(info->handle, page_num, addr);
}

static void *nvmap_dmabuf_kmap_atomic(struct dma_buf *dmabuf,
				      unsigned long page_num)
{
	WARN(1, "%s() can't be called from atomic\n", __func__);
	return NULL;
}

static int nvmap_dmabuf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	return __nvmap_map(info->handle, vma);
}

static void *nvmap_dmabuf_vmap(struct dma_buf *dmabuf)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s() 0x%p\n", __func__, info->handle);
	return __nvmap_mmap(info->handle);
}

static void nvmap_dmabuf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s() 0x%p\n", __func__, info->handle);
	__nvmap_munmap(info->handle, vaddr);
}

static struct dma_buf_ops nvmap_dma_buf_ops = {
	.attach		= nvmap_dmabuf_attach,
	.detach		= nvmap_dmabuf_detach,
	.map_dma_buf	= nvmap_dmabuf_map_dma_buf,
	.unmap_dma_buf	= nvmap_dmabuf_unmap_dma_buf,
	.release	= nvmap_dmabuf_release,
	.kmap_atomic	= nvmap_dmabuf_kmap_atomic,
	.kmap		= nvmap_dmabuf_kmap,
	.kunmap		= nvmap_dmabuf_kunmap,
	.mmap		= nvmap_dmabuf_mmap,
	.vmap		= nvmap_dmabuf_vmap,
	.vunmap		= nvmap_dmabuf_vunmap,
};

/*
 * Make a dmabuf object for an nvmap handle.
 */
struct dma_buf *__nvmap_make_dmabuf(struct nvmap_client *client,
				    struct nvmap_handle *handle)
{
	int err;
	struct dma_buf *dmabuf;
	struct nvmap_handle_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto err_nomem;
	}
	info->handle = handle;

	dmabuf = dma_buf_export(info, &nvmap_dma_buf_ops, handle->size,
				O_RDWR);
	if (IS_ERR(dmabuf)) {
		err = PTR_ERR(dmabuf);
		goto err_export;
	}
	nvmap_handle_get(handle);
	pr_debug("%s() 0x%p => 0x%p\n", __func__, info->handle, dmabuf);
	return dmabuf;

err_export:
	kfree(info);
err_nomem:
	return ERR_PTR(err);
}

int nvmap_get_dmabuf_fd(struct nvmap_client *client, ulong id)
{
	int fd;
	struct dma_buf *dmabuf;

	dmabuf = __nvmap_dmabuf_export(client, id);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);
	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	if (fd < 0)
		goto err_out;
	return fd;

err_out:
	dma_buf_put(dmabuf);
	return fd;
}

struct dma_buf *__nvmap_dmabuf_export(struct nvmap_client *client,
				 unsigned long id)
{
	struct nvmap_handle *handle;
	struct dma_buf *buf;

	handle = nvmap_validate_get(client, id, 0);
	if (!handle)
		return ERR_PTR(-EINVAL);
	buf = handle->dmabuf;
	if (WARN(!buf, "Attempting to get a freed dma_buf!\n")) {
		nvmap_handle_put(handle);
		return NULL;
	}

	get_dma_buf(buf);

	/*
	 * Don't want to take out refs on the handle here.
	 */
	nvmap_handle_put(handle);

	return handle->dmabuf;
}

/*
 * Increments ref count on the dma_buf. You are reponsbile for calling
 * dma_buf_put() on the returned dma_buf object.
 */
struct dma_buf *nvmap_dmabuf_export(struct nvmap_client *client,
				 unsigned long user_id)
{
	return __nvmap_dmabuf_export(client, unmarshal_user_id(user_id));
}

/*
 * Similar to nvmap_dmabuf_export() only use a ref to get the buf instead of a
 * user_id. You must dma_buf_put() the dma_buf object when you are done with
 * it.
 */
struct dma_buf *nvmap_dmabuf_export_from_ref(struct nvmap_handle_ref *ref)
{
	if (!virt_addr_valid(ref))
		return ERR_PTR(-EINVAL);

	get_dma_buf(ref->handle->dmabuf);
	return ref->handle->dmabuf;
}

/*
 * Returns the nvmap handle ID associated with the passed dma_buf's fd. This
 * does not affect the ref count of the dma_buf.
 */
ulong nvmap_get_id_from_dmabuf_fd(struct nvmap_client *client, int fd)
{
	ulong id = -EINVAL;
	struct dma_buf *dmabuf;
	struct nvmap_handle_info *info;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);
	if (dmabuf->ops == &nvmap_dma_buf_ops) {
		info = dmabuf->priv;
		id = (ulong) info->handle;
	}
	dma_buf_put(dmabuf);
	return id;
}

int nvmap_ioctl_share_dmabuf(struct file *filp, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_client *client = filp->private_data;
	ulong handle;

	BUG_ON(!client);

	if (copy_from_user(&op, (void __user *)arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_id(op.id);
	if (!handle)
		return -EINVAL;

	op.fd = nvmap_get_dmabuf_fd(client, handle);
	if (op.fd < 0)
		return op.fd;

	if (copy_to_user((void __user *)arg, &op, sizeof(op))) {
		sys_close(op.fd);
		return -EFAULT;
	}
	return 0;
}

int nvmap_get_dmabuf_param(struct dma_buf *dmabuf, u32 param, u64 *result)
{
	struct nvmap_handle_info *info;

	if (WARN_ON(!virt_addr_valid(dmabuf)))
		return -EINVAL;

	info = dmabuf->priv;
	return __nvmap_get_handle_param(NULL, info->handle, param, result);
}
