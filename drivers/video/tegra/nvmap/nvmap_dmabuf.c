/*
 * dma_buf exporter for nvmap
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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
	struct nvmap_client *client;
	ulong id;
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *handle;
};

static int nvmap_dmabuf_attach(struct dma_buf *dmabuf, struct device *dev,
			       struct dma_buf_attachment *attach)
{
	struct nvmap_handle_info *info = dmabuf->priv;
	struct nvmap_handle_ref *ref;

	ref = nvmap_duplicate_handle_id(info->client, info->id, 0);
	if (IS_ERR(ref))
		return PTR_ERR(ref);

	info->ref = ref;
	attach->priv = info;

	dev_dbg(dev, "%s(%08lx)\n", __func__, info->id);
	return 0;
}

static void nvmap_dmabuf_detach(struct dma_buf *dmabuf,
				struct dma_buf_attachment *attach)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	nvmap_free(info->client, info->ref);

	dev_dbg(attach->dev, "%s(%08lx)\n", __func__, info->id);
}

static struct sg_table *nvmap_dmabuf_map_dma_buf(
	struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = attach->dmabuf->priv;
	int err;
	struct sg_table *sgt;
	dma_addr_t addr;

	sgt = nvmap_sg_table(info->client, info->ref);
	if (IS_ERR(sgt))
		return sgt;

	err = nvmap_pin(info->client, info->ref, &addr);
	if (err)
		goto err_pin;

	sg_dma_address(sgt->sgl) = addr;

	dev_dbg(attach->dev, "%s(%08lx)\n", __func__, info->id);
	return sgt;

err_pin:
	nvmap_free_sg_table(info->client, info->ref, sgt);
	return ERR_PTR(err);
}

static void nvmap_dmabuf_unmap_dma_buf(struct dma_buf_attachment *attach,
				       struct sg_table *sgt,
				       enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = attach->dmabuf->priv;

	nvmap_unpin(info->client, info->ref);
	nvmap_free_sg_table(info->client, info->ref, sgt);

	dev_dbg(attach->dev, "%s(%08lx)\n", __func__, info->id);
}

static void nvmap_dmabuf_release(struct dma_buf *dmabuf)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s(%08lx)\n", __func__, info->id);

	nvmap_handle_put(info->handle);
	nvmap_client_put(info->client);
	kfree(info);
}

static void *nvmap_dmabuf_kmap(struct dma_buf *dmabuf, unsigned long page_num)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s(%08lx)\n", __func__, info->id);
	return nvmap_kmap(info->ref, page_num);
}

static void nvmap_dmabuf_kunmap(struct dma_buf *dmabuf,
		unsigned long page_num, void *addr)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s(%08lx)\n", __func__, info->id);
	nvmap_kunmap(info->ref, page_num, addr);
}

static void *nvmap_dmabuf_kmap_atomic(struct dma_buf *dmabuf,
				      unsigned long page_num)
{
	WARN(1, "%s() can't be called from atomic\n", __func__);
	return NULL;
}

static int nvmap_dmabuf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	WARN(1, "%s() not implemented yet\n", __func__);
	return -1;
}

static void *nvmap_dmabuf_vmap(struct dma_buf *dmabuf)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s(%08lx)\n", __func__, info->id);
	return nvmap_mmap(info->ref);
}

static void nvmap_dmabuf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	pr_debug("%s(%08lx)\n", __func__, info->id);
	nvmap_munmap(info->ref, vaddr);
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

struct dma_buf *nvmap_share_dmabuf(struct nvmap_client *client, ulong id)
{
	struct dma_buf *dmabuf;
	struct nvmap_handle_info *info;
	int err;
	struct nvmap_handle *handle;

	if (!nvmap_client_get(client))
		return ERR_PTR(-EINVAL);

	handle = nvmap_validate_get(client, id, 0);
	if (!handle) {
		err = -EINVAL;
		goto err_nvmap_validate_get;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto err_nomem;
	}
	info->id = id;
	info->handle = handle;
	info->client = client;

	dmabuf = dma_buf_export(info, &nvmap_dma_buf_ops, handle->size,
				O_RDWR);
	if (IS_ERR(dmabuf)) {
		err = PTR_ERR(dmabuf);
		goto err_export;
	}
	pr_debug("%s(%08lx) %p\n", __func__, info->id, dmabuf);
	return dmabuf;

err_export:
	kfree(info);
err_nomem:
	nvmap_handle_put(handle);
err_nvmap_validate_get:
	nvmap_client_put(client);
	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(nvmap_share_dmabuf);

int nvmap_ioctl_share_dmabuf(struct file *filp, void __user *arg)
{
	int err;
	struct nvmap_create_handle op;
	struct nvmap_client *client = filp->private_data;
	struct dma_buf *dmabuf;
	ulong handle;

	BUG_ON(!client);

	if (copy_from_user(&op, (void __user *)arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_id(op.id);
	if (!handle)
		return -EINVAL;

	dmabuf = nvmap_share_dmabuf(client, handle);
	if (IS_ERR(dmabuf))
		return -EINVAL;

	op.fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	if (op.fd < 0) {
		err = op.fd;
		goto err_out;
	}

	if (copy_to_user((void __user *)arg, &op, sizeof(op))) {
		err = -EFAULT;
		goto err_out;
	}
	return 0;

err_out:
	nvmap_dmabuf_release(dmabuf);
	return err;
}
