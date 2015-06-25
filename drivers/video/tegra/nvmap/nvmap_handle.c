/*
 * drivers/video/tegra/nvmap/nvmap_handle.c
 *
 * Handle allocation and freeing routines for nvmap
 *
 * Copyright (c) 2009-2015, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/dma-buf.h>
#include <linux/moduleparam.h>
#include <linux/nvmap.h>
#include <linux/tegra-soc.h>

#include <asm/pgtable.h>

#include <trace/events/nvmap.h>

#include "nvmap_priv.h"
#include "nvmap_ioctl.h"

static void add_handle_ref(struct nvmap_client *client,
			   struct nvmap_handle_ref *ref)
{
	struct rb_node **p, *parent = NULL;

	nvmap_ref_lock(client);
	p = &client->handle_refs.rb_node;
	while (*p) {
		struct nvmap_handle_ref *node;
		parent = *p;
		node = rb_entry(parent, struct nvmap_handle_ref, node);
		if (ref->handle > node->handle)
			p = &parent->rb_right;
		else
			p = &parent->rb_left;
	}
	rb_link_node(&ref->node, parent, p);
	rb_insert_color(&ref->node, &client->handle_refs);
	client->handle_count++;
	if (client->handle_count > nvmap_max_handle_count)
		nvmap_max_handle_count = client->handle_count;
	atomic_inc(&ref->handle->share_count);
	nvmap_ref_unlock(client);
}

struct nvmap_handle_ref *nvmap_create_handle(struct nvmap_client *client,
					     size_t size)
{
	void *err = ERR_PTR(-ENOMEM);
	struct nvmap_handle *h;
	struct nvmap_handle_ref *ref = NULL;

	if (!client)
		return ERR_PTR(-EINVAL);

	if (!size)
		return ERR_PTR(-EINVAL);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return ERR_PTR(-ENOMEM);

	ref = kzalloc(sizeof(*ref), GFP_KERNEL);
	if (!ref)
		goto ref_alloc_fail;

	atomic_set(&h->ref, 1);
	atomic_set(&h->pin, 0);
	h->owner = client;
	BUG_ON(!h->owner);
	h->size = h->orig_size = size;
	h->flags = NVMAP_HANDLE_WRITE_COMBINE;
	h->peer = NVMAP_IVM_INVALID_PEER;
	mutex_init(&h->lock);
	INIT_LIST_HEAD(&h->vmas);
	INIT_LIST_HEAD(&h->lru);

	/*
	 * This takes out 1 ref on the dambuf. This corresponds to the
	 * handle_ref that gets automatically made by nvmap_create_handle().
	 */
	h->dmabuf = __nvmap_make_dmabuf(client, h);
	if (IS_ERR(h->dmabuf)) {
		err = h->dmabuf;
		goto make_dmabuf_fail;
	}

	/*
	 * Pre-attach nvmap to this new dmabuf. This gets unattached during the
	 * dma_buf_release() operation.
	 */
	h->attachment = dma_buf_attach(h->dmabuf, nvmap_dev->dev_user.parent);
	if (IS_ERR(h->attachment)) {
		err = h->attachment;
		goto dma_buf_attach_fail;
	}

	nvmap_handle_add(nvmap_dev, h);

	/*
	 * Major assumption here: the dma_buf object that the handle contains
	 * is created with a ref count of 1.
	 */
	atomic_set(&ref->dupes, 1);
	ref->handle = h;
	atomic_set(&ref->pin, 0);
	add_handle_ref(client, ref);
	trace_nvmap_create_handle(client, client->name, h, size, ref);
	return ref;

dma_buf_attach_fail:
	dma_buf_put(h->dmabuf);
make_dmabuf_fail:
	kfree(ref);
ref_alloc_fail:
	kfree(h);
	return err;
}

struct nvmap_handle_ref *nvmap_duplicate_handle(struct nvmap_client *client,
					struct nvmap_handle *h, bool skip_val)
{
	struct nvmap_handle_ref *ref = NULL;

	BUG_ON(!client);
	/* on success, the reference count for the handle should be
	 * incremented, so the success paths will not call nvmap_handle_put */
	h = nvmap_validate_get(h);

	if (!h) {
		pr_debug("%s duplicate handle failed\n",
			    current->group_leader->comm);
		return ERR_PTR(-EPERM);
	}

	if (!h->alloc) {
		pr_err("%s duplicating unallocated handle\n",
			current->group_leader->comm);
		nvmap_handle_put(h);
		return ERR_PTR(-EINVAL);
	}

	nvmap_ref_lock(client);
	ref = __nvmap_validate_locked(client, h);

	if (ref) {
		/* handle already duplicated in client; just increment
		 * the reference count rather than re-duplicating it */
		atomic_inc(&ref->dupes);
		nvmap_ref_unlock(client);
		return ref;
	}

	nvmap_ref_unlock(client);

	ref = kzalloc(sizeof(*ref), GFP_KERNEL);
	if (!ref) {
		nvmap_handle_put(h);
		return ERR_PTR(-ENOMEM);
	}

	atomic_set(&ref->dupes, 1);
	ref->handle = h;
	atomic_set(&ref->pin, 0);
	add_handle_ref(client, ref);

	/*
	 * Ref counting on the dma_bufs follows the creation and destruction of
	 * nvmap_handle_refs. That is every time a handle_ref is made the
	 * dma_buf ref count goes up and everytime a handle_ref is destroyed
	 * the dma_buf ref count goes down.
	 */
	get_dma_buf(h->dmabuf);

	trace_nvmap_duplicate_handle(client, h, ref);
	return ref;
}

struct nvmap_handle_ref *nvmap_create_handle_from_fd(
			struct nvmap_client *client, int fd)
{
	struct nvmap_handle *handle;
	struct nvmap_handle_ref *ref;

	BUG_ON(!client);

	handle = nvmap_handle_get_from_dmabuf_fd(client, fd);
	if (IS_ERR(handle))
		return ERR_CAST(handle);
	ref = nvmap_duplicate_handle(client, handle, 1);
	nvmap_handle_put(handle);
	return ref;
}

int __nvmap_get_handle_param(struct nvmap_client *client,
			     struct nvmap_handle *h, u32 param, u64 *result)
{
	int err = 0;

	if (WARN_ON(!virt_addr_valid(h)))
		return -EINVAL;

	switch (param) {
	case NVMAP_HANDLE_PARAM_SIZE:
		*result = h->orig_size;
		break;
	case NVMAP_HANDLE_PARAM_ALIGNMENT:
		*result = h->align;
		break;
	case NVMAP_HANDLE_PARAM_BASE:
		if (!h->alloc || !atomic_read(&h->pin))
			*result = -EINVAL;
		else if (!h->heap_pgalloc) {
			mutex_lock(&h->lock);
			*result = h->carveout->base;
			mutex_unlock(&h->lock);
		} else if (h->attachment->priv)
			*result = sg_dma_address(
				((struct sg_table *)h->attachment->priv)->sgl);
		else
			*result = -EINVAL;
		break;
	case NVMAP_HANDLE_PARAM_HEAP:
		if (!h->alloc)
			*result = 0;
		else if (!h->heap_pgalloc) {
			mutex_lock(&h->lock);
			*result = nvmap_carveout_usage(client, h->carveout);
			mutex_unlock(&h->lock);
		} else
			*result = NVMAP_HEAP_IOVMM;
		break;
	case NVMAP_HANDLE_PARAM_KIND:
		*result = h->kind;
		break;
	case NVMAP_HANDLE_PARAM_COMPR:
		/* ignored, to be removed */
		break;
	default:
		err = -EINVAL;
		break;
	}
	return err;
}

int nvmap_get_handle_param(struct nvmap_client *client,
			   struct nvmap_handle_ref *ref, u32 param, u64 *result)
{
	if (WARN_ON(!virt_addr_valid(ref)) ||
	    WARN_ON(!virt_addr_valid(client)) ||
	    WARN_ON(!result))
		return -EINVAL;

	return __nvmap_get_handle_param(client, ref->handle, param, result);
}
