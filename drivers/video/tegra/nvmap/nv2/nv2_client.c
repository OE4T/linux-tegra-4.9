/*
 * drivers/video/tegra/nvmap/nvmap_handle.c
 *
 * Handle allocation and freeing routines for nvmap
 *
 * Copyright (c) 2009-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/dma-buf.h>
#include <linux/platform/tegra/tegra_fd.h>
#include <linux/moduleparam.h>
#include <linux/nvmap.h>
#include <soc/tegra/chip-id.h>

#include <asm/pgtable.h>

#include <trace/events/nvmap.h>

#include "nvmap_priv.h"
#include "nvmap_ioctl.h"

extern bool dmabuf_is_nvmap(struct dma_buf *dmabuf);

int NVMAP2_client_add_handle(struct nvmap_client *client,
			   struct nvmap_handle *handle)
{
	struct nvmap_handle_ref *ref;

	ref = NVMAP2_client_to_handle_ref(client, handle);
	if (ref) {
		NVMAP2_handle_ref_get(ref);
		return 0;
	}

	ref = NVMAP2_handle_ref_create(handle);
	if (!ref) {
		return -ENOMEM;
	}

	NVMAP2_client_add_ref(client, ref);
	NVMAP2_handle_add_owner(handle, client);

	return 0;
}

void NVMAP2_client_remove_handle(struct nvmap_client *client,
			   struct nvmap_handle *handle)
{
	struct nvmap_handle_ref *ref;
	int ref_count;

	ref = NVMAP2_client_to_handle_ref(client, handle);
	if (!ref)
		return;

	ref_count = NVMAP2_handle_ref_put(ref);
	if (ref_count == 0) {
		NVMAP2_client_remove_ref(client, ref);
		NVMAP2_handle_ref_free(ref);
	}
}

int NVMAP2_client_create_handle(struct nvmap_client *client, size_t size)
{
	struct nvmap_handle *handle = NULL;
	int err;
	int fd;

	handle = NVMAP2_handle_create(size);
	if (IS_ERR_OR_NULL(handle)) {
		return -1;
	}

	err = NVMAP2_client_add_handle(client, handle);
	if (err) {
		NVMAP2_handle_put(handle);
		return -1;
	}
	/* This is the first handle ref we are creating and we want the dmabuf
	 * to have a ref of 1.
	 * client_add_handle increases the dmabuf ref so decrease it again
	 */
	dma_buf_put(handle->dmabuf);

	fd = NVMAP2_client_create_fd(client);
	if (fd < 0) {
		NVMAP2_client_remove_handle(client, handle);
		NVMAP2_handle_put(handle);
		return -1;
	}
	NVMAP2_handle_install_fd(handle, fd);

	return fd;
}

void NVMAP2_client_add_ref(struct nvmap_client *client,
			   struct nvmap_handle_ref *ref)
{
	struct rb_node **p, *parent = NULL;

	if(IS_ERR(ref)) {
		pr_warn("Putting Error Ref into client\n");
		return;
	}

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

void NVMAP2_client_remove_ref(struct nvmap_client *client,
					struct nvmap_handle_ref *ref)
{
	nvmap_ref_lock(client);

	smp_rmb();
	rb_erase(&ref->node, &client->handle_refs);
	client->handle_count--;
	atomic_dec(&ref->handle->share_count);

	nvmap_ref_unlock(client);
}

struct nvmap_handle_ref *NVMAP2_client_to_handle_ref(struct nvmap_client *client,
					struct nvmap_handle *handle)
{
	struct rb_node *n = client->handle_refs.rb_node;
	struct nvmap_handle_ref *return_ref = NULL;

	nvmap_ref_lock(client);

	n = client->handle_refs.rb_node;
	while (n) {
		struct nvmap_handle_ref *ref;
		ref = rb_entry(n, struct nvmap_handle_ref, node);
		if (IS_ERR(ref)) {
			pr_warn("Ref error in client!\n");
			nvmap_ref_unlock(client);
			return NULL;
		}
		if (ref->handle == handle) {
			return_ref = ref;
			break;
		}
		else if ((uintptr_t)handle > (uintptr_t)ref->handle)
			n = n->rb_right;
		else
			n = n->rb_left;
	}

	nvmap_ref_unlock(client);
	return return_ref;

}

int NVMAP2_client_create_fd(struct nvmap_client *client)
{
	int flags = O_CLOEXEC;
	int start_fd = CONFIG_NVMAP_FD_START;

#ifdef CONFIG_NVMAP_DEFER_FD_RECYCLE
	if (client->next_fd < CONFIG_NVMAP_FD_START)
		client->next_fd = CONFIG_NVMAP_FD_START;
	start_fd = client->next_fd++;
	if (client->next_fd >= CONFIG_NVMAP_DEFER_FD_RECYCLE_MAX_FD)
		client->next_fd = CONFIG_NVMAP_FD_START;
#endif
	/* Allocate fd from start_fd(>=1024) onwards to overcome
	 * __FD_SETSIZE limitation issue for select(),
	 * pselect() syscalls.
	 */
	// TODO: What is this current?
	return tegra_alloc_fd(current->files, start_fd, flags);
}

int NVMAP2_client_give_dmabuf_new_fd(struct nvmap_client *client,
				struct dma_buf *dmabuf)
{
	int fd;

	fd = NVMAP2_client_create_fd(client);
	if (fd > 0)
		NVMAP2_dmabuf_install_fd(dmabuf, fd);
	return fd;
}

void NVMAP2_client_warn_if_no_tag(struct nvmap_client *client,
					unsigned int flags)
{
	int tag = flags >> 16;
	char task_comm[TASK_COMM_LEN];

	if (!tag && client && !client->tag_warned) {
		client->tag_warned = 1;
		get_task_comm(task_comm, client->task);
		pr_err("PID %d: %s: WARNING: "
			"All NvMap Allocations must have a tag "
			"to identify the subsystem allocating memory."
			"Please pass the tag to the API call"
			" NvRmMemHanldeAllocAttr() or relevant. \n",
			client->task->pid, task_comm);
	}
}
