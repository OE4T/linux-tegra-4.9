/*
 * NVHOST buffer management for T194
 *
 * Copyright (c) 2016-2017, NVIDIA Corporation.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>

#include "dev.h"
#include "nvhost_buffer.h"

/**
 * nvhost_vm_buffer - Virtual mapping information for a buffer
 *
 * @buf:		Pointer to dma_buf struct
 * @attach:		Pointer to dma_buf_attachment struct
 * @sgt:		Pointer to sg_table struct
 * @addr:		Physical address of the buffer
 * @size:		Size of the buffer
 * @user_map_count:	Buffer reference count from user space
 * @submit_map_count:	Buffer reference count from task submit
 * @pin_list:		List of pinned buffer
 *
 */
struct nvhost_vm_buffer {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;

	dma_addr_t addr;
	size_t size;

	s32 user_map_count;
	s32 submit_map_count;
	struct list_head pin_list;
};

static struct nvhost_vm_buffer *nvhost_find_map_buffer(
		struct nvhost_buffers *nvhost_buffers, struct dma_buf *dmabuf)
{
	struct nvhost_vm_buffer *vm;

	/* Go through the handles and look for matching dmabuf */
	list_for_each_entry(vm, &nvhost_buffers->buffer_list, pin_list) {
		if (vm->dmabuf == dmabuf)
			return vm;
	}

	return NULL;
}

int nvhost_get_iova_addr(struct nvhost_buffers *nvhost_buffers,
			struct dma_buf *dmabuf, dma_addr_t *addr)
{
	struct nvhost_vm_buffer *vm;
	int err = -EINVAL;

	mutex_lock(&nvhost_buffers->buffer_list_mutex);

	vm = nvhost_find_map_buffer(nvhost_buffers, dmabuf);
	if (vm) {
		*addr = vm->addr;
		err = 0;
	}

	mutex_unlock(&nvhost_buffers->buffer_list_mutex);

	return err;
}

static int nvhost_buffer_map(struct platform_device *pdev,
				struct dma_buf *dmabuf,
				struct nvhost_vm_buffer *vm)
{
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t addr;
	int err = 0;

	/* increase dmabuf refcount */
	get_dma_buf(dmabuf);

	attach = dma_buf_attach(dmabuf, &pdev->dev);
	if (IS_ERR_OR_NULL(attach)) {
		err = PTR_ERR(dmabuf);
		dev_err(&pdev->dev, "dma_attach failed: %d\n", err);
		goto buf_attach_err;
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(sgt)) {
		err = PTR_ERR(sgt);
		dev_err(&pdev->dev, "dma mapping failed: %d\n", err);
		goto buf_map_err;
	}

	addr = sg_dma_address(sgt->sgl);
	if (!addr)
		addr = sg_phys(sgt->sgl);

	vm->sgt = sgt;
	vm->attach = attach;
	vm->dmabuf = dmabuf;
	vm->size = dmabuf->size;
	vm->addr = addr;
	vm->user_map_count = 1;

	return err;

buf_map_err:
	dma_buf_detach(dmabuf, attach);
buf_attach_err:
	dma_buf_put(dmabuf);
	return err;
}

static void nvhost_free_buffers(struct kref *kref)
{
	struct nvhost_buffers *nvhost_buffers =
		container_of(kref, struct nvhost_buffers, kref);

	kfree(nvhost_buffers);
}

static void nvhost_buffer_unmap(struct nvhost_vm_buffer *vm)
{
	nvhost_dbg_fn("");

	if ((vm->user_map_count != 0) ||
		(vm->submit_map_count != 0))
		return;

	dma_buf_unmap_attachment(vm->attach, vm->sgt,
				DMA_BIDIRECTIONAL);
	dma_buf_detach(vm->dmabuf, vm->attach);
	dma_buf_put(vm->dmabuf);

	list_del(&vm->pin_list);
	kfree(vm);
}

struct nvhost_buffers *nvhost_buffer_init(struct platform_device *pdev)
{
	struct nvhost_buffers *nvhost_buffers;
	int err = 0;

	nvhost_buffers = kzalloc(sizeof(struct nvhost_buffers), GFP_KERNEL);
	if (!nvhost_buffers) {
		err = -ENOMEM;
		goto nvhost_buffer_init_err;
	}

	nvhost_buffers->pdev = pdev;
	mutex_init(&nvhost_buffers->buffer_list_mutex);
	INIT_LIST_HEAD(&nvhost_buffers->buffer_list);
	kref_init(&nvhost_buffers->kref);

	return nvhost_buffers;

nvhost_buffer_init_err:
	return ERR_PTR(err);
}

int nvhost_buffer_submit_pin(struct nvhost_buffers *nvhost_buffers,
			     struct dma_buf **dmabufs, u32 count,
			     dma_addr_t *paddr, size_t *psize)
{
	struct nvhost_vm_buffer *vm;
	int i = 0;

	kref_get(&nvhost_buffers->kref);

	mutex_lock(&nvhost_buffers->buffer_list_mutex);

	for (i = 0; i < count; i++) {

		vm = nvhost_find_map_buffer(nvhost_buffers, dmabufs[i]);
		if (vm) {
			vm->submit_map_count++;
			paddr[i] = vm->addr;
			psize[i] = vm->size;
		} else {
			goto submit_err;
		}
	}

	mutex_unlock(&nvhost_buffers->buffer_list_mutex);
	return 0;

submit_err:
	mutex_unlock(&nvhost_buffers->buffer_list_mutex);

	count = i;

	nvhost_buffer_submit_unpin(nvhost_buffers, dmabufs, count);

	return -EINVAL;
}

int nvhost_buffer_pin(struct nvhost_buffers *nvhost_buffers,
			struct dma_buf **dmabufs,
			u32 count)
{
	struct nvhost_vm_buffer *vm;
	int i = 0;
	int err = 0;

	mutex_lock(&nvhost_buffers->buffer_list_mutex);

	for (i = 0; i < count; i++) {

		vm = nvhost_find_map_buffer(nvhost_buffers, dmabufs[i]);
		if (vm) {
			vm->user_map_count++;
			continue;
		}

		vm = kzalloc(sizeof(struct nvhost_vm_buffer), GFP_KERNEL);
		if (!vm)
			goto unpin;

		err = nvhost_buffer_map(nvhost_buffers->pdev, dmabufs[i], vm);
		if (err)
			goto free_vm;

		list_add_tail(&vm->pin_list, &nvhost_buffers->buffer_list);
	}

	mutex_unlock(&nvhost_buffers->buffer_list_mutex);
	return err;

free_vm:
	kfree(vm);
unpin:
	mutex_unlock(&nvhost_buffers->buffer_list_mutex);

	/* free pinned buffers */
	count = i;
	nvhost_buffer_unpin(nvhost_buffers, dmabufs, count);

	return err;
}

void nvhost_buffer_submit_unpin(struct nvhost_buffers *nvhost_buffers,
				struct dma_buf **dmabufs, u32 count)
{
	struct nvhost_vm_buffer *vm;
	int i = 0;

	mutex_lock(&nvhost_buffers->buffer_list_mutex);

	for (i = 0; i < count; i++) {

		vm = nvhost_find_map_buffer(nvhost_buffers, dmabufs[i]);
		if (vm) {
			if (vm->submit_map_count-- < 0)
				vm->submit_map_count = 0;
			nvhost_buffer_unmap(vm);
		}
	}

	mutex_unlock(&nvhost_buffers->buffer_list_mutex);

	kref_put(&nvhost_buffers->kref, nvhost_free_buffers);
}

void nvhost_buffer_unpin(struct nvhost_buffers *nvhost_buffers,
			 struct dma_buf **dmabufs, u32 count)
{
	int i = 0;

	mutex_lock(&nvhost_buffers->buffer_list_mutex);

	for (i = 0; i < count; i++) {
		struct nvhost_vm_buffer *vm = NULL;

		vm = nvhost_find_map_buffer(nvhost_buffers, dmabufs[i]);
		if (vm) {
			if (vm->user_map_count-- < 0)
				vm->user_map_count = 0;
			nvhost_buffer_unmap(vm);
		}
	}

	mutex_unlock(&nvhost_buffers->buffer_list_mutex);
}

void nvhost_buffer_put(struct nvhost_buffers *nvhost_buffers)
{
	struct nvhost_vm_buffer *vm, *n;

	mutex_lock(&nvhost_buffers->buffer_list_mutex);

	list_for_each_entry_safe(vm, n, &nvhost_buffers->buffer_list,
				 pin_list) {
		vm->user_map_count = 0;
		nvhost_buffer_unmap(vm);
	}

	mutex_unlock(&nvhost_buffers->buffer_list_mutex);

	kref_put(&nvhost_buffers->kref, nvhost_free_buffers);
}
