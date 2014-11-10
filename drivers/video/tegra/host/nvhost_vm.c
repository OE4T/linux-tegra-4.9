/*
 * Tegra Graphics Host Virtual Memory
 *
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
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
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>

#include "nvhost_vm.h"

struct nvhost_vm {
	struct platform_device *pdev;

	struct kref kref;	/* reference to this VM */
	struct mutex mutex;

	/* list of buffers mapped into this VM */
	struct list_head buffer_list;

	/* count of application viewed buffers mapped into this VM */
	unsigned int num_user_mapped_buffers;
};

struct nvhost_vm_buffer {
	struct nvhost_vm *vm;

	/* buffer attachment */
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;

	/* context specific view to the buffer */
	dma_addr_t addr;
	size_t size;

	struct kref kref;	/* reference to this buffer */

	/* bookkeeping */
	unsigned int user_map_count;	/* application view to the buffer */
	unsigned int submit_map_count;	/* hw view to this buffer */
	struct list_head list;
};

struct nvhost_vm_pin {
	/* list of pinned buffers */
	struct nvhost_vm_buffer **buffers;
	unsigned int num_buffers;
};

static struct nvhost_vm_buffer *nvhost_vm_find_buffer(struct nvhost_vm *vm,
						      struct dma_buf *dmabuf)
{
	struct nvhost_vm_buffer *buffer;
	list_for_each_entry(buffer, &vm->buffer_list, list)
		if (buffer->dmabuf == dmabuf)
			return buffer;

	return NULL;
}

static void nvhost_vm_destroy_buffer_locked(struct nvhost_vm_buffer *buffer)
{
	dma_buf_unmap_attachment(buffer->attach, buffer->sgt,
				 DMA_BIDIRECTIONAL);
	dma_buf_detach(buffer->dmabuf, buffer->attach);
	dma_buf_put(buffer->dmabuf);
	list_del(&buffer->list);

	kfree(buffer);
	buffer = NULL;
}

static void nvhost_vm_buffer_deinit_locked(struct kref *kref)
{
	struct nvhost_vm_buffer *buffer =
			container_of(kref, struct nvhost_vm_buffer, kref);
	nvhost_vm_destroy_buffer_locked(buffer);
}

void nvhost_vm_buffer_put_locked(struct nvhost_vm_buffer *buffer)
{
	kref_put(&buffer->kref, nvhost_vm_buffer_deinit_locked);
}

void nvhost_vm_buffer_get(struct nvhost_vm_buffer *buffer)
{
	kref_get(&buffer->kref);
}

void nvhost_vm_unmap_dmabuf(struct nvhost_vm *vm, struct dma_buf *dmabuf)
{
	struct nvhost_vm_buffer *buffer;

	mutex_lock(&vm->mutex);

	/* find the buffer */
	buffer = nvhost_vm_find_buffer(vm, dmabuf);
	if (!buffer)
		goto err_find_buffer;

	/* check that the buffer is mapped by user */
	if (!buffer->user_map_count)
		goto err_dec_refcount;

	/* handle bookkeeping */
	buffer->user_map_count--;
	if (!buffer->user_map_count)
		vm->num_user_mapped_buffers--;

	nvhost_vm_buffer_put_locked(buffer);

	mutex_unlock(&vm->mutex);

	return;

err_dec_refcount:
err_find_buffer:
	mutex_unlock(&vm->mutex);
	WARN(1, "dmabuf %p already unmapped", dmabuf);
}

int nvhost_vm_map_dmabuf(struct nvhost_vm *vm, struct dma_buf *dmabuf,
			 dma_addr_t *addr)
{
	struct nvhost_vm_buffer *buffer;
	int err;

	mutex_lock(&vm->mutex);

	/* avoid duplicate mappings */
	buffer = nvhost_vm_find_buffer(vm, dmabuf);
	if (buffer) {
		buffer->user_map_count++;
		if (buffer->user_map_count == 1)
			vm->num_user_mapped_buffers++;
		nvhost_vm_buffer_get(buffer);
		mutex_unlock(&vm->mutex);
		*addr = buffer->addr;
		return 0;
	}

	/* allocate room to hold buffer data */
	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		goto err_alloc_buffer;

	get_dma_buf(dmabuf);
	buffer->dmabuf = dmabuf;
	buffer->size = dmabuf->size;
	buffer->user_map_count = 1;
	kref_init(&buffer->kref);
	INIT_LIST_HEAD(&buffer->list);

	/* attach buffer to host1x device */
	buffer->attach = dma_buf_attach(dmabuf, &vm->pdev->dev);
	if (IS_ERR(buffer->attach)) {
		err = PTR_ERR(buffer->attach);
		goto err_attach;
	}

	/* ..and map the attachment */
	buffer->sgt = dma_buf_map_attachment(buffer->attach,
					     DMA_BIDIRECTIONAL);
	if (IS_ERR(buffer->sgt)) {
		err = PTR_ERR(buffer->sgt);
		goto err_map_attachment;
	}

	/* get dma address */
	buffer->addr = sg_dma_address(buffer->sgt->sgl);

	/* handle physical addresses */
	if (!buffer->addr)
		buffer->addr = sg_phys(buffer->sgt->sgl);

	/* add buffer to the buffer list to avoid duplicate mappings */
	list_add_tail(&buffer->list, &vm->buffer_list);
	vm->num_user_mapped_buffers++;

	mutex_unlock(&vm->mutex);

	*addr = buffer->addr;
	return 0;

err_map_attachment:
	dma_buf_detach(dmabuf, buffer->attach);
err_attach:
	kfree(buffer);
err_alloc_buffer:
	mutex_unlock(&vm->mutex);

	return -ENOMEM;
}

void nvhost_vm_unpin_buffers(struct nvhost_vm *vm, struct nvhost_vm_pin *pin)
{
	int i;

	mutex_lock(&vm->mutex);

	/* for each pinned buffer: */
	for (i = 0; i < pin->num_buffers; i++) {
		struct nvhost_vm_buffer *buffer = pin->buffers[i];

		/* check the buffer state */
		if (!buffer->submit_map_count) {
			WARN(1, "inconsistent state while unpinning. leaking memory to avoid crash\n");
			mutex_unlock(&vm->mutex);
			return;
		}

		/* reduce number of submit maps */
		buffer->submit_map_count--;

		nvhost_vm_buffer_put_locked(buffer);
	}

	mutex_unlock(&vm->mutex);

	kfree(pin->buffers);
	kfree(pin);
}

struct nvhost_vm_pin *nvhost_vm_pin_buffers(struct nvhost_vm *vm)
{
	struct nvhost_vm_pin *pin;
	struct nvhost_vm_buffer **buffers, *buffer;
	int i = 0;

	/* allocate space for pin.. */
	pin = kzalloc(sizeof(*pin), GFP_KERNEL);
	if (!pin)
		goto err_alloc_pin;

	/* ..and buffers */
	buffers = kzalloc(sizeof(*buffers) * vm->num_user_mapped_buffers,
			  GFP_KERNEL);
	if (!buffers)
		goto err_alloc_buffers;

	mutex_lock(&vm->mutex);

	/* go through all mapped buffers */
	list_for_each_entry(buffer, &vm->buffer_list, list) {
		/* ...that are visible in application view */
		if (!buffer->user_map_count)
			continue;

		/* and add them to the list of submit buffers */
		buffer->submit_map_count++;
		nvhost_vm_buffer_get(buffer);
		buffers[i] = buffer;
		i++;
	}

	/* store this data into pin */
	pin->num_buffers = vm->num_user_mapped_buffers;
	pin->buffers = buffers;

	mutex_unlock(&vm->mutex);

	return pin;

err_alloc_buffers:
	kfree(pin);
err_alloc_pin:
	return NULL;
}

static void nvhost_vm_deinit(struct kref *kref)
{
	struct nvhost_vm *vm = container_of(kref, struct nvhost_vm, kref);
	struct nvhost_vm_buffer *buffer, *buffer_tmp;

	mutex_lock(&vm->mutex);

	/* go through all remaining buffers (if any) and free them here */
	list_for_each_entry_safe(buffer, buffer_tmp, &vm->buffer_list, list)
		nvhost_vm_destroy_buffer_locked(buffer);

	mutex_unlock(&vm->mutex);

	kfree(vm);
	vm = NULL;
}

void nvhost_vm_put(struct nvhost_vm *vm)
{
	kref_put(&vm->kref, nvhost_vm_deinit);
}

void nvhost_vm_get(struct nvhost_vm *vm)
{
	kref_get(&vm->kref);
}

struct nvhost_vm *nvhost_vm_allocate(struct platform_device *pdev)
{
	struct nvhost_vm *vm;

	/* get room to keep vm */
	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		goto err_alloc_vm;

	INIT_LIST_HEAD(&vm->buffer_list);
	mutex_init(&vm->mutex);
	kref_init(&vm->kref);
	vm->pdev = pdev;

	return vm;

err_alloc_vm:
	return NULL;
}
