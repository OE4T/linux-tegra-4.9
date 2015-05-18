/*
 * Tegra Graphics Host Virtual Memory
 *
 * Copyright (c) 2014-2015, NVIDIA Corporation. All rights reserved.
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

#include "chip_support.h"
#include "nvhost_vm.h"
#include "dev.h"

struct nvhost_vm_pin {
	/* list of pinned buffers */
	struct nvhost_vm_buffer **buffers;
	unsigned int num_buffers;
};

int nvhost_vm_init_device(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (!vm_op().init_device || !pdata->isolate_contexts)
		return 0;

	return vm_op().init_device(pdev);
}

int nvhost_vm_get_id(struct nvhost_vm *vm)
{
	if (!vm_op().get_id)
		return -ENOSYS;

	return vm_op().get_id(vm);
}

int nvhost_vm_map_static(struct platform_device *pdev,
			 void *vaddr, dma_addr_t paddr,
			 size_t size)
{
	/* if static mappings are not supported, exit */
	if (!vm_op().pin_static_buffer)
		return 0;

	return vm_op().pin_static_buffer(pdev, vaddr, paddr, size);
}

static struct nvhost_vm_buffer *nvhost_vm_find_buffer(struct rb_root *root,
						      struct dma_buf *dmabuf)
{
	struct rb_node *node = root->rb_node;

	while (node) {
		struct nvhost_vm_buffer *buffer =
			container_of(node, struct nvhost_vm_buffer, node);

		if (buffer->dmabuf == dmabuf)
			return buffer;
		else if (buffer->dmabuf > dmabuf)
			node = node->rb_left;
		else
			node = node->rb_right;
	}

	return NULL;
}

static int insert_mapped_buffer(struct rb_root *root,
				struct nvhost_vm_buffer *buffer)
{
	struct rb_node **new_node = &(root->rb_node), *parent = NULL;

	/* Figure out where to put new node */
	while (*new_node) {
		struct nvhost_vm_buffer *cmp_with =
			container_of(*new_node, struct nvhost_vm_buffer,
			node);

		parent = *new_node;

		if (cmp_with->dmabuf > buffer->dmabuf)
			new_node = &((*new_node)->rb_left);
		else if (cmp_with->dmabuf != buffer->dmabuf)
			new_node = &((*new_node)->rb_right);
		else
			return -EINVAL; /* duplicate buffer not allowed */
	}

	/* Add new node and rebalance tree. */
	rb_link_node(&buffer->node, parent, new_node);
	rb_insert_color(&buffer->node, root);

	return 0;
}

static void nvhost_vm_destroy_buffer_locked(struct nvhost_vm_buffer *buffer)
{
	struct nvhost_vm *vm = buffer->vm;

	dma_buf_unmap_attachment(buffer->attach, buffer->sgt,
				 DMA_BIDIRECTIONAL);
	dma_buf_detach(buffer->dmabuf, buffer->attach);
	dma_buf_put(buffer->dmabuf);

	rb_erase(&buffer->node, &vm->buffer_list);

	if (!vm->buffer_list.rb_node && vm_op().deinit && vm->enable_hw)
		vm_op().deinit(vm);

	kfree(buffer);
	buffer = NULL;
}

static void nvhost_vm_buffer_deinit_locked(struct kref *kref)
{
	struct nvhost_vm_buffer *buffer =
			container_of(kref, struct nvhost_vm_buffer, kref);
	nvhost_vm_destroy_buffer_locked(buffer);
}

static void nvhost_vm_buffer_put_locked(struct nvhost_vm_buffer *buffer)
{
	kref_put(&buffer->kref, nvhost_vm_buffer_deinit_locked);
}

static void nvhost_vm_buffer_get(struct nvhost_vm_buffer *buffer)
{
	kref_get(&buffer->kref);
}

void nvhost_vm_unmap_dmabuf(struct nvhost_vm *vm, struct dma_buf *dmabuf)
{
	struct nvhost_vm_buffer *buffer;

	mutex_lock(&vm->mutex);

	/* find the buffer */
	buffer = nvhost_vm_find_buffer(&vm->buffer_list, dmabuf);
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
	buffer = nvhost_vm_find_buffer(&vm->buffer_list, dmabuf);
	if (buffer) {
		buffer->user_map_count++;
		if (buffer->user_map_count == 1)
			vm->num_user_mapped_buffers++;
		nvhost_vm_buffer_get(buffer);
		mutex_unlock(&vm->mutex);
		*addr = buffer->addr;
		return 0;
	}

	if (!vm->buffer_list.rb_node && vm_op().init && vm->enable_hw) {
		err = vm_op().init(vm);
		if (err)
			goto err_init;
	}

	/* allocate room to hold buffer data */
	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		goto err_alloc_buffer;

	get_dma_buf(dmabuf);
	buffer->dmabuf = dmabuf;
	buffer->vm = vm;
	buffer->size = dmabuf->size;
	buffer->user_map_count = 1;
	kref_init(&buffer->kref);

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

	buffer->addr = sg_dma_address(buffer->sgt->sgl);

	/* handle physical addresses */
	if (!buffer->addr)
		buffer->addr = sg_phys(buffer->sgt->sgl);

	/* add buffer to the buffer list to avoid duplicate mappings */
	err = insert_mapped_buffer(&vm->buffer_list, buffer);
	if (err) {
		nvhost_err(&vm->pdev->dev, "failed to insert mapped buffer\n");
		goto err_insert_buffer;
	}
	vm->num_user_mapped_buffers++;

	mutex_unlock(&vm->mutex);

	*addr = buffer->addr;
	return 0;

err_insert_buffer:
	dma_buf_unmap_attachment(buffer->attach,
			buffer->sgt, DMA_BIDIRECTIONAL);
err_map_attachment:
	dma_buf_detach(dmabuf, buffer->attach);
err_attach:
	kfree(buffer);
err_alloc_buffer:
	if (vm_op().deinit && vm->enable_hw)
		vm_op().deinit(vm);
err_init:
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
	struct rb_node *node;
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
	node = rb_first(&vm->buffer_list);
	while (node) {
		buffer = container_of(node, struct nvhost_vm_buffer, node);

		/* ...that are visible in application view */
		if (!buffer->user_map_count) {
			node = rb_next(&buffer->node);
			continue;
		}

		/* and add them to the list of submit buffers */
		buffer->submit_map_count++;
		nvhost_vm_buffer_get(buffer);
		buffers[i] = buffer;
		i++;

		node = rb_next(&buffer->node);
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
	struct nvhost_master *host = nvhost_get_host(vm->pdev);
	struct nvhost_vm_buffer *buffer;
	struct rb_node *node;

	/* remove this vm from the vms list */
	mutex_lock(&host->vm_mutex);
	list_del(&vm->vm_list);
	mutex_unlock(&host->vm_mutex);

	mutex_lock(&vm->mutex);

	/* go through all remaining buffers (if any) and free them here */
	node = rb_first(&vm->buffer_list);
	while (node) {
		buffer = container_of(node, struct nvhost_vm_buffer, node);

		nvhost_vm_destroy_buffer_locked(buffer);

		node = rb_first(&vm->buffer_list);
	}

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
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_master *host = nvhost_get_host(pdev);
	struct nvhost_vm *vm;

	/* get room to keep vm */
	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return NULL;

	vm->buffer_list = RB_ROOT;
	mutex_init(&vm->mutex);
	kref_init(&vm->kref);
	vm->pdev = pdev;
	vm->enable_hw = pdata->isolate_contexts;

	/* add this vm into list of vms */
	mutex_lock(&host->vm_mutex);
	list_add_tail(&vm->vm_list, &host->vm_list);
	mutex_unlock(&host->vm_mutex);

	return vm;
}
