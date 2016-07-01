/*
 * PVA buffer management for T194
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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
 * pva_vm_buffer - Virtual mapping information for a buffer
 *
 * @buf:		Pointer to dma_buf struct
 * @attach:		Pointer to dma_buf_attachment struct
 * @sgt:		Pointer to sg_table struct
 * @addr:		Physical address of the buffer
 * @size:		Size of the buffer
 * @memhandle:		MemHandle of the buffer passed from user space
 * @user_map_count:	Buffer reference count from user space
 * @submit_map_count:	Buffer reference count from task submit
 * @pin_list:		List of pinned buffer
 *
 */
struct pva_vm_buffer {
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;

	dma_addr_t addr;
	size_t size;
	u32 memhandle;

	s32 user_map_count;
	s32 submit_map_count;
	struct list_head pin_list;
};

static struct pva_vm_buffer *pva_find_map_buffer(
		struct pva_buffers *pva_buffers, u32 handle)
{
	struct pva_vm_buffer *vm;

	list_for_each_entry(vm, &pva_buffers->buffer_list, pin_list) {
		if (vm->memhandle == handle)
			return vm;
	}

	return NULL;
}

static int pva_buffer_map(struct platform_device *pdev, u32 mem_id,
			struct pva_vm_buffer *vm)
{
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t addr;
	int err = 0;

	buf = dma_buf_get(mem_id);
	if (IS_ERR_OR_NULL(buf)) {
		err = PTR_ERR(buf);
		dev_err(&pdev->dev, "dma_buf_get failed: %d\n", err);
		goto buf_get_err;
	}

	attach = dma_buf_attach(buf, &pdev->dev);
	if (IS_ERR_OR_NULL(attach)) {
		err = PTR_ERR(buf);
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
	vm->buf = buf;
	vm->size = buf->size;
	vm->addr = addr;
	vm->memhandle = mem_id;
	vm->user_map_count = 1;

	return err;

buf_map_err:
	dma_buf_detach(buf, attach);
buf_attach_err:
	dma_buf_put(buf);
buf_get_err:
	return err;
}

static void pva_free_buffers(struct kref *kref)
{
	struct pva_buffers *pva_buffers = container_of(kref,
					struct pva_buffers, kref);
	kfree(pva_buffers);
}

static void pva_buffer_unmap(struct pva_vm_buffer *vm)
{
	nvhost_dbg_fn("");

	if ((vm->user_map_count != 0) &&
		(vm->submit_map_count != 0))
		return;

	dma_buf_unmap_attachment(vm->attach, vm->sgt,
				DMA_BIDIRECTIONAL);
	dma_buf_detach(vm->buf, vm->attach);
	dma_buf_put(vm->buf);

	list_del(&vm->pin_list);
	kfree(vm);
}

struct pva_buffers *pva_buffer_init(struct platform_device *pdev)
{
	struct pva_buffers *pva_buffers;
	int err = 0;

	pva_buffers = kzalloc(sizeof(struct pva_buffers), GFP_KERNEL);
	if (!pva_buffers) {
		err = -ENOMEM;
		goto pva_buffer_init_err;
	}

	pva_buffers->pdev = pdev;
	mutex_init(&pva_buffers->buffer_list_mutex);
	INIT_LIST_HEAD(&pva_buffers->buffer_list);
	kref_init(&pva_buffers->kref);

	return pva_buffers;

pva_buffer_init_err:
	return ERR_PTR(err);
}

int pva_buffer_submit_pin(struct pva_buffers *pva_buffers,
				u32 *handles, u32 count)
{
	struct pva_vm_buffer *vm;
	int i = 0;

	mutex_lock(&pva_buffers->buffer_list_mutex);

	for (i = 0; i < count; i++) {

		vm = pva_find_map_buffer(pva_buffers, handles[i]);
		if (vm)
			vm->submit_map_count++;
		else
			goto submit_err;
	}

	kref_get(&pva_buffers->kref);
	mutex_unlock(&pva_buffers->buffer_list_mutex);
	return 0;

submit_err:
	mutex_unlock(&pva_buffers->buffer_list_mutex);

	count = i;
	pva_buffer_submit_unpin(pva_buffers, handles, count);

	return -EINVAL;
}

int pva_buffer_pin(struct pva_buffers *pva_buffers, u32 *handles, u32 count)
{
	struct pva_vm_buffer *vm;
	int i = 0;
	int err = 0;

	mutex_lock(&pva_buffers->buffer_list_mutex);

	for (i = 0; i < count; i++) {

		vm = pva_find_map_buffer(pva_buffers, handles[i]);
		if (vm) {
			vm->user_map_count++;
			continue;
		}

		vm = kzalloc(sizeof(struct pva_vm_buffer), GFP_KERNEL);
		if (!vm)
			goto buf_alloc_err;

		err = pva_buffer_map(pva_buffers->pdev, handles[i], vm);
		if (err)
			goto pva_pin_err;

		list_add_tail(&vm->pin_list, &pva_buffers->buffer_list);
	}

	mutex_unlock(&pva_buffers->buffer_list_mutex);
	return err;

buf_alloc_err:
pva_pin_err:
	mutex_unlock(&pva_buffers->buffer_list_mutex);

	/* free pinned buffers */
	count = i;
	pva_buffer_unpin(pva_buffers, handles, count);

	return err;
}

void pva_buffer_submit_unpin(struct pva_buffers *pva_buffers,
					u32 *handles, u32 count)
{
	struct pva_vm_buffer *vm;
	int i = 0;

	mutex_lock(&pva_buffers->buffer_list_mutex);

	for (i = 0; i < count; i++) {

		vm = pva_find_map_buffer(pva_buffers, handles[i]);
		if (vm) {
			if (vm->submit_map_count-- < 0)
				vm->submit_map_count = 0;
			pva_buffer_unmap(vm);
		}
	}

	mutex_unlock(&pva_buffers->buffer_list_mutex);

	kref_put(&pva_buffers->kref, pva_free_buffers);
}

void pva_buffer_unpin(struct pva_buffers *pva_buffers, u32 *handles, u32 count)
{
	int i = 0;

	mutex_lock(&pva_buffers->buffer_list_mutex);

	for (i = 0; i < count; i++) {
		struct pva_vm_buffer *vm = NULL;

		vm = pva_find_map_buffer(pva_buffers, handles[i]);
		if (vm) {
			if (vm->user_map_count-- < 0)
				vm->user_map_count = 0;
			pva_buffer_unmap(vm);
		}
	}

	mutex_unlock(&pva_buffers->buffer_list_mutex);
}

void pva_buffer_put(struct pva_buffers *pva_buffers)
{
	struct pva_vm_buffer *vm;

	mutex_lock(&pva_buffers->buffer_list_mutex);

	list_for_each_entry(vm, &pva_buffers->buffer_list, pin_list) {
		vm->user_map_count = 0;
		pva_buffer_unmap(vm);
	}

	mutex_unlock(&pva_buffers->buffer_list_mutex);

	kref_put(&pva_buffers->kref, pva_free_buffers);
}
