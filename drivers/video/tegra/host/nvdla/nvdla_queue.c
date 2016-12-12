/*
 * NVDLA queue and task management for T194
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

#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "dev.h"
#include "bus_client.h"
#include "chip_support.h"
#include "nvhost_acm.h"
#include "nvhost_queue.h"
#include "nvhost_syncpt_unit_interface.h"

#include "nvdla/nvdla.h"
#include "nvdla/nvdla_debug.h"
#include "dla_os_interface.h"

/* TODO: 1. revisit timeout post silicon
 *       2. when silicon and sim tests go live at same time,
 *          make timeout selection runtime based on platform
 */
#define NVDLA_QUEUE_ABORT_TIMEOUT	10000	/* 10 sec */
#define NVDLA_QUEUE_ABORT_RETRY_PERIOD	500	/* 500 ms */

#define NVDLA_MAX_PREACTION_SIZE (MAX_NUM_NVDLA_PREFENCES * \
				sizeof(struct dla_action_opcode) + \
				MAX_NUM_NVDLA_PREFENCES * \
				sizeof(struct dla_action_semaphore) + \
				sizeof(struct dla_action_opcode))

#define NVDLA_MAX_POSTACTION_SIZE (MAX_NUM_NVDLA_POSTFENCES * \
				sizeof(struct dla_action_opcode) + \
				MAX_NUM_NVDLA_POSTFENCES * \
				sizeof(struct dla_action_semaphore) + \
				sizeof(struct dla_action_opcode))

static DEFINE_DMA_ATTRS(attrs);

/* task management API's */
static void task_free(struct kref *ref)
{
	struct nvdla_task *task = container_of(ref, struct nvdla_task, ref);
	struct platform_device *pdev = task->queue->pool->pdev;

	nvdla_dbg_info(pdev, "freeing task[%p]", task);

	/* free allocated task desc */
	if (task->task_desc) {
		dma_free_attrs(&pdev->dev, task->buf_size,
			task->task_desc, task->task_desc_pa,
			&attrs);
		task->task_desc = NULL;
	}

	/* free operation descriptor handle */
	if (task->memory_handles)
		kfree(task->memory_handles);

	/* finally free task */
	kfree(task);
}

void nvdla_task_put(struct nvdla_task *task)
{
	/* release queue refcnt */
	nvhost_queue_put(task->queue);

	kref_put(&task->ref, task_free);
}

void nvdla_task_get(struct nvdla_task *task)
{
	kref_get(&task->ref);

	/* update queue refcnt */
	nvhost_queue_get(task->queue);
}

static void nvdla_task_free_locked(struct nvdla_task *task)
{
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_info(pdev,
		"task[%p] completed. syncpt[%d] fence[%d]",
		task, queue->syncpt_id, task->fence);

	/* give syncpoint reference */
	nvhost_syncpt_put_ref(task->sp, queue->syncpt_id);

	/* unpin submit ref */
	if (task->num_handles)
		nvhost_buffer_submit_unpin(task->buffers,
			task->memory_handles, task->num_handles);

	/* update takslist */
	list_del(&task->list);

	/* give taks refs */
	nvdla_task_put(task);
}

static void nvdla_task_syncpt_reset(struct nvhost_syncpt *syncpt,
			u32 id, u32 fence)
{
	atomic_set(&syncpt->min_val[id], fence);
	syncpt_op().reset(syncpt, id);
	nvhost_syncpt_update_min(syncpt, id);
}

static void nvdla_queue_update(void *priv, int nr_completed)
{
	int task_complete;
	struct nvdla_task *task, *safe;
	struct nvhost_queue *queue = priv;
	struct platform_device *pdev = queue->pool->pdev;

	mutex_lock(&queue->list_lock);

	/* check which task(s) finished */
	list_for_each_entry_safe(task, safe, &queue->tasklist, list) {

		task_complete = nvhost_syncpt_is_expired(task->sp,
					queue->syncpt_id, task->fence);

		/* clean task and remove from list */
		if (task_complete)
			nvdla_task_free_locked(task);
	}
	/* put pm refcount */
	nvhost_module_idle_mult(pdev, nr_completed);

	mutex_unlock(&queue->list_lock);
}

static int nvdla_map_task_memory(struct nvdla_task *task)
{
	int i;
	int err = 0;
	u32 *handles;
	size_t *dma_size;
	void *ptr = NULL;
	dma_addr_t *dma_addr;
	dma_addr_t *dma_memory;
	struct dma_buf *buf = NULL;
	struct nvdla_mem_handle *addresses;
	struct nvhost_buffers *buffers = task->buffers;
	struct dla_task_descriptor *task_desc = task->task_desc;

	task->num_handles = 0;

	/* keep address list always last */
	if (task->num_addresses)
		task->num_handles = task->num_addresses + 1;

	if (task->num_handles == 0)
		return err;

	/*
	 * Allocate memory to store information for DMA mapping of
	 * buffers allocated from user space
	 */
	task->memory_handles = kcalloc(task->num_handles, sizeof(u32),
				GFP_KERNEL);
	if (!task->memory_handles) {
		err = -ENOMEM;
		goto fail_to_alloc_handles;
	}

	handles = task->memory_handles;

	dma_addr = kcalloc(task->num_handles, sizeof(dma_addr_t),
				GFP_KERNEL);
	if (!dma_addr) {
		err = -ENOMEM;
		goto fail_to_alloc_dma_addr;
	}

	dma_memory = dma_addr;
	dma_size = kcalloc(task->num_handles, sizeof(u32),
				GFP_KERNEL);
	if (!dma_size) {
		err = -ENOMEM;
		goto fail_to_alloc_dma_size;
	}

	/*
	 * Fill in handles from list of addresses, need to map
	 * address list buffer in kernel and update same buffer
	 * with DMA addresses obtained.
	 */
	if (task->num_addresses) {
		uintptr_t temp;

		*handles++ = task->address_list.handle;

		buf = dma_buf_get(task->address_list.handle);
		if (IS_ERR(buf)) {
			err = PTR_ERR(buf);
			goto fail_to_pin_mem;
		}

		ptr = dma_buf_vmap(buf);
		if (!ptr) {
			err = -ENOMEM;
			goto fail_to_pin_mem;
		}

		dma_buf_begin_cpu_access(buf, task->address_list.offset,
				sizeof(uint64_t) * task->num_addresses,
				DMA_TO_DEVICE);

		temp = (uintptr_t)(ptr);
		addresses =
			(struct nvdla_mem_handle *)
				(temp + task->address_list.offset);

		for (i = 0; i < task->num_addresses; i++, addresses++)
			*handles++ = addresses->handle;
	}

	/* Get DMA addresses for all handles */
	err = nvhost_buffer_submit_pin(buffers, task->memory_handles,
				task->num_handles, dma_addr, dma_size);
	if (err) {
		goto fail_to_pin_mem;
	}

	/* Update IOVA addresses in task descriptor */
	task_desc->num_addresses = task->num_addresses;
	if (task->num_addresses) {
		uintptr_t temp;
		uint64_t *dma_addr_list;

		temp = (uintptr_t)(ptr);
		dma_addr_list = (uint64_t *)
				(temp + task->address_list.offset);
		addresses =
			(struct nvdla_mem_handle *)
				(temp + task->address_list.offset);

		task_desc->address_list = (*dma_addr++) +
					task->address_list.offset;

		for (i = 0; i < task->num_addresses; i++, addresses++) {
			uint64_t offset = (uint64_t)addresses->offset;

			*dma_addr_list++ = (uint64_t)(*dma_addr++) + offset;
		}

		dma_buf_vunmap(buf, ptr);

		dma_buf_end_cpu_access(buf, task->address_list.offset,
				sizeof(uint64_t) * task->num_addresses,
				DMA_TO_DEVICE);

		dma_buf_put(buf);
	}

	if (dma_memory)
		kfree(dma_memory);
	if (dma_size)
		kfree(dma_size);

	return 0;

fail_to_pin_mem:
	if (dma_size)
		kfree(dma_size);
fail_to_alloc_dma_size:
	if (dma_memory)
		kfree(dma_memory);
fail_to_alloc_dma_addr:
	if (task->memory_handles)
		kfree(task->memory_handles);
fail_to_alloc_handles:
	return err;
}

static size_t nvdla_get_task_desc_size(void)
{

	size_t size = 0;

	/* calculate size of task desc, actions and its list, buffers
	 * this is max possible size for updating task desc and
	 * and allocated mem size can be more than required size
	 */
	size += sizeof(struct dla_task_descriptor);
	size += (2 * MAX_NUM_ACTION_LIST * sizeof(struct dla_action_list));
	size +=	NVDLA_MAX_PREACTION_SIZE;
	size +=	NVDLA_MAX_POSTACTION_SIZE;

	return size;

}

static int nvdla_fill_postactions(struct nvdla_task *task)
{
	struct dla_task_descriptor *task_desc = task->task_desc;
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;
	struct dla_action_semaphore *postaction;
	struct dla_action_list *postactionl;
	struct dla_action_opcode *opcode;
	uint16_t postactionlist_of;
	void *mem;
	int i;

	/* update postaction list offset */
	postactionlist_of = task_desc->postactions +
		sizeof(struct dla_action_list) + NVDLA_MAX_PREACTION_SIZE;

	/* fill all postactions */
	for (i = 0; i < task->num_postfences; i++, postaction++) {
		void *next = NULL;

		/* get next post action base */
		next = (char *)task_desc + postactionlist_of +
		 i * (sizeof(struct dla_action_opcode) +
			sizeof(struct dla_action_semaphore));

		/* get base opcode */
		opcode = (struct dla_action_opcode *)next;

		/* update end of list */
		if (i == task->num_postfences) {
			opcode->value = POSTACTION_TERMINATE;
			break;
		}

		/* set action type */
		opcode->value = POSTACTION_SEM;

		/* get actual post action mem */
		postaction = (struct dla_action_semaphore *)
			((char *)opcode + sizeof(struct dla_action_opcode));

		/* update action */
		postaction->address = nvhost_syncpt_address(pdev,
						queue->syncpt_id);
	}

	mem = (char *)task_desc + task_desc->postactions;
	postactionl = (struct dla_action_list *)mem;
	postactionl->offset = postactionlist_of;
	postactionl->size = i * (sizeof(struct dla_action_opcode) +
			sizeof(struct dla_action_semaphore)) +
			sizeof(struct dla_action_opcode);

	return 0;
}

static int nvdla_fill_preactions(struct nvdla_task *task)
{
	struct dla_task_descriptor *task_desc = task->task_desc;
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;
	struct dla_action_semaphore *preaction;
	struct dla_action_list *preactionl;
	struct dla_action_opcode *opcode;
	uint16_t preactionlist_of;
	void *mem;
	int i;

	/* preaction list offset update */
	preactionlist_of = task_desc->postactions +
					sizeof(struct dla_action_list);

	/* fill all preactions */
	for (i = 0; i <= task->num_prefences; i++) {
		void *next = NULL;

		/* get next preaction base */
		next = (char *)task_desc + preactionlist_of +
		  i * (sizeof(struct dla_action_opcode) +
			sizeof(struct dla_action_semaphore));

		/* get base opcode */
		opcode = (struct dla_action_opcode *)next;

		/* update end of action list */
		if (i == task->num_prefences) {
			opcode->value = PREACTION_TERMINATE;
			break;
		}

		/* set action type */
		opcode->value = PREACTION_SEM_GE;

		/* get actual preaction address */
		preaction = (struct dla_action_semaphore *)
			((char *)opcode + sizeof(struct dla_action_opcode));

		/* update action */
		preaction->address = nvhost_syncpt_address(pdev,
					task->prefences[i].syncpoint_index);
		preaction->value = task->prefences[i].syncpoint_value;
	}

	/* actually update lists data */
	mem = (char *)task_desc + task_desc->preactions;
	preactionl = (struct dla_action_list *)mem;
	preactionl->offset = preactionlist_of;
	preactionl->size = i * (sizeof(struct dla_action_opcode) +
			sizeof(struct dla_action_semaphore)) +
			sizeof(struct dla_action_opcode);

	return 0;
}

int nvdla_fill_task_desc(struct nvdla_task *task)
{
	int err;
	u32 *buffer_va;
	size_t buf_size;
	dma_addr_t buffer_pa;
	struct dla_task_descriptor *task_desc;
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_fn(pdev, "");

	buf_size = nvdla_get_task_desc_size();

	/* allocate task descriptor */
	buffer_va = dma_alloc_attrs(&pdev->dev, buf_size, &buffer_pa,
				GFP_KERNEL, &attrs);

	if (!buffer_va) {
		nvdla_dbg_err(pdev, "dma memory allocation failed for task");
		err = -ENOMEM;
		goto fail_to_dma_alloc;
	}

	task->task_desc = (struct dla_task_descriptor *)(buffer_va);
	task_desc = task->task_desc;
	task->task_desc_pa = buffer_pa;
	task->buf_size = buf_size;

	/* update task desc fields */
	task_desc->version = DLA_DESCRIPTOR_VERSION;
	task_desc->engine_id = DLA_ENGINE_ID;
	task_desc->size = buf_size;

	/* update current task sequeue, make sure wrap around condition */
	queue->sequence = queue->sequence + 1;
	if (unlikely(queue->sequence >= (UINT_MAX - 1)))
		queue->sequence = 0;

	task_desc->sequence = queue->sequence;

	/* below are actual number of action lists
	 * DLA has one preaction list and one postaction list
	 */
	task_desc->num_preactions = MAX_NUM_ACTION_LIST;
	task_desc->num_postactions = MAX_NUM_ACTION_LIST;

	task_desc->queue_id = queue->id;

	nvdla_dbg_info(pdev, "Queue id[%d]", task_desc->queue_id);

	/* get pre/post action list HEAD mem offset
	 * - preactions list HEAD stored after dla_task_descriptor
	 * - postactions list HEAD followed after preaction list head offset
	 * - DLA has only one list of actions for each of pre and post
	 */
	task_desc->preactions = sizeof(struct dla_task_descriptor);
	task_desc->postactions = task_desc->preactions +
					sizeof(struct dla_action_list);

	/* fill pre actions */
	nvdla_fill_preactions(task);

	/* fill post actions */
	nvdla_fill_postactions(task);

	/* ping user memory before submit to engine */
	err = nvdla_map_task_memory(task);
	if (err) {
		nvdla_dbg_err(pdev, "fail to pin mem");
		goto fail_to_map_mem;
	}

	nvdla_dbg_info(pdev, "task[%p] initialized", task);

	return 0;
fail_to_map_mem:
	/* TODO: free dma mem.
	 * Fixme after task static alloc */
fail_to_dma_alloc:
	return err;
}

/* Queue management API */
static int nvdla_queue_submit(struct nvhost_queue *queue, void *in_task)
{
	struct nvdla_task *task = (struct nvdla_task *)in_task;
	struct nvdla_task *last_task = NULL;
	struct platform_device *pdev = queue->pool->pdev;
	uint32_t method_data;
	uint32_t method_id;
	int err = 0;

	nvdla_dbg_fn(pdev, "");

	/* get pm refcount */
	if (nvhost_module_busy(pdev))
		return -EINVAL;

	mutex_lock(&queue->list_lock);

	/* get task ref and add to list */
	nvdla_task_get(task);

	/* update last task desc's next */
	if (!list_empty(&queue->tasklist)) {
		last_task = list_last_entry(&queue->tasklist,
						struct nvdla_task, list);
		last_task->task_desc->next = (uint64_t)task->task_desc_pa;
	}
	list_add_tail(&task->list, &queue->tasklist);

	nvdla_dbg_info(pdev, "task[%p] added to list", task);

	/* get fence from nvhost */
	task->fence = nvhost_syncpt_incr_max(task->sp, queue->syncpt_id, 1);

	nvdla_dbg_fn(pdev, "syncpt[%d] fence[%d] task[%p]", queue->syncpt_id,
				task->fence, task);

	/* get syncpoint reference */
	nvhost_syncpt_get_ref(task->sp, queue->syncpt_id);

	/* enable INT_ON_COMPLETE and INT_ON_ERROR falcon interrupts */
	method_id = (DLA_CMD_SUBMIT_TASK & DLA_METHOD_ID_CMD_MASK) |
			(1 << DLA_INT_ON_COMPLETE_SHIFT) |
			(1 << DLA_INT_ON_ERROR_SHIFT);
	method_data = ((task->task_desc_pa >> 8) & 0xffffffff);

	/* register notifier with fence */
	err = nvhost_intr_register_notifier(pdev, queue->syncpt_id,
		task->fence, nvdla_queue_update, queue);
	if (err)
		goto fail_to_register;

	/* Pass fence as through 0th postfences */
	task->postfences[0].syncpoint_index = queue->syncpt_id;
	task->postfences[0].syncpoint_value = task->fence;

	/* submit task to engine */
	err = nvdla_send_cmd(pdev, method_id, method_data, true);
	if (err)
		nvdla_task_syncpt_reset(task->sp, queue->syncpt_id,
				task->fence);

fail_to_register:
	mutex_unlock(&queue->list_lock);

	return err;
}

static int nvdla_queue_abort(struct nvhost_queue *queue)
{
	int err;
	struct nvdla_task *t;
	struct platform_device *pdev = queue->pool->pdev;
	int retry = NVDLA_QUEUE_ABORT_TIMEOUT / NVDLA_QUEUE_ABORT_RETRY_PERIOD;

	nvdla_dbg_fn(pdev, "");

	/* get pm refcount */
	err = nvhost_module_busy(pdev);
	if (err) {
		nvdla_dbg_err(pdev, "failed to poweron, err: %d", err);
		return err;
	}

	/* flush engine side queues */
	do {
		err = nvdla_send_cmd(pdev, DLA_CMD_QUEUE_FLUSH, queue->id,
					true);
		if (err == DLA_ERR_PROCESSOR_BUSY)
			mdelay(NVDLA_QUEUE_ABORT_RETRY_PERIOD);
		else
			break;
	} while (--retry);

	if (!retry || err) {
		nvdla_dbg_err(pdev,
		"Q %d abort fail. err:%d, retry:%d",
			queue->id, err, retry);
		goto done;
	}

	nvdla_dbg_info(pdev, "Engine Q[%d] flush done", queue->id);

	/* if task present free them by reset syncpoint */
	if (!list_empty(&queue->tasklist)) {
		t = list_last_entry(&queue->tasklist, struct nvdla_task, list);

		/* reset syncpoint to release all tasks */
		nvdla_task_syncpt_reset(t->sp, queue->syncpt_id, t->fence);

		/* dump details */
		nvdla_dbg_info(pdev, "Q id %d reset syncpt[%d] done",
			queue->id, queue->syncpt_id);
		nvdla_dbg_info(pdev, "syncpt[%d], min[%u], max[%u]",
			queue->syncpt_id,
			nvhost_syncpt_update_min(t->sp, queue->syncpt_id),
			nvhost_syncpt_read_max(t->sp, queue->syncpt_id));
	}

done:
	nvhost_module_idle(pdev);
	return err;
}

struct nvhost_queue_ops nvdla_queue_ops = {
	.abort = nvdla_queue_abort,
	.submit = nvdla_queue_submit,
};
