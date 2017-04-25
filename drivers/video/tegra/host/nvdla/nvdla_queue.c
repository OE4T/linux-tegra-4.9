/*
 * NVDLA queue and task management for T194
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

#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "../drivers/staging/android/sync.h"

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

/* task management API's */
int nvdla_get_task_mem(struct nvhost_queue *queue,
			struct nvdla_task **ptask)
{
	int err;
	struct nvdla_task *task = NULL;
	struct nvhost_queue_task_mem_info task_mem_info;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_fn(pdev, "");

	/* get mem task descriptor and task mem from task_mem_pool */
	err = nvhost_queue_alloc_task_memory(queue, &task_mem_info);
	task = task_mem_info.kmem_addr;
	if ((err < 0) || !task)
		goto fail_to_assign_pool;

	/* check if IOVA is correctly aligned */
	if (task_mem_info.dma_addr & 0xff) {
		err = -EFAULT;
		goto fail_to_aligned_dma;
	}

	task->task_desc = task_mem_info.va;
	task->task_desc_pa = task_mem_info.dma_addr;
	task->pool_index = task_mem_info.pool_index;

	*ptask = task;

fail_to_aligned_dma:
fail_to_assign_pool:
	return err;
}

void nvdla_put_task_mem(struct nvdla_task *task)
{
	/* release allocated task desc and task mem */
	nvhost_queue_free_task_memory(task->queue, task->pool_index);

	task = NULL;
}

static void task_free(struct kref *ref)
{
	struct nvdla_task *task = container_of(ref, struct nvdla_task, ref);
	struct platform_device *pdev = task->queue->pool->pdev;

	nvdla_dbg_info(pdev, "freeing task[%p]", task);

	nvdla_put_task_mem(task);
}

void nvdla_task_put(struct nvdla_task *task)
{
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_fn(pdev, "task:[%p]", task);

	kref_put(&task->ref, task_free);

	/* Queue should be last to update */
	nvhost_queue_put(queue);
}

void nvdla_task_get(struct nvdla_task *task)
{
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_fn(pdev, "task:[%p]", task);

	/* update queue refcnt */
	nvhost_queue_get(task->queue);

	kref_get(&task->ref);
}

static int nvdla_unmap_task_memory(struct nvdla_task *task)
{
	int ii;
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_fn(pdev, "task:[%p]", task);

	/* unpin address list */
	for (ii = 0; ii < task->num_addresses; ii++) {
		if (task->memory_handles[ii].handle) {
			nvhost_buffer_submit_unpin(task->buffers,
				&task->memory_dmabuf[ii], 1);
			dma_buf_put(task->memory_dmabuf[ii]);
		}
	}
	nvdla_dbg_fn(pdev, "all mem handles unmaped");

	/* unpin prefences memory */
	for (ii = 0; ii < task->num_prefences; ii++) {
		if (task->prefences[ii].type == NVDLA_FENCE_TYPE_SEMAPHORE &&
			task->prefences[ii].sem_handle) {
			nvhost_buffer_submit_unpin(task->buffers,
				&task->prefences_sem_dmabuf[ii], 1);
			dma_buf_put(task->prefences_sem_dmabuf[ii]);
		}
	}
	nvdla_dbg_fn(pdev, "all prefences unmaped");

	/* unpin input task status memory */
	for (ii = 0; ii < task->num_in_task_status; ii++) {
		if (task->in_task_status[ii].handle) {
			nvhost_buffer_submit_unpin(task->buffers,
				&task->in_task_status_dmabuf[ii], 1);
			dma_buf_put(task->in_task_status_dmabuf[ii]);
		}
	}
	nvdla_dbg_fn(pdev, "all in task status unmaped");

	/* unpin postfences memory */
	for (ii = 0; ii < task->num_postfences; ii++) {
		if ((task->postfences[ii].type == NVDLA_FENCE_TYPE_SEMAPHORE ||
		  task->postfences[ii].type == NVDLA_FENCE_TYPE_TS_SEMAPHORE) &&
		  task->postfences[ii].sem_handle) {
			nvhost_buffer_submit_unpin(task->buffers,
				&task->postfences_sem_dmabuf[ii], 1);
			dma_buf_put(task->postfences_sem_dmabuf[ii]);
		}
	}
	nvdla_dbg_fn(pdev, "all postfences unmaped");

	/* unpin input task status memory */
	for (ii = 0; ii < task->num_out_task_status; ii++) {
		if (task->out_task_status[ii].handle) {
			nvhost_buffer_submit_unpin(task->buffers,
				&task->out_task_status_dmabuf[ii], 1);
			dma_buf_put(task->out_task_status_dmabuf[ii]);
		}
	}
	nvdla_dbg_fn(pdev, "all out task status unmaped");

	return 0;
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

	/* unmap all memory shared with engine */
	nvdla_unmap_task_memory(task);

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

	nvdla_dbg_fn(pdev, "");

	/* check which task(s) finished */
	list_for_each_entry_safe(task, safe, &queue->tasklist, list) {

		task_complete = nvhost_syncpt_is_expired(task->sp,
					queue->syncpt_id, task->fence);

		/* clean task and remove from list */
		if (task_complete) {
			nvdla_dbg_fn(pdev, "task with syncpt[%d] val[%d] done",
				queue->syncpt_id, task->fence);
			nvdla_task_free_locked(task);
		}
	}
	/* put pm refcount */
	nvhost_module_idle_mult(pdev, nr_completed);

	mutex_unlock(&queue->list_lock);
}

static inline int nvdla_get_max_preaction_size(void)
{
	return (((MAX_NUM_NVDLA_PREFENCES + MAX_NUM_NVDLA_IN_TASK_STATUS) *
		sizeof(struct dla_action_opcode)) +
		(MAX_NUM_NVDLA_PREFENCES *
			sizeof(struct dla_action_semaphore)) +
		(MAX_NUM_NVDLA_IN_TASK_STATUS *
			sizeof(struct dla_action_task_status)) +
		sizeof(struct dla_action_opcode));
}

static inline int nvdla_get_max_postaction_size(void)
{
	return (((MAX_NUM_NVDLA_POSTFENCES + MAX_NUM_NVDLA_OUT_TASK_STATUS) *
		sizeof(struct dla_action_opcode)) +
		(MAX_NUM_NVDLA_POSTFENCES *
			sizeof(struct dla_action_semaphore)) +
		(MAX_NUM_NVDLA_OUT_TASK_STATUS *
			sizeof(struct dla_action_task_status)) +
		sizeof(struct dla_action_opcode));
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
	size += nvdla_get_max_preaction_size();
	size += nvdla_get_max_postaction_size();

	size = roundup(size, 8);
	size += MAX_NUM_NVDLA_BUFFERS_PER_TASK * sizeof(struct dla_mem_addr);

	/* falcon requires IOVA addr to be 256 aligned */
	size = roundup(size, SZ_256);

	return size;
}

static void nvdla_get_task_desc_memsize(size_t *dma_size, size_t *kmem_size)
{
	*dma_size = nvdla_get_task_desc_size();
	*kmem_size = nvdla_get_max_task_size();
}

static inline u8 *add_address(u8 *mem, uint64_t addr)
{
	struct dla_mem_addr *address = (struct dla_mem_addr *)mem;

	address->val = addr;

	return mem + sizeof(struct dla_mem_addr);
}

static inline u8 *add_opcode(u8 *mem, uint8_t op)
{
	struct dla_action_opcode *opcode = (struct dla_action_opcode *)mem;

	opcode->value = op;

	return mem + sizeof(struct dla_action_opcode);
}

static u8 *add_fence_action(u8 *mem, uint8_t op, uint64_t addr, uint32_t val)
{
	struct dla_action_semaphore *action;

	mem = add_opcode(mem, op);

	action = (struct dla_action_semaphore *)mem;
	action->address = addr;
	action->value = val;

	return mem + sizeof(struct dla_action_semaphore);
}

static u8 *add_status_action(u8 *mem, uint8_t op, uint64_t addr,
				uint16_t status)
{
	struct dla_action_task_status *action;

	mem = add_opcode(mem, op);

	action = (struct dla_action_task_status *)mem;
	action->address = addr;
	action->status = status;

	return mem + sizeof(struct dla_action_task_status);
}

static u8 *add_gos_action(u8 *mem, uint8_t op, uint8_t index, uint16_t offset,
				uint32_t value)
{
	struct dla_action_gos *action;

	mem = add_opcode(mem, op);

	action = (struct dla_action_gos *)mem;
	action->index = index;
	action->offset = offset;
	action->value = value;

	return mem + sizeof(struct dla_action_gos);
}

static int nvdla_map_task_memory(struct nvdla_task *task)
{
	int jj;
	int err = 0;
	size_t offset;
	struct nvhost_buffers *buffers = task->buffers;
	struct platform_device *pdev = task->queue->pool->pdev;
	struct dla_task_descriptor *task_desc = task->task_desc;
	u8 *next;

	nvdla_dbg_fn(pdev, "");

	/* get address list offset */
	offset = task_desc->postactions +
	   sizeof(struct dla_action_list) + nvdla_get_max_preaction_size() +
	   sizeof(struct dla_action_list) + nvdla_get_max_postaction_size();
	offset = roundup(offset, 8);
	nvdla_dbg_fn(pdev, "addresslist offset is[%zu]", offset);

	/* get task desc address list to update list from kernel */
	next = (u8 *)task_desc + offset;

	/* send address lists task desc dma to engine */
	task_desc->address_list = (uint64_t)((u8 *)task->task_desc_pa + offset);
	task_desc->num_addresses = task->num_addresses;

	/* update address list with all dma */
	for (jj = 0; jj < task->num_addresses; jj++) {
		dma_addr_t dma_addr;
		size_t dma_size;
		err = -EFAULT;

		nvdla_dbg_info(pdev, "count[%d] handle[%u] offset[%u]",
				jj,
				task->memory_handles[jj].handle,
				task->memory_handles[jj].offset);

		if (!task->memory_handles[jj].handle)
			goto fail_to_pin_mem;

		task->memory_dmabuf[jj] =
			dma_buf_get(task->memory_handles[jj].handle);
		if (IS_ERR_OR_NULL(task->memory_dmabuf[jj])) {
			task->memory_dmabuf[jj] = NULL;
			err = -EFAULT;
			nvdla_dbg_err(pdev, "fail to get buf");
			goto fail_to_pin_mem;
		}

		err = nvhost_buffer_submit_pin(buffers,
				&task->memory_dmabuf[jj],
				1, &dma_addr, &dma_size);
		if (err) {
			nvdla_dbg_err(pdev, "fail to pin address list");
			goto fail_to_pin_mem;
		}
		next = add_address(next,
			dma_addr + task->memory_handles[jj].offset);
	}

fail_to_pin_mem:
	return err;
}

static int nvdla_fill_postactions(struct nvdla_task *task)
{
	struct dla_task_descriptor *task_desc = task->task_desc;
	struct nvhost_buffers *buffers = task->buffers;
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	struct dla_action_list *postactionl;
	uint16_t postactionlist_of;
	u8 *next, *start;
	void *mem;
	int i, j = 0;

	/* update postaction list offset */
	postactionlist_of = task_desc->postactions +
		sizeof(struct dla_action_list) + nvdla_get_max_preaction_size();

	start = next = (u8 *)task_desc + postactionlist_of;

	/* fill output task status */
	for (j = 0; j < task->num_out_task_status; j++) {
		dma_addr_t dma_addr;
		size_t dma_size;

		nvdla_dbg_info(pdev, "i[%d] h[%u] o[%u] status[%d]",
					j,
					task->out_task_status[j].handle,
					task->out_task_status[j].offset,
					task->out_task_status[j].status);

			task->out_task_status_dmabuf[j] =
				dma_buf_get(task->out_task_status[j].handle);
			if (IS_ERR_OR_NULL(task->out_task_status_dmabuf[j])) {
				task->out_task_status_dmabuf[j] = NULL;
				nvdla_dbg_err(pdev, "fail to get buf");
				break;
			}

			if (nvhost_buffer_submit_pin(buffers,
					&task->out_task_status_dmabuf[j],
					1, &dma_addr, &dma_size))
				break;

			next = add_status_action(next, POSTACTION_TASK_STATUS,
				dma_addr + task->out_task_status[j].offset,
				task->out_task_status[j].status);
	}

	/* fill all postactions */
	for (i = 0; i < task->num_postfences; i++) {

		/* update action */
		switch (task->postfences[i].type) {
		case NVDLA_FENCE_TYPE_SYNCPT:
		case NVDLA_FENCE_TYPE_SYNC_FD: {
			dma_addr_t syncpt_addr;
			u32 gos_id, gos_offset;

			/* update GoS backing if available  */
			if (nvdla_dev->is_gos_enabled &&
			    !nvhost_syncpt_get_gos(pdev, queue->syncpt_id,
					&gos_id, &gos_offset)) {
				u32 max;

				/* send incremented max */
				max = nvhost_syncpt_read_maxval(pdev,
					queue->syncpt_id);
				nvdla_dbg_info(pdev, "post i:%d syncpt:[%u] gos_id[%u] gos_offset[%u] val[%u]",
					i, queue->syncpt_id, gos_id,
					gos_offset, max + 1);
				next = add_gos_action(next, POSTACTION_GOS,
					gos_id, gos_offset, max + 1);
			}

			/* For postaction also update MSS addr */
			syncpt_addr = nvhost_syncpt_address(pdev,
					queue->syncpt_id);
			next = add_fence_action(next, POSTACTION_SEM,
					syncpt_addr, 1);

			nvdla_dbg_info(pdev, "post i:%d syncpt:[%u] mss:[%pad]",
					i, queue->syncpt_id, &syncpt_addr);
			break;
		}
		case NVDLA_FENCE_TYPE_TS_SEMAPHORE: {
			dma_addr_t dma_addr;
			size_t dma_size;

			nvdla_dbg_info(pdev, "POSTTS i:%d semh:%u semo:%u v:%d",
					i,
					task->postfences[i].sem_handle,
					task->postfences[i].sem_offset,
					task->postfences[i].sem_val);

			/* TS SEMAPHORE just has extra memory bytes allocated
			 * to store TS as compared default semaphore.
			 * override action/opecode type here.
			 */
			task->postfences_sem_dmabuf[i] =
				dma_buf_get(task->postfences[i].sem_handle);
			if (IS_ERR_OR_NULL(task->postfences_sem_dmabuf[i])) {
				task->postfences_sem_dmabuf[i] = NULL;
				nvdla_dbg_err(pdev, "fail to get buf");
				break;
			}

			if (nvhost_buffer_submit_pin(buffers,
					&task->postfences_sem_dmabuf[i],
					1, &dma_addr, &dma_size))
				break;

			next = add_fence_action(next, POSTACTION_TS_SEM,
				dma_addr + task->postfences[i].sem_offset,
				task->postfences[i].sem_val);
			break;
		}
		case NVDLA_FENCE_TYPE_SEMAPHORE: {
			dma_addr_t dma_addr;
			size_t dma_size;

			nvdla_dbg_info(pdev, "POST i:%d semh:%u semo:%u v:%d",
					i,
					task->postfences[i].sem_handle,
					task->postfences[i].sem_offset,
					task->postfences[i].sem_val);

			task->postfences_sem_dmabuf[i] =
				dma_buf_get(task->postfences[i].sem_handle);
			if (IS_ERR_OR_NULL(task->postfences_sem_dmabuf[i])) {
				task->postfences_sem_dmabuf[i] = NULL;
				nvdla_dbg_err(pdev, "fail to get buf");
				break;
			}

			if (nvhost_buffer_submit_pin(buffers,
					&task->postfences_sem_dmabuf[i],
					1, &dma_addr, &dma_size))
				break;

			next = add_fence_action(next, POSTACTION_SEM,
				dma_addr + task->postfences[i].sem_offset,
				task->postfences[i].sem_val);
			break;
		}
		default:
			nvdla_dbg_err(pdev, "Invalid postfence sync type[%d]",
				task->postfences[i].type);
			return -EINVAL;
		}
	}

	/* update end of action list */
	next = add_opcode(next, POSTACTION_TERMINATE);

	mem = (char *)task_desc + task_desc->postactions;
	postactionl = (struct dla_action_list *)mem;
	postactionl->offset = postactionlist_of;
	postactionl->size = next - start;

	return 0;
}

static int nvdla_fill_preactions(struct nvdla_task *task)
{
	struct dla_task_descriptor *task_desc = task->task_desc;
	struct nvhost_buffers *buffers = task->buffers;
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	struct nvhost_master *host = nvhost_get_host(pdev);
	struct nvhost_syncpt *sp = &host->syncpt;
	struct dla_action_list *preactionl;
	uint16_t preactionlist_of;
	u8 *next, *start;
	void *mem;
	int i, j;

	/* preaction list offset update */
	preactionlist_of = task_desc->postactions +
					sizeof(struct dla_action_list);

	start = next = (u8 *)task_desc + preactionlist_of;

	/* fill all preactions */
	for (i = 0; i < task->num_prefences; i++) {

		switch (task->prefences[i].type) {
		case NVDLA_FENCE_TYPE_SYNC_FD: {
			struct sync_fence *f;
			struct sync_pt *pt;
			u32 id, thresh, j;

			f = nvhost_sync_fdget(task->prefences[i].sync_fd);
			if (!f) {
				nvdla_dbg_err(pdev, "failed to get sync fd");
				break;
			}

			j = id = thresh = 0;

			for (j = 0; j < f->num_fences; j++) {
				u32 gos_id, gos_offset;

				pt = sync_pt_from_fence(f->cbs[j].sync_pt);
				id = nvhost_sync_pt_id(pt);
				thresh = nvhost_sync_pt_thresh(pt);

				if (!id ||
				     !nvhost_syncpt_is_valid_hw_pt(sp, id)) {
					nvdla_dbg_err(pdev, "Invalid sync_fd");
					sync_fence_put(f);
					break;
				}

				/* check if GoS backing available */
				if (nvdla_dev->is_gos_enabled &&
				    !nvhost_syncpt_get_gos(pdev, id, &gos_id,
						&gos_offset)) {
					nvdla_dbg_info(pdev, "pre i:%d syncfd_pt:[%u] gos_id[%u] gos_offset[%u] val[%u]",
						i, id, gos_id,
						gos_offset, thresh);
					next = add_gos_action(next,
						PREACTION_GOS_GE,
						gos_id, gos_offset, thresh);
				} else {
					dma_addr_t syncpt_addr;

					nvdla_dbg_info(pdev, "pre i:%d GoS missing for syncfd [%d]",
							i, id);
					syncpt_addr = nvhost_syncpt_address(pdev,
							id);
					nvdla_dbg_info(pdev, "pre i:%d syncfd_pt:[%u] mss_dma_addr[%pad]",
						i, id, &syncpt_addr);
					next = add_fence_action(next, PREACTION_SEM_GE,
							syncpt_addr, thresh);
				}
			}
			break;
		}
		case NVDLA_FENCE_TYPE_SYNCPT: {
			u32 gos_id, gos_offset;

			nvdla_dbg_info(pdev, "i[%d] id[%d] val[%d]",
					i,
					task->prefences[i].syncpoint_index,
					task->prefences[i].syncpoint_value);

			if (nvdla_dev->is_gos_enabled &&
			    !nvhost_syncpt_get_gos(pdev,
				task->prefences[i].syncpoint_index, &gos_id,
						&gos_offset)) {
				nvdla_dbg_info(pdev, "pre i:%d syncpt:[%u] gos_id[%u] gos_offset[%u] val[%u]",
					i, task->prefences[i].syncpoint_index,
					gos_id, gos_offset,
					task->prefences[i].syncpoint_value);
				next = add_gos_action(next, PREACTION_GOS_GE,
					gos_id, gos_offset,
					task->prefences[i].syncpoint_value);
			} else {
				dma_addr_t syncpt_addr;

				nvdla_dbg_info(pdev, "pre i:%d GoS missing", i);

				syncpt_addr = nvhost_syncpt_address(pdev,
					task->prefences[i].syncpoint_index);
				nvdla_dbg_info(pdev, "pre i:%d syncpt:[%u] dma_addr[%pad]",
					i,
					task->prefences[i].syncpoint_index,
					&syncpt_addr);

				next = add_fence_action(next, PREACTION_SEM_GE,
					syncpt_addr,
					task->prefences[i].syncpoint_value);
			}
			break;
		}
		case NVDLA_FENCE_TYPE_SEMAPHORE: {
			dma_addr_t dma_addr;
			size_t dma_size;

			nvdla_dbg_info(pdev, "i[%d] semh[%u] semo[%u] val[%d]",
					i,
					task->prefences[i].sem_handle,
					task->prefences[i].sem_offset,
					task->prefences[i].sem_val);

			task->prefences_sem_dmabuf[i] =
				dma_buf_get(task->prefences[i].sem_handle);
			if (IS_ERR_OR_NULL(task->prefences_sem_dmabuf[i])) {
				task->prefences_sem_dmabuf[i] = NULL;
				nvdla_dbg_err(pdev, "fail to get buf");
				break;
			}

			if (nvhost_buffer_submit_pin(buffers,
					&task->prefences_sem_dmabuf[i],
					1, &dma_addr, &dma_size))
				break;

			next = add_fence_action(next, PREACTION_SEM_GE,
				dma_addr + task->prefences[i].sem_offset,
				task->prefences[i].sem_val);
			break;
		}
		default:
			nvdla_dbg_err(pdev, "Invalid sync_type[%d]",
				task->prefences[i].type);
			return -EINVAL;
		}
	}

	/* fill input status after filling sem/synpt/gos */
	for (j = 0; j < task->num_in_task_status; j++) {
		dma_addr_t dma_addr;
		size_t dma_size;

		nvdla_dbg_info(pdev, "i[%d] h[%u] o[%u] status[%d]",
					j,
					task->in_task_status[j].handle,
					task->in_task_status[j].offset,
					task->in_task_status[j].status);

			task->in_task_status_dmabuf[j] =
				dma_buf_get(task->in_task_status[j].handle);
			if (IS_ERR_OR_NULL(task->in_task_status_dmabuf[j])) {
				task->in_task_status_dmabuf[j] = NULL;
				nvdla_dbg_err(pdev, "fail to get buf");
				break;
			}

			if (nvhost_buffer_submit_pin(buffers,
					&task->in_task_status_dmabuf[j],
					1, &dma_addr, &dma_size))
				break;

			next = add_status_action(next, PREACTION_TASK_STATUS,
				dma_addr + task->in_task_status[j].offset,
				task->in_task_status[j].status);
	}

	/* update end of action list */
	next = add_opcode(next, PREACTION_TERMINATE);

	/* actually update lists data */
	mem = (char *)task_desc + task_desc->preactions;
	preactionl = (struct dla_action_list *)mem;
	preactionl->offset = preactionlist_of;
	preactionl->size = next - start;

	return 0;
}

int nvdla_fill_task_desc(struct nvdla_task *task)
{
	int err;
	struct dla_task_descriptor *task_desc;
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_fn(pdev, "");

	/* update task desc fields */
	task_desc = task->task_desc;
	task_desc->version = DLA_DESCRIPTOR_VERSION;
	task_desc->engine_id = DLA_ENGINE_ID;
	task_desc->size = nvdla_get_task_desc_size();

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
	nvdla_dbg_info(pdev, "version[%d]", task_desc->version);
	nvdla_dbg_info(pdev, "engine_id[%d]", task_desc->engine_id);
	nvdla_dbg_info(pdev, "task desc size[%u]", task_desc->size);
	nvdla_dbg_info(pdev, "task desc sequence[%u]", task_desc->sequence);

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
	return err;
}

/* Queue management API */
static int nvdla_queue_submit(struct nvhost_queue *queue, void *in_task)
{
	struct nvdla_task *task = (struct nvdla_task *)in_task;
	struct nvdla_task *last_task = NULL;
	struct platform_device *pdev = queue->pool->pdev;
	struct nvdla_cmd_data cmd_data;
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

		nvdla_dbg_info(pdev, "last task[%p] last_task_desc_pa[%llu]",
				last_task, task->task_desc_pa);
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
	method_data = ALIGNED_DMA(task->task_desc_pa);

	/* register notifier with fence */
	err = nvhost_intr_register_notifier(pdev, queue->syncpt_id,
		task->fence, nvdla_queue_update, queue);
	if (err)
		goto fail_to_register;

	/* Pass fence as through 0th postfences */
	task->postfences[0].syncpoint_index = queue->syncpt_id;
	task->postfences[0].syncpoint_value = task->fence;

	/* prepare command */
	cmd_data.method_id = method_id;
	cmd_data.method_data = method_data;
	cmd_data.wait = true;

	/* submit task to engine */
	err = nvdla_send_cmd(pdev, &cmd_data);
	if (err) {
		nvdla_task_syncpt_reset(task->sp, queue->syncpt_id,
				task->fence);
		nvdla_dbg_err(pdev, "task[%p] submit failed", task);
	}

fail_to_register:
	mutex_unlock(&queue->list_lock);

	return err;
}

int nvdla_set_queue_state(struct nvhost_queue *queue, int cmd)
{
	struct platform_device *pdev = queue->pool->pdev;
	struct nvdla_cmd_data cmd_data;
	int err;

	nvdla_dbg_fn(pdev, "");

	if ((cmd != DLA_CMD_QUEUE_SUSPEND) &&
		(cmd != DLA_CMD_QUEUE_RESUME)) {
		nvdla_dbg_err(pdev, "invalid cmd %d", cmd);
		return -EINVAL;
	}

	/* get pm refcount */
	err = nvhost_module_busy(pdev);
	if (err) {
		nvdla_dbg_err(pdev, "failed to poweron, err: %d", err);
		goto fail_to_poweron;
	}

	/* prepare command */
	cmd_data.method_id = cmd;
	cmd_data.method_data = queue->id;
	cmd_data.wait = true;

	err = nvdla_send_cmd(pdev, &cmd_data);
	if (err) {
		nvdla_dbg_err(pdev, "failed to suspend queue %d", err);
		goto fail_to_suspend;
	}

fail_to_suspend:
	nvhost_module_idle(pdev);
fail_to_poweron:
	return err;
}

static int nvdla_queue_abort(struct nvhost_queue *queue)
{
	int err;
	struct nvdla_task *t;
	struct nvdla_cmd_data cmd_data;
	struct platform_device *pdev = queue->pool->pdev;
	int retry = NVDLA_QUEUE_ABORT_TIMEOUT / NVDLA_QUEUE_ABORT_RETRY_PERIOD;

	nvdla_dbg_fn(pdev, "");

	if (list_empty(&queue->tasklist))
		return 0;

	/* get pm refcount */
	err = nvhost_module_busy(pdev);
	if (err) {
		nvdla_dbg_err(pdev, "failed to poweron, err: %d", err);
		return err;
	}

	/* prepare command */
	cmd_data.method_id = DLA_CMD_QUEUE_FLUSH;
	cmd_data.method_data = queue->id;
	cmd_data.wait = true;

	/* flush engine side queues */
	do {
		err = nvdla_send_cmd(pdev, &cmd_data);
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
	.get_task_size =  nvdla_get_task_desc_memsize,
};
