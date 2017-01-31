/*
 * PVA Task Management
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

#include <linux/delay.h>
#include <asm/ioctls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/errno.h>

#include <uapi/linux/nvhost_pva_ioctl.h>

#include "nvhost_syncpt_unit_interface.h"
#include "../drivers/staging/android/sync.h"
#include "pva.h"
#include "pva-task.h"
#include "nvhost_buffer.h"
#include "nvhost_queue.h"
#include "pva_mailbox.h"
#include "pva_queue.h"
#include "dev.h"

static void pva_task_dump(struct pva_submit_task *task)
{
	int i;

	nvhost_dbg_info("task=%p, input_scalars=(handle=%u, offset=%x), "
			"input_surfaces=%p, input_points=(handle=%u, offset=%u), "
			"input_rois=(handle=%u, offset=%u), "
			"output_scalars=(handle=%u, offset=%u), "
			"output_surfaces=%p, output_points=(handle=%u, offset=%u), "
			"output_rois=(handle=%u, offset=%u)",
			task, task->input_scalars.handle,
			task->input_scalars.offset, task->input_surfaces,
			task->input_2dpoint.handle,
			task->input_2dpoint.offset,
			task->input_rois.handle, task->input_rois.offset,
			task->output_scalars.handle,
			task->output_scalars.offset,
			task->output_surfaces,
			task->output_2dpoint.handle,
			task->output_2dpoint.offset,
			task->output_rois.handle, task->output_rois.offset);

	for (i = 0; i < task->num_prefences; i++)
		nvhost_dbg_info("prefence %d: type=%u, "
				"syncpoint_index=%u, syncpoint_value=%u, "
				"sync_fd=%u, semaphore_handle=%u, "
				"semaphore_offset=%u, semaphore_value=%u", i,
				task->prefences[i].type,
				task->prefences[i].syncpoint_index,
				task->prefences[i].syncpoint_value,
				task->prefences[i].sync_fd,
				task->prefences[i].semaphore_handle,
				task->prefences[i].semaphore_offset,
				task->prefences[i].semaphore_value);

	for (i = 0; i < task->num_postfences; i++)
		nvhost_dbg_info("postfence %d: type=%u, "
				"syncpoint_index=%u, syncpoint_value=%u, "
				"sync_fd=%u, semaphore_handle=%u, "
				"semaphore_offset=%u, semaphore_value=%u", i,
				task->postfences[i].type,
				task->postfences[i].syncpoint_index,
				task->postfences[i].syncpoint_value,
				task->postfences[i].sync_fd,
				task->postfences[i].semaphore_handle,
				task->postfences[i].semaphore_offset,
				task->postfences[i].semaphore_value);

	for (i = 0; i < task->num_input_surfaces; i++)
		nvhost_dbg_info("input surface %d: format=%llu, "
				"surface_handle=%u, surface_offset=%u, "
				"roi_handle=%u, roi_offset=%u, surface_stride=%u, "
				"line_stride=%u, depth=%u, width=%u, height=%u, "
				"layout=%u", i,
				task->input_surfaces[i].format,
				task->input_surfaces[i].surface_handle,
				task->input_surfaces[i].surface_offset,
				task->input_surfaces[i].roi_handle,
				task->input_surfaces[i].roi_offset,
				task->input_surfaces[i].surface_stride,
				task->input_surfaces[i].line_stride,
				task->input_surfaces[i].depth,
				task->input_surfaces[i].width,
				task->input_surfaces[i].height,
				task->input_surfaces[i].layout);

	for (i = 0; i < task->num_output_surfaces; i++)
		nvhost_dbg_info("output surface %d: format=%llu, "
				"surface_handle=%u, surface_offset=%u, "
				"roi_handle=%u, roi_offset=%u, surface_stride=%u,"
				"line_stride=%u, depth=%u, width=%u, height=%u, "
				"layout=%u", i,
				task->output_surfaces[i].format,
				task->output_surfaces[i].surface_handle,
				task->output_surfaces[i].surface_offset,
				task->output_surfaces[i].roi_handle,
				task->output_surfaces[i].roi_offset,
				task->output_surfaces[i].surface_stride,
				task->output_surfaces[i].line_stride,
				task->output_surfaces[i].depth,
				task->output_surfaces[i].width,
				task->output_surfaces[i].height,
				task->output_surfaces[i].layout);

	for (i = 0; i < task->num_input_task_status; i++)
		nvhost_dbg_info("input task status %d: handle=%u, offset=%u",
				i, task->input_task_status[i].handle,
				task->input_task_status[i].offset);

	for (i = 0; i < task->num_output_task_status; i++)
		nvhost_dbg_info("output task status %d: handle=%u, offset=%u",
				i, task->output_task_status[i].handle,
				task->output_task_status[i].offset);
}

static size_t pva_task_get_size(void)
{
	size_t size = 0;

	/* Add task base structure */
	size = sizeof(struct pva_task);

	/* Allocate room for input action list */
	size += sizeof(struct pva_action_list);
	size = roundup(size, sizeof(u64));

	/* Allocate room for output action list */
	size += sizeof(struct pva_action_list);
	size = roundup(size, sizeof(u64));

	/* Allocate space for input parameter list */
	size += PVA_PARAM_LAST * sizeof(struct pva_task_parameter_array);

	/* Allocate space for output parameter list */
	size += PVA_PARAM_LAST * sizeof(struct pva_task_parameter_array);

	/*
	 * Calculate space needed for input actions
	 */

	/* Pre-fences */
	size += PVA_MAX_PREFENCES *
		(1 + sizeof(struct pva_task_action_ptr));
	/* Input status checks */
	size += PVA_MAX_INPUT_STATUS *
		(1 + sizeof(struct pva_task_action_status));
	/* Action list termination */
	size += 1;
	size = roundup(size, sizeof(u64));

	/*
	 * Calculate space needed for output actions
	 */

	/* Output status writes */
	size += PVA_MAX_OUTPUT_STATUS *
		(1 + sizeof(struct pva_task_action_status));
	/* Postfences requested by userspace */
	size += PVA_MAX_POSTFENCES *
		(1 + sizeof(struct pva_task_action_ptr));
	/* Syncpoint increment */
	size += sizeof(struct pva_task_action_ptr);
	/* Action list termination */
	size += 1;
	size = roundup(size, sizeof(u64));

	/* Add space required for the surfaces */
	size += PVA_MAX_INPUT_SURFACES * sizeof(struct pva_task_surface);
	size += PVA_MAX_OUTPUT_SURFACES * sizeof(struct pva_task_surface);

	return size;
}


static void pva_task_get_memsize(size_t *dma_size, size_t *kmem_size)
{
	*dma_size = pva_task_get_size();
	*kmem_size = sizeof(struct pva_submit_task);
}

static void pva_task_unpin_mem(struct pva_submit_task *task)
{
	int i;

#define UNPIN_MEMORY(dst_name)						\
	do {								\
		if ((((dst_name).dmabuf) != NULL) &&			\
				((dst_name).dma_addr != 0)) {		\
			nvhost_buffer_submit_unpin(task->buffers,	\
				&((dst_name).dmabuf), 1);		\
			dma_buf_put((dst_name).dmabuf);			\
		}							\
	} while (0)

	for (i = 0; i < task->num_input_surfaces; i++) {
		UNPIN_MEMORY(task->input_surfaces_ext[i]);
		UNPIN_MEMORY(task->input_surface_rois_ext[i]);
	}


	for (i = 0; i < task->num_output_surfaces; i++) {
		UNPIN_MEMORY(task->output_surfaces_ext[i]);
		UNPIN_MEMORY(task->output_surface_rois_ext[i]);
	}

	for (i = 0; i < task->num_prefences; i++) {
		if ((task->prefences[i].type == PVA_FENCE_TYPE_SEMAPHORE)
			&& task->prefences[i].semaphore_handle)
			UNPIN_MEMORY(task->prefences_sema_ext[i]);
	}

	for (i = 0; i < task->num_postfences; i++) {
		if ((task->postfences[i].type == PVA_FENCE_TYPE_SEMAPHORE)
			&& task->postfences[i].semaphore_handle)
			UNPIN_MEMORY(task->postfences_sema_ext[i]);
	}

	for (i = 0; i < task->num_input_task_status; i++) {
		if (task->input_task_status[i].handle) {
			UNPIN_MEMORY(task->input_task_status_ext[i]);
		}
	}

	for (i = 0; i < task->num_output_task_status; i++) {
		if (task->output_task_status[i].handle) {
			UNPIN_MEMORY(task->output_task_status_ext[i]);
		}
	}

	UNPIN_MEMORY(task->input_scalars_ext);
	UNPIN_MEMORY(task->input_rois_ext);
	UNPIN_MEMORY(task->input_2dpoint_ext);
	UNPIN_MEMORY(task->output_scalars_ext);
	UNPIN_MEMORY(task->output_rois_ext);
	UNPIN_MEMORY(task->output_2dpoint_ext);

#undef UNPIN_MEMORY
}

static int pva_task_pin_mem(struct pva_submit_task *task)
{
	const u32 cvsram_base = 0x50000000;
	const u32 cvsram_sz = 0x400000;
	int err;
	int i;

#define PIN_MEMORY(dst_name, dmabuf_fd)					\
	do {								\
		if (!(dmabuf_fd)) {					\
			err = -EFAULT;					\
			goto err_map_handle;				\
		}							\
		((dst_name).dmabuf) = dma_buf_get(dmabuf_fd);		\
		if (IS_ERR_OR_NULL((dst_name).dmabuf)) {		\
			(dst_name).dmabuf = NULL;			\
			err = -EFAULT;					\
			goto err_map_handle;				\
		}							\
		err = nvhost_buffer_submit_pin(task->buffers,		\
				&(dst_name).dmabuf, 1,			\
				&(dst_name).dma_addr,			\
				&(dst_name).size);			\
		if (err < 0)						\
			goto err_map_handle;				\
	} while (0)

	/* Pin input surfaces */
	for (i = 0; i < task->num_input_surfaces; i++) {
		/* HACK: nvmap doesn't support CVNAS yet */
		if (task->input_surfaces[i].surface_handle == 0) {
			u32 offset = task->input_surfaces[i].surface_offset;

			if (offset > cvsram_sz) {
				err = -EINVAL;
				goto err_map_handle;
			}

			task->input_surfaces_ext[i].dma_addr = cvsram_base;
			task->input_surfaces_ext[i].size = cvsram_sz - offset;
			task->input_surfaces_ext[i].cvsram = true;
		} else {
			PIN_MEMORY(task->input_surfaces_ext[i],
				task->input_surfaces[i].surface_handle);
		}

		if (task->input_surfaces[i].roi_handle)
			PIN_MEMORY(task->input_surface_rois_ext[i],
				task->input_surfaces[i].roi_handle);
	}

	/* ...and then output surfaces */
	for (i = 0; i < task->num_output_surfaces; i++) {
		/* HACK: nvmap doesn't support CVNAS yet */
		if (task->output_surfaces[i].surface_handle == 0) {
			u32 offset = task->output_surfaces[i].surface_offset;

			if (offset > cvsram_sz) {
				err = -EINVAL;
				goto err_map_handle;
			}

			task->output_surfaces_ext[i].dma_addr = cvsram_base;
			task->output_surfaces_ext[i].size = cvsram_sz - offset;
			task->output_surfaces_ext[i].cvsram = true;
		} else {
			PIN_MEMORY(task->output_surfaces_ext[i],
				task->output_surfaces[i].surface_handle);
		}

		if (task->output_surfaces[i].roi_handle)
			PIN_MEMORY(task->output_surface_rois_ext[i],
				task->output_surfaces[i].roi_handle);
	}

	/* check fence semaphore_type before memory pin */
	for (i = 0; i < task->num_prefences; i++) {
		if ((task->prefences[i].type == PVA_FENCE_TYPE_SEMAPHORE)
			&& task->prefences[i].semaphore_handle) {
			PIN_MEMORY(task->prefences_sema_ext[i],
				task->prefences[i].semaphore_handle);
		}
	}

	for (i = 0; i < task->num_postfences; i++) {
		if ((task->postfences[i].type == PVA_FENCE_TYPE_SEMAPHORE)
			&& task->postfences[i].semaphore_handle) {
			PIN_MEMORY(task->postfences_sema_ext[i],
				task->postfences[i].semaphore_handle);
		}
	}

	/* Pin the input and output action status */
	for (i = 0; i < task->num_input_task_status; i++) {
		if (task->input_task_status[i].handle) {
			PIN_MEMORY(task->input_task_status_ext[i],
				task->input_task_status[i].handle);
		}
	}

	for (i = 0; i < task->num_output_task_status; i++) {
		if (task->output_task_status[i].handle) {
			PIN_MEMORY(task->output_task_status_ext[i],
				task->output_task_status[i].handle);
		}
	}

	/* Pin rest */
	if (task->input_scalars.handle)
		PIN_MEMORY(task->input_scalars_ext,
			task->input_scalars.handle);

	if (task->input_rois.handle)
		PIN_MEMORY(task->input_rois_ext, task->input_rois.handle);

	if (task->input_2dpoint.handle)
		PIN_MEMORY(task->input_2dpoint_ext,
			task->input_2dpoint.handle);

	if (task->output_scalars.handle)
		PIN_MEMORY(task->output_scalars_ext,
			task->output_scalars.handle);

	if (task->output_rois.handle)
		PIN_MEMORY(task->output_rois_ext, task->output_rois.handle);

	if (task->output_2dpoint.handle)
		PIN_MEMORY(task->output_2dpoint_ext,
			task->output_2dpoint.handle);

#undef PIN_MEMORY

	return 0;

err_map_handle:
	pva_task_unpin_mem(task);
	return err;
}

static void pva_task_write_surfaces(struct pva_task_surface *hw_surface,
		struct pva_surface *surface,
		struct pva_parameter_ext *surface_ext,
		struct pva_parameter_ext *roi_ext,
		unsigned int count)
{
	int i;

	for (i = 0; i < count; i++) {
		hw_surface[i].address = surface_ext[i].dma_addr +
			surface[i].surface_offset;
		hw_surface[i].surface_size = surface_ext[i].size;
		hw_surface[i].roi_addr = roi_ext[i].dma_addr +
			surface[i].roi_offset;
		hw_surface[i].roi_size = roi_ext[i].size;
		hw_surface[i].format = surface[i].format;
		hw_surface[i].width = surface[i].width;
		hw_surface[i].height = surface[i].height;
		hw_surface[i].line_stride = surface[i].line_stride;
		hw_surface[i].plane_stride = surface[i].surface_stride;
		hw_surface[i].num_planes = surface[i].depth;
		hw_surface[i].layout = surface[i].layout;
		hw_surface[i].block_height_log2 = surface[i].block_height_log2;

		/* Set bit 39 for block linear surfaces in the address field.
		*  This bit is used for indicating that memory subsystem should
		*  convert the block linear format into common block linear format
		*  that is used by other engines in Tegra. Thebit in itself is
		*  dropped before making the address translation in SMMU.
		*/
		if (surface[i].layout == PVA_TASK_SURFACE_LAYOUT_BLOCK_LINEAR)
			hw_surface[i].address |= PVA_BIT64(39);

		/* Only DRAM is supported currently */
		hw_surface[i].memory = surface_ext[i].cvsram;
	}
}

static inline int pva_task_write_atomic_op(u8 *base, u8 action)
{
	*base = action;

	return 1;
}

static inline int pva_task_write_ptr_op(u8 *base, u8 action, u64 addr, u32 val)
{
	int i = 0;

	base[i++] = action;
	base[i++] = (u8)((addr >> 0) & 0xff);
	base[i++] = (u8)((addr >> 8) & 0xff);
	base[i++] = (u8)((addr >> 16) & 0xff);
	base[i++] = (u8)((addr >> 24) & 0xff);
	base[i++] = (u8)((addr >> 32) & 0xff);
	base[i++] = (u8)((addr >> 40) & 0xff);
	base[i++] = (u8)((addr >> 48) & 0xff);
	base[i++] = (u8)((addr >> 56) & 0xff);
	base[i++] = (u8)((val >> 0) & 0xff);
	base[i++] = (u8)((val >> 8) & 0xff);
	base[i++] = (u8)((val >> 16) & 0xff);
	base[i++] = (u8)((val >> 24) & 0xff);

	return i;
}

static int pva_task_write_preactions(struct pva_submit_task *task,
		struct pva_action_list *hw_preaction_list,
		u16 *offset)
{
	u8 *hw_preactions = (void *)((u8 *)task->va + *offset);
	int i = 0, j = 0, ptr = 0;

	/* Add waits to preactions list */
	for (i = 0; i < task->num_prefences; i++) {
		struct pva_fence *fence = task->prefences + i;

		switch (fence->type) {
		case PVA_FENCE_TYPE_SYNCPT: {
			dma_addr_t syncpt_addr = nvhost_syncpt_gos_address(
							task->pva->pdev,
							fence->syncpoint_index);
			if (!syncpt_addr)
				syncpt_addr = nvhost_syncpt_address(
							task->pva->pdev,
							fence->syncpoint_index);

			ptr += pva_task_write_ptr_op(&hw_preactions[ptr],
				TASK_ACT_PTR_BLK_GTREQL, syncpt_addr,
				fence->syncpoint_value);
			break;
		}
		case PVA_FENCE_TYPE_SEMAPHORE:
		case PVA_FENCE_TYPE_SEMAPHORE_TS:{
			ptr += pva_task_write_ptr_op(&hw_preactions[ptr],
				TASK_ACT_PTR_BLK_GTREQL,
				task->prefences_sema_ext[i].dma_addr  +
					fence->semaphore_offset,
				fence->semaphore_value);
			break;
		}
		case PVA_FENCE_TYPE_SYNC_FD: {
			int thresh, id;
			dma_addr_t syncpt_addr;
			struct sync_fence *syncfd_fence;
			struct sync_pt *pt;
			struct nvhost_master *host = nvhost_get_host(
							task->pva->pdev);
			struct nvhost_syncpt *sp = &host->syncpt;

			if (!fence->sync_fd)
				break;

			syncfd_fence = nvhost_sync_fdget(fence->sync_fd);
			if (!syncfd_fence)
				break;

			for (j = 0; j < syncfd_fence->num_fences; j++) {
				pt = sync_pt_from_fence(
					syncfd_fence->cbs[j].sync_pt);
				if (!pt)
					break;

				id = nvhost_sync_pt_id(pt);
				thresh = nvhost_sync_pt_thresh(pt);

				/* validate the synpt ids */
				if (!id ||
				!nvhost_syncpt_is_valid_hw_pt(sp, id)) {
					sync_fence_put(syncfd_fence);
					break;
				}

				if (nvhost_syncpt_is_expired(sp,
							id, thresh))
					continue;

				syncpt_addr = nvhost_syncpt_gos_address(
							task->pva->pdev, id);
				if (!syncpt_addr)
					syncpt_addr = nvhost_syncpt_address(
							task->pva->pdev, id);

				ptr += pva_task_write_ptr_op(
						&hw_preactions[ptr],
						TASK_ACT_PTR_BLK_GTREQL,
						syncpt_addr, thresh);

			}
			break;
		}
		default:
			return -ENOSYS;
		}
	}

	/* Perform input status checks */
	for (i = 0; i < task->num_input_task_status; i++) {
		struct pva_status_handle *input_status =
					task->input_task_status + i;
		dma_addr_t input_status_addr =
				task->input_task_status_ext[i].dma_addr  +
				input_status->offset;

		ptr += pva_task_write_ptr_op(
					&hw_preactions[ptr],
					TASK_ACT_READ_STATUS,
					input_status_addr, 0);
	}

	ptr += pva_task_write_atomic_op(&hw_preactions[ptr],
		TASK_ACT_TERMINATE);

	/* Store the preaction list */
	hw_preaction_list->offset = *offset;
	hw_preaction_list->length = ptr;

	/* Mark this part of task memory as used */
	*offset += ptr;
	*offset = roundup(*offset, sizeof(u64));

	return 0;
}

static void pva_task_write_postactions(struct pva_submit_task *task,
		struct pva_action_list *hw_postaction_list,
		u16 *offset)
{
	dma_addr_t syncpt_addr = nvhost_syncpt_address(task->pva->pdev,
				task->queue->syncpt_id);
	dma_addr_t syncpt_gos_addr = nvhost_syncpt_gos_address(task->pva->pdev,
				task->queue->syncpt_id);
	u8 *hw_postactions = (void *)((u8 *)task->va + *offset);
	int ptr = 0, i = 0;
	struct platform_device *host1x_pdev =
			to_platform_device(task->pva->pdev->dev.parent);
	u32 thresh;

	/* Write Output action status */
	for (i = 0; i < task->num_output_task_status; i++) {
		struct pva_status_handle *output_status =
					task->output_task_status + i;
		dma_addr_t output_status_addr =
				task->output_task_status_ext[i].dma_addr  +
				output_status->offset;

		ptr += pva_task_write_ptr_op(
					&hw_postactions[ptr],
					TASK_ACT_WRITE_STATUS,
					output_status_addr, 0);
	}

	/* Add postactions list for semaphore */
	for (i = 0; i < task->num_postfences; i++) {
		struct pva_fence *fence = task->postfences + i;

		if (fence->type == PVA_FENCE_TYPE_SEMAPHORE) {
			ptr += pva_task_write_ptr_op(&hw_postactions[ptr],
				TASK_ACT_PTR_WRITE_VAL,
				task->postfences_sema_ext[i].dma_addr  +
					fence->semaphore_offset,
				fence->semaphore_value);
		} else if (fence->type == PVA_FENCE_TYPE_SEMAPHORE_TS) {
			/*
			 * Timestamp will be filled by ucode hence making the
			 * place holder for timestamp size, sizeof(u64).
			 */
			ptr = ptr + sizeof(u64) +
				pva_task_write_ptr_op(&hw_postactions[ptr],
				TASK_ACT_PTR_WRITE_VAL_TS,
				task->postfences_sema_ext[i].dma_addr  +
					fence->semaphore_offset,
				fence->semaphore_value);
		}
	}

	/* Make a syncpoint increment */
	if (syncpt_gos_addr) {
		thresh = nvhost_syncpt_read_maxval(host1x_pdev,
				task->queue->syncpt_id) + 1;
		ptr += pva_task_write_ptr_op(&hw_postactions[ptr],
			TASK_ACT_PTR_WRITE_VAL, syncpt_gos_addr, thresh);
	}
	ptr += pva_task_write_ptr_op(&hw_postactions[ptr],
		TASK_ACT_PTR_WRITE_VAL, syncpt_addr, 1);
	ptr += pva_task_write_atomic_op(&hw_postactions[ptr],
		TASK_ACT_TERMINATE);

	/* Store the postaction list */
	hw_postaction_list->offset = *offset;
	hw_postaction_list->length = ptr;

	/* Mark this part of task memory as used */
	*offset += ptr;
	*offset = roundup(*offset, sizeof(u64));
}

static void pva_task_write_output_surfaces(struct pva_submit_task *task,
		struct pva_task_parameter_array *hw_output_parameters,
		u32 *num_output_parameters, u16 *offset)
{
	struct pva_task_surface *hw_output_surfaces;
	struct pva_task_parameter_desc *hw_output_surface_desc;

	if (task->num_output_surfaces == 0) {
		return;
	}

	/* Write parameter descriptor */
	hw_output_parameters[*num_output_parameters].address =
			task->dma_addr + *offset;
	hw_output_parameters[*num_output_parameters].type =
			PVA_PARAM_SURFACE_LIST;
	hw_output_parameters[*num_output_parameters].size =
			sizeof(*hw_output_surface_desc) +
			sizeof(*hw_output_surfaces) *
			task->num_output_surfaces;
	*num_output_parameters = *num_output_parameters + 1;

	/* Write the surface descriptor base information */
	hw_output_surface_desc = (void *)((u8 *)task->va + *offset);
	hw_output_surface_desc->num_parameters = task->num_output_surfaces;
	hw_output_surface_desc->reserved = 0;
	*offset = *offset + sizeof(*hw_output_surface_desc);

	/* Get surface base address */
	hw_output_surfaces = (void *)((u8 *)task->va + *offset);

	/* Write the output surfaces */
	pva_task_write_surfaces(hw_output_surfaces,
			task->output_surfaces,
			task->output_surfaces_ext,
			task->output_surface_rois_ext,
			task->num_output_surfaces);

	/* Track the offset change */
	*offset = *offset + sizeof(*hw_output_surfaces) *
		PVA_MAX_OUTPUT_SURFACES;
}

static void pva_task_write_input_surfaces(struct pva_submit_task *task,
		struct pva_task_parameter_array *hw_input_parameters,
		u32 *num_input_parameters, u16 *offset)
{
	struct pva_task_surface *hw_input_surfaces;
	struct pva_task_parameter_desc *hw_input_surface_desc;

	if (task->num_input_surfaces == 0) {
		return;
	}

	/* Write parameter descriptor */
	hw_input_parameters[*num_input_parameters].address =
			task->dma_addr + *offset;
	hw_input_parameters[*num_input_parameters].type =
			PVA_PARAM_SURFACE_LIST;
	hw_input_parameters[*num_input_parameters].size =
			sizeof(*hw_input_surface_desc) +
			sizeof(*hw_input_surfaces) *
			task->num_input_surfaces;
	*num_input_parameters = *num_input_parameters + 1;

	/* Write the surface descriptor base information */
	hw_input_surface_desc = (void *)((u8 *)task->va + *offset);
	hw_input_surface_desc->num_parameters = task->num_input_surfaces;
	hw_input_surface_desc->reserved = 0;
	*offset = *offset + sizeof(*hw_input_surface_desc);

	/* Get surface base address */
	hw_input_surfaces = (void *)((u8 *)task->va + *offset);

	/* Write the input surfaces */
	pva_task_write_surfaces(hw_input_surfaces,
			task->input_surfaces,
			task->input_surfaces_ext,
			task->input_surface_rois_ext,
			task->num_input_surfaces);

	/* Track the offset change */
	*offset = *offset + sizeof(*hw_input_surfaces) *
			PVA_MAX_INPUT_SURFACES;
}

static int pva_task_write(struct pva_submit_task *task, bool atomic)
{
	struct pva_task_parameter_array *hw_input_parameters;
	struct pva_task_parameter_array *hw_output_parameters;
	struct pva_action_list *hw_postaction_list;
	struct pva_action_list *hw_preaction_list;
	struct pva_task *hw_task;
	u32 num_input_parameters = 0;
	u32 num_output_parameters = 0;
	u16 offset = 0;
	int err;
	int i;

	/* Task start from the memory base */
	hw_task = task->va;
	offset += sizeof(*hw_task);

	/* Allocate room for postactions list */
	hw_postaction_list = (void *)((u8 *)task->va + offset);
	hw_task->gen_task.postaction_lists_p = offset;
	offset = roundup(offset + sizeof(*hw_postaction_list), sizeof(u64));

	/* Allocate room for preactions list */
	hw_preaction_list = (void *)((u8 *)task->va + offset);
	hw_task->gen_task.preaction_lists_p = offset;
	offset = roundup(offset + sizeof(*hw_preaction_list), sizeof(u64));

	/* Allocate space for the input parameters */
	hw_input_parameters = (void *)((u8 *)task->va + offset);
	hw_task->input_parameters = offset;
	offset += sizeof(*hw_input_parameters) * PVA_PARAM_LAST;

	/* ..and then output parameters */
	hw_output_parameters = (void *)((u8 *)task->va + offset);
	hw_task->output_parameters = offset;
	offset += sizeof(*hw_output_parameters) * PVA_PARAM_LAST;

	/* Write the preaction list */
	err = pva_task_write_preactions(task, hw_preaction_list, &offset);
	if (err < 0)
		return err;

	/* Write the postaction list */
	pva_task_write_postactions(task, hw_postaction_list, &offset);

	/* Initialize parameters */

#define COPY_PARAMETER(target, name, name_ext, param_type, count)	\
	do {								\
		if ((name).handle) {					\
			target[(count)].address = (name_ext).dma_addr;	\
			target[(count)].size = (name_ext).size;		\
			target[(count)].type = (param_type);		\
			(count)++;					\
		}							\
	} while (0)

	COPY_PARAMETER(hw_input_parameters, task->input_scalars,
		       task->input_scalars_ext,
		       PVA_PARAM_SCALAR_LIST, num_input_parameters);
	COPY_PARAMETER(hw_input_parameters, task->input_rois,
		       task->input_rois_ext,
		       PVA_PARAM_ROI_LIST, num_input_parameters);
	COPY_PARAMETER(hw_input_parameters, task->input_2dpoint,
		       task->input_2dpoint_ext,
		       PVA_PARAM_2DPOINTS_LIST, num_input_parameters);
	COPY_PARAMETER(hw_output_parameters, task->output_scalars,
		       task->output_scalars_ext,
		       PVA_PARAM_SCALAR_LIST, num_output_parameters);
	COPY_PARAMETER(hw_output_parameters, task->output_rois,
		       task->output_rois_ext,
		       PVA_PARAM_ROI_LIST, num_output_parameters);
	COPY_PARAMETER(hw_output_parameters, task->output_2dpoint,
		       task->output_2dpoint_ext,
		       PVA_PARAM_2DPOINTS_LIST, num_output_parameters);
#undef COPY_PARAMETER

	/* Write input surfaces */
	pva_task_write_input_surfaces(task, hw_input_parameters,
			&num_input_parameters, &offset);

	/* Write output surfaces */
	pva_task_write_output_surfaces(task, hw_output_parameters,
			&num_output_parameters, &offset);

	hw_task->gen_task.versionid = TASK_VERSION_ID;
	hw_task->gen_task.engineid = PVA_ENGINE_ID;
	hw_task->gen_task.sequence = 0;
	hw_task->gen_task.length = offset;
	hw_task->gen_task.n_preaction_lists = 1;
	hw_task->gen_task.n_postaction_lists = 1;
	hw_task->runlist_version = PVA_TASK_VERSION_ID;
	hw_task->queue_id = task->queue->id;
	hw_task->num_input_parameters = num_input_parameters;
	hw_task->num_output_parameters = num_output_parameters;
	hw_task->flags = atomic ? PVA_TASK_FL_ATOMIC : 0;
	hw_task->operation = task->operation;
	hw_task->timeout = task->timeout;

	/* This should be delivered from userspace - hard-code
	 * until the mechanism is in place.
	 */
	hw_task->operation_version = 1;

	for (i = 0; i < roundup(offset, 16) / 16; i++) {
		u8 *task_va = task->va;
		u32 base = i * 16;

		nvhost_dbg_info("%02x, %02x, %02x, %02x, %02x, %02x, %02x %02x, "
				"%02x, %02x, %02x, %02x, %02x, %02x, %02x %02x",
				task_va[base],
				task_va[base + 1],
				task_va[base + 2],
				task_va[base + 3],
				task_va[base + 4],
				task_va[base + 5],
				task_va[base + 6],
				task_va[base + 7],
				task_va[base + 8],
				task_va[base + 9],
				task_va[base + 10],
				task_va[base + 11],
				task_va[base + 12],
				task_va[base + 13],
				task_va[base + 14],
				task_va[base + 15]);
	}

	return 0;
}

static void pva_completed_task_status(struct pva *pva)
{
	struct pva_cmd cmd;
	u32 nregs, flags;
	struct pva_mailbox_status_regs status;
	int err;

	/* Construct the command */
	flags = PVA_CMD_INT_ON_ERR | PVA_CMD_INT_ON_COMPLETE;
	nregs = pva_cmd_completed_task(&cmd, flags);

	/* Submit request to PVA and wait for response */
	err = pva_mailbox_send_cmd_sync(pva, &cmd, nregs, &status);
	if (err < 0) {
		nvhost_warn(&pva->pdev->dev,
			"Failed to check submit task: %d", err);
		return;
	}

	/* Check the status returned */
	nvhost_dbg_info("CCQ_Status4 0x%x\n",
				status.status[PVA_CCQ_STATUS4_INDEX]);

	nvhost_dbg_info("CCQ_Status5 0x%x\n",
				status.status[PVA_CCQ_STATUS5_INDEX]);

	nvhost_dbg_info("CCQ_Status6 0x%x\n",
				status.status[PVA_CCQ_STATUS6_INDEX]);

	nvhost_dbg_info("CCQ_Status7 0x%x\n",
				status.status[PVA_CCQ_STATUS7_INDEX]);

}

static void pva_task_update(void *priv, int nr_completed)
{
	struct pva_submit_task *task = priv;
	struct nvhost_queue *queue = task->queue;

	nvhost_dbg_info("Completed task %p (0x%llx)", task,
			(u64)task->dma_addr);

	/* Unpin job memory. PVA shouldn't be using it anymore */
	pva_task_unpin_mem(task);

	pva_completed_task_status(task->pva);

	/* Drop PM runtime reference of PVA */
	nvhost_module_idle(task->pva->pdev);

	/* remove the task from the queue */
	mutex_lock(&queue->list_lock);
	list_del(&task->node);
	mutex_unlock(&queue->list_lock);

	/* Release memory that was allocated for the task */
	nvhost_queue_free_task_memory(task->queue, task->pool_index);

	/* Drop queue reference to allow reusing it */
	nvhost_queue_put(queue);
}

static void pva_queue_dump(struct nvhost_queue *queue, struct seq_file *s)
{
	struct pva_submit_task *task;
	int i = 0;

	seq_printf(s, "Queue %u, Tasks\n", queue->id);

	mutex_lock(&queue->list_lock);
	list_for_each_entry(task, &queue->tasklist, node) {
		int j;

		seq_printf(s, "    #%u: Operation = %u\n",
				i++, task->operation);

		for (j = 0; j < task->num_prefences; j++)
			seq_printf(s, "    prefence %d: \n\t"
				"syncpoint_index=%u, syncpoint_value=%u\n",
				j,
				task->prefences[j].syncpoint_index,
				task->prefences[j].syncpoint_value);

		for (j = 0; j < task->num_postfences; j++)
			seq_printf(s, "    postfence %d: \n\t"
				"syncpoint_index=%u, syncpoint_value=%u\n",
				j,
				task->postfences[j].syncpoint_index,
				task->postfences[j].syncpoint_value);


	}
	mutex_unlock(&queue->list_lock);
}

static int pva_task_submit(struct pva_submit_task *task)
{
	struct platform_device *host1x_pdev =
			to_platform_device(task->pva->pdev->dev.parent);
	struct pva_mailbox_status_regs status;
	struct nvhost_queue *queue;
	u32 thresh, flags, nregs;
	struct pva_cmd cmd;
	int err = 0;
	int i;

	nvhost_dbg_info("Submitting task %p (0x%llx)", task,
			(u64)task->dma_addr);

	/* Turn on the hardware */
	err = nvhost_module_busy(task->pva->pdev);
	if (err)
		goto err_module_busy;

	/* Construct submit command */
	flags = PVA_CMD_INT_ON_ERR | PVA_CMD_INT_ON_COMPLETE;
	nregs = pva_cmd_submit(&cmd, task->queue->id,
				task->dma_addr, flags);

	/* Submit request to PVA and wait for response */
	err = pva_mailbox_send_cmd_sync(task->pva, &cmd, nregs, &status);
	if (err < 0) {
		nvhost_warn(&task->pva->pdev->dev,
			"Failed to submit task: %d", err);
		goto err_submit;
	}

	/* Ensure that response is valid */
	if (status.error != PVA_ERR_NO_ERROR) {
		nvhost_warn(&task->pva->pdev->dev, "PVA task rejected: %u",
				status.error);
		err = -EINVAL;
		goto err_submit;
	}

	/* Get a reference of the queue to avoid it being reused. It
	 * gets freed in the callback...
	 */
	queue = task->queue;
	nvhost_queue_get(queue);

	/* ...that is registered here */
	thresh = nvhost_syncpt_incr_max_ext(host1x_pdev,
			queue->syncpt_id, 1);
	err = nvhost_intr_register_notifier(host1x_pdev,
			queue->syncpt_id, thresh,
			pva_task_update, task);
	if (err < 0) {
		nvhost_queue_put(queue);
		goto err_register_isr;
	}

	mutex_lock(&queue->list_lock);
	list_add_tail(&task->node, &queue->tasklist);
	mutex_unlock(&queue->list_lock);

	nvhost_dbg_info("Postfence id=%u, value=%u",
			queue->syncpt_id, thresh);

	/* Return post-fences */
	for (i = 0; i < task->num_postfences; i++) {
		struct pva_fence *fence = task->postfences + i;

		switch (fence->type) {
		case PVA_FENCE_TYPE_SYNCPT: {
			fence->syncpoint_index = queue->syncpt_id;
			fence->syncpoint_value = thresh;
			break;
		}
		case PVA_FENCE_TYPE_SYNC_FD: {
			struct nvhost_ctrl_sync_fence_info pts;

			/* Fail if any previous sync_create_fence_fd failed */
			if (err < 0)
				break;

			pts.id = queue->syncpt_id;
			pts.thresh = thresh;

			err = nvhost_sync_create_fence_fd(host1x_pdev,
					&pts, 1, "fence_pva", &fence->sync_fd);

			break;
		}
		case PVA_FENCE_TYPE_SEMAPHORE:
			break;
		default:
			err = -ENOSYS;
			goto err_write_fences;
		}
	}

err_write_fences:
	return err;

err_register_isr:
err_submit:
	nvhost_module_idle(task->pva->pdev);
err_module_busy:
	return err;
}

static int pva_queue_submit(struct nvhost_queue *queue, void *args)
{
	struct pva_submit_tasks *task_header = args;
	int err = 0;
	int i;

	for (i = 0; i < task_header->num_tasks; i++) {
		struct pva_submit_task *task = task_header->tasks[i];

		/* First, dump the task that we are submitting */
		pva_task_dump(task);

		/* Pin job memory */
		err = pva_task_pin_mem(task);
		if (err < 0)
			break;

		/* Write the task data */
		pva_task_write(task, false);

		err = pva_task_submit(task);
		if (err < 0)
			break;
	}

	return err;
}

static int pva_queue_set_attribute(struct nvhost_queue *queue, void *args)
{
	uint32_t flags = PVA_CMD_INT_ON_ERR | PVA_CMD_INT_ON_COMPLETE;
	struct pva_queue_attribute *attr = args;
	struct pva_mailbox_status_regs status;
	struct pva_cmd cmd;
	int err = 0;
	u32 nregs;

	nregs = pva_cmd_set_queue_attributes(&cmd, queue->id, attr->id,
			attr->value,
			flags);

	/* Submit request to PVA and wait for response */
	err = pva_mailbox_send_cmd_sync(attr->pva, &cmd, nregs, &status);
	if (err < 0) {
		nvhost_warn(&attr->pva->pdev->dev,
			"Failed to submit task: %d\n", err);
		goto end;
	}

	/* Ensure that response is valid */
	if (status.error != PVA_ERR_NO_ERROR) {
		nvhost_warn(&attr->pva->pdev->dev,
				"PVA Q attribute rejected: %u\n",
				status.error);
		err = -EINVAL;
	}

 end:
	return err;
}

static int pva_queue_abort(struct nvhost_queue *queue)
{
	/* TBD: Abort pending tasks from the queue */

	return 0;
}

struct nvhost_queue_ops pva_queue_ops = {
	.abort = pva_queue_abort,
	.submit = pva_queue_submit,
	.get_task_size = pva_task_get_memsize,
	.dump = pva_queue_dump,
	.set_attribute = pva_queue_set_attribute,
};
