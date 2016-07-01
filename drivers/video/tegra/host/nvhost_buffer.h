/*
 * NVHOST Buffer Management Header
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

#ifndef __NVHOST_NVHOST_BUFFER_H__
#define __NVHOST_NVHOST_BUFFER_H__

/**
 * nvhost_buffers - Information needed for buffers
 *
 * @pdev:		Pointer to NVHOST device
 * @buffer_list:	List of all the buffers used by a file pointer
 * @buffer_list_mutex:	Mutex for the buffer list
 * @kref:		Reference count for the bufferlist
 *
 */
struct nvhost_buffers {
	struct platform_device *pdev;
	struct list_head buffer_list;
	struct mutex buffer_list_mutex;
	struct kref kref;
};

/**
 * nvhost_buffer_init - Initialize the nvhost_buffer per open request
 *
 * @nvhost_buffers:	Pointer to nvhost_buffers struct
 *
 * Return:		nvhost_buffers pointer on success or negative on error
 *
 * This function allocates nvhost_buffers struct and init the bufferlist
 * and mutex.
 */
struct nvhost_buffers *nvhost_buffer_init(struct platform_device *pdev);

/**
 * nvhost_buffer_pin - Pin the memhandle using dma_buf functions
 *
 * @nvhost_buffers:	Pointer to nvhost_buffers struct
 * @handles:		Pointer to MemHandle list
 * @count:		Number of memhandles in the list
 *
 * Return: 0 on success or negative on error
 *
 * This function maps the buffer memhandle list passed from user side
 * to device iova.
 */
int nvhost_buffer_pin(struct nvhost_buffers *nvhost_buffers, u32 *handles,
			u32 count);

/**
 * nvhost_buffer_unpin - UnPins the mapped address space.
 *
 * @nvhost_buffers:	Pointer to nvhost_buffer struct
 * @handles:		Pointer to MemHandle list
 * @count:		Number of memhandles in the list
 *
 * Return: None
 *
 */
void nvhost_buffer_unpin(struct nvhost_buffers *nvhost_buffers, u32 *handles,
				u32 count);

/**
 * nvhost_buffer_submit_pin - Pin the mapped buffer for a task submit
 *
 * @nvhost_buffers:	Pointer to nvhost_buffer struct
 * @handles:		Pointer to MemHandle list
 * @count:		Number of memhandles in the list
 *
 * Return: 0 on success or negative on error
 *
 * This function increased the reference count for a mapped buffer during
 * task submission.
 */
int nvhost_buffer_submit_pin(struct nvhost_buffers *nvhost_buffers,
					u32 *handles, u32 count);

/**
 * nvhost_buffer_unpin - UnPins the mapped address space on task completion.
 *
 * @nvhost_buffers:	Pointer to nvhost_buffer struct
 * @handles:		Pointer to MemHandle list
 * @count:		Number of memhandles in the list
 *
 * Return: None
 *
 * This function decrease the reference count for a mapped buffer when the
 * task get completed or aborted.
 */
void nvhost_buffer_submit_unpin(struct nvhost_buffers *nvhost_buffers,
					u32 *handles, u32 count);

/**
 * nvhost_buffer_put - Cleanup all the buffers in the list
 *
 * @nvhost_buffer:	Pointer to nvhost_buffer struct
 *
 * Return: None
 *
 */
void nvhost_buffer_put(struct nvhost_buffers *nvhost_buffers);

#endif /*__NVHOST_NVHOST_BUFFER_H__ */
