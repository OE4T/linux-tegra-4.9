/*
 * PVA Buffer Management Header
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

#ifndef __NVHOST_PVA_BUFFER_H__
#define __NVHOST_PVA_BUFFER_H__

/**
 * pva_buffers - Information needed for buffers
 *
 * @pdev:		Pointer to PVA device
 * @buffer_list:	List of all the buffers used by a file pointer
 * @buffer_list_mutex:	Mutex for the buffer list
 * @kref:		Reference count for the bufferlist
 *
 */
struct pva_buffers {
	struct platform_device *pdev;
	struct list_head buffer_list;
	struct mutex buffer_list_mutex;
	struct kref kref;
};

/**
 * pva_buffer_init - Initialize the pva_buffer per open request
 *
 * @pva_buffers:	Pointer to pva_buffers struct
 *
 * Return:		pva_buffers pointer on success or negative on error
 *
 * This function allocates pva_buffers struct and init the bufferlist
 * and mutex.
 */
struct pva_buffers *pva_buffer_init(struct platform_device *pdev);

/**
 * pva_buffer_pin - Pin the memhandle using dma_buf functions
 *
 * @pva_buffers:	Pointer to pva_buffers struct
 * @handles:		Pointer to MemHandle list
 * @count:		Number of memhandles in the list
 *
 * Return: 0 on success or negative on error
 *
 * This function maps the buffer memhandle list passed from user side
 * to device iova.
 */
int pva_buffer_pin(struct pva_buffers *pva_buffers, u32 *handles, u32 count);

/**
 * pva_buffer_unpin - UnPins the mapped address space.
 *
 * @pva_buffers:	Pointer to pva_buffer struct
 * @handles:		Pointer to MemHandle list
 * @count:		Number of memhandles in the list
 *
 * Return: None
 *
 */
void pva_buffer_unpin(struct pva_buffers *pva_buffers, u32 *handles, u32 count);

/**
 * pva_buffer_submit_pin - Pin the mapped buffer for a task submit
 *
 * @pva_buffers:	Pointer to pva_buffer struct
 * @handles:		Pointer to MemHandle list
 * @count:		Number of memhandles in the list
 *
 * Return: 0 on success or negative on error
 *
 * This function increased the reference count for a mapped buffer during
 * task submission.
 */
int pva_buffer_submit_pin(struct pva_buffers *pva_buffers,
					u32 *handles, u32 count);

/**
 * pva_buffer_unpin - UnPins the mapped address space on task completion.
 *
 * @pva_buffers:	Pointer to pva_buffer struct
 * @handles:		Pointer to MemHandle list
 * @count:		Number of memhandles in the list
 *
 * Return: None
 *
 * This function decrease the reference count for a mapped buffer when the
 * task get completed or aborted.
 */
void pva_buffer_submit_unpin(struct pva_buffers *pva_buffers,
					u32 *handles, u32 count);

/**
 * pva_buffer_put - Cleanup all the buffers in the list
 *
 * @pva_buffer:	Pointer to pva_buffer struct
 *
 * Return: None
 *
 */
void pva_buffer_put(struct pva_buffers *pva_buffers);

#endif /*__NVHOST_PVA_BUFFER_H__ */
