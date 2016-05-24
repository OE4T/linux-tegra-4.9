/*
 * PVA Queue management header for T194
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

#ifndef __NVHOST_PVA_QUEUE_H__
#define __NVHOST_PVA_QUEUE_H__

struct pva;

/**
 * struct pva_queue - Information needed in a Queue
 *
 * @pdev:	Platform device for thi
 * @kref:	struct kref for reference count
 * @syncpt_id:	Host1x syncpt id
 * @id:		Queue id
 *
 */
struct pva_queue {
	struct pva *pva;
	struct kref kref;
	u32 syncpt_id;
	u32 id;
};

/**
 * pva_queue_init() - Initialize PVA queue structures
 *
 * @pva
 *
 * Return:	None
 *
 * This function initializes PVA data structures within PVA device data.
 */
void pva_queue_init(struct pva *pva);

/**
 * pva_queue_put() - Release reference of a queue
 *
 * @queue:	Pointer to an allocated queue.
 *
 * Return:	None
 *
 * This function releases reference for a queue.
 */
void pva_queue_put(struct pva_queue *queue);

/**
 * pva_queue_get() - Get reference on a queue.
 *
 * @queue:	Pointer to an allocated queue.
 *
 * Return:	None
 *
 * This function used to get a reference to an already allocated queue.
 */
void pva_queue_get(struct pva_queue *queue);

/**
 * pva_queue_alloc() - Allocate a queue for PVA.
 *
 * @pdev:	Pointer to a PVA platform device.
 *
 * Return:	Pointer to a queue struct on Success
 *		or negative error on failure.
 *
 * This function allocates a queue from the PVA device for the user.
 */
struct pva_queue *pva_queue_alloc(struct platform_device *pdev);

/**
 * pva_queue_abort() - Abort tasks within a PVA queue
 *
 * @pdev:	Pointer to an allocated queue
 *
 * Return:	None
 *
 * This function aborts all tasks from the given PVA queue. If there is no
 * active tasks, the function call is no-op.
 *
 * It is expected to be called when an active device fd gets closed.
 */
int pva_queue_abort(struct pva_queue *queue);

#endif
