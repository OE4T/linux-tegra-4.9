/*
 * drivers/video/tegra/host/pva/pva.h
 *
 * Tegra PVA header
 *
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_PVA_H__
#define __NVHOST_PVA_H__

#include "pva_queue.h"

extern const struct file_operations tegra_pva_ctrl_ops;

/**
 * Queue count of 16 is maintained per PVA.
 */
#define MAX_PVA_QUEUE_COUNT 16

/**
 * struct pva - Driver private data, shared with all applications
 *
 * @pdev:			Pointer to the PVA device
 * @queues:			Queues available for the PVA
 * @allocated_queues_mutex:	Mutex for the bitmap of reserved queues
 * @allocated_queues:		Bitmap of allocated queues
 *
 */
struct pva {
	struct platform_device *pdev;
	struct pva_queue queues[MAX_PVA_QUEUE_COUNT];
	struct mutex allocated_queues_mutex;
	unsigned long allocated_queues;
};

/**
 * pva_finalize_poweron() - Finalize the PVA Power-on-Sequence.
 *
 * @pdev:	Pointer to PVA device
 *
 * Return:	0 on Success or negative error code
 *
 * This function called from host subsystem driver after the PVA
 * partition has been brought up, clocks enabled and reset deasserted.
 * In production mode, the function needs to wait until the ready  bit
 * within the PVA aperture has been set. After that enable the PVA IRQ.
 * Register the queue priorities on the PVA.
 */
int pva_finalize_poweron(struct platform_device *pdev);

/**
 * pva_prepare_poweroff() - Prepare PVA poweroff.
 *
 * @pdev:	Pointer to PVA device
 *
 * Return:	0 on Success or negative error code
 *
 * This function called from host subsystem driver before turning off
 * the PVA. The function should turn off the PVA IRQ.
 */
int pva_prepare_poweroff(struct platform_device *pdev);

#endif
