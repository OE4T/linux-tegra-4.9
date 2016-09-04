/*
 * PVA Task Management
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

#ifndef PVA_QUEUE_H
#define PVA_QUEUE_H

#include <uapi/linux/nvhost_pva_ioctl.h>
#include "nvhost_queue.h"

extern struct nvhost_queue_ops pva_queue_ops;

/**
 * struct pva_submit_task - Describe a task for PVA
 *
 * @num_prefences: Number of pre-fences in this task
 * @num_postfences: Number of post-fences in this task
 * @num_input_surfaces: Number of input surfaces
 * @num_output_surfaces: Number of output surfaces
 * @num_input_task_status: Number of input task status structures
 * @num_output_task_status: Number of output task status structures
 * @reserved: Reserved for future usage.
 * @timeout: Latest Unix time when the task must complete. 0 if disabled.
 * @prefences: Pointer to pre-fence structures
 * @postfences: Pointer to post-fence structures
 * @input_surfaces: Pointer to input surfaces
 * @input_scalars: Information for input scalars
 * @input_2dpoint: Information for input 2d points
 * @input_rois: Pointer to input ROIs
 * @output_surfaces: Pointer to output surfaces
 * @output_scalars: Information for output scalars
 * @output_2dpoint: Information for output 2d points
 * @output_rois: Pointer to output ROIs
 * @input_task_status: Pointer to input status structure
 * @output_task_status: Pointer to output status structure
 *
 * This is an internal representation of the task structure. All
 * pointers refer to kernel memory.
 *
 */
struct pva_submit_task {
	u8 num_prefences;
	u8 num_postfences;
	u8 num_input_surfaces;
	u8 num_output_surfaces;
	u8 num_input_task_status;
	u8 num_output_task_status;
	u32 operation;
	u64 timeout;
	struct pva_fence *prefences;
	struct pva_fence *postfences;
	struct pva_surface *input_surfaces;
	struct pva_task_parameter input_scalars;
	struct pva_task_parameter input_2dpoint;
	struct pva_task_parameter input_rois;
	struct pva_surface *output_surfaces;
	struct pva_task_parameter output_scalars;
	struct pva_task_parameter output_2dpoint;
	struct pva_task_parameter output_rois;
	struct pva_status_handle *input_task_status;
	struct pva_status_handle *output_task_status;
};

struct pva_submit_tasks {
	struct pva_submit_task *tasks;
	u16 flags;
	u16 num_tasks;
};

#endif
