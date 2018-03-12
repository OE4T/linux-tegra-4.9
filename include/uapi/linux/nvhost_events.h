/*
 * Eventlib interface for PVA
 *
 * Copyright (c) 2016-2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef NVHOST_EVENTS_H
#define NVHOST_EVENTS_H

enum {
	NVHOST_SCHEMA_VERSION = 1
};

#define NVHOST_EVENT_PROVIDER_NAME "nv_mm_nvhost"

/* Marks that the task is submitted to hardware */
struct nvhost_task_submit {
	/* Engine class ID */
	u32 class_id;

	/* Syncpoint ID */
	u32 syncpt_id;

	/* Threshold for task completion */
	u32 syncpt_thresh;

	/* PID */
	u32 pid;

	/* TID */
	u32 tid;
} __packed;

/* Marks that the task is moving to execution */
struct nvhost_task_begin {
	/* Engine class ID */
	u32 class_id;

	/* Syncpoint ID */
	u32 syncpt_id;

	/* Threshold for task completion */
	u32 syncpt_thresh;
} __packed;

/* Marks that the task is completed */
struct nvhost_task_end {
	/* Engine class ID */
	u32 class_id;

	/* Syncpoint ID */
	u32 syncpt_id;

	/* Threshold for task completion */
	u32 syncpt_thresh;
} __packed;

enum {
	/* struct nvhost_task_submit */
	NVHOST_TASK_SUBMIT = 0,

	/* struct nvhost_task_begin */
	NVHOST_TASK_BEGIN = 1,

	/* struct nvhost_task_end */
	NVHOST_TASK_END = 2,

	NVHOST_NUM_EVENT_TYPES = 3
};

enum {
	NVHOST_NUM_CUSTOM_FILTER_FLAGS = 0
};

#endif /* NVHOST_EVENTS_H */
