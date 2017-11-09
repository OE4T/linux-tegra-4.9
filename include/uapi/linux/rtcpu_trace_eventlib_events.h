/*
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef RTCPU_TRACE_EVENTS_H
#define RTCPU_TRACE_EVENTS_H

enum {
	RTCPU_TRACE_SCHEMA_VERSION = 1
};

#define RTCPU_TRACE_EVENT_PROVIDER_NAME "rtcpu_trace_"

// Marks that the task is moving to execution
struct rtcpu_trace_task_begin {

	// Engine class ID
	uint32_t class_id;

	// Syncpoint ID
	uint32_t syncpt_id;

	// Threshold for task completion
	uint32_t syncpt_thresh;
} __packed;

// Marks that the task is completed
struct rtcpu_trace_task_end {

	// Engine class ID
	uint32_t class_id;

	// Syncpoint ID
	uint32_t syncpt_id;

	// Threshold for task completion
	uint32_t syncpt_thresh;
} __packed;

// VI Notify events
struct rtcpu_trace_vinotify_event {

	// Engine class ID
	uint32_t class_id;

	// Syncpoint ID
	uint32_t syncpt_id;

	// Threshold for task completion
	uint32_t syncpt_thresh;
} __packed;

// VI Notify errors
struct rtcpu_trace_vinotify_error {

	// Engine class ID
	uint32_t class_id;

	// Syncpoint ID
	uint32_t syncpt_id;

	// Threshold for task completion
	uint32_t syncpt_thresh;
} __packed;

enum {
	// struct rtcpu_trace_task_begin
	RTCPU_TRACE_TASK_BEGIN = 0,

	// struct rtcpu_trace_task_end
	RTCPU_TRACE_TASK_END = 1,

	// struct rtcpu_trace_vinotify_event
	RTCPU_TRACE_VINOTIFY_EVENT = 2,

	// struct rtcpu_trace_vinotify_error
	RTCPU_TRACE_VINOTIFY_ERROR = 3,

	RTCPU_TRACE_NUM_EVENT_TYPES = 4
};

union rtcpu_trace_event_union {
	struct rtcpu_trace_task_begin task_begin;
	struct rtcpu_trace_task_end task_end;
	struct rtcpu_trace_vinotify_event vinotify_event;
	struct rtcpu_trace_vinotify_error vinotify_error;
};

enum {
	RTCPU_TRACE_NUM_CUSTOM_FILTER_FLAGS = 0
};

#endif	// RTCPU_TRACE_EVENTS_H
