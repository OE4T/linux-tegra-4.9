/*
 * Nvhost event logging to ftrace.
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM nvhost_pva

#if !defined(_TRACE_NVHOST_PVA_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_NVHOST_PVA_H

#include <linux/tracepoint.h>


TRACE_EVENT(nvhost_pva_write,

	TP_PROTO(
		u64 delta_time,
		u8 major,
		u8 minor,
		u8 flags,
		u8 sequence,
		u32 arg1,
		u32 arg2
		),

	TP_ARGS(
		delta_time,
		major,
		minor,
		flags,
		sequence,
		arg1,
		arg2
		),

	TP_STRUCT__entry(
		__field(u64, delta_time)
		__field(u8, major)
		__field(u8, minor)
		__field(u8, flags)
		__field(u8, sequence)
		__field(u32, arg1)
		__field(u32, arg2)
		),

	TP_fast_assign(
		__entry->delta_time = delta_time;
		__entry->major = major;
		__entry->minor = minor;
		__entry->flags = flags;
		__entry->sequence = sequence;
		__entry->arg1 = arg1;
		__entry->arg2 = arg2;
		),

	TP_printk("time: %llu\tmajor: 0x%x\tminor: 0x%x\tflags: 0x%x\t"
		"sequence: 0x%x\targ1: %u\targ2: %u",
		__entry->delta_time, __entry->major, __entry->minor,
		__entry->flags, __entry->sequence, __entry->arg1, __entry->arg2)
);

#endif /*  _TRACE_NVHOST_PVA_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH ../../../nvhost-t19x/include/trace/events/

#define TRACE_INCLUDE_FILE nvhost_pva
/* This part must be outside protection */
#include <trace/define_trace.h>
