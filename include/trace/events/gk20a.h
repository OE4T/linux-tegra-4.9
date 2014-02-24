/*
 * gk20a event logging to ftrace.
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM gk20a

#if !defined(_TRACE_GK20A_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_GK20A_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(gk20a,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(__field(const char *, name)),
	TP_fast_assign(__entry->name = name;),
	TP_printk("name=%s", __entry->name)
);

DEFINE_EVENT(gk20a, gk20a_channel_open,
	TP_PROTO(const char *name),
	TP_ARGS(name)
);

DEFINE_EVENT(gk20a, gk20a_channel_release,
	TP_PROTO(const char *name),
	TP_ARGS(name)
);

TRACE_EVENT(gk20a_push_cmdbuf,
	TP_PROTO(const char *name, u32 mem_id,
			u32 words, u32 offset, void *cmdbuf),

	TP_ARGS(name, mem_id, words, offset, cmdbuf),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, mem_id)
		__field(u32, words)
		__field(u32, offset)
		__field(bool, cmdbuf)
		__dynamic_array(u32, cmdbuf, words)
	),

	TP_fast_assign(
		if (cmdbuf) {
			memcpy(__get_dynamic_array(cmdbuf), cmdbuf+offset,
					words * sizeof(u32));
		}
		__entry->cmdbuf = cmdbuf;
		__entry->name = name;
		__entry->mem_id = mem_id;
		__entry->words = words;
		__entry->offset = offset;
	),

	TP_printk("name=%s, mem_id=%08x, words=%u, offset=%d, contents=[%s]",
	  __entry->name, __entry->mem_id,
	  __entry->words, __entry->offset,
	  __print_hex(__get_dynamic_array(cmdbuf),
		  __entry->cmdbuf ? __entry->words * 4 : 0))
);

TRACE_EVENT(gk20a_channel_submit_gpfifo,
		TP_PROTO(const char *name, u32 hw_chid, u32 num_entries,
		u32 flags, u32 wait_id, u32 wait_value),

		TP_ARGS(name, hw_chid, num_entries, flags, wait_id, wait_value),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, hw_chid)
		__field(u32, num_entries)
		__field(u32, flags)
		__field(u32, wait_id)
		__field(u32, wait_value)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->hw_chid = hw_chid;
		__entry->num_entries = num_entries;
		__entry->flags = flags;
		__entry->wait_id = wait_id;
		__entry->wait_value = wait_value;
	),

	TP_printk("name=%s, hw_chid=%d, num_entries=%u, flags=%u, wait_id=%d,"
		" wait_value=%u",
		__entry->name, __entry->hw_chid, __entry->num_entries,
		__entry->flags, __entry->wait_id, __entry->wait_value)
);

TRACE_EVENT(gk20a_channel_submitted_gpfifo,
		TP_PROTO(const char *name, u32 hw_chid, u32 num_entries,
		u32 flags, u32 incr_id, u32 incr_value),

		TP_ARGS(name, hw_chid, num_entries, flags,
			incr_id, incr_value),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, hw_chid)
		__field(u32, num_entries)
		__field(u32, flags)
		__field(u32, incr_id)
		__field(u32, incr_value)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->hw_chid = hw_chid;
		__entry->num_entries = num_entries;
		__entry->flags = flags;
		__entry->incr_id = incr_id;
		__entry->incr_value = incr_value;
	),

	TP_printk("name=%s, hw_chid=%d, num_entries=%u, flags=%u,"
		" incr_id=%u, incr_value=%u",
		__entry->name, __entry->hw_chid, __entry->num_entries,
		__entry->flags, __entry->incr_id, __entry->incr_value)
);


TRACE_EVENT(gk20a_as_dev_open,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
			 __field(const char *, name)
			 ),
	TP_fast_assign(
		       __entry->name = name;
		       ),
	TP_printk("name=%s ",  __entry->name)
);

TRACE_EVENT(gk20a_as_dev_release,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
			 __field(const char *, name)
			 ),
	TP_fast_assign(
		       __entry->name = name;
		       ),
	TP_printk("name=%s ",  __entry->name)
);


TRACE_EVENT(gk20a_as_ioctl_bind_channel,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
			 __field(const char *, name)
			 ),
	TP_fast_assign(
		       __entry->name = name;
		       ),
	TP_printk("name=%s ",  __entry->name)
);


TRACE_EVENT(gk20a_as_ioctl_alloc_space,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
			 __field(const char *, name)
			 ),
	TP_fast_assign(
		       __entry->name = name;
		       ),
	TP_printk("name=%s ",  __entry->name)
);

TRACE_EVENT(gk20a_as_ioctl_free_space,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
			 __field(const char *, name)
			 ),
	TP_fast_assign(
		       __entry->name = name;
		       ),
	TP_printk("name=%s ",  __entry->name)
);

TRACE_EVENT(gk20a_as_ioctl_map_buffer,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
			 __field(const char *, name)
			 ),
	TP_fast_assign(
		       __entry->name = name;
		       ),
	TP_printk("name=%s ",  __entry->name)
);

TRACE_EVENT(gk20a_as_ioctl_unmap_buffer,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
			 __field(const char *, name)
			 ),
	TP_fast_assign(
		       __entry->name = name;
		       ),
	TP_printk("name=%s ",  __entry->name)
);

TRACE_EVENT(gk20a_mmu_fault,
	    TP_PROTO(u32 fault_hi, u32 fault_lo,
		     u32 fault_info,
		     u32 instance,
		     u32 engine_id,
		     const char *engine,
		     const char *client,
		     const char *fault_type),
	    TP_ARGS(fault_hi, fault_lo, fault_info,
		    instance, engine_id, engine, client, fault_type),
	    TP_STRUCT__entry(
			 __field(u32, fault_hi)
			 __field(u32, fault_lo)
			 __field(u32, fault_info)
			 __field(u32, instance)
			 __field(u32, engine_id)
			 __field(const char *, engine)
			 __field(const char *, client)
			 __field(const char *, fault_type)
			 ),
	    TP_fast_assign(
		       __entry->fault_hi = fault_hi;
		       __entry->fault_lo = fault_lo;
		       __entry->fault_info = fault_info;
		       __entry->instance = instance;
		       __entry->engine_id = engine_id;
		       __entry->engine = engine;
		       __entry->client = client;
		       __entry->fault_type = fault_type;
		       ),
	    TP_printk("fault=0x%x,%08x info=0x%x instance=0x%x engine_id=%d engine=%s client=%s type=%s",
		      __entry->fault_hi, __entry->fault_lo,
		      __entry->fault_info, __entry->instance, __entry->engine_id,
		      __entry->engine, __entry->client, __entry->fault_type)
);

#endif /*  _TRACE_GK20A_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
