/*
 * Copyright (c) 2016 NVIDIA CORPORATION. All rights reserved.
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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM tegra_rtcpu

#if !defined(_TRACE_TEGRA_RTCPU_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_TEGRA_RTCPU_H

#include <linux/tracepoint.h>

/*
 * Classes
 */

DECLARE_EVENT_CLASS(rtcpu__noarg,
	TP_PROTO(u64 tstamp),
	TP_ARGS(tstamp),
	TP_STRUCT__entry(
		__field(u64, tstamp)
	),
	TP_fast_assign(
		__entry->tstamp = tstamp;
	),
	TP_printk("tstamp:%llu", __entry->tstamp)
);

DECLARE_EVENT_CLASS(rtcpu__dump,
	TP_PROTO(u64 tstamp, u32 id, u32 len, void *data),
	TP_ARGS(tstamp, id, len, data),
	TP_STRUCT__entry(
		__field(u64, tstamp)
		__field(u32, id)
		__field(u32, len)
		__dynamic_array(__u8, data, len)
	),
	TP_fast_assign(
		__entry->tstamp = tstamp;
		__entry->id = id;
		__entry->len = len;
		memcpy(__get_dynamic_array(data), data, len);
	),
	TP_printk("tstamp:%llu id:0x%08x len:%u data:%s",
		__entry->tstamp, __entry->id, __entry->len,
		__print_hex(__get_dynamic_array(data), __entry->len))
);

/*
 * Unknown events
 */

DEFINE_EVENT(rtcpu__dump, rtcpu_unknown,
	TP_PROTO(u64 tstamp, u32 id, u32 len, void *data),
	TP_ARGS(tstamp, id, len, data)
);

/*
 * Non ARRAY event types
 */

TRACE_EVENT(rtcpu_armv7_exception,
	TP_PROTO(u64 tstamp, u32 type),
	TP_ARGS(tstamp, type),
	TP_STRUCT__entry(
		__field(u64, tstamp)
		__field(u32, type)
	),
	TP_fast_assign(
		__entry->tstamp = tstamp;
		__entry->type = type;
	),
	TP_printk("tstamp:%llu type:%u", __entry->tstamp, __entry->type)
);

TRACE_EVENT(rtcpu_start,
	TP_PROTO(u64 tstamp),
	TP_ARGS(tstamp),
	TP_STRUCT__entry(
		__field(u64, tstamp)
	),
	TP_fast_assign(
		__entry->tstamp = tstamp;
	),
	TP_printk("tstamp:%llu", __entry->tstamp)
);

TRACE_EVENT(rtcpu_string,
	TP_PROTO(u64 tstamp, u32 id, u32 len, const char *data),
	TP_ARGS(tstamp, id, len, data),
	TP_STRUCT__entry(
		__field(u64, tstamp)
		__field(u32, id)
		__field(u32, len)
		__string(data, data)
	),
	TP_fast_assign(
		__entry->tstamp = tstamp;
		__entry->id = id;
		__entry->len = len;
		__assign_str(data, data);
	),
	TP_printk("tstamp:%llu id:0x%08x len:%u str:\"%s\"",
		__entry->tstamp, __entry->id, __entry->len,
		__get_str(data))
);

DEFINE_EVENT(rtcpu__dump, rtcpu_bulk,
	TP_PROTO(u64 tstamp, u32 id, u32 len, void *data),
	TP_ARGS(tstamp, id, len, data)
);

/*
 * Base events
 */

DEFINE_EVENT(rtcpu__noarg, rtcpu_target_init,
	TP_PROTO(u64 tstamp),
	TP_ARGS(tstamp)
);

DEFINE_EVENT(rtcpu__noarg, rtcpu_start_scheduler,
	TP_PROTO(u64 tstamp),
	TP_ARGS(tstamp)
);

/*
 * VI Notify events
 */

extern char *g_trace_vinotify_tag_strs[];
extern unsigned int g_trace_vinotify_tag_str_count;

TRACE_EVENT(rtcpu_vinotify_handle_msg,
	TP_PROTO(u64 tstamp, u8 tag, u32 ch_frame),
	TP_ARGS(tstamp, tag, ch_frame),
	TP_STRUCT__entry(
		__field(u64, tstamp)
		__field(u8, tag)
		__field(u32, ch_frame)
	),
	TP_fast_assign(
		__entry->tstamp = tstamp;
		__entry->tag = tag;
		__entry->ch_frame = ch_frame;
	),
	TP_printk("tstamp:%llu tag:%s channel:%u frame:%u",
		__entry->tstamp,
		(__entry->tag < g_trace_vinotify_tag_str_count) ?
			g_trace_vinotify_tag_strs[__entry->tag] :
			__print_hex(&__entry->tag, 1),
		(__entry->ch_frame >> 8) & 0xff,
		(__entry->ch_frame >> 16) & 0xffff)
);

#endif /* _TRACE_TEGRA_RTCPU_H */

#include <trace/define_trace.h>
