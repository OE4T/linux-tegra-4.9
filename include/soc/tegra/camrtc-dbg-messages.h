/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_DBG_MESSAGES_H
#define INCLUDE_CAMRTC_DBG_MESSAGES_H

#include "camrtc-common.h"

/* All the enums and the fields inside the structs described in this header
 * file supports only uintX_t types, where X can be 8,16,32,64.
 */

/* Requests and responses
 */
enum camrtc_request {
	/* Ping request. RTCPU returns the fw version in the data field and
	 * 0 in the status field.
	 */
	CAMRTC_REQ_PING = 1,                            /*  1 (0x01) */
	/* PM sleep request */
	CAMRTC_REQ_PM_SLEEP,                            /*  2 (0x02) */
	/* Test request */
	CAMRTC_REQ_MODS_TEST,                           /*  3 (0x03) */
	/* Set log level */
	CAMRTC_REQ_SET_LOGLEVEL,                        /*  4 (0x04) */
	CAMRTC_REQ_LOGLEVEL = CAMRTC_REQ_SET_LOGLEVEL,  /*  4 (0x04) */
	/* Get FreeRTOS state */
	CAMRTC_REQ_RTOS_STATE,                          /*  5 (0x05) */

	/* Read memory */
	CAMRTC_REQ_READ_MEMORY_32BIT,                   /*  6 (0x06) */
	CAMRTC_REQ_READ_MEMORY,                         /*  7 (0x07) */

	/* Performance counter */
	CAMRTC_REQ_SET_PERF_COUNTERS,                   /*  8 (0x08) */
	CAMRTC_REQ_GET_PERF_COUNTERS,                   /*  9 (0x09) */

	CAMRTC_REQ_GET_LOGLEVEL,                        /* 10 (0x0a) */

	CAMRTC_REQ_RUN_TEST,                            /* 11 (0x0b) */

	CAMRTC_REQ_GET_TASK_STAT,                       /* 12 (0x0c) */

	CAMRTC_REQ_ENABLE_VI_STAT,                      /* 13 (0x0d) */
	CAMRTC_REQ_GET_VI_STAT,                         /* 14 (0x0e) */

	CAMRTC_REQ_GET_MEM_USAGE,                       /* 15 (0x0f) */

	CAMRTC_REQUEST_TYPE_MAX,
};

enum camrtc_response {
	CAMRTC_RESP_PONG = 1,
	CAMRTC_RESP_PM_SLEEP,
	CAMRTC_RESP_MODS_RESULT,
	CAMRTC_RESP_LOGLEVEL,
	CAMRTC_RESP_RTOS_STATE,

	CAMRTC_RESP_READ_MEMORY_32BIT,
	CAMRTC_RESP_READ_MEMORY,

	CAMRTC_RESP_SET_PERF_COUNTERS,
	CAMRTC_RESP_GET_PERF_COUNTERS,

	CAMRTC_RESPONSE_TYPE_MAX,
};

enum camrtc_response_status {
	CAMRTC_STATUS_OK = 0,
	CAMRTC_STATUS_ERROR = 1,		/* Generic error */
	CAMRTC_STATUS_REQ_UNKNOWN = 2,		/* Unknown req_type */
	CAMRTC_STATUS_NOT_IMPLEMENTED = 3,	/* Request not implemented */
	CAMRTC_STATUS_INVALID_PARAM = 4,	/* Invalid parameter */
};

enum {
	CAMRTC_DBG_FRAME_SIZE = 384,
	CAMRTC_DBG_MAX_DATA = 376,
	CAMRTC_DBG_READ_MEMORY_COUNT_MAX = 256,
	CAMRTC_DBG_MAX_PERF_COUNTERS = 31,
	CAMRTC_DBG_TASK_STAT_MAX = 16,
};

/* This struct is used to query or set the wake timeout for the target.
 * Fields:
 * force_entry:	when set forces the target to sleep for a set time
 */
struct camrtc_pm_data {
	uint32_t force_entry;
} __packed;

/* This struct is used to send the loop count to perform the mods test
 * on the target.
 * Fields:
 * mods_loops:	number of times mods test should be run
 */
struct camrtc_mods_data {
	uint32_t mods_loops;
} __packed;

/* This struct is used to extract the firmware version of the RTCPU.
 * Fields:
 * data:	buffer to store the version string. Uses uint8_t
 */
struct camrtc_ping_data {
	uint64_t ts_req;		/* requestor timestamp */
	uint64_t ts_resp;		/* response timestamp */
	uint8_t data[64];		/* data */
} __packed;

struct camrtc_log_data {
	uint32_t level;
} __packed;

struct camrtc_rtos_state_data {
	uint8_t rtos_state[CAMRTC_DBG_MAX_DATA];	/* string data */
} __packed;

/* This structure is used to read 32 bit data from firmware address space.
 * Fields:
 *   addr: address to read from. should be 4 byte aligned.
 *   data: 32 bit value read from memory.
 */
struct camrtc_dbg_read_memory_32bit {
	uint32_t addr;
} __packed;

struct camrtc_dbg_read_memory_32bit_result {
	uint32_t data;
} __packed;

/* This structure is used to read memory in firmware address space.
 * Fields:
 *   addr: starting address. no alignment requirement
 *   count: number of bytes to read. limited to CAMRTC_DBG_READ_MEMORY_COUNT_MAX
 *   data: contents read from memory.
 */
struct camrtc_dbg_read_memory {
	uint32_t addr;
	uint32_t count;
} __packed;

struct camrtc_dbg_read_memory_result {
	uint8_t data[CAMRTC_DBG_READ_MEMORY_COUNT_MAX];
} __packed;

/* These structure is used to set event type that each performance counter
 * will monitor. This doesn't include fixed performance counter. If there
 * are 4 counters available, only 3 of them are configurable.
 * Fields:
 *   number: Number of performance counters to set.
 *     This excludes a fixed performance counter: cycle counter
 *   do_reset: Whether to reset counters
 *   cycle_counter_div64: Whether to enable cycle counter divider
 *   events: Event type to monitor
 */
struct camrtc_dbg_set_perf_counters {
	uint32_t number;
	uint32_t do_reset;
	uint32_t cycle_counter_div64;
	uint32_t events[CAMRTC_DBG_MAX_PERF_COUNTERS];
} __packed;

/* These structure is used to get performance counters.
 * Fields:
 *   number: Number of performance counters.
 *     This includes a fixed performance counter: cycle counter
 *   counters: Descriptors of event counters. First entry is for cycle counter.
 *     event: Event type that the value represents.
 *       For first entry, this field is don't care.
 *     value: Value of performance counter.
 */
struct camrtc_dbg_get_perf_counters_result {
	uint32_t number;
	struct {
		uint32_t event;
		uint32_t value;
	} counters[CAMRTC_DBG_MAX_PERF_COUNTERS];
} __packed;


#define CAMRTC_DBG_MAX_TEST_DATA (CAMRTC_DBG_MAX_DATA - 8)

struct camrtc_dbg_run_test_data {
	uint64_t timeout;	/* Time in nanoseconds */
	uint8_t data[CAMRTC_DBG_MAX_TEST_DATA];
} __packed;

struct camrtc_dbg_task_stat {
	uint32_t n_task;
	uint32_t total_count;
	struct {
		uint32_t id[2];
		uint32_t count;
	} task[CAMRTC_DBG_TASK_STAT_MAX];
} __packed;

/* These structure is used to get VI message statistics.
 * Fields:
 *   enable: enable/disable collecting vi message statistics
 */
struct camrtc_dbg_enable_vi_stat {
	uint32_t enable;
} __packed;

/* These structure is used to get VI message statistics.
 * Fields:
 *   avg: running average of VI message latency.
 *   max: maximum VI message latency observed so far.
 */
struct camrtc_dbg_vi_stat {
	uint32_t avg;
	uint32_t max;
} __packed;

/* These structure is used to get memory usage.
 * Fields:
 *   text: code memory usage
 *   bss: global/static memory usage.
 *   data: global/static memory usage.
 *   heap: heap memory usage.
 *   stack: cpu stack memory usage.
 *   free: remaining free memory.
 */
struct camrtc_dbg_mem_usage {
	uint32_t text;
	uint32_t bss;
	uint32_t data;
	uint32_t heap;
	uint32_t stack;
	uint32_t free;
} __packed;

/* This struct encapsulates the type of the request and the respective
 * data associated with that request.
 * Fields:
 * req_type:	indicates the type of the request be it pm related,
 *		mods or ping.
 * data:	Union of structs of all the request types.
 */
struct camrtc_dbg_request {
	uint32_t req_type;
	uint32_t reserved;
	union {
		struct camrtc_pm_data	pm_data;
		struct camrtc_mods_data mods_data;
		struct camrtc_ping_data ping_data;
		struct camrtc_log_data	log_data;
		struct camrtc_dbg_read_memory_32bit rm_32bit_data;
		struct camrtc_dbg_read_memory rm_data;
		struct camrtc_dbg_set_perf_counters set_perf_data;
		struct camrtc_dbg_run_test_data run_test_data;
		struct camrtc_dbg_enable_vi_stat enable_vi_stat;
	} data;
} __packed;

/* This struct encapsulates the type of the response and the respective
 * data associated with that response.
 * Fields:
 * resp_type:	indicates the type of the response be it pm related,
 *		mods or ping.
 * status:	response in regard to the request i.e success/failure.
 *		In case of mods, this field is the result.
 * data:	Union of structs of all the request/response types.
 */
struct camrtc_dbg_response {
	uint32_t resp_type;
	uint32_t status;
	union {
		struct camrtc_pm_data pm_data;
		struct camrtc_ping_data ping_data;
		struct camrtc_log_data log_data;
		struct camrtc_rtos_state_data rtos_state_data;
		struct camrtc_dbg_read_memory_32bit_result rm_32bit_data;
		struct camrtc_dbg_read_memory_result rm_data;
		struct camrtc_dbg_get_perf_counters_result get_perf_data;
		struct camrtc_dbg_run_test_data run_test_data;
		struct camrtc_dbg_task_stat task_stat_data;
		struct camrtc_dbg_vi_stat vi_stat;
		struct camrtc_dbg_mem_usage mem_usage;
	} data;
} __packed;

#endif /* INCLUDE_CAMRTC_DBG_MESSAGES_H */
