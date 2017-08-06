/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_TRACE_H
#define INCLUDE_CAMRTC_TRACE_H

#include "camrtc-common.h"

/*
 * Trace memory consists of three part.
 *
 * 1. Trace memory header: This describes the layout of trace memory,
 * and latest activities.
 *
 * 2. Exception memory: This is an array of exception entries. Each
 * entry describes an exception occurred in the firmware.
 *
 * 3. Event memory: This is an array of event entries. This is implemented
 * as a ring buffer.
 *
 * The next index gets updated when new messages are committed to the
 * trace memory. The next index points to the entry to be written to
 * at next occurrence of the exception or event.
 *
 * Trace memory layout
 *
 * 0x00000 +-------------------------------+
 *         |      Trace Memory Header      |
 * 0x01000 +-------------------------------+
 *         |                               |
 *         |        Exception Memory       | <- exception_next_idx
 *         |                               |
 * 0x10000 +-------------------------------+
 *         |                               |
 *         |                               |
 *         |          Event Memory         |
 *         |                               | <- event_next_idx
 *         |                               |
 *         +-------------------------------+
 */

/* Offset of each memory */
#define CAMRTC_TRACE_NEXT_IDX_SIZE	U32_C(64)
#define CAMRTC_TRACE_EXCEPTION_OFFSET	U32_C(0x01000)
#define CAMRTC_TRACE_EVENT_OFFSET	U32_C(0x10000)

/* Size of each entry */
#define CAMRTC_TRACE_EXCEPTION_SIZE	U32_C(1024)
#define CAMRTC_TRACE_EVENT_SIZE		U32_C(64)

/* Depth of call stack */
#define CAMRTC_TRACE_CALLSTACK_MAX	U32_C(32)
#define CAMRTC_TRACE_CALLSTACK_MIN	U32_C(4)

/*
 * Trace memory header
 */

#define CAMRTC_TRACE_SIGNATURE_1	U32_C(0x5420564e)
#define CAMRTC_TRACE_SIGNATURE_2	U32_C(0x45434152)

struct camrtc_trace_memory_header {
	/* layout: offset 0 */
	uint32_t signature[2];
	uint32_t revision;
	uint32_t reserved1;
	uint32_t exception_offset;
	uint32_t exception_size;
	uint32_t exception_entries;
	uint32_t reserved2;
	uint32_t event_offset;
	uint32_t event_size;
	uint32_t event_entries;
	uint32_t reserved3;
	uint32_t reserved4[0xd0 / 4];

	/* pointer: offset 0x100 */
	uint32_t exception_next_idx;
	uint32_t event_next_idx;
	uint32_t reserved_ptrs[0x38 / 4];
} __packed;

/*
 * Exception entry
 */

enum camrtc_trace_armv7_exception_type {
	/* Reset = 0 */
	CAMRTC_ARMV7_EXCEPTION_UNDEFINED_INSTRUCTION = 1,
	/* SWI = 2 */
	CAMRTC_ARMV7_EXCEPTION_PREFETCH_ABORT = 3,
	CAMRTC_ARMV7_EXCEPTION_DATA_ABORT,
	CAMRTC_ARMV7_EXCEPTION_RSVD,	/* Should never happen */
	CAMRTC_ARMV7_EXCEPTION_IRQ,	/* Unhandled IRQ */
	CAMRTC_ARMV7_EXCEPTION_FIQ,	/* Unhandled FIQ */
};

struct camrtc_trace_callstack {
	uint32_t lr_stack_addr;		/* address in stack where lr is saved */
	uint32_t lr;			/* value of saved lr */
} __packed;

struct camrtc_trace_armv7_exception {
	uint32_t len;		/* length in byte including this */
	uint32_t type;		/* enum camrtc_trace_armv7_exception_type */
	union {
		uint32_t data[24];
		struct {
			uint32_t r0, r1, r2, r3;
			uint32_t r4, r5, r6, r7;
			uint32_t r8, r9, r10, r11;
			uint32_t r12, sp, lr, pc;
			uint32_t r8_prev, r9_prev, r10_prev, r11_prev, r12_prev;
			uint32_t sp_prev, lr_prev;
			uint32_t reserved;
		};
	} gpr;
	/* program status registers */
	uint32_t cpsr, spsr;
	/* data fault status/address register */
	uint32_t dfsr, dfar, adfsr;
	/* instruction fault status/address register */
	uint32_t ifsr, ifar, aifsr;
	struct camrtc_trace_callstack callstack[CAMRTC_TRACE_CALLSTACK_MAX];
} __packed;

/*
 * Each trace event shares the header.
 * The format of event data is determined by event type.
 */

#define CAMRTC_TRACE_EVENT_HEADER_SIZE		U32_C(16)
#define CAMRTC_TRACE_EVENT_PAYLOAD_SIZE		\
	(CAMRTC_TRACE_EVENT_SIZE - CAMRTC_TRACE_EVENT_HEADER_SIZE)

#define CAMRTC_EVENT_TYPE_OFFSET		U32_C(24)
#define CAMRTC_EVENT_TYPE_MASK			\
	(U32_C(0xff) << CAMRTC_EVENT_TYPE_OFFSET)
#define CAMRTC_EVENT_TYPE_FROM_ID(id)		\
	(((id) & CAMRTC_EVENT_TYPE_MASK) >> CAMRTC_EVENT_TYPE_OFFSET)

#define CAMRTC_EVENT_MODULE_OFFSET		U32_C(16)
#define CAMRTC_EVENT_MODULE_MASK		\
	(U32_C(0xff) << CAMRTC_EVENT_MODULE_OFFSET)
#define CAMRTC_EVENT_MODULE_FROM_ID(id)		\
	(((id) & CAMRTC_EVENT_MODULE_MASK) >> CAMRTC_EVENT_MODULE_OFFSET)

#define CAMRTC_EVENT_SUBID_OFFSET		U32_C(0)
#define CAMRTC_EVENT_SUBID_MASK			\
	(U32_C(0xffff) << CAMRTC_EVENT_SUBID_OFFSET)
#define CAMRTC_EVENT_SUBID_FROM_ID(id)		\
	(((id) & CAMRTC_EVENT_SUBID_MASK) >> CAMRTC_EVENT_SUBID_OFFSET)

#define CAMRTC_EVENT_MAKE_ID(type, module, subid) \
	(((type) << CAMRTC_EVENT_TYPE_OFFSET) | \
	((module) << CAMRTC_EVENT_MODULE_OFFSET) | (subid))

struct camrtc_event_header {
	uint32_t len;		/* Size in bytes including this field */
	uint32_t id;		/* Event ID */
	uint64_t tstamp;	/* Timestamp from TKE TSC */
} __packed;

struct camrtc_event_struct {
	struct camrtc_event_header header;
	union {
		uint8_t data8[CAMRTC_TRACE_EVENT_PAYLOAD_SIZE];
		uint32_t data32[CAMRTC_TRACE_EVENT_PAYLOAD_SIZE / 4];
	} data;
} __packed;

// camrtc_event_type
#define	CAMRTC_EVENT_TYPE_ARRAY		U32_C(0)
#define	CAMRTC_EVENT_TYPE_ARMV7_EXCEPTION	U32_C(1)
#define	CAMRTC_EVENT_TYPE_PAD		U32_C(2)
#define	CAMRTC_EVENT_TYPE_START		U32_C(3)
#define	CAMRTC_EVENT_TYPE_STRING	U32_C(4)
#define	CAMRTC_EVENT_TYPE_BULK		U32_C(5)

// camrtc_event_module
#define	CAMRTC_EVENT_MODULE_UNKNOWN	U32_C(0)
#define	CAMRTC_EVENT_MODULE_BASE	U32_C(1)
#define	CAMRTC_EVENT_MODULE_RTOS	U32_C(2)
#define	CAMRTC_EVENT_MODULE_HEARTBEAT	U32_C(3)
#define	CAMRTC_EVENT_MODULE_DBG		U32_C(4)
#define	CAMRTC_EVENT_MODULE_MODS	U32_C(5)
#define	CAMRTC_EVENT_MODULE_VINOTIFY	U32_C(6)
#define	CAMRTC_EVENT_MODULE_I2C		U32_C(7)

// camrtc_trace_event_type_ids
#define	camrtc_trace_type_exception \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARMV7_EXCEPTION, \
		CAMRTC_EVENT_MODULE_BASE, U32_C(0))
#define	camrtc_trace_type_pad \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_PAD, \
		CAMRTC_EVENT_MODULE_BASE, U32_C(0))
#define	camrtc_trace_type_start \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_START, \
		CAMRTC_EVENT_MODULE_BASE, U32_C(0))
#define	camrtc_trace_type_string \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_STRING, \
		CAMRTC_EVENT_MODULE_BASE, U32_C(0))

// camrtc_trace_base_ids
#define	camrtc_trace_base_ids_begin \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
			CAMRTC_EVENT_MODULE_BASE, U32_C(0))
#define	camrtc_trace_base_target_init \
		camrtc_trace_base_ids_begin + U32_C(1)
#define	camrtc_trace_base_start_scheduler \
		camrtc_trace_base_ids_begin + U32_C(2)

// camrtc_trace_event_rtos_ids
#define camrtc_trace_rtos_ids_begin \
        CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
            CAMRTC_EVENT_MODULE_RTOS, U32_C(0))
#define camrtc_trace_rtos_task_switched_in \
        camrtc_trace_rtos_ids_begin + U32_C(1)
#define camrtc_trace_rtos_increase_tick_count \
        camrtc_trace_rtos_ids_begin + U32_C(2)
#define camrtc_trace_rtos_low_power_idle_begin \
        camrtc_trace_rtos_ids_begin + U32_C(3)
#define camrtc_trace_rtos_low_power_idle_end \
        camrtc_trace_rtos_ids_begin + U32_C(4)
#define camrtc_trace_rtos_task_switched_out \
        camrtc_trace_rtos_ids_begin + U32_C(5)
#define camrtc_trace_rtos_task_priority_inherit \
        camrtc_trace_rtos_ids_begin + U32_C(6)
#define camrtc_trace_rtos_task_priority_disinherit \
        camrtc_trace_rtos_ids_begin + U32_C(7)
#define camrtc_trace_rtos_blocking_on_queue_receive \
        camrtc_trace_rtos_ids_begin + U32_C(8)
#define camrtc_trace_rtos_blocking_on_queue_send \
        camrtc_trace_rtos_ids_begin + U32_C(9)
#define camrtc_trace_rtos_moved_task_to_ready_state \
        camrtc_trace_rtos_ids_begin + U32_C(10)
#define camrtc_trace_rtos_queue_create \
        camrtc_trace_rtos_ids_begin + U32_C(11)
#define camrtc_trace_rtos_queue_create_failed \
        camrtc_trace_rtos_ids_begin + U32_C(12)
#define camrtc_trace_rtos_create_mutex \
        camrtc_trace_rtos_ids_begin + U32_C(13)
#define camrtc_trace_rtos_create_mutex_failed \
        camrtc_trace_rtos_ids_begin + U32_C(14)
#define camrtc_trace_rtos_give_mutex_recursive \
        camrtc_trace_rtos_ids_begin + U32_C(15)
#define camrtc_trace_rtos_give_mutex_recursive_failed \
        camrtc_trace_rtos_ids_begin + U32_C(16)
#define camrtc_trace_rtos_take_mutex_recursive \
        camrtc_trace_rtos_ids_begin + U32_C(17)
#define camrtc_trace_rtos_take_mutex_recursive_failed \
        camrtc_trace_rtos_ids_begin + U32_C(18)
#define camrtc_trace_rtos_create_counting_semaphore \
        camrtc_trace_rtos_ids_begin + U32_C(19)
#define camrtc_trace_rtos_create_counting_semaphore_failed \
        camrtc_trace_rtos_ids_begin + U32_C(20)
#define camrtc_trace_rtos_queue_send \
        camrtc_trace_rtos_ids_begin + U32_C(21)
#define camrtc_trace_rtos_queue_send_failed \
        camrtc_trace_rtos_ids_begin + U32_C(22)
#define camrtc_trace_rtos_queue_receive \
        camrtc_trace_rtos_ids_begin + U32_C(23)
#define camrtc_trace_rtos_queue_peek \
        camrtc_trace_rtos_ids_begin + U32_C(24)
#define camrtc_trace_rtos_queue_peek_from_isr \
        camrtc_trace_rtos_ids_begin + U32_C(25)
#define camrtc_trace_rtos_queue_receive_failed \
        camrtc_trace_rtos_ids_begin + U32_C(26)
#define camrtc_trace_rtos_queue_send_from_isr \
        camrtc_trace_rtos_ids_begin + U32_C(27)
#define camrtc_trace_rtos_queue_send_from_isr_failed \
        camrtc_trace_rtos_ids_begin + U32_C(28)
#define camrtc_trace_rtos_queue_receive_from_isr \
        camrtc_trace_rtos_ids_begin + U32_C(29)
#define camrtc_trace_rtos_queue_receive_from_isr_failed \
        camrtc_trace_rtos_ids_begin + U32_C(30)
#define camrtc_trace_rtos_queue_peek_from_isr_failed \
        camrtc_trace_rtos_ids_begin + U32_C(31)
#define camrtc_trace_rtos_queue_delete \
        camrtc_trace_rtos_ids_begin + U32_C(32)
#define camrtc_trace_rtos_task_create \
        camrtc_trace_rtos_ids_begin + U32_C(33)
#define camrtc_trace_rtos_task_create_failed \
        camrtc_trace_rtos_ids_begin + U32_C(34)
#define camrtc_trace_rtos_task_delete \
        camrtc_trace_rtos_ids_begin + U32_C(35)
#define camrtc_trace_rtos_task_delay_until \
        camrtc_trace_rtos_ids_begin + U32_C(36)
#define camrtc_trace_rtos_task_delay \
        camrtc_trace_rtos_ids_begin + U32_C(37)
#define camrtc_trace_rtos_task_priority_set \
        camrtc_trace_rtos_ids_begin + U32_C(38)
#define camrtc_trace_rtos_task_suspend \
        camrtc_trace_rtos_ids_begin + U32_C(39)
#define camrtc_trace_rtos_task_resume \
        camrtc_trace_rtos_ids_begin + U32_C(40)
#define camrtc_trace_rtos_task_resume_from_isr \
        camrtc_trace_rtos_ids_begin + U32_C(41)
#define camrtc_trace_rtos_task_increment_tick \
        camrtc_trace_rtos_ids_begin + U32_C(42)
#define camrtc_trace_rtos_timer_create \
        camrtc_trace_rtos_ids_begin + U32_C(43)
#define camrtc_trace_rtos_timer_create_failed \
        camrtc_trace_rtos_ids_begin + U32_C(44)
#define camrtc_trace_rtos_timer_command_send \
        camrtc_trace_rtos_ids_begin + U32_C(45)
#define camrtc_trace_rtos_timer_expired \
        camrtc_trace_rtos_ids_begin + U32_C(46)
#define camrtc_trace_rtos_timer_command_received \
        camrtc_trace_rtos_ids_begin + U32_C(47)
#define camrtc_trace_rtos_malloc \
        camrtc_trace_rtos_ids_begin + U32_C(48)
#define camrtc_trace_rtos_free \
        camrtc_trace_rtos_ids_begin + U32_C(49)
#define camrtc_trace_rtos_event_group_create \
        camrtc_trace_rtos_ids_begin + U32_C(50)
#define camrtc_trace_rtos_event_group_create_failed \
        camrtc_trace_rtos_ids_begin + U32_C(51)
#define camrtc_trace_rtos_event_group_sync_block \
        camrtc_trace_rtos_ids_begin + U32_C(52)
#define camrtc_trace_rtos_event_group_sync_end \
        camrtc_trace_rtos_ids_begin + U32_C(53)
#define camrtc_trace_rtos_event_group_wait_bits_block \
        camrtc_trace_rtos_ids_begin + U32_C(54)
#define camrtc_trace_rtos_event_group_wait_bits_end \
        camrtc_trace_rtos_ids_begin + U32_C(55)
#define camrtc_trace_rtos_event_group_clear_bits \
        camrtc_trace_rtos_ids_begin + U32_C(56)
#define camrtc_trace_rtos_event_group_clear_bits_from_isr \
        camrtc_trace_rtos_ids_begin + U32_C(57)
#define camrtc_trace_rtos_event_group_set_bits \
        camrtc_trace_rtos_ids_begin + U32_C(58)
#define camrtc_trace_rtos_event_group_set_bits_from_isr \
        camrtc_trace_rtos_ids_begin + U32_C(59)
#define camrtc_trace_rtos_event_group_delete \
        camrtc_trace_rtos_ids_begin + U32_C(60)
#define camrtc_trace_rtos_pend_func_call \
        camrtc_trace_rtos_ids_begin + U32_C(61)
#define camrtc_trace_rtos_pend_func_call_from_isr \
        camrtc_trace_rtos_ids_begin + U32_C(62)
#define camrtc_trace_rtos_queue_registry_add \
        camrtc_trace_rtos_ids_begin + U32_C(63)

// camrtc_trace_dbg_ids
#define	camrtc_trace_dbg_ids_begin \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
			CAMRTC_EVENT_MODULE_DBG, U32_C(0))
#define	camrtc_trace_dbg_unknown \
		camrtc_trace_dbg_ids_begin + U32_C(1)
#define	camrtc_trace_dbg_enter \
		camrtc_trace_dbg_ids_begin + U32_C(2)
#define	camrtc_trace_dbg_exit \
		camrtc_trace_dbg_ids_begin + U32_C(3)
#define	camrtc_trace_dbg_set_loglevel \
		camrtc_trace_dbg_ids_begin + U32_C(4)

// camrtc_trace_vinotify_ids
#define	camrtc_trace_vinotify_ids_begin \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
			CAMRTC_EVENT_MODULE_VINOTIFY, U32_C(0))
#define	camrtc_trace_vinotify_handle_msg \
		camrtc_trace_vinotify_ids_begin + U32_C(1)

#endif /* INCLUDE_CAMRTC_TRACE_H */
