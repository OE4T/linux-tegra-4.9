/*
 * NVDLA OS Interface
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

#ifndef _DLA_OS_INTERFACE_H_
#define _DLA_OS_INTERFACE_H_

#define DLA_DESCRIPTOR_VERSION	1
#define DLA_ENGINE_ID		0x44

#define DLA_METHOD_ID_CMD_MASK		0xff
#define DLA_RESPONSE_MSG_MASK		0xff
#define DLA_RESPONSE_CMD_MASK		0xff
#define DLA_RESPONSE_ERROR_MASK		0xff

#define DLA_RESPONSE_MSG_SHIFT		0
#define DLA_RESPONSE_CMD_SHIFT		8
#define DLA_RESPONSE_ERROR_SHIFT	16

#define DLA_INT_ON_COMPLETE_SHIFT	8
#define DLA_INT_ON_ERROR_SHIFT		9

#define PREACTION_TERMINATE	0x0
#define PREACTION_SEM_EQ	0x90
#define PREACTION_SEM_GE	0x92
#define PREACTION_GOS_EQ	0xB0
#define PREACTION_GOS_GE	0xB2
#define PREACTION_TASK_STATUS	0xC0

#define POSTACTION_TERMINATE	0x0
#define POSTACTION_SEM		0x80
#define POSTACTION_TS_SEM	0x83
#define POSTACTION_GOS		0xA0
#define POSTACTION_TASK_STATUS	0xC1

#define PING_DATA_SIZE		4
#define BUFFER_MULTIPLIER	4
#define MAX_NUM_GRIDS		6

#define ERR(code) -DLA_ERR_##code

/* Commands from host to DLA Falcon */
enum dla_cmds_e {
	DLA_CMD_PING = 1,
	DLA_CMD_GET_STATUS = 2,
	DLA_CMD_RESET = 3,
	DLA_CMD_DLA_CONTROL = 4,
	DLA_CMD_GET_QUEUE_STATUS = 5,
	DLA_CMD_GET_STATISTICS = 6,
	DLA_CMD_SUBMIT_TASK = 7,
	DLA_CMD_SET_SCHEDULER = 8,
	DLA_CMD_READ_INFO = 9,
	DLA_CMD_SET_DEBUG = 10,
	DLA_CMD_SET_REGIONS = 11,
	DLA_CMD_QUEUE_SUSPEND = 12,
	DLA_CMD_QUEUE_RESUME = 13,
	DLA_CMD_QUEUE_FLUSH = 14,
};

/* Error codes */
enum dla_errors_e {
	DLA_ERR_NONE = 0,
	DLA_ERR_INVALID_METHOD = 1,
	DLA_ERR_INVALID_TASK = 2,
	DLA_ERR_INVALID_INPUT = 3,
	DLA_ERR_INVALID_FALC_DMA = 4,
	DLA_ERR_INVALID_QUEUE = 5,
	DLA_ERR_INVALID_PREACTION = 6,
	DLA_ERR_INVALID_POSTACTION = 7,
	DLA_ERR_NO_MEM = 8,
	DLA_ERR_INVALID_DESC_VER = 9,
	DLA_ERR_INVALID_ENGINE_ID = 10,
	DLA_ERR_INVALID_REGION = 11,
	DLA_ERR_PROCESSOR_BUSY = 12,
	DLA_ERR_RETRY = 13,
};

/* Notifications from DLA Falcon to Host */
enum dla_msg_e {
	DLA_CMD_ERROR = 1,
	DLA_CMD_COMPLETE = 2,
	DLA_EXCEPTION = 3,
	DLA_SWBREAKPT = 4,
	DLA_UNHANDLED_INTERRUPT = 5,
	DLA_UNUSED = 6,
	DLA_DEBUG_PRINT = 7,
};

/**
 * Task descriptor for DLA_CMD_SUBMIT_TASK
 *
 * @next: Pointer to next task descriptor in queue
 * @version: Descriptor version
 * @engine_id : DLA engine ID
 * @size: Task descriptor size including preactions and postactions
 * @sequence: Not used in DLA
 * @num_preactions: Number of preactions
 * @num_postactions : Number of postactions
 * @preactions: Offset to preactions list
 * @postactions: Offset to postactions list
 * @operation_desc: IOVA for operation descriptors list
 * @surface_desc: IOVA for surface descriptors list
 * @address_list: IOVA address list for addresses used in surface descriptors
 * @num_operations: Number of operations in operations list
 * @queue_id: ID fo queue to insert this task
 * @status: Update task status here after completion
 * @reserved: Reserved for future use and alignment
 */
struct dla_task_descriptor {
	uint64_t next;
	uint8_t version;
	uint8_t engine_id;
	uint16_t size;
	uint16_t sequence;
	uint8_t num_preactions;
	uint8_t num_postactions;
	uint16_t preactions;
	uint16_t postactions;
	uint64_t operation_desc;
	uint64_t surface_desc;
	uint64_t address_list;
	uint8_t queue_id;
	uint16_t status;
	uint64_t lut_data;
	uint64_t roi_desc_array;
	uint64_t surface;
	uint8_t dynamic_roi;
	uint8_t num_luts;
	uint16_t num_operations;
	uint16_t num_addresses;
} __attribute__ ((packed, aligned(4)));

struct dla_action_list {
	uint16_t offset;
	uint16_t size;
} __attribute__ ((packed));

struct dla_action_opcode {
	uint8_t value;
} __attribute__ ((packed));

/**
 * Semaphore action structure
 *
 * OPCODE = 0x90/0x80/0x92/0x83
 *
 * @address: Address to read or write value
 * @value: Value to compare
 */
struct dla_action_semaphore {
	uint64_t address;
	uint32_t value;
} __attribute__ ((packed));

/**
 * GoS action structure
 *
 * OPCODE = 0xA0/0xB0/0xA2
 *
 * @index: GoS index
 * @offset: Offset within grid
 * @value: Value to compare
 */
struct dla_action_gos {
	uint8_t index;
	uint16_t offset;
	uint32_t value;
} __attribute__ ((packed));

/**
 * Status notifier action structure
 *
 * OPCODE = 0xC0/0xC1
 *
 * @address: Address to read or write status notifier
 * @status: Value to compare
 */
struct dla_action_task_status {
	uint64_t address;
	uint16_t status;
} __attribute__ ((packed));

/**
 * Status notifier structure
 *
 * @address: 64-bit timestamp representing the time at which
 * the notifier was written
 * @status_engine: status work captured from HW engine
 * @subframe: NA
 * @status_task: status word as configured from an action list
 */
struct dla_task_status {
	uint64_t timestamp;
	uint32_t status_engine;
	uint16_t subframe;
	uint16_t status_task;
} __attribute__ ((packed));

/**
 * Regions to be configured from host
 */
enum dla_regions_e {
	DLA_REGION_PRINTF = 1,
	DLA_REGION_GOS = 2,
	DLA_REGION_TRACE = 3,
};

/**
 * DLA_PRINTF_REGION
 *
 * Command to configure printf regions from host
 *
 * @region: value for DLA_PRINTF_REGION
 * @address: region address
 * @size: size of region
 */
struct dla_region_printf {
	uint32_t region;
	uint64_t address;
	uint32_t size;
} __attribute__ ((packed, aligned(8)));

/**
 * DLA_REGION_GOS
 *
 * Command to set GoS regions
 *
 * @region: value for DLA_REGION_GOS
 * @address: IOVA/PA address of region
 * @num_regions: Number of grids
 * @grid_size: Size of each grid
 */
struct dla_region_gos {
	uint32_t region;
	uint16_t num_grids;
	uint16_t grid_size;
	uint64_t address[MAX_NUM_GRIDS];
} __attribute__ ((packed, aligned(8)));

#define MAX_MESSAGE_SIZE	512

struct print_data {
	char buffer[MAX_MESSAGE_SIZE];
} __attribute__ ((packed, aligned(256)));

static inline uint32_t dla_response(uint32_t msg, uint32_t cmd, uint32_t error)
{
	return ((msg & DLA_RESPONSE_MSG_MASK) |
		(cmd << DLA_RESPONSE_CMD_SHIFT) |
		(error << DLA_RESPONSE_ERROR_SHIFT));
}

static inline uint32_t dla_command(uint32_t method_id)
{
	return (method_id & (uint32_t)DLA_METHOD_ID_CMD_MASK);
}

static inline uint32_t is_int_on_complete(uint32_t method_id)
{
	return !!((method_id >> DLA_INT_ON_COMPLETE_SHIFT) & 0x1);
}

static inline uint32_t is_int_on_error(uint32_t method_id)
{
	return !!((method_id >> DLA_INT_ON_ERROR_SHIFT) & 0x1);
}

#endif
