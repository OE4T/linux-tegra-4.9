/*
 * drivers/video/tegra/host/nvdla/nvdla.h
 *
 * Tegra Graphics Host NVDLA
 *
 * Copyright (c) 2016-2017 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_NVDLA_H__
#define __NVHOST_NVDLA_H__

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/nvhost_nvdla_ioctl.h>
#include "nvhost_buffer.h"

#include "dla_os_interface.h"

/**
 * Method ID and Method data THI registers
 */
#define NV_DLA_THI_METHOD_ID	0x00000040      /* RW-4R */
#define NV_DLA_THI_METHOD_DATA	0x00000044      /* RW-4R */

#define NV_DLA_OS_VERSION	0x00001080      /* RW-4R */

#define MAX_NUM_ACTION_LIST	1

/**
 * Maximum number of queue's per engine
 */
#define MAX_NVDLA_QUEUE_COUNT	16

/**
 * Maximum number of tasks per queue
 */
#define MAX_NVDLA_TASK_COUNT	32


/**
 * Maximum number of buffers per task
 */
#define MAX_NVDLA_PIN_BUFFERS	64
/**
 * Trace Buffer Size
 */
#define TRACE_BUFFER_SIZE		0x100000

/**
 * Max number of fences supported
 */
#define MAX_NUM_NVDLA_PREFENCES		4
#define MAX_NUM_NVDLA_POSTFENCES	4
#define MAX_NUM_NVDLA_IN_TASK_STATUS	4
#define MAX_NUM_NVDLA_OUT_TASK_STATUS	4

/**
 * keep list of DLA command size here.
 * max among them will be used to allocate memory
 *
 */
union nvdla_cmd_mem_list {
	struct dla_region_printf dont_use_me;
	u32 ping_dont_use_me;
};

#define MAX_COMMANDS_PER_DEVICE		1
#define MAX_CMD_SIZE			sizeof(union nvdla_cmd_mem_list)
#define NVDLA_CMD_OFFSET(index)		(MAX_CMD_SIZE * index)

/**
 * data structure to keep command memory
 *
 * @pa			IOVA pointing to cmd memory with offset
 * @va			VA of cmd memory
 * @index		index pointing command in pool
 *
 */

struct nvdla_cmd_mem_info {
	dma_addr_t pa;
	void *va;
	int index;
};

/**
 * data structure to keep command memory pool
 *
 * @pa			IOVA pointing to cmd memory
 * @va			VA of cmd memory
 * @lock		mutex lock
 * @alloc_table		memory usage bitmap table
 */
struct nvdla_cmd_mem {
	dma_addr_t pa;
	void *va;
	struct mutex lock;
	unsigned long alloc_table;
};

/**
 * data structure to keep per DLA engine device data
 *
 * @pdev		pointer to platform device
 * @pool		pointer to queue table
 * @dbg_mask		debug mask for print level
 * @en_trace		flag to enable kernel tracing
 * @fw_version		saves current firmware version
 * @cmd_mem		structure to hold command memory pool
 * @trace_enable	to enable/disable the DLA firmware trace
 * @events_mask		mask to set/reset the different DLA firmware trace event
 */
struct nvdla_device {
	struct platform_device *pdev;
	struct nvhost_queue_pool *pool;
	struct completion cmd_completion;
	struct mutex cmd_lock;
	int cmd_status;
	int waiting;
	u32 dbg_mask;
	u32 en_trace;
	u32 fw_version;
	struct nvdla_cmd_mem cmd_mem;
	u32 trace_enable;
	u32 events_mask;
};

/**
 * struct nvdla_task:	structure for task info
 *
 * @queue		Queue in which task submitted
 * @buffers		nvhost buffers for priv/task
 * @sp			pointer to syncpt
 * @prefences		pointer to prefences
 * @postfences		pointer to post fences
 * @fence		fence tracking for current task
 * @ref			Reference count for task
 * @list		List entry
 * @task_desc		DLA task desc VA
 * @task_desc_pa	DLA task desc PA
 * @buf_size		Total size of task dma alloc
 * @timeout		max timeout to wait for task completion
 * @op_handle		pointer to handle list of operation descriptor
 *
 */
struct nvdla_task {
	struct nvhost_queue *queue;
	struct nvhost_buffers *buffers;
	struct nvhost_syncpt *sp;
	struct nvdla_fence *prefences;
	struct nvdla_fence *postfences;
	struct nvdla_status_notify *in_task_status;
	struct nvdla_status_notify *out_task_status;
	u32 num_prefences;
	u32 num_postfences;
	u32 num_in_task_status;
	u32 num_out_task_status;
	u32 fence;
	struct kref ref;
	struct list_head list;
	struct dla_task_descriptor *task_desc;
	dma_addr_t task_desc_pa;
	size_t buf_size;
	int timeout;
	u32 num_addresses;
	struct nvdla_mem_handle address_list;
	u32 *memory_handles;
	u32 num_handles;
	int pool_index;
};

extern const struct file_operations tegra_nvdla_ctrl_ops;
extern struct nvhost_queue_ops nvdla_queue_ops;

/**
 * nvhost_nvdla_finalize_poweron() finalize power on for DLA
 *
 * @pdev	Pointer for platform device
 *
 * Return	0 on success otherwise negative
 *
 * This function called from nvhost ACM subsystem,
 * to boot falcon and wait until falcon goes idle after initial setup
 */
int nvhost_nvdla_finalize_poweron(struct platform_device *pdev);

/**
 * nvhost_nvdla_prepare_poweron() prepare to poweroff DLA
 *
 * @pdev	Pointer for platform device
 *
 * Return	0 on success otherwise negative
 *
 * This function called from nvhost ACM subsystem,
 * disables falcon interrupts and pass PM core to powergate and clockgate
 */
int nvhost_nvdla_prepare_poweroff(struct platform_device *pdev);

/**
 * nvhost_nvdla_flcn_isr() falcon interrupt handler
 *
 * @pdev	Pointer for platform device
 *
 * Return	0 on success otherwise negative
 *
 * This function called from nvhost falcon subsystem on recieving falcon
 * interrupt, like INT_ON_COMPLETE, INT_ON_ERR, DLA_DEBUG etc.
 */
int nvhost_nvdla_flcn_isr(struct platform_device *pdev);

/**
 * nvdla_send_cmd() send command to DLA
 *
 * @pdev		Pointer for platform device
 * @method_id		method id with command and other info
 * @method_data		method data for command
 * @wait		If set to true then this function waits for command
 *			complete notification from firmware
 *
 * Return		0 on success otherwise negative
 *
 * This function used to send method to falcon embedding different supporting
 * command. This uses THI registers to send method id and method data
 */
int nvdla_send_cmd(struct platform_device *pdev,
			uint32_t method_id, uint32_t method_data, bool wait);

/**
 * nvdla_task_put()	decrease task reference count
 *
 * @task		Pointer to task in operation
 *
 * Return		void
 *
 * This function puts task reference count and zero reference count
 * invokes function to free task.
 */
void nvdla_task_put(struct nvdla_task *task);

/**
 * nvdla_task_get()	increase task reference count
 *
 * @task		Pointer to task in operation
 *
 * Return		void
 *
 * This function gets task reference count
 */
void nvdla_task_get(struct nvdla_task *task);

/**
 * nvdla_task_alloc()	allocate task for a give queue
 *
 * @task		Pointer to nvdla_task
 *
 * Return		allocated task in success, otherwise pointer to err
 *
 * This function allocates task desc and fills up initial task descriptor as
 * task parameter detais
 */
int nvdla_fill_task_desc(struct nvdla_task *task);

/**
 * nvdla_send_postfences()	send back fences to UMD
 *
 * @task		Pointer to nvhost queue
 * @usr_task		Pointer to user task to be updated
 *
 * Return		0 on success otherwise negative
 *
 * This function send post fences back to UMD after task submit
 */
int nvdla_send_postfences(struct nvdla_task *task,
			struct nvdla_ioctl_submit_task usr_task);

int nvdla_get_cmd_memory(struct platform_device *pdev,
				struct nvdla_cmd_mem_info *cmd_mem_info);
int nvdla_put_cmd_memory(struct platform_device *pdev, int index);
int nvdla_set_queue_state(struct nvhost_queue *queue, int cmd);

#endif /* End of __NVHOST_NVDLA_H__ */
