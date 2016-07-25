/*
 * include/linux/nvhost_nvdla_ioctl.h
 *
 * Tegra NvDLA Driver
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_NVHOST_NVDLA_IOCTL_H
#define __LINUX_NVHOST_NVDLA_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if !defined(__KERNEL__)
#define __user
#endif

struct nvdla_ctrl_ping_args {
	__u32 in_challenge;
	__u32 out_response;
};

struct nvdla_ctrl_pin_unpin_args {
	__u64 buffers;
	__u32 num_buffers;
	__u32 reserved;
};

struct nvdla_ctrl_submit_args {
	__u64 tasks;
	__u16 num_tasks;
#define MAX_TASKS_PER_SUBMIT	32
#define NVDLA_SUBMIT_FLAGS_ATOMIC	(1 << 0)
	__u16 flags;
	__u32 version;
};

struct nvdla_ctrl_ioctl_submit_task {
	__u8 num_prefences;
	__u8 num_postfences;
	__u8 num_input_task_status;
	__u8 num_output_task_status;
	__u16 flags;
	__u16 reserved;

	__u64 prefences;
	__u64 postfences;

	__u64 input_task_status;
	__u64 output_task_status;

	__u32 num_operations;
	__u32 num_addresses;
	__u64 operation_desc;

	__u64 surface_desc;
	__u64 address_list;
};

struct nvdla_fence {
	__u32 type;
#define NVDLA_FENCE_TYPE_SYNCPT		0
#define NVDLA_FENCE_TYPE_SYNC_FD	1
	__u32 syncpoint_index;
	__u32 syncpoint_value;
	__u32 sync_fd;
};

struct nvdla_task_surface {
	__u64 format;
	__u32 handle;
	__u32 offset;
};

struct nvdla_task_roi {
	__u32 handle;
	__u32 offset;
};

struct nvdla_task_status_handle {
	__u32 handle;
	__u32 reserved;
	__u64 offset;
};

#define NVHOST_NVDLA_IOCTL_MAGIC 'D'

#define NVDLA_IOCTL_CTRL_PING		\
		_IOWR(NVHOST_NVDLA_IOCTL_MAGIC, 1, struct nvdla_ctrl_ping_args)
#define NVDLA_IOCTL_CTRL_PIN   \
	_IOW(NVHOST_NVDLA_IOCTL_MAGIC, 2, struct nvdla_ctrl_pin_unpin_args)
#define NVDLA_IOCTL_CTRL_UNPIN \
	_IOW(NVHOST_NVDLA_IOCTL_MAGIC, 3, struct nvdla_ctrl_pin_unpin_args)
#define NVDLA_IOCTL_CTRL_SUBMIT \
	_IOW(NVHOST_NVDLA_IOCTL_MAGIC, 4, struct nvdla_ctrl_submit_args)
#define NVDLA_IOCTL_CTRL_LAST		\
		_IOC_NR(NVDLA_IOCTL_CTRL_SUBMIT)

#define NVDLA_IOCTL_CTRL_MAX_ARG_SIZE  \
		sizeof(struct nvdla_ctrl_pin_unpin_args)

#endif /* __LINUX_NVHOST_NVDLA_IOCTL_H */
