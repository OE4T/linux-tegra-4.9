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

#define NVHOST_NVDLA_IOCTL_MAGIC 'D'

#define NVDLA_IOCTL_CTRL_PING		\
		_IOWR(NVHOST_NVDLA_IOCTL_MAGIC, 1, struct nvdla_ctrl_ping_args)
#define NVDLA_IOCTL_CTRL_PIN   \
	_IOW(NVHOST_NVDLA_IOCTL_MAGIC, 2, struct nvdla_ctrl_pin_unpin_args)
#define NVDLA_IOCTL_CTRL_UNPIN \
	_IOW(NVHOST_NVDLA_IOCTL_MAGIC, 3, struct nvdla_ctrl_pin_unpin_args)

#define NVDLA_IOCTL_CTRL_LAST		\
		_IOC_NR(NVDLA_IOCTL_CTRL_UNPIN)

#define NVDLA_IOCTL_CTRL_MAX_ARG_SIZE  \
		sizeof(struct nvdla_ctrl_pin_unpin_args)

#endif /* __LINUX_NVHOST_NVDLA_IOCTL_H */
