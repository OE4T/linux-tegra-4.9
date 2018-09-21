/*
 * Tegra GK20A GPU Debugger Driver
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef DBG_GPU_IOCTL_GK20A_H
#define DBG_GPU_IOCTL_GK20A_H
#include <linux/poll.h>

#include "gk20a/dbg_gpu_gk20a.h"

/* NVGPU_DBG_GPU_IOCTL_REG_OPS: the upper limit for the number
 * of regops */
#define NVGPU_IOCTL_DBG_REG_OPS_LIMIT 1024

struct dbg_session_gk20a_linux {
	struct device	*dev;
	struct dbg_session_gk20a dbg_s;
};

struct dbg_session_channel_data_linux {
	/*
	 * We have to keep a ref to the _file_, not the channel, because
	 * close(channel_fd) is synchronous and would deadlock if we had an
	 * open debug session fd holding a channel ref at that time. Holding a
	 * ref to the file makes close(channel_fd) just drop a kernel ref to
	 * the file; the channel will close when the last file ref is dropped.
	 */
	struct file *ch_f;
	struct dbg_session_channel_data ch_data;
};

/* module debug driver interface */
int gk20a_dbg_gpu_dev_release(struct inode *inode, struct file *filp);
int gk20a_dbg_gpu_dev_open(struct inode *inode, struct file *filp);
long gk20a_dbg_gpu_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
unsigned int gk20a_dbg_gpu_dev_poll(struct file *filep, poll_table *wait);

/* used by profiler driver interface */
int gk20a_prof_gpu_dev_open(struct inode *inode, struct file *filp);

#endif