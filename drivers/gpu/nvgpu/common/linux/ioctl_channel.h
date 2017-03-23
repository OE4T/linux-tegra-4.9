/*
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
 */
#ifndef __NVGPU_IOCTL_CHANNEL_H__
#define __NVGPU_IOCTL_CHANNEL_H__

int gk20a_channel_open(struct inode *inode, struct file *filp);
int gk20a_channel_release(struct inode *inode, struct file *filp);
long gk20a_channel_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg);

#endif
