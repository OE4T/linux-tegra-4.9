/*
 * GK20A Address Spaces
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __NVGPU_COMMON_LINUX_AS_H__
#define __NVGPU_COMMON_LINUX_AS_H__

struct inode;
struct file;

/* struct file_operations driver interface */
int gk20a_as_dev_open(struct inode *inode, struct file *filp);
int gk20a_as_dev_release(struct inode *inode, struct file *filp);
long gk20a_as_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif
