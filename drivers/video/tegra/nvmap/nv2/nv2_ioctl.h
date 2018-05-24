/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __NVMAP2_IOCTL_H
#define __NVMAP2_IOCTL_H

int NVMAP2_ioctl_create(struct file *filp, unsigned int cmd, void __user *arg);
int NVMAP2_ioctl_free(struct file *filp, unsigned long fd);
int NVMAP2_ioctl_alloc(struct file *filp, unsigned int cmd, void __user *arg);
int NVMAP2_ioctl_getfd(struct file *filp, void __user *arg);

int NVMAP2_ioctl_create_from_va(struct file *filp, void __user *arg);
int NVMAP2_ioctl_create_from_ivc(struct file *filp, void __user *arg);

int NVMAP2_ioctl_get_ivc_heap(struct file *filp, void __user *arg);
int NVMAP2_ioctl_get_ivcid(struct file *filp, void __user *arg);

int NVMAP2_ioctl_vpr_floor_size(struct file *filp, void __user *arg);

int NVMAP2_ioctl_cache_maint(struct file *filp, void __user *arg, int op_size);
int NVMAP2_ioctl_cache_maint_list(struct file *filp, void __user *arg,
				 bool is_reserve_ioctl);

int NVMAP2_ioctl_rw_handle(struct file *filp, int is_read, void __user *arg,
			  size_t op_size);

#endif /* __NVMAP2_IOCTL_H */
