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

#define NVHOST_NVDLA_IOCTL_MAGIC 'D'

#endif /* __LINUX_NVHOST_NVDLA_IOCTL_H */
