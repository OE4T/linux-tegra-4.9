/*
 * include/linux/nvhost_gpu_ioctl.h
 *
 * Tegra GPU Driver
 *
 * Copyright (c) 2011-2013, NVIDIA CORPORATION.  All rights reserved.
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
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __LINUX_NVHOST_GPU_IOCTL_H
#define __LINUX_NVHOST_GPU_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if !defined(__KERNEL__)
#define __user
#endif

#define NVHOST_GPU_IOCTL_MAGIC 'G'

/*
 * /dev/nvhost-ctrl-gr3d devices
 *
 * Opening a '/dev/nvhost-ctrl-gr3d' device node creates a way to send
 * ctrl ioctl to gpu driver.
 *
 * /dev/nvhost-gr3d is for channel (context specific) operations. We use
 * /dev/nvhost-ctrl-gr3d for global (context independent) operations on
 * gpu device.
 */

/* return zcull ctx size */
struct nvhost_gpu_zcull_get_ctx_size_args {
	__u32 size;
} __packed;

/* return zcull info */
struct nvhost_gpu_zcull_get_info_args {
	__u32 width_align_pixels;
	__u32 height_align_pixels;
	__u32 pixel_squares_by_aliquots;
	__u32 aliquot_total;
	__u32 region_byte_multiplier;
	__u32 region_header_size;
	__u32 subregion_header_size;
	__u32 subregion_width_align_pixels;
	__u32 subregion_height_align_pixels;
	__u32 subregion_count;
};

#define NVHOST_ZBC_COLOR_VALUE_SIZE	4
#define NVHOST_ZBC_TYPE_INVALID		0
#define NVHOST_ZBC_TYPE_COLOR		1
#define NVHOST_ZBC_TYPE_DEPTH		2

struct nvhost_gpu_zbc_set_table_args {
	__u32 color_ds[NVHOST_ZBC_COLOR_VALUE_SIZE];
	__u32 color_l2[NVHOST_ZBC_COLOR_VALUE_SIZE];
	__u32 depth;
	__u32 format;
	__u32 type;	/* color or depth */
} __packed;
/* TBD: remove this once mobilerm removed old references */
#define nvhost_zbc_set_table_args nvhost_gpu_zbc_set_table_args

struct nvhost_gpu_zbc_query_table_args {
	__u32 color_ds[NVHOST_ZBC_COLOR_VALUE_SIZE];
	__u32 color_l2[NVHOST_ZBC_COLOR_VALUE_SIZE];
	__u32 depth;
	__u32 ref_cnt;
	__u32 format;
	__u32 type;		/* color or depth */
	__u32 index_size;	/* [out] size, [in] index */
} __packed;

#define NVHOST_GPU_IOCTL_ZCULL_GET_CTX_SIZE \
	_IOR(NVHOST_GPU_IOCTL_MAGIC, 1, struct nvhost_gpu_zcull_get_ctx_size_args)
#define NVHOST_GPU_IOCTL_ZCULL_GET_INFO \
	_IOR(NVHOST_GPU_IOCTL_MAGIC, 2, struct nvhost_gpu_zcull_get_info_args)
#define NVHOST_GPU_IOCTL_ZBC_SET_TABLE	\
	_IOW(NVHOST_GPU_IOCTL_MAGIC, 3, struct nvhost_gpu_zbc_set_table_args)
#define NVHOST_GPU_IOCTL_ZBC_QUERY_TABLE	\
	_IOWR(NVHOST_GPU_IOCTL_MAGIC, 4, struct nvhost_gpu_zbc_query_table_args)

#define NVHOST_GPU_IOCTL_LAST		\
	_IOC_NR(NVHOST_GPU_IOCTL_ZBC_QUERY_TABLE)
#define NVHOST_GPU_IOCTL_MAX_ARG_SIZE	\
	sizeof(struct nvhost_gpu_zbc_query_table_args)

#endif
