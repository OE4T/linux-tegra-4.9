/*
 * include/linux/nvhost_gpu_ioctl.h
 *
 * Tegra GPU Driver
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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


/* This contains the minimal set by which the userspace can
   determine all the properties of the GPU */

#define NVHOST_GPU_ARCH_GK100 0x000000E0
#define NVHOST_GPU_IMPL_GK20A 0x0000000A

#define NVHOST_GPU_ARCH_GM200 0x00000120
#define NVHOST_GPU_IMPL_GM20B 0x0000000B

#define NVHOST_GPU_BUS_TYPE_NONE         0
#define NVHOST_GPU_BUS_TYPE_AXI         32

#define NVHOST_GPU_FLAGS_HAS_SYNCPOINTS	(1 << 0)

struct nvhost_gpu_characteristics {
	__u32 arch;
	__u32 impl;
	__u32 rev;

	__u32 num_gpc;

	__u64 L2_cache_size;               /* bytes */
	__u64 on_board_video_memory_size;  /* bytes */

	__u32 num_tpc_per_gpc;
	__u32 bus_type;

	__u32 big_page_size;
	__u32 compression_page_size;

	__u32 pde_coverage_bit_count;
	__u32 reserved;

	__u64 flags;

	/* Notes:
	   - This struct can be safely appended with new fields. However, always
	     keep the structure size multiple of 8 and make sure that the binary
	     layout does not change between 32-bit and 64-bit architectures.
	   - If the last field is reserved/padding, it is not
	     generally safe to repurpose the field in future revisions.
	*/
};

struct nvhost_gpu_get_characteristics {
	/* [in]  size reserved by the user space. Can be 0.
	   [out] full buffer size by kernel */
	__u64 gpu_characteristics_buf_size;

	/* [in]  address of nvhost_gpu_characteristics buffer. Filled with field
	   values by exactly MIN(buf_size_in, buf_size_out) bytes. Ignored, if
	   buf_size_in is zero.  */
	__u64 gpu_characteristics_buf_addr;
};

#define NVHOST_GPU_COMPBITS_NONE	0
#define NVHOST_GPU_COMPBITS_GPU		(1 << 0)
#define NVHOST_GPU_COMPBITS_CDEH	(1 << 1)
#define NVHOST_GPU_COMPBITS_CDEV	(1 << 2)

struct nvhost_gpu_prepare_compressible_read_args {
	__u32 handle;			/* in, dmabuf fd */
	union {
		__u32 request_compbits;	/* in */
		__u32 valid_compbits;	/* out */
	};
	__u64 offset;			/* in, within handle */
	__u64 compbits_hoffset;		/* in, within handle */
	__u64 compbits_voffset;		/* in, within handle */
	__u32 width;			/* in, in pixels */
	__u32 height;			/* in, in pixels */
	__u32 block_height_log2;	/* in */
	__u32 submit_flags;		/* in (NVHOST_SUBMIT_GPFIFO_FLAGS_) */
	union {
		struct {
			__u32 syncpt_id;
			__u32 syncpt_value;
		};
		__s32 fd;
	} fence;			/* in/out */
	__u32 zbc_color;		/* out */
	__u32 reserved[5];		/* must be zero */
};

struct nvhost_gpu_mark_compressible_write_args {
	__u32 handle;			/* in, dmabuf fd */
	__u32 valid_compbits;		/* in */
	__u64 offset;			/* in, within handle */
	__u32 zbc_color;		/* in */
	__u32 reserved[3];		/* must be zero */
};

#define NVHOST_GPU_IOCTL_ZCULL_GET_CTX_SIZE \
	_IOR(NVHOST_GPU_IOCTL_MAGIC, 1, struct nvhost_gpu_zcull_get_ctx_size_args)
#define NVHOST_GPU_IOCTL_ZCULL_GET_INFO \
	_IOR(NVHOST_GPU_IOCTL_MAGIC, 2, struct nvhost_gpu_zcull_get_info_args)
#define NVHOST_GPU_IOCTL_ZBC_SET_TABLE	\
	_IOW(NVHOST_GPU_IOCTL_MAGIC, 3, struct nvhost_gpu_zbc_set_table_args)
#define NVHOST_GPU_IOCTL_ZBC_QUERY_TABLE	\
	_IOWR(NVHOST_GPU_IOCTL_MAGIC, 4, struct nvhost_gpu_zbc_query_table_args)
#define NVHOST_GPU_IOCTL_GET_CHARACTERISTICS   \
	_IOWR(NVHOST_GPU_IOCTL_MAGIC, 5, struct nvhost_gpu_get_characteristics)
#define NVHOST_GPU_IOCTL_PREPARE_COMPRESSIBLE_READ \
	_IOWR(NVHOST_GPU_IOCTL_MAGIC, 6, struct nvhost_gpu_prepare_compressible_read_args)
#define NVHOST_GPU_IOCTL_MARK_COMPRESSIBLE_WRITE \
	_IOWR(NVHOST_GPU_IOCTL_MAGIC, 7, struct nvhost_gpu_mark_compressible_write_args)

#define NVHOST_GPU_IOCTL_LAST		\
	_IOC_NR(NVHOST_GPU_IOCTL_MARK_COMPRESSIBLE_WRITE)
#define NVHOST_GPU_IOCTL_MAX_ARG_SIZE	\
	sizeof(struct nvhost_gpu_prepare_compressible_read_args)


/*
 * /dev/nvhost-tsg-gpu devices
 *
 * Opening a '/dev/nvhost-tsg-gpu' device node creates a way to
 * bind/unbind a channel to/from TSG group
 */
#define NVGPU_TSG_IOCTL_MAGIC 'T'

#define NVGPU_TSG_IOCTL_BIND_CHANNEL \
	_IOW(NVGPU_TSG_IOCTL_MAGIC, 1, int)
#define NVGPU_TSG_IOCTL_UNBIND_CHANNEL \
	_IOW(NVGPU_TSG_IOCTL_MAGIC, 2, int)

#define NVGPU_TSG_IOCTL_MAX_ARG_SIZE	\
	sizeof(int)
#define NVGPU_TSG_IOCTL_LAST		\
	_IOC_NR(NVGPU_TSG_IOCTL_UNBIND_CHANNEL)

#endif
