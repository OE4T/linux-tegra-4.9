/*
 * NVGPU Public Interface Header
 *
 * Copyright (c) 2011-2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _UAPI__LINUX_NVGPU_IOCTL_H
#define _UAPI__LINUX_NVGPU_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if !defined(__KERNEL__)
#define __user
#endif

/*
 * /dev/nvhost-ctrl-gpu device
 *
 * Opening a '/dev/nvhost-ctrl-gpu' device node creates a way to send
 * ctrl ioctl to gpu driver.
 *
 * /dev/nvhost-gpu is for channel (context specific) operations. We use
 * /dev/nvhost-ctrl-gpu for global (context independent) operations on
 * gpu device.
 */

#define NVGPU_GPU_IOCTL_MAGIC 'G'

/* return zcull ctx size */
struct nvgpu_gpu_zcull_get_ctx_size_args {
	__u32 size;
} __packed;

/* return zcull info */
struct nvgpu_gpu_zcull_get_info_args {
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

#define NVGPU_ZBC_COLOR_VALUE_SIZE	4
#define NVGPU_ZBC_TYPE_INVALID		0
#define NVGPU_ZBC_TYPE_COLOR		1
#define NVGPU_ZBC_TYPE_DEPTH		2

struct nvgpu_gpu_zbc_set_table_args {
	__u32 color_ds[NVGPU_ZBC_COLOR_VALUE_SIZE];
	__u32 color_l2[NVGPU_ZBC_COLOR_VALUE_SIZE];
	__u32 depth;
	__u32 format;
	__u32 type;	/* color or depth */
} __packed;

struct nvgpu_gpu_zbc_query_table_args {
	__u32 color_ds[NVGPU_ZBC_COLOR_VALUE_SIZE];
	__u32 color_l2[NVGPU_ZBC_COLOR_VALUE_SIZE];
	__u32 depth;
	__u32 ref_cnt;
	__u32 format;
	__u32 type;		/* color or depth */
	__u32 index_size;	/* [out] size, [in] index */
} __packed;


/* This contains the minimal set by which the userspace can
   determine all the properties of the GPU */

#define NVGPU_GPU_ARCH_GK100 0x000000E0
#define NVGPU_GPU_IMPL_GK20A 0x0000000A

#define NVGPU_GPU_ARCH_GM200 0x00000120
#define NVGPU_GPU_IMPL_GM20B 0x0000000B

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include <linux/nvgpu-t18x.h>
#endif

#define NVGPU_GPU_BUS_TYPE_NONE         0
#define NVGPU_GPU_BUS_TYPE_AXI         32

#define NVGPU_GPU_FLAGS_HAS_SYNCPOINTS			(1 << 0)
/* MAP_BUFFER_EX with partial mappings */
#define NVGPU_GPU_FLAGS_SUPPORT_PARTIAL_MAPPINGS	(1 << 1)
/* MAP_BUFFER_EX with sparse allocations */
#define NVGPU_GPU_FLAGS_SUPPORT_SPARSE_ALLOCS		(1 << 2)
/* sync fence FDs are available in, e.g., submit_gpfifo */
#define NVGPU_GPU_FLAGS_SUPPORT_SYNC_FENCE_FDS		(1 << 3)
/* NVGPU_IOCTL_CHANNEL_CYCLE_STATS is available */
#define NVGPU_GPU_FLAGS_SUPPORT_CYCLE_STATS		(1 << 4)
/* NVGPU_IOCTL_CHANNEL_CYCLE_STATS_SNAPSHOT is available */
#define NVGPU_GPU_FLAGS_SUPPORT_CYCLE_STATS_SNAPSHOT	(1 << 6)
/* User-space managed address spaces support */
#define NVGPU_GPU_FLAGS_SUPPORT_USERSPACE_MANAGED_AS	(1 << 7)

struct nvgpu_gpu_characteristics {
	__u32 arch;
	__u32 impl;
	__u32 rev;
	__u32 num_gpc;

	__u64 L2_cache_size;               /* bytes */
	__u64 on_board_video_memory_size;  /* bytes */

	__u32 num_tpc_per_gpc; /* the architectural maximum */
	__u32 bus_type;

	__u32 big_page_size; /* the default big page size */
	__u32 compression_page_size;

	__u32 pde_coverage_bit_count;

	/* bit N set ==> big page size 2^N is available in
	   NVGPU_GPU_IOCTL_ALLOC_AS. The default big page size is
	   always available regardless of this field. */
	__u32 available_big_page_sizes;

	__u64 flags;

	__u32 twod_class;
	__u32 threed_class;
	__u32 compute_class;
	__u32 gpfifo_class;
	__u32 inline_to_memory_class;
	__u32 dma_copy_class;

	__u32 gpc_mask; /* enabled GPCs */

	__u32 sm_arch_sm_version; /* sm version */
	__u32 sm_arch_spa_version; /* sm instruction set */
	__u32 sm_arch_warp_count;

	/* IOCTL interface levels by service. -1 if not supported */
	__s16 gpu_ioctl_nr_last;
	__s16 tsg_ioctl_nr_last;
	__s16 dbg_gpu_ioctl_nr_last;
	__s16 ioctl_channel_nr_last;
	__s16 as_ioctl_nr_last;

	__u8 gpu_va_bit_count;
	__u8 reserved;

	__u32 max_fbps_count;
	__u32 fbp_en_mask;
	__u32 max_ltc_per_fbp;
	__u32 max_lts_per_ltc;
	__u32 max_tex_per_tpc;
	__u32 max_gpc_count;
	/* mask of Rop_L2 for each FBP */
	__u32 rop_l2_en_mask[2];


	__u8 chipname[8];

	__u64 gr_compbit_store_base_hw;
	__u32 gr_gobs_per_comptagline_per_slice;
	__u32 num_ltc;
	__u32 lts_per_ltc;
	__u32 cbc_cache_line_size;
	__u32 cbc_comptags_per_line;

	/* MAP_BUFFER_BATCH: the upper limit for num_unmaps and
	 * num_maps */
	__u32 map_buffer_batch_limit;

	__u64 max_freq;

	/* Notes:
	   - This struct can be safely appended with new fields. However, always
	     keep the structure size multiple of 8 and make sure that the binary
	     layout does not change between 32-bit and 64-bit architectures.
	   - If the last field is reserved/padding, it is not
	     generally safe to repurpose the field in future revisions.
	*/
};

struct nvgpu_gpu_get_characteristics {
	/* [in]  size reserved by the user space. Can be 0.
	   [out] full buffer size by kernel */
	__u64 gpu_characteristics_buf_size;

	/* [in]  address of nvgpu_gpu_characteristics buffer. Filled with field
	   values by exactly MIN(buf_size_in, buf_size_out) bytes. Ignored, if
	   buf_size_in is zero.  */
	__u64 gpu_characteristics_buf_addr;
};

#define NVGPU_GPU_COMPBITS_NONE		0
#define NVGPU_GPU_COMPBITS_GPU		(1 << 0)
#define NVGPU_GPU_COMPBITS_CDEH		(1 << 1)
#define NVGPU_GPU_COMPBITS_CDEV		(1 << 2)

struct nvgpu_gpu_prepare_compressible_read_args {
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
	__u32 submit_flags;		/* in (NVGPU_SUBMIT_GPFIFO_FLAGS_) */
	union {
		struct {
			__u32 syncpt_id;
			__u32 syncpt_value;
		};
		__s32 fd;
	} fence;			/* in/out */
	__u32 zbc_color;		/* out */
	__u32 reserved;		/* must be zero */
	__u64 scatterbuffer_offset;	/* in, within handle */
	__u32 reserved2[2];		/* must be zero */
};

struct nvgpu_gpu_mark_compressible_write_args {
	__u32 handle;			/* in, dmabuf fd */
	__u32 valid_compbits;		/* in */
	__u64 offset;			/* in, within handle */
	__u32 zbc_color;		/* in */
	__u32 reserved[3];		/* must be zero */
};

struct nvgpu_alloc_as_args {
	__u32 big_page_size;
	__s32 as_fd;

/*
 * The GPU address space will be managed by the userspace. This has
 * the following changes in functionality:
 *   1. All non-fixed-offset user mappings are rejected (i.e.,
 *      fixed-offset only)
 *   2. Address space does not need to be allocated for fixed-offset
 *      mappings, except to mark sparse address space areas.
 *   3. Maps and unmaps are immediate. In particular, mapping ref
 *      increments at kickoffs and decrements at job completion are
 *      bypassed.
 */
#define NVGPU_GPU_IOCTL_ALLOC_AS_FLAGS_USERSPACE_MANAGED (1 << 0)
	__u32 flags;

	__u32 reserved;			/* must be zero */
};

struct nvgpu_gpu_open_tsg_args {
	__u32 tsg_fd;			/* out, tsg fd */
	__u32 reserved;			/* must be zero */
};

struct nvgpu_gpu_get_tpc_masks_args {
	/* [in]  TPC mask buffer size reserved by userspace. Should be
		 at least sizeof(__u32) * fls(gpc_mask) to receive TPC
		 mask for each GPC.
	   [out] full kernel buffer size
	*/
	__u32 mask_buf_size;
	__u32 reserved;

	/* [in]  pointer to TPC mask buffer. It will receive one
		 32-bit TPC mask per GPC or 0 if GPC is not enabled or
		 not present. This parameter is ignored if
		 mask_buf_size is 0. */
	__u64 mask_buf_addr;
};

struct nvgpu_gpu_open_channel_args {
	__s32 channel_fd;
};

/* L2 cache writeback, optionally invalidate clean lines and flush fb */
struct nvgpu_gpu_l2_fb_args {
	__u32 l2_flush:1;
	__u32 l2_invalidate:1;
	__u32 fb_flush:1;
	__u32 reserved;
} __packed;

struct nvgpu_gpu_inval_icache_args {
	int channel_fd;
	__u32 reserved;
} __packed;

struct nvgpu_gpu_mmu_debug_mode_args {
	__u32 state;
	__u32 reserved;
} __packed;

struct nvgpu_gpu_sm_debug_mode_args {
	int channel_fd;
	__u32 enable;
	__u64 sms;
} __packed;

struct warpstate {
	__u64 valid_warps[2];
	__u64 trapped_warps[2];
	__u64 paused_warps[2];
};

struct nvgpu_gpu_wait_pause_args {
	__u64 pwarpstate;
};

struct nvgpu_gpu_tpc_exception_en_status_args {
	__u64 tpc_exception_en_sm_mask;
};

struct nvgpu_gpu_num_vsms {
	__u32 num_vsms;
	__u32 reserved;
};

struct nvgpu_gpu_vsms_mapping {
	__u64 vsms_map_buf_addr;
};

struct nvgpu_gpu_get_buffer_info_args {
	union {
		struct {
			__u32 dmabuf_fd;   /* dma-buf fd */
		} in;
		struct {
			__u64 id;          /* Unique within live
					    * buffers */
			__u64 length;      /* Allocated length of the
					    * buffer */
			__u64 reserved0;
			__u64 reserved1;
		} out;
	};
};

#define NVGPU_GPU_IOCTL_ZCULL_GET_CTX_SIZE \
	_IOR(NVGPU_GPU_IOCTL_MAGIC, 1, struct nvgpu_gpu_zcull_get_ctx_size_args)
#define NVGPU_GPU_IOCTL_ZCULL_GET_INFO \
	_IOR(NVGPU_GPU_IOCTL_MAGIC, 2, struct nvgpu_gpu_zcull_get_info_args)
#define NVGPU_GPU_IOCTL_ZBC_SET_TABLE	\
	_IOW(NVGPU_GPU_IOCTL_MAGIC, 3, struct nvgpu_gpu_zbc_set_table_args)
#define NVGPU_GPU_IOCTL_ZBC_QUERY_TABLE	\
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 4, struct nvgpu_gpu_zbc_query_table_args)
#define NVGPU_GPU_IOCTL_GET_CHARACTERISTICS   \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 5, struct nvgpu_gpu_get_characteristics)
#define NVGPU_GPU_IOCTL_PREPARE_COMPRESSIBLE_READ \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 6, struct nvgpu_gpu_prepare_compressible_read_args)
#define NVGPU_GPU_IOCTL_MARK_COMPRESSIBLE_WRITE \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 7, struct nvgpu_gpu_mark_compressible_write_args)
#define NVGPU_GPU_IOCTL_ALLOC_AS \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 8, struct nvgpu_alloc_as_args)
#define NVGPU_GPU_IOCTL_OPEN_TSG \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 9, struct nvgpu_gpu_open_tsg_args)
#define NVGPU_GPU_IOCTL_GET_TPC_MASKS \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 10, struct nvgpu_gpu_get_tpc_masks_args)
#define NVGPU_GPU_IOCTL_OPEN_CHANNEL \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 11, struct nvgpu_gpu_open_channel_args)
#define NVGPU_GPU_IOCTL_FLUSH_L2 \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 12, struct nvgpu_gpu_l2_fb_args)
#define NVGPU_GPU_IOCTL_INVAL_ICACHE \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 13, struct nvgpu_gpu_inval_icache_args)
#define NVGPU_GPU_IOCTL_SET_MMUDEBUG_MODE \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 14, struct nvgpu_gpu_mmu_debug_mode_args)
#define NVGPU_GPU_IOCTL_SET_SM_DEBUG_MODE \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 15, struct nvgpu_gpu_sm_debug_mode_args)
#define NVGPU_GPU_IOCTL_WAIT_FOR_PAUSE \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 16, struct nvgpu_gpu_wait_pause_args)
#define NVGPU_GPU_IOCTL_GET_TPC_EXCEPTION_EN_STATUS \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 17, struct nvgpu_gpu_tpc_exception_en_status_args)
#define NVGPU_GPU_IOCTL_NUM_VSMS \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 18, struct nvgpu_gpu_num_vsms)
#define NVGPU_GPU_IOCTL_VSMS_MAPPING \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 19, struct nvgpu_gpu_vsms_mapping)
#define NVGPU_GPU_IOCTL_GET_BUFFER_INFO \
	_IOWR(NVGPU_GPU_IOCTL_MAGIC, 20, struct nvgpu_gpu_get_buffer_info_args)
#define NVGPU_GPU_IOCTL_RESUME_FROM_PAUSE \
	_IO(NVGPU_GPU_IOCTL_MAGIC, 21)
#define NVGPU_GPU_IOCTL_TRIGGER_SUSPEND \
	_IO(NVGPU_GPU_IOCTL_MAGIC, 22)
#define NVGPU_GPU_IOCTL_CLEAR_SM_ERRORS \
	_IO(NVGPU_GPU_IOCTL_MAGIC, 23)

#define NVGPU_GPU_IOCTL_LAST		\
	_IOC_NR(NVGPU_GPU_IOCTL_CLEAR_SM_ERRORS)
#define NVGPU_GPU_IOCTL_MAX_ARG_SIZE	\
	sizeof(struct nvgpu_gpu_prepare_compressible_read_args)

/*
 * /dev/nvhost-tsg-gpu device
 *
 * Opening a '/dev/nvhost-tsg-gpu' device node creates a way to
 * bind/unbind a channel to/from TSG group
 */

#define NVGPU_TSG_IOCTL_MAGIC 'T'

#define NVGPU_TSG_IOCTL_BIND_CHANNEL \
	_IOW(NVGPU_TSG_IOCTL_MAGIC, 1, int)
#define NVGPU_TSG_IOCTL_UNBIND_CHANNEL \
	_IOW(NVGPU_TSG_IOCTL_MAGIC, 2, int)
#define NVGPU_IOCTL_TSG_ENABLE \
	_IO(NVGPU_TSG_IOCTL_MAGIC, 3)
#define NVGPU_IOCTL_TSG_DISABLE \
	_IO(NVGPU_TSG_IOCTL_MAGIC, 4)
#define NVGPU_IOCTL_TSG_PREEMPT \
	_IO(NVGPU_TSG_IOCTL_MAGIC, 5)
#define NVGPU_IOCTL_TSG_SET_PRIORITY \
	_IOW(NVGPU_TSG_IOCTL_MAGIC, 6, struct nvgpu_set_priority_args)

#define NVGPU_TSG_IOCTL_MAX_ARG_SIZE	\
	sizeof(struct nvgpu_set_priority_args)
#define NVGPU_TSG_IOCTL_LAST		\
	_IOC_NR(NVGPU_IOCTL_TSG_SET_PRIORITY)
/*
 * /dev/nvhost-dbg-gpu device
 *
 * Opening a '/dev/nvhost-dbg-gpu' device node creates a new debugger
 * session.  nvgpu channels (for the same module) can then be bound to such a
 * session.
 *
 * Once a nvgpu channel has been bound to a debugger session it cannot be
 * bound to another.
 *
 * As long as there is an open device file to the session, or any bound
 * nvgpu channels it will be valid.  Once all references to the session
 * are removed the session is deleted.
 *
 */

#define NVGPU_DBG_GPU_IOCTL_MAGIC 'D'

/*
 * Binding/attaching a debugger session to an nvgpu channel
 *
 * The 'channel_fd' given here is the fd used to allocate the
 * gpu channel context.  To detach/unbind the debugger session
 * use a channel_fd of -1.
 *
 */
struct nvgpu_dbg_gpu_bind_channel_args {
	__u32 channel_fd; /* in */
	__u32 _pad0[1];
};

#define NVGPU_DBG_GPU_IOCTL_BIND_CHANNEL				\
	_IOWR(NVGPU_DBG_GPU_IOCTL_MAGIC, 1, struct nvgpu_dbg_gpu_bind_channel_args)

/*
 * Register operations
 */
/* valid op values */
#define NVGPU_DBG_GPU_REG_OP_READ_32                             (0x00000000)
#define NVGPU_DBG_GPU_REG_OP_WRITE_32                            (0x00000001)
#define NVGPU_DBG_GPU_REG_OP_READ_64                             (0x00000002)
#define NVGPU_DBG_GPU_REG_OP_WRITE_64                            (0x00000003)
/* note: 8b ops are unsupported */
#define NVGPU_DBG_GPU_REG_OP_READ_08                             (0x00000004)
#define NVGPU_DBG_GPU_REG_OP_WRITE_08                            (0x00000005)

/* valid type values */
#define NVGPU_DBG_GPU_REG_OP_TYPE_GLOBAL                         (0x00000000)
#define NVGPU_DBG_GPU_REG_OP_TYPE_GR_CTX                         (0x00000001)
#define NVGPU_DBG_GPU_REG_OP_TYPE_GR_CTX_TPC                     (0x00000002)
#define NVGPU_DBG_GPU_REG_OP_TYPE_GR_CTX_SM                      (0x00000004)
#define NVGPU_DBG_GPU_REG_OP_TYPE_GR_CTX_CROP                    (0x00000008)
#define NVGPU_DBG_GPU_REG_OP_TYPE_GR_CTX_ZROP                    (0x00000010)
/*#define NVGPU_DBG_GPU_REG_OP_TYPE_FB                           (0x00000020)*/
#define NVGPU_DBG_GPU_REG_OP_TYPE_GR_CTX_QUAD                    (0x00000040)

/* valid status values */
#define NVGPU_DBG_GPU_REG_OP_STATUS_SUCCESS                      (0x00000000)
#define NVGPU_DBG_GPU_REG_OP_STATUS_INVALID_OP                   (0x00000001)
#define NVGPU_DBG_GPU_REG_OP_STATUS_INVALID_TYPE                 (0x00000002)
#define NVGPU_DBG_GPU_REG_OP_STATUS_INVALID_OFFSET               (0x00000004)
#define NVGPU_DBG_GPU_REG_OP_STATUS_UNSUPPORTED_OP               (0x00000008)
#define NVGPU_DBG_GPU_REG_OP_STATUS_INVALID_MASK                 (0x00000010)

struct nvgpu_dbg_gpu_reg_op {
	__u8    op;
	__u8    type;
	__u8    status;
	__u8    quad;
	__u32   group_mask;
	__u32   sub_group_mask;
	__u32   offset;
	__u32   value_lo;
	__u32   value_hi;
	__u32   and_n_mask_lo;
	__u32   and_n_mask_hi;
};

struct nvgpu_dbg_gpu_exec_reg_ops_args {
	__u64 ops; /* pointer to nvgpu_reg_op operations */
	__u32 num_ops;
	__u32 _pad0[1];
};

#define NVGPU_DBG_GPU_IOCTL_REG_OPS					\
	_IOWR(NVGPU_DBG_GPU_IOCTL_MAGIC, 2, struct nvgpu_dbg_gpu_exec_reg_ops_args)

/* Enable/disable/clear event notifications */
struct nvgpu_dbg_gpu_events_ctrl_args {
	__u32 cmd; /* in */
	__u32 _pad0[1];
};

/* valid event ctrl values */
#define NVGPU_DBG_GPU_EVENTS_CTRL_CMD_DISABLE                    (0x00000000)
#define NVGPU_DBG_GPU_EVENTS_CTRL_CMD_ENABLE                     (0x00000001)
#define NVGPU_DBG_GPU_EVENTS_CTRL_CMD_CLEAR                      (0x00000002)

#define NVGPU_DBG_GPU_IOCTL_EVENTS_CTRL				\
	_IOWR(NVGPU_DBG_GPU_IOCTL_MAGIC, 3, struct nvgpu_dbg_gpu_events_ctrl_args)


/* Powergate/Unpowergate control */

#define NVGPU_DBG_GPU_POWERGATE_MODE_ENABLE                                 1
#define NVGPU_DBG_GPU_POWERGATE_MODE_DISABLE                                2

struct nvgpu_dbg_gpu_powergate_args {
	__u32 mode;
} __packed;

#define NVGPU_DBG_GPU_IOCTL_POWERGATE					\
	_IOWR(NVGPU_DBG_GPU_IOCTL_MAGIC, 4, struct nvgpu_dbg_gpu_powergate_args)


/* SMPC Context Switch Mode */
#define NVGPU_DBG_GPU_SMPC_CTXSW_MODE_NO_CTXSW               (0x00000000)
#define NVGPU_DBG_GPU_SMPC_CTXSW_MODE_CTXSW                  (0x00000001)

struct nvgpu_dbg_gpu_smpc_ctxsw_mode_args {
	__u32 mode;
} __packed;

#define NVGPU_DBG_GPU_IOCTL_SMPC_CTXSW_MODE \
	_IOWR(NVGPU_DBG_GPU_IOCTL_MAGIC, 5, struct nvgpu_dbg_gpu_smpc_ctxsw_mode_args)

/* Suspend /Resume SM control */
#define NVGPU_DBG_GPU_SUSPEND_ALL_SMS 1
#define NVGPU_DBG_GPU_RESUME_ALL_SMS  2

struct nvgpu_dbg_gpu_suspend_resume_all_sms_args {
	__u32 mode;
} __packed;

#define NVGPU_DBG_GPU_IOCTL_SUSPEND_RESUME_ALL_SMS			\
	_IOWR(NVGPU_DBG_GPU_IOCTL_MAGIC, 6, struct nvgpu_dbg_gpu_suspend_resume_all_sms_args)

struct nvgpu_dbg_gpu_perfbuf_map_args {
	__u32 dmabuf_fd;	/* in */
	__u32 reserved;
	__u64 mapping_size;	/* in, size of mapped buffer region */
	__u64 offset;		/* out, virtual address of the mapping */
};

struct nvgpu_dbg_gpu_perfbuf_unmap_args {
	__u64 offset;
};

#define NVGPU_DBG_GPU_IOCTL_PERFBUF_MAP \
	_IOWR(NVGPU_DBG_GPU_IOCTL_MAGIC, 7, struct nvgpu_dbg_gpu_perfbuf_map_args)
#define NVGPU_DBG_GPU_IOCTL_PERFBUF_UNMAP \
	_IOWR(NVGPU_DBG_GPU_IOCTL_MAGIC, 8, struct nvgpu_dbg_gpu_perfbuf_unmap_args)

/* Enable/disable PC Sampling */
struct nvgpu_dbg_gpu_pc_sampling_args {
	__u32 enable;
	__u32 _pad0[1];
};

#define NVGPU_DBG_GPU_IOCTL_PC_SAMPLING_DISABLE	0
#define NVGPU_DBG_GPU_IOCTL_PC_SAMPLING_ENABLE	1

#define NVGPU_DBG_GPU_IOCTL_PC_SAMPLING \
	_IOW(NVGPU_DBG_GPU_IOCTL_MAGIC,  9, struct nvgpu_dbg_gpu_pc_sampling_args)

/* Enable/Disable timeouts */
#define NVGPU_DBG_GPU_IOCTL_TIMEOUT_ENABLE                                   1
#define NVGPU_DBG_GPU_IOCTL_TIMEOUT_DISABLE                                  0

struct nvgpu_dbg_gpu_timeout_args {
	__u32 enable;
	__u32 padding;
};

#define NVGPU_DBG_GPU_IOCTL_TIMEOUT \
	_IOW(NVGPU_DBG_GPU_IOCTL_MAGIC, 10, struct nvgpu_dbg_gpu_timeout_args)

#define NVGPU_DBG_GPU_IOCTL_GET_TIMEOUT \
	_IOR(NVGPU_DBG_GPU_IOCTL_MAGIC, 11, struct nvgpu_dbg_gpu_timeout_args)

#define NVGPU_DBG_GPU_IOCTL_LAST		\
	_IOC_NR(NVGPU_DBG_GPU_IOCTL_GET_TIMEOUT)
#define NVGPU_DBG_GPU_IOCTL_MAX_ARG_SIZE		\
	sizeof(struct nvgpu_dbg_gpu_perfbuf_map_args)

/*
 * /dev/nvhost-gpu device
 */

#define NVGPU_IOCTL_MAGIC 'H'
#define NVGPU_NO_TIMEOUT (-1)
#define NVGPU_PRIORITY_LOW 50
#define NVGPU_PRIORITY_MEDIUM 100
#define NVGPU_PRIORITY_HIGH 150
#define NVGPU_TIMEOUT_FLAG_DISABLE_DUMP		0

/* this is also the hardware memory format */
struct nvgpu_gpfifo {
	__u32 entry0; /* first word of gpfifo entry */
	__u32 entry1; /* second word of gpfifo entry */
};

struct nvgpu_get_param_args {
	__u32 value;
} __packed;

struct nvgpu_channel_open_args {
	__s32 channel_fd;
};

struct nvgpu_set_nvmap_fd_args {
	__u32 fd;
} __packed;

#define NVGPU_ALLOC_OBJ_FLAGS_LOCKBOOST_ZERO	(1 << 0)

struct nvgpu_alloc_obj_ctx_args {
	__u32 class_num; /* kepler3d, 2d, compute, etc       */
	__u32 flags;     /* input, output */
	__u64 obj_id;    /* output, used to free later       */
};

struct nvgpu_free_obj_ctx_args {
	__u64 obj_id; /* obj ctx to free */
};

struct nvgpu_alloc_gpfifo_args {
	__u32 num_entries;
#define NVGPU_ALLOC_GPFIFO_FLAGS_VPR_ENABLED	(1 << 0) /* set owner channel of this gpfifo as a vpr channel */
	__u32 flags;

};

struct gk20a_sync_pt_info {
	__u64 hw_op_ns;
};

struct nvgpu_fence {
	__u32 id;        /* syncpoint id or sync fence fd */
	__u32 value;     /* syncpoint value (discarded when using sync fence) */
};

/* insert a wait on the fence before submitting gpfifo */
#define NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT	(1 << 0)
/* insert a fence update after submitting gpfifo and
   return the new fence for others to wait on */
#define NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET	(1 << 1)
/* choose between different gpfifo entry formats */
#define NVGPU_SUBMIT_GPFIFO_FLAGS_HW_FORMAT	(1 << 2)
/* interpret fence as a sync fence fd instead of raw syncpoint fence */
#define NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE	(1 << 3)
/* suppress WFI before fence trigger */
#define NVGPU_SUBMIT_GPFIFO_FLAGS_SUPPRESS_WFI	(1 << 4)
/* skip buffer refcounting during submit */
#define NVGPU_SUBMIT_GPFIFO_FLAGS_SKIP_BUFFER_REFCOUNTING	(1 << 5)

struct nvgpu_submit_gpfifo_args {
	__u64 gpfifo;
	__u32 num_entries;
	__u32 flags;
	struct nvgpu_fence fence;
};

struct nvgpu_map_buffer_args {
	__u32 flags;
#define NVGPU_MAP_BUFFER_FLAGS_ALIGN		0x0
#define NVGPU_MAP_BUFFER_FLAGS_OFFSET		(1 << 0)
#define NVGPU_MAP_BUFFER_FLAGS_KIND_PITCH	0x0
#define NVGPU_MAP_BUFFER_FLAGS_KIND_SPECIFIED	(1 << 1)
#define NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_FALSE	0x0
#define NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_TRUE	(1 << 2)
	__u32 nvmap_handle;
	union {
		__u64 offset; /* valid if _offset flag given (in|out) */
		__u64 align;  /* alignment multiple (0:={1 or n/a})   */
	} offset_alignment;
	__u32 kind;
#define NVGPU_MAP_BUFFER_KIND_GENERIC_16BX2 0xfe
};

struct nvgpu_unmap_buffer_args {
	__u64 offset;
};

struct nvgpu_wait_args {
#define NVGPU_WAIT_TYPE_NOTIFIER	0x0
#define NVGPU_WAIT_TYPE_SEMAPHORE	0x1
	__u32 type;
	__u32 timeout;
	union {
		struct {
			/* handle and offset for notifier memory */
			__u32 dmabuf_fd;
			__u32 offset;
			__u32 padding1;
			__u32 padding2;
		} notifier;
		struct {
			/* handle and offset for semaphore memory */
			__u32 dmabuf_fd;
			__u32 offset;
			/* semaphore payload to wait for */
			__u32 payload;
			__u32 padding;
		} semaphore;
	} condition; /* determined by type field */
};

/* cycle stats support */
struct nvgpu_cycle_stats_args {
	__u32 dmabuf_fd;
} __packed;

struct nvgpu_set_timeout_args {
	__u32 timeout;
} __packed;

struct nvgpu_set_timeout_ex_args {
	__u32 timeout;
	__u32 flags;
};

struct nvgpu_set_priority_args {
	__u32 priority;
} __packed;

#define NVGPU_ZCULL_MODE_GLOBAL		0
#define NVGPU_ZCULL_MODE_NO_CTXSW		1
#define NVGPU_ZCULL_MODE_SEPARATE_BUFFER	2
#define NVGPU_ZCULL_MODE_PART_OF_REGULAR_BUF	3

struct nvgpu_zcull_bind_args {
	__u64 gpu_va;
	__u32 mode;
	__u32 padding;
};

struct nvgpu_set_error_notifier {
	__u64 offset;
	__u64 size;
	__u32 mem;
	__u32 padding;
};

struct nvgpu_notification {
	struct {			/* 0000- */
		__u32 nanoseconds[2];	/* nanoseconds since Jan. 1, 1970 */
	} time_stamp;			/* -0007 */
	__u32 info32;	/* info returned depends on method 0008-000b */
#define	NVGPU_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT	8
#define	NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY	13
#define	NVGPU_CHANNEL_GR_SEMAPHORE_TIMEOUT	24
#define	NVGPU_CHANNEL_GR_ILLEGAL_NOTIFY	25
#define	NVGPU_CHANNEL_FIFO_ERROR_MMU_ERR_FLT	31
#define	NVGPU_CHANNEL_PBDMA_ERROR		32
#define	NVGPU_CHANNEL_RESETCHANNEL_VERIF_ERROR	43
	__u16 info16;	/* info returned depends on method 000c-000d */
	__u16 status;	/* user sets bit 15, NV sets status 000e-000f */
#define	NVGPU_CHANNEL_SUBMIT_TIMEOUT		1
};

/* Enable/disable/clear event notifications */
struct nvgpu_channel_events_ctrl_args {
	__u32 cmd; /* in */
	__u32 _pad0[1];
};

/* valid event ctrl values */
#define NVGPU_IOCTL_CHANNEL_EVENTS_CTRL_CMD_DISABLE 0
#define NVGPU_IOCTL_CHANNEL_EVENTS_CTRL_CMD_ENABLE  1
#define NVGPU_IOCTL_CHANNEL_EVENTS_CTRL_CMD_CLEAR   2

/* cycle stats snapshot buffer support for mode E */
struct nvgpu_cycle_stats_snapshot_args {
	__u32 cmd;		/* in: command to handle     */
	__u32 dmabuf_fd;	/* in: dma buffer handler    */
	__u32 extra;		/* in/out: extra payload e.g.*/
				/*    counter/start perfmon  */
	__u32 pad0[1];
};

/* valid commands to control cycle stats shared buffer */
#define NVGPU_IOCTL_CHANNEL_CYCLE_STATS_SNAPSHOT_CMD_FLUSH   0
#define NVGPU_IOCTL_CHANNEL_CYCLE_STATS_SNAPSHOT_CMD_ATTACH  1
#define NVGPU_IOCTL_CHANNEL_CYCLE_STATS_SNAPSHOT_CMD_DETACH  2

/* disable watchdog per-channel */
struct nvgpu_channel_wdt_args {
	__u32 wdt_status;
	__u32 padding;
};
#define NVGPU_IOCTL_CHANNEL_DISABLE_WDT		1
#define NVGPU_IOCTL_CHANNEL_ENABLE_WDT		2

#define NVGPU_IOCTL_CHANNEL_SET_NVMAP_FD	\
	_IOW(NVGPU_IOCTL_MAGIC, 5, struct nvgpu_set_nvmap_fd_args)
#define NVGPU_IOCTL_CHANNEL_SET_TIMEOUT	\
	_IOW(NVGPU_IOCTL_MAGIC, 11, struct nvgpu_set_timeout_args)
#define NVGPU_IOCTL_CHANNEL_GET_TIMEDOUT	\
	_IOR(NVGPU_IOCTL_MAGIC, 12, struct nvgpu_get_param_args)
#define NVGPU_IOCTL_CHANNEL_SET_PRIORITY	\
	_IOW(NVGPU_IOCTL_MAGIC, 13, struct nvgpu_set_priority_args)
#define NVGPU_IOCTL_CHANNEL_SET_TIMEOUT_EX	\
	_IOWR(NVGPU_IOCTL_MAGIC, 18, struct nvgpu_set_timeout_ex_args)
#define NVGPU_IOCTL_CHANNEL_ALLOC_GPFIFO	\
	_IOW(NVGPU_IOCTL_MAGIC,  100, struct nvgpu_alloc_gpfifo_args)
#define NVGPU_IOCTL_CHANNEL_WAIT		\
	_IOWR(NVGPU_IOCTL_MAGIC, 102, struct nvgpu_wait_args)
#define NVGPU_IOCTL_CHANNEL_CYCLE_STATS	\
	_IOWR(NVGPU_IOCTL_MAGIC, 106, struct nvgpu_cycle_stats_args)
#define NVGPU_IOCTL_CHANNEL_SUBMIT_GPFIFO	\
	_IOWR(NVGPU_IOCTL_MAGIC, 107, struct nvgpu_submit_gpfifo_args)
#define NVGPU_IOCTL_CHANNEL_ALLOC_OBJ_CTX	\
	_IOWR(NVGPU_IOCTL_MAGIC, 108, struct nvgpu_alloc_obj_ctx_args)
#define NVGPU_IOCTL_CHANNEL_FREE_OBJ_CTX	\
	_IOR(NVGPU_IOCTL_MAGIC,  109, struct nvgpu_free_obj_ctx_args)
#define NVGPU_IOCTL_CHANNEL_ZCULL_BIND		\
	_IOWR(NVGPU_IOCTL_MAGIC, 110, struct nvgpu_zcull_bind_args)
#define NVGPU_IOCTL_CHANNEL_SET_ERROR_NOTIFIER  \
	_IOWR(NVGPU_IOCTL_MAGIC, 111, struct nvgpu_set_error_notifier)
#define NVGPU_IOCTL_CHANNEL_OPEN	\
	_IOR(NVGPU_IOCTL_MAGIC,  112, struct nvgpu_channel_open_args)
#define NVGPU_IOCTL_CHANNEL_ENABLE	\
	_IO(NVGPU_IOCTL_MAGIC,  113)
#define NVGPU_IOCTL_CHANNEL_DISABLE	\
	_IO(NVGPU_IOCTL_MAGIC,  114)
#define NVGPU_IOCTL_CHANNEL_PREEMPT	\
	_IO(NVGPU_IOCTL_MAGIC,  115)
#define NVGPU_IOCTL_CHANNEL_FORCE_RESET	\
	_IO(NVGPU_IOCTL_MAGIC,  116)
#define NVGPU_IOCTL_CHANNEL_EVENTS_CTRL	\
	_IOW(NVGPU_IOCTL_MAGIC,  117, struct nvgpu_channel_events_ctrl_args)
#define NVGPU_IOCTL_CHANNEL_CYCLE_STATS_SNAPSHOT	\
	_IOWR(NVGPU_IOCTL_MAGIC, 118, struct nvgpu_cycle_stats_snapshot_args)
#define NVGPU_IOCTL_CHANNEL_WDT \
	_IOW(NVGPU_IOCTL_MAGIC, 119, struct nvgpu_channel_wdt_args)

#define NVGPU_IOCTL_CHANNEL_LAST	\
	_IOC_NR(NVGPU_IOCTL_CHANNEL_WDT)
#define NVGPU_IOCTL_CHANNEL_MAX_ARG_SIZE sizeof(struct nvgpu_submit_gpfifo_args)

/*
 * /dev/nvhost-as-gpu device
 *
 * Opening a '/dev/nvhost-as-gpu' device node creates a new address
 * space.  nvgpu channels (for the same module) can then be bound to such an
 * address space to define the addresses it has access to.
 *
 * Once a nvgpu channel has been bound to an address space it cannot be
 * unbound.  There is no support for allowing an nvgpu channel to change from
 * one address space to another (or from one to none).
 *
 * As long as there is an open device file to the address space, or any bound
 * nvgpu channels it will be valid.  Once all references to the address space
 * are removed the address space is deleted.
 *
 */

#define NVGPU_AS_IOCTL_MAGIC 'A'

/*
 * Allocating an address space range:
 *
 * Address ranges created with this ioctl are reserved for later use with
 * fixed-address buffer mappings.
 *
 * If _FLAGS_FIXED_OFFSET is specified then the new range starts at the 'offset'
 * given.  Otherwise the address returned is chosen to be a multiple of 'align.'
 *
 */
struct nvgpu32_as_alloc_space_args {
	__u32 pages;     /* in, pages */
	__u32 page_size; /* in, bytes */
	__u32 flags;     /* in */
#define NVGPU_AS_ALLOC_SPACE_FLAGS_FIXED_OFFSET 0x1
#define NVGPU_AS_ALLOC_SPACE_FLAGS_SPARSE 0x2
	union {
		__u64 offset; /* inout, byte address valid iff _FIXED_OFFSET */
		__u64 align;  /* in, alignment multiple (0:={1 or n/a}) */
	} o_a;
};

struct nvgpu_as_alloc_space_args {
	__u32 pages;     /* in, pages */
	__u32 page_size; /* in, bytes */
	__u32 flags;     /* in */
	__u32 padding;     /* in */
	union {
		__u64 offset; /* inout, byte address valid iff _FIXED_OFFSET */
		__u64 align;  /* in, alignment multiple (0:={1 or n/a}) */
	} o_a;
};

/*
 * Releasing an address space range:
 *
 * The previously allocated region starting at 'offset' is freed.  If there are
 * any buffers currently mapped inside the region the ioctl will fail.
 */
struct nvgpu_as_free_space_args {
	__u64 offset; /* in, byte address */
	__u32 pages;     /* in, pages */
	__u32 page_size; /* in, bytes */
};

/*
 * Binding a nvgpu channel to an address space:
 *
 * A channel must be bound to an address space before allocating a gpfifo
 * in nvgpu.  The 'channel_fd' given here is the fd used to allocate the
 * channel.  Once a channel has been bound to an address space it cannot
 * be unbound (except for when the channel is destroyed).
 */
struct nvgpu_as_bind_channel_args {
	__u32 channel_fd; /* in */
} __packed;

/*
 * Mapping nvmap buffers into an address space:
 *
 * The start address is the 'offset' given if _FIXED_OFFSET is specified.
 * Otherwise the address returned is a multiple of 'align.'
 *
 * If 'page_size' is set to 0 the nvmap buffer's allocation alignment/sizing
 * will be used to determine the page size (largest possible).  The page size
 * chosen will be returned back to the caller in the 'page_size' parameter in
 * that case.
 */
struct nvgpu_as_map_buffer_args {
	__u32 flags;		/* in/out */
#define NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET	    (1 << 0)
#define NVGPU_AS_MAP_BUFFER_FLAGS_CACHEABLE	    (1 << 2)
#define NVGPU_AS_MAP_BUFFER_FLAGS_UNMAPPED_PTE	    (1 << 5)
#define NVGPU_AS_MAP_BUFFER_FLAGS_MAPPABLE_COMPBITS (1 << 6)
	__u32 reserved;		/* in */
	__u32 dmabuf_fd;	/* in */
	__u32 page_size;	/* inout, 0:= best fit to buffer */
	union {
		__u64 offset; /* inout, byte address valid iff _FIXED_OFFSET */
		__u64 align;  /* in, alignment multiple (0:={1 or n/a})   */
	} o_a;
};

 /*
 * Mapping dmabuf fds into an address space:
 *
 * The caller requests a mapping to a particular page 'kind'.
 *
 * If 'page_size' is set to 0 the dmabuf's alignment/sizing will be used to
 * determine the page size (largest possible).  The page size chosen will be
 * returned back to the caller in the 'page_size' parameter in that case.
 */
struct nvgpu_as_map_buffer_ex_args {
	__u32 flags;		/* in/out */
#define NV_KIND_DEFAULT -1
	__s32 kind;		/* in (-1 represents default) */
	__u32 dmabuf_fd;	/* in */
	__u32 page_size;	/* inout, 0:= best fit to buffer */

	__u64 buffer_offset;	/* in, offset of mapped buffer region */
	__u64 mapping_size;	/* in, size of mapped buffer region */

	__u64 offset;		/* in/out, we use this address if flag
				 * FIXED_OFFSET is set. This will fail
				 * if space is not properly allocated. The
				 * actual virtual address to which we mapped
				 * the buffer is returned in this field. */
};

/*
 * Get info about buffer compbits. Requires that buffer is mapped with
 * NVGPU_AS_MAP_BUFFER_FLAGS_MAPPABLE_COMPBITS.
 *
 * The compbits for a mappable buffer are organized in a mappable
 * window to the compbits store. In case the window contains comptags
 * for more than one buffer, the buffer comptag line index may differ
 * from the window comptag line index.
 */
struct nvgpu_as_get_buffer_compbits_info_args {

	/* in: address of an existing buffer mapping */
	__u64 mapping_gva;

	/* out: size of compbits mapping window (bytes) */
	__u64 compbits_win_size;

	/* out: comptag line index of the window start */
	__u32 compbits_win_ctagline;

	/* out: comptag line index of the buffer mapping */
	__u32 mapping_ctagline;

/* Buffer uses compbits */
#define NVGPU_AS_GET_BUFFER_COMPBITS_INFO_FLAGS_HAS_COMPBITS    (1 << 0)

/* Buffer compbits are mappable */
#define NVGPU_AS_GET_BUFFER_COMPBITS_INFO_FLAGS_MAPPABLE        (1 << 1)

/* Buffer IOVA addresses are discontiguous */
#define NVGPU_AS_GET_BUFFER_COMPBITS_INFO_FLAGS_DISCONTIG_IOVA  (1 << 2)

	/* out */
	__u32 flags;

	__u32 reserved1;
};

/*
 * Map compbits of a mapped buffer to the GPU address space. The
 * compbits mapping is automatically unmapped when the buffer is
 * unmapped.
 *
 * The compbits mapping always uses small pages, it is read-only, and
 * is GPU cacheable. The mapping is a window to the compbits
 * store. The window may not be exactly the size of the cache lines
 * for the buffer mapping.
 */
struct nvgpu_as_map_buffer_compbits_args {

	/* in: address of an existing buffer mapping */
	__u64 mapping_gva;

	/* in: gva to the mapped compbits store window when
	 * FIXED_OFFSET is set. Otherwise, ignored and should be be 0.
	 *
	 * For FIXED_OFFSET mapping:
	 * - If compbits are already mapped compbits_win_gva
	 *   must match with the previously mapped gva.
	 * - The user must have allocated enough GVA space for the
	 *   mapping window (see compbits_win_size in
	 *   nvgpu_as_get_buffer_compbits_info_args)
	 *
	 * out: gva to the mapped compbits store window */
	__u64 compbits_win_gva;

	/* in: reserved, must be 0
	   out: physical or IOMMU address for mapping */
	union {
		/* contiguous iova addresses */
		__u64 mapping_iova;

		/* buffer to receive discontiguous iova addresses (reserved) */
		__u64 mapping_iova_buf_addr;
	};

	/* in: Buffer size (in bytes) for discontiguous iova
	 * addresses. Reserved, must be 0. */
	__u64 mapping_iova_buf_size;

#define NVGPU_AS_MAP_BUFFER_COMPBITS_FLAGS_FIXED_OFFSET        (1 << 0)
	__u32 flags;
	__u32 reserved1;
};

/*
 * Unmapping a buffer:
 *
 * To unmap a previously mapped buffer set 'offset' to the offset returned in
 * the mapping call.  This includes where a buffer has been mapped into a fixed
 * offset of a previously allocated address space range.
 */
struct nvgpu_as_unmap_buffer_args {
	__u64 offset; /* in, byte address */
};


struct nvgpu_as_va_region {
	__u64 offset;
	__u32 page_size;
	__u32 reserved;
	__u64 pages;
};

struct nvgpu_as_get_va_regions_args {
	__u64 buf_addr; /* Pointer to array of nvgpu_as_va_region:s.
			 * Ignored if buf_size is 0 */
	__u32 buf_size; /* in:  userspace buf size (in bytes)
			   out: kernel buf size    (in bytes) */
	__u32 reserved;
};

struct nvgpu_as_map_buffer_batch_args {
	__u64 unmaps; /* ptr to array of nvgpu_unmap_buffer_args */
	__u64 maps;   /* ptr to array of nvgpu_as_map_buffer_ex_args */
	__u32 num_unmaps; /* in: number of unmaps
			   * out: on error, number of successful unmaps */
	__u32 num_maps;   /* in: number of maps
			   * out: on error, number of successful maps */
	__u64 reserved;
};

#define NVGPU_AS_IOCTL_BIND_CHANNEL \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 1, struct nvgpu_as_bind_channel_args)
#define NVGPU32_AS_IOCTL_ALLOC_SPACE \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 2, struct nvgpu32_as_alloc_space_args)
#define NVGPU_AS_IOCTL_FREE_SPACE \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 3, struct nvgpu_as_free_space_args)
#define NVGPU_AS_IOCTL_MAP_BUFFER \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 4, struct nvgpu_as_map_buffer_args)
#define NVGPU_AS_IOCTL_UNMAP_BUFFER \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 5, struct nvgpu_as_unmap_buffer_args)
#define NVGPU_AS_IOCTL_ALLOC_SPACE \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 6, struct nvgpu_as_alloc_space_args)
#define NVGPU_AS_IOCTL_MAP_BUFFER_EX \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 7, struct nvgpu_as_map_buffer_ex_args)
#define NVGPU_AS_IOCTL_GET_VA_REGIONS \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 8, struct nvgpu_as_get_va_regions_args)
#define NVGPU_AS_IOCTL_GET_BUFFER_COMPBITS_INFO \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 9, struct nvgpu_as_get_buffer_compbits_info_args)
#define NVGPU_AS_IOCTL_MAP_BUFFER_COMPBITS \
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 10, struct nvgpu_as_map_buffer_compbits_args)
#define NVGPU_AS_IOCTL_MAP_BUFFER_BATCH	\
	_IOWR(NVGPU_AS_IOCTL_MAGIC, 11, struct nvgpu_as_map_buffer_batch_args)

#define NVGPU_AS_IOCTL_LAST            \
	_IOC_NR(NVGPU_AS_IOCTL_MAP_BUFFER_BATCH)
#define NVGPU_AS_IOCTL_MAX_ARG_SIZE	\
	sizeof(struct nvgpu_as_map_buffer_ex_args)

#endif
