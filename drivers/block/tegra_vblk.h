/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _TEGRA_VBLK_H_
#define _TEGRA_VBLK_H_

#include <linux/types.h> /* size_t */
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/tegra-ivc.h>
#include <linux/workqueue.h>
#include <linux/mmc/ioctl.h>
#include <linux/mutex.h>

#define DRV_NAME "tegra_hv_vblk"


/* Minor number and partition management. */
#define VBLK_MINORS 16

#define IVC_RESET_RETRIES	30

#define VS_LOG_HEADS 4
#define VS_LOG_SECTS 16

enum cmd_mode {
	VBLK_GET_CONFIG = 2,
	VBLK_READ,
	VBLK_WRITE,
	VBLK_FLUSH,
	VBLK_IOCTL,
	CMD_PASS_THROUGH = 0x55aaaa55,
	UNKNOWN_CMD = 0xffffffff,
};

#define SECTOR_SIZE 512

/* Request type features supported */
#define VBLK_BLK_REQ_F          (1 << 0)
#define VBLK_IOCTL_REQ_F        (1 << 1)
#define VBLK_REQ_F_NONE         0

/* Block Request Operation type features supported */
#define VBLK_READ_OP_F          (1 << 0)
#define VBLK_WRITE_OP_F         (1 << 1)
#define VBLK_FLUSH_OP_F         (1 << 2)
#define VBLK_OP_F_NONE          0

#pragma pack(push)
#pragma pack(1)
struct ivc_request {
	enum cmd_mode cmd;      /* 0:read 1:write */
	uint64_t blk_offset; /* Blk cursor */
	union {
		uint32_t num_blks; /* Total Block number to transfer */
		uint32_t ioctl_len; /* Length of the mempool area associated
				       with ioctl */
	};
	uint32_t serial_number;
	uint32_t data_offset; /* Offset into mempool for data region */
};

struct ivc_result {
	uint32_t status;        /* 0 for success, < 0 for error */
	uint32_t num_blks; /* number of blocks to complete */
	uint32_t serial_number;
};

struct virtual_storage_configinfo {
	enum cmd_mode cmd;              /* 2:configinfo */
	uint64_t num_blks;              /* Total number of blocks */
	uint32_t hardblk_size;          /* Block Size */
	uint32_t max_blks_per_io;       /* Limit number of Blocks per I/O*/
	uint32_t virtual_storage_ver;	/* Version of virtual storage */
	uint32_t req_types_supported;	/* Types of requests supported
					   by the virtual block device */
	uint32_t blk_req_ops_supported;	/* Allowed operations by block
					   requests */
};

struct combo_cmd_t {
	uint32_t cmd;
	uint32_t arg;
	uint32_t response[4];
	uint32_t buf_offset;
	uint32_t data_len;
};

struct combo_info_t {
	enum     cmd_mode cmd;
	uint32_t count;
	int32_t  result;
};

#define VBLK_SG_IO_ID 0x1001

#define VBLK_SG_MAX_CMD_LEN 16

enum scsi_data_direction {
        SCSI_BIDIRECTIONAL = 0,
        SCSI_TO_DEVICE = 1,
        SCSI_FROM_DEVICE = 2,
        SCSI_DATA_NONE = 3,
};

struct vblk_sg_io_hdr
{
    int32_t data_direction;     /* [i] data transfer direction  */
    uint8_t cmd_len;            /* [i] SCSI command length */
    uint8_t mx_sb_len;          /* [i] max length to write to sbp */
    uint32_t dxfer_len;         /* [i] byte count of data transfer */
    uint32_t xfer_arg_offset;  /* [i], [*io] offset to data transfer memory */
    uint32_t cmdp_arg_offset;   /* [i], [*i] offset to command to perform */
    uint32_t sbp_arg_offset;    /* [i], [*o] offset to sense_buffer memory */
    uint32_t status;            /* [o] scsi status */
    uint8_t sb_len_wr;          /* [o] byte count actually written to sbp */
};

#pragma pack(pop)

#define MAX_VSC_REQS 32

struct vblk_ioctl_req {
	void *ioctl_buf;
	uint32_t ioctl_len;
};

struct vsc_request {
	struct ivc_request ivc_req;
	struct request *req;
	struct req_iterator iter;
	struct vblk_ioctl_req *ioctl_req;
	void *mempool_virt;
	uint32_t mempool_offset;
	uint32_t mempool_len;
	uint32_t id;
	struct vblk_dev* vblkdev;
};

enum IOCTL_STATUS {
	IOCTL_SUCCESS = 0,
	IOCTL_FAILURE,
	IOCTL_IDLE,
	IOCTL_WAIT_BUS,
	IOCTL_PROGRESS,
	IOCTL_UNKNOWN = 0xffffffff,
};

/*
* The drvdata of virtual device.
*/
struct vblk_dev {
	struct virtual_storage_configinfo config;
	uint64_t size;                   /* Device size in bytes */
	short users;                     /* How many users */
	short media_change;              /* Flag a media change? */
	spinlock_t lock;                 /* For mutual exclusion */
	struct request_queue *queue;     /* The device request queue */
	struct gendisk *gd;              /* The gendisk structure */
	uint32_t ivc_id;
	uint32_t ivm_id;
	struct tegra_hv_ivc_cookie *ivck;
	struct tegra_hv_ivm_cookie *ivmk;
	uint32_t devnum;
	bool initialized;
	struct work_struct init;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct device *device;
	void *shared_buffer;
	uint8_t *cmd_frame;
	struct mutex ioctl_lock;
	spinlock_t queue_lock;
	enum IOCTL_STATUS ioctl_status;
	struct completion ioctl_complete;
	struct vsc_request reqs[MAX_VSC_REQS];
	DECLARE_BITMAP(pending_reqs, MAX_VSC_REQS);
	uint32_t inflight_reqs;
	uint32_t max_requests;
	struct mutex req_lock;
};
#endif
