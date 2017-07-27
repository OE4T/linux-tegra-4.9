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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h> /* printk() */
#include <linux/slab.h>   /* kmalloc() */
#include <linux/fs.h>   /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/types.h> /* size_t */
#include <linux/fcntl.h> /* O_ACCMODE */
#include <linux/hdreg.h> /* HDIO_GETGEO */
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h> /* invalidate_bdev */
#include <linux/bio.h>
#include <linux/interrupt.h>
#include <soc/tegra/chip-id.h>
#include <linux/tegra-ivc.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/mmc/ioctl.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm-generic/bug.h>
#include <linux/dma-mapping.h>

#define DRV_NAME "tegra_hv_vblk"

static int vblk_major;

/* Minor number and partition management. */
#define VBLK_MINORS 16

#define IVC_RESET_RETRIES	30

#define VS_LOG_HEADS 4
#define VS_LOG_SECTS 16

enum cmd_mode {
	R_TRANSFER = 0,
	W_TRANSFER,
	R_CONFIG,
	R_TRANSFER_SHARED_BUF,
	W_TRANSFER_SHARED_BUF,
	CMD_PASS_THROUGH = 0x55aaaa55,
	UNKNOWN_CMD = 0xffffffff,
};

#define SECTOR_SIZE 512

#pragma pack(push)
#pragma pack(1)
struct ivc_blk_request {
	enum cmd_mode cmd;      /* 0:read 1:write */
	uint64_t blk_offset; /* Blk cursor */
	uint32_t num_blks; /* Total Block number to transfer */
	uint32_t serial_number;
	uint32_t mempool_offset;
};

struct ivc_blk_result {
	uint32_t status;        /* 0 for success, < 0 for error */
	uint32_t num_blks; /* number of blocks to complete */
	uint32_t serial_number;
};

struct virtual_storage_configinfo {
	enum cmd_mode cmd;                /* 2:configinfo */
	uint64_t num_blks;                /* Total number of blocks */
	uint32_t hardblk_size;           /* Block Size */
	uint32_t max_blks_per_io;       /* Limit number of Blocks per I/O*/
	uint32_t virtual_storage_ver;     /* Version of virtual storage */
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
#pragma pack(pop)

#define MAX_VSC_REQS 32

struct vsc_request {
	struct ivc_blk_request ivc_req;
	struct request *req;
	struct req_iterator iter;
	void *mempool_virt;
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
 * ToDo:
 * we should replace mmc_combo_cmd_info and MMC_COMBO_IOC_CMD
 * with mmc_ioc_multi_cmd and MMC_IOC_MULTI_CMD _IOWR if mnand
 * tools use kernel-4.4 default command mmc_ioc_multi_cmd
 */
struct mmc_combo_cmd_info {
	uint8_t  num_of_combo_cmds;
	struct mmc_ioc_cmd *mmc_ioc_cmd_list;
};
#define MMC_COMBO_IOC_CMD _IOWR(MMC_BLOCK_MAJOR, 1, struct mmc_combo_cmd_info)

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
	struct mmc_combo_cmd_info mcci;
	struct mutex ioctl_lock;
	spinlock_t queue_lock;
	enum IOCTL_STATUS ioctl_status;
	struct completion ioctl_complete;
	struct vsc_request reqs[MAX_VSC_REQS];
	DECLARE_BITMAP(pending_reqs, MAX_VSC_REQS);
	uint32_t inflight_reqs;
	uint32_t max_requests;
	struct mutex req_lock;
	struct mutex ivc_lock;
};

/* MMCSD passthrough commands used */
#define MMC_SEND_CID                2
#define MMC_SWITCH                  6
#define MMC_SEND_EXT_CSD            8
#define MMC_IF_COND                 8
#define MMC_SEND_STATUS             13
#define MMC_READ_SINGLE_BLOCK       17     /* read single block */
#define MMC_READ_MULTIPLE_BLOCK     18     /* read multiple blocks */
#define MMC_WRITE_BLOCK             24     /* write single block */
#define MMC_WRITE_MULTIPLE_BLOCK    25     /* write multiple blocks */
#define MMC_GEN_CMD                 56     /* Hynix F26/Toshiba specific */
#define MMC_MANF0_CMD               60     /* Hynix F20 specific */
#define MMC_STOP_TRANSMISSION       12
#define MMC_SECTOR_START            32     /* S */
#define MMC_SECTOR_END              33     /* S */
#define MMC_ERASE_GROUP_START       35     /* S */
#define MMC_ERASE_GROUP_END         36     /* S */
#define MMC_ERASE                   38     /* S */

/**
 * vblk_get_req: Get a handle to free vsc request.
 */
static struct vsc_request *vblk_get_req(struct vblk_dev *vblkdev)
{
	struct vsc_request *req = NULL;
	unsigned long bit;

	mutex_lock(&vblkdev->req_lock);
	bit = find_first_zero_bit(vblkdev->pending_reqs, vblkdev->max_requests);
	if (bit < vblkdev->max_requests) {
		req = &vblkdev->reqs[bit];
		req->id = bit;
		req->ivc_req.serial_number = bit;
		req->vblkdev = vblkdev;
		set_bit(bit, vblkdev->pending_reqs);
		vblkdev->inflight_reqs++;
	}
	mutex_unlock(&vblkdev->req_lock);

	return req;
}

static struct vsc_request *vblk_get_req_by_sr_num(struct vblk_dev *vblkdev,
		uint32_t num)
{
	struct vsc_request *req;

	if (num >= vblkdev->max_requests)
		return NULL;

	mutex_lock(&vblkdev->req_lock);
	req = &vblkdev->reqs[num];
	if (test_bit(req->id, vblkdev->pending_reqs) == 0) {
		dev_err(vblkdev->device,
			"sr_num: Request index %d is not active!\n",
			req->id);
		req = NULL;
	}
	mutex_unlock(&vblkdev->req_lock);

	/* Assuming serial number is same as index into request array */
	return req;
}

/**
 * vblk_put_req: Free an active vsc request.
 */
static void vblk_put_req(struct vsc_request *req)
{
	struct vblk_dev *vblkdev;

	vblkdev = req->vblkdev;
	if (vblkdev == NULL) {
		pr_err("Request %d does not have valid vblkdev!\n",
				req->id);
		return;
	}

	if (req->id >= vblkdev->max_requests) {
		dev_err(vblkdev->device, "Request Index %d out of range!\n",
				req->id);
		return;
	}

	mutex_lock(&vblkdev->req_lock);
	if (req != &vblkdev->reqs[req->id]) {
		dev_err(vblkdev->device,
			"Request Index %d does not match with the request!\n",
				req->id);
		goto exit;
	}

	if (test_bit(req->id, vblkdev->pending_reqs) == 0) {
		dev_err(vblkdev->device,
			"Request index %d is not active!\n",
			req->id);
	} else {
		clear_bit(req->id, vblkdev->pending_reqs);
		memset(req, 0, sizeof(struct vsc_request));
		vblkdev->inflight_reqs--;
	}
exit:
	mutex_unlock(&vblkdev->req_lock);
}

static int vblk_send_config_cmd(struct vblk_dev *vblkdev)
{
	struct ivc_blk_request *ivc_blk_req;
	int i = 0;

	/* This while loop exits as long as the remote endpoint cooperates. */
	if (tegra_hv_ivc_channel_notified(vblkdev->ivck) != 0) {
		pr_notice("vblk: send_config wait for ivc channel reset\n");
		while (tegra_hv_ivc_channel_notified(vblkdev->ivck) != 0) {
			if (i++ > IVC_RESET_RETRIES) {
				dev_err(vblkdev->device, "ivc reset timeout\n");
				return -EIO;
			}
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(usecs_to_jiffies(1));
		}
	}
	ivc_blk_req = (struct ivc_blk_request *)
		tegra_hv_ivc_write_get_next_frame(vblkdev->ivck);
	if (IS_ERR_OR_NULL(ivc_blk_req)) {
		dev_err(vblkdev->device, "no empty frame for write\n");
		return -EIO;
	}

	ivc_blk_req->cmd = R_CONFIG;
	ivc_blk_req->blk_offset = 0;
	ivc_blk_req->num_blks = 0;
	ivc_blk_req->mempool_offset = 0;

	dev_info(vblkdev->device, "send config cmd to ivc #%d\n",
		vblkdev->ivc_id);

	if (tegra_hv_ivc_write_advance(vblkdev->ivck)) {
		dev_err(vblkdev->device, "ivc write failed\n");
		return -EIO;
	}

	return 0;
}

static int vblk_prepare_passthrough_cmd(struct vblk_dev *vblkdev,
	void __user *user, uint8_t combo)
{
	int err = 0;
	struct combo_info_t *combo_info;
	struct combo_cmd_t *combo_cmd;
	int i = 0;
	u8 num_cmd;
	struct mmc_combo_cmd_info mcci = {0};
	struct mmc_ioc_cmd ic;
	struct mmc_ioc_cmd __user *usr_ptr;
	uint32_t combo_cmd_size;
	unsigned long remainingbytes;
	uint8_t *tmpaddr;

	combo_info = (struct combo_info_t *)vblkdev->cmd_frame;

	if (combo) {
		tmpaddr = (uint8_t *)&mcci;
		if (copy_from_user((void *)tmpaddr, user,
			sizeof(struct mmc_combo_cmd_info))) {
			err = -EFAULT;
			goto out;
		}

		num_cmd = mcci.num_of_combo_cmds;
		if (num_cmd < 1) {
			err = -EINVAL;
			goto out;
		}

		usr_ptr = (void * __user)mcci.mmc_ioc_cmd_list;
	} else {
		num_cmd = 1;
		usr_ptr = (void * __user)user;
	}
	combo_info->cmd = CMD_PASS_THROUGH;
	combo_info->count = num_cmd;

	combo_cmd = (struct combo_cmd_t *)(vblkdev->cmd_frame +
		sizeof(struct combo_info_t));

	combo_cmd_size = sizeof(struct combo_info_t) +
		sizeof(struct combo_cmd_t) * combo_info->count;

	tmpaddr = (uint8_t *)&ic;
	for (i = 0; i < combo_info->count; i++) {
		if (copy_from_user((void *)tmpaddr, usr_ptr, sizeof(ic))) {
			err = -EFAULT;
			goto out;
		}
		combo_cmd->cmd = ic.opcode;
		combo_cmd->arg = ic.arg;
		combo_cmd->data_len = (uint32_t)(ic.blksz * ic.blocks);
		combo_cmd_size += combo_cmd->data_len;
		if (combo_cmd_size > vblkdev->ivck->frame_size) {
			dev_err(vblkdev->device,
				" ivc frame has no enough space to serve ioctl\n");
			err = -EFAULT;
			goto out;
		}
		combo_cmd->buf_offset = combo_cmd_size - combo_cmd->data_len;

		if (ic.write_flag && combo_cmd->data_len) {
			remainingbytes = copy_from_user((
				(void *)vblkdev->cmd_frame +
				combo_cmd->buf_offset),
				(void __user *)(unsigned long)ic.data_ptr,
				(u64)combo_cmd->data_len);
			if (remainingbytes) {
				dev_err(vblkdev->device,
					"copy from user remainingbytes = %ld\n",
					remainingbytes);
				err = -EFAULT;
				goto out;
			}
		}
		combo_cmd++;
		usr_ptr++;
	}

	vblkdev->ioctl_status = IOCTL_WAIT_BUS;
	queue_work_on(WORK_CPU_UNBOUND, vblkdev->wq, &vblkdev->work);

out:
	return err;
}

static int vblk_get_configinfo(struct vblk_dev *vblkdev)
{
	uint32_t len;

	dev_info(vblkdev->device, "get config data from ivc #%d\n",
		vblkdev->ivc_id);

	len = tegra_hv_ivc_read(vblkdev->ivck, (void *)&(vblkdev->config),
		sizeof(struct virtual_storage_configinfo));
	if (len != sizeof(struct virtual_storage_configinfo))
		return -EIO;

	if (vblkdev->config.cmd != R_CONFIG)
		return -EIO;

	if (vblkdev->config.num_blks == 0) {
		dev_err(vblkdev->device, "controller init failed\n");
		return -EINVAL;
	}

	return 0;
}

static void req_error_handler(struct vblk_dev *vblkdev, struct request *breq)
{
	dev_err(vblkdev->device,
		"Error for request pos %llx direction %d size %x\n",
		(blk_rq_pos(breq) * (uint64_t)SECTOR_SIZE),
		rq_data_dir(breq),
		blk_rq_bytes(breq));

	spin_lock(vblkdev->queue->queue_lock);
	__blk_end_request_all(breq, -EIO);
	spin_unlock(vblkdev->queue->queue_lock);
}

static void ioctl_handler(struct vblk_dev *vblkdev)
{
	struct combo_info_t *combo_info;
	uint8_t *frame_ptr;

	if (vblkdev->ioctl_status == IOCTL_WAIT_BUS) {
		frame_ptr = tegra_hv_ivc_write_get_next_frame(vblkdev->ivck);

		if (IS_ERR_OR_NULL(frame_ptr)) {
			return;
		}

		vblkdev->ioctl_status = IOCTL_PROGRESS;
		memcpy((void *)frame_ptr, (void *)vblkdev->cmd_frame,
			vblkdev->ivck->frame_size);

		if (tegra_hv_ivc_write_advance(vblkdev->ivck))  {
			vblkdev->ioctl_status = IOCTL_FAILURE;
			complete(&vblkdev->ioctl_complete);
			dev_err(vblkdev->device, "send combo_cmd failed\n");
		}
	} else if (vblkdev->ioctl_status == IOCTL_PROGRESS) {
		combo_info = (struct combo_info_t *)
			tegra_hv_ivc_read_get_next_frame(vblkdev->ivck);

		if (IS_ERR_OR_NULL(combo_info))
			return;

		if (combo_info->cmd == CMD_PASS_THROUGH) {
			if (combo_info->result) {
				vblkdev->ioctl_status = IOCTL_FAILURE;
				goto out;
			}

			memcpy((void *)vblkdev->cmd_frame, (void *)combo_info,
				 vblkdev->ivck->frame_size);

			vblkdev->ioctl_status = IOCTL_IDLE;
out:
			tegra_hv_ivc_read_advance(vblkdev->ivck);
			complete(&vblkdev->ioctl_complete);
			queue_work_on(WORK_CPU_UNBOUND, vblkdev->wq, &vblkdev->work);
		}
	}
}

/**
 * complete_bio_req: Complete a bio request after server is
 *		done processing the request.
 */

static bool complete_bio_req(struct vblk_dev *vblkdev)
{
	struct ivc_blk_result *ivc_blk_res;
	int status = 0;
	struct bio_vec bvec;
	size_t size;
	size_t total_size = 0;
	struct vsc_request *vsc_req = NULL;
	struct request *bio_req;
	void *buffer;

	/* Take a lock to protect concurrent accesses to IVC queue */
	mutex_lock(&vblkdev->ivc_lock);

	if (!tegra_hv_ivc_can_read(vblkdev->ivck))
		goto no_valid_io;

	ivc_blk_res = (struct ivc_blk_result *)
		tegra_hv_ivc_read_get_next_frame(vblkdev->ivck);
	if (IS_ERR_OR_NULL(ivc_blk_res)) {
		dev_err(vblkdev->device, "ivc read failed\n");
		goto no_valid_io;
	}

	status = ivc_blk_res->status;
	if (status != 0) {
		dev_err(vblkdev->device, "Block IO request error = %d\n",
				status);
	}

	vsc_req = vblk_get_req_by_sr_num(vblkdev, ivc_blk_res->serial_number);
	if (vsc_req == NULL) {
		dev_err(vblkdev->device, "serial_number mismatch num %d!\n",
				ivc_blk_res->serial_number);
		goto no_valid_io;
	}

	bio_req = vsc_req->req;

	if ((status == 0) && (bio_req != NULL)) {
		if (rq_data_dir(bio_req) == READ) {
			rq_for_each_segment(bvec, bio_req, vsc_req->iter) {
				size = bvec.bv_len;
				buffer = page_address(bvec.bv_page) +
					bvec.bv_offset;

				if ((total_size + size) >
					(vsc_req->ivc_req.num_blks *
						vblkdev->config.hardblk_size))
					size = (vsc_req->ivc_req.num_blks *
						vblkdev->config.hardblk_size) -
						total_size;
				memcpy(buffer,
					vsc_req->mempool_virt + total_size,
					size);

				total_size += size;
				if (total_size == (vsc_req->ivc_req.num_blks *
					vblkdev->config.hardblk_size))
					break;
			}
		}

		spin_lock(vblkdev->queue->queue_lock);
		if (__blk_end_request(bio_req, 0,
			vsc_req->ivc_req.num_blks *
				vblkdev->config.hardblk_size)) {
			dev_err(vblkdev->device,
				"Error completing the request!\n");
		}
		spin_unlock(vblkdev->queue->queue_lock);

	} else if ((bio_req != NULL) && (status != 0)) {
		req_error_handler(vblkdev, bio_req);
	} else {
		dev_err(vblkdev->device,
			"VSC request %d has null bio request!\n",
			vsc_req->id);
	}

	dma_free_coherent(vblkdev->device,
		(vsc_req->ivc_req.num_blks * vblkdev->config.hardblk_size),
		NULL,
		(dma_addr_t)(vsc_req->mempool_virt - vblkdev->shared_buffer));

	vblk_put_req(vsc_req);

	if (tegra_hv_ivc_read_advance(vblkdev->ivck)) {
		dev_err(vblkdev->device,
			"Couldn't increment read frame pointer!\n");
	}

	mutex_unlock(&vblkdev->ivc_lock);

	return true;

no_valid_io:
	mutex_unlock(&vblkdev->ivc_lock);

	return false;
}

static bool bio_req_sanity_check(struct vblk_dev *vblkdev,
		struct request *bio_req)
{
	uint64_t start_offset = (blk_rq_pos(bio_req) * (uint64_t)SECTOR_SIZE);
	uint64_t req_bytes = blk_rq_bytes(bio_req);
	uint64_t max_io_bytes = (vblkdev->config.hardblk_size *
			vblkdev->config.max_blks_per_io);


	if ((start_offset >= vblkdev->size) || (req_bytes > vblkdev->size) ||
		((start_offset + req_bytes) > vblkdev->size))
	{
		dev_err(vblkdev->device,
			"Invalid I/O limit start 0x%llx size 0x%llx > 0x%llx\n",
			start_offset,
			req_bytes, vblkdev->size);
		return false;
	}

	if ((start_offset % vblkdev->config.hardblk_size) != 0) {
		dev_err(vblkdev->device, "Unaligned block offset (%lld %d)\n",
			start_offset, vblkdev->config.hardblk_size);
		return false;
	}

	if ((req_bytes % vblkdev->config.hardblk_size) != 0) {
		dev_err(vblkdev->device, "Unaligned io length (%lld %d)\n",
			req_bytes, vblkdev->config.hardblk_size);
		return false;
	}

	if (req_bytes > max_io_bytes) {
		dev_err(vblkdev->device, "Req bytes %llx greater than %llx!\n",
			req_bytes, max_io_bytes);
		return false;
	}

	return true;
}

/**
 * submit_bio_req: Fetch a bio request and submit it to
 * server for processing.
 */
static bool submit_bio_req(struct vblk_dev *vblkdev)
{
	struct vsc_request *vsc_req = NULL;
	struct request *bio_req = NULL;
	struct ivc_blk_request *ivc_blk_req;
	dma_addr_t phys;
	struct bio_vec bvec;
	size_t size;
	size_t total_size = 0;
	void *buffer;

	/* Take a lock to protect concurrent accesses to IVC queue */
	mutex_lock(&vblkdev->ivc_lock);

	if (!tegra_hv_ivc_can_write(vblkdev->ivck))
		goto bio_exit;

	if (vblkdev->queue == NULL)
		goto bio_exit;

	vsc_req = vblk_get_req(vblkdev);
	if (vsc_req == NULL)
		goto bio_exit;

	spin_lock(vblkdev->queue->queue_lock);
	bio_req = blk_fetch_request(vblkdev->queue);
	spin_unlock(vblkdev->queue->queue_lock);

	if (bio_req != NULL) {
		if (bio_req->cmd_type != REQ_TYPE_FS) {
			dev_err(vblkdev->device, "Skip non-fs request\n");
			goto bio_exit;
		}
	} else {
		goto bio_exit;
	}

	if (rq_data_dir(bio_req) == READ) {
		vsc_req->ivc_req.cmd = R_TRANSFER_SHARED_BUF;
	} else if (rq_data_dir(bio_req) == WRITE) {
		vsc_req->ivc_req.cmd = W_TRANSFER_SHARED_BUF;
	} else {
		dev_err(vblkdev->device,
			"Request direction is not read/write!\n");
		goto bio_exit;
	}

	vsc_req->iter.bio = NULL;
	if (!bio_req_sanity_check(vblkdev, bio_req)) {
		goto bio_exit;
	}

	vsc_req->req = bio_req;

	ivc_blk_req = &vsc_req->ivc_req;
	ivc_blk_req->blk_offset = ((blk_rq_pos(bio_req) *
			(uint64_t)SECTOR_SIZE) / vblkdev->config.hardblk_size);
	ivc_blk_req->num_blks = ((blk_rq_sectors(bio_req) * SECTOR_SIZE) /
					vblkdev->config.hardblk_size);

	vsc_req->mempool_virt = dma_alloc_coherent(vblkdev->device,
				blk_rq_bytes(bio_req),
				&phys,
				GFP_KERNEL);
	if (phys >= vblkdev->ivmk->size) {
		dev_err(vblkdev->device,
			"dma alloc coherent of size %d failed\n",
			blk_rq_bytes(bio_req));
		goto bio_exit;
	}

	ivc_blk_req->mempool_offset = phys;
	vsc_req->mempool_virt = (vblkdev->shared_buffer + phys);

	if (rq_data_dir(bio_req) == WRITE) {
		rq_for_each_segment(bvec, bio_req, vsc_req->iter) {
			size = bvec.bv_len;
			buffer = page_address(bvec.bv_page) +
						bvec.bv_offset;

			if ((total_size + size) > (ivc_blk_req->num_blks *
				vblkdev->config.hardblk_size))
				size = (ivc_blk_req->num_blks *
					vblkdev->config.hardblk_size) -
					total_size;

			memcpy(vsc_req->mempool_virt + total_size, buffer,
					size);
			total_size += size;
			if (total_size == (ivc_blk_req->num_blks *
				vblkdev->config.hardblk_size))
				break;
		}
	}

	if (!tegra_hv_ivc_write(vblkdev->ivck, ivc_blk_req,
				sizeof(struct ivc_blk_request))) {
		dev_err(vblkdev->device,
			"Request Id %d IVC write failed!\n",
				vsc_req->id);
		goto bio_exit;
	}

	mutex_unlock(&vblkdev->ivc_lock);

	return true;

bio_exit:
	mutex_unlock(&vblkdev->ivc_lock);

	if (vsc_req != NULL) {
		vblk_put_req(vsc_req);
	}

	if (bio_req != NULL) {
		req_error_handler(vblkdev, bio_req);
		return true;
	}

	return false;
}

static void vblk_request_work(struct work_struct *ws)
{
	struct vblk_dev *vblkdev =
		container_of(ws, struct vblk_dev, work);
	bool req_submitted, req_completed;

	if (tegra_hv_ivc_channel_notified(vblkdev->ivck) != 0)
		return;

	req_submitted = true;
	req_completed = true;
	while (req_submitted || req_completed) {
		if (vblkdev->ioctl_status != IOCTL_PROGRESS) {
			req_completed = complete_bio_req(vblkdev);
		} else
			req_completed = false;

		if (vblkdev->ioctl_status == IOCTL_IDLE) {
			req_submitted = submit_bio_req(vblkdev);
		} else
			req_submitted = false;
	}

	if (vblkdev->ioctl_status != IOCTL_IDLE &&
			vblkdev->inflight_reqs == 0) {
		ioctl_handler(vblkdev);
	}
}

/* The simple form of the request function. */
static void vblk_request(struct request_queue *q)
{
	struct vblk_dev *vblkdev = q->queuedata;

	queue_work_on(WORK_CPU_UNBOUND, vblkdev->wq, &vblkdev->work);
}

/* Open and release */
static int vblk_open(struct block_device *device, fmode_t mode)
{
	struct vblk_dev *vblkdev = device->bd_disk->private_data;

	spin_lock(&vblkdev->lock);
	if (!vblkdev->users)
		check_disk_change(device);
	vblkdev->users++;

	spin_unlock(&vblkdev->lock);
	return 0;
}

static void vblk_release(struct gendisk *disk, fmode_t mode)
{
	struct vblk_dev *vblkdev = disk->private_data;

	spin_lock(&vblkdev->lock);

	vblkdev->users--;

	spin_unlock(&vblkdev->lock);
}

int vblk_getgeo(struct block_device *device, struct hd_geometry *geo)
{
	geo->heads = VS_LOG_HEADS;
	geo->sectors = VS_LOG_SECTS;
	geo->cylinders = get_capacity(device->bd_disk) /
		(geo->heads * geo->sectors);

	return 0;
}

static int vblk_ioctl_cmd(struct block_device *bdev,
		unsigned int cmd, void __user *user)
{
	struct vblk_dev *vblkdev = bdev->bd_disk->private_data;
	u8 num_cmd;
	struct mmc_combo_cmd_info mcci = {0};
	struct mmc_ioc_cmd ic;
	struct mmc_ioc_cmd *ic_ptr = &ic;
	struct mmc_ioc_cmd __user *usr_ptr = NULL;
	struct combo_cmd_t *combo_cmd;
	uint32_t i;
	int err = 0;
	unsigned long remainingbytes;

	/*
	 * The caller must have CAP_SYS_RAWIO, and must be calling this on the
	 * whole block device, not on a partition.  This prevents overspray
	 * between sibling partitions.
	 */
	if ((!capable(CAP_SYS_RAWIO)) || (bdev != bdev->bd_contains))
		return -EPERM;

	reinit_completion(&vblkdev->ioctl_complete);

	if (vblk_prepare_passthrough_cmd(vblkdev, user, cmd)) {
		err = -EINVAL;
		goto out;
	}

	/* waiting for storage server to complete this ioctl request */
	wait_for_completion(&vblkdev->ioctl_complete);

	if (vblkdev->ioctl_status == IOCTL_FAILURE) {
		vblkdev->ioctl_status = IOCTL_IDLE;
		err = -EIO;
		goto out;
	}

	if (cmd == MMC_COMBO_IOC_CMD) {
		if (copy_from_user(&mcci, user,
			sizeof(struct mmc_combo_cmd_info))) {
			err = -EFAULT;
			goto out;
		}

		num_cmd = mcci.num_of_combo_cmds;
		if (num_cmd < 1) {
			err = -EINVAL;
			goto out;
		}

		usr_ptr = (void * __user)mcci.mmc_ioc_cmd_list;
	} else {
		usr_ptr = (void * __user)user;
		num_cmd = 1;
	}

	combo_cmd = (struct combo_cmd_t *)(vblkdev->cmd_frame +
		sizeof(struct combo_info_t));

	for (i = 0; i < num_cmd; i++) {
		if (copy_from_user((void *)ic_ptr, usr_ptr,
			sizeof(struct mmc_ioc_cmd))) {
			err = -EFAULT;
			goto out;
		}

		if (copy_to_user(&(usr_ptr->response), combo_cmd->response,
			sizeof(combo_cmd->response))) {
			err = -EFAULT;
			goto out;
		}

		if (!ic.write_flag && combo_cmd->data_len) {
			remainingbytes = copy_to_user(
				(void __user *)(unsigned long)ic.data_ptr,
				(vblkdev->cmd_frame + combo_cmd->buf_offset),
				(u64)combo_cmd->data_len);
			if (remainingbytes) {
				dev_err(vblkdev->device,
					"copy to user remainingbytes = %ld\n",
					remainingbytes);
				err = -EFAULT;
				goto out;
			}
		}
		combo_cmd++;
		usr_ptr++;
	}

out:
	return err;
}

/* The ioctl() implementation */
int vblk_ioctl(struct block_device *bdev, fmode_t mode,
	unsigned int cmd, unsigned long arg)
{
	int ret;
	struct vblk_dev *vblkdev = bdev->bd_disk->private_data;

	mutex_lock(&vblkdev->ioctl_lock);
	switch (cmd) {
	case MMC_COMBO_IOC_CMD:
	case MMC_IOC_CMD:
		ret = vblk_ioctl_cmd(bdev,
			cmd, (void __user *)arg);
		break;

	default:  /* unknown command */
		ret = -ENOTTY;
		break;
	}
	mutex_unlock(&vblkdev->ioctl_lock);

	return ret;
}

/* The device operations structure. */
const struct block_device_operations vblk_ops = {
	.owner           = THIS_MODULE,
	.open            = vblk_open,
	.release         = vblk_release,
	.getgeo          = vblk_getgeo,
	.ioctl           = vblk_ioctl
};

/* Set up virtual device. */
static void setup_device(struct vblk_dev *vblkdev)
{
	uint32_t max_requests;
	uint32_t max_io_bytes;

	vblkdev->size =
		vblkdev->config.num_blks * vblkdev->config.hardblk_size;

	spin_lock_init(&vblkdev->lock);
	spin_lock_init(&vblkdev->queue_lock);
	mutex_init(&vblkdev->ioctl_lock);

	vblkdev->queue = blk_init_queue(vblk_request, &vblkdev->queue_lock);
	if (vblkdev->queue == NULL) {
		dev_err(vblkdev->device, "failed to init blk queue\n");
		return;
	}

	vblkdev->queue->queuedata = vblkdev;

	if (elevator_change(vblkdev->queue, "noop")) {
		dev_err(vblkdev->device, "(elevator_init) fail\n");
		return;
	}

	blk_queue_logical_block_size(vblkdev->queue,
		vblkdev->config.hardblk_size);
	blk_queue_physical_block_size(vblkdev->queue,
		vblkdev->config.hardblk_size);

	/* Set the maximum number of requests possible using
	 * server returned information */
	max_io_bytes = (vblkdev->config.hardblk_size *
				vblkdev->config.max_blks_per_io);
	max_requests = ((vblkdev->ivmk->size) / max_io_bytes);
	vblkdev->max_requests = max_requests;

	if (max_requests < MAX_VSC_REQS) {
		dev_warn(vblkdev->device,
			"Setting Max requests to %d, consider "
			"increasing mempool size !\n",
			vblkdev->max_requests);
	} else if (max_requests > MAX_VSC_REQS) {
		vblkdev->max_requests = MAX_VSC_REQS;
		dev_warn(vblkdev->device,
			"Reducing the max requests to %d, consider"
			" supporting more requests for the vblkdev!\n",
			MAX_VSC_REQS);
	}

	if (vblkdev->ivck->nframes < vblkdev->max_requests) {
		dev_warn(vblkdev->device,
			"IVC frames %d less than possible max requests %d!\n",
			vblkdev->ivck->nframes, vblkdev->max_requests);
	}

	mutex_init(&vblkdev->req_lock);
	mutex_init(&vblkdev->ivc_lock);

	blk_queue_max_hw_sectors(vblkdev->queue, max_io_bytes / SECTOR_SIZE);
	queue_flag_set_unlocked(QUEUE_FLAG_NONROT, vblkdev->queue);

	/* And the gendisk structure. */
	vblkdev->gd = alloc_disk(VBLK_MINORS);
	if (!vblkdev->gd) {
		dev_err(vblkdev->device, "alloc_disk failure\n");
		return;
	}
	vblkdev->gd->major = vblk_major;
	vblkdev->gd->first_minor = vblkdev->devnum * VBLK_MINORS;
	vblkdev->gd->fops = &vblk_ops;
	vblkdev->gd->queue = vblkdev->queue;
	vblkdev->gd->private_data = vblkdev;
	snprintf(vblkdev->gd->disk_name, 32, "vblkdev%d", vblkdev->devnum);
	set_capacity(vblkdev->gd, (vblkdev->size / SECTOR_SIZE));
	add_disk(vblkdev->gd);
}

static void vblk_init_device(struct work_struct *ws)
{
	struct vblk_dev *vblkdev = container_of(ws, struct vblk_dev, init);

	/* wait for ivc channel reset to finish */
	if (tegra_hv_ivc_channel_notified(vblkdev->ivck) != 0)
		return;	/* this will be rescheduled by irq handler */

	if (tegra_hv_ivc_can_read(vblkdev->ivck) && !vblkdev->initialized) {
		if (vblk_get_configinfo(vblkdev))
			return;

		vblkdev->initialized = true;
		setup_device(vblkdev);
	}
}

static irqreturn_t ivc_irq_handler(int irq, void *data)
{
	struct vblk_dev *vblkdev = (struct vblk_dev *)data;

	if (vblkdev->initialized)
		queue_work_on(WORK_CPU_UNBOUND, vblkdev->wq, &vblkdev->work);
	else
		schedule_work(&vblkdev->init);

	return IRQ_HANDLED;
}

static int tegra_hv_vblk_probe(struct platform_device *pdev)
{
	static struct device_node *vblk_node;
	struct vblk_dev *vblkdev;
	struct device *dev = &pdev->dev;
	int ret;
	struct tegra_hv_ivm_cookie *ivmk;

	if (!is_tegra_hypervisor_mode()) {
		dev_err(dev, "Hypervisor is not present\n");
		return -ENODEV;
	}

	if (vblk_major == 0) {
		dev_err(dev, "major number is invalid\n");
		return -ENODEV;
	}

	vblk_node = dev->of_node;
	if (vblk_node == NULL) {
		dev_err(dev, "No of_node data\n");
		return -ENODEV;
	}

	dev_info(dev, "allocate drvdata buffer\n");
	vblkdev = kzalloc(sizeof(struct vblk_dev), GFP_KERNEL);
	if (vblkdev == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, vblkdev);
	vblkdev->device = dev;

	/* Get properties of instance and ivc channel id */
	if (of_property_read_u32(vblk_node, "instance", &(vblkdev->devnum))) {
		dev_err(dev, "Failed to read instance property\n");
		ret = -ENODEV;
		goto free_drvdata;
	} else {
		if (of_property_read_u32_index(vblk_node, "ivc", 1,
			&(vblkdev->ivc_id))) {
			dev_err(dev, "Failed to read ivc property\n");
			ret = -ENODEV;
			goto free_drvdata;
		}
		if (of_property_read_u32_index(vblk_node, "mempool", 0,
			&(vblkdev->ivm_id))) {
			dev_err(dev, "Failed to read mempool property\n");
			ret = -ENODEV;
			goto free_drvdata;
		}
	}

	vblkdev->ivck = tegra_hv_ivc_reserve(NULL, vblkdev->ivc_id, NULL);
	if (IS_ERR_OR_NULL(vblkdev->ivck)) {
		dev_err(dev, "Failed to reserve IVC channel %d\n",
			vblkdev->ivc_id);
		vblkdev->ivck = NULL;
		ret = -ENODEV;
		goto free_drvdata;
	}

	ivmk = tegra_hv_mempool_reserve(NULL, vblkdev->ivm_id);
	if (IS_ERR_OR_NULL(ivmk)) {
		dev_err(dev, "Failed to reserve IVM channel %d\n",
			vblkdev->ivm_id);
		ivmk = NULL;
		ret = -ENODEV;
		goto free_drvdata;
	}

	vblkdev->shared_buffer = ioremap_cache(ivmk->ipa, ivmk->size);
	if (IS_ERR_OR_NULL(vblkdev->shared_buffer)) {
		dev_err(dev, "Failed to map mempool area %d\n",
				vblkdev->ivm_id);
		ret = -ENOMEM;
		goto free_mempool;
	}

	ret = dma_declare_coherent_memory(vblkdev->device, ivmk->ipa,
				0, ivmk->size,
			DMA_MEMORY_NOMAP | DMA_MEMORY_EXCLUSIVE);
	if (!(ret & DMA_MEMORY_NOMAP)) {
		dev_err(dev, "dma_declare_coherent_memory failed\n");
		goto unmap_mempool;
	}

	vblkdev->ivmk = ivmk;

	vblkdev->cmd_frame = kzalloc(vblkdev->ivck->frame_size, GFP_KERNEL);
	if (vblkdev->cmd_frame == NULL) {
		ret = -ENOMEM;
		goto unmap_mempool;
	}

	vblkdev->ioctl_status = IOCTL_IDLE;
	init_completion(&vblkdev->ioctl_complete);
	vblkdev->initialized = false;

	vblkdev->wq = alloc_workqueue("vblk_req_wq%d",
		WQ_UNBOUND | WQ_MEM_RECLAIM,
		1, vblkdev->devnum);
	if (vblkdev->wq == NULL) {
		dev_err(dev, "Failed to allocate workqueue\n");
		ret = -ENOMEM;
		goto free_cmd_frame;
	}
	INIT_WORK(&vblkdev->init, vblk_init_device);
	INIT_WORK(&vblkdev->work, vblk_request_work);

	if (request_irq(vblkdev->ivck->irq, ivc_irq_handler, 0,
		"vblk", vblkdev)) {
		dev_err(dev, "Failed to request irq %d\n", vblkdev->ivck->irq);
		ret = -EINVAL;
		goto free_wq;
	}

	tegra_hv_ivc_channel_reset(vblkdev->ivck);
	if (vblk_send_config_cmd(vblkdev)) {
		dev_err(dev, "Failed to send config cmd\n");
		ret = -EACCES;
		goto free_irq;
	}

	return 0;

free_irq:
	free_irq(vblkdev->ivck->irq, vblkdev);

free_wq:
	destroy_workqueue(vblkdev->wq);

free_cmd_frame:
	kfree(vblkdev->cmd_frame);

unmap_mempool:
	iounmap(vblkdev->shared_buffer);

free_mempool:
	kfree(ivmk);

free_drvdata:
	platform_set_drvdata(pdev, NULL);
	kfree(vblkdev);

	return ret;
}

static int tegra_hv_vblk_remove(struct platform_device *pdev)
{
	struct vblk_dev *vblkdev = platform_get_drvdata(pdev);

	if (vblkdev->gd) {
		del_gendisk(vblkdev->gd);
		put_disk(vblkdev->gd);
	}

	if (vblkdev->queue)
		blk_cleanup_queue(vblkdev->queue);

	destroy_workqueue(vblkdev->wq);
	free_irq(vblkdev->ivck->irq, vblkdev);
	tegra_hv_ivc_unreserve(vblkdev->ivck);
	platform_set_drvdata(pdev, NULL);
	kfree(vblkdev->cmd_frame);
	kfree(vblkdev);

	return 0;
}

static int __init vblk_init(void)
{
	vblk_major = 0;
	vblk_major = register_blkdev(vblk_major, "vblk");
	if (vblk_major <= 0) {
		pr_err("vblk: unable to get major number\n");
		return -ENODEV;
	}

	return 0;
}

static void vblk_exit(void)
{
	unregister_blkdev(vblk_major, "vblk");
}

#ifdef CONFIG_OF
static struct of_device_id tegra_hv_vblk_match[] = {
	{ .compatible = "nvidia,tegra-hv-storage", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_hv_vblk_match);
#endif /* CONFIG_OF */

static struct platform_driver tegra_hv_vblk_driver = {
	.probe	= tegra_hv_vblk_probe,
	.remove	= tegra_hv_vblk_remove,
	.driver	= {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_hv_vblk_match),
	},
};

module_platform_driver(tegra_hv_vblk_driver);

module_init(vblk_init);
module_exit(vblk_exit);

MODULE_AUTHOR("Dilan Lee <dilee@nvidia.com>");
MODULE_DESCRIPTION("Virtual storage device over Tegra Hypervisor IVC channel");
MODULE_LICENSE("GPL");

