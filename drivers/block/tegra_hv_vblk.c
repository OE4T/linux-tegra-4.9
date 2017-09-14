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
#include <linux/pm.h>
#include <linux/slab.h>   /* kmalloc() */
#include <linux/fs.h>   /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/fcntl.h> /* O_ACCMODE */
#include <linux/hdreg.h> /* HDIO_GETGEO */
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <soc/tegra/chip-id.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm-generic/bug.h>
#include <scsi/scsi.h>
#include <scsi/sg.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include "tegra_vblk.h"

static int vblk_major;

static int vblk_complete_ioctl_req(struct vblk_dev *vblkdev,
		struct vsc_request *vsc_req);

static int vblk_prep_ioctl_req(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		struct vsc_request *vsc_req);

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
		req->ivc_req.serial_number = bit;
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
		memset(&req->ivc_req, 0, sizeof(struct ivc_request));
		req->req = NULL;
		memset(&req->iter, 0, sizeof(struct req_iterator));
		vblkdev->inflight_reqs--;
	}
exit:
	mutex_unlock(&vblkdev->req_lock);
}

static int vblk_send_config_cmd(struct vblk_dev *vblkdev)
{
	struct ivc_request *ivc_req;
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
	ivc_req = (struct ivc_request *)
		tegra_hv_ivc_write_get_next_frame(vblkdev->ivck);
	if (IS_ERR_OR_NULL(ivc_req)) {
		dev_err(vblkdev->device, "no empty frame for write\n");
		return -EIO;
	}

	ivc_req->cmd = VBLK_GET_CONFIG;

	dev_info(vblkdev->device, "send config cmd to ivc #%d\n",
		vblkdev->ivc_id);

	if (tegra_hv_ivc_write_advance(vblkdev->ivck)) {
		dev_err(vblkdev->device, "ivc write failed\n");
		return -EIO;
	}

	return 0;
}

static int vblk_prepare_passthrough_cmd(struct vblk_dev *vblkdev,
	void __user *user, unsigned int combo)
{
	int err = 0;
	struct combo_info_t *combo_info;
	struct combo_cmd_t *combo_cmd;
	int i = 0;
	uint64_t num_cmd;
	struct mmc_ioc_cmd ic;
	struct mmc_ioc_multi_cmd __user *user_cmd;
	struct mmc_ioc_cmd __user *usr_ptr;
	uint32_t combo_cmd_size;
	unsigned long remainingbytes;
	uint8_t *tmpaddr;

	combo_info = (struct combo_info_t *)vblkdev->cmd_frame;

	if (combo == MMC_IOC_MULTI_CMD) {
		user_cmd = (struct mmc_ioc_multi_cmd __user *)user;
		if (copy_from_user(&num_cmd, &user_cmd->num_of_cmds,
				sizeof(num_cmd))) {
			err = -EFAULT;
			goto out;
		}

		if (num_cmd > MMC_IOC_MAX_CMDS)
			return -EINVAL;

		usr_ptr = (void * __user)&user_cmd->cmds;
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

	if (combo_cmd_size > vblkdev->ivck->frame_size) {
		dev_err(vblkdev->device,
			" ivc frame has no enough space to serve ioctl\n");
		err = -EFAULT;
		goto out;
	}

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

	if (vblkdev->config.cmd != VBLK_GET_CONFIG)
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
		"Error for request pos %llx type %llx size %x\n",
		(blk_rq_pos(breq) * (uint64_t)SECTOR_SIZE),
		req_op(breq),
		blk_rq_bytes(breq));

	blk_end_request_all(breq, -EIO);
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
	struct ivc_result *ivc_res;
	int status = 0;
	struct bio_vec bvec;
	size_t size;
	size_t total_size = 0;
	struct vsc_request *vsc_req = NULL;
	struct ivc_request *ivc_req;
	struct request *bio_req;
	void *buffer;

	if (!tegra_hv_ivc_can_read(vblkdev->ivck))
		goto no_valid_io;

	ivc_res = (struct ivc_result *)
		tegra_hv_ivc_read_get_next_frame(vblkdev->ivck);
	if (IS_ERR_OR_NULL(ivc_res)) {
		dev_err(vblkdev->device, "ivc read failed\n");
		goto no_valid_io;
	}

	status = ivc_res->status;
	if (status != 0) {
		dev_err(vblkdev->device, "IO request error = %d\n",
				status);
	}

	vsc_req = vblk_get_req_by_sr_num(vblkdev, ivc_res->serial_number);
	if (vsc_req == NULL) {
		dev_err(vblkdev->device, "serial_number mismatch num %d!\n",
				ivc_res->serial_number);
		goto no_valid_io;
	}

	bio_req = vsc_req->req;
	ivc_req = &vsc_req->ivc_req;

	if ((bio_req != NULL) && (status == 0)) {
		if (bio_req->cmd_type == REQ_TYPE_DRV_PRIV) {
			if (vblk_complete_ioctl_req(vblkdev, vsc_req)) {
				req_error_handler(vblkdev, bio_req);
			} else {
				if (blk_end_request(bio_req, 0, 0)) {
					dev_err(vblkdev->device,
						"Error completing private request!\n");
				}
			}
		} else {
			if (req_op(bio_req) == REQ_OP_READ) {
				rq_for_each_segment(bvec, bio_req,
					vsc_req->iter) {
					size = bvec.bv_len;
					buffer = page_address(bvec.bv_page) +
						bvec.bv_offset;

					if ((total_size + size) >
						(ivc_req->num_blks *
						vblkdev->config.hardblk_size))
					{
						size =
						(ivc_req->num_blks *
						vblkdev->config.hardblk_size) -
							total_size;
					}
					memcpy(buffer,
						vsc_req->mempool_virt +
						total_size,
						size);

					total_size += size;
					if (total_size ==
						(ivc_req->num_blks *
						vblkdev->config.hardblk_size))
						break;
				}
			}

			if (blk_end_request(bio_req, 0,
				ivc_req->num_blks *
					vblkdev->config.hardblk_size)) {
				dev_err(vblkdev->device,
					"Error completing fs request!\n");
			}
		}
	} else if ((bio_req != NULL) && (status != 0)) {
		req_error_handler(vblkdev, bio_req);
	} else {
		dev_err(vblkdev->device,
			"VSC request %d has null bio request!\n",
			vsc_req->id);
	}

	vblk_put_req(vsc_req);

	if (tegra_hv_ivc_read_advance(vblkdev->ivck)) {
		dev_err(vblkdev->device,
			"Couldn't increment read frame pointer!\n");
	}

	return true;

no_valid_io:
	return false;
}

static bool bio_req_sanity_check(struct vblk_dev *vblkdev,
		struct request *bio_req,
		struct vsc_request *vsc_req)
{
	uint64_t start_offset = (blk_rq_pos(bio_req) * (uint64_t)SECTOR_SIZE);
	uint64_t req_bytes = blk_rq_bytes(bio_req);


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

	if (req_bytes > (uint64_t)vsc_req->mempool_len) {
		dev_err(vblkdev->device, "Req bytes %llx greater than %x!\n",
			req_bytes, vsc_req->mempool_len);
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
	struct ivc_request *ivc_req;
	struct bio_vec bvec;
	size_t size;
	size_t total_size = 0;
	void *buffer;

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

	if (bio_req == NULL)
		goto bio_exit;

	if ((bio_req->cmd_type != REQ_TYPE_FS) &&
		      (bio_req->cmd_type != REQ_TYPE_DRV_PRIV))	{
		dev_err(vblkdev->device, "unsupported cmd type %d!\n",
				bio_req->cmd_type);
		goto bio_exit;
	}

	vsc_req->req = bio_req;
	ivc_req = &vsc_req->ivc_req;

	if (bio_req->cmd_type == REQ_TYPE_FS) {
		if (req_op(bio_req) == REQ_OP_READ) {
			ivc_req->cmd = VBLK_READ;
		} else if (req_op(bio_req) == REQ_OP_WRITE) {
			ivc_req->cmd = VBLK_WRITE;
		} else if (req_op(bio_req) == REQ_OP_FLUSH) {
			ivc_req->cmd = VBLK_FLUSH;
		} else {
			dev_err(vblkdev->device,
				"Request direction is not read/write!\n");
			goto bio_exit;
		}

		vsc_req->iter.bio = NULL;
		if (req_op(bio_req) == REQ_OP_FLUSH) {
			ivc_req->blk_offset = 0;
			ivc_req->num_blks = vblkdev->config.num_blks;
		} else {
			if (!bio_req_sanity_check(vblkdev, bio_req, vsc_req)) {
				goto bio_exit;
			}

			ivc_req->blk_offset = ((blk_rq_pos(bio_req) *
				(uint64_t)SECTOR_SIZE)
				/ vblkdev->config.hardblk_size);
			ivc_req->num_blks = ((blk_rq_sectors(bio_req) *
				SECTOR_SIZE) /
				vblkdev->config.hardblk_size);

			ivc_req->data_offset = vsc_req->mempool_offset;
		}

		if (req_op(bio_req) == REQ_OP_WRITE) {
			rq_for_each_segment(bvec, bio_req, vsc_req->iter) {
				size = bvec.bv_len;
				buffer = page_address(bvec.bv_page) +
						bvec.bv_offset;

				if ((total_size + size) >
					(ivc_req->num_blks *
					vblkdev->config.hardblk_size))
				{
					size = (ivc_req->num_blks *
						vblkdev->config.hardblk_size) -
						total_size;
				}

				memcpy(vsc_req->mempool_virt + total_size,
					buffer, size);
				total_size += size;
				if (total_size == (ivc_req->num_blks *
					vblkdev->config.hardblk_size)) {
					break;
				}
			}
		}
	} else if (bio_req->cmd_type == REQ_TYPE_DRV_PRIV) {
		if (vblk_prep_ioctl_req(vblkdev,
			(struct vblk_ioctl_req *)bio_req->special,
			vsc_req)) {
			dev_err(vblkdev->device,
				"Failed to prepare ioctl request!\n");
			goto bio_exit;
		}
	}

	if (!tegra_hv_ivc_write(vblkdev->ivck, ivc_req,
				sizeof(struct ivc_request))) {
		dev_err(vblkdev->device,
			"Request Id %d IVC write failed!\n",
				vsc_req->id);
		goto bio_exit;
	}

	return true;

bio_exit:
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
	uint64_t num_cmd;
	struct mmc_ioc_cmd ic;
	struct mmc_ioc_cmd *ic_ptr = &ic;
	struct mmc_ioc_multi_cmd __user *user_cmd;
	struct mmc_ioc_cmd __user *usr_ptr;
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

	if (cmd == MMC_IOC_MULTI_CMD) {
		user_cmd = (struct mmc_ioc_multi_cmd __user *)user;
		if (copy_from_user(&num_cmd, &user_cmd->num_of_cmds,
				sizeof(num_cmd))) {
			err = -EFAULT;
			goto out;
		}

		if (num_cmd > MMC_IOC_MAX_CMDS)
			return -EINVAL;

		usr_ptr = (void * __user)&user_cmd->cmds;
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

static int vblk_prep_sg_io(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		void __user *user)
{
	int err = 0;
	sg_io_hdr_t *hp = NULL;
	uint32_t header_len = sizeof(sg_io_hdr_t);
	struct vblk_sg_io_hdr *vblk_hp;
	uint32_t vblk_sg_header_len = sizeof(struct vblk_sg_io_hdr);
	uint32_t ioctl_offset;
	uint32_t vblk_hp_offset;
	uint32_t cmnd_offset;
	void *cmnd;
	uint32_t sbp_offset;
	void *sbp;
	uint32_t data_buf_offset;
	void *data_buf;
	uint32_t data_buf_size;
	uint32_t ioctl_len;
	uint32_t *ioctl_id;
	void *ioctl_buf = NULL;

	hp = kmalloc(header_len, GFP_KERNEL);
	if (hp == NULL) {
		return -ENOMEM;
	}

	if (copy_from_user(hp, user, header_len)) {
		err = -EFAULT;
		goto free_hp;
	}

	if ((!hp->cmdp) || (hp->cmd_len < 6) ||
		(hp->cmd_len > VBLK_SG_MAX_CMD_LEN)) {
		err = -EMSGSIZE;
		goto free_hp;
	}

	ioctl_offset = 0;
	vblk_hp_offset = ioctl_offset + sizeof(*ioctl_id);
	if (vblk_hp_offset < ioctl_offset) {
		err = -ENOMEM;
		goto free_hp;
	}

	cmnd_offset = (vblk_hp_offset + vblk_sg_header_len);
	if (cmnd_offset < vblk_hp_offset) {
		err = -ENOMEM;
		goto free_hp;
	}

	sbp_offset = (cmnd_offset + hp->cmd_len);
	if (sbp_offset < cmnd_offset) {
		err = - EMSGSIZE;
		goto free_hp;
	}

	data_buf_offset = (sbp_offset + hp->mx_sb_len);
	if (data_buf_offset < sbp_offset) {
		err = -EMSGSIZE;
		goto free_hp;
	}
	data_buf_offset = ALIGN(data_buf_offset,
			vblkdev->config.hardblk_size);
	data_buf_size = ALIGN(hp->dxfer_len,
			vblkdev->config.hardblk_size);

	ioctl_len = data_buf_offset + data_buf_size;
	if (ioctl_len < data_buf_offset) {
		err = -EMSGSIZE;
		goto free_hp;
	}

	ioctl_buf = kmalloc(ioctl_len, GFP_KERNEL);
	if (ioctl_buf == NULL) {
		err = -ENOMEM;
		goto free_hp;
	}

	ioctl_id = (uint32_t *)ioctl_buf;
	*ioctl_id = VBLK_SG_IO_ID;

	vblk_hp = (struct vblk_sg_io_hdr *)(ioctl_buf + vblk_hp_offset);
	sbp = (ioctl_buf + sbp_offset);
	cmnd = (ioctl_buf + cmnd_offset);
	if (copy_from_user(cmnd, hp->cmdp, hp->cmd_len)) {
		err = -EFAULT;
		goto free_ioctl_buf;
	}

	data_buf = (ioctl_buf + data_buf_offset);

	switch (hp->dxfer_direction) {
	case SG_DXFER_NONE:
		vblk_hp->data_direction = SCSI_DATA_NONE;
		break;
	case SG_DXFER_TO_DEV:
		vblk_hp->data_direction = SCSI_TO_DEVICE;
		break;
	case SG_DXFER_FROM_DEV:
		vblk_hp->data_direction = SCSI_FROM_DEVICE;
		break;
	case SG_DXFER_TO_FROM_DEV:
		vblk_hp->data_direction = SCSI_BIDIRECTIONAL;
		break;
	default:
		err = -EBADMSG;
		goto free_ioctl_buf;
	}

	if ((vblk_hp->data_direction == SCSI_TO_DEVICE) ||
		(vblk_hp->data_direction == SCSI_BIDIRECTIONAL)) {
		if (copy_from_user(data_buf, hp->dxferp, hp->dxfer_len)) {
			err = -EFAULT;
			goto free_ioctl_buf;
		}
	}

	vblk_hp->cmd_len = hp->cmd_len;
	vblk_hp->mx_sb_len = hp->mx_sb_len;
	vblk_hp->dxfer_len = hp->dxfer_len;
	vblk_hp->xfer_arg_offset = data_buf_offset;
	vblk_hp->cmdp_arg_offset = cmnd_offset;
	vblk_hp->sbp_arg_offset = sbp_offset;

	ioctl_req->ioctl_buf = ioctl_buf;
	ioctl_req->ioctl_len = ioctl_len;

free_ioctl_buf:
	if (err && ioctl_buf)
		kfree (ioctl_buf);

free_hp:
	if (hp)
		kfree(hp);

	return err;
}

static int vblk_complete_sg_io(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		void __user *user)
{
	sg_io_hdr_t *hp = NULL;
	uint32_t header_len = sizeof(sg_io_hdr_t);
	struct vblk_sg_io_hdr *vblk_hp;
	uint32_t vblk_hp_offset;
	void *sbp;
	void *data_buf;
	int err = 0;

	hp = kmalloc(header_len, GFP_KERNEL);
	if (hp == NULL) {
		return -ENOMEM;
	}

	if (copy_from_user(hp, user, header_len)) {
		err = -EFAULT;
		goto free_hp;
	}

	vblk_hp_offset = sizeof(uint32_t);
	vblk_hp = (struct vblk_sg_io_hdr *)(ioctl_req->ioctl_buf + vblk_hp_offset);
	hp->status = 0xff & vblk_hp->status;
	hp->masked_status = status_byte(vblk_hp->status);
	hp->msg_status = msg_byte(vblk_hp->status);
	hp->host_status = host_byte(vblk_hp->status);
	hp->driver_status = driver_byte(vblk_hp->status);
	hp->sb_len_wr = vblk_hp->sb_len_wr;
	/* TODO: Handle the residual length */
	hp->resid = 0;

	sbp = (ioctl_req->ioctl_buf + vblk_hp->sbp_arg_offset);
	if ((hp->sb_len_wr != 0) && (hp->sbp != NULL)) {
		if (copy_to_user(hp->sbp, sbp, hp->sb_len_wr)) {
			err = -EFAULT;
			goto free_hp;
		}
	}

	data_buf = (ioctl_req->ioctl_buf + vblk_hp->xfer_arg_offset);

	if ((vblk_hp->data_direction == SCSI_FROM_DEVICE) ||
		(vblk_hp->data_direction == SCSI_BIDIRECTIONAL)) {
		if (copy_to_user(hp->dxferp, data_buf, vblk_hp->dxfer_len)) {
			err = -EFAULT;
			goto free_hp;
		}
	}

	if (copy_to_user(user, hp, header_len)) {
		err = -EFAULT;
		goto free_hp;
	}

free_hp:
	if (ioctl_req->ioctl_buf)
		kfree(ioctl_req->ioctl_buf);

	if (hp)
		kfree(hp);

	return err;
}

static int vblk_complete_ioctl_req(struct vblk_dev *vblkdev,
		struct vsc_request *vsc_req)
{
	struct vblk_ioctl_req *ioctl_req = vsc_req->ioctl_req;
	int32_t ret = 0;

	if (ioctl_req == NULL) {
		dev_err(vblkdev->device,
			"Invalid ioctl request for completion!\n");
		ret = -EINVAL;
		goto comp_exit;
	}

	memcpy(ioctl_req->ioctl_buf, vsc_req->mempool_virt,
			ioctl_req->ioctl_len);
comp_exit:
	return ret;
}

static int vblk_prep_ioctl_req(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		struct vsc_request *vsc_req)
{
	int32_t ret = 0;
	struct ivc_request *ivc_req;

	if (ioctl_req == NULL) {
		dev_err(vblkdev->device,
			"Invalid ioctl request for preparation!\n");
		return -EINVAL;
	}


	if (ioctl_req->ioctl_len > vsc_req->mempool_len) {
		dev_err(vblkdev->device,
			"Ioctl length exceeding mempool length!\n");
		return -EINVAL;
	}

	if (ioctl_req->ioctl_buf == NULL) {
		dev_err(vblkdev->device,
			"Ioctl buffer invalid!\n");
		return -EINVAL;
	}

	ivc_req = &vsc_req->ivc_req;
	ivc_req->cmd = VBLK_IOCTL;
	memcpy(vsc_req->mempool_virt, ioctl_req->ioctl_buf,
			ioctl_req->ioctl_len);
	ivc_req->data_offset = vsc_req->mempool_offset;
	ivc_req->ioctl_len = ioctl_req->ioctl_len;

	vsc_req->ioctl_req = ioctl_req;

	return ret;
}

static int vblk_submit_ioctl_req(struct block_device *bdev,
		unsigned int cmd, void __user *user)
{
	struct vblk_dev *vblkdev = bdev->bd_disk->private_data;
	struct vblk_ioctl_req *ioctl_req = NULL;
	struct request *rq;
	int err;

	/*
	 * The caller must have CAP_SYS_RAWIO, and must be calling this on the
	 * whole block device, not on a partition.  This prevents overspray
	 * between sibling partitions.
	 */
	if ((!capable(CAP_SYS_RAWIO)) || (bdev != bdev->bd_contains))
		return -EPERM;

	ioctl_req = kmalloc(sizeof(struct vblk_ioctl_req), GFP_KERNEL);
	if (!ioctl_req) {
		dev_err(vblkdev->device,
			"failed to alloc memory for ioctl req!\n");
		return -ENOMEM;
	}

	switch (cmd) {
	case SG_IO:
		err = vblk_prep_sg_io(vblkdev, ioctl_req,
			user);
		break;
	default:
		dev_err(vblkdev->device, "unsupported command %x!\n", cmd);
		err = -EINVAL;
		goto free_ioctl_req;
	}

	if (err)
		goto free_ioctl_req;

	rq = blk_get_request(vblkdev->queue, READ, GFP_KERNEL);
	if (IS_ERR_OR_NULL(rq)) {
		dev_err(vblkdev->device,
			"Failed to get handle to a request!\n");
		err = PTR_ERR(rq);
		goto free_ioctl_req;
	}

	rq->cmd_type = REQ_TYPE_DRV_PRIV;
	rq->special = (void *)ioctl_req;

	err = blk_execute_rq(vblkdev->queue, vblkdev->gd, rq, 0);

	blk_put_request(rq);

	if (err)
		goto free_ioctl_req;

	switch (cmd) {
	case SG_IO:
		err = vblk_complete_sg_io(vblkdev, ioctl_req,
			user);
		break;
	default:
		dev_err(vblkdev->device, "unsupported command %x!\n", cmd);
		err = -EINVAL;
		goto free_ioctl_req;
	}

free_ioctl_req:
	if (ioctl_req)
		kfree(ioctl_req);

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
	case MMC_IOC_MULTI_CMD:
	case MMC_IOC_CMD:
		ret = vblk_ioctl_cmd(bdev,
			cmd, (void __user *)arg);
		break;

	case SG_IO:
		ret = vblk_submit_ioctl_req(bdev, cmd,
			(void __user *)arg);
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
	uint32_t max_io_bytes;
	uint32_t req_id;
	uint32_t max_requests;
	struct vsc_request *req;

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

	if ((vblkdev->config.req_types_supported & VBLK_BLK_REQ_F) &&
		(vblkdev->config.blk_req_ops_supported & VBLK_FLUSH_OP_F)) {
		blk_queue_write_cache(vblkdev->queue, true, false);
	}

	/* Set the maximum number of requests possible using
	 * server returned information */
	max_io_bytes = (vblkdev->config.hardblk_size *
				vblkdev->config.max_blks_per_io);
	if (max_io_bytes == 0) {
		dev_err(vblkdev->device, "Maximum io bytes value is 0!\n");
		return;
	}

	max_requests = ((vblkdev->ivmk->size) / max_io_bytes);

	if (max_requests < MAX_VSC_REQS) {
		dev_warn(vblkdev->device,
			"Setting Max requests to %d, consider "
			"increasing mempool size !\n",
			max_requests);
	} else if (max_requests > MAX_VSC_REQS) {
		max_requests = MAX_VSC_REQS;
		dev_warn(vblkdev->device,
			"Reducing the max requests to %d, consider"
			" supporting more requests for the vblkdev!\n",
			MAX_VSC_REQS);
	}

	if (vblkdev->ivck->nframes < max_requests) {
		dev_warn(vblkdev->device,
			"IVC frames %d less than possible max requests %d!\n",
			vblkdev->ivck->nframes, max_requests);
	}

	for (req_id = 0; req_id < max_requests; req_id++){
		req = &vblkdev->reqs[req_id];
		req->mempool_virt = (void *)((uintptr_t)vblkdev->shared_buffer +
			(uintptr_t)(req_id * max_io_bytes));
		req->mempool_offset = (req_id * max_io_bytes);
		req->mempool_len = max_io_bytes;
		req->id = req_id;
		req->vblkdev = vblkdev;
	}

	if (max_requests == 0) {
		dev_err(vblkdev->device,
			"maximum requests set to 0!\n");
		return;
	}
	mutex_init(&vblkdev->req_lock);

	vblkdev->max_requests = max_requests;
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

	/* Don't allow scanning of the device when block
	 * requests are not supported */
	if (!(vblkdev->config.req_types_supported & VBLK_BLK_REQ_F)) {
		vblkdev->gd->flags |= GENHD_FL_NO_PART_SCAN;
	}

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
	vblkdev->ivmk = ivmk;

	vblkdev->shared_buffer = ioremap_cache(ivmk->ipa, ivmk->size);
	if (IS_ERR_OR_NULL(vblkdev->shared_buffer)) {
		dev_err(dev, "Failed to map mempool area %d\n",
				vblkdev->ivm_id);
		ret = -ENOMEM;
		goto free_mempool;
	}

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
	tegra_hv_mempool_unreserve(ivmk);

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

#ifdef CONFIG_PM_SLEEP
static int tegra_hv_vblk_suspend(struct device *dev)
{
	struct vblk_dev *vblkdev = dev_get_drvdata(dev);
	unsigned long flags;

	if (vblkdev->queue) {
		spin_lock_irqsave(vblkdev->queue->queue_lock, flags);
		blk_stop_queue(vblkdev->queue);
		spin_unlock_irqrestore(vblkdev->queue->queue_lock, flags);

		disable_irq(vblkdev->ivck->irq);

		flush_workqueue(vblkdev->wq);
	}

	return 0;
}

static int tegra_hv_vblk_resume(struct device *dev)
{
	struct vblk_dev *vblkdev = dev_get_drvdata(dev);
	unsigned long flags;

	if (vblkdev->queue) {
		enable_irq(vblkdev->ivck->irq);

		spin_lock_irqsave(vblkdev->queue->queue_lock, flags);
		blk_start_queue(vblkdev->queue);
		spin_unlock_irqrestore(vblkdev->queue->queue_lock, flags);

		queue_work_on(WORK_CPU_UNBOUND, vblkdev->wq, &vblkdev->work);
	}

	return 0;
}

static const struct dev_pm_ops tegra_hv_vblk_pm_ops = {
	.suspend = tegra_hv_vblk_suspend,
	.resume = tegra_hv_vblk_resume,
};
#endif /* CONFIG_PM_SLEEP */

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
#ifdef CONFIG_PM_SLEEP
		.pm = &tegra_hv_vblk_pm_ops,
#endif
	},
};

module_platform_driver(tegra_hv_vblk_driver);

module_init(vblk_init);
module_exit(vblk_exit);

MODULE_AUTHOR("Dilan Lee <dilee@nvidia.com>");
MODULE_DESCRIPTION("Virtual storage device over Tegra Hypervisor IVC channel");
MODULE_LICENSE("GPL");

