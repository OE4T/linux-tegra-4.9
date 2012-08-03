/*
 * drivers/video/tegra/host/gk20a/ctrl_gk20a.c
 *
 * GK20A Ctrl
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/highmem.h>
#include <linux/cdev.h>
#include <linux/nvhost_gpu_ioctl.h>

#include "dev.h"
#include "class_ids.h"
#include "bus_client.h"

#include "gk20a.h"

int gk20a_ctrl_dev_open(struct inode *inode, struct file *filp)
{
	struct nvhost_device *dev;

	nvhost_dbg_fn("");

	dev = container_of(inode->i_cdev, struct nvhost_device, ctrl_cdev);

	BUG_ON(dev == NULL);

	filp->private_data = dev;

	nvhost_gk20a_init(dev);

	return 0;
}

int gk20a_ctrl_dev_release(struct inode *inode, struct file *filp)
{
	nvhost_dbg_fn("");

	return 0;
}

long gk20a_ctrl_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct nvhost_device *dev = filp->private_data;
	struct gk20a *g = get_gk20a(dev);
	struct nvhost_gpu_zcull_get_ctx_size_args *get_ctx_size_args;
	struct nvhost_gpu_zcull_get_info_args *get_info_args;
	struct gr_zcull_info zcull_info;
	u8 buf[NVHOST_GPU_IOCTL_MAX_ARG_SIZE];
	int err = 0;

	nvhost_dbg_fn("");

	if ((_IOC_TYPE(cmd) != NVHOST_GPU_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVHOST_GPU_IOCTL_LAST))
		return -EFAULT;

	BUG_ON(_IOC_SIZE(cmd) > NVHOST_GPU_IOCTL_MAX_ARG_SIZE);

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	/* TBD: module busy here ? */

	switch (cmd) {
	case NVHOST_GPU_IOCTL_ZCULL_GET_CTX_SIZE:
		get_ctx_size_args = (struct nvhost_gpu_zcull_get_ctx_size_args *)buf;

		get_ctx_size_args->size = gr_gk20a_get_ctxsw_zcull_size(g, &g->gr);

		break;
	case NVHOST_GPU_IOCTL_ZCULL_GET_INFO:
		get_info_args = (struct nvhost_gpu_zcull_get_info_args *)buf;

		memset(get_info_args, 0, sizeof(struct nvhost_gpu_zcull_get_info_args));
		memset(&zcull_info, 0, sizeof(struct gr_zcull_info));

		err = gr_gk20a_get_zcull_info(g, &g->gr, &zcull_info);
		if (err)
			break;

		get_info_args->width_align_pixels = zcull_info.width_align_pixels;
		get_info_args->height_align_pixels = zcull_info.height_align_pixels;
		get_info_args->pixel_squares_by_aliquots = zcull_info.pixel_squares_by_aliquots;
		get_info_args->aliquot_total = zcull_info.aliquot_total;
		get_info_args->region_byte_multiplier = zcull_info.region_byte_multiplier;
		get_info_args->region_header_size = zcull_info.region_header_size;
		get_info_args->subregion_header_size = zcull_info.subregion_header_size;
		get_info_args->subregion_width_align_pixels = zcull_info.subregion_width_align_pixels;
		get_info_args->subregion_height_align_pixels = zcull_info.subregion_height_align_pixels;
		get_info_args->subregion_count = zcull_info.subregion_count;

		break;
	default:
		nvhost_err(dev_from_gk20a(g), "unrecognized gpu ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	/* TBD: module idle here ? */

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

