/*
 * Copyright (c) 2011-2014, NVIDIA Corporation.  All rights reserved.
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
#include <linux/file.h>
#include <linux/anon_inodes.h>
#include <uapi/linux/nvgpu.h>

#include "gk20a.h"
#include "fence_gk20a.h"

int gk20a_ctrl_dev_open(struct inode *inode, struct file *filp)
{
	int err;
	struct gk20a *g;

	gk20a_dbg_fn("");

	g = container_of(inode->i_cdev,
			 struct gk20a, ctrl.cdev);

	filp->private_data = g->dev;

	err = gk20a_get_client(g);
	if (err) {
		gk20a_dbg_fn("fail to get channel!");
		return err;
	}

	return 0;
}

int gk20a_ctrl_dev_release(struct inode *inode, struct file *filp)
{
	struct platform_device *dev = filp->private_data;

	gk20a_dbg_fn("");

	gk20a_put_client(get_gk20a(dev));
	return 0;
}

static long
gk20a_ctrl_ioctl_gpu_characteristics(
	struct gk20a *g,
	struct nvgpu_gpu_get_characteristics *request)
{
	struct nvgpu_gpu_characteristics *pgpu = &g->gpu_characteristics;
	long err = 0;

	if (request->gpu_characteristics_buf_size > 0) {
		size_t write_size = sizeof(*pgpu);

		if (write_size > request->gpu_characteristics_buf_size)
			write_size = request->gpu_characteristics_buf_size;

		err = copy_to_user((void __user *)(uintptr_t)
				   request->gpu_characteristics_buf_addr,
				   pgpu, write_size);
	}

	if (err == 0)
		request->gpu_characteristics_buf_size = sizeof(*pgpu);

	return err;
}

static int gk20a_ctrl_prepare_compressible_read(
		struct gk20a *g,
		struct nvgpu_gpu_prepare_compressible_read_args *args)
{
	struct nvgpu_fence fence;
	struct gk20a_fence *fence_out = NULL;
	int ret = 0;
	int flags = args->submit_flags;

	fence.id = args->fence.syncpt_id;
	fence.value = args->fence.syncpt_value;

	ret = gk20a_busy(g->dev);
	if (ret)
		return ret;

	ret = gk20a_prepare_compressible_read(g, args->handle,
			args->request_compbits, args->offset,
			args->compbits_hoffset, args->compbits_voffset,
			args->width, args->height, args->block_height_log2,
			flags, &fence, &args->valid_compbits,
			&args->zbc_color, &fence_out);
	gk20a_idle(g->dev);

	if (ret)
		return ret;

	/* Convert fence_out to something we can pass back to user space. */
	if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET) {
		if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE) {
			if (fence_out) {
				int fd = gk20a_fence_install_fd(fence_out);
				if (fd < 0)
					ret = fd;
				else
					args->fence.fd = fd;
			} else {
				args->fence.fd = -1;
			}
		} else {
			if (fence_out) {
				args->fence.syncpt_id = fence_out->syncpt_id;
				args->fence.syncpt_value =
						fence_out->syncpt_value;
			} else {
				args->fence.syncpt_id = -1;
				args->fence.syncpt_value = 0;
			}
		}
	}
	gk20a_fence_put(fence_out);

	return 0;
}

static int gk20a_ctrl_mark_compressible_write(
		struct gk20a *g,
		struct nvgpu_gpu_mark_compressible_write_args *args)
{
	int ret;

	ret = gk20a_busy(g->dev);
	if (ret)
		return ret;
	ret = gk20a_mark_compressible_write(g, args->handle,
			args->valid_compbits, args->offset, args->zbc_color);
	gk20a_idle(g->dev);

	return ret;
}

static int gk20a_ctrl_alloc_as(
		struct gk20a *g,
		struct nvgpu_alloc_as_args *args)
{
	struct platform_device *dev = g->dev;
	struct gk20a_as_share *as_share;
	int err;
	int fd;
	struct file *file;
	char *name;

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		return err;
	fd = err;

	name = kasprintf(GFP_KERNEL, "nvhost-%s-fd%d",
			dev_name(&dev->dev), fd);

	file = anon_inode_getfile(name, g->as.cdev.ops, NULL, O_RDWR);
	kfree(name);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}
	fd_install(fd, file);

	err = gk20a_get_client(g);
	if (err)
		goto clean_up;

	err = gk20a_as_alloc_share(&g->as, args->big_page_size, &as_share);
	if (err)
		goto clean_up_client;

	file->private_data = as_share;

	args->as_fd = fd;
	return 0;

clean_up_client:
	gk20a_put_client(g);
clean_up:
	put_unused_fd(fd);
	return err;
}

long gk20a_ctrl_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct platform_device *dev = filp->private_data;
	struct gk20a *g = get_gk20a(dev);
	struct nvgpu_gpu_zcull_get_ctx_size_args *get_ctx_size_args;
	struct nvgpu_gpu_zcull_get_info_args *get_info_args;
	struct nvgpu_gpu_zbc_set_table_args *set_table_args;
	struct nvgpu_gpu_zbc_query_table_args *query_table_args;
	u8 buf[NVGPU_GPU_IOCTL_MAX_ARG_SIZE];
	struct gr_zcull_info *zcull_info;
	struct zbc_entry *zbc_val;
	struct zbc_query_params *zbc_tbl;
	int i, err = 0;
	struct gk20a_platform *platform = platform_get_drvdata(dev);

	gk20a_dbg_fn("");

	if ((_IOC_TYPE(cmd) != NVGPU_GPU_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVGPU_GPU_IOCTL_LAST))
		return -EINVAL;

	BUG_ON(_IOC_SIZE(cmd) > NVGPU_GPU_IOCTL_MAX_ARG_SIZE);

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	if (!g->gr.sw_ready) {
		err = gk20a_busy(g->dev);
		if (err)
			return err;

		gk20a_idle(g->dev);
	}

	switch (cmd) {
	case NVGPU_GPU_IOCTL_ZCULL_GET_CTX_SIZE:
		get_ctx_size_args = (struct nvgpu_gpu_zcull_get_ctx_size_args *)buf;

		get_ctx_size_args->size = gr_gk20a_get_ctxsw_zcull_size(g, &g->gr);

		break;
	case NVGPU_GPU_IOCTL_ZCULL_GET_INFO:
		get_info_args = (struct nvgpu_gpu_zcull_get_info_args *)buf;

		memset(get_info_args, 0, sizeof(struct nvgpu_gpu_zcull_get_info_args));

		zcull_info = kzalloc(sizeof(struct gr_zcull_info), GFP_KERNEL);
		if (zcull_info == NULL)
			return -ENOMEM;

		err = g->ops.gr.get_zcull_info(g, &g->gr, zcull_info);
		if (err) {
			kfree(zcull_info);
			break;
		}

		get_info_args->width_align_pixels = zcull_info->width_align_pixels;
		get_info_args->height_align_pixels = zcull_info->height_align_pixels;
		get_info_args->pixel_squares_by_aliquots = zcull_info->pixel_squares_by_aliquots;
		get_info_args->aliquot_total = zcull_info->aliquot_total;
		get_info_args->region_byte_multiplier = zcull_info->region_byte_multiplier;
		get_info_args->region_header_size = zcull_info->region_header_size;
		get_info_args->subregion_header_size = zcull_info->subregion_header_size;
		get_info_args->subregion_width_align_pixels = zcull_info->subregion_width_align_pixels;
		get_info_args->subregion_height_align_pixels = zcull_info->subregion_height_align_pixels;
		get_info_args->subregion_count = zcull_info->subregion_count;

		kfree(zcull_info);
		break;
	case NVGPU_GPU_IOCTL_ZBC_SET_TABLE:
		set_table_args = (struct nvgpu_gpu_zbc_set_table_args *)buf;

		if (platform->virtual_dev)
			return -ENOMEM;

		zbc_val = kzalloc(sizeof(struct zbc_entry), GFP_KERNEL);
		if (zbc_val == NULL)
			return -ENOMEM;

		zbc_val->format = set_table_args->format;
		zbc_val->type = set_table_args->type;

		switch (zbc_val->type) {
		case GK20A_ZBC_TYPE_COLOR:
			for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
				zbc_val->color_ds[i] = set_table_args->color_ds[i];
				zbc_val->color_l2[i] = set_table_args->color_l2[i];
			}
			break;
		case GK20A_ZBC_TYPE_DEPTH:
			zbc_val->depth = set_table_args->depth;
			break;
		default:
			err = -EINVAL;
		}

		if (!err) {
			err = gk20a_busy(dev);
			if (!err)
				err = gk20a_gr_zbc_set_table(g, &g->gr,
							     zbc_val);
			gk20a_idle(dev);
		}

		if (zbc_val)
			kfree(zbc_val);
		break;
	case NVGPU_GPU_IOCTL_ZBC_QUERY_TABLE:
		query_table_args = (struct nvgpu_gpu_zbc_query_table_args *)buf;

		zbc_tbl = kzalloc(sizeof(struct zbc_query_params), GFP_KERNEL);
		if (zbc_tbl == NULL)
			return -ENOMEM;

		zbc_tbl->type = query_table_args->type;
		zbc_tbl->index_size = query_table_args->index_size;

		err = gr_gk20a_query_zbc(g, &g->gr, zbc_tbl);

		if (!err) {
			switch (zbc_tbl->type) {
			case GK20A_ZBC_TYPE_COLOR:
				for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
					query_table_args->color_ds[i] = zbc_tbl->color_ds[i];
					query_table_args->color_l2[i] = zbc_tbl->color_l2[i];
				}
				break;
			case GK20A_ZBC_TYPE_DEPTH:
				query_table_args->depth = zbc_tbl->depth;
				break;
			case GK20A_ZBC_TYPE_INVALID:
				query_table_args->index_size = zbc_tbl->index_size;
				break;
			default:
				err = -EINVAL;
			}
			if (!err) {
				query_table_args->format = zbc_tbl->format;
				query_table_args->ref_cnt = zbc_tbl->ref_cnt;
			}
		}

		if (zbc_tbl)
			kfree(zbc_tbl);
		break;

	case NVGPU_GPU_IOCTL_GET_CHARACTERISTICS:
		err = gk20a_ctrl_ioctl_gpu_characteristics(
			g, (struct nvgpu_gpu_get_characteristics *)buf);
		break;
	case NVGPU_GPU_IOCTL_PREPARE_COMPRESSIBLE_READ:
		err = gk20a_ctrl_prepare_compressible_read(g,
			(struct nvgpu_gpu_prepare_compressible_read_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_MARK_COMPRESSIBLE_WRITE:
		err = gk20a_ctrl_mark_compressible_write(g,
			(struct nvgpu_gpu_mark_compressible_write_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_ALLOC_AS:
		err = gk20a_ctrl_alloc_as(g,
			(struct nvgpu_alloc_as_args *)buf);
		break;
	default:
		dev_dbg(dev_from_gk20a(g), "unrecognized gpu ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

