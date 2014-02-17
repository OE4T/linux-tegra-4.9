/*
 * drivers/video/tegra/host/gk20a/as_gk20a.c
 *
 * GK20A Address Spaces
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
 */

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include <trace/events/nvhost.h>

#include "gk20a.h"

/* dumb allocator... */
static int generate_as_share_id(struct gk20a_as *as)
{
	nvhost_dbg_fn("");
	return ++as->last_share_id;
}
/* still dumb */
static void release_as_share_id(struct gk20a_as *as, int id)
{
	nvhost_dbg_fn("");
	return;
}

static int gk20a_as_alloc_share(struct gk20a_as *as,
				struct gk20a_as_share **out)
{
	struct gk20a_as_share *as_share;
	int err = 0;

	nvhost_dbg_fn("");

	*out = 0;
	as_share = kzalloc(sizeof(*as_share), GFP_KERNEL);
	if (!as_share)
		return -ENOMEM;

	as_share->as = as;
	as_share->id = generate_as_share_id(as_share->as);
	as_share->ref_cnt.counter = 1;

	/* this will set as_share->vm. */
	err = gk20a_vm_alloc_share(as_share);
	if (err)
		goto failed;

	*out = as_share;
	return 0;

 failed:
	kfree(as_share);
	return err;
}

/*
 * channels and the device nodes call this to release.
 * once the ref_cnt hits zero the share is deleted.
 */
int gk20a_as_release_share(struct gk20a_as_share *as_share)
{
	int err;

	nvhost_dbg_fn("");

	if (atomic_dec_return(&as_share->ref_cnt) > 0)
		return 0;

	err = gk20a_vm_release_share(as_share);
	release_as_share_id(as_share->as, as_share->id);
	kfree(as_share);
	return err;
}

static int gk20a_as_ioctl_bind_channel(
		struct gk20a_as_share *as_share,
		struct nvhost_as_bind_channel_args *args)
{
	int err = 0;
	struct channel_gk20a *ch;

	nvhost_dbg_fn("");

	ch = gk20a_get_channel_from_file(args->channel_fd);
	if (!ch || gk20a_channel_as_bound(ch))
		return -EINVAL;

	atomic_inc(&as_share->ref_cnt);

	/* this will set channel_gk20a->vm */
	err = gk20a_vm_bind_channel(as_share, ch);
	if (err) {
		atomic_dec(&as_share->ref_cnt);
		return err;
	}

	return err;
}

static int gk20a_as_ioctl_alloc_space(
		struct gk20a_as_share *as_share,
		struct nvhost_as_alloc_space_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_vm_alloc_space(as_share, args);
}

static int gk20a_as_ioctl_free_space(
		struct gk20a_as_share *as_share,
		struct nvhost_as_free_space_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_vm_free_space(as_share, args);
}

static int gk20a_as_ioctl_map_buffer_ex(
		struct gk20a_as_share *as_share,
		struct nvhost_as_map_buffer_ex_args *args)
{
	int i;

	nvhost_dbg_fn("");

	/* ensure that padding is not set. this is required for ensuring that
	 * we can safely use these fields later */
	for (i = 0; i < ARRAY_SIZE(args->padding); i++)
		if (args->padding[i])
			return -EINVAL;

	return gk20a_vm_map_buffer(as_share, args->dmabuf_fd,
				   &args->offset, args->flags,
				   args->kind);
}

static int gk20a_as_ioctl_map_buffer(
		struct gk20a_as_share *as_share,
		struct nvhost_as_map_buffer_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_vm_map_buffer(as_share, args->nvmap_handle,
				   &args->o_a.align,
				   args->flags, NV_KIND_DEFAULT);
	/* args->o_a.offset will be set if !err */
}

static int gk20a_as_ioctl_unmap_buffer(
		struct gk20a_as_share *as_share,
		struct nvhost_as_unmap_buffer_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_vm_unmap_buffer(as_share, args->offset);
}

int gk20a_as_dev_open(struct inode *inode, struct file *filp)
{
	struct gk20a_as_share *as_share;
	struct gk20a *g;
	int err;

	nvhost_dbg_fn("");

	g = container_of(inode->i_cdev, struct gk20a, as.cdev);

	err = gk20a_get_client(g);
	if (err) {
		nvhost_dbg_fn("fail to get channel!");
		return err;
	}

	err = gk20a_as_alloc_share(&g->as, &as_share);
	if (err) {
		nvhost_dbg_fn("failed to alloc share");
		gk20a_put_client(g);
		return err;
	}

	filp->private_data = as_share;
	return 0;
}

int gk20a_as_dev_release(struct inode *inode, struct file *filp)
{
	struct gk20a_as_share *as_share = filp->private_data;
	int ret;
	struct gk20a *g = gk20a_from_as(as_share->as);

	nvhost_dbg_fn("");

	ret = gk20a_as_release_share(as_share);

	gk20a_put_client(g);

	return ret;
}

long gk20a_as_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct gk20a_as_share *as_share = filp->private_data;
	struct gk20a *g = gk20a_from_as(as_share->as);

	u8 buf[NVHOST_AS_IOCTL_MAX_ARG_SIZE];

	if ((_IOC_TYPE(cmd) != NVHOST_AS_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVHOST_AS_IOCTL_LAST))
		return -EFAULT;

	BUG_ON(_IOC_SIZE(cmd) > NVHOST_AS_IOCTL_MAX_ARG_SIZE);

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	err = gk20a_channel_busy(g->dev);
	if (err)
		return err;

	switch (cmd) {
	case NVHOST_AS_IOCTL_BIND_CHANNEL:
		trace_nvhost_as_ioctl_bind_channel(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_bind_channel(as_share,
			       (struct nvhost_as_bind_channel_args *)buf);

		break;
	case NVHOST32_AS_IOCTL_ALLOC_SPACE:
	{
		struct nvhost32_as_alloc_space_args *args32 =
			(struct nvhost32_as_alloc_space_args *)buf;
		struct nvhost_as_alloc_space_args args;

		args.pages = args32->pages;
		args.page_size = args32->page_size;
		args.flags = args32->flags;
		args.o_a.offset = args32->o_a.offset;
		trace_nvhost_as_ioctl_alloc_space(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_alloc_space(as_share, &args);
		args32->o_a.offset = args.o_a.offset;
		break;
	}
	case NVHOST_AS_IOCTL_ALLOC_SPACE:
		trace_nvhost_as_ioctl_alloc_space(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_alloc_space(as_share,
				(struct nvhost_as_alloc_space_args *)buf);
		break;
	case NVHOST_AS_IOCTL_FREE_SPACE:
		trace_nvhost_as_ioctl_free_space(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_free_space(as_share,
				(struct nvhost_as_free_space_args *)buf);
		break;
	case NVHOST_AS_IOCTL_MAP_BUFFER:
		trace_nvhost_as_ioctl_map_buffer(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_map_buffer(as_share,
				(struct nvhost_as_map_buffer_args *)buf);
		break;
	case NVHOST_AS_IOCTL_MAP_BUFFER_EX:
		trace_nvhost_as_ioctl_map_buffer(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_map_buffer_ex(as_share,
				(struct nvhost_as_map_buffer_ex_args *)buf);
		break;
	case NVHOST_AS_IOCTL_UNMAP_BUFFER:
		trace_nvhost_as_ioctl_unmap_buffer(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_unmap_buffer(as_share,
				(struct nvhost_as_unmap_buffer_args *)buf);
		break;
	default:
		dev_err(dev_from_gk20a(g), "unrecognized as ioctl: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	gk20a_channel_idle(g->dev);

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}
