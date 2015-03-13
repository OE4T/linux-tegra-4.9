/*
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

#include <trace/events/gk20a.h>

#include <uapi/linux/nvgpu.h>

#include "gk20a.h"

/* dumb allocator... */
static int generate_as_share_id(struct gk20a_as *as)
{
	gk20a_dbg_fn("");
	return ++as->last_share_id;
}
/* still dumb */
static void release_as_share_id(struct gk20a_as *as, int id)
{
	gk20a_dbg_fn("");
	return;
}

int gk20a_as_alloc_share(struct gk20a_as *as,
			 u32 flags, struct gk20a_as_share **out)
{
	struct gk20a *g = gk20a_from_as(as);
	struct gk20a_as_share *as_share;
	int err = 0;

	gk20a_dbg_fn("");

	*out = NULL;
	as_share = kzalloc(sizeof(*as_share), GFP_KERNEL);
	if (!as_share)
		return -ENOMEM;

	as_share->as = as;
	as_share->id = generate_as_share_id(as_share->as);
	as_share->ref_cnt.counter = 1;

	/* this will set as_share->vm. */
	err = gk20a_busy(g->dev);
	if (err)
		goto failed;
	err = g->ops.mm.vm_alloc_share(as_share, flags);
	gk20a_idle(g->dev);

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
	struct gk20a *g = as_share->vm->mm->g;
	int err;

	gk20a_dbg_fn("");

	if (atomic_dec_return(&as_share->ref_cnt) > 0)
		return 0;

	err = gk20a_busy(g->dev);
	if (err)
		return err;

	err = gk20a_vm_release_share(as_share);

	gk20a_idle(g->dev);

	release_as_share_id(as_share->as, as_share->id);
	kfree(as_share);
	return err;
}

static int gk20a_as_ioctl_bind_channel(
		struct gk20a_as_share *as_share,
		struct nvgpu_as_bind_channel_args *args)
{
	int err = 0;
	struct channel_gk20a *ch;

	gk20a_dbg_fn("");

	ch = gk20a_get_channel_from_file(args->channel_fd);
	if (!ch || gk20a_channel_as_bound(ch))
		return -EINVAL;

	atomic_inc(&as_share->ref_cnt);

	/* this will set channel_gk20a->vm */
	err = ch->g->ops.mm.vm_bind_channel(as_share, ch);
	if (err) {
		atomic_dec(&as_share->ref_cnt);
		return err;
	}

	return err;
}

static int gk20a_as_ioctl_alloc_space(
		struct gk20a_as_share *as_share,
		struct nvgpu_as_alloc_space_args *args)
{
	gk20a_dbg_fn("");
	return gk20a_vm_alloc_space(as_share, args);
}

static int gk20a_as_ioctl_free_space(
		struct gk20a_as_share *as_share,
		struct nvgpu_as_free_space_args *args)
{
	gk20a_dbg_fn("");
	return gk20a_vm_free_space(as_share, args);
}

static int gk20a_as_ioctl_map_buffer_ex(
		struct gk20a_as_share *as_share,
		struct nvgpu_as_map_buffer_ex_args *args)
{
	gk20a_dbg_fn("");

	return gk20a_vm_map_buffer(as_share->vm, args->dmabuf_fd,
				   &args->offset, args->flags,
				   args->kind,
				   args->buffer_offset,
				   args->mapping_size
				   );
}

static int gk20a_as_ioctl_map_buffer(
		struct gk20a_as_share *as_share,
		struct nvgpu_as_map_buffer_args *args)
{
	gk20a_dbg_fn("");
	return gk20a_vm_map_buffer(as_share->vm, args->dmabuf_fd,
				   &args->o_a.offset,
				   args->flags, NV_KIND_DEFAULT,
				   0, 0);
	/* args->o_a.offset will be set if !err */
}

static int gk20a_as_ioctl_unmap_buffer(
		struct gk20a_as_share *as_share,
		struct nvgpu_as_unmap_buffer_args *args)
{
	gk20a_dbg_fn("");
	return gk20a_vm_unmap_buffer(as_share->vm, args->offset);
}

static int gk20a_as_ioctl_get_va_regions(
		struct gk20a_as_share *as_share,
		struct nvgpu_as_get_va_regions_args *args)
{
	unsigned int i;
	unsigned int write_entries;
	struct nvgpu_as_va_region __user *user_region_ptr;
	struct vm_gk20a *vm = as_share->vm;
	int page_sizes = gmmu_nr_page_sizes;

	gk20a_dbg_fn("");

	if (!vm->big_pages)
		page_sizes--;

	write_entries = args->buf_size / sizeof(struct nvgpu_as_va_region);
	if (write_entries > page_sizes)
		write_entries = page_sizes;

	user_region_ptr =
		(struct nvgpu_as_va_region __user *)(uintptr_t)args->buf_addr;

	for (i = 0; i < write_entries; ++i) {
		struct nvgpu_as_va_region region;
		u32 base, limit;

		memset(&region, 0, sizeof(struct nvgpu_as_va_region));

		if (!vm->vma[i].constraint.enable) {
			base = vm->vma[i].base;
			limit = vm->vma[i].limit;
		} else {
			base = vm->vma[i].constraint.base;
			limit = vm->vma[i].constraint.limit;
		}

		region.page_size = vm->gmmu_page_sizes[i];
		region.offset = (u64)base * region.page_size;
		region.pages = limit - base; /* NOTE: limit is exclusive */

		if (copy_to_user(user_region_ptr + i, &region, sizeof(region)))
			return -EFAULT;
	}

	args->buf_size =
		page_sizes * sizeof(struct nvgpu_as_va_region);

	return 0;
}

int gk20a_as_dev_open(struct inode *inode, struct file *filp)
{
	struct gk20a_as_share *as_share;
	struct gk20a *g;
	int err;

	gk20a_dbg_fn("");

	g = container_of(inode->i_cdev, struct gk20a, as.cdev);

	err = gk20a_as_alloc_share(&g->as, 0, &as_share);
	if (err) {
		gk20a_dbg_fn("failed to alloc share");
		return err;
	}

	filp->private_data = as_share;
	return 0;
}

int gk20a_as_dev_release(struct inode *inode, struct file *filp)
{
	struct gk20a_as_share *as_share = filp->private_data;

	gk20a_dbg_fn("");

	if (!as_share)
		return 0;

	return gk20a_as_release_share(as_share);
}

long gk20a_as_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct gk20a_as_share *as_share = filp->private_data;
	struct gk20a *g = gk20a_from_as(as_share->as);

	u8 buf[NVGPU_AS_IOCTL_MAX_ARG_SIZE];

	if ((_IOC_TYPE(cmd) != NVGPU_AS_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVGPU_AS_IOCTL_LAST))
		return -EINVAL;

	BUG_ON(_IOC_SIZE(cmd) > NVGPU_AS_IOCTL_MAX_ARG_SIZE);

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	err = gk20a_busy(g->dev);
	if (err)
		return err;

	switch (cmd) {
	case NVGPU_AS_IOCTL_BIND_CHANNEL:
		trace_gk20a_as_ioctl_bind_channel(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_bind_channel(as_share,
			       (struct nvgpu_as_bind_channel_args *)buf);

		break;
	case NVGPU32_AS_IOCTL_ALLOC_SPACE:
	{
		struct nvgpu32_as_alloc_space_args *args32 =
			(struct nvgpu32_as_alloc_space_args *)buf;
		struct nvgpu_as_alloc_space_args args;

		args.pages = args32->pages;
		args.page_size = args32->page_size;
		args.flags = args32->flags;
		args.o_a.offset = args32->o_a.offset;
		trace_gk20a_as_ioctl_alloc_space(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_alloc_space(as_share, &args);
		args32->o_a.offset = args.o_a.offset;
		break;
	}
	case NVGPU_AS_IOCTL_ALLOC_SPACE:
		trace_gk20a_as_ioctl_alloc_space(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_alloc_space(as_share,
				(struct nvgpu_as_alloc_space_args *)buf);
		break;
	case NVGPU_AS_IOCTL_FREE_SPACE:
		trace_gk20a_as_ioctl_free_space(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_free_space(as_share,
				(struct nvgpu_as_free_space_args *)buf);
		break;
	case NVGPU_AS_IOCTL_MAP_BUFFER:
		trace_gk20a_as_ioctl_map_buffer(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_map_buffer(as_share,
				(struct nvgpu_as_map_buffer_args *)buf);
		break;
	case NVGPU_AS_IOCTL_MAP_BUFFER_EX:
		trace_gk20a_as_ioctl_map_buffer(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_map_buffer_ex(as_share,
				(struct nvgpu_as_map_buffer_ex_args *)buf);
		break;
	case NVGPU_AS_IOCTL_UNMAP_BUFFER:
		trace_gk20a_as_ioctl_unmap_buffer(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_unmap_buffer(as_share,
				(struct nvgpu_as_unmap_buffer_args *)buf);
		break;
	case NVGPU_AS_IOCTL_GET_VA_REGIONS:
		trace_gk20a_as_ioctl_get_va_regions(dev_name(dev_from_gk20a(g)));
		err = gk20a_as_ioctl_get_va_regions(as_share,
				(struct nvgpu_as_get_va_regions_args *)buf);
		break;
	default:
		dev_dbg(dev_from_gk20a(g), "unrecognized as ioctl: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	gk20a_idle(g->dev);

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		if (copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd)))
			err = -EFAULT;

	return err;
}
