/*
 * drivers/video/tegra/host/dev.c
 *
 * Tegra Graphics Host Driver Entrypoint
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "dev.h"

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/file.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>
#define CREATE_TRACE_POINTS
#include <trace/events/nvhost.h>

#include <asm/io.h>

#include <linux/nvhost.h>
#include <linux/nvhost_ioctl.h>
#include <mach/nvmap.h>
#include <mach/gpufuse.h>
#include <mach/hardware.h>

#include "nvhost_scale.h"
#include "debug.h"

#define DRIVER_NAME "tegra_grhost"
#define IFACE_NAME "nvhost"

static int nvhost_major = NVHOST_MAJOR;
static int nvhost_minor;
static unsigned int register_sets;

struct nvhost_channel_userctx {
	struct nvhost_channel *ch;
	struct nvhost_hwctx *hwctx;
	struct nvhost_submit_hdr_ext hdr;
	struct nvmap_handle_ref *gather_mem;
	u32 *gathers;
	u32 *cur_gather;
	int pinarray_size;
	struct nvmap_pinarray_elem pinarray[NVHOST_MAX_HANDLES];
	struct nvmap_handle *unpinarray[NVHOST_MAX_HANDLES];
	struct nvmap_client *nvmap;
	struct nvhost_waitchk waitchks[NVHOST_MAX_WAIT_CHECKS];
	struct nvhost_waitchk *cur_waitchk;
	struct nvhost_userctx_timeout timeout;
};

struct nvhost_ctrl_userctx {
	struct nvhost_master *dev;
	u32 *mod_locks;
};

static int nvhost_channelrelease(struct inode *inode, struct file *filp)
{
	struct nvhost_channel_userctx *priv = filp->private_data;

	trace_nvhost_channel_release(priv->ch->desc->name);

	filp->private_data = NULL;

	nvhost_module_remove_client(&priv->ch->mod, priv);
	nvhost_putchannel(priv->ch, priv->hwctx);

	if (priv->hwctx)
		priv->ch->ctxhandler.put(priv->hwctx);

	if (priv->gathers)
		nvmap_munmap(priv->gather_mem, priv->gathers);

	if (!IS_ERR_OR_NULL(priv->gather_mem))
		nvmap_free(priv->ch->dev->nvmap, priv->gather_mem);

	nvmap_client_put(priv->nvmap);
	kfree(priv);
	return 0;
}

static int nvhost_channelopen(struct inode *inode, struct file *filp)
{
	struct nvhost_channel_userctx *priv;
	struct nvhost_channel *ch;


	ch = container_of(inode->i_cdev, struct nvhost_channel, cdev);
	ch = nvhost_getchannel(ch);
	if (!ch)
		return -ENOMEM;
	trace_nvhost_channel_open(ch->desc->name);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		nvhost_putchannel(ch, NULL);
		return -ENOMEM;
	}
	filp->private_data = priv;
	priv->ch = ch;
	nvhost_module_add_client(&ch->mod, priv);
	priv->gather_mem = nvmap_alloc(ch->dev->nvmap,
				sizeof(u32) * 2 * NVHOST_MAX_GATHERS, 32,
				NVMAP_HANDLE_CACHEABLE);
	if (IS_ERR(priv->gather_mem))
		goto fail;

	if (ch->ctxhandler.alloc) {
		priv->hwctx = ch->ctxhandler.alloc(ch);
		if (!priv->hwctx)
			goto fail;
		priv->hwctx->timeout = &priv->timeout;
	}

	priv->gathers = nvmap_mmap(priv->gather_mem);

	return 0;
fail:
	nvhost_channelrelease(inode, filp);
	return -ENOMEM;
}

static void add_gather(struct nvhost_channel_userctx *ctx,
		u32 mem_id, u32 words, u32 offset)
{
	struct nvmap_pinarray_elem *pin;
	u32* cur_gather = ctx->cur_gather;
	pin = &ctx->pinarray[ctx->pinarray_size++];
	pin->patch_mem = (u32)nvmap_ref_to_handle(ctx->gather_mem);
	pin->patch_offset = ((cur_gather + 1) - ctx->gathers) * sizeof(u32);
	pin->pin_mem = mem_id;
	pin->pin_offset = offset;
	cur_gather[0] = words;
	ctx->cur_gather = cur_gather + 2;
}

static int set_submit(struct nvhost_channel_userctx *ctx)
{
	/* submit should have at least 1 cmdbuf */
	if (!ctx->hdr.num_cmdbufs)
		return -EIO;

	/* check submit doesn't exceed static structs */
	if ((ctx->hdr.num_cmdbufs + ctx->hdr.num_relocs) > NVHOST_MAX_HANDLES) {
		dev_err(&ctx->ch->dev->pdev->dev,
			"channel submit exceeded max handles (%d > %d)\n",
			ctx->hdr.num_cmdbufs + ctx->hdr.num_relocs,
			NVHOST_MAX_HANDLES);
		return -EIO;
	}
	if (ctx->hdr.num_waitchks > NVHOST_MAX_WAIT_CHECKS) {
		dev_err(&ctx->ch->dev->pdev->dev,
			"channel submit exceeded max waitchks (%d > %d)\n",
			ctx->hdr.num_waitchks,
			NVHOST_MAX_WAIT_CHECKS);
		return -EIO;
	}

	ctx->cur_gather = ctx->gathers;
	ctx->cur_waitchk = ctx->waitchks;
	ctx->pinarray_size = 0;

	return 0;
}

static void reset_submit(struct nvhost_channel_userctx *ctx)
{
	ctx->hdr.num_cmdbufs = 0;
	ctx->hdr.num_relocs = 0;
	ctx->hdr.num_waitchks = 0;
}

static ssize_t nvhost_channelwrite(struct file *filp, const char __user *buf,
				size_t count, loff_t *offp)
{
	struct nvhost_channel_userctx *priv = filp->private_data;
	size_t remaining = count;
	int err = 0;

	while (remaining) {
		size_t consumed;
		if (!priv->hdr.num_relocs &&
		    !priv->hdr.num_cmdbufs &&
		    !priv->hdr.num_waitchks) {
			consumed = sizeof(struct nvhost_submit_hdr);
			if (remaining < consumed)
				break;
			if (copy_from_user(&priv->hdr, buf, consumed)) {
				err = -EFAULT;
				break;
			}
			priv->hdr.submit_version = NVHOST_SUBMIT_VERSION_V0;
			err = set_submit(priv);
			if (err)
				break;
			trace_nvhost_channel_write_submit(priv->ch->desc->name,
			  count, priv->hdr.num_cmdbufs, priv->hdr.num_relocs);
		} else if (priv->hdr.num_cmdbufs) {
			struct nvhost_cmdbuf cmdbuf;
			consumed = sizeof(cmdbuf);
			if (remaining < consumed)
				break;
			if (copy_from_user(&cmdbuf, buf, consumed)) {
				err = -EFAULT;
				break;
			}
			trace_nvhost_channel_write_cmdbuf(priv->ch->desc->name,
			  cmdbuf.mem, cmdbuf.words, cmdbuf.offset);
			add_gather(priv,
				cmdbuf.mem, cmdbuf.words, cmdbuf.offset);
			priv->hdr.num_cmdbufs--;
		} else if (priv->hdr.num_relocs) {
			int numrelocs = remaining / sizeof(struct nvhost_reloc);
			if (!numrelocs)
				break;
			numrelocs = min_t(int, numrelocs, priv->hdr.num_relocs);
			consumed = numrelocs * sizeof(struct nvhost_reloc);
			if (copy_from_user(&priv->pinarray[priv->pinarray_size],
						buf, consumed)) {
				err = -EFAULT;
				break;
			}
			trace_nvhost_channel_write_relocs(priv->ch->desc->name,
			  numrelocs);
			priv->pinarray_size += numrelocs;
			priv->hdr.num_relocs -= numrelocs;
		} else if (priv->hdr.num_waitchks) {
			int numwaitchks =
				(remaining / sizeof(struct nvhost_waitchk));
			if (!numwaitchks)
				break;
			numwaitchks = min_t(int,
				numwaitchks, priv->hdr.num_waitchks);
			consumed = numwaitchks * sizeof(struct nvhost_waitchk);
			if (copy_from_user(priv->cur_waitchk, buf, consumed)) {
				err = -EFAULT;
				break;
			}
			trace_nvhost_channel_write_waitchks(
			  priv->ch->desc->name, numwaitchks,
			  priv->hdr.waitchk_mask);
			priv->cur_waitchk += numwaitchks;
			priv->hdr.num_waitchks -= numwaitchks;
		} else {
			err = -EFAULT;
			break;
		}
		remaining -= consumed;
		buf += consumed;
	}

	if (err < 0) {
		dev_err(&priv->ch->dev->pdev->dev, "channel write error\n");
		reset_submit(priv);
		return err;
	}

	return (count - remaining);
}

static int nvhost_ioctl_channel_flush(
	struct nvhost_channel_userctx *ctx,
	struct nvhost_get_param_args *args,
	int null_kickoff)
{
	struct device *device = &ctx->ch->dev->pdev->dev;
	int num_unpin;
	int err;

	trace_nvhost_ioctl_channel_flush(ctx->ch->desc->name);

	if (ctx->hdr.num_relocs ||
	    ctx->hdr.num_cmdbufs ||
	    ctx->hdr.num_waitchks) {
		reset_submit(ctx);
		dev_err(device, "channel submit out of sync\n");
		return -EFAULT;
	}
	if (!ctx->nvmap) {
		dev_err(device, "no nvmap context set\n");
		return -EFAULT;
	}
	if (ctx->cur_gather == ctx->gathers)
		return 0;

	/* pin mem handles and patch physical addresses */
	num_unpin = nvmap_pin_array(ctx->nvmap,
				    nvmap_ref_to_handle(ctx->gather_mem),
				    ctx->pinarray, ctx->pinarray_size,
				    ctx->unpinarray);
	if (num_unpin < 0) {
		dev_warn(device, "nvmap_pin_array failed: %d\n", num_unpin);
		return num_unpin;
	}

	if (nvhost_debug_null_kickoff_pid == current->tgid)
		null_kickoff = 1;

	if ((nvhost_debug_force_timeout_pid == current->tgid) &&
	    (nvhost_debug_force_timeout_channel == ctx->ch->chid)) {
		ctx->timeout.timeout = nvhost_debug_force_timeout_val;
	}
	ctx->timeout.syncpt_id = ctx->hdr.syncpt_id;

	/* context switch if needed, and submit user's gathers to the channel */
	BUG_ON(!channel_op(ctx->ch).submit);
	err = channel_op(ctx->ch).submit(ctx->ch, ctx->hwctx, ctx->nvmap,
				ctx->gathers, ctx->cur_gather,
				ctx->waitchks, ctx->cur_waitchk,
				ctx->hdr.waitchk_mask,
				ctx->unpinarray, num_unpin,
				ctx->hdr.syncpt_id, ctx->hdr.syncpt_incrs,
				&ctx->timeout,
				&args->value,
				null_kickoff);
	if (err)
		nvmap_unpin_handles(ctx->nvmap, ctx->unpinarray, num_unpin);

	return 0;
}

static int nvhost_ioctl_channel_read_3d_reg(
	struct nvhost_channel_userctx *ctx,
	struct nvhost_read_3d_reg_args *args)
{
	BUG_ON(!channel_op(ctx->ch).read3dreg);
	return channel_op(ctx->ch).read3dreg(ctx->ch, ctx->hwctx,
			&ctx->timeout,
			args->offset, &args->value);
}

static long nvhost_channelctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct nvhost_channel_userctx *priv = filp->private_data;
	u8 buf[NVHOST_IOCTL_CHANNEL_MAX_ARG_SIZE];
	int err = 0;

	if ((_IOC_TYPE(cmd) != NVHOST_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVHOST_IOCTL_CHANNEL_LAST))
		return -EFAULT;

	BUG_ON(_IOC_SIZE(cmd) > NVHOST_IOCTL_CHANNEL_MAX_ARG_SIZE);

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	switch (cmd) {
	case NVHOST_IOCTL_CHANNEL_FLUSH:
		err = nvhost_ioctl_channel_flush(priv, (void *)buf, 0);
		break;
	case NVHOST_IOCTL_CHANNEL_NULL_KICKOFF:
		err = nvhost_ioctl_channel_flush(priv, (void *)buf, 1);
		break;
	case NVHOST_IOCTL_CHANNEL_SUBMIT_EXT:
	{
		struct nvhost_submit_hdr_ext *hdr;

		if (priv->hdr.num_relocs ||
		    priv->hdr.num_cmdbufs ||
		    priv->hdr.num_waitchks) {
			reset_submit(priv);
			dev_err(&priv->ch->dev->pdev->dev,
				"channel submit out of sync\n");
			err = -EIO;
			break;
		}

		hdr = (struct nvhost_submit_hdr_ext *)buf;
		if (hdr->submit_version > NVHOST_SUBMIT_VERSION_MAX_SUPPORTED) {
			dev_err(&priv->ch->dev->pdev->dev,
				"submit version %d > max supported %d\n",
				hdr->submit_version,
				NVHOST_SUBMIT_VERSION_MAX_SUPPORTED);
			err = -EINVAL;
			break;
		}
		memcpy(&priv->hdr, hdr, sizeof(struct nvhost_submit_hdr_ext));
		err = set_submit(priv);
		trace_nvhost_ioctl_channel_submit(priv->ch->desc->name,
			priv->hdr.submit_version,
			priv->hdr.num_cmdbufs, priv->hdr.num_relocs,
			priv->hdr.num_waitchks);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_SYNCPOINTS:
		/* host syncpt ID is used by the RM (and never be given out) */
		BUG_ON(priv->ch->desc->syncpts & (1 << NVSYNCPT_GRAPHICS_HOST));
		((struct nvhost_get_param_args *)buf)->value =
			priv->ch->desc->syncpts;
		break;
	case NVHOST_IOCTL_CHANNEL_GET_WAITBASES:
		((struct nvhost_get_param_args *)buf)->value =
			priv->ch->desc->waitbases;
		break;
	case NVHOST_IOCTL_CHANNEL_GET_MODMUTEXES:
		((struct nvhost_get_param_args *)buf)->value =
			priv->ch->desc->modulemutexes;
		break;
	case NVHOST_IOCTL_CHANNEL_SET_NVMAP_FD:
	{
		int fd = (int)((struct nvhost_set_nvmap_fd_args *)buf)->fd;
		struct nvmap_client *new_client = nvmap_client_get_file(fd);

		if (IS_ERR(new_client)) {
			err = PTR_ERR(new_client);
			break;
		}

		if (priv->nvmap)
			nvmap_client_put(priv->nvmap);

		priv->nvmap = new_client;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_READ_3D_REG:
		err = nvhost_ioctl_channel_read_3d_reg(priv, (void *)buf);
		break;
	case NVHOST_IOCTL_CHANNEL_GET_CLK_RATE:
	{
		unsigned long rate;
		struct nvhost_clk_rate_args *arg =
				(struct nvhost_clk_rate_args *)buf;

		err = nvhost_module_get_rate(&priv->ch->mod, &rate, 0);
		if (err == 0)
			arg->rate = rate;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_CLK_RATE:
	{
		struct nvhost_clk_rate_args *arg =
				(struct nvhost_clk_rate_args *)buf;
		unsigned long rate = (unsigned long)arg->rate;

		err = nvhost_module_set_rate(&priv->ch->mod, priv, rate, 0);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_TIMEOUT:
		priv->timeout.timeout =
			(u32)((struct nvhost_set_timeout_args *)buf)->timeout;
		dev_dbg(&priv->ch->dev->pdev->dev,
			"%s: setting buffer timeout (%d ms) for userctx 0x%p\n",
			__func__, priv->timeout.timeout, priv);
		break;
	case NVHOST_IOCTL_CHANNEL_GET_TIMEDOUT:
		((struct nvhost_get_param_args *)buf)->value =
				priv->timeout.has_timedout;
		break;
	default:
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

static struct file_operations nvhost_channelops = {
	.owner = THIS_MODULE,
	.release = nvhost_channelrelease,
	.open = nvhost_channelopen,
	.write = nvhost_channelwrite,
	.unlocked_ioctl = nvhost_channelctl
};

static int nvhost_ctrlrelease(struct inode *inode, struct file *filp)
{
	struct nvhost_ctrl_userctx *priv = filp->private_data;
	int i;

	trace_nvhost_ctrlrelease(priv->dev->mod.name);

	filp->private_data = NULL;
	if (priv->mod_locks[0])
		nvhost_module_idle(&priv->dev->mod);
	for (i = 1; i < priv->dev->nb_mlocks; i++)
		if (priv->mod_locks[i])
			nvhost_mutex_unlock(&priv->dev->cpuaccess, i);
	kfree(priv->mod_locks);
	kfree(priv);
	return 0;
}

static int nvhost_ctrlopen(struct inode *inode, struct file *filp)
{
	struct nvhost_master *host = container_of(inode->i_cdev, struct nvhost_master, cdev);
	struct nvhost_ctrl_userctx *priv;
	u32 *mod_locks;

	trace_nvhost_ctrlopen(host->mod.name);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	mod_locks = kzalloc(sizeof(u32)*host->nb_mlocks, GFP_KERNEL);

	if (!(priv && mod_locks)) {
		kfree(priv);
		kfree(mod_locks);
		return -ENOMEM;
	}

	priv->dev = host;
	priv->mod_locks = mod_locks;
	filp->private_data = priv;
	return 0;
}

static int nvhost_ioctl_ctrl_syncpt_read(
	struct nvhost_ctrl_userctx *ctx,
	struct nvhost_ctrl_syncpt_read_args *args)
{
	if (args->id >= ctx->dev->syncpt.nb_pts)
		return -EINVAL;
	trace_nvhost_ioctl_ctrl_syncpt_read(args->id);
	args->value = nvhost_syncpt_read(&ctx->dev->syncpt, args->id);
	return 0;
}

static int nvhost_ioctl_ctrl_syncpt_incr(
	struct nvhost_ctrl_userctx *ctx,
	struct nvhost_ctrl_syncpt_incr_args *args)
{
	if (args->id >= ctx->dev->syncpt.nb_pts)
		return -EINVAL;
	trace_nvhost_ioctl_ctrl_syncpt_incr(args->id);
	nvhost_syncpt_incr(&ctx->dev->syncpt, args->id);
	return 0;
}

static int nvhost_ioctl_ctrl_syncpt_waitex(
	struct nvhost_ctrl_userctx *ctx,
	struct nvhost_ctrl_syncpt_waitex_args *args)
{
	u32 timeout;
	if (args->id >= ctx->dev->syncpt.nb_pts)
		return -EINVAL;
	if (args->timeout == NVHOST_NO_TIMEOUT)
		timeout = MAX_SCHEDULE_TIMEOUT;
	else
		timeout = (u32)msecs_to_jiffies(args->timeout);

	trace_nvhost_ioctl_ctrl_syncpt_wait(args->id, args->thresh,
	  args->timeout);
	return nvhost_syncpt_wait_timeout(&ctx->dev->syncpt, args->id,
					args->thresh, timeout, &args->value);
}

static int nvhost_ioctl_ctrl_module_mutex(
	struct nvhost_ctrl_userctx *ctx,
	struct nvhost_ctrl_module_mutex_args *args)
{
	int err = 0;
	if (args->id >= ctx->dev->nb_mlocks ||
	    args->lock > 1)
		return -EINVAL;

	trace_nvhost_ioctl_ctrl_module_mutex(args->lock, args->id);
	if (args->lock && !ctx->mod_locks[args->id]) {
		if (args->id == 0)
			nvhost_module_busy(&ctx->dev->mod);
		else
			err = nvhost_mutex_try_lock(&ctx->dev->cpuaccess, args->id);
		if (!err)
			ctx->mod_locks[args->id] = 1;
	}
	else if (!args->lock && ctx->mod_locks[args->id]) {
		if (args->id == 0)
			nvhost_module_idle(&ctx->dev->mod);
		else
			nvhost_mutex_unlock(&ctx->dev->cpuaccess, args->id);
		ctx->mod_locks[args->id] = 0;
	}
	return err;
}

static int nvhost_ioctl_ctrl_module_regrdwr(
	struct nvhost_ctrl_userctx *ctx,
	struct nvhost_ctrl_module_regrdwr_args *args)
{
	u32 num_offsets = args->num_offsets;
	u32 *offsets = args->offsets;
	void *values = args->values;
	u32 vals[64];

	if (!(args->id < ctx->dev->nb_modules) ||
	    (num_offsets == 0))
		return -EINVAL;

	while (num_offsets--) {
		u32 remaining = args->block_size;
		u32 offs;
		if (get_user(offs, offsets))
			return -EFAULT;
		offsets++;
		while (remaining) {
			u32 batch = min(remaining, 64*sizeof(u32));
			if (args->write) {
				if (copy_from_user(vals, values, batch))
					return -EFAULT;
				nvhost_write_module_regs(&ctx->dev->cpuaccess,
							args->id, offs, batch, vals);
			} else {
				nvhost_read_module_regs(&ctx->dev->cpuaccess,
							args->id, offs, batch, vals);
				if (copy_to_user(values, vals, batch))
					return -EFAULT;
			}
			remaining -= batch;
			offs += batch;
			values += batch;
		}
	}

	return 0;
}

static long nvhost_ctrlctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct nvhost_ctrl_userctx *priv = filp->private_data;
	u8 buf[NVHOST_IOCTL_CTRL_MAX_ARG_SIZE];
	int err = 0;

	if ((_IOC_TYPE(cmd) != NVHOST_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVHOST_IOCTL_CTRL_LAST))
		return -EFAULT;

	BUG_ON(_IOC_SIZE(cmd) > NVHOST_IOCTL_CTRL_MAX_ARG_SIZE);

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	switch (cmd) {
	case NVHOST_IOCTL_CTRL_SYNCPT_READ:
		err = nvhost_ioctl_ctrl_syncpt_read(priv, (void *)buf);
		break;
	case NVHOST_IOCTL_CTRL_SYNCPT_INCR:
		err = nvhost_ioctl_ctrl_syncpt_incr(priv, (void *)buf);
		break;
	case NVHOST_IOCTL_CTRL_SYNCPT_WAIT:
		err = nvhost_ioctl_ctrl_syncpt_waitex(priv, (void *)buf);
		break;
	case NVHOST_IOCTL_CTRL_MODULE_MUTEX:
		err = nvhost_ioctl_ctrl_module_mutex(priv, (void *)buf);
		break;
	case NVHOST_IOCTL_CTRL_MODULE_REGRDWR:
		err = nvhost_ioctl_ctrl_module_regrdwr(priv, (void *)buf);
		break;
	case NVHOST_IOCTL_CTRL_SYNCPT_WAITEX:
		err = nvhost_ioctl_ctrl_syncpt_waitex(priv, (void *)buf);
		break;
	default:
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

static struct file_operations nvhost_ctrlops = {
	.owner = THIS_MODULE,
	.release = nvhost_ctrlrelease,
	.open = nvhost_ctrlopen,
	.unlocked_ioctl = nvhost_ctrlctl
};

static void power_host(struct nvhost_module *mod, enum nvhost_power_action action)
{
	struct nvhost_master *dev = container_of(mod, struct nvhost_master, mod);

	if (action == NVHOST_POWER_ACTION_ON) {
		nvhost_intr_start(&dev->intr, clk_get_rate(mod->clk[0].clk));
	} else if (action == NVHOST_POWER_ACTION_OFF) {
		int i;
		for (i = 0; i < dev->nb_channels; i++)
			nvhost_channel_suspend(&dev->channels[i]);
		nvhost_syncpt_save(&dev->syncpt);
		nvhost_intr_stop(&dev->intr);
	}
}

static int nvhost_user_init(struct nvhost_master *host)
{
	int i, err, devno;

	host->nvhost_class = class_create(THIS_MODULE, IFACE_NAME);
	if (IS_ERR(host->nvhost_class)) {
		err = PTR_ERR(host->nvhost_class);
		dev_err(&host->pdev->dev, "failed to create class\n");
		goto fail;
	}

	if (nvhost_major) {
		devno = MKDEV(nvhost_major, nvhost_minor);
		err = register_chrdev_region(devno, host->nb_channels + 1,
					     IFACE_NAME);
	} else {
		err = alloc_chrdev_region(&devno, nvhost_minor,
					host->nb_channels + 1, IFACE_NAME);
		nvhost_major = MAJOR(devno);
	}
	if (err < 0) {
		dev_err(&host->pdev->dev, "failed to reserve chrdev region\n");
		goto fail;
	}

	for (i = 0; i < host->nb_channels; i++) {
		struct nvhost_channel *ch = &host->channels[i];

		cdev_init(&ch->cdev, &nvhost_channelops);
		ch->cdev.owner = THIS_MODULE;

		devno = MKDEV(nvhost_major, nvhost_minor + i);
		err = cdev_add(&ch->cdev, devno, 1);
		if (err < 0) {
			dev_err(&host->pdev->dev, "failed to add chan %i cdev\n", i);
			goto fail;
		}
		ch->node = device_create(host->nvhost_class, NULL, devno, NULL,
				IFACE_NAME "-%s", ch->desc->name);
		if (IS_ERR(ch->node)) {
			err = PTR_ERR(ch->node);
			dev_err(&host->pdev->dev, "failed to create chan %i device\n", i);
			goto fail;
		}
	}

	cdev_init(&host->cdev, &nvhost_ctrlops);
	host->cdev.owner = THIS_MODULE;
	devno = MKDEV(nvhost_major, nvhost_minor + host->nb_channels);
	err = cdev_add(&host->cdev, devno, 1);
	if (err < 0)
		goto fail;
	host->ctrl = device_create(host->nvhost_class, NULL, devno, NULL,
			IFACE_NAME "-ctrl");
	if (IS_ERR(host->ctrl)) {
		err = PTR_ERR(host->ctrl);
		dev_err(&host->pdev->dev, "failed to create ctrl device\n");
		goto fail;
	}

	return 0;
fail:
	return err;
}

static void nvhost_remove_chip_support(struct nvhost_master *host)
{

	kfree(host->channels);
	host->channels = 0;

	kfree(host->syncpt.min_val);
	host->syncpt.min_val = 0;

	kfree(host->syncpt.max_val);
	host->syncpt.max_val = 0;

	kfree(host->syncpt.base_val);
	host->syncpt.base_val = 0;

	kfree(host->intr.syncpt);
	host->intr.syncpt = 0;

	kfree(host->cpuaccess.regs);
	host->cpuaccess.regs = 0;

	kfree(host->cpuaccess.reg_mem);
	host->cpuaccess.reg_mem = 0;

	kfree(host->cpuaccess.lock_counts);
	host->cpuaccess.lock_counts = 0;
}

static int nvhost_init_chip_support(struct nvhost_master *host)
{
	int err;
	switch (tegra_chip_id) {
	case TEGRA20:
		err = nvhost_init_t20_support(host);
		break;

	case TEGRA30:
		err = nvhost_init_t30_support(host);
		break;
	default:
		return -ENODEV;
	}

	if (err)
		return err;

	/* allocate items sized in chip specific support init */
	host->channels = kzalloc(sizeof(struct nvhost_channel) *
				 host->nb_channels, GFP_KERNEL);

	host->syncpt.min_val = kzalloc(sizeof(atomic_t) *
				       host->syncpt.nb_pts, GFP_KERNEL);

	host->syncpt.max_val = kzalloc(sizeof(atomic_t) *
				       host->syncpt.nb_pts, GFP_KERNEL);

	host->syncpt.base_val = kzalloc(sizeof(u32) *
					host->syncpt.nb_bases, GFP_KERNEL);

	host->intr.syncpt = kzalloc(sizeof(struct nvhost_intr_syncpt) *
				    host->syncpt.nb_pts, GFP_KERNEL);

	host->cpuaccess.reg_mem = kzalloc(sizeof(struct resource *) *
				       host->nb_modules, GFP_KERNEL);

	host->cpuaccess.regs = kzalloc(sizeof(void __iomem *) *
				       host->nb_modules, GFP_KERNEL);

	host->cpuaccess.lock_counts = kzalloc(sizeof(atomic_t) *
				       host->nb_mlocks, GFP_KERNEL);

	if (!(host->channels && host->syncpt.min_val &&
	      host->syncpt.max_val && host->syncpt.base_val &&
	      host->intr.syncpt && host->cpuaccess.reg_mem &&
	      host->cpuaccess.regs && host->cpuaccess.lock_counts)) {
		/* frees happen in the support removal phase */
		return -ENOMEM;
	}

	return 0;
}


static ssize_t enable_3d_scaling_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n", scale3d_is_enabled());

	return res;
}

static ssize_t enable_3d_scaling_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = 0;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	scale3d_enable(val);

	return count;
}

static DEVICE_ATTR(enable_3d_scaling, S_IRUGO | S_IWUSR,
	enable_3d_scaling_show, enable_3d_scaling_store);

void nvhost_remove_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_enable_3d_scaling);
}

void nvhost_create_sysfs(struct device *dev)
{
	int error = device_create_file(dev, &dev_attr_enable_3d_scaling);
	if (error)
		dev_err(dev, "failed to create sysfs attributes");
}

static int nvhost_probe(struct platform_device *pdev)
{
	struct nvhost_master *host;
	struct resource *regs, *intr0, *intr1;
	int i, err;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	intr0 = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	intr1 = platform_get_resource(pdev, IORESOURCE_IRQ, 1);

	if (!regs || !intr0 || !intr1) {
		dev_err(&pdev->dev, "missing required platform resources\n");
		return -ENXIO;
	}

	host = kzalloc(sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	host->pdev = pdev;

	host->nvmap = nvmap_create_client(nvmap_dev, "nvhost");
	if (!host->nvmap) {
		dev_err(&pdev->dev, "unable to create nvmap client\n");
		err = -EIO;
		goto fail;
	}

	host->reg_mem = request_mem_region(regs->start,
					resource_size(regs), pdev->name);
	if (!host->reg_mem) {
		dev_err(&pdev->dev, "failed to get host register memory\n");
		err = -ENXIO;
		goto fail;
	}
	host->aperture = ioremap(regs->start, resource_size(regs));
	if (!host->aperture) {
		dev_err(&pdev->dev, "failed to remap host registers\n");
		err = -ENXIO;
		goto fail;
	}

	err = nvhost_init_chip_support(host);
	if (err) {
		dev_err(&pdev->dev, "failed to init chip support\n");
		goto fail;
	}

	for (i = 0; i < host->nb_channels; i++) {
		struct nvhost_channel *ch = &host->channels[i];
		BUG_ON(!host_channel_op(host).init);
		err = host_channel_op(host).init(ch, host, i);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to init channel %d\n", i);
			goto fail;
		}
	}

	err = nvhost_cpuaccess_init(&host->cpuaccess, pdev);
	if (err)
		goto fail;

	err = nvhost_intr_init(&host->intr, intr1->start, intr0->start);
	if (err)
		goto fail;

	err = nvhost_user_init(host);
	if (err)
		goto fail;

	err = nvhost_module_init(&host->mod, "host1x", power_host, NULL, &pdev->dev);
	if (err)
		goto fail;


	platform_set_drvdata(pdev, host);

	clk_enable(host->mod.clk[0].clk);
	nvhost_syncpt_reset(&host->syncpt);
	clk_disable(host->mod.clk[0].clk);

	nvhost_bus_register(host);

	nvhost_debug_init(host);

	nvhost_create_sysfs(&pdev->dev);

	dev_info(&pdev->dev, "initialized\n");
	return 0;

fail:
	nvhost_remove_chip_support(host);
	if (host->nvmap)
		nvmap_client_put(host->nvmap);
	kfree(host);
	return err;
}

static int __exit nvhost_remove(struct platform_device *pdev)
{
	struct nvhost_master *host = platform_get_drvdata(pdev);
	nvhost_remove_chip_support(host);
	nvhost_remove_sysfs(&pdev->dev);
	return 0;
}

static int nvhost_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct nvhost_master *host = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "suspending\n");
	nvhost_module_suspend(&host->mod, true);
	clk_enable(host->mod.clk[0].clk);
	nvhost_syncpt_save(&host->syncpt);
	clk_disable(host->mod.clk[0].clk);
	dev_info(&pdev->dev, "suspended\n");
	return 0;
}

static int nvhost_resume(struct platform_device *pdev)
{
	struct nvhost_master *host = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "resuming\n");
	clk_enable(host->mod.clk[0].clk);
	nvhost_syncpt_reset(&host->syncpt);
	clk_disable(host->mod.clk[0].clk);
	scale3d_reset();
	dev_info(&pdev->dev, "resumed\n");
	return 0;
}

static struct platform_driver nvhost_driver = {
	.remove = __exit_p(nvhost_remove),
	.suspend = nvhost_suspend,
	.resume = nvhost_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME
	}
};

static int __init nvhost_mod_init(void)
{
	register_sets = tegra_gpu_register_sets();
	return platform_driver_probe(&nvhost_driver, nvhost_probe);
}

static void __exit nvhost_mod_exit(void)
{
	platform_driver_unregister(&nvhost_driver);
}

module_init(nvhost_mod_init);
module_exit(nvhost_mod_exit);

module_param_call(register_sets, NULL, param_get_uint, &register_sets, 0444);
MODULE_PARM_DESC(register_sets, "Number of register sets");

MODULE_AUTHOR("NVIDIA");
MODULE_DESCRIPTION("Graphics host driver for Tegra products");
MODULE_VERSION("1.0");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform-nvhost");
