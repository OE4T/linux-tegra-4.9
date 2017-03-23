/*
 * NVGPU IOCTLs
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/file.h>

#include <nvgpu/nvgpu_common.h>

#include "gk20a/gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/ctxsw_trace_gk20a.h"
#include "ioctl_channel.h"
#include "ioctl_ctrl.h"
#include "ioctl_as.h"
#include "ioctl_tsg.h"

#define GK20A_NUM_CDEVS 7

const struct file_operations gk20a_channel_ops = {
	.owner = THIS_MODULE,
	.release = gk20a_channel_release,
	.open = gk20a_channel_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gk20a_channel_ioctl,
#endif
	.unlocked_ioctl = gk20a_channel_ioctl,
};

static const struct file_operations gk20a_ctrl_ops = {
	.owner = THIS_MODULE,
	.release = gk20a_ctrl_dev_release,
	.open = gk20a_ctrl_dev_open,
	.unlocked_ioctl = gk20a_ctrl_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gk20a_ctrl_dev_ioctl,
#endif
};

static const struct file_operations gk20a_dbg_ops = {
	.owner = THIS_MODULE,
	.release = gk20a_dbg_gpu_dev_release,
	.open = gk20a_dbg_gpu_dev_open,
	.unlocked_ioctl = gk20a_dbg_gpu_dev_ioctl,
	.poll = gk20a_dbg_gpu_dev_poll,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gk20a_dbg_gpu_dev_ioctl,
#endif
};

static const struct file_operations gk20a_as_ops = {
	.owner = THIS_MODULE,
	.release = gk20a_as_dev_release,
	.open = gk20a_as_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gk20a_as_dev_ioctl,
#endif
	.unlocked_ioctl = gk20a_as_dev_ioctl,
};

/*
 * Note: We use a different 'open' to trigger handling of the profiler session.
 * Most of the code is shared between them...  Though, at some point if the
 * code does get too tangled trying to handle each in the same path we can
 * separate them cleanly.
 */
static const struct file_operations gk20a_prof_ops = {
	.owner = THIS_MODULE,
	.release = gk20a_dbg_gpu_dev_release,
	.open = gk20a_prof_gpu_dev_open,
	.unlocked_ioctl = gk20a_dbg_gpu_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gk20a_dbg_gpu_dev_ioctl,
#endif
};

static const struct file_operations gk20a_tsg_ops = {
	.owner = THIS_MODULE,
	.release = nvgpu_ioctl_tsg_dev_release,
	.open = nvgpu_ioctl_tsg_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvgpu_ioctl_tsg_dev_ioctl,
#endif
	.unlocked_ioctl = nvgpu_ioctl_tsg_dev_ioctl,
};

static const struct file_operations gk20a_ctxsw_ops = {
	.owner = THIS_MODULE,
	.release = gk20a_ctxsw_dev_release,
	.open = gk20a_ctxsw_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gk20a_ctxsw_dev_ioctl,
#endif
	.unlocked_ioctl = gk20a_ctxsw_dev_ioctl,
	.poll = gk20a_ctxsw_dev_poll,
	.read = gk20a_ctxsw_dev_read,
	.mmap = gk20a_ctxsw_dev_mmap,
};

static const struct file_operations gk20a_sched_ops = {
	.owner = THIS_MODULE,
	.release = gk20a_sched_dev_release,
	.open = gk20a_sched_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gk20a_sched_dev_ioctl,
#endif
	.unlocked_ioctl = gk20a_sched_dev_ioctl,
	.poll = gk20a_sched_dev_poll,
	.read = gk20a_sched_dev_read,
};

static int gk20a_create_device(
	struct device *dev, int devno,
	const char *interface_name, const char *cdev_name,
	struct cdev *cdev, struct device **out,
	const struct file_operations *ops,
	struct class *class)
{
	struct device *subdev;
	int err;

	gk20a_dbg_fn("");

	cdev_init(cdev, ops);
	cdev->owner = THIS_MODULE;

	err = cdev_add(cdev, devno, 1);
	if (err) {
		dev_err(dev, "failed to add %s cdev\n", cdev_name);
		return err;
	}

	subdev = device_create(class, NULL, devno, NULL,
		interface_name, cdev_name);

	if (IS_ERR(subdev)) {
		err = PTR_ERR(dev);
		cdev_del(cdev);
		dev_err(dev, "failed to create %s device for %s\n",
			cdev_name, dev_name(dev));
		return err;
	}

	*out = subdev;
	return 0;
}

void gk20a_user_deinit(struct device *dev, struct class *class)
{
	struct gk20a *g = gk20a_from_dev(dev);

	if (g->channel.node) {
		device_destroy(class, g->channel.cdev.dev);
		cdev_del(&g->channel.cdev);
	}

	if (g->as_dev.node) {
		device_destroy(class, g->as_dev.cdev.dev);
		cdev_del(&g->as_dev.cdev);
	}

	if (g->ctrl.node) {
		device_destroy(class, g->ctrl.cdev.dev);
		cdev_del(&g->ctrl.cdev);
	}

	if (g->dbg.node) {
		device_destroy(class, g->dbg.cdev.dev);
		cdev_del(&g->dbg.cdev);
	}

	if (g->prof.node) {
		device_destroy(class, g->prof.cdev.dev);
		cdev_del(&g->prof.cdev);
	}

	if (g->tsg.node) {
		device_destroy(class, g->tsg.cdev.dev);
		cdev_del(&g->tsg.cdev);
	}

	if (g->ctxsw.node) {
		device_destroy(class, g->ctxsw.cdev.dev);
		cdev_del(&g->ctxsw.cdev);
	}

	if (g->sched.node) {
		device_destroy(class, g->sched.cdev.dev);
		cdev_del(&g->sched.cdev);
	}

	if (g->cdev_region)
		unregister_chrdev_region(g->cdev_region, GK20A_NUM_CDEVS);
}

int gk20a_user_init(struct device *dev, const char *interface_name,
		    struct class *class)
{
	int err;
	dev_t devno;
	struct gk20a *g = gk20a_from_dev(dev);

	err = alloc_chrdev_region(&devno, 0, GK20A_NUM_CDEVS, dev_name(dev));
	if (err) {
		dev_err(dev, "failed to allocate devno\n");
		goto fail;
	}
	g->cdev_region = devno;

	err = gk20a_create_device(dev, devno++, interface_name, "",
				  &g->channel.cdev, &g->channel.node,
				  &gk20a_channel_ops,
				  class);
	if (err)
		goto fail;

	err = gk20a_create_device(dev, devno++, interface_name, "-as",
				  &g->as_dev.cdev, &g->as_dev.node,
				  &gk20a_as_ops,
				  class);
	if (err)
		goto fail;

	err = gk20a_create_device(dev, devno++, interface_name, "-ctrl",
				  &g->ctrl.cdev, &g->ctrl.node,
				  &gk20a_ctrl_ops,
				  class);
	if (err)
		goto fail;

	err = gk20a_create_device(dev, devno++, interface_name, "-dbg",
				  &g->dbg.cdev, &g->dbg.node,
				  &gk20a_dbg_ops,
				  class);
	if (err)
		goto fail;

	err = gk20a_create_device(dev, devno++, interface_name, "-prof",
				  &g->prof.cdev, &g->prof.node,
				  &gk20a_prof_ops,
				  class);
	if (err)
		goto fail;

	err = gk20a_create_device(dev, devno++, interface_name, "-tsg",
				  &g->tsg.cdev, &g->tsg.node,
				  &gk20a_tsg_ops,
				  class);
	if (err)
		goto fail;

#ifdef CONFIG_GK20A_CTXSW_TRACE
	err = gk20a_create_device(dev, devno++, interface_name, "-ctxsw",
				  &g->ctxsw.cdev, &g->ctxsw.node,
				  &gk20a_ctxsw_ops,
				  class);
	if (err)
		goto fail;
#endif

	err = gk20a_create_device(dev, devno++, interface_name, "-sched",
				  &g->sched.cdev, &g->sched.node,
				  &gk20a_sched_ops,
				  class);
	if (err)
		goto fail;

	return 0;
fail:
	gk20a_user_deinit(dev, &nvgpu_class);
	return err;
}
