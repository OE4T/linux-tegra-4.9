/*
 * Copyright (c) 2016 NVIDIA CORPORATION. All rights reserved.
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
 * You should have eeceived a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/tegra-camera-rtcpu.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>

#include "camrtc-dbg-messages.h"

struct camrtc_debug {
	struct mutex mutex;
	struct dentry *root;
	wait_queue_head_t waitq;
	struct {
		u32 completion_timeout;
		u32 mods_loops;
	} parameters;
};

/* Get a camera-rtcpu device */
static struct device *camrtc_get_device(struct tegra_ivc_channel *ch)
{
	if (unlikely(ch == NULL))
		return NULL;

	BUG_ON(ch->dev.parent == NULL);
	BUG_ON(ch->dev.parent->parent == NULL);

	return ch->dev.parent->parent;
}

#define INIT_OPEN_FOPS(_open) { \
	.open = _open, \
	.read = seq_read, \
	.llseek = seq_lseek, \
	.release = single_release \
}

#define DEFINE_SEQ_FOPS(_fops_, _show_) \
static int _fops_ ## _open(struct inode *inode, struct file *file) \
{ \
	return single_open(file, _show_, inode->i_private); \
} \
static const struct file_operations _fops_ = INIT_OPEN_FOPS(_fops_ ## _open)

static int camrtc_show_reboot(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct device *rce_dev = camrtc_get_device(ch);

	tegra_camrtc_set_halt(rce_dev, true);

	tegra_camrtc_reset(rce_dev);

	tegra_camrtc_set_halt(rce_dev, false);

	seq_puts(file, "0\n");

	return 0;
}

DEFINE_SEQ_FOPS(fops_reboot, camrtc_show_reboot);

static int show_halt(void *data, u64 *val)
{
	struct tegra_ivc_channel *ch = data;
	struct device *rce_dev = camrtc_get_device(ch);
	int ret;
	bool value;

	ret = tegra_camrtc_get_halt(rce_dev, &value);

	if (likely(ret == 0))
		*val = value;

	return ret;
}

static int store_halt(void *data, u64 val)
{
	struct tegra_ivc_channel *ch = data;
	struct device *rce_dev = camrtc_get_device(ch);

	return tegra_camrtc_set_halt(rce_dev, val != 0);
}

DEFINE_SIMPLE_ATTRIBUTE(fops_halt, show_halt, store_halt, "%lld\n");

static int camrtc_show_reset(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = data;
	struct device *rce_dev = camrtc_get_device(ch);

	tegra_camrtc_reset(rce_dev);

	seq_puts(file, "0\n");

	return 0;
}

DEFINE_SEQ_FOPS(fops_reset, camrtc_show_reset);

static void camrtc_debug_notify(struct tegra_ivc_channel *ch)
{
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);

	wake_up_all(&crd->waitq);
}

static int camrtc_ivc_dbg_xact(
	struct tegra_ivc_channel *ch,
	struct camrtc_dbg_request *req,
	struct camrtc_dbg_response *resp,
	long timeout)
{
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	int ret;

	if (timeout == 0)
		timeout = crd->parameters.completion_timeout;

	ret = mutex_lock_interruptible(&crd->mutex);
	if (ret)
		return ret;

	while (tegra_ivc_can_read(&ch->ivc)) {
		tegra_ivc_read_advance(&ch->ivc);
		dev_warn(&ch->dev, "stray response\n");
	}

	timeout = wait_event_interruptible_timeout(crd->waitq,
				tegra_ivc_can_write(&ch->ivc), timeout);
	if (timeout <= 0) {
		ret = timeout ?: -ETIMEDOUT;
		goto out;
	}

	ret = tegra_ivc_write(&ch->ivc, req, sizeof(*req));
	if (ret < 0) {
		dev_err(&ch->dev, "IVC write error: %d\n", ret);
		goto out;
	}

	for (;;) {
		timeout = wait_event_interruptible_timeout(crd->waitq,
			tegra_ivc_can_read(&ch->ivc), timeout);
		if (timeout <= 0) {
			ret = timeout ?: -ETIMEDOUT;
			goto out;
		}

		dev_dbg(&ch->dev, "rx msg\n");

		ret = tegra_ivc_read_peek(&ch->ivc, resp, 0, sizeof (*resp));
		if (ret < 0) {
			dev_err(&ch->dev, "IVC read error: %d\n", ret);
			goto out;
		}

		tegra_ivc_read_advance(&ch->ivc);

		if (resp->resp_type == req->req_type) {
			ret = 0;
			break;
		}

		dev_err(&ch->dev, "unexpected response\n");
	}
out:
	mutex_unlock(&crd->mutex);
	return ret;
}

static int camrtc_show_ping(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_PING,
	};
	struct camrtc_dbg_response resp;
	union ktime sent, recv;
	int ret = 0;

	sent = ktime_get_raw();
	req.data.ping_data.ts_req = sent.tv64;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, 0);
	if (ret == 0) {
		recv = ktime_get_raw();
		seq_printf(file, "roundtrip=%llu\n", recv.tv64 - sent.tv64);
	}

	return ret;
}

DEFINE_SEQ_FOPS(fops_ping, camrtc_show_ping);

static int camrtc_show_mods_result(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_MODS_TEST,
	};
	struct camrtc_dbg_response resp;
	int ret = 0;
	unsigned long timeout = crd->parameters.completion_timeout;
	u32 loops = crd->parameters.mods_loops;

	req.data.mods_data.mods_loops = loops;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, loops * timeout);
	if (ret == 0)
		seq_printf(file, "mods=%u\n", resp.status);

	return ret;
}

DEFINE_SEQ_FOPS(fops_mods_result, camrtc_show_mods_result);

struct camrtc_dbgfs_node {
	char *name;
	mode_t mode;
	const struct file_operations *fops;
};

static const struct camrtc_dbgfs_node rce_nodes[] = {
	{
		.name = "reboot",
		.mode = S_IRUGO,
		.fops = &fops_reboot,
	},
	{
		.name = "reset",
		.mode = S_IRUGO,
		.fops = &fops_reset,
	},
	{
		.name = "halt",
		.mode = S_IRUGO | S_IWUSR,
		.fops = &fops_halt,
	},
	{
		.name = "ping",
		.mode = S_IRUGO,
		.fops = &fops_ping,
	},
	{
		.name = "mods",
	},
	{
		.name = "result",
		.mode = S_IRUGO,
		.fops = &fops_mods_result,
	},
};

static int camrtc_debug_populate(struct tegra_ivc_channel *ch)
{
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	struct dentry *dir;
	int i;

	crd->root = dir = debugfs_create_dir("camrtc", NULL);
	if (dir == NULL)
		return -ENOMEM;

	if (!debugfs_create_u32("timeout", S_IRUGO|S_IWUSR, dir,
				&crd->parameters.completion_timeout))
		goto error;

	for (i = 0; i < ARRAY_SIZE(rce_nodes); i++) {
		const struct camrtc_dbgfs_node *n = &rce_nodes[i];
		struct dentry *d;

		if (n->fops == NULL)
			d = dir = debugfs_create_dir(n->name, crd->root);
		else
			d = debugfs_create_file(n->name,
				n->mode, dir, ch, n->fops);

		if (IS_ERR_OR_NULL(d))
			return PTR_ERR(d);
	}

	if (!debugfs_create_u32("loops", S_IRUGO|S_IWUSR, dir,
				&crd->parameters.mods_loops))
		goto error;

	return 0;
error:
	debugfs_remove_recursive(crd->root);
	return -ENOMEM;
}

static int camrtc_debug_probe(struct tegra_ivc_channel *ch)
{
	struct device *dev = &ch->dev;
	struct camrtc_debug *crd;

	dev_info(dev, "probing");

	BUG_ON(ch->ivc.frame_size < sizeof(struct camrtc_dbg_request));
	BUG_ON(ch->ivc.frame_size < sizeof(struct camrtc_dbg_response));

	crd = devm_kzalloc(dev, sizeof(*crd), GFP_KERNEL);
	if (unlikely(crd == NULL))
		return -ENOMEM;

	crd->parameters.completion_timeout = 50;
	crd->parameters.mods_loops = 20;

	mutex_init(&crd->mutex);
	init_waitqueue_head(&crd->waitq);

	tegra_ivc_channel_set_drvdata(ch, crd);

	if (camrtc_debug_populate(ch))
		return -ENOMEM;

	dev_info(dev, "probed OK");

	return 0;
}

static void camrtc_debug_remove(struct tegra_ivc_channel *ch)
{
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);

	debugfs_remove_recursive(crd->root);
}

static const struct tegra_ivc_channel_ops tegra_ivc_channel_debug_ops = {
	.probe	= camrtc_debug_probe,
	.remove	= camrtc_debug_remove,
	.notify	= camrtc_debug_notify,
};

static const struct of_device_id camrtc_debug_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-debug" },
	{ },
};

static struct tegra_ivc_driver camrtc_debug_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.bus	= &tegra_ivc_bus_type,
		.name	= "tegra-camera-rtcpu-debugfs",
		.of_match_table = camrtc_debug_of_match,
	},
	.dev_type	= &tegra_ivc_channel_type,
	.ops.channel	= &tegra_ivc_channel_debug_ops,
};
tegra_ivc_module_driver(camrtc_debug_driver);

MODULE_DESCRIPTION("Debug Driver for Camera RTCPU");
MODULE_AUTHOR("Pekka Pessi <ppessi@nvidia.com>");
MODULE_LICENSE("GPL v2");
