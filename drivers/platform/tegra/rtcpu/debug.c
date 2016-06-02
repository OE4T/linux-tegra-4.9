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

#include "camrtc-dbg-messages.h"

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/tegra_ast.h>
#include <linux/tegra-camera-rtcpu.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>

struct camrtc_debug {
	struct mutex mutex;
	struct dentry *root;
	wait_queue_head_t waitq;
	struct {
		u32 completion_timeout;
		u32 mods_loops;
	} parameters;
};

#define NV(x) "nvidia," #x

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

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_reboot, camrtc_show_reboot);

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

DEFINE_SIMPLE_ATTRIBUTE(camrtc_dbgfs_fops_halt,
			show_halt, store_halt, "%lld\n");

static int camrtc_show_reset(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = data;
	struct device *rce_dev = camrtc_get_device(ch);

	tegra_camrtc_reset(rce_dev);

	seq_puts(file, "0\n");

	return 0;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_reset, camrtc_show_reset);

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
	u64 sent, recv, tsc;
	int ret = 0;

	sent = sched_clock();
	req.data.ping_data.ts_req = sent;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, 0);
	if (ret)
		return ret;

	recv = sched_clock();
	tsc = resp.data.ping_data.ts_resp;
	seq_printf(file,
		"roundtrip=%llu.%03llu us "
		"(sent=%llu.%09llu recv=%llu.%09llu)\n",
		(recv - sent) / 1000, (recv - sent) % 1000,
		sent / 1000000000, sent % 1000000000,
		recv / 1000000000, recv % 1000000000);
	seq_printf(file,
		"rtcpu tsc=%llu.%09llu offset=%llu.%09llu\n",
		tsc / (1000000000 / 32), tsc % (1000000000 / 32),
		(tsc * 32ULL - sent) / 1000000000,
		(tsc * 32ULL - sent) % 1000000000);

	return 0;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_ping, camrtc_show_ping);

static int camrtc_show_mods_result(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_MODS_TEST,
	};
	struct camrtc_dbg_response resp;
	int ret;
	unsigned long timeout = crd->parameters.completion_timeout;
	u32 loops = crd->parameters.mods_loops;

	req.data.mods_data.mods_loops = loops;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, loops * timeout);
	if (ret == 0)
		seq_printf(file, "mods=%u\n", resp.status);

	return ret;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_mods_result, camrtc_show_mods_result);

static int camrtc_dbgfs_show_freertos_state(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_RTOS_STATE,
	};
	struct camrtc_dbg_response resp;
	int ret = 0;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, 0);
	if (ret == 0) {
		seq_printf(file, "%.*s",
			(int) sizeof(resp.data.rtos_state_data.rtos_state),
			resp.data.rtos_state_data.rtos_state);
	}

	return ret;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_freertos_state,
		camrtc_dbgfs_show_freertos_state);

static void camrtc_dbgfs_show_ast_region(struct seq_file *file,
				void __iomem *ast[],
				int ast_dma,
				u32 index)
{
	void __iomem *base = ast[ast_dma];
	struct tegra_ast_region_info info;

	tegra_ast_get_region_info(base, index, &info);

	seq_printf(file, "ast%u region %u %s\n", ast_dma, index,
		info.enabled ? "enabled" : "disabled");

	if (!info.enabled)
		return;

	seq_printf(file,
		"\tslave=0x%llx\n"
		"\tmaster=0x%llx\n"
		"\tsize=0x%llx\n"
		"\tlock=%u snoop=%u non_secure=%u ns_passthru=%u\n"
		"\tcarveout_id=%u carveout_al=%u\n"
		"\tvpr_rd=%u vpr_wr=%u vpr_passthru=%u\n"
		"\tvm_index=%u physical=%u\n"
		"\tstream_id=%u enabled=%u\n",
		info.slave, info.master, info.mask + 1,
		info.lock, info.snoop,
		info.non_secure, info.ns_passthru,
		info.carveout_id, info.carveout_al,
		info.vpr_rd, info.vpr_wr, info.vpr_passthru,
		info.vm_index, info.physical,
		info.stream_id, info.stream_id_enabled);
}

struct camrtc_dbgfs_ast_node
{
	struct tegra_ivc_channel *ch;
	uint8_t dma;
	uint8_t mask;
};

static int camrtc_dbgfs_show_ast(struct seq_file *file,
				void *data)
{
	struct camrtc_dbgfs_ast_node *node = file->private;
	struct tegra_ivc_channel *ch = node->ch;
	struct device *rce_dev = camrtc_get_device(ch);
	void __iomem *ast[2];
	int i, ret;

	BUG_ON(node->dma >= ARRAY_SIZE(ast));

	ret = tegra_ast_map(rce_dev, NV(ast), ARRAY_SIZE(ast), ast);
	if (ret) {
		dev_err(&ch->dev, "RTCPU ASTs not found: %d\n", ret);
		return ret;
	}

	for (i = 0; i <= 7; i++) {
		if (!(node->mask & BIT(i)))
			continue;

		camrtc_dbgfs_show_ast_region(file, ast, node->dma, i);

		if (node->mask & (node->mask - 1)) /* are multiple bits set? */
			seq_puts(file, "\n");
	}

	tegra_ast_unmap(rce_dev, ARRAY_SIZE(ast), ast);

	return 0;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_ast, camrtc_dbgfs_show_ast);

static int camrtc_debug_populate(struct tegra_ivc_channel *ch)
{
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	struct dentry *dir;
	struct camrtc_dbgfs_ast_node *ast_nodes;
	unsigned dma, region;

	crd->root = dir = debugfs_create_dir("camrtc", NULL);
	if (dir == NULL)
		return -ENOMEM;

	if (!debugfs_create_file("reboot", S_IRUGO, dir, ch,
			&camrtc_dbgfs_fops_reboot))
		goto error;
	if (!debugfs_create_file("reset", S_IRUGO, dir, ch,
			&camrtc_dbgfs_fops_reset))
		goto error;
	if (!debugfs_create_file("halt", S_IRUGO | S_IWUSR, dir, ch,
			&camrtc_dbgfs_fops_halt))
		goto error;
	if (!debugfs_create_file("ping", S_IRUGO, dir, ch,
			&camrtc_dbgfs_fops_ping))
		goto error;
	if (!debugfs_create_u32("timeout", S_IRUGO | S_IWUSR, dir,
			&crd->parameters.completion_timeout))
		goto error;

	dir = debugfs_create_dir("mods", crd->root);
	if (!dir)
		goto error;
	if (!debugfs_create_u32("loops", S_IRUGO | S_IWUSR, dir,
			&crd->parameters.mods_loops))
		goto error;
	if (!debugfs_create_file("result", S_IRUGO, dir, ch,
			&camrtc_dbgfs_fops_mods_result))
		goto error;

	dir = debugfs_create_dir("rtos", crd->root);
	if (!dir)
		goto error;
	if (!debugfs_create_file("state", S_IRUGO, dir, ch,
			&camrtc_dbgfs_fops_freertos_state))
		goto error;

	ast_nodes = devm_kzalloc(&ch->dev, 18 * sizeof(*ast_nodes),
					GFP_KERNEL);
	if (unlikely(ast_nodes == NULL))
		goto error;

	for (dma = 0; dma <= 1; dma++) {
		dir = debugfs_create_dir(dma ? "ast1" : "ast0", crd->root);
		if (dir == NULL)
			goto error;

		ast_nodes->ch = ch;
		ast_nodes->dma = dma;
		ast_nodes->mask = 0xff;

		if (!debugfs_create_file("all", S_IRUGO, dir, ast_nodes,
						&camrtc_dbgfs_fops_ast))
			goto error;

		ast_nodes++;

		for (region = 0; region < 8; region++) {
			char name[8];

			snprintf(name, sizeof name, "%u", region);

			ast_nodes->ch = ch;
			ast_nodes->dma = dma;
			ast_nodes->mask = BIT(region);

			if (!debugfs_create_file(name, S_IRUGO, dir, ast_nodes,
						&camrtc_dbgfs_fops_ast))
				goto error;

			ast_nodes++;
		}
	}

	return 0;
error:
	debugfs_remove_recursive(crd->root);
	return -ENOMEM;
}

static int camrtc_debug_probe(struct tegra_ivc_channel *ch)
{
	struct device *dev = &ch->dev;
	struct camrtc_debug *crd;

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
