/*
 * drivers/i2c/busses/i2c-ivc-single.c
 *
 * Copyright (C) 2017 NVIDIA Corporation. All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include <soc/tegra/tegra-ivc-rpc.h>

#include "soc/tegra/camrtc-i2c-common.h"
#include "i2c-ivc-single.h"

/*
 * I2C IVC Single driver internal data structure
 */

#define TEGRA_I2C_SINGLE_MAX_DEV    4

#define I2C_CAMRTC_RPC_TIMEOUT_MS   250

struct tegra_i2c_ivc_dev {
	/* IVC RPC */
	const char *name;
	struct tegra_ivc_channel *chan;
	bool is_taken;
	bool is_failed;
	bool is_added;
	bool is_online;

	/* I2C device behind this IVC */
	u32 bus_id;
	u32 reg_base;
	u32 bus_clk_rate;

	/* statistics */
	struct {
		unsigned int xfer_requests;
		unsigned int total_bytes;
		unsigned int reads, read_bytes;
		unsigned int writes, write_bytes;
		unsigned int errors;
	} stat;

	/* RPC call for I2C_REQUEST_SINGLE */
	struct tegra_ivc_rpc_call_param rpc_i2c_req;
	u8 rpc_i2c_req_buf[CAMRTC_I2C_REQUEST_MAX_LEN];
	struct camrtc_rpc_i2c_response rpc_i2c_rsp;
};

static struct tegra_i2c_ivc_dev g_i2c_ivc_devs[TEGRA_I2C_SINGLE_MAX_DEV];

u32 tegra_i2c_get_clk_freq(struct device_node *np)
{
	int ret;
	u32 bus_clk_rate;

	ret = of_property_read_u32(np, "clock-frequency",
			&bus_clk_rate);
	if (ret)
		bus_clk_rate = 100000; /* default clock rate */

	return bus_clk_rate;
}
EXPORT_SYMBOL(tegra_i2c_get_clk_freq);

u32 tegra_i2c_get_reg_base(struct device_node *np)
{
	int ret;
	u32 reg_base;

	ret = of_property_read_u32_index(np, "reg", 1, &reg_base);

	BUG_ON(ret < 0);

	return reg_base;
}
EXPORT_SYMBOL(tegra_i2c_get_reg_base);

/*
 * I2C interface
 */
struct tegra_i2c_ivc_dev *tegra_ivc_i2c_get_dev(u32 reg_base)
{
	int i;
	struct tegra_i2c_ivc_dev *i2c_ivc_dev = ERR_PTR(-ENODEV);

	/* Find an IVC channel */
	for (i = 0; i < TEGRA_I2C_SINGLE_MAX_DEV; ++i) {
		if (g_i2c_ivc_devs[i].is_taken) {
			if (g_i2c_ivc_devs[i].reg_base == reg_base) {
				if (g_i2c_ivc_devs[i].is_failed)
					return ERR_PTR(-ENODEV);
				else
					break;
			}
		} else
			return ERR_PTR(-EPROBE_DEFER);
	}

	if (i == TEGRA_I2C_SINGLE_MAX_DEV)
		return ERR_PTR(-ENOMEM);

	i2c_ivc_dev = g_i2c_ivc_devs + i;

	return i2c_ivc_dev;
}
EXPORT_SYMBOL(tegra_ivc_i2c_get_dev);

static int tegra_ivc_i2c_add_single(
	struct tegra_ivc_channel *chan);

int tegra_ivc_i2c_single_xfer(struct tegra_i2c_ivc_dev *i2c_ivc_dev,
	const struct i2c_msg *reqs, int num)
{
	u8 *pbuf = NULL, *pprev_len = NULL;
	const struct i2c_msg *preq, *preq_end;
	int ret = 0, len = 0;
	u8 *read_ptr = NULL;
	int read_len = 0;

	if (i2c_ivc_dev == NULL || i2c_ivc_dev->chan == NULL)
		return -EIO;

	if (num == 0)
		return 0;

	ret = tegra_ivc_channel_runtime_get(i2c_ivc_dev->chan);
	if (ret < 0)
		return ret;

	if (tegra_ivc_rpc_channel_is_suspended(i2c_ivc_dev->chan)) {
		ret = -EBUSY;
		goto error;
	}
	if (!i2c_ivc_dev->is_added) {
		BUG_ON(!i2c_ivc_dev->is_online);
		ret = tegra_ivc_i2c_add_single(i2c_ivc_dev->chan);
		if (ret != 0) {
			dev_err(&i2c_ivc_dev->chan->dev,
				"I2C device not ready\n");
			goto error;
		}
	}

	++i2c_ivc_dev->stat.xfer_requests;

	/* First byte is bus ID */

	pbuf = i2c_ivc_dev->rpc_i2c_req_buf + 1;
	len = 1;

	preq = reqs;
	preq_end = reqs + num;

	for (;;) {
		int bytes;
		bool is_read;

		if ((preq == preq_end) || ((len + 4 + preq->len) >
			CAMRTC_I2C_REQUEST_MAX_LEN)) {
			struct camrtc_rpc_i2c_response *rpc_rsp;

			i2c_ivc_dev->rpc_i2c_req.request_len = len;
			ret = tegra_ivc_rpc_call(i2c_ivc_dev->chan,
				&i2c_ivc_dev->rpc_i2c_req);

			if (ret < 0) {
				++i2c_ivc_dev->stat.errors;
				dev_err(&i2c_ivc_dev->chan->dev,
					"I2C transaction to 0x%x failed: %d\n",
					reqs[0].addr, ret);
				ret = -EIO;
				goto error;
			}

			rpc_rsp = &i2c_ivc_dev->rpc_i2c_rsp;
			if (rpc_rsp->result) {
				ret = -EIO;
				goto error;
			}
			if (read_ptr) {
				memcpy(read_ptr, rpc_rsp->read_data, read_len);
				read_ptr = NULL;
			}

			if (preq == preq_end)
				break;

			pbuf = i2c_ivc_dev->rpc_i2c_req_buf + 1;
			len = 1;
		}

		is_read = (preq->flags & I2C_M_RD);

		if (!is_read) {
			pbuf[0] = 0;
			++i2c_ivc_dev->stat.writes;
			i2c_ivc_dev->stat.write_bytes += preq->len;
		} else {
			read_ptr = preq->buf;
			read_len = preq->len;
			pbuf[0] = CAMRTC_I2C_REQUEST_FLAG_READ;
			++i2c_ivc_dev->stat.reads;
			i2c_ivc_dev->stat.read_bytes += preq->len;
		}

		if ((preq->flags & I2C_M_NOSTART) == 0) {
			pbuf[1] = preq->addr & 0xff;
			if (preq->flags & I2C_M_TEN) {
				pbuf[0] |= CAMRTC_I2C_REQUEST_FLAG_TEN;
				pbuf[2] = (preq->addr >> 8) & 0xff;
				bytes = 2;
			} else {
				pbuf[2] = 0;
				bytes = 1;
			}
		} else {
			pbuf[0] |= CAMRTC_I2C_REQUEST_FLAG_NOSTART;
			/* slave address is don't care */
			pbuf[1] = 0;
			pbuf[2] = 0;
			bytes = 0;
		}

		pbuf[3] = preq->len;
		pprev_len = pbuf + 3;
		bytes += preq->len;

		pbuf += 4;
		len += 4;

		if (!is_read) {
			u8 *psrc = preq->buf;

			switch (preq->len) {
			case 4:
				*pbuf++ = *psrc++;
			case 3:
				*pbuf++ = *psrc++;
			case 2:
				*pbuf++ = *psrc++;
			case 1:
				*pbuf++ = *psrc++;
			case 0:
				break;
			default:
				memcpy(pbuf, psrc, preq->len);
				pbuf += preq->len;
				break;
			}

			len += preq->len;
		}
		++preq;

		/* Merge requests with NOSTART */
		while (preq != preq_end && (preq->flags & I2C_M_NOSTART)) {
			u8 *psrc = preq->buf;

			if ((len + preq->len) >
				CAMRTC_I2C_REQUEST_MAX_LEN)
				continue;

			switch (preq->len) {
			case 4:
				*pbuf++ = *psrc++;
			case 3:
				*pbuf++ = *psrc++;
			case 2:
				*pbuf++ = *psrc++;
			case 1:
				*pbuf++ = *psrc++;
			case 0:
				break;
			default:
				memcpy(pbuf, psrc, preq->len);
				pbuf += preq->len;
				break;
			}

			*pprev_len += preq->len;
			len += preq->len;
			bytes += preq->len;

			if (!is_read)
				i2c_ivc_dev->stat.write_bytes += preq->len;
			else
				i2c_ivc_dev->stat.read_bytes += preq->len;

			++preq;
		}

		i2c_ivc_dev->stat.total_bytes += bytes;
	}

error:
	tegra_ivc_channel_runtime_put(i2c_ivc_dev->chan);
	return (ret < 0) ? -EIO : num;
}
EXPORT_SYMBOL(tegra_ivc_i2c_single_xfer);

/*
 * IVC channel Debugfs
 */

#define DEFINE_SEQ_FOPS(_fops_, _show_) \
	static int _fops_ ## _open(struct inode *inode, struct file *file) \
	{ \
		return single_open(file, _show_, inode->i_private); \
	} \
	static const struct file_operations _fops_ = { \
		.open = _fops_ ## _open, \
		.read = seq_read, \
		.llseek = seq_lseek, \
		.release = single_release }

static int tegra_ivc_i2c_single_stat_show(
	struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *chan = file->private;
	struct tegra_i2c_ivc_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);

	seq_printf(file, "Xfer requests: %u\n",
		i2c_ivc_dev->stat.xfer_requests);
	seq_printf(file, "Total bytes: %u\n", i2c_ivc_dev->stat.total_bytes);
	seq_printf(file, "Read requests: %u\n", i2c_ivc_dev->stat.reads);
	seq_printf(file, "Read bytes: %u\n", i2c_ivc_dev->stat.read_bytes);
	seq_printf(file, "Write requests: %u\n", i2c_ivc_dev->stat.writes);
	seq_printf(file, "Write bytes: %u\n", i2c_ivc_dev->stat.write_bytes);
	seq_printf(file, "Errors: %u\n", i2c_ivc_dev->stat.errors);

	return 0;
}

DEFINE_SEQ_FOPS(tegra_ivc_i2c_debugfs_stats,
	tegra_ivc_i2c_single_stat_show);

static void tegra_ivc_i2c_single_create_debugfs(
	struct tegra_ivc_channel *chan,
	struct dentry *debugfs_root)
{
	debugfs_create_file("stats", S_IRUGO,
		debugfs_root, chan,
		&tegra_ivc_i2c_debugfs_stats);
}

/*
 * IVC channel driver interface
 */

static int tegra_ivc_i2c_add_single(
	struct tegra_ivc_channel *chan)
{
	struct tegra_i2c_ivc_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);
	struct camrtc_rpc_i2c_add_single rpc_add_single;
	u32 bus_id;
	int ret;

	if (i2c_ivc_dev->reg_base == 0) {
		dev_err(&chan->dev,
			"Invalid I2C device at 0x%08x\n",
			i2c_ivc_dev->reg_base);
		return -ENODEV;
	}

	rpc_add_single.reg_base = i2c_ivc_dev->reg_base;
	rpc_add_single.bus_clk_rate = i2c_ivc_dev->bus_clk_rate;

	/* Register an I2C device to CamRTC */
	ret = tegra_ivc_rpc_call_pl(chan,
		CAMRTC_RPC_REQ_I2C_ADD_SINGLE_DEV,
		sizeof(rpc_add_single), &rpc_add_single,
		TEGRA_IVC_RPC_RSP_RET_CODE,
		sizeof(bus_id), &bus_id,
		NULL, NULL, 0);
	if (ret < 0) {
		dev_err(&chan->dev,
			"Failed to register an I2C device at 0x%08x: %d\n",
			i2c_ivc_dev->reg_base, ret);
		ret = -EIO;
		goto fail_remove_chan;
	}

	dev_info(&chan->dev,
		"Registered an I2C device at 0x%08x to bus %d\n",
		i2c_ivc_dev->reg_base, bus_id);

	i2c_ivc_dev->bus_id = bus_id;
	i2c_ivc_dev->rpc_i2c_req_buf[0] = bus_id;
	i2c_ivc_dev->rpc_i2c_req.request_id =
		CAMRTC_RPC_REQ_I2C_REQUEST_SINGLE;
	i2c_ivc_dev->rpc_i2c_req.request = i2c_ivc_dev->rpc_i2c_req_buf;
	i2c_ivc_dev->rpc_i2c_req.response_id = CAMRTC_RPC_RSP_I2C_RESPONSE;
	i2c_ivc_dev->rpc_i2c_req.response_len =
		sizeof(i2c_ivc_dev->rpc_i2c_rsp);
	i2c_ivc_dev->rpc_i2c_req.response = &i2c_ivc_dev->rpc_i2c_rsp;
	i2c_ivc_dev->rpc_i2c_req.callback = NULL;
	i2c_ivc_dev->rpc_i2c_req.callback_param = NULL;
	i2c_ivc_dev->rpc_i2c_req.timeout_ms = I2C_CAMRTC_RPC_TIMEOUT_MS;
	i2c_ivc_dev->is_added = true;

	return 0;

fail_remove_chan:
	tegra_ivc_rpc_channel_remove(chan);
	i2c_ivc_dev->is_failed = true;
	return ret;
}

static void tegra_ivc_i2c_single_ready(
	struct tegra_ivc_channel *chan, bool online)
{
	struct tegra_i2c_ivc_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);

	i2c_ivc_dev->is_online = online;

	if (!online)
		i2c_ivc_dev->is_added = false;
}

static struct tegra_ivc_rpc_ops tegra_ivc_rpc_user_ops = {
	.create_debugfs = tegra_ivc_i2c_single_create_debugfs,
	.ready = tegra_ivc_i2c_single_ready,
};

/* Platform device */
static int tegra_ivc_rpc_i2c_single_probe(struct tegra_ivc_channel *chan)
{
	int ret;
	int i;
	struct tegra_i2c_ivc_dev *i2c_ivc_dev;
	struct device_node *i2c_node;

	/* Find an empty slot */
	for (i = 0; i < TEGRA_I2C_SINGLE_MAX_DEV; ++i) {
		if (!g_i2c_ivc_devs[i].is_taken)
			break;
	}

	if (i == TEGRA_I2C_SINGLE_MAX_DEV)
		return -ENOMEM;

	i2c_ivc_dev = g_i2c_ivc_devs + i;

	i2c_node = of_parse_phandle(chan->dev.of_node, "device", 0);
	if (i2c_node == NULL) {
		dev_err(&chan->dev, "Cannot get i2c device node");
		return -ENODEV;
	}

	/* Read properties */
	ret = of_property_read_string(chan->dev.of_node, "nvidia,service",
		&i2c_ivc_dev->name);
	if (ret) {
		dev_err(&chan->dev,
		"Cannot read property nvidia,service: %d\n", ret);
		return ret;
	}

	/* Register IVC/RPC channel */
	ret = tegra_ivc_rpc_channel_probe(chan, &tegra_ivc_rpc_user_ops);
	if (ret < 0) {
		dev_err(&chan->dev, "Cannot start IVC/RPC interface");
		goto fail_free_ivc_dev;
	}

	i2c_ivc_dev->is_added = false;
	i2c_ivc_dev->is_failed = false;
	i2c_ivc_dev->is_online = false;
	i2c_ivc_dev->reg_base = tegra_i2c_get_reg_base(i2c_node);
	i2c_ivc_dev->bus_clk_rate = tegra_i2c_get_clk_freq(i2c_node);
	i2c_ivc_dev->chan = chan;
	tegra_ivc_channel_set_drvdata(chan, i2c_ivc_dev);
	i2c_ivc_dev->is_taken = true;

	return 0;

fail_free_ivc_dev:
	devm_kfree(&chan->dev, i2c_ivc_dev);
	return ret;
}

static void tegra_ivc_rpc_i2c_single_remove(struct tegra_ivc_channel *chan)
{
	tegra_ivc_rpc_channel_remove(chan);
}

static const struct of_device_id tegra_ivc_rpc_i2c_single_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-rpc-i2c-single", },
	{ },
};
TEGRA_IVC_RPC_DRIVER_DEFINE(i2c_single, "tegra-ivc-rpc-i2c-single")

MODULE_AUTHOR("Ashish Singh <assingh@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra CAMRTC I2C IVC driver");
MODULE_LICENSE("GPL");
