/* Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* The NVS = NVidia Sensor framework */
/* See nvs_iio.c and nvs.h for documentation */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/nvs.h>

#define DSM_DRIVER_VERSION		(1)
#define DSM_VENDOR			"NVIDIA Corporation"
#define DSM_NAME			"nvs_dsm"

enum DSM_DBG {
	DSM_DBG_STS = 0,
	DSM_DBG_PUSH = 0xC6, /* use 0xD0 on cmd line */
	DSM_DBG_MSG_FLAGS,
	DSM_DBG_MSG_DEV_ID,
	DSM_DBG_MSG_SNSR_ID,
	DSM_DBG_UUID,
};

struct dsm_state {
	struct platform_device *pd;
	struct nvs_fn_if *nvs;
	void *nvs_st;
	struct sensor_cfg cfg;
	unsigned int connect_n;
	unsigned int disconnect_n;
	struct nvs_dsm_msg msg;
	s64 msg_ts;
	int msg_err;
	bool dbg;
};

static struct dsm_state *dsm_state_local;

int nvs_dsm_push(int dev_id, bool connect, int snsr_id, unsigned char *uuid)
{
	struct dsm_state *st = dsm_state_local;
	int ret;

	if (st == NULL)
		return -EPERM;

	st->nvs->nvs_mutex_lock(st->nvs_st);
	if (!st->dbg) {
		memset(&st->msg, 0, sizeof(st->msg));
		st->msg.ver = sizeof(st->msg);
		st->msg.dev_id = dev_id;
		st->msg.snsr_id = snsr_id;
		if (connect) {
			st->msg.flags = 1 << NVS_DSM_MSG_FLAGS_CONNECT;
			st->connect_n++;
			if (!st->connect_n)
				st->connect_n--;
		} else {
			st->disconnect_n++;
			if (!st->disconnect_n)
				st->disconnect_n--;
		}
		if (uuid)
			memcpy(&st->msg.uuid, uuid, sizeof(st->msg.uuid));
	}
	st->msg_ts = nvs_timestamp();
	ret = st->nvs->handler(st->nvs_st, &st->msg, st->msg_ts);
	if (ret > 0)
		ret = 0;
	st->msg_err = ret;
	st->nvs->nvs_mutex_unlock(st->nvs_st);
	return ret;
}

static int dsm_nvs_write(void *client, int snsr_id, unsigned int nvs)
{
	struct dsm_state *st = (struct dsm_state *)client;
	unsigned int i;

	switch (nvs & 0xFF) {
	case DSM_DBG_STS:
		return 0;

	case DSM_DBG_PUSH:
		st->dbg = true;
		nvs_dsm_push(0, false, 0, NULL);
		st->dbg = false;
		return 0;

	case DSM_DBG_MSG_FLAGS:
		st->msg.flags = (nvs >> 8) & 0xFF;
		i = (nvs >> 16) & 0xFF;
		if (i)
			st->msg.ver = i;
		else
			st->msg.ver = sizeof(st->msg);
		return 0;

	case DSM_DBG_MSG_DEV_ID:
		st->msg.dev_id = nvs >> 8;
		return 0;

	case DSM_DBG_MSG_SNSR_ID:
		st->msg.snsr_id = nvs >> 8;
		return 0;

	case DSM_DBG_UUID:
		i = (nvs >> 16) & 0xF;
		st->msg.uuid[i] = (nvs >> 8) & 0xFF;
		return 0;
	}

	return -EINVAL;
}

static int dsm_nvs_read(void *client, int snsr_id, char *buf)
{
	struct dsm_state *st = (struct dsm_state *)client;
	int i;
	ssize_t t;

	st->nvs->nvs_mutex_lock(st->nvs_st);
	t = snprintf(buf, PAGE_SIZE, "driver v.%u\n", DSM_DRIVER_VERSION);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "connections=%u\n", st->connect_n);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "disconnections=%u\n", st->disconnect_n);
	t += snprintf(buf + t, PAGE_SIZE - t, "Last DSM message:\n");
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "   msg ver/size: %u\n", st->msg.ver);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "   flags: 0x%X\n", st->msg.flags);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "   dev_id: %d\n", st->msg.dev_id);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "   snsr_id: %d\n", st->msg.snsr_id);
	t += snprintf(buf + t, PAGE_SIZE - t, "   uuid: ");
	for (i = 0; i < 16; i++)
		t += snprintf(buf + t, PAGE_SIZE - t, "%02x ",
			      st->msg.uuid[i]);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "\nmessage timestamp=%lld\n", st->msg_ts);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "message error=%d\n", st->msg_err);
	st->nvs->nvs_mutex_unlock(st->nvs_st);
	return t;
}

static struct nvs_fn_dev dsm_fn_dev = {
	.nvs_write			= dsm_nvs_write,
	.nvs_read			= dsm_nvs_read,
};

static void dsm_shutdown(struct platform_device *pd)
{
	struct dsm_state *st = (struct dsm_state *)dev_get_drvdata(&pd->dev);

	dsm_state_local = NULL;
	if (st->nvs && st->nvs_st)
		st->nvs->shutdown(st->nvs_st);
}

static int dsm_remove(struct platform_device *pd)
{
	struct dsm_state *st = (struct dsm_state *)dev_get_drvdata(&pd->dev);

	if (st != NULL) {
		dsm_shutdown(pd);
		if (st->nvs && st->nvs_st)
			st->nvs->remove(st->nvs_st);
	}
	dev_info(&st->pd->dev, "%s\n", __func__);
	return 0;
}

static struct sensor_cfg dsm_sensor_cfg = {
	.name			= "dynamic_sensor_meta",
	.ch_n			= 1,
	.ch_sz			= sizeof(struct nvs_dsm_msg),
	.part			= DSM_NAME,
	.vendor			= DSM_VENDOR,
	.version		= DSM_DRIVER_VERSION,
	.flags			= SENSOR_FLAG_SPECIAL_REPORTING_MODE,
};

static int dsm_of_dt(struct dsm_state *st)
{
	memcpy(&st->cfg, &dsm_sensor_cfg, sizeof(st->cfg));
	return nvs_of_dt(st->pd->dev.of_node, &st->cfg, NULL);
}

static int dsm_init(struct dsm_state *st)
{
	int ret;

	ret = dsm_of_dt(st);
	if (ret < 0) {
		if (ret == -ENODEV) {
			dev_info(&st->pd->dev, "%s DT disabled\n", __func__);
		} else {
			dev_err(&st->pd->dev, "%s _of_dt ERR\n", __func__);
			ret = -ENODEV;
		}
		return ret;
	}

	st->nvs = nvs_iio();
	if (st->nvs == NULL) {
		dev_err(&st->pd->dev, "%s nvs_iio ERR\n", __func__);
		ret = -ENODEV;
		return ret;
	}

	ret = st->nvs->probe(&st->nvs_st, st, &st->pd->dev,
			     &dsm_fn_dev, &st->cfg);
	if (ret) {
		dev_err(&st->pd->dev, "%s nvs_probe ERR\n", __func__);
		ret = -ENODEV;
		return ret;
	}

	return 0;
}

static int dsm_probe(struct platform_device *pd)
{
	struct dsm_state *st;
	int ret;

	dev_info(&pd->dev, "%s\n", __func__);
	st = devm_kzalloc(&pd->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		dev_err(&pd->dev, "%s devm_kzalloc ERR\n", __func__);
		return -ENOMEM;
	}

	dev_set_drvdata(&pd->dev, st);
	st->pd = pd;
	ret = dsm_init(st);
	if (ret < 0)
		dsm_remove(pd);
	else
		dsm_state_local = st;
	dev_info(&pd->dev, "%s done\n", __func__);
	return ret;
}

static const struct of_device_id dsm_of_match[] = {
	{ .compatible = "nvidia,nvs_dsm", },
	{},
};

MODULE_DEVICE_TABLE(of, dsm_of_match);

static struct platform_driver dsm_driver = {
	.probe		= dsm_probe,
	.remove		= dsm_remove,
	.shutdown	= dsm_shutdown,
	.driver = {
		.name		= DSM_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(dsm_of_match),
	},
};
module_platform_driver(dsm_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NVidia Sensor DSM driver");
MODULE_AUTHOR("NVIDIA Corporation");

