/* Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
/* See nvs_light.c and nvs_light.h for documentation */


#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/nvs.h>
#include <linux/nvs_light.h>

#define JSA_VENDOR			"SolteamOpto"
#define JSA_NAME			"jsa1127"
#define JSA_LIGHT_VERSION		(1)
#define JSA_LIGHT_MAX_RANGE_IVAL	(109000)
#define JSA_LIGHT_MAX_RANGE_MICRO	(0)
#define JSA_LIGHT_RESOLUTION_IVAL	(0)
#define JSA_LIGHT_RESOLUTION_MICRO	(210000)
#define JSA_LIGHT_MILLIAMP_IVAL		(0)
#define JSA_LIGHT_MILLIAMP_MICRO	(90000)
#define JSA_LIGHT_SCALE_IVAL		(0)
#define JSA_LIGHT_SCALE_MICRO		(10000)
#define JSA_LIGHT_OFFSET_IVAL		(0)
#define JSA_LIGHT_OFFSET_MICRO		(0)
#define JSA_LIGHT_THRESHOLD_LO		(100)
#define JSA_LIGHT_THRESHOLD_HI		(100)
#define JSA_POLL_DLY_MS_MIN		(800)
#define JSA_POLL_DLY_MS_MAX		(4000)
/* HW registers */
#define JSA_CMD_SHUTDOWN		(0x80)
#define JSA_CMD_EN_CONT			(0x0C)
#define JSA_CMD_EN			(0x04)
#define JSA_CMD_START			(0x08)
#define JSA_CMD_STOP			(0x30)
#define JSA_CMD_MASK			(0x3F)
#define JSA_VAL_VALID			(15)
#define JSA_HW_DELAY_MS			(60)


/* regulator names in order of powering on */
static char *jsa_vregs[] = {
	"vdd",
};

static unsigned short jsa_i2c_addrs[] = {
	0x29,
	0x39,
	0x44,
};

static struct nvs_light_dynamic jsa_nld_tbl[] = {
	{{0, 210000}, {6500,   0}, {0, 90000}, 800, 0},
	{{0, 420000}, {13000,  0}, {0, 90000}, 400, 0},
	{{0, 560000}, {18000,  0}, {0, 90000}, 300, 0},
	{{0, 830000}, {27000,  0}, {0, 90000}, 200, 0},
	{{1, 670000}, {54000,  0}, {0, 90000}, 100, 0},
	{{3, 330000}, {109000, 0}, {0, 90000}, 50,  0}
};

struct jsa_state {
	struct i2c_client *i2c;
	struct nvs_fn_if *nvs;
	void *nvs_data;
	struct sensor_cfg cfg;
	struct delayed_work dw;
	struct regulator_bulk_data vreg[ARRAY_SIZE(jsa_vregs)];
	struct nvs_light light;
	unsigned int sts;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int enabled;		/* enable status */
	bool iio_ts_en;			/* use IIO timestamps */
	bool hw_it;			/* HW defined integration time */
	u16 i2c_addr;			/* I2C address */
	u8 rc_cmd;			/* store for register dump */
};


static s64 jsa_get_time_ns(struct jsa_state *st)
{
	struct timespec ts;

	if (st->iio_ts_en)
		return iio_get_time_ns();

	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

static void jsa_err(struct jsa_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

static int jsa_i2c_rd(struct jsa_state *st, u16 *val)
{
	struct i2c_msg msg;
	int ret = 0;

	msg.addr = st->i2c_addr;
	msg.flags = I2C_M_RD;
	msg.len = 2;
	msg.buf = (__u8 *)val;
	if (i2c_transfer(st->i2c->adapter, &msg, 1) == 1) {
		*val = le16_to_cpup(val);
	} else {
		jsa_err(st);
		ret = -EIO;
	}
	return ret;
}

static int jsa_i2c_wr(struct jsa_state *st, u8 val)
{
	struct i2c_msg msg;
	u8 buf[1];
	int ret = 0;

	if (st->i2c_addr) {
		buf[0] = val;
		msg.addr = st->i2c_addr;
		msg.flags = 0;
		msg.len = sizeof(buf);
		msg.buf = buf;
		if (i2c_transfer(st->i2c->adapter, &msg, 1) == 1) {
			st->rc_cmd = val;
		} else {
			jsa_err(st);
			ret = -EIO;
		}
		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s=%hhx err=%d\n",
				 __func__, val, ret);
	}
	return ret;
}

static int jsa_pm(struct jsa_state *st, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
				       ARRAY_SIZE(jsa_vregs));
		if (ret)
			mdelay(JSA_HW_DELAY_MS);
	} else {
		ret = nvs_vregs_sts(st->vreg, ARRAY_SIZE(jsa_vregs));
		if ((ret < 0) || (ret == ARRAY_SIZE(jsa_vregs))) {
			ret = jsa_i2c_wr(st, JSA_CMD_SHUTDOWN);
		} else if (ret > 0) {
			ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
					       ARRAY_SIZE(jsa_vregs));
			mdelay(JSA_HW_DELAY_MS);
			ret = jsa_i2c_wr(st, JSA_CMD_SHUTDOWN);
		}
		ret |= nvs_vregs_disable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(jsa_vregs));
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		dev_err(&st->i2c->dev, "%s pwr=%x ERR=%d\n",
			__func__, enable, ret);
	} else {
		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s pwr=%x\n",
				 __func__, enable);
	}
	return ret;
}

static void jsa_pm_exit(struct jsa_state *st)
{
	jsa_pm(st, false);
	nvs_vregs_exit(&st->i2c->dev, st->vreg, ARRAY_SIZE(jsa_vregs));
}

static int jsa_pm_init(struct jsa_state *st)
{
	int ret;

	st->enabled = 0;
	nvs_vregs_init(&st->i2c->dev,
		       st->vreg, ARRAY_SIZE(jsa_vregs), jsa_vregs);
	ret = jsa_pm(st, true);
	return ret;
}

static int jsa_start(struct jsa_state *st)
{
	int ret;

	if (st->hw_it) {
		ret = jsa_i2c_wr(st, JSA_CMD_EN_CONT);
	} else {
		ret = jsa_i2c_wr(st, JSA_CMD_EN);
		ret |= jsa_i2c_wr(st, JSA_CMD_START);
	}
	return ret;
}

static unsigned int jsa_sleep(struct jsa_state *st)
{
	unsigned int ms;

	ms = st->light.delay_us * 1000;
	if (ms > st->light.poll_delay_ms) {
		jsa_i2c_wr(st, JSA_CMD_SHUTDOWN);
		ms -= st->light.poll_delay_ms;
	} else {
		jsa_start(st);
		ms = st->light.poll_delay_ms;
	}
	return ms;
}

static int jsa_rd(struct jsa_state *st)
{
	s64 ts;
	u16 hw;
	int ret;

	ret = jsa_i2c_rd(st, &hw);
	if (ret)
		return ret;

	if (!(hw & (1 << JSA_VAL_VALID)))
		/* data not ready */
		return -EINVAL;

	hw &= ~(1 << JSA_VAL_VALID);
	ts = jsa_get_time_ns(st);
	if (st->sts & NVS_STS_SPEW_DATA)
		dev_info(&st->i2c->dev,
			 "poll light hw %hu %lld  diff=%d %lldns  index=%u\n",
			 hw, ts, hw - st->light.hw, ts - st->light.timestamp,
			 st->light.nld_i);
	st->light.hw = hw;
	st->light.timestamp = ts;
	nvs_light_read(&st->light);
	return 0;
}

static void jsa_read(struct jsa_state *st)
{
	unsigned int ms = st->light.poll_delay_ms;
	int ret;

	st->nvs->mutex_lock(st->nvs_data);
	if (st->enabled) {
		if (st->hw_it) {
			jsa_rd(st);
		} else {
			if (st->rc_cmd == JSA_CMD_START) {
				ret = jsa_i2c_wr(st, JSA_CMD_STOP);
				if (!ret) {
					ret = jsa_rd(st);
					if (ret)
						jsa_start(st);
					else
						ms = jsa_sleep(st);
				}
			} else {
				jsa_start(st);
			}
		}
		schedule_delayed_work(&st->dw, msecs_to_jiffies(ms));
	}
	st->nvs->mutex_unlock(st->nvs_data);
}

static void jsa_work(struct work_struct *ws)
{
	struct jsa_state *st = container_of((struct delayed_work *)ws,
					    struct jsa_state, dw);

	jsa_read(st);
}

static int jsa_disable(struct jsa_state *st)
{
	int ret;

	cancel_delayed_work(&st->dw);
	ret = jsa_pm(st, false);
	if (!ret)
		st->enabled = 0;
	return ret;
}

static int jsa_enable(void *client, int snsr_id, int enable)
{
	struct jsa_state *st = (struct jsa_state *)client;
	int ret;

	if (enable < 0)
		return st->enabled;

	if (enable) {
		ret = jsa_pm(st, true);
		if (!ret) {
			nvs_light_enable(&st->light);
			ret = jsa_start(st);
			if (ret) {
				jsa_disable(st);
			} else {
				st->enabled = 1;
				schedule_delayed_work(&st->dw,
					      msecs_to_jiffies(st->light.
							       poll_delay_ms));
			}
		}
	} else {
		ret = jsa_disable(st);
	}
	return ret;
}

static int jsa_batch(void *client, int snsr_id, int flags,
		     unsigned int period, unsigned int timeout)
{
	struct jsa_state *st = (struct jsa_state *)client;

	if (timeout)
		/* timeout not supported (no HW FIFO) */
		return -EINVAL;

	st->light.delay_us = period;
	return 0;
}

static int jsa_regs(void *client, int snsr_id, char *buf)
{
	struct jsa_state *st = (struct jsa_state *)client;
	ssize_t t;
	u16 val;
	int ret;

	t = sprintf(buf, "registers:\n");
	t += sprintf(buf + t, "CMD=%#2x\n", st->rc_cmd);
	ret = jsa_i2c_rd(st, &val);
	t += sprintf(buf + t, "VAL=%#4x  ERR=%d\n", val, ret);
	return t;
}

static struct nvs_fn_dev jsa_fn_dev = {
	.enable				= jsa_enable,
	.batch				= jsa_batch,
	.regs				= jsa_regs,
};

static int jsa_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct jsa_state *st = i2c_get_clientdata(client);
	int ret = 0;

	st->sts |= NVS_STS_SUSPEND;
	if (st->nvs && st->nvs_data)
		ret = st->nvs->suspend(st->nvs_data);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int jsa_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct jsa_state *st = i2c_get_clientdata(client);
	int ret = 0;

	if (st->nvs && st->nvs_data)
		ret = st->nvs->resume(st->nvs_data);
	st->sts &= ~NVS_STS_SUSPEND;
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static SIMPLE_DEV_PM_OPS(jsa_pm_ops, jsa_suspend, jsa_resume);

static void jsa_shutdown(struct i2c_client *client)
{
	struct jsa_state *st = i2c_get_clientdata(client);

	st->sts |= NVS_STS_SHUTDOWN;
	if (st->nvs && st->nvs_data)
		st->nvs->shutdown(st->nvs_data);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int jsa_remove(struct i2c_client *client)
{
	struct jsa_state *st = i2c_get_clientdata(client);

	if (st != NULL) {
		jsa_shutdown(client);
		if (st->nvs && st->nvs_data)
			st->nvs->remove(st->nvs_data);
		if (st->dw.wq)
			destroy_workqueue(st->dw.wq);
		jsa_pm_exit(st);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int jsa_id_dev(struct jsa_state *st, const char *name)
{
	u16 val;
	int ret = 0;

	if (name == NULL) {
		ret = jsa_i2c_rd(st, &val);
		if (!ret)
			dev_info(&st->i2c->dev, "%s %hx responded\n",
				 __func__, st->i2c_addr);
	}
	return ret;
}

static int jsa_id_i2c(struct jsa_state *st, const char *name)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(jsa_i2c_addrs); i++) {
		if (st->i2c->addr == jsa_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(jsa_i2c_addrs)) {
		st->i2c_addr = st->i2c->addr;
		ret = jsa_id_dev(st, name);
	} else {
		name = NULL;
		for (i = 0; i < ARRAY_SIZE(jsa_i2c_addrs); i++) {
			st->i2c_addr = jsa_i2c_addrs[i];
			ret = jsa_id_dev(st, name);
			if (!ret)
				break;
		}
	}
	if (ret)
		st->i2c_addr = 0;
	return ret;
}

static struct sensor_cfg jsa_cfg_dflt = {
	.name			= NVS_LIGHT_STRING,
	.ch_n			= ARRAY_SIZE(iio_chan_spec_nvs_light),
	.ch_inf			= &iio_chan_spec_nvs_light,
	.part			= JSA_NAME,
	.vendor			= JSA_VENDOR,
	.version		= JSA_LIGHT_VERSION,
	.max_range		= {
		.ival		= JSA_LIGHT_MAX_RANGE_IVAL,
		.fval		= JSA_LIGHT_MAX_RANGE_MICRO,
	},
	.resolution		= {
		.ival		= JSA_LIGHT_RESOLUTION_IVAL,
		.fval		= JSA_LIGHT_RESOLUTION_MICRO,
	},
	.milliamp		= {
		.ival		= JSA_LIGHT_MILLIAMP_IVAL,
		.fval		= JSA_LIGHT_MILLIAMP_MICRO,
	},
	.delay_us_min		= JSA_POLL_DLY_MS_MIN * 1000,
	.delay_us_max		= JSA_POLL_DLY_MS_MAX * 1000,
	.scale			= {
		.ival		= JSA_LIGHT_SCALE_IVAL,
		.fval		= JSA_LIGHT_SCALE_MICRO,
	},
	.thresh_lo		= JSA_LIGHT_THRESHOLD_LO,
	.thresh_hi		= JSA_LIGHT_THRESHOLD_HI,
};

static int jsa_of_dt(struct jsa_state *st, struct device_node *dn)
{
	unsigned int i;

	/* default NVS programmable parameters */
	memcpy(&st->cfg, &jsa_cfg_dflt, sizeof(st->cfg));
	st->light.cfg = &st->cfg;
	st->light.hw_mask = 0x7FFF;
	st->light.nld_tbl = jsa_nld_tbl;
	/* device tree parameters */
	if (dn)
		/* common NVS programmable parameters */
		st->iio_ts_en = of_property_read_bool(dn, "iio_timestamps");
	/* common NVS parameters */
	nvs_of_dt(dn, &st->cfg, NULL);
	/* this device supports these programmable parameters */
	if (nvs_light_of_dt(&st->light, dn, NULL)) {
		/* default is HW IT */
		st->light.nld_i_lo = 0;
		st->light.nld_i_hi = 0;
	}
	if (st->light.nld_i_lo == st->light.nld_i_hi) {
		/* HW IT enabled when indexes are the same */
		st->hw_it = true;
		st->light.nld_tbl = NULL; /* disable dynamic resolution */
	}
	i = st->light.nld_i_lo;
	st->cfg.resolution.ival = jsa_nld_tbl[i].resolution.ival;
	st->cfg.resolution.fval = jsa_nld_tbl[i].resolution.fval;
	i = st->light.nld_i_hi;
	st->cfg.max_range.ival = jsa_nld_tbl[i].max_range.ival;
	st->cfg.max_range.fval = jsa_nld_tbl[i].max_range.fval;
	st->cfg.delay_us_min = jsa_nld_tbl[i].delay_min_ms * 1000;
	return 0;
}

static int jsa_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct jsa_state *st;
	int ret;

	dev_info(&client->dev, "%s\n", __func__);
	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		dev_err(&client->dev, "%s devm_kzalloc ERR\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, st);
	st->i2c = client;
	ret = jsa_of_dt(st, client->dev.of_node);
	if (ret) {
		dev_err(&client->dev, "%s _of_dt ERR\n", __func__);
		ret = -ENODEV;
		goto jsa_probe_exit;
	}

	jsa_pm_init(st);
	ret = jsa_id_i2c(st, id->name);
	if (ret) {
		dev_err(&client->dev, "%s _id_i2c ERR\n", __func__);
		ret = -ENODEV;
		goto jsa_probe_exit;
	}

	jsa_fn_dev.errs = &st->errs;
	jsa_fn_dev.sts = &st->sts;
	st->nvs = nvs_iio();
	if (st->nvs == NULL) {
		dev_err(&client->dev, "%s nvs_iio ERR\n", __func__);
		ret = -ENODEV;
		goto jsa_probe_exit;
	}

	ret = st->nvs->probe(&st->nvs_data, st, &client->dev,
			     &jsa_fn_dev, &st->cfg);
	if (ret) {
		dev_err(&client->dev, "%s nvs_probe ERR\n", __func__);
		ret = -ENODEV;
		goto jsa_probe_exit;
	}

	st->light.nvs_data = st->nvs_data;
	st->light.handler = st->nvs->handler;
	INIT_DELAYED_WORK(&st->dw, jsa_work);
	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

jsa_probe_exit:
	jsa_remove(client);
	return ret;
}

static const struct i2c_device_id jsa_i2c_device_id[] = {
	{ JSA_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, jsa_i2c_device_id);

static const struct of_device_id jsa_of_match[] = {
	{ .compatible = "solteamopto,jsa1127", },
	{},
};

MODULE_DEVICE_TABLE(of, jsa_of_match);

static struct i2c_driver jsa_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= jsa_probe,
	.remove		= jsa_remove,
	.shutdown	= jsa_shutdown,
	.driver = {
		.name		= JSA_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(jsa_of_match),
		.pm		= &jsa_pm_ops,
	},
	.id_table	= jsa_i2c_device_id,
};
module_i2c_driver(jsa_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JSA1127 driver");
MODULE_AUTHOR("NVIDIA Corporation");
