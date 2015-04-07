/* Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/nvs.h>
#include <linux/nvs_light.h>

#define CM_VENDOR			"Capella Microsystems, Inc."
#define CM_NAME				"cm3218x"
#define CM_NAME_CM3218			"cm3218"
#define CM_NAME_CM32180			"cm32180"
#define CM_NAME_CM32181			"cm32181"
#define CM_DEVID_CM3218			(0x01)
#define CM_DEVID_CM32180		(0x02)
#define CM_DEVID_CM32181		(0x03)
#define CM_HW_DELAY_MS			(10)
#define CM_ALS_SM_DFLT			(0x01)
#define CM_ALS_PERS_DFLT		(0x00)
#define CM_ALS_PSM_DFLT			(0x07)
#define CM_R_SET_DFLT			(604)
#define CM_LIGHT_VERSION		(1)
#define CM_LIGHT_SCALE_IVAL		(0)
#define CM_LIGHT_SCALE_MICRO		(10000)
#define CM_LIGHT_THRESHOLD_LO		(100)
#define CM_LIGHT_THRESHOLD_HI		(100)
#define CM_POLL_DLY_MS_MAX		(4000)
/* HW registers */
#define CM_REG_CFG			(0x00)
#define CM_REG_CFG_ALS_SM		(11)
#define CM_REG_CFG_ALS_IT		(6)
#define CM_REG_CFG_ALS_PERS		(4)
#define CM_REG_CFG_RSRV_ID		(2)
#define CM_REG_CFG_ALS_INT_EN		(1)
#define CM_REG_CFG_ALS_SD		(0)
#define CM_REG_WH			(0x01)
#define CM_REG_WL			(0x02)
#define CM_REG_PSM			(0x03)
#define CM_REG_PSM_PSM			(1)
#define CM_REG_PSM_EN			(0)
#define CM_REG_PSM_MASK			(0x07)
#define CM_REG_ALS			(0x04)
#define CM_REG_ALS_IF			(0x06)
#define CM_REG_ALS_IF_L			(15)
#define CM_REG_ALS_IF_H			(14)


/* regulator names in order of powering on */
static char *cm_vregs[] = {
	"vdd",
};

static unsigned short cm_i2c_addrs[] = {
	0x10,
	0x48,
};

static struct nvs_light_dynamic cm3218_nld_tbl[] = {
	{ {0, 3570},  {233,  959950}, {0, 130000}, 1000, 0x00C0 },
	{ {0, 7140},  {467,  919900}, {0, 130000}, 500,  0x0080 },
	{ {0, 14280}, {935,  839800}, {0, 130000}, 250,  0x0040 },
	{ {0, 28560}, {1871, 679600}, {0, 130000}, 125,  0x0000 }
};

static struct nvs_light_dynamic cm32180_nld_tbl[] = {
	{ {0, 890},  {58,  326150}, {0, 130000}, 1000, 0x00C0 },
	{ {0, 1780}, {116, 652300}, {0, 130000}, 500,  0x0080 },
	{ {0, 3560}, {233, 304600}, {0, 130000}, 250,  0x0040 },
	{ {0, 7120}, {466, 609200}, {0, 130000}, 125,  0x0000 }
};

static struct nvs_light_dynamic cm32181_nld_tbl[] = {
	{ {0, 5000},   {327,   675000}, {0, 35000}, 800, 0x00C0 },
	{ {0, 10000},  {655,   350000}, {0, 35000}, 400, 0x0080 },
	{ {0, 21000},  {13762, 350000}, {0, 35000}, 200, 0x0040 },
	{ {0, 42000},  {27524, 700000}, {0, 35000}, 100, 0x0000 },
	{ {0, 84000},  {55049, 400000}, {0, 35000}, 50,  0x0200 },
	{ {0, 167000}, {10944, 345000}, {0, 35000}, 25,  0x0300 }
};

struct cm_psm {
	unsigned int ms;
	struct nvs_float milliamp;
};

static struct cm_psm cm_psm_tbl[] = {
	{ 500,  {0, 21000} },
	{ 1000, {0, 15000} },
	{ 2000, {0, 10000} },
	{ 4000, {0, 6000} }
};

struct cm_state {
	struct i2c_client *i2c;
	struct nvs_fn_if *nvs;
	void *nvs_data;
	struct sensor_cfg cfg;
	struct delayed_work dw;
	struct regulator_bulk_data vreg[ARRAY_SIZE(cm_vregs)];
	struct nvs_light light;
	struct nvs_light_dynamic nld_tbl[ARRAY_SIZE(cm32181_nld_tbl)];
	unsigned int sts;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int enabled;		/* enable status */
	bool iio_ts_en;			/* use IIO timestamps */
	bool hw_change;			/* HW changed so drop first sample */
	u16 i2c_addr;			/* I2C address */
	u8 dev_id;			/* device ID */
	u16 als_cfg;			/* ALS register 0 defaults */
	u16 als_psm;			/* ALS Power Save Mode */
	u32 r_set;			/* Rset resistor value */
};


static s64 cm_get_time_ns(struct cm_state *st)
{
	struct timespec ts;

	if (st->iio_ts_en)
		return iio_get_time_ns();

	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

static void cm_err(struct cm_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

static int cm_i2c_rd(struct cm_state *st, u8 reg, u16 *val)
{
	struct i2c_msg msg[2];

	msg[0].addr = st->i2c_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = st->i2c_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (__u8 *)val;
	if (i2c_transfer(st->i2c->adapter, msg, 2) != 2) {
		cm_err(st);
		return -EIO;
	}

	*val = le16_to_cpup(val);
	return 0;
}

static int cm_i2c_wr(struct cm_state *st, u8 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[3];

	if (st->i2c_addr) {
		buf[0] = reg;
		val = cpu_to_le16(val);
		buf[1] = val & 0xFF;
		buf[2] = val >> 8;
		msg.addr = st->i2c_addr;
		msg.flags = 0;
		msg.len = sizeof(buf);
		msg.buf = buf;
		if (i2c_transfer(st->i2c->adapter, &msg, 1) != 1) {
			cm_err(st);
			return -EIO;
		}
	}

	return 0;
}

static int cm_pm(struct cm_state *st, bool enable)
{
	int ret;

	if (enable) {
		ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
				       ARRAY_SIZE(cm_vregs));
		if (ret) {
			mdelay(CM_HW_DELAY_MS);
			if (st->dev_id == CM_DEVID_CM32181)
				cm_i2c_wr(st, CM_REG_PSM, st->als_psm);
		}
	} else {
		ret = nvs_vregs_sts(st->vreg, ARRAY_SIZE(cm_vregs));
		if ((ret < 0) || (ret == ARRAY_SIZE(cm_vregs))) {
			ret = cm_i2c_wr(st, CM_REG_CFG,
					1 << CM_REG_CFG_ALS_SD);
		} else if (ret > 0) {
			nvs_vregs_enable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(cm_vregs));
			mdelay(CM_HW_DELAY_MS);
			ret = cm_i2c_wr(st, CM_REG_CFG,
					1 << CM_REG_CFG_ALS_SD);
		}
		ret |= nvs_vregs_disable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(cm_vregs));
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

static void cm_pm_exit(struct cm_state *st)
{
	cm_pm(st, false);
	nvs_vregs_exit(&st->i2c->dev, st->vreg, ARRAY_SIZE(cm_vregs));
}

static int cm_pm_init(struct cm_state *st)
{
	int ret;

	st->enabled = 0;
	nvs_vregs_init(&st->i2c->dev,
		       st->vreg, ARRAY_SIZE(cm_vregs), cm_vregs);
	ret = cm_pm(st, true);
	return ret;
}

static int cm_cmd_wr(struct cm_state *st, bool irq_en)
{
	u16 als_cfg;
	int ret = 0;

	als_cfg = st->als_cfg;
	als_cfg |= st->nld_tbl[st->light.nld_i].driver_data;
	if (irq_en && st->i2c->irq) {
		ret = cm_i2c_wr(st, CM_REG_WL, st->light.hw_thresh_lo);
		ret |= cm_i2c_wr(st, CM_REG_WH, st->light.hw_thresh_hi);
		if (!ret) {
			als_cfg |= (1 << CM_REG_CFG_ALS_INT_EN);
			ret = RET_HW_UPDATE; /* flag IRQ enabled */
		}
	}
	ret |= cm_i2c_wr(st, CM_REG_CFG, als_cfg);
	if (ret >= 0)
		st->hw_change = true;
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s als_cfg=%hx\n",
			 __func__, als_cfg);
	return ret;
}

static int cm_rd(struct cm_state *st)
{
	u16 sts;
	u16 hw;
	s64 ts;
	int ret;

	/* spec is vague so one of these should clear the IRQ */
	ret = cm_i2c_rd(st, CM_REG_ALS, &hw);
	ret |= cm_i2c_rd(st, CM_REG_ALS_IF, &sts);
	if (ret)
		return ret;

	if (st->hw_change) {
		/* drop first sample after HW change */
		st->hw_change = false;
		return 0;
	}

	ts = cm_get_time_ns(st);
	if (st->sts & NVS_STS_SPEW_DATA)
		dev_info(&st->i2c->dev,
			 "poll light hw %hu %lld  diff=%d %lldns  index=%u\n",
			 hw, ts, hw - st->light.hw, ts - st->light.timestamp,
			 st->light.nld_i);
	st->light.hw = hw;
	st->light.timestamp = ts;
	ret = nvs_light_read(&st->light);
	switch (ret) {
	case RET_POLL_NEXT:
		if (st->light.nld_i_change)
			ret = cm_cmd_wr(st, false);
		break;

	case RET_NO_CHANGE:
		if (st->i2c->irq)
			ret = RET_HW_UPDATE;
		break;

	case RET_HW_UPDATE:
		ret = cm_cmd_wr(st, true);
		break;

	default:
		break;
	}

	return ret;
}

static void cm_read(struct cm_state *st)
{
	int ret;

	st->nvs->nvs_mutex_lock(st->nvs_data);
	if (st->enabled) {
		ret = cm_rd(st);
		if (ret < RET_HW_UPDATE)
			schedule_delayed_work(&st->dw,
				    msecs_to_jiffies(st->light.poll_delay_ms));
	}
	st->nvs->nvs_mutex_unlock(st->nvs_data);
}

static void cm_work(struct work_struct *ws)
{
	struct cm_state *st = container_of((struct delayed_work *)ws,
					   struct cm_state, dw);

	cm_read(st);
}

static irqreturn_t cm_irq_thread(int irq, void *dev_id)
{
	struct cm_state *st = (struct cm_state *)dev_id;

	if (st->sts & NVS_STS_SPEW_IRQ)
		dev_info(&st->i2c->dev, "%s\n", __func__);
	cm_read(st);
	return IRQ_HANDLED;
}

static int cm_disable(struct cm_state *st)
{
	int ret;

	cancel_delayed_work(&st->dw);
	ret = cm_pm(st, false);
	if (!ret)
		st->enabled = 0;
	return ret;
}

static int cm_enable(void *client, int snsr_id, int enable)
{
	struct cm_state *st = (struct cm_state *)client;
	unsigned int ms;
	int ret;

	if (enable < 0)
		return st->enabled;

	if (enable) {
		ret = cm_pm(st, true);
		if (!ret) {
			nvs_light_enable(&st->light);
			ret = cm_cmd_wr(st, false);
			if (ret) {
				cm_disable(st);
			} else {
				st->enabled = enable;
				ms = st->light.poll_delay_ms;
				schedule_delayed_work(&st->dw,
						      msecs_to_jiffies(ms));
			}
		}
	} else {
		ret = cm_disable(st);
	}
	return ret;
}

static int cm_batch(void *client, int snsr_id, int flags,
		    unsigned int period, unsigned int timeout)
{
	struct cm_state *st = (struct cm_state *)client;

	if (timeout)
		/* timeout not supported (no HW FIFO) */
		return -EINVAL;

	st->light.delay_us = period;
	return 0;
}

static int cm_regs(void *client, int snsr_id, char *buf)
{
	struct cm_state *st = (struct cm_state *)client;
	ssize_t t;
	u16 val;
	u8 i;
	int ret;

	t = sprintf(buf, "registers:\n");
	for (i = 0; i <= CM_REG_ALS_IF; i++) {
		ret = cm_i2c_rd(st, i, &val);
		if (!ret)
			t += sprintf(buf + t, "%#2x=%#4x\n", i, val);
	}
	return t;
}

static struct nvs_fn_dev cm_fn_dev = {
	.enable				= cm_enable,
	.batch				= cm_batch,
	.regs				= cm_regs,
};

static int cm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cm_state *st = i2c_get_clientdata(client);
	int ret = 0;

	st->sts |= NVS_STS_SUSPEND;
	if (st->nvs && st->nvs_data)
		ret = st->nvs->suspend(st->nvs_data);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int cm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cm_state *st = i2c_get_clientdata(client);
	int ret = 0;

	if (st->nvs && st->nvs_data)
		ret = st->nvs->resume(st->nvs_data);
	st->sts &= ~NVS_STS_SUSPEND;
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static SIMPLE_DEV_PM_OPS(cm_pm_ops, cm_suspend, cm_resume);

static void cm_shutdown(struct i2c_client *client)
{
	struct cm_state *st = i2c_get_clientdata(client);

	st->sts |= NVS_STS_SHUTDOWN;
	if (st->nvs && st->nvs_data)
		st->nvs->shutdown(st->nvs_data);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int cm_remove(struct i2c_client *client)
{
	struct cm_state *st = i2c_get_clientdata(client);

	if (st != NULL) {
		cm_shutdown(client);
		if (st->nvs && st->nvs_data)
			st->nvs->remove(st->nvs_data);
		if (st->dw.wq)
			destroy_workqueue(st->dw.wq);
		cm_pm_exit(st);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int cm_id_dev(struct cm_state *st, const char *name)
{
	u16 val = 0;
	unsigned int i;
	unsigned int j;
	int ret = 1;

	if (!strcmp(name, CM_NAME_CM3218))
		st->dev_id = CM_DEVID_CM3218;
	else if (!strcmp(name, CM_NAME_CM32180))
		st->dev_id = CM_DEVID_CM32180;
	else if (!strcmp(name, CM_NAME_CM32181))
		st->dev_id = CM_DEVID_CM32181;
	if (!st->dev_id) {
		ret = cm_i2c_rd(st, CM_REG_CFG, &val);
		if (ret) {
			return ret;
		} else {
			val &= (1 << CM_REG_CFG_RSRV_ID);
			st->als_cfg |= val;
			if (val) {
				if (st->i2c_addr == 0x10)
					st->dev_id = CM_DEVID_CM3218;
				else
					/* st->i2c_addr == 0x48 */
					st->dev_id = CM_DEVID_CM32180;
			} else {
				st->dev_id = CM_DEVID_CM32181;
			}
		}
	}
	switch (st->dev_id) {
	case CM_DEVID_CM3218:
		st->cfg.part = CM_NAME_CM3218;
		memcpy(&st->nld_tbl, &cm3218_nld_tbl, sizeof(cm3218_nld_tbl));
		i = ARRAY_SIZE(cm3218_nld_tbl) - 1;
		if (st->light.nld_i_hi > i)
			st->light.nld_i_hi = i;
		if (st->light.nld_i_lo > i)
			st->light.nld_i_lo = i;
		break;

	case CM_DEVID_CM32180:
		st->cfg.part = CM_NAME_CM32180;
		memcpy(&st->nld_tbl, &cm32180_nld_tbl,
		       sizeof(cm32180_nld_tbl));
		i = ARRAY_SIZE(cm32180_nld_tbl) - 1;
		if (st->light.nld_i_hi > i)
			st->light.nld_i_hi = i;
		if (st->light.nld_i_lo > i)
			st->light.nld_i_lo = i;
		break;

	case CM_DEVID_CM32181:
		st->cfg.part = CM_NAME_CM32181;
		memcpy(&st->nld_tbl, &cm32181_nld_tbl, sizeof(st->nld_tbl));
		if (st->als_psm & (1 << CM_REG_PSM_EN)) {
			j = st->als_psm >> 1;
			for (i = 0; i < ARRAY_SIZE(cm32181_nld_tbl); i++) {
				st->nld_tbl[i].delay_min_ms +=
							      cm_psm_tbl[j].ms;
				st->nld_tbl[i].milliamp.ival =
						   cm_psm_tbl[j].milliamp.ival;
				st->nld_tbl[i].milliamp.fval =
						   cm_psm_tbl[j].milliamp.fval;
			}
		}
		break;
	}

	if (!ret)
		dev_info(&st->i2c->dev, "%s found %s\n",
			 __func__, st->cfg.part);
	i = st->light.nld_i_lo;
	st->cfg.resolution.ival = st->nld_tbl[i].resolution.ival;
	st->cfg.resolution.fval = st->nld_tbl[i].resolution.fval;
	i = st->light.nld_i_hi;
	st->cfg.max_range.ival = st->nld_tbl[i].max_range.ival;
	st->cfg.max_range.fval = st->nld_tbl[i].max_range.fval;
	st->cfg.milliamp.ival = st->nld_tbl[i].milliamp.ival;
	st->cfg.milliamp.fval = st->nld_tbl[i].milliamp.fval;
	st->cfg.delay_us_min = st->nld_tbl[i].delay_min_ms * 1000;
	return 0;
}

static int cm_id_i2c(struct cm_state *st, const char *name)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(cm_i2c_addrs); i++) {
		if (st->i2c->addr == cm_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(cm_i2c_addrs)) {
		st->i2c_addr = st->i2c->addr;
		ret = cm_id_dev(st, name);
	} else {
		name = CM_NAME;
		for (i = 0; i < ARRAY_SIZE(cm_i2c_addrs); i++) {
			st->i2c_addr = cm_i2c_addrs[i];
			ret = cm_id_dev(st, name);
			if (!ret)
				break;
		}
	}
	if (ret)
		st->i2c_addr = 0;
	return ret;
}

static struct sensor_cfg cm_cfg_dflt = {
	.name			= NVS_LIGHT_STRING,
	.ch_n			= ARRAY_SIZE(iio_chan_spec_nvs_light),
	.ch_inf			= &iio_chan_spec_nvs_light,
	.part			= CM_NAME,
	.vendor			= CM_VENDOR,
	.version		= CM_LIGHT_VERSION,
	.delay_us_max		= CM_POLL_DLY_MS_MAX * 1000,
	.scale			= {
		.ival		= CM_LIGHT_SCALE_IVAL,
		.fval		= CM_LIGHT_SCALE_MICRO,
	},
	.thresh_lo		= CM_LIGHT_THRESHOLD_LO,
	.thresh_hi		= CM_LIGHT_THRESHOLD_HI,
};

static int cm_of_dt(struct cm_state *st, struct device_node *dn)
{
	u16 als_sm;
	u16 als_pers;

	/* default device specific parameters */
	als_sm = CM_ALS_SM_DFLT;
	als_pers = CM_ALS_PERS_DFLT;
	st->als_psm = CM_ALS_PSM_DFLT;
	st->r_set = CM_R_SET_DFLT;
	/* default NVS ALS programmable parameters */
	memcpy(&st->cfg, &cm_cfg_dflt, sizeof(st->cfg));
	st->light.cfg = &st->cfg;
	st->light.hw_mask = 0xFFFF;
	st->light.nld_tbl = st->nld_tbl;
	/* device tree parameters */
	if (dn) {
		/* common NVS IIO programmable parameters */
		st->iio_ts_en = of_property_read_bool(dn, "iio_timestamps");
		/* device specific parameters */
		of_property_read_u16(dn, "als_sm", &als_sm);
		of_property_read_u16(dn, "als_pers", &als_pers);
		of_property_read_u16(dn, "als_psm", &st->als_psm);
		of_property_read_u32(dn, "Rset", &st->r_set);
	}
	/* common NVS parameters */
	nvs_of_dt(dn, &st->cfg, NULL);
	/* this device supports these programmable parameters */
	if (nvs_light_of_dt(&st->light, dn, NULL)) {
		st->light.nld_i_lo = 0;
		st->light.nld_i_hi = ARRAY_SIZE(cm32181_nld_tbl) - 1;
	}
	st->als_psm &= CM_REG_PSM_MASK;
	st->als_cfg = als_pers << CM_REG_CFG_ALS_PERS;
	st->als_cfg |= als_sm << CM_REG_CFG_ALS_SM;
	return 0;
}

static int cm_probe(struct i2c_client *client,
		    const struct i2c_device_id *id)
{
	struct cm_state *st;
	int ret;

	dev_info(&client->dev, "%s\n", __func__);
	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		dev_err(&client->dev, "%s devm_kzalloc ERR\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, st);
	st->i2c = client;
	ret = cm_of_dt(st, client->dev.of_node);
	if (ret) {
		dev_err(&client->dev, "%s _of_dt ERR\n", __func__);
		ret = -ENODEV;
		goto cm_probe_exit;
	}

	cm_pm_init(st);
	ret = cm_id_i2c(st, id->name);
	if (ret) {
		dev_err(&client->dev, "%s _id_i2c ERR\n", __func__);
		ret = -ENODEV;
		goto cm_probe_exit;
	}

	cm_pm(st, false);
	cm_fn_dev.errs = &st->errs;
	cm_fn_dev.sts = &st->sts;
	st->nvs = nvs_iio();
	if (st->nvs == NULL) {
		dev_err(&client->dev, "%s nvs_iio ERR\n", __func__);
		ret = -ENODEV;
		goto cm_probe_exit;
	}

	ret = st->nvs->probe(&st->nvs_data, st, &client->dev,
			     &cm_fn_dev, &st->cfg);
	if (ret) {
		dev_err(&client->dev, "%s nvs_probe ERR\n", __func__);
		ret = -ENODEV;
		goto cm_probe_exit;
	}

	st->light.nvs_data = st->nvs_data;
	st->light.handler = st->nvs->handler;
	INIT_DELAYED_WORK(&st->dw, cm_work);
	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL, cm_irq_thread,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   CM_NAME, st);
		if (ret) {
			dev_err(&client->dev, "%s req_threaded_irq ERR %d\n",
				__func__, ret);
			ret = -ENOMEM;
			goto cm_probe_exit;
		}
	}

	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

cm_probe_exit:
	cm_remove(client);
	return ret;
}

static const struct i2c_device_id cm_i2c_device_id[] = {
	{ CM_NAME, 0 },
	{ CM_NAME_CM3218, 0 },
	{ CM_NAME_CM32180, 0 },
	{ CM_NAME_CM32181, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, cm_i2c_device_id);

static const struct of_device_id cm_of_match[] = {
	{ .compatible = "capella,cm3218x", },
	{ .compatible = "capella,cm3218", },
	{ .compatible = "capella,cm32180", },
	{ .compatible = "capella,cm32181", },
	{},
};

MODULE_DEVICE_TABLE(of, cm_of_match);

static struct i2c_driver cm_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= cm_probe,
	.remove		= cm_remove,
	.shutdown	= cm_shutdown,
	.driver = {
		.name		= CM_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(cm_of_match),
		.pm		= &cm_pm_ops,
	},
	.id_table	= cm_i2c_device_id,
};
module_i2c_driver(cm_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM3218x driver");
MODULE_AUTHOR("NVIDIA Corporation");
