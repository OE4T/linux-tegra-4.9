/* Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
/* See nvs_proximity.c and nvs_proximity.h for documentation */

/* !!!!!!!!!!!!!TODO: add IQS253 support */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/nvs.h>
#include <linux/nvs_proximity.h>


#define IQS_DRIVER_VERSION		(1)
#define IQS_VENDOR			"Azoteq"
#define IQS_NAME			"iqs2x3"
#define IQS_NAME_IQS253			"iqs253"
#define IQS_NAME_IQS263			"iqs263"
#define IQS_NAME_SAR_PROXIMITY		"SAR_proximity"
#define IQS_NAME_TOUCH_PROXIMITY	"touch_proximity"
#define IQS_DEVID_IQS253		(0x29)
#define IQS_DEVID_IQS263		(0x3C)
#define IQS_HW_DELAY_MS			(10)
#define IQS_START_DELAY_MS		(100)
#define IQS_PROX_MILLIAMP_MICRO		(180000)
#define IQS_PROX_THRESHOLD		(10)
/* configuration */
#define IQS_POLL_DLY_MS_MIN		(1000)
#define IQS_POLL_DLY_MS_MAX		(1000)
#define IQS_I2C_RETRY_N			(10)
#define IQS_RDY_RETRY_N			(25)
/* proximity defines */
#define IQS_PROX_VERSION		(1)
/* binary proximity when max_range and resolution are 1.0 */
#define IQS_PROX_MAX_RANGE_IVAL		(1)
#define IQS_PROX_MAX_RANGE_MICRO	(0)
#define IQS_PROX_RESOLUTION_IVAL	(1)
#define IQS_PROX_RESOLUTION_MICRO	(0)
#define IQS_PROX_MILLIAMP_IVAL		(0)
/* devices */
#define IQS_DEV_PROX			(0)
#define IQS_DEV_TOUCH			(1)
#define IQS_DEV_N			(2)


/* regulator names in order of powering on */
static char *iqs_vregs[] = {
	"vddhi",
};

static unsigned short iqs_i2c_addrs[] = {
	0x44,
	0x45,
	0x46,
	0x47,
};

struct iqs_wr {
	u8 len;
	u8 reg;
	u8 val[20];
	u8 msk[20];
};

static struct iqs_wr iqs263_wr_en_prox[] = {
	{ 1, 0x09, { 0x10, },
		   { 0x10, } },
	{ -1,}, /* write to HW */
	{ 1, 0x09, { 0x08, },
		   { 0x08, } },
	{ }, /* end - done - exit */
};

static struct iqs_wr iqs253_wr_en_prox[] = {
	/* TODO */
	{ }, /* end - done - exit */
};

static struct iqs_wr iqs263_wr_en_touch[] = {
	{ 1, 0x09, { 0x10, },
		   { 0xFF, } },
	{ -1,}, /* write to HW */
	{ 1, 0x09, { 0x08, },
		   { 0xFF, } },
	{ }, /* end - done - exit */
};

static struct iqs_wr iqs253_wr_en_touch[] = {
	/* TODO */
	{ }, /* end - done - exit */
};

static struct iqs_wr iqs263_wr_init[] = {
	{ 5, 0x09, { 0x40, 0x03, 0x00, 0x01, 0x03, },
		   { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, } },
	{ 1, 0x0D, { 0x09, },
		   { 0xFF, } },
	{ 1, 0x01, { 0x00, },
		   { 0xFF, } },
	{ 5, 0x07, { 0x36, 0x10, 0x20, 0x2A, 0x44, },
		   { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, } },
	{ 8, 0x0A, { 0x08, 0x10, 0x10, 0x02, 0x04, 0x08, 0x14, 0x3F, },
		   { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, } },
	{ 3, 0x0B, { 0x00, 0x64, 0x80, },
		   { 0xFF, 0xFF, 0xFF, } },
	{ }, /* end - done - exit */
};


static struct iqs_wr iqs253_wr_init[] = {
	/* TODO */
	{ }, /* end - done - exit */
};


static struct iqs_wr iqs263_wr_disable[] = {
	{ 3, 0x09, { 0x00, 0x00, 0xC0, },
		   { 0xFF, 0xFF, 0xFF, } },
	{ }, /* end - done - exit */
};

static struct iqs_wr iqs253_wr_disable[] = {
	/* TODO */
	{ }, /* end - done - exit */
};

struct iqs_hal {
	u8 reg;
	u16 len;
	u8 ndx;
	bool wr;
};

/* RI = Register Init */
/* RL = Register Length */
/* BI = Buffer Index */
#define IQS_BI_DEVINF			(0)
#define IQS_RL_DEVINF			(2)
/* IQS263 */
#define IQS263_BI_SYSFLAGS		(IQS_BI_DEVINF + \
					 IQS_RL_DEVINF + 1)
#define IQS263_RL_SYSFLAGS		(2)
#define IQS263_BI_COORDINATES		(IQS263_BI_SYSFLAGS + \
					 IQS263_RL_SYSFLAGS + 1)
#define IQS263_RL_COORDINATES		(3)
#define IQS263_BI_TOUCH			(IQS263_BI_COORDINATES + \
					 IQS263_RL_COORDINATES + 1)
#define IQS263_RL_TOUCH			(2)
#define IQS263_BI_COUNTS		(IQS263_BI_TOUCH + \
					 IQS263_RL_TOUCH + 1)
#define IQS263_RL_COUNTS		(10)
#define IQS263_BI_LTA			(IQS263_BI_COUNTS + \
					 IQS263_RL_COUNTS + 1)
#define IQS263_RL_LTA			(10)
#define IQS263_BI_DELTAS		(IQS263_BI_LTA + \
					 IQS263_RL_LTA + 1)
#define IQS263_RL_DELTAS		(8)
#define IQS263_BI_MULTIPLIERS		(IQS263_BI_DELTAS + \
					 IQS263_RL_DELTAS + 1)
#define IQS263_RL_MULTIPLIERS		(5)
#define IQS263_BI_COMPENSATION		(IQS263_BI_MULTIPLIERS + \
					 IQS263_RL_MULTIPLIERS + 1)
#define IQS263_RL_COMPENSATION		(4)
#define IQS263_BI_PROXSETTINGS		(IQS263_BI_COMPENSATION + \
					 IQS263_RL_COMPENSATION + 1)
#define IQS263_RL_PROXSETTINGS		(5)
#define IQS263_BI_THRESHOLDS		(IQS263_BI_PROXSETTINGS + \
					 IQS263_RL_PROXSETTINGS + 1)
#define IQS263_RL_THRESHOLDS		(8)
#define IQS263_BI_TIMINGS		(IQS263_BI_THRESHOLDS + \
					 IQS263_RL_THRESHOLDS + 1)
#define IQS263_RL_TIMINGS		(3)
#define IQS263_BI_GESTURE_TIMERS	(IQS263_BI_TIMINGS + \
					 IQS263_RL_TIMINGS + 1)
#define IQS263_RL_GESTURE_TIMERS	(3)
#define IQS263_BI_ACTIVE_CH		(IQS263_BI_GESTURE_TIMERS + \
					 IQS263_RL_GESTURE_TIMERS + 1)
#define IQS263_RL_ACTIVE_CH		(1)
#define IQS263_BI_N			(IQS263_BI_ACTIVE_CH + \
					 IQS263_RL_ACTIVE_CH + 1)
/* IQS253 */
#define IQS253_BI_SYSFLAGS		(IQS_BI_DEVINF + \
					 IQS_RL_DEVINF + 1)
#define IQS253_RL_SYSFLAGS		(1)
#define IQS253_BI_PROXSTS		(IQS253_BI_SYSFLAGS + \
					 IQS253_RL_SYSFLAGS + 1)
#define IQS253_RL_PROXSTS		(1)
#define IQS253_BI_TOUCHSTS		(IQS253_BI_PROXSTS + \
					 IQS253_RL_PROXSTS + 1)
#define IQS253_RL_TOUCHSTS		(3)
#define IQS253_BI_HALT			(IQS253_BI_TOUCHSTS + \
					 IQS253_RL_TOUCHSTS + 1)
#define IQS253_RL_HALT			(1)
#define IQS253_BI_ACTIVE_CH		(IQS253_BI_HALT + \
					 IQS253_RL_HALT + 1)
#define IQS253_RL_ACTIVE_CH		(1)
#define IQS253_BI_COUNTS		(IQS253_BI_ACTIVE_CH + \
					 IQS253_RL_ACTIVE_CH + 1)
#define IQS253_RL_COUNTS		(2)
#define IQS253_BI_LTA			(IQS253_BI_COUNTS + \
					 IQS253_RL_COUNTS + 1)
#define IQS253_RL_LTA			(2)
#define IQS253_BI_SETTINGS		(IQS253_BI_LTA + \
					 IQS253_RL_LTA + 1)
#define IQS253_RL_SETTINGS		(26)
#define IQS253_BI_N			(IQS253_BI_SETTINGS + \
					 IQS253_RL_SETTINGS + 1)

#if IQS263_BI_N < IQS253_BI_N
#define IQS_BI_N			(IQS253_BI_N)
#else
#define IQS_BI_N			(IQS263_BI_N)
#endif /* IQS263_BI_N < IQS253_BI_N */

static const struct iqs_hal const iqs263_hal_tbl[] = {
	{ 0x00, IQS_RL_DEVINF, IQS_BI_DEVINF, false, },
	{ 0x01, IQS263_RL_SYSFLAGS, IQS263_BI_SYSFLAGS, true, },
	{ 0x02, IQS263_RL_COORDINATES, IQS263_BI_COORDINATES, false, },
	{ 0x03, IQS263_RL_TOUCH, IQS263_BI_TOUCH, false, },
	{ 0x04, IQS263_RL_COUNTS, IQS263_BI_COUNTS, false, },
	{ 0x05, IQS263_RL_LTA, IQS263_BI_LTA, false, },
	{ 0x06, IQS263_RL_DELTAS, IQS263_BI_DELTAS, false, },
	{ 0x07, IQS263_RL_MULTIPLIERS, IQS263_BI_MULTIPLIERS, true, },
	{ 0x08, IQS263_RL_COMPENSATION, IQS263_BI_COMPENSATION, true, },
	{ 0x09, IQS263_RL_PROXSETTINGS, IQS263_BI_PROXSETTINGS, true, },
	{ 0x0A, IQS263_RL_THRESHOLDS, IQS263_BI_THRESHOLDS, true, },
	{ 0x0B, IQS263_RL_TIMINGS, IQS263_BI_TIMINGS, true, },
	{ 0x0C, IQS263_RL_GESTURE_TIMERS, IQS263_BI_GESTURE_TIMERS, true, },
	{ 0x0D, IQS263_RL_ACTIVE_CH, IQS263_BI_ACTIVE_CH, true, },
};

static const struct iqs_hal const iqs253_hal_tbl[] = {
	{ 0x00, IQS_RL_DEVINF, IQS_BI_DEVINF, false, },
	{ 0x10, IQS253_RL_SYSFLAGS, IQS253_BI_SYSFLAGS, false, },
	{ 0x31, IQS253_RL_PROXSTS, IQS253_BI_PROXSTS, false, },
	{ 0x35, IQS253_RL_TOUCHSTS, IQS253_BI_TOUCHSTS, false, },
	{ 0x39, IQS253_RL_HALT, IQS253_BI_HALT, false, },
	{ 0x3D, IQS253_RL_ACTIVE_CH, IQS253_BI_ACTIVE_CH, false, },
	{ 0x42, IQS253_RL_COUNTS, IQS253_BI_COUNTS, false, },
	{ 0x83, IQS253_RL_LTA, IQS253_BI_LTA, true, },
	{ 0xC4, IQS253_RL_SETTINGS, IQS253_BI_SETTINGS, true, },
};

#define IQS263_MSG_N			(14)
#define IQS253_MSG_N			(9)

#if IQS263_MSG_N < IQS253_MSG_N
#define IQS_MSG_N			(IQS253_MSG_N * 2)
#else
#define IQS_MSG_N			(IQS263_MSG_N * 2)
#endif /* IQS263_MSG_N < IQS253_MSG_N */

struct iqs_hal_iom {
	u8 hal_i;
	u8 offset;
	u8 mask;
};

struct iqs_hal_bit {
	struct iqs_hal_iom devinf_id;
	struct iqs_hal_iom sysflag_reset;
	struct iqs_hal_iom touch_prox;
	struct iqs_hal_iom touch_touch;
	struct iqs_hal_iom count_prox;
	struct iqs_hal_iom count_touch;
	struct iqs_hal_iom thresh_prox;
	struct iqs_hal_iom thresh_touch;
};

static const struct iqs_hal_bit iqs263_hal_bit = {
	.devinf_id			= {
		.hal_i			= 0,
		.offset			= 0,
		.mask			= 0xFF,
	},
	.sysflag_reset			= {
		.hal_i			= 1,
		.offset			= 0,
		.mask			= 0x80,
	},
	.touch_prox			= {
		.hal_i			= 3,
		.offset			= 0,
		.mask			= 0x01,
	},
	.touch_touch			= {
		.hal_i			= 3,
		.offset			= 0,
		.mask			= 0xE0,
	},
	.count_prox			= {
		.hal_i			= 4,
		.offset			= 0,
		.mask			= 1,
	},
	.count_touch			= {
		.hal_i			= 4,
		.offset			= 4,
		.mask			= 3,
	},
	.thresh_prox			= {
		.hal_i			= 10,
		.offset			= 0,
		.mask			= 1,
	},
	.thresh_touch			= {
		.hal_i			= 10,
		.offset			= 1,
		.mask			= 3,
	},
};

static const struct iqs_hal_bit iqs253_hal_bit = {
	.devinf_id			= {
		.hal_i			= 0,
		.offset			= 0,
		.mask			= 0xFF,
	},
	.sysflag_reset			= {
		.hal_i			= 1,
		.offset			= 0,
		.mask			= 0x20,
	},
	.thresh_prox			= {
		.hal_i			= 10,
		.offset			= 0,
		.mask			= 1,
	},
	.thresh_touch			= {
		.hal_i			= 10,
		.offset			= 1,
		.mask			= 3,
	},
	.count_prox			= {
		.hal_i			= 4,
		.offset			= 0,
		.mask			= 1,
	},
	.count_touch			= {
		.hal_i			= 4,
		.offset			= 4,
		.mask			= 3,
	},
};

struct iqs_state {
	struct i2c_client *i2c;
	struct nvs_fn_if *nvs;
	void *nvs_st[IQS_DEV_N];
	struct sensor_cfg cfg[IQS_DEV_N];
	struct delayed_work dw;
	struct regulator_bulk_data vreg[ARRAY_SIZE(iqs_vregs)];
	struct nvs_proximity prox[IQS_DEV_N];
	unsigned int sts;		/* status flags */
	unsigned int errs;		/* error count */
	unsigned int enabled;		/* enable status */
	u16 i2c_addr;			/* I2C address */
	u8 dev_id;			/* device ID */
	bool irq_dis;			/* interrupt disable flag */
	unsigned int os;		/* OS options */
	unsigned int i2c_retry;
	unsigned int gpio_rdy_retry;	/* GPIO RDY assert timeout */
	int gpio_rdy;
	int gpio_sar;
	unsigned int sar_assert_pol;	/* sar assert polarity */
	unsigned int msg_n;		/* I2C transaction count */
	struct i2c_msg msg[IQS_MSG_N];	/* max possible I2C transactions */
	const struct iqs_hal *hal_tbl;	/* HAL register table */
	unsigned int hal_tbl_n;		/* HAL register table count */
	const struct iqs_hal_bit *hal_bit;
	struct iqs_wr *wr_disable;
	struct iqs_wr *wr_init;
	struct iqs_wr *wr_en_prox;
	struct iqs_wr *wr_en_touch;
	u8 ri[IQS_BI_N];		/* register initialization */
	u8 rc[IQS_BI_N];		/* register cache */
};


static s64 iqs_get_time_ns(void)
{
	struct timespec ts;

	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

static void iqs_err(struct iqs_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

static void iqs_mutex_lock(struct iqs_state *st)
{
	unsigned int i;

	if (st->nvs) {
		for (i = 0; i < IQS_DEV_N; i++) {
			if (st->nvs_st[i])
				st->nvs->mutex_lock(st->nvs_st[i]);
		}
	}
}

static void iqs_mutex_unlock(struct iqs_state *st)
{
	unsigned int i;

	if (st->nvs) {
		for (i = 0; i < IQS_DEV_N; i++) {
			if (st->nvs_st[i])
				st->nvs->mutex_unlock(st->nvs_st[i]);
		}
	}
}

static void iqs_disable_irq(struct iqs_state *st)
{
	if (st->i2c->irq && !st->irq_dis) {
		disable_irq_nosync(st->i2c->irq);
		st->irq_dis = true;
		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s IRQ disabled\n", __func__);
	}
}

static void iqs_enable_irq(struct iqs_state *st)
{
	if (st->i2c->irq && st->irq_dis) {
		enable_irq(st->i2c->irq);
		st->irq_dis = false;
		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s IRQ enabled\n", __func__);
	}
}

static int iqs_gpio_sar(struct iqs_state *st, int prox)
{
	int gpio_sar_val = -1;
	int ret = -EINVAL;

	if (st->prox[IQS_DEV_PROX].proximity_binary_hw && st->gpio_sar >= 0) {
		gpio_sar_val = st->sar_assert_pol;
		gpio_sar_val ^= prox;
		ret = gpio_direction_output(st->gpio_sar, gpio_sar_val);
	}
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s prox=%d  gpio_sar %d=%d  ret=%d\n",
			 __func__, prox, st->gpio_sar, gpio_sar_val, ret);
	return ret;
}

static int iqs_gpio_rdy(struct iqs_state *st, int level)
{
	unsigned int i;
	unsigned int j;
	int ret = 0;

	iqs_disable_irq(st);
	for (i = 0; i < st->gpio_rdy_retry; i++) {
		gpio_direction_output(st->gpio_rdy, 0);
		usleep_range(10000, 12000);
		/* put to tristate */
		gpio_direction_input(st->gpio_rdy);
		for (j = 0; j < 2000; j++) {
			usleep_range(500, 1000);
			ret = gpio_get_value(st->gpio_rdy);
			if (!ret)
				break;
		}
		if (!ret)
			break;
	}
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s gpio_rdy=%d  retries=%u\n",
			 __func__, ret, i);
	return ret;
}

static int iqs_i2c(struct iqs_state *st)
{
	ssize_t t;
	char spew[128];
	unsigned int i;
	unsigned int n = 0;
	int ret = -ENODEV;

	if (st->i2c_addr && st->msg_n) {
#ifdef IQS_I2C_M_NO_RD_ACK
		/* This IQS device mangles the I2C protocol for read restarts.
		 * In order to do I2C message stacking to avoid an I2C STOP
		 * that would end the communication window, the last read
		 * byte must not be acknowledged, and instead the restart done.
		 * Probably no normal I2C HW supports this thus requiring the
		 * I2C bus to be bit-banged.
		 * I2C_M_NO_RD_ACK probably wouldn't work (if supported)
		 * because it's just the last read byte that requires this,
		 * not all of them.
		 */
		for (i = 0; i < st->msg_n - 1; i++) {
			if (st->msg[i].flags & I2C_M_RD)
				st->msg[i].flags |= I2C_M_NO_RD_ACK;
		}
#endif /* IQS_I2C_M_NO_RD_ACK */

		for (i = 0; i < st->i2c_retry; i++) {
			ret = gpio_get_value(st->gpio_rdy);
			if (ret) {
				ret = iqs_gpio_rdy(st, 0);
				if (ret)
					continue;
			}

			ret = i2c_transfer(st->i2c->adapter,
					   &st->msg[n], st->msg_n);
			if (ret == st->msg_n) {
				ret = 0;
				break;
			} else {
				if (ret < 0)
					continue;

				/* skip the successful messages */
				n = ret;
				while (n && st->msg[n].flags & I2C_M_RD)
					n--;
				st->msg_n -= n;
				ret = -EIO;
			}
		}
		if (ret)
			iqs_err(st);
		if (st->sts & NVS_STS_SPEW_MSG) {
			st->msg_n += n;
			dev_info(&st->i2c->dev, "%s retries=%u err=%d\n",
				 __func__, i, ret);
			for (i = 0; i < st->msg_n; i++) {
				n = 0;
				if (st->msg[i].flags & I2C_M_RD) {
					t = sprintf(spew, "read=");
				} else {
					if (st->msg[i].len == 1) {
						/* part of read transaction */
						t = sprintf(spew, "read %#2x=",
							    st->msg[i].buf[0]);
						i++;
					} else {
						t = sprintf(spew,
							    "write %#2x=",
							    st->msg[i].buf[0]);
						n = 1;
					}
				}
				for (; n < st->msg[i].len; n++)
					t += sprintf(spew + t, "%#2x ",
						     st->msg[i].buf[n]);
				dev_info(&st->i2c->dev, "%s %s\n",
					 __func__, spew);
			}
		}
	} else {
		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev,
				 "%s NO OP: i2c_addr=%hx  msg_n=%u\n",
				 __func__, st->i2c_addr, st->msg_n);
	}
	st->msg_n = 0;
	return ret;
}

static int iqs_i2c_rd(struct iqs_state *st, int hal_i, u16 len)
{
	if (st->msg_n + 1 < ARRAY_SIZE(st->msg)) {
		st->msg[st->msg_n].flags = 0;
		st->msg[st->msg_n].len = 1;
		st->msg[st->msg_n].buf = &st->rc[st->hal_tbl[hal_i].ndx];
		st->msg_n++;
		st->msg[st->msg_n].flags = I2C_M_RD;
		if (len)
			st->msg[st->msg_n].len = len;
		else
			st->msg[st->msg_n].len = st->hal_tbl[hal_i].len;
		st->msg[st->msg_n].buf = &st->rc[st->hal_tbl[hal_i].ndx + 1];
		st->msg_n++;
		return 0;
	}

	return -EINVAL;
};

static int iqs_i2c_read(struct iqs_state *st, int hal_i, u16 len)
{
	iqs_i2c_rd(st, hal_i, len);
	return iqs_i2c(st);
};

static int iqs_i2c_wr(struct iqs_state *st, int hal_i, u16 len)
{
	if (st->msg_n < ARRAY_SIZE(st->msg)) {
		st->msg[st->msg_n].flags = 0;
		if (len)
			st->msg[st->msg_n].len = len + 1;
		else
			st->msg[st->msg_n].len = st->hal_tbl[hal_i].len + 1;
		st->msg[st->msg_n].buf = &st->rc[st->hal_tbl[hal_i].ndx];
		st->msg_n++;
		return 0;
	}

	return -EINVAL;
};

static int iqs_i2c_write(struct iqs_state *st, int hal_i, u16 len)
{
	iqs_i2c_wr(st, hal_i, len);
	return iqs_i2c(st);
};

static int iqs_write(struct iqs_state *st, struct iqs_wr *wr)
{
	unsigned int i = 0;
	unsigned int j;
	unsigned int k;
	unsigned int b;
	int ret = 0;

	if (!st->hal_tbl_n)
		/* exit if HAL not initialized */
		return -EINVAL;

	while (wr[i].len) {
		if (wr[i].len == 0xFF) {
			ret |= iqs_i2c(st);
			i++;
			continue;
		}

		for (j = st->hal_tbl_n - 1; j > 0; j--) {
			if (st->hal_tbl[j].reg == wr[i].reg) {
				if (st->hal_tbl[j].len >= wr[i].len) {
					b = st->hal_tbl[j].ndx + 1;
					for (k = 0; k < wr[i].len; k++) {
						st->rc[b + k] &= ~wr[i].msk[k];
						st->rc[b + k] |= wr[i].val[k];
					}
					ret |= iqs_i2c_wr(st, j, wr[i].len);
				}
			}
		}
		i++;
	}
	return ret;
};

static int iqs_en(struct iqs_state *st, int snsr_id)
{
	struct iqs_wr *wr;
	int ret;

	if (snsr_id == IQS_DEV_PROX)
		wr = st->wr_en_prox;
	else if (snsr_id == IQS_DEV_TOUCH)
		wr = st->wr_en_touch;
	else
		return -EINVAL;

	ret = iqs_write(st, wr);
	if (!ret)
		ret = iqs_i2c(st);
	if (!ret)
		ret = nvs_proximity_enable(&st->prox[snsr_id]);
	return ret;
}

static int iqs_init(struct iqs_state *st)
{
	int ret;

	ret = iqs_write(st, st->wr_init);
	if (!ret)
		ret = iqs_i2c(st);
	return ret;
}

static int iqs_dis(struct iqs_state *st)
{
	int ret;

	ret = iqs_write(st, st->wr_disable);
	if (!ret)
		ret = iqs_i2c(st);
	return ret;
}

static int iqs_pm(struct iqs_state *st, bool enable)
{
	int ret = 0;

	if (enable) {
		nvs_vregs_enable(&st->i2c->dev, st->vreg,
				 ARRAY_SIZE(iqs_vregs));
		if (ret)
			mdelay(IQS_HW_DELAY_MS);
		if (st->hal_tbl_n)
			/* only if HAL initialized */
			ret = iqs_init(st);
	} else {
		ret = nvs_vregs_sts(st->vreg, ARRAY_SIZE(iqs_vregs));
		if ((ret < 0) || (ret == ARRAY_SIZE(iqs_vregs))) {
			ret = iqs_dis(st);
		} else if (ret > 0) {
			nvs_vregs_enable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(iqs_vregs));
			mdelay(IQS_HW_DELAY_MS);
			ret = iqs_init(st);
			ret |= iqs_dis(st);
		}
		ret |= nvs_vregs_disable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(iqs_vregs));
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

static void iqs_pm_exit(struct iqs_state *st)
{
	iqs_pm(st, false);
	nvs_vregs_exit(&st->i2c->dev, st->vreg, ARRAY_SIZE(iqs_vregs));
}

static int iqs_pm_init(struct iqs_state *st)
{
	int ret;

	st->enabled = 0;
	nvs_vregs_init(&st->i2c->dev,
		       st->vreg, ARRAY_SIZE(iqs_vregs), iqs_vregs);
	ret = iqs_pm(st, true);
	return ret;
}

static int iqs_rd_touch(struct iqs_state *st, s64 ts)
{
	u16 hw;
	unsigned int i;
	int ret;

	if (st->prox[IQS_DEV_TOUCH].proximity_binary_hw) {
		i = st->hal_tbl[st->hal_bit->touch_touch.hal_i].ndx + 1;
		i += st->hal_bit->touch_touch.offset;
		hw = !!(st->rc[i] & st->hal_bit->touch_touch.mask);
	} else {
		i = st->hal_tbl[st->hal_bit->count_touch.hal_i].ndx + 1;
		i += st->hal_bit->count_touch.offset;
		hw = (u16)st->rc[i];
	}
	if (st->sts & NVS_STS_SPEW_DATA)
		dev_info(&st->i2c->dev, "touch hw %hu %lld  diff=%d %lldns\n",
			 hw, ts, hw - st->prox[IQS_DEV_TOUCH].hw,
			 ts - st->prox[IQS_DEV_TOUCH].timestamp);
	st->prox[IQS_DEV_TOUCH].hw = hw;
	st->prox[IQS_DEV_TOUCH].timestamp = ts;
	ret = nvs_proximity_read(&st->prox[IQS_DEV_TOUCH]);
	return ret;
}

static int iqs_rd_prox(struct iqs_state *st, s64 ts)
{
	u16 hw;
	unsigned int i;
	int ret;

	if (st->prox[IQS_DEV_PROX].proximity_binary_hw) {
		i = st->hal_tbl[st->hal_bit->touch_prox.hal_i].ndx + 1;
		i += st->hal_bit->touch_prox.offset;
		/* reverse polarity for Android (0=close 1=far) */
		hw = !(st->rc[i] & st->hal_bit->touch_prox.mask);
	} else {
		i = st->hal_tbl[st->hal_bit->count_prox.hal_i].ndx + 1;
		i += st->hal_bit->count_prox.offset;
		hw = (u16)st->rc[i];
	}
	if (st->sts & NVS_STS_SPEW_DATA)
		dev_info(&st->i2c->dev, "prox hw %hu %lld  diff=%d %lldns\n",
			 hw, ts, hw - st->prox[IQS_DEV_PROX].hw,
			 ts - st->prox[IQS_DEV_PROX].timestamp);
	st->prox[IQS_DEV_PROX].hw = hw;
	st->prox[IQS_DEV_PROX].timestamp = ts;
	ret = nvs_proximity_read(&st->prox[IQS_DEV_PROX]);
	iqs_gpio_sar(st, st->prox[IQS_DEV_PROX].proximity);
	return ret;
}

static int iqs_rd(struct iqs_state *st)
{
	s64 ts;
	bool prox_binary = false;
	bool prox_full = false;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < IQS_DEV_N; i++) {
		if (st->enabled & (1 << i)) {
			if (st->prox[i].proximity_binary_hw)
				prox_binary = true;
			else
				prox_full = true;
		}
	}
#ifdef IQS_I2C_M_NO_RD_ACK
	/* I2C message stacking */
	iqs_i2c_rd(st, st->hal_bit->sysflag_reset.hal_i, 0);
	if (prox_full)
		/* read counts */
		iqs_i2c_rd(st, st->hal_bit->count_prox.hal_i, 0);
	if (prox_binary)
		/* read binary status */
		iqs_i2c_rd(st, st->hal_bit->touch_prox.hal_i, 0);
	ret = iqs_i2c(st);
#else
	ret = iqs_i2c_read(st, st->hal_bit->sysflag_reset.hal_i, 0);
	if (prox_full)
		/* read counts */
		ret |= iqs_i2c_read(st, st->hal_bit->count_prox.hal_i, 0);
	if (prox_binary)
		/* read counts */
		ret |= iqs_i2c_read(st, st->hal_bit->touch_prox.hal_i, 0);
#endif /* IQS_I2C_M_NO_RD_ACK */
	if (!ret) {
		i = st->hal_tbl[st->hal_bit->sysflag_reset.hal_i].ndx + 1;
		if (st->rc[i] & st->hal_bit->sysflag_reset.mask) {
			iqs_err(st);
			iqs_init(st);
			for (i = 0; i < IQS_DEV_N; i++) {
				if (st->enabled & (1 << i))
					iqs_en(st, i);
			}
			return RET_POLL_NEXT;
		}

		ts = iqs_get_time_ns();
		if (st->enabled & (1 << IQS_DEV_PROX))
			ret |= iqs_rd_prox(st, ts);
		if (st->enabled & (1 << IQS_DEV_TOUCH))
			ret |= iqs_rd_touch(st, ts);
	}
	if (ret >= RET_NO_CHANGE) {
		if (!st->i2c->irq)
			/* no interrupt - force polling */
			ret = RET_POLL_NEXT;
	} /* else  poll if error or more reporting */
	return ret;
}

static unsigned int iqs_polldelay(struct iqs_state *st)
{
	unsigned int poll_delay_ms = IQS_POLL_DLY_MS_MAX;
	unsigned int i;

	for (i = 0; i < IQS_DEV_N; i++) {
		if (st->enabled & (1 << i)) {
			if (poll_delay_ms > st->prox[i].poll_delay_ms)
				poll_delay_ms = st->prox[i].poll_delay_ms;
		}
	}
	return poll_delay_ms;
}

static void iqs_read(struct iqs_state *st)
{
	int ret;

	iqs_mutex_lock(st);
	if (st->enabled) {
		ret = iqs_rd(st);
		if (ret < RET_NO_CHANGE)
			schedule_delayed_work(&st->dw,
					  msecs_to_jiffies(iqs_polldelay(st)));
		else
			iqs_enable_irq(st);
	}
	iqs_mutex_unlock(st);
}

static void iqs_work(struct work_struct *ws)
{
	struct iqs_state *st = container_of((struct delayed_work *)ws,
					    struct iqs_state, dw);

	iqs_read(st);
}

static irqreturn_t iqs_irq_thread(int irq, void *dev_id)
{
	struct iqs_state *st = (struct iqs_state *)dev_id;

	if (st->sts & NVS_STS_SPEW_IRQ)
		dev_info(&st->i2c->dev, "%s\n", __func__);
	iqs_read(st);
	return IRQ_HANDLED;
}

static int iqs_disable(struct iqs_state *st, int snsr_id)
{
	bool disable = true;
	int ret = 0;

	if (snsr_id >= 0) {
		if (st->enabled & ~(1 << snsr_id)) {
			st->enabled &= ~(1 << snsr_id);
			disable = false;
		}
	}
	if (disable) {
		iqs_disable_irq(st);
		cancel_delayed_work(&st->dw);
		ret = iqs_pm(st, false);
		if (!ret)
			st->enabled = 0;
	}
	return ret;
}

static int iqs_enable(void *client, int snsr_id, int enable)
{
	struct iqs_state *st = (struct iqs_state *)client;
	int ret;

	if (enable < 0)
		return st->enabled & (1 << snsr_id);

	if (enable) {
		enable = st->enabled | (1 << snsr_id);
		ret = iqs_pm(st, true);
		if (!ret) {
			ret = iqs_en(st, snsr_id);
			if (ret < 0) {
				iqs_disable(st, snsr_id);
			} else {
				st->enabled = enable;
				schedule_delayed_work(&st->dw,
					 msecs_to_jiffies(IQS_START_DELAY_MS));
			}
		}
	} else {
		ret = iqs_disable(st, snsr_id);
	}
	return ret;
}

static int iqs_enable_os(void *client, int snsr_id, int enable)
{
	struct iqs_state *st = (struct iqs_state *)client;

	if (st->os)
		return iqs_enable(st, snsr_id, enable);

	if (enable < 0)
		return st->enabled & (1 << snsr_id);

	if (enable)
		st->prox[snsr_id].report = st->cfg[snsr_id].report_n;
	return 0;
}

static int iqs_batch(void *client, int snsr_id, int flags,
		    unsigned int period_us, unsigned int timeout_us)
{
	struct iqs_state *st = (struct iqs_state *)client;

	if (timeout_us)
		/* timeout not supported (no HW FIFO) */
		return -EINVAL;

	st->prox[snsr_id].delay_us = period_us;
	return 0;
}

static	int iqs_thresh(void *client, int snsr_id, int thresh)
{
	struct iqs_state *st = (struct iqs_state *)client;
	unsigned int hal_i;
	unsigned int i;
	unsigned int n;
	int ret;

	if (snsr_id == IQS_DEV_TOUCH) {
		hal_i = st->hal_bit->thresh_touch.hal_i;
		i = st->hal_bit->thresh_touch.offset + 1;
		n = i + st->hal_bit->thresh_touch.mask;
	} else {
		hal_i = st->hal_bit->thresh_prox.hal_i;
		i = st->hal_bit->thresh_prox.offset + 1;
		n = i + st->hal_bit->thresh_prox.mask;
	}
	for (; i < n; i++) {
		st->rc[st->hal_tbl[hal_i].ndx + i] = thresh & 0xFF;
		thresh >>= 8;
	}
	ret = iqs_i2c_write(st, hal_i, 0);
	if (!ret) {
		st->cfg[snsr_id].thresh_lo = thresh;
		st->cfg[snsr_id].thresh_hi = thresh;
	}
	return ret;
}

static int iqs_regs(void *client, int snsr_id, char *buf)
{
	struct iqs_state *st = (struct iqs_state *)client;
	ssize_t t;
	bool irq_dis;
	unsigned int i;
	unsigned int j;
	unsigned int n;
	int ret = 0;

	iqs_mutex_lock(st);
	irq_dis = st->irq_dis;
#ifdef IQS_I2C_M_NO_RD_ACK
	/* I2C message stacking */
	for (i = 0; i < st->hal_tbl_n; i++)
		iqs_i2c_rd(st, i, 0);
	ret = iqs_i2c(st);
	t = sprintf(buf, "registers: (ERR=%d)\n", ret);
	for (i = 0; i < st->hal_tbl_n; i++) {
		n = st->hal_tbl[i].ndx;
		t += sprintf(buf + t, "0x%hhx=", st->rc[n]);
		n = st->hal_tbl[i].ndx + st->hal_tbl[i].len;
		for (j = st->hal_tbl[i].ndx + 1; j <= n; j++)
			t += sprintf(buf + t, "0x%hhx ", st->rc[j]);
		t += sprintf(buf + t, "\n");
	}
#else
	t = sprintf(buf, "registers:\n");
	for (i = 0; i < st->hal_tbl_n; i++) {
		ret = iqs_i2c_read(st, i, 0);
		n = st->hal_tbl[i].ndx;
		if (ret) {
			t += sprintf(buf + t, "0x%hhx=ERR %d",
				     st->rc[n], ret);
		} else {
			t += sprintf(buf + t, "0x%hhx=", st->rc[n]);
			n = st->hal_tbl[i].ndx + st->hal_tbl[i].len;
			for (j = st->hal_tbl[i].ndx + 1; j <= n; j++)
				t += sprintf(buf + t, "0x%hhx ", st->rc[j]);
			t += sprintf(buf + t, "\n");
		}
	}
#endif /* IQS_I2C_M_NO_RD_ACK */
	if (!irq_dis)
		iqs_enable_irq(st);
	iqs_mutex_unlock(st);
	return t;
}

static int iqs_nvs_write(void *client, int snsr_id, unsigned int nvs)
{
	struct iqs_state *st = (struct iqs_state *)client;
	u8 val = (nvs >> 8) & 0xFF;
	unsigned int offset;
	unsigned int reg;
	unsigned int i;
	int ret = -EINVAL;

	switch (nvs & 0xFF) {
	case 0x10:
		ret = gpio_direction_input(st->gpio_rdy);
		dev_info(&st->i2c->dev,
			 "%s gpio_direction_input(gpio_rdy(%d))=%d\n",
			 __func__, st->gpio_rdy, ret);
		return ret;

	case 0x11:
		val = !!val;
		ret = gpio_direction_output(st->gpio_rdy, val);
		dev_info(&st->i2c->dev,
			 "%s gpio_direction_output(gpio_rdy(%d), %hhx)=%d\n",
			 __func__, st->gpio_rdy, val, ret);
		return ret;

	case 0x12:
		val = !!val;
		if (st->gpio_sar >= 0)
			ret = gpio_direction_output(st->gpio_sar, val);
		dev_info(&st->i2c->dev,
			 "%s gpio_direction_output(gpio_sar(%d), %hhx)=%d\n",
			 __func__, st->gpio_sar, val, ret);
		return ret;

	case 0x13:
		offset = ((nvs >> 16) & 0xFF) + 1;
		reg = (nvs >> 24) & 0xFF;
		for (i = 0; i < st->hal_tbl_n; i++) {
			if (st->hal_tbl[i].reg == reg) {
				if (offset > st->hal_tbl[i].len)
					break;

				iqs_mutex_lock(st);
				st->rc[st->hal_tbl[i].ndx + offset] = val;
				ret = iqs_i2c_write(st, i, 0);
				iqs_mutex_unlock(st);
				dev_info(&st->i2c->dev,
					 "%s %x => %x + %u err=%d\n",
					 __func__, val, reg, offset - 1, ret);
				return ret;
			}
		}
		return ret;

	default:
		ret = 0;
		break;
	}

	return ret;
}

static int iqs_nvs_read(void *client, int snsr_id, char *buf)
{
	struct iqs_state *st = (struct iqs_state *)client;
	ssize_t t;

	t = sprintf(buf, "IQS driver v. %u\n", IQS_DRIVER_VERSION);
	t += sprintf(buf + t, "os_options=%x\n", st->os);
	t += sprintf(buf + t, "i2c_retry=%u\n", st->i2c_retry);
	t += sprintf(buf + t, "gpio_rdy_retry=%u\n", st->gpio_rdy_retry);
	if (st->gpio_rdy < 0)
		t += sprintf(buf + t, "gpio_rdy=%d\n", st->gpio_rdy);
	else
		t += sprintf(buf + t, "gpio_rdy %d=%d\n",
			     st->gpio_rdy, gpio_get_value(st->gpio_rdy));
	if (st->gpio_sar < 0)
		t += sprintf(buf + t, "gpio_sar=%d\n", st->gpio_sar);
	else
		t += sprintf(buf + t, "gpio_sar %d=%d\n",
			     st->gpio_sar, gpio_get_value(st->gpio_sar));
	t += sprintf(buf + t, "irq=%d\n", st->i2c->irq);
	t += sprintf(buf + t, "irq_disable=%x\n", st->irq_dis);
	t += sprintf(buf + t, "SAR_proximity_binary_hw=%x\n",
		     st->prox[IQS_DEV_PROX].proximity_binary_hw);
	t += sprintf(buf + t, "touch_proximity_binary_hw=%x\n",
		     st->prox[IQS_DEV_TOUCH].proximity_binary_hw);
	return t;
}

static struct nvs_fn_dev iqs_fn_dev = {
	.enable				= iqs_enable_os,
	.batch				= iqs_batch,
	.thresh_lo			= iqs_thresh,
	.thresh_hi			= iqs_thresh,
	.regs				= iqs_regs,
	.nvs_write			= iqs_nvs_write,
	.nvs_read			= iqs_nvs_read,
};

static int iqs_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iqs_state *st = i2c_get_clientdata(client);
	unsigned int i;
	int ret = 0;

	st->sts |= NVS_STS_SUSPEND;
	if (st->os) {
		if (st->nvs) {
			for (i = 0; i < IQS_DEV_N; i++) {
				if (st->nvs_st[i])
					ret |= st->nvs->suspend(st->nvs_st[i]);
			}
		}
	}
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s err=%d\n", __func__, ret);
	return ret;
}

static int iqs_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iqs_state *st = i2c_get_clientdata(client);
	unsigned int i;
	int ret = 0;

	if (st->os) {
		if (st->nvs) {
			for (i = 0; i < IQS_DEV_N; i++) {
				if (st->nvs_st[i])
					ret |= st->nvs->resume(st->nvs_st[i]);
			}
		}
	}
	st->sts &= ~NVS_STS_SUSPEND;
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s err=%d\n", __func__, ret);
	return 0;
}

static SIMPLE_DEV_PM_OPS(iqs_pm_ops, iqs_suspend, iqs_resume);

static void iqs_shutdown(struct i2c_client *client)
{
	struct iqs_state *st = i2c_get_clientdata(client);
	unsigned int i;

	st->sts |= NVS_STS_SHUTDOWN;
	if (!st->os)
		iqs_disable(st, -1);
	if (st->nvs) {
		for (i = 0; i < IQS_DEV_N; i++) {
			if (st->nvs_st[i])
				st->nvs->shutdown(st->nvs_st[i]);
		}
	}
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int iqs_remove(struct i2c_client *client)
{
	struct iqs_state *st = i2c_get_clientdata(client);
	unsigned int i;

	if (st != NULL) {
		iqs_shutdown(client);
		if (st->nvs) {
			for (i = 0; i < IQS_DEV_N; i++) {
				if (st->nvs_st[i])
					st->nvs->remove(st->nvs_st[i]);
			}
		}
		if (st->dw.wq)
			destroy_workqueue(st->dw.wq);
		iqs_pm_exit(st);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static void iqs_id_part(struct iqs_state *st, const char *part)
{
	unsigned int i;

	for (i = 0; i < IQS_DEV_N; i++)
		st->cfg[i].part = part;
}

static int iqs_id_dev(struct iqs_state *st, const char *name)
{
	unsigned int hal_i;
	unsigned int i;
	int ret = 1;

	/* assume for initial dev id read */
	st->hal_tbl = iqs263_hal_tbl;
	st->hal_tbl_n = ARRAY_SIZE(iqs263_hal_tbl);
	st->hal_bit = &iqs263_hal_bit;
	for (i = 0; i < ARRAY_SIZE(st->msg); i++)
		st->msg[i].addr = st->i2c_addr;
	if (!strcmp(name, IQS_NAME_IQS263))
		st->dev_id = IQS_DEVID_IQS263;
	else if (!strcmp(name, IQS_NAME_IQS253))
		st->dev_id = IQS_DEVID_IQS253;
	if (!st->dev_id) {
		hal_i = st->hal_bit->devinf_id.hal_i;
		i = st->hal_bit->devinf_id.offset + 1;
		ret = iqs_i2c_read(st, hal_i, i);
		if (ret) {
			return ret;
		} else {
			i += st->hal_tbl[hal_i].ndx;
			st->dev_id = st->rc[i];
		}
	}

	switch (st->dev_id) {
	case IQS_DEVID_IQS263:
		BUG_ON(IQS263_MSG_N != ARRAY_SIZE(iqs263_hal_tbl));
		iqs_id_part(st, IQS_NAME_IQS263);
		st->wr_disable = iqs263_wr_disable;
		st->wr_init = iqs263_wr_init;
		st->wr_en_prox = iqs263_wr_en_prox;
		st->wr_en_touch = iqs263_wr_en_touch;
		break;

	case IQS_DEVID_IQS253:
		BUG_ON(IQS253_MSG_N != ARRAY_SIZE(iqs253_hal_tbl));
		iqs_id_part(st, IQS_NAME_IQS253);
		st->hal_tbl = iqs253_hal_tbl;
		st->hal_tbl_n = ARRAY_SIZE(iqs253_hal_tbl);
		st->hal_bit = &iqs253_hal_bit;
		st->wr_disable = iqs253_wr_disable;
		st->wr_init = iqs253_wr_init;
		st->wr_en_prox = iqs253_wr_en_prox;
		st->wr_en_touch = iqs253_wr_en_touch;
		break;

	default:
		return -ENODEV;
	}

	for (i = 0; i < st->hal_tbl_n; i++)
		/* fill in register addresses for I2C writes */
		st->rc[st->hal_tbl[i].ndx] = st->hal_tbl[i].reg;
	if (!ret)
		dev_info(&st->i2c->dev, "%s found %s\n",
			 __func__, st->cfg[0].part);
	return 0;
}

static int iqs_id_i2c(struct iqs_state *st, const char *name)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(iqs_i2c_addrs); i++) {
		if (st->i2c->addr == iqs_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(iqs_i2c_addrs)) {
		st->i2c_addr = st->i2c->addr;
		ret = iqs_id_dev(st, name);
	} else {
		name = IQS_NAME;
		for (i = 0; i < ARRAY_SIZE(iqs_i2c_addrs); i++) {
			st->i2c_addr = iqs_i2c_addrs[i];
			ret = iqs_id_dev(st, name);
			if (!ret)
				break;
		}
	}
	if (ret)
		st->i2c_addr = 0;
	return ret;
}

static const char *iqs_sensor_cfg_name[] = {
	IQS_NAME_SAR_PROXIMITY,
	IQS_NAME_TOUCH_PROXIMITY,
};

static const struct sensor_cfg iqs_cfg_dflt = {
	.snsr_id			= SENSOR_TYPE_PROXIMITY,
	.ch_n				= 1,
	.ch_sz				= 4,
	.part				= IQS_NAME,
	.vendor				= IQS_VENDOR,
	.version			= IQS_PROX_VERSION,
	.max_range			= {
		.ival			= IQS_PROX_MAX_RANGE_IVAL,
		.fval			= IQS_PROX_MAX_RANGE_MICRO,
	},
	.resolution			= {
		.ival			= IQS_PROX_RESOLUTION_IVAL,
		.fval			= IQS_PROX_RESOLUTION_MICRO,
	},
	.milliamp			= {
		.ival			= IQS_PROX_MILLIAMP_IVAL,
		.fval			= IQS_PROX_MILLIAMP_MICRO,
	},
	.delay_us_min			= IQS_POLL_DLY_MS_MIN * 1000,
	.delay_us_max			= IQS_POLL_DLY_MS_MAX * 1000,
	.thresh_lo			= IQS_PROX_THRESHOLD,
	.thresh_hi			= IQS_PROX_THRESHOLD,
};

static int iqs_of_dt(struct iqs_state *st, struct device_node *dn)
{
	unsigned int i;
	int ret;

	/* default device specific parameters */
	for (i = 0; i < IQS_DEV_N; i++) {
		memcpy(&st->cfg[i], &iqs_cfg_dflt, sizeof(st->cfg[0]));
		st->cfg[i].name = iqs_sensor_cfg_name[i];
		st->prox[i].cfg = &st->cfg[i];
		st->prox[i].hw_mask = 0xFFFF;
		st->prox[i].proximity_binary_hw = true;
		nvs_proximity_of_dt(&st->prox[i], dn, st->cfg[i].name);
	}
	st->i2c_retry = IQS_I2C_RETRY_N;
	st->gpio_rdy_retry = IQS_RDY_RETRY_N;
	st->gpio_rdy = -1;
	st->gpio_sar = -1;
	/* device tree parameters */
	if (dn) {
		/* device specific parameters */
		of_property_read_u32(dn, "os_options", &st->os);
		of_property_read_u32(dn, "i2c_retry", &st->i2c_retry);
		of_property_read_u32(dn, "gpio_rdy_retry",
				     &st->gpio_rdy_retry);
		of_property_read_u32(dn, "sar_assert_polarity",
				     &st->sar_assert_pol);
		st->sar_assert_pol = !!st->sar_assert_pol;
		st->gpio_rdy = of_get_named_gpio(dn, "gpio_rdy", 0);
		st->gpio_sar = of_get_named_gpio(dn, "gpio_sar", 0);
	}
	if (gpio_is_valid(st->gpio_rdy)) {
		ret = gpio_request(st->gpio_rdy, IQS_NAME);
		if (ret) {
			dev_err(&st->i2c->dev,
				"%s gpio_request(%d %s) ERR:%d\n",
				__func__, st->gpio_rdy, IQS_NAME, ret);
			/* can't communicate with device without this GPIO */
			return -EPROBE_DEFER;
		} else {
			ret = gpio_direction_input(st->gpio_rdy);
			if (ret < 0) {
				dev_err(&st->i2c->dev,
					"%s gpio_direction_input(%d) ERR:%d\n",
					__func__, st->gpio_rdy, ret);
				return -ENODEV;
			}
		}
	} else {
		/* can't communicate with device without this GPIO */
		return -EPROBE_DEFER;
	}

	if (gpio_is_valid(st->gpio_sar)) {
		ret = gpio_request(st->gpio_sar, IQS_NAME);
		if (ret) {
			dev_err(&st->i2c->dev,
				"%s gpio_request(%d %s) ERR:%d\n",
				__func__, st->gpio_sar, IQS_NAME, ret);
		} else {
			/* start with SAR asserted (proximity==0) */
			ret = iqs_gpio_sar(st, 0);
			if (ret < 0) {
				dev_err(&st->i2c->dev,
					"%s gpio_sar(%d) ERR:%d\n",
					__func__, st->gpio_sar, ret);
				st->gpio_sar = -1;
			}
		}
	}
	return 0;
}

static int iqs_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct iqs_state *st;
	unsigned long irqflags;
	unsigned int i;
	unsigned int n;
	int ret;

	dev_info(&client->dev, "%s\n", __func__);
	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		dev_err(&client->dev, "%s devm_kzalloc ERR\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, st);
	st->i2c = client;
	ret = iqs_of_dt(st, client->dev.of_node);
	if (ret) {
		dev_err(&client->dev, "%s _of_dt ERR\n", __func__);
		goto iqs_probe_exit;
	}

	iqs_pm_init(st);
	ret = iqs_id_i2c(st, id->name);
	if (ret) {
		dev_err(&client->dev, "%s _id_i2c ERR\n", __func__);
		ret = -ENODEV;
		goto iqs_probe_exit;
	}

	iqs_pm(st, false);
	iqs_fn_dev.sts = &st->sts;
	iqs_fn_dev.errs = &st->errs;
	st->nvs = nvs_iio();
	if (st->nvs == NULL) {
		ret = -ENODEV;
		goto iqs_probe_exit;
	}

	n = 0;
	for (i = 0; i < IQS_DEV_N; i++) {
		ret = nvs_of_dt(client->dev.of_node, &st->cfg[i], NULL);
		if (ret == -ENODEV)
			/* the entire device has been disabled */
			goto iqs_probe_exit;

		ret = st->nvs->probe(&st->nvs_st[i], st, &client->dev,
				     &iqs_fn_dev, &st->cfg[i]);
		st->cfg[i].snsr_id = i;
		if (!ret) {
			st->prox[i].nvs_data = st->nvs_st[i];
			st->prox[i].handler = st->nvs->handler;
			n++;
		}
	}
	if (!n) {
		dev_err(&client->dev, "%s nvs_probe ERR\n", __func__);
		ret = -ENODEV;
		goto iqs_probe_exit;
	}

	INIT_DELAYED_WORK(&st->dw, iqs_work);
	if (client->irq) {
		irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = request_threaded_irq(client->irq, NULL, iqs_irq_thread,
					   irqflags, IQS_NAME, st);
		if (ret) {
			dev_err(&client->dev, "%s req_threaded_irq ERR %d\n",
				__func__, ret);
			ret = -ENOMEM;
			goto iqs_probe_exit;
		}
	}

	if (!st->os) {
		iqs_enable(st, IQS_DEV_PROX, 1);
		iqs_enable(st, IQS_DEV_TOUCH, 1);
	}
	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

iqs_probe_exit:
	iqs_remove(client);
	return ret;
}

static const struct i2c_device_id iqs_i2c_device_id[] = {
	{ IQS_NAME, 0 },
	{ IQS_NAME_IQS253, 0 },
	{ IQS_NAME_IQS263, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, iqs_i2c_device_id);

static const struct of_device_id iqs_of_match[] = {
	{ .compatible = "azoteq,iqs2x3", },
	{ .compatible = "azoteq,iqs253", },
	{ .compatible = "azoteq,iqs263", },
	{},
};

MODULE_DEVICE_TABLE(of, iqs_of_match);

static struct i2c_driver iqs_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= iqs_probe,
	.remove		= iqs_remove,
	.shutdown	= iqs_shutdown,
	.driver = {
		.name		= IQS_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(iqs_of_match),
		.pm		= &iqs_pm_ops,
	},
	.id_table	= iqs_i2c_device_id,
};
module_i2c_driver(iqs_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IQS2X3 driver");
MODULE_AUTHOR("NVIDIA Corporation");

