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


#define IQS_DRIVER_VERSION		(6)
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
#define IQS_MULTI_THRESHOLD		(5)
/* configuration */
#define IQS_POLL_DLY_MS_MIN		(1000)
#define IQS_POLL_DLY_MS_MAX		(1000)
#define IQS_POLL_DLY_MS_WATCHDOG	(30000)
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
#define IQS_CH_N			(4)


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

static unsigned char iqs263_wr_stream[] = {
	2, 0x09, 0x00, 0x00, /* data */
		 0x18, 0x40, /* mask */
	0 /* end - done - exit */
};

static unsigned char iqs253_wr_stream[] = {
	16, 0xC4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x04, /* data */
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x04, /* mask */
	0, /* end - done - exit */
};

static unsigned char iqs263_wr_events[] = {
	2, 0x09, 0x00, 0x40, /* data */
		 0x18, 0x40, /* mask */
	0 /* end - done - exit */
};

static unsigned char iqs253_wr_events[] = {
	16, 0xC4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, /* data */
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x04, /* mask */
	0 /* end - done - exit */
};

static unsigned char iqs263_wr_reseed[] = {
	1, 0x09, 0x08, /* data */
		 0x18, /* mask */
	0 /* end - done - exit */
};

static unsigned char iqs253_wr_reseed[] = {
	14, 0xC4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x08, /* data */
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x08, /* mask */
	0 /* end - done - exit */
};

static unsigned char iqs263_wr_disable[] = {
	3, 0x09, 0x00, 0x00, 0xC0, /* data */
		 0xFF, 0xFF, 0xFF, /* mask */
	0 /* end - done - exit */
};

static unsigned char iqs253_wr_disable[] = {
	/* TODO */
	0 /* end - done - exit */
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

#define IQS_DT_INIT_N			(128) /* max DT init bytes */
#define IQS_DT_ABLE_N			(64) /* max DT en/dis-able bytes */
#define IQS_DT_EVNT_N			(32) /* max DT en/dis-able bytes */
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
	struct iqs_hal_iom event_mode;
	struct iqs_hal_iom ati_partial;
	struct iqs_hal_iom active_ch;
	struct iqs_hal_iom multi_comp;
	struct iqs_hal_iom multi_sens;
	struct iqs_hal_iom touch_prx;
	struct iqs_hal_iom touch_tch;
	struct iqs_hal_iom count_prx;
	struct iqs_hal_iom count_tch;
	struct iqs_hal_iom thresh_prx;
	struct iqs_hal_iom thresh_tch;
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
	.event_mode			= {
		.hal_i			= 9,
		.offset			= 1,
		.mask			= 0x40,
	},
	.ati_partial			= {
		.hal_i			= 9,
		.offset			= 0,
		.mask			= 0x40,
	},
	.active_ch			= {
		.hal_i			= 0x0D,
		.offset			= 0,
		.mask			= 0x0F,
	},
	.multi_comp			= {
		.hal_i			= 7,
		.offset			= 0,
		.mask			= 0x0F,
	},
	.multi_sens			= {
		.hal_i			= 7,
		.offset			= 0,
		.mask			= 0x30,
	},
	.touch_prx			= {
		.hal_i			= 3,
		.offset			= 0,
		.mask			= 0x01,
	},
	.touch_tch			= {
		.hal_i			= 3,
		.offset			= 0,
		.mask			= 0xE0,
	},
	.count_prx			= {
		.hal_i			= 4,
		.offset			= 0,
		.mask			= 1,
	},
	.count_tch			= {
		.hal_i			= 4,
		.offset			= 4,
		.mask			= 3,
	},
	.thresh_prx			= {
		.hal_i			= 10,
		.offset			= 0,
		.mask			= 1,
	},
	.thresh_tch			= {
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
	.thresh_prx			= {
		.hal_i			= 10,
		.offset			= 0,
		.mask			= 1,
	},
	.thresh_tch			= {
		.hal_i			= 10,
		.offset			= 1,
		.mask			= 3,
	},
	.count_prx			= {
		.hal_i			= 4,
		.offset			= 0,
		.mask			= 1,
	},
	.count_tch			= {
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
	bool irq_first;			/* first interrupt flag */
	bool reseed;			/* do reseed flag */
	int op_i;			/* operational index */
	int op_read_n;			/* operational register read count */
	int op_read_reg[IQS_DEV_N + 2];	/* operational registers to read */
	unsigned int os;		/* OS options */
	unsigned int stream;		/* configured for stream mode only */
	unsigned int wd_to_ms;		/* watchdog timeout ms */
	unsigned int i2c_retry;
	unsigned int gpio_rdy_retry;	/* GPIO RDY assert timeout */
	int gpio_rdy;
	int gpio_sar;
	int gpio_sar_val;
	unsigned int sar_assert_pol;	/* sar assert polarity */
	unsigned int msg_n;		/* I2C transaction count */
	struct i2c_msg msg[IQS_MSG_N];	/* max possible I2C transactions */
	const struct iqs_hal *hal_tbl;	/* HAL register table */
	unsigned int hal_tbl_n;		/* HAL register table count */
	const struct iqs_hal_bit *hal_bit;
	unsigned char *wr_disable;
	unsigned char *wr_stream;
	unsigned char *wr_events;
	unsigned char *wr_reseed;
	unsigned char dt_init263[IQS_DT_INIT_N];
	unsigned char dt_init253[IQS_DT_INIT_N];
	unsigned char dt_en_prx263[IQS_DT_ABLE_N];
	unsigned char dt_en_tch263[IQS_DT_ABLE_N];
	unsigned char dt_en_prx253[IQS_DT_ABLE_N];
	unsigned char dt_en_tch253[IQS_DT_ABLE_N];
	unsigned char dt_dis_prx263[IQS_DT_ABLE_N];
	unsigned char dt_dis_tch263[IQS_DT_ABLE_N];
	unsigned char dt_dis_prx253[IQS_DT_ABLE_N];
	unsigned char dt_dis_tch253[IQS_DT_ABLE_N];
	unsigned char dt_evnt263[IQS_DT_EVNT_N];
	unsigned char dt_evnt253[IQS_DT_EVNT_N];
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
				st->nvs->nvs_mutex_lock(st->nvs_st[i]);
		}
	}
}

static void iqs_mutex_unlock(struct iqs_state *st)
{
	unsigned int i;

	if (st->nvs) {
		for (i = 0; i < IQS_DEV_N; i++) {
			if (st->nvs_st[i])
				st->nvs->nvs_mutex_unlock(st->nvs_st[i]);
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
		st->irq_first = true;
		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s IRQ enabled\n", __func__);
	}
}

static int iqs_gpio_sar(struct iqs_state *st, int prox)
{
	int gpio_sar_val;
	int ret = -EINVAL;

	if (st->gpio_sar >= 0) {
		gpio_sar_val = st->sar_assert_pol;
		gpio_sar_val ^= prox;
		if (st->gpio_sar_val != gpio_sar_val) {
			ret = gpio_direction_output(st->gpio_sar,
						    gpio_sar_val);
			if (ret) {
				dev_err(&st->i2c->dev,
					"%s prox=%d gpio_sar %d=%d  err=%d\n",
					 __func__, prox, st->gpio_sar,
					 gpio_sar_val, ret);
			} else {
				st->gpio_sar_val = gpio_sar_val;
				if (st->sts & NVS_STS_SPEW_MSG)
					dev_info(&st->i2c->dev,
						 "%s prox=%d gpio_sar %d=%d\n",
						 __func__, prox, st->gpio_sar,
						 gpio_sar_val);
			}
		}
	}
	return ret;
}

static int iqs_gpio_rdy(struct iqs_state *st)
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
				ret = iqs_gpio_rdy(st);
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
			if (i || ret)
				dev_info(&st->i2c->dev,
					 "%s retries=%u err=%d\n",
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

static int iqs_wr(struct iqs_state *st, unsigned char *wr)
{
	u8 ndx = 0;
	unsigned int i;
	unsigned int j;
	unsigned int k;
	unsigned char len;
	unsigned char reg;
	int hal_i;
	int ret = 0;

	if (!st->hal_tbl_n)
		/* exit if HAL not initialized */
		return -EINVAL;

	i = 0;
	while (wr[i] != 0) { /* while a length */
		len = wr[i];
		i++;
		if (len == 0xFF) {
			/* if length == FF then do an I2C write now */
			ret |= iqs_i2c(st);
			if (wr[i])
				mdelay(wr[i]);
			i++;
			continue;
		}

		/* get the register */
		reg = wr[i];
		i++;
		/* find the register and reg cache index in the hal table */
		for (hal_i = st->hal_tbl_n - 1; hal_i > 0; hal_i--) {
			if (st->hal_tbl[hal_i].reg == reg) {
				if (st->hal_tbl[hal_i].len >= len) {
					ndx = st->hal_tbl[hal_i].ndx + 1;
					break;
				}
			}
		}
		if (hal_i) {
			/* if register and index found, mask data to cache */
			for (j = 0; j < len; j++) {
				k = ndx + j;
				st->rc[k] &= ~wr[i + len];
				st->rc[k] |= wr[i];
				i++;
			}
			ret |= iqs_i2c_wr(st, hal_i, len);
			i += len;
		} else {
			/* if register not found we're lost and need to exit */
			return -EFAULT;
		}
	}

	return ret;
};

static int iqs_write(struct iqs_state *st, unsigned char *wr)
{
	int ret;

	ret = iqs_wr(st, wr);
	if (st->msg_n && !ret)
		ret = iqs_i2c(st);
	return ret;
}

static void iqs_op_rd(struct iqs_state *st)
{
	bool prox_binary = false;
	bool prox_full = false;
	unsigned int i;

	st->op_read_n = 0;
	/* add multi_comp if ATI partial is enabled */
	i = st->hal_tbl[st->hal_bit->ati_partial.hal_i].ndx + 1;
	i += st->hal_bit->ati_partial.offset;
	if (st->rc[i] & st->hal_bit->ati_partial.mask) {
		st->op_read_reg[st->op_read_n] = st->hal_bit->multi_comp.hal_i;
		st->op_read_n++;
	}
	/* always test for device reset */
	st->op_read_reg[st->op_read_n] = st->hal_bit->sysflag_reset.hal_i;
	st->op_read_n++;
	/* read either binary data or full counts */
	for (i = 0; i < IQS_DEV_N; i++) {
		if (st->enabled & (1 << i)) {
			if (st->prox[i].proximity_binary_hw) {
				if (!prox_binary) {
					st->op_read_reg[st->op_read_n] =
						 st->hal_bit->touch_prx.hal_i;
					prox_binary = true;
					st->op_read_n++;
				}
			} else if (!prox_full) {
				st->op_read_reg[st->op_read_n] =
						 st->hal_bit->count_prx.hal_i;
				prox_full = true;
				st->op_read_n++;
			}
		}
	}
	st->op_i = st->op_read_n; /* force new read cycle */
}

static int iqs_init(struct iqs_state *st)
{
	unsigned char *wr = st->dt_init263;
	int ret = 0;

	if (st->dev_id == IQS_DEVID_IQS253)
		wr = st->dt_init253;
	if (st->hal_tbl_n)
		/* only if HAL initialized */
		ret = iqs_write(st, wr);
	return ret;
}

static int iqs_en(struct iqs_state *st, int snsr_id)
{
	unsigned char *wr;
	int ret;

	if (snsr_id == IQS_DEV_PROX) {
		if (st->dev_id == IQS_DEVID_IQS253)
			wr = st->dt_en_prx253;
		else
			wr = st->dt_en_prx263;
	} else if (snsr_id == IQS_DEV_TOUCH) {
		if (st->dev_id == IQS_DEVID_IQS253)
			wr = st->dt_en_tch253;
		else
			wr = st->dt_en_tch263;
	} else {
		return -EINVAL;
	}

	/* if sensor enabled */
	ret = iqs_write(st, wr);
	if (!ret)
		ret = nvs_proximity_enable(&st->prox[snsr_id]);
	return ret;
}

static int iqs_dis(struct iqs_state *st, int snsr_id)
{
	unsigned char *wr;
	int ret = 0;

	if (snsr_id == IQS_DEV_PROX) {
		if (st->dev_id == IQS_DEVID_IQS253)
			wr = st->dt_dis_prx253;
		else
			wr = st->dt_dis_prx263;
	} else if (snsr_id == IQS_DEV_TOUCH) {
		if (st->dev_id == IQS_DEVID_IQS253)
			wr = st->dt_dis_tch253;
		else
			wr = st->dt_dis_tch263;
	} else if (snsr_id < 0) {
		wr = st->wr_disable;
	} else {
		return -EINVAL;
	}

	if (st->hal_tbl_n)
		/* only if HAL initialized */
		ret = iqs_write(st, wr);
	return ret;
}

static int iqs_pm(struct iqs_state *st, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
				       ARRAY_SIZE(iqs_vregs));
		if (ret > 0)
			mdelay(IQS_HW_DELAY_MS);
		ret = iqs_init(st);
	} else {
		ret = nvs_vregs_sts(st->vreg, ARRAY_SIZE(iqs_vregs));
		if ((ret < 0) || (ret == ARRAY_SIZE(iqs_vregs))) {
			ret = iqs_dis(st, -1);
		} else if (ret > 0) {
			nvs_vregs_enable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(iqs_vregs));
			mdelay(IQS_HW_DELAY_MS);
			ret = iqs_init(st);
			ret |= iqs_dis(st, -1);
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
		i = st->hal_tbl[st->hal_bit->touch_tch.hal_i].ndx + 1;
		i += st->hal_bit->touch_tch.offset;
		hw = !!(st->rc[i] & st->hal_bit->touch_tch.mask);
	} else {
		i = st->hal_tbl[st->hal_bit->count_tch.hal_i].ndx + 1;
		i += st->hal_bit->count_tch.offset;
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

static int iqs_rd_proximity(struct iqs_state *st, s64 ts)
{
	u16 hw;
	unsigned int i;
	int ret;

	if (st->prox[IQS_DEV_PROX].proximity_binary_hw) {
		i = st->hal_tbl[st->hal_bit->touch_prx.hal_i].ndx + 1;
		i += st->hal_bit->touch_prx.offset;
		/* reverse polarity for Android (0=close 1=far) */
		hw = !(st->rc[i] & st->hal_bit->touch_prx.mask);
	} else {
		i = st->hal_tbl[st->hal_bit->count_prx.hal_i].ndx + 1;
		i += st->hal_bit->count_prx.offset;
		hw = (u16)st->rc[i];
	}
	if (st->sts & NVS_STS_SPEW_DATA)
		dev_info(&st->i2c->dev, "prox hw %hu %lld  diff=%d %lldns\n",
			 hw, ts, hw - st->prox[IQS_DEV_PROX].hw,
			 ts - st->prox[IQS_DEV_PROX].timestamp);
	st->prox[IQS_DEV_PROX].hw = hw;
	st->prox[IQS_DEV_PROX].timestamp = ts;
	ret = nvs_proximity_read(&st->prox[IQS_DEV_PROX]);
	if (st->prox[IQS_DEV_PROX].proximity_binary_hw)
		/* TODO: Expect the PO pin used for proximity_binary_hw.
		 *       Use a proximity threshold for SAR GPIO so that
		 *       proximity doesn't have to be in HW binary mode.
		 */
		iqs_gpio_sar(st, st->prox[IQS_DEV_PROX].proximity);
	return ret;
}

static int iqs_rd(struct iqs_state *st)
{
	s64 ts;
	unsigned int i;
	unsigned int k;
	unsigned int ch;
	int mc;
	int ret = 0;

#ifdef IQS_I2C_M_NO_RD_ACK
	/* I2C message stacking */
	for (i = 0; i < st->op_read_n; i++)
		iqs_i2c_rd(st, st->op_read_reg[i], 0);
	ret = iqs_i2c(st);
#else
	st->op_i++;
	if (st->op_i >= st->op_read_n) {
		/* restart read cycle */
		st->op_i = 0;
		if (!st->stream)
			/* enter stream mode on first I2C transaction */
			iqs_wr(st, st->wr_stream);
		if (st->dev_id == IQS_DEVID_IQS253)
			iqs_wr(st, st->dt_evnt253);
		else
			iqs_wr(st, st->dt_evnt263);
	}
	if ((st->op_i == st->op_read_n - 1) && !st->stream)
		iqs_wr(st, st->wr_events); /* event mode at end of reads */
	if (st->reseed) {
		iqs_wr(st, st->wr_reseed);
		st->reseed = false;
	}
	iqs_i2c_rd(st, st->op_read_reg[st->op_i], 0);
	ret = iqs_i2c(st);
#endif /* IQS_I2C_M_NO_RD_ACK */
	if (!ret) {
		/* test for device reset */
		i = st->hal_bit->sysflag_reset.hal_i;
		if (st->op_read_reg[st->op_i] == i) {
			i = st->hal_tbl[i].ndx + 1;
			i += st->hal_bit->sysflag_reset.offset;
			if (st->rc[i] & st->hal_bit->sysflag_reset.mask) {
				iqs_err(st);
				iqs_init(st);
				for (i = 0; i < IQS_DEV_N; i++) {
					if (st->enabled & (1 << i))
						iqs_en(st, i);
				}
				iqs_op_rd(st);
				return RET_POLL_NEXT;
			}
		}

		i = st->hal_bit->multi_comp.hal_i;
		if (st->op_read_reg[st->op_i] == i) {
			/* check if reseed needed */
			i = st->hal_bit->active_ch.hal_i;
			ch = st->hal_tbl[i].ndx + 1;
			for (i = 0; i < IQS_CH_N; i++) {
				if (st->rc[ch] & (1 << i)) {
					mc = st->hal_bit->multi_comp.hal_i;
					mc = st->hal_tbl[mc].ndx + 1;
					mc += i;
					mc = st->rc[mc];
					mc &= st->hal_bit->multi_comp.mask;
					if (i)
						k = IQS_DEV_TOUCH;
					else
						k = IQS_DEV_PROX;
					if (mc > st->cfg[k].thresh_hi) {
						st->reseed = true;
						break;
					}
				}
			}
		}
		/* read data */
		ts = iqs_get_time_ns();
		if (st->enabled & (1 << IQS_DEV_PROX))
			ret |= iqs_rd_proximity(st, ts);
		if (st->enabled & (1 << IQS_DEV_TOUCH))
			ret |= iqs_rd_touch(st, ts);
	}
	if (st->stream) {
		if (ret != RET_NO_CHANGE) {
			/* keep IRQ enabled if anything but no change */
			ret = RET_HW_UPDATE;
		} else if (st->op_i == st->op_read_n - 1) {
			/* throttle IRQ at end of read cycle */
			iqs_disable_irq(st);
			ret = RET_POLL_NEXT;
		}
	}
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
	unsigned int i;
	unsigned int ms;
	int ret;

	iqs_mutex_lock(st);
	if (st->enabled) {
		ms = st->wd_to_ms;
#ifdef IQS_I2C_M_NO_RD_ACK
		ret = iqs_rd(st);
		if (ret > RET_POLL_NEXT)
			iqs_enable_irq(st);
		else
			ms = iqs_polldelay(st);
#else
		if (st->irq_dis) {
			/* if IRQ disabled then in irq throttle mode */
			iqs_enable_irq(st); /* IRQ driven mode */
		} else {
			i = st->hal_tbl[st->hal_bit->event_mode.hal_i].ndx + 1;
			if (st->irq_first && !(st->rc[i] &
					       st->hal_bit->event_mode.mask)) {
				/* if first IRQ and in streaming mode then skip
				 * read to sync to the rdy signal.
				 */
				st->irq_first = false;
			} else {
				ret = iqs_rd(st);
				if (ret > RET_POLL_NEXT)
					iqs_enable_irq(st);
				else
					ms = iqs_polldelay(st);
			}
		}
#endif /* IQS_I2C_M_NO_RD_ACK */
		/* always start a delayed work thread as a watchdog */
		mod_delayed_work(system_freezable_wq, &st->dw,
				 msecs_to_jiffies(ms));
		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s work delay=%ums\n",
				 __func__, ms);
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
	unsigned int i;
	int ret = 0;

	if (snsr_id >= 0) {
		ret = iqs_dis(st, snsr_id);
		if (!ret)
			st->enabled &= ~(1 << snsr_id);
		if (st->enabled)
			disable = false;
	} else {
		for (i = 0; i < IQS_DEV_N; i++) {
			if (st->enabled & (1 << i))
				iqs_dis(st, snsr_id);
		}
	}
	if (disable) {
		iqs_disable_irq(st);
		if (st->dw.work.func)
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
				iqs_op_rd(st);
				mod_delayed_work(system_freezable_wq, &st->dw,
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

	if (st->os || st->sts & NVS_STS_SHUTDOWN)
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
		hal_i = st->hal_bit->thresh_tch.hal_i;
		i = st->hal_bit->thresh_tch.offset + 1;
		n = i + st->hal_bit->thresh_tch.mask;
	} else {
		hal_i = st->hal_bit->thresh_prx.hal_i;
		i = st->hal_bit->thresh_prx.offset + 1;
		n = i + st->hal_bit->thresh_prx.mask;
	}
	for (; i < n; i++) {
		st->rc[st->hal_tbl[hal_i].ndx + i] = thresh & 0xFF;
		thresh >>= 8;
	}
	ret = iqs_i2c_write(st, hal_i, 0);
	if (!ret)
		st->cfg[snsr_id].thresh_lo = thresh;
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
		if (st->gpio_sar >= 0) {
			ret = gpio_direction_output(st->gpio_sar, val);
			if (!ret)
				st->gpio_sar_val = val;
		}
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

static ssize_t iqs_nvs_dbg_db(struct iqs_state *st, char *buf, ssize_t t,
			      unsigned char *db)
{
	unsigned int i;
	unsigned int j;
	unsigned int n;

	i = 0;
	while (db[i]) {
		n = db[i];
		i++;
		if (n == 0xFF) {
			t += sprintf(buf + t, "flush write and mdelay=%hhu\n",
				     db[i]);
			i++;
			continue;
		}

		t += sprintf(buf + t, "len=%x reg=%x data/mask=", n, db[i]);
		i++;
		for (j = 0; j < n; j++)
			t += sprintf(buf + t, "%x/%x ",
				     db[i + j], db[i + j + n]);
		t += sprintf(buf + t, "\n");
		i += (n << 1);
	}
	return t;
}

static int iqs_nvs_read(void *client, int snsr_id, char *buf)
{
	struct iqs_state *st = (struct iqs_state *)client;
	ssize_t t;

	t = sprintf(buf, "IQS driver v. %u\n", IQS_DRIVER_VERSION);
	t += sprintf(buf + t, "os_options=%x\n", st->os);
	t += sprintf(buf + t, "stream_mode=%x\n", st->stream);
	t += sprintf(buf + t, "watchdog_timeout_ms=%u\n", st->wd_to_ms);
	t += sprintf(buf + t, "i2c_retry=%u\n", st->i2c_retry);
	t += sprintf(buf + t, "gpio_rdy_retry=%u\n", st->gpio_rdy_retry);
	if (st->gpio_rdy < 0)
		t += sprintf(buf + t, "NO gpio_rdy\n");
	else
		t += sprintf(buf + t, "gpio_rdy %d=%d\n",
			     st->gpio_rdy, gpio_get_value(st->gpio_rdy));
	if (st->gpio_sar < 0)
		t += sprintf(buf + t, "NO gpio_sar\n");
	else
		t += sprintf(buf + t, "gpio_sar %d=%d\n",
			     st->gpio_sar, gpio_get_value(st->gpio_sar));
	t += sprintf(buf + t, "irq=%d\n", st->i2c->irq);
	t += sprintf(buf + t, "irq_disable=%x\n", st->irq_dis);
	t += sprintf(buf + t, "SAR_proximity_binary_hw=%x\n",
		     st->prox[IQS_DEV_PROX].proximity_binary_hw);
	t += sprintf(buf + t, "touch_proximity_binary_hw=%x\n",
		     st->prox[IQS_DEV_TOUCH].proximity_binary_hw);
	t += sprintf(buf + t, "IQS263 initialization:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_init263);
	t += sprintf(buf + t, "IQS263 proximity enable:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_en_prx263);
	t += sprintf(buf + t, "IQS263 touch enable:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_en_tch263);
	t += sprintf(buf + t, "IQS263 proximity disable:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_dis_prx263);
	t += sprintf(buf + t, "IQS263 touch disable:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_dis_tch263);
	t += sprintf(buf + t, "IQS263 event:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_evnt263);
	t += sprintf(buf + t, "IQS253 initialization:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_init253);
	t += sprintf(buf + t, "IQS253 proximity enable:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_en_prx253);
	t += sprintf(buf + t, "IQS253 touch enable:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_en_tch253);
	t += sprintf(buf + t, "IQS253 proximity disable:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_dis_prx253);
	t += sprintf(buf + t, "IQS253 touch disable:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_dis_tch253);
	t += sprintf(buf + t, "IQS253 event:\n");
	t += iqs_nvs_dbg_db(st, buf, t, st->dt_evnt253);
	return t;
}

static struct nvs_fn_dev iqs_fn_dev = {
	.enable				= iqs_enable_os,
	.batch				= iqs_batch,
	.thresh_lo			= iqs_thresh,
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
	if (st->nvs) {
		for (i = 0; i < IQS_DEV_N; i++) {
			if (st->nvs_st[i])
				ret |= st->nvs->suspend(st->nvs_st[i]);
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

	if (st->nvs) {
		for (i = 0; i < IQS_DEV_N; i++) {
			if (st->nvs_st[i])
				ret |= st->nvs->resume(st->nvs_st[i]);
		}
	}
	st->sts &= ~NVS_STS_SUSPEND;
	if (!st->os) {
		for (i = 0; i < IQS_DEV_N; i++) {
			if (!(st->enabled & (1 << i)))
				iqs_enable(st, i, 1);
		}
	}
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
			st->hal_tbl_n = 0; /* disable PM I2C */
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
		st->wr_stream = iqs263_wr_stream;
		st->wr_events = iqs263_wr_events;
		st->wr_reseed = iqs263_wr_reseed;
		break;

	case IQS_DEVID_IQS253:
		BUG_ON(IQS253_MSG_N != ARRAY_SIZE(iqs253_hal_tbl));
		iqs_id_part(st, IQS_NAME_IQS253);
		st->hal_tbl = iqs253_hal_tbl;
		st->hal_tbl_n = ARRAY_SIZE(iqs253_hal_tbl);
		st->hal_bit = &iqs253_hal_bit;
		st->wr_disable = iqs253_wr_disable;
		st->wr_stream = iqs253_wr_stream;
		st->wr_events = iqs253_wr_events;
		st->wr_reseed = iqs253_wr_reseed;
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
	.flags				= SENSOR_FLAG_ON_CHANGE_MODE |
					  SENSOR_FLAG_WAKE_UP,
	.thresh_lo			= IQS_PROX_THRESHOLD,
	.thresh_hi			= IQS_MULTI_THRESHOLD,
};

static int iqs_of_dt_db(struct iqs_state *st, struct device_node *dn,
			char *name, unsigned char *dt_db, int db_n)
{
	char str[16];
	const char *charp;
	unsigned int limit = IQS_MSG_N;
	unsigned int i;
	int n;
	int lenp;
	int ret;

	n = 0;
	for (i = 0; i < limit; i++) {
		ret = sprintf(str, "%s_%u", name, i);
		if (ret <= 0) {
			dev_err(&st->i2c->dev, "%s sprintf(%s_%u)\n",
				__func__, name, i);
			return -ENODEV;
		}

		charp = of_get_property(dn, str, &lenp);
		if (charp) {
			if (lenp < (db_n - n)) {
				memcpy(&dt_db[n], charp, lenp);
				if (dt_db[n] == 0xFF)
					/* flush is done so limit extended */
					limit = i + IQS_MSG_N;
				n += lenp;
			} else {
				dev_err(&st->i2c->dev, "%s ERR: NOMEM @ %s\n",
					__func__, str);
				return -ENOMEM;
			}
		} else {
			dt_db[n] = 0; /* terminate byte stream */
			break;
		}
	}

	return 0;
}

static int iqs_of_dt(struct iqs_state *st, struct device_node *dn)
{
	unsigned int i;
	int ret;

	/* just test if global disable */
	ret = nvs_of_dt(dn, NULL, NULL);
	if (ret == -ENODEV)
		return -ENODEV;

	/* default device specific parameters */
	for (i = 0; i < IQS_DEV_N; i++) {
		memcpy(&st->cfg[i], &iqs_cfg_dflt, sizeof(st->cfg[0]));
		st->cfg[i].name = iqs_sensor_cfg_name[i];
		st->prox[i].cfg = &st->cfg[i];
		st->prox[i].hw_mask = 0xFFFF;
		st->prox[i].proximity_binary_hw = true;
		nvs_proximity_of_dt(&st->prox[i], dn, st->cfg[i].name);
	}
	st->wd_to_ms = IQS_POLL_DLY_MS_WATCHDOG;
	st->i2c_retry = IQS_I2C_RETRY_N;
	st->gpio_rdy_retry = IQS_RDY_RETRY_N;
	st->gpio_rdy = -1;
	st->gpio_sar = -1;
	/* device tree parameters */
	if (dn) {
		/* device specific parameters */
		of_property_read_u32(dn, "os_options", &st->os);
		of_property_read_u32(dn, "stream_mode", &st->stream);
		of_property_read_u32(dn, "watchdog_timeout_ms", &st->wd_to_ms);
		of_property_read_u32(dn, "i2c_retry", &st->i2c_retry);
		of_property_read_u32(dn, "gpio_rdy_retry",
				     &st->gpio_rdy_retry);
		of_property_read_u32(dn, "sar_assert_polarity",
				     &st->sar_assert_pol);
		st->sar_assert_pol = !!st->sar_assert_pol;
		st->gpio_rdy = of_get_named_gpio(dn, "gpio_rdy", 0);
		st->gpio_sar = of_get_named_gpio(dn, "gpio_sar", 0);
		ret = iqs_of_dt_db(st, dn, "263init", st->dt_init263,
				   sizeof(st->dt_init263));
		ret |= iqs_of_dt_db(st, dn, "263en_prox", st->dt_en_prx263,
				    sizeof(st->dt_en_prx263));
		ret |= iqs_of_dt_db(st, dn, "263en_touch", st->dt_en_tch263,
				    sizeof(st->dt_en_tch263));
		ret |= iqs_of_dt_db(st, dn, "263dis_prox", st->dt_dis_prx263,
				    sizeof(st->dt_dis_prx263));
		ret |= iqs_of_dt_db(st, dn, "263dis_touch", st->dt_dis_tch263,
				    sizeof(st->dt_dis_tch263));
		ret |= iqs_of_dt_db(st, dn, "263event", st->dt_evnt263,
				    sizeof(st->dt_evnt263));
		ret |= iqs_of_dt_db(st, dn, "253init", st->dt_init253,
				    sizeof(st->dt_en_tch253));
		ret |= iqs_of_dt_db(st, dn, "253en_prox", st->dt_en_prx253,
				    sizeof(st->dt_en_tch253));
		ret |= iqs_of_dt_db(st, dn, "253en_touch", st->dt_en_tch253,
				    sizeof(st->dt_en_tch253));
		ret |= iqs_of_dt_db(st, dn, "253dis_prox", st->dt_dis_prx253,
				    sizeof(st->dt_dis_tch253));
		ret |= iqs_of_dt_db(st, dn, "253dis_touch", st->dt_dis_tch253,
				    sizeof(st->dt_dis_tch253));
		ret |= iqs_of_dt_db(st, dn, "253event", st->dt_evnt253,
				    sizeof(st->dt_evnt253));
		if (ret)
			return ret;
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
			st->gpio_sar_val = -1;
			iqs_gpio_sar(st, 0);
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
		if (ret == -ENODEV) {
			dev_info(&client->dev, "%s DT disabled\n", __func__);
		} else {
			dev_err(&client->dev, "%s _of_dt ERR\n", __func__);
			ret = -ENODEV;
		}
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
		nvs_of_dt(client->dev.of_node, &st->cfg[i], NULL);
		ret = st->nvs->probe(&st->nvs_st[i], st, &client->dev,
				     &iqs_fn_dev, &st->cfg[i]);
		st->cfg[i].snsr_id = i;
		if (!ret) {
			st->prox[i].nvs_st = st->nvs_st[i];
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
	if (st->gpio_rdy > 0)
		st->i2c->irq = gpio_to_irq(st->gpio_rdy);
	if (client->irq) {
		irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		for (i = 0; i < IQS_DEV_N; i++) {
			if (st->cfg[i].snsr_id >= 0) {
				if (st->cfg[i].flags & SENSOR_FLAG_WAKE_UP)
					irqflags |= IRQF_NO_SUSPEND;
			}
		}
		ret = request_threaded_irq(client->irq, NULL, iqs_irq_thread,
					   irqflags, IQS_NAME, st);
		if (ret) {
			dev_err(&client->dev, "%s req_threaded_irq ERR %d\n",
				__func__, ret);
			ret = -ENOMEM;
			goto iqs_probe_exit;
		}
	}

	iqs_mutex_lock(st);
	if (st->os) {
		iqs_disable(st, -1);
	} else {
		ret = 0;
		if (st->nvs_st[IQS_DEV_PROX])
			ret |= iqs_enable(st, IQS_DEV_PROX, 1);
		if (st->nvs_st[IQS_DEV_TOUCH])
			ret |= iqs_enable(st, IQS_DEV_TOUCH, 1);
		if (ret) {
			iqs_err(st);
			/* if an error then switch to OS controlled */
			dev_err(&client->dev,
				"%s auto enable ERR=%d (now OS controlled)\n",
				__func__, ret);
			st->os = 3;
			iqs_disable(st, -1);
		}
	}
	iqs_mutex_unlock(st);
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

