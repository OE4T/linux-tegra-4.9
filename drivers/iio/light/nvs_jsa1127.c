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
/* The NVS implementation of scan_elements enable/disable works as follows
 * (See NVS HAL for further explaination):
 * To enable, the NVS HAL will:
 * 1. Disable buffer
 * 2. Enable channels
 * 3. Calculate buffer alignments based on enabled channels
 * 4. Enable buffer
 * It is expected that the NVS kernel driver will detect the channels enabled
 * and enable the device using the IIO iio_buffer_setup_ops.
 * To disable, the NVS HAL will:
 * 1. Disable buffer
 * 2. Disable channels
 * 3. Calculate buffer alignments based on enabled channels
 * 4. If (one or more channels are enabled)
 *        4a. Enable buffer
 *    else
 *        4b. Disable master enable
 * It is expected that the master enable will be enabled as part of the
 * iio_buffer_setup_ops.
 * The NVS sysfs attribute for the master enable is "enable" without any
 * channel name.
 */
/* The NVS HAL will use the IIO scale and offset sysfs attributes to modify the
 * data using the following formula: (data * scale) + offset
 * A scale value of 0 disables scale.
 * A scale value of 1 puts the NVS HAL into calibration mode where the scale
 * and offset are read everytime the data is read to allow realtime calibration
 * of the scale and offset values to be used in the device tree parameters.
 * Keep in mind the data is buffered but the NVS HAL will display the data and
 * scale/offset parameters in the log.  See calibration steps below.
 */
/* NVS light/proximity drivers have two calibration mechanisms:
 * Method 1 (preferred):
 * This method uses interpolation and requires a low and high uncalibrated
 * value along with the corresponding low and high calibrated values.  The
 * uncalibrated values are what is read from the sensor in the steps below.
 * The corresponding calibrated values are what the correct value should be.
 * All values are programmed into the device tree settings.
 * 1. Read scale sysfs attribute.  This value will need to be written back.
 * 2. Disable device.
 * 3. Write 1 to the scale sysfs attribute.
 * 4. Enable device.
 * 5. The NVS HAL will announce in the log that calibration mode is enabled and
 *    display the data along with the scale and offset parameters applied.
 * 6. Write the scale value read in step 1 back to the scale sysfs attribute.
 * 7. Put the device into a state where the data read is a low value.
 * 8. Note the values displayed in the log.  Separately measure the actual
 *    value.  The value from the sensor will be the uncalibrated value and the
 *    separately measured value will be the calibrated value for the current
 *    state (low or high values).
 * 9. Put the device into a state where the data read is a high value.
 * 10. Repeat step 8.
 * 11. Enter the values in the device tree settings for the device.  Both
 *     calibrated and uncalibrated values will be the values before scale and
 *     offset are applied.
 *     For example, a light sensor has the following device tree parameters:
 *     light_uncalibrated_lo
 *     light_calibrated_lo
 *     light_uncalibrated_hi
 *     light_calibrated_hi
 *     The proximity sensor parameters are:
 *     proximity_uncalibrated_lo
 *     proximity_calibrated_lo
 *     proximity_uncalibrated_hi
 *     proximity_calibrated_hi
 *
 * Method 2:
 * 1. Disable device.
 * 2. Write 1 to the scale sysfs attribute.
 * 3. Enable device.
 * 4. The NVS HAL will announce in the log that calibration mode is enabled and
 *    display the data along with the scale and offset parameters applied.
 * 5. Write to scale and offset sysfs attributes as needed to get the data
 *    modified as desired.
 * 6. Disabling the device disables calibration mode.
 * 7. Set the new scale and offset parameters in the device tree:
 *    <IIO channel>_scale_val
 *    <IIO channel>_scale_val2
 *    <IIO channel>_offset_val
 *    <IIO channel>_offset_val2
 *    The values are in IIO IIO_VAL_INT_PLUS_MICRO format.
 */

/* This driver was developed without a part available so there are a few
 * TODOs:
 * - see if this driver works - start with st->hw_it = true
 * - dynamic integration time (hw integration time == false)
 * - dynamic range and resolution
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>

#define JSA_VERSION_DRIVER		(101)
#define JSA_VENDOR			"SolteamOpto"
#define JSA_NAME			"jsa1127"
/* setting _REPORT_N to 2 causes an extra reading after crossing the threshold
 * allowing a more accurate settled reported value.
 */
#define JSA_REPORT_N			(2)
#define JSA_LIGHT_VERSION		(1)
#define JSA_LIGHT_MAX_RANGE_MICRO	(0)
#define JSA_LIGHT_RESOLUTION_IVAL	(0)
#define JSA_LIGHT_RESOLUTION_MICRO	(10000)
#define JSA_LIGHT_SCALE_IVAL		(0)
#define JSA_LIGHT_SCALE_MICRO		(10000)
#define JSA_LIGHT_OFFSET_IVAL		(0)
#define JSA_LIGHT_OFFSET_MICRO		(0)
#define JSA_LIGHT_MILLIAMP		"0.09"
#define JSA_POLL_DELAY_MS_DFLT		(2000)
#define JSA_THRESHOLD_LUX_DFLT		(5000) /* lux change outside 100 */
/* HW registers */
#define JSA_CMD_SHUTDOWN		(0x80)
#define JSA_CMD_EN_CONT			(0x0C)
#define JSA_CMD_EN			(0x04)
#define JSA_CMD_START			(0x08)
#define JSA_CMD_STOP			(0x30)
#define JSA_CMD_MASK			(0x3F)
#define JSA_VAL_VALID			(15)
#define JSA_HW_DELAY_MS			(60)
/* _buf_push expects this scan order */
#define JSA_SCAN_LIGHT			0
#define JSA_SCAN_TIMESTAMP		1
/* debug flags */
#define JSA_DBG_SPEW_MSG		(1 << 0)
#define JSA_DBG_SPEW_LIGHT		(1 << 1)
#define JSA_DBG_SPEW_LIGHT_POLL		(1 << 2)
#define JSA_DBG_VAL_LIGHT		(1 << 3)

enum JSA_ATTR {
	JSA_ATTR_PART,
	JSA_ATTR_VENDOR,
	JSA_ATTR_VERSION,
	JSA_ATTR_MILLIAMP,
	JSA_ATTR_ENABLE,
	JSA_ATTR_THRESH_LO,
	JSA_ATTR_THRESH_HI,
};

enum JSA_INFO {
	JSA_INFO_DATA = 0,
	JSA_INFO_VER,
	JSA_INFO_ERRS,
	JSA_INFO_REGS,
	JSA_INFO_DBG,
	JSA_INFO_LIGHT_SPEW,
	JSA_INFO_LIGHT_POLL_SPEW,
	JSA_INFO_LIMIT_MAX,
};

/* regulator names in order of powering on */
static char *jsa_vregs[] = {
	"vdd",
};

static unsigned short jsa_i2c_addrs[] = {
	0x29,
	0x39,
	0x44,
};

struct jsa_it {				/* integration time */
	unsigned int ms;		/* time ms */
	unsigned int resolution;	/* resolution */
	unsigned int range;		/* range */
};

static struct jsa_it jsa_it_tbl[] = {	/* lux_lo    lux_hi */
	{ 800, 210000, 6500 },		/* 0.21 <-> 6500    */
	{ 400, 420000, 13000 },		/* 0.42 <-> 13000   */
	{ 300, 560000, 18000 },		/* 0.56 <-> 18000   */
	{ 200, 830000, 27000 },		/* 0.83 <-> 27000   */
	{ 100, 1670000, 54000 },	/* 1.67 <-> 54000   */
	{ 50, 3330000, 109000 }		/* 3.33 <-> 109000  */
					/* note: lux_lo = resolution */
};

struct jsa_state {
	struct i2c_client *i2c;
	struct mutex mutex_client;
	struct iio_trigger *trig;
	struct delayed_work dw;
	struct regulator_bulk_data vreg[ARRAY_SIZE(jsa_vregs)];
	unsigned int info;		/* info data to return */
	unsigned int dbg;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int enable;		/* enable status */
	unsigned int poll_delay_ms;	/* requested sampling delay (ms) */
	unsigned int queue_delay_ms;	/* workqueue delay time (ms) */
	unsigned int scale_i;		/* index into HW IT settings table */
	int scale_val;			/* user scale val */
	int scale_val2;			/* user scale val2 */
	int offset_val;			/* user offset val */
	int offset_val2;		/* user offset val2 */
	int lux_uc_lo;			/* interpolation x1 uncalibrated lo */
	int lux_uc_hi;			/* interpolation x3 uncalibrated hi */
	int lux_c_lo;			/* interpolation y1 calibrated lo */
	int lux_c_hi;			/* interpolation y3 calibrated hi */
	unsigned int report;		/* used to report first valid sample */
	unsigned int report_n;		/* this many on-change data reports */
	unsigned int lux_thr_lo;	/* report when new lux below this */
	unsigned int lux_thr_hi;	/* report when new lux above this */
	unsigned int it_i_lo;		/* integration time index low limit */
	unsigned int it_i_hi;		/* integration time index high limit */
	unsigned long mult;		/* used to calc lux from HW values */
	bool iio_ts_en;			/* use IIO timestamps */
	bool shutdown;			/* shutdown active flag */
	bool suspend;			/* suspend active flag */
	bool hw_it;			/* HW defined integration time */
	u16 i2c_addr;			/* I2C address */
	u8 rc_cmd;			/* store for register dump */
	u32 light;			/* sample data */
	s64 ts;				/* sample data timestamp */
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
	mutex_lock(&st->mutex_client);
	if (i2c_transfer(st->i2c->adapter, &msg, 1) == 1) {
		*val = le16_to_cpup(val);
	} else {
		jsa_err(st);
		ret = -EIO;
	}
	mutex_unlock(&st->mutex_client);
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
		mutex_lock(&st->mutex_client);
		if (i2c_transfer(st->i2c->adapter, &msg, 1) == 1) {
			st->rc_cmd = val;
		} else {
			jsa_err(st);
			ret = -EIO;
		}
		mutex_unlock(&st->mutex_client);
		if (st->dbg & JSA_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s=%hhx err=%d\n",
				 __func__, val, ret);
	}
	return ret;
}

static int jsa_vreg_dis(struct jsa_state *st, unsigned int i)
{
	int ret = 0;

	if (st->vreg[i].ret && (st->vreg[i].consumer != NULL)) {
		ret = regulator_disable(st->vreg[i].consumer);
		if (ret)
			dev_err(&st->i2c->dev, "%s %s ERR\n",
				__func__, st->vreg[i].supply);
		else
			st->vreg[i].ret = 0;
			dev_dbg(&st->i2c->dev, "%s %s\n",
				__func__, st->vreg[i].supply);
	}
	return ret;
}

static int jsa_vreg_dis_all(struct jsa_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = ARRAY_SIZE(jsa_vregs); i > 0; i--)
		ret |= jsa_vreg_dis(st, (i - 1));
	return ret;
}

static int jsa_vreg_en(struct jsa_state *st, unsigned int i)
{
	int ret = 0;

	if ((!st->vreg[i].ret) && (st->vreg[i].consumer != NULL)) {
		ret = regulator_enable(st->vreg[i].consumer);
		if (ret) {
			dev_err(&st->i2c->dev, "%s %s ERR\n",
				__func__, st->vreg[i].supply);
		} else {
			st->vreg[i].ret = 1;
			dev_dbg(&st->i2c->dev, "%s %s\n",
				__func__, st->vreg[i].supply);
			ret = 1; /* flag regulator state change */
		}
	}
	return ret;
}

static int jsa_vreg_en_all(struct jsa_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(jsa_vregs); i++)
		ret |= jsa_vreg_en(st, i);
	return ret;
}

static void jsa_vreg_exit(struct jsa_state *st)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(jsa_vregs); i++) {
		if (st->vreg[i].consumer != NULL) {
			devm_regulator_put(st->vreg[i].consumer);
			st->vreg[i].consumer = NULL;
			dev_dbg(&st->i2c->dev, "%s %s\n",
				__func__, st->vreg[i].supply);
		}
	}
}

static int jsa_vreg_init(struct jsa_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(jsa_vregs); i++) {
		st->vreg[i].supply = jsa_vregs[i];
		st->vreg[i].ret = 0;
		st->vreg[i].consumer = devm_regulator_get(&st->i2c->dev,
							  st->vreg[i].supply);
		if (IS_ERR(st->vreg[i].consumer)) {
			ret |= PTR_ERR(st->vreg[i].consumer);
			dev_err(&st->i2c->dev, "%s ret %d for %s\n",
				__func__, ret, st->vreg[i].supply);
			st->vreg[i].consumer = NULL;
		} else {
			dev_dbg(&st->i2c->dev, "%s %s\n",
				__func__, st->vreg[i].supply);
		}
	}
	return ret;
}

static int jsa_vreg_sts(struct jsa_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(jsa_vregs); i++) {
		if (st->vreg[i].consumer != NULL)
			break;
	}
	if (i < ARRAY_SIZE(jsa_vregs)) {
		/* ret == number of regulators on */
		for (i = 0; i < ARRAY_SIZE(jsa_vregs); i++) {
			if (st->vreg[i].ret)
				ret++;
		}
	} else {
		/* no regulator support (can assume always on) */
		ret = -EINVAL;
	}
	return ret;
}

static int jsa_pm(struct jsa_state *st, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = jsa_vreg_en_all(st);
		if (ret)
			mdelay(JSA_HW_DELAY_MS);
	} else {
		ret = jsa_vreg_sts(st);
		if ((ret < 0) || (ret == ARRAY_SIZE(jsa_vregs))) {
			ret = jsa_i2c_wr(st, JSA_CMD_SHUTDOWN);
		} else if (ret > 0) {
			jsa_vreg_en_all(st);
			mdelay(JSA_HW_DELAY_MS);
			ret = jsa_i2c_wr(st, JSA_CMD_SHUTDOWN);
		}
		ret |= jsa_vreg_dis_all(st);
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		dev_err(&st->i2c->dev, "%s pwr=%x ERR=%d\n",
			__func__, enable, ret);
	} else {
		if (st->dbg & JSA_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s pwr=%x\n",
				 __func__, enable);
	}
	return ret;
}

static void jsa_pm_exit(struct jsa_state *st)
{
	jsa_pm(st, false);
	jsa_vreg_exit(st);
}

static int jsa_pm_init(struct jsa_state *st)
{
	int ret;

	st->enable = 0;
	st->poll_delay_ms = JSA_POLL_DELAY_MS_DFLT;
	jsa_vreg_init(st);
	ret = jsa_pm(st, true);
	return ret;
}

static unsigned int jsa_buf_index(unsigned int size, unsigned int *bytes)
{
	unsigned int index;

	if (!(*bytes % size))
		index = *bytes;
	else
		index = *bytes - *bytes % size + size;
	*bytes = index + size;
	return index;
}

static void jsa_buf_push(struct iio_dev *indio_dev, struct jsa_state *st)
{
	unsigned char buf[16];
	unsigned int n;
	unsigned int i;
	unsigned int bytes = 0;

	if (!iio_buffer_enabled(indio_dev))
		return;

	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, JSA_SCAN_LIGHT)) {
		n = sizeof(st->light);
		i = jsa_buf_index(n, &bytes);
		memcpy(&buf[i], &st->light, n);
		if (st->dbg & JSA_DBG_SPEW_LIGHT)
			dev_info(&st->i2c->dev, "light %u %lld\n",
				 st->light, st->ts);
	}
	if (indio_dev->buffer->scan_timestamp) {
		n = sizeof(st->ts);
		i = jsa_buf_index(n, &bytes);
		memcpy(&buf[i], &st->ts, n);
	}
	iio_push_to_buffers(indio_dev, buf);
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

	if (st->poll_delay_ms > st->queue_delay_ms) {
		jsa_i2c_wr(st, JSA_CMD_SHUTDOWN);
		ms = st->poll_delay_ms - st->queue_delay_ms;
	} else {
		jsa_start(st);
		ms = st->queue_delay_ms;
	}
	return ms;
}

static void jsa_delay(struct jsa_state *st)
{
	st->queue_delay_ms = jsa_it_tbl[st->scale_i].ms;
	if (st->dbg & JSA_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s queue_delay_ms=%u\n",
			 __func__, st->queue_delay_ms);
}

static int jsa_interpolate(int x1, int x2, int x3, int y1, int *y2, int y3)
{
	int dividend;
	int divisor;

	if (x2 < x1) {
		/* y2 = x2 * y1 / x1 */
		if (x1 == 0)
			return -EINVAL;
		*y2 = x2 * y1 / x1;
	} else {
		/* y2 = ((x2 - x1)(y3 - y1)/(x3 - x1)) + y1 */
		divisor = (x3 - x1);
		if (!divisor)
			return -EINVAL;

		dividend = (x2 - x1) * (y3 - y1);
		*y2 = (dividend / divisor) + y1;
	}
	return 0;
}

static void jsa_report_init(struct jsa_state *st)
{
	st->ts = 0;
	st->report = st->report_n;
}

static int jsa_rd(struct iio_dev *indio_dev)
{
	struct jsa_state *st = iio_priv(indio_dev);
	s64 ts;
	s64 ts_elapsed;
	u64 calc;
	u32 lux;
	u16 hw;
	bool t_min = false;
	int ret;

	ret = jsa_i2c_rd(st, &hw);
	if (ret)
		return ret;

	if (!(hw & (1 << JSA_VAL_VALID)))
		/* data not ready */
		return -EINVAL;

	hw &= ~(1 << JSA_VAL_VALID);
	calc = jsa_it_tbl[st->scale_i].resolution;
	calc *= hw;
	if (st->scale_val2)
		do_div(calc, st->scale_val2);
	lux = calc;
	jsa_interpolate(st->lux_uc_lo, lux, st->lux_uc_hi,
		       st->lux_c_lo, &lux, st->lux_c_hi);
	if (lux > (st->light + st->lux_thr_hi)) {
		st->report = st->report_n;
	} else if (st->light > st->lux_thr_lo) {
		if (lux < (st->light - st->lux_thr_lo))
			st->report = st->report_n;
	}
	ts = jsa_get_time_ns(st);
	ts_elapsed = ts - st->ts;
	if (ts_elapsed >= st->poll_delay_ms * 1000000)
		t_min = true;
	if (st->dbg & JSA_DBG_SPEW_LIGHT_POLL)
		dev_info(&st->i2c->dev,
			 "poll light %d %lld  diff: %d %lldns  hw=%hu\n",
			 lux, ts, lux - st->light, ts_elapsed, hw);
	if ((st->report && t_min) || ((st->scale_val == 1) &&
				      !st->scale_val2)) {
		/* report if:
		 * - st->report && time since last report >= polling delay
		 * - in calibration mode (scale == 1)
		 */
		if (st->report)
			st->report--;
		if (!(st->dbg & JSA_DBG_VAL_LIGHT))
			st->light = lux;
		st->ts = ts;
		jsa_buf_push(indio_dev, st);
	}
	return 0;
}

static void jsa_read(struct iio_dev *indio_dev)
{
	struct jsa_state *st = iio_priv(indio_dev);
	unsigned int ms;
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (st->enable) {
		if (st->hw_it) {
			jsa_rd(indio_dev);
			if (st->poll_delay_ms > st->queue_delay_ms)
				ms = st->poll_delay_ms;
			else
				ms = st->queue_delay_ms;
		} else {
			ms = st->queue_delay_ms;
			if (st->rc_cmd == JSA_CMD_START) {
				ret = jsa_i2c_wr(st, JSA_CMD_STOP);
				if (!ret) {
					ret = jsa_rd(indio_dev);
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
	mutex_unlock(&indio_dev->mlock);
}

static void jsa_work(struct work_struct *ws)
{
	struct jsa_state *st = container_of((struct delayed_work *)ws,
					   struct jsa_state, dw);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);

	jsa_read(indio_dev);
}

static int jsa_disable(struct iio_dev *indio_dev)
{
	struct jsa_state *st = iio_priv(indio_dev);
	int ret;

	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  JSA_SCAN_LIGHT)))
		st->dbg &= ~JSA_DBG_VAL_LIGHT;
	cancel_delayed_work(&st->dw);
	ret = jsa_pm(st, false);
	if (!ret)
		st->enable = 0;
	return ret;
}

static int jsa_enable(struct iio_dev *indio_dev)
{
	struct jsa_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (iio_scan_mask_query(indio_dev,
				indio_dev->buffer, JSA_SCAN_LIGHT)) {
		ret = jsa_pm(st, true);
		if (!ret) {
			ret = jsa_start(st);
			if (ret) {
				jsa_disable(indio_dev);
			} else {
				jsa_delay(st);
				jsa_report_init(st);
				st->enable = 1;
				schedule_delayed_work(&st->dw,
					 msecs_to_jiffies(st->queue_delay_ms));
			}
		}
	}
	return ret;
}

static ssize_t jsa_attr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct jsa_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	const char *msg;
	unsigned int new;
	unsigned int old = 0;
	int ret;

	ret = kstrtouint(buf, 10, &new);
	if (ret)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	if (st->shutdown || st->suspend) {
		mutex_unlock(&indio_dev->mlock);
		return -EPERM;
	}

	switch (this_attr->address) {
	case JSA_ATTR_ENABLE:
		msg = "ATTR_ENABLE";
		if (st->enable)
			old = 1;
		if (new)
			ret = jsa_enable(indio_dev);
		else
			ret = jsa_disable(indio_dev);
		break;

	case JSA_ATTR_THRESH_LO:
		msg = "ATTR_THRESH_LO";
		old = st->lux_thr_lo;
		st->lux_thr_lo = new;
		break;

	case JSA_ATTR_THRESH_HI:
		msg = "ATTR_THRESH_HI";
		old = st->lux_thr_hi;
		st->lux_thr_hi = new;
		break;

	default:
		msg = "ATTR_UNKNOWN";
		ret = -EINVAL;
	}

	jsa_report_init(st);
	mutex_unlock(&indio_dev->mlock);
	jsa_read(indio_dev);
	if (st->dbg & JSA_DBG_SPEW_MSG) {
		if (ret)
			dev_err(&st->i2c->dev, "%s %s %d->%d ERR=%d\n",
				__func__, msg, old, new, ret);
		else
			dev_info(&st->i2c->dev, "%s %s %d->%d\n",
				 __func__, msg, old, new);
	}
	if (ret)
		return ret;

	return count;
}

static ssize_t jsa_attr_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct jsa_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	switch (this_attr->address) {
	case JSA_ATTR_ENABLE:
		return sprintf(buf, "%x\n", st->enable);

	case JSA_ATTR_PART:
		return sprintf(buf, "%s light\n", JSA_NAME);

	case JSA_ATTR_VENDOR:
		return sprintf(buf, "%s\n", JSA_VENDOR);

	case JSA_ATTR_VERSION:
		return sprintf(buf, "%u\n", JSA_LIGHT_VERSION);

	case JSA_ATTR_MILLIAMP:
		return sprintf(buf, "%s\n", JSA_LIGHT_MILLIAMP);

	case JSA_ATTR_THRESH_LO:
		return sprintf(buf, "%u\n", st->lux_thr_lo);

	case JSA_ATTR_THRESH_HI:
		return sprintf(buf, "%u\n", st->lux_thr_hi);

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static ssize_t jsa_data_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct jsa_state *st = iio_priv(dev_get_drvdata(dev));
	unsigned int info;
	int ret;

	ret = kstrtouint(buf, 10, &info);
	if (ret)
		return -EINVAL;

	if (info >= JSA_INFO_LIMIT_MAX)
		return -EINVAL;

	st->info = info;
	switch (info) {
	case JSA_INFO_DATA:
		st->dbg = 0;
		break;

	case JSA_INFO_DBG:
		st->dbg ^= JSA_DBG_SPEW_MSG;
		break;

	case JSA_INFO_LIGHT_SPEW:
		st->dbg ^= JSA_DBG_SPEW_LIGHT;
		break;

	case JSA_INFO_LIGHT_POLL_SPEW:
		st->dbg ^= JSA_DBG_SPEW_LIGHT_POLL;
		break;

	default:
		break;
	}

	return count;
}

static ssize_t jsa_data_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct jsa_state *st = iio_priv(dev_get_drvdata(dev));
	enum JSA_INFO info;
	ssize_t t;
	u16 val;
	int ret;

	info = st->info;
	st->info = JSA_INFO_DATA;
	switch (info) {
	case JSA_INFO_DATA:
		t = sprintf(buf, "LIGHT: %d  ts: %lld\n",
			    st->light, st->ts);
		return t;

	case JSA_INFO_VER:
		return sprintf(buf, "version=%u\n", JSA_VERSION_DRIVER);

	case JSA_INFO_ERRS:
		t = sprintf(buf, "error count=%u\n", st->errs);
		st->errs = 0;
		return t;

	case JSA_INFO_REGS:
		t = sprintf(buf, "registers:\n");
		t += sprintf(buf + t, "CMD=%#2x\n", st->rc_cmd);
		ret = jsa_i2c_rd(st, &val);
		t += sprintf(buf + t, "VAL=%#4x  ERR=%d\n", val, ret);
		return t;

	case JSA_INFO_DBG:
		return sprintf(buf, "debug spew=%x\n",
			       !!(st->dbg & JSA_DBG_SPEW_MSG));

	case JSA_INFO_LIGHT_SPEW:
		return sprintf(buf, "lux_ts spew=%x\n",
			       !!(st->dbg & JSA_DBG_SPEW_LIGHT));

	case JSA_INFO_LIGHT_POLL_SPEW:
		return sprintf(buf, "lux_poll_ts spew=%x\n",
			       !!(st->dbg & JSA_DBG_SPEW_LIGHT_POLL));

	default:
		break;
	}

	return -EINVAL;
}

static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		       jsa_attr_show, jsa_attr_store, JSA_ATTR_ENABLE);
static IIO_DEVICE_ATTR(illuminance_part, S_IRUGO,
		       jsa_attr_show, NULL, JSA_ATTR_PART);
static IIO_DEVICE_ATTR(illuminance_vendor, S_IRUGO,
		       jsa_attr_show, NULL, JSA_ATTR_VENDOR);
static IIO_DEVICE_ATTR(illuminance_version, S_IRUGO,
		       jsa_attr_show, NULL, JSA_ATTR_VERSION);
static IIO_DEVICE_ATTR(illuminance_milliamp, S_IRUGO,
		       jsa_attr_show, NULL, JSA_ATTR_MILLIAMP);
static IIO_DEVICE_ATTR(illuminance_thresh_rising_value,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       jsa_attr_show, jsa_attr_store, JSA_ATTR_THRESH_HI);
static IIO_DEVICE_ATTR(illuminance_thresh_falling_value,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       jsa_attr_show, jsa_attr_store, JSA_ATTR_THRESH_LO);
static DEVICE_ATTR(data, S_IRUGO | S_IWUSR | S_IWGRP,
		   jsa_data_show, jsa_data_store);

static struct attribute *jsa_attrs[] = {
	&dev_attr_data.attr,
	&iio_dev_attr_enable.dev_attr.attr,
	&iio_dev_attr_illuminance_part.dev_attr.attr,
	&iio_dev_attr_illuminance_vendor.dev_attr.attr,
	&iio_dev_attr_illuminance_version.dev_attr.attr,
	&iio_dev_attr_illuminance_milliamp.dev_attr.attr,
	&iio_dev_attr_illuminance_thresh_rising_value.dev_attr.attr,
	&iio_dev_attr_illuminance_thresh_falling_value.dev_attr.attr,
	NULL
};

static struct attribute_group jsa_attr_group = {
	.name = JSA_NAME,
	.attrs = jsa_attrs
};

static int jsa_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct jsa_state *st = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = st->light;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->enable) {
			*val = st->poll_delay_ms * 1000; /* ms => us */
		} else {
			if (st->hw_it)
				*val = jsa_it_tbl[st->scale_i].ms * 1000;
			else
				*val = jsa_it_tbl[st->it_i_hi].ms * 1000;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->scale_val;
		*val2 = st->scale_val2;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		*val = st->offset_val;
		*val2 = st->offset_val2;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_PEAK:
		*val2 = JSA_LIGHT_MAX_RANGE_MICRO;
		if (st->hw_it)
			*val = jsa_it_tbl[st->scale_i].range;
		else
			*val = jsa_it_tbl[st->it_i_hi].range;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_PEAK_SCALE:
		*val = JSA_LIGHT_RESOLUTION_IVAL;
		*val2 = JSA_LIGHT_RESOLUTION_MICRO;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return ret;
}

static int jsa_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask)
{
	struct jsa_state *st = iio_priv(indio_dev);
	char *msg;
	int old = 0;
	int old2 = 0;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (st->shutdown || st->suspend) {
		mutex_unlock(&indio_dev->mlock);
		return -EPERM;
	}

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		msg = "IIO_CHAN_INFO_SAMP_FREQ";
		old = st->poll_delay_ms * 1000; /* ms => us*/
		st->poll_delay_ms = (unsigned int)(val / 1000); /* us => ms */
		break;

	case IIO_CHAN_INFO_SCALE:
		msg = "IIO_CHAN_INFO_SCALE";
		old = st->scale_val;
		old2 = st->scale_val2;
		st->scale_val = val;
		st->scale_val2 = val2;
		break;

	case IIO_CHAN_INFO_OFFSET:
		msg = "IIO_CHAN_INFO_SCALE";
		old = st->offset_val;
		old2 = st->offset_val2;
		st->offset_val = val;
		st->offset_val2 = val2;
		break;

	case IIO_CHAN_INFO_RAW:
		msg = "IIO_CHAN_INFO_RAW";
		old = st->light;
		st->light = val;
		st->ts = jsa_get_time_ns(st);
		st->dbg |= JSA_DBG_VAL_LIGHT;
		jsa_buf_push(indio_dev, st);
		break;

	default:
		msg = "IIO_CHAN_INFO_UNKNOWN";
		ret = -EINVAL;
	}

	jsa_report_init(st);
	mutex_unlock(&indio_dev->mlock);
	jsa_read(indio_dev);
	if (st->dbg & JSA_DBG_SPEW_MSG) {
		if (ret)
			dev_err(&st->i2c->dev, "%s c=%d %d:%d->%d:%d ERR=%d\n",
				msg, chan->scan_index,
				old, old2, val, val2, ret);
		else
			dev_info(&st->i2c->dev, "%s %s chan=%d %d:%d->%d:%d\n",
				 __func__, msg, chan->scan_index,
				 old, old2, val, val2);
	}
	return ret;
}

static const struct iio_info jsa_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &jsa_attr_group,
	.read_raw = &jsa_read_raw,
	.write_raw = &jsa_write_raw,
};

static const struct iio_chan_spec jsa_channels[] = {
	{
		.type			= IIO_LIGHT,
		.scan_index		= JSA_SCAN_LIGHT,
		.scan_type		= IIO_ST('u', 32, 32, 0),
		.info_mask		= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
	},
	IIO_CHAN_SOFT_TIMESTAMP(JSA_SCAN_TIMESTAMP)
};

static int jsa_buffer_preenable(struct iio_dev *indio_dev)
{
	struct jsa_state *st = iio_priv(indio_dev);

	if (st->shutdown || st->suspend)
		return -EINVAL;

	return iio_sw_buffer_preenable(indio_dev);
}

static int jsa_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;

	ret = jsa_enable(indio_dev);
	/* never return > 0 to IIO buffer engine */
	if (ret > 0)
		ret = 0;
	return ret;
}

static const struct iio_buffer_setup_ops jsa_buffer_setup_ops = {
	/* iio_sw_buffer_preenable:
	 * Generic function for equal sized ring elements + 64 bit timestamp
	 * Assumes that any combination of channels can be enabled.
	 * Typically replaced to implement restrictions on what combinations
	 * can be captured (hardware scan modes).
	 */
	.preenable = &jsa_buffer_preenable,
	/* iio_triggered_buffer_postenable:
	 * Generic function that simply attaches the pollfunc to the trigger.
	 * Replace this to mess with hardware state before we attach the
	 * trigger.
	 */
	.postenable = &jsa_buffer_postenable,
	/* this driver relies on the NVS HAL to power off this device with the
	 * master enable.
	 *.predisable = N/A
	 *.postdisable = N/A
	 */
};

static const struct iio_trigger_ops jsa_trigger_ops = {
	.owner = THIS_MODULE,
};

static int jsa_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct jsa_state *st = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (st->enable)
		ret = jsa_disable(indio_dev);
	st->suspend = true;
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & JSA_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int jsa_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct jsa_state *st = iio_priv(indio_dev);

	st->suspend = false;
	if (st->dbg & JSA_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(jsa_pm_ops, jsa_suspend, jsa_resume);

static int jsa_id_dev(struct iio_dev *indio_dev, const char *name)
{
	struct jsa_state *st = iio_priv(indio_dev);
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

static int jsa_id_i2c(struct iio_dev *indio_dev, const char *name)
{
	struct jsa_state *st = iio_priv(indio_dev);
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(jsa_i2c_addrs); i++) {
		if (st->i2c->addr == jsa_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(jsa_i2c_addrs)) {
		st->i2c_addr = st->i2c->addr;
		ret = jsa_id_dev(indio_dev, name);
	} else {
		name = NULL;
		for (i = 0; i < ARRAY_SIZE(jsa_i2c_addrs); i++) {
			st->i2c_addr = jsa_i2c_addrs[i];
			ret = jsa_id_dev(indio_dev, name);
			if (!ret)
				break;
		}
	}
	if (ret)
		st->i2c_addr = 0;
	return ret;
}

static void jsa_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct jsa_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (st->enable)
		jsa_disable(indio_dev);
	st->shutdown = true;
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & JSA_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int jsa_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct jsa_state *st = iio_priv(indio_dev);

	if (st != NULL) {
		jsa_shutdown(client);
		if (indio_dev->dev.devt)
			iio_device_unregister(indio_dev);
		if (st->trig != NULL) {
			iio_trigger_unregister(st->trig);
			iio_trigger_free(st->trig);
		}
		if (indio_dev->buffer != NULL) {
			iio_buffer_unregister(indio_dev);
			iio_kfifo_free(indio_dev->buffer);
		}
		if (st->dw.wq)
			destroy_workqueue(st->dw.wq);
		jsa_pm_exit(st);
		iio_device_free(indio_dev);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int jsa_of_dt(struct i2c_client *client, struct jsa_state *st)
{
	struct device_node *dn = client->dev.of_node;
	unsigned int val;
	unsigned int i;

	/* default NVS ALS programmable parameters */
	st->scale_val = JSA_LIGHT_SCALE_IVAL;
	st->scale_val2 = JSA_LIGHT_SCALE_MICRO;
	st->offset_val = JSA_LIGHT_OFFSET_IVAL;
	st->offset_val2 = JSA_LIGHT_OFFSET_MICRO;
	st->lux_thr_lo = JSA_THRESHOLD_LUX_DFLT;
	st->lux_thr_hi = JSA_THRESHOLD_LUX_DFLT;
	st->it_i_lo = 0;
	st->it_i_hi = ARRAY_SIZE(jsa_it_tbl);
	st->hw_it = true;
	/* device tree parameters */
	if (client->dev.of_node) {
		/* common NVS programmable parameters */
		st->iio_ts_en = of_property_read_bool(dn, "iio_timestamps");
		of_property_read_u32(dn, "report_count", &st->report_n);
		/* common NVS ALS programmable parameters */
		of_property_read_s32(dn, "light_uncalibrated_lo",
				     &st->lux_uc_lo);
		of_property_read_s32(dn, "light_uncalibrated_hi",
				     &st->lux_uc_hi);
		of_property_read_s32(dn, "light_calibrated_lo",
				     &st->lux_c_lo);
		of_property_read_s32(dn, "light_calibrated_hi",
				     &st->lux_c_hi);
		of_property_read_s32(dn, "light_scale_val",
				     &st->scale_val);
		of_property_read_s32(dn, "light_scale_val2",
				     &st->scale_val2);
		of_property_read_s32(dn, "light_offset_val",
				     &st->offset_val);
		of_property_read_s32(dn, "light_offset_val2",
				     &st->offset_val2);
		of_property_read_u32(dn, "light_threshold_lo",
				     &st->lux_thr_lo);
		of_property_read_u32(dn, "light_threshold_hi",
				     &st->lux_thr_hi);
		/* this device supports these programmable parameters */
		if (!of_property_read_u32(dn, "light_integration_time_ms_lo",
					  &val)) {
			for (i = ARRAY_SIZE(jsa_it_tbl); i > 1; i--) {
				if (val <= jsa_it_tbl[i - 1].ms)
					break;
			}
			st->it_i_hi = i;
			st->hw_it = false;
		}
		if (!of_property_read_u32(dn, "light_integration_time_ms_hi",
					  &val)) {
			for (i = 0; i < ARRAY_SIZE(jsa_it_tbl) - 1; i++) {
				if (val >= jsa_it_tbl[i].ms)
					break;
			}
			st->it_i_lo = i;
			st->hw_it = false;
		}
		if (st->it_i_hi < st->it_i_lo) {
			dev_err(&client->dev,
				"%s light_integration_time_ms_ ERR\n",
				__func__);
			st->it_i_lo = 0;
			st->it_i_hi = ARRAY_SIZE(jsa_it_tbl);
		}
		if (!of_property_read_u32(dn, "light_integration_time_ms_hw",
					  &val)) {
			for (i = 0; i < ARRAY_SIZE(jsa_it_tbl) - 1; i++) {
				if (val >= jsa_it_tbl[i].ms)
					break;
			}
			st->scale_i = i;
			st->hw_it = true;
		}
	}
	if (!st->report_n)
		st->report_n = JSA_REPORT_N;
	return 0;
}

static int jsa_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct jsa_state *st;
	int ret;

	dev_info(&client->dev, "%s\n", __func__);
	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		dev_err(&client->dev, "%s iio_device_alloc ERR\n", __func__);
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	st->i2c = client;
	i2c_set_clientdata(client, indio_dev);
	ret = jsa_of_dt(client, st);
	if (ret) {
		dev_err(&client->dev, "%s _of_dt ERR\n", __func__);
		ret = -ENODEV;
		goto jsa_probe_err;
	}

	mutex_init(&st->mutex_client);
	jsa_pm_init(st);
	ret = jsa_id_i2c(indio_dev, id->name);
	if (ret) {
		dev_err(&client->dev, "%s _id_i2c ERR\n", __func__);
		ret = -ENODEV;
		goto jsa_probe_exit;
	}

	indio_dev->buffer = iio_kfifo_allocate(indio_dev);
	if (!indio_dev->buffer) {
		dev_err(&client->dev, "%s iio_kfifo_allocate ERR\n", __func__);
		ret = -ENOMEM;
		goto jsa_probe_err;
	}

	indio_dev->buffer->scan_timestamp = true;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = jsa_channels;
	indio_dev->num_channels = ARRAY_SIZE(jsa_channels);
	indio_dev->name = JSA_NAME;
	indio_dev->info = &jsa_iio_info;
	indio_dev->setup_ops = &jsa_buffer_setup_ops;
	ret = iio_buffer_register(indio_dev, indio_dev->channels,
				  indio_dev->num_channels);
	if (ret) {
		dev_err(&client->dev, "%s iio_buffer_register ERR\n",
			__func__);
		goto jsa_probe_err;
	}

	INIT_DELAYED_WORK(&st->dw, jsa_work);
	st->trig = iio_trigger_alloc("%s-dev%d",
				     indio_dev->name, indio_dev->id);
	if (st->trig == NULL) {
		dev_err(&client->dev, "%s iio_allocate_trigger ERR\n",
			__func__);
		ret = -ENOMEM;
		goto jsa_probe_err;
	}

	st->trig->dev.parent = &st->i2c->dev;
	st->trig->ops = &jsa_trigger_ops;
	ret = iio_trigger_register(st->trig);
	if (ret) {
		dev_err(&client->dev, "%s iio_trigger_register ERR\n",
			__func__);
		ret = -ENOMEM;
		goto jsa_probe_err;
	}

	indio_dev->trig = st->trig;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&client->dev, "%s iio_device_register ERR\n",
			__func__);
		goto jsa_probe_err;
	}

	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

jsa_probe_err:
	dev_err(&client->dev, "%s ERR %d\n", __func__, ret);
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
