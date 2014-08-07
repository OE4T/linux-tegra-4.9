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

/* IT = Integration Time.  The amount of time the photons hit the sensor.
 * STEP = the value from HW which is the photon count during IT.
 * LUX = STEP * (CM_RESOLUTION_STEP / IT) / CM_RESOLUTION_DIVISOR
 * The above LUX reported as LUX * CM_LIGHT_DIVISOR.
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

#define CM_VERSION_DRIVER		(102)
#define CM_VENDOR			"Capella Microsystems, Inc."
#define CM_NAME				"cm3217"
/* setting _REPORT_N to 2 causes an extra reading after crossing the threshold
 * allowing a more accurate settled reported value.
 */
#define CM_REPORT_N			(2)
#define CM_LIGHT_VERSION		(1)
#define CM_LIGHT_MAX_RANGE_IVAL		(119154)
#define CM_LIGHT_MAX_RANGE_MICRO	(420000)
#define CM_LIGHT_RESOLUTION_IVAL	(0)
#define CM_LIGHT_RESOLUTION_MICRO	(10000)
#define CM_LIGHT_SCALE_IVAL		(0)
#define CM_LIGHT_SCALE_MICRO		(10000)
#define CM_LIGHT_OFFSET_IVAL		(0)
#define CM_LIGHT_OFFSET_MICRO		(0)
#define CM_LIGHT_MILLIAMP		"0.09"
#define CM_RESOLUTION_STEP		(6000000L)
#define CM_RESOLUTION_DIVISOR		(1000L)
#define CM_LIGHT_DIVISOR		(1000000 / CM_LIGHT_SCALE_MICRO)
#define CM_POLL_DELAY_MS_DFLT		(2000)
#define CM_THRESHOLD_LUX_DFLT		(100)
/* HW registers */
#define CM_I2C_ADDR_CMD1_WR		(0x10)
#define CM_I2C_ADDR_CMD2_WR		(0x11)
#define CM_I2C_ADDR_RD			(0x10)
#define CM_HW_CMD1_DFLT			(0x22)
#define CM_HW_CMD1_BIT_SD		(0)
#define CM_HW_CMD1_BIT_IT_T		(2)
#define CM_HW_CMD2_BIT_FD_IT		(5)
#define CM_HW_DELAY_MS			(10)
/* _buf_push expects this scan order */
#define CM_SCAN_LIGHT			0
#define CM_SCAN_TIMESTAMP		1
/* debug flags */
#define CM_DBG_SPEW_MSG			(1 << 0)
#define CM_DBG_SPEW_LIGHT		(1 << 1)
#define CM_DBG_SPEW_LIGHT_POLL		(1 << 2)
#define CM_DBG_VAL_LIGHT		(1 << 3)

enum CM_ATTR {
	CM_ATTR_PART,
	CM_ATTR_VENDOR,
	CM_ATTR_VERSION,
	CM_ATTR_MILLIAMP,
	CM_ATTR_ENABLE,
	CM_ATTR_THRESH_LO,
	CM_ATTR_THRESH_HI,
};

enum CM_INFO {
	CM_INFO_DATA = 0,
	CM_INFO_VER,
	CM_INFO_ERRS,
	CM_INFO_REGS,
	CM_INFO_DBG,
	CM_INFO_LIGHT_SPEW,
	CM_INFO_LIGHT_POLL_SPEW,
	CM_INFO_LIMIT_MAX,
};

/* regulator names in order of powering on */
static char *cm_vregs[] = {
	"vdd",
};

struct cm_it {				/* integration time */
	unsigned int ms;		/* time ms */
	u8 fd_it;			/* FD_IT HW */
	u8 it_t;			/* IT_T HW */
};

static struct cm_it cm_it_tbl[] = {	/* (ms)       lux_lo    lux_hi   */
	{ 3200, 0, 3 },			/* (800ms * 4)  0.01 <-> 1228.78 */
	{ 1600, 0, 2 },			/* (800ms * 2)  0.03 <-> 2457.56 */
	{ 800, 0, 1 },			/* (800ms * 1)  0.07 <-> 4915.12 */
	{ 400, 1, 1 },			/* (400ms * 1)  0.15 <-> 9830.25 */
	{ 266, 2, 1 },			/* (266ms * 1) 0.22 <-> 14782.07 */
	{ 200, 3, 1 },			/* (200ms * 1) 0.30 <-> 19660.50 */
	{ 130, 4, 1 },			/* (130ms * 1) 0.46 <-> 30246.36 */
	{ 100, 5, 1 },			/* (100ms * 1) 0.60 <-> 39321.00 */
	{ 80, 6, 1 },			/* (80ms * 1)  0.75 <-> 49151.25 */
	{ 66, 7, 1 },			/* (66ms * 1)  0.90 <-> 59577.21 */
	{ 33, 7, 0 }			/* (66ms / 2) 1.81 <-> 119154.42 */
					/* note: lux_lo = resolution */
};

struct cm_state {
	struct i2c_client *i2c;
	struct iio_trigger *trig;
	struct delayed_work dw;
	struct regulator_bulk_data vreg[ARRAY_SIZE(cm_vregs)];
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
	bool hw_change;			/* HW changed so drop first sample */
	bool hw_sync;			/* queue time match HW sample time */
	u8 cmd1;			/* store for register dump */
	u8 cmd2;			/* store for register dump */
	u32 light;			/* sample data */
	s64 ts;				/* sample data timestamp */
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

static int cm_i2c_rd(struct cm_state *st, u16 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];

	msg[0].addr = CM_I2C_ADDR_RD + 1;
	msg[0].flags = I2C_M_RD;
	msg[0].len = 1;
	msg[0].buf = &buf[0];
	msg[1].addr = CM_I2C_ADDR_RD;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	if (i2c_transfer(st->i2c->adapter, msg, 2) != 2) {
		cm_err(st);
		return -EIO;
	}

	*val = (__u16)((buf[1] << 8) | buf[0]);
	return 0;
}

static int cm_i2c_wr(struct cm_state *st, u8 cmd1, u8 cmd2)
{
	struct i2c_msg msg[2];
	u8 buf[2];

	buf[0] = cmd1;
	buf[1] = cmd2;
	msg[0].addr = CM_I2C_ADDR_CMD1_WR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf[0];
	msg[1].addr = CM_I2C_ADDR_CMD2_WR;
	msg[1].flags = 0;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	if (i2c_transfer(st->i2c->adapter, msg, 2) != 2) {
		cm_err(st);
		return -EIO;
	}

	st->cmd1 = cmd1;
	st->cmd2 = cmd2;
	return 0;
}

static int cm_vreg_dis(struct cm_state *st, unsigned int i)
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

static int cm_vreg_dis_all(struct cm_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = ARRAY_SIZE(cm_vregs); i > 0; i--)
		ret |= cm_vreg_dis(st, (i - 1));
	return ret;
}

static int cm_vreg_en(struct cm_state *st, unsigned int i)
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

static int cm_vreg_en_all(struct cm_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(cm_vregs); i++)
		ret |= cm_vreg_en(st, i);
	return ret;
}

static void cm_vreg_exit(struct cm_state *st)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cm_vregs); i++) {
		if (st->vreg[i].consumer != NULL) {
			devm_regulator_put(st->vreg[i].consumer);
			st->vreg[i].consumer = NULL;
			dev_dbg(&st->i2c->dev, "%s %s\n",
				__func__, st->vreg[i].supply);
		}
	}
}

static int cm_vreg_init(struct cm_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(cm_vregs); i++) {
		st->vreg[i].supply = cm_vregs[i];
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

static int cm_vreg_sts(struct cm_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(cm_vregs); i++) {
		if (st->vreg[i].consumer != NULL)
			break;
	}
	if (i < ARRAY_SIZE(cm_vregs)) {
		/* ret == number of regulators on */
		for (i = 0; i < ARRAY_SIZE(cm_vregs); i++) {
			if (st->vreg[i].ret)
				ret++;
		}
	} else {
		/* no regulator support (can assume always on) */
		ret = -EINVAL;
	}
	return ret;
}

static int cm_pm(struct cm_state *st, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = cm_vreg_en_all(st);
		if (ret)
			mdelay(CM_HW_DELAY_MS);
	} else {
		ret = cm_vreg_sts(st);
		if ((ret < 0) || (ret == ARRAY_SIZE(cm_vregs))) {
			ret = cm_i2c_wr(st, (CM_HW_CMD1_DFLT |
					     CM_HW_CMD1_BIT_SD), 0);
		} else if (ret > 0) {
			cm_vreg_en_all(st);
			mdelay(CM_HW_DELAY_MS);
			ret = cm_i2c_wr(st, (CM_HW_CMD1_DFLT |
					     CM_HW_CMD1_BIT_SD), 0);
		}
		ret |= cm_vreg_dis_all(st);
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		dev_err(&st->i2c->dev, "%s pwr=%x ERR=%d\n",
			__func__, enable, ret);
	} else {
		if (st->dbg & CM_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s pwr=%x\n",
				 __func__, enable);
	}
	return ret;
}

static void cm_pm_exit(struct cm_state *st)
{
	cm_pm(st, false);
	cm_vreg_exit(st);
}

static int cm_pm_init(struct cm_state *st)
{
	int ret;

	st->enable = 0;
	st->poll_delay_ms = CM_POLL_DELAY_MS_DFLT;
	cm_vreg_init(st);
	/* on off for low power mode if regulator still on */
	ret = cm_pm(st, true);
	ret |= cm_pm(st, false);
	return ret;
}

static unsigned int cm_buf_index(unsigned int size, unsigned int *bytes)
{
	unsigned int index;

	if (!(*bytes % size))
		index = *bytes;
	else
		index = *bytes - *bytes % size + size;
	*bytes = index + size;
	return index;
}

static void cm_buf_push(struct iio_dev *indio_dev, struct cm_state *st)
{
	unsigned char buf[16];
	unsigned int n;
	unsigned int i;
	unsigned int bytes = 0;

	if (!iio_buffer_enabled(indio_dev))
		return;

	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, CM_SCAN_LIGHT)) {
		n = sizeof(st->light);
		i = cm_buf_index(n, &bytes);
		memcpy(&buf[i], &st->light, n);
		if (st->dbg & CM_DBG_SPEW_LIGHT)
			dev_info(&st->i2c->dev, "light %u %lld\n",
				 st->light, st->ts);
	}
	if (indio_dev->buffer->scan_timestamp) {
		n = sizeof(st->ts);
		i = cm_buf_index(n, &bytes);
		memcpy(&buf[i], &st->ts, n);
	}
	iio_push_to_buffers(indio_dev, buf);
}

static int cm_cmd_wr(struct cm_state *st, u8 it_t, u8 fd_it)
{
	u8 cmd1;
	u8 cmd2;
	int ret;

	cmd1 = CM_HW_CMD1_DFLT;
	cmd1 |= (it_t << CM_HW_CMD1_BIT_IT_T);
	cmd2 = fd_it << CM_HW_CMD2_BIT_FD_IT;
	ret = cm_i2c_wr(st, cmd1, cmd2);
	if (st->dbg & CM_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s cmd1=%hhx cmd2=%hhx\n",
			 __func__, cmd1, cmd2);
	return ret;
}

static void cm_delay(struct cm_state *st, bool hw_sync)
{
	unsigned int ms;

	if (hw_sync)
		st->hw_sync = true;
	ms = cm_it_tbl[st->scale_i].ms;
	ms += CM_HW_DELAY_MS;
	if ((ms < st->poll_delay_ms) && !hw_sync)
		st->queue_delay_ms = st->poll_delay_ms;
	else
		/* we're either outside the HW integration time (IT) window and
		 * want to get within the window as fast as HW allows us
		 * (hw_sync = true)
		 * OR
		 * HW IT is not as fast as requested polling time
		 */
		st->queue_delay_ms = ms;
	if (st->dbg & CM_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s queue_delay_ms=%u\n",
			 __func__, st->queue_delay_ms);
}

static int cm_it_wr(struct cm_state *st, unsigned int ms)
{
	unsigned int i;
	int ret;

	/* get the HW settings for integration time (IT) ms */
	for (i = st->it_i_lo; i < st->it_i_hi; i++) {
		if (ms >= cm_it_tbl[i].ms)
			break;
	}
	if (i >= st->it_i_hi)
		i = (st->it_i_hi - 1);
	ret = cm_cmd_wr(st, cm_it_tbl[i].it_t, cm_it_tbl[i].fd_it);
	if (!ret) {
		st->hw_change = true;
		st->scale_i = i;
		ms = cm_it_tbl[i].ms;
		st->mult = CM_RESOLUTION_STEP / (long)ms;
	}
	if (st->dbg & CM_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s IT=%u err=%d\n",
			 __func__, cm_it_tbl[i].ms, ret);
	return ret;
}

static int cm_interpolate(int x1, int x2, int x3, int y1, int *y2, int y3)
{
	int dividend;
	int divisor;

	/* y2 = ((x2 - x1)(y3 - y1)/(x3 - x1)) + y1 */
	divisor = (x3 - x1);
	if (!divisor)
		return -EINVAL;

	dividend = (x2 - x1) * (y3 - y1);
	*y2 = (dividend / divisor) + y1;
	return 0;
}

static void cm_report_init(struct cm_state *st)
{
	st->ts = 0;
	st->report = st->report_n;
}

static int cm_rd(struct iio_dev *indio_dev)
{
	struct cm_state *st = iio_priv(indio_dev);
	u16 step;
	u32 lux;
	s64 ts;
	s64 ts_elapsed;
	bool t_min = false;
	unsigned long calc;
	int ret;

	if (st->hw_change && !st->report) {
		/* drop first sample after HW change */
		st->hw_change = false;
		return 0;
	}

	ret = cm_i2c_rd(st, &step);
	if (ret)
		return ret;

	calc = step;
	calc *= st->mult;
	calc /= CM_RESOLUTION_DIVISOR;
	lux = calc;
	cm_interpolate(st->lux_uc_lo, lux, st->lux_uc_hi,
		       st->lux_c_lo, &lux, st->lux_c_hi);
	if ((step == 0xFFFF) && (st->scale_i < (st->it_i_hi - 1))) {
		/* too many photons - need to decrease integration time */
		ret = cm_it_wr(st, cm_it_tbl[st->scale_i + 1].ms);
		if (!ret)
			cm_delay(st, true);
	} else if ((lux <= CM_LIGHT_DIVISOR) && (st->scale_i >
						 (st->it_i_lo + 1))) {
		/* not enough photons - need to increase integration time */
		ret = cm_it_wr(st, cm_it_tbl[st->scale_i - 1].ms);
		if (!ret)
			cm_delay(st, true);
	} else if (st->hw_sync) {
		/* adjust queue time to max(polling delay, HW IT) */
		st->hw_sync = false;
		cm_delay(st, false);
	}
	if (lux > (st->light + st->lux_thr_hi)) {
		st->report = st->report_n;
	} else if (st->light > st->lux_thr_lo) {
		if (lux < (st->light - st->lux_thr_lo))
			st->report = st->report_n;
	}
	ts = cm_get_time_ns(st);
	ts_elapsed = ts - st->ts;
	if (ts_elapsed >= st->poll_delay_ms * 1000000)
		t_min = true;
	if (st->dbg & CM_DBG_SPEW_LIGHT_POLL)
		dev_info(&st->i2c->dev,
			 "poll light %d %lld  diff: %d %lldns\n",
			 lux, ts, lux - st->light, ts_elapsed);
	if ((st->report && t_min) || ((st->scale_val == 1) &&
				      !st->scale_val2)) {
		/* report if:
		 * - st->report && time since last report >= polling delay
		 * - in calibration mode (scale == 1)
		 */
		if (st->report)
			st->report--;
		if (!(st->dbg & CM_DBG_VAL_LIGHT))
			st->light = lux;
		st->ts = ts;
		cm_buf_push(indio_dev, st);
	}
	return ret;
}

static void cm_read(struct iio_dev *indio_dev)
{
	struct cm_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (st->enable) {
		cm_rd(indio_dev);
		schedule_delayed_work(&st->dw,
				      msecs_to_jiffies(st->queue_delay_ms));
	}
	mutex_unlock(&indio_dev->mlock);
}

static void cm_work(struct work_struct *ws)
{
	struct cm_state *st = container_of((struct delayed_work *)ws,
					   struct cm_state, dw);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);

	cm_read(indio_dev);
}

static int cm_disable(struct iio_dev *indio_dev)
{
	struct cm_state *st = iio_priv(indio_dev);
	int ret;

	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  CM_SCAN_LIGHT)))
		st->dbg &= ~CM_DBG_VAL_LIGHT;
	cancel_delayed_work(&st->dw);
	ret = cm_pm(st, false);
	if (!ret)
		st->enable = 0;
	return ret;
}

static int cm_enable(struct iio_dev *indio_dev)
{
	struct cm_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, CM_SCAN_LIGHT)) {
		ret = cm_pm(st, true);
		if (!ret) {
			ret = cm_it_wr(st, 0);
			if (ret) {
				cm_disable(indio_dev);
			} else {
				cm_delay(st, true);
				cm_report_init(st);
				st->enable = 1;
				schedule_delayed_work(&st->dw,
					     msecs_to_jiffies(CM_HW_DELAY_MS));
			}
		}
	}
	return ret;
}

static ssize_t cm_attr_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm_state *st = iio_priv(indio_dev);
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
	case CM_ATTR_ENABLE:
		msg = "ATTR_ENABLE";
		if (st->enable)
			old = 1;
		if (new)
			ret = cm_enable(indio_dev);
		else
			ret = cm_disable(indio_dev);
		break;

	case CM_ATTR_THRESH_LO:
		msg = "ATTR_THRESH_LO";
		old = st->lux_thr_lo;
		st->lux_thr_lo = new;
		break;

	case CM_ATTR_THRESH_HI:
		msg = "ATTR_THRESH_HI";
		old = st->lux_thr_hi;
		st->lux_thr_hi = new;
		break;

	default:
		msg = "ATTR_UNKNOWN";
		ret = -EINVAL;
	}

	cm_report_init(st);
	mutex_unlock(&indio_dev->mlock);
	cm_read(indio_dev);
	if (st->dbg & CM_DBG_SPEW_MSG) {
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

static ssize_t cm_attr_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	switch (this_attr->address) {
	case CM_ATTR_ENABLE:
		return sprintf(buf, "%x\n", st->enable);

	case CM_ATTR_PART:
		return sprintf(buf, "%s light\n", CM_NAME);

	case CM_ATTR_VENDOR:
		return sprintf(buf, "%s\n", CM_VENDOR);

	case CM_ATTR_VERSION:
		return sprintf(buf, "%u\n", CM_LIGHT_VERSION);

	case CM_ATTR_MILLIAMP:
		return sprintf(buf, "%s\n", CM_LIGHT_MILLIAMP);

	case CM_ATTR_THRESH_LO:
		return sprintf(buf, "%u\n", st->lux_thr_lo);

	case CM_ATTR_THRESH_HI:
		return sprintf(buf, "%u\n", st->lux_thr_hi);

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static ssize_t cm_data_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cm_state *st = iio_priv(dev_get_drvdata(dev));
	unsigned int info;
	int ret;

	ret = kstrtouint(buf, 10, &info);
	if (ret)
		return -EINVAL;

	if (info >= CM_INFO_LIMIT_MAX)
		return -EINVAL;

	st->info = info;
	switch (info) {
	case CM_INFO_DATA:
		st->dbg = 0;
		break;

	case CM_INFO_DBG:
		st->dbg ^= CM_DBG_SPEW_MSG;
		break;

	case CM_INFO_LIGHT_SPEW:
		st->dbg ^= CM_DBG_SPEW_LIGHT;
		break;

	case CM_INFO_LIGHT_POLL_SPEW:
		st->dbg ^= CM_DBG_SPEW_LIGHT_POLL;
		break;

	default:
		break;
	}

	return count;
}

static ssize_t cm_data_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct cm_state *st = iio_priv(dev_get_drvdata(dev));
	enum CM_INFO info;
	ssize_t t;

	info = st->info;
	st->info = CM_INFO_DATA;
	switch (info) {
	case CM_INFO_DATA:
		t = sprintf(buf, "LIGHT: %d  ts: %lld\n",
			    st->light, st->ts);
		return t;

	case CM_INFO_VER:
		return sprintf(buf, "version=%u\n", CM_VERSION_DRIVER);

	case CM_INFO_ERRS:
		t = sprintf(buf, "error count=%u\n", st->errs);
		st->errs = 0;
		return t;

	case CM_INFO_REGS:
		t = sprintf(buf, "registers:\n");
		t += sprintf(buf + t, "0x20=%#2x\n", st->cmd1);
		t += sprintf(buf + t, "0x22=%#2x\n", st->cmd2);
		return t;

	case CM_INFO_DBG:
		return sprintf(buf, "debug spew=%x\n",
			       !!(st->dbg & CM_DBG_SPEW_MSG));

	case CM_INFO_LIGHT_SPEW:
		return sprintf(buf, "lux_ts spew=%x\n",
			       !!(st->dbg & CM_DBG_SPEW_LIGHT));

	case CM_INFO_LIGHT_POLL_SPEW:
		return sprintf(buf, "lux_poll_ts spew=%x\n",
			       !!(st->dbg & CM_DBG_SPEW_LIGHT_POLL));

	default:
		break;
	}

	return -EINVAL;
}

static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		       cm_attr_show, cm_attr_store, CM_ATTR_ENABLE);
static IIO_DEVICE_ATTR(illuminance_part, S_IRUGO,
		       cm_attr_show, NULL, CM_ATTR_PART);
static IIO_DEVICE_ATTR(illuminance_vendor, S_IRUGO,
		       cm_attr_show, NULL, CM_ATTR_VENDOR);
static IIO_DEVICE_ATTR(illuminance_version, S_IRUGO,
		       cm_attr_show, NULL, CM_ATTR_VERSION);
static IIO_DEVICE_ATTR(illuminance_milliamp, S_IRUGO,
		       cm_attr_show, NULL, CM_ATTR_MILLIAMP);
static IIO_DEVICE_ATTR(illuminance_thresh_rising_value,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       cm_attr_show, cm_attr_store, CM_ATTR_THRESH_HI);
static IIO_DEVICE_ATTR(illuminance_thresh_falling_value,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       cm_attr_show, cm_attr_store, CM_ATTR_THRESH_LO);
static DEVICE_ATTR(data, S_IRUGO | S_IWUSR | S_IWGRP,
		   cm_data_show, cm_data_store);

static struct attribute *cm_attrs[] = {
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

static struct attribute_group cm_attr_group = {
	.name = CM_NAME,
	.attrs = cm_attrs
};

static int cm_read_raw(struct iio_dev *indio_dev,
		       struct iio_chan_spec const *chan,
		       int *val, int *val2, long mask)
{
	struct cm_state *st = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = st->light;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->enable)
			*val = st->poll_delay_ms * 1000; /* ms => us */
		else
			*val = cm_it_tbl[st->it_i_hi - 1].ms * 1000;
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
		*val = CM_LIGHT_MAX_RANGE_IVAL;
		*val2 = CM_LIGHT_MAX_RANGE_MICRO;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_PEAK_SCALE:
		*val = CM_LIGHT_RESOLUTION_IVAL;
		*val2 = CM_LIGHT_RESOLUTION_MICRO;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return ret;
}

static int cm_write_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int val, int val2, long mask)
{
	struct cm_state *st = iio_priv(indio_dev);
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
		cm_delay(st, false);
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
		st->ts = cm_get_time_ns(st);
		st->dbg |= CM_DBG_VAL_LIGHT;
		cm_buf_push(indio_dev, st);
		break;

	default:
		msg = "IIO_CHAN_INFO_UNKNOWN";
		ret = -EINVAL;
	}

	cm_report_init(st);
	mutex_unlock(&indio_dev->mlock);
	cm_read(indio_dev);
	if (st->dbg & CM_DBG_SPEW_MSG) {
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

static const struct iio_info cm_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &cm_attr_group,
	.read_raw = &cm_read_raw,
	.write_raw = &cm_write_raw,
};

static const struct iio_chan_spec cm_channels[] = {
	{
		.type			= IIO_LIGHT,
		.scan_index		= CM_SCAN_LIGHT,
		.scan_type		= IIO_ST('u', 32, 32, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
	},
	IIO_CHAN_SOFT_TIMESTAMP(CM_SCAN_TIMESTAMP)
};

static int cm_buffer_preenable(struct iio_dev *indio_dev)
{
	struct cm_state *st = iio_priv(indio_dev);

	if (st->shutdown || st->suspend)
		return -EINVAL;

	return 0;
}

static int cm_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;

	ret = cm_enable(indio_dev);
	/* never return > 0 to IIO buffer engine */
	if (ret > 0)
		ret = 0;
	return ret;
}

static const struct iio_buffer_setup_ops cm_buffer_setup_ops = {
	/* iio_sw_buffer_preenable:
	 * Generic function for equal sized ring elements + 64 bit timestamp
	 * Assumes that any combination of channels can be enabled.
	 * Typically replaced to implement restrictions on what combinations
	 * can be captured (hardware scan modes).
	 */
	.preenable = &cm_buffer_preenable,
	/* iio_triggered_buffer_postenable:
	 * Generic function that simply attaches the pollfunc to the trigger.
	 * Replace this to mess with hardware state before we attach the
	 * trigger.
	 */
	.postenable = &cm_buffer_postenable,
	/* this driver relies on the NVS HAL to power off this device with the
	 * master enable.
	 *.predisable = N/A
	 *.postdisable = N/A
	 */
};

static const struct iio_trigger_ops cm_trigger_ops = {
	.owner = THIS_MODULE,
};

static int cm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm_state *st = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (st->enable)
		ret = cm_disable(indio_dev);
	st->suspend = true;
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & CM_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int cm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm_state *st = iio_priv(indio_dev);

	st->suspend = false;
	if (st->dbg & CM_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(cm_pm_ops, cm_suspend, cm_resume);

static void cm_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (st->enable)
		cm_disable(indio_dev);
	st->shutdown = true;
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & CM_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int cm_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm_state *st = iio_priv(indio_dev);

	if (st != NULL) {
		cm_shutdown(client);
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
		cm_pm_exit(st);
		iio_device_free(indio_dev);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int cm_of_dt(struct i2c_client *client, struct cm_state *st)
{
	struct device_node *dn = client->dev.of_node;
	unsigned int val;
	unsigned int i;

	/* default NVS ALS programmable parameters */
	st->scale_val = CM_LIGHT_SCALE_IVAL;
	st->scale_val2 = CM_LIGHT_SCALE_MICRO;
	st->offset_val = CM_LIGHT_OFFSET_IVAL;
	st->offset_val2 = CM_LIGHT_OFFSET_MICRO;
	st->lux_thr_lo = (1000000 / st->scale_val2) * CM_THRESHOLD_LUX_DFLT;
	st->lux_thr_hi = (1000000 / st->scale_val2) * CM_THRESHOLD_LUX_DFLT;
	st->it_i_lo = 0;
	st->it_i_hi = ARRAY_SIZE(cm_it_tbl);
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
		if (of_property_read_u32(dn, "light_threshold_lo", &val))
			st->lux_thr_lo = val;
		if (of_property_read_u32(dn, "light_threshold_hi", &val))
			st->lux_thr_hi = val;
		/* this device supports these programmable parameters */
		if (of_property_read_u32(dn, "light_integration_time_ms_lo",
					 &val)) {
			for (i = ARRAY_SIZE(cm_it_tbl); i > 1; i--) {
				if (val <= cm_it_tbl[i - 1].ms)
					break;
			}
			st->it_i_hi = i;
		}
		if (of_property_read_u32(dn, "light_integration_time_ms_hi",
					 &val)) {
			for (i = 0; i < ARRAY_SIZE(cm_it_tbl) - 1; i++) {
				if (val >= cm_it_tbl[i].ms)
					break;
			}
			st->it_i_lo = i;
		}
		if (st->it_i_hi < st->it_i_lo) {
			dev_err(&client->dev,
				"%s light_integration_time_ms_ ERR\n",
				__func__);
			st->it_i_lo = 0;
			st->it_i_hi = ARRAY_SIZE(cm_it_tbl);
		}
	}
	if (!st->report_n)
		st->report_n = CM_REPORT_N;
	return 0;
}

static int cm_probe(struct i2c_client *client,
		    const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct cm_state *st;
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
	ret = cm_of_dt(client, st);
	if (ret) {
		ret = -ENODEV;
		goto cm_probe_err;
	}

	ret = cm_pm_init(st);
	if (ret) {
		ret = -ENODEV;
		goto cm_probe_exit;
	}

	indio_dev->buffer = iio_kfifo_allocate(indio_dev);
	if (!indio_dev->buffer) {
		dev_err(&client->dev, "%s iio_kfifo_allocate ERR\n", __func__);
		ret = -ENOMEM;
		goto cm_probe_err;
	}

	indio_dev->buffer->scan_timestamp = true;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = cm_channels;
	indio_dev->num_channels = ARRAY_SIZE(cm_channels);
	indio_dev->name = CM_NAME;
	indio_dev->info = &cm_iio_info;
	indio_dev->setup_ops = &cm_buffer_setup_ops;
	ret = iio_buffer_register(indio_dev, indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto cm_probe_err;

	INIT_DELAYED_WORK(&st->dw, cm_work);
	st->trig = iio_trigger_alloc("%s-dev%d",
				     indio_dev->name, indio_dev->id);
	if (st->trig == NULL) {
		dev_err(&client->dev, "%s iio_allocate_trigger ERR\n",
			__func__);
		ret = -ENOMEM;
		goto cm_probe_err;
	}

	st->trig->dev.parent = &st->i2c->dev;
	st->trig->ops = &cm_trigger_ops;
	ret = iio_trigger_register(st->trig);
	if (ret) {
		dev_err(&client->dev, "%s iio_trigger_register ERR\n",
			__func__);
		ret = -ENOMEM;
		goto cm_probe_err;
	}

	indio_dev->trig = st->trig;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_device_register(indio_dev);
	if (ret)
		goto cm_probe_err;

	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

cm_probe_err:
	dev_err(&client->dev, "%s ERR %d\n", __func__, ret);
cm_probe_exit:
	cm_remove(client);
	return ret;
}

static const struct i2c_device_id cm_i2c_device_id[] = {
	{ CM_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, cm_i2c_device_id);

static const struct of_device_id cm_of_match[] = {
	{ .compatible = "capella,cm3217", },
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
MODULE_DESCRIPTION("CM3217 driver");
MODULE_AUTHOR("NVIDIA Corporation");
