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
/* NVS proximity drivers can be configured for binary output.  If the
 * proximity_binary_threshold setting in the device tree is set, the driver
 * will configure the rest of the settings so that a 1 is reported for
 * "far away" and 0 for "near".  The threshold is typically set for maximum
 * range allowing the minimal LED drive power to determine the actual range.
 * To disable binary output, set the proximity_binary_threshold to 0.  The
 * driver will then require the interpolation calibration for reporting actual
 * distances.
 */
/* Calibration for proximity_binary_threshold is done with the following steps:
 * 1. Disable proximity.
 * 2. Write 1 to the scale sysfs attribute.
 * 3. Enable device.
 * 4. The NVS HAL will announce in the log that calibration mode is enabled and
 *    display the raw HW proximity values instead of the 0 or 1.
 * 5. Move a surface within the range of the proximity where the output
 *    state should change.  There should be two places: the transition from
 *    1 to 0 and vice versa.  The middle of these two points will be the
 *    proximity_binary_threshold and the difference between each point and
 *    the proximity_binary_threshold will be the proximity_threshold_lo and
 *    proximity_threshold_hi settings in the device tree which will allow
 *    hysteresis around the threshold and can be skewed as such.
 * 6. Write 0 to the scale sysfs attribute.
 * 7. Disabling the device will turn off this calibration mode.
 * Note that proximity_threshold_lo and proximity_threshold_hi do not have to
 * be identical and can in fact result in being outside the HW range:
 * Example: HW values range from 0 to 255.
 *          proximity_binary_threshold is set to 1.
 *          proximity_threshold_lo is set to 10.
 *          proximity_threshold_hi is set to 20.
 *          In this configuration, the low state changes at 0 and the
 *          high at 20.
 * The driver will automatically handle out of bounds values.
 * Also, the proximity_thresh_falling_value and proximity_thresh_rising_value
 * sysfs attributes can change the proximity_threshold_lo/hi values at runtime
 * for further calibration flexibility.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>


#define MX_VERSION_DRIVER		(101)
#define MX_VENDOR			"Maxim"
#define MX_NAME				"max4400x"
#define MX_NAME_MAX44005		"max44005"
#define MX_NAME_MAX44006		"max44006"
#define MX_NAME_MAX44008		"max44008"
#define MX_DEVID_MAX44005		(0x05)
#define MX_DEVID_MAX44006		(0x06)
#define MX_DEVID_MAX44008		(0x08)
#define MX_HW_DELAY_MS			(1)
#define MX_POLL_DLY_MS_DFLT		(2000)
#define MX_POLL_DLY_MS_MIN		(100)
#define MX_THRESHOLD_LUX_DFLT		(50)
#define MX_THRESHOLD_PROX_DFLT		(10)
#define MX_THRESHOLD_PROX_BINARY_DFLT	(1)
#define MX_AMB_CFG_DFLT			(0x43)
#define MX_PRX_CFG_DFLT			(0x12)
/* light defines */
#define MX_LIGHT_VERSION		(1)
#define MX_LIGHT_MAX_RANGE_IVAL		(14323)
#define MX_LIGHT_MAX_RANGE_MICRO	(0)
#define MX_LIGHT_RESOLUTION_IVAL	(0)
#define MX_LIGHT_RESOLUTION_MICRO	(14000)
#define MX_LIGHT_SCALE_IVAL		(0)
#define MX_LIGHT_SCALE_MICRO		(1000)
#define MX_LIGHT_OFFSET_IVAL		(0)
#define MX_LIGHT_OFFSET_MICRO		(0)
#define MX_LIGHT_MILLIAMP		"0.0135"
/* temp defines */
#define MX_TEMP_VERSION			(1)
#define MX_TEMP_MAX_RANGE_IVAL		(125)
#define MX_TEMP_MAX_RANGE_MICRO		(0)
#define MX_TEMP_RESOLUTION_IVAL		(0)
#define MX_TEMP_RESOLUTION_MICRO	(250000)
#define MX_TEMP_SCALE_IVAL		(0)
#define MX_TEMP_SCALE_MICRO		(10000)
#define MX_TEMP_OFFSET_IVAL		(0)
#define MX_TEMP_OFFSET_MICRO		(0)
#define MX_TEMP_MILLIAMP		"0.0135"
/* proximity defines */
#define MX_PROX_VERSION			(1)
#define MX_PROX_MAX_RANGE_IVAL		(1)
#define MX_PROX_MAX_RANGE_MICRO		(0)
#define MX_PROX_RESOLUTION_IVAL		(1)
#define MX_PROX_RESOLUTION_MICRO	(0)
#define MX_PROX_SCALE_IVAL		(0)
#define MX_PROX_SCALE_MICRO		(0)
#define MX_PROX_OFFSET_IVAL		(0)
#define MX_PROX_OFFSET_MICRO		(0)
#define MX_PROX_MILLIAMP		"10.0195"
/* HW registers */
#define MX_REG_STS			(0x00)
#define MX_REG_STS_RESET		(4)
#define MX_REG_STS_SHDN			(3)
#define MX_REG_STS_PWRON		(2)
#define MX_REG_STS_PRXINTS		(1)
#define MX_REG_STS_AMBINTS		(0)
#define MX_REG_CFG_MAIN			(0x01)
#define MX_REG_CFG_MAIN_POR		(0x00)
#define MX_REG_CFG_MAIN_MODE		(4)
#define MX_REG_CFG_MAIN_AMBSEL		(2)
#define MX_REG_CFG_MAIN_PRXINTE		(1)
#define MX_REG_CFG_MAIN_AMBINTE		(0)
#define MX_REG_CFG_AMB			(0x02)
#define MX_REG_CFG_AMB_POR		(0x20)
#define MX_REG_CFG_AMB_TRIM		(7)
#define MX_REG_CFG_AMB_COMPEN		(6)
#define MX_REG_CFG_AMB_TEMPEN		(5)
#define MX_REG_CFG_AMB_AMBTIM		(2)
#define MX_REG_CFG_AMB_AMBTIM_MASK	(0x1C)
#define MX_REG_CFG_AMB_AMBPGA		(0)
#define MX_REG_CFG_AMB_AMBPGA_MASK	(0x03)
#define MX_REG_CFG_PRX			(0x03)
#define MX_REG_CFG_PRX_DRV		(4)
#define MX_REG_CFG_PRX_PRXTIM		(1)
#define MX_REG_CFG_PRX_PRXPGA		(0)
#define MX_REG_DATA_AMB_CLEAR		(0x04)
#define MX_REG_DATA_AMB_CLEAR_H		(0x04)
#define MX_REG_DATA_AMB_CLEAR_L		(0x05)
#define MX_REG_DATA_AMB_RED_H		(0x06)
#define MX_REG_DATA_AMB_RED_L		(0x07)
#define MX_REG_DATA_AMB_GREEN_H		(0x08)
#define MX_REG_DATA_AMB_GREEN_L		(0x09)
#define MX_REG_DATA_AMB_BLUE_H		(0x0A)
#define MX_REG_DATA_AMB_BLUE_L		(0x0B)
#define MX_REG_DATA_AMB_IR_H		(0x0C)
#define MX_REG_DATA_AMB_IR_L		(0x0D)
#define MX_REG_DATA_AMB_IRCOMP_H	(0x0E)
#define MX_REG_DATA_AMB_IRCOMP_L	(0x0F)
#define MX_REG_DATA_PROX_H		(0x10)
#define MX_REG_DATA_PROX_L		(0x11)
#define MX_REG_DATA_TEMP_H		(0x12)
#define MX_REG_DATA_TEMP_L		(0x13)
#define MX_REG_AMB_UPTHR_H		(0x14)
#define MX_REG_AMB_UPTHR_L		(0x15)
#define MX_REG_AMB_LOTHR_H		(0x16)
#define MX_REG_AMB_LOTHR_L		(0x17)
#define MX_REG_CFG_THR			(0x18)
#define MX_REG_CFG_THR_PRXPST		(2)
#define MX_REG_CFG_THR_AMBPST		(0)
#define MX_REG_PRX_UPTHR_H		(0x19)
#define MX_REG_PRX_UPTHR_L		(0x1A)
#define MX_REG_PRX_LOTHR_H		(0x1B)
#define MX_REG_PRX_LOTHR_L		(0x1C)
#define MX_REG_TRIM_CLEAR		(0x1D)
#define MX_REG_TRIM_RED			(0x1E)
#define MX_REG_TRIM_GREEN		(0x1F)
#define MX_REG_TRIM_BLUE		(0x20)
#define MX_REG_TRIM_IR			(0x21)
/* _buf_push expects this scan order */
#define MX_SCAN_LIGHT			(0)
#define MX_SCAN_TEMP			(1)
#define MX_SCAN_PROX			(2)
#define MX_SCAN_TIMESTAMP		(3)
#define MX_DEV_LIGHT			(MX_SCAN_LIGHT)
#define MX_DEV_TEMP			(MX_SCAN_TEMP)
#define MX_DEV_PROX			(MX_SCAN_PROX)
#define MX_DEV_N			(MX_SCAN_TIMESTAMP)
/* debug flags */
#define MX_DBG_SPEW_MSG			(1 << 0)
#define MX_DBG_SPEW_LIGHT		(1 << 1)
#define MX_DBG_SPEW_LIGHT_POLL		(1 << 2)
#define MX_DBG_SPEW_TEMP		(1 << 3)
#define MX_DBG_SPEW_TEMP_POLL		(1 << 4)
#define MX_DBG_SPEW_PROX		(1 << 5)
#define MX_DBG_SPEW_PROX_POLL		(1 << 6)
#define MX_DBG_IRQ			(1 << 7)
#define MX_DBG_VAL_LIGHT		(1 << 8)
#define MX_DBG_VAL_TEMP			(1 << 9)
#define MX_DBG_VAL_PROX			(1 << 10)

enum MX_ATTR {
	MX_ATTR_ENABLE,
	MX_ATTR_VENDOR,
	MX_ATTR_LIGHT_PART,
	MX_ATTR_LIGHT_VERSION,
	MX_ATTR_LIGHT_MILLIAMP,
	MX_ATTR_LIGHT_THRESH_LO,
	MX_ATTR_LIGHT_THRESH_HI,
	MX_ATTR_TEMP_PART,
	MX_ATTR_TEMP_VERSION,
	MX_ATTR_TEMP_MILLIAMP,
	MX_ATTR_PROX_PART,
	MX_ATTR_PROX_VERSION,
	MX_ATTR_PROX_MILLIAMP,
	MX_ATTR_PROX_THRESH_LO,
	MX_ATTR_PROX_THRESH_HI,
};

enum MX_INFO {
	MX_INFO_DATA = 0,
	MX_INFO_VER,
	MX_INFO_ERRS,
	MX_INFO_REGS,
	MX_INFO_DBG,
	MX_INFO_LIGHT_SPEW,
	MX_INFO_LIGHT_POLL_SPEW,
	MX_INFO_TEMP_SPEW,
	MX_INFO_TEMP_POLL_SPEW,
	MX_INFO_PROXIMITY_SPEW,
	MX_INFO_PROXIMITY_POLL_SPEW,
	MX_INFO_DBG_IRQ,
	MX_INFO_LIMIT_MAX,
};

/* regulator names in order of powering on */
static char *mx_vregs[] = {
	"vdd",
};

static unsigned short mx_i2c_addrs[] = {
	0x40,
	0x41,
	0x44,
	0x45,
};

struct mx_iio_float {
	int ival;
	int micro;
};

static u8 mx_mode_tbl[] = {		/* device enable */
	0x00,				/* nothing */
	0x00,				/* light */
	0x00,				/* temp */
	0x00,				/* temp + light */
	0x05,				/* proximity */
	0x03,				/* proximity + light */
	0x03,				/* proximity + temp */
	0x03				/* proximity + temp + light */
};

/* 1 nW/cm^2 = 0.00683 lux */
static unsigned int mx_ambpga_44005[] = {
	/* values to be scaled by val2 */
	20490,				/* 0.02049 lux / LSb */
	81960,				/* 0.08196 lux / LSb */
	327680,				/* 0.32768 lux / LSb */
	5245440				/* 5.24544 lux / LSb */
};

static unsigned int mx_ambpga_44006[] = {
	/* values to be scaled by val2 */
	13660,				/* 0.01366 lux / LSb */
	54640,				/* 0.05464 lux / LSb */
	218560,				/* 0.21856 lux / LSb */
	3496960				/* 3.49696 lux / LSb */
};

static unsigned int mx_ambtim_mask[] = {
	0x3FFF,				/* 14 bits */
	0x0FFF,				/* 12 bits */
	0x03FF,				/* 10 bits */
	0x00FF,				/* 8 bits */
	0x3FFF,				/* 14 bits */
	0x3FFF,				/* N/A */
	0x3FFF,				/* N/A */
	0x3FFF,				/* N/A */
};

struct mx_state {
	struct i2c_client *i2c;
	struct iio_trigger *trig;
	struct delayed_work dw;
	struct regulator_bulk_data vreg[ARRAY_SIZE(mx_vregs)];
	unsigned int info;		/* info data to return */
	unsigned int dbg;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int enable;		/* enable status */
	unsigned int poll_delay_ms;	/* requested sampling delay (ms) */
	unsigned int delay_us[MX_DEV_N]; /* device sampling delay */
	unsigned int scale_i;		/* index into HW IT settings table */
	int scale_val[MX_DEV_N];	/* user scale val */
	int scale_val2[MX_DEV_N];	/* user scale val2 */
	int offset_val[MX_DEV_N];	/* user offset val */
	int offset_val2[MX_DEV_N];	/* user offset val2 */
	int peak_val[MX_DEV_N];		/* IIO_VAL_INT_PLUS_MICRO max range */
	int peak_val2[MX_DEV_N];	/* IIO_VAL_INT_PLUS_MICRO max range */
	int peak_scale_val[MX_DEV_N];	/* IIO_VAL_INT_PLUS_MICRO resolution */
	int peak_scale_val2[MX_DEV_N];	/* IIO_VAL_INT_PLUS_MICRO resolution */
	int i_uc_lo[MX_DEV_N];		/* interpolation x1 uncalibrated lo */
	int i_uc_hi[MX_DEV_N];		/* interpolation x3 uncalibrated hi */
	int i_c_lo[MX_DEV_N];		/* interpolation y1 calibrated lo */
	int i_c_hi[MX_DEV_N];		/* interpolation y3 calibrated hi */
	unsigned int report;		/* used to report first valid sample */
	unsigned int report_n;		/* this many on-change data reports */
	unsigned int lux_thr_lo;	/* report when lux below this */
	unsigned int lux_thr_hi;	/* report when lux above this */
	unsigned int prx_thr_lo;	/* report when proximity below this */
	unsigned int prx_thr_hi;	/* report when proximity above this */
	unsigned int prx_thr_bin;	/* proximity binary mode threshold */
	bool shutdown;			/* shutdown active flag */
	bool suspend;			/* suspend active flag */
	const char *part;		/* part name */
	u16 i2c_addr;			/* I2C address */
	u8 dev_id;			/* device ID */
	u8 amb_cfg;			/* ambient configuration register */
	u8 prx_cfg;			/* proximity configuration register */
	u8 thr_cfg;			/* threshold persist register */
	u8 rc_main_cfg;			/* cache of main configuration */
	u8 rc_amb_cfg;			/* cache of ambient configuration */
	u32 light;			/* sample light data */
	s16 temp;			/* sample temp data */
	u32 proximity;			/* reported proximity data */
	u32 prox;			/* actual proximity data */
	s64 ts;				/* sample data timestamp */
};


static void mx_err(struct mx_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

static int mx_i2c_read(struct mx_state *st, u8 reg, u16 len, u8 *val)
{
	struct i2c_msg msg[2];

	msg[0].addr = st->i2c_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = st->i2c_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = val;
	if (i2c_transfer(st->i2c->adapter, msg, 2) != 2) {
		mx_err(st);
		return -EIO;
	}

	return 0;
}

static int mx_i2c_rd(struct mx_state *st, u8 reg, u8 *val)
{
	return mx_i2c_read(st, reg, 1, val);
}

static int mx_i2c_write(struct mx_state *st, u16 len, u8 *buf)
{
	struct i2c_msg msg;

	msg.addr = st->i2c_addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = buf;
	if (i2c_transfer(st->i2c->adapter, &msg, 1) != 1) {
		mx_err(st);
		return -EIO;
	}

	return 0;
}

static int mx_i2c_wr(struct mx_state *st, u8 reg, u8 val)
{
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;
	return mx_i2c_write(st, sizeof(buf), buf);
}

static int mx_vreg_dis(struct mx_state *st, unsigned int i)
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

static int mx_vreg_dis_all(struct mx_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = ARRAY_SIZE(mx_vregs); i > 0; i--)
		ret |= mx_vreg_dis(st, (i - 1));
	return ret;
}

static int mx_vreg_en(struct mx_state *st, unsigned int i)
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

static int mx_vreg_en_all(struct mx_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(mx_vregs); i++)
		ret |= mx_vreg_en(st, i);
	return ret;
}

static void mx_vreg_exit(struct mx_state *st)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mx_vregs); i++) {
		if (st->vreg[i].consumer != NULL) {
			devm_regulator_put(st->vreg[i].consumer);
			st->vreg[i].consumer = NULL;
			dev_dbg(&st->i2c->dev, "%s %s\n",
				__func__, st->vreg[i].supply);
		}
	}
}

static int mx_vreg_init(struct mx_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(mx_vregs); i++) {
		st->vreg[i].supply = mx_vregs[i];
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

static int mx_vreg_sts(struct mx_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(mx_vregs); i++) {
		if (st->vreg[i].consumer != NULL)
			break;
	}
	if (i < ARRAY_SIZE(mx_vregs)) {
		/* ret == number of regulators on */
		for (i = 0; i < ARRAY_SIZE(mx_vregs); i++) {
			if (st->vreg[i].ret)
				ret++;
		}
	} else {
		/* no regulator support (can assume always on) */
		ret = -EINVAL;
	}
	return ret;
}

static int mx_pm(struct mx_state *st, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = mx_vreg_en_all(st);
		if (ret) {
			mdelay(MX_HW_DELAY_MS);
			mx_i2c_wr(st, MX_REG_STS, (1 << MX_REG_STS_RESET));
			mdelay(MX_HW_DELAY_MS);
			st->rc_main_cfg = MX_REG_CFG_MAIN_POR;
			st->rc_amb_cfg = MX_REG_CFG_AMB_POR;
		}
		mx_i2c_wr(st, MX_REG_STS, 0);
	} else {
		ret = mx_vreg_sts(st);
		if ((ret < 0) || (ret == ARRAY_SIZE(mx_vregs))) {
			ret = mx_i2c_wr(st, MX_REG_STS, 1 << MX_REG_STS_SHDN);
		} else if (ret > 0) {
			mx_vreg_en_all(st);
			mdelay(MX_HW_DELAY_MS);
			ret = mx_i2c_wr(st, MX_REG_STS, 1 << MX_REG_STS_SHDN);
		}
		ret |= mx_vreg_dis_all(st);
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		dev_err(&st->i2c->dev, "%s pwr=%x ERR=%d\n",
			__func__, enable, ret);
	} else {
		if (st->dbg & MX_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s pwr=%x\n",
				 __func__, enable);
	}
	return ret;
}

static void mx_pm_exit(struct mx_state *st)
{
	mx_pm(st, false);
	mx_vreg_exit(st);
}

static int mx_pm_init(struct mx_state *st)
{
	int ret;

	st->enable = 0;
	st->poll_delay_ms = MX_POLL_DLY_MS_DFLT;
	mx_vreg_init(st);
	ret = mx_pm(st, true);
	return ret;
}

static unsigned int mx_buf_index(unsigned int size, unsigned int *bytes)
{
	unsigned int index;

	if (!(*bytes % size))
		index = *bytes;
	else
		index = *bytes - *bytes % size + size;
	*bytes = index + size;
	return index;
}

static void mx_buf_push(struct iio_dev *indio_dev)
{
	struct mx_state *st = iio_priv(indio_dev);
	unsigned char buf[24];
	unsigned int n;
	unsigned int i;
	unsigned int bytes = 0;

	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, MX_SCAN_LIGHT)) {
		n = sizeof(st->light);
		i = mx_buf_index(n, &bytes);
		memcpy(&buf[i], &st->light, n);
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, MX_SCAN_TEMP)) {
		n = sizeof(st->temp);
		i = mx_buf_index(n, &bytes);
		memcpy(&buf[i], &st->temp, n);
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, MX_SCAN_PROX)) {
		n = sizeof(st->proximity);
		i = mx_buf_index(n, &bytes);
		memcpy(&buf[i], &st->proximity, n);
	}
	if (indio_dev->buffer->scan_timestamp) {
		n = sizeof(st->ts);
		i = mx_buf_index(n, &bytes);
		memcpy(&buf[i], &st->ts, n);
	}
	if (iio_buffer_enabled(indio_dev)) {
		iio_push_to_buffers(indio_dev, buf);
		if (st->dbg & MX_DBG_SPEW_LIGHT)
			dev_info(&st->i2c->dev, "light: %u %lld\n",
				 st->light, st->ts);
		if (st->dbg & MX_DBG_SPEW_TEMP)
			dev_info(&st->i2c->dev, "temp: %hd %lld\n",
				 st->temp, st->ts);
		if (st->dbg & MX_DBG_SPEW_PROX)
			dev_info(&st->i2c->dev, "proximity: %u %lld\n",
				 st->proximity, st->ts);
	}
}

static int mx_cmd_wr(struct mx_state *st, unsigned int enable, bool irq_en)
{
	u8 amb_cfg = st->amb_cfg;
	u8 main_cfg = 0;
	int ret;
	int ret_t = 0;

	if (enable & (1 << MX_DEV_TEMP))
		amb_cfg |= (1 << MX_REG_CFG_AMB_TEMPEN);
	if (amb_cfg != st->rc_amb_cfg) {
		ret = mx_i2c_wr(st, MX_REG_CFG_AMB, amb_cfg);
		if (ret)
			ret_t |= ret;
		else
			st->rc_amb_cfg = amb_cfg;
		if (st->dbg & MX_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s amb_cfg=%hhx err=%d\n",
				 __func__, amb_cfg, ret);
	}
	main_cfg = mx_mode_tbl[enable] << MX_REG_CFG_MAIN_MODE;
	if (irq_en) {
		if (enable & (1 << MX_DEV_LIGHT))
			main_cfg |= (1 << MX_REG_CFG_MAIN_AMBINTE);
		if (enable & (1 << MX_DEV_PROX))
			main_cfg |= (1 << MX_REG_CFG_MAIN_PRXINTE);
	}
	if (main_cfg != st->rc_main_cfg) {
		ret = mx_i2c_wr(st, MX_REG_CFG_MAIN, main_cfg);
		if (ret)
			ret_t |= ret;
		else
			st->rc_main_cfg = main_cfg;
		if (st->dbg & MX_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s main_cfg=%hhx err=%d\n",
				 __func__, main_cfg, ret);
	}
	if (irq_en && !ret_t)
		ret_t = 1; /* flag IRQ enabled */
	return ret_t;
}

static void mx_delay(struct mx_state *st)
{
	unsigned int i;
	unsigned int delay_us = ~0;

	for (i = 0; i < MX_DEV_N; i++) {
		if (st->enable & (1 << i)) {
			if (st->delay_us[i] && (st->delay_us[i] < delay_us))
				delay_us = st->delay_us[i];
		}
	}
	if (delay_us == ~0)
		delay_us = MX_POLL_DLY_MS_DFLT * 1000;
	else if (delay_us < (MX_POLL_DLY_MS_MIN * 1000))
		delay_us = MX_POLL_DLY_MS_MIN * 1000;
	st->poll_delay_ms = delay_us / 1000;
}

static int mx_interpolate(int x1, int x2, int x3, int y1, int *y2, int y3)
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

static void mx_report_init(struct mx_state *st)
{
	st->ts = iio_get_time_ns();
	if (st->report_n)
		st->report = st->report_n;
	else
		st->report = 1;
}

static int mx_thr_wr(struct mx_state *st, u8 reg, u16 thr_lo, u16 thr_hi)
{
	u8 buf[5];
	u16 thr_be;
	int ret = 0;

	if (st->i2c->irq) {
		buf[0] = reg;
		thr_be = cpu_to_be16(thr_hi);
		buf[1] = thr_be & 0xFF;
		buf[2] = thr_be >> 8;
		thr_be = cpu_to_be16(thr_lo);
		buf[3] = thr_be & 0xFF;
		buf[4] = thr_be >> 8;
		ret = mx_i2c_write(st, sizeof(buf), buf);
		if (st->dbg & MX_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev,
				 "%s reg=%hhx lo=%hd hi=%hd ret=%d\n",
				 __func__, reg, thr_lo, thr_hi, ret);
	}
	return ret;
}

static int mx_thr(struct mx_state *st, unsigned int dev, u8 reg, unsigned lsb,
		  int val_new, int val_old, int thr_lo, int thr_hi, u16 hw_max)
{
	u64 calc;
	int ret = 0;

	if (thr_lo || thr_hi) {
		if (val_new > (val_old + thr_hi)) {
			ret = 1;
		} else if (val_old > thr_lo) {
			if (val_new < (val_old - thr_lo))
				ret = 1;
		}
		if (ret) {
			thr_lo = val_new - thr_lo;
			/* get the uncalibrated value */
			mx_interpolate(st->i_c_lo[dev], thr_lo,
				       st->i_c_hi[dev],
				       st->i_uc_lo[dev], &thr_lo,
				       st->i_uc_hi[dev]);
			if (thr_lo < 0)
				thr_lo = 0;
			/* convert to HW value */
			calc = thr_lo;
			if (st->scale_val2[dev])
				calc *= st->scale_val2[dev];
			if (lsb)
				do_div(calc, lsb);
			thr_lo = calc;
			thr_hi = val_new + thr_hi;
			/* get the uncalibrated value */
			mx_interpolate(st->i_c_lo[dev], thr_hi,
				       st->i_c_hi[dev],
				       st->i_uc_lo[dev], &thr_hi,
				       st->i_uc_hi[dev]);
			/* convert to HW value */
			calc = thr_hi;
			if (st->scale_val2[dev])
				calc *= st->scale_val2[dev];
			if (lsb)
				do_div(calc, lsb);
			thr_hi = calc;
			if (thr_hi > hw_max)
				thr_hi = hw_max;
			ret = mx_thr_wr(st, reg, thr_lo, thr_hi);
			if (ret)
				ret = 2; /* poll mode */
			else
				ret = 1; /* flag report */
		}
	} else if (val_new != val_old) {
		ret = 2; /* poll mode */
	}
	return ret;
}

static int mx_rd_light(struct mx_state *st, s64 ts)
{
	u16 lux;
	u16 lux_max;
	u64 calc;
	u32 light;
	unsigned int lsb;
	int i;
	int ret;

	ret = mx_i2c_read(st, MX_REG_DATA_AMB_CLEAR, 2, (u8 *)&lux);
	if (ret)
		return ret;

	lux = be16_to_cpu(lux);
	i = st->rc_amb_cfg & MX_REG_CFG_AMB_AMBPGA_MASK;
	if (st->dev_id == MX_DEVID_MAX44005)
		lsb = mx_ambpga_44005[i];
	else
		lsb = mx_ambpga_44006[i];
	calc = lsb;
	calc *= lux;
	if (st->scale_val2[MX_DEV_LIGHT])
		do_div(calc, st->scale_val2[MX_DEV_LIGHT]);
	light = calc;
	/* get calibrated value */
	mx_interpolate(st->i_uc_lo[MX_DEV_LIGHT], light,
		       st->i_uc_hi[MX_DEV_LIGHT],
		       st->i_c_lo[MX_DEV_LIGHT], &light,
		       st->i_c_hi[MX_DEV_LIGHT]);
	i = st->rc_amb_cfg & MX_REG_CFG_AMB_AMBTIM_MASK;
	i >>= MX_REG_CFG_AMB_AMBTIM;
	lux_max = mx_ambtim_mask[i];
	ret = mx_thr(st, MX_DEV_LIGHT, MX_REG_AMB_UPTHR_H, lsb, light,
		     st->light, st->lux_thr_lo, st->lux_thr_hi, lux_max);
	if ((ret > 0) && !(st->dbg & MX_DBG_VAL_LIGHT))
		st->light = light;
	if (st->dbg & MX_DBG_SPEW_LIGHT_POLL) {
		dev_info(&st->i2c->dev,
			 "poll light %u %lld  diff=%d %lldns  hw=%hu\n",
			 light, ts, light - st->light, ts - st->ts, lux);
		if (ret > 0)
			ret = 2; /* poll mode */
		else
			ret = -1; /* poll without report */
	}
	return ret;
}

static int mx_rd_temp(struct mx_state *st, s64 ts)
{
	u16 temp;
	int ret;

	ret = mx_i2c_read(st, MX_REG_DATA_TEMP_H, 2, (u8 *)&temp);
	if (ret)
		return ret;

	temp = be16_to_cpu(temp);
	if (!(st->dbg & MX_DBG_VAL_TEMP))
		st->temp = temp;
	if (st->dbg & MX_DBG_SPEW_TEMP_POLL) {
		dev_info(&st->i2c->dev,
			 "poll temp %d %lld  diff=%d %lldns  hw=%hd\n",
			 temp, ts, temp - st->temp, ts - st->ts, temp);
		if (ret > 0)
			ret = 2; /* poll mode */
		else
			ret = -1; /* poll without report */
	}
	return 1;
}

static int mx_rd_prox(struct mx_state *st, s64 ts)
{
	u16 prox;
	u16 prox_lo;
	u16 prox_hi;
	unsigned int proximity;
	int ret = 0;

	ret = mx_i2c_read(st, MX_REG_DATA_PROX_H, 2, (u8 *)&prox);
	if (ret)
		return ret;

	st->prox = be16_to_cpu(prox);
	if (st->prx_cfg & (1 << MX_REG_CFG_PRX_PRXTIM))
		prox_hi = 0x00FF;
	else
		prox_hi = 0x03FF;
	if (st->prx_thr_bin) {
		/* proximity has binary threshold */
		if ((st->scale_val[MX_DEV_PROX] == 1) &&
					  (st->scale_val2[MX_DEV_PROX] == 0)) {
			/* binary proximity in test mode - display raw data */
			proximity = st->prox;
			ret = 2; /* flag report and poll */
		} else {
			if (st->prox > st->prx_thr_bin) {
				proximity = 0;
				if (st->prx_thr_bin > st->prx_thr_lo)
					prox_lo = st->prx_thr_bin -
								st->prx_thr_lo;
				else
					prox_lo = 1;
				/* prox_hi already disabled */
			} else {
				proximity = 1;
				prox_lo = 0; /* disable */
				prox = st->prx_thr_bin + st->prx_thr_hi;
				if (prox < prox_hi)
					prox_hi = prox;
				else
					prox_hi--;
			}
			if (proximity != st->proximity) {
				ret = mx_thr_wr(st, MX_REG_PRX_UPTHR_H,
						prox_lo, prox_hi);
				if (ret)
					ret = 2; /* flag report */
				else
					ret = 1; /* flag report */
			}
		}
	} else {
		/* FIXME */
		/* reverse the lo/hi thresholds */
		ret = mx_thr(st, MX_DEV_PROX, MX_REG_PRX_UPTHR_H, 0,
			     st->prox, st->proximity,
			     st->prx_thr_hi, st->prx_thr_lo, prox_hi);
		/* reverse the high/low order */
		proximity = prox_hi - st->prox;
	}
	if ((ret > 0) && !(st->dbg & MX_DBG_VAL_PROX))
		st->proximity = proximity;
	if (st->dbg & MX_DBG_SPEW_PROX_POLL) {
		dev_info(&st->i2c->dev,
			 "poll proximity %u %lld  diff=%d %lldns  hw=%hu\n",
			 proximity, ts,
			 proximity - st->proximity, ts - st->ts, st->prox);
		if (ret > 0)
			ret = 2; /* poll mode */
		else
			ret = -1; /* poll without report */
	}
	return ret;
}

static int mx_en(struct mx_state *st, unsigned int enable)
{
	int ret;

	mx_report_init(st);
	ret = mx_i2c_wr(st, MX_REG_CFG_THR, st->thr_cfg);
	if (enable & (1 << MX_DEV_PROX))
		ret |= mx_i2c_wr(st, MX_REG_CFG_PRX, st->prx_cfg);
	ret |= mx_cmd_wr(st, enable, false);
	if (st->dbg & MX_DBG_SPEW_MSG) {
		if (enable & (1 << MX_DEV_PROX))
			dev_info(&st->i2c->dev,
				 "%s thr_cfg=%hhx prx_cfg=%hhx err=%d\n",
				 __func__, st->thr_cfg, st->prx_cfg, ret);
		else
			dev_info(&st->i2c->dev, "%s thr_cfg=%hhx err=%d\n",
				 __func__, st->thr_cfg, ret);
	}
	return ret;
}

static int mx_rd(struct iio_dev *indio_dev)
{
	struct mx_state *st = iio_priv(indio_dev);
	u8 sts;
	s64 ts;
	bool report = false;
	unsigned int i;
	int ret;
	int ret_t = 0;

	/* clear possible IRQ */
	ret = mx_i2c_rd(st, MX_REG_STS, &sts);
	if (ret)
		return ret;

	if (sts & (1 << MX_REG_STS_PWRON)) {
		/* restart */
		mx_en(st, st->enable);
		return -1;
	}

	ts = iio_get_time_ns();
	if ((st->report < st->report_n) || !st->report) {
		/* calculate elapsed time for allowed report rate */
		if ((ts - st->ts) < st->poll_delay_ms * 1000000)
			/* data changes are happening faster than allowed to
			 * report so we poll for the next data at an allowed
			 * rate with interrupts disabled.
			 */
			ret = mx_cmd_wr(st, st->enable, false);
	}
	/* proximity device */
	if (st->enable & (1 << MX_DEV_PROX)) {
		ret = mx_rd_prox(st, ts);
		if (ret > 0)
			report = true;
		if ((ret < 0) || (ret > 1))
			ret_t |= ret;
	}
	/* temp device */
	if (st->enable & (1 << MX_DEV_TEMP)) {
		ret = mx_rd_temp(st, ts);
		if (ret > 0)
			report = true;
		if ((ret < 0) || (ret > 1))
			ret_t |= ret;
	}
	/* light device */
	if (st->enable & (1 << MX_DEV_LIGHT)) {
		ret = mx_rd_light(st, ts);
		if (ret > 0)
			report = true;
		if ((ret < 0) || (ret > 1))
			ret_t |= ret;
	}
	/* if IIO scale == 1 then in calibrate/test mode */
	for (i = 0; i < MX_DEV_N; i++) {
		if ((st->scale_val[i] == 1) && (st->scale_val2[i] == 0)) {
			report = true;
			break;
		}
	}
	/* report if:
	 * - st->report (usually used to report the first sample regardless)
	 * - time since last report >= polling delay && data outside thresholds
	 * - report regardless of change (for test and calibration)
	 */
	if (st->report || report) {
		if (st->report)
			st->report--;
		st->ts = ts;
		mx_buf_push(indio_dev);
	}
	if (ret_t || st->report)
		/* poll if error or forcing reports */
		ret = mx_cmd_wr(st, st->enable, false);
	else
		ret = mx_cmd_wr(st, st->enable, true);
	return ret;
}

static void mx_read(struct iio_dev *indio_dev)
{
	struct mx_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (st->enable) {
		ret = mx_rd(indio_dev);
		if (ret < 1)
			schedule_delayed_work(&st->dw,
					  msecs_to_jiffies(st->poll_delay_ms));
	}
	mutex_unlock(&indio_dev->mlock);
}

static void mx_work(struct work_struct *ws)
{
	struct mx_state *st = container_of((struct delayed_work *)ws,
					   struct mx_state, dw);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);

	mx_read(indio_dev);
}

static irqreturn_t mx_irq_thread(int irq, void *dev_id)
{
	struct mx_state *st = (struct mx_state *)dev_id;
	struct iio_dev *indio_dev = iio_priv_to_dev(st);

	if (st->dbg & MX_DBG_IRQ)
		dev_info(&st->i2c->dev, "%s\n", __func__);
	mx_read(indio_dev);
	return IRQ_HANDLED;
}

static int mx_disable(struct iio_dev *indio_dev)
{
	struct mx_state *st = iio_priv(indio_dev);
	int ret = 0;

	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  MX_SCAN_LIGHT)))
		st->dbg &= ~MX_DBG_VAL_LIGHT;
	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  MX_SCAN_TEMP)))
		st->dbg &= ~MX_DBG_VAL_TEMP;
	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  MX_SCAN_PROX)))
		st->dbg &= ~MX_DBG_VAL_PROX;
	cancel_delayed_work_sync(&st->dw);
	if (st->enable & (1 << MX_DEV_PROX))
		ret = mx_i2c_wr(st, MX_REG_CFG_PRX, 0);
	ret |= mx_pm(st, false);
	if (!ret)
		st->enable = 0;
	return ret;
}

static int mx_enable(struct iio_dev *indio_dev)
{
	struct mx_state *st = iio_priv(indio_dev);
	unsigned int enable = 0;
	int ret = -EINVAL;

	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, MX_SCAN_LIGHT))
		enable |= (1 << MX_DEV_LIGHT);
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, MX_SCAN_TEMP))
		enable |= (1 << MX_DEV_TEMP);
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, MX_SCAN_PROX))
		enable |= (1 << MX_DEV_PROX);
	if (enable) {
		ret = mx_pm(st, true);
		if (!ret) {
			mx_delay(st);
			ret = mx_en(st, enable);
			if (ret < 0) {
				mx_disable(indio_dev);
			} else {
				st->enable = enable;
				schedule_delayed_work(&st->dw,
					 msecs_to_jiffies(MX_POLL_DLY_MS_MIN));
			}
		}
	}
	return ret;
}

static ssize_t mx_attr_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct mx_state *st = iio_priv(indio_dev);
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
	case MX_ATTR_ENABLE:
		msg = "ATTR_ENABLE";
		if (st->enable)
			old = 1;
		if (new)
			ret = mx_enable(indio_dev);
		else
			ret = mx_disable(indio_dev);
		break;

	case MX_ATTR_LIGHT_THRESH_LO:
		msg = "ATTR_THRESH_LO";
		old = st->lux_thr_lo;
		st->lux_thr_lo = new;
		break;

	case MX_ATTR_LIGHT_THRESH_HI:
		msg = "ATTR_LIGHT_THRESH_HI";
		old = st->lux_thr_hi;
		st->lux_thr_hi = new;
		break;

	case MX_ATTR_PROX_THRESH_LO:
		msg = "ATTR_PROX_THRESH_LO";
		old = st->prx_thr_lo;
		st->prx_thr_lo = new;
		break;

	case MX_ATTR_PROX_THRESH_HI:
		msg = "ATTR_PROX_THRESH_HI";
		old = st->prx_thr_hi;
		st->prx_thr_hi = new;
		break;

	default:
		msg = "ATTR_UNKNOWN";
		ret = -EINVAL;
	}

	mx_report_init(st);
	mutex_unlock(&indio_dev->mlock);
	mx_read(indio_dev);
	if (st->dbg & MX_DBG_SPEW_MSG) {
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

static ssize_t mx_attr_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct mx_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	switch (this_attr->address) {
	case MX_ATTR_ENABLE:
		return sprintf(buf, "%x\n", st->enable);

	case MX_ATTR_VENDOR:
		return sprintf(buf, "%s\n", MX_VENDOR);

	case MX_ATTR_LIGHT_PART:
		return sprintf(buf, "%s light\n", st->part);

	case MX_ATTR_LIGHT_VERSION:
		return sprintf(buf, "%u\n", MX_LIGHT_VERSION);

	case MX_ATTR_LIGHT_MILLIAMP:
		return sprintf(buf, "%s\n", MX_LIGHT_MILLIAMP);

	case MX_ATTR_LIGHT_THRESH_LO:
		return sprintf(buf, "%u\n", st->lux_thr_lo);

	case MX_ATTR_LIGHT_THRESH_HI:
		return sprintf(buf, "%u\n", st->lux_thr_hi);

	case MX_ATTR_TEMP_PART:
		return sprintf(buf, "%s temperature\n", st->part);

	case MX_ATTR_TEMP_VERSION:
		return sprintf(buf, "%u\n", MX_TEMP_VERSION);

	case MX_ATTR_TEMP_MILLIAMP:
		return sprintf(buf, "%s\n", MX_TEMP_MILLIAMP);

	case MX_ATTR_PROX_PART:
		return sprintf(buf, "%s proximity\n", st->part);

	case MX_ATTR_PROX_VERSION:
		return sprintf(buf, "%u\n", MX_PROX_VERSION);

	case MX_ATTR_PROX_MILLIAMP:
		return sprintf(buf, "%s\n", MX_PROX_MILLIAMP);

	case MX_ATTR_PROX_THRESH_LO:
		return sprintf(buf, "%u\n", st->prx_thr_lo);

	case MX_ATTR_PROX_THRESH_HI:
		return sprintf(buf, "%u\n", st->prx_thr_hi);

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static ssize_t mx_data_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct mx_state *st = iio_priv(dev_get_drvdata(dev));
	unsigned int info;
	int ret;

	ret = kstrtouint(buf, 10, &info);
	if (ret)
		return -EINVAL;

	if (info >= MX_INFO_LIMIT_MAX)
		return -EINVAL;

	st->info = info;
	switch (info) {
	case MX_INFO_DATA:
		st->dbg = 0;
		break;

	case MX_INFO_DBG:
		st->dbg ^= MX_DBG_SPEW_MSG;
		break;

	case MX_INFO_LIGHT_SPEW:
		st->dbg ^= MX_DBG_SPEW_LIGHT;
		break;

	case MX_INFO_LIGHT_POLL_SPEW:
		st->dbg ^= MX_DBG_SPEW_LIGHT_POLL;
		break;

	case MX_INFO_TEMP_SPEW:
		st->dbg ^= MX_DBG_SPEW_TEMP;
		break;

	case MX_INFO_TEMP_POLL_SPEW:
		st->dbg ^= MX_DBG_SPEW_TEMP_POLL;
		break;

	case MX_INFO_PROXIMITY_SPEW:
		st->dbg ^= MX_DBG_SPEW_PROX;
		break;

	case MX_INFO_PROXIMITY_POLL_SPEW:
		st->dbg ^= MX_DBG_SPEW_PROX_POLL;
		break;

	case MX_INFO_DBG_IRQ:
		st->dbg ^= MX_DBG_IRQ;
		break;

	default:
		break;
	}

	return count;
}

static ssize_t mx_data_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct mx_state *st = iio_priv(dev_get_drvdata(dev));
	enum MX_INFO info;
	ssize_t t;
	u8 val[2];
	u8 i;
	int ret;

	info = st->info;
	st->info = MX_INFO_DATA;
	switch (info) {
	case MX_INFO_DATA:
		t = sprintf(buf, "light: %u\n", st->light);
		t += sprintf(buf + t, "temp: %d\n", st->temp);
		if (st->dev_id == MX_DEVID_MAX44005)
			t += sprintf(buf + t, "proximity: %u\n",
				     st->proximity);
		t += sprintf(buf + t, "timestamp: %lld\n", st->ts);
		return t;

	case MX_INFO_VER:
		return sprintf(buf, "version=%u\n", MX_VERSION_DRIVER);

	case MX_INFO_ERRS:
		t = sprintf(buf, "error count=%u\n", st->errs);
		st->errs = 0;
		return t;

	case MX_INFO_REGS:
		t = sprintf(buf, "registers:\n");
		for (i = 0; i <= MX_REG_CFG_PRX; i++) {
			ret = mx_i2c_rd(st, i, val);
			if (!ret)
				t += sprintf(buf + t, "0x%hhx=0x%hhx\n",
					     i, val[0]);
		}
		for (i = MX_REG_DATA_AMB_CLEAR; i < MX_REG_CFG_THR; i += 2) {
			ret = mx_i2c_read(st, i, 2, val);
			if (!ret)
				t += sprintf(buf + t, "0x%hhx:0x%hhx=0x%hx\n",
					i, i + 1, be16_to_cpup((__be16 *)val));
		}
		ret = mx_i2c_rd(st, MX_REG_CFG_THR, val);
		if (!ret)
			t += sprintf(buf + t, "0x%hhx=0x%hhx\n",
				     MX_REG_CFG_THR, val[0]);
		for (i = MX_REG_PRX_UPTHR_H; i < MX_REG_TRIM_CLEAR; i += 2) {
			ret = mx_i2c_read(st, i, 2, val);
			if (!ret)
				t += sprintf(buf + t, "0x%hhx:0x%hhx=0x%hx\n",
					i, i + 1, be16_to_cpup((__be16 *)val));
		}
		for (i = MX_REG_TRIM_CLEAR; i <= MX_REG_TRIM_IR; i++) {
			ret = mx_i2c_rd(st, i, val);
			if (!ret)
				t += sprintf(buf + t, "0x%hhx=0x%hhx\n",
					     i, val[0]);
		}
		return t;

	case MX_INFO_DBG:
		return sprintf(buf, "debug spew=%x\n",
			       st->dbg & MX_DBG_SPEW_MSG);

	case MX_INFO_LIGHT_SPEW:
		return sprintf(buf, "light_ts spew=%x\n",
			       !!(st->dbg & MX_DBG_SPEW_LIGHT));

	case MX_INFO_LIGHT_POLL_SPEW:
		return sprintf(buf, "light_poll_ts spew=%x\n",
			       !!(st->dbg & MX_DBG_SPEW_LIGHT_POLL));

	case MX_INFO_TEMP_SPEW:
		return sprintf(buf, "temp_ts spew=%x\n",
			       !!(st->dbg & MX_DBG_SPEW_TEMP));

	case MX_INFO_TEMP_POLL_SPEW:
		return sprintf(buf, "temp_poll_ts spew=%x\n",
			       !!(st->dbg & MX_DBG_SPEW_TEMP_POLL));

	case MX_INFO_PROXIMITY_SPEW:
		return sprintf(buf, "proximity_ts spew=%x\n",
			       !!(st->dbg & MX_DBG_SPEW_PROX));

	case MX_INFO_PROXIMITY_POLL_SPEW:
		return sprintf(buf, "proximity_poll_ts spew=%x\n",
			       !!(st->dbg & MX_DBG_SPEW_PROX_POLL));

	case MX_INFO_DBG_IRQ:
		return sprintf(buf, "debug IRQ=%x\n",
			       !!(st->dbg & MX_DBG_IRQ));

	default:
		break;
	}

	return -EINVAL;
}

static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		       mx_attr_show, mx_attr_store, MX_ATTR_ENABLE);
static IIO_DEVICE_ATTR(illuminance_part, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_LIGHT_PART);
static IIO_DEVICE_ATTR(illuminance_vendor, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_VENDOR);
static IIO_DEVICE_ATTR(illuminance_version, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_LIGHT_VERSION);
static IIO_DEVICE_ATTR(illuminance_milliamp, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_LIGHT_MILLIAMP);
static IIO_DEVICE_ATTR(illuminance_thresh_rising_value,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       mx_attr_show, mx_attr_store, MX_ATTR_LIGHT_THRESH_HI);
static IIO_DEVICE_ATTR(illuminance_thresh_falling_value,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       mx_attr_show, mx_attr_store, MX_ATTR_LIGHT_THRESH_LO);
static IIO_DEVICE_ATTR(temp_part, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_TEMP_PART);
static IIO_DEVICE_ATTR(temp_vendor, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_VENDOR);
static IIO_DEVICE_ATTR(temp_version, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_TEMP_VERSION);
static IIO_DEVICE_ATTR(temp_milliamp, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_TEMP_MILLIAMP);
static IIO_DEVICE_ATTR(proximity_part, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_PROX_PART);
static IIO_DEVICE_ATTR(proximity_vendor, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_VENDOR);
static IIO_DEVICE_ATTR(proximity_version, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_PROX_VERSION);
static IIO_DEVICE_ATTR(proximity_milliamp, S_IRUGO,
		       mx_attr_show, NULL, MX_ATTR_PROX_MILLIAMP);
static IIO_DEVICE_ATTR(proximity_thresh_rising_value,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       mx_attr_show, mx_attr_store, MX_ATTR_PROX_THRESH_HI);
static IIO_DEVICE_ATTR(proximity_thresh_falling_value,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       mx_attr_show, mx_attr_store, MX_ATTR_PROX_THRESH_LO);
static DEVICE_ATTR(data, S_IRUGO | S_IWUSR | S_IWGRP,
		   mx_data_show, mx_data_store);

static struct attribute *mx_attrs[] = {
	&dev_attr_data.attr,
	&iio_dev_attr_enable.dev_attr.attr,
	&iio_dev_attr_illuminance_part.dev_attr.attr,
	&iio_dev_attr_illuminance_vendor.dev_attr.attr,
	&iio_dev_attr_illuminance_version.dev_attr.attr,
	&iio_dev_attr_illuminance_milliamp.dev_attr.attr,
	&iio_dev_attr_illuminance_thresh_rising_value.dev_attr.attr,
	&iio_dev_attr_illuminance_thresh_falling_value.dev_attr.attr,
	&iio_dev_attr_temp_part.dev_attr.attr,
	&iio_dev_attr_temp_vendor.dev_attr.attr,
	&iio_dev_attr_temp_version.dev_attr.attr,
	&iio_dev_attr_temp_milliamp.dev_attr.attr,
	/* proximity_part must be the first proximity attribute */
	&iio_dev_attr_proximity_part.dev_attr.attr,
	&iio_dev_attr_proximity_vendor.dev_attr.attr,
	&iio_dev_attr_proximity_version.dev_attr.attr,
	&iio_dev_attr_proximity_milliamp.dev_attr.attr,
	&iio_dev_attr_proximity_thresh_rising_value.dev_attr.attr,
	&iio_dev_attr_proximity_thresh_falling_value.dev_attr.attr,
	NULL
};

static struct attribute_group mx_attr_group = {
	.name = MX_NAME,
	.attrs = mx_attrs
};

static int mx_read_raw(struct iio_dev *indio_dev,
		       struct iio_chan_spec const *chan,
		       int *val, int *val2, long mask)
{
	struct mx_state *st = iio_priv(indio_dev);
	int i = chan->scan_index;
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			*val = st->light;
			return IIO_VAL_INT;

		case IIO_TEMP:
			*val = st->temp;
			return IIO_VAL_INT;

		case IIO_PROXIMITY:
			*val = st->proximity;
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->enable & (1 << i))
			*val = st->delay_us[i];
		else
			*val = MX_POLL_DLY_MS_MIN * 1000;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->scale_val[i];
		*val2 = st->scale_val2[i];
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		*val = st->offset_val[i];
		*val2 = st->offset_val2[i];
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_PEAK:
		*val = st->peak_val[i];
		*val2 = st->peak_val2[i];
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_PEAK_SCALE:
		*val = st->peak_scale_val[i];
		*val2 = st->peak_scale_val2[i];
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return ret;
}

static int mx_write_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int val, int val2, long mask)
{
	struct mx_state *st = iio_priv(indio_dev);
	char *msg;
	int i = chan->scan_index;
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
		old = st->delay_us[i];
		st->delay_us[i] = val;
		mx_delay(st);
		break;

	case IIO_CHAN_INFO_SCALE:
		msg = "IIO_CHAN_INFO_SCALE";
		old = st->scale_val[i];
		old2 = st->scale_val2[i];
		st->scale_val[i] = val;
		st->scale_val2[i] = val2;
		break;

	case IIO_CHAN_INFO_OFFSET:
		msg = "IIO_CHAN_INFO_SCALE";
		old = st->offset_val[i];
		old2 = st->offset_val2[i];
		st->offset_val[i] = val;
		st->offset_val2[i] = val2;
		break;

	case IIO_CHAN_INFO_PEAK:
		msg = "IIO_CHAN_INFO_PEAK";
		old = st->peak_val[i];
		old2 = st->peak_val2[i];
		st->peak_val[i] = val;
		st->peak_val2[i] = val2;
		break;

	case IIO_CHAN_INFO_PEAK_SCALE:
		msg = "IIO_CHAN_INFO_PEAK_SCALE";
		old = st->peak_scale_val[i];
		old2 = st->peak_scale_val2[i];
		st->peak_scale_val[i] = val;
		st->peak_scale_val2[i] = val2;
		break;

	case IIO_CHAN_INFO_RAW:
		msg = "IIO_CHAN_INFO_RAW";
		switch (chan->type) {
		case IIO_LIGHT:
			old = st->light;
			st->light = val;
			st->dbg |= MX_DBG_VAL_LIGHT;
			break;

		case IIO_TEMP:
			old = st->temp;
			st->temp = val;
			st->dbg |= MX_DBG_VAL_TEMP;
			break;

		case IIO_PROXIMITY:
			old = st->proximity;
			st->proximity = val;
			st->dbg |= MX_DBG_VAL_PROX;
			break;

		default:
			ret = -EINVAL;
		}

		break;

	default:
		msg = "IIO_CHAN_INFO_UNKNOWN";
		ret = -EINVAL;
	}

	mx_report_init(st);
	mutex_unlock(&indio_dev->mlock);
	mx_read(indio_dev);
	if (st->dbg & MX_DBG_SPEW_MSG) {
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

static const struct iio_info mx_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &mx_attr_group,
	.read_raw = &mx_read_raw,
	.write_raw = &mx_write_raw,
};

static struct iio_chan_spec mx_channels[] = {
	{
		.type			= IIO_LIGHT,
		.scan_index		= MX_SCAN_LIGHT,
		.scan_type		= IIO_ST('u', 32, 32, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
	},
	{
		.type			= IIO_TEMP,
		.scan_index		= MX_SCAN_TEMP,
		.scan_type		= IIO_ST('s', 14, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type			= IIO_PROXIMITY,
		.scan_index		= MX_SCAN_PROX,
		.scan_type		= IIO_ST('u', 32, 32, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
	},
	IIO_CHAN_SOFT_TIMESTAMP(MX_SCAN_TIMESTAMP)
};

static int mx_buffer_preenable(struct iio_dev *indio_dev)
{
	struct mx_state *st = iio_priv(indio_dev);

	if (st->shutdown || st->suspend)
		return -EINVAL;

	return 0;
}

static int mx_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;

	ret = mx_enable(indio_dev);
	/* never return > 0 to IIO buffer engine */
	if (ret > 0)
		ret = 0;
	return ret;
}

static const struct iio_buffer_setup_ops mx_buffer_setup_ops = {
	/* iio_sw_buffer_preenable:
	 * Generic function for equal sized ring elements + 64 bit timestamp
	 * Assumes that any combination of channels can be enabled.
	 * Typically replaced to implement restrictions on what combinations
	 * can be captured (hardware scan modes).
	 */
	.preenable = &mx_buffer_preenable,
	/* iio_triggered_buffer_postenable:
	 * Generic function that simply attaches the pollfunc to the trigger.
	 * Replace this to mess with hardware state before we attach the
	 * trigger.
	 */
	.postenable = &mx_buffer_postenable,
	/* this driver relies on the NVS HAL to power off this device with the
	 * master enable.
	 *.predisable = N/A
	 *.postdisable = N/A
	 */
};

static const struct iio_trigger_ops mx_trigger_ops = {
	.owner = THIS_MODULE,
};

static int mx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct mx_state *st = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (st->enable && !(iio_scan_mask_query(indio_dev, indio_dev->buffer,
						MX_SCAN_PROX))) {
		ret = mx_disable(indio_dev);
		st->suspend = true;
	}
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & MX_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int mx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct mx_state *st = iio_priv(indio_dev);

	st->suspend = false;
	if (st->dbg & MX_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(mx_pm_ops, mx_suspend, mx_resume);

static int mx_id_dev(struct iio_dev *indio_dev, const char *name)
{
	struct mx_state *st = iio_priv(indio_dev);
	u8 val = 0;
	int ret = 0;

	if (!strcmp(name, MX_NAME_MAX44005)) {
		st->dev_id = MX_DEVID_MAX44005;
		st->part = MX_NAME_MAX44005;
	} else if (!strcmp(name, MX_NAME_MAX44006)) {
		st->dev_id = MX_DEVID_MAX44006;
		st->part = MX_NAME_MAX44006;
	} else if (!strcmp(name, MX_NAME_MAX44008)) {
		st->dev_id = MX_DEVID_MAX44008;
		st->part = MX_NAME_MAX44008;
	}
	if (!st->dev_id) {
		ret = mx_i2c_rd(st, MX_REG_STS, &val);
		if (ret)
			return -ENODEV;

		if (val != (1 << MX_REG_STS_PWRON))
			return -ENODEV;

		ret = mx_i2c_rd(st, MX_REG_CFG_PRX, &val);
		if (!ret) {
			if (val == (1 << MX_REG_CFG_PRX_PRXTIM)) {
				st->dev_id = MX_DEVID_MAX44005;
				st->part = MX_NAME_MAX44005;
			} else {
				st->dev_id = MX_DEVID_MAX44006;
				st->part = MX_NAME_MAX44006;
			}
			dev_info(&st->i2c->dev, "%s found %s\n",
				__func__, st->part);
		}
	}
	return ret;
}

static int mx_id_i2c(struct iio_dev *indio_dev, const char *name)
{
	struct mx_state *st = iio_priv(indio_dev);
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mx_i2c_addrs); i++) {
		if (st->i2c->addr == mx_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(mx_i2c_addrs)) {
		st->i2c_addr = st->i2c->addr;
		ret = mx_id_dev(indio_dev, name);
	} else {
		name = MX_NAME;
		for (i = 0; i < ARRAY_SIZE(mx_i2c_addrs); i++) {
			st->i2c_addr = mx_i2c_addrs[i];
			ret = mx_id_dev(indio_dev, name);
			if (!ret)
				break;
		}
	}
	return ret;
}

static void mx_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct mx_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (st->enable)
		mx_disable(indio_dev);
	st->shutdown = true;
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & MX_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int mx_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct mx_state *st = iio_priv(indio_dev);

	if (st != NULL) {
		mx_shutdown(client);
		if (indio_dev->dev.devt)
			iio_device_unregister(indio_dev);
		if (st->trig != NULL) {
			if (client->irq)
				free_irq(client->irq, st);
			iio_trigger_unregister(st->trig);
			iio_trigger_free(st->trig);
		}
		if (indio_dev->buffer != NULL) {
			iio_buffer_unregister(indio_dev);
			iio_kfifo_free(indio_dev->buffer);
		}
		if (st->dw.wq)
			destroy_workqueue(st->dw.wq);
		mx_pm_exit(st);
		iio_device_free(indio_dev);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static const struct mx_iio_float mx_scale_dflt[] = {
	{
		.ival			= MX_LIGHT_SCALE_IVAL,
		.micro			= MX_LIGHT_SCALE_MICRO,
	},
	{
		.ival			= MX_TEMP_SCALE_IVAL,
		.micro			= MX_TEMP_SCALE_MICRO,
	},
	{
		.ival			= MX_PROX_SCALE_IVAL,
		.micro			= MX_PROX_SCALE_MICRO,
	},
};

static const struct mx_iio_float mx_offset_dflt[] = {
	{
		.ival			= MX_LIGHT_OFFSET_IVAL,
		.micro			= MX_LIGHT_OFFSET_MICRO,
	},
	{
		.ival			= MX_TEMP_OFFSET_IVAL,
		.micro			= MX_TEMP_OFFSET_MICRO,
	},
	{
		.ival			= MX_PROX_OFFSET_IVAL,
		.micro			= MX_PROX_OFFSET_MICRO,
	},
};

static const struct mx_iio_float mx_peak_dflt[] = {
	{
		.ival			= MX_LIGHT_MAX_RANGE_IVAL,
		.micro			= MX_LIGHT_MAX_RANGE_MICRO,
	},
	{
		.ival			= MX_TEMP_MAX_RANGE_IVAL,
		.micro			= MX_TEMP_MAX_RANGE_MICRO,
	},
	{
		.ival			= MX_PROX_MAX_RANGE_IVAL,
		.micro			= MX_PROX_MAX_RANGE_MICRO,
	},
};

static const struct mx_iio_float mx_peak_scale_dflt[] = {
	{
		.ival			= MX_LIGHT_RESOLUTION_IVAL,
		.micro			= MX_LIGHT_RESOLUTION_MICRO,
	},
	{
		.ival			= MX_TEMP_RESOLUTION_IVAL,
		.micro			= MX_TEMP_RESOLUTION_MICRO,
	},
	{
		.ival			= MX_PROX_RESOLUTION_IVAL,
		.micro			= MX_PROX_RESOLUTION_MICRO,
	},
};

static int mx_parse_dt(struct i2c_client *client, struct mx_state *st)
{
	struct device_node *np = client->dev.of_node;
	unsigned int i;

	/* default device specific parameters */
	st->amb_cfg = MX_AMB_CFG_DFLT;
	st->prx_cfg = MX_PRX_CFG_DFLT;
	/* default NVS ALS programmable parameters */
	for (i = 0; i < MX_DEV_N; i++) {
		st->scale_val[i] = mx_scale_dflt[i].ival;
		st->scale_val2[i] = mx_scale_dflt[i].micro;
		st->offset_val[i] = mx_offset_dflt[i].ival;
		st->offset_val2[i] = mx_offset_dflt[i].micro;
		st->peak_val[i] = mx_peak_dflt[i].ival;
		st->peak_val2[i] = mx_peak_dflt[i].micro;
		st->peak_scale_val[i] = mx_peak_scale_dflt[i].ival;
		st->peak_scale_val2[i] = mx_peak_scale_dflt[i].micro;
	}

	if (st->scale_val2[MX_DEV_LIGHT]) {
		st->lux_thr_lo = 1000000 / st->scale_val2[MX_DEV_LIGHT];
		st->lux_thr_hi = 1000000 / st->scale_val2[MX_DEV_LIGHT];
	}
	st->lux_thr_lo *= MX_THRESHOLD_LUX_DFLT;
	st->lux_thr_hi *= MX_THRESHOLD_LUX_DFLT;
	/* proximity has binary threshold */
	st->prx_thr_bin = 1;
	st->prx_thr_lo = MX_THRESHOLD_PROX_DFLT;
	st->prx_thr_hi = MX_THRESHOLD_PROX_DFLT;
	/* device tree parameters */
	if (client->dev.of_node) {
		/* device specific parameters */
		of_property_read_u8(np, "ambient_cfg_reg", &st->amb_cfg);
		of_property_read_u8(np, "proximity_cfg_reg", &st->prx_cfg);
		of_property_read_u8(np, "threshold_persist_reg", &st->thr_cfg);
		/* common NVS programmable parameters */
		of_property_read_u32(np, "report_count", &st->report_n);
		/* common NVS ALS programmable parameters */
		of_property_read_s32(np, "light_uncalibrated_lo",
				     &st->i_uc_lo[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_uncalibrated_hi",
				     &st->i_uc_hi[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_calibrated_lo",
				     &st->i_c_lo[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_calibrated_hi",
				     &st->i_c_hi[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_max_range_val",
				     &st->peak_val[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_max_range_val2",
				     &st->peak_val2[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_resolution_val",
				     &st->peak_scale_val[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_resolution_val2",
				     &st->peak_scale_val2[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_scale_val",
				     &st->scale_val[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_scale_val2",
				     &st->scale_val2[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_offset_val",
				     &st->offset_val[MX_DEV_LIGHT]);
		of_property_read_s32(np, "light_offset_val2",
				     &st->offset_val2[MX_DEV_LIGHT]);
		of_property_read_u32(np, "light_threshold_lo",
				     &st->lux_thr_lo);
		of_property_read_u32(np, "light_threshold_hi",
				     &st->lux_thr_hi);
		/* common NVS temp programmable parameters */
		of_property_read_s32(np, "temp_max_range_val",
				     &st->peak_val[MX_DEV_TEMP]);
		of_property_read_s32(np, "temp_max_range_val2",
				     &st->peak_val2[MX_DEV_TEMP]);
		of_property_read_s32(np, "temp_resolution_val",
				     &st->peak_scale_val[MX_DEV_TEMP]);
		of_property_read_s32(np, "temp_resolution_val2",
				     &st->peak_scale_val2[MX_DEV_TEMP]);
		of_property_read_s32(np, "temp_scale_val",
				     &st->scale_val[MX_DEV_TEMP]);
		of_property_read_s32(np, "temp_scale_val2",
				     &st->scale_val2[MX_DEV_TEMP]);
		of_property_read_s32(np, "temp_offset_val",
				     &st->offset_val[MX_DEV_TEMP]);
		of_property_read_s32(np, "temp_offset_val2",
				     &st->offset_val2[MX_DEV_TEMP]);
		/* common NVS proximity programmable parameters */
		of_property_read_s32(np, "proximity_uncalibrated_lo",
				     &st->i_uc_lo[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_uncalibrated_hi",
				     &st->i_uc_hi[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_calibrated_lo",
				     &st->i_c_lo[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_calibrated_hi",
				     &st->i_c_hi[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_max_range_val",
				     &st->peak_val[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_max_range_val2",
				     &st->peak_val2[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_resolution_val",
				     &st->peak_scale_val[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_resolution_val2",
				     &st->peak_scale_val2[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_scale_val",
				     &st->scale_val[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_scale_val2",
				     &st->scale_val2[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_offset_val",
				     &st->offset_val[MX_DEV_PROX]);
		of_property_read_s32(np, "proximity_offset_val2",
				     &st->offset_val2[MX_DEV_PROX]);
		of_property_read_u32(np, "proximity_threshold_lo",
				     &st->prx_thr_lo);
		of_property_read_u32(np, "proximity_threshold_hi",
				     &st->prx_thr_hi);
		of_property_read_u32(np, "proximity_binary_threshold",
				     &st->prx_thr_bin);
		if (st->prx_thr_bin) {
			/* proximity has binary threshold */
			st->peak_val[MX_DEV_PROX] = 1;
			st->peak_val2[MX_DEV_PROX] = 0;
			st->peak_scale_val[MX_DEV_PROX] = 1;
			st->peak_scale_val2[MX_DEV_PROX] = 0;
			st->scale_val[MX_DEV_PROX] = 0;
			st->scale_val2[MX_DEV_PROX] = 0;
			st->offset_val[MX_DEV_PROX] = 0;
			st->offset_val2[MX_DEV_PROX] = 0;
		}
	}
	return 0;
}

static int mx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct mx_state *st;
	unsigned int i;
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
	ret = mx_parse_dt(client, st);
	if (ret) {
		ret = -ENODEV;
		goto mx_probe_err;
	}

	mx_pm_init(st);
	ret = mx_id_i2c(indio_dev, id->name);
	if (ret) {
		ret = -ENODEV;
		goto mx_probe_exit;
	}

	mx_pm(st, false);
	indio_dev->buffer = iio_kfifo_allocate(indio_dev);
	if (!indio_dev->buffer) {
		dev_err(&client->dev, "%s iio_kfifo_allocate ERR\n", __func__);
		ret = -ENOMEM;
		goto mx_probe_err;
	}

	indio_dev->buffer->scan_timestamp = true;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = mx_channels;
	if (st->dev_id != MX_DEVID_MAX44005) {
		/* remove proximity IIO channel */
		indio_dev->num_channels = (ARRAY_SIZE(mx_channels) - 1);
		memcpy(&mx_channels[MX_SCAN_PROX],
		       &mx_channels[MX_SCAN_TIMESTAMP],
		       sizeof(struct iio_chan_spec));
		/* remove proximity attributes */
		for (i = 0; i < ARRAY_SIZE(mx_attrs); i++) {
			if (mx_attrs[i] == &iio_dev_attr_proximity_part.
					   dev_attr.attr) {
				mx_attrs[i] = NULL;
				break;
			}
		}
	} else {
		indio_dev->num_channels = ARRAY_SIZE(mx_channels);
	}
	indio_dev->name = MX_NAME;
	indio_dev->info = &mx_iio_info;
	indio_dev->setup_ops = &mx_buffer_setup_ops;
	ret = iio_buffer_register(indio_dev, indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto mx_probe_err;

	INIT_DELAYED_WORK(&st->dw, mx_work);
	if (client->irq) {
		unsigned long irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		if (st->dev_id == MX_DEVID_MAX44005)
			irqflags |= IRQF_NO_SUSPEND; /* for proximity */
		ret = request_threaded_irq(client->irq, NULL, mx_irq_thread,
					   irqflags, MX_NAME, st);
		if (ret) {
			dev_err(&client->dev, "%s req_threaded_irq ERR %d\n",
				__func__, ret);
			ret = -ENOMEM;
			goto mx_probe_err;
		}
	}

	st->trig = iio_trigger_alloc("%s-dev%d",
				     indio_dev->name, indio_dev->id);
	if (st->trig == NULL) {
		dev_err(&client->dev, "%s iio_allocate_trigger ERR\n",
			__func__);
		ret = -ENOMEM;
		goto mx_probe_err;
	}

	st->trig->dev.parent = &st->i2c->dev;
	st->trig->ops = &mx_trigger_ops;
	ret = iio_trigger_register(st->trig);
	if (ret) {
		dev_err(&client->dev, "%s iio_trigger_register ERR\n",
			__func__);
		ret = -ENOMEM;
		goto mx_probe_err;
	}

	indio_dev->trig = st->trig;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_device_register(indio_dev);
	if (ret)
		goto mx_probe_err;

	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

mx_probe_err:
	dev_err(&client->dev, "%s ERR %d\n", __func__, ret);
mx_probe_exit:
	mx_remove(client);
	return ret;
}

static const struct i2c_device_id mx_i2c_device_id[] = {
	{ MX_NAME, 0 },
	{ MX_NAME_MAX44005, 0 },
	{ MX_NAME_MAX44006, 0 },
	{ MX_NAME_MAX44008, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, mx_i2c_device_id);

static const struct of_device_id mx_of_match[] = {
	{ .compatible = "maxim,max4400x", },
	{ .compatible = "maxim,max44005", },
	{ .compatible = "maxim,max44006", },
	{ .compatible = "maxim,max44008", },
	{},
};

MODULE_DEVICE_TABLE(of, mx_of_match);

static struct i2c_driver mx_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= mx_probe,
	.remove		= mx_remove,
	.shutdown	= mx_shutdown,
	.driver = {
		.name		= MX_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(mx_of_match),
		.pm		= &mx_pm_ops,
	},
	.id_table	= mx_i2c_device_id,
};
module_i2c_driver(mx_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAX4400x driver");
MODULE_AUTHOR("NVIDIA Corporation");
