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

/* Device mapping is done via three parameters:
 * 1. If BMP_NVI_MPU_SUPPORT (defined below) is set, the code is included to
 *    support the device behind an Invensense MPU running an NVI (NVidia/
 *    Invensense) driver.
 *    If BMP_NVI_MPU_SUPPORT is 0 then this driver is only for the device in a
 *    stand-alone configuration without any dependancies on an Invensense MPU.
 * 2. Device tree platform configuration nvi_config:
 *    - auto = automatically detect if connected to host or MPU
 *    - mpu = connected to MPU
 *    - host = connected to host
 *    This is only available if BMP_NVI_MPU_SUPPORT is set.
 * 3. device in board file:
 *    - bmpX80 = automatically detect the device
 *    - force the device for:
 *      - bmp180
 *      - bmp280
 * If you have no clue what the device is and don't know how it is
 * connected then use auto and bmpX80.  The auto-detect mechanisms are for
 * platforms that have multiple possible configurations but takes longer to
 * initialize.  No device identification and connect testing is done for
 * specific configurations.
 */
/* A defined interrupt can be used as a SW flag to configure the device if
 * behind the MPU.  When an interrupt is defined in struct i2c_client.irq,
 * the driver is configured to only use the device's continuous mode if the
 * device supports it.  The interrupt itself is never touched so any value can
 * be used.  This frees the MPU auxiliary port used for writes.  This
 * configuration would be used if another MPU auxiliary port was needed for
 * another device connected to the MPU.  The delay timing used in continuous
 * mode is equal to or the next fastest supported speed.
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
 * The NVS sysfs attribute for the master enable is "enable".
 */
/* The NVS (NVidia Sensor) implementation of batch and flush are as follows:
 * If batch/flush is supported in a device, the kernel driver must have the
 * following sysfs attributes:
 * - <iio_device>_batch_flags
 *       read: returns the last _batch_flags written.
 *       write: sets the batch_flags parameter.
 * - <iio_device>_batch_period
 *       read: returns the last _batch_period written.
 *       write: sets the batch_period parameter.
 * - <iio_device>_batch_timeout
 *       read: returns the last _batch_timeout written.
 *       write: sets the batch_timeout and initiates the batch command using the
 *          current batch_flags and batch_period parameters.
 * - <iio_device>_flush
 *       read: returns the batch_flags that are supported.  If batch/flush is
 *             supported, then bit 0 must be 1.
 *       write: initiates a flush.
 * As part of the NVS initialization, NVS will attempt to read the _flush
 * attribute.  This will provide the supported flags.  Bit 0 of batch_flags
 * must be 1 if batch/flush is supported.  For efficiency, NVS will then use the
 * read result of this to determine if a batch command is supported before
 * attempting the batch command.
 * When the batch is done, the following sysfs attribute write order is required
 * by the NVS kernel driver:
 * 1. _batch_flags
 * 2. _batch_period
 * 3. _batch_timeout
 * The write to the _batch_timeout sysfs attribute is what initiates a batch
 * command and uses the last _batch_flags and _batch_period parameters written.
 * When a flush is initiated, NVS writes to the sysfs <iio_device>_flush
 * attribute.  When the flush is completed, the kernel driver will send the
 * data timestamp = 0.
 */

#define BMP_NVI_MPU_SUPPORT		(1) /* includes NVI MPU code */

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
#if BMP_NVI_MPU_SUPPORT
#include <linux/mpu_iio.h>
#endif /* BMP_NVI_MPU_SUPPORT */

#define BMP_VERSION_DRIVER		(102)
#define BMP_VENDOR			"Bosch"
#define BMP_NAME			"bmpX80"
#define BMP180_NAME			"bmp180"
#define BMP280_NAME			"bmp280"
#define BMP_PRES_MAX_RANGE_IVAL		(1100)
#define BMP_PRES_MAX_RANGE_MICRO	(0)
#define BMP_TEMP_MAX_RANGE_IVAL		(125)
#define BMP_TEMP_MAX_RANGE_MICRO	(0)
#define BMP_PRES_OFFSET_IVAL		(0)
#define BMP_PRES_OFFSET_MICRO		(0)
#define BMP_TEMP_OFFSET_IVAL		(0)
#define BMP_TEMP_OFFSET_MICRO		(0)
#define BMP_MAX_RANGE_MICRO
#define BMP180_RANGE_DFLT		(0)
/* until BMP280 OSS pressure calculation is supported, this defaults to 5 */
#define BMP280_RANGE_DFLT		(5)
#define BMP_HW_DELAY_POR_MS		(10)
#define BMP_POLL_DELAY_MS_DFLT		(200)
#define BMP_MPU_RETRY_COUNT		(50)
#define BMP_MPU_RETRY_DELAY_MS		(20)
/* HW registers */
#define BMP_REG_ID			(0xD0)
#define BMP_REG_ID_BMP180		(0x55)
#define BMP_REG_ID_BMP280		(0x56)
#define BMP_REG_RESET			(0xE0)
#define BMP_REG_RESET_VAL		(0xB6)
#define BMP_REG_CTRL			(0xF4)
#define BMP_REG_CTRL_MODE_SLEEP		(0)
#define BMP180_REG_CTRL_MODE_MASK	(0x1F)
#define BMP180_REG_CTRL_MODE_PRES	(0x34)
#define BMP180_REG_CTRL_MODE_TEMP	(0x2E)
#define BMP180_REG_CTRL_SCO		(5)
#define BMP180_REG_CTRL_OSS		(6)
#define BMP280_REG_CTRL_MODE_MASK	(0x03)
#define BMP280_REG_CTRL_MODE_FORCED1	(1)
#define BMP280_REG_CTRL_MODE_FORCED2	(2)
#define BMP280_REG_CTRL_MODE_NORMAL	(3)
#define BMP280_REG_CTRL_OSRS_P		(2)
#define BMP280_REG_CTRL_OSRS_P_MASK	(0x1C)
#define BMP280_REG_CTRL_OSRS_T		(5)
#define BMP280_REG_CTRL_OSRS_T_MASK	(0xE0)
#define BMP180_REG_OUT_MSB		(0xF6)
#define BMP180_REG_OUT_LSB		(0xF7)
#define BMP180_REG_OUT_XLSB		(0xF8)
#define BMP280_REG_STATUS		(0xF3)
#define BMP280_REG_STATUS_MEASURING	(3)
#define BMP280_REG_STATUS_IM_UPDATE	(0)
#define BMP280_REG_CONFIG		(0xF5)
#define BMP280_REG_CONFIG_T_SB		(5)
#define BMP280_REG_PRESS_MSB		(0xF7)
#define BMP280_REG_PRESS_LSB		(0xF8)
#define BMP280_REG_PRESS_XLSB		(0xF9)
#define BMP280_REG_TEMP_MSB		(0xFA)
#define BMP280_REG_TEMP_LSB		(0xFB)
#define BMP280_REG_TEMP_XLSB		(0xFC)
/* ROM registers */
#define BMP180_REG_AC1			(0xAA)
#define BMP180_REG_AC2			(0xAC)
#define BMP180_REG_AC3			(0xAE)
#define BMP180_REG_AC4			(0xB0)
#define BMP180_REG_AC5			(0xB2)
#define BMP180_REG_AC6			(0xB4)
#define BMP180_REG_B1			(0xB6)
#define BMP180_REG_B2			(0xB8)
#define BMP180_REG_MB			(0xBA)
#define BMP180_REG_MC			(0xBC)
#define BMP180_REG_MD			(0xBE)
#define BMP280_REG_CWORD00		(0x88)
#define BMP280_REG_CWORD01		(0x8A)
#define BMP280_REG_CWORD02		(0x8C)
#define BMP280_REG_CWORD03		(0x8E)
#define BMP280_REG_CWORD04		(0x90)
#define BMP280_REG_CWORD05		(0x92)
#define BMP280_REG_CWORD06		(0x94)
#define BMP280_REG_CWORD07		(0x96)
#define BMP280_REG_CWORD08		(0x98)
#define BMP280_REG_CWORD09		(0x9A)
#define BMP280_REG_CWORD10		(0x9C)
#define BMP280_REG_CWORD11		(0x9E)
#define BMP280_REG_CWORD12		(0xA0)

#define BATCH_DRY_RUN			(0x1)
#define BATCH_WAKE_UPON_FIFO_FULL	(0x2)
#define WR				(0)
#define RD				(1)
/* _buf_push expects this scan order */
#define BMP_SCAN_PRES			(0)
#define BMP_SCAN_TEMP			(1)
#define BMP_SCAN_TIMESTAMP		(2)
#define BMP_SCAN_N			(2)
#define BMP_DEV_PRES			BMP_SCAN_PRES
#define BMP_DEV_TEMP			BMP_SCAN_TEMP
#define BMP_DEV_N			BMP_SCAN_TIMESTAMP
/* debug flags */
#define BMP_DBG_SPEW_MSG		(1 << 0)
#define BMP_DBG_SPEW_PRESSURE		(1 << 1)
#define BMP_DBG_SPEW_TEMPERATURE	(1 << 2)
#define BMP_DBG_SPEW_PRESSURE_RAW	(1 << 3)
#define BMP_DBG_SPEW_TEMPERATURE_UC	(1 << 4)
#define BMP_DBG_VAL_PRESSURE		(1 << 5)
#define BMP_DBG_VAL_TEMP		(1 << 6)

enum BMP_ATTR {
	BMP_ATTR_ENABLE,
	BMP_ATTR_VENDOR,
	BMP_ATTR_BATCH_FLAGS,
	BMP_ATTR_BATCH_PERIOD,
	BMP_ATTR_BATCH_TIMEOUT,
	BMP_ATTR_FLUSH,
	BMP_ATTR_FIFO_RSRV_EVNT_CNT,
	BMP_ATTR_FIFO_MAX_EVNT_CNT,
	BMP_ATTR_PRES_PART,
	BMP_ATTR_PRES_VERSION,
	BMP_ATTR_PRES_MILLIAMP,
	BMP_ATTR_TEMP_PART,
	BMP_ATTR_TEMP_VERSION,
	BMP_ATTR_TEMP_MILLIAMP,
};

enum BMP_INFO {
	BMP_INFO_DATA = 0,
	BMP_INFO_VER,
	BMP_INFO_ERRS,
	BMP_INFO_RESET,
	BMP_INFO_REGS,
	BMP_INFO_DBG,
	BMP_INFO_PRESSURE_SPEW,
	BMP_INFO_TEMPERATURE_SPEW,
	BMP_INFO_PRESSURE_UC_SPEW,
	BMP_INFO_TEMPERATURE_UC_SPEW,
	BMP_INFO_LIMIT_MAX,
};

/* regulator names in order of powering on */
static char *bmp_vregs[] = {
	"vdd",
	"vddio",
};

static char *bmp_configs[] = {
	"auto",
	"mpu",
	"host",
};

static u8 bmp_ids[] = {
	BMP_REG_ID_BMP180,
	BMP_REG_ID_BMP280,
	0x57,
	0x58,
};

static unsigned short bmp_i2c_addrs[] = {
	0x76,
	0x77,
};

struct bmp_iio_float {
	int ival;
	int micro;
};

struct bmp_scale {
	unsigned long delay_ms;
	struct bmp_iio_float resolution;
	const char *power_ma;
};

struct bmp_hal_dev {
	int version;
	unsigned int scale_i_max;
	unsigned int scale_dflt;
	struct bmp_scale *scale;
	struct bmp_iio_float scale_float;
};

union bmp_rom {
	struct bmp180_rom {
		s16 ac1;
		s16 ac2;
		s16 ac3;
		u16 ac4;
		u16 ac5;
		u16 ac6;
		s16 b1;
		s16 b2;
		s16 mb;
		s16 mc;
		s16 md;
	} bmp180;
	struct bmp280_rom {
		u16 dig_T1;
		s16 dig_T2;
		s16 dig_T3;
		u16 dig_P1;
		s16 dig_P2;
		s16 dig_P3;
		s16 dig_P4;
		s16 dig_P5;
		s16 dig_P6;
		s16 dig_P7;
		s16 dig_P8;
		s16 dig_P9;
		s16 reserved;
	} bmp280;
} rom;

struct bmp_cmode {
	unsigned int t_us;
	u8 t_sb;
};

struct bmp_state {
	struct i2c_client *i2c;
	struct iio_trigger *trig;
	struct regulator_bulk_data vreg[ARRAY_SIZE(bmp_vregs)];
	struct delayed_work dw;
	struct bmp_hal *hal;		/* Hardware Abstaction Layer */
	union bmp_rom rom;		/* calibration data */
	unsigned int info;		/* info data to return */
	unsigned int dbg;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int enable;		/* enable status */
	unsigned int poll_delay_us;	/* global sampling delay */
	unsigned int delay_us[BMP_DEV_N]; /* device sampling delay */
	unsigned int scale_i;		/* oversampling index */
	unsigned int scale_user;	/* user oversampling index */
	unsigned int mpu_batch_flags;	/* MPU supported batch flags */
	unsigned int batch_flags;	/* batch flags */
	unsigned int batch_period_us;	/* batch period us */
	unsigned int batch_timeout_ms;	/* batch timeout ms */
	unsigned int fifo_reserve;	/* fifoReservedEventCount */
	unsigned int fifo_max;		/* fifoMaxEventCount */
	u16 i2c_addr;			/* I2C address */
	u8 dev_id;			/* device ID */
	bool iio_ts_en;			/* use IIO timestamps */
	bool shutdown;			/* shutdown active flag */
	bool suspend;			/* suspend active flag */
	bool initd;			/* set if initialized */
	bool mpu_en;			/* if device behind MPU */
	bool fifo_en;			/* MPU FIFO enable */
	bool port_en[2];		/* enable status of MPU write port */
	int port_id[2];			/* MPU port ID */
	u8 data_out;			/* write value to mode register */
	s32 ut;				/* uncompensated temperature */
	s32 up;				/* uncompensated pressure */
	s32 t_fine;			/* temperature used in pressure calc */
	s32 temp;			/* true temperature */
	int pressure;			/* true pressure hPa/100 Pa/1 mBar */
	s64 ts;				/* sample data timestamp */
	u8 nvi_config;			/* NVI configuration */
};

struct bmp_hal {
	struct bmp_hal_dev *p;
	struct bmp_hal_dev *t;
	const char *part;
	u8 rom_addr_start;
	u8 rom_size;
	bool rom_big_endian;
	u8 mode_mask;
	struct bmp_cmode *cmode_tbl;
	int (*bmp_read)(struct iio_dev *indio_dev);
	unsigned int mpu_id;
};


static s64 bmp_get_time_ns(struct bmp_state *st)
{
	struct timespec ts;

	if (st->iio_ts_en)
		return iio_get_time_ns();

	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

static void bmp_err(struct bmp_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

static int bmp_i2c_rd(struct bmp_state *st, u8 reg, u16 len, u8 *val)
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
		bmp_err(st);
		return -EIO;
	}

	return 0;
}

static int bmp_i2c_wr(struct bmp_state *st, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];

	if (st->i2c_addr) {
		buf[0] = reg;
		buf[1] = val;
		msg.addr = st->i2c_addr;
		msg.flags = 0;
		msg.len = 2;
		msg.buf = buf;
		if (i2c_transfer(st->i2c->adapter, &msg, 1) != 1) {
			bmp_err(st);
			return -EIO;
		}
	}

	return 0;
}

static int bmp_nvi_mpu_bypass_request(struct bmp_state *st)
{
	int ret = 0;
#if BMP_NVI_MPU_SUPPORT
	int i;

	if (st->mpu_en) {
		for (i = 0; i < BMP_MPU_RETRY_COUNT; i++) {
			ret = nvi_mpu_bypass_request(true);
			if ((!ret) || (ret == -EPERM))
				break;

			msleep(BMP_MPU_RETRY_DELAY_MS);
		}
		if (ret == -EPERM)
			ret = 0;
	}
#endif /* BMP_NVI_MPU_SUPPORT */
	return ret;
}

static int bmp_nvi_mpu_bypass_release(struct bmp_state *st)
{
	int ret = 0;

#if BMP_NVI_MPU_SUPPORT
	if (st->mpu_en)
		ret = nvi_mpu_bypass_release();
#endif /* BMP_NVI_MPU_SUPPORT */
	return ret;
}

static int bmp_mode_wr(struct bmp_state *st, u8 mode)
{
	int ret;

#if BMP_NVI_MPU_SUPPORT
	if (st->mpu_en && !st->i2c->irq) {
		ret = nvi_mpu_data_out(st->port_id[WR], mode);
	} else {
		ret = bmp_nvi_mpu_bypass_request(st);
		if (!ret) {
			if (st->i2c->irq) {
				ret = bmp_i2c_wr(st, BMP_REG_CTRL,
						 BMP_REG_CTRL_MODE_SLEEP);
				if (mode & st->hal->mode_mask) {
					udelay(BMP_HW_DELAY_POR_MS);
					ret |= bmp_i2c_wr(st, BMP_REG_CTRL,
							  mode);
				}
			} else {
				ret = bmp_i2c_wr(st, BMP_REG_CTRL, mode);
			}
			bmp_nvi_mpu_bypass_release(st);
		}
	}
#else /* BMP_NVI_MPU_SUPPORT */
	ret = bmp_i2c_wr(st, BMP_REG_CTRL, mode);
#endif /* BMP_NVI_MPU_SUPPORT */
	if (!ret)
		st->data_out = mode;
	return ret;
}

static int bmp_vreg_dis(struct bmp_state *st, unsigned int i)
{
	int ret = 0;

	if (st->vreg[i].ret && (st->vreg[i].consumer != NULL)) {
		ret = regulator_disable(st->vreg[i].consumer);
		if (ret) {
			dev_err(&st->i2c->dev, "%s %s ERR\n",
				__func__, st->vreg[i].supply);
		} else {
			st->vreg[i].ret = 0;
			if (st->dbg & BMP_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s %s\n",
					 __func__, st->vreg[i].supply);
		}
	}
	return ret;
}

static int bmp_vreg_dis_all(struct bmp_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = ARRAY_SIZE(bmp_vregs); i > 0; i--)
		ret |= bmp_vreg_dis(st, (i - 1));
	return ret;
}

static int bmp_vreg_en(struct bmp_state *st, unsigned int i)
{
	int ret = 0;

	if ((!st->vreg[i].ret) && (st->vreg[i].consumer != NULL)) {
		ret = regulator_enable(st->vreg[i].consumer);
		if (ret) {
			dev_err(&st->i2c->dev, "%s %s ERR\n",
				__func__, st->vreg[i].supply);
		} else {
			st->vreg[i].ret = 1;
			if (st->dbg & BMP_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s %s\n",
					 __func__, st->vreg[i].supply);
			ret = 1; /* flag regulator state change */
		}
	}
	return ret;
}

static int bmp_vreg_en_all(struct bmp_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(bmp_vregs); i++)
		ret |= bmp_vreg_en(st, i);
	return ret;
}

static void bmp_vreg_exit(struct bmp_state *st)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(bmp_vregs); i++) {
		if (st->vreg[i].consumer != NULL) {
			devm_regulator_put(st->vreg[i].consumer);
			st->vreg[i].consumer = NULL;
			if (st->dbg & BMP_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s %s\n",
					 __func__, st->vreg[i].supply);
		}
	}
}

static int bmp_vreg_init(struct bmp_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(bmp_vregs); i++) {
		st->vreg[i].supply = bmp_vregs[i];
		st->vreg[i].ret = 0;
		st->vreg[i].consumer = devm_regulator_get(&st->i2c->dev,
							  st->vreg[i].supply);
		if (IS_ERR(st->vreg[i].consumer)) {
			ret |= PTR_ERR(st->vreg[i].consumer);
			dev_err(&st->i2c->dev, "%s ret %d for %s\n",
				__func__, ret, st->vreg[i].supply);
			st->vreg[i].consumer = NULL;
		} else {
			if (st->dbg & BMP_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s %s\n",
					 __func__, st->vreg[i].supply);
		}
	}
	return ret;
}

static int bmp_vreg_sts(struct bmp_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(bmp_vregs); i++) {
		if (st->vreg[i].consumer != NULL)
			break;
	}
	if (i < ARRAY_SIZE(bmp_vregs)) {
		/* ret == number of regulators on */
		for (i = 0; i < ARRAY_SIZE(bmp_vregs); i++) {
			if (st->vreg[i].ret)
				ret++;
		}
	} else {
		/* no regulator support (can assume always on) */
		ret = -EINVAL;
	}
	return ret;
}

static int bmp_pm(struct bmp_state *st, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = bmp_vreg_en_all(st);
		if (ret)
			mdelay(BMP_HW_DELAY_POR_MS);
	} else {
		if (st->i2c->irq) {
			ret = bmp_vreg_sts(st);
			if ((ret < 0) || (ret == ARRAY_SIZE(bmp_vregs))) {
				ret = bmp_mode_wr(st, BMP_REG_CTRL_MODE_SLEEP);
			} else if (ret > 0) {
				bmp_vreg_en_all(st);
				mdelay(BMP_HW_DELAY_POR_MS);
				ret = bmp_mode_wr(st, BMP_REG_CTRL_MODE_SLEEP);
			}
		}
		ret |= bmp_vreg_dis_all(st);
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		dev_err(&st->i2c->dev, "%s pwr=%x ERR=%d\n",
			__func__, enable, ret);
	} else {
		if (st->dbg & BMP_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s pwr=%x\n",
				 __func__, enable);
	}
	return ret;
}

static int bmp_port_free(struct bmp_state *st, int port)
{
	int ret = 0;

#if BMP_NVI_MPU_SUPPORT
	if (st->port_id[port] >= 0) {
		ret = nvi_mpu_port_free(st->port_id[port]);
		if (!ret)
			st->port_id[port] = -1;
	}
#endif /* BMP_NVI_MPU_SUPPORT */
	return ret;
}

static int bmp_ports_free(struct bmp_state *st)
{
	int ret;

	ret = bmp_port_free(st, WR);
	ret |= bmp_port_free(st, RD);
	return ret;
}

static void bmp_pm_exit(struct bmp_state *st)
{
	bmp_ports_free(st);
	bmp_pm(st, false);
	bmp_vreg_exit(st);
}

static int bmp_pm_init(struct bmp_state *st)
{
	int ret;

	st->enable = 0;
	st->delay_us[BMP_DEV_PRES] = (BMP_POLL_DELAY_MS_DFLT * 1000);
	st->delay_us[BMP_DEV_TEMP] = (BMP_POLL_DELAY_MS_DFLT * 1000);
	st->poll_delay_us = (BMP_POLL_DELAY_MS_DFLT * 1000);
	st->shutdown = false;
	st->initd = false;
	st->mpu_en = false;
	st->fifo_en = true;
	st->port_en[WR] = false;
	st->port_en[RD] = false;
	st->port_id[WR] = -1;
	st->port_id[RD] = -1;
	bmp_vreg_init(st);
	ret = bmp_pm(st, true);
	return ret;
}

static int bmp_port_enable(struct bmp_state *st, int port, bool enable)
{
	int ret = 0;

#if BMP_NVI_MPU_SUPPORT
	if ((enable != st->port_en[port]) && (st->port_id[port] >= 0)) {
		ret = nvi_mpu_enable(st->port_id[port], enable, st->fifo_en);
		if (!ret)
			st->port_en[port] = enable;
	}
#endif /* BMP_NVI_MPU_SUPPORT */
	return ret;
}

static int bmp_ports_enable(struct bmp_state *st, bool enable)
{
	int ret;

	ret = bmp_port_enable(st, WR, enable);
	ret |= bmp_port_enable(st, RD, enable);
	return ret;
}

static int bmp_wr(struct bmp_state *st, u8 reg, u8 val)
{
	int ret = 0;

	ret = bmp_nvi_mpu_bypass_request(st);
	if (!ret) {
		ret = bmp_i2c_wr(st, reg, val);
		bmp_nvi_mpu_bypass_release(st);
	}
	return ret;
}

static int bmp_reset(struct bmp_state *st)
{
	int ret;

	ret = bmp_wr(st, BMP_REG_RESET, BMP_REG_RESET_VAL);
	if (!ret)
		mdelay(BMP_HW_DELAY_POR_MS);
	return ret;
}

static int bmp_mode(struct bmp_state *st)
{
	u8 mode;
	u8 t_sb;
	unsigned int t_us;
	unsigned int i;
	int ret;

	if (st->dev_id == BMP_REG_ID_BMP180) {
		mode = st->scale_i << BMP180_REG_CTRL_OSS;
		mode |= BMP180_REG_CTRL_MODE_TEMP;
	} else {
		mode = st->scale_i + 1;
		mode = ((mode << BMP280_REG_CTRL_OSRS_T) |
			(mode << BMP280_REG_CTRL_OSRS_P));
		mode |= BMP280_REG_CTRL_MODE_FORCED1;
	}
	if (st->i2c->irq) {
		i = 0;
		t_sb = 0;
		while (st->hal->cmode_tbl[i].t_us) {
			t_sb = st->hal->cmode_tbl[i].t_sb;
			t_us = st->hal->cmode_tbl[i].t_us;
			if (st->poll_delay_us >=
					    st->hal->cmode_tbl[i].t_us)
				break;

			i++;
			if (!st->mpu_en) {
				t_us -= st->hal->cmode_tbl[i].t_us;
				t_us >>= 1;
				t_us += st->hal->cmode_tbl[i].t_us;
				if (st->poll_delay_us > t_us)
					break;
			}
		}
		t_sb <<= BMP280_REG_CONFIG_T_SB;
		bmp_wr(st, BMP280_REG_CONFIG, t_sb);
		mode |= BMP280_REG_CTRL_MODE_NORMAL;
	}
	ret = bmp_mode_wr(st, mode);
	return ret;
}

static unsigned int bmp_buf_index(unsigned int size, unsigned int *bytes)
{
	unsigned int index;

	if (!(*bytes % size))
		index = *bytes;
	else
		index = *bytes - *bytes % size + size;
	*bytes = index + size;
	return index;
}

static void bmp_buf_push(struct iio_dev *indio_dev, s64 ts)
{
	struct bmp_state *st = iio_priv(indio_dev);
	unsigned char buf[16];
	unsigned int n;
	unsigned int i;
	unsigned int bytes = 0;

	if (!iio_buffer_enabled(indio_dev))
		return;

	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, BMP_SCAN_PRES)) {
		n = sizeof(st->pressure);
		i = bmp_buf_index(n, &bytes);
		memcpy(&buf[i], &st->pressure, n);
		if (st->dbg & BMP_DBG_SPEW_PRESSURE_RAW)
			dev_info(&st->i2c->dev, "pr %d %lld\n", st->up, ts);
		if (st->dbg & BMP_DBG_SPEW_PRESSURE)
			dev_info(&st->i2c->dev, "p %d %lld\n",
				 st->pressure, ts);
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, BMP_SCAN_TEMP)) {
		n = sizeof(st->temp);
		i = bmp_buf_index(n, &bytes);
		memcpy(&buf[i], &st->temp, n);
		if (st->dbg & BMP_DBG_SPEW_TEMPERATURE_UC)
			dev_info(&st->i2c->dev, "tr %d %lld\n", st->ut, ts);
		if (st->dbg & BMP_DBG_SPEW_TEMPERATURE)
			dev_info(&st->i2c->dev, "t %d %lld\n", st->temp, ts);
	}
	if (indio_dev->buffer->scan_timestamp) {
		n = sizeof(ts);
		i = bmp_buf_index(n, &bytes);
		memcpy(&buf[i], &ts, n);
	}
	iio_push_to_buffers(indio_dev, buf);
}

static void bmp_calc_180(struct bmp_state *st)
{
	long X1, X2, X3, B3, B5, B6, p;
	unsigned long B4, B7;
	long pressure;

	X1 = ((st->ut - st->rom.bmp180.ac6) * st->rom.bmp180.ac5) >> 15;
	X2 = st->rom.bmp180.mc * (1 << 11) / (X1 + st->rom.bmp180.md);
	B5 = X1 + X2;
	if (!(st->dbg & BMP_DBG_VAL_TEMP))
		st->temp = (B5 + 8) >> 4;
	B6 = B5 - 4000;
	X1 = (st->rom.bmp180.b2 * ((B6 * B6) >> 12)) >> 11;
	X2 = (st->rom.bmp180.ac2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((st->rom.bmp180.ac1 << 2) + X3) << st->scale_i) + 2) >> 2;
	X1 = (st->rom.bmp180.ac3 * B6) >> 13;
	X2 = (st->rom.bmp180.b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (st->rom.bmp180.ac4 * (unsigned long)(X3 + 32768)) >> 15;
	B7 = ((unsigned long)st->up - B3) * (50000 >> st->scale_i);
	if (B7 < 0x80000000)
		p = (B7 << 1) / B4;
	else
		p = (B7 / B4) << 1;
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	pressure = p + ((X1 + X2 + 3791) >> 4);
	if (!(st->dbg & BMP_DBG_VAL_PRESSURE))
		st->pressure = (int)pressure;
}

static int bmp_read_sts_180(struct bmp_state *st, u8 *data, s64 ts)
{
	s32 val;
	int ret = 0;

	/* BMP180_REG_CTRL_SCO is 0 when data is ready */
	if (!(data[0] & (1 << BMP180_REG_CTRL_SCO))) {
		ret = -1;
		if (data[0] == 0x0A) { /* temperature */
			st->ut = ((data[2] << 8) + data[3]);
			st->data_out = BMP180_REG_CTRL_MODE_PRES |
					(st->scale_i << BMP180_REG_CTRL_OSS);
		} else { /* pressure */
			val = ((data[2] << 16) + (data[3] << 8) +
						data[4]) >> (8 - st->scale_i);
			st->data_out = BMP180_REG_CTRL_MODE_TEMP;
			st->up = val;
			bmp_calc_180(st);
			st->ts = ts;
			ret = 1;
		}
	}
	return ret;
}

static int bmp_read_180(struct iio_dev *indio_dev)
{
	struct bmp_state *st = iio_priv(indio_dev);
	s64 ts;
	u8 data[5];
	int ret;

	ret = bmp_i2c_rd(st, BMP_REG_CTRL, 5, data);
	if (ret)
		return ret;

	ts = bmp_get_time_ns(st);
	ret = bmp_read_sts_180(st, data, ts);
	if (ret > 0) {
		bmp_buf_push(indio_dev, ts);
		bmp_i2c_wr(st, BMP_REG_CTRL, st->data_out);
	} else if (ret < 0) {
		bmp_i2c_wr(st, BMP_REG_CTRL, st->data_out);
	}
	return 0;
}

static void bmp_calc_temp_280(struct bmp_state *st)
{
	s32 adc_t;
	s32 var1;
	s32 var2;

	adc_t = st->ut;
	adc_t >>= (4 - st->scale_i);
	var1 = adc_t >> 3;
	var1 -= ((s32)st->rom.bmp280.dig_T1 << 1);
	var1 *= (s32)st->rom.bmp280.dig_T2;
	var1 >>= 11;
	var2 = adc_t >> 4;
	var2 -= (s32)st->rom.bmp280.dig_T1;
	var2 *= var2;
	var2 >>= 12;
	var2 *= (s32)st->rom.bmp280.dig_T3;
	var2 >>= 14;
	st->t_fine = var1 + var2;
	if (!(st->dbg & BMP_DBG_VAL_TEMP))
		st->temp = (st->t_fine * 5 + 128) >> 8;
}

static int bmp_calc_pres_280(struct bmp_state *st)
{
	s32 adc_p;
	s32 var1;
	s32 var2;
	s32 var3;
	u32 p;

	adc_p = st->up;
	var1 = st->t_fine >> 1;
	var1 -= 64000;
	var2 = var1 >> 2;
	var2 *= var2;
	var3 = var2;
	var2 >>= 11;
	var2 *= st->rom.bmp280.dig_P6;
	var2 += ((var1 * st->rom.bmp280.dig_P5) << 1);
	var2 >>= 2;
	var2 += (st->rom.bmp280.dig_P4 << 16);
	var3 >>= 13;
	var3 *= st->rom.bmp280.dig_P3;
	var3 >>= 3;
	var1 *= st->rom.bmp280.dig_P2;
	var1 >>= 1;
	var1 += var3;
	var1 >>= 18;
	var1 += 32768;
	var1 *= st->rom.bmp280.dig_P1;
	var1 >>= 15;
	if (!var1)
		return -1;

	p = ((u32)(((s32)1048576) - adc_p) - (var2 >> 12)) * 3125;
	if (p < 0x80000000)
		p = (p << 1) / ((u32)var1);
	else
		p = (p / (u32)var1) << 1;
	var3 = p >> 3;
	var3 *= var3;
	var3 >>= 13;
	var1 = (s32)st->rom.bmp280.dig_P9 * var3;
	var1 >>= 12;
	var2 = (s32)(p >> 2);
	var2 *= (s32)st->rom.bmp280.dig_P8;
	var2 >>= 13;
	var3 = var1 + var2 + st->rom.bmp280.dig_P7;
	var3 >>= 4;
	p = (u32)((s32)p + var3);
	if (!(st->dbg & BMP_DBG_VAL_PRESSURE))
		st->pressure = (int)p;
	return 1;
}

static int bmp_read_sts_280(struct bmp_state *st, u8 *data, s64 ts)
{
	u8 sts;
	s32 val;
	int ret;

	sts = data[1] & BMP280_REG_CTRL_MODE_MASK;
	if ((sts == BMP280_REG_CTRL_MODE_FORCED1) ||
					 (sts == BMP280_REG_CTRL_MODE_FORCED2))
		return 0;

	val = (data[4] << 16) | (data[5] << 8) | data[6];
	val = le32_to_cpup(&val);
	val >>= 4;
	st->up = val;
	val = (data[7] << 16) | (data[8] << 8) | data[9];
	val = le32_to_cpup(&val);
	val >>= 4;
	st->ut = val;
	bmp_calc_temp_280(st);
	ret = bmp_calc_pres_280(st);
	if (ret > 0)
		st->ts = ts;
	return ret;
}

static int bmp_read_280(struct iio_dev *indio_dev)
{
	struct bmp_state *st = iio_priv(indio_dev);
	s64 ts;
	u8 data[10];
	int ret;

	ret = bmp_i2c_rd(st, BMP280_REG_STATUS, 10, data);
	if (ret)
		return ret;

	ts = bmp_get_time_ns(st);
	ret = bmp_read_sts_280(st, data, ts);
	if (ret > 0) {
		bmp_buf_push(indio_dev, ts);
		bmp_i2c_wr(st, BMP_REG_CTRL, st->data_out);
	}
	return 0;
}

#if BMP_NVI_MPU_SUPPORT
static void bmp_mpu_handler_280(u8 *data, unsigned int len, s64 ts, void *p_val)
{
	struct iio_dev *indio_dev;
	struct bmp_state *st;
	int ret;

	indio_dev = (struct iio_dev *)p_val;
	st = iio_priv(indio_dev);
	if (!ts) {
		/* no timestamp means flush done */
		bmp_buf_push(indio_dev, 0);
		return;
	}

	if (st->enable) {
		ret = bmp_read_sts_280(st, data, ts);
		if (ret > 0)
			bmp_buf_push(indio_dev, ts);
	}
}

static void bmp_mpu_handler_180(u8 *data, unsigned int len, s64 ts, void *p_val)
{
	struct iio_dev *indio_dev;
	struct bmp_state *st;
	int ret;

	indio_dev = (struct iio_dev *)p_val;
	st = iio_priv(indio_dev);
	if (!ts) {
		/* no timestamp means flush done */
		bmp_buf_push(indio_dev, 0);
		return;
	}

	if (st->enable) {
		ret = bmp_read_sts_180(st, data, ts);
		if (ret > 0) {
			bmp_buf_push(indio_dev, ts);
			nvi_mpu_data_out(st->port_id[WR], st->data_out);
		} else if (ret < 0) {
			nvi_mpu_data_out(st->port_id[WR], st->data_out);
		}
	}
}
#endif /* BMP_NVI_MPU_SUPPORT */

static void bmp_work(struct work_struct *ws)
{
	struct bmp_state *st = container_of((struct delayed_work *)ws,
					    struct bmp_state, dw);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);

	mutex_lock(&indio_dev->mlock);
	if (st->enable) {
		st->hal->bmp_read(indio_dev);
		schedule_delayed_work(&st->dw,
				      usecs_to_jiffies(st->poll_delay_us));
	}
	mutex_unlock(&indio_dev->mlock);
}

static unsigned int bmp_poll_delay(struct iio_dev *indio_dev)
{
	struct bmp_state *st = iio_priv(indio_dev);
	unsigned int i;
	unsigned int delay_us = -1;

	for (i = 0; i < BMP_SCAN_N; i++) {
		if (iio_scan_mask_query(indio_dev, indio_dev->buffer, i)) {
			if (st->delay_us[i] < delay_us)
				delay_us = st->delay_us[i];
		}
	}
	if (delay_us == -1)
		delay_us = (BMP_POLL_DELAY_MS_DFLT * 1000);
	return delay_us;
}

static int bmp_delay(struct bmp_state *st,
		     unsigned int delay_us, int scale_user)
{
	unsigned int i;
	int ret;
	int ret_t = 0;

	if (scale_user) {
		i = scale_user - 1;
	} else {
		for (i = (st->hal->p->scale_i_max - 1); i > 0; i--) {
			if (delay_us >= (st->hal->p->scale[i].delay_ms * 1000))
				break;
		}
	}
	if (i != st->scale_i) {
		ret = 0;
#if BMP_NVI_MPU_SUPPORT
		if (st->mpu_en)
			ret = nvi_mpu_delay_ms(st->port_id[WR],
					       st->hal->p->scale[i].delay_ms);
#endif /* BMP_NVI_MPU_SUPPORT */
		if (ret < 0)
			ret_t |= ret;
		else
			st->scale_i = i;
	}
	if (delay_us < (st->hal->p->scale[st->scale_i].delay_ms * 1000))
		delay_us = (st->hal->p->scale[st->scale_i].delay_ms * 1000);
	if (delay_us != st->poll_delay_us) {
		if (st->dbg & BMP_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s: %u\n",
				 __func__, delay_us);
#if BMP_NVI_MPU_SUPPORT
		ret = 0;
		if (st->mpu_en)
			ret = nvi_mpu_delay_us(st->port_id[RD],
					       (unsigned long)delay_us);
		if (ret)
			ret_t |= ret;
		else
#endif /* BMP_NVI_MPU_SUPPORT */
			st->poll_delay_us = delay_us;
	}
	return ret_t;
}

static int bmp_init_hw(struct bmp_state *st)
{
	u8 *p_rom8;
	u16 *p_rom16;
	int i;
	int ret = 0;

	st->ut = 0;
	st->up = 0;
	st->temp = 0;
	st->pressure = 0;
	p_rom8 = (u8 *)&st->rom;
	ret = bmp_nvi_mpu_bypass_request(st);
	if (!ret) {
		ret = bmp_i2c_rd(st, st->hal->rom_addr_start,
				 st->hal->rom_size, p_rom8);
		bmp_nvi_mpu_bypass_release(st);
	}
	if (ret)
		return ret;

	p_rom16 = (u16 *)&st->rom;
	for (i = 0; i < (st->hal->rom_size >> 1); i++) {
		if (st->hal->rom_big_endian)
			*p_rom16 = be16_to_cpup(p_rom16);
		else
			*p_rom16 = le16_to_cpup(p_rom16);
		p_rom16++;
	}
	st->initd = true;
	return ret;
}

static int bmp_dis(struct bmp_state *st)
{
	int ret = 0;

	if (st->mpu_en)
		ret = bmp_ports_enable(st, false);
	else
		cancel_delayed_work_sync(&st->dw);
	if (!ret)
		st->enable = 0;
	return ret;
}

static int bmp_disable(struct iio_dev *indio_dev)
{
	struct bmp_state *st = iio_priv(indio_dev);
	int ret;

	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  BMP_SCAN_PRES)))
		st->dbg &= ~BMP_DBG_VAL_PRESSURE;
	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  BMP_SCAN_TEMP)))
		st->dbg &= ~BMP_DBG_VAL_TEMP;
	ret = bmp_dis(st);
	if (!ret)
		bmp_pm(st, false);
	return ret;
}

static int bmp_en(struct bmp_state *st)
{
	int ret = 0;

	bmp_pm(st, true);
	if (!st->initd)
		ret = bmp_init_hw(st);
	return ret;
}

static int bmp_enable(struct iio_dev *indio_dev)
{
	struct bmp_state *st = iio_priv(indio_dev);
	unsigned int dev = 0;
	int ret = -EINVAL;

	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, BMP_SCAN_PRES))
		dev |= (1 << BMP_DEV_PRES);
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, BMP_SCAN_TEMP))
		dev |= (1 << BMP_DEV_TEMP);
	if (dev) {
		ret = bmp_en(st);
		if (!ret) {
			ret |= bmp_delay(st, bmp_poll_delay(indio_dev),
					 st->scale_user);
			ret |= bmp_mode(st);
			if (st->mpu_en)
				ret |= bmp_ports_enable(st, true);
			if (ret) {
				bmp_disable(indio_dev);
			} else {
				st->enable = dev;
				if (!st->mpu_en)
					schedule_delayed_work(&st->dw,
							      usecs_to_jiffies(
							   st->poll_delay_us));
			}
		}
	}
	return ret;
}

static int bmp_able(struct iio_dev *indio_dev, unsigned int en)
{
	int ret;

	if (en)
		ret = bmp_enable(indio_dev);
	else
		ret = bmp_disable(indio_dev);
	return ret;
}

static ssize_t bmp_attr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmp_state *st = iio_priv(indio_dev);
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
	case BMP_ATTR_ENABLE:
		msg = "ATTR_ENABLE";
		old = st->enable;
		ret = bmp_able(indio_dev, new);
		break;

#if BMP_NVI_MPU_SUPPORT
	case BMP_ATTR_BATCH_FLAGS:
		msg = "ATTR_PRES_BATCH_FLAGS";
		old = st->batch_flags;
		st->batch_flags = new;
		break;

	case BMP_ATTR_BATCH_PERIOD:
		msg = "ATTR_PRES_BATCH_PERIOD";
		if (new < st->hal->p->scale[st->scale_i].delay_ms * 1000)
			new = st->hal->p->scale[st->scale_i].delay_ms * 1000;
		old = st->batch_period_us;
		st->batch_period_us = new;
		break;

	case BMP_ATTR_BATCH_TIMEOUT:
		msg = "ATTR_BATCH_TIMEOUT";
		old = st->batch_timeout_ms;
		if (((st->batch_flags & BATCH_WAKE_UPON_FIFO_FULL) &
		     st->mpu_batch_flags) == (st->batch_flags &
					      BATCH_WAKE_UPON_FIFO_FULL)) {
			/* if special flags supported */
			if (new) {
				/* if timeout */
				if (st->batch_flags == (st->batch_flags &
							st->mpu_batch_flags))
					/* if batch && options supported */
					ret = nvi_mpu_batch(st->port_id[RD],
							    st->batch_flags,
							   st->batch_period_us,
							    new);
				else
					/* batch || options not supported */
					ret = -EINVAL;
			} else {
				if (st->mpu_batch_flags) {
					/* if batch supported */
					ret = nvi_mpu_batch(st->port_id[RD],
							    st->batch_flags,
							   st->batch_period_us,
							    new);
				} else {
					/* batch used to set delay */
					if (!(st->batch_flags & BATCH_DRY_RUN))
						ret = bmp_delay(st,
								new * 1000,
							       st->scale_user);
					else
						ret = 0;
				}
			}
			if (ret >= 0) {
				st->batch_timeout_ms = new;
				ret = 0;
			}
		} else {
			/* special flags not supported */
			ret = -EINVAL;
		}
		break;

	case BMP_ATTR_FLUSH:
		msg = "ATTR_FLUSH";
		if (st->mpu_batch_flags)
			ret = nvi_mpu_flush(st->port_id[RD]);
		else
			ret = -EINVAL;
		break;
#endif /* BMP_NVI_MPU_SUPPORT */

	default:
		msg = "ATTR_UNKNOWN";
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & BMP_DBG_SPEW_MSG) {
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

static ssize_t bmp_attr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmp_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret;

	switch (this_attr->address) {
	case BMP_ATTR_ENABLE:
		return sprintf(buf, "%x\n", st->enable);

	case BMP_ATTR_VENDOR:
		return sprintf(buf, "%s\n", BMP_VENDOR);

#if BMP_NVI_MPU_SUPPORT
	case BMP_ATTR_BATCH_FLAGS:
		return sprintf(buf, "%u\n", st->batch_flags);

	case BMP_ATTR_BATCH_PERIOD:
		return sprintf(buf, "%uus\n", st->batch_period_us);

	case BMP_ATTR_BATCH_TIMEOUT:
		return sprintf(buf, "%ums\n", st->batch_timeout_ms);

	case BMP_ATTR_FLUSH:
		return sprintf(buf, "%u\n", st->mpu_batch_flags);

	case BMP_ATTR_FIFO_RSRV_EVNT_CNT:
		ret = nvi_mpu_fifo(st->port_id[RD],
				   &st->fifo_reserve, &st->fifo_max);
		if (ret)
			return ret;

		return sprintf(buf, "%u\n", st->fifo_reserve);

	case BMP_ATTR_FIFO_MAX_EVNT_CNT:
		ret = nvi_mpu_fifo(st->port_id[RD],
				   &st->fifo_reserve, &st->fifo_max);
		if (ret)
			return ret;

		return sprintf(buf, "%u\n", st->fifo_max);
#endif /* BMP_NVI_MPU_SUPPORT */

	case BMP_ATTR_PRES_PART:
		return sprintf(buf, "%s pressure\n", st->hal->part);

	case BMP_ATTR_PRES_VERSION:
		return sprintf(buf, "%u\n", st->hal->p->version);

	case BMP_ATTR_PRES_MILLIAMP:
		return sprintf(buf, "%s\n",
			       st->hal->p->scale[st->scale_i].power_ma);

	case BMP_ATTR_TEMP_PART:
		return sprintf(buf, "%s temperature\n", st->hal->part);

	case BMP_ATTR_TEMP_VERSION:
		return sprintf(buf, "%u\n", st->hal->t->version);

	case BMP_ATTR_TEMP_MILLIAMP:
		return sprintf(buf, "%s\n",
			       st->hal->t->scale[st->scale_i].power_ma);

	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t bmp_data_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct bmp_state *st = iio_priv(dev_get_drvdata(dev));
	unsigned int info;
	int ret;

	ret = kstrtouint(buf, 10, &info);
	if ((ret) || (info >= BMP_INFO_LIMIT_MAX))
		return -EINVAL;

	st->info = info;
	switch (info) {
	case BMP_INFO_DATA:
		st->dbg = 0;
		break;

	case BMP_INFO_DBG:
		st->dbg ^= BMP_DBG_SPEW_MSG;
		break;

	case BMP_INFO_PRESSURE_SPEW:
		st->dbg ^= BMP_DBG_SPEW_PRESSURE;
		break;

	case BMP_INFO_TEMPERATURE_SPEW:
		st->dbg ^= BMP_DBG_SPEW_TEMPERATURE;
		break;

	case BMP_INFO_PRESSURE_UC_SPEW:
		st->dbg ^= BMP_DBG_SPEW_PRESSURE_RAW;
		break;

	case BMP_INFO_TEMPERATURE_UC_SPEW:
		st->dbg ^= BMP_DBG_SPEW_TEMPERATURE_UC;
		break;

	default:
		break;
	}

	return count;
}

static ssize_t bmp_data_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmp_state *st = iio_priv(indio_dev);
	enum BMP_INFO info;
	ssize_t t;
	u8 data[11];
	u16 *cal;
	bool enable;
	unsigned int i;
	int ret = 0;

	info = st->info;
	st->info = BMP_INFO_DATA;
	enable = st->enable;
	switch (info) {
	case BMP_INFO_DATA:
		t = sprintf(buf, "PRESSURE: %d  UC: %d\n",
			    st->pressure, st->up);
		t += sprintf(buf + t, "TEMP: %d  UC: %d\n",
			     st->temp, st->ut);
		if (st->dev_id != BMP_REG_ID_BMP180)
			t += sprintf(buf + t, "TEMP_FINE: %d\n",
				     st->t_fine);
		t += sprintf(buf + t, "TIMESTAMP: %lld\n", st->ts);
		return t;

	case BMP_INFO_VER:
		return sprintf(buf, "version=%u\n", BMP_VERSION_DRIVER);

	case BMP_INFO_ERRS:
		i = st->errs;
		st->errs = 0;
		return sprintf(buf, "error count=%u\n", i);

	case BMP_INFO_RESET:
		bmp_dis(st);
		bmp_en(st);
		ret = bmp_reset(st);
		bmp_able(indio_dev, enable);
		if (ret)
			return sprintf(buf, "reset ERR %d\n", ret);
		else
			return sprintf(buf, "reset done\n");

	case BMP_INFO_REGS:
		if (!st->initd) {
			t = sprintf(buf, "calibration: NEED ENABLE\n");
		} else {
			t = sprintf(buf, "calibration:\n");
			cal = &st->rom.bmp280.dig_T1;
			for (i = 0; i < st->hal->rom_size; i = i + 2)
				t += sprintf(buf + t, "%#2x=%#2x\n",
					     st->hal->rom_addr_start + i,
					     *cal++);
		}
		ret = bmp_nvi_mpu_bypass_request(st);
		if (!ret) {
			ret = bmp_i2c_rd(st, BMP_REG_ID, 1, data);
			ret |= bmp_i2c_rd(st, BMP280_REG_STATUS,
					  10, &data[1]);
			bmp_nvi_mpu_bypass_release(st);
		}
		if (ret) {
			t += sprintf(buf + t, "registers: ERR %d\n", ret);
		} else {
			t += sprintf(buf + t, "registers:\n");
			t += sprintf(buf + t, "%#2x=%#2x\n",
				     BMP_REG_ID, data[0]);
			for (i = 0; i < 10; i++)
				t += sprintf(buf + t, "%#2x=%#2x\n",
					     BMP280_REG_STATUS + i,
					     data[i + 1]);
		}
		return t;

	case BMP_INFO_DBG:
		return sprintf(buf, "debug spew=%x\n",
			       !!(st->dbg & BMP_DBG_SPEW_MSG));

	case BMP_INFO_PRESSURE_SPEW:
		return sprintf(buf, "pressure spew=%x\n",
			       !!(st->dbg & BMP_DBG_SPEW_PRESSURE));

	case BMP_INFO_TEMPERATURE_SPEW:
		return sprintf(buf, "temperature spew=%x\n",
			       !!(st->dbg & BMP_DBG_SPEW_TEMPERATURE));

	case BMP_INFO_PRESSURE_UC_SPEW:
		return sprintf(buf, "pressure_uncalibrated spew=%x\n",
			       !!(st->dbg & BMP_DBG_SPEW_PRESSURE_RAW));

	case BMP_INFO_TEMPERATURE_UC_SPEW:
		return sprintf(buf, "temperature_uncalibrated spew=%x\n",
			       !!(st->dbg & BMP_DBG_SPEW_TEMPERATURE_UC));

	default:
		break;
	}

	return -EINVAL;
}

static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store, BMP_ATTR_ENABLE);
static IIO_DEVICE_ATTR(pressure_part, S_IRUGO,
		       bmp_attr_show, NULL, BMP_ATTR_PRES_PART);
static IIO_DEVICE_ATTR(pressure_vendor, S_IRUGO,
		       bmp_attr_show, NULL, BMP_ATTR_VENDOR);
static IIO_DEVICE_ATTR(pressure_version, S_IRUGO,
		       bmp_attr_show, NULL, BMP_ATTR_PRES_VERSION);
static IIO_DEVICE_ATTR(pressure_milliamp, S_IRUGO,
		       bmp_attr_show, NULL, BMP_ATTR_PRES_MILLIAMP);
#if BMP_NVI_MPU_SUPPORT
static IIO_DEVICE_ATTR(pressure_batch_flags, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_BATCH_FLAGS);
static IIO_DEVICE_ATTR(pressure_batch_period, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_BATCH_PERIOD);
static IIO_DEVICE_ATTR(pressure_batch_timeout, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_BATCH_TIMEOUT);
static IIO_DEVICE_ATTR(pressure_flush, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store, BMP_ATTR_FLUSH);
static IIO_DEVICE_ATTR(pressure_fifo_reserved_event_count,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_FIFO_RSRV_EVNT_CNT);
static IIO_DEVICE_ATTR(pressure_fifo_max_event_count,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_FIFO_MAX_EVNT_CNT);
#endif /* BMP_NVI_MPU_SUPPORT */
static IIO_DEVICE_ATTR(temp_part, S_IRUGO,
		       bmp_attr_show, NULL, BMP_ATTR_TEMP_PART);
static IIO_DEVICE_ATTR(temp_vendor, S_IRUGO,
		       bmp_attr_show, NULL, BMP_ATTR_VENDOR);
static IIO_DEVICE_ATTR(temp_version, S_IRUGO,
		       bmp_attr_show, NULL, BMP_ATTR_TEMP_VERSION);
static IIO_DEVICE_ATTR(temp_milliamp, S_IRUGO,
		       bmp_attr_show, NULL, BMP_ATTR_TEMP_MILLIAMP);
#if BMP_NVI_MPU_SUPPORT
static IIO_DEVICE_ATTR(temp_batch_flags, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_BATCH_FLAGS);
static IIO_DEVICE_ATTR(temp_batch_period, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_BATCH_PERIOD);
static IIO_DEVICE_ATTR(temp_batch_timeout, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_BATCH_TIMEOUT);
static IIO_DEVICE_ATTR(temp_flush, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store, BMP_ATTR_FLUSH);
static IIO_DEVICE_ATTR(temp_fifo_reserved_event_count,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_FIFO_RSRV_EVNT_CNT);
static IIO_DEVICE_ATTR(temp_fifo_max_event_count, S_IRUGO | S_IWUSR | S_IWGRP,
		       bmp_attr_show, bmp_attr_store,
		       BMP_ATTR_FIFO_MAX_EVNT_CNT);
#endif /* BMP_NVI_MPU_SUPPORT */
static DEVICE_ATTR(data, S_IRUGO | S_IWUSR | S_IWGRP,
		   bmp_data_show, bmp_data_store);

static struct attribute *bmp_attrs[] = {
	&iio_dev_attr_enable.dev_attr.attr,
	&iio_dev_attr_pressure_part.dev_attr.attr,
	&iio_dev_attr_pressure_vendor.dev_attr.attr,
	&iio_dev_attr_pressure_version.dev_attr.attr,
	&iio_dev_attr_pressure_milliamp.dev_attr.attr,
#if BMP_NVI_MPU_SUPPORT
	&iio_dev_attr_pressure_batch_flags.dev_attr.attr,
	&iio_dev_attr_pressure_batch_period.dev_attr.attr,
	&iio_dev_attr_pressure_batch_timeout.dev_attr.attr,
	&iio_dev_attr_pressure_flush.dev_attr.attr,
	&iio_dev_attr_pressure_fifo_reserved_event_count.dev_attr.attr,
	&iio_dev_attr_pressure_fifo_max_event_count.dev_attr.attr,
#endif /* BMP_NVI_MPU_SUPPORT */
	&iio_dev_attr_temp_part.dev_attr.attr,
	&iio_dev_attr_temp_vendor.dev_attr.attr,
	&iio_dev_attr_temp_version.dev_attr.attr,
	&iio_dev_attr_temp_milliamp.dev_attr.attr,
#if BMP_NVI_MPU_SUPPORT
	&iio_dev_attr_temp_batch_flags.dev_attr.attr,
	&iio_dev_attr_temp_batch_period.dev_attr.attr,
	&iio_dev_attr_temp_batch_timeout.dev_attr.attr,
	&iio_dev_attr_temp_flush.dev_attr.attr,
	&iio_dev_attr_temp_fifo_reserved_event_count.dev_attr.attr,
	&iio_dev_attr_temp_fifo_max_event_count.dev_attr.attr,
#endif /* BMP_NVI_MPU_SUPPORT */
	&dev_attr_data.attr,
	NULL
};

static struct attribute_group bmp_attr_group = {
	.name = BMP_NAME,
	.attrs = bmp_attrs
};

static int bmp_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct bmp_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_PRESSURE:
			*val = st->pressure;
			return IIO_VAL_INT;

		case IIO_TEMP:
			*val = st->temp;
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->enable)
			*val = st->poll_delay_us;
		else
			*val = st->hal->p->scale[st->scale_i].delay_ms * 1000;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PEAK:
		switch (chan->type) {
		case IIO_PRESSURE:
			*val = BMP_PRES_MAX_RANGE_IVAL;
			*val2 = BMP_PRES_MAX_RANGE_MICRO;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_TEMP:
			*val = BMP_TEMP_MAX_RANGE_IVAL;
			*val2 = BMP_TEMP_MAX_RANGE_MICRO;
			return IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_PEAK_SCALE:
		switch (chan->type) {
		case IIO_PRESSURE:
			*val = st->hal->p->scale[st->scale_i].resolution.ival;
			*val2 = st->hal->p->scale[st->scale_i].resolution.micro;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_TEMP:
			*val = st->hal->t->scale[st->scale_i].resolution.ival;
			*val2 = st->hal->t->scale[st->scale_i].resolution.micro;
			return IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

		return -EINVAL;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_PRESSURE:
			/* Pa -> hPa*/
			*val = st->hal->p->scale_float.ival;
			*val2 = st->hal->p->scale_float.micro;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_TEMP:
			*val = st->hal->t->scale_float.ival;
			*val2 = st->hal->t->scale_float.micro;
			return IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_PRESSURE:
			/* Pa -> hPa*/
			*val = BMP_PRES_OFFSET_IVAL;
			*val2 = BMP_PRES_OFFSET_MICRO;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_TEMP:
			*val = BMP_TEMP_OFFSET_IVAL;
			*val2 = BMP_TEMP_OFFSET_MICRO;
			return IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}

	return ret;
}

static int bmp_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask)
{
	struct bmp_state *st = iio_priv(indio_dev);
	const char *msg;
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
		if (chan->scan_index > BMP_SCAN_N) {
			ret = -EINVAL;
			break;
		}

		if (st->dev_id == BMP_REG_ID_BMP180)
			/* since we rotate between acquiring data for
			 * pressure and temperature on the BMP180 we
			 * need to go twice as fast.
			 */
			val >>= 1;
		old = st->delay_us[chan->scan_index];
		st->delay_us[chan->scan_index] = val;
		ret = bmp_delay(st, bmp_poll_delay(indio_dev), st->scale_user);
		if (st->enable && st->i2c->irq && !ret)
			ret = bmp_mode(st);
		if (ret)
			st->delay_us[chan->scan_index] = old;
		break;

	case IIO_CHAN_INFO_SCALE:
		msg = "IIO_CHAN_INFO_SCALE";
		old = st->scale_user;
		switch (chan->type) {
		case IIO_PRESSURE:
			if ((val >= 0) && (val <= st->hal->p->scale_i_max)) {
				ret = bmp_delay(st, st->poll_delay_us, val);
				if (!ret) {
					st->scale_user = val;
					if (st->enable && st->i2c->irq)
						ret = bmp_mode(st);
				}
			} else {
				ret = -EINVAL;
			}
			break;

		case IIO_TEMP:
			ret = -EINVAL;
			break;

		default:
			ret = -EINVAL;
		}

		break;

	case IIO_CHAN_INFO_RAW:
		msg = "IIO_CHAN_INFO_RAW";
		switch (chan->type) {
		case IIO_PRESSURE:
			old = st->pressure;
			st->pressure = val;
			st->dbg |= BMP_DBG_VAL_PRESSURE;
			break;

		case IIO_TEMP:
			old = st->temp;
			st->temp = val;
			st->dbg |= BMP_DBG_VAL_TEMP;
			break;

		default:
			ret = -EINVAL;
		}

		break;

	default:
		msg = "IIO_CHAN_INFO_UNKNOWN";
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & BMP_DBG_SPEW_MSG) {
		if (ret) {
			dev_err(&st->i2c->dev, "%s %s chan=%d %d:%d->%d:%d\n",
				__func__, msg, chan->scan_index,
				 old, old2, val, val2);
			dev_err(&st->i2c->dev, "%s ERR=%d mask=%ld type=%d\n",
				__func__, ret, mask, chan->type);
		} else {
			dev_info(&st->i2c->dev, "%s %s chan=%d %d:%d->%d:%d\n",
				 __func__, msg, chan->scan_index,
				 old, old2, val, val2);
		}
	}
	return ret;
}

static const struct iio_info bmp_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &bmp_attr_group,
	.read_raw = &bmp_read_raw,
	.write_raw = &bmp_write_raw,
};

static const struct iio_chan_spec bmp_channels[] = {
	{
		.type			= IIO_PRESSURE,
		.scan_index		= BMP_SCAN_PRES,
		.scan_type		= IIO_ST('s', 32, 32, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type			= IIO_TEMP,
		.scan_index		= BMP_SCAN_TEMP,
		.scan_type		= IIO_ST('s', 32, 32, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	IIO_CHAN_SOFT_TIMESTAMP(BMP_SCAN_TIMESTAMP)
};

static int bmp_buffer_preenable(struct iio_dev *indio_dev)
{
	struct bmp_state *st = iio_priv(indio_dev);

	if (st->shutdown || st->suspend)
		return -EINVAL;

	return 0;
}

static int bmp_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;

	ret = bmp_enable(indio_dev);
	/* never return > 0 to IIO buffer engine */
	if (ret > 0)
		ret = 0;
	return ret;
}

static const struct iio_buffer_setup_ops bmp_buffer_setup_ops = {
	/* iio_sw_buffer_preenable:
	 * Generic function for equal sized ring elements + 64 bit timestamp
	 * Assumes that any combination of channels can be enabled.
	 * Typically replaced to implement restrictions on what combinations
	 * can be captured (hardware scan modes).
	 */
	.preenable = &bmp_buffer_preenable,
	/* iio_triggered_buffer_postenable:
	 * Generic function that simply attaches the pollfunc to the trigger.
	 * Replace this to mess with hardware state before we attach the
	 * trigger.
	 */
	.postenable = &bmp_buffer_postenable,
	/* this driver relies on the NVS HAL to power off this device with the
	 * master enable.
	 *.predisable = N/A
	 *.postdisable = N/A
	 */
};

static const struct iio_trigger_ops bmp_trigger_ops = {
	.owner = THIS_MODULE,
};

static int bmp_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bmp_state *st = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (st->enable)
		ret = bmp_disable(indio_dev);
	st->suspend = true;
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & BMP_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int bmp_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bmp_state *st = iio_priv(indio_dev);

	st->suspend = false;
	if (st->dbg & BMP_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(bmp_pm_ops, bmp_suspend, bmp_resume);

static struct bmp_scale bmp_scale_pres_180[] = {
	{
		.delay_ms		= 5,
		.resolution		= {
			.ival		= 0,
			.micro		= 10000,
		},
		.power_ma		= "0.003",
	},
	{
		.delay_ms		= 8,
		.resolution		= {
			.ival		= 0,
			.micro		= 10000,
		},
		.power_ma		= "0.005",
	},
	{
		.delay_ms		= 14,
		.resolution		= {
			.ival		= 0,
			.micro		= 10000,
		},
		.power_ma		= "0.007",
	},
	{
		.delay_ms		= 26,
		.resolution		= {
			.ival		= 0,
			.micro		= 10000,
		},
		.power_ma		= "0.012",
	},
};

static struct bmp_scale bmp_scale_temp_180[] = {
	{
		.delay_ms		= 5,
		.resolution		= {
			.ival		= 0,
			.micro		= 100000,
		},
		.power_ma		= "0.003",
	},
	{
		.delay_ms		= 8,
		.resolution		= {
			.ival		= 0,
			.micro		= 100000,
		},
		.power_ma		= "0.005",
	},
	{
		.delay_ms		= 14,
		.resolution		= {
			.ival		= 0,
			.micro		= 100000,
		},
		.power_ma		= "0.007",
	},
	{
		.delay_ms		= 26,
		.resolution		= {
			.ival		= 0,
			.micro		= 100000,
		},
		.power_ma		= "0.012",
	},
};

static struct bmp_hal_dev bmp_hal_dev_pres_180 = {
	.version			= 1,
	.scale_i_max			= ARRAY_SIZE(bmp_scale_pres_180),
	.scale_dflt			= BMP180_RANGE_DFLT,
	.scale				= bmp_scale_pres_180,
	.scale_float			= {
		.ival			= 0,
		.micro			= 10000,
	}
};

static struct bmp_hal_dev bmp_hal_dev_temp_180 = {
	.version			= 1,
	.scale_i_max			= ARRAY_SIZE(bmp_scale_temp_180),
	.scale_dflt			= BMP180_RANGE_DFLT,
	.scale				= bmp_scale_temp_180,
	.scale_float			= {
		.ival			= 0,
		.micro			= 100000,
	}
};

static struct bmp_hal bmp_hal_180 = {
	.p				= &bmp_hal_dev_pres_180,
	.t				= &bmp_hal_dev_temp_180,
	.part				= BMP180_NAME,
	.rom_addr_start			= BMP180_REG_AC1,
	.rom_size			= 22,
	.rom_big_endian			= true,
	.mode_mask			= BMP180_REG_CTRL_MODE_MASK,
	.cmode_tbl			= NULL,
	.bmp_read			= &bmp_read_180,
};

static struct bmp_scale bmp_scale_pres_280[] = {
	{
		.delay_ms		= 9,
		.resolution		= {
			.ival		= 0,
			.micro		= 28700,
		},
		.power_ma		= "0.00274",
	},
	{
		.delay_ms		= 12,
		.resolution		= {
			.ival		= 0,
			.micro		= 14300,
		},
		.power_ma		= "0.00417",
	},
	{
		.delay_ms		= 18,
		.resolution		= {
			.ival		= 0,
			.micro		= 7200,
		},
		.power_ma		= "0.00702",
	},
	{
		.delay_ms		= 30,
		.resolution		= {
			.ival		= 0,
			.micro		= 3600,
		},
		.power_ma		= "0.0127",
	},
	{
		.delay_ms		= 57,
		.resolution		= {
			.ival		= 0,
			.micro		= 1800,
		},
		.power_ma		= "0.0248",
	},
};

static struct bmp_scale bmp_scale_temp_280[] = {
	{
		.delay_ms		= 9,
		.resolution		= {
			.ival		= 0,
			.micro		= 5000,
		},
		.power_ma		= "0.00274",
	},
	{
		.delay_ms		= 12,
		.resolution		= {
			.ival		= 0,
			.micro		= 2500,
		},
		.power_ma		= "0.00417",
	},
	{
		.delay_ms		= 18,
		.resolution		= {
			.ival		= 0,
			.micro		= 1200,
		},
		.power_ma		= "0.00702",
	},
	{
		.delay_ms		= 30,
		.resolution		= {
			.ival		= 0,
			.micro		= 600,
		},
		.power_ma		= "0.0127",
	},
	{
		.delay_ms		= 57,
		.resolution		= {
			.ival		= 0,
			.micro		= 300,
		},
		.power_ma		= "0.0248",
	},
};

static struct bmp_hal_dev bmp_hal_dev_pres_280 = {
	.version			= 1,
	.scale_i_max			= ARRAY_SIZE(bmp_scale_pres_280),
	.scale_dflt			= BMP280_RANGE_DFLT,
	.scale				= bmp_scale_pres_280,
	.scale_float			= {
		.ival			= 0,
		.micro			= 10000,
	}
};

static struct bmp_hal_dev bmp_hal_dev_temp_280 = {
	.version			= 1,
	.scale_i_max			= ARRAY_SIZE(bmp_scale_temp_280),
	.scale_dflt			= BMP280_RANGE_DFLT,
	.scale				= bmp_scale_temp_280,
	.scale_float			= {
		.ival			= 0,
		.micro			= 10000,
	}
};

static struct bmp_cmode bmp_cmode_280[] = {
	{
		.t_us			= 4000000,
		.t_sb			= 0x07,
	},
	{
		.t_us			= 2000000,
		.t_sb			= 0x06,
	},
	{
		.t_us			= 1000000,
		.t_sb			= 0x05,
	},
	{
		.t_us			= 500000,
		.t_sb			= 0x04,
	},
	{
		.t_us			= 250000,
		.t_sb			= 0x03,
	},
	{
		.t_us			= 125000,
		.t_sb			= 0x02,
	},
	{
		.t_us			= 62500,
		.t_sb			= 0x01,
	},
	{
		.t_us			= 500,
		.t_sb			= 0x00,
	},
	{},
};

static struct bmp_hal bmp_hal_280 = {
	.p				= &bmp_hal_dev_pres_280,
	.t				= &bmp_hal_dev_temp_280,
	.part				= BMP280_NAME,
	.rom_addr_start			= BMP280_REG_CWORD00,
	.rom_size			= 26,
	.rom_big_endian			= false,
	.mode_mask			= BMP280_REG_CTRL_MODE_MASK,
	.cmode_tbl			= bmp_cmode_280,
	.bmp_read			= &bmp_read_280,
#if BMP_NVI_MPU_SUPPORT
	.mpu_id				= PRESSURE_ID_BMP280,
#endif /* BMP_NVI_MPU_SUPPORT */
};

static int bmp_id_hal(struct bmp_state *st)
{
	int ret = 0;

	switch (st->dev_id) {
	case BMP_REG_ID_BMP280:
		st->hal = &bmp_hal_280;
		break;

	case BMP_REG_ID_BMP180:
		st->hal = &bmp_hal_180;
		break;

	default:
		dev_err(&st->i2c->dev, "%s ERR: Unknown device\n", __func__);
		st->hal = &bmp_hal_180; /* to prevent NULL pointers */
		ret = -ENODEV;
		break;
	}

	st->scale_user = st->hal->p->scale_dflt;
	return ret;
}

static int bmp_id_compare(struct bmp_state *st, u8 val, const char *name)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(bmp_ids); i++) {
		if (val == bmp_ids[i]) {
			if ((st->dev_id == BMP_REG_ID_BMP180) &&
							   (st->dev_id != val))
				dev_err(&st->i2c->dev, "%s ERR: %x != %s\n",
					__func__, st->dev_id, name);
			if (val != BMP_REG_ID_BMP180)
				/* BMP280 may have more ID's than 0x56 */
				val = BMP_REG_ID_BMP280;
			st->dev_id = val;
			break;
		}
	}
	if (!st->dev_id) {
		ret = -ENODEV;
		dev_err(&st->i2c->dev, "%s ERR: ID %x != %s\n",
			__func__, val, name);
	} else {
		dev_dbg(&st->i2c->dev, "%s using ID %x for %s\n",
			__func__, st->dev_id, name);
	}
	return ret;
}

static int bmp_id_dev(struct iio_dev *indio_dev, const char *name)
{
	struct bmp_state *st = iio_priv(indio_dev);
#if BMP_NVI_MPU_SUPPORT
	struct nvi_mpu_port nmp;
	u8 config_boot;
#endif /* BMP_NVI_MPU_SUPPORT */
	u8 val = 0;
	int ret;

	if (!strcmp(name, BMP180_NAME))
		st->dev_id = BMP_REG_ID_BMP180;
	else if (!strcmp(name, BMP280_NAME))
		st->dev_id = BMP_REG_ID_BMP280;
#if BMP_NVI_MPU_SUPPORT
	config_boot = st->nvi_config & NVI_CONFIG_BOOT_MASK;
	if ((config_boot == NVI_CONFIG_BOOT_MPU) && (!st->dev_id)) {
		dev_err(&st->i2c->dev, "%s ERR: NVI_CONFIG_BOOT_MPU && %s\n",
			__func__, name);
		config_boot = NVI_CONFIG_BOOT_AUTO;
	}
	if (config_boot == NVI_CONFIG_BOOT_AUTO) {
		nmp.addr = st->i2c_addr | 0x80;
		nmp.reg = BMP_REG_ID;
		nmp.ctrl = 1;
		ret = nvi_mpu_dev_valid(&nmp, &val);
		dev_info(&st->i2c->dev, "%s AUTO ID=%x ret=%d\n",
			 __func__, val, ret);
		/* see mpu_iio.h for possible return values */
		if ((ret == -EAGAIN) || (ret == -EBUSY))
			return -EAGAIN;

		if (!ret)
			ret = bmp_id_compare(st, val, name);
		if ((!ret) || ((ret == -EIO) && (st->dev_id)))
			config_boot = NVI_CONFIG_BOOT_MPU;
	}
	if (config_boot == NVI_CONFIG_BOOT_MPU) {
		st->mpu_en = true;
		bmp_id_hal(st);
		nmp.addr = st->i2c_addr | 0x80;
		nmp.data_out = 0;
		nmp.delay_ms = 0;
		nmp.delay_us = st->poll_delay_us;
		if ((st->hal->cmode_tbl != NULL) && st->i2c->irq)
			nmp.shutdown_bypass = true;
		else
			nmp.shutdown_bypass = false;
		nmp.ext_driver = (void *)indio_dev;
		if (st->dev_id == BMP_REG_ID_BMP180) {
			nmp.reg = BMP_REG_CTRL;
			nmp.ctrl = 6; /* MPU FIFO can't handle odd size */
			nmp.handler = &bmp_mpu_handler_180;
		} else {
			nmp.reg = BMP280_REG_STATUS;
			nmp.ctrl = 10; /* MPU FIFO can't handle odd size */
			nmp.handler = &bmp_mpu_handler_280;
		}
		nmp.rate_scale = 34;
		ret = nvi_mpu_port_alloc(&nmp);
		dev_dbg(&st->i2c->dev, "%s MPU port/ret=%d\n",
			__func__, ret);
		if (ret < 0)
			return ret;

		st->port_id[RD] = ret;
		ret = 0;
		if ((st->hal->cmode_tbl == NULL) || !st->i2c->irq) {
			st->i2c->irq = 0;
			nmp.addr = st->i2c_addr;
			nmp.reg = BMP_REG_CTRL;
			nmp.ctrl = 1;
			nmp.data_out = st->data_out;
			nmp.delay_ms = st->hal->p->scale[st->scale_i].delay_ms;
			nmp.delay_us = 0;
			nmp.shutdown_bypass = false;
			nmp.handler = NULL;
			nmp.ext_driver = NULL;
			nmp.type = SECONDARY_SLAVE_TYPE_PRESSURE;
			nmp.id = st->hal->mpu_id;
			ret = nvi_mpu_port_alloc(&nmp);
			dev_dbg(&st->i2c->dev, "%s MPU port/ret=%d\n",
				__func__, ret);
			if (ret < 0) {
				bmp_ports_free(st);
				dev_err(&st->i2c->dev, "%s ERR %d\n",
					__func__, ret);
			} else {
				st->port_id[WR] = ret;
				ret = nvi_mpu_batch(st->port_id[RD], 0, 0, 0);
				if (ret > 0)
					st->mpu_batch_flags = ret;
				ret = 0;
			}
		}
		return ret;
	}
#endif /* BMP_NVI_MPU_SUPPORT */
	/* NVI_CONFIG_BOOT_HOST */
	st->mpu_en = false;
	ret = bmp_i2c_rd(st, BMP_REG_ID, 1, &val);
	dev_dbg(&st->i2c->dev, "%s Host read ID=%x ret=%d\n",
		__func__, val, ret);
	if (!ret) {
		ret = bmp_id_compare(st, val, name);
		if (!ret)
			ret = bmp_id_hal(st);
	}
	return ret;
}

static int bmp_id_i2c(struct iio_dev *indio_dev,
		      const struct i2c_device_id *id)
{
	struct bmp_state *st = iio_priv(indio_dev);
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(bmp_i2c_addrs); i++) {
		if (st->i2c->addr == bmp_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(bmp_i2c_addrs)) {
		st->i2c_addr = st->i2c->addr;
		ret = bmp_id_dev(indio_dev, id->name);
	} else {
		for (i = 0; i < ARRAY_SIZE(bmp_i2c_addrs); i++) {
			st->i2c_addr = bmp_i2c_addrs[i];
			ret = bmp_id_dev(indio_dev, BMP_NAME);
			if ((ret == -EAGAIN) || (!ret))
				break;
		}
	}
	if (ret)
		st->i2c_addr = 0;
	return ret;
}

static void bmp_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bmp_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	st->shutdown = true;
	if (st->enable)
		bmp_disable(indio_dev);
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & BMP_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int bmp_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bmp_state *st = iio_priv(indio_dev);

	if (st != NULL) {
		bmp_shutdown(client);
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
		bmp_pm_exit(st);
		iio_device_free(indio_dev);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int bmp_of_dt(struct i2c_client *client, struct bmp_state *st)
{
	struct device_node *dn = client->dev.of_node;
	char const *pchar;
	u8 cfg;

	/* common NVS programmable parameters */
	st->iio_ts_en = of_property_read_bool(dn, "iio_timestamps");
	/* this device supports these programmable parameters */
	if (!(of_property_read_string(dn, "nvi_config", &pchar))) {
		for (cfg = 0; cfg < ARRAY_SIZE(bmp_configs); cfg++) {
			if (!strcasecmp(pchar, bmp_configs[cfg])) {
				st->nvi_config = cfg;
				break;
			}
		}
	}

	return 0;
}

static int bmp_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct bmp_state *st;
	int ret;

	dev_info(&client->dev, "%s %s\n", id->name, __func__);
	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		dev_err(&client->dev, "%s iio_device_alloc ERR\n", __func__);
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	st->i2c = client;
	i2c_set_clientdata(client, indio_dev);
	if (client->dev.of_node) {
		ret = bmp_of_dt(client, st);
		if (ret)
			goto bmp_probe_err;
	}

	bmp_pm_init(st);
	ret = bmp_id_i2c(indio_dev, id);
	if (ret == -EAGAIN)
		goto bmp_probe_again;
	else if (ret)
		goto bmp_probe_err;

	bmp_init_hw(st);
	bmp_pm(st, false);
	if (!st->mpu_en)
		INIT_DELAYED_WORK(&st->dw, bmp_work);
	indio_dev->buffer = iio_kfifo_allocate(indio_dev);
	if (!indio_dev->buffer) {
		dev_err(&client->dev, "%s iio_kfifo_allocate ERR\n", __func__);
		ret = -ENOMEM;
		goto bmp_probe_err;
	}

	indio_dev->buffer->scan_timestamp = true;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = bmp_channels;
	indio_dev->num_channels = ARRAY_SIZE(bmp_channels);
	indio_dev->name = BMP_NAME;
	indio_dev->info = &bmp_iio_info;
	indio_dev->setup_ops = &bmp_buffer_setup_ops;
	ret = iio_buffer_register(indio_dev, indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto bmp_probe_err;

	st->trig = iio_trigger_alloc("%s-dev%d",
				     indio_dev->name, indio_dev->id);
	if (st->trig == NULL) {
		dev_err(&client->dev, "%s iio_allocate_trigger ERR\n",
			__func__);
		ret = -ENOMEM;
		goto bmp_probe_err;
	}

	st->trig->dev.parent = &st->i2c->dev;
	st->trig->ops = &bmp_trigger_ops;
	ret = iio_trigger_register(st->trig);
	if (ret) {
		dev_err(&client->dev, "%s iio_trigger_register ERR\n",
			__func__);
		ret = -ENOMEM;
		goto bmp_probe_err;
	}
	indio_dev->trig = st->trig;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_device_register(indio_dev);
	if (ret)
		goto bmp_probe_err;

	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

bmp_probe_err:
	dev_err(&client->dev, "%s ERR %d\n", __func__, ret);
bmp_probe_again:
	bmp_remove(client);
	return ret;
}

static const struct i2c_device_id bmp_i2c_device_id[] = {
	{ BMP_NAME, 0 },
	{ BMP180_NAME, 0 },
	{ BMP280_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, bmp_i2c_device_id);

static const struct of_device_id bmp_of_match[] = {
	{ .compatible = "bmp,bmpX80", },
	{ .compatible = "bmp,bmp180", },
	{ .compatible = "bmp,bmp280", },
	{},
};

MODULE_DEVICE_TABLE(of, bmp_of_match);

static struct i2c_driver bmp_driver = {
	.class				= I2C_CLASS_HWMON,
	.probe				= bmp_probe,
	.remove				= bmp_remove,
	.shutdown			= bmp_shutdown,
	.driver				= {
		.name			= BMP_NAME,
		.owner			= THIS_MODULE,
		.of_match_table		= of_match_ptr(bmp_of_match),
		.pm			= &bmp_pm_ops,
	},
	.id_table			= bmp_i2c_device_id,
};

static int __init bmp_init(void)
{
	return i2c_add_driver(&bmp_driver);
}

static void __exit bmp_exit(void)
{
	i2c_del_driver(&bmp_driver);
}

late_initcall(bmp_init);
module_exit(bmp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMPX80 driver");
MODULE_AUTHOR("NVIDIA Corporation");

