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
 * 1. If AKM_NVI_MPU_SUPPORT (defined below) is set, the code is included to
 *    support the device behind an Invensense MPU running an NVI (NVidia/
 *    Invensense) driver.
 *    If AKM_NVI_MPU_SUPPORT is 0 then this driver is only for the device in a
 *    stand-alone configuration without any dependencies on an Invensense MPU.
 * 2. Device tree platform configuration nvi_config:
 *    - auto = automatically detect if connected to host or MPU
 *    - mpu = connected to MPU
 *    - host = connected to host
 *    This is only available if AKM_NVI_MPU_SUPPORT is set.
 * 3. device in board file:
 *    - ak89xx = automatically detect the device
 *    - force the device for:
 *      - ak8963
 *      - ak8975
 *      - ak09911
 * If you have no clue what the device is and don't know how it is
 * connected then use auto and akm89xx.  The auto-detect mechanisms are for
 * platforms that have multiple possible configurations but takes longer to
 * initialize.  No device identification and connect testing is done for
 * specific configurations.
 *
 * An interrupt can be used to configure the device.  When an interrupt is
 * defined in struct i2c_client.irq, the driver is configured to only use the
 * device's continuous mode if the device supports it.  If the device does not
 * support continuous mode, then the interrupt is not used.
 * If the device is connected to the MPU, the interrupt from the board file is
 * used as a SW flag.  The interrupt itself is never touched so any value can
 * be used.  If the struct i2c_client.irq is > 0, then the driver will only use
 * the continuous modes of the device if supported.  This frees the MPU
 * auxiliary port used for writes.  This configuration would be used if another
 * MPU auxiliary port was needed for another device connected to the MPU.
 * If the device is connected to the host, the delay timing used in continuous
 * mode is the one closest to the device's supported modes.  Example: A 70ms
 * request will use the 125ms from the possible 10ms and 125ms on the AK8963.
 * If the device is connected to the MPU, the delay timing used in continuous
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

#define AKM_NVI_MPU_SUPPORT		(1) /* includes NVI MPU code */

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
#if AKM_NVI_MPU_SUPPORT
#include <linux/mpu_iio.h>
#endif /* AKM_NVI_MPU_SUPPORT */

#define AKM_VERSION_DRIVER		(101)
#define AKM_VENDOR			"AsahiKASEI"
#define AKM_NAME			"ak89xx"
#define AKM_NAME_AK8963			"ak8963"
#define AKM_NAME_AK8975			"ak8975"
#define AKM_NAME_AK09911		"ak09911"
#define AKM_OFFSET_IVAL			(0)
#define AKM_OFFSET_MICRO		(0)
#define AKM_HW_DELAY_POR_MS		(50)
#define AKM_HW_DELAY_TSM_MS		(10) /* Time Single Measurement */
#define AKM_HW_DELAY_US			(100)
#define AKM_HW_DELAY_ROM_ACCESS_US	(200)
#define AKM_POLL_DELAY_MS_DFLT		(200)
#define AKM_MPU_RETRY_COUNT		(50)
#define AKM_MPU_RETRY_DELAY_MS		(20)
#define AKM_ERR_CNT_MAX			(20)
/* HW registers */
#define AKM_WIA_ID			(0x48)
#define AKM_DEVID_AK8963		(0x01)
#define AKM_DEVID_AK8975		(0x03)
#define AKM_DEVID_AK09911		(0x05)
#define AKM_REG_WIA			(0x00)
#define AKM_REG_WIA2			(0x01)
#define AKM_BIT_DRDY			(0x01)
#define AKM_BIT_DOR			(0x02)
#define AKM_BIT_DERR			(0x04)
#define AKM_BIT_HOFL			(0x08)
#define AKM_BIT_BITM			(0x10)
#define AKM_BIT_SRST			(0x01)
#define AKM_BIT_SELF			(0x40)
#define AKM_MODE_POWERDOWN		(0x00)
#define AKM_MODE_SINGLE			(0x01)

#define BATCH_DRY_RUN			(0x1)
#define BATCH_WAKE_UPON_FIFO_FULL	(0x2)
#define WR				(0)
#define RD				(1)
#define AXIS_X				(0)
#define AXIS_Y				(1)
#define AXIS_Z				(2)
#define AXIS_N				(3)
/* _buf_push expects this scan order */
#define AKM_SCAN_X			AXIS_X
#define AKM_SCAN_Y			AXIS_Y
#define AKM_SCAN_Z			AXIS_Z
#define AKM_SCAN_TIMESTAMP		AXIS_N
/* debug flags */
#define AKM_DBG_SPEW_MSG		(1 << 0)
#define AKM_DBG_SPEW_MAGNETIC_FIELD	(1 << 1)
#define AKM_DBG_SPEW_MAGNETIC_FIELD_UC	(1 << 2)
#define AKM_DBG_AXIS_X			(1 << 3)
#define AKM_DBG_AXIS_Y			(1 << 4)
#define AKM_DBG_AXIS_Z			(1 << 5)

enum AKM_ATTR {
	AKM_ATTR_ENABLE,
	AKM_ATTR_PART,
	AKM_ATTR_VENDOR,
	AKM_ATTR_VERSION,
	AKM_ATTR_MILLIAMP,
	AKM_ATTR_BATCH_FLAGS,
	AKM_ATTR_BATCH_PERIOD,
	AKM_ATTR_BATCH_TIMEOUT,
	AKM_ATTR_FLUSH,
	AKM_ATTR_FIFO_RSRV_EVNT_CNT,
	AKM_ATTR_FIFO_MAX_EVNT_CNT,
	AKM_ATTR_SELF_TEST,
};

enum AKM_INFO {
	AKM_INFO_DATA = 0,
	AKM_INFO_VER,
	AKM_INFO_ERRS,
	AKM_INFO_RESET,
	AKM_INFO_REGS,
	AKM_INFO_DBG,
	AKM_INFO_MAGNETIC_FIELD_SPEW,
	AKM_INFO_MAGNETIC_FIELD_UC_SPEW,
	AKM_INFO_LIMIT_MAX,
};

/* regulator names in order of powering on */
static char *akm_vregs[] = {
	"vdd",
	"vid",
};

static char *akm_configs[] = {
	"auto",
	"mpu",
	"host",
};

static unsigned short akm_i2c_addrs[] = {
	0x0C,
	0x0D,
	0x0E,
	0x0F,
};

struct akm_iio_float {
	int ival;
	int micro;
};

struct akm_scale {
	struct akm_iio_float max_range;
	struct akm_iio_float resolution;
	struct akm_iio_float scale;
	s16 range_lo[3];
	s16 range_hi[3];
};

struct akm_cmode {
	unsigned int t_us;
	u8 mode;
};

struct akm_state {
	struct i2c_client *i2c;
	struct iio_trigger *trig;
	struct regulator_bulk_data vreg[ARRAY_SIZE(akm_vregs)];
	struct delayed_work dw;
	struct akm_hal *hal;		/* Hardware Abstraction Layer */
	u8 asa[3];			/* axis sensitivity adjustment */
	unsigned int info;		/* info data to return */
	unsigned int dbg;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int enable;		/* enable status */
	unsigned int poll_delay_us;	/* requested sampling delay (us) */
	unsigned int scale_i;		/* scale index */
	unsigned int mpu_batch_flags;	/* MPU supported batch flags */
	unsigned int batch_flags;	/* batch flags */
	unsigned int batch_period_us;	/* batch period us */
	unsigned int batch_timeout_ms;	/* batch timeout ms */
	unsigned int fifo_reserve;	/* fifoReservedEventCount */
	unsigned int fifo_max;		/* fifoMaxEventCount */
	u16 i2c_addr;			/* I2C address */
	u8 dev_id;			/* device ID */
	bool shutdown;			/* shutdown active flag */
	bool suspend;			/* suspend active flag */
	bool initd;			/* set if initialized */
	bool mpu_en;			/* if device behind MPU */
	bool fifo_en;			/* MPU FIFO enable */
	bool port_en[2];		/* enable status of MPU write port */
	int port_id[2];			/* MPU port ID */
	u8 data_out;			/* write value to trigger a sample */
	s16 magn_uc[AXIS_N];		/* uncalibrated sample data */
	s16 magn[AXIS_N];		/* sample data after calibration */
	s64 ts;				/* sample data timestamp */
	signed char matrix[9];		/* device orientation on platform */
	u8 nvi_config;			/* NVI configuration */
};

struct akm_hal {
	const char *part;
	int version;
	struct akm_scale *scale;
	u8 scale_i_max;
	const char *power_ma;
	unsigned int min_delay_us;
	u8 reg_start_rd;
	u8 reg_st1;
	u8 reg_st2;
	u8 reg_cntl1;
	u8 reg_mode;
	u8 reg_reset;
	u8 reg_astc;
	u8 reg_asa;
	u8 mode_mask;
	u8 mode_self_test;
	u8 mode_rom_read;
	struct akm_cmode *cmode_tbl;
	bool irq;
	unsigned int mpu_id;
};


static void akm_err(struct akm_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

static int akm_i2c_rd(struct akm_state *st, u8 reg, u16 len, u8 *val)
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
		akm_err(st);
		return -EIO;
	}

	return 0;
}

static int akm_i2c_wr(struct akm_state *st, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;
	msg.addr = st->i2c_addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;
	if (i2c_transfer(st->i2c->adapter, &msg, 1) != 1) {
		akm_err(st);
		return -EIO;
	}

	return 0;
}

static int akm_nvi_mpu_bypass_request(struct akm_state *st)
{
	int ret = 0;
#if AKM_NVI_MPU_SUPPORT
	int i;

	if (st->mpu_en) {
		for (i = 0; i < AKM_MPU_RETRY_COUNT; i++) {
			ret = nvi_mpu_bypass_request(true);
			if ((!ret) || (ret == -EPERM))
				break;

			msleep(AKM_MPU_RETRY_DELAY_MS);
		}
		if (ret == -EPERM)
			ret = 0;
	}
#endif /* AKM_NVI_MPU_SUPPORT */
	return ret;
}

static int akm_nvi_mpu_bypass_release(struct akm_state *st)
{
	int ret = 0;

#if AKM_NVI_MPU_SUPPORT
	if (st->mpu_en)
		ret = nvi_mpu_bypass_release();
#endif /* AKM_NVI_MPU_SUPPORT */
	return ret;
}

static int akm_mode_wr(struct akm_state *st, u8 mode)
{
	int ret = 0;

#if AKM_NVI_MPU_SUPPORT
	if (st->mpu_en && !st->i2c->irq) {
		ret = nvi_mpu_data_out(st->port_id[WR], mode);
	} else {
		ret = akm_nvi_mpu_bypass_request(st);
		if (!ret) {
			if (st->i2c->irq) {
				ret = akm_i2c_wr(st, st->hal->reg_mode,
						 AKM_MODE_POWERDOWN);
				if (mode & st->hal->mode_mask) {
					udelay(AKM_HW_DELAY_US);
					ret |= akm_i2c_wr(st,
							  st->hal->reg_mode,
							  mode);
				}
			} else {
				ret = akm_i2c_wr(st, st->hal->reg_mode, mode);
			}
			akm_nvi_mpu_bypass_release(st);
		}
	}
#else /* AKM_NVI_MPU_SUPPORT */
	if (st->i2c->irq) {
		ret = akm_i2c_wr(st, st->hal->reg_mode,
				 AKM_MODE_POWERDOWN);
		if (mode & st->hal->mode_mask) {
			udelay(AKM_HW_DELAY_US);
			ret |= akm_i2c_wr(st,
					  st->hal->reg_mode,
					  mode);
		}
	} else {
		ret = akm_i2c_wr(st, st->hal->reg_mode, mode);
	}
#endif /* AKM_NVI_MPU_SUPPORT */
	if (!ret)
		st->data_out = mode;
	return ret;
}

static int akm_vreg_dis(struct akm_state *st, unsigned int i)
{
	int ret = 0;

	if (st->vreg[i].ret && (st->vreg[i].consumer != NULL)) {
		ret = regulator_disable(st->vreg[i].consumer);
		if (ret) {
			dev_err(&st->i2c->dev, "%s %s ERR\n",
				__func__, st->vreg[i].supply);
		} else {
			st->vreg[i].ret = 0;
			if (st->dbg & AKM_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s %s\n",
					 __func__, st->vreg[i].supply);
		}
	}
	return ret;
}

static int akm_vreg_dis_all(struct akm_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = ARRAY_SIZE(akm_vregs); i > 0; i--)
		ret |= akm_vreg_dis(st, (i - 1));
	return ret;
}

static int akm_vreg_en(struct akm_state *st, unsigned int i)
{
	int ret = 0;

	if ((!st->vreg[i].ret) && (st->vreg[i].consumer != NULL)) {
		ret = regulator_enable(st->vreg[i].consumer);
		if (ret) {
			dev_err(&st->i2c->dev, "%s %s ERR\n",
				__func__, st->vreg[i].supply);
		} else {
			st->vreg[i].ret = 1;
			if (st->dbg & AKM_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s %s\n",
					 __func__, st->vreg[i].supply);
			ret = 1; /* flag regulator state change */
		}
	}
	return ret;
}

static int akm_vreg_en_all(struct akm_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(akm_vregs); i++)
		ret |= akm_vreg_en(st, i);
	return ret;
}

static void akm_vreg_exit(struct akm_state *st)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(akm_vregs); i++) {
		if (st->vreg[i].consumer != NULL) {
			devm_regulator_put(st->vreg[i].consumer);
			st->vreg[i].consumer = NULL;
			if (st->dbg & AKM_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s %s\n",
					 __func__, st->vreg[i].supply);
		}
	}
}

static int akm_vreg_init(struct akm_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(akm_vregs); i++) {
		st->vreg[i].supply = akm_vregs[i];
		st->vreg[i].ret = 0;
		st->vreg[i].consumer = devm_regulator_get(&st->i2c->dev,
							  st->vreg[i].supply);
		if (IS_ERR(st->vreg[i].consumer)) {
			ret |= PTR_ERR(st->vreg[i].consumer);
			dev_err(&st->i2c->dev, "%s ret %d for %s\n",
				__func__, ret, st->vreg[i].supply);
			st->vreg[i].consumer = NULL;
		} else {
			if (st->dbg & AKM_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s %s\n",
					 __func__, st->vreg[i].supply);
		}
	}
	return ret;
}

static int akm_vreg_sts(struct akm_state *st)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(akm_vregs); i++) {
		if (st->vreg[i].consumer != NULL)
			break;
	}
	if (i < ARRAY_SIZE(akm_vregs)) {
		/* ret == number of regulators on */
		for (i = 0; i < ARRAY_SIZE(akm_vregs); i++) {
			if (st->vreg[i].ret)
				ret++;
		}
	} else {
		/* no regulator support (can assume always on) */
		ret = -EINVAL;
	}
	return ret;
}

static int akm_pm(struct akm_state *st, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = akm_vreg_en_all(st);
		if (ret > 0)
			mdelay(AKM_HW_DELAY_POR_MS);
	} else {
		if (st->i2c->irq) {
			ret = akm_vreg_sts(st);
			if ((ret < 0) || (ret == ARRAY_SIZE(akm_vregs))) {
				ret = akm_mode_wr(st, AKM_MODE_POWERDOWN);
			} else if (ret > 0) {
				akm_vreg_en_all(st);
				mdelay(AKM_HW_DELAY_POR_MS);
				ret = akm_mode_wr(st, AKM_MODE_POWERDOWN);
			}
		}
		ret |= akm_vreg_dis_all(st);
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		dev_err(&st->i2c->dev, "%s pwr=%x ERR=%d\n",
			__func__, enable, ret);
	} else {
		if (st->dbg & AKM_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s pwr=%x\n",
				 __func__, enable);
	}
	return ret;
}

static int akm_port_free(struct akm_state *st, int port)
{
	int ret = 0;

#if AKM_NVI_MPU_SUPPORT
	if (st->port_id[port] >= 0) {
		ret = nvi_mpu_port_free(st->port_id[port]);
		if (!ret)
			st->port_id[port] = -1;
	}
#endif /* AKM_NVI_MPU_SUPPORT */
	return ret;
}

static int akm_ports_free(struct akm_state *st)
{
	int ret;

	ret = akm_port_free(st, WR);
	ret |= akm_port_free(st, RD);
	return ret;
}

static void akm_pm_exit(struct akm_state *st)
{
	akm_ports_free(st);
	akm_pm(st, false);
	akm_vreg_exit(st);
}

static int akm_pm_init(struct akm_state *st)
{
	int ret;

	st->enable = 0;
	st->poll_delay_us = (AKM_POLL_DELAY_MS_DFLT * 1000);
	st->shutdown = false;
	st->initd = false;
	st->mpu_en = false;
	st->fifo_en = true;
	st->port_en[WR] = false;
	st->port_en[RD] = false;
	st->port_id[WR] = -1;
	st->port_id[RD] = -1;
	akm_vreg_init(st);
	ret = akm_pm(st, true);
	return ret;
}

static int akm_port_enable(struct akm_state *st, int port, bool enable)
{
	int ret = 0;

#if AKM_NVI_MPU_SUPPORT
	if ((enable != st->port_en[port]) && (st->port_id[port] >= 0)) {
		ret = nvi_mpu_enable(st->port_id[port], enable, st->fifo_en);
		if (!ret)
			st->port_en[port] = enable;
	}
#endif /* AKM_NVI_MPU_SUPPORT */
	return ret;
}

static int akm_ports_enable(struct akm_state *st, bool enable)
{
	int ret;

	ret = akm_port_enable(st, RD, enable);
	ret |= akm_port_enable(st, WR, enable);
	return ret;
}

static int akm_reset(struct akm_state *st)
{
	u8 val;
	unsigned int i;
	int ret = 0;

	if (st->hal->reg_reset) {
		ret = akm_nvi_mpu_bypass_request(st);
		if (!ret) {
			ret = akm_i2c_wr(st, st->hal->reg_reset,
					 AKM_BIT_SRST);
			for (i = 0; i < AKM_HW_DELAY_POR_MS; i++) {
				mdelay(1);
				ret = akm_i2c_rd(st, st->hal->reg_reset,
						 1, &val);
				if (ret)
					continue;

				if (!(val & AKM_BIT_SRST))
					break;
			}
			akm_nvi_mpu_bypass_release(st);
		}
	}

	return 0;
}

static int akm_mode(struct akm_state *st)
{
	u8 mode;
	unsigned int t_us;
	unsigned int i;
	int ret;

	mode = AKM_MODE_SINGLE;
	if (st->i2c->irq) {
		i = 0;
		while (st->hal->cmode_tbl[i].t_us) {
			mode = st->hal->cmode_tbl[i].mode;
			t_us = st->hal->cmode_tbl[i].t_us;
			if (st->poll_delay_us >= st->hal->cmode_tbl[i].t_us)
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
	}
	if (st->scale_i)
		mode |= AKM_BIT_BITM;
	ret = akm_mode_wr(st, mode);
	return ret;
}

static unsigned int akm_buf_index(unsigned int size, unsigned int *bytes)
{
	unsigned int index;

	if (!(*bytes % size))
		index = *bytes;
	else
		index = *bytes - *bytes % size + size;
	*bytes = index + size;
	return index;
}

static void akm_buf_push(struct iio_dev *indio_dev, s64 ts)
{
	struct akm_state *st = iio_priv(indio_dev);
	unsigned char buf[16];
	unsigned int n;
	unsigned int i;
	unsigned int bytes = 0;
	unsigned int axis;

	for (axis = 0; axis < AXIS_N; axis++) {
		if (iio_scan_mask_query(indio_dev, indio_dev->buffer, axis)) {
			n = sizeof(st->magn[axis]);
			i = akm_buf_index(n, &bytes);
			memcpy(&buf[i], &st->magn[axis], n);
		}
	}
	if (indio_dev->buffer->scan_timestamp) {
		n = sizeof(ts);
		i = akm_buf_index(n, &bytes);
		memcpy(&buf[i], &ts, n);
	}
	if (iio_buffer_enabled(indio_dev))
		iio_push_to_buffers(indio_dev, buf);
	if (st->dbg & AKM_DBG_SPEW_MAGNETIC_FIELD_UC)
		dev_info(&st->i2c->dev, "uc %hd %hd %hd %lld\n",
			 st->magn_uc[AXIS_X], st->magn_uc[AXIS_Y],
			 st->magn_uc[AXIS_Z], ts);
	if (st->dbg & AKM_DBG_SPEW_MAGNETIC_FIELD)
		dev_info(&st->i2c->dev, "%hd %hd %hd %lld\n",
			 st->magn[AXIS_X], st->magn[AXIS_Y],
			 st->magn[AXIS_Z], ts);
}

static void akm_calc(struct akm_state *st, u8 *data, s64 ts)
{
	s16 x;
	s16 y;
	s16 z;
	u8 st1;

	st->ts = ts;
	st1 = st->hal->reg_st1 - st->hal->reg_start_rd;
	/* data[st1 + 1] = register HXL
	 * data[st1 + 2] = register HXH
	 * data[st1 + 3] = register HYL
	 * data[st1 + 4] = register HYH
	 * data[st1 + 5] = register HZL
	 * data[st1 + 6] = register HZH
	 */
	x = (s16)le16_to_cpup((__le16 *)(&data[st1 + 1]));
	y = (s16)le16_to_cpup((__le16 *)(&data[st1 + 3]));
	z = (s16)le16_to_cpup((__le16 *)(&data[st1 + 5]));
	st->magn_uc[AXIS_X] = x;
	st->magn_uc[AXIS_Y] = y;
	st->magn_uc[AXIS_Z] = z;
	x = (((int)x * (st->asa[AXIS_X] + 128)) >> 8);
	y = (((int)y * (st->asa[AXIS_Y] + 128)) >> 8);
	z = (((int)z * (st->asa[AXIS_Z] + 128)) >> 8);
	if (!(st->dbg & AKM_DBG_AXIS_X))
		st->magn[AXIS_X] = x;
	if (!(st->dbg & AKM_DBG_AXIS_Y))
		st->magn[AXIS_Y] = y;
	if (!(st->dbg & AKM_DBG_AXIS_Z))
		st->magn[AXIS_Z] = z;
}

static int akm_read_sts(struct akm_state *st, u8 *data)
{
	u8 st1;
	u8 st2;
	int ret = 0; /* assume still processing */

	st1 = st->hal->reg_st1 - st->hal->reg_start_rd;
	st2 = st->hal->reg_st2 - st->hal->reg_start_rd;
	if (data[st2] & (AKM_BIT_HOFL | AKM_BIT_DERR)) {
		if (st->dbg & AKM_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s ERR\n", __func__);
		akm_err(st);
		ret = -1; /* error */
	} else if (data[st1] & AKM_BIT_DRDY) {
		ret = 1; /* data ready to be reported */
	}
	return ret;
}

static int akm_read(struct iio_dev *indio_dev)
{
	struct akm_state *st = iio_priv(indio_dev);
	s64 ts;
	u8 data[10];
	int ret;

	ret = akm_i2c_rd(st, st->hal->reg_start_rd, 10, data);
	if (ret)
		return ret;

	ts = iio_get_time_ns();
	ret = akm_read_sts(st, data);
	if (ret > 0) {
		akm_calc(st, data, ts);
		akm_buf_push(indio_dev, ts);
	}
	return ret;
}

#if AKM_NVI_MPU_SUPPORT
static void akm_mpu_handler(u8 *data, unsigned int len, s64 ts, void *p_val)
{
	struct iio_dev *indio_dev;
	struct akm_state *st;
	int ret;

	indio_dev = (struct iio_dev *)p_val;
	st = iio_priv(indio_dev);
	if (!ts) {
		/* no timestamp means flush done */
		akm_buf_push(indio_dev, 0);
		return;
	}

	if (st->enable) {
		ret = akm_read_sts(st, data);
		if (ret > 0) {
			akm_calc(st, data, ts);
			akm_buf_push(indio_dev, ts);
		}
	}
}
#endif /* AKM_NVI_MPU_SUPPORT */

static void akm_work(struct work_struct *ws)
{
	struct akm_state *st = container_of((struct delayed_work *)ws,
					    struct akm_state, dw);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (st->enable) {
		ret = akm_read(indio_dev);
		if (ret > 0) {
			akm_i2c_wr(st, st->hal->reg_mode, st->data_out);
		} else if (ret < 0) {
			akm_reset(st);
			akm_mode(st);
		}
		schedule_delayed_work(&st->dw,
				      usecs_to_jiffies(st->poll_delay_us));
	}
	mutex_unlock(&indio_dev->mlock);
}

static irqreturn_t akm_irq_thread(int irq, void *dev_id)
{
	struct akm_state *st = (struct akm_state *)dev_id;
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	int ret;

	ret = akm_read(indio_dev);
	if (ret < 0) {
		akm_reset(st);
		akm_mode(st);
	}
	return IRQ_HANDLED;
}

static int akm_self_test(struct akm_state *st)
{
	u8 data[10];
	u8 mode;
	int ret;
	int ret_t;

	ret_t = akm_i2c_wr(st, st->hal->reg_mode, AKM_MODE_POWERDOWN);
	udelay(AKM_HW_DELAY_US);
	if (st->hal->reg_astc) {
		ret_t |= akm_i2c_wr(st, st->hal->reg_astc, AKM_BIT_SELF);
		udelay(AKM_HW_DELAY_US);
	}
	mode = st->hal->mode_self_test;
	if (st->scale_i)
		mode |= AKM_BIT_BITM;
	ret_t |= akm_i2c_wr(st, st->hal->reg_mode, mode);
	mdelay(AKM_HW_DELAY_TSM_MS);
	ret = akm_i2c_rd(st, st->hal->reg_start_rd, 10, data);
	if (!ret) {
		ret = akm_read_sts(st, data);
		if (ret > 0) {
			akm_calc(st, data, 0);
			ret = 0;
		} else {
			ret = -EBUSY;
		}
	}
	ret_t |= ret;
	if (st->hal->reg_astc)
		akm_i2c_wr(st, st->hal->reg_astc, 0);
	if (!ret_t) {
		if ((st->magn[AXIS_X] <
			       st->hal->scale[st->scale_i].range_lo[AXIS_X]) ||
				(st->magn[AXIS_X] >
				 st->hal->scale[st->scale_i].range_hi[AXIS_X]))
			ret_t |= 1 << AXIS_X;
		if ((st->magn[AXIS_Y] <
			       st->hal->scale[st->scale_i].range_lo[AXIS_Y]) ||
				(st->magn[AXIS_Y] >
				 st->hal->scale[st->scale_i].range_hi[AXIS_Y]))
			ret_t |= 1 << AXIS_Y;
		if ((st->magn[AXIS_Z] <
			       st->hal->scale[st->scale_i].range_lo[AXIS_Z]) ||
				(st->magn[AXIS_Z] >
				 st->hal->scale[st->scale_i].range_hi[AXIS_Z]))
			ret_t |= 1 << AXIS_Z;
		if (ret_t) {
			dev_err(&st->i2c->dev, "%s ERR: out_of_range %x\n",
				__func__, ret_t);

		}
	} else {
		dev_err(&st->i2c->dev, "%s ERR: %d\n",
			__func__, ret_t);
	}
	return ret_t;
}

static int akm_delay(struct akm_state *st, unsigned int delay_us)
{
	int ret = 0;

	if (delay_us < st->hal->min_delay_us)
		delay_us = st->hal->min_delay_us;
	if (delay_us != st->poll_delay_us) {
#if AKM_NVI_MPU_SUPPORT
		if (st->port_id[RD] >= 0)
			ret = nvi_mpu_delay_us(st->port_id[RD],
					       (unsigned long)delay_us);
		if (!ret)
#endif /* AKM_NVI_MPU_SUPPORT */
			st->poll_delay_us = delay_us;
	}
	return ret;
}

static int akm_init_hw(struct akm_state *st)
{
	int ret;

	ret = akm_nvi_mpu_bypass_request(st);
	if (!ret) {
		ret = akm_i2c_wr(st, st->hal->reg_mode,
				 st->hal->mode_rom_read);
		udelay(AKM_HW_DELAY_ROM_ACCESS_US);
		ret |= akm_i2c_rd(st, st->hal->reg_asa, 3, st->asa);
		ret |= akm_i2c_wr(st, st->hal->reg_mode, AKM_MODE_POWERDOWN);
		akm_self_test(st);
		akm_nvi_mpu_bypass_release(st);
	}
	if (!ret)
		st->initd = true;
	else
		dev_err(&st->i2c->dev, "%s ERR %d\n", __func__, ret);
	return ret;
}

static int akm_dis(struct akm_state *st)
{
	int ret = 0;

	if (st->mpu_en) {
		ret = akm_ports_enable(st, false);
	} else {
		if (st->i2c->irq)
			disable_irq(st->i2c->irq);
		else
			cancel_delayed_work_sync(&st->dw);
	}
	if (!ret)
		st->enable = 0;
	return ret;
}

static int akm_disable(struct iio_dev *indio_dev)
{
	struct akm_state *st = iio_priv(indio_dev);
	int ret;

	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  AKM_SCAN_X)))
		st->dbg &= ~AKM_DBG_AXIS_X;
	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  AKM_SCAN_Y)))
		st->dbg &= ~AKM_DBG_AXIS_Y;
	if (!(iio_scan_mask_query(indio_dev, indio_dev->buffer,
				  AKM_SCAN_Z)))
		st->dbg &= ~AKM_DBG_AXIS_Z;
	ret = akm_dis(st);
	if (!ret)
		akm_pm(st, false);
	return ret;
}

static int akm_en(struct akm_state *st)
{
	int ret = 0;

	akm_pm(st, true);
	if (!st->initd)
		ret = akm_init_hw(st);
	return ret;
}

static int akm_enable(struct iio_dev *indio_dev)
{
	struct akm_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, AKM_SCAN_X) ||
	       iio_scan_mask_query(indio_dev, indio_dev->buffer, AKM_SCAN_Y) ||
	       iio_scan_mask_query(indio_dev, indio_dev->buffer, AKM_SCAN_Z)) {
		ret = akm_en(st);
		if (!ret) {
			ret = akm_mode(st);
			if (st->mpu_en)
				ret |= akm_ports_enable(st, true);
			if (ret) {
				akm_disable(indio_dev);
			} else {
				st->enable = 1;
				if (!st->mpu_en) {
					if (st->i2c->irq)
						enable_irq(st->i2c->irq);
					else
						schedule_delayed_work(&st->dw,
							      usecs_to_jiffies(
							   st->poll_delay_us));
				}
			}
		}
	}
	return ret;
}

static int akm_able(struct iio_dev *indio_dev, unsigned int en)
{
	int ret;

	if (en)
		ret = akm_enable(indio_dev);
	else
		ret = akm_disable(indio_dev);
	return ret;
}

static ssize_t akm_attr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct akm_state *st = iio_priv(indio_dev);
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
	case AKM_ATTR_ENABLE:
		msg = "ATTR_ENABLE";
		old = st->enable;
		ret = akm_able(indio_dev, new);
		break;

#if AKM_NVI_MPU_SUPPORT
	case AKM_ATTR_BATCH_FLAGS:
		msg = "ATTR_BATCH_FLAGS";
		old = st->batch_flags;
		st->batch_flags = new;
		break;

	case AKM_ATTR_BATCH_PERIOD:
		msg = "ATTR_BATCH_PERIOD";
		if (new < st->hal->min_delay_us)
			new = st->hal->min_delay_us;
		old = st->batch_period_us;
		st->batch_period_us = new;
		break;

	case AKM_ATTR_BATCH_TIMEOUT:
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
						ret = akm_delay(st,
								new * 1000);
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

	case AKM_ATTR_FLUSH:
		msg = "ATTR_FLUSH";
		if (st->mpu_batch_flags)
			ret = nvi_mpu_flush(st->port_id[RD]);
		else
			ret = -EINVAL;
		break;
#endif /* AKM_NVI_MPU_SUPPORT */

	default:
		msg = "ATTR_UNKNOWN";
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & AKM_DBG_SPEW_MSG) {
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

static ssize_t akm_attr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct akm_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	ssize_t t;
	unsigned int enable;
	int ret;

	switch (this_attr->address) {
	case AKM_ATTR_ENABLE:
		return sprintf(buf, "%x\n", st->enable);

	case AKM_ATTR_PART:
		return sprintf(buf, "%s\n", st->hal->part);

	case AKM_ATTR_VENDOR:
		return sprintf(buf, "%s\n", AKM_VENDOR);

	case AKM_ATTR_VERSION:
		return sprintf(buf, "%u\n", st->hal->version);

	case AKM_ATTR_MILLIAMP:
		return sprintf(buf, "%s\n", st->hal->power_ma);

#if AKM_NVI_MPU_SUPPORT
	case AKM_ATTR_BATCH_FLAGS:
		return sprintf(buf, "%u\n", st->batch_flags);

	case AKM_ATTR_BATCH_PERIOD:
		return sprintf(buf, "%u\n", st->batch_period_us);

	case AKM_ATTR_BATCH_TIMEOUT:
		return sprintf(buf, "%u\n", st->batch_timeout_ms);

	case AKM_ATTR_FLUSH:
		return sprintf(buf, "%u\n", st->mpu_batch_flags);

	case AKM_ATTR_FIFO_RSRV_EVNT_CNT:
		ret = nvi_mpu_fifo(st->port_id[RD],
				   &st->fifo_reserve, &st->fifo_max);
		if (ret)
			return ret;

		return sprintf(buf, "%u\n", st->fifo_reserve);

	case AKM_ATTR_FIFO_MAX_EVNT_CNT:
		ret = nvi_mpu_fifo(st->port_id[RD],
				   &st->fifo_reserve, &st->fifo_max);
		if (ret)
			return ret;

		return sprintf(buf, "%u\n", st->fifo_max);
#endif /* AKM_NVI_MPU_SUPPORT */

	case AKM_ATTR_SELF_TEST:
		mutex_lock(&indio_dev->mlock);
		enable = st->enable;
		akm_dis(st);
		akm_en(st);
		ret = akm_nvi_mpu_bypass_request(st);
		if (!ret) {
			ret = akm_self_test(st);
			akm_nvi_mpu_bypass_release(st);
		}
		if (ret < 0) {
			t = sprintf(buf, "ERR: %d\n", ret);
		} else {
			t = sprintf(buf, "%d   xyz: %hd %hd %hd   ", ret,
				    st->magn[AXIS_X],
				    st->magn[AXIS_Y],
				    st->magn[AXIS_Z]);
			t += sprintf(buf + t, "uncalibrated: %hd %hd %hd   ",
				    st->magn_uc[AXIS_X],
				    st->magn_uc[AXIS_Y],
				    st->magn_uc[AXIS_Z]);
			if (ret > 0) {
				if (ret & (1 << AXIS_X))
					t += sprintf(buf + t, "X ");
				if (ret & (1 << AXIS_Y))
					t += sprintf(buf + t, "Y ");
				if (ret & (1 << AXIS_Z))
					t += sprintf(buf + t, "Z ");
				t += sprintf(buf + t, "FAILED\n");
			} else {
				t += sprintf(buf + t, "PASS\n");
			}
		}
		akm_able(indio_dev, enable);
		mutex_unlock(&indio_dev->mlock);
		return t;

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static ssize_t akm_data_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct akm_state *st = iio_priv(dev_get_drvdata(dev));
	unsigned int info;
	int ret;

	ret = kstrtouint(buf, 10, &info);
	if ((ret) || (info >= AKM_INFO_LIMIT_MAX))
		return -EINVAL;

	st->info = info;
	switch (info) {
	case AKM_INFO_DATA:
		st->dbg = 0;
		break;

	case AKM_INFO_DBG:
		st->dbg ^= AKM_DBG_SPEW_MSG;
		break;

	case AKM_INFO_MAGNETIC_FIELD_SPEW:
		st->dbg ^= AKM_DBG_SPEW_MAGNETIC_FIELD;
		break;

	case AKM_INFO_MAGNETIC_FIELD_UC_SPEW:
		st->dbg ^= AKM_DBG_SPEW_MAGNETIC_FIELD_UC;
		break;

	default:
		break;
	}

	return count;
}

static ssize_t akm_data_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct akm_state *st = iio_priv(indio_dev);
	enum AKM_INFO info;
	ssize_t t;
	u8 data1[25];
	u8 data2[3];
	unsigned int i;
	unsigned int enable;
	int ret = 0;

	info = st->info;
	st->info = AKM_INFO_DATA;
	enable = st->enable;
	switch (info) {
	case AKM_INFO_DATA:
		t = sprintf(buf, "MAGN_XYZ: %hd %hd %hd  ",
			    st->magn[AXIS_X],
			    st->magn[AXIS_Y],
			    st->magn[AXIS_Z]);
		t += sprintf(buf + t, "MAGN_UC_XYZ: %hd %hd %hd  ",
			     st->magn_uc[AXIS_X],
			     st->magn_uc[AXIS_Y],
			     st->magn_uc[AXIS_Z]);
		t += sprintf(buf + t, "TIMESTAMP: %lld\n", st->ts);
		return t;

	case AKM_INFO_VER:
		return sprintf(buf, "version=%u\n", AKM_VERSION_DRIVER);

	case AKM_INFO_ERRS:
		i = st->errs;
		st->errs = 0;
		return sprintf(buf, "error count=%u\n", i);

	case AKM_INFO_RESET:
		akm_dis(st);
		akm_pm(st, true);
		ret = akm_reset(st);
		akm_able(indio_dev, enable);
		if (ret)
			return sprintf(buf, "reset ERR %d\n", ret);
		else
			return sprintf(buf, "reset done\n");

	case AKM_INFO_REGS:
		if (!st->initd)
			t = sprintf(buf, "calibration: NEED ENABLE\n");
		else
			t = sprintf(buf, "calibration: x=%#2x y=%#2x z=%#2x\n",
				    st->asa[AXIS_X],
				    st->asa[AXIS_Y],
				    st->asa[AXIS_Z]);
		ret = akm_nvi_mpu_bypass_request(st);
		if (!ret) {
			ret = akm_i2c_rd(st, AKM_REG_WIA,
					 st->hal->reg_st2 + 1, data1);
			ret |= akm_i2c_rd(st, st->hal->reg_cntl1, 3, data2);
			akm_nvi_mpu_bypass_release(st);
		}
		if (ret) {
			t += sprintf(buf + t, "registers: ERR %d\n", ret);
		} else {
			t += sprintf(buf + t, "registers:\n");
			for (i = 0; i <= st->hal->reg_st2; i++)
				t += sprintf(buf + t, "%#2x=%#2x\n",
					     AKM_REG_WIA + i, data1[i]);
			for (i = 0; i < 3; i++)
				t += sprintf(buf + t, "%#2x=%#2x\n",
					     st->hal->reg_cntl1 + i,
					     data2[i]);
		}
		return t;

	case AKM_INFO_DBG:
		return sprintf(buf, "debug spew=%x\n",
			       st->dbg & AKM_DBG_SPEW_MSG);

	case AKM_INFO_MAGNETIC_FIELD_SPEW:
		return sprintf(buf, "xyz_ts spew=%x\n",
			       !!(st->dbg & AKM_DBG_SPEW_MAGNETIC_FIELD));

	case AKM_INFO_MAGNETIC_FIELD_UC_SPEW:
		return sprintf(buf, "xyz_uncalibrated_ts spew=%x\n",
			       !!(st->dbg & AKM_DBG_SPEW_MAGNETIC_FIELD_UC));

	default:
		break;
	}

	return -EINVAL;
}

static ssize_t akm_matrix_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct akm_state *st = iio_priv(dev_get_drvdata(dev));
	signed char *m;

	m = st->matrix;
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		       m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}

static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		       akm_attr_show, akm_attr_store, AKM_ATTR_ENABLE);
static IIO_DEVICE_ATTR(magn_part, S_IRUGO,
		       akm_attr_show, NULL, AKM_ATTR_PART);
static IIO_DEVICE_ATTR(magn_vendor, S_IRUGO,
		       akm_attr_show, NULL, AKM_ATTR_VENDOR);
static IIO_DEVICE_ATTR(magn_version, S_IRUGO,
		       akm_attr_show, NULL, AKM_ATTR_VERSION);
static IIO_DEVICE_ATTR(magn_milliamp, S_IRUGO,
		       akm_attr_show, NULL, AKM_ATTR_MILLIAMP);
#if AKM_NVI_MPU_SUPPORT
static IIO_DEVICE_ATTR(magn_batch_flags, S_IRUGO | S_IWUSR | S_IWGRP,
		       akm_attr_show, akm_attr_store, AKM_ATTR_BATCH_FLAGS);
static IIO_DEVICE_ATTR(magn_batch_period, S_IRUGO | S_IWUSR | S_IWGRP,
		       akm_attr_show, akm_attr_store, AKM_ATTR_BATCH_PERIOD);
static IIO_DEVICE_ATTR(magn_batch_timeout, S_IRUGO | S_IWUSR | S_IWGRP,
		       akm_attr_show, akm_attr_store, AKM_ATTR_BATCH_TIMEOUT);
static IIO_DEVICE_ATTR(magn_flush, S_IRUGO | S_IWUSR | S_IWGRP,
		       akm_attr_show, akm_attr_store, AKM_ATTR_FLUSH);
static IIO_DEVICE_ATTR(magn_fifo_reserved_event_count,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       akm_attr_show, akm_attr_store,
		       AKM_ATTR_FIFO_RSRV_EVNT_CNT);
static IIO_DEVICE_ATTR(magn_fifo_max_event_count, S_IRUGO | S_IWUSR | S_IWGRP,
		       akm_attr_show, akm_attr_store,
		       AKM_ATTR_FIFO_MAX_EVNT_CNT);
#endif /* AKM_NVI_MPU_SUPPORT */
static IIO_DEVICE_ATTR(magn_self_test, S_IRUGO,
		       akm_attr_show, NULL, AKM_ATTR_SELF_TEST);
static DEVICE_ATTR(magn_matrix, S_IRUGO,
		   akm_matrix_show, NULL);
static DEVICE_ATTR(data, S_IRUGO | S_IWUSR | S_IWGRP,
		   akm_data_show, akm_data_store);

static struct attribute *akm_attrs[] = {
	&iio_dev_attr_enable.dev_attr.attr,
	&iio_dev_attr_magn_part.dev_attr.attr,
	&iio_dev_attr_magn_vendor.dev_attr.attr,
	&iio_dev_attr_magn_version.dev_attr.attr,
	&iio_dev_attr_magn_milliamp.dev_attr.attr,
#if AKM_NVI_MPU_SUPPORT
	&iio_dev_attr_magn_batch_flags.dev_attr.attr,
	&iio_dev_attr_magn_batch_period.dev_attr.attr,
	&iio_dev_attr_magn_batch_timeout.dev_attr.attr,
	&iio_dev_attr_magn_flush.dev_attr.attr,
	&iio_dev_attr_magn_fifo_reserved_event_count.dev_attr.attr,
	&iio_dev_attr_magn_fifo_max_event_count.dev_attr.attr,
#endif /* AKM_NVI_MPU_SUPPORT */
	&iio_dev_attr_magn_self_test.dev_attr.attr,
	&dev_attr_magn_matrix.attr,
	&dev_attr_data.attr,
	NULL
};

static struct attribute_group akm_attr_group = {
	.name = AKM_NAME,
	.attrs = akm_attrs
};

static int akm_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct akm_state *st = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = st->magn[chan->channel2 - IIO_MOD_X];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->enable)
			*val = st->poll_delay_us;
		else
			*val = st->hal->min_delay_us;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PEAK:
		*val = st->hal->scale[st->scale_i].max_range.ival;
		*val2 = st->hal->scale[st->scale_i].max_range.micro;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_PEAK_SCALE:
		*val = st->hal->scale[st->scale_i].resolution.ival;
		*val2 = st->hal->scale[st->scale_i].resolution.micro;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_SCALE:
		*val = st->hal->scale[st->scale_i].scale.ival;
		*val2 = st->hal->scale[st->scale_i].scale.micro;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		*val = AKM_OFFSET_IVAL;
		*val2 = AKM_OFFSET_MICRO;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}

	return ret;
}

static int akm_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask)
{
	struct akm_state *st = iio_priv(indio_dev);
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
		old = st->poll_delay_us;
		ret = akm_delay(st, (unsigned int)val);
		if (st->i2c->irq && st->enable && (val != old) && !ret)
			ret = akm_mode(st);
		if (ret < 0)
			st->poll_delay_us = old;
		break;

	case IIO_CHAN_INFO_SCALE:
		msg = "IIO_CHAN_INFO_SCALE";
		if ((val >= 0) && (val <= st->hal->scale_i_max)) {
			old = st->scale_i;
			st->scale_i = val;
			if (st->enable && (val != old)) {
				ret = akm_mode(st);
				if (ret < 0)
					st->scale_i = old;
			}
		} else {
			ret = -EINVAL;
		}
		break;

	case IIO_CHAN_INFO_RAW:
		msg = "IIO_CHAN_INFO_RAW";
		old = st->magn[chan->channel2 - IIO_MOD_X];
		st->magn[chan->channel2 - IIO_MOD_X] = val;
		st->dbg |= (AKM_DBG_AXIS_X << (chan->channel2 - IIO_MOD_X));
		break;

	default:
		msg = "IIO_CHAN_INFO_UNKNOWN";
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & AKM_DBG_SPEW_MSG) {
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

static const struct iio_info akm_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &akm_attr_group,
	.read_raw = &akm_read_raw,
	.write_raw = &akm_write_raw,
};

static const struct iio_chan_spec akm_channels[] = {
	{
		.type			= IIO_MAGN,
		.channel2		= IIO_MOD_X,
		.scan_index		= AKM_SCAN_X,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					    BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_OFFSET),
		.modified		= 1,
	},
	{
		.type			= IIO_MAGN,
		.channel2		= IIO_MOD_Y,
		.scan_index		= AKM_SCAN_Y,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					    BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_OFFSET),
		.modified		= 1,
	},
	{
		.type			= IIO_MAGN,
		.channel2		= IIO_MOD_Z,
		.scan_index		= AKM_SCAN_Z,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					    BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_OFFSET),
		.modified		= 1,
	},
	IIO_CHAN_SOFT_TIMESTAMP(AKM_SCAN_TIMESTAMP)
};

static int akm_buffer_preenable(struct iio_dev *indio_dev)
{
	struct akm_state *st = iio_priv(indio_dev);

	if (st->shutdown || st->suspend)
		return -EINVAL;

	return 0;
}

static int akm_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;

	ret = akm_enable(indio_dev);
	/* never return > 0 to IIO buffer engine */
	if (ret > 0)
		ret = 0;
	return ret;
}

static const struct iio_buffer_setup_ops akm_buffer_setup_ops = {
	/* iio_sw_buffer_preenable:
	 * Generic function for equal sized ring elements + 64 bit timestamp
	 * Assumes that any combination of channels can be enabled.
	 * Typically replaced to implement restrictions on what combinations
	 * can be captured (hardware scan modes).
	 */
	.preenable = &akm_buffer_preenable,
	/* iio_triggered_buffer_postenable:
	 * Generic function that simply attaches the pollfunc to the trigger.
	 * Replace this to mess with hardware state before we attach the
	 * trigger.
	 */
	.postenable = &akm_buffer_postenable,
	/* this driver relies on the NVS HAL to power off this device with the
	 * master enable.
	 *.predisable = N/A
	 *.postdisable = N/A
	 */
};

static const struct iio_trigger_ops akm_trigger_ops = {
	.owner = THIS_MODULE,
};

static int akm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct akm_state *st = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (st->enable)
		ret = akm_disable(indio_dev);
	st->suspend = true;
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & AKM_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int akm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct akm_state *st = iio_priv(indio_dev);

	st->suspend = false;
	if (st->dbg & AKM_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(akm_pm_ops, akm_suspend, akm_resume);

static struct akm_scale akm_scale_09911[] = {
	{
		.max_range		= {
			.ival		= 9825,
			.micro		= 0,
		},
		.resolution		= {
			.ival		= 0,
			.micro		= 600000,
		},
		.scale			= {
			.ival		= 0,
			.micro		= 600000,
		},
		.range_lo[AXIS_X]	= -30,
		.range_hi[AXIS_X]	= 30,
		.range_lo[AXIS_Y]	= -30,
		.range_hi[AXIS_Y]	= 30,
		.range_lo[AXIS_Z]	= -400,
		.range_hi[AXIS_Z]	= -50,
	},
};

static struct akm_cmode akm_cmode_09911[] = {
	{
		.t_us			= 100000,
		.mode			= 0x02,
	},
	{
		.t_us			= 50000,
		.mode			= 0x04,
	},
	{
		.t_us			= 20000,
		.mode			= 0x06,
	},
	{
		.t_us			= 10000,
		.mode			= 0x08,
	},
	{},
};

static struct akm_hal akm_hal_09911 = {
	.part				= AKM_NAME_AK09911,
	.version			= 2,
	.scale				= akm_scale_09911,
	.scale_i_max			= ARRAY_SIZE(akm_scale_09911) - 1,
	.power_ma			= "2.4",
	.min_delay_us			= 10000,
	.reg_start_rd			= 0x10,
	.reg_st1			= 0x10,
	.reg_st2			= 0x18,
	.reg_cntl1			= 0x30,
	.reg_mode			= 0x31,
	.reg_reset			= 0x32,
	.reg_astc			= 0, /* N/A */
	.reg_asa			= 0x60,
	.mode_mask			= 0x1F,
	.mode_self_test			= 0x10,
	.mode_rom_read			= 0x1F,
	.cmode_tbl			= akm_cmode_09911,
	.irq				= false,
#if AKM_NVI_MPU_SUPPORT
	.mpu_id				= COMPASS_ID_AK09911,
#endif /* AKM_NVI_MPU_SUPPORT */
};

static struct akm_scale akm_scale_8975[] = {
	{
		.max_range		= {
			.ival		= 2459,
			.micro		= 0,
		},
		.resolution		= {
			.ival		= 0,
			.micro		= 300000,
		},
		.scale			= {
			.ival		= 0,
			.micro		= 300000,
		},
		.range_lo[AXIS_X]	= -100,
		.range_hi[AXIS_X]	= 100,
		.range_lo[AXIS_Y]	= -100,
		.range_hi[AXIS_Y]	= 100,
		.range_lo[AXIS_Z]	= -1000,
		.range_hi[AXIS_Z]	= -300,
	},
};

static struct akm_hal akm_hal_8975 = {
	.part				= AKM_NAME_AK8975,
	.version			= 2,
	.scale				= akm_scale_8975,
	.scale_i_max			= ARRAY_SIZE(akm_scale_8975) - 1,
	.power_ma			= "3.0",
	.min_delay_us			= 10000,
	.reg_start_rd			= 0x01,
	.reg_st1			= 0x02,
	.reg_st2			= 0x09,
	.reg_cntl1			= 0x0A,
	.reg_mode			= 0x0A,
	.reg_reset			= 0, /* N/A */
	.reg_astc			= 0x0C,
	.reg_asa			= 0x10,
	.mode_mask			= 0x0F,
	.mode_self_test			= 0x08,
	.mode_rom_read			= 0x0F,
	.cmode_tbl			= NULL,
	.irq				= true,
#if AKM_NVI_MPU_SUPPORT
	.mpu_id				= COMPASS_ID_AK8975,
#endif /* AKM_NVI_MPU_SUPPORT */
};

static struct akm_scale akm_scale_8963[] = {
	{
		.max_range		= {
			.ival		= 9825,
			.micro		= 0,
		},
		.resolution		= {
			.ival		= 0,
			.micro		= 600000,
		},
		.scale			= {
			.ival		= 0,
			.micro		= 600000,
		},
		.range_lo[AXIS_X]	= -50,
		.range_hi[AXIS_X]	= 50,
		.range_lo[AXIS_Y]	= -50,
		.range_hi[AXIS_Y]	= 50,
		.range_lo[AXIS_Z]	= -800,
		.range_hi[AXIS_Z]	= -200,
	},
	{
		.max_range		= {
			.ival		= 9825,
			.micro		= 0,
		},
		.resolution		= {
			.ival		= 0,
			.micro		= 150000,
		},
		.scale			= {
			.ival		= 0,
			.micro		= 150000,
		},
		.range_lo[AXIS_X]	= -200,
		.range_hi[AXIS_X]	= 200,
		.range_lo[AXIS_Y]	= -200,
		.range_hi[AXIS_Y]	= 200,
		.range_lo[AXIS_Z]	= -3200,
		.range_hi[AXIS_Z]	= -800,
	},
};

static struct akm_cmode akm_cmode_8963[] = {
	{
		.t_us			= 125000,
		.mode			= 0x02,
	},
	{
		.t_us			= 10000,
		.mode			= 0x04,
	},
	{},
};

static struct akm_hal akm_hal_8963 = {
	.part				= AKM_NAME_AK8963,
	.version			= 2,
	.scale				= akm_scale_8963,
	.scale_i_max			= ARRAY_SIZE(akm_scale_8963) - 1,
	.power_ma			= "2.8",
	.min_delay_us			= 10000,
	.reg_start_rd			= 0x01,
	.reg_st1			= 0x02,
	.reg_st2			= 0x09,
	.reg_cntl1			= 0x0A,
	.reg_mode			= 0x0A,
	.reg_reset			= 0x0B,
	.reg_astc			= 0x0C,
	.reg_asa			= 0x10,
	.mode_mask			= 0x0F,
	.mode_self_test			= 0x08,
	.mode_rom_read			= 0x0F,
	.cmode_tbl			= akm_cmode_8963,
	.irq				= true,
#if AKM_NVI_MPU_SUPPORT
	.mpu_id				= COMPASS_ID_AK8963,
#endif /* AKM_NVI_MPU_SUPPORT */
};

static int akm_id_hal(struct akm_state *st, u8 dev_id)
{
	int ret = 0;

	switch (dev_id) {
	case AKM_DEVID_AK09911:
		st->hal = &akm_hal_09911;
		break;

	case AKM_DEVID_AK8975:
		st->hal = &akm_hal_8975;
		break;

	case AKM_DEVID_AK8963:
		st->hal = &akm_hal_8963;
		break;

	default:
		st->hal = &akm_hal_8975;
		ret = -ENODEV;
	}
	st->scale_i = st->hal->scale_i_max;
	return ret;
}

static int akm_id_compare(struct akm_state *st, const char *name)
{
	u8 wia;
	u8 val;
	int ret;
	int ret_t;

	ret_t = akm_nvi_mpu_bypass_request(st);
	if (!ret_t) {
		ret_t = akm_i2c_rd(st, AKM_REG_WIA2, 1, &wia);
		if (ret_t)
			wia = 0;
		akm_id_hal(st, wia);
		if (wia != AKM_DEVID_AK09911) {
			/* we can autodetect AK8963 with BITM */
			ret = akm_i2c_wr(st, st->hal->reg_mode,
					 AKM_BIT_BITM);
			if (!ret) {
				ret = akm_i2c_rd(st, st->hal->reg_st2,
						 1, &val);
				if (!ret) {
					if (val & AKM_BIT_BITM)
						wia = AKM_DEVID_AK8963;
					else
						wia = AKM_DEVID_AK8975;
					akm_id_hal(st, wia);
				} else {
					ret_t |= ret;
					wia = 0;
				}
			} else {
				ret_t |= ret;
			}
		}
		akm_nvi_mpu_bypass_release(st);
		if ((!st->dev_id) && (!wia)) {
			dev_err(&st->i2c->dev, "%s ERR: %s HW ID FAIL\n",
				__func__, name);
			ret = -ENODEV;
		} else if ((!st->dev_id) && wia) {
			st->dev_id = wia;
			dev_dbg(&st->i2c->dev, "%s %s using ID %x\n",
				__func__, name, st->dev_id);
		} else if (st->dev_id && (!wia)) {
			dev_err(&st->i2c->dev, "%s WARN: %s HW ID FAIL\n",
				__func__, name);
		} else if (st->dev_id != wia) {
			dev_err(&st->i2c->dev, "%s WARN: %s != HW ID %x\n",
				__func__, name, wia);
			st->dev_id = wia;
		} else {
			dev_dbg(&st->i2c->dev, "%s %s == HW ID %x\n",
				__func__, name, wia);
		}
	}
	return ret_t;
}

static int akm_id_dev(struct iio_dev *indio_dev, const char *name)
{
	struct akm_state *st = iio_priv(indio_dev);
#if AKM_NVI_MPU_SUPPORT
	struct nvi_mpu_port nmp;
	u8 config_boot;
#endif /* AKM_NVI_MPU_SUPPORT */
	u8 val = 0;
	int ret;

	if (st->i2c->irq < 0)
		st->i2c->irq = 0;
	if (!strcmp(name, AKM_NAME_AK8963))
		st->dev_id = AKM_DEVID_AK8963;
	else if (!strcmp(name, AKM_NAME_AK8975))
		st->dev_id = AKM_DEVID_AK8975;
	else if (!strcmp(name, AKM_NAME_AK09911))
		st->dev_id = AKM_DEVID_AK09911;
#if AKM_NVI_MPU_SUPPORT
	config_boot = st->nvi_config & NVI_CONFIG_BOOT_MASK;
	if (config_boot == NVI_CONFIG_BOOT_AUTO) {
		nmp.addr = st->i2c_addr | 0x80;
		nmp.reg = AKM_REG_WIA;
		nmp.ctrl = 1;
		ret = nvi_mpu_dev_valid(&nmp, &val);
		dev_info(&st->i2c->dev, "%s AUTO ID=%x ret=%d\n",
			 __func__, val, ret);
		/* see mpu_iio.h for possible return values */
		if ((ret == -EAGAIN) || (ret == -EBUSY))
			return -EAGAIN;

		if ((val == AKM_WIA_ID) || ((ret == -EIO) && st->dev_id))
			config_boot = NVI_CONFIG_BOOT_MPU;
	}
	if (config_boot == NVI_CONFIG_BOOT_MPU) {
		st->mpu_en = true;
		if (st->dev_id)
			ret = akm_id_hal(st, st->dev_id);
		else
			ret = akm_id_compare(st, name);
		if (!ret) {
			nmp.addr = st->i2c_addr | 0x80;
			nmp.reg = st->hal->reg_start_rd;
			nmp.ctrl = 10; /* MPU FIFO can't handle odd size */
			nmp.data_out = 0;
			nmp.delay_ms = 0;
			nmp.delay_us = st->poll_delay_us;
			if ((st->hal->cmode_tbl != NULL) && st->i2c->irq)
				nmp.shutdown_bypass = true;
			else
				nmp.shutdown_bypass = false;
			nmp.handler = &akm_mpu_handler;
			nmp.ext_driver = (void *)indio_dev;
			nmp.type = SECONDARY_SLAVE_TYPE_COMPASS;
			nmp.id = st->hal->mpu_id;
			nmp.rate_scale = 10;
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
				nmp.reg = st->hal->reg_mode;
				nmp.ctrl = 1;
				nmp.data_out = AKM_MODE_SINGLE;
				nmp.delay_ms = AKM_HW_DELAY_TSM_MS;
				nmp.delay_us = 0;
				nmp.shutdown_bypass = false;
				nmp.handler = NULL;
				nmp.ext_driver = NULL;
				nmp.type = SECONDARY_SLAVE_TYPE_COMPASS;
				ret = nvi_mpu_port_alloc(&nmp);
				dev_dbg(&st->i2c->dev, "%s MPU port/ret=%d\n",
					__func__, ret);
				if (ret < 0) {
					akm_ports_free(st);
				} else {
					st->port_id[WR] = ret;
					ret = nvi_mpu_batch(st->port_id[RD],
							    0, 0, 0);
					if (ret > 0)
						st->mpu_batch_flags = ret;
					ret = 0;
				}
			}
		}
		return ret;
	}
#endif /* AKM_NVI_MPU_SUPPORT */
	/* NVI_CONFIG_BOOT_HOST */
	st->mpu_en = false;
	if (st->dev_id) {
		ret = akm_id_hal(st, st->dev_id);
	} else {
		ret = akm_i2c_rd(st, AKM_REG_WIA, 1, &val);
		dev_info(&st->i2c->dev, "%s Host read ID=%x ret=%d\n",
			__func__, val, ret);
		if ((!ret) && (val == AKM_WIA_ID))
			ret = akm_id_compare(st, name);
	}
	if (st->i2c->irq && !ret) {
		if ((st->hal->cmode_tbl == NULL) || !st->hal->irq) {
			disable_irq(st->i2c->irq);
			st->i2c->irq = 0;
		}
	}
	return ret;
}

static int akm_id_i2c(struct iio_dev *indio_dev,
		      const struct i2c_device_id *id)
{
	struct akm_state *st = iio_priv(indio_dev);
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(akm_i2c_addrs); i++) {
		if (st->i2c->addr == akm_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(akm_i2c_addrs)) {
		st->i2c_addr = st->i2c->addr;
		ret = akm_id_dev(indio_dev, id->name);
	} else {
		for (i = 0; i < ARRAY_SIZE(akm_i2c_addrs); i++) {
			st->i2c_addr = akm_i2c_addrs[i];
			ret = akm_id_dev(indio_dev, AKM_NAME);
			if ((ret == -EAGAIN) || (!ret))
				break;
		}
	}
	return ret;
}

static void akm_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct akm_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	st->shutdown = true;
	if (st->enable)
		akm_disable(indio_dev);
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & AKM_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int akm_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct akm_state *st = iio_priv(indio_dev);

	if (st != NULL) {
		akm_shutdown(client);
		if (indio_dev->dev.devt)
			iio_device_unregister(indio_dev);
		if (st->trig != NULL) {
			if (client->irq && !st->mpu_en)
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
		akm_pm_exit(st);
		iio_device_free(indio_dev);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int akm_of_dt(struct i2c_client *client, struct akm_state *st)
{
	struct device_node *dn = client->dev.of_node;
	char const *pchar;
	int len;
	u8 cfg;

	pchar = of_get_property(dn, "matrix", &len);
	if (pchar && len == sizeof(st->matrix)) {
		memcpy(&st->matrix, pchar, len);
	} else { /* obsolete */
		pchar = of_get_property(dn, "orientation", &len);
		if (pchar && len == sizeof(st->matrix))
			memcpy(&st->matrix, pchar, len);
	}
	if (!(of_property_read_string(dn, "nvi_config", &pchar))) {
		for (cfg = 0; cfg < ARRAY_SIZE(akm_configs); cfg++) {
			if (!strcasecmp(pchar, akm_configs[cfg])) {
				st->nvi_config = cfg;
				break;
			}
		}
	}

	return 0;
}

static int akm_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct akm_state *st;
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
		ret = akm_of_dt(client, st);
		if (ret)
			goto akm_probe_err;
	}

	akm_pm_init(st);
	ret = akm_id_i2c(indio_dev, id);
	if (ret == -EAGAIN)
		goto akm_probe_again;
	else if (ret)
		goto akm_probe_err;

	akm_init_hw(st);
	akm_pm(st, false);
	if (!st->mpu_en)
		INIT_DELAYED_WORK(&st->dw, akm_work);
	indio_dev->buffer = iio_kfifo_allocate(indio_dev);
	if (!indio_dev->buffer) {
		dev_err(&client->dev, "%s iio_kfifo_allocate ERR\n", __func__);
		ret = -ENOMEM;
		goto akm_probe_err;
	}

	indio_dev->buffer->scan_timestamp = true;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = akm_channels;
	indio_dev->num_channels = ARRAY_SIZE(akm_channels);
	indio_dev->name = AKM_NAME;
	indio_dev->info = &akm_iio_info;
	indio_dev->setup_ops = &akm_buffer_setup_ops;
	ret = iio_buffer_register(indio_dev, indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto akm_probe_err;

	if ((st->i2c->irq > 0) && !st->mpu_en) {
		ret = request_threaded_irq(st->i2c->irq, NULL, akm_irq_thread,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   AKM_NAME, st);
		if (ret) {
			dev_err(&client->dev, "%s req_threaded_irq ERR %d\n",
				__func__, ret);
			ret = -ENOMEM;
			goto akm_probe_err;
		}
	}

	st->trig = iio_trigger_alloc("%s-dev%d",
				     indio_dev->name, indio_dev->id);
	if (st->trig == NULL) {
		dev_err(&client->dev, "%s iio_allocate_trigger ERR\n",
			__func__);
		ret = -ENOMEM;
		goto akm_probe_err;
	}

	st->trig->dev.parent = &st->i2c->dev;
	st->trig->ops = &akm_trigger_ops;
	ret = iio_trigger_register(st->trig);
	if (ret) {
		dev_err(&client->dev, "%s iio_trigger_register ERR\n",
			__func__);
		ret = -ENOMEM;
		goto akm_probe_err;
	}

	indio_dev->trig = st->trig;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_device_register(indio_dev);
	if (ret)
		goto akm_probe_err;

	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

akm_probe_err:
	dev_err(&client->dev, "%s ERR %d\n", __func__, ret);
akm_probe_again:
	akm_remove(client);
	return ret;
}

static const struct i2c_device_id akm_i2c_device_id[] = {
	{ AKM_NAME, 0 },
	{ AKM_NAME_AK8963, 0 },
	{ AKM_NAME_AK8975, 0 },
	{ AKM_NAME_AK09911, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, akm_i2c_device_id);

static const struct of_device_id akm_of_match[] = {
	{ .compatible = "ak,ak89xx", },
	{ .compatible = "ak,ak8963", },
	{ .compatible = "ak,ak8975", },
	{ .compatible = "ak,ak09911", },
	{}
};

MODULE_DEVICE_TABLE(of, akm_of_match);

static struct i2c_driver akm_driver = {
	.class				= I2C_CLASS_HWMON,
	.probe				= akm_probe,
	.remove				= akm_remove,
	.shutdown			= akm_shutdown,
	.driver				= {
		.name			= AKM_NAME,
		.owner			= THIS_MODULE,
		.of_match_table		= of_match_ptr(akm_of_match),
		.pm			= &akm_pm_ops,
	},
	.id_table			= akm_i2c_device_id,
};

static int __init akm_init(void)
{
	return i2c_add_driver(&akm_driver);
}

static void __exit akm_exit(void)
{
	i2c_del_driver(&akm_driver);
}

late_initcall(akm_init);
module_exit(akm_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AKM driver");
MODULE_AUTHOR("NVIDIA Corporation");

