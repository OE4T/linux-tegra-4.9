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

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>
#include <linux/mpu_iio.h>

#include "nvi.h"

#define NVI_DRIVER_VERSION		(102)
#define NVI_NAME			"mpu6xxx"
#define NVI_NAME_MPU6050		"MPU6050"
#define NVI_NAME_MPU6500		"MPU6500"
#define NVI_NAME_MPU6515		"MPU6515"
#define NVI_NAME_ICM20628		"ICM20628"
#define NVI_VENDOR			"Invensense"

/* _buf_push expects this scan order */
#define NVI_SCAN_ACCEL_X		(0)
#define NVI_SCAN_ACCEL_Y		(1)
#define NVI_SCAN_ACCEL_Z		(2)
#define NVI_SCAN_ANGLVEL_X		(3)
#define NVI_SCAN_ANGLVEL_Y		(4)
#define NVI_SCAN_ANGLVEL_Z		(5)
#define NVI_SCAN_TEMP			(6)
#define NVI_SCAN_TIMESTAMP		(7)

enum NVI_ATTR {
	NVI_ATTR_ENABLE,
	NVI_ATTR_PART,
	NVI_ATTR_VENDOR,
	NVI_ATTR_FLUSH,
	NVI_ATTR_FIFO_RSRV_EVNT_CNT,
	NVI_ATTR_FIFO_MAX_EVNT_CNT,
	NVI_ATTR_ACCEL_PART,
	NVI_ATTR_ACCEL_VERSION,
	NVI_ATTR_ACCEL_MILLIAMP,
	NVI_ATTR_ANGLVEL_PART,
	NVI_ATTR_ANGLVEL_VERSION,
	NVI_ATTR_ANGLVEL_MILLIAMP,
	NVI_ATTR_TEMP_PART,
	NVI_ATTR_TEMP_VERSION,
	NVI_ATTR_TEMP_MILLIAMP,
	INV_ATTR_SELF_TEST,
};

enum NVI_INFO {
	NVI_INFO_DATA = 0,
	NVI_INFO_VER,
	NVI_INFO_ERRS,
	NVI_INFO_RESET,
	NVI_INFO_REGS,
	NVI_INFO_DBG,
	NVI_INFO_AUX_SPEW,
	NVI_INFO_ACCEL_UC_SPEW,
	NVI_INFO_ACCEL_SPEW,
	NVI_INFO_ANGLVEL_SPEW,
	NVI_INFO_TEMP_SPEW,
	NVI_INFO_FIFO_SPEW,
	NVI_INFO_FIFO_BUF,
	NVI_INFO_FIFO_IRQ,
	NVI_INFO_LIMIT_MAX,
};

/* regulator names in order of powering on */
static char *nvi_vregs[] = {
	"vdd",
	"vlogic",
};

static unsigned short nvi_i2c_addrs[] = {
	0x68,
	0x69,
};

static struct iio_dev *indio_dev_local;


static int nvi_nb_vreg(struct nvi_state *st,
		       unsigned long event, unsigned int i);

static int nvi_nb_vreg_vdd(struct notifier_block *nb,
			   unsigned long event, void *ignored)
{
	struct nvi_state *st = container_of(nb, struct nvi_state, nb_vreg[0]);

	return nvi_nb_vreg(st, event, 0);
}

static int nvi_nb_vreg_vlogic(struct notifier_block *nb,
			      unsigned long event, void *ignored)
{
	struct nvi_state *st = container_of(nb, struct nvi_state, nb_vreg[1]);

	return nvi_nb_vreg(st, event, 1);
}

static int (* const nvi_nb_vreg_pf[])(struct notifier_block *nb,
				      unsigned long event, void *ignored) = {
	nvi_nb_vreg_vdd,
	nvi_nb_vreg_vlogic,
};

s64 nvi_get_time_ns(struct nvi_state *st)
{
	struct timespec ts;

	if (st->iio_ts_en)
		return iio_get_time_ns();

	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

static void nvi_err(struct nvi_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

int nvi_i2c_write(struct nvi_state *st, u16 addr, u16 len, u8 *buf)
{
	struct i2c_msg msg;

	msg.addr = addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = buf;
	if (i2c_transfer(st->i2c->adapter, &msg, 1) != 1) {
		nvi_err(st);
		return -EIO;
	}

	return 0;
}

int nvi_i2c_wr(struct nvi_state *st, u8 reg, u8 val)
{
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;
	return nvi_i2c_write(st, st->i2c_addr, sizeof(buf), buf);
}

/* Register REG_BANK_SEL */
static int nvi_wr_reg_bank_sel(struct nvi_state *st, u8 reg_bank_sel)
{
	int ret = 0;

	if (st->hal->part >= ICM20628) {
		reg_bank_sel <<= 4;
		if (reg_bank_sel != st->rc.reg_bank_sel) {
			ret = nvi_i2c_wr(st, st->hal->reg->reg_bank_sel.reg,
					 reg_bank_sel);
			if (ret) {
				dev_err(&st->i2c->dev, "%s: %x->%x ERR=%d\n",
					__func__, st->rc.reg_bank_sel,
					reg_bank_sel, ret);
			} else {
				if (st->dbg & NVI_DBG_SPEW_MSG)
					dev_info(&st->i2c->dev, "%s: %x->%x\n",
						 __func__, st->rc.reg_bank_sel,
						 reg_bank_sel);
				st->rc.reg_bank_sel = reg_bank_sel;
			}
		}
	}
	return ret;
}

int nvi_i2c_read(struct nvi_state *st, u16 addr, u8 reg, u16 len, u8 *buf)
{
	struct i2c_msg msg[2];

	msg[0].addr = addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;
	if (i2c_transfer(st->i2c->adapter, msg, 2) != 2) {
		nvi_err(st);
		return -EIO;
	}

	return 0;
}

int nvi_i2c_rd(struct nvi_state *st, u8 bank, u8 reg, u16 len, u8 *buf)
{
	int ret;

	ret = nvi_wr_reg_bank_sel(st, bank);
	if (!ret)
		ret = nvi_i2c_read(st, st->i2c_addr, reg, len, buf);
	return ret;
}

/* Register ACCEL OFFSET */
static int nvi_rd_accel_offset(struct nvi_state *st)
{
	u8 buf[2];
	unsigned int i;
	int ret;

	for (i = 0; i < AXIS_N; i++) {
		ret = nvi_i2c_rd(st, st->hal->reg->a_offset_h[i].bank,
				 st->hal->reg->a_offset_h[i].reg, 2, buf);
		if (!ret)
			st->rc.accel_offset[i] = be16_to_cpup((__be16 *)buf);
	}
	return ret;
}

/* Register ACCEL OFFSET */
int nvi_wr_accel_offset(struct nvi_state *st, unsigned int axis, u16 offset)
{
	u8 buf[3];
	u16 offset_le;
	int ret = 0;

	if (axis >= AXIS_N)
		return -EINVAL;

	if ((offset != st->rc.accel_offset[axis]) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st,
					  st->hal->reg->a_offset_h[axis].bank);
		if (ret)
			return ret;

		buf[0] = st->hal->reg->a_offset_h[axis].reg;
		offset_le = cpu_to_le16(offset);
		buf[1] = offset_le >> 8;
		buf[2] = offset_le & 0xFF;
		ret = nvi_i2c_write(st, st->i2c_addr, sizeof(buf), buf);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->a_offset_h[axis].reg,
				st->rc.accel_offset[axis], offset, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__, st->hal->reg->
					 a_offset_h[axis].reg,
					 st->rc.accel_offset[axis], offset);
			st->rc.accel_offset[axis] = offset;
		}
	}
	return ret;
}

/* Register GYRO OFFSET */
static int nvi_rd_gyro_offset(struct nvi_state *st)
{
	u8 buf[AXIS_N * 2];
	unsigned int i;
	int ret;

	ret = nvi_i2c_rd(st, st->hal->reg->xg_offset_h.bank,
			 st->hal->reg->xg_offset_h.reg, 6, buf);
	if (!ret) {
		for (i = 0; i < AXIS_N; i++)
			st->rc.gyro_offset[i] = be16_to_cpup((__be16 *)
							     (&buf[i << 1]));
	}
	return ret;
}

/* Register GYRO OFFSET */
int nvi_wr_gyro_offset(struct nvi_state *st, unsigned int axis, u16 offset)
{
	u8 buf[3];
	u16 offset_le;
	int ret = 0;

	if (axis >= AXIS_N)
		return -EINVAL;

	if ((offset != st->rc.gyro_offset[axis]) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->xg_offset_h.bank);
		if (ret)
			return ret;

		buf[0] = st->hal->reg->xg_offset_h.reg + (axis << 1);
		offset_le = cpu_to_le16(offset);
		buf[1] = offset_le >> 8;
		buf[2] = offset_le & 0xFF;
		ret = nvi_i2c_write(st, st->i2c_addr,
				    sizeof(buf), buf);
		if (ret) {
			dev_err(&st->i2c->dev, "%s: %x->%x ERR=%d\n",
				__func__, st->rc.gyro_offset[axis],
				offset, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s: %x->%x\n",
					 __func__,
					 st->rc.gyro_offset[axis],
					 offset);
			st->rc.gyro_offset[axis] = offset;
		}
	}
	return ret;
}

/* Register ACCEL_SMPLRT_DIV */
/* Register GYRO_SMPLRT_DIV */
int nvi_wr_smplrt_div(struct nvi_state *st, unsigned int dev, u16 val)
{
	u8 buf[3];
	u16 len;
	u16 val_be;
	int ret = 0;

	dev = st->hal->smplrt[dev]->dev;
	if ((val != st->rc.smplrt_div[dev]) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st,
					  st->hal->reg->smplrt_div[dev].bank);
		if (ret)
			return ret;

		len = st->hal->bit->smplrt_div_n[dev] / 8;
		if (st->hal->bit->smplrt_div_n[dev] % 8)
			len++;
		val_be = cpu_to_be16(val);
		val_be >>= ((sizeof(val_be) - len) * 8);
		len++;
		buf[0] = st->hal->reg->smplrt_div[dev].reg;
		buf[1] = val_be & 0xFF;
		buf[2] = val_be >> 8;
		ret = nvi_i2c_write(st, st->i2c_addr, len, buf);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->smplrt_div[dev].reg,
				st->rc.smplrt_div[dev], val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					__func__,
					 st->hal->reg->smplrt_div[dev].reg,
					 st->rc.smplrt_div[dev], val);
			st->rc.smplrt_div[dev] = val;
		}
	}
	return ret;
}

/* Register GYRO_CONFIG1 */
static int nvi_wr_gyro_config1(struct nvi_state *st, u8 val)
{
	int ret = 0;

	val |= st->hal->reg->gyro_config1.dflt;
	if ((val != st->rc.gyro_config1) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->gyro_config1.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->gyro_config1.reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->gyro_config1.reg,
				st->rc.gyro_config1, val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__,
					 st->hal->reg->gyro_config1.reg,
					 st->rc.gyro_config1, val);
			st->rc.gyro_config1 = val;
		}
	}
	return ret;
}

/* Register GYRO_CONFIG2 */
int nvi_wr_gyro_config(struct nvi_state *st, u8 test, u8 fsr, u8 lpf)
{
	u8 val = 0;
	int ret = 0;

	if (st->hal->part < MPU6500) {
		if (lpf < 8)
			ret = nvi_wr_gyro_config1(st, lpf);
		if (fsr > 3)
			val |= st->rc.gyro_config2 & 0x18;
		else
			val |= fsr << 3;
	} else if (st->hal->part < ICM20628) {
		if (lpf < 8)
			ret = nvi_wr_gyro_config1(st, lpf);
		if (test > 7)
			val |= st->rc.gyro_config2 & 0xE0;
		else
			val |= test << 5;
		if (fsr > 3)
			val |= st->rc.gyro_config2 & 0x18;
		else
			val |= fsr << 3;
	} else {
		if (lpf > 7)
			val |= st->rc.gyro_config1 & 0x38;
		else
			val |= lpf << 3;
		if (fsr > 3)
			val |= st->rc.gyro_config1 & 0x06;
		else
			val |= fsr << 1;
		ret = nvi_wr_gyro_config1(st, val);
		if (test > 7)
			return ret;

		val = test;
	}
	if (((val != st->rc.gyro_config2) || st->rc_dis) && !ret) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->gyro_config2.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->gyro_config2.reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->gyro_config2.reg,
				st->rc.gyro_config2, val, ret);

		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__,
					 st->hal->reg->gyro_config2.reg,
					 st->rc.gyro_config2, val);
			st->rc.gyro_config2 = val;
		}
	}
	return ret;
}

/* Register ACCEL_CONFIG2 */
static int nvi_wr_accel_config2(struct nvi_state *st, u8 val)
{
	int ret = 0;

	if ((val != st->rc.accel_config2) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st,
					  st->hal->reg->accel_config2.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->accel_config2.reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->accel_config2.reg,
				st->rc.accel_config2, val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__,
					 st->hal->reg->accel_config2.reg,
					 st->rc.accel_config2, val);
			st->rc.accel_config2 = val;
		}
	}
	return ret;
}

/* Register ACCEL_CONFIG */
int nvi_wr_accel_config(struct nvi_state *st, u8 test, u8 fsr, u8 lpf)
{
	u8 val = 0;
	int ret = 0;

	if (st->hal->part < MPU6500) {
		if (test > 7)
			val |= st->rc.accel_config & 0xE0;
		else
			val |= test << 5;
		if (fsr > 3)
			val |= st->rc.accel_config & 0x18;
		else
			val |= fsr << 3;
		if (lpf > 7)
			val |= st->rc.accel_config & 0x07;
		else
			val |= lpf;
	} else if (st->hal->part < ICM20628) {
		if (lpf < 8)
			ret = nvi_wr_accel_config2(st, BIT_FIFO_SIZE_1K | lpf);
		if (test > 7)
			val |= st->rc.accel_config & 0xE0;
		else
			val |= test << 5;
		if (fsr > 3)
			val |= st->rc.accel_config & 0x18;
		else
			val |= fsr << 3;
	} else {
		if (test < 8)
			ret = nvi_wr_accel_config2(st, test << 2);
		if (lpf > 7)
			val |= st->rc.accel_config & 0x38;
		else
			val |= lpf << 3;
		if (fsr > 3)
			val |= st->rc.accel_config & 0x06;
		else
			val |= fsr << 1;
	}
	if (((val != st->rc.accel_config) || st->rc_dis) && !ret) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->accel_config.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->accel_config.reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->accel_config.reg,
				st->rc.accel_config, val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__,
					 st->hal->reg->accel_config.reg,
					 st->rc.accel_config, val);
			st->rc.accel_config = val;
		}
	}
	return ret;
}

/* Register LP_CONFIG */
int nvi_wr_lp_config(struct nvi_state *st, u8 val)
{
	int ret = 0;

	val |= st->hal->reg->lp_config.dflt;
	if ((val != st->rc.lp_config) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->lp_config.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->lp_config.reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->lp_config.reg,
				st->rc.lp_config, val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__, st->hal->reg->lp_config.reg,
					 st->rc.lp_config, val);
			st->rc.lp_config = val;
		}
	}
	return ret;
}

/* Register FIFO_EN */
int nvi_wr_fifo_en(struct nvi_state *st, u16 fifo_en)
{
	u8 buf[3];
	u16 len = 1 + (st->hal->bit->bit_fifo_en_max / 8);
	u16 fifo_en_le = cpu_to_le16(fifo_en);
	int ret = 0;

	if ((fifo_en != st->rc.fifo_en) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->fifo_en.bank);
		if (ret)
			return ret;

		buf[0] = st->hal->reg->fifo_en.reg;
		buf[1] = st->hal->reg->fifo_en.dflt & 0xFF;
		buf[1] |= fifo_en_le & 0xFF;
		buf[2] = st->hal->reg->fifo_en.dflt >> 8;
		buf[2] |= fifo_en_le >> 8;
		ret = nvi_i2c_write(st, st->i2c_addr, len, buf);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->fifo_en.reg,
				st->rc.fifo_en, fifo_en, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__, st->hal->reg->fifo_en.reg,
					 st->rc.fifo_en, fifo_en);
			st->rc.fifo_en = fifo_en;
		}
	}
	return ret;
}

/* Register I2C_MST_ODR_CONFIG */
static int nvi_wr_i2c_mst_odr_config(struct nvi_state *st, u8 val)
{
	int ret = 0;

	if (st->hal->reg->i2c_mst_odr_config.bank == 0xFF)
		return 0;

	if ((val != st->rc.i2c_mst_odr_config) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st,
					st->hal->reg->i2c_mst_odr_config.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->i2c_mst_odr_config.reg,
				 val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->i2c_mst_odr_config.reg,
				st->rc.i2c_mst_odr_config, val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__, st->hal->reg->
					 i2c_mst_odr_config.reg,
					 st->rc.i2c_mst_odr_config, val);
			st->rc.i2c_mst_odr_config = val;
		}
	}
	return ret;
}

/* Register I2C_MST_DELAY_CTRL */
static int nvi_wr_i2c_mst_delay_ctrl(struct nvi_state *st, u8 val)
{
	int ret = 0;

	if ((val != st->rc.i2c_mst_delay_ctrl) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st,
					st->hal->reg->i2c_mst_delay_ctrl.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->i2c_mst_delay_ctrl.reg,
				 val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->i2c_mst_delay_ctrl.reg,
				st->rc.i2c_mst_delay_ctrl, val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__,
					 st->hal->reg->i2c_mst_delay_ctrl.reg,
					 st->rc.i2c_mst_delay_ctrl, val);
			st->rc.i2c_mst_delay_ctrl = val;
		}
	}
	return ret;
}

/* Register I2C_SLV0_ADDR */
/* Register I2C_SLV1_ADDR */
/* Register I2C_SLV2_ADDR */
/* Register I2C_SLV3_ADDR */
/* Register I2C_SLV4_ADDR */
static int nvi_wr_i2c_slv_addr(struct nvi_state *st, int port, u8 val)
{
	u8 reg;
	int ret = 0;

	if ((val != st->rc.i2c_slv_addr[port]) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st,
					  st->hal->reg->i2c_slv0_addr.bank);
		if (ret)
			return ret;

		if (st->hal->part >= ICM20628)
			reg = (st->hal->reg->i2c_slv0_addr.reg + (port << 2));
		else
			reg = (st->hal->reg->i2c_slv0_addr.reg + (port * 3));
		ret = nvi_i2c_wr(st, reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s[%d]:%x=%x->%x ERR=%d\n",
				__func__, port, reg, st->rc.i2c_slv_addr[port],
				val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s[%d]:%x=%x->%x\n",
					 __func__, port, reg,
					 st->rc.i2c_slv_addr[port], val);
			st->rc.i2c_slv_addr[port] = val;
		}
	}
	return ret;
}

/* Register I2C_SLV0_REG */
/* Register I2C_SLV1_REG */
/* Register I2C_SLV2_REG */
/* Register I2C_SLV3_REG */
/* Register I2C_SLV4_REG */
static int nvi_wr_i2c_slv_reg(struct nvi_state *st, int port, u8 val)
{
	u8 reg;
	int ret = 0;

	if ((val != st->rc.i2c_slv_reg[port]) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->i2c_slv0_reg.bank);
		if (ret)
			return ret;

		if (st->hal->part >= ICM20628)
			reg = (st->hal->reg->i2c_slv0_reg.reg + (port << 2));
		else
			reg = (st->hal->reg->i2c_slv0_reg.reg + (port * 3));
		ret = nvi_i2c_wr(st, reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s[%d]:%x=%x->%x ERR=%d\n",
				__func__, port, reg,
				st->rc.i2c_slv_reg[port], val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s[%d]:%x=%x->%x\n",
					 __func__, port, reg,
					 st->rc.i2c_slv_reg[port], val);
			st->rc.i2c_slv_reg[port] = val;
		}
	}
	return ret;
}

/* Register I2C_SLV0_CTRL */
/* Register I2C_SLV1_CTRL */
/* Register I2C_SLV2_CTRL */
/* Register I2C_SLV3_CTRL */
static int nvi_wr_i2c_slv_ctrl(struct nvi_state *st, int port, u8 val)
{
	u8 reg;
	int ret = 0;

	if ((val != st->rc.i2c_slv_ctrl[port]) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st,
					  st->hal->reg->i2c_slv0_ctrl.bank);
		if (ret)
			return ret;

		if (st->hal->part >= ICM20628)
			reg = (st->hal->reg->i2c_slv0_ctrl.reg + (port << 2));
		else
			reg = (st->hal->reg->i2c_slv0_ctrl.reg + (port * 3));
		ret = nvi_i2c_wr(st, reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s[%d]:%x=%x->%x ERR=%d\n",
				__func__, port, reg,
				st->rc.i2c_slv_ctrl[port], val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s[%d]:%x=%x->%x\n",
					 __func__, port, reg,
					 st->rc.i2c_slv_ctrl[port], val);
			st->rc.i2c_slv_ctrl[port] = val;
			ret = 1; /* flag change made */
		}
	}
	return ret;
}

/* Register I2C_SLV0_DO */
/* Register I2C_SLV1_DO */
/* Register I2C_SLV2_DO */
/* Register I2C_SLV3_DO */
/* Register I2C_SLV4_DO */
static int nvi_wr_i2c_slv_do(struct nvi_state *st, int port, u8 val)
{
	u8 reg;
	int ret = 0;

	if ((val != st->rc.i2c_slv_do[port]) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->i2c_slv0_do.bank);
		if (ret)
			return ret;

		if (st->hal->part >= ICM20628) {
			reg = (st->hal->reg->i2c_slv0_do.reg + (port << 2));
		} else {
			if (port == AUX_PORT_IO)
				reg = st->hal->reg->i2c_slv4_do.reg;
			else
				reg = st->hal->reg->i2c_slv0_do.reg + port;
		}
		ret = nvi_i2c_wr(st, reg, val);
		if (ret)
			dev_err(&st->i2c->dev, "%s[%d]:%x=%x->%x ERR=%d\n",
				__func__, port, reg,
				st->rc.i2c_slv_do[port], val, ret);
		else
			st->rc.i2c_slv_do[port] = val;
	}
	return ret;
}

/* Register I2C_SLV4_CTRL */
static int nvi_wr_i2c_slv4_ctrl(struct nvi_state *st, bool slv4_en)
{
	u8 val;
	int ret = 0;

	val = st->aux.delay_hw;
	val |= (st->aux.port[AUX_PORT_IO].nmp.ctrl & BIT_I2C_SLV_REG_DIS);
	if (slv4_en)
		val |= BIT_SLV_EN;
	if ((val != st->rc.i2c_slv_ctrl[AUX_PORT_IO]) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st,
					  st->hal->reg->i2c_slv4_ctrl.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->i2c_slv4_ctrl.reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->i2c_slv4_ctrl.reg,
				st->rc.i2c_slv_ctrl[AUX_PORT_IO], val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__,
					 st->hal->reg->i2c_slv4_ctrl.reg,
					st->rc.i2c_slv_ctrl[AUX_PORT_IO], val);
			st->rc.i2c_slv_ctrl[AUX_PORT_IO] = val;
			ret = 1; /* flag change made */
		}
	}
	return ret;
}

/* Register INT_PIN_CFG */
static int nvi_wr_int_pin_cfg(struct nvi_state *st, u8 val)
{
	int ret = 0;

	if ((val != st->rc.int_pin_cfg) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->int_pin_cfg.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->int_pin_cfg.reg, val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->int_pin_cfg.reg,
				st->rc.int_pin_cfg, val, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__,
					 st->hal->reg->int_pin_cfg.reg,
					 st->rc.int_pin_cfg, val);
			st->rc.int_pin_cfg = val;
		}
	}
	return ret;
}

/* Register INT_ENABLE */
static int nvi_wr_int_enable(struct nvi_state *st, u32 int_enable)
{
	u8 buf[5];
	u16 len = 1 + (st->hal->bit->bit_int_enable_max / 8);
	u32 int_enable_le = cpu_to_le32(int_enable);
	int ret = 0;

	if ((int_enable != st->rc.int_enable) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->int_enable.bank);
		if (ret)
			return ret;

		buf[0] = st->hal->reg->int_enable.reg;
		buf[1] = int_enable_le & 0xFF;
		buf[2] = int_enable_le >> 8;
		buf[3] = int_enable_le >> 16;
		buf[4] = int_enable_le >> 24;
		ret = nvi_i2c_write(st, st->i2c_addr, len, buf);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->int_enable.reg,
				st->rc.int_enable, int_enable, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					__func__, st->hal->reg->int_enable.reg,
					 st->rc.int_enable, int_enable);
			st->rc.int_enable = int_enable;
			if (int_enable && st->irq_dis) {
				enable_irq(st->i2c->irq);
				st->irq_dis = false;
			}
		}
	}
	return ret;
}

int nvi_int_able(struct nvi_state *st, bool enable)
{
	u32 int_enable = 0;
	int ret;

	if (enable) {
		if (st->master_enable & (1 << DEV_DMP))
			int_enable |= 1 << st->hal->bit->dmp_int_en;
		else if (st->master_enable &
			   (DEV_MPU_MASK | (1 << DEV_AUX)))
			int_enable |= 1 << st->hal->bit->raw_data_0_rdy_en;
	}
	ret = nvi_wr_int_enable(st, int_enable);
	return ret;
}

/* Register USER_CTRL */
static int nvi_wr_user_ctrl_rc(struct nvi_state *st, u8 user_ctrl)
{
	int ret = 0;

	if ((user_ctrl != st->rc.user_ctrl) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->user_ctrl.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->user_ctrl.reg, user_ctrl);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->user_ctrl.reg,
				st->rc.user_ctrl, user_ctrl, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__, st->hal->reg->user_ctrl.reg,
					 st->rc.user_ctrl, user_ctrl);
			st->rc.user_ctrl = user_ctrl;
		}
	}
	return ret;
}

int nvi_user_ctrl_en(struct nvi_state *st, bool fifo_enable, bool i2c_enable)
{
	u16 val = 0;
	u16 fifo_sample_size = 0;
	int i;
	int ret;

	st->fifo_sample_size = 0;
	if (fifo_enable && !(st->master_enable & (1 << DEV_DMP))) {
		if (st->enable[DEV_ACCEL]) {
			val |= 1 << st->hal->bit->accel_fifo_en;
			fifo_sample_size += 6;
		}
		if (st->enable[DEV_TEMP] && st->chip_config.temp_fifo_en) {
			val |= 1 << st->hal->bit->temp_fifo_en;
			fifo_sample_size += 2;
		}
		if (st->enable[DEV_ANGLVEL] & (1 << AXIS_X)) {
			val |= 1 << st->hal->bit->gyro_x_fifo_en;
			fifo_sample_size += 2;
		}
		if (st->enable[DEV_ANGLVEL] & (1 << AXIS_Y)) {
			val |= 1 << st->hal->bit->gyro_y_fifo_en;
			fifo_sample_size += 2;
		}
		if (st->enable[DEV_ANGLVEL] & (1 << AXIS_Z)) {
			val |= 1 << st->hal->bit->gyro_z_fifo_en;
			fifo_sample_size += 2;
		}
		for (i = 0; i < AUX_PORT_IO; i++) {
			if (st->aux.port[i].fifo_en &&
				  (st->aux.port[i].nmp.addr & BIT_I2C_READ) &&
				      (st->rc.i2c_slv_ctrl[i] & BIT_SLV_EN)) {
				val |= 1 << st->hal->bit->slv_fifo_en[i];
				fifo_sample_size += st->aux.port[i].nmp.ctrl &
						    BITS_I2C_SLV_CTRL_LEN;
			}
		}
	}
	st->fifo_sample_size = fifo_sample_size;
	ret = nvi_wr_fifo_en(st, val);
	if (val && !ret)
		val = BIT_FIFO_EN;
	else
		val = 0;
	if (i2c_enable && (st->master_enable & (1 << DEV_AUX)))
		val |= BIT_I2C_MST_EN;
	if (st->master_enable & (1 << DEV_DMP))
		val |= BIT_DMP_EN;
	ret |= nvi_wr_user_ctrl_rc(st, val);
	return ret;
}

/* Register USER_CTRL */
int nvi_wr_user_ctrl(struct nvi_state *st, u8 user_ctrl)
{
	bool fifo_enable = true;
	bool i2c_enable = true;
	int i;
	int ret;
	int ret_t = 0;

	if (!(user_ctrl & BITS_USER_CTRL_RST))
		return nvi_wr_user_ctrl_rc(st, user_ctrl);

	if (user_ctrl & BIT_I2C_MST_RST)
		i2c_enable = false;
	if (user_ctrl & BIT_SIG_COND_RST)
		user_ctrl = BITS_USER_CTRL_RST;
	if (user_ctrl & BIT_DMP_RST)
		user_ctrl |= BIT_FIFO_RST;
	if (user_ctrl & BIT_FIFO_RST) {
		st->flush = true;
		fifo_enable = false;
		/* must make sure FIFO is off or IRQ storm will occur */
		nvi_user_ctrl_en(st, fifo_enable, i2c_enable);
		if (st->hal->part >= ICM20628) {
			ret_t = nvi_wr_reg_bank_sel(st, REG_FIFO_RST_BANK);
			if (!ret_t) {
				ret_t =  nvi_i2c_wr(st, REG_FIFO_RST, 0x1F);
				ret_t |=  nvi_i2c_wr(st, REG_FIFO_RST, 0x1E);
			}
			if (user_ctrl == BIT_FIFO_RST)
				/* then done */
				return ret_t;
		}
	}

	nvi_user_ctrl_en(st, fifo_enable, i2c_enable);
	ret_t |= nvi_wr_reg_bank_sel(st, st->hal->reg->user_ctrl.bank);
	if (!ret_t) {
		ret_t =  nvi_i2c_wr(st, st->hal->reg->user_ctrl.reg,
				    user_ctrl);
		if (ret_t) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->user_ctrl.reg,
				st->rc.user_ctrl, user_ctrl, ret_t);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					 __func__, st->hal->reg->user_ctrl.reg,
					 st->rc.user_ctrl, user_ctrl);
			for (i = 0; i < POWER_UP_TIME; i++) {
				user_ctrl = -1;
				ret = nvi_i2c_rd(st,
						 st->hal->reg->user_ctrl.bank,
						 st->hal->reg->user_ctrl.reg,
						 1, &user_ctrl);
				if (!(user_ctrl & BITS_USER_CTRL_RST))
					break;

				mdelay(1);
			}
			ret_t |= ret;
			st->rc.user_ctrl = user_ctrl;
		}
	}
	return ret_t;
}

/* Register PWR_MGMT_1 */
static int nvi_wr_pwr_mgmt_1_war(struct nvi_state *st)
{
	u8 val;
	int i;
	int ret;

	ret = nvi_wr_reg_bank_sel(st, st->hal->reg->pwr_mgmt_1.bank);
	if (!ret) {
		for (i = 0; i < (POWER_UP_TIME / REG_UP_TIME); i++) {
			ret = nvi_i2c_wr(st, st->hal->reg->pwr_mgmt_1.reg,
					 st->hal->reg->pwr_mgmt_1.dflt);
			mdelay(REG_UP_TIME);
			val = -1;
			ret = nvi_i2c_rd(st, st->hal->reg->pwr_mgmt_1.bank,
					st->hal->reg->pwr_mgmt_1.reg, 1, &val);
			if ((!ret) && (val == st->hal->reg->pwr_mgmt_1.dflt))
				break;
		}
		st->rc.pwr_mgmt_1 = val;
	}
	return ret;
}

/* Register PWR_MGMT_1 */
int nvi_wr_pwr_mgmt_1(struct nvi_state *st, u8 pwr_mgmt_1)
{
	unsigned int i;
	int ret = 0;

	pwr_mgmt_1 |= st->hal->reg->pwr_mgmt_1.dflt;
	if ((pwr_mgmt_1 != st->rc.pwr_mgmt_1) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->pwr_mgmt_1.bank);
		if (ret)
			return ret;

		if (pwr_mgmt_1 & BIT_H_RESET) {
			nvi_wr_user_ctrl(st, BITS_USER_CTRL_RST);
			ret = nvi_i2c_wr(st, st->hal->reg->pwr_mgmt_1.reg,
					 BIT_H_RESET);
		} else {
			ret = nvi_i2c_wr(st, st->hal->reg->pwr_mgmt_1.reg,
					 pwr_mgmt_1);
		}
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->pwr_mgmt_1.reg,
				st->rc.pwr_mgmt_1, pwr_mgmt_1, ret);
		} else {
			if (pwr_mgmt_1 & BIT_H_RESET) {
				memset(&st->rc, 0, sizeof(struct nvi_rc));
				if (st->hal->por2rc)
					st->hal->por2rc(st);
				for (i = 0; i < DEV_N_AUX; i++)
					st->smplrt_delay_us[i] = 0;
				for (i = 0; i < (POWER_UP_TIME / REG_UP_TIME);
									 i++) {
					mdelay(REG_UP_TIME);
					pwr_mgmt_1 = -1;
					ret = nvi_i2c_rd(st,
						 st->hal->reg->pwr_mgmt_1.bank,
						  st->hal->reg->pwr_mgmt_1.reg,
							 1, &pwr_mgmt_1);
					if ((!ret) &&
						 (!(pwr_mgmt_1 & BIT_H_RESET)))
						break;
				}
				nvi_rd_accel_offset(st);
				nvi_rd_gyro_offset(st);
			}
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					__func__, st->hal->reg->pwr_mgmt_1.reg,
					 st->rc.pwr_mgmt_1, pwr_mgmt_1);
			st->rc.pwr_mgmt_1 = pwr_mgmt_1;
		}
	}
	return ret;
}

/* Register PWR_MGMT_2 */
static int nvi_wr_pwr_mgmt_2(struct nvi_state *st, u8 pwr_mgmt_2)
{
	int ret = 0;

	pwr_mgmt_2 |= st->hal->reg->pwr_mgmt_2.dflt;
	if ((pwr_mgmt_2 != st->rc.pwr_mgmt_2) || st->rc_dis) {
		ret = nvi_wr_reg_bank_sel(st, st->hal->reg->pwr_mgmt_2.bank);
		if (ret)
			return ret;

		ret = nvi_i2c_wr(st, st->hal->reg->pwr_mgmt_2.reg, pwr_mgmt_2);
		if (ret) {
			dev_err(&st->i2c->dev, "%s:%x=%x->%x ERR=%d\n",
				__func__, st->hal->reg->pwr_mgmt_2.reg,
				st->rc.pwr_mgmt_2, pwr_mgmt_2, ret);
		} else {
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev, "%s:%x=%x->%x\n",
					__func__, st->hal->reg->pwr_mgmt_2.reg,
					 st->rc.pwr_mgmt_2, pwr_mgmt_2);
			st->rc.pwr_mgmt_2 = pwr_mgmt_2;
		}
	}
	return ret;
}

static int nvi_nb_vreg(struct nvi_state *st,
		       unsigned long event, unsigned int i)
{
	if (event & REGULATOR_EVENT_POST_ENABLE)
		st->vreg_en_ts[i] = nvi_get_time_ns(st);
	else if (event & (REGULATOR_EVENT_DISABLE |
			  REGULATOR_EVENT_FORCE_DISABLE))
		st->vreg_en_ts[i] = 0;
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s %s event=0x%x ts=%lld\n",
			 __func__, st->vreg[i].supply, (unsigned int)event,
			 st->vreg_en_ts[i]);
	return NOTIFY_OK;
}

int nvi_pm_wr(struct nvi_state *st, u8 pwr_mgmt_1, u8 pwr_mgmt_2, u8 lp)
{
	s64 por_ns;
	bool rc_dis;
	unsigned int delay_ms;
	unsigned int i;
	int ret;
	int ret_t = 0;

	ret = nvs_vregs_enable(&st->i2c->dev, st->vreg, ARRAY_SIZE(nvi_vregs));
	if (ret) {
		rc_dis = st->rc_dis;
		st->rc_dis = true;
		delay_ms = 0;
		for (i = 0; i < ARRAY_SIZE(nvi_vregs); i++) {
			por_ns = nvi_get_time_ns(st) - st->vreg_en_ts[i];
			if ((por_ns < 0) || (!st->vreg_en_ts[i])) {
				delay_ms = (POR_MS * 1000000);
				break;
			}

			if (por_ns < (POR_MS * 1000000)) {
				por_ns = (POR_MS * 1000000) - por_ns;
				if (por_ns > delay_ms)
					delay_ms = (unsigned int)por_ns;
			}
		}
		delay_ms /= 1000000;
		if (st->dbg & NVI_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s %ums delay\n",
				 __func__, delay_ms);
		if (delay_ms)
			msleep(delay_ms);
		ret_t |= nvi_wr_pwr_mgmt_1_war(st);
		ret_t |= nvi_wr_pwr_mgmt_1(st, BIT_H_RESET);
		ret_t |= nvi_wr_pwr_mgmt_1_war(st);
		st->rc_dis = rc_dis;
	} else {
		ret_t |= nvi_wr_pwr_mgmt_1_war(st);
	}
	if (st->hal->part < MPU6500) {
		pwr_mgmt_2 |= lp << 6;
		ret = nvi_wr_pwr_mgmt_2(st, pwr_mgmt_2);
		if (ret)
			ret_t |= ret;
		else
			st->rc.lp_config = lp;
		ret_t |= nvi_wr_pwr_mgmt_1(st, pwr_mgmt_1);
	} else if (st->hal->part < ICM20628) {
		if (pwr_mgmt_1 & BIT_CYCLE) {
			ret_t |= nvi_wr_lp_config(st, lp);
			ret_t |= nvi_wr_accel_config2(st, BIT_FIFO_SIZE_1K |
						      BIT_ACCEL_FCHOCIE_B);
		}
		ret_t |= nvi_wr_pwr_mgmt_2(st, pwr_mgmt_2);
		ret_t |= nvi_wr_pwr_mgmt_1(st, pwr_mgmt_1);
		if (!(pwr_mgmt_1 & BIT_CYCLE))
			ret_t |= nvi_wr_accel_config2(st, BIT_FIFO_SIZE_1K);
	} else { /* ICM20628 */
		ret_t |= nvi_wr_lp_config(st, lp);
		ret_t |= nvi_wr_pwr_mgmt_2(st, pwr_mgmt_2);
		ret_t |= nvi_wr_pwr_mgmt_1(st, pwr_mgmt_1);
	}
	return ret_t;
}

static int nvi_reset(struct nvi_state *st,
		     bool reset_fifo, bool reset_i2c);
static int nvi_aux_bypass_enable(struct nvi_state *st, bool enable);

/**
 * @param st
 * @param pm_req: call with one of the following:
 *      NVI_PM_OFF_FORCE = force off state
 *      NVI_PM_ON = minimum power for device access
 *      NVI_PM_ON_FULL = power for anglvel
 *      NVI_PM_AUTO = automatically sets power for configuration
 *      Typical use is to set needed power for configuration and
 *      then call with NVI_PM_AUTO when done.
 *      All other NVI_PM_ levels are handled automatically and
 *      are for internal use.
 * @return int: returns 0 for success or error code
 */
int nvi_pm(struct nvi_state *st, int pm_req)
{
	bool irq;
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
	u8 lp;
	int i;
	int pm;
	int ret = 0;

	lp = st->rc.lp_config;
	if (pm_req == NVI_PM_AUTO) {
		pwr_mgmt_2 = 0;
		if (!(st->enable[DEV_ACCEL] & (1 << AXIS_X)))
			pwr_mgmt_2 |= BIT_STBY_XA;
		if (!(st->enable[DEV_ACCEL] & (1 << AXIS_Y)))
			pwr_mgmt_2 |= BIT_STBY_YA;
		if (!(st->enable[DEV_ACCEL] & (1 << AXIS_Z)))
			pwr_mgmt_2 |= BIT_STBY_ZA;
		if (!(st->enable[DEV_ANGLVEL] & (1 << AXIS_X)))
			pwr_mgmt_2 |= BIT_STBY_XG;
		if (!(st->enable[DEV_ANGLVEL] & (1 << AXIS_Y)))
			pwr_mgmt_2 |= BIT_STBY_YG;
		if (!(st->enable[DEV_ANGLVEL] & (1 << AXIS_Z)))
			pwr_mgmt_2 |= BIT_STBY_ZG;
		if (st->master_enable & DEV_PM_ON_FULL) {
			pm = NVI_PM_ON_FULL;
		} else if (st->master_enable & DEV_PM_ON) {
			pm = NVI_PM_ON;
		} else if ((st->master_enable & DEV_PM_LPA) == DEV_PM_LPA) {
			if (st->delay_us[DEV_ACCEL] >=
						st->chip_config.lpa_delay_us) {
				for (lp = 0; lp < st->hal->lpa_tbl_n; lp++) {
					if (st->delay_us[DEV_ACCEL] >=
							  st->hal->lpa_tbl[lp])
						break;
				}
				pm = NVI_PM_ON_CYCLE;
			} else {
				pm = NVI_PM_ON;
			}
		} else if (st->master_enable & (1 << DEV_ACCEL)) {
			pm = NVI_PM_ON;
		} else if ((st->master_enable & DEV_PM_STDBY) ||
							 st->aux.bypass_lock) {
			pm = NVI_PM_STDBY;
		} else {
			pm = NVI_PM_OFF;
		}
	} else {
		pwr_mgmt_2 = st->rc.pwr_mgmt_2;
		if ((pm_req > NVI_PM_STDBY) && (pm_req < st->pm))
			pm = st->pm;
		else
			pm = pm_req;
	}
	if (pm == NVI_PM_OFF) {
		for (i = 0; i < AUX_PORT_IO; i++) {
			if (st->aux.port[i].nmp.shutdown_bypass) {
				nvi_aux_bypass_enable(st, true);
				pm = NVI_PM_STDBY;
				break;
			}
		}
		if (st->master_enable & EN_FW)
			pm = NVI_PM_STDBY;
	}

	switch (pm) {
	case NVI_PM_OFF_FORCE:
	case NVI_PM_OFF:
		pm = NVI_PM_OFF;
	case NVI_PM_STDBY:
		pwr_mgmt_2 = (BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY);
		pwr_mgmt_1 = BIT_SLEEP;
		break;

	case NVI_PM_ON_CYCLE:
		pwr_mgmt_1 = BIT_CYCLE;
		break;

	case NVI_PM_ON:
		pwr_mgmt_1 = INV_CLK_INTERNAL;
		break;

	case NVI_PM_ON_FULL:
		pwr_mgmt_1 = INV_CLK_PLL;
		/* anglvel must be turned on before going to PLL clock */
		pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
		break;

	default:
		dev_err(&st->i2c->dev, "%s %d=>%d ERR=EINVAL\n",
			__func__, st->pm, pm);
		return -EINVAL;
	}

	if ((pm != st->pm) || (lp != st->rc.lp_config) ||
					   (pwr_mgmt_1 != st->rc.pwr_mgmt_1) ||
					    (pwr_mgmt_2 != (st->rc.pwr_mgmt_2 &
				  (BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY)))) {
		nvi_int_able(st, false);
		st->push_ts = 0;
		if (pm == NVI_PM_OFF) {
			switch (st->pm) {
			case NVI_PM_STDBY:
			case NVI_PM_OFF_FORCE:
			case NVI_PM_OFF:
			case NVI_PM_ERR:
				break;

			default:
				/* disables aux before turning off */
				nvi_reset(st, true, false);
				break;
			}
		}
		if ((!(st->rc.pwr_mgmt_1 & (BIT_SLEEP | BIT_CYCLE))) &&
			     (pm < NVI_PM_ON) && (st->pm > NVI_PM_ON_CYCLE)) {
			/* tasks that need access before low power state */
			if (pm_req == NVI_PM_AUTO)
				/* turn off FIFO and I2C */
				nvi_user_ctrl_en(st, false, false);
		}
		if (pm == NVI_PM_OFF) {
			if (st->pm > NVI_PM_OFF) {
				ret |= nvi_wr_pwr_mgmt_1_war(st);
				ret |= nvi_wr_pwr_mgmt_1(st, BIT_H_RESET);
			}
			ret |= nvi_pm_wr(st, pwr_mgmt_1, pwr_mgmt_2, lp);
			ret |= nvs_vregs_disable(&st->i2c->dev, st->vreg,
						 ARRAY_SIZE(nvi_vregs));
		} else {
			ret |= nvi_pm_wr(st, pwr_mgmt_1, pwr_mgmt_2, lp);
			if (pm > NVI_PM_STDBY)
				mdelay(REG_UP_TIME);
		}
		if (ret < 0) {
			dev_err(&st->i2c->dev, "%s %d=>%d ERR=%d\n",
				__func__, st->pm, pm, ret);
			pm = NVI_PM_ERR;
		}
		if (st->dbg & NVI_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s %d=>%d PM2=%x LPA=%x\n",
				 __func__, st->pm, pm, pwr_mgmt_2, lp);
		st->pm = pm;
		if (ret > 0)
			ret = 0;
	}
	if (pm_req == NVI_PM_AUTO) {
		if (pm > NVI_PM_STDBY)
			irq = true;
		else
			irq = false;
		if (pm > NVI_PM_ON_CYCLE)
			nvi_user_ctrl_en(st, true, true);
		if ((pm == NVI_PM_ON_FULL) && (!st->push_ts))
			st->push_ts = nvi_get_time_ns(st) +
					   st->chip_config.gyro_start_delay_ns;
	} else {
		/* interrupts are disabled until NVI_PM_AUTO */
		irq = false;
	}
	nvi_int_able(st, irq);
	return ret;
}

static void nvi_pm_exit(struct nvi_state *st)
{
	if (st->hal)
		nvi_pm(st, NVI_PM_OFF_FORCE);
	nvs_vregs_exit(&st->i2c->dev, st->vreg, ARRAY_SIZE(nvi_vregs));
}

static int nvi_pm_init(struct nvi_state *st)
{
	int ret;

	ret = nvs_vregs_init(&st->i2c->dev,
			     st->vreg, ARRAY_SIZE(nvi_vregs), nvi_vregs);
	st->pm = NVI_PM_ERR;
	return ret;
}

static int nvi_aux_delay(struct nvi_state *st)
{
	u8 val;
	unsigned int delay_us;
	unsigned int delay_new;
	int i;
	int j;
	int ret = 0;

	/* determine valid delays by ports enabled */
	delay_new = 0;
	for (i = 0; i < AUX_PORT_MAX; i++) {
		if (st->rc.i2c_slv_ctrl[i] & BIT_SLV_EN) {
			if (delay_new < st->aux.port[i].nmp.delay_ms)
				delay_new = st->aux.port[i].nmp.delay_ms;
		}
	}
	delay_new *= 1000;
	/* fastest delays for AUX smplrt source */
	delay_us = -1;
	for (i = 0; i < st->hal->smplrt[DEV_AUX]->dev_delays_n; i++) {
		j = st->hal->smplrt[DEV_AUX]->dev_delays[i];
		if (st->enable[j] && st->delay_us[j]) {
			if (st->delay_us[j] < delay_us)
				delay_us = st->delay_us[j];
		}
	}
	if (delay_us == -1)
		delay_us = st->hal->smplrt[DEV_AUX]->delay_us_dflt;
	if (delay_us < st->hal->smplrt[DEV_AUX]->delay_us_min)
		delay_us = st->hal->smplrt[DEV_AUX]->delay_us_min;
	if (delay_us > st->hal->smplrt[DEV_AUX]->delay_us_max)
		delay_us = st->hal->smplrt[DEV_AUX]->delay_us_max;
	st->smplrt_delay_us[DEV_AUX] = delay_us;

	if (delay_new % delay_us) {
		delay_new /= delay_us;
	} else {
		delay_new /= delay_us;
		if (delay_new)
			delay_new--;
	}
	st->aux.delay_hw = delay_new;
	ret = nvi_wr_i2c_slv4_ctrl(st, (bool)
			      (st->rc.i2c_slv_ctrl[AUX_PORT_IO] & BIT_SLV_EN));
	if (ret > 0)
		ret = 0;
	/* HW port delay enable */
	val = BIT_DELAY_ES_SHADOW;
	for (i = 0; i < AUX_PORT_MAX; i++) {
		if (st->aux.port[i].nmp.delay_ms)
			val |= (1 << i);
	}
	ret |= nvi_wr_i2c_mst_delay_ctrl(st, val);

	if (st->hal->part >= ICM20628)
		/* FIXME */
		ret |= nvi_wr_i2c_mst_odr_config(st, 4);
	return ret;
}

static int nvi_dev_delay(struct nvi_state *st, unsigned int dev)
{
	unsigned int delay_us;
	unsigned int delay_us_old;
	unsigned int fs_hz;
	u8 lpf = 0;
	u16 smplrt_div;
	int i;
	int j;
	int ret;
	int ret_t = 0;

	/* find the fastest polling of all the enabled devices */
	delay_us = -1;
	for (i = 0; i < st->hal->smplrt[dev]->dev_delays_n; i++) {
		j = st->hal->smplrt[dev]->dev_delays[i];
		if (st->enable[j] && st->delay_us[j]) {
			if (st->delay_us[j] < delay_us)
				delay_us = st->delay_us[j];
		}
	}
	if (delay_us == -1)
		delay_us = st->hal->smplrt[dev]->delay_us_dflt;
	if (delay_us < st->hal->smplrt[dev]->delay_us_min)
		delay_us = st->hal->smplrt[dev]->delay_us_min;
	if (delay_us > st->hal->smplrt[dev]->delay_us_max)
		delay_us = st->hal->smplrt[dev]->delay_us_max;
	delay_us_old = st->smplrt_delay_us[dev];
	st->smplrt_delay_us[dev] = delay_us;
	/* calculate smplrt_div */
	fs_hz = st->hal->smplrt[dev]->base_hz;
	smplrt_div = st->smplrt_delay_us[dev] / fs_hz - 1;
	/* calculate LPF */
	if (st->hal->smplrt[dev]->lpf_us_tbl_n) {
		delay_us <<= 1;
		for (lpf = 0; lpf < st->hal->smplrt[dev]->lpf_us_tbl_n; lpf++) {
			if (delay_us < st->hal->smplrt[dev]->lpf_us_tbl[lpf])
				break;
		}
	}

	if (st->dbg)
		dev_info(&st->i2c->dev, "%s dev=%u delay=%u\n",
			 __func__, dev, st->smplrt_delay_us[dev]);
	if (st->smplrt_delay_us[dev] < delay_us_old) {
		/* go faster */
		nvi_aux_delay(st);
		ret = st->hal->smplrt[dev]->lpf_wr(st, -1, -1, lpf);
		if (ret < 0)
			ret_t |= ret;
		ret_t |= nvi_wr_smplrt_div(st, dev, smplrt_div);
	} else {
		/* go slower */
		ret_t |= nvi_wr_smplrt_div(st, dev, smplrt_div);
		ret = st->hal->smplrt[dev]->lpf_wr(st, -1, -1, lpf);
		nvi_aux_delay(st);
		if (ret < 0)
			ret_t |= ret;
	}
	return ret_t;
}

static void nvi_en(struct iio_dev *indio_dev)
{
	struct nvi_state *st = iio_priv(indio_dev);
	unsigned int i;

	for (i = 0; i < DEV_N; i++)
		st->enable[i] = 0;
	st->master_enable &= ~DEV_MPU_MASK;
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer,
				NVI_SCAN_ACCEL_X)) {
		st->master_enable |= (1 << DEV_ACCEL);
		st->enable[DEV_ACCEL] |= (1 << AXIS_X);
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer,
				NVI_SCAN_ACCEL_Y)) {
		st->master_enable |= (1 << DEV_ACCEL);
		st->enable[DEV_ACCEL] |= (1 << AXIS_Y);
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer,
				NVI_SCAN_ACCEL_Z)) {
		st->master_enable |= (1 << DEV_ACCEL);
		st->enable[DEV_ACCEL] |= (1 << AXIS_Z);
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer,
				NVI_SCAN_ANGLVEL_X)) {
		st->master_enable |= (1 << DEV_ANGLVEL);
		st->enable[DEV_ANGLVEL] |= (1 << AXIS_X);
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer,
				NVI_SCAN_ANGLVEL_Y)) {
		st->master_enable |= (1 << DEV_ANGLVEL);
		st->enable[DEV_ANGLVEL] |= (1 << AXIS_Y);
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer,
				NVI_SCAN_ANGLVEL_Z)) {
		st->master_enable |= (1 << DEV_ANGLVEL);
		st->enable[DEV_ANGLVEL] |= (1 << AXIS_Z);
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, NVI_SCAN_TEMP)) {
		st->master_enable |= (1 << DEV_TEMP);
		st->enable[DEV_TEMP] |= DEV_TEMP_EN;
	}
}

int nvi_enable(struct iio_dev *indio_dev)
{
	struct nvi_state *st = iio_priv(indio_dev);
	int i;
	int ret;
	int ret_t;

	nvi_en(indio_dev);
	if (st->master_enable & (1 << DEV_ANGLVEL))
		ret_t = nvi_pm(st, NVI_PM_ON_FULL);
	else if (st->master_enable & (DEV_MPU_MASK | (1 << DEV_AUX)))
		ret_t = nvi_pm(st, NVI_PM_ON);
	else
		return nvi_pm(st, NVI_PM_AUTO);

	if (st->master_enable & (1 << DEV_ACCEL)) {
		for (i = 0; i < AXIS_N; i++)
			ret_t |= nvi_wr_accel_offset(st, i,
						(u16)(st->rom_accel_offset[i] +
					    (st->input_accel_offset[i] << 1)));
		ret = nvi_wr_accel_config(st, 0, st->chip_config.accel_fs, 0);
		if (ret < 0)
			ret_t |= ret;
		ret_t |= nvi_dev_delay(st, DEV_ACCEL);
	}
	if (st->master_enable & (1 << DEV_ANGLVEL)) {
		for (i = 0; i < AXIS_N; i++)
			ret_t |= nvi_wr_gyro_offset(st, i,
						 (u16)(st->rom_gyro_offset[i] +
						    st->input_gyro_offset[i]));
		ret = nvi_wr_gyro_config(st, 0, st->chip_config.fsr, 0);
		if (ret < 0)
			ret_t |= ret;
		ret_t |= nvi_dev_delay(st, DEV_ANGLVEL);
	}
	if ((st->master_enable & DEV_MPU_MASK) || (st->aux.reset_fifo &&
						(st->master_enable & DEV_AUX)))
		ret_t |= nvi_reset(st, true, false);
	ret_t |= nvi_pm(st, NVI_PM_AUTO);
	return ret_t;
}

static int nvi_batch(struct nvi_state *st)
{
	int ret = 0;

	if (st->hal->dmp)
		ret = 0;
	return ret;
}

static void nvi_aux_dbg(struct nvi_state *st, char *tag, int val)
{
	struct nvi_mpu_port *n;
	struct aux_port *p;
	struct aux_ports *a;
	u8 data[4];
	int i;

	if (!(st->dbg & NVI_DBG_SPEW_AUX))
		return;

	dev_info(&st->i2c->dev, "%s %s %d\n", __func__, tag, val);
	for (i = 0; i < AUX_PORT_MAX; i++) {
		nvi_i2c_rd(st, st->hal->reg->i2c_slv0_addr.bank,
			   st->hal->reg->i2c_slv0_addr.reg + (i * 3), 3, data);
		nvi_i2c_rd(st, st->hal->reg->i2c_slv0_do.bank,
			   st->hal->reg->i2c_slv0_do.reg + i, 1, &data[3]);
		/* HW = hardware */
		pr_info("HW: P%d AD=%x RG=%x CL=%x DO=%x\n",
			i, data[0], data[1], data[2], data[3]);
		n = &st->aux.port[i].nmp;
		/* NS = nmp structure */
		pr_info("NS: P%d AD=%x RG=%x CL=%x DO=%x MS=%u US=%lu SB=%x\n",
			i, n->addr, n->reg, n->ctrl, n->data_out, n->delay_ms,
			n->delay_us, n->shutdown_bypass);
		p = &st->aux.port[i];
		/* PS = port structure */
		pr_info("PS: P%d OFFSET=%u EN=%x FIFOEN=%x HWDOUT=%x\n", i,
			p->ext_data_offset, !!(st->enable[DEV_AUX] & (1 << i)),
			p->fifo_en, p->hw_do);
	}
	a = &st->aux;
	pr_info("AUX: EN=%x MEN=%x MDLY=%x GDLY=%u DATN=%u BPEN=%x BPLK=%d\n",
		!!(st->master_enable & (1 << DEV_AUX)),
		!!(st->rc.user_ctrl & BIT_I2C_MST_EN),
		(st->rc.i2c_slv_ctrl[AUX_PORT_IO] & BITS_I2C_MST_DLY),
		st->smplrt_delay_us[DEV_AUX], a->ext_data_n,
		(st->rc.int_pin_cfg & BIT_BYPASS_EN), a->bypass_lock);
}

static void nvi_aux_read(struct nvi_state *st, s64 ts)
{
	struct aux_port *ap;
	unsigned int i;
	unsigned int len;
	u8 *p;
	int ret;

	if ((!st->aux.ext_data_n) || (!(st->rc.user_ctrl & BIT_I2C_MST_EN)))
		return;

	ret = nvi_i2c_rd(st, st->hal->reg->ext_sens_data_00.bank,
			 st->hal->reg->ext_sens_data_00.reg,
			 st->aux.ext_data_n, (u8 *)&st->aux.ext_data);
	if (ret)
		return;

	if (st->flush)
		ts = 0;
	for (i = 0; i < AUX_PORT_IO; i++) {
		ap = &st->aux.port[i];
		if ((st->rc.i2c_slv_ctrl[i] & BIT_SLV_EN) && (!ap->fifo_en) &&
					       (ap->nmp.addr & BIT_I2C_READ) &&
						   (ap->nmp.handler != NULL)) {
			p = &st->aux.ext_data[ap->ext_data_offset];
			len = ap->nmp.ctrl & BITS_I2C_SLV_CTRL_LEN;
			ap->nmp.handler(p, len, ts, ap->nmp.ext_driver);
		}
	}
}

static void nvi_aux_ext_data_offset(struct nvi_state *st)
{
	int i;
	unsigned short offset;

	offset = 0;
	for (i = 0; i < AUX_PORT_IO; i++) {
		if ((st->rc.i2c_slv_ctrl[i] & BIT_SLV_EN) &&
				  (st->aux.port[i].nmp.addr & BIT_I2C_READ)) {
			st->aux.port[i].ext_data_offset = offset;
			offset += (st->aux.port[i].nmp.ctrl &
				   BITS_I2C_SLV_CTRL_LEN);
		}
	}
	if (offset > AUX_EXT_DATA_REG_MAX) {
		offset = AUX_EXT_DATA_REG_MAX;
		dev_err(&st->i2c->dev,
			"%s ERR MPU slaves exceed data storage\n", __func__);
	}
	st->aux.ext_data_n = offset;
	return;
}

static int nvi_aux_port_data_out(struct nvi_state *st,
				 int port, u8 data_out)
{
	int ret;

	ret = nvi_wr_i2c_slv_do(st, port, data_out);
	if (!ret) {
		st->aux.port[port].nmp.data_out = data_out;
		st->aux.port[port].hw_do = true;
	} else {
		st->aux.port[port].hw_do = false;
	}
	return ret;
}

static int nvi_aux_port_wr(struct nvi_state *st, int port)
{
	struct aux_port *ap;
	int ret;

	ap = &st->aux.port[port];
	ret = nvi_wr_i2c_slv_addr(st, port, ap->nmp.addr);
	ret |= nvi_wr_i2c_slv_reg(st, port, ap->nmp.reg);
	ret |= nvi_wr_i2c_slv_do(st, port, ap->nmp.data_out);
	return ret;
}

static int nvi_aux_port_en(struct nvi_state *st,
			   int port, bool en)
{
	struct aux_port *ap;
	u8 val;
	int ret = 0;

	st->aux.ext_data_n = 0;
	ap = &st->aux.port[port];
	if ((!(st->rc.i2c_slv_addr[port])) && en) {
		ret = nvi_aux_port_wr(st, port);
		if (!ret)
			ap->hw_do = true;
	}
	if ((!ap->hw_do) && en)
		nvi_aux_port_data_out(st, port, ap->nmp.data_out);
	if (port == AUX_PORT_IO) {
		ret = nvi_wr_i2c_slv4_ctrl(st, en);
	} else {
		if (en)
			val = (ap->nmp.ctrl | BIT_SLV_EN);
		else
			val = 0;
		ret = nvi_wr_i2c_slv_ctrl(st, port, val);
	}
	if (ret > 0) {
		nvi_aux_ext_data_offset(st);
		ret = 0;
	}
	return ret;
}

static int nvi_aux_enable(struct nvi_state *st, bool enable)
{
	bool en;
	unsigned int i;
	int ret = 0;

	if (st->rc.int_pin_cfg & BIT_BYPASS_EN)
		enable = false;
	en = false;
	if (enable) {
		/* global enable is honored only if a port is enabled */
		for (i = 0; i < AUX_PORT_MAX; i++) {
			if (st->enable[DEV_AUX] & (1 << i)) {
				en = true;
				break;
			}
		}
		if (en == (bool)(st->rc.user_ctrl & BIT_I2C_MST_EN))
			/* if already on then just update delays */
			nvi_dev_delay(st, DEV_AUX);
	}
	if (en)
		st->master_enable |= (1 << DEV_AUX);
	else
		st->master_enable &= (~(1 << DEV_AUX));
	if ((bool)(st->rc.user_ctrl & BIT_I2C_MST_EN) == en) {
		if (st->aux.reset_fifo)
			nvi_reset(st, true, false);
		return 0;
	}

	if (en) {
		for (i = 0; i < AUX_PORT_MAX; i++) {
			if (st->enable[DEV_AUX] & (1 << i))
				ret |= nvi_aux_port_en(st, i, true);
		}
	} else {
		for (i = 0; i < AUX_PORT_MAX; i++) {
			if (st->rc.i2c_slv_addr[i])
				nvi_aux_port_en(st, i, false);
		}
	}
	ret |= nvi_dev_delay(st, DEV_AUX);
	if (st->aux.reset_fifo)
		ret |= nvi_reset(st, true, false);
	else
		ret |= nvi_user_ctrl_en(st, true, en);
	return ret;
}

static int nvi_aux_port_enable(struct nvi_state *st,
			       int port, bool enable, bool fifo_enable)
{
	unsigned int i;
	int ret;

	if (enable)
		st->enable[DEV_AUX] |= (1 << port);
	else
		st->enable[DEV_AUX] &= ~(1 << port);
	/* find the fastest polling of all the enabled aux devices */
	st->delay_us[DEV_AUX] = st->hal->smplrt[DEV_AUX]->delay_us_max;
	for (i = 0; i < AUX_PORT_MAX; i++) {
		if (st->aux.port[i].nmp.delay_us && (st->enable[DEV_AUX] &
						     (1 << i))) {
			if (st->aux.port[i].nmp.delay_us <
							 st->delay_us[DEV_AUX])
				st->delay_us[DEV_AUX] =
						  st->aux.port[i].nmp.delay_us;
		}
	}
	if ((!enable) || (!(st->aux.port[port].nmp.addr & BIT_I2C_READ)))
		fifo_enable = false;
#ifdef NVI_AUX_FIFO_ENABLE
	if (st->aux.port[port].fifo_en != fifo_enable)
		st->aux.reset_fifo = true;
	st->aux.port[port].fifo_en = fifo_enable;
#else
	st->aux.port[port].fifo_en = false;
#endif /* NVI_AUX_FIFO_ENABLE */
	if (enable && (st->rc.int_pin_cfg & BIT_BYPASS_EN))
		return 0;

	ret = nvi_aux_port_en(st, port, enable);
	ret |= nvi_aux_enable(st, true);
	return ret;
}

static int nvi_reset(struct nvi_state *st,
		     bool reset_fifo, bool reset_i2c)
{
	u8 val;
	bool irq = false;
	unsigned long flags;
	int ret;

	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s FIFO=%x I2C=%x\n",
			 __func__, reset_fifo, reset_i2c);
	if (st->rc.int_enable)
		irq = true;
	ret = nvi_int_able(st, false);
	val = 0;
	if (reset_i2c) {
		st->aux.reset_i2c = false;
		ret |= nvi_aux_enable(st, false);
		val |= BIT_I2C_MST_RST;
	}
	if (reset_fifo) {
		st->aux.reset_fifo = false;
		val |= BIT_FIFO_RST;
		if (st->master_enable & (1 << DEV_DMP))
			val |= BIT_DMP_RST;
	}
	ret |= nvi_user_ctrl_en(st, !reset_fifo, !reset_i2c);
	val |= st->rc.user_ctrl;
	ret |= nvi_wr_user_ctrl(st, val);
	if (reset_i2c)
		ret |= nvi_aux_enable(st, true);
	else
		ret |= nvi_user_ctrl_en(st, true, true);
	if (reset_fifo && (st->rc.user_ctrl & BIT_FIFO_EN)) {
		spin_lock_irqsave(&st->time_stamp_lock, flags);
		kfifo_reset(&st->timestamps);
		spin_unlock_irqrestore(&st->time_stamp_lock, flags);
		st->fifo_ts = nvi_get_time_ns(st);
	}
	if (irq)
		ret |= nvi_int_able(st, true);
	return ret;
}

static int nvi_aux_port_free(struct nvi_state *st, int port)
{
	memset(&st->aux.port[port], 0, sizeof(struct aux_port));
	st->enable[DEV_AUX] &= ~(1 << port);
	if (st->rc.i2c_slv_addr[port]) {
		nvi_aux_port_wr(st, port);
		nvi_aux_port_en(st, port, false);
		nvi_aux_enable(st, false);
		nvi_aux_enable(st, true);
		if (port != AUX_PORT_IO)
			st->aux.reset_i2c = true;
	}
	return 0;
}

static int nvi_aux_port_alloc(struct nvi_state *st,
			      struct nvi_mpu_port *nmp, int port)
{
	int i;

	if (st->aux.reset_i2c)
		nvi_reset(st, false, true);
	if (port < 0) {
		for (i = 0; i < AUX_PORT_IO; i++) {
			if (st->aux.port[i].nmp.addr == 0)
				break;
		}
		if (i == AUX_PORT_IO)
			return -ENODEV;
	} else {
		if (st->aux.port[port].nmp.addr == 0)
			i = port;
		else
			return -ENODEV;
	}

	memset(&st->aux.port[i], 0, sizeof(struct aux_port));
	memcpy(&st->aux.port[i].nmp, nmp, sizeof(struct nvi_mpu_port));
	return i;
}

static int nvi_aux_bypass_enable(struct nvi_state *st, bool enable)
{
	u8 val;
	int ret;

	if ((bool)(st->rc.int_pin_cfg & BIT_BYPASS_EN) == enable)
		return 0;

	val = st->rc.int_pin_cfg;
	if (enable) {
		ret = nvi_aux_enable(st, false);
		if (!ret) {
			val |= BIT_BYPASS_EN;
			ret = nvi_wr_int_pin_cfg(st, val);
		}
	} else {
		val &= ~BIT_BYPASS_EN;
		ret = nvi_wr_int_pin_cfg(st, val);
		if (!ret)
			nvi_aux_enable(st, true);
	}
	return ret;
}

static int nvi_aux_bypass_request(struct nvi_state *st, bool enable)
{
	s64 ns;
	s64 to;
	int ret = 0;

	if ((bool)(st->rc.int_pin_cfg & BIT_BYPASS_EN) == enable) {
		st->aux.bypass_timeout_ns = nvi_get_time_ns(st);
		st->aux.bypass_lock++;
		if (!st->aux.bypass_lock)
			dev_err(&st->i2c->dev, "%s rollover ERR\n", __func__);
	} else {
		if (st->aux.bypass_lock) {
			ns = nvi_get_time_ns(st) - st->aux.bypass_timeout_ns;
			to = st->chip_config.bypass_timeout_ms * 1000000;
			if (ns > to)
				st->aux.bypass_lock = 0;
			else
				ret = -EBUSY;
		}
		if (!st->aux.bypass_lock) {
			ret = nvi_aux_bypass_enable(st, enable);
			if (ret)
				dev_err(&st->i2c->dev, "%s ERR=%d\n",
					__func__, ret);
			else
				st->aux.bypass_lock++;
		}
	}
	return ret;
}

static int nvi_aux_bypass_release(struct nvi_state *st)
{
	int ret = 0;

	if (st->aux.bypass_lock)
		st->aux.bypass_lock--;
	if (!st->aux.bypass_lock) {
		ret = nvi_aux_bypass_enable(st, false);
		if (ret)
			dev_err(&st->i2c->dev, "%s ERR=%d\n", __func__, ret);
	}
	return ret;
}

static int nvi_aux_dev_valid(struct nvi_state *st,
			     struct nvi_mpu_port *nmp, u8 *data)
{
	u8 val;
	int i;
	int ret;

	/* turn off bypass */
	ret = nvi_aux_bypass_request(st, false);
	if (ret)
		return -EBUSY;

	/* grab the special port */
	ret = nvi_aux_port_alloc(st, nmp, AUX_PORT_IO);
	if (ret != AUX_PORT_IO) {
		nvi_aux_bypass_release(st);
		return -EBUSY;
	}

	/* enable it */
	st->aux.port[AUX_PORT_IO].nmp.delay_ms = 0;
	st->aux.port[AUX_PORT_IO].nmp.delay_us =
					st->hal->smplrt[DEV_AUX]->delay_us_min;
	ret = nvi_aux_port_enable(st, AUX_PORT_IO, true, false);
	if (ret) {
		nvi_aux_port_free(st, AUX_PORT_IO);
		nvi_aux_bypass_release(st);
		return -EBUSY;
	}

	/* now turn off all the other ports for fastest response */
	for (i = 0; i < AUX_PORT_IO; i++) {
		if (st->rc.i2c_slv_addr[i])
			nvi_aux_port_en(st, i, false);
	}
	/* start reading the results */
	for (i = 0; i < AUX_DEV_VALID_READ_LOOP_MAX; i++) {
		mdelay(AUX_DEV_VALID_READ_DELAY_MS);
		val = 0;
		ret = nvi_i2c_rd(st, st->hal->reg->i2c_mst_status.bank,
				 st->hal->reg->i2c_mst_status.reg, 1, &val);
		if (ret)
			continue;

		if (val & 0x50)
			break;
	}
	/* these will restore all previously disabled ports */
	nvi_aux_bypass_release(st);
	nvi_aux_port_free(st, AUX_PORT_IO);
	if (i == AUX_DEV_VALID_READ_LOOP_MAX)
		return -ENODEV;

	if (val & 0x10) /* NACK */
		return -EIO;

	if (nmp->addr & BIT_I2C_READ) {
		ret = nvi_i2c_rd(st, st->hal->reg->i2c_slv4_di.bank,
				 st->hal->reg->i2c_slv4_di.reg, 1, &val);
		if (ret)
			return -EBUSY;

		*data = (u8)val;
		dev_info(&st->i2c->dev, "%s MPU read 0x%x from device 0x%x\n",
			__func__, val, (nmp->addr & ~BIT_I2C_READ));
	} else {
		dev_info(&st->i2c->dev, "%s MPU found device 0x%x\n",
			__func__, (nmp->addr & ~BIT_I2C_READ));
	}
	return 0;
}

static int nvi_aux_mpu_call_pre(struct nvi_state *st, int port)
{
	if ((port < 0) || (port >= AUX_PORT_IO))
		return -EINVAL;

	if (st->shutdown || st->suspend)
		return -EPERM;

	if (!st->aux.port[port].nmp.addr)
		return -EINVAL;

	return 0;
}

static int nvi_aux_mpu_call_post(struct nvi_state *st,
				 char *tag, int ret)
{
	if (ret < 0)
		ret = -EBUSY;
	nvi_aux_dbg(st, tag, ret);
	return ret;
}

/* See the mpu.h file for details on the nvi_mpu_ calls.
 */
int nvi_mpu_dev_valid(struct nvi_mpu_port *nmp, u8 *data)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret = -EPERM;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s\n", __func__);
	} else {
		pr_debug("%s ERR -EAGAIN\n", __func__);
		return -EAGAIN;
	}

	if (nmp == NULL)
		return -EINVAL;

	if ((nmp->addr & BIT_I2C_READ) && (data == NULL))
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	if (!st->shutdown && !st->suspend) {
		nvi_pm(st, NVI_PM_ON);
		ret = nvi_aux_dev_valid(st, nmp, data);
		nvi_pm(st, NVI_PM_AUTO);
		nvi_aux_dbg(st, "nvi_mpu_dev_valid ret=", ret);
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_dev_valid);

int nvi_mpu_port_alloc(struct nvi_mpu_port *nmp)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret = -EPERM;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s\n", __func__);
	} else {
		pr_debug("%s ERR -EAGAIN\n", __func__);
		return -EAGAIN;
	}

	if (nmp == NULL)
		return -EINVAL;

	if (!(nmp->ctrl & BITS_I2C_SLV_CTRL_LEN))
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	if (!st->shutdown && !st->suspend) {
		nvi_pm(st, NVI_PM_ON);
		ret = nvi_aux_port_alloc(st, nmp, -1);
		nvi_pm(st, NVI_PM_AUTO);
		ret = nvi_aux_mpu_call_post(st,
					 "nvi_mpu_port_alloc ret/port=", ret);
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_port_alloc);

int nvi_mpu_port_free(int port)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s port %d\n", __func__, port);
	} else {
		pr_debug("%s port %d ERR -EAGAIN\n", __func__, port);
		return -EAGAIN;
	}

	mutex_lock(&indio_dev->mlock);
	ret = nvi_aux_mpu_call_pre(st, port);
	if (!ret) {
		nvi_pm(st, NVI_PM_ON);
		ret = nvi_aux_port_free(st, port);
		nvi_pm(st, NVI_PM_AUTO);
		ret = nvi_aux_mpu_call_post(st, "nvi_mpu_port_free ret=", ret);
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_port_free);

int nvi_mpu_enable(int port, bool enable, bool fifo_enable)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s port %d: %x\n", __func__, port, enable);
	} else {
		pr_debug("%s port %d: %x ERR -EAGAIN\n",
			 __func__, port, enable);
		return -EAGAIN;
	}

	mutex_lock(&indio_dev->mlock);
	ret = nvi_aux_mpu_call_pre(st, port);
	if (!ret) {
		nvi_pm(st, NVI_PM_ON);
		ret = nvi_aux_port_enable(st, port, enable, fifo_enable);
		nvi_pm(st, NVI_PM_AUTO);
		ret = nvi_aux_mpu_call_post(st, "nvi_mpu_enable ret=", ret);
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_enable);

int nvi_mpu_delay_ms(int port, u8 delay_ms)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s port %d: %u\n", __func__, port, delay_ms);
	} else {
		pr_debug("%s port %d: %u ERR -EAGAIN\n",
			 __func__, port, delay_ms);
		return -EAGAIN;
	}

	mutex_lock(&indio_dev->mlock);
	ret = nvi_aux_mpu_call_pre(st, port);
	if (!ret) {
		st->aux.port[port].nmp.delay_ms = delay_ms;
		if (st->rc.i2c_slv_ctrl[port] & BIT_SLV_EN)
			ret = nvi_dev_delay(st, DEV_AUX);
		ret |= nvi_aux_mpu_call_post(st, "nvi_mpu_delay_ms ret=", ret);
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_delay_ms);

int nvi_mpu_delay_us(int port, unsigned long delay_us)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s port %d: %lu\n", __func__, port, delay_us);
	} else {
		pr_debug("%s port %d: %lu ERR -EAGAIN\n",
			__func__, port, delay_us);
		return -EAGAIN;
	}

	mutex_lock(&indio_dev->mlock);
	ret = nvi_aux_mpu_call_pre(st, port);
	if (!ret) {
		if (st->aux.port[port].nmp.delay_us != delay_us) {
			st->aux.port[port].nmp.delay_us = delay_us;
			if (st->rc.i2c_slv_ctrl[port] & BIT_SLV_EN)
				ret = nvi_dev_delay(st, DEV_AUX);
		}
		ret |= nvi_aux_mpu_call_post(st, "nvi_mpu_delay_us ret=", ret);
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_delay_us);

int nvi_mpu_data_out(int port, u8 data_out)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret;

	if (indio_dev == NULL)
		return -EAGAIN;

	st = iio_priv(indio_dev);
	ret = nvi_aux_mpu_call_pre(st, port);
	if (!ret) {
		if (st->rc.i2c_slv_ctrl[port] & BIT_SLV_EN) {
			ret = nvi_aux_port_data_out(st, port, data_out);
		} else {
			st->aux.port[port].nmp.data_out = data_out;
			st->aux.port[port].hw_do = false;
		}
		if (ret < 0)
			ret = -EBUSY;
	}
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_data_out);

int nvi_mpu_batch(int port, unsigned int flags,
		  unsigned int period_us, unsigned int timeout_us)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s port %d: f=%x p=%u t=%u\n",
				__func__, port, flags, period_us, timeout_us);
	} else {
		pr_debug("%s port %d: f=%x p=%u t=%u ERR -EAGAIN\n",
			__func__, port, flags, period_us, timeout_us);
		return -EAGAIN;
	}

	mutex_lock(&indio_dev->mlock);
	ret = nvi_aux_mpu_call_pre(st, port);
	if (!ret) {
		if ((st->aux.port[port].nmp.id != ID_INVALID) &&
				(st->aux.port[port].nmp.id < ID_INVALID_END)) {
			st->aux.port[port].batch_flags = flags;
			st->aux.port[port].batch_period_us = period_us;
			st->aux.port[port].batch_timeout_us = timeout_us;
			if (st->aux.port[port].nmp.delay_us != period_us) {
				st->aux.port[port].nmp.delay_us = period_us;
				if (st->rc.i2c_slv_ctrl[port] & BIT_SLV_EN)
					ret = nvi_dev_delay(st, DEV_AUX);
			}
			ret = nvi_batch(st);
			ret = nvi_aux_mpu_call_post(st,
					      "nvi_mpu_batch ret/flags=", ret);
		} else {
			ret = -EINVAL;
		}
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_batch);

int nvi_mpu_flush(int port)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s port %d\n", __func__, port);
	} else {
		pr_debug("%s port %d ERR -EAGAIN\n", __func__, port);
		return -EAGAIN;
	}

	mutex_lock(&indio_dev->mlock);
	ret = nvi_aux_mpu_call_pre(st, port);
	if (!ret) {
		if ((st->aux.port[port].nmp.id != ID_INVALID) &&
				(st->aux.port[port].nmp.id < ID_INVALID_END)) {
			ret = nvi_enable(indio_dev);
			ret = nvi_aux_mpu_call_post(st, "nvi_mpu_flush ret=",
						    ret);
		} else {
			ret = -EINVAL;
		}
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_flush);

int nvi_mpu_fifo(int port, unsigned int *reserve, unsigned int *max)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s port %d\n", __func__, port);
	} else {
		pr_debug("%s port %d ERR -EAGAIN\n", __func__, port);
		return -EAGAIN;
	}

	mutex_lock(&indio_dev->mlock);
	ret = nvi_aux_mpu_call_pre(st, port);
	if (!ret) {
		if ((st->aux.port[port].nmp.id != ID_INVALID) &&
			(st->aux.port[port].nmp.id < ID_INVALID_END)) {
			if (reserve) {
				if (st->hal->dmp)
					*reserve =
						  st->chip_config.fifo_reserve;
				else
					*reserve = 0;
			}
			if (max) {
				if (st->hal->dmp)
					*max = st->chip_config.fifo_max;
				else
					*max = 0;
			}
			ret = nvi_aux_mpu_call_post(st, "nvi_mpu_fifo ret=",
						    ret);
		} else {
			ret = -EINVAL;
		}
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_fifo);

int nvi_mpu_bypass_request(bool enable)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;
	int ret = -EPERM;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s enable=%x\n", __func__, enable);
	} else {
		pr_debug("%s ERR -EAGAIN\n", __func__);
		return -EAGAIN;
	}

	mutex_lock(&indio_dev->mlock);
	if (!st->shutdown && !st->suspend) {
		nvi_pm(st, NVI_PM_ON);
		ret = nvi_aux_bypass_request(st, enable);
		nvi_pm(st, NVI_PM_AUTO);
		ret = nvi_aux_mpu_call_post(st, "nvi_mpu_bypass_request ret=",
					    ret);
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
EXPORT_SYMBOL(nvi_mpu_bypass_request);

int nvi_mpu_bypass_release(void)
{
	struct iio_dev *indio_dev = indio_dev_local;
	struct nvi_state *st;

	if (indio_dev != NULL) {
		st = iio_priv(indio_dev);
		if (st->dbg & NVI_DBG_SPEW_AUX)
			pr_info("%s\n", __func__);
	} else {
		pr_debug("%s\n", __func__);
		return 0;
	}

	mutex_lock(&indio_dev->mlock);
	if (!st->shutdown && !st->suspend) {
		nvi_pm(st, NVI_PM_ON);
		nvi_aux_bypass_release(st);
		nvi_pm(st, NVI_PM_AUTO);
		nvi_aux_mpu_call_post(st, "nvi_mpu_bypass_release", 0);
	}
	mutex_unlock(&indio_dev->mlock);
	return 0;
}
EXPORT_SYMBOL(nvi_mpu_bypass_release);


static unsigned int nvi_buf_index(unsigned int size, unsigned int *bytes)
{
	unsigned int index;

	if (!(*bytes % size))
		index = *bytes;
	else
		index = *bytes - *bytes % size + size;
	*bytes = index + size;
	return index;
}

static void nvi_buf_push(struct iio_dev *indio_dev, s64 ts)
{
	struct nvi_state *st = iio_priv(indio_dev);
	unsigned char buf[24];
	unsigned int n;
	unsigned int i;
	unsigned int bytes = 0;
	unsigned int axis;

	for (axis = 0; axis < AXIS_N; axis++) {
		if (iio_scan_mask_query(indio_dev, indio_dev->buffer,
					axis + NVI_SCAN_ACCEL_X)) {
			n = sizeof(st->accel[axis]);
			i = nvi_buf_index(n, &bytes);
			memcpy(&buf[i], &st->accel[axis], n);
		}
	}
	for (axis = 0; axis < AXIS_N; axis++) {
		if (iio_scan_mask_query(indio_dev, indio_dev->buffer,
					axis + NVI_SCAN_ANGLVEL_X)) {
			n = sizeof(st->anglvel[axis]);
			i = nvi_buf_index(n, &bytes);
			memcpy(&buf[i], &st->anglvel[axis], n);
		}
	}
	if (iio_scan_mask_query(indio_dev, indio_dev->buffer, NVI_SCAN_TEMP)) {
		n = sizeof(st->temp);
		i = nvi_buf_index(n, &bytes);
		memcpy(&buf[i], &st->temp, n);
	}
	st->ts = ts;
	if (indio_dev->buffer->scan_timestamp) {
		if (st->flush || (ts < st->push_ts))
			ts = 0;
		n = sizeof(ts);
		i = nvi_buf_index(n, &bytes);
		memcpy(&buf[i], &ts, n);
	}
	if (iio_buffer_enabled(indio_dev)) {
		if (st->dbg & NVI_DBG_SPEW_BUF) {
			for (i = 0; i < bytes; i++)
				dev_info(&st->i2c->dev, "buf[%d]=%x\n",
					 i, buf[i]);
		}
		iio_push_to_buffers(indio_dev, buf);
	}
}

static unsigned int nvi_report_accel(struct nvi_state *st, u8 *data, s64 ts)
{
	s16 accel[AXIS_N];
	s16 accel_uc[AXIS_N];
	unsigned int i;
	unsigned int buf_i = 0;

	accel_uc[AXIS_X] = be16_to_cpup((__be16 *)&data[0]);
	accel_uc[AXIS_Y] = be16_to_cpup((__be16 *)&data[2]);
	accel_uc[AXIS_Z] = be16_to_cpup((__be16 *)&data[4]);
	accel[AXIS_X] = accel_uc[AXIS_X] * st->chip_info.multi;
	accel[AXIS_Y] = accel_uc[AXIS_Y] * st->chip_info.multi;
	accel[AXIS_Z] = accel_uc[AXIS_Z] * st->chip_info.multi;
	if (!(st->rc.pwr_mgmt_2 & BIT_STBY_XA)) {
		buf_i += 2;
	} else {
		accel_uc[AXIS_X] = 0;
		accel[AXIS_X] = 0;
	}
	if (!(st->rc.pwr_mgmt_2 & BIT_STBY_YA)) {
		buf_i += 2;
	} else {
		accel_uc[AXIS_Y] = 0;
		accel[AXIS_Y] = 0;
	}
	if (!(st->rc.pwr_mgmt_2 & BIT_STBY_ZA)) {
		buf_i += 2;
	} else {
		accel_uc[AXIS_Z] = 0;
		accel[AXIS_Z] = 0;
	}
	for (i = 0; i < AXIS_N; i++) {
		st->accel_uc[i] = accel_uc[i];
		st->accel[i] = accel[i];
	}
	if (st->dbg & NVI_DBG_SPEW_ACCEL_UC)
		dev_info(&st->i2c->dev, "accel_uc %hd %hd %hd %lld\n",
			 st->accel_uc[AXIS_X], st->accel_uc[AXIS_Y],
			 st->accel_uc[AXIS_Z], ts);
	if (st->dbg & NVI_DBG_SPEW_ACCEL)
		dev_info(&st->i2c->dev, "accel %hd %hd %hd %lld\n",
			 st->accel[AXIS_X], st->accel[AXIS_Y],
			 st->accel[AXIS_Z], ts);
	return buf_i;
}

static void nvi_report_temp(struct nvi_state *st, u8 *data, s64 ts)
{
	st->temp = be16_to_cpup((__be16 *)data);
	if (st->dbg & NVI_DBG_SPEW_TEMP)
		dev_info(&st->i2c->dev, "temp %d %lld\n", st->temp, ts);
}

static unsigned int nvi_report_gyro(struct nvi_state *st, unsigned int mask,
				    u8 *data, s64 ts)
{
	unsigned int i;
	unsigned int buf_i = 0;

	for (i = 0; i < AXIS_N; i++) {
		if (mask & (1 << i)) {
			st->anglvel[i] = be16_to_cpup((__be16 *)&data[buf_i]);
			buf_i += 2;
		}
	}
	if (st->dbg & NVI_DBG_SPEW_ANGLVEL)
		dev_info(&st->i2c->dev, "anglvel %hd %hd %hd %lld\n",
			 st->anglvel[AXIS_X], st->anglvel[AXIS_Y],
			 st->anglvel[AXIS_Z], ts);
	return buf_i;
}

static int nvi_accel_read(struct nvi_state *st, s64 ts)
{
	u8 data[6];
	int ret;

	ret = nvi_i2c_rd(st, st->hal->reg->accel_xout_h.bank,
			 st->hal->reg->accel_xout_h.reg, 6, data);
	if (!ret)
		ret = nvi_report_accel(st, data, ts);
	return ret;
}

static int nvi_temp_read(struct nvi_state *st, s64 ts)
{
	u8 data[2];
	int ret;

	ret = nvi_i2c_rd(st, st->hal->reg->temp_out_h.bank,
			 st->hal->reg->temp_out_h.reg, 2, data);
	if (!ret)
		nvi_report_temp(st, data, ts);
	return ret;
}

static unsigned int nvi_fifo_read_accel(struct nvi_state *st,
					unsigned int buf_i, s64 ts)
{
	if (st->rc.fifo_en & (1 << st->hal->bit->accel_fifo_en)) {
		nvi_report_accel(st, &st->buf[buf_i], ts);
		buf_i += 6;
	}
	return buf_i;
}

static unsigned int nvi_fifo_read_anglvel(struct nvi_state *st,
					  unsigned int buf_i, s64 ts)
{
	unsigned int mask = 0;

	if (st->rc.fifo_en & (1 << st->hal->bit->gyro_x_fifo_en))
		mask |= 1 << AXIS_X;
	if (st->rc.fifo_en & (1 << st->hal->bit->gyro_y_fifo_en))
		mask |= 1 << AXIS_Y;
	if (st->rc.fifo_en & (1 << st->hal->bit->gyro_z_fifo_en))
		mask |= 1 << AXIS_Z;
	if (mask)
		buf_i += nvi_report_gyro(st, mask, &st->buf[buf_i], ts);
	return buf_i;
}

static unsigned int nvi_fifo_read_temp(struct nvi_state *st,
				       unsigned int buf_i, s64 ts)
{
	if (st->rc.fifo_en & (1 << st->hal->bit->temp_fifo_en)) {
		nvi_report_temp(st, &st->buf[buf_i], ts);
		buf_i += 2;
	}
	return buf_i;
}

static irqreturn_t nvi_irq_thread(int irq, void *dev_id)
{
	struct nvi_state *st = (struct nvi_state *)dev_id;
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	struct aux_port *ap;
	u16 fifo_count = 0;
	u16 fifo_sample_size;
	u16 fifo_rd_n;
	u16 fifo_align;
	s64 ts;
	s64 ts_irq;
	s64 delay;
	bool push;
	unsigned int mask;
	unsigned int buf_index;
	unsigned int ts_len;
	unsigned int samples;
	unsigned int len;
	unsigned int i;
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (st->suspend)
		goto nvi_irq_thread_exit;

	ts = nvi_get_time_ns(st);
	/* if only accelermeter data */
	if (st->rc.pwr_mgmt_1 & BIT_CYCLE) {
		ret = nvi_accel_read(st, ts);
		if (ret < 0)
			goto nvi_irq_thread_exit;

		nvi_buf_push(indio_dev, ts);
		st->flush = false;
		goto nvi_irq_thread_exit;
	}

	/* handle FIFO disabled data */
	push = false;
	if (((~st->rc.pwr_mgmt_2) & BIT_PWR_ACCEL_STBY) &&
		    (!(st->rc.fifo_en & (1 << st->hal->bit->accel_fifo_en)))) {
		ret = nvi_accel_read(st, ts);
		if (ret > 0)
			push = true;
	}
	if (!(st->rc.fifo_en & (1 << st->hal->bit->temp_fifo_en)))
		ret = nvi_temp_read(st, ts);
	mask = 0;
	if (!(st->rc.fifo_en & (1 << st->hal->bit->gyro_x_fifo_en)))
		mask |= 1 << AXIS_X;
	if (!(st->rc.fifo_en & (1 << st->hal->bit->gyro_y_fifo_en)))
		mask |= 1 << AXIS_Y;
	if (!(st->rc.fifo_en & (1 << st->hal->bit->gyro_z_fifo_en)))
		mask |= 1 << AXIS_Z;
	/* FIXME change to STANDBY_XYZ */
	if (st->enable[DEV_ANGLVEL] & mask) {
		buf_index = 0;
		ret = 0;
		if (mask & (1 << AXIS_X)) {
			ret |= nvi_i2c_rd(st, st->hal->reg->gyro_xout_h.bank,
					  st->hal->reg->gyro_xout_h.reg,
					  2, &st->buf[buf_index]);
			buf_index += 2;
		}
		if (mask & (1 << AXIS_Y)) {
			ret |= nvi_i2c_rd(st, st->hal->reg->gyro_xout_h.bank,
					  st->hal->reg->gyro_xout_h.reg + 2,
					  2, &st->buf[buf_index]);
			buf_index += 2;
		}
		if (mask & (1 << AXIS_Z)) {
			ret |= nvi_i2c_rd(st, st->hal->reg->gyro_xout_h.bank,
					  st->hal->reg->gyro_xout_h.reg + 4,
					  2, &st->buf[buf_index]);
			buf_index += 2;
		}
		if (!ret) {
			buf_index = nvi_report_gyro(st, mask, st->buf, ts);
			if (buf_index)
				push = true;
		}
	}
	nvi_aux_read(st, ts);
	if (!(st->rc.user_ctrl & BIT_FIFO_EN)) {
		if (push)
			/* push FIFO disabled data before exit */
			nvi_buf_push(indio_dev, ts);
		st->flush = false;
		goto nvi_irq_thread_exit;
	}

	/* handle FIFO enabled data */
	fifo_sample_size = st->fifo_sample_size;
	if (!fifo_sample_size)
		goto nvi_irq_thread_exit;

	/* must get IRQ timestamp len first for timestamp best-fit algorithm */
	ts_len = kfifo_len(&st->timestamps);
	ret = nvi_i2c_rd(st, st->hal->reg->fifo_count_h.bank,
			 st->hal->reg->fifo_count_h.reg, 2, st->buf);
	if (ret)
		goto nvi_irq_thread_exit;

	fifo_count = be16_to_cpup((__be16 *)(&st->buf));
	/* FIFO threshold */
	if (st->chip_config.fifo_thr > fifo_sample_size) {
		if (fifo_count > st->chip_config.fifo_thr) {
			if (st->dbg & NVI_DBG_SPEW_FIFO)
				dev_info(&st->i2c->dev,
					 "FIFO threshold exceeded\n");
			goto nvi_irq_thread_exit_reset;
		}
	}

	fifo_align = fifo_count % fifo_sample_size;
	if (fifo_count < fifo_sample_size + fifo_align)
		/* consider resetting FIFO if doesn't divide cleanly */
		goto nvi_irq_thread_exit;

	ts = st->fifo_ts;
	delay = st->smplrt_delay_us[DEV_ACCEL] * 1000;
	samples = (fifo_count / fifo_sample_size);
	if (st->dbg & NVI_DBG_SPEW_FIFO)
		dev_info(&st->i2c->dev,
			 "fifo_count=%u sample_size=%u offset=%u samples=%u\n",
			 fifo_count, fifo_sample_size, fifo_align, samples);
	fifo_rd_n = 0;
	buf_index = 0;
	while (samples) {
		if (buf_index >= fifo_rd_n) {
			fifo_rd_n = sizeof(st->buf);
			fifo_rd_n -= fifo_align;
			fifo_rd_n /= fifo_sample_size;
			if (samples < fifo_rd_n)
				fifo_rd_n = samples;
			fifo_rd_n *= fifo_sample_size;
			fifo_rd_n += fifo_align;
			ret = nvi_i2c_rd(st, st->hal->reg->fifo_r_w.bank,
					 st->hal->reg->fifo_r_w.reg,
					 fifo_rd_n, st->buf);
			if (ret)
				goto nvi_irq_thread_exit;

			buf_index = fifo_align;
		}

		if (ts_len) {
			len = ts_len;
			for (i = 0; i < len; i++) {
				ret = kfifo_out_peek(&st->timestamps,
						     &ts_irq, 1);
				if (ret != 1)
					goto nvi_irq_thread_exit_reset;

				if (ts < (ts_irq - delay))
					break;

				ret = kfifo_out(&st->timestamps,
						&ts_irq, 1);
				if (ret != 1)
					goto nvi_irq_thread_exit_reset;

				ts_len--;
				if (ts < (ts_irq + delay)) {
					ts = ts_irq;
					break;
				}
			}
			if ((st->dbg & NVI_DBG_SPEW_FIFO) && (ts != ts_irq))
				dev_info(&st->i2c->dev,
					 "%s TS=%lld != IRQ=%lld s=%u i=%u\n",
					__func__, ts, ts_irq, samples, ts_len);
		} else {
			if (st->dbg & NVI_DBG_SPEW_FIFO)
				dev_info(&st->i2c->dev,
					 "%s NO IRQ_TS TS=%lld s=%u\n",
					 __func__, ts, samples);
		}

		for (i = 0; i < st->hal->fifo_read_n; i++)
			buf_index = st->hal->fifo_read[i](st, buf_index, ts);
		nvi_buf_push(indio_dev, ts);
		if (st->flush)
			ts_irq = 0;
		else
			ts_irq = ts;
		for (i = 0; i < AUX_PORT_IO; i++) {
			ap = &st->aux.port[i];
			if (ap->fifo_en &&
				      (st->rc.i2c_slv_ctrl[i] & BIT_SLV_EN)) {
				len = ap->nmp.ctrl & BITS_I2C_SLV_CTRL_LEN;
				if (ap->nmp.handler != NULL)
					ap->nmp.handler(&st->buf[buf_index],
							len, ts_irq,
							ap->nmp.ext_driver);
				buf_index += len;
			}
		}
		st->flush = false;
		ts += delay;
		samples--;
	}
	if (ts_len) {
		if (st->dbg & NVI_DBG_SPEW_FIFO)
			dev_info(&st->i2c->dev, "%s SYNC TO IRQ_TS %lld\n",
				 __func__, ts);
		for (i = 0; i < ts_len; i++) {
			ret = kfifo_out(&st->timestamps, &ts, 1);
			if (ret != 1)
				goto nvi_irq_thread_exit_reset;
		}
	}

	st->fifo_ts = ts;
nvi_irq_thread_exit:
	if (st->irq_dis) {
		enable_irq(st->i2c->irq);
		st->irq_dis = false;
	}
	mutex_unlock(&indio_dev->mlock);
	return IRQ_HANDLED;

nvi_irq_thread_exit_reset:
	if (st->dbg & NVI_DBG_SPEW_FIFO)
		dev_info(&st->i2c->dev,
			 "%s_exit_reset fifo_count=%u fifo_sample_size=%u\n",
			 __func__, fifo_count, fifo_sample_size);
	nvi_reset(st, true, false);
	mutex_unlock(&indio_dev->mlock);
	return IRQ_HANDLED;
}

static irqreturn_t nvi_irq_handler(int irq, void *dev_id)
{
	struct nvi_state *st = (struct nvi_state *)dev_id;
	s64 ts = 0;

	if (!(st->master_enable & (1 << DEV_DMP))) {
		ts = nvi_get_time_ns(st);
		kfifo_in_spinlocked(&st->timestamps, &ts, 1,
				    &st->time_stamp_lock);
		if (kfifo_is_full(&st->timestamps)) {
			disable_irq_nosync(st->i2c->irq);
			st->irq_dis = true;
		}
	}
	if (st->dbg & NVI_DBG_SPEW_IRQ)
		dev_info(&st->i2c->dev, "%s %lld\n", __func__, ts);
	return IRQ_WAKE_THREAD;
}

static ssize_t nvi_attr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
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
	case NVI_ATTR_ENABLE:
		msg = "ATTR_ENABLE";
		old = st->master_enable;
		if (new & (1 << DEV_MASTER))
			st->master_enable |= new;
		else if (new)
			st->master_enable &= ~new;
		ret = nvi_enable(indio_dev);
		break;

	case NVI_ATTR_FIFO_RSRV_EVNT_CNT:
		msg = "ATTR_FIFO_RSRV_EVNT_CNT";
		old = st->chip_config.fifo_reserve;
		st->chip_config.fifo_reserve = new;
		break;

	case NVI_ATTR_FIFO_MAX_EVNT_CNT:
		msg = "ATTR_FIFO_MAX_EVNT_CNT";
		old = st->chip_config.fifo_max;
		st->chip_config.fifo_max = new;
		break;

	case INV_ATTR_SELF_TEST:
		msg = "ATTR_SELF_TEST";
		old = !!(st->master_enable & (1 << EN_SELF_TEST));
		if (new)
			st->master_enable |= (1 << EN_SELF_TEST);
		else
			st->master_enable &= ~(1 << EN_SELF_TEST);
		break;

	default:
		msg = "ATTR_UNKNOWN";
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & NVI_DBG_SPEW_MSG) {
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

static ssize_t nvi_attr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret;

	switch (this_attr->address) {
	case NVI_ATTR_ENABLE:
		return sprintf(buf, "%x\n", st->master_enable);

	case NVI_ATTR_PART:
		return sprintf(buf, "%s\n", st->hal->part_name);

	case NVI_ATTR_VENDOR:
		return sprintf(buf, "%s\n", NVI_VENDOR);

	case NVI_ATTR_FIFO_RSRV_EVNT_CNT:
		return sprintf(buf, "%u\n", st->chip_config.fifo_reserve);

	case NVI_ATTR_FIFO_MAX_EVNT_CNT:
		return sprintf(buf, "%u\n", st->chip_config.fifo_max);

	case NVI_ATTR_ACCEL_PART:
		return sprintf(buf, "%s accelerometer\n", st->hal->part_name);

	case NVI_ATTR_ACCEL_VERSION:
		return sprintf(buf, "%d\n", st->hal->dev[DEV_ACCEL]->version);

	case NVI_ATTR_ACCEL_MILLIAMP:
		return sprintf(buf, "%d.%06u\n",
			       st->hal->dev[DEV_ACCEL]->milliamp.ival,
			       st->hal->dev[DEV_ACCEL]->milliamp.fval);

	case NVI_ATTR_ANGLVEL_PART:
		return sprintf(buf, "%s gyro\n", st->hal->part_name);

	case NVI_ATTR_ANGLVEL_VERSION:
		return sprintf(buf, "%d\n",
			       st->hal->dev[DEV_ANGLVEL]->version);

	case NVI_ATTR_ANGLVEL_MILLIAMP:
		return sprintf(buf, "%d.%06u\n",
			       st->hal->dev[DEV_ANGLVEL]->milliamp.ival,
			       st->hal->dev[DEV_ANGLVEL]->milliamp.fval);

	case NVI_ATTR_TEMP_PART:
		return sprintf(buf, "%s temperature\n", st->hal->part_name);

	case NVI_ATTR_TEMP_VERSION:
		return sprintf(buf, "%d\n", st->hal->dev[DEV_TEMP]->version);

	case NVI_ATTR_TEMP_MILLIAMP:
		return sprintf(buf, "%d.%06u\n",
			       st->hal->dev[DEV_TEMP]->milliamp.ival,
			       st->hal->dev[DEV_TEMP]->milliamp.fval);

	case INV_ATTR_SELF_TEST:
		mutex_lock(&indio_dev->mlock);
		ret = inv_hw_self_test(indio_dev);
		mutex_unlock(&indio_dev->mlock);
		return sprintf(buf, "%d\n", ret);

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static ssize_t nvi_data_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	unsigned int info;
	int ret;

	ret = kstrtouint(buf, 10, &info);
	if ((ret) || (info >= NVI_INFO_LIMIT_MAX))
		return -EINVAL;

	st->info = info;
	switch (info) {
	case NVI_INFO_DATA:
		st->dbg = 0;
		break;

	case NVI_INFO_DBG:
		st->dbg ^= NVI_DBG_SPEW_MSG;
		break;

	case NVI_INFO_AUX_SPEW:
		st->dbg ^= NVI_DBG_SPEW_AUX;
		nvi_aux_dbg(st, "SNAPSHOT", 0);
		break;

	case NVI_INFO_ANGLVEL_SPEW:
		st->dbg ^= NVI_DBG_SPEW_ANGLVEL;
		break;

	case NVI_INFO_TEMP_SPEW:
		st->dbg ^= NVI_DBG_SPEW_TEMP;
		break;

	case NVI_INFO_ACCEL_SPEW:
		st->dbg ^= NVI_DBG_SPEW_ACCEL;
		break;

	case NVI_INFO_ACCEL_UC_SPEW:
		st->dbg ^= NVI_DBG_SPEW_ACCEL_UC;
		break;

	case NVI_INFO_FIFO_SPEW:
		st->dbg ^= NVI_DBG_SPEW_FIFO;
		break;

	case NVI_INFO_FIFO_BUF:
		st->dbg ^= NVI_DBG_SPEW_BUF;
		break;

	case NVI_INFO_FIFO_IRQ:
		st->dbg ^= NVI_DBG_SPEW_IRQ;
		break;

	default:
		break;
	}

	return count;
}

static ssize_t nvi_data_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	enum NVI_INFO info;
	ssize_t t;
	u8 data;
	unsigned int i;
	unsigned int j;
	int ret;

	info = st->info;
	st->info = NVI_INFO_DATA;
	switch (info) {
	case NVI_INFO_DATA:
		t = sprintf(buf, "ACCEL_UC_XYZ: %hd %hd %hd\n",
			    st->accel_uc[AXIS_X],
			    st->accel_uc[AXIS_Y],
			    st->accel_uc[AXIS_Z]);
		t += sprintf(buf + t, "ACCEL_XYZ: %hd %hd %hd\n",
			     st->accel[AXIS_X],
			     st->accel[AXIS_Y],
			     st->accel[AXIS_Z]);
		t += sprintf(buf + t, "ANGLVEL_XYZ: %hd %hd %hd\n",
			     st->anglvel[AXIS_X],
			     st->anglvel[AXIS_Y],
			     st->anglvel[AXIS_Z]);
		t += sprintf(buf + t, "TEMP: %hd\n", st->temp);
		t += sprintf(buf + t, "TIMESTAMP: %lld\n", st->ts);
		return t;

	case NVI_INFO_VER:
		return sprintf(buf, "version=%u\n", NVI_DRIVER_VERSION);

	case NVI_INFO_ERRS:
		i = st->errs;
		st->errs = 0;
		return sprintf(buf, "error count=%u\n", i);

	case NVI_INFO_RESET:
		mutex_lock(&indio_dev->mlock);
		ret = nvi_pm(st, NVI_PM_ON);
		ret |= nvi_wr_pwr_mgmt_1(st, BIT_H_RESET);
		ret |= nvi_enable(indio_dev);
		mutex_unlock(&indio_dev->mlock);
		if (ret)
			return sprintf(buf, "reset ERR\n");
		else
			return sprintf(buf, "reset done\n");

	case NVI_INFO_REGS:
		t = sprintf(buf, "mutex_lock=%d (< 1 locked)\n",
			    indio_dev->mlock.count.counter);
		mutex_lock(&indio_dev->mlock);
		t += sprintf(buf + t, "registers: (only data != 0 shown)\n");
		for (j = 0; j < st->hal->reg_bank_n; j++) {
			t += sprintf(buf + t, "bank %u:\n", j);
			for (i = 0; i < st->hal->regs_n; i++) {
				if ((j == st->hal->reg->fifo_r_w.bank) &&
					     (i == st->hal->reg->fifo_r_w.reg))
					data = 0;
				else
					nvi_i2c_rd(st, j, i, 1, &data);
				if (data)
					t += sprintf(buf + t,
						     "%#2x=%#2x\n", i, data);
			}
		}
		mutex_unlock(&indio_dev->mlock);
		return t;

	case NVI_INFO_DBG:
		return sprintf(buf, "DBG spew=%x\n",
			       !!(st->dbg & NVI_DBG_SPEW_MSG));

	case NVI_INFO_AUX_SPEW:
		return sprintf(buf, "AUX spew=%x\n",
			       !!(st->dbg & NVI_DBG_SPEW_AUX));

	case NVI_INFO_ANGLVEL_SPEW:
		return sprintf(buf, "anglvel_xyz_ts spew=%x\n",
			       !!(st->dbg & NVI_DBG_SPEW_ANGLVEL));

	case NVI_INFO_TEMP_SPEW:
		return sprintf(buf, "temp_ts spew=%x\n",
			       !!(st->dbg & NVI_DBG_SPEW_TEMP));

	case NVI_INFO_ACCEL_SPEW:
		return sprintf(buf, "accel_xyz_ts spew=%x\n",
			       !!(st->dbg & NVI_DBG_SPEW_ACCEL));

	case NVI_INFO_ACCEL_UC_SPEW:
		return sprintf(buf, "accel_xyz_uncalibrated_ts spew=%x\n",
			       !!(st->dbg & NVI_DBG_SPEW_ACCEL_UC));

	case NVI_INFO_FIFO_SPEW:
		return sprintf(buf, "FIFO spew=%x\n",
			       !!(st->dbg & NVI_DBG_SPEW_FIFO));

	case NVI_INFO_FIFO_BUF:
		return sprintf(buf, "BUF spew=%x\n",
			       !!(st->dbg & NVI_DBG_SPEW_BUF));

	case NVI_INFO_FIFO_IRQ:
		return sprintf(buf, "IRQ spew=%x\n",
			       !!(st->dbg & NVI_DBG_SPEW_IRQ));

	default:
		break;
	}

	return -EINVAL;
}

static ssize_t nvi_matrix_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	signed char *m;

	m = st->pdata.orientation;
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		       m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}

#ifdef NVI_I2C_DEBUG_INTERFACE
static ssize_t nvi_dbg_i2c_addr_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	u16 dbg_i2c_addr;

	if (kstrtou16(buf, 16, &dbg_i2c_addr))
		return -EINVAL;

	st->dbg_i2c_addr = dbg_i2c_addr;
	return count;
}

static ssize_t nvi_dbg_i2c_addr_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	u16 dbg_i2c_addr;

	if (st->dbg_i2c_addr)
		dbg_i2c_addr = st->dbg_i2c_addr;
	else
		dbg_i2c_addr = st->i2c->addr;
	return sprintf(buf, "%#2x\n", dbg_i2c_addr);
}

static ssize_t nvi_dbg_bank_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	u8 dbg_bank;

	if (kstrtou8(buf, 16, &dbg_bank))
		return -EINVAL;

	st->dbg_bank = dbg_bank;
	return count;
}

static ssize_t nvi_dbg_bank_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%#2x\n", st->dbg_bank);
}

static ssize_t nvi_dbg_reg_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	u8 dbg_reg;

	if (kstrtou8(buf, 16, &dbg_reg))
		return -EINVAL;

	st->dbg_reg = dbg_reg;
	return count;
}

static ssize_t nvi_dbg_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%#2x\n", st->dbg_reg);
}

static ssize_t nvi_dbg_dat_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	u16 dbg_i2c_addr;
	u8 data[2];
	int ret;

	ret = kstrtou8(buf, 16, &data[1]);
	if (ret)
		return -EINVAL;

	if (st->dbg_i2c_addr)
		dbg_i2c_addr = st->dbg_i2c_addr;
	else
		dbg_i2c_addr = st->i2c->addr;
	data[0] = st->dbg_reg;
	ret = nvi_wr_reg_bank_sel(st, st->dbg_bank);
	ret |= nvi_i2c_write(st, dbg_i2c_addr, sizeof(data), data);
	pr_info("%s dev=%x bank=%x reg=%x data=%x ret=%d\n",
		__func__, dbg_i2c_addr, st->dbg_bank, data[0], data[1], ret);
	return count;
}

static ssize_t nvi_dbg_dat_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvi_state *st = iio_priv(indio_dev);
	u16 dbg_i2c_addr;
	u8 dbg_dat = 0;
	int ret;

	if (st->dbg_i2c_addr)
		dbg_i2c_addr = st->dbg_i2c_addr;
	else
		dbg_i2c_addr = st->i2c->addr;
	ret = nvi_wr_reg_bank_sel(st, st->dbg_bank);
	ret |= nvi_i2c_read(st, dbg_i2c_addr, st->dbg_reg, 1, &dbg_dat);
	return sprintf(buf + ret, "%s dev=%x bank=%x reg=%x data=%x ret=%d\n",
		       __func__, dbg_i2c_addr, st->dbg_bank, st->dbg_reg,
		       dbg_dat, ret);
}

static DEVICE_ATTR(dbg_i2c_addr, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvi_dbg_i2c_addr_show, nvi_dbg_i2c_addr_store);
static DEVICE_ATTR(dbg_bank, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvi_dbg_bank_show, nvi_dbg_bank_store);
static DEVICE_ATTR(dbg_reg, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvi_dbg_reg_show, nvi_dbg_reg_store);
static DEVICE_ATTR(dbg_dat, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvi_dbg_dat_show, nvi_dbg_dat_store);
#endif /* NVI_I2C_DEBUG_INTERFACE */

static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		       nvi_attr_show, nvi_attr_store, NVI_ATTR_ENABLE);
static IIO_DEVICE_ATTR(part, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_PART);

static IIO_DEVICE_ATTR(accel_part, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_ACCEL_PART);
static IIO_DEVICE_ATTR(accel_vendor, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_VENDOR);
static IIO_DEVICE_ATTR(accel_version, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_ACCEL_VERSION);
static IIO_DEVICE_ATTR(accel_milliamp, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_ACCEL_MILLIAMP);
static IIO_DEVICE_ATTR(accel_fifo_reserved_event_count,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       nvi_attr_show, nvi_attr_store,
		       NVI_ATTR_FIFO_RSRV_EVNT_CNT);
static IIO_DEVICE_ATTR(accel_fifo_max_event_count, S_IRUGO | S_IWUSR | S_IWGRP,
		       nvi_attr_show, nvi_attr_store,
		       NVI_ATTR_FIFO_MAX_EVNT_CNT);
static DEVICE_ATTR(accel_matrix, S_IRUGO,
		   nvi_matrix_show, NULL);

static IIO_DEVICE_ATTR(anglvel_part, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_ANGLVEL_PART);
static IIO_DEVICE_ATTR(anglvel_vendor, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_VENDOR);
static IIO_DEVICE_ATTR(anglvel_version, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_ANGLVEL_VERSION);
static IIO_DEVICE_ATTR(anglvel_milliamp, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_ANGLVEL_MILLIAMP);
static IIO_DEVICE_ATTR(anglvel_fifo_reserved_event_count,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       nvi_attr_show, nvi_attr_store,
		       NVI_ATTR_FIFO_RSRV_EVNT_CNT);
static IIO_DEVICE_ATTR(anglvel_fifo_max_event_count,
		       S_IRUGO | S_IWUSR | S_IWGRP,
		       nvi_attr_show, nvi_attr_store,
		       NVI_ATTR_FIFO_MAX_EVNT_CNT);
static DEVICE_ATTR(anglvel_matrix, S_IRUGO,
		   nvi_matrix_show, NULL);

static IIO_DEVICE_ATTR(temp_part, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_TEMP_PART);
static IIO_DEVICE_ATTR(temp_vendor, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_VENDOR);
static IIO_DEVICE_ATTR(temp_version, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_TEMP_VERSION);
static IIO_DEVICE_ATTR(temp_milliamp, S_IRUGO,
		       nvi_attr_show, NULL, NVI_ATTR_TEMP_MILLIAMP);

static IIO_DEVICE_ATTR(self_test, S_IRUGO | S_IWUSR | S_IWGRP,
		       nvi_attr_show, nvi_attr_store, INV_ATTR_SELF_TEST);

static DEVICE_ATTR(data, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvi_data_show, nvi_data_store);

static struct attribute *nvi_attrs[] = {
	&iio_dev_attr_enable.dev_attr.attr,
	&iio_dev_attr_part.dev_attr.attr,

	&iio_dev_attr_accel_part.dev_attr.attr,
	&iio_dev_attr_accel_vendor.dev_attr.attr,
	&iio_dev_attr_accel_version.dev_attr.attr,
	&iio_dev_attr_accel_milliamp.dev_attr.attr,
	&iio_dev_attr_accel_fifo_reserved_event_count.dev_attr.attr,
	&iio_dev_attr_accel_fifo_max_event_count.dev_attr.attr,
	&dev_attr_accel_matrix.attr,

	&iio_dev_attr_anglvel_part.dev_attr.attr,
	&iio_dev_attr_anglvel_vendor.dev_attr.attr,
	&iio_dev_attr_anglvel_version.dev_attr.attr,
	&iio_dev_attr_anglvel_milliamp.dev_attr.attr,
	&iio_dev_attr_anglvel_fifo_reserved_event_count.dev_attr.attr,
	&iio_dev_attr_anglvel_fifo_max_event_count.dev_attr.attr,
	&dev_attr_anglvel_matrix.attr,

	&iio_dev_attr_temp_part.dev_attr.attr,
	&iio_dev_attr_temp_vendor.dev_attr.attr,
	&iio_dev_attr_temp_version.dev_attr.attr,
	&iio_dev_attr_temp_milliamp.dev_attr.attr,

	&iio_dev_attr_self_test.dev_attr.attr,

	&dev_attr_data.attr,

#ifdef NVI_I2C_DEBUG_INTERFACE
	&dev_attr_dbg_i2c_addr.attr,
	&dev_attr_dbg_bank.attr,
	&dev_attr_dbg_reg.attr,
	&dev_attr_dbg_dat.attr,
#endif /* NVI_I2C_DEBUG_INTERFACE */
	NULL
};

static struct attribute_group nvi_attr_group = {
	.name = NVI_NAME,
	.attrs = nvi_attrs
};

static int nvi_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct nvi_state *st = iio_priv(indio_dev);
	int i;
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_ACCEL:
			*val = st->accel[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;

		case IIO_ANGL_VEL:
			*val = st->anglvel[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;

		case IIO_TEMP:
			if (st->pm >= NVI_PM_ON)
				nvi_temp_read(st, 0);
			*val = st->temp;
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_BATCH_FLUSH:
		*val = (int)st->flush;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_BATCH_PERIOD:
		switch (chan->type) {
		case IIO_ACCEL:
			i = DEV_ACCEL;
			break;

		case IIO_ANGL_VEL:
			i = DEV_ANGLVEL;
			break;

		case IIO_TEMP:
			i = DEV_TEMP;
			break;

		default:
			return -EINVAL;
		}
		if (st->master_enable & (1 << i))
			*val = st->delay_us[i];
		else
			*val = st->hal->smplrt[i]->delay_us_min;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_BATCH_TIMEOUT:
		switch (chan->type) {
		case IIO_ACCEL:
			i = DEV_ACCEL;
			break;

		case IIO_ANGL_VEL:
			i = DEV_ANGLVEL;
			break;

		case IIO_TEMP:
			i = DEV_TEMP;
			break;

		default:
			return -EINVAL;
		}
		*val = st->batch_timeout_us[i];
		if (st->master_enable & (1 << i))
			*val = st->batch_timeout_us[i];
		else
			*val = st->hal->smplrt[i]->delay_us_max;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_BATCH_FLAGS:
		switch (chan->type) {
		case IIO_ACCEL:
			i = DEV_ACCEL;
			break;

		case IIO_ANGL_VEL:
			i = DEV_ANGLVEL;
			break;

		case IIO_TEMP:
			i = DEV_TEMP;
			break;

		default:
			return -EINVAL;
		}
		*val = st->batch_flags[i];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			*val = st->hal->dev[DEV_ACCEL]->scale.ival;
			*val2 = st->hal->dev[DEV_ACCEL]->scale.fval;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_ANGL_VEL:
			*val = st->hal->dev[DEV_ANGLVEL]->scale.ival;
			*val2 = st->hal->dev[DEV_ANGLVEL]->scale.fval;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_TEMP:
			*val = st->hal->dev[DEV_TEMP]->scale.ival;
			*val2 = st->hal->dev[DEV_TEMP]->scale.fval;
			return IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_ACCEL:
			/* offset is handled in HW */
			*val = 0;
			*val2 = 0;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_ANGL_VEL:
			/* offset is handled in HW */
			*val = 0;
			*val2 = 0;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_TEMP:
			*val = st->hal->dev[DEV_TEMP]->offset.ival;
			*val2 = st->hal->dev[DEV_TEMP]->offset.fval;
			return IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_PEAK:
		switch (chan->type) {
		case IIO_ACCEL:
			if ((st->master_enable & (1 << EN_SELF_TEST))) {
				*val = st->hal->dev[DEV_ACCEL]->selftest_scale;
				return IIO_VAL_INT;
			} else {
				i = st->chip_config.accel_fs;
				*val = st->hal->dev[DEV_ACCEL]->
							  rr[i].max_range.ival;
				*val2 = st->hal->dev[DEV_ACCEL]->
							 rr[i].max_range.fval;
				return IIO_VAL_INT_PLUS_MICRO;
			}

		case IIO_ANGL_VEL:
			if ((st->master_enable & (1 << EN_SELF_TEST))) {
				*val = st->hal->dev[DEV_ANGLVEL]->
								selftest_scale;
				return IIO_VAL_INT;
			} else {
				i = st->chip_config.fsr;
				*val = st->hal->dev[DEV_ANGLVEL]->
							  rr[i].max_range.ival;
				*val2 = st->hal->dev[DEV_ANGLVEL]->
							 rr[i].max_range.fval;
				return IIO_VAL_INT_PLUS_MICRO;
			}

		case IIO_TEMP:
			*val = st->hal->dev[DEV_TEMP]->rr->max_range.ival;
			*val2 = st->hal->dev[DEV_TEMP]->rr->max_range.fval;
			return IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_PEAK_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			i = st->chip_config.accel_fs;
			*val = st->hal->dev[DEV_ACCEL]->rr[i].resolution.ival;
			*val2 = st->hal->dev[DEV_ACCEL]->
							rr[i].resolution.fval;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_ANGL_VEL:
			i = st->chip_config.fsr;
			*val = st->hal->dev[DEV_ANGLVEL]->
							 rr[i].resolution.ival;
			*val2 = st->hal->dev[DEV_ANGLVEL]->
							rr[i].resolution.fval;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_TEMP:
			*val = st->hal->dev[DEV_TEMP]->rr->resolution.ival;
			*val2 = st->hal->dev[DEV_TEMP]->rr->resolution.fval;
			return IIO_VAL_INT_PLUS_MICRO;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_CALIBBIAS:
		i = chan->channel2 - IIO_MOD_X;
		switch (chan->type) {
		case IIO_ACCEL:
			*val = st->accel_bias[i] * st->chip_info.multi;
			return IIO_VAL_INT;

		case IIO_ANGL_VEL:
			*val = st->gyro_bias[i];
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}


	default:
		return -EINVAL;
	}

	return ret;
}

static int nvi_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask)
{
	struct nvi_state *st = iio_priv(indio_dev);
	const char *msg;
	unsigned int dev;
	unsigned int axis;
	int old = 0;
	int old2 = 0;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (st->shutdown || st->suspend) {
		mutex_unlock(&indio_dev->mlock);
		return -EPERM;
	}

	switch (mask) {
	case IIO_CHAN_INFO_BATCH_FLUSH:
		msg = "IIO_CHAN_INFO_BATCH_FLUSH";
		switch (chan->type) {
		case IIO_ACCEL:
			dev = DEV_ACCEL;
			break;

		case IIO_ANGL_VEL:
			dev = DEV_ANGLVEL;
			break;

		case IIO_TEMP:
			dev = DEV_TEMP;
			break;

		default:
			ret = -EINVAL;
		}
		if (!ret) {
			if (st->enable[dev])
				ret = nvi_enable(indio_dev);
		}
		break;

	case IIO_CHAN_INFO_BATCH_PERIOD:
		msg = "IIO_CHAN_INFO_BATCH_PERIOD";
		switch (chan->type) {
		case IIO_ACCEL:
			dev = DEV_ACCEL;
			break;

		case IIO_ANGL_VEL:
			dev = DEV_ANGLVEL;
			break;

		case IIO_TEMP:
			dev = DEV_TEMP;
			break;

		default:
			ret = -EINVAL;
		}
		if (!ret) {
			if (val < st->hal->smplrt[dev]->delay_us_min)
				val = st->hal->smplrt[dev]->delay_us_min;
			if (val > st->hal->smplrt[dev]->delay_us_max)
				val = st->hal->smplrt[dev]->delay_us_max;
			old = st->delay_us[dev];
			st->delay_us[dev] = val;
			if (st->enable[dev]) {
				ret = nvi_enable(indio_dev);
				if (ret)
					st->delay_us[dev] = old;
			}
		}
		break;

	case IIO_CHAN_INFO_BATCH_TIMEOUT:
		msg = "IIO_CHAN_INFO_BATCH_TIMEOUT";
		switch (chan->type) {
		case IIO_ACCEL:
			dev = DEV_ACCEL;
			break;

		case IIO_ANGL_VEL:
			dev = DEV_ANGLVEL;
			break;

		case IIO_TEMP:
			dev = DEV_TEMP;
			break;

		default:
			ret = -EINVAL;
		}
		if (!ret) {
			old = st->batch_timeout_us[dev];
			st->batch_timeout_us[dev] = val;
		}
		break;

	case IIO_CHAN_INFO_BATCH_FLAGS:
		msg = "IIO_CHAN_INFO_BATCH_FLAGS";
		switch (chan->type) {
		case IIO_ACCEL:
			dev = DEV_ACCEL;
			break;

		case IIO_ANGL_VEL:
			dev = DEV_ANGLVEL;
			break;

		case IIO_TEMP:
			dev = DEV_TEMP;
			break;

		default:
			ret = -EINVAL;
		}
		if (!ret) {
			old = st->batch_flags[dev];
			st->batch_flags[dev] = val;
		}
		break;

	case IIO_CHAN_INFO_PEAK:
		msg = "IIO_CHAN_INFO_PEAK";
		switch (chan->type) {
		case IIO_ACCEL:
			old = st->chip_config.accel_fs;
			st->chip_config.accel_fs = val;
			if (st->master_enable & (1 << DEV_ACCEL)) {
				ret = nvi_enable(indio_dev);
				if (ret)
					st->chip_config.accel_fs = old;
			}
			break;

		case IIO_ANGL_VEL:
			old = st->chip_config.fsr;
			st->chip_config.fsr = val;
			if (st->master_enable & (1 << DEV_ANGLVEL)) {
				ret = nvi_enable(indio_dev);
				if (ret)
					st->chip_config.fsr = old;
			}
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case IIO_CHAN_INFO_OFFSET:
		msg = "IIO_CHAN_INFO_OFFSET";
		switch (chan->type) {
		case IIO_ACCEL:
			axis = chan->channel2 - IIO_MOD_X;
			old = st->input_accel_offset[axis];
			st->input_accel_offset[axis] = val;
			if (st->master_enable & (1 << DEV_ACCEL)) {
				ret = nvi_enable(indio_dev);
				if (ret)
					st->input_accel_offset[axis] = old;
			}
			break;

		case IIO_ANGL_VEL:
			axis = chan->channel2 - IIO_MOD_X;
			old = st->input_gyro_offset[axis];
			st->input_gyro_offset[axis] = val;
			if (st->master_enable & (1 << DEV_ANGLVEL)) {
				ret = nvi_enable(indio_dev);
				if (ret)
					st->input_gyro_offset[axis] = old;
			}
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case IIO_CHAN_INFO_RAW:
		msg = "IIO_CHAN_INFO_RAW";
		switch (chan->type) {
		case IIO_ACCEL:
			axis = chan->channel2 - IIO_MOD_X;
			old = st->accel[axis];
			st->accel[axis] = val;
			st->dbg |= (NVI_DBG_ACCEL_AXIS_X << axis);
			break;

		case IIO_ANGL_VEL:
			axis = chan->channel2 - IIO_MOD_X;
			old = st->anglvel[axis];
			st->anglvel[chan->channel2 - IIO_MOD_X] = val;
			st->dbg |= (NVI_DBG_ANGLVEL_AXIS_X << axis);
			break;

		case IIO_TEMP:
			old = st->temp;
			st->temp = val;
			st->dbg |= NVI_DBG_TEMP_VAL;
			break;

		default:
			return -EINVAL;
		}

	default:
		msg = "IIO_CHAN_INFO_UNKNOWN";
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & NVI_DBG_SPEW_MSG) {
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

static const struct iio_info nvi_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &nvi_attr_group,
	.read_raw = &nvi_read_raw,
	.write_raw = &nvi_write_raw,
};

static const struct iio_chan_spec nvi_channels[] = {
	{
		.type			= IIO_ACCEL,
		.channel2		= IIO_MOD_X,
		.scan_index		= NVI_SCAN_ACCEL_X,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_CALIBBIAS) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					    BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					    BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					    BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE),
		.modified		= 1,
	},
	{
		.type			= IIO_ACCEL,
		.channel2		= IIO_MOD_Y,
		.scan_index		= NVI_SCAN_ACCEL_Y,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_CALIBBIAS) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					    BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					    BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					    BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE),
		.modified		= 1,
	},
	{
		.type			= IIO_ACCEL,
		.channel2		= IIO_MOD_Z,
		.scan_index		= NVI_SCAN_ACCEL_Z,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_CALIBBIAS) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					    BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					    BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					    BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE),
		.modified		= 1,
	},
	{
		.type			= IIO_ANGL_VEL,
		.channel2		= IIO_MOD_X,
		.scan_index		= NVI_SCAN_ANGLVEL_X,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_CALIBBIAS) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					    BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					    BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					    BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE),
		.modified		= 1,
	},
	{
		.type			= IIO_ANGL_VEL,
		.channel2		= IIO_MOD_Y,
		.scan_index		= NVI_SCAN_ANGLVEL_Y,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_CALIBBIAS) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					    BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					    BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					    BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE),
		.modified		= 1,
	},
	{
		.type			= IIO_ANGL_VEL,
		.channel2		= IIO_MOD_Z,
		.scan_index		= NVI_SCAN_ANGLVEL_Z,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_CALIBBIAS) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					    BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					    BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					    BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE),
		.modified		= 1,
	},
	{
		.type			= IIO_TEMP,
		.scan_index		= NVI_SCAN_TEMP,
		.scan_type		= IIO_ST('s', 16, 16, 0),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					    BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					    BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					    BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					    BIT(IIO_CHAN_INFO_PEAK) |
					    BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					    BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_OFFSET),
	},
	IIO_CHAN_SOFT_TIMESTAMP(NVI_SCAN_TIMESTAMP)
};

static int nvi_buffer_preenable(struct iio_dev *indio_dev)
{
	struct nvi_state *st = iio_priv(indio_dev);

	if (st->shutdown || st->suspend)
		return -EINVAL;

	return 0;
}

static int nvi_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;

	ret = nvi_enable(indio_dev);
	/* never return > 0 to IIO buffer engine */
	if (ret > 0)
		ret = 0;
	return ret;
}

static const struct iio_buffer_setup_ops nvi_buffer_setup_ops = {
	/* iio_sw_buffer_preenable:
	 * Generic function for equal sized ring elements + 64 bit timestamp
	 * Assumes that any combination of channels can be enabled.
	 * Typically replaced to implement restrictions on what combinations
	 * can be captured (hardware scan modes).
	 */
	.preenable = &nvi_buffer_preenable,
	/* iio_triggered_buffer_postenable:
	 * Generic function that simply attaches the pollfunc to the trigger.
	 * Replace this to mess with hardware state before we attach the
	 * trigger.
	 */
	.postenable = &nvi_buffer_postenable,
	/* this driver relies on the NVS HAL to power off this device with the
	 * master enable.
	 *.predisable = N/A
	 *.postdisable = N/A
	 */
};

static const struct iio_trigger_ops nvi_trigger_ops = {
	.owner = THIS_MODULE,
};

static const unsigned int nvi_lpf_us_tbl_6050[] = {
	0, /* WAR: disabled 3906, 256Hz */
	5319,	/* 188Hz */
	10204,	/* 98Hz */
	23810,	/* 42Hz */
	50000,	/* 20Hz */
	100000,	/* 10Hz */
	/* 200000, 5Hz */
};

static const unsigned long nvi_lpa_delay_us_tbl_6050[] = {
	800000,	/* 800ms */
	200000,	/* 200ms */
	50000,	/* 50ms */
	/* 25000, 25ms */
};

static const unsigned int smplrt_6050_dev_delays[] = {
	DEV_ACCEL,
	DEV_ANGLVEL,
	DEV_TEMP,
	DEV_AUX,
};

static const struct nvi_smplrt smplrt_6050 = {
	.dev				= DEV_ACCEL,
	.delay_us_min			= 10000,
	.delay_us_max			= 256000,
	.delay_us_dflt			= 100000,
	.dev_delays_n			= ARRAY_SIZE(smplrt_6050_dev_delays),
	.dev_delays			= smplrt_6050_dev_delays,
	.lpf_us_tbl_n			= ARRAY_SIZE(nvi_lpf_us_tbl_6050),
	.lpf_us_tbl			= nvi_lpf_us_tbl_6050,
	.base_hz			= 1000,
	.lpf_wr				= nvi_wr_gyro_config,
};

static struct nvi_rr nvi_rr_accel[] = {
	/* all accelerometer values are in g's */
	{
		.max_range		= {
			.ival		= 19,
			.fval		= 613300,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 61,
		},
	},
	{
		.max_range		= {
			.ival		= 39,
			.fval		= 226600,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 122,
		},
	},
	{
		.max_range		= {
			.ival		= 78,
			.fval		= 453200,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 244,
		},
	},
	{
		.max_range		= {
			.ival		= 156,
			.fval		= 906400,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 488,
		},
	},
};

static struct nvi_rr nvi_rr_anglvel[] = {
	/* rad / sec */
	{
		.max_range		= {
			.ival		= 4,
			.fval		= 363323,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 133,
		},
	},
	{
		.max_range		= {
			.ival		= 8,
			.fval		= 726646,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 266,
		},
	},
	{
		.max_range		= {
			.ival		= 17,
			.fval		= 453292,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 533,
		},
	},
	{
		.max_range		= {
			.ival		= 34,
			.fval		= 906585,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 1065,
		},
	},
};

static struct nvi_rr nvi_rr_temp[] = {
	{
		.max_range		= {
			.ival		= 125,
			.fval		= 0,
		},
		.resolution		= {
			.ival		= 1,
			.fval		= 0,
		},
	},
};

static const struct nvi_hal_dev nvi_hal_6050_accel = {
	.version			= 3,
	.selftest_scale			= 8,
	.rr				= nvi_rr_accel,
	.scale				= {
		.ival			= 0,
		.fval			= 0,
	},
	.offset				= {
		.ival			= 0,
		.fval			= 0,
	},
	.milliamp			= {
		.ival			= 0,
		.fval			= 500000,
	},
};

static const struct nvi_hal_dev nvi_hal_6050_anglvel = {
	.version			= 3,
	.selftest_scale			= 250,
	.rr				= nvi_rr_anglvel,
	.scale				= {
		.ival			= 0,
		.fval			= 0,
	},
	.offset				= {
		.ival			= 0,
		.fval			= 0,
	},
	.milliamp			= {
		.ival			= 3,
		.fval			= 700000,
	},
};

static const struct nvi_hal_dev nvi_hal_6050_temp = {
	.version			= 2,
	.rr				= nvi_rr_temp,
	.scale				= {
		.ival			= 3158064,
		.fval			= 0,
	},
	.offset				= {
		.ival			= 2394184,
		.fval			= 0,
	},
	.milliamp			= {
		.ival			= 3,
		.fval			= 700000,
	},
};

static const struct nvi_hal_reg nvi_hal_reg_6050 = {
	.a_offset_h[AXIS_X]		= {
		.reg			= 0x06,
	},
	.a_offset_h[AXIS_Y]		= {
		.reg			= 0x08,
	},
	.a_offset_h[AXIS_Z]		= {
		.reg			= 0x0A,
	},
	.xg_offset_h			= {
		.reg			= 0x13,
	},
	.smplrt_div[DEV_ACCEL]		= {
		.reg			= 0x19,
	},
	.gyro_config1			= {
		.reg			= 0x1A,
	},
	.gyro_config2			= {
		.reg			= 0x1B,
	},
	.accel_config			= {
		.reg			= 0x1C,
	},
	.accel_config2			= {
		.reg			= 0x1D,
	},
	.lp_config			= {
		.reg			= 0x1E,
	},
	.fifo_en			= {
		.reg			= 0x23,
		.dflt			= 0x4000,
	},
	.i2c_mst_ctrl			= {
		.reg			= 0x24,
	},
	.i2c_slv0_addr			= {
		.reg			= 0x25,
	},
	.i2c_slv0_reg			= {
		.reg			= 0x26,
	},
	.i2c_slv0_ctrl			= {
		.reg			= 0x27,
	},
	.i2c_slv4_ctrl			= {
		.reg			= 0x34,
	},
	.i2c_slv4_di			= {
		.reg			= 0x35,
	},
	.i2c_mst_status			= {
		.reg			= 0x36,
	},
	.int_pin_cfg			= {
		.reg			= 0x37,
	},
	.int_enable			= {
		.reg			= 0x38,
	},
	.int_status			= {
		.reg			= 0x3A,
	},
	.accel_xout_h			= {
		.reg			= 0x3B,
	},
	.temp_out_h			= {
		.reg			= 0x41,
	},
	.gyro_xout_h			= {
		.reg			= 0x43,
	},
	.ext_sens_data_00		= {
		.reg			= 0x49,
	},
	.i2c_slv0_do			= {
		.reg			= 0x63,
	},
	.i2c_slv4_do			= {
		.reg			= 0x33,
	},
	.i2c_mst_odr_config		= {
		.bank			= -1,
	},
	.i2c_mst_delay_ctrl		= {
		.reg			= 0x67,
	},
	.signal_path_reset		= {
		.reg			= 0x68,
	},
	.accel_intel_ctrl		= {
		.reg			= 0x69,
	},
	.user_ctrl			= {
		.reg			= 0x6A,
	},
	.pwr_mgmt_1			= {
		.reg			= 0x6B,
	},
	.pwr_mgmt_2			= {
		.reg			= 0x6C,
	},
	.fifo_count_h			= {
		.reg			= 0x72,
	},
	.fifo_r_w			= {
		.reg			= 0x74,
	},
	.who_am_i			= {
		.reg			= 0x75,
	},
};

static const struct nvi_hal_bit nvi_hal_bit_6050 = {
	.smplrt_div_n[DEV_ACCEL]	= 8,
	.i2c_mst_int_en			= 0,
	.dmp_int_en			= 1,
	.pll_rdy_en			= 7,
	.wom_int_en			= 6,
	.reg_wof_en			= 3,
	.raw_data_0_rdy_en		= 0,
	.raw_data_1_rdy_en		= 0,
	.raw_data_2_rdy_en		= 0,
	.raw_data_3_rdy_en		= 0,
	.fifo_overflow_en		= 4,
	.fifo_wm_en			= 6,
	.bit_int_enable_max		= 8,
	.slv_fifo_en[0]			= 0,
	.slv_fifo_en[1]			= 1,
	.slv_fifo_en[2]			= 2,
	.slv_fifo_en[3]			= 13,
	.temp_fifo_en			= 7,
	.gyro_x_fifo_en			= 6,
	.gyro_y_fifo_en			= 5,
	.gyro_z_fifo_en			= 4,
	.accel_fifo_en			= 3,
	.bit_fifo_en_max		= 16,
};

static unsigned int (*fifo_read_6050[])(struct nvi_state *st,
					unsigned int buf_i, s64 ts) = {
	nvi_fifo_read_accel,
	nvi_fifo_read_temp,
	nvi_fifo_read_anglvel,
};

static const struct nvi_hal nvi_hal_6050 = {
	.part				= MPU6050,
	.part_name			= NVI_NAME_MPU6050,
	.regs_n				= 118,
	.reg_bank_n			= 1,
	.dmp				= false,
	.fifo_size			= 1024,
	.lpa_tbl			= nvi_lpa_delay_us_tbl_6050,
	.lpa_tbl_n			= ARRAY_SIZE(nvi_lpa_delay_us_tbl_6050),
	.smplrt[DEV_ACCEL]		= &smplrt_6050,
	.smplrt[DEV_ANGLVEL]		= &smplrt_6050,
	.smplrt[DEV_TEMP]		= &smplrt_6050,
	.smplrt[DEV_AUX]		= &smplrt_6050,
	.dev[DEV_ACCEL]			= &nvi_hal_6050_accel,
	.dev[DEV_ANGLVEL]		= &nvi_hal_6050_anglvel,
	.dev[DEV_TEMP]			= &nvi_hal_6050_temp,
	.reg				= &nvi_hal_reg_6050,
	.bit				= &nvi_hal_bit_6050,
	.fifo_read_n			= ARRAY_SIZE(fifo_read_6050),
	.fifo_read			= fifo_read_6050,
	.init				= inv_get_silicon_rev_mpu6050,
};

static const unsigned long nvi_lpa_delay_us_tbl_6500[] = {
	4096000,/* 4096ms */
	2048000,/* 2048ms */
	1024000,/* 1024ms */
	512000,	/* 512ms */
	256000,	/* 256ms */
	128000,	/* 128ms */
	64000,	/* 64ms */
	32000,	/* 32ms */
	16000,	/* 16ms */
	8000,	/* 8ms */
	4000,	/* 4ms */
	/* 2000, 2ms */
};

static const struct nvi_hal_dev nvi_hal_6500_accel = {
	.version			= 3,
	.selftest_scale			= 2,
	.rr				= nvi_rr_accel,
	.scale				= {
		.ival			= 0,
		.fval			= 0,
	},
	.offset				= {
		.ival			= 0,
		.fval			= 0,
	},
	.milliamp			= {
		.ival			= 0,
		.fval			= 500000,
	},
};

static const struct nvi_hal_dev nvi_hal_6500_temp = {
	.version			= 2,
	.rr				= nvi_rr_temp,
	.scale				= {
		.ival			= 3340827,
		.fval			= 0,
	},
	.offset				= {
		.ival			= 1376256,
		.fval			= 0,
	},
	.milliamp			= {
		.ival			= 3,
		.fval			= 700000,
	},
};

static const struct nvi_hal_reg nvi_hal_reg_6500 = {
	.a_offset_h[AXIS_X]		= {
		.reg			= 0x77,
	},
	.a_offset_h[AXIS_Y]		= {
		.reg			= 0x7A,
	},
	.a_offset_h[AXIS_Z]		= {
		.reg			= 0x7D,
	},
	.xg_offset_h			= {
		.reg			= 0x13,
	},
	.smplrt_div[DEV_ACCEL]		= {
		.reg			= 0x19,
	},
	.gyro_config1			= {
		.reg			= 0x1A,
	},
	.gyro_config2			= {
		.reg			= 0x1B,
	},
	.accel_config			= {
		.reg			= 0x1C,
	},
	.accel_config2			= {
		.reg			= 0x1D,
	},
	.lp_config			= {
		.reg			= 0x1E,
	},
	.fifo_en			= {
		.reg			= 0x23,
		.dflt			= 0x4000,
	},
	.i2c_mst_ctrl			= {
		.reg			= 0x24,
	},
	.i2c_slv0_addr			= {
		.reg			= 0x25,
	},
	.i2c_slv0_reg			= {
		.reg			= 0x26,
	},
	.i2c_slv0_ctrl			= {
		.reg			= 0x27,
	},
	.i2c_slv4_ctrl			= {
		.reg			= 0x34,
	},
	.i2c_slv4_di			= {
		.reg			= 0x35,
	},
	.i2c_mst_status			= {
		.reg			= 0x36,
	},
	.int_pin_cfg			= {
		.reg			= 0x37,
	},
	.int_enable			= {
		.reg			= 0x38,
	},
	.int_status			= {
		.reg			= 0x3A,
	},
	.accel_xout_h			= {
		.reg			= 0x3B,
	},
	.temp_out_h			= {
		.reg			= 0x41,
	},
	.gyro_xout_h			= {
		.reg			= 0x43,
	},
	.ext_sens_data_00		= {
		.reg			= 0x49,
	},
	.i2c_slv0_do			= {
		.reg			= 0x63,
	},
	.i2c_slv4_do			= {
		.reg			= 0x33,
	},
	.i2c_mst_odr_config		= {
		.bank			= -1,
	},
	.i2c_mst_delay_ctrl		= {
		.reg			= 0x67,
	},
	.signal_path_reset		= {
		.reg			= 0x68,
	},
	.accel_intel_ctrl		= {
		.reg			= 0x69,
	},
	.user_ctrl			= {
		.reg			= 0x6A,
	},
	.pwr_mgmt_1			= {
		.reg			= 0x6B,
	},
	.pwr_mgmt_2			= {
		.reg			= 0x6C,
	},
	.fifo_count_h			= {
		.reg			= 0x72,
	},
	.fifo_r_w			= {
		.reg			= 0x74,
	},
	.who_am_i			= {
		.reg			= 0x75,
	},
};

static const struct nvi_hal nvi_hal_6500 = {
	.part				= MPU6500,
	.part_name			= NVI_NAME_MPU6500,
	.regs_n				= 128,
	.reg_bank_n			= 1,
	.dmp				= false,
	.fifo_size			= 4096,
	.lpa_tbl			= nvi_lpa_delay_us_tbl_6500,
	.lpa_tbl_n			= ARRAY_SIZE(nvi_lpa_delay_us_tbl_6500),
	.smplrt[DEV_ACCEL]		= &smplrt_6050,
	.smplrt[DEV_ANGLVEL]		= &smplrt_6050,
	.smplrt[DEV_TEMP]		= &smplrt_6050,
	.smplrt[DEV_AUX]		= &smplrt_6050,
	.dev[DEV_ACCEL]			= &nvi_hal_6500_accel,
	.dev[DEV_ANGLVEL]		= &nvi_hal_6050_anglvel,
	.dev[DEV_TEMP]			= &nvi_hal_6500_temp,
	.reg				= &nvi_hal_reg_6500,
	.bit				= &nvi_hal_bit_6050,
	.fifo_read_n			= ARRAY_SIZE(fifo_read_6050),
	.fifo_read			= fifo_read_6050,
	.init				= inv_get_silicon_rev_mpu6500,
};

static const struct nvi_hal nvi_hal_6515 = {
	.part				= MPU6515,
	.part_name			= NVI_NAME_MPU6515,
	.regs_n				= 128,
	.reg_bank_n			= 1,
	.dmp				= true,
	.fifo_size			= 4096,
	.lpa_tbl			= nvi_lpa_delay_us_tbl_6500,
	.lpa_tbl_n			= ARRAY_SIZE(nvi_lpa_delay_us_tbl_6500),
	.smplrt[DEV_ACCEL]		= &smplrt_6050,
	.smplrt[DEV_ANGLVEL]		= &smplrt_6050,
	.smplrt[DEV_TEMP]		= &smplrt_6050,
	.smplrt[DEV_AUX]		= &smplrt_6050,
	.dev[DEV_ACCEL]			= &nvi_hal_6500_accel,
	.dev[DEV_ANGLVEL]		= &nvi_hal_6050_anglvel,
	.dev[DEV_TEMP]			= &nvi_hal_6500_temp,
	.reg				= &nvi_hal_reg_6500,
	.bit				= &nvi_hal_bit_6050,
	.fifo_read_n			= ARRAY_SIZE(fifo_read_6050),
	.fifo_read			= fifo_read_6050,
	.init				= inv_get_silicon_rev_mpu6500,
};

static const unsigned int smplrt_20628_accel_dd[] = {
	DEV_ACCEL,
	DEV_AUX,
};

static const struct nvi_smplrt smplrt_20628_accel = {
	.dev				= DEV_ACCEL,
	.delay_us_min			= 10000,
	.delay_us_max			= 256000,
	.delay_us_dflt			= 100000,
	.dev_delays_n			= ARRAY_SIZE(smplrt_20628_accel_dd),
	.dev_delays			= smplrt_20628_accel_dd,
	.base_hz			= 1125,
	.lpf_wr				= nvi_wr_accel_config,
};

static const unsigned int smplrt_20628_anglvel_dd[] = {
	DEV_ANGLVEL,
	DEV_TEMP,
};

static const struct nvi_smplrt smplrt_20628_anglvel = {
	.dev				= DEV_ANGLVEL,
	.delay_us_min			= 10000,
	.delay_us_max			= 256000,
	.delay_us_dflt			= 100000,
	.dev_delays_n			= ARRAY_SIZE(smplrt_20628_anglvel_dd),
	.dev_delays			= smplrt_20628_anglvel_dd,
	.base_hz			= 1125,
	.lpf_wr				= nvi_wr_gyro_config,
};

static const unsigned int smplrt_20628_aux_dd[] = {
	DEV_AUX,
};

static const struct nvi_smplrt smplrt_20628_aux = {
	.dev				= DEV_AUX,
	.delay_us_min			= 10000,
	.delay_us_max			= 256000,
	.delay_us_dflt			= 100000,
	.dev_delays_n			= ARRAY_SIZE(smplrt_20628_aux_dd),
	.dev_delays			= smplrt_20628_aux_dd,
	.base_hz			= 1125,
	.lpf_wr				= nvi_wr_gyro_config,
};

static const struct nvi_hal_reg nvi_hal_reg_20628 = {
/* register bank 0 */
	.lp_config			= {
		.bank			= 0,
		.reg			= 0x05,
		.dflt			= 0x70,
	},
	.fifo_en			= {
		.bank			= 0,
		.reg			= 0x66,
	},
	.i2c_mst_status			= {
		.bank			= 0,
		.reg			= 0x17,
	},
	.int_pin_cfg			= {
		.bank			= 0,
		.reg			= 0x0F,
	},
	.int_enable			= {
		.bank			= 0,
		.reg			= 0x10,
	},
	.int_status			= {
		.bank			= 0,
		.reg			= 0x19,
	},
	.accel_xout_h			= {
		.bank			= 0,
		.reg			= 0x2D,
	},
	.temp_out_h			= {
		.bank			= 0,
		.reg			= 0x39,
	},
	.gyro_xout_h			= {
		.bank			= 0,
		.reg			= 0x33,
	},
	.ext_sens_data_00		= {
		.bank			= 0,
		.reg			= 0x3B,
	},
	.signal_path_reset		= {
		.bank			= 0,
		.reg			= 0x04,
	},
	.user_ctrl			= {
		.bank			= 0,
		.reg			= 0x03,
	},
	.pwr_mgmt_1			= {
		.bank			= 0,
		.reg			= 0x06,
		.dflt			= 0x01,
	},
	.pwr_mgmt_2			= {
		.bank			= 0,
		.reg			= 0x07,
		.dflt			= 0x40,
	},
	.fifo_count_h			= {
		.bank			= 0,
		.reg			= 0x70,
	},
	.fifo_r_w			= {
		.bank			= 0,
		.reg			= 0x72,
	},
	.who_am_i			= {
		.bank			= 0,
		.reg			= 0x00,
	},
	.reg_bank_sel			= {
		.bank			= 0,
		.reg			= 0x7F,
	},
/* register bank 1 */
	.self_test_x_gyro		= {
		.bank			= 1,
		.reg			= 0x02,
	},
	.self_test_y_gyro		= {
		.bank			= 1,
		.reg			= 0x03,
	},
	.self_test_z_gyro		= {
		.bank			= 1,
		.reg			= 0x04,
	},
	.self_test_x_accel		= {
		.bank			= 1,
		.reg			= 0x0E,
	},
	.self_test_y_accel		= {
		.bank			= 1,
		.reg			= 0x0F,
	},
	.self_test_z_accel		= {
		.bank			= 1,
		.reg			= 0x10,
	},
	.a_offset_h[AXIS_X]		= {
		.bank			= 1,
		.reg			= 0x14,
	},
	.a_offset_h[AXIS_Y]		= {
		.bank			= 1,
		.reg			= 0x17,
	},
	.a_offset_h[AXIS_Z]		= {
		.bank			= 1,
		.reg			= 0x1A,
	},
/* register bank 2 */
	.smplrt_div[DEV_ANGLVEL]	= {
		.bank			= 2,
		.reg			= 0x00,
	},
	.gyro_config1			= {
		.bank			= 2,
		.reg			= 0x01,
		.dflt			= 0x01,
	},
	.gyro_config2			= {
		.bank			= 2,
		.reg			= 0x02,
	},
	.xg_offset_h			= {
		.bank			= 2,
		.reg			= 0x03,
	},
	.smplrt_div[DEV_ACCEL]		= {
		.bank			= 2,
		.reg			= 0x10,
	},
	.accel_intel_ctrl		= {
		.bank			= 2,
		.reg			= 0x12,
	},
	.accel_config			= {
		.bank			= 2,
		.reg			= 0x14,
	},
	.accel_config2			= {
		.bank			= 2,
		.reg			= 0x15,
	},
/* register bank 3 */
	.i2c_mst_odr_config		= {
		.bank			= 3,
		.reg			= 0x00,
	},
	.i2c_mst_ctrl			= {
		.bank			= 3,
		.reg			= 0x01,
	},
	.i2c_mst_delay_ctrl		= {
		.bank			= 3,
		.reg			= 0x02,
	},
	.i2c_slv0_addr			= {
		.bank			= 3,
		.reg			= 0x03,
	},
	.i2c_slv0_reg			= {
		.bank			= 3,
		.reg			= 0x04,
	},
	.i2c_slv0_ctrl			= {
		.bank			= 3,
		.reg			= 0x05,
	},
	.i2c_slv0_do			= {
		.bank			= 3,
		.reg			= 0x06,
	},
	.i2c_slv4_ctrl			= {
		.bank			= 3,
		.reg			= 0x15,
	},
	.i2c_slv4_do			= {
		.bank			= 3,
		.reg			= 0x16,
	},
	.i2c_slv4_di			= {
		.bank			= 3,
		.reg			= 0x17,
	},
};

static const struct nvi_hal_bit nvi_hal_bit_20628 = {
	.smplrt_div_n[DEV_ACCEL]	= 12,
	.smplrt_div_n[DEV_ANGLVEL]	= 8,
	.i2c_mst_int_en			= 0,
	.dmp_int_en			= 1,
	.pll_rdy_en			= 2,
	.wom_int_en			= 3,
	.reg_wof_en			= 7,
	.raw_data_0_rdy_en		= 8,
	.raw_data_1_rdy_en		= 9,
	.raw_data_2_rdy_en		= 10,
	.raw_data_3_rdy_en		= 11,
	.fifo_overflow_en		= 16,
	.fifo_wm_en			= 24,
	.bit_int_enable_max		= 32,
	.slv_fifo_en[0]			= 0,
	.slv_fifo_en[1]			= 1,
	.slv_fifo_en[2]			= 2,
	.slv_fifo_en[3]			= 3,
	.temp_fifo_en			= 8,
	.gyro_x_fifo_en			= 9,
	.gyro_y_fifo_en			= 10,
	.gyro_z_fifo_en			= 11,
	.accel_fifo_en			= 12,
	.bit_fifo_en_max		= 16,
};

static unsigned int (*fifo_read_20628[])(struct nvi_state *st,
					 unsigned int buf_i, s64 ts) = {
	nvi_fifo_read_accel,
	nvi_fifo_read_anglvel,
	nvi_fifo_read_temp,
};

static void nvi_por2rc_20628(struct nvi_state *st)
{
	st->rc.lp_config = 0x40;
	st->rc.pwr_mgmt_1 = 0x41;
	st->rc.gyro_config1 = 0x01;
	st->rc.accel_config = 0x01;
}

static const struct nvi_hal nvi_hal_20628 = {
	.part				= ICM20628,
	.part_name			= NVI_NAME_ICM20628,
	.regs_n				= 128,
	.reg_bank_n			= 4,
	.dmp				= true,
	.fifo_size			= 4096,
	.lpa_tbl			= nvi_lpa_delay_us_tbl_6500,
	.lpa_tbl_n			= ARRAY_SIZE(nvi_lpa_delay_us_tbl_6500),
	.smplrt[DEV_ACCEL]		= &smplrt_20628_accel,
	.smplrt[DEV_ANGLVEL]		= &smplrt_20628_anglvel,
	.smplrt[DEV_TEMP]		= &smplrt_20628_anglvel,
	.smplrt[DEV_AUX]		= &smplrt_20628_accel,
	.dev[DEV_ACCEL]			= &nvi_hal_6500_accel,
	.dev[DEV_ANGLVEL]		= &nvi_hal_6050_anglvel,
	.dev[DEV_TEMP]			= &nvi_hal_6500_temp,
	.reg				= &nvi_hal_reg_20628,
	.bit				= &nvi_hal_bit_20628,
	.fifo_read_n			= ARRAY_SIZE(fifo_read_20628),
	.fifo_read			= fifo_read_20628,
	.por2rc				= nvi_por2rc_20628,
	.init				= inv_icm_init,
};

static void nvi_init_config(struct nvi_state *st)
{
	unsigned int i;

	st->hal = &nvi_hal_6050;
	for (i = 0; i < DEV_N_AUX; i++) {
		st->enable[i] = 0;
		st->delay_us[i] = 0;
	}
	st->master_enable = (1 << EN_STDBY);
	st->chip_config.lpa_delay_us = -1; /* disabled */
	st->chip_config.gyro_start_delay_ns = GYRO_STARTUP_DELAY_NS;
	st->chip_config.bypass_timeout_ms = NVI_BYPASS_TIMEOUT_MS;
	st->chip_config.temp_fifo_en = 0;
	st->chip_config.fifo_thr = FIFO_THRESHOLD;
	st->chip_config.fifo_reserve = FIFO_RESERVED_EVENT_COUNT;
	st->chip_config.fifo_max = FIFO_MAX_EVENT_COUNT;

	st->chip_config.fsr = INV_FSR_2000DPS;
	st->chip_config.lpf = INV_FS_02G;

	st->ped.step = 0;
	st->ped.time = 0;
}

static int nvi_id_hal(struct nvi_state *st, u8 dev_id)
{
	int ret = 0;

	switch (dev_id) {
	case MPU6050_ID:
		st->hal = &nvi_hal_6050;
		break;

	case MPU6500_ID:
	case MPU9250_ID:
		st->hal = &nvi_hal_6500;
		break;

	case MPU6515_ID:
	case MPU9350_ID:
		st->hal = &nvi_hal_6515;
		break;

	case ICM20628_ID:
		st->hal = &nvi_hal_20628;
		break;

	default:
		ret = -ENODEV;
	}

	return ret;
}

static int nvi_id_dev(struct iio_dev *indio_dev, const char *name)
{
	struct nvi_state *st = iio_priv(indio_dev);
	u8 dev_id;
	unsigned int i;
	int ret;

	if (!strcmp(name, "mpu6xxx"))
		dev_id = 0xFF;
	else if (!strcmp(name, "mpu6050"))
		dev_id = MPU6050_ID;
	else if (!strcmp(name, "mpu6500"))
		dev_id = MPU6500_ID;
	else if (!strcmp(name, "mpu6515"))
		dev_id = MPU6515_ID;
	else if (!strcmp(name, "mpu9150"))
		dev_id = MPU6050_ID;
	else if (!strcmp(name, "mpu9250"))
		dev_id = MPU9250_ID;
	else if (!strcmp(name, "mpu9350"))
		dev_id = MPU9350_ID;
	else if (!strcmp(name, "icm20628"))
		dev_id = ICM20628_ID;
	else
		return -ENODEV;

	if (dev_id == 0xFF) {
		st->hal = &nvi_hal_6050;
		nvi_pm_wr(st, 0, 0, 0);
		ret = nvi_i2c_read(st, st->i2c_addr,
				   nvi_hal_reg_6050.who_am_i.reg, 1, &dev_id);
		if (ret) {
			dev_err(&st->i2c->dev, "%s AUTO ID FAILED\n",
				__func__);
			return -ENODEV;
		}

		ret = nvi_id_hal(st, dev_id);
		if (ret) {
			nvs_vregs_disable(&st->i2c->dev, st->vreg,
					  ARRAY_SIZE(nvi_vregs));
			st->hal = &nvi_hal_20628;
			ret = nvi_pm_wr(st, 0, 0, 0);
			ret = nvi_i2c_read(st, st->i2c_addr,
					   nvi_hal_reg_20628.who_am_i.reg,
					   1, &dev_id);
			if (ret) {
				dev_err(&st->i2c->dev, "%s AUTO ID FAILED\n",
					__func__);
				return -ENODEV;
			}

			ret = nvi_id_hal(st, dev_id);
			if (ret) {
				dev_err(&st->i2c->dev, "%s AUTO ID FAILED\n",
					__func__);
				return -ENODEV;
			}
		}
	} else {
		nvi_id_hal(st, dev_id);
		nvi_pm_wr(st, 0, 0, 0);
	}
	ret = st->hal->init(st);
	nvi_rd_accel_offset(st);
	nvi_rd_gyro_offset(st);
	for (i = 0; i < AXIS_N; i++) {
		st->rom_accel_offset[i] = (s16)st->rc.accel_offset[i];
		st->rom_gyro_offset[i] = (s16)st->rc.gyro_offset[i];
		st->input_accel_offset[i] = 0;
		st->input_accel_dmp_bias[i] = 0;
		st->input_gyro_offset[i] = 0;
		st->input_gyro_dmp_bias[i] = 0;
	}
	dev_info(&st->i2c->dev, "%s: DT=%s ID=%x USING: %s\n",
		 __func__, name, dev_id, st->hal->part_name);
	return ret;
}

static int nvi_id_i2c(struct iio_dev *indio_dev,
		      const struct i2c_device_id *id)
{
	struct nvi_state *st = iio_priv(indio_dev);
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(nvi_i2c_addrs); i++) {
		if (st->i2c->addr == nvi_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(nvi_i2c_addrs)) {
		st->i2c_addr = st->i2c->addr;
		ret = nvi_id_dev(indio_dev, id->name);
	} else {
		for (i = 0; i < ARRAY_SIZE(nvi_i2c_addrs); i++) {
			st->i2c_addr = nvi_i2c_addrs[i];
			ret = nvi_id_dev(indio_dev, NVI_NAME);
			if (!ret)
				break;
		}
	}
	return ret;
}

static int nvi_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct nvi_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	ret = nvi_pm(st, NVI_PM_OFF);
	st->suspend = true;
	mutex_unlock(&indio_dev->mlock);
	if (ret)
		dev_err(dev, "%s ERR\n", __func__);
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(dev, "%s done\n", __func__);
	return 0;
}

static int nvi_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct nvi_state *st = iio_priv(indio_dev);
	unsigned int i;

	mutex_lock(&indio_dev->mlock);
	st->suspend = false;
	for (i = 0; i < AUX_PORT_MAX; i++) {
		if (st->aux.port[i].nmp.shutdown_bypass)
			break;
	}
	if (i < AUX_PORT_MAX) {
		nvi_pm(st, NVI_PM_ON);
		nvi_aux_bypass_enable(st, false);
	}
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(dev, "%s done\n", __func__);
	return 0;
}

static const struct dev_pm_ops nvi_pm_ops = {
	.suspend = nvi_suspend,
	.resume = nvi_resume,
};

static void nvi_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct nvi_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	st->shutdown = true;
	if (st->i2c->irq)
		disable_irq_nosync(st->i2c->irq);
	if (st->hal)
		nvi_pm(st, NVI_PM_OFF);
	mutex_unlock(&indio_dev->mlock);
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int nvi_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct nvi_state *st = iio_priv(indio_dev);

	if (st != NULL) {
		nvi_shutdown(client);
		if (indio_dev->dev.devt)
			iio_device_unregister(indio_dev);
		if (st->trig != NULL) {
			if (client->irq)
				free_irq(client->irq, st);
			iio_trigger_unregister(st->trig);
			iio_trigger_free(st->trig);
		}
		if (kfifo_initialized(&st->timestamps))
			kfifo_free(&st->timestamps);
		if (indio_dev->buffer != NULL) {
			iio_buffer_unregister(indio_dev);
			iio_kfifo_free(indio_dev->buffer);
		}
		nvi_pm_exit(st);
		iio_device_free(indio_dev);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int nvi_of_dt(struct i2c_client *client, struct nvi_state *st)
{
	struct device_node *dn = client->dev.of_node;
	char const *pchar;
	u32 tmp;
	int len;

	/* common NVS programmable parameters */
	st->iio_ts_en = of_property_read_bool(dn, "invensense,iio_timestamps");
	/* device specific parameters */
	pchar = of_get_property(dn, "invensense,matrix", &len);
	if (pchar && len == sizeof(st->pdata.orientation)) {
		memcpy(&st->pdata.orientation, pchar, len);
	} else { /* obsolete */
		pchar = of_get_property(dn, "invensense,orientation", &len);
		if (pchar && len == sizeof(st->pdata.orientation))
			memcpy(&st->pdata.orientation, pchar, len);
	}
	nvi_init_config(st);
	if (!of_property_read_u32(dn, "invensense,standby_en", &tmp)) {
		if (tmp)
			st->master_enable |= (1 << EN_STDBY);
		else
			st->master_enable &= ~(1 << EN_STDBY);
	}
	of_property_read_u32(dn, "invensense,lpa_delay_us",
			     &st->chip_config.lpa_delay_us);
	if (st->chip_config.lpa_delay_us)
		st->master_enable |= (1 << EN_LPA);
	else
		st->master_enable &= ~(1 << EN_LPA);
	of_property_read_u32(dn, "invensense,gyro_start_delay_ns",
			     (u32 *)&st->chip_config.gyro_start_delay_ns);
	of_property_read_u32(dn, "invensense,bypass_timeout_ms",
			     &st->chip_config.bypass_timeout_ms);
	of_property_read_u32(dn, "invensense,temp_fifo_en",
			     &st->chip_config.temp_fifo_en);
	of_property_read_u32(dn, "invensense,fifo_threshold",
			     &st->chip_config.fifo_thr);
	of_property_read_u32(dn, "invensense,fifoReservedEventCount",
			     &st->chip_config.fifo_reserve);
	of_property_read_u32(dn, "invensense,fifoMaxEventCount",
			     &st->chip_config.fifo_max);
	if (!of_property_read_u32(dn, "invensense,fsr", &tmp))
		st->chip_config.fsr = tmp;
	if (!of_property_read_u32(dn, "invensense,lpf", &tmp))
		st->chip_config.lpf = tmp;
	/* mpu_platform_data may be used by invensense code */
	of_property_read_u8(dn, "invensense,int_config",
			    &st->pdata.int_config);
	of_property_read_u8(dn, "invensense,level_shifter",
			    &st->pdata.level_shifter);
	pchar = of_get_property(dn, "invensense,key", &len);
	if (pchar && len == sizeof(st->pdata.key))
		memcpy(&st->pdata.key, pchar, len);
	return 0;
}

static int nvi_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct nvi_state *st;
	struct mpu_platform_data *pdata;
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
		ret = nvi_of_dt(client, st);
		if (ret)
			goto nvi_probe_err;
	} else {
		nvi_init_config(st);
		pdata = (struct mpu_platform_data *)
						dev_get_platdata(&client->dev);
		if (pdata) {
			st->pdata = *pdata;
		} else {
			dev_err(&client->dev, "%s dev_get_platdata ERR\n",
				__func__);
			ret = -EINVAL;
			goto nvi_probe_err;
		}
	}

	spin_lock_init(&st->time_stamp_lock);
	nvi_pm_init(st);
	ret = nvi_id_i2c(indio_dev, id);
	if (ret) {
		dev_err(&client->dev, "%s ERR: nvi_id_i2c\n", __func__);
		goto nvi_probe_err;
	}

	indio_dev->buffer = iio_kfifo_allocate(indio_dev);
	if (!indio_dev->buffer) {
		dev_err(&client->dev, "%s iio_kfifo_allocate ERR\n", __func__);
		ret = -ENOMEM;
		goto nvi_probe_err;
	}

	indio_dev->buffer->scan_timestamp = true;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = nvi_channels;
	indio_dev->num_channels = ARRAY_SIZE(nvi_channels);
	indio_dev->name = NVI_NAME;
	indio_dev->info = &nvi_iio_info;
	indio_dev->setup_ops = &nvi_buffer_setup_ops;
	ret = iio_buffer_register(indio_dev, indio_dev->channels,
				  indio_dev->num_channels);
	if (ret) {
		dev_err(&client->dev, "%s ERR: iio_buffer_register\n",
			__func__);
		goto nvi_probe_err;
	}

	indio_dev->buffer->access->set_length(indio_dev->buffer, KBUF_SZ);
	indio_dev->buffer->access->request_update(indio_dev->buffer);
	if (!st->i2c->irq) {
		dev_err(&client->dev, "%s ERR: no interrupt\n", __func__);
		ret = -EINVAL;
		goto nvi_probe_err;
	}

	INIT_KFIFO(st->timestamps);
	ret = request_threaded_irq(st->i2c->irq,
				   nvi_irq_handler, nvi_irq_thread,
				   IRQF_TRIGGER_RISING, NVI_NAME, st);
	if (ret) {
		dev_err(&client->dev, "%s req_threaded_irq ERR %d\n",
			__func__, ret);
		ret = -ENOMEM;
		goto nvi_probe_err;
	}

	st->trig = iio_trigger_alloc("%s-dev%d",
				     indio_dev->name, indio_dev->id);
	if (st->trig == NULL) {
		dev_err(&client->dev, "%s iio_allocate_trigger ERR\n",
			__func__);
		ret = -ENOMEM;
		goto nvi_probe_err;
	}

	st->trig->dev.parent = &st->i2c->dev;
	st->trig->ops = &nvi_trigger_ops;
	ret = iio_trigger_register(st->trig);
	if (ret) {
		dev_err(&client->dev, "%s iio_trigger_register ERR\n",
			__func__);
		ret = -ENOMEM;
		goto nvi_probe_err;
	}

	indio_dev->trig = st->trig;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_device_register(indio_dev);
	if (ret)
		goto nvi_probe_err;

	if (st->hal->dmp)
		inv_create_dmp_sysfs(indio_dev);
	nvi_pm(st, NVI_PM_AUTO);
	indio_dev_local = indio_dev;
	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

nvi_probe_err:
	dev_err(&client->dev, "%s ERR %d\n", __func__, ret);
	nvi_remove(client);
	return ret;
}

static struct i2c_device_id nvi_i2c_device_id[] = {
	{ NVI_NAME, 0 },
	{ "mpu6050", 0 },
	{ "mpu6500", 0 },
	{ "mpu6515", 0 },
	{ "mpu9250", 0 },
	{ "mpu9150", 0 },
	{ "mpu9350", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, nvi_i2c_device_id);

static const struct of_device_id nvi_of_match[] = {
	{ .compatible = "invensense,mpu6xxx", },
	{ .compatible = "invensense,mpu6050", },
	{ .compatible = "invensense,mpu6500", },
	{ .compatible = "invensense,mpu6515", },
	{ .compatible = "invensense,mpu9150", },
	{ .compatible = "invensense,mpu9250", },
	{ .compatible = "invensense,mpu9350", },
	{}
};

MODULE_DEVICE_TABLE(of, nvi_of_match);

static struct i2c_driver nvi_driver = {
	.class				= I2C_CLASS_HWMON,
	.probe				= nvi_probe,
	.remove				= nvi_remove,
	.shutdown			= nvi_shutdown,
	.driver				= {
		.name			= NVI_NAME,
		.owner			= THIS_MODULE,
		.of_match_table		= of_match_ptr(nvi_of_match),
		.pm			= &nvi_pm_ops,
	},
	.id_table			= nvi_i2c_device_id,
};

module_i2c_driver(nvi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NVidiaInvensense driver");
MODULE_AUTHOR("NVIDIA Corporation");

