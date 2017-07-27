/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c-mux.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <dt-bindings/gpio/tegra186-gpio.h>
#include "stepper.h"

#define MCNTL_MOTOR_START	0x80
#define MCNTL_MOTOR_RESTART	0x40
#define MAX_RAMP_FACTOR		15
#define MAX_PULSE_WIDTH		0x1fff

extern struct class *stepper_class;

enum address_map {
	MODE,
	WDTOI,
	WDTCNT,
	IO_CFG,
	INTMODE,
	MSK,
	INTSTAT,
	IP,
	INT_MTR_ACT,
	EXTRASTEPS0,
	EXTRASTEPS1,
	OP_CFGH_PHS,
	OP_STAT_TO,
	RUCNTL,
	RDCNTL,
	PMA,
	LOOPDLY_CW,
	LOOPDLY_CCW,
	CWSCOUNTL,
	CWSCOUNTH,
	CCWSCOUNTL,
	CCWSCOUNTH,
	CWPWL,
	CWPWH,
	CCWPWL,
	CCWPWH,
	MCNTL,
	SUBADDR1,
	SUBADDR2,
	SUBADDR3,
	ALLCALLADR,
	STEPCOUNT1,
	STEPCOUNT2,
	STEPCOUNT3,
	MAX_REG
};

struct pca9629a_stepper {
	struct i2c_client	*stepper_i2c;
	struct stepper_device	*stepper_dev;
};


/*Send device commands to set the stepper motor direction*/
static int stepper_set_direction(struct i2c_client *client, enum stepper_direction direction)
{
	struct i2c_adapter *adap = client->adapter;
	int ret = 0;
	unsigned char command[2], data;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command[0],
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
	};
	command[0] = MCNTL;
	ret = i2c_transfer(adap, msg, 2);
	if (ret == 2) {
		if (direction == STEPPER_CLKWISE) {
			command[1] = data & (0xfc);
		} else if (direction == STEPPER_ANTI_CLKWISE) {
			command[1] = data | 0x01;
		}
		msg[0].len = 2;
		ret = i2c_transfer(adap, msg, 1);
		if (ret != 1) {
			dev_err(&client->dev, "%s: direction i2c trxfer error\n", __func__);
			ret = -EIO;
		} else {
			ret = 0;
		}
	} else {
		dev_err(&client->dev, "%s: i2c txfer failure\n", __func__);
		ret = -EIO;
	}

	return ret;
}
/*Enable the stepper motor*/
static int stepper_start(struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	int ret = 0;
	unsigned char command[2], data;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command[0],
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
	};
	command[0] = MCNTL;
	ret = i2c_transfer(adap, msg, 2);
	if (ret == 2) {
		command[1] = data | 0x80;
		msg[0].len = 2;
		ret = i2c_transfer(adap, msg, 1);
		if (ret != 1) {
			ret = -EIO;
		} else {
			ret = 0;
		}
	} else {
		dev_err(&client->dev, "%s: i2c txfer failure\n", __func__);
		ret = -EIO;
	}

	return ret;
}

/*Restart the stepper motor with new parameters if it is already running*/
static int stepper_restart(struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	int ret = 0;
	unsigned char command[2], data;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command[0],
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
	};
	command[0] = MCNTL;
	ret = i2c_transfer(adap, msg, 2);
	if (ret == 2) {
		/*Send restart only if motor is running*/
		if (data & MCNTL_MOTOR_START) {
			command[1] = data |
				(MCNTL_MOTOR_START | MCNTL_MOTOR_RESTART);
			msg[0].len = 2;
			ret = i2c_transfer(adap, &msg[0], 1);
			if (ret != 1) {
				ret = -EIO;
			} else {
				ret = 0;
			}
		} else {
		       ret = -EINVAL;
		}
	} else {
		dev_err(&client->dev, "%s: i2c txfer failure\n", __func__);
		ret = -EIO;
	}

	return ret;
}

/*Disable the stepper motor*/
static int stepper_stop(struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	int ret = 0;
	unsigned char command[2], data;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command[0],
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
	};
	command[0] = MCNTL;
	ret = i2c_transfer(adap, msg, 2);

	if (ret == 2) {
		command[1] = data & (u8)(~MCNTL_MOTOR_START);
		msg[0].len = 2;
		ret = i2c_transfer(adap, msg, 1);
		if (ret != 1) {
			dev_err(&client->dev, "%s: i2c txfer failure\n", __func__);
			ret = -EIO;
		} else {
			ret = 0;
		}
	} else {
		dev_err(&client->dev, "%s: i2c txfer failure\n", __func__);
		ret = -EIO;
	}

	return ret;

}

/*Immediately stop without caring for ramp down delay if enabled*/
static int stepper_emergency_stop(struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	int ret = 0;
	unsigned char command[2], data;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command[0],
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
	};
	command[0] = MCNTL;
	ret = i2c_transfer(adap, msg, 2);
	if (ret == 2)
	{
		command[1] = data | 0x20;
		msg[0].len = 2;
		ret = i2c_transfer(adap, msg, 1);
		if (ret != 1) {
			dev_err(&client->dev,"%s: i2c txfer failure\n", __func__);
			ret = -EIO;
		} else {
			ret = 0;
		}
	} else
		ret = -EIO;

	return ret;

}

/*Set step rate  rate is input as percentage.
  range - 0 to 100
  rate is changed by changing the pulse width of both CW and CCW directions
  registers - 0x16, 0x17 for CW, 0x18, 0x19 for CCW.
  pulse width register has reserved 3 bits in MSB for prescaler.
  width = 2POW(prescaler)*(pulse width)
  lower the width, faster the stepping rate
  Min = 0 = 3usec
  Max = 0xffff = 3145728 usec = 3.146 sec
  Limiting the prescaler = 0, step granularity is 3us.  Max = 1fff = 24573usec = 24.573ms
  rate = 0 = Max
  rate = 100 = Min
*/
static int stepper_set_rate(struct i2c_client *client, int rate, bool is_continuous)
{
	struct i2c_adapter *adap = client->adapter;
	int ret = -1;
	unsigned short width;
	unsigned char command[5];
	struct i2c_msg msg = {
			.addr = client->addr,
			.flags = 0,
			.len = 5,
			.buf = &command[0],
	};
	if (rate > 100)
		return -1;

	command[0] = CWPWL | 0x80;
	width = (unsigned short) MAX_PULSE_WIDTH - (unsigned short)(rate *MAX_PULSE_WIDTH / 100);
	command[1] = command[3] = width & 0xff;
	command[2] = command[4] = (width & 0xff00) >> 8;
	ret = i2c_transfer(adap, &msg, 1);
	if (ret == 1) {
		if (is_continuous) {
			ret = stepper_restart(client);
		} else
			ret = 0;
	} else {
		ret = -1;
	}

	return ret;
}

static int stepper_reset(struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	int ret = 0;
	unsigned char data;
	unsigned char parameters[32] = {
	0x80,
	0x20, 0x00, 0x00, 0x03, 0x13,
	0x1c, 0x00, 0x00, 0x68, 0x00,
	0x00, 0x50, 0x80, 0x35, 0x35,
	0x00, 0x00, 0x00, 0xff, 0xff,
	0xff, 0xff, 0x0a, 0x1a, 0x0a,
	0x1a, 0xc0, 0xe2, 0xe4, 0xe6,
	0xe0
	};
	struct i2c_msg msg[2];

	/*Reset the device*/
	msg[0].addr = 0;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &data;
	data = 0x06;
	ret = i2c_transfer(adap, msg, 1);
	if (ret == 1) {
		/*Set motor parameters*/
		msg[0].addr = client->addr;
		msg[0].flags = 0;
		msg[0].len = 32;
		msg[0].buf = parameters;
		ret = i2c_transfer(adap, msg, 1);
		if (ret != 32) {
			ret = -1;
		} else {
			ret = 0;
		}
	} else {
		ret = -1;
	}
	return ret;
}

static int stepper_send(struct i2c_client *client, const char *buf, size_t len)
{
	struct i2c_adapter *adap = client->adapter;
	char data[50];
	int ret = 0;
	struct i2c_msg msg = {
	    .addr = client->addr,
	    .flags = 0,
	    .len = len,
	    .buf = data,
	};
	if (data == NULL)
		return -ENOMEM;
	memcpy(data, buf, len);
	/*Set address auto-increment flag*/
	data[0] |= 0x80;
	ret = i2c_transfer(adap, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_err(&client->dev, "%s: i2c txfer failure\n", __func__);
		ret = -EIO;
	}
	return ret;
}

static int stepper_receive(struct i2c_client *client, int offset, char *buf, size_t len)
{
	struct i2c_adapter *adap = client->adapter;
	int ret = 0, i;
	char command = 0x80 | offset;
	char data[50];
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	if (len > MAX_REG)
		len = MAX_REG;
	if (data == NULL)
		return -ENOMEM;
	ret = i2c_transfer(adap, &msg[0], 2);
	if (ret == 2) {
		ret = 0;
		for (i = 0; i < len; i++)
			buf[i] = data[i];
	} else {
		dev_err(&client->dev, "%s: i2c txfer failure\n", __func__);
		ret = -1;
	}
	return ret;
}

int pca9629a_stepper_open(struct device *dev)
{
	return 0;
}
int pca9629a_stepper_release(struct device *dev)
{
	return 0;
}
int pca9629a_stepper_ioctl(struct device *dev, unsigned int attr, unsigned long val)
{
	switch (attr) {
	case 1/*start*/:
		if (val == 1)
			stepper_start(to_i2c_client(dev->parent));
		else
			stepper_stop(to_i2c_client(dev->parent));
	break;
	case 3:/*direction*/
		if (val == 0)
			stepper_set_direction(to_i2c_client(dev->parent), STEPPER_CLKWISE);
		else
			stepper_set_direction(to_i2c_client(dev->parent), STEPPER_ANTI_CLKWISE);
	break;
	case 4/*Emergency stop*/:
		stepper_emergency_stop(to_i2c_client(dev->parent));
	break;
	case 5/*Reset*/:
		stepper_reset(to_i2c_client(dev->parent));
	break;
	case 6/*set rate*/:
		stepper_set_rate(to_i2c_client(dev->parent), val, 1);
	break;

	default:
	break;
	}

	return 0;
}
int pca9629a_stepper_write(struct device *dev, const char *buf, size_t len)
{
	int ret = stepper_send(to_i2c_client(dev->parent), buf, len);

	return ret;
}

int pca9629a_stepper_read(struct device *dev, char *buf, size_t len)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret = stepper_receive(to_i2c_client(dev->parent), stepper->offset, buf, len);

	return ret;
}

int pca9629a_stepper_start(struct device *dev)
{
	int ret = stepper_start(to_i2c_client(dev->parent));

	return ret;
}

int pca9629a_stepper_restart(struct device *dev)
{
	int ret = stepper_restart(to_i2c_client(dev->parent));

	return ret;
}

int pca9629a_stepper_stop(struct device *dev)
{
	int ret = stepper_stop(to_i2c_client(dev->parent));

	return ret;
}

int pca9629a_stepper_set_dir(struct device *dev, enum stepper_direction dir)
{
	int ret = stepper_set_direction(to_i2c_client(dev->parent), dir);

	return ret;
}

int pca9629a_stepper_set_rate(struct device *dev, int val, enum stepper_rate_ramp immediate)
{
	bool is_continuous;
	int ret;
	if (immediate == STEPPER_RATE_CONTINUOUS)
		is_continuous = 1;
	else
		is_continuous = 0;
	ret = stepper_set_rate(to_i2c_client(dev->parent), val, is_continuous);

	return ret;
}

int pca9629a_stepper_set_ramp(struct device *dev, int val, enum stepper_ramp_mode mode)
{
	char buff[2];
	int ret;
	buff[1] = 0x30 | (unsigned char)((val * MAX_RAMP_FACTOR)/100);
	if (mode == STEPPER_RAMP_UP)
		buff[0] = RUCNTL;
	else
		buff[0] = RDCNTL;
	ret = stepper_send(to_i2c_client(dev->parent), buff, 2);

	return ret;
}

int pca9629a_stepper_get_param(struct device *dev, enum stepper_param param, int offset)
{
	int ret = -1, reg = 0, err = -1, val2 = 0;
	char data[4];
	switch (param) {
	case STEPPER_STATUS:
		reg = IP;
		err = stepper_receive(to_i2c_client(dev->parent), reg, data, 1);
		if (err != 0)
			ret = 2;
		else
			ret = *((int *)(&data[0]));
	break;

	case STEPPER_START:
		reg = MCNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, data, 1);
		if (err != 0)
			ret = 2;
		else
			ret = *((int *)(&data[0]));
	break;
	case STEPPER_CW_RATE:
		reg = CWPWL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&ret, 2);
		if (err == 0) {
			ret = ret & 0x1fff;
			ret = MAX_PULSE_WIDTH - ret;
			goto exit;
		}
		ret = -1;
	break;
	case STEPPER_CCW_RATE:
		reg = CCWPWL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&ret, 2);
		if (err == 0) {
			ret = ret & 0x1fff;
			ret = MAX_PULSE_WIDTH - ret;
			goto exit;
		}
		ret = -1;
	break;
	case STEPPER_MAX_RATE:
		ret = MAX_PULSE_WIDTH;
	break;
	case STEPPER_RAMPUP_VAL:
		reg = RUCNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&ret, 1);
		if (err == 0) {
			ret = ret & 0x0f;
			goto exit;
		}
	break;
	case STEPPER_RAMPDOWN_VAL:
		reg = RDCNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&ret, 1);
		if (err == 0) {
			ret = ret & 0x0f;
			goto exit;
		}
	break;
	case STEPPER_DIRECTION:
		reg =  MCNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&ret, 1);
		if (err == 0) {
			ret = ret & 0x01;
			goto exit;
		}
	break;
	case STEPPER_CW_STEPS:
		reg = CWSCOUNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&ret, 2);
		if (err == 0) {
			ret = ret & 0xffff;
			goto exit;
		}

	break;
	case STEPPER_CCW_STEPS:
		reg = CCWSCOUNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&ret, 2);
		if (err == 0) {
			ret = ret & 0xffff;
			goto exit;
		}

	break;
	case STEPPER_MODE:
		reg =  MCNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&ret, 1);
		if (err == 0) {
			/*Read PMA*/
			reg = PMA;
			err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&val2, 1);
			if (err) {
				ret = -1;
				goto exit;
			}

			if (val2 & 0xff)
				ret = STEPPER_FINITE_STEPS;
			else {
				if ((ret & 0x03) >= 0x02) {
					if (ret & 0x01)
						ret = STEPPER_CCW_THEN_CW;
					else
						ret = STEPPER_CW_THEN_CCW;
				} else
					ret = STEPPER_CONTINUOUS;
			}
		}
	break;
	case STEPPER_REG_OP:
		if ((offset < MAX_REG) && (offset >= 0)) {
			ret = stepper_receive(to_i2c_client(dev->parent), offset, (char *)&val2, 1);
			if (!ret)
				ret = val2;
			else
				ret = -1;
		}
	break;
	default:
	break;
	}
exit:
	return ret;
}

int pca9629a_stepper_set_param(struct device *dev, enum stepper_param param, int offset, int value)
{
	int ret = -1, reg = 0, err = -1, val2 = 0;
	char *buf;

	buf = devm_kzalloc(dev, 10, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	switch (param) {
	case STEPPER_START:
		if (value & 0x01)
			ret = pca9629a_stepper_start(dev);
		else
			ret = pca9629a_stepper_stop(dev);
	break;
	case STEPPER_CW_RATE:
		/*Value will be limited to MAX_PULSE_WIDTH = 0x1fff*/
		val2 = MAX_PULSE_WIDTH - value;
		buf[1] = val2 & 0xff;
		buf[2] = (val2 >> 8) & 0x1f;
		buf[0] = CWPWL | 0x80;
		ret = stepper_send(to_i2c_client(dev->parent), buf, 3);
		if (ret != 0)
			goto exit;
		reg =  MCNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&val2, 1);
		if (err == 0) {
			buf[0] = reg;
			buf[1] = (val2 & 0xff) | 0x40;/*Restart with updated rate*/
			ret = stepper_send(to_i2c_client(dev->parent), buf, 2);
			goto exit;
		}
		ret = -1;
	break;
	case STEPPER_CCW_RATE:
		/*Value will be limited to MAX_PULSE_WIDTH = 0x1fff*/
		val2 = MAX_PULSE_WIDTH - value;
		buf[1] = val2 & 0xff;
		buf[2] = (val2 >> 8) & 0x1f;
		buf[0] = CCWPWL | 0x80;
		ret = stepper_send(to_i2c_client(dev->parent), buf, 3);
		if (ret != 0)
			goto exit;
		reg =  MCNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&val2, 1);
		if (err == 0) {
			buf[0] = reg;
			buf[1] = (val2 & 0xff) | 0x40;/*Restart with updated rate*/
			ret = stepper_send(to_i2c_client(dev->parent), buf, 2);
			goto exit;
		}
		ret = -1;

	break;
	case STEPPER_RAMPUP_VAL:
		buf[0] = RUCNTL;
		buf[1] = (value & 0x0f) | 0x30;/*Enable immediately*/
		ret = stepper_send(to_i2c_client(dev->parent), buf, 2);
	break;
	case STEPPER_RAMPDOWN_VAL:
		buf[0] = RDCNTL;
		buf[1] = (value & 0x0f) | 0x30;/*Enable immediately*/
		ret = stepper_send(to_i2c_client(dev->parent), buf, 2);
	break;
	case STEPPER_DIRECTION:
		buf[0] =  MCNTL;
		err = stepper_receive(to_i2c_client(dev->parent), buf[0], (char *)&ret, 1);
		if (err == 0) {
			buf[1] = (ret & 0xfe) | value;
			ret = stepper_send(to_i2c_client(dev->parent), buf, 2);
		}
	break;
	case STEPPER_CW_STEPS:
		buf[0] = CWSCOUNTL;
		buf[1] = value & 0xff;
		buf[2] = (value >> 8) & 0xff;
		ret = stepper_send(to_i2c_client(dev->parent), buf, 3);
	break;
	case STEPPER_CCW_STEPS:
		buf[0] = CCWSCOUNTL;
		buf[1] = value & 0xff;
		buf[2] = (value >> 8) & 0xff;
		ret = stepper_send(to_i2c_client(dev->parent), buf, 3);
	break;
	case STEPPER_MODE:
		reg =  MCNTL;
		err = stepper_receive(to_i2c_client(dev->parent), reg, (char *)&ret, 1);
		if (err == 0) {
			buf[0] = reg;
			if (value == STEPPER_CONTINUOUS)
				buf[1] = (ret & 0xfd);
			else if (value == STEPPER_CW_THEN_CCW)
				buf[1] = (ret & 0xfc) | 0x02;
			else if (value == STEPPER_CCW_THEN_CW)
				buf[1] = (ret & 0xfc) | 0x03;
			else
				buf[1] = ret & 0xff;

			err = stepper_send(to_i2c_client(dev->parent), buf, 2);
			if (err) {
				ret = -1;
				goto exit;
			}
			/*write PMA*/
			buf[0] = PMA;
			if (value == STEPPER_CONTINUOUS)
				buf[1] = 0;
			else if (value == STEPPER_CW_THEN_CCW)
				buf[1] = 0;
			else if (value == STEPPER_CCW_THEN_CW)
				buf[1] = 0;
			else
				buf[1] = 1;

			err = stepper_send(to_i2c_client(dev->parent), buf, 2);
			if (err) {
				ret = -1;
				goto exit;
			}
		}
	break;
	case STEPPER_EMERGENCY:
		stepper_emergency_stop(to_i2c_client(dev->parent));
	break;
	case STEPPER_REG_OP:
		if ((offset < MAX_REG) && (offset >= 0)) {
			buf[0] = offset;
			buf[1] = value;
			ret = stepper_send(to_i2c_client(dev->parent), buf, 2);
		}
	break;

	default:
	break;
	}
exit:
	devm_kfree(dev, buf);
	return ret;
}
static const struct stepper_ops pca9629a_stepper_ops = {
	.open = pca9629a_stepper_open,
	.release = pca9629a_stepper_release,
	.ioctl = pca9629a_stepper_ioctl,
	.write = pca9629a_stepper_write,
	.read = pca9629a_stepper_read,
	.start = pca9629a_stepper_start,
	.restart = pca9629a_stepper_restart,
	.stop = pca9629a_stepper_stop,
	.set_direction = pca9629a_stepper_set_dir,
	.set_rate = pca9629a_stepper_set_rate,
	.set_ramp_rate = pca9629a_stepper_set_ramp,
	.get_param = pca9629a_stepper_get_param,
	.set_param = pca9629a_stepper_set_param
};


static int pca9629a_stepper_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pca9629a_stepper *stepper;
	int err = 0;

	stepper = devm_kzalloc(&client->dev, sizeof(struct pca9629a_stepper), GFP_KERNEL);
	if (!stepper) {
		dev_err(&client->dev, "Error assigning memory for pca9629a_stepper\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, stepper);
	stepper->stepper_dev = stepper_device_register(client->name, &client->dev, &pca9629a_stepper_ops, THIS_MODULE);
	if (!stepper->stepper_dev)
		return -ENODEV;

	if (&stepper->stepper_dev->chardev == NULL) {
		goto err;
		err = -ENODEV;
	}
	stepper_reset(client);

err:
	return err;
}

static int pca9629a_stepper_remove(struct i2c_client *client)
{
	struct pca9629a_stepper *stepper;
	stepper = i2c_get_clientdata(client);
	stepper->stepper_dev->ops->ioctl(&stepper->stepper_dev->dev, 1, 0);
	stepper_device_remove(stepper->stepper_dev);
	return 0;
}
static struct i2c_device_id stepper_id_table[] = {
	{"stepper_pca", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, stepper_id_table);
static const struct of_device_id stepper_dt_ids[] = {
	{.compatible = "stepper_pca", },
	{}
};
MODULE_DEVICE_TABLE(of, stepper_dt_ids);

static struct i2c_driver stepper_driver = {
	.driver = {
		.name = "stepper_pca",
	},
	.id_table = stepper_id_table,
	.probe = pca9629a_stepper_probe,
	.remove = pca9629a_stepper_remove,
};
module_i2c_driver(stepper_driver);

MODULE_AUTHOR("Vishruth Jain <vishruthj@nvidia.com>");
MODULE_DESCRIPTION("PCA9629A stepper device driver");
MODULE_LICENSE("GPL");
