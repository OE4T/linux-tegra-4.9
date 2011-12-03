/*
 * drivers/misc/nct1008.c
 *
 * Driver for NCT1008, temperature monitoring device from ON Semiconductors
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/nct1008.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#define DRIVER_NAME "nct1008"

/* Register Addresses */
#define LOCAL_TEMP_RD			0x00
#define EXT_TEMP_RD_HI			0x01
#define EXT_TEMP_RD_LO			0x10
#define STATUS_RD			0x02
#define CONFIG_RD			0x03

#define LOCAL_TEMP_HI_LIMIT_RD		0x05
#define LOCAL_TEMP_LO_LIMIT_RD		0x06

#define EXT_TEMP_HI_LIMIT_HI_BYTE_RD	0x07

#define CONFIG_WR			0x09
#define CONV_RATE_WR			0x0A
#define LOCAL_TEMP_HI_LIMIT_WR		0x0B
#define LOCAL_TEMP_LO_LIMIT_WR		0x0C
#define EXT_TEMP_HI_LIMIT_HI_BYTE_WR	0x0D
#define EXT_TEMP_LO_LIMIT_HI_BYTE_WR	0x0E
#define OFFSET_WR			0x11
#define OFFSET_QUARTER_WR		0x12
#define EXT_THERM_LIMIT_WR		0x19
#define LOCAL_THERM_LIMIT_WR		0x20
#define THERM_HYSTERESIS_WR		0x21

/* Configuration Register Bits */
#define EXTENDED_RANGE_BIT		BIT(2)
#define THERM2_BIT			BIT(5)
#define STANDBY_BIT			BIT(6)
#define ALERT_BIT			BIT(7)

/* Max Temperature Measurements */
#define EXTENDED_RANGE_OFFSET		64U
#define STANDARD_RANGE_MAX		127U
#define EXTENDED_RANGE_MAX		(150U + EXTENDED_RANGE_OFFSET)

#define NCT1008_MIN_TEMP -64
#define NCT1008_MAX_TEMP 191

#define MAX_STR_PRINT 50

#define MIN_SLEEP_MSEC			20
#define CELSIUS_TO_MILLICELSIUS(x) ((x)*1000)
#define MILLICELSIUS_TO_CELSIUS(x) ((x)/1000)

static inline s8 value_to_temperature(bool extended, u8 value)
{
	return extended ? (s8)(value - EXTENDED_RANGE_OFFSET) : (s8)value;
}

static inline u8 temperature_to_value(bool extended, s8 temp)
{
	return extended ? (u8)(temp + EXTENDED_RANGE_OFFSET) : (u8)temp;
}

static int nct1008_get_temp(struct device *dev, long *pTemp)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	s8 temp_local;
	u8 temp_ext_lo;
	s8 temp_ext_hi;
	long temp_ext_milli;
	long temp_local_milli;
	u8 value;

	/* Read Local Temp */
	value = i2c_smbus_read_byte_data(client, LOCAL_TEMP_RD);
	if (value < 0)
		goto error;
	temp_local = value_to_temperature(pdata->ext_range, value);
	temp_local_milli = CELSIUS_TO_MILLICELSIUS(temp_local);

	/* Read External Temp */
	value = i2c_smbus_read_byte_data(client, EXT_TEMP_RD_LO);
	if (value < 0)
		goto error;
	temp_ext_lo = (value >> 6);

	value = i2c_smbus_read_byte_data(client, EXT_TEMP_RD_HI);
	if (value < 0)
		goto error;
	temp_ext_hi = value_to_temperature(pdata->ext_range, value);

	temp_ext_milli = CELSIUS_TO_MILLICELSIUS(temp_ext_hi) +
				temp_ext_lo * 250;

	/* Return max between Local and External Temp */
	*pTemp = max(temp_local_milli, temp_ext_milli);

	dev_dbg(dev, "\n %s: ret temp=%ldC ", __func__, *pTemp);
	return 0;
error:
	dev_err(&client->dev, "\n error in file=: %s %s() line=%d: "
		"error=%d ", __FILE__, __func__, __LINE__, value);
	return value;
}

static ssize_t nct1008_show_temp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	s8 temp1 = 0;
	s8 temp = 0;
	u8 temp2 = 0;
	int value = 0;

	if (!dev || !buf || !attr)
		return -EINVAL;

	value = i2c_smbus_read_byte_data(client, LOCAL_TEMP_RD);
	if (value < 0)
		goto error;
	temp1 = value_to_temperature(pdata->ext_range, value);

	value = i2c_smbus_read_byte_data(client, EXT_TEMP_RD_LO);
	if (value < 0)
		goto error;
	temp2 = (value >> 6);
	value = i2c_smbus_read_byte_data(client, EXT_TEMP_RD_HI);
	if (value < 0)
		goto error;
	temp = value_to_temperature(pdata->ext_range, value);

	return snprintf(buf, MAX_STR_PRINT, "%d %d.%d\n",
		temp1, temp, temp2 * 25);

error:
	return snprintf(buf, MAX_STR_PRINT,
		"Error read local/ext temperature\n");
}

static ssize_t nct1008_show_temp_overheat(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	int value;
	s8 temp, temp2;

	/* Local temperature h/w shutdown limit */
	value = i2c_smbus_read_byte_data(client, LOCAL_THERM_LIMIT_WR);
	if (value < 0)
		goto error;
	temp = value_to_temperature(pdata->ext_range, value);

	/* External temperature h/w shutdown limit */
	value = i2c_smbus_read_byte_data(client, EXT_THERM_LIMIT_WR);
	if (value < 0)
		goto error;
	temp2 = value_to_temperature(pdata->ext_range, value);

	return snprintf(buf, MAX_STR_PRINT, "%d %d\n", temp, temp2);
error:
	dev_err(dev, "%s: failed to read temperature-overheat "
		"\n", __func__);
	return snprintf(buf, MAX_STR_PRINT, " Rd overheat Error\n");
}

static ssize_t nct1008_set_temp_overheat(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	long int num;
	int err;
	u8 temp;
	long currTemp;
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	char bufTemp[MAX_STR_PRINT];
	char bufOverheat[MAX_STR_PRINT];
	unsigned int ret;

	if (strict_strtol(buf, 0, &num)) {
		dev_err(dev, "\n file: %s, line=%d return %s() ", __FILE__,
			__LINE__, __func__);
		return -EINVAL;
	}
	if (((int)num < NCT1008_MIN_TEMP) || ((int)num >= NCT1008_MAX_TEMP)) {
		dev_err(dev, "\n file: %s, line=%d return %s() ", __FILE__,
			__LINE__, __func__);
		return -EINVAL;
	}
	/* check for system power down */
	err = nct1008_get_temp(dev, &currTemp);
	if (err)
		goto error;

	currTemp = MILLICELSIUS_TO_CELSIUS(currTemp);

	if (currTemp >= (int)num) {
		ret = nct1008_show_temp(dev, attr, bufTemp);
		ret = nct1008_show_temp_overheat(dev, attr, bufOverheat);
		dev_err(dev, "\nCurrent temp: %s ", bufTemp);
		dev_err(dev, "\nOld overheat limit: %s ", bufOverheat);
		dev_err(dev, "\nReset from overheat: curr temp=%ld, "
			"new overheat temp=%d\n\n", currTemp, (int)num);
	}

	/* External temperature h/w shutdown limit */
	temp = temperature_to_value(pdata->ext_range, (s8)num);
	err = i2c_smbus_write_byte_data(client, EXT_THERM_LIMIT_WR, temp);
	if (err < 0)
		goto error;

	/* Local temperature h/w shutdown limit */
	temp = temperature_to_value(pdata->ext_range, (s8)num);
	err = i2c_smbus_write_byte_data(client, LOCAL_THERM_LIMIT_WR, temp);
	if (err < 0)
		goto error;
	return count;
error:
	dev_err(dev, " %s: failed to set temperature-overheat\n", __func__);
	return err;
}

static ssize_t nct1008_show_temp_alert(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	int value;
	s8 temp, temp2;
	/* External Temperature Throttling limit */
	value = i2c_smbus_read_byte_data(client, EXT_TEMP_HI_LIMIT_HI_BYTE_RD);
	if (value < 0)
		goto error;
	temp2 = value_to_temperature(pdata->ext_range, value);

	/* Local Temperature Throttling limit */
	value = i2c_smbus_read_byte_data(client, LOCAL_TEMP_HI_LIMIT_RD);
	if (value < 0)
		goto error;
	temp = value_to_temperature(pdata->ext_range, value);

	return snprintf(buf, MAX_STR_PRINT, "%d %d\n", temp, temp2);
error:
	dev_err(dev, "%s: failed to read temperature-overheat "
		"\n", __func__);
	return snprintf(buf, MAX_STR_PRINT, " Rd overheat Error\n");
}

static ssize_t nct1008_set_temp_alert(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	long int num;
	int value;
	int err;
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_platform_data *pdata = client->dev.platform_data;

	if (strict_strtol(buf, 0, &num)) {
		dev_err(dev, "\n file: %s, line=%d return %s() ", __FILE__,
			__LINE__, __func__);
		return -EINVAL;
	}
	if (((int)num < NCT1008_MIN_TEMP) || ((int)num >= NCT1008_MAX_TEMP)) {
		dev_err(dev, "\n file: %s, line=%d return %s() ", __FILE__,
			__LINE__, __func__);
		return -EINVAL;
	}

	/* External Temperature Throttling limit */
	value = temperature_to_value(pdata->ext_range, (s8)num);
	err = i2c_smbus_write_byte_data(client, EXT_TEMP_HI_LIMIT_HI_BYTE_WR,
		value);
	if (err < 0)
		goto error;

	/* Local Temperature Throttling limit */
	err = i2c_smbus_write_byte_data(client, LOCAL_TEMP_HI_LIMIT_WR,
		value);
	if (err < 0)
		goto error;

	return count;
error:
	dev_err(dev, "%s: failed to set temperature-alert "
		"\n", __func__);
	return err;
}

static ssize_t nct1008_show_ext_temp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	s8 temp_value;
	int data = 0;
	int data_lo;

	if (!dev || !buf || !attr)
		return -EINVAL;

	/* When reading the full external temperature value, read the
	 * LSB first. This causes the MSB to be locked (that is, the
	 * ADC does not write to it) until it is read */
	data_lo = i2c_smbus_read_byte_data(client, EXT_TEMP_RD_LO);
	if (data_lo < 0) {
		dev_err(&client->dev, "%s: failed to read "
			"ext_temperature, i2c error=%d\n", __func__, data_lo);
		goto error;
	}

	data = i2c_smbus_read_byte_data(client, EXT_TEMP_RD_HI);
	if (data < 0) {
		dev_err(&client->dev, "%s: failed to read "
			"ext_temperature, i2c error=%d\n", __func__, data);
		goto error;
	}

	temp_value = value_to_temperature(pdata->ext_range, data);

	return snprintf(buf, MAX_STR_PRINT, "%d.%d\n", temp_value,
		(25 * (data_lo >> 6)));
error:
	return snprintf(buf, MAX_STR_PRINT, "Error read ext temperature\n");
}

static DEVICE_ATTR(temperature, S_IRUGO, nct1008_show_temp, NULL);
static DEVICE_ATTR(temperature_overheat, (S_IRUGO | (S_IWUSR | S_IWGRP)),
		nct1008_show_temp_overheat, nct1008_set_temp_overheat);
static DEVICE_ATTR(temperature_alert, (S_IRUGO | (S_IWUSR | S_IWGRP)),
		nct1008_show_temp_alert, nct1008_set_temp_alert);
static DEVICE_ATTR(ext_temperature, S_IRUGO, nct1008_show_ext_temp, NULL);

static struct attribute *nct1008_attributes[] = {
	&dev_attr_temperature.attr,
	&dev_attr_temperature_overheat.attr,
	&dev_attr_temperature_alert.attr,
	&dev_attr_ext_temperature.attr,
	NULL
};

static const struct attribute_group nct1008_attr_group = {
	.attrs = nct1008_attributes,
};

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
static void print_reg(const char *reg_name, struct seq_file *s,
		int offset)
{
	struct nct1008_data *nct_data = s->private;
	int ret;

	ret = i2c_smbus_read_byte_data(nct_data->client,
		offset);
	if (ret >= 0)
		seq_printf(s, "Reg %s Addr = 0x%02x Reg 0x%02x "
		"Value 0x%02x\n", reg_name,
		nct_data->client->addr,
			offset, ret);
	else
		seq_printf(s, "%s: line=%d, i2c read error=%d\n",
		__func__, __LINE__, ret);
}

static int dbg_nct1008_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "nct1008 Registers\n");
	seq_printf(s, "------------------\n");
	print_reg("Local Temp Value    ",     s, 0x00);
	print_reg("Ext Temp Value Hi   ",     s, 0x01);
	print_reg("Status              ",     s, 0x02);
	print_reg("Configuration       ",     s, 0x03);
	print_reg("Conversion Rate     ",     s, 0x04);
	print_reg("Local Temp Hi Limit ",     s, 0x05);
	print_reg("Local Temp Lo Limit ",     s, 0x06);
	print_reg("Ext Temp Hi Limit Hi",     s, 0x07);
	print_reg("Ext Temp Hi Limit Lo",     s, 0x13);
	print_reg("Ext Temp Lo Limit Hi",     s, 0x08);
	print_reg("Ext Temp Lo Limit Lo",     s, 0x14);
	print_reg("Ext Temp Value Lo   ",     s, 0x10);
	print_reg("Ext Temp Offset Hi  ",     s, 0x11);
	print_reg("Ext Temp Offset Lo  ",     s, 0x12);
	print_reg("Ext THERM Limit     ",     s, 0x19);
	print_reg("Local THERM Limit   ",     s, 0x20);
	print_reg("THERM Hysteresis    ",     s, 0x21);
	print_reg("Consecutive ALERT   ",     s, 0x22);
	return 0;
}

static int dbg_nct1008_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_nct1008_show, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_nct1008_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init nct1008_debuginit(struct nct1008_data *nct)
{
	int err = 0;
	struct dentry *d;
	d = debugfs_create_file("nct1008", S_IRUGO, NULL,
			(void *)nct, &debug_fops);
	if ((!d) || IS_ERR(d)) {
		dev_err(&nct->client->dev, "Error: %s debugfs_create_file"
			" returned an error\n", __func__);
		err = -ENOENT;
		goto end;
	}
	if (d == ERR_PTR(-ENODEV)) {
		dev_err(&nct->client->dev, "Error: %s debugfs not supported "
			"error=-ENODEV\n", __func__);
		err = -ENODEV;
	} else {
		nct->dent = d;
	}
end:
	return err;
}
#else
static int __init nct1008_debuginit(struct nct1008_data *nct)
{
	return 0;
}
#endif

static int nct1008_enable(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);
	int err;

	err = i2c_smbus_write_byte_data(client, CONFIG_WR,
				  data->config & ~STANDBY_BIT);
	if (err < 0)
		dev_err(&client->dev, "%s, line=%d, i2c write error=%d\n",
		__func__, __LINE__, err);
	return err;
}

static int nct1008_disable(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);
	int err;

	err = i2c_smbus_write_byte_data(client, CONFIG_WR,
				  data->config | STANDBY_BIT);
	if (err < 0)
		dev_err(&client->dev, "%s, line=%d, i2c write error=%d\n",
		__func__, __LINE__, err);
	return err;
}

static int nct1008_disable_alert(struct nct1008_data *data)
{
	struct i2c_client *client = data->client;
	int ret = 0;
	int val;

	/*
	 * Disable ALERT# output, because these chips don't implement
	 * SMBus alert correctly; they should only hold the alert line
	 * low briefly.
	 */
	val = i2c_smbus_read_byte_data(data->client, CONFIG_RD);
	if (val < 0) {
		dev_err(&client->dev, "%s, line=%d, disable alert failed ... "
			"i2c read error=%d\n", __func__, __LINE__, val);
		return val;
	}
	data->config = val | ALERT_BIT;
	ret = i2c_smbus_write_byte_data(client, CONFIG_WR, data->config);
	if (ret)
		dev_err(&client->dev, "%s: fail to disable alert, i2c "
			"write error=%d#\n", __func__, ret);

	return ret;
}

static int nct1008_enable_alert(struct nct1008_data *data)
{
	int val;
	int ret;

	val = i2c_smbus_read_byte_data(data->client, CONFIG_RD);
	if (val < 0) {
		dev_err(&data->client->dev, "%s, line=%d, enable alert "
			"failed ... i2c read error=%d\n", __func__,
			__LINE__, val);
		return val;
	}
	val &= ~(ALERT_BIT | THERM2_BIT);
	ret = i2c_smbus_write_byte_data(data->client, CONFIG_WR, val);
	if (ret) {
		dev_err(&data->client->dev, "%s: fail to enable alert, i2c "
			"write error=%d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static void nct1008_work_func(struct work_struct *work)
{
	struct nct1008_data *data = container_of(work, struct nct1008_data,
						work);
	int err = 0;
	int intr_status = i2c_smbus_read_byte_data(data->client, STATUS_RD);

	if (intr_status < 0) {
		dev_err(&data->client->dev, "%s, line=%d, i2c read error=%d\n",
			__func__, __LINE__, intr_status);
		return;
	}

	intr_status &= (BIT(3) | BIT(4));
	if (!intr_status)
		return;

	if (data->alert_func) {
		err = nct1008_disable_alert(data);
		if (err) {
			dev_err(&data->client->dev,
				"%s: disable alert fail(error=%d)\n",
				__func__, err);
			return;
		}

		data->alert_func(data->alert_data);

		nct1008_enable_alert(data);
	}


	if (err)
		dev_err(&data->client->dev, "%s: fail(error=%d)\n", __func__,
			err);
	else
		pr_debug("%s: done\n", __func__);
}

static irqreturn_t nct1008_irq(int irq, void *dev_id)
{
	struct nct1008_data *data = dev_id;

	schedule_work(&data->work);
	return IRQ_HANDLED;
}

static void nct1008_power_control(struct nct1008_data *data, bool is_enable)
{
	int ret;
	if (!data->nct_reg) {
		data->nct_reg = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR_OR_NULL(data->nct_reg)) {
			dev_warn(&data->client->dev, "Error [%d] in"
				"getting the regulator handle for vdd "
				"of %s\n", (int)data->nct_reg,
				dev_name(&data->client->dev));
			data->nct_reg = NULL;
			return;
		}
	}
	if (is_enable)
		ret = regulator_enable(data->nct_reg);
	else
		ret = regulator_disable(data->nct_reg);

	if (ret < 0)
		dev_err(&data->client->dev, "Error in %s rail vdd_nct1008, "
			"error %d\n", (is_enable) ? "enabling" : "disabling",
			ret);
	else
		dev_info(&data->client->dev, "success in %s rail vdd_nct1008\n",
			(is_enable) ? "enabling" : "disabling");
}

static int nct1008_configure_sensor(struct nct1008_data* data)
{
	struct i2c_client *client = data->client;
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	u8 value;
	s8 temp;
	u8 temp2;
	int err;

	if (!pdata || !pdata->supported_hwrev)
		return -ENODEV;

	/* Place in Standby */
	data->config = STANDBY_BIT;
	err = i2c_smbus_write_byte_data(client, CONFIG_WR, data->config);
	if (err)
		goto error;

	/* External temperature h/w shutdown limit */
	value = temperature_to_value(pdata->ext_range, NCT1008_MAX_TEMP);
	err = i2c_smbus_write_byte_data(client, EXT_THERM_LIMIT_WR, value);
	if (err)
		goto error;

	/* Local temperature h/w shutdown limit */
	value = temperature_to_value(pdata->ext_range, NCT1008_MAX_TEMP);
	err = i2c_smbus_write_byte_data(client, LOCAL_THERM_LIMIT_WR, value);
	if (err)
		goto error;

	/* set extended range mode if needed */
	if (pdata->ext_range)
		data->config |= EXTENDED_RANGE_BIT;
	data->config &= ~(THERM2_BIT | ALERT_BIT);

	err = i2c_smbus_write_byte_data(client, CONFIG_WR, data->config);
	if (err)
		goto error;

	/* Temperature conversion rate */
	err = i2c_smbus_write_byte_data(client, CONV_RATE_WR, pdata->conv_rate);
	if (err)
		goto error;

	/* Setup local hi and lo limits */
	err = i2c_smbus_write_byte_data(client,
		LOCAL_TEMP_HI_LIMIT_WR, NCT1008_MAX_TEMP);
	if (err)
		goto error;

	err = i2c_smbus_write_byte_data(client,
		LOCAL_TEMP_LO_LIMIT_WR, 0);
	if (err)
		goto error;

	/* Setup external hi and lo limits */
	err = i2c_smbus_write_byte_data(client,
		EXT_TEMP_LO_LIMIT_HI_BYTE_WR, 0);
	if (err)
		goto error;
	err = i2c_smbus_write_byte_data(client, EXT_TEMP_HI_LIMIT_HI_BYTE_WR,
			NCT1008_MAX_TEMP);
	if (err)
		goto error;

	/* read initial temperature */
	value = i2c_smbus_read_byte_data(client, LOCAL_TEMP_RD);
	if (value < 0) {
		err = value;
		goto error;
	}
	temp = value_to_temperature(pdata->ext_range, value);
	dev_dbg(&client->dev, "\n initial local temp = %d ", temp);

	value = i2c_smbus_read_byte_data(client, EXT_TEMP_RD_LO);
	if (value < 0) {
		err = value;
		goto error;
	}
	temp2 = (value >> 6);
	value = i2c_smbus_read_byte_data(client, EXT_TEMP_RD_HI);
	if (value < 0) {
		err = value;
		goto error;
	}
	temp = value_to_temperature(pdata->ext_range, value);

	if (temp2 > 0)
		dev_dbg(&client->dev, "\n initial ext temp = %d.%d deg",
				temp, temp2 * 25);
	else
		dev_dbg(&client->dev, "\n initial ext temp = %d.0 deg", temp);

	/* Remote channel offset */
	err = i2c_smbus_write_byte_data(client, OFFSET_WR, pdata->offset / 4);
	if (err < 0)
		goto error;

	/* Remote channel offset fraction (quarters) */
	err = i2c_smbus_write_byte_data(client, OFFSET_QUARTER_WR,
					(pdata->offset % 4) << 6);
	if (err < 0)
		goto error;

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &nct1008_attr_group);
	if (err < 0) {
		dev_err(&client->dev, "\n sysfs create err=%d ", err);
		goto error;
	}

	return 0;
error:
	dev_err(&client->dev, "\n exit %s, err=%d ", __func__, err);
	return err;
}

static int nct1008_configure_irq(struct nct1008_data *data)
{
	INIT_WORK(&data->work, nct1008_work_func);

	if (data->client->irq < 0)
		return 0;
	else
		return request_irq(data->client->irq, nct1008_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			DRIVER_NAME, data);
}

static unsigned int get_ext_mode_delay_ms(unsigned int conv_rate)
{
	switch (conv_rate) {
	case 0:
		return 16000;
	case 1:
		return 8000;
	case 2:
		return 4000;
	case 3:
		return 2000;
	case 4:
		return 1000;
	case 5:
		return 500;
	case 6:
		return 250;
	case 7:
		return 125;
	case 9:
		return 32;
	case 10:
		return 16;
	case 8:
	default:
		return 63;
	}
}

int nct1008_thermal_get_temp(struct nct1008_data *data, long *temp)
{
	return nct1008_get_temp(&data->client->dev, temp);
}

int nct1008_thermal_set_limits(struct nct1008_data *data,
				long lo_limit_milli,
				long hi_limit_milli)
{
	int err;
	u8 value;
	bool extended_range = data->plat_data.ext_range;
	long lo_limit = MILLICELSIUS_TO_CELSIUS(lo_limit_milli);
	long hi_limit = MILLICELSIUS_TO_CELSIUS(hi_limit_milli);

	if (lo_limit >= hi_limit)
		return -EINVAL;

	if (data->current_lo_limit == lo_limit &&
		data->current_hi_limit == hi_limit)
		return 0;

	if (data->current_lo_limit != lo_limit) {
		value = temperature_to_value(extended_range, lo_limit);
		pr_debug("%s: %d\n", __func__, value);
		err = i2c_smbus_write_byte_data(data->client,
				EXT_TEMP_LO_LIMIT_HI_BYTE_WR, value);
		if (err)
			return err;

		data->current_lo_limit = lo_limit;
	}

	if (data->current_hi_limit != hi_limit) {
		value = temperature_to_value(extended_range, hi_limit);
		pr_debug("%s: %d\n", __func__, value);
		err = i2c_smbus_write_byte_data(data->client,
				EXT_TEMP_HI_LIMIT_HI_BYTE_WR, value);
		if (err)
			return err;

		data->current_hi_limit = hi_limit;
	}

	return 0;
}

int nct1008_thermal_set_alert(struct nct1008_data *data,
				void (*alert_func)(void *),
				void *alert_data)
{
	data->alert_data = alert_data;
	data->alert_func = alert_func;

	return 0;
}

int nct1008_thermal_set_shutdown_temp(struct nct1008_data *data,
					long shutdown_temp_milli)
{
	struct i2c_client *client = data->client;
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	int err;
	u8 value;
	long shutdown_temp;

	shutdown_temp = MILLICELSIUS_TO_CELSIUS(shutdown_temp_milli);

	/* External temperature h/w shutdown limit */
	value = temperature_to_value(pdata->ext_range, shutdown_temp);
	err = i2c_smbus_write_byte_data(client, EXT_THERM_LIMIT_WR, value);
	if (err)
		return err;

	/* Local temperature h/w shutdown limit */
	value = temperature_to_value(pdata->ext_range, shutdown_temp);
	err = i2c_smbus_write_byte_data(client, LOCAL_THERM_LIMIT_WR, value);
	if (err)
		return err;

	return 0;
}

/*
 * Manufacturer(OnSemi) recommended sequence for
 * Extended Range mode is as follows
 * 1. Place in Standby
 * 2. Scale the THERM and ALERT limits
 *	appropriately(for Extended Range mode).
 * 3. Enable Extended Range mode.
 *	ALERT mask/THERM2 mode may be done here
 *	as these are not critical
 * 4. Set Conversion Rate as required
 * 5. Take device out of Standby
 */

/*
 * function nct1008_probe takes care of initial configuration
 */
static int nct1008_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct nct1008_data *data;
	int err;
	unsigned int delay;

	data = kzalloc(sizeof(struct nct1008_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	memcpy(&data->plat_data, client->dev.platform_data,
		sizeof(struct nct1008_platform_data));
	i2c_set_clientdata(client, data);

	nct1008_power_control(data, true);
	/* extended range recommended steps 1 through 4 taken care
	 * in nct1008_configure_sensor function */
	err = nct1008_configure_sensor(data);	/* sensor is in standby */
	if (err < 0) {
		dev_err(&client->dev, "\n error file: %s : %s(), line=%d ",
			__FILE__, __func__, __LINE__);
		goto error;
	}

	err = nct1008_configure_irq(data);
	if (err < 0) {
		dev_err(&client->dev, "\n error file: %s : %s(), line=%d ",
			__FILE__, __func__, __LINE__);
		goto error;
	}
	dev_info(&client->dev, "%s: initialized\n", __func__);

	/* extended range recommended step 5 is in nct1008_enable function */
	err = nct1008_enable(client);		/* sensor is running */
	if (err < 0) {
		dev_err(&client->dev, "Error: %s, line=%d, error=%d\n",
			__func__, __LINE__, err);
		goto error;
	}

	err = nct1008_debuginit(data);
	if (err < 0)
		err = 0; /* without debugfs we may continue */

	/* switch to extended mode reports correct temperature
	 * from next measurement cycle */
	if (data->plat_data.ext_range) {
		delay = get_ext_mode_delay_ms(
			data->plat_data.conv_rate);
		msleep(delay); /* 63msec for default conv rate 0x8 */
	}

	/* notify callback that probe is done */
	if (data->plat_data.probe_callback)
		data->plat_data.probe_callback(data);

	return 0;

error:
	dev_err(&client->dev, "\n exit %s, err=%d ", __func__, err);
	nct1008_power_control(data, false);
	if (data->nct_reg)
		regulator_put(data->nct_reg);
	kfree(data);
	return err;
}

static int nct1008_remove(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);

	if (data->dent)
		debugfs_remove(data->dent);

	free_irq(data->client->irq, data);
	cancel_work_sync(&data->work);
	sysfs_remove_group(&client->dev.kobj, &nct1008_attr_group);
	nct1008_power_control(data, false);
	if (data->nct_reg)
		regulator_put(data->nct_reg);

	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int nct1008_suspend(struct i2c_client *client, pm_message_t state)
{
	int err;

	disable_irq(client->irq);
	err = nct1008_disable(client);
	return err;
}

static int nct1008_resume(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);
	int err;

	err = nct1008_enable(client);
	if (err < 0) {
		dev_err(&client->dev, "Error: %s, error=%d\n",
			__func__, err);
		return err;
	}
	enable_irq(client->irq);
	schedule_work(&data->work);

	return 0;
}
#endif

static const struct i2c_device_id nct1008_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nct1008_id);

static struct i2c_driver nct1008_driver = {
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe		= nct1008_probe,
	.remove		= nct1008_remove,
	.id_table	= nct1008_id,
#ifdef CONFIG_PM
	.suspend	= nct1008_suspend,
	.resume		= nct1008_resume,
#endif
};

static int __init nct1008_init(void)
{
	return i2c_add_driver(&nct1008_driver);
}

static void __exit nct1008_exit(void)
{
	i2c_del_driver(&nct1008_driver);
}

MODULE_DESCRIPTION("Temperature sensor driver for OnSemi NCT1008");
MODULE_LICENSE("GPL");

module_init(nct1008_init);
module_exit(nct1008_exit);
