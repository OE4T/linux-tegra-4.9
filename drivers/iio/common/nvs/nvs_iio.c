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
 * The master enable is an enhancement of the standard IIO enable/disable
 * procedure.  Consider a device that contains multiple sensors.  To enable all
 * the sensors with the standard IIO enable mechanism, the device would be
 * powered off and on for each sensor that was enabled.  By having a master
 * enable, the device does not have to be powerered down for each buffer
 * disable but only when the master enable is disabled, thereby improving
 * efficiency.
 */
/* This NVS kernel driver has a test mechanism for sending specific data up the
 * SW stack by writing the requested data value to the IIO raw sysfs attribute.
 * That data will be sent anytime there would normally be new data from the
 * device.
 * The feature is disabled whenever the device is disabled.  It remains
 * disabled until the IIO raw sysfs attribute is written to again.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>
#include <linux/nvs.h>


#define NVS_IIO_DRIVER_VERSION		(200)
#define NVS_IIO_SENSOR_TYPE_ERR		(0xFF)
#define NVS_ATTRS_ARRAY_SIZE		(11)

enum NVS_ATTR {
	NVS_ATTR_ENABLE,
	NVS_ATTR_PART,
	NVS_ATTR_VENDOR,
	NVS_ATTR_VERSION,
	NVS_ATTR_MILLIAMP,
	NVS_ATTR_FIFO_RSRV_EVNT_CNT,
	NVS_ATTR_FIFO_MAX_EVNT_CNT,
	NVS_ATTR_FLAGS,
	NVS_ATTR_MATRIX,
};

enum NVS_DBG {
	NVS_INFO_DATA = 0,
	NVS_INFO_VER,
	NVS_INFO_ERRS,
	NVS_INFO_RESET,
	NVS_INFO_REGS,
	NVS_INFO_DBG,
	NVS_INFO_DBG_DATA,
	NVS_INFO_DBG_BUF,
	NVS_INFO_DBG_IRQ,
	NVS_INFO_LIMIT_MAX,
};

struct nvs_state {
	void *client;
	struct device *dev;
	struct nvs_fn_dev *fn_dev;
	struct sensor_cfg *cfg;
	struct iio_trigger *trig;
	struct iio_chan_spec *ch;
	struct attribute *attrs[NVS_ATTRS_ARRAY_SIZE];
	struct attribute_group attr_group;
	struct iio_info info;
	bool flush;
	int batch_flags;
	unsigned int batch_period_us;
	unsigned int batch_timeout_us;
	unsigned int dbg;
	long dbg_data_lock;
	s64 ts;
	u8 *buf;
};

static u8 iio_sensor_type[] = {
	NVS_IIO_SENSOR_TYPE_ERR,
	IIO_ACCEL,		/* SENSOR_TYPE_ACCELEROMETER */
	IIO_MAGN,		/* SENSOR_TYPE_MAGNETIC_FIELD */
	IIO_ORIENTATION,	/* SENSOR_TYPE_ORIENTATION */
	IIO_ANGL_VEL,		/* SENSOR_TYPE_GYROSCOPE */
	IIO_LIGHT,		/* SENSOR_TYPE_LIGHT */
	IIO_PRESSURE,		/* SENSOR_TYPE_PRESSURE */
	IIO_TEMP,		/* SENSOR_TYPE_TEMPERATURE */
	IIO_PROXIMITY,		/* SENSOR_TYPE_PROXIMITY */
	IIO_GRAVITY,		/* SENSOR_TYPE_GRAVITY */
	IIO_LINEAR_ACCEL,	/* SENSOR_TYPE_LINEAR_ACCELERATION */
	IIO_ROT,		/* SENSOR_TYPE_ROTATION_VECTOR */
	IIO_HUMIDITY,		/* SENSOR_TYPE_RELATIVE_HUMIDITY */
	IIO_TEMP,		/* SENSOR_TYPE_AMBIENT_TEMPERATURE */
	IIO_MAGN_UNCAL,		/* SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED */
	IIO_GAME_ROT,		/* SENSOR_TYPE_GAME_ROTATION_VECTOR */
	IIO_ANGLVEL_UNCAL,	/* SENSOR_TYPE_GYROSCOPE_UNCALIBRATED */
	IIO_MOTION,		/* SENSOR_TYPE_SIGNIFICANT_MOTION */
	IIO_STEP,		/* SENSOR_TYPE_STEP_DETECTOR */
	IIO_STEP_COUNT,		/* SENSOR_TYPE_STEP_COUNTER */
	IIO_GEOMAGN_ROT,	/* SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR */
	IIO_HEART_RATE,		/* SENSOR_TYPE_HEART_RATE */
	IIO_INCLI,		/* SENSOR_TYPE_TILT_DETECTOR */
	IIO_GESTURE_WAKE,	/* SENSOR_TYPE_WAKE_GESTURE */
	IIO_GESTURE_GLANCE,	/* SENSOR_TYPE_GLANCE_GESTURE */
	IIO_GESTURE_PICKUP,	/* SENSOR_TYPE_PICK_UP_GESTURE */
};


static void nvs_mutex_lock(void *handle)
{
	struct iio_dev *indio_dev = (struct iio_dev *)handle;

	mutex_lock(&indio_dev->mlock);
}

static void nvs_mutex_unlock(void *handle)
{
	struct iio_dev *indio_dev = (struct iio_dev *)handle;

	mutex_unlock(&indio_dev->mlock);
}

static unsigned int nvs_buf_index(unsigned int size, unsigned int *bytes)
{
	unsigned int index;

	if (!(*bytes % size))
		index = *bytes;
	else
		index = *bytes - *bytes % size + size;
	*bytes = index + size;
	return index;
}

static int nvs_buf_i(struct iio_dev *indio_dev, unsigned int ch)
{
	unsigned int buf_i = 0;
	unsigned int n;
	unsigned int i;

	if (!iio_scan_mask_query(indio_dev, indio_dev->buffer, ch))
		return -EINVAL;

	n = 0;
	for (i = 0; i <= ch; i++) {
		if (iio_scan_mask_query(indio_dev, indio_dev->buffer, i))
			buf_i = nvs_buf_index(indio_dev->channels[i].scan_type.
					      storagebits / 8, &n);
	}
	return buf_i;
}

static ssize_t nvs_dbg_data(struct iio_dev *indio_dev, char *buf)
{
	struct nvs_state *st = iio_priv(indio_dev);
	struct iio_chan_spec const *ch;
	ssize_t t;
	unsigned int shift;
	unsigned int buf_i;
	unsigned int ch_n;
	unsigned int n;
	unsigned int i;
	u64 data;

	t = sprintf(buf, "data: ");
	n = 0;
	for (i = 0; i < indio_dev->num_channels - 1; i++) {
		ch = &indio_dev->channels[i];
		if (iio_scan_mask_query(indio_dev, indio_dev->buffer, i)) {
			ch_n = ch->scan_type.storagebits / 8;
			buf_i = nvs_buf_index(ch_n, &n);
		} else {
			t += sprintf(buf + t, "disabled ");
			continue;
		}

		if (ch_n <= sizeof(data)) {
			data = 0LL;
			memcpy(&data, &st->buf[buf_i], ch_n);
			if (ch->scan_type.sign == 's') {
				shift = 64 - ch->scan_type.realbits;
				t += sprintf(buf + t, "%lld ",
					     (s64)(data << shift) >> shift);
			} else {
				t += sprintf(buf + t, "%llu ", data);
			}
		} else {
			t += sprintf(buf + t, "ERR ");
		}
	}
	t += sprintf(buf + t, "ts=%lld\n", st->ts);
	return t;
}

static int nvs_buf_push(struct iio_dev *indio_dev, unsigned char *data, s64 ts)
{
	struct nvs_state *st = iio_priv(indio_dev);
	char char_buf[128];
	unsigned int n;
	unsigned int i;
	unsigned int src_i = 0;
	unsigned int dst_i;
	unsigned int bytes = 0;
	int ret = 0;

	if (data) {
		for (i = 0; i < indio_dev->num_channels - 1; i++) {
			if (iio_scan_mask_query(indio_dev,
						indio_dev->buffer, i)) {
				n = indio_dev->
					 channels[i].scan_type.storagebits / 8;
				dst_i = nvs_buf_index(n, &bytes);
				if (!(st->dbg_data_lock & (1 << i)))
					memcpy(&st->buf[dst_i],
					       &data[src_i], n);
				src_i += n;
			}
		}
	}
	if (!ts)
		st->flush = false;
	st->ts = ts;
	if (indio_dev->buffer->scan_timestamp) {
		n = sizeof(ts);
		dst_i = nvs_buf_index(n, &bytes);
		memcpy(&st->buf[dst_i], &ts, n);
	}
	if (iio_buffer_enabled(indio_dev)) {
		ret = iio_push_to_buffers(indio_dev, st->buf);
		if (*st->fn_dev->sts & NVS_STS_SPEW_BUF) {
			for (i = 0; i < bytes; i++)
				dev_info(st->dev, "buf[%u]=%x\n",
					 i, st->buf[i]);
			dev_info(st->dev, "ts=%lld\n", ts);
		}
	}
	if (*st->fn_dev->sts & NVS_STS_SPEW_DATA) {
		nvs_dbg_data(indio_dev, char_buf);
		dev_info(st->dev, "%s", char_buf);
	}
	return ret;
}

static int nvs_handler(void *handle, void *buffer, s64 ts)
{
	struct iio_dev *indio_dev = (struct iio_dev *)handle;
	unsigned char *buf = buffer;

	nvs_buf_push(indio_dev, buf, ts);
	return 0;
}

static int nvs_enable(struct iio_dev *indio_dev, bool en)
{
	struct nvs_state *st = iio_priv(indio_dev);
	unsigned int i;
	int enable = 0;

	if (en) {
		for (i = 0; i < indio_dev->num_channels - 1; i++) {
			if (iio_scan_mask_query(indio_dev,
						indio_dev->buffer, i))
				enable |= 1 << i;
		}

		if (i >= indio_dev->num_channels)
			return -EINVAL;
	} else {
		st->dbg_data_lock = 0;
	}
	return st->fn_dev->enable(st->client, st->cfg->snsr_id, enable);
}

static ssize_t nvs_attr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvs_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	const char *msg;
	unsigned int new;
	unsigned int old = 0;
	int ret;

	ret = kstrtouint(buf, 10, &new);
	if (ret)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	if (*st->fn_dev->sts & (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND)) {
		mutex_unlock(&indio_dev->mlock);
		return -EPERM;
	}

	switch (this_attr->address) {
	case NVS_ATTR_ENABLE:
		msg = "ATTR_ENABLE";
		old = st->fn_dev->enable(st->client, st->cfg->snsr_id, -1);
		ret = nvs_enable(indio_dev, (bool)new);
		if (ret > 0)
			ret = 0;
		break;

	default:
		msg = "ATTR_UNKNOWN";
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);
	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG) {
		if (ret)
			dev_err(st->dev, "%s %s %d->%d ERR=%d\n",
				__func__, msg, old, new, ret);
		else
			dev_info(st->dev, "%s %s %d->%d\n",
				 __func__, msg, old, new);
	}
	if (ret)
		return ret;

	return count;
}

static ssize_t nvs_attr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvs_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	switch (this_attr->address) {
	case NVS_ATTR_ENABLE:
		return sprintf(buf, "%x\n",
			       st->fn_dev->enable(st->client,
						  st->cfg->snsr_id, -1));

	case NVS_ATTR_PART:
		return sprintf(buf, "%s\n", st->cfg->part);

	case NVS_ATTR_VENDOR:
		return sprintf(buf, "%s\n", st->cfg->vendor);

	case NVS_ATTR_VERSION:
		return sprintf(buf, "%d\n", st->cfg->version);

	case NVS_ATTR_MILLIAMP:
		return sprintf(buf, "%d.%06u\n",
			       st->cfg->milliamp.ival,
			       st->cfg->milliamp.fval);

	case NVS_ATTR_FIFO_RSRV_EVNT_CNT:
		return sprintf(buf, "%u\n", st->cfg->fifo_rsrv_evnt_cnt);

	case NVS_ATTR_FIFO_MAX_EVNT_CNT:
		return sprintf(buf, "%u\n", st->cfg->fifo_max_evnt_cnt);

	case NVS_ATTR_FLAGS:
		return sprintf(buf, "%u\n", st->cfg->flags);

	case NVS_ATTR_MATRIX:
		return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			       st->cfg->matrix[0],
			       st->cfg->matrix[1],
			       st->cfg->matrix[2],
			       st->cfg->matrix[3],
			       st->cfg->matrix[4],
			       st->cfg->matrix[5],
			       st->cfg->matrix[6],
			       st->cfg->matrix[7],
			       st->cfg->matrix[8]);

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static ssize_t nvs_info_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvs_state *st = iio_priv(indio_dev);
	unsigned int dbg;
	int ret;

	ret = kstrtouint(buf, 10, &dbg);
	if ((ret) || (dbg >= sizeof(*st->fn_dev->sts) * 8))
		return -EINVAL;

	st->dbg = dbg;
	switch (dbg) {
	case NVS_INFO_DATA:
		*st->fn_dev->sts &= ~NVS_STS_SPEW_COMMON;
		break;

	case NVS_INFO_DBG:
		*st->fn_dev->sts ^= NVS_STS_SPEW_MSG;
		break;

	case NVS_INFO_DBG_DATA:
		*st->fn_dev->sts ^= NVS_STS_SPEW_DATA;
		break;

	case NVS_INFO_DBG_BUF:
		*st->fn_dev->sts ^= NVS_STS_SPEW_BUF;
		break;

	case NVS_INFO_DBG_IRQ:
		*st->fn_dev->sts ^= NVS_STS_SPEW_IRQ;
		break;

	default:
		break;
	}

	return count;
}

static ssize_t nvs_info_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nvs_state *st = iio_priv(indio_dev);
	enum NVS_DBG dbg;
	unsigned int i;
	int ret = -EINVAL;

	dbg = st->dbg;
	st->dbg = NVS_INFO_DATA;
	switch (dbg) {
	case NVS_INFO_DATA:
		return nvs_dbg_data(indio_dev, buf);

	case NVS_INFO_VER:
		return sprintf(buf, "version=%u\n", NVS_IIO_DRIVER_VERSION);

	case NVS_INFO_ERRS:
		i = *st->fn_dev->errs;
		*st->fn_dev->errs = 0;
		return sprintf(buf, "error count=%u\n", i);

	case NVS_INFO_RESET:
		if (st->fn_dev->reset) {
			mutex_lock(&indio_dev->mlock);
			ret = st->fn_dev->reset(st->client, st->cfg->snsr_id);
			mutex_unlock(&indio_dev->mlock);
		}
		if (ret)
			return sprintf(buf, "reset ERR\n");
		else
			return sprintf(buf, "reset done\n");

	case NVS_INFO_REGS:
		if (st->fn_dev->regs)
			return st->fn_dev->regs(st->client,
						st->cfg->snsr_id, buf);
		break;

	case NVS_INFO_DBG:
		return sprintf(buf, "DBG spew=%x\n",
			       !!(*st->fn_dev->sts & NVS_STS_SPEW_MSG));

	case NVS_INFO_DBG_DATA:
		return sprintf(buf, "DATA spew=%x\n",
			       !!(*st->fn_dev->sts & NVS_STS_SPEW_DATA));

	case NVS_INFO_DBG_BUF:
		return sprintf(buf, "BUF spew=%x\n",
			       !!(*st->fn_dev->sts & NVS_STS_SPEW_BUF));

	case NVS_INFO_DBG_IRQ:
		return sprintf(buf, "IRQ spew=%x\n",
			       !!(*st->fn_dev->sts & NVS_STS_SPEW_IRQ));

	default:
		break;
	}

	return ret;
}

static DEVICE_ATTR(nvs, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_info_show, nvs_info_store);
static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		       nvs_attr_show, nvs_attr_store, NVS_ATTR_ENABLE);
static IIO_DEVICE_ATTR(part, S_IRUGO,
		       nvs_attr_show, NULL, NVS_ATTR_PART);
static IIO_DEVICE_ATTR(vendor, S_IRUGO,
		       nvs_attr_show, NULL, NVS_ATTR_VENDOR);
static IIO_DEVICE_ATTR(version, S_IRUGO,
		       nvs_attr_show, NULL, NVS_ATTR_VERSION);
static IIO_DEVICE_ATTR(milliamp, S_IRUGO,
		       nvs_attr_show, NULL, NVS_ATTR_MILLIAMP);
static IIO_DEVICE_ATTR(fifo_reserved_event_count, S_IRUGO | S_IWUSR | S_IWGRP,
		       nvs_attr_show, nvs_attr_store,
		       NVS_ATTR_FIFO_RSRV_EVNT_CNT);
static IIO_DEVICE_ATTR(fifo_max_event_count, S_IRUGO | S_IWUSR | S_IWGRP,
		       nvs_attr_show, nvs_attr_store,
		       NVS_ATTR_FIFO_MAX_EVNT_CNT);
static IIO_DEVICE_ATTR(flags, S_IRUGO,
		       nvs_attr_show, NULL, NVS_ATTR_FLAGS);
static IIO_DEVICE_ATTR(matrix, S_IRUGO,
		       nvs_attr_show, NULL, NVS_ATTR_MATRIX);

static struct attribute *nvs_attrs[] = {
	&dev_attr_nvs.attr,
	&iio_dev_attr_enable.dev_attr.attr,
	&iio_dev_attr_part.dev_attr.attr,
	&iio_dev_attr_vendor.dev_attr.attr,
	&iio_dev_attr_version.dev_attr.attr,
	&iio_dev_attr_milliamp.dev_attr.attr,
	&iio_dev_attr_fifo_reserved_event_count.dev_attr.attr,
	&iio_dev_attr_fifo_max_event_count.dev_attr.attr,
	&iio_dev_attr_flags.dev_attr.attr,
	&iio_dev_attr_matrix.dev_attr.attr,
	NULL
};

static int nvs_attr(struct iio_dev *indio_dev)
{
	struct nvs_state *st = iio_priv(indio_dev);
	unsigned int n;
	unsigned int i;

	BUG_ON(NVS_ATTRS_ARRAY_SIZE < ARRAY_SIZE(nvs_attrs));
	memcpy(st->attrs, nvs_attrs, sizeof(st->attrs));
	/* test if matrix data */
	for (i = 0; i < ARRAY_SIZE(st->cfg->matrix); i++) {
		if (st->cfg->matrix[i])
			break;
	}
	if (i >= ARRAY_SIZE(st->cfg->matrix)) {
		/* remove matrix attribute */
		for (i = 0; i < ARRAY_SIZE(st->attrs); i++) {
			if (st->attrs[i] == &iio_dev_attr_matrix.
					    dev_attr.attr) {
				do {
					n = i + 1;
					st->attrs[i] = st->attrs[n];
					i++;
				} while (st->attrs[i] != NULL);
				break;
			}
		}
	}

	st->attr_group.attrs = st->attrs;
	return 0;
}

static int nvs_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct nvs_state *st = iio_priv(indio_dev);
	unsigned int n;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = nvs_buf_i(indio_dev, chan->scan_index);
		if (ret < 0)
			return ret;

		*val = 0;
		n = chan->scan_type.storagebits / 8;
		if (n > sizeof(val))
			n = sizeof(val);
		memcpy(val, &st->buf[ret], n);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_BATCH_FLUSH:
		*val = (int)st->flush;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_BATCH_PERIOD:
		if (st->fn_dev->enable(st->client, st->cfg->snsr_id, -1))
			*val = st->batch_period_us;
		else
			*val = st->cfg->delay_us_min;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_BATCH_TIMEOUT:
		if (st->fn_dev->enable(st->client, st->cfg->snsr_id, -1))
			*val = st->batch_timeout_us;
		else
			*val = st->cfg->delay_us_max;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_BATCH_FLAGS:
		*val = st->batch_flags;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->cfg->scale.ival;
		*val2 = st->cfg->scale.fval;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		*val = st->cfg->offset.ival;
		*val2 = st->cfg->offset.fval;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_THRESHOLD_LOW:
		*val = st->cfg->thresh_lo;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_THRESHOLD_HIGH:
		*val = st->cfg->thresh_hi;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PEAK:
		*val = st->cfg->max_range.ival;
		*val2 = st->cfg->max_range.fval;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_PEAK_SCALE:
		*val = st->cfg->resolution.ival;
		*val2 = st->cfg->resolution.fval;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int nvs_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask)
{
	struct nvs_state *st = iio_priv(indio_dev);
	char *msg;
	int old = 0;
	int old2 = 0;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (*st->fn_dev->sts & (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND)) {
		mutex_unlock(&indio_dev->mlock);
		return -EPERM;
	}

	switch (mask) {
	case IIO_CHAN_INFO_BATCH_FLUSH:
		msg = "IIO_CHAN_INFO_BATCH_FLUSH";
		old = (int)st->flush;
		st->flush = true;
		if (st->fn_dev->flush) {
			ret = st->fn_dev->flush(st->client, st->cfg->snsr_id);
			if (ret) {
				nvs_buf_push(indio_dev, st->buf, 0);
				ret = 0;
			}
		} else {
			nvs_buf_push(indio_dev, st->buf, 0);
		}
		break;

	case IIO_CHAN_INFO_BATCH_PERIOD:
		msg = "IIO_CHAN_INFO_BATCH_PERIOD";
		old = st->batch_period_us;
		if (val < st->cfg->delay_us_min)
			val = st->cfg->delay_us_min;
		if (st->fn_dev->batch) {
			ret = st->fn_dev->batch(st->client, st->cfg->snsr_id,
						st->batch_flags,
						val,
						st->batch_timeout_us);
			if (!ret)
				st->batch_period_us = val;
		} else {
			if (st->batch_timeout_us)
				ret = -EINVAL;
		}
		break;

	case IIO_CHAN_INFO_BATCH_TIMEOUT:
		msg = "IIO_CHAN_INFO_BATCH_TIMEOUT";
		old = st->batch_timeout_us;
		st->batch_timeout_us = val;
		break;

	case IIO_CHAN_INFO_BATCH_FLAGS:
		msg = "IIO_CHAN_INFO_BATCH_FLAGS";
		old = st->batch_flags;
		st->batch_flags = val;
		break;

	case IIO_CHAN_INFO_SCALE:
		msg = "IIO_CHAN_INFO_SCALE";
		old = st->cfg->scale.ival;
		old2 = st->cfg->scale.fval;
		if (st->fn_dev->scale) {
			ret = st->fn_dev->scale(st->client,
						st->cfg->snsr_id, val);
		} else {
			st->cfg->scale.ival = val;
			st->cfg->scale.fval = val2;
		}
		break;

	case IIO_CHAN_INFO_OFFSET:
		msg = "IIO_CHAN_INFO_OFFSET";
		old = st->cfg->offset.ival;
		old2 = st->cfg->offset.fval;
		if (st->fn_dev->offset) {
			ret = st->fn_dev->offset(st->client,
						 st->cfg->snsr_id, val);
		} else {
			st->cfg->offset.ival = val;
			st->cfg->offset.fval = val2;
		}
		break;

	case IIO_CHAN_INFO_THRESHOLD_LOW:
		msg = "IIO_CHAN_INFO_THRESHOLD_LOW";
		old = st->cfg->thresh_lo;
		if (st->fn_dev->thresh_lo)
			ret = st->fn_dev->thresh_lo(st->client,
						    st->cfg->snsr_id, val);
		else
			st->cfg->thresh_lo = val;
		break;

	case IIO_CHAN_INFO_THRESHOLD_HIGH:
		msg = "IIO_CHAN_INFO_THRESHOLD_HIGH";
		old = st->cfg->thresh_lo;
		if (st->fn_dev->thresh_hi)
			ret = st->fn_dev->thresh_hi(st->client,
						    st->cfg->snsr_id, val);
		else
			st->cfg->thresh_hi = val;
		break;

	case IIO_CHAN_INFO_PEAK:
		msg = "IIO_CHAN_INFO_PEAK";
		old = st->cfg->max_range.ival;
		old2 = st->cfg->max_range.fval;
		if (st->fn_dev->max_range) {
			ret = st->fn_dev->max_range(st->client,
						    st->cfg->snsr_id, val);
		} else {
			st->cfg->max_range.ival = val;
			st->cfg->max_range.fval = val2;
		}
		break;

	case IIO_CHAN_INFO_PEAK_SCALE:
		msg = "IIO_CHAN_INFO_PEAK_SCALE";
		old = st->cfg->resolution.ival;
		old2 = st->cfg->resolution.fval;
		if (st->fn_dev->resolution) {
			ret = st->fn_dev->resolution(st->client,
						     st->cfg->snsr_id, val);
		} else {
			st->cfg->resolution.ival = val;
			st->cfg->resolution.fval = val2;
		}
		break;

	case IIO_CHAN_INFO_RAW:
		/* writing to raw is a debug feature allowing a sticky data
		   value to be pushed up and the same value pushed until the
		   device is turned off.
		 */
		msg = "IIO_CHAN_INFO_RAW";
		ret = nvs_buf_i(indio_dev, chan->scan_index);
		if (ret >= 0) {
			memcpy(&st->buf[ret], &val,
			       chan->scan_type.storagebits / 8);
			st->dbg_data_lock |= (1 << chan->scan_index);
			st->ts++;
			ret = nvs_buf_push(indio_dev, st->buf, st->ts);
		}
		break;

	default:
		msg = "IIO_CHAN_INFO_UNKNOWN";
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);
	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG) {
		if (ret)
			dev_err(st->dev, "%s c=%d %d:%d->%d:%d ERR=%d\n",
				msg, chan->scan_index,
				old, old2, val, val2, ret);
		else
			dev_info(st->dev, "%s %s chan=%d %d:%d->%d:%d\n",
				 __func__, msg, chan->scan_index,
				 old, old2, val, val2);
	}
	return ret;
}

static int nvs_buffer_preenable(struct iio_dev *indio_dev)
{
	struct nvs_state *st = iio_priv(indio_dev);

	if (*st->fn_dev->sts & (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND))
		return -EINVAL;

	return iio_sw_buffer_preenable(indio_dev);
}

static int nvs_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;

	ret = nvs_enable(indio_dev, true);
	/* never return > 0 to IIO buffer engine */
	if (ret > 0)
		ret = 0;
	return ret;
}

static const struct iio_buffer_setup_ops nvs_buffer_setup_ops = {
	/* iio_sw_buffer_preenable:
	 * Generic function for equal sized ring elements + 64 bit timestamp
	 * Assumes that any combination of channels can be enabled.
	 * Typically replaced to implement restrictions on what combinations
	 * can be captured (hardware scan modes).
	 */
	.preenable = &nvs_buffer_preenable,
	/* iio_triggered_buffer_postenable:
	 * Generic function that simply attaches the pollfunc to the trigger.
	 * Replace this to mess with hardware state before we attach the
	 * trigger.
	 */
	.postenable = &nvs_buffer_postenable,
	/* this driver relies on the NVS HAL to power off this device with the
	 * master enable.
	 *.predisable = N/A
	 *.postdisable = N/A
	 */
};

static const long nvs_info_mask_dflt =	BIT(IIO_CHAN_INFO_RAW) |
					BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					BIT(IIO_CHAN_INFO_PEAK) |
					BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					BIT(IIO_CHAN_INFO_SCALE) |
					BIT(IIO_CHAN_INFO_OFFSET);

static const long nvs_info_mask_shared_by_type_dflt =
					BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					BIT(IIO_CHAN_INFO_PEAK) |
					BIT(IIO_CHAN_INFO_PEAK_SCALE);

static const struct iio_chan_spec nvs_ch_ts[] = {
	IIO_CHAN_SOFT_TIMESTAMP(0)
};

static int nvs_chan(struct iio_dev *indio_dev)
{
	struct nvs_state *st = iio_priv(indio_dev);
	size_t buf_sz = 0;
	unsigned int n;
	unsigned int i;
	int scan_index;

	n = st->cfg->ch_n;
	if (!n)
		n++;
	if (st->cfg->ch_inf) {
		/* if channels already defined */
		indio_dev->channels = (struct iio_chan_spec *)st->cfg->ch_inf;
		indio_dev->num_channels = n;
		for (i = 0; i < n; i++)
			nvs_buf_index(indio_dev->channels[i].
				      scan_type.storagebits / 8, &buf_sz);
		st->buf = devm_kzalloc(st->dev, buf_sz, GFP_KERNEL);
		if (st->buf == NULL)
			return -ENOMEM;

		return 0;
	}

	/* create IIO channels */
	n++; /* timestamp */
	st->ch = devm_kzalloc(st->dev,  n * sizeof(struct iio_chan_spec),
			      GFP_KERNEL);
	if (st->ch == NULL)
		return -ENOMEM;

	scan_index = 0;
	for (i = 0; i < n; i++) {
		if (i == (n - 1)) {
			/* timestamp channel */
			memcpy(&st->ch[i], &nvs_ch_ts, sizeof(nvs_ch_ts));
		} else {
			if (st->cfg->snsr_id > ARRAY_SIZE(iio_sensor_type))
				continue;

			st->ch[i].type = iio_sensor_type[st->cfg->snsr_id];
			if (st->ch[i].type == NVS_IIO_SENSOR_TYPE_ERR)
				continue;

			if (st->cfg->ch_sz < 0) {
				/* signed data */
				st->ch[i].scan_type.sign = 's';
				st->ch[i].scan_type.storagebits = -8 *
								st->cfg->ch_sz;
			} else {
				/* unsigned data */
				st->ch[i].scan_type.sign = 'u';
				st->ch[i].scan_type.storagebits = 8 *
								st->cfg->ch_sz;
			}
			st->ch[i].scan_type.realbits =
					       st->ch[i].scan_type.storagebits;
			st->ch[i].info_mask = nvs_info_mask_dflt;
			if (n - 1) {
				/* multiple channels */
				st->ch[i].channel2 = i + 1;
				st->ch[i].modified = 1;
				st->ch[i].info_mask_shared_by_type =
					     nvs_info_mask_shared_by_type_dflt;
				st->ch[i].info_mask_separate =
					  ~st->ch[i].info_mask_shared_by_type &
							   st->ch[i].info_mask;
			} else {
				/* single channel */
				st->ch[i].info_mask_separate =
							   st->ch[i].info_mask;
			}
		}
		st->ch[i].scan_index = scan_index;
		scan_index++;
		nvs_buf_index(st->ch[i].scan_type.storagebits / 8, &buf_sz);
	}
	if (scan_index <= 1)
		return -ENODEV;

	st->buf = devm_kzalloc(st->dev, buf_sz, GFP_KERNEL);
	if (st->buf == NULL)
		return -ENOMEM;

	indio_dev->channels = st->ch;
	indio_dev->num_channels = n;
	return 0;
}

static const struct iio_trigger_ops nvs_trigger_ops = {
	.owner = THIS_MODULE,
};

static int nvs_suspend(void *handle)
{
	return 0;
}

static int nvs_resume(void *handle)
{
	return 0;
}

static void nvs_shutdown(void *handle)
{
}

static int nvs_remove(void *handle)
{
	struct iio_dev *indio_dev = (struct iio_dev *)handle;
	struct nvs_state *st = iio_priv(indio_dev);

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
	if (st->ch)
		devm_kfree(st->dev, st->ch);
	if (st->buf)
		devm_kfree(st->dev, st->buf);
	dev_info(st->dev, "%s\n", __func__);
	iio_device_free(indio_dev);
	return 0;
}

static int nvs_probe(void **handle, void *dev_client, struct device *dev,
	      struct nvs_fn_dev *fn_dev, struct sensor_cfg *snsr_cfg)
{
	struct iio_dev *indio_dev;
	struct nvs_state *st;
	int ret;

	dev_info(dev, "%s\n", __func__);
	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		dev_err(dev, "%s iio_device_alloc ERR\n", __func__);
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	st->client = dev_client;
	st->dev = dev;
	st->fn_dev = fn_dev;
	st->cfg = snsr_cfg;
	ret = nvs_attr(indio_dev);
	if (ret) {
		dev_err(st->dev, "%s nvs_attr ERR=%d\n", __func__, ret);
		goto nvs_probe_err;
	}

	ret = nvs_chan(indio_dev);
	if (ret) {
		dev_err(st->dev, "%s nvs_chan ERR=%d\n", __func__, ret);
		goto nvs_probe_err;
	}

	indio_dev->buffer = iio_kfifo_allocate(indio_dev);
	if (!indio_dev->buffer) {
		dev_err(st->dev, "%s iio_kfifo_allocate ERR\n", __func__);
		ret = -ENOMEM;
		goto nvs_probe_err;
	}

	indio_dev->buffer->scan_timestamp = true;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = st->dev;
	indio_dev->name = st->cfg->name;
	st->info.driver_module = THIS_MODULE;
	st->info.attrs = &st->attr_group;
	st->info.read_raw = &nvs_read_raw;
	st->info.write_raw = &nvs_write_raw;
	indio_dev->info = &st->info;
	indio_dev->setup_ops = &nvs_buffer_setup_ops;
	ret = iio_buffer_register(indio_dev, indio_dev->channels,
				  indio_dev->num_channels);
	if (ret) {
		dev_err(st->dev, "%s iio_buffer_register ERR\n", __func__);
		goto nvs_probe_err;
	}

	if (st->cfg->kbuf_sz) {
		indio_dev->buffer->access->set_length(indio_dev->buffer,
						      st->cfg->kbuf_sz);
		indio_dev->buffer->access->request_update(indio_dev->buffer);
	}
	st->trig = iio_trigger_alloc("%s-dev%d",
				     indio_dev->name, indio_dev->id);
	if (st->trig == NULL) {
		dev_err(st->dev, "%s iio_allocate_trigger ERR\n", __func__);
		ret = -ENOMEM;
		goto nvs_probe_err;
	}

	st->trig->dev.parent = st->dev;
	st->trig->ops = &nvs_trigger_ops;
	ret = iio_trigger_register(st->trig);
	if (ret) {
		dev_err(st->dev, "%s iio_trigger_register ERR\n", __func__);
		ret = -ENOMEM;
		goto nvs_probe_err;
	}

	indio_dev->trig = st->trig;
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	indio_dev->multi_link = true;
	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(st->dev, "%s iio_device_register ERR\n", __func__);
		goto nvs_probe_err;
	}

	*handle = indio_dev;
	dev_info(st->dev, "%s done\n", __func__);
	return 0;

nvs_probe_err:
	dev_err(st->dev, "%s ERR %d\n", __func__, ret);
	nvs_remove(indio_dev);
	return ret;
}

static struct nvs_fn_if nvs_fn_if_iio = {
	.probe				= nvs_probe,
	.remove				= nvs_remove,
	.shutdown			= nvs_shutdown,
	.mutex_lock			= nvs_mutex_lock,
	.mutex_unlock			= nvs_mutex_unlock,
	.suspend			= nvs_suspend,
	.resume				= nvs_resume,
	.handler			= nvs_handler,
};

struct nvs_fn_if *nvs_iio(void)
{
	return &nvs_fn_if_iio;
}

