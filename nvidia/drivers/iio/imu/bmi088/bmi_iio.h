/* Copyright (c) 2014-2022, NVIDIA CORPORATION.  All rights reserved.
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


#ifndef _BMI_IIO_H_
#define _BMI_IIO_H_

#include <linux/device.h>
#include <linux/iio/iio.h>

#define BMI_STS_SHUTDOWN	(1 << 0)
#define BMI_STS_SUSPEND		(1 << 1)

struct bmi_float {
	int ival;
	int fval;
};

struct sensor_cfg {
	const char *name;
	int snsr_id;
	int kbuf_sz;
	unsigned int ch_n;
	const char *part;
	struct bmi_float max_range;
	int delay_us_max;
	signed char matrix[9];
	unsigned int float_significance;
	struct bmi_float scale;
};

struct iio_fn_dev {
	unsigned int *sts;
	int (*enable)(void *client, int snsr_id, int enable, bool is_gte);
	int (*freq_read)(void *client, int snsr_id, int *val, int *val2);
	int (*freq_write)(void *client, int snsr_id, int val, int val2);
	int (*scale_write)(void *client, int snsr_id, int val, int val2);
	int (*regs)(void *client, int snsr_id, char *buf);
	int (*read_err)(void *client, int snsr_id, char *buf);
	int (*get_data)(void *client, int snsr_id, int axis, int *val);
};

void bmi_iio_remove(struct iio_dev *indio_dev);
int bmi_iio_push_buf(struct iio_dev *indio_dev, unsigned char *data, u64 ts);
int bmi_08x_iio_init(struct iio_dev **handle, void *dev_client,
		     struct device *dev, struct iio_fn_dev *fn_dev,
		     struct sensor_cfg *snsr_cfg);

#endif
