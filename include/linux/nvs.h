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


#ifndef _NVS_H_
#define _NVS_H_

#include <linux/device.h>
#include <linux/regulator/consumer.h>

#define NVS_STS_SHUTDOWN		(1 << 0)
#define NVS_STS_SUSPEND			(1 << 1)
#define NVS_STS_SYS_N			(2)

#define NVS_STS_SPEW_MSG		(1 << (NVS_STS_SYS_N + 0))
#define NVS_STS_SPEW_DATA		(1 << (NVS_STS_SYS_N + 1))
#define NVS_STS_SPEW_BUF		(1 << (NVS_STS_SYS_N + 2))
#define NVS_STS_SPEW_IRQ		(1 << (NVS_STS_SYS_N + 3))
#define NVS_STS_SPEW_COMMON		(NVS_STS_SPEW_MSG | \
					NVS_STS_SPEW_DATA | \
					NVS_STS_SPEW_BUF | \
					NVS_STS_SPEW_IRQ)
#define NVS_STS_DBG_N			(NVS_STS_SYS_N + 4)
#define NVS_STS_EXT_N			(NVS_STS_DBG_N)

struct nvs_float {
	int ival;
	int fval;
};

struct sensor_cfg {
	const char *name;
	int snsr_id;
	int samplebytes;
	bool no_suspend;
	int kbuf_sz;
	unsigned int ch_n;
	int ch_sz;
	void *ch_inf;
	const char *part;
	const char *vendor;
	int version;
	struct nvs_float max_range;
	struct nvs_float resolution;
	struct nvs_float milliamp;
	int delay_us_min;
	int delay_us_max;
	unsigned int fifo_rsrv_evnt_cnt;
	unsigned int fifo_max_evnt_cnt;
	unsigned int flags;
	signed char matrix[9];		/* device orientation on platform */
	struct nvs_float scale;
	struct nvs_float offset;
	int uncal_lo;
	int uncal_hi;
	int cal_lo;
	int cal_hi;
	int thresh_lo;
	int thresh_hi;
	int report_n;
};

struct nvs_fn_dev {
/**
 * enable - enable/disable the device
 * @client: clients private data
 * @snsr_id: sensor ID
 * @enable: 0 = off
 *          1 = on
 *          -1 = query status
 *
 * Returns device enable state or a negative error code.
 *
 * Note that the enable value may be a bitmap of the channels.
 */
	int (*enable)(void *client, int snsr_id, int enable);
/**
 * batch - see Android definition of batch
 * http://source.android.com/devices/sensors/batching.html
 * @client: clients private data
 * @snsr_id: sensor ID
 * @flags: see Android definition of flags (currently obsolete)
 * @period: period timeout in microseconds
 * @timeout: batch timeout in microseconds
 *
 * Returns 0 on success or a negative error code.
 *
 * Note that period should be implemented for setting delay if
 * batching is not supported.
 */
	int (*batch)(void *client, int snsr_id, int flags,
		     unsigned int period, unsigned int timeout);
/**
 * flush - see Android definition of flush
 * http://source.android.com/devices/sensors/batching.html
 * @client: clients private data
 * @snsr_id: sensor ID
 *
 * Returns 0 on success or a negative error code.
 *
 * Note that if not implemented at the device level, it is
 * implemented in the NVS layer.  In other words, if the device
 * does not support batching, leave this NULL.
 */
	int (*flush)(void *client, int snsr_id);
/**
 * resolution - set device resolution
 * @client: clients private data
 * @snsr_id: sensor ID
 * @resolution: resolution value
 *
 * Returns 0 on success or a negative error code.
 *
 * Note that if not implemented, resolution changes will change
 * sensor_cfg->resolution.  If implemented, it is expected
 * that the resolution value will be device-specific. In other
 * words, only the device layer will understand the value which
 * will typically be used to change the mode.
 */
	int (*resolution)(void *client, int snsr_id, int resolution);
/**
 * max_range - set device max_range
 * @client: clients private data
 * @snsr_id: sensor ID
 * @max_range: max_range value
 *
 * Returns 0 on success or a negative error code.
 *
 * Note that if not implemented, max_range changes will change
 * sensor_cfg->max_range.  If implemented, it is expected
 * that the max_range value will be device-specific. In other
 * words, only the device layer will understand the value which
 * will typically be used to change the mode.
 */
	int (*max_range)(void *client, int snsr_id, int max_range);
/**
 * scale - set device scale
 * @client: clients private data
 * @snsr_id: sensor ID
 * @scale: scale value
 *
 * Returns 0 on success or a negative error code.
 *
 * Note that if not implemented, scale changes will change
 * sensor_cfg->scale.  If implemented, it is expected
 * that the scale value will be device-specific. In other words,
 * only the device layer will understand the value which will
 * typically be used to change the mode.
 */
	int (*scale)(void *client, int snsr_id, int scale);
/**
 * offset - set device offset
 * @client: clients private data
 * @snsr_id: sensor ID
 * @offset: offset value
 *
 * Returns 0 on success or a negative error code.
 *
 * Note that if not implemented, offset changes will change
 * sensor_cfg->offset.  If implemented, it is expected
 * that the offset value will be device-specific. In other
 * words, only the device layer will understand the value which
 * will typically be used to set calibration.
 */
	int (*offset)(void *client, int snsr_id, int offset);
/**
 * thresh_lo - set device low threshold
 * @client: clients private data
 * @snsr_id: sensor ID
 * @thresh_lo: low threshold value
 *
 * Returns 0 on success or a negative error code.
 *
 * Note that if not implemented, thresh_lo changes will change
 * sensor_cfg->thresh_lo.  If implemented, it is expected
 * that the thresh_lo value will be device-specific. In other
 * words, only the device layer will understand the value.
 */
	int (*thresh_lo)(void *client, int snsr_id, int thresh_lo);
/**
 * thresh_hi - set device high threshold
 * @client: clients private data
 * @snsr_id: sensor ID
 * @thresh_hi: high threshold value
 *
 * Returns 0 on success or a negative error code.
 *
 * Note that if not implemented, thresh_hi changes will change
 * sensor_cfg->thresh_hi.  If implemented, it is expected
 * that the thresh_hi value will be device-specific. In other
 * words, only the device layer will understand the value.
 */
	int (*thresh_hi)(void *client, int snsr_id, int thresh_hi);
/**
 * reset - device reset
 * @client: clients private data
 * @snsr_id: sensor ID
 *
 * Returns 0 on success or a negative error code.
 *
 * Note a < 0 value for snsr_id is another reset option,
 * e.g. global device reset such as on a sensor hub.
 */
	int (*reset)(void *client, int snsr_id);
/**
 * selftest - device self-test
 * @client: clients private data
 * @snsr_id: sensor ID
 * @buf: character buffer to write to
 *
 * Returns 0 on success or a negative error code if buf == NULL.
 * if buf != NULL, return number of characters.
 */
	int (*selftest)(void *client, int snsr_id, char *buf);
/**
 * regs - device register dump
 * @client: clients private data
 * @snsr_id: sensor ID
 * @buf: character buffer to write to
 *
 * Returns buf count or a negative error code.
 */
	int (*regs)(void *client, int snsr_id, char *buf);
/**
 * sts - status flags
 * used by both device and NVS layers
 * See NVS_STS_ defines
 */
	unsigned int *sts;
/**
 * errs - error counter
 * used by both device and NVS layers
 */
	unsigned int *errs;
};

struct nvs_fn_if {
	int (*probe)(void **handle, void *dev_client, struct device *dev,
		     struct nvs_fn_dev *fn_dev, struct sensor_cfg *snsr_cfg);
	int (*remove)(void *handle);
	void (*shutdown)(void *handle);
	void (*mutex_lock)(void *handle);
	void (*mutex_unlock)(void *handle);
	int (*suspend)(void *handle);
	int (*resume)(void *handle);
	int (*handler)(void *handle, void *buffer, s64 ts);
};

struct nvs_fn_if *nvs_iio(void);

int nvs_of_dt(const struct device_node *np, struct sensor_cfg *cfg,
	      const char *dev_name);
int nvs_vreg_dis(struct device *dev, struct regulator_bulk_data *vreg);
int nvs_vregs_disable(struct device *dev, struct regulator_bulk_data *vregs,
		      unsigned int vregs_n);
int nvs_vreg_en(struct device *dev, struct regulator_bulk_data *vreg);
int nvs_vregs_enable(struct device *dev, struct regulator_bulk_data *vregs,
		     unsigned int vregs_n);
void nvs_vregs_exit(struct device *dev, struct regulator_bulk_data *vregs,
		   unsigned int vregs_n);
int nvs_vregs_init(struct device *dev, struct regulator_bulk_data *vregs,
		   unsigned int vregs_n, char **vregs_name);
int nvs_vregs_sts(struct regulator_bulk_data *vregs, unsigned int vregs_n);

#endif /* _NVS_H_ */
