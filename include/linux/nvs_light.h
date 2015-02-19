/* Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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


#ifndef _NVS_LIGHT_H_
#define _NVS_LIGHT_H_

#include <linux/iio/iio.h>
#include <linux/of.h>
#include <linux/nvs.h>

#define RET_POLL_NEXT			(-1)
#define RET_NO_CHANGE			(0)
#define RET_HW_UPDATE			(1)

#define NVS_LIGHT_STRING		"light"
#define SENSOR_FLAG_ON_CHANGE_MODE	0x2 /* from AOS sensors.h */

/**
 * struct nvs_light_dynamic - the structure that allows the NVS
 * light module to control dynamic resolution/range.
 * @resolution_ival: resolution.
 * @max_range_ival: max_range.
 * @milliamp: milliamp.
 * @delay_min_ms: minimum delay time in ms.
 * @driver_data: private data for driver.
 *
 * Driver includes a pointer to this table to enable dynamic
 * resolution/max_range.
 * The data must be from low to high max_range.
 */
struct nvs_light_dynamic {
	struct nvs_float resolution;
	struct nvs_float max_range;
	struct nvs_float milliamp;
	unsigned int delay_min_ms;	/* minimum ms delay time */
	unsigned int driver_data;	/* private data for driver */
};

/**
 * struct nvs_light - the common structure between the ALS
 * driver and the NVS common module for light.
 * @timestamp: Driver writes the timestamp HW was read.
 * @timestamp_report: NVS writes the timestamp used to report.
 * @lux: NVS writes the lux value that is reported.
 * @hw: Driver writes the value read from HW.
 * @hw_mask: Driver writes the HW value mask (maximum value).
 * @hw_thresh_lo: NVS writes the low threshold value needed.
 * @hw_thresh_hi: NVS writes the high threshold value needed.
 * @hw_limit_lo: NVS determines if HW at low limit.  Driver can
 *               use this for dynamic resolution if disabled in
 *               NVS.  See nld_tbl.
 * @hw_limit_hi: NVS determines if HW at high limit.  Driver can
 *               use this for dynamic resolution if disabled in
 *               NVS.  See nld_tbl.
 * @thresh_valid_lo: NVS determines if cfg.thresh_lo valid.
 * @thresh_valid_hi: NVS determines if cfg.thresh_hi valid.
 * @thresholds_valid: NVS determines if both thresholds valid.
 * @nld_i_change: NVS determined dynamic resolution changed.
 *                Driver needs to update HW.
 * @calibration_en: NVS determines if calibration is enabled.
 * @poll_delay_ms: NVS writes the poll time needed if polling.
 * @delay_us: Driver writes the requested sample time.
 * @report: NVS writes the report count.
 * @nld_i: NVS writes index to nvs_light_dynamic table.
 * @nld_i_lo: Driver writes low index limit to nvs_light_dynamic
 *            table.
 *            Note: nvs_light_of_dt can be used.
 * @nld_i_hi: Driver writes high index limit to
 *            nvs_light_dynamic table.
 *            Note: nvs_light_of_dt can be used.
 * @nld_tbl: Driver writes pointer to nvs_light_dynamic table.
 *           If this is NULL then dynamic resolution is disabled
 *           in NVS.
 * @cfg: Driver writes the sensor_cfg structure pointer.
 * @nvs_data: Driver writes the private pointer for handler.
 * @handler: Driver writes the handler pointer.
 */
struct nvs_light {
	s64 timestamp;			/* sample timestamp */
	s64 timestamp_report;		/* last reported timestamp */
	u32 lux;			/* calculated lux */
	u32 hw;				/* HW light value */
	u32 hw_mask;			/* HW light mask */
	u32 hw_thresh_lo;		/* HW low threshold value */
	u32 hw_thresh_hi;		/* HW high threshold value */
	bool hw_limit_lo;		/* hw < hw_thresh_lo or hw = 0 */
	bool hw_limit_hi;		/* hw > hw_thresh_hi or hw = hw_mask */
	bool thresh_valid_lo;		/* valid cfg.thresh_lo */
	bool thresh_valid_hi;		/* valid cfg.thresh_hi */
	bool thresholds_valid;		/* both thresholds valid */
	bool nld_i_change;		/* flag that dynamic index changed */
	bool calibration_en;		/* if calibration enabled */
	unsigned int poll_delay_ms;	/* HW polling delay (ms) */
	unsigned int delay_us;		/* OS requested sample delay */
	unsigned int report;		/* report count */
	unsigned int nld_i;		/* index to light dynamic table */
	unsigned int nld_i_lo;		/* low index limit to dynamic table */
	unsigned int nld_i_hi;		/* high index limit to dynamic table */
	struct nvs_light_dynamic *nld_tbl; /* ptr to nvs_light_dynamic table */
	struct sensor_cfg *cfg;		/* pointer to sensor configuration */
	void *nvs_data;			/* NVS data for NVS handler */
	int (*handler)(void *handle, void *buffer, s64 ts);
};

static const struct iio_chan_spec iio_chan_spec_nvs_light[] = {
	{
		.type			= IIO_LIGHT,
		.scan_type		= { .sign = 'u',
					    .realbits = 32,
					    .storagebits = 32,
					    .endianness = IIO_CPU,
					  },
		.info_mask_shared_by_all
					= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					  BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					  BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					  BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET) |
					  BIT(IIO_CHAN_INFO_THRESHOLD_LOW) |
					  BIT(IIO_CHAN_INFO_THRESHOLD_HIGH),
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_BATCH_FLUSH) |
					  BIT(IIO_CHAN_INFO_BATCH_PERIOD) |
					  BIT(IIO_CHAN_INFO_BATCH_TIMEOUT) |
					  BIT(IIO_CHAN_INFO_BATCH_FLAGS) |
					  BIT(IIO_CHAN_INFO_PEAK) |
					  BIT(IIO_CHAN_INFO_PEAK_SCALE) |
					  BIT(IIO_CHAN_INFO_SCALE) |
					  BIT(IIO_CHAN_INFO_OFFSET) |
					  BIT(IIO_CHAN_INFO_THRESHOLD_LOW) |
					  BIT(IIO_CHAN_INFO_THRESHOLD_HIGH),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

int nvs_light_read(struct nvs_light *nl);
int nvs_light_enable(struct nvs_light *nl);
int nvs_light_of_dt(struct nvs_light *nl, const struct device_node *np,
		    const char *dev_name);
ssize_t nvs_light_dbg(struct nvs_light *nl, char *buf);

#endif /* _NVS_LIGHT_H_ */
