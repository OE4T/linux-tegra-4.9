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


#include <linux/nvs.h>
#include <linux/of.h>


int nvs_of_dt(const struct device_node *np, struct sensor_cfg *cfg,
	      const char *dev_name)
{
	char str[256];
	char const *charp;
	int lenp;
	int ret;

	if (np == NULL || cfg == NULL)
		return -EINVAL;

	if (dev_name == NULL)
		dev_name = cfg->name;
	ret = sprintf(str, "%s_buffer_size", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->kbuf_sz);
	ret = sprintf(str, "%s_max_range_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->max_range.ival);
	ret = sprintf(str, "%s_max_range_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->max_range.fval);
	ret = sprintf(str, "%s_resolution_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->resolution.ival);
	ret = sprintf(str, "%s_resolution_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->resolution.fval);
	ret = sprintf(str, "%s_milliamp_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->milliamp.ival);
	ret = sprintf(str, "%s_milliamp_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->milliamp.fval);
	ret = sprintf(str, "%s_delay_us_min", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->delay_us_min);
	ret = sprintf(str, "%s_delay_us_max", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->delay_us_max);
	ret = sprintf(str, "%s_fifo_max_event_count", dev_name);
	if (ret > 0)
		of_property_read_u32(np, str, (u32 *)cfg->fifo_rsrv_evnt_cnt);
	ret = sprintf(str, "%s_fifo_reserved_event_count", dev_name);
	if (ret > 0)
		of_property_read_u32(np, str, (u32 *)cfg->fifo_max_evnt_cnt);
	ret = sprintf(str, "%s_flags", dev_name);
	if (ret > 0)
		of_property_read_u32(np, str, (u32 *)cfg->flags);
	ret = sprintf(str, "%s_matrix", dev_name);
	if (ret > 0) {
		charp = of_get_property(np, str, &lenp);
		if (charp && lenp == sizeof(cfg->matrix))
			memcpy(cfg->matrix, charp, lenp);
	}
	ret = sprintf(str, "%s_scale_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->scale.ival);
	ret = sprintf(str, "%s_scale_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->scale.fval);
	ret = sprintf(str, "%s_offset_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->offset.ival);
	ret = sprintf(str, "%s_offset_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->offset.fval);
	ret = sprintf(str, "%s_uncalibrated_lo", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->uncal_lo);
	ret = sprintf(str, "%s_uncalibrated_hi", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->uncal_hi);
	ret = sprintf(str, "%s_calibrated_lo", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->cal_lo);
	ret = sprintf(str, "%s_calibrated_hi", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->cal_hi);
	ret = sprintf(str, "%s_threshold_lo", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->thresh_lo);
	ret = sprintf(str, "%s_threshold_hi", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->thresh_hi);
	ret = sprintf(str, "%s_report_count", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)cfg->report_n);
	return 0;
}

