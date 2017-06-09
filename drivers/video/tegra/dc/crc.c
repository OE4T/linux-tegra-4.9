/*
 * crc.c: CRC functions for tegradc EXT device
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
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
 */
#include <linux/errno.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include "dc.h"
#include "dc_priv_defs.h"
#include "dc_priv.h"
#include "dc_reg.h"
#include <video/tegra_dc_ext.h>

#define TEGRA_DC_FLIP_BUF_CAPACITY 1024 /* in units of number of elements */
#define TEGRA_DC_CRC_BUF_CAPACITY 1024 /* in units of number of elements */

/* Called when enabling the DC head.
 * Avoid calling it when disabling the DC head so as to avoid any issues caused
 * by an impending or a missing flush of values to the registers
 */
void tegra_dc_crc_reset(struct tegra_dc *dc)
{
	tegra_nvdisp_crc_reset(dc);
}

/* Called when disabling the DC head
 * This function resets only the SW state associated with the CRC mechanism
 */
void tegra_dc_crc_deinit(struct tegra_dc *dc)
{
	if (!dc->crc_initialized)
		return;

	kfree(dc->flip_buf.data);
	kfree(dc->crc_buf.data);

	dc->flip_buf.size = 0;
	dc->flip_buf.head = 0;
	dc->flip_buf.tail = 0;
	dc->crc_buf.size = 0;
	dc->crc_buf.head = 0;
	dc->crc_buf.tail = 0;

	dc->flip_id = 0;

	atomic_set(&dc->crc_ref_cnt.global, 0);
	atomic_set(&dc->crc_ref_cnt.rg_comp_sor, 0);
	atomic_set(&dc->crc_ref_cnt.regional, 0);
	dc->crc_ref_cnt.legacy = false;

	mutex_destroy(&dc->flip_buf.lock);
	mutex_destroy(&dc->crc_buf.lock);

	dc->crc_initialized = false;
}

static long tegra_dc_crc_init(struct tegra_dc *dc)
{
	dc->flip_buf.capacity = TEGRA_DC_FLIP_BUF_CAPACITY;
	dc->crc_buf.capacity = TEGRA_DC_CRC_BUF_CAPACITY;

	dc->flip_buf.type = TEGRA_DC_RING_BUF_FLIP;
	dc->crc_buf.type = TEGRA_DC_RING_BUF_CRC;

	dc->flip_buf.data = kcalloc(TEGRA_DC_FLIP_BUF_CAPACITY,
				    sizeof(struct tegra_dc_flip_buf_ele),
				    GFP_KERNEL);
	if (!dc->flip_buf.data)
		return -ENOMEM;

	dc->crc_buf.data = kcalloc(TEGRA_DC_CRC_BUF_CAPACITY,
				   sizeof(struct tegra_dc_crc_buf_ele),
				   GFP_KERNEL);
	if (!dc->crc_buf.data)
		return -ENOMEM;

	mutex_init(&dc->flip_buf.lock);
	mutex_init(&dc->crc_buf.lock);

	dc->crc_initialized = true;

	return 0;
}

long tegra_dc_crc_enable(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg)
{
	int ret;
	struct tegra_dc_ext_crc_conf *conf =
				(struct tegra_dc_ext_crc_conf *)arg->conf;
	u8 iter;

	if (!(tegra_dc_is_t18x() || tegra_dc_is_t19x()))
		return -ENOTSUPP;

	if (!dc->enabled)
		return -ENODEV;

	if (dc->crc_ref_cnt.legacy)
		return -EBUSY;

	if (!dc->crc_initialized) {
		ret = tegra_dc_crc_init(dc);
		if (ret)
			return ret;
	}

	for (iter = 0; iter < arg->num_conf; iter++) {
		ret = tegra_nvdisp_crc_enable(dc, conf + iter);
		if (ret)
			return ret;
	}

	tegra_nvdisp_update_enable_general_ack_req(dc);

	return 0;
}

long tegra_dc_crc_disable(struct tegra_dc *dc,
			  struct tegra_dc_ext_crc_arg *arg)
{
	int ret;
	struct tegra_dc_ext_crc_conf *conf =
				(struct tegra_dc_ext_crc_conf *)arg->conf;
	u8 iter;

	if (!(tegra_dc_is_t18x() || tegra_dc_is_t19x()))
		return -ENOTSUPP;

	if (!dc->enabled)
		return -ENODEV;

	if (!dc->crc_initialized)
		return -EPERM;

	if (dc->crc_ref_cnt.legacy)
		return -EBUSY;

	for (iter = 0; iter < arg->num_conf; iter++) {
		ret = tegra_nvdisp_crc_disable(dc, conf + iter);
		if (ret)
			return ret;
	}

	tegra_nvdisp_update_enable_general_ack_req(dc);

	return 0;
}

long tegra_dc_crc_get(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg)
{
	return -ENOTSUPP;
}
