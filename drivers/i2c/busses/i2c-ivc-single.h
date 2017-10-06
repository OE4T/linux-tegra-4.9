/*
 * Copyright (c) 2017 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _LINUX_I2C_IVC_SINGLE_H
#define _LINUX_I2C_IVC_SINGLE_H

struct tegra_i2c_ivc_single_dev;

struct tegra_i2c_ivc_single_dev *tegra_i2c_ivc_get_dev(u32 reg_base);
int tegra_i2c_ivc_single_xfer(struct tegra_i2c_ivc_single_dev *ivc_dev,
	const struct i2c_msg *reqs, int num);

#endif /* _LINUX_I2C_IVC_SINGLE_H */
