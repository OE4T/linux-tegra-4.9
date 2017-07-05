/*
 * include/linux/media/isp_channel.h
 *
 * ISP channel driver header
 *
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ISP_CHANNEL_H__
#define __ISP_CHANNEL_H__

#include <linux/of_platform.h>

struct isp_channel_drv;

int isp_channel_drv_register(struct platform_device *);
void isp_channel_drv_unregister(struct device *);

struct tegra_isp_channel {
	struct device *isp_dev;
	struct platform_device *ndev;
	struct isp_channel_drv *drv;
	struct rcu_head rcu;
	struct isp_capture *capture_data;
};

#endif //__ISP_CHANNEL_H__
