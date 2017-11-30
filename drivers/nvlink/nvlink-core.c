/*
 * nvlink-core.c:
 * This driver manages the entire NVLINK system that the Tegra SOC is connected
 * to. The NVLINK core driver interfaces with the NVLINK endpoint drivers. Each
 * endpoint driver is responsible for the HW programming of 1 particular NVLINK
 * device. The core driver uses the endpoint drivers to manage the NVLINK
 * system.
 *
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/printk.h>

#include "nvlink.h"

#define NVLINK_DRV_NAME "nvlink-core"

u32 nvlink_log_mask = NVLINK_DEFAULT_LOG_MASK;

int nvlink_register_endpt_drv(struct nvlink_link *link)
{
	if (link->device_id == NVLINK_ENDPT_T19X &&
		link->remote_device_info.device_id == NVLINK_ENDPT_T19X) {
		nvlink_dbg("Loopback topology detected!");
		nvlink_dbg("Endpoint driver registered successfully!");
		return 0;
	} else {
		nvlink_err("Invalid topology!");
		nvlink_err("Endpoint driver registeration failed!");
		return -1;
	}
}

int nvlink_init_link(struct nvlink_device *ndev)
{
	int ret = 0;

	ret = ndev->links[0].link_ops.enable_link(ndev);
	if (ret < 0)
		nvlink_err("Failed to enable the link!");
	else
		nvlink_dbg("Link enabled successfully!");

	return ret;
}
