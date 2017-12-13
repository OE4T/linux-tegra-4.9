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

struct topology {
	int slave_dev_id;
	int master_dev_id;
	int slave_link_id;
	int master_link_id;
};

struct nvlink_core {
	struct nvlink_device *ndevs[NVLINK_MAX_DEVICES];
	struct nvlink_link *nlinks[NVLINK_MAX_LINKS];
	struct topology topology;
	struct mutex mutex;
};

u32 nvlink_log_mask = NVLINK_DEFAULT_LOG_MASK;
static struct nvlink_core nvlink_core;
static int first_dev = 1;

int nvlink_register_device(struct nvlink_device *ndev)
{
	int ret = 0;
	struct topology *topology = &(nvlink_core.topology);

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}

	if (first_dev)
		mutex_init(&nvlink_core.mutex);

	mutex_lock(&nvlink_core.mutex);

	if (ndev->device_id >= NVLINK_MAX_DEVICES) {
		nvlink_err("Invalid device_id");
		ret = -ENODEV;
		goto fail;
	}

	nvlink_core.ndevs[ndev->device_id] = ndev;

	if (first_dev) {
		if (ndev->is_master) {
			nvlink_dbg("Device %d is the master", ndev->device_id);
			topology->master_dev_id = ndev->device_id;
			topology->master_link_id = ndev->link.link_id;
			topology->slave_dev_id =
					ndev->link.remote_dev_info.device_id;
			topology->slave_link_id =
					ndev->link.remote_dev_info.link_id;
		} else {
			nvlink_dbg("Device %d is the slave", ndev->device_id);
			topology->master_dev_id =
					ndev->link.remote_dev_info.device_id;
			topology->master_link_id =
					ndev->link.remote_dev_info.link_id;
			topology->slave_dev_id = ndev->device_id;
			topology->slave_link_id = ndev->link.link_id;
		}

		if (topology->master_dev_id == topology->slave_dev_id) {
			nvlink_dbg("Tegra loopback topology detected");
		} else if (topology->master_dev_id == NVLINK_ENDPT_GV100) {
			nvlink_dbg("GV100 (master) connected to Tegra ");
		} else {
			nvlink_err("Invalid topology info in device tree");
			ret = -1;
			goto fail;
		}
	}

	goto success;

fail:
	nvlink_err("Device register failed!");
success:
	first_dev = 0;
	mutex_unlock(&nvlink_core.mutex);
	return ret;
}
EXPORT_SYMBOL(nvlink_register_device);

int nvlink_register_link(struct nvlink_link *link)
{
	int ret = 0;

	if (!link) {
		nvlink_err("Invalid link struct pointer");
		return -EINVAL;
	}

	mutex_lock(&nvlink_core.mutex);
	if (link->link_id >= NVLINK_MAX_LINKS) {
		nvlink_err("Invalid link_id");
		ret = -ENODEV;
		goto fail;
	}

	nvlink_core.nlinks[link->link_id] = link;

	goto success;

fail:
	nvlink_err("Link register failed!");
success:
	mutex_unlock(&nvlink_core.mutex);
	return ret;
}
EXPORT_SYMBOL(nvlink_register_link);

int nvlink_unregister_device(struct nvlink_device* ndev)
{
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}

	mutex_lock(&nvlink_core.mutex);

	if (ndev->device_id >= NVLINK_MAX_DEVICES) {
		nvlink_err("Invalid device_id");
		ret = -ENODEV;
		goto fail;
	}

	nvlink_core.ndevs[ndev->device_id] = NULL;

	nvlink_core.topology.master_link_id = -1;
	nvlink_core.topology.master_dev_id = -1;
	nvlink_core.topology.slave_link_id = -1;
	nvlink_core.topology.slave_dev_id = -1;

	goto success;

fail:
	nvlink_err("Device unregister failed!");
success:
	mutex_unlock(&nvlink_core.mutex);
	return ret;
}
EXPORT_SYMBOL(nvlink_unregister_device);

int nvlink_unregister_link(struct nvlink_link *link)
{
	int ret = 0;

	if (!link) {
		nvlink_err("Invalid link struct pointer");
		return -EINVAL;
	}

	mutex_lock(&nvlink_core.mutex);
	if (link->link_id >= NVLINK_MAX_LINKS) {
		nvlink_err("Invalid link_id");
		ret = -ENODEV;
		goto fail;
	}

	nvlink_core.nlinks[link->link_id] = NULL;

	goto success;

fail:
	nvlink_err("Link unregister failed!");
success:
	mutex_unlock(&nvlink_core.mutex);
	return ret;
}
EXPORT_SYMBOL(nvlink_unregister_link);

int nvlink_init_link(struct nvlink_device *ndev)
{
	int ret = 0;

	ret = ndev->link.link_ops.enable_link(ndev);
	if (ret < 0)
		nvlink_err("Failed to enable the link!");
	else
		nvlink_dbg("Link enabled successfully!");

	return ret;
}
EXPORT_SYMBOL(nvlink_init_link);
