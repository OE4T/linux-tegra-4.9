/*
 * t19x-nvlink-endpt-debugfs.c:
 * This file adds various debugfs nodes for the Tegra NVLINK controller.
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

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"

/* TODO: Add debugfs nodes */
void t19x_nvlink_endpt_debugfs_init(struct nvlink_device *ndev)
{
	struct tegra_nvlink_device *tdev =
				(struct tegra_nvlink_device *)ndev->priv;

	if (!nvlink_debugfs) {
		nvlink_err("Root NVLINK debugfs directory doesn't exist");
		goto fail;
	}

	tdev->tegra_debugfs = debugfs_create_dir(NVLINK_DRV_NAME,
						nvlink_debugfs);
	if (!tdev->tegra_debugfs) {
		nvlink_err("Failed to create Tegra NVLINK endpoint driver's"
			" debugfs directory");
		goto fail;
	}

	return;

fail:
	nvlink_err("Failed to create debugfs nodes");
	debugfs_remove_recursive(tdev->tegra_debugfs);
	tdev->tegra_debugfs = NULL;
}

void t19x_nvlink_endpt_debugfs_deinit(struct nvlink_device *ndev)
{
	struct tegra_nvlink_device *tdev =
				(struct tegra_nvlink_device *)ndev->priv;

	debugfs_remove_recursive(tdev->tegra_debugfs);
	tdev->tegra_debugfs = NULL;
}
