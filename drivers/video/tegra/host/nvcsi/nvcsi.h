/*
 * drivers/video/tegra/host/nvcsi/nvcsi.h
 *
 * Tegra Graphics Host NVCSI
 *
 * Copyright (c) 2015-2016 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_NVCSI_H__
#define __NVHOST_NVCSI_H__

#include "camera/csi/csi.h"

extern const struct file_operations tegra_nvcsi_ctrl_ops;

struct nvcsi {
	struct platform_device *pdev;
	struct regulator *regulator;
	struct tegra_csi_device csi;
};

int nvcsi_finalize_poweron(struct platform_device *pdev);
int nvcsi_prepare_poweroff(struct platform_device *pdev);

struct tegra_csi_device *tegra_get_mc_csi(void);
#endif
