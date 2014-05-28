/*
 * drivers/video/tegra/host/vi/vi.h
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2012-2014, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __NVHOST_VI_H__
#define __NVHOST_VI_H__

#include "camera_priv_defs.h"

#define CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK_0		0x850
#define CSI_CSI_PIXEL_PARSER_A_STATUS_0			0x854
#define PPA_FIFO_OVRF					(1 << 5)

#define CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK_0		0x884
#define CSI_CSI_PIXEL_PARSER_B_STATUS_0			0x888
#define PPB_FIFO_OVRF					(1 << 5)

struct tegra_vi_stats {
	atomic_t overflow;
};

struct vi {
	struct tegra_camera *camera;
	struct platform_device *ndev;
	struct regulator *reg;
	int vi_irq;
	uint vi_bw;
	struct dentry *debugdir;
	struct tegra_vi_stats vi_out;
	struct work_struct stats_work;
#if defined(CONFIG_TEGRA_ISOMGR)
	tegra_isomgr_handle isomgr_handle;
#endif
};

extern const struct file_operations tegra_vi_ctrl_ops;
int nvhost_vi_prepare_poweroff(struct platform_device *);
int nvhost_vi_finalize_poweron(struct platform_device *);
void nvhost_vi_reset(struct platform_device *);

#endif
