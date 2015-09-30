/*
 * drivers/video/tegra/host/vi/vi4.h
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_VI_NOTIFY_H__
#define __NVHOST_VI_NOTIFY_H__

extern const struct file_operations tegra_vi_notify_ctrl_ops;

int nvhost_vi_notify_prepare_poweroff(struct platform_device *pdev);
int nvhost_vi_notify_finalize_poweron(struct platform_device *pdev);

int nvhost_vi_notify_dev_probe(struct platform_device *);
int nvhost_vi_notify_dev_remove(struct platform_device *);

struct nvhost_vi_dev {
#ifdef CONFIG_TEGRA_VI_NOTIFY
	struct vi_notify_dev *notify;
#endif
};

#endif
