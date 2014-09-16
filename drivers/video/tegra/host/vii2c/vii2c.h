/*
 * Tegra Graphics Host VII2C
 *
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __NVHOST_VII2C_H__
#define __NVHOST_VII2C_H__

int nvhost_vii2c_finalize_poweron(struct platform_device *pdev);
int nvhost_vii2c_prepare_poweroff(struct platform_device *pdev);
void nvhost_vii2c_module_reset(struct platform_device *pdev);

#endif
