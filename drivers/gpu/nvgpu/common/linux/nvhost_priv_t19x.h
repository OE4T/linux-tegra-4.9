/*
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

#ifndef __NVGPU_NVHOST_PRIV_T19X_H__
#define __NVGPU_NVHOST_PRIV_T19X_H__

/*
 * This file is copy of common/linux/nvhost_priv.h in nvgpu repo
 * This file should be removed when nvgpu-t19x repo is merged
 * into common nvgpu repo
 */
struct nvgpu_nvhost_dev {
	struct platform_device *host1x_pdev;
};

#endif /* __NVGPU_NVHOST_PRIV_T19X_H__ */
