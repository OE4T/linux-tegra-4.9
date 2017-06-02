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

#ifndef __NVGPU_NVHOST_T19X_H__
#define __NVGPU_NVHOST_T19X_H__

#ifdef CONFIG_TEGRA_GK20A_NVHOST
struct nvgpu_nvhost_dev;

int nvgpu_nvhost_syncpt_unit_interface_get_aperture(
		struct nvgpu_nvhost_dev *nvhost_dev,
		phys_addr_t *base,
		size_t *size);
u32 nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(u32 syncpt_id);

#endif
#endif /* __NVGPU_NVHOST_T19X_H__ */
