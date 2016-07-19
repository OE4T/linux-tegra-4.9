/*
 * Engine side synchronization support
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#ifndef NVHOST_SYNCPT_UNIT_INTERFACE_H
#define NVHOST_SYNCPT_UNIT_INTERFACE_H

struct platform_device;

bool nvhost_syncpt_gos_supported(struct platform_device *engine_pdev,
				 u32 syncpt_id);

int nvhost_syncpt_gos_address(struct platform_device *engine_pdev,
			      u32 syncpt_id,
			      u32 *gos_id,
			      u32 *gos_offset);

int nvhost_syncpt_unit_interface_get_aperture(
				struct platform_device *host_pdev,
				phys_addr_t *base,
				size_t *size);

u32 nvhost_syncpt_unit_interface_get_byte_offset(u32 syncpt_id);

dma_addr_t nvhost_syncpt_address(struct platform_device *engine_pdev, u32 id);

int nvhost_syncpt_unit_interface_init(struct platform_device *engine_pdev);

#endif
