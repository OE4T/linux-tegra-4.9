/*
 * drivers/video/tegra/host/t30/3dctx_t30.h
 *
 * Tegra Graphics Host Context Switching for Tegra3
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __NVHOST_3D_T30_H
#define __NVHOST_3D_T30_H

/* Registers of 3D unit */

#define AR3D_PSEQ_QUAD_ID 0x545
#define AR3D_DW_MEMORY_OUTPUT_ADDRESS 0x904
#define AR3D_DW_MEMORY_OUTPUT_DATA 0x905
#define AR3D_GSHIM_WRITE_MASK 0xb00
#define AR3D_GSHIM_READ_SELECT 0xb01
#define AR3D_GLOBAL_MEMORY_OUTPUT_READS 0xe40

#endif
