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

#ifndef __NVGPU_GMMU_T19X_H__
#define __NVGPU_GMMU_T19X_H__

struct nvgpu_gmmu_attrs;

struct nvgpu_gmmu_attrs_t19x {
	bool			 l3_alloc;
};

void nvgpu_gmmu_add_t19x_attrs(struct nvgpu_gmmu_attrs *attrs, u32 flags);

#endif
