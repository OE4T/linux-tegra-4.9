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
 */
#ifndef __NVGPU_FUSE_H__
#define __NVGPU_FUSE_H__

struct gk20a;

int nvgpu_tegra_get_gpu_speedo_id(struct gk20a *g);

void nvgpu_tegra_fuse_write_bypass(struct gk20a *g, u32 val);
void nvgpu_tegra_fuse_write_access_sw(struct gk20a *g, u32 val);
void nvgpu_tegra_fuse_write_opt_gpu_tpc0_disable(struct gk20a *g, u32 val);
void nvgpu_tegra_fuse_write_opt_gpu_tpc1_disable(struct gk20a *g, u32 val);
int nvgpu_tegra_fuse_read_gcplex_config_fuse(struct gk20a *g, u32 *val);
int nvgpu_tegra_fuse_read_reserved_calib(struct gk20a *g, u32 *val);

#endif
