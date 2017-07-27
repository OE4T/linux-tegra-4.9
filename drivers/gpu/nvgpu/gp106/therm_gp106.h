/*
 * general thermal control structures & definitions
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_THERM_GP106_H
#define NVGPU_THERM_GP106_H

struct gk20a;

void gp106_get_internal_sensor_limits(s32 *max_24_8, s32 *min_24_8);
int gp106_get_internal_sensor_curr_temp(struct gk20a *g, u32 *temp_f24_8);
#ifdef CONFIG_DEBUG_FS
void gp106_therm_debugfs_init(struct gk20a *g);
#endif
int gp106_elcg_init_idle_filters(struct gk20a *g);
u32 gp106_configure_therm_alert(struct gk20a *g, s32 curr_warn_temp);

#endif
