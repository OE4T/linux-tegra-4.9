/*
 * drivers/video/tegra/host/host1x/host1x_actmon.h
 *
 * Tegra Graphics Actmon
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
#ifndef _NVHOST_HOST1X_ACTMON_H_
#define _NVHOST_HOST1X_ACTMON_H_

struct nvhost_master;

int host1x_actmon_init(struct nvhost_master *host);
void host1x_actmon_deinit(struct nvhost_master *host);
int host1x_actmon_avg(struct nvhost_master *host, u32 *val);
void host1x_actmon_intr_above_wmark(void);
void host1x_actmon_intr_below_wmark(void);
int host1x_actmon_above_wmark_count(void);
int host1x_actmon_below_wmark_count(void);

#endif
