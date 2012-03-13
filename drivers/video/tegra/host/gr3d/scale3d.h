/*
 * drivers/video/tegra/host/t30/scale3d.h
 *
 * Tegra Graphics Host 3D Clock Scaling
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#ifndef NVHOST_T30_SCALE3D_H
#define NVHOST_T30_SCALE3D_H

struct nvhost_device;
struct device;
struct dentry;

/* Initialization and de-initialization for module */
void nvhost_scale3d_init(struct nvhost_device *);
void nvhost_scale3d_deinit(struct nvhost_device *);

/* Suspend is called when powering down module */
void nvhost_scale3d_suspend(struct nvhost_device *);

/* reset 3d module load counters, called on resume */
void nvhost_scale3d_reset(void);

/*
 * call when performing submit to notify scaling mechanism that 3d module is
 * in use
 */
void nvhost_scale3d_notify_busy(struct nvhost_device *);
void nvhost_scale3d_notify_idle(struct nvhost_device *);

void nvhost_scale3d_debug_init(struct dentry *de);

#endif
