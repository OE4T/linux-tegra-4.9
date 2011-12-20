/*
 * drivers/video/tegra/host/mpe/mpe.h
 *
 * Tegra Graphics Host MPE
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

#ifndef __NVHOST_MPE_MPE_H
#define __NVHOST_MPE_MPE_H

struct nvhost_hwctx_handler;

int nvhost_mpe_ctxhandler_init(struct nvhost_hwctx_handler *h);
int nvhost_mpe_prepare_power_off(struct nvhost_module *mod);

#endif
