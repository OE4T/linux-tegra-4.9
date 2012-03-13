/*
 * drivers/video/tegra/host/t30/t30.h
 *
 * Tegra Graphics Chip support for Tegra3
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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
#ifndef _NVHOST_T30_H_
#define _NVHOST_T30_H_

struct nvhost_master;

int nvhost_init_t30_channel_support(struct nvhost_master *);
int nvhost_init_t30_debug_support(struct nvhost_master *);
int nvhost_init_t30_support(struct nvhost_master *host);

#endif /* _NVHOST_T30_H_ */
