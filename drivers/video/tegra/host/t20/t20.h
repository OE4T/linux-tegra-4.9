/*
 * drivers/video/tegra/host/t20/t20.h
 *
 * Tegra Graphics Chip support for T20
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
#ifndef _NVHOST_T20_H_
#define _NVHOST_T20_H_

struct nvhost_master;
struct nvhost_module;

int nvhost_init_t20_channel_support(struct nvhost_master *);
int nvhost_init_t20_debug_support(struct nvhost_master *);
int nvhost_init_t20_syncpt_support(struct nvhost_master *);
int nvhost_init_t20_intr_support(struct nvhost_master *);
int nvhost_init_t20_support(struct nvhost_master *host);
int nvhost_t20_save_context(struct nvhost_module *mod, u32 syncpt_id);

#endif /* _NVHOST_T20_H_ */
