/*
 * drivers/video/tegra/host/nvhost_memmgr.h
 *
 * Tegra Graphics Host Memory Management Abstraction header
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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

#ifndef _NVHOST_MEM_MGR_H_
#define _NVHOST_MEM_MGR_H_

struct nvhost_chip_support;

enum mem_mgr_flag {
	mem_mgr_flag_uncacheable = 0,
	mem_mgr_flag_write_combine = 1,
};

struct mem_mgr_handle {
	struct mem_mgr *client;
	struct mem_handle *handle;
};

int nvhost_memmgr_init(struct nvhost_chip_support *chip);

#endif
