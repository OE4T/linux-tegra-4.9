/*
 * Tegra NVJPG Module Support
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVHOST_NVJPG_H__
#define __NVHOST_NVJPG_H__

#include <linux/types.h>
#include <linux/dma-attrs.h>
#include <linux/nvhost.h>

int nvhost_nvjpg_finalize_poweron(struct platform_device *dev);
int nvhost_nvjpg_t210_finalize_poweron(struct platform_device *dev);
int nvhost_nvjpg_prepare_poweroff(struct platform_device *dev);

/* Would have preferred a static inline here... but we're using this
 * in a place where a constant initializer is required */
#define NVHOST_ENCODE_NVJPG_VER(maj, min) \
	((((maj) & 0xff) << 8) | ((min) & 0xff))

static inline void decode_nvjpg_ver(int version, u8 *maj, u8 *min)
{
	u32 uv32 = (u32)version;
	*maj = (u8)((uv32 >> 8) & 0xff);
	*min = (u8)(uv32 & 0xff);
}

struct nvjpg {
	bool valid;
	size_t size;

	struct {
		u32 bin_data_offset;
		u32 data_offset;
		u32 data_size;
		u32 code_offset;
		u32 size;
	} os;

	struct dma_attrs attrs;
	dma_addr_t phys;
	u32 *mapped;
};

struct nvjpg_ucode_bin_header_v1 {
	u32 bin_magic;		/* 0x10de */
	u32 bin_ver;		/* cya, versioning of bin format (1) */
	u32 bin_size;		/* entire image size including this header */
	u32 os_bin_header_offset;
	u32 os_bin_data_offset;
	u32 os_bin_size;
};

struct nvjpg_ucode_os_code_header_v1 {
	u32 offset;
	u32 size;
};

struct nvjpg_ucode_os_header_v1 {
	u32 os_code_offset;
	u32 os_code_size;
	u32 os_data_offset;
	u32 os_data_size;
	u32 num_apps;
	struct nvjpg_ucode_os_code_header_v1 *app_code;
	struct nvjpg_ucode_os_code_header_v1 *app_data;
	u32 *os_ovl_offset;
	u32 *of_ovl_size;
};

struct nvjpg_ucode_v1 {
	struct nvjpg_ucode_bin_header_v1 *bin_header;
	struct nvjpg_ucode_os_header_v1  *os_header;
	bool valid;
};

#endif
