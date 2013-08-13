/*
 * drivers/video/tegra/host/nvenc/nvenc.h
 *
 * Tegra NVENC Module Support
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVHOST_NVENC_H__
#define __NVHOST_NVENC_H__

#include <linux/nvhost.h>

struct mem_handle;
struct sg_table;

int nvhost_nvenc_finalize_poweron(struct platform_device *dev);
int nvhost_nvenc_init(struct platform_device *dev);
void nvhost_nvenc_deinit(struct platform_device *dev);

/* Would have preferred a static inline here... but we're using this
 * in a place where a constant initializer is required */
#define NVHOST_ENCODE_NVENC_VER(maj, min) \
	((((maj) & 0xff) << 8) | ((min) & 0xff))

static inline void decode_nvenc_ver(int version, u8 *maj, u8 *min)
{
	u32 uv32 = (u32)version;
	*maj = (u8)((uv32 >> 8) & 0xff);
	*min = (u8)(uv32 & 0xff);
}

struct nvenc {
	bool valid;
	u32  size;
	struct mem_handle *mem_r;

	struct {
		u32 bin_data_offset;
		u32 data_offset;
		u32 data_size;
		u32 code_offset;
		u32 size;
	} os;

	struct sg_table *pa;
	u8 *mapped;
};

struct nvenc_ucode_bin_header_v1 {
	u32 bin_magic;		/* 0x10de */
	u32 bin_ver;		/* cya, versioning of bin format (1) */
	u32 bin_size;		/* entire image size including this header */
	u32 os_bin_header_offset;
	u32 os_bin_data_offset;
	u32 os_bin_size;
};

struct nvenc_ucode_os_code_header_v1 {
	u32 offset;
	u32 size;
};

struct nvenc_ucode_os_header_v1 {
	u32 os_code_offset;
	u32 os_code_size;
	u32 os_data_offset;
	u32 os_data_size;
	u32 num_apps;
	struct nvenc_ucode_os_code_header_v1 *app_code;
	struct nvenc_ucode_os_code_header_v1 *app_data;
	u32 *os_ovl_offset;
	u32 *of_ovl_size;
};

struct nvenc_ucode_v1 {
	struct nvenc_ucode_bin_header_v1 *bin_header;
	struct nvenc_ucode_os_header_v1  *os_header;
	struct mem_handle *mem;
	struct sg_table *pa;
	bool valid;
};

#endif
