/*
 * Copyright (c) 2012-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __SOC_TEGRA_FUSE_H__
#define __SOC_TEGRA_FUSE_H__

#include <linux/tegra-soc.h>

#define TEGRA_FUSE_PRODUCTION_MODE 0x0
#define FUSE_FUSEBYPASS_0	0x24
#define FUSE_WRITE_ACCESS_SW_0	0x30
#define TEGRA_FUSE_SKU_CALIB_0	0xf0
#define TEGRA30_FUSE_SATA_CALIB	0x124
#define TEGRA_FUSE_USB_CALIB_EXT_0 0x250

#define FUSE_OPT_SUBREVISION		0x148
#define FUSE_OPT_SUBREVISION_MASK	0xF

#define FUSE_GCPLEX_CONFIG_FUSE_0	0x1c8

#define FUSE_RESERVED_CALIB0_0		0x204

#define FUSE_OPT_GPU_TPC0_DISABLE_0	0x20c
#define FUSE_OPT_GPU_TPC1_DISABLE_0	0x23c

#ifndef __ASSEMBLY__

u32 tegra_read_chipid(void);
u32 tegra_read_straps(void);
u32 tegra_read_ram_code(void);
u32 tegra_read_chipid(void);
enum tegra_chipid tegra_get_chipid(void);

int tegra_fuse_control_read(unsigned long offset, u32 *value);
void tegra_fuse_control_write(u32 value, unsigned long offset);

int tegra_fuse_readl(unsigned long offset, u32 *value);
void tegra_fuse_writel(u32 val, unsigned long offset);
enum tegra_revision tegra_chip_get_revision(void);

/* TODO: Dummy implementation till upstream fuse driver implements these*/
static inline bool tegra_spare_fuse(int bit)
{ return 0; }
static inline int tegra_get_sku_override(void)
{ return 0; }
static inline u32 tegra_get_sku_id(void)
{ return 0; }

#endif /* __ASSEMBLY__ */
u32 tegra_fuse_get_subrevision(void);

#endif /* __SOC_TEGRA_FUSE_H__ */
