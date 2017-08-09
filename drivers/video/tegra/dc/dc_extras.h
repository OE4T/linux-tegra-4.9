/*
 * dc_extras.h: Definitions that were moved from kernel repo to display.
 *
 * Copyright (c) 2010-2017, NVIDIA CORPORATION, All rights reserved.
 *
 * Author:
 *      Naveen Kumar S <nkumars@nvidia.com>
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_DC_EXTRAS_H
#define __MACH_TEGRA_DC_EXTRAS_H

#define TEGRA_DISPLAY_BASE		0x54200000
#define TEGRA_DISPLAY_SIZE		SZ_256K

#define TEGRA_DISPLAY2_BASE		0x54240000
#define TEGRA_DISPLAY2_SIZE		SZ_256K

#define TEGRA_HDMI_BASE			0x54280000
#define TEGRA_HDMI_SIZE			SZ_256K

#if defined(CONFIG_TEGRA_NVDISPLAY)
#define TEGRA_DSI_BASE			0x15300000
#else
#define TEGRA_DSI_BASE			0x54300000
#endif
#define TEGRA_DSI_SIZE			SZ_256K

#define TEGRA_VIC_BASE			0x54340000
#define TEGRA_VIC_SIZE			SZ_256K

#if defined(CONFIG_TEGRA_NVDISPLAY)
#define TEGRA_DSIB_BASE			0x15400000
#else
#define TEGRA_DSIB_BASE			0x54400000
#endif
#define TEGRA_DSIB_SIZE			SZ_256K

#if defined(CONFIG_TEGRA_NVDISPLAY)
#define TEGRA_DSIC_BASE			0x15900000
#define TEGRA_DSIC_SIZE			SZ_256K

#define TEGRA_DSID_BASE			0x15940000
#define TEGRA_DSID_SIZE			SZ_256K

#define TEGRA_DSI_PADCTL_BASE		0x15880000
#define TEGRA_DSI_PADCTL_SIZE		SZ_64K
#endif

#define TEGRA_SOR_BASE		0x54540000
#define TEGRA_SOR_SIZE		SZ_256K

#define TEGRA_SOR1_BASE		0x54580000
#define TEGRA_SOR1_SIZE		SZ_256K

#define TEGRA_DPAUX_BASE	0x545c0000
#define TEGRA_DPAUX_SIZE	SZ_256K

#define TEGRA_DPAUX1_BASE	0x54040000
#define TEGRA_DPAUX1_SIZE	SZ_256K

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
#define TEGRA_MIPI_CAL_BASE		0x700E3000
#define TEGRA_MIPI_CAL_SIZE		SZ_256
#endif
#endif
