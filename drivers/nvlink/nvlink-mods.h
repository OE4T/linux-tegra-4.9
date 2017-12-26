/*
 * nvlink-mods.h:
 * This header contains the structures and variables needed for
 * the NVLINK MODs APIs exported by the Tegra NVLINK endpoint driver.
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

/* TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS */

#define NVLINK_VERSION_10				0x00000001
#define NVLINK_VERSION_20				0x00000002
#define NVLINK_VERSION_22				0x00000004

#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_INVALID	(0x00000000)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_1_0	(0x00000001)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_0	(0x00000002)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_2	(0x00000004)

#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_INVALID	(0x00000000)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_0		(0x00000002)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_2		(0x00000004)

#define TEGRA_CTRL_NVLINK_CAPS_SUPPORTED		BIT(0)
#define TEGRA_CTRL_NVLINK_CAPS_P2P_SUPPORTED		BIT(1)
#define TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ACCESS		BIT(2)
#define TEGRA_CTRL_NVLINK_CAPS_P2P_ATOMICS		BIT(3)
#define TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ATOMICS		BIT(4)
#define TEGRA_CTRL_NVLINK_CAPS_PEX_TUNNELING		BIT(5)
#define TEGRA_CTRL_NVLINK_CAPS_SLI_BRIDGE		BIT(6)
#define TEGRA_CTRL_NVLINK_CAPS_SLI_BRIDGE_SENSABLE	BIT(7)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0		BIT(8)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L1		BIT(9)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L2		BIT(10)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L3		BIT(11)
#define TEGRA_CTRL_NVLINK_CAPS_VALID			BIT(12)

struct nvlink_caps {
	__u16 nvlink_caps;

	__u8 lowest_nvlink_version;
	__u8 highest_nvlink_version;
	__u8 lowest_nci_version;
	__u8 highest_nci_version;

	__u32 discovered_link_mask;
	__u32 enabled_link_mask;
};

/* TODO: choose a unique MAGIC number for ioctl implementation */
#define TEGRA_NVLINK_IOC_MAGIC	  'T'
#define	TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS		\
		_IOWR(TEGRA_NVLINK_IOC_MAGIC,  1, struct nvlink_caps)

