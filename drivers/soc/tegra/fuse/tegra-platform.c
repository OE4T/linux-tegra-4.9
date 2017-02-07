/*
 * Copyright (c) 2014-2016, NVIDIA CORPORATION.  All rights reserved.
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
 *
 */

#include <linux/tegra-soc.h>

#define MINOR_QT		0
#define MINOR_FPGA		1
#define MINOR_ASIM_QT		2
#define MINOR_ASIM_LINSIM	3
#define MINOR_DSIM_ASIM_LINSIM	4
#define MINOR_UNIT_FPGA		5

enum tegra_platform tegra_get_platform(void)
{
	u32 chipid;

	chipid = tegra_read_chipid();
	if (!((chipid >> 4) & 0xf)) {
		u32 minor;

		minor = (chipid >> 16) & 0xff;
		switch (minor) {
		case MINOR_QT:
			return TEGRA_PLATFORM_QT;
		case MINOR_FPGA:
			return TEGRA_PLATFORM_FPGA;
		case MINOR_ASIM_QT:
			return TEGRA_PLATFORM_QT;
		case MINOR_ASIM_LINSIM:
			return TEGRA_PLATFORM_LINSIM;
		case MINOR_DSIM_ASIM_LINSIM:
			return TEGRA_PLATFORM_LINSIM;
		case MINOR_UNIT_FPGA:
			return TEGRA_PLATFORM_UNIT_FPGA;
		}
	}

	return TEGRA_PLATFORM_SILICON;
}
EXPORT_SYMBOL(tegra_get_platform);

bool tegra_cpu_is_asim(void)
{
	u32 chipid;

	chipid = tegra_read_chipid();
	if (!((chipid >> 4) & 0xf)) {
		u32 minor;

		minor = (chipid >> 16) & 0xff;
		switch (minor) {
		case MINOR_QT:
		case MINOR_FPGA:
			return false;
		case MINOR_ASIM_QT:
		case MINOR_ASIM_LINSIM:
		case MINOR_DSIM_ASIM_LINSIM:
		case MINOR_UNIT_FPGA:
			return true;
		}
	}

	return false;
}

bool tegra_cpu_is_dsim(void)
{
	u32 chipid;

	chipid = tegra_read_chipid();
	if (!((chipid >> 4) & 0xf)) {
		u32 minor;

		minor = (chipid >> 16) & 0xff;
		switch (minor) {
		case MINOR_QT:
		case MINOR_FPGA:
		case MINOR_ASIM_QT:
		case MINOR_ASIM_LINSIM:
		case MINOR_UNIT_FPGA:
			return false;
		case MINOR_DSIM_ASIM_LINSIM:
			return true;
		}
	}

	return false;
}
