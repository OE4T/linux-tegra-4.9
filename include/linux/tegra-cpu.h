/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_TEGRA_CPU_H
#define _LINUX_TEGRA_CPU_H

#include <asm/cputype.h>

#define ARM_CPU_IMP_NVIDIA	0x4E /* 'N' */

static int tegra_cpu_is_denver(void)
{
	return read_cpuid_implementor() == ARM_CPU_IMP_NVIDIA;
}

static int tegra_cpu_is_arm(void)
{
	return read_cpuid_implementor() == ARM_CPU_IMP_ARM;
}

#endif
