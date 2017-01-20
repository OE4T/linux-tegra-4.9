/*
 * drivers/video/tegra/host/nvhost_ktime.h
 *
 * Tegra Graphics Host Interrupt Management
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_KTIME_H
#define __NVHOST_KTIME_H

#include <linux/version.h>

#ifdef CONFIG_TEGRA_PTP_NOTIFIER
#include <linux/platform/tegra/ptp-notifier.h>

#define nvhost_ktime_get_ts(ts)		\
do {					\
	u64 time_ns = get_ptp_hwtime();	\
	*ts = ns_to_timespec(time_ns);	\
} while (0)

#else
#include <linux/ktime.h>

#define nvhost_ktime_get_ts(ts)		\
do {					\
	ktime_get_ts(ts);		\
} while (0)

#endif

#endif
