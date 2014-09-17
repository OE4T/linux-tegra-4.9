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

#ifndef __TEGRA_ISC_MGR_H__
#define __TEGRA_ISC_MGR_H__

struct tegra_isc_mgr_sinfo {
	int pid;
	int sig_no;
	void *context;
};

#define TEGRA_ISC_MGR_POWER_ALL	5

#define TEGRA_ISC_MGR_IOCTL_PWR_DN	_IOW('o', 1, __s16)
#define TEGRA_ISC_MGR_IOCTL_PWR_UP	_IOR('o', 2, __s16)
#define TEGRA_ISC_MGR_IOCTL_SET_PID	_IOW('o', 3,\
					struct tegra_isc_mgr_sinfo)
#define TEGRA_ISC_MGR_IOCTL_SUSPEND_SIGNAL	_IOW('o', 4, __s16)
#define TEGRA_ISC_MGR_IOCTL_RESUME_SIGNAL	_IOW('o', 5, __s16)

#endif  /* __TEGRA_ISC_MGR_H__ */
