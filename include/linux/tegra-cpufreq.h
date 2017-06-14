/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_TEGRA_CPUFREQ_H
#define _LINUX_TEGRA_CPUFREQ_H

enum tegra_cpufreq_msg_ids {
	TEGRA_CPU_FREQ_THROTTLE,
	TEGRA_CPU_FREQ_SET_RATE,
	MAX_IVC_MSG_ID,
};

int tegra_cpufreq_tx_ivc_msg(uint32_t id, uint32_t len, void *msg_buf);
int parse_hv_dt_data(struct device_node *dn);
void tegra_update_cpu_speed_hv(uint32_t rate, uint8_t cpu);

#endif
