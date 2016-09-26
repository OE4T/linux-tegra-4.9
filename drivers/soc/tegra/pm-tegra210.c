/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/init.h>

#include <soc/tegra/fuse.h>
#include <soc/tegra/tegra_bpmp.h>

/* Change CPU tolerance level according to hotplug state */
static int tegra_bpmp_tolerate_idle(int cpu, int ccxtl, int scxtl)
{
	s32 data[3];

	data[0] = cpu_to_le32(cpu);
	data[1] = cpu_to_le32(ccxtl);
	data[2] = cpu_to_le32(scxtl);

	return tegra_bpmp_send(MRQ_TOLERATE_IDLE, data, sizeof(data));
}

static int tegra210_cpu_notify(struct notifier_block *nb, unsigned long action,
			       void *hcpu)
{
	int cpu = (long)hcpu;

	switch (action) {
	case CPU_POST_DEAD:
	case CPU_DEAD_FROZEN:
		tegra_bpmp_tolerate_idle(cpu, TEGRA_PM_CC7, TEGRA_PM_SC7);
		break;
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		tegra_bpmp_tolerate_idle(cpu, TEGRA_PM_CC1, TEGRA_PM_SC1);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block tegra210_cpu_nb = {
	.notifier_call = tegra210_cpu_notify
};

static int __init tegra210_pm_init(void)
{
	if (tegra_get_chip_id() != TEGRA210)
		goto out;

	register_cpu_notifier(&tegra210_cpu_nb);

out:
	return 0;
}
device_initcall(tegra210_pm_init);
