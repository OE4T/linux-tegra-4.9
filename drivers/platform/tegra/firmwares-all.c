/*
 * Copyright (c) 2016-2022, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/tegra-firmwares.h>
#include <linux/cpu.h>
#include <asm/cpu.h>

static enum cpuhp_state tegra_firmwares_cpu_state;
static DEFINE_PER_CPU(struct device *, fwdev);

static int tegra_cpu_online(unsigned int cpu)
{
	char s_cpu[10];
	char s_aidr[64];
	struct cpuinfo_arm64 *cpuinfo = &per_cpu(cpu_data, cpu);

	if (MIDR_IMPLEMENTOR(cpuinfo->reg_midr) != ARM_CPU_IMP_NVIDIA)
		return 0;
	snprintf(s_cpu, sizeof(s_cpu), "CPU%d", cpu);
	snprintf(s_aidr, sizeof(s_aidr), "%u (0%x)",
		cpuinfo->reg_aidr,
		cpuinfo->reg_aidr);
	per_cpu(fwdev, cpu) = tegrafw_register_string(s_cpu, s_aidr);
	return 0;
}

static int tegra_cpu_prepare_down(unsigned int cpu)
{
	if (per_cpu(fwdev, cpu))
		tegrafw_unregister(per_cpu(fwdev, cpu));
	per_cpu(fwdev, cpu) = NULL;
	return 0;
}

static enum cpuhp_state tegra_cpu_fw_register(void)
{
	return cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
		"cpu/tegra-firmwares:online",
		tegra_cpu_online,
		tegra_cpu_prepare_down);
}

static void tegra_cpu_fw_unregister(enum cpuhp_state state)
{
	cpuhp_remove_state(state);
}

static struct device *firmwares[10];

static int __init tegra_firmwares_init(void)
{
	struct device **dev = firmwares;
	char *versions[] = { "mb1", "mb2", "mb1-bct", "qb", "osl" };
	int v;

	for (v = 0; v < ARRAY_SIZE(versions); v++) {
		if (dev - firmwares >= ARRAY_SIZE(firmwares)) {
			pr_err("Increase firmwares array size");
			return 0;
		}
		*dev++ = tegrafw_register_dt_string(versions[v],
			"/tegra-firmwares", versions[v]);
	}

	tegra_firmwares_cpu_state = tegra_cpu_fw_register();
	return 0;
}

static void __exit tegra_firmwares_exit(void)
{
	struct device **dev = firmwares;

	tegra_cpu_fw_unregister(tegra_firmwares_cpu_state);
	while (dev - firmwares < ARRAY_SIZE(firmwares))
		tegrafw_unregister(*dev++);
}
module_init(tegra_firmwares_init);
module_exit(tegra_firmwares_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("dmitry pervushin <dpervushin@nvidia.com>");
