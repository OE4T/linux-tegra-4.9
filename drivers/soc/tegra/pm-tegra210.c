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
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/syscore_ops.h>

#include <soc/tegra/flowctrl.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/tegra_bpmp.h>

struct tegra210_pm_data {
	bool cc4_no_retention;
};
static struct tegra210_pm_data t210_pm_data;

static void do_cc4_init(void)
{
	flowctrl_update(FLOW_CTLR_CC4_HVC_CONTROL,
			2 << 3 | FLOW_CTRL_CC4_HVC_ENABLE);
	flowctrl_update(FLOW_CTRL_CC4_RETENTION_CONTROL, 2 << 3);
	flowctrl_update(FLOW_CTRL_CC4_HVC_RETRY, 2);
}

static struct syscore_ops cc4_syscore_ops = {
	.restore = do_cc4_init,
	.resume = do_cc4_init
};

static int tegra210_cpuidle_cc4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator *reg;
	uint32_t uv;
	int r;

	/* T210 BPMP supports CC4 retention only with max77621 or ovr2. */
	t210_pm_data.cc4_no_retention = of_property_read_bool(dev->of_node,
							"cc4-no-retention");

	/* If cc4-microvolt is not found, assume not max77621 */
	if (of_property_read_u32(dev->of_node, "cc4-microvolt", &uv))
		goto out;

	reg = regulator_get(dev, "vdd-cpu");
	if (IS_ERR(reg)) {
		dev_err(dev, "vdd-cpu regulator get failed\n");
		return PTR_ERR(reg);
	}

	r = regulator_set_sleep_voltage(reg, uv - 100000, uv + 100000);
	if (r)
		dev_err(dev, "failed to set retention voltage: %d\n", r);

	dev_info(dev, "retention voltage is %u uv\n", uv);
out:
	do_cc4_init();
	register_syscore_ops(&cc4_syscore_ops);

	return 0;
}

static const struct of_device_id tegra210_cpuidle_of[] = {
	{ .compatible = "nvidia,tegra210-cpuidle" },
	{}
};

static struct platform_driver tegra210_cpuidle_driver = {
	.probe = tegra210_cpuidle_cc4_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cpuidle-tegra210",
		.of_match_table = tegra210_cpuidle_of,
	}
};

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
	platform_driver_register(&tegra210_cpuidle_driver);

out:
	return 0;
}
device_initcall(tegra210_pm_init);
