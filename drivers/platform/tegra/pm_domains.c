/*
 * drivers/platform/tegra/pm_domains.c
 *
 * Copyright (c) 2012-2017, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/module.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/tegra_pm_domains.h>
#include <linux/tegra-powergate.h>
#include <soc/tegra/bpmp_t210_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#ifdef CONFIG_TEGRA_APE_AGIC
#include <linux/irqchip/tegra-agic.h>
#endif
#include <linux/slab.h>
#include <linux/wakelock.h>

#define TEGRA_PD_DEV_CALLBACK(callback, dev)			\
({								\
	int (*__routine)(struct device *__d);			\
	int __ret = 0;						\
								\
	if (dev->type && dev->type->pm)				\
		__routine = dev->type->pm->callback;		\
	else if (dev->class && dev->class->pm)			\
		__routine = dev->class->pm->callback;		\
	else if (dev->bus && dev->bus->pm)			\
		__routine = dev->bus->pm->callback;		\
	else							\
		__routine = NULL;				\
								\
	if (!__routine && dev->driver && dev->driver->pm)	\
		__routine = dev->driver->pm->callback;		\
								\
	if (__routine)						\
		__ret = __routine(dev);				\
	__ret;							\
})

#ifdef CONFIG_ARCH_TEGRA_21x_SOC
static int tegra_mc_clk_power_off(struct generic_pm_domain *genpd)
{
	int32_t val = cpu_to_le32(true);
	struct tegra_pm_domain *pd = to_tegra_pd(genpd);

	if (!pd)
		return -EINVAL;

	tegra_bpmp_send(MRQ_SCX_ENABLE, &val, sizeof(val));

	return 0;
}

static int tegra_mc_clk_power_on(struct generic_pm_domain *genpd)
{
	int val = cpu_to_le32(false);
	struct tegra_pm_domain *pd = to_tegra_pd(genpd);

	if (!pd)
		return -EINVAL;

	tegra_bpmp_send(MRQ_SCX_ENABLE, &val, sizeof(val));

	return 0;
}
#endif

#if defined(CONFIG_ARCH_TEGRA_APE)

enum {
	APE_CLK = 0,
	APB2APE_CLK,
	ADSP_CLK,
	APE_MAX_CLK,
};

struct tegra_ape_clks {
	char *name;
	int idx;
};

static struct tegra_ape_clks ape_pd_clks[] = {
	{"ape", APE_CLK},
	{"apb2ape", APB2APE_CLK},
	{"adsp", ADSP_CLK},
};

/* State of APE power domain */
static bool tegra_ape_pd_initialized;

/* APE wakelock is needed to preserve AGIC state */
static struct wake_lock tegra_ape_wakelock;

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
#define APE_CLK_GET(name) clk_get_sys(NULL, name)
#else
#define APE_CLK_GET(name) of_clk_get_by_name(ape_pd->gpd.of_node, name)
#endif /* CONFIG_ARCH_TEGRA_21x_SOC */

static int tegra_ape_pd_enable_clks(struct tegra_pm_domain *ape_pd)
{
	int i;
	int ret = 0;

	for (i = 0; i < APE_MAX_CLK; i++) {
		int clk_idx = ape_pd_clks[i].idx;

		if (unlikely(!ape_pd->clk[clk_idx])) {
			struct clk *clk;

			clk = APE_CLK_GET(ape_pd_clks[i].name);
			if (IS_ERR_OR_NULL(clk)) {
				pr_err("%s:unable to find %s clock\n"
				       , __func__, ape_pd_clks[i].name);
				ret = PTR_ERR(clk);
				return ret;
			}
			ape_pd->clk[clk_idx] = clk;
		}

		ret = clk_prepare_enable(ape_pd->clk[clk_idx]);
		if (ret) {
			pr_err("%s:unable to enable %s clock\n",
			       __func__, ape_pd_clks[i].name);
			return ret;
		}
	}
	return ret;
}

static int tegra_ape_pd_disable_clks(struct tegra_pm_domain *ape_pd)
{
	int i;
	int ret = 0;

	for (i = 0; i < APE_MAX_CLK; i++) {
		int clk_idx = ape_pd_clks[i].idx;

		if (unlikely(!ape_pd->clk[clk_idx])) {
			struct clk *clk;

			clk = APE_CLK_GET(ape_pd_clks[i].name);
			if (IS_ERR_OR_NULL(clk)) {
				pr_err("%s:unable to find %s clock\n"
				       , __func__, ape_pd_clks[i].name);
				ret = PTR_ERR(clk);
				return ret;
			}
			ape_pd->clk[clk_idx] = clk;
		}

		/* The adsp clock is already disabled */
		if (clk_idx == ADSP_CLK)
			continue;

		clk_disable_unprepare(ape_pd->clk[clk_idx]);
	}
	return ret;
}

static int tegra_ape_power_on(struct generic_pm_domain *genpd)
{
	struct tegra_pm_domain *ape_pd;
	struct pm_domain_data *pdd;
	int ret = 0;
	int partition_id;
	struct device_node *dn;

	dn = genpd->of_node;
	ret = of_property_read_u32(dn, "partition-id", &partition_id);
	if (ret)
		return -EINVAL;

	wake_lock(&tegra_ape_wakelock);

	ape_pd = to_tegra_pd(genpd);

	/* check before clocks are enabled */
	tegra_ape_pd_initialized = ape_pd->clk[APE_CLK] ? true : false;

	ret = tegra_ape_pd_enable_clks(ape_pd);
	if (ret) {
		wake_unlock(&tegra_ape_wakelock);
		return ret;
	}

	if (tegra_ape_pd_initialized) {
		ret = tegra_unpowergate_partition(partition_id);
		if (ret) {
			tegra_ape_pd_disable_clks(ape_pd);
			wake_unlock(&tegra_ape_wakelock);
			return ret;
		}

		/*
		 * The adsp clock shall be enabled only when ADSP cpu is
		 * on. ADSP cpu is switched on only for ADSP use-cases
		 * and the ADSP driver enables the adsp clock.
		 */
		clk_disable_unprepare(ape_pd->clk[ADSP_CLK]);

#ifdef CONFIG_TEGRA_APE_AGIC
		tegra_agic_restore_registers();
#endif
		list_for_each_entry(pdd, &genpd->dev_list, list_node)
			TEGRA_PD_DEV_CALLBACK(resume, pdd->dev);
	} else {
		/*
		 * The adsp clock shall be enabled only when ADSP cpu is
		 * on. ADSP cpu is switched on only for ADSP use-cases
		 * and the ADSP driver enables the adsp clock.
		 */
		clk_disable_unprepare(ape_pd->clk[ADSP_CLK]);
	}

	return ret;
}

static int tegra_ape_power_off(struct generic_pm_domain *genpd)
{
	struct tegra_pm_domain *ape_pd;
	struct pm_domain_data *pdd;
	int ret = 0;
	int partition_id;
	struct device_node *dn;

	dn = genpd->of_node;
	ret = of_property_read_u32(dn, "partition-id", &partition_id);
	if (ret)
		return -EINVAL;

	list_for_each_entry(pdd, &genpd->dev_list, list_node)
		TEGRA_PD_DEV_CALLBACK(suspend, pdd->dev);

#ifdef CONFIG_TEGRA_APE_AGIC
	tegra_agic_save_registers();
#endif
	ape_pd = to_tegra_pd(genpd);

	ret = tegra_ape_pd_disable_clks(ape_pd);
	if (ret) {
		wake_unlock(&tegra_ape_wakelock);
		return ret;
	}

	ret = tegra_powergate_partition(partition_id);

	wake_unlock(&tegra_ape_wakelock);
	return ret;
}

#endif

typedef int (*of_tegra_pd_init_cb_t)(struct generic_pm_domain *);

static int __init tegra_init_mc_clk(struct generic_pm_domain *pd)
{
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	pd->power_off = tegra_mc_clk_power_off;
	pd->power_on = tegra_mc_clk_power_on;
#endif
	return 0;
}

#if defined(CONFIG_ARCH_TEGRA_APE)
static int __init tegra_init_ape(struct generic_pm_domain *pd)
{
	pd->power_off = tegra_ape_power_off;
	pd->power_on = tegra_ape_power_on;

	wake_lock_init(&tegra_ape_wakelock, WAKE_LOCK_SUSPEND,
		       "tegra-ape-pd");
	return 0;
}
#else
static int __init tegra_init_ape(struct generic_pm_domain *pd)
{
	return 0;
}
#endif

/* Do not add to this list */
static const struct of_device_id tegra_pd_match[] __initconst = {
	{.compatible = "nvidia,tegra210-mc-clk-pd", .data = tegra_init_mc_clk},
	{.compatible = "nvidia,tegra210-ape-pd", .data = tegra_init_ape },
	{.compatible = "nvidia,tegra210-adsp-pd", .data = NULL},
	{.compatible = "nvidia,tegra186-ape-pd", .data = tegra_init_ape },
	{.compatible = "nvidia,tegra186-adsp-pd", .data = NULL},
	{},
};

/* node: device tree node of the child power domain */
static int attach_subdomain(struct device_node *node)
{
	int ret;
	struct of_phandle_args master_phandle, child_phandle;

	child_phandle.np = node;
	child_phandle.args_count = 0;

	ret = of_parse_phandle_with_args(node, "power-domains",
					 "#power-domain-cells", 0,
					 &master_phandle);

	if (ret < 0)
		return ret;

	pr_info("Adding domain %s to PM domain %s\n",
		node->name, master_phandle.np->name);
	of_genpd_add_subdomain(&master_phandle, &child_phandle);

	return 0;
}

static int __init tegra_init_pd(struct device_node *np)
{
	struct tegra_pm_domain *tpd;
	struct generic_pm_domain *gpd;
	of_tegra_pd_init_cb_t tpd_init_cb;
	const struct of_device_id *match = of_match_node(tegra_pd_match, np);
	bool is_off = false;

	tpd = (struct tegra_pm_domain *)kzalloc
			(sizeof(struct tegra_pm_domain), GFP_KERNEL);
	if (!tpd) {
		pr_err("Failed to allocate memory for %s domain\n",
					of_node_full_name(np));
		return -ENOMEM;
	}

	gpd = &tpd->gpd;
	gpd->name = (char *)np->name;
	tpd_init_cb = match->data;

	if (tpd_init_cb)
		tpd_init_cb(gpd);

	if (of_property_read_bool(np, "is_off"))
		is_off = true;

	pm_genpd_init(gpd, &simple_qos_governor, is_off);

	of_genpd_add_provider_simple(np, gpd);

	attach_subdomain(np);
	return 0;
}

static int __init tegra_init_pm_domain(void)
{
	struct device_node *np;
	int ret = 0;

	for_each_matching_node(np, tegra_pd_match) {
		ret = tegra_init_pd(np);
		if (ret)
			return ret;
	}

	return ret;
}
core_initcall(tegra_init_pm_domain);

int tegra_pd_get_powergate_id(const struct of_device_id *dev_id)
{
	struct device_node *dn = NULL;
	u32 partition_id;
	int ret = -EINVAL;

	for_each_matching_node(dn, dev_id) {
		ret = of_property_read_u32(dn, "partition-id", &partition_id);
		break;
	}

	if (ret) {
		pr_err("Reading powergate-id failed\n");
		return ret;
	}

	return partition_id;
}
EXPORT_SYMBOL(tegra_pd_get_powergate_id);

int tegra_pd_add_domain(struct of_device_id *dev_id,
					struct generic_pm_domain *gpd)
{
#ifdef CONFIG_PM_GENERIC_DOMAINS
	struct tegra_pm_domain *tpd;
	struct device_node *dn = NULL;

	for_each_matching_node(dn, dev_id) {
		tpd = to_tegra_pd(gpd);
		if (!gpd)
			return -ENOMEM;

		gpd->name = (char *)dn->name;

		if (of_property_read_bool(dn, "is_off"))
			tpd->is_off = true;
		if (of_property_read_u32(dn, "partition-id",
						&tpd->partition_id))
			return -EINVAL;
		pm_genpd_init(gpd, NULL, tpd->is_off);
		of_genpd_add_provider_simple(dn, gpd);

		attach_subdomain(dn);
	}
#endif
	return 0;
}
EXPORT_SYMBOL(tegra_pd_add_domain);
