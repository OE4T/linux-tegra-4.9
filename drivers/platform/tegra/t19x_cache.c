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
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/tegra-mce.h>

#define MASK GENMASK(15, 12)

#define CCPLEX_CACHE_CONTROL		49
#define CCPLEX_CC_GPU_ONLY_BITS_SHIFT	8

#define MAX_L3_WAYS			16

static void t19x_flush_cache_all(void)
{
	u64 id_afr0;
	u64 ret;
	u64 retry = 10;

	asm volatile ("mrs %0, ID_AFR0_EL1" : "=r"(id_afr0));
	/* check if cache flush through mts is supported */
	if (likely(id_afr0 & MASK)) {
		do {
			asm volatile ("mrs %0, s3_0_c15_c3_7" : "=r" (ret));
			WARN_ONCE(retry-- == 0, "%s failed\n", __func__);
			if (!retry)
				break;
		} while (!ret);
		asm volatile ("dsb sy");
	} else {
		tegra_roc_flush_cache();
	}
}

static void t19x_flush_dcache_all(void)
{
	u64 id_afr0;
	u64 ret;
	u64 retry = 10;

	asm volatile ("mrs %0, ID_AFR0_EL1" : "=r"(id_afr0));
	/* check if cache flush through mts is supported */
	if (likely(id_afr0 & MASK)) {
		do {
			asm volatile ("mrs %0, s3_0_c15_c3_6" : "=r" (ret));
			WARN_ONCE(retry-- == 0, "%s failed\n", __func__);
			if (!retry)
				break;
		} while (!ret);
		asm volatile ("dsb sy");
	} else {
		tegra_roc_flush_cache_only();
	}
}

static void t19x_clean_dcache_all(void)
{
	u64 id_afr0;
	u64 ret;
	u64 retry = 10;

	asm volatile ("mrs %0, ID_AFR0_EL1" : "=r"(id_afr0));
	/* check if cache flush through mts is supported */
	if (likely(id_afr0 & MASK)) {
		do {
			asm volatile ("mrs %0, s3_0_c15_c3_5" : "=r" (ret));
			WARN_ONCE(retry-- == 0, "%s failed\n", __func__);
			if (!retry)
				break;
		} while (!ret);
		asm volatile ("dsb sy");
	} else {
		tegra_roc_clean_cache();
	}
}

static int t19x_set_l3_cache_ways(u64 gpu_only_ways, u64 gpu_ways)
{
	u64 nvg_index = CCPLEX_CACHE_CONTROL;
	u64 nvg_data;
	u64 ret;

	if ((gpu_only_ways > MAX_L3_WAYS) || (gpu_ways > MAX_L3_WAYS)) {
		pr_err("gpu_ways:%llu or gpu_only_ways:%llu exceeds 16!!\n",
			gpu_ways, gpu_only_ways);
		return -EINVAL;
	}

	if (gpu_ways < gpu_only_ways) {
		pr_err("gpu_ways:%llu is smaller than gpu_only_ways:%llu\n",
			gpu_ways, gpu_only_ways);
		return -EINVAL;
	}

	gpu_only_ways <<= CCPLEX_CC_GPU_ONLY_BITS_SHIFT;
	nvg_data = gpu_ways | gpu_only_ways;

	asm volatile("msr s3_0_c15_c1_3, %0" : : "r" (nvg_data));
	asm volatile("msr s3_0_c15_c1_2, %0" : : "r" (nvg_index));
	asm volatile ("mrs %0, s3_0_c15_c1_3" : "=r" (ret));

	if (ret != nvg_data) {
		pr_err("CCPLEX_CACHE_CONTROL contents are not updated!!\n");
		return -ENODEV;
	}
	return 0;
}

static int __init tegra19x_cache_op_init(void)
{
	struct device_node *np;
	u64 gpu_ways, gpu_only_ways;
	int ret = 0;

	tegra_flush_cache_all = t19x_flush_cache_all;
	tegra_flush_dcache_all = t19x_flush_dcache_all;
	tegra_clean_dcache_all = t19x_clean_dcache_all;

	for_each_compatible_node(np, NULL, "nvidia,cache-t19x") {
		ret = of_property_read_u64_array(np, "l3-gpu-ways",
					&gpu_ways, 1);
		if (ret <= 0) {
			pr_err("Did not find l3-gpu-ways property\n");
			return ret;
		}

		ret = of_property_read_u64_array(np, "l3-gpu-only-ways",
					&gpu_only_ways, 1);
		if (ret <= 0) {
			pr_err("Did not find l3-gpu-only-ways property\n");
			return ret;
		}

		ret = t19x_set_l3_cache_ways(gpu_only_ways, gpu_ways);
		if (ret)
			pr_err("Could not set gpu_ways in L3\n");
		else
			pr_info("set L3 gpu-ways:%llu and gpu-only-ways:%llu\n",
				gpu_ways, gpu_only_ways);
	}

	return ret;
}
arch_initcall(tegra19x_cache_op_init);

MODULE_DESCRIPTION("T19x Cache operations registration");
MODULE_AUTHOR("Sri Krishna chowdary K <schowdary@nvidia.com>");
MODULE_LICENSE("GPL v2");
