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
#include <linux/tegra-mce.h>

#define MASK GENMASK(15, 12)

void t19x_flush_cache_all(void)
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

void t19x_flush_dcache_all(void)
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

void t19x_clean_dcache_all(void)
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

static int __init tegra19x_cache_op_init(void)
{
	tegra_flush_cache_all = t19x_flush_cache_all;
	tegra_flush_dcache_all = t19x_flush_dcache_all;
	tegra_clean_dcache_all = t19x_clean_dcache_all;
	return 0;
}
arch_initcall(tegra19x_cache_op_init);

MODULE_DESCRIPTION("T19x Cache operations registration");
MODULE_AUTHOR("Sri Krishna chowdary K <schowdary@nvidia.com>");
MODULE_LICENSE("GPL v2");
