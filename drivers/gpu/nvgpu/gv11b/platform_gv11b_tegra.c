/*
 * GV11B Tegra Platform Interface
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/nvmap.h>
#include <linux/reset.h>
#include <linux/hashtable.h>
#ifdef CONFIG_TEGRA_GK20A_NVHOST
#include <linux/nvhost_t194.h>
#endif

#include <uapi/linux/nvgpu.h>

#include "gk20a/platform_gk20a.h"
#include "gk20a/gk20a.h"
#include "tegra/linux/clk.h"

#include "gp10b/platform_gp10b.h"
#include "tegra/linux/platform_gp10b_tegra.h"

#include "tegra/linux/platform_gk20a_tegra.h"
#include "gr_gv11b.h"
#include "nvgpu_gpuid_t19x.h"

static void gr_gv11b_remove_sysfs(struct device *dev);

static int gv11b_tegra_probe(struct device *dev)
{
	int err = 0;
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a *g = platform->g;
	struct device_node *np = dev->of_node;
	struct device_node *host1x_node;
	struct platform_device *host1x_pdev;
	const __be32 *host1x_ptr;

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	host1x_ptr = of_get_property(np, "nvidia,host1x", NULL);
	if (!host1x_ptr) {
		dev_err(dev, "host1x device not available");
		return -ENOSYS;
	}

	host1x_node = of_find_node_by_phandle(be32_to_cpup(host1x_ptr));
	host1x_pdev = of_find_device_by_node(host1x_node);
	if (!host1x_pdev) {
		dev_err(dev, "host1x device not available");
		return -ENOSYS;
	}

	platform->g->host1x_dev = host1x_pdev;
	err = nvhost_syncpt_unit_interface_get_aperture(
				g->host1x_dev,
				&g->syncpt_unit_base,
				&g->syncpt_unit_size);
	if (err) {
		dev_err(dev, "Failed to get syncpt interface");
		return -ENOSYS;
	}
	g->syncpt_size = nvhost_syncpt_unit_interface_get_byte_offset(1);
	gk20a_dbg_info("syncpt_unit_base %llx syncpt_unit_size %zx size %x\n",
			g->syncpt_unit_base, g->syncpt_unit_size,
			g->syncpt_size);
#endif

	platform->bypass_smmu = !device_is_iommuable(dev);
	platform->disable_bigpage = platform->bypass_smmu;

	platform->g->gr.t18x.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;
	platform->g->gr.t18x.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;

	platform->g->gr.t18x.ctx_vars.force_preemption_gfxp = false;
	platform->g->gr.t18x.ctx_vars.force_preemption_cilp = false;

	/* soc memory accessed as system memory aperture */
	platform->g->mm.vidmem_is_vidmem = platform->vidmem_is_vidmem;

	gp10b_tegra_get_clocks(dev);
	nvgpu_linux_init_clk_support(platform->g);

	return 0;
}

static int gv11b_tegra_remove(struct device *dev)
{
	gp10b_tegra_remove(dev);

	gr_gv11b_remove_sysfs(dev);

	return 0;
}

static bool gv11b_tegra_is_railgated(struct device *dev)
{
	bool ret = false;

	return ret;
}

static int gv11b_tegra_railgate(struct device *dev)
{
	return 0;
}

static int gv11b_tegra_unrailgate(struct device *dev)
{
	int ret = 0;
	return ret;
}

static int gv11b_tegra_suspend(struct device *dev)
{
	return 0;
}

struct gk20a_platform t19x_gpu_tegra_platform = {
	.has_syncpoints = false,

	/* power management configuration */

	/* ptimer src frequency in hz*/
	.ptimer_src_freq	= 31250000,

	.probe = gv11b_tegra_probe,
	.remove = gv11b_tegra_remove,

	/* power management callbacks */
	.suspend = gv11b_tegra_suspend,
	.railgate = gv11b_tegra_railgate,
	.unrailgate = gv11b_tegra_unrailgate,
	.is_railgated = gv11b_tegra_is_railgated,

	.busy = gk20a_tegra_busy,
	.idle = gk20a_tegra_idle,

	.dump_platform_dependencies = gk20a_tegra_debug_dump,

	.default_big_page_size	= SZ_64K,

	.soc_name = "tegra19x",

	.vidmem_is_vidmem = true,

	.reset_assert = gp10b_tegra_reset_assert,
	.reset_deassert = gp10b_tegra_reset_deassert,
};

static struct device_attribute *dev_attr_sm_l1_tag_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_sm_l1_tag_ecc_uncorrected_err_count_array;
static struct device_attribute *dev_attr_sm_cbu_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_sm_cbu_ecc_uncorrected_err_count_array;
static struct device_attribute *dev_attr_sm_l1_data_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_sm_l1_data_ecc_uncorrected_err_count_array;
static struct device_attribute *dev_attr_sm_icache_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_sm_icache_ecc_uncorrected_err_count_array;
static struct device_attribute *dev_attr_gcc_l15_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_gcc_l15_ecc_uncorrected_err_count_array;
static struct device_attribute *dev_attr_mmu_l1tlb_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_mmu_l1tlb_ecc_uncorrected_err_count_array;

static struct device_attribute *dev_attr_fecs_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_fecs_ecc_uncorrected_err_count_array;
static struct device_attribute *dev_attr_gpccs_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_gpccs_ecc_uncorrected_err_count_array;

static struct device_attribute *dev_attr_l2_cache_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_l2_cache_ecc_uncorrected_err_count_array;

static struct device_attribute *dev_attr_mmu_l2tlb_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_mmu_l2tlb_ecc_uncorrected_err_count_array;
static struct device_attribute *dev_attr_mmu_hubtlb_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_mmu_hubtlb_ecc_uncorrected_err_count_array;
static struct device_attribute *dev_attr_mmu_fillunit_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_mmu_fillunit_ecc_uncorrected_err_count_array;

void gr_gv11b_create_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int error = 0;
	/* This stat creation function is called on GR init. GR can get
       initialized multiple times but we only need to create the ECC
       stats once. Therefore, add the following check to avoid
       creating duplicate stat sysfs nodes. */
	if (g->ecc.gr.t19x.sm_l1_tag_corrected_err_count.counters != NULL)
		return;

	gr_gp10b_create_sysfs(dev);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_l1_tag_ecc_corrected_err_count",
				&g->ecc.gr.t19x.sm_l1_tag_corrected_err_count,
				dev_attr_sm_l1_tag_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_l1_tag_ecc_uncorrected_err_count",
				&g->ecc.gr.t19x.sm_l1_tag_uncorrected_err_count,
				dev_attr_sm_l1_tag_ecc_uncorrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_cbu_ecc_corrected_err_count",
				&g->ecc.gr.t19x.sm_cbu_corrected_err_count,
				dev_attr_sm_cbu_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_cbu_ecc_uncorrected_err_count",
				&g->ecc.gr.t19x.sm_cbu_uncorrected_err_count,
				dev_attr_sm_cbu_ecc_uncorrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_l1_data_ecc_corrected_err_count",
				&g->ecc.gr.t19x.sm_l1_data_corrected_err_count,
				dev_attr_sm_l1_data_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_l1_data_ecc_uncorrected_err_count",
				&g->ecc.gr.t19x.sm_l1_data_uncorrected_err_count,
				dev_attr_sm_l1_data_ecc_uncorrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_icache_ecc_corrected_err_count",
				&g->ecc.gr.t19x.sm_icache_corrected_err_count,
				dev_attr_sm_icache_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_icache_ecc_uncorrected_err_count",
				&g->ecc.gr.t19x.sm_icache_uncorrected_err_count,
				dev_attr_sm_icache_ecc_uncorrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"gcc_l15_ecc_corrected_err_count",
				&g->ecc.gr.t19x.gcc_l15_corrected_err_count,
				dev_attr_gcc_l15_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"gcc_l15_ecc_uncorrected_err_count",
				&g->ecc.gr.t19x.gcc_l15_uncorrected_err_count,
				dev_attr_gcc_l15_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->ltc_count,
				"ltc",
				"l2_cache_uncorrected_err_count",
				&g->ecc.ltc.t19x.l2_cache_uncorrected_err_count,
				dev_attr_l2_cache_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->ltc_count,
				"ltc",
				"l2_cache_corrected_err_count",
				&g->ecc.ltc.t19x.l2_cache_corrected_err_count,
				dev_attr_l2_cache_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				"gpc",
				"fecs_ecc_uncorrected_err_count",
				&g->ecc.gr.t19x.fecs_uncorrected_err_count,
				dev_attr_fecs_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				"gpc",
				"fecs_ecc_corrected_err_count",
				&g->ecc.gr.t19x.fecs_corrected_err_count,
				dev_attr_fecs_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->gr.gpc_count,
				"gpc",
				"gpccs_ecc_uncorrected_err_count",
				&g->ecc.gr.t19x.gpccs_uncorrected_err_count,
				dev_attr_gpccs_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->gr.gpc_count,
				"gpc",
				"gpccs_ecc_corrected_err_count",
				&g->ecc.gr.t19x.gpccs_corrected_err_count,
				dev_attr_gpccs_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->gr.gpc_count,
				"gpc",
				"mmu_l1tlb_ecc_uncorrected_err_count",
				&g->ecc.gr.t19x.mmu_l1tlb_uncorrected_err_count,
				dev_attr_mmu_l1tlb_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->gr.gpc_count,
				"gpc",
				"mmu_l1tlb_ecc_corrected_err_count",
				&g->ecc.gr.t19x.mmu_l1tlb_corrected_err_count,
				dev_attr_mmu_l1tlb_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				"eng",
				"mmu_l2tlb_ecc_uncorrected_err_count",
				&g->ecc.eng.t19x.mmu_l2tlb_uncorrected_err_count,
				dev_attr_mmu_l2tlb_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				"eng",
				"mmu_l2tlb_ecc_corrected_err_count",
				&g->ecc.eng.t19x.mmu_l2tlb_corrected_err_count,
				dev_attr_mmu_l2tlb_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				"eng",
				"mmu_hubtlb_ecc_uncorrected_err_count",
				&g->ecc.eng.t19x.mmu_hubtlb_uncorrected_err_count,
				dev_attr_mmu_hubtlb_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				"eng",
				"mmu_hubtlb_ecc_corrected_err_count",
				&g->ecc.eng.t19x.mmu_hubtlb_corrected_err_count,
				dev_attr_mmu_hubtlb_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				"eng",
				"mmu_fillunit_ecc_uncorrected_err_count",
				&g->ecc.eng.t19x.mmu_fillunit_uncorrected_err_count,
				dev_attr_mmu_fillunit_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				"eng",
				"mmu_fillunit_ecc_corrected_err_count",
				&g->ecc.eng.t19x.mmu_fillunit_corrected_err_count,
				dev_attr_mmu_fillunit_ecc_corrected_err_count_array);

	if (error)
		dev_err(dev, "Failed to create gv11b sysfs attributes!\n");
}

static void gr_gv11b_remove_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.sm_l1_tag_corrected_err_count,
			dev_attr_sm_l1_tag_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.sm_l1_tag_uncorrected_err_count,
			dev_attr_sm_l1_tag_ecc_uncorrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.sm_cbu_corrected_err_count,
			dev_attr_sm_cbu_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.sm_cbu_uncorrected_err_count,
			dev_attr_sm_cbu_ecc_uncorrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.sm_l1_data_corrected_err_count,
			dev_attr_sm_l1_data_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.sm_l1_data_uncorrected_err_count,
			dev_attr_sm_l1_data_ecc_uncorrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.sm_icache_corrected_err_count,
			dev_attr_sm_icache_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.sm_icache_uncorrected_err_count,
			dev_attr_sm_icache_ecc_uncorrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.gcc_l15_corrected_err_count,
			dev_attr_gcc_l15_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t19x.gcc_l15_uncorrected_err_count,
			dev_attr_gcc_l15_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->ltc_count,
			&g->ecc.ltc.t19x.l2_cache_uncorrected_err_count,
			dev_attr_l2_cache_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->ltc_count,
			&g->ecc.ltc.t19x.l2_cache_corrected_err_count,
			dev_attr_l2_cache_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.gr.t19x.fecs_uncorrected_err_count,
			dev_attr_fecs_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.gr.t19x.fecs_corrected_err_count,
			dev_attr_fecs_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->gr.gpc_count,
			&g->ecc.gr.t19x.gpccs_uncorrected_err_count,
			dev_attr_gpccs_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->gr.gpc_count,
			&g->ecc.gr.t19x.gpccs_corrected_err_count,
			dev_attr_gpccs_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->gr.gpc_count,
			&g->ecc.gr.t19x.mmu_l1tlb_uncorrected_err_count,
			dev_attr_mmu_l1tlb_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->gr.gpc_count,
			&g->ecc.gr.t19x.mmu_l1tlb_corrected_err_count,
			dev_attr_mmu_l1tlb_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.eng.t19x.mmu_l2tlb_uncorrected_err_count,
			dev_attr_mmu_l2tlb_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.eng.t19x.mmu_l2tlb_corrected_err_count,
			dev_attr_mmu_l2tlb_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.eng.t19x.mmu_hubtlb_uncorrected_err_count,
			dev_attr_mmu_hubtlb_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.eng.t19x.mmu_hubtlb_corrected_err_count,
			dev_attr_mmu_hubtlb_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.eng.t19x.mmu_fillunit_uncorrected_err_count,
			dev_attr_mmu_fillunit_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.eng.t19x.mmu_fillunit_corrected_err_count,
			dev_attr_mmu_fillunit_ecc_corrected_err_count_array);
}
