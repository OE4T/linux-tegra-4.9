/*
 * GV11B Tegra Platform Interface
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/nvmap.h>
#include <linux/reset.h>
#include <linux/hashtable.h>
#include <linux/clk.h>
#include <linux/platform/tegra/emc_bwmgr.h>

#include <nvgpu/nvhost.h>

#include <uapi/linux/nvgpu.h>

#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/tegra_powergate.h>

#include "gk20a/gk20a.h"
#include "platform_gk20a.h"
#include "clk.h"
#include "scale.h"

#include "gp10b/platform_gp10b.h"
#include "platform_gp10b_tegra.h"

#include "os_linux.h"
#include "platform_gk20a_tegra.h"
#include "gv11b/gr_gv11b.h"

static void gr_gv11b_remove_sysfs(struct device *dev);

static int gv11b_tegra_probe(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int err;

	err = nvgpu_nvhost_syncpt_init(platform->g);
	if (err) {
		if (err != -ENOSYS)
			return err;
	}

	err = gk20a_tegra_init_secure_alloc(platform);
	if (err)
		return err;

	platform->disable_bigpage = !device_is_iommuable(dev);

	platform->g->gr.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;
	platform->g->gr.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;

	platform->g->gr.ctx_vars.force_preemption_gfxp = false;
	platform->g->gr.ctx_vars.force_preemption_cilp = false;

	gp10b_tegra_get_clocks(dev);
	nvgpu_linux_init_clk_support(platform->g);

	return 0;
}

static int gv11b_tegra_late_probe(struct device *dev)
{
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
#ifdef TEGRA194_POWER_DOMAIN_GPU
	struct gk20a *g = get_gk20a(dev);

	if (tegra_bpmp_running()) {
		nvgpu_log(g, gpu_dbg_info, "bpmp running");
		ret = !tegra_powergate_is_powered(TEGRA194_POWER_DOMAIN_GPU);

		nvgpu_log(g, gpu_dbg_info, "railgated? %s", ret ? "yes" : "no");
	} else {
		nvgpu_log(g, gpu_dbg_info, "bpmp not running");
	}
#endif
	return ret;
}

static int gv11b_tegra_railgate(struct device *dev)
{
#ifdef TEGRA194_POWER_DOMAIN_GPU
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a *g = get_gk20a(dev);
	int i;

	/* remove emc frequency floor */
	if (profile)
		tegra_bwmgr_set_emc(
			(struct tegra_bwmgr_client *)profile->private_data,
			0, TEGRA_BWMGR_SET_EMC_FLOOR);

	if (tegra_bpmp_running()) {
		nvgpu_log(g, gpu_dbg_info, "bpmp running");
		if (!tegra_powergate_is_powered(TEGRA194_POWER_DOMAIN_GPU)) {
			nvgpu_log(g, gpu_dbg_info, "powergate is not powered");
			return 0;
		}
		nvgpu_log(g, gpu_dbg_info, "clk_disable_unprepare");
		for (i = 0; i < platform->num_clks; i++) {
			if (platform->clk[i])
				clk_disable_unprepare(platform->clk[i]);
		}
		nvgpu_log(g, gpu_dbg_info, "powergate_partition");
		tegra_powergate_partition(TEGRA194_POWER_DOMAIN_GPU);
	} else {
		nvgpu_log(g, gpu_dbg_info, "bpmp not running");
	}
#endif
	return 0;
}

static int gv11b_tegra_unrailgate(struct device *dev)
{
	int ret = 0;
#ifdef TEGRA194_POWER_DOMAIN_GPU
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	int i;

	if (tegra_bpmp_running()) {
		nvgpu_log(g, gpu_dbg_info, "bpmp running");
		ret = tegra_unpowergate_partition(TEGRA194_POWER_DOMAIN_GPU);
		if (ret) {
			nvgpu_log(g, gpu_dbg_info,
				"unpowergate partition failed");
			return ret;
		}
		nvgpu_log(g, gpu_dbg_info, "clk_prepare_enable");
		for (i = 0; i < platform->num_clks; i++) {
			if (platform->clk[i])
				clk_prepare_enable(platform->clk[i]);
		}
	} else {
		nvgpu_log(g, gpu_dbg_info, "bpmp not running");
	}

	/* to start with set emc frequency floor to max rate*/
	if (profile)
		tegra_bwmgr_set_emc(
			(struct tegra_bwmgr_client *)profile->private_data,
			tegra_bwmgr_get_max_emc_rate(),
			TEGRA_BWMGR_SET_EMC_FLOOR);
#endif
	return ret;
}

static int gv11b_tegra_suspend(struct device *dev)
{
	return 0;
}

struct gk20a_platform gv11b_tegra_platform = {
	.has_syncpoints = true,

	/* ptimer src frequency in hz*/
	.ptimer_src_freq	= 31250000,

	.ch_wdt_timeout_ms = 5000,

	.probe = gv11b_tegra_probe,
	.late_probe = gv11b_tegra_late_probe,
	.remove = gv11b_tegra_remove,
	.railgate_delay_init    = 500,
	.can_railgate_init      = true,

	.can_slcg               = true,
	.can_blcg               = true,
	.can_elcg               = true,
	.enable_slcg            = true,
	.enable_blcg            = true,
	.enable_elcg            = true,
	.enable_perfmon         = true,

	/* power management configuration */
	.enable_elpg		= true,
	.can_elpg_init		= true,
	.enable_aelpg           = true,

	/* power management callbacks */
	.suspend = gv11b_tegra_suspend,
	.railgate = gv11b_tegra_railgate,
	.unrailgate = gv11b_tegra_unrailgate,
	.is_railgated = gv11b_tegra_is_railgated,

	.busy = gk20a_tegra_busy,
	.idle = gk20a_tegra_idle,

	.clk_round_rate = gp10b_round_clk_rate,
	.get_clk_freqs = gp10b_clk_get_freqs,

	/* frequency scaling configuration */
	.initscale = gp10b_tegra_scale_init,
	.prescale = gp10b_tegra_prescale,
	.postscale = gp10b_tegra_postscale,
	.devfreq_governor = "nvhost_podgov",

	.qos_notify = gk20a_scale_qos_notify,

	.dump_platform_dependencies = gk20a_tegra_debug_dump,

	.soc_name = "tegra19x",

	.honors_aperture = true,
	.unified_memory = true,

	.reset_assert = gp10b_tegra_reset_assert,
	.reset_deassert = gp10b_tegra_reset_deassert,

	.secure_buffer_size = 667648,
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

static struct device_attribute *dev_attr_pmu_ecc_corrected_err_count_array;
static struct device_attribute *dev_attr_pmu_ecc_uncorrected_err_count_array;

void gr_gv11b_create_sysfs(struct gk20a *g)
{
	struct device *dev = dev_from_gk20a(g);
	int error = 0;
	/* This stat creation function is called on GR init. GR can get
       initialized multiple times but we only need to create the ECC
       stats once. Therefore, add the following check to avoid
       creating duplicate stat sysfs nodes. */
	if (g->ecc.gr.sm_l1_tag_corrected_err_count.counters != NULL)
		return;

	gr_gp10b_create_sysfs(g);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_l1_tag_ecc_corrected_err_count",
				&g->ecc.gr.sm_l1_tag_corrected_err_count,
				&dev_attr_sm_l1_tag_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_l1_tag_ecc_uncorrected_err_count",
				&g->ecc.gr.sm_l1_tag_uncorrected_err_count,
				&dev_attr_sm_l1_tag_ecc_uncorrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_cbu_ecc_corrected_err_count",
				&g->ecc.gr.sm_cbu_corrected_err_count,
				&dev_attr_sm_cbu_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_cbu_ecc_uncorrected_err_count",
				&g->ecc.gr.sm_cbu_uncorrected_err_count,
				&dev_attr_sm_cbu_ecc_uncorrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_l1_data_ecc_corrected_err_count",
				&g->ecc.gr.sm_l1_data_corrected_err_count,
				&dev_attr_sm_l1_data_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_l1_data_ecc_uncorrected_err_count",
				&g->ecc.gr.sm_l1_data_uncorrected_err_count,
				&dev_attr_sm_l1_data_ecc_uncorrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_icache_ecc_corrected_err_count",
				&g->ecc.gr.sm_icache_corrected_err_count,
				&dev_attr_sm_icache_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_icache_ecc_uncorrected_err_count",
				&g->ecc.gr.sm_icache_uncorrected_err_count,
				&dev_attr_sm_icache_ecc_uncorrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"gcc_l15_ecc_corrected_err_count",
				&g->ecc.gr.gcc_l15_corrected_err_count,
				&dev_attr_gcc_l15_ecc_corrected_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"gcc_l15_ecc_uncorrected_err_count",
				&g->ecc.gr.gcc_l15_uncorrected_err_count,
				&dev_attr_gcc_l15_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->ltc_count,
				0,
				"ltc",
				NULL,
				"l2_cache_uncorrected_err_count",
				&g->ecc.ltc.l2_cache_uncorrected_err_count,
				&dev_attr_l2_cache_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->ltc_count,
				0,
				"ltc",
				NULL,
				"l2_cache_corrected_err_count",
				&g->ecc.ltc.l2_cache_corrected_err_count,
				&dev_attr_l2_cache_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"gpc",
				NULL,
				"fecs_ecc_uncorrected_err_count",
				&g->ecc.gr.fecs_uncorrected_err_count,
				&dev_attr_fecs_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"gpc",
				NULL,
				"fecs_ecc_corrected_err_count",
				&g->ecc.gr.fecs_corrected_err_count,
				&dev_attr_fecs_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->gr.gpc_count,
				0,
				"gpc",
				NULL,
				"gpccs_ecc_uncorrected_err_count",
				&g->ecc.gr.gpccs_uncorrected_err_count,
				&dev_attr_gpccs_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->gr.gpc_count,
				0,
				"gpc",
				NULL,
				"gpccs_ecc_corrected_err_count",
				&g->ecc.gr.gpccs_corrected_err_count,
				&dev_attr_gpccs_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->gr.gpc_count,
				0,
				"gpc",
				NULL,
				"mmu_l1tlb_ecc_uncorrected_err_count",
				&g->ecc.gr.mmu_l1tlb_uncorrected_err_count,
				&dev_attr_mmu_l1tlb_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				g->gr.gpc_count,
				0,
				"gpc",
				NULL,
				"mmu_l1tlb_ecc_corrected_err_count",
				&g->ecc.gr.mmu_l1tlb_corrected_err_count,
				&dev_attr_mmu_l1tlb_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"eng",
				NULL,
				"mmu_l2tlb_ecc_uncorrected_err_count",
				&g->ecc.fb.mmu_l2tlb_uncorrected_err_count,
				&dev_attr_mmu_l2tlb_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"eng",
				NULL,
				"mmu_l2tlb_ecc_corrected_err_count",
				&g->ecc.fb.mmu_l2tlb_corrected_err_count,
				&dev_attr_mmu_l2tlb_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"eng",
				NULL,
				"mmu_hubtlb_ecc_uncorrected_err_count",
				&g->ecc.fb.mmu_hubtlb_uncorrected_err_count,
				&dev_attr_mmu_hubtlb_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"eng",
				NULL,
				"mmu_hubtlb_ecc_corrected_err_count",
				&g->ecc.fb.mmu_hubtlb_corrected_err_count,
				&dev_attr_mmu_hubtlb_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"eng",
				NULL,
				"mmu_fillunit_ecc_uncorrected_err_count",
				&g->ecc.fb.mmu_fillunit_uncorrected_err_count,
				&dev_attr_mmu_fillunit_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"eng",
				NULL,
				"mmu_fillunit_ecc_corrected_err_count",
				&g->ecc.fb.mmu_fillunit_corrected_err_count,
				&dev_attr_mmu_fillunit_ecc_corrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"eng",
				NULL,
				"pmu_ecc_uncorrected_err_count",
				&g->ecc.pmu.pmu_uncorrected_err_count,
				&dev_attr_pmu_ecc_uncorrected_err_count_array);

	error |= gp10b_ecc_stat_create(dev,
				1,
				0,
				"eng",
				NULL,
				"pmu_ecc_corrected_err_count",
				&g->ecc.pmu.pmu_corrected_err_count,
				&dev_attr_pmu_ecc_corrected_err_count_array);


	if (error)
		dev_err(dev, "Failed to create gv11b sysfs attributes!\n");
}

static void gr_gv11b_remove_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.sm_l1_tag_corrected_err_count,
			dev_attr_sm_l1_tag_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.sm_l1_tag_uncorrected_err_count,
			dev_attr_sm_l1_tag_ecc_uncorrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.sm_cbu_corrected_err_count,
			dev_attr_sm_cbu_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.sm_cbu_uncorrected_err_count,
			dev_attr_sm_cbu_ecc_uncorrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.sm_l1_data_corrected_err_count,
			dev_attr_sm_l1_data_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.sm_l1_data_uncorrected_err_count,
			dev_attr_sm_l1_data_ecc_uncorrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.sm_icache_corrected_err_count,
			dev_attr_sm_icache_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.sm_icache_uncorrected_err_count,
			dev_attr_sm_icache_ecc_uncorrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.gcc_l15_corrected_err_count,
			dev_attr_gcc_l15_ecc_corrected_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.gcc_l15_uncorrected_err_count,
			dev_attr_gcc_l15_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->ltc_count,
			&g->ecc.ltc.l2_cache_uncorrected_err_count,
			dev_attr_l2_cache_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->ltc_count,
			&g->ecc.ltc.l2_cache_corrected_err_count,
			dev_attr_l2_cache_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.gr.fecs_uncorrected_err_count,
			dev_attr_fecs_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.gr.fecs_corrected_err_count,
			dev_attr_fecs_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->gr.gpc_count,
			&g->ecc.gr.gpccs_uncorrected_err_count,
			dev_attr_gpccs_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->gr.gpc_count,
			&g->ecc.gr.gpccs_corrected_err_count,
			dev_attr_gpccs_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->gr.gpc_count,
			&g->ecc.gr.mmu_l1tlb_uncorrected_err_count,
			dev_attr_mmu_l1tlb_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			g->gr.gpc_count,
			&g->ecc.gr.mmu_l1tlb_corrected_err_count,
			dev_attr_mmu_l1tlb_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.fb.mmu_l2tlb_uncorrected_err_count,
			dev_attr_mmu_l2tlb_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.fb.mmu_l2tlb_corrected_err_count,
			dev_attr_mmu_l2tlb_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.fb.mmu_hubtlb_uncorrected_err_count,
			dev_attr_mmu_hubtlb_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.fb.mmu_hubtlb_corrected_err_count,
			dev_attr_mmu_hubtlb_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.fb.mmu_fillunit_uncorrected_err_count,
			dev_attr_mmu_fillunit_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.fb.mmu_fillunit_corrected_err_count,
			dev_attr_mmu_fillunit_ecc_corrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.pmu.pmu_uncorrected_err_count,
			dev_attr_pmu_ecc_uncorrected_err_count_array);

	gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.pmu.pmu_corrected_err_count,
			dev_attr_pmu_ecc_corrected_err_count_array);
}
