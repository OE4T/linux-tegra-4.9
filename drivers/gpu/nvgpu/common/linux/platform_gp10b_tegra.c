/*
 * GP10B Tegra Platform Interface
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/platform/tegra/emc_bwmgr.h>

#include <uapi/linux/nvgpu.h>

#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/tegra_powergate.h>

#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/enabled.h>
#include <nvgpu/hashtable.h>
#include <nvgpu/nvhost.h>

#include "clk.h"

#include "gk20a/platform_gk20a.h"
#include "gk20a/gk20a.h"
#include "gk20a/gk20a_scale.h"

#include "platform_gk20a_tegra.h"
#include "gp10b/platform_gp10b.h"
#include "platform_gp10b_tegra.h"

/* Select every GP10B_FREQ_SELECT_STEP'th frequency from h/w table */
#define GP10B_FREQ_SELECT_STEP	8
/* Max number of freq supported in h/w */
#define GP10B_MAX_SUPPORTED_FREQS 120
static unsigned long
gp10b_freq_table[GP10B_MAX_SUPPORTED_FREQS / GP10B_FREQ_SELECT_STEP];

#define TEGRA_GP10B_BW_PER_FREQ 64
#define TEGRA_DDR4_BW_PER_FREQ 16

#define EMC_BW_RATIO  (TEGRA_GP10B_BW_PER_FREQ / TEGRA_DDR4_BW_PER_FREQ)

static struct {
	char *name;
	unsigned long default_rate;
} tegra_gp10b_clocks[] = {
	{"gpu", 1000000000},
	{"gpu_sys", 204000000} };

static void gr_gp10b_remove_sysfs(struct device *dev);

/*
 * gp10b_tegra_get_clocks()
 *
 * This function finds clocks in tegra platform and populates
 * the clock information to gp10b platform data.
 */

int gp10b_tegra_get_clocks(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	unsigned int i;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		return 0;

	platform->num_clks = 0;
	for (i = 0; i < ARRAY_SIZE(tegra_gp10b_clocks); i++) {
		long rate = tegra_gp10b_clocks[i].default_rate;
		struct clk *c;

		c = clk_get(dev, tegra_gp10b_clocks[i].name);
		if (IS_ERR(c)) {
			nvgpu_err(platform->g, "cannot get clock %s",
					tegra_gp10b_clocks[i].name);
		} else {
			clk_set_rate(c, rate);
			platform->clk[i] = c;
		}
	}
	platform->num_clks = i;

	return 0;
}

static void gp10b_tegra_scale_init(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct tegra_bwmgr_client *bwmgr_handle;

	if (!profile)
		return;

	bwmgr_handle = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_GPU);
	if (!bwmgr_handle)
		return;

	profile->private_data = (void *)bwmgr_handle;
}

static void gp10b_tegra_scale_exit(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;

	if (profile)
		tegra_bwmgr_unregister(
			(struct tegra_bwmgr_client *)profile->private_data);
}

static int gp10b_tegra_probe(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
#ifdef CONFIG_TEGRA_GK20A_NVHOST
	int ret;

	ret = nvgpu_get_nvhost_dev(platform->g);
	if (ret)
		return ret;
#endif

	platform->bypass_smmu = !device_is_iommuable(dev);
	platform->disable_bigpage = platform->bypass_smmu;

	platform->g->gr.t18x.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;
	platform->g->gr.t18x.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;

	platform->g->gr.t18x.ctx_vars.force_preemption_gfxp = false;
	platform->g->gr.t18x.ctx_vars.force_preemption_cilp = false;

	gp10b_tegra_get_clocks(dev);
	nvgpu_linux_init_clk_support(platform->g);
	gk20a_tegra_init_secure_alloc(platform->g);

	return 0;
}

static int gp10b_tegra_late_probe(struct device *dev)
{
	/* Cause early VPR resize */
	gk20a_tegra_secure_page_alloc(dev);

	/* Initialise tegra specific scaling quirks */
	gp10b_tegra_scale_init(dev);
	return 0;
}

int gp10b_tegra_remove(struct device *dev)
{
	gr_gp10b_remove_sysfs(dev);

	/* deinitialise tegra specific scaling quirks */
	gp10b_tegra_scale_exit(dev);

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	nvgpu_free_nvhost_dev(get_gk20a(dev));
#endif

	return 0;
}

static bool gp10b_tegra_is_railgated(struct device *dev)
{
	bool ret = false;

	if (tegra_bpmp_running())
		ret = !tegra_powergate_is_powered(TEGRA186_POWER_DOMAIN_GPU);

	return ret;
}

static int gp10b_tegra_railgate(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;

	/* remove emc frequency floor */
	if (profile)
		tegra_bwmgr_set_emc(
			(struct tegra_bwmgr_client *)profile->private_data,
			0, TEGRA_BWMGR_SET_EMC_FLOOR);

	if (tegra_bpmp_running() &&
	    tegra_powergate_is_powered(TEGRA186_POWER_DOMAIN_GPU)) {
		int i;
		for (i = 0; i < platform->num_clks; i++) {
			if (platform->clk[i])
				clk_disable_unprepare(platform->clk[i]);
		}
		tegra_powergate_partition(TEGRA186_POWER_DOMAIN_GPU);
	}
	return 0;
}

static int gp10b_tegra_unrailgate(struct device *dev)
{
	int ret = 0;
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;

	if (tegra_bpmp_running()) {
		int i;
		ret = tegra_unpowergate_partition(TEGRA186_POWER_DOMAIN_GPU);
		for (i = 0; i < platform->num_clks; i++) {
			if (platform->clk[i])
				clk_prepare_enable(platform->clk[i]);
		}
	}

	/* to start with set emc frequency floor to max rate*/
	if (profile)
		tegra_bwmgr_set_emc(
			(struct tegra_bwmgr_client *)profile->private_data,
			tegra_bwmgr_get_max_emc_rate(),
			TEGRA_BWMGR_SET_EMC_FLOOR);
	return ret;
}

static int gp10b_tegra_suspend(struct device *dev)
{
	return 0;
}

int gp10b_tegra_reset_assert(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	int ret = 0;

	if (!platform->reset_control)
		return -EINVAL;

	ret = reset_control_assert(platform->reset_control);

	return ret;
}

int gp10b_tegra_reset_deassert(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	int ret = 0;

	if (!platform->reset_control)
		return -EINVAL;

	ret = reset_control_deassert(platform->reset_control);

	return ret;
}

static void gp10b_tegra_prescale(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	u32 avg = 0;

	gk20a_dbg_fn("");

	nvgpu_pmu_load_norm(g, &avg);

	gk20a_dbg_fn("done");
}

static void gp10b_tegra_postscale(struct device *pdev,
					unsigned long freq)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a *g = get_gk20a(pdev);
	unsigned long emc_rate;

	gk20a_dbg_fn("");
	if (profile && !gp10b_tegra_is_railgated(pdev)) {
		unsigned long emc_scale;

		if (freq <= gp10b_freq_table[0])
			emc_scale = 0;
		else
			emc_scale = g->emc3d_ratio;

		emc_rate = (freq * EMC_BW_RATIO * emc_scale) / 1000;

		if (emc_rate > tegra_bwmgr_get_max_emc_rate())
			emc_rate = tegra_bwmgr_get_max_emc_rate();

		tegra_bwmgr_set_emc(
			(struct tegra_bwmgr_client *)profile->private_data,
			emc_rate, TEGRA_BWMGR_SET_EMC_FLOOR);
	}
	gk20a_dbg_fn("done");
}

static long gp10b_round_clk_rate(struct device *dev, unsigned long rate)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_scale_profile *profile = g->scale_profile;
	unsigned long *freq_table = profile->devfreq_profile.freq_table;
	int max_states = profile->devfreq_profile.max_state;
	int i;

	for (i = 0; i < max_states; ++i)
		if (freq_table[i] >= rate)
			return freq_table[i];

	return freq_table[max_states - 1];
}

static int gp10b_clk_get_freqs(struct device *dev,
				unsigned long **freqs, int *num_freqs)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	unsigned long max_rate;
	unsigned long new_rate = 0, prev_rate = 0;
	int i = 0, freq_counter = 0;

	max_rate = clk_round_rate(platform->clk[0], (UINT_MAX - 1));

	/*
	 * Walk the h/w frequency table and only select
	 * GP10B_FREQ_SELECT_STEP'th frequencies and
	 * add MAX freq to last
	 */
	for (; i < GP10B_MAX_SUPPORTED_FREQS; ++i) {
		prev_rate = new_rate;
		new_rate = clk_round_rate(platform->clk[0], prev_rate + 1);

		if (i % GP10B_FREQ_SELECT_STEP == 0 ||
				new_rate == max_rate) {
			gp10b_freq_table[freq_counter++] = new_rate;

			if (new_rate == max_rate)
				break;
		}
	}

	WARN_ON(i == GP10B_MAX_SUPPORTED_FREQS);

	/* Fill freq table */
	*freqs = gp10b_freq_table;
	*num_freqs = freq_counter;

	gk20a_dbg_info("min rate: %ld max rate: %ld num_of_freq %d\n",
				gp10b_freq_table[0], max_rate, *num_freqs);

	return 0;
}

struct gk20a_platform gp10b_tegra_platform = {
	.has_syncpoints = true,

	/* power management configuration */
	.railgate_delay_init	= 500,

	/* power management configuration */
	.can_railgate_init	= true,
	.enable_elpg            = true,
	.can_elpg_init          = true,
	.enable_blcg		= true,
	.enable_slcg		= true,
	.enable_elcg		= true,
	.can_slcg               = true,
	.can_blcg               = true,
	.can_elcg               = true,
	.enable_aelpg       = true,
	.enable_perfmon         = true,

	/* ptimer src frequency in hz*/
	.ptimer_src_freq	= 31250000,

	.ch_wdt_timeout_ms = 5000,

	.probe = gp10b_tegra_probe,
	.late_probe = gp10b_tegra_late_probe,
	.remove = gp10b_tegra_remove,

	/* power management callbacks */
	.suspend = gp10b_tegra_suspend,
	.railgate = gp10b_tegra_railgate,
	.unrailgate = gp10b_tegra_unrailgate,
	.is_railgated = gp10b_tegra_is_railgated,

	.busy = gk20a_tegra_busy,
	.idle = gk20a_tegra_idle,

	.dump_platform_dependencies = gk20a_tegra_debug_dump,

	.has_cde = true,

	.clk_round_rate = gp10b_round_clk_rate,
	.get_clk_freqs = gp10b_clk_get_freqs,

	/* frequency scaling configuration */
	.prescale = gp10b_tegra_prescale,
	.postscale = gp10b_tegra_postscale,
	.devfreq_governor = "nvhost_podgov",

	.qos_notify = gk20a_scale_qos_notify,

	.reset_assert = gp10b_tegra_reset_assert,
	.reset_deassert = gp10b_tegra_reset_deassert,

	.force_reset_in_do_idle = false,

	.soc_name = "tegra18x",

	.unified_memory = true,
};


#define ECC_STAT_NAME_MAX_SIZE	100


static DEFINE_HASHTABLE(ecc_hash_table, 5);

static struct device_attribute *dev_attr_sm_lrf_ecc_single_err_count_array;
static struct device_attribute *dev_attr_sm_lrf_ecc_double_err_count_array;

static struct device_attribute *dev_attr_sm_shm_ecc_sec_count_array;
static struct device_attribute *dev_attr_sm_shm_ecc_sed_count_array;
static struct device_attribute *dev_attr_sm_shm_ecc_ded_count_array;

static struct device_attribute *dev_attr_tex_ecc_total_sec_pipe0_count_array;
static struct device_attribute *dev_attr_tex_ecc_total_ded_pipe0_count_array;
static struct device_attribute *dev_attr_tex_ecc_unique_sec_pipe0_count_array;
static struct device_attribute *dev_attr_tex_ecc_unique_ded_pipe0_count_array;
static struct device_attribute *dev_attr_tex_ecc_total_sec_pipe1_count_array;
static struct device_attribute *dev_attr_tex_ecc_total_ded_pipe1_count_array;
static struct device_attribute *dev_attr_tex_ecc_unique_sec_pipe1_count_array;
static struct device_attribute *dev_attr_tex_ecc_unique_ded_pipe1_count_array;

static struct device_attribute *dev_attr_l2_ecc_sec_count_array;
static struct device_attribute *dev_attr_l2_ecc_ded_count_array;


static u32 gen_ecc_hash_key(char *str)
{
	int i = 0;
	u32 hash_key = 0;

	while (str[i]) {
		hash_key += (u32)(str[i]);
		i++;
	};

	return hash_key;
}

static ssize_t ecc_stat_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	const char *ecc_stat_full_name = attr->attr.name;
	const char *ecc_stat_base_name;
	unsigned int hw_unit;
	struct gk20a_ecc_stat *ecc_stat;
	u32 hash_key;

	if (sscanf(ecc_stat_full_name, "ltc%u", &hw_unit) == 1) {
		ecc_stat_base_name = &(ecc_stat_full_name[strlen("ltc0_")]);
	} else if (sscanf(ecc_stat_full_name, "gpc0_tpc%u", &hw_unit) == 1) {
		ecc_stat_base_name = &(ecc_stat_full_name[strlen("gpc0_tpc0_")]);
	} else if (sscanf(ecc_stat_full_name, "gpc%u", &hw_unit) == 1) {
		ecc_stat_base_name = &(ecc_stat_full_name[strlen("gpc0_")]);
	} else if (sscanf(ecc_stat_full_name, "eng%u", &hw_unit) == 1) {
		ecc_stat_base_name = &(ecc_stat_full_name[strlen("eng0_")]);
	} else {
		return snprintf(buf,
				PAGE_SIZE,
				"Error: Invalid ECC stat name!\n");
	}

	hash_key = gen_ecc_hash_key((char *)ecc_stat_base_name);
	hash_for_each_possible(ecc_hash_table,
				ecc_stat,
				hash_node,
				hash_key) {
		if (!strcmp(ecc_stat_full_name, ecc_stat->names[hw_unit]))
			return snprintf(buf, PAGE_SIZE, "%u\n", ecc_stat->counters[hw_unit]);
	}

	return snprintf(buf, PAGE_SIZE, "Error: No ECC stat found!\n");
}

int gr_gp10b_ecc_stat_create(struct device *dev,
				int is_l2,
				char *ecc_stat_name,
				struct gk20a_ecc_stat *ecc_stat,
				struct device_attribute **dev_attr_array)
{
	struct gk20a *g = get_gk20a(dev);
	char *ltc_unit_name = "ltc";
	char *gr_unit_name = "gpc0_tpc";
	int num_hw_units = 0;

	if (is_l2)
		num_hw_units = g->ltc_count;
	else
		num_hw_units = g->gr.tpc_count;


	return gp10b_ecc_stat_create(dev, num_hw_units,
				is_l2 ? ltc_unit_name : gr_unit_name,
				ecc_stat_name,
				ecc_stat,
				dev_attr_array);
}

int gp10b_ecc_stat_create(struct device *dev,
				int num_hw_units,
				char *ecc_unit_name,
				char *ecc_stat_name,
				struct gk20a_ecc_stat *ecc_stat,
				struct device_attribute **__dev_attr_array)
{
	int error = 0;
	struct gk20a *g = get_gk20a(dev);
	int hw_unit = 0;
	u32 hash_key = 0;
	struct device_attribute *dev_attr_array;

	/* Allocate arrays */
	dev_attr_array = nvgpu_kzalloc(g, sizeof(struct device_attribute) *
				       num_hw_units);
	ecc_stat->counters = nvgpu_kzalloc(g, sizeof(u32) * num_hw_units);
	ecc_stat->names = nvgpu_kzalloc(g, sizeof(char *) * num_hw_units);
	for (hw_unit = 0; hw_unit < num_hw_units; hw_unit++) {
		ecc_stat->names[hw_unit] = nvgpu_kzalloc(g, sizeof(char) *
						ECC_STAT_NAME_MAX_SIZE);
	}

	for (hw_unit = 0; hw_unit < num_hw_units; hw_unit++) {
		/* Fill in struct device_attribute members */
			snprintf(ecc_stat->names[hw_unit],
				ECC_STAT_NAME_MAX_SIZE,
				"%s%d_%s",
				ecc_unit_name,
				hw_unit,
				ecc_stat_name);

		sysfs_attr_init(&dev_attr_array[hw_unit].attr);
		dev_attr_array[hw_unit].attr.name = ecc_stat->names[hw_unit];
		dev_attr_array[hw_unit].attr.mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO);
		dev_attr_array[hw_unit].show = ecc_stat_show;
		dev_attr_array[hw_unit].store = NULL;

		/* Create sysfs file */
		error |= device_create_file(dev, &dev_attr_array[hw_unit]);
	}

	/* Add hash table entry */
	hash_key = gen_ecc_hash_key(ecc_stat_name);
	hash_add(ecc_hash_table,
		&ecc_stat->hash_node,
		hash_key);

	*__dev_attr_array = dev_attr_array;

	return error;
}

void gr_gp10b_ecc_stat_remove(struct device *dev,
				int is_l2,
				struct gk20a_ecc_stat *ecc_stat,
				struct device_attribute *dev_attr_array)
{
	struct gk20a *g = get_gk20a(dev);
	int num_hw_units = 0;

	if (is_l2)
		num_hw_units = g->ltc_count;
	else
		num_hw_units = g->gr.tpc_count;

	gp10b_ecc_stat_remove(dev, num_hw_units, ecc_stat, dev_attr_array);
}

void gp10b_ecc_stat_remove(struct device *dev,
				int num_hw_units,
				struct gk20a_ecc_stat *ecc_stat,
				struct device_attribute *dev_attr_array)
{
	struct gk20a *g = get_gk20a(dev);
	int hw_unit = 0;

	/* Remove sysfs files */
	for (hw_unit = 0; hw_unit < num_hw_units; hw_unit++) {
		device_remove_file(dev, &dev_attr_array[hw_unit]);
	}

	/* Remove hash table entry */
	hash_del(&ecc_stat->hash_node);

	/* Free arrays */
	nvgpu_kfree(g, ecc_stat->counters);
	for (hw_unit = 0; hw_unit < num_hw_units; hw_unit++) {
		nvgpu_kfree(g, ecc_stat->names[hw_unit]);
	}
	nvgpu_kfree(g, ecc_stat->names);
	nvgpu_kfree(g, dev_attr_array);
}

void gr_gp10b_create_sysfs(struct device *dev)
{
	int error = 0;
	struct gk20a *g = get_gk20a(dev);

	/* This stat creation function is called on GR init. GR can get
	   initialized multiple times but we only need to create the ECC
	   stats once. Therefore, add the following check to avoid
	   creating duplicate stat sysfs nodes. */
	if (g->ecc.gr.t18x.sm_lrf_single_err_count.counters != NULL)
		return;

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_lrf_ecc_single_err_count",
				&g->ecc.gr.t18x.sm_lrf_single_err_count,
				&dev_attr_sm_lrf_ecc_single_err_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_lrf_ecc_double_err_count",
				&g->ecc.gr.t18x.sm_lrf_double_err_count,
				&dev_attr_sm_lrf_ecc_double_err_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_shm_ecc_sec_count",
				&g->ecc.gr.t18x.sm_shm_sec_count,
				&dev_attr_sm_shm_ecc_sec_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_shm_ecc_sed_count",
				&g->ecc.gr.t18x.sm_shm_sed_count,
				&dev_attr_sm_shm_ecc_sed_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"sm_shm_ecc_ded_count",
				&g->ecc.gr.t18x.sm_shm_ded_count,
				&dev_attr_sm_shm_ecc_ded_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"tex_ecc_total_sec_pipe0_count",
				&g->ecc.gr.t18x.tex_total_sec_pipe0_count,
				&dev_attr_tex_ecc_total_sec_pipe0_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"tex_ecc_total_ded_pipe0_count",
				&g->ecc.gr.t18x.tex_total_ded_pipe0_count,
				&dev_attr_tex_ecc_total_ded_pipe0_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"tex_ecc_unique_sec_pipe0_count",
				&g->ecc.gr.t18x.tex_unique_sec_pipe0_count,
				&dev_attr_tex_ecc_unique_sec_pipe0_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"tex_ecc_unique_ded_pipe0_count",
				&g->ecc.gr.t18x.tex_unique_ded_pipe0_count,
				&dev_attr_tex_ecc_unique_ded_pipe0_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"tex_ecc_total_sec_pipe1_count",
				&g->ecc.gr.t18x.tex_total_sec_pipe1_count,
				&dev_attr_tex_ecc_total_sec_pipe1_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"tex_ecc_total_ded_pipe1_count",
				&g->ecc.gr.t18x.tex_total_ded_pipe1_count,
				&dev_attr_tex_ecc_total_ded_pipe1_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"tex_ecc_unique_sec_pipe1_count",
				&g->ecc.gr.t18x.tex_unique_sec_pipe1_count,
				&dev_attr_tex_ecc_unique_sec_pipe1_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				0,
				"tex_ecc_unique_ded_pipe1_count",
				&g->ecc.gr.t18x.tex_unique_ded_pipe1_count,
				&dev_attr_tex_ecc_unique_ded_pipe1_count_array);

	error |= gr_gp10b_ecc_stat_create(dev,
				1,
				"lts0_ecc_sec_count",
				&g->ecc.gr.t18x.l2_sec_count,
				&dev_attr_l2_ecc_sec_count_array);
	error |= gr_gp10b_ecc_stat_create(dev,
				1,
				"lts0_ecc_ded_count",
				&g->ecc.gr.t18x.l2_ded_count,
				&dev_attr_l2_ecc_ded_count_array);

	if (error)
		dev_err(dev, "Failed to create sysfs attributes!\n");
}

static void gr_gp10b_remove_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.sm_lrf_single_err_count,
			dev_attr_sm_lrf_ecc_single_err_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.sm_lrf_double_err_count,
			dev_attr_sm_lrf_ecc_double_err_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.sm_shm_sec_count,
			dev_attr_sm_shm_ecc_sec_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.sm_shm_sed_count,
			dev_attr_sm_shm_ecc_sed_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.sm_shm_ded_count,
			dev_attr_sm_shm_ecc_ded_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.tex_total_sec_pipe0_count,
			dev_attr_tex_ecc_total_sec_pipe0_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.tex_total_ded_pipe0_count,
			dev_attr_tex_ecc_total_ded_pipe0_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.tex_unique_sec_pipe0_count,
			dev_attr_tex_ecc_unique_sec_pipe0_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.tex_unique_ded_pipe0_count,
			dev_attr_tex_ecc_unique_ded_pipe0_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.tex_total_sec_pipe1_count,
			dev_attr_tex_ecc_total_sec_pipe1_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.tex_total_ded_pipe1_count,
			dev_attr_tex_ecc_total_ded_pipe1_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.tex_unique_sec_pipe1_count,
			dev_attr_tex_ecc_unique_sec_pipe1_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			0,
			&g->ecc.gr.t18x.tex_unique_ded_pipe1_count,
			dev_attr_tex_ecc_unique_ded_pipe1_count_array);

	gr_gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.gr.t18x.l2_sec_count,
			dev_attr_l2_ecc_sec_count_array);
	gr_gp10b_ecc_stat_remove(dev,
			1,
			&g->ecc.gr.t18x.l2_ded_count,
			dev_attr_l2_ecc_ded_count_array);
}
