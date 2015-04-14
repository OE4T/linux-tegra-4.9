/*
 * drivers/video/tegra/host/gk20a/gk20a_sysfs.c
 *
 * GK20A Graphics
 *
 * Copyright (c) 2011-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/gk20a.h>

#include <mach/clk.h>

#include "gk20a.h"
#include "gr_gk20a.h"
#include "fifo_gk20a.h"
#include "pmu_gk20a.h"


#define PTIMER_FP_FACTOR			1000000
/* PTIMER_REF_FREQ_HZ corresponds to a period of 32 nanoseconds. 32 ns is
   the resolution of ptimer. */
#define PTIMER_REF_FREQ_HZ			31250000

#define ROOTRW (S_IRWXU|S_IRGRP|S_IROTH)

static ssize_t elcg_enable_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	err = gk20a_busy(g->dev);
	if (err)
		return err;

	if (val) {
		g->elcg_enabled = true;
		gr_gk20a_init_elcg_mode(g, ELCG_AUTO, ENGINE_GR_GK20A);
		gr_gk20a_init_elcg_mode(g, ELCG_AUTO, ENGINE_CE2_GK20A);
	} else {
		g->elcg_enabled = false;
		gr_gk20a_init_elcg_mode(g, ELCG_RUN, ENGINE_GR_GK20A);
		gr_gk20a_init_elcg_mode(g, ELCG_RUN, ENGINE_CE2_GK20A);
	}
	gk20a_idle(g->dev);

	dev_info(device, "ELCG is %s.\n", g->elcg_enabled ? "enabled" :
			"disabled");

	return count;
}

static ssize_t elcg_enable_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);

	return sprintf(buf, "%d\n", g->elcg_enabled ? 1 : 0);
}

static DEVICE_ATTR(elcg_enable, ROOTRW, elcg_enable_read, elcg_enable_store);

static ssize_t blcg_enable_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val)
		g->blcg_enabled = true;
	else
		g->blcg_enabled = false;

	err = gk20a_busy(g->dev);
	if (err)
		return err;

	if (g->ops.clock_gating.blcg_bus_load_gating_prod)
		g->ops.clock_gating.blcg_bus_load_gating_prod(g, g->blcg_enabled);
	if (g->ops.clock_gating.blcg_ctxsw_firmware_load_gating_prod)
		g->ops.clock_gating.blcg_ctxsw_firmware_load_gating_prod(g, g->blcg_enabled);
	if (g->ops.clock_gating.blcg_fb_load_gating_prod)
		g->ops.clock_gating.blcg_fb_load_gating_prod(g, g->blcg_enabled);
	if (g->ops.clock_gating.blcg_fifo_load_gating_prod)
		g->ops.clock_gating.blcg_fifo_load_gating_prod(g, g->blcg_enabled);
	g->ops.clock_gating.blcg_gr_load_gating_prod(g, g->blcg_enabled);
	if (g->ops.clock_gating.blcg_ltc_load_gating_prod)
		g->ops.clock_gating.blcg_ltc_load_gating_prod(g, g->blcg_enabled);
	if (g->ops.clock_gating.blcg_pmu_load_gating_prod)
		g->ops.clock_gating.blcg_pmu_load_gating_prod(g, g->blcg_enabled);
	gk20a_idle(g->dev);

	dev_info(device, "BLCG is %s.\n", g->blcg_enabled ? "enabled" :
			"disabled");

	return count;
}

static ssize_t blcg_enable_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);

	return sprintf(buf, "%d\n", g->blcg_enabled ? 1 : 0);
}


static DEVICE_ATTR(blcg_enable, ROOTRW, blcg_enable_read, blcg_enable_store);

static ssize_t slcg_enable_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val)
		g->slcg_enabled = true;
	else
		g->slcg_enabled = false;

	/*
	 * TODO: slcg_therm_load_gating is not enabled anywhere during
	 * init. Therefore, it would be incongruous to add it here. Once
	 * it is added to init, we should add it here too.
	 */
	err = gk20a_busy(g->dev);
	if (err)
		return err;

	if (g->ops.clock_gating.slcg_bus_load_gating_prod)
		g->ops.clock_gating.slcg_bus_load_gating_prod(g, g->slcg_enabled);
	if (g->ops.clock_gating.slcg_ce2_load_gating_prod)
		g->ops.clock_gating.slcg_ce2_load_gating_prod(g, g->slcg_enabled);
	if (g->ops.clock_gating.slcg_chiplet_load_gating_prod)
		g->ops.clock_gating.slcg_chiplet_load_gating_prod(g, g->slcg_enabled);
	if (g->ops.clock_gating.slcg_ctxsw_firmware_load_gating_prod)
		g->ops.clock_gating.slcg_ctxsw_firmware_load_gating_prod(g, g->slcg_enabled);
	if (g->ops.clock_gating.slcg_fb_load_gating_prod)
		g->ops.clock_gating.slcg_fb_load_gating_prod(g, g->slcg_enabled);
	if (g->ops.clock_gating.slcg_fifo_load_gating_prod)
		g->ops.clock_gating.slcg_fifo_load_gating_prod(g, g->slcg_enabled);
	g->ops.clock_gating.slcg_gr_load_gating_prod(g, g->slcg_enabled);
	if (g->ops.clock_gating.slcg_ltc_load_gating_prod)
		g->ops.clock_gating.slcg_ltc_load_gating_prod(g, g->slcg_enabled);
	g->ops.clock_gating.slcg_perf_load_gating_prod(g, g->slcg_enabled);
	if (g->ops.clock_gating.slcg_priring_load_gating_prod)
		g->ops.clock_gating.slcg_priring_load_gating_prod(g, g->slcg_enabled);
	if (g->ops.clock_gating.slcg_pmu_load_gating_prod)
		g->ops.clock_gating.slcg_pmu_load_gating_prod(g, g->slcg_enabled);
	if (g->ops.clock_gating.slcg_xbar_load_gating_prod)
		g->ops.clock_gating.slcg_xbar_load_gating_prod(g, g->slcg_enabled);
	gk20a_idle(g->dev);

	dev_info(device, "SLCG is %s.\n", g->slcg_enabled ? "enabled" :
			"disabled");

	return count;
}

static ssize_t slcg_enable_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);

	return sprintf(buf, "%d\n", g->slcg_enabled ? 1 : 0);
}

static DEVICE_ATTR(slcg_enable, ROOTRW, slcg_enable_read, slcg_enable_store);

static ssize_t ptimer_scale_factor_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	u32 tsc_freq_hz = clk_get_rate(clk_get_sys(NULL, "clk_m"));
	u32 scaling_factor_fp = (u32)(PTIMER_REF_FREQ_HZ) /
				((u32)(tsc_freq_hz) /
				(u32)(PTIMER_FP_FACTOR));
	ssize_t res = snprintf(buf,
				PAGE_SIZE,
				"%u.%u\n",
				scaling_factor_fp / PTIMER_FP_FACTOR,
				scaling_factor_fp % PTIMER_FP_FACTOR);

	return res;
}

static DEVICE_ATTR(ptimer_scale_factor,
			S_IRUGO,
			ptimer_scale_factor_show,
			NULL);

static ssize_t railgate_delay_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct platform_device *ndev = to_platform_device(dev);
	int railgate_delay = 0, ret = 0;
	struct gk20a *g = get_gk20a(ndev);
	int err;

	if (!platform->can_railgate) {
		dev_info(dev, "does not support power-gating\n");
		return count;
	}

	ret = sscanf(buf, "%d", &railgate_delay);
	if (ret == 1 && railgate_delay >= 0) {
		struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
		platform->railgate_delay = railgate_delay;
		pm_genpd_set_poweroff_delay(genpd, platform->railgate_delay);
	} else
		dev_err(dev, "Invalid powergate delay\n");
	/* wake-up system to make rail-gating delay effective immediately */
	err = gk20a_busy(g->dev);
	if (err)
		return err;
	gk20a_idle(g->dev);

	return count;
}
static ssize_t railgate_delay_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", platform->railgate_delay);
}
static DEVICE_ATTR(railgate_delay, ROOTRW, railgate_delay_show,
		   railgate_delay_store);

static ssize_t clockgate_delay_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int clockgate_delay = 0, ret = 0;

	ret = sscanf(buf, "%d", &clockgate_delay);
	if (ret == 1 && clockgate_delay >= 0) {
		platform->clockgate_delay = clockgate_delay;
		pm_runtime_set_autosuspend_delay(dev,
						 platform->clockgate_delay);
	} else
		dev_err(dev, "Invalid clockgate delay\n");

	return count;
}
static ssize_t clockgate_delay_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", platform->clockgate_delay);
}
static DEVICE_ATTR(clockgate_delay, ROOTRW, clockgate_delay_show,
		   clockgate_delay_store);

static ssize_t counters_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gk20a *g = get_gk20a(pdev);
	u32 busy_cycles, total_cycles;
	ssize_t res;

	gk20a_pmu_get_load_counters(g, &busy_cycles, &total_cycles);

	res = snprintf(buf, PAGE_SIZE, "%u %u\n", busy_cycles, total_cycles);

	return res;
}
static DEVICE_ATTR(counters, S_IRUGO, counters_show, NULL);

static ssize_t counters_show_reset(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	ssize_t res = counters_show(dev, attr, buf);
	struct platform_device *pdev = to_platform_device(dev);
	struct gk20a *g = get_gk20a(pdev);

	gk20a_pmu_reset_load_counters(g);

	return res;
}
static DEVICE_ATTR(counters_reset, S_IRUGO, counters_show_reset, NULL);

static ssize_t gk20a_load_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gk20a *g = get_gk20a(pdev);
	u32 busy_time;
	ssize_t res;
	int err;

	if (!g->power_on) {
		busy_time = 0;
	} else {
		err = gk20a_busy(g->dev);
		if (err)
			return err;

		gk20a_pmu_load_update(g);
		gk20a_pmu_load_norm(g, &busy_time);
		gk20a_idle(g->dev);
	}

	res = snprintf(buf, PAGE_SIZE, "%u\n", busy_time);

	return res;
}
static DEVICE_ATTR(load, S_IRUGO, gk20a_load_show, NULL);

static ssize_t elpg_enable_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	/*
	 * Since elpg is refcounted, we should not unnecessarily call
	 * enable/disable if it is already so.
	 */
	err = gk20a_busy(g->dev);
	if (err)
		return -EAGAIN;

	if (val && !g->elpg_enabled) {
		g->elpg_enabled = true;
		gk20a_pmu_enable_elpg(g);
	} else if (!val && g->elpg_enabled) {
		g->elpg_enabled = false;
		gk20a_pmu_disable_elpg(g);
	}
	gk20a_idle(g->dev);

	dev_info(device, "ELPG is %s.\n", g->elpg_enabled ? "enabled" :
			"disabled");

	return count;
}

static ssize_t elpg_enable_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);

	return sprintf(buf, "%d\n", g->elpg_enabled ? 1 : 0);
}

static DEVICE_ATTR(elpg_enable, ROOTRW, elpg_enable_read, elpg_enable_store);

static ssize_t aelpg_param_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	int status = 0;
	union pmu_ap_cmd ap_cmd;
	int *paramlist = (int *)g->pmu.aelpg_param;
	u32 defaultparam[5] = {
			APCTRL_SAMPLING_PERIOD_PG_DEFAULT_US,
			APCTRL_MINIMUM_IDLE_FILTER_DEFAULT_US,
			APCTRL_MINIMUM_TARGET_SAVING_DEFAULT_US,
			APCTRL_POWER_BREAKEVEN_DEFAULT_US,
			APCTRL_CYCLES_PER_SAMPLE_MAX_DEFAULT
	};

	/* Get each parameter value from input string*/
	sscanf(buf, "%d %d %d %d %d", &paramlist[0], &paramlist[1],
				&paramlist[2], &paramlist[3], &paramlist[4]);

	/* If parameter value is 0 then reset to SW default values*/
	if ((paramlist[0] | paramlist[1] | paramlist[2]
		| paramlist[3] | paramlist[4]) == 0x00) {
		memcpy(paramlist, defaultparam, sizeof(defaultparam));
	}

	/* If aelpg is enabled & pmu is ready then post values to
	 * PMU else store then post later
	 */
	if (g->aelpg_enabled && g->pmu.pmu_ready) {
		/* Disable AELPG */
		ap_cmd.init.cmd_id = PMU_AP_CMD_ID_DISABLE_CTRL;
		status = gk20a_pmu_ap_send_command(g, &ap_cmd, false);

		/* Enable AELPG */
		gk20a_aelpg_init(g);
		gk20a_aelpg_init_and_enable(g, PMU_AP_CTRL_ID_GRAPHICS);
	}

	return count;
}

static ssize_t aelpg_param_read(struct device *device,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);

	return sprintf(buf, "%d %d %d %d %d\n", g->pmu.aelpg_param[0],
		g->pmu.aelpg_param[1], g->pmu.aelpg_param[2],
		g->pmu.aelpg_param[3], g->pmu.aelpg_param[4]);
}

static DEVICE_ATTR(aelpg_param, ROOTRW,
		aelpg_param_read, aelpg_param_store);

static ssize_t aelpg_enable_store(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	unsigned long val = 0;
	int status = 0;
	union pmu_ap_cmd ap_cmd;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	err = gk20a_busy(g->dev);
	if (err)
		return err;

	if (g->pmu.pmu_ready) {
		if (val && !g->aelpg_enabled) {
			g->aelpg_enabled = true;
			/* Enable AELPG */
			ap_cmd.init.cmd_id = PMU_AP_CMD_ID_ENABLE_CTRL;
			status = gk20a_pmu_ap_send_command(g, &ap_cmd, false);
		} else if (!val && g->aelpg_enabled) {
			g->aelpg_enabled = false;
			/* Disable AELPG */
			ap_cmd.init.cmd_id = PMU_AP_CMD_ID_DISABLE_CTRL;
			status = gk20a_pmu_ap_send_command(g, &ap_cmd, false);
		}
	} else {
		dev_info(device, "PMU is not ready, AELPG request failed\n");
	}
	gk20a_idle(g->dev);

	dev_info(device, "AELPG is %s.\n", g->aelpg_enabled ? "enabled" :
			"disabled");

	return count;
}

static ssize_t aelpg_enable_read(struct device *device,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);

	return sprintf(buf, "%d\n", g->aelpg_enabled ? 1 : 0);
}

static DEVICE_ATTR(aelpg_enable, ROOTRW,
		aelpg_enable_read, aelpg_enable_store);


static ssize_t allow_all_enable_read(struct device *device,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	return sprintf(buf, "%d\n", g->allow_all ? 1 : 0);
}

static ssize_t allow_all_enable_store(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	err = gk20a_busy(g->dev);
	g->allow_all = (val ? true : false);
	gk20a_idle(g->dev);

	return count;
}

static DEVICE_ATTR(allow_all, ROOTRW,
		allow_all_enable_read, allow_all_enable_store);

static ssize_t emc3d_ratio_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	g->emc3d_ratio = val;

	return count;
}

static ssize_t emc3d_ratio_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);

	return sprintf(buf, "%d\n", g->emc3d_ratio);
}

static DEVICE_ATTR(emc3d_ratio, ROOTRW, emc3d_ratio_read, emc3d_ratio_store);

#ifdef CONFIG_PM_RUNTIME
static ssize_t force_idle_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	unsigned long val = 0;
	int err = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val) {
		if (g->forced_idle)
			return count; /* do nothing */
		else {
			err = __gk20a_do_idle(ndev, false);
			if (!err) {
				g->forced_idle = 1;
				dev_info(device, "gpu is idle : %d\n",
					g->forced_idle);
			}
		}
	} else {
		if (!g->forced_idle)
			return count; /* do nothing */
		else {
			err = __gk20a_do_unidle(ndev);
			if (!err) {
				g->forced_idle = 0;
				dev_info(device, "gpu is idle : %d\n",
					g->forced_idle);
			}
		}
	}

	return count;
}

static ssize_t force_idle_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);

	return sprintf(buf, "%d\n", g->forced_idle ? 1 : 0);
}

static DEVICE_ATTR(force_idle, ROOTRW, force_idle_read, force_idle_store);
#endif

static ssize_t tpc_fs_mask_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (!g->gr.gpc_tpc_mask)
		return -ENODEV;

	if (val && val != g->gr.gpc_tpc_mask[0] && g->ops.gr.set_gpc_tpc_mask) {
		g->gr.gpc_tpc_mask[0] = val;

		g->ops.gr.set_gpc_tpc_mask(g, 0);

		kfree(g->gr.ctx_vars.local_golden_image);
		g->gr.ctx_vars.local_golden_image = NULL;
		g->gr.ctx_vars.golden_image_initialized = false;
		g->gr.ctx_vars.golden_image_size = 0;
		g->gr.sw_ready = false;
	}

	return count;
}

static ssize_t tpc_fs_mask_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct gk20a *g = get_gk20a(ndev);
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index;
	u32 tpc_fs_mask = 0;
	int err = 0;

	err = gk20a_busy(g->dev);
	if (err)
		return err;

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		if (g->ops.gr.get_gpc_tpc_mask)
			tpc_fs_mask |=
				g->ops.gr.get_gpc_tpc_mask(g, gpc_index) <<
				(gr->max_tpc_per_gpc_count * gpc_index);
	}

	gk20a_idle(g->dev);

	return sprintf(buf, "0x%x\n", tpc_fs_mask);
}

static DEVICE_ATTR(tpc_fs_mask, ROOTRW, tpc_fs_mask_read, tpc_fs_mask_store);

void gk20a_remove_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(to_platform_device(dev));

	device_remove_file(dev, &dev_attr_elcg_enable);
	device_remove_file(dev, &dev_attr_blcg_enable);
	device_remove_file(dev, &dev_attr_slcg_enable);
	device_remove_file(dev, &dev_attr_ptimer_scale_factor);
	device_remove_file(dev, &dev_attr_elpg_enable);
	device_remove_file(dev, &dev_attr_emc3d_ratio);
	device_remove_file(dev, &dev_attr_counters);
	device_remove_file(dev, &dev_attr_counters_reset);
	device_remove_file(dev, &dev_attr_load);
	device_remove_file(dev, &dev_attr_railgate_delay);
	device_remove_file(dev, &dev_attr_clockgate_delay);
#ifdef CONFIG_PM_RUNTIME
	device_remove_file(dev, &dev_attr_force_idle);
#endif
	device_remove_file(dev, &dev_attr_aelpg_param);
	device_remove_file(dev, &dev_attr_aelpg_enable);
	device_remove_file(dev, &dev_attr_allow_all);
	device_remove_file(dev, &dev_attr_tpc_fs_mask);

	if (g->host1x_dev && (dev->parent != &g->host1x_dev->dev))
		sysfs_remove_link(&g->host1x_dev->dev.kobj, dev_name(dev));
}

void gk20a_create_sysfs(struct platform_device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int error = 0;

	error |= device_create_file(&dev->dev, &dev_attr_elcg_enable);
	error |= device_create_file(&dev->dev, &dev_attr_blcg_enable);
	error |= device_create_file(&dev->dev, &dev_attr_slcg_enable);
	error |= device_create_file(&dev->dev, &dev_attr_ptimer_scale_factor);
	error |= device_create_file(&dev->dev, &dev_attr_elpg_enable);
	error |= device_create_file(&dev->dev, &dev_attr_emc3d_ratio);
	error |= device_create_file(&dev->dev, &dev_attr_counters);
	error |= device_create_file(&dev->dev, &dev_attr_counters_reset);
	error |= device_create_file(&dev->dev, &dev_attr_load);
	error |= device_create_file(&dev->dev, &dev_attr_railgate_delay);
	error |= device_create_file(&dev->dev, &dev_attr_clockgate_delay);
#ifdef CONFIG_PM_RUNTIME
	error |= device_create_file(&dev->dev, &dev_attr_force_idle);
#endif
	error |= device_create_file(&dev->dev, &dev_attr_aelpg_param);
	error |= device_create_file(&dev->dev, &dev_attr_aelpg_enable);
	error |= device_create_file(&dev->dev, &dev_attr_allow_all);
	error |= device_create_file(&dev->dev, &dev_attr_tpc_fs_mask);

	if (g->host1x_dev && (dev->dev.parent != &g->host1x_dev->dev))
		error |= sysfs_create_link(&g->host1x_dev->dev.kobj,
					   &dev->dev.kobj,
					   dev_name(&dev->dev));

	if (error)
		dev_err(&dev->dev, "Failed to create sysfs attributes!\n");

}
