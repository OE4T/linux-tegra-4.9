/*
 * GK20A Graphics
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/platform/tegra/common.h>

#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/soc.h>
#include <nvgpu/enabled.h>
#include <nvgpu/debug.h>

#include "gk20a/platform_gk20a.h"
#include "sysfs.h"
#include "vgpu/vgpu.h"
#include "gk20a/gk20a_scale.h"
#include "gk20a/ctxsw_trace_gk20a.h"
#include "pci.h"
#include "module.h"
#include "intr.h"
#ifdef CONFIG_TEGRA_19x_GPU
#include "nvgpu_gpuid_t19x.h"
#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
#include "vgpu/vgpu_t19x.h"
#endif
#endif
#include "os_linux.h"

#define CLASS_NAME "nvidia-gpu"
/* TODO: Change to e.g. "nvidia-gpu%s" once we have symlinks in place. */

#define GK20A_WAIT_FOR_IDLE_MS	2000

#define CREATE_TRACE_POINTS
#include <trace/events/gk20a.h>

void gk20a_busy_noresume(struct gk20a *g)
{
	pm_runtime_get_noresume(dev_from_gk20a(g));
}

int gk20a_busy(struct gk20a *g)
{
	int ret = 0;
	struct device *dev;

	if (!g)
		return -ENODEV;

	atomic_inc(&g->usage_count);

	down_read(&g->busy_lock);

	if (!gk20a_can_busy(g)) {
		ret = -ENODEV;
		atomic_dec(&g->usage_count);
		goto fail;
	}

	dev = dev_from_gk20a(g);

	if (pm_runtime_enabled(dev)) {
		/* Increment usage count and attempt to resume device */
		ret = pm_runtime_get_sync(dev);
		if (ret < 0) {
			/* Mark suspended so runtime pm will retry later */
			pm_runtime_set_suspended(dev);
			pm_runtime_put_noidle(dev);
			atomic_dec(&g->usage_count);
			goto fail;
		}
	} else {
		nvgpu_mutex_acquire(&g->poweron_lock);
		if (!g->power_on) {
			ret = gk20a_gpu_is_virtual(dev) ?
				vgpu_pm_finalize_poweron(dev)
				: gk20a_pm_finalize_poweron(dev);
			if (ret) {
				atomic_dec(&g->usage_count);
				nvgpu_mutex_release(&g->poweron_lock);
				goto fail;
			}
		}
		nvgpu_mutex_release(&g->poweron_lock);
	}

fail:
	up_read(&g->busy_lock);

	return ret < 0 ? ret : 0;
}

void gk20a_idle_nosuspend(struct gk20a *g)
{
	pm_runtime_put_noidle(dev_from_gk20a(g));
}

void gk20a_idle(struct gk20a *g)
{
	struct device *dev;

	atomic_dec(&g->usage_count);

	dev = dev_from_gk20a(g);

	if (!(dev && gk20a_can_busy(g)))
		return;

	if (pm_runtime_enabled(dev)) {
		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_sync_autosuspend(dev);
	}
}

int gk20a_pm_finalize_poweron(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	int err, nice_value;

	gk20a_dbg_fn("");

	if (g->power_on)
		return 0;

	trace_gk20a_finalize_poweron(dev_name(dev));

	/* Increment platform power refcount */
	if (platform->busy) {
		err = platform->busy(dev);
		if (err < 0) {
			nvgpu_err(g, "failed to poweron platform dependency");
			return err;
		}
	}

	err = gk20a_restore_registers(g);
	if (err)
		return err;

	nice_value = task_nice(current);
	set_user_nice(current, -20);

	/* Enable interrupt workqueue */
	if (!g->nonstall_work_queue) {
		g->nonstall_work_queue = alloc_workqueue("%s",
						WQ_HIGHPRI, 1, "mc_nonstall");
		INIT_WORK(&g->nonstall_fn_work, nvgpu_intr_nonstall_cb);
	}

	err = gk20a_finalize_poweron(g);
	set_user_nice(current, nice_value);
	if (err)
		goto done;

	trace_gk20a_finalize_poweron_done(dev_name(dev));

	enable_irq(g->irq_stall);
	if (g->irq_stall != g->irq_nonstall)
		enable_irq(g->irq_nonstall);
	g->irqs_enabled = 1;

	gk20a_scale_resume(dev_from_gk20a(g));

	if (platform->has_cde)
		gk20a_init_cde_support(g);

done:
	if (err)
		g->power_on = false;

	return err;
}

static int gk20a_pm_prepare_poweroff(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int ret = 0;
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	gk20a_dbg_fn("");

	nvgpu_mutex_acquire(&g->poweroff_lock);

	if (!g->power_on)
		goto done;

	gk20a_scale_suspend(dev);

	ret = gk20a_prepare_poweroff(g);
	if (ret)
		goto error;

	/*
	 * After this point, gk20a interrupts should not get
	 * serviced.
	 */
	disable_irq(g->irq_stall);
	if (g->irq_stall != g->irq_nonstall)
		disable_irq(g->irq_nonstall);

	/* Decrement platform power refcount */
	if (platform->idle)
		platform->idle(dev);

	/* Stop CPU from accessing the GPU registers. */
	gk20a_lockout_registers(g);

	nvgpu_mutex_release(&g->poweroff_lock);
	return 0;

error:
	gk20a_scale_resume(dev);
done:
	nvgpu_mutex_release(&g->poweroff_lock);

	return ret;
}

static struct of_device_id tegra_gk20a_of_match[] = {
#ifdef CONFIG_TEGRA_GK20A
	{ .compatible = "nvidia,tegra210-gm20b",
		.data = &gm20b_tegra_platform },
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = "nvidia,tegra186-gp10b",
		.data = &gp10b_tegra_platform },
#endif
#ifdef CONFIG_TEGRA_19x_GPU
	{ .compatible = TEGRA_19x_GPU_COMPAT_TEGRA,
		.data = &t19x_gpu_tegra_platform },
#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	{ .compatible = TEGRA_19x_VGPU_COMPAT_TEGRA,
		.data = &t19x_vgpu_tegra_platform },
#endif
#endif
#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	{ .compatible = "nvidia,tegra124-gk20a-vgpu",
		.data = &vgpu_tegra_platform },
#endif
#endif

	{ },
};

#ifdef CONFIG_PM
/**
 * __gk20a_do_idle() - force the GPU to idle and railgate
 *
 * In success, this call MUST be balanced by caller with __gk20a_do_unidle()
 *
 * Acquires two locks : &g->busy_lock and &platform->railgate_lock
 * In success, we hold these locks and return
 * In failure, we release these locks and return
 */
int __gk20a_do_idle(struct gk20a *g, bool force_reset)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct nvgpu_timeout timeout;
	int ref_cnt;
	int target_ref_cnt = 0;
	bool is_railgated;
	int err = 0;

	/*
	 * Hold back deterministic submits and changes to deterministic
	 * channels - this must be outside the power busy locks.
	 */
	gk20a_channel_deterministic_idle(g);

	/* acquire busy lock to block other busy() calls */
	down_write(&g->busy_lock);

	/* acquire railgate lock to prevent unrailgate in midst of do_idle() */
	nvgpu_mutex_acquire(&platform->railgate_lock);

	/* check if it is already railgated ? */
	if (platform->is_railgated(dev))
		return 0;

	/*
	 * release railgate_lock, prevent suspend by incrementing usage counter,
	 * re-acquire railgate_lock
	 */
	nvgpu_mutex_release(&platform->railgate_lock);
	pm_runtime_get_sync(dev);

	/*
	 * One refcount taken in this API
	 * If User disables rail gating, we take one more
	 * extra refcount
	 */
	if (g->user_railgate_disabled)
		target_ref_cnt = 2;
	else
		target_ref_cnt = 1;
	nvgpu_mutex_acquire(&platform->railgate_lock);

	nvgpu_timeout_init(g, &timeout, GK20A_WAIT_FOR_IDLE_MS,
			   NVGPU_TIMER_CPU_TIMER);

	/* check and wait until GPU is idle (with a timeout) */
	do {
		nvgpu_usleep_range(1000, 1100);
		ref_cnt = atomic_read(&dev->power.usage_count);
	} while (ref_cnt != target_ref_cnt && !nvgpu_timeout_expired(&timeout));

	if (ref_cnt != target_ref_cnt) {
		nvgpu_err(g, "failed to idle - refcount %d != target_ref_cnt",
			ref_cnt);
		goto fail_drop_usage_count;
	}

	/* check if global force_reset flag is set */
	force_reset |= platform->force_reset_in_do_idle;

	nvgpu_timeout_init(g, &timeout, GK20A_WAIT_FOR_IDLE_MS,
			   NVGPU_TIMER_CPU_TIMER);

	if (g->can_railgate && !force_reset) {
		/*
		 * Case 1 : GPU railgate is supported
		 *
		 * if GPU is now idle, we will have only one ref count,
		 * drop this ref which will rail gate the GPU
		 */
		pm_runtime_put_sync(dev);

		/* add sufficient delay to allow GPU to rail gate */
		nvgpu_msleep(g->railgate_delay);

		/* check in loop if GPU is railgated or not */
		do {
			nvgpu_usleep_range(1000, 1100);
			is_railgated = platform->is_railgated(dev);
		} while (!is_railgated && !nvgpu_timeout_expired(&timeout));

		if (is_railgated) {
			return 0;
		} else {
			nvgpu_err(g, "failed to idle in timeout");
			goto fail_timeout;
		}
	} else {
		/*
		 * Case 2 : GPU railgate is not supported or we explicitly
		 * do not want to depend on runtime PM
		 *
		 * if GPU is now idle, call prepare_poweroff() to save the
		 * state and then do explicit railgate
		 *
		 * __gk20a_do_unidle() needs to unrailgate, call
		 * finalize_poweron(), and then call pm_runtime_put_sync()
		 * to balance the GPU usage counter
		 */

		/* Save the GPU state */
		err = gk20a_pm_prepare_poweroff(dev);
		if (err)
			goto fail_drop_usage_count;

		/* railgate GPU */
		platform->railgate(dev);

		nvgpu_udelay(10);

		g->forced_reset = true;
		return 0;
	}

fail_drop_usage_count:
	pm_runtime_put_noidle(dev);
fail_timeout:
	nvgpu_mutex_release(&platform->railgate_lock);
	up_write(&g->busy_lock);
	gk20a_channel_deterministic_unidle(g);
	return -EBUSY;
}

/**
 * gk20a_do_idle() - wrap up for __gk20a_do_idle() to be called
 * from outside of GPU driver
 *
 * In success, this call MUST be balanced by caller with gk20a_do_unidle()
 */
static int gk20a_do_idle(void *_g)
{
	struct gk20a *g = (struct gk20a *)_g;

	return __gk20a_do_idle(g, true);
}

/**
 * __gk20a_do_unidle() - unblock all the tasks blocked by __gk20a_do_idle()
 */
int __gk20a_do_unidle(struct gk20a *g)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int err;

	if (g->forced_reset) {
		/*
		 * If we did a forced-reset/railgate
		 * then unrailgate the GPU here first
		 */
		platform->unrailgate(dev);

		/* restore the GPU state */
		err = gk20a_pm_finalize_poweron(dev);
		if (err)
			return err;

		/* balance GPU usage counter */
		pm_runtime_put_sync(dev);

		g->forced_reset = false;
	}

	/* release the lock and open up all other busy() calls */
	nvgpu_mutex_release(&platform->railgate_lock);
	up_write(&g->busy_lock);

	gk20a_channel_deterministic_unidle(g);

	return 0;
}

/**
 * gk20a_do_unidle() - wrap up for __gk20a_do_unidle()
 */
static int gk20a_do_unidle(void *_g)
{
	struct gk20a *g = (struct gk20a *)_g;

	return __gk20a_do_unidle(g);
}
#endif

static void __iomem *gk20a_ioremap_resource(struct platform_device *dev, int i,
					    struct resource **out)
{
	struct resource *r = platform_get_resource(dev, IORESOURCE_MEM, i);

	if (!r)
		return NULL;
	if (out)
		*out = r;
	return devm_ioremap_resource(&dev->dev, r);
}

static irqreturn_t gk20a_intr_isr_stall(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;

	return nvgpu_intr_stall(g);
}

static irqreturn_t gk20a_intr_isr_nonstall(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;

	return nvgpu_intr_nonstall(g);
}

static irqreturn_t gk20a_intr_thread_stall(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;

	return nvgpu_intr_thread_stall(g);
}

void gk20a_remove_support(struct gk20a *g)
{
	tegra_unregister_idle_unidle(gk20a_do_idle);

	nvgpu_kfree(g, g->dbg_regops_tmp_buf);

	if (g->pmu.remove_support)
		g->pmu.remove_support(&g->pmu);

	if (g->gr.remove_support)
		g->gr.remove_support(&g->gr);

	if (g->mm.remove_ce_support)
		g->mm.remove_ce_support(&g->mm);

	if (g->fifo.remove_support)
		g->fifo.remove_support(&g->fifo);

	if (g->mm.remove_support)
		g->mm.remove_support(&g->mm);

	if (g->sim.remove_support)
		g->sim.remove_support(&g->sim);

	/* free mappings to registers, etc */

	if (g->regs) {
		iounmap(g->regs);
		g->regs = NULL;
	}
	if (g->bar1) {
		iounmap(g->bar1);
		g->bar1 = NULL;
	}
}

static int gk20a_init_support(struct platform_device *dev)
{
	int err = 0;
	struct gk20a *g = get_gk20a(&dev->dev);

	tegra_register_idle_unidle(gk20a_do_idle, gk20a_do_unidle, g);

	g->regs = gk20a_ioremap_resource(dev, GK20A_BAR0_IORESOURCE_MEM,
					 &g->reg_mem);
	if (IS_ERR(g->regs)) {
		nvgpu_err(g, "failed to remap gk20a registers");
		err = PTR_ERR(g->regs);
		goto fail;
	}

	g->bar1 = gk20a_ioremap_resource(dev, GK20A_BAR1_IORESOURCE_MEM,
					 &g->bar1_mem);
	if (IS_ERR(g->bar1)) {
		nvgpu_err(g, "failed to remap gk20a bar1");
		err = PTR_ERR(g->bar1);
		goto fail;
	}

	if (nvgpu_platform_is_simulation(g)) {
		g->sim.g = g;
		g->sim.regs = gk20a_ioremap_resource(dev,
						     GK20A_SIM_IORESOURCE_MEM,
						     &g->sim.reg_mem);
		if (IS_ERR(g->sim.regs)) {
			nvgpu_err(g, "failed to remap gk20a sim regs");
			err = PTR_ERR(g->sim.regs);
			goto fail;
		}

		err = gk20a_init_sim_support(dev);
		if (err)
			goto fail;
	}

	return 0;

fail:
	return err;
}

static int gk20a_pm_railgate(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int ret = 0;
#ifdef CONFIG_DEBUG_FS
	struct gk20a *g = get_gk20a(dev);

	g->pstats.last_rail_gate_start = jiffies;

	if (g->pstats.railgating_cycle_count >= 1)
		g->pstats.total_rail_ungate_time_ms =
			g->pstats.total_rail_ungate_time_ms +
			jiffies_to_msecs(g->pstats.last_rail_gate_start -
					g->pstats.last_rail_ungate_complete);
#endif

	if (platform->railgate)
		ret = platform->railgate(dev);

#ifdef CONFIG_DEBUG_FS
	g->pstats.last_rail_gate_complete = jiffies;
#endif

	return ret;
}

static int gk20a_pm_unrailgate(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int ret = 0;
#ifdef CONFIG_DEBUG_FS
	struct gk20a *g = get_gk20a(dev);

	g->pstats.last_rail_ungate_start = jiffies;
	if (g->pstats.railgating_cycle_count >= 1)
		g->pstats.total_rail_gate_time_ms =
			g->pstats.total_rail_gate_time_ms +
			jiffies_to_msecs(g->pstats.last_rail_ungate_start -
				g->pstats.last_rail_gate_complete);

	g->pstats.railgating_cycle_count++;
#endif

	trace_gk20a_pm_unrailgate(dev_name(dev));

	if (platform->unrailgate) {
		nvgpu_mutex_acquire(&platform->railgate_lock);
		ret = platform->unrailgate(dev);
		nvgpu_mutex_release(&platform->railgate_lock);
	}

#ifdef CONFIG_DEBUG_FS
	g->pstats.last_rail_ungate_complete = jiffies;
#endif

	return ret;
}

static void gk20a_pm_shutdown(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct gk20a *g = platform->g;
	int err;

	nvgpu_info(g, "shutting down");

	/* vgpu has nothing to clean up currently */
	if (gk20a_gpu_is_virtual(&pdev->dev))
		return;

	if (!g->power_on)
		goto finish;

	gk20a_driver_start_unload(g);

	/* If GPU is already railgated,
	 * just prevent more requests, and return */
	if (platform->is_railgated && platform->is_railgated(&pdev->dev)) {
		__pm_runtime_disable(&pdev->dev, false);
		nvgpu_info(g, "already railgated, shut down complete");
		return;
	}

	/* Prevent more requests by disabling Runtime PM */
	__pm_runtime_disable(&pdev->dev, false);

	err = gk20a_wait_for_idle(&pdev->dev);
	if (err) {
		nvgpu_err(g, "failed to idle GPU, err=%d", err);
		goto finish;
	}

	err = gk20a_fifo_disable_all_engine_activity(g, true);
	if (err) {
		nvgpu_err(g, "failed to disable engine activity, err=%d",
			err);
		goto finish;
	}

	err = gk20a_fifo_wait_engine_idle(g);
	if (err) {
		nvgpu_err(g, "failed to idle engines, err=%d",
			err);
		goto finish;
	}

	if (gk20a_gpu_is_virtual(&pdev->dev))
		err = vgpu_pm_prepare_poweroff(&pdev->dev);
	else
		err = gk20a_pm_prepare_poweroff(&pdev->dev);
	if (err) {
		nvgpu_err(g, "failed to prepare for poweroff, err=%d",
			err);
		goto finish;
	}

	err = gk20a_pm_railgate(&pdev->dev);
	if (err)
		nvgpu_err(g, "failed to railgate, err=%d", err);

finish:
	nvgpu_info(g, "shut down complete");
}

#ifdef CONFIG_PM
static int gk20a_pm_runtime_resume(struct device *dev)
{
	int err = 0;

	err = gk20a_pm_unrailgate(dev);
	if (err)
		goto fail;

	err = gk20a_pm_finalize_poweron(dev);
	if (err)
		goto fail_poweron;

	return 0;

fail_poweron:
	gk20a_pm_railgate(dev);
fail:
	return err;
}

static int gk20a_pm_runtime_suspend(struct device *dev)
{
	int err = 0;

	err = gk20a_pm_prepare_poweroff(dev);
	if (err)
		goto fail;

	err = gk20a_pm_railgate(dev);
	if (err)
		goto fail_railgate;

	return 0;

fail_railgate:
	gk20a_pm_finalize_poweron(dev);
fail:
	pm_runtime_mark_last_busy(dev);
	return err;
}

static int gk20a_pm_suspend(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a *g = get_gk20a(dev);
	int ret = 0;

	if (g->user_railgate_disabled)
		gk20a_idle_nosuspend(g);

	if (atomic_read(&dev->power.usage_count) > 1) {
		ret = -EBUSY;
		goto fail;
	}

	if (!g->power_on)
		return 0;

	ret = gk20a_pm_runtime_suspend(dev);
	if (ret)
		goto fail;

	if (platform->suspend)
		platform->suspend(dev);

	g->suspended = true;

	return 0;

fail:
	if (g->user_railgate_disabled)
		gk20a_busy_noresume(g);

	return ret;
}

static int gk20a_pm_resume(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int ret = 0;

	if (g->user_railgate_disabled)
		gk20a_busy_noresume(g);

	if (!g->suspended)
		return 0;

	ret = gk20a_pm_runtime_resume(dev);

	g->suspended = false;

	return ret;
}

static const struct dev_pm_ops gk20a_pm_ops = {
	.runtime_resume = gk20a_pm_runtime_resume,
	.runtime_suspend = gk20a_pm_runtime_suspend,
	.resume = gk20a_pm_resume,
	.suspend = gk20a_pm_suspend,
};
#endif

static int gk20a_pm_init(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int err = 0;

	gk20a_dbg_fn("");

	/* Initialise pm runtime */
	if (g->railgate_delay) {
		pm_runtime_set_autosuspend_delay(dev,
				 g->railgate_delay);
		pm_runtime_use_autosuspend(dev);
	}

	if (g->can_railgate) {
		pm_runtime_enable(dev);
		if (!pm_runtime_enabled(dev))
			gk20a_pm_unrailgate(dev);
		else
			gk20a_pm_railgate(dev);
	} else {
		__pm_runtime_disable(dev, false);
		gk20a_pm_unrailgate(dev);
	}

	return err;
}

static inline void set_gk20a(struct platform_device *pdev, struct gk20a *gk20a)
{
	gk20a_get_platform(&pdev->dev)->g = gk20a;
}

static int gk20a_probe(struct platform_device *dev)
{
	struct nvgpu_os_linux *l;
	struct gk20a *gk20a;
	int err;
	struct gk20a_platform *platform = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_gk20a_of_match, &dev->dev);
		if (match)
			platform = (struct gk20a_platform *)match->data;
	} else
		platform = (struct gk20a_platform *)dev->dev.platform_data;

	if (!platform) {
		dev_err(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	gk20a_dbg_fn("");

	platform_set_drvdata(dev, platform);

	if (gk20a_gpu_is_virtual(&dev->dev))
		return vgpu_probe(dev);

	l = kzalloc(sizeof(*l), GFP_KERNEL);
	if (!l) {
		dev_err(&dev->dev, "couldn't allocate gk20a support");
		return -ENOMEM;
	}

	gk20a = &l->g;
	set_gk20a(dev, gk20a);
	l->dev = &dev->dev;
	gk20a->log_mask = NVGPU_DEFAULT_DBG_MASK;

	nvgpu_kmem_init(gk20a);

	err = nvgpu_init_enabled_flags(gk20a);
	if (err)
		return err;

	if (nvgpu_platform_is_simulation(gk20a))
		__nvgpu_set_enabled(gk20a, NVGPU_IS_FMODEL, true);

	gk20a->irq_stall = platform_get_irq(dev, 0);
	gk20a->irq_nonstall = platform_get_irq(dev, 1);
	if (gk20a->irq_stall < 0 || gk20a->irq_nonstall < 0)
		return -ENXIO;

	err = devm_request_threaded_irq(&dev->dev,
			gk20a->irq_stall,
			gk20a_intr_isr_stall,
			gk20a_intr_thread_stall,
			0, "gk20a_stall", gk20a);
	if (err) {
		dev_err(&dev->dev,
			"failed to request stall intr irq @ %d\n",
				gk20a->irq_stall);
		return err;
	}
	err = devm_request_irq(&dev->dev,
			gk20a->irq_nonstall,
			gk20a_intr_isr_nonstall,
			0, "gk20a_nonstall", gk20a);
	if (err) {
		dev_err(&dev->dev,
			"failed to request non-stall intr irq @ %d\n",
				gk20a->irq_nonstall);
		return err;
	}
	disable_irq(gk20a->irq_stall);
	if (gk20a->irq_stall != gk20a->irq_nonstall)
		disable_irq(gk20a->irq_nonstall);

	err = gk20a_init_support(dev);
	if (err)
		return err;

#ifdef CONFIG_RESET_CONTROLLER
	platform->reset_control = devm_reset_control_get(&dev->dev, NULL);
	if (IS_ERR(platform->reset_control))
		platform->reset_control = NULL;
#endif

	err = nvgpu_probe(gk20a, "gpu.0", INTERFACE_NAME, &nvgpu_class);
	if (err)
		return err;

	err = gk20a_pm_init(&dev->dev);
	if (err) {
		dev_err(&dev->dev, "pm init failed");
		return err;
	}

	gk20a->mm.has_physical_mode = !nvgpu_is_hypervisor_mode(gk20a);

	return 0;
}

static int __exit gk20a_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	gk20a_dbg_fn("");

	if (gk20a_gpu_is_virtual(dev))
		return vgpu_remove(pdev);

	if (platform->has_cde)
		gk20a_cde_destroy(g);

	gk20a_ctxsw_trace_cleanup(g);

	gk20a_sched_ctrl_cleanup(g);

	if (IS_ENABLED(CONFIG_GK20A_DEVFREQ))
		gk20a_scale_exit(dev);

	if (g->remove_support)
		g->remove_support(g);

	gk20a_ce_destroy(g);

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	nvgpu_clk_arb_cleanup_arbiter(g);
#endif

	gk20a_user_deinit(dev, &nvgpu_class);

	gk20a_debug_deinit(g);

	nvgpu_remove_sysfs(dev);

	if (platform->secure_buffer.destroy)
		platform->secure_buffer.destroy(g,
				&platform->secure_buffer);

	if (pm_runtime_enabled(dev))
		pm_runtime_disable(dev);

	if (platform->remove)
		platform->remove(dev);

	set_gk20a(pdev, NULL);
	gk20a_put(g);

	gk20a_dbg_fn("removed");

	return 0;
}

static struct platform_driver gk20a_driver = {
	.probe = gk20a_probe,
	.remove = __exit_p(gk20a_remove),
	.shutdown = gk20a_pm_shutdown,
	.driver = {
		.owner = THIS_MODULE,
		.name = "gk20a",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
#endif
#ifdef CONFIG_OF
		.of_match_table = tegra_gk20a_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &gk20a_pm_ops,
#endif
		.suppress_bind_attrs = true,
	}
};

struct class nvgpu_class = {
	.owner = THIS_MODULE,
	.name = CLASS_NAME,
};

static int __init gk20a_init(void)
{

	int ret;

	ret = class_register(&nvgpu_class);
	if (ret)
		return ret;

	ret = nvgpu_pci_init();
	if (ret)
		return ret;

	return platform_driver_register(&gk20a_driver);
}

static void __exit gk20a_exit(void)
{
	nvgpu_pci_exit();
	platform_driver_unregister(&gk20a_driver);
	class_unregister(&nvgpu_class);
}

MODULE_LICENSE("GPL v2");
module_init(gk20a_init);
module_exit(gk20a_exit);
