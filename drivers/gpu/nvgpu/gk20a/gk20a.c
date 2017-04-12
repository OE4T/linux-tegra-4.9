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
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/thermal.h>
#include <asm/cacheflush.h>
#include <linux/debugfs.h>
#include <linux/clk/tegra.h>
#include <linux/kthread.h>
#include <linux/platform/tegra/common.h>
#include <linux/reset.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/version.h>

#include <nvgpu/nvgpu_common.h>
#include <nvgpu/kmem.h>
#include <nvgpu/allocator.h>
#include <nvgpu/timers.h>
#include <nvgpu/soc.h>

#include "gk20a.h"
#include "debug_gk20a.h"
#include "channel_sync_gk20a.h"

#include "gk20a_scale.h"
#include "ctxsw_trace_gk20a.h"
#include "dbg_gpu_gk20a.h"
#include "mc_gk20a.h"
#include "hal.h"
#include "vgpu/vgpu.h"
#include "pci.h"
#include "bus_gk20a.h"
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "pstate/pstate.h"
#endif


#define CREATE_TRACE_POINTS
#include <trace/events/gk20a.h>

#ifdef CONFIG_TEGRA_19x_GPU
#include "nvgpu_gpuid_t19x.h"
#endif

#include <nvgpu/hw/gk20a/hw_top_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ltc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_fb_gk20a.h>


#ifdef CONFIG_ARM64
#define __cpuc_flush_dcache_area __flush_dcache_area
#endif

#define CLASS_NAME "nvidia-gpu"
/* TODO: Change to e.g. "nvidia-gpu%s" once we have symlinks in place. */

#define GK20A_NUM_CDEVS 7

#define GK20A_WAIT_FOR_IDLE_MS	2000

static int gk20a_pm_prepare_poweroff(struct device *dev);

#ifdef CONFIG_DEBUG_FS
static int railgate_residency_show(struct seq_file *s, void *data)
{
	struct device *dev = s->private;
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a *g = get_gk20a(dev);
	unsigned long time_since_last_state_transition_ms;
	unsigned long total_rail_gate_time_ms;
	unsigned long total_rail_ungate_time_ms;

	if (platform->is_railgated(dev)) {
		time_since_last_state_transition_ms =
				jiffies_to_msecs(jiffies -
				g->pstats.last_rail_gate_complete);
		total_rail_ungate_time_ms = g->pstats.total_rail_ungate_time_ms;
		total_rail_gate_time_ms =
					g->pstats.total_rail_gate_time_ms +
					time_since_last_state_transition_ms;
	} else {
		time_since_last_state_transition_ms =
				jiffies_to_msecs(jiffies -
				g->pstats.last_rail_ungate_complete);
		total_rail_gate_time_ms = g->pstats.total_rail_gate_time_ms;
		total_rail_ungate_time_ms =
					g->pstats.total_rail_ungate_time_ms +
					time_since_last_state_transition_ms;
	}

	seq_printf(s, "Time with Rails Gated: %lu ms\n"
			"Time with Rails UnGated: %lu ms\n"
			"Total railgating cycles: %lu\n",
			total_rail_gate_time_ms,
			total_rail_ungate_time_ms,
			g->pstats.railgating_cycle_count - 1);
	return 0;

}

static int railgate_residency_open(struct inode *inode, struct file *file)
{
	return single_open(file, railgate_residency_show, inode->i_private);
}

static const struct file_operations railgate_residency_fops = {
	.open		= railgate_residency_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int gk20a_railgating_debugfs_init(struct device *dev)
{
	struct dentry *d;
	struct gk20a_platform *platform = dev_get_drvdata(dev);

	if (!platform->can_railgate)
		return 0;

	d = debugfs_create_file(
		"railgate_residency", S_IRUGO|S_IWUSR, platform->debugfs, dev,
						&railgate_residency_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}
#endif

static inline void set_gk20a(struct platform_device *pdev, struct gk20a *gk20a)
{
	gk20a_get_platform(&pdev->dev)->g = gk20a;
}

void __nvgpu_check_gpu_state(struct gk20a *g)
{
	u32 boot_0 = g->ops.mc.boot_0(g, NULL, NULL, NULL);

	if (boot_0 == 0xffffffff) {
		pr_err("nvgpu: GPU has disappeared from bus!!\n");
		pr_err("nvgpu: Rebooting system!!\n");
		kernel_restart(NULL);
	}
}

/*
 * Locks out the driver from accessing GPU registers. This prevents access to
 * thse registers after the GPU has been clock or power gated. This should help
 * find annoying bugs where register reads and writes are silently dropped
 * after the GPU has been turned off. On older chips these reads and writes can
 * also lock the entire CPU up.
 */
int gk20a_lockout_registers(struct gk20a *g)
{
	g->regs = NULL;
	g->bar1 = NULL;

	return 0;
}

/*
 * Undoes gk20a_lockout_registers().
 */
int gk20a_restore_registers(struct gk20a *g)
{
	g->regs = g->regs_saved;
	g->bar1 = g->bar1_saved;

	return 0;
}

void __gk20a_warn_on_no_regs(void)
{
	WARN_ONCE(1, "Attempted access to GPU regs after unmapping!");
}

void __iomem *gk20a_ioremap_resource(struct platform_device *dev, int i,
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

	return g->ops.mc.isr_stall(g);
}

static irqreturn_t gk20a_intr_isr_nonstall(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;

	return g->ops.mc.isr_nonstall(g);
}

static irqreturn_t gk20a_intr_thread_stall(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;
	return g->ops.mc.isr_thread_stall(g);
}

void gk20a_remove_support(struct gk20a *g)
{
#ifdef CONFIG_TEGRA_COMMON
	tegra_unregister_idle_unidle();
#endif
	if (g->dbg_regops_tmp_buf)
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

#ifdef CONFIG_TEGRA_COMMON
	tegra_register_idle_unidle(gk20a_do_idle, gk20a_do_unidle);
#endif

	g->regs = gk20a_ioremap_resource(dev, GK20A_BAR0_IORESOURCE_MEM,
					 &g->reg_mem);
	if (IS_ERR(g->regs)) {
		nvgpu_err(g, "failed to remap gk20a registers\n");
		err = PTR_ERR(g->regs);
		goto fail;
	}

	g->bar1 = gk20a_ioremap_resource(dev, GK20A_BAR1_IORESOURCE_MEM,
					 &g->bar1_mem);
	if (IS_ERR(g->bar1)) {
		nvgpu_err(g, "failed to remap gk20a bar1\n");
		err = PTR_ERR(g->bar1);
		goto fail;
	}

	if (nvgpu_platform_is_simulation(g)) {
		err = gk20a_init_sim_support(dev);
		if (err)
			goto fail;
	}

	return 0;

 fail:
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

	if (gk20a_fifo_is_engine_busy(g)) {
		nvgpu_mutex_release(&g->poweroff_lock);
		return -EBUSY;
	}
	gk20a_scale_suspend(dev);

	/* cancel any pending cde work */
	gk20a_cde_suspend(g);

	gk20a_ce_suspend(g);

	ret = gk20a_channel_suspend(g);
	if (ret)
		goto done;

	/* disable elpg before gr or fifo suspend */
	if (g->ops.pmu.is_pmu_supported(g))
		ret |= gk20a_pmu_destroy(g);
	/*
	 * After this point, gk20a interrupts should not get
	 * serviced.
	 */
	disable_irq(g->irq_stall);
	if (g->irq_stall != g->irq_nonstall)
		disable_irq(g->irq_nonstall);

	ret |= gk20a_gr_suspend(g);
	ret |= gk20a_mm_suspend(g);
	ret |= gk20a_fifo_suspend(g);

	if (g->ops.pmu.mclk_deinit)
		g->ops.pmu.mclk_deinit(g);

	/* Disable GPCPLL */
	if (g->ops.clk.suspend_clk_support)
		ret |= g->ops.clk.suspend_clk_support(g);

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	if (g->ops.pmupstate)
		gk20a_deinit_pstate_support(g);
#endif
	g->power_on = false;

	/* Decrement platform power refcount */
	if (platform->idle)
		platform->idle(dev);

	/* Stop CPU from accessing the GPU registers. */
	gk20a_lockout_registers(g);

done:
	nvgpu_mutex_release(&g->poweroff_lock);

	return ret;
}

static int gk20a_detect_chip(struct gk20a *g)
{
	struct nvgpu_gpu_characteristics *gpu = &g->gpu_characteristics;
	u32 val;

	if (gpu->arch)
		return 0;

	val = gk20a_mc_boot_0(g, &gpu->arch, &gpu->impl, &gpu->rev);

	gk20a_dbg_info("arch: %x, impl: %x, rev: %x\n",
			g->gpu_characteristics.arch,
			g->gpu_characteristics.impl,
			g->gpu_characteristics.rev);

	return gpu_init_hal(g);
}

int gk20a_pm_finalize_poweron(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	int err, nice_value;

	gk20a_dbg_fn("");

	if (g->power_on)
		return 0;

	trace_gk20a_finalize_poweron(g->name);

	/* Increment platform power refcount */
	if (platform->busy) {
		err = platform->busy(dev);
		if (err < 0) {
			nvgpu_err(g, "%s: failed to poweron platform dependency\n",
				__func__);
			goto done;
		}
	}

	err = gk20a_restore_registers(g);
	if (err)
		return err;

	nice_value = task_nice(current);
	set_user_nice(current, -20);

	g->power_on = true;

	err = gk20a_detect_chip(g);
	if (err)
		goto done;

	/*
	 * Before probing the GPU make sure the GPU's state is cleared. This is
	 * relevant for rebind operations.
	 */
	if (g->ops.xve.reset_gpu && !g->gpu_reset_done) {
		g->ops.xve.reset_gpu(g);
		g->gpu_reset_done = true;
	}

	if (g->ops.bios_init)
		err = g->ops.bios_init(g);
	if (err)
		goto done;

	g->ops.bus.init_hw(g);

	if (g->ops.clk.disable_slowboot)
		g->ops.clk.disable_slowboot(g);

	/* Enable interrupt workqueue */
	if (!g->nonstall_work_queue) {
		g->nonstall_work_queue = alloc_workqueue("%s",
						WQ_HIGHPRI, 1, "mc_nonstall");
		INIT_WORK(&g->nonstall_fn_work, g->ops.mc.isr_nonstall_cb);
	}

	gk20a_enable_priv_ring(g);

	/* TBD: move this after graphics init in which blcg/slcg is enabled.
	   This function removes SlowdownOnBoot which applies 32x divider
	   on gpcpll bypass path. The purpose of slowdown is to save power
	   during boot but it also significantly slows down gk20a init on
	   simulation and emulation. We should remove SOB after graphics power
	   saving features (blcg/slcg) are enabled. For now, do it here. */
	if (g->ops.clk.init_clk_support) {
		err = g->ops.clk.init_clk_support(g);
		if (err) {
			nvgpu_err(g, "failed to init gk20a clk");
			goto done;
		}
	}

	err = g->ops.fifo.reset_enable_hw(g);

	if (err) {
		nvgpu_err(g, "failed to reset gk20a fifo");
		goto done;
	}

	if (g->ops.ltc.init_fs_state)
		g->ops.ltc.init_fs_state(g);

	err = gk20a_init_mm_support(g);
	if (err) {
		nvgpu_err(g, "failed to init gk20a mm");
		goto done;
	}

	err = gk20a_init_fifo_support(g);
	if (err) {
		nvgpu_err(g, "failed to init gk20a fifo");
		goto done;
	}

	if (g->ops.therm.elcg_init_idle_filters)
		g->ops.therm.elcg_init_idle_filters(g);

	g->ops.mc.intr_enable(g);

	err = gk20a_enable_gr_hw(g);
	if (err) {
		nvgpu_err(g, "failed to enable gr");
		goto done;
	}

	if (g->ops.pmu.is_pmu_supported(g)) {
		if (g->ops.pmu.prepare_ucode)
			err = g->ops.pmu.prepare_ucode(g);
		if (err) {
			nvgpu_err(g, "failed to init pmu ucode");
			goto done;
		}
	}

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	if (g->ops.pmupstate) {
		err = gk20a_init_pstate_support(g);
		if (err) {
			nvgpu_err(g, "failed to init pstates");
			goto done;
		}
	}
#endif

	if (g->ops.pmu.is_pmu_supported(g)) {
		err = gk20a_init_pmu_support(g);
		if (err) {
			nvgpu_err(g, "failed to init gk20a pmu");
			goto done;
		}
	}

	err = gk20a_init_gr_support(g);
	if (err) {
		nvgpu_err(g, "failed to init gk20a gr");
		goto done;
	}

	if (g->ops.pmu.mclk_init) {
		err = g->ops.pmu.mclk_init(g);
		if (err) {
			nvgpu_err(g, "failed to set mclk");
			/* Indicate error dont goto done */
		}
	}

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	if (g->ops.pmupstate) {
		err = gk20a_init_pstate_pmu_support(g);
		if (err) {
			nvgpu_err(g, "failed to init pstates");
			goto done;
		}
	}

	err = nvgpu_clk_arb_init_arbiter(g);
	if (err) {
		nvgpu_err(g, "failed to init clk arb");
		goto done;
	}
#endif

	err = gk20a_init_therm_support(g);
	if (err) {
		nvgpu_err(g, "failed to init gk20a therm");
		goto done;
	}

	err = g->ops.chip_init_gpu_characteristics(g);
	if (err) {
		nvgpu_err(g, "failed to init gk20a gpu characteristics");
		goto done;
	}

	err = gk20a_ctxsw_trace_init(g);
	if (err)
		nvgpu_warn(g, "could not initialize ctxsw tracing");

	err = gk20a_sched_ctrl_init(g);
	if (err) {
		nvgpu_err(g, "failed to init sched control");
		goto done;
	}

	/* Restore the debug setting */
	g->ops.fb.set_debug_mode(g, g->mmu_debug_ctrl);

	gk20a_channel_resume(g);
	set_user_nice(current, nice_value);

	gk20a_scale_resume(dev);

	trace_gk20a_finalize_poweron_done(g->name);

	if (platform->has_cde)
		gk20a_init_cde_support(g);

	gk20a_init_ce_support(g);

	gk20a_init_mm_ce_context(g);

	enable_irq(g->irq_stall);
	if (g->irq_stall != g->irq_nonstall)
		enable_irq(g->irq_nonstall);
	g->irqs_enabled = 1;

	if (g->ops.xve.available_speeds) {
		u32 speed;

		if (platform->disable_aspm && g->ops.xve.disable_aspm)
			g->ops.xve.disable_aspm(g);

		g->ops.xve.sw_init(dev);
		g->ops.xve.available_speeds(g, &speed);

		/* Set to max speed */
		speed = 1 << (fls(speed) - 1);
		err = g->ops.xve.set_speed(g, speed);
		if (err) {
			nvgpu_err(g, "Failed to set PCIe bus speed!\n");
			goto done;
		}
	}

done:
	if (err)
		g->power_on = false;

	return err;
}

static struct of_device_id tegra_gk20a_of_match[] = {
#ifdef CONFIG_TEGRA_GK20A
	{ .compatible = "nvidia,tegra124-gk20a",
		.data = &gk20a_tegra_platform },
	{ .compatible = "nvidia,tegra210-gm20b",
		.data = &gm20b_tegra_platform },
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = "nvidia,tegra186-gp10b",
		.data = &gp10b_tegra_platform },
#endif
#ifdef CONFIG_TEGRA_19x_GPU
	{ .compatible = TEGRA_19x_GPU_COMPAT_TEGRA,
		.data = &t19x_gpu_tegra_platform },
#endif
#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	{ .compatible = "nvidia,tegra124-gk20a-vgpu",
		.data = &vgpu_tegra_platform },
#endif
#else
	{ .compatible = "nvidia,tegra124-gk20a",
		.data = &gk20a_generic_platform },
	{ .compatible = "nvidia,tegra210-gm20b",
		.data = &gk20a_generic_platform },
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = TEGRA_18x_GPU_COMPAT_TEGRA,
		.data = &gk20a_generic_platform },
#endif

#endif
	{ .compatible = "nvidia,generic-gk20a",
		.data = &gk20a_generic_platform },
	{ .compatible = "nvidia,generic-gm20b",
		.data = &gk20a_generic_platform },
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = "nvidia,generic-gp10b",
		.data = &gk20a_generic_platform },
#endif
	{ },
};

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
	struct gk20a *g = get_gk20a(dev);

#ifdef CONFIG_DEBUG_FS
	g->pstats.last_rail_ungate_start = jiffies;
	if (g->pstats.railgating_cycle_count >= 1)
		g->pstats.total_rail_gate_time_ms =
			g->pstats.total_rail_gate_time_ms +
			jiffies_to_msecs(g->pstats.last_rail_ungate_start -
				g->pstats.last_rail_gate_complete);

	g->pstats.railgating_cycle_count++;
#endif

	trace_gk20a_pm_unrailgate(g->name);

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
	nvgpu_info(g, "shut down complete\n");
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

	if (platform->user_railgate_disabled)
		gk20a_idle_nosuspend(dev);

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
	if (platform->user_railgate_disabled)
		gk20a_busy_noresume(dev);

	return ret;
}

static int gk20a_pm_resume(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int ret = 0;

	if (platform->user_railgate_disabled)
		gk20a_busy_noresume(dev);

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

int gk20a_pm_init(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int err = 0;

	gk20a_dbg_fn("");

	/* Initialise pm runtime */
	if (platform->railgate_delay) {
		pm_runtime_set_autosuspend_delay(dev,
				 platform->railgate_delay);
		pm_runtime_use_autosuspend(dev);
	}

	if (platform->can_railgate) {
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

int gk20a_secure_page_alloc(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int err = 0;

	if (platform->secure_page_alloc) {
		err = platform->secure_page_alloc(dev);
		if (!err)
			platform->secure_alloc_ready = true;
	}

	return err;
}

static int gk20a_probe(struct platform_device *dev)
{
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

	gk20a = kzalloc(sizeof(struct gk20a), GFP_KERNEL);
	if (!gk20a) {
		dev_err(&dev->dev, "couldn't allocate gk20a support");
		return -ENOMEM;
	}

	set_gk20a(dev, gk20a);
	gk20a->dev = &dev->dev;

	if (nvgpu_platform_is_simulation(gk20a))
		platform->is_fmodel = true;

	nvgpu_kmem_init(gk20a);

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

	/*
	 * is_fmodel needs to be in gk20a struct for deferred teardown
	 */
	gk20a->is_fmodel = platform->is_fmodel;

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

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	nvgpu_clk_arb_cleanup_arbiter(g);
#endif

	gk20a_user_deinit(dev, &nvgpu_class);

	debugfs_remove_recursive(platform->debugfs);
	debugfs_remove_recursive(platform->debugfs_alias);

	gk20a_remove_sysfs(dev);

	if (platform->secure_buffer.destroy)
		platform->secure_buffer.destroy(dev,
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

void gk20a_busy_noresume(struct device *dev)
{
	pm_runtime_get_noresume(dev);
}

/*
 * Start the process for unloading the driver. Set g->driver_is_dying.
 */
void gk20a_driver_start_unload(struct gk20a *g)
{
	gk20a_dbg(gpu_dbg_shutdown, "Driver is now going down!\n");

	down_write(&g->busy_lock);
	g->driver_is_dying = 1;
	up_write(&g->busy_lock);

	if (gk20a_gpu_is_virtual(g->dev))
		return;

	gk20a_wait_for_idle(g->dev);

	nvgpu_wait_for_deferred_interrupts(g);
	gk20a_channel_cancel_pending_sema_waits(g);

	if (g->nonstall_work_queue) {
		cancel_work_sync(&g->nonstall_fn_work);
		destroy_workqueue(g->nonstall_work_queue);
		g->nonstall_work_queue = NULL;
	}
}

int gk20a_wait_for_idle(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform;
	int wait_length = 150; /* 3 second overall max wait. */
	int target_usage_count = 0;

	if (!g)
		return -ENODEV;

	platform = dev_get_drvdata(dev);
	if (platform->user_railgate_disabled)
		target_usage_count = 1;

	while ((atomic_read(&g->usage_count) != target_usage_count)
			&& (wait_length-- >= 0))
		nvgpu_msleep(20);

	if (wait_length < 0) {
		pr_warn("%s: Timed out waiting for idle (%d)!\n",
			__func__, atomic_read(&g->usage_count));
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Check if the device can go busy. Basically if the driver is currently
 * in the process of dying then do not let new places make the driver busy.
 */
static int gk20a_can_busy(struct gk20a *g)
{
	if (g->driver_is_dying)
		return 0;
	return 1;
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

	dev = g->dev;

	if (pm_runtime_enabled(dev)) {
		ret = pm_runtime_get_sync(dev);
		if (ret < 0) {
			pm_runtime_put_noidle(dev);
			atomic_dec(&g->usage_count);
			goto fail;
		}
	} else {
		if (!g->power_on) {
			ret = gk20a_gpu_is_virtual(dev) ?
				vgpu_pm_finalize_poweron(dev)
				: gk20a_pm_finalize_poweron(dev);
			if (ret) {
				atomic_dec(&g->usage_count);
				goto fail;
			}
		}
	}

	gk20a_scale_notify_busy(dev);

fail:
	up_read(&g->busy_lock);

	return ret < 0 ? ret : 0;
}

void gk20a_idle_nosuspend(struct device *dev)
{
	pm_runtime_put_noidle(dev);
}

void gk20a_idle(struct gk20a *g)
{
	struct device *dev;

	atomic_dec(&g->usage_count);
	down_read(&g->busy_lock);

	dev = g->dev;

	if (!(dev && gk20a_can_busy(g)))
		goto fail;

	if (pm_runtime_enabled(dev)) {
#ifdef CONFIG_PM
		if (atomic_read(&g->dev->power.usage_count) == 1)
			gk20a_scale_notify_idle(dev);
#endif

		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_sync_autosuspend(dev);

	} else {
		gk20a_scale_notify_idle(dev);
	}
fail:
	up_read(&g->busy_lock);
}

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
int __gk20a_do_idle(struct device *dev, bool force_reset)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct nvgpu_timeout timeout;
	int ref_cnt;
	int target_ref_cnt = 0;
	bool is_railgated;
	int err = 0;

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
	if (platform->user_railgate_disabled)
		target_ref_cnt = 2;
	else
		target_ref_cnt = 1;
	nvgpu_mutex_acquire(&platform->railgate_lock);

	nvgpu_timeout_init(g, &timeout, GK20A_WAIT_FOR_IDLE_MS,
			   NVGPU_TIMER_CPU_TIMER);

	/* check and wait until GPU is idle (with a timeout) */
	do {
		nvgpu_msleep(1);
		ref_cnt = atomic_read(&dev->power.usage_count);
	} while (ref_cnt != target_ref_cnt && !nvgpu_timeout_expired(&timeout));

	if (ref_cnt != target_ref_cnt) {
		nvgpu_err(g, "failed to idle - refcount %d != 1\n",
			ref_cnt);
		goto fail_drop_usage_count;
	}

	/* check if global force_reset flag is set */
	force_reset |= platform->force_reset_in_do_idle;

	nvgpu_timeout_init(g, &timeout, GK20A_WAIT_FOR_IDLE_MS,
			   NVGPU_TIMER_CPU_TIMER);

	if (platform->can_railgate && !force_reset) {
		/*
		 * Case 1 : GPU railgate is supported
		 *
		 * if GPU is now idle, we will have only one ref count,
		 * drop this ref which will rail gate the GPU
		 */
		pm_runtime_put_sync(dev);

		/* add sufficient delay to allow GPU to rail gate */
		nvgpu_msleep(platform->railgate_delay);

		/* check in loop if GPU is railgated or not */
		do {
			nvgpu_msleep(1);
			is_railgated = platform->is_railgated(dev);
		} while (!is_railgated && !nvgpu_timeout_expired(&timeout));

		if (is_railgated) {
			return 0;
		} else {
			nvgpu_err(g, "failed to idle in timeout\n");
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
	return -EBUSY;
}

/**
 * gk20a_do_idle() - wrap up for __gk20a_do_idle() to be called
 * from outside of GPU driver
 *
 * In success, this call MUST be balanced by caller with gk20a_do_unidle()
 */
int gk20a_do_idle(void)
{
	struct device_node *node =
			of_find_matching_node(NULL, tegra_gk20a_of_match);
	struct platform_device *pdev = of_find_device_by_node(node);

	int ret =  __gk20a_do_idle(&pdev->dev, true);

	of_node_put(node);

	return ret;
}

/**
 * __gk20a_do_unidle() - unblock all the tasks blocked by __gk20a_do_idle()
 */
int __gk20a_do_unidle(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);

	if (g->forced_reset) {
		/*
		 * If we did a forced-reset/railgate
		 * then unrailgate the GPU here first
		 */
		platform->unrailgate(dev);

		/* restore the GPU state */
		gk20a_pm_finalize_poweron(dev);

		/* balance GPU usage counter */
		pm_runtime_put_sync(dev);

		g->forced_reset = false;
	}

	/* release the lock and open up all other busy() calls */
	nvgpu_mutex_release(&platform->railgate_lock);
	up_write(&g->busy_lock);

	return 0;
}

/**
 * gk20a_do_unidle() - wrap up for __gk20a_do_unidle()
 */
int gk20a_do_unidle(void)
{
	struct device_node *node =
			of_find_matching_node(NULL, tegra_gk20a_of_match);
	struct platform_device *pdev = of_find_device_by_node(node);

	int ret = __gk20a_do_unidle(&pdev->dev);

	of_node_put(node);

	return ret;
}
#endif

int gk20a_init_gpu_characteristics(struct gk20a *g)
{
	struct nvgpu_gpu_characteristics *gpu = &g->gpu_characteristics;
	struct gk20a_platform *platform = dev_get_drvdata(g->dev);

	gpu->L2_cache_size = g->ops.ltc.determine_L2_size_bytes(g);
	gpu->on_board_video_memory_size = 0; /* integrated GPU */

	gpu->num_gpc = g->gr.gpc_count;
	gpu->max_gpc_count = g->gr.max_gpc_count;

	gpu->num_tpc_per_gpc = g->gr.max_tpc_per_gpc_count;

	gpu->bus_type = NVGPU_GPU_BUS_TYPE_AXI; /* always AXI for now */

	gpu->big_page_size = g->mm.pmu.vm.big_page_size;
	gpu->compression_page_size = g->ops.fb.compression_page_size(g);
	gpu->pde_coverage_bit_count =
		gk20a_mm_pde_coverage_bit_count(&g->mm.pmu.vm);

	if (g->mm.disable_bigpage) {
		gpu->big_page_size = 0;
		gpu->available_big_page_sizes = 0;
	} else {
		gpu->available_big_page_sizes = gpu->big_page_size;
		if (g->ops.mm.get_big_page_sizes)
			gpu->available_big_page_sizes |= g->ops.mm.get_big_page_sizes();
	}

	gpu->flags = NVGPU_GPU_FLAGS_SUPPORT_PARTIAL_MAPPINGS;

	if (IS_ENABLED(CONFIG_SYNC))
		gpu->flags |= NVGPU_GPU_FLAGS_SUPPORT_SYNC_FENCE_FDS;

	if (g->ops.mm.support_sparse && g->ops.mm.support_sparse(g))
		gpu->flags |= NVGPU_GPU_FLAGS_SUPPORT_SPARSE_ALLOCS;

	if (gk20a_platform_has_syncpoints(g->dev))
		gpu->flags |= NVGPU_GPU_FLAGS_HAS_SYNCPOINTS;

	gpu->flags |= NVGPU_GPU_FLAGS_SUPPORT_USERSPACE_MANAGED_AS;
	gpu->flags |= NVGPU_GPU_FLAGS_SUPPORT_TSG;
	gpu->flags |= NVGPU_GPU_FLAGS_SUPPORT_MAP_COMPBITS;

	if (g->ops.clk_arb.get_arbiter_clk_domains)
		gpu->flags |= NVGPU_GPU_FLAGS_SUPPORT_CLOCK_CONTROLS;

	gpu->gpc_mask = (1 << g->gr.gpc_count)-1;

	g->ops.gr.detect_sm_arch(g);

	if (g->ops.gr.init_cyclestats)
		g->ops.gr.init_cyclestats(g);

	gpu->gpu_ioctl_nr_last = NVGPU_GPU_IOCTL_LAST;
	gpu->tsg_ioctl_nr_last = NVGPU_TSG_IOCTL_LAST;
	gpu->dbg_gpu_ioctl_nr_last = NVGPU_DBG_GPU_IOCTL_LAST;
	gpu->ioctl_channel_nr_last = NVGPU_IOCTL_CHANNEL_LAST;
	gpu->as_ioctl_nr_last = NVGPU_AS_IOCTL_LAST;
	gpu->event_ioctl_nr_last = NVGPU_EVENT_IOCTL_LAST;
	gpu->gpu_va_bit_count = 40;

	strlcpy(gpu->chipname, g->name, sizeof(gpu->chipname));
	gpu->max_fbps_count = g->ops.gr.get_max_fbps_count(g);
	gpu->fbp_en_mask = g->ops.gr.get_fbp_en_mask(g);
	gpu->max_ltc_per_fbp =  g->ops.gr.get_max_ltc_per_fbp(g);
	gpu->max_lts_per_ltc = g->ops.gr.get_max_lts_per_ltc(g);
	g->ops.gr.get_rop_l2_en_mask(g);
	gpu->gr_compbit_store_base_hw = g->gr.compbit_store.base_hw;
	gpu->gr_gobs_per_comptagline_per_slice =
		g->gr.gobs_per_comptagline_per_slice;
	gpu->num_ltc = g->ltc_count;
	gpu->lts_per_ltc = g->gr.slices_per_ltc;
	gpu->cbc_cache_line_size = g->gr.cacheline_size;
	gpu->cbc_comptags_per_line = g->gr.comptags_per_cacheline;

	gpu->map_buffer_batch_limit = 256;

	if (platform->clk_round_rate)
		gpu->max_freq = platform->clk_round_rate(g->dev, UINT_MAX);

	g->ops.gr.get_preemption_mode_flags(g, &g->gr.preemption_mode_rec);
	gpu->graphics_preemption_mode_flags =
		g->gr.preemption_mode_rec.graphics_preemption_mode_flags;
	gpu->compute_preemption_mode_flags =
		g->gr.preemption_mode_rec.compute_preemption_mode_flags;
	gpu->default_graphics_preempt_mode =
		g->gr.preemption_mode_rec.default_graphics_preempt_mode;
	gpu->default_compute_preempt_mode =
		g->gr.preemption_mode_rec.default_compute_preempt_mode;

	gpu->local_video_memory_size = g->mm.vidmem.size;

	gpu->pci_vendor_id = g->pci_vendor_id;
	gpu->pci_device_id = g->pci_device_id;
	gpu->pci_subsystem_vendor_id = g->pci_subsystem_vendor_id;
	gpu->pci_subsystem_device_id = g->pci_subsystem_device_id;
	gpu->pci_class = g->pci_class;
	gpu->pci_revision = g->pci_revision;

	gpu->reg_ops_limit = 1024;

	return 0;
}

/*
 * Free the gk20a struct.
 */
static void gk20a_free_cb(struct kref *refcount)
{
	struct gk20a *g = container_of(refcount,
		struct gk20a, refcount);

	gk20a_dbg(gpu_dbg_shutdown, "Freeing GK20A struct!");

	gk20a_ce_destroy(g);

	if (g->remove_support)
		g->remove_support(g);

	kfree(g);
}

/**
 * gk20a_get() - Increment ref count on driver
 *
 * @g The driver to increment
 * This will fail if the driver is in the process of being released. In that
 * case it will return NULL. Otherwise a pointer to the driver passed in will
 * be returned.
 */
struct gk20a * __must_check gk20a_get(struct gk20a *g)
{
	int success;

	/*
	 * Handle the possibility we are still freeing the gk20a struct while
	 * gk20a_get() is called. Unlikely but plausible race condition. Ideally
	 * the code will never be in such a situation that this race is
	 * possible.
	 */
	success = kref_get_unless_zero(&g->refcount);

	gk20a_dbg(gpu_dbg_shutdown, "GET: refs currently %d %s",
		atomic_read(&g->refcount.refcount), success ? "" : "(FAILED)");

	return success ? g : NULL;
}

/**
 * gk20a_put() - Decrement ref count on driver
 *
 * @g - The driver to decrement
 *
 * Decrement the driver ref-count. If neccesary also free the underlying driver
 * memory
 */
void gk20a_put(struct gk20a *g)
{
	/*
	 * Note - this is racy, two instances of this could run before the
	 * actual kref_put(0 runs, you could see something like:
	 *
	 *  ... PUT: refs currently 2
	 *  ... PUT: refs currently 2
	 *  ... Freeing GK20A struct!
	 */
	gk20a_dbg(gpu_dbg_shutdown, "PUT: refs currently %d",
		atomic_read(&g->refcount.refcount));

	kref_put(&g->refcount, gk20a_free_cb);
}

MODULE_LICENSE("GPL v2");
module_init(gk20a_init);
module_exit(gk20a_exit);
