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

#include <linux/reboot.h>

#include <nvgpu/nvgpu_common.h>
#include <nvgpu/kmem.h>
#include <nvgpu/allocator.h>
#include <nvgpu/timers.h>
#include <nvgpu/soc.h>
#include <nvgpu/enabled.h>
#include <nvgpu/pmu.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/ltc.h>

#include <trace/events/gk20a.h>

#include "gk20a.h"
#include "platform_gk20a.h"
#include "channel_sync_gk20a.h"

#include "ctxsw_trace_gk20a.h"
#include "dbg_gpu_gk20a.h"
#include "mc_gk20a.h"
#include "hal.h"
#include "vgpu/vgpu.h"
#include "bus_gk20a.h"
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "pstate/pstate.h"
#endif

#ifdef CONFIG_TEGRA_19x_GPU
#include "nvgpu_gpuid_t19x.h"
#endif

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

int gk20a_prepare_poweroff(struct gk20a *g)
{
	int ret = 0;

	gk20a_dbg_fn("");

	if (gk20a_fifo_is_engine_busy(g))
		return -EBUSY;

	/* cancel any pending cde work */
	gk20a_cde_suspend(g);

	gk20a_ce_suspend(g);

	ret = gk20a_channel_suspend(g);
	if (ret)
		return ret;

	/* disable elpg before gr or fifo suspend */
	if (g->ops.pmu.is_pmu_supported(g))
		ret |= nvgpu_pmu_destroy(g);

	ret |= gk20a_gr_suspend(g);
	ret |= gk20a_mm_suspend(g);
	ret |= gk20a_fifo_suspend(g);

	/* Disable GPCPLL */
	if (g->ops.clk.suspend_clk_support)
		ret |= g->ops.clk.suspend_clk_support(g);

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	if (nvgpu_is_enabled(g, NVGPU_PMU_PSTATE))
		gk20a_deinit_pstate_support(g);
#endif
	g->power_on = false;

	return ret;
}

int gk20a_finalize_poweron(struct gk20a *g)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev_from_gk20a(g));
	int err;

	gk20a_dbg_fn("");

	if (g->power_on)
		return 0;

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

	/*
	 * Do this early so any early VMs that get made are capable of mapping
	 * buffers.
	 */
	err = nvgpu_pd_cache_init(g);
	if (err)
		return err;

	/* init interface layer support for PMU falcon */
	nvgpu_flcn_sw_init(g, FALCON_ID_PMU);
	nvgpu_flcn_sw_init(g, FALCON_ID_SEC2);

	if (g->ops.bios_init)
		err = g->ops.bios_init(g);
	if (err)
		goto done;

	g->ops.bus.init_hw(g);

	if (g->ops.clk.disable_slowboot)
		g->ops.clk.disable_slowboot(g);

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

	err = nvgpu_init_ltc_support(g);
	if (err) {
		nvgpu_err(g, "failed to init ltc");
		goto done;
	}

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
	if (nvgpu_is_enabled(g, NVGPU_PMU_PSTATE)) {
		err = gk20a_init_pstate_support(g);
		if (err) {
			nvgpu_err(g, "failed to init pstates");
			goto done;
		}
	}
#endif

	if (g->ops.pmu.is_pmu_supported(g)) {
		err = nvgpu_init_pmu_support(g);
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

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	if (nvgpu_is_enabled(g, NVGPU_PMU_PSTATE)) {
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

	gk20a_init_ce_support(g);

	gk20a_init_mm_ce_context(g);

	if (g->ops.xve.available_speeds) {
		u32 speed;

		if (platform->disable_aspm && g->ops.xve.disable_aspm)
			g->ops.xve.disable_aspm(g);

		g->ops.xve.sw_init(g);
		g->ops.xve.available_speeds(g, &speed);

		/* Set to max speed */
		speed = 1 << (fls(speed) - 1);
		err = g->ops.xve.set_speed(g, speed);
		if (err) {
			nvgpu_err(g, "Failed to set PCIe bus speed!");
			goto done;
		}
	}

done:
	if (err)
		g->power_on = false;

	return err;
}

/*
 * Check if the device can go busy. Basically if the driver is currently
 * in the process of dying then do not let new places make the driver busy.
 */
int gk20a_can_busy(struct gk20a *g)
{
	if (nvgpu_is_enabled(g, NVGPU_DRIVER_IS_DYING))
		return 0;
	return 1;
}

int gk20a_wait_for_idle(struct gk20a *g)
{
	int wait_length = 150; /* 3 second overall max wait. */
	int target_usage_count = 0;

	if (!g)
		return -ENODEV;

	if (g->user_railgate_disabled)
		target_usage_count = 1;

	while ((nvgpu_atomic_read(&g->usage_count) != target_usage_count)
			&& (wait_length-- >= 0))
		nvgpu_msleep(20);

	if (wait_length < 0) {
		pr_warn("%s: Timed out waiting for idle (%d)!\n",
			__func__, nvgpu_atomic_read(&g->usage_count));
		return -ETIMEDOUT;
	}

	return 0;
}

int gk20a_init_gpu_characteristics(struct gk20a *g)
{
	struct nvgpu_gpu_characteristics *gpu = &g->gpu_characteristics;
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));

	gpu->L2_cache_size = g->ops.ltc.determine_L2_size_bytes(g);
	gpu->on_board_video_memory_size = 0; /* integrated GPU */

	gpu->num_gpc = g->gr.gpc_count;
	gpu->max_gpc_count = g->gr.max_gpc_count;

	gpu->num_tpc_per_gpc = g->gr.max_tpc_per_gpc_count;

	gpu->bus_type = NVGPU_GPU_BUS_TYPE_AXI; /* always AXI for now */

	gpu->compression_page_size = g->ops.fb.compression_page_size(g);
	gpu->big_page_size = g->ops.mm.get_default_big_page_size();
	gpu->pde_coverage_bit_count =
		g->ops.mm.get_mmu_levels(g, gpu->big_page_size)[0].lo_bit[0];

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

	if (gk20a_platform_has_syncpoints(g))
		gpu->flags |= NVGPU_GPU_FLAGS_HAS_SYNCPOINTS;

	/*
	 * Fast submits are supported as long as the user doesn't request
	 * anything that depends on job tracking. (Here, fast means strictly no
	 * metadata, just the gpfifo contents are copied and gp_put updated).
	 */
	gpu->flags |= NVGPU_GPU_FLAGS_SUPPORT_DETERMINISTIC_SUBMIT_NO_JOBTRACKING;

	/*
	 * Sync framework requires deferred job cleanup, wrapping syncs in FDs,
	 * and other heavy stuff, which prevents deterministic submits. This is
	 * supported otherwise, provided that the user doesn't request anything
	 * that depends on deferred cleanup.
	 */
	if (!gk20a_channel_sync_needs_sync_framework(g))
		gpu->flags |= NVGPU_GPU_FLAGS_SUPPORT_DETERMINISTIC_SUBMIT_FULL;

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
		gpu->max_freq = platform->clk_round_rate(dev_from_gk20a(g),
							 UINT_MAX);

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
static void gk20a_free_cb(struct nvgpu_ref *refcount)
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
	success = nvgpu_ref_get_unless_zero(&g->refcount);

	gk20a_dbg(gpu_dbg_shutdown, "GET: refs currently %d %s",
		nvgpu_atomic_read(&g->refcount.refcount),
			success ? "" : "(FAILED)");

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
		nvgpu_atomic_read(&g->refcount.refcount));

	nvgpu_ref_put(&g->refcount, gk20a_free_cb);
}
