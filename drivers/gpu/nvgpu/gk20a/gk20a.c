/*
 * GK20A Graphics
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <nvgpu/nvgpu_common.h>
#include <nvgpu/kmem.h>
#include <nvgpu/allocator.h>
#include <nvgpu/timers.h>
#include <nvgpu/soc.h>
#include <nvgpu/enabled.h>
#include <nvgpu/pmu.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/ltc.h>
#include <nvgpu/vidmem.h>
#include <nvgpu/mm.h>
#include <nvgpu/ctxsw_trace.h>

#include <trace/events/gk20a.h>

#include "gk20a.h"
#include "channel_sync_gk20a.h"

#include "dbg_gpu_gk20a.h"
#include "mc_gk20a.h"
#include "hal.h"
#include "bus_gk20a.h"
#include "pstate/pstate.h"

void __nvgpu_check_gpu_state(struct gk20a *g)
{
	u32 boot_0 = 0xffffffff;

	if (!g->ops.mc.boot_0) {
		nvgpu_err(g, "Can't determine GPU state, mc.boot_0 unset");
		return;
	}

	boot_0 = g->ops.mc.boot_0(g, NULL, NULL, NULL);
	if (boot_0 == 0xffffffff) {
		nvgpu_err(g, "GPU has disappeared from bus!!");
		nvgpu_err(g, "Rebooting system!!");
		nvgpu_kernel_restart(NULL);
	}
}

void __gk20a_warn_on_no_regs(void)
{
	WARN_ONCE(1, "Attempted access to GPU regs after unmapping!");
}

int gk20a_detect_chip(struct gk20a *g)
{
	struct nvgpu_gpu_params *p = &g->params;

	if (p->gpu_arch)
		return 0;

	gk20a_mc_boot_0(g, &p->gpu_arch, &p->gpu_impl, &p->gpu_rev);

	gk20a_dbg_info("arch: %x, impl: %x, rev: %x\n",
			g->params.gpu_arch,
			g->params.gpu_impl,
			g->params.gpu_rev);

	return gpu_init_hal(g);
}

int gk20a_prepare_poweroff(struct gk20a *g)
{
	int ret = 0;

	gk20a_dbg_fn("");

	if (g->ops.fifo.channel_suspend) {
		ret = g->ops.fifo.channel_suspend(g);
		if (ret)
			return ret;
	}

	/* disable elpg before gr or fifo suspend */
	if (g->ops.pmu.is_pmu_supported(g))
		ret |= nvgpu_pmu_destroy(g);

	ret |= gk20a_gr_suspend(g);
	ret |= nvgpu_mm_suspend(g);
	ret |= gk20a_fifo_suspend(g);

	gk20a_ce_suspend(g);

	/* Disable GPCPLL */
	if (g->ops.clk.suspend_clk_support)
		ret |= g->ops.clk.suspend_clk_support(g);

	if (nvgpu_is_enabled(g, NVGPU_PMU_PSTATE))
		gk20a_deinit_pstate_support(g);

	g->power_on = false;

	return ret;
}

int gk20a_finalize_poweron(struct gk20a *g)
{
	int err;
#if defined(CONFIG_TEGRA_GK20A_NVHOST)
	u32 nr_pages;
#endif

	gk20a_dbg_fn("");

	if (g->power_on)
		return 0;

	g->power_on = true;

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
	nvgpu_flcn_sw_init(g, FALCON_ID_NVDEC);

	if (g->ops.bios.init)
		err = g->ops.bios.init(g);
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

	if (nvgpu_is_enabled(g, NVGPU_SUPPORT_NVLINK)) {
		if (g->ops.nvlink.init) {
			err = g->ops.nvlink.init(g);
			if (err) {
				nvgpu_err(g, "failed to init nvlink");
				__nvgpu_set_enabled(g, NVGPU_SUPPORT_NVLINK,
									false);
			}
		} else
			__nvgpu_set_enabled(g, NVGPU_SUPPORT_NVLINK, false);
	}

	if (g->ops.fb.mem_unlock) {
		err = g->ops.fb.mem_unlock(g);
		if (err) {
			nvgpu_err(g, "failed to unlock memory");
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

	err = nvgpu_init_mm_support(g);
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

	if (nvgpu_is_enabled(g, NVGPU_PMU_PSTATE)) {
		err = gk20a_init_pstate_support(g);
		if (err) {
			nvgpu_err(g, "failed to init pstates");
			goto done;
		}
	}

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

#ifdef CONFIG_GK20A_CTXSW_TRACE
	err = gk20a_ctxsw_trace_init(g);
	if (err)
		nvgpu_warn(g, "could not initialize ctxsw tracing");
#endif

	/* Restore the debug setting */
	g->ops.fb.set_debug_mode(g, g->mmu_debug_ctrl);

	gk20a_init_ce_support(g);

	if (g->ops.xve.available_speeds) {
		u32 speed;

		if (!nvgpu_is_enabled(g, NVGPU_SUPPORT_ASPM) && g->ops.xve.disable_aspm)
			g->ops.xve.disable_aspm(g);

		g->ops.xve.available_speeds(g, &speed);

		/* Set to max speed */
		speed = 1 << (fls(speed) - 1);
		err = g->ops.xve.set_speed(g, speed);
		if (err) {
			nvgpu_err(g, "Failed to set PCIe bus speed!");
			goto done;
		}
	}

#if defined(CONFIG_TEGRA_GK20A_NVHOST)
	if (gk20a_platform_has_syncpoints(g) && g->syncpt_unit_size) {
		if (!nvgpu_mem_is_valid(&g->syncpt_mem)) {
			nr_pages = DIV_ROUND_UP(g->syncpt_unit_size, PAGE_SIZE);
			__nvgpu_mem_create_from_phys(g, &g->syncpt_mem,
					g->syncpt_unit_base, nr_pages);
		}
	}
#endif

	if (g->ops.fifo.channel_resume)
		g->ops.fifo.channel_resume(g);

	nvgpu_init_mm_ce_context(g);

	nvgpu_vidmem_thread_unpause(&g->mm);

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
		nvgpu_warn(g, "Timed out waiting for idle (%d)!\n",
			   nvgpu_atomic_read(&g->usage_count));
		return -ETIMEDOUT;
	}

	return 0;
}

int gk20a_init_gpu_characteristics(struct gk20a *g)
{
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_PARTIAL_MAPPINGS, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_MAP_DIRECT_KIND_CTRL, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_MAP_BUFFER_BATCH, true);

	if (IS_ENABLED(CONFIG_SYNC))
		__nvgpu_set_enabled(g, NVGPU_SUPPORT_SYNC_FENCE_FDS, true);

	if (g->ops.mm.support_sparse && g->ops.mm.support_sparse(g))
		__nvgpu_set_enabled(g, NVGPU_SUPPORT_SPARSE_ALLOCS, true);

	if (gk20a_platform_has_syncpoints(g))
		__nvgpu_set_enabled(g, NVGPU_HAS_SYNCPOINTS, true);

	/*
	 * Fast submits are supported as long as the user doesn't request
	 * anything that depends on job tracking. (Here, fast means strictly no
	 * metadata, just the gpfifo contents are copied and gp_put updated).
	 */
	__nvgpu_set_enabled(g,
			NVGPU_SUPPORT_DETERMINISTIC_SUBMIT_NO_JOBTRACKING,
			true);

	/*
	 * Sync framework requires deferred job cleanup, wrapping syncs in FDs,
	 * and other heavy stuff, which prevents deterministic submits. This is
	 * supported otherwise, provided that the user doesn't request anything
	 * that depends on deferred cleanup.
	 */
	if (!gk20a_channel_sync_needs_sync_framework(g))
		__nvgpu_set_enabled(g,
				NVGPU_SUPPORT_DETERMINISTIC_SUBMIT_FULL,
				true);

	__nvgpu_set_enabled(g, NVGPU_SUPPORT_DETERMINISTIC_OPTS, true);

	__nvgpu_set_enabled(g, NVGPU_SUPPORT_USERSPACE_MANAGED_AS, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_TSG, true);

	if (g->ops.clk_arb.get_arbiter_clk_domains)
		__nvgpu_set_enabled(g, NVGPU_SUPPORT_CLOCK_CONTROLS, true);

	g->ops.gr.detect_sm_arch(g);

	if (g->ops.gr.init_cyclestats)
		g->ops.gr.init_cyclestats(g);

	g->ops.gr.get_rop_l2_en_mask(g);

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

	if (g->free)
		g->free(g);
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
