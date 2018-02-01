/*
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

#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <uapi/linux/nvgpu.h>

#include <nvgpu/defaults.h>
#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/soc.h>
#include <nvgpu/bug.h>
#include <nvgpu/enabled.h>
#include <nvgpu/debug.h>
#include <nvgpu/sizes.h>

#include "gk20a/gk20a.h"
#include "platform_gk20a.h"
#include "module.h"
#include "os_linux.h"
#include "sysfs.h"
#include "ioctl.h"
#include "gk20a/regops_gk20a.h"

#define EMC3D_DEFAULT_RATIO 750

static void nvgpu_init_vars(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = dev_get_drvdata(dev);

	nvgpu_cond_init(&l->sw_irq_stall_last_handled_wq);
	nvgpu_cond_init(&l->sw_irq_nonstall_last_handled_wq);

	init_rwsem(&l->busy_lock);
	nvgpu_rwsem_init(&g->deterministic_busy);

	nvgpu_spinlock_init(&g->mc_enable_lock);

	nvgpu_mutex_init(&platform->railgate_lock);
	nvgpu_mutex_init(&g->dbg_sessions_lock);
	nvgpu_mutex_init(&g->client_lock);
	nvgpu_mutex_init(&g->poweron_lock);
	nvgpu_mutex_init(&g->poweroff_lock);

	l->regs_saved = l->regs;
	l->bar1_saved = l->bar1;

	g->emc3d_ratio = EMC3D_DEFAULT_RATIO;

	/* Set DMA parameters to allow larger sgt lists */
	dev->dma_parms = &l->dma_parms;
	dma_set_max_seg_size(dev, UINT_MAX);

	/* 34 bit mask - can be expanded for later chips is needed. */
	dma_set_mask(dev, DMA_BIT_MASK(34));
	dma_set_coherent_mask(dev, DMA_BIT_MASK(34));

	nvgpu_init_list_node(&g->pending_sema_waits);
	nvgpu_raw_spinlock_init(&g->pending_sema_waits_lock);

	nvgpu_init_list_node(&g->profiler_objects);

	nvgpu_init_list_node(&g->boardobj_head);
	nvgpu_init_list_node(&g->boardobjgrp_head);
}

static void nvgpu_init_gr_vars(struct gk20a *g)
{
	gk20a_init_gr(g);

	gk20a_dbg_info("total ram pages : %lu", totalram_pages);
	g->gr.max_comptag_mem = totalram_pages
				 >> (10 - (PAGE_SHIFT - 10));
}

static void nvgpu_init_timeout(struct gk20a *g)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));

	g->gr_idle_timeout_default = NVGPU_DEFAULT_GR_IDLE_TIMEOUT;
	if (nvgpu_platform_is_silicon(g))
		g->timeouts_enabled = true;
	else if (nvgpu_platform_is_fpga(g)) {
		g->gr_idle_timeout_default = GK20A_TIMEOUT_FPGA;
		g->timeouts_enabled = true;
	}
	g->ch_wdt_timeout_ms = platform->ch_wdt_timeout_ms;
}

static void nvgpu_init_timeslice(struct gk20a *g)
{
	g->runlist_interleave = true;

	g->timeslice_low_priority_us = 1300;
	g->timeslice_medium_priority_us = 2600;
	g->timeslice_high_priority_us = 5200;

	g->min_timeslice_us = 1000;
	g->max_timeslice_us = 50000;
}

static void nvgpu_init_pm_vars(struct gk20a *g)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));

	/*
	 * Set up initial power settings. For non-slicon platforms, disable
	 * power features and for silicon platforms, read from platform data
	 */
	g->slcg_enabled =
		nvgpu_platform_is_silicon(g) ? platform->enable_slcg : false;
	g->blcg_enabled =
		nvgpu_platform_is_silicon(g) ? platform->enable_blcg : false;
	g->elcg_enabled =
		nvgpu_platform_is_silicon(g) ? platform->enable_elcg : false;
	g->elpg_enabled =
		nvgpu_platform_is_silicon(g) ? platform->enable_elpg : false;
	g->aelpg_enabled =
		nvgpu_platform_is_silicon(g) ? platform->enable_aelpg : false;
	g->mscg_enabled =
		nvgpu_platform_is_silicon(g) ? platform->enable_mscg : false;
	g->can_elpg =
		nvgpu_platform_is_silicon(g) ? platform->can_elpg_init : false;

	__nvgpu_set_enabled(g, NVGPU_GPU_CAN_ELCG,
		nvgpu_platform_is_silicon(g) ? platform->can_elcg : false);
	__nvgpu_set_enabled(g, NVGPU_GPU_CAN_SLCG,
		nvgpu_platform_is_silicon(g) ? platform->can_slcg : false);
	__nvgpu_set_enabled(g, NVGPU_GPU_CAN_BLCG,
		nvgpu_platform_is_silicon(g) ? platform->can_blcg : false);

	g->default_pri_timeout = platform->default_pri_timeout;
	g->aggressive_sync_destroy = platform->aggressive_sync_destroy;
	g->aggressive_sync_destroy_thresh = platform->aggressive_sync_destroy_thresh;
	g->has_syncpoints = platform->has_syncpoints;
	g->has_cde = platform->has_cde;
	g->ptimer_src_freq = platform->ptimer_src_freq;
	g->support_pmu = support_gk20a_pmu(dev_from_gk20a(g));
	g->can_railgate = platform->can_railgate_init;
	g->railgate_delay = platform->railgate_delay_init;
	__nvgpu_set_enabled(g, NVGPU_PMU_PERFMON, platform->enable_perfmon);

	/* set default values to aelpg parameters */
	g->pmu.aelpg_param[0] = APCTRL_SAMPLING_PERIOD_PG_DEFAULT_US;
	g->pmu.aelpg_param[1] = APCTRL_MINIMUM_IDLE_FILTER_DEFAULT_US;
	g->pmu.aelpg_param[2] = APCTRL_MINIMUM_TARGET_SAVING_DEFAULT_US;
	g->pmu.aelpg_param[3] = APCTRL_POWER_BREAKEVEN_DEFAULT_US;
	g->pmu.aelpg_param[4] = APCTRL_CYCLES_PER_SAMPLE_MAX_DEFAULT;

	__nvgpu_set_enabled(g, NVGPU_SUPPORT_ASPM, !platform->disable_aspm);
}

static void nvgpu_init_vbios_vars(struct gk20a *g)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));

	__nvgpu_set_enabled(g, NVGPU_PMU_RUN_PREOS, platform->run_preos);
	g->vbios_min_version = platform->vbios_min_version;
}

static void  nvgpu_init_ltc_vars(struct gk20a *g)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));

	g->ltc_streamid = platform->ltc_streamid;
}

static void nvgpu_init_mm_vars(struct gk20a *g)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));

	g->mm.disable_bigpage = platform->disable_bigpage;
	__nvgpu_set_enabled(g, NVGPU_MM_HONORS_APERTURE,
			    platform->honors_aperture);
	__nvgpu_set_enabled(g, NVGPU_MM_UNIFIED_MEMORY,
			    platform->unified_memory);
	__nvgpu_set_enabled(g, NVGPU_MM_UNIFY_ADDRESS_SPACES,
			    platform->unify_address_spaces);

	nvgpu_mutex_init(&g->mm.tlb_lock);
	nvgpu_mutex_init(&g->mm.priv_lock);
}

int nvgpu_probe(struct gk20a *g,
		const char *debugfs_symlink,
		const char *interface_name,
		struct class *class)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int err = 0;

	nvgpu_init_vars(g);
	nvgpu_init_gr_vars(g);
	nvgpu_init_timeout(g);
	nvgpu_init_timeslice(g);
	nvgpu_init_pm_vars(g);
	nvgpu_init_vbios_vars(g);
	nvgpu_init_ltc_vars(g);

	/* Initialize the platform interface. */
	err = platform->probe(dev);
	if (err) {
		if (err == -EPROBE_DEFER)
			nvgpu_info(g, "platform probe failed");
		else
			nvgpu_err(g, "platform probe failed");
		return err;
	}

	nvgpu_init_mm_vars(g);

	/* platform probe can defer do user init only if probe succeeds */
	err = gk20a_user_init(dev, interface_name, class);
	if (err)
		return err;

	if (platform->late_probe) {
		err = platform->late_probe(dev);
		if (err) {
			nvgpu_err(g, "late probe failed");
			return err;
		}
	}

	nvgpu_create_sysfs(dev);
	gk20a_debug_init(g, debugfs_symlink);

	g->dbg_regops_tmp_buf = nvgpu_kzalloc(g, SZ_4K);
	if (!g->dbg_regops_tmp_buf) {
		nvgpu_err(g, "couldn't allocate regops tmp buf");
		return -ENOMEM;
	}
	g->dbg_regops_tmp_buf_ops =
		SZ_4K / sizeof(g->dbg_regops_tmp_buf[0]);

	g->remove_support = gk20a_remove_support;

	nvgpu_ref_init(&g->refcount);

	return 0;
}

/**
 * cyclic_delta - Returns delta of cyclic integers a and b.
 *
 * @a - First integer
 * @b - Second integer
 *
 * Note: if a is ahead of b, delta is positive.
 */
static int cyclic_delta(int a, int b)
{
	return a - b;
}

/**
 * nvgpu_wait_for_deferred_interrupts - Wait for interrupts to complete
 *
 * @g - The GPU to wait on.
 *
 * Waits until all interrupt handlers that have been scheduled to run have
 * completed.
 */
void nvgpu_wait_for_deferred_interrupts(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	int stall_irq_threshold = atomic_read(&l->hw_irq_stall_count);
	int nonstall_irq_threshold = atomic_read(&l->hw_irq_nonstall_count);

	/* wait until all stalling irqs are handled */
	NVGPU_COND_WAIT(&l->sw_irq_stall_last_handled_wq,
		   cyclic_delta(stall_irq_threshold,
				atomic_read(&l->sw_irq_stall_last_handled))
		   <= 0, 0);

	/* wait until all non-stalling irqs are handled */
	NVGPU_COND_WAIT(&l->sw_irq_nonstall_last_handled_wq,
		   cyclic_delta(nonstall_irq_threshold,
				atomic_read(&l->sw_irq_nonstall_last_handled))
		   <= 0, 0);
}

static void nvgpu_free_gk20a(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	kfree(l);
}

void nvgpu_init_gk20a(struct gk20a *g)
{
	g->free = nvgpu_free_gk20a;
}
