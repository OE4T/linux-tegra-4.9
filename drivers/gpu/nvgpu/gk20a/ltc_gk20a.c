/*
 * GK20A L2
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

#include <trace/events/gk20a.h>
#include <nvgpu/timers.h>
#include <nvgpu/log.h>
#include <nvgpu/bug.h>

#include "gk20a.h"
#include "ltc_gk20a.h"

#include <nvgpu/hw/gk20a/hw_ltc_gk20a.h>

#include "ltc_common.c"

static int gk20a_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr)
{
	/* max memory size (MB) to cover */
	u32 max_size = gr->max_comptag_mem;
	/* one tag line covers 128KB */
	u32 max_comptag_lines = max_size << 3;

	u32 hw_max_comptag_lines =
		ltc_ltcs_ltss_cbc_ctrl3_clear_upper_bound_init_v();

	u32 cbc_param =
		gk20a_readl(g, ltc_ltcs_ltss_cbc_param_r());
	u32 comptags_per_cacheline =
		ltc_ltcs_ltss_cbc_param_comptags_per_cache_line_v(cbc_param);
	u32 slices_per_fbp =
		ltc_ltcs_ltss_cbc_param_slices_per_fbp_v(cbc_param);
	u32 cacheline_size =
		512 << ltc_ltcs_ltss_cbc_param_cache_line_size_v(cbc_param);

	u32 compbit_backing_size;

	int err;

	gk20a_dbg_fn("");

	if (max_comptag_lines == 0)
		return 0;

	if (max_comptag_lines > hw_max_comptag_lines)
		max_comptag_lines = hw_max_comptag_lines;

	/* no hybird fb */
	compbit_backing_size =
		DIV_ROUND_UP(max_comptag_lines, comptags_per_cacheline) *
		cacheline_size * slices_per_fbp * gr->num_fbps;

	/* aligned to 2KB * num_fbps */
	compbit_backing_size +=
		gr->num_fbps << ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	/* must be a multiple of 64KB */
	compbit_backing_size = roundup(compbit_backing_size, 64*1024);

	max_comptag_lines =
		(compbit_backing_size * comptags_per_cacheline) /
		cacheline_size * slices_per_fbp * gr->num_fbps;

	if (max_comptag_lines > hw_max_comptag_lines)
		max_comptag_lines = hw_max_comptag_lines;

	gk20a_dbg_info("compbit backing store size : %d",
		compbit_backing_size);
	gk20a_dbg_info("max comptag lines : %d",
		max_comptag_lines);

	if (g->is_fmodel)
		err = gk20a_ltc_alloc_phys_cbc(g, compbit_backing_size);
	else
		err = gk20a_ltc_alloc_virt_cbc(g, compbit_backing_size);

	if (err)
		return err;

	err = gk20a_comptag_allocator_init(&gr->comp_tags, max_comptag_lines);
	if (err)
		return err;

	gr->comptags_per_cacheline = comptags_per_cacheline;
	gr->slices_per_ltc = slices_per_fbp / g->ltc_count;
	gr->cacheline_size = cacheline_size;

	return 0;
}

static int gk20a_ltc_cbc_ctrl(struct gk20a *g, enum gk20a_cbc_op op,
			      u32 min, u32 max)
{
	int err = 0;
	struct gr_gk20a *gr = &g->gr;
	u32 fbp, slice, ctrl1, val, hw_op = 0;
	u32 slices_per_fbp =
		ltc_ltcs_ltss_cbc_param_slices_per_fbp_v(
			gk20a_readl(g, ltc_ltcs_ltss_cbc_param_r()));
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);
	u32 lts_stride = nvgpu_get_litter_value(g, GPU_LIT_LTS_STRIDE);

	gk20a_dbg_fn("");

	trace_gk20a_ltc_cbc_ctrl_start(g->name, op, min, max);

	if (gr->compbit_store.mem.size == 0)
		return 0;

	nvgpu_mutex_acquire(&g->mm.l2_op_lock);

	if (op == gk20a_cbc_op_clear) {
		gk20a_writel(g, ltc_ltcs_ltss_cbc_ctrl2_r(),
			     ltc_ltcs_ltss_cbc_ctrl2_clear_lower_bound_f(min));
		gk20a_writel(g, ltc_ltcs_ltss_cbc_ctrl3_r(),
			     ltc_ltcs_ltss_cbc_ctrl3_clear_upper_bound_f(max));
		hw_op = ltc_ltcs_ltss_cbc_ctrl1_clear_active_f();
	} else if (op == gk20a_cbc_op_clean) {
		hw_op = ltc_ltcs_ltss_cbc_ctrl1_clean_active_f();
	} else if (op == gk20a_cbc_op_invalidate) {
		hw_op = ltc_ltcs_ltss_cbc_ctrl1_invalidate_active_f();
	} else {
		BUG_ON(1);
	}

	gk20a_writel(g, ltc_ltcs_ltss_cbc_ctrl1_r(),
		     gk20a_readl(g, ltc_ltcs_ltss_cbc_ctrl1_r()) | hw_op);

	for (fbp = 0; fbp < gr->num_fbps; fbp++) {
		struct nvgpu_timeout timeout;

		nvgpu_timeout_init(g, &timeout, 200, NVGPU_TIMER_RETRY_TIMER);
		for (slice = 0; slice < slices_per_fbp; slice++) {


			ctrl1 = ltc_ltc0_lts0_cbc_ctrl1_r() +
				fbp * ltc_stride +
				slice * lts_stride;

			do {
				val = gk20a_readl(g, ctrl1);
				if (!(val & hw_op))
					break;
				nvgpu_udelay(5);

			} while (!nvgpu_timeout_expired(&timeout));

			if (nvgpu_timeout_peek_expired(&timeout)) {
				nvgpu_err(g, "comp tag clear timeout");
				err = -EBUSY;
				goto out;
			}
		}
	}
out:
	trace_gk20a_ltc_cbc_ctrl_done(g->name);
	nvgpu_mutex_release(&g->mm.l2_op_lock);
	return err;
}


static void gk20a_ltc_init_fs_state(struct gk20a *g)
{
	gk20a_dbg_info("initialize gk20a L2");

	g->max_ltc_count = g->ltc_count = 1;
}

static void gk20a_ltc_isr(struct gk20a *g)
{
	u32 intr;

	intr = gk20a_readl(g, ltc_ltc0_ltss_intr_r());
	nvgpu_err(g, "ltc: %08x", intr);
	gk20a_writel(g, ltc_ltc0_ltss_intr_r(), intr);
}

static int gk20a_determine_L2_size_bytes(struct gk20a *g)
{
	u32 lts_per_ltc;
	u32 ways;
	u32 sets;
	u32 bytes_per_line;
	u32 active_ltcs;
	u32 cache_size;

	u32 tmp;
	u32 active_sets_value;

	tmp = gk20a_readl(g, ltc_ltc0_lts0_tstg_cfg1_r());
	ways = hweight32(ltc_ltc0_lts0_tstg_cfg1_active_ways_v(tmp));

	active_sets_value = ltc_ltc0_lts0_tstg_cfg1_active_sets_v(tmp);
	if (active_sets_value == ltc_ltc0_lts0_tstg_cfg1_active_sets_all_v()) {
		sets = 64;
	} else if (active_sets_value ==
		 ltc_ltc0_lts0_tstg_cfg1_active_sets_half_v()) {
		sets = 32;
	} else if (active_sets_value ==
		 ltc_ltc0_lts0_tstg_cfg1_active_sets_quarter_v()) {
		sets = 16;
	} else {
		nvgpu_err(g,
			"Unknown constant %u for active sets",
		       (unsigned)active_sets_value);
		sets = 0;
	}

	active_ltcs = g->gr.num_fbps;

	/* chip-specific values */
	lts_per_ltc = 1;
	bytes_per_line = 128;
	cache_size = active_ltcs * lts_per_ltc * ways * sets * bytes_per_line;

	return cache_size;
}

void gk20a_init_ltc(struct gpu_ops *gops)
{
	gops->ltc.determine_L2_size_bytes = gk20a_determine_L2_size_bytes;
	gops->ltc.init_comptags = gk20a_ltc_init_comptags;
	gops->ltc.cbc_ctrl = gk20a_ltc_cbc_ctrl;
	gops->ltc.set_zbc_color_entry = gk20a_ltc_set_zbc_color_entry;
	gops->ltc.set_zbc_depth_entry = gk20a_ltc_set_zbc_depth_entry;
	gops->ltc.init_cbc = gk20a_ltc_init_cbc;
#ifdef CONFIG_DEBUG_FS
	gops->ltc.sync_debugfs = gk20a_ltc_sync_debugfs;
#endif
	gops->ltc.init_fs_state = gk20a_ltc_init_fs_state;
	gops->ltc.isr = gk20a_ltc_isr;
}
