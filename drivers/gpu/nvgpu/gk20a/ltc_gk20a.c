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
#include <nvgpu/enabled.h>
#include <nvgpu/dma.h>

#include "gk20a.h"
#include "gr_gk20a.h"
#include "ltc_gk20a.h"

#include <nvgpu/hw/gk20a/hw_ltc_gk20a.h>


/* Non HW reg dependent stuff: */

int gk20a_ltc_alloc_phys_cbc(struct gk20a *g, size_t compbit_backing_size)
{
	struct gr_gk20a *gr = &g->gr;

	return nvgpu_dma_alloc_flags_sys(g, NVGPU_DMA_FORCE_CONTIGUOUS,
				    compbit_backing_size,
				    &gr->compbit_store.mem);
}

int gk20a_ltc_alloc_virt_cbc(struct gk20a *g, size_t compbit_backing_size)
{
	struct gr_gk20a *gr = &g->gr;

	return nvgpu_dma_alloc_flags_sys(g, NVGPU_DMA_NO_KERNEL_MAPPING,
				    compbit_backing_size,
				    &gr->compbit_store.mem);
}

/* HW reg dependent stuff: */
int gk20a_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr)
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

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
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

int gk20a_ltc_cbc_ctrl(struct gk20a *g, enum gk20a_cbc_op op,
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


void gk20a_ltc_init_fs_state(struct gk20a *g)
{
	gk20a_dbg_info("initialize gk20a L2");

	g->max_ltc_count = g->ltc_count = 1;
}

void gk20a_ltc_isr(struct gk20a *g)
{
	u32 intr;

	intr = gk20a_readl(g, ltc_ltc0_ltss_intr_r());
	nvgpu_err(g, "ltc: %08x", intr);
	gk20a_writel(g, ltc_ltc0_ltss_intr_r(), intr);
}

int gk20a_determine_L2_size_bytes(struct gk20a *g)
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

/*
 * Sets the ZBC color for the passed index.
 */
void gk20a_ltc_set_zbc_color_entry(struct gk20a *g,
					  struct zbc_entry *color_val,
					  u32 index)
{
	u32 i;
	u32 real_index = index + GK20A_STARTOF_ZBC_TABLE;

	gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_index_r(),
		     ltc_ltcs_ltss_dstg_zbc_index_address_f(real_index));

	for (i = 0;
	     i < ltc_ltcs_ltss_dstg_zbc_color_clear_value__size_1_v(); i++) {
		gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_color_clear_value_r(i),
			     color_val->color_l2[i]);
	}
	gk20a_readl(g, ltc_ltcs_ltss_dstg_zbc_index_r());
}

/*
 * Sets the ZBC depth for the passed index.
 */
void gk20a_ltc_set_zbc_depth_entry(struct gk20a *g,
					  struct zbc_entry *depth_val,
					  u32 index)
{
	u32 real_index = index + GK20A_STARTOF_ZBC_TABLE;

	gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_index_r(),
		     ltc_ltcs_ltss_dstg_zbc_index_address_f(real_index));

	gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_depth_clear_value_r(),
		     depth_val->depth);

	gk20a_readl(g, ltc_ltcs_ltss_dstg_zbc_index_r());
}

void gk20a_ltc_init_cbc(struct gk20a *g, struct gr_gk20a *gr)
{
	u32 max_size = gr->max_comptag_mem;
	u32 max_comptag_lines = max_size << 3;

	u32 compbit_base_post_divide;
	u64 compbit_base_post_multiply64;
	u64 compbit_store_iova;
	u64 compbit_base_post_divide64;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		compbit_store_iova = gk20a_mem_phys(&gr->compbit_store.mem);
	else
		compbit_store_iova = g->ops.mm.get_iova_addr(g,
				gr->compbit_store.mem.priv.sgt->sgl, 0);

	compbit_base_post_divide64 = compbit_store_iova >>
		ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	do_div(compbit_base_post_divide64, g->ltc_count);
	compbit_base_post_divide = u64_lo32(compbit_base_post_divide64);

	compbit_base_post_multiply64 = ((u64)compbit_base_post_divide *
		g->ltc_count) << ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	if (compbit_base_post_multiply64 < compbit_store_iova)
		compbit_base_post_divide++;

	/* Bug 1477079 indicates sw adjustment on the posted divided base. */
	if (g->ops.ltc.cbc_fix_config)
		compbit_base_post_divide =
			g->ops.ltc.cbc_fix_config(g, compbit_base_post_divide);

	gk20a_writel(g, ltc_ltcs_ltss_cbc_base_r(),
		compbit_base_post_divide);

	gk20a_dbg(gpu_dbg_info | gpu_dbg_map_v | gpu_dbg_pte,
		   "compbit base.pa: 0x%x,%08x cbc_base:0x%08x\n",
		   (u32)(compbit_store_iova >> 32),
		   (u32)(compbit_store_iova & 0xffffffff),
		   compbit_base_post_divide);

	gr->compbit_store.base_hw = compbit_base_post_divide;

	g->ops.ltc.cbc_ctrl(g, gk20a_cbc_op_invalidate,
			    0, max_comptag_lines - 1);

}

#ifdef CONFIG_DEBUG_FS
void gk20a_ltc_sync_debugfs(struct gk20a *g)
{
	u32 reg_f = ltc_ltcs_ltss_tstg_set_mgmt_2_l2_bypass_mode_enabled_f();

	nvgpu_spinlock_acquire(&g->debugfs_lock);
	if (g->mm.ltc_enabled != g->mm.ltc_enabled_debug) {
		u32 reg = gk20a_readl(g, ltc_ltcs_ltss_tstg_set_mgmt_2_r());

		if (g->mm.ltc_enabled_debug)
			/* bypass disabled (normal caching ops)*/
			reg &= ~reg_f;
		else
			/* bypass enabled (no caching) */
			reg |= reg_f;

		gk20a_writel(g, ltc_ltcs_ltss_tstg_set_mgmt_2_r(), reg);
		g->mm.ltc_enabled = g->mm.ltc_enabled_debug;
	}
	nvgpu_spinlock_release(&g->debugfs_lock);
}
#endif
