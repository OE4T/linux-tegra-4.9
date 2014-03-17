/*
 * drivers/video/tegra/host/gk20a/ltc_gk20a.c
 *
 * GK20A Graphics
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>

#include "hw_ltc_gk20a.h"
#include "hw_proj_gk20a.h"

#include "ltc_common.c"

static int gk20a_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr)
{
	struct device *d = dev_from_gk20a(g);
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t iova;

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

	gk20a_dbg_fn("");

	if (max_comptag_lines == 0) {
		gr->compbit_store.size = 0;
		return 0;
	}

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

	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
	gr->compbit_store.size = compbit_backing_size;
	gr->compbit_store.pages = dma_alloc_attrs(d, gr->compbit_store.size,
					&iova, GFP_KERNEL, &attrs);
	if (!gr->compbit_store.pages) {
		gk20a_err(dev_from_gk20a(g), "failed to allocate"
			   "backing store for compbit : size %d",
			   compbit_backing_size);
		return -ENOMEM;
	}
	gr->compbit_store.base_iova = iova;

	gk20a_allocator_init(&gr->comp_tags, "comptag",
			      1, /* start */
			      max_comptag_lines - 1, /* length*/
			      1); /* align */

	return 0;
}

static int gk20a_ltc_clear_comptags(struct gk20a *g, u32 min, u32 max)
{
	struct gr_gk20a *gr = &g->gr;
	u32 fbp, slice, ctrl1, val;
	unsigned long end_jiffies = jiffies +
		msecs_to_jiffies(gk20a_get_gr_idle_timeout(g));
	u32 delay = GR_IDLE_CHECK_DEFAULT;
	u32 slices_per_fbp =
		ltc_ltcs_ltss_cbc_param_slices_per_fbp_v(
			gk20a_readl(g, ltc_ltcs_ltss_cbc_param_r()));

	gk20a_dbg_fn("");

	if (gr->compbit_store.size == 0)
		return 0;

	gk20a_writel(g, ltc_ltcs_ltss_cbc_ctrl2_r(),
		     ltc_ltcs_ltss_cbc_ctrl2_clear_lower_bound_f(min));
	gk20a_writel(g, ltc_ltcs_ltss_cbc_ctrl3_r(),
		     ltc_ltcs_ltss_cbc_ctrl3_clear_upper_bound_f(max));
	gk20a_writel(g, ltc_ltcs_ltss_cbc_ctrl1_r(),
		     gk20a_readl(g, ltc_ltcs_ltss_cbc_ctrl1_r()) |
		     ltc_ltcs_ltss_cbc_ctrl1_clear_active_f());

	for (fbp = 0; fbp < gr->num_fbps; fbp++) {
		for (slice = 0; slice < slices_per_fbp; slice++) {

			delay = GR_IDLE_CHECK_DEFAULT;

			ctrl1 = ltc_ltc0_lts0_cbc_ctrl1_r() +
				fbp * proj_ltc_stride_v() +
				slice * proj_lts_stride_v();

			do {
				val = gk20a_readl(g, ctrl1);
				if (ltc_ltcs_ltss_cbc_ctrl1_clear_v(val) !=
				    ltc_ltcs_ltss_cbc_ctrl1_clear_active_v())
					break;

				usleep_range(delay, delay * 2);
				delay = min_t(u32, delay << 1,
					GR_IDLE_CHECK_MAX);

			} while (time_before(jiffies, end_jiffies) ||
					!tegra_platform_is_silicon());

			if (!time_before(jiffies, end_jiffies)) {
				gk20a_err(dev_from_gk20a(g),
					   "comp tag clear timeout\n");
				return -EBUSY;
			}
		}
	}

	return 0;
}


#ifdef CONFIG_DEBUG_FS
static void gk20a_ltc_sync_debugfs(struct gk20a *g)
{
	u32 reg_f = ltc_ltcs_ltss_tstg_set_mgmt_2_l2_bypass_mode_enabled_f();

	spin_lock(&g->debugfs_lock);
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
	spin_unlock(&g->debugfs_lock);
}
#endif

static void gk20a_ltc_init_fs_state(struct gk20a *g)
{
	gk20a_dbg_info("initialize gk20a L2");

	g->max_ltc_count = g->ltc_count = 1;
}

void gk20a_init_ltc(struct gpu_ops *gops)
{
	gops->ltc.determine_L2_size_bytes = gk20a_determine_L2_size_bytes;
	gops->ltc.set_max_ways_evict_last = gk20a_ltc_set_max_ways_evict_last;
	gops->ltc.init_comptags = gk20a_ltc_init_comptags;
	gops->ltc.clear_comptags = gk20a_ltc_clear_comptags;
	gops->ltc.set_zbc_color_entry = gk20a_ltc_set_zbc_color_entry;
	gops->ltc.set_zbc_depth_entry = gk20a_ltc_set_zbc_depth_entry;
	gops->ltc.clear_zbc_color_entry = gk20a_ltc_clear_zbc_color_entry;
	gops->ltc.clear_zbc_depth_entry = gk20a_ltc_clear_zbc_depth_entry;
	gops->ltc.init_zbc = gk20a_ltc_init_zbc;
	gops->ltc.init_cbc = gk20a_ltc_init_cbc;
#ifdef CONFIG_DEBUG_FS
	gops->ltc.sync_debugfs = gk20a_ltc_sync_debugfs;
#endif
	gops->ltc.elpg_flush = gk20a_mm_g_elpg_flush_locked;
	gops->ltc.init_fs_state = gk20a_ltc_init_fs_state;
}
