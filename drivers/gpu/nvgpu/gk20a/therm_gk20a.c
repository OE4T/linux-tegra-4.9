/*
 * GK20A Therm
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

#include <nvgpu/enabled.h>

#include "gk20a.h"

#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_therm_gk20a.h>

static int gk20a_init_therm_reset_enable_hw(struct gk20a *g)
{
	return 0;
}

static int gk20a_init_therm_setup_sw(struct gk20a *g)
{
	return 0;
}

int gk20a_init_therm_support(struct gk20a *g)
{
	u32 err;

	gk20a_dbg_fn("");

	err = gk20a_init_therm_reset_enable_hw(g);
	if (err)
		return err;

	err = gk20a_init_therm_setup_sw(g);
	if (err)
		return err;

	if (g->ops.therm.init_therm_setup_hw)
		err = g->ops.therm.init_therm_setup_hw(g);
	if (err)
		return err;

#ifdef CONFIG_DEBUG_FS
	if (g->ops.therm.therm_debugfs_init)
	    g->ops.therm.therm_debugfs_init(g);
#endif

	return err;
}

int gk20a_elcg_init_idle_filters(struct gk20a *g)
{
	u32 gate_ctrl, idle_filter;
	u32 engine_id;
	u32 active_engine_id = 0;
	struct fifo_gk20a *f = &g->fifo;

	gk20a_dbg_fn("");

	for (engine_id = 0; engine_id < f->num_engines; engine_id++) {
		active_engine_id = f->active_engines_list[engine_id];
		gate_ctrl = gk20a_readl(g, therm_gate_ctrl_r(active_engine_id));

		if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
			gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_delay_after_m(),
				therm_gate_ctrl_eng_delay_after_f(4));
		}

		/* 2 * (1 << 9) = 1024 clks */
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_idle_filt_exp_m(),
			therm_gate_ctrl_eng_idle_filt_exp_f(9));
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_idle_filt_mant_m(),
			therm_gate_ctrl_eng_idle_filt_mant_f(2));
		gk20a_writel(g, therm_gate_ctrl_r(active_engine_id), gate_ctrl);
	}

	/* default fecs_idle_filter to 0 */
	idle_filter = gk20a_readl(g, therm_fecs_idle_filter_r());
	idle_filter &= ~therm_fecs_idle_filter_value_m();
	gk20a_writel(g, therm_fecs_idle_filter_r(), idle_filter);
	/* default hubmmu_idle_filter to 0 */
	idle_filter = gk20a_readl(g, therm_hubmmu_idle_filter_r());
	idle_filter &= ~therm_hubmmu_idle_filter_value_m();
	gk20a_writel(g, therm_hubmmu_idle_filter_r(), idle_filter);

	gk20a_dbg_fn("done");
	return 0;
}
