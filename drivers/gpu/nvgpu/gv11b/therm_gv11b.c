/*
 * GV11B Therm
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"

#include <nvgpu/soc.h>

#include <nvgpu/hw/gv11b/hw_therm_gv11b.h>

int gv11b_elcg_init_idle_filters(struct gk20a *g)
{
	u32 gate_ctrl, idle_filter;
	u32 engine_id;
	u32 active_engine_id = 0;
	struct fifo_gk20a *f = &g->fifo;

	if (nvgpu_platform_is_simulation(g))
		return 0;

	gk20a_dbg_info("init clock/power gate reg");

	for (engine_id = 0; engine_id < f->num_engines; engine_id++) {
		active_engine_id = f->active_engines_list[engine_id];

		gate_ctrl = gk20a_readl(g, therm_gate_ctrl_r(active_engine_id));
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_idle_filt_exp_m(),
			therm_gate_ctrl_eng_idle_filt_exp__prod_f());
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_idle_filt_mant_m(),
			therm_gate_ctrl_eng_idle_filt_mant__prod_f());
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_delay_before_m(),
			therm_gate_ctrl_eng_delay_before__prod_f());
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_delay_after_m(),
				therm_gate_ctrl_eng_delay_after__prod_f());
		gk20a_writel(g, therm_gate_ctrl_r(active_engine_id), gate_ctrl);
	}

	idle_filter = gk20a_readl(g, therm_fecs_idle_filter_r());
	idle_filter = set_field(idle_filter,
			therm_fecs_idle_filter_value_m(),
			therm_fecs_idle_filter_value__prod_f());
	gk20a_writel(g, therm_fecs_idle_filter_r(), idle_filter);

	idle_filter = gk20a_readl(g, therm_hubmmu_idle_filter_r());
	idle_filter = set_field(idle_filter,
			therm_hubmmu_idle_filter_value_m(),
			therm_hubmmu_idle_filter_value__prod_f());
	gk20a_writel(g, therm_hubmmu_idle_filter_r(), idle_filter);

	return 0;
}
