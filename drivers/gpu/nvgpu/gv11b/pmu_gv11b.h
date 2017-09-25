/*
 * GV11B PMU
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __PMU_GV11B_H_
#define __PMU_GV11B_H_

struct gk20a;

bool gv11b_is_pmu_supported(struct gk20a *g);
int gv11b_pmu_bootstrap(struct nvgpu_pmu *pmu);
int gv11b_pg_gr_init(struct gk20a *g, u32 pg_engine_id);
int gv11b_pg_set_subfeature_mask(struct gk20a *g, u32 pg_engine_id);
bool gv11b_is_lazy_bootstrap(u32 falcon_id);
bool gv11b_is_priv_load(u32 falcon_id);

#endif /*__PMU_GV11B_H_*/
