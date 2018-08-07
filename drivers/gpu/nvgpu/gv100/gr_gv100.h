/*
 * GV100 GPU GR
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVGPU_GR_GV100_H_
#define _NVGPU_GR_GV100_H_

void gr_gv100_bundle_cb_defaults(struct gk20a *g);
void gr_gv100_cb_size_default(struct gk20a *g);
void gr_gv100_set_gpc_tpc_mask(struct gk20a *g, u32 gpc_index);
int gr_gv100_init_sm_id_table(struct gk20a *g);
void gr_gv100_program_sm_id_numbering(struct gk20a *g,
					u32 gpc, u32 tpc, u32 smid);
int gr_gv100_load_smid_config(struct gk20a *g);
u32 gr_gv100_get_patch_slots(struct gk20a *g);
int gr_gv100_add_ctxsw_reg_pm_fbpa(struct gk20a *g,
				struct ctxsw_buf_offset_map_entry *map,
				struct aiv_list_gk20a *regs,
				u32 *count, u32 *offset,
				u32 max_cnt, u32 base,
				u32 num_fbpas, u32 stride, u32 mask);
int gr_gv100_add_ctxsw_reg_perf_pma(struct ctxsw_buf_offset_map_entry *map,
	struct aiv_list_gk20a *regs,
	u32 *count, u32 *offset,
	u32 max_cnt, u32 base, u32 mask);
void gr_gv100_split_fbpa_broadcast_addr(struct gk20a *g, u32 addr,
	u32 num_fbpas,
	u32 *priv_addr_table, u32 *t);
#endif
