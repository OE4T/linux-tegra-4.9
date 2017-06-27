/*
 *
 * Volta GPU series copy engine
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program.
 */
#ifndef __CE_GV11B_H__
#define __CE_GV11B_H__

struct gk20a;

void gv11b_ce_mthd_buffer_fault_in_bar2_fault(struct gk20a *g);
u32 gv11b_ce_get_num_lce(struct gk20a *g);
u32 gv11b_ce_get_num_pce(struct gk20a *g);
void gv11b_ce_isr(struct gk20a *g, u32 inst_id, u32 pri_base);

#endif /*__CE2_GV11B_H__*/
