/*
 * GP10B Fifo
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef FIFO_GP10B_H
#define FIFO_GP10B_H

struct gpu_ops;
struct channel_gk20a;
struct fifo_gk20a;
struct mmu_fault_info;

int channel_gp10b_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries,
			unsigned long acquire_timeout, u32 flags);
u32 gp10b_fifo_get_pbdma_signature(struct gk20a *g);
int gp10b_fifo_resetup_ramfc(struct channel_gk20a *c);
int gp10b_fifo_engine_enum_from_type(struct gk20a *g, u32 engine_type,
					u32 *inst_id);
void gp10b_device_info_data_parse(struct gk20a *g, u32 table_entry,
				u32 *inst_id, u32 *pri_base, u32 *fault_id);
void gp10b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f);
void gp10b_fifo_get_mmu_fault_info(struct gk20a *g, u32 mmu_fault_id,
	struct mmu_fault_info *mmfault);
int channel_gp10b_commit_userd(struct channel_gk20a *c);

#endif
