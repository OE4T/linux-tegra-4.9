/*
 * GM20B Fifo
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

#ifndef _NVHOST_GM20B_FIFO
#define _NVHOST_GM20B_FIFO
struct gk20a;

void channel_gm20b_bind(struct channel_gk20a *c);
void gm20b_fifo_trigger_mmu_fault(struct gk20a *g,
		unsigned long engine_ids);
u32 gm20b_fifo_get_num_fifos(struct gk20a *g);
void gm20b_device_info_data_parse(struct gk20a *g,
						u32 table_entry, u32 *inst_id,
						u32 *pri_base, u32 *fault_id);
void gm20b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f);
void gm20b_fifo_tsg_verify_status_ctx_reload(struct channel_gk20a *ch);

#endif
