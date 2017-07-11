/*
 * drivers/video/tegra/host/gk20a/pmu_gk20a.h
 *
 * GK20A PMU (aka. gPMU outside gk20a context)
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef __PMU_GK20A_H__
#define __PMU_GK20A_H__

#include <nvgpu/flcnif_cmn.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include <nvgpu/pmu.h>

struct nvgpu_firmware;

#define ZBC_MASK(i)			(~(~(0) << ((i)+1)) & 0xfffe)

bool gk20a_pmu_is_interrupted(struct nvgpu_pmu *pmu);
void gk20a_pmu_isr(struct gk20a *g);

u32 gk20a_pmu_pg_engines_list(struct gk20a *g);
u32 gk20a_pmu_pg_feature_list(struct gk20a *g, u32 pg_engine_id);

void gk20a_pmu_save_zbc(struct gk20a *g, u32 entries);

void gk20a_pmu_init_perfmon_counter(struct gk20a *g);

void gk20a_pmu_pg_idle_counter_config(struct gk20a *g, u32 pg_engine_id);

int gk20a_pmu_mutex_acquire(struct nvgpu_pmu *pmu, u32 id, u32 *token);
int gk20a_pmu_mutex_release(struct nvgpu_pmu *pmu, u32 id, u32 *token);

int gk20a_pmu_queue_head(struct nvgpu_pmu *pmu, struct pmu_queue *queue,
			u32 *head, bool set);
int gk20a_pmu_queue_tail(struct nvgpu_pmu *pmu, struct pmu_queue *queue,
			u32 *tail, bool set);
void gk20a_pmu_msgq_tail(struct nvgpu_pmu *pmu, u32 *tail, bool set);

u32 gk20a_pmu_read_idle_counter(struct gk20a *g, u32 counter_id);
void gk20a_pmu_reset_idle_counter(struct gk20a *g, u32 counter_id);

int gk20a_init_pmu_setup_hw1(struct gk20a *g);
void gk20a_write_dmatrfbase(struct gk20a *g, u32 addr);
bool gk20a_is_pmu_supported(struct gk20a *g);

int pmu_bootstrap(struct nvgpu_pmu *pmu);

void gk20a_pmu_dump_elpg_stats(struct nvgpu_pmu *pmu);
void gk20a_pmu_dump_falcon_stats(struct nvgpu_pmu *pmu);

void pmu_enable_irq(struct nvgpu_pmu *pmu, bool enable);
int pmu_wait_message_cond(struct nvgpu_pmu *pmu, u32 timeout_ms,
				 u32 *var, u32 val);
void pmu_handle_fecs_boot_acr_msg(struct gk20a *g, struct pmu_msg *msg,
				void *param, u32 handle, u32 status);
void gk20a_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data);
bool gk20a_pmu_is_engine_in_reset(struct gk20a *g);
int gk20a_pmu_engine_reset(struct gk20a *g, bool do_reset);

#endif /*__PMU_GK20A_H__*/
