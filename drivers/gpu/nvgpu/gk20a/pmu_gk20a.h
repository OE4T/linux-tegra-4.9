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

#include <linux/version.h>
#include <nvgpu/flcnif_cmn.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include <nvgpu/pmu.h>

struct nvgpu_firmware;

#define ZBC_MASK(i)			(~(~(0) << ((i)+1)) & 0xfffe)

#define APP_VERSION_NC_3	22204331
#define APP_VERSION_NC_2	20429989
#define APP_VERSION_NC_1	20313802
#define APP_VERSION_NC_0	20360931
#define APP_VERSION_GM206	20652057
#define APP_VERSION_NV_GPU	21307569
#define APP_VERSION_NV_GPU_1	21308030
#define APP_VERSION_GM20B_5 20490253
#define APP_VERSION_GM20B_4 19008461
#define APP_VERSION_GM20B_3 18935575
#define APP_VERSION_GM20B_2 18694072
#define APP_VERSION_GM20B_1 18547257
#define APP_VERSION_GM20B 17615280
#define APP_VERSION_3 18357968
#define APP_VERSION_2 18542378
#define APP_VERSION_1 17997577 /*Obsolete this once 18357968 gets in*/
#define APP_VERSION_0 16856675

/*Fuse defines*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
#define FUSE_GCPLEX_CONFIG_FUSE_0           0x2C8
#endif

#define PMU_PGENG_GR_BUFFER_IDX_INIT	(0)
#define PMU_PGENG_GR_BUFFER_IDX_ZBC	(1)
#define PMU_PGENG_GR_BUFFER_IDX_FECS	(2)

struct pmu_payload {
	struct {
		void *buf;
		u32 offset;
		u32 size;
		u32 fb_size;
	} in, out;
};

struct pmu_surface {
	struct nvgpu_mem vidmem_desc;
	struct nvgpu_mem sysmem_desc;
	struct flcn_mem_desc_v0 params;
};

/*PG defines used by nvpgu-pmu*/
struct pmu_pg_stats_data {
	u32 gating_cnt;
	u32 ingating_time;
	u32 ungating_time;
	u32 avg_entry_latency_us;
	u32 avg_exit_latency_us;
};

#define PMU_PG_IDLE_THRESHOLD_SIM		1000
#define PMU_PG_POST_POWERUP_IDLE_THRESHOLD_SIM	4000000
/* TBD: QT or else ? */
#define PMU_PG_IDLE_THRESHOLD			15000
#define PMU_PG_POST_POWERUP_IDLE_THRESHOLD	1000000

#define PMU_PG_LPWR_FEATURE_RPPG 0x0
#define PMU_PG_LPWR_FEATURE_MSCG 0x1

/* state transition :
    OFF => [OFF_ON_PENDING optional] => ON_PENDING => ON => OFF
    ON => OFF is always synchronized */
#define PMU_ELPG_STAT_OFF		0   /* elpg is off */
#define PMU_ELPG_STAT_ON		1   /* elpg is on */
#define PMU_ELPG_STAT_ON_PENDING	2   /* elpg is off, ALLOW cmd has been sent, wait for ack */
#define PMU_ELPG_STAT_OFF_PENDING	3   /* elpg is on, DISALLOW cmd has been sent, wait for ack */
#define PMU_ELPG_STAT_OFF_ON_PENDING	4   /* elpg is off, caller has requested on, but ALLOW
					       cmd hasn't been sent due to ENABLE_ALLOW delay */

#define PG_REQUEST_TYPE_GLOBAL 0x0
#define PG_REQUEST_TYPE_PSTATE 0x1

#define PMU_MSCG_DISABLED 0
#define PMU_MSCG_ENABLED 1

/* Default Sampling Period of AELPG */
#define APCTRL_SAMPLING_PERIOD_PG_DEFAULT_US                    (1000000)

/* Default values of APCTRL parameters */
#define APCTRL_MINIMUM_IDLE_FILTER_DEFAULT_US                   (100)
#define APCTRL_MINIMUM_TARGET_SAVING_DEFAULT_US                 (10000)
#define APCTRL_POWER_BREAKEVEN_DEFAULT_US                       (2000)
#define APCTRL_CYCLES_PER_SAMPLE_MAX_DEFAULT                    (200)
/*PG defines used by nvpgu-pmu*/

int gk20a_init_pmu_support(struct gk20a *g);
int gk20a_init_pmu_bind_fecs(struct gk20a *g);

void gk20a_pmu_isr(struct gk20a *g);

/* send a cmd to pmu */
int gk20a_pmu_cmd_post(struct gk20a *g, struct pmu_cmd *cmd, struct pmu_msg *msg,
		struct pmu_payload *payload, u32 queue_id,
		pmu_callback callback, void* cb_param,
		u32 *seq_desc, unsigned long timeout);

int gk20a_pmu_enable_elpg(struct gk20a *g);
int gk20a_pmu_disable_elpg(struct gk20a *g);
int gk20a_pmu_pg_global_enable(struct gk20a *g, u32 enable_pg);

u32 gk20a_pmu_pg_engines_list(struct gk20a *g);
u32 gk20a_pmu_pg_feature_list(struct gk20a *g, u32 pg_engine_id);

void gk20a_pmu_save_zbc(struct gk20a *g, u32 entries);

int gk20a_pmu_perfmon_enable(struct gk20a *g, bool enable);

int pmu_mutex_acquire(struct nvgpu_pmu *pmu, u32 id, u32 *token);
int pmu_mutex_release(struct nvgpu_pmu *pmu, u32 id, u32 *token);
int gk20a_pmu_destroy(struct gk20a *g);
int gk20a_pmu_load_norm(struct gk20a *g, u32 *load);
int gk20a_pmu_load_update(struct gk20a *g);
void gk20a_pmu_reset_load_counters(struct gk20a *g);
void gk20a_pmu_get_load_counters(struct gk20a *g, u32 *busy_cycles,
		u32 *total_cycles);
void gk20a_init_pmu_ops(struct gpu_ops *gops);

void pmu_copy_to_dmem(struct nvgpu_pmu *pmu,
		u32 dst, u8 *src, u32 size, u8 port);
void pmu_copy_from_dmem(struct nvgpu_pmu *pmu,
		u32 src, u8 *dst, u32 size, u8 port);
int pmu_reset(struct nvgpu_pmu *pmu);
int pmu_bootstrap(struct nvgpu_pmu *pmu);
int gk20a_init_pmu(struct nvgpu_pmu *pmu);
void pmu_dump_falcon_stats(struct nvgpu_pmu *pmu);
void gk20a_remove_pmu_support(struct nvgpu_pmu *pmu);
void pmu_seq_init(struct nvgpu_pmu *pmu);

int gk20a_init_pmu(struct nvgpu_pmu *pmu);

int gk20a_pmu_ap_send_command(struct gk20a *g,
		union pmu_ap_cmd *p_ap_cmd, bool b_block);
int gk20a_aelpg_init(struct gk20a *g);
int gk20a_aelpg_init_and_enable(struct gk20a *g, u8 ctrl_id);
void pmu_enable_irq(struct nvgpu_pmu *pmu, bool enable);
int pmu_wait_message_cond(struct nvgpu_pmu *pmu, u32 timeout_ms,
				 u32 *var, u32 val);
void pmu_handle_fecs_boot_acr_msg(struct gk20a *g, struct pmu_msg *msg,
				void *param, u32 handle, u32 status);
void gk20a_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data);
int gk20a_pmu_reset(struct gk20a *g);
int pmu_idle(struct nvgpu_pmu *pmu);
int pmu_enable_hw(struct nvgpu_pmu *pmu, bool enable);

void gk20a_pmu_surface_free(struct gk20a *g, struct nvgpu_mem *mem);
void gk20a_pmu_surface_describe(struct gk20a *g, struct nvgpu_mem *mem,
		struct flcn_mem_desc_v0 *fb);
int gk20a_pmu_vidmem_surface_alloc(struct gk20a *g, struct nvgpu_mem *mem,
		u32 size);
int gk20a_pmu_sysmem_surface_alloc(struct gk20a *g, struct nvgpu_mem *mem,
		u32 size);
int gk20a_pmu_get_pg_stats(struct gk20a *g,
		u32 pg_engine_id, struct pmu_pg_stats_data *pg_stat_data);
bool nvgpu_find_hex_in_string(char *strings, struct gk20a *g, u32 *hex_pos);

int nvgpu_pmu_perfmon_start_sampling(struct nvgpu_pmu *pmu);
int nvgpu_pmu_perfmon_stop_sampling(struct nvgpu_pmu *pmu);

#endif /*__PMU_GK20A_H__*/
