/*
 * GV11B PMU
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
 */

#include <linux/delay.h>	/* for udelay */
#include <linux/clk.h>

#include <soc/tegra/fuse.h>

#include <nvgpu/pmu.h>
#include <nvgpu/falcon.h>
#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"

#include "gp10b/pmu_gp10b.h"
#include "gp106/pmu_gp106.h"

#include "pmu_gv11b.h"

#include <nvgpu/hw/gv11b/hw_pwr_gv11b.h>

#define gv11b_dbg_pmu(fmt, arg...) \
	gk20a_dbg(gpu_dbg_pmu, fmt, ##arg)

#define ALIGN_4KB     12

bool gv11b_is_pmu_supported(struct gk20a *g)
{
	return true;
}

int gv11b_pmu_bootstrap(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct mm_gk20a *mm = &g->mm;
	struct pmu_ucode_desc *desc = pmu->desc;
	u64 addr_code_lo, addr_data_lo, addr_load_lo;
	u64 addr_code_hi, addr_data_hi, addr_load_hi;
	u32 i, blocks, addr_args;

	gk20a_dbg_fn("");

	gk20a_writel(g, pwr_falcon_itfen_r(),
		gk20a_readl(g, pwr_falcon_itfen_r()) |
		pwr_falcon_itfen_ctxen_enable_f());

	gk20a_writel(g, pwr_pmu_new_instblk_r(),
		pwr_pmu_new_instblk_ptr_f(
		gk20a_mm_inst_block_addr(g, &mm->pmu.inst_block) >> ALIGN_4KB)
		| pwr_pmu_new_instblk_valid_f(1)
		| pwr_pmu_new_instblk_target_sys_ncoh_f());

	/* TBD: load all other surfaces */
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_size(
		pmu, GK20A_PMU_TRACE_BUFSIZE);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_base(pmu);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_idx(
		pmu, GK20A_PMU_DMAIDX_VIRT);

	g->ops.pmu_ver.set_pmu_cmdline_args_cpu_freq(pmu,
		g->ops.clk.get_rate(g, CTRL_CLK_DOMAIN_PWRCLK));

	addr_args = (pwr_falcon_hwcfg_dmem_size_v(
		gk20a_readl(g, pwr_falcon_hwcfg_r()))
			<< GK20A_PMU_DMEM_BLKSIZE2) -
		g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu);

	nvgpu_flcn_copy_to_dmem(pmu->flcn, addr_args,
			(u8 *)(g->ops.pmu_ver.get_pmu_cmdline_args_ptr(pmu)),
			g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu), 0);

	gk20a_writel(g, pwr_falcon_dmemc_r(0),
		pwr_falcon_dmemc_offs_f(0) |
		pwr_falcon_dmemc_blk_f(0)  |
		pwr_falcon_dmemc_aincw_f(1));

	addr_code_lo = u64_lo32((pmu->ucode.gpu_va +
			desc->app_start_offset +
			desc->app_resident_code_offset) >> 8);

	addr_code_hi = u64_hi32((pmu->ucode.gpu_va +
			desc->app_start_offset +
			desc->app_resident_code_offset) >> 8);
	addr_data_lo = u64_lo32((pmu->ucode.gpu_va +
			desc->app_start_offset +
			desc->app_resident_data_offset) >> 8);
	addr_data_hi = u64_hi32((pmu->ucode.gpu_va +
			desc->app_start_offset +
			desc->app_resident_data_offset) >> 8);
	addr_load_lo = u64_lo32((pmu->ucode.gpu_va +
			desc->bootloader_start_offset) >> 8);
	addr_load_hi = u64_hi32((pmu->ucode.gpu_va +
			desc->bootloader_start_offset) >> 8);

	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), GK20A_PMU_DMAIDX_UCODE);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), addr_code_lo << 8);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), addr_code_hi);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), desc->app_resident_code_offset);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), desc->app_resident_code_size);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x0);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), desc->app_imem_entry);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), addr_data_lo << 8);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), addr_data_hi);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), desc->app_resident_data_size);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x1);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), addr_args);

	g->ops.pmu.write_dmatrfbase(g,
			addr_load_lo - (desc->bootloader_imem_offset >> 8));

	blocks = ((desc->bootloader_size + 0xFF) & ~0xFF) >> 8;

	for (i = 0; i < blocks; i++) {
		gk20a_writel(g, pwr_falcon_dmatrfmoffs_r(),
			desc->bootloader_imem_offset + (i << 8));
		gk20a_writel(g, pwr_falcon_dmatrffboffs_r(),
			desc->bootloader_imem_offset + (i << 8));
		gk20a_writel(g, pwr_falcon_dmatrfcmd_r(),
			pwr_falcon_dmatrfcmd_imem_f(1)  |
			pwr_falcon_dmatrfcmd_write_f(0) |
			pwr_falcon_dmatrfcmd_size_f(6)  |
			pwr_falcon_dmatrfcmd_ctxdma_f(GK20A_PMU_DMAIDX_UCODE));
	}

	nvgpu_flcn_bootstrap(pmu->flcn, desc->bootloader_entry_point);

	gk20a_writel(g, pwr_falcon_os_r(), desc->app_version);

	return 0;
}

static void pmu_handle_pg_sub_feature_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	gk20a_dbg_fn("");

	if (status != 0) {
		nvgpu_err(g, "Sub-feature mask update cmd aborted\n");
		return;
	}

	gv11b_dbg_pmu("sub-feature mask update is acknowledged from PMU %x\n",
							msg->msg.pg.msg_type);
}

static void pmu_handle_pg_param_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	gk20a_dbg_fn("");

	if (status != 0) {
		nvgpu_err(g, "GR PARAM cmd aborted\n");
		return;
	}

	gv11b_dbg_pmu("GR PARAM is acknowledged from PMU %x\n",
							msg->msg.pg.msg_type);
}

int gv11b_pg_gr_init(struct gk20a *g, u32 pg_engine_id)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;

	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {
		memset(&cmd, 0, sizeof(struct pmu_cmd));
		cmd.hdr.unit_id = PMU_UNIT_PG;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
				sizeof(struct pmu_pg_cmd_gr_init_param_v1);
		cmd.cmd.pg.gr_init_param_v1.cmd_type =
				PMU_PG_CMD_ID_PG_PARAM;
		cmd.cmd.pg.gr_init_param_v1.sub_cmd_id =
				PMU_PG_PARAM_CMD_GR_INIT_PARAM;
		cmd.cmd.pg.gr_init_param_v1.featuremask =
				PMU_PG_FEATURE_GR_POWER_GATING_ENABLED;

		gv11b_dbg_pmu("cmd post PMU_PG_CMD_ID_PG_PARAM_INIT\n");
		nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
					pmu_handle_pg_param_msg, pmu, &seq, ~0);

	} else
		return -EINVAL;

	return 0;
}

int gv11b_pg_set_subfeature_mask(struct gk20a *g, u32 pg_engine_id)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;

	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {
		memset(&cmd, 0, sizeof(struct pmu_cmd));
		cmd.hdr.unit_id = PMU_UNIT_PG;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
			sizeof(struct pmu_pg_cmd_sub_feature_mask_update);
		cmd.cmd.pg.sf_mask_update.cmd_type =
				PMU_PG_CMD_ID_PG_PARAM;
		cmd.cmd.pg.sf_mask_update.sub_cmd_id =
				PMU_PG_PARAM_CMD_SUB_FEATURE_MASK_UPDATE;
		cmd.cmd.pg.sf_mask_update.ctrl_id =
				PMU_PG_ELPG_ENGINE_ID_GRAPHICS;
		cmd.cmd.pg.sf_mask_update.enabled_mask =
				PMU_PG_FEATURE_GR_POWER_GATING_ENABLED;

		gv11b_dbg_pmu("cmd post PMU_PG_CMD_SUB_FEATURE_MASK_UPDATE\n");
		nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
				pmu_handle_pg_sub_feature_msg, pmu, &seq, ~0);
	} else
		return -EINVAL;

	return 0;
}
