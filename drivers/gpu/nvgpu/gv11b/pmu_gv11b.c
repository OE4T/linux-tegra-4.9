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

#include "gk20a/gk20a.h"

#include "gp10b/pmu_gp10b.h"
#include "gp106/pmu_gp106.h"

#include "pmu_gv11b.h"

#include <nvgpu/hw/gv11b/hw_pwr_gv11b.h>

#define ALIGN_4KB     12

static bool gv11b_is_pmu_supported(struct gk20a *g)
{
	return true;
}

static int gv11b_pmu_bootstrap(struct nvgpu_pmu *pmu)
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

	pmu_copy_to_dmem(pmu, addr_args,
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

	gk20a_writel(g, pwr_falcon_bootvec_r(),
		pwr_falcon_bootvec_vec_f(desc->bootloader_entry_point));

	gk20a_writel(g, pwr_falcon_cpuctl_r(),
		pwr_falcon_cpuctl_startcpu_f(1));

	gk20a_writel(g, pwr_falcon_os_r(), desc->app_version);

	return 0;
}


void gv11b_init_pmu_ops(struct gpu_ops *gops)
{
	gp10b_init_pmu_ops(gops);
	gops->pmu.pmu_nsbootstrap = gv11b_pmu_bootstrap;
	gops->pmu.is_pmu_supported = gv11b_is_pmu_supported;
	gops->pmu.reset = gp106_pmu_reset;
	gops->pmu.pmu_get_queue_head = pwr_pmu_queue_head_r;
	gops->pmu.pmu_get_queue_head_size = pwr_pmu_queue_head__size_1_v;
	gops->pmu.pmu_get_queue_tail = pwr_pmu_queue_tail_r;
	gops->pmu.pmu_get_queue_tail_size = pwr_pmu_queue_tail__size_1_v;
	gops->pmu.pmu_elpg_statistics = gp106_pmu_elpg_statistics;
}
