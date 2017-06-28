/*
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

#include <nvgpu/pmu.h>

#include "gk20a/gk20a.h"
#include "gk20a/pmu_gk20a.h"

#include "gm20b/pmu_gm20b.h"

#include "gp10b/pmu_gp10b.h"

#include "gp106/pmu_gp106.h"

#include "sec2_gp106.h"

#include <nvgpu/hw/gp106/hw_mc_gp106.h>
#include <nvgpu/hw/gp106/hw_pwr_gp106.h>
#include <nvgpu/hw/gp106/hw_psec_gp106.h>

/*Defines*/
#define gm20b_dbg_pmu(fmt, arg...) \
	gk20a_dbg(gpu_dbg_pmu, fmt, ##arg)

int sec2_clear_halt_interrupt_status(struct gk20a *g, unsigned int timeout)
{
	u32 data = 0;
	struct nvgpu_timeout to;

	nvgpu_timeout_init(g, &to, timeout, NVGPU_TIMER_CPU_TIMER);
	do {
		gk20a_writel(g, psec_falcon_irqsclr_r(),
			     gk20a_readl(g, psec_falcon_irqsclr_r()) | (0x10));
		data = gk20a_readl(g, psec_falcon_irqstat_r());
		if ((data & psec_falcon_irqstat_halt_true_f()) !=
			psec_falcon_irqstat_halt_true_f())
			/*halt irq is clear*/
			break;
		nvgpu_udelay(1);
	} while (!nvgpu_timeout_expired(&to));

	if (nvgpu_timeout_peek_expired(&to))
		return -EBUSY;
	return 0;
}

int sec2_wait_for_halt(struct gk20a *g, unsigned int timeout)
{
	u32 data = 0;
	int completion = -EBUSY;
	struct nvgpu_timeout to;

	nvgpu_timeout_init(g, &to, timeout, NVGPU_TIMER_CPU_TIMER);
	do {
		data = gk20a_readl(g, psec_falcon_cpuctl_r());
		if (data & psec_falcon_cpuctl_halt_intr_m()) {
			/*CPU is halted break*/
			completion = 0;
			break;
		}
		nvgpu_udelay(1);
	} while (!nvgpu_timeout_expired(&to));

	if (completion) {
		nvgpu_err(g, "ACR boot timed out");
		return completion;
	}

	g->acr.capabilities = gk20a_readl(g, psec_falcon_mailbox1_r());
	gm20b_dbg_pmu("ACR capabilities %x\n", g->acr.capabilities);
	data = gk20a_readl(g, psec_falcon_mailbox0_r());
	if (data) {

		nvgpu_err(g, "ACR boot failed, err %x", data);
		completion = -EAGAIN;
	}

	init_pmu_setup_hw1(g);

	return completion;
}

int bl_bootstrap_sec2(struct nvgpu_pmu *pmu,
	void *desc, u32 bl_sz)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct acr_desc *acr = &g->acr;
	struct mm_gk20a *mm = &g->mm;
	u32 imem_dst_blk = 0;
	u32 virt_addr = 0;
	u32 tag = 0;
	u32 index = 0;
	struct hsflcn_bl_desc *pmu_bl_gm10x_desc = g->acr.pmu_hsbl_desc;
	u32 *bl_ucode;
	u32 data = 0;

	gk20a_dbg_fn("");

	/* SEC2 Config */
	gk20a_writel(g, psec_falcon_itfen_r(),
			gk20a_readl(g, psec_falcon_itfen_r()) |
			psec_falcon_itfen_ctxen_enable_f());

	gk20a_writel(g, psec_falcon_nxtctx_r(),
			pwr_pmu_new_instblk_ptr_f(
			gk20a_mm_inst_block_addr(g, &mm->pmu.inst_block) >> 12) |
			pwr_pmu_new_instblk_valid_f(1) |
			nvgpu_aperture_mask(g, &mm->pmu.inst_block,
				pwr_pmu_new_instblk_target_sys_coh_f(),
				pwr_pmu_new_instblk_target_fb_f()));

	data = gk20a_readl(g, psec_falcon_debug1_r());
	data |= psec_falcon_debug1_ctxsw_mode_m();
	gk20a_writel(g, psec_falcon_debug1_r(), data);

	data = gk20a_readl(g, psec_falcon_engctl_r());
	data |= (1 << 3);
	gk20a_writel(g, psec_falcon_engctl_r(), data);

	/* TBD: load all other surfaces */
	/*copy bootloader interface structure to dmem*/
	gk20a_writel(g, psec_falcon_dmemc_r(0),
			psec_falcon_dmemc_offs_f(0) |
			psec_falcon_dmemc_blk_f(0)  |
			psec_falcon_dmemc_aincw_f(1));
	nvgpu_flcn_copy_to_dmem(&g->sec2_flcn, 0, (u8 *)desc,
		sizeof(struct flcn_bl_dmem_desc), 0);
	/*TODO This had to be copied to bl_desc_dmem_load_off, but since
	 * this is 0, so ok for now*/

	/* Now copy bootloader to TOP of IMEM */
	imem_dst_blk = (psec_falcon_hwcfg_imem_size_v(
			gk20a_readl(g, psec_falcon_hwcfg_r()))) - bl_sz/256;

	/* Set Auto-Increment on write */
	gk20a_writel(g, psec_falcon_imemc_r(0),
			psec_falcon_imemc_offs_f(0) |
			psec_falcon_imemc_blk_f(imem_dst_blk)  |
			psec_falcon_imemc_aincw_f(1));
	virt_addr = pmu_bl_gm10x_desc->bl_start_tag << 8;
	tag = virt_addr >> 8; /* tag is always 256B aligned */
	bl_ucode = (u32 *)(acr->hsbl_ucode.cpu_va);
	for (index = 0; index < bl_sz/4; index++) {
		if ((index % 64) == 0) {
			gk20a_writel(g, psec_falcon_imemt_r(0),
				(tag & 0xffff) << 0);
			tag++;
		}
		gk20a_writel(g, psec_falcon_imemd_r(0),
				bl_ucode[index] & 0xffffffff);
	}
	gk20a_writel(g, psec_falcon_imemt_r(0), (0 & 0xffff) << 0);

	gm20b_dbg_pmu("Before starting falcon with BL\n");

	gk20a_writel(g, psec_falcon_mailbox0_r(), 0xDEADA5A5);

	gk20a_writel(g, psec_falcon_bootvec_r(),
			psec_falcon_bootvec_vec_f(virt_addr));

	gk20a_writel(g, psec_falcon_cpuctl_r(),
			psec_falcon_cpuctl_startcpu_f(1));

	return 0;
}

void init_pmu_setup_hw1(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct nvgpu_pmu *pmu = &g->pmu;

	/* PMU TRANSCFG */
	/* setup apertures - virtual */
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_UCODE),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_local_fb_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_VIRT),
			pwr_fbif_transcfg_mem_type_virtual_f());
	/* setup apertures - physical */
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_VID),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_local_fb_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_COH),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_coherent_sysmem_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_NCOH),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_noncoherent_sysmem_f());

	/* PMU Config */
	gk20a_writel(g, pwr_falcon_itfen_r(),
				gk20a_readl(g, pwr_falcon_itfen_r()) |
				pwr_falcon_itfen_ctxen_enable_f());
	gk20a_writel(g, pwr_pmu_new_instblk_r(),
				pwr_pmu_new_instblk_ptr_f(
					gk20a_mm_inst_block_addr(g, &mm->pmu.inst_block) >> 12) |
				pwr_pmu_new_instblk_valid_f(1) |
				nvgpu_aperture_mask(g, &mm->pmu.inst_block,
					pwr_pmu_new_instblk_target_sys_coh_f(),
					pwr_pmu_new_instblk_target_fb_f()));

	/*Copying pmu cmdline args*/
	g->ops.pmu_ver.set_pmu_cmdline_args_cpu_freq(pmu, 0);
	g->ops.pmu_ver.set_pmu_cmdline_args_secure_mode(pmu, 1);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_size(
		pmu, GK20A_PMU_TRACE_BUFSIZE);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_base(pmu);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_idx(
		pmu, GK20A_PMU_DMAIDX_VIRT);

	nvgpu_flcn_copy_to_dmem(pmu->flcn, g->acr.pmu_args,
			(u8 *)(g->ops.pmu_ver.get_pmu_cmdline_args_ptr(pmu)),
			g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu), 0);

}

int gp106_sec2_reset(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	gk20a_writel(g, psec_falcon_engine_r(),
			pwr_falcon_engine_reset_true_f());
	nvgpu_udelay(10);
	gk20a_writel(g, psec_falcon_engine_r(),
			pwr_falcon_engine_reset_false_f());

	nvgpu_log_fn(g, "done");
	return 0;
}

int init_sec2_setup_hw1(struct gk20a *g,
		void *desc, u32 bl_sz)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	int err;
	u32 data = 0;

	nvgpu_log_fn(g, " ");

	nvgpu_flcn_reset(&g->sec2_flcn);

	data = gk20a_readl(g, psec_fbif_ctl_r());
	data |= psec_fbif_ctl_allow_phys_no_ctx_allow_f();
	gk20a_writel(g, psec_fbif_ctl_r(), data);

	data = gk20a_readl(g, psec_falcon_dmactl_r());
	data &= ~(psec_falcon_dmactl_require_ctx_f(1));
	gk20a_writel(g, psec_falcon_dmactl_r(), data);

	/* setup apertures - virtual */
	gk20a_writel(g, psec_fbif_transcfg_r(GK20A_PMU_DMAIDX_UCODE),
			psec_fbif_transcfg_mem_type_physical_f() |
			psec_fbif_transcfg_target_local_fb_f());
	gk20a_writel(g, psec_fbif_transcfg_r(GK20A_PMU_DMAIDX_VIRT),
			psec_fbif_transcfg_mem_type_virtual_f());
	/* setup apertures - physical */
	gk20a_writel(g, psec_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_VID),
			psec_fbif_transcfg_mem_type_physical_f() |
			psec_fbif_transcfg_target_local_fb_f());
	gk20a_writel(g, psec_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_COH),
			psec_fbif_transcfg_mem_type_physical_f() |
			psec_fbif_transcfg_target_coherent_sysmem_f());
	gk20a_writel(g, psec_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_NCOH),
			psec_fbif_transcfg_mem_type_physical_f() |
			psec_fbif_transcfg_target_noncoherent_sysmem_f());

	err = bl_bootstrap_sec2(pmu, desc, bl_sz);
	if (err)
		return err;

	return 0;
}
