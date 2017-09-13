/*
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

#include <nvgpu/pmu.h>
#include <nvgpu/falcon.h>

#include "gk20a/gk20a.h"
#include "sec2_gp106.h"

#include <nvgpu/hw/gp106/hw_pwr_gp106.h>
#include <nvgpu/hw/gp106/hw_psec_gp106.h>

/*Defines*/
#define gm20b_dbg_pmu(fmt, arg...) \
	gk20a_dbg(gpu_dbg_pmu, fmt, ##arg)

int sec2_clear_halt_interrupt_status(struct gk20a *g, unsigned int timeout)
{
	int status = 0;

	if (nvgpu_flcn_clear_halt_intr_status(&g->sec2_flcn, timeout))
		status = -EBUSY;

	return status;
}

int sec2_wait_for_halt(struct gk20a *g, unsigned int timeout)
{
	u32 data = 0;
	int completion = 0;

	completion = nvgpu_flcn_wait_for_halt(&g->sec2_flcn, timeout);
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
	u32 virt_addr = 0;
	struct hsflcn_bl_desc *pmu_bl_gm10x_desc = g->acr.pmu_hsbl_desc;
	u32 data = 0;
	u32 dst;

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

	/*copy bootloader interface structure to dmem*/
	nvgpu_flcn_copy_to_dmem(&g->sec2_flcn, 0, (u8 *)desc,
		sizeof(struct flcn_bl_dmem_desc), 0);

	/* copy bootloader to TOP of IMEM */
	dst = (psec_falcon_hwcfg_imem_size_v(
			gk20a_readl(g, psec_falcon_hwcfg_r())) << 8) - bl_sz;

	nvgpu_flcn_copy_to_imem(&g->sec2_flcn, dst,
		(u8 *)(acr->hsbl_ucode.cpu_va), bl_sz, 0, 0,
		pmu_bl_gm10x_desc->bl_start_tag);

	gm20b_dbg_pmu("Before starting falcon with BL\n");

	gk20a_writel(g, psec_falcon_mailbox0_r(), 0xDEADA5A5);

	virt_addr = pmu_bl_gm10x_desc->bl_start_tag << 8;

	nvgpu_flcn_bootstrap(&g->sec2_flcn, virt_addr);

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
