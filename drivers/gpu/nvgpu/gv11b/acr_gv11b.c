/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include <nvgpu/types.h>
#include <linux/platform/tegra/mc.h>

#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/timers.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/acr/nvgpu_acr.h>
#include <nvgpu/firmware.h>

#include "gk20a/gk20a.h"
#include "acr_gv11b.h"
#include "pmu_gv11b.h"
#include "gk20a/pmu_gk20a.h"
#include "gm20b/mm_gm20b.h"
#include "gm20b/acr_gm20b.h"
#include "gp106/acr_gp106.h"

#include <nvgpu/hw/gv11b/hw_pwr_gv11b.h>

/*Defines*/
#define gv11b_dbg_pmu(fmt, arg...) \
	gk20a_dbg(gpu_dbg_pmu, fmt, ##arg)

static void flcn64_set_dma(struct falc_u64 *dma_addr, u64 value)
{
	dma_addr->lo |= u64_lo32(value);
	dma_addr->hi |= u64_hi32(value);
}
/*Externs*/

/*Forwards*/

/*Loads ACR bin to FB mem and bootstraps PMU with bootloader code
 * start and end are addresses of ucode blob in non-WPR region*/
int gv11b_bootstrap_hs_flcn(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	int err = 0;
	u64 *acr_dmem;
	u32 img_size_in_bytes = 0;
	u32 status, size, index;
	u64 start;
	struct acr_desc *acr = &g->acr;
	struct nvgpu_firmware *acr_fw = acr->acr_fw;
	struct flcn_bl_dmem_desc_v1 *bl_dmem_desc = &acr->bl_dmem_desc_v1;
	u32 *acr_ucode_header_t210_load;
	u32 *acr_ucode_data_t210_load;

	start = nvgpu_mem_get_addr(g, &acr->ucode_blob);
	size = acr->ucode_blob.size;

	gv11b_dbg_pmu("acr ucode blob start %llx\n", start);
	gv11b_dbg_pmu("acr ucode blob size %x\n", size);

	gv11b_dbg_pmu("");

	if (!acr_fw) {
		/*First time init case*/
		acr_fw = nvgpu_request_firmware(g,
						GM20B_HSBIN_PMU_UCODE_IMAGE, 0);
		if (!acr_fw) {
			nvgpu_err(g, "pmu ucode get fail");
			return -ENOENT;
		}
		acr->acr_fw = acr_fw;
		acr->hsbin_hdr = (struct bin_hdr *)acr_fw->data;
		acr->fw_hdr = (struct acr_fw_header *)(acr_fw->data +
				acr->hsbin_hdr->header_offset);
		acr_ucode_data_t210_load = (u32 *)(acr_fw->data +
				acr->hsbin_hdr->data_offset);
		acr_ucode_header_t210_load = (u32 *)(acr_fw->data +
				acr->fw_hdr->hdr_offset);
		img_size_in_bytes = ALIGN((acr->hsbin_hdr->data_size), 256);

		gv11b_dbg_pmu("sig dbg offset %u\n",
						acr->fw_hdr->sig_dbg_offset);
		gv11b_dbg_pmu("sig dbg size %u\n", acr->fw_hdr->sig_dbg_size);
		gv11b_dbg_pmu("sig prod offset %u\n",
						acr->fw_hdr->sig_prod_offset);
		gv11b_dbg_pmu("sig prod size %u\n",
						acr->fw_hdr->sig_prod_size);
		gv11b_dbg_pmu("patch loc  %u\n", acr->fw_hdr->patch_loc);
		gv11b_dbg_pmu("patch sig  %u\n", acr->fw_hdr->patch_sig);
		gv11b_dbg_pmu("header offset %u\n", acr->fw_hdr->hdr_offset);
		gv11b_dbg_pmu("header size %u\n", acr->fw_hdr->hdr_size);

		/* Lets patch the signatures first.. */
		if (acr_ucode_patch_sig(g, acr_ucode_data_t210_load,
					(u32 *)(acr_fw->data +
						acr->fw_hdr->sig_prod_offset),
					(u32 *)(acr_fw->data +
						acr->fw_hdr->sig_dbg_offset),
					(u32 *)(acr_fw->data +
						acr->fw_hdr->patch_loc),
					(u32 *)(acr_fw->data +
						acr->fw_hdr->patch_sig)) < 0) {
			nvgpu_err(g, "patch signatures fail");
			err = -1;
			goto err_release_acr_fw;
		}
		err = nvgpu_dma_alloc_map_sys(vm, img_size_in_bytes,
				&acr->acr_ucode);
		if (err) {
			err = -ENOMEM;
			goto err_release_acr_fw;
		}

		for (index = 0; index < 9; index++)
			gv11b_dbg_pmu("acr_ucode_header_t210_load %u\n",
					acr_ucode_header_t210_load[index]);

		acr_dmem = (u64 *)
			&(((u8 *)acr_ucode_data_t210_load)[
					acr_ucode_header_t210_load[2]]);
		acr->acr_dmem_desc_v1 = (struct flcn_acr_desc_v1 *)((u8 *)(
			acr->acr_ucode.cpu_va) + acr_ucode_header_t210_load[2]);
		((struct flcn_acr_desc_v1 *)acr_dmem)->nonwpr_ucode_blob_start =
			(start);
		((struct flcn_acr_desc_v1 *)acr_dmem)->nonwpr_ucode_blob_size =
			size;
		((struct flcn_acr_desc_v1 *)acr_dmem)->regions.no_regions = 2;
		((struct flcn_acr_desc_v1 *)acr_dmem)->wpr_offset = 0;

		nvgpu_mem_wr_n(g, &acr->acr_ucode, 0,
				acr_ucode_data_t210_load, img_size_in_bytes);
		/*
		 * In order to execute this binary, we will be using
		 * a bootloader which will load this image into PMU IMEM/DMEM.
		 * Fill up the bootloader descriptor for PMU HAL to use..
		 * TODO: Use standard descriptor which the generic bootloader is
		 * checked in.
		 */
		bl_dmem_desc->signature[0] = 0;
		bl_dmem_desc->signature[1] = 0;
		bl_dmem_desc->signature[2] = 0;
		bl_dmem_desc->signature[3] = 0;
		bl_dmem_desc->ctx_dma = GK20A_PMU_DMAIDX_VIRT;
		flcn64_set_dma(&bl_dmem_desc->code_dma_base,
						acr->acr_ucode.gpu_va);
		bl_dmem_desc->non_sec_code_off  = acr_ucode_header_t210_load[0];
		bl_dmem_desc->non_sec_code_size = acr_ucode_header_t210_load[1];
		bl_dmem_desc->sec_code_off = acr_ucode_header_t210_load[5];
		bl_dmem_desc->sec_code_size = acr_ucode_header_t210_load[6];
		bl_dmem_desc->code_entry_point = 0; /* Start at 0th offset */
		flcn64_set_dma(&bl_dmem_desc->data_dma_base,
				acr->acr_ucode.gpu_va +
				acr_ucode_header_t210_load[2]);
		bl_dmem_desc->data_size = acr_ucode_header_t210_load[3];
	} else
		acr->acr_dmem_desc->nonwpr_ucode_blob_size = 0;
	status = pmu_exec_gen_bl(g, bl_dmem_desc, 1);
	if (status != 0) {
		err = status;
		goto err_free_ucode_map;
	}

	return 0;
err_free_ucode_map:
	nvgpu_dma_unmap_free(vm, &acr->acr_ucode);
err_release_acr_fw:
	nvgpu_release_firmware(g, acr_fw);
	acr->acr_fw = NULL;

	return err;
}

static int bl_bootstrap(struct nvgpu_pmu *pmu,
	struct flcn_bl_dmem_desc_v1 *pbl_desc, u32 bl_sz)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct acr_desc *acr = &g->acr;
	struct mm_gk20a *mm = &g->mm;
	u32 virt_addr = 0;
	struct hsflcn_bl_desc *pmu_bl_gm10x_desc = g->acr.pmu_hsbl_desc;
	u32 dst;

	gk20a_dbg_fn("");

	gk20a_writel(g, pwr_falcon_itfen_r(),
			gk20a_readl(g, pwr_falcon_itfen_r()) |
			pwr_falcon_itfen_ctxen_enable_f());
	gk20a_writel(g, pwr_pmu_new_instblk_r(),
		pwr_pmu_new_instblk_ptr_f(
		gk20a_mm_inst_block_addr(g, &mm->pmu.inst_block) >> 12) |
		pwr_pmu_new_instblk_valid_f(1) |
		pwr_pmu_new_instblk_target_sys_ncoh_f());

	/*copy bootloader interface structure to dmem*/
	nvgpu_flcn_copy_to_dmem(pmu->flcn, 0, (u8 *)pbl_desc,
		sizeof(struct flcn_bl_dmem_desc_v1), 0);

	/* copy bootloader to TOP of IMEM */
	dst = (pwr_falcon_hwcfg_imem_size_v(
			gk20a_readl(g, pwr_falcon_hwcfg_r())) << 8) - bl_sz;

	nvgpu_flcn_copy_to_imem(pmu->flcn, dst,
		(u8 *)(acr->hsbl_ucode.cpu_va), bl_sz, 0, 0,
		pmu_bl_gm10x_desc->bl_start_tag);

	gv11b_dbg_pmu("Before starting falcon with BL\n");

	virt_addr = pmu_bl_gm10x_desc->bl_start_tag << 8;

	nvgpu_flcn_bootstrap(pmu->flcn, virt_addr);

	return 0;
}

int gv11b_init_pmu_setup_hw1(struct gk20a *g,
		void *desc, u32 bl_sz)
{

	struct nvgpu_pmu *pmu = &g->pmu;
	int err;

	gk20a_dbg_fn("");

	nvgpu_mutex_acquire(&pmu->isr_mutex);
	nvgpu_flcn_reset(pmu->flcn);
	pmu->isr_enabled = true;
	nvgpu_mutex_release(&pmu->isr_mutex);

	/* setup apertures - virtual */
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_UCODE),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_noncoherent_sysmem_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_VIRT),
			pwr_fbif_transcfg_mem_type_virtual_f());
	/* setup apertures - physical */
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_VID),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_noncoherent_sysmem_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_COH),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_coherent_sysmem_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_NCOH),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_noncoherent_sysmem_f());

	/*Copying pmu cmdline args*/
	g->ops.pmu_ver.set_pmu_cmdline_args_cpu_freq(pmu,
				g->ops.clk.get_rate(g, CTRL_CLK_DOMAIN_PWRCLK));
	g->ops.pmu_ver.set_pmu_cmdline_args_secure_mode(pmu, 1);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_size(
		pmu, GK20A_PMU_TRACE_BUFSIZE);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_base(pmu);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_idx(
		pmu, GK20A_PMU_DMAIDX_VIRT);
	nvgpu_flcn_copy_to_dmem(pmu->flcn, g->acr.pmu_args,
			(u8 *)(g->ops.pmu_ver.get_pmu_cmdline_args_ptr(pmu)),
			g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu), 0);
	/*disable irqs for hs falcon booting as we will poll for halt*/
	nvgpu_mutex_acquire(&pmu->isr_mutex);
	pmu_enable_irq(pmu, false);
	pmu->isr_enabled = false;
	nvgpu_mutex_release(&pmu->isr_mutex);
	/*Clearing mailbox register used to reflect capabilities*/
	gk20a_writel(g, pwr_falcon_mailbox1_r(), 0);
	err = bl_bootstrap(pmu, desc, bl_sz);
	if (err)
		return err;
	return 0;
}

