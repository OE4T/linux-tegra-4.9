/*
 * GM20B ACR
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __ACR_GM20B_H_
#define __ACR_GM20B_H_

#define GM20B_PMU_UCODE_IMAGE "gpmu_ucode_image.bin"
#define GM20B_PMU_UCODE_DESC "gpmu_ucode_desc.bin"
#define GM20B_HSBIN_PMU_UCODE_IMAGE "acr_ucode.bin"
#define GM20B_HSBIN_PMU_BL_UCODE_IMAGE "pmu_bl.bin"
#define GM20B_PMU_UCODE_SIG "pmu_sig.bin"
#define GM20B_FECS_UCODE_SIG "fecs_sig.bin"
#define T18x_GPCCS_UCODE_SIG "gpccs_sig.bin"

bool gm20b_is_pmu_supported(struct gk20a *g);
int prepare_ucode_blob(struct gk20a *g);
int gm20b_bootstrap_hs_flcn(struct gk20a *g);
bool gm20b_is_lazy_bootstrap(u32 falcon_id);
bool gm20b_is_priv_load(u32 falcon_id);
void gm20b_wpr_info(struct gk20a *g, struct wpr_carveout_info *inf);
int gm20b_alloc_blob_space(struct gk20a *g, size_t size, struct nvgpu_mem *mem);
int gm20b_pmu_populate_loader_cfg(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size);
int gm20b_flcn_populate_bl_dmem_desc(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size, u32 falconid);
int pmu_wait_for_halt(struct gk20a *g, unsigned int timeout_ms);
int clear_halt_interrupt_status(struct gk20a *g, unsigned int timeout);
int gm20b_init_pmu_setup_hw1(struct gk20a *g, void *desc, u32 bl_sz);

int gm20b_pmu_setup_sw(struct gk20a *g);
int pmu_exec_gen_bl(struct gk20a *g, void *desc, u8 b_wait_for_halt);
int gm20b_init_nspmu_setup_hw1(struct gk20a *g);
int acr_ucode_patch_sig(struct gk20a *g,
		unsigned int *p_img,
		unsigned int *p_prod_sig,
		unsigned int *p_dbg_sig,
		unsigned int *p_patch_loc,
		unsigned int *p_patch_ind);
#endif /*__ACR_GM20B_H_*/
