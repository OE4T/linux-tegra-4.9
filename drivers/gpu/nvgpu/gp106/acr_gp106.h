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

#ifndef __ACR_GP106_H_
#define __ACR_GP106_H_

#define GP106_FECS_UCODE_SIG "gp106/fecs_sig.bin"
#define GP106_GPCCS_UCODE_SIG "gp106/gpccs_sig.bin"
#define GP104_FECS_UCODE_SIG "gp104/fecs_sig.bin"
#define GP104_GPCCS_UCODE_SIG "gp104/gpccs_sig.bin"


int gp106_bootstrap_hs_flcn(struct gk20a *g);
int gp106_prepare_ucode_blob(struct gk20a *g);
int gp106_alloc_blob_space(struct gk20a *g,
		size_t size, struct nvgpu_mem *mem);

void gp106_wpr_info(struct gk20a *g, struct wpr_carveout_info *inf);

void lsfm_free_ucode_img_res(struct gk20a *g,
				    struct flcn_ucode_img_v1 *p_img);
void lsfm_free_nonpmu_ucode_img_res(struct gk20a *g,
					   struct flcn_ucode_img_v1 *p_img);
int lsf_gen_wpr_requirements(struct gk20a *g,
		struct ls_flcn_mgr_v1 *plsfm);
void free_acr_resources(struct gk20a *g, struct ls_flcn_mgr_v1 *plsfm);
void lsfm_fill_static_lsb_hdr_info(struct gk20a *g,
	u32 falcon_id, struct lsfm_managed_ucode_img_v2 *pnode);
int gp106_pmu_populate_loader_cfg(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size);

int pmu_ucode_details(struct gk20a *g, struct flcn_ucode_img_v1 *p_img);
int fecs_ucode_details(struct gk20a *g,
		struct flcn_ucode_img_v1 *p_img);
int gpccs_ucode_details(struct gk20a *g,
		struct flcn_ucode_img_v1 *p_img);
int lsfm_add_ucode_img(struct gk20a *g, struct ls_flcn_mgr_v1 *plsfm,
	struct flcn_ucode_img_v1 *ucode_image, u32 falcon_id);
int lsfm_discover_ucode_images(struct gk20a *g,
	struct ls_flcn_mgr_v1 *plsfm);
void lsfm_init_wpr_contents(struct gk20a *g,
		struct ls_flcn_mgr_v1 *plsfm, struct nvgpu_mem *nonwpr);
int gp106_flcn_populate_bl_dmem_desc(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size, u32 falconid);
int lsfm_fill_flcn_bl_gen_desc(struct gk20a *g,
		struct lsfm_managed_ucode_img_v2 *pnode);
#endif /*__PMU_GP106_H_*/
