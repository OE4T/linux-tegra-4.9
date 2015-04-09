/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>	/* for mdelay */
#include <linux/firmware.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include "../../../../arch/arm/mach-tegra/iomap.h"

#include <linux/platform/tegra/mc.h>

#include "gk20a/gk20a.h"
#include "gk20a/pmu_gk20a.h"
#include "gk20a/semaphore_gk20a.h"
#include "hw_pwr_gm20b.h"

/*Defines*/
#define gm20b_dbg_pmu(fmt, arg...) \
	gk20a_dbg(gpu_dbg_pmu, fmt, ##arg)

typedef int (*get_ucode_details)(struct gk20a *g, struct flcn_ucode_img *udata);

/*Externs*/

/*Forwards*/
static int lsfm_discover_ucode_images(struct gk20a *g,
	struct ls_flcn_mgr *plsfm);
static int lsfm_add_ucode_img(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	struct flcn_ucode_img *ucode_image, u32 falcon_id);
static void lsfm_free_ucode_img_res(struct flcn_ucode_img *p_img);
static void lsfm_free_nonpmu_ucode_img_res(struct flcn_ucode_img *p_img);
static int lsf_gen_wpr_requirements(struct gk20a *g, struct ls_flcn_mgr *plsfm);
static int lsfm_init_wpr_contents(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	void *nonwpr_addr);
static int acr_ucode_patch_sig(struct gk20a *g,
		unsigned int *p_img,
		unsigned int *p_prod_sig,
		unsigned int *p_dbg_sig,
		unsigned int *p_patch_loc,
		unsigned int *p_patch_ind);
static void free_acr_resources(struct gk20a *g, struct ls_flcn_mgr *plsfm);

/*Globals*/
static get_ucode_details pmu_acr_supp_ucode_list[] = {
	pmu_ucode_details,
	fecs_ucode_details,
	gpccs_ucode_details,
};

/*Once is LS mode, cpuctl_alias is only accessible*/
static void start_gm20b_pmu(struct gk20a *g)
{
	/*disable irqs for hs falcon booting as we will poll for halt*/
	mutex_lock(&g->pmu.isr_mutex);
	pmu_enable_irq(&g->pmu, true);
	g->pmu.isr_enabled = true;
	mutex_unlock(&g->pmu.isr_mutex);
	gk20a_writel(g, pwr_falcon_cpuctl_alias_r(),
		pwr_falcon_cpuctl_startcpu_f(1));
}

void gm20b_init_secure_pmu(struct gpu_ops *gops)
{
	gops->pmu.prepare_ucode = prepare_ucode_blob;
	gops->pmu.pmu_setup_hw_and_bootstrap = gm20b_bootstrap_hs_flcn;
}
/* TODO - check if any free blob res needed*/

int pmu_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img)
{
	const struct firmware *pmu_fw, *pmu_desc, *pmu_sig;
	struct pmu_gk20a *pmu = &g->pmu;
	struct lsf_ucode_desc *lsf_desc;
	int err;
	gm20b_dbg_pmu("requesting PMU ucode in GM20B\n");
	pmu_fw = gk20a_request_firmware(g, GM20B_PMU_UCODE_IMAGE);
	if (!pmu_fw) {
		gk20a_err(dev_from_gk20a(g), "failed to load pmu ucode!!");
		return -ENOENT;
	}
	g->acr.pmu_fw = pmu_fw;
	gm20b_dbg_pmu("Loaded PMU ucode in for blob preparation");

	gm20b_dbg_pmu("requesting PMU ucode desc in GM20B\n");
	pmu_desc = gk20a_request_firmware(g, GM20B_PMU_UCODE_DESC);
	if (!pmu_desc) {
		gk20a_err(dev_from_gk20a(g), "failed to load pmu ucode desc!!");
		err = -ENOENT;
		goto release_img_fw;
	}
	pmu_sig = gk20a_request_firmware(g, GM20B_PMU_UCODE_SIG);
	if (!pmu_sig) {
		gk20a_err(dev_from_gk20a(g), "failed to load pmu sig!!");
		err = -ENOENT;
		goto release_desc;
	}
	pmu->desc = (struct pmu_ucode_desc *)pmu_desc->data;
	pmu->ucode_image = (u32 *)pmu_fw->data;
	g->acr.pmu_desc = pmu_desc;

	err = gk20a_init_pmu(pmu);
	if (err) {
		gm20b_dbg_pmu("failed to set function pointers\n");
		goto release_desc;
	}

	lsf_desc = kzalloc(sizeof(struct lsf_ucode_desc), GFP_KERNEL);
	if (!lsf_desc) {
		err = -ENOMEM;
		goto release_sig;
	}
	memcpy(lsf_desc, (void *)pmu_sig->data, sizeof(struct lsf_ucode_desc));
	lsf_desc->falcon_id = LSF_FALCON_ID_PMU;

	p_img->desc = pmu->desc;
	p_img->data = pmu->ucode_image;
	p_img->data_size = pmu->desc->image_size;
	p_img->fw_ver = NULL;
	p_img->header = NULL;
	p_img->lsf_desc = (struct lsf_ucode_desc *)lsf_desc;
	gm20b_dbg_pmu("requesting PMU ucode in GM20B exit\n");
	release_firmware(pmu_sig);
	return 0;
release_sig:
	release_firmware(pmu_sig);
release_desc:
	release_firmware(pmu_desc);
release_img_fw:
	release_firmware(pmu_fw);
	return err;
}

int fecs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img)
{
	struct lsf_ucode_desc *lsf_desc;
	const struct firmware *fecs_sig;
	int err;

	fecs_sig = gk20a_request_firmware(g, GM20B_FECS_UCODE_SIG);
	if (!fecs_sig) {
		gk20a_err(dev_from_gk20a(g), "failed to load fecs sig");
		return -ENOENT;
	}
	lsf_desc = kzalloc(sizeof(struct lsf_ucode_desc), GFP_KERNEL);
	if (!lsf_desc) {
		err = -ENOMEM;
		goto rel_sig;
	}
	memcpy(lsf_desc, (void *)fecs_sig->data, sizeof(struct lsf_ucode_desc));
	lsf_desc->falcon_id = LSF_FALCON_ID_FECS;

	p_img->desc = kzalloc(sizeof(struct pmu_ucode_desc), GFP_KERNEL);
	if (p_img->desc == NULL) {
		err = -ENOMEM;
		goto free_lsf_desc;
	}

	p_img->desc->bootloader_start_offset =
		g->ctxsw_ucode_info.fecs.boot.offset;
	p_img->desc->bootloader_size =
		ALIGN(g->ctxsw_ucode_info.fecs.boot.size, 256);
	p_img->desc->bootloader_imem_offset =
		g->ctxsw_ucode_info.fecs.boot_imem_offset;
	p_img->desc->bootloader_entry_point =
		g->ctxsw_ucode_info.fecs.boot_entry;

	p_img->desc->image_size =
		ALIGN(g->ctxsw_ucode_info.fecs.boot.size, 256) +
		ALIGN(g->ctxsw_ucode_info.fecs.code.size, 256) +
		ALIGN(g->ctxsw_ucode_info.fecs.data.size, 256);
	p_img->desc->app_size = ALIGN(g->ctxsw_ucode_info.fecs.code.size, 256) +
		ALIGN(g->ctxsw_ucode_info.fecs.data.size, 256);
	p_img->desc->app_start_offset = g->ctxsw_ucode_info.fecs.code.offset;
	p_img->desc->app_imem_offset = 0;
	p_img->desc->app_imem_entry = 0;
	p_img->desc->app_dmem_offset = 0;
	p_img->desc->app_resident_code_offset = 0;
	p_img->desc->app_resident_code_size =
		g->ctxsw_ucode_info.fecs.code.size;
	p_img->desc->app_resident_data_offset =
		g->ctxsw_ucode_info.fecs.data.offset -
		g->ctxsw_ucode_info.fecs.code.offset;
	p_img->desc->app_resident_data_size =
		g->ctxsw_ucode_info.fecs.data.size;
	p_img->data = g->ctxsw_ucode_info.surface_desc.cpu_va;
	p_img->data_size = p_img->desc->image_size;

	p_img->fw_ver = NULL;
	p_img->header = NULL;
	p_img->lsf_desc = (struct lsf_ucode_desc *)lsf_desc;
	gm20b_dbg_pmu("fecs fw loaded\n");
	release_firmware(fecs_sig);
	return 0;
free_lsf_desc:
	kfree(lsf_desc);
rel_sig:
	release_firmware(fecs_sig);
	return err;
}
int gpccs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img)
{
	struct lsf_ucode_desc *lsf_desc;
	const struct firmware *gpccs_sig;
	int err;

	if (g->ops.securegpccs == false)
		return -ENOENT;

	gpccs_sig = gk20a_request_firmware(g, GM20B_FECS_UCODE_SIG);
	if (!gpccs_sig) {
		gk20a_err(dev_from_gk20a(g), "failed to load gpccs sig");
		return -ENOENT;
	}
	lsf_desc = kzalloc(sizeof(struct lsf_ucode_desc), GFP_KERNEL);
	if (!lsf_desc) {
		err = -ENOMEM;
		goto rel_sig;
	}
	memcpy(lsf_desc, (void *)gpccs_sig->data,
		sizeof(struct lsf_ucode_desc));
	lsf_desc->falcon_id = LSF_FALCON_ID_GPCCS;

	p_img->desc = kzalloc(sizeof(struct pmu_ucode_desc), GFP_KERNEL);
	if (p_img->desc == NULL) {
		err = -ENOMEM;
		goto free_lsf_desc;
	}

	p_img->desc->bootloader_start_offset =
		0;
	p_img->desc->bootloader_size =
		ALIGN(g->ctxsw_ucode_info.gpccs.boot.size, 256);
	p_img->desc->bootloader_imem_offset =
		g->ctxsw_ucode_info.gpccs.boot_imem_offset;
	p_img->desc->bootloader_entry_point =
		g->ctxsw_ucode_info.gpccs.boot_entry;

	p_img->desc->image_size =
		ALIGN(g->ctxsw_ucode_info.gpccs.boot.size, 256) +
		ALIGN(g->ctxsw_ucode_info.gpccs.code.size, 256) +
		ALIGN(g->ctxsw_ucode_info.gpccs.data.size, 256);
	p_img->desc->app_size = ALIGN(g->ctxsw_ucode_info.gpccs.code.size, 256)
		+ ALIGN(g->ctxsw_ucode_info.gpccs.data.size, 256);
	p_img->desc->app_start_offset = p_img->desc->bootloader_size;
	p_img->desc->app_imem_offset = 0;
	p_img->desc->app_imem_entry = 0;
	p_img->desc->app_dmem_offset = 0;
	p_img->desc->app_resident_code_offset = 0;
	p_img->desc->app_resident_code_size =
		ALIGN(g->ctxsw_ucode_info.gpccs.code.size, 256);
	p_img->desc->app_resident_data_offset =
		ALIGN(g->ctxsw_ucode_info.gpccs.data.offset, 256) -
		ALIGN(g->ctxsw_ucode_info.gpccs.code.offset, 256);
	p_img->desc->app_resident_data_size =
		ALIGN(g->ctxsw_ucode_info.gpccs.data.size, 256);
	p_img->data = (u32 *)((u8 *)g->ctxsw_ucode_info.surface_desc.cpu_va +
		g->ctxsw_ucode_info.gpccs.boot.offset);
	p_img->data_size = ALIGN(p_img->desc->image_size, 256);
	p_img->fw_ver = NULL;
	p_img->header = NULL;
	p_img->lsf_desc = (struct lsf_ucode_desc *)lsf_desc;
	gm20b_dbg_pmu("gpccs fw loaded\n");
	release_firmware(gpccs_sig);
	return 0;
free_lsf_desc:
	kfree(lsf_desc);
rel_sig:
	release_firmware(gpccs_sig);
	return err;
}

int prepare_ucode_blob(struct gk20a *g)
{
	int err;
	struct ls_flcn_mgr lsfm_l, *plsfm;
	struct pmu_gk20a *pmu = &g->pmu;

	if (g->acr.ucode_blob.cpu_va) {
		/*Recovery case, we do not need to form
		non WPR blob of ucodes*/
		err = gk20a_init_pmu(pmu);
		if (err) {
			gm20b_dbg_pmu("failed to set function pointers\n");
			return err;
		}
		return 0;
	}
	plsfm = &lsfm_l;
	memset((void *)plsfm, 0, sizeof(struct ls_flcn_mgr));
	gm20b_dbg_pmu("fetching GMMU regs\n");
	gm20b_mm_mmu_vpr_info_fetch(g);
	gr_gk20a_init_ctxsw_ucode(g);

	/* Discover all managed falcons*/
	err = lsfm_discover_ucode_images(g, plsfm);
	gm20b_dbg_pmu(" Managed Falcon cnt %d\n", plsfm->managed_flcn_cnt);
	if (err)
		return err;

	if (plsfm->managed_flcn_cnt && !g->acr.ucode_blob.cpu_va) {
		/* Generate WPR requirements*/
		err = lsf_gen_wpr_requirements(g, plsfm);
		if (err)
			return err;

		/*Alloc memory to hold ucode blob contents*/
		err = gk20a_gmmu_alloc(g, plsfm->wpr_size, &g->acr.ucode_blob);
		if (err)
			return err;

		gm20b_dbg_pmu("managed LS falcon %d, WPR size %d bytes.\n",
			plsfm->managed_flcn_cnt, plsfm->wpr_size);
		lsfm_init_wpr_contents(g, plsfm, g->acr.ucode_blob.cpu_va);
	} else {
		gm20b_dbg_pmu("LSFM is managing no falcons.\n");
	}
	gm20b_dbg_pmu("prepare ucode blob return 0\n");
	free_acr_resources(g, plsfm);
	return 0;
}

static u8 lsfm_falcon_disabled(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	u32 falcon_id)
{
	return (plsfm->disable_mask >> falcon_id) & 0x1;
}

/* Discover all managed falcon ucode images */
static int lsfm_discover_ucode_images(struct gk20a *g,
	struct ls_flcn_mgr *plsfm)
{
	struct pmu_gk20a *pmu = &g->pmu;
	struct flcn_ucode_img ucode_img;
	u32 falcon_id;
	u32 i;
	int status;

	/* LSFM requires a secure PMU, discover it first.*/
	/* Obtain the PMU ucode image and add it to the list if required*/
	memset(&ucode_img, 0, sizeof(ucode_img));
	status = pmu_ucode_details(g, &ucode_img);
	if (status == 0) {
		if (ucode_img.lsf_desc != NULL) {
			/* The falon_id is formed by grabbing the static base
			 * falon_id from the image and adding the
			 * engine-designated falcon instance.*/
			pmu->pmu_mode |= PMU_SECURE_MODE;
			falcon_id = ucode_img.lsf_desc->falcon_id +
				ucode_img.flcn_inst;

			if (!lsfm_falcon_disabled(g, plsfm, falcon_id)) {
				pmu->falcon_id = falcon_id;
				if (lsfm_add_ucode_img(g, plsfm, &ucode_img,
					pmu->falcon_id) == 0)
					pmu->pmu_mode |= PMU_LSFM_MANAGED;

				plsfm->managed_flcn_cnt++;
			} else {
				gm20b_dbg_pmu("id not managed %d\n",
					ucode_img.lsf_desc->falcon_id);
			}
		}

		/*Free any ucode image resources if not managing this falcon*/
		if (!(pmu->pmu_mode & PMU_LSFM_MANAGED)) {
			gm20b_dbg_pmu("pmu is not LSFM managed\n");
			lsfm_free_ucode_img_res(&ucode_img);
		}
	}

	/* Enumerate all constructed falcon objects,
	 as we need the ucode image info and total falcon count.*/

	/*0th index is always PMU which is already handled in earlier
	if condition*/
	for (i = 1; i < (MAX_SUPPORTED_LSFM); i++) {
		memset(&ucode_img, 0, sizeof(ucode_img));
		if (pmu_acr_supp_ucode_list[i](g, &ucode_img) == 0) {
			if (ucode_img.lsf_desc != NULL) {
				/* We have engine sigs, ensure that this falcon
				is aware of the secure mode expectations
				(ACR status)*/

				/* falon_id is formed by grabbing the static
				base falonId from the image and adding the
				engine-designated falcon instance. */
				falcon_id = ucode_img.lsf_desc->falcon_id +
					ucode_img.flcn_inst;

				if (!lsfm_falcon_disabled(g, plsfm,
					falcon_id)) {
					/* Do not manage non-FB ucode*/
					if (lsfm_add_ucode_img(g,
						plsfm, &ucode_img, falcon_id)
						== 0)
						plsfm->managed_flcn_cnt++;
				} else {
					gm20b_dbg_pmu("not managed %d\n",
						ucode_img.lsf_desc->falcon_id);
					lsfm_free_nonpmu_ucode_img_res(
						&ucode_img);
				}
			}
		} else {
			/* Consumed all available falcon objects */
			gm20b_dbg_pmu("Done checking for ucodes %d\n", i);
			break;
		}
	}
	return 0;
}


static int pmu_populate_loader_cfg(struct gk20a *g,
	struct lsfm_managed_ucode_img *lsfm,
	union flcn_bl_generic_desc *p_bl_gen_desc, u32 *p_bl_gen_desc_size)
{
	struct mc_carveout_info inf;
	struct pmu_gk20a *pmu = &g->pmu;
	struct flcn_ucode_img *p_img = &(lsfm->ucode_img);
	struct loader_config *ldr_cfg =
		(struct loader_config *)(&p_bl_gen_desc->loader_cfg);
	u64 addr_base;
	struct pmu_ucode_desc *desc;
	u64 addr_code, addr_data;
	u32 addr_args;

	if (p_img->desc == NULL) /*This means its a header based ucode,
				  and so we do not fill BL gen desc structure*/
		return -EINVAL;
	desc = p_img->desc;
	/*
	 Calculate physical and virtual addresses for various portions of
	 the PMU ucode image
	 Calculate the 32-bit addresses for the application code, application
	 data, and bootloader code. These values are all based on IM_BASE.
	 The 32-bit addresses will be the upper 32-bits of the virtual or
	 physical addresses of each respective segment.
	*/
	addr_base = lsfm->lsb_header.ucode_off;
	mc_get_carveout_info(&inf, NULL, MC_SECURITY_CARVEOUT2);
	addr_base += inf.base;
	gm20b_dbg_pmu("pmu loader cfg u32 addrbase %x\n", (u32)addr_base);
	/*From linux*/
	addr_code = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_code_offset) >> 8);
	gm20b_dbg_pmu("app start %d app res code off %d\n",
		desc->app_start_offset, desc->app_resident_code_offset);
	addr_data = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_data_offset) >> 8);
	gm20b_dbg_pmu("app res data offset%d\n",
		desc->app_resident_data_offset);
	gm20b_dbg_pmu("bl start off %d\n", desc->bootloader_start_offset);

	addr_args = ((pwr_falcon_hwcfg_dmem_size_v(
			gk20a_readl(g, pwr_falcon_hwcfg_r())))
			<< GK20A_PMU_DMEM_BLKSIZE2);
	addr_args -= g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu);

	gm20b_dbg_pmu("addr_args %x\n", addr_args);

	/* Populate the loader_config state*/
	ldr_cfg->dma_idx = GK20A_PMU_DMAIDX_UCODE;
	ldr_cfg->code_dma_base = addr_code;
	ldr_cfg->code_size_total = desc->app_size;
	ldr_cfg->code_size_to_load = desc->app_resident_code_size;
	ldr_cfg->code_entry_point = desc->app_imem_entry;
	ldr_cfg->data_dma_base = addr_data;
	ldr_cfg->data_size = desc->app_resident_data_size;
	ldr_cfg->overlay_dma_base = addr_code;

	/* Update the argc/argv members*/
	ldr_cfg->argc = 1;
	ldr_cfg->argv = addr_args;

	*p_bl_gen_desc_size = sizeof(p_bl_gen_desc->loader_cfg);
	g->acr.pmu_args = addr_args;
	return 0;
}

static int flcn_populate_bl_dmem_desc(struct gk20a *g,
	struct lsfm_managed_ucode_img *lsfm,
	union flcn_bl_generic_desc *p_bl_gen_desc, u32 *p_bl_gen_desc_size)
{
	struct mc_carveout_info inf;
	struct flcn_ucode_img *p_img = &(lsfm->ucode_img);
	struct flcn_bl_dmem_desc *ldr_cfg =
		(struct flcn_bl_dmem_desc *)(&p_bl_gen_desc->bl_dmem_desc);
	u64 addr_base;
	struct pmu_ucode_desc *desc;
	u64 addr_code, addr_data;

	if (p_img->desc == NULL) /*This means its a header based ucode,
				  and so we do not fill BL gen desc structure*/
		return -EINVAL;
	desc = p_img->desc;

	/*
	 Calculate physical and virtual addresses for various portions of
	 the PMU ucode image
	 Calculate the 32-bit addresses for the application code, application
	 data, and bootloader code. These values are all based on IM_BASE.
	 The 32-bit addresses will be the upper 32-bits of the virtual or
	 physical addresses of each respective segment.
	*/
	addr_base = lsfm->lsb_header.ucode_off;
	mc_get_carveout_info(&inf, NULL, MC_SECURITY_CARVEOUT2);
	addr_base += inf.base;
	gm20b_dbg_pmu("gen loader cfg %x u32 addrbase %x ID\n", (u32)addr_base,
		lsfm->wpr_header.falcon_id);
	addr_code = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_code_offset) >> 8);
	addr_data = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_data_offset) >> 8);

	gm20b_dbg_pmu("gen cfg %x u32 addrcode %x & data %x load offset %xID\n",
		(u32)addr_code, (u32)addr_data, desc->bootloader_start_offset,
		lsfm->wpr_header.falcon_id);

	/* Populate the LOADER_CONFIG state */
	memset((void *) ldr_cfg, 0, sizeof(struct flcn_bl_dmem_desc));
	ldr_cfg->ctx_dma = GK20A_PMU_DMAIDX_UCODE;
	ldr_cfg->code_dma_base = addr_code;
	ldr_cfg->non_sec_code_size = desc->app_resident_code_size;
	ldr_cfg->data_dma_base = addr_data;
	ldr_cfg->data_size = desc->app_resident_data_size;
	ldr_cfg->code_entry_point = desc->app_imem_entry;
	*p_bl_gen_desc_size = sizeof(p_bl_gen_desc->bl_dmem_desc);
	return 0;
}

/* Populate falcon boot loader generic desc.*/
static int lsfm_fill_flcn_bl_gen_desc(struct gk20a *g,
		struct lsfm_managed_ucode_img *pnode)
{

	struct pmu_gk20a *pmu = &g->pmu;
	if (pnode->wpr_header.falcon_id != pmu->falcon_id) {
		gm20b_dbg_pmu("non pmu. write flcn bl gen desc\n");
		flcn_populate_bl_dmem_desc(g, pnode, &pnode->bl_gen_desc,
				&pnode->bl_gen_desc_size);
		return 0;
	}

	if (pmu->pmu_mode & PMU_LSFM_MANAGED) {
		gm20b_dbg_pmu("pmu write flcn bl gen desc\n");
		if (pnode->wpr_header.falcon_id == pmu->falcon_id)
			return pmu_populate_loader_cfg(g, pnode,
				&pnode->bl_gen_desc, &pnode->bl_gen_desc_size);
	}

	/* Failed to find the falcon requested. */
	return -ENOENT;
}

/* Initialize WPR contents */
static int lsfm_init_wpr_contents(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	void *nonwpr_addr)
{

	int status = 0;
	union flcn_bl_generic_desc *nonwpr_bl_gen_desc;
	if (nonwpr_addr == NULL) {
		status = -ENOMEM;
	} else {
		struct lsfm_managed_ucode_img *pnode = plsfm->ucode_img_list;
		struct lsf_wpr_header *wpr_hdr;
		struct lsf_lsb_header *lsb_hdr;
		void *ucode_off;
		u32 i;

		/* The WPR array is at the base of the WPR */
		wpr_hdr = (struct lsf_wpr_header *)nonwpr_addr;
		pnode = plsfm->ucode_img_list;
		i = 0;

		/*
		 * Walk the managed falcons, flush WPR and LSB headers to FB.
		 * flush any bl args to the storage area relative to the
		 * ucode image (appended on the end as a DMEM area).
		 */
		while (pnode) {
			/* Flush WPR header to memory*/
			memcpy(&wpr_hdr[i], &pnode->wpr_header,
					sizeof(struct lsf_wpr_header));
			gm20b_dbg_pmu("wpr header as in memory and pnode\n");
			gm20b_dbg_pmu("falconid :%d %d\n",
				pnode->wpr_header.falcon_id,
				wpr_hdr[i].falcon_id);
			gm20b_dbg_pmu("lsb_offset :%x %x\n",
				pnode->wpr_header.lsb_offset,
				wpr_hdr[i].lsb_offset);
			gm20b_dbg_pmu("bootstrap_owner :%d %d\n",
				pnode->wpr_header.bootstrap_owner,
				wpr_hdr[i].bootstrap_owner);
			gm20b_dbg_pmu("lazy_bootstrap :%d %d\n",
				pnode->wpr_header.lazy_bootstrap,
				wpr_hdr[i].lazy_bootstrap);
			gm20b_dbg_pmu("status :%d %d\n",
				pnode->wpr_header.status, wpr_hdr[i].status);

			/*Flush LSB header to memory*/
			lsb_hdr = (struct lsf_lsb_header *)((u8 *)nonwpr_addr +
					pnode->wpr_header.lsb_offset);
			memcpy(lsb_hdr, &pnode->lsb_header,
					sizeof(struct lsf_lsb_header));
			gm20b_dbg_pmu("lsb header as in memory and pnode\n");
			gm20b_dbg_pmu("ucode_off :%x %x\n",
				pnode->lsb_header.ucode_off,
				lsb_hdr->ucode_off);
			gm20b_dbg_pmu("ucode_size :%x %x\n",
				pnode->lsb_header.ucode_size,
				lsb_hdr->ucode_size);
			gm20b_dbg_pmu("data_size :%x %x\n",
				pnode->lsb_header.data_size,
				lsb_hdr->data_size);
			gm20b_dbg_pmu("bl_code_size :%x %x\n",
				pnode->lsb_header.bl_code_size,
				lsb_hdr->bl_code_size);
			gm20b_dbg_pmu("bl_imem_off :%x %x\n",
				pnode->lsb_header.bl_imem_off,
				lsb_hdr->bl_imem_off);
			gm20b_dbg_pmu("bl_data_off :%x %x\n",
				pnode->lsb_header.bl_data_off,
				lsb_hdr->bl_data_off);
			gm20b_dbg_pmu("bl_data_size :%x %x\n",
				pnode->lsb_header.bl_data_size,
				lsb_hdr->bl_data_size);
			gm20b_dbg_pmu("app_code_off :%x %x\n",
				pnode->lsb_header.app_code_off,
				lsb_hdr->app_code_off);
			gm20b_dbg_pmu("app_code_size :%x %x\n",
				pnode->lsb_header.app_code_size,
				lsb_hdr->app_code_size);
			gm20b_dbg_pmu("app_data_off :%x %x\n",
				pnode->lsb_header.app_data_off,
				lsb_hdr->app_data_off);
			gm20b_dbg_pmu("app_data_size :%x %x\n",
				pnode->lsb_header.app_data_size,
				lsb_hdr->app_data_size);
			gm20b_dbg_pmu("flags :%x %x\n",
				pnode->lsb_header.flags, lsb_hdr->flags);

			/*If this falcon has a boot loader and related args,
			 * flush them.*/
			if (!pnode->ucode_img.header) {
				nonwpr_bl_gen_desc =
					(union flcn_bl_generic_desc *)
					((u8 *)nonwpr_addr +
					pnode->lsb_header.bl_data_off);

				/*Populate gen bl and flush to memory*/
				lsfm_fill_flcn_bl_gen_desc(g, pnode);
				memcpy(nonwpr_bl_gen_desc, &pnode->bl_gen_desc,
					pnode->bl_gen_desc_size);
			}
			ucode_off = (void *)(pnode->lsb_header.ucode_off +
				(u8 *)nonwpr_addr);
			/*Copying of ucode*/
			memcpy(ucode_off, pnode->ucode_img.data,
				pnode->ucode_img.data_size);
			pnode = pnode->next;
			i++;
		}

		/* Tag the terminator WPR header with an invalid falcon ID. */
		gk20a_mem_wr32(&wpr_hdr[plsfm->managed_flcn_cnt].falcon_id,
			0, LSF_FALCON_ID_INVALID);
	}
	return status;
}

/*!
 * lsfm_parse_no_loader_ucode: parses UCODE header of falcon
 *
 * @param[in] p_ucodehdr : UCODE header
 * @param[out] lsb_hdr : updates values in LSB header
 *
 * @return 0
 */
static int lsfm_parse_no_loader_ucode(u32 *p_ucodehdr,
	struct lsf_lsb_header *lsb_hdr)
{

	u32 code_size = 0;
	u32 data_size = 0;
	u32 i = 0;
	u32 total_apps = p_ucodehdr[FLCN_NL_UCODE_HDR_NUM_APPS_IND];

	/* Lets calculate code size*/
	code_size += p_ucodehdr[FLCN_NL_UCODE_HDR_OS_CODE_SIZE_IND];
	for (i = 0; i < total_apps; i++) {
		code_size += p_ucodehdr[FLCN_NL_UCODE_HDR_APP_CODE_SIZE_IND
			(total_apps, i)];
	}
	code_size += p_ucodehdr[FLCN_NL_UCODE_HDR_OS_OVL_SIZE_IND(total_apps)];

	/* Calculate data size*/
	data_size += p_ucodehdr[FLCN_NL_UCODE_HDR_OS_DATA_SIZE_IND];
	for (i = 0; i < total_apps; i++) {
		data_size += p_ucodehdr[FLCN_NL_UCODE_HDR_APP_DATA_SIZE_IND
			(total_apps, i)];
	}

	lsb_hdr->ucode_size = code_size;
	lsb_hdr->data_size = data_size;
	lsb_hdr->bl_code_size = p_ucodehdr[FLCN_NL_UCODE_HDR_OS_CODE_SIZE_IND];
	lsb_hdr->bl_imem_off = 0;
	lsb_hdr->bl_data_off = p_ucodehdr[FLCN_NL_UCODE_HDR_OS_DATA_OFF_IND];
	lsb_hdr->bl_data_size = p_ucodehdr[FLCN_NL_UCODE_HDR_OS_DATA_SIZE_IND];
	return 0;
}

/*!
 * @brief lsfm_fill_static_lsb_hdr_info
 * Populate static LSB header infomation using the provided ucode image
 */
static void lsfm_fill_static_lsb_hdr_info(struct gk20a *g,
	u32 falcon_id, struct lsfm_managed_ucode_img *pnode)
{

	struct pmu_gk20a *pmu = &g->pmu;
	u32 full_app_size = 0;
	u32 data = 0;

	if (pnode->ucode_img.lsf_desc)
		memcpy(&pnode->lsb_header.signature, pnode->ucode_img.lsf_desc,
			sizeof(struct lsf_ucode_desc));
	pnode->lsb_header.ucode_size = pnode->ucode_img.data_size;

	/* The remainder of the LSB depends on the loader usage */
	if (pnode->ucode_img.header) {
		/* Does not use a loader */
		pnode->lsb_header.data_size = 0;
		pnode->lsb_header.bl_code_size = 0;
		pnode->lsb_header.bl_data_off = 0;
		pnode->lsb_header.bl_data_size = 0;

		lsfm_parse_no_loader_ucode(pnode->ucode_img.header,
			&(pnode->lsb_header));

		/* Load the first 256 bytes of IMEM. */
		/* Set LOAD_CODE_AT_0 and DMACTL_REQ_CTX.
		True for all method based falcons */
		data = NV_FLCN_ACR_LSF_FLAG_LOAD_CODE_AT_0_TRUE |
			NV_FLCN_ACR_LSF_FLAG_DMACTL_REQ_CTX_TRUE;
		pnode->lsb_header.flags = data;
	} else {
		/* Uses a loader. that is has a desc */
		pnode->lsb_header.data_size = 0;

		/* The loader code size is already aligned (padded) such that
		the code following it is aligned, but the size in the image
		desc is not, bloat it up to be on a 256 byte alignment. */
		pnode->lsb_header.bl_code_size = ALIGN(
			pnode->ucode_img.desc->bootloader_size,
			LSF_BL_CODE_SIZE_ALIGNMENT);
		full_app_size = ALIGN(pnode->ucode_img.desc->app_size,
			LSF_BL_CODE_SIZE_ALIGNMENT) +
			pnode->lsb_header.bl_code_size;
		pnode->lsb_header.ucode_size = ALIGN(
			pnode->ucode_img.desc->app_resident_data_offset,
			LSF_BL_CODE_SIZE_ALIGNMENT) +
			pnode->lsb_header.bl_code_size;
		pnode->lsb_header.data_size = full_app_size -
			pnode->lsb_header.ucode_size;
		/* Though the BL is located at 0th offset of the image, the VA
		is different to make sure that it doesnt collide the actual OS
		VA range */
		pnode->lsb_header.bl_imem_off =
			pnode->ucode_img.desc->bootloader_imem_offset;

		/* TODO: OBJFLCN should export properties using which the below
			flags should be populated.*/
		pnode->lsb_header.flags = 0;

		if (falcon_id == pmu->falcon_id) {
			data = NV_FLCN_ACR_LSF_FLAG_DMACTL_REQ_CTX_TRUE;
			pnode->lsb_header.flags = data;
		}
		if (falcon_id == LSF_FALCON_ID_GPCCS) {
			pnode->lsb_header.flags |=
				NV_FLCN_ACR_LSF_FLAG_FORCE_PRIV_LOAD_TRUE;
		}
	}
}

/* Adds a ucode image to the list of managed ucode images managed. */
static int lsfm_add_ucode_img(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	struct flcn_ucode_img *ucode_image, u32 falcon_id)
{

	struct lsfm_managed_ucode_img *pnode;
	pnode = kzalloc(sizeof(struct lsfm_managed_ucode_img), GFP_KERNEL);
	if (pnode == NULL)
		return -ENOMEM;

	/* Keep a copy of the ucode image info locally */
	memcpy(&pnode->ucode_img, ucode_image, sizeof(struct flcn_ucode_img));

	/* Fill in static WPR header info*/
	pnode->wpr_header.falcon_id = falcon_id;
	pnode->wpr_header.bootstrap_owner = LSF_BOOTSTRAP_OWNER_DEFAULT;
	pnode->wpr_header.status = LSF_IMAGE_STATUS_COPY;

	if (falcon_id == LSF_FALCON_ID_GPCCS)
		pnode->wpr_header.lazy_bootstrap = 1;

	/*TODO to check if PDB_PROP_FLCN_LAZY_BOOTSTRAP is to be supported by
	Android */
	/* Fill in static LSB header info elsewhere */
	lsfm_fill_static_lsb_hdr_info(g, falcon_id, pnode);
	pnode->next = plsfm->ucode_img_list;
	plsfm->ucode_img_list = pnode;
	return 0;
}

/* Free any ucode image structure resources*/
static void lsfm_free_ucode_img_res(struct flcn_ucode_img *p_img)
{
	if (p_img->lsf_desc != NULL) {
		kfree(p_img->lsf_desc);
		p_img->lsf_desc = NULL;
	}
}

/* Free any ucode image structure resources*/
static void lsfm_free_nonpmu_ucode_img_res(struct flcn_ucode_img *p_img)
{
	if (p_img->lsf_desc != NULL) {
		kfree(p_img->lsf_desc);
		p_img->lsf_desc = NULL;
	}
	if (p_img->desc != NULL) {
		kfree(p_img->desc);
		p_img->desc = NULL;
	}
}

static void free_acr_resources(struct gk20a *g, struct ls_flcn_mgr *plsfm)
{
	u32 cnt = plsfm->managed_flcn_cnt;
	struct lsfm_managed_ucode_img *mg_ucode_img;
	while (cnt) {
		mg_ucode_img = plsfm->ucode_img_list;
		if (mg_ucode_img->ucode_img.lsf_desc->falcon_id ==
				LSF_FALCON_ID_PMU)
			lsfm_free_ucode_img_res(&mg_ucode_img->ucode_img);
		else
			lsfm_free_nonpmu_ucode_img_res(
				&mg_ucode_img->ucode_img);
		plsfm->ucode_img_list = mg_ucode_img->next;
		kfree(mg_ucode_img);
		cnt--;
	}
}

/* Generate WPR requirements for ACR allocation request */
static int lsf_gen_wpr_requirements(struct gk20a *g, struct ls_flcn_mgr *plsfm)
{
	struct lsfm_managed_ucode_img *pnode = plsfm->ucode_img_list;
	u32 wpr_offset;

	/* Calculate WPR size required */

	/* Start with an array of WPR headers at the base of the WPR.
	 The expectation here is that the secure falcon will do a single DMA
	 read of this array and cache it internally so it's OK to pack these.
	 Also, we add 1 to the falcon count to indicate the end of the array.*/
	wpr_offset = sizeof(struct lsf_wpr_header) *
		(plsfm->managed_flcn_cnt+1);

	/* Walk the managed falcons, accounting for the LSB structs
	as well as the ucode images. */
	while (pnode) {
		/* Align, save off, and include an LSB header size */
		wpr_offset = ALIGN(wpr_offset,
			LSF_LSB_HEADER_ALIGNMENT);
		pnode->wpr_header.lsb_offset = wpr_offset;
		wpr_offset += sizeof(struct lsf_lsb_header);

		/* Align, save off, and include the original (static)
		ucode image size */
		wpr_offset = ALIGN(wpr_offset,
			LSF_UCODE_DATA_ALIGNMENT);
		pnode->lsb_header.ucode_off = wpr_offset;
		wpr_offset += pnode->ucode_img.data_size;

		/* For falcons that use a boot loader (BL), we append a loader
		desc structure on the end of the ucode image and consider this
		the boot loader data. The host will then copy the loader desc
		args to this space within the WPR region (before locking down)
		and the HS bin will then copy them to DMEM 0 for the loader. */
		if (!pnode->ucode_img.header) {
			/* Track the size for LSB details filled in later
			 Note that at this point we don't know what kind of i
			boot loader desc, so we just take the size of the
			generic one, which is the largest it will will ever be.
			*/
			/* Align (size bloat) and save off generic
			descriptor size*/
			pnode->lsb_header.bl_data_size = ALIGN(
				sizeof(pnode->bl_gen_desc),
				LSF_BL_DATA_SIZE_ALIGNMENT);

			/*Align, save off, and include the additional BL data*/
			wpr_offset = ALIGN(wpr_offset,
				LSF_BL_DATA_ALIGNMENT);
			pnode->lsb_header.bl_data_off = wpr_offset;
			wpr_offset += pnode->lsb_header.bl_data_size;
		} else {
			/* bl_data_off is already assigned in static
			information. But that is from start of the image */
			pnode->lsb_header.bl_data_off +=
				(wpr_offset - pnode->ucode_img.data_size);
		}

		/* Finally, update ucode surface size to include updates */
		pnode->full_ucode_size = wpr_offset -
			pnode->lsb_header.ucode_off;
		if (pnode->wpr_header.falcon_id != LSF_FALCON_ID_PMU) {
			pnode->lsb_header.app_code_off =
				pnode->lsb_header.bl_code_size;
			pnode->lsb_header.app_code_size =
				pnode->lsb_header.ucode_size -
				pnode->lsb_header.bl_code_size;
			pnode->lsb_header.app_data_off =
				pnode->lsb_header.ucode_size;
			pnode->lsb_header.app_data_size =
				pnode->lsb_header.data_size;
		}
		pnode = pnode->next;
	}
	plsfm->wpr_size = wpr_offset;
	return 0;
}

/*Loads ACR bin to FB mem and bootstraps PMU with bootloader code
 * start and end are addresses of ucode blob in non-WPR region*/
int gm20b_bootstrap_hs_flcn(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = &mm->pmu.vm;
	int i, err = 0;
	u64 *acr_dmem;
	u32 img_size_in_bytes = 0;
	u32 status, size;
	u64 start;
	struct acr_gm20b *acr = &g->acr;
	const struct firmware *acr_fw = acr->acr_fw;
	struct flcn_bl_dmem_desc *bl_dmem_desc = &acr->bl_dmem_desc;
	u32 *acr_ucode_header_t210_load;
	u32 *acr_ucode_data_t210_load;

	start = g->ops.mm.get_iova_addr(g, acr->ucode_blob.sgt->sgl, 0);
	size = acr->ucode_blob.size;

	gm20b_dbg_pmu("");

	if (!acr_fw) {
		/*First time init case*/
		acr_fw = gk20a_request_firmware(g, GM20B_HSBIN_PMU_UCODE_IMAGE);
		if (!acr_fw) {
			gk20a_err(dev_from_gk20a(g), "pmu ucode get fail");
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
			gk20a_err(dev_from_gk20a(g), "patch signatures fail");
			err = -1;
			goto err_release_acr_fw;
		}
		err = gk20a_gmmu_alloc_map(vm, img_size_in_bytes,
				&acr->acr_ucode);
		if (err) {
			err = -ENOMEM;
			goto err_release_acr_fw;
		}

		acr_dmem = (u64 *)
			&(((u8 *)acr_ucode_data_t210_load)[
					acr_ucode_header_t210_load[2]]);
		acr->acr_dmem_desc = (struct flcn_acr_desc *)((u8 *)(
			acr->acr_ucode.cpu_va) + acr_ucode_header_t210_load[2]);
		((struct flcn_acr_desc *)acr_dmem)->nonwpr_ucode_blob_start =
			start;
		((struct flcn_acr_desc *)acr_dmem)->nonwpr_ucode_blob_size =
			size;
		((struct flcn_acr_desc *)acr_dmem)->regions.no_regions = 2;
		((struct flcn_acr_desc *)acr_dmem)->wpr_offset = 0;

		for (i = 0; i < (img_size_in_bytes/4); i++) {
			gk20a_mem_wr32(acr->acr_ucode.cpu_va, i,
					acr_ucode_data_t210_load[i]);
		}
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
		bl_dmem_desc->code_dma_base =
			(unsigned int)(((u64)acr->acr_ucode.gpu_va >> 8));
		bl_dmem_desc->non_sec_code_off  = acr_ucode_header_t210_load[0];
		bl_dmem_desc->non_sec_code_size = acr_ucode_header_t210_load[1];
		bl_dmem_desc->sec_code_off = acr_ucode_header_t210_load[5];
		bl_dmem_desc->sec_code_size = acr_ucode_header_t210_load[6];
		bl_dmem_desc->code_entry_point = 0; /* Start at 0th offset */
		bl_dmem_desc->data_dma_base =
			bl_dmem_desc->code_dma_base +
			((acr_ucode_header_t210_load[2]) >> 8);
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
	gk20a_gmmu_unmap_free(vm, &acr->acr_ucode);
err_release_acr_fw:
	release_firmware(acr_fw);
	acr->acr_fw = NULL;
	return err;
}

static u8 pmu_is_debug_mode_en(struct gk20a *g)
{
	u32 ctl_stat =  gk20a_readl(g, pwr_pmu_scpctl_stat_r());
	return pwr_pmu_scpctl_stat_debug_mode_v(ctl_stat);
}

/*
 * @brief Patch signatures into ucode image
 */
static int
acr_ucode_patch_sig(struct gk20a *g,
		unsigned int *p_img,
		unsigned int *p_prod_sig,
		unsigned int *p_dbg_sig,
		unsigned int *p_patch_loc,
		unsigned int *p_patch_ind)
{
	int i, *p_sig;
	gm20b_dbg_pmu("");

	if (!pmu_is_debug_mode_en(g)) {
		p_sig = p_prod_sig;
		gm20b_dbg_pmu("PRODUCTION MODE\n");
	} else {
		p_sig = p_dbg_sig;
		gm20b_dbg_pmu("DEBUG MODE\n");
	}

	/* Patching logic:*/
	for (i = 0; i < sizeof(*p_patch_loc)>>2; i++) {
		p_img[(p_patch_loc[i]>>2)] = p_sig[(p_patch_ind[i]<<2)];
		p_img[(p_patch_loc[i]>>2)+1] = p_sig[(p_patch_ind[i]<<2)+1];
		p_img[(p_patch_loc[i]>>2)+2] = p_sig[(p_patch_ind[i]<<2)+2];
		p_img[(p_patch_loc[i]>>2)+3] = p_sig[(p_patch_ind[i]<<2)+3];
	}
	return 0;
}

static int bl_bootstrap(struct pmu_gk20a *pmu,
	struct flcn_bl_dmem_desc *pbl_desc, u32 bl_sz)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct acr_gm20b *acr = &g->acr;
	struct mm_gk20a *mm = &g->mm;
	u32 imem_dst_blk = 0;
	u32 virt_addr = 0;
	u32 tag = 0;
	u32 index = 0;
	struct hsflcn_bl_desc *pmu_bl_gm10x_desc = g->acr.pmu_hsbl_desc;
	u32 *bl_ucode;

	gk20a_dbg_fn("");
	gk20a_writel(g, pwr_falcon_itfen_r(),
			gk20a_readl(g, pwr_falcon_itfen_r()) |
			pwr_falcon_itfen_ctxen_enable_f());
	gk20a_writel(g, pwr_pmu_new_instblk_r(),
			pwr_pmu_new_instblk_ptr_f(
				gk20a_mem_phys(&mm->pmu.inst_block) >> 12) |
			pwr_pmu_new_instblk_valid_f(1) |
			pwr_pmu_new_instblk_target_sys_coh_f());

	/* TBD: load all other surfaces */
	/*copy bootloader interface structure to dmem*/
	gk20a_writel(g, pwr_falcon_dmemc_r(0),
			pwr_falcon_dmemc_offs_f(0) |
			pwr_falcon_dmemc_blk_f(0)  |
			pwr_falcon_dmemc_aincw_f(1));
	pmu_copy_to_dmem(pmu, 0, (u8 *)pbl_desc,
		sizeof(struct flcn_bl_dmem_desc), 0);
	/*TODO This had to be copied to bl_desc_dmem_load_off, but since
	 * this is 0, so ok for now*/

	/* Now copy bootloader to TOP of IMEM */
	imem_dst_blk = (pwr_falcon_hwcfg_imem_size_v(
			gk20a_readl(g, pwr_falcon_hwcfg_r()))) - bl_sz/256;

	/* Set Auto-Increment on write */
	gk20a_writel(g, pwr_falcon_imemc_r(0),
			pwr_falcon_imemc_offs_f(0) |
			pwr_falcon_imemc_blk_f(imem_dst_blk)  |
			pwr_falcon_imemc_aincw_f(1));
	virt_addr = pmu_bl_gm10x_desc->bl_start_tag << 8;
	tag = virt_addr >> 8; /* tag is always 256B aligned */
	bl_ucode = (u32 *)(acr->hsbl_ucode.cpu_va);
	for (index = 0; index < bl_sz/4; index++) {
		if ((index % 64) == 0) {
			gk20a_writel(g, pwr_falcon_imemt_r(0),
				(tag & 0xffff) << 0);
			tag++;
		}
		gk20a_writel(g, pwr_falcon_imemd_r(0),
				bl_ucode[index] & 0xffffffff);
	}

	gk20a_writel(g, pwr_falcon_imemt_r(0), (0 & 0xffff) << 0);
	gm20b_dbg_pmu("Before starting falcon with BL\n");

	gk20a_writel(g, pwr_falcon_bootvec_r(),
			pwr_falcon_bootvec_vec_f(virt_addr));

	gk20a_writel(g, pwr_falcon_cpuctl_r(),
			pwr_falcon_cpuctl_startcpu_f(1));

	return 0;
}

static int gm20b_init_pmu_setup_hw1(struct gk20a *g,
		struct flcn_bl_dmem_desc *desc, u32 bl_sz)
{

	struct pmu_gk20a *pmu = &g->pmu;
	int err;
	struct gk20a_platform *platform = platform_get_drvdata(g->dev);

	gk20a_dbg_fn("");

	mutex_lock(&pmu->isr_mutex);
	pmu_reset(pmu);
	pmu->isr_enabled = true;
	mutex_unlock(&pmu->isr_mutex);

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

	/*Copying pmu cmdline args*/
	g->ops.pmu_ver.set_pmu_cmdline_args_cpu_freq(pmu,
				clk_get_rate(platform->clk[1]));
	g->ops.pmu_ver.set_pmu_cmdline_args_secure_mode(pmu, 1);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_size(
		pmu, GK20A_PMU_TRACE_BUFSIZE);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_base(pmu);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_idx(
		pmu, GK20A_PMU_DMAIDX_VIRT);
	pmu_copy_to_dmem(pmu, g->acr.pmu_args,
			(u8 *)(g->ops.pmu_ver.get_pmu_cmdline_args_ptr(pmu)),
			g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu), 0);
	/*disable irqs for hs falcon booting as we will poll for halt*/
	mutex_lock(&pmu->isr_mutex);
	pmu_enable_irq(pmu, false);
	pmu->isr_enabled = false;
	mutex_unlock(&pmu->isr_mutex);
	err = bl_bootstrap(pmu, desc, bl_sz);
	if (err)
		return err;
	return 0;
}

/*
* Executes a generic bootloader and wait for PMU to halt.
* This BL will be used for those binaries that are loaded
* and executed at times other than RM PMU Binary execution.
*
* @param[in] g			gk20a pointer
* @param[in] desc		Bootloader descriptor
* @param[in] dma_idx		DMA Index
* @param[in] b_wait_for_halt	Wait for PMU to HALT
*/
int pmu_exec_gen_bl(struct gk20a *g, void *desc, u8 b_wait_for_halt)
{
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = &mm->pmu.vm;
	struct device *d = dev_from_gk20a(g);
	int i, err = 0;
	u32 bl_sz;
	struct acr_gm20b *acr = &g->acr;
	const struct firmware *hsbl_fw = acr->hsbl_fw;
	struct hsflcn_bl_desc *pmu_bl_gm10x_desc;
	u32 *pmu_bl_gm10x = NULL;
	gm20b_dbg_pmu("");

	if (!hsbl_fw) {
		hsbl_fw = gk20a_request_firmware(g,
			GM20B_HSBIN_PMU_BL_UCODE_IMAGE);
		if (!hsbl_fw) {
			gk20a_err(dev_from_gk20a(g), "pmu ucode load fail");
			return -ENOENT;
		}
		acr->hsbl_fw = hsbl_fw;
		acr->bl_bin_hdr = (struct bin_hdr *)hsbl_fw->data;
		acr->pmu_hsbl_desc = (struct hsflcn_bl_desc *)(hsbl_fw->data +
				acr->bl_bin_hdr->header_offset);
		pmu_bl_gm10x_desc = acr->pmu_hsbl_desc;
		pmu_bl_gm10x = (u32 *)(hsbl_fw->data +
			acr->bl_bin_hdr->data_offset);
		bl_sz = ALIGN(pmu_bl_gm10x_desc->bl_img_hdr.bl_code_size,
				256);
		acr->hsbl_ucode.size = bl_sz;
		gm20b_dbg_pmu("Executing Generic Bootloader\n");

		/*TODO in code verify that enable PMU is done,
			scrubbing etc is done*/
		/*TODO in code verify that gmmu vm init is done*/
		err = gk20a_gmmu_alloc_attr(g,
				DMA_ATTR_READ_ONLY, bl_sz, &acr->hsbl_ucode);
		if (err) {
			gk20a_err(d, "failed to allocate memory\n");
			goto err_done;
		}

		acr->hsbl_ucode.gpu_va = gk20a_gmmu_map(vm, &acr->hsbl_ucode.sgt,
				bl_sz,
				0, /* flags */
				gk20a_mem_flag_read_only);
		if (!acr->hsbl_ucode.gpu_va) {
			gk20a_err(d, "failed to map pmu ucode memory!!");
			goto err_free_ucode;
		}

		for (i = 0; i < (bl_sz) >> 2; i++)
			gk20a_mem_wr32(acr->hsbl_ucode.cpu_va, i, pmu_bl_gm10x[i]);
		gm20b_dbg_pmu("Copied bl ucode to bl_cpuva\n");
	}
	/*
	 * Disable interrupts to avoid kernel hitting breakpoint due
	 * to PMU halt
	 */

	if (clear_halt_interrupt_status(g, gk20a_get_gr_idle_timeout(g)))
		goto err_unmap_bl;

	gm20b_dbg_pmu("phys sec reg %x\n", gk20a_readl(g,
		pwr_falcon_mmu_phys_sec_r()));
	gm20b_dbg_pmu("sctl reg %x\n", gk20a_readl(g, pwr_falcon_sctl_r()));

	gm20b_init_pmu_setup_hw1(g, desc, acr->hsbl_ucode.size);
	/* Poll for HALT */
	if (b_wait_for_halt) {
		err = pmu_wait_for_halt(g, gk20a_get_gr_idle_timeout(g));
		if (err == 0) {
			/* Clear the HALT interrupt */
		  if (clear_halt_interrupt_status(g, gk20a_get_gr_idle_timeout(g)))
			goto err_unmap_bl;
		}
		else
			goto err_unmap_bl;
	}
	gm20b_dbg_pmu("after waiting for halt, err %x\n", err);
	gm20b_dbg_pmu("phys sec reg %x\n", gk20a_readl(g,
		pwr_falcon_mmu_phys_sec_r()));
	gm20b_dbg_pmu("sctl reg %x\n", gk20a_readl(g, pwr_falcon_sctl_r()));
	start_gm20b_pmu(g);
	return 0;
err_unmap_bl:
	gk20a_gmmu_unmap(vm, acr->hsbl_ucode.gpu_va,
			acr->hsbl_ucode.size, gk20a_mem_flag_none);
err_free_ucode:
	gk20a_gmmu_free(g, &acr->hsbl_ucode);
err_done:
	release_firmware(hsbl_fw);
	return err;
}

/*!
*	Wait for PMU to halt
*	@param[in]	g		GPU object pointer
*	@param[in]	timeout_us	Timeout in Us for PMU to halt
*	@return '0' if PMU halts
*/
int pmu_wait_for_halt(struct gk20a *g, unsigned int timeout)
{
	u32 data = 0;
	while (timeout != 0) {
		data = gk20a_readl(g, pwr_falcon_cpuctl_r());
		if (data & pwr_falcon_cpuctl_halt_intr_m())
			/*CPU is halted break*/
			break;
		timeout--;
		udelay(1);
	}
	if (timeout == 0)
		return -EBUSY;
	data = gk20a_readl(g, pwr_falcon_mailbox0_r());
	if (data) {
		gk20a_err(dev_from_gk20a(g), "ACR boot failed, err %x", data);
		return -EAGAIN;
	}
	return 0;
}

/*!
*	Wait for PMU halt interrupt status to be cleared
*	@param[in]	g		GPU object pointer
*	@param[in]	timeout_us	Timeout in Us for PMU to halt
*	@return '0' if PMU halt irq status is clear
*/
int clear_halt_interrupt_status(struct gk20a *g, unsigned int timeout)
{
	u32 data = 0;
	while (timeout != 0) {
		gk20a_writel(g, pwr_falcon_irqsclr_r(),
			     gk20a_readl(g, pwr_falcon_irqsclr_r()) | (0x10));
		data = gk20a_readl(g, (pwr_falcon_irqstat_r()));
		if ((data & pwr_falcon_irqstat_halt_true_f()) !=
			pwr_falcon_irqstat_halt_true_f())
			/*halt irq is clear*/
			break;
		timeout--;
		udelay(1);
	}
	if (timeout == 0)
		return -EBUSY;
	return 0;
}
