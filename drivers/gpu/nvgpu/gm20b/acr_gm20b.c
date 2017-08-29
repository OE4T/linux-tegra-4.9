/*
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
#include <nvgpu/pmu.h>
#include <nvgpu/falcon.h>
#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"
#include "gk20a/pmu_gk20a.h"
#include "mm_gm20b.h"
#include "acr_gm20b.h"

#include <nvgpu/hw/gm20b/hw_pwr_gm20b.h>

/*Defines*/
#define gm20b_dbg_pmu(fmt, arg...) \
	gk20a_dbg(gpu_dbg_pmu, fmt, ##arg)

typedef int (*get_ucode_details)(struct gk20a *g, struct flcn_ucode_img *udata);

/*Externs*/

/*Forwards*/
static int pmu_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img);
static int fecs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img);
static int gpccs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img);
static int lsfm_discover_ucode_images(struct gk20a *g,
	struct ls_flcn_mgr *plsfm);
static int lsfm_add_ucode_img(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	struct flcn_ucode_img *ucode_image, u32 falcon_id);
static void lsfm_free_ucode_img_res(struct gk20a *g,
				    struct flcn_ucode_img *p_img);
static void lsfm_free_nonpmu_ucode_img_res(struct gk20a *g,
					   struct flcn_ucode_img *p_img);
static int lsf_gen_wpr_requirements(struct gk20a *g, struct ls_flcn_mgr *plsfm);
static void lsfm_init_wpr_contents(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	struct nvgpu_mem *nonwpr);
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
	nvgpu_mutex_acquire(&g->pmu.isr_mutex);
	pmu_enable_irq(&g->pmu, true);
	g->pmu.isr_enabled = true;
	nvgpu_mutex_release(&g->pmu.isr_mutex);
	gk20a_writel(g, pwr_falcon_cpuctl_alias_r(),
		pwr_falcon_cpuctl_startcpu_f(1));
}

void gm20b_wpr_info(struct gk20a *g, struct wpr_carveout_info *inf)
{
	struct mc_carveout_info mem_inf;

	mc_get_carveout_info(&mem_inf, NULL, MC_SECURITY_CARVEOUT2);

	inf->wpr_base = mem_inf.base;
	inf->nonwpr_base = 0;
	inf->size = mem_inf.size;
}

bool gm20b_is_pmu_supported(struct gk20a *g)
{
	return true;
}

static int pmu_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img)
{
	struct nvgpu_firmware *pmu_fw, *pmu_desc, *pmu_sig;
	struct nvgpu_pmu *pmu = &g->pmu;
	struct lsf_ucode_desc *lsf_desc;
	int err;
	gm20b_dbg_pmu("requesting PMU ucode in GM20B\n");
	pmu_fw = nvgpu_request_firmware(g, GM20B_PMU_UCODE_IMAGE, 0);
	if (!pmu_fw) {
		nvgpu_err(g, "failed to load pmu ucode!!");
		return -ENOENT;
	}
	g->acr.pmu_fw = pmu_fw;
	gm20b_dbg_pmu("Loaded PMU ucode in for blob preparation");

	gm20b_dbg_pmu("requesting PMU ucode desc in GM20B\n");
	pmu_desc = nvgpu_request_firmware(g, GM20B_PMU_UCODE_DESC, 0);
	if (!pmu_desc) {
		nvgpu_err(g, "failed to load pmu ucode desc!!");
		err = -ENOENT;
		goto release_img_fw;
	}
	pmu_sig = nvgpu_request_firmware(g, GM20B_PMU_UCODE_SIG, 0);
	if (!pmu_sig) {
		nvgpu_err(g, "failed to load pmu sig!!");
		err = -ENOENT;
		goto release_desc;
	}
	pmu->desc = (struct pmu_ucode_desc *)pmu_desc->data;
	pmu->ucode_image = (u32 *)pmu_fw->data;
	g->acr.pmu_desc = pmu_desc;

	err = nvgpu_init_pmu_fw_support(pmu);
	if (err) {
		gm20b_dbg_pmu("failed to set function pointers\n");
		goto release_sig;
	}

	lsf_desc = nvgpu_kzalloc(g, sizeof(struct lsf_ucode_desc));
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
	nvgpu_release_firmware(g, pmu_sig);
	return 0;
release_sig:
	nvgpu_release_firmware(g, pmu_sig);
release_desc:
	nvgpu_release_firmware(g, pmu_desc);
release_img_fw:
	nvgpu_release_firmware(g, pmu_fw);
	return err;
}

static int fecs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img)
{
	struct lsf_ucode_desc *lsf_desc;
	struct nvgpu_firmware *fecs_sig;
	int err;

	fecs_sig = nvgpu_request_firmware(g, GM20B_FECS_UCODE_SIG, 0);
	if (!fecs_sig) {
		nvgpu_err(g, "failed to load fecs sig");
		return -ENOENT;
	}
	lsf_desc = nvgpu_kzalloc(g, sizeof(struct lsf_ucode_desc));
	if (!lsf_desc) {
		err = -ENOMEM;
		goto rel_sig;
	}
	memcpy(lsf_desc, (void *)fecs_sig->data, sizeof(struct lsf_ucode_desc));
	lsf_desc->falcon_id = LSF_FALCON_ID_FECS;

	p_img->desc = nvgpu_kzalloc(g, sizeof(struct pmu_ucode_desc));
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
	nvgpu_release_firmware(g, fecs_sig);
	return 0;
free_lsf_desc:
	nvgpu_kfree(g, lsf_desc);
rel_sig:
	nvgpu_release_firmware(g, fecs_sig);
	return err;
}
static int gpccs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img)
{
	struct lsf_ucode_desc *lsf_desc;
	struct nvgpu_firmware *gpccs_sig;
	int err;

	if (!nvgpu_is_enabled(g, NVGPU_SEC_SECUREGPCCS))
		return -ENOENT;

	gpccs_sig = nvgpu_request_firmware(g, T18x_GPCCS_UCODE_SIG, 0);
	if (!gpccs_sig) {
		nvgpu_err(g, "failed to load gpccs sig");
		return -ENOENT;
	}
	lsf_desc = nvgpu_kzalloc(g, sizeof(struct lsf_ucode_desc));
	if (!lsf_desc) {
		err = -ENOMEM;
		goto rel_sig;
	}
	memcpy(lsf_desc, (void *)gpccs_sig->data,
		sizeof(struct lsf_ucode_desc));
	lsf_desc->falcon_id = LSF_FALCON_ID_GPCCS;

	p_img->desc = nvgpu_kzalloc(g, sizeof(struct pmu_ucode_desc));
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
	nvgpu_release_firmware(g, gpccs_sig);
	return 0;
free_lsf_desc:
	nvgpu_kfree(g, lsf_desc);
rel_sig:
	nvgpu_release_firmware(g, gpccs_sig);
	return err;
}

bool gm20b_is_lazy_bootstrap(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = false;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = false;
		break;
	default:
		break;
	}

	return enable_status;
}

bool gm20b_is_priv_load(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = false;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = false;
		break;
	default:
		break;
	}

	return enable_status;
}

int gm20b_alloc_blob_space(struct gk20a *g,
		size_t size, struct nvgpu_mem *mem)
{
	int err;

	err = nvgpu_dma_alloc_sys(g, size, mem);

	return err;
}

int prepare_ucode_blob(struct gk20a *g)
{

	int err;
	struct ls_flcn_mgr lsfm_l, *plsfm;
	struct nvgpu_pmu *pmu = &g->pmu;
	phys_addr_t wpr_addr, wpr_page;
	u32 wprsize;
	int i;
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	struct wpr_carveout_info wpr_inf;
	struct page **pages;

	if (g->acr.ucode_blob.cpu_va) {
		/*Recovery case, we do not need to form
		non WPR blob of ucodes*/
		err = nvgpu_init_pmu_fw_support(pmu);
		if (err) {
			gm20b_dbg_pmu("failed to set function pointers\n");
			return err;
		}
		return 0;
	}
	plsfm = &lsfm_l;
	memset((void *)plsfm, 0, sizeof(struct ls_flcn_mgr));
	gm20b_dbg_pmu("fetching GMMU regs\n");
	g->ops.fb.vpr_info_fetch(g);
	gr_gk20a_init_ctxsw_ucode(g);

	g->ops.pmu.get_wpr(g, &wpr_inf);
	wpr_addr = (phys_addr_t)wpr_inf.wpr_base;
	wprsize = (u32)wpr_inf.size;
	gm20b_dbg_pmu("wpr carveout base:%llx\n", wpr_inf.wpr_base);
	gm20b_dbg_pmu("wpr carveout size :%x\n", wprsize);

	pages = nvgpu_kmalloc(g, sizeof(struct page *) * (wprsize / PAGE_SIZE));
	if (!pages)
		return -ENOMEM;

	wpr_page = wpr_addr;
	for (i = 0; wpr_page < (wpr_addr + wprsize); i++, wpr_page += PAGE_SIZE)
		pages[i] = phys_to_page(wpr_page);
	__nvgpu_mem_create_from_pages(g, &g->pmu.wpr_buf, pages,
				      wprsize / PAGE_SIZE);
	nvgpu_kfree(g, pages);

	g->pmu.wpr_buf.gpu_va = nvgpu_gmmu_map(vm, &g->pmu.wpr_buf,
					       wprsize, 0, gk20a_mem_flag_none,
					       false, APERTURE_SYSMEM);
	gm20b_dbg_pmu("wpr mapped gpu va :%llx\n", g->pmu.wpr_buf.gpu_va);

	/* Discover all managed falcons*/
	err = lsfm_discover_ucode_images(g, plsfm);
	gm20b_dbg_pmu(" Managed Falcon cnt %d\n", plsfm->managed_flcn_cnt);
	if (err)
		goto free_sgt;

	if (plsfm->managed_flcn_cnt && !g->acr.ucode_blob.cpu_va) {
		/* Generate WPR requirements*/
		err = lsf_gen_wpr_requirements(g, plsfm);
		if (err)
			goto free_sgt;

		/*Alloc memory to hold ucode blob contents*/
		err = g->ops.pmu.alloc_blob_space(g, plsfm->wpr_size
				, &g->acr.ucode_blob);
		if (err)
			goto free_sgt;

		gm20b_dbg_pmu("managed LS falcon %d, WPR size %d bytes.\n",
			plsfm->managed_flcn_cnt, plsfm->wpr_size);
		lsfm_init_wpr_contents(g, plsfm, &g->acr.ucode_blob);
	} else {
		gm20b_dbg_pmu("LSFM is managing no falcons.\n");
	}
	gm20b_dbg_pmu("prepare ucode blob return 0\n");
	free_acr_resources(g, plsfm);
free_sgt:
	nvgpu_gmmu_unmap(vm, &g->pmu.wpr_buf, g->pmu.wpr_buf.gpu_va);
	return err;
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
	struct nvgpu_pmu *pmu = &g->pmu;
	struct flcn_ucode_img ucode_img;
	u32 falcon_id;
	u32 i;
	int status;

	/* LSFM requires a secure PMU, discover it first.*/
	/* Obtain the PMU ucode image and add it to the list if required*/
	memset(&ucode_img, 0, sizeof(ucode_img));
	status = pmu_ucode_details(g, &ucode_img);
	if (status)
		return status;

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

	/*Free any ucode image resources if not managing this falcon*/
	if (!(pmu->pmu_mode & PMU_LSFM_MANAGED)) {
		gm20b_dbg_pmu("pmu is not LSFM managed\n");
		lsfm_free_ucode_img_res(g, &ucode_img);
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
					lsfm_free_nonpmu_ucode_img_res(g,
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


int gm20b_pmu_populate_loader_cfg(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size)
{
	struct wpr_carveout_info wpr_inf;
	struct nvgpu_pmu *pmu = &g->pmu;
	struct lsfm_managed_ucode_img *p_lsfm =
			(struct lsfm_managed_ucode_img *)lsfm;
	struct flcn_ucode_img *p_img = &(p_lsfm->ucode_img);
	struct loader_config *ldr_cfg = &(p_lsfm->bl_gen_desc.loader_cfg);
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
	addr_base = p_lsfm->lsb_header.ucode_off;
	g->ops.pmu.get_wpr(g, &wpr_inf);
	addr_base += wpr_inf.wpr_base;
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
	ldr_cfg->code_dma_base1 = 0x0;
	ldr_cfg->code_size_total = desc->app_size;
	ldr_cfg->code_size_to_load = desc->app_resident_code_size;
	ldr_cfg->code_entry_point = desc->app_imem_entry;
	ldr_cfg->data_dma_base = addr_data;
	ldr_cfg->data_dma_base1 = 0;
	ldr_cfg->data_size = desc->app_resident_data_size;
	ldr_cfg->overlay_dma_base = addr_code;
	ldr_cfg->overlay_dma_base1 = 0x0;

	/* Update the argc/argv members*/
	ldr_cfg->argc = 1;
	ldr_cfg->argv = addr_args;

	*p_bl_gen_desc_size = sizeof(struct loader_config);
	g->acr.pmu_args = addr_args;
	return 0;
}

int gm20b_flcn_populate_bl_dmem_desc(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size, u32 falconid)
{
	struct wpr_carveout_info wpr_inf;
	struct lsfm_managed_ucode_img *p_lsfm =
			(struct lsfm_managed_ucode_img *)lsfm;
	struct flcn_ucode_img *p_img = &(p_lsfm->ucode_img);
	struct flcn_bl_dmem_desc *ldr_cfg =
			&(p_lsfm->bl_gen_desc.bl_dmem_desc);
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
	addr_base = p_lsfm->lsb_header.ucode_off;
	g->ops.pmu.get_wpr(g, &wpr_inf);
	if (falconid == LSF_FALCON_ID_GPCCS)
		addr_base += g->pmu.wpr_buf.gpu_va;
	else
		addr_base += wpr_inf.wpr_base;
	gm20b_dbg_pmu("gen loader cfg %x u32 addrbase %x ID\n", (u32)addr_base,
			p_lsfm->wpr_header.falcon_id);
	addr_code = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_code_offset) >> 8);
	addr_data = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_data_offset) >> 8);

	gm20b_dbg_pmu("gen cfg %x u32 addrcode %x & data %x load offset %xID\n",
		(u32)addr_code, (u32)addr_data, desc->bootloader_start_offset,
		p_lsfm->wpr_header.falcon_id);

	/* Populate the LOADER_CONFIG state */
	memset((void *) ldr_cfg, 0, sizeof(struct flcn_bl_dmem_desc));
	ldr_cfg->ctx_dma = GK20A_PMU_DMAIDX_UCODE;
	ldr_cfg->code_dma_base = addr_code;
	ldr_cfg->non_sec_code_size = desc->app_resident_code_size;
	ldr_cfg->data_dma_base = addr_data;
	ldr_cfg->data_size = desc->app_resident_data_size;
	ldr_cfg->code_entry_point = desc->app_imem_entry;
	*p_bl_gen_desc_size = sizeof(struct flcn_bl_dmem_desc);
	return 0;
}

/* Populate falcon boot loader generic desc.*/
static int lsfm_fill_flcn_bl_gen_desc(struct gk20a *g,
		struct lsfm_managed_ucode_img *pnode)
{

	struct nvgpu_pmu *pmu = &g->pmu;
	if (pnode->wpr_header.falcon_id != pmu->falcon_id) {
		gm20b_dbg_pmu("non pmu. write flcn bl gen desc\n");
		g->ops.pmu.flcn_populate_bl_dmem_desc(g,
				pnode, &pnode->bl_gen_desc_size,
				pnode->wpr_header.falcon_id);
		return 0;
	}

	if (pmu->pmu_mode & PMU_LSFM_MANAGED) {
		gm20b_dbg_pmu("pmu write flcn bl gen desc\n");
		if (pnode->wpr_header.falcon_id == pmu->falcon_id)
			return g->ops.pmu.pmu_populate_loader_cfg(g, pnode,
				&pnode->bl_gen_desc_size);
	}

	/* Failed to find the falcon requested. */
	return -ENOENT;
}

/* Initialize WPR contents */
static void lsfm_init_wpr_contents(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	struct nvgpu_mem *ucode)
{
	struct lsfm_managed_ucode_img *pnode = plsfm->ucode_img_list;
	u32 i;

	/* The WPR array is at the base of the WPR */
	pnode = plsfm->ucode_img_list;
	i = 0;

	/*
	 * Walk the managed falcons, flush WPR and LSB headers to FB.
	 * flush any bl args to the storage area relative to the
	 * ucode image (appended on the end as a DMEM area).
	 */
	while (pnode) {
		/* Flush WPR header to memory*/
		nvgpu_mem_wr_n(g, ucode, i * sizeof(pnode->wpr_header),
				&pnode->wpr_header, sizeof(pnode->wpr_header));

		gm20b_dbg_pmu("wpr header");
		gm20b_dbg_pmu("falconid :%d",
				pnode->wpr_header.falcon_id);
		gm20b_dbg_pmu("lsb_offset :%x",
				pnode->wpr_header.lsb_offset);
		gm20b_dbg_pmu("bootstrap_owner :%d",
			pnode->wpr_header.bootstrap_owner);
		gm20b_dbg_pmu("lazy_bootstrap :%d",
				pnode->wpr_header.lazy_bootstrap);
		gm20b_dbg_pmu("status :%d",
				pnode->wpr_header.status);

		/*Flush LSB header to memory*/
		nvgpu_mem_wr_n(g, ucode, pnode->wpr_header.lsb_offset,
				&pnode->lsb_header, sizeof(pnode->lsb_header));

		gm20b_dbg_pmu("lsb header");
		gm20b_dbg_pmu("ucode_off :%x",
				pnode->lsb_header.ucode_off);
		gm20b_dbg_pmu("ucode_size :%x",
				pnode->lsb_header.ucode_size);
		gm20b_dbg_pmu("data_size :%x",
				pnode->lsb_header.data_size);
		gm20b_dbg_pmu("bl_code_size :%x",
				pnode->lsb_header.bl_code_size);
		gm20b_dbg_pmu("bl_imem_off :%x",
				pnode->lsb_header.bl_imem_off);
		gm20b_dbg_pmu("bl_data_off :%x",
				pnode->lsb_header.bl_data_off);
		gm20b_dbg_pmu("bl_data_size :%x",
				pnode->lsb_header.bl_data_size);
		gm20b_dbg_pmu("app_code_off :%x",
				pnode->lsb_header.app_code_off);
		gm20b_dbg_pmu("app_code_size :%x",
				pnode->lsb_header.app_code_size);
		gm20b_dbg_pmu("app_data_off :%x",
				pnode->lsb_header.app_data_off);
		gm20b_dbg_pmu("app_data_size :%x",
				pnode->lsb_header.app_data_size);
		gm20b_dbg_pmu("flags :%x",
				pnode->lsb_header.flags);

		/*If this falcon has a boot loader and related args,
		 * flush them.*/
		if (!pnode->ucode_img.header) {
			/*Populate gen bl and flush to memory*/
			lsfm_fill_flcn_bl_gen_desc(g, pnode);
			nvgpu_mem_wr_n(g, ucode,
					pnode->lsb_header.bl_data_off,
					&pnode->bl_gen_desc,
					pnode->bl_gen_desc_size);
		}
		/*Copying of ucode*/
		nvgpu_mem_wr_n(g, ucode, pnode->lsb_header.ucode_off,
				pnode->ucode_img.data,
				pnode->ucode_img.data_size);
		pnode = pnode->next;
		i++;
	}

	/* Tag the terminator WPR header with an invalid falcon ID. */
	nvgpu_mem_wr32(g, ucode,
			plsfm->managed_flcn_cnt * sizeof(struct lsf_wpr_header) +
			offsetof(struct lsf_wpr_header, falcon_id),
			LSF_FALCON_ID_INVALID);
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

	struct nvgpu_pmu *pmu = &g->pmu;
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

		if (g->ops.pmu.is_priv_load(falcon_id)) {
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
	pnode = nvgpu_kzalloc(g, sizeof(struct lsfm_managed_ucode_img));
	if (pnode == NULL)
		return -ENOMEM;

	/* Keep a copy of the ucode image info locally */
	memcpy(&pnode->ucode_img, ucode_image, sizeof(struct flcn_ucode_img));

	/* Fill in static WPR header info*/
	pnode->wpr_header.falcon_id = falcon_id;
	pnode->wpr_header.bootstrap_owner = LSF_BOOTSTRAP_OWNER_DEFAULT;
	pnode->wpr_header.status = LSF_IMAGE_STATUS_COPY;

	pnode->wpr_header.lazy_bootstrap =
			g->ops.pmu.is_lazy_bootstrap(falcon_id);

	/*TODO to check if PDB_PROP_FLCN_LAZY_BOOTSTRAP is to be supported by
	Android */
	/* Fill in static LSB header info elsewhere */
	lsfm_fill_static_lsb_hdr_info(g, falcon_id, pnode);
	pnode->next = plsfm->ucode_img_list;
	plsfm->ucode_img_list = pnode;
	return 0;
}

/* Free any ucode image structure resources. */
static void lsfm_free_ucode_img_res(struct gk20a *g,
				    struct flcn_ucode_img *p_img)
{
	if (p_img->lsf_desc != NULL) {
		nvgpu_kfree(g, p_img->lsf_desc);
		p_img->lsf_desc = NULL;
	}
}

/* Free any ucode image structure resources. */
static void lsfm_free_nonpmu_ucode_img_res(struct gk20a *g,
					   struct flcn_ucode_img *p_img)
{
	if (p_img->lsf_desc != NULL) {
		nvgpu_kfree(g, p_img->lsf_desc);
		p_img->lsf_desc = NULL;
	}
	if (p_img->desc != NULL) {
		nvgpu_kfree(g, p_img->desc);
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
			lsfm_free_ucode_img_res(g, &mg_ucode_img->ucode_img);
		else
			lsfm_free_nonpmu_ucode_img_res(g,
				&mg_ucode_img->ucode_img);
		plsfm->ucode_img_list = mg_ucode_img->next;
		nvgpu_kfree(g, mg_ucode_img);
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
	struct vm_gk20a *vm = mm->pmu.vm;
	int err = 0;
	u64 *acr_dmem;
	u32 img_size_in_bytes = 0;
	u32 status, size;
	u64 start;
	struct acr_desc *acr = &g->acr;
	struct nvgpu_firmware *acr_fw = acr->acr_fw;
	struct flcn_bl_dmem_desc *bl_dmem_desc = &acr->bl_dmem_desc;
	u32 *acr_ucode_header_t210_load;
	u32 *acr_ucode_data_t210_load;

	start = nvgpu_mem_get_addr(g, &acr->ucode_blob);
	size = acr->ucode_blob.size;

	gm20b_dbg_pmu("");

	if (!acr_fw) {
		/*First time init case*/
		acr_fw = nvgpu_request_firmware(g, GM20B_HSBIN_PMU_UCODE_IMAGE, 0);
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
		bl_dmem_desc->code_dma_base =
			(unsigned int)(((u64)acr->acr_ucode.gpu_va >> 8));
		bl_dmem_desc->code_dma_base1 = 0x0;
		bl_dmem_desc->non_sec_code_off  = acr_ucode_header_t210_load[0];
		bl_dmem_desc->non_sec_code_size = acr_ucode_header_t210_load[1];
		bl_dmem_desc->sec_code_off = acr_ucode_header_t210_load[5];
		bl_dmem_desc->sec_code_size = acr_ucode_header_t210_load[6];
		bl_dmem_desc->code_entry_point = 0; /* Start at 0th offset */
		bl_dmem_desc->data_dma_base =
			bl_dmem_desc->code_dma_base +
			((acr_ucode_header_t210_load[2]) >> 8);
		bl_dmem_desc->data_dma_base1 = 0x0;
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

static u8 pmu_is_debug_mode_en(struct gk20a *g)
{
	u32 ctl_stat =  gk20a_readl(g, pwr_pmu_scpctl_stat_r());
	return pwr_pmu_scpctl_stat_debug_mode_v(ctl_stat);
}

/*
 * @brief Patch signatures into ucode image
 */
int acr_ucode_patch_sig(struct gk20a *g,
		unsigned int *p_img,
		unsigned int *p_prod_sig,
		unsigned int *p_dbg_sig,
		unsigned int *p_patch_loc,
		unsigned int *p_patch_ind)
{
	unsigned int i, *p_sig;
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

static int bl_bootstrap(struct nvgpu_pmu *pmu,
	struct flcn_bl_dmem_desc *pbl_desc, u32 bl_sz)
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
			pwr_pmu_new_instblk_target_sys_coh_f());

	/*copy bootloader interface structure to dmem*/
	nvgpu_flcn_copy_to_dmem(pmu->flcn, 0, (u8 *)pbl_desc,
		sizeof(struct flcn_bl_dmem_desc), 0);

	/* copy bootloader to TOP of IMEM */
	dst = (pwr_falcon_hwcfg_imem_size_v(
			gk20a_readl(g, pwr_falcon_hwcfg_r())) << 8) - bl_sz;

	nvgpu_flcn_copy_to_imem(pmu->flcn, dst,
		(u8 *)(acr->hsbl_ucode.cpu_va), bl_sz, 0, 0,
		pmu_bl_gm10x_desc->bl_start_tag);

	gm20b_dbg_pmu("Before starting falcon with BL\n");

	virt_addr = pmu_bl_gm10x_desc->bl_start_tag << 8;

	nvgpu_flcn_bootstrap(pmu->flcn, virt_addr);

	return 0;
}

int gm20b_init_nspmu_setup_hw1(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	int err = 0;

	gk20a_dbg_fn("");

	nvgpu_mutex_acquire(&pmu->isr_mutex);
	nvgpu_flcn_reset(pmu->flcn);
	pmu->isr_enabled = true;
	nvgpu_mutex_release(&pmu->isr_mutex);

	/* setup apertures - virtual */
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_UCODE),
		pwr_fbif_transcfg_mem_type_virtual_f());
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

	err = g->ops.pmu.pmu_nsbootstrap(pmu);

	return err;
}

int gm20b_init_pmu_setup_hw1(struct gk20a *g,
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
	struct vm_gk20a *vm = mm->pmu.vm;
	int err = 0;
	u32 bl_sz;
	struct acr_desc *acr = &g->acr;
	struct nvgpu_firmware *hsbl_fw = acr->hsbl_fw;
	struct hsflcn_bl_desc *pmu_bl_gm10x_desc;
	u32 *pmu_bl_gm10x = NULL;
	gm20b_dbg_pmu("");

	if (!hsbl_fw) {
		hsbl_fw = nvgpu_request_firmware(g,
			GM20B_HSBIN_PMU_BL_UCODE_IMAGE, 0);
		if (!hsbl_fw) {
			nvgpu_err(g, "pmu ucode load fail");
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
		err = nvgpu_dma_alloc_flags_sys(g,
				NVGPU_DMA_READ_ONLY, bl_sz, &acr->hsbl_ucode);
		if (err) {
			nvgpu_err(g, "failed to allocate memory");
			goto err_done;
		}

		acr->hsbl_ucode.gpu_va = nvgpu_gmmu_map(vm,
				&acr->hsbl_ucode,
				bl_sz,
				0, /* flags */
				gk20a_mem_flag_read_only, false,
				acr->hsbl_ucode.aperture);
		if (!acr->hsbl_ucode.gpu_va) {
			nvgpu_err(g, "failed to map pmu ucode memory!!");
			goto err_free_ucode;
		}

		nvgpu_mem_wr_n(g, &acr->hsbl_ucode, 0, pmu_bl_gm10x, bl_sz);
		gm20b_dbg_pmu("Copied bl ucode to bl_cpuva\n");
	}
	/*
	 * Disable interrupts to avoid kernel hitting breakpoint due
	 * to PMU halt
	 */

	if (g->ops.pmu.falcon_clear_halt_interrupt_status(g,
			gk20a_get_gr_idle_timeout(g)))
		goto err_unmap_bl;

	gm20b_dbg_pmu("phys sec reg %x\n", gk20a_readl(g,
		pwr_falcon_mmu_phys_sec_r()));
	gm20b_dbg_pmu("sctl reg %x\n", gk20a_readl(g, pwr_falcon_sctl_r()));

	g->ops.pmu.init_falcon_setup_hw(g, desc, acr->hsbl_ucode.size);

	/* Poll for HALT */
	if (b_wait_for_halt) {
		err = g->ops.pmu.falcon_wait_for_halt(g,
				ACR_COMPLETION_TIMEOUT_MS);
		if (err == 0) {
			/* Clear the HALT interrupt */
		  if (g->ops.pmu.falcon_clear_halt_interrupt_status(g,
				  gk20a_get_gr_idle_timeout(g)))
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
	nvgpu_gmmu_unmap(vm, &acr->hsbl_ucode, acr->hsbl_ucode.gpu_va);
err_free_ucode:
	nvgpu_dma_free(g, &acr->hsbl_ucode);
err_done:
	nvgpu_release_firmware(g, hsbl_fw);
	return err;
}

/*!
*	Wait for PMU to halt
*	@param[in]	g		GPU object pointer
*	@param[in]	timeout_ms	Timeout in msec for PMU to halt
*	@return '0' if PMU halts
*/
int pmu_wait_for_halt(struct gk20a *g, unsigned int timeout_ms)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	u32 data = 0;
	int ret = -EBUSY;

	ret = nvgpu_flcn_wait_for_halt(pmu->flcn, timeout_ms);
	if (ret) {
		nvgpu_err(g, "ACR boot timed out");
		return ret;
	}

	g->acr.capabilities = gk20a_readl(g, pwr_falcon_mailbox1_r());
	gm20b_dbg_pmu("ACR capabilities %x\n", g->acr.capabilities);
	data = gk20a_readl(g, pwr_falcon_mailbox0_r());
	if (data) {
		nvgpu_err(g, "ACR boot failed, err %x", data);
		ret = -EAGAIN;
	}

	return ret;
}

/*!
*	Wait for PMU halt interrupt status to be cleared
*	@param[in]	g		GPU object pointer
*	@param[in]	timeout_ms	Timeout in msec for halt to clear
*	@return '0' if PMU halt irq status is clear
*/
int clear_halt_interrupt_status(struct gk20a *g, unsigned int timeout_ms)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	int status = 0;

	if (nvgpu_flcn_clear_halt_intr_status(pmu->flcn, timeout_ms))
		status = -EBUSY;

	return status;
}
