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

#ifndef __NVGPU_ACR_H__
#define __NVGPU_ACR_H__

#include "gk20a/mm_gk20a.h"

#include "acr_lsfm.h"
#include "acr_flcnbl.h"
#include "acr_objlsfm.h"
#include "acr_objflcn.h"

#define MAX_SUPPORTED_LSFM 3 /*PMU, FECS, GPCCS*/

#define ACR_COMPLETION_TIMEOUT_MS 10000 /*in msec */

#define PMU_SECURE_MODE (0x1)
#define PMU_LSFM_MANAGED (0x2)

struct bin_hdr {
	/* 0x10de */
	u32 bin_magic;
	/* versioning of bin format */
	u32 bin_ver;
	/* Entire image size including this header */
	u32 bin_size;
	/*
	 * Header offset of executable binary metadata,
	 * start @ offset- 0x100 *
	 */
	u32 header_offset;
	/*
	 * Start of executable binary data, start @
	 * offset- 0x200
	 */
	u32 data_offset;
	/* Size of executable binary */
	u32 data_size;
};

struct acr_fw_header {
	u32 sig_dbg_offset;
	u32 sig_dbg_size;
	u32 sig_prod_offset;
	u32 sig_prod_size;
	u32 patch_loc;
	u32 patch_sig;
	u32 hdr_offset; /* This header points to acr_ucode_header_t210_load */
	u32 hdr_size; /* Size of above header */
};

struct wpr_carveout_info {
	u64 wpr_base;
	u64 nonwpr_base;
	u64 size;
};

struct acr_desc {
	struct mem_desc ucode_blob;
	struct mem_desc wpr_dummy;
	struct bin_hdr *bl_bin_hdr;
	struct hsflcn_bl_desc *pmu_hsbl_desc;
	struct bin_hdr *hsbin_hdr;
	struct acr_fw_header *fw_hdr;
	u32 pmu_args;
	const struct firmware *acr_fw;
	union{
		struct flcn_acr_desc *acr_dmem_desc;
		struct flcn_acr_desc_v1 *acr_dmem_desc_v1;
	};
	struct mem_desc acr_ucode;
	const struct firmware *hsbl_fw;
	struct mem_desc hsbl_ucode;
	union {
		struct flcn_bl_dmem_desc bl_dmem_desc;
		struct flcn_bl_dmem_desc_v1 bl_dmem_desc_v1;
	};
	const struct firmware *pmu_fw;
	const struct firmware *pmu_desc;
	u32 capabilities;
};

#endif /*__NVGPU_ACR_H__*/
