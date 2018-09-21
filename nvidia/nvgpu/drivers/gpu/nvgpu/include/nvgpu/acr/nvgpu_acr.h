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

#ifndef __NVGPU_ACR_H__
#define __NVGPU_ACR_H__

#include "gk20a/mm_gk20a.h"

#include "acr_lsfm.h"
#include "acr_flcnbl.h"
#include "acr_objlsfm.h"
#include "acr_objflcn.h"

struct nvgpu_firmware;

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
	struct nvgpu_mem ucode_blob;
	struct nvgpu_mem wpr_dummy;
	struct bin_hdr *bl_bin_hdr;
	struct hsflcn_bl_desc *pmu_hsbl_desc;
	struct bin_hdr *hsbin_hdr;
	struct acr_fw_header *fw_hdr;
	u32 pmu_args;
	struct nvgpu_firmware *acr_fw;
	union{
		struct flcn_acr_desc *acr_dmem_desc;
		struct flcn_acr_desc_v1 *acr_dmem_desc_v1;
	};
	struct nvgpu_mem acr_ucode;
	struct nvgpu_firmware *hsbl_fw;
	struct nvgpu_mem hsbl_ucode;
	union {
		struct flcn_bl_dmem_desc bl_dmem_desc;
		struct flcn_bl_dmem_desc_v1 bl_dmem_desc_v1;
	};
	struct nvgpu_firmware *pmu_fw;
	struct nvgpu_firmware *pmu_desc;
	u32 capabilities;
};

#endif /*__NVGPU_ACR_H__*/
