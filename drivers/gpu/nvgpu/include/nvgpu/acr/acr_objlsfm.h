/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __ACR_OBJLSFM_H__
#define __ACR_OBJLSFM_H__

#ifndef __NVGPU_ACR_H__
#warning "acr_objlsfm.h not included from nvgpu_acr.h!" \
	"Include nvgpu_acr.h instead of acr_xxx.h to get access to ACR interfaces"
#endif

#include "acr_flcnbl.h"
#include "acr_objflcn.h"

/*
 * LSFM Managed Ucode Image
 * next             : Next image the list, NULL if last.
 * wpr_header         : WPR header for this ucode image
 * lsb_header         : LSB header for this ucode image
 * bl_gen_desc     : Bootloader generic desc structure for this ucode image
 * bl_gen_desc_size : Sizeof bootloader desc structure for this ucode image
 * full_ucode_size  : Surface size required for final ucode image
 * ucode_img        : Ucode image info
 */
struct lsfm_managed_ucode_img {
	struct lsfm_managed_ucode_img *next;
	struct lsf_wpr_header wpr_header;
	struct lsf_lsb_header lsb_header;
	union flcn_bl_generic_desc bl_gen_desc;
	u32 bl_gen_desc_size;
	u32 full_ucode_size;
	struct flcn_ucode_img ucode_img;
};

struct lsfm_managed_ucode_img_v2 {
	struct lsfm_managed_ucode_img_v2 *next;
	struct lsf_wpr_header_v1 wpr_header;
	struct lsf_lsb_header_v1 lsb_header;
	union flcn_bl_generic_desc_v1 bl_gen_desc;
	u32 bl_gen_desc_size;
	u32 full_ucode_size;
	struct flcn_ucode_img_v1 ucode_img;
};

/*
 * Defines the structure used to contain all generic information related to
 * the LSFM.
 * Contains the Light Secure Falcon Manager (LSFM) feature related data.
 */
struct ls_flcn_mgr {
	u16 managed_flcn_cnt;
	u32 wpr_size;
	u32 disable_mask;
	struct lsfm_managed_ucode_img *ucode_img_list;
	void *wpr_client_req_state;/*PACR_CLIENT_REQUEST_STATE originally*/
};

struct ls_flcn_mgr_v1 {
	u16 managed_flcn_cnt;
	u32 wpr_size;
	u32 disable_mask;
	struct lsfm_managed_ucode_img_v2 *ucode_img_list;
	void *wpr_client_req_state;/*PACR_CLIENT_REQUEST_STATE originally*/
};


#endif /* __ACR_OBJLSFM_H__ */
