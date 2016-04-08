/*
 * Tegra Graphics Chip support for T194
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _NVHOST_T194_H_
#define _NVHOST_T194_H_

#include "chip_support.h"

#define T194_NVHOST_NUMCHANNELS 63

extern struct nvhost_device_data t19_host1x_info;
extern struct nvhost_device_data t19_host1x_hv_info;
extern struct nvhost_device_data t19_host1xb_info;
extern struct nvhost_device_data t19_vic_info;
extern struct nvhost_device_data t19_nvdec_info;
extern struct nvhost_device_data t19_nvjpg_info;
extern struct nvhost_device_data t19_msenc_info;
extern struct nvhost_device_data t19_isp_info;
extern struct nvhost_device_data t19_vi_info;
extern struct nvhost_device_data t19_tsec_info;
extern struct nvhost_device_data t19_tsecb_info;
extern struct nvhost_device_data t19_nvcsi_info;
extern struct nvhost_device_data t19_pvaa_info;
extern struct nvhost_device_data t19_pvab_info;

int nvhost_init_t194_support(struct nvhost_master *host,
			     struct nvhost_chip_support *op);
int nvhost_init_t194_channel_support(struct nvhost_master *host,
			     struct nvhost_chip_support *op);

#endif /* _NVHOST_T194_H_ */
