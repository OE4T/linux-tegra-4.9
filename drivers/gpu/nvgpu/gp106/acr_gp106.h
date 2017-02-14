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

void gp106_init_secure_pmu(struct gpu_ops *gops);

#endif /*__PMU_GP106_H_*/
