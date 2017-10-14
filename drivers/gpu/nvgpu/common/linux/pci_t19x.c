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

#include <nvgpu/types.h>

#include <nvgpu/hw/gv11b/hw_usermode_gv11b.h>

#include "common/linux/os_linux.h"

void t19x_nvgpu_pci_init_support(struct nvgpu_os_linux *l)
{
	l->t19x.usermode_regs = l->regs + usermode_cfg0_r();
	l->t19x.usermode_regs_saved = l->t19x.usermode_regs;
}
