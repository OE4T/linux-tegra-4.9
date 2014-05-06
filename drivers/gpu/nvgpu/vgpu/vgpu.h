/*
 * Virtualized GPU Interfaces
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _VIRT_H_
#define _VIRT_H_

#include <linux/tegra_gr_comm.h>
#include <linux/tegra_vgpu.h>
#include "gk20a/gk20a.h"

int vgpu_pm_prepare_poweroff(struct device *dev);
int vgpu_pm_finalize_poweron(struct device *dev);
int vgpu_probe(struct platform_device *dev);
int vgpu_remove(struct platform_device *dev);
u64 vgpu_bar1_map(struct gk20a *g, struct sg_table **sgt, u64 size);
int vgpu_gr_isr(struct gk20a *g, struct tegra_vgpu_gr_intr_info *info);
void vgpu_init_fifo_ops(struct gpu_ops *gops);
void vgpu_init_gr_ops(struct gpu_ops *gops);
void vgpu_init_ltc_ops(struct gpu_ops *gops);
void vgpu_init_mm_ops(struct gpu_ops *gops);
int vgpu_init_mm_support(struct gk20a *g);
int vgpu_init_gr_support(struct gk20a *g);
int vgpu_init_fifo_support(struct gk20a *g);

int vgpu_get_attribute(u64 handle, u32 attrib, u32 *value);
int vgpu_comm_sendrecv(struct tegra_vgpu_cmd_msg *msg, size_t size_in,
		size_t size_out);

#endif
