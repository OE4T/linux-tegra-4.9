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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __XVE_GP106_H__
#define __XVE_GP106_H__

#include "gk20a/gk20a.h"

#include <nvgpu/log2.h>

int gp106_init_xve_ops(struct gpu_ops *gops);

/*
 * Best guess for a reasonable timeout.
 */
#define GPU_XVE_TIMEOUT_MS	500

/*
 * Debugging for the speed change.
 */
enum xv_speed_change_steps {
	PRE_CHANGE = 0,
	DISABLE_ASPM,
	DL_SAFE_MODE,
	CHECK_LINK,
	LINK_SETTINGS,
	EXEC_CHANGE,
	EXEC_VERIF,
	CLEANUP
};

#define xv_dbg(fmt, args...)			\
	gk20a_dbg(gpu_dbg_xv, fmt, ##args)

#define xv_sc_dbg(step, fmt, args...)					\
	xv_dbg("[%d] %15s | " fmt, step, __stringify(step), ##args)

void xve_xve_writel_gp106(struct gk20a *g, u32 reg, u32 val);
u32 xve_xve_readl_gp106(struct gk20a *g, u32 reg);
void xve_reset_gpu_gp106(struct gk20a *g);
int xve_get_speed_gp106(struct gk20a *g, u32 *xve_link_speed);
void xve_disable_aspm_gp106(struct gk20a *g);
int xve_set_speed_gp106(struct gk20a *g, u32 next_link_speed);
void xve_available_speeds_gp106(struct gk20a *g, u32 *speed_mask);
u32 xve_get_link_control_status(struct gk20a *g);
#if defined(CONFIG_PCI_MSI)
void xve_rearm_msi_gp106(struct gk20a *g);
#endif
void xve_enable_shadow_rom_gp106(struct gk20a *g);
void xve_disable_shadow_rom_gp106(struct gk20a *g);

#endif
