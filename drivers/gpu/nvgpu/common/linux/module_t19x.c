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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <nvgpu/types.h>

#include <nvgpu/hw/gv11b/hw_usermode_gv11b.h>

#include "common/linux/os_linux.h"

/*
 * Locks out the driver from accessing GPU registers. This prevents access to
 * thse registers after the GPU has been clock or power gated. This should help
 * find annoying bugs where register reads and writes are silently dropped
 * after the GPU has been turned off. On older chips these reads and writes can
 * also lock the entire CPU up.
 */
void t19x_lockout_registers(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	l->t19x.usermode_regs = NULL;
}

/*
 * Undoes t19x_lockout_registers().
 */
void t19x_restore_registers(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	l->t19x.usermode_regs = l->t19x.usermode_regs_saved;
}

void t19x_remove_support(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (l->t19x.usermode_regs) {
		l->t19x.usermode_regs = NULL;
	}
}

void t19x_init_support(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	l->t19x.usermode_regs = l->regs + usermode_cfg0_r();
	l->t19x.usermode_regs_saved = l->t19x.usermode_regs;
}
