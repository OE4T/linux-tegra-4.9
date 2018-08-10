/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#include "os_linux.h"

#include "os_ops_gm20b.h"
#include "os_ops_gp10b.h"

int nvgpu_init_os_linux_ops(struct nvgpu_os_linux *l)
{
	struct gk20a *g = &l->g;
	u32 ver = g->params.gpu_arch + g->params.gpu_impl;

	switch (ver) {
	case GK20A_GPUID_GM20B:
	case GK20A_GPUID_GM20B_B:
		nvgpu_gm20b_init_os_ops(l);
		break;
	case NVGPU_GPUID_GP10B:
		nvgpu_gp10b_init_os_ops(l);
		break;
	default:
		break;
	}

	return 0;
}
