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

#include <nvgpu/bus.h>

#include "gk20a/gk20a.h"

int nvgpu_get_timestamps_zipper(struct gk20a *g,
		u32 source_id, u32 count,
		struct nvgpu_cpu_time_correlation_sample *samples)
{
	int err = 0;
	unsigned int i = 0;

	if (source_id != NVGPU_GPU_GET_CPU_TIME_CORRELATION_INFO_SRC_ID_TSC) {
		nvgpu_err(g, "source_id %u not supported", source_id);
		return -EINVAL;
	}

	if (gk20a_busy(g)) {
		nvgpu_err(g, "GPU not powered on\n");
		err = -EINVAL;
		goto end;
	}

	for (i = 0; i < count; i++) {
		err = g->ops.bus.read_ptimer(g, &samples[i].gpu_timestamp);
		if (err)
			return err;

		samples[i].cpu_timestamp = (u64)get_cycles();
	}

end:
	gk20a_idle(g);
	return err;
}
