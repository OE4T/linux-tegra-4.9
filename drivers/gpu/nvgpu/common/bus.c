/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
