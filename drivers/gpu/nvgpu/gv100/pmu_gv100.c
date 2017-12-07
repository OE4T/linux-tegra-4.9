/*
 * GV100 PMU
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/pmu.h>

#include "gk20a/gk20a.h"

int gv100_pmu_init_acr(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct nv_pmu_rpc_struct_acr_init_wpr_region rpc;
	int status = 0;

	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_struct_acr_init_wpr_region));
	rpc.wpr_regionId = 0x1;
	rpc.wpr_offset = 0x0;
	PMU_RPC_EXECUTE(status, pmu, ACR, INIT_WPR_REGION, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC status=0x%x",
			status);
	}

	return status;
}
