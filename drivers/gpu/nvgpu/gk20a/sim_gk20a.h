/*
 * drivers/video/tegra/host/gk20a/sim_gk20a.h
 *
 * GK20A sim support
 *
 * Copyright (c) 2013-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __SIM_GK20A_H__
#define __SIM_GK20A_H__

struct gk20a;

struct sim_gk20a {
	struct gk20a *g;
	u32 send_ring_put;
	u32 recv_ring_get;
	u32 recv_ring_put;
	u32 sequence_base;
	struct nvgpu_mem send_bfr;
	struct nvgpu_mem recv_bfr;
	struct nvgpu_mem msg_bfr;
	void (*remove_support)(struct sim_gk20a *);
	int (*esc_readl)(
		struct gk20a *g, char *path, u32 index, u32 *data);
};

#endif /*__SIM_GK20A_H__*/
