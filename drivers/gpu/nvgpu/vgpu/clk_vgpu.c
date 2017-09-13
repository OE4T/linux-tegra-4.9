/*
 * Virtualized GPU Clock Interface
 *
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

#include "vgpu/vgpu.h"
#include "vgpu/clk_vgpu.h"

static unsigned long
vgpu_freq_table[TEGRA_VGPU_GPU_FREQ_TABLE_SIZE];

static unsigned long vgpu_clk_get_rate(struct gk20a *g, u32 api_domain)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_gpu_clk_rate_params *p = &msg.params.gpu_clk_rate;
	int err;
	unsigned long ret = 0;

	gk20a_dbg_fn("");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		msg.cmd = TEGRA_VGPU_CMD_GET_GPU_CLK_RATE;
		msg.handle = vgpu_get_handle(g);
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		err = err ? err : msg.ret;
		if (err)
			nvgpu_err(g, "%s failed - %d", __func__, err);
		else
			/* return frequency in Hz */
			ret = p->rate * 1000;
		break;
	case CTRL_CLK_DOMAIN_PWRCLK:
		nvgpu_err(g, "unsupported clock: %u", api_domain);
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		break;
	}

	return ret;
}

static int vgpu_clk_set_rate(struct gk20a *g,
				u32 api_domain, unsigned long rate)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_gpu_clk_rate_params *p = &msg.params.gpu_clk_rate;
	int err = -EINVAL;

	gk20a_dbg_fn("");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		msg.cmd = TEGRA_VGPU_CMD_SET_GPU_CLK_RATE;
		msg.handle = vgpu_get_handle(g);

		/* server dvfs framework requires frequency in kHz */
		p->rate = (u32)(rate / 1000);
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		err = err ? err : msg.ret;
		if (err)
			nvgpu_err(g, "%s failed - %d", __func__, err);
		break;
	case CTRL_CLK_DOMAIN_PWRCLK:
		nvgpu_err(g, "unsupported clock: %u", api_domain);
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		break;
	}

	return err;
}

void vgpu_init_clk_support(struct gk20a *g)
{
	g->ops.clk.get_rate = vgpu_clk_get_rate;
	g->ops.clk.set_rate = vgpu_clk_set_rate;
}

long vgpu_clk_round_rate(struct device *dev, unsigned long rate)
{
	/* server will handle frequency rounding */
	return rate;
}

int vgpu_clk_get_freqs(struct device *dev,
		unsigned long **freqs, int *num_freqs)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_get_gpu_freq_table_params *p =
					&msg.params.get_gpu_freq_table;
	unsigned int i;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_GET_GPU_FREQ_TABLE;
	msg.handle = vgpu_get_handle(g);

	p->num_freqs = TEGRA_VGPU_GPU_FREQ_TABLE_SIZE;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(g, "%s failed - %d", __func__, err);
		return err;
	}

	/* return frequency in Hz */
	for (i = 0; i < p->num_freqs; i++)
		vgpu_freq_table[i] = p->freqs[i] * 1000;

	*freqs = vgpu_freq_table;
	*num_freqs = p->num_freqs;

	return 0;
}

int vgpu_clk_cap_rate(struct device *dev, unsigned long rate)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_gpu_clk_rate_params *p = &msg.params.gpu_clk_rate;
	int err = 0;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CAP_GPU_CLK_RATE;
	msg.handle = vgpu_get_handle(g);
	p->rate = (u32)rate;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(g, "%s failed - %d", __func__, err);
		return err;
	}

	return 0;
}
