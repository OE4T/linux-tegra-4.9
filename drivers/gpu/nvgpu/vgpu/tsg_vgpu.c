/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/tegra_vgpu.h>

#include "gk20a/gk20a.h"
#include "gk20a/channel_gk20a.h"
#include "gk20a/platform_gk20a.h"
#include "gk20a/tsg_gk20a.h"
#include "vgpu.h"

#include <nvgpu/bug.h>

static int vgpu_tsg_open(struct tsg_gk20a *tsg)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_open_params *p =
				&msg.params.tsg_open;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_TSG_OPEN;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(tsg->g,
			"vgpu_tsg_open failed, tsgid %d", tsg->tsgid);
	}

	return err;
}

static int vgpu_tsg_bind_channel(struct tsg_gk20a *tsg,
			struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_bind_unbind_channel_params *p =
				&msg.params.tsg_bind_unbind_channel;
	int err;

	gk20a_dbg_fn("");

	err = gk20a_tsg_bind_channel(tsg, ch);
	if (err)
		return err;

	msg.cmd = TEGRA_VGPU_CMD_TSG_BIND_CHANNEL;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	p->ch_handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(tsg->g,
			"vgpu_tsg_bind_channel failed, ch %d tsgid %d",
			ch->chid, tsg->tsgid);
		gk20a_tsg_unbind_channel(ch);
	}

	return err;
}

static int vgpu_tsg_unbind_channel(struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_bind_unbind_channel_params *p =
				&msg.params.tsg_bind_unbind_channel;
	int err;

	gk20a_dbg_fn("");

	err = gk20a_tsg_unbind_channel(ch);
	if (err)
		return err;

	msg.cmd = TEGRA_VGPU_CMD_TSG_UNBIND_CHANNEL;
	msg.handle = vgpu_get_handle(ch->g);
	p->ch_handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);

	return err;
}

static int vgpu_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_tsg_timeslice_params *p =
				&msg.params.tsg_timeslice;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_TSG_SET_TIMESLICE;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	p->timeslice_us = timeslice;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);
	if (!err)
		tsg->timeslice_us = timeslice;

	return err;
}

void vgpu_init_tsg_ops(struct gpu_ops *gops)
{
	gops->fifo.tsg_bind_channel = vgpu_tsg_bind_channel;
	gops->fifo.tsg_unbind_channel = vgpu_tsg_unbind_channel;
	gops->fifo.tsg_set_timeslice = vgpu_tsg_set_timeslice;
	gops->fifo.tsg_open = vgpu_tsg_open;
}
