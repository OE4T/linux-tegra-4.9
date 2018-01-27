/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "gk20a/channel_gk20a.h"
#include "gk20a/tsg_gk20a.h"
#include "common/linux/platform_gk20a.h"
#include "vgpu.h"
#include "fifo_vgpu.h"

#include <nvgpu/bug.h>
#include <nvgpu/vgpu/tegra_vgpu.h>

int vgpu_tsg_open(struct tsg_gk20a *tsg)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_open_rel_params *p =
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

void vgpu_tsg_release(struct tsg_gk20a *tsg)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_open_rel_params *p =
				&msg.params.tsg_release;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_TSG_RELEASE;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(tsg->g,
			"vgpu_tsg_release failed, tsgid %d", tsg->tsgid);
	}
}

int vgpu_enable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry)
		g->ops.fifo.enable_channel(ch);
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);

	return 0;
}

int vgpu_tsg_bind_channel(struct tsg_gk20a *tsg,
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

int vgpu_tsg_unbind_channel(struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_bind_unbind_channel_params *p =
				&msg.params.tsg_bind_unbind_channel;
	int err;

	gk20a_dbg_fn("");

	err = gk20a_fifo_tsg_unbind_channel(ch);
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

int vgpu_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice)
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
