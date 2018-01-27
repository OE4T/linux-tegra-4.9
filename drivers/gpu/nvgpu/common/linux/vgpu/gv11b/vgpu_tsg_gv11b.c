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

#include <nvgpu/vgpu/tegra_vgpu.h>
#include "gk20a/gk20a.h"
#include "common/linux/vgpu/vgpu.h"

#include "vgpu_tsg_gv11b.h"

int vgpu_gv11b_tsg_bind_channel(struct tsg_gk20a *tsg,
				struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_bind_channel_ex_params *p =
				&msg.params.tsg_bind_channel_ex;
	int err;

	gk20a_dbg_fn("");

	err = gk20a_tsg_bind_channel(tsg, ch);
	if (err)
		return err;

	msg.cmd = TEGRA_VGPU_CMD_TSG_BIND_CHANNEL_EX;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	p->ch_handle = ch->virt_ctx;
	p->subctx_id = ch->subctx_id;
	p->runqueue_sel = ch->runqueue_sel;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(tsg->g,
			"vgpu_gv11b_tsg_bind_channel failed, ch %d tsgid %d",
			ch->chid, tsg->tsgid);
		gk20a_tsg_unbind_channel(ch);
	}

	return err;
}
