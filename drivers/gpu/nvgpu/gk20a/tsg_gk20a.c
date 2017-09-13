/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/kmem.h>
#include <nvgpu/log.h>

#include "gk20a.h"
#include "tsg_gk20a.h"

bool gk20a_is_channel_marked_as_tsg(struct channel_gk20a *ch)
{
	return !(ch->tsgid == NVGPU_INVALID_TSG_ID);
}

int gk20a_enable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;
	bool is_next, is_ctx_reload;

	gk20a_fifo_disable_tsg_sched(g, tsg);

	/*
	 * Due to h/w bug that exists in Maxwell and Pascal,
	 * we first need to enable all channels with NEXT and CTX_RELOAD set,
	 * and then rest of the channels should be enabled
	 */
	down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		is_next = gk20a_fifo_channel_status_is_next(g, ch->chid);
		is_ctx_reload = gk20a_fifo_channel_status_is_ctx_reload(g, ch->chid);

		if (is_next || is_ctx_reload)
			g->ops.fifo.enable_channel(ch);
	}

	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		is_next = gk20a_fifo_channel_status_is_next(g, ch->chid);
		is_ctx_reload = gk20a_fifo_channel_status_is_ctx_reload(g, ch->chid);

		if (is_next || is_ctx_reload)
			continue;

		g->ops.fifo.enable_channel(ch);
	}
	up_read(&tsg->ch_list_lock);

	gk20a_fifo_enable_tsg_sched(g, tsg);

	return 0;
}

int gk20a_disable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		g->ops.fifo.disable_channel(ch);
	}
	up_read(&tsg->ch_list_lock);

	return 0;
}

static bool gk20a_is_channel_active(struct gk20a *g, struct channel_gk20a *ch)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist;
	unsigned int i;

	for (i = 0; i < f->max_runlists; ++i) {
		runlist = &f->runlist_info[i];
		if (test_bit(ch->chid, runlist->active_channels))
			return true;
	}

	return false;
}

/*
 * API to mark channel as part of TSG
 *
 * Note that channel is not runnable when we bind it to TSG
 */
int gk20a_tsg_bind_channel(struct tsg_gk20a *tsg,
			struct channel_gk20a *ch)
{
	gk20a_dbg_fn("");

	/* check if channel is already bound to some TSG */
	if (gk20a_is_channel_marked_as_tsg(ch)) {
		return -EINVAL;
	}

	/* channel cannot be bound to TSG if it is already active */
	if (gk20a_is_channel_active(tsg->g, ch)) {
		return -EINVAL;
	}

	ch->tsgid = tsg->tsgid;

	/* all the channel part of TSG should need to be same runlist_id */
	if (tsg->runlist_id == FIFO_INVAL_TSG_ID)
		tsg->runlist_id = ch->runlist_id;
	else if (tsg->runlist_id != ch->runlist_id) {
		nvgpu_err(tsg->g,
			"Error: TSG channel should be share same runlist ch[%d] tsg[%d]",
			ch->runlist_id, tsg->runlist_id);
		return -EINVAL;
	}

	down_write(&tsg->ch_list_lock);
	nvgpu_list_add_tail(&ch->ch_entry, &tsg->ch_list);
	up_write(&tsg->ch_list_lock);

	nvgpu_ref_get(&tsg->refcount);

	gk20a_dbg(gpu_dbg_fn, "BIND tsg:%d channel:%d\n",
					tsg->tsgid, ch->chid);

	gk20a_dbg_fn("done");
	return 0;
}

int gk20a_tsg_unbind_channel(struct channel_gk20a *ch)
{
	struct fifo_gk20a *f = &ch->g->fifo;
	struct tsg_gk20a *tsg = &f->tsg[ch->tsgid];

	down_write(&tsg->ch_list_lock);
	nvgpu_list_del(&ch->ch_entry);
	up_write(&tsg->ch_list_lock);

	nvgpu_ref_put(&tsg->refcount, gk20a_tsg_release);

	ch->tsgid = NVGPU_INVALID_TSG_ID;

	return 0;
}

int gk20a_init_tsg_support(struct gk20a *g, u32 tsgid)
{
	struct tsg_gk20a *tsg = NULL;
	int err;

	if (tsgid >= g->fifo.num_channels)
		return -EINVAL;

	tsg = &g->fifo.tsg[tsgid];

	tsg->in_use = false;
	tsg->tsgid = tsgid;

	nvgpu_init_list_node(&tsg->ch_list);
	init_rwsem(&tsg->ch_list_lock);

	nvgpu_init_list_node(&tsg->event_id_list);
	err = nvgpu_mutex_init(&tsg->event_id_list_lock);
	if (err) {
		tsg->in_use = true; /* make this TSG unusable */
		return err;
	}

	return 0;
}

int gk20a_tsg_set_priority(struct gk20a *g, struct tsg_gk20a *tsg,
				u32 priority)
{
	u32 timeslice_us;

	switch (priority) {
	case NVGPU_PRIORITY_LOW:
		timeslice_us = g->timeslice_low_priority_us;
		break;
	case NVGPU_PRIORITY_MEDIUM:
		timeslice_us = g->timeslice_medium_priority_us;
		break;
	case NVGPU_PRIORITY_HIGH:
		timeslice_us = g->timeslice_high_priority_us;
		break;
	default:
		pr_err("Unsupported priority");
		return -EINVAL;
	}

	return gk20a_tsg_set_timeslice(tsg, timeslice_us);
}

int gk20a_tsg_set_runlist_interleave(struct tsg_gk20a *tsg, u32 level)
{
	struct gk20a *g = tsg->g;
	int ret;

	gk20a_dbg(gpu_dbg_sched, "tsgid=%u interleave=%u", tsg->tsgid, level);

	switch (level) {
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_LOW:
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_MEDIUM:
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_HIGH:
		ret = g->ops.fifo.set_runlist_interleave(g, tsg->tsgid,
							true, 0, level);
		if (!ret)
			tsg->interleave_level = level;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret ? ret : g->ops.fifo.update_runlist(g, tsg->runlist_id, ~0, true, true);
}

int gk20a_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice)
{
	struct gk20a *g = tsg->g;

	gk20a_dbg(gpu_dbg_sched, "tsgid=%u timeslice=%u us", tsg->tsgid, timeslice);

	return g->ops.fifo.tsg_set_timeslice(tsg, timeslice);
}

u32 gk20a_tsg_get_timeslice(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;

	if (!tsg->timeslice_us)
		return g->ops.fifo.default_timeslice_us(g);

	return tsg->timeslice_us;
}

static void release_used_tsg(struct fifo_gk20a *f, struct tsg_gk20a *tsg)
{
	nvgpu_mutex_acquire(&f->tsg_inuse_mutex);
	f->tsg[tsg->tsgid].in_use = false;
	nvgpu_mutex_release(&f->tsg_inuse_mutex);
}

static struct tsg_gk20a *gk20a_tsg_acquire_unused_tsg(struct fifo_gk20a *f)
{
	struct tsg_gk20a *tsg = NULL;
	unsigned int tsgid;

	nvgpu_mutex_acquire(&f->tsg_inuse_mutex);
	for (tsgid = 0; tsgid < f->num_channels; tsgid++) {
		if (!f->tsg[tsgid].in_use) {
			f->tsg[tsgid].in_use = true;
			tsg = &f->tsg[tsgid];
			break;
		}
	}
	nvgpu_mutex_release(&f->tsg_inuse_mutex);

	return tsg;
}

struct tsg_gk20a *gk20a_tsg_open(struct gk20a *g)
{
	struct tsg_gk20a *tsg;
	int err;

	tsg = gk20a_tsg_acquire_unused_tsg(&g->fifo);
	if (!tsg)
		return NULL;

	tsg->g = g;
	tsg->num_active_channels = 0;
	nvgpu_ref_init(&tsg->refcount);

	tsg->tsg_gr_ctx = NULL;
	tsg->vm = NULL;
	tsg->interleave_level = NVGPU_RUNLIST_INTERLEAVE_LEVEL_LOW;
	tsg->timeslice_us = 0;
	tsg->timeslice_timeout = 0;
	tsg->timeslice_scale = 0;
	tsg->runlist_id = ~0;
	tsg->tgid = current->tgid;

	if (g->ops.fifo.init_eng_method_buffers)
		g->ops.fifo.init_eng_method_buffers(g, tsg);

	if (g->ops.fifo.tsg_open) {
		err = g->ops.fifo.tsg_open(tsg);
		if (err) {
			nvgpu_err(g, "tsg %d fifo open failed %d",
				  tsg->tsgid, err);
			goto clean_up;
		}
	}

	gk20a_dbg(gpu_dbg_fn, "tsg opened %d\n", tsg->tsgid);

	gk20a_sched_ctrl_tsg_added(g, tsg);

	return tsg;

clean_up:
	nvgpu_ref_put(&tsg->refcount, gk20a_tsg_release);
	return NULL;
}

void gk20a_tsg_release(struct nvgpu_ref *ref)
{
	struct tsg_gk20a *tsg = container_of(ref, struct tsg_gk20a, refcount);
	struct gk20a *g = tsg->g;
	struct gk20a_event_id_data *event_id_data, *event_id_data_temp;

	if (tsg->tsg_gr_ctx) {
		gr_gk20a_free_tsg_gr_ctx(tsg);
		tsg->tsg_gr_ctx = NULL;
	}

	if (g->ops.fifo.deinit_eng_method_buffers)
		g->ops.fifo.deinit_eng_method_buffers(g, tsg);

	if (tsg->vm) {
		nvgpu_vm_put(tsg->vm);
		tsg->vm = NULL;
	}

	gk20a_sched_ctrl_tsg_removed(g, tsg);

	/* unhook all events created on this TSG */
	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	nvgpu_list_for_each_entry_safe(event_id_data, event_id_data_temp,
				&tsg->event_id_list,
				gk20a_event_id_data,
				event_id_node) {
		nvgpu_list_del(&event_id_data->event_id_node);
	}
	nvgpu_mutex_release(&tsg->event_id_list_lock);

	release_used_tsg(&g->fifo, tsg);

	tsg->runlist_id = ~0;

	gk20a_dbg(gpu_dbg_fn, "tsg released %d\n", tsg->tsgid);
	gk20a_put(g);
}

struct tsg_gk20a *tsg_gk20a_from_ch(struct channel_gk20a *ch)
{
	struct tsg_gk20a *tsg = NULL;

	if (gk20a_is_channel_marked_as_tsg(ch)) {
		struct gk20a *g = ch->g;
		struct fifo_gk20a *f = &g->fifo;
		tsg = &f->tsg[ch->tsgid];
	}

	return tsg;
}
