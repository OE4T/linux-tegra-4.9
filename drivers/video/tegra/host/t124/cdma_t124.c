/*
 * drivers/video/tegra/host/t124/cdma_t124.c
 *
 * Tegra Graphics Host Command DMA
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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

#include "nvhost_cdma.h"
#include "dev.h"

#include "t124.h"
#include "hardware_t124.h"
#include "host1x/hw_host1x02_sync.h"
#include "host1x/hw_host1x02_channel.h"

#include "gk20a/gk20a.h"
#include "gk20a/cdma_gk20a.h"

#include "chip_support.h"

#include "host1x/host1x_cdma.c"

static void t124_push_buffer_reset(struct push_buffer *pb)
{
#if defined(CONFIG_TEGRA_GK20A)
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	nvhost_dbg_fn("");

	if (ch->dev == &tegra_gk20a_device)
		gk20a_push_buffer_reset(pb);
	else
#endif
		host1x_pushbuffer_ops.reset(pb);
}

/**
 * Init push buffer resources
 */
static int t124_push_buffer_init(struct push_buffer *pb)
{
#if defined(CONFIG_TEGRA_GK20A)
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	nvhost_dbg_fn("");

	if (ch->dev == &tegra_gk20a_device)
		return gk20a_push_buffer_init(pb);
	else
#endif
		return host1x_pushbuffer_ops.init(pb);
}

/**
 * Clean up push buffer resources
 */
static void t124_push_buffer_destroy(struct push_buffer *pb)
{
#if defined(CONFIG_TEGRA_GK20A)
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	nvhost_dbg_fn("");

	if (ch->dev == &tegra_gk20a_device)
		gk20a_push_buffer_destroy(pb);
	else
#endif
		host1x_pushbuffer_ops.destroy(pb);
}

/**
 * Push two words to the push buffer
 * Caller must ensure push buffer is not full
 */
static void t124_push_buffer_push_to(struct push_buffer *pb,
			    struct mem_mgr *client,
			    struct mem_handle *handle, u32 op1, u32 op2)
{
#if defined(CONFIG_TEGRA_GK20A)
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	nvhost_dbg_fn("");

	if (ch->dev == &tegra_gk20a_device)
		gk20a_push_buffer_push_to(pb, client, handle, op1, op2);
	else
#endif
		host1x_pushbuffer_ops.push_to(pb, client, handle, op1, op2);
}

/**
 * Pop a number of two word slots from the push buffer
 * Caller must ensure push buffer is not empty
 */
static void t124_push_buffer_pop_from(struct push_buffer *pb, unsigned int slots)
{
#if defined(CONFIG_TEGRA_GK20A)
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	nvhost_dbg_fn("");

	if (ch->dev == &tegra_gk20a_device)
		gk20a_push_buffer_pop_from(pb, slots);
	else
#endif
		host1x_pushbuffer_ops.pop_from(pb, slots);
}

/**
 * Return the number of two word slots free in the push buffer
 */
static u32 t124_push_buffer_space(struct push_buffer *pb)
{
#if defined(CONFIG_TEGRA_GK20A)
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	nvhost_dbg_fn("");

	if (ch->dev == &tegra_gk20a_device)
		return gk20a_push_buffer_space(pb);
	else
#endif
		return host1x_pushbuffer_ops.space(pb);
}

static u32 t124_push_buffer_putptr(struct push_buffer *pb)
{
#if defined(CONFIG_TEGRA_GK20A)
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	nvhost_dbg_fn("");

	if (ch->dev == &tegra_gk20a_device)
		return gk20a_push_buffer_putptr(pb);
	else
#endif
		return host1x_pushbuffer_ops.putptr(pb);
}

/*
 * The syncpt incr buffer is filled with methods to increment syncpts, which
 * is later GATHER-ed into the mainline PB. It's used when a timed out context
 * is interleaved with other work, so needs to inline the syncpt increments
 * to maintain the count (but otherwise does no work).
 */

/**
 * Init timeout and syncpt incr buffer resources
 */
static int t124_cdma_timeout_init(struct nvhost_cdma *cdma,
				 u32 syncpt_id)
{
	nvhost_dbg_fn("");

#if defined(CONFIG_TEGRA_GK20A)
	if (cdma_to_channel(cdma)->dev == &tegra_gk20a_device)
		return -EINVAL; /*XXX gk20a_cdma_timeout_init(cdma, syncpt_id);*/
	else
#endif
		return host1x_cdma_ops.timeout_init(cdma, syncpt_id);
}
static void t124_cdma_timeout_destroy(struct nvhost_cdma *cdma)
{
	nvhost_dbg_fn("");

#if defined(CONFIG_TEGRA_GK20A)
	if (cdma_to_channel(cdma)->dev == &tegra_gk20a_device)
		return; /*XXX gk20a_cdma_timeout_destroy(cdma);*/
	else
#endif
		host1x_cdma_ops.timeout_destroy(cdma);
}

void t124_cdma_timeout_teardown_begin(struct nvhost_cdma *cdma)
{
	nvhost_dbg_fn("");

#if defined(CONFIG_TEGRA_GK20A)
	if (cdma_to_channel(cdma)->dev == &tegra_gk20a_device)
		return; /*XXX gk20a_cdma_teardown_begin(cdma);*/
	else
#endif
		host1x_cdma_ops.timeout_teardown_begin(cdma);
}

void t124_cdma_timeout_teardown_end(struct nvhost_cdma *cdma, u32 getptr)
{
	nvhost_dbg_fn("");

#if defined(CONFIG_TEGRA_GK20A)
	if (cdma_to_channel(cdma)->dev == &tegra_gk20a_device)
		return; /*XXX gk20a_cdma_teardown_end(cdma, getptr);*/
	else
#endif
		host1x_cdma_ops.timeout_teardown_end(cdma, getptr);
}
static void t124_cdma_timeout_cpu_incr(struct nvhost_cdma *cdma, u32 getptr,
       u32 syncpt_incrs, u32 syncval, u32 nr_slots, u32 waitbases)
{
	nvhost_dbg_fn("");

#if defined(CONFIG_TEGRA_GK20A)
	if (cdma_to_channel(cdma)->dev == &tegra_gk20a_device)
		return; /*XXX gk20a_cdma_timeout_cpu_incr(cdma, getptr,
			  syncpt_incrs, syncval, nr_slots);*/
	else
#endif
		host1x_cdma_ops.timeout_cpu_incr(cdma, getptr, syncpt_incrs,
					  syncval, nr_slots, waitbases);
}

/**
 * Start channel DMA
 */
static void t124_cdma_start(struct nvhost_cdma *cdma)
{
	nvhost_dbg_fn("");

#if defined(CONFIG_TEGRA_GK20A)
	if (cdma_to_channel(cdma)->dev == &tegra_gk20a_device)
		gk20a_cdma_start(cdma);
	else
#endif
		host1x_cdma_ops.start(cdma);
}

/**
 * Kick channel DMA into action by writing its PUT offset (if it has changed)
 */
static void t124_cdma_kick(struct nvhost_cdma *cdma)
{
	nvhost_dbg_fn("");

#if defined(CONFIG_TEGRA_GK20A)
	if (cdma_to_channel(cdma)->dev == &tegra_gk20a_device)
		gk20a_cdma_kick(cdma);
	else
#endif
		host1x_cdma_ops.kick(cdma);
}

static void t124_cdma_stop(struct nvhost_cdma *cdma)
{
	nvhost_dbg_fn("");

#if defined(CONFIG_TEGRA_GK20A)
	if (cdma_to_channel(cdma)->dev == &tegra_gk20a_device)
		gk20a_cdma_stop(cdma);
	else
#endif
		host1x_cdma_ops.stop(cdma);
}

int nvhost_init_t124_cdma_support(struct nvhost_chip_support *op)
{
	/* i'm of half a mind to call t30 init here and crib its
	   pointers and call them only for host1x channels, and
	   call out to gk20a stuff for gr
	*/

	nvhost_dbg_fn("");

	op->cdma.start = t124_cdma_start;
	op->cdma.stop = t124_cdma_stop;
	op->cdma.kick = t124_cdma_kick;

	op->cdma.timeout_init = t124_cdma_timeout_init;
	op->cdma.timeout_destroy = t124_cdma_timeout_destroy;
	op->cdma.timeout_teardown_begin = t124_cdma_timeout_teardown_begin;
	op->cdma.timeout_teardown_end = t124_cdma_timeout_teardown_end;
	op->cdma.timeout_cpu_incr = t124_cdma_timeout_cpu_incr;

	op->push_buffer.reset = t124_push_buffer_reset;
	op->push_buffer.init = t124_push_buffer_init;
	op->push_buffer.destroy = t124_push_buffer_destroy;
	op->push_buffer.push_to = t124_push_buffer_push_to;
	op->push_buffer.pop_from = t124_push_buffer_pop_from;
	op->push_buffer.space = t124_push_buffer_space;
	op->push_buffer.putptr = t124_push_buffer_putptr;

	return 0;
}
