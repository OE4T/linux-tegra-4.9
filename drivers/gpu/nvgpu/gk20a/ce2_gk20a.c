/*
 * GK20A Graphics Copy Engine  (gr host)
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>

#include "gk20a.h"

#include <nvgpu/log.h>

#include <nvgpu/hw/gk20a/hw_ce2_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ccsr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ram_gk20a.h>
#include <nvgpu/hw/gk20a/hw_top_gk20a.h>
#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>
#include <nvgpu/barrier.h>

static u32 ce2_nonblockpipe_isr(struct gk20a *g, u32 fifo_intr)
{
	gk20a_dbg(gpu_dbg_intr, "ce2 non-blocking pipe interrupt\n");

	return ce2_intr_status_nonblockpipe_pending_f();
}

static u32 ce2_blockpipe_isr(struct gk20a *g, u32 fifo_intr)
{
	gk20a_dbg(gpu_dbg_intr, "ce2 blocking pipe interrupt\n");

	return ce2_intr_status_blockpipe_pending_f();
}

static u32 ce2_launcherr_isr(struct gk20a *g, u32 fifo_intr)
{
	gk20a_dbg(gpu_dbg_intr, "ce2 launch error interrupt\n");

	return ce2_intr_status_launcherr_pending_f();
}

void gk20a_ce2_isr(struct gk20a *g, u32 inst_id, u32 pri_base)
{
	u32 ce2_intr = gk20a_readl(g, ce2_intr_status_r());
	u32 clear_intr = 0;

	gk20a_dbg(gpu_dbg_intr, "ce2 isr %08x\n", ce2_intr);

	/* clear blocking interrupts: they exibit broken behavior */
	if (ce2_intr & ce2_intr_status_blockpipe_pending_f())
		clear_intr |= ce2_blockpipe_isr(g, ce2_intr);

	if (ce2_intr & ce2_intr_status_launcherr_pending_f())
		clear_intr |= ce2_launcherr_isr(g, ce2_intr);

	gk20a_writel(g, ce2_intr_status_r(), clear_intr);
	return;
}

int gk20a_ce2_nonstall_isr(struct gk20a *g, u32 inst_id, u32 pri_base)
{
	int ops = 0;
	u32 ce2_intr = gk20a_readl(g, ce2_intr_status_r());

	gk20a_dbg(gpu_dbg_intr, "ce2 nonstall isr %08x\n", ce2_intr);

	if (ce2_intr & ce2_intr_status_nonblockpipe_pending_f()) {
		gk20a_writel(g, ce2_intr_status_r(),
			ce2_nonblockpipe_isr(g, ce2_intr));
		ops |= (gk20a_nonstall_ops_wakeup_semaphore |
			gk20a_nonstall_ops_post_events);
	}
	return ops;
}

/* static CE app api */
static void gk20a_ce_notify_all_user(struct gk20a *g, u32 event)
{
	struct gk20a_ce_app *ce_app = &g->ce_app;
	struct gk20a_gpu_ctx *ce_ctx, *ce_ctx_save;

	if (!ce_app->initialised)
		return;

	nvgpu_mutex_acquire(&ce_app->app_mutex);

	nvgpu_list_for_each_entry_safe(ce_ctx, ce_ctx_save,
			&ce_app->allocated_contexts, gk20a_gpu_ctx, list) {
		if (ce_ctx->user_event_callback) {
			ce_ctx->user_event_callback(ce_ctx->ctx_id,
				event);
		}
	}

	nvgpu_mutex_release(&ce_app->app_mutex);
}

static void gk20a_ce_finished_ctx_cb(struct channel_gk20a *ch, void *data)
{
	struct gk20a_gpu_ctx *ce_ctx = data;
	bool channel_idle;
	u32 event;

	channel_gk20a_joblist_lock(ch);
	channel_idle = channel_gk20a_joblist_is_empty(ch);
	channel_gk20a_joblist_unlock(ch);

	if (!channel_idle)
		return;

	gk20a_dbg(gpu_dbg_fn, "ce: finished %p", ce_ctx);

	if (ch->has_timedout)
		event = NVGPU_CE_CONTEXT_JOB_TIMEDOUT;
	else
		event = NVGPU_CE_CONTEXT_JOB_COMPLETED;

	if (ce_ctx->user_event_callback)
		ce_ctx->user_event_callback(ce_ctx->ctx_id,
			event);

	++ce_ctx->completed_seq_number;
}

static void gk20a_ce_free_command_buffer_stored_fence(struct gk20a_gpu_ctx *ce_ctx)
{
	u32 cmd_buf_index;
	u32 cmd_buf_read_offset;
	u32 fence_index;
	u32 *cmd_buf_cpu_va;

	for (cmd_buf_index = 0;
		cmd_buf_index < ce_ctx->cmd_buf_end_queue_offset;
		cmd_buf_index++) {
		cmd_buf_read_offset = (cmd_buf_index *
			(NVGPU_CE_MAX_COMMAND_BUFF_SIZE_PER_KICKOFF / sizeof(u32)));

		/* at end of command buffer has gk20a_fence for command buffer sync */
		fence_index = (cmd_buf_read_offset +
			((NVGPU_CE_MAX_COMMAND_BUFF_SIZE_PER_KICKOFF / sizeof(u32)) -
			(NVGPU_CE_MAX_COMMAND_BUFF_SIZE_FOR_TRACING / sizeof(u32))));

		cmd_buf_cpu_va = (u32 *)ce_ctx->cmd_buf_mem.cpu_va;

		/* 0 is treated as invalid pre-sync */
		if (cmd_buf_cpu_va[fence_index]) {
			struct gk20a_fence * ce_cmd_buf_fence_in = NULL;

			memcpy((void *)&ce_cmd_buf_fence_in,
					(void *)(cmd_buf_cpu_va + fence_index),
					sizeof(struct gk20a_fence *));
			gk20a_fence_put(ce_cmd_buf_fence_in);
			/* Reset the stored last pre-sync */
			memset((void *)(cmd_buf_cpu_va + fence_index),
					0,
					NVGPU_CE_MAX_COMMAND_BUFF_SIZE_FOR_TRACING);
		}
	}
}

/* assume this api should need to call under nvgpu_mutex_acquire(&ce_app->app_mutex) */
static void gk20a_ce_delete_gpu_context(struct gk20a_gpu_ctx *ce_ctx)
{
	struct nvgpu_list_node *list = &ce_ctx->list;

	ce_ctx->gpu_ctx_state = NVGPU_CE_GPU_CTX_DELETED;

	nvgpu_mutex_acquire(&ce_ctx->gpu_ctx_mutex);

	if (ce_ctx->cmd_buf_mem.cpu_va) {
		gk20a_ce_free_command_buffer_stored_fence(ce_ctx);
		nvgpu_dma_unmap_free(ce_ctx->vm, &ce_ctx->cmd_buf_mem);
	}

	/* free the channel */
	if (ce_ctx->ch)
		gk20a_channel_close(ce_ctx->ch);

	/* housekeeping on app */
	if (list->prev && list->next)
		nvgpu_list_del(list);

	nvgpu_mutex_release(&ce_ctx->gpu_ctx_mutex);
	nvgpu_mutex_destroy(&ce_ctx->gpu_ctx_mutex);

	nvgpu_kfree(ce_ctx->g, ce_ctx);
}

static inline unsigned int gk20a_ce_get_method_size(int request_operation)
{
	/* failure size */
	unsigned int methodsize = UINT_MAX;

	if (request_operation & NVGPU_CE_PHYS_MODE_TRANSFER)
		methodsize = 10 * 2 * sizeof(u32);
	else if (request_operation & NVGPU_CE_MEMSET)
		methodsize = 9 * 2 * sizeof(u32);

	return methodsize;
}

static inline int gk20a_get_valid_launch_flags(struct gk20a *g, int launch_flags)
{
	/* there is no local memory available,
	don't allow local memory related CE flags */
	if (!g->mm.vidmem.size) {
		launch_flags &= ~(NVGPU_CE_SRC_LOCATION_LOCAL_FB |
			NVGPU_CE_DST_LOCATION_LOCAL_FB);
	}
	return launch_flags;
}

static int gk20a_ce_prepare_submit(u64 src_buf,
		u64 dst_buf,
		u64 size,
		u32 *cmd_buf_cpu_va,
		u32 max_cmd_buf_size,
		unsigned int payload,
		int launch_flags,
		int request_operation,
		u32 dma_copy_class,
		struct gk20a_fence *gk20a_fence_in)
{
	u32 launch = 0;
	u32 methodSize = 0;

	/* failure case handling */
	if ((gk20a_ce_get_method_size(request_operation) > max_cmd_buf_size) ||
		(!size) ||
		(request_operation > NVGPU_CE_MEMSET))
		return 0;

	/* set the channel object */
	cmd_buf_cpu_va[methodSize++] = 0x20018000;
	cmd_buf_cpu_va[methodSize++] = dma_copy_class;

	if (request_operation & NVGPU_CE_PHYS_MODE_TRANSFER) {
		/* setup the source */
		cmd_buf_cpu_va[methodSize++] = 0x20018101;
		cmd_buf_cpu_va[methodSize++] = (u64_lo32(src_buf) &
					NVGPU_CE_LOWER_ADDRESS_OFFSET_MASK);

		cmd_buf_cpu_va[methodSize++] = 0x20018100;
		cmd_buf_cpu_va[methodSize++] = (u64_hi32(src_buf) &
					NVGPU_CE_UPPER_ADDRESS_OFFSET_MASK);

		cmd_buf_cpu_va[methodSize++] = 0x20018098;
		if (launch_flags & NVGPU_CE_SRC_LOCATION_LOCAL_FB) {
			cmd_buf_cpu_va[methodSize++] = 0x00000000;
		} else if (launch_flags & NVGPU_CE_SRC_LOCATION_NONCOHERENT_SYSMEM) {
			cmd_buf_cpu_va[methodSize++] = 0x00000002;
		} else {
			cmd_buf_cpu_va[methodSize++] = 0x00000001;
		}

		launch |= 0x00001000;
	} else if (request_operation & NVGPU_CE_MEMSET) {
		cmd_buf_cpu_va[methodSize++] = 0x200181c2;
		cmd_buf_cpu_va[methodSize++] = 0x00030004;

		cmd_buf_cpu_va[methodSize++] = 0x200181c0;
		cmd_buf_cpu_va[methodSize++] = payload;

		launch |= 0x00000400;

		/* converted into number of words */
		size /= sizeof(u32);
	}

	/* setup the destination/output */
	cmd_buf_cpu_va[methodSize++] = 0x20018103;
	cmd_buf_cpu_va[methodSize++] = (u64_lo32(dst_buf) & NVGPU_CE_LOWER_ADDRESS_OFFSET_MASK);

	cmd_buf_cpu_va[methodSize++] = 0x20018102;
	cmd_buf_cpu_va[methodSize++] = (u64_hi32(dst_buf) & NVGPU_CE_UPPER_ADDRESS_OFFSET_MASK);

	cmd_buf_cpu_va[methodSize++] = 0x20018099;
	if (launch_flags & NVGPU_CE_DST_LOCATION_LOCAL_FB) {
		cmd_buf_cpu_va[methodSize++] = 0x00000000;
	} else if (launch_flags & NVGPU_CE_DST_LOCATION_NONCOHERENT_SYSMEM) {
		cmd_buf_cpu_va[methodSize++] = 0x00000002;
	} else {
		cmd_buf_cpu_va[methodSize++] = 0x00000001;
	}

	launch |= 0x00002000;

	/* setup the format */
	cmd_buf_cpu_va[methodSize++] = 0x20018107;
	cmd_buf_cpu_va[methodSize++] = 1;
	cmd_buf_cpu_va[methodSize++] = 0x20018106;
	cmd_buf_cpu_va[methodSize++] =  u64_lo32(size);

	launch |= 0x00000004;

	if (launch_flags & NVGPU_CE_SRC_MEMORY_LAYOUT_BLOCKLINEAR)
		launch |= 0x00000000;
	else
		launch |= 0x00000080;

	if (launch_flags & NVGPU_CE_DST_MEMORY_LAYOUT_BLOCKLINEAR)
		launch |= 0x00000000;
	else
		launch |= 0x00000100;

	if (launch_flags & NVGPU_CE_DATA_TRANSFER_TYPE_NON_PIPELINED)
		launch |= 0x00000002;
	else
		launch |= 0x00000001;

	cmd_buf_cpu_va[methodSize++] = 0x200180c0;
	cmd_buf_cpu_va[methodSize++] = launch;

	return methodSize;
}

/* global CE app related apis */
int gk20a_init_ce_support(struct gk20a *g)
{
	struct gk20a_ce_app *ce_app = &g->ce_app;
	int err;
	u32 ce_reset_mask;

	ce_reset_mask = gk20a_fifo_get_all_ce_engine_reset_mask(g);

	g->ops.mc.reset(g, ce_reset_mask);

	if (g->ops.clock_gating.slcg_ce2_load_gating_prod)
		g->ops.clock_gating.slcg_ce2_load_gating_prod(g,
				g->slcg_enabled);
	if (g->ops.clock_gating.blcg_ce_load_gating_prod)
		g->ops.clock_gating.blcg_ce_load_gating_prod(g,
				g->blcg_enabled);

	if (ce_app->initialised) {
		/* assume this happen during poweron/poweroff GPU sequence */
		ce_app->app_state = NVGPU_CE_ACTIVE;
		gk20a_ce_notify_all_user(g, NVGPU_CE_CONTEXT_RESUME);
		return 0;
	}

	gk20a_dbg(gpu_dbg_fn, "ce: init");

	err = nvgpu_mutex_init(&ce_app->app_mutex);
	if (err)
		return err;

	nvgpu_mutex_acquire(&ce_app->app_mutex);

	nvgpu_init_list_node(&ce_app->allocated_contexts);
	ce_app->ctx_count = 0;
	ce_app->next_ctx_id = 0;
	ce_app->initialised = true;
	ce_app->app_state = NVGPU_CE_ACTIVE;

	nvgpu_mutex_release(&ce_app->app_mutex);
	gk20a_dbg(gpu_dbg_cde_ctx, "ce: init finished");

	return 0;
}

void gk20a_ce_destroy(struct gk20a *g)
{
	struct gk20a_ce_app *ce_app = &g->ce_app;
	struct gk20a_gpu_ctx *ce_ctx, *ce_ctx_save;

	if (!ce_app->initialised)
		return;

	ce_app->app_state = NVGPU_CE_SUSPEND;
	ce_app->initialised = false;

	nvgpu_mutex_acquire(&ce_app->app_mutex);

	nvgpu_list_for_each_entry_safe(ce_ctx, ce_ctx_save,
			&ce_app->allocated_contexts, gk20a_gpu_ctx, list) {
		gk20a_ce_delete_gpu_context(ce_ctx);
	}

	nvgpu_init_list_node(&ce_app->allocated_contexts);
	ce_app->ctx_count = 0;
	ce_app->next_ctx_id = 0;

	nvgpu_mutex_release(&ce_app->app_mutex);

	nvgpu_mutex_destroy(&ce_app->app_mutex);
}

void gk20a_ce_suspend(struct gk20a *g)
{
	struct gk20a_ce_app *ce_app = &g->ce_app;

	if (!ce_app->initialised)
		return;

	ce_app->app_state = NVGPU_CE_SUSPEND;
	gk20a_ce_notify_all_user(g, NVGPU_CE_CONTEXT_SUSPEND);

	return;
}

/* CE app utility functions */
u32 gk20a_ce_create_context_with_cb(struct gk20a *g,
		int runlist_id,
		int priority,
		int timeslice,
		int runlist_level,
		ce_event_callback user_event_callback)
{
	struct gk20a_gpu_ctx *ce_ctx;
	struct gk20a_ce_app *ce_app = &g->ce_app;
	u32 ctx_id = ~0;
	int err = 0;

	if (!ce_app->initialised || ce_app->app_state != NVGPU_CE_ACTIVE)
		return ctx_id;

	ce_ctx = nvgpu_kzalloc(g, sizeof(*ce_ctx));
	if (!ce_ctx)
		return ctx_id;

	err = nvgpu_mutex_init(&ce_ctx->gpu_ctx_mutex);
	if (err) {
		nvgpu_kfree(g, ce_ctx);
		return ctx_id;
	}

	ce_ctx->g = g;
	ce_ctx->user_event_callback = user_event_callback;

	ce_ctx->cmd_buf_read_queue_offset = 0;
	ce_ctx->cmd_buf_end_queue_offset =
		(NVGPU_CE_COMMAND_BUF_SIZE / NVGPU_CE_MAX_COMMAND_BUFF_SIZE_PER_KICKOFF);

	ce_ctx->submitted_seq_number = 0;
	ce_ctx->completed_seq_number = 0;

	ce_ctx->vm = g->mm.ce.vm;

	/* always kernel client needs privileged channel */
	ce_ctx->ch = gk20a_open_new_channel_with_cb(g, gk20a_ce_finished_ctx_cb,
					ce_ctx,
					runlist_id,
					true);
	if (!ce_ctx->ch) {
		nvgpu_err(g, "ce: gk20a channel not available");
		goto end;
	}
	ce_ctx->ch->wdt_enabled = false;

	/* bind the channel to the vm */
	err = __gk20a_vm_bind_channel(g->mm.ce.vm, ce_ctx->ch);
	if (err) {
		nvgpu_err(g, "ce: could not bind vm");
		goto end;
	}

	/* allocate gpfifo (1024 should be more than enough) */
	err = gk20a_channel_alloc_gpfifo(ce_ctx->ch, 1024, 0, 0);
	if (err) {
		nvgpu_err(g, "ce: unable to allocate gpfifo");
		goto end;
	}

	/* allocate command buffer (4096 should be more than enough) from sysmem*/
	err = nvgpu_dma_alloc_map_sys(ce_ctx->vm, NVGPU_CE_COMMAND_BUF_SIZE, &ce_ctx->cmd_buf_mem);
	 if (err) {
		nvgpu_err(g,
			"ce: could not allocate command buffer for CE context");
		goto end;
	}

	memset(ce_ctx->cmd_buf_mem.cpu_va, 0x00, ce_ctx->cmd_buf_mem.size);

	/* -1 means default channel priority */
	if (priority != -1) {
		err = gk20a_fifo_set_priority(ce_ctx->ch, priority);
		if (err) {
			nvgpu_err(g,
				"ce: could not set the channel priority for CE context");
			goto end;
		}
	}

	/* -1 means default channel timeslice value */
	if (timeslice != -1) {
		err = gk20a_fifo_set_timeslice(ce_ctx->ch, timeslice);
		if (err) {
			nvgpu_err(g,
				"ce: could not set the channel timeslice value for CE context");
			goto end;
		}
	}

	/* -1 means default channel runlist level */
	if (runlist_level != -1) {
		err = gk20a_channel_set_runlist_interleave(ce_ctx->ch, runlist_level);
		if (err) {
			nvgpu_err(g,
				"ce: could not set the runlist interleave for CE context");
			goto end;
		}
	}

	nvgpu_mutex_acquire(&ce_app->app_mutex);
	ctx_id = ce_ctx->ctx_id = ce_app->next_ctx_id;
	nvgpu_list_add(&ce_ctx->list, &ce_app->allocated_contexts);
	++ce_app->next_ctx_id;
	++ce_app->ctx_count;
	nvgpu_mutex_release(&ce_app->app_mutex);

	ce_ctx->gpu_ctx_state = NVGPU_CE_GPU_CTX_ALLOCATED;

end:
	if (ctx_id == (u32)~0) {
		nvgpu_mutex_acquire(&ce_app->app_mutex);
		gk20a_ce_delete_gpu_context(ce_ctx);
		nvgpu_mutex_release(&ce_app->app_mutex);
	}
	return ctx_id;

}
EXPORT_SYMBOL(gk20a_ce_create_context_with_cb);

int gk20a_ce_execute_ops(struct gk20a *g,
		u32 ce_ctx_id,
		u64 src_buf,
		u64 dst_buf,
		u64 size,
		unsigned int payload,
		int launch_flags,
		int request_operation,
		struct gk20a_fence *gk20a_fence_in,
		u32 submit_flags,
		struct gk20a_fence **gk20a_fence_out)
{
	int ret = -EPERM;
	struct gk20a_ce_app *ce_app = &g->ce_app;
	struct gk20a_gpu_ctx *ce_ctx, *ce_ctx_save;
	bool found = false;
	u32 *cmd_buf_cpu_va;
	u64 cmd_buf_gpu_va = 0;
	u32 methodSize;
	u32 cmd_buf_read_offset;
	u32 fence_index;
	struct nvgpu_gpfifo gpfifo;
	struct nvgpu_fence fence = {0,0};
	struct gk20a_fence *ce_cmd_buf_fence_out = NULL;
	struct nvgpu_gpu_characteristics *gpu_capability = &g->gpu_characteristics;

	if (!ce_app->initialised ||ce_app->app_state != NVGPU_CE_ACTIVE)
		goto end;

	nvgpu_mutex_acquire(&ce_app->app_mutex);

	nvgpu_list_for_each_entry_safe(ce_ctx, ce_ctx_save,
			&ce_app->allocated_contexts, gk20a_gpu_ctx, list) {
		if (ce_ctx->ctx_id == ce_ctx_id) {
			found = true;
			break;
		}
	}

	nvgpu_mutex_release(&ce_app->app_mutex);

	if (!found) {
		ret = -EINVAL;
		goto end;
	}

	if (ce_ctx->gpu_ctx_state != NVGPU_CE_GPU_CTX_ALLOCATED) {
		ret = -ENODEV;
		goto end;
	}

	nvgpu_mutex_acquire(&ce_ctx->gpu_ctx_mutex);

	ce_ctx->cmd_buf_read_queue_offset %= ce_ctx->cmd_buf_end_queue_offset;

	cmd_buf_read_offset = (ce_ctx->cmd_buf_read_queue_offset *
			(NVGPU_CE_MAX_COMMAND_BUFF_SIZE_PER_KICKOFF / sizeof(u32)));

	/* at end of command buffer has gk20a_fence for command buffer sync */
	fence_index = (cmd_buf_read_offset +
			((NVGPU_CE_MAX_COMMAND_BUFF_SIZE_PER_KICKOFF / sizeof(u32)) -
			(NVGPU_CE_MAX_COMMAND_BUFF_SIZE_FOR_TRACING / sizeof(u32))));

	if (sizeof(struct gk20a_fence *) > NVGPU_CE_MAX_COMMAND_BUFF_SIZE_FOR_TRACING) {
		ret = -ENOMEM;
		goto noop;
	}

	cmd_buf_cpu_va = (u32 *)ce_ctx->cmd_buf_mem.cpu_va;

	/* 0 is treated as invalid pre-sync */
	if (cmd_buf_cpu_va[fence_index]) {
		struct gk20a_fence * ce_cmd_buf_fence_in = NULL;

		memcpy((void *)&ce_cmd_buf_fence_in,
				(void *)(cmd_buf_cpu_va + fence_index),
				sizeof(struct gk20a_fence *));
		ret = gk20a_fence_wait(g, ce_cmd_buf_fence_in,
				       gk20a_get_gr_idle_timeout(g));

		gk20a_fence_put(ce_cmd_buf_fence_in);
		/* Reset the stored last pre-sync */
		memset((void *)(cmd_buf_cpu_va + fence_index),
				0,
				NVGPU_CE_MAX_COMMAND_BUFF_SIZE_FOR_TRACING);
		if (ret)
			goto noop;
	}

	cmd_buf_gpu_va = (ce_ctx->cmd_buf_mem.gpu_va + (u64)(cmd_buf_read_offset *sizeof(u32)));

	methodSize = gk20a_ce_prepare_submit(src_buf,
					dst_buf,
					size,
					&cmd_buf_cpu_va[cmd_buf_read_offset],
					NVGPU_CE_MAX_COMMAND_BUFF_SIZE_PER_KICKOFF,
					payload,
					gk20a_get_valid_launch_flags(g, launch_flags),
					request_operation,
					gpu_capability->dma_copy_class,
					gk20a_fence_in);

	if (methodSize) {
		/* TODO: Remove CPU pre-fence wait */
		if (gk20a_fence_in) {
			ret = gk20a_fence_wait(g, gk20a_fence_in,
					       gk20a_get_gr_idle_timeout(g));
			gk20a_fence_put(gk20a_fence_in);
			if (ret)
				goto noop;
		}

		/* store the element into gpfifo */
		gpfifo.entry0 =
			u64_lo32(cmd_buf_gpu_va);
		gpfifo.entry1 =
			(u64_hi32(cmd_buf_gpu_va) |
			pbdma_gp_entry1_length_f(methodSize));

		/* take always the postfence as it is needed for protecting the ce context */
		submit_flags |= NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET;

		nvgpu_smp_wmb();

		ret = gk20a_submit_channel_gpfifo(ce_ctx->ch, &gpfifo, NULL,
					1, submit_flags, &fence,
					&ce_cmd_buf_fence_out, false, NULL);

		if (!ret) {
			memcpy((void *)(cmd_buf_cpu_va + fence_index),
					(void *)&ce_cmd_buf_fence_out,
					sizeof(struct gk20a_fence *));

			if (gk20a_fence_out) {
				gk20a_fence_get(ce_cmd_buf_fence_out);
				*gk20a_fence_out = ce_cmd_buf_fence_out;
			}

			/* Next available command buffer queue Index */
			++ce_ctx->cmd_buf_read_queue_offset;
			++ce_ctx->submitted_seq_number;
			}
	} else
		ret = -ENOMEM;
noop:
	nvgpu_mutex_release(&ce_ctx->gpu_ctx_mutex);
end:
	return ret;
}
EXPORT_SYMBOL(gk20a_ce_execute_ops);

void gk20a_ce_delete_context(struct gk20a *g,
		u32 ce_ctx_id)
{
	gk20a_ce_delete_context_priv(g, ce_ctx_id);
}

void gk20a_ce_delete_context_priv(struct gk20a *g,
		u32 ce_ctx_id)
{
	struct gk20a_ce_app *ce_app = &g->ce_app;
	struct gk20a_gpu_ctx *ce_ctx, *ce_ctx_save;

	if (!ce_app->initialised ||ce_app->app_state != NVGPU_CE_ACTIVE)
		return;

	nvgpu_mutex_acquire(&ce_app->app_mutex);

	nvgpu_list_for_each_entry_safe(ce_ctx, ce_ctx_save,
			&ce_app->allocated_contexts, gk20a_gpu_ctx, list) {
		if (ce_ctx->ctx_id == ce_ctx_id) {
			gk20a_ce_delete_gpu_context(ce_ctx);
			--ce_app->ctx_count;
			break;
		}
	}

	nvgpu_mutex_release(&ce_app->app_mutex);
	return;
}
EXPORT_SYMBOL(gk20a_ce_delete_context);
