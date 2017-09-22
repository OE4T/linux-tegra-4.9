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

#include <nvgpu/pmu.h>
#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include <nvgpu/enabled.h>
#include <nvgpu/barrier.h>

#include "gk20a/gk20a.h"

#define PMU_MEM_SCRUBBING_TIMEOUT_MAX 1000
#define PMU_MEM_SCRUBBING_TIMEOUT_DEFAULT 10

static int nvgpu_pg_init_task(void *arg);

static int pmu_enable_hw(struct nvgpu_pmu *pmu, bool enable)
{
	struct gk20a *g = pmu->g;
	struct nvgpu_timeout timeout;
	int err = 0;

	nvgpu_log_fn(g, " %s ", g->name);

	if (enable) {
		/* bring PMU falcon/engine out of reset */
		g->ops.pmu.reset_engine(g, true);

		if (g->ops.clock_gating.slcg_pmu_load_gating_prod)
			g->ops.clock_gating.slcg_pmu_load_gating_prod(g,
				g->slcg_enabled);

		if (g->ops.clock_gating.blcg_pmu_load_gating_prod)
			g->ops.clock_gating.blcg_pmu_load_gating_prod(g,
				g->blcg_enabled);

		/* check for PMU IMEM/DMEM scrubbing complete status */
		nvgpu_timeout_init(g, &timeout,
			PMU_MEM_SCRUBBING_TIMEOUT_MAX /
			PMU_MEM_SCRUBBING_TIMEOUT_DEFAULT,
			NVGPU_TIMER_RETRY_TIMER);
		do {
			if (nvgpu_flcn_get_mem_scrubbing_status(pmu->flcn))
				goto exit;

			nvgpu_udelay(PMU_MEM_SCRUBBING_TIMEOUT_DEFAULT);
		} while (!nvgpu_timeout_expired(&timeout));

		/* keep PMU falcon/engine in reset
		*  if IMEM/DMEM scrubbing fails
		*/
		g->ops.pmu.reset_engine(g, false);
		nvgpu_err(g, "Falcon mem scrubbing timeout");
		err = -ETIMEDOUT;
	} else
		/* keep PMU falcon/engine in reset */
		g->ops.pmu.reset_engine(g, false);

exit:
	nvgpu_log_fn(g, "%s Done, status - %d ", g->name, err);
	return err;
}

static int pmu_enable(struct nvgpu_pmu *pmu, bool enable)
{
	struct gk20a *g = pmu->g;
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (!enable) {
		if (!g->ops.pmu.is_engine_in_reset(g)) {
			pmu_enable_irq(pmu, false);
			pmu_enable_hw(pmu, false);
		}
	} else {
		err = pmu_enable_hw(pmu, true);
		if (err)
			goto exit;

		err = nvgpu_flcn_wait_idle(pmu->flcn);
		if (err)
			goto exit;

		pmu_enable_irq(pmu, true);
	}

exit:
	nvgpu_log_fn(g, "Done, status - %d ", err);
	return err;
}

int nvgpu_pmu_reset(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	int err = 0;

	nvgpu_log_fn(g, " %s ", g->name);

	err = nvgpu_flcn_wait_idle(pmu->flcn);
	if (err)
		goto exit;

	err = pmu_enable(pmu, false);
	if (err)
		goto exit;

	err = pmu_enable(pmu, true);

exit:
	nvgpu_log_fn(g, " %s Done, status - %d ", g->name, err);
	return err;
}

static int nvgpu_init_task_pg_init(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	char thread_name[64];
	int err = 0;

	nvgpu_log_fn(g, " ");

	nvgpu_cond_init(&pmu->pg_init.wq);

	snprintf(thread_name, sizeof(thread_name),
				"nvgpu_pg_init_%s", g->name);

	err = nvgpu_thread_create(&pmu->pg_init.state_task, g,
			nvgpu_pg_init_task, thread_name);
	if (err)
		nvgpu_err(g, "failed to start nvgpu_pg_init thread");

	return err;
}

static int nvgpu_init_pmu_setup_sw(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	unsigned int i;
	int err = 0;
	u8 *ptr;

	nvgpu_log_fn(g, " ");

	/* start with elpg disabled until first enable call */
	pmu->elpg_refcnt = 0;

	/* Create thread to handle PMU state machine */
	nvgpu_init_task_pg_init(g);

	if (pmu->sw_ready) {
		for (i = 0; i < pmu->mutex_cnt; i++) {
			pmu->mutex[i].id    = i;
			pmu->mutex[i].index = i;
		}
		nvgpu_pmu_seq_init(pmu);

		nvgpu_log_fn(g, "skip init");
		goto skip_init;
	}

	/* no infoRom script from vbios? */

	/* TBD: sysmon subtask */

	if (IS_ENABLED(CONFIG_TEGRA_GK20A_PERFMON))
		pmu->perfmon_sampling_enabled = true;

	pmu->mutex_cnt = g->ops.pmu.pmu_mutex_size();
	pmu->mutex = nvgpu_kzalloc(g, pmu->mutex_cnt *
		sizeof(struct pmu_mutex));
	if (!pmu->mutex) {
		err = -ENOMEM;
		goto err;
	}

	for (i = 0; i < pmu->mutex_cnt; i++) {
		pmu->mutex[i].id    = i;
		pmu->mutex[i].index = i;
	}

	pmu->seq = nvgpu_kzalloc(g, PMU_MAX_NUM_SEQUENCES *
		sizeof(struct pmu_sequence));
	if (!pmu->seq) {
		err = -ENOMEM;
		goto err_free_mutex;
	}

	nvgpu_pmu_seq_init(pmu);

	err = nvgpu_dma_alloc_map_sys(vm, GK20A_PMU_SEQ_BUF_SIZE,
			&pmu->seq_buf);
	if (err) {
		nvgpu_err(g, "failed to allocate memory");
		goto err_free_seq;
	}

	ptr = (u8 *)pmu->seq_buf.cpu_va;

	/* TBD: remove this if ZBC save/restore is handled by PMU
	 * end an empty ZBC sequence for now
	 */
	ptr[0] = 0x16; /* opcode EXIT */
	ptr[1] = 0; ptr[2] = 1; ptr[3] = 0;
	ptr[4] = 0; ptr[5] = 0; ptr[6] = 0; ptr[7] = 0;

	pmu->seq_buf.size = GK20A_PMU_SEQ_BUF_SIZE;

	err = nvgpu_dma_alloc_map(vm, GK20A_PMU_TRACE_BUFSIZE,
			&pmu->trace_buf);
	if (err) {
		nvgpu_err(g, "failed to allocate pmu trace buffer\n");
		goto err_free_seq_buf;
	}

	pmu->sw_ready = true;

skip_init:
	nvgpu_log_fn(g, "done");
	return 0;
 err_free_seq_buf:
	nvgpu_dma_unmap_free(vm, &pmu->seq_buf);
 err_free_seq:
	nvgpu_kfree(g, pmu->seq);
 err_free_mutex:
	nvgpu_kfree(g, pmu->mutex);
 err:
	nvgpu_log_fn(g, "fail");
	return err;
}

int nvgpu_init_pmu_support(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	u32 err;

	nvgpu_log_fn(g, " ");

	if (pmu->initialized)
		return 0;

	err = pmu_enable_hw(pmu, true);
	if (err)
		return err;

	if (g->support_pmu) {
		err = nvgpu_init_pmu_setup_sw(g);
		if (err)
			return err;
		err = g->ops.pmu.pmu_setup_hw_and_bootstrap(g);
		if (err)
			return err;

		nvgpu_pmu_state_change(g, PMU_STATE_STARTING, false);
	}

	return err;
}

int nvgpu_pmu_process_init_msg(struct nvgpu_pmu *pmu,
			struct pmu_msg *msg)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_v *pv = &g->ops.pmu_ver;
	union pmu_init_msg_pmu *init;
	struct pmu_sha1_gid_data gid_data;
	u32 i, tail = 0;

	nvgpu_log_fn(g, " ");

	nvgpu_pmu_dbg(g, "init received\n");

	g->ops.pmu.pmu_msgq_tail(pmu, &tail, QUEUE_GET);

	nvgpu_flcn_copy_from_dmem(pmu->flcn, tail,
		(u8 *)&msg->hdr, PMU_MSG_HDR_SIZE, 0);
	if (msg->hdr.unit_id != PMU_UNIT_INIT) {
		nvgpu_err(g, "expecting init msg");
		return -EINVAL;
	}

	nvgpu_flcn_copy_from_dmem(pmu->flcn, tail + PMU_MSG_HDR_SIZE,
		(u8 *)&msg->msg, msg->hdr.size - PMU_MSG_HDR_SIZE, 0);

	if (msg->msg.init.msg_type != PMU_INIT_MSG_TYPE_PMU_INIT) {
		nvgpu_err(g, "expecting init msg");
		return -EINVAL;
	}

	tail += ALIGN(msg->hdr.size, PMU_DMEM_ALIGNMENT);
	g->ops.pmu.pmu_msgq_tail(pmu, &tail, QUEUE_SET);

	init = pv->get_pmu_msg_pmu_init_msg_ptr(&(msg->msg.init));
	if (!pmu->gid_info.valid) {

		nvgpu_flcn_copy_from_dmem(pmu->flcn,
			pv->get_pmu_init_msg_pmu_sw_mg_off(init),
			(u8 *)&gid_data,
			sizeof(struct pmu_sha1_gid_data), 0);

		pmu->gid_info.valid =
			(*(u32 *)gid_data.signature == PMU_SHA1_GID_SIGNATURE);

		if (pmu->gid_info.valid) {

			BUG_ON(sizeof(pmu->gid_info.gid) !=
				sizeof(gid_data.gid));

			memcpy(pmu->gid_info.gid, gid_data.gid,
				sizeof(pmu->gid_info.gid));
		}
	}

	for (i = 0; i < PMU_QUEUE_COUNT; i++)
		nvgpu_pmu_queue_init(pmu, i, init);

	if (!nvgpu_alloc_initialized(&pmu->dmem)) {
		/* Align start and end addresses */
		u32 start = ALIGN(pv->get_pmu_init_msg_pmu_sw_mg_off(init),
			PMU_DMEM_ALLOC_ALIGNMENT);
		u32 end = (pv->get_pmu_init_msg_pmu_sw_mg_off(init) +
			pv->get_pmu_init_msg_pmu_sw_mg_size(init)) &
			~(PMU_DMEM_ALLOC_ALIGNMENT - 1);
		u32 size = end - start;

		nvgpu_bitmap_allocator_init(g, &pmu->dmem, "gk20a_pmu_dmem",
			start, size, PMU_DMEM_ALLOC_ALIGNMENT, 0);
	}

	pmu->pmu_ready = true;

	nvgpu_pmu_state_change(g, PMU_STATE_INIT_RECEIVED, true);

	nvgpu_pmu_dbg(g, "init received end\n");

	return 0;
}

static void pmu_setup_hw_enable_elpg(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;

	nvgpu_log_fn(g, " ");

	pmu->initialized = true;
	nvgpu_pmu_state_change(g, PMU_STATE_STARTED, true);

	if (nvgpu_is_enabled(g, NVGPU_PMU_ZBC_SAVE)) {
		/* Save zbc table after PMU is initialized. */
		pmu->zbc_ready = true;
		gk20a_pmu_save_zbc(g, 0xf);
	}

	if (g->elpg_enabled) {
		/* Init reg with prod values*/
		if (g->ops.pmu.pmu_setup_elpg)
			g->ops.pmu.pmu_setup_elpg(g);
		nvgpu_pmu_enable_elpg(g);
	}

	nvgpu_udelay(50);

	/* Enable AELPG */
	if (g->aelpg_enabled) {
		nvgpu_aelpg_init(g);
		nvgpu_aelpg_init_and_enable(g, PMU_AP_CTRL_ID_GRAPHICS);
	}
}

void nvgpu_pmu_state_change(struct gk20a *g, u32 pmu_state,
		bool post_change_event)
{
	struct nvgpu_pmu *pmu = &g->pmu;

	nvgpu_pmu_dbg(g, "pmu_state - %d", pmu_state);

	pmu->pmu_state = pmu_state;

	if (post_change_event) {
		pmu->pg_init.state_change = true;
		nvgpu_cond_signal(&pmu->pg_init.wq);
	}

	/* make status visible */
	nvgpu_smp_mb();
}

static int nvgpu_pg_init_task(void *arg)
{
	struct gk20a *g = (struct gk20a *)arg;
	struct nvgpu_pmu *pmu = &g->pmu;
	struct nvgpu_pg_init *pg_init = &pmu->pg_init;
	u32 pmu_state = 0;

	nvgpu_log_fn(g, "thread start");

	while (true) {

		NVGPU_COND_WAIT_INTERRUPTIBLE(&pg_init->wq,
			(pg_init->state_change == true), 0);

		pmu->pg_init.state_change = false;
		pmu_state = NV_ACCESS_ONCE(pmu->pmu_state);

		if (pmu_state == PMU_STATE_EXIT) {
			nvgpu_pmu_dbg(g, "pmu state exit");
			break;
		}

		switch (pmu_state) {
		case PMU_STATE_INIT_RECEIVED:
			nvgpu_pmu_dbg(g, "pmu starting");
			if (g->can_elpg)
				nvgpu_pmu_init_powergating(g);
			break;
		case PMU_STATE_ELPG_BOOTED:
			nvgpu_pmu_dbg(g, "elpg booted");
			nvgpu_pmu_init_bind_fecs(g);
			break;
		case PMU_STATE_LOADING_PG_BUF:
			nvgpu_pmu_dbg(g, "loaded pg buf");
			nvgpu_pmu_setup_hw_load_zbc(g);
			break;
		case PMU_STATE_LOADING_ZBC:
			nvgpu_pmu_dbg(g, "loaded zbc");
			pmu_setup_hw_enable_elpg(g);
			break;
		case PMU_STATE_STARTED:
			nvgpu_pmu_dbg(g, "PMU booted, thread exiting");
			return 0;
		default:
			nvgpu_pmu_dbg(g, "invalid state");
			break;
		}

	}

	while (!nvgpu_thread_should_stop(&pg_init->state_task))
		nvgpu_usleep_range(5000, 5100);

	nvgpu_log_fn(g, "thread exit");

	return 0;
}

int nvgpu_pmu_destroy(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_pg_stats_data pg_stat_data = { 0 };
	struct nvgpu_timeout timeout;
	int i;

	nvgpu_log_fn(g, " ");

	if (!g->support_pmu)
		return 0;

	/* make sure the pending operations are finished before we continue */
	if (nvgpu_thread_is_running(&pmu->pg_init.state_task)) {

		/* post PMU_STATE_EXIT to exit PMU state machine loop */
		nvgpu_pmu_state_change(g, PMU_STATE_EXIT, true);

		/* Make thread stop*/
		nvgpu_thread_stop(&pmu->pg_init.state_task);

		/* wait to confirm thread stopped */
		nvgpu_timeout_init(g, &timeout, 1000, NVGPU_TIMER_RETRY_TIMER);
		do {
			if (!nvgpu_thread_is_running(&pmu->pg_init.state_task))
				break;
			nvgpu_udelay(2);
		} while (!nvgpu_timeout_expired_msg(&timeout,
			"timeout - waiting PMU state machine thread stop"));
	}

	nvgpu_pmu_get_pg_stats(g,
		PMU_PG_ELPG_ENGINE_ID_GRAPHICS,	&pg_stat_data);

	if (nvgpu_pmu_disable_elpg(g))
		nvgpu_err(g, "failed to set disable elpg");
	pmu->initialized = false;

	/* update the s/w ELPG residency counters */
	g->pg_ingating_time_us += (u64)pg_stat_data.ingating_time;
	g->pg_ungating_time_us += (u64)pg_stat_data.ungating_time;
	g->pg_gating_cnt += pg_stat_data.gating_cnt;

	nvgpu_mutex_acquire(&pmu->isr_mutex);
	pmu->isr_enabled = false;
	nvgpu_mutex_release(&pmu->isr_mutex);

	for (i = 0; i < PMU_QUEUE_COUNT; i++)
		nvgpu_mutex_destroy(&pmu->queue[i].mutex);

	nvgpu_pmu_state_change(g, PMU_STATE_OFF, false);
	pmu->pmu_ready = false;
	pmu->perfmon_ready = false;
	pmu->zbc_ready = false;
	g->pmu_lsf_pmu_wpr_init_done = false;
	__nvgpu_set_enabled(g, NVGPU_PMU_FECS_BOOTSTRAP_DONE, false);

	nvgpu_log_fn(g, "done");
	return 0;
}

void nvgpu_pmu_surface_describe(struct gk20a *g, struct nvgpu_mem *mem,
		struct flcn_mem_desc_v0 *fb)
{
	fb->address.lo = u64_lo32(mem->gpu_va);
	fb->address.hi = u64_hi32(mem->gpu_va);
	fb->params = ((u32)mem->size & 0xFFFFFF);
	fb->params |= (GK20A_PMU_DMAIDX_VIRT << 24);
}

int nvgpu_pmu_vidmem_surface_alloc(struct gk20a *g, struct nvgpu_mem *mem,
		u32 size)
{
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	int err;

	err = nvgpu_dma_alloc_map_vid(vm, size, mem);
	if (err) {
		nvgpu_err(g, "memory allocation failed");
		return -ENOMEM;
	}

	return 0;
}

int nvgpu_pmu_sysmem_surface_alloc(struct gk20a *g, struct nvgpu_mem *mem,
		u32 size)
{
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	int err;

	err = nvgpu_dma_alloc_map_sys(vm, size, mem);
	if (err) {
		nvgpu_err(g, "failed to allocate memory\n");
		return -ENOMEM;
	}

	return 0;
}

void nvgpu_pmu_surface_free(struct gk20a *g, struct nvgpu_mem *mem)
{
	nvgpu_dma_free(g, mem);
	memset(mem, 0, sizeof(struct nvgpu_mem));
}
