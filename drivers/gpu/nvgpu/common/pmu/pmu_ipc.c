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

#include <nvgpu/enabled.h>
#include <nvgpu/pmu.h>
#include <nvgpu/log.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>

#include "gk20a/gk20a.h"

void nvgpu_pmu_seq_init(struct nvgpu_pmu *pmu)
{
	u32 i;

	memset(pmu->seq, 0,
		sizeof(struct pmu_sequence) * PMU_MAX_NUM_SEQUENCES);
	memset(pmu->pmu_seq_tbl, 0,
		sizeof(pmu->pmu_seq_tbl));

	for (i = 0; i < PMU_MAX_NUM_SEQUENCES; i++)
		pmu->seq[i].id = i;
}

static int pmu_seq_acquire(struct nvgpu_pmu *pmu,
			struct pmu_sequence **pseq)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_sequence *seq;
	u32 index;

	nvgpu_mutex_acquire(&pmu->pmu_seq_lock);
	index = find_first_zero_bit(pmu->pmu_seq_tbl,
				sizeof(pmu->pmu_seq_tbl));
	if (index >= sizeof(pmu->pmu_seq_tbl)) {
		nvgpu_err(g, "no free sequence available");
		nvgpu_mutex_release(&pmu->pmu_seq_lock);
		return -EAGAIN;
	}
	set_bit(index, pmu->pmu_seq_tbl);
	nvgpu_mutex_release(&pmu->pmu_seq_lock);

	seq = &pmu->seq[index];
	seq->state = PMU_SEQ_STATE_PENDING;

	*pseq = seq;
	return 0;
}

static void pmu_seq_release(struct nvgpu_pmu *pmu,
			struct pmu_sequence *seq)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	seq->state	= PMU_SEQ_STATE_FREE;
	seq->desc	= PMU_INVALID_SEQ_DESC;
	seq->callback	= NULL;
	seq->cb_params	= NULL;
	seq->msg	= NULL;
	seq->out_payload = NULL;
	g->ops.pmu_ver.pmu_allocation_set_dmem_size(pmu,
		g->ops.pmu_ver.get_pmu_seq_in_a_ptr(seq), 0);
	g->ops.pmu_ver.pmu_allocation_set_dmem_size(pmu,
		g->ops.pmu_ver.get_pmu_seq_out_a_ptr(seq), 0);

	clear_bit(seq->id, pmu->pmu_seq_tbl);
}
/* mutex */
int nvgpu_pmu_mutex_acquire(struct nvgpu_pmu *pmu, u32 id, u32 *token)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	return g->ops.pmu.pmu_mutex_acquire(pmu, id, token);
}

int nvgpu_pmu_mutex_release(struct nvgpu_pmu *pmu, u32 id, u32 *token)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	return g->ops.pmu.pmu_mutex_release(pmu, id, token);
}

/* queue */
int nvgpu_pmu_queue_init(struct nvgpu_pmu *pmu,
		u32 id, union pmu_init_msg_pmu *init)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_queue *queue = &pmu->queue[id];
	int err;

	err = nvgpu_mutex_init(&queue->mutex);
	if (err)
		return err;

	queue->id	= id;
	g->ops.pmu_ver.get_pmu_init_msg_pmu_queue_params(queue, id, init);
	queue->mutex_id = id;

	nvgpu_pmu_dbg(g, "queue %d: index %d, offset 0x%08x, size 0x%08x",
		id, queue->index, queue->offset, queue->size);

	return 0;
}

static int pmu_queue_head(struct nvgpu_pmu *pmu, struct pmu_queue *queue,
			u32 *head, bool set)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	return g->ops.pmu.pmu_queue_head(pmu, queue, head, set);
}

static int pmu_queue_tail(struct nvgpu_pmu *pmu, struct pmu_queue *queue,
			u32 *tail, bool set)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	return g->ops.pmu.pmu_queue_tail(pmu, queue, tail, set);
}

static inline void pmu_queue_read(struct nvgpu_pmu *pmu,
			u32 offset, u8 *dst, u32 size)
{
	nvgpu_flcn_copy_from_dmem(pmu->flcn, offset, dst, size, 0);
}

static inline void pmu_queue_write(struct nvgpu_pmu *pmu,
			u32 offset, u8 *src, u32 size)
{
	nvgpu_flcn_copy_to_dmem(pmu->flcn, offset, src, size, 0);
}


static int pmu_queue_lock(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue)
{
	int err;

	if (PMU_IS_MESSAGE_QUEUE(queue->id))
		return 0;

	if (PMU_IS_SW_COMMAND_QUEUE(queue->id)) {
		nvgpu_mutex_acquire(&queue->mutex);
		return 0;
	}

	err = nvgpu_pmu_mutex_acquire(pmu, queue->mutex_id, &queue->mutex_lock);
	return err;
}

static int pmu_queue_unlock(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue)
{
	int err;

	if (PMU_IS_MESSAGE_QUEUE(queue->id))
		return 0;

	if (PMU_IS_SW_COMMAND_QUEUE(queue->id)) {
		nvgpu_mutex_release(&queue->mutex);
		return 0;
	}

	err = nvgpu_pmu_mutex_release(pmu, queue->mutex_id, &queue->mutex_lock);
	return err;
}

/* called by pmu_read_message, no lock */
bool nvgpu_pmu_queue_is_empty(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue)
{
	u32 head, tail;

	pmu_queue_head(pmu, queue, &head, QUEUE_GET);
	if (queue->opened && queue->oflag == OFLAG_READ)
		tail = queue->position;
	else
		pmu_queue_tail(pmu, queue, &tail, QUEUE_GET);

	return head == tail;
}

static bool pmu_queue_has_room(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue, u32 size, bool *need_rewind)
{
	u32 head, tail;
	bool rewind = false;
	unsigned int free;

	size = ALIGN(size, QUEUE_ALIGNMENT);

	pmu_queue_head(pmu, queue, &head, QUEUE_GET);
	pmu_queue_tail(pmu, queue, &tail, QUEUE_GET);
	if (head >= tail) {
		free = queue->offset + queue->size - head;
		free -= PMU_CMD_HDR_SIZE;

		if (size > free) {
			rewind = true;
			head = queue->offset;
		}
	}

	if (head < tail)
		free = tail - head - 1;

	if (need_rewind)
		*need_rewind = rewind;

	return size <= free;
}

static int pmu_queue_push(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue, void *data, u32 size)
{
	struct gk20a *g = pmu->g;

	nvgpu_log_fn(g, " ");

	if (!queue->opened && queue->oflag == OFLAG_WRITE) {
		nvgpu_err(gk20a_from_pmu(pmu), "queue not opened for write");
		return -EINVAL;
	}

	pmu_queue_write(pmu, queue->position, data, size);
	queue->position += ALIGN(size, QUEUE_ALIGNMENT);
	return 0;
}

static int pmu_queue_pop(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue, void *data, u32 size,
			u32 *bytes_read)
{
	u32 head, tail, used;

	*bytes_read = 0;

	if (!queue->opened && queue->oflag == OFLAG_READ) {
		nvgpu_err(gk20a_from_pmu(pmu), "queue not opened for read");
		return -EINVAL;
	}

	pmu_queue_head(pmu, queue, &head, QUEUE_GET);
	tail = queue->position;

	if (head == tail)
		return 0;

	if (head > tail)
		used = head - tail;
	else
		used = queue->offset + queue->size - tail;

	if (size > used) {
		nvgpu_warn(gk20a_from_pmu(pmu),
			"queue size smaller than request read");
		size = used;
	}

	pmu_queue_read(pmu, tail, data, size);
	queue->position += ALIGN(size, QUEUE_ALIGNMENT);
	*bytes_read = size;
	return 0;
}

static void pmu_queue_rewind(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_cmd cmd;

	nvgpu_log_fn(g, " ");

	if (!queue->opened) {
		nvgpu_err(gk20a_from_pmu(pmu), "queue not opened");
		return;
	}

	if (queue->oflag == OFLAG_WRITE) {
		cmd.hdr.unit_id = PMU_UNIT_REWIND;
		cmd.hdr.size = PMU_CMD_HDR_SIZE;
		pmu_queue_push(pmu, queue, &cmd, cmd.hdr.size);
		nvgpu_pmu_dbg(g, "queue %d rewinded", queue->id);
	}

	queue->position = queue->offset;
}

/* open for read and lock the queue */
static int pmu_queue_open_read(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue)
{
	int err;

	err = pmu_queue_lock(pmu, queue);
	if (err)
		return err;

	if (queue->opened)
		BUG();

	pmu_queue_tail(pmu, queue, &queue->position, QUEUE_GET);
	queue->oflag = OFLAG_READ;
	queue->opened = true;

	return 0;
}

/* open for write and lock the queue
 * make sure there's enough free space for the write
 * */
static int pmu_queue_open_write(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue, u32 size)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	bool rewind = false;
	int err;

	err = pmu_queue_lock(pmu, queue);
	if (err)
		return err;

	if (queue->opened)
		BUG();

	if (!pmu_queue_has_room(pmu, queue, size, &rewind)) {
		nvgpu_pmu_dbg(g, "queue full: queue-id %d: index %d",
				queue->id, queue->index);
		pmu_queue_unlock(pmu, queue);
		return -EAGAIN;
	}

	pmu_queue_head(pmu, queue, &queue->position, QUEUE_GET);
	queue->oflag = OFLAG_WRITE;
	queue->opened = true;

	if (rewind)
		pmu_queue_rewind(pmu, queue);

	return 0;
}

/* close and unlock the queue */
static int pmu_queue_close(struct nvgpu_pmu *pmu,
			struct pmu_queue *queue, bool commit)
{
	if (!queue->opened)
		return 0;

	if (commit) {
		if (queue->oflag == OFLAG_READ)
			pmu_queue_tail(pmu, queue,
				&queue->position, QUEUE_SET);
		else
			pmu_queue_head(pmu, queue,
				&queue->position, QUEUE_SET);
	}

	queue->opened = false;

	pmu_queue_unlock(pmu, queue);

	return 0;
}

static bool pmu_validate_cmd(struct nvgpu_pmu *pmu, struct pmu_cmd *cmd,
			struct pmu_msg *msg, struct pmu_payload *payload,
			u32 queue_id)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_queue *queue;
	u32 in_size, out_size;

	if (!PMU_IS_SW_COMMAND_QUEUE(queue_id))
		goto invalid_cmd;

	queue = &pmu->queue[queue_id];
	if (cmd->hdr.size < PMU_CMD_HDR_SIZE)
		goto invalid_cmd;

	if (cmd->hdr.size > (queue->size >> 1))
		goto invalid_cmd;

	if (msg != NULL && msg->hdr.size < PMU_MSG_HDR_SIZE)
		goto invalid_cmd;

	if (!PMU_UNIT_ID_IS_VALID(cmd->hdr.unit_id))
		goto invalid_cmd;

	if (payload == NULL)
		return true;

	if (payload->in.buf == NULL && payload->out.buf == NULL)
		goto invalid_cmd;

	if ((payload->in.buf != NULL && payload->in.size == 0) ||
	    (payload->out.buf != NULL && payload->out.size == 0))
		goto invalid_cmd;

	in_size = PMU_CMD_HDR_SIZE;
	if (payload->in.buf) {
		in_size += payload->in.offset;
		in_size += g->ops.pmu_ver.get_pmu_allocation_struct_size(pmu);
	}

	out_size = PMU_CMD_HDR_SIZE;
	if (payload->out.buf) {
		out_size += payload->out.offset;
		out_size += g->ops.pmu_ver.get_pmu_allocation_struct_size(pmu);
	}

	if (in_size > cmd->hdr.size || out_size > cmd->hdr.size)
		goto invalid_cmd;


	if ((payload->in.offset != 0 && payload->in.buf == NULL) ||
	    (payload->out.offset != 0 && payload->out.buf == NULL))
		goto invalid_cmd;

	return true;

invalid_cmd:
	nvgpu_err(g, "invalid pmu cmd :\n"
		"queue_id=%d,\n"
		"cmd_size=%d, cmd_unit_id=%d, msg=%p, msg_size=%d,\n"
		"payload in=%p, in_size=%d, in_offset=%d,\n"
		"payload out=%p, out_size=%d, out_offset=%d",
		queue_id, cmd->hdr.size, cmd->hdr.unit_id,
		msg, msg ? msg->hdr.unit_id : ~0,
		&payload->in, payload->in.size, payload->in.offset,
		&payload->out, payload->out.size, payload->out.offset);

	return false;
}

static int pmu_write_cmd(struct nvgpu_pmu *pmu, struct pmu_cmd *cmd,
			u32 queue_id, unsigned long timeout_ms)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_queue *queue;
	struct nvgpu_timeout timeout;
	int err;

	nvgpu_log_fn(g, " ");

	queue = &pmu->queue[queue_id];
	nvgpu_timeout_init(g, &timeout, timeout_ms, NVGPU_TIMER_CPU_TIMER);

	do {
		err = pmu_queue_open_write(pmu, queue, cmd->hdr.size);
		if (err == -EAGAIN && !nvgpu_timeout_expired(&timeout))
			nvgpu_usleep_range(1000, 2000);
		else
			break;
	} while (1);

	if (err)
		goto clean_up;

	pmu_queue_push(pmu, queue, cmd, cmd->hdr.size);


	err = pmu_queue_close(pmu, queue, true);

clean_up:
	if (err)
		nvgpu_err(g, "fail to write cmd to queue %d", queue_id);
	else
		nvgpu_log_fn(g, "done");

	return err;
}

int nvgpu_pmu_cmd_post(struct gk20a *g, struct pmu_cmd *cmd,
		struct pmu_msg *msg, struct pmu_payload *payload,
		u32 queue_id, pmu_callback callback, void *cb_param,
		u32 *seq_desc, unsigned long timeout)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_v *pv = &g->ops.pmu_ver;
	struct pmu_sequence *seq;
	void *in = NULL, *out = NULL;
	int err;

	nvgpu_log_fn(g, " ");

	if ((!cmd) || (!seq_desc) || (!pmu->pmu_ready)) {
		if (!cmd)
			nvgpu_warn(g, "%s(): PMU cmd buffer is NULL", __func__);
		else if (!seq_desc)
			nvgpu_warn(g, "%s(): Seq descriptor is NULL", __func__);
		else
			nvgpu_warn(g, "%s(): PMU is not ready", __func__);

		WARN_ON(1);
		return -EINVAL;
	}

	if (!pmu_validate_cmd(pmu, cmd, msg, payload, queue_id))
		return -EINVAL;

	err = pmu_seq_acquire(pmu, &seq);
	if (err)
		return err;

	cmd->hdr.seq_id = seq->id;

	cmd->hdr.ctrl_flags = 0;
	cmd->hdr.ctrl_flags |= PMU_CMD_FLAGS_STATUS;
	cmd->hdr.ctrl_flags |= PMU_CMD_FLAGS_INTR;

	seq->callback = callback;
	seq->cb_params = cb_param;
	seq->msg = msg;
	seq->out_payload = NULL;
	seq->desc = pmu->next_seq_desc++;

	if (payload)
		seq->out_payload = payload->out.buf;

	*seq_desc = seq->desc;

	if (payload && payload->in.offset != 0) {
		pv->set_pmu_allocation_ptr(pmu, &in,
		((u8 *)&cmd->cmd + payload->in.offset));

		if (payload->in.buf != payload->out.buf)
			pv->pmu_allocation_set_dmem_size(pmu, in,
			(u16)payload->in.size);
		else
			pv->pmu_allocation_set_dmem_size(pmu, in,
			(u16)max(payload->in.size, payload->out.size));

		*(pv->pmu_allocation_get_dmem_offset_addr(pmu, in)) =
			nvgpu_alloc(&pmu->dmem,
				     pv->pmu_allocation_get_dmem_size(pmu, in));
		if (!*(pv->pmu_allocation_get_dmem_offset_addr(pmu, in)))
			goto clean_up;

		if (payload->in.fb_size != 0x0) {
			seq->in_mem = nvgpu_kzalloc(g,
					sizeof(struct nvgpu_mem));
			if (!seq->in_mem) {
				err = -ENOMEM;
				goto clean_up;
			}

			nvgpu_pmu_vidmem_surface_alloc(g, seq->in_mem,
				payload->in.fb_size);
			nvgpu_pmu_surface_describe(g, seq->in_mem,
				(struct flcn_mem_desc_v0 *)
				pv->pmu_allocation_get_fb_addr(pmu, in));

			nvgpu_mem_wr_n(g, seq->in_mem, 0,
				payload->in.buf, payload->in.fb_size);

		} else {
			nvgpu_flcn_copy_to_dmem(pmu->flcn,
				(pv->pmu_allocation_get_dmem_offset(pmu, in)),
				payload->in.buf, payload->in.size, 0);
		}
		pv->pmu_allocation_set_dmem_size(pmu,
		pv->get_pmu_seq_in_a_ptr(seq),
		pv->pmu_allocation_get_dmem_size(pmu, in));
		pv->pmu_allocation_set_dmem_offset(pmu,
		pv->get_pmu_seq_in_a_ptr(seq),
		pv->pmu_allocation_get_dmem_offset(pmu, in));
	}

	if (payload && payload->out.offset != 0) {
		pv->set_pmu_allocation_ptr(pmu, &out,
		((u8 *)&cmd->cmd + payload->out.offset));
		pv->pmu_allocation_set_dmem_size(pmu, out,
		(u16)payload->out.size);

		if (payload->in.buf != payload->out.buf) {
			*(pv->pmu_allocation_get_dmem_offset_addr(pmu, out)) =
				nvgpu_alloc(&pmu->dmem,
				    pv->pmu_allocation_get_dmem_size(pmu, out));
			if (!*(pv->pmu_allocation_get_dmem_offset_addr(pmu,
					out)))
				goto clean_up;

			if (payload->out.fb_size != 0x0) {
				seq->out_mem = nvgpu_kzalloc(g,
					sizeof(struct nvgpu_mem));
				if (!seq->out_mem) {
					err = -ENOMEM;
					goto clean_up;
				}
				nvgpu_pmu_vidmem_surface_alloc(g, seq->out_mem,
					payload->out.fb_size);
				nvgpu_pmu_surface_describe(g, seq->out_mem,
					(struct flcn_mem_desc_v0 *)
					pv->pmu_allocation_get_fb_addr(pmu,
					out));
			}
		} else {
			BUG_ON(in == NULL);
			seq->out_mem = seq->in_mem;
			pv->pmu_allocation_set_dmem_offset(pmu, out,
			pv->pmu_allocation_get_dmem_offset(pmu, in));
		}
		pv->pmu_allocation_set_dmem_size(pmu,
		pv->get_pmu_seq_out_a_ptr(seq),
		pv->pmu_allocation_get_dmem_size(pmu, out));
		pv->pmu_allocation_set_dmem_offset(pmu,
		pv->get_pmu_seq_out_a_ptr(seq),
		pv->pmu_allocation_get_dmem_offset(pmu, out));

	}



	seq->state = PMU_SEQ_STATE_USED;

	err = pmu_write_cmd(pmu, cmd, queue_id, timeout);
	if (err)
		seq->state = PMU_SEQ_STATE_PENDING;

	nvgpu_log_fn(g, "done");

	return 0;

clean_up:
	nvgpu_log_fn(g, "fail");
	if (in)
		nvgpu_free(&pmu->dmem,
			pv->pmu_allocation_get_dmem_offset(pmu, in));
	if (out)
		nvgpu_free(&pmu->dmem,
			pv->pmu_allocation_get_dmem_offset(pmu, out));

	pmu_seq_release(pmu, seq);
	return err;
}

static int pmu_response_handle(struct nvgpu_pmu *pmu,
			struct pmu_msg *msg)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_sequence *seq;
	struct pmu_v *pv = &g->ops.pmu_ver;
	int ret = 0;

	nvgpu_log_fn(g, " ");

	seq = &pmu->seq[msg->hdr.seq_id];
	if (seq->state != PMU_SEQ_STATE_USED &&
	    seq->state != PMU_SEQ_STATE_CANCELLED) {
		nvgpu_err(g, "msg for an unknown sequence %d", seq->id);
		return -EINVAL;
	}

	if (msg->hdr.unit_id == PMU_UNIT_RC &&
	    msg->msg.rc.msg_type == PMU_RC_MSG_TYPE_UNHANDLED_CMD) {
		nvgpu_err(g, "unhandled cmd: seq %d", seq->id);
	} else if (seq->state != PMU_SEQ_STATE_CANCELLED) {
		if (seq->msg) {
			if (seq->msg->hdr.size >= msg->hdr.size) {
				memcpy(seq->msg, msg, msg->hdr.size);
			}  else {
				nvgpu_err(g, "sequence %d msg buffer too small",
					seq->id);
			}
		}
		if (pv->pmu_allocation_get_dmem_size(pmu,
		pv->get_pmu_seq_out_a_ptr(seq)) != 0) {
			nvgpu_flcn_copy_from_dmem(pmu->flcn,
			pv->pmu_allocation_get_dmem_offset(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)),
			seq->out_payload,
			pv->pmu_allocation_get_dmem_size(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)), 0);
		}
	} else
		seq->callback = NULL;
	if (pv->pmu_allocation_get_dmem_size(pmu,
			pv->get_pmu_seq_in_a_ptr(seq)) != 0)
		nvgpu_free(&pmu->dmem,
			pv->pmu_allocation_get_dmem_offset(pmu,
			pv->get_pmu_seq_in_a_ptr(seq)));
	if (pv->pmu_allocation_get_dmem_size(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)) != 0)
		nvgpu_free(&pmu->dmem,
			pv->pmu_allocation_get_dmem_offset(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)));

	if (seq->out_mem != NULL) {
		memset(pv->pmu_allocation_get_fb_addr(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)), 0x0,
			pv->pmu_allocation_get_fb_size(pmu,
				pv->get_pmu_seq_out_a_ptr(seq)));

		nvgpu_pmu_surface_free(g, seq->out_mem);
		if (seq->out_mem != seq->in_mem)
			nvgpu_kfree(g, seq->out_mem);
		else
			seq->out_mem = NULL;
	}

	if (seq->in_mem != NULL) {
		memset(pv->pmu_allocation_get_fb_addr(pmu,
			pv->get_pmu_seq_in_a_ptr(seq)), 0x0,
			pv->pmu_allocation_get_fb_size(pmu,
				pv->get_pmu_seq_in_a_ptr(seq)));

		nvgpu_pmu_surface_free(g, seq->in_mem);
		nvgpu_kfree(g, seq->in_mem);
		seq->in_mem = NULL;
	}

	if (seq->callback)
		seq->callback(g, msg, seq->cb_params, seq->desc, ret);

	pmu_seq_release(pmu, seq);

	/* TBD: notify client waiting for available dmem */

	nvgpu_log_fn(g, "done");

	return 0;
}

static int pmu_handle_event(struct nvgpu_pmu *pmu, struct pmu_msg *msg)
{
	int err = 0;
	struct gk20a *g = gk20a_from_pmu(pmu);

	nvgpu_log_fn(g, " ");
	switch (msg->hdr.unit_id) {
	case PMU_UNIT_PERFMON:
	case PMU_UNIT_PERFMON_T18X:
		err = nvgpu_pmu_handle_perfmon_event(pmu, &msg->msg.perfmon);
		break;
	case PMU_UNIT_PERF:
		if (g->ops.perf.handle_pmu_perf_event != NULL) {
			err = g->ops.perf.handle_pmu_perf_event(g,
				(void *)&msg->msg.perf);
		} else {
			WARN_ON(1);
		}
		break;
	case PMU_UNIT_THERM:
		err = nvgpu_pmu_handle_therm_event(pmu, &msg->msg.therm);
		break;
	default:
		break;
	}

	return err;
}

static bool pmu_read_message(struct nvgpu_pmu *pmu, struct pmu_queue *queue,
			struct pmu_msg *msg, int *status)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	u32 read_size, bytes_read;
	int err;

	*status = 0;

	if (nvgpu_pmu_queue_is_empty(pmu, queue))
		return false;

	err = pmu_queue_open_read(pmu, queue);
	if (err) {
		nvgpu_err(g, "fail to open queue %d for read", queue->id);
		*status = err;
		return false;
	}

	err = pmu_queue_pop(pmu, queue, &msg->hdr,
			PMU_MSG_HDR_SIZE, &bytes_read);
	if (err || bytes_read != PMU_MSG_HDR_SIZE) {
		nvgpu_err(g, "fail to read msg from queue %d", queue->id);
		*status = err | -EINVAL;
		goto clean_up;
	}

	if (msg->hdr.unit_id == PMU_UNIT_REWIND) {
		pmu_queue_rewind(pmu, queue);
		/* read again after rewind */
		err = pmu_queue_pop(pmu, queue, &msg->hdr,
				PMU_MSG_HDR_SIZE, &bytes_read);
		if (err || bytes_read != PMU_MSG_HDR_SIZE) {
			nvgpu_err(g,
				"fail to read msg from queue %d", queue->id);
			*status = err | -EINVAL;
			goto clean_up;
		}
	}

	if (!PMU_UNIT_ID_IS_VALID(msg->hdr.unit_id)) {
		nvgpu_err(g, "read invalid unit_id %d from queue %d",
			msg->hdr.unit_id, queue->id);
			*status = -EINVAL;
			goto clean_up;
	}

	if (msg->hdr.size > PMU_MSG_HDR_SIZE) {
		read_size = msg->hdr.size - PMU_MSG_HDR_SIZE;
		err = pmu_queue_pop(pmu, queue, &msg->msg,
			read_size, &bytes_read);
		if (err || bytes_read != read_size) {
			nvgpu_err(g,
				"fail to read msg from queue %d", queue->id);
			*status = err;
			goto clean_up;
		}
	}

	err = pmu_queue_close(pmu, queue, true);
	if (err) {
		nvgpu_err(g, "fail to close queue %d", queue->id);
		*status = err;
		return false;
	}

	return true;

clean_up:
	err = pmu_queue_close(pmu, queue, false);
	if (err)
		nvgpu_err(g, "fail to close queue %d", queue->id);
	return false;
}

int nvgpu_pmu_process_message(struct nvgpu_pmu *pmu)
{
	struct pmu_msg msg;
	int status;
	struct gk20a *g = gk20a_from_pmu(pmu);

	if (unlikely(!pmu->pmu_ready)) {
		nvgpu_pmu_process_init_msg(pmu, &msg);
		if (g->ops.pmu.init_wpr_region != NULL)
			g->ops.pmu.init_wpr_region(g);
		if (nvgpu_is_enabled(g, NVGPU_PMU_PERFMON))
			nvgpu_pmu_init_perfmon(pmu);

		return 0;
	}

	while (pmu_read_message(pmu,
		&pmu->queue[PMU_MESSAGE_QUEUE], &msg, &status)) {

		nvgpu_pmu_dbg(g, "read msg hdr: ");
		nvgpu_pmu_dbg(g, "unit_id = 0x%08x, size = 0x%08x",
			msg.hdr.unit_id, msg.hdr.size);
		nvgpu_pmu_dbg(g, "ctrl_flags = 0x%08x, seq_id = 0x%08x",
			msg.hdr.ctrl_flags, msg.hdr.seq_id);

		msg.hdr.ctrl_flags &= ~PMU_CMD_FLAGS_PMU_MASK;

		if (msg.hdr.ctrl_flags == PMU_CMD_FLAGS_EVENT)
			pmu_handle_event(pmu, &msg);
		else
			pmu_response_handle(pmu, &msg);
	}

	return 0;
}

int pmu_wait_message_cond(struct nvgpu_pmu *pmu, u32 timeout_ms,
				 u32 *var, u32 val)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;

	nvgpu_timeout_init(g, &timeout, timeout_ms, NVGPU_TIMER_CPU_TIMER);

	do {
		if (*var == val)
			return 0;

		if (gk20a_pmu_is_interrupted(pmu))
			gk20a_pmu_isr(g);

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	return -ETIMEDOUT;
}

