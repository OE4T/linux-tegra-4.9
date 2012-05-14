/*
 * drivers/video/tegra/host/nvhost_job.c
 *
 * Tegra Graphics Host Job
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/err.h>
#include <linux/vmalloc.h>
#include <mach/nvmap.h>
#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "nvhost_hwctx.h"
#include "dev.h"

/* Magic to use to fill freed handle slots */
#define BAD_MAGIC 0xdeadbeef

static int job_size(struct nvhost_submit_hdr_ext *hdr)
{
	int num_pins = hdr ? (hdr->num_relocs + hdr->num_cmdbufs)*2 : 0;
	int num_waitchks = hdr ? hdr->num_waitchks : 0;

	return sizeof(struct nvhost_job)
			+ num_pins * sizeof(struct nvmap_pinarray_elem)
			+ num_pins * sizeof(struct nvmap_handle *)
			+ num_waitchks * sizeof(struct nvhost_waitchk);
}

static int gather_size(int num_cmdbufs)
{
	return num_cmdbufs * sizeof(struct nvhost_channel_gather);
}

static void free_gathers(struct nvhost_job *job)
{
	if (job->gathers) {
		nvmap_munmap(job->gather_mem, job->gathers);
		job->gathers = NULL;
	}
	if (job->gather_mem) {
		nvmap_free(job->nvmap, job->gather_mem);
		job->gather_mem = NULL;
	}
}

static int alloc_gathers(struct nvhost_job *job,
		int num_cmdbufs)
{
	int err = 0;

	job->gather_mem = NULL;
	job->gathers = NULL;
	job->gather_mem_size = 0;

	if (num_cmdbufs) {
		/* Allocate memory */
		job->gather_mem = nvmap_alloc(job->nvmap,
				gather_size(num_cmdbufs),
				32, NVMAP_HANDLE_CACHEABLE, 0);
		if (IS_ERR_OR_NULL(job->gather_mem)) {
			err = job->gather_mem ? PTR_ERR(job->gather_mem) : -ENOMEM;
			job->gather_mem = NULL;
			goto error;
		}
		job->gather_mem_size = gather_size(num_cmdbufs);

		/* Map memory to kernel */
		job->gathers = nvmap_mmap(job->gather_mem);
		if (IS_ERR_OR_NULL(job->gathers)) {
			err = job->gathers ? PTR_ERR(job->gathers) : -ENOMEM;
			job->gathers = NULL;
			goto error;
		}
	}

	return 0;

error:
	free_gathers(job);
	return err;
}

static int realloc_gathers(struct nvhost_job *oldjob,
		struct nvhost_job *newjob,
		int num_cmdbufs)
{
	int err = 0;

	/* Check if we can reuse gather buffer */
	if (oldjob->gather_mem_size < gather_size(num_cmdbufs)
			|| oldjob->nvmap != newjob->nvmap) {
		free_gathers(oldjob);
		err = alloc_gathers(newjob, num_cmdbufs);
	} else {
		newjob->gather_mem = oldjob->gather_mem;
		newjob->gathers = oldjob->gathers;
		newjob->gather_mem_size = oldjob->gather_mem_size;

		oldjob->gather_mem = NULL;
		oldjob->gathers = NULL;
		oldjob->gather_mem_size = 0;
	}
	return err;
}

static void init_fields(struct nvhost_job *job,
		struct nvhost_submit_hdr_ext *hdr,
		int priority, int clientid)
{
	int num_pins = hdr ? (hdr->num_relocs + hdr->num_cmdbufs)*2 : 0;
	int num_waitchks = hdr ? hdr->num_waitchks : 0;
	void *mem = job;

	/* First init state to zero */
	job->num_gathers = 0;
	job->num_pins = 0;
	job->num_unpins = 0;
	job->num_waitchk = 0;
	job->waitchk_mask = 0;
	job->syncpt_id = 0;
	job->syncpt_incrs = 0;
	job->syncpt_end = 0;
	job->priority = priority;
	job->clientid = clientid;
	job->null_kickoff = false;
	job->first_get = 0;
	job->num_slots = 0;

	/* Redistribute memory to the structs */
	mem += sizeof(struct nvhost_job);
	if (num_pins) {
		job->pinarray = mem;
		mem += num_pins * sizeof(struct nvmap_pinarray_elem);
		job->unpins = mem;
		mem += num_pins * sizeof(struct nvmap_handle *);
	} else {
		job->pinarray = NULL;
		job->unpins = NULL;
	}

	job->waitchk = num_waitchks ? mem : NULL;

	/* Copy information from header */
	if (hdr) {
		job->waitchk_mask = hdr->waitchk_mask;
		job->syncpt_id = hdr->syncpt_id;
		job->syncpt_incrs = hdr->syncpt_incrs;
	}
}

struct nvhost_job *nvhost_job_alloc(struct nvhost_channel *ch,
		struct nvhost_hwctx *hwctx,
		struct nvhost_submit_hdr_ext *hdr,
		struct nvmap_client *nvmap,
		int priority,
		int clientid)
{
	struct nvhost_job *job = NULL;
	int num_cmdbufs = hdr ? hdr->num_cmdbufs : 0;
	int err = 0;

	job = vzalloc(job_size(hdr));
	if (!job)
		goto error;

	kref_init(&job->ref);
	job->ch = ch;
	job->hwctx = hwctx;
	if (hwctx)
		hwctx->h->get(hwctx);
	job->nvmap = nvmap ? nvmap_client_get(nvmap) : NULL;

	err = alloc_gathers(job, num_cmdbufs);
	if (err)
		goto error;

	init_fields(job, hdr, priority, clientid);

	return job;

error:
	if (job)
		nvhost_job_put(job);
	return NULL;
}

struct nvhost_job *nvhost_job_realloc(
		struct nvhost_job *oldjob,
		struct nvhost_hwctx *hwctx,
		struct nvhost_submit_hdr_ext *hdr,
		struct nvmap_client *nvmap,
		int priority, int clientid)
{
	struct nvhost_job *newjob = NULL;
	int num_cmdbufs = hdr ? hdr->num_cmdbufs : 0;
	int err = 0;

	newjob = vzalloc(job_size(hdr));
	if (!newjob)
		goto error;
	kref_init(&newjob->ref);
	newjob->ch = oldjob->ch;
	newjob->hwctx = hwctx;
	if (hwctx)
		newjob->hwctx->h->get(newjob->hwctx);
	newjob->timeout = oldjob->timeout;
	newjob->nvmap = nvmap ? nvmap_client_get(nvmap) : NULL;

	err = realloc_gathers(oldjob, newjob, num_cmdbufs);
	if (err)
		goto error;

	nvhost_job_put(oldjob);

	init_fields(newjob, hdr, priority, clientid);

	return newjob;

error:
	if (newjob)
		nvhost_job_put(newjob);
	if (oldjob)
		nvhost_job_put(oldjob);
	return NULL;
}

void nvhost_job_get(struct nvhost_job *job)
{
	kref_get(&job->ref);
}

static void job_free(struct kref *ref)
{
	struct nvhost_job *job = container_of(ref, struct nvhost_job, ref);

	if (job->hwctxref)
		job->hwctxref->h->put(job->hwctxref);
	if (job->hwctx)
		job->hwctx->h->put(job->hwctx);
	if (job->gathers)
		nvmap_munmap(job->gather_mem, job->gathers);
	if (job->gather_mem)
		nvmap_free(job->nvmap, job->gather_mem);
	if (job->nvmap)
		nvmap_client_put(job->nvmap);
	vfree(job);
}

/* Acquire reference to a hardware context. Used for keeping saved contexts in
 * memory. */
void nvhost_job_get_hwctx(struct nvhost_job *job, struct nvhost_hwctx *hwctx)
{
	BUG_ON(job->hwctxref);

	job->hwctxref = hwctx;
	hwctx->h->get(hwctx);
}

void nvhost_job_put(struct nvhost_job *job)
{
	kref_put(&job->ref, job_free);
}

void nvhost_job_add_gather(struct nvhost_job *job,
		u32 mem_id, u32 words, u32 offset)
{
	struct nvmap_pinarray_elem *pin;
	struct nvhost_channel_gather *cur_gather =
			&job->gathers[job->num_gathers];

	pin = &job->pinarray[job->num_pins++];
	pin->patch_mem = (u32)nvmap_ref_to_handle(job->gather_mem);
	pin->patch_offset = (void *)&(cur_gather->mem) - (void *)job->gathers;
	pin->pin_mem = nvmap_convert_handle_u2k(mem_id);
	pin->pin_offset = offset;
	cur_gather->words = words;
	cur_gather->mem_id = mem_id;
	cur_gather->offset = offset;
	job->num_gathers += 1;
}

int nvhost_job_pin(struct nvhost_job *job)
{
	int err = 0;

	/* pin mem handles and patch physical addresses */
	job->num_unpins = nvmap_pin_array(job->nvmap,
				nvmap_ref_to_handle(job->gather_mem),
				job->pinarray, job->num_pins,
				job->unpins);
	if (job->num_unpins < 0)
		err = job->num_unpins;

	return err;
}

void nvhost_job_unpin(struct nvhost_job *job)
{
	nvmap_unpin_handles(job->nvmap, job->unpins,
			job->num_unpins);
	memset(job->unpins, BAD_MAGIC,
			job->num_unpins * sizeof(struct nvmap_handle *));
}

/**
 * Debug routine used to dump job entries
 */
void nvhost_job_dump(struct device *dev, struct nvhost_job *job)
{
	dev_dbg(dev, "    SYNCPT_ID   %d\n",
		job->syncpt_id);
	dev_dbg(dev, "    SYNCPT_VAL  %d\n",
		job->syncpt_end);
	dev_dbg(dev, "    FIRST_GET   0x%x\n",
		job->first_get);
	dev_dbg(dev, "    TIMEOUT     %d\n",
		job->timeout);
	dev_dbg(dev, "    CTX 0x%p\n",
		job->hwctx);
	dev_dbg(dev, "    NUM_SLOTS   %d\n",
		job->num_slots);
	dev_dbg(dev, "    NUM_HANDLES %d\n",
		job->num_unpins);
}
