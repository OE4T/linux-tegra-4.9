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
#include <linux/nvmap.h>
#include <trace/events/nvhost.h>
#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "nvhost_hwctx.h"
#include "dev.h"

/* Magic to use to fill freed handle slots */
#define BAD_MAGIC 0xdeadbeef

static int job_size(struct nvhost_submit_hdr_ext *hdr)
{
	int num_relocs = hdr ? hdr->num_relocs : 0;
	int num_waitchks = hdr ? hdr->num_waitchks : 0;
	int num_cmdbufs = hdr ? hdr->num_cmdbufs : 0;
	int num_unpins = num_cmdbufs + num_relocs;

	return sizeof(struct nvhost_job)
			+ num_relocs * sizeof(struct nvmap_pinarray_elem)
			+ num_unpins * sizeof(struct nvmap_handle_ref *)
			+ num_waitchks * sizeof(struct nvhost_waitchk)
			+ num_cmdbufs * sizeof(struct nvhost_job_gather);
}

static void init_fields(struct nvhost_job *job,
		struct nvhost_submit_hdr_ext *hdr,
		int priority, int clientid)
{
	int num_relocs = hdr ? hdr->num_relocs : 0;
	int num_waitchks = hdr ? hdr->num_waitchks : 0;
	int num_cmdbufs = hdr ? hdr->num_cmdbufs : 0;
	int num_unpins = num_cmdbufs + num_relocs;
	void *mem = job;

	/* First init state to zero */
	job->priority = priority;
	job->clientid = clientid;

	/* Redistribute memory to the structs */
	mem += sizeof(struct nvhost_job);
	job->pinarray = num_relocs ? mem : NULL;
	mem += num_relocs * sizeof(struct nvmap_pinarray_elem);
	job->unpins = num_unpins ? mem : NULL;
	mem += num_unpins * sizeof(struct nvmap_handle_ref *);
	job->waitchk = num_waitchks ? mem : NULL;
	mem += num_waitchks * sizeof(struct nvhost_waitchk);
	job->gathers = num_cmdbufs ? mem : NULL;

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

	job = vzalloc(job_size(hdr));
	if (!job)
		goto error;

	kref_init(&job->ref);
	job->ch = ch;
	job->hwctx = hwctx;
	if (hwctx)
		hwctx->h->get(hwctx);
	job->nvmap = nvmap ? nvmap_client_get(nvmap) : NULL;

	init_fields(job, hdr, priority, clientid);

	return job;

error:
	if (job)
		nvhost_job_put(job);
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
	struct nvhost_job_gather *cur_gather =
			&job->gathers[job->num_gathers];

	cur_gather->words = words;
	cur_gather->mem_id = mem_id;
	cur_gather->offset = offset;
	job->num_gathers += 1;
}

static int do_relocs(struct nvhost_job *job, u32 patch_mem, void *patch_addr)
{
	phys_addr_t pin_phys;
	int i;
	u32 mem_id = 0;
	struct nvmap_handle_ref *pin_ref = NULL;

	/* pin & patch the relocs for one gather */
	for (i = 0; i < job->num_relocs; i++) {
		struct nvmap_pinarray_elem *pin = &job->pinarray[i];

		/* skip all other gathers */
		if (patch_mem != pin->patch_mem)
			continue;

		/* check if pin-mem is same as previous */
		if (pin->pin_mem != mem_id) {
			pin_ref = nvmap_duplicate_handle_id(job->nvmap,
					pin->pin_mem);
			if (IS_ERR(pin_ref))
				return PTR_ERR(pin_ref);

			pin_phys = nvmap_pin(job->nvmap, pin_ref);
			if (IS_ERR((void *)pin_phys)) {
				nvmap_free(job->nvmap, pin_ref);
				return pin_phys;
			}

			mem_id = pin->pin_mem;
			job->unpins[job->num_unpins++] = pin_ref;
		}

		__raw_writel((pin_phys + pin->pin_offset) >> pin->reloc_shift,
				(patch_addr + pin->patch_offset));

		/* Different gathers might have same mem_id. This ensures we
		 * perform reloc only once per gather memid. */
		pin->patch_mem = 0;
	}

	return 0;
}

int nvhost_job_pin(struct nvhost_job *job)
{
	int err = 0, i = 0;
	phys_addr_t gather_phys = 0;
	void *gather_addr = NULL;

	/* pin gathers */
	for (i = 0; i < job->num_gathers; i++) {
		struct nvhost_job_gather *g = &job->gathers[i];

		/* process each gather mem only once */
		if (!g->ref) {
			g->ref = nvmap_duplicate_handle_id(job->nvmap,
					job->gathers[i].mem_id);
			if (IS_ERR(g->ref)) {
				err = PTR_ERR(g->ref);
				g->ref = NULL;
				break;
			}

			gather_phys = nvmap_pin(job->nvmap, g->ref);
			if (IS_ERR((void *)gather_phys)) {
				nvmap_free(job->nvmap, g->ref);
				err = gather_phys;
				break;
			}

			/* store the gather ref into unpin array */
			job->unpins[job->num_unpins++] = g->ref;

			gather_addr = nvmap_mmap(g->ref);
			if (!gather_addr) {
				err = -ENOMEM;
				break;
			}

			err = do_relocs(job, g->mem_id, gather_addr);
			nvmap_munmap(g->ref, gather_addr);

			if (err)
				break;
		}
		g->mem = gather_phys + g->offset;
	}
	wmb();

	return err;
}

void nvhost_job_unpin(struct nvhost_job *job)
{
	int i;

	for (i = 0; i < job->num_unpins; i++) {
		nvmap_unpin(job->nvmap, job->unpins[i]);
		nvmap_free(job->nvmap, job->unpins[i]);
	}

	memset(job->unpins, BAD_MAGIC,
			job->num_unpins * sizeof(struct nvmap_handle_ref *));
	job->num_unpins = 0;
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
