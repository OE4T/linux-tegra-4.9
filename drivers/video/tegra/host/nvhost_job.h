/*
 * drivers/video/tegra/host/nvhost_job.h
 *
 * Tegra Graphics Host Interrupt Management
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

#ifndef __NVHOST_JOB_H
#define __NVHOST_JOB_H

#include <linux/nvhost_ioctl.h>

struct nvhost_channel;
struct nvhost_hwctx;
struct nvmap_client;
struct nvhost_waitchk;
struct nvmap_handle;

/*
 * Each submit is tracked as a nvhost_job.
 */
struct nvhost_job {
	/* When refcount goes to zero, job can be freed */
	struct kref ref;

	/* List entry */
	struct list_head list;

	/* Channel where job is submitted to */
	struct nvhost_channel *ch;

	/* Hardware context valid for this client */
	struct nvhost_hwctx *hwctx;
	int clientid;

	/* Nvmap to be used for pinning & unpinning memory */
	struct nvmap_client *nvmap;

	/* Gathers and their memory */
	struct nvmap_handle_ref *gather_mem;
	struct nvhost_channel_gather *gathers;
	int num_gathers;
	int gather_mem_size;

	/* Wait checks to be processed at submit time */
	struct nvhost_waitchk *waitchk;
	int num_waitchk;
	u32 waitchk_mask;

	/* Array of handles to be pinned & unpinned */
	struct nvmap_pinarray_elem *pinarray;
	int num_pins;
	struct nvmap_handle **unpins;
	int num_unpins;

	/* Sync point id, number of increments and end related to the submit */
	u32 syncpt_id;
	u32 syncpt_incrs;
	u32 syncpt_end;

	/* Priority of this submit. */
	int priority;

	/* Maximum time to wait for this job */
	int timeout;

	/* Null kickoff prevents submit from being sent to hardware */
	bool null_kickoff;

	/* Index and number of slots used in the push buffer */
	int first_get;
	int num_slots;

	/* Context to be freed */
	struct nvhost_hwctx *hwctxref;
};

/*
 * Allocate memory for a job. Just enough memory will be allocated to
 * accomodate the submit announced in submit header.
 */
struct nvhost_job *nvhost_job_alloc(struct nvhost_channel *ch,
		struct nvhost_hwctx *hwctx,
		struct nvhost_submit_hdr_ext *hdr,
		struct nvmap_client *nvmap,
		int priority, int clientid);

/*
 * Allocate memory for a job. Just enough memory will be allocated to
 * accomodate the submit announced in submit header. Gather memory from
 * oldjob will be reused, and nvhost_job_put() will be called to it.
 */
struct nvhost_job *nvhost_job_realloc(struct nvhost_job *oldjob,
		struct nvhost_hwctx *hwctx,
		struct nvhost_submit_hdr_ext *hdr,
		struct nvmap_client *nvmap,
		int priority, int clientid);

/*
 * Add a gather to a job.
 */
void nvhost_job_add_gather(struct nvhost_job *job,
		u32 mem_id, u32 words, u32 offset);

/*
 * Increment reference going to nvhost_job.
 */
void nvhost_job_get(struct nvhost_job *job);

/*
 * Increment reference for a hardware context.
 */
void nvhost_job_get_hwctx(struct nvhost_job *job, struct nvhost_hwctx *hwctx);

/*
 * Decrement reference job, free if goes to zero.
 */
void nvhost_job_put(struct nvhost_job *job);

/*
 * Pin memory related to job. This handles relocation of addresses to the
 * host1x address space. Handles both the gather memory and any other memory
 * referred to from the gather buffers.
 */
int nvhost_job_pin(struct nvhost_job *job);

/*
 * Unpin memory related to job.
 */
void nvhost_job_unpin(struct nvhost_job *job);

/*
 * Dump contents of job to debug output.
 */
void nvhost_job_dump(struct device *dev, struct nvhost_job *job);

#endif
