/*
 * PVA queue management for T194
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include "dev.h"
#include "pva_queue.h"
#include "pva.h"

void pva_queue_init(struct pva *pva)
{
	unsigned int i;

	for (i = 0; i < MAX_PVA_QUEUE_COUNT; i++)
		pva->queues[i].id = i;
}

static void pva_queue_release(struct kref *ref)
{
	struct pva_queue *queue = container_of(ref, struct pva_queue, kref);
	struct pva *pva = queue->pva;

	nvhost_dbg_fn("");

	/* release allocated resources */
	nvhost_syncpt_put_ref_ext(pva->pdev, queue->syncpt_id);

	/* ..and mark the queue free */
	mutex_lock(&pva->allocated_queues_mutex);
	clear_bit(queue->id, &pva->allocated_queues);
	mutex_unlock(&pva->allocated_queues_mutex);
}

void pva_queue_put(struct pva_queue *queue)
{
	nvhost_dbg_fn("");
	kref_put(&queue->kref, pva_queue_release);
}

void pva_queue_get(struct pva_queue *queue)
{
	nvhost_dbg_fn("");
	kref_get(&queue->kref);
}

struct pva_queue *pva_queue_alloc(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva *pva = pdata->private_data;
	struct pva_queue *queue;
	int index = 0;
	int err = 0;

	mutex_lock(&pva->allocated_queues_mutex);

	index = find_first_zero_bit(&pva->allocated_queues,
				MAX_PVA_QUEUE_COUNT);

	/* quit if we found a queue */
	if (index >= MAX_PVA_QUEUE_COUNT) {
		dev_err(&pdev->dev, "failed to get free Queue\n");
		err = -ENOMEM;
		goto err_alloc_queue;
	}

	/* reserve the queue */
	queue = pva->queues + index;
	set_bit(index, &pva->allocated_queues);

	/* allocate a syncpt for the queue */
	queue->syncpt_id = nvhost_get_syncpt_host_managed(pdev, index, NULL);
	if (!queue->syncpt_id) {
		dev_err(&pdev->dev, "failed to get syncpt id\n");
		err = -ENOMEM;
		goto err_alloc_syncpt;
	}

	/* initialize queue ref count */
	kref_init(&queue->kref);
	queue->pva = pva;

	mutex_unlock(&pva->allocated_queues_mutex);

	return queue;

err_alloc_syncpt:
	clear_bit(queue->id, &pva->allocated_queues);
err_alloc_queue:
	mutex_unlock(&pva->allocated_queues_mutex);

	return ERR_PTR(err);
}

int pva_queue_abort(struct pva_queue *queue)
{
	/* TBD: Abort pending tasks from the queue */
	return 0;
}

