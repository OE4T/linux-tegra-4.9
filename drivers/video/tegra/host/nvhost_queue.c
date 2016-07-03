/*
 * NVHOST queue management for T194
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
#include "nvhost_queue.h"

struct nvhost_queue_pool *nvhost_queue_init(struct platform_device *pdev,
					struct nvhost_queue_ops *ops,
					unsigned int num_queues)
{
	struct nvhost_queue_pool *pool;
	struct nvhost_queue *queues;
	struct nvhost_queue *queue;
	unsigned int i;
	int err;

	pool = kzalloc(sizeof(struct nvhost_queue_pool), GFP_KERNEL);
	if (pool == NULL) {
		err = -ENOMEM;
		goto fail_alloc_pool;
	}

	queues = kcalloc(num_queues, sizeof(struct nvhost_queue), GFP_KERNEL);
	if (queues == NULL) {
		err = -ENOMEM;
		goto fail_alloc_queues;
	}
	/* initialize pool and queues */
	pool->pdev = pdev;
	pool->ops = ops;
	pool->queues = queues;
	pool->alloc_table = 0;
	pool->max_queue_cnt = num_queues;
	mutex_init(&pool->queue_lock);

	for (i = 0; i < num_queues; i++) {
		queue = &queues[i];
		queue->id = i;
		queue->pool = pool;
	}

	return pool;

fail_alloc_queues:
	kfree(pool);
fail_alloc_pool:
	return ERR_PTR(err);
}

void nvhost_queue_deinit(struct nvhost_queue_pool *pool)
{
	if (!pool)
		return;

	kfree(pool->queues);
	kfree(pool);
	pool = NULL;
}

static void nvhost_queue_release(struct kref *ref)
{
	struct nvhost_queue *queue = container_of(ref, struct nvhost_queue,
						kref);
	struct nvhost_queue_pool *pool = queue->pool;

	nvhost_dbg_fn("");

	/* release allocated resources */
	nvhost_syncpt_put_ref_ext(pool->pdev, queue->syncpt_id);

	/* ..and mark the queue free */
	mutex_lock(&pool->queue_lock);
	clear_bit(queue->id, &pool->alloc_table);
	mutex_unlock(&pool->queue_lock);
}

void nvhost_queue_put(struct nvhost_queue *queue)
{
	nvhost_dbg_fn("");
	kref_put(&queue->kref, nvhost_queue_release);
}

void nvhost_queue_get(struct nvhost_queue *queue)
{
	nvhost_dbg_fn("");
	kref_get(&queue->kref);
}

struct nvhost_queue *nvhost_queue_alloc(struct nvhost_queue_pool *pool)
{
	struct platform_device *pdev = pool->pdev;
	struct nvhost_queue *queues = pool->queues;
	struct nvhost_queue *queue;
	int index = 0;
	int err = 0;

	mutex_lock(&pool->queue_lock);

	index = find_first_zero_bit(&pool->alloc_table,
				    pool->max_queue_cnt);

	/* quit if we found a queue */
	if (index >= pool->max_queue_cnt) {
		dev_err(&pdev->dev, "failed to get free Queue\n");
		err = -ENOMEM;
		goto err_alloc_queue;
	}

	/* reserve the queue */
	queue = &queues[index];
	set_bit(index, &pool->alloc_table);

	/* allocate a syncpt for the queue */
	queue->syncpt_id = nvhost_get_syncpt_host_managed(pdev, index, NULL);
	if (!queue->syncpt_id) {
		dev_err(&pdev->dev, "failed to get syncpt id\n");
		err = -ENOMEM;
		goto err_alloc_syncpt;
	}

	/* initialize queue ref count */
	kref_init(&queue->kref);

	mutex_unlock(&pool->queue_lock);

	return queue;

err_alloc_syncpt:
	clear_bit(queue->id, &pool->alloc_table);
err_alloc_queue:
	mutex_unlock(&pool->queue_lock);

	return ERR_PTR(err);
}

int nvhost_queue_abort(struct nvhost_queue *queue)
{
	struct nvhost_queue_pool *pool = queue->pool;

	if (pool->ops && pool->ops->abort)
		return pool->ops->abort(queue);

	return 0;
}

