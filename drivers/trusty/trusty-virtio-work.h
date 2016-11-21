/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef __TRUSTY_VIRTIO_H
#define __TRUSTY_VIRTIO_H

#ifndef CONFIG_PREEMPT_RT_FULL       /* non-RT case */
#include <linux/workqueue.h>

typedef struct work_struct workitem_t;

#define INIT_WORKITEM(t, func) INIT_WORK((t), (func))

static inline void schedule_workitem(workitem_t *t)
{
	schedule_work(t);
}

static inline void cancel_workitem(workitem_t *t)
{
	cancel_work_sync(t);
}

#else				     /* PREEMPT_RT case */
#include <linux/swork.h>

typedef struct swork_event workitem_t;
#define INIT_WORKITEM(t, func) INIT_SWORK((t), (func))

static inline void schedule_workitem(workitem_t *t)
{
	swork_queue(t);
}


static inline void cancel_workitem(workitem_t *t)
{
}
#endif /* defined(CONFIG_PREEMPT_RT_FULL) */

#endif /* __TRUSTY_VITRIO_H */
