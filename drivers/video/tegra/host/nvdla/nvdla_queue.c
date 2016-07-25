/*
 * NVDLA queue and task management for T194
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

#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#include "dev.h"
#include "bus_client.h"

#include "nvhost_queue.h"
#include "nvdla/nvdla.h"

/* Queue management API */
static int nvdla_queue_abort(struct nvhost_queue *queue)
{
	/* TBD: Abort pending tasks from the queue */

	return 0;
}

struct nvhost_queue_ops nvdla_queue_ops = {
	.abort = nvdla_queue_abort,
};
