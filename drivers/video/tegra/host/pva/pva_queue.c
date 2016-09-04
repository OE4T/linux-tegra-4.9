/*
 * PVA Task Management
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

#include <asm/ioctls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include <uapi/linux/nvhost_pva_ioctl.h>

#include "nvhost_queue.h"
#include "pva_queue.h"

static int pva_queue_submit(struct nvhost_queue *queue, void *args)
{
	return 0;
}

static int pva_queue_abort(struct nvhost_queue *queue)
{
	/* TBD: Abort pending tasks from the queue */

	return 0;
}

struct nvhost_queue_ops pva_queue_ops = {
	.abort = pva_queue_abort,
	.submit = pva_queue_submit,
};
