/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kthread.h>

#include <nvgpu/thread.h>

int nvgpu_thread_create(struct nvgpu_thread *thread,
		void *data,
		int (*threadfn)(void *data), const char *name)
{
	struct task_struct *task = kthread_create(threadfn, data, name);
	if (IS_ERR(task))
		return PTR_ERR(task);

	thread->task = task;
	wake_up_process(task);
	return 0;
};

void nvgpu_thread_stop(struct nvgpu_thread *thread)
{
	kthread_stop(thread->task);
};

bool nvgpu_thread_should_stop(struct nvgpu_thread *thread)
{
	return kthread_should_stop();
};
