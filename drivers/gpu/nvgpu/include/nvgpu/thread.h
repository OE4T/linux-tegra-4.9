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

#ifndef __NVGPU_THREAD_H__
#define __NVGPU_THREAD_H__

#ifdef __KERNEL__
#include <nvgpu/linux/thread.h>
#else
#include <nvgpu_rmos/include/thread.h>
#endif

/**
 * nvgpu_thread_create - Create and run a new thread.
 *
 * @thread - thread structure to use
 * @data - data to pass to threadfn
 * @threadfn - Thread function
 * @name - name of the thread
 *
 * Create a thread and run threadfn in it. The thread stays alive as long as
 * threadfn is running. As soon as threadfn returns the thread is destroyed.
 *
 * threadfn needs to continuously poll nvgpu_thread_should_stop() to determine
 * if it should exit.
 */
int nvgpu_thread_create(struct nvgpu_thread *thread,
		void *data,
		int (*threadfn)(void *data), const char *name);

/**
 * nvgpu_thread_stop - Destroy or request to destroy a thread
 *
 * @thread - thread to stop
 *
 * Request a thread to stop by setting nvgpu_thread_should_stop() to
 * true and wait for thread to exit.
 */
void nvgpu_thread_stop(struct nvgpu_thread *thread);

/**
 * nvgpu_thread_should_stop - Query if thread should stop
 *
 * @thread
 *
 * Return true if thread should exit. Can be run only in the thread's own
 * context and with the thread as parameter.
 */
bool nvgpu_thread_should_stop(struct nvgpu_thread *thread);

/**
 * nvgpu_thread_is_running - Query if thread is running
 *
 * @thread
 *
 * Return true if thread is started.
 */
bool nvgpu_thread_is_running(struct nvgpu_thread *thread);

#endif /* __NVGPU_THREAD_H__ */
