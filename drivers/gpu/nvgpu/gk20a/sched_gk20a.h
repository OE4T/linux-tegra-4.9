/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __SCHED_GK20A_H
#define __SCHED_GK20A_H

struct gk20a;
struct gpu_ops;
struct tsg_gk20a;
struct poll_table_struct;

struct gk20a_sched_ctrl {
	struct gk20a *g;

	struct nvgpu_mutex control_lock;
	bool control_locked;
	bool sw_ready;
	struct nvgpu_mutex status_lock;
	struct nvgpu_mutex busy_lock;

	u64 status;

	size_t bitmap_size;
	u64 *active_tsg_bitmap;
	u64 *recent_tsg_bitmap;
	u64 *ref_tsg_bitmap;

	wait_queue_head_t readout_wq;
};

int gk20a_sched_dev_release(struct inode *inode, struct file *filp);
int gk20a_sched_dev_open(struct inode *inode, struct file *filp);
long gk20a_sched_dev_ioctl(struct file *, unsigned int, unsigned long);
ssize_t gk20a_sched_dev_read(struct file *, char __user *, size_t, loff_t *);
unsigned int gk20a_sched_dev_poll(struct file *, struct poll_table_struct *);

void gk20a_sched_ctrl_tsg_added(struct gk20a *, struct tsg_gk20a *);
void gk20a_sched_ctrl_tsg_removed(struct gk20a *, struct tsg_gk20a *);
int gk20a_sched_ctrl_init(struct gk20a *);

void gk20a_sched_ctrl_cleanup(struct gk20a *g);

#endif /* __SCHED_GK20A_H */
