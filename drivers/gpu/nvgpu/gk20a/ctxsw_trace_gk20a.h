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

#ifndef __CTXSW_TRACE_GK20A_H
#define __CTXSW_TRACE_GK20A_H

#define GK20A_CTXSW_TRACE_NUM_DEVS			1

struct file;
struct inode;
struct gk20a;
struct gpu_ops;
struct nvgpu_ctxsw_trace_entry;
struct channel_gk20a;
struct channel_ctx_gk20a;
struct gk20a_ctxsw_dev;
struct gk20a_fecs_trace;
struct tsg_gk20a;
struct poll_table_struct;

int gk20a_ctxsw_dev_release(struct inode *inode, struct file *filp);
int gk20a_ctxsw_dev_open(struct inode *inode, struct file *filp);
long gk20a_ctxsw_dev_ioctl(struct file *filp,
			 unsigned int cmd, unsigned long arg);
ssize_t gk20a_ctxsw_dev_read(struct file *, char __user *, size_t, loff_t *);
unsigned int gk20a_ctxsw_dev_poll(struct file *, struct poll_table_struct *);
int gk20a_ctxsw_dev_mmap(struct file *, struct vm_area_struct *);
int gk20a_ctxsw_dev_ring_alloc(struct gk20a *g, void **buf, size_t *size);
int gk20a_ctxsw_dev_ring_free(struct gk20a *g);
int gk20a_ctxsw_dev_mmap_buffer(struct gk20a *g, struct vm_area_struct *vma);

int gk20a_ctxsw_trace_init(struct gk20a *);
void gk20a_ctxsw_trace_cleanup(struct gk20a *);
int gk20a_ctxsw_trace_write(struct gk20a *, struct nvgpu_ctxsw_trace_entry *);
void gk20a_ctxsw_trace_wake_up(struct gk20a *g, int vmid);

void gk20a_ctxsw_trace_channel_reset(struct gk20a *g, struct channel_gk20a *ch);
void gk20a_ctxsw_trace_tsg_reset(struct gk20a *g, struct tsg_gk20a *tsg);


#endif /* __CTXSW_TRACE_GK20A_H */
