/*
 * Copyright (C) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * Circular buffer implementation
 *
 * This header is BSD licensed so anyone can use the definitions to implement
 * compatible drivers/servers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of NVIDIA CORPORATION nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NVIDIA CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __CBUF_H__
#define __CBUF_H__

#include <asm/barrier.h>
#include <linux/compiler.h>

struct cbuf {
	int end_idx;	/* Array size - 1 */
	int struct_size;
	int w_pos;	/* Always points at empty pos */
	int r_pos;	/* May point at empty/full. is_empty should tell */
	char buf[];	/* Special buffer pointer, shmem is same,
			   but different address across VMs */
};

/* barriers; make them work even on uP */
#ifdef CONFIG_SMP
static inline void cbuf_mb(void)
{
	smp_mb();
}
static inline void cbuf_rmb(void)
{
	smp_rmb();
}
static inline void cbuf_wmb(void)
{
	smp_wmb();
}
#else
static inline void cbuf_mb(void)
{
	mb();
}
static inline void cbuf_rmb(void)
{
	rmb();
}
static inline void cbuf_wmb(void)
{
	wmb();
}
#endif

static inline struct cbuf *cbuf_init(void *buf, int struct_size, int elems)
{
	struct cbuf *cb = buf;	/* Start of buffer */
	cb->end_idx = elems - 1;
	cb->struct_size = struct_size;
	cb->w_pos = 0;
	cb->r_pos = 0;

	return cb;
}

static inline int cbuf_is_empty(struct cbuf *cbuf)
{
	return ACCESS_ONCE(cbuf->w_pos) == ACCESS_ONCE(cbuf->r_pos);
}

static inline int cbuf_is_full(struct cbuf *cbuf)
{
	int w_pos = ACCESS_ONCE(cbuf->w_pos);
	int r_pos = ACCESS_ONCE(cbuf->r_pos);

	/* Full when r=0, w=end_idx */
	if (w_pos - cbuf->end_idx == r_pos)
		return 1;
	/* Full when w=r-1 */
	else if (w_pos == r_pos - 1)
		return 1;
	else
		return 0;
}

static inline void cbuf_advance_w_pos(struct cbuf *cb)
{
	int w_pos = ACCESS_ONCE(cb->w_pos);

	/* Check if need to wrap */
	if (w_pos >= cb->end_idx)
		ACCESS_ONCE(cb->w_pos) = 0;	/* Wrap position */
	else
		ACCESS_ONCE(cb->w_pos) = w_pos + 1; /* Go to next empty pos */
}

static inline void cbuf_advance_r_pos(struct cbuf *cb)
{
	int r_pos = ACCESS_ONCE(cb->r_pos);

	/* Check if need to wrap */
	if (r_pos >= cb->end_idx)
		ACCESS_ONCE(cb->r_pos) = 0;
	else
		ACCESS_ONCE(cb->r_pos) = r_pos + 1;
}

#endif /*__CBUF_H__ */
