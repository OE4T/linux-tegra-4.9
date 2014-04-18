/*
 * Copyright (C) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * Circular buffer with 1 empty slot
 *
 * Traits:
 * - It requires no locking across reader - writer
 *
 * Cons:
 * - Wastes one empty slot
 * - Does copying
 *
 * [0] <-R <-W
 * [1]		Empty
 * [2]
 *
 * [0]
 * [1] <-W	Full
 * [2] <-R
 * [3]
 *
 * [0]
 * [1]	Full
 * [2] <-W
 * [3] <-R
 *
 *
 * [0] <-R
 * [1]		Full
 * [2]
 * [3] <-W
 *
 * This file is BSD licensed so anyone can use the definitions to implement
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

#include <linux/string.h>
#include <linux/errno.h>

#include "cbuf.h"

int cbuf_write(struct cbuf *cb, void *data)
{
	/* Check if full */
	if (cbuf_is_full(cb))
		return -ENOMEM;

	/* Write */
	memcpy(cbuf_write_data_ptr(cb), data, cb->struct_size);

	cbuf_wmb();

	/* Check if need to wrap */
	if (cb->w_pos == cb->end_idx)
		cb->w_pos = 0;	/* Wrap position */
	else
		cb->w_pos++;	/* Go to next empty pos */

	return 0;
}

int cbuf_read(struct cbuf *cb, void *data)
{
	/* Check if empty */
	if (cbuf_is_empty(cb))
		return -ENOMEM;

	/* Read */
	memcpy(data, cbuf_read_data_ptr(cb), cb->struct_size);

	/* Update */
	if (cb->r_pos == cb->end_idx)
		cb->r_pos = 0;
	else
		cb->r_pos++;

	return 0;
}

