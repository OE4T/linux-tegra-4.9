/*
 * GK20A Address Spaces
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef __NVGPU_AS_H__
#define __NVGPU_AS_H__

struct vm_gk20a;

struct gk20a_as {
	int last_share_id; /* dummy allocator for now */
};

struct gk20a_as_share {
	struct gk20a_as *as;
	struct vm_gk20a *vm;
	int id;
};

int gk20a_as_release_share(struct gk20a_as_share *as_share);

/* if big_page_size == 0, the default big page size is used */
int gk20a_as_alloc_share(struct gk20a *g, u32 big_page_size,
			 u32 flags, struct gk20a_as_share **out);

#endif
