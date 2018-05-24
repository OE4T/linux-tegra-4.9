/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __NVMAP2_CARVEOUT_H
#define __NVMAP2_CARVEOUT_H

int NVMAP2_carveout_create(const struct nvmap_platform_carveout *co);

struct nvmap_heap_block *NVMAP2_carveout_alloc(struct nvmap_heap *h,
					struct nvmap_handle *handle,
					phys_addr_t *start,
					size_t len,
					size_t align,
					unsigned int prot,
					int peer);

#endif /* __NVMAP2_CARVEOUT_H */
