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

#ifndef __NVMAP2_CACHE_H
#define __NVMAP2_CACHE_H

/*
 * TODO: put op at beginning of APIS
 * 	- remove the cache_maint_op
 *
 */

int NVMAP2_cache_maint(struct cache_maint_op *cache_work);
void NVMAP2_cache_maint_inner(unsigned int op, void *vaddr, size_t size);

bool NVMAP2_cache_can_fast_maint(unsigned long start,
			unsigned long end, unsigned int op);
void NVMAP2_cache_fast_maint(unsigned int op);

void NVMAP2_cache_maint_heap_page_outer(struct page **pages,
				unsigned int op,
				unsigned long start, unsigned long end);

void NVMAP2_cache_clean_pages(struct page **pages, int numpages);

int NVMAP2_cache_maint_phys_range(unsigned int op, phys_addr_t pstart,
					phys_addr_t pend);

void NVMAP2_cache_inner_clean_all(void);
void NVMAP2_cache_inner_flush_all(void);

#endif /* __NVMAP2_CACHE_H */
