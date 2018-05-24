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

#ifndef __NVMAP2_HEAP_ALLOC_H
#define __NVMAP2_HEAP_ALLOC_H

struct device *NVMAP2_heap_type_to_dev(unsigned long type);
unsigned int NVMAP2_heap_type_conversion(unsigned int orig_heap);

int NVMAP2_heap_type_is_carveout(unsigned int heap_type);
int NVMAP2_heap_type_is_iovmm(unsigned int heap_type);
int NVMAP2_heap_type_is_dma(unsigned long type);

const unsigned int *NVMAP2_heap_mask_to_policy(unsigned int heap_mask, int nr_page);

struct page **NVMAP2_heap_alloc_iovmm_pages(size_t size, bool contiguous);

struct page **NVMAP2_heap_alloc_from_va(size_t size, ulong vaddr);

struct page **NVMAP2_heap_alloc_dma_pages(size_t size, unsigned long type);
void NVMAP2_heap_dealloc_dma_pages(size_t size, unsigned long type,
				struct page **pages);


#endif /* __NVMAP2_HEAP_ALLOC_H */
