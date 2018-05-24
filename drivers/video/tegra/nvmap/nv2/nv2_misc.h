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

#ifndef __NVMAP2_MISC_H
#define __NVMAP2_MISC_H

void *NVMAP2_altalloc(size_t len);
void NVMAP2_altfree(void *ptr, size_t len);

static inline size_t NVMAP2_ivmid_to_size(u64 ivm_id)
{
	return (ivm_id &
			((1ULL << NVMAP_IVM_LENGTH_WIDTH) - 1)) << PAGE_SHIFT;
}

static inline phys_addr_t NVMAP2_ivmid_to_offset(u64 ivm_id)
{
	return ((ivm_id &
			~((u64)NVMAP_IVM_IVMID_MASK << NVMAP_IVM_IVMID_SHIFT)) >>
			NVMAP_IVM_LENGTH_WIDTH) << (ffs(NVMAP_IVM_ALIGNMENT) - 1);
}

static inline int NVMAP2_ivmid_to_peer(u64 ivm_id)
{
	return (ivm_id >> NVMAP_IVM_IVMID_SHIFT);

}

static inline int NVMAP2_calculate_ivm_id(int vm_id, size_t len,
						unsigned int offs)
{
	int ivm_id = 0;

	BUG_ON(offs & (NVMAP_IVM_ALIGNMENT - 1));
	BUG_ON((offs >> ffs(NVMAP_IVM_ALIGNMENT)) &
			~((1 << NVMAP_IVM_OFFSET_WIDTH) - 1));
	BUG_ON(vm_id & ~(NVMAP_IVM_IVMID_MASK));

	BUG_ON(len & ~(PAGE_MASK));

	ivm_id = ((u64)vm_id << NVMAP_IVM_IVMID_SHIFT);
	ivm_id |= (((offs >> (ffs(NVMAP_IVM_ALIGNMENT) - 1)) &
			((1ULL << NVMAP_IVM_OFFSET_WIDTH) - 1)) <<
				NVMAP_IVM_OFFSET_SHIFT);
	ivm_id |= (len >> PAGE_SHIFT);

	return ivm_id;
}

#endif /* __NVMAP2_MISC_H */
