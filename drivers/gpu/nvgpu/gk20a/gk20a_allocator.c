/*
 * gk20a allocator
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a_allocator.h"
#include <linux/vmalloc.h>

/* init allocator struct */
int gk20a_allocator_init(struct gk20a_allocator *allocator,
		const char *name, u32 start, u32 len)
{
	memset(allocator, 0, sizeof(struct gk20a_allocator));

	strncpy(allocator->name, name, 32);

	allocator->base = start;
	allocator->limit = start + len - 1;

	allocator->bitmap = vzalloc(BITS_TO_LONGS(len) * sizeof(long));
	if (!allocator->bitmap)
		return -ENOMEM;

	allocator_dbg(allocator, "%s : base %d, limit %d",
		allocator->name, allocator->base, allocator->limit);

	init_rwsem(&allocator->rw_sema);

	allocator->alloc = gk20a_allocator_block_alloc;
	allocator->free = gk20a_allocator_block_free;

	return 0;
}

/* destroy allocator, free all remaining blocks if any */
void gk20a_allocator_destroy(struct gk20a_allocator *allocator)
{
	down_write(&allocator->rw_sema);

	vfree(allocator->bitmap);

	memset(allocator, 0, sizeof(struct gk20a_allocator));
}

/*
 * *addr != ~0 for fixed address allocation. if *addr == 0, base addr is
 * returned to caller in *addr.
 *
 * contiguous allocation, which allocates one block of
 * contiguous address.
*/
int gk20a_allocator_block_alloc(struct gk20a_allocator *allocator,
		u32 *addr, u32 len, u32 align)
{
	unsigned long _addr;

	allocator_dbg(allocator, "[in] addr %d, len %d", *addr, len);

	if ((*addr != 0 && *addr < allocator->base) || /* check addr range */
	    *addr + len > allocator->limit || /* check addr range */
	    *addr & (align - 1) || /* check addr alignment */
	     len == 0)                        /* check len */
		return -EINVAL;

	len = ALIGN(len, align);
	if (!len)
		return -ENOMEM;

	down_write(&allocator->rw_sema);

	_addr = bitmap_find_next_zero_area(allocator->bitmap,
			allocator->limit - allocator->base + 1,
			*addr ? (*addr - allocator->base) : 0,
			len,
			align - 1);
	if ((_addr > allocator->limit - allocator->base + 1) ||
	    (*addr && *addr != (_addr + allocator->base))) {
		up_write(&allocator->rw_sema);
		return -ENOMEM;
	}

	bitmap_set(allocator->bitmap, _addr, len);
	*addr = allocator->base + _addr;

	up_write(&allocator->rw_sema);

	allocator_dbg(allocator, "[out] addr %d, len %d", *addr, len);

	return 0;
}

/* free all blocks between start and end */
int gk20a_allocator_block_free(struct gk20a_allocator *allocator,
		u32 addr, u32 len, u32 align)
{
	allocator_dbg(allocator, "[in] addr %d, len %d", addr, len);

	if (addr + len > allocator->limit || /* check addr range */
	    addr < allocator->base ||
	    addr & (align - 1))   /* check addr alignment */
		return -EINVAL;

	len = ALIGN(len, align);
	if (!len)
		return -EINVAL;

	down_write(&allocator->rw_sema);
	bitmap_clear(allocator->bitmap, addr - allocator->base, len);
	up_write(&allocator->rw_sema);

	allocator_dbg(allocator, "[out] addr %d, len %d", addr, len);

	return 0;
}
