/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

/*
 * The following structure is used for reference counting of objects in nvgpu.
 */
#ifndef __NVGPU_KREF_H__
#define __NVGPU_KREF_H__

#include <nvgpu/atomic.h>

struct nvgpu_ref {
	nvgpu_atomic_t refcount;
};

/*
 * Initialize object.
 * @ref: the nvgpu_ref object to initialize
 */
static inline void nvgpu_ref_init(struct nvgpu_ref *ref)
{
	nvgpu_atomic_set(&ref->refcount, 1);
}

/*
 * Increment reference count for the object
 * @ref: the nvgpu_ref object
 */
static inline void nvgpu_ref_get(struct nvgpu_ref *ref)
{
	nvgpu_atomic_inc(&ref->refcount);
}

/*
 * Decrement reference count for the object and call release() if it becomes
 * zero.
 * @ref: the nvgpu_ref object
 * @release: pointer to the function that would be invoked to clean up the
 *	object when the reference count becomes zero, i.e. the last
 *	reference corresponding to this object is removed.
 * Return 1 if object was removed, otherwise return 0. The user should not
 * make any assumptions about the status of the object in the memory when
 * the function returns 0 and should only use it to know that there are no
 * further references to this object.
 */
static inline int nvgpu_ref_put(struct nvgpu_ref *ref,
		void (*release)(struct nvgpu_ref *r))
{
	if (nvgpu_atomic_sub_and_test(1, &ref->refcount)) {
		if (release != NULL)
			release(ref);
		return 1;
	}
	return 0;
}

/*
 * Increment reference count for the object unless it is zero.
 * @ref: the nvgpu_ref object
 * Return non-zero if the increment succeeds, Otherwise return 0.
 */
static inline int __must_check nvgpu_ref_get_unless_zero(struct nvgpu_ref *ref)
{
	return nvgpu_atomic_add_unless(&ref->refcount, 1, 0);
}

#endif /* __NVGPU_KREF_H__ */
