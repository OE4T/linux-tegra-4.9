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

#include <trace/events/gk20a.h>

#include <nvgpu/kmem.h>

#include "gk20a/gk20a.h"

/* dumb allocator... */
static int generate_as_share_id(struct gk20a_as *as)
{
	gk20a_dbg_fn("");
	return ++as->last_share_id;
}
/* still dumb */
static void release_as_share_id(struct gk20a_as *as, int id)
{
	gk20a_dbg_fn("");
	return;
}

int gk20a_as_alloc_share(struct gk20a *g,
			 u32 big_page_size, u32 flags,
			 struct gk20a_as_share **out)
{
	struct gk20a_as_share *as_share;
	int err = 0;

	gk20a_dbg_fn("");
	g = gk20a_get(g);
	if (!g)
		return -ENODEV;

	*out = NULL;
	as_share = nvgpu_kzalloc(g, sizeof(*as_share));
	if (!as_share)
		return -ENOMEM;

	as_share->as = &g->as;
	as_share->id = generate_as_share_id(as_share->as);

	/* this will set as_share->vm. */
	err = gk20a_busy(g);
	if (err)
		goto failed;
	err = g->ops.mm.vm_alloc_share(as_share, big_page_size, flags);
	gk20a_idle(g);

	if (err)
		goto failed;

	*out = as_share;
	return 0;

failed:
	nvgpu_kfree(g, as_share);
	return err;
}

/*
 * channels and the device nodes call this to release.
 * once the ref_cnt hits zero the share is deleted.
 */
int gk20a_as_release_share(struct gk20a_as_share *as_share)
{
	struct gk20a *g = as_share->vm->mm->g;
	int err;

	gk20a_dbg_fn("");

	err = gk20a_busy(g);

	if (err)
		goto release_fail;

	err = gk20a_vm_release_share(as_share);

	gk20a_idle(g);

release_fail:
	release_as_share_id(as_share->as, as_share->id);
	gk20a_put(g);
	nvgpu_kfree(g, as_share);

	return err;
}
