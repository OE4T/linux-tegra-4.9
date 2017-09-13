/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/module.h>
#include <linux/uaccess.h>

#include <nvgpu/kmem.h>

#include "gk20a/gk20a.h"
#include "boardobj.h"
#include "ctrl/ctrlboardobj.h"

u32 boardobj_construct_super(struct gk20a *g, struct boardobj **ppboardobj,
				u16 size, void *args)
{
	struct boardobj  *pboardobj = NULL;
	struct boardobj  *devtmp = (struct boardobj *)args;

	gk20a_dbg_info(" ");

	if (devtmp == NULL)
		return -EINVAL;

	if (*ppboardobj == NULL) {
		*ppboardobj = nvgpu_kzalloc(g, size);
		if (*ppboardobj == NULL)
			return -ENOMEM;
	}

	pboardobj = *ppboardobj;
	pboardobj->g = g;
	pboardobj->type = devtmp->type;
	pboardobj->idx = CTRL_BOARDOBJ_IDX_INVALID;
	pboardobj->type_mask   = BIT(pboardobj->type) | devtmp->type_mask;

	pboardobj->implements  = boardobj_implements_super;
	pboardobj->destruct    = boardobj_destruct_super;
	pboardobj->pmudatainit = boardobj_pmudatainit_super;

	return 0;
}

u32 boardobj_destruct_super(struct boardobj *pboardobj)
{
	gk20a_dbg_info("");
	if (pboardobj == NULL)
		return -EINVAL;
	nvgpu_kfree(pboardobj->g, pboardobj);
	return 0;
}

bool boardobj_implements_super(struct gk20a *g, struct boardobj *pboardobj,
	u8 type)
{
	gk20a_dbg_info("");

	return (0 != (pboardobj->type_mask & BIT(type)));
}

u32 boardobj_pmudatainit_super(struct gk20a *g, struct boardobj *pboardobj,
				struct nv_pmu_boardobj *pmudata)
{
	gk20a_dbg_info("");
	if (pboardobj == NULL)
		return -EINVAL;
	if (pmudata == NULL)
		return -EINVAL;
	pmudata->type = pboardobj->type;
	gk20a_dbg_info(" Done");
	return 0;
}
