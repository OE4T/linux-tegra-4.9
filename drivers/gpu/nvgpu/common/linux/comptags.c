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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/dma-buf.h>

#include <nvgpu/comptags.h>

#include <nvgpu/linux/vm.h>

#include "dmabuf.h"

void gk20a_get_comptags(struct nvgpu_os_buffer *buf,
			struct gk20a_comptags *comptags)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(buf->dmabuf,
							     buf->dev);

	if (!comptags)
		return;

	if (!priv) {
		memset(comptags, 0, sizeof(*comptags));
		return;
	}

	*comptags = priv->comptags;
}

int gk20a_alloc_comptags(struct gk20a *g,
			 struct nvgpu_os_buffer *buf,
			 struct gk20a_comptag_allocator *allocator,
			 u32 lines)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(buf->dmabuf,
							     buf->dev);
	u32 ctaglines_allocsize;
	u32 offset;
	int err;

	if (!priv)
		return -ENOSYS;

	if (!lines)
		return -EINVAL;

	ctaglines_allocsize = lines;

	/* store the allocator so we can use it when we free the ctags */
	priv->comptag_allocator = allocator;
	err = gk20a_comptaglines_alloc(allocator, &offset,
			       ctaglines_allocsize);
	if (err)
		return err;

	priv->comptags.offset = offset;
	priv->comptags.lines = lines;
	priv->comptags.allocated_lines = ctaglines_allocsize;

	return 0;
}
