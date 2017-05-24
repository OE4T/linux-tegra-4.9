/*
 * Copyright (C) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "debug_allocator.h"
#include "gk20a/platform_gk20a.h"

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <nvgpu/allocator.h>

u32 nvgpu_alloc_tracing_on;

void nvgpu_alloc_print_stats(struct nvgpu_allocator *__a,
			     struct seq_file *s, int lock)
{
	__a->ops->print_stats(__a, s, lock);
}

static int __alloc_show(struct seq_file *s, void *unused)
{
	struct nvgpu_allocator *a = s->private;

	nvgpu_alloc_print_stats(a, s, 1);

	return 0;
}

static int __alloc_open(struct inode *inode, struct file *file)
{
	return single_open(file, __alloc_show, inode->i_private);
}

static const struct file_operations __alloc_fops = {
	.open = __alloc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void nvgpu_init_alloc_debug(struct gk20a *g, struct nvgpu_allocator *a)
{
	if (!g->debugfs_allocators)
		return;

	a->debugfs_entry = debugfs_create_file(a->name, S_IRUGO,
					       g->debugfs_allocators,
					       a, &__alloc_fops);
}

void nvgpu_fini_alloc_debug(struct nvgpu_allocator *a)
{
	if (!IS_ERR_OR_NULL(a->debugfs_entry))
		debugfs_remove(a->debugfs_entry);
}

void nvgpu_alloc_debugfs_init(struct gk20a *g)
{
	struct gk20a_platform *platform = dev_get_drvdata(g->dev);

	g->debugfs_allocators = debugfs_create_dir("allocators", platform->debugfs);
	if (IS_ERR_OR_NULL(g->debugfs_allocators)) {
		g->debugfs_allocators = NULL;
		return;
	}

	debugfs_create_u32("tracing", 0664, g->debugfs_allocators,
			   &nvgpu_alloc_tracing_on);
}
