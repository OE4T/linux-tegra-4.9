/*
 * gk20a allocator
 *
 * Copyright (c) 2011-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/slab.h>

#include "mm_gk20a.h"
#include "platform_gk20a.h"
#include "gk20a_allocator.h"

u32 gk20a_alloc_tracing_on;

static struct dentry *gk20a_alloc_debugfs_root;

u64 gk20a_alloc_length(struct gk20a_allocator *a)
{
	return a->ops->length(a);
}

u64 gk20a_alloc_base(struct gk20a_allocator *a)
{
	return a->ops->base(a);
}

u64 gk20a_alloc_initialized(struct gk20a_allocator *a)
{
	if (!a->ops)
		return 0;

	return a->ops->inited(a);
}

u64 gk20a_alloc_end(struct gk20a_allocator *a)
{
	return a->ops->end(a);
}

u64 gk20a_alloc(struct gk20a_allocator *a, u64 len)
{
	return a->ops->alloc(a, len);
}

void gk20a_free(struct gk20a_allocator *a, u64 addr)
{
	a->ops->free(a, addr);
}

u64 gk20a_alloc_fixed(struct gk20a_allocator *a, u64 base, u64 len)
{
	return a->ops->alloc_fixed(a, base, len);
}

void gk20a_free_fixed(struct gk20a_allocator *a, u64 base, u64 len)
{
	/*
	 * If this operation is not defined for the allocator then just do
	 * nothing. The alternative would be to fall back on the regular
	 * free but that may be harmful in unexpected ways.
	 */
	if (a->ops->free_fixed)
		a->ops->free_fixed(a, base, len);
}

void gk20a_alloc_destroy(struct gk20a_allocator *a)
{
	a->ops->fini(a);
	memset(a, 0, sizeof(*a));
}

/*
 * Handle the common init stuff for a gk20a_allocator.
 */
int __gk20a_alloc_common_init(struct gk20a_allocator *a,
			      const char *name, void *priv, bool dbg,
			      const struct gk20a_allocator_ops *ops)
{
	if (!ops)
		return -EINVAL;

	a->ops = ops;
	a->priv = priv;
	a->debug = dbg;

	mutex_init(&a->lock);

	strlcpy(a->name, name, sizeof(a->name));

	return 0;
}

void gk20a_alloc_print_stats(struct gk20a_allocator *__a,
			     struct seq_file *s, int lock)
{
	__a->ops->print_stats(__a, s, lock);
}

static int __alloc_show(struct seq_file *s, void *unused)
{
	struct gk20a_allocator *a = s->private;

	gk20a_alloc_print_stats(a, s, 1);

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

void gk20a_init_alloc_debug(struct gk20a_allocator *a)
{
	if (!gk20a_alloc_debugfs_root)
		return;

	a->debugfs_entry = debugfs_create_file(a->name, S_IRUGO,
					       gk20a_alloc_debugfs_root,
					       a, &__alloc_fops);
}

void gk20a_fini_alloc_debug(struct gk20a_allocator *a)
{
	if (!gk20a_alloc_debugfs_root)
		return;

	if (!IS_ERR_OR_NULL(a->debugfs_entry))
		debugfs_remove(a->debugfs_entry);
}

void gk20a_alloc_debugfs_init(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct dentry *gpu_root = platform->debugfs;

	gk20a_alloc_debugfs_root = debugfs_create_dir("allocators", gpu_root);
	if (IS_ERR_OR_NULL(gk20a_alloc_debugfs_root))
		return;

	debugfs_create_u32("tracing", 0664, gk20a_alloc_debugfs_root,
			   &gk20a_alloc_tracing_on);
}
