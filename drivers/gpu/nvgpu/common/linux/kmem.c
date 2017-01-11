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

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/rbtree.h>
#include <linux/debugfs.h>
#include <linux/spinlock.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/stacktrace.h>

#include <nvgpu/kmem.h>

#include "gk20a/gk20a.h"

#include "kmem_priv.h"

/*
 * Statically declared because this needs to be shared across all nvgpu driver
 * instances. This makes sure that all kmem caches are _definitely_ uniquely
 * named.
 */
static atomic_t kmem_cache_id;

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE

static void lock_tracker(struct nvgpu_mem_alloc_tracker *tracker)
{
	mutex_lock(&tracker->lock);
}

static void unlock_tracker(struct nvgpu_mem_alloc_tracker *tracker)
{
	mutex_unlock(&tracker->lock);
}

static void kmem_print_mem_alloc(struct gk20a *g,
				 struct nvgpu_mem_alloc *alloc,
				 struct seq_file *s)
{
#ifdef __NVGPU_SAVE_KALLOC_STACK_TRACES
	int i;

	__pstat(s, "nvgpu-alloc: addr=0x%llx size=%ld\n",
		alloc->addr, alloc->size);
	for (i = 0; i < alloc->stack_length; i++)
		__pstat(s, "  %3d [<%p>] %pS\n", i,
			(void *)alloc->stack[i],
			(void *)alloc->stack[i]);
	__pstat(s, "\n");
#else
	__pstat(s, "nvgpu-alloc: addr=0x%llx size=%ld src=%pF\n",
		alloc->addr, alloc->size, alloc->ip);
#endif
}

static int nvgpu_add_alloc(struct nvgpu_mem_alloc_tracker *tracker,
			   struct nvgpu_mem_alloc *alloc)
{
	struct rb_node **new = &tracker->allocs.rb_node;
	struct rb_node *parent = NULL;

	while (*new) {
		struct nvgpu_mem_alloc *tmp = rb_entry(*new,
						       struct nvgpu_mem_alloc,
						       allocs_entry);

		parent = *new;

		if (alloc->addr < tmp->addr)
			new = &(*new)->rb_left;
		else if (alloc->addr > tmp->addr)
			new = &(*new)->rb_right;
		else
			return -EINVAL;
	}

	/* Put the new node there */
	rb_link_node(&alloc->allocs_entry, parent, new);
	rb_insert_color(&alloc->allocs_entry, &tracker->allocs);

	return 0;
}

static struct nvgpu_mem_alloc *nvgpu_rem_alloc(
	struct nvgpu_mem_alloc_tracker *tracker, u64 alloc_addr)
{
	struct rb_node *node = tracker->allocs.rb_node;
	struct nvgpu_mem_alloc *alloc;

	while (node) {
		alloc = container_of(node,
				     struct nvgpu_mem_alloc, allocs_entry);

		if (alloc_addr < alloc->addr)
			node = node->rb_left;
		else if (alloc_addr > alloc->addr)
			node = node->rb_right;
		else
			break;
	}

	if (!node)
		return NULL;

	rb_erase(node, &tracker->allocs);

	return alloc;
}

static int __nvgpu_save_kmem_alloc(struct nvgpu_mem_alloc_tracker *tracker,
				   unsigned long size, unsigned long real_size,
				   u64 addr, unsigned long ip)
{
	int ret;
	struct nvgpu_mem_alloc *alloc;
#ifdef __NVGPU_SAVE_KALLOC_STACK_TRACES
	struct stack_trace stack_trace;
#endif

	alloc = kzalloc(sizeof(*alloc), GFP_KERNEL);
	if (!alloc)
		return -ENOMEM;

	alloc->owner = tracker;
	alloc->size = size;
	alloc->real_size = real_size;
	alloc->addr = addr;
	alloc->ip = (void *)(uintptr_t)ip;

#ifdef __NVGPU_SAVE_KALLOC_STACK_TRACES
	stack_trace.max_entries = MAX_STACK_TRACE;
	stack_trace.nr_entries = 0;
	stack_trace.entries = alloc->stack;
	/*
	 * This 4 here skips the 2 function calls that happen for all traced
	 * allocs due to nvgpu:
	 *
	 *   __nvgpu_save_kmem_alloc+0x7c/0x128
	 *   __nvgpu_track_kzalloc+0xcc/0xf8
	 *
	 * And the function calls that get made by the stack trace code itself.
	 * If the trace savings code changes this will likely have to change
	 * as well.
	 */
	stack_trace.skip = 4;
	save_stack_trace(&stack_trace);
	alloc->stack_length = stack_trace.nr_entries;
#endif

	lock_tracker(tracker);
	tracker->bytes_alloced += size;
	tracker->bytes_alloced_real += real_size;
	tracker->nr_allocs++;

	/* Keep track of this for building a histogram later on. */
	if (tracker->max_alloc < size)
		tracker->max_alloc = size;
	if (tracker->min_alloc > size)
		tracker->min_alloc = size;

	ret = nvgpu_add_alloc(tracker, alloc);
	if (ret) {
		WARN(1, "Duplicate alloc??? 0x%llx\n", addr);
		kfree(alloc);
		unlock_tracker(tracker);
		return ret;
	}
	unlock_tracker(tracker);

	return 0;
}

static int __nvgpu_free_kmem_alloc(struct nvgpu_mem_alloc_tracker *tracker,
				   u64 addr)
{
	struct nvgpu_mem_alloc *alloc;

	lock_tracker(tracker);
	alloc = nvgpu_rem_alloc(tracker, addr);
	if (WARN(!alloc, "Possible double-free detected: 0x%llx!", addr)) {
		unlock_tracker(tracker);
		return -EINVAL;
	}

	tracker->nr_frees++;
	tracker->bytes_freed += alloc->size;
	tracker->bytes_freed_real += alloc->real_size;
	unlock_tracker(tracker);

	return 0;
}

static void __nvgpu_check_valloc_size(unsigned long size)
{
	WARN(size < PAGE_SIZE, "Alloc smaller than page size! (%lu)!\n", size);
}

static void __nvgpu_check_kalloc_size(size_t size)
{
	WARN(size > PAGE_SIZE, "Alloc larger than page size! (%zu)!\n", size);
}

void *__nvgpu_track_vmalloc(struct gk20a *g, unsigned long size,
			    unsigned long ip)
{
	void *alloc = vmalloc(size);

	if (!alloc)
		return NULL;

	kmem_dbg("vmalloc: size=%-6ld addr=0x%p", size, alloc);
	__nvgpu_check_valloc_size(size);

	/*
	 * Ignore the return message. If this fails let's not cause any issues
	 * for the rest of the driver.
	 */
	__nvgpu_save_kmem_alloc(g->vmallocs, size, roundup_pow_of_two(size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void *__nvgpu_track_vzalloc(struct gk20a *g, unsigned long size,
			    unsigned long ip)
{
	void *alloc = vzalloc(size);

	if (!alloc)
		return NULL;

	kmem_dbg("vzalloc: size=%-6ld addr=0x%p", size, alloc);
	__nvgpu_check_valloc_size(size);

	/*
	 * Ignore the return message. If this fails let's not cause any issues
	 * for the rest of the driver.
	 */
	__nvgpu_save_kmem_alloc(g->vmallocs, size, roundup_pow_of_two(size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void *__nvgpu_track_kmalloc(struct gk20a *g, size_t size, unsigned long ip)
{
	void *alloc = kmalloc(size, GFP_KERNEL);

	if (!alloc)
		return NULL;

	kmem_dbg("kmalloc: size=%-6ld addr=0x%p gfp=0x%08x",
		 size, alloc, GFP_KERNEL);
	__nvgpu_check_kalloc_size(size);

	__nvgpu_save_kmem_alloc(g->kmallocs, size, roundup_pow_of_two(size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void *__nvgpu_track_kzalloc(struct gk20a *g, size_t size, unsigned long ip)
{
	void *alloc = kzalloc(size, GFP_KERNEL);

	if (!alloc)
		return NULL;

	kmem_dbg("kzalloc: size=%-6ld addr=0x%p gfp=0x%08x",
		 size, alloc, GFP_KERNEL);
	__nvgpu_check_kalloc_size(size);

	__nvgpu_save_kmem_alloc(g->kmallocs, size, roundup_pow_of_two(size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void *__nvgpu_track_kcalloc(struct gk20a *g, size_t n, size_t size,
			    unsigned long ip)
{
	void *alloc = kcalloc(n, size, GFP_KERNEL);

	if (!alloc)
		return NULL;

	kmem_dbg("kcalloc: size=%-6ld addr=0x%p gfp=0x%08x",
		 n * size, alloc, GFP_KERNEL);
	__nvgpu_check_kalloc_size(n * size);

	__nvgpu_save_kmem_alloc(g->kmallocs, n * size,
				roundup_pow_of_two(n * size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void __nvgpu_track_vfree(struct gk20a *g, void *addr)
{
	/*
	 * Often it is accepted practice to pass NULL pointers into free
	 * functions to save code.
	 */
	if (!addr)
		return;

	vfree(addr);

	kmem_dbg("vfree: addr=0x%p", addr);

	__nvgpu_free_kmem_alloc(g->vmallocs, (u64)(uintptr_t)addr);
}

void __nvgpu_track_kfree(struct gk20a *g, void *addr)
{
	if (!addr)
		return;

	kfree(addr);

	kmem_dbg("kfree: addr=0x%p", addr);

	__nvgpu_free_kmem_alloc(g->kmallocs, (u64)(uintptr_t)addr);
}

/**
 * to_human_readable_bytes - Determine  suffix for passed size.
 *
 * @bytes - Number of bytes to generate a suffix for.
 * @hr_bytes [out] - The human readable number of bytes.
 * @hr_suffix [out] - The suffix for the HR number of bytes.
 *
 * Computes a human readable decomposition of the passed number of bytes. The
 * suffix for the bytes is passed back through the @hr_suffix pointer. The right
 * number of bytes is then passed back in @hr_bytes. This returns the following
 * ranges:
 *
 *   0 - 1023 B
 *   1 - 1023 KB
 *   1 - 1023 MB
 *   1 - 1023 GB
 *   1 - 1023 TB
 *   1 - ...  PB
 */
static void __to_human_readable_bytes(u64 bytes, u64 *hr_bytes,
				      const char **hr_suffix)
{
	static const char *suffixes[] =
		{ "B", "KB", "MB", "GB", "TB", "PB" };

	u64 suffix_ind = 0;

	while (suffix_ind < ARRAY_SIZE(suffixes) && bytes >= 1024) {
		bytes >>= 10;
		suffix_ind++;
	}

	/*
	 * Handle case where bytes > 1023PB.
	 */
	suffix_ind = suffix_ind < ARRAY_SIZE(suffixes) ?
		suffix_ind : ARRAY_SIZE(suffixes) - 1;

	*hr_bytes = bytes;
	*hr_suffix = suffixes[suffix_ind];
}

/**
 * print_hr_bytes - Print human readable bytes
 *
 * @s - A seq_file to print to. May be NULL.
 * @msg - A message to print before the bytes.
 * @bytes - Number of bytes.
 *
 * Print @msg followed by the human readable decomposition of the passed number
 * of bytes.
 *
 * If @s is NULL then this prints will be made to the kernel log.
 */
static void print_hr_bytes(struct seq_file *s, const char *msg, u64 bytes)
{
	u64 hr_bytes;
	const char *hr_suffix;

	__to_human_readable_bytes(bytes, &hr_bytes, &hr_suffix);
	__pstat(s, "%s%lld %s\n", msg, hr_bytes, hr_suffix);
}

/**
 * print_histogram - Build a histogram of the memory usage.
 *
 * @tracker The tracking to pull data from.
 * @s       A seq_file to dump info into.
 */
static void print_histogram(struct nvgpu_mem_alloc_tracker *tracker,
			    struct seq_file *s)
{
	int i;
	u64 pot_min, pot_max;
	u64 nr_buckets;
	unsigned int *buckets;
	unsigned int total_allocs;
	struct rb_node *node;
	static const char histogram_line[] =
		"++++++++++++++++++++++++++++++++++++++++";

	/*
	 * pot_min is essentially a round down to the nearest power of 2. This
	 * is the start of the histogram. pot_max is just a round up to the
	 * nearest power of two. Each histogram bucket is one power of two so
	 * the histogram buckets are exponential.
	 */
	pot_min = (u64)rounddown_pow_of_two(tracker->min_alloc);
	pot_max = (u64)roundup_pow_of_two(tracker->max_alloc);

	nr_buckets = __ffs(pot_max) - __ffs(pot_min);

	buckets = kzalloc(sizeof(*buckets) * nr_buckets, GFP_KERNEL);
	if (!buckets) {
		__pstat(s, "OOM: could not allocate bucket storage!?\n");
		return;
	}

	/*
	 * Iterate across all of the allocs and determine what bucket they
	 * should go in. Round the size down to the nearest power of two to
	 * find the right bucket.
	 */
	for (node = rb_first(&tracker->allocs);
	     node != NULL;
	     node = rb_next(node)) {
		int b;
		u64 bucket_min;
		struct nvgpu_mem_alloc *alloc;

		alloc = container_of(node, struct nvgpu_mem_alloc,
				     allocs_entry);
		bucket_min = (u64)rounddown_pow_of_two(alloc->size);
		if (bucket_min < tracker->min_alloc)
			bucket_min = tracker->min_alloc;

		b = __ffs(bucket_min) - __ffs(pot_min);

		/*
		 * Handle the one case were there's an alloc exactly as big as
		 * the maximum bucket size of the largest bucket. Most of the
		 * buckets have an inclusive minimum and exclusive maximum. But
		 * the largest bucket needs to have an _inclusive_ maximum as
		 * well.
		 */
		if (b == (int)nr_buckets)
			b--;

		buckets[b]++;
	}

	total_allocs = 0;
	for (i = 0; i < (int)nr_buckets; i++)
		total_allocs += buckets[i];

	__pstat(s, "Alloc histogram:\n");

	/*
	 * Actually compute the histogram lines.
	 */
	for (i = 0; i < (int)nr_buckets; i++) {
		char this_line[sizeof(histogram_line) + 1];
		u64 line_length;
		u64 hr_bytes;
		const char *hr_suffix;

		memset(this_line, 0, sizeof(this_line));

		/*
		 * Compute the normalized line length. Cant use floating point
		 * so we will just multiply everything by 1000 and use fixed
		 * point.
		 */
		line_length = (1000 * buckets[i]) / total_allocs;
		line_length *= sizeof(histogram_line);
		line_length /= 1000;

		memset(this_line, '+', line_length);

		__to_human_readable_bytes(1 << (__ffs(pot_min) + i),
					  &hr_bytes, &hr_suffix);
		__pstat(s, "  [%-4lld %-4lld] %-2s %5u | %s\n",
			hr_bytes, hr_bytes << 1,
			hr_suffix, buckets[i], this_line);
	}
}

/**
 * nvgpu_kmem_print_stats - Print kmem tracking stats.
 *
 * @tracker The tracking to pull data from.
 * @s       A seq_file to dump info into.
 *
 * Print stats from a tracker. If @s is non-null then seq_printf() will be
 * used with @s. Otherwise the stats are pr_info()ed.
 */
void nvgpu_kmem_print_stats(struct nvgpu_mem_alloc_tracker *tracker,
			    struct seq_file *s)
{
	lock_tracker(tracker);

	__pstat(s, "Mem tracker: %s\n\n", tracker->name);

	__pstat(s, "Basic Stats:\n");
	__pstat(s,        "  Number of allocs        %lld\n",
		tracker->nr_allocs);
	__pstat(s,        "  Number of frees         %lld\n",
		tracker->nr_frees);
	print_hr_bytes(s, "  Smallest alloc          ", tracker->min_alloc);
	print_hr_bytes(s, "  Largest alloc           ", tracker->max_alloc);
	print_hr_bytes(s, "  Bytes allocated         ", tracker->bytes_alloced);
	print_hr_bytes(s, "  Bytes freed             ", tracker->bytes_freed);
	print_hr_bytes(s, "  Bytes allocated (real)  ",
		       tracker->bytes_alloced_real);
	print_hr_bytes(s, "  Bytes freed (real)      ",
		       tracker->bytes_freed_real);
	__pstat(s, "\n");

	print_histogram(tracker, s);

	unlock_tracker(tracker);
}

#if defined(CONFIG_DEBUG_FS)
static int __kmem_tracking_show(struct seq_file *s, void *unused)
{
	struct nvgpu_mem_alloc_tracker *tracker = s->private;

	nvgpu_kmem_print_stats(tracker, s);

	return 0;
}

static int __kmem_tracking_open(struct inode *inode, struct file *file)
{
	return single_open(file, __kmem_tracking_show, inode->i_private);
}

static const struct file_operations __kmem_tracking_fops = {
	.open = __kmem_tracking_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __kmem_traces_dump_tracker(struct gk20a *g,
				      struct nvgpu_mem_alloc_tracker *tracker,
				      struct seq_file *s)
{
	struct rb_node *node;

	for (node = rb_first(&tracker->allocs);
	     node != NULL;
	     node = rb_next(node)) {
		struct nvgpu_mem_alloc *alloc;

		alloc = container_of(node, struct nvgpu_mem_alloc,
				     allocs_entry);

		kmem_print_mem_alloc(g, alloc, s);
	}

	return 0;
}

static int __kmem_traces_show(struct seq_file *s, void *unused)
{
	struct gk20a *g = s->private;

	lock_tracker(g->vmallocs);
	seq_puts(s, "Oustanding vmallocs:\n");
	__kmem_traces_dump_tracker(g, g->vmallocs, s);
	seq_puts(s, "\n");
	unlock_tracker(g->vmallocs);

	lock_tracker(g->kmallocs);
	seq_puts(s, "Oustanding kmallocs:\n");
	__kmem_traces_dump_tracker(g, g->kmallocs, s);
	unlock_tracker(g->kmallocs);

	return 0;
}

static int __kmem_traces_open(struct inode *inode, struct file *file)
{
	return single_open(file, __kmem_traces_show, inode->i_private);
}

static const struct file_operations __kmem_traces_fops = {
	.open = __kmem_traces_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void nvgpu_kmem_debugfs_init(struct device *dev)
{
	struct gk20a_platform *plat = dev_get_drvdata(dev);
	struct gk20a *g = get_gk20a(dev);
	struct dentry *gpu_root = plat->debugfs;
	struct dentry *node;

	g->debugfs_kmem = debugfs_create_dir("kmem_tracking", gpu_root);
	if (IS_ERR_OR_NULL(g->debugfs_kmem))
		return;

	node = debugfs_create_file(g->vmallocs->name, S_IRUGO,
				   g->debugfs_kmem,
				   g->vmallocs, &__kmem_tracking_fops);
	node = debugfs_create_file(g->kmallocs->name, S_IRUGO,
				   g->debugfs_kmem,
				   g->kmallocs, &__kmem_tracking_fops);
	node = debugfs_create_file("traces", S_IRUGO,
				   g->debugfs_kmem,
				   g, &__kmem_traces_fops);
}
#else
void nvgpu_kmem_debugfs_init(struct device *dev)
{
}
#endif

static int __do_check_for_outstanding_allocs(
	struct gk20a *g,
	struct nvgpu_mem_alloc_tracker *tracker,
	const char *type, bool silent)
{
	struct rb_node *node;
	int count = 0;

	for (node = rb_first(&tracker->allocs);
	     node != NULL;
	     node = rb_next(node)) {
		struct nvgpu_mem_alloc *alloc;

		alloc = container_of(node, struct nvgpu_mem_alloc,
				     allocs_entry);

		if (!silent)
			kmem_print_mem_alloc(g, alloc, NULL);

		count++;
	}

	return count;
}

/**
 * check_for_outstanding_allocs - Count and display outstanding allocs
 *
 * @g      - The GPU.
 * @silent - If set don't print anything about the allocs.
 *
 * Dump (or just count) the number of allocations left outstanding.
 */
static int check_for_outstanding_allocs(struct gk20a *g, bool silent)
{
	int count = 0;

	count += __do_check_for_outstanding_allocs(g, g->kmallocs, "kmalloc",
						   silent);
	count += __do_check_for_outstanding_allocs(g, g->vmallocs, "vmalloc",
						   silent);

	return count;
}

static void do_nvgpu_kmem_cleanup(struct nvgpu_mem_alloc_tracker *tracker,
				  void (*force_free_func)(const void *))
{
	struct rb_node *node;

	while ((node = rb_first(&tracker->allocs)) != NULL) {
		struct nvgpu_mem_alloc *alloc;

		alloc = container_of(node, struct nvgpu_mem_alloc,
				     allocs_entry);
		if (force_free_func)
			force_free_func((void *)alloc->addr);

		kfree(alloc);
	}
}

/**
 * nvgpu_kmem_cleanup - Cleanup the kmem tracking
 *
 * @g          - The GPU.
 * @force_free - If set will also free leaked objects if possible.
 *
 * Cleanup all of the allocs made by nvgpu_kmem tracking code. If @force_free
 * is non-zero then the allocation made by nvgpu is also freed. This is risky,
 * though, as it is possible that the memory is still in use by other parts of
 * the GPU driver not aware that this has happened.
 *
 * In theory it should be fine if the GPU driver has been deinitialized and
 * there are no bugs in that code. However, if there are any bugs in that code
 * then they could likely manifest as odd crashes indeterminate amounts of time
 * in the future. So use @force_free at your own risk.
 */
static void nvgpu_kmem_cleanup(struct gk20a *g, bool force_free)
{
	do_nvgpu_kmem_cleanup(g->kmallocs, force_free ? kfree : NULL);
	do_nvgpu_kmem_cleanup(g->vmallocs, force_free ? vfree : NULL);
}

void nvgpu_kmem_fini(struct gk20a *g, int flags)
{
	int count;
	bool silent, force_free;

	if (!flags)
		return;

	silent = !(flags & NVGPU_KMEM_FINI_DUMP_ALLOCS);
	force_free = !!(flags & NVGPU_KMEM_FINI_FORCE_CLEANUP);

	count = check_for_outstanding_allocs(g, silent);
	nvgpu_kmem_cleanup(g, force_free);

	/*
	 * If we leak objects we can either BUG() out or just WARN(). In general
	 * it doesn't make sense to BUG() on here since leaking a few objects
	 * won't crash the kernel but it can be helpful for development.
	 *
	 * If neither flag is set then we just silently do nothing.
	 */
	if (count > 0) {
		if (flags & NVGPU_KMEM_FINI_WARN) {
			WARN(1, "Letting %d allocs leak!!\n", count);
		} else if (flags & NVGPU_KMEM_FINI_BUG) {
			gk20a_err(g->dev, "Letting %d allocs leak!!\n", count);
			BUG();
		}
	}
}

int nvgpu_kmem_init(struct gk20a *g)
{
	int err;

	g->vmallocs = kzalloc(sizeof(*g->vmallocs), GFP_KERNEL);
	g->kmallocs = kzalloc(sizeof(*g->kmallocs), GFP_KERNEL);

	if (!g->vmallocs || !g->kmallocs) {
		err = -ENOMEM;
		goto fail;
	}

	g->vmallocs->name = "vmalloc";
	g->kmallocs->name = "kmalloc";

	g->vmallocs->allocs = RB_ROOT;
	g->kmallocs->allocs = RB_ROOT;

	mutex_init(&g->vmallocs->lock);
	mutex_init(&g->kmallocs->lock);

	g->vmallocs->min_alloc = PAGE_SIZE;
	g->kmallocs->min_alloc = KMALLOC_MIN_SIZE;

	/*
	 * This needs to go after all the other initialization since they use
	 * the nvgpu_kzalloc() API.
	 */
	g->vmallocs->allocs_cache = nvgpu_kmem_cache_create(g,
						sizeof(struct nvgpu_mem_alloc));
	g->kmallocs->allocs_cache = nvgpu_kmem_cache_create(g,
						sizeof(struct nvgpu_mem_alloc));

	if (!g->vmallocs->allocs_cache || !g->kmallocs->allocs_cache) {
		err = -ENOMEM;
		if (g->vmallocs->allocs_cache)
			nvgpu_kmem_cache_destroy(g->vmallocs->allocs_cache);
		if (g->kmallocs->allocs_cache)
			nvgpu_kmem_cache_destroy(g->kmallocs->allocs_cache);
		goto fail;
	}

	return 0;

fail:
	if (g->vmallocs)
		kfree(g->vmallocs);
	if (g->kmallocs)
		kfree(g->kmallocs);
	return err;
}

#else /* !CONFIG_NVGPU_TRACK_MEM_USAGE */

int nvgpu_kmem_init(struct gk20a *g)
{
	return 0;
}

void nvgpu_kmem_fini(struct gk20a *g, int flags)
{
}
#endif /* CONFIG_NVGPU_TRACK_MEM_USAGE */

struct nvgpu_kmem_cache *nvgpu_kmem_cache_create(struct gk20a *g, size_t size)
{
	struct nvgpu_kmem_cache *cache =
		nvgpu_kzalloc(g, sizeof(struct nvgpu_kmem_cache));

	if (!cache)
		return NULL;

	cache->g = g;

	snprintf(cache->name, sizeof(cache->name),
		 "nvgpu-cache-0x%p-%d-%d", g, (int)size,
		 atomic_inc_return(&kmem_cache_id));
	cache->cache = kmem_cache_create(cache->name,
					 size, size, 0, NULL);
	if (!cache->cache) {
		nvgpu_kfree(g, cache);
		return NULL;
	}

	return cache;
}

void nvgpu_kmem_cache_destroy(struct nvgpu_kmem_cache *cache)
{
	struct gk20a *g = cache->g;

	kmem_cache_destroy(cache->cache);
	nvgpu_kfree(g, cache);
}

void *nvgpu_kmem_cache_alloc(struct nvgpu_kmem_cache *cache)
{
	return kmem_cache_alloc(cache->cache, GFP_KERNEL);
}

void nvgpu_kmem_cache_free(struct nvgpu_kmem_cache *cache, void *ptr)
{
	kmem_cache_free(cache->cache, ptr);
}
