/*
 * drivers/video/tegra/nvmap/nvmap_dev.c
 *
 * User-space interface to nvmap
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/backing-dev.h>
#include <linux/bitmap.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/oom.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/nvmap.h>
#include <linux/module.h>
#include <linux/resource.h>
#include <linux/security.h>
#include <linux/stat.h>
#include <linux/kthread.h>
#include <linux/highmem.h>
#include <linux/lzo.h>

#include <asm/cputype.h>

#define CREATE_TRACE_POINTS
#include <trace/events/nvmap.h>

#include "nvmap_priv.h"
#include "nvmap_ioctl.h"

#define NVMAP_CARVEOUT_KILLER_RETRY_TIME 100 /* msecs */

/* this is basically the L2 cache size */
#ifdef CONFIG_DENVER_CPU
size_t cache_maint_inner_threshold = SZ_2M * 8;
#else
size_t cache_maint_inner_threshold = SZ_2M;
#endif

#ifdef CONFIG_NVMAP_OUTER_CACHE_MAINT_BY_SET_WAYS
size_t cache_maint_outer_threshold = SZ_1M;
#endif

struct nvmap_device *nvmap_dev;
struct nvmap_stats nvmap_stats;

static struct backing_dev_info nvmap_bdi = {
	.ra_pages	= 0,
	.capabilities	= (BDI_CAP_NO_ACCT_AND_WRITEBACK |
			   BDI_CAP_READ_MAP | BDI_CAP_WRITE_MAP),
};

static struct device_dma_parameters nvmap_dma_parameters = {
	.max_segment_size = UINT_MAX,
};

static int nvmap_open(struct inode *inode, struct file *filp);
static int nvmap_release(struct inode *inode, struct file *filp);
static long nvmap_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int nvmap_map(struct file *filp, struct vm_area_struct *vma);
static void nvmap_vma_close(struct vm_area_struct *vma);
static int nvmap_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf);

static const struct file_operations nvmap_user_fops = {
	.owner		= THIS_MODULE,
	.open		= nvmap_open,
	.release	= nvmap_release,
	.unlocked_ioctl	= nvmap_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvmap_ioctl,
#endif
	.mmap		= nvmap_map,
};

static struct vm_operations_struct nvmap_vma_ops = {
	.open		= nvmap_vma_open,
	.close		= nvmap_vma_close,
	.fault		= nvmap_vma_fault,
};

int is_nvmap_vma(struct vm_area_struct *vma)
{
	return vma->vm_ops == &nvmap_vma_ops;
}

/*
 * Verifies that the passed ID is a valid handle ID. Then the passed client's
 * reference to the handle is returned.
 *
 * Note: to call this function make sure you own the client ref lock.
 */
struct nvmap_handle_ref *__nvmap_validate_locked(struct nvmap_client *c,
						 struct nvmap_handle *h)
{
	struct rb_node *n = c->handle_refs.rb_node;

	while (n) {
		struct nvmap_handle_ref *ref;
		ref = rb_entry(n, struct nvmap_handle_ref, node);
		if (ref->handle == h)
			return ref;
		else if ((uintptr_t)h > (uintptr_t)ref->handle)
			n = n->rb_right;
		else
			n = n->rb_left;
	}

	return NULL;
}

unsigned long nvmap_carveout_usage(struct nvmap_client *c,
				   struct nvmap_heap_block *b)
{
	struct nvmap_heap *h = nvmap_block_to_heap(b);
	struct nvmap_carveout_node *n;
	int i;

	for (i = 0; i < nvmap_dev->nr_carveouts; i++) {
		n = &nvmap_dev->heaps[i];
		if (n->carveout == h)
			return n->heap_bit;
	}
	return 0;
}

/*
 * This routine is used to flush the carveout memory from cache.
 * Why cache flush is needed for carveout? Consider the case, where a piece of
 * carveout is allocated as cached and released. After this, if the same memory is
 * allocated for uncached request and the memory is not flushed out from cache.
 * In this case, the client might pass this to H/W engine and it could start modify
 * the memory. As this was cached earlier, it might have some portion of it in cache.
 * During cpu request to read/write other memory, the cached portion of this memory
 * might get flushed back to main memory and would cause corruptions, if it happens
 * after H/W writes data to memory.
 *
 * But flushing out the memory blindly on each carveout allocation is redundant.
 *
 * In order to optimize the carveout buffer cache flushes, the following
 * strategy is used.
 *
 * The whole Carveout is flushed out from cache during its initialization.
 * During allocation, carveout buffers are not flused from cache.
 * During deallocation, carveout buffers are flushed, if they were allocated as cached.
 * if they were allocated as uncached/writecombined, no cache flush is needed.
 * Just draining store buffers is enough.
 */
int nvmap_flush_heap_block(struct nvmap_client *client,
	struct nvmap_heap_block *block, size_t len, unsigned int prot)
{
	ulong kaddr;
	phys_addr_t phys = block->base;
	phys_addr_t end = block->base + len;
	struct vm_struct *area = NULL;

	if (prot == NVMAP_HANDLE_UNCACHEABLE || prot == NVMAP_HANDLE_WRITE_COMBINE)
		goto out;

#ifdef CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS
	if (len >= cache_maint_inner_threshold) {
		inner_flush_cache_all();
		if (prot != NVMAP_HANDLE_INNER_CACHEABLE)
			outer_flush_range(block->base, block->base + len);
		goto out;
	}
#endif

	area = alloc_vm_area(PAGE_SIZE, NULL);
	if (!area)
		return -ENOMEM;

	kaddr = (ulong)area->addr;

	while (phys < end) {
		phys_addr_t next = (phys + PAGE_SIZE) & PAGE_MASK;
		void *base = (void *)kaddr + (phys & ~PAGE_MASK);

		next = min(next, end);
		ioremap_page_range(kaddr, kaddr + PAGE_SIZE,
			phys, PG_PROT_KERNEL);
		FLUSH_DCACHE_AREA(base, next - phys);
		phys = next;
		unmap_kernel_range(kaddr, PAGE_SIZE);
	}

	if (prot != NVMAP_HANDLE_INNER_CACHEABLE)
		outer_flush_range(block->base, block->base + len);

	free_vm_area(area);
out:
	wmb();
	return 0;
}

static
struct nvmap_heap_block *do_nvmap_carveout_alloc(struct nvmap_client *client,
					      struct nvmap_handle *handle,
					      unsigned long type)
{
	struct nvmap_carveout_node *co_heap;
	struct nvmap_device *dev = nvmap_dev;
	int i;

	for (i = 0; i < dev->nr_carveouts; i++) {
		struct nvmap_heap_block *block;
		co_heap = &dev->heaps[i];

		if (!(co_heap->heap_bit & type))
			continue;

		block = nvmap_heap_alloc(co_heap->carveout, handle);
		if (block)
			return block;
	}
	return NULL;
}

struct nvmap_heap_block *nvmap_carveout_alloc(struct nvmap_client *client,
					      struct nvmap_handle *handle,
					      unsigned long type)
{
	return do_nvmap_carveout_alloc(client, handle, type);
}

/* remove a handle from the device's tree of all handles; called
 * when freeing handles. */
int nvmap_handle_remove(struct nvmap_device *dev, struct nvmap_handle *h)
{
	spin_lock(&dev->handle_lock);

	/* re-test inside the spinlock if the handle really has no clients;
	 * only remove the handle if it is unreferenced */
	if (atomic_add_return(0, &h->ref) > 0) {
		spin_unlock(&dev->handle_lock);
		return -EBUSY;
	}
	smp_rmb();
	BUG_ON(atomic_read(&h->ref) < 0);
	BUG_ON(atomic_read(&h->pin) != 0);

	nvmap_lru_del(h);
	rb_erase(&h->node, &dev->handles);

	spin_unlock(&dev->handle_lock);
	return 0;
}

/* adds a newly-created handle to the device master tree */
void nvmap_handle_add(struct nvmap_device *dev, struct nvmap_handle *h)
{
	struct rb_node **p;
	struct rb_node *parent = NULL;

	spin_lock(&dev->handle_lock);
	p = &dev->handles.rb_node;
	while (*p) {
		struct nvmap_handle *b;

		parent = *p;
		b = rb_entry(parent, struct nvmap_handle, node);
		if (h > b)
			p = &parent->rb_right;
		else
			p = &parent->rb_left;
	}
	rb_link_node(&h->node, parent, p);
	rb_insert_color(&h->node, &dev->handles);
	nvmap_lru_add(h);
	spin_unlock(&dev->handle_lock);
}

/* Validates that a handle is in the device master tree and that the
 * client has permission to access it. */
struct nvmap_handle *nvmap_validate_get(struct nvmap_handle *id)
{
	struct nvmap_handle *h = NULL;
	struct rb_node *n;

	spin_lock(&nvmap_dev->handle_lock);

	n = nvmap_dev->handles.rb_node;

	while (n) {
		h = rb_entry(n, struct nvmap_handle, node);
		if (h == id) {
			h = nvmap_handle_get(h);
			spin_unlock(&nvmap_dev->handle_lock);
			return h;
		}
		if (id > h)
			n = n->rb_right;
		else
			n = n->rb_left;
	}
	spin_unlock(&nvmap_dev->handle_lock);
	return NULL;
}

struct nvmap_client *__nvmap_create_client(struct nvmap_device *dev,
					   const char *name)
{
	struct nvmap_client *client;
	struct task_struct *task;

	if (WARN_ON(!dev))
		return NULL;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return NULL;

	client->name = name;
	client->kernel_client = true;
	client->handle_refs = RB_ROOT;

	get_task_struct(current->group_leader);
	task_lock(current->group_leader);
	/* don't bother to store task struct for kernel threads,
	   they can't be killed anyway */
	if (current->flags & PF_KTHREAD) {
		put_task_struct(current->group_leader);
		task = NULL;
	} else {
		task = current->group_leader;
	}
	task_unlock(current->group_leader);
	client->task = task;

	mutex_init(&client->ref_lock);
	atomic_set(&client->count, 1);

	mutex_lock(&dev->clients_lock);
	list_add(&client->list, &dev->clients);
	mutex_unlock(&dev->clients_lock);
	return client;
}

static void destroy_client(struct nvmap_client *client)
{
	struct rb_node *n;

	if (!client)
		return;

	mutex_lock(&nvmap_dev->clients_lock);
	list_del(&client->list);
	mutex_unlock(&nvmap_dev->clients_lock);

	while ((n = rb_first(&client->handle_refs))) {
		struct nvmap_handle_ref *ref;
		int pins, dupes;

		ref = rb_entry(n, struct nvmap_handle_ref, node);

		smp_rmb();
		pins = atomic_read(&ref->pin);

		while (pins--)
			__nvmap_unpin(ref);

		if (ref->handle->owner == client)
			ref->handle->owner = NULL;

		dma_buf_put(ref->handle->dmabuf);
		rb_erase(&ref->node, &client->handle_refs);
		atomic_dec(&ref->handle->share_count);

		dupes = atomic_read(&ref->dupes);
		while (dupes--)
			nvmap_handle_put(ref->handle);

		kfree(ref);
	}

	if (client->task)
		put_task_struct(client->task);

	kfree(client);
}

struct nvmap_client *nvmap_client_get(struct nvmap_client *client)
{
	if (!virt_addr_valid(client))
		return NULL;

	if (!atomic_add_unless(&client->count, 1, 0))
		return NULL;

	return client;
}

void nvmap_client_put(struct nvmap_client *client)
{
	if (!client)
		return;

	if (!atomic_dec_return(&client->count))
		destroy_client(client);
}

static int nvmap_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *miscdev = filp->private_data;
	struct nvmap_device *dev = dev_get_drvdata(miscdev->parent);
	struct nvmap_client *priv;
	int ret;
	__attribute__((unused)) struct rlimit old_rlim, new_rlim;

	ret = nonseekable_open(inode, filp);
	if (unlikely(ret))
		return ret;

	BUG_ON(dev != nvmap_dev);
	priv = __nvmap_create_client(dev, "user");
	if (!priv)
		return -ENOMEM;
	trace_nvmap_open(priv, priv->name);

	priv->kernel_client = false;

	filp->f_mapping->backing_dev_info = &nvmap_bdi;

	filp->private_data = priv;
	return 0;
}

static int nvmap_release(struct inode *inode, struct file *filp)
{
	struct nvmap_client *priv = filp->private_data;

	trace_nvmap_release(priv, priv->name);
	nvmap_client_put(priv);
	return 0;
}

int __nvmap_map(struct nvmap_handle *h, struct vm_area_struct *vma)
{
	struct nvmap_vma_priv *priv;

	h = nvmap_handle_get(h);
	if (!h)
		return -EINVAL;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->handle = h;

	vma->vm_flags |= VM_SHARED | VM_DONTEXPAND |
			  VM_DONTDUMP | VM_DONTCOPY |
			  (h->heap_pgalloc ? 0 : VM_PFNMAP);
	vma->vm_ops = &nvmap_vma_ops;
	BUG_ON(vma->vm_private_data != NULL);
	vma->vm_private_data = priv;
	vma->vm_page_prot = nvmap_pgprot(h, vma->vm_page_prot);
	nvmap_vma_open(vma);
	return 0;
}

static int nvmap_map(struct file *filp, struct vm_area_struct *vma)
{
	BUG_ON(vma->vm_private_data != NULL);
	vma->vm_flags |= (VM_SHARED | VM_DONTEXPAND |
			  VM_DONTDUMP | VM_DONTCOPY);
	vma->vm_ops = &nvmap_vma_ops;
	return 0;
}

static long nvmap_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *uarg = (void __user *)arg;

	if (_IOC_TYPE(cmd) != NVMAP_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > NVMAP_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, uarg, _IOC_SIZE(cmd));
	if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, uarg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
	case NVMAP_IOC_CREATE:
	case NVMAP_IOC_FROM_FD:
		err = nvmap_ioctl_create(filp, cmd, uarg);
		break;

	case NVMAP_IOC_FROM_ID:
	case NVMAP_IOC_GET_ID:
		pr_warn_once("nvmap: unsupported FROM_ID/GET_ID IOCTLs used.\n");
		return -ENOTTY;

	case NVMAP_IOC_GET_FD:
		err = nvmap_ioctl_getfd(filp, uarg);
		break;

#ifdef CONFIG_COMPAT
	case NVMAP_IOC_PARAM_32:
		err = nvmap_ioctl_get_param(filp, uarg, true);
		break;
#endif

	case NVMAP_IOC_PARAM:
		err = nvmap_ioctl_get_param(filp, uarg, false);
		break;

#ifdef CONFIG_COMPAT
	case NVMAP_IOC_UNPIN_MULT_32:
	case NVMAP_IOC_PIN_MULT_32:
		err = nvmap_ioctl_pinop(filp, cmd == NVMAP_IOC_PIN_MULT_32,
			uarg, true);
		break;
#endif

	case NVMAP_IOC_UNPIN_MULT:
	case NVMAP_IOC_PIN_MULT:
		err = nvmap_ioctl_pinop(filp, cmd == NVMAP_IOC_PIN_MULT,
			uarg, false);
		break;

	case NVMAP_IOC_ALLOC:
		err = nvmap_ioctl_alloc(filp, uarg);
		break;

	case NVMAP_IOC_ALLOC_KIND:
		err = nvmap_ioctl_alloc_kind(filp, uarg);
		break;

	case NVMAP_IOC_FREE:
		err = nvmap_ioctl_free(filp, arg);
		break;

#ifdef CONFIG_COMPAT
	case NVMAP_IOC_MMAP_32:
		err = nvmap_map_into_caller_ptr(filp, uarg, true);
		break;
#endif

	case NVMAP_IOC_MMAP:
		err = nvmap_map_into_caller_ptr(filp, uarg, false);
		break;

#ifdef CONFIG_COMPAT
	case NVMAP_IOC_WRITE_32:
	case NVMAP_IOC_READ_32:
		err = nvmap_ioctl_rw_handle(filp, cmd == NVMAP_IOC_READ_32,
			uarg, true);
		break;
#endif

	case NVMAP_IOC_WRITE:
	case NVMAP_IOC_READ:
		err = nvmap_ioctl_rw_handle(filp, cmd == NVMAP_IOC_READ, uarg,
			false);
		break;

#ifdef CONFIG_COMPAT
	case NVMAP_IOC_CACHE_32:
		err = nvmap_ioctl_cache_maint(filp, uarg, true);
		break;
#endif

	case NVMAP_IOC_CACHE:
		err = nvmap_ioctl_cache_maint(filp, uarg, false);
		break;

	case NVMAP_IOC_CACHE_LIST:
	case NVMAP_IOC_RESERVE:
		err = nvmap_ioctl_cache_maint_list(filp, uarg,
						   cmd == NVMAP_IOC_RESERVE);
		break;

	case NVMAP_IOC_SHARE:
		err = nvmap_ioctl_share_dmabuf(filp, uarg);
		break;

	default:
		return -ENOTTY;
	}
	return err;
}

/* to ensure that the backing store for the VMA isn't freed while a fork'd
 * reference still exists, nvmap_vma_open increments the reference count on
 * the handle, and nvmap_vma_close decrements it. alternatively, we could
 * disallow copying of the vma, or behave like pmem and zap the pages. FIXME.
*/
void nvmap_vma_open(struct vm_area_struct *vma)
{
	struct nvmap_vma_priv *priv;
	struct nvmap_handle *h;
	struct nvmap_vma_list *vma_list, *tmp;
	struct list_head *tmp_head = NULL;
	pid_t current_pid = current->pid;
	bool vma_pos_found = false;

	priv = vma->vm_private_data;
	BUG_ON(!priv);
	BUG_ON(!priv->handle);

	atomic_inc(&priv->count);
	h = priv->handle;
	nvmap_umaps_inc(h);

	vma_list = kmalloc(sizeof(*vma_list), GFP_KERNEL);
	if (vma_list) {
		mutex_lock(&h->lock);
		tmp_head = &h->vmas;

		/* insert vma into handle's vmas list in the increasing order of
		 * handle offsets
		 */
		list_for_each_entry(tmp, &h->vmas, list) {
			BUG_ON(tmp->vma == vma);

			if (!vma_pos_found && (current_pid == tmp->pid)) {
				if (vma->vm_pgoff < tmp->vma->vm_pgoff) {
					tmp_head = &tmp->list;
					vma_pos_found = true;
				} else {
					tmp_head = tmp->list.next;
				}
			}
		}

		vma_list->vma = vma;
		vma_list->pid = current_pid;
		list_add_tail(&vma_list->list, tmp_head);
		mutex_unlock(&h->lock);
	} else {
		WARN(1, "vma not tracked");
	}
}

static void nvmap_vma_close(struct vm_area_struct *vma)
{
	struct nvmap_vma_priv *priv = vma->vm_private_data;
	struct nvmap_vma_list *vma_list;
	struct nvmap_handle *h;
	bool vma_found = false;

	if (!priv)
		return;

	BUG_ON(!priv->handle);

	h = priv->handle;
	mutex_lock(&h->lock);
	list_for_each_entry(vma_list, &h->vmas, list) {
		if (vma_list->vma != vma)
			continue;
		list_del(&vma_list->list);
		kfree(vma_list);
		vma_found = true;
		break;
	}
	BUG_ON(!vma_found);
	mutex_unlock(&h->lock);

	if (__atomic_add_unless(&priv->count, -1, 0) == 1) {
		if (priv->handle)
			nvmap_handle_put(priv->handle);
		vma->vm_private_data = NULL;
		kfree(priv);
	}
	nvmap_umaps_dec(h);
}

static int nvmap_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct page *page;
	struct nvmap_vma_priv *priv;
	unsigned long offs;

	offs = (unsigned long)(vmf->virtual_address - vma->vm_start);
	priv = vma->vm_private_data;
	if (!priv || !priv->handle || !priv->handle->alloc)
		return VM_FAULT_SIGBUS;

	offs += priv->offs;
	/* if the VMA was split for some reason, vm_pgoff will be the VMA's
	 * offset from the original VMA */
	offs += (vma->vm_pgoff << PAGE_SHIFT);

	if (offs >= priv->handle->size)
		return VM_FAULT_SIGBUS;

	if (!priv->handle->heap_pgalloc) {
		unsigned long pfn;
		BUG_ON(priv->handle->carveout->base & ~PAGE_MASK);
		pfn = ((priv->handle->carveout->base + offs) >> PAGE_SHIFT);
		if (!pfn_valid(pfn)) {
			vm_insert_pfn(vma,
				(unsigned long)vmf->virtual_address, pfn);
			return VM_FAULT_NOPAGE;
		}
		/* CMA memory would get here */
		page = pfn_to_page(pfn);
	} else {
		offs >>= PAGE_SHIFT;
		if (nvmap_page_reserved(priv->handle->pgalloc.pages[offs]))
			return VM_FAULT_SIGBUS;
		page = nvmap_to_page(priv->handle->pgalloc.pages[offs]);
		nvmap_page_mkdirty(&priv->handle->pgalloc.pages[offs]);
	}

	if (page)
		get_page(page);
	vmf->page = page;
	return (page) ? 0 : VM_FAULT_SIGBUS;
}

#define DEBUGFS_OPEN_FOPS(name) \
static int nvmap_debug_##name##_open(struct inode *inode, \
					    struct file *file) \
{ \
	return single_open(file, nvmap_debug_##name##_show, \
			    inode->i_private); \
} \
\
static const struct file_operations debug_##name##_fops = { \
	.open = nvmap_debug_##name##_open, \
	.read = seq_read, \
	.llseek = seq_lseek, \
	.release = single_release, \
}

#define K(x) (x >> 10)

static void client_stringify(struct nvmap_client *client, struct seq_file *s)
{
	char task_comm[TASK_COMM_LEN];
	if (!client->task) {
		seq_printf(s, "%-18s %18s %8u", client->name, "kernel", 0);
		return;
	}
	get_task_comm(task_comm, client->task);
	seq_printf(s, "%-18s %18s %8u", client->name, task_comm,
		   client->task->pid);
}

static void allocations_stringify(struct nvmap_client *client,
				  struct seq_file *s, u32 heap_type)
{
	struct rb_node *n;

	nvmap_ref_lock(client);
	n = rb_first(&client->handle_refs);
	for (; n != NULL; n = rb_next(n)) {
		struct nvmap_handle_ref *ref =
			rb_entry(n, struct nvmap_handle_ref, node);
		struct nvmap_handle *handle = ref->handle;
		if (handle->alloc && handle->heap_type == heap_type) {
			phys_addr_t base = heap_type == NVMAP_HEAP_IOVMM ? 0 :
					   (handle->carveout->base);
			seq_printf(s,
				"%-18s %-18s %8llx %10zuK %8x %6u %6u %6u %6u %6u %6u %8p\n",
				"", "",
				(unsigned long long)base, K(handle->size),
				handle->userflags,
				atomic_read(&handle->ref),
				atomic_read(&ref->dupes),
				atomic_read(&ref->pin),
				atomic_read(&handle->kmap_count),
				atomic_read(&handle->umap_count),
				atomic_read(&handle->share_count),
				handle);
		}
	}
	nvmap_ref_unlock(client);
}

/* compute the total amount of handle physical memory that is mapped
 * into client's virtual address space. Remember that vmas list is
 * sorted in ascending order of handle offsets.
 * NOTE: This function should be called while holding handle's lock mutex.
 */
static void nvmap_get_client_handle_mss(struct nvmap_client *client,
				struct nvmap_handle *handle, u64 *total)
{
	struct nvmap_vma_list *vma_list = NULL;
	struct vm_area_struct *vma = NULL;
	u64 end_offset = 0, vma_start_offset, vma_size;
	int64_t overlap_size;

	*total = 0;
	list_for_each_entry(vma_list, &handle->vmas, list) {

		if (client->task->pid == vma_list->pid) {
			vma = vma_list->vma;
			vma_size = vma->vm_end - vma->vm_start;

			vma_start_offset = vma->vm_pgoff << PAGE_SHIFT;
			if (end_offset < vma_start_offset + vma_size) {
				*total += vma_size;

				overlap_size = end_offset - vma_start_offset;
				if (overlap_size > 0)
					*total -= overlap_size;
				end_offset = vma_start_offset + vma_size;
			}
		}
	}
}

static void maps_stringify(struct nvmap_client *client,
				struct seq_file *s, u32 heap_type)
{
	struct rb_node *n;
	struct nvmap_vma_list *vma_list = NULL;
	struct vm_area_struct *vma = NULL;
	u64 total_mapped_size, vma_size;

	nvmap_ref_lock(client);
	n = rb_first(&client->handle_refs);
	for (; n != NULL; n = rb_next(n)) {
		struct nvmap_handle_ref *ref =
			rb_entry(n, struct nvmap_handle_ref, node);
		struct nvmap_handle *handle = ref->handle;
		if (handle->alloc && handle->heap_type == heap_type) {
			phys_addr_t base = heap_type == NVMAP_HEAP_IOVMM ? 0 :
					   (handle->carveout->base);
			seq_printf(s,
				"%-18s %-18s %8llx %10zuK %8x %6u %16p "
				"%12s %12s ",
				"", "",
				(unsigned long long)base, K(handle->size),
				handle->userflags,
				atomic_read(&handle->share_count),
				handle, "", "");

			mutex_lock(&handle->lock);
			nvmap_get_client_handle_mss(client, handle,
							&total_mapped_size);
			seq_printf(s, "%6lluK\n", K(total_mapped_size));

			list_for_each_entry(vma_list, &handle->vmas, list) {

				if (vma_list->pid == client->task->pid) {
					vma = vma_list->vma;
					vma_size = vma->vm_end - vma->vm_start;
					seq_printf(s,
					  "%-18s %-18s %8s %11s %8s %6s %16s "
					  "%-12lx-%12lx %6lluK\n",
					  "", "", "", "", "", "", "",
					  vma->vm_start, vma->vm_end,
					  K(vma_size));
				}
			}
			mutex_unlock(&handle->lock);
		}
	}
	nvmap_ref_unlock(client);
}

static void nvmap_get_client_mss(struct nvmap_client *client,
				 u64 *total, u32 heap_type)
{
	struct rb_node *n;

	*total = 0;
	nvmap_ref_lock(client);
	n = rb_first(&client->handle_refs);
	for (; n != NULL; n = rb_next(n)) {
		struct nvmap_handle_ref *ref =
			rb_entry(n, struct nvmap_handle_ref, node);
		struct nvmap_handle *handle = ref->handle;
		if (handle->alloc && handle->heap_type == heap_type)
			*total += handle->size /
				  atomic_read(&handle->share_count);
	}
	nvmap_ref_unlock(client);
}

static void nvmap_get_total_mss(u64 *pss, u64 *non_pss,
				      u64 *total, u32 heap_type)
{
	int i;
	struct rb_node *n;
	struct nvmap_device *dev = nvmap_dev;

	*total = 0;
	if (pss)
		*pss = 0;
	if (non_pss)
		*non_pss = 0;
	if (!dev)
		return;
	spin_lock(&dev->handle_lock);
	n = rb_first(&dev->handles);
	for (; n != NULL; n = rb_next(n)) {
		struct nvmap_handle *h =
			rb_entry(n, struct nvmap_handle, node);

		if (!h || !h->alloc || h->heap_type != heap_type)
			continue;
		if (!non_pss) {
			*total += h->size;
			continue;
		}

		for (i = 0; i < h->size >> PAGE_SHIFT; i++) {
			struct page *page = nvmap_to_page(h->pgalloc.pages[i]);
			int mapcount = page_mapcount(page);
			if (!mapcount)
				*non_pss += PAGE_SIZE;
			*total += PAGE_SIZE;
		}
	}
	if (pss && non_pss)
		*pss = *total - *non_pss;
	spin_unlock(&dev->handle_lock);
}

static int nvmap_debug_allocations_show(struct seq_file *s, void *unused)
{
	u64 total;
	struct nvmap_client *client;
	u32 heap_type = (u32)(uintptr_t)s->private;

	mutex_lock(&nvmap_dev->clients_lock);
	seq_printf(s, "%-18s %18s %8s %11s\n",
		"CLIENT", "PROCESS", "PID", "SIZE");
	seq_printf(s, "%-18s %18s %8s %11s %8s %6s %6s %6s %6s %6s %6s %8s\n",
			"", "", "BASE", "SIZE", "FLAGS", "REFS",
			"DUPES", "PINS", "KMAPS", "UMAPS", "SHARE", "UID");
	list_for_each_entry(client, &nvmap_dev->clients, list) {
		u64 client_total;
		client_stringify(client, s);
		nvmap_get_client_mss(client, &client_total, heap_type);
		seq_printf(s, " %10lluK\n", K(client_total));
		allocations_stringify(client, s, heap_type);
		seq_printf(s, "\n");
	}
	mutex_unlock(&nvmap_dev->clients_lock);
	nvmap_get_total_mss(NULL, NULL, &total, heap_type);
	seq_printf(s, "%-18s %-18s %8s %10lluK\n", "total", "", "", K(total));
	return 0;
}

DEBUGFS_OPEN_FOPS(allocations);

static int nvmap_debug_maps_show(struct seq_file *s, void *unused)
{
	u64 total;
	struct nvmap_client *client;
	u32 heap_type = (u32)(uintptr_t)s->private;

	mutex_lock(&nvmap_dev->clients_lock);
	seq_printf(s, "%-18s %18s %8s %11s\n",
		"CLIENT", "PROCESS", "PID", "SIZE");
	seq_printf(s, "%-18s %18s %8s %11s %8s %6s %9s %21s %18s\n",
		"", "", "BASE", "SIZE", "FLAGS", "SHARE", "UID",
		"MAPS", "MAPSIZE");

	list_for_each_entry(client, &nvmap_dev->clients, list) {
		u64 client_total;
		client_stringify(client, s);
		nvmap_get_client_mss(client, &client_total, heap_type);
		seq_printf(s, " %10lluK\n", K(client_total));
		maps_stringify(client, s, heap_type);
		seq_printf(s, "\n");
	}
	mutex_unlock(&nvmap_dev->clients_lock);

	nvmap_get_total_mss(NULL, NULL, &total, heap_type);
	seq_printf(s, "%-18s %-18s %8s %10lluK\n", "total", "", "", K(total));
	return 0;
}

DEBUGFS_OPEN_FOPS(maps);

static int nvmap_debug_clients_show(struct seq_file *s, void *unused)
{
	u64 total;
	struct nvmap_client *client;
	ulong heap_type = (ulong)s->private;

	mutex_lock(&nvmap_dev->clients_lock);
	seq_printf(s, "%-18s %18s %8s %11s\n",
		"CLIENT", "PROCESS", "PID", "SIZE");
	list_for_each_entry(client, &nvmap_dev->clients, list) {
		u64 client_total;
		client_stringify(client, s);
		nvmap_get_client_mss(client, &client_total, heap_type);
		seq_printf(s, " %10lluK\n", K(client_total));
	}
	mutex_unlock(&nvmap_dev->clients_lock);
	nvmap_get_total_mss(NULL, NULL, &total, heap_type);
	seq_printf(s, "%-18s %18s %8s %10lluK\n", "total", "", "", K(total));
	return 0;
}

DEBUGFS_OPEN_FOPS(clients);

#define PRINT_MEM_STATS_NOTE(x) \
do { \
	seq_printf(s, "Note: total memory is precise account of pages " \
		"allocated by NvMap.\nIt doesn't match with all clients " \
		"\"%s\" accumulated as shared memory \nis accounted in " \
		"full in each clients \"%s\" that shared memory.\n", #x, #x); \
} while (0)

static int nvmap_debug_lru_allocations_show(struct seq_file *s, void *unused)
{
	struct nvmap_handle *h;
	int total_handles = 0, migratable_handles = 0;
	size_t total_size = 0, migratable_size = 0;

	seq_printf(s, "%-18s %18s %8s %11s %8s %6s %6s %6s %6s %6s %8s\n",
			"", "", "", "", "", "",
			"", "PINS", "KMAPS", "UMAPS", "UID");
	spin_lock(&nvmap_dev->lru_lock);
	list_for_each_entry(h, &nvmap_dev->lru_handles, lru) {
		total_handles++;
		total_size += h->size;
		if (!atomic_read(&h->pin) && !atomic_read(&h->kmap_count)) {
			migratable_handles++;
			migratable_size += h->size;
		}
		seq_printf(s, "%-18s %18s %8s %10zuK %8s %6s %6s %6u %6u "
			"%6u %8p\n", "", "", "", K(h->size), "", "",
			"", atomic_read(&h->pin),
			    atomic_read(&h->kmap_count),
			    atomic_read(&h->umap_count),
			    h);
	}
	seq_printf(s, "total_handles = %d, migratable_handles = %d,"
		"total_size=%zuK, migratable_size=%zuK\n",
		total_handles, migratable_handles,
		K(total_size), K(migratable_size));
	spin_unlock(&nvmap_dev->lru_lock);
	PRINT_MEM_STATS_NOTE(SIZE);
	return 0;
}

DEBUGFS_OPEN_FOPS(lru_allocations);

static void nvmap_iovmm_get_client_mss(struct nvmap_client *client, u64 *pss,
				   u64 *non_pss, u64 *total)
{
	int i;
	struct rb_node *n;

	*pss = *non_pss = *total = 0;
	nvmap_ref_lock(client);
	n = rb_first(&client->handle_refs);
	for (; n != NULL; n = rb_next(n)) {
		struct nvmap_handle_ref *ref =
			rb_entry(n, struct nvmap_handle_ref, node);
		struct nvmap_handle *h = ref->handle;

		if (!h || !h->alloc || !h->heap_pgalloc)
			continue;

		for (i = 0; i < h->size >> PAGE_SHIFT; i++) {
			struct page *page = nvmap_to_page(h->pgalloc.pages[i]);
			int mapcount = page_mapcount(page);
			if (!mapcount)
				*non_pss += PAGE_SIZE;
			*total += PAGE_SIZE;
		}
		*pss = *total - *non_pss;
	}
	nvmap_ref_unlock(client);
}

static int nvmap_debug_iovmm_procrank_show(struct seq_file *s, void *unused)
{
	u64 pss, non_pss, total;
	struct nvmap_client *client;
	struct nvmap_device *dev = s->private;
	u64 total_memory, total_pss, total_non_pss;

	mutex_lock(&dev->clients_lock);
	seq_printf(s, "%-18s %18s %8s %11s %11s %11s\n",
		"CLIENT", "PROCESS", "PID", "PSS", "NON-PSS", "TOTAL");
	list_for_each_entry(client, &dev->clients, list) {
		client_stringify(client, s);
		nvmap_iovmm_get_client_mss(client, &pss, &non_pss, &total);
		seq_printf(s, " %10lluK %10lluK %10lluK\n", K(pss),
			K(non_pss), K(total));
	}
	mutex_unlock(&dev->clients_lock);

	nvmap_get_total_mss(&total_pss, &total_non_pss, &total_memory, NVMAP_HEAP_IOVMM);
	seq_printf(s, "%-18s %18s %8s %10lluK %10lluK %10lluK\n",
		"total", "", "", K(total_pss),
		K(total_non_pss), K(total_memory));
	PRINT_MEM_STATS_NOTE(TOTAL);
	return 0;
}

DEBUGFS_OPEN_FOPS(iovmm_procrank);

ulong nvmap_iovmm_get_used_pages(void)
{
	u64 total;

	nvmap_get_total_mss(NULL, NULL, &total, NVMAP_HEAP_IOVMM);
	return total >> PAGE_SHIFT;
}

static int nvmap_stats_reset(void *data, u64 val)
{
	int i;

	if (val) {
		atomic64_set(&nvmap_stats.collect, 0);
		for (i = 0; i < NS_NUM; i++) {
			if (i == NS_TOTAL)
				continue;
			atomic64_set(&nvmap_stats.stats[i], 0);
		}
	}
	return 0;
}

static int nvmap_stats_get(void *data, u64 *val)
{
	atomic64_t *ptr = data;

	*val = atomic64_read(ptr);
	return 0;
}

static int nvmap_stats_set(void *data, u64 val)
{
	atomic64_t *ptr = data;

	atomic64_set(ptr, val);
	return 0;
}

static int page_zero_filled(void *ptr)
{
	unsigned int pos;
	unsigned long *page;

	page = (unsigned long *)ptr;

	for (pos = 0; pos != PAGE_SIZE / sizeof(*page); pos++) {
		if (page[pos])
			return 0;
	}

	return 1;
}

static size_t compress_bytes(struct page *page)
{
	void *addr;
	size_t clen = 0;
	static void *compress_workmem;
	static void *compress_buffer;

	if (!compress_workmem) {
		void *tmp = kzalloc(LZO1X_MEM_COMPRESS, GFP_KERNEL);
		if (cmpxchg(&compress_workmem, NULL, tmp))
			kfree(tmp);
	}

	if (!compress_buffer) {
		ulong tmp;
		tmp = __get_free_pages(GFP_KERNEL | __GFP_ZERO, 1);
		if (cmpxchg(&compress_buffer, NULL, tmp))
			free_pages(tmp, 1);
	}

	if (compress_workmem && compress_buffer) {
		addr = kmap(page);
		lzo1x_1_compress(addr, PAGE_SIZE, compress_buffer, &clen,
					compress_workmem);
		kunmap(page);
	}

	return clen;
}

static int nvmap_debug_compress_show(struct seq_file *s, void *unused)
{
	int i;
	void *addr;
	size_t clen;
	struct rb_node *n;
	size_t min_clen = PAGE_SIZE;
	size_t max_clen = 0;
	bool is_zero_page;
	u32 all_clen = 0;
	u32 num_pages = 0;
	u32 num_non_zero_pages = 0;
	u32 zero_filled_pages = 0;
	u64 total_compressed_mem = 0;
	u64 total_uncompressed_mem = 0;
	u64 total_compressed_non_zero_mem = 0;
	u64 total_uncompressed_non_zero_mem = 0;
	struct nvmap_handle *h_put = NULL;
	struct nvmap_device *dev = nvmap_dev;

	if (!dev)
		return 0;

	spin_lock(&dev->handle_lock);
	n = rb_first(&dev->handles);
	while (n != NULL) {
		struct nvmap_handle *h =
			rb_entry(n, struct nvmap_handle, node);

		if (!h || !h->alloc || !h->heap_pgalloc)
			continue;

		nvmap_handle_get(h);
		spin_unlock(&dev->handle_lock);
		if (h_put)
			nvmap_handle_put(h_put);
		for (i = 0; i < h->size >> PAGE_SHIFT; i++) {
			struct page *page = nvmap_to_page(h->pgalloc.pages[i]);

			addr = kmap(page);
			is_zero_page = page_zero_filled(addr);
			kunmap(page);
			clen = compress_bytes(page);
			all_clen += clen;
			if (!is_zero_page && clen < min_clen)
				min_clen = clen;
			if (!is_zero_page && clen > max_clen)
				max_clen = clen;
			total_uncompressed_mem += PAGE_SIZE;
			total_compressed_mem += clen;
			if (!is_zero_page) {
				total_uncompressed_non_zero_mem += PAGE_SIZE;
				total_compressed_non_zero_mem += clen;
				num_non_zero_pages++;
			}
			zero_filled_pages += is_zero_page ? 1 : 0;
			num_pages++;
		}
		spin_lock(&dev->handle_lock);
		n = rb_next(n);
		h_put = h;
		if (!n)
			nvmap_handle_put(h_put);
	}
	seq_puts(s, "compression algo: \tlzo\n");
	seq_printf(s, "uncompressed bytes: \t%lld\n", total_uncompressed_mem);
	seq_printf(s, "compressed bytes: \t%lld\n", total_compressed_mem);
	seq_printf(s, "compression %%: \t%d\n",
		(u32)((total_uncompressed_mem - total_compressed_mem) >>
		      PAGE_SHIFT) * 100 / num_pages);
	seq_printf(s, "uncompressed non-zero bytes: \t%lld\n",
		total_uncompressed_non_zero_mem);
	seq_printf(s, "compressed non-zero bytes: \t%lld\n",
		total_compressed_non_zero_mem);
	seq_printf(s, "compression non-zero bytes %%: \t%d\n",
		(u32)((total_uncompressed_non_zero_mem -
		       total_compressed_non_zero_mem) >>
		      PAGE_SHIFT) * 100 / num_non_zero_pages);
	seq_printf(s, "zero filled page bytes: \t%d\n",
		zero_filled_pages << PAGE_SHIFT);
	seq_printf(s, "zero filled bytes %%: \t%d\n",
		zero_filled_pages * 100 / num_pages);
	seq_printf(s, "min compress bytes: \t%zu\n", min_clen);
	seq_printf(s, "max compress bytes: \t%zu\n", max_clen);
	seq_printf(s, "average compress bytes: \t%d\n", all_clen / num_pages);
	spin_unlock(&dev->handle_lock);
	return 0;
}

DEBUGFS_OPEN_FOPS(compress);
DEFINE_SIMPLE_ATTRIBUTE(reset_stats_fops, NULL, nvmap_stats_reset, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(stats_fops, nvmap_stats_get, nvmap_stats_set, "%llu\n");

static void nvmap_stats_init(struct dentry *nvmap_debug_root)
{
	struct dentry *stats_root;

#define CREATE_DF(x, y) \
	debugfs_create_file(#x, S_IRUGO, stats_root, &y, &stats_fops);

	stats_root = debugfs_create_dir("stats", nvmap_debug_root);
	if (!IS_ERR_OR_NULL(stats_root)) {
		CREATE_DF(alloc, nvmap_stats.stats[NS_ALLOC]);
		CREATE_DF(release, nvmap_stats.stats[NS_RELEASE]);
		CREATE_DF(ualloc, nvmap_stats.stats[NS_UALLOC]);
		CREATE_DF(urelease, nvmap_stats.stats[NS_URELEASE]);
		CREATE_DF(kalloc, nvmap_stats.stats[NS_KALLOC]);
		CREATE_DF(krelease, nvmap_stats.stats[NS_KRELEASE]);
		CREATE_DF(cflush_rq, nvmap_stats.stats[NS_CFLUSH_RQ]);
		CREATE_DF(cflush_done, nvmap_stats.stats[NS_CFLUSH_DONE]);
		CREATE_DF(ucflush_rq, nvmap_stats.stats[NS_UCFLUSH_RQ]);
		CREATE_DF(ucflush_done, nvmap_stats.stats[NS_UCFLUSH_DONE]);
		CREATE_DF(kcflush_rq, nvmap_stats.stats[NS_KCFLUSH_RQ]);
		CREATE_DF(kcflush_done, nvmap_stats.stats[NS_KCFLUSH_DONE]);
		CREATE_DF(total_memory, nvmap_stats.stats[NS_TOTAL]);

		debugfs_create_file("collect", S_IRUGO | S_IWUSR,
			stats_root, &nvmap_stats.collect, &stats_fops);
		debugfs_create_file("reset", S_IWUSR,
			stats_root, NULL, &reset_stats_fops);
	}

#undef CREATE_DF
	debugfs_create_file("compression", S_IRUGO, stats_root,
		NULL, &debug_compress_fops);
}

void nvmap_stats_inc(enum nvmap_stats_t stat, size_t size)
{
	if (atomic64_read(&nvmap_stats.collect) || stat == NS_TOTAL)
		atomic64_add(size, &nvmap_stats.stats[stat]);
}

void nvmap_stats_dec(enum nvmap_stats_t stat, size_t size)
{
	if (atomic64_read(&nvmap_stats.collect) || stat == NS_TOTAL)
		atomic64_sub(size, &nvmap_stats.stats[stat]);
}

u64 nvmap_stats_read(enum nvmap_stats_t stat)
{
	return atomic64_read(&nvmap_stats.stats[stat]);
}

int nvmap_probe(struct platform_device *pdev)
{
	struct nvmap_platform_data *plat;
	struct nvmap_device *dev;
	struct dentry *nvmap_debug_root;
	unsigned int i;
	int e;

	nvmap_init(pdev);

	plat = pdev->dev.platform_data;
	if (!plat) {
		dev_err(&pdev->dev, "no platform data?\n");
		return -ENODEV;
	}

	/*
	 * The DMA mapping API uses these parameters to decide how to map the
	 * passed buffers. If the maximum physical segment size is set to
	 * smaller than the size of the buffer, then the buffer will be mapped
	 * as separate IO virtual address ranges.
	 */
	pdev->dev.dma_parms = &nvmap_dma_parameters;

	if (WARN_ON(nvmap_dev != NULL)) {
		dev_err(&pdev->dev, "only one nvmap device may be present\n");
		return -ENODEV;
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "out of memory for device\n");
		return -ENOMEM;
	}

	nvmap_dev = dev;

	dev->dev_user.minor = MISC_DYNAMIC_MINOR;
	dev->dev_user.name = "nvmap";
	dev->dev_user.fops = &nvmap_user_fops;
	dev->dev_user.parent = &pdev->dev;

	dev->handles = RB_ROOT;

#ifdef CONFIG_NVMAP_PAGE_POOLS
	e = nvmap_page_pool_init(dev);
	if (e)
		goto fail;
#endif

	spin_lock_init(&dev->handle_lock);
	INIT_LIST_HEAD(&dev->clients);
	mutex_init(&dev->clients_lock);
	INIT_LIST_HEAD(&dev->lru_handles);
	spin_lock_init(&dev->lru_lock);

	e = misc_register(&dev->dev_user);
	if (e) {
		dev_err(&pdev->dev, "unable to register miscdevice %s\n",
			dev->dev_user.name);
		goto fail;
	}

	dev->nr_carveouts = 0;
	dev->heaps = kzalloc(sizeof(struct nvmap_carveout_node) *
			     plat->nr_carveouts, GFP_KERNEL);
	if (!dev->heaps) {
		e = -ENOMEM;
		dev_err(&pdev->dev, "couldn't allocate carveout memory\n");
		goto fail;
	}

	nvmap_debug_root = debugfs_create_dir("nvmap", NULL);
	if (IS_ERR_OR_NULL(nvmap_debug_root))
		dev_err(&pdev->dev, "couldn't create debug files\n");

	debugfs_create_u32("max_handle_count", S_IRUGO,
			nvmap_debug_root, &nvmap_max_handle_count);

	for (i = 0; i < plat->nr_carveouts; i++) {
		struct nvmap_carveout_node *node = &dev->heaps[dev->nr_carveouts];
		const struct nvmap_platform_carveout *co = &plat->carveouts[i];
		node->base = round_up(co->base, PAGE_SIZE);
		node->size = round_down(co->size -
					(node->base - co->base), PAGE_SIZE);
		if (!co->size)
			continue;

		node->carveout = nvmap_heap_create(
				dev->dev_user.this_device, co,
				node->base, node->size, node);

		if (!node->carveout) {
			e = -ENOMEM;
			dev_err(&pdev->dev, "couldn't create %s\n", co->name);
			goto fail_heaps;
		}
		node->index = dev->nr_carveouts;
		dev->nr_carveouts++;
		node->heap_bit = co->usage_mask;

		if (!IS_ERR_OR_NULL(nvmap_debug_root)) {
			struct dentry *heap_root =
				debugfs_create_dir(co->name, nvmap_debug_root);
			if (!IS_ERR_OR_NULL(heap_root)) {
				debugfs_create_file("clients", S_IRUGO,
					heap_root,
					(void *)(uintptr_t)node->heap_bit,
					&debug_clients_fops);
				debugfs_create_file("allocations", S_IRUGO,
					heap_root,
					(void *)(uintptr_t)node->heap_bit,
					&debug_allocations_fops);
				debugfs_create_file("maps", S_IRUGO,
					heap_root,
					(void *)(uintptr_t)node->heap_bit,
					&debug_maps_fops);
				nvmap_heap_debugfs_init(heap_root,
							node->carveout);
			}
		}
	}
	if (!IS_ERR_OR_NULL(nvmap_debug_root)) {
		struct dentry *iovmm_root =
			debugfs_create_dir("iovmm", nvmap_debug_root);
		if (!IS_ERR_OR_NULL(iovmm_root)) {
			debugfs_create_file("clients", S_IRUGO, iovmm_root,
				(void *)(uintptr_t)NVMAP_HEAP_IOVMM,
				&debug_clients_fops);
			debugfs_create_file("allocations", S_IRUGO, iovmm_root,
				(void *)(uintptr_t)NVMAP_HEAP_IOVMM,
				&debug_allocations_fops);
			debugfs_create_file("maps", S_IRUGO, iovmm_root,
				(void *)(uintptr_t)NVMAP_HEAP_IOVMM,
				&debug_maps_fops);
			debugfs_create_file("procrank", S_IRUGO, iovmm_root,
				dev, &debug_iovmm_procrank_fops);
		}
#ifdef CONFIG_NVMAP_PAGE_POOLS
		nvmap_page_pool_debugfs_init(nvmap_debug_root);
#endif
#ifdef CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS
		debugfs_create_size_t("cache_maint_inner_threshold",
				      S_IRUSR | S_IWUSR,
				      nvmap_debug_root,
				      &cache_maint_inner_threshold);

		/* cortex-a9 */
		if ((read_cpuid_id() >> 4 & 0xfff) == 0xc09)
			cache_maint_inner_threshold = SZ_32K;
		pr_info("nvmap:inner cache maint threshold=%zd",
			cache_maint_inner_threshold);
#endif
#ifdef CONFIG_NVMAP_OUTER_CACHE_MAINT_BY_SET_WAYS
		debugfs_create_size_t("cache_maint_outer_threshold",
				      S_IRUSR | S_IWUSR,
				      nvmap_debug_root,
				      &cache_maint_outer_threshold);
		pr_info("nvmap:outer cache maint threshold=%zd",
			cache_maint_outer_threshold);
#endif
	}

	nvmap_stats_init(nvmap_debug_root);
	platform_set_drvdata(pdev, dev);

	nvmap_dmabuf_debugfs_init(nvmap_debug_root);
	e = nvmap_dmabuf_stash_init();
	if (e)
		goto fail_heaps;

	return 0;
fail_heaps:
	for (i = 0; i < dev->nr_carveouts; i++) {
		struct nvmap_carveout_node *node = &dev->heaps[i];
		nvmap_heap_destroy(node->carveout);
	}
fail:
#ifdef CONFIG_NVMAP_PAGE_POOLS
	nvmap_page_pool_fini(nvmap_dev);
#endif
	kfree(dev->heaps);
	if (dev->dev_user.minor != MISC_DYNAMIC_MINOR)
		misc_deregister(&dev->dev_user);
	kfree(dev);
	nvmap_dev = NULL;
	return e;
}

int nvmap_remove(struct platform_device *pdev)
{
	struct nvmap_device *dev = platform_get_drvdata(pdev);
	struct rb_node *n;
	struct nvmap_handle *h;
	int i;

	misc_deregister(&dev->dev_user);

	while ((n = rb_first(&dev->handles))) {
		h = rb_entry(n, struct nvmap_handle, node);
		rb_erase(&h->node, &dev->handles);
		kfree(h);
	}

	for (i = 0; i < dev->nr_carveouts; i++) {
		struct nvmap_carveout_node *node = &dev->heaps[i];
		nvmap_heap_destroy(node->carveout);
	}
	kfree(dev->heaps);

	kfree(dev);
	nvmap_dev = NULL;
	return 0;
}
