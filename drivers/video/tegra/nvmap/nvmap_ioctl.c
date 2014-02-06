/*
 * drivers/video/tegra/nvmap/nvmap_ioctl.c
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

#define pr_fmt(fmt)	"nvmap: %s() " fmt, __func__

#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/nvmap.h>
#include <linux/vmalloc.h>

#include <asm/memory.h>

#include <trace/events/nvmap.h>

#include "nvmap_ioctl.h"
#include "nvmap_priv.h"

#include <linux/list.h>

static ssize_t rw_handle(struct nvmap_client *client, struct nvmap_handle *h,
			 int is_read, unsigned long h_offs,
			 unsigned long sys_addr, unsigned long h_stride,
			 unsigned long sys_stride, unsigned long elem_size,
			 unsigned long count);

static ulong __attribute__((unused)) fd_to_handle_id(int handle)
{
	ulong id;

	id = nvmap_get_id_from_dmabuf_fd(NULL, (int)handle);
	if (!IS_ERR_VALUE(id))
		return id;
	return 0;
}

#ifdef CONFIG_COMPAT
ulong unmarshal_user_handle(__u32 handle)
{
	ulong h = (handle | PAGE_OFFSET);

#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	return fd_to_handle_id((int)handle);
#endif
	return h;
}

ulong unmarshal_user_handle_array(__u32 handles, __u32 idx)
{
	__u32 *ptr = (__u32 *)((uintptr_t)handles);
	return unmarshal_user_handle(ptr[idx]);
}

ulong unmarshal_user_handle_array_single(__u32 handles)
{
	return unmarshal_user_handle((ulong)handles);
}

__u32 marshal_kernel_handle(ulong handle)
{
	return (__u32)handle;
}

ulong unmarshal_user_id(u32 id)
{
	return unmarshal_user_handle(id);
}

/*
 * marshal_id/unmarshal_id are for get_id/handle_from_id.
 * These are added to support using Fd's for handle.
 */
__u32 marshal_id(ulong id)
{
	return (__u32)(id >> 2);
}

ulong unmarshal_id(__u32 id)
{
	ulong h = ((id << 2) | PAGE_OFFSET);

	return h;
}

ulong *unmarshal_user_pointer(__u32 ptr)
{
	return (ulong *)((uintptr_t)ptr);
}

__u32 marshal_kernel_vaddr(ulong address)
{
	return (__u32)address;
}

#else
#define NVMAP_XOR_HASH_MASK 0xFFFFFFFC
ulong unmarshal_user_handle(struct nvmap_handle *handle)
{
	if ((ulong)handle == 0)
		return (ulong)handle;

#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	return fd_to_handle_id((int)handle);
#endif
#ifdef CONFIG_NVMAP_HANDLE_MARSHAL
	return (ulong)handle ^ NVMAP_XOR_HASH_MASK;
#else
	return (ulong)handle;
#endif
}

ulong unmarshal_user_handle_array(struct nvmap_handle **handles, __u32 idx)
{
	return unmarshal_user_handle(handles[idx]);
}

ulong unmarshal_user_handle_array_single(struct nvmap_handle **handles)
{
	return unmarshal_user_handle((struct nvmap_handle *)handles);
}

struct nvmap_handle *marshal_kernel_handle(ulong handle)
{
	if (handle == 0)
		return (struct nvmap_handle *)handle;

#ifdef CONFIG_NVMAP_HANDLE_MARSHAL
	return (struct nvmap_handle *)(handle ^ NVMAP_XOR_HASH_MASK);
#else
	return (struct nvmap_handle *)handle;
#endif
}

ulong unmarshal_user_id(ulong id)
{
	return unmarshal_user_handle((struct nvmap_handle *)id);
}

ulong marshal_id(ulong id)
{
	return id;
}

ulong unmarshal_id(ulong id)
{
	return id;
}

ulong *unmarshal_user_pointer(unsigned long __user *ptr)
{
	return ptr;
}

ulong marshal_kernel_vaddr(ulong address)
{
	return address;
}
#endif

ulong __nvmap_ref_to_id(struct nvmap_handle_ref *ref)
{
	if (!virt_addr_valid(ref))
		return 0;
	return (unsigned long)ref->handle;
}

int nvmap_ioctl_pinop(struct file *filp, bool is_pin, void __user *arg)
{
	struct nvmap_pin_handle op;
	struct nvmap_handle *h;
	unsigned long on_stack[16];
	unsigned long *refs;
#ifdef CONFIG_COMPAT
	u32 __user *output;
#else
	unsigned long __user *output;
#endif
	unsigned int i;
	int err = 0;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!op.count)
		return -EINVAL;

	if (op.count > 1) {
		size_t bytes = op.count * sizeof(*refs); /* kcalloc below will catch overflow. */

		if (op.count > ARRAY_SIZE(on_stack))
			refs = kcalloc(op.count, sizeof(*refs), GFP_KERNEL);
		else
			refs = on_stack;

		if (!refs)
			return -ENOMEM;

		if (!access_ok(VERIFY_READ, op.handles, bytes)) {
			err = -EFAULT;
			goto out;
		}

		for (i = 0; i < op.count; i++) {
#ifdef CONFIG_COMPAT
			u32 handle;
			u32 *handles = (u32 *)op.handles;
#else
			struct nvmap_handle *handle;
			struct nvmap_handle **handles = op.handles;
#endif
			if (__get_user(handle, &handles[i])) {
				err = -EFAULT;
				goto out;
			}
			refs[i] = unmarshal_user_handle(handle);
			if (!refs[i]) {
				err = -EINVAL;
				goto out;
			}
		}
	} else {
		refs = on_stack;
		on_stack[0] = unmarshal_user_handle_array_single(op.handles);
		if (!on_stack[0]) {
			err = -EINVAL;
			goto out;
		}
	}

	trace_nvmap_ioctl_pinop(filp->private_data, is_pin, op.count, refs);
	if (is_pin)
		err = nvmap_pin_ids(filp->private_data, op.count, refs);
	else
		nvmap_unpin_ids(filp->private_data, op.count, refs);

	/* skip the output stage on unpin */
	if (err || !is_pin)
		goto out;

	/* it is guaranteed that if nvmap_pin_ids returns 0 that
	 * all of the handle_ref objects are valid, so dereferencing
	 * directly here is safe */
	if (op.count > 1)
		output = (typeof(output))op.addr;
	else {
		struct nvmap_pin_handle __user *tmp = arg;
		output = (typeof(output))&(tmp->addr);
	}

	if (!output)
		goto out;

	for (i = 0; i < op.count && !err; i++) {
		unsigned long addr;

		h = (struct nvmap_handle *)refs[i];
		if (h->heap_pgalloc && h->pgalloc.contig)
			addr = page_to_phys(h->pgalloc.pages[0]);
		else if (h->heap_pgalloc)
			addr = sg_dma_address(
				((struct sg_table *)h->attachment->priv)->sgl);
		else
			addr = h->carveout->base;

		err = put_user(marshal_kernel_vaddr(addr), &output[i]);
	}

	if (err)
		nvmap_unpin_ids(filp->private_data, op.count, refs);

out:
	if (refs != on_stack)
		kfree(refs);

	return err;
}

int nvmap_ioctl_getid(struct file *filp, void __user *arg)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_create_handle op;
	struct nvmap_handle *h = NULL;
	ulong handle;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_handle(op.handle);
	if (!handle)
		return -EINVAL;

	h = nvmap_get_handle_id(client, handle);

	if (!h)
		return -EPERM;

	op.id = marshal_id((ulong)h);
	if (client == h->owner)
		h->global = true;

	nvmap_handle_put(h);

	return copy_to_user(arg, &op, sizeof(op)) ? -EFAULT : 0;
}

static int nvmap_share_release(struct inode *inode, struct file *file)
{
	struct nvmap_handle *h = file->private_data;

	nvmap_handle_put(h);
	return 0;
}

static int nvmap_share_mmap(struct file *file, struct vm_area_struct *vma)
{
	/* unsupported operation */
	WARN(1, "mmap is not supported on fd, which shares nvmap handle");
	return -EPERM;
}

const struct file_operations nvmap_fd_fops = {
	.owner		= THIS_MODULE,
	.release	= nvmap_share_release,
	.mmap		= nvmap_share_mmap,
};

int nvmap_ioctl_getfd(struct file *filp, void __user *arg)
{
	ulong handle;
	struct nvmap_create_handle op;
	struct nvmap_client *client = filp->private_data;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_handle(op.handle);
	if (!handle)
		return -EINVAL;

	op.fd = nvmap_get_dmabuf_fd(client, handle);
	if (op.fd < 0)
		return op.fd;

	if (copy_to_user(arg, &op, sizeof(op))) {
		sys_close(op.fd);
		return -EFAULT;
	}
	return 0;
}

int nvmap_ioctl_alloc(struct file *filp, void __user *arg)
{
	struct nvmap_alloc_handle op;
	struct nvmap_client *client = filp->private_data;
	ulong handle;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_handle(op.handle);
	if (!handle)
		return -EINVAL;

	if (op.align & (op.align - 1))
		return -EINVAL;

	/* user-space handles are aligned to page boundaries, to prevent
	 * data leakage. */
	op.align = max_t(size_t, op.align, PAGE_SIZE);
#if defined(CONFIG_NVMAP_FORCE_ZEROED_USER_PAGES)
	op.flags |= NVMAP_HANDLE_ZEROED_PAGES;
#endif

	return nvmap_alloc_handle_id(client, handle, op.heap_mask,
				     op.align,
				     0, /* no kind */
				     op.flags & (~NVMAP_HANDLE_KIND_SPECIFIED));
}

int nvmap_ioctl_alloc_kind(struct file *filp, void __user *arg)
{
	struct nvmap_alloc_kind_handle op;
	struct nvmap_client *client = filp->private_data;
	ulong handle;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_handle(op.handle);
	if (!handle)
		return -EINVAL;

	if (op.align & (op.align - 1))
		return -EINVAL;

	/* user-space handles are aligned to page boundaries, to prevent
	 * data leakage. */
	op.align = max_t(size_t, op.align, PAGE_SIZE);
#if defined(CONFIG_NVMAP_FORCE_ZEROED_USER_PAGES)
	op.flags |= NVMAP_HANDLE_ZEROED_PAGES;
#endif

	return nvmap_alloc_handle_id(client, handle, op.heap_mask,
				     op.align,
				     op.kind,
				     op.flags);
}

int nvmap_create_fd(struct nvmap_handle *h)
{
	int fd;

	fd = __nvmap_dmabuf_fd(h->dmabuf, O_CLOEXEC);
	BUG_ON(fd == 0);
	if (fd < 0) {
		pr_err("Out of file descriptors");
		return fd;
	}
	/* __nvmap_dmabuf_fd() associates fd with dma_buf->file *.
	 * fd close drops one ref count on dmabuf->file *.
	 * to balance ref count, ref count dma_buf.
	 */
	get_dma_buf(h->dmabuf);
	return fd;
}

int nvmap_ioctl_create(struct file *filp, unsigned int cmd, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_handle_ref *ref = NULL;
	struct nvmap_client *client = filp->private_data;
	int err = 0;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!client)
		return -ENODEV;

	if (cmd == NVMAP_IOC_CREATE) {
		ref = nvmap_create_handle(client, PAGE_ALIGN(op.size));
		if (!IS_ERR(ref))
			ref->handle->orig_size = op.size;
	} else if (cmd == NVMAP_IOC_FROM_ID) {
		ref = nvmap_duplicate_handle_id(client, unmarshal_id(op.id), 0);
	} else if (cmd == NVMAP_IOC_FROM_FD) {
		ref = nvmap_create_handle_from_fd(client, op.fd);
	} else {
		return -EINVAL;
	}

	if (IS_ERR(ref))
		return PTR_ERR(ref);

#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	op.handle = (typeof(op.handle))nvmap_create_fd(ref->handle);
	if (IS_ERR(op.handle))
		err = (int)op.handle;
#else
	op.handle = marshal_kernel_handle(__nvmap_ref_to_id(ref));
#endif

	if (copy_to_user(arg, &op, sizeof(op))) {
		err = -EFAULT;
		nvmap_free_handle_id(client, __nvmap_ref_to_id(ref));
	}

#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	if (err && (int)op.handle > 0)
		sys_close((int)op.handle);
#endif
	return err;
}

int nvmap_map_into_caller_ptr(struct file *filp, void __user *arg)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_map_caller op;
	struct nvmap_vma_priv *vpriv;
	struct vm_area_struct *vma;
	struct nvmap_handle *h = NULL;
	int err = 0;
	ulong handle;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_handle(op.handle);

	if (!handle)
		return -EINVAL;

	h = nvmap_get_handle_id(client, handle);

	if (!h)
		return -EPERM;

	if(!h->alloc) {
		nvmap_handle_put(h);
		return -EFAULT;
	}

	trace_nvmap_map_into_caller_ptr(client, h, op.offset,
					op.length, op.flags);
	down_read(&current->mm->mmap_sem);

	vma = find_vma(current->mm, op.addr);
	if (!vma || !vma->vm_private_data) {
		err = -ENOMEM;
		goto out;
	}

	if (op.offset & ~PAGE_MASK) {
		err = -EFAULT;
		goto out;
	}

	if (op.offset >= h->size || op.length > h->size - op.offset) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	vpriv = vma->vm_private_data;
	BUG_ON(!vpriv);

	/* the VMA must exactly match the requested mapping operation, and the
	 * VMA that is targetted must have been created by this driver
	 */
	if ((vma->vm_start != op.addr) || !is_nvmap_vma(vma) ||
	    (vma->vm_end-vma->vm_start != op.length)) {
		err = -EPERM;
		goto out;
	}

	/* verify that each mmap() system call creates a unique VMA */

	if (vpriv->handle && (h == vpriv->handle)) {
		goto out;
	} else if (vpriv->handle) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	if (!h->heap_pgalloc && (h->carveout->base & ~PAGE_MASK)) {
		err = -EFAULT;
		goto out;
	}

	vma->vm_flags |= (h->heap_pgalloc ? 0 : VM_PFNMAP);
	vpriv->handle = h;
	vpriv->offs = op.offset;
	vma->vm_page_prot = nvmap_pgprot(h, vma->vm_page_prot);

out:
	up_read(&current->mm->mmap_sem);

	if (err)
		nvmap_handle_put(h);
	return err;
}

int nvmap_ioctl_get_param(struct file *filp, void __user* arg)
{
	struct nvmap_handle_param op;
	struct nvmap_client *client = filp->private_data;
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *h;
	u64 result;
	int err = 0;
	ulong handle;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_handle(op.handle);
	if (!handle)
		return -EINVAL;

	h = nvmap_get_handle_id(client, handle);
	if (!h)
		return -EINVAL;

	nvmap_ref_lock(client);
	ref = __nvmap_validate_id_locked(client, handle);
	if (IS_ERR_OR_NULL(ref)) {
		err = ref ? PTR_ERR(ref) : -EINVAL;
		goto ref_fail;
	}

	err = nvmap_get_handle_param(client, ref, op.param, &result);
	op.result = (long unsigned int)result;

	if (!err && copy_to_user(arg, &op, sizeof(op)))
		err = -EFAULT;

ref_fail:
	nvmap_ref_unlock(client);
	nvmap_handle_put(h);
	return err;
}

int nvmap_ioctl_rw_handle(struct file *filp, int is_read, void __user* arg)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_rw_handle __user *uarg = arg;
	struct nvmap_rw_handle op;
	struct nvmap_handle *h;
	ssize_t copied;
	int err = 0;
	ulong handle;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_handle(op.handle);
	if (!handle || !op.addr || !op.count || !op.elem_size)
		return -EINVAL;

	h = nvmap_get_handle_id(client, handle);
	if (!h)
		return -EPERM;

	trace_nvmap_ioctl_rw_handle(client, h, is_read, op.offset,
				    op.addr, op.hmem_stride,
				    op.user_stride, op.elem_size, op.count);
	copied = rw_handle(client, h, is_read, op.offset,
			   (unsigned long)op.addr, op.hmem_stride,
			   op.user_stride, op.elem_size, op.count);

	if (copied < 0) {
		err = copied;
		copied = 0;
	} else if (copied < (op.count * op.elem_size))
		err = -EINTR;

	__put_user(copied, &uarg->count);

	nvmap_handle_put(h);

	return err;
}

int nvmap_ioctl_cache_maint(struct file *filp, void __user *arg)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_cache_op op;
	struct vm_area_struct *vma;
	struct nvmap_vma_priv *vpriv;
	unsigned long start;
	unsigned long end;
	int err = 0;
	ulong handle;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = unmarshal_user_handle(op.handle);
	if (!handle || !op.addr || op.op < NVMAP_CACHE_OP_WB ||
	    op.op > NVMAP_CACHE_OP_WB_INV)
		return -EINVAL;

	down_read(&current->mm->mmap_sem);

	vma = find_vma(current->active_mm, (unsigned long)op.addr);
	if (!vma || !is_nvmap_vma(vma) ||
	    (ulong)op.addr < vma->vm_start ||
	    (ulong)op.addr >= vma->vm_end ||
	    op.len > vma->vm_end - (ulong)op.addr) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	vpriv = (struct nvmap_vma_priv *)vma->vm_private_data;

	if ((unsigned long)vpriv->handle != handle) {
		err = -EFAULT;
		goto out;
	}

	start = (unsigned long)op.addr - vma->vm_start +
		(vma->vm_pgoff << PAGE_SHIFT);
	end = start + op.len;

	err = __nvmap_cache_maint(client, vpriv->handle, start, end, op.op,
				  CACHE_MAINT_ALLOW_DEFERRED);
out:
	up_read(&current->mm->mmap_sem);
	return err;
}

int nvmap_ioctl_free(struct file *filp, unsigned long arg)
{
	struct nvmap_client *client = filp->private_data;

	if (!arg)
		return 0;

	nvmap_free_handle_user_id(client, arg);
#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	return sys_close(arg);
#endif
	return 0;
}

static void inner_cache_maint(unsigned int op, void *vaddr, size_t size)
{
	if (op == NVMAP_CACHE_OP_WB_INV)
		dmac_flush_range(vaddr, vaddr + size);
	else if (op == NVMAP_CACHE_OP_INV)
		dmac_map_area(vaddr, size, DMA_FROM_DEVICE);
	else
		dmac_map_area(vaddr, size, DMA_TO_DEVICE);
}

static void outer_cache_maint(unsigned int op, phys_addr_t paddr, size_t size)
{
	if (op == NVMAP_CACHE_OP_WB_INV)
		outer_flush_range(paddr, paddr + size);
	else if (op == NVMAP_CACHE_OP_INV)
		outer_inv_range(paddr, paddr + size);
	else
		outer_clean_range(paddr, paddr + size);
}

static void heap_page_cache_maint(
	struct nvmap_handle *h, unsigned long start, unsigned long end,
	unsigned int op, bool inner, bool outer, pte_t **pte,
	unsigned long kaddr, pgprot_t prot)
{
	struct page *page;
	phys_addr_t paddr;
	unsigned long next;
	unsigned long off;
	size_t size;

	while (start < end) {
		page = h->pgalloc.pages[start >> PAGE_SHIFT];
		next = min(((start + PAGE_SIZE) & PAGE_MASK), end);
		off = start & ~PAGE_MASK;
		size = next - start;
		paddr = page_to_phys(page) + off;

		if (inner) {
			void *vaddr = (void *)kaddr + off;
			BUG_ON(!pte);
			BUG_ON(!kaddr);
			set_pte_at(&init_mm, kaddr, *pte,
				pfn_pte(__phys_to_pfn(paddr), prot));
			nvmap_flush_tlb_kernel_page(kaddr);
			inner_cache_maint(op, vaddr, size);
		}

		if (outer)
			outer_cache_maint(op, paddr, size);
		start = next;
	}
}

#if defined(CONFIG_NVMAP_OUTER_CACHE_MAINT_BY_SET_WAYS)
static bool fast_cache_maint_outer(unsigned long start,
		unsigned long end, unsigned int op)
{
	bool result = false;
	if (end - start >= cache_maint_outer_threshold) {
		if (op == NVMAP_CACHE_OP_WB_INV) {
			outer_flush_all();
			result = true;
		}
		if (op == NVMAP_CACHE_OP_WB) {
			outer_clean_all();
			result = true;
		}
	}

	return result;
}
#else
static inline bool fast_cache_maint_outer(unsigned long start,
		unsigned long end, unsigned int op)
{
	return false;
}
#endif

#if defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS)
static bool fast_cache_maint(struct nvmap_handle *h,
	unsigned long start,
	unsigned long end, unsigned int op)
{
	if ((op == NVMAP_CACHE_OP_INV) ||
		((end - start) < cache_maint_inner_threshold))
		return false;

	if (op == NVMAP_CACHE_OP_WB_INV)
		inner_flush_cache_all();
	else if (op == NVMAP_CACHE_OP_WB)
		inner_clean_cache_all();

	/* outer maintenance */
	if (h->flags != NVMAP_HANDLE_INNER_CACHEABLE) {
		if(!fast_cache_maint_outer(start, end, op))
		{
			if (h->heap_pgalloc) {
				heap_page_cache_maint(h, start,
					end, op, false, true, NULL, 0, 0);
			} else  {
				phys_addr_t pstart;

				pstart = start + h->carveout->base;
				outer_cache_maint(op, pstart, end - start);
			}
		}
	}
	return true;
}
#else
static inline bool fast_cache_maint(struct nvmap_handle *h,
				    unsigned long start, unsigned long end,
				    unsigned int op)
{
	return false;
}
#endif

struct cache_maint_op {
	phys_addr_t start;
	phys_addr_t end;
	unsigned int op;
	struct nvmap_handle *h;
	bool inner;
	bool outer;
};

static int do_cache_maint(struct cache_maint_op *cache_work)
{
	pgprot_t prot;
	pte_t **pte = NULL;
	unsigned long kaddr;
	phys_addr_t pstart = cache_work->start;
	phys_addr_t pend = cache_work->end;
	phys_addr_t loop;
	int err = 0;
	struct nvmap_handle *h = cache_work->h;
	struct nvmap_client *client;
	unsigned int op = cache_work->op;

	if (!h || !h->alloc)
		return -EFAULT;

	client = h->owner;
	if (client)
		trace_cache_maint(client, h, pstart, pend, op);
	wmb();
	if (h->flags == NVMAP_HANDLE_UNCACHEABLE ||
	    h->flags == NVMAP_HANDLE_WRITE_COMBINE || pstart == pend)
		goto out;

	if (fast_cache_maint(h, pstart, pend, op))
		goto out;

	prot = nvmap_pgprot(h, PG_PROT_KERNEL);
	pte = nvmap_alloc_pte(h->dev, (void **)&kaddr);
	if (IS_ERR(pte)) {
		err = PTR_ERR(pte);
		pte = NULL;
		goto out;
	}

	if (h->heap_pgalloc) {
		heap_page_cache_maint(h, pstart, pend, op, true,
			(h->flags == NVMAP_HANDLE_INNER_CACHEABLE) ?
					false : true,
			pte, kaddr, prot);
		goto out;
	}

	if (pstart > h->size || pend > h->size) {
		pr_warn("cache maintenance outside handle\n");
		err = -EINVAL;
		goto out;
	}

	pstart += h->carveout->base;
	pend += h->carveout->base;
	loop = pstart;

	while (loop < pend) {
		phys_addr_t next = (loop + PAGE_SIZE) & PAGE_MASK;
		void *base = (void *)kaddr + (loop & ~PAGE_MASK);
		next = min(next, pend);

		set_pte_at(&init_mm, kaddr, *pte,
			   pfn_pte(__phys_to_pfn(loop), prot));
		nvmap_flush_tlb_kernel_page(kaddr);

		inner_cache_maint(op, base, next - loop);
		loop = next;
	}

	if (h->flags != NVMAP_HANDLE_INNER_CACHEABLE)
		outer_cache_maint(op, pstart, pend - pstart);

out:
	if (pte)
		nvmap_free_pte(h->dev, pte);
	return err;
}

int __nvmap_cache_maint(struct nvmap_client *client,
			struct nvmap_handle *h,
			unsigned long start, unsigned long end,
			unsigned int op, unsigned int allow_deferred)
{
	int err;
	struct cache_maint_op cache_op;

	h = nvmap_handle_get(h);
	if (!h)
		return -EFAULT;

	cache_op.h = h;
	cache_op.start = start;
	cache_op.end = end;
	cache_op.op = op;
	cache_op.inner = h->flags == NVMAP_HANDLE_CACHEABLE ||
			 h->flags == NVMAP_HANDLE_INNER_CACHEABLE;
	cache_op.outer = h->flags == NVMAP_HANDLE_CACHEABLE;

	err = do_cache_maint(&cache_op);
	nvmap_handle_put(h);
	return err;
}

static int rw_handle_page(struct nvmap_handle *h, int is_read,
			  unsigned long start, unsigned long rw_addr,
			  unsigned long bytes, unsigned long kaddr, pte_t *pte)
{
	pgprot_t prot = nvmap_pgprot(h, PG_PROT_KERNEL);
	unsigned long end = start + bytes;
	int err = 0;

	while (!err && start < end) {
		struct page *page = NULL;
		phys_addr_t phys;
		size_t count;
		void *src;

		if (!h->heap_pgalloc) {
			phys = h->carveout->base + start;
		} else {
			page = h->pgalloc.pages[start >> PAGE_SHIFT];
			BUG_ON(!page);
			get_page(page);
			phys = page_to_phys(page) + (start & ~PAGE_MASK);
		}

		set_pte_at(&init_mm, kaddr, pte,
			   pfn_pte(__phys_to_pfn(phys), prot));
		nvmap_flush_tlb_kernel_page(kaddr);

		src = (void *)kaddr + (phys & ~PAGE_MASK);
		phys = PAGE_SIZE - (phys & ~PAGE_MASK);
		count = min_t(size_t, end - start, phys);

		if (is_read)
			err = copy_to_user((void *)rw_addr, src, count);
		else
			err = copy_from_user(src, (void *)rw_addr, count);

		if (err)
			err = -EFAULT;

		rw_addr += count;
		start += count;

		if (page)
			put_page(page);
	}

	return err;
}

static ssize_t rw_handle(struct nvmap_client *client, struct nvmap_handle *h,
			 int is_read, unsigned long h_offs,
			 unsigned long sys_addr, unsigned long h_stride,
			 unsigned long sys_stride, unsigned long elem_size,
			 unsigned long count)
{
	ssize_t copied = 0;
	pte_t **pte;
	void *addr;
	int ret = 0;

	if (!elem_size)
		return -EINVAL;

	if (!h->alloc)
		return -EFAULT;

	if (elem_size == h_stride && elem_size == sys_stride) {
		elem_size *= count;
		h_stride = elem_size;
		sys_stride = elem_size;
		count = 1;
	}

	pte = nvmap_alloc_pte(nvmap_dev, &addr);
	if (IS_ERR(pte))
		return PTR_ERR(pte);

	while (count--) {
		if (h_offs + elem_size > h->size) {
			nvmap_warn(client, "read/write outside of handle\n");
			ret = -EFAULT;
			break;
		}
		if (is_read)
			__nvmap_cache_maint(client, h, h_offs,
				h_offs + elem_size, NVMAP_CACHE_OP_INV,
				CACHE_MAINT_IMMEDIATE);

		ret = rw_handle_page(h, is_read, h_offs, sys_addr,
				     elem_size, (unsigned long)addr, *pte);

		if (ret)
			break;

		if (!is_read)
			__nvmap_cache_maint(client, h, h_offs,
				h_offs + elem_size, NVMAP_CACHE_OP_WB_INV,
				CACHE_MAINT_IMMEDIATE);

		copied += elem_size;
		sys_addr += sys_stride;
		h_offs += h_stride;
	}

	nvmap_free_pte(nvmap_dev, pte);
	return ret ?: copied;
}
