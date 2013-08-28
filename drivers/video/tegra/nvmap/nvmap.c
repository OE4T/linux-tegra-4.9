/*
 * drivers/video/tegra/nvmap/nvmap.c
 *
 * Memory manager for Tegra GPU
 *
 * Copyright (c) 2009-2013, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/rbtree.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/export.h>

#include <asm/pgtable.h>
#include <asm/tlbflush.h>

#include <mach/iovmm.h>
#include <linux/nvmap.h>
#include <trace/events/nvmap.h>

#include "nvmap_priv.h"
#include "nvmap_mru.h"

/* private nvmap_handle flag for pinning duplicate detection */
#define NVMAP_HANDLE_VISITED (0x1ul << 31)

/* map the backing pages for a heap_pgalloc handle into its IOVMM area */
static int map_iovmm_area(struct nvmap_handle *h)
{
	int err;

	BUG_ON(!h->heap_pgalloc ||
	       !h->pgalloc.area ||
	       h->size & ~PAGE_MASK);
	WARN_ON(!h->pgalloc.dirty);

	err = tegra_iovmm_vm_insert_pages(h->pgalloc.area,
					  h->pgalloc.area->iovm_start,
					  h->pgalloc.pages,
					  h->size >> PAGE_SHIFT);
	if (err) {
		tegra_iovmm_zap_vm(h->pgalloc.area);
		return err;
	}
	h->pgalloc.dirty = false;
	return 0;
}

/* must be called inside nvmap_pin_lock, to ensure that an entire stream
 * of pins will complete without racing with a second stream. handle should
 * have nvmap_handle_get (or nvmap_validate_get) called before calling
 * this function. */
static int pin_locked(struct nvmap_client *client, struct nvmap_handle *h)
{
	struct tegra_iovmm_area *area;
	BUG_ON(!h->alloc);
	if (atomic_inc_return(&h->pin) == 1) {
		if (h->heap_pgalloc && !h->pgalloc.contig) {
			area = nvmap_handle_iovmm_locked(h);
			if (!area) {
				/* no race here, inside the pin mutex */
				atomic_dec(&h->pin);
				return -ENOMEM;
			}
			if (area != h->pgalloc.area)
				h->pgalloc.dirty = true;
			h->pgalloc.area = area;
		}
	}
	trace_handle_pin(client, client ? client->name : "kernel",
			 h, atomic_read(&h->pin));
	return 0;
}

/* doesn't need to be called inside nvmap_pin_lock, since this will only
 * expand the available VM area */
static int handle_unpin(struct nvmap_client *client,
		struct nvmap_handle *h, int free_vm)
{
	int ret = 0;

	nvmap_mru_lock(nvmap_share);

	if (atomic_read(&h->pin) == 0) {
		trace_handle_unpin_error(client,
			client ? client->name : "kernel",
			h, atomic_read(&h->pin));
		nvmap_err(client, "%s unpinning unpinned handle %p\n",
			  current->group_leader->comm, h);
		nvmap_mru_unlock(nvmap_share);
		return 0;
	}

	BUG_ON(!h->alloc);

	if (!atomic_dec_return(&h->pin)) {
		if (h->heap_pgalloc && h->pgalloc.area) {
			/* if a secure handle is clean (i.e., mapped into
			 * IOVMM, it needs to be zapped on unpin. */
			if (h->secure && !h->pgalloc.dirty) {
				tegra_iovmm_zap_vm(h->pgalloc.area);
				h->pgalloc.dirty = true;
			}
			if (free_vm) {
				tegra_iovmm_free_vm(h->pgalloc.area);
				h->pgalloc.area = NULL;
			} else
				nvmap_mru_insert_locked(nvmap_share, h);
			ret = 1;
		}
	}

	trace_handle_unpin(client, client ? client->name : "kernel",
			   h, atomic_read(&h->pin));
	nvmap_mru_unlock(nvmap_share);
	nvmap_handle_put(h);
	return ret;
}

static int pin_array_locked(struct nvmap_client *client,
		struct nvmap_handle **h, int count)
{
	int pinned;
	int i;
	int err = 0;

	/* Flush deferred cache maintenance if needed */
	for (pinned = 0; pinned < count; pinned++)
		if (nvmap_find_cache_maint_op(nvmap_dev, h[pinned]))
			nvmap_cache_maint_ops_flush(nvmap_dev, h[pinned]);

	nvmap_mru_lock(nvmap_share);
	for (pinned = 0; pinned < count; pinned++) {
		err = pin_locked(client, h[pinned]);
		if (err)
			break;
	}
	nvmap_mru_unlock(nvmap_share);

	if (err) {
		/* unpin pinned handles */
		for (i = 0; i < pinned; i++) {
			/* inc ref counter, because
			 * handle_unpin decrements it */
			nvmap_handle_get(h[i]);
			/* unpin handles and free vm */
			handle_unpin(client, h[i], true);
		}
	}

	if (err && tegra_iovmm_get_max_free(nvmap_share->iovmm) >=
		   nvmap_mru_vm_size(nvmap_share->iovmm)) {
		/* First attempt to pin in empty iovmm
		 * may still fail because of fragmentation caused by
		 * placing handles in MRU areas. After such failure
		 * all MRU gets cleaned and iovm space is freed.
		 *
		 * We have to do pinning again here since there might be is
		 * no more incoming pin_wait wakeup calls from unpin
		 * operations */
		nvmap_mru_lock(nvmap_share);
		for (pinned = 0; pinned < count; pinned++) {
			err = pin_locked(client, h[pinned]);
			if (err)
				break;
		}
		nvmap_mru_unlock(nvmap_share);

		if (err) {
			pr_err("Pinning in empty iovmm failed!!!\n");
			BUG_ON(1);
		}
	}
	return err;
}

static int wait_pin_array_locked(struct nvmap_client *client,
		struct nvmap_handle **h, int count)
{
	int ret = 0;

	ret = pin_array_locked(client, h, count);

	if (ret) {
		ret = wait_event_interruptible(nvmap_share->pin_wait,
				!pin_array_locked(client, h, count));
	}
	return ret ? -EINTR : 0;
}

static int handle_unpin_noref(struct nvmap_client *client, unsigned long id)
{
	struct nvmap_handle *h;
	int w;

	h = nvmap_validate_get(client, id, 0);
	if (unlikely(!h)) {
		nvmap_err(client, "%s attempting to unpin invalid handle %p\n",
			  current->group_leader->comm, (void *)id);
		return 0;
	}

	nvmap_err(client, "%s unpinning unreferenced handle %p\n",
		  current->group_leader->comm, h);
	WARN_ON(1);

	w = handle_unpin(client, h, false);
	nvmap_handle_put(h);
	return w;
}

void nvmap_unpin_ids(struct nvmap_client *client,
		     unsigned int nr, const unsigned long *ids)
{
	unsigned int i;
	int do_wake = 0;

	for (i = 0; i < nr; i++) {
		struct nvmap_handle_ref *ref;

		if (!ids[i] || WARN_ON(!virt_addr_valid(ids[i])))
			continue;

		nvmap_ref_lock(client);
		ref = __nvmap_validate_id_locked(client, ids[i]);
		if (ref) {
			struct nvmap_handle *h = ref->handle;
			int e = atomic_add_unless(&ref->pin, -1, 0);

			nvmap_ref_unlock(client);

			if (!e) {
				nvmap_err(client, "%s unpinning unpinned "
					  "handle %08lx\n",
					  current->group_leader->comm, ids[i]);
			} else {
				do_wake |= handle_unpin(client, h, false);
			}
		} else {
			nvmap_ref_unlock(client);
			if (client->super)
				do_wake |= handle_unpin_noref(client, ids[i]);
			else
				nvmap_err(client, "%s unpinning invalid "
					  "handle %08lx\n",
					  current->group_leader->comm, ids[i]);
		}
	}

	if (do_wake)
		wake_up(&nvmap_share->pin_wait);
}

/* pins a list of handle_ref objects; same conditions apply as to
 * _nvmap_handle_pin, but also bumps the pin count of each handle_ref. */
int nvmap_pin_ids(struct nvmap_client *client,
		  unsigned int nr, const unsigned long *ids)
{
	int ret = 0;
	int i;
	struct nvmap_handle **h = (struct nvmap_handle **)ids;
	struct nvmap_handle_ref *ref;

	/* to optimize for the common case (client provided valid handle
	 * references and the pin succeeds), increment the handle_ref pin
	 * count during validation. in error cases, the tree will need to
	 * be re-walked, since the handle_ref is discarded so that an
	 * allocation isn't required. if a handle_ref is not found,
	 * locally validate that the caller has permission to pin the handle;
	 * handle_refs are not created in this case, so it is possible that
	 * if the caller crashes after pinning a global handle, the handle
	 * will be permanently leaked. */
	nvmap_ref_lock(client);
	for (i = 0; i < nr; i++) {
		ref = __nvmap_validate_id_locked(client, ids[i]);
		if (ref) {
			atomic_inc(&ref->pin);
			nvmap_handle_get(h[i]);
		} else {
			struct nvmap_handle *verify;
			nvmap_ref_unlock(client);
			verify = nvmap_validate_get(client, ids[i], 0);
			if (verify) {
				nvmap_warn(client, "%s pinning unreferenced "
					   "handle %p\n",
					   current->group_leader->comm, h[i]);
			} else {
				ret = -EPERM;
				nr = i;
				break;
			}
			nvmap_ref_lock(client);
		}
		if (!h[i]->alloc) {
			ret = -EFAULT;
			nr = i + 1;
			break;
		}
	}
	nvmap_ref_unlock(client);

	if (ret)
		goto out;

	ret = mutex_lock_interruptible(&nvmap_share->pin_lock);
	if (WARN_ON(ret))
		goto out;

	ret = wait_pin_array_locked(client, h, nr);

	mutex_unlock(&nvmap_share->pin_lock);

	if (ret) {
		ret = -EINTR;
	} else {
		for (i = 0; i < nr; i++) {
			if (h[i]->heap_pgalloc && h[i]->pgalloc.dirty) {
				ret = map_iovmm_area(h[i]);
				while (ret && --i >= 0)
					tegra_iovmm_zap_vm(h[i]->pgalloc.area);
			}
		}
	}

out:
	if (ret) {
		nvmap_ref_lock(client);
		for (i = 0; i < nr; i++) {
			if (!ids[i])
				continue;

			ref = __nvmap_validate_id_locked(client, ids[i]);
			if (!ref) {
				nvmap_warn(client, "%s freed handle %p "
					   "during pinning\n",
					   current->group_leader->comm,
					   (void *)ids[i]);
				continue;
			}
			atomic_dec(&ref->pin);
		}
		nvmap_ref_unlock(client);

		for (i = 0; i < nr; i++)
			if(h[i])
				nvmap_handle_put(h[i]);
	}

	return ret;
}

static phys_addr_t handle_phys(struct nvmap_handle *h)
{
	phys_addr_t addr;

	if (h->heap_pgalloc && h->pgalloc.contig) {
		addr = page_to_phys(h->pgalloc.pages[0]);
	} else if (h->heap_pgalloc) {
		BUG_ON(!h->pgalloc.area);
		addr = h->pgalloc.area->iovm_start;
	} else {
		addr = h->carveout->base;
	}

	return addr;
}

/*
 * Get physical address of the handle. Handle should be
 * already validated and pinned.
 */
phys_addr_t nvmap_get_addr_from_user_id(ulong user_id)
{
	struct nvmap_handle *h;

	h = (struct nvmap_handle *)((uintptr_t)unmarshal_user_id(user_id));
	return handle_phys(h);
}

int __nvmap_pin(struct nvmap_client *client, struct nvmap_handle *h,
	       phys_addr_t *phys)
{
	int ret = 0;

	if (!virt_addr_valid(h))
		return -EINVAL;

	h = nvmap_handle_get(h);

	if (WARN_ON(mutex_lock_interruptible(&nvmap_share->pin_lock))) {
		ret = -EINTR;
	} else {
		ret = wait_pin_array_locked(client, &h, 1);
		mutex_unlock(&nvmap_share->pin_lock);
	}

	if (ret) {
		goto err_out;
	} else {
		if (h->heap_pgalloc && h->pgalloc.dirty)
			ret = map_iovmm_area(h);
		if (ret)
			goto err_out_unpin;
		*phys = handle_phys(h);
	}

	return 0;

err_out_unpin:
	nvmap_handle_get(h);
	handle_unpin(client, h, true);
err_out:
	nvmap_handle_put(h);
	return ret;
}

/*
 * Pin handle without slow validation step
 */
int _nvmap_pin(struct nvmap_client *client, struct nvmap_handle_ref *ref,
	       phys_addr_t *phys)
{
	int ret = 0;

	if (!virt_addr_valid(client) ||
	    !virt_addr_valid(ref) ||
	    !virt_addr_valid(ref->handle) ||
	    !phys)
		return -EINVAL;

	atomic_inc(&ref->pin);
	ret = __nvmap_pin(client, ref->handle, phys);
	if (ret)
		atomic_dec(&ref->pin);

	return ret;
}

int nvmap_pin(struct nvmap_client *client, struct nvmap_handle_ref *ref,
	      phys_addr_t *phys)
{
	struct nvmap_handle *h;

	if (!virt_addr_valid(client) ||
	    !virt_addr_valid(ref) ||
	    !virt_addr_valid(ref->handle))
		return -EINVAL;

	nvmap_ref_lock(client);
	ref = __nvmap_validate_id_locked(client, (unsigned long)ref->handle);
	if (ref)
		h = ref->handle;
	nvmap_ref_unlock(client);

	return _nvmap_pin(client, ref, phys);
}
EXPORT_SYMBOL(nvmap_pin);

static phys_addr_t nvmap_handle_address(struct nvmap_client *c,
					unsigned long id)
{
	struct nvmap_handle *h;
	phys_addr_t phys;

	h = nvmap_get_handle_id(c, id);
	if (!h)
		return -EPERM;
	mutex_lock(&h->lock);
	phys = handle_phys(h);
	mutex_unlock(&h->lock);
	nvmap_handle_put(h);

	return phys;
}

phys_addr_t nvmap_handle_address_user_id(struct nvmap_client *c,
					unsigned long user_id)
{
	if (!virt_addr_valid(c))
		return -EINVAL;
	return nvmap_handle_address(c, unmarshal_user_id(user_id));
}

void __nvmap_unpin(struct nvmap_client *client, struct nvmap_handle *h)
{
	if (!h)
		return;

	if (handle_unpin(client, h, false))
		wake_up(&nvmap_share->pin_wait);
}

void nvmap_unpin(struct nvmap_client *client, struct nvmap_handle_ref *ref)
{
	if (!ref ||
	    WARN_ON(!virt_addr_valid(client)) ||
	    WARN_ON(!virt_addr_valid(ref)) ||
	    WARN_ON(!virt_addr_valid(ref->handle)))
		return;

	atomic_dec(&ref->pin);
	__nvmap_unpin(client, ref->handle);
}
EXPORT_SYMBOL(nvmap_unpin);

void nvmap_unpin_handles(struct nvmap_client *client,
			 struct nvmap_handle **h, int nr)
{
	int i;
	int do_wake = 0;

	if (!virt_addr_valid(client) ||
	    !virt_addr_valid(h))
		return;
	for (i = 0; i < nr; i++) {
		if (WARN_ON(!h[i]))
			continue;
		do_wake |= handle_unpin(client, h[i], false);
	}

	if (do_wake)
		wake_up(&nvmap_share->pin_wait);
}

void *__nvmap_kmap(struct nvmap_handle *h, unsigned int pagenum)
{
	phys_addr_t paddr;
	unsigned long kaddr;
	pgprot_t prot;
	pte_t **pte;

	if (!virt_addr_valid(h))
		return NULL;

	h = nvmap_handle_get(h);
	if (!h)
		return NULL;

	if (pagenum >= h->size >> PAGE_SHIFT)
		goto out;
	prot = nvmap_pgprot(h, pgprot_kernel);
	pte = nvmap_alloc_pte(nvmap_dev, (void **)&kaddr);
	if (!pte)
		goto out;

	if (h->heap_pgalloc)
		paddr = page_to_phys(h->pgalloc.pages[pagenum]);
	else
		paddr = h->carveout->base + pagenum * PAGE_SIZE;

	set_pte_at(&init_mm, kaddr, *pte,
				pfn_pte(__phys_to_pfn(paddr), prot));
	nvmap_flush_tlb_kernel_page(kaddr);
	return (void *)kaddr;
out:
	nvmap_handle_put(h);
	return NULL;
}

void *nvmap_kmap(struct nvmap_handle_ref *ref, unsigned int pagenum)
{
	if (!virt_addr_valid(ref) ||
	    !virt_addr_valid(ref->handle))
		return NULL;

	return __nvmap_kmap(ref->handle, pagenum);
}

void __nvmap_kunmap(struct nvmap_handle *h, unsigned int pagenum,
		  void *addr)
{
	phys_addr_t paddr;
	pte_t **pte;

	if (!h ||
	    WARN_ON(!virt_addr_valid(h)) ||
	    WARN_ON(!addr))
		return;

	if (WARN_ON(pagenum >= h->size >> PAGE_SHIFT))
		return;

	if (nvmap_find_cache_maint_op(h->dev, h)) {
		struct nvmap_share *share = nvmap_get_share_from_dev(h->dev);
		/* acquire pin lock to ensure maintenance is done before
		 * handle is pinned */
		mutex_lock(&share->pin_lock);
		nvmap_cache_maint_ops_flush(h->dev, h);
		mutex_unlock(&share->pin_lock);
	}

	if (h->heap_pgalloc)
		paddr = page_to_phys(h->pgalloc.pages[pagenum]);
	else
		paddr = h->carveout->base + pagenum * PAGE_SIZE;

	if (h->flags != NVMAP_HANDLE_UNCACHEABLE &&
	    h->flags != NVMAP_HANDLE_WRITE_COMBINE) {
		dmac_flush_range(addr, addr + PAGE_SIZE);
#ifndef CONFIG_ARM64
		outer_flush_range(paddr, paddr + PAGE_SIZE); /* FIXME */
#endif
	}

	pte = nvmap_vaddr_to_pte(nvmap_dev, (unsigned long)addr);
	nvmap_free_pte(nvmap_dev, pte);
	nvmap_handle_put(h);
}

void nvmap_kunmap(struct nvmap_handle_ref *ref, unsigned int pagenum,
		  void *addr)
{
	if (!ref ||
	    WARN_ON(!virt_addr_valid(ref)) ||
	    WARN_ON(!addr))
		return;

	__nvmap_kunmap(ref->handle, pagenum, addr);
}

void *__nvmap_mmap(struct nvmap_handle *h)
{
	pgprot_t prot;
	unsigned long adj_size;
	unsigned long offs;
	struct vm_struct *v;
	void *p;

	if (!virt_addr_valid(h))
		return NULL;

	h = nvmap_handle_get(h);
	if (!h)
		return NULL;

	prot = nvmap_pgprot(h, pgprot_kernel);

	if (h->heap_pgalloc)
		return vm_map_ram(h->pgalloc.pages, h->size >> PAGE_SHIFT,
				  -1, prot);

	/* carveout - explicitly map the pfns into a vmalloc area */

	adj_size = h->carveout->base & ~PAGE_MASK;
	adj_size += h->size;
	adj_size = PAGE_ALIGN(adj_size);

	v = alloc_vm_area(adj_size, 0);
	if (!v) {
		nvmap_handle_put(h);
		return NULL;
	}

	p = v->addr + (h->carveout->base & ~PAGE_MASK);

	for (offs = 0; offs < adj_size; offs += PAGE_SIZE) {
		unsigned long addr = (unsigned long) v->addr + offs;
		unsigned int pfn;
		pgd_t *pgd;
		pud_t *pud;
		pmd_t *pmd;
		pte_t *pte;

		pfn = __phys_to_pfn(h->carveout->base + offs);
		pgd = pgd_offset_k(addr);
		pud = pud_alloc(&init_mm, pgd, addr);
		if (!pud)
			break;
		pmd = pmd_alloc(&init_mm, pud, addr);
		if (!pmd)
			break;
		pte = pte_alloc_kernel(pmd, addr);
		if (!pte)
			break;
		set_pte_at(&init_mm, addr, pte, pfn_pte(pfn, prot));
		nvmap_flush_tlb_kernel_page(addr);
	}

	if (offs != adj_size) {
		free_vm_area(v);
		nvmap_handle_put(h);
		return NULL;
	}

	/* leave the handle ref count incremented by 1, so that
	 * the handle will not be freed while the kernel mapping exists.
	 * nvmap_handle_put will be called by unmapping this address */
	return p;
}

void *nvmap_mmap(struct nvmap_handle_ref *ref)
{
	if (!virt_addr_valid(ref))
		return NULL;

	return __nvmap_mmap(ref->handle);
}
EXPORT_SYMBOL(nvmap_mmap);

void __nvmap_munmap(struct nvmap_handle *h, void *addr)
{
	if (!h ||
	    WARN_ON(!virt_addr_valid(h)) ||
	    WARN_ON(!addr))
		return;

	if (nvmap_find_cache_maint_op(h->dev, h)) {
		struct nvmap_share *share = nvmap_get_share_from_dev(h->dev);
		/* acquire pin lock to ensure maintenance is done before
		 * handle is pinned */
		mutex_lock(&share->pin_lock);
		nvmap_cache_maint_ops_flush(h->dev, h);
		mutex_unlock(&share->pin_lock);
	}

	/* Handle can be locked by cache maintenance in
	 * separate thread */
	if (h->heap_pgalloc) {
		vm_unmap_ram(addr, h->size >> PAGE_SHIFT);
	} else {
		struct vm_struct *vm;
		addr -= (h->carveout->base & ~PAGE_MASK);
		vm = remove_vm_area(addr);
		BUG_ON(!vm);
		kfree(vm);
	}
	nvmap_handle_put(h);
}

void nvmap_munmap(struct nvmap_handle_ref *ref, void *addr)
{
	if (!ref ||
	    WARN_ON(!virt_addr_valid(ref)) ||
	    WARN_ON(!addr))
		return;

	__nvmap_munmap(ref->handle, addr);
}
EXPORT_SYMBOL(nvmap_munmap);

static struct nvmap_client *nvmap_get_dmabuf_client(void)
{
	static struct nvmap_client *client;

	if (!client) {
		struct nvmap_client *temp;

		temp = nvmap_create_client(nvmap_dev, "dmabuf_client");
		if (!temp)
			return NULL;
		if (cmpxchg(&client, NULL, temp))
			nvmap_client_put(temp);
	}
	BUG_ON(!client);
	return client;
}

struct dma_buf *nvmap_alloc_dmabuf(size_t size, size_t align,
				   unsigned int flags,
				   unsigned int heap_mask)
{
	struct dma_buf *dmabuf;
	struct nvmap_handle_ref *ref;
	struct nvmap_client *client = nvmap_get_dmabuf_client();

	ref = nvmap_alloc(client, size, align, flags, heap_mask);
	if (!ref)
		return ERR_PTR(-ENOMEM);

	dmabuf = nvmap_dmabuf_export_from_ref(ref);
	nvmap_free(client, ref);
	return dmabuf;
}

struct nvmap_handle_ref *nvmap_alloc(struct nvmap_client *client, size_t size,
				     size_t align, unsigned int flags,
				     unsigned int heap_mask)
{
	const unsigned int default_heap = NVMAP_HEAP_CARVEOUT_GENERIC;
	struct nvmap_handle_ref *r = NULL;
	int err;

	if (!virt_addr_valid(client))
		return ERR_PTR(-EINVAL);

	if (heap_mask == 0)
		heap_mask = default_heap;

	r = nvmap_create_handle(client, size);
	if (IS_ERR(r))
		return r;

	err = nvmap_alloc_handle_id(client, nvmap_ref_to_id(r),
				    heap_mask, align,
				    0, /* kind n/a */
				    flags & ~(NVMAP_HANDLE_KIND_SPECIFIED |
					      NVMAP_HANDLE_COMPR_SPECIFIED));

	if (err) {
		nvmap_free_handle_id(client, nvmap_ref_to_id(r));
		return ERR_PTR(err);
	}

	return r;
}
EXPORT_SYMBOL(nvmap_alloc);

void nvmap_free(struct nvmap_client *client, struct nvmap_handle_ref *r)
{
	unsigned long ref_id = nvmap_ref_to_id(r);

	if (!r ||
	    WARN_ON(!virt_addr_valid(client)) ||
	    WARN_ON(!virt_addr_valid(r)) ||
	    WARN_ON(!virt_addr_valid(ref_id)))
		return;

	nvmap_free_handle_id(client, ref_id);
}
EXPORT_SYMBOL(nvmap_free);

void nvmap_handle_put(struct nvmap_handle *h)
{
	int cnt;

	if (WARN_ON(!virt_addr_valid(h)))
		return;
	cnt = atomic_dec_return(&h->ref);

	if (WARN_ON(cnt < 0)) {
		pr_err("%s: %s put to negative references\n",
			__func__, current->comm);
	} else if (cnt == 0)
		_nvmap_handle_free(h);
}

void nvmap_put_handle_user_id(ulong user_id)
{
	struct nvmap_handle *h;

	h = (struct nvmap_handle *)unmarshal_user_id(user_id);
	nvmap_handle_put(h);
}

struct sg_table *__nvmap_sg_table(struct nvmap_client *client,
		struct nvmap_handle *h)
{
	struct sg_table *sgt = NULL;
	int err, npages;

	if (!virt_addr_valid(h))
		return ERR_PTR(-EINVAL);

	h = nvmap_handle_get(h);
	if (!h)
		return ERR_PTR(-EINVAL);

	npages = PAGE_ALIGN(h->size) >> PAGE_SHIFT;
	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		err = -ENOMEM;
		goto err;
	}

	if (!h->heap_pgalloc) {
		err = sg_alloc_table(sgt, 1, GFP_KERNEL);
		if (err)
			goto err;
		sg_set_buf(sgt->sgl, phys_to_virt(handle_phys(h)), h->size);
	} else {
		err = sg_alloc_table_from_pages(sgt, h->pgalloc.pages,
				npages, 0, h->size, GFP_KERNEL);
		if (err)
			goto err;
	}
	nvmap_handle_put(h);
	return sgt;

err:
	kfree(sgt);
	nvmap_handle_put(h);
	return ERR_PTR(err);
}

struct sg_table *nvmap_sg_table(struct nvmap_client *client,
		struct nvmap_handle_ref *ref)
{
	if (!virt_addr_valid(ref))
		return ERR_PTR(-EINVAL);
	return __nvmap_sg_table(client, ref->handle);
}

void __nvmap_free_sg_table(struct nvmap_client *client,
		struct nvmap_handle *h, struct sg_table *sgt)
{
	if (WARN_ON(!virt_addr_valid(sgt)))
		return;
	sg_free_table(sgt);
	kfree(sgt);
}

void nvmap_free_sg_table(struct nvmap_client *client,
		struct nvmap_handle_ref *ref, struct sg_table *sgt)
{
	if (WARN_ON(!virt_addr_valid(ref)))
		return;
	__nvmap_free_sg_table(client, ref->handle, sgt);
}

void nvmap_set_nvhost_private(struct nvmap_handle_ref *ref, void *priv,
		void (*delete)(void *priv))
{
	struct nvmap_handle *h;

	if (WARN_ON(!virt_addr_valid(ref)) ||
	    WARN_ON(!virt_addr_valid(ref->handle)))
		return;

	h = nvmap_handle_get(ref->handle);
	if (WARN_ON(!h))
		return;

	h->nvhost_priv = priv;
	h->nvhost_priv_delete = delete;
	nvmap_handle_put(ref->handle);
}

void *nvmap_get_nvhost_private(struct nvmap_handle_ref *ref)
{
	struct nvmap_handle *h;
	void *priv;

	if (!virt_addr_valid(ref) ||
	    !virt_addr_valid(ref->handle))
		return ERR_PTR(-EINVAL);

	h = nvmap_handle_get(ref->handle);
	if (!h)
		return ERR_PTR(-EINVAL);

	priv = h->nvhost_priv;
	nvmap_handle_put(ref->handle);

	return priv;
}

void nvmap_flush_deferred_cache(struct nvmap_client *client,
		struct nvmap_handle_ref *ref)
{
#if CONFIG_NVMAP_DEFERRED_CACHE_MAINT
	struct nvmap_handle *h;

	if (WARN_ON(!virt_addr_valid(ref)) ||
	    WARN_ON(!virt_addr_valid(ref->handle)))
		return;

	h = nvmap_handle_get(ref->handle);
	if (!h)
		return;

	if (nvmap_find_cache_maint_op(h->dev, h))
		nvmap_cache_maint_ops_flush(h->dev, h);

	nvmap_handle_put(ref->handle);
#endif
}
