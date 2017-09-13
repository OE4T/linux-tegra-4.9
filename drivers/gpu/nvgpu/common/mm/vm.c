/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/log.h>
#include <nvgpu/dma.h>
#include <nvgpu/vm.h>
#include <nvgpu/vm_area.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/lock.h>
#include <nvgpu/list.h>
#include <nvgpu/rbtree.h>
#include <nvgpu/semaphore.h>
#include <nvgpu/enabled.h>

#include <nvgpu/vgpu/vm.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"

int vm_aspace_id(struct vm_gk20a *vm)
{
	return vm->as_share ? vm->as_share->id : -1;
}

static void __nvgpu_vm_free_entries(struct vm_gk20a *vm,
				    struct nvgpu_gmmu_pd *pd,
				    int level)
{
	int i;

	if (pd->mem) {
		__nvgpu_pd_free(vm, pd);
		pd->mem = NULL;
	}

	if (pd->entries) {
		for (i = 0; i < pd->num_entries; i++)
			__nvgpu_vm_free_entries(vm, &pd->entries[i],
					      level + 1);
		nvgpu_vfree(vm->mm->g, pd->entries);
		pd->entries = NULL;
	}
}

static void nvgpu_vm_free_entries(struct vm_gk20a *vm,
				  struct nvgpu_gmmu_pd *pdb)
{
	struct gk20a *g = vm->mm->g;
	int i;

	__nvgpu_pd_cache_free_direct(g, pdb);

	if (!pdb->entries)
		return;

	for (i = 0; i < pdb->num_entries; i++)
		__nvgpu_vm_free_entries(vm, &pdb->entries[i], 1);

	nvgpu_vfree(g, pdb->entries);
	pdb->entries = NULL;
}

u64 __nvgpu_vm_alloc_va(struct vm_gk20a *vm, u64 size,
			enum gmmu_pgsz_gk20a pgsz_idx)

{
	struct gk20a *g = vm->mm->g;
	struct nvgpu_allocator *vma = NULL;
	u64 addr;
	u64 page_size = vm->gmmu_page_sizes[pgsz_idx];

	vma = vm->vma[pgsz_idx];

	if (pgsz_idx >= gmmu_nr_page_sizes) {
		nvgpu_err(g, "(%s) invalid page size requested", vma->name);
		return 0;
	}

	if ((pgsz_idx == gmmu_page_size_big) && !vm->big_pages) {
		nvgpu_err(g, "(%s) unsupportd page size requested", vma->name);
		return 0;
	}

	/* Be certain we round up to page_size if needed */
	size = (size + ((u64)page_size - 1)) & ~((u64)page_size - 1);

	addr = nvgpu_alloc(vma, size);
	if (!addr) {
		nvgpu_err(g, "(%s) oom: sz=0x%llx", vma->name, size);
		return 0;
	}

	return addr;
}

int __nvgpu_vm_free_va(struct vm_gk20a *vm, u64 addr,
		       enum gmmu_pgsz_gk20a pgsz_idx)
{
	struct nvgpu_allocator *vma = vm->vma[pgsz_idx];

	nvgpu_free(vma, addr);

	return 0;
}

void nvgpu_vm_mapping_batch_start(struct vm_gk20a_mapping_batch *mapping_batch)
{
	memset(mapping_batch, 0, sizeof(*mapping_batch));
	mapping_batch->gpu_l2_flushed = false;
	mapping_batch->need_tlb_invalidate = false;
}

void nvgpu_vm_mapping_batch_finish_locked(
	struct vm_gk20a *vm, struct vm_gk20a_mapping_batch *mapping_batch)
{
	/* hanging kref_put batch pointer? */
	WARN_ON(vm->kref_put_batch == mapping_batch);

	if (mapping_batch->need_tlb_invalidate) {
		struct gk20a *g = gk20a_from_vm(vm);
		g->ops.fb.tlb_invalidate(g, vm->pdb.mem);
	}
}

void nvgpu_vm_mapping_batch_finish(struct vm_gk20a *vm,
				   struct vm_gk20a_mapping_batch *mapping_batch)
{
	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	nvgpu_vm_mapping_batch_finish_locked(vm, mapping_batch);
	nvgpu_mutex_release(&vm->update_gmmu_lock);
}

/*
 * Determine if the passed address space can support big pages or not.
 */
int nvgpu_big_pages_possible(struct vm_gk20a *vm, u64 base, u64 size)
{
	u64 mask = ((u64)vm->big_page_size << 10) - 1;

	if (base & mask || size & mask)
		return 0;
	return 1;
}

/*
 * Initialize a semaphore pool. Just return successfully if we do not need
 * semaphores (i.e when sync-pts are active).
 */
static int nvgpu_init_sema_pool(struct vm_gk20a *vm)
{
	struct nvgpu_semaphore_sea *sema_sea;
	struct mm_gk20a *mm = vm->mm;
	struct gk20a *g = mm->g;
	int err;

	/*
	 * Don't waste the memory on semaphores if we don't need them.
	 */
	if (g->gpu_characteristics.flags & NVGPU_GPU_FLAGS_HAS_SYNCPOINTS)
		return 0;

	if (vm->sema_pool)
		return 0;

	sema_sea = nvgpu_semaphore_sea_create(g);
	if (!sema_sea)
		return -ENOMEM;

	vm->sema_pool = nvgpu_semaphore_pool_alloc(sema_sea);
	if (!vm->sema_pool)
		return -ENOMEM;

	/*
	 * Allocate a chunk of GPU VA space for mapping the semaphores. We will
	 * do a fixed alloc in the kernel VM so that all channels have the same
	 * RO address range for the semaphores.
	 *
	 * !!! TODO: cleanup.
	 */
	sema_sea->gpu_va = nvgpu_alloc_fixed(&vm->kernel,
					     vm->va_limit -
					     mm->channel.kernel_size,
					     512 * PAGE_SIZE,
					     SZ_4K);
	if (!sema_sea->gpu_va) {
		nvgpu_free(&vm->kernel, sema_sea->gpu_va);
		nvgpu_vm_put(vm);
		return -ENOMEM;
	}

	err = nvgpu_semaphore_pool_map(vm->sema_pool, vm);
	if (err) {
		nvgpu_semaphore_pool_unmap(vm->sema_pool, vm);
		nvgpu_free(vm->vma[gmmu_page_size_small],
			   vm->sema_pool->gpu_va);
		return err;
	}

	return 0;
}

static int __nvgpu_vm_init(struct mm_gk20a *mm,
			   struct vm_gk20a *vm,
			   u32 big_page_size,
			   u64 low_hole,
			   u64 kernel_reserved,
			   u64 aperture_size,
			   bool big_pages,
			   bool userspace_managed,
			   char *name)
{
	int err;
	char alloc_name[32];
	u64 kernel_vma_flags;
	u64 user_vma_start, user_vma_limit;
	u64 user_lp_vma_start, user_lp_vma_limit;
	u64 kernel_vma_start, kernel_vma_limit;
	struct gk20a *g = gk20a_from_mm(mm);

	if (WARN_ON(kernel_reserved + low_hole > aperture_size))
		return -ENOMEM;

	nvgpu_log_info(g, "Init space for %s: valimit=0x%llx, "
		       "LP size=0x%x lowhole=0x%llx",
		       name, aperture_size,
		       (unsigned int)big_page_size, low_hole);

	vm->mm = mm;

	vm->gmmu_page_sizes[gmmu_page_size_small]  = SZ_4K;
	vm->gmmu_page_sizes[gmmu_page_size_big]    = big_page_size;
	vm->gmmu_page_sizes[gmmu_page_size_kernel] = SZ_4K;

	/* Set up vma pointers. */
	vm->vma[gmmu_page_size_small]  = &vm->user;
	vm->vma[gmmu_page_size_big]    = &vm->user;
	vm->vma[gmmu_page_size_kernel] = &vm->kernel;
	if (!nvgpu_is_enabled(g, NVGPU_MM_UNIFY_ADDRESS_SPACES))
		vm->vma[gmmu_page_size_big] = &vm->user_lp;

	vm->va_start  = low_hole;
	vm->va_limit  = aperture_size;

	vm->big_page_size     = vm->gmmu_page_sizes[gmmu_page_size_big];
	vm->userspace_managed = userspace_managed;
	vm->mmu_levels        = g->ops.mm.get_mmu_levels(g, vm->big_page_size);

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	if (g->is_virtual && userspace_managed) {
		nvgpu_err(g, "vGPU: no userspace managed addr space support");
		return -ENOSYS;
	}
	if (g->is_virtual && vgpu_vm_init(g, vm)) {
		nvgpu_err(g, "Failed to init vGPU VM!");
		return -ENOMEM;
	}
#endif

	/* Initialize the page table data structures. */
	strncpy(vm->name, name, min(strlen(name), sizeof(vm->name)));
	err = nvgpu_gmmu_init_page_table(vm);
	if (err)
		goto clean_up_vgpu_vm;

	/* Setup vma limits. */
	if (kernel_reserved + low_hole < aperture_size) {
		/*
		 * If big_pages are disabled for this VM then it only makes
		 * sense to make one VM, same as if the unified address flag
		 * is set.
		 */
		if (!big_pages ||
		    nvgpu_is_enabled(g, NVGPU_MM_UNIFY_ADDRESS_SPACES)) {
			user_vma_start = low_hole;
			user_vma_limit = vm->va_limit - kernel_reserved;
			user_lp_vma_start = user_vma_limit;
			user_lp_vma_limit = user_vma_limit;
		} else {
			user_vma_start = low_hole;
			user_vma_limit = __nv_gmmu_va_small_page_limit();
			user_lp_vma_start = __nv_gmmu_va_small_page_limit();
			user_lp_vma_limit = vm->va_limit - kernel_reserved;
		}
	} else {
		user_vma_start = 0;
		user_vma_limit = 0;
		user_lp_vma_start = 0;
		user_lp_vma_limit = 0;
	}
	kernel_vma_start = vm->va_limit - kernel_reserved;
	kernel_vma_limit = vm->va_limit;

	nvgpu_log_info(g, "user_vma     [0x%llx,0x%llx)",
		       user_vma_start, user_vma_limit);
	nvgpu_log_info(g, "user_lp_vma  [0x%llx,0x%llx)",
		       user_lp_vma_start, user_lp_vma_limit);
	nvgpu_log_info(g, "kernel_vma   [0x%llx,0x%llx)",
		       kernel_vma_start, kernel_vma_limit);

	if (WARN_ON(user_vma_start > user_vma_limit) ||
	    WARN_ON(user_lp_vma_start > user_lp_vma_limit) ||
	    WARN_ON(kernel_vma_start >= kernel_vma_limit)) {
		err = -EINVAL;
		goto clean_up_page_tables;
	}

	kernel_vma_flags = (kernel_reserved + low_hole) == aperture_size ?
		0 : GPU_ALLOC_GVA_SPACE;

	/*
	 * A "user" area only makes sense for the GVA spaces. For VMs where
	 * there is no "user" area user_vma_start will be equal to
	 * user_vma_limit (i.e a 0 sized space). In such a situation the kernel
	 * area must be non-zero in length.
	 */
	if (user_vma_start >= user_vma_limit &&
	    kernel_vma_start >= kernel_vma_limit) {
		err = -EINVAL;
		goto clean_up_page_tables;
	}

	/*
	 * Determine if big pages are possible in this VM. If a split address
	 * space is used then check the user_lp vma instead of the user vma.
	 */
	if (nvgpu_is_enabled(g, NVGPU_MM_UNIFY_ADDRESS_SPACES))
		vm->big_pages = big_pages &&
			nvgpu_big_pages_possible(vm, user_vma_start,
					user_vma_limit - user_vma_start);
	else
		vm->big_pages = big_pages &&
			nvgpu_big_pages_possible(vm, user_lp_vma_start,
					user_lp_vma_limit - user_lp_vma_start);

	/*
	 * User VMA.
	 */
	if (user_vma_start < user_vma_limit) {
		snprintf(alloc_name, sizeof(alloc_name), "gk20a_%s", name);
		err = __nvgpu_buddy_allocator_init(g, &vm->user,
						   vm, alloc_name,
						   user_vma_start,
						   user_vma_limit -
						   user_vma_start,
						   SZ_4K,
						   GPU_BALLOC_MAX_ORDER,
						   GPU_ALLOC_GVA_SPACE);
		if (err)
			goto clean_up_page_tables;
	} else {
		/*
		 * Make these allocator pointers point to the kernel allocator
		 * since we still use the legacy notion of page size to choose
		 * the allocator.
		 */
		vm->vma[0] = &vm->kernel;
		vm->vma[1] = &vm->kernel;
	}

	/*
	 * User VMA for large pages when a split address range is used.
	 */
	if (user_lp_vma_start < user_lp_vma_limit) {
		snprintf(alloc_name, sizeof(alloc_name), "gk20a_%s_lp", name);
		err = __nvgpu_buddy_allocator_init(g, &vm->user_lp,
						   vm, alloc_name,
						   user_lp_vma_start,
						   user_lp_vma_limit -
						   user_lp_vma_start,
						   vm->big_page_size,
						   GPU_BALLOC_MAX_ORDER,
						   GPU_ALLOC_GVA_SPACE);
		if (err)
			goto clean_up_allocators;
	}

	/*
	 * Kernel VMA. Must always exist for an address space.
	 */
	snprintf(alloc_name, sizeof(alloc_name), "gk20a_%s-sys", name);
	err = __nvgpu_buddy_allocator_init(g, &vm->kernel,
					   vm, alloc_name,
					   kernel_vma_start,
					   kernel_vma_limit - kernel_vma_start,
					   SZ_4K,
					   GPU_BALLOC_MAX_ORDER,
					   kernel_vma_flags);
	if (err)
		goto clean_up_allocators;

	vm->mapped_buffers = NULL;

	nvgpu_mutex_init(&vm->update_gmmu_lock);
	nvgpu_ref_init(&vm->ref);
	nvgpu_init_list_node(&vm->vm_area_list);

	/*
	 * This is only necessary for channel address spaces. The best way to
	 * distinguish channel address spaces from other address spaces is by
	 * size - if the address space is 4GB or less, it's not a channel.
	 */
	if (vm->va_limit > SZ_4G) {
		err = nvgpu_init_sema_pool(vm);
		if (err)
			goto clean_up_allocators;
	}

	return 0;

clean_up_allocators:
	if (nvgpu_alloc_initialized(&vm->kernel))
		nvgpu_alloc_destroy(&vm->kernel);
	if (nvgpu_alloc_initialized(&vm->user))
		nvgpu_alloc_destroy(&vm->user);
	if (nvgpu_alloc_initialized(&vm->user_lp))
		nvgpu_alloc_destroy(&vm->user_lp);
clean_up_page_tables:
	/* Cleans up nvgpu_gmmu_init_page_table() */
	__nvgpu_pd_cache_free_direct(g, &vm->pdb);
clean_up_vgpu_vm:
#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	if (g->is_virtual)
		vgpu_vm_remove(vm);
#endif
	return err;
}

/**
 * nvgpu_init_vm() - Initialize an address space.
 *
 * @mm - Parent MM.
 * @vm - The VM to init.
 * @big_page_size - Size of big pages associated with this VM.
 * @low_hole - The size of the low hole (unaddressable memory at the bottom of
 *	       the address space).
 * @kernel_reserved - Space reserved for kernel only allocations.
 * @aperture_size - Total size of the aperture.
 * @big_pages - If true then big pages are possible in the VM. Note this does
 *              not guarantee that big pages will be possible.
 * @name - Name of the address space.
 *
 * This function initializes an address space according to the following map:
 *
 *     +--+ 0x0
 *     |  |
 *     +--+ @low_hole
 *     |  |
 *     ~  ~   This is the "user" section.
 *     |  |
 *     +--+ @aperture_size - @kernel_reserved
 *     |  |
 *     ~  ~   This is the "kernel" section.
 *     |  |
 *     +--+ @aperture_size
 *
 * The user section is therefor what ever is left over after the @low_hole and
 * @kernel_reserved memory have been portioned out. The @kernel_reserved is
 * always persent at the top of the memory space and the @low_hole is always at
 * the bottom.
 *
 * For certain address spaces a "user" section makes no sense (bar1, etc) so in
 * such cases the @kernel_reserved and @low_hole should sum to exactly
 * @aperture_size.
 */
struct vm_gk20a *nvgpu_vm_init(struct gk20a *g,
			       u32 big_page_size,
			       u64 low_hole,
			       u64 kernel_reserved,
			       u64 aperture_size,
			       bool big_pages,
			       bool userspace_managed,
			       char *name)
{
	struct vm_gk20a *vm = nvgpu_kzalloc(g, sizeof(*vm));

	if (!vm)
		return NULL;

	if (__nvgpu_vm_init(&g->mm, vm, big_page_size, low_hole,
			    kernel_reserved, aperture_size, big_pages,
			    userspace_managed, name)) {
		nvgpu_kfree(g, vm);
		return NULL;
	}

	return vm;
}

/*
 * Cleanup the VM!
 */
static void __nvgpu_vm_remove(struct vm_gk20a *vm)
{
	struct nvgpu_mapped_buf *mapped_buffer;
	struct nvgpu_vm_area *vm_area, *vm_area_tmp;
	struct nvgpu_rbtree_node *node = NULL;
	struct gk20a *g = vm->mm->g;

	/*
	 * Do this outside of the update_gmmu_lock since unmapping the semaphore
	 * pool involves unmapping a GMMU mapping which means aquiring the
	 * update_gmmu_lock.
	 */
	if (!(g->gpu_characteristics.flags & NVGPU_GPU_FLAGS_HAS_SYNCPOINTS)) {
		if (vm->sema_pool) {
			nvgpu_semaphore_pool_unmap(vm->sema_pool, vm);
			nvgpu_semaphore_pool_put(vm->sema_pool);
		}
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	nvgpu_rbtree_enum_start(0, &node, vm->mapped_buffers);
	while (node) {
		mapped_buffer = mapped_buffer_from_rbtree_node(node);
		nvgpu_vm_unmap_locked(mapped_buffer, NULL);
		nvgpu_rbtree_enum_start(0, &node, vm->mapped_buffers);
	}

	/* destroy remaining reserved memory areas */
	nvgpu_list_for_each_entry_safe(vm_area, vm_area_tmp,
			&vm->vm_area_list,
			nvgpu_vm_area, vm_area_list) {
		nvgpu_list_del(&vm_area->vm_area_list);
		nvgpu_kfree(vm->mm->g, vm_area);
	}

	if (nvgpu_alloc_initialized(&vm->kernel))
		nvgpu_alloc_destroy(&vm->kernel);
	if (nvgpu_alloc_initialized(&vm->user))
		nvgpu_alloc_destroy(&vm->user);
	if (nvgpu_alloc_initialized(&vm->user_lp))
		nvgpu_alloc_destroy(&vm->user_lp);

	nvgpu_vm_free_entries(vm, &vm->pdb);

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	if (g->is_virtual)
		vgpu_vm_remove(vm);
#endif

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	nvgpu_kfree(g, vm);
}

static void __nvgpu_vm_remove_ref(struct nvgpu_ref *ref)
{
	struct vm_gk20a *vm = container_of(ref, struct vm_gk20a, ref);

	__nvgpu_vm_remove(vm);
}

void nvgpu_vm_get(struct vm_gk20a *vm)
{
	nvgpu_ref_get(&vm->ref);
}

void nvgpu_vm_put(struct vm_gk20a *vm)
{
	nvgpu_ref_put(&vm->ref, __nvgpu_vm_remove_ref);
}

int nvgpu_insert_mapped_buf(struct vm_gk20a *vm,
			    struct nvgpu_mapped_buf *mapped_buffer)
{
	mapped_buffer->node.key_start = mapped_buffer->addr;
	mapped_buffer->node.key_end = mapped_buffer->addr + mapped_buffer->size;

	nvgpu_rbtree_insert(&mapped_buffer->node, &vm->mapped_buffers);

	return 0;
}

void nvgpu_remove_mapped_buf(struct vm_gk20a *vm,
			     struct nvgpu_mapped_buf *mapped_buffer)
{
	nvgpu_rbtree_unlink(&mapped_buffer->node, &vm->mapped_buffers);
}

struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf(
	struct vm_gk20a *vm, u64 addr)
{
	struct nvgpu_rbtree_node *node = NULL;
	struct nvgpu_rbtree_node *root = vm->mapped_buffers;

	nvgpu_rbtree_search(addr, &node, root);
	if (!node)
		return NULL;

	return mapped_buffer_from_rbtree_node(node);
}

struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf_range(
	struct vm_gk20a *vm, u64 addr)
{
	struct nvgpu_rbtree_node *node = NULL;
	struct nvgpu_rbtree_node *root = vm->mapped_buffers;

	nvgpu_rbtree_range_search(addr, &node, root);
	if (!node)
		return NULL;

	return mapped_buffer_from_rbtree_node(node);
}

struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf_less_than(
	struct vm_gk20a *vm, u64 addr)
{
	struct nvgpu_rbtree_node *node = NULL;
	struct nvgpu_rbtree_node *root = vm->mapped_buffers;

	nvgpu_rbtree_less_than_search(addr, &node, root);
	if (!node)
		return NULL;

	return mapped_buffer_from_rbtree_node(node);
}
