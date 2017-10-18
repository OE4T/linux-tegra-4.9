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
#include "gk20a/kind_gk20a.h"

static void __nvgpu_vm_unmap(struct nvgpu_mapped_buf *mapped_buffer,
			     struct vm_gk20a_mapping_batch *batch);

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
	if (nvgpu_is_enabled(g, NVGPU_HAS_SYNCPOINTS))
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
	if (!nvgpu_is_enabled(g, NVGPU_HAS_SYNCPOINTS)) {
		if (vm->sema_pool) {
			nvgpu_semaphore_pool_unmap(vm->sema_pool, vm);
			nvgpu_semaphore_pool_put(vm->sema_pool);
		}
	}

#if defined(CONFIG_TEGRA_GK20A_NVHOST) && defined(CONFIG_TEGRA_19x_GPU)
	if (nvgpu_mem_is_valid(&g->syncpt_mem) && vm->syncpt_ro_map_gpu_va)
		nvgpu_gmmu_unmap(vm, &g->syncpt_mem,
				vm->syncpt_ro_map_gpu_va);
#endif

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	nvgpu_rbtree_enum_start(0, &node, vm->mapped_buffers);
	while (node) {
		mapped_buffer = mapped_buffer_from_rbtree_node(node);
		__nvgpu_vm_unmap(mapped_buffer, NULL);
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

int nvgpu_vm_get_buffers(struct vm_gk20a *vm,
			 struct nvgpu_mapped_buf ***mapped_buffers,
			 int *num_buffers)
{
	struct nvgpu_mapped_buf *mapped_buffer;
	struct nvgpu_mapped_buf **buffer_list;
	struct nvgpu_rbtree_node *node = NULL;
	int i = 0;

	if (vm->userspace_managed) {
		*mapped_buffers = NULL;
		*num_buffers = 0;
		return 0;
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	buffer_list = nvgpu_big_zalloc(vm->mm->g, sizeof(*buffer_list) *
				       vm->num_user_mapped_buffers);
	if (!buffer_list) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		return -ENOMEM;
	}

	nvgpu_rbtree_enum_start(0, &node, vm->mapped_buffers);
	while (node) {
		mapped_buffer = mapped_buffer_from_rbtree_node(node);
		buffer_list[i] = mapped_buffer;
		nvgpu_ref_get(&mapped_buffer->ref);
		i++;
		nvgpu_rbtree_enum_next(&node, node);
	}

	BUG_ON(i != vm->num_user_mapped_buffers);

	*num_buffers = vm->num_user_mapped_buffers;
	*mapped_buffers = buffer_list;

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return 0;
}

void nvgpu_vm_put_buffers(struct vm_gk20a *vm,
				 struct nvgpu_mapped_buf **mapped_buffers,
				 int num_buffers)
{
	int i;
	struct vm_gk20a_mapping_batch batch;

	if (num_buffers == 0)
		return;

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	nvgpu_vm_mapping_batch_start(&batch);
	vm->kref_put_batch = &batch;

	for (i = 0; i < num_buffers; ++i)
		nvgpu_ref_put(&mapped_buffers[i]->ref, __nvgpu_vm_unmap_ref);

	vm->kref_put_batch = NULL;
	nvgpu_vm_mapping_batch_finish_locked(vm, &batch);
	nvgpu_mutex_release(&vm->update_gmmu_lock);

	nvgpu_big_free(vm->mm->g, mapped_buffers);
}

/*
 * Really unmap. This does the real GMMU unmap and removes the mapping from the
 * VM map tracking tree (and vm_area list if necessary).
 */
static void __nvgpu_vm_unmap(struct nvgpu_mapped_buf *mapped_buffer,
			     struct vm_gk20a_mapping_batch *batch)
{
	struct vm_gk20a *vm = mapped_buffer->vm;
	struct gk20a *g = vm->mm->g;

	vm->num_user_mapped_buffers--;

	g->ops.mm.gmmu_unmap(vm,
			     mapped_buffer->addr,
			     mapped_buffer->size,
			     mapped_buffer->pgsz_idx,
			     mapped_buffer->va_allocated,
			     gk20a_mem_flag_none,
			     mapped_buffer->vm_area ?
			     mapped_buffer->vm_area->sparse : false,
			     batch);

	/*
	 * Remove from mapped buffer tree. Then delete the buffer from the
	 * linked list of mapped buffers; though note: not all mapped buffers
	 * are part of a vm_area.
	 */
	nvgpu_remove_mapped_buf(vm, mapped_buffer);
	nvgpu_list_del(&mapped_buffer->buffer_list);

	/*
	 * OS specific freeing. This is after the generic freeing incase the
	 * generic freeing relies on some component of the OS specific
	 * nvgpu_mapped_buf in some abstraction or the like.
	 */
	nvgpu_vm_unmap_system(mapped_buffer);

	nvgpu_kfree(g, mapped_buffer);
}

void __nvgpu_vm_unmap_ref(struct nvgpu_ref *ref)
{
	struct nvgpu_mapped_buf *mapped_buffer =
		container_of(ref, struct nvgpu_mapped_buf, ref);

	__nvgpu_vm_unmap(mapped_buffer, mapped_buffer->vm->kref_put_batch);
}

/*
 * For fixed-offset buffers we must sync the buffer. That means we wait for the
 * buffer to hit a ref-count of 1 before proceeding.
 *
 * Note: this requires the update_gmmu_lock to be held since we release it and
 * re-aquire it in this function.
 */
static int nvgpu_vm_unmap_sync_buffer(struct vm_gk20a *vm,
				      struct nvgpu_mapped_buf *mapped_buffer)
{
	struct nvgpu_timeout timeout;
	int ret = 0;

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	/*
	 * 500ms second timer.
	 */
	nvgpu_timeout_init(vm->mm->g, &timeout, 50, NVGPU_TIMER_CPU_TIMER);

	do {
		if (nvgpu_atomic_read(&mapped_buffer->ref.refcount) == 1)
			break;
		nvgpu_msleep(10);
	} while (!nvgpu_timeout_expired_msg(&timeout,
					    "sync-unmap failed on 0x%llx"));

	if (nvgpu_timeout_expired(&timeout))
		ret = -ETIMEDOUT;

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	return ret;
}

void nvgpu_vm_unmap(struct vm_gk20a *vm, u64 offset,
		    struct vm_gk20a_mapping_batch *batch)
{
	struct nvgpu_mapped_buf *mapped_buffer;

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	mapped_buffer = __nvgpu_vm_find_mapped_buf(vm, offset);
	if (!mapped_buffer)
		goto done;

	if (mapped_buffer->flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET) {
		if (nvgpu_vm_unmap_sync_buffer(vm, mapped_buffer))
			/*
			 * Looks like we have failed... Better not continue in
			 * case the buffer is in use.
			 */
			goto done;
	}

	/*
	 * Make sure we have access to the batch if we end up calling through to
	 * the unmap_ref function.
	 */
	vm->kref_put_batch = batch;
	nvgpu_ref_put(&mapped_buffer->ref, __nvgpu_vm_unmap_ref);
	vm->kref_put_batch = NULL;

done:
	nvgpu_mutex_release(&vm->update_gmmu_lock);
	return;
}

int nvgpu_vm_init_kind_info(struct nvgpu_ctag_buffer_info *binfo,
			    s16 compr_kind, s16 incompr_kind)
{
	if (binfo->flags & NVGPU_AS_MAP_BUFFER_FLAGS_DIRECT_KIND_CTRL) {
		/* were we supplied with a kind in either parameter? */
		if ((compr_kind < 0 || compr_kind >= NV_KIND_ATTR_SIZE) &&
		    (incompr_kind < 0 || incompr_kind >= NV_KIND_ATTR_SIZE))
			return -EINVAL;

		if (compr_kind != NV_KIND_INVALID) {
			binfo->use_kind_v = true;
			binfo->kind_v = (u8)compr_kind;
		}

		if (incompr_kind != NV_KIND_INVALID) {
			binfo->use_uc_kind_v = true;
			binfo->uc_kind_v = (u8)incompr_kind;
		}
	} else {
		if (compr_kind < 0 || compr_kind >= NV_KIND_ATTR_SIZE)
			return -EINVAL;

		binfo->use_kind_v = true;
		binfo->kind_v = (u8)compr_kind;

		/*
		 * Note: nvgpu_vm_kind_and_compression() will figure out
		 * uc_kind_v or return an error.
		 */
	}

	return 0;
}

static int nvgpu_vm_setup_kind_legacy(struct vm_gk20a *vm,
				      struct nvgpu_ctag_buffer_info *binfo,
				      bool *pkind_compressible)
{
	struct gk20a *g = gk20a_from_vm(vm);
	bool kind_compressible;

	if (unlikely(binfo->kind_v == g->ops.mm.get_kind_invalid()))
		binfo->kind_v = g->ops.mm.get_kind_pitch();

	if (unlikely(!gk20a_kind_is_supported(binfo->kind_v))) {
		nvgpu_err(g, "kind 0x%x not supported", binfo->kind_v);
		return -EINVAL;
	}

	binfo->uc_kind_v = g->ops.mm.get_kind_invalid();

	/*
	 * Find a suitable incompressible kind if it becomes necessary later.
	 */
	kind_compressible = gk20a_kind_is_compressible(binfo->kind_v);
	if (kind_compressible) {
		binfo->uc_kind_v = gk20a_get_uncompressed_kind(binfo->kind_v);
		if (binfo->uc_kind_v == g->ops.mm.get_kind_invalid()) {
			/*
			 * Shouldn't happen, but it is worth cross-checking.
			 */
			nvgpu_err(g, "comptag kind 0x%x can't be"
				  " downgraded to uncompressed kind",
				  binfo->kind_v);
			return -EINVAL;
		}
	}

	*pkind_compressible = kind_compressible;

	return 0;
}

int nvgpu_vm_compute_kind_and_compression(struct vm_gk20a *vm,
					  struct nvgpu_ctag_buffer_info *binfo)
{
	bool kind_compressible;
	struct gk20a *g = gk20a_from_vm(vm);
	int ctag_granularity = g->ops.fb.compression_page_size(g);

	if (!binfo->use_kind_v)
		binfo->kind_v = g->ops.mm.get_kind_invalid();
	if (!binfo->use_uc_kind_v)
		binfo->uc_kind_v = g->ops.mm.get_kind_invalid();

	if (binfo->flags & NVGPU_AS_MAP_BUFFER_FLAGS_DIRECT_KIND_CTRL) {
		kind_compressible = (binfo->kind_v !=
				     g->ops.mm.get_kind_invalid());
		if (!kind_compressible)
			binfo->kind_v = binfo->uc_kind_v;
	} else {
		int err = nvgpu_vm_setup_kind_legacy(vm, binfo,
						     &kind_compressible);

		if (err)
			return err;
	}

	/* comptags only supported for suitable kinds, 128KB pagesize */
	if (kind_compressible &&
	    vm->gmmu_page_sizes[binfo->pgsz_idx] <
	    g->ops.fb.compressible_page_size(g)) {
		/* it is safe to fall back to uncompressed as
		   functionality is not harmed */
		binfo->kind_v = binfo->uc_kind_v;
		kind_compressible = false;
	}

	if (kind_compressible)
		binfo->ctag_lines = DIV_ROUND_UP_ULL(binfo->size,
						     ctag_granularity);
	else
		binfo->ctag_lines = 0;

	binfo->use_kind_v = (binfo->kind_v != g->ops.mm.get_kind_invalid());
	binfo->use_uc_kind_v = (binfo->uc_kind_v !=
				g->ops.mm.get_kind_invalid());

	return 0;
}
