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

#include <nvgpu/vm.h>
#include <nvgpu/vm_area.h>
#include <nvgpu/lock.h>
#include <nvgpu/list.h>
#include <nvgpu/rbtree.h>
#include <nvgpu/semaphore.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"

int vm_aspace_id(struct vm_gk20a *vm)
{
	return vm->as_share ? vm->as_share->id : -1;
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
		g->ops.fb.tlb_invalidate(g, &vm->pdb.mem);
	}
}

void nvgpu_vm_mapping_batch_finish(struct vm_gk20a *vm,
				   struct vm_gk20a_mapping_batch *mapping_batch)
{
	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	nvgpu_vm_mapping_batch_finish_locked(vm, mapping_batch);
	nvgpu_mutex_release(&vm->update_gmmu_lock);
}

void nvgpu_vm_remove_support_nofree(struct vm_gk20a *vm)
{
	struct nvgpu_mapped_buf *mapped_buffer;
	struct nvgpu_vm_area *vm_area, *vm_area_tmp;
	struct nvgpu_rbtree_node *node = NULL;
	struct gk20a *g = vm->mm->g;

	gk20a_dbg_fn("");

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

	nvgpu_deinit_vm(vm);

	nvgpu_mutex_release(&vm->update_gmmu_lock);
}

void nvgpu_vm_remove_support(struct vm_gk20a *vm)
{
	nvgpu_vm_remove_support_nofree(vm);
	/* vm is not used anymore. release it. */
	nvgpu_kfree(vm->mm->g, vm);
}

static void nvgpu_vm_remove_support_kref(struct kref *ref)
{
	struct vm_gk20a *vm = container_of(ref, struct vm_gk20a, ref);
	struct gk20a *g = gk20a_from_vm(vm);

	g->ops.mm.vm_remove(vm);
}

void nvgpu_vm_get(struct vm_gk20a *vm)
{
	kref_get(&vm->ref);
}

void nvgpu_vm_put(struct vm_gk20a *vm)
{
	kref_put(&vm->ref, nvgpu_vm_remove_support_kref);
}

void nvgpu_remove_vm(struct vm_gk20a *vm, struct nvgpu_mem *inst_block)
{
	struct gk20a *g = vm->mm->g;

	gk20a_dbg_fn("");

	gk20a_free_inst_block(g, inst_block);
	nvgpu_vm_remove_support_nofree(vm);
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
