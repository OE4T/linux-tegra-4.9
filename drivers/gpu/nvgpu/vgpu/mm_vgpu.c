/*
 * Virtualized GPU Memory Management
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/dma-mapping.h>

#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/bug.h>
#include <nvgpu/vm.h>
#include <nvgpu/vm_area.h>

#include <nvgpu/vgpu/vm.h>

#include <nvgpu/linux/nvgpu_mem.h>

#include "vgpu/vgpu.h"
#include "gk20a/mm_gk20a.h"
#include "gm20b/mm_gm20b.h"

#include "common/linux/vm_priv.h"

static int vgpu_init_mm_setup_sw(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;

	gk20a_dbg_fn("");

	if (mm->sw_ready) {
		gk20a_dbg_fn("skip init");
		return 0;
	}

	nvgpu_mutex_init(&mm->tlb_lock);
	nvgpu_mutex_init(&mm->priv_lock);

	mm->g = g;

	/*TBD: make channel vm size configurable */
	mm->channel.user_size = NV_MM_DEFAULT_USER_SIZE;
	mm->channel.kernel_size = NV_MM_DEFAULT_KERNEL_SIZE;

	gk20a_dbg_info("channel vm size: user %dMB  kernel %dMB",
		       (int)(mm->channel.user_size >> 20),
		       (int)(mm->channel.kernel_size >> 20));

	mm->sw_ready = true;

	return 0;
}

int vgpu_init_mm_support(struct gk20a *g)
{
	int err;

	gk20a_dbg_fn("");

	err = vgpu_init_mm_setup_sw(g);
	if (err)
		return err;

	if (g->ops.mm.init_mm_setup_hw)
		err = g->ops.mm.init_mm_setup_hw(g);

	return err;
}

static u64 vgpu_locked_gmmu_map(struct vm_gk20a *vm,
				u64 map_offset,
				struct nvgpu_sgt *sgt,
				u64 buffer_offset,
				u64 size,
				int pgsz_idx,
				u8 kind_v,
				u32 ctag_offset,
				u32 flags,
				int rw_flag,
				bool clear_ctags,
				bool sparse,
				bool priv,
				struct vm_gk20a_mapping_batch *batch,
				enum nvgpu_aperture aperture)
{
	int err = 0;
	struct device *d = dev_from_vm(vm);
	struct gk20a *g = gk20a_from_vm(vm);
	struct dma_iommu_mapping *mapping = to_dma_iommu_mapping(d);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_map_params *p = &msg.params.as_map;
	u64 addr = nvgpu_sgt_get_gpu_addr(sgt, g, sgt->sgl, NULL);
	u8 prot;

	gk20a_dbg_fn("");

	/* Allocate (or validate when map_offset != 0) the virtual address. */
	if (!map_offset) {
		map_offset = __nvgpu_vm_alloc_va(vm, size,
					  pgsz_idx);
		if (!map_offset) {
			nvgpu_err(g, "failed to allocate va space");
			err = -ENOMEM;
			goto fail;
		}
	}

	if (rw_flag == gk20a_mem_flag_read_only)
		prot = TEGRA_VGPU_MAP_PROT_READ_ONLY;
	else if (rw_flag == gk20a_mem_flag_write_only)
		prot = TEGRA_VGPU_MAP_PROT_WRITE_ONLY;
	else
		prot = TEGRA_VGPU_MAP_PROT_NONE;

	msg.cmd = TEGRA_VGPU_CMD_AS_MAP;
	msg.handle = vgpu_get_handle(g);
	p->handle = vm->handle;
	p->addr = addr;
	p->gpu_va = map_offset;
	p->size = size;
	if (pgsz_idx == gmmu_page_size_kernel) {
		u32 page_size = vm->gmmu_page_sizes[pgsz_idx];

		if (page_size == vm->gmmu_page_sizes[gmmu_page_size_small]) {
			pgsz_idx = gmmu_page_size_small;
		} else if (page_size ==
				vm->gmmu_page_sizes[gmmu_page_size_big]) {
			pgsz_idx = gmmu_page_size_big;
		} else {
			nvgpu_err(g, "invalid kernel page size %d",
				page_size);
			goto fail;
		}
	}
	p->pgsz_idx = pgsz_idx;
	p->iova = mapping ? 1 : 0;
	p->kind = kind_v;
	p->cacheable =
		(flags & NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_TRUE) ? 1 : 0;
	p->prot = prot;
	p->ctag_offset = ctag_offset;
	p->clear_ctags = clear_ctags;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err)
		goto fail;

	/* TLB invalidate handled on server side */

	return map_offset;
fail:
	nvgpu_err(g, "%s: failed with err=%d", __func__, err);
	return 0;
}

static void vgpu_locked_gmmu_unmap(struct vm_gk20a *vm,
				u64 vaddr,
				u64 size,
				int pgsz_idx,
				bool va_allocated,
				int rw_flag,
				bool sparse,
				struct vm_gk20a_mapping_batch *batch)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_map_params *p = &msg.params.as_map;
	int err;

	gk20a_dbg_fn("");

	if (va_allocated) {
		err = __nvgpu_vm_free_va(vm, vaddr, pgsz_idx);
		if (err) {
			dev_err(dev_from_vm(vm),
				"failed to free va");
			return;
		}
	}

	msg.cmd = TEGRA_VGPU_CMD_AS_UNMAP;
	msg.handle = vgpu_get_handle(g);
	p->handle = vm->handle;
	p->gpu_va = vaddr;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		dev_err(dev_from_vm(vm),
			"failed to update gmmu ptes on unmap");

	/* TLB invalidate handled on server side */
}

/*
 * This is called by the common VM init routine to handle vGPU specifics of
 * intializing a VM on a vGPU. This alone is not enough to init a VM. See
 * nvgpu_vm_init().
 */
int vgpu_vm_init(struct gk20a *g, struct vm_gk20a *vm)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_share_params *p = &msg.params.as_share;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_AS_ALLOC_SHARE;
	msg.handle = vgpu_get_handle(g);
	p->size = vm->va_limit;
	p->big_page_size = vm->big_page_size;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		return -ENOMEM;

	vm->handle = p->handle;

	return 0;
}

/*
 * Similar to vgpu_vm_init() this is called as part of the cleanup path for
 * VMs. This alone is not enough to remove a VM - see nvgpu_vm_remove().
 */
void vgpu_vm_remove(struct vm_gk20a *vm)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_share_params *p = &msg.params.as_share;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_AS_FREE_SHARE;
	msg.handle = vgpu_get_handle(g);
	p->handle = vm->handle;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

u64 vgpu_bar1_map(struct gk20a *g, struct sg_table **sgt, u64 size)
{
	struct dma_iommu_mapping *mapping =
			to_dma_iommu_mapping(dev_from_gk20a(g));
	u64 addr = nvgpu_mem_get_addr_sgl(g, (*sgt)->sgl);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_map_params *p = &msg.params.as_map;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_MAP_BAR1;
	msg.handle = vgpu_get_handle(g);
	p->addr = addr;
	p->size = size;
	p->iova = mapping ? 1 : 0;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		addr = 0;
	else
		addr = p->gpu_va;

	return addr;
}

static int vgpu_vm_bind_channel(struct gk20a_as_share *as_share,
				struct channel_gk20a *ch)
{
	struct vm_gk20a *vm = as_share->vm;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_bind_share_params *p = &msg.params.as_bind_share;
	int err;

	gk20a_dbg_fn("");

	ch->vm = vm;
	msg.cmd = TEGRA_VGPU_CMD_AS_BIND_SHARE;
	msg.handle = vgpu_get_handle(ch->g);
	p->as_handle = vm->handle;
	p->chan_handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	if (err || msg.ret) {
		ch->vm = NULL;
		err = -ENOMEM;
	}

	if (ch->vm)
		nvgpu_vm_get(ch->vm);

	return err;
}

static void vgpu_cache_maint(u64 handle, u8 op)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_cache_maint_params *p = &msg.params.cache_maint;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_CACHE_MAINT;
	msg.handle = handle;
	p->op = op;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

static int vgpu_mm_fb_flush(struct gk20a *g)
{

	gk20a_dbg_fn("");

	vgpu_cache_maint(vgpu_get_handle(g), TEGRA_VGPU_FB_FLUSH);
	return 0;
}

static void vgpu_mm_l2_invalidate(struct gk20a *g)
{

	gk20a_dbg_fn("");

	vgpu_cache_maint(vgpu_get_handle(g), TEGRA_VGPU_L2_MAINT_INV);
}

static void vgpu_mm_l2_flush(struct gk20a *g, bool invalidate)
{
	u8 op;

	gk20a_dbg_fn("");

	if (invalidate)
		op = TEGRA_VGPU_L2_MAINT_FLUSH_INV;
	else
		op =  TEGRA_VGPU_L2_MAINT_FLUSH;

	vgpu_cache_maint(vgpu_get_handle(g), op);
}

static void vgpu_mm_tlb_invalidate(struct gk20a *g, struct nvgpu_mem *pdb)
{
	gk20a_dbg_fn("");

	nvgpu_err(g, "call to RM server not supported");
}

static void vgpu_mm_mmu_set_debug_mode(struct gk20a *g, bool enable)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_mmu_debug_mode *p = &msg.params.mmu_debug_mode;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_SET_MMU_DEBUG_MODE;
	msg.handle = vgpu_get_handle(g);
	p->enable = (u32)enable;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

void vgpu_init_mm_ops(struct gpu_ops *gops)
{
	gops->fb.is_debug_mode_enabled = NULL;
	gops->fb.set_debug_mode = vgpu_mm_mmu_set_debug_mode;
	gops->mm.gmmu_map = vgpu_locked_gmmu_map;
	gops->mm.gmmu_unmap = vgpu_locked_gmmu_unmap;
	gops->mm.vm_bind_channel = vgpu_vm_bind_channel;
	gops->mm.fb_flush = vgpu_mm_fb_flush;
	gops->mm.l2_invalidate = vgpu_mm_l2_invalidate;
	gops->mm.l2_flush = vgpu_mm_l2_flush;
	gops->fb.tlb_invalidate = vgpu_mm_tlb_invalidate;
	gops->mm.get_physical_addr_bits = gk20a_mm_get_physical_addr_bits;
	gops->mm.gpu_phys_addr = gm20b_gpu_phys_addr;
	gops->mm.init_mm_setup_hw = NULL;
}
