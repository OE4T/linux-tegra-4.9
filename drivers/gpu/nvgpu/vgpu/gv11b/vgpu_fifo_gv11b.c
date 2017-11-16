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

#include <gk20a/gk20a.h>

#include "vgpu/vgpu.h"
#include "gv11b/fifo_gv11b.h"
#include <nvgpu/nvhost_t19x.h>

#include <linux/tegra_vgpu.h>

#ifdef CONFIG_TEGRA_GK20A_NVHOST
int vgpu_gv11b_fifo_alloc_syncpt_buf(struct channel_gk20a *c,
				u32 syncpt_id, struct nvgpu_mem *syncpt_buf)
{
	int err;
	struct gk20a *g = c->g;
	struct vm_gk20a *vm = c->vm;
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_map_syncpt_params *p = &msg.params.t19x.map_syncpt;

	/*
	 * Add ro map for complete sync point shim range in vm.
	 * All channels sharing same vm will share same ro mapping.
	 * Create rw map for current channel sync point.
	 */
	if (!vm->syncpt_ro_map_gpu_va) {
		vm->syncpt_ro_map_gpu_va = __nvgpu_vm_alloc_va(vm,
				g->syncpt_unit_size,
				gmmu_page_size_kernel);
		if (!vm->syncpt_ro_map_gpu_va) {
			nvgpu_err(g, "allocating read-only va space failed");
			return -ENOMEM;
		}

		msg.cmd = TEGRA_VGPU_CMD_MAP_SYNCPT;
		msg.handle = vgpu_get_handle(g);
		p->as_handle = c->vm->handle;
		p->gpu_va = vm->syncpt_ro_map_gpu_va;
		p->len = g->syncpt_unit_size;
		p->offset = 0;
		p->prot = TEGRA_VGPU_MAP_PROT_READ_ONLY;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		err = err ? err : msg.ret;
		if (err) {
			nvgpu_err(g,
				"mapping read-only va space failed err %d",
				err);
			__nvgpu_vm_free_va(c->vm, vm->syncpt_ro_map_gpu_va,
					gmmu_page_size_kernel);
			vm->syncpt_ro_map_gpu_va = 0;
			return err;
		}
	}

	syncpt_buf->gpu_va = __nvgpu_vm_alloc_va(c->vm, g->syncpt_size,
			gmmu_page_size_kernel);
	if (!syncpt_buf->gpu_va) {
		nvgpu_err(g, "allocating syncpt va space failed");
		return -ENOMEM;
	}

	msg.cmd = TEGRA_VGPU_CMD_MAP_SYNCPT;
	msg.handle = vgpu_get_handle(g);
	p->as_handle = c->vm->handle;
	p->gpu_va = syncpt_buf->gpu_va;
	p->len = g->syncpt_size;
	p->offset =
		nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(syncpt_id);
	p->prot = TEGRA_VGPU_MAP_PROT_NONE;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(g, "mapping syncpt va space failed err %d", err);
		__nvgpu_vm_free_va(c->vm, syncpt_buf->gpu_va,
				gmmu_page_size_kernel);
		return err;
	}

	return 0;
}
#endif /* CONFIG_TEGRA_GK20A_NVHOST */

int vgpu_gv11b_init_fifo_setup_hw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	int err;

	err = vgpu_get_attribute(vgpu_get_handle(g),
			TEGRA_VGPU_ATTRIB_MAX_SUBCTX_COUNT,
			&f->t19x.max_subctx_count);
	if (err) {
		nvgpu_err(g, "get max_subctx_count failed %d", err);
		return err;
	}

	return 0;
}
