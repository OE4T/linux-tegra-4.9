/*
 * Color decompression engine support
 *
 * Copyright (c) 2014, NVIDIA Corporation.  All rights reserved.
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

#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>

#include "gk20a.h"
#include "channel_gk20a.h"
#include "mm_gk20a.h"
#include "cde_gk20a.h"
#include "fence_gk20a.h"
#include "gr_gk20a.h"
#include "debug_gk20a.h"
#include "semaphore_gk20a.h"

#include "hw_ccsr_gk20a.h"
#include "hw_pbdma_gk20a.h"

static int gk20a_cde_load(struct gk20a_cde_ctx *cde_ctx);
static struct gk20a_cde_ctx *gk20a_cde_allocate_context(struct gk20a *g);

#define CTX_DELETE_TIME 1000

static void gk20a_deinit_cde_img(struct gk20a_cde_ctx *cde_ctx)
{
	struct device *dev = &cde_ctx->pdev->dev;
	int i;

	for (i = 0; i < cde_ctx->num_bufs; i++) {
		struct gk20a_cde_mem_desc *mem = cde_ctx->mem + i;
		gk20a_gmmu_unmap(cde_ctx->vm, mem->gpu_va, mem->num_bytes, 1);
		gk20a_free_sgtable(&mem->sgt);
		dma_free_writecombine(dev, mem->num_bytes, mem->cpuva,
							  mem->iova);
	}

	for (i = 0; i < cde_ctx->num_obj_ids; i++)
		gk20a_free_obj_ctx(cde_ctx->ch,
			&(struct nvgpu_free_obj_ctx_args)
			{ cde_ctx->obj_ids[i] });

	kfree(cde_ctx->init_cmd);
	kfree(cde_ctx->convert_cmd);

	cde_ctx->convert_cmd = NULL;
	cde_ctx->init_cmd = NULL;
	cde_ctx->num_bufs = 0;
	cde_ctx->num_obj_ids = 0;
	cde_ctx->num_params = 0;
	cde_ctx->init_cmd_num_entries = 0;
	cde_ctx->convert_cmd_num_entries = 0;
	cde_ctx->init_cmd_executed = false;
}

static void gk20a_cde_remove_ctx(struct gk20a_cde_ctx *cde_ctx)
__must_hold(&cde_app->mutex)
{
	struct gk20a *g = cde_ctx->g;
	struct channel_gk20a *ch = cde_ctx->ch;
	struct vm_gk20a *vm = ch->vm;

	/* free the channel */
	gk20a_free_channel(cde_ctx->ch, true);

	/* ..then release mapped memory */
	gk20a_deinit_cde_img(cde_ctx);
	gk20a_gmmu_unmap(vm, cde_ctx->backing_store_vaddr,
			 g->gr.compbit_store.size, 1);

	/* housekeeping on app */
	list_del(&cde_ctx->list);
	cde_ctx->g->cde_app.ctx_count--;
	kfree(cde_ctx);
}

static void gk20a_cde_prepare_ctx_remove(struct gk20a_cde_ctx *cde_ctx)
__releases(&cde_app->mutex)
__acquires(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &cde_ctx->g->cde_app;

	/* permanent contexts do not have deleter works */
	if (!cde_ctx->is_temporary)
		return;

	/* safe to go off the mutex since app is deinitialised. deleter works
	 * may be only at waiting for the mutex or before, going to abort */
	mutex_unlock(&cde_app->mutex);

	/* the deleter can rearm itself */
	do {
		cancel_delayed_work_sync(&cde_ctx->ctx_deleter_work);
	} while (delayed_work_pending(&cde_ctx->ctx_deleter_work));

	mutex_lock(&cde_app->mutex);
}

static void gk20a_cde_remove_contexts(struct gk20a *g)
__must_hold(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &g->cde_app;
	struct gk20a_cde_ctx *cde_ctx, *cde_ctx_save;

	list_for_each_entry_safe(cde_ctx, cde_ctx_save,
			&cde_app->free_contexts, list) {
		gk20a_cde_prepare_ctx_remove(cde_ctx);
		gk20a_cde_remove_ctx(cde_ctx);
	}

	list_for_each_entry_safe(cde_ctx, cde_ctx_save,
			&cde_app->used_contexts, list) {
		gk20a_cde_prepare_ctx_remove(cde_ctx);
		gk20a_cde_remove_ctx(cde_ctx);
	}
}

static void gk20a_cde_stop(struct gk20a *g)
__must_hold(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &g->cde_app;

	/* prevent further conversions and delayed works from working */
	cde_app->initialised = false;
	/* free all data, empty the list */
	gk20a_cde_remove_contexts(g);
}

void gk20a_cde_destroy(struct gk20a *g)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &g->cde_app;

	if (!cde_app->initialised)
		return;

	mutex_lock(&cde_app->mutex);
	gk20a_cde_stop(g);
	mutex_unlock(&cde_app->mutex);
}

void gk20a_cde_suspend(struct gk20a *g)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &g->cde_app;
	struct gk20a_cde_ctx *cde_ctx, *cde_ctx_save;

	if (!cde_app->initialised)
		return;

	mutex_lock(&cde_app->mutex);

	list_for_each_entry_safe(cde_ctx, cde_ctx_save,
			&cde_app->free_contexts, list) {
		if (cde_ctx->is_temporary)
			cancel_delayed_work(&cde_ctx->ctx_deleter_work);
	}

	list_for_each_entry_safe(cde_ctx, cde_ctx_save,
			&cde_app->used_contexts, list) {
		if (cde_ctx->is_temporary)
			cancel_delayed_work(&cde_ctx->ctx_deleter_work);
	}

	mutex_unlock(&cde_app->mutex);

}

static int gk20a_cde_create_context(struct gk20a *g)
__must_hold(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &g->cde_app;
	struct gk20a_cde_ctx *cde_ctx;

	cde_ctx = gk20a_cde_allocate_context(g);
	if (IS_ERR(cde_ctx))
		return PTR_ERR(cde_ctx);

	list_add(&cde_ctx->list, &cde_app->free_contexts);
	cde_app->ctx_count++;
	if (cde_app->ctx_count > cde_app->ctx_count_top)
		cde_app->ctx_count_top = cde_app->ctx_count;

	return 0;
}

static int gk20a_cde_create_contexts(struct gk20a *g)
__must_hold(&g->cde_app->mutex)
{
	int err;
	int i;

	for (i = 0; i < NUM_CDE_CONTEXTS; i++) {
		err = gk20a_cde_create_context(g);
		if (err)
			goto out;
	}

	return 0;
out:
	gk20a_cde_remove_contexts(g);
	return err;
}

static int gk20a_init_cde_buf(struct gk20a_cde_ctx *cde_ctx,
			      const struct firmware *img,
			      struct gk20a_cde_hdr_buf *buf)
{
	struct device *dev = &cde_ctx->pdev->dev;
	struct gk20a_cde_mem_desc *mem;
	int err;

	/* check that the file can hold the buf */
	if (buf->data_byte_offset != 0 &&
	    buf->data_byte_offset + buf->num_bytes > img->size) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: invalid data section. buffer idx = %d",
			   cde_ctx->num_bufs);
		return -EINVAL;
	}

	/* check that we have enough buf elems available */
	if (cde_ctx->num_bufs > MAX_CDE_BUFS) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: invalid data section. buffer idx = %d",
			   cde_ctx->num_bufs);
		return -ENOMEM;
	}

	/* allocate buf */
	mem = cde_ctx->mem + cde_ctx->num_bufs;
	mem->num_bytes = buf->num_bytes;
	mem->cpuva = dma_alloc_writecombine(dev, mem->num_bytes, &mem->iova,
					GFP_KERNEL);
	if (!mem->cpuva) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: could not allocate device memory. buffer idx = %d",
			   cde_ctx->num_bufs);
		return -ENOMEM;
	}

	err = gk20a_get_sgtable(dev, &mem->sgt, mem->cpuva, mem->iova,
				mem->num_bytes);
	if (err) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: could not get sg table. buffer idx = %d",
			   cde_ctx->num_bufs);
		err = -ENOMEM;
		goto err_get_sgtable;
	}

	mem->gpu_va = gk20a_gmmu_map(cde_ctx->vm, &mem->sgt, mem->num_bytes,
					0,
					gk20a_mem_flag_none);
	if (!mem->gpu_va) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: could not map buffer to gpuva. buffer idx = %d",
			   cde_ctx->num_bufs);
		err = -ENOMEM;
		goto err_map_buffer;
	}

	/* copy the content */
	if (buf->data_byte_offset != 0)
		memcpy(mem->cpuva, img->data + buf->data_byte_offset,
		       buf->num_bytes);

	cde_ctx->num_bufs++;

	return 0;

err_map_buffer:
	gk20a_free_sgtable(&mem->sgt);
	kfree(mem->sgt);
err_get_sgtable:
	dma_free_writecombine(dev, mem->num_bytes, &mem->cpuva, mem->iova);
	return err;
}

static int gk20a_replace_data(struct gk20a_cde_ctx *cde_ctx, void *target,
			      int type, s32 shift, u64 mask, u64 value)
{
	u32 *target_mem_ptr = target;
	u64 *target_mem_ptr_u64 = target;
	u64 current_value, new_value;

	value = (shift >= 0) ? value << shift : value >> -shift;
	value &= mask;

	/* read current data from the location */
	current_value = 0;
	if (type == TYPE_PARAM_TYPE_U32) {
		if (mask != 0xfffffffful)
			current_value = *target_mem_ptr;
	} else if (type == TYPE_PARAM_TYPE_U64_LITTLE) {
		if (mask != ~0ul)
			current_value = *target_mem_ptr_u64;
	} else if (type == TYPE_PARAM_TYPE_U64_BIG) {
		current_value = *target_mem_ptr_u64;
		current_value = (u64)(current_value >> 32) |
			(u64)(current_value << 32);
	} else {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: unknown type. type=%d",
			   type);
		return -EINVAL;
	}

	current_value &= ~mask;
	new_value = current_value | value;

	/* store the element data back */
	if (type == TYPE_PARAM_TYPE_U32)
		*target_mem_ptr = (u32)new_value;
	else if (type == TYPE_PARAM_TYPE_U64_LITTLE)
		*target_mem_ptr_u64 = new_value;
	else  {
		new_value = (u64)(new_value >> 32) |
			(u64)(new_value << 32);
		*target_mem_ptr_u64 = new_value;
	}

	return 0;
}

static int gk20a_init_cde_replace(struct gk20a_cde_ctx *cde_ctx,
				  const struct firmware *img,
				  struct gk20a_cde_hdr_replace *replace)
{
	struct gk20a_cde_mem_desc *source_mem;
	struct gk20a_cde_mem_desc *target_mem;
	u32 *target_mem_ptr;
	u64 vaddr;
	int err;

	if (replace->target_buf >= cde_ctx->num_bufs ||
	    replace->source_buf >= cde_ctx->num_bufs) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: invalid buffer. target_buf=%u, source_buf=%u, num_bufs=%d",
			   replace->target_buf, replace->source_buf,
			   cde_ctx->num_bufs);
		return -EINVAL;
	}

	source_mem = cde_ctx->mem + replace->source_buf;
	target_mem = cde_ctx->mem + replace->target_buf;
	target_mem_ptr = target_mem->cpuva;

	if (source_mem->num_bytes < (replace->source_byte_offset + 3) ||
	    target_mem->num_bytes < (replace->target_byte_offset + 3)) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: invalid buffer offsets. target_buf_offs=%lld, source_buf_offs=%lld, source_buf_size=%zu, dest_buf_size=%zu",
			   replace->target_byte_offset,
			   replace->source_byte_offset,
			 source_mem->num_bytes,
			 target_mem->num_bytes);
		return -EINVAL;
	}

	/* calculate the target pointer */
	target_mem_ptr += (replace->target_byte_offset / sizeof(u32));

	/* determine patch value */
	vaddr = source_mem->gpu_va + replace->source_byte_offset;
	err = gk20a_replace_data(cde_ctx, target_mem_ptr, replace->type,
				 replace->shift, replace->mask,
				 vaddr);
	if (err) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: replace failed. err=%d, target_buf=%u, target_buf_offs=%lld, source_buf=%u, source_buf_offs=%lld",
			   err, replace->target_buf,
			   replace->target_byte_offset,
			   replace->source_buf,
			   replace->source_byte_offset);
	}

	return err;
}

static int gk20a_cde_patch_params(struct gk20a_cde_ctx *cde_ctx)
{
	struct gk20a *g = cde_ctx->g;
	struct gk20a_cde_mem_desc *target_mem;
	u32 *target_mem_ptr;
	u64 new_data;
	int user_id = 0, i, err;

	for (i = 0; i < cde_ctx->num_params; i++) {
		struct gk20a_cde_hdr_param *param = cde_ctx->params + i;
		target_mem = cde_ctx->mem + param->target_buf;
		target_mem_ptr = target_mem->cpuva;
		target_mem_ptr += (param->target_byte_offset / sizeof(u32));

		switch (param->id) {
		case TYPE_PARAM_COMPTAGS_PER_CACHELINE:
			new_data = g->gr.comptags_per_cacheline;
			break;
		case TYPE_PARAM_GPU_CONFIGURATION:
			new_data = g->ltc_count * g->gr.slices_per_ltc *
				g->gr.cacheline_size;
			break;
		case TYPE_PARAM_FIRSTPAGEOFFSET:
			new_data = cde_ctx->surf_param_offset;
			break;
		case TYPE_PARAM_NUMPAGES:
			new_data = cde_ctx->surf_param_lines;
			break;
		case TYPE_PARAM_BACKINGSTORE:
			new_data = cde_ctx->backing_store_vaddr;
			break;
		case TYPE_PARAM_DESTINATION:
			new_data = cde_ctx->compbit_vaddr;
			break;
		case TYPE_PARAM_DESTINATION_SIZE:
			new_data = cde_ctx->compbit_size;
			break;
		case TYPE_PARAM_BACKINGSTORE_SIZE:
			new_data = g->gr.compbit_store.size;
			break;
		case TYPE_PARAM_SOURCE_SMMU_ADDR:
			new_data = gk20a_mm_gpuva_to_iova_base(cde_ctx->vm,
							cde_ctx->surf_vaddr);
			if (new_data == 0)
				err = -EINVAL;
			break;
		case TYPE_PARAM_BACKINGSTORE_BASE_HW:
			new_data = g->gr.compbit_store.base_hw;
			break;
		default:
			user_id = param->id - NUM_RESERVED_PARAMS;
			if (user_id < 0 || user_id >= MAX_CDE_USER_PARAMS)
				continue;
			new_data = cde_ctx->user_param_values[user_id];
		}

		gk20a_dbg(gpu_dbg_cde, "cde: patch: idx_in_file=%d	param_id=%d	target_buf=%u	target_byte_offset=%lld	data_value=0x%llx	data_offset/data_diff=%lld	data_type=%d	data_shift=%d	data_mask=0x%llx",
			  i, param->id, param->target_buf,
			  param->target_byte_offset, new_data,
			  param->data_offset, param->type, param->shift,
			  param->mask);

		new_data += param->data_offset;

		err = gk20a_replace_data(cde_ctx, target_mem_ptr, param->type,
					 param->shift, param->mask, new_data);

		if (err) {
			gk20a_warn(&cde_ctx->pdev->dev, "cde: patch failed. err=%d, idx=%d, id=%d, target_buf=%u, target_buf_offs=%lld, patch_value=%llu",
				   err, i, param->id, param->target_buf,
				   param->target_byte_offset, new_data);
			return err;
		}
	}

	return 0;
}

static int gk20a_init_cde_param(struct gk20a_cde_ctx *cde_ctx,
				const struct firmware *img,
				struct gk20a_cde_hdr_param *param)
{
	struct gk20a_cde_mem_desc *target_mem;

	if (param->target_buf >= cde_ctx->num_bufs) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: invalid buffer parameter. param idx = %d, target_buf=%u, num_bufs=%u",
			   cde_ctx->num_params, param->target_buf,
			   cde_ctx->num_bufs);
		return -EINVAL;
	}

	target_mem = cde_ctx->mem + param->target_buf;
	if (target_mem->num_bytes < (param->target_byte_offset + 3)) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: invalid buffer parameter. param idx = %d, target_buf_offs=%lld, target_buf_size=%zu",
			   cde_ctx->num_params, param->target_byte_offset,
			   target_mem->num_bytes);
		return -EINVAL;
	}

	/* does this parameter fit into our parameter structure */
	if (cde_ctx->num_params >= MAX_CDE_PARAMS) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: no room for new parameters param idx = %d",
			   cde_ctx->num_params);
		return -ENOMEM;
	}

	/* is the given id valid? */
	if (param->id >= NUM_RESERVED_PARAMS + MAX_CDE_USER_PARAMS) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: parameter id is not valid. param idx = %d, id=%u, max=%u",
			   param->id, cde_ctx->num_params,
			   NUM_RESERVED_PARAMS + MAX_CDE_USER_PARAMS);
		return -EINVAL;
	}

	cde_ctx->params[cde_ctx->num_params] = *param;
	cde_ctx->num_params++;

	return 0;
}

static int gk20a_init_cde_required_class(struct gk20a_cde_ctx *cde_ctx,
					 const struct firmware *img,
					 u32 required_class)
{
	struct nvgpu_alloc_obj_ctx_args alloc_obj_ctx;
	int err;

	if (cde_ctx->num_obj_ids >= MAX_CDE_OBJ_IDS) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: running out of class ids");
		return -ENOMEM;
	}

	alloc_obj_ctx.class_num = required_class;
	alloc_obj_ctx.padding = 0;

	err = gk20a_alloc_obj_ctx(cde_ctx->ch, &alloc_obj_ctx);
	if (err) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: failed to allocate ctx. err=%d",
			   err);
		return err;
	}

	cde_ctx->obj_ids[cde_ctx->num_obj_ids] = alloc_obj_ctx.obj_id;
	cde_ctx->num_obj_ids++;

	return 0;
}

static int gk20a_init_cde_command(struct gk20a_cde_ctx *cde_ctx,
				  const struct firmware *img,
				  u32 op,
				  struct gk20a_cde_cmd_elem *cmd_elem,
				  u32 num_elems)
{
	struct nvgpu_gpfifo **gpfifo, *gpfifo_elem;
	u32 *num_entries;
	int i;

	/* check command type */
	if (op == TYPE_BUF_COMMAND_INIT) {
		gpfifo = &cde_ctx->init_cmd;
		num_entries = &cde_ctx->init_cmd_num_entries;
	} else if (op == TYPE_BUF_COMMAND_CONVERT) {
		gpfifo = &cde_ctx->convert_cmd;
		num_entries = &cde_ctx->convert_cmd_num_entries;
	} else {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: unknown command. op=%u",
			   op);
		return -EINVAL;
	}

	/* allocate gpfifo entries to be pushed */
	*gpfifo = kzalloc(sizeof(struct nvgpu_gpfifo) * num_elems,
			  GFP_KERNEL);
	if (!*gpfifo) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: could not allocate memory for gpfifo entries");
		return -ENOMEM;
	}

	gpfifo_elem = *gpfifo;
	for (i = 0; i < num_elems; i++, cmd_elem++, gpfifo_elem++) {
		struct gk20a_cde_mem_desc *target_mem;

		/* validate the current entry */
		if (cmd_elem->target_buf >= cde_ctx->num_bufs) {
			gk20a_warn(&cde_ctx->pdev->dev, "cde: target buffer is not available (target=%u, num_bufs=%u)",
				   cmd_elem->target_buf, cde_ctx->num_bufs);
			return -EINVAL;
		}

		target_mem = cde_ctx->mem + cmd_elem->target_buf;
		if (target_mem->num_bytes <
		    cmd_elem->target_byte_offset + cmd_elem->num_bytes) {
			gk20a_warn(&cde_ctx->pdev->dev, "cde: target buffer cannot hold all entries (target_size=%zu, target_byte_offset=%lld, num_bytes=%llu)",
				   target_mem->num_bytes,
				   cmd_elem->target_byte_offset,
				   cmd_elem->num_bytes);
			return -EINVAL;
		}

		/* store the element into gpfifo */
		gpfifo_elem->entry0 =
			u64_lo32(target_mem->gpu_va +
			cmd_elem->target_byte_offset);
		gpfifo_elem->entry1 =
			u64_hi32(target_mem->gpu_va +
			cmd_elem->target_byte_offset) |
			pbdma_gp_entry1_length_f(cmd_elem->num_bytes /
						 sizeof(u32));
	}

	*num_entries = num_elems;
	return 0;
}

static int gk20a_init_cde_img(struct gk20a_cde_ctx *cde_ctx,
			      const struct firmware *img)
{
	struct gk20a_cde_app *cde_app = &cde_ctx->g->cde_app;
	u32 *data = (u32 *)img->data;
	u32 num_of_elems;
	struct gk20a_cde_hdr_elem *elem;
	u32 min_size = 0;
	int err = 0;
	int i;

	min_size += 2 * sizeof(u32);
	if (img->size < min_size) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: invalid image header");
		return -EINVAL;
	}

	cde_app->firmware_version = data[0];
	num_of_elems = data[1];

	min_size += num_of_elems * sizeof(*elem);
	if (img->size < min_size) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: bad image");
		return -EINVAL;
	}

	elem = (struct gk20a_cde_hdr_elem *)&data[2];
	for (i = 0; i < num_of_elems; i++) {
		int err = 0;
		switch (elem->type) {
		case TYPE_BUF:
			err = gk20a_init_cde_buf(cde_ctx, img, &elem->buf);
			break;
		case TYPE_REPLACE:
			err = gk20a_init_cde_replace(cde_ctx, img,
						     &elem->replace);
			break;
		case TYPE_PARAM:
			err = gk20a_init_cde_param(cde_ctx, img, &elem->param);
			break;
		case TYPE_REQUIRED_CLASS:
			err = gk20a_init_cde_required_class(cde_ctx, img,
				elem->required_class);
			break;
		case TYPE_COMMAND:
		{
			struct gk20a_cde_cmd_elem *cmd = (void *)
				&img->data[elem->command.data_byte_offset];
			err = gk20a_init_cde_command(cde_ctx, img,
				elem->command.op, cmd,
				elem->command.num_entries);
			break;
		}
		case TYPE_ARRAY:
			memcpy(&cde_app->arrays[elem->array.id][0],
				elem->array.data,
				MAX_CDE_ARRAY_ENTRIES*sizeof(u32));
			break;
		default:
			gk20a_warn(&cde_ctx->pdev->dev, "cde: unknown header element");
			err = -EINVAL;
		}

		if (err)
			goto deinit_image;

		elem++;
	}

	if (!cde_ctx->init_cmd || !cde_ctx->init_cmd_num_entries) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: convert command not defined");
		err = -EINVAL;
		goto deinit_image;
	}

	if (!cde_ctx->convert_cmd || !cde_ctx->convert_cmd_num_entries) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: convert command not defined");
		err = -EINVAL;
		goto deinit_image;
	}

	return 0;

deinit_image:
	gk20a_deinit_cde_img(cde_ctx);
	return err;
}

static int gk20a_cde_execute_buffer(struct gk20a_cde_ctx *cde_ctx,
				    u32 op, struct nvgpu_fence *fence,
				    u32 flags, struct gk20a_fence **fence_out)
{
	struct nvgpu_gpfifo *gpfifo = NULL;
	int num_entries = 0;

	/* check command type */
	if (op == TYPE_BUF_COMMAND_INIT) {
		gpfifo = cde_ctx->init_cmd;
		num_entries = cde_ctx->init_cmd_num_entries;
	} else if (op == TYPE_BUF_COMMAND_CONVERT) {
		gpfifo = cde_ctx->convert_cmd;
		num_entries = cde_ctx->convert_cmd_num_entries;
	} else {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: unknown buffer");
		return -EINVAL;
	}

	if (gpfifo == NULL || num_entries == 0) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: buffer not available");
		return -ENOSYS;
	}

	return gk20a_submit_channel_gpfifo(cde_ctx->ch, gpfifo,
					   num_entries, flags, fence, fence_out);
}

static void gk20a_ctx_release(struct gk20a_cde_ctx *cde_ctx)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &cde_ctx->g->cde_app;

	gk20a_dbg(gpu_dbg_cde_ctx, "releasing use on %p", cde_ctx);

	mutex_lock(&cde_app->mutex);

	cde_ctx->in_use = false;
	list_move(&cde_ctx->list, &cde_app->free_contexts);
	cde_app->ctx_usecount--;

	mutex_unlock(&cde_app->mutex);
}

static void gk20a_cde_ctx_deleter_fn(struct work_struct *work)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct delayed_work *delay_work = to_delayed_work(work);
	struct gk20a_cde_ctx *cde_ctx = container_of(delay_work,
			struct gk20a_cde_ctx, ctx_deleter_work);
	struct gk20a_cde_app *cde_app = &cde_ctx->g->cde_app;
	struct platform_device *pdev = cde_ctx->pdev;
	int err;

	/* someone has just taken it? engine deletion started? */
	if (cde_ctx->in_use || !cde_app->initialised)
		return;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_cde_ctx,
			"cde: attempting to delete temporary %p", cde_ctx);

	/* this should fail only when shutting down the whole device */
	err = gk20a_busy(pdev);
	if (WARN(err, "gk20a cde: cannot set gk20a on, not freeing channel yet."
				" rescheduling...")) {
		schedule_delayed_work(&cde_ctx->ctx_deleter_work,
			msecs_to_jiffies(CTX_DELETE_TIME));
		return;
	}

	/* mark so that nobody else assumes it's free to take */
	mutex_lock(&cde_app->mutex);
	if (cde_ctx->in_use || !cde_app->initialised) {
		gk20a_dbg(gpu_dbg_cde_ctx,
				"cde: context use raced, not deleting %p",
				cde_ctx);
		goto out;
	}
	cde_ctx->in_use = true;

	gk20a_cde_remove_ctx(cde_ctx);
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_cde_ctx,
			"cde: destroyed %p count=%d use=%d max=%d",
			cde_ctx, cde_app->ctx_count, cde_app->ctx_usecount,
			cde_app->ctx_count_top);

out:
	mutex_unlock(&cde_app->mutex);
	gk20a_idle(pdev);
}

static struct gk20a_cde_ctx *gk20a_cde_get_context(struct gk20a *g)
__must_hold(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &g->cde_app;
	struct gk20a_cde_ctx *cde_ctx;

	/* idle context available? */

	if (!list_empty(&cde_app->free_contexts)) {
		cde_ctx = list_first_entry(&cde_app->free_contexts,
				struct gk20a_cde_ctx, list);
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_cde_ctx,
				"cde: got free %p count=%d use=%d max=%d",
				cde_ctx, cde_app->ctx_count,
				cde_app->ctx_usecount,
				cde_app->ctx_count_top);
		/* deleter work may be scheduled, but in_use prevents it */
		cde_ctx->in_use = true;
		list_move(&cde_ctx->list, &cde_app->used_contexts);
		cde_app->ctx_usecount++;
		return cde_ctx;
	}

	/* no free contexts, get a temporary one */

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_cde_ctx,
			"cde: no free contexts, count=%d",
			cde_app->ctx_count);

	cde_ctx = gk20a_cde_allocate_context(g);
	if (IS_ERR(cde_ctx)) {
		gk20a_warn(&g->dev->dev, "cde: cannot allocate context: %ld",
				PTR_ERR(cde_ctx));
		return cde_ctx;
	}

	cde_ctx->in_use = true;
	cde_ctx->is_temporary = true;
	cde_app->ctx_usecount++;
	cde_app->ctx_count++;
	if (cde_app->ctx_count > cde_app->ctx_count_top)
		cde_app->ctx_count_top = cde_app->ctx_count;
	list_add(&cde_ctx->list, &cde_app->used_contexts);

	return cde_ctx;
}

static struct gk20a_cde_ctx *gk20a_cde_allocate_context(struct gk20a *g)
{
	struct gk20a_cde_ctx *cde_ctx;
	int ret;

	cde_ctx = kzalloc(sizeof(*cde_ctx), GFP_KERNEL);
	if (!cde_ctx)
		return ERR_PTR(-ENOMEM);

	cde_ctx->g = g;
	cde_ctx->pdev = g->dev;

	ret = gk20a_cde_load(cde_ctx);
	if (ret) {
		kfree(cde_ctx);
		return ERR_PTR(ret);
	}

	INIT_LIST_HEAD(&cde_ctx->list);
	cde_ctx->is_temporary = false;
	cde_ctx->in_use = false;
	INIT_DELAYED_WORK(&cde_ctx->ctx_deleter_work,
			gk20a_cde_ctx_deleter_fn);

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_cde_ctx, "cde: allocated %p", cde_ctx);
	return cde_ctx;
}

int gk20a_cde_convert(struct gk20a *g,
		      struct dma_buf *compbits_buf,
		      s32 compbits_kind, u64 compbits_byte_offset,
		      u32 compbits_size, struct nvgpu_fence *fence,
		      u32 __flags, struct gk20a_cde_param *params,
		      int num_params, struct gk20a_fence **fence_out)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_ctx *cde_ctx = NULL;
	struct gk20a_comptags comptags;
	u64 compbits_offset = 0;
	u64 map_vaddr = 0;
	u64 map_offset = 0;
	u32 map_size = 0;
	u64 big_page_mask = 0;
	u32 flags;
	int err, i;

	mutex_lock(&g->cde_app.mutex);

	cde_ctx = gk20a_cde_get_context(g);
	if (IS_ERR(cde_ctx)) {
		err = PTR_ERR(cde_ctx);
		goto exit_unlock;
	}

	/* First, map the buffer to local va */

	/* ensure that the compbits buffer has drvdata */
	err = gk20a_dmabuf_alloc_drvdata(compbits_buf, &g->dev->dev);
	if (err)
		goto exit_unlock;

	/* compbits don't start at page aligned offset, so we need to align
	   the region to be mapped */
	big_page_mask = cde_ctx->vm->big_page_size - 1;
	map_offset = compbits_byte_offset & ~big_page_mask;

	/* compute compbit start offset from the beginning of the mapped
	   area */
	compbits_offset = compbits_byte_offset & big_page_mask;

	if (!compbits_size) {
		compbits_size = compbits_buf->size - compbits_byte_offset;
		map_size = compbits_buf->size - map_offset;
	}

	/* map the destination buffer */
	get_dma_buf(compbits_buf); /* a ref for gk20a_vm_map */
	map_vaddr = gk20a_vm_map(cde_ctx->vm, compbits_buf, 0,
				 NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_TRUE,
				 compbits_kind, NULL, true,
				 gk20a_mem_flag_none,
				 map_offset, map_size);
	if (!map_vaddr) {
		dma_buf_put(compbits_buf);
		err = -EINVAL;
		goto exit_unlock;
	}

	/* store source buffer compression tags */
	gk20a_get_comptags(&g->dev->dev, compbits_buf, &comptags);
	cde_ctx->surf_param_offset = comptags.offset;
	cde_ctx->surf_param_lines = comptags.lines;

	/* store surface vaddr. This is actually compbit vaddr, but since
	   compbits live in the same surface, and we can get the alloc base
	   address by using gk20a_mm_gpuva_to_iova_base, this will do */
	cde_ctx->surf_vaddr = map_vaddr;

	/* store information about destination */
	cde_ctx->compbit_vaddr = map_vaddr + compbits_offset;
	cde_ctx->compbit_size = compbits_size;

	/* remove existing argument data */
	memset(cde_ctx->user_param_values, 0,
	       sizeof(cde_ctx->user_param_values));

	/* read user space arguments for the conversion */
	for (i = 0; i < num_params; i++) {
		struct gk20a_cde_param *param = params + i;
		int id = param->id - NUM_RESERVED_PARAMS;

		if (id < 0 || id >= MAX_CDE_USER_PARAMS) {
			gk20a_warn(&cde_ctx->pdev->dev, "cde: unknown user parameter");
			err = -EINVAL;
			goto exit_unlock;
		}
		cde_ctx->user_param_values[id] = param->value;
	}

	/* patch data */
	err = gk20a_cde_patch_params(cde_ctx);
	if (err) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: failed to patch parameters");
		goto exit_unlock;
	}

	gk20a_dbg(gpu_dbg_cde, "cde: buffer=cbc, size=%zu, gpuva=%llx\n",
		 g->gr.compbit_store.size, cde_ctx->backing_store_vaddr);
	gk20a_dbg(gpu_dbg_cde, "cde: buffer=compbits, size=%llu, gpuva=%llx\n",
		 cde_ctx->compbit_size, cde_ctx->compbit_vaddr);

	/* execute the init push buffer */
	if (!cde_ctx->init_cmd_executed) {
		err = gk20a_cde_execute_buffer(cde_ctx, TYPE_BUF_COMMAND_INIT,
					       NULL, 0, NULL);
		if (err)
			goto exit_unlock;

		cde_ctx->init_cmd_executed = true;
	}

	/* take always the postfence as it is needed for protecting the
	 * cde context */
	flags = __flags | NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET;

	/* execute the conversion buffer */
	err = gk20a_cde_execute_buffer(cde_ctx, TYPE_BUF_COMMAND_CONVERT,
				       fence, flags, fence_out);

exit_unlock:

	/* unmap the buffers - channel holds references to them now */
	if (map_vaddr)
		gk20a_vm_unmap(cde_ctx->vm, map_vaddr);

	mutex_unlock(&g->cde_app.mutex);
	return err;
}

static void gk20a_cde_finished_ctx_cb(struct channel_gk20a *ch, void *data)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_ctx *cde_ctx = data;
	struct gk20a *g = cde_ctx->g;
	struct gk20a_cde_app *cde_app = &g->cde_app;
	bool channel_idle;

	mutex_lock(&ch->jobs_lock);
	channel_idle = list_empty(&ch->jobs);
	mutex_unlock(&ch->jobs_lock);

	if (!channel_idle)
		return;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_cde_ctx, "cde: finished %p", cde_ctx);

	if (ch->has_timedout) {
		if (cde_ctx->is_temporary) {
			gk20a_warn(&cde_ctx->pdev->dev,
					"cde: channel had timed out"
					" (temporary channel)");
			/* going to be deleted anyway */
		} else {
			gk20a_warn(&cde_ctx->pdev->dev,
					"cde: channel had timed out"
					", reloading");
			/* mark it to be deleted, replace with a new one */
			mutex_lock(&cde_app->mutex);
			cde_ctx->is_temporary = true;
			if (gk20a_cde_create_context(g)) {
				gk20a_err(&cde_ctx->pdev->dev,
						"cde: can't replace context");
			}
			mutex_unlock(&cde_app->mutex);
		}
	} else {
		gk20a_ctx_release(cde_ctx);
	}

	/* delete temporary contexts later */
	if (cde_ctx->is_temporary)
		schedule_delayed_work(&cde_ctx->ctx_deleter_work,
			msecs_to_jiffies(CTX_DELETE_TIME));
}

static int gk20a_cde_load(struct gk20a_cde_ctx *cde_ctx)
{
	struct gk20a *g = cde_ctx->g;
	const struct firmware *img;
	struct channel_gk20a *ch;
	struct gr_gk20a *gr = &g->gr;
	int err = 0;
	u64 vaddr;

	img = gk20a_request_firmware(g, "gpu2cde.bin");
	if (!img) {
		dev_err(&cde_ctx->pdev->dev, "cde: could not fetch the firmware");
		return -ENOSYS;
	}

	ch = gk20a_open_new_channel_with_cb(g, gk20a_cde_finished_ctx_cb,
			cde_ctx);
	if (!ch) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: gk20a channel not available");
		err = -ENOMEM;
		goto err_get_gk20a_channel;
	}

	/* bind the channel to the vm */
	gk20a_vm_get(&g->mm.pmu.vm);
	ch->vm = &g->mm.pmu.vm;
	err = channel_gk20a_commit_va(ch);
	if (err) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: could not bind vm");
		goto err_commit_va;
	}

	/* allocate gpfifo (1024 should be more than enough) */
	err = gk20a_alloc_channel_gpfifo(ch,
		&(struct nvgpu_alloc_gpfifo_args){1024, 0});
	if (err) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: unable to allocate gpfifo");
		goto err_alloc_gpfifo;
	}

	/* map backing store to gpu virtual space */
	vaddr = gk20a_gmmu_map(ch->vm, &gr->compbit_store.sgt,
			       g->gr.compbit_store.size,
			       NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_TRUE,
			       gk20a_mem_flag_read_only);

	if (!vaddr) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: cannot map compression bit backing store");
		err = ENOMEM;
		goto err_map_backingstore;
	}

	/* store initialisation data */
	cde_ctx->ch = ch;
	cde_ctx->vm = ch->vm;
	cde_ctx->backing_store_vaddr = vaddr;

	/* initialise the firmware */
	err = gk20a_init_cde_img(cde_ctx, img);
	if (err) {
		gk20a_warn(&cde_ctx->pdev->dev, "cde: image initialisation failed");
		goto err_init_cde_img;
	}

	/* initialisation done */
	release_firmware(img);

	return 0;

err_init_cde_img:
	gk20a_gmmu_unmap(ch->vm, vaddr, g->gr.compbit_store.size, 1);
err_map_backingstore:
err_alloc_gpfifo:
	gk20a_vm_put(ch->vm);
err_commit_va:
err_get_gk20a_channel:
	release_firmware(img);
	dev_err(&cde_ctx->pdev->dev, "cde: couldn't initialise buffer converter: %d",
		err);
	return err;
}

int gk20a_cde_reload(struct gk20a *g)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &g->cde_app;
	int err;

	if (!cde_app->initialised)
		return -ENOSYS;

	err = gk20a_busy(g->dev);
	if (err)
		return err;

	mutex_lock(&cde_app->mutex);

	gk20a_cde_stop(g);

	err = gk20a_cde_create_contexts(g);
	if (!err)
		cde_app->initialised = true;

	mutex_unlock(&cde_app->mutex);

	gk20a_idle(g->dev);
	return err;
}

int gk20a_init_cde_support(struct gk20a *g)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &g->cde_app;
	int err;

	if (cde_app->initialised)
		return 0;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_cde_ctx, "cde: init");

	mutex_init(&cde_app->mutex);
	mutex_lock(&cde_app->mutex);

	INIT_LIST_HEAD(&cde_app->free_contexts);
	INIT_LIST_HEAD(&cde_app->used_contexts);
	cde_app->ctx_count = 0;
	cde_app->ctx_count_top = 0;
	cde_app->ctx_usecount = 0;

	err = gk20a_cde_create_contexts(g);
	if (!err)
		cde_app->initialised = true;

	mutex_unlock(&cde_app->mutex);
	gk20a_dbg(gpu_dbg_cde_ctx, "cde: init finished: %d", err);
	return err;
}

enum cde_launch_patch_id {
	PATCH_H_QMD_CTA_RASTER_WIDTH_ID     = 1024,
	PATCH_H_QMD_CTA_RASTER_HEIGHT_ID    = 1025,
	PATCH_QMD_CTA_RASTER_DEPTH_ID       = 1026, /* for firmware v0 only */
	PATCH_QMD_CTA_THREAD_DIMENSION0_ID  = 1027,
	PATCH_QMD_CTA_THREAD_DIMENSION1_ID  = 1028,
	PATCH_QMD_CTA_THREAD_DIMENSION2_ID  = 1029, /* for firmware v0 only */
	PATCH_USER_CONST_XTILES_ID          = 1030, /* for firmware v0 only */
	PATCH_USER_CONST_YTILES_ID          = 1031, /* for firmware v0 only */
	PATCH_USER_CONST_BLOCKHEIGHTLOG2_ID = 1032,
	PATCH_USER_CONST_DSTPITCH_ID        = 1033, /* for firmware v0 only */
	PATCH_H_USER_CONST_FLAGS_ID         = 1034, /* for firmware v0 only */
	PATCH_H_VPC_CURRENT_GRID_SIZE_X_ID  = 1035,
	PATCH_H_VPC_CURRENT_GRID_SIZE_Y_ID  = 1036,
	PATCH_H_VPC_CURRENT_GRID_SIZE_Z_ID  = 1037,
	PATCH_VPC_CURRENT_GROUP_SIZE_X_ID   = 1038,
	PATCH_VPC_CURRENT_GROUP_SIZE_Y_ID   = 1039,
	PATCH_VPC_CURRENT_GROUP_SIZE_Z_ID   = 1040,
	PATCH_USER_CONST_XBLOCKS_ID         = 1041,
	PATCH_H_USER_CONST_DSTOFFSET_ID     = 1042,
	PATCH_V_QMD_CTA_RASTER_WIDTH_ID     = 1043,
	PATCH_V_QMD_CTA_RASTER_HEIGHT_ID    = 1044,
	PATCH_V_USER_CONST_DSTOFFSET_ID     = 1045,
	PATCH_V_VPC_CURRENT_GRID_SIZE_X_ID  = 1046,
	PATCH_V_VPC_CURRENT_GRID_SIZE_Y_ID  = 1047,
	PATCH_V_VPC_CURRENT_GRID_SIZE_Z_ID  = 1048,
	PATCH_H_LAUNCH_WORD1_ID             = 1049,
	PATCH_H_LAUNCH_WORD2_ID             = 1050,
	PATCH_V_LAUNCH_WORD1_ID             = 1051,
	PATCH_V_LAUNCH_WORD2_ID             = 1052,
	PATCH_H_QMD_PROGRAM_OFFSET_ID       = 1053,
	PATCH_H_QMD_REGISTER_COUNT_ID       = 1054,
	PATCH_V_QMD_PROGRAM_OFFSET_ID       = 1055,
	PATCH_V_QMD_REGISTER_COUNT_ID       = 1056,
};

enum programs {
	PROG_HPASS              = 0,
	PROG_VPASS_LARGE        = 1,
	PROG_VPASS_SMALL        = 2,
	PROG_HPASS_DEBUG        = 3,
	PROG_VPASS_LARGE_DEBUG  = 4,
	PROG_VPASS_SMALL_DEBUG  = 5,
	PROG_PASSTHROUGH        = 6,
	NUM_PROGRAMS            = 7
};

/* maximum number of WRITE_PATCHes in the below function */
#define MAX_CDE_LAUNCH_PATCHES		  32

static int gk20a_buffer_convert_gpu_to_cde_v0(
		struct gk20a *g,
		struct dma_buf *dmabuf, u32 consumer,
		u64 offset, u64 compbits_hoffset, u64 compbits_voffset,
		u32 width, u32 height, u32 block_height_log2,
		u32 submit_flags, struct nvgpu_fence *fence_in,
		struct gk20a_buffer_state *state)
{
	struct gk20a_cde_param params[MAX_CDE_LAUNCH_PATCHES];
	int param = 0;
	int err = 0;
	struct gk20a_fence *new_fence = NULL;
	const int wgx = 8;
	const int wgy = 8;
	const int compbits_per_byte = 4; /* one byte stores 4 compbit pairs */
	const int xalign = compbits_per_byte * wgx;
	const int yalign = wgy;

	/* firmware v0 needs to call swizzling twice */
	int i;
	for (i = 0; i < 2; i++) {
		/* Compute per launch parameters */
		const bool vpass = (i == 1);
		const int transposed_width = vpass ? height : width;
		const int transposed_height = vpass ? width : height;
		const int xtiles = (transposed_width + 7) >> 3;
		const int ytiles = (transposed_height + 7) >> 3;
		const int gridw = roundup(xtiles, xalign) / xalign;
		const int gridh = roundup(ytiles, yalign) / yalign;
		const int flags = (vpass ? 4 : 0) |
			g->cde_app.shader_parameter;
		const int dst_stride = 128; /* chip constant */

		if ((vpass && !(consumer & NVGPU_GPU_COMPBITS_CDEV)) ||
		    (!vpass && !(consumer & NVGPU_GPU_COMPBITS_CDEH)))
			continue;

		if (xtiles > 4096 / 8 || ytiles > 4096 / 8)
			gk20a_warn(&g->dev->dev, "cde: surface is exceptionally large (xtiles=%d, ytiles=%d)",
				   xtiles, ytiles);

		gk20a_dbg(gpu_dbg_cde, "pass=%c", vpass ? 'V' : 'H');
		gk20a_dbg(gpu_dbg_cde, "w=%d, h=%d, bh_log2=%d, compbits_hoffset=0x%llx, compbits_voffset=0x%llx",
			  width, height, block_height_log2,
			  compbits_hoffset, compbits_voffset);
		gk20a_dbg(gpu_dbg_cde, "resolution (%d, %d) tiles (%d, %d)",
			  width, height, xtiles, ytiles);
		gk20a_dbg(gpu_dbg_cde, "group (%d, %d) grid (%d, %d)",
			  wgx, wgy, gridw, gridh);

		/* Write parameters */
#define WRITE_PATCH(NAME, VALUE) \
	params[param++] = (struct gk20a_cde_param){NAME##_ID, 0, VALUE}
		param = 0;
		WRITE_PATCH(PATCH_USER_CONST_XTILES, xtiles);
		WRITE_PATCH(PATCH_USER_CONST_YTILES, ytiles);
		WRITE_PATCH(PATCH_USER_CONST_BLOCKHEIGHTLOG2,
			block_height_log2);
		WRITE_PATCH(PATCH_USER_CONST_DSTPITCH, dst_stride);
		WRITE_PATCH(PATCH_H_USER_CONST_FLAGS, flags);
		WRITE_PATCH(PATCH_H_VPC_CURRENT_GRID_SIZE_X, gridw);
		WRITE_PATCH(PATCH_H_VPC_CURRENT_GRID_SIZE_Y, gridh);
		WRITE_PATCH(PATCH_H_VPC_CURRENT_GRID_SIZE_Z, 1);
		WRITE_PATCH(PATCH_VPC_CURRENT_GROUP_SIZE_X, wgx);
		WRITE_PATCH(PATCH_VPC_CURRENT_GROUP_SIZE_Y, wgy);
		WRITE_PATCH(PATCH_VPC_CURRENT_GROUP_SIZE_Z, 1);
		WRITE_PATCH(PATCH_H_QMD_CTA_RASTER_WIDTH, gridw);
		WRITE_PATCH(PATCH_H_QMD_CTA_RASTER_HEIGHT, gridh);
		WRITE_PATCH(PATCH_QMD_CTA_RASTER_DEPTH, 1);
		WRITE_PATCH(PATCH_QMD_CTA_THREAD_DIMENSION0, wgx);
		WRITE_PATCH(PATCH_QMD_CTA_THREAD_DIMENSION1, wgy);
		WRITE_PATCH(PATCH_QMD_CTA_THREAD_DIMENSION2, 1);
#undef WRITE_PATCH

		err = gk20a_cde_convert(g, dmabuf,
					0, /* dst kind */
					vpass ?
					compbits_voffset :
					compbits_hoffset,
					0, /* dst_size, 0 = auto */
					fence_in, submit_flags,
					params, param,
					&new_fence);
		if (err)
			goto out;

		/* compbits generated, update state & fence */
		gk20a_fence_put(state->fence);
		state->fence = new_fence;
		state->valid_compbits |= vpass ?
			NVGPU_GPU_COMPBITS_CDEV :
			NVGPU_GPU_COMPBITS_CDEH;
	}
out:
	return err;
}

static int gk20a_buffer_convert_gpu_to_cde_v1(
		struct gk20a *g,
		struct dma_buf *dmabuf, u32 consumer,
		u64 offset, u64 compbits_hoffset, u64 compbits_voffset,
		u32 width, u32 height, u32 block_height_log2,
		u32 submit_flags, struct nvgpu_fence *fence_in,
		struct gk20a_buffer_state *state)
{
	struct gk20a_cde_param params[MAX_CDE_LAUNCH_PATCHES];
	int param = 0;
	int err = 0;
	struct gk20a_fence *new_fence = NULL;
	const int wgx = 8;
	const int wgy = 8;
	const int compbits_per_byte = 4; /* one byte stores 4 compbit pairs */
	const int xalign = compbits_per_byte * wgx;
	const int yalign = wgy;

	/* Compute per launch parameters */
	const int xtiles = (width + 7) >> 3;
	const int ytiles = (height + 7) >> 3;
	const int gridw_h = roundup(xtiles, xalign) / xalign;
	const int gridh_h = roundup(ytiles, yalign) / yalign;
	const int gridw_v = roundup(ytiles, xalign) / xalign;
	const int gridh_v = roundup(xtiles, yalign) / yalign;
	const int xblocks = (xtiles + 1) >> 1;
	const int voffset = compbits_voffset - compbits_hoffset;

	int hprog = PROG_HPASS;
	int vprog = (block_height_log2 >= 2) ?
		PROG_VPASS_LARGE : PROG_VPASS_SMALL;
	if (g->cde_app.shader_parameter == 1) {
		hprog = PROG_PASSTHROUGH;
		vprog = PROG_PASSTHROUGH;
	} else if (g->cde_app.shader_parameter == 2) {
		hprog = PROG_HPASS_DEBUG;
		vprog = (block_height_log2 >= 2) ?
			PROG_VPASS_LARGE_DEBUG :
			PROG_VPASS_SMALL_DEBUG;
	}

	if (xtiles > 4096 / 8 || ytiles > 4096 / 8)
		gk20a_warn(&g->dev->dev, "cde: surface is exceptionally large (xtiles=%d, ytiles=%d)",
			   xtiles, ytiles);

	gk20a_dbg(gpu_dbg_cde, "w=%d, h=%d, bh_log2=%d, compbits_hoffset=0x%llx, compbits_voffset=0x%llx",
		  width, height, block_height_log2,
		  compbits_hoffset, compbits_voffset);
	gk20a_dbg(gpu_dbg_cde, "resolution (%d, %d) tiles (%d, %d)",
		  width, height, xtiles, ytiles);
	gk20a_dbg(gpu_dbg_cde, "group (%d, %d) gridH (%d, %d) gridV (%d, %d)",
		  wgx, wgy, gridw_h, gridh_h, gridw_v, gridh_v);
	gk20a_dbg(gpu_dbg_cde, "hprog=%d, offset=0x%x, regs=%d, vprog=%d, offset=0x%x, regs=%d",
		  hprog,
		  g->cde_app.arrays[ARRAY_PROGRAM_OFFSET][hprog],
		  g->cde_app.arrays[ARRAY_REGISTER_COUNT][hprog],
		  vprog,
		  g->cde_app.arrays[ARRAY_PROGRAM_OFFSET][vprog],
		  g->cde_app.arrays[ARRAY_REGISTER_COUNT][vprog]);

	/* Write parameters */
#define WRITE_PATCH(NAME, VALUE) \
	params[param++] = (struct gk20a_cde_param){NAME##_ID, 0, VALUE}
	WRITE_PATCH(PATCH_USER_CONST_XBLOCKS, xblocks);
	WRITE_PATCH(PATCH_USER_CONST_BLOCKHEIGHTLOG2,
		block_height_log2);
	WRITE_PATCH(PATCH_QMD_CTA_THREAD_DIMENSION0, wgx);
	WRITE_PATCH(PATCH_QMD_CTA_THREAD_DIMENSION1, wgy);
	WRITE_PATCH(PATCH_VPC_CURRENT_GROUP_SIZE_X, wgx);
	WRITE_PATCH(PATCH_VPC_CURRENT_GROUP_SIZE_Y, wgy);
	WRITE_PATCH(PATCH_VPC_CURRENT_GROUP_SIZE_Z, 1);

	WRITE_PATCH(PATCH_H_QMD_CTA_RASTER_WIDTH, gridw_h);
	WRITE_PATCH(PATCH_H_QMD_CTA_RASTER_HEIGHT, gridh_h);
	WRITE_PATCH(PATCH_H_USER_CONST_DSTOFFSET, 0);
	WRITE_PATCH(PATCH_H_VPC_CURRENT_GRID_SIZE_X, gridw_h);
	WRITE_PATCH(PATCH_H_VPC_CURRENT_GRID_SIZE_Y, gridh_h);
	WRITE_PATCH(PATCH_H_VPC_CURRENT_GRID_SIZE_Z, 1);

	WRITE_PATCH(PATCH_V_QMD_CTA_RASTER_WIDTH, gridw_v);
	WRITE_PATCH(PATCH_V_QMD_CTA_RASTER_HEIGHT, gridh_v);
	WRITE_PATCH(PATCH_V_USER_CONST_DSTOFFSET, voffset);
	WRITE_PATCH(PATCH_V_VPC_CURRENT_GRID_SIZE_X, gridw_v);
	WRITE_PATCH(PATCH_V_VPC_CURRENT_GRID_SIZE_Y, gridh_v);
	WRITE_PATCH(PATCH_V_VPC_CURRENT_GRID_SIZE_Z, 1);

	WRITE_PATCH(PATCH_H_QMD_PROGRAM_OFFSET,
		g->cde_app.arrays[ARRAY_PROGRAM_OFFSET][hprog]);
	WRITE_PATCH(PATCH_H_QMD_REGISTER_COUNT,
		g->cde_app.arrays[ARRAY_REGISTER_COUNT][hprog]);
	WRITE_PATCH(PATCH_V_QMD_PROGRAM_OFFSET,
		g->cde_app.arrays[ARRAY_PROGRAM_OFFSET][vprog]);
	WRITE_PATCH(PATCH_V_QMD_REGISTER_COUNT,
		g->cde_app.arrays[ARRAY_REGISTER_COUNT][vprog]);

	if (consumer & NVGPU_GPU_COMPBITS_CDEH) {
		WRITE_PATCH(PATCH_H_LAUNCH_WORD1,
			g->cde_app.arrays[ARRAY_LAUNCH_COMMAND][0]);
		WRITE_PATCH(PATCH_H_LAUNCH_WORD2,
			g->cde_app.arrays[ARRAY_LAUNCH_COMMAND][1]);
	} else {
		WRITE_PATCH(PATCH_H_LAUNCH_WORD1,
			g->cde_app.arrays[ARRAY_LAUNCH_COMMAND][2]);
		WRITE_PATCH(PATCH_H_LAUNCH_WORD2,
			g->cde_app.arrays[ARRAY_LAUNCH_COMMAND][3]);
	}

	if (consumer & NVGPU_GPU_COMPBITS_CDEV) {
		WRITE_PATCH(PATCH_V_LAUNCH_WORD1,
			g->cde_app.arrays[ARRAY_LAUNCH_COMMAND][0]);
		WRITE_PATCH(PATCH_V_LAUNCH_WORD2,
			g->cde_app.arrays[ARRAY_LAUNCH_COMMAND][1]);
	} else {
		WRITE_PATCH(PATCH_V_LAUNCH_WORD1,
			g->cde_app.arrays[ARRAY_LAUNCH_COMMAND][2]);
		WRITE_PATCH(PATCH_V_LAUNCH_WORD2,
			g->cde_app.arrays[ARRAY_LAUNCH_COMMAND][3]);
	}
#undef WRITE_PATCH

	err = gk20a_cde_convert(g, dmabuf,
				0, /* dst kind */
				compbits_hoffset,
				0, /* dst_size, 0 = auto */
				fence_in, submit_flags,
				params, param, &new_fence);
	if (err)
		goto out;

	/* compbits generated, update state & fence */
	gk20a_fence_put(state->fence);
	state->fence = new_fence;
	state->valid_compbits |= consumer &
		(NVGPU_GPU_COMPBITS_CDEH | NVGPU_GPU_COMPBITS_CDEV);
out:
	return err;
}

static int gk20a_buffer_convert_gpu_to_cde(
		struct gk20a *g, struct dma_buf *dmabuf, u32 consumer,
		u64 offset, u64 compbits_hoffset, u64 compbits_voffset,
		u32 width, u32 height, u32 block_height_log2,
		u32 submit_flags, struct nvgpu_fence *fence_in,
		struct gk20a_buffer_state *state)
{
	int err = 0;

	if (!g->cde_app.initialised)
		return -ENOSYS;

	err = gk20a_busy(g->dev);
	if (err)
		return err;

	gk20a_dbg(gpu_dbg_cde, "firmware version = %d\n",
		g->cde_app.firmware_version);

	if (g->cde_app.firmware_version == 0) {
		err = gk20a_buffer_convert_gpu_to_cde_v0(
		    g, dmabuf, consumer, offset, compbits_hoffset,
		    compbits_voffset, width, height, block_height_log2,
		    submit_flags, fence_in, state);
	} else {
		err = gk20a_buffer_convert_gpu_to_cde_v1(
		    g, dmabuf, consumer, offset, compbits_hoffset,
		    compbits_voffset, width, height, block_height_log2,
		    submit_flags, fence_in, state);
	}

	gk20a_idle(g->dev);
	return err;
}

int gk20a_prepare_compressible_read(
		struct gk20a *g, u32 buffer_fd, u32 request, u64 offset,
		u64 compbits_hoffset, u64 compbits_voffset,
		u32 width, u32 height, u32 block_height_log2,
		u32 submit_flags, struct nvgpu_fence *fence,
		u32 *valid_compbits, u32 *zbc_color,
		struct gk20a_fence **fence_out)
{
	int err = 0;
	struct gk20a_buffer_state *state;
	struct dma_buf *dmabuf;
	u32 missing_bits;

	dmabuf = dma_buf_get(buffer_fd);
	if (IS_ERR(dmabuf))
		return -EINVAL;

	err = gk20a_dmabuf_get_state(dmabuf, dev_from_gk20a(g),
				     offset, &state);
	if (err) {
		dma_buf_put(dmabuf);
		return err;
	}

	missing_bits = (state->valid_compbits ^ request) & request;

	mutex_lock(&state->lock);

	if (state->valid_compbits && request == NVGPU_GPU_COMPBITS_NONE) {

		gk20a_fence_put(state->fence);
		state->fence = NULL;
		/* state->fence = decompress();
		state->valid_compbits = 0; */
		err = -EINVAL;
		goto out;
	} else if (missing_bits) {
		u32 missing_cde_bits = missing_bits &
			 (NVGPU_GPU_COMPBITS_CDEH | NVGPU_GPU_COMPBITS_CDEV);
		if ((state->valid_compbits & NVGPU_GPU_COMPBITS_GPU) &&
		    missing_cde_bits) {
			err = gk20a_buffer_convert_gpu_to_cde(
					g, dmabuf,
					missing_cde_bits,
					offset, compbits_hoffset,
					compbits_voffset,
					width, height, block_height_log2,
					submit_flags, fence,
					state);
			if (err)
				goto out;
		}
	}

	if (state->fence && fence_out)
		*fence_out = gk20a_fence_get(state->fence);

	if (valid_compbits)
		*valid_compbits = state->valid_compbits;

	if (zbc_color)
		*zbc_color = state->zbc_color;

out:
	mutex_unlock(&state->lock);
	dma_buf_put(dmabuf);
	return err;
}

int gk20a_mark_compressible_write(struct gk20a *g, u32 buffer_fd,
				  u32 valid_compbits, u64 offset, u32 zbc_color)
{
	int err;
	struct gk20a_buffer_state *state;
	struct dma_buf *dmabuf;

	dmabuf = dma_buf_get(buffer_fd);
	if (IS_ERR(dmabuf)) {
		dev_err(dev_from_gk20a(g), "invalid dmabuf");
		return -EINVAL;
	}

	err = gk20a_dmabuf_get_state(dmabuf, dev_from_gk20a(g), offset, &state);
	if (err) {
		dev_err(dev_from_gk20a(g), "could not get state from dmabuf");
		dma_buf_put(dmabuf);
		return err;
	}

	mutex_lock(&state->lock);

	/* Update the compbits state. */
	state->valid_compbits = valid_compbits;
	state->zbc_color = zbc_color;

	/* Discard previous compbit job fence. */
	gk20a_fence_put(state->fence);
	state->fence = NULL;

	mutex_unlock(&state->lock);
	dma_buf_put(dmabuf);
	return 0;
}

static ssize_t gk20a_cde_reload_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct gk20a *g = file->private_data;
	gk20a_cde_reload(g);
	return count;
}

static const struct file_operations gk20a_cde_reload_fops = {
	.open		= simple_open,
	.write		= gk20a_cde_reload_write,
};

void gk20a_cde_debugfs_init(struct platform_device *dev)
{
	struct gk20a_platform *platform = platform_get_drvdata(dev);
	struct gk20a *g = get_gk20a(dev);

	debugfs_create_u32("cde_parameter", S_IWUSR | S_IRUGO,
			   platform->debugfs, &g->cde_app.shader_parameter);
	debugfs_create_file("reload_cde_firmware", S_IWUSR, platform->debugfs,
			    g, &gk20a_cde_reload_fops);
}
