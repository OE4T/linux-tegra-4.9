/*
 * Virtualized GPU Graphics
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include "vgpu/vgpu.h"
#include "gk20a/hw_gr_gk20a.h"

static int vgpu_gr_commit_inst(struct channel_gk20a *c, u64 gpu_va)
{
	struct gk20a_platform *platform = gk20a_get_platform(c->g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_CTX;
	msg.handle = platform->virt_handle;
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -1 : 0;
}

static int vgpu_gr_commit_global_ctx_buffers(struct gk20a *g,
					struct channel_gk20a *c, bool patch)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_GLOBAL_CTX;
	msg.handle = platform->virt_handle;
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -1 : 0;
}

/* load saved fresh copy of gloden image into channel gr_ctx */
static int vgpu_gr_load_golden_ctx_image(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_LOAD_GR_GOLDEN_CTX;
	msg.handle = platform->virt_handle;
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -1 : 0;
}

static int vgpu_gr_init_ctx_state(struct gk20a *g, struct gr_gk20a *gr)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);

	gk20a_dbg_fn("");

	vgpu_get_attribute(platform->virt_handle,
			TEGRA_VGPU_ATTRIB_GOLDEN_CTX_SIZE,
			&g->gr.ctx_vars.golden_image_size);
	vgpu_get_attribute(platform->virt_handle,
			TEGRA_VGPU_ATTRIB_ZCULL_CTX_SIZE,
			&g->gr.ctx_vars.zcull_ctxsw_image_size);
	if (!g->gr.ctx_vars.golden_image_size ||
		!g->gr.ctx_vars.zcull_ctxsw_image_size)
		return -ENXIO;

	gr->ctx_vars.buffer_size = g->gr.ctx_vars.golden_image_size;
	g->gr.ctx_vars.priv_access_map_size = 512 * 1024;
	return 0;
}

static int vgpu_gr_alloc_global_ctx_buffers(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int attr_buffer_size;

	u32 cb_buffer_size = gr->bundle_cb_default_size *
		gr_scc_bundle_cb_size_div_256b_byte_granularity_v();

	u32 pagepool_buffer_size = gr_scc_pagepool_total_pages_hwmax_value_v() *
		gr_scc_pagepool_total_pages_byte_granularity_v();

	gk20a_dbg_fn("");

	attr_buffer_size = g->ops.gr.calc_global_ctx_buffer_size(g);

	gk20a_dbg_info("cb_buffer_size : %d", cb_buffer_size);
	gr->global_ctx_buffer[CIRCULAR].mem.size = cb_buffer_size;

	gk20a_dbg_info("pagepool_buffer_size : %d", pagepool_buffer_size);
	gr->global_ctx_buffer[PAGEPOOL].mem.size = pagepool_buffer_size;

	gk20a_dbg_info("attr_buffer_size : %d", attr_buffer_size);
	gr->global_ctx_buffer[ATTRIBUTE].mem.size = attr_buffer_size;

	gk20a_dbg_info("priv access map size : %d",
		gr->ctx_vars.priv_access_map_size);
	gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem.size =
		gr->ctx_vars.priv_access_map_size;

	return 0;
}

static int vgpu_gr_map_global_ctx_buffers(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
	struct vm_gk20a *ch_vm = c->vm;
	u64 *g_bfr_va = c->ch_ctx.global_ctx_buffer_va;
	u64 *g_bfr_size = c->ch_ctx.global_ctx_buffer_size;
	struct gr_gk20a *gr = &g->gr;
	u64 gpu_va;
	u32 i;
	int err;

	gk20a_dbg_fn("");

	/* FIXME: add VPR support */

	/* Circular Buffer */
	gpu_va = gk20a_vm_alloc_va(ch_vm,
				gr->global_ctx_buffer[CIRCULAR].mem.size, 0);

	if (!gpu_va)
		goto clean_up;
	g_bfr_va[CIRCULAR_VA] = gpu_va;
	g_bfr_size[CIRCULAR_VA] = gr->global_ctx_buffer[CIRCULAR].mem.size;

	/* Attribute Buffer */
	gpu_va = gk20a_vm_alloc_va(ch_vm,
				gr->global_ctx_buffer[ATTRIBUTE].mem.size, 0);

	if (!gpu_va)
		goto clean_up;
	g_bfr_va[ATTRIBUTE_VA] = gpu_va;
	g_bfr_size[ATTRIBUTE_VA] = gr->global_ctx_buffer[ATTRIBUTE].mem.size;

	/* Page Pool */
	gpu_va = gk20a_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[PAGEPOOL].mem.size, 0);
	if (!gpu_va)
		goto clean_up;
	g_bfr_va[PAGEPOOL_VA] = gpu_va;
	g_bfr_size[PAGEPOOL_VA] = gr->global_ctx_buffer[PAGEPOOL].mem.size;

	/* Priv register Access Map */
	gpu_va = gk20a_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem.size, 0);
	if (!gpu_va)
		goto clean_up;
	g_bfr_va[PRIV_ACCESS_MAP_VA] = gpu_va;
	g_bfr_size[PRIV_ACCESS_MAP_VA] =
		gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem.size;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_MAP_GR_GLOBAL_CTX;
	msg.handle = platform->virt_handle;
	p->handle = c->virt_ctx;
	p->cb_va = g_bfr_va[CIRCULAR_VA];
	p->attr_va = g_bfr_va[ATTRIBUTE_VA];
	p->page_pool_va = g_bfr_va[PAGEPOOL_VA];
	p->priv_access_map_va = g_bfr_va[PRIV_ACCESS_MAP_VA];
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		goto clean_up;

	c->ch_ctx.global_ctx_buffer_mapped = true;
	return 0;

 clean_up:
	for (i = 0; i < NR_GLOBAL_CTX_BUF_VA; i++) {
		if (g_bfr_va[i]) {
			gk20a_vm_free_va(ch_vm, g_bfr_va[i],
					g_bfr_size[i], 0);
			g_bfr_va[i] = 0;
		}
	}
	return -ENOMEM;
}

static void vgpu_gr_unmap_global_ctx_buffers(struct channel_gk20a *c)
{
	struct gk20a_platform *platform = gk20a_get_platform(c->g->dev);
	struct vm_gk20a *ch_vm = c->vm;
	u64 *g_bfr_va = c->ch_ctx.global_ctx_buffer_va;
	u64 *g_bfr_size = c->ch_ctx.global_ctx_buffer_size;
	u32 i;

	gk20a_dbg_fn("");

	if (c->ch_ctx.global_ctx_buffer_mapped) {
		struct tegra_vgpu_cmd_msg msg;
		struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
		int err;

		msg.cmd = TEGRA_VGPU_CMD_CHANNEL_UNMAP_GR_GLOBAL_CTX;
		msg.handle = platform->virt_handle;
		p->handle = c->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		WARN_ON(err || msg.ret);
	}

	for (i = 0; i < NR_GLOBAL_CTX_BUF_VA; i++) {
		if (g_bfr_va[i]) {
			gk20a_vm_free_va(ch_vm, g_bfr_va[i], g_bfr_size[i], 0);
			g_bfr_va[i] = 0;
			g_bfr_size[i] = 0;
		}
	}
	c->ch_ctx.global_ctx_buffer_mapped = false;
}

static int vgpu_gr_alloc_channel_gr_ctx(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
	struct gr_gk20a *gr = &g->gr;
	struct gr_ctx_desc *gr_ctx;
	struct vm_gk20a *ch_vm = c->vm;
	int err;

	gk20a_dbg_fn("");

	if (gr->ctx_vars.buffer_size == 0)
		return 0;

	/* alloc channel gr ctx buffer */
	gr->ctx_vars.buffer_size = gr->ctx_vars.golden_image_size;
	gr->ctx_vars.buffer_total_size = gr->ctx_vars.golden_image_size;

	gr_ctx = kzalloc(sizeof(*gr_ctx), GFP_KERNEL);
	if (!gr_ctx)
		return -ENOMEM;

	gr_ctx->mem.size = gr->ctx_vars.buffer_total_size;
	gr_ctx->mem.gpu_va = gk20a_vm_alloc_va(ch_vm, gr_ctx->mem.size, 0);

	if (!gr_ctx->mem.gpu_va) {
		kfree(gr_ctx);
		return -ENOMEM;
	}

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_ALLOC_GR_CTX;
	msg.handle = platform->virt_handle;
	p->handle = c->virt_ctx;
	p->gr_ctx_va = gr_ctx->mem.gpu_va;
	p->class_num = c->obj_class;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	if (err || msg.ret) {
		kfree(gr_ctx);
		gk20a_vm_free_va(ch_vm, gr_ctx->mem.gpu_va,
				 gr_ctx->mem.size, 0);
		err = -ENOMEM;
	} else
		c->ch_ctx.gr_ctx = gr_ctx;

	return err;
}

static void vgpu_gr_free_channel_gr_ctx(struct channel_gk20a *c)
{
	struct gk20a_platform *platform = gk20a_get_platform(c->g->dev);
	struct channel_ctx_gk20a *ch_ctx = &c->ch_ctx;
	struct vm_gk20a *ch_vm = c->vm;

	gk20a_dbg_fn("");

	if (ch_ctx->gr_ctx && ch_ctx->gr_ctx->mem.gpu_va) {
		struct tegra_vgpu_cmd_msg msg;
		struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
		int err;

		msg.cmd = TEGRA_VGPU_CMD_CHANNEL_FREE_GR_CTX;
		msg.handle = platform->virt_handle;
		p->handle = c->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		WARN_ON(err || msg.ret);

		gk20a_vm_free_va(ch_vm, ch_ctx->gr_ctx->mem.gpu_va,
				ch_ctx->gr_ctx->mem.size, 0);
		ch_ctx->gr_ctx->mem.gpu_va = 0;
		kfree(ch_ctx->gr_ctx);
	}
}

static int vgpu_gr_alloc_channel_patch_ctx(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct patch_desc *patch_ctx = &c->ch_ctx.patch_ctx;
	struct vm_gk20a *ch_vm = c->vm;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
	int err;

	gk20a_dbg_fn("");

	patch_ctx->mem.size = 128 * sizeof(u32);
	patch_ctx->mem.gpu_va = gk20a_vm_alloc_va(ch_vm,
						  patch_ctx->mem.size, 0);
	if (!patch_ctx->mem.gpu_va)
		return -ENOMEM;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_ALLOC_GR_PATCH_CTX;
	msg.handle = platform->virt_handle;
	p->handle = c->virt_ctx;
	p->patch_ctx_va = patch_ctx->mem.gpu_va;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret) {
		gk20a_vm_free_va(ch_vm, patch_ctx->mem.gpu_va,
				 patch_ctx->mem.size, 0);
		err = -ENOMEM;
	}

	return err;
}

static void vgpu_gr_free_channel_patch_ctx(struct channel_gk20a *c)
{
	struct gk20a_platform *platform = gk20a_get_platform(c->g->dev);
	struct patch_desc *patch_ctx = &c->ch_ctx.patch_ctx;
	struct vm_gk20a *ch_vm = c->vm;

	gk20a_dbg_fn("");

	if (patch_ctx->mem.gpu_va) {
		struct tegra_vgpu_cmd_msg msg;
		struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
		int err;

		msg.cmd = TEGRA_VGPU_CMD_CHANNEL_FREE_GR_PATCH_CTX;
		msg.handle = platform->virt_handle;
		p->handle = c->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		WARN_ON(err || msg.ret);

		gk20a_vm_free_va(ch_vm, patch_ctx->mem.gpu_va,
				 patch_ctx->mem.size, 0);
		patch_ctx->mem.gpu_va = 0;
	}
}

static void vgpu_gr_free_channel_ctx(struct channel_gk20a *c)
{
	gk20a_dbg_fn("");

	vgpu_gr_unmap_global_ctx_buffers(c);
	vgpu_gr_free_channel_patch_ctx(c);
	if (!gk20a_is_channel_marked_as_tsg(c))
		vgpu_gr_free_channel_gr_ctx(c);

	/* zcull_ctx, pm_ctx */

	memset(&c->ch_ctx, 0, sizeof(struct channel_ctx_gk20a));

	c->num_objects = 0;
	c->first_init = false;
}

static int vgpu_gr_alloc_obj_ctx(struct channel_gk20a  *c,
				struct nvgpu_alloc_obj_ctx_args *args)
{
	struct gk20a *g = c->g;
	struct fifo_gk20a *f = &g->fifo;
	struct channel_ctx_gk20a *ch_ctx = &c->ch_ctx;
	struct tsg_gk20a *tsg = NULL;
	int err = 0;

	gk20a_dbg_fn("");

	/* an address space needs to have been bound at this point.*/
	if (!gk20a_channel_as_bound(c)) {
		gk20a_err(dev_from_gk20a(g),
			   "not bound to address space at time"
			   " of grctx allocation");
		return -EINVAL;
	}

	if (!g->ops.gr.is_valid_class(g, args->class_num)) {
		gk20a_err(dev_from_gk20a(g),
			   "invalid obj class 0x%x", args->class_num);
		err = -EINVAL;
		goto out;
	}
	c->obj_class = args->class_num;

	/* FIXME: add TSG support */
	if (gk20a_is_channel_marked_as_tsg(c))
		tsg = &f->tsg[c->tsgid];

	/* allocate gr ctx buffer */
	if (!ch_ctx->gr_ctx) {
		err = vgpu_gr_alloc_channel_gr_ctx(g, c);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"fail to allocate gr ctx buffer");
			goto out;
		}
	} else {
		/*TBD: needs to be more subtle about which is
		 * being allocated as some are allowed to be
		 * allocated along same channel */
		gk20a_err(dev_from_gk20a(g),
			"too many classes alloc'd on same channel");
		err = -EINVAL;
		goto out;
	}

	/* commit gr ctx buffer */
	err = vgpu_gr_commit_inst(c, ch_ctx->gr_ctx->mem.gpu_va);
	if (err) {
		gk20a_err(dev_from_gk20a(g),
			"fail to commit gr ctx buffer");
		goto out;
	}

	/* allocate patch buffer */
	if (ch_ctx->patch_ctx.mem.pages == NULL) {
		err = vgpu_gr_alloc_channel_patch_ctx(g, c);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"fail to allocate patch buffer");
			goto out;
		}
	}

	/* map global buffer to channel gpu_va and commit */
	if (!ch_ctx->global_ctx_buffer_mapped) {
		err = vgpu_gr_map_global_ctx_buffers(g, c);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"fail to map global ctx buffer");
			goto out;
		}
		gr_gk20a_elpg_protected_call(g,
				vgpu_gr_commit_global_ctx_buffers(g, c, true));
	}

	/* load golden image */
	if (!c->first_init) {
		err = gr_gk20a_elpg_protected_call(g,
				vgpu_gr_load_golden_ctx_image(g, c));
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"fail to load golden ctx image");
			goto out;
		}
		c->first_init = true;
	}

	c->num_objects++;

	gk20a_dbg_fn("done");
	return 0;
out:
	/* 1. gr_ctx, patch_ctx and global ctx buffer mapping
	   can be reused so no need to release them.
	   2. golden image load is a one time thing so if
	   they pass, no need to undo. */
	gk20a_err(dev_from_gk20a(g), "fail");
	return err;
}

static int vgpu_gr_free_obj_ctx(struct channel_gk20a  *c,
				struct nvgpu_free_obj_ctx_args *args)
{
	unsigned long timeout = gk20a_get_gr_idle_timeout(c->g);

	gk20a_dbg_fn("");

	if (c->num_objects == 0)
		return 0;

	c->num_objects--;

	if (c->num_objects == 0) {
		c->first_init = false;
		gk20a_disable_channel(c,
			!c->has_timedout,
			timeout);
	}

	return 0;
}

static int vgpu_gr_init_gr_config(struct gk20a *g, struct gr_gk20a *gr)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	u32 gpc_index;

	gk20a_dbg_fn("");

	if (vgpu_get_attribute(platform->virt_handle,
			TEGRA_VGPU_ATTRIB_GPC_COUNT, &gr->gpc_count))
		return -ENOMEM;

	if (vgpu_get_attribute(platform->virt_handle,
			TEGRA_VGPU_ATTRIB_MAX_TPC_PER_GPC_COUNT,
			&gr->max_tpc_per_gpc_count))
		return -ENOMEM;

	if (vgpu_get_attribute(platform->virt_handle,
			TEGRA_VGPU_ATTRIB_MAX_TPC_COUNT,
			&gr->max_tpc_count))
		return -ENOMEM;

	gr->gpc_tpc_mask = kzalloc(gr->gpc_count * sizeof(u32), GFP_KERNEL);
	if (!gr->gpc_tpc_mask) {
		gk20a_err(dev_from_gk20a(g), "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		if (g->ops.gr.get_gpc_tpc_mask)
			gr->gpc_tpc_mask[gpc_index] =
				g->ops.gr.get_gpc_tpc_mask(g, gpc_index);
	}

	g->ops.gr.bundle_cb_defaults(g);
	g->ops.gr.cb_size_default(g);
	g->ops.gr.calc_global_ctx_buffer_size(g);
	return 0;
}

static int vgpu_gr_bind_ctxsw_zcull(struct gk20a *g, struct gr_gk20a *gr,
				struct channel_gk20a *c, u64 zcull_va,
				u32 mode)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_zcull_bind_params *p = &msg.params.zcull_bind;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_BIND_ZCULL;
	msg.handle = platform->virt_handle;
	p->handle = c->virt_ctx;
	p->zcull_va = zcull_va;
	p->mode = mode;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -ENOMEM : 0;
}

static int vgpu_gr_get_zcull_info(struct gk20a *g, struct gr_gk20a *gr,
				struct gr_zcull_info *zcull_params)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_zcull_info_params *p = &msg.params.zcull_info;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_GET_ZCULL_INFO;
	msg.handle = platform->virt_handle;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		return -ENOMEM;

	zcull_params->width_align_pixels = p->width_align_pixels;
	zcull_params->height_align_pixels = p->height_align_pixels;
	zcull_params->pixel_squares_by_aliquots = p->pixel_squares_by_aliquots;
	zcull_params->aliquot_total = p->aliquot_total;
	zcull_params->region_byte_multiplier = p->region_byte_multiplier;
	zcull_params->region_header_size = p->region_header_size;
	zcull_params->subregion_header_size = p->subregion_header_size;
	zcull_params->subregion_width_align_pixels =
		p->subregion_width_align_pixels;
	zcull_params->subregion_height_align_pixels =
		p->subregion_height_align_pixels;
	zcull_params->subregion_count = p->subregion_count;

	return 0;
}

static void vgpu_gr_detect_sm_arch(struct gk20a *g)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	u32 v = 0, raw_version, version = 0;

	gk20a_dbg_fn("");

	if (vgpu_get_attribute(platform->virt_handle,
			TEGRA_VGPU_ATTRIB_GPC0_TPC0_SM_ARCH, &v))
		gk20a_err(dev_from_gk20a(g), "failed to retrieve SM arch");

	raw_version = gr_gpc0_tpc0_sm_arch_spa_version_v(v);
	if (raw_version == gr_gpc0_tpc0_sm_arch_spa_version_smkepler_lp_v())
		version = 0x320; /* SM 3.2 */
	else
		gk20a_err(dev_from_gk20a(g), "Unknown SM version 0x%x",
			  raw_version);

	/* on Kepler, SM version == SPA version */
	g->gpu_characteristics.sm_arch_spa_version = version;
	g->gpu_characteristics.sm_arch_sm_version = version;

	g->gpu_characteristics.sm_arch_warp_count =
		gr_gpc0_tpc0_sm_arch_warp_count_v(v);
}

static u32 vgpu_gr_get_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
	/* One TPC for gk20a */
	return 0x1;
}

static u32 vgpu_gr_get_max_fbps_count(struct gk20a *g)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	u32 max_fbps_count = 0;

	gk20a_dbg_fn("");

	if (vgpu_get_attribute(platform->virt_handle,
			TEGRA_VGPU_ATTRIB_NUM_FBPS, &max_fbps_count))
		gk20a_err(dev_from_gk20a(g), "failed to retrieve num fbps");

	return max_fbps_count;
}

static u32 vgpu_gr_get_fbp_en_mask(struct gk20a *g)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	u32 fbp_en_mask = 0;

	gk20a_dbg_fn("");

	if (vgpu_get_attribute(platform->virt_handle,
			TEGRA_VGPU_ATTRIB_FBP_EN_MASK, &fbp_en_mask))
		gk20a_err(dev_from_gk20a(g), "failed to retrieve fbp en mask");

	return fbp_en_mask;
}

static void vgpu_remove_gr_support(struct gr_gk20a *gr)
{
	gk20a_dbg_fn("");

	gk20a_allocator_destroy(&gr->comp_tags);

	kfree(gr->gpc_tpc_mask);
	gr->gpc_tpc_mask = NULL;
}

static int vgpu_gr_init_gr_setup_sw(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int err;

	gk20a_dbg_fn("");

	if (gr->sw_ready) {
		gk20a_dbg_fn("skip init");
		return 0;
	}

	gr->g = g;

	err = vgpu_gr_init_gr_config(g, gr);
	if (err)
		goto clean_up;

	err = vgpu_gr_init_ctx_state(g, gr);
	if (err)
		goto clean_up;

	err = g->ops.ltc.init_comptags(g, gr);
	if (err)
		goto clean_up;

	err = vgpu_gr_alloc_global_ctx_buffers(g);
	if (err)
		goto clean_up;

	mutex_init(&gr->ctx_mutex);

	gr->remove_support = vgpu_remove_gr_support;
	gr->sw_ready = true;

	gk20a_dbg_fn("done");
	return 0;

clean_up:
	gk20a_err(dev_from_gk20a(g), "fail");
	vgpu_remove_gr_support(gr);
	return err;
}

int vgpu_init_gr_support(struct gk20a *g)
{
	gk20a_dbg_fn("");

	return vgpu_gr_init_gr_setup_sw(g);
}

int vgpu_gr_isr(struct gk20a *g, struct tegra_vgpu_gr_intr_info *info)
{
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch = &f->channel[info->chid];

	gk20a_dbg_fn("");
	if (info->type != TEGRA_VGPU_GR_INTR_NOTIFY)
		gk20a_err(dev_from_gk20a(g), "gr intr (%d) on ch %u",
			info->type, info->chid);

	switch (info->type) {
	case TEGRA_VGPU_GR_INTR_NOTIFY:
		wake_up(&ch->notifier_wq);
		break;
	case TEGRA_VGPU_GR_INTR_SEMAPHORE_TIMEOUT:
		gk20a_set_error_notifier(ch,
				NVGPU_CHANNEL_GR_SEMAPHORE_TIMEOUT);
		break;
	case TEGRA_VGPU_GR_INTR_ILLEGAL_NOTIFY:
		gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_GR_ILLEGAL_NOTIFY);
	case TEGRA_VGPU_GR_INTR_ILLEGAL_METHOD:
		break;
	case TEGRA_VGPU_GR_INTR_ILLEGAL_CLASS:
		gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_FECS_ERROR:
		break;
	case TEGRA_VGPU_GR_INTR_CLASS_ERROR:
		gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_FIRMWARE_METHOD:
		gk20a_set_error_notifier(ch,
				NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_EXCEPTION:
		gk20a_set_error_notifier(ch,
				NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY);
		break;
	default:
		WARN_ON(1);
		break;
	}

	return 0;
}

void vgpu_init_gr_ops(struct gpu_ops *gops)
{
	gops->gr.free_channel_ctx = vgpu_gr_free_channel_ctx;
	gops->gr.alloc_obj_ctx = vgpu_gr_alloc_obj_ctx;
	gops->gr.free_obj_ctx = vgpu_gr_free_obj_ctx;
	gops->gr.bind_ctxsw_zcull = vgpu_gr_bind_ctxsw_zcull;
	gops->gr.get_zcull_info = vgpu_gr_get_zcull_info;
	gops->gr.detect_sm_arch = vgpu_gr_detect_sm_arch;
	gops->gr.get_gpc_tpc_mask = vgpu_gr_get_gpc_tpc_mask;
	gops->gr.get_max_fbps_count = vgpu_gr_get_max_fbps_count;
	gops->gr.get_fbp_en_mask = vgpu_gr_get_fbp_en_mask;
}
