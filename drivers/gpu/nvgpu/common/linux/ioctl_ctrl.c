/*
 * Copyright (c) 2011-2017, NVIDIA Corporation.  All rights reserved.
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

#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/anon_inodes.h>
#include <linux/fs.h>
#include <uapi/linux/nvgpu.h>

#include <nvgpu/bitops.h>
#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/bus.h>

#include "ioctl_ctrl.h"
#include "ioctl_tsg.h"
#include "gk20a/gk20a.h"
#include "gk20a/platform_gk20a.h"
#include "gk20a/fence_gk20a.h"
#include "os_linux.h"

#include <nvgpu/log.h>

#define HZ_TO_MHZ(a) ((a > 0xF414F9CD7ULL) ? 0xffff : (a >> 32) ? \
	(u32) ((a * 0x10C8ULL) >> 32) : (u16) ((u32) a/MHZ))
#define MHZ_TO_HZ(a) ((u64)a * MHZ)

struct gk20a_ctrl_priv {
	struct device *dev;
	struct gk20a *g;
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	struct nvgpu_clk_session *clk_session;
#endif
};

int gk20a_ctrl_dev_open(struct inode *inode, struct file *filp)
{
	struct nvgpu_os_linux *l;
	struct gk20a *g;
	struct gk20a_ctrl_priv *priv;
	int err = 0;

	gk20a_dbg_fn("");

	l = container_of(inode->i_cdev,
			 struct nvgpu_os_linux, ctrl.cdev);
	g = gk20a_get(&l->g);
	if (!g)
		return -ENODEV;

	priv = nvgpu_kzalloc(g, sizeof(struct gk20a_ctrl_priv));
	if (!priv) {
		err = -ENOMEM;
		goto free_ref;
	}
	filp->private_data = priv;
	priv->dev = dev_from_gk20a(g);
	/*
	 * We dont close the arbiter fd's after driver teardown to support
	 * GPU_LOST events, so we store g here, instead of dereferencing the
	 * dev structure on teardown
	 */
	priv->g = g;

	if (!g->gr.sw_ready) {
		err = gk20a_busy(g);
		if (err)
			goto free_ref;
		gk20a_idle(g);
	}

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	err = nvgpu_clk_arb_init_session(g, &priv->clk_session);
#endif
free_ref:
	if (err)
		gk20a_put(g);
	return err;
}
int gk20a_ctrl_dev_release(struct inode *inode, struct file *filp)
{
	struct gk20a_ctrl_priv *priv = filp->private_data;
	struct gk20a *g = priv->g;

	gk20a_dbg_fn("");

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	if (priv->clk_session)
		nvgpu_clk_arb_release_session(g, priv->clk_session);
#endif

	gk20a_put(g);
	nvgpu_kfree(g, priv);

	return 0;
}

static long
gk20a_ctrl_ioctl_gpu_characteristics(
	struct gk20a *g,
	struct nvgpu_gpu_get_characteristics *request)
{
	struct nvgpu_gpu_characteristics *pgpu = &g->gpu_characteristics;
	long err = 0;

	if (request->gpu_characteristics_buf_size > 0) {
		size_t write_size = sizeof(*pgpu);

		if (write_size > request->gpu_characteristics_buf_size)
			write_size = request->gpu_characteristics_buf_size;

		err = copy_to_user((void __user *)(uintptr_t)
				   request->gpu_characteristics_buf_addr,
				   pgpu, write_size);
	}

	if (err == 0)
		request->gpu_characteristics_buf_size = sizeof(*pgpu);

	return err;
}

static int gk20a_ctrl_prepare_compressible_read(
		struct gk20a *g,
		struct nvgpu_gpu_prepare_compressible_read_args *args)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct nvgpu_fence fence;
	struct gk20a_fence *fence_out = NULL;
	int ret = 0;
	int flags = args->submit_flags;

	fence.id = args->fence.syncpt_id;
	fence.value = args->fence.syncpt_value;

	ret = gk20a_prepare_compressible_read(l, args->handle,
			args->request_compbits, args->offset,
			args->compbits_hoffset, args->compbits_voffset,
			args->scatterbuffer_offset,
			args->width, args->height, args->block_height_log2,
			flags, &fence, &args->valid_compbits,
			&args->zbc_color, &fence_out);

	if (ret)
		return ret;

	/* Convert fence_out to something we can pass back to user space. */
	if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET) {
		if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE) {
			if (fence_out) {
				int fd = gk20a_fence_install_fd(fence_out);
				if (fd < 0)
					ret = fd;
				else
					args->fence.fd = fd;
			} else {
				args->fence.fd = -1;
			}
		} else {
			if (fence_out) {
				args->fence.syncpt_id = fence_out->syncpt_id;
				args->fence.syncpt_value =
						fence_out->syncpt_value;
			} else {
				args->fence.syncpt_id = -1;
				args->fence.syncpt_value = 0;
			}
		}
	}
	gk20a_fence_put(fence_out);

	return 0;
}

static int gk20a_ctrl_mark_compressible_write(
		struct gk20a *g,
		struct nvgpu_gpu_mark_compressible_write_args *args)
{
	int ret;

	ret = gk20a_mark_compressible_write(g, args->handle,
			args->valid_compbits, args->offset, args->zbc_color);

	return ret;
}

static int gk20a_ctrl_alloc_as(
		struct gk20a *g,
		struct nvgpu_alloc_as_args *args)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct gk20a_as_share *as_share;
	int err;
	int fd;
	struct file *file;
	char name[64];

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		return err;
	fd = err;

	snprintf(name, sizeof(name), "nvhost-%s-fd%d", g->name, fd);

	file = anon_inode_getfile(name, l->as_dev.cdev.ops, NULL, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}

	err = gk20a_as_alloc_share(g, args->big_page_size, args->flags,
				   &as_share);
	if (err)
		goto clean_up_file;

	fd_install(fd, file);
	file->private_data = as_share;

	args->as_fd = fd;
	return 0;

clean_up_file:
	fput(file);
clean_up:
	put_unused_fd(fd);
	return err;
}

static int gk20a_ctrl_open_tsg(struct gk20a *g,
			       struct nvgpu_gpu_open_tsg_args *args)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	int err;
	int fd;
	struct file *file;
	char name[64];

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		return err;
	fd = err;

	snprintf(name, sizeof(name), "nvgpu-%s-tsg%d", g->name, fd);

	file = anon_inode_getfile(name, l->tsg.cdev.ops, NULL, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}

	err = nvgpu_ioctl_tsg_open(g, file);
	if (err)
		goto clean_up_file;

	fd_install(fd, file);
	args->tsg_fd = fd;
	return 0;

clean_up_file:
	fput(file);
clean_up:
	put_unused_fd(fd);
	return err;
}

static int gk20a_ctrl_get_tpc_masks(struct gk20a *g,
				    struct nvgpu_gpu_get_tpc_masks_args *args)
{
	struct gr_gk20a *gr = &g->gr;
	int err = 0;
	const u32 gpc_tpc_mask_size = sizeof(u32) * gr->gpc_count;

	if (args->mask_buf_size > 0) {
		size_t write_size = gpc_tpc_mask_size;

		if (write_size > args->mask_buf_size)
			write_size = args->mask_buf_size;

		err = copy_to_user((void __user *)(uintptr_t)
				   args->mask_buf_addr,
				   gr->gpc_tpc_mask, write_size);
	}

	if (err == 0)
		args->mask_buf_size = gpc_tpc_mask_size;

	return err;
}

static int gk20a_ctrl_get_fbp_l2_masks(
	struct gk20a *g, struct nvgpu_gpu_get_fbp_l2_masks_args *args)
{
	struct gr_gk20a *gr = &g->gr;
	int err = 0;
	const u32 fbp_l2_mask_size = sizeof(u32) * gr->max_fbps_count;

	if (args->mask_buf_size > 0) {
		size_t write_size = fbp_l2_mask_size;

		if (write_size > args->mask_buf_size)
			write_size = args->mask_buf_size;

		err = copy_to_user((void __user *)(uintptr_t)
				   args->mask_buf_addr,
				   gr->fbp_rop_l2_en_mask, write_size);
	}

	if (err == 0)
		args->mask_buf_size = fbp_l2_mask_size;

	return err;
}

static int nvgpu_gpu_ioctl_l2_fb_ops(struct gk20a *g,
		struct nvgpu_gpu_l2_fb_args *args)
{
	int err = 0;

	if (args->l2_flush)
		g->ops.mm.l2_flush(g, args->l2_invalidate ? true : false);

	if (args->fb_flush)
		g->ops.mm.fb_flush(g);

	return err;
}

/* Invalidate i-cache for kepler & maxwell */
static int nvgpu_gpu_ioctl_inval_icache(
		struct gk20a *g,
		struct nvgpu_gpu_inval_icache_args *args)
{
	struct channel_gk20a *ch;
	int err;

	ch = gk20a_get_channel_from_file(args->channel_fd);
	if (!ch)
		return -EINVAL;

	/* Take the global lock, since we'll be doing global regops */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	err = g->ops.gr.inval_icache(g, ch);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_channel_put(ch);
	return err;
}

static int nvgpu_gpu_ioctl_set_mmu_debug_mode(
		struct gk20a *g,
		struct nvgpu_gpu_mmu_debug_mode_args *args)
{
	if (gk20a_busy(g)) {
		nvgpu_err(g, "failed to power on gpu");
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	g->ops.fb.set_debug_mode(g, args->state == 1);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_idle(g);
	return 0;
}

static int nvgpu_gpu_ioctl_set_debug_mode(
		struct gk20a *g,
		struct nvgpu_gpu_sm_debug_mode_args *args)
{
	struct channel_gk20a *ch;
	int err;

	ch = gk20a_get_channel_from_file(args->channel_fd);
	if (!ch)
		return -EINVAL;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	if (g->ops.gr.set_sm_debug_mode)
		err = g->ops.gr.set_sm_debug_mode(g, ch,
				args->sms, !!args->enable);
	else
		err = -ENOSYS;
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_channel_put(ch);
	return err;
}

static int nvgpu_gpu_ioctl_trigger_suspend(struct gk20a *g)
{
	int err;

	err = gk20a_busy(g);
	if (err)
	    return err;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	err = g->ops.gr.trigger_suspend(g);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_ioctl_wait_for_pause(struct gk20a *g,
		struct nvgpu_gpu_wait_pause_args *args)
{
	int err;
	struct warpstate *w_state;
	u32 sm_count, size;

	sm_count = g->gr.gpc_count * g->gr.tpc_count;
	size = sm_count * sizeof(struct warpstate);
	w_state = nvgpu_kzalloc(g, size);
	if (!w_state)
		return -ENOMEM;

	err = gk20a_busy(g);
	if (err)
		goto out_free;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	g->ops.gr.wait_for_pause(g, w_state);

	/* Copy to user space - pointed by "args->pwarpstate" */
	if (copy_to_user((void __user *)(uintptr_t)args->pwarpstate, w_state, size)) {
		gk20a_dbg_fn("copy_to_user failed!");
		err = -EFAULT;
	}

	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_idle(g);

out_free:
	nvgpu_kfree(g, w_state);

	return err;
}

static int nvgpu_gpu_ioctl_resume_from_pause(struct gk20a *g)
{
	int err;

	err = gk20a_busy(g);
	if (err)
	    return err;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	err = g->ops.gr.resume_from_pause(g);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_ioctl_clear_sm_errors(struct gk20a *g)
{
	int err;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = g->ops.gr.clear_sm_errors(g);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_ioctl_has_any_exception(
		struct gk20a *g,
		struct nvgpu_gpu_tpc_exception_en_status_args *args)
{
	u32 tpc_exception_en;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	tpc_exception_en = g->ops.gr.tpc_enabled_exceptions(g);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	args->tpc_exception_en_sm_mask = tpc_exception_en;

	return 0;
}

static int gk20a_ctrl_get_num_vsms(struct gk20a *g,
				    struct nvgpu_gpu_num_vsms *args)
{
	struct gr_gk20a *gr = &g->gr;
	args->num_vsms = gr->no_of_sm;
	return 0;
}

static int gk20a_ctrl_vsm_mapping(struct gk20a *g,
				    struct nvgpu_gpu_vsms_mapping *args)
{
	int err = 0;
	struct gr_gk20a *gr = &g->gr;
	size_t write_size = gr->no_of_sm *
				sizeof(struct nvgpu_gpu_vsms_mapping_entry);
	struct nvgpu_gpu_vsms_mapping_entry *vsms_buf;
	u32 i;

	vsms_buf = nvgpu_kzalloc(g, write_size);
	if (vsms_buf == NULL)
		return -ENOMEM;

	for (i = 0; i < gr->no_of_sm; i++) {
		vsms_buf[i].gpc_index = gr->sm_to_cluster[i].gpc_index;
		vsms_buf[i].tpc_index = gr->sm_to_cluster[i].tpc_index;
	}

	err = copy_to_user((void __user *)(uintptr_t)
			   args->vsms_map_buf_addr,
			   vsms_buf, write_size);
	nvgpu_kfree(g, vsms_buf);

	return err;
}

static int gk20a_ctrl_get_buffer_info(
	struct gk20a *g, struct nvgpu_gpu_get_buffer_info_args *args)
{
	return gk20a_mm_get_buffer_info(dev_from_gk20a(g), args->in.dmabuf_fd,
					&args->out.id, &args->out.length);
}

static int nvgpu_gpu_get_cpu_time_correlation_info(
	struct gk20a *g,
	struct nvgpu_gpu_get_cpu_time_correlation_info_args *args)
{
	struct nvgpu_cpu_time_correlation_sample *samples;
	int err;
	u32 i;

	if (args->count > NVGPU_GPU_GET_CPU_TIME_CORRELATION_INFO_MAX_COUNT)
		return -EINVAL;

	samples = nvgpu_kzalloc(g, args->count *
		sizeof(struct nvgpu_cpu_time_correlation_sample));
	if (!samples) {
		return -ENOMEM;
	}

	err = g->ops.bus.get_timestamps_zipper(g,
			args->source_id, args->count, samples);
	if (!err) {
		for (i = 0; i < args->count; i++) {
			args->samples[i].cpu_timestamp = samples[i].cpu_timestamp;
			args->samples[i].gpu_timestamp = samples[i].gpu_timestamp;
		}
	}

	nvgpu_kfree(g, samples);

	return err;
}

static int nvgpu_gpu_get_gpu_time(
	struct gk20a *g,
	struct nvgpu_gpu_get_gpu_time_args *args)
{
	u64 time;
	int err;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = g->ops.bus.read_ptimer(g, &time);
	if (!err)
		args->gpu_timestamp = time;

	gk20a_idle(g);
	return err;
}

static int nvgpu_gpu_get_engine_info(
	struct gk20a *g,
	struct nvgpu_gpu_get_engine_info_args *args)
{
	int err = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;
	u32 report_index = 0;
	u32 engine_id_idx;
	const u32 max_buffer_engines = args->engine_info_buf_size /
		sizeof(struct nvgpu_gpu_get_engine_info_item);
	struct nvgpu_gpu_get_engine_info_item __user *dst_item_list =
		(void __user *)(uintptr_t)args->engine_info_buf_addr;

	for (engine_id_idx = 0; engine_id_idx < g->fifo.num_engines;
		++engine_id_idx) {
		u32 active_engine_id = g->fifo.active_engines_list[engine_id_idx];
		const struct fifo_engine_info_gk20a *src_info =
			&g->fifo.engine_info[active_engine_id];
		struct nvgpu_gpu_get_engine_info_item dst_info;

		memset(&dst_info, 0, sizeof(dst_info));

		engine_enum = src_info->engine_enum;

		switch (engine_enum) {
		case ENGINE_GR_GK20A:
			dst_info.engine_id = NVGPU_GPU_ENGINE_ID_GR;
			break;

		case ENGINE_GRCE_GK20A:
			dst_info.engine_id = NVGPU_GPU_ENGINE_ID_GR_COPY;
			break;

		case ENGINE_ASYNC_CE_GK20A:
			dst_info.engine_id = NVGPU_GPU_ENGINE_ID_ASYNC_COPY;
			break;

		default:
			nvgpu_err(g, "Unmapped engine enum %u",
				  engine_enum);
			continue;
		}

		dst_info.engine_instance = src_info->inst_id;
		dst_info.runlist_id = src_info->runlist_id;

		if (report_index < max_buffer_engines) {
			err = copy_to_user(&dst_item_list[report_index],
					   &dst_info, sizeof(dst_info));
			if (err)
				goto clean_up;
		}

		++report_index;
	}

	args->engine_info_buf_size =
		report_index * sizeof(struct nvgpu_gpu_get_engine_info_item);

clean_up:
	return err;
}

static int nvgpu_gpu_alloc_vidmem(struct gk20a *g,
			struct nvgpu_gpu_alloc_vidmem_args *args)
{
	u32 align = args->in.alignment ? args->in.alignment : SZ_4K;
	int fd;

	gk20a_dbg_fn("");

	/* not yet supported */
	if (WARN_ON(args->in.flags & NVGPU_GPU_ALLOC_VIDMEM_FLAG_CPU_MASK))
		return -EINVAL;

	/* not yet supported */
	if (WARN_ON(args->in.flags & NVGPU_GPU_ALLOC_VIDMEM_FLAG_VPR))
		return -EINVAL;

	if (args->in.size & (SZ_4K - 1))
		return -EINVAL;

	if (!args->in.size)
		return -EINVAL;

	if (align & (align - 1))
		return -EINVAL;

	if (align > roundup_pow_of_two(args->in.size)) {
		/* log this special case, buddy allocator detail */
		nvgpu_warn(g,
			"alignment larger than buffer size rounded up to power of 2 is not supported");
		return -EINVAL;
	}

	fd = gk20a_vidmem_buf_alloc(g, args->in.size);
	if (fd < 0)
		return fd;

	args->out.dmabuf_fd = fd;

	gk20a_dbg_fn("done, fd=%d", fd);

	return 0;
}

static int nvgpu_gpu_get_memory_state(struct gk20a *g,
			struct nvgpu_gpu_get_memory_state_args *args)
{
	int err;

	gk20a_dbg_fn("");

	if (args->reserved[0] || args->reserved[1] ||
	    args->reserved[2] || args->reserved[3])
		return -EINVAL;

	err = gk20a_vidmem_get_space(g, &args->total_free_bytes);

	gk20a_dbg_fn("done, err=%d, bytes=%lld", err, args->total_free_bytes);

	return err;
}

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
static int nvgpu_gpu_clk_get_vf_points(struct gk20a *g,
		struct gk20a_ctrl_priv *priv,
		struct nvgpu_gpu_clk_vf_points_args *args)
{
	struct nvgpu_gpu_clk_vf_point clk_point;
	struct nvgpu_gpu_clk_vf_point __user *entry;
	struct nvgpu_clk_session *session = priv->clk_session;
	u32 clk_domains = 0;
	int err;
	u16 last_mhz;
	u16 *fpoints;
	u32 i;
	u32 max_points = 0;
	u32 num_points = 0;
	u16 min_mhz;
	u16 max_mhz;

	gk20a_dbg_fn("");

	if (!session || args->flags)
		return -EINVAL;

	clk_domains = nvgpu_clk_arb_get_arbiter_clk_domains(g);
	args->num_entries = 0;

	if (!nvgpu_clk_arb_is_valid_domain(g, args->clk_domain))
		return -EINVAL;

	err = nvgpu_clk_arb_get_arbiter_clk_f_points(g,
			args->clk_domain, &max_points, NULL);
	if (err)
		return err;

	if (!args->max_entries) {
		args->max_entries = max_points;
		return 0;
	}

	if (args->max_entries < max_points)
		return -EINVAL;

	err = nvgpu_clk_arb_get_arbiter_clk_range(g, args->clk_domain,
			&min_mhz, &max_mhz);
	if (err)
		return err;

	fpoints = nvgpu_kcalloc(g, max_points, sizeof(u16));
	if (!fpoints)
		return -ENOMEM;

	err = nvgpu_clk_arb_get_arbiter_clk_f_points(g,
			args->clk_domain, &max_points, fpoints);
	if (err)
		goto fail;

	entry = (struct nvgpu_gpu_clk_vf_point __user *)
			(uintptr_t)args->clk_vf_point_entries;

	last_mhz = 0;
	num_points = 0;
	for (i = 0; (i < max_points) && !err; i++) {

		/* filter out duplicate frequencies */
		if (fpoints[i] == last_mhz)
			continue;

		/* filter out out-of-range frequencies */
		if ((fpoints[i] < min_mhz) || (fpoints[i] > max_mhz))
			continue;

		last_mhz = fpoints[i];
		clk_point.freq_hz = MHZ_TO_HZ(fpoints[i]);

		err = copy_to_user((void __user *)entry, &clk_point,
				sizeof(clk_point));

		num_points++;
		entry++;
	}

	args->num_entries = num_points;

fail:
	nvgpu_kfree(g, fpoints);
	return err;
}

static int nvgpu_gpu_clk_get_range(struct gk20a *g,
		struct gk20a_ctrl_priv *priv,
		struct nvgpu_gpu_clk_range_args *args)
{
	struct nvgpu_gpu_clk_range clk_range;
	struct nvgpu_gpu_clk_range __user *entry;
	struct nvgpu_clk_session *session = priv->clk_session;

	u32 clk_domains = 0;
	u32 num_domains;
	u32 num_entries;
	u32 i;
	int bit;
	int err;
	u16 min_mhz, max_mhz;

	gk20a_dbg_fn("");

	if (!session)
		return -EINVAL;

	clk_domains = nvgpu_clk_arb_get_arbiter_clk_domains(g);
	num_domains = hweight_long(clk_domains);

	if (!args->flags) {
		if (!args->num_entries) {
			args->num_entries = num_domains;
			return 0;
		}

		if (args->num_entries < num_domains)
			return -EINVAL;

		args->num_entries = 0;
		num_entries = num_domains;

	} else {
		if (args->flags != NVGPU_GPU_CLK_FLAG_SPECIFIC_DOMAINS)
			return -EINVAL;

		num_entries = args->num_entries;
		if (num_entries > num_domains)
			return -EINVAL;
	}

	entry = (struct nvgpu_gpu_clk_range __user *)
			(uintptr_t)args->clk_range_entries;

	for (i = 0; i < num_entries; i++, entry++) {

		if (args->flags == NVGPU_GPU_CLK_FLAG_SPECIFIC_DOMAINS) {
			if (copy_from_user(&clk_range, (void __user *)entry,
					sizeof(clk_range)))
				return -EFAULT;
		} else {
			bit = ffs(clk_domains) - 1;
			clk_range.clk_domain = bit;
			clk_domains &= ~BIT(bit);
		}

		clk_range.flags = 0;
		err = nvgpu_clk_arb_get_arbiter_clk_range(g,
				clk_range.clk_domain,
				&min_mhz, &max_mhz);
		clk_range.min_hz = MHZ_TO_HZ(min_mhz);
		clk_range.max_hz = MHZ_TO_HZ(max_mhz);

		if (err)
			return err;

		err = copy_to_user(entry, &clk_range, sizeof(clk_range));
		if (err)
			return -EFAULT;
	}

	args->num_entries = num_entries;

	return 0;
}


static int nvgpu_gpu_clk_set_info(struct gk20a *g,
		struct gk20a_ctrl_priv *priv,
		struct nvgpu_gpu_clk_set_info_args *args)
{
	struct nvgpu_gpu_clk_info clk_info;
	struct nvgpu_gpu_clk_info __user *entry;
	struct nvgpu_clk_session *session = priv->clk_session;

	int fd;
	u32 clk_domains = 0;
	u16 freq_mhz;
	int i;
	int ret;

	gk20a_dbg_fn("");

	if (!session || args->flags)
		return -EINVAL;

	gk20a_dbg_info("line=%d", __LINE__);

	clk_domains = nvgpu_clk_arb_get_arbiter_clk_domains(g);
	if (!clk_domains)
		return -EINVAL;

	entry = (struct nvgpu_gpu_clk_info __user *)
			(uintptr_t)args->clk_info_entries;

	gk20a_dbg_info("line=%d", __LINE__);

	for (i = 0; i < args->num_entries; i++, entry++) {

	gk20a_dbg_info("line=%d", __LINE__);
		if (copy_from_user(&clk_info, entry, sizeof(clk_info)))
			return -EFAULT;

	gk20a_dbg_info("i=%d domain=0x%08x", i, clk_info.clk_domain);

		if (!nvgpu_clk_arb_is_valid_domain(g, clk_info.clk_domain))
			return -EINVAL;
	}

	entry = (struct nvgpu_gpu_clk_info __user *)
			(uintptr_t)args->clk_info_entries;

	ret = nvgpu_clk_arb_install_request_fd(g, session, &fd);
	if (ret < 0)
		return ret;

	for (i = 0; i < args->num_entries; i++, entry++) {

		if (copy_from_user(&clk_info, (void __user *)entry,
				sizeof(clk_info)))
			return -EFAULT;
		freq_mhz = HZ_TO_MHZ(clk_info.freq_hz);

		nvgpu_clk_arb_set_session_target_mhz(session, fd,
				clk_info.clk_domain, freq_mhz);
	}

	ret = nvgpu_clk_arb_commit_request_fd(g, session, fd);
	if (ret < 0)
		return ret;

	args->completion_fd = fd;

	return ret;
}

static int nvgpu_gpu_clk_get_info(struct gk20a *g,
		struct gk20a_ctrl_priv *priv,
		struct nvgpu_gpu_clk_get_info_args *args)
{
	struct nvgpu_gpu_clk_info clk_info;
	struct nvgpu_gpu_clk_info __user *entry;
	struct nvgpu_clk_session *session = priv->clk_session;
	u32 clk_domains = 0;
	u32 num_domains;
	u32 num_entries;
	u32 i;
	u16 freq_mhz;
	int err;
	int bit;

	gk20a_dbg_fn("");

	if (!session)
		return -EINVAL;

	clk_domains = nvgpu_clk_arb_get_arbiter_clk_domains(g);
	num_domains = hweight_long(clk_domains);

	if (!args->flags) {
		if (!args->num_entries) {
			args->num_entries = num_domains;
			return 0;
		}

		if (args->num_entries < num_domains)
			return -EINVAL;

		args->num_entries = 0;
		num_entries = num_domains;

	} else {
		if (args->flags != NVGPU_GPU_CLK_FLAG_SPECIFIC_DOMAINS)
			return -EINVAL;

		num_entries = args->num_entries;
		if (num_entries > num_domains * 3)
			return -EINVAL;
	}

	entry = (struct nvgpu_gpu_clk_info __user *)
			(uintptr_t)args->clk_info_entries;

	for (i = 0; i < num_entries; i++, entry++) {

		if (args->flags == NVGPU_GPU_CLK_FLAG_SPECIFIC_DOMAINS) {
			if (copy_from_user(&clk_info, (void __user *)entry,
					sizeof(clk_info)))
				return -EFAULT;
		} else {
			bit = ffs(clk_domains) - 1;
			clk_info.clk_domain = bit;
			clk_domains &= ~BIT(bit);
			clk_info.clk_type = args->clk_type;
		}

		switch (clk_info.clk_type) {
		case NVGPU_GPU_CLK_TYPE_TARGET:
			err = nvgpu_clk_arb_get_session_target_mhz(session,
					clk_info.clk_domain, &freq_mhz);
			break;
		case NVGPU_GPU_CLK_TYPE_ACTUAL:
			err = nvgpu_clk_arb_get_arbiter_actual_mhz(g,
					clk_info.clk_domain, &freq_mhz);
			break;
		case NVGPU_GPU_CLK_TYPE_EFFECTIVE:
			err = nvgpu_clk_arb_get_arbiter_effective_mhz(g,
					clk_info.clk_domain, &freq_mhz);
			break;
		default:
			freq_mhz = 0;
			err = -EINVAL;
			break;
		}
		if (err)
			return err;

		clk_info.flags = 0;
		clk_info.freq_hz = MHZ_TO_HZ(freq_mhz);

		err = copy_to_user((void __user *)entry, &clk_info,
				sizeof(clk_info));
		if (err)
			return -EFAULT;
	}

	args->num_entries = num_entries;

	return 0;
}

static int nvgpu_gpu_get_event_fd(struct gk20a *g,
	struct gk20a_ctrl_priv *priv,
	struct nvgpu_gpu_get_event_fd_args *args)
{
	struct nvgpu_clk_session *session = priv->clk_session;

	gk20a_dbg_fn("");

	if (!session)
		return -EINVAL;

	return nvgpu_clk_arb_install_event_fd(g, session, &args->event_fd,
		args->flags);
}

static int nvgpu_gpu_get_voltage(struct gk20a *g,
		struct nvgpu_gpu_get_voltage_args *args)
{
	int err = -EINVAL;

	gk20a_dbg_fn("");

	if (args->reserved)
		return -EINVAL;

	if (!(g->gpu_characteristics.flags & NVGPU_GPU_FLAGS_SUPPORT_GET_VOLTAGE))
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
	    return err;

	switch (args->which) {
	case NVGPU_GPU_VOLTAGE_CORE:
		err = volt_get_voltage(g, CTRL_VOLT_DOMAIN_LOGIC, &args->voltage);
		break;
	case NVGPU_GPU_VOLTAGE_SRAM:
		err = volt_get_voltage(g, CTRL_VOLT_DOMAIN_SRAM, &args->voltage);
		break;
	case NVGPU_GPU_VOLTAGE_BUS:
		err = pmgr_pwr_devices_get_voltage(g, &args->voltage);
		break;
	default:
		err = -EINVAL;
	}

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_get_current(struct gk20a *g,
		struct nvgpu_gpu_get_current_args *args)
{
	int err;

	gk20a_dbg_fn("");

	if (args->reserved[0] || args->reserved[1] || args->reserved[2])
		return -EINVAL;

	if (!(g->gpu_characteristics.flags & NVGPU_GPU_FLAGS_SUPPORT_GET_CURRENT))
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = pmgr_pwr_devices_get_current(g, &args->currnt);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_get_power(struct gk20a *g,
		struct nvgpu_gpu_get_power_args *args)
{
	int err;

	gk20a_dbg_fn("");

	if (args->reserved[0] || args->reserved[1] || args->reserved[2])
		return -EINVAL;

	if (!(g->gpu_characteristics.flags & NVGPU_GPU_FLAGS_SUPPORT_GET_POWER))
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = pmgr_pwr_devices_get_power(g, &args->power);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_get_temperature(struct gk20a *g,
		struct nvgpu_gpu_get_temperature_args *args)
{
	int err;
	u32 temp_f24_8;

	gk20a_dbg_fn("");

	if (args->reserved[0] || args->reserved[1] || args->reserved[2])
		return -EINVAL;

	if (!g->ops.therm.get_internal_sensor_curr_temp)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = g->ops.therm.get_internal_sensor_curr_temp(g, &temp_f24_8);

	gk20a_idle(g);

	args->temp_f24_8 = (s32)temp_f24_8;

	return err;
}
#endif

static int nvgpu_gpu_set_therm_alert_limit(struct gk20a *g,
		struct nvgpu_gpu_set_therm_alert_limit_args *args)
{
	int err;

	gk20a_dbg_fn("");

	if (args->reserved[0] || args->reserved[1] || args->reserved[2])
		return -EINVAL;

	if (!g->ops.therm.configure_therm_alert)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = g->ops.therm.configure_therm_alert(g, args->temp_f24_8);

	gk20a_idle(g);

	return err;
}

long gk20a_ctrl_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gk20a_ctrl_priv *priv = filp->private_data;
	struct gk20a *g = priv->g;
	struct nvgpu_gpu_zcull_get_ctx_size_args *get_ctx_size_args;
	struct nvgpu_gpu_zcull_get_info_args *get_info_args;
	struct nvgpu_gpu_zbc_set_table_args *set_table_args;
	struct nvgpu_gpu_zbc_query_table_args *query_table_args;
	u8 buf[NVGPU_GPU_IOCTL_MAX_ARG_SIZE];
	struct gr_zcull_info *zcull_info;
	struct zbc_entry *zbc_val;
	struct zbc_query_params *zbc_tbl;
	int i, err = 0;

	gk20a_dbg_fn("start %d", _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != NVGPU_GPU_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVGPU_GPU_IOCTL_LAST) ||
		(_IOC_SIZE(cmd) > NVGPU_GPU_IOCTL_MAX_ARG_SIZE))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	if (!g->gr.sw_ready) {
		err = gk20a_busy(g);
		if (err)
			return err;

		gk20a_idle(g);
	}

	switch (cmd) {
	case NVGPU_GPU_IOCTL_ZCULL_GET_CTX_SIZE:
		get_ctx_size_args = (struct nvgpu_gpu_zcull_get_ctx_size_args *)buf;

		get_ctx_size_args->size = gr_gk20a_get_ctxsw_zcull_size(g, &g->gr);

		break;
	case NVGPU_GPU_IOCTL_ZCULL_GET_INFO:
		get_info_args = (struct nvgpu_gpu_zcull_get_info_args *)buf;

		memset(get_info_args, 0, sizeof(struct nvgpu_gpu_zcull_get_info_args));

		zcull_info = nvgpu_kzalloc(g, sizeof(struct gr_zcull_info));
		if (zcull_info == NULL)
			return -ENOMEM;

		err = g->ops.gr.get_zcull_info(g, &g->gr, zcull_info);
		if (err) {
			nvgpu_kfree(g, zcull_info);
			break;
		}

		get_info_args->width_align_pixels = zcull_info->width_align_pixels;
		get_info_args->height_align_pixels = zcull_info->height_align_pixels;
		get_info_args->pixel_squares_by_aliquots = zcull_info->pixel_squares_by_aliquots;
		get_info_args->aliquot_total = zcull_info->aliquot_total;
		get_info_args->region_byte_multiplier = zcull_info->region_byte_multiplier;
		get_info_args->region_header_size = zcull_info->region_header_size;
		get_info_args->subregion_header_size = zcull_info->subregion_header_size;
		get_info_args->subregion_width_align_pixels = zcull_info->subregion_width_align_pixels;
		get_info_args->subregion_height_align_pixels = zcull_info->subregion_height_align_pixels;
		get_info_args->subregion_count = zcull_info->subregion_count;

		nvgpu_kfree(g, zcull_info);
		break;
	case NVGPU_GPU_IOCTL_ZBC_SET_TABLE:
		set_table_args = (struct nvgpu_gpu_zbc_set_table_args *)buf;

		zbc_val = nvgpu_kzalloc(g, sizeof(struct zbc_entry));
		if (zbc_val == NULL)
			return -ENOMEM;

		zbc_val->format = set_table_args->format;
		zbc_val->type = set_table_args->type;

		switch (zbc_val->type) {
		case GK20A_ZBC_TYPE_COLOR:
			for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
				zbc_val->color_ds[i] = set_table_args->color_ds[i];
				zbc_val->color_l2[i] = set_table_args->color_l2[i];
			}
			break;
		case GK20A_ZBC_TYPE_DEPTH:
		case T19X_ZBC:
			zbc_val->depth = set_table_args->depth;
			break;
		default:
			err = -EINVAL;
		}

		if (!err) {
			err = gk20a_busy(g);
			if (!err) {
				err = g->ops.gr.zbc_set_table(g, &g->gr,
							     zbc_val);
				gk20a_idle(g);
			}
		}

		if (zbc_val)
			nvgpu_kfree(g, zbc_val);
		break;
	case NVGPU_GPU_IOCTL_ZBC_QUERY_TABLE:
		query_table_args = (struct nvgpu_gpu_zbc_query_table_args *)buf;

		zbc_tbl = nvgpu_kzalloc(g, sizeof(struct zbc_query_params));
		if (zbc_tbl == NULL)
			return -ENOMEM;

		zbc_tbl->type = query_table_args->type;
		zbc_tbl->index_size = query_table_args->index_size;

		err = g->ops.gr.zbc_query_table(g, &g->gr, zbc_tbl);

		if (!err) {
			switch (zbc_tbl->type) {
			case GK20A_ZBC_TYPE_COLOR:
				for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
					query_table_args->color_ds[i] = zbc_tbl->color_ds[i];
					query_table_args->color_l2[i] = zbc_tbl->color_l2[i];
				}
				break;
			case GK20A_ZBC_TYPE_DEPTH:
			case T19X_ZBC:
				query_table_args->depth = zbc_tbl->depth;
				break;
			case GK20A_ZBC_TYPE_INVALID:
				query_table_args->index_size = zbc_tbl->index_size;
				break;
			default:
				err = -EINVAL;
			}
			if (!err) {
				query_table_args->format = zbc_tbl->format;
				query_table_args->ref_cnt = zbc_tbl->ref_cnt;
			}
		}

		if (zbc_tbl)
			nvgpu_kfree(g, zbc_tbl);
		break;

	case NVGPU_GPU_IOCTL_GET_CHARACTERISTICS:
		err = gk20a_ctrl_ioctl_gpu_characteristics(
			g, (struct nvgpu_gpu_get_characteristics *)buf);
		break;
	case NVGPU_GPU_IOCTL_PREPARE_COMPRESSIBLE_READ:
		err = gk20a_ctrl_prepare_compressible_read(g,
			(struct nvgpu_gpu_prepare_compressible_read_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_MARK_COMPRESSIBLE_WRITE:
		err = gk20a_ctrl_mark_compressible_write(g,
			(struct nvgpu_gpu_mark_compressible_write_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_ALLOC_AS:
		err = gk20a_ctrl_alloc_as(g,
			(struct nvgpu_alloc_as_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_OPEN_TSG:
		err = gk20a_ctrl_open_tsg(g,
			(struct nvgpu_gpu_open_tsg_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_GET_TPC_MASKS:
		err = gk20a_ctrl_get_tpc_masks(g,
			(struct nvgpu_gpu_get_tpc_masks_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_GET_FBP_L2_MASKS:
		err = gk20a_ctrl_get_fbp_l2_masks(g,
			(struct nvgpu_gpu_get_fbp_l2_masks_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_OPEN_CHANNEL:
		/* this arg type here, but ..gpu_open_channel_args in nvgpu.h
		 * for consistency - they are the same */
		err = gk20a_channel_open_ioctl(g,
			(struct nvgpu_channel_open_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_FLUSH_L2:
		err = nvgpu_gpu_ioctl_l2_fb_ops(g,
			   (struct nvgpu_gpu_l2_fb_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_INVAL_ICACHE:
		err = gr_gk20a_elpg_protected_call(g,
				nvgpu_gpu_ioctl_inval_icache(g, (struct nvgpu_gpu_inval_icache_args *)buf));
		break;

	case NVGPU_GPU_IOCTL_SET_MMUDEBUG_MODE:
		err =  nvgpu_gpu_ioctl_set_mmu_debug_mode(g,
				(struct nvgpu_gpu_mmu_debug_mode_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_SET_SM_DEBUG_MODE:
		err = gr_gk20a_elpg_protected_call(g,
				nvgpu_gpu_ioctl_set_debug_mode(g, (struct nvgpu_gpu_sm_debug_mode_args *)buf));
		break;

	case NVGPU_GPU_IOCTL_TRIGGER_SUSPEND:
		err = nvgpu_gpu_ioctl_trigger_suspend(g);
		break;

	case NVGPU_GPU_IOCTL_WAIT_FOR_PAUSE:
		err = nvgpu_gpu_ioctl_wait_for_pause(g,
				(struct nvgpu_gpu_wait_pause_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_RESUME_FROM_PAUSE:
		err = nvgpu_gpu_ioctl_resume_from_pause(g);
		break;

	case NVGPU_GPU_IOCTL_CLEAR_SM_ERRORS:
		err = nvgpu_gpu_ioctl_clear_sm_errors(g);
		break;

	case NVGPU_GPU_IOCTL_GET_TPC_EXCEPTION_EN_STATUS:
		err =  nvgpu_gpu_ioctl_has_any_exception(g,
				(struct nvgpu_gpu_tpc_exception_en_status_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_NUM_VSMS:
		err = gk20a_ctrl_get_num_vsms(g,
			(struct nvgpu_gpu_num_vsms *)buf);
		break;
	case NVGPU_GPU_IOCTL_VSMS_MAPPING:
		err = gk20a_ctrl_vsm_mapping(g,
			(struct nvgpu_gpu_vsms_mapping *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_BUFFER_INFO:
		err = gk20a_ctrl_get_buffer_info(g,
			(struct nvgpu_gpu_get_buffer_info_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_CPU_TIME_CORRELATION_INFO:
		err = nvgpu_gpu_get_cpu_time_correlation_info(g,
			(struct nvgpu_gpu_get_cpu_time_correlation_info_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_GPU_TIME:
		err = nvgpu_gpu_get_gpu_time(g,
			(struct nvgpu_gpu_get_gpu_time_args *)buf);
		break;

        case NVGPU_GPU_IOCTL_GET_ENGINE_INFO:
		err = nvgpu_gpu_get_engine_info(g,
			(struct nvgpu_gpu_get_engine_info_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_ALLOC_VIDMEM:
		err = nvgpu_gpu_alloc_vidmem(g,
			(struct nvgpu_gpu_alloc_vidmem_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_MEMORY_STATE:
		err = nvgpu_gpu_get_memory_state(g,
			(struct nvgpu_gpu_get_memory_state_args *)buf);
		break;

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	case NVGPU_GPU_IOCTL_CLK_GET_RANGE:
		err = nvgpu_gpu_clk_get_range(g, priv,
			(struct nvgpu_gpu_clk_range_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_CLK_GET_VF_POINTS:
		err = nvgpu_gpu_clk_get_vf_points(g, priv,
			(struct nvgpu_gpu_clk_vf_points_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_CLK_SET_INFO:
		err = nvgpu_gpu_clk_set_info(g, priv,
			(struct nvgpu_gpu_clk_set_info_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_CLK_GET_INFO:
		err = nvgpu_gpu_clk_get_info(g, priv,
			(struct nvgpu_gpu_clk_get_info_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_EVENT_FD:
		err = nvgpu_gpu_get_event_fd(g, priv,
			(struct nvgpu_gpu_get_event_fd_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_VOLTAGE:
		err = nvgpu_gpu_get_voltage(g,
			(struct nvgpu_gpu_get_voltage_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_CURRENT:
		err = nvgpu_gpu_get_current(g,
			(struct nvgpu_gpu_get_current_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_POWER:
		err = nvgpu_gpu_get_power(g,
			(struct nvgpu_gpu_get_power_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_TEMPERATURE:
		err = nvgpu_gpu_get_temperature(g,
			(struct nvgpu_gpu_get_temperature_args *)buf);
		break;
#endif

	case NVGPU_GPU_IOCTL_SET_THERM_ALERT_LIMIT:
		err = nvgpu_gpu_set_therm_alert_limit(g,
			(struct nvgpu_gpu_set_therm_alert_limit_args *)buf);
		break;

	default:
		gk20a_dbg_info("unrecognized gpu ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}
