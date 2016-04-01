/*
 * NVIDIA Tegra Video Input Device
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/nvhost.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/lcm.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/camera_common.h>

#include <mach/clk.h>
#include <mach/io_dpd.h>

#include "camera/mc_common.h"
#include "vi/vi.h"

extern int _vb2_fop_release(struct file *file, struct mutex *lock);

void tegra_channel_write(struct tegra_channel *chan,
			unsigned int addr, u32 val)
{
	writel(val, chan->vi->iomem + addr);
}

/* CSI registers */
static void csi_write(struct tegra_channel *chan, unsigned int index,
			unsigned int addr, u32 val)
{
	writel(val, chan->csibase[index] + addr);
}

static u32 csi_read(struct tegra_channel *chan, unsigned int index,
					unsigned int addr)
{
	return readl(chan->csibase[index] + addr);
}

static void gang_buffer_offsets(struct tegra_channel *chan)
{
	int i;
	u32 offset = 0;

	for (i = 0; i < chan->total_ports; i++) {
		switch (chan->gang_mode) {
		case CAMERA_NO_GANG_MODE:
		case CAMERA_GANG_L_R:
		case CAMERA_GANG_R_L:
			offset = chan->gang_bytesperline;
			break;
		case CAMERA_GANG_T_B:
		case CAMERA_GANG_B_T:
			offset = chan->gang_sizeimage;
			break;
		default:
			offset = 0;
		}
		offset = ((offset + TEGRA_SURFACE_ALIGNMENT - 1) &
					~(TEGRA_SURFACE_ALIGNMENT - 1));
		chan->buffer_offset[i] = i * offset;
	}
}

static u32 gang_mode_width(enum camera_gang_mode gang_mode,
					unsigned int width)
{
	if ((gang_mode == CAMERA_GANG_L_R) ||
		(gang_mode == CAMERA_GANG_R_L))
		return width >> 1;
	else
		return width;
}

static u32 gang_mode_height(enum camera_gang_mode gang_mode,
					unsigned int height)
{
	if ((gang_mode == CAMERA_GANG_T_B) ||
		(gang_mode == CAMERA_GANG_B_T))
		return height >> 1;
	else
		return height;
}

static void update_gang_mode_params(struct tegra_channel *chan)
{
	chan->gang_width = gang_mode_width(chan->gang_mode,
						chan->format.width);
	chan->gang_height = gang_mode_height(chan->gang_mode,
						chan->format.height);
	chan->gang_bytesperline = chan->gang_width *
					chan->fmtinfo->bpp;
	chan->gang_sizeimage = chan->gang_bytesperline *
					chan->format.height;
	gang_buffer_offsets(chan);
}

static void update_gang_mode(struct tegra_channel *chan)
{
	int width = chan->format.width;
	int height = chan->format.height;

	/*
	 * At present only 720p, 1080p and 4k resolutions
	 * are supported and only 4K requires gang mode
	 * Update this code with CID for future extensions
	 * Also, validate width and height of images based
	 * on gang mode and surface stride alignment
	 */
	if ((width > 1920) && (height > 1080)) {
		chan->gang_mode = CAMERA_GANG_L_R;
		chan->valid_ports = chan->total_ports;
	} else {
		chan->gang_mode = CAMERA_NO_GANG_MODE;
		chan->valid_ports = 1;
	}

	update_gang_mode_params(chan);
}

static void tegra_channel_fmts_bitmap_init(struct tegra_channel *chan)
{
	int ret, pixel_format_index = 0, init_code = 0;
	struct v4l2_subdev *subdev = chan->subdev[0];
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	bitmap_zero(chan->fmts_bitmap, MAX_FORMAT_NUM);

	/*
	 * Initialize all the formats available from
	 * the sub-device and extract the corresponding
	 * index from the pre-defined video formats and initialize
	 * the channel default format with the active code
	 * Index zero as the only sub-device is sensor
	 */
	while (1) {
		ret = v4l2_subdev_call(subdev, pad, enum_mbus_code,
				       NULL, &code);
		if (ret < 0)
			/* no more formats */
			break;

		pixel_format_index = tegra_core_get_idx_by_code(code.code);
		if (pixel_format_index >= 0) {
			bitmap_set(chan->fmts_bitmap, pixel_format_index, 1);
			if (!init_code)
				init_code = code.code;
		}

		code.index++;
	}

	if (!init_code) {
		pixel_format_index = tegra_core_get_idx_by_code(TEGRA_VF_DEF);
		if (pixel_format_index >= 0) {
			bitmap_set(chan->fmts_bitmap, pixel_format_index, 1);
			init_code = TEGRA_VF_DEF;
		}
	}
		/* Get the format based on active code of the sub-device */
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret)
		return;

	chan->fmtinfo = tegra_core_get_format_by_code(mbus_fmt.code);
	v4l2_fill_pix_format(&chan->format, &fmt.format);
	chan->format.pixelformat = chan->fmtinfo->fourcc;
	chan->format.bytesperline = chan->format.width *
					chan->fmtinfo->bpp;
	chan->format.sizeimage = chan->format.bytesperline *
		chan->format.height;
	if (chan->total_ports > 1)
		update_gang_mode(chan);
}

/*
 * -----------------------------------------------------------------------------
 * Tegra channel frame setup and capture operations
 * -----------------------------------------------------------------------------
 */

static int tegra_channel_capture_setup(struct tegra_channel *chan)
{
	u32 height = chan->format.height;
	u32 width = chan->format.width;
	u32 format = chan->fmtinfo->img_fmt;
	u32 data_type = chan->fmtinfo->img_dt;
	u32 word_count = tegra_core_get_word_count(width, chan->fmtinfo);
	u32 bypass_pixel_transform = 1;
	int index;

	if (chan->valid_ports > 1) {
		height = chan->gang_height;
		width = chan->gang_width;
		word_count = tegra_core_get_word_count(width, chan->fmtinfo);
	}

	if (chan->vi->pg_mode ||
	   (chan->fmtinfo->vf_code == TEGRA_VF_YUV422) ||
	   (chan->fmtinfo->vf_code == TEGRA_VF_RGB888))
		bypass_pixel_transform = 0;

	for (index = 0; index < chan->valid_ports; index++) {
		csi_write(chan, index, TEGRA_VI_CSI_ERROR_STATUS, 0xFFFFFFFF);
		csi_write(chan, index, TEGRA_VI_CSI_IMAGE_DEF,
		  (bypass_pixel_transform << BYPASS_PXL_TRANSFORM_OFFSET) |
		  (format << IMAGE_DEF_FORMAT_OFFSET) |
		  IMAGE_DEF_DEST_MEM);
		csi_write(chan, index, TEGRA_VI_CSI_IMAGE_DT, data_type);
		csi_write(chan, index, TEGRA_VI_CSI_IMAGE_SIZE_WC, word_count);
		csi_write(chan, index, TEGRA_VI_CSI_IMAGE_SIZE,
			  (height << IMAGE_SIZE_HEIGHT_OFFSET) | width);
	}

	return 0;
}

static void tegra_channel_capture_error(struct tegra_channel *chan)
{
	u32 val;
	int index = 0;

	for (index = 0; index < chan->valid_ports; index++) {
		val = csi_read(chan, index, TEGRA_VI_CSI_ERROR_STATUS);
		dev_err(&chan->video.dev,
			"TEGRA_VI_CSI_ERROR_STATUS 0x%08x\n", val);
		tegra_csi_status(chan->vi->csi, chan->port[index]);
	}
}

static void tegra_channel_capture_frame(struct tegra_channel *chan,
				       struct tegra_channel_buffer *buf)
{
	int err = 0;
	u32 val, frame_start;
	int bytes_per_line = chan->format.bytesperline;
	int index = 0;
	u32 thresh[TEGRA_CSI_BLOCKS] = { 0 };
	int valid_ports = chan->valid_ports;

	for (index = 0; index < valid_ports; index++) {
		/* Program buffer address by using surface 0 */
		csi_write(chan, index, TEGRA_VI_CSI_SURFACE0_OFFSET_MSB, 0x0);
		csi_write(chan, index,
			TEGRA_VI_CSI_SURFACE0_OFFSET_LSB,
			(buf->addr + chan->buffer_offset[index]));
		csi_write(chan, index,
			TEGRA_VI_CSI_SURFACE0_STRIDE, bytes_per_line);

		/* Program syncpoint */
		thresh[index] = nvhost_syncpt_incr_max_ext(chan->vi->ndev,
					chan->syncpt[index], 1);
		frame_start = VI_CSI_PP_FRAME_START(chan->port[index]);
		val = VI_CFG_VI_INCR_SYNCPT_COND(frame_start) |
				chan->syncpt[index];
		tegra_channel_write(chan, TEGRA_VI_CFG_VI_INCR_SYNCPT, val);

		csi_write(chan, index,
			TEGRA_VI_CSI_SINGLE_SHOT, SINGLE_SHOT_CAPTURE);
	}

	for (index = 0; index < valid_ports; index++) {
		err = nvhost_syncpt_wait_timeout_ext(chan->vi->ndev,
			chan->syncpt[index], thresh[index],
			TEGRA_VI_SYNCPT_WAIT_TIMEOUT,
			NULL,
			NULL);
		if (err) {
			dev_err(&chan->video.dev,
				"frame start syncpt timeout!%d\n", index);
			tegra_channel_capture_error(chan);
		}
	}

	if (atomic_read(&chan->is_hdmiin_unplug)) {
		vb2_buffer_done(&buf->buf, VB2_BUF_STATE_ERROR);
		complete(&chan->capture_comp);
	} else {
		/* Move buffer to capture done queue */
		spin_lock(&chan->done_lock);
		list_add_tail(&buf->queue, &chan->done);
		spin_unlock(&chan->done_lock);

		/* Wait up kthread for capture done */
		wake_up_interruptible(&chan->done_wait);
	}
}

static void tegra_channel_capture_done(struct tegra_channel *chan,
				       struct tegra_channel_buffer *buf)
{
	struct vb2_v4l2_buffer *vb = &buf->buf;
	int err = 0;
	u32 val, mw_ack_done;
	int index = 0;
	u32 thresh[TEGRA_CSI_BLOCKS] = { 0 };
	int valid_ports = chan->valid_ports;

	for (index = 0; index < valid_ports; index++) {
		/* Program syncpoint */
		thresh[index] = nvhost_syncpt_incr_max_ext(chan->vi->ndev,
				chan->syncpt[index], 1);

		mw_ack_done = VI_CSI_MW_ACK_DONE(chan->port[index]);
		val = VI_CFG_VI_INCR_SYNCPT_COND(mw_ack_done) |
				chan->syncpt[index];
		tegra_channel_write(chan, TEGRA_VI_CFG_VI_INCR_SYNCPT, val);
	}

	for (index = 0; index < valid_ports; index++) {
		err = nvhost_syncpt_wait_timeout_ext(chan->vi->ndev,
			chan->syncpt[index], thresh[index],
			TEGRA_VI_SYNCPT_WAIT_TIMEOUT,
			NULL,
			NULL);
		if (err)
			dev_err(&chan->video.dev,
				"MW_ACK_DONE syncpoint time out!%d\n", index);
	}

	if (atomic_read(&chan->is_hdmiin_unplug)) {
		vb2_buffer_done(&vb->vb2_buf, err < 0 ? VB2_BUF_STATE_ERROR :
				VB2_BUF_STATE_DONE);
		complete(&chan->done_comp);
	} else {
		/* Captured one frame */
		vb->sequence = chan->sequence++;
		vb->field = V4L2_FIELD_NONE;
		v4l2_get_timestamp(&vb->timestamp);
		vb2_set_plane_payload(&vb->vb2_buf, 0, chan->format.sizeimage);
		vb2_buffer_done(&vb->vb2_buf, err < 0 ? VB2_BUF_STATE_ERROR
				: VB2_BUF_STATE_DONE);
	}
}

static int tegra_channel_kthread_capture_start(void *data)
{
	struct tegra_channel *chan = data;
	struct tegra_channel_buffer *buf;

	set_freezable();

	while (1) {
		try_to_freeze();
		wait_event_interruptible(chan->start_wait,
					 !list_empty(&chan->capture) ||
					 kthread_should_stop() ||
					 atomic_read(&chan->is_hdmiin_unplug));
		if (kthread_should_stop() ||
		    atomic_read(&chan->is_hdmiin_unplug)) {
			complete(&chan->capture_comp);
			break;
		}

		spin_lock(&chan->start_lock);
		if (list_empty(&chan->capture)) {
			spin_unlock(&chan->start_lock);
			continue;
		}

		buf = list_entry(chan->capture.next,
				 struct tegra_channel_buffer, queue);
		list_del_init(&buf->queue);
		spin_unlock(&chan->start_lock);

		tegra_channel_capture_frame(chan, buf);
	}

	return 0;
}

static int tegra_channel_kthread_capture_done(void *data)
{
	struct tegra_channel *chan = data;
	struct tegra_channel_buffer *buf;

	set_freezable();

	while (1) {
		try_to_freeze();
		wait_event_interruptible(chan->done_wait,
					 !list_empty(&chan->done) ||
					 kthread_should_stop() ||
					 atomic_read(&chan->is_hdmiin_unplug));

		if ((chan->vi->pg_mode &&
			kthread_should_stop()) ||
			atomic_read(&chan->is_hdmiin_unplug)) {
			complete(&chan->done_comp);
			break;
		} else if (kthread_should_stop() &&
			list_empty(&chan->done)) {
			complete(&chan->done_comp);
			break;
		}

		spin_lock(&chan->done_lock);
		if (list_empty(&chan->done)) {
			spin_unlock(&chan->done_lock);
			continue;
		}

		buf = list_entry(chan->done.next,
				 struct tegra_channel_buffer, queue);
		list_del_init(&buf->queue);
		spin_unlock(&chan->done_lock);

		tegra_channel_capture_done(chan, buf);
	}

	return 0;
}

static void tegra_channel_queued_buf_done(struct tegra_channel *chan,
					  enum vb2_buffer_state state,
					  bool is_capture);

static void tegra_channel_stop_kthreads(struct tegra_channel *chan)
{

	mutex_lock(&chan->stop_kthread_lock);
	/* Stop the kthread for capture */
	if (chan->kthread_capture_start) {
		kthread_stop(chan->kthread_capture_start);
		wait_for_completion(&chan->capture_comp);
		chan->kthread_capture_start = NULL;
	}
	if (chan->kthread_capture_done) {
		kthread_stop(chan->kthread_capture_done);
		wait_for_completion(&chan->done_comp);
		chan->kthread_capture_done = NULL;
	}
	tegra_channel_queued_buf_done(chan, VB2_BUF_STATE_ERROR, 1);
	tegra_channel_queued_buf_done(chan, VB2_BUF_STATE_ERROR, 0);
	mutex_unlock(&chan->stop_kthread_lock);
}

void tegra_channel_query_hdmiin_unplug(struct tegra_channel *chan,
		struct v4l2_event *event)
{
	struct v4l2_dv_timings timings;
	struct v4l2_bt_timings *bt;
	struct v4l2_subdev *sd;
	bool is_hdmiin = false;
	int num_sd, ret, index = 0;
	int valid_ports = chan->valid_ports;

	if (event->type != V4L2_EVENT_SOURCE_CHANGE)
		return;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		struct v4l2_subdev *sd = chan->subdev[num_sd];
		if (v4l2_subdev_has_op(sd, video, s_dv_timings))
			is_hdmiin = true;
	}

	if (!is_hdmiin)
		return;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		sd = chan->subdev[num_sd];
		ret = v4l2_subdev_call(sd, video, query_dv_timings,
				&timings);

		/* Stop capture threads when unplug HDMI-IN */
		if (sd && (ret == 0 || ret != -ENOIOCTLCMD)) {
			bt = &timings.bt;
			if (bt->width == 0 && bt->height == 0) {
				dev_info(&chan->video.dev,
					 "Got unplug event during capture!\n");

				atomic_set(&chan->is_hdmiin_unplug, 1);
				for (index = 0; index < valid_ports; index++) {
					nvhost_syncpt_cpu_incr_ext(
							chan->vi->ndev,
							chan->syncpt[index]);
					nvhost_syncpt_cpu_incr_ext(
							chan->vi->ndev,
							chan->syncpt[index]);
				}

				tegra_channel_stop_kthreads(chan);
			}
		}
	}
}
EXPORT_SYMBOL(tegra_channel_query_hdmiin_unplug);

/*
 * -----------------------------------------------------------------------------
 * videobuf2 queue operations
 * -----------------------------------------------------------------------------
 */
static int
tegra_channel_queue_setup(struct vb2_queue *vq, const void *parg,
		     unsigned int *nbuffers, unsigned int *nplanes,
		     unsigned int sizes[], void *alloc_ctxs[])
{
	const struct v4l2_format *fmt = parg;
	struct tegra_channel *chan = vb2_get_drv_priv(vq);
	/* Make sure the image size is large enough. */
	if (fmt && fmt->fmt.pix.sizeimage < chan->format.sizeimage)
		return -EINVAL;

	*nplanes = 1;

	sizes[0] = fmt ? fmt->fmt.pix.sizeimage : chan->format.sizeimage;
	alloc_ctxs[0] = chan->alloc_ctx;

	if (!*nbuffers)
		*nbuffers = 2;

	return 0;
}

static int tegra_channel_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct tegra_channel *chan = vb2_get_drv_priv(vb->vb2_queue);
	struct tegra_channel_buffer *buf = to_tegra_channel_buffer(vbuf);

	buf->chan = chan;
	vb2_set_plane_payload(vb, 0, chan->format.sizeimage);
#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	buf->addr = vb2_dma_contig_plane_dma_addr(vb, 0);
#endif

	return 0;
}

static void tegra_channel_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct tegra_channel *chan = vb2_get_drv_priv(vb->vb2_queue);
	struct tegra_channel_buffer *buf = to_tegra_channel_buffer(vbuf);

	/* for bypass mode - do nothing */
	if (chan->bypass)
		return;

	/* Put buffer into the capture queue */
	spin_lock(&chan->start_lock);
	list_add_tail(&buf->queue, &chan->capture);
	spin_unlock(&chan->start_lock);

	/* Wait up kthread for capture */
	wake_up_interruptible(&chan->start_wait);
}

/* Return all queued buffers back to videobuf2 */
static void tegra_channel_queued_buf_done(struct tegra_channel *chan,
					  enum vb2_buffer_state state,
					  bool is_capture)
{
	struct tegra_channel_buffer *buf, *nbuf;
	spinlock_t *lock = is_capture ? &chan->start_lock : &chan->done_lock;
	struct list_head *q = is_capture ? &chan->capture : &chan->done;

	spin_lock(lock);
	list_for_each_entry_safe(buf, nbuf, q, queue) {
		vb2_buffer_done(&buf->buf.vb2_buf, state);
		list_del(&buf->queue);
	}
	spin_unlock(lock);
}

/*
 * -----------------------------------------------------------------------------
 * subdevice set/unset operations
 * -----------------------------------------------------------------------------
 */
static int tegra_channel_set_stream(struct tegra_channel *chan, bool on)
{
	int num_sd = 0;
	int ret = 0;
	struct v4l2_subdev *subdev = chan->subdev[num_sd];

	while (subdev != NULL) {
		ret = v4l2_subdev_call(subdev, video, s_stream, on);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;

		num_sd++;
		if (num_sd >= chan->num_subdevs)
			break;

		subdev = chan->subdev[num_sd];
	}

	return 0;
}

static int tegra_channel_set_power(struct tegra_channel *chan, bool on)
{
	int num_sd = 0;
	int ret = 0;
	struct v4l2_subdev *subdev = chan->subdev[num_sd];

	while (subdev != NULL) {
		ret = v4l2_subdev_call(subdev, core, s_power, on);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;

		num_sd++;
		if (num_sd >= chan->num_subdevs)
			break;

		subdev = chan->subdev[num_sd];
	}

	return 0;
}


static int tegra_channel_start_streaming(struct vb2_queue *vq, u32 count)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);
	struct media_pipeline *pipe = chan->video.entity.pipe;
	int ret = 0, i;

	if (chan->vi->pg_mode)
		for (i = 0; i < chan->valid_ports; i++)
			tegra_csi_tpg_start_streaming(chan->vi->csi,
						      chan->port[i]);
	else {
		ret = media_entity_pipeline_start(&chan->video.entity, pipe);
		if (ret < 0)
			goto error_pipeline_start;

		/* Start the pipeline. */
		ret = tegra_channel_set_stream(chan, true);
		if (ret < 0)
			goto error_set_stream;
	}

	if (chan->bypass)
		return ret;

	for (i = 0; i < chan->valid_ports; i++)
		tegra_csi_start_streaming(chan->vi->csi, chan->port[i]);
	/* Note: Program VI registers after TPG, sensors and CSI streaming */
	ret = tegra_channel_capture_setup(chan);
	if (ret < 0)
		goto error_capture_setup;

	chan->sequence = 0;
	atomic_set(&chan->is_hdmiin_unplug, 0);

	/* Start kthread to capture data to buffer */
	chan->kthread_capture_start = kthread_run(
					tegra_channel_kthread_capture_start,
					chan, chan->video.name);
	if (IS_ERR(chan->kthread_capture_start)) {
		dev_err(&chan->video.dev,
			"failed to run kthread for capture start\n");
		ret = PTR_ERR(chan->kthread_capture_start);
		goto error_capture_setup;
	}

	chan->kthread_capture_done = kthread_run(
					tegra_channel_kthread_capture_done,
					chan, chan->video.name);
	if (IS_ERR(chan->kthread_capture_done)) {
		dev_err(&chan->video.dev,
			"failed to run kthread for capture done\n");
		ret = PTR_ERR(chan->kthread_capture_done);
		goto error_capture_setup;
	}

	return 0;

error_capture_setup:
	if (!chan->vi->pg_mode)
		tegra_channel_set_stream(chan, false);
error_set_stream:
	if (!chan->vi->pg_mode)
		media_entity_pipeline_stop(&chan->video.entity);
error_pipeline_start:
	vq->start_streaming_called = 0;
	tegra_channel_queued_buf_done(chan, VB2_BUF_STATE_QUEUED, 1);

	return ret;
}

static void tegra_channel_stop_streaming(struct vb2_queue *vq)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);
	int index;

	if (!chan->bypass) {
		tegra_channel_stop_kthreads(chan);
		for (index = 0; index < chan->valid_ports; index++)
			tegra_csi_stop_streaming(chan->vi->csi,
							chan->port[index]);
	}

	if (!chan->vi->pg_mode) {
		tegra_channel_set_stream(chan, false);
		media_entity_pipeline_stop(&chan->video.entity);
	}

}

static const struct vb2_ops tegra_channel_queue_qops = {
	.queue_setup = tegra_channel_queue_setup,
	.buf_prepare = tegra_channel_buffer_prepare,
	.buf_queue = tegra_channel_buffer_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = tegra_channel_start_streaming,
	.stop_streaming = tegra_channel_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
tegra_channel_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->device_caps |= V4L2_CAP_EXT_PIX_FORMAT;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	strlcpy(cap->driver, "tegra-video", sizeof(cap->driver));
	strlcpy(cap->card, chan->video.name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s:%u",
		 dev_name(chan->vi->dev), chan->port[0]);

	return 0;
}

static int
tegra_channel_enum_format(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	unsigned int index = 0, i;
	unsigned long *fmts_bitmap = NULL;

	if (chan->vi->pg_mode)
		fmts_bitmap = chan->vi->tpg_fmts_bitmap;
	else
		fmts_bitmap = chan->fmts_bitmap;

	if (f->index >= bitmap_weight(fmts_bitmap, MAX_FORMAT_NUM))
		return -EINVAL;

	for (i = 0; i < f->index + 1; i++, index++)
		index = find_next_bit(fmts_bitmap, MAX_FORMAT_NUM, index);

	index -= 1;
	f->pixelformat = tegra_core_get_fourcc_by_idx(index);
	tegra_core_get_description_by_idx(index, f->description);

	return 0;
}

static int
tegra_channel_g_edid(struct file *file, void *fh, struct v4l2_edid *edid)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	int num_sd;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		struct v4l2_subdev *sd = chan->subdev[num_sd];
		int ret = v4l2_subdev_call(sd, pad, get_edid, edid);

		if (sd && (ret == 0 || ret != -ENOIOCTLCMD))
			return ret;
	}
	return -ENOIOCTLCMD;
}

static int
tegra_channel_s_edid(struct file *file, void *fh, struct v4l2_edid *edid)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	int num_sd;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		struct v4l2_subdev *sd = chan->subdev[num_sd];
		int ret = v4l2_subdev_call(sd, pad, set_edid, edid);

		if (sd && (ret == 0 || ret != -ENOIOCTLCMD))
			return ret;
	}
	return -ENOIOCTLCMD;
}

static int
tegra_channel_s_dv_timings(struct file *file, void *fh,
		struct v4l2_dv_timings *timings)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	struct v4l2_bt_timings *bt = &timings->bt;
	int num_sd;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		struct v4l2_subdev *sd = chan->subdev[num_sd];
		int ret = v4l2_subdev_call(sd, video, s_dv_timings, timings);

		if (sd && (ret == 0 || ret != -ENOIOCTLCMD)) {
			if (!ret) {
				chan->format.width = bt->width;
				chan->format.height = bt->height;
				chan->format.bytesperline = bt->width *
					chan->fmtinfo->bpp;
				chan->format.sizeimage = chan->format.bytesperline *
					chan->format.height;
			}

			if (chan->total_ports > 1)
				update_gang_mode(chan);

			return ret;
		}
	}

	return -ENOIOCTLCMD;
}

static int
tegra_channel_g_dv_timings(struct file *file, void *fh,
		struct v4l2_dv_timings *timings)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	int num_sd;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		struct v4l2_subdev *sd = chan->subdev[num_sd];
		int ret = v4l2_subdev_call(sd, video, g_dv_timings, timings);

		if (sd && (ret == 0 || ret != -ENOIOCTLCMD))
			return ret;
	}
	return -ENOIOCTLCMD;
}

static int
tegra_channel_query_dv_timings(struct file *file, void *fh,
		struct v4l2_dv_timings *timings)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	int num_sd;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		struct v4l2_subdev *sd = chan->subdev[num_sd];
		int ret = v4l2_subdev_call(sd, video, query_dv_timings,
				timings);

		if (sd && (ret == 0 || ret != -ENOIOCTLCMD))
			return ret;
	}
	return -ENOIOCTLCMD;
}

static int
tegra_channel_enum_dv_timings(struct file *file, void *fh,
		struct v4l2_enum_dv_timings *timings)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	int num_sd;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		struct v4l2_subdev *sd = chan->subdev[num_sd];
		int ret = v4l2_subdev_call(sd, pad, enum_dv_timings, timings);

		if (sd && (ret == 0 || ret != -ENOIOCTLCMD))
			return ret;
	}
	return -ENOIOCTLCMD;
}

static int
tegra_channel_dv_timings_cap(struct file *file, void *fh,
		struct v4l2_dv_timings_cap *cap)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	int num_sd;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		struct v4l2_subdev *sd = chan->subdev[num_sd];
		int ret = v4l2_subdev_call(sd, pad, dv_timings_cap, cap);

		if (sd && (ret == 0 || ret != -ENOIOCTLCMD))
			return ret;
	}
	return -ENOIOCTLCMD;
}

static void tegra_channel_fmt_align(struct v4l2_pix_format *pix,
			unsigned int channel_align, unsigned int bpp)
{
	unsigned int min_width;
	unsigned int max_width;
	unsigned int min_bpl;
	unsigned int max_bpl;
	unsigned int width;
	unsigned int align;
	unsigned int bpl;

	/* The transfer alignment requirements are expressed in bytes. Compute
	 * the minimum and maximum values, clamp the requested width and convert
	 * it back to pixels.
	 */
	align = lcm(channel_align, bpp);
	min_width = roundup(TEGRA_MIN_WIDTH, align);
	max_width = rounddown(TEGRA_MAX_WIDTH, align);
	width = roundup(pix->width * bpp, align);

	pix->width = clamp(width, min_width, max_width) / bpp;
	pix->height = clamp(pix->height, TEGRA_MIN_HEIGHT, TEGRA_MAX_HEIGHT);

	/* Clamp the requested bytes per line value. If the maximum bytes per
	 * line value is zero, the module doesn't support user configurable line
	 * sizes. Override the requested value with the minimum in that case.
	 */
	min_bpl = pix->width * bpp;
	max_bpl = rounddown(TEGRA_MAX_WIDTH, channel_align);
	bpl = roundup(pix->bytesperline, channel_align);

	pix->bytesperline = clamp(bpl, min_bpl, max_bpl);
	pix->sizeimage = pix->bytesperline * pix->height;
}

static int tegra_channel_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tegra_channel *chan = container_of(ctrl->handler,
				struct tegra_channel, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_VI_BYPASS_MODE:
		if (switch_ctrl_qmenu[ctrl->val] == SWITCH_ON)
			chan->bypass = true;
		else
			chan->bypass = false;
		break;
	default:
		dev_err(&chan->video.dev, "%s:Not valid ctrl\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops channel_ctrl_ops = {
	.s_ctrl	= tegra_channel_s_ctrl,
};

/**
 * By default channel will be in VI mode
 * User space can set it to 0 for working in bypass mode
 */
static const struct v4l2_ctrl_config bypass_mode_ctrl = {
	.ops = &channel_ctrl_ops,
	.id = V4L2_CID_VI_BYPASS_MODE,
	.name = "Bypass Mode",
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.def = 1,
	.min = 0,
	.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
	.menu_skip_mask = 0,
	.qmenu_int = switch_ctrl_qmenu,
};

static int tegra_channel_setup_controls(struct tegra_channel *chan)
{
	int num_sd = 0;
	struct v4l2_subdev *sd = NULL;

	/* Initialize the subdev and controls here at first open */
	sd = chan->subdev[num_sd];
	while ((sd = chan->subdev[num_sd++]) &&
		(num_sd <= chan->num_subdevs)) {
		/* Add control handler for the subdevice */
		v4l2_ctrl_add_handler(&chan->ctrl_handler,
					sd->ctrl_handler, NULL);
		if (chan->ctrl_handler.error)
			dev_err(chan->vi->dev,
				"Failed to add sub-device controls\n");
	}

	/* Add the bypass mode ctrl */
	v4l2_ctrl_new_custom(&chan->ctrl_handler, &bypass_mode_ctrl, NULL);
	if (chan->ctrl_handler.error) {
		dev_err(chan->vi->dev,
			"Failed to add bypass control\n");
		return chan->ctrl_handler.error;
	}

	if (chan->vi->pg_mode) {
		/* Add VI control handler for TPG control */
		v4l2_ctrl_add_handler(&chan->ctrl_handler,
					&chan->vi->ctrl_handler, NULL);
		if (chan->ctrl_handler.error)
			dev_err(chan->vi->dev,
				"Failed to add VI controls\n");
	}

	/* setup the controls */
	return v4l2_ctrl_handler_setup(&chan->ctrl_handler);
}

int tegra_channel_init_subdevices(struct tegra_channel *chan)
{
	struct media_entity *entity;
	struct media_pad *pad;
	int index = 0;
	int num_sd = 0;

	/* set_stream of CSI */
	entity = &chan->video.entity;
	pad = media_entity_remote_pad(&chan->pad);
	if (!pad)
		return -ENODEV;

	entity = pad->entity;
	chan->subdev[num_sd++] = media_entity_to_v4l2_subdev(entity);

	index = pad->index - 1;
	while (index >= 0) {
		pad = &entity->pads[index];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (pad == NULL ||
		    media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;

		if (num_sd >= MAX_SUBDEVICES)
			break;

		entity = pad->entity;
		chan->subdev[num_sd++] = media_entity_to_v4l2_subdev(entity);

		index = pad->index - 1;
	}
	chan->num_subdevs = num_sd;

	/* initialize the available formats */
	if (chan->num_subdevs)
		tegra_channel_fmts_bitmap_init(chan);

	return tegra_channel_setup_controls(chan);
}

static int
__tegra_channel_get_format(struct tegra_channel *chan,
			struct v4l2_pix_format *pix)
{
	struct tegra_video_format const *vfmt;
	struct v4l2_subdev_format fmt;
	int ret = 0;
	struct v4l2_subdev *sd = chan->subdev[0];
	int num_sd = 0;

	while (sd != NULL) {
		memset(&fmt, 0x0, sizeof(fmt));
		fmt.pad = 0;
		ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);
		if (ret) {
			if (ret == -ENOIOCTLCMD) {
				num_sd++;
				if (num_sd < chan->num_subdevs) {
					sd = chan->subdev[num_sd];
					continue;
				} else
					break;
			}
		}

		v4l2_fill_pix_format(pix, &fmt.format);
		vfmt = tegra_core_get_format_by_code(fmt.format.code);
		if (vfmt != NULL) {
			pix->pixelformat = vfmt->fourcc;
			pix->bytesperline = pix->width * vfmt->bpp;
			pix->sizeimage = pix->height * pix->bytesperline;
		}
		return ret;

	}

	return -ENOTTY;
}

static int
tegra_channel_get_format(struct file *file, void *fh,
			struct v4l2_format *format)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	struct v4l2_pix_format *pix = &format->fmt.pix;

	return  __tegra_channel_get_format(chan, pix);
}

static int
tegra_channel_get_format(struct file *file, void *fh,
			struct v4l2_format *format)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	struct v4l2_pix_format *pix = &format->fmt.pix;

	return  __tegra_channel_get_format(chan, pix);
}

static int
__tegra_channel_try_format(struct tegra_channel *chan,
			struct v4l2_pix_format *pix)
{
	const struct tegra_video_format *vfmt;
	struct v4l2_subdev_format fmt;
	int num_sd = 0;
	struct v4l2_subdev *sd = chan->subdev[0];
	int ret = 0;

	/* Use the channel format if pixformat is not supported */
	vfmt = tegra_core_get_format_by_fourcc(pix->pixelformat);
	if (!vfmt) {
		pix->pixelformat = chan->format.pixelformat;
		vfmt = tegra_core_get_format_by_fourcc(pix->pixelformat);
	}

	tegra_channel_fmt_align(pix, chan->align, vfmt->bpp);

	fmt.which = V4L2_SUBDEV_FORMAT_TRY;
	fmt.pad = 0;
	v4l2_fill_mbus_format(&fmt.format, pix, vfmt->code);

	while (sd != NULL) {
		ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &fmt);
		if (ret) {
			if (ret == -ENOIOCTLCMD) {
				num_sd++;
				if (num_sd < chan->num_subdevs) {
					sd = chan->subdev[num_sd];
					continue;
				} else
					break;
			}
		}

		v4l2_fill_pix_format(pix, &fmt.format);
		if (ret)
			pix->bytesperline = pix->width * chan->fmtinfo->bpp;
		else
			pix->bytesperline = pix->width * vfmt->bpp;

		pix->sizeimage = pix->height * pix->bytesperline;

		return ret;
	}

	return -ENOTTY;
}

static int
tegra_channel_try_format(struct file *file, void *fh,
			struct v4l2_format *format)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);

	return  __tegra_channel_try_format(chan, &format->fmt.pix);
}

static int
__tegra_channel_set_format(struct tegra_channel *chan,
			struct v4l2_pix_format *pix)
{
	const struct tegra_video_format *vfmt;
	struct v4l2_subdev_format fmt;
	int num_sd = 0;
	struct v4l2_subdev *sd = chan->subdev[0];
	int ret = 0;

	vfmt = tegra_core_get_format_by_fourcc(pix->pixelformat);

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = 0;
	v4l2_fill_mbus_format(&fmt.format, pix, vfmt->code);

	while (sd != NULL) {
		ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &fmt);
		if (ret) {
			if (ret == -ENOIOCTLCMD) {
				num_sd++;
				if (num_sd < chan->num_subdevs) {
					sd = chan->subdev[num_sd];
					continue;
				} else
					break;
			}
		}

		v4l2_fill_pix_format(pix, &fmt.format);
		pix->bytesperline = pix->width * vfmt->bpp;
		pix->sizeimage = pix->height * pix->bytesperline;

		if (!ret) {
			chan->format = *pix;
			chan->fmtinfo = vfmt;
			if (chan->total_ports > 1)
				update_gang_mode(chan);

		}

		return ret;
	}

	return -ENOTTY;
}

static int
tegra_channel_set_format(struct file *file, void *fh,
			struct v4l2_format *format)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	int ret = 0;

	/* get the supported format by try_fmt */
	ret = __tegra_channel_try_format(chan, &format->fmt.pix);
	if (ret)
		return ret;

	if (vb2_is_busy(&chan->queue))
		return -EBUSY;

	return __tegra_channel_set_format(chan, &format->fmt.pix);
}

static int tegra_channel_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_event_subscribe(fh, sub, 4, NULL);
	}
	return v4l2_ctrl_subscribe_event(fh, sub);
}

static int
tegra_channel_enum_input(struct file *file, void *fh, struct v4l2_input *inp)
{
	struct v4l2_fh *vfh = file->private_data;
	struct tegra_channel *chan = to_tegra_channel(vfh->vdev);
	int num_sd;

	if (inp->index)
		return -EINVAL;

	for (num_sd = 0; num_sd < chan->num_subdevs; num_sd++) {
		struct v4l2_subdev *sd = chan->subdev[num_sd];
		int ret;

		ret = v4l2_subdev_call(sd,
			video, g_input_status, &inp->status);

		if (sd && (ret == 0 || ret != -ENOIOCTLCMD)) {
			if (v4l2_subdev_has_op(sd, video, s_dv_timings))
				inp->capabilities = V4L2_IN_CAP_DV_TIMINGS;

			inp->type = V4L2_INPUT_TYPE_CAMERA;
			if (inp->capabilities == V4L2_IN_CAP_DV_TIMINGS)
				snprintf(inp->name,
					sizeof(inp->name), "HDMI %u",
					chan->port[0]);
			else
				snprintf(inp->name,
					sizeof(inp->name), "Camera %u",
					chan->port[0]);

			return ret;
		}
	}

	return -ENOTTY;
}

static int tegra_channel_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int tegra_channel_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;
	return 0;
}

static const struct v4l2_ioctl_ops tegra_channel_ioctl_ops = {
	.vidioc_querycap		= tegra_channel_querycap,
	.vidioc_enum_fmt_vid_cap	= tegra_channel_enum_format,
	.vidioc_g_fmt_vid_cap		= tegra_channel_get_format,
	.vidioc_s_fmt_vid_cap		= tegra_channel_set_format,
	.vidioc_try_fmt_vid_cap		= tegra_channel_try_format,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_g_edid			= tegra_channel_g_edid,
	.vidioc_s_edid			= tegra_channel_s_edid,
	.vidioc_s_dv_timings		= tegra_channel_s_dv_timings,
	.vidioc_g_dv_timings		= tegra_channel_g_dv_timings,
	.vidioc_query_dv_timings	= tegra_channel_query_dv_timings,
	.vidioc_enum_dv_timings		= tegra_channel_enum_dv_timings,
	.vidioc_dv_timings_cap		= tegra_channel_dv_timings_cap,
	.vidioc_subscribe_event		= tegra_channel_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
	.vidioc_enum_input		= tegra_channel_enum_input,
	.vidioc_g_input			= tegra_channel_g_input,
	.vidioc_s_input			= tegra_channel_s_input,
};

static int tegra_channel_open(struct file *fp)
{
	int ret;
	struct video_device *vdev = video_devdata(fp);
	struct tegra_channel *chan = video_get_drvdata(vdev);
	struct tegra_mc_vi *vi;
	struct tegra_csi_device *csi;

	mutex_lock(&chan->video_lock);
	ret = v4l2_fh_open(fp);
	if (ret || !v4l2_fh_is_singular_file(fp))
		goto unlock;

	if (chan->subdev[0] == NULL) {
		ret = -ENODEV;
		goto unlock;
	}

	vi = chan->vi;
	csi = vi->csi;

	/* The first open then turn on power */
	if (atomic_add_return(1, &vi->power_on_refcnt) == 1) {
		tegra_vi_power_on(vi);
		tegra_csi_power_on(csi);
	}

	if (!vi->pg_mode &&
		(atomic_add_return(1, &chan->power_on_refcnt) == 1)) {
		/* power on sensors connected in channel */
		tegra_csi_channel_power_on(csi, chan->port);
		ret = tegra_channel_set_power(chan, 1);
		if (ret < 0)
			goto unlock;
	}

	chan->fh = (struct v4l2_fh *)fp->private_data;

unlock:
	mutex_unlock(&chan->video_lock);
	return ret;
}

static int tegra_channel_close(struct file *fp)
{
	int ret = 0;
	struct video_device *vdev = video_devdata(fp);
	struct tegra_channel *chan = video_get_drvdata(vdev);
	struct tegra_mc_vi *vi = chan->vi;
	struct tegra_csi_device *csi = vi->csi;
	bool is_singular;

	mutex_lock(&chan->video_lock);
	is_singular = v4l2_fh_is_singular_file(fp);
	ret = _vb2_fop_release(fp, NULL);

	if (!is_singular) {
		mutex_unlock(&chan->video_lock);
		return ret;
	}

	if (!vi->pg_mode &&
		atomic_dec_and_test(&chan->power_on_refcnt)) {
		/* power off sensors connected in channel */
		tegra_csi_channel_power_off(csi, chan->port);
		ret = tegra_channel_set_power(chan, 0);
		if (ret < 0)
			dev_err(vi->dev, "Failed to power off subdevices\n");
	}

	/* The last release then turn off power */
	if (atomic_dec_and_test(&vi->power_on_refcnt)) {
		tegra_csi_power_off(csi);
		tegra_vi_power_off(vi);
	}

	mutex_unlock(&chan->video_lock);
	return ret;
}

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */
static const struct v4l2_file_operations tegra_channel_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	.open		= tegra_channel_open,
	.release	= tegra_channel_close,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};

static void vi_channel_syncpt_init(struct tegra_channel *chan)
{
	int i;

	for (i = 0; i < chan->total_ports; i++)
		chan->syncpt[i] =
			nvhost_get_syncpt_client_managed(chan->vi->ndev, "vi");
}

static void vi_channel_syncpt_free(struct tegra_channel *chan)
{
	int i;

	for (i = 0; i < chan->total_ports; i++)
		nvhost_syncpt_put_ref_ext(chan->vi->ndev, chan->syncpt[i]);
}

static void tegra_channel_csi_init(struct tegra_mc_vi *vi, unsigned int index)
{
	int numlanes = 0;
	int idx = 0;
	struct tegra_channel *chan  = &vi->chans[index];

	chan->gang_mode = CAMERA_NO_GANG_MODE;
	chan->total_ports = 0;
	memset(&chan->port[0], INVALID_CSI_PORT, TEGRA_CSI_BLOCKS);
	if (vi->pg_mode) {
		chan->port[0] = index;
		chan->numlanes = 2;
	} else
		tegra_vi_get_port_info(chan, vi->dev->of_node, index);

	for (idx = 0; csi_port_is_valid(chan->port[idx]); idx++) {
		chan->total_ports++;
		numlanes = chan->numlanes - (idx * 4);
		numlanes = numlanes > 4 ? 4 : numlanes;
		/* maximum of 4 lanes are present per CSI block */
		chan->csibase[idx] = vi->iomem +
					TEGRA_VI_CSI_BASE(chan->port[idx]);
		set_csi_portinfo(vi->csi, chan->port[idx], numlanes);
	}
	/* based on gang mode valid ports will be updated - set default to 1 */
	chan->valid_ports = chan->total_ports ? 1 : 0;
}

static int tegra_channel_init(struct tegra_mc_vi *vi, unsigned int index)
{
	int ret;
	struct tegra_channel *chan = &vi->chans[index];

	chan->vi = vi;
	tegra_channel_csi_init(vi, index);

	chan->align = 64;
	chan->num_subdevs = 0;
	mutex_init(&chan->video_lock);
	INIT_LIST_HEAD(&chan->capture);
	INIT_LIST_HEAD(&chan->done);
	init_waitqueue_head(&chan->start_wait);
	init_waitqueue_head(&chan->done_wait);
	spin_lock_init(&chan->start_lock);
	spin_lock_init(&chan->done_lock);
	mutex_init(&chan->stop_kthread_lock);
	init_completion(&chan->capture_comp);
	init_completion(&chan->done_comp);
	atomic_set(&chan->is_hdmiin_unplug, 0);

	/* Init video format */
	chan->fmtinfo = tegra_core_get_format_by_code(TEGRA_VF_DEF);
	chan->format.pixelformat = chan->fmtinfo->fourcc;
	chan->format.colorspace = V4L2_COLORSPACE_SRGB;
	chan->format.field = V4L2_FIELD_NONE;
	chan->format.width = TEGRA_DEF_WIDTH;
	chan->format.height = TEGRA_DEF_HEIGHT;
	chan->format.bytesperline = chan->format.width * chan->fmtinfo->bpp;
	chan->format.sizeimage = chan->format.bytesperline *
				    chan->format.height;
	chan->buffer_offset[0] = 0;

	/* Initialize the media entity... */
	chan->pad.flags = MEDIA_PAD_FL_SINK;

	ret = media_entity_init(&chan->video.entity, 1, &chan->pad, 0);
	if (ret < 0)
		return ret;

	/* init control handler */
	ret = v4l2_ctrl_handler_init(&chan->ctrl_handler, MAX_CID_CONTROLS);
	if (chan->ctrl_handler.error) {
		dev_err(&chan->video.dev, "failed to init control handler\n");
		goto video_register_error;
	}

	/* init video node... */
	chan->video.fops = &tegra_channel_fops;
	chan->video.v4l2_dev = &vi->v4l2_dev;
	chan->video.queue = &chan->queue;
	snprintf(chan->video.name, sizeof(chan->video.name), "%s-%s-%u",
		dev_name(vi->dev), vi->pg_mode ? "tpg" : "output",
		chan->port[0]);
	chan->video.vfl_type = VFL_TYPE_GRABBER;
	chan->video.vfl_dir = VFL_DIR_RX;
	chan->video.release = video_device_release_empty;
	chan->video.ioctl_ops = &tegra_channel_ioctl_ops;
	chan->video.ctrl_handler = &chan->ctrl_handler;
	chan->video.lock = &chan->video_lock;

	set_bit(V4L2_FL_USE_FH_PRIO, &chan->video.flags);

	video_set_drvdata(&chan->video, chan);

#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	vi_channel_syncpt_init(chan);

	/* get the buffers queue... */
	chan->alloc_ctx = vb2_dma_contig_init_ctx(chan->vi->dev);
	if (IS_ERR(chan->alloc_ctx)) {
		dev_err(chan->vi->dev, "failed to init vb2 buffer\n");
		ret = -ENOMEM;
		goto vb2_init_error;
	}
#endif

	chan->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	chan->queue.io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ | VB2_USERPTR;
	chan->queue.lock = &chan->video_lock;
	chan->queue.drv_priv = chan;
	chan->queue.buf_struct_size = sizeof(struct tegra_channel_buffer);
	chan->queue.ops = &tegra_channel_queue_qops;
#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	chan->queue.mem_ops = &vb2_dma_contig_memops;
#endif
	chan->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
				   | V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	ret = vb2_queue_init(&chan->queue);
	if (ret < 0) {
		dev_err(chan->vi->dev, "failed to initialize VB2 queue\n");
		goto vb2_queue_error;
	}

	ret = video_register_device(&chan->video, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		dev_err(&chan->video.dev, "failed to register video device\n");
		goto video_register_error;
	}

	return 0;

video_register_error:
	vb2_queue_release(&chan->queue);
vb2_queue_error:
#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	vb2_dma_contig_cleanup_ctx(chan->alloc_ctx);
vb2_init_error:
#endif
	media_entity_cleanup(&chan->video.entity);
	return ret;
}

static int tegra_channel_cleanup(struct tegra_channel *chan)
{
	video_unregister_device(&chan->video);

	v4l2_ctrl_handler_free(&chan->ctrl_handler);
	vb2_queue_release(&chan->queue);
#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	vb2_dma_contig_cleanup_ctx(chan->alloc_ctx);
#endif

	vi_channel_syncpt_free(chan);
	media_entity_cleanup(&chan->video.entity);

	return 0;
}

int tegra_vi_channels_init(struct tegra_mc_vi *vi)
{
	unsigned int i;
	int ret;

	for (i = 0; i < vi->num_channels; i++) {
		ret = tegra_channel_init(vi, i);
		if (ret < 0) {
			dev_err(vi->dev, "channel %d init failed\n", i);
			return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL(tegra_vi_channels_init);

int tegra_vi_channels_cleanup(struct tegra_mc_vi *vi)
{
	unsigned int i;
	int ret;

	for (i = 0; i < vi->num_channels; i++) {
		ret = tegra_channel_cleanup(&vi->chans[i]);
		if (ret < 0) {
			dev_err(vi->dev, "channel %d cleanup failed\n", i);
			return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL(tegra_vi_channels_cleanup);
