/*
 * Tegra Video Input 5 device common APIs
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frank@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/nvhost.h>
#include <linux/tegra-powergate.h>
#include <linux/semaphore.h>
#include <media/tegra_camera_platform.h>
#include <media/mc_common.h>
#include <media/capture_vi_channel.h>
#include <media/capture.h>
#include <soc/tegra/camrtc-capture.h>
#include "linux/nvhost_ioctl.h"
#include "nvhost_acm.h"
#include "vi5_formats.h"
#include "vi5_fops.h"
#include <trace/events/camera_common.h>

#define DEFAULT_FRAMERATE	30
#define BPP_MEM			2
#define VI_CSI_CLK_SCALE	110
#define PG_BITRATE		32
#define	STREAM		0U

static void tegra_channel_stop_kthreads(struct tegra_channel *chan);

static const struct vi_capture_setup default_setup = {
	.channel_flags = 0
	| CAPTURE_CHANNEL_FLAG_VIDEO
	| CAPTURE_CHANNEL_FLAG_RAW
	| CAPTURE_CHANNEL_FLAG_EMBDATA
	| CAPTURE_CHANNEL_FLAG_LINETIMER
	,

	.vi_channel_mask = ~0ULL,

	.queue_depth = CAPTURE_QUEUE_DEPTH,
	.request_size = sizeof(struct capture_descriptor),
	.mem = 0,  /* fill in later */
};

static const struct capture_descriptor capture_template = {
	.sequence = 0,

	.capture_flags = 0
	| CAPTURE_FLAG_STATUS_REPORT_ENABLE
	| CAPTURE_FLAG_ERROR_REPORT_ENABLE
	,

	.ch_cfg = {
		.pixfmt_enable = 0,		/* no output */
		.match = {
			.stream = 0,		/* one-hot bit encoding */
			.stream_mask = 0x3f,
			.vc = (1u << 0),	/* one-hot bit encoding */
			.vc_mask = 0xf,
		},
	},
};

static void vi5_init_video_formats(struct tegra_channel *chan)
{
	int i;

	chan->num_video_formats = ARRAY_SIZE(vi5_video_formats);
	for (i = 0; i < chan->num_video_formats; i++)
		chan->video_formats[i] = &vi5_video_formats[i];
}

static int tegra_vi5_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tegra_channel *chan = container_of(ctrl->handler,
				struct tegra_channel, ctrl_handler);
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_WRITE_ISPFORMAT:
		chan->write_ispformat = ctrl->val;
		break;
	default:
		dev_err(&chan->video.dev, "%s:Not valid ctrl\n", __func__);
		return -EINVAL;
	}

	return err;
}

static const struct v4l2_ctrl_ops vi5_ctrl_ops = {
	.s_ctrl	= tegra_vi5_s_ctrl,
};

static const struct v4l2_ctrl_config vi5_custom_ctrls[] = {
	{
		.ops = &vi5_ctrl_ops,
		.id = TEGRA_CAMERA_CID_WRITE_ISPFORMAT,
		.name = "Write ISP format",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = 1,
		.min = 1,
		.max = 1,
		.step = 1,
	},
};

static int vi5_add_ctrls(struct tegra_channel *chan)
{
	int i;

	/* Add vi5 custom controls */
	for (i = 0; i < ARRAY_SIZE(vi5_custom_ctrls); i++) {
		v4l2_ctrl_new_custom(&chan->ctrl_handler,
			&vi5_custom_ctrls[i], NULL);
		if (chan->ctrl_handler.error) {
			dev_err(chan->vi->dev,
				"Failed to add %s ctrl\n",
				vi5_custom_ctrls[i].name);
			return chan->ctrl_handler.error;
		}
	}

	return 0;
}

static void tegra_channel_surface_setup(
	struct tegra_channel *chan, struct tegra_channel_buffer *buf, int index,
	unsigned int descr_index)
{
	unsigned int offset = chan->buffer_offset[index];
	u32 height = chan->format.height;
	u32 width = chan->format.width;
	u32 format = chan->fmtinfo->img_fmt;
	u32 data_type = chan->fmtinfo->img_dt;
	u32 csi_port = chan->port[index];
	struct capture_descriptor *desc = &chan->request[descr_index];

	memcpy(desc, &capture_template, sizeof(capture_template));

	desc->sequence = chan->capture_descr_sequence;
	desc->ch_cfg.match.stream = (1u << csi_port); /* one-hot bit encoding */
	desc->ch_cfg.frame.frame_x = width;
	desc->ch_cfg.frame.frame_y = height;
	desc->ch_cfg.match.datatype = data_type;
	desc->ch_cfg.match.datatype_mask = 0x3f;
	desc->ch_cfg.pixfmt_enable = 1;
	desc->ch_cfg.pixfmt.format = format;

	desc->ch_cfg.atomp.surface[0].offset = buf->addr + offset;
	desc->ch_cfg.atomp.surface[0].offset_hi = 0;
	desc->ch_cfg.atomp.surface_stride[0] = chan->format.bytesperline;

	if (chan->embedded_data_height > 0) {
		desc->ch_cfg.embdata_enable = 1;
		desc->ch_cfg.frame.embed_x = chan->embedded_data_width * BPP_MEM;
		desc->ch_cfg.frame.embed_y = chan->embedded_data_height;
		desc->ch_cfg.atomp.surface[VI_ATOMP_SURFACE_EMBEDDED].offset
			= chan->vi->emb_buf;
		desc->ch_cfg.atomp.surface[VI_ATOMP_SURFACE_EMBEDDED].offset_hi
			= 0;
		desc->ch_cfg.atomp.surface_stride[VI_ATOMP_SURFACE_EMBEDDED]
			= chan->embedded_data_width * BPP_MEM;
	}

	chan->capture_descr_sequence += 1;
}

static int tegra_channel_capture_setup(struct tegra_channel *chan,
		unsigned int index)
{
	struct vi_capture_setup setup = default_setup;
	long err;

	trace_tegra_channel_capture_setup(chan, index);
	chan->request = dma_alloc_coherent(chan->tegra_vi_channel->rtcpu_dev,
					setup.queue_depth * setup.request_size,
					&setup.iova, GFP_KERNEL);
	if (chan->request == NULL)
		dev_err(chan->vi->dev, "dma_alloc_coherent failed\n");

	if (chan->is_slvsec) {
		setup.channel_flags |= CAPTURE_CHANNEL_FLAG_SLVSEC;
		setup.slvsec_stream_main = STREAM;
		setup.slvsec_stream_sub = SLVSEC_STREAM_DISABLED;
	}

	err = vi_capture_setup(chan->tegra_vi_channel, &setup);
	if (err) {
		dev_err(chan->vi->dev, "vi capture setup failed\n");
		return err;
	}

	return 0;
}

static int tegra_channel_capture_enqueue(struct tegra_channel *chan,
	struct tegra_channel_buffer *buf)
{
	int err = 0;
	unsigned long flags;
	int i;
	struct vi_capture_req request = {
		.buffer_index = 0,
	};

	/* Reset the channel if it is currently in a state of error */
	spin_lock_irqsave(&chan->capture_state_lock, flags);
	if (chan->capture_state == CAPTURE_ERROR) {
		dev_err(chan->vi->dev, "channel error, resetting the channel\n");
		tegra_channel_init_ring_buffer(chan);
		chan->capture_state = CAPTURE_IDLE;
	}
	spin_unlock_irqrestore(&chan->capture_state_lock, flags);

	if (!chan->bfirst_fstart) {
		err = tegra_channel_set_stream(chan, true);
		if (err < 0)
			return err;
		chan->bfirst_fstart = true;
	}

	err = down_interruptible(&chan->capture_slots);
	if (err)
		return err;

	/* Set up buffer and enqueue capture request for a frame */
	for (i = 0; i < chan->valid_ports; i++)
		tegra_channel_surface_setup(chan, buf, i,
			chan->capture_descr_index);

	request.buffer_index = chan->capture_descr_index;

	err = vi_capture_request(chan->tegra_vi_channel, &request);
	if (err) {
		dev_err(chan->vi->dev, "vi capture request enqueue failed\n");

		buf->vb2_state = VB2_BUF_STATE_ERROR;

		spin_lock_irqsave(&chan->capture_state_lock, flags);
		chan->capture_state = CAPTURE_ERROR;
		spin_unlock_irqrestore(&chan->capture_state_lock, flags);

		goto done;
	}

	spin_lock_irqsave(&chan->capture_state_lock, flags);
	if (chan->capture_state != CAPTURE_ERROR)
		chan->capture_state = CAPTURE_GOOD;
	spin_unlock_irqrestore(&chan->capture_state_lock, flags);

	buf->capture_descr_index = chan->capture_descr_index;

	chan->capture_descr_index = ((chan->capture_descr_index + 1)
		% CAPTURE_QUEUE_DEPTH);

done:
	/* Move buffer into dequeue queue */
	spin_lock(&chan->dequeue_lock);
	list_add_tail(&buf->queue, &chan->dequeue);
	spin_unlock(&chan->dequeue_lock);

	/* Wake up kthread for capture dequeue */
	wake_up_interruptible(&chan->dequeue_wait);

	return 0;
}

static void tegra_channel_capture_dequeue(struct tegra_channel *chan,
	struct tegra_channel_buffer *buf)
{
	int err = 0;
	unsigned long flags;
	struct vb2_v4l2_buffer *vb = &buf->buf;
	struct timespec ts;
	struct capture_descriptor *descr =
		&chan->request[buf->capture_descr_index];

	if (buf->vb2_state != VB2_BUF_STATE_ACTIVE)
		goto done;

	/* Dequeue a frame and check its capture status */
	err = vi_capture_status(chan->tegra_vi_channel, 2500);

	/* Mark frame as in error and discard */
	if (err || (descr->status.status != CAPTURE_STATUS_SUCCESS)) {
		dev_err(chan->vi->dev, "vi capture dequeue status failed\n");

		buf->vb2_state = VB2_BUF_STATE_ERROR;

		spin_lock_irqsave(&chan->capture_state_lock, flags);
		chan->capture_state = CAPTURE_ERROR;
		spin_unlock_irqrestore(&chan->capture_state_lock, flags);

		goto done;
	}

	buf->vb2_state = VB2_BUF_STATE_DONE;

	spin_lock_irqsave(&chan->capture_state_lock, flags);
	if (chan->capture_state != CAPTURE_ERROR)
		chan->capture_state = CAPTURE_GOOD;
	spin_unlock_irqrestore(&chan->capture_state_lock, flags);

	/* Read SOF from capture descriptor */
	ts = ns_to_timespec((s64)descr->status.sof_timestamp);
	trace_tegra_channel_capture_frame("sof", ts);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	/* update time stamp of the buffer */
	vb->timestamp.tv_sec = ts.tv_sec;
	vb->timestamp.tv_usec = ts.tv_nsec / NSEC_PER_USEC;
#else
	/* TODO: granular time code information */
	vb->timecode.seconds = ts.tv_sec;
#endif

done:
	up(&chan->capture_slots);
	chan->buffer_state[chan->free_index] = buf->vb2_state;
	free_ring_buffers(chan, 1);
}

static int tegra_channel_kthread_capture_enqueue(void *data)
{
	struct tegra_channel *chan = data;
	struct tegra_channel_buffer *buf;
	int err = 0;

	set_freezable();

	while (1) {
		try_to_freeze();

		wait_event_interruptible(chan->start_wait,
			(!list_empty(&chan->capture) || kthread_should_stop()));

		if (kthread_should_stop())
			break;

		/* source is not streaming if error is non-zero */
		/* wait till kthread stop and dont DeQ buffers */
		if (err)
			continue;
		buf = dequeue_buffer(chan);
		if (!buf)
			continue;
		buf->vb2_state = VB2_BUF_STATE_ACTIVE;

		err = tegra_channel_capture_enqueue(chan, buf);
	}

	return 0;
}

static int tegra_channel_kthread_capture_dequeue(void *data)
{
	struct tegra_channel *chan = data;
	struct tegra_channel_buffer *buf;

	set_freezable();

	while (1) {
		try_to_freeze();

		wait_event_interruptible(chan->dequeue_wait,
			(!list_empty(&chan->dequeue) || kthread_should_stop()));

		if (kthread_should_stop() && list_empty(&chan->dequeue))
			break;

		do {
			buf = dequeue_dequeue_buffer(chan);
			if (!buf)
				break;

			tegra_channel_capture_dequeue(chan, buf);
		} while (!list_empty(&chan->dequeue));
	}

	return 0;
}

static void tegra_channel_stop_kthreads(struct tegra_channel *chan)
{
	mutex_lock(&chan->stop_kthread_lock);
	/* Stop the kthread for capture enqueue */
	if (chan->kthread_capture_start) {
		kthread_stop(chan->kthread_capture_start);
		chan->kthread_capture_start = NULL;
	}
	/* Stop the kthread for capture dequeue */
	if (chan->kthread_capture_dequeue) {
		kthread_stop(chan->kthread_capture_dequeue);
		chan->kthread_capture_dequeue = NULL;
	}
	mutex_unlock(&chan->stop_kthread_lock);
}

static int vi5_channel_start_streaming(struct vb2_queue *vq, u32 count)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);
	/* WAR: With newer version pipe init has some race condition */
	/* TODO: resolve this issue to block userspace not to cleanup media */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	struct media_pipeline *pipe = chan->video.entity.pipe;
#endif
	int ret = 0, i;
	unsigned long flags;
	struct v4l2_subdev *sd;
	struct device_node *node;
	struct sensor_mode_properties *sensor_mode;
	struct camera_common_data *s_data;
	unsigned int emb_buf_size = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	ret = media_entity_pipeline_start(&chan->video.entity, pipe);
	if (ret < 0)
		goto error_pipeline_start;
#endif

	if (chan->bypass) {
		ret = tegra_channel_set_stream(chan, true);
		if (ret < 0)
			goto error_set_stream;
		return ret;
	}

	chan->tegra_vi_channel = vi_channel_open_ex(chan->id, false);
	if (IS_ERR(chan->tegra_vi_channel))
		return PTR_ERR(chan);

	spin_lock_irqsave(&chan->capture_state_lock, flags);
	chan->capture_state = CAPTURE_IDLE;
	spin_unlock_irqrestore(&chan->capture_state_lock, flags);

	if (!chan->pg_mode) {
		sd = chan->subdev_on_csi;
		node = sd->dev->of_node;
		s_data = to_camera_common_data(sd->dev);

		if (s_data == NULL) {
			dev_err(&chan->video.dev,
				"Camera common data missing!\n");
			return -EINVAL;
		}

		/* get sensor properties from DT */
		if (node != NULL) {
			int idx = s_data->mode_prop_idx;

			emb_buf_size = 0;
			if (idx < s_data->sensor_props.num_modes) {
				sensor_mode =
					&s_data->sensor_props.sensor_modes[idx];

				chan->embedded_data_width =
					sensor_mode->image_properties.width;
				chan->embedded_data_height =
					sensor_mode->image_properties.\
					embedded_metadata_height;
				/* rounding up to page size */
				emb_buf_size =
					round_up(chan->embedded_data_width *
						chan->embedded_data_height *
						BPP_MEM,
						PAGE_SIZE);
			}
		}

		/* Allocate buffer for Embedded Data if need to*/
		if (emb_buf_size > chan->vi->emb_buf_size) {
			/*
			 * if old buffer is smaller than what we need,
			 * release the old buffer and re-allocate a bigger
			 * one below
			 */
			if (chan->vi->emb_buf_size > 0) {
				dma_free_coherent(chan->vi->dev,
					chan->vi->emb_buf_size,
					chan->vi->emb_buf_addr, chan->vi->emb_buf);
				chan->vi->emb_buf_size = 0;
			}

			chan->vi->emb_buf_addr =
				dma_alloc_coherent(chan->vi->dev,
					emb_buf_size,
					&chan->vi->emb_buf, GFP_KERNEL);
			if (!chan->vi->emb_buf_addr) {
				dev_err(&chan->video.dev,
						"Can't allocate memory for embedded data\n");
				goto error_capture_setup;
			}
			chan->vi->emb_buf_size = emb_buf_size;
		}
	}

	for (i = 0; i < chan->valid_ports; i++) {
		ret = tegra_channel_capture_setup(chan, i);
		if (ret < 0)
			goto error_capture_setup;
	}

	chan->sequence = 0;
	tegra_channel_init_ring_buffer(chan);

	/* Start kthread to enqueue captures to RCE */
	chan->kthread_capture_start = kthread_run(
		tegra_channel_kthread_capture_enqueue, chan, chan->video.name);
	if (IS_ERR(chan->kthread_capture_start)) {
		dev_err(&chan->video.dev,
			"failed to run kthread for capture enqueue\n");
		ret = PTR_ERR(chan->kthread_capture_start);
		goto error_capture_setup;
	}

	/* Start kthread to dequeue captures to buffer */
	chan->kthread_capture_dequeue = kthread_run(
		tegra_channel_kthread_capture_dequeue, chan, chan->video.name);
	if (IS_ERR(chan->kthread_capture_dequeue)) {
		dev_err(&chan->video.dev,
			"failed to run kthread for capture dequeue\n");
		ret = PTR_ERR(chan->kthread_capture_dequeue);
		goto error_capture_setup;
	}

	return 0;

error_capture_setup:
	if (!chan->pg_mode)
		tegra_channel_set_stream(chan, false);
error_set_stream:
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	media_entity_pipeline_stop(&chan->video.entity);
error_pipeline_start:
#endif
	vq->start_streaming_called = 0;
	tegra_channel_queued_buf_done(chan, VB2_BUF_STATE_QUEUED);

	return ret;
}

static int vi5_channel_stop_streaming(struct vb2_queue *vq)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);
	int i;
	long err;

	for (i = 0; i < chan->valid_ports; i++) {
		if (chan->vnc_id[i] == -1)
			return 0;
	}

	if (!chan->bypass) {
		tegra_channel_stop_kthreads(chan);
		/* free all the ring buffers */
		free_ring_buffers(chan, chan->num_buffers);
		/* dequeue buffers back to app which are in capture queue */
		tegra_channel_queued_buf_done(chan, VB2_BUF_STATE_ERROR);
	}

	/* CSI channel to be closed before closing VI channel */
	tegra_channel_set_stream(chan, false);

	if (!chan->bypass) {
		err = vi_capture_release(chan->tegra_vi_channel, 0);
		if (err)
			dev_err(&chan->video.dev, "vi capture release failed\n");

		vi_channel_close_ex(chan->id, chan->tegra_vi_channel);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	media_entity_pipeline_stop(&chan->video.entity);
#endif

	return 0;
}

int tegra_vi5_power_on(struct tegra_mc_vi *vi)
{
	int ret;

	ret = nvhost_module_busy(vi->ndev);
	if (ret) {
		dev_err(vi->dev, "%s:nvhost module is busy\n", __func__);
		return ret;
	}

	ret = tegra_camera_emc_clk_enable();
	if (ret)
		goto err_emc_enable;

	return 0;

err_emc_enable:
	nvhost_module_idle(vi->ndev);

	return ret;
}

void tegra_vi5_power_off(struct tegra_mc_vi *vi)
{
	tegra_channel_ec_close(vi);
	tegra_camera_emc_clk_disable();
	nvhost_module_idle(vi->ndev);
}

static int vi5_power_on(struct tegra_channel *chan)
{
	int ret = 0;
	struct tegra_mc_vi *vi;
	struct tegra_csi_device *csi;

	vi = chan->vi;
	csi = vi->csi;

	/* Use chan->video as identifier of vi5 nvhost_module client
	 * since they are unique per channel
	 */
	ret = nvhost_module_add_client(vi->ndev, &chan->video);
	if (ret < 0)
		return ret;

	ret = tegra_vi5_power_on(vi);
	if (ret < 0)
		return ret;

	if (atomic_add_return(1, &chan->power_on_refcnt) == 1) {
		ret = tegra_channel_set_power(chan, 1);
		if (ret < 0) {
			dev_err(vi->dev, "Failed to power on subdevices\n");
			return ret;
		}
	}

	return 0;
}

static void vi5_power_off(struct tegra_channel *chan)
{
	int ret = 0;
	struct tegra_mc_vi *vi;
	struct tegra_csi_device *csi;

	vi = chan->vi;
	csi = vi->csi;

	if (atomic_dec_and_test(&chan->power_on_refcnt)) {
		ret = tegra_channel_set_power(chan, 0);
		if (ret < 0)
			dev_err(vi->dev, "Failed to power off subdevices\n");
	}

	tegra_vi5_power_off(vi);
	nvhost_module_remove_client(vi->ndev, &chan->video);
}

struct tegra_vi_fops vi5_fops = {
	.vi_power_on = vi5_power_on,
	.vi_power_off = vi5_power_off,
	.vi_start_streaming = vi5_channel_start_streaming,
	.vi_stop_streaming = vi5_channel_stop_streaming,
	.vi_add_ctrls = vi5_add_ctrls,
	.vi_init_video_formats = vi5_init_video_formats,
};
