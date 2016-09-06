/*
 * Tegra Video Input 2 device common APIs
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/device.h>
#include <linux/nvhost.h>
#include <linux/tegra-powergate.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <media/tegra_camera_platform.h>
#include "nvhost_acm.h"
#include "vi/vi.h"
#include "camera/vi/mc_common.h"
#include "camera/csi/csi2_fops.h"

#define FRAMERATE	30
#define BPP_MEM		2

extern void tegra_channel_queued_buf_done(struct tegra_channel *chan,
					  enum vb2_buffer_state state);
static void tegra_channel_stop_kthreads(struct tegra_channel *chan);
extern int tegra_channel_set_stream(struct tegra_channel *chan, bool on);
extern void tegra_channel_ring_buffer(struct tegra_channel *chan,
									struct vb2_v4l2_buffer *vb,
									struct timespec *ts, int state);
extern struct tegra_channel_buffer *dequeue_buffer(struct tegra_channel *chan);
extern int update_clk(struct tegra_mc_vi *vi);
extern void tegra_channel_init_ring_buffer(struct tegra_channel *chan);
extern void free_ring_buffers(struct tegra_channel *chan, int frames);
extern int tegra_channel_set_power(struct tegra_channel *chan, bool on);

static void vi_write(struct tegra_mc_vi *vi, unsigned int addr, u32 val)
{
	writel(val, vi->iomem + addr);
}

static u32 tegra_channel_read(struct tegra_channel *chan,
			unsigned int addr)
{
	return readl(chan->vi->iomem + addr);
}

static void tegra_channel_write(struct tegra_channel *chan,
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
		  (format << IMAGE_DEF_FORMAT_OFFSET));
		csi_write(chan, index, TEGRA_VI_CSI_IMAGE_DT, data_type);
		csi_write(chan, index, TEGRA_VI_CSI_IMAGE_SIZE_WC, word_count);
		csi_write(chan, index, TEGRA_VI_CSI_IMAGE_SIZE,
			(height << IMAGE_SIZE_HEIGHT_OFFSET) | width);
	}

	return 0;
}

static int tegra_channel_enable_stream(struct tegra_channel *chan)
{
	int ret = 0, i;

	/*
	 * enable pad power and perform calibration before arming
	 * single shot for first frame after the HW setup is complete
	 */
	/* enable pad power */
	tegra_csi_pad_control(chan->vi->csi, chan->port, ENABLE);
	/* start streaming */
	if (chan->vi->pg_mode) {
		for (i = 0; i < chan->valid_ports; i++)
			tegra_csi_tpg_start_streaming(chan->vi->csi,
							chan->port[i]);
		atomic_set(&chan->is_streaming, ENABLE);
	} else {
		ret = tegra_channel_set_stream(chan, true);
		if (ret < 0)
			return ret;
	}
	/* perform calibration as sensor started streaming */
	/*
	tegra_mipi_bias_pad_enable();
	if (!chan->vi->pg_mode) {
		mutex_lock(&chan->vi->mipical_lock);
		tegra_channel_mipi_cal(chan, 0);
		mutex_unlock(&chan->vi->mipical_lock);
	}*/

	return ret;
}

static void tegra_channel_ec_init(struct tegra_channel *chan)
{
	/*
	 * error recover initialization sequence
	 * set timeout as 200 ms, use default if fps not available
	 * Time limit allow CSI to capture good frames and drop error frames
	 * Timeout units is jiffies, 1 jiffy = 10ms
	 * TODO: Get frame rate from sub-device and adopt timeout
	 */
	chan->timeout = 20;

	/*
	 * Sync point FIFO full blocks host interface
	 * Below setting enables SW to process error recovery
	 */
	tegra_channel_write(chan, TEGRA_VI_CFG_VI_INCR_SYNCPT_CNTRL, 0x100);
}


static void tegra_channel_clear_singleshot(struct tegra_channel *chan,
						int index)
{
	/* clear single shot */
	csi_write(chan, index, TEGRA_VI_CSI_SW_RESET, 0xF);
	csi_write(chan, index, TEGRA_VI_CSI_SW_RESET, 0x0);
}

static void tegra_channel_vi_csi_recover(struct tegra_channel *chan)
{
	u32 error_val = tegra_channel_read(chan,
					TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR);
	u32 frame_start;
	int index, valid_ports = chan->valid_ports;

	/* Disable pad power to start recovery */
	tegra_csi_pad_control(chan->vi->csi, chan->port, DISABLE);
	/* Disable clock gating to enable continuous clock */
	tegra_channel_write(chan, TEGRA_VI_CFG_CG_CTRL, DISABLE);
	/* clear CSI state */
	for (index = 0; index < valid_ports; index++) {
		tegra_csi_error_recover(&chan->vi->csi->chans[index],
				chan->port[index]);
		csi_write(chan, index,
				TEGRA_VI_CSI_IMAGE_DEF, 0);
		tegra_channel_clear_singleshot(chan, index);
	}

	/* clear VI errors */
	for (index = 0; index < valid_ports; index++) {
		frame_start = VI_CSI_PP_FRAME_START(chan->port[index]);
		if (error_val & frame_start)
			chan->syncpoint_fifo[index] = SYNCPT_FIFO_DEPTH;
	}
	/* clear FIFO error status */
	tegra_channel_write(chan,
		TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR, error_val);

	/* Enable clock gating so VI can be clock gated if necessary */
	tegra_channel_write(chan, TEGRA_VI_CFG_CG_CTRL, ENABLE);

	/* re-init VI and CSI */
	tegra_channel_capture_setup(chan);
	for (index = 0; index < valid_ports; index++) {
		csi2_stop_streaming(chan->vi->csi,
						chan->port[index]);
		csi2_start_streaming(chan->vi->csi,
						chan->port[index]);
		nvhost_syncpt_set_min_eq_max_ext(chan->vi->ndev,
						chan->syncpt[index]);
	}
}

static void tegra_channel_capture_error(struct tegra_channel *chan)
{
	u32 val;
	int index = 0;

	for (index = 0; index < chan->valid_ports; index++) {
		val = csi_read(chan, index, TEGRA_VI_CSI_ERROR_STATUS);
		dev_dbg(&chan->video.dev,
			"TEGRA_VI_CSI_ERROR_STATUS 0x%08x\n", val);
		tegra_csi_status(&chan->vi->csi->chans[index],
				chan->port[index]);
	}
}

static void tegra_channel_ec_recover(struct tegra_channel *chan)
{
	tegra_channel_capture_error(chan);
	tegra_channel_vi_csi_recover(chan);
}

static int tegra_channel_error_status(struct tegra_channel *chan)
{
	u32 val;
	int err = 0;
	int index = 0;

	for (index = 0; index < chan->valid_ports; index++) {
		/* Ignore error based on resolution but reset status */
		val = csi_read(chan, index, TEGRA_VI_CSI_ERROR_STATUS);
		csi_write(chan, index, TEGRA_VI_CSI_ERROR_STATUS, val);
		err = tegra_csi_error(&chan->vi->csi->chans[index],
				chan->port[index]);
	}

	if (err)
		dev_err(chan->vi->dev, "%s:error %x frame %d\n",
				__func__, err, chan->sequence);
	return err;
}

static int tegra_channel_capture_frame(struct tegra_channel *chan,
					struct tegra_channel_buffer *buf)
{
	struct vb2_v4l2_buffer *vb = &buf->buf;
	struct timespec ts;
	int err = 0;
	u32 val, frame_start;
	int bytes_per_line = chan->format.bytesperline;
	int index = 0;
	u32 thresh[TEGRA_CSI_BLOCKS] = { 0 };
	int valid_ports = chan->valid_ports;
	int state = VB2_BUF_STATE_DONE;

	/* Init registers related to each frames */
	for (index = 0; index < valid_ports; index++) {
		/* Program buffer address by using surface 0 */
		csi_write(chan, index, TEGRA_VI_CSI_SURFACE0_OFFSET_MSB, 0x0);
		csi_write(chan, index, TEGRA_VI_CSI_SURFACE0_OFFSET_LSB,
			(buf->addr + chan->buffer_offset[index]));
		csi_write(chan, index,
			TEGRA_VI_CSI_SURFACE0_STRIDE, bytes_per_line);

		/* Program syncpoints */
		thresh[index] = nvhost_syncpt_incr_max_ext(chan->vi->ndev,
					chan->syncpt[index], 1);
		/* Do not arm sync points if FIFO had entries before */
		if (!chan->syncpoint_fifo[index]) {
			frame_start = VI_CSI_PP_FRAME_START(chan->port[index]);
			val = VI_CFG_VI_INCR_SYNCPT_COND(frame_start) |
				chan->syncpt[index];
			tegra_channel_write(chan,
				TEGRA_VI_CFG_VI_INCR_SYNCPT, val);
		} else
			chan->syncpoint_fifo[index]--;
	}

	/* enable input stream once the VI registers are configured */
	if (!chan->bfirst_fstart) {
		err = tegra_channel_enable_stream(chan);
		if (err) {
			state = VB2_BUF_STATE_ERROR;
			chan->capture_state = CAPTURE_ERROR;
			tegra_channel_ring_buffer(chan, vb, &ts, state);
			return err;
		}
		/* Bit controls VI memory write, enable after all regs */
		for (index = 0; index < valid_ports; index++) {
			val = csi_read(chan, index,
					TEGRA_VI_CSI_IMAGE_DEF);
			csi_write(chan, index, TEGRA_VI_CSI_IMAGE_DEF,
					val | IMAGE_DEF_DEST_MEM);
		}
	}

	/* Ensure all CSI ports are ready with setup to avoid timing issue */
	for (index = 0; index < valid_ports; index++)
		csi_write(chan, index,
			TEGRA_VI_CSI_SINGLE_SHOT, SINGLE_SHOT_CAPTURE);

	chan->capture_state = CAPTURE_GOOD;
	for (index = 0; index < valid_ports; index++) {
		err = nvhost_syncpt_wait_timeout_ext(chan->vi->ndev,
			chan->syncpt[index], thresh[index],
			chan->timeout, NULL, &ts);
		if (err) {
			dev_err(&chan->video.dev,
				"frame start syncpt timeout!%d\n", index);
			state = VB2_BUF_STATE_ERROR;
			/* perform error recovery for timeout */
			tegra_channel_ec_recover(chan);
			chan->capture_state = CAPTURE_TIMEOUT;
			break;
		}
	}

	if (!err && !chan->vi->pg_mode) {
		/* Marking error frames and resume capture */
		/* TODO: TPG has frame height short error always set */
		err = tegra_channel_error_status(chan);
		if (err) {
			state = VB2_BUF_STATE_ERROR;
			chan->capture_state = CAPTURE_ERROR;
			/* do we have to run recover here ?? */
			/* tegra_channel_ec_recover(chan); */
		}
	}

	tegra_channel_ring_buffer(chan, vb, &ts, state);

	return 0;
}

static void tegra_channel_capture_done(struct tegra_channel *chan)
{
	struct timespec ts;
	int index, err;
	int bytes_per_line = chan->format.bytesperline;
	u32 val, mw_ack_done;
	u32 thresh[TEGRA_CSI_BLOCKS] = { 0 };
	struct tegra_channel_buffer *buf;
	int state = VB2_BUF_STATE_DONE;

	/* dequeue buffer and return if no buffer exists */
	buf = dequeue_buffer(chan);
	if (!buf)
		return;

	for (index = 0; index < chan->valid_ports; index++) {
		/* Program buffer address by using surface 0 */
		csi_write(chan, index, TEGRA_VI_CSI_SURFACE0_OFFSET_MSB, 0x0);
		csi_write(chan, index, TEGRA_VI_CSI_SURFACE0_OFFSET_LSB,
			(buf->addr + chan->buffer_offset[index]));
		csi_write(chan, index, TEGRA_VI_CSI_SURFACE0_STRIDE, bytes_per_line);

		/* Program syncpoints */
		thresh[index] = nvhost_syncpt_incr_max_ext(chan->vi->ndev,
					chan->syncpt[index], 1);
		mw_ack_done = VI_CSI_MW_ACK_DONE(chan->port[index]);
		val = VI_CFG_VI_INCR_SYNCPT_COND(mw_ack_done) |
				chan->syncpt[index];
		tegra_channel_write(chan, TEGRA_VI_CFG_VI_INCR_SYNCPT, val);
		csi_write(chan, index,
			TEGRA_VI_CSI_SINGLE_SHOT, SINGLE_SHOT_CAPTURE);
	}

	for (index = 0; index < chan->valid_ports; index++) {
		err = nvhost_syncpt_wait_timeout_ext(chan->vi->ndev,
			chan->syncpt[index], thresh[index],
			chan->timeout, NULL, &ts);
		if (err) {
			dev_err(&chan->video.dev,
				"MW_ACK_DONE syncpoint time out!%d\n", index);
			state = VB2_BUF_STATE_ERROR;
			/* perform error recovery for timeout */
			tegra_channel_ec_recover(chan);
			chan->capture_state = CAPTURE_TIMEOUT;
			break;
		}
	}

	/* Mark capture state to IDLE as capture is finished */
	chan->capture_state = CAPTURE_IDLE;

	tegra_channel_ring_buffer(chan, &buf->buf, &ts, state);
}

static int tegra_channel_kthread_capture_start(void *data)
{
	struct tegra_channel *chan = data;
	struct tegra_channel_buffer *buf;
	int err = 0;

	set_freezable();

	while (1) {

		try_to_freeze();

		wait_event_interruptible(chan->start_wait,
					 !list_empty(&chan->capture) ||
					 kthread_should_stop());

		if (kthread_should_stop()) {
			complete(&chan->capture_comp);
			break;
		}

		/* source is not streaming if error is non-zero */
		/* wait till kthread stop and dont DeQ buffers */
		if (err)
			continue;

		buf = dequeue_buffer(chan);
		if (!buf)
			continue;

		err = tegra_channel_capture_frame(chan, buf);
	}

	return 0;
}

static void tegra_channel_stop_kthreads(struct tegra_channel *chan)
{
	mutex_lock(&chan->stop_kthread_lock);
	/* Stop the kthread for capture */
	if (chan->kthread_capture_start) {
		kthread_stop(chan->kthread_capture_start);
		wait_for_completion(&chan->capture_comp);
		chan->kthread_capture_start = NULL;
	}
	mutex_unlock(&chan->stop_kthread_lock);
}

static void tegra_channel_update_clknbw(struct tegra_channel *chan, u8 on)
{
	/* width * height * fps * KBytes write to memory
	 * WAR: Using fix fps until we have a way to set it
	 */
	chan->requested_kbyteps = (on > 0 ? 1 : -1) * ((chan->format.width
				* chan->format.height
				* FRAMERATE * BPP_MEM) / 1000);
	chan->requested_hz = on > 0 ? chan->format.width * chan->format.height
				* FRAMERATE : 0;
	mutex_lock(&chan->vi->bw_update_lock);
	chan->vi->aggregated_kbyteps += chan->requested_kbyteps;
	vi_v4l2_update_isobw(chan->vi->aggregated_kbyteps, 0);
	vi_v4l2_set_la(tegra_vi_get(), 0, 0);
	update_clk(chan->vi);
	mutex_unlock(&chan->vi->bw_update_lock);
}

int vi2_channel_start_streaming(struct vb2_queue *vq, u32 count)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);
	struct media_pipeline *pipe = chan->video.entity.pipe;
	int ret = 0, i;

	vi_channel_syncpt_init(chan);

	tegra_channel_ec_init(chan);

	if (!chan->vi->pg_mode) {
		/* Start the pipeline. */
		ret = media_entity_pipeline_start(&chan->video.entity, pipe);
		if (ret < 0)
			goto error_pipeline_start;
	}

	if (chan->bypass) {
		ret = tegra_channel_set_stream(chan, true);
		if (ret < 0)
			goto error_set_stream;
		/*
		nvhost_module_enable_clk(chan->vi->dev);
		tegra_mipi_bias_pad_enable();
		mutex_lock(&chan->vi->mipical_lock);
		tegra_channel_mipi_cal(chan, 1);
		mutex_unlock(&chan->vi->mipical_lock);
		nvhost_module_disable_clk(chan->vi->dev);*/
		return ret;
	}
	chan->capture_state = CAPTURE_IDLE;
	for (i = 0; i < chan->valid_ports; i++) {
		csi2_start_streaming(chan->vi->csi, chan->port[i]);
		/* ensure sync point state is clean */
		nvhost_syncpt_set_min_eq_max_ext(chan->vi->ndev,
							chan->syncpt[i]);
	}

	/* Note: Program VI registers after TPG, sensors and CSI streaming */
	ret = tegra_channel_capture_setup(chan);
	if (ret < 0)
		goto error_capture_setup;

	chan->sequence = 0;
	tegra_channel_init_ring_buffer(chan);

	/* Update clock and bandwidth based on the format */
	tegra_channel_update_clknbw(chan, 1);

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

	return 0;

error_capture_setup:
	if (!chan->vi->pg_mode)
		tegra_channel_set_stream(chan, false);
error_set_stream:
	if (!chan->vi->pg_mode)
		media_entity_pipeline_stop(&chan->video.entity);
error_pipeline_start:
	vq->start_streaming_called = 0;
	tegra_channel_queued_buf_done(chan, VB2_BUF_STATE_QUEUED);

	return ret;
}

void vi2_channel_stop_streaming(struct vb2_queue *vq)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);
	int index;
	bool is_streaming = atomic_read(&chan->is_streaming);

	if (!chan->bypass) {
		tegra_channel_stop_kthreads(chan);
		/* wait for last frame memory write ack */
		if (is_streaming && chan->capture_state == CAPTURE_GOOD)
			tegra_channel_capture_done(chan);
		/* free all the ring buffers */
		free_ring_buffers(chan, chan->num_buffers);
		/* dequeue buffers back to app which are in capture queue */
		tegra_channel_queued_buf_done(chan, VB2_BUF_STATE_ERROR);

		/* Disable clock gating to enable continuous clock */
		tegra_channel_write(chan, TEGRA_VI_CFG_CG_CTRL, DISABLE);
		for (index = 0; index < chan->valid_ports; index++) {
			csi2_stop_streaming(chan->vi->csi,
						chan->port[index]);
			/* Always clear single shot if armed at close */
			if (csi_read(chan, index, TEGRA_VI_CSI_SINGLE_SHOT))
				tegra_channel_clear_singleshot(chan, index);
		}
		/* Enable clock gating so VI can be clock gated if necessary */
		tegra_channel_write(chan, TEGRA_VI_CFG_CG_CTRL, ENABLE);
		tegra_csi_pad_control(chan->vi->csi, chan->port, DISABLE);
	}

	if (!chan->vi->pg_mode) {
		tegra_channel_set_stream(chan, false);
		media_entity_pipeline_stop(&chan->video.entity);
	}

	if (!chan->bypass)
		tegra_channel_update_clknbw(chan, 0);

	vi_channel_syncpt_free(chan);
}

int tegra_vi2_power_on(struct tegra_mc_vi *vi)
{
	int ret;

	if (atomic_add_return(1, &vi->power_on_refcnt) > 1)
		return 0;

	ret = nvhost_module_busy_ext(vi->ndev);
	if (ret) {
		dev_err(vi->dev, "%s:nvhost module is busy\n", __func__);
		return ret;
	}

	if (vi->reg) {
		ret = regulator_enable(vi->reg);
		if (ret) {
			dev_err(vi->dev, "%s: enable csi regulator failed.\n",
					__func__);
			goto error_regulator_fail;
		}
	}

	vi_write(vi, TEGRA_VI_CFG_CG_CTRL, 1);

	/* unpowergate VE */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);
	if (ret) {
		dev_err(vi->dev, "failed to unpower gate VI\n");
		goto error_unpowergate;
	}

	/* clock settings */
	ret = clk_prepare_enable(vi->clk);
	if (ret) {
		dev_err(vi->dev, "failed to enable vi clock\n");
		goto error_clk_enable;
	}

	ret = clk_set_rate(vi->clk, 0);
	if (ret) {
		dev_err(vi->dev, "failed to set vi clock\n");
		goto error_clk_set_rate;
	}

	ret = tegra_camera_emc_clk_enable();
	if (ret)
		goto err_emc_enable;

	return 0;
err_emc_enable:
error_clk_set_rate:
	clk_disable_unprepare(vi->clk);
error_clk_enable:
	tegra_powergate_partition(TEGRA_POWERGATE_VENC);
error_unpowergate:
	regulator_disable(vi->reg);
error_regulator_fail:
	nvhost_module_idle_ext(vi->ndev);

	return ret;
}

void tegra_vi2_power_off(struct tegra_mc_vi *vi)
{
	if (!atomic_dec_and_test(&vi->power_on_refcnt))
		return;

	tegra_channel_ec_close(vi);
	tegra_camera_emc_clk_disable();
	clk_disable_unprepare(vi->clk);
	tegra_powergate_partition(TEGRA_POWERGATE_VENC);
	regulator_disable(vi->reg);
	nvhost_module_idle_ext(vi->ndev);
}

int vi2_power_on(struct tegra_channel *chan)
{
	int ret = 0;
	struct tegra_mc_vi *vi;
	struct tegra_csi_device *csi;

	vi = chan->vi;
	csi = vi->csi;

	if (atomic_add_return(1, &vi->power_on_refcnt) == 1) {
	/*	tegra_vi2_power_on(vi);
		tegra_csi_power_on(csi);
		if (vi->pg_mode)
			tegra_vi->tpg_opened = true;
		else
			tegra_vi->sensor_opened = true;
			*/
	}

	if (!vi->pg_mode &&
		(atomic_add_return(1, &chan->power_on_refcnt) == 1)) {
		/* power on sensors connected in channel */
		//tegra_csi_channel_power_on(csi, chan->port);
		ret = tegra_channel_set_power(chan, 1);
	}

	return ret;
}

void vi2_power_off(struct tegra_channel *chan)
{
	int ret = 0;
	struct tegra_mc_vi *vi;
	struct tegra_csi_device *csi;

	vi = chan->vi;
	csi = vi->csi;

	if (!vi->pg_mode &&
		atomic_dec_and_test(&chan->power_on_refcnt)) {
		/* power off sensors connected in channel */
		//tegra_csi_channel_power_off(csi, chan->port);
		ret = tegra_channel_set_power(chan, 0);
		if (ret < 0)
			dev_err(vi->dev, "Failed to power off subdevices\n");
	}

	/* The last release then turn off power */
	if (atomic_dec_and_test(&vi->power_on_refcnt)) {
		/*tegra_csi_power_off(csi);
		tegra_vi2_power_off(vi);
		if (vi->pg_mode)
			tegra_vi->tpg_opened = false;
		else
			tegra_vi->sensor_opened = false;
			*/
	}
}
