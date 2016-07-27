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

#include "camera/mc_common.h"

/* VI2 register accessors */
void vi2_write(struct tegra_mc_vi *vi, unsigned int addr, u32 val)
{
	writel(val, vi->iomem + addr);
}

u32 vi2_channel_read(struct tegra_channel *chan,
			unsigned int addr)
{
	return readl(chan->vi->iomem + addr);
}

void vi2_channel_write(struct tegra_channel *chan,
			unsigned int addr, u32 val)
{
	writel(val, chan->vi->iomem + addr);
}

static void vi2_channel_csi_write(struct tegra_channel *chan,
				   unsigned int index, unsigned int addr,
				   u32 val)
{
	writel(val, chan->csibase[index] + addr);
}

static u32 vi2_channel_csi_read(struct tegra_channel *chan, unsigned int index,
					unsigned int addr)
{
	return readl(chan->csibase[index] + addr);
}

/* VI2 Media-Controller releated fops */
int vi2_power_on(struct tegra_mc_vi *vi)
{
	int ret;

	vi2_write(vi, TEGRA_VI_CFG_CG_CTRL, 1);

	/* unpowergate VE */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);
	if (ret)
		dev_err(vi->dev, "failed to unpower gate VI\n");

	return ret;
}

void vi2_power_off(struct tegra_mc_vi *vi)
{
	tegra_powergate_partition(TEGRA_POWERGATE_VENC);
}

/* VI2 channel operation related fops */
void vi2_channel_ec_init(struct tegra_channel *chan)
{
	/*
	 * Sync point FIFO full blocks host interface
	 * Below setting enables SW to process error recovery
	 */
	vi2_channel_write(chan, TEGRA_VI_CFG_VI_INCR_SYNCPT_CNTRL, 0x100);
}

int vi2_channel_capture_setup(struct tegra_channel *chan)
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
		vi2_channel_csi_write(chan, index, TEGRA_VI_CSI_ERROR_STATUS,
				       0xFFFFFFFF);
		vi2_channel_csi_write(chan, index, TEGRA_VI_CSI_IMAGE_DEF,
		  (bypass_pixel_transform << BYPASS_PXL_TRANSFORM_OFFSET) |
		  (format << IMAGE_DEF_FORMAT_OFFSET));
		vi2_channel_csi_write(chan, index, TEGRA_VI_CSI_IMAGE_DT,
				       data_type);
		vi2_channel_csi_write(chan, index, TEGRA_VI_CSI_IMAGE_SIZE_WC,
				       word_count);
		vi2_channel_csi_write(chan, index, TEGRA_VI_CSI_IMAGE_SIZE,
			  (height << IMAGE_SIZE_HEIGHT_OFFSET) | width);
	}

	return 0;
}

void vi2_channel_capture_frame_init(struct tegra_channel *chan,
				     struct tegra_channel_buffer *buf,
				     u32 *thresh)
{
	int index = 0;
	int valid_ports = chan->valid_ports;
	int bytes_per_line = chan->format.bytesperline;
	u32 val, frame_start;

	for (index = 0; index < valid_ports; index++) {
		/* Program buffer address by using surface 0 */
		vi2_channel_csi_write(chan, index,
				TEGRA_VI_CSI_SURFACE0_OFFSET_MSB, 0x0);
		vi2_channel_csi_write(chan, index,
			TEGRA_VI_CSI_SURFACE0_OFFSET_LSB,
			(buf->addr + chan->buffer_offset[index]));
		vi2_channel_csi_write(chan, index,
			TEGRA_VI_CSI_SURFACE0_STRIDE, bytes_per_line);

		/* Program syncpoints */
		thresh[index] = nvhost_syncpt_incr_max_ext(chan->vi->ndev,
					chan->syncpt[index], 1);
		/* Do not arm sync points if FIFO had entries before */
		if (!chan->syncpoint_fifo[index]) {
			frame_start = VI_CSI_PP_FRAME_START(chan->port[index]);
			val = VI_CFG_VI_INCR_SYNCPT_COND(frame_start) |
				chan->syncpt[index];
			vi2_channel_write(chan,
				TEGRA_VI_CFG_VI_INCR_SYNCPT, val);
		} else
			chan->syncpoint_fifo[index]--;
	}
}

void vi2_channel_capture_frame_enable(struct tegra_channel *chan)
{
	int index;
	u32 val;
	int valid_ports = chan->valid_ports;

	/* Bit controls VI memory write, enable after all regs */
	for (index = 0; index < valid_ports; index++) {
		val = vi2_channel_csi_read(chan, index,
				TEGRA_VI_CSI_IMAGE_DEF);
		vi2_channel_csi_write(chan, index,
				TEGRA_VI_CSI_IMAGE_DEF,
				val | IMAGE_DEF_DEST_MEM);
	}
}

int vi2_channel_capture_frame(struct tegra_channel *chan,
			       struct timespec *ts, u32 *thresh)
{

	int index = 0;
	int valid_ports = chan->valid_ports;
	int err = 0;

	/* Ensure all CSI ports are ready with setup to avoid timing issue */
	for (index = 0; index < valid_ports; index++)
		vi2_channel_csi_write(chan, index,
			TEGRA_VI_CSI_SINGLE_SHOT, SINGLE_SHOT_CAPTURE);

	chan->capture_state = CAPTURE_GOOD;
	for (index = 0; index < valid_ports; index++) {
		err = nvhost_syncpt_wait_timeout_ext(chan->vi->ndev,
			chan->syncpt[index], thresh[index],
			chan->timeout, NULL, ts);
		if (err) {
			dev_err(&chan->video.dev,
				"frame start syncpt timeout!%d\n", index);
			chan->capture_state = CAPTURE_TIMEOUT;
			return err;
		}
	}

	return err;
}

int vi2_channel_error_status(struct tegra_channel *chan)
{
	u32 val;
	int err = 0;
	int index = 0;

	for (index = 0; index < chan->valid_ports; index++) {
		val = vi2_channel_csi_read(chan, index,
				TEGRA_VI_CSI_ERROR_STATUS);
		vi2_channel_csi_write(chan, index,
				TEGRA_VI_CSI_ERROR_STATUS, val);
		err |= val;
		err |= tegra_csi_error(chan->vi->csi, chan->port[index]);
	}

	if (err)
		dev_err(chan->vi->dev, "%s:error %x frame %d\n",
				__func__, err, chan->sequence);
	return err;
}

int vi2_channel_capture_done(struct tegra_channel *chan,
			      struct tegra_channel_buffer *buf,
			      struct timespec *ts)
{
	int index, err;
	int bytes_per_line = chan->format.bytesperline;
	u32 val, mw_ack_done;
	u32 thresh[TEGRA_CSI_BLOCKS] = { 0 };

	for (index = 0; index < chan->valid_ports; index++) {
		/* Program buffer address by using surface 0 */
		vi2_channel_csi_write(chan, index,
				TEGRA_VI_CSI_SURFACE0_OFFSET_MSB, 0x0);
		vi2_channel_csi_write(chan, index,
			TEGRA_VI_CSI_SURFACE0_OFFSET_LSB,
			(buf->addr + chan->buffer_offset[index]));
		vi2_channel_csi_write(chan, index,
			TEGRA_VI_CSI_SURFACE0_STRIDE, bytes_per_line);

		/* Program syncpoints */
		thresh[index] = nvhost_syncpt_incr_max_ext(chan->vi->ndev,
					chan->syncpt[index], 1);
		mw_ack_done = VI_CSI_MW_ACK_DONE(chan->port[index]);
		val = VI_CFG_VI_INCR_SYNCPT_COND(mw_ack_done) |
				chan->syncpt[index];
		vi2_channel_write(chan,
				TEGRA_VI_CFG_VI_INCR_SYNCPT, val);
		vi2_channel_csi_write(chan, index,
			TEGRA_VI_CSI_SINGLE_SHOT, SINGLE_SHOT_CAPTURE);
	}

	for (index = 0; index < chan->valid_ports; index++) {
		err = nvhost_syncpt_wait_timeout_ext(chan->vi->ndev,
			chan->syncpt[index], thresh[index],
			chan->timeout, NULL, ts);
		if (err) {
			dev_err(&chan->video.dev,
				"MW_ACK_DONE syncpoint time out!%d\n", index);
			return err;
		}
	}

	return 0;
}

void vi2_channel_ec_recover(struct tegra_channel *chan)
{
	u32 error_val = vi2_channel_read(chan,
				TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR);
	int index, valid_ports = chan->valid_ports;
	u32 frame_start, val;

	/* Get error status for debugging */
	for (index = 0; index < valid_ports; index++) {
		val = vi2_channel_csi_read(chan, index,
				TEGRA_VI_CSI_ERROR_STATUS);
		dev_dbg(&chan->video.dev,
			"TEGRA_VI_CSI_ERROR_STATUS 0x%08x\n", val);
		tegra_csi_status(chan->vi->csi, chan->port[index]);
	}

	/* Disable clock gating to enable continuous clock */
	vi2_channel_write(chan, TEGRA_VI_CFG_CG_CTRL, DISABLE);

	/* Clear CSI state */
	for (index = 0; index < valid_ports; index++) {
		tegra_csi_error_recover(chan->vi->csi, chan->port[index]);
		vi2_channel_csi_write(chan, index,
				TEGRA_VI_CSI_IMAGE_DEF, 0);
		/* Clear single shot */
		vi2_channel_csi_write(chan, index,
				TEGRA_VI_CSI_SW_RESET, 0xF);
		vi2_channel_csi_write(chan, index,
				TEGRA_VI_CSI_SW_RESET, 0x0);
	}

	/* Clear VI errors */
	for (index = 0; index < valid_ports; index++) {
		frame_start = VI_CSI_PP_FRAME_START(chan->port[index]);
		if (error_val & frame_start)
			chan->syncpoint_fifo[index] = SYNCPT_FIFO_DEPTH;
	}

	/* Clear FIFO error status */
	vi2_channel_write(chan,
		TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR, error_val);

	/* Enable clock gating so VI can be clock gated if necessary */
	vi2_channel_write(chan, TEGRA_VI_CFG_CG_CTRL, ENABLE);
}

void vi2_channel_stop_streaming(struct tegra_channel *chan)
{
	int index;

	/* Disable clock gating to enable continuous clock */
	vi2_channel_write(chan, TEGRA_VI_CFG_CG_CTRL, DISABLE);

	for (index = 0; index < chan->valid_ports; index++) {
		tegra_csi_stop_streaming(chan->vi->csi,
				chan->port[index]);
		/* Always clear single shot if armed at close */
		if (vi2_channel_csi_read(chan, index,
					TEGRA_VI_CSI_SINGLE_SHOT)) {
			vi2_channel_csi_write(chan, index,
					TEGRA_VI_CSI_SW_RESET, 0xF);
			vi2_channel_csi_write(chan, index,
					TEGRA_VI_CSI_SW_RESET, 0x0);
		}
	}

	/* Enable clock gating so VI can be clock gated if necessary */
	vi2_channel_write(chan, TEGRA_VI_CFG_CG_CTRL, ENABLE);
}
