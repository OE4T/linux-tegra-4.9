/*
 * Tegra ISP capture operations
 *
 * Tegra NvCapture ISP KMD
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Sudhir Vyas <svyas@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/completion.h>
#include <linux/nvhost.h>
#include <linux/of_platform.h>
#include <linux/printk.h>
#include <linux/tegra-capture-ivc.h>
#include <media/capture_common.h>
#include <media/capture_isp.h>
#include <media/isp_channel.h>
#include <media/mc_common.h>

#include "soc/tegra/camrtc-capture.h"
#include "soc/tegra/camrtc-capture-messages.h"

#define CAPTURE_CHANNEL_UNKNOWN_RESP 0xFFFFFFFF
#define CAPTURE_CHANNEL_ISP_INVALID_ID 0xFFFF

struct isp_desc_rec {
	struct capture_common_buf requests;
	struct capture_common_buf requests_isp;
	size_t request_buf_size;
	uint32_t queue_depth;
	uint32_t request_size;
	uint32_t desc_mem;

	struct mutex unpins_list_lock;
	struct capture_common_unpins **unpins_list;
};

/* ISP capture context per channel */
struct isp_capture {
	uint16_t channel_id;
	struct device *rtcpu_dev;
	struct tegra_isp_channel *isp_channel;

	/* isp capture desc and it's ring buffer related details */
	struct isp_desc_rec capture_desc_ctx;

	/* isp program desc and it's ring buffer related details */
	struct isp_desc_rec program_desc_ctx;

	struct syncpoint_info progress_sp;
	struct syncpoint_info stats_progress_sp;

	struct completion control_resp;
	struct completion capture_resp;
	struct completion capture_program_resp;

	struct mutex control_msg_lock;
	struct CAPTURE_CONTROL_MSG control_resp_msg;
};

static void isp_capture_ivc_control_callback(const void *ivc_resp,
		const void *pcontext)
{
	const struct CAPTURE_CONTROL_MSG *control_msg = ivc_resp;
	struct isp_capture *capture = (struct isp_capture *)pcontext;
	struct tegra_isp_channel *chan = capture->isp_channel;

	if (unlikely(capture == NULL)) {
		dev_err(chan->isp_dev, "%s: invalid context", __func__);
		return;
	}

	if (unlikely(control_msg == NULL)) {
		dev_err(chan->isp_dev, "%s: invalid response", __func__);
		return;
	}

	switch (control_msg->header.msg_id) {
	case CAPTURE_CHANNEL_ISP_SETUP_RESP:
	case CAPTURE_CHANNEL_ISP_RESET_RESP:
	case CAPTURE_CHANNEL_ISP_RELEASE_RESP:
		memcpy(&capture->control_resp_msg, control_msg,
				sizeof(*control_msg));
		complete(&capture->control_resp);
		break;
	default:
		dev_err(chan->isp_dev,
			"%s: unknown capture isp control resp", __func__);
		break;
	}
}

static void isp_capture_request_unpin(struct tegra_isp_channel *chan,
		uint32_t buffer_index);

static void isp_capture_program_request_unpin(struct tegra_isp_channel *chan,
		uint32_t buffer_index);

static void isp_capture_ivc_status_callback(const void *ivc_resp,
		const void *pcontext)
{
	struct CAPTURE_MSG *status_msg = (struct CAPTURE_MSG *)ivc_resp;
	struct isp_capture *capture = (struct isp_capture *)pcontext;
	struct tegra_isp_channel *chan = capture->isp_channel;
	uint32_t buffer_index;

	if (unlikely(capture == NULL)) {
		dev_err(chan->isp_dev, "%s: invalid context", __func__);
		return;
	}

	if (unlikely(status_msg == NULL)) {
		dev_err(chan->isp_dev, "%s: invalid response", __func__);
		return;
	}

	switch (status_msg->header.msg_id) {
	case CAPTURE_ISP_STATUS_IND:
		buffer_index = status_msg->capture_isp_status_ind.buffer_index;
		isp_capture_request_unpin(chan, buffer_index);
		dma_sync_single_range_for_cpu(capture->rtcpu_dev,
		    capture->capture_desc_ctx.requests.iova,
		    buffer_index * capture->capture_desc_ctx.request_size,
		    capture->capture_desc_ctx.request_size, DMA_FROM_DEVICE);
		complete(&capture->capture_resp);
		dev_dbg(chan->isp_dev, "%s: status chan_id %u msg_id %u\n",
				__func__, status_msg->header.channel_id,
				status_msg->header.msg_id);
		break;
	case CAPTURE_ISP_PROGRAM_STATUS_IND:
		buffer_index =
			status_msg->capture_isp_program_status_ind.buffer_index;
		isp_capture_program_request_unpin(chan, buffer_index);
		dma_sync_single_range_for_cpu(capture->rtcpu_dev,
		    capture->program_desc_ctx.requests.iova,
		    buffer_index * capture->program_desc_ctx.request_size,
		    capture->program_desc_ctx.request_size, DMA_FROM_DEVICE);
		complete(&capture->capture_program_resp);
		dev_dbg(chan->isp_dev,
			"%s: isp_ program status chan_id %u msg_id %u\n",
			__func__, status_msg->header.channel_id,
			status_msg->header.msg_id);
		break;
	default:
		dev_err(chan->isp_dev,
			"%s: unknown capture resp", __func__);
		break;
	}
}

int isp_capture_init(struct tegra_isp_channel *chan)
{
	struct isp_capture *capture;
	struct device_node *dn;
	struct platform_device *rtc_pdev;

	dev_dbg(chan->isp_dev, "%s++\n", __func__);
	dn = of_find_node_by_path("tegra-camera-rtcpu");
	if (of_device_is_available(dn) == 0) {
		dev_err(chan->isp_dev, "failed to find rtcpu device node\n");
		return -ENODEV;
	}
	rtc_pdev = of_find_device_by_node(dn);
	if (rtc_pdev == NULL) {
		dev_err(chan->isp_dev, "failed to find rtcpu platform\n");
		return -ENODEV;
	}

	capture = devm_kzalloc(chan->isp_dev,
			sizeof(*capture), GFP_KERNEL);
	if (unlikely(capture == NULL)) {
		dev_err(chan->isp_dev, "failed to allocate capture channel\n");
		return -ENOMEM;
	}

	capture->rtcpu_dev = &rtc_pdev->dev;

	init_completion(&capture->control_resp);
	init_completion(&capture->capture_resp);
	init_completion(&capture->capture_program_resp);

	mutex_init(&capture->control_msg_lock);
	mutex_init(&capture->capture_desc_ctx.unpins_list_lock);
	mutex_init(&capture->program_desc_ctx.unpins_list_lock);

	capture->isp_channel = chan;
	chan->capture_data = capture;

	capture->channel_id = CAPTURE_CHANNEL_ISP_INVALID_ID;

	return 0;
}

void isp_capture_shutdown(struct tegra_isp_channel *chan)
{
	struct isp_capture *capture = chan->capture_data;

	dev_dbg(chan->isp_dev, "%s--\n", __func__);
	if (capture == NULL)
		return;

	if (capture->channel_id != CAPTURE_CHANNEL_ISP_INVALID_ID)
		isp_capture_release(chan, 0);

	devm_kfree(chan->isp_dev, capture);
	chan->capture_data = NULL;
}

static int isp_capture_ivc_send_control(struct tegra_isp_channel *chan,
		const struct CAPTURE_CONTROL_MSG *msg, size_t size,
		uint32_t resp_id)
{
	struct isp_capture *capture = chan->capture_data;
	struct CAPTURE_MSG_HEADER resp_header = msg->header;
	uint32_t timeout = HZ;
	int err = 0;

	dev_dbg(chan->isp_dev, "%s: sending chan_id %u msg_id %u\n",
			__func__, resp_header.channel_id, resp_header.msg_id);

	resp_header.msg_id = resp_id;

	/* Send capture control IVC message */
	mutex_lock(&capture->control_msg_lock);
	err = tegra_capture_ivc_control_submit(msg, size);
	if (err < 0) {
		dev_err(chan->isp_dev, "IVC control submit failed\n");
		goto fail;
	}

	timeout = wait_for_completion_killable_timeout(
			&capture->control_resp, timeout);
	if (timeout <= 0) {
		dev_err(chan->isp_dev,
			"no reply from camera processor\n");
		err = -ETIMEDOUT;
		goto fail;
	}

	if (memcmp(&resp_header, &capture->control_resp_msg.header,
			sizeof(resp_header)) != 0) {
		dev_err(chan->isp_dev,
			"unexpected response from camera processor\n");
		err = -EINVAL;
		goto fail;
	}
	mutex_unlock(&capture->control_msg_lock);

	dev_dbg(chan->isp_dev, "%s: response chan_id %u msg_id %u\n",
			__func__, capture->control_resp_msg.header.channel_id,
			capture->control_resp_msg.header.msg_id);
	return 0;

fail:
	mutex_unlock(&capture->control_msg_lock);
	return err;
}

static int isp_capture_setup_syncpts(struct tegra_isp_channel *chan);
static void isp_capture_release_syncpts(struct tegra_isp_channel *chan);

static int isp_capture_setup_syncpt(struct tegra_isp_channel *chan,
				const char *name, bool enable,
				struct syncpoint_info *sp)
{
	struct platform_device *pdev = chan->ndev;
	uint32_t id;

	memset(sp, 0, sizeof(*sp));
	sp->gos_index = GOS_INDEX_INVALID;

	if (!enable)
		return 0;

	id = nvhost_get_syncpt_client_managed(pdev, name);
	if (id == 0)
		return -ENODEV;

	sp->id = id;

	/* TODO: Add GOS and shim_addr support */

	return 0;
}

static int isp_capture_setup_syncpts(struct tegra_isp_channel *chan)
{
	struct isp_capture *capture = chan->capture_data;
	int err = 0;

	err = isp_capture_setup_syncpt(chan, "progress", true,
			&capture->progress_sp);
	if (err < 0)
		goto fail;

	err = isp_capture_setup_syncpt(chan, "stats_progress",
				true,
				&capture->stats_progress_sp);
	if (err < 0)
		goto fail;

	return 0;

fail:
	isp_capture_release_syncpts(chan);
	return err;
}

static void isp_capture_release_syncpt(struct tegra_isp_channel *chan,
				struct syncpoint_info *sp)
{
	struct platform_device *pdev = chan->ndev;

	if (sp->id)
		nvhost_syncpt_put_ref_ext(pdev, sp->id);

	memset(sp, 0, sizeof(*sp));
}

static void isp_capture_release_syncpts(struct tegra_isp_channel *chan)
{
	struct isp_capture *capture = chan->capture_data;

	isp_capture_release_syncpt(chan, &capture->progress_sp);
	isp_capture_release_syncpt(chan, &capture->stats_progress_sp);
}

int isp_capture_setup(struct tegra_isp_channel *chan,
		struct isp_capture_setup *setup)
{
	struct isp_capture *capture = chan->capture_data;
	uint32_t transaction;
	struct CAPTURE_CONTROL_MSG control_msg;
	struct CAPTURE_CONTROL_MSG *resp_msg = &capture->control_resp_msg;
	struct capture_channel_isp_config *config =
		&control_msg.channel_isp_setup_req.channel_config;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->isp_dev,
			 "%s: isp capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id != CAPTURE_CHANNEL_ISP_INVALID_ID) {
		dev_err(chan->isp_dev,
			"%s: already setup, release first\n", __func__);
		return -EEXIST;
	}

	dev_dbg(chan->isp_dev, "chan flags %u\n", setup->channel_flags);
	dev_dbg(chan->isp_dev, "queue depth %u\n", setup->queue_depth);
	dev_dbg(chan->isp_dev, "request size %u\n", setup->request_size);

	if (setup->channel_flags == 0 ||
			setup->queue_depth == 0 ||
			setup->request_size == 0)
		return -EINVAL;

	/* pin the capture descriptor ring buffer to RTCPU */
	dev_dbg(chan->isp_dev, "%s: descr buffer handle 0x%x\n",
			__func__, setup->mem);
	err = capture_common_pin_memory(capture->rtcpu_dev,
			setup->mem, &capture->capture_desc_ctx.requests);
	if (err < 0) {
		dev_err(chan->isp_dev, "%s: memory setup failed\n", __func__);
		return -EFAULT;
	}

	/* pin the capture descriptor ring buffer to ISP */
	err = capture_common_pin_memory(chan->isp_dev,
			setup->mem, &capture->capture_desc_ctx.requests_isp);
	if (err < 0) {
		dev_err(chan->isp_dev, "%s: memory setup failed\n", __func__);
		return -EFAULT;
	}

	capture->capture_desc_ctx.desc_mem = setup->mem;

	/* cache isp capture desc ring buffer details */
	capture->capture_desc_ctx.queue_depth = setup->queue_depth;
	capture->capture_desc_ctx.request_size = setup->request_size;
	capture->capture_desc_ctx.request_buf_size = setup->request_size *
							setup->queue_depth;

	/* allocate isp capture desc unpin list based on queue depth */
	capture->capture_desc_ctx.unpins_list = devm_kzalloc(chan->isp_dev,
			sizeof(struct capture_common_unpins *) *
				capture->capture_desc_ctx.queue_depth,
				GFP_KERNEL);
	if (unlikely(capture->capture_desc_ctx.unpins_list == NULL)) {
		dev_err(chan->isp_dev, "failed to allocate unpins array\n");
		goto unpins_list_fail;
	}

	/* pin the isp program descriptor ring buffer */
	dev_dbg(chan->isp_dev, "%s: descr buffer handle %u\n",
			__func__, setup->isp_program_mem);
	err = capture_common_pin_memory(capture->rtcpu_dev,
				setup->isp_program_mem,
				&capture->program_desc_ctx.requests);
	if (err < 0) {
		dev_err(chan->isp_dev,
			"%s: isp_program memory setup failed\n", __func__);
		goto prog_pin_fail;
	}

	/* pin the isp program descriptor ring buffer to ISP */
	err = capture_common_pin_memory(chan->isp_dev,
				setup->isp_program_mem,
				&capture->program_desc_ctx.requests_isp);
	if (err < 0) {
		dev_err(chan->isp_dev,
			"%s: isp_program memory setup failed\n", __func__);
		goto prog_pin_fail;
	}

	capture->program_desc_ctx.desc_mem = setup->isp_program_mem;

	/* cache isp program desc ring buffer details */
	capture->program_desc_ctx.queue_depth = setup->isp_program_queue_depth;
	capture->program_desc_ctx.request_size =
					setup->isp_program_request_size;
	capture->program_desc_ctx.request_buf_size =
					setup->isp_program_request_size *
						setup->isp_program_queue_depth;

	/* allocate isp program unpin list based on queue depth */
	capture->program_desc_ctx.unpins_list = devm_kzalloc(chan->isp_dev,
			sizeof(struct capture_common_unpins *) *
				capture->program_desc_ctx.queue_depth,
				GFP_KERNEL);
	if (unlikely(capture->program_desc_ctx.unpins_list == NULL)) {
		dev_err(chan->isp_dev,
			"failed to allocate isp program unpins array\n");
		goto prog_unpins_list_fail;
	}

	err = isp_capture_setup_syncpts(chan);
	if (err < 0) {
		dev_err(chan->isp_dev, "%s: syncpt setup failed\n", __func__);
		goto syncpt_fail;
	}

	err = tegra_capture_ivc_register_control_cb(
			&isp_capture_ivc_control_callback,
			&transaction, capture);
	if (err < 0) {
		dev_err(chan->isp_dev, "failed to register control callback\n");
		goto control_cb_fail;
	}

	/* Fill in control config msg to be sent over ctrl ivc chan to RTCPU */
	memset(&control_msg, 0, sizeof(control_msg));

	control_msg.header.msg_id = CAPTURE_CHANNEL_ISP_SETUP_REQ;
	control_msg.header.transaction = transaction;

	config->channel_flags = setup->channel_flags;

	config->request_queue_depth = setup->queue_depth;
	config->request_size = setup->request_size;
	config->requests = capture->capture_desc_ctx.requests.iova;

	config->program_queue_depth = setup->isp_program_queue_depth;
	config->program_size = setup->isp_program_request_size;
	config->programs = capture->program_desc_ctx.requests.iova;

	config->progress_sp = capture->progress_sp;
	config->stats_progress_sp = capture->stats_progress_sp;

	err = isp_capture_ivc_send_control(chan, &control_msg,
			sizeof(control_msg), CAPTURE_CHANNEL_ISP_SETUP_RESP);
	if (err < 0)
		goto submit_fail;

	if (resp_msg->channel_isp_setup_resp.result != CAPTURE_OK) {
		dev_err(chan->isp_dev, "%s: control failed, errno %d", __func__,
			resp_msg->channel_setup_resp.result);
		err = -EINVAL;
		goto submit_fail;
	}

	capture->channel_id = resp_msg->channel_isp_setup_resp.channel_id;

	err = tegra_capture_ivc_notify_chan_id(capture->channel_id,
			transaction);
	if (err < 0) {
		dev_err(chan->isp_dev, "failed to update control callback\n");
		goto cb_fail;
	}

	err = tegra_capture_ivc_register_capture_cb(
			&isp_capture_ivc_status_callback,
			capture->channel_id, capture);
	if (err < 0) {
		dev_err(chan->isp_dev, "failed to register capture callback\n");
		goto cb_fail;
	}

	return 0;

cb_fail:
	isp_capture_release(chan, CAPTURE_CHANNEL_RESET_FLAG_IMMEDIATE);
	return err;
submit_fail:
	tegra_capture_ivc_unregister_control_cb(transaction);
control_cb_fail:
	isp_capture_release_syncpts(chan);
syncpt_fail:
	devm_kfree(chan->isp_dev, capture->program_desc_ctx.unpins_list);
prog_unpins_list_fail:
	capture_common_unpin_memory(&capture->program_desc_ctx.requests);
prog_pin_fail:
	devm_kfree(chan->isp_dev, capture->capture_desc_ctx.unpins_list);
unpins_list_fail:
	capture_common_unpin_memory(&capture->capture_desc_ctx.requests);
	return err;
}

int isp_capture_reset(struct tegra_isp_channel *chan,
		uint32_t reset_flags)
{
	struct isp_capture *capture = chan->capture_data;
	struct CAPTURE_CONTROL_MSG control_msg;
	struct CAPTURE_CONTROL_MSG *resp_msg = &capture->control_resp_msg;
	int i;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->isp_dev,
			 "%s: isp capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_ISP_INVALID_ID) {
		dev_err(chan->isp_dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	memset(&control_msg, 0, sizeof(control_msg));
	control_msg.header.msg_id = CAPTURE_CHANNEL_ISP_RESET_REQ;
	control_msg.header.channel_id = capture->channel_id;
	control_msg.channel_isp_reset_req.reset_flags = reset_flags;

	err = isp_capture_ivc_send_control(chan, &control_msg,
			sizeof(control_msg), CAPTURE_CHANNEL_ISP_RESET_RESP);
	if (err < 0)
		goto error;

	if (resp_msg->channel_isp_reset_resp.result != CAPTURE_OK) {
		dev_err(chan->isp_dev, "%s: control failed, errno %d", __func__,
			resp_msg->channel_isp_reset_resp.result);
		err = -EINVAL;
		goto error;
	}

	for (i = 0; i < capture->program_desc_ctx.queue_depth; i++)
		isp_capture_program_request_unpin(chan, i);

	for (i = 0; i < capture->capture_desc_ctx.queue_depth; i++)
		isp_capture_request_unpin(chan, i);

	return 0;

error:
	return err;
}

int isp_capture_release(struct tegra_isp_channel *chan,
		uint32_t reset_flags)
{
	struct isp_capture *capture = chan->capture_data;
	struct CAPTURE_CONTROL_MSG control_msg;
	struct CAPTURE_CONTROL_MSG *resp_msg = &capture->control_resp_msg;
	int i;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->isp_dev,
			 "%s: isp capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_ISP_INVALID_ID) {
		dev_err(chan->isp_dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	memset(&control_msg, 0, sizeof(control_msg));

	control_msg.header.msg_id = CAPTURE_CHANNEL_ISP_RELEASE_REQ;
	control_msg.header.channel_id = capture->channel_id;
	control_msg.channel_release_req.reset_flags = reset_flags;

	err = isp_capture_ivc_send_control(chan, &control_msg,
			sizeof(control_msg), CAPTURE_CHANNEL_ISP_RELEASE_RESP);
	if (err < 0)
		goto error;

	if (resp_msg->channel_isp_release_resp.result != CAPTURE_OK) {
		dev_err(chan->isp_dev, "%s: control failed, errno %d", __func__,
			resp_msg->channel_release_resp.result);
		err = -EINVAL;
		goto error;
	}

	err = tegra_capture_ivc_unregister_capture_cb(capture->channel_id);
	if (err < 0) {
		dev_err(chan->isp_dev,
			"failed to unregister capture callback\n");
		goto error;
	}

	err = tegra_capture_ivc_unregister_control_cb(capture->channel_id);
	if (err < 0) {
		dev_err(chan->isp_dev,
			"failed to unregister control callback\n");
		goto error;
	}

	for (i = 0; i < capture->program_desc_ctx.queue_depth; i++)
		isp_capture_program_request_unpin(chan, i);

	capture_common_unpin_memory(&capture->program_desc_ctx.requests);
	capture_common_unpin_memory(&capture->program_desc_ctx.requests_isp);

	for (i = 0; i < capture->capture_desc_ctx.queue_depth; i++)
		isp_capture_request_unpin(chan, i);

	isp_capture_release_syncpts(chan);

	capture_common_unpin_memory(&capture->capture_desc_ctx.requests);
	capture_common_unpin_memory(&capture->capture_desc_ctx.requests_isp);
	capture->channel_id = CAPTURE_CHANNEL_ISP_INVALID_ID;

	return 0;

error:
	return err;
}

static int isp_capture_read_syncpt(struct tegra_isp_channel *chan,
		struct syncpoint_info *sp, uint32_t *val)
{
	int err;

	if (sp->id) {
		err = nvhost_syncpt_read_ext_check(chan->ndev,
						sp->id, val);
		if (err < 0) {
			dev_err(chan->isp_dev,
				"%s: get syncpt %i val failed\n", __func__,
				sp->id);
			return -EINVAL;
		}
	}

	return 0;
}

int isp_capture_get_info(struct tegra_isp_channel *chan,
		struct isp_capture_info *info)
{
	struct isp_capture *capture = chan->capture_data;
	int err;

	if (capture == NULL) {
		dev_err(chan->isp_dev,
			 "%s: isp capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_ISP_INVALID_ID) {
		dev_err(chan->isp_dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	if (info == NULL) {
		dev_err(chan->isp_dev,
			"%s: Invalid user parameter\n", __func__);
		return -EINVAL;
	}

	info->syncpts.progress_syncpt = capture->progress_sp.id;
	info->syncpts.stats_progress_syncpt =
			capture->stats_progress_sp.id;

	err = isp_capture_read_syncpt(chan, &capture->progress_sp,
			&info->syncpts.progress_syncpt_val);
	if (err < 0)
		return err;

	err = isp_capture_read_syncpt(chan, &capture->stats_progress_sp,
			&info->syncpts.stats_progress_syncpt_val);
	if (err < 0)
		return err;

	return 0;
}

static void isp_capture_request_unpin(struct tegra_isp_channel *chan,
		uint32_t buffer_index)
{
	struct isp_capture *capture = chan->capture_data;
	struct capture_common_unpins *unpins;
	int i = 0;

	mutex_lock(&capture->capture_desc_ctx.unpins_list_lock);
	unpins = capture->capture_desc_ctx.unpins_list[buffer_index];
	if (unpins == NULL) {
		mutex_unlock(&capture->capture_desc_ctx.unpins_list_lock);
		return;
	}
	for (i = 0; i < unpins->num_unpins; i++)
		capture_common_unpin_memory(&unpins->data[i]);
	devm_kfree(chan->isp_dev, unpins);
	capture->capture_desc_ctx.unpins_list[buffer_index] = NULL;
	mutex_unlock(&capture->capture_desc_ctx.unpins_list_lock);
}

static void isp_capture_program_request_unpin(struct tegra_isp_channel *chan,
		uint32_t buffer_index)
{
	struct isp_capture *capture = chan->capture_data;
	struct capture_common_unpins *unpins;
	int i = 0;

	mutex_lock(&capture->program_desc_ctx.unpins_list_lock);
	unpins = capture->program_desc_ctx.unpins_list[buffer_index];
	if (unpins == NULL) {
		mutex_unlock(&capture->program_desc_ctx.unpins_list_lock);
		return;
	}
	for (i = 0; i < unpins->num_unpins; i++)
		capture_common_unpin_memory(&unpins->data[i]);
	devm_kfree(chan->isp_dev, unpins);
	capture->program_desc_ctx.unpins_list[buffer_index] = NULL;
	mutex_unlock(&capture->program_desc_ctx.unpins_list_lock);
}

int isp_capture_program_request(struct tegra_isp_channel *chan,
		struct isp_program_req *req)
{
	struct isp_capture *capture = chan->capture_data;
	struct CAPTURE_MSG capture_msg;
	int err = 0;
	struct capture_common_unpins *unpins;
	struct capture_common_pin_req cap_common_req;
	struct capture_common_relocs *relocs;
	uint32_t __user *reloc_relatives;

	if (capture == NULL) {
		dev_err(chan->isp_dev,
			"%s: isp capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_ISP_INVALID_ID) {
		dev_err(chan->isp_dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	memset(&capture_msg, 0, sizeof(capture_msg));
	capture_msg.header.msg_id = CAPTURE_ISP_PROGRAM_REQUEST_REQ;
	capture_msg.header.channel_id = capture->channel_id;
	capture_msg.capture_isp_program_request_req.buffer_index =
				req->buffer_index;

	/* memory pin and reloc */
	unpins = devm_kzalloc(chan->isp_dev,
			sizeof(struct capture_common_unpins) +
				(sizeof(struct capture_common_buf) *
				req->isp_program_relocs.num_relocs),
			GFP_KERNEL);
	if (unpins == NULL)
		return -ENOMEM;

	relocs = devm_kzalloc(chan->isp_dev,
			sizeof(struct capture_common_relocs) +
			(sizeof(uint32_t) * req->isp_program_relocs.num_relocs),
			GFP_KERNEL);
	if (unlikely(relocs == NULL)) {
		dev_err(chan->isp_dev, "failed to allocate relocs\n");
		goto fail;
	}

	relocs->num_relocs = req->isp_program_relocs.num_relocs;

	reloc_relatives = (uint32_t __user *)
			(uintptr_t)req->isp_program_relocs.reloc_relatives;

	err = copy_from_user(relocs->reloc_relatives, reloc_relatives,
			relocs->num_relocs * sizeof(uint32_t)) ? -EFAULT : 0;
	if (err < 0) {
		dev_err(chan->isp_dev, "failed copy program user-relocs\n");
		goto reloc_fail;
	}

	cap_common_req.dev = chan->isp_dev;
	cap_common_req.rtcpu_dev = capture->rtcpu_dev;
	cap_common_req.unpins = unpins;
	cap_common_req.requests = &capture->program_desc_ctx.requests;
	cap_common_req.requests_dev = &capture->program_desc_ctx.requests_isp;
	cap_common_req.request_size = capture->program_desc_ctx.request_size;
	cap_common_req.request_offset = req->buffer_index *
					capture->program_desc_ctx.request_size;
	cap_common_req.relocs = relocs;
	cap_common_req.requests_mem = capture->program_desc_ctx.desc_mem;

	capture_common_request_pin_and_reloc(&cap_common_req);

	/* add pinned memory ctx to unpins_list */
	mutex_lock(&capture->program_desc_ctx.unpins_list_lock);
	capture->program_desc_ctx.unpins_list[req->buffer_index] = unpins;
	mutex_unlock(&capture->program_desc_ctx.unpins_list_lock);

	dev_dbg(chan->isp_dev, "%s: sending chan_id %u msg_id %u buf:%u\n",
			__func__, capture_msg.header.channel_id,
			capture_msg.header.msg_id, req->buffer_index);

	err = tegra_capture_ivc_capture_submit(&capture_msg,
			sizeof(capture_msg));
	if (err < 0) {
		dev_err(chan->isp_dev, "IVC program submit failed\n");
		goto reloc_fail;
	}

	return 0;

reloc_fail:
	devm_kfree(chan->isp_dev, relocs);
fail:
	isp_capture_program_request_unpin(chan, req->buffer_index);
	return err;
}

int isp_capture_program_status(struct tegra_isp_channel *chan)
{
	struct isp_capture *capture = chan->capture_data;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->isp_dev,
			 "%s: isp capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_ISP_INVALID_ID) {
		dev_err(chan->isp_dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	dev_dbg(chan->isp_dev, "%s: waiting for isp program status\n",
		__func__);

	/* no timeout as an isp_program may get used for mutliple frames */
	err = wait_for_completion_killable(&capture->capture_program_resp);
	if (err <= 0) {
		dev_err(chan->isp_dev,
			"no reply from camera processor\n");
		return err;
	}

	return err;
}

int isp_capture_request(struct tegra_isp_channel *chan,
		struct isp_capture_req *req)
{
	struct isp_capture *capture = chan->capture_data;
	struct CAPTURE_MSG capture_msg;
	struct capture_common_unpins *unpins;
	struct capture_common_pin_req cap_common_req;
	struct capture_common_relocs *relocs;
	uint32_t __user *reloc_relatives;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->isp_dev,
			"%s: isp capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_ISP_INVALID_ID) {
		dev_err(chan->isp_dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	memset(&capture_msg, 0, sizeof(capture_msg));
	capture_msg.header.msg_id = CAPTURE_ISP_REQUEST_REQ;
	capture_msg.header.channel_id = capture->channel_id;
	capture_msg.capture_isp_request_req.buffer_index = req->buffer_index;

	unpins = devm_kzalloc(chan->isp_dev,
				sizeof(struct capture_common_unpins) +
					(sizeof(struct capture_common_buf) *
					req->isp_relocs.num_relocs),
				GFP_KERNEL);
	if (unpins == NULL)
		return -ENOMEM;

	relocs = devm_kzalloc(chan->isp_dev,
			sizeof(struct capture_common_relocs) +
				(sizeof(uint32_t) * req->isp_relocs.num_relocs),
				GFP_KERNEL);
	if (unlikely(relocs == NULL)) {
		dev_err(chan->isp_dev, "failed to allocate relocs\n");
		goto fail;
	}

	relocs->num_relocs = req->isp_relocs.num_relocs;

	reloc_relatives = (uint32_t __user *)
				(uintptr_t)req->isp_relocs.reloc_relatives;

	err = copy_from_user(relocs->reloc_relatives, reloc_relatives,
			relocs->num_relocs * sizeof(uint32_t)) ? -EFAULT : 0;
	if (err < 0) {
		dev_err(chan->isp_dev, "failed to copy request user relocs\n");
		goto reloc_fail;
	}

	/* pin and reloc */
	cap_common_req.dev = chan->isp_dev;
	cap_common_req.rtcpu_dev = capture->rtcpu_dev;
	cap_common_req.unpins = unpins;
	cap_common_req.requests = &capture->capture_desc_ctx.requests;
	cap_common_req.requests_dev = &capture->capture_desc_ctx.requests_isp;
	cap_common_req.request_size = capture->capture_desc_ctx.request_size;
	cap_common_req.request_offset = req->buffer_index *
					capture->capture_desc_ctx.request_size;
	cap_common_req.relocs = relocs;
	cap_common_req.requests_mem = capture->capture_desc_ctx.desc_mem;

	capture_common_request_pin_and_reloc(&cap_common_req);

	/* add pinned memory ctx to unpins_list */
	mutex_lock(&capture->capture_desc_ctx.unpins_list_lock);
	capture->capture_desc_ctx.unpins_list[req->buffer_index] = unpins;
	mutex_unlock(&capture->capture_desc_ctx.unpins_list_lock);

	dev_dbg(chan->isp_dev, "%s: sending chan_id %u msg_id %u buf:%u\n",
			__func__, capture_msg.header.channel_id,
			capture_msg.header.msg_id, req->buffer_index);

	err = tegra_capture_ivc_capture_submit(&capture_msg,
			sizeof(capture_msg));
	if (err < 0) {
		dev_err(chan->isp_dev, "IVC capture submit failed\n");
		goto reloc_fail;
	}

	return 0;

reloc_fail:
	devm_kfree(chan->isp_dev, relocs);
fail:
	isp_capture_request_unpin(chan, req->buffer_index);
	return err;
}

int isp_capture_status(struct tegra_isp_channel *chan,
		int32_t timeout_ms)
{
	struct isp_capture *capture = chan->capture_data;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->isp_dev,
			 "%s: isp capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_ISP_INVALID_ID) {
		dev_err(chan->isp_dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	err = wait_for_completion_killable_timeout(
			&capture->capture_resp,
			(unsigned long)(timeout_ms/1000*HZ));
	if (err <= 0) {
		dev_err(chan->isp_dev, "no reply from camera processor\n");
		return -ETIMEDOUT;
	}

	return err;
}
