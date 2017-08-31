/*
 * Tegra Video Input capture operations
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: David Wang <davidw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/uaccess.h>
#include <linux/completion.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/nvhost.h>
#include <linux/of_platform.h>
#include <linux/printk.h>
#include <linux/scatterlist.h>
#include <linux/tegra-capture-ivc.h>
#include <media/capture.h>
#include <media/capture_vi_channel.h>

#include "soc/tegra/camrtc-capture.h"
#include "soc/tegra/camrtc-capture-messages.h"

/* Remove after constants integrated into camrtc-capture-messages.h */
#ifndef CAPTURE_CHANNEL_TPG_SETUP_REQ
#define CAPTURE_CHANNEL_TPG_SETUP_REQ   U32_C(0x30)
#define CAPTURE_CHANNEL_TPG_SETUP_RESP  U32_C(0x31)
#define CAPTURE_CHANNEL_TPG_START_REQ   U32_C(0x32)
#define CAPTURE_CHANNEL_TPG_START_RESP  U32_C(0x33)
#define CAPTURE_CHANNEL_TPG_STOP_REQ    U32_C(0x34)
#define CAPTURE_CHANNEL_TPG_STOP_RESP   U32_C(0x35)
#endif

#define CAPTURE_CHANNEL_UNKNOWN_RESP 0xFFFFFFFF
#define CAPTURE_CHANNEL_INVALID_ID 0xFFFF
#define CAPTURE_CHANNEL_INVALID_MASK 0llu
#define PROGRESS_SP_IDX 0
#define EMBDATA_SP_IDX 1
#define LINETIMER_SP_IDX 2
#define CAPTURE_CHANNEL_MAX_NUM_SPS 3

struct vi_capture_buf {
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t iova;
};

struct vi_capture_unpins {
	uint32_t num_unpins;
	struct vi_capture_buf data[];
};

struct vi_capture {
	uint16_t channel_id;
	struct device *rtcpu_dev;
	struct tegra_vi_channel *vi_channel;
	struct vi_capture_buf requests;
	size_t request_buf_size;
	uint32_t queue_depth;
	uint32_t request_size;

	uint32_t syncpts[CAPTURE_CHANNEL_MAX_NUM_SPS];

	struct completion control_resp;
	struct completion capture_resp;
	struct mutex control_msg_lock;
	struct CAPTURE_CONTROL_MSG control_resp_msg;

	struct mutex unpins_list_lock;
	struct vi_capture_unpins **unpins_list;
};

static void vi_capture_ivc_control_callback(const void *ivc_resp,
		const void *pcontext)
{
	const struct CAPTURE_CONTROL_MSG *control_msg = ivc_resp;
	struct vi_capture *capture = (struct vi_capture *)pcontext;
	struct tegra_vi_channel *chan = capture->vi_channel;

	if (unlikely(capture == NULL)) {
		dev_err(chan->dev, "%s: invalid context", __func__);
		return;
	}

	if (unlikely(control_msg == NULL)) {
		dev_err(chan->dev, "%s: invalid response", __func__);
		return;
	}

	switch (control_msg->header.msg_id) {
	case CAPTURE_CHANNEL_SETUP_RESP:
	case CAPTURE_CHANNEL_RESET_RESP:
	case CAPTURE_CHANNEL_RELEASE_RESP:
	case CAPTURE_COMPAND_CONFIG_RESP:
	case CAPTURE_PDAF_CONFIG_RESP:
	case CAPTURE_SYNCGEN_ENABLE_RESP:
	case CAPTURE_SYNCGEN_DISABLE_RESP:
	case CAPTURE_CHANNEL_TPG_SETUP_RESP:
	case CAPTURE_CHANNEL_TPG_START_RESP:
	case CAPTURE_CHANNEL_TPG_STOP_RESP:
		memcpy(&capture->control_resp_msg, control_msg,
				sizeof(*control_msg));
		complete(&capture->control_resp);
		break;
	default:
		dev_err(chan->dev,
			"%s: unknown capture control resp 0x%x", __func__,
			control_msg->header.msg_id);
		break;
	}
}

static void vi_capture_request_unpin(struct tegra_vi_channel *chan,
		uint32_t buffer_index);
static void vi_capture_ivc_status_callback(const void *ivc_resp,
		const void *pcontext)
{
	struct CAPTURE_MSG *status_msg = (struct CAPTURE_MSG *)ivc_resp;
	struct vi_capture *capture = (struct vi_capture *)pcontext;
	struct tegra_vi_channel *chan = capture->vi_channel;
	uint32_t buffer_index;

	if (unlikely(capture == NULL)) {
		dev_err(chan->dev, "%s: invalid context", __func__);
		return;
	}

	if (unlikely(status_msg == NULL)) {
		dev_err(chan->dev, "%s: invalid response", __func__);
		return;
	}

	switch (status_msg->header.msg_id) {
	case CAPTURE_STATUS_IND:
		buffer_index = status_msg->capture_status_ind.buffer_index;
		vi_capture_request_unpin(chan, buffer_index);
		dma_sync_single_range_for_cpu(capture->rtcpu_dev,
		    capture->requests.iova,
		    buffer_index * capture->request_size,
		    capture->request_size, DMA_FROM_DEVICE);
		complete(&capture->capture_resp);
		dev_dbg(chan->dev, "%s: status chan_id %u msg_id %u\n",
				__func__, status_msg->header.channel_id,
				status_msg->header.msg_id);
		break;
	default:
		dev_err(chan->dev,
			"%s: unknown capture resp", __func__);
		break;
	}
}

int vi_capture_init(struct tegra_vi_channel *chan)
{
	struct vi_capture *capture;
	struct device_node *dn;
	struct platform_device *rtc_pdev;

	dev_dbg(chan->dev, "%s++\n", __func__);
	dn = of_find_node_by_path("tegra-camera-rtcpu");
	if (of_device_is_available(dn) == 0) {
		dev_err(chan->dev, "failed to find rtcpu device node\n");
		return -ENODEV;
	}
	rtc_pdev = of_find_device_by_node(dn);
	if (rtc_pdev == NULL) {
		dev_err(chan->dev, "failed to find rtcpu platform\n");
		return -ENODEV;
	}

	capture = devm_kzalloc(chan->dev,
			sizeof(*capture), GFP_KERNEL);
	if (unlikely(capture == NULL)) {
		dev_err(chan->dev, "failed to allocate capture channel\n");
		return -ENOMEM;
	}

	capture->rtcpu_dev = &rtc_pdev->dev;

	init_completion(&capture->control_resp);
	init_completion(&capture->capture_resp);

	mutex_init(&capture->control_msg_lock);
	mutex_init(&capture->unpins_list_lock);

	capture->vi_channel = chan;
	chan->capture_data = capture;

	capture->channel_id = CAPTURE_CHANNEL_INVALID_ID;

	return 0;
}

void vi_capture_shutdown(struct tegra_vi_channel *chan)
{
	struct vi_capture *capture = chan->capture_data;

	dev_dbg(chan->dev, "%s--\n", __func__);
	if (capture == NULL)
		return;

	if (capture->channel_id != CAPTURE_CHANNEL_INVALID_ID)
		vi_capture_release(chan, 0);

	devm_kfree(chan->dev, capture);
	chan->capture_data = NULL;
}

static int vi_capture_ivc_send_control(struct tegra_vi_channel *chan,
		const struct CAPTURE_CONTROL_MSG *msg, size_t size,
		uint32_t resp_id)
{
	struct vi_capture *capture = chan->capture_data;
	struct CAPTURE_MSG_HEADER resp_header = msg->header;
	uint32_t timeout = HZ;
	int err = 0;

	dev_dbg(chan->dev, "%s: sending chan_id %u msg_id %u\n",
			__func__, resp_header.channel_id, resp_header.msg_id);
	resp_header.msg_id = resp_id;
	/* Send capture control IVC message */
	mutex_lock(&capture->control_msg_lock);
	err = tegra_capture_ivc_control_submit(msg, size);
	if (err < 0) {
		dev_err(chan->dev, "IVC control submit failed\n");
		goto fail;
	}

	timeout = wait_for_completion_killable_timeout(
			&capture->control_resp, timeout);
	if (timeout <= 0) {
		dev_err(chan->dev,
			"no reply from camera processor\n");
		err = -ETIMEDOUT;
		goto fail;
	}

	if (memcmp(&resp_header, &capture->control_resp_msg.header,
			sizeof(resp_header)) != 0) {
		dev_err(chan->dev,
			"unexpected response from camera processor\n");
		err = -EINVAL;
		goto fail;
	}

	mutex_unlock(&capture->control_msg_lock);
	dev_dbg(chan->dev, "%s: response chan_id %u msg_id %u\n",
			__func__, capture->control_resp_msg.header.channel_id,
			capture->control_resp_msg.header.msg_id);
	return 0;

fail:
	mutex_unlock(&capture->control_msg_lock);
	return err;
}

static int pin_memory(struct device *dev,
		uint32_t mem, struct vi_capture_buf *unpin_data);
static void unpin_memory(struct vi_capture_buf *unpin_data);

static int pin_memory(struct device *dev,
		uint32_t mem, struct vi_capture_buf *unpin_data)
{
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	int err = 0;

	buf = dma_buf_get(mem);
	if (IS_ERR(buf)) {
		err = PTR_ERR(buf);
		goto fail;
	}

	attach = dma_buf_attach(buf, dev);
	if (IS_ERR(attach)) {
		err = PTR_ERR(attach);
		goto fail;
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		err = PTR_ERR(sgt);
		goto fail;
	}

	if (sg_dma_address(sgt->sgl) == 0)
		sg_dma_address(sgt->sgl) = sg_phys(sgt->sgl);

	unpin_data->iova = sg_dma_address(sgt->sgl);
	unpin_data->buf = buf;
	unpin_data->attach = attach;
	unpin_data->sgt = sgt;

	return 0;

fail:
	unpin_memory(unpin_data);
	return err;
}

static void unpin_memory(struct vi_capture_buf *unpin_data)
{
	if (unpin_data->sgt != NULL)
		dma_buf_unmap_attachment(unpin_data->attach, unpin_data->sgt,
				DMA_BIDIRECTIONAL);
	if (unpin_data->attach != NULL)
		dma_buf_detach(unpin_data->buf, unpin_data->attach);
	if (unpin_data->buf != NULL)
		dma_buf_put(unpin_data->buf);

	unpin_data->sgt = NULL;
	unpin_data->attach = NULL;
	unpin_data->buf = NULL;
	unpin_data->iova = 0;
}

static int vi_capture_setup_syncpts(struct tegra_vi_channel *chan);
static void vi_capture_release_syncpts(struct tegra_vi_channel *chan);

static int vi_capture_setup_syncpts(struct tegra_vi_channel *chan)
{
	struct vi_capture *capture = chan->capture_data;
	int i;
	int err = 0;

	for (i = 0; i < CAPTURE_CHANNEL_MAX_NUM_SPS; i++) {
			capture->syncpts[i] = nvhost_get_syncpt_client_managed(
			chan->ndev, "vi-capture");
		if (capture->syncpts[i] == 0) {
			dev_err(chan->dev, "failed to get syncpt %i!\n", i);
			err = -ENODEV;
			goto fail;
		}
	}

	return 0;

fail:
	vi_capture_release_syncpts(chan);
	return err;
}

static void vi_capture_release_syncpts(struct tegra_vi_channel *chan)
{
	struct vi_capture *capture = chan->capture_data;
	int i;

	for (i = 0; i < CAPTURE_CHANNEL_MAX_NUM_SPS; i++) {
		if (capture->syncpts[i] != 0)
			nvhost_syncpt_put_ref_ext(chan->ndev,
					capture->syncpts[i]);
		capture->syncpts[i] = 0;
	}
}

int vi_capture_setup(struct tegra_vi_channel *chan,
		struct vi_capture_setup *setup)
{
	struct vi_capture *capture = chan->capture_data;
	uint32_t transaction;
	struct CAPTURE_CONTROL_MSG control_desc;
	struct CAPTURE_CONTROL_MSG *resp_msg = &capture->control_resp_msg;
	struct capture_channel_config *config =
		&control_desc.channel_setup_req.channel_config;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->dev,
			 "%s: vi capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id != CAPTURE_CHANNEL_INVALID_ID) {
		dev_err(chan->dev,
			"%s: already setup, release first\n", __func__);
		return -EEXIST;
	}

	dev_dbg(chan->dev, "chan flags %u\n", setup->channel_flags);
	dev_dbg(chan->dev, "chan mask %llx\n", setup->vi_channel_mask);
	dev_dbg(chan->dev, "queue depth %u\n", setup->queue_depth);
	dev_dbg(chan->dev, "request size %u\n", setup->request_size);

	if (setup->vi_channel_mask == CAPTURE_CHANNEL_INVALID_MASK ||
			setup->channel_flags == 0 ||
			setup->queue_depth == 0 ||
			setup->request_size == 0)
		return -EINVAL;

	/* pin the capture descriptor ring buffer */
	dev_dbg(chan->dev, "%s: descr buffer handle %u\n",
			__func__, setup->mem);
	err = pin_memory(capture->rtcpu_dev, setup->mem, &capture->requests);
	if (err < 0) {
		dev_err(chan->dev, "%s: memory setup failed\n", __func__);
		return -EFAULT;
	}
	capture->queue_depth = setup->queue_depth;
	capture->request_size = setup->request_size;
	capture->request_buf_size = setup->request_size * setup->queue_depth;

	/* allocate for unpin list based on queue depth */
	capture->unpins_list = devm_kzalloc(chan->dev,
			sizeof(struct vi_capture_unpins *) * capture->queue_depth,
			GFP_KERNEL);
	if (unlikely(capture->unpins_list == NULL)) {
		dev_err(chan->dev, "failed to allocate unpins array\n");
		goto unpins_list_fail;
	}

	err = vi_capture_setup_syncpts(chan);
	if (err < 0) {
		dev_err(chan->dev, "%s: syncpt setup failed\n", __func__);
		goto syncpt_fail;
	}

	err = tegra_capture_ivc_register_control_cb(
			&vi_capture_ivc_control_callback,
			&transaction, capture);
	if (err < 0) {
		dev_err(chan->dev, "failed to register control callback\n");
		goto control_cb_fail;
	}

	memset(&control_desc, 0, sizeof(control_desc));
	control_desc.header.msg_id = CAPTURE_CHANNEL_SETUP_REQ;
	control_desc.header.transaction = transaction;

	config->channel_flags = setup->channel_flags;
	config->vi_channel_mask = setup->vi_channel_mask;

	config->queue_depth = setup->queue_depth;
	config->request_size = setup->request_size;
	config->requests = capture->requests.iova;

	config->progress_sp.id = capture->syncpts[PROGRESS_SP_IDX];
	config->embdata_sp.id = capture->syncpts[EMBDATA_SP_IDX];
	config->linetimer_sp.id = capture->syncpts[LINETIMER_SP_IDX];

	err = vi_capture_ivc_send_control(chan, &control_desc,
			sizeof(control_desc), CAPTURE_CHANNEL_SETUP_RESP);
	if (err < 0)
		goto submit_fail;

	if (resp_msg->channel_setup_resp.result != CAPTURE_OK) {
		dev_err(chan->dev, "%s: control failed, errno %d", __func__,
			resp_msg->channel_setup_resp.result);
		err = -EINVAL;
		goto resp_fail;
	}

	capture->channel_id = resp_msg->channel_setup_resp.channel_id;

	err = tegra_capture_ivc_notify_chan_id(capture->channel_id,
			transaction);
	if (err < 0) {
		dev_err(chan->dev, "failed to update control callback\n");
		goto cb_fail;
	}

	err = tegra_capture_ivc_register_capture_cb(
			&vi_capture_ivc_status_callback,
			capture->channel_id, capture);
	if (err < 0) {
		dev_err(chan->dev, "failed to register capture callback\n");
		goto cb_fail;
	}

	return 0;

cb_fail:
	vi_capture_release(chan, CAPTURE_CHANNEL_RESET_FLAG_IMMEDIATE);
resp_fail:
submit_fail:
	tegra_capture_ivc_unregister_control_cb(transaction);
control_cb_fail:
	vi_capture_release_syncpts(chan);
syncpt_fail:
	devm_kfree(chan->dev, capture->unpins_list);
unpins_list_fail:
	unpin_memory(&capture->requests);
	return err;
}

int vi_capture_reset(struct tegra_vi_channel *chan,
		uint32_t reset_flags)
{
	struct vi_capture *capture = chan->capture_data;
	struct CAPTURE_CONTROL_MSG control_desc;
	struct CAPTURE_CONTROL_MSG *resp_msg = &capture->control_resp_msg;
	int i;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->dev,
			 "%s: vi capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_INVALID_ID) {
		dev_err(chan->dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	memset(&control_desc, 0, sizeof(control_desc));
	control_desc.header.msg_id = CAPTURE_CHANNEL_RESET_REQ;
	control_desc.header.channel_id = capture->channel_id;
	control_desc.channel_reset_req.reset_flags = reset_flags;

	err = vi_capture_ivc_send_control(chan, &control_desc,
			sizeof(control_desc), CAPTURE_CHANNEL_RESET_RESP);
	if (err < 0)
		goto submit_fail;

	if (resp_msg->channel_reset_resp.result != CAPTURE_OK) {
		dev_err(chan->dev, "%s: control failed, errno %d", __func__,
			resp_msg->channel_reset_resp.result);
		err = -EINVAL;
	}

	for (i = 0; i < capture->queue_depth; i++)
		vi_capture_request_unpin(chan, i);

	return 0;

submit_fail:
	return err;
}

int vi_capture_release(struct tegra_vi_channel *chan,
		uint32_t reset_flags)
{
	struct vi_capture *capture = chan->capture_data;
	struct CAPTURE_CONTROL_MSG control_desc;
	struct CAPTURE_CONTROL_MSG *resp_msg = &capture->control_resp_msg;
	int i;
	int err = 0;
	int ret = 0;

	if (capture == NULL) {
		dev_err(chan->dev,
			 "%s: vi capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_INVALID_ID) {
		dev_err(chan->dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	memset(&control_desc, 0, sizeof(control_desc));
	control_desc.header.msg_id = CAPTURE_CHANNEL_RELEASE_REQ;
	control_desc.header.channel_id = capture->channel_id;
	control_desc.channel_release_req.reset_flags = reset_flags;

	err = vi_capture_ivc_send_control(chan, &control_desc,
			sizeof(control_desc), CAPTURE_CHANNEL_RELEASE_RESP);
	if (err < 0)
		goto submit_fail;

	if (resp_msg->channel_release_resp.result != CAPTURE_OK) {
		dev_err(chan->dev, "%s: control failed, errno %d", __func__,
			resp_msg->channel_release_resp.result);
		err = -EINVAL;
	}

	ret = tegra_capture_ivc_unregister_capture_cb(capture->channel_id);
	if (ret < 0 && err == 0) {
		dev_err(chan->dev,
			"failed to unregister capture callback\n");
		err = ret;
	}

	ret = tegra_capture_ivc_unregister_control_cb(capture->channel_id);
	if (ret < 0 && err == 0) {
		dev_err(chan->dev,
			"failed to unregister control callback\n");
		err = ret;
	}

	for (i = 0; i < capture->queue_depth; i++)
		vi_capture_request_unpin(chan, i);

	vi_capture_release_syncpts(chan);
	unpin_memory(&capture->requests);

	capture->channel_id = CAPTURE_CHANNEL_INVALID_ID;

	return 0;

submit_fail:
	return err;
}

static int vi_capture_read_syncpt(struct tegra_vi_channel *chan,
		uint32_t index, uint32_t *val)
{
	struct vi_capture *capture = chan->capture_data;
	int err;

	err = nvhost_syncpt_read_ext_check(chan->ndev,
			capture->syncpts[index], val);
	if (err < 0) {
		dev_err(chan->dev,
			 "%s: get progress syncpt %i val failed\n", __func__,
			 index);
		return -EINVAL;
	}

	return 0;
}

int vi_capture_get_info(struct tegra_vi_channel *chan,
		struct vi_capture_info *info)
{
	struct vi_capture *capture = chan->capture_data;
	int err;

	if (capture == NULL) {
		dev_err(chan->dev,
			 "%s: vi capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_INVALID_ID) {
		dev_err(chan->dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	if (info == NULL)
		return -EINVAL;

	info->syncpts.progress_syncpt = capture->syncpts[PROGRESS_SP_IDX];
	info->syncpts.emb_data_syncpt = capture->syncpts[EMBDATA_SP_IDX];
	info->syncpts.line_timer_syncpt = capture->syncpts[LINETIMER_SP_IDX];

	err = vi_capture_read_syncpt(chan, PROGRESS_SP_IDX,
			&info->syncpts.progress_syncpt_val);
	if (err < 0)
		return err;
	err = vi_capture_read_syncpt(chan, EMBDATA_SP_IDX,
			&info->syncpts.emb_data_syncpt_val);
	if (err < 0)
		return err;
	err = vi_capture_read_syncpt(chan, LINETIMER_SP_IDX,
			&info->syncpts.line_timer_syncpt_val);
	if (err < 0)
		return err;

	return 0;
}

int vi_capture_control_message(struct tegra_vi_channel *chan,
		struct vi_capture_control_msg *msg)
{
	struct vi_capture *capture = chan->capture_data;
	const void __user *msg_ptr =
			(const void __user *)(uintptr_t)msg->ptr;
	void __user *response =
			(void __user *)(uintptr_t)msg->response;
	void *msg_cpy;
	struct CAPTURE_MSG_HEADER *header;
	uint32_t resp_id;
	struct CAPTURE_CONTROL_MSG *resp_msg = &capture->control_resp_msg;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->dev,
			 "%s: vi capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (msg->ptr == 0ull || msg->response == 0ull || msg->size == 0)
		return -EINVAL;

	msg_cpy = devm_kzalloc(chan->dev, msg->size, GFP_KERNEL);
	if (unlikely(msg_cpy == NULL))
		return -ENOMEM;

	err = copy_from_user(msg_cpy, msg_ptr, msg->size) ? -EFAULT : 0;
	if (err < 0)
		goto fail;
	header = (struct CAPTURE_MSG_HEADER *)msg_cpy;
	header->channel_id = capture->channel_id;

	switch (header->msg_id) {
	case CAPTURE_COMPAND_CONFIG_REQ:
		resp_id = CAPTURE_COMPAND_CONFIG_RESP;
		break;
	case CAPTURE_PDAF_CONFIG_REQ:
		resp_id = CAPTURE_PDAF_CONFIG_RESP;
		break;
	case CAPTURE_SYNCGEN_ENABLE_REQ:
		resp_id = CAPTURE_SYNCGEN_ENABLE_RESP;
		break;
	case CAPTURE_SYNCGEN_DISABLE_REQ:
		resp_id = CAPTURE_SYNCGEN_DISABLE_RESP;
		break;
	case CAPTURE_CHANNEL_TPG_SETUP_REQ:
		resp_id = CAPTURE_CHANNEL_TPG_SETUP_RESP;
		break;
	case CAPTURE_CHANNEL_TPG_START_REQ:
		resp_id = CAPTURE_CHANNEL_TPG_START_RESP;
		break;
	case CAPTURE_CHANNEL_TPG_STOP_REQ:
		resp_id = CAPTURE_CHANNEL_TPG_STOP_RESP;
		break;
	default:
		dev_err(chan->dev,
				"%s: unknown capture control req %x", __func__,
				header->msg_id);
		err = -EINVAL;
		goto fail;
	}

	err = vi_capture_ivc_send_control(chan, msg_cpy, msg->size, resp_id);
	if (err < 0)
		goto fail;

	err = copy_to_user(response, resp_msg,
			sizeof(*resp_msg)) ? -EFAULT : 0;

fail:
	devm_kfree(chan->dev, msg_cpy);
	return err;
}

struct surface_t {
	uint32_t offset;
	uint32_t offset_hi;
};

static int vi_capture_request_pin_and_reloc(struct tegra_vi_channel *chan,
		struct vi_capture_req *req)
{
	struct vi_capture *capture = chan->capture_data;
	uint32_t num_relocs = req->num_relocs;
	uint32_t __user *reloc_relatives =
			(uint32_t __user *)(uintptr_t)req->reloc_relatives;
	uint32_t local_reloc_relatives[VI_NUM_ATOMP_SURFACES];
	struct vi_capture_unpins *unpins;
	uint32_t request_offset = req->buffer_index * capture->request_size;
	void *reloc_page_addr = NULL;
	uint32_t prev_mem = 0;
	int last_page = -1;
	dma_addr_t surface_phys_addr = 0;
	int i = 0;
	int err = 0;

	err = copy_from_user(local_reloc_relatives, reloc_relatives,
		num_relocs * sizeof(uint32_t)) ? -EFAULT : 0;
	if (err < 0)
		return err;

	unpins = devm_kzalloc(chan->dev,
			sizeof(struct vi_capture_unpins) +
			sizeof(struct vi_capture_buf) * num_relocs,
			GFP_KERNEL);
	if (unpins == NULL)
		return -ENOMEM;

	dev_dbg(chan->dev, "%s: relocating %u surfaces\n",
			__func__, num_relocs);
	for (i = 0; i < num_relocs; i++) {
		uint32_t reloc_offset =
				request_offset + local_reloc_relatives[i];
		uint64_t surface_raw;
		struct surface_t *surface;
		uint32_t mem;
		uint32_t target_offset;
		dma_addr_t target_phys_addr;

		dev_dbg(chan->dev,
			"%s: idx:%i reloc:%u reloc_offset:%u", __func__,
			i, local_reloc_relatives[i], reloc_offset);

		/* locate page of the request descr buffer relocation is on */
		if (last_page != reloc_offset >> PAGE_SHIFT) {
			if (reloc_page_addr != NULL)
				dma_buf_kunmap(capture->requests.buf, last_page,
						reloc_page_addr);

			reloc_page_addr = dma_buf_kmap(capture->requests.buf,
					reloc_offset >> PAGE_SHIFT);
			last_page = reloc_offset >> PAGE_SHIFT;

			if (unlikely(reloc_page_addr == NULL)) {
				dev_err(chan->dev,
					"%s: couldn't map request\n", __func__);
				goto fail;
			}
		}

		/* read surface offset and memory handle from request descr */
		surface_raw = __raw_readq(
				(void __iomem *)(reloc_page_addr +
				(reloc_offset & ~PAGE_MASK)));
		surface = (struct surface_t *)&surface_raw;
		target_offset = surface->offset;
		mem = surface->offset_hi;
		dev_dbg(chan->dev, "%s: hmem:%u offset:%u\n", __func__,
				mem, target_offset);

		if (mem != prev_mem) {
			err = pin_memory(chan->dev,
					 mem, &unpins->data[unpins->num_unpins]);
			if (err < 0) {
				goto fail;
			}
			unpins->num_unpins++;
			surface_phys_addr = unpins->data[i].iova;
		}

		target_phys_addr = surface_phys_addr + target_offset;
		dev_dbg(chan->dev, "%s: surface addr %lx at desc %lx\n",
		    __func__, (unsigned long)target_phys_addr,
		    (unsigned long)reloc_page_addr +
		    (reloc_offset & ~PAGE_MASK));

		/* write relocated physical address to request descr */
		__raw_writeq(
			target_phys_addr,
			(void __iomem *)(reloc_page_addr +
				(reloc_offset & ~PAGE_MASK)));

		dma_sync_single_range_for_device(capture->rtcpu_dev,
		    capture->requests.iova, request_offset,
		    capture->request_size, DMA_TO_DEVICE);
	}

	/* assign the unpins list to the capture to be unpinned and */
	/* freed at capture completion (vi_capture_request_unpin) */
	mutex_lock(&capture->unpins_list_lock);
	capture->unpins_list[req->buffer_index] = unpins;
	mutex_unlock(&capture->unpins_list_lock);

	return 0;

fail:
	if (reloc_page_addr != NULL)
		dma_buf_kunmap(capture->requests.buf, last_page,
			reloc_page_addr);

	for (i = 0; i < unpins->num_unpins; i++)
		unpin_memory(&unpins->data[i]);
	devm_kfree(chan->dev, unpins);

	return err;
}

static void vi_capture_request_unpin(struct tegra_vi_channel *chan,
		uint32_t buffer_index)
{
	struct vi_capture *capture = chan->capture_data;
	struct vi_capture_unpins *unpins;
	int i = 0;

	mutex_lock(&capture->unpins_list_lock);
	unpins = capture->unpins_list[buffer_index];
	if (unpins != NULL) {
		for (i = 0; i < unpins->num_unpins; i++)
			unpin_memory(&unpins->data[i]);
		capture->unpins_list[buffer_index] = NULL;
		devm_kfree(chan->dev, unpins);
	}
	mutex_unlock(&capture->unpins_list_lock);
}

int vi_capture_request(struct tegra_vi_channel *chan,
		struct vi_capture_req *req)
{
	struct vi_capture *capture = chan->capture_data;
	struct CAPTURE_MSG capture_desc;
	int err = 0;

	if (capture == NULL) {
		dev_err(chan->dev,
			"%s: vi capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_INVALID_ID) {
		dev_err(chan->dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	memset(&capture_desc, 0, sizeof(capture_desc));
	capture_desc.header.msg_id = CAPTURE_REQUEST_REQ;
	capture_desc.header.channel_id = capture->channel_id;
	capture_desc.capture_request_req.buffer_index = req->buffer_index;

	/* perform surface pinning and relocation */
	err = vi_capture_request_pin_and_reloc(chan, req);
	if (err < 0) {
		dev_err(chan->dev, "relocation failed\n");
		return err;
	}

	dev_dbg(chan->dev, "%s: sending chan_id %u msg_id %u buf:%u\n",
			__func__, capture_desc.header.channel_id,
			capture_desc.header.msg_id, req->buffer_index);
	err = tegra_capture_ivc_capture_submit(&capture_desc,
			sizeof(capture_desc));
	if (err < 0) {
		dev_err(chan->dev, "IVC capture submit failed\n");
		goto fail;
	}

	return 0;

fail:
	vi_capture_request_unpin(chan, req->buffer_index);
	return err;
}

int vi_capture_status(struct tegra_vi_channel *chan,
		int32_t timeout_ms)
{
	struct vi_capture *capture = chan->capture_data;
	int ret = 0;

	if (capture == NULL) {
		dev_err(chan->dev,
			 "%s: vi capture uninitialized\n", __func__);
		return -ENODEV;
	}

	if (capture->channel_id == CAPTURE_CHANNEL_INVALID_ID) {
		dev_err(chan->dev,
			"%s: setup channel first\n", __func__);
		return -ENODEV;
	}

	dev_dbg(chan->dev, "%s: waiting for status, timeout:%d ms\n",
		__func__, timeout_ms);

	ret = wait_for_completion_killable_timeout(
			&capture->capture_resp,
			(unsigned long)(timeout_ms/1000*HZ));
	/* Possible return values:
	 * -ERESTARTSYS if interrupted, 0 if timed out, positive (at least 1,
	 * or number of jiffies left till timeout) if completed */
	if (ret <= 0) {
		dev_err(chan->dev,
			"no reply from camera processor\n");
		return -ETIMEDOUT;
	} else
		return 0;
}
