/*
 * Copyright (c) 2016 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <linux/trusty/trusty.h>
#include <linux/trusty/trusty_ipc.h>

#include "trusty-ote.h"

/* Time out in milli seconds */
#define REPLY_TIMEOUT	5000
#define TXBUF_TIMEOUT	15000

//#define DEBUG

#ifdef DEBUG
#define trusty_ote_debug(...)	\
		pr_err("trusty-ote: " __VA_ARGS__)
#else
#define trusty_ote_debug(...)		\
({						\
	if (0)					\
		pr_info("trusty-ote: " __VA_ARGS__); \
	0;					\
})
#endif

/*
 * Currently assumed that only one buffer is sent using these APIs
 * per operation apart from header
 */
#define MAX_TIPC_MSG_NUM (1 + 1)

struct tipc_chan_ctx {
	struct tipc_chan *chan;
	struct completion reply_comp;
	int state;
	/*
	 * Stores the data that is sent/received over channel
	 */
	void *data[MAX_TIPC_MSG_NUM];
	size_t len[MAX_TIPC_MSG_NUM];
	int cur_msg;
	int total_msg;
};

enum tipc_chan_state {
	TIPC_DISCONNECTED = 0,
	TIPC_CONNECTED
};

static int wait_for_response(struct  tipc_chan_ctx *chan_ctx, int timeout)
{
	int ret;

	ret = wait_for_completion_interruptible_timeout(&chan_ctx->reply_comp,
					msecs_to_jiffies(timeout));
	if (ret)
		/* Received reply */
		ret = 0;
	else
		/* No reply from remote */
		ret = -ETIMEDOUT;

	return ret;
}

static struct tipc_msg_buf *_handle_msg(void *data, struct tipc_msg_buf *rxbuf)
{
	struct tipc_chan_ctx *chan_ctx = (struct tipc_chan_ctx *)data;

	trusty_ote_debug("%s\n", __func__);
	if (mb_avail_data(rxbuf) != chan_ctx->len[chan_ctx->cur_msg]) {
		pr_err("%s:ERROR: expected msg len %zu: actual %zu",
				__func__, chan_ctx->len[chan_ctx->cur_msg],
				mb_avail_data(rxbuf));
		return rxbuf;
	}

	/* copy data received over channel */
	memcpy(chan_ctx->data[chan_ctx->cur_msg],
			mb_get_data(rxbuf, chan_ctx->len[chan_ctx->cur_msg]),
			chan_ctx->len[chan_ctx->cur_msg]);

	chan_ctx->cur_msg++;
	/* wake up client if all messages are received */
	if (chan_ctx->cur_msg == chan_ctx->total_msg)
		complete(&chan_ctx->reply_comp);

	return rxbuf;
}

void handle_connect_event(struct tipc_chan_ctx *chan_ctx)
{
	chan_ctx->state = TIPC_CONNECTED;
	complete(&chan_ctx->reply_comp);
}

static void _handle_event(void *data, int event)
{
	struct tipc_chan_ctx *chan_ctx = data;

	trusty_ote_debug("%s: event %d state %d\n", __func__,
			event, chan_ctx->state);
	switch (event) {
	case TIPC_CHANNEL_SHUTDOWN:
		pr_err("%s: channel(state:%d) shutting down\n",
				__func__, chan_ctx->state);
		chan_ctx->state = TIPC_CHANNEL_DISCONNECTED;
		/* wake up the pending client */
		complete(&chan_ctx->reply_comp);
		break;

	case TIPC_CHANNEL_DISCONNECTED:
		pr_err("%s: channel(state:%d) disconnected\n",
				__func__, chan_ctx->state);
		chan_ctx->state = TIPC_CHANNEL_DISCONNECTED;
		/* wake up the pending client */
		complete(&chan_ctx->reply_comp);
		break;

	case TIPC_CHANNEL_CONNECTED:
		handle_connect_event(chan_ctx);
		break;

	default:
		pr_err("%s: unhandled event %d\n", __func__, event);
		break;
	}
	return;
}

struct tipc_chan_ops chan_ops = {
	.handle_msg = _handle_msg,
	.handle_event = _handle_event,
};

/*
 * Embeds the data in TIPC buffer and queue the message
 * to send to secure world.
 */
static int queue_msg(struct tipc_chan_ctx *chan_ctx, void *data, size_t len)
{
	int ret;
	struct tipc_msg_buf *txbuf = NULL;

	trusty_ote_debug("%s: data %p len %zu\n", __func__, data, len);

	txbuf = tipc_chan_get_txbuf_timeout(chan_ctx->chan, TXBUF_TIMEOUT);
	if (IS_ERR(txbuf)) {
		pr_err("%s:error(%ld) in get txbuf\n", __func__, PTR_ERR(txbuf));
		return PTR_ERR(txbuf);
	}

	/* check available space */
	if (len > mb_avail_space(txbuf)) {
		ret = -EMSGSIZE;
		goto err;
	}

	/* copy in message data */
	memcpy(mb_put_data(txbuf, len), data, len);

	/* queue message */
	ret = tipc_chan_queue_msg(chan_ctx->chan, txbuf);
	if (ret)
		goto err;

	return ret;
err:
	tipc_chan_put_txbuf(chan_ctx->chan, txbuf);
	return ret;
}

/*
 * Constructs OTE message and sent it to TA over TIPC channel.
 * And then wait till response is received from TA.
 */
static int handle_ote_msg(struct tipc_chan_ctx *chan_ctx, void *buf,
		size_t len, uint32_t cmd, te_error_t *op_status)
{
	int ret;
	te_operation_container_t op_ctr;
	te_oper_param_t *param;

	trusty_ote_debug("%s: buf %p len %zu\n", __func__, buf, len);

	memset(&op_ctr, 0, sizeof(op_ctr));
	op_ctr.command = cmd;
	if (buf) {
		op_ctr.list_count = 1;
		param = op_ctr.params;
		param->index = 0;
		param->type = TE_PARAM_TYPE_MEM_RW;
		param->u.Mem.base = (uint64_t)(uintptr_t)buf;
		param->u.Mem.len = len;
	}

	chan_ctx->cur_msg = 0;
	chan_ctx->total_msg = 1;
	chan_ctx->data[0] = &op_ctr;
	chan_ctx->len[0] = sizeof(op_ctr);
	if(buf) {
		chan_ctx->data[1] = buf;
		chan_ctx->len[1] = len;
		chan_ctx->total_msg++;
	}

	/* queue ote header */
	trusty_ote_debug("%s: queue ote header\n", __func__);
	ret = queue_msg(chan_ctx, &op_ctr, sizeof(op_ctr));
	if (ret) {
		pr_err("%s:error(%d) in queue header\n", __func__, ret);
		return ret;
	}

	/* queue buffer if present */
	if (buf) {
		trusty_ote_debug("%s: queue ote buffer\n", __func__);
		ret = queue_msg(chan_ctx, buf, len);
		if (ret) {
			pr_err("%s:error(%d) in queue header\n", __func__, ret);
			return ret;
		}
	}

	trusty_ote_debug("%s: waiting for response\n", __func__);
	ret = wait_for_response(chan_ctx, REPLY_TIMEOUT);
	if (ret < 0) {
		pr_err("%s:ERROR(%d) in receving header\n", __func__, ret);
		return ret;
	}

	/* sanity check */
	WARN_ON(chan_ctx->state != TIPC_CONNECTED);

	*op_status = op_ctr.status;
	trusty_ote_debug("%s: op status 0x%08x\n", __func__, *op_status);

	return ret;
}

/*
 * te_open_trusted_session - Establishes the session with TA
 * @name(in): name of the TA to connect to.
 * @ctx(out): pointer to the private data associated to the open session
 * Returns 0 on Success else error code.
 */
int te_open_trusted_session(char *name, void **ctx)
{
	int ret;
	te_error_t op_status = OTE_ERROR_GENERIC;
	struct tipc_chan_ctx *chan_ctx;

	trusty_ote_debug("%s: service %s\n", __func__, name);

	if (!ctx || !name)
		return -EINVAL;

	chan_ctx = kzalloc(sizeof(struct tipc_chan_ctx), GFP_KERNEL);
	if (!chan_ctx) {
		ret = -ENOMEM;
		return ret;
	}

	chan_ctx->chan = tipc_create_channel(NULL, &chan_ops, chan_ctx);
	if(IS_ERR(chan_ctx->chan)) {
		ret = PTR_ERR(chan_ctx->chan);
		pr_err("%s:ERROR(%d) in tipc_create_channel\n", __func__, ret);
		goto err_chan;
	}

	init_completion(&chan_ctx->reply_comp);
	chan_ctx->state = TIPC_DISCONNECTED;

	ret = tipc_chan_connect(chan_ctx->chan, name);
	if (ret) {
		pr_err("%s:ERROR(%d) in tipc_chan_connect\n", __func__, ret);
		goto err_conn;
	}

	ret = wait_for_response(chan_ctx, REPLY_TIMEOUT);
	if (ret < 0) {
		pr_err("%s:ERROR(%d) in receving response from service\n",
								__func__, ret);
		goto err_conn;
	}

	if (chan_ctx->state != TIPC_CONNECTED) {
		pr_err("%s:Invalid channel state %d\n",
				__func__, chan_ctx->state);
		ret = -ENOTCONN;
		goto err_conn;
	}

	ret = handle_ote_msg(chan_ctx, NULL, 0, 0, &op_status);
	if (ret) {
		pr_err("%s:ERROR(%d) in handle_ote_msg\n", __func__, ret);
		goto err;
	}

	if (op_status) {
		pr_err("%s: ERROR in operation 0x%08x", __func__, op_status);
		ret = -EINVAL;
		goto err;
	}
	*ctx = chan_ctx;
	return 0;

err:
	tipc_chan_shutdown(chan_ctx->chan);
err_conn:
	tipc_chan_destroy(chan_ctx->chan);
err_chan:
	kfree(chan_ctx);
	return ret;
}
EXPORT_SYMBOL(te_open_trusted_session);

/*
 * te_close_trusted_session - Closes the session established
 * @ctx: ctx returned by open session
 */
void te_close_trusted_session(void *ctx)
{
	struct tipc_chan_ctx *chan_ctx;

	trusty_ote_debug("%s \n", __func__);
	if (!ctx)
		return;

	chan_ctx = (struct tipc_chan_ctx *)ctx;

	/* wake up the pending client */
	complete(&chan_ctx->reply_comp);

	if (chan_ctx->state == TIPC_CHANNEL_CONNECTED)
		tipc_chan_shutdown(chan_ctx->chan);
	tipc_chan_destroy(chan_ctx->chan);

	kfree(chan_ctx);
}
EXPORT_SYMBOL(te_close_trusted_session);

/*
 * te_launch_trusted_oper - Communicate with TA to perform any operation
 * @buf: Buffer to sent to secure world.
 * @buf_len: length of the buffer.
 * @ta_cmd: command to sent to secure world.
 * @ctx: ctx returned by open session.
 * Returns 0 on Success else error code.
 */
int te_launch_trusted_oper(void *buf, size_t buf_len, uint32_t ta_cmd,
		void *ctx)
{
	int ret;
	te_error_t op_status = OTE_ERROR_GENERIC;
	struct tipc_chan_ctx *chan_ctx;

	trusty_ote_debug("%s: cmd %u\n", __func__, ta_cmd);

	if (!ctx || !buf || !buf_len)
		return -EINVAL;

	chan_ctx = (struct tipc_chan_ctx *)ctx;

	if (chan_ctx->state != TIPC_CONNECTED) {
		pr_err("%s:Invalid channel state %d\n",
				__func__, chan_ctx->state);
		return -ENOTCONN;
	}

	ret = handle_ote_msg(chan_ctx, buf, buf_len, ta_cmd, &op_status);
	if (ret) {
		pr_err("%s:ERROR(%d) in handle_ote_msg\n", __func__, ret);
		return ret;
	}

	if (op_status) {
		pr_err("%s: ERROR in operation 0x%08x", __func__, op_status);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(te_launch_trusted_oper);
