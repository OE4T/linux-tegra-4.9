/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/tegra-ivc.h>
#include "bpmp.h"
#include "mail_t186.h"

static struct ivc ivc_channels[NR_CHANNELS];

static bool imo_tx_ready(int ch)
{
	void *frame;
	bool ready;

	frame = tegra_ivc_write_get_next_frame(ivc_channels + ch);
	ready = !IS_ERR_OR_NULL(frame);
	channel_area[ch].ob = ready ? frame : NULL;

	return ready;
}

static bool imo_rx_ready(int ch)
{
	void *frame;
	bool ready;

	frame = tegra_ivc_read_get_next_frame(ivc_channels + ch);
	ready = !IS_ERR_OR_NULL(frame);
	channel_area[ch].ib = ready ? frame : NULL;

	return ready;
}

static void imo_signal_slave(int ch)
{
	if (tegra_ivc_write_advance(ivc_channels + ch)) {
		WARN_ON(1);
		return;
	}

	/* hint to the firmware mail probe */
	bpmp_writel(1 << ch, HSP_SHRD_SEM_1_SET);
}

static void imo_free_master(int ch)
{
	if (tegra_ivc_read_advance(ivc_channels + ch))
		WARN_ON(1);
}

static void imo_return_data(int ch, int code, void *data, int sz)
{
	const int flags = channel_area[ch].ib->flags;
	struct ivc *ivc = ivc_channels + ch;
	struct mb_data *frame;
	int r;

	r = tegra_ivc_read_advance(ivc);
	WARN_ON(r);

	if (!(flags & DO_ACK))
		return;

	frame = tegra_ivc_write_get_next_frame(ivc);
	if (IS_ERR_OR_NULL(frame)) {
		WARN_ON(1);
		return;
	}

	frame->code = code;
	memcpy(frame->data, data, sz);
	r = tegra_ivc_write_advance(ivc);
	WARN_ON(r);

	if (flags & RING_DOORBELL)
		bpmp_ring_doorbell();
}

static void imo_notify(struct ivc *ivc)
{
}

static int imo_channel_init(int ch, uint8_t *obmem, uint8_t *ibmem, size_t sz)
{
	struct ivc *ivc;
	uintptr_t rx_base;
	uintptr_t tx_base;
	size_t msg_sz;
	size_t frame_sz;
	size_t frame_off;
	int r;

	msg_sz = tegra_ivc_align(MSG_SZ);
	frame_sz = tegra_ivc_total_queue_size(msg_sz);
	frame_off = ch * frame_sz;

	if (frame_off + frame_sz > sz) {
		pr_err("ivc frame overflow: ch %d\n", ch);
		WARN_ON(1);
		return -EINVAL;
	}

	rx_base = (uintptr_t)(ibmem + frame_off);
	tx_base = (uintptr_t)(obmem + frame_off);

	/* init the channel frame */
	memset_io((void *)tx_base, 0, frame_sz);

	ivc = ivc_channels + ch;
	r = tegra_ivc_init(ivc, rx_base, tx_base,
			1, msg_sz, device, imo_notify);
	if (r) {
		pr_err("tegra_ivc_init() ch %d returned %d\n", ch, r);
		WARN_ON(1);
		return r;
	}

	return 0;
}

const struct mail_ops mail_ops = {
	.channel_init = imo_channel_init,
	.free_master = imo_free_master,
	.master_acked = imo_rx_ready,
	.master_free = imo_tx_ready,
	.return_data = imo_return_data,
	.signal_slave = imo_signal_slave,
	.slave_signalled = imo_rx_ready
};
