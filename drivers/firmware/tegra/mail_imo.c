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
#include <linux/tegra-ivc.h>
#include <linux/of.h>
#include "bpmp.h"
#include "mail_t186.h"

struct transport_layer_ops trans_ops;

static bool imo_tx_ready(int ch)
{
	struct ivc *ivc;
	void *frame;
	bool ready;

	ivc = trans_ops.channel_to_ivc(ch);
	frame = tegra_ivc_write_get_next_frame(ivc);
	ready = !IS_ERR_OR_NULL(frame);
	channel_area[ch].ob = ready ? frame : NULL;

	return ready;
}

static bool imo_rx_ready(int ch)
{
	struct ivc *ivc;
	void *frame;
	bool ready;

	ivc = trans_ops.channel_to_ivc(ch);
	frame = tegra_ivc_read_get_next_frame(ivc);
	ready = !IS_ERR_OR_NULL(frame);
	channel_area[ch].ib = ready ? frame : NULL;

	return ready;
}

static void imo_signal_slave(int ch)
{
	struct ivc *ivc;

	ivc = trans_ops.channel_to_ivc(ch);
	if (tegra_ivc_write_advance(ivc))
		WARN_ON(1);
}

static void imo_free_master(int ch)
{
	struct ivc *ivc;

	ivc = trans_ops.channel_to_ivc(ch);
	if (tegra_ivc_read_advance(ivc))
		WARN_ON(1);
}

static void imo_return_data(int ch, int code, void *data, int sz)
{
	const int flags = channel_area[ch].ib->flags;
	struct ivc *ivc;
	struct mb_data *frame;
	int r;

	ivc = trans_ops.channel_to_ivc(ch);
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

/* FIXME: consider using an attr */
static const char *ofm_native = "nvidia,tegra186-bpmp";

static int imo_probe(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, ofm_native);
	if (np) {
		of_node_put(np);
		return init_native_override();
	}

	WARN_ON(1);
	return -ENODEV;
}

struct mail_ops mail_ops = {
	.probe = imo_probe,
	.free_master = imo_free_master,
	.master_acked = imo_rx_ready,
	.master_free = imo_tx_ready,
	.return_data = imo_return_data,
	.signal_slave = imo_signal_slave,
	.slave_signalled = imo_rx_ready
};
