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
#include "bpmp.h"
#include "mail_t186.h"

/*
 * How the token bits are interpretted
 *
 * SL_SIGL (b00): slave ch in signalled state
 * SL_QUED (b01): slave ch is in queue
 * MA_FREE (b10): master ch is free
 * MA_ACKD (b11): master ch is acked
 *
 * Ideally, the slave should only set bits while the
 * master do only clear them. But there is an exception -
 * see bpmp_ack_master()
 */
#define CH_MASK(ch)	(0x3 << ((ch) * 2))
#define SL_SIGL(ch)	(0x0 << ((ch) * 2))
#define SL_QUED(ch)	(0x1 << ((ch) * 2))
#define MA_FREE(ch)	(0x2 << ((ch) * 2))
#define MA_ACKD(ch)	(0x3 << ((ch) * 2))

static u32 tmo_ch_sta(int ch)
{
	return bpmp_readl(HSP_SHRD_SEM_0_STA) & CH_MASK(ch);
}

static void tmo_ack_master(int ch, int flags)
{
	bpmp_writel(MA_ACKD(ch), HSP_SHRD_SEM_0_SET);

	if (flags & DO_ACK)
		return;

	/*
	 * We have to violate the bit modification rule while
	 * moving from SL_QUED to MA_FREE (DO_ACK not set) so that
	 * the channel won't be in ACKD state forever.
	 */
	bpmp_writel(MA_ACKD(ch) ^ MA_FREE(ch), HSP_SHRD_SEM_0_CLR);
}

static void tmo_free_master(int ch)
{
	bpmp_writel(MA_ACKD(ch) ^ MA_FREE(ch), HSP_SHRD_SEM_0_CLR);
}

static bool tmo_master_free(int ch)
{
	return tmo_ch_sta(ch) == MA_FREE(ch);
}

static bool tmo_slave_signalled(int ch)
{
	return tmo_ch_sta(ch) == SL_SIGL(ch);
}

static bool tmo_master_acked(int ch)
{
	return tmo_ch_sta(ch) == MA_ACKD(ch);
}

static void tmo_signal_slave(int ch)
{
	bpmp_writel(CH_MASK(ch), HSP_SHRD_SEM_0_CLR);
}

static void tmo_return_data(int ch, int code, void *data, int sz)
{
	struct mb_data *p;
	int flags;

	p = channel_area[ch].ob;
	p->code = code;
	memcpy(p->data, data, sz);

	flags = channel_area[ch].ib->flags;
	tmo_ack_master(ch, flags);
	if (flags & RING_DOORBELL)
		bpmp_ring_doorbell();
}

static int tmo_channel_init(int ch, uint8_t *obmem, uint8_t *ibmem, size_t sz)
{
	uint32_t off = ch * MSG_SZ;

	if (off >= sz)
		return -EINVAL;

	channel_area[ch].ib = (struct mb_data *)(ibmem + off);
	channel_area[ch].ob = (struct mb_data *)(obmem + off);

	return 0;
}

const struct mail_ops mail_ops = {
	.channel_init = tmo_channel_init,
	.free_master = tmo_free_master,
	.master_acked = tmo_master_acked,
	.master_free = tmo_master_free,
	.return_data = tmo_return_data,
	.signal_slave = tmo_signal_slave,
	.slave_signalled = tmo_slave_signalled
};
