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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/tegra-hsp.h>
#include "bpmp.h"
#include "mail_t186.h"

bool bpmp_master_acked(int ch)
{
	return mail_ops.master_acked(ch);
}

bool bpmp_slave_signalled(int ch)
{
	return mail_ops.slave_signalled(ch);
}

void bpmp_free_master(int ch)
{
	mail_ops.free_master(ch);
}

void bpmp_signal_slave(int ch)
{
	mail_ops.signal_slave(ch);
}

bool bpmp_master_free(int ch)
{
	return mail_ops.master_free(ch);
}

void tegra_bpmp_mail_return_data(int ch, int code, void *data, int sz)
{
	if (sz > MSG_DATA_SZ) {
		WARN_ON(1);
		return;
	}

	mail_ops.return_data(ch, code, data, sz);
}
EXPORT_SYMBOL(tegra_bpmp_mail_return_data);

void bpmp_ring_doorbell(void)
{
	if (mail_ops.ring_doorbell)
		mail_ops.ring_doorbell();
}

int bpmp_thread_ch_index(int ch)
{
	if (ch < CPU_NA_0_TO_BPMP_CH || ch > CPU_NA_6_TO_BPMP_CH)
		return -1;
	return ch - CPU_NA_0_TO_BPMP_CH;
}

int bpmp_thread_ch(int idx)
{
	return CPU_NA_0_TO_BPMP_CH + idx;
}

int bpmp_ob_channel(void)
{
	return smp_processor_id() + CPU_0_TO_BPMP_CH;
}

int bpmp_init_irq(void)
{
	if (mail_ops.init_irq)
		return mail_ops.init_irq();

	return 0;
}

static int bpmp_channel_init(void)
{
	int e = 0;
	int i;

	if (!mail_ops.channel_init)
		return 0;

	for (i = 0; i < NR_CHANNELS && !e; i++)
		e = mail_ops.channel_init(i);

	return e;
}

void tegra_bpmp_resume(void)
{
	if (mail_ops.resume)
		mail_ops.resume();
}

int bpmp_connect(void)
{
	int ret = 0;

	if (connected)
		return 0;

	if (mail_ops.iomem_init)
		ret = mail_ops.iomem_init();

	if (ret) {
		pr_err("bpmp iomem init failed (%d)\n", ret);
		return ret;
	}

	ret = mail_ops.handshake();
	if (ret) {
		pr_err("bpmp handshake failed (%d)\n", ret);
		return ret;
	}

	bpmp_channel_init();
	connected = 1;
	return 0;
}

int bpmp_mail_init_prepare(void)
{
	int r;

	r = mail_ops.probe();
	if (r)
		return r;

	if (mail_ops.init_prepare)
		return mail_ops.init_prepare();

	return 0;
}

early_initcall(bpmp_mail_init);

void tegra_bpmp_init_early(void)
{
	bpmp_mail_init();
}
