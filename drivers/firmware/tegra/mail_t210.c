/*
 * Copyright (c) 2013-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/of_irq.h>
#include <soc/tegra/doorbell.h>
#include "../../../arch/arm/mach-tegra/iomap.h"
#include "bpmp.h"

static void *arb_sema, *atomics;

/* CPU to BPMP atomic channels */
#define CPU0_OB_CH0		0
#define CPU1_OB_CH0		1
#define CPU2_OB_CH0		2
#define CPU3_OB_CH0		3

/* CPU to BPMP non-atomic channels */
#define CPU0_OB_CH1		4
#define CPU1_OB_CH1		5
#define CPU2_OB_CH1		6
#define CPU3_OB_CH1		7

/* BPMP to CPU channels */
#define CPU0_IB_CH		8
#define CPU1_IB_CH		9
#define CPU2_IB_CH		10
#define CPU3_IB_CH		11

#define CPU_OB_DOORBELL		4

#define ATOMICS_AP0_TRIGGER	(atomics + 0x000)
#define ATOMICS_AP0_RESULT(id)	(atomics + 0xc00 + id * 4)
#define TRIGGER_ID_SHIFT	16
#define TRIGGER_CMD_GET		4

#define RES_SEMA_SHRD_SMP_STA	(arb_sema)
#define RES_SEMA_SHRD_SMP_SET	(arb_sema + 4)
#define RES_SEMA_SHRD_SMP_CLR	(arb_sema + 8)

#define PER_CPU_IB_CH(i)	(CPU0_IB_CH + i)

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

static u32 bpmp_ch_sta(int ch)
{
	return __raw_readl(RES_SEMA_SHRD_SMP_STA) & CH_MASK(ch);
}

bool bpmp_master_free(int ch)
{
	return bpmp_ch_sta(ch) == MA_FREE(ch);
}

bool bpmp_slave_signalled(int ch)
{
	return bpmp_ch_sta(ch) == SL_SIGL(ch);
}

bool bpmp_master_acked(int ch)
{
	return bpmp_ch_sta(ch) == MA_ACKD(ch);
}

void bpmp_signal_slave(int ch)
{
	__raw_writel(CH_MASK(ch), RES_SEMA_SHRD_SMP_CLR);
}

static void bpmp_ack_master(int ch, int flags)
{
	__raw_writel(MA_ACKD(ch), RES_SEMA_SHRD_SMP_SET);

	if (flags & DO_ACK)
		return;

	/*
	 * We have to violate the bit modification rule while
	 * moving from SL_QUED to MA_FREE (DO_ACK not set) so that
	 * the channel won't be in ACKD state forever.
	 */
	__raw_writel(MA_ACKD(ch) ^ MA_FREE(ch), RES_SEMA_SHRD_SMP_CLR);
}

/* MA_ACKD to MA_FREE */
void bpmp_free_master(int ch)
{
	__raw_writel(MA_ACKD(ch) ^ MA_FREE(ch), RES_SEMA_SHRD_SMP_CLR);
}

void bpmp_ring_doorbell(int ch)
{
	tegra_ring_doorbell(CPU_OB_DOORBELL);
}

void tegra_bpmp_mail_return_data(int ch, int code, void *data, int sz)
{
	struct mb_data *p;
	int flags;

	if (sz > MSG_DATA_SZ) {
		WARN_ON(1);
		return;
	}

	p = channel_area[ch].ob;
	p->code = code;
	memcpy(p->data, data, sz);

	flags = channel_area[ch].ib->flags;
	bpmp_ack_master(ch, flags);
	if (flags & RING_DOORBELL)
		bpmp_ring_doorbell(ch);
}
EXPORT_SYMBOL(tegra_bpmp_mail_return_data);

int bpmp_thread_ch_index(int ch)
{
	if (ch < CPU0_OB_CH1 || ch > CPU3_OB_CH1)
		return -1;
	return ch - CPU0_OB_CH1;
}

int bpmp_thread_ch(int idx)
{
	return CPU0_OB_CH1 + idx;
}

int bpmp_ob_channel(void)
{
	return smp_processor_id() + CPU0_OB_CH0;
}

static void bpmp_doorbell_handler(void *data)
{
	int ch = (long)data;

	bpmp_handle_irq(ch);
}

int bpmp_init_irq(void)
{
	long ch;
	int r;
	int i;

	for (i = 0; i < 4; i++) {
		ch = PER_CPU_IB_CH(i);
		r = tegra_register_doorbell_handler(i, bpmp_doorbell_handler,
						   (void *)ch);
		if (r)
			return r;
	}

	return 0;
}

/* Channel area is setup by BPMP before signalling handshake */
static u32 bpmp_channel_area(int ch)
{
	u32 a;

	writel(ch << TRIGGER_ID_SHIFT | TRIGGER_CMD_GET, ATOMICS_AP0_TRIGGER);
	a = readl(ATOMICS_AP0_RESULT(ch));

	return a;
}

static int __bpmp_connect(void)
{
	void *p;
	u32 channel_hwaddr[NR_CHANNELS];
	int i;

	if (connected)
		return 0;

	/* handshake */
	if (!readl(RES_SEMA_SHRD_SMP_STA))
		return -ENODEV;

	for (i = 0; i < NR_CHANNELS; i++) {
		channel_hwaddr[i] = bpmp_channel_area(i);
		if (!channel_hwaddr[i])
			return -EFAULT;
	}

	for (i = 0; i < NR_CHANNELS; i++) {
		p = ioremap(channel_hwaddr[i], 0x80);

		channel_area[i].ib = p;
		channel_area[i].ob = p;
	}

	connected = 1;
	return 0;
}

int bpmp_connect(struct platform_device *pdev)
{
	struct resource *res;
	/* firmware loaded after boot */
	if (IS_ENABLED(CONFIG_ARCH_TEGRA_12x_SOC))
		return 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	atomics = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(atomics))
		return PTR_ERR(atomics);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	arb_sema = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(arb_sema))
		return PTR_ERR(arb_sema);

	return __bpmp_connect();
}

void bpmp_detach(void)
{
	int i;

	connected = 0;
	writel(0xffffffff, RES_SEMA_SHRD_SMP_CLR);

	for (i = 0; i < NR_CHANNELS; i++) {
		channel_area[i].ib = NULL;
		channel_area[i].ob = NULL;
	}
}

int bpmp_attach(void)
{
	int i;

	WARN_ON(connected);

	for (i = 0; i < MSEC_PER_SEC * 60; i += 20) {
		if (!__bpmp_connect())
			return 0;
		msleep(20);
	}

	return -ETIMEDOUT;
}

void tegra_bpmp_resume(void)
{
}

int bpmp_mail_init_prepare(void)
{
	return 0;
}
