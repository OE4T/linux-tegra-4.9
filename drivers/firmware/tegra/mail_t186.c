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
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/tegra-hsp.h>
#include "bpmp.h"

#define TEGRA_BPMP_HSP_BASE		0x150000
#define TEGRA_BPMP_HSP_SHRD_SEM_BASE	(TEGRA_BPMP_HSP_BASE + 0x50000)
#define HSP_SHRD_SEM_0_STA		(TEGRA_BPMP_HSP_SHRD_SEM_BASE + 0x0)
#define HSP_SHRD_SEM_0_SET		(TEGRA_BPMP_HSP_SHRD_SEM_BASE + 0x4)
#define HSP_SHRD_SEM_0_CLR		(TEGRA_BPMP_HSP_SHRD_SEM_BASE + 0x8)

#define CPU_0_TO_BPMP_CH		0
#define CPU_1_TO_BPMP_CH		1
#define CPU_2_TO_BPMP_CH		2
#define CPU_3_TO_BPMP_CH		3
#define CPU_4_TO_BPMP_CH		4
#define CPU_5_TO_BPMP_CH		5
#define CPU_NA_0_TO_BPMP_CH		6
#define CPU_NA_1_TO_BPMP_CH		7
#define CPU_NA_2_TO_BPMP_CH		8
#define CPU_NA_3_TO_BPMP_CH		9
#define CPU_NA_4_TO_BPMP_CH		10
#define CPU_NA_5_TO_BPMP_CH		11
#define CPU_NA_6_TO_BPMP_CH		12
#define BPMP_TO_CPU_CH			13

static void __iomem *bpmp_base;
static void __iomem *cpu_ma_page;
static void __iomem *cpu_sl_page;

static uint32_t bpmp_readl(uint32_t reg)
{
	return __raw_readl(bpmp_base + reg);
}

static void bpmp_writel(uint32_t val, uint32_t reg)
{
	__raw_writel(val, bpmp_base + reg);
}

uint32_t bpmp_mail_token(void)
{
	return bpmp_readl(HSP_SHRD_SEM_0_STA);
}

void bpmp_mail_token_set(uint32_t val)
{
	bpmp_writel(val, HSP_SHRD_SEM_0_SET);
}

void bpmp_mail_token_clr(uint32_t val)
{
	bpmp_writel(val, HSP_SHRD_SEM_0_CLR);
}

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
	return bpmp_mail_token() & CH_MASK(ch);
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
	bpmp_mail_token_clr(CH_MASK(ch));
}

static void bpmp_ack_master(int ch, int flags)
{
	bpmp_mail_token_set(MA_ACKD(ch));

	/*
	 * We have to violate the bit modification rule while
	 * moving from SL_QUED to MA_FREE (DO_ACK not set) so that
	 * the channel won't be in ACKD state forever.
	 */
	if (!(flags & DO_ACK))
		bpmp_mail_token_clr(MA_ACKD(ch) ^ MA_FREE(ch));
}

/* MA_ACKD to MA_FREE */
void bpmp_free_master(int ch)
{
	bpmp_mail_token_clr(MA_ACKD(ch) ^ MA_FREE(ch));
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
		bpmp_ring_doorbell();
}
EXPORT_SYMBOL(tegra_bpmp_mail_return_data);

void bpmp_ring_doorbell(void)
{
	tegra_hsp_db_ring(HSP_DB_BPMP);
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

static void bpmp_inbox_irq(int master, void *data)
{
	bpmp_handle_irq(BPMP_TO_CPU_CH);
}

int bpmp_init_irq(void)
{
	tegra_hsp_db_add_handler(HSP_MASTER_BPMP, bpmp_inbox_irq, NULL);
	tegra_hsp_db_enable_master(HSP_MASTER_BPMP);
	return 0;
}

static void __bpmp_channel_init(int ch)
{
	channel_area[ch].ib = cpu_sl_page + ch * MSG_SZ;
	channel_area[ch].ob = cpu_ma_page + ch * MSG_SZ;
}

static void bpmp_channel_init(void)
{
	int i;

	for (i = 0; i < NR_CHANNELS; i++)
		__bpmp_channel_init(i);
}

static int bpmp_handshake(void)
{
	if (!tegra_hsp_db_can_ring(HSP_DB_BPMP)) {
		pr_err("bpmp db not enabled\n");
		return -ENODEV;
	}

	pr_info("bpmp: waiting for handshake\n");
	while (!bpmp_readl(HSP_SHRD_SEM_0_STA))
		;

	pr_info("bpmp: handshake completed\n");
	return 0;
}

static int bpmp_iomem_init(void)
{
	struct device_node *of_node;

	of_node = of_find_node_by_path("/bpmp");
	if (!of_node) {
		WARN_ON(!of_node);
		return -ENODEV;
	}

	bpmp_base = of_iomap(of_node, 0);
	if (!bpmp_base)
		return -ENODEV;

	cpu_ma_page = of_iomap(of_node, 1);
	if (!cpu_ma_page)
		return -ENODEV;

	cpu_sl_page = of_iomap(of_node, 2);
	if (!cpu_sl_page)
		return -ENODEV;

	return 0;
}

int bpmp_connect(void)
{
	int ret;

	if (connected)
		return 0;

	ret = bpmp_iomem_init();
	if (ret) {
		pr_err("bpmp iomem init failed (%d)\n", ret);
		return ret;
	}

	ret = bpmp_handshake();
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
	return tegra_hsp_init();
}

early_initcall(bpmp_mail_init);

void tegra_bpmp_init_early(void)
{
	bpmp_mail_init();
}
