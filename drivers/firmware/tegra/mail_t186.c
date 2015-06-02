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
#include "mail_t186.h"

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

uint32_t bpmp_readl(uint32_t reg)
{
	return __raw_readl(bpmp_base + reg);
}

void bpmp_writel(uint32_t val, uint32_t reg)
{
	__raw_writel(val, bpmp_base + reg);
}

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

static int bpmp_channel_init(void)
{
	int e = 0;
	int i;

	for (i = 0; i < NR_CHANNELS && !e; i++)
		e = mail_ops.channel_init(i, cpu_ma_page, cpu_sl_page, SZ_4K);

	return e;
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
