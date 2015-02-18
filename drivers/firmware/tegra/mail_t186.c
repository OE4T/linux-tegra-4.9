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

int bpmp_init_irq(struct platform_device *pdev)
{
	tegra_hsp_db_add_handler(HSP_MASTER_BPMP, bpmp_inbox_irq, NULL);
	tegra_hsp_db_enable_master(HSP_MASTER_BPMP);
	return 0;
}

#define CPU_MA_PAGE	0
#define CPU_SL_PAGE	1
#define GSC_PAGE_SZ	4096

static void bpmp_get_sysram_off(int ch, int *ib_off, int *ob_off)
{
	int ib_page;
	int ob_page;

	if (ch == BPMP_TO_CPU_CH) {
		ib_page = CPU_SL_PAGE;
		ob_page = CPU_MA_PAGE;
	} else {
		ib_page = CPU_MA_PAGE;
		ob_page = CPU_SL_PAGE;
	}

	*ib_off = SZ_256K + ib_page * GSC_PAGE_SZ + ch * MSG_SZ;
	*ob_off = SZ_256K + ob_page * GSC_PAGE_SZ + ch * MSG_SZ;
}

static void bpmp_channel_init(void *virt)
{
	uint8_t *base = virt;
	int ib_off;
	int ob_off;
	int i;

	for (i = 0; i < NR_CHANNELS; i++) {
		bpmp_get_sysram_off(i, &ib_off, &ob_off);
		channel_area[i].ib = (struct mb_data *)(base + ib_off);
		channel_area[i].ob = (struct mb_data *)(base + ob_off);
	}
}

static int bpmp_handshake(void)
{
	if (!tegra_hsp_db_can_ring(HSP_DB_BPMP)) {
		dev_err(device, "doorbell not enabled\n");
		return -EPERM;
	}

	if (!bpmp_readl(HSP_SHRD_SEM_0_STA)) {
		dev_err(device, "firmware mailman not ready\n");
		return -ENODEV;
	}

	return 0;
}

int bpmp_connect(void)
{
	void __iomem *mb;
	int ret;

	if (connected)
		return 0;

	bpmp_base = of_iomap(device->of_node, 0);
	mb = of_iomap(device->of_node, 1);
	bpmp_channel_init(mb);

	ret = bpmp_handshake();
	if (ret) {
		dev_err(device, "bpmp handshake failed (%d)\n", ret);
		return ret;
	}

	connected = 1;
	return 0;
}
