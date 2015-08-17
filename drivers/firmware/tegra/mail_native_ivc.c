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

#include <linux/of_address.h>
#include <linux/tegra-hsp.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/tegra-soc.h>
#include "bpmp.h"
#include "mail_t186.h"

static struct ivc ivc_channels[NR_CHANNELS];

static void __iomem *bpmp_base;
static void __iomem *cpu_ma_page;
static void __iomem *cpu_sl_page;

static int native_init_prepare(void)
{
	return tegra_hsp_init();
}

static void native_inbox_irq(int master, void *data)
{
	bpmp_handle_irq(BPMP_TO_CPU_CH);
}

static int native_init_irq(void)
{
	tegra_hsp_db_add_handler(HSP_MASTER_BPMP, native_inbox_irq, NULL);
	tegra_hsp_db_enable_master(HSP_MASTER_BPMP);

	return 0;
}

static int native_iomem_init(void)
{
	struct device_node *of_node;

	/* FIXME: do not assume DT path */
	of_node = of_find_node_by_path("/bpmp");
	if (WARN_ON(!of_device_is_available(of_node)))
		return -ENODEV;

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

static int native_handshake(void)
{
	/* HSP_SHRD_SEM_1_STA is not modelled in all unit FPGAs*/
	if (tegra_platform_is_unit_fpga())
		return -ENODEV;

	/* FIXME: short-term WAR */
	if (!tegra_hsp_db_can_ring(HSP_DB_BPMP) &&
			!__raw_readl(bpmp_base + HSP_SHRD_SEM_1_STA)) {
		pr_err("bpmp db not enabled\n");
		return -ENODEV;
	}

	/* FIXME: remove HSP_SHRD_SEM_0_STA */
	pr_info("bpmp: waiting for handshake\n");
	while (!__raw_readl(bpmp_base + HSP_SHRD_SEM_0_STA) ||
			!tegra_hsp_db_can_ring(HSP_DB_BPMP))
		;

	pr_info("bpmp: handshake completed\n");
	return 0;
}

static void native_resume(void)
{
	size_t sz;
	int i;

	pr_info("bpmp: waiting for handshake\n");
	while (!tegra_hsp_db_can_ring(HSP_DB_BPMP))
		;

	pr_info("bpmp: resuming channels\n");
	sz = tegra_ivc_total_queue_size(0);

	for (i = 0; i < NR_CHANNELS; i++)
		memset_io((void *)ivc_channels[i].tx_channel, 0, sz);
}

static void native_notify(struct ivc *ivc)
{
}

static int native_channel_init(int ch)
{
	struct ivc *ivc;
	uintptr_t rx_base;
	uintptr_t tx_base;
	size_t msg_sz;
	size_t que_sz;
	size_t hdr_sz;
	int r;

	msg_sz = tegra_ivc_align(MSG_SZ);
	hdr_sz = tegra_ivc_total_queue_size(0);
	que_sz = tegra_ivc_total_queue_size(msg_sz);

	rx_base = (uintptr_t)((uint8_t *)cpu_sl_page + ch * que_sz);
	tx_base = (uintptr_t)((uint8_t *)cpu_ma_page + ch * que_sz);

	/* init the channel frame */
	memset_io((void *)tx_base, 0, hdr_sz);

	ivc = ivc_channels + ch;
	r = tegra_ivc_init(ivc, rx_base, tx_base,
			1, msg_sz, device, native_notify);
	if (r) {
		pr_err("tegra_ivc_init() ch %d returned %d\n", ch, r);
		WARN_ON(1);
		return r;
	}

	return 0;
}

static void native_ring_doorbell(void)
{
	tegra_hsp_db_ring(HSP_DB_BPMP);
}

static struct ivc *native_channel_to_ivc(int ch)
{
	return ivc_channels + ch;
}

int init_native_override(void)
{
	trans_ops.channel_to_ivc = native_channel_to_ivc;

	mail_ops.init_prepare = native_init_prepare;
	mail_ops.ring_doorbell = native_ring_doorbell;
	mail_ops.init_irq = native_init_irq;
	mail_ops.iomem_init = native_iomem_init;
	mail_ops.handshake = native_handshake;
	mail_ops.channel_init = native_channel_init;
	mail_ops.resume = native_resume;

	return 0;
}
