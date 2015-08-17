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

#ifndef MAIL_T186_H
#define MAIL_T186_H

#include <linux/kernel.h>

#define TEGRA_BPMP_HSP_BASE		0x150000
#define TEGRA_BPMP_HSP_SHRD_SEM_BASE	(TEGRA_BPMP_HSP_BASE + 0x50000)
#define HSP_SHRD_SEM_0_STA		(TEGRA_BPMP_HSP_SHRD_SEM_BASE + 0x0)
#define HSP_SHRD_SEM_0_SET		(TEGRA_BPMP_HSP_SHRD_SEM_BASE + 0x4)
#define HSP_SHRD_SEM_0_CLR		(TEGRA_BPMP_HSP_SHRD_SEM_BASE + 0x8)
#define HSP_SHRD_SEM_1_STA		(TEGRA_BPMP_HSP_SHRD_SEM_BASE + 0x10000)
#define HSP_SHRD_SEM_1_SET		(TEGRA_BPMP_HSP_SHRD_SEM_BASE + 0x10004)
#define HSP_SHRD_SEM_1_CLR		(TEGRA_BPMP_HSP_SHRD_SEM_BASE + 0x10008)

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

struct ivc;

struct transport_layer_ops {
	struct ivc *(*channel_to_ivc)(int ch);
};

struct mail_ops {
	int (*probe)(void);
	int (*init_prepare)(void);
	int (*init_irq)(void);
	int (*iomem_init)(void);
	int (*handshake)(void);
	bool (*master_acked)(int ch);
	bool (*master_free)(int ch);
	bool (*slave_signalled)(int ch);
	int (*channel_init)(int ch);
	void (*free_master)(int ch);
	void (*resume)(void);
	void (*return_data)(int ch, int code, void *data, int sz);
	void (*signal_slave)(int ch);
	void (*ring_doorbell)(void);
};

extern struct mail_ops mail_ops;
extern struct transport_layer_ops trans_ops;

int init_native_override(void);

#endif
