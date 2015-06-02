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

struct mail_ops {
	bool (*master_acked)(int ch);
	bool (*master_free)(int ch);
	bool (*slave_signalled)(int ch);
	int (*channel_init)(int ch, uint8_t *obmem, uint8_t *ibmem, size_t sz);
	void (*free_master)(int ch);
	void (*return_data)(int ch, int code, void *data, int sz);
	void (*signal_slave)(int ch);
};

extern const struct mail_ops mail_ops;

uint32_t bpmp_readl(uint32_t reg);
void bpmp_writel(uint32_t val, uint32_t reg);

#endif
