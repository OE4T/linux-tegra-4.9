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

#ifndef _LINUX_TEGRA_IVC_CLIENT_H
#define _LINUX_TEGRA_IVC_CLIENT_H

#include <linux/tegra-hsp.h>

struct tegra_ivcc_chan;

#define TEGRA_IVCC_FLAG_POLL		BIT(0)
#define TEGRA_IVCC_FLAG_NOLOCKS		BIT(1)

#define TEGRA_IVCC_NOTIMEOUT		0xffffffff

typedef void (*tegra_ivcc_notify)(struct tegra_ivcc_chan *ipc);

int tegra_ivcc_send(struct tegra_ivcc_chan *ipcd, void *frame,
		u32 size, u32 flags);

int tegra_ivcc_send_timeout(struct tegra_ivcc_chan *ipcd, void *frame,
		u32 size, u32 flags, u32 timeout);

int tegra_ivcc_recv(struct tegra_ivcc_chan *ipcd, void *frame,
		u32 size, u32 flags);

int tegra_ivcc_recv_timeout(struct tegra_ivcc_chan *ipcd, void *frame,
		u32 size, u32 flags, u32 timeout);

void tegra_ivcc_set_send_notify(struct tegra_ivcc_chan *ipcd,
		tegra_ivcc_notify func);

void tegra_ivcc_set_recv_notify(struct tegra_ivcc_chan *ipcd,
		tegra_ivcc_notify func);

struct tegra_ivcc_chan *tegra_ivcc_find_queue_by_phandle(
		struct device_node *np, int index);

struct tegra_ivcc_chan *tegra_ivcc_find_queue_by_name(const char *name);

#endif
