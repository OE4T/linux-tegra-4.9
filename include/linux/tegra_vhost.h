/*
 * Tegra Host Virtualization Interfaces to Server
 *
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
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

#ifndef __TEGRA_VHOST_H
#define __TEGRA_VHOST_H

enum {
	TEGRA_VHOST_MODULE_HOST = 1,
};

enum {
	TEGRA_VHOST_QUEUE_CMD = 0,
	TEGRA_VHOST_QUEUE_INTR
};

enum {
	TEGRA_VHOST_CMD_CONNECT = 0,
	TEGRA_VHOST_CMD_DISCONNECT,
	TEGRA_VHOST_CMD_ABORT,
	TEGRA_VHOST_CMD_SYNCPT_WRITE,
	TEGRA_VHOST_CMD_SYNCPT_READ,
	TEGRA_VHOST_CMD_SYNCPT_CPU_INCR,
	TEGRA_VHOST_CMD_WAITBASE_WRITE,
	TEGRA_VHOST_CMD_WAITBASE_READ,
	TEGRA_VHOST_CMD_MUTEX_TRY_LOCK,
	TEGRA_VHOST_CMD_MUTEX_UNLOCK,
	TEGRA_VHOST_CMD_SYNCPT_ENABLE_INTR,
	TEGRA_VHOST_CMD_SYNCPT_DISABLE_INTR,
	TEGRA_VHOST_CMD_SYNCPT_DISABLE_INTR_ALL,
	TEGRA_VHOST_CMD_SYNCPT_GET_RANGE
};

struct tegra_vhost_connect_params {
	u32 module;
	u64 handle;
};

struct tegra_vhost_syncpt_params {
	u32 id;
	u32 val;
};

struct tegra_vhost_syncpt_range_params {
	u32 base;
	u32 size;
};

struct tegra_vhost_waitbase_params {
	u32 id;
	u32 val;
};

struct tegra_vhost_mutex_params {
	u32 id;
	u32 locked;
	u32 owner;
};

struct tegra_vhost_syncpt_intr_params {
	u32 id;
	u32 thresh;
};

struct tegra_vhost_cmd_msg {
	u32 cmd;
	int ret;
	u64 handle;
	union {
		struct tegra_vhost_connect_params connect;
		struct tegra_vhost_syncpt_params syncpt;
		struct tegra_vhost_syncpt_range_params syncpt_range;
		struct tegra_vhost_waitbase_params waitbase;
		struct tegra_vhost_mutex_params mutex;
		struct tegra_vhost_syncpt_intr_params syncpt_intr;
	} params;
};

enum {
	TEGRA_VHOST_EVENT_SYNCPT_INTR = 0,
	TEGRA_VHOST_EVENT_ABORT
};

struct tegra_vhost_syncpt_intr_msg {
	unsigned int event;
	u32 id;
	u32 thresh;
};

#define TEGRA_VHOST_QUEUE_SIZES			\
	sizeof(struct tegra_vhost_cmd_msg),	\
	sizeof(struct tegra_vhost_syncpt_intr_msg)

#endif
