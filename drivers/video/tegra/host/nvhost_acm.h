/*
 * drivers/video/tegra/host/nvhost_acm.h
 *
 * Tegra Graphics Host Automatic Clock Management
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __NVHOST_ACM_H
#define __NVHOST_ACM_H

#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/clk.h>

#define NVHOST_MODULE_MAX_CLOCKS 3
#define NVHOST_MODULE_MAX_POWERGATE_IDS 2
struct nvhost_module;
struct nvhost_master;

struct nvhost_moduledesc_clock {
	char *name;
	long default_rate;
};

#define NVHOST_MODULE_NO_POWERGATING .powergate_ids = {-1, -1}
#define NVHOST_DEFAULT_POWERDOWN_DELAY .powerdown_delay = 25

struct nvhost_moduledesc {
	int (*prepare_poweroff)(struct nvhost_module *mod);
	void (*finalize_poweron)(struct nvhost_module *mod);
	void (*busy)(struct nvhost_module *);
	void (*idle)(struct nvhost_module *);
	void (*suspend)(struct nvhost_module *);
	void (*init)(struct device *dev, struct nvhost_module *);
	void (*deinit)(struct device *dev, struct nvhost_module *);

	int powergate_ids[NVHOST_MODULE_MAX_POWERGATE_IDS];
	bool can_powergate;
	int powerdown_delay;
	struct nvhost_moduledesc_clock clocks[NVHOST_MODULE_MAX_CLOCKS];
};

struct nvhost_module {
	const char *name;
	struct delayed_work powerdown;
	int num_clks;
	struct clk *clk[NVHOST_MODULE_MAX_CLOCKS];
	struct mutex lock;
	bool powered;
	atomic_t refcount;
	wait_queue_head_t idle;
	struct nvhost_module *parent;
	const struct nvhost_moduledesc *desc;
	struct list_head client_list;
};

int nvhost_module_init(struct nvhost_module *mod, const char *name,
		const struct nvhost_moduledesc *desc,
		struct nvhost_module *parent,
		struct device *dev);
void nvhost_module_deinit(struct device *dev, struct nvhost_module *mod);
void nvhost_module_suspend(struct nvhost_module *mod, bool system_suspend);

void nvhost_module_reset(struct device *dev, struct nvhost_module *mod);
void nvhost_module_busy(struct nvhost_module *mod);
void nvhost_module_idle_mult(struct nvhost_module *mod, int refs);
int nvhost_module_add_client(struct nvhost_master *host,
		struct nvhost_module *mod,
		void *priv);
void nvhost_module_remove_client(struct nvhost_master *host,
		struct nvhost_module *mod,
		void *priv);
int nvhost_module_get_rate(struct nvhost_master *host,
		struct nvhost_module *mod,
		unsigned long *rate,
		int index);
int nvhost_module_set_rate(struct nvhost_master *host,
		struct nvhost_module *mod, void *priv,
		unsigned long rate, int index);

#define host_acm_op(host) (host->op.acm)

static inline bool nvhost_module_powered(struct nvhost_module *mod)
{
	return mod->powered;
}

static inline void nvhost_module_idle(struct nvhost_module *mod)
{
	nvhost_module_idle_mult(mod, 1);
}


#endif
