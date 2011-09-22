/*
 * drivers/video/tegra/host/nvhost_acm.c
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

#include "nvhost_acm.h"
#include "dev.h"
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <mach/powergate.h>
#include <mach/clk.h>
#include <mach/hardware.h>

#define ACM_SUSPEND_WAIT_FOR_IDLE_TIMEOUT (2 * HZ)
#define POWERGATE_DELAY 10
#define MAX_DEVID_LENGTH 16

void nvhost_module_reset(struct device *dev, struct nvhost_module *mod)
{
	dev_dbg(dev,
		"%s: asserting %s module reset (id %d, id2 %d)\n",
		__func__, mod->name,
		mod->desc->powergate_ids[0], mod->desc->powergate_ids[1]);

	/* assert module and mc client reset */
	if (mod->desc->powergate_ids[0] != -1) {
		tegra_powergate_mc_disable(mod->desc->powergate_ids[0]);
		tegra_periph_reset_assert(mod->clk[0]);
		tegra_powergate_mc_flush(mod->desc->powergate_ids[0]);
	}
	if (mod->desc->powergate_ids[1] != -1) {
		tegra_powergate_mc_disable(mod->desc->powergate_ids[1]);
		tegra_periph_reset_assert(mod->clk[1]);
		tegra_powergate_mc_flush(mod->desc->powergate_ids[1]);
	}

	udelay(POWERGATE_DELAY);

	/* deassert reset */
	if (mod->desc->powergate_ids[0] != -1) {
		tegra_powergate_mc_flush_done(mod->desc->powergate_ids[0]);
		tegra_periph_reset_deassert(mod->clk[0]);
		tegra_powergate_mc_enable(mod->desc->powergate_ids[0]);
	}
	if (mod->desc->powergate_ids[1] != -1) {
		tegra_powergate_mc_flush_done(mod->desc->powergate_ids[1]);
		tegra_periph_reset_deassert(mod->clk[1]);
		tegra_powergate_mc_enable(mod->desc->powergate_ids[1]);
	}

	dev_dbg(dev, "%s: module %s out of reset\n",
		__func__, mod->name);
}

static void clock_enable(struct nvhost_module *mod)
{
	int i;
	for (i = 0; i < mod->num_clks; i++)
		clk_enable(mod->clk[i]);
}

static void unpowergate(struct nvhost_module *mod)
{
	const struct nvhost_moduledesc *desc = mod->desc;
	if (desc->can_powergate) {
		if (desc->powergate_ids[0] != -1)
			tegra_unpowergate_partition(desc->powergate_ids[0]);

		if (desc->powergate_ids[1] != -1)
			tegra_unpowergate_partition(desc->powergate_ids[1]);
	}
}

void nvhost_module_busy(struct nvhost_module *mod)
{
	mutex_lock(&mod->lock);
	cancel_delayed_work(&mod->powerdown);
	if (mod->desc->busy)
		mod->desc->busy(mod);

	if ((atomic_inc_return(&mod->refcount) == 1) && !mod->powered) {
		if (mod->parent)
			nvhost_module_busy(mod->parent);
		unpowergate(mod);
		clock_enable(mod);
		if (mod->desc->finalize_poweron)
			mod->desc->finalize_poweron(mod);
		mod->powered = true;
	}
	mutex_unlock(&mod->lock);
}

static void clock_disable(struct nvhost_module *mod)
{
	int i;
	for (i = 0; i < mod->num_clks; i++)
		clk_disable(mod->clk[i]);
}

static void powergate(struct nvhost_module *mod)
{
	if (mod->desc->can_powergate) {
		if (mod->desc->powergate_ids[0] != -1)
			tegra_powergate_partition(mod->desc->powergate_ids[0]);

		if (mod->desc->powergate_ids[1] != -1)
			tegra_powergate_partition(mod->desc->powergate_ids[1]);
	}
}

static void powerdown_handler(struct work_struct *work)
{
	struct nvhost_module *mod;

	mod = container_of(to_delayed_work(work),
			struct nvhost_module,
			powerdown);
	mutex_lock(&mod->lock);
	if ((atomic_read(&mod->refcount) == 0) && mod->powered) {
		if (mod->desc->prepare_poweroff
			&& mod->desc->prepare_poweroff(mod)) {
			/* If poweroff fails, retry */
			mutex_unlock(&mod->lock);
			schedule_delayed_work(&mod->powerdown,
					msecs_to_jiffies(
						mod->desc->powerdown_delay));
			return;
		}
		clock_disable(mod);
		powergate(mod);
		mod->powered = false;
		if (mod->parent)
			nvhost_module_idle(mod->parent);
	}
	mutex_unlock(&mod->lock);
}


void nvhost_module_idle_mult(struct nvhost_module *mod, int refs)
{
	bool kick = false;

	mutex_lock(&mod->lock);
	if (atomic_sub_return(refs, &mod->refcount) == 0) {
		BUG_ON(!mod->powered);
		schedule_delayed_work(&mod->powerdown,
			msecs_to_jiffies(mod->desc->powerdown_delay));
		kick = true;
	}
	mutex_unlock(&mod->lock);

	if (kick) {
		wake_up(&mod->idle);

		if (mod->desc->idle)
			mod->desc->idle(mod);
	}
}

int nvhost_module_get_rate(struct nvhost_master *host,
		struct nvhost_module *mod, unsigned long *rate,
		int index)
{
	int ret = -EINVAL;
	if (host_acm_op(host).get_rate)
		ret = host_acm_op(host).get_rate(mod, rate, index);
	return ret;
}

int nvhost_module_set_rate(struct nvhost_master *host,
		struct nvhost_module *mod, void *priv,
		unsigned long rate, int index)
{
	int ret = -EINVAL;
	if (host_acm_op(host).set_rate)
		ret = host_acm_op(host).set_rate(mod, priv, rate, index);
	return ret;
}

int nvhost_module_add_client(struct nvhost_master *host,
		struct nvhost_module *mod, void *priv)
{
	int ret = 0;
	if (host_acm_op(host).add_client)
		ret = host_acm_op(host).add_client(mod, priv);
	return ret;
}

void nvhost_module_remove_client(struct nvhost_master *host,
		struct nvhost_module *mod, void *priv)
{
	if (host_acm_op(host).remove_client)
		host_acm_op(host).remove_client(mod, priv);
}

int nvhost_module_init(struct nvhost_module *mod, const char *name,
		const struct nvhost_moduledesc *desc,
		struct nvhost_module *parent,
		struct device *dev)
{
	int i = 0;

	mod->name = name;

	INIT_LIST_HEAD(&mod->client_list);
	while (desc->clocks[i].name && i < NVHOST_MODULE_MAX_CLOCKS) {
		char devname[MAX_DEVID_LENGTH];
		long rate = desc->clocks[i].default_rate;

		snprintf(devname, MAX_DEVID_LENGTH, "tegra_%s", name);
		mod->clk[i] = clk_get_sys(devname, desc->clocks[i].name);
		BUG_ON(IS_ERR_OR_NULL(mod->clk[i]));

		rate = clk_round_rate(mod->clk[i], rate);
		clk_enable(mod->clk[i]);
		clk_set_rate(mod->clk[i], rate);
		clk_disable(mod->clk[i]);
		i++;
	}
	mod->num_clks = i;
	mod->desc = desc;
	mod->parent = parent;
	mod->powered = false;

	mutex_init(&mod->lock);
	init_waitqueue_head(&mod->idle);
	INIT_DELAYED_WORK(&mod->powerdown, powerdown_handler);

	if (!desc->can_powergate) {
		if (desc->powergate_ids[0] != -1)
			tegra_unpowergate_partition(desc->powergate_ids[0]);
		if (desc->powergate_ids[1] != -1)
			tegra_unpowergate_partition(desc->powergate_ids[1]);
	}

	if (desc->init)
		desc->init(dev, mod);

	return 0;
}

static int is_module_idle(struct nvhost_module *mod)
{
	int count;
	mutex_lock(&mod->lock);
	count = atomic_read(&mod->refcount);
	mutex_unlock(&mod->lock);
	return (count == 0);
}

static void debug_not_idle(struct nvhost_master *dev)
{
	int i;
	bool lock_released = true;

	for (i = 0; i < dev->nb_channels; i++) {
		struct nvhost_module *m = &dev->channels[i].mod;
		if (m->name)
			dev_warn(&dev->pdev->dev, "tegra_grhost: %s: refcnt %d\n",
				m->name, atomic_read(&m->refcount));
	}

	for (i = 0; i < dev->nb_mlocks; i++) {
		int c = atomic_read(&dev->cpuaccess.lock_counts[i]);
		if (c) {
			dev_warn(&dev->pdev->dev,
				"tegra_grhost: lock id %d: refcnt %d\n",
				i, c);
			lock_released = false;
		}
	}
	if (lock_released)
		dev_dbg(&dev->pdev->dev, "tegra_grhost: all locks released\n");
}

void nvhost_module_suspend(struct nvhost_module *mod, bool system_suspend)
{
	int ret;
	struct nvhost_master *dev;

	if (system_suspend) {
		dev = container_of(mod, struct nvhost_master, mod);
		if (!is_module_idle(mod))
			debug_not_idle(dev);
	} else {
		dev = container_of(mod, struct nvhost_channel, mod)->dev;
	}

	ret = wait_event_timeout(mod->idle, is_module_idle(mod),
			ACM_SUSPEND_WAIT_FOR_IDLE_TIMEOUT);
	if (ret == 0)
		nvhost_debug_dump(dev);

	if (system_suspend)
		dev_dbg(&dev->pdev->dev, "tegra_grhost: entered idle\n");

	flush_delayed_work(&mod->powerdown);

	if (system_suspend)
		dev_dbg(&dev->pdev->dev, "tegra_grhost: flushed delayed work\n");

	if (mod->desc->suspend)
		mod->desc->suspend(mod);

	BUG_ON(mod->powered);
}

void nvhost_module_deinit(struct device *dev, struct nvhost_module *mod)
{
	int i;

	if (mod->desc->deinit)
		mod->desc->deinit(dev, mod);

	nvhost_module_suspend(mod, false);
	for (i = 0; i < mod->num_clks; i++)
		clk_put(mod->clk[i]);
}

