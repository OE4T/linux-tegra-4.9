/*
 * drivers/video/tegra/host/nvhost_acm.c
 *
 * Tegra Graphics Host Automatic Clock Management
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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
#include <linux/platform_device.h>
#include <mach/powergate.h>
#include <mach/clk.h>
#include <mach/hardware.h>

#define ACM_SUSPEND_WAIT_FOR_IDLE_TIMEOUT (2 * HZ)
#define POWERGATE_DELAY 10
#define MAX_DEVID_LENGTH 16

DEFINE_MUTEX(client_list_lock);

struct nvhost_module_client {
	struct list_head node;
	unsigned long rate[NVHOST_MODULE_MAX_CLOCKS];
	void *priv;
};

static void do_powergate_locked(int id)
{
	if (id != -1 && tegra_powergate_is_powered(id))
		tegra_powergate_partition(id);
}

static void do_unpowergate_locked(int id)
{
	if (id != -1)
		tegra_unpowergate_partition(id);
}

void nvhost_module_reset(struct nvhost_device *dev)
{
	dev_dbg(&dev->dev,
		"%s: asserting %s module reset (id %d, id2 %d)\n",
		__func__, dev->name,
		dev->powergate_ids[0], dev->powergate_ids[1]);

	mutex_lock(&dev->lock);

	/* assert module and mc client reset */
	if (dev->powergate_ids[0] != -1) {
		tegra_powergate_mc_disable(dev->powergate_ids[0]);
		tegra_periph_reset_assert(dev->clk[0]);
		tegra_powergate_mc_flush(dev->powergate_ids[0]);
	}
	if (dev->powergate_ids[1] != -1) {
		tegra_powergate_mc_disable(dev->powergate_ids[1]);
		tegra_periph_reset_assert(dev->clk[1]);
		tegra_powergate_mc_flush(dev->powergate_ids[1]);
	}

	udelay(POWERGATE_DELAY);

	/* deassert reset */
	if (dev->powergate_ids[0] != -1) {
		tegra_powergate_mc_flush_done(dev->powergate_ids[0]);
		tegra_periph_reset_deassert(dev->clk[0]);
		tegra_powergate_mc_enable(dev->powergate_ids[0]);
	}
	if (dev->powergate_ids[1] != -1) {
		tegra_powergate_mc_flush_done(dev->powergate_ids[1]);
		tegra_periph_reset_deassert(dev->clk[1]);
		tegra_powergate_mc_enable(dev->powergate_ids[1]);
	}

	mutex_unlock(&dev->lock);

	dev_dbg(&dev->dev, "%s: module %s out of reset\n",
		__func__, dev->name);
}

static void to_state_clockgated_locked(struct nvhost_device *dev)
{
	if (dev->powerstate == NVHOST_POWER_STATE_RUNNING) {
		int i;
		for (i = 0; i < dev->num_clks; i++)
			clk_disable(dev->clk[i]);
		if (dev->dev.parent)
			nvhost_module_idle(to_nvhost_device(dev->dev.parent));
	} else if (dev->powerstate == NVHOST_POWER_STATE_POWERGATED
			&& dev->can_powergate) {
		do_unpowergate_locked(dev->powergate_ids[0]);
		do_unpowergate_locked(dev->powergate_ids[1]);
	}
	dev->powerstate = NVHOST_POWER_STATE_CLOCKGATED;
}

static void to_state_running_locked(struct nvhost_device *dev)
{
	int prev_state = dev->powerstate;
	if (dev->powerstate == NVHOST_POWER_STATE_POWERGATED)
		to_state_clockgated_locked(dev);
	if (dev->powerstate == NVHOST_POWER_STATE_CLOCKGATED) {
		int i;

		if (dev->dev.parent)
			nvhost_module_busy(to_nvhost_device(dev->dev.parent));

		for (i = 0; i < dev->num_clks; i++) {
			int err = clk_enable(dev->clk[i]);
			BUG_ON(err);
		}

		if (prev_state == NVHOST_POWER_STATE_POWERGATED
				&& dev->finalize_poweron)
			dev->finalize_poweron(dev);
	}
	dev->powerstate = NVHOST_POWER_STATE_RUNNING;
}

/* This gets called from powergate_handler() and from module suspend.
 * Module suspend is done for all modules, runtime power gating only
 * for modules with can_powergate set.
 */
static int to_state_powergated_locked(struct nvhost_device *dev)
{
	int err = 0;

	if (dev->prepare_poweroff
			&& dev->powerstate != NVHOST_POWER_STATE_POWERGATED) {
		/* Clock needs to be on in prepare_poweroff */
		to_state_running_locked(dev);
		err = dev->prepare_poweroff(dev);
		if (err)
			return err;
	}

	if (dev->powerstate == NVHOST_POWER_STATE_RUNNING)
		to_state_clockgated_locked(dev);

	if (dev->can_powergate) {
		do_powergate_locked(dev->powergate_ids[0]);
		do_powergate_locked(dev->powergate_ids[1]);
	}

	dev->powerstate = NVHOST_POWER_STATE_POWERGATED;
	return 0;
}

static void schedule_powergating_locked(struct nvhost_device *dev)
{
	if (dev->can_powergate)
		schedule_delayed_work(&dev->powerstate_down,
				msecs_to_jiffies(dev->powergate_delay));
}

static void schedule_clockgating_locked(struct nvhost_device *dev)
{
	schedule_delayed_work(&dev->powerstate_down,
			msecs_to_jiffies(dev->clockgate_delay));
}

void nvhost_module_busy(struct nvhost_device *dev)
{
	if (dev->busy)
		dev->busy(dev);

	mutex_lock(&dev->lock);
	cancel_delayed_work(&dev->powerstate_down);

	dev->refcount++;
	if (dev->refcount > 0 && !nvhost_module_powered(dev))
		to_state_running_locked(dev);
	mutex_unlock(&dev->lock);
}

static void powerstate_down_handler(struct work_struct *work)
{
	struct nvhost_device *dev;

	dev = container_of(to_delayed_work(work),
			struct nvhost_device,
			powerstate_down);

	mutex_lock(&dev->lock);
	if (dev->refcount == 0) {
		switch (dev->powerstate) {
		case NVHOST_POWER_STATE_RUNNING:
			to_state_clockgated_locked(dev);
			schedule_powergating_locked(dev);
			break;
		case NVHOST_POWER_STATE_CLOCKGATED:
			if (to_state_powergated_locked(dev))
				schedule_powergating_locked(dev);
			break;
		default:
			break;
		}
	}
	mutex_unlock(&dev->lock);
}


void nvhost_module_idle_mult(struct nvhost_device *dev, int refs)
{
	bool kick = false;

	mutex_lock(&dev->lock);
	dev->refcount -= refs;
	if (dev->refcount == 0) {
		if (nvhost_module_powered(dev))
			schedule_clockgating_locked(dev);
		kick = true;
	}
	mutex_unlock(&dev->lock);

	if (kick) {
		wake_up(&dev->idle_wq);

		if (dev->idle)
			dev->idle(dev);
	}
}

int nvhost_module_get_rate(struct nvhost_device *dev, unsigned long *rate,
		int index)
{
	struct clk *c;

	c = dev->clk[index];
	if (IS_ERR_OR_NULL(c))
		return -EINVAL;

	/* Need to enable client to get correct rate */
	nvhost_module_busy(dev);
	*rate = clk_get_rate(c);
	nvhost_module_idle(dev);
	return 0;

}

static int nvhost_module_update_rate(struct nvhost_device *dev, int index)
{
	unsigned long rate = 0;
	struct nvhost_module_client *m;

	if (!dev->clk[index])
		return -EINVAL;

	list_for_each_entry(m, &dev->client_list, node) {
		rate = max(m->rate[index], rate);
	}
	if (!rate)
		rate = clk_round_rate(dev->clk[index],
				dev->clocks[index].default_rate);

	return clk_set_rate(dev->clk[index], rate);
}

int nvhost_module_set_rate(struct nvhost_device *dev, void *priv,
		unsigned long rate, int index)
{
	struct nvhost_module_client *m;
	int ret;

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &dev->client_list, node) {
		if (m->priv == priv) {
			rate = clk_round_rate(dev->clk[index], rate);
			m->rate[index] = rate;
			break;
		}
	}
	ret = nvhost_module_update_rate(dev, index);
	mutex_unlock(&client_list_lock);
	return ret;

}

int nvhost_module_add_client(struct nvhost_device *dev, void *priv)
{
	int i;
	unsigned long rate;
	struct nvhost_module_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	INIT_LIST_HEAD(&client->node);
	client->priv = priv;

	for (i = 0; i < dev->num_clks; i++) {
		rate = clk_round_rate(dev->clk[i],
				dev->clocks[i].default_rate);
		client->rate[i] = rate;
	}
	mutex_lock(&client_list_lock);
	list_add_tail(&client->node, &dev->client_list);
	mutex_unlock(&client_list_lock);
	return 0;
}

void nvhost_module_remove_client(struct nvhost_device *dev, void *priv)
{
	int i;
	struct nvhost_module_client *m;

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &dev->client_list, node) {
		if (priv == m->priv) {
			list_del(&m->node);
			break;
		}
	}
	if (m) {
		kfree(m);
		for (i = 0; i < dev->num_clks; i++)
			nvhost_module_update_rate(dev, i);
	}
	mutex_unlock(&client_list_lock);
}

int nvhost_module_init(struct nvhost_device *dev)
{
	int i = 0;

	/* initialize clocks to known state */
	INIT_LIST_HEAD(&dev->client_list);
	while (dev->clocks[i].name && i < NVHOST_MODULE_MAX_CLOCKS) {
		char devname[MAX_DEVID_LENGTH];
		long rate = dev->clocks[i].default_rate;
		struct clk *c;

		snprintf(devname, MAX_DEVID_LENGTH, "tegra_%s", dev->name);
		c = clk_get_sys(devname, dev->clocks[i].name);
		BUG_ON(IS_ERR_OR_NULL(c));

		rate = clk_round_rate(c, rate);
		clk_enable(c);
		clk_set_rate(c, rate);
		clk_disable(c);
		dev->clk[i] = c;
		i++;
	}
	dev->num_clks = i;

	mutex_init(&dev->lock);
	init_waitqueue_head(&dev->idle_wq);
	INIT_DELAYED_WORK(&dev->powerstate_down, powerstate_down_handler);

	/* power gate units that we can power gate */
	if (dev->can_powergate) {
		do_powergate_locked(dev->powergate_ids[0]);
		do_powergate_locked(dev->powergate_ids[1]);
		dev->powerstate = NVHOST_POWER_STATE_POWERGATED;
	} else {
		do_unpowergate_locked(dev->powergate_ids[0]);
		do_unpowergate_locked(dev->powergate_ids[1]);
		dev->powerstate = NVHOST_POWER_STATE_CLOCKGATED;
	}

	return 0;
}

static int is_module_idle(struct nvhost_device *dev)
{
	int count;
	mutex_lock(&dev->lock);
	count = dev->refcount;
	mutex_unlock(&dev->lock);
	return (count == 0);
}

static void debug_not_idle(struct nvhost_master *host)
{
	int i;
	bool lock_released = true;

	for (i = 0; i < host->nb_channels; i++) {
		struct nvhost_device *dev = host->channels[i].dev;
		mutex_lock(&dev->lock);
		if (dev->name)
			dev_warn(&host->pdev->dev,
					"tegra_grhost: %s: refcnt %d\n",
					dev->name, dev->refcount);
		mutex_unlock(&dev->lock);
	}

	for (i = 0; i < host->syncpt.nb_mlocks; i++) {
		int c = atomic_read(&host->syncpt.lock_counts[i]);
		if (c) {
			dev_warn(&host->pdev->dev,
				"tegra_grhost: lock id %d: refcnt %d\n",
				i, c);
			lock_released = false;
		}
	}
	if (lock_released)
		dev_dbg(&host->pdev->dev, "tegra_grhost: all locks released\n");
}

int nvhost_module_suspend(struct nvhost_device *dev, bool system_suspend)
{
	int ret;
	struct nvhost_master *host = nvhost_get_host(dev);

	if (system_suspend && !is_module_idle(dev))
		debug_not_idle(host);

	ret = wait_event_timeout(dev->idle_wq, is_module_idle(dev),
			ACM_SUSPEND_WAIT_FOR_IDLE_TIMEOUT);
	if (ret == 0) {
		dev_info(&dev->dev, "%s prevented suspend\n",
				dev->name);
		return -EBUSY;
	}

	if (system_suspend)
		dev_dbg(&dev->dev, "tegra_grhost: entered idle\n");

	mutex_lock(&dev->lock);
	cancel_delayed_work(&dev->powerstate_down);
	to_state_powergated_locked(dev);
	mutex_unlock(&dev->lock);

	if (dev->suspend)
		dev->suspend(dev);

	return 0;
}

void nvhost_module_deinit(struct nvhost_device *dev)
{
	int i;

	if (dev->deinit)
		dev->deinit(dev);

	nvhost_module_suspend(dev, false);
	for (i = 0; i < dev->num_clks; i++)
		clk_put(dev->clk[i]);
	dev->powerstate = NVHOST_POWER_STATE_DEINIT;
}

