/*
 * drivers/video/tegra/host/nvhost_acm.c
 *
 * Tegra Graphics Host Automatic Clock Management
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <mach/powergate.h>
#include <mach/clk.h>
#include <mach/hardware.h>

#include "nvhost_acm.h"
#include "dev.h"

#define ACM_SUSPEND_WAIT_FOR_IDLE_TIMEOUT	(2 * HZ)
#define POWERGATE_DELAY 			10
#define MAX_DEVID_LENGTH			16

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

static void do_module_reset_locked(struct nvhost_device *dev)
{
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
}

void nvhost_module_reset(struct nvhost_device *dev)
{
	dev_dbg(&dev->dev,
		"%s: asserting %s module reset (id %d, id2 %d)\n",
		__func__, dev->name,
		dev->powergate_ids[0], dev->powergate_ids[1]);

	mutex_lock(&dev->lock);
	do_module_reset_locked(dev);
	mutex_unlock(&dev->lock);

	dev_dbg(&dev->dev, "%s: module %s out of reset\n",
		__func__, dev->name);
}

static void to_state_clockgated_locked(struct nvhost_device *dev)
{
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);

	if (dev->powerstate == NVHOST_POWER_STATE_RUNNING) {
		int i, err;
		if (drv->prepare_clockoff) {
			err = drv->prepare_clockoff(dev);
			if (err) {
				dev_err(&dev->dev, "error clock gating");
				return;
			}
		}
		for (i = 0; i < dev->num_clks; i++)
			clk_disable(dev->clk[i]);
		if (dev->dev.parent)
			nvhost_module_idle(to_nvhost_device(dev->dev.parent));
	} else if (dev->powerstate == NVHOST_POWER_STATE_POWERGATED
			&& dev->can_powergate) {
		do_unpowergate_locked(dev->powergate_ids[0]);
		do_unpowergate_locked(dev->powergate_ids[1]);

		if (dev->powerup_reset)
			do_module_reset_locked(dev);
	}
	dev->powerstate = NVHOST_POWER_STATE_CLOCKGATED;
}

static void to_state_running_locked(struct nvhost_device *dev)
{
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);
	int prev_state = dev->powerstate;

	if (dev->powerstate == NVHOST_POWER_STATE_POWERGATED)
		to_state_clockgated_locked(dev);

	if (dev->powerstate == NVHOST_POWER_STATE_CLOCKGATED) {
		int i;

		if (dev->dev.parent)
			nvhost_module_busy(to_nvhost_device(dev->dev.parent));

		for (i = 0; i < dev->num_clks; i++) {
			int err = clk_enable(dev->clk[i]);
			if (err) {
				dev_err(&dev->dev, "Cannot turn on clock %s",
					dev->clocks[i].name);
				return;
			}
		}

		/* Invoke callback after enabling clock. This is used for
		 * re-enabling host1x interrupts. */
		if (prev_state == NVHOST_POWER_STATE_CLOCKGATED
				&& drv->finalize_clockon)
			drv->finalize_clockon(dev);

		/* Invoke callback after power un-gating. This is used for
		 * restoring context. */
		if (prev_state == NVHOST_POWER_STATE_POWERGATED
				&& drv->finalize_poweron)
			drv->finalize_poweron(dev);
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
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);

	if (drv->prepare_poweroff
			&& dev->powerstate != NVHOST_POWER_STATE_POWERGATED) {
		/* Clock needs to be on in prepare_poweroff */
		to_state_running_locked(dev);
		err = drv->prepare_poweroff(dev);
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
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);

	if (drv->busy)
		drv->busy(dev);

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
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);
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

		if (drv->idle)
			drv->idle(dev);
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
	unsigned long devfreq_rate, default_rate;

	if (!dev->clk[index])
		return -EINVAL;

	/* If devfreq is on, use that clock rate, otherwise default */
	devfreq_rate = dev->clocks[index].devfreq_rate;
	default_rate = devfreq_rate ?
		devfreq_rate : dev->clocks[index].default_rate;
	default_rate = clk_round_rate(dev->clk[index], default_rate);

	list_for_each_entry(m, &dev->client_list, node) {
		unsigned long r = m->rate[index];
		if (!r)
			r = default_rate;
		rate = max(r, rate);
	}
	if (!rate)
		rate = default_rate;

	return clk_set_rate(dev->clk[index], rate);
}

int nvhost_module_set_rate(struct nvhost_device *dev, void *priv,
		unsigned long rate, int index)
{
	struct nvhost_module_client *m;
	int ret = 0;

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &dev->client_list, node) {
		if (m->priv == priv)
			m->rate[index] = clk_round_rate(dev->clk[index], rate);
	}

	ret = nvhost_module_update_rate(dev, index);
	mutex_unlock(&client_list_lock);
	return ret;

}

int nvhost_module_add_client(struct nvhost_device *dev, void *priv)
{
	struct nvhost_module_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	INIT_LIST_HEAD(&client->node);
	client->priv = priv;

	mutex_lock(&client_list_lock);
	list_add_tail(&client->node, &dev->client_list);
	mutex_unlock(&client_list_lock);
	return 0;
}

void nvhost_module_remove_client(struct nvhost_device *dev, void *priv)
{
	int i;
	struct nvhost_module_client *m;
	int found = 0;

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &dev->client_list, node) {
		if (priv == m->priv) {
			list_del(&m->node);
			found = 1;
			break;
		}
	}
	if (found) {
		kfree(m);
		for (i = 0; i < dev->num_clks; i++)
			nvhost_module_update_rate(dev, i);
	}
	mutex_unlock(&client_list_lock);
}

static ssize_t refcount_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int ret;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr, \
			power_attr[NVHOST_POWER_SYSFS_ATTRIB_REFCOUNT]);
	struct nvhost_device *dev = power_attribute->ndev;

	mutex_lock(&dev->lock);
	ret = sprintf(buf, "%d\n", dev->refcount);
	mutex_unlock(&dev->lock);

	return ret;
}

static ssize_t powergate_delay_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int powergate_delay = 0, ret = 0;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr, \
			power_attr[NVHOST_POWER_SYSFS_ATTRIB_POWERGATE_DELAY]);
	struct nvhost_device *dev = power_attribute->ndev;

	if (!dev->can_powergate) {
		dev_info(&dev->dev, "does not support power-gating\n");
		return count;
	}

	mutex_lock(&dev->lock);
	ret = sscanf(buf, "%d", &powergate_delay);
	if (ret == 1 && powergate_delay >= 0)
		dev->powergate_delay = powergate_delay;
	else
		dev_err(&dev->dev, "Invalid powergate delay\n");
	mutex_unlock(&dev->lock);

	return count;
}

static ssize_t powergate_delay_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int ret;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr, \
			power_attr[NVHOST_POWER_SYSFS_ATTRIB_POWERGATE_DELAY]);
	struct nvhost_device *dev = power_attribute->ndev;

	mutex_lock(&dev->lock);
	ret = sprintf(buf, "%d\n", dev->powergate_delay);
	mutex_unlock(&dev->lock);

	return ret;
}

static ssize_t clockgate_delay_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int clockgate_delay = 0, ret = 0;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr, \
			power_attr[NVHOST_POWER_SYSFS_ATTRIB_CLOCKGATE_DELAY]);
	struct nvhost_device *dev = power_attribute->ndev;

	mutex_lock(&dev->lock);
	ret = sscanf(buf, "%d", &clockgate_delay);
	if (ret == 1 && clockgate_delay >= 0)
		dev->clockgate_delay = clockgate_delay;
	else
		dev_err(&dev->dev, "Invalid clockgate delay\n");
	mutex_unlock(&dev->lock);

	return count;
}

static ssize_t clockgate_delay_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int ret;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr, \
			power_attr[NVHOST_POWER_SYSFS_ATTRIB_CLOCKGATE_DELAY]);
	struct nvhost_device *dev = power_attribute->ndev;

	mutex_lock(&dev->lock);
	ret = sprintf(buf, "%d\n", dev->clockgate_delay);
	mutex_unlock(&dev->lock);

	return ret;
}

int nvhost_module_set_devfreq_rate(struct nvhost_device *dev, int index,
		unsigned long rate)
{
	rate = clk_round_rate(dev->clk[index], rate);
	dev->clocks[index].devfreq_rate = rate;

	return nvhost_module_update_rate(dev, index);
}

int nvhost_module_init(struct nvhost_device *dev)
{
	int i = 0, err = 0;
	struct kobj_attribute *attr = NULL;

	/* initialize clocks to known state */
	INIT_LIST_HEAD(&dev->client_list);
	while (dev->clocks[i].name && i < NVHOST_MODULE_MAX_CLOCKS) {
		char devname[MAX_DEVID_LENGTH];
		long rate = dev->clocks[i].default_rate;
		struct clk *c;

		snprintf(devname, MAX_DEVID_LENGTH, "tegra_%s", dev->name);
		c = clk_get_sys(devname, dev->clocks[i].name);
		if (IS_ERR_OR_NULL(c)) {
			dev_err(&dev->dev, "Cannot get clock %s\n",
					dev->clocks[i].name);
			continue;
		}

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

	/* Init the power sysfs attributes for this device */
	dev->power_attrib = kzalloc(sizeof(struct nvhost_device_power_attr),
		GFP_KERNEL);
	if (!dev->power_attrib) {
		dev_err(&dev->dev, "Unable to allocate sysfs attributes\n");
		return -ENOMEM;
	}
	dev->power_attrib->ndev = dev;

	dev->power_kobj = kobject_create_and_add("acm", &dev->dev.kobj);
	if (!dev->power_kobj) {
		dev_err(&dev->dev, "Could not add dir 'power'\n");
		err = -EIO;
		goto fail_attrib_alloc;
	}

	attr = &dev->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_CLOCKGATE_DELAY];
	attr->attr.name = "clockgate_delay";
	attr->attr.mode = S_IWUSR | S_IRUGO;
	attr->show = clockgate_delay_show;
	attr->store = clockgate_delay_store;
	if (sysfs_create_file(dev->power_kobj, &attr->attr)) {
		dev_err(&dev->dev, "Could not create sysfs attribute clockgate_delay\n");
		err = -EIO;
		goto fail_clockdelay;
	}

	attr = &dev->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_POWERGATE_DELAY];
	attr->attr.name = "powergate_delay";
	attr->attr.mode = S_IWUSR | S_IRUGO;
	attr->show = powergate_delay_show;
	attr->store = powergate_delay_store;
	if (sysfs_create_file(dev->power_kobj, &attr->attr)) {
		dev_err(&dev->dev, "Could not create sysfs attribute powergate_delay\n");
		err = -EIO;
		goto fail_powergatedelay;
	}

	attr = &dev->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_REFCOUNT];
	attr->attr.name = "refcount";
	attr->attr.mode = S_IRUGO;
	attr->show = refcount_show;
	if (sysfs_create_file(dev->power_kobj, &attr->attr)) {
		dev_err(&dev->dev, "Could not create sysfs attribute refcount\n");
		err = -EIO;
		goto fail_refcount;
	}

	return 0;

fail_refcount:
	attr = &dev->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_POWERGATE_DELAY];
	sysfs_remove_file(dev->power_kobj, &attr->attr);

fail_powergatedelay:
	attr = &dev->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_CLOCKGATE_DELAY];
	sysfs_remove_file(dev->power_kobj, &attr->attr);

fail_clockdelay:
	kobject_put(dev->power_kobj);

fail_attrib_alloc:
	kfree(dev->power_attrib);

	return err;
}

static int is_module_idle(struct nvhost_device *dev)
{
	int count;
	mutex_lock(&dev->lock);
	count = dev->refcount;
	mutex_unlock(&dev->lock);
	return (count == 0);
}

int nvhost_module_suspend(struct nvhost_device *dev)
{
	int ret;
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);

	ret = wait_event_timeout(dev->idle_wq, is_module_idle(dev),
			ACM_SUSPEND_WAIT_FOR_IDLE_TIMEOUT);
	if (ret == 0) {
		dev_info(&dev->dev, "%s prevented suspend\n",
				dev->name);
		return -EBUSY;
	}

	mutex_lock(&dev->lock);
	cancel_delayed_work(&dev->powerstate_down);
	to_state_powergated_locked(dev);
	mutex_unlock(&dev->lock);

	if (drv->suspend_ndev)
		drv->suspend_ndev(dev);

	return 0;
}

void nvhost_module_deinit(struct nvhost_device *dev)
{
	int i;
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);

	if (drv->deinit)
		drv->deinit(dev);

	nvhost_module_suspend(dev);
	for (i = 0; i < dev->num_clks; i++)
		clk_put(dev->clk[i]);
	dev->powerstate = NVHOST_POWER_STATE_DEINIT;
}

/* public host1x power management APIs */
bool nvhost_module_powered_ext(struct nvhost_device *dev)
{
	return nvhost_module_powered(dev);
}

void nvhost_module_busy_ext(struct nvhost_device *dev)
{
	nvhost_module_busy(dev);
}

void nvhost_module_idle_ext(struct nvhost_device *dev)
{
	nvhost_module_idle(dev);
}
