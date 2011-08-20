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
#include <linux/debugfs.h>
#include "nvhost_scale.h"

#define ACM_POWERDOWN_HANDLER_DELAY_MSEC  25
#define ACM_SUSPEND_WAIT_FOR_IDLE_TIMEOUT (2 * HZ)
#define POWERGATE_DELAY 10

#define HOST_EMC_FLOOR 300000000

void nvhost_module_reset(struct nvhost_module *mod)
{
	struct nvhost_master *dev;
	dev = container_of(mod, struct nvhost_channel, mod)->dev;

	dev_dbg(&dev->pdev->dev,
		"%s: asserting %s module reset (id %d, id2 %d)\n",
		__func__, mod->name,
		mod->powergate_id, mod->powergate_id2);

	/* assert module and mc client reset */
	if (mod->powergate_id != -1) {
		tegra_powergate_mc_disable(mod->powergate_id);
		tegra_periph_reset_assert(mod->clk[0]);
		tegra_powergate_mc_flush(mod->powergate_id);
	}
	if (mod->powergate_id2 != -1) {
		tegra_powergate_mc_disable(mod->powergate_id2);
		tegra_periph_reset_assert(mod->clk[1]);
		tegra_powergate_mc_flush(mod->powergate_id2);
	}

	udelay(POWERGATE_DELAY);

	/* deassert reset */
	if (mod->powergate_id != -1) {
		tegra_powergate_mc_flush_done(mod->powergate_id);
		tegra_periph_reset_deassert(mod->clk[0]);
		tegra_powergate_mc_enable(mod->powergate_id);
	}
	if (mod->powergate_id2 != -1) {
		tegra_powergate_mc_flush_done(mod->powergate_id2);
		tegra_periph_reset_deassert(mod->clk[1]);
		tegra_powergate_mc_enable(mod->powergate_id2);
	}

	dev_dbg(&dev->pdev->dev, "%s: module %s out of reset\n",
		__func__, mod->name);
}

void nvhost_module_busy(struct nvhost_module *mod)
{
	mutex_lock(&mod->lock);
	cancel_delayed_work(&mod->powerdown);
	if ((atomic_inc_return(&mod->refcount) == 1) && !mod->powered) {
		int i = 0;
		if (mod->parent)
			nvhost_module_busy(mod->parent);
		if (mod->can_powergate) {
			if (mod->powergate_id != -1)
				tegra_unpowergate_partition(mod->powergate_id);
			if (mod->powergate_id2 != -1)
				tegra_unpowergate_partition(mod->powergate_id2);
		}
		while (i < mod->num_clks)
			clk_enable(mod->clk[i++]);
		if (mod->can_powergate && mod->func)
			mod->func(mod, NVHOST_POWER_ACTION_ON);
		mod->powered = true;
	}
	mutex_unlock(&mod->lock);
}

static void powerdown_handler(struct work_struct *work)
{
	struct nvhost_module *mod;

	mod = container_of(to_delayed_work(work), struct nvhost_module, powerdown);
	mutex_lock(&mod->lock);
	if ((atomic_read(&mod->refcount) == 0) && mod->powered) {
		int i;
		if (mod->func)
			mod->func(mod, NVHOST_POWER_ACTION_OFF);
		for (i = 0; i < mod->num_clks; i++)
			clk_disable(mod->clk[i]);
		if (mod->can_powergate) {
			if (mod->powergate_id != -1)
				tegra_powergate_partition(mod->powergate_id);
			if (mod->powergate_id2 != -1)
				tegra_powergate_partition(mod->powergate_id2);
		}
		mod->powered = false;
		if (mod->parent)
			nvhost_module_idle(mod->parent);
	}
	mutex_unlock(&mod->lock);
}

/*
 * 3d clock scaling
 *
 * module3d_notify_busy() is called upon submit, module3d_notify_idle() is
 * called when all outstanding submits are completed. Idle times are measured
 * over a fixed time period (scale3d.p_period). If the 3d module idle time
 * percentage goes over the limit (set in scale3d.p_idle_max), 3d clocks are
 * scaled down. If the percentage goes under the minimum limit (set in
 * scale3d.p_idle_min), 3d clocks are scaled up. An additional test is made
 * over the time frame given in scale3d.p_fast_response for clocking up
 * quickly in response to sudden load peaks.
 */
static struct scale3d_info_rec scale3d;

static void scale_3d_clocks(unsigned long percent)
{
	unsigned long hz, curr;

	if (!tegra_is_clk_enabled(scale3d.clk_3d))
		return;

	if (tegra_chip_id == TEGRA30)
		if (!tegra_is_clk_enabled(scale3d.clk_3d2))
			return;

	curr = clk_get_rate(scale3d.clk_3d);
	hz = percent * (curr / 100);

	if (tegra_chip_id == TEGRA30)
		clk_set_rate(scale3d.clk_3d2, 0);
	clk_set_rate(scale3d.clk_3d, hz);
}

static void scale_3d_clocks_handler(struct work_struct *work)
{
	unsigned int scale;

	spin_lock(&scale3d.lock);
	scale = scale3d.scale;
	spin_unlock(&scale3d.lock);

	if (scale != 0) {
		mutex_lock(&scale3d.set_lock);
		scale_3d_clocks(scale);
		mutex_unlock(&scale3d.set_lock);
	}
}

static void scale3d_init(struct nvhost_module *mod)
{
	spin_lock_init(&scale3d.lock);
	mutex_init(&scale3d.set_lock);

	scale3d.clk_3d = mod->clk[0];
	if (tegra_chip_id == TEGRA30)
		scale3d.clk_3d2 = mod->clk[1];

	INIT_WORK(&scale3d.work, scale_3d_clocks_handler);

	/* set scaling parameter defaults */
	scale3d.enable = 1;
	scale3d.p_period = 1200000;
	scale3d.p_idle_min = 17;
	scale3d.p_idle_max = 17;
	scale3d.p_fast_response = 16000;
	scale3d.p_verbosity = 0;

	scale3d_reset();

	scale3d.init = 1;
}

/* set 3d clocks to max */
static void reset_3d_clocks(void)
{
	unsigned long hz;

	mutex_lock(&scale3d.set_lock);
	hz = clk_round_rate(scale3d.clk_3d, UINT_MAX);
	clk_set_rate(scale3d.clk_3d, hz);
	if (tegra_chip_id == TEGRA30)
		clk_set_rate(scale3d.clk_3d2, hz);
	mutex_unlock(&scale3d.set_lock);
}

int scale3d_is_enabled(void)
{
	int enable;

	spin_lock(&scale3d.lock);
	enable = scale3d.enable;
	spin_unlock(&scale3d.lock);

	return enable;
}

void scale3d_enable(int enable)
{
	int disable = 0;

	spin_lock(&scale3d.lock);

	if (enable)
		scale3d.enable = 1;
	else {
		scale3d.enable = 0;
		disable = 1;
	}

	spin_unlock(&scale3d.lock);

	if (disable)
		reset_3d_clocks();
}

static void reset_scaling_counters(ktime_t time)
{
	scale3d.idle_total = 0;
	scale3d.last_idle = time;
	scale3d.last_busy = time;
	scale3d.idle_frame = time;
}

static void scaling_state_check(ktime_t time)
{
	unsigned long dt;

	/* check for load peaks */
	dt = (unsigned long) ktime_us_delta(time, scale3d.fast_frame);
	if (dt > scale3d.p_fast_response) {
		unsigned long idleness = (scale3d.idle_total * 100) / dt;
		scale3d.fast_frame = time;
		/* if too busy, scale up */
		if (idleness < scale3d.p_idle_min) {
			if (scale3d.p_verbosity > 5)
				pr_info("scale3d: %ld%% busy\n",
					100 - idleness);

			scale3d.scale = 200;
			schedule_work(&scale3d.work);
			reset_scaling_counters(time);
			return;
		}
	}

	dt = (unsigned long) ktime_us_delta(time, scale3d.idle_frame);
	if (dt > scale3d.p_period) {
		unsigned long idleness = (scale3d.idle_total * 100) / dt;

		if (scale3d.p_verbosity > 5)
			pr_info("scale3d: idle %lu, ~%lu%%\n",
				scale3d.idle_total, idleness);

		if (idleness > scale3d.p_idle_max) {
			/* if idle time is high, clock down */
			scale3d.scale = 100 - (idleness - scale3d.p_idle_min);
			schedule_work(&scale3d.work);
		} else if (idleness < scale3d.p_idle_min) {
			/* if idle time is low, clock up */
			scale3d.scale = 200;
			schedule_work(&scale3d.work);
		}
		reset_scaling_counters(time);
	}
}

static void module3d_notify_idle(void)
{
	spin_lock(&scale3d.lock);

	if (!scale3d.enable)
		goto done;

	scale3d.last_idle = ktime_get();
	scale3d.is_idle = 1;

	scaling_state_check(scale3d.last_idle);

done:
	spin_unlock(&scale3d.lock);
}

void module3d_notify_busy(void)
{
	unsigned long idle;
	ktime_t t;

	spin_lock(&scale3d.lock);

	if (!scale3d.enable)
		goto done;

	t = ktime_get();

	if (scale3d.is_idle) {
		scale3d.last_busy = t;
		idle = (unsigned long)
			ktime_us_delta(scale3d.last_busy, scale3d.last_idle);
		scale3d.idle_total += idle;
		scale3d.is_idle = 0;
	}

	scaling_state_check(t);

done:
	spin_unlock(&scale3d.lock);
}

void scale3d_reset()
{
	ktime_t t = ktime_get();
	spin_lock(&scale3d.lock);
	reset_scaling_counters(t);
	spin_unlock(&scale3d.lock);
}

void nvhost_module_idle_mult(struct nvhost_module *mod, int refs)
{
	bool kick = false;

	mutex_lock(&mod->lock);
	if (atomic_sub_return(refs, &mod->refcount) == 0) {
		BUG_ON(!mod->powered);
		schedule_delayed_work(&mod->powerdown,
			msecs_to_jiffies(mod->powerdown_delay));
		kick = true;
	}
	mutex_unlock(&mod->lock);

	if (kick) {
		wake_up(&mod->idle);

		if (strcmp(mod->name, "gr3d") == 0)
			module3d_notify_idle();
	}
}

static const char *get_module_clk_id(const char *module, int index)
{
	if (index == 0)
		return module;
	if (strcmp(module, "gr2d") == 0) {
		if (index == 1)
			return "epp";
		if (index == 2)
			return "emc";
	}
	if (strcmp(module, "gr3d") == 0) {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		if (index == 1)
			return "emc";
#else
		if (index == 1)
			return "gr3d2";
		if (index == 2)
			return "emc";
#endif
	}
	if (strcmp(module, "mpe") == 0) {
		if (index == 1)
			return "emc";
	}
	return NULL;
}

/* 3D power gating disabled as it causes syncpt hangs */
static bool _3d_powergating_disabled(void)
{
	return 1;
}

/* MPE power gating disabled as it causes syncpt hangs */
static bool _mpe_powergating_disabled(void)
{
	return 1;
}
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
int nvhost_module_get_rate(struct nvhost_module *mod, unsigned long *rate,
			    int index)
{
	struct clk *c;

	c = mod->clk[index];
	if (IS_ERR_OR_NULL(c))
		return -EINVAL;

	*rate = clk_get_rate(c);
	return 0;
}

int nvhost_module_update_rate(struct nvhost_module *mod, int index)
{
	unsigned long rate = mod->min_rate[index];
	struct nvhost_module_client *m;

	list_for_each_entry(m, &mod->client_list, node) {
		rate = max(m->rate[index], rate);
	}
	if (!mod->clk[index])
		return -EINVAL;
	clk_set_rate(mod->clk[index], rate);
	return 0;
}

int nvhost_module_set_rate(struct nvhost_module *mod, void *priv,
			    unsigned long rate, int index)
{
	struct nvhost_module_client *m;

	list_for_each_entry(m, &mod->client_list, node) {
		if (m->priv == priv) {
			rate = clk_round_rate(mod->clk[index], rate);
			m->rate[index] = rate;
			break;
		}
	}
	return nvhost_module_update_rate(mod, index);
}

int nvhost_module_add_client(struct nvhost_module *mod, void *priv)
{
	int i;
	unsigned long rate;
	struct nvhost_module_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&client->node);
	client->priv = priv;

	for (i = 0; i < mod->num_clks; i++) {
		rate = clk_round_rate(mod->clk[i], 0);
		client->rate[i] = rate;
	}
	list_add_tail(&client->node, &mod->client_list);
	return 0;
}

void nvhost_module_remove_client(struct nvhost_module *mod, void *priv)
{
	int i;
	struct nvhost_module_client *m;

	list_for_each_entry(m, &mod->client_list, node) {
		if (priv == m->priv) {
			list_del(&m->node);
			break;
		}
	}
	m->priv = NULL;
	kfree(m);
	for (i = 0; i < mod->num_clks; i++)
		nvhost_module_update_rate(mod, i);
}
#else
int nvhost_module_get_rate(struct nvhost_module *mod, unsigned long *rate,
                            int index)
{
        return 0;
}

int nvhost_module_set_rate(struct nvhost_module *mod, void *priv,
                            unsigned long rate, int index)
{
        return 0;
}

int nvhost_module_add_client(struct nvhost_module *mod, void *priv)
{
        return 0;
}

void nvhost_module_remove_client(struct nvhost_module *mod, void *priv)
{
}
#endif

int nvhost_module_init(struct nvhost_module *mod, const char *name,
		nvhost_modulef func, struct nvhost_module *parent,
		struct device *dev)
{
	int i = 0;

	mod->name = name;

	INIT_LIST_HEAD(&mod->client_list);
	while (i < NVHOST_MODULE_MAX_CLOCKS) {
		long rate;
		const char *clk_name = get_module_clk_id(name, i);
		mod->min_rate[i] = 0;
		mod->clk[i] = clk_get(dev, clk_name);
		if (IS_ERR_OR_NULL(mod->clk[i]))
			break;

		if (strcmp(clk_name, "emc") == 0
				&& tegra_chip_id != TEGRA20) {
			rate = clk_round_rate(mod->clk[i], HOST_EMC_FLOOR);
			if (!IS_ERR_VALUE(rate))
				mod->min_rate[i] = rate;
		} else if (strcmp(name, "gr2d") == 0
				&& tegra_chip_id != TEGRA20)
			rate = clk_round_rate(mod->clk[i], 0);
		else
			rate = clk_round_rate(mod->clk[i], UINT_MAX);
		if (rate < 0) {
			pr_err("%s: can't get maximum rate for %s\n",
				__func__, name);
			break;
		}
		clk_enable(mod->clk[i]);
		clk_set_rate(mod->clk[i], rate);
		clk_disable(mod->clk[i]);
		i++;
	}

	mod->num_clks = i;
	mod->func = func;
	mod->parent = parent;
	mod->can_powergate = false;
	mod->powered = false;
	mod->powergate_id = -1;
	mod->powergate_id2 = -1;
	mod->powerdown_delay = ACM_POWERDOWN_HANDLER_DELAY_MSEC;

	if (strcmp(name, "gr2d") == 0)
		mod->powerdown_delay = 0;
	else if (strcmp(name, "gr3d") == 0) {
		mod->can_powergate = !_3d_powergating_disabled();
		if (!scale3d.init)
			scale3d_init(mod);
		mod->powergate_id = TEGRA_POWERGATE_3D;
		if (!mod->can_powergate)
			tegra_unpowergate_partition(mod->powergate_id);
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
		mod->powergate_id2 = TEGRA_POWERGATE_3D1;
		if (!mod->can_powergate)
			tegra_unpowergate_partition(mod->powergate_id2);
#endif
	} else if (strcmp(name, "mpe") == 0) {
		mod->can_powergate = true;
		mod->powergate_id = TEGRA_POWERGATE_MPE;
	}

	if (mod->powergate_id == TEGRA_POWERGATE_MPE
		&& _mpe_powergating_disabled()) {
		tegra_unpowergate_partition(mod->powergate_id);
		mod->powergate_id = -1;
	}

	if (mod->powergate_id == TEGRA_POWERGATE_3D
		&& _3d_powergating_disabled()) {
		tegra_unpowergate_partition(mod->powergate_id);
		mod->powergate_id = -1;

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
		if (mod->powergate_id2 == TEGRA_POWERGATE_3D1) {
			tegra_unpowergate_partition(mod->powergate_id2);
			mod->powergate_id2 = -1;
		}
#endif
	}

	mutex_init(&mod->lock);
	init_waitqueue_head(&mod->idle);
	INIT_DELAYED_WORK(&mod->powerdown, powerdown_handler);

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
			printk("tegra_grhost: %s: refcnt %d\n",
				m->name, atomic_read(&m->refcount));
	}

	for (i = 0; i < dev->nb_mlocks; i++) {
		int c = atomic_read(&dev->cpuaccess.lock_counts[i]);
		if (c) {
			printk("tegra_grhost: lock id %d: refcnt %d\n", i, c);
			lock_released = false;
		}
	}
	if (lock_released)
		printk("tegra_grhost: all locks released\n");
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
		printk("tegra_grhost: entered idle\n");

	flush_delayed_work(&mod->powerdown);
	cancel_work_sync(&scale3d.work);

	if (system_suspend)
		printk("tegra_grhost: flushed delayed work\n");
	BUG_ON(mod->powered);
}

void nvhost_module_deinit(struct nvhost_module *mod)
{
	int i;

	nvhost_module_suspend(mod, false);
	for (i = 0; i < mod->num_clks; i++)
		clk_put(mod->clk[i]);
}


/*
 * debugfs parameters to control 3d clock scaling
 */

void nvhost_debug_scale_init(struct dentry *de)
{
	struct dentry *d, *f;

	d = debugfs_create_dir("scaling", de);
	if (!d) {
		pr_err("scale3d: can\'t create debugfs directory\n");
		return;
	}

#define CREATE_SCALE3D_FILE(fname) \
	do {\
		f = debugfs_create_u32(#fname, S_IRUGO | S_IWUSR, d,\
			&scale3d.p_##fname);\
		if (NULL == f) {\
			pr_err("scale3d: can\'t create file " #fname "\n");\
			return;\
		} \
	} while (0)

	CREATE_SCALE3D_FILE(fast_response);
	CREATE_SCALE3D_FILE(idle_min);
	CREATE_SCALE3D_FILE(idle_max);
	CREATE_SCALE3D_FILE(period);
	CREATE_SCALE3D_FILE(verbosity);
#undef CREATE_SCALE3D_FILE
}
