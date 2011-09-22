/*
 * drivers/video/tegra/host/t30/acm_t30.c
 *
 * Tegra Graphics Host Power Management for Tegra3
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

#include <linux/slab.h>
#include <linux/err.h>
#include "../dev.h"
#include "../chip_support.h"
#include "../nvhost_acm.h"
#include "t30.h"
DEFINE_MUTEX(client_list_lock);

struct nvhost_module_client {
	struct list_head node;
	unsigned long rate[NVHOST_MODULE_MAX_CLOCKS];
	void *priv;
};

int t30_acm_get_rate(struct nvhost_module *mod, unsigned long *rate,
			    int index)
{
	struct clk *c;

	c = mod->clk[index];
	if (IS_ERR_OR_NULL(c))
		return -EINVAL;

	/* Need to enable client to get correct rate */
	nvhost_module_busy(mod);
	*rate = clk_get_rate(c);
	nvhost_module_idle(mod);
	return 0;
}

static int t30_acm_update_rate(struct nvhost_module *mod, int index)
{
	unsigned long rate = 0;
	struct nvhost_module_client *m;

	if (!mod->clk[index])
		return -EINVAL;

	list_for_each_entry(m, &mod->client_list, node) {
		rate = max(m->rate[index], rate);
	}
	if (!rate)
		rate = clk_round_rate(mod->clk[index],
				mod->desc->clocks[index].default_rate);

	return clk_set_rate(mod->clk[index], rate);
}

int t30_acm_set_rate(struct nvhost_module *mod, void *priv,
			    unsigned long rate, int index)
{
	struct nvhost_module_client *m;
	int ret;

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &mod->client_list, node) {
		if (m->priv == priv) {
			rate = clk_round_rate(mod->clk[index], rate);
			m->rate[index] = rate;
			break;
		}
	}
	ret = t30_acm_update_rate(mod, index);
	mutex_unlock(&client_list_lock);
	return ret;
}

int t30_acm_add_client(struct nvhost_module *mod, void *priv)
{
	int i;
	unsigned long rate;
	struct nvhost_module_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	INIT_LIST_HEAD(&client->node);
	client->priv = priv;

	for (i = 0; i < mod->num_clks; i++) {
		rate = clk_round_rate(mod->clk[i],
				mod->desc->clocks[i].default_rate);
		client->rate[i] = rate;
	}
	mutex_lock(&client_list_lock);
	list_add_tail(&client->node, &mod->client_list);
	mutex_unlock(&client_list_lock);
	return 0;
}

void t30_acm_remove_client(struct nvhost_module *mod, void *priv)
{
	int i;
	struct nvhost_module_client *m;

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &mod->client_list, node) {
		if (priv == m->priv) {
			list_del(&m->node);
			break;
		}
	}
	if (m) {
		kfree(m);
		for (i = 0; i < mod->num_clks; i++)
			t30_acm_update_rate(mod, i);
	}
	mutex_unlock(&client_list_lock);
}

int nvhost_init_t30_acm(struct nvhost_master *host)
{
	host->op.acm.get_rate = t30_acm_get_rate;
	host->op.acm.set_rate = t30_acm_set_rate;
	host->op.acm.add_client = t30_acm_add_client;
	host->op.acm.remove_client = t30_acm_remove_client;

	return 0;
}
