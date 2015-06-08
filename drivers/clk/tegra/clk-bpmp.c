/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <soc/tegra/tegra_bpmp.h>

#include "clk.h"
#include "clk-mrq.h"

struct bpmp_clk_req {
	u32	cmd;
	u8	args[0];
};

#define BPMP_CLK_CMD(cmd, id) ((id) | ((cmd) << 24))
#define MAX_PARENTS 16

struct possible_parents {
	u8	num_of_parents;
	s32	clk_ids[MAX_PARENTS];
};

static struct clk **clks;
static struct clk_onecell_data clk_data;

static int bpmp_send_clk_message(struct bpmp_clk_req *req, int size,
				 u8 *reply, int reply_size)
{
	int err;

	err = tegra_bpmp_send_receive_atomic(MRQ_CLK, req, size, reply,
					     reply_size);

	return err;
}

static int clk_bpmp_enable(struct clk_hw *hw)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	struct bpmp_clk_req req;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_ENABLE, bpmp_clk->clk_num);

	return bpmp_send_clk_message(&req, sizeof(req), NULL, 0);
}

static void clk_bpmp_disable(struct clk_hw *hw)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	struct bpmp_clk_req req;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_DISABLE, bpmp_clk->clk_num);

	bpmp_send_clk_message(&req, sizeof(req), NULL, 0);
}

static int clk_bpmp_is_enabled(struct clk_hw *hw)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	struct bpmp_clk_req req;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_IS_ENABLED, bpmp_clk->clk_num);

	return bpmp_send_clk_message(&req, sizeof(req), NULL, 0);
}

static int clk_bpmp_get_parent_num(int clk_num)
{
	int err;
	u8 reply[4];
	struct bpmp_clk_req req;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_GET_PARENT, clk_num);
	err = bpmp_send_clk_message(&req, sizeof(req), reply, sizeof(reply));
	if (err < 0)
		return err;

	return ((s32 *)reply)[0];
}

static u8 clk_bpmp_get_parent(struct clk_hw *hw)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	int parent_id, i;

	parent_id = bpmp_clk->parent;

	if (parent_id < 0)
		goto err_out;

	for (i = 0; i < bpmp_clk->num_parents; i++) {
		if (bpmp_clk->parent_ids[i] == parent_id)
			return i;
	}

err_out:
	pr_err("clk_bpmp_get_parent for %s parent_id: %d, num_parents: %d\n",
		__clk_get_name(hw->clk), parent_id, bpmp_clk->num_parents);
	WARN_ON(1);

	return 0;
}

static int clk_bpmp_set_parent(struct clk_hw *hw, u8 index)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	u8 req_d[12], reply[4];
	struct bpmp_clk_req *req  = (struct bpmp_clk_req *)&req_d[0];
	int err;

	if (index > bpmp_clk->num_parents - 1)
		return -EINVAL;

	req->cmd = BPMP_CLK_CMD(MRQ_CLK_SET_PARENT, bpmp_clk->clk_num);
	*((u32 *)&req->args[0]) = bpmp_clk->parent_ids[index];

	err = bpmp_send_clk_message(req, sizeof(req_d), reply, sizeof(reply));
	if (!err)
		bpmp_clk->parent = bpmp_clk->parent_ids[index];

	return err;
}

static int clk_bpmp_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	u8 req_d[16], reply[8];
	struct bpmp_clk_req *req = (struct bpmp_clk_req *)&req_d[0];

	req->cmd = BPMP_CLK_CMD(MRQ_CLK_SET_RATE, bpmp_clk->clk_num);
	*((s64 *)&req->args[4]) = rate;

	return bpmp_send_clk_message(req, sizeof(req_d), reply, sizeof(reply));
}

static long clk_bpmp_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	int err;
	u8 req_d[16], reply[8];
	struct bpmp_clk_req *req = (struct bpmp_clk_req *)&req_d[0];

	req->cmd = BPMP_CLK_CMD(MRQ_CLK_ROUND_RATE, bpmp_clk->clk_num);
	*((s64 *)&req->args[4]) = rate;
	err = bpmp_send_clk_message(req, sizeof(req_d), reply, sizeof(reply));

	if (err < 0)
		return err;

	return ((s64 *)reply)[0];
}

static unsigned long clk_bpmp_get_rate_clk_num(int clk_num)
{
	u8 reply[8];
	struct bpmp_clk_req req;
	int err;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_GET_RATE, clk_num);
	err = bpmp_send_clk_message(&req, sizeof(req), reply, sizeof(reply));
	if (err < 0)
		return err;

	return ((s64 *)reply)[0];
}

static unsigned long clk_bpmp_get_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);

	return clk_bpmp_get_rate_clk_num(bpmp_clk->clk_num);
}

static int clk_bpmp_get_properties(int clk_num)
{
	struct bpmp_clk_req req;
	u8 reply[4];
	int err;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_PROPERTIES, clk_num);
	err = bpmp_send_clk_message(&req, sizeof(req), reply, sizeof(reply));
	if (err < 0)
		return err;

	return  ((int *)reply)[0];
}

const struct clk_ops tegra_clk_bpmp_gate_ops = {
	.is_enabled = clk_bpmp_is_enabled,
	.prepare = clk_bpmp_enable,
	.unprepare = clk_bpmp_disable,
};

const struct clk_ops tegra_clk_bpmp_mux_rate_ops = {
	.is_enabled = clk_bpmp_is_enabled,
	.prepare = clk_bpmp_enable,
	.unprepare = clk_bpmp_disable,
	.get_parent = clk_bpmp_get_parent,
	.set_parent = clk_bpmp_set_parent,
	.set_rate = clk_bpmp_set_rate,
	.round_rate = clk_bpmp_round_rate,
	.recalc_rate = clk_bpmp_get_rate,
};

const struct clk_ops tegra_clk_bpmp_rate_ops = {
	.is_enabled = clk_bpmp_is_enabled,
	.prepare = clk_bpmp_enable,
	.unprepare = clk_bpmp_disable,
	.set_rate = clk_bpmp_set_rate,
	.round_rate = clk_bpmp_round_rate,
	.recalc_rate = clk_bpmp_get_rate,
};

const struct clk_ops  tegra_clk_bpmp_mux_ops = {
	.get_parent = clk_bpmp_get_parent,
	.set_parent = clk_bpmp_set_parent,
};

struct clk *tegra_clk_register_bpmp(const char *name, int parent,
		const char **parent_names, int *parent_ids, int num_parents,
		unsigned long flags, int clk_num, int bpmp_flags)
{
	struct tegra_clk_bpmp *bpmp_clk;
	struct clk *clk;
	struct clk_init_data init;

	if (num_parents > 1)
		bpmp_clk = kzalloc(sizeof(*bpmp_clk) + num_parents
				   * sizeof(int), GFP_KERNEL);
	else
		bpmp_clk = kzalloc(sizeof(*bpmp_clk), GFP_KERNEL);

	if (!bpmp_clk) {
		pr_err("%s: unable to allocate clock %s\n", __func__, name);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.flags = flags;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	if ((bpmp_flags & (BPMP_CLK_HAS_MUX | BPMP_CLK_HAS_SET_RATE))
		== (BPMP_CLK_HAS_MUX | BPMP_CLK_HAS_SET_RATE))
		init.ops = &tegra_clk_bpmp_mux_rate_ops;
	else if (bpmp_flags & BPMP_CLK_HAS_SET_RATE)
		init.ops = &tegra_clk_bpmp_rate_ops;
	else if (bpmp_flags & BPMP_CLK_HAS_MUX)
		init.ops = &tegra_clk_bpmp_mux_ops;
	else
		init.ops = &tegra_clk_bpmp_gate_ops;

	/* Data in .init is copied by clk_register(), so stack variable OK */
	bpmp_clk->clk_num = clk_num;
	bpmp_clk->hw.init = &init;
	bpmp_clk->num_parents = num_parents;
	bpmp_clk->parent = parent;

	if (num_parents > 1)
		memcpy(&bpmp_clk->parent_ids[0], parent_ids,
		       num_parents * sizeof(int));

	clk = clk_register(NULL, &bpmp_clk->hw);
	if (IS_ERR(clk)) {
		pr_err("registration failed for clock %s\n", name);
		kfree(bpmp_clk);
	}

	return clk;
}

static char *clk_bpmp_lookup_name(int clk_num,
			struct tegra_bpmp_clk_init *init_clks, int num_clks)
{
	int i;

	for (i = 0; i < num_clks; i++) {
		if (init_clks[i].clk_num == clk_num)
			return init_clks[i].name;
	}

	return NULL;
}

static int clk_bpmp_get_possible_parents(int clk_num,
					 struct possible_parents *p)
{
	struct bpmp_clk_req req;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_POSSIBLE_PARENTS, clk_num);

	return bpmp_send_clk_message(&req, sizeof(req), (u8 *)p, sizeof(*p));
}

struct clk **tegra_bpmp_clk_init(struct tegra_bpmp_clk_init *init_clks,
				 int num_clks, struct device_node *np)
{
	int max_clk_id = 0;
	int i, j;
	struct clk *clk;
	static const char *parent_names[MAX_PARENTS];
	struct possible_parents parents;

	/* find highest clock id */
	for (i = 0; i < num_clks; i++) {
		if (init_clks[i].clk_num > max_clk_id)
			max_clk_id = init_clks[i].clk_num;
	}

	clks = kmalloc((max_clk_id + 1) * sizeof(struct clk *), GFP_KERNEL);
	if (!clks) {
		WARN_ON(1);
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < max_clk_id + 1; i++)
		clks[i] = ERR_PTR(-EINVAL);

	for (i = 0; i < num_clks; i++) {
		struct tegra_bpmp_clk_init *clk_init = init_clks + i;
		int flags, parent, num_parents, clk_num;
		unsigned long rate;

		clk_num = clk_init->clk_num;

		flags = clk_bpmp_get_properties(clk_num);
		if (flags < 0)
			continue;

		if (flags & BPMP_CLK_IS_ROOT)
			parent = 0;
		else {
			parent = clk_bpmp_get_parent_num(clk_num);
			if (parent < 0)
				continue;
		}

		if (flags & BPMP_CLK_HAS_MUX) {
			int err = clk_bpmp_get_possible_parents(clk_num,
								&parents);
			if (err < 0) {
				pr_err("bpmp returned %d for clk %d possible parents\n",
					err, clk_num);
				continue;
			}

			num_parents = parents.num_of_parents;
			for (j = 0; j < num_parents; j++) {
				parent_names[j] =
				clk_bpmp_lookup_name(parents.clk_ids[j],
						     init_clks, num_clks);
				if (!parent_names[j])
					pr_err("%s: no name for parent %d\n",
						clk_init->name,
						parents.clk_ids[j]);
			}
		} else {
			num_parents = 1;
			parent_names[0]  = clk_bpmp_lookup_name(parent,
						 init_clks, num_clks);
		}

		if (flags & BPMP_CLK_IS_ROOT) {
			rate = clk_bpmp_get_rate_clk_num(clk_num);
			clk = clk_register_fixed_rate(NULL, clk_init->name,
						 NULL, CLK_IS_ROOT, rate);
		} else if (num_parents == 1) {
			clk = tegra_clk_register_bpmp(clk_init->name, parent,
						parent_names, NULL,
						num_parents, 0,
						clk_num, flags);
		} else {
			clk = tegra_clk_register_bpmp(clk_init->name, parent,
					parent_names, &parents.clk_ids[0],
					num_parents, 0, clk_num, flags);
		}

		clks[clk_num] = clk;
	}

	clk_data.clks = clks;
	clk_data.clk_num = max_clk_id + 1;
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);

	return clks;
}
