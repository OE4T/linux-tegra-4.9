/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Laxman Dewangan<ldewangan@nvidia.com>
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

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/of_fdt.h>

static struct property *__of_copy_property(const struct property *prop,
		gfp_t flags)
{
	struct property *propn;

	propn = kzalloc(sizeof(*prop), flags);
	if (propn == NULL)
		return NULL;

	propn->name = kstrdup(prop->name, flags);
	if (propn->name == NULL)
		goto err_fail_name;

	if (prop->length > 0) {
		propn->value = kmalloc(prop->length, flags);
		if (propn->value == NULL)
			goto err_fail_value;
		memcpy(propn->value, prop->value, prop->length);
		propn->length = prop->length;
	}
	return propn;

err_fail_value:
	kfree(propn->name);
err_fail_name:
	kfree(propn);
	return NULL;
}


static int __init update_target_node_from_overlay(
		struct device_node *target, struct device_node *overlay)
{
	struct property *prop;
	struct property *tprop;
	struct property *new_prop;
	int lenp = 0;
	int ret;

	for_each_property_of_node(overlay, prop) {
		/* Skip those we do not want to proceed */
		if (!strcmp(prop->name, "name") ||
			!strcmp(prop->name, "phandle") ||
			!strcmp(prop->name, "linux,phandle"))
				continue;
		tprop = of_find_property(target, prop->name, &lenp);
		if (!tprop) {
			ret = of_add_property(target, prop);
			if (ret < 0) {
				pr_err("Prop %s can not be added on node %s\n",
					prop->name, target->full_name);
				return ret;
			}
		} else {
			new_prop = __of_copy_property(prop, GFP_KERNEL);
			if (!new_prop) {
				pr_err("Prop %s can not be duplicated\n",
					prop->name);
				return -EINVAL;
			}
			ret = of_update_property(target, new_prop);
			if (ret < 0) {
				pr_err("Prop %s can not be updated on node %s\n",
					prop->name, target->full_name);
				return ret;
			}
		}
	}
	return 0;
}

static int __init parse_fragment(struct device_node *np)
{
	struct device_node *tchild, *ochild;
	struct device_node *board_np, *overlay, *target;
	const char *bname;
	struct property *prop;
	int board_count;
	bool found = false;
	int ret;

	overlay = of_get_child_by_name(np, "_overlay_");
	if (!overlay) {
		pr_err("Node %s does not have Overlay\n", np->name);
		return -EINVAL;
	}

	board_count = of_property_count_strings(np, "board-ids");
	if (!board_count) {
		pr_err("Node %s does not have property board-ids\n", np->name);
		return -EINVAL;
	}

	target = of_parse_phandle(np, "target", 0);
	if (!target) {
		pr_err("Node %s does not have targer node\n", np->name);
		return -EINVAL;
	}

	board_np = of_find_node_by_path("/chosen/board-id");
	if (!board_np) {
		pr_err("Chosen node does not have Board-id listed\n");
		return -EINVAL;
	}

	of_property_for_each_string(np, "board-ids", prop, bname) {
		found = of_property_read_bool(board_np, bname);
		if (found) {
			pr_info("node %s match with board %s\n",
				np->full_name, bname);
			break;
		}
	}
	if (!found)
		return 0;

	ret = update_target_node_from_overlay(target, overlay);
	if (ret < 0) {
		pr_err("Target %s update with overlay %s failed: %d\n",
			target->name, overlay->name, ret);
		return ret;
	}

	for_each_child_of_node(overlay, ochild) {
		tchild = of_get_child_by_name(target, ochild->name);
		if (!tchild) {
			pr_err("Overlay child %s not found on target %s\n",
				ochild->full_name, tchild->full_name);
			continue;
		}

		ret = update_target_node_from_overlay(tchild, ochild);
		if (ret < 0) {
			pr_err("Target %s update with overlay %s failed: %d\n",
				tchild->name, ochild->name, ret);
			return ret;
		}
	}
	return 0;
}

static int __init plugin_manager_init(void)
{
	struct device_node *root;
	struct device_node *pm_node;
	struct device_node *child;
	int ret;

	pr_info("Initializing plugin-manager\n");

	root = of_find_node_by_path("/");
	if (!root) {
		pr_info("Root node not found\n");
		return 0;
	}

	pm_node = of_find_node_by_name(root, "plugin-manager");
	if (!pm_node) {
		pr_info("Plugin-manager not available\n");
		return 0;
	}

	for_each_child_of_node(pm_node, child) {
		ret = parse_fragment(child);
		if (ret < 0)
			pr_err("Error in parsing node %s: %d\n",
				child->full_name, ret);
	}
	return 0;
}
core_initcall(plugin_manager_init);
