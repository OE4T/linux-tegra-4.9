/*
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
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

#include "device-group.h"

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include "drivers/video/tegra/host/nvhost_acm.h"

static int get_grouped_device(struct camrtc_device_group *grp,
			struct device *dev, char const *name, int index)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_parse_phandle(dev->of_node, name, index);
	if (np == NULL)
		return 0;

	pdev = of_find_device_by_node(np);
	of_node_put(np);

	if (pdev == NULL) {
		dev_WARN(dev, "%s[%u] node has no device\n", name, index);
		return 0;
	}

	if (pdev->dev.driver == NULL) {
		dev_info(dev, "deferring, %s is not probed\n",
			dev_name(&pdev->dev));
		platform_device_put(pdev);
		return -EPROBE_DEFER;
	}

	grp->devices[index] = pdev;

	return 0;
}

static void camrtc_device_group_release(struct device *dev, void *res)
{
	const struct camrtc_device_group *grp = res;
	int i;

	for (i = 0; i < grp->ndevices; i++)
		platform_device_put(grp->devices[i]);
}

struct camrtc_device_group *camrtc_device_group_get(
	struct device *dev, char const *name)
{
	int index, err;
	struct camrtc_device_group *grp;
	int ndevices;

	if (!dev || !dev->of_node)
		return ERR_PTR(-EINVAL);

	ndevices = of_count_phandle_with_args(dev->of_node, name, NULL);
	if (ndevices <= 0)
		return ERR_PTR(-ENOENT);

	grp = devres_alloc(camrtc_device_group_release,
			offsetof(struct camrtc_device_group, devices[ndevices]),
			GFP_KERNEL);
	if (!grp)
		return ERR_PTR(-ENOMEM);

	grp->ndevices = ndevices;

	for (index = 0; index < grp->ndevices; index++) {
		err = get_grouped_device(grp, dev, name, index);
		if (err) {
			devres_free(grp);
			return ERR_PTR(err);
		}
	}

	devres_add(dev, grp);
	return grp;
}
EXPORT_SYMBOL(camrtc_device_group_get);

int camrtc_device_group_busy(const struct camrtc_device_group *grp)
{
	int err = -EINVAL, index, idle;

	if (!grp)
		return 0;

	if (IS_ERR(grp))
		return err;

	for (index = 0; index < grp->ndevices; index++) {
		if (!grp->devices[index])
			continue;

		err = nvhost_module_busy(grp->devices[index]);
		if (err < 0)
			goto error;
	}

	return 0;

error:
	for (idle = 0; idle < index; idle++)
		nvhost_module_idle(grp->devices[idle]);

	return err;
}
EXPORT_SYMBOL(camrtc_device_group_busy);

void camrtc_device_group_idle(const struct camrtc_device_group *grp)
{
	int index;

	if (IS_ERR_OR_NULL(grp))
		return;

	for (index = 0; index < grp->ndevices; index++)
		if (grp->devices[index])
			nvhost_module_idle(grp->devices[index]);
}
EXPORT_SYMBOL(camrtc_device_group_idle);

void camrtc_device_group_reset(const struct camrtc_device_group *grp)
{
	int index;

	if (!grp)
		return;

	if (IS_ERR(grp))
		return;

	for (index = 0; index < grp->ndevices; index++) {
		if (!grp->devices[index])
			continue;

		nvhost_module_reset(grp->devices[index], false);
	}
}
EXPORT_SYMBOL(camrtc_device_group_reset);
