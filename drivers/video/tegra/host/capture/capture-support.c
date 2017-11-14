/*
 * Device driver for owning the separate Stream-ID used for GoS
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
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

#include "capture/capture-support.h"

#include <linux/device.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <soc/tegra/camrtc-capture.h>
#include <soc/tegra/chip-id.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_syncpt_unit_interface.h"
#include "t194/t194.h"

int t194_capture_alloc_syncpt(struct platform_device *pdev,
			const char *name,
			uint32_t *syncpt_id,
			dma_addr_t *syncpt_addr,
			uint32_t *gos_index,
			uint32_t *gos_offset)
{
	uint32_t id;
	uint32_t index = GOS_INDEX_INVALID;
	uint32_t offset = 0;
	dma_addr_t addr;
	int err = -ENODEV;

	id = nvhost_get_syncpt_client_managed(pdev, name);
	if (id == 0) {
		dev_err(&pdev->dev, "%s: syncpt allocation failed\n", __func__);
		return -ENODEV;
	}

	addr = nvhost_syncpt_address(pdev, id);

	err = nvhost_syncpt_get_gos(pdev, id, &index, &offset);
	if (err < 0) {
		if (!tegra_platform_is_sim())
			goto cleanup;

		dev_warn(&pdev->dev, "%s: GoS not supported on VDK\n",
			__func__);
	}

	*syncpt_id = id;
	*syncpt_addr = addr;
	*gos_index = index;
	*gos_offset = offset;

	dev_dbg(&pdev->dev,
		"%s: id=%u addr=0x%llx gos_index=%u gos_offset=%u\n",
		__func__, id, addr, index, offset);

	return 0;

cleanup:
	nvhost_syncpt_put_ref_ext(pdev, id);
	return err;
}
EXPORT_SYMBOL_GPL(t194_capture_alloc_syncpt);

void t194_capture_release_syncpt(struct platform_device *pdev, uint32_t id)
{
	dev_dbg(&pdev->dev, "%s: id=%u\n", __func__, id);
	nvhost_syncpt_put_ref_ext(pdev, id);
}
EXPORT_SYMBOL_GPL(t194_capture_release_syncpt);

void t194_capture_get_gos_table(struct platform_device *pdev,
				int *gos_count,
				const dma_addr_t **gos_table)
{
	int count = 0;
	dma_addr_t *table = NULL;

	/* Using information cached during the probe, this should never fail */
	(void)nvhost_syncpt_get_cv_dev_address_table(pdev, &count, &table);

	*gos_count = count;
	*gos_table = table;
}
EXPORT_SYMBOL_GPL(t194_capture_get_gos_table);

static int check_rce_rm(struct device *dev)
{
	struct device_node *rm_np;
	struct platform_device *rm = NULL;

	rm_np = of_parse_phandle(dev->of_node, "nvidia,rce-rm-device", 0);
	if (rm_np == NULL) {
		dev_warn(dev, "missing %s handle\n", "nvidia,rce-rm-device");
		return -ENODEV;
	}

	rm = of_find_device_by_node(rm_np);
	of_node_put(rm_np);

	if (rm == NULL)
		return -ENODEV;

	/* Make sure rce_rm is probed before trying to use GoS */

	if (rm->dev.driver == NULL) {
		dev_info(dev, "rce-rm not probed, deferring\n");
		platform_device_put(rm);
		return -EPROBE_DEFER;
	}

	platform_device_put(rm);

	return 0;
}

static int t194_capture_support_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *info;
	int err = 0;
	int gos_count;
	dma_addr_t *gos_table;

	info = (void *)of_device_get_match_data(dev);
	if (WARN_ON(info == NULL))
		return -ENODATA;

	err = check_rce_rm(dev);
	if (err)
		return err;

	info->pdev = pdev;
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		goto error;

	err = nvhost_module_init(pdev);
	if (err)
		goto error;

	err = nvhost_client_device_init(pdev);
	if (err)
		goto deinit;

	err = nvhost_syncpt_unit_interface_init(pdev);
	if (err)
		goto device_release;

	err = nvhost_syncpt_get_cv_dev_address_table(pdev,
						&gos_count, &gos_table);
	if (err) {
		if (!tegra_platform_is_sim()) {
			dev_err(&pdev->dev,
				"%s: failed to get GoS table: err=%d\n",
				__func__, err);
			goto device_release;
		}

		dev_warn(&pdev->dev, "%s: GoS not supported on VDK\n",
			__func__);
	}

	return 0;

device_release:
	nvhost_client_device_release(pdev);
deinit:
	nvhost_module_deinit(pdev);
error:
	if (err != -EPROBE_DEFER)
		dev_err(dev, "probe failed: %d\n", err);
	return err;
}

static int t194_capture_support_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id t194_capture_support_match[] = {
	{
		.compatible = "nvidia,tegra194-isp-thi",
		.data = &t19_isp_thi_info,
	},
	{
		.compatible = "nvidia,tegra194-vi-thi",
		.data = &t19_vi_thi_info,
	},
	{ },
};

static struct platform_driver t194_capture_support_driver = {
	.probe = t194_capture_support_probe,
	.remove = t194_capture_support_remove,
	.driver = {
		/* Only suitable name for dummy falcon driver */
		.name = "scare-pigeon",
		.of_match_table = t194_capture_support_match,
		.pm = &nvhost_module_pm_ops,
	},
};

module_platform_driver(t194_capture_support_driver);
