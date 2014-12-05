/*
 * drivers/watchdog/tegra_hv_wdt.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/timer.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-soc.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>

/* The hypervisor monitor service requires the watchdog to be refreshed at
 * least once every 60 seconds, so set our refresh time to less than half of
 * that to account for client reboots that may not be communicated to the
 * monitor service. Note that should all client reboots be communicated to
 * the monitor service, then the refresh interval can be safely doubled. */
#define REFRESH_INTERVAL_SECS (28)

struct tegra_hv_wdt {
	struct tegra_hv_ivc_cookie *ivc;
	struct platform_device *pdev;
	struct timer_list timer;
};

static void tegra_hv_wdt_refresh(unsigned long data)
{
	struct tegra_hv_wdt *wdt = (struct tegra_hv_wdt *)data;
	int ret;

	ret = tegra_hv_ivc_write(wdt->ivc, "x", 1);
	if (ret != 1)
		dev_info(&wdt->pdev->dev, "ivc write error %d\n", ret);
	mod_timer(&wdt->timer, jiffies + HZ * REFRESH_INTERVAL_SECS);
}

static int tegra_hv_wdt_probe(struct platform_device *pdev)
{
	struct tegra_hv_wdt *wdt;
	struct device_node *dn;
	struct device_node *qn;
	u32 id;

	if (!is_tegra_hypervisor_mode()) {
		dev_info(&pdev->dev, "hypervisor is not present\n");
		return -ENODEV;
	}

	dn = pdev->dev.of_node;
	if (dn == NULL) {
		dev_err(&pdev->dev, "failed to find device node\n");
		return -EINVAL;
	}

	if (of_property_read_u32_index(dn, "ivc", 1, &id) != 0) {
		dev_err(&pdev->dev, "failed to find ivc property\n");
		return -EINVAL;
	}

	qn = of_parse_phandle(dn, "ivc", 0);
	if (qn == NULL) {
		dev_err(&pdev->dev, "failed to find queue node\n");
		return -EINVAL;
	}

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt) {
		of_node_put(qn);
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	wdt->pdev = pdev;
	wdt->ivc = tegra_hv_ivc_reserve(qn, id, NULL);
	of_node_put(qn);

	if (IS_ERR_OR_NULL(wdt->ivc)) {
		devm_kfree(&pdev->dev, wdt);
		dev_err(&pdev->dev, "failed to reserve ivc %u\n", id);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, wdt);

	init_timer(&wdt->timer);
	setup_timer(&wdt->timer, tegra_hv_wdt_refresh, (unsigned long)wdt);
	tegra_hv_wdt_refresh((unsigned long)wdt);

	dev_info(&pdev->dev, "id=%u irq=%d peer=%d num=%d size=%d\n",
		id, wdt->ivc->irq, wdt->ivc->peer_vmid, wdt->ivc->nframes,
		wdt->ivc->frame_size);

	return 0;
}

static int tegra_hv_wdt_remove(struct platform_device *pdev)
{
	struct tegra_hv_wdt *wdt = platform_get_drvdata(pdev);

	del_timer_sync(&wdt->timer);
	if (tegra_hv_ivc_unreserve(wdt->ivc) != 0)
		dev_err(&pdev->dev, "failed to unreserve ivc\n");
	devm_kfree(&pdev->dev, wdt);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id tegra_hv_wdt_match[] = {
	{ .compatible = "nvidia,tegra-hv-wdt", },
	{}
};

static struct platform_driver tegra_hv_wdt_driver = {
	.probe		= tegra_hv_wdt_probe,
	.remove		= tegra_hv_wdt_remove,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "tegra_hv_wdt",
		.of_match_table = of_match_ptr(tegra_hv_wdt_match),
	},
};

static int __init tegra_hv_wdt_init(void)
{
	return platform_driver_register(&tegra_hv_wdt_driver);
}

static void __exit tegra_hv_wdt_exit(void)
{
	platform_driver_unregister(&tegra_hv_wdt_driver);
}

subsys_initcall(tegra_hv_wdt_init);
module_exit(tegra_hv_wdt_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("Tegra Hypervisor Watchdog Driver");
MODULE_LICENSE("GPL");
