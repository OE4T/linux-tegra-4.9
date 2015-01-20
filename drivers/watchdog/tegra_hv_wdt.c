/*
 * drivers/watchdog/tegra_hv_wdt.c
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/tegra-ivc.h>
#include <linux/tegra-soc.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>

/* The hypervisor monitor service requires the watchdog to be refreshed at
 * least once every 60 seconds, so set our refresh time to less than half of
 * that to account for client reboots that may not be communicated to the
 * monitor service. Note that should all client reboots be communicated to
 * the monitor service, then the refresh interval can be safely doubled. */
#define REFRESH_INTERVAL_SECS (28)

#define IVC_MESSAGE_DATA "x"
#define IVC_SUCCESS_CODE sizeof(IVC_MESSAGE_DATA)

struct tegra_hv_wdt {
	struct watchdog_device wdt;
	struct tegra_hv_ivc_cookie *ivc;
	struct platform_device *pdev;
	struct task_struct *thread;
	struct mutex lock;
	bool user;
	int saved;
};

static int tegra_hv_wdt_start(struct watchdog_device *wdt)
{
	struct tegra_hv_wdt *hv = container_of(wdt, struct tegra_hv_wdt, wdt);

	dev_info(&hv->pdev->dev, "device opened\n");
	hv->user = true;

	return 0;
}

/*
 * This function is only used when the user does a "magic close"
 * (by writing 'V' to the /dev/watchdog device.)
 */
static int tegra_hv_wdt_stop(struct watchdog_device *wdt)
{
	struct tegra_hv_wdt *hv = container_of(wdt, struct tegra_hv_wdt, wdt);

	dev_info(&hv->pdev->dev, "device closed\n");
	hv->user = false;

	return 0;
}

static int tegra_hv_wdt_ping(struct watchdog_device *wdt)
{
	struct tegra_hv_wdt *hv = container_of(wdt, struct tegra_hv_wdt, wdt);
	int ret;

	mutex_lock(&hv->lock);
	ret = tegra_hv_ivc_write(hv->ivc, IVC_MESSAGE_DATA,
		sizeof(IVC_MESSAGE_DATA));

	/*
	 * If a write error occurs, we log the problem only on state changes
	 * to error states. This gives us indications when a problem may be
	 * starting but won't flood system logs with duplicate information of
	 * little value if the error condition persists.
	 */
	if ((ret != IVC_SUCCESS_CODE) && (ret != hv->saved))
		dev_err(&hv->pdev->dev, "ivc write error %d\n", ret);
	hv->saved = ret;

	mutex_unlock(&hv->lock);

	return 0;
}

static int tegra_hv_wdt_loop(void *arg)
{
	struct tegra_hv_wdt *hv = (struct tegra_hv_wdt *)arg;

	while (!kthread_should_stop()) {
		if (!hv->user)
			tegra_hv_wdt_ping(&hv->wdt);
		ssleep(REFRESH_INTERVAL_SECS);
	}

	return 0;
}

static int tegra_hv_wdt_parse(struct platform_device *pdev,
	struct device_node **qn, u32 *id)
{
	struct device_node *dn;

	dn = pdev->dev.of_node;
	if (dn == NULL) {
		dev_err(&pdev->dev, "failed to find device node\n");
		return -EINVAL;
	}

	if (of_property_read_u32_index(dn, "ivc", 1, id) != 0) {
		dev_err(&pdev->dev, "failed to find ivc property\n");
		return -EINVAL;
	}

	*qn = of_parse_phandle(dn, "ivc", 0);
	if (*qn == NULL) {
		dev_err(&pdev->dev, "failed to find queue node\n");
		return -EINVAL;
	}

	return 0;
}

static const struct watchdog_info tegra_hv_wdt_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "Tegra HV WDT",
	.firmware_version = 1,
};

static const struct watchdog_ops tegra_hv_wdt_ops = {
	.owner = THIS_MODULE,
	.start = tegra_hv_wdt_start,
	.stop = tegra_hv_wdt_stop,
	.ping = tegra_hv_wdt_ping,
};

static int tegra_hv_wdt_probe(struct platform_device *pdev)
{
	struct tegra_hv_wdt *hv;
	struct device_node *qn;
	int errcode;
	u32 id;

	if (!is_tegra_hypervisor_mode()) {
		dev_info(&pdev->dev, "hypervisor is not present\n");
		return -ENODEV;
	}

	errcode = tegra_hv_wdt_parse(pdev, &qn, &id);
	if (errcode < 0) {
		dev_err(&pdev->dev, "failed to parse device tree\n");
		return -ENODEV;
	}

	hv = devm_kzalloc(&pdev->dev, sizeof(*hv), GFP_KERNEL);
	if (!hv) {
		of_node_put(qn);
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	hv->pdev = pdev;
	hv->ivc = tegra_hv_ivc_reserve(qn, id, NULL);
	of_node_put(qn);

	if (IS_ERR_OR_NULL(hv->ivc)) {
		devm_kfree(&pdev->dev, hv);
		dev_err(&pdev->dev, "failed to reserve ivc %u\n", id);
		return -EINVAL;
	}

	mutex_init(&hv->lock);
	hv->user = false;
	hv->saved = IVC_SUCCESS_CODE;

	hv->thread = kthread_create(tegra_hv_wdt_loop, (void *)hv,
		"tegra-hv-wdt");
	if (IS_ERR_OR_NULL(hv->thread)) {
		if (tegra_hv_ivc_unreserve(hv->ivc) != 0)
			dev_err(&pdev->dev, "failed to unreserve ivc\n");
		devm_kfree(&pdev->dev, hv);
		dev_err(&pdev->dev, "failed to create kthread\n");
		return -ENOMEM;
	}

	hv->wdt.info = &tegra_hv_wdt_info;
	hv->wdt.ops = &tegra_hv_wdt_ops;
	hv->wdt.timeout = REFRESH_INTERVAL_SECS;

	errcode = watchdog_register_device(&hv->wdt);
	if (errcode < 0) {
		if (kthread_stop(hv->thread) != -EINTR)
			dev_err(&pdev->dev, "failed to stop thread\n");
		if (tegra_hv_ivc_unreserve(hv->ivc) != 0)
			dev_err(&pdev->dev, "failed to unreserve ivc\n");
		devm_kfree(&pdev->dev, hv);
		dev_err(&pdev->dev, "failed to register device\n");
		return errcode;
	}

	platform_set_drvdata(pdev, hv);

	errcode = wake_up_process(hv->thread);
	if (errcode != 1)
		dev_err(&pdev->dev, "failed to wake up thread\n");

	dev_info(&pdev->dev, "id=%u irq=%d peer=%d num=%d size=%d\n",
		id, hv->ivc->irq, hv->ivc->peer_vmid, hv->ivc->nframes,
		hv->ivc->frame_size);

	return 0;
}

static int tegra_hv_wdt_remove(struct platform_device *pdev)
{
	struct tegra_hv_wdt *hv = platform_get_drvdata(pdev);

	if (kthread_stop(hv->thread) != 0)
		dev_err(&pdev->dev, "failed to stop thread\n");
	watchdog_unregister_device(&hv->wdt);
	if (tegra_hv_ivc_unreserve(hv->ivc) != 0)
		dev_err(&pdev->dev, "failed to unreserve ivc\n");
	devm_kfree(&pdev->dev, hv);
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
