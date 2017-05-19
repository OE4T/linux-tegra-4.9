/*
 * drivers/power/reset/tmpm32xi2c-poweroff.c
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/mfd/tmpm32xi2c.h>

struct tmpm32xi2c_poweroff_data {
	struct device *dev;
	struct notifier_block reboot_nb;
};

static int tmpm32xi2c_poweroff_reboot_notify(struct notifier_block *nb,
					     unsigned long mode, void *cmd)
{
	struct tmpm32xi2c_poweroff_data *data =
		container_of(nb, struct tmpm32xi2c_poweroff_data, reboot_nb);
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(data->dev->parent);
	uint8_t tx_buf[] = { CMD_SHUTDOWN_PREPARE, SHUTDOWN_PREPARE_SET };
	int ret;

	if ((mode == SYS_HALT) || (mode == SYS_POWER_OFF)) {
		/*
		 * Send a CMD_SHUTDOWN_PREPARE set command to notify MCU that
		 * the system is going to shutdown.
		 */
		ret = chip->write_read(chip, tx_buf, sizeof(tx_buf), NULL, 0);
		if (ret < 0)
			dev_err(data->dev,
				"Failed to send SHUTDOWN_PREPARE_SET, %d\n",
				ret);
		else
			dev_dbg(data->dev, "Sent SHUTDOWN_PREPARE_SET\n");
	}

	return NOTIFY_DONE;
}

static int tmpm32xi2c_poweroff_probe(struct platform_device *pdev)
{
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct tmpm32xi2c_poweroff_data *data;
	uint8_t tx_buf[] = { CMD_SHUTDOWN_PREPARE, SHUTDOWN_PREPARE_CLEAR };
	int ret;

	data = devm_kzalloc(&pdev->dev,
			    sizeof(struct tmpm32xi2c_poweroff_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;

	/*
	 * Send a CMD_SHUTDOWN_PREPARE clear command to clear a flag of
	 * shutdown prepare in MCU F/W.
	 */
	ret = chip->write_read(chip, tx_buf, sizeof(tx_buf), NULL, 0);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to send SHUTDOWN_PREPARE_CLEAR, %d\n", ret);
		return ret;
	}
	dev_dbg(data->dev, "Sent SHUTDOWN_PREPARE_CLEAR\n");

	data->reboot_nb.notifier_call = tmpm32xi2c_poweroff_reboot_notify;
	ret = register_reboot_notifier(&data->reboot_nb);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to register reboot notifier, %d\n", ret);
		return ret;
	}

	return 0;
}

static int tmpm32xi2c_poweroff_remove(struct platform_device *pdev)
{
	struct tmpm32xi2c_poweroff_data *data = platform_get_drvdata(pdev);

	unregister_reboot_notifier(&data->reboot_nb);

	return 0;
}

static const struct platform_device_id tmpm32xi2c_poweroff_id[] = {
	{ "tmpm32xi2c-poweroff", 0, },
	{ }
};
MODULE_DEVICE_TABLE(platform, tmpm32xi2c_poweroff_id);

static struct platform_driver tmpm32xi2c_poweroff_driver = {
	.driver = {
		.name	= "tmpm32xi2c-poweroff",
	},
	.probe		= tmpm32xi2c_poweroff_probe,
	.remove		= tmpm32xi2c_poweroff_remove,
	.id_table	= tmpm32xi2c_poweroff_id,
};

static int __init tmpm32xi2c_poweroff_init(void)
{
	return platform_driver_register(&tmpm32xi2c_poweroff_driver);
}
/* register after postcore initcall and subsys initcall that may rely on I2C. */
subsys_initcall_sync(tmpm32xi2c_poweroff_init);

static void __exit tmpm32xi2c_poweroff_exit(void)
{
	platform_driver_unregister(&tmpm32xi2c_poweroff_driver);
}
module_exit(tmpm32xi2c_poweroff_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("Power off driver for TMPM32x I2C");
MODULE_LICENSE("GPL");
