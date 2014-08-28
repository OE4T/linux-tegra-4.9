/*
 *  drivers/extcon/extcon_gpio.c
 *
 *  Single-state GPIO extcon driver based on extcon class
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * Based on extcon-gpio driver by
 *	Copyright (C) 2008 Google, Inc.
 *	Author: Mike Lockwood <lockwood@android.com>
 *
 * Modified by MyungJoo Ham <myungjoo.ham@samsung.com> to support extcon
 * (originally switch class is supported)
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/extcon.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

struct gpio_extcon_cables {
	int gstate;
	int cstate;
};

struct gpio_extcon_platform_data {
	const char *name;
	int gpio;
	unsigned long debounce;
	unsigned long irq_flags;
	const char **cable_name;
	int cable_count;
	struct gpio_extcon_cables *cable_states;
	int cable_states_count;
};

struct gpio_extcon_data {
	struct device *dev;
	int gpio;
	int irq;
	struct extcon_dev edev;
	struct delayed_work work;
	unsigned long debounce_jiffies;
	struct gpio_extcon_platform_data *pdata;
};

static void gpio_extcon_work(struct work_struct *work)
{
	int state;
	int cstate = -1;
	struct gpio_extcon_data	*data =
		container_of(to_delayed_work(work), struct gpio_extcon_data,
			     work);
	int i;

	state = gpio_get_value_cansleep(data->gpio);
	state &= 0x1;
	for (i = 0; i < data->pdata->cable_states_count; ++i) {
		if (data->pdata->cable_states[i].gstate == state) {
			cstate = data->pdata->cable_states[i].cstate;
			break;
		}
	}

	if (cstate == -1) {
		dev_info(data->dev, "Cable state not found %d\n", state);
		cstate = 0;
	}

	dev_info(data->dev, "Cable state %d\n", cstate);
	extcon_set_state(&data->edev, cstate);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_extcon_data *extcon_data = dev_id;

	if (extcon_data->debounce_jiffies)
		schedule_delayed_work(&extcon_data->work,
			      extcon_data->debounce_jiffies);
	else
		gpio_extcon_work(&extcon_data->work.work);

	return IRQ_HANDLED;
}

static struct gpio_extcon_platform_data *of_get_platform_data(
		struct platform_device *pdev)
{
	struct gpio_extcon_platform_data *pdata;
	struct device_node *np = pdev->dev.of_node;
	int gpio;
	u32 pval;
	int ret;
	const char *names;
	struct property *prop;
	int count;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	of_property_read_string(np, "extcon-gpio,name", &pdata->name);
	if (!pdata->name)
		pdata->name = np->name;

	gpio = of_get_named_gpio(np, "gpio", 0);
	if ((gpio < 0) && (gpio != -ENOENT))
		return ERR_PTR(gpio);
	pdata->gpio = gpio;

	ret = of_property_read_u32(np, "extcon-gpio,irq-flags", &pval);
	if (!ret)
		pdata->irq_flags = pval;
	else
		pdata->irq_flags = IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING;

	ret = of_property_read_u32(np, "extcon-gpio,debounce", &pval);
	if (!ret)
		pdata->debounce = pval;

	pdata->cable_count = of_property_count_strings(np,
					"extcon-gpio,cable-name");
	if (pdata->cable_count <= 0) {
		dev_err(&pdev->dev, "not found cable names\n");
		return ERR_PTR(-EINVAL);
	}

	pdata->cable_name = devm_kzalloc(&pdev->dev, (pdata->cable_count + 1) *
				sizeof(*pdata->cable_name), GFP_KERNEL);
	if (!pdata->cable_name)
		return ERR_PTR(-ENOMEM);
	count = 0;
	of_property_for_each_string(np, "extcon-gpio,cable-name", prop, names)
		pdata->cable_name[count++] = names;
	pdata->cable_name[count] = NULL;

	pdata->cable_states_count = of_property_count_u32(np,
						"extcon-gpio,cable-states");
	if (pdata->cable_states_count < 2) {
		dev_err(&pdev->dev, "not found proper cable state\n");
		return ERR_PTR(-EINVAL);
	}
	pdata->cable_states_count /= 2;
	pdata->cable_states = devm_kzalloc(&pdev->dev,
				(pdata->cable_states_count) *
				sizeof(*pdata->cable_states), GFP_KERNEL);
	if (!pdata->cable_states)
		return ERR_PTR(-ENOMEM);
	for (count = 0;  count < pdata->cable_states_count; ++count) {
		ret = of_property_read_u32_index(np, "extcon-gpio,cable-states",
				count * 2, &pval);
		if (!ret)
			pdata->cable_states[count].gstate = pval;

		ret = of_property_read_u32_index(np, "extcon-gpio,cable-states",
				count * 2 + 1, &pval);
		if (!ret)
			pdata->cable_states[count].cstate = pval;
	}

	return pdata;
}

static int gpio_extcon_probe(struct platform_device *pdev)
{
	struct gpio_extcon_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_extcon_data *extcon_data;
	int ret = 0;

	if (!pdata && pdev->dev.of_node) {
		pdata = of_get_platform_data(pdev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	if (!pdata)
		return -EBUSY;

	if (!pdata->irq_flags) {
		dev_err(&pdev->dev, "IRQ flag is not specified.\n");
		return -EINVAL;
	}

	extcon_data = devm_kzalloc(&pdev->dev, sizeof(struct gpio_extcon_data),
				   GFP_KERNEL);
	if (!extcon_data)
		return -ENOMEM;

	extcon_data->dev = &pdev->dev;
	extcon_data->edev.name = pdata->name;
	extcon_data->edev.dev.parent = &pdev->dev;
	extcon_data->gpio = pdata->gpio;
	extcon_data->debounce_jiffies = msecs_to_jiffies(pdata->debounce);
	extcon_data->edev.supported_cable = pdata->cable_name;
	extcon_data->pdata = pdata;

	ret = extcon_dev_register(&extcon_data->edev);
	if (ret < 0)
		return ret;

	INIT_DELAYED_WORK(&extcon_data->work, gpio_extcon_work);

	ret = devm_gpio_request_one(&pdev->dev, extcon_data->gpio, GPIOF_DIR_IN,
				    pdev->name);
	if (ret < 0)
		goto err;

	extcon_data->irq = gpio_to_irq(extcon_data->gpio);
	if (extcon_data->irq < 0) {
		ret = extcon_data->irq;
		goto err;
	}

	ret = request_any_context_irq(extcon_data->irq, gpio_irq_handler,
				      pdata->irq_flags, pdev->name,
				      extcon_data);
	if (ret < 0)
		goto err;

	platform_set_drvdata(pdev, extcon_data);
	device_set_wakeup_capable(extcon_data->dev, true);

	/* Perform initial detection */
	gpio_extcon_work(&extcon_data->work.work);
	return 0;

err:
	extcon_dev_unregister(&extcon_data->edev);
	return ret;
}

static int gpio_extcon_remove(struct platform_device *pdev)
{
	struct gpio_extcon_data *extcon_data = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&extcon_data->work);
	free_irq(extcon_data->irq, extcon_data);
	extcon_dev_unregister(&extcon_data->edev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_extcon_suspend(struct device *dev)
{
	struct gpio_extcon_data *extcon_data = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&extcon_data->work);
	if (device_may_wakeup(extcon_data->dev))
		enable_irq_wake(extcon_data->irq);

	return 0;
}

static int gpio_extcon_resume(struct device *dev)
{
	struct gpio_extcon_data *extcon_data = dev_get_drvdata(dev);

	if (device_may_wakeup(extcon_data->dev))
		disable_irq_wake(extcon_data->irq);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(gpio_extcon_pm_ops, gpio_extcon_suspend,
						gpio_extcon_resume);

static struct of_device_id of_extcon_gpio_tbl[] = {
	{ .compatible = "extcon-gpio-states", },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, of_extcon_gpio_tbl);

static struct platform_driver gpio_extcon_driver = {
	.probe		= gpio_extcon_probe,
	.remove		= gpio_extcon_remove,
	.driver		= {
		.name	= "extcon-gpio-states",
		.owner	= THIS_MODULE,
		.of_match_table = of_extcon_gpio_tbl,
		.pm = &gpio_extcon_pm_ops,
	},
};

static int __init gpio_extcon_driver_init(void)
{
	return platform_driver_register(&gpio_extcon_driver);
}
subsys_initcall_sync(gpio_extcon_driver_init);

static void __exit gpio_extcon_driver_exit(void)
{
	platform_driver_unregister(&gpio_extcon_driver);
}
module_exit(gpio_extcon_driver_exit);

MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_DESCRIPTION("GPIO state based extcon driver");
MODULE_LICENSE("GPL v2");
