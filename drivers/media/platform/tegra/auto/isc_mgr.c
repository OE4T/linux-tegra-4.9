/*
 * isc manager.
 *
 * Copyright (c) 2015, NVIDIA Corporation. All Rights Reserved.
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <media/tegra_isc_mgr.h>

struct isc_mgr_priv {
	u32 *pwdn_gpios;
	enum of_gpio_flags *pwdn_gpios_flags;
	int gpio_irq;
	int num_pwdn_gpios;
	struct siginfo sinfo;
	int sig_no; /* store signal number from user space */
	atomic_t refcnt;
	struct task_struct *t;
	spinlock_t spinlock;
	struct miscdevice *misc_device;
};

#define PW_ON(flag)	((flag) ? 0 : 1)
#define PW_OFF(flag)	((flag) ? 1 : 0)

static irqreturn_t isc_mgr_isr(int irq, void *data)
{
	struct isc_mgr_priv *isc_mgr;
	int ret;
	unsigned long flags;

	if (data) {
		isc_mgr = (struct isc_mgr_priv *)data;
		spin_lock_irqsave(&isc_mgr->spinlock, flags);
		if (isc_mgr->sinfo.si_signo && isc_mgr->t) {
			/* send the signal to user space */
			ret = send_sig_info(isc_mgr->sinfo.si_signo,
					&isc_mgr->sinfo,
					isc_mgr->t);
			if (ret < 0) {
				pr_err("error sending signal\n");
				return IRQ_HANDLED;
			}
		}
		spin_unlock_irqrestore(&isc_mgr->spinlock, flags);
	}

	return IRQ_HANDLED;
}

static int isc_mgr_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc_dev = file->private_data;
	struct device *dev = misc_dev->this_device;
	struct isc_mgr_priv *isc_mgr = dev_get_drvdata(dev);

	/* only one application can open one isc_mgr device */
	if (atomic_xchg(&isc_mgr->refcnt, 1))
		return -EBUSY;

	return 0;
}

static int isc_mgr_release(struct inode *inode, struct file *file)
{
	struct miscdevice *misc_dev = file->private_data;
	struct device *dev = misc_dev->this_device;
	struct isc_mgr_priv *isc_mgr = dev_get_drvdata(dev);
	unsigned long flags;

	/* clear sinfo to prevent report error after handler is closed */
	spin_lock_irqsave(&isc_mgr->spinlock, flags);
	memset(&isc_mgr->sinfo, 0, sizeof(struct siginfo));
	isc_mgr->t = NULL;
	atomic_set(&isc_mgr->refcnt, 0);
	spin_unlock_irqrestore(&isc_mgr->spinlock, flags);

	return 0;
}

static int isc_mgr_write_pid(struct file *file,
			const void __user *arg)
{
	struct miscdevice *misc_dev = file->private_data;
	struct device *dev = misc_dev->this_device;
	struct isc_mgr_priv *isc_mgr = dev_get_drvdata(dev);
	struct tegra_isc_mgr_sinfo sinfo;
	unsigned long flags;

	if (copy_from_user(&sinfo, arg,
		sizeof(struct tegra_isc_mgr_sinfo))) {
		dev_err(dev, "failed to copy from user\n");
		return -EFAULT;
	}

	if (isc_mgr->sinfo.si_int) {
		dev_err(dev, "exist signal info\n");
		return -EINVAL;
	}

	if ((sinfo.sig_no < SIGRTMIN) || (sinfo.sig_no > SIGRTMAX)) {
		dev_err(dev, "Invalid signal number\n");
		return -EINVAL;
	}

	if (!sinfo.pid) {
		dev_err(dev, "Invalid PID\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&isc_mgr->spinlock, flags);
	isc_mgr->sinfo.si_signo = isc_mgr->sig_no = sinfo.sig_no;
	isc_mgr->sinfo.si_code = SI_QUEUE;
	isc_mgr->sinfo.si_ptr = sinfo.context;
	spin_unlock_irqrestore(&isc_mgr->spinlock, flags);

	rcu_read_lock();
	isc_mgr->t = pid_task(find_pid_ns(sinfo.pid, &init_pid_ns),
				PIDTYPE_PID);
	if (isc_mgr->t == NULL) {
		dev_err(dev, "no such pid\n");
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();

	return 0;
}

static int isc_mgr_power_up(struct device *dev,
			struct isc_mgr_priv *isc_mgr,
			unsigned long arg)
{
	int i;

	if (!isc_mgr)
		return -EINVAL;

	if (!isc_mgr->num_pwdn_gpios)
		return 0;

	if (isc_mgr->pwdn_gpios && isc_mgr->pwdn_gpios_flags) {
		if (arg < isc_mgr->num_pwdn_gpios) {
			gpio_set_value(isc_mgr->pwdn_gpios[arg],
				PW_ON(isc_mgr->pwdn_gpios_flags[arg]));
		} else {
			for (i = 0; i < isc_mgr->num_pwdn_gpios; i++)
				gpio_set_value(isc_mgr->pwdn_gpios[i],
					PW_ON(isc_mgr->pwdn_gpios_flags[i]));
		}
		mdelay(7);
	} else {
		dev_warn(dev, "No gpio to control\n");
	}

	return 0;
}


static int isc_mgr_power_down(struct device *dev,
			struct isc_mgr_priv *isc_mgr,
			unsigned long arg)
{
	int i;

	if (!isc_mgr)
		return -EINVAL;

	if (!isc_mgr->num_pwdn_gpios)
		return 0;

	if (isc_mgr->pwdn_gpios && isc_mgr->pwdn_gpios_flags) {
		if (arg < isc_mgr->num_pwdn_gpios) {
			gpio_set_value(isc_mgr->pwdn_gpios[arg],
				PW_OFF(isc_mgr->pwdn_gpios_flags[arg]));
		} else {
			for (i = 0; i < isc_mgr->num_pwdn_gpios; i++)
				gpio_set_value(isc_mgr->pwdn_gpios[i],
					PW_OFF(isc_mgr->pwdn_gpios_flags[i]));
		}
		mdelay(7);
	} else {
		dev_warn(dev, "No gpio to control\n");
	}

	return 0;
}


static long isc_mgr_ioctl(struct file *file,
			 unsigned int cmd,
			 unsigned long arg)
{
	struct miscdevice *misc_dev = file->private_data;
	struct device *dev = misc_dev->this_device;
	struct isc_mgr_priv *isc_mgr = dev_get_drvdata(dev);
	int err = 0;
	unsigned long flags;

	/* command distributor */
	switch (cmd) {
	case TEGRA_ISC_MGR_IOCTL_PWR_DN:
		err = isc_mgr_power_down(dev, isc_mgr, arg);
		break;
	case TEGRA_ISC_MGR_IOCTL_PWR_UP:
		err = isc_mgr_power_up(dev, isc_mgr, arg);
		break;
	case TEGRA_ISC_MGR_IOCTL_SET_PID:
		err = isc_mgr_write_pid(file, (const void __user *)arg);
		break;
	case TEGRA_ISC_MGR_IOCTL_RESUME_SIGNAL:
		if (!isc_mgr->sig_no) {
			dev_err(dev, "invalid sig_no, setup pid first\n");
			return -EINVAL;
		}
		spin_lock_irqsave(&isc_mgr->spinlock, flags);
		isc_mgr->sinfo.si_signo = isc_mgr->sig_no;
		spin_unlock_irqrestore(&isc_mgr->spinlock, flags);
		break;
	case TEGRA_ISC_MGR_IOCTL_SUSPEND_SIGNAL:
		spin_lock_irqsave(&isc_mgr->spinlock, flags);
		isc_mgr->sinfo.si_signo = 0;
		spin_unlock_irqrestore(&isc_mgr->spinlock, flags);
		break;
	default:
		dev_err(dev, "%s unsupported ioctl: %x\n",
			__func__, cmd);
		err = -EINVAL;
	}

	if (err)
		dev_dbg(dev, "err = %d\n", err);

	return err;
}

static const struct file_operations isc_mgr_fileops = {
	.owner = THIS_MODULE,
	.open = isc_mgr_open,
	.unlocked_ioctl = isc_mgr_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = isc_mgr_ioctl,
#endif
	.release = isc_mgr_release,
};

static int isc_mgr_probe(struct platform_device *pdev)
{
	int err = 0;
	struct isc_mgr_priv *isc_mgr;
	unsigned int i;

	dev_info(&pdev->dev, "%sing...\n", __func__);

	isc_mgr = devm_kzalloc(&pdev->dev,
			sizeof(struct isc_mgr_priv),
			GFP_KERNEL);
	if (!isc_mgr) {
		dev_err(&pdev->dev, "Unable to allocate memory!\n");
		return -ENOMEM;
	}

	isc_mgr->num_pwdn_gpios = of_gpio_named_count(pdev->dev.of_node,
				"pwdn-gpios");
	if (isc_mgr->num_pwdn_gpios > 0) {
		isc_mgr->pwdn_gpios = devm_kzalloc(&pdev->dev,
				sizeof(u32) * isc_mgr->num_pwdn_gpios,
				GFP_KERNEL);
		isc_mgr->pwdn_gpios_flags = devm_kzalloc(&pdev->dev,
				sizeof(enum of_gpio_flags) *
					isc_mgr->num_pwdn_gpios,
				GFP_KERNEL);

		for (i = 0; i < isc_mgr->num_pwdn_gpios; i++) {
			isc_mgr->pwdn_gpios[i] = of_get_named_gpio(
					pdev->dev.of_node,
					"pwdn-gpios", i);
			if (!gpio_is_valid(isc_mgr->pwdn_gpios[i]))
				goto err_probe;
			err = of_get_named_gpio_flags(pdev->dev.of_node,
				"pwdn-gpios", i, &isc_mgr->pwdn_gpios_flags[i]);
			if (err < 0)
				goto err_probe;
		}

		for (i = 0; i < isc_mgr->num_pwdn_gpios; i++) {
			if (devm_gpio_request(&pdev->dev,
					isc_mgr->pwdn_gpios[i],
					"pwdn-gpios")) {
				dev_err(&pdev->dev, "failed to request GPIO: %d\n",
					isc_mgr->pwdn_gpios[i]);
				goto err_gpio_request;
			}

			err = gpio_direction_output(isc_mgr->pwdn_gpios[i],
					PW_ON(isc_mgr->pwdn_gpios_flags[i]));
			if (err < 0) {
				dev_err(&pdev->dev, "failed to setup GPIO: %d\n",
					isc_mgr->pwdn_gpios[i]);
				i++;
				goto err_gpio;
			}
		}
	} else {
		isc_mgr->num_pwdn_gpios = 0;
	}

	spin_lock_init(&isc_mgr->spinlock);
	atomic_set(&isc_mgr->refcnt, 0);
	isc_mgr->gpio_irq = platform_get_irq(pdev, 0);
	dev_set_drvdata(&pdev->dev, isc_mgr);

	if (isc_mgr->gpio_irq > 0) {
		err = devm_request_irq(&pdev->dev,
				isc_mgr->gpio_irq,
				isc_mgr_isr, 0, pdev->name, isc_mgr);
		if (err) {
			dev_err(&pdev->dev,
				"request_irq failed with err %d\n", err);
			goto err_gpio;
		}
	}

	isc_mgr->misc_device = devm_kzalloc(&pdev->dev,
					sizeof(struct miscdevice),
					GFP_KERNEL);
	if (!isc_mgr->misc_device) {
		dev_err(&pdev->dev, "Unable to allocate misc_device!\n");
		goto err_interrupt;
	}

	isc_mgr->misc_device->name = pdev->dev.of_node->name;
	isc_mgr->misc_device->minor = MISC_DYNAMIC_MINOR;
	isc_mgr->misc_device->fops = &isc_mgr_fileops;

	err = misc_register(isc_mgr->misc_device);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to register misc device!\n");
		goto err_interrupt;
	}

	dev_set_drvdata(isc_mgr->misc_device->this_device, isc_mgr);

	return 0;

err_interrupt:
	devm_free_irq(&pdev->dev, isc_mgr->gpio_irq, isc_mgr);
err_gpio:
	i = isc_mgr->num_pwdn_gpios;
err_gpio_request:
	for (i--; i < 0; i--)
		devm_gpio_free(&pdev->dev, isc_mgr->pwdn_gpios[i]);
err_probe:
	return err;
}

static int isc_mgr_remove(struct platform_device *pdev)
{
	struct isc_mgr_priv *isc_mgr;
	int i, err;

	isc_mgr = (struct isc_mgr_priv *)dev_get_drvdata(&pdev->dev);

	for (i = 0; i < isc_mgr->num_pwdn_gpios; i++) {
		err = gpio_direction_output(isc_mgr->pwdn_gpios[i],
				PW_OFF(isc_mgr->pwdn_gpios_flags[i]));
		if (err < 0) {
			dev_err(&pdev->dev, "failed to off GPIO: %d\n",
				isc_mgr->pwdn_gpios[i]);
		}
		devm_gpio_free(&pdev->dev, isc_mgr->pwdn_gpios[i]);
	}
	if (isc_mgr->gpio_irq > 0)
		devm_free_irq(&pdev->dev, isc_mgr->gpio_irq, isc_mgr);

	misc_deregister(isc_mgr->misc_device);

	return 0;
}

static const struct of_device_id isc_mgr_of_match[] = {
	{ .compatible = "nvidia,isc-mgr", },
	{ }
};
MODULE_DEVICE_TABLE(of, isc_mgr_of_match);

static struct platform_driver isc_mgr_driver = {
	.driver = {
		.name = "isc-mgr",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(isc_mgr_of_match)
	},
	.probe = isc_mgr_probe,
	.remove = isc_mgr_remove,
};

module_platform_driver(isc_mgr_driver);

MODULE_DESCRIPTION("tegra auto isc manager driver");
MODULE_AUTHOR("Songhee Baek <sbeak@nvidia.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:isc_mgr");
