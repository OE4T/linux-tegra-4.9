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
#include <linux/slab.h>
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
#include <linux/i2c.h>
#include <media/isc-dev.h>
#include <media/isc-mgr.h>

struct isc_mgr_priv {
	struct device *dev;
	struct miscdevice misc_device;
	struct i2c_adapter *adap;
	struct isc_mgr_platform_data *pdata;
	struct list_head dev_list;
	struct mutex mutex;
	struct task_struct *t;
	struct siginfo sinfo;
	int sig_no; /* store signal number from user space */
	spinlock_t spinlock;
	atomic_t in_use;
	int err_irq;
	char devname[32];
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

int isc_delete_lst(void *ptr, struct i2c_client *client)
{
	struct isc_mgr_priv *isc_mgr = ptr;
	struct isc_mgr_client *isc_dev;

	if (ptr == NULL)
		return -EFAULT;

	mutex_lock(&isc_mgr->mutex);
	list_for_each_entry(isc_dev, &isc_mgr->dev_list, list) {
		if (isc_dev->client == client) {
			list_del(&isc_dev->list);
			break;
		}
	}
	mutex_unlock(&isc_mgr->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(isc_delete_lst);

static int isc_remove_dev(struct isc_mgr_priv *isc_mgr, unsigned long arg)
{
	struct isc_mgr_client *isc_dev;

	dev_dbg(isc_mgr->dev, "%s %ld\n", __func__, arg);
	mutex_lock(&isc_mgr->mutex);
	list_for_each_entry(isc_dev, &isc_mgr->dev_list, list) {
		if (isc_dev->id == arg) {
			list_del(&isc_dev->list);
			break;
		}
	}
	mutex_unlock(&isc_mgr->mutex);

	if (&isc_dev->list != &isc_mgr->dev_list)
		i2c_unregister_device(isc_dev->client);
	else
		dev_err(isc_mgr->dev, "%s: list %lx un-exist\n", __func__, arg);
	return 0;
}

static int isc_create_dev(struct isc_mgr_priv *isc_mgr, const void __user *arg)
{
	struct isc_mgr_client *isc_dev;
	struct i2c_board_info brd;
	int err = 0;

	isc_dev = devm_kzalloc(isc_mgr->dev, sizeof(*isc_dev), GFP_KERNEL);
	if (!isc_dev) {
		dev_err(isc_mgr->dev, "Unable to allocate memory!\n");
		return -ENOMEM;
	}

	if (copy_from_user(&isc_dev->cfg, arg, sizeof(isc_dev->cfg))) {
		dev_err(isc_mgr->dev,
			"%s: failed to copy from user\n", __func__);
		err = -EFAULT;
		goto dev_create_err;
	}
	dev_dbg(isc_mgr->dev, "%s - %s @ %x, %d %d\n", __func__,
		isc_dev->cfg.drv_name, isc_dev->cfg.addr,
		isc_dev->cfg.reg_bits, isc_dev->cfg.val_bits);

	if (isc_dev->cfg.addr >= 0x80) {
		dev_err(isc_mgr->dev, "%s: invalid slave addr %x\n",
			__func__, isc_dev->cfg.addr);
		err = -EINVAL;
		goto dev_create_err;
	}

	isc_dev->pdata.isc_mgr = isc_mgr;
	snprintf(isc_dev->pdata.drv_name, sizeof(isc_dev->pdata.drv_name),
			"%s.%u.%02x", isc_dev->cfg.drv_name,
			isc_mgr->adap->nr, isc_dev->cfg.addr);
	isc_dev->pdata.regmap_cfg.name = isc_dev->pdata.drv_name;
	isc_dev->pdata.regmap_cfg.reg_bits = isc_dev->cfg.reg_bits;
	isc_dev->pdata.regmap_cfg.val_bits = isc_dev->cfg.val_bits;

	mutex_init(&isc_dev->mutex);
	INIT_LIST_HEAD(&isc_dev->list);

	memset(&brd, 0, sizeof(brd));
	strncpy(brd.type, "isc-dev", sizeof(brd.type));
	brd.addr = isc_dev->cfg.addr;
	brd.platform_data = &isc_dev->pdata;
	isc_dev->client = i2c_new_device(isc_mgr->adap, &brd);
	if (!isc_dev->client) {
		dev_err(isc_mgr->dev,
			"%s cannot allocate client: %s bus %d, %x\n", __func__,
			isc_dev->pdata.drv_name, isc_mgr->adap->nr, brd.addr);
		err = -EINVAL;
		goto dev_create_err;
	}

	mutex_lock(&isc_mgr->mutex);
	if (!list_empty(&isc_mgr->dev_list))
		isc_dev->id = list_entry(isc_mgr->dev_list.prev,
			struct isc_mgr_client, list)->id + 1;
	list_add(&isc_dev->list, &isc_mgr->dev_list);
	mutex_unlock(&isc_mgr->mutex);

dev_create_err:
	if (err) {
		devm_kfree(isc_mgr->dev, isc_dev);
		return err;
	} else
		return isc_dev->id;
}

static int isc_mgr_write_pid(struct file *file,
			const void __user *arg)
{
	struct isc_mgr_priv *isc_mgr = file->private_data;
	struct isc_mgr_sinfo sinfo;
	unsigned long flags;

	if (copy_from_user(&sinfo, arg, sizeof(sinfo))) {
		dev_err(isc_mgr->dev,
			"%s: failed to copy from user\n", __func__);
		return -EFAULT;
	}

	if (isc_mgr->sinfo.si_int) {
		dev_err(isc_mgr->dev, "exist signal info\n");
		return -EINVAL;
	}

	if ((sinfo.sig_no < SIGRTMIN) || (sinfo.sig_no > SIGRTMAX)) {
		dev_err(isc_mgr->dev, "Invalid signal number\n");
		return -EINVAL;
	}

	if (!sinfo.pid) {
		dev_err(isc_mgr->dev, "Invalid PID\n");
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
		dev_err(isc_mgr->dev, "no such pid\n");
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();

	return 0;
}

static int isc_mgr_power_up(struct isc_mgr_priv *isc_mgr,
			unsigned long arg)
{
	struct isc_mgr_platform_data *pd = isc_mgr->pdata;
	int i;

	dev_dbg(isc_mgr->dev, "%s - %lu\n", __func__, arg);

	if (!pd->num_gpios)
		goto pwr_up_end;

	if (arg < pd->num_gpios)
		gpio_set_value(pd->gpios[arg], PW_ON(pd->flags[arg]));
	else
		for (i = 0; i < pd->num_gpios; i++) {
			dev_dbg(isc_mgr->dev, "  - %d, %d\n",
				pd->gpios[i], PW_ON(pd->flags[i]));
			gpio_set_value(pd->gpios[i], PW_ON(pd->flags[i]));
		}
	mdelay(7);

pwr_up_end:
	if (isc_mgr->err_irq)
		enable_irq(isc_mgr->err_irq);

	return 0;
}

static int isc_mgr_power_down(struct isc_mgr_priv *isc_mgr,
			unsigned long arg)
{
	struct isc_mgr_platform_data *pd = isc_mgr->pdata;
	int i;

	dev_dbg(isc_mgr->dev, "%s - %lx\n", __func__, arg);

	if (isc_mgr->err_irq)
		disable_irq(isc_mgr->err_irq);
	if (!pd->num_gpios)
		goto pwr_dn_end;

	if (arg < pd->num_gpios)
		gpio_set_value(pd->gpios[arg], PW_OFF(pd->flags[arg]));
	else
		for (i = 0; i < pd->num_gpios; i++) {
			dev_dbg(isc_mgr->dev, "  - %d, %d\n",
				pd->gpios[i], PW_OFF(pd->flags[i]));
			gpio_set_value(pd->gpios[i], PW_OFF(pd->flags[i]));
		}
	mdelay(7);

pwr_dn_end:
	return 0;
}


static long isc_mgr_ioctl(struct file *file,
			 unsigned int cmd,
			 unsigned long arg)
{
	struct isc_mgr_priv *isc_mgr = file->private_data;
	int err = 0;
	unsigned long flags;

	/* command distributor */
	switch (cmd) {
	case ISC_MGR_IOCTL_DEV_ADD:
		err = isc_create_dev(isc_mgr, (const void __user *)arg);
		break;
	case ISC_MGR_IOCTL_DEV_DEL:
		isc_remove_dev(isc_mgr, arg);
		break;
	case ISC_MGR_IOCTL_PWR_DN:
		err = isc_mgr_power_down(isc_mgr, arg);
		break;
	case ISC_MGR_IOCTL_PWR_UP:
		err = isc_mgr_power_up(isc_mgr, arg);
		break;
	case ISC_MGR_IOCTL_SET_PID:
		err = isc_mgr_write_pid(file, (const void __user *)arg);
		break;
	case ISC_MGR_IOCTL_SIGNAL:
		switch (arg) {
		case ISC_MGR_SIGNAL_RESUME:
			if (!isc_mgr->sig_no) {
				dev_err(isc_mgr->dev,
					"invalid sig_no, setup pid first\n");
				return -EINVAL;
			}
			spin_lock_irqsave(&isc_mgr->spinlock, flags);
			isc_mgr->sinfo.si_signo = isc_mgr->sig_no;
			spin_unlock_irqrestore(&isc_mgr->spinlock, flags);
			break;
		case ISC_MGR_SIGNAL_SUSPEND:
			spin_lock_irqsave(&isc_mgr->spinlock, flags);
			isc_mgr->sinfo.si_signo = 0;
			spin_unlock_irqrestore(&isc_mgr->spinlock, flags);
			break;
		default:
			dev_err(isc_mgr->dev, "%s unrecognized signal: %lx\n",
				__func__, arg);
		}
		break;
	default:
		dev_err(isc_mgr->dev, "%s unsupported ioctl: %x\n",
			__func__, cmd);
		err = -EINVAL;
	}

	if (err)
		dev_dbg(isc_mgr->dev, "err = %d\n", err);

	return err;
}

static int isc_mgr_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc_dev = file->private_data;
	struct isc_mgr_priv *isc_mgr =
		container_of(misc_dev, struct isc_mgr_priv, misc_device);

	if (!isc_mgr)
		return -ENODEV;
	/* only one application can open one isc_mgr device */
	if (atomic_xchg(&isc_mgr->in_use, 1))
		return -EBUSY;

	dev_dbg(isc_mgr->dev, "%s\n", __func__);
	file->private_data = isc_mgr;
	return 0;
}

static int isc_mgr_release(struct inode *inode, struct file *file)
{
	struct isc_mgr_priv *isc_mgr = file->private_data;
	unsigned long flags;

	/* clear sinfo to prevent report error after handler is closed */
	spin_lock_irqsave(&isc_mgr->spinlock, flags);
	memset(&isc_mgr->sinfo, 0, sizeof(struct siginfo));
	isc_mgr->t = NULL;
	WARN_ON(!atomic_xchg(&isc_mgr->in_use, 0));
	spin_unlock_irqrestore(&isc_mgr->spinlock, flags);

	return 0;
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

static void isc_mgr_del(struct isc_mgr_priv *isc_mgr)
{
	struct isc_mgr_platform_data *pd = isc_mgr->pdata;
	struct isc_mgr_client *isc_dev;
	int i;

	mutex_lock(&isc_mgr->mutex);
	list_for_each_entry(isc_dev, &isc_mgr->dev_list, list) {
		list_del(&isc_dev->list);
		i2c_unregister_device(isc_dev->client);
	}
	mutex_unlock(&isc_mgr->mutex);

	for (i = 0; i < pd->num_gpios; i++)
		if (pd->gpios[i])
			gpio_direction_output(
				pd->gpios[i], PW_OFF(pd->flags[i]));
}

struct isc_mgr_platform_data *of_isc_mgr_pdata(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct isc_mgr_platform_data *pd = NULL;
	int err, i;

	dev_dbg(&pdev->dev, "%s\n", __func__);
	pd = devm_kzalloc(&pdev->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		dev_err(&pdev->dev, "%s: allocate memory error\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	pd->drv_name = (void *)of_get_property(np, "drv_name", NULL);
	if (pd->drv_name)
		dev_dbg(&pdev->dev, "    drvname: %s\n", pd->drv_name);

	err = of_property_read_u32(np, "i2c-bus", &pd->bus);
	if (err) {
		dev_err(&pdev->dev, "%s: missing i2c bus # DT %s\n",
			__func__, np->full_name);
		return ERR_PTR(-EEXIST);
	}
	dev_dbg(&pdev->dev, "    i2c-bus: %d\n", pd->bus);

	err = of_property_read_u32(np, "csi-port", &pd->csi_port);
	if (err) {
		dev_err(&pdev->dev, "%s: missing csi port # DT %s\n",
			__func__, np->full_name);
		return ERR_PTR(-EEXIST);
	}
	dev_dbg(&pdev->dev, "    csiport: %d\n", pd->csi_port);

	pd->num_gpios = of_gpio_named_count(np, "pwdn-gpios");
	if (pd->num_gpios < 0)
		pd->num_gpios = 0;
	dev_dbg(&pdev->dev, "    gpionum: %d\n", pd->num_gpios);
	if (pd->num_gpios > 0) {
		for (i = 0; (i < pd->num_gpios) &&
			(i < ARRAY_SIZE(pd->gpios)); i++) {
			pd->gpios[i] = of_get_named_gpio_flags(
				np, "pwdn-gpios", i, &pd->flags[i]);
			if (pd->gpios[i] < 0) {
				dev_err(&pdev->dev, "%s: gpio[%d] invalid\n",
					__func__, i);
				return ERR_PTR(-EINVAL);
			}
			dev_dbg(&pdev->dev, "        [%d] - %d %x\n",
				i, pd->gpios[i], pd->flags[i]);
		}
	}

	pd->default_pwr_on = of_property_read_bool(np, "default-power-on");

	return pd;
}

static int isc_mgr_probe(struct platform_device *pdev)
{
	int err = 0;
	struct isc_mgr_priv *isc_mgr;
	struct isc_mgr_platform_data *pd;
	unsigned int i;

	dev_info(&pdev->dev, "%sing...\n", __func__);

	isc_mgr = devm_kzalloc(&pdev->dev,
			sizeof(struct isc_mgr_priv),
			GFP_KERNEL);
	if (!isc_mgr) {
		dev_err(&pdev->dev, "Unable to allocate memory!\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&isc_mgr->dev_list);
	mutex_init(&isc_mgr->mutex);

	if (pdev->dev.of_node) {
		pd = of_isc_mgr_pdata(pdev);
		if (IS_ERR(pd))
			return PTR_ERR(pd);
		isc_mgr->pdata = pd;
	} else if (pdev->dev.platform_data) {
		isc_mgr->pdata = pdev->dev.platform_data;
		pd = isc_mgr->pdata;
	} else {
		dev_err(&pdev->dev, "%s No platform data.\n", __func__);
		return -EFAULT;
	}

	isc_mgr->adap = i2c_get_adapter(pd->bus);
	if (!isc_mgr->adap) {
		dev_err(&pdev->dev, "%s no such i2c bus %d\n",
			__func__, pd->bus);
		return -ENODEV;
	}

	if (pd->num_gpios > 0) {
		for (i = 0; i < pd->num_gpios; i++) {
			if (!gpio_is_valid(pd->gpios[i]))
				goto err_probe;

			if (devm_gpio_request(
				&pdev->dev, pd->gpios[i], "pwdn-gpios")) {
				dev_err(&pdev->dev, "failed to req GPIO: %d\n",
					pd->gpios[i]);
				goto err_probe;
			}

			err = gpio_direction_output(pd->gpios[i],
				pd->default_pwr_on ?
				PW_ON(pd->flags[i]) : PW_OFF(pd->flags[i]));
			if (err < 0) {
				dev_err(&pdev->dev, "failed to setup GPIO: %d\n",
					pd->gpios[i]);
				i++;
				goto err_probe;
			}
		}
	}

	spin_lock_init(&isc_mgr->spinlock);
	atomic_set(&isc_mgr->in_use, 0);

	isc_mgr->err_irq = platform_get_irq(pdev, 0);
	if (isc_mgr->err_irq > 0) {
		err = devm_request_irq(&pdev->dev,
				isc_mgr->err_irq,
				isc_mgr_isr, 0, pdev->name, isc_mgr);
		if (err) {
			dev_err(&pdev->dev,
				"request_irq failed with err %d\n", err);
			isc_mgr->err_irq = 0;
			goto err_probe;
		}
	}

	isc_mgr->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, isc_mgr);

	if (pd->drv_name)
		snprintf(isc_mgr->devname, sizeof(isc_mgr->devname),
			"%s.%x.%c", pd->drv_name, pd->bus, 'a' + pd->csi_port);
	else
		snprintf(isc_mgr->devname, sizeof(isc_mgr->devname),
			"isc-mgr.%x.%c", pd->bus, 'a' + pd->csi_port);

	isc_mgr->misc_device.name = isc_mgr->devname;
	isc_mgr->misc_device.minor = MISC_DYNAMIC_MINOR;
	isc_mgr->misc_device.fops = &isc_mgr_fileops;

	err = misc_register(&isc_mgr->misc_device);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to register misc device!\n");
		goto err_probe;
	}

	return 0;

err_probe:
	isc_mgr_del(isc_mgr);
	return err;
}

static int isc_mgr_remove(struct platform_device *pdev)
{
	struct isc_mgr_priv *isc_mgr = dev_get_drvdata(&pdev->dev);

	if (isc_mgr) {
		isc_mgr_del(isc_mgr);
		misc_deregister(&isc_mgr->misc_device);
	}

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
