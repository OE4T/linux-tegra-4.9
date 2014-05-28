/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/nvhost.h>
#include <linux/nvhost_vi_ioctl.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <asm/uaccess.h>

#include <mach/clk.h>
#include <mach/latency_allowance.h>

#include "bus_client.h"
#include "chip_support.h"
#include "host1x/host1x.h"
#include "vi.h"
#include "vi_irq.h"

static DEFINE_MUTEX(la_lock);

#define T12_VI_CFG_CG_CTRL	0xb8
#define T12_CG_2ND_LEVEL_EN	1
#define T12_VI_CSI_0_SW_RESET	0x100
#define T12_CSI_CSI_SW_SENSOR_A_RESET 0x858
#define T12_CSI_CSICIL_SW_SENSOR_A_RESET 0x94c
#define T12_VI_CSI_0_CSI_IMAGE_DT 0x120

#define T12_VI_CSI_1_SW_RESET	0x200
#define T12_CSI_CSI_SW_SENSOR_B_RESET 0x88c
#define T12_CSI_CSICIL_SW_SENSOR_B_RESET 0x980
#define T12_VI_CSI_1_CSI_IMAGE_DT 0x220

/*
 * MAX_BW = max(VI clock) * 2BPP, in KBps.
 * Here default max VI clock is 420MHz.
 */
#define VI_DEFAULT_MAX_BW	840000

#ifdef TEGRA_12X_OR_HIGHER_CONFIG

int nvhost_vi_finalize_poweron(struct platform_device *dev)
{
	int ret = 0;
	struct vi *tegra_vi;

	tegra_vi = (struct vi *)nvhost_get_private_data(dev);

	if (tegra_vi && tegra_vi->reg) {
		ret = regulator_enable(tegra_vi->reg);
		if (ret) {
			dev_err(&dev->dev,
					"%s: enable csi regulator failed.\n",
					__func__);
			goto fail;
		}
	}

	/* Only do this for vi.0 not for slave device vi.1 */
	if (dev->id == 0)
		host1x_writel(dev, T12_VI_CFG_CG_CTRL, T12_CG_2ND_LEVEL_EN);

fail:
	return ret;
}

int nvhost_vi_prepare_poweroff(struct platform_device *dev)
{
	int ret = 0;
	struct vi *tegra_vi;
	tegra_vi = (struct vi *)nvhost_get_private_data(dev);

	if (tegra_vi && tegra_vi->reg) {
		ret = regulator_disable(tegra_vi->reg);
		if (ret) {
			dev_err(&dev->dev,
				"%s: disable csi regulator failed.\n",
				__func__);
			goto fail;
		}
	}
fail:
	return ret;
}

#if defined(CONFIG_TEGRA_ISOMGR)
static int vi_isomgr_register(struct vi *tegra_vi)
{
	int iso_client_id = TEGRA_ISO_CLIENT_VI_0;

	dev_dbg(&tegra_vi->ndev->dev, "%s++\n", __func__);

	if (tegra_vi->ndev->id)
		iso_client_id = TEGRA_ISO_CLIENT_VI_1;

	/* Register with max possible BW in VI usecases.*/
	tegra_vi->isomgr_handle = tegra_isomgr_register(iso_client_id,
					VI_DEFAULT_MAX_BW,
					NULL,	/* tegra_isomgr_renegotiate */
					NULL);	/* *priv */

	if (!tegra_vi->isomgr_handle) {
		dev_err(&tegra_vi->ndev->dev, "%s: unable to register isomgr\n",
					__func__);
		return -ENOMEM;
	}

	return 0;
}

static int vi_set_isomgr_request(struct vi *tegra_vi, uint vi_bw, uint lt)
{
	int ret = 0;

	dev_dbg(&tegra_vi->ndev->dev,
		"%s++ bw=%u, lt=%u\n", __func__, vi_bw, lt);

	/* return value of tegra_isomgr_reserve is dvfs latency in usec */
	ret = tegra_isomgr_reserve(tegra_vi->isomgr_handle,
				vi_bw,	/* KB/sec */
				lt);	/* usec */
	if (!ret) {
		dev_err(&tegra_vi->ndev->dev,
		"%s: failed to reserve %u KBps\n", __func__, vi_bw);
		return -ENOMEM;
	}

	/* return value of tegra_isomgr_realize is dvfs latency in usec */
	ret = tegra_isomgr_realize(tegra_vi->isomgr_handle);
	if (ret)
		dev_dbg(&tegra_vi->ndev->dev,
		"%s: tegra_vi isomgr latency is %d usec",
		__func__, ret);
	else {
		dev_err(&tegra_vi->ndev->dev,
		"%s: failed to realize %u KBps\n", __func__, vi_bw);
			return -ENOMEM;
	}
	return 0;
}

static int vi_isomgr_release(struct vi *tegra_vi)
{
	int ret = 0;
	dev_dbg(&tegra_vi->ndev->dev, "%s++\n", __func__);

	/* deallocate isomgr bw */
	ret = vi_set_isomgr_request(tegra_vi, 0, 0);
	if (ret) {
		dev_err(&tegra_vi->ndev->dev,
		"%s: failed to deallocate memory in isomgr\n",
		__func__);
		return -ENOMEM;
	}

	return 0;
}
#endif

static int vi_set_la(struct vi *tegra_vi1, uint vi_bw)
{
	struct nvhost_device_data *pdata_vi1, *pdata_vi2;
	struct vi *tegra_vi2;
	struct clk *clk_vi;
	int ret;
	uint total_vi_bw;

	pdata_vi1 =
		(struct nvhost_device_data *)tegra_vi1->ndev->dev.platform_data;

	if (!pdata_vi1)
		return -ENODEV;

	/* Copy device data for other vi device */
	mutex_lock(&la_lock);

	tegra_vi1->vi_bw = vi_bw / 1000;
	total_vi_bw = tegra_vi1->vi_bw;
	if (pdata_vi1->master)
		pdata_vi2 = (struct nvhost_device_data *)
			pdata_vi1->master->dev.platform_data;
	else
		pdata_vi2 = (struct nvhost_device_data *)
			pdata_vi1->slave->dev.platform_data;

	if (!pdata_vi2) {
		mutex_unlock(&la_lock);
		return -ENODEV;
	}

	tegra_vi2 = (struct vi *)pdata_vi2->private_data;

	if (!tegra_vi2) {
		mutex_unlock(&la_lock);
		return -ENODATA;
	}

	clk_vi = clk_get(&tegra_vi2->ndev->dev, "emc");
	if (tegra_is_clk_enabled(clk_vi))
		total_vi_bw += tegra_vi2->vi_bw;

	mutex_unlock(&la_lock);

	ret = tegra_set_camera_ptsa(TEGRA_LA_VI_W, total_vi_bw, 1);

	return ret;
}


long vi_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct vi *tegra_vi = file->private_data;

	if (_IOC_TYPE(cmd) != NVHOST_VI_IOCTL_MAGIC)
		return -EFAULT;

	switch (cmd) {
	case NVHOST_VI_IOCTL_ENABLE_TPG: {
		uint enable;
		int ret;
		struct clk *clk;

		if (copy_from_user(&enable,
			(const void __user *)arg, sizeof(uint))) {
			dev_err(&tegra_vi->ndev->dev,
				"%s: Failed to copy arg from user\n", __func__);
			return -EFAULT;
		}

		clk = clk_get(&tegra_vi->ndev->dev, "pll_d");
		if (IS_ERR(clk))
			return -EINVAL;

		if (enable)
			ret = tegra_clk_cfg_ex(clk,
				TEGRA_CLK_PLLD_CSI_OUT_ENB, 1);
		else
			ret = tegra_clk_cfg_ex(clk,
				TEGRA_CLK_MIPI_CSI_OUT_ENB, 1);
		clk_put(clk);

		return ret;
	}
	case NVHOST_VI_IOCTL_SET_EMC_INFO: {
		uint vi_bw;
		int ret;
		if (copy_from_user(&vi_bw,
			(const void __user *)arg, sizeof(uint))) {
			dev_err(&tegra_vi->ndev->dev,
				"%s: Failed to copy arg from user\n", __func__);
			return -EFAULT;
		}

		ret = vi_set_la(tegra_vi, vi_bw);
		if (ret) {
			dev_err(&tegra_vi->ndev->dev,
			"%s: failed to set la for vi_bw %u MBps\n",
			__func__, vi_bw/1000);
			return -ENOMEM;
		}

#if defined(CONFIG_TEGRA_ISOMGR)
		/*
		 * Register VI as isomgr client.
		 */
		if (!tegra_vi->isomgr_handle) {
			ret = vi_isomgr_register(tegra_vi);
			if (ret) {
				dev_err(&tegra_vi->ndev->dev,
				"%s: failed to register VI as isomgr client\n",
				__func__);
				return -ENOMEM;
			}
		}

		/*
		 * Set VI ISO BW requirements.
		 * There is no way to figure out what latency
		 * can be tolerated in VI without reading VI
		 * registers for now. 3 usec is minimum time
		 * to switch PLL source. Let's put 4 usec as
		 * latency for now.
		 */
		ret = vi_set_isomgr_request(tegra_vi, vi_bw, 4);
		if (ret) {
			dev_err(&tegra_vi->ndev->dev,
			"%s: failed to reserve %u KBps\n",
			__func__, vi_bw);
			return -ENOMEM;
		}
#endif
		return ret;
	}
	default:
		dev_err(&tegra_vi->ndev->dev,
			"%s: Unknown vi ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int vi_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata;
	struct vi *vi;
	int err = 0;

	pdata = container_of(inode->i_cdev,
		struct nvhost_device_data, ctrl_cdev);
	if (WARN_ONCE(pdata == NULL, "pdata not found, %s failed\n", __func__))
		return -ENODEV;

	vi = (struct vi *)pdata->private_data;
	if (WARN_ONCE(vi == NULL, "vi not found, %s failed\n", __func__))
		return -ENODEV;

	file->private_data = vi;

	err = vi_enable_irq(vi);
	if (err)
		dev_err(&vi->ndev->dev, "%s: vi_enable_irq failed\n", __func__);

	return err;
}

static int vi_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct vi *tegra_vi = file->private_data;

	ret = vi_disable_irq(tegra_vi);
	if (ret) {
		dev_err(&tegra_vi->ndev->dev,
			"%s: vi_disable_irq failed\n",
			__func__);
		return ret;
	}

#if defined(CONFIG_TEGRA_ISOMGR)
	/* nullify isomgr request */
	if (tegra_vi->isomgr_handle) {
		ret = vi_isomgr_release(tegra_vi);
		if (ret) {
			dev_err(&tegra_vi->ndev->dev,
			"%s: failed to deallocate memory in isomgr\n",
			__func__);
			return -ENOMEM;
		}
	}
#endif

	return 0;
}

const struct file_operations tegra_vi_ctrl_ops = {
	.owner = THIS_MODULE,
	.open = vi_open,
	.unlocked_ioctl = vi_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vi_ioctl,
#endif
	.release = vi_release,
};
#endif

void nvhost_vi_reset(struct platform_device *pdev)
{
	void __iomem *reset_reg[4];
	int i;
	struct nvhost_device_data *pdata = pdev->dev.platform_data;

	if (pdev->id == 0) {
		reset_reg[0] = pdata->aperture[0] +
			       T12_VI_CSI_0_SW_RESET;
		reset_reg[1] = pdata->aperture[0] +
			       T12_CSI_CSI_SW_SENSOR_A_RESET;
		reset_reg[2] = pdata->aperture[0] +
			       T12_CSI_CSICIL_SW_SENSOR_A_RESET;
		reset_reg[3] = pdata->aperture[0] +
			       T12_VI_CSI_0_CSI_IMAGE_DT;
	} else {
		pdata = pdata->master->dev.platform_data;
		reset_reg[0] = pdata->aperture[0] +
			       T12_VI_CSI_1_SW_RESET;
		reset_reg[1] = pdata->aperture[0] +
			       T12_CSI_CSI_SW_SENSOR_B_RESET;
		reset_reg[2] = pdata->aperture[0] +
			       T12_CSI_CSICIL_SW_SENSOR_B_RESET;
		reset_reg[3] = pdata->aperture[0] +
			       T12_VI_CSI_1_CSI_IMAGE_DT;
	}

	writel(0, reset_reg[3]);
	writel(0x1, reset_reg[2]);
	writel(0x1, reset_reg[1]);
	writel(0x1f, reset_reg[0]);

	udelay(10);

	for (i = 2; i > 0; i--)
		writel(0, reset_reg[i]);
}

