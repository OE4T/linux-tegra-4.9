/*
 * drivers/video/tegra/host/vi/tegra_vi.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/module.h>
#include <linux/nvhost.h>
#include <linux/nvhost_vi_ioctl.h>
#include <linux/platform_device.h>

#include "bus_client.h"
#include "chip_support.h"
#include "host1x/host1x.h"
#include "vi.h"

#define T12_VI_CFG_CG_CTRL	0x2e
#define T12_CG_2ND_LEVEL_EN	1
#define T12_VI_CSI_0_SW_RESET	0x40
#define T12_VI_CSI_1_SW_RESET	0x80
#define T12_VI_CSI_SW_RESET_MCCIF_RESET 3

#ifdef TEGRA_12X_OR_HIGHER_CONFIG
int nvhost_vi_init(struct platform_device *dev)
{
	int ret = 0;
	struct vi *tegra_vi;
	tegra_vi = (struct vi *)nvhost_get_private_data(dev);

	tegra_vi->reg = regulator_get(&dev->dev, "avdd_dsi_csi");
	if (IS_ERR(tegra_vi->reg)) {
		if (tegra_vi->reg == ERR_PTR(-ENODEV)) {
			ret = -ENODEV;
			dev_info(&dev->dev,
				"%s: no regulator device\n",
				__func__);
		} else {
			dev_err(&dev->dev,
				"%s: couldn't get regulator\n",
				__func__);
		}
		tegra_vi->reg = NULL;
	}
	return ret;
}

void nvhost_vi_deinit(struct platform_device *dev)
{
	struct vi *tegra_vi;
	tegra_vi = (struct vi *)nvhost_get_private_data(dev);

	if (tegra_vi->reg) {
		regulator_put(tegra_vi->reg);
		tegra_vi->reg = NULL;
	}
}

int nvhost_vi_finalize_poweron(struct platform_device *dev)
{
	int ret = 0;
	struct vi *tegra_vi;
	tegra_vi = (struct vi *)nvhost_get_private_data(dev);

	if (tegra_vi->reg) {
		ret = regulator_enable(tegra_vi->reg);
		if (ret)
			dev_err(&dev->dev,
				"%s: enable csi regulator failed.\n",
				__func__);
	}

	if (nvhost_client_can_writel(dev))
		nvhost_client_writel(dev,
				T12_CG_2ND_LEVEL_EN, T12_VI_CFG_CG_CTRL);
	return ret;
}

int nvhost_vi_prepare_poweroff(struct platform_device *dev)
{
	int ret = 0;
	struct vi *tegra_vi;
	tegra_vi = (struct vi *)nvhost_get_private_data(dev);

	if (tegra_vi->reg) {
		ret = regulator_disable(tegra_vi->reg);
		if (ret)
			dev_err(&dev->dev,
				"%s: disable csi regulator failed.\n",
				__func__);
	}
	return ret;
}

long vi_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct vi *tegra_vi;

	if (_IOC_TYPE(cmd) != NVHOST_VI_IOCTL_MAGIC)
		return -EFAULT;

	tegra_vi = file->private_data;
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

	pdata = container_of(inode->i_cdev,
		struct nvhost_device_data, ctrl_cdev);
	BUG_ON(pdata == NULL);

	vi = (struct vi *)pdata->private_data;
	BUG_ON(vi == NULL);

	file->private_data = vi;
	return 0;
}

static int vi_release(struct inode *inode, struct file *file)
{
	return 0;
}

const struct file_operations tegra_vi_ctrl_ops = {
	.owner = THIS_MODULE,
	.open = vi_open,
	.unlocked_ioctl = vi_ioctl,
	.release = vi_release,
};
#endif

void nvhost_vi_reset(struct platform_device *pdev)
{
	u32 reset_reg;

	if (pdev->id == 0)
		reset_reg = T12_VI_CSI_0_SW_RESET;
	else
		reset_reg = T12_VI_CSI_1_SW_RESET;

	nvhost_client_writel(pdev, T12_VI_CSI_SW_RESET_MCCIF_RESET,
			reset_reg);

	udelay(10);

	nvhost_client_writel(pdev, 0, reset_reg);
}

