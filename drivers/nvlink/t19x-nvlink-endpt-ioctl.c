/*
 * t19x-nvlink-endpt-ioctl.c:
 * This file adds various IOCTLs for the Tegra NVLINK controller.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/uaccess.h>
#include <linux/clk.h>

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"
#include "nvlink-mods.h"

static bool is_nvlink_loopback_topology(struct nvlink_device *ndev)
{
	struct nvlink_link *link = &ndev->link;

	if (link->device_id == NVLINK_ENDPT_T19X &&
		link->remote_dev_info.device_id == NVLINK_ENDPT_T19X)
		return true;

	return false;
}

static int t19x_nvlink_get_caps(struct nvlink_device *ndev,
			struct nvlink_caps *caps)
{
	if (is_nvlink_loopback_topology(ndev)) {
		caps->nvlink_caps |= (TEGRA_CTRL_NVLINK_CAPS_SUPPORTED |
					TEGRA_CTRL_NVLINK_CAPS_VALID);
	} else {
		/* TODO: */
	}

	/* Sysmem atomics are supported for NVLINK versions > 1.0 */
	if (NVLINK_IP_VERSION > NVLINK_VERSION_10)
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ATOMICS;

	switch (NVLINK_IP_VERSION) {
	case NVLINK_VERSION_22:
		caps->lowest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_2;
		caps->highest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_2;
		caps->lowest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_2;
		caps->highest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_2;

		/* Supported power states */
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0;
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L2;
		break;
	case NVLINK_VERSION_20:
		caps->lowest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_0;
		caps->highest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_0;
		caps->lowest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_0;
		caps->highest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_0;

		/* Supported power states */
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0;
		break;
	default:
		caps->lowest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_1_0;
		caps->highest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_1_0;
		caps->lowest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_1_0;
		caps->highest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_1_0;

		/* Supported power states */
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0;
		break;
	}

	caps->enabled_link_mask = BIT(0);
	caps->discovered_link_mask = BIT(0);

	return 0;
}

static int t19x_nvlink_get_status(struct nvlink_device *ndev,
				struct nvlink_status *status)
{
	struct tegra_nvlink_device *tdev = ndev->priv;
	struct nvlink_link *link = &ndev->link;
	u32 reg_val = 0;
	u32 state = 0;

	/*
	 * Link should be connected and in HS mode, otherwise
	 * t19x_nvlink_endpt_open() will fail and we wouldn't be here.
	 */
	status->link_info.connected = true;

	status->link_info.remote_device_link_number =
					link->remote_dev_info.link_id;
	status->link_info.local_device_link_number = link->link_id;

	if (is_nvlink_loopback_topology(ndev)) {
		status->link_info.caps |= TEGRA_CTRL_NVLINK_CAPS_VALID;
		status->link_info.loop_property =
			TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_LOOPBACK;

		status->link_info.local_device_info.device_type =
			TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_TEGRA;
		status->link_info.local_device_info.domain = 0;
		status->link_info.local_device_info.bus = 0;
		status->link_info.local_device_info.device = 0;
		status->link_info.local_device_info.function = 0;
		status->link_info.local_device_info.pci_device_id = 0;

		status->link_info.remote_device_info.device_type =
			status->link_info.local_device_info.device_type;
		status->link_info.remote_device_info.domain =
			status->link_info.local_device_info.domain;
		status->link_info.remote_device_info.bus =
			status->link_info.local_device_info.bus;
		status->link_info.remote_device_info.device =
			status->link_info.local_device_info.device;
		status->link_info.remote_device_info.function =
			status->link_info.local_device_info.function;
		status->link_info.remote_device_info.pci_device_id =
			status->link_info.local_device_info.pci_device_id;
	} else {
		/* TODO: Handle other topologies */
	}

	status->enabled_link_mask = 1;
	status->link_info.phy_type = TEGRA_CTRL_NVLINK_STATUS_PHY_NVHS;
	status->link_info.sublink_width = 8;

	status->link_info.link_state = t19x_nvlink_get_link_state(ndev);

	t19x_nvlink_get_tx_sublink_state(ndev, &state);
	status->link_info.tx_sublink_status = (u8)state;

	t19x_nvlink_get_rx_sublink_state(ndev, &state);
	status->link_info.rx_sublink_status = (u8)state;

	reg_val = nvlw_nvl_readl(ndev, NVL_SL1_CONFIG_RX);
	if (reg_val & BIT(NVL_SL1_CONFIG_RX_REVERSAL_OVERRIDE)) {
		/* Overridden */
		if (reg_val & BIT(NVL_SL1_CONFIG_RX_LANE_REVERSE))
			status->link_info.bLane_reversal = true;
		else
			status->link_info.bLane_reversal = false;
	} else {
		/* Sensed in HW */
		if (reg_val & BIT(NVL_SL1_CONFIG_RX_HW_LANE_REVERSE))
			status->link_info.bLane_reversal = true;
		else
			status->link_info.bLane_reversal = false;
	}

	switch (NVLINK_IP_VERSION) {
	case NVLINK_VERSION_22:
		status->link_info.nvlink_version =
				TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_2;
		status->link_info.nci_version =
				TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_2;
		break;
	case NVLINK_VERSION_20:
		status->link_info.nvlink_version =
				TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_0;
		status->link_info.nci_version =
				TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_0;
		break;
	default:
		status->link_info.nvlink_version =
				TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_1_0;
		status->link_info.nci_version =
				TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_1_0;
		break;
	}
	status->link_info.phy_version =
				TEGRA_CTRL_NVLINK_STATUS_NVHS_VERSION_1_0;

	/* TODO: Change nvlink_link_clockKHz after having correct INITPLL */
	status->link_info.nvlink_link_clockKHz = 24750000;
	status->link_info.nvlink_ref_clk_speedKHz =
				clk_get_rate(tdev->clk_pllnvhs) / 1000;
	status->link_info.nvlink_common_clock_speedKHz =
				status->link_info.nvlink_link_clockKHz / 16;

	status->link_info.nvlink_link_clockMhz =
				status->link_info.nvlink_link_clockKHz / 1000;
	status->link_info.nvlink_ref_clk_speedMhz =
			status->link_info.nvlink_ref_clk_speedKHz / 1000;
	status->link_info.nvlink_common_clock_speedMhz =
			status->link_info.nvlink_common_clock_speedKHz / 1000;

	status->link_info.nvlink_ref_clk_type =
				TEGRA_CTRL_NVLINK_REFCLK_TYPE_NVHS;

	return 0;
}

static long t19x_nvlink_endpt_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct nvlink_device *ndev = file->private_data;
	struct nvlink_caps *caps;
	struct nvlink_status *status;
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid nvlink device");
		return -ENODEV;
	}

	switch (cmd) {
	case TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS:
		caps = devm_kzalloc(ndev->dev, sizeof(struct nvlink_caps),
					GFP_KERNEL);
		if (!caps) {
			nvlink_err("Can't allocate memory for nvlink caps");
			return -ENOMEM;
		}

		ret = t19x_nvlink_get_caps(ndev, caps);
		if (ret < 0) {
			nvlink_err("nvlink get caps failed");
			kfree(caps);
			break;
		}
		ret = copy_to_user((void __user *)arg, caps, sizeof(*caps));
		if (ret) {
			nvlink_err("Error while copying caps to userspace");
			ret = -EFAULT;
		}
		kfree(caps);
		break;

	case TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_STATUS:
		status = devm_kzalloc(ndev->dev, sizeof(struct nvlink_status),
					GFP_KERNEL);
		if (!status) {
			nvlink_err("Can't allocate memory for nvlink status");
			return -ENOMEM;
		}

		ret = t19x_nvlink_get_status(ndev, status);
		if (ret < 0) {
			nvlink_err("nvlink get status failed");
			kfree(status);
			break;
		}
		ret = copy_to_user((void __user *)arg, status, sizeof(*status));
		if (ret) {
			nvlink_err("Error while copying status to userspace");
			ret = -EFAULT;
		}
		kfree(status);
		break;

	default:
		nvlink_err("Unsupported IOCTL call");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int t19x_nvlink_endpt_open(struct inode *in, struct file *filp)
{
	int ret = 0;
	unsigned int minor = iminor(in);
	struct nvlink_device *ndev = container_of(in->i_cdev,
						struct nvlink_device,
						cdev);

	if (minor > 0) {
		nvlink_err("Incorrect minor number");
		return -EBADFD;
	}

	ret = nvlink_init_link(ndev);

	filp->private_data = ndev;

	return ret;
}

static int t19x_nvlink_endpt_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* File ops for device node */
const struct file_operations t19x_nvlink_endpt_ops = {
	.owner = THIS_MODULE,
	.open = t19x_nvlink_endpt_open,
	.release = t19x_nvlink_endpt_release,
	.unlocked_ioctl = t19x_nvlink_endpt_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = t19x_nvlink_endpt_ioctl,
#endif
};
