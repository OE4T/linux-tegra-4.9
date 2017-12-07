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

static int t19x_nvlink_clear_counters(struct nvlink_device *ndev,
				struct nvlink_clear_counters *clear_counters)
{
	u32 reg_val = 0;
	u32 counter_mask = clear_counters->counter_mask;

	if ((counter_mask) & (TEGRA_CTRL_NVLINK_COUNTER_TL_TX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)) {
		reg_val = nvlw_nvltlc_readl(ndev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)
			reg_val |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_RESETTX0);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)
			reg_val |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_RESETTX1);
		nvlw_nvltlc_writel(ndev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL, reg_val);

		reg_val = nvlw_nvltlc_readl(ndev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)
			reg_val |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_RESETRX0);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)
			reg_val |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_RESETRX1);
		nvlw_nvltlc_writel(ndev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL, reg_val);
	}

	if ((counter_mask) & (TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L0 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L1 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L2 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L3 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L4 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L5 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L6 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L7)) {
		reg_val = nvlw_nvl_readl(ndev, NVL_SL1_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_LANE_CRC);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_RATES);
		nvlw_nvl_writel(ndev, NVL_SL1_ERROR_COUNT_CTRL, reg_val);
	}

	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT) {
		reg_val = nvlw_nvl_readl(ndev, NVL_SL1_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_FLIT_CRC);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_RATES);
		nvlw_nvl_writel(ndev, NVL_SL1_ERROR_COUNT_CTRL, reg_val);
	}

	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY) {
		reg_val = nvlw_nvl_readl(ndev, NVL_SL0_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_SL0_ERROR_COUNT_CTRL_CLEAR_REPLAY);
		nvlw_nvl_writel(ndev, NVL_SL0_ERROR_COUNT_CTRL, reg_val);
	}

	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY) {
		reg_val = nvlw_nvl_readl(ndev, NVL_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_ERROR_COUNT_CTRL_CLEAR_RECOVERY);
		nvlw_nvl_writel(ndev, NVL_ERROR_COUNT_CTRL, reg_val);
	}

	return 0;
}

static bool t19x_nvlink_is_lane_reversal(struct nvlink_device *ndev)
{
	u32 reg_val;
	bool lane_reversal;

	reg_val = nvlw_nvl_readl(ndev, NVL_SL1_CONFIG_RX);
	if (reg_val & BIT(NVL_SL1_CONFIG_RX_REVERSAL_OVERRIDE)) {
		if (reg_val & BIT(NVL_SL1_CONFIG_RX_LANE_REVERSE))
			lane_reversal = true;
		else
			lane_reversal = false;
	} else {
		if (reg_val & BIT(NVL_SL1_CONFIG_RX_HW_LANE_REVERSE))
			lane_reversal = true;
		else
			lane_reversal = false;
	}

	return lane_reversal;
}

static void t19x_nvlink_get_lane_crc_errors(struct nvlink_device *ndev,
				struct nvlink_get_counters *get_counters)
{
	int i;
	int lane_id;
	u32 reg_val;
	u64 lane_crc_val;
	u32 counter_mask = get_counters->counter_mask;

	for (i = 0; i < TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_SIZE;
				i++) {
		if (counter_mask &
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L(i)) {

			lane_id = i;

			if (t19x_nvlink_is_lane_reversal(ndev))
				lane_id = 7 - lane_id;

			if (lane_id < 4) {
				reg_val = nvlw_nvl_readl(ndev,
						NVL_SL1_ERROR_COUNT2_LANECRC);
			} else {
				reg_val = nvlw_nvl_readl(ndev,
						NVL_SL1_ERROR_COUNT3_LANECRC);
			}

			switch (lane_id) {
			case 0:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L0_V(
					reg_val);
				get_counters->nvlink_counters[
					nvlink_counter(i)] = lane_crc_val;
				break;
			case 1:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L1_V(
					reg_val);
				get_counters->nvlink_counters[
					nvlink_counter(i)] = lane_crc_val;
				break;
			case 2:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L2_V(
					reg_val);
				get_counters->nvlink_counters[
					nvlink_counter(i)] = lane_crc_val;
				break;
			case 3:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L3_V(
					reg_val);
				get_counters->nvlink_counters[
					nvlink_counter(i)] = lane_crc_val;
				break;
			case 4:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L4_V(
					reg_val);
				get_counters->nvlink_counters[
					nvlink_counter(i)] = lane_crc_val;
				break;
			case 5:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L5_V(
					reg_val);
				get_counters->nvlink_counters[
					nvlink_counter(i)] = lane_crc_val;
				break;
			case 6:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L6_V(
					reg_val);
				get_counters->nvlink_counters[
					nvlink_counter(i)] = lane_crc_val;
				break;
			case 7:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L7_V(
					reg_val);
				get_counters->nvlink_counters[
					nvlink_counter(i)] = lane_crc_val;
				break;
			default:
				break;
			}
		}
	}
}

static int t19x_nvlink_get_counters(struct nvlink_device *ndev,
				struct nvlink_get_counters *get_counters)
{
	u32 reg_val = 0;
	u64 reg_low = 0;
	u64 reg_hi = 0;
	u32 counter_mask = get_counters->counter_mask;

	if (counter_mask & (TEGRA_CTRL_NVLINK_COUNTER_TL_TX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)) {
		reg_low = nvlw_nvltlc_readl(ndev, NVLTLC_TX_DEBUG_TP_CNTR0_LO);
		reg_hi = nvlw_nvltlc_readl(ndev, NVLTLC_TX_DEBUG_TP_CNTR0_HI);
		if (reg_hi & BIT(NVLTLC_TX_DEBUG_TP_CNTR0_HI_ROLLOVER)) {
			get_counters->bTx0_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_TX_DEBUG_TP_CNTR0_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)] = 0;
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)] |=
				(((u64) 0xffffffff & reg_hi) << 32);

		reg_low = nvlw_nvltlc_readl(ndev, NVLTLC_TX_DEBUG_TP_CNTR1_LO);
		reg_hi = nvlw_nvltlc_readl(ndev, NVLTLC_TX_DEBUG_TP_CNTR1_HI);
		if (reg_hi & BIT(NVLTLC_TX_DEBUG_TP_CNTR1_HI_ROLLOVER)) {
			get_counters->bTx1_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_TX_DEBUG_TP_CNTR1_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)] = 0;
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)] |=
				(((u64) 0xffffffff & reg_hi) << 32);

		reg_low = nvlw_nvltlc_readl(ndev, NVLTLC_RX_DEBUG_TP_CNTR0_LO);
		reg_hi = nvlw_nvltlc_readl(ndev, NVLTLC_RX_DEBUG_TP_CNTR0_HI);
		if (reg_hi & BIT(NVLTLC_RX_DEBUG_TP_CNTR0_HI_ROLLOVER)) {
			get_counters->bRx0_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_RX_DEBUG_TP_CNTR0_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)] = 0;
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)] |=
				(((u64) 0xffffffff & reg_hi) << 32);

		reg_low = nvlw_nvltlc_readl(ndev, NVLTLC_RX_DEBUG_TP_CNTR1_LO);
		reg_hi = nvlw_nvltlc_readl(ndev, NVLTLC_RX_DEBUG_TP_CNTR1_HI);
		if (reg_hi & BIT(NVLTLC_RX_DEBUG_TP_CNTR1_HI_ROLLOVER)) {
			get_counters->bRx1_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_RX_DEBUG_TP_CNTR1_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)] = 0;
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)] |=
				(((u64) 0xffffffff & reg_hi) << 32);
	}

	/* Get the count of flit CRC errors */
	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT) {
		reg_val = nvlw_nvl_readl(ndev, NVL_SL1_ERROR_COUNT1);
		get_counters->nvlink_counters[BIT_IDX_32(
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT)] =
			(u64)NVL_SL1_ERROR_COUNT1_FLIT_CRC_ERRORS_V(reg_val);
	}

	/* Get the count of lane CRC errors */
	t19x_nvlink_get_lane_crc_errors(ndev, get_counters);

	/* Get the count of replays for the link */
	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY) {
		reg_val = nvlw_nvl_readl(ndev, NVL_SL0_ERROR_COUNT4);
		get_counters->nvlink_counters[BIT_IDX_32(
			TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY)] =
			(u64) NVL_SL0_ERROR_COUNT4_REPLAY_EVENTS_V(reg_val);
	}

	/*  Get the count of HW recoveries for the link */
	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY) {
		reg_val = nvlw_nvl_readl(ndev, NVL_ERROR_COUNT1);
		get_counters->nvlink_counters[BIT_IDX_32(
			TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY)] =
			(u64) NVL_ERROR_COUNT1_RECOVERY_EVENTS_V(reg_val);
	}

	return 0;
}

static long t19x_nvlink_endpt_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct nvlink_device *ndev = file->private_data;
	struct nvlink_caps *caps;
	struct nvlink_status *status;
	struct nvlink_clear_counters *clear_counters;
	struct nvlink_get_counters *get_counters;
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

	case TEGRA_CTRL_CMD_NVLINK_CLEAR_COUNTERS:
		clear_counters = devm_kzalloc(ndev->dev,
				sizeof(struct nvlink_clear_counters),
				GFP_KERNEL);
		if (!clear_counters) {
			nvlink_err("Can't allocate memory for clear counters");
			return -ENOMEM;
		}

		ret = copy_from_user(clear_counters, (void __user *)arg,
					sizeof(*clear_counters));
		if (ret) {
			nvlink_err("Error while copying from userspace");
			ret = -EFAULT;
			kfree(clear_counters);
			break;
		}

		if (clear_counters->link_mask & 0x1) {
			ret = t19x_nvlink_clear_counters(ndev, clear_counters);
			if (ret < 0) {
				nvlink_err("nvlink clear counters failed");
				kfree(clear_counters);
				break;
			}
			ret = copy_to_user((void __user *)arg, clear_counters,
						sizeof(*clear_counters));
			if (ret) {
				nvlink_err("Error while copying clear"
						" counters");
				ret = -EFAULT;
			}
		} else {
			nvlink_err("Invalid link mask specified");
			ret = -EINVAL;
		}
		kfree(clear_counters);
		break;

	case TEGRA_CTRL_CMD_NVLINK_GET_COUNTERS:
		get_counters = devm_kzalloc(ndev->dev,
					sizeof(struct nvlink_get_counters),
					GFP_KERNEL);
		if (!get_counters) {
			nvlink_err("Can't allocate memory for get counters");
			return -ENOMEM;
		}

		ret = copy_from_user(get_counters, (void __user *)arg,
					sizeof(*get_counters));
		if (ret) {
			nvlink_err("Error while copying from userspace");
			ret = -EFAULT;
			kfree(get_counters);
			break;
		}

		if (get_counters->link_id != 0) {
			nvlink_err("Invalid link id specified");
			ret = -EINVAL;
			kfree(get_counters);
			break;
		}

		ret = t19x_nvlink_get_counters(ndev, get_counters);
		if (ret < 0) {
			nvlink_err("nvlink get counters failed");
			kfree(get_counters);
			break;
		}

		ret = copy_to_user((void __user *)arg, get_counters,
					sizeof(*get_counters));
		if (ret) {
			nvlink_err("Error while copying get counters");
			ret = -EFAULT;
		}
		kfree(get_counters);
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
