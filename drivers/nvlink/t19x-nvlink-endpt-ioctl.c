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
#include <uapi/linux/tegra-nvlink-uapi.h>

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"

static bool is_nvlink_loopback_topology(struct tnvlink_dev *tdev)
{
	struct nvlink_device *ndev = tdev->ndev;
	struct nvlink_link *link = &ndev->link;

	if (link->device_id == NVLINK_ENDPT_T19X &&
		link->remote_dev_info.device_id == NVLINK_ENDPT_T19X)
		return true;

	return false;
}

static int t19x_nvlink_get_caps(struct tnvlink_dev *tdev,
			struct tegra_nvlink_caps *caps)
{
	if (is_nvlink_loopback_topology(tdev)) {
		caps->nvlink_caps |= (TEGRA_CTRL_NVLINK_CAPS_SUPPORTED |
					TEGRA_CTRL_NVLINK_CAPS_VALID);
	} else {
		/* TODO: */
	}

	/* Sysmem atomics are supported for NVLINK versions > 1.0 */
	if (NVLINK_IP_VERSION > TEGRA_NVLINK_VERSION_10)
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ATOMICS;

	switch (NVLINK_IP_VERSION) {
	case TEGRA_NVLINK_VERSION_22:
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
	case TEGRA_NVLINK_VERSION_20:
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

static int t19x_nvlink_get_status(struct tnvlink_dev *tdev,
				struct tegra_nvlink_status *status)
{
	struct nvlink_device *ndev = tdev->ndev;
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

	if (is_nvlink_loopback_topology(tdev)) {
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

	reg_val = nvlw_nvl_readl(tdev, NVL_SL1_CONFIG_RX);
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
	case TEGRA_NVLINK_VERSION_22:
		status->link_info.nvlink_version =
				TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_2;
		status->link_info.nci_version =
				TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_2;
		break;
	case TEGRA_NVLINK_VERSION_20:
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

	status->link_info.nvlink_link_clockKHz = ndev->link_bitrate / 1000;
	if (tdev->refclk == NVLINK_REFCLK_150)
		status->link_info.nvlink_ref_clk_speedKHz = 150000;
	else if (tdev->refclk == NVLINK_REFCLK_156)
		status->link_info.nvlink_ref_clk_speedKHz = 156250;

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

static int t19x_nvlink_clear_counters(struct tnvlink_dev *tdev,
			struct tegra_nvlink_clear_counters *clear_counters)
{
	u32 reg_val = 0;
	u32 counter_mask = clear_counters->counter_mask;

	if ((counter_mask) & (TEGRA_CTRL_NVLINK_COUNTER_TL_TX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)) {
		reg_val = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)
			reg_val |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_RESETTX0);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)
			reg_val |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_RESETTX1);
		nvlw_nvltlc_writel(tdev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL, reg_val);

		reg_val = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)
			reg_val |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_RESETRX0);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)
			reg_val |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_RESETRX1);
		nvlw_nvltlc_writel(tdev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL, reg_val);
	}

	if ((counter_mask) & (TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L0 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L1 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L2 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L3 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L4 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L5 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L6 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L7)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL1_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_LANE_CRC);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_RATES);
		nvlw_nvl_writel(tdev, NVL_SL1_ERROR_COUNT_CTRL, reg_val);
	}

	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL1_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_FLIT_CRC);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_RATES);
		nvlw_nvl_writel(tdev, NVL_SL1_ERROR_COUNT_CTRL, reg_val);
	}

	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL0_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_SL0_ERROR_COUNT_CTRL_CLEAR_REPLAY);
		nvlw_nvl_writel(tdev, NVL_SL0_ERROR_COUNT_CTRL, reg_val);
	}

	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY) {
		reg_val = nvlw_nvl_readl(tdev, NVL_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_ERROR_COUNT_CTRL_CLEAR_RECOVERY);
		nvlw_nvl_writel(tdev, NVL_ERROR_COUNT_CTRL, reg_val);
	}

	return 0;
}

static bool t19x_nvlink_is_lane_reversal(struct tnvlink_dev *tdev)
{
	u32 reg_val;
	bool lane_reversal;

	reg_val = nvlw_nvl_readl(tdev, NVL_SL1_CONFIG_RX);
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

static void t19x_nvlink_get_lane_crc_errors(struct tnvlink_dev *tdev,
				struct tegra_nvlink_get_counters *get_counters)
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

			if (t19x_nvlink_is_lane_reversal(tdev))
				lane_id = 7 - lane_id;

			if (lane_id < 4) {
				reg_val = nvlw_nvl_readl(tdev,
						NVL_SL1_ERROR_COUNT2_LANECRC);
			} else {
				reg_val = nvlw_nvl_readl(tdev,
						NVL_SL1_ERROR_COUNT3_LANECRC);
			}

			switch (lane_id) {
			case 0:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L0_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 1:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L1_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 2:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L2_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 3:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L3_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 4:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L4_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 5:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L5_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 6:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L6_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 7:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L7_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			default:
				break;
			}
		}
	}
}

static int t19x_nvlink_get_counters(struct tnvlink_dev *tdev,
				struct tegra_nvlink_get_counters *get_counters)
{
	u32 reg_val = 0;
	u64 reg_low = 0;
	u64 reg_hi = 0;
	u32 counter_mask = get_counters->counter_mask;

	if (counter_mask & (TEGRA_CTRL_NVLINK_COUNTER_TL_TX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)) {
		reg_low = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR0_LO);
		reg_hi = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR0_HI);
		if (reg_hi & BIT(NVLTLC_TX_DEBUG_TP_CNTR0_HI_ROLLOVER)) {
			get_counters->bTx0_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_TX_DEBUG_TP_CNTR0_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)] = 0;
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)] |=
				(((u64) 0xffffffff & reg_hi) << 32);

		reg_low = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR1_LO);
		reg_hi = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR1_HI);
		if (reg_hi & BIT(NVLTLC_TX_DEBUG_TP_CNTR1_HI_ROLLOVER)) {
			get_counters->bTx1_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_TX_DEBUG_TP_CNTR1_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)] = 0;
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)] |=
				(((u64) 0xffffffff & reg_hi) << 32);

		reg_low = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR0_LO);
		reg_hi = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR0_HI);
		if (reg_hi & BIT(NVLTLC_RX_DEBUG_TP_CNTR0_HI_ROLLOVER)) {
			get_counters->bRx0_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_RX_DEBUG_TP_CNTR0_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)] = 0;
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)] |=
				(((u64) 0xffffffff & reg_hi) << 32);

		reg_low = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR1_LO);
		reg_hi = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR1_HI);
		if (reg_hi & BIT(NVLTLC_RX_DEBUG_TP_CNTR1_HI_ROLLOVER)) {
			get_counters->bRx1_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_RX_DEBUG_TP_CNTR1_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)] = 0;
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)] |=
				(((u64) 0xffffffff & reg_hi) << 32);
	}

	/* Get the count of flit CRC errors */
	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL1_ERROR_COUNT1);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT)] =
			(u64)NVL_SL1_ERROR_COUNT1_FLIT_CRC_ERRORS_V(reg_val);
	}

	/* Get the count of lane CRC errors */
	t19x_nvlink_get_lane_crc_errors(tdev, get_counters);

	/* Get the count of replays for the link */
	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL0_ERROR_COUNT4);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
			TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY)] =
			(u64) NVL_SL0_ERROR_COUNT4_REPLAY_EVENTS_V(reg_val);
	}

	/*  Get the count of HW recoveries for the link */
	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY) {
		reg_val = nvlw_nvl_readl(tdev, NVL_ERROR_COUNT1);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
			TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY)] =
			(u64) NVL_ERROR_COUNT1_RECOVERY_EVENTS_V(reg_val);
	}

	return 0;
}

static int t19x_nvlink_get_err_info(struct tnvlink_dev *tdev,
				struct tegra_nvlink_get_err_info *get_err_info)
{
	struct nvlink_device *ndev = tdev->ndev;
	u32 reg_val = 0;
	u32 state = 0;
	bool excess_err_dl = false;

	get_err_info->link_mask = BIT(0);

	get_err_info->link_err_info.tl_err_log = 0;
	get_err_info->link_err_info.tl_intr_en = 0;

	get_err_info->link_err_info.tlc_tx_err_status0 =
				tdev->tlink.tlc_tx_err_status0;
	get_err_info->link_err_info.tlc_rx_err_status0 =
				tdev->tlink.tlc_rx_err_status0;
	get_err_info->link_err_info.tlc_rx_err_status1 =
				tdev->tlink.tlc_rx_err_status1;

	get_err_info->link_err_info.tlc_tx_err_log_en0 =
				nvlw_nvltlc_readl(tdev, NVLTLC_TX_ERR_STATUS_0);
	get_err_info->link_err_info.tlc_rx_err_log_en0 =
				nvlw_nvltlc_readl(tdev, NVLTLC_RX_ERR_STATUS_0);
	get_err_info->link_err_info.tlc_rx_err_log_en1 =
				nvlw_nvltlc_readl(tdev, NVLTLC_RX_ERR_STATUS_1);

	/* Reset Errlog after clients get the value */
	tdev->tlink.tlc_tx_err_status0 = 0;
	tdev->tlink.tlc_rx_err_status0 = 0;
	tdev->tlink.tlc_rx_err_status1 = 0;

	/* MIF blocks doesn't exist on tegra. Hence 0 the mif err fields */
	get_err_info->link_err_info.mif_tx_err_status0 = 0;
	get_err_info->link_err_info.mif_rx_err_status0 = 0;

	t19x_nvlink_get_tx_sublink_state(ndev, &state);
	get_err_info->link_err_info.dl_speed_status_tx = state;

	t19x_nvlink_get_rx_sublink_state(ndev, &state);
	get_err_info->link_err_info.dl_speed_status_rx = state;

	if (nvlw_nvl_readl(tdev, NVL_INTR_STALL_EN) &
				BIT(NVL_INTR_STALL_EN_RX_SHORT_ERROR_RATE)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_INTR);
		if (reg_val & BIT(NVL_INTR_RX_SHORT_ERROR_RATE)) {
			excess_err_dl = true;
			nvlw_nvl_writel(tdev, NVL_INTR, reg_val);
		}
	}

	get_err_info->link_err_info.bExcess_error_dl = excess_err_dl;

	return 0;
}

/* Get the number of successful error recoveries */
static int t19x_nvlink_get_error_recoveries(struct tnvlink_dev *tdev,
		struct tegra_nvlink_get_error_recoveries *get_err_recoveries)
{
	get_err_recoveries->num_recoveries = tdev->tlink.error_recoveries;
	/* Clear the counts */
	tdev->tlink.error_recoveries = 0;

	return 0;
}

static int t19x_nvlink_setup_eom(struct tnvlink_dev *tdev,
				struct tegra_nvlink_setup_eom *setup_eom)
{
	return minion_send_cmd(tdev, MINION_NVLINK_DL_CMD_COMMAND_CONFIGEOM,
			setup_eom->params);
}

static void nvlink_get_endpoint_state(struct tnvlink_dev *tdev,
			struct tegra_nvlink_link_state *link_state)
{
	struct nvlink_device *ndev = tdev->ndev;

	link_state->link_mode = t19x_nvlink_get_link_mode(ndev);
	link_state->tx_sublink_mode = t19x_nvlink_get_sublink_mode(ndev, 0);
	link_state->rx_sublink_mode = t19x_nvlink_get_sublink_mode(ndev, 1);
}


static int t19x_nvlink_train_intranode_conn(struct tnvlink_dev *tdev,
		struct tegra_nvlink_train_intranode_conn *train_intranode_conn)
{
	struct nvlink_device *ndev = tdev->ndev;
	int ret;

	if (train_intranode_conn->src_end_point.node_id !=
			train_intranode_conn->dst_end_point.node_id)
		return -EINVAL;

	if (train_intranode_conn->src_end_point.link_index !=
			ndev->link.link_id)
		return -EINVAL;

	if (train_intranode_conn->dst_end_point.link_index !=
			ndev->link.remote_dev_info.link_id)
		return -EINVAL;

	switch (train_intranode_conn->train_to) {
	case nvlink_train_conn_off_to_swcfg:
		ret = nvlink_transition_intranode_conn_off_to_safe(ndev);
		break;

	case nvlink_train_conn_swcfg_to_active:
		ret = nvlink_train_intranode_conn_safe_to_hs(ndev);
		break;

	case nvlink_train_conn_to_off:
		/* OFF state transitions are not supported/tested */
		nvlink_err("OFF state transitions are not supported");
		ret = -EINVAL;
		break;

	case nvlink_train_conn_active_to_swcfg:
		ret = nvlink_transition_intranode_conn_hs_to_safe(ndev);
		break;

	case nvlink_train_conn_swcfg_to_off:
		/* OFF state transitions are not supported/tested */
		nvlink_err("OFF state transitions are not supported");
		ret = -EINVAL;
		break;

	default:
		nvlink_err("Invalid training mode specified");
		ret = -EINVAL;
		break;
	}

	nvlink_get_endpoint_state(tdev, &train_intranode_conn->src_end_state);

	if (is_nvlink_loopback_topology(tdev)) {
		train_intranode_conn->dst_end_state.link_mode =
			train_intranode_conn->src_end_state.link_mode;
		train_intranode_conn->dst_end_state.tx_sublink_mode =
			train_intranode_conn->src_end_state.tx_sublink_mode;
		train_intranode_conn->dst_end_state.rx_sublink_mode =
			train_intranode_conn->src_end_state.rx_sublink_mode;
	} else {
		/* TODO: */
		/* Handle other topologies */
	}

	return ret;
}

/*
 * Get count for LP hardware counters that are specified in counter_valid_mask.
 * Clear unsupported ones in counter_valid_mask.
 */
static int t19x_nvlink_get_lp_counters(struct tnvlink_dev *tdev,
			struct tegra_nvlink_get_lp_counters *get_lp_counters)
{
	u32 counter_valid_mask_out = 0;
	u32 counter_valid_mask = get_lp_counters->counter_valid_mask;
	u32 reg_val;
	u32 cnt_idx;

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_NVHS;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_A);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_A_COUNT_TX_STATE_NVHS_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_EIGHTH;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_A);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_A_COUNT_TX_STATE_EIGHTH_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_OTHER;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_B);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_B_COUNT_TX_STATE_OTHER_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_NUM_TX_LP_ENTER;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_D);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_D_NUM_TX_LP_ENTER_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_NUM_TX_LP_EXIT;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_D);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_D_NUM_TX_LP_EXIT_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	get_lp_counters->counter_valid_mask = counter_valid_mask_out;

	return 0;
}

static int t19x_nvlink_clear_lp_counters(struct tnvlink_dev *tdev,
		struct tegra_nvlink_clear_lp_counters *clear_lp_counters)
{
	u32 reg_val;

	reg_val = nvlw_nvl_readl(tdev, NVL_STATS_CTRL);
	reg_val |= BIT(NVL_STATS_CTRL_CLEAR_ALL);
	nvlw_nvl_writel(tdev, NVL_STATS_CTRL, reg_val);

	return 0;
}

static long t19x_nvlink_endpt_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct tnvlink_dev *tdev = file->private_data;
	struct tegra_nvlink_caps *caps;
	struct tegra_nvlink_status *status;
	struct tegra_nvlink_clear_counters *clear_counters;
	struct tegra_nvlink_get_counters *get_counters;
	struct tegra_nvlink_get_err_info *get_err_info;
	struct tegra_nvlink_get_error_recoveries *get_err_recoveries;
	struct tegra_nvlink_setup_eom *setup_eom;
	struct tegra_nvlink_train_intranode_conn *train_intranode_conn;
	struct tegra_nvlink_get_lp_counters *get_lp_counters;
	struct tegra_nvlink_clear_lp_counters *clear_lp_counters;
	int arg_size = _IOC_SIZE(cmd);
	void *arg_copy;
	int ret = 0;

	if (!tdev) {
		nvlink_err("Invalid Tegra nvlink device");
		return -ENODEV;
	}

	arg_copy = kzalloc(arg_size, GFP_KERNEL);
	if (!arg_copy) {
		nvlink_err("Can't allocate memory for arg_copy, cmd 0x%x", cmd);
		return -ENOMEM;
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(arg_copy, (void __user *)arg, arg_size)) {
			nvlink_err("failed to copy ioctl data, cmd 0x%x", cmd);
			ret = -EFAULT;
			goto cleanup;
		}
	}

	switch (cmd) {
	case TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS:
		if (arg_size != sizeof(struct tegra_nvlink_caps)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		caps = (struct tegra_nvlink_caps *) arg_copy;
		ret = t19x_nvlink_get_caps(tdev, caps);
		if (ret < 0) {
			nvlink_err("nvlink get caps failed");
			goto cleanup;
		}

		break;

	case TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_STATUS:
		if (arg_size != sizeof(struct tegra_nvlink_status)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		status = (struct tegra_nvlink_status *) arg_copy;
		ret = t19x_nvlink_get_status(tdev, status);
		if (ret < 0) {
			nvlink_err("nvlink get status failed");
			goto cleanup;
		}

		break;

	case TEGRA_CTRL_CMD_NVLINK_CLEAR_COUNTERS:
		if (arg_size != sizeof(struct tegra_nvlink_clear_counters)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		clear_counters =
			(struct tegra_nvlink_clear_counters *) arg_copy;
		if (clear_counters->link_mask & 0x1) {
			ret = t19x_nvlink_clear_counters(tdev, clear_counters);
			if (ret < 0) {
				nvlink_err("nvlink clear counters failed");
				goto cleanup;
			}
		} else {
			nvlink_err("Invalid link mask specified");
			ret = -EINVAL;
			goto cleanup;
		}

		break;

	case TEGRA_CTRL_CMD_NVLINK_GET_COUNTERS:
		if (arg_size != sizeof(struct tegra_nvlink_get_counters)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		get_counters = (struct tegra_nvlink_get_counters *) arg_copy;
		if (get_counters->link_id != 0) {
			nvlink_err("Invalid link id specified");
			ret = -EINVAL;
			goto cleanup;
		}

		ret = t19x_nvlink_get_counters(tdev, get_counters);
		if (ret < 0) {
			nvlink_err("nvlink get counters failed");
			goto cleanup;
		}

		break;

	case TEGRA_CTRL_CMD_NVLINK_GET_ERR_INFO:
		if (arg_size != sizeof(struct tegra_nvlink_get_err_info)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		get_err_info = (struct tegra_nvlink_get_err_info *) arg_copy;
		ret = t19x_nvlink_get_err_info(tdev, get_err_info);
		if (ret < 0) {
			nvlink_err("nvlink get err info failed");
			goto cleanup;
		}

		break;

	case TEGRA_CTRL_CMD_NVLINK_GET_ERROR_RECOVERIES:
		if (arg_size !=
			sizeof(struct tegra_nvlink_get_error_recoveries)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		get_err_recoveries =
			(struct tegra_nvlink_get_error_recoveries *) arg_copy;
		if (get_err_recoveries->link_mask & 0x1) {
			ret = t19x_nvlink_get_error_recoveries(tdev,
							get_err_recoveries);
			if (ret < 0) {
				nvlink_err("nvlink get err recoveries failed");
				goto cleanup;
			}
		} else {
			nvlink_err("Invalid link mask specified");
			ret = -EINVAL;
			goto cleanup;
		}

		break;

	case TEGRA_CTRL_CMD_NVLINK_SETUP_EOM:
		if (arg_size != sizeof(struct tegra_nvlink_setup_eom)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		setup_eom = (struct tegra_nvlink_setup_eom *) arg_copy;
		if (setup_eom->link_id != 0) {
			nvlink_err("Invalid link id specified");
			ret = -EINVAL;
			goto cleanup;
		}

		ret = t19x_nvlink_setup_eom(tdev, setup_eom);
		if (ret < 0) {
			nvlink_err("nvlink setup eom failed");
			goto cleanup;
		}

		break;

	case TEGRA_CTRL_NVLINK_TRAIN_INTRANODE_CONN:
		if (arg_size !=
			sizeof(struct tegra_nvlink_train_intranode_conn)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		train_intranode_conn =
			(struct tegra_nvlink_train_intranode_conn *) arg_copy;
		ret = t19x_nvlink_train_intranode_conn(tdev,
				train_intranode_conn);
		train_intranode_conn->status = ret;
		if (ret < 0) {
			nvlink_err("nvlink train intranode conn failed");
			break;
		}

		break;

	case TEGRA_CTRL_CMD_NVLINK_GET_LP_COUNTERS:
		if (arg_size != sizeof(struct tegra_nvlink_get_lp_counters)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		get_lp_counters =
			(struct tegra_nvlink_get_lp_counters *) arg_copy;
		if (get_lp_counters->link_id != 0) {
			nvlink_err("Invalid link id specified");
			ret = -EINVAL;
			goto cleanup;
		}

		ret = t19x_nvlink_get_lp_counters(tdev, get_lp_counters);
		if (ret < 0) {
			nvlink_err("nvlink get LP counters failed");
			goto cleanup;
		}

		break;

	case TEGRA_CTRL_CMD_NVLINK_CLEAR_LP_COUNTERS:
		if (arg_size != sizeof(struct tegra_nvlink_clear_lp_counters)) {
			nvlink_err("invalid parameter passed, cmd 0x%x", cmd);
			ret = -EINVAL;
			goto cleanup;
		}

		clear_lp_counters =
			(struct tegra_nvlink_clear_lp_counters *) arg_copy;
		if (clear_lp_counters->link_id != 0) {
			nvlink_err("Invalid link id specified");
			ret = -EINVAL;
			goto cleanup;
		}

		ret = t19x_nvlink_clear_lp_counters(tdev, clear_lp_counters);
		if (ret < 0) {
			nvlink_err("nvlink clear LP counters failed");
			goto cleanup;
		}

		break;

	default:
		nvlink_err("Unsupported IOCTL call");
		ret = -EINVAL;
		goto cleanup;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		if (copy_to_user((void __user *)arg, arg_copy, arg_size)) {
			nvlink_err("Error while copying to userspace");
			ret = -EFAULT;
		}
	}

cleanup:
	kfree(arg_copy);
	return ret;
}

static int t19x_nvlink_endpt_open(struct inode *in, struct file *filp)
{
	int ret = 0;
	unsigned int minor = iminor(in);
	struct tnvlink_dev *tdev = container_of(in->i_cdev,
						struct tnvlink_dev,
						cdev);

	struct nvlink_device *ndev = tdev->ndev;

	if (minor > 0) {
		nvlink_err("Incorrect minor number");
		return -EBADFD;
	}

	ret = nvlink_enumerate(ndev);
	if (ret < 0)
		nvlink_err("Failed to enable the link!");
	else
		nvlink_dbg("Link enabled successfully!");

	filp->private_data = tdev;
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
