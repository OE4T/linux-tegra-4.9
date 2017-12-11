/*
 * t19x-nvlink-endpt-link.c:
 * This file contains link state transition and link trainig code for the Tegra
 * NVLINK controller.
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"

#define SUBLINK_TIMEOUT_MS	200 /* msec */

u32 t19x_nvlink_get_link_state(struct nvlink_device *ndev)
{
	return (nvlw_nvl_readl(ndev, NVL_LINK_STATE) &
				NVL_LINK_STATE_STATE_MASK);
}

u32 t19x_nvlink_get_link_mode(struct nvlink_device *ndev)
{
	u32 link_mode;
	u32 link_state = t19x_nvlink_get_link_state(ndev);

	switch (link_state) {
	case NVL_LINK_STATE_STATE_INIT:
		link_mode = NVLINK_LINK_OFF;
		break;
	case NVL_LINK_STATE_STATE_HWCFG:
		link_mode = NVLINK_LINK_DETECT;
		break;
	case NVL_LINK_STATE_STATE_SWCFG:
		link_mode = NVLINK_LINK_SAFE;
		break;
	case NVL_LINK_STATE_STATE_ACTIVE:
		link_mode = NVLINK_LINK_HS;
		break;
	case NVL_LINK_STATE_STATE_FAULT:
		link_mode = NVLINK_LINK_FAULT;
		break;
	case NVL_LINK_STATE_STATE_RCVY_AC:
	case NVL_LINK_STATE_STATE_RCVY_SW:
	case NVL_LINK_STATE_STATE_RCVY_RX:
		link_mode = NVLINK_LINK_RECOVERY;
		break;
	default:
		link_mode = NVLINK_LINK_OFF;
		break;
	}

	return link_mode;
}

u32 t19x_nvlink_get_sublink_mode(struct nvlink_device *ndev, bool is_rx_sublink)
{
	u32 reg_val;
	u8 state;
	u32 sublink_mode;

	if (!is_rx_sublink) {
		reg_val = nvlw_nvl_readl(ndev, NVL_SL0_SLSM_STATUS_TX);
		state = (reg_val & NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_MASK) >>
				NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_SHIFT;

		switch (state) {
		case NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_EIGHTH:
			sublink_mode = NVLINK_TX_SINGLE_LANE;
			break;

		case NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_HS:
			sublink_mode = NVLINK_TX_HS;
			break;

		case NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_SAFE:
			sublink_mode = NVLINK_TX_SAFE;
			break;

		case NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_OFF:
			sublink_mode = NVLINK_TX_OFF;
			break;

		default:
			sublink_mode = NVLINK_TX_OFF;
			break;
		}
	} else {
		reg_val = nvlw_nvl_readl(ndev, NVL_SL1_SLSM_STATUS_RX);
		state = (reg_val & NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_MASK) >>
				NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_SHIFT;

		switch (state) {
		case NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_EIGHTH:
			sublink_mode = NVLINK_RX_SINGLE_LANE;
			break;

		case NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_HS:
			sublink_mode = NVLINK_RX_HS;
			break;

		case NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_SAFE:
			sublink_mode = NVLINK_RX_SAFE;
			break;

		case NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_OFF:
			sublink_mode = NVLINK_RX_OFF;
			break;

		default:
			sublink_mode = NVLINK_RX_OFF;
			break;
		}
	}

	return sublink_mode;
}

void t19x_nvlink_get_tx_sublink_state(struct nvlink_device *ndev,
				u32 *tx_sublink_state)
{
	u32 reg_val;

	reg_val = nvlw_nvl_readl(ndev, NVL_SL0_SLSM_STATUS_TX);
	*tx_sublink_state = (reg_val &
			NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_MASK) >>
			NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_SHIFT;
}

void t19x_nvlink_get_rx_sublink_state(struct nvlink_device *ndev,
				u32 *rx_sublink_state)
{
	u32 reg_val;

	reg_val = nvlw_nvl_readl(ndev, NVL_SL1_SLSM_STATUS_RX);
	*rx_sublink_state = (reg_val &
			NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_MASK) >>
			NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_SHIFT;
}

int t19x_nvlink_poll_link_state(struct nvlink_device *ndev, u32 link_state,
			u32 timeout_ms)
{
	u32 link_mode;
	u32 timeout_us = timeout_ms * 1000;

	link_mode = t19x_nvlink_get_link_mode(ndev);
	while (link_mode != link_state) {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US * 2);
		timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
		if (timeout_us <= 0) {
			nvlink_err("Timeout occurred while polling on link");
			return -ETIMEDOUT;
		}
		link_mode = t19x_nvlink_get_link_mode(ndev);
	}

	return 0;
}
int t19x_nvlink_poll_tx_sublink_state(struct nvlink_device *ndev,
				u32 tx_sublink_state, u32 timeout_ms)
{
	u32 sublink_mode;
	u32 timeout_us = timeout_ms * 1000;

	sublink_mode = t19x_nvlink_get_sublink_mode(ndev, false);
	while (sublink_mode != tx_sublink_state) {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US * 2);
		timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
		if (timeout_us <= 0) {
			nvlink_err("Timeout while polling on Tx sublink");
			return -ETIMEDOUT;
		}
		sublink_mode = t19x_nvlink_get_sublink_mode(ndev, false);
	}

	return 0;
}

int t19x_nvlink_poll_rx_sublink_state(struct nvlink_device *ndev,
				u32 rx_sublink_state, u32 timeout_ms)
{
	u32 sublink_mode;
	u32 timeout_us = timeout_ms * 1000;

	sublink_mode = t19x_nvlink_get_sublink_mode(ndev, true);
	while (sublink_mode != rx_sublink_state) {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US * 2);
		timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
		if (timeout_us <= 0) {
			nvlink_err("Timeout while polling on Rx sublink");
			return -ETIMEDOUT;
		}
		sublink_mode = t19x_nvlink_get_sublink_mode(ndev, true);
	}

	return 0;
}

int t19x_nvlink_poll_sublink_state(struct nvlink_device *ndev0,
				u32 tx_sublink_state,
				struct nvlink_device *ndev1,
				u32 rx_sublink_state,
				u32 timeout_ms)
{
	int status;

	status = t19x_nvlink_poll_tx_sublink_state(ndev0, tx_sublink_state,
						timeout_ms);
	if (status) {
		/* polling on tx sublink failed. skip any rx polling */
		return status;
	}

	status = t19x_nvlink_poll_rx_sublink_state(ndev1, rx_sublink_state,
						timeout_ms);

	return status;
}

/* Configure and Start the PRBS generator */
int t19x_nvlink_prbs_gen_en(struct nvlink_device *ndev)
{
	u32 reg_val;

	/* Not specified in doc but is required as per HW team */
	nvlw_nvl_writel(ndev, NVL_SL1_RXSLSM_TIMEOUT_2, 0);

	/*
	 * Minion will set these *_PBRS_* and *_SCRAM_* registers
	 * as a part of DLPL init.
	 *
	 * Note: A remote endpoint is expected to set exact same *_PBRS_*
	 * and *_SCRAM_* values. Otherwise, link training fails.
	 */

	reg_val = nvlw_nvl_readl(ndev, NVL_TXIOBIST_CONFIG);
	reg_val |= BIT(NVL_TXIOBIST_CONFIG_DPG_PRBSSEEDLD);
	nvlw_nvl_writel(ndev, NVL_TXIOBIST_CONFIG, reg_val);

	reg_val = nvlw_nvl_readl(ndev, NVL_TXIOBIST_CONFIG);
	reg_val &= ~BIT(NVL_TXIOBIST_CONFIG_DPG_PRBSSEEDLD);
	nvlw_nvl_writel(ndev, NVL_TXIOBIST_CONFIG, reg_val);

	return 0;
}

/* Put RX in calibration */
int t19x_nvlink_rxcal_enable(struct nvlink_device *ndev)
{
	/* TODO: Put RX in calibration */

	return 0;
}

int t19x_nvlink_set_sublink_mode(struct nvlink_device *ndev, bool is_rx_sublink,
				u32 mode)
{
	u32 rx_sublink_state, tx_sublink_state;
	u32 reg_val, timeout_us;
	int status = 0;

	timeout_us = SUBLINK_TIMEOUT_MS * 1000;

	t19x_nvlink_get_tx_sublink_state(ndev, &tx_sublink_state);
	t19x_nvlink_get_rx_sublink_state(ndev, &rx_sublink_state);

	/* Check if SLSM is ready to accept a sublink change request */
	do {
		reg_val = nvlw_nvl_readl(ndev, NVL_SUBLINK_CHANGE);
		if ((reg_val & NVL_SUBLINK_CHANGE_STATUS_MASK) ==
					NVL_SUBLINK_CHANGE_STATUS_DONE)
			break;

		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US * 2);
		timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
	} while (timeout_us > 0);

	if (timeout_us <= 0) {
		nvlink_err("SLSM not ready to accept a state change request");
		return -EINVAL;
	}

	if (!is_rx_sublink) {
		switch (mode) {
		case NVLINK_TX_COMMON:
			/* TODO  */
			break;

		case NVLINK_TX_COMMON_DISABLE:
			/* This is a NOP for the GPU side.
			 * Need to check if anything needs to be done for Tegra.
			 */
			break;

		case NVLINK_TX_DATA_READY:
			/*TODO */
			break;

		case NVLINK_TX_PRBS_EN:
			status = t19x_nvlink_prbs_gen_en(ndev);
			if (status) {
				nvlink_err("Unable to start PRBS generator"
					" for link");
				return status;
			}

			break;

		case NVLINK_TX_HS:
			if (tx_sublink_state ==
				NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_HS) {
				nvlink_dbg("TX already in High Speed mode");
				break;
			} else if (tx_sublink_state ==
				NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_OFF) {
				nvlink_err("TX cannot be taken from OFF to"
					" High Speed directly");
				return -EPERM;
			}

			nvlink_dbg("Changing TX sublink state to High Speed");
			reg_val = nvlw_nvl_readl(ndev, NVL_SUBLINK_CHANGE);
			reg_val &= ~NVL_SUBLINK_CHANGE_NEWSTATE_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_NEWSTATE_F(
					NVL_SUBLINK_CHANGE_NEWSTATE_HS);
			reg_val &= ~NVL_SUBLINK_CHANGE_SUBLINK_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_SUBLINK_F(
					NVL_SUBLINK_CHANGE_SUBLINK_TX);
			reg_val &= ~NVL_SUBLINK_CHANGE_ACTION_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_ACTION_F(
					NVL_SUBLINK_CHANGE_ACTION_SLSM_CHANGE);
			nvlw_nvl_writel(ndev, NVL_SUBLINK_CHANGE, reg_val);

			timeout_us = SUBLINK_TIMEOUT_MS * 1000;
			do {
				reg_val = nvlw_nvl_readl(ndev,
							NVL_SUBLINK_CHANGE);
				if ((reg_val &
				NVL_SUBLINK_CHANGE_STATUS_MASK) ==
				NVL_SUBLINK_CHANGE_STATUS_DONE)
					break;
				else if ((reg_val &
				NVL_SUBLINK_CHANGE_STATUS_MASK) ==
				NVL_SUBLINK_CHANGE_STATUS_FAULT) {
					nvlink_err("Fault while changing TX"
						" sublink to High Speed");
					return -EPROTO;
				}
				usleep_range(DEFAULT_LOOP_SLEEP_US,
					DEFAULT_LOOP_SLEEP_US * 2);
				timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
			} while (timeout_us > 0);

			if (timeout_us <= 0) {
				nvlink_err("Timeout while waiting for TX"
					" sublink to go to High Speed");
				return -ETIMEDOUT;
			}

			break;

		case NVLINK_TX_SAFE:
			if (tx_sublink_state ==
				NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_SAFE) {
				nvlink_dbg("TX already in Safe mode");
				break;
			}

			nvlink_dbg("Changing TX sublink state to Safe mode");
			reg_val = nvlw_nvl_readl(ndev, NVL_SUBLINK_CHANGE);
			reg_val &= ~NVL_SUBLINK_CHANGE_NEWSTATE_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_NEWSTATE_F(
					NVL_SUBLINK_CHANGE_NEWSTATE_SAFE);
			reg_val &= ~NVL_SUBLINK_CHANGE_SUBLINK_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_SUBLINK_F(
					NVL_SUBLINK_CHANGE_SUBLINK_TX);
			reg_val &= ~NVL_SUBLINK_CHANGE_ACTION_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_ACTION_F(
					NVL_SUBLINK_CHANGE_ACTION_SLSM_CHANGE);
			nvlw_nvl_writel(ndev, NVL_SUBLINK_CHANGE, reg_val);

			timeout_us = SUBLINK_TIMEOUT_MS * 1000;
			do {
				reg_val = nvlw_nvl_readl(ndev,
							NVL_SUBLINK_CHANGE);
				if ((reg_val &
				NVL_SUBLINK_CHANGE_STATUS_MASK) ==
				NVL_SUBLINK_CHANGE_STATUS_DONE)
					break;
				else if ((reg_val &
				NVL_SUBLINK_CHANGE_STATUS_MASK) ==
				NVL_SUBLINK_CHANGE_STATUS_FAULT) {
					nvlink_err("Fault while changing TX"
						" sublink to SAFE MODE");
					return -EPROTO;
				}
				usleep_range(DEFAULT_LOOP_SLEEP_US,
					DEFAULT_LOOP_SLEEP_US * 2);
				timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
			} while (timeout_us > 0);

			if (timeout_us <= 0) {
				nvlink_err("Timeout while waiting for TX"
					" sublink to go to SAFE MODE");
				return -ETIMEDOUT;
			}

			break;

		case NVLINK_TX_OFF:
			/* TODO */
			break;

		default:
			/* TODO */
			break;
		}
	} else {
		switch (mode) {
		case NVLINK_RX_OFF:
			/* TODO */
			break;
		case NVLINK_RX_RXCAL:
			status = t19x_nvlink_rxcal_enable(ndev);
			if (status) {
				nvlink_err("Unable to put RX"
					" in RXCAL for link");
				return status;
			}
			break;

		default:
			/* TODO */
			break;
		}
	}

	return status;
}

int t19x_nvlink_set_link_mode(struct nvlink_device *ndev, u32 mode)
{
	u32 link_state;
	u32 reg_val;
	int status = 0;

	link_state = t19x_nvlink_get_link_state(ndev);

	switch (mode) {
	case NVLINK_LINK_SAFE:
		if (link_state == NVL_LINK_STATE_STATE_SWCFG) {
			nvlink_dbg("Link is already in Safe mode");
			break;
		} else if (link_state == NVL_LINK_STATE_STATE_HWCFG) {
			nvlink_dbg("Link already transitioning to"
				" Safe mode");
			break;
		}

		nvlink_dbg("Changing Link state to Safe for link");

		if (link_state == NVL_LINK_STATE_STATE_INIT) {
			reg_val = nvlw_nvl_readl(ndev, NVL_LINK_CHANGE);
			reg_val &= ~NVL_LINK_CHANGE_NEWSTATE_F(~0);
			reg_val |= NVL_LINK_CHANGE_NEWSTATE_F(
					NVL_LINK_CHANGE_NEWSTATE_HWCFG);
			reg_val &= ~NVL_LINK_CHANGE_OLDSTATE_MASK_F(~0);
			reg_val |= NVL_LINK_CHANGE_OLDSTATE_MASK_F(
					NVL_LINK_CHANGE_OLDSTATE_MASK_DONTCARE);
			reg_val &= ~NVL_LINK_CHANGE_ACTION_F(~0);
			reg_val |= NVL_LINK_CHANGE_ACTION_F(
					NVL_LINK_CHANGE_ACTION_LTSSM_CHANGE);
			nvlw_nvl_writel(ndev, NVL_LINK_CHANGE, reg_val);
		} else if (link_state == NVL_LINK_STATE_STATE_ACTIVE) {
			/* TODO :
			 * Disable PM first since we are moving out of
			 * ACTIVE state.
			 */
			reg_val = nvlw_nvl_readl(ndev, NVL_LINK_CHANGE);
			reg_val &= ~NVL_LINK_CHANGE_NEWSTATE_F(~0);
			reg_val |= NVL_LINK_CHANGE_NEWSTATE_F(
					NVL_LINK_CHANGE_NEWSTATE_SWCFG);
			reg_val &= ~NVL_LINK_CHANGE_OLDSTATE_MASK_F(~0);
			reg_val |= NVL_LINK_CHANGE_OLDSTATE_MASK_F(
					NVL_LINK_CHANGE_OLDSTATE_MASK_DONTCARE);
			reg_val &= ~NVL_LINK_CHANGE_ACTION_F(~0);
			reg_val |= NVL_LINK_CHANGE_ACTION_F(
					NVL_LINK_CHANGE_ACTION_LTSSM_CHANGE);
			nvlw_nvl_writel(ndev, NVL_LINK_CHANGE, reg_val);
		}
		break;

	case NVLINK_LINK_HS:
		if (link_state == NVL_LINK_STATE_STATE_ACTIVE) {
			nvlink_dbg("Link is already in Active mode");
			break;
		} else if (link_state == NVL_LINK_STATE_STATE_INIT) {
			nvlink_err("Link cannot be taken from INIT state"
				" to Active mode");
			return -EPERM;
		}

		nvlink_dbg("changing Link state to Active...");
		reg_val = nvlw_nvl_readl(ndev, NVL_LINK_CHANGE);
		reg_val &= ~NVL_LINK_CHANGE_NEWSTATE_F(~0);
		reg_val |= NVL_LINK_CHANGE_NEWSTATE_F(
				NVL_LINK_CHANGE_NEWSTATE_ACTIVE);
		reg_val &= ~NVL_LINK_CHANGE_OLDSTATE_MASK_F(~0);
		reg_val |= NVL_LINK_CHANGE_OLDSTATE_MASK_F(
				NVL_LINK_CHANGE_OLDSTATE_MASK_DONTCARE);
		reg_val &= ~NVL_LINK_CHANGE_ACTION_F(~0);
		reg_val |= NVL_LINK_CHANGE_ACTION_F(
				NVL_LINK_CHANGE_ACTION_LTSSM_CHANGE);
		nvlw_nvl_writel(ndev, NVL_LINK_CHANGE, reg_val);

		break;

	/* TODO: other "case"s need to be implemented here */

	default:
		nvlink_err("Invalid link mode specified");
		status = -EINVAL;
		break;
	}

	return status;
}

int go_to_safe_mode(struct nvlink_device *ndev)
{
	u32 reg_val = 0;
	u32 state = 0;

	nvlink_dbg("Transitioning to SAFE mode ...");

	nvlw_nvl_writel(ndev, NVL_LINK_CHANGE, 0x14);
	usleep_range(1000, 2000);
	reg_val = nvlw_nvl_readl(ndev, NVL_LINK_STATE);
	state = reg_val & NVL_LINK_STATE_STATE_MASK;
	if (state != NVL_LINK_STATE_STATE_SWCFG) {
		nvlink_err("Failed to transition to SAFE mode");
		return -1;
	}

	nvlink_dbg("Successfully transitioned to SAFE mode");
	return 0;
}


/* For a given link, check whether tx sublink mode is at the requested mode */
bool nvlink_check_tx_sublink_mode(struct nvlink_device *ndev, u32 sublink_mode)
{
	u32 curr_sublink_mode = NVLINK_TX_OFF;

	curr_sublink_mode = ndev->link.link_ops.get_sublink_mode(ndev, false);
	switch (sublink_mode) {
	case NVLINK_TX_OFF:
		if (curr_sublink_mode == NVLINK_TX_OFF) {
			nvlink_dbg("Tx sublink is in OFF mode");
			return true;
		}
		break;
	case NVLINK_TX_SAFE:
		if (curr_sublink_mode == NVLINK_TX_SAFE) {
			nvlink_dbg("Tx sublink is in SAFE mode");
			return true;
		}
		break;
	case NVLINK_TX_HS:
		if ((curr_sublink_mode == NVLINK_TX_SINGLE_LANE)
			|| (curr_sublink_mode == NVLINK_TX_HS)) {
			nvlink_dbg("Tx sublink is in HS mode");
			return true;
		}
		break;
	}

	/* return false for default case or the states are not matching */
	return false;
}

/* For a given link, check whether rx sublink mode is at the requested mode */
bool nvlink_check_rx_sublink_mode(struct nvlink_device *ndev, u32 sublink_mode)
{
	u32 curr_sublink_mode = NVLINK_RX_OFF;

	curr_sublink_mode = ndev->link.link_ops.get_sublink_mode(ndev, true);
	switch (sublink_mode) {
	case NVLINK_RX_OFF:
		if (curr_sublink_mode == NVLINK_RX_OFF) {
			nvlink_dbg("Rx sublink is in OFF mode");
			return true;
		}
		break;
	case NVLINK_RX_SAFE:
		if (curr_sublink_mode == NVLINK_RX_SAFE) {
			nvlink_dbg("Rx sublink is in SAFE mode");
			return true;
		}
		break;
	case NVLINK_RX_HS:
		if ((curr_sublink_mode == NVLINK_RX_SINGLE_LANE)
			|| (curr_sublink_mode == NVLINK_RX_HS)) {
			nvlink_dbg("Rx sublink is in HS mode");
			return true;
		}
		break;
	}

	/* return false for default case or the states are not matching */
	return false;
}

/* For the given link, check whether the link mode is at the requested mode */
bool nvlink_check_link_mode(struct nvlink_device *ndev, u32 link_mode)
{
	u32 curr_link_mode = NVLINK_LINK_OFF;

	curr_link_mode = ndev->link.link_ops.get_link_mode(ndev);
	if (link_mode == curr_link_mode)
		return true;
	else
		return false;
}

/* Check if the given intranode connection is in the specified mode */
int nvlink_check_intranode_conn_mode(
				struct nvlink_intranode_conn *conn,
				u32 link_mode,
				bool *match)
{
	struct nvlink_device *ndev0 = conn->ndev0;
	struct nvlink_device *ndev1 = conn->ndev1;
	int ret = 0;
	bool is_mode = false;

	switch (link_mode) {
	case NVLINK_LINK_OFF:
		/* Check if both links are OFF */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_OFF) &&
			nvlink_check_link_mode(ndev1, NVLINK_LINK_OFF)) {
			*match = true;
			nvlink_dbg("Intranode connection is OFF");
			return ret;
		}

		/* Check if one of the links is OFF */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_OFF) ||
			nvlink_check_link_mode(ndev1, NVLINK_LINK_OFF)) {
			nvlink_err("Link is in bad state");
			*match = false;
			return -ENOLINK;
		}

		nvlink_dbg("Link not OFF yet.");
		*match = false;
		break;

	case NVLINK_LINK_SAFE:
		/* Check if both links and sublinks are already in SAFE mode */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_SAFE) &&
			nvlink_check_link_mode(ndev1, NVLINK_LINK_SAFE)) {
			is_mode = nvlink_check_tx_sublink_mode(ndev0,
						NVLINK_TX_SAFE) &&
					nvlink_check_tx_sublink_mode(ndev1,
						NVLINK_TX_SAFE) &&
					nvlink_check_rx_sublink_mode(ndev0,
						NVLINK_RX_SAFE) &&
					nvlink_check_rx_sublink_mode(ndev1,
						NVLINK_RX_SAFE);
			if (!is_mode) {
				nvlink_err("Sublinks in bad state");
				*match = false;
				return -ENOLINK;
			}
			*match = true;
			nvlink_dbg("Intranode connection in Safe mode");
			return ret;
		}

		/* Check if one of the links in SAFE mode */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_SAFE) ||
			nvlink_check_link_mode(ndev1, NVLINK_LINK_SAFE)) {
			nvlink_err("Link is in bad state");
			*match = false;
			return -ENOLINK;
		}

		nvlink_dbg("Link is not in Safe mode");
		*match = false;
		break;

	case NVLINK_LINK_HS:
		/* Check if both links and sublinks are in HS mode */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_HS) &&
			nvlink_check_link_mode(ndev1, NVLINK_LINK_HS)) {
			is_mode = nvlink_check_tx_sublink_mode(ndev0,
						NVLINK_TX_HS) &&
					nvlink_check_tx_sublink_mode(ndev1,
						NVLINK_TX_HS) &&
					nvlink_check_rx_sublink_mode(ndev0,
						NVLINK_RX_HS) &&
					nvlink_check_rx_sublink_mode(ndev1,
						NVLINK_RX_HS);
			if (!is_mode) {
				nvlink_err("Sublinks in bad state");
				*match = false;
				return -ENOLINK;
			}
			*match = true;
			nvlink_dbg("Intranode connection in HS mode");
			return ret;
		}

		/* Check if one of the links in HS mode */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_HS) ||
			nvlink_check_link_mode(ndev1, NVLINK_LINK_HS)) {
			nvlink_err("Link is in bad state");
			*match = false;
			return -ENOLINK;
		}

		nvlink_dbg("Link is not in High Speed mode");
		*match = false;
		break;

	default:
		*match = false;
	}
	return ret;
}

int nvlink_train_intranode_conn_to_hs(struct nvlink_intranode_conn *conn)
{
	int ret = 0;
	struct nvlink_device *ndev0 = conn->ndev0;
	struct nvlink_device *ndev1 = conn->ndev1;
	struct nvlink_link *link0 = &(ndev0->link);
	struct nvlink_link *link1 = &(ndev1->link);
	bool match = false;

	/* Check if both the link and sublink state are HS for both ends */
	ret = nvlink_check_intranode_conn_mode(conn,
						NVLINK_LINK_HS,
						&match);
	/* Return if the links are in bad state or already in HS mode */
	if (ret < 0) {
		nvlink_err("Can't transition to HS as link is in bad state");
		return ret;
	}
	if (match) {
		nvlink_err("Exiting HS transition as link is already in HS");
		return ret;
	}

	/* We can train connection to HS only if they are in Safe mode */
	ret = nvlink_check_intranode_conn_mode(conn,
						NVLINK_LINK_SAFE,
						&match);
	/* Return if the links are in bad state or not in Safe mode */
	if (ret < 0) {
		nvlink_err("Can't transition to HS as link is in bad state");
		return ret;
	}
	if (!match) {
		nvlink_err("Exiting HS transition as link is not in SAFE mode");
		return ret;
	}

	/* Enable PRBS generator on both ends */
	ret = link0->link_ops.set_sublink_mode(ndev0,
					false,
					NVLINK_TX_PRBS_EN);
	if (ret < 0) {
		nvlink_err("Failed to enable PRBS generator for ndev0");
		return ret;
	}

	ret = link1->link_ops.set_sublink_mode(ndev1,
					false,
					NVLINK_TX_PRBS_EN);
	if (ret < 0) {
		nvlink_err("Failed to enable PRBS generator for ndev1");
		return ret;
	}

	/* Put TX sublink on end0 in High Speed */
	ret = link0->link_ops.set_sublink_mode(ndev0,
					false,
					NVLINK_TX_HS);
	if (ret < 0) {
		nvlink_err("Failed to set TX sublink mode to HS for ndev0");
		return ret;
	}

	/* Put TX sublink on end1 in High Speed */
	ret = link1->link_ops.set_sublink_mode(ndev1,
					false,
					NVLINK_TX_HS);
	if (ret < 0) {
		nvlink_err("Failed to set TX sublink mode to HS for ndev1");
		return ret;
	}

	/* wait for sublinks to go in High Speed */
	ret = t19x_nvlink_poll_sublink_state(ndev0,
					NVLINK_TX_HS,
					ndev1,
					NVLINK_RX_HS,
					NVLINK_TRANSITION_HS_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set sublinks in high speed mode");
		return ret;
	}

	ret = t19x_nvlink_poll_sublink_state(ndev1,
					NVLINK_TX_HS,
					ndev0,
					NVLINK_RX_HS,
					NVLINK_TRANSITION_HS_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set sublinks in high speed mode");
		return ret;
	}

	/*
	 * Put only end0 in ACTIVE mode. The other end should automatically
	 * go to Active mode.
	 */
	link0->link_ops.set_link_mode(ndev0, NVLINK_LINK_HS);

	/* Wait for other end to go in ACTIVE mode */
	ret = t19x_nvlink_poll_link_state(ndev1,
					NVLINK_LINK_HS,
					NVLINK_TRANSITION_HS_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set links in high speed mode");
		return ret;
	}

	nvlink_dbg("Link in High Speed mode!");
	return ret;
}

/* Enable ANO packets over link */
void nvlink_enable_AN0_packets(struct nvlink_device *ndev)
{
	u32 reg_val = 0;

	reg_val = nvlw_nvl_readl(ndev, NVL_LINK_CONFIG);
	reg_val |= BIT(NVL_LINK_CONFIG_LINK_EN);
	nvlw_nvl_writel(ndev, NVL_LINK_CONFIG, reg_val);
}
