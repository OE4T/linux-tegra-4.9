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

