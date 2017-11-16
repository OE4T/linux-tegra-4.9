/*
 * sor_t19x.h: T19x specific SOR definitions
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#ifndef __DRIVER_VIDEO_TEGRA_DC_SOR_T19X_H__
#define __DRIVER_VIDEO_TEGRA_DC_SOR_T19X_H__

/* This enum is a T19x specific extension of tegra_dc_sor_link_speed_key.
 * When merging the T19x code back to linux-nvidia project, please remove this
 * definition, and merge the enumerated values added here to the original enum
 */
enum tegra_dc_sor_link_speed_key_t19x {
	TEGRA_DC_SOR_LINK_SPEED_G1_62_UNUSED,
	TEGRA_DC_SOR_LINK_SPEED_G2_7_UNUSED,
	TEGRA_DC_SOR_LINK_SPEED_G5_4_UNUSED,
	/* newly added for T19x */
	TEGRA_DC_SOR_LINK_SPEED_G8_1,
};

enum tegra_dc_dp_training_pattern_key_t19x {
	TEGRA_DC_DP_TRAINING_PATTERN_DISABLE_UNUSED,
	TEGRA_DC_DP_TRAINING_PATTERN_1_UNUSED,
	TEGRA_DC_DP_TRAINING_PATTERN_2_UNUSED,
	TEGRA_DC_DP_TRAINING_PATTERN_3_UNUSED,
	TEGRA_DC_DP_TRAINING_PATTERN_D102_UNUSED,
	TEGRA_DC_DP_TRAINING_PATTERN_SBLERRRATE_UNUSED,
	TEGRA_DC_DP_TRAINING_PATTERN_PRBS7_UNUSED,
	TEGRA_DC_DP_TRAINING_PATTERN_CSTM_UNUSED,
	TEGRA_DC_DP_TRAINING_PATTERN_HBR2_COMPLIANCE_UNUSED,
	/* newly added for T19x */
	TEGRA_DC_DP_TRAINING_PATTERN_4,
};

#endif /* __DRIVER_VIDEO_TEGRA_DC_SOR_T19X_H__ */
