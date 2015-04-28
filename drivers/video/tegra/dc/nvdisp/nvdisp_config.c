/*
 * drivers/video/tegra/nvdisp/nvdisp_config.c
 *
 * Copyright (c) 2014-2014, NVIDIA CORPORATION, All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "dc_config.h"

/*
 *	TO DO: Currently setting same feature for all heads
 *	provide separate table for each as they differ.
 *	The table and definitions are taken from T210,
 *	modify as needed.
*/


#define TEGRA_WIN_FMT_T186_LOW (TEGRA_WIN_FMT_BASE | \
				BIT(TEGRA_WIN_FMT_T_R4G4B4A4))
#define TEGRA_WIN_FMT_T186_HIGH TEGRA_WIN_FMT_T124_HIGH

/* for windows that support compression */
#define TEGRA_WIN_FMT_COMPRESSION_T186_LOW (BIT(TEGRA_WIN_FMT_B8G8R8A8) | \
				BIT(TEGRA_WIN_FMT_R8G8B8A8))
#define TEGRA_WIN_FMT_COMPRESSION_T186_HIGH  (0)

/* for windows that can't support planar rotation */
#define TEGRA_WIN_FMT_ROTATION_T186_LOW TEGRA_WIN_FMT_BASE
#define TEGRA_WIN_FMT_ROTATION_T186_HIGH (0)

#define TEGRA_WIN_FMT_SIMPLE_T186_LOW (BIT(TEGRA_WIN_FMT_B4G4R4A4) | \
				BIT(TEGRA_WIN_FMT_B5G5R5A) | \
				BIT(TEGRA_WIN_FMT_B5G6R5) | \
				BIT(TEGRA_WIN_FMT_T_R4G4B4A4) | \
				BIT(TEGRA_WIN_FMT_B8G8R8A8) | \
				BIT(TEGRA_WIN_FMT_R8G8B8A8))

#define TEGRA_WIN_FMT_SIMPLE_T186_HIGH (HIGHBIT(TEGRA_WIN_FMT_A8B8G8R8))

static struct tegra_dc_feature_entry t186_feature_entries_a[] = {
	{ 0, TEGRA_DC_FEATURE_FORMATS,
			{ TEGRA_WIN_FMT_T186_LOW, TEGRA_WIN_FMT_T186_HIGH } },
	{ 0, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 0, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 0, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {2, 2, 2, 2} },
	{ 0, TEGRA_DC_FEATURE_FILTER_TYPE, {1, 1} },
	{ 0, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 1} },
	{ 0, TEGRA_DC_FEATURE_INVERT_TYPE, {1, 1, 1} },
	{ 0, TEGRA_DC_FEATURE_FIELD_TYPE, {1} },
	{ 0, TEGRA_DC_FEATURE_COMPRESSION_FORMATS,
		{ TEGRA_WIN_FMT_COMPRESSION_T186_LOW,
		TEGRA_WIN_FMT_COMPRESSION_T186_HIGH } },
	/* dispA:windowA can rotate the planar formats */
	{ 0, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 2304, 1} },
	{ 0, TEGRA_DC_FEATURE_ROTATION_FORMATS,
			{ TEGRA_WIN_FMT_T186_LOW, TEGRA_WIN_FMT_T186_HIGH } },
	{ 0, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },

	{ 1, TEGRA_DC_FEATURE_FORMATS,
			{ TEGRA_WIN_FMT_T186_LOW, TEGRA_WIN_FMT_T186_HIGH } },
	{ 1, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 1, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 1, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {2, 2, 2, 2} },
	{ 1, TEGRA_DC_FEATURE_FILTER_TYPE, {1, 1} },
	{ 1, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 1} },
	{ 1, TEGRA_DC_FEATURE_INVERT_TYPE, {1, 1, 1} },
	{ 1, TEGRA_DC_FEATURE_FIELD_TYPE, {1} },
	{ 1, TEGRA_DC_FEATURE_COMPRESSION_FORMATS, {0, 0} },
	/* cannot rotate planar format */
	{ 1, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE, {0, 0, 0, 0} },
	{ 1, TEGRA_DC_FEATURE_ROTATION_FORMATS,
			{ TEGRA_WIN_FMT_ROTATION_T186_LOW,
			TEGRA_WIN_FMT_ROTATION_T186_HIGH } },
	{ 1, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },

	{ 2, TEGRA_DC_FEATURE_FORMATS,
			{ TEGRA_WIN_FMT_T186_LOW, TEGRA_WIN_FMT_T186_HIGH } },
	{ 2, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 2, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 2, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {2, 2, 2, 2} },
	{ 2, TEGRA_DC_FEATURE_FILTER_TYPE, {1, 1} },
	{ 2, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 1} },
	{ 2, TEGRA_DC_FEATURE_INVERT_TYPE, {1, 1, 1} },
	{ 2, TEGRA_DC_FEATURE_FIELD_TYPE, {1} },
	{ 2, TEGRA_DC_FEATURE_COMPRESSION_FORMATS, {0, 0} },
	{ 2, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE, {0, 0, 0, 0} },
	{ 2, TEGRA_DC_FEATURE_ROTATION_FORMATS,
			{ TEGRA_WIN_FMT_ROTATION_T186_LOW,
			TEGRA_WIN_FMT_ROTATION_T186_HIGH } },
	{ 2, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },

	{ 3, TEGRA_DC_FEATURE_FORMATS, { TEGRA_WIN_FMT_SIMPLE_T186_LOW,
		TEGRA_WIN_FMT_SIMPLE_T186_HIGH, } },
	{ 3, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 3, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 3, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {1, 1, 1, 1} },
	{ 3, TEGRA_DC_FEATURE_FILTER_TYPE, {0, 0} },
	{ 3, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 0} },
	{ 3, TEGRA_DC_FEATURE_INVERT_TYPE, {0, 0, 0} },
	{ 3, TEGRA_DC_FEATURE_FIELD_TYPE, {0} },
	{ 3, TEGRA_DC_FEATURE_COMPRESSION_FORMATS, {0, 0} },
	{ 3, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE, {0, 0, 0, 0} },
	/* cannot rotate any formats */
	{ 3, TEGRA_DC_FEATURE_ROTATION_FORMATS, { 0, 0 } },
	{ 3, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },
};

static struct tegra_dc_feature t186_feature_table_a = {
	ARRAY_SIZE(t186_feature_entries_a), t186_feature_entries_a,
};

void nvdisp_dc_feature_register(struct tegra_dc *dc)
{
	/* TO DO: provide separate table for each head,
	 * if the feature will differ per head
	 */
	dc->feature = &t186_feature_table_a;
}
