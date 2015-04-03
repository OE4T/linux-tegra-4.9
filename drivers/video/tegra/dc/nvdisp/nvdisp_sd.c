/*
 * drivers/video/tegra/dc/nvdisp_sd.c
 *
 * Copyright (c) 2015, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <mach/dc.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/backlight.h>
#include <linux/stat.h>
#include <linux/delay.h>

#include "hw_nvdisp_nvdisp.h"
#include "nvsd2.h"
#include "nvdisp.h"
#include "nvdisp_priv.h"
#include "hw_win_nvdisp.h"

#include "dc_config.h"
#include "dc_priv.h"
#include "dc_reg.h"

/* Elements for sysfs access */
#define NVSD_ATTR(__name) static struct kobj_attribute nvsd_attr_##__name = \
	__ATTR(__name, S_IRUGO|S_IWUSR, nvsd_settings_show, nvsd_settings_store)
#define NVSD_ATTRS_ENTRY(__name) (&nvsd_attr_##__name.attr)
#define IS_NVSD_ATTR(__name) (attr == &nvsd_attr_##__name)

#ifndef __DRIVERS_VIDEO_TEGRA_DC_NVSD_H
#define __DRIVERS_VIDEO_TEGRA_DC_NVSD_H
#endif
#define SD3_MAX_HIST_BINS	256
#define GAIN_TABLE_MAX_ENTRIES	256
#define ADAPTATION_FACTOR_MAX_LEVELS	21
#define MAX_AGGRESSIVENESS_LEVELS	5
/* ADAPTATION factor is in multiples of 2%
 * i.e image & backlight levels can be adapted by 2%, 4%... etc
 * upto a max of 40%
 */
#define SD3_ADAPTATION_STEP_PERCENT	2
#define MAX_PIXEL_ADAPTATION_PERCENT	((ADAPTATION_FACTOR_MAX_LEVELS - 1) \
		* SD3_ADAPTATION_STEP_PERCENT)
#define MAX_PIXEL_DEPTH_BITS	12
#define MAX_PIXEL_VALUE	((1 << MAX_PIXEL_DEPTH_BITS) - 1)
#define SMARTDIMMER_STATE_DISABLED	0
#define SMARTDIMMER_STATE_ENABLED	1
#define SMARTDIMMER_MAX_BOUND	255
#define SMARTDIMMER_MIN_BOUND	0
#define SMARTDIMMER_MAX_SATURATED_PIXELS	20
#define SMARTDIMMER_DARK_REGION_HIST_LIMIT	10
/* This should be in sync with NvAPI spec */
#define SMARTDIMMER_AGGR_LEVEL_MIN	1
#define SMARTDIMMER_AGGR_LEVEL_MAX	5
#define NUM_PHASE_IN_STEPS	4
#define TEGRA_SMARTDIMMER_TIMEOUT	10
#define BACKLIGHT_ADJUST_STEPS_DEFAULT	1
#define MAX_BACKLIGHT_STEPS 256

static int phase_backlight_table[MAX_BACKLIGHT_STEPS];
static unsigned int gain_table[GAIN_TABLE_MAX_ENTRIES];
static unsigned int current_gain_table[GAIN_TABLE_MAX_ENTRIES];

static int const aggr_table[SMARTDIMMER_AGGR_LEVEL_MAX] = {
	5, 10, 15, 20, 25
};

static int const pixel_gain_tables[ADAPTATION_FACTOR_MAX_LEVELS]
	[GAIN_TABLE_MAX_ENTRIES] = {
	{
		0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000,
		0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000,
		0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000,
		0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000
	},
	{
		0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144,
		0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144,
		0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144,
		0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4134
	},
	{
		0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c,
		0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c,
		0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c,
		0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x4274, 0x419c
	},
	{
		0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4,
		0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4,
		0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4,
		0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43b4, 0x4300, 0x41bc
	} ,
	{
		0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c,
		0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c,
		0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c,
		0x451c, 0x451c, 0x451c, 0x451c, 0x44f8, 0x4458, 0x4344, 0x41d0
	},
	{
		0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664,
		0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664,
		0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664,
		0x4664, 0x4664, 0x4664, 0x463c, 0x45ac, 0x44b8, 0x4370, 0x41d8
	},
	{
		0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac,
		0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac,
		0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac,
		0x47ac, 0x47ac, 0x4784, 0x4700, 0x4624, 0x44fc, 0x438c, 0x41e0
	},
	{
		0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4,
		0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4,
		0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4,
		0x48f4, 0x48d0, 0x4854, 0x478c, 0x4678, 0x4528, 0x43a0, 0x41e4
	},
	{
		0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c,
		0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c,
		0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c,
		0x4a1c, 0x49ac, 0x48f0, 0x47f0, 0x46b8, 0x454c, 0x43b0, 0x41e8
	},
	{
		0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84,
		0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84,
		0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b68,
		0x4b00, 0x4a54, 0x4964, 0x4840, 0x46e8, 0x4564, 0x43bc, 0x41ec
	},
	{
		0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc,
		0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc,
		0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4cb8, 0x4c58,
		0x4bb8, 0x4ad8, 0x49c4, 0x4880, 0x4710, 0x457c, 0x43c4, 0x41ec
	},
	{
		0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14,
		0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14,
		0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e04, 0x4db4, 0x4d1c,
		0x4c4c, 0x4b44, 0x4a10, 0x48b4, 0x4730, 0x458c, 0x43cc, 0x41f0
	},
	{
		0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c,
		0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c,
		0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f54, 0x4f0c, 0x4e84, 0x4dc0,
		0x4cc4, 0x4ba0, 0x4a50, 0x48dc, 0x474c, 0x459c, 0x43d0, 0x41f0
	},
	{
		0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0,
		0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0,
		0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x5068, 0x4fec, 0x4f34, 0x4e48,
		0x4d2c, 0x4bec, 0x4a88, 0x4900, 0x4760, 0x45a8, 0x43d8, 0x41f0
	},
	{
		0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8,
		0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8,
		0x51e8, 0x51e8, 0x51e8, 0x51c0, 0x5154, 0x50a8, 0x4fc8, 0x4eb8,
		0x4d84, 0x4c2c, 0x4ab4, 0x4920, 0x4774, 0x45b0, 0x43dc, 0x41f4
	},
	{
		0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330,
		0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330,
		0x5330, 0x5330, 0x5318, 0x52b8, 0x521c, 0x5148, 0x5048, 0x4f1c,
		0x4dd0, 0x4c60, 0x4ad8, 0x4938, 0x4784, 0x45b8, 0x43e0, 0x41f4
	},
	{
		0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478,
		0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478,
		0x5478, 0x546c, 0x5420, 0x5390, 0x52c8, 0x51d0, 0x50b4, 0x4f70,
		0x4e0c, 0x4c90, 0x4afc, 0x4950, 0x4790, 0x45c0, 0x43e0, 0x41f4
	},
	{
		0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0,
		0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0,
		0x55bc, 0x5580, 0x5500, 0x5448, 0x535c, 0x5248, 0x5110, 0x4fb8,
		0x4e44, 0x4cb8, 0x4b14, 0x4960, 0x479c, 0x45c8, 0x43e4, 0x41f4
	},
	{
		0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708,
		0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708,
		0x56e4, 0x5674, 0x55cc, 0x54ec, 0x53e4, 0x52b4, 0x5168, 0x4ffc,
		0x4e78, 0x4cdc, 0x4b30, 0x4974, 0x47a8, 0x45cc, 0x43e8, 0x41f4
	},
	{
		0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850,
		0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5840,
		0x57e4, 0x574c, 0x5678, 0x557c, 0x5458, 0x5310, 0x51ac, 0x5034,
		0x4ea0, 0x4cfc, 0x4b48, 0x4984, 0x47b0, 0x45d0, 0x43e8, 0x41f4
	},
	{
		0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5998,
		0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5994, 0x5954,
		0x58cc, 0x5808, 0x5718, 0x55fc, 0x54c0, 0x5364, 0x51f0, 0x5068,
		0x4ec8, 0x4d18, 0x4b5c, 0x4990, 0x47b8, 0x45d8, 0x43ec, 0x41f8
	}
};

int const sd_backlight_table[SD3_MAX_HIST_BINS] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6,
	6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 14,
	14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23,
	23, 24, 25, 25, 26, 27, 27, 28, 28, 29, 30, 31, 31, 32, 33, 33, 34, 35,
	36, 36, 37, 38, 39, 39, 40, 41, 42, 43, 44, 44, 45, 46, 47, 48, 49, 50,
	51, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
	68, 70, 71, 72, 73, 74, 75, 76, 77, 78, 80, 81, 82, 83, 84, 86, 87, 88,
	89, 91, 92, 93, 94, 96, 97, 98, 100, 101, 102, 104, 105, 106, 108, 109,
	110, 112, 113, 115, 116, 118, 119, 120, 122, 123, 125, 126, 128, 129,
	131, 132, 134, 136, 137, 139, 140, 142, 143, 145, 147, 148, 150, 152,
	153, 155, 157, 158, 160, 162, 164, 165, 167, 169, 171, 172, 174, 176,
	178, 179, 181, 183, 185, 187, 189, 191, 192, 194, 196, 198, 200, 202,
	204, 206, 208, 210, 212, 214, 216, 218, 220, 222, 224, 226, 228, 230,
	232, 234, 237, 239, 241, 243, 245, 247, 249, 252, 254, 256
};

static ssize_t nvsd_settings_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf);

static ssize_t nvsd_settings_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count);

static ssize_t nvsd_registers_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf);

static struct kobj_attribute nvsd_attr_registers =
	__ATTR(registers, S_IRUGO, nvsd_registers_show, NULL);

NVSD_ATTR(enable);
NVSD_ATTR(aggressiveness);

static struct attribute *nvsd_attrs[] = {
	NVSD_ATTRS_ENTRY(enable),
	NVSD_ATTRS_ENTRY(aggressiveness),
	NVSD_ATTRS_ENTRY(registers),
	NULL,
};

static struct attribute_group nvsd_attr_group = {
	.attrs = nvsd_attrs,
};

static struct kobject *nvsd_kobj;

/* shared brightness variable */
static atomic_t *_sd_brightness;

/* tegra_sd_poll_register:
 * function to poll smartdimmer control registers
 * after issuing commands
 * */
static int tegra_sd_poll_register(struct tegra_dc *dc,
		u32 reg, u32 mask, u32 exp_val,
		u32 poll_interval_us, u32 timeout_ms)
{
	unsigned long timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);

	u32 reg_val;
	do {
		usleep_range(poll_interval_us, poll_interval_us << 1);
		reg_val = tegra_dc_readl(dc, reg);
	} while (((reg_val & mask) != exp_val) &&
			time_after(timeout_jf, jiffies));

	if ((reg_val & mask) == exp_val)
		return 0;
	dev_err(&dc->ndev->dev,
			"smartdimmer_poll_register 0x%x: timeout\n", reg);
	return jiffies - timeout_jf + 1;
}

/* tegra_dc_calc_params:
 * 1. Calculate the upper and lower bounds
 * 2. Select correct gain & backlight based on oversaturated bin
 */
void tegra_dc_calc_params(struct tegra_dc_sd_settings *sd)
{
	unsigned pixel_k = 100;
	unsigned pixel_adaptation;
	unsigned gain_table_index = 0, i = 0;

	if (sd->over_saturated_bin > 0)
		pixel_k = (SD3_MAX_HIST_BINS * 100) / sd->over_saturated_bin;

	pixel_adaptation = (((pixel_k - 100) * 100 /
				SD3_ADAPTATION_STEP_PERCENT)
			* SD3_ADAPTATION_STEP_PERCENT) / 100;

	if ((pixel_adaptation >= MAX_PIXEL_ADAPTATION_PERCENT) ||
		(sd->over_saturated_bin == SMARTDIMMER_MIN_BOUND))
		pixel_adaptation = MAX_PIXEL_ADAPTATION_PERCENT;

	sd->upper_bound = ((SD3_MAX_HIST_BINS * 10000)
		/ (100 + pixel_adaptation)) / 100;
	sd->lower_bound = ((SD3_MAX_HIST_BINS * 10000)
		/ (100 + pixel_adaptation + SD3_ADAPTATION_STEP_PERCENT)) / 100;

	sd->upper_bound = (sd->upper_bound > SMARTDIMMER_MAX_BOUND) ?
		SMARTDIMMER_MAX_BOUND : sd->upper_bound;

	if (pixel_adaptation == SMARTDIMMER_MIN_BOUND)
		sd->lower_bound = SMARTDIMMER_MIN_BOUND;

	gain_table_index = (pixel_adaptation * 100 /
			SD3_ADAPTATION_STEP_PERCENT) / 100;

	for (i = 0; i < GAIN_TABLE_MAX_ENTRIES; i++)
		sd->gain_table[i] = pixel_gain_tables[gain_table_index][i];

	sd->new_backlight = sd_backlight_table[sd->over_saturated_bin];
}

/* tegra_sd_set_params:
 * 1. Calculate the phase in steps
 * 2. Set the backlight steps
 * 3. Set the upper and lower bounds
 * */
static int tegra_sd_set_params(struct tegra_dc *dc,
		struct tegra_dc_sd_settings *sd)
{
	unsigned phase_in_steps = NUM_PHASE_IN_STEPS;
	int i;
	bool backlight_diff = true;

	/* For cases where backlight does not change */
	if (sd->new_backlight == sd->old_backlight) {
		tegra_dc_writel(dc,
			nvdisp_sd_hist_int_bounds_upper_f(sd->upper_bound) |
			nvdisp_sd_hist_int_bounds_lower_f(sd->lower_bound) |
			nvdisp_sd_hist_int_bounds_set_enable_f(),
			nvdisp_sd_hist_int_bounds_r());
		return 0;
	}

	if (sd->new_backlight < sd->old_backlight)
		backlight_diff = false;

	phase_in_steps = (backlight_diff ? (sd->new_backlight -
		sd->old_backlight) : (sd->old_backlight - sd->new_backlight));

	phase_in_steps /= sd->backlight_adjust_steps;

	if (phase_in_steps < 1)
		phase_in_steps = 1;

	sd->phase_in_steps = phase_in_steps;

	sd->phase_backlight_table[phase_in_steps - 1] = sd->new_backlight;

	for (i = 0; i < phase_in_steps - 1; i++) {
		if (backlight_diff)
			sd->phase_backlight_table[i] =
				sd->old_backlight +
				sd->backlight_adjust_steps*(i+1);
		else
			sd->phase_backlight_table[i] =
				sd->old_backlight -
				sd->backlight_adjust_steps*(i+1);
	}

	sd->old_backlight = sd->new_backlight;
	sd->last_phase_step = 0;

	tegra_dc_writel(dc,
		nvdisp_sd_hist_int_bounds_upper_f(sd->upper_bound) |
		nvdisp_sd_hist_int_bounds_lower_f(sd->lower_bound) |
		nvdisp_sd_hist_int_bounds_set_enable_f(),
		nvdisp_sd_hist_int_bounds_r());

	return 0;
}

/* tegra_sd_update_brightness :
 * called by the vblank interrupt worker
 * when we get a histogram interrupt
 * 1. Read oversaturated bin if smartdimmer interrupt
 * 2. Calculate gain based on phase step
 * 3. Set pixel gain registerss and backlight
 * */
bool tegra_sd_update_brightness(struct tegra_dc *dc)
{
	int i;
	int gain_step;
	int phases_left;
	int gain_frac;
	struct tegra_dc_sd_settings *sd = dc->out->sd_settings;

	sd->over_saturated_bin = tegra_dc_readl(dc,
		nvdisp_sd_hist_over_sat_r());

	if ((sd->over_saturated_bin == 255) ||
			(sd->over_saturated_bin < 128)) {
		return false;
	}

	if (sd->update_sd) {
		/* Calculate Parameters */
		tegra_dc_calc_params(sd);
		/* Set the parameters */
		tegra_sd_set_params(dc, sd);
		sd->update_sd = false;
	}

	/* Calculate the gain step based on current gain and phase steps*/
	phases_left = sd->phase_in_steps - sd->last_phase_step;

	for (i = 0; i < GAIN_TABLE_MAX_ENTRIES; i++) {
		if (sd->current_gain_table[i] > sd->gain_table[i]) {
			gain_step = sd->gain_table[i] -
				sd->current_gain_table[i];
			if (phases_left > 1) {
				gain_step = gain_step * 100 / phases_left;
				gain_step = gain_step / 100;
			}
			sd->current_gain_table[i] += gain_step;
		} else {
			gain_step = sd->current_gain_table[i] -
				sd->gain_table[i];
			if (phases_left) {
				gain_step = gain_step * 100 / phases_left;
				gain_step = gain_step / 100;
			}
			sd->current_gain_table[i] -= gain_step;
		}
	}

	/*set backlight */
	atomic_set(_sd_brightness,
			sd->phase_backlight_table[sd->last_phase_step]);
	sd->old_backlight = sd->phase_backlight_table[sd->last_phase_step];

	/* reset the gain ctrl register */
	tegra_dc_writel(dc,
		nvdisp_sd_gain_ctrl_reset_start_f(),
		nvdisp_sd_gain_ctrl_r());

	if (tegra_sd_poll_register(dc,
			nvdisp_sd_gain_ctrl_r(),
			nvdisp_sd_gain_ctrl_reset_start_f(),
			0x0, 100, TEGRA_SMARTDIMMER_TIMEOUT)) {
		dev_err(&dc->ndev->dev,
				"NV_SD: pixel gain reset failed\n");
		return false;
	}

	/* fill pixel gain tables */
	for (i = 0; i < GAIN_TABLE_MAX_ENTRIES; i++) {
		gain_frac = (sd->current_gain_table[i] & 0x3fff) >> 2;
		tegra_dc_writel(dc,	nvdisp_sd_gain_rg_rint_f(0x1) |
				nvdisp_sd_gain_rg_rfrac_f(gain_frac),
				nvdisp_sd_gain_rg_r());
		tegra_dc_writel(dc,	nvdisp_sd_gain_b_int_f(0x1) |
				nvdisp_sd_gain_b_frac_f(gain_frac),
				nvdisp_sd_gain_b_r());
	}

	/* set gain register to update*/
	tegra_dc_writel(dc,
			nvdisp_sd_gain_ctrl_set_enable_f() |
			nvdisp_sd_gain_ctrl_update_start_f() |
			nvdisp_sd_gain_ctrl_timing_f(0x1),
			nvdisp_sd_gain_ctrl_r());

	/* Increment the phase step and check for last step */
	sd->last_phase_step++;

	if (sd->last_phase_step == sd->phase_in_steps) {
		sd->phase_in_steps = 0;
		sd->last_phase_step = 0;
	}

	return true;
}

/* tegra_sd_init:
 * 1. Allocate and initialize gain tables
 * 2. Set default interrupt bounds
 * 3. Calculate over saturated pixels based on aggressiveness
 * 4. Set default backlight and activate smartdimmer
 */
void tegra_sd_init(struct tegra_dc *dc)
{
	int i;
	struct tegra_dc_sd_settings *sd;
	int total_pixel_count;

	tegra_dc_io_start(dc);

	/* If SD's not present, clear histogram register and return */
	if (!dc->out->sd_settings) {
			tegra_dc_writel(dc, 0, nvdisp_sd_hist_ctrl_r());
			if (_sd_brightness)
				atomic_set(_sd_brightness, 255);
			tegra_dc_io_end(dc);
			return;
	}
	sd = dc->out->sd_settings;

	if (!sd)
		return;

	if (sd->enable) {
		tegra_dc_io_end(dc);
		return;
	}

	/* Allocate sw gain table */
	sd->gain_table = gain_table;
	sd->current_gain_table = current_gain_table;
	sd->phase_backlight_table = phase_backlight_table;

	/* Check if sd is initialized */
	if (!nvdisp_sd_hist_ctrl_is_enable_v
		(tegra_dc_readl(dc, nvdisp_sd_hist_ctrl_r()))) {

		/* Initialize pixel gain table */
		tegra_dc_writel(dc,
			nvdisp_sd_gain_ctrl_reset_start_f(),
			nvdisp_sd_gain_ctrl_r());

		if (tegra_sd_poll_register(dc,
					nvdisp_sd_gain_ctrl_r(),
					nvdisp_sd_gain_ctrl_reset_start_f(),
					0x0, 100, TEGRA_SMARTDIMMER_TIMEOUT)) {
			dev_err(&dc->ndev->dev,
					"NV_SD: pixel gain reset failed\n");
			goto sd_fail;
		}

		/* Fill pixel gain table with 1.00 (default value) */
		for (i = 0; i < GAIN_TABLE_MAX_ENTRIES; i++) {
			tegra_dc_writel(dc, 0x40004000, nvdisp_sd_gain_rg_r());
			tegra_dc_writel(dc, 0x40004000, nvdisp_sd_gain_b_r());
			sd->gain_table[i] = 0x4000;
			sd->current_gain_table[i] = 0x4000;
		}

		/* set gain control register to update and enable gain ctrl */
		tegra_dc_writel(dc,
				nvdisp_sd_gain_ctrl_update_start_f() |
				nvdisp_sd_gain_ctrl_set_enable_f(),
				nvdisp_sd_gain_ctrl_r());

		if (tegra_sd_poll_register(dc,
					nvdisp_sd_gain_ctrl_r(),
					nvdisp_sd_gain_ctrl_update_start_f(),
					0x0, 100, TEGRA_SMARTDIMMER_TIMEOUT)) {
			dev_err(&dc->ndev->dev,
					"NV_SD: pixel gain update failed\n");
			goto sd_fail;
		}

		_sd_brightness = sd->sd_brightness;
		/* Set the backlight and the old backlight value */
		atomic_set(_sd_brightness, (int)255);
		sd->old_backlight = 255;
		sd->backlight_adjust_steps = BACKLIGHT_ADJUST_STEPS_DEFAULT;
	}

	/* Setting lower and higher interrupt bounds */
	tegra_dc_writel(dc,
		nvdisp_sd_hist_int_bounds_upper_f(128) |
		nvdisp_sd_hist_int_bounds_lower_f(SMARTDIMMER_MIN_BOUND) |
		nvdisp_sd_hist_int_bounds_set_enable_f(),
		nvdisp_sd_hist_int_bounds_r());

	sd->upper_bound = SMARTDIMMER_MAX_BOUND;
	sd->lower_bound = SMARTDIMMER_MIN_BOUND;

	/* Getting the over saturated pixel count based
	 * on users specified aggressiveness */
	total_pixel_count = dc->mode.h_active *
		dc->mode.v_active;

	sd->num_over_saturated_pixels = (total_pixel_count *
			aggr_table[sd->aggressiveness - 1]) / 100;
	/* enable the histogram function */
	tegra_dc_writel(dc,
		nvdisp_sd_hist_ctrl_max_pixel_f(
			sd->num_over_saturated_pixels) |
		nvdisp_sd_hist_int_bounds_set_enable_f(),
		nvdisp_sd_hist_ctrl_r());

	sd->phase_in_steps = 0;
	sd->enable = 1;

sd_fail:
	tegra_dc_io_end(dc);
	return;
}

/* tegra_sd_stop:
 * 1. Disable gain function
 * 2. Disable interrupt bounds
 * 3. Disable the histogram function
 * */
void tegra_sd_stop(struct tegra_dc *dc)
{
	struct tegra_dc_sd_settings *sd = dc->out->sd_settings;

	if (!dc->out->sd_settings)
		return;

	if (sd->enable == 0)
		return;

	sd->enable = 0;
	/*Disable the gain function*/
	tegra_dc_writel(dc, 0x0,
		nvdisp_sd_gain_ctrl_r());

	/*Disable the interrupt bounds*/
	tegra_dc_writel(dc, 0x0,
		nvdisp_sd_hist_int_bounds_r());

	/*Disable the histogram function*/
	tegra_dc_writel(dc, 0, nvdisp_sd_hist_ctrl_r());

	return;
}

static ssize_t nvsd_settings_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct device *dev = container_of((kobj->parent), struct device, kobj);
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_sd_settings *sd = dc->out->sd_settings;
	ssize_t res = 0;

	if (sd) {
		if (IS_NVSD_ATTR(enable))
			res = snprintf(buf, PAGE_SIZE, "%d\n",
					sd->enable);
		else if (IS_NVSD_ATTR(aggressiveness))
			res = snprintf(buf, PAGE_SIZE, "%d\n",
					sd->aggressiveness);
	} else {
		res = -EINVAL;
	}

	return res;
}

#define nvsd_check_and_update(_min, _max, _varname) { \
	long int val; \
	int err = kstrtol(buf, 10, &val); \
	if (err) \
		return err; \
	if (val >= _min && val <= _max) { \
		sd->_varname = val; \
		settings_updated = true; \
	} }

static ssize_t nvsd_settings_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct device *dev = container_of((kobj->parent), struct device, kobj);
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_sd_settings *sd = dc->out->sd_settings;
	ssize_t res = count;
	long int result;
	int err;
	bool settings_updated = false;

	if (sd) {
		if (IS_NVSD_ATTR(enable)) {
			nvsd_check_and_update(0, 1, enable);
			settings_updated = true;
		}
		if (IS_NVSD_ATTR(aggressiveness)) {
			err = kstrtol(buf, 10, &result);
			if (err)
				return err;
			if ((result >= 1) && (result <= 5))
				sd->aggressiveness = result;
				settings_updated = true;
		}
	}

	if (settings_updated) {
		mutex_lock(&dc->lock);
		if (!dc->enabled) {
			mutex_unlock(&dc->lock);
			return -ENODEV;
		}
		tegra_dc_get(dc);
		if (sd->enable) {
			tegra_dc_unmask_interrupt(dc, SMARTDIM_INT);
			tegra_sd_stop(dc);
			tegra_sd_init(dc);
		} else {
			tegra_sd_stop(dc);
		}
		tegra_dc_put(dc);
		mutex_unlock(&dc->lock);
	}

	if (!sd->enable && sd->bl_device)
		backlight_update_status(sd->bl_device);

	return res;
}

#define NVSD_PRINT_REG(__name) { \
	u32 val = tegra_dc_readl(dc, __name); \
	res += snprintf(buf + res, PAGE_SIZE - res, #__name ": 0x%08x\n", \
	val); \
}

static ssize_t nvsd_registers_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct device *dev = container_of((kobj->parent), struct device, kobj);
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	ssize_t res = 0;

	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return -ENODEV;
	}

	tegra_dc_get(dc);

	NVSD_PRINT_REG(nvdisp_sd_hist_ctrl_r());
	NVSD_PRINT_REG(nvdisp_sd_gain_ctrl_r());
	NVSD_PRINT_REG(nvdisp_sd_hist_over_sat_r());
	NVSD_PRINT_REG(nvdisp_sd_hist_int_bounds_r());

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return res;
}

/* Sysfs initializer */
int tegra_sd_create_sysfs(struct device *dev)
{
	int retval = 0;
	nvsd_kobj = kobject_create_and_add("smartdimmer", &dev->kobj);

	if (!nvsd_kobj)
		return -ENOMEM;

	retval = sysfs_create_group(nvsd_kobj, &nvsd_attr_group);
	if (retval) {
		kobject_put(nvsd_kobj);
		dev_err(dev, "%s: failed to create attributes\n", __func__);
	}
	return retval;
}
EXPORT_SYMBOL(tegra_sd_create_sysfs);

/* Sysfs destructor */
void tegra_sd_remove_sysfs(struct device *dev)
{
	if (nvsd_kobj) {
		sysfs_remove_group(nvsd_kobj, &nvsd_attr_group);
		kobject_put(nvsd_kobj);
	}
}
EXPORT_SYMBOL(tegra_sd_remove_sysfs);
