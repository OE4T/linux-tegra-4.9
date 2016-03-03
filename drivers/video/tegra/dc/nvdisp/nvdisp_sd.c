/*
 * drivers/video/tegra/dc/nvdisp_sd.c
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION, All rights reserved.
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
#define SD3_MAX_HIST_BINS	255
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
		0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4144, 0x4128
	},
	{
		0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c,
		0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c,
		0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c,
		0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x428c, 0x4260, 0x4170
	},
	{
		0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4,
		0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4,
		0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4,
		0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x43d4, 0x439c, 0x42d0, 0x4184
	},
	{
		0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c,
		0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c,
		0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c, 0x451c,
		0x451c, 0x451c, 0x451c, 0x451c, 0x44ec, 0x443c, 0x4320, 0x419c
	},
	{
		0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664,
		0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664,
		0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664, 0x4664,
		0x4664, 0x4664, 0x4664, 0x4620, 0x457c, 0x447c, 0x432c, 0x4198
	},
	{
		0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac,
		0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac,
		0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac, 0x47ac,
		0x47ac, 0x47ac, 0x476c, 0x46d8, 0x45f0, 0x44bc, 0x434c, 0x41a0
	},
	{
		0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4,
		0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4,
		0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4, 0x48f4,
		0x48f4, 0x48b8, 0x482c, 0x4758, 0x463c, 0x44e8, 0x4360, 0x41a4
	},
	{
		0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c,
		0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c,
		0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c, 0x4a3c,
		0x4a04, 0x4980, 0x48b8, 0x47b0, 0x4674, 0x4504, 0x4368, 0x41a8
	},
	{
		0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84,
		0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84,
		0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b84, 0x4b64,
		0x4af4, 0x4a3c, 0x4948, 0x481c, 0x46c0, 0x4538, 0x4388, 0x41b4
	},
	{
		0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc,
		0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc,
		0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4ccc, 0x4cac, 0x4c40,
		0x4b94, 0x4aac, 0x4990, 0x4848, 0x46d4, 0x4540, 0x4384, 0x41b0
	},
	{
		0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14,
		0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14,
		0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e14, 0x4e04, 0x4dac, 0x4d10,
		0x4c38, 0x4b2c, 0x49f0, 0x4890, 0x4708, 0x455c, 0x4398, 0x41b8
	},
	{
		0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c,
		0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c,
		0x4f5c, 0x4f5c, 0x4f5c, 0x4f5c, 0x4f4c, 0x4ef4, 0x4e60, 0x4d90,
		0x4c90, 0x4b64, 0x4a10, 0x489c, 0x4708, 0x4558, 0x4390, 0x41b0
	},
	{
		0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0,
		0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0, 0x50a0,
		0x50a0, 0x50a0, 0x50a0, 0x509c, 0x5058, 0x4fd4, 0x4f10, 0x4e20,
		0x4d00, 0x4bb8, 0x4a50, 0x48c8, 0x4728, 0x456c, 0x439c, 0x41b4
	},
	{
		0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8,
		0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8, 0x51e8,
		0x51e8, 0x51e8, 0x51e8, 0x51b8, 0x5140, 0x5090, 0x4fa8, 0x4e94,
		0x4d5c, 0x4c00, 0x4a84, 0x48ec, 0x4740, 0x4578, 0x43a0, 0x41b8
	},
	{
		0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330,
		0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330, 0x5330,
		0x5330, 0x5330, 0x5314, 0x52ac, 0x5208, 0x5130, 0x5028, 0x4ef8,
		0x4da8, 0x4c38, 0x4aac, 0x4908, 0x4750, 0x4584, 0x43a8, 0x41b8
	},
	{
		0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478,
		0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478, 0x5478,
		0x5478, 0x5468, 0x5414, 0x537c, 0x52b0, 0x51b4, 0x5090, 0x4f48,
		0x4de4, 0x4c64, 0x4acc, 0x491c, 0x475c, 0x4588, 0x43a8, 0x41b8
	},
	{
		0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0,
		0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0, 0x55c0,
		0x55bc, 0x5574, 0x54ec, 0x542c, 0x533c, 0x5224, 0x50e8, 0x4f8c,
		0x4e14, 0x4c84, 0x4ae0, 0x492c, 0x4764, 0x458c, 0x43a8, 0x41b8
	},
	{
		0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708,
		0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708, 0x5708,
		0x56d4, 0x565c, 0x55a8, 0x54c0, 0x53b0, 0x5280, 0x512c, 0x4fbc,
		0x4e38, 0x4c9c, 0x4af0, 0x4934, 0x4764, 0x458c, 0x43a8, 0x41b8
	},
	{
		0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850,
		0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5850, 0x5830,
		0x57c4, 0x571c, 0x5644, 0x553c, 0x5410, 0x52c8, 0x5160, 0x4fe4,
		0x4e50, 0x4cac, 0x4af8, 0x4934, 0x4764, 0x458c, 0x43a4, 0x41b4
	},
	{
		0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5998,
		0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5998, 0x5990, 0x5948,
		0x58bc, 0x57f4, 0x56fc, 0x55dc, 0x549c, 0x5340, 0x51c8, 0x503c,
		0x4e9c, 0x4ce8, 0x4b28, 0x495c, 0x4784, 0x45a0, 0x43b0, 0x41bc
	},
	{
		0x5ae0, 0x5ae0, 0x5ae0, 0x5ae0, 0x5ae0, 0x5ae0, 0x5ae0, 0x5ae0,
		0x5ae0, 0x5ae0, 0x5ae0, 0x5ae0, 0x5ae0, 0x5ae0, 0x5aa8, 0x5a28,
		0x596c, 0x5880, 0x5768, 0x5630, 0x54d8, 0x5368, 0x51e4, 0x504c,
		0x4ea0, 0x4cec, 0x4b24, 0x4954, 0x477c, 0x4598, 0x43ac, 0x41b8
	},
	{
		0x5c28, 0x5c28, 0x5c28, 0x5c28, 0x5c28, 0x5c28, 0x5c28, 0x5c28,
		0x5c28, 0x5c28, 0x5c28, 0x5c28, 0x5c28, 0x5c14, 0x5bb4, 0x5b10,
		0x5a38, 0x5930, 0x5800, 0x56b4, 0x554c, 0x53cc, 0x5238, 0x5090,
		0x4edc, 0x4d1c, 0x4b4c, 0x4974, 0x4790, 0x45a8, 0x43b4, 0x41bc
	},
	{
		0x5d70, 0x5d70, 0x5d70, 0x5d70, 0x5d70, 0x5d70, 0x5d70, 0x5d70,
		0x5d70, 0x5d70, 0x5d70, 0x5d70, 0x5d68, 0x5d18, 0x5c84, 0x5bb0,
		0x5ab0, 0x598c, 0x5844, 0x56e0, 0x5568, 0x53dc, 0x523c, 0x5090,
		0x4ed4, 0x4d10, 0x4b40, 0x4964, 0x4784, 0x459c, 0x43ac, 0x41b8
	},
	{
		0x5eb8, 0x5eb8, 0x5eb8, 0x5eb8, 0x5eb8, 0x5eb8, 0x5eb8, 0x5eb8,
		0x5eb8, 0x5eb8, 0x5eb8, 0x5eb8, 0x5e90, 0x5e18, 0x5d60, 0x5c70,
		0x5b54, 0x5a18, 0x58bc, 0x574c, 0x55c0, 0x5428, 0x527c, 0x50c4,
		0x4f00, 0x4d34, 0x4b5c, 0x497c, 0x4794, 0x45a8, 0x43b4, 0x41bc
	},
	{
		0x6000, 0x6000, 0x6000, 0x6000, 0x6000, 0x6000, 0x6000, 0x6000,
		0x6000, 0x6000, 0x6000, 0x5ff8, 0x5fa4, 0x5f04, 0x5e28, 0x5d1c,
		0x5be8, 0x5a98, 0x592c, 0x57a8, 0x5610, 0x5468, 0x52b4, 0x50f4,
		0x4f28, 0x4d54, 0x4b74, 0x4990, 0x47a4, 0x45b0, 0x43bc, 0x41c0
	},
	{
		0x6144, 0x6144, 0x6144, 0x6144, 0x6144, 0x6144, 0x6144, 0x6144,
		0x6144, 0x6144, 0x6144, 0x6104, 0x6074, 0x5fa0, 0x5e9c, 0x5d70,
		0x5c24, 0x5abc, 0x593c, 0x57ac, 0x5608, 0x545c, 0x52a0, 0x50dc,
		0x4f0c, 0x4d38, 0x4b5c, 0x4978, 0x4790, 0x45a0, 0x43b0, 0x41b8
	},
	{
		0x628c, 0x628c, 0x628c, 0x628c, 0x628c, 0x628c, 0x628c, 0x628c,
		0x628c, 0x628c, 0x6278, 0x6208, 0x614c, 0x605c, 0x5f3c, 0x5df8,
		0x5c94, 0x5b1c, 0x5990, 0x57f0, 0x5644, 0x548c, 0x52c8, 0x50fc,
		0x4f28, 0x4d4c, 0x4b6c, 0x4984, 0x4798, 0x45a8, 0x43b4, 0x41bc
	},
	{
		0x63d4, 0x63d4, 0x63d4, 0x63d4, 0x63d4, 0x63d4, 0x63d4, 0x63d4,
		0x63d4, 0x63d4, 0x6390, 0x62f4, 0x6214, 0x6100, 0x5fc8, 0x5e70,
		0x5cfc, 0x5b74, 0x59d8, 0x5830, 0x5678, 0x54b8, 0x52ec, 0x5118,
		0x4f40, 0x4d60, 0x4b78, 0x4990, 0x47a0, 0x45ac, 0x43b4, 0x41bc
	},
	{
		0x651c, 0x651c, 0x651c, 0x651c, 0x651c, 0x651c, 0x651c, 0x651c,
		0x651c, 0x6504, 0x648c, 0x63c4, 0x62c4, 0x6198, 0x6044, 0x5ed8,
		0x5d54, 0x5bbc, 0x5a14, 0x5860, 0x56a4, 0x54d8, 0x5308, 0x5130,
		0x4f50, 0x4d6c, 0x4b84, 0x4994, 0x47a4, 0x45b0, 0x43b8, 0x41bc
	},
	{
		0x6664, 0x6664, 0x6664, 0x6664, 0x6664, 0x6664, 0x6664, 0x6664,
		0x6664, 0x6618, 0x6570, 0x6480, 0x6360, 0x6218, 0x60b4, 0x5f34,
		0x5da0, 0x5bfc, 0x5a4c, 0x588c, 0x56c4, 0x54f4, 0x5320, 0x5140,
		0x4f60, 0x4d78, 0x4b8c, 0x499c, 0x47a8, 0x45b0, 0x43b8, 0x41bc
	},
	{
		0x67ac, 0x67ac, 0x67ac, 0x67ac, 0x67ac, 0x67ac, 0x67ac, 0x67ac,
		0x6790, 0x670c, 0x6638, 0x6528, 0x63ec, 0x628c, 0x6110, 0x5f80,
		0x5de0, 0x5c30, 0x5a74, 0x58b0, 0x56e0, 0x550c, 0x5330, 0x5150,
		0x4f68, 0x4d7c, 0x4b90, 0x499c, 0x47a8, 0x45b0, 0x43b8, 0x41bc
	},
	{
		0x68f4, 0x68f4, 0x68f4, 0x68f4, 0x68f4, 0x68f4, 0x68f4, 0x68f4,
		0x68a0, 0x67e8, 0x66e8, 0x65b8, 0x6460, 0x62ec, 0x6160, 0x5fc4,
		0x5e14, 0x5c5c, 0x5a98, 0x58c8, 0x56f4, 0x551c, 0x533c, 0x5158,
		0x4f6c, 0x4d80, 0x4b90, 0x499c, 0x47a8, 0x45b0, 0x43b8, 0x41bc
	},
	{
		0x6a3c, 0x6a3c, 0x6a3c, 0x6a3c, 0x6a3c, 0x6a3c, 0x6a3c, 0x6a1c,
		0x698c, 0x68a4, 0x6784, 0x6634, 0x64c4, 0x633c, 0x61a4, 0x5ff8,
		0x5e40, 0x5c7c, 0x5ab0, 0x58dc, 0x5704, 0x5524, 0x5340, 0x5158,
		0x4f70, 0x4d80, 0x4b90, 0x499c, 0x47a8, 0x45b0, 0x43b4, 0x41bc
	},
	{
		0x6b84, 0x6b84, 0x6b84, 0x6b84, 0x6b84, 0x6b84, 0x6b80, 0x6b24,
		0x6a5c, 0x6948, 0x6804, 0x669c, 0x6518, 0x6380, 0x61d4, 0x6020,
		0x5e5c, 0x5c94, 0x5ac0, 0x58e8, 0x570c, 0x5528, 0x5344, 0x5158,
		0x4f6c, 0x4d7c, 0x4b8c, 0x4998, 0x47a4, 0x45ac, 0x43b4, 0x41b8
	},
	{
		0x6ccc, 0x6ccc, 0x6ccc, 0x6ccc, 0x6ccc, 0x6ccc, 0x6cc0, 0x6c3c,
		0x6b54, 0x6a2c, 0x68d4, 0x6758, 0x65c8, 0x6420, 0x6268, 0x60a8,
		0x5ed8, 0x5d04, 0x5b28, 0x5948, 0x5760, 0x5578, 0x5388, 0x5198,
		0x4fa4, 0x4dac, 0x4bb4, 0x49b8, 0x47bc, 0x45c0, 0x43c0, 0x41c0
	},
	{
		0x6e14, 0x6e14, 0x6e14, 0x6e14, 0x6e14, 0x6e14, 0x6dd0, 0x6d0c,
		0x6bf4, 0x6aa4, 0x6930, 0x67a0, 0x65fc, 0x6448, 0x6288, 0x60bc,
		0x5ee8, 0x5d0c, 0x5b2c, 0x5948, 0x575c, 0x5570, 0x5380, 0x5190,
		0x4f9c, 0x4da4, 0x4bac, 0x49b0, 0x47b8, 0x45b8, 0x43bc, 0x41bc
	},
	{
		0x6f5c, 0x6f5c, 0x6f5c, 0x6f5c, 0x6f5c, 0x6f4c, 0x6eb8, 0x6db8,
		0x6c74, 0x6b08, 0x697c, 0x67d8, 0x6624, 0x6464, 0x6298, 0x60c4,
		0x5ee8, 0x5d08, 0x5b24, 0x593c, 0x5754, 0x5564, 0x5374, 0x5184,
		0x4f8c, 0x4d98, 0x4ba0, 0x49a8, 0x47b0, 0x45b4, 0x43b8, 0x41bc
	},
	{
		0x70a0, 0x70a0, 0x70a0, 0x70a0, 0x70a0, 0x7054, 0x6f74, 0x6e40,
		0x6cdc, 0x6b50, 0x69b0, 0x67fc, 0x663c, 0x6470, 0x629c, 0x60c4,
		0x5ee4, 0x5d00, 0x5b18, 0x5930, 0x5744, 0x5554, 0x5364, 0x5174,
		0x4f80, 0x4d88, 0x4b94, 0x499c, 0x47a4, 0x45ac, 0x43b4, 0x41b8
	},
	{
		0x71e8, 0x71e8, 0x71e8, 0x71e8, 0x71e8, 0x7168, 0x7064, 0x6f14,
		0x6d98, 0x6bfc, 0x6a4c, 0x688c, 0x66c0, 0x64e8, 0x630c, 0x612c,
		0x5f44, 0x5d58, 0x5b6c, 0x5978, 0x5788, 0x5590, 0x539c, 0x51a4,
		0x4fa8, 0x4db0, 0x4bb4, 0x49b8, 0x47b8, 0x45bc, 0x43bc, 0x41bc
	},
	{
		0x7330, 0x7330, 0x7330, 0x7330, 0x7300, 0x7220, 0x70e4, 0x6f6c,
		0x6dd0, 0x6c20, 0x6a60, 0x6894, 0x66bc, 0x64e0, 0x62fc, 0x6118,
		0x5f2c, 0x5d40, 0x5b50, 0x5960, 0x576c, 0x5578, 0x5384, 0x518c,
		0x4f94, 0x4d9c, 0x4ba4, 0x49a8, 0x47ac, 0x45b4, 0x43b8, 0x41bc
	},
	{
		0x7478, 0x7478, 0x7478, 0x7478, 0x7414, 0x730c, 0x71b0, 0x7020,
		0x6e74, 0x6cb4, 0x6ae8, 0x6910, 0x6730, 0x654c, 0x6360, 0x6174,
		0x5f80, 0x5d8c, 0x5b98, 0x59a0, 0x57a8, 0x55b0, 0x53b4, 0x51b8,
		0x4fb8, 0x4dbc, 0x4bbc, 0x49c0, 0x47c0, 0x45c0, 0x43c0, 0x41c0
	},
	{
		0x75c0, 0x75c0, 0x75c0, 0x75a8, 0x74d0, 0x7380, 0x71f4, 0x7048,
		0x6e84, 0x6cb4, 0x6adc, 0x68fc, 0x6714, 0x652c, 0x633c, 0x614c,
		0x5f5c, 0x5d68, 0x5b74, 0x597c, 0x5784, 0x558c, 0x5394, 0x5198,
		0x4fa0, 0x4da4, 0x4ba8, 0x49ac, 0x47b0, 0x45b4, 0x43b8, 0x41bc
	},
	{
		0x7708, 0x7708, 0x7708, 0x76c4, 0x75b4, 0x7440, 0x729c, 0x70e0,
		0x6f10, 0x6d34, 0x6b50, 0x6968, 0x6778, 0x6588, 0x6394, 0x619c,
		0x5fa4, 0x5dac, 0x5bb0, 0x59b4, 0x57b8, 0x55bc, 0x53bc, 0x51c0,
		0x4fc0, 0x4dc0, 0x4bc0, 0x49c0, 0x47c0, 0x45c0, 0x43c0, 0x41c0
	},
	{
		0x7850, 0x7850, 0x784c, 0x7778, 0x7610, 0x746c, 0x72a8, 0x70d4,
		0x6ef4, 0x6d10, 0x6b24, 0x6938, 0x6744, 0x6554, 0x635c, 0x6168,
		0x5f70, 0x5d78, 0x5b80, 0x5988, 0x578c, 0x5590, 0x5398, 0x519c,
		0x4fa0, 0x4da4, 0x4ba8, 0x49ac, 0x47b0, 0x45b4, 0x43b8, 0x41bc
	},
	{
		0x7998, 0x7998, 0x7974, 0x7854, 0x76c4, 0x7504, 0x7330, 0x7150,
		0x6f68, 0x6d78, 0x6b84, 0x6990, 0x6798, 0x65a0, 0x63a4, 0x61a8,
		0x5fac, 0x5db0, 0x5bb4, 0x59b8, 0x57b8, 0x55b8, 0x53bc, 0x51bc,
		0x4fbc, 0x4dbc, 0x4bc0, 0x49c0, 0x47c0, 0x45c0, 0x43c0, 0x41c0
	},
	{
		0x7ae0, 0x7ae0, 0x7a1c, 0x788c, 0x76c4, 0x74e8, 0x7304, 0x7114,
		0x6f24, 0x6d34, 0x6b3c, 0x6948, 0x6750, 0x6558, 0x6360, 0x6164,
		0x5f6c, 0x5d74, 0x5b78, 0x5980, 0x5784, 0x5588, 0x5390, 0x5194,
		0x4f98, 0x4d9c, 0x4ba4, 0x49a8, 0x47ac, 0x45b0, 0x43b4, 0x41bc
	},
	{
		0x7c28, 0x7c20, 0x7ae4, 0x7928, 0x7748, 0x755c, 0x736c, 0x7174,
		0x6f7c, 0x6d84, 0x6b88, 0x6990, 0x6794, 0x6598, 0x6398, 0x619c,
		0x5fa0, 0x5da0, 0x5ba4, 0x59a8, 0x57a8, 0x55ac, 0x53ac, 0x51b0,
		0x4fb0, 0x4db4, 0x4bb4, 0x49b8, 0x47b8, 0x45b8, 0x43bc, 0x41bc
	},
	{
		0x7d70, 0x7d28, 0x7b88, 0x79a8, 0x77b8, 0x75c0, 0x73c8, 0x71cc,
		0x6fcc, 0x6dcc, 0x6bd0, 0x69d0, 0x67d0, 0x65d0, 0x63d0, 0x61d0,
		0x5fcc, 0x5dcc, 0x5bcc, 0x59cc, 0x57cc, 0x55c8, 0x53c8, 0x51c8,
		0x4fc8, 0x4dc4, 0x4bc4, 0x49c4, 0x47c4, 0x45c0, 0x43c0, 0x41c0
	},
	{
		0x7eb8, 0x7d30, 0x7b38, 0x7940, 0x7744, 0x7548, 0x734c, 0x7154,
		0x6f58, 0x6d5c, 0x6b60, 0x6964, 0x6768, 0x656c, 0x6370, 0x6174,
		0x5f78, 0x5d7c, 0x5b84, 0x5988, 0x578c, 0x5590, 0x5394, 0x5198,
		0x4f9c, 0x4da0, 0x4ba4, 0x49a8, 0x47ac, 0x45b0, 0x43b4, 0x41bc
	},
	{
		0x8000, 0x7dfc, 0x7bf8, 0x79f8, 0x77f4, 0x75f4, 0x73f0, 0x71f0,
		0x6fec, 0x6dec, 0x6be8, 0x69e8, 0x67e4, 0x65e4, 0x63e0, 0x61e0,
		0x5fdc, 0x5ddc, 0x5bd8, 0x59d8, 0x57d4, 0x55d4, 0x53d0, 0x51d0,
		0x4fcc, 0x4dcc, 0x4bc8, 0x49c8, 0x47c4, 0x45c4, 0x43c0, 0x41c0
	}
};

static int const sd_backlight_table[MAX_PIXEL_ADAPTATION_PERCENT+1] = {
	255, 245, 235, 226, 217, 209, 201, 194, 187, 180, 174, 168, 162,
	157, 152, 147, 142, 138, 134, 130, 126, 122, 119, 115, 112, 109,
	106, 103, 100, 98, 95, 93, 90, 88, 86, 84, 82, 80, 78, 76, 74,
	73, 71, 69, 68, 66, 65, 63, 62, 61, 59
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
NVSD_ATTR(sd_window_enable);
NVSD_ATTR(sd_window);
NVSD_ATTR(debug_info);
NVSD_ATTR(sw_update_delay);
NVSD_ATTR(brightness);

static struct attribute *nvsd_attrs[] = {
	NVSD_ATTRS_ENTRY(enable),
	NVSD_ATTRS_ENTRY(aggressiveness),
	NVSD_ATTRS_ENTRY(sd_window_enable),
	NVSD_ATTRS_ENTRY(sd_window),
	NVSD_ATTRS_ENTRY(registers),
	NVSD_ATTRS_ENTRY(debug_info),
	NVSD_ATTRS_ENTRY(sw_update_delay),
	NVSD_ATTRS_ENTRY(brightness),
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
static void tegra_dc_calc_params(struct tegra_dc_sd_settings *sd)
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

	if (sd->upper_bound == sd->over_saturated_bin)
		sd->upper_bound++;

	if (pixel_adaptation == MAX_PIXEL_ADAPTATION_PERCENT)
		sd->lower_bound = SMARTDIMMER_MIN_BOUND;

	gain_table_index = (pixel_adaptation * 100 /
			SD3_ADAPTATION_STEP_PERCENT) / 100;

	for (i = 0; i < GAIN_TABLE_MAX_ENTRIES; i++) {
		if (sd->gain_luts_parsed)
			sd->gain_table[i] =
				sd->pixel_gain_tables[gain_table_index][i];
		else
			sd->gain_table[i] =
				pixel_gain_tables[gain_table_index][i];
	}

	/* Since pixel adapation step is 2, we only have the even
	 * values in the backlight luts. Hence we have divided the
	 * pixel adaptation by 2
	 * */
	if (sd->gain_luts_parsed)
		sd->new_backlight = sd->backlight_table[pixel_adaptation/2];
	else
		sd->new_backlight = sd_backlight_table[pixel_adaptation/2];
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
		sd->over_saturated_bin = sd->last_over_saturated_bin;
		return 1;
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
	unsigned int gain_val;
	unsigned int ret_val;

	struct tegra_dc_sd_settings *sd = dc->out->sd_settings;

	sd->over_saturated_bin = tegra_dc_readl(dc,
		nvdisp_sd_hist_over_sat_r());
	if ((sd->over_saturated_bin == 255) || (sd->over_saturated_bin == 0))
		if (sd->over_saturated_bin == sd->last_over_saturated_bin)
			return false;

	if (sd->update_sd) {
		/* Calculate Parameters */
		tegra_dc_calc_params(sd);
		/* Set the parameters */
		ret_val = tegra_sd_set_params(dc, sd);
		if (ret_val)
			return false;
		sd->update_sd = false;
		sd->frame_runner = 0;
	}

	sd->frame_runner++;

	/* Calculate the gain step based on current gain and phase steps*/
	phases_left = sd->phase_in_steps - sd->last_phase_step;

	if (sd->sw_update_delay) {
		if (phases_left > 0)
			if (sd->frame_runner % sd->sw_update_delay)
				return false;
	}

	for (i = 0; i < GAIN_TABLE_MAX_ENTRIES; i++) {
		if (sd->current_gain_table[i] < sd->gain_table[i]) {
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
		gain_val = sd->current_gain_table[i];
		tegra_dc_writel(dc, nvdisp_sd_gain_rg_rval_f(gain_val) |
				nvdisp_sd_gain_rg_gval_f(gain_val),
				nvdisp_sd_gain_rg_r());
		tegra_dc_writel(dc, nvdisp_sd_gain_b_val_f(gain_val),
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
		sd->last_over_saturated_bin = sd->over_saturated_bin;
	}

	return true;
}

static void tegra_sd_set_over_saturated_pixels(struct tegra_dc *dc)
{
	int reg_val;
	int total_pixel_count;
	struct tegra_dc_sd_settings *sd;

	sd = dc->out->sd_settings;

	reg_val = tegra_dc_readl(dc,
		nvdisp_sd_hist_ctrl_r());

	reg_val &= 0xff000000;

	if (sd->sd_window_enable) {
		total_pixel_count = sd->sd_window.h_size * sd->sd_window.v_size;
	} else {
		total_pixel_count = dc->mode.h_active *
			dc->mode.v_active;
	}

	sd->num_over_saturated_pixels = (total_pixel_count *
		aggr_table[sd->aggressiveness - 1]) / 100;

	reg_val |= nvdisp_sd_hist_ctrl_max_pixel_f(
		sd->num_over_saturated_pixels);
	tegra_dc_writel(dc, reg_val,
		nvdisp_sd_hist_ctrl_r());

	return;
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
	int reg_val;

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
		sd->last_over_saturated_bin = 256;
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

	if (sd->sd_window_enable) {
		total_pixel_count = sd->sd_window.h_size * sd->sd_window.v_size;
	} else {
		total_pixel_count = dc->mode.h_active *
			dc->mode.v_active;
	}

	sd->num_over_saturated_pixels = (total_pixel_count *
			aggr_table[sd->aggressiveness - 1]) / 100;
	/* enable the histogram function */

	reg_val = nvdisp_sd_hist_ctrl_max_pixel_f(
			sd->num_over_saturated_pixels) |
		nvdisp_sd_hist_int_bounds_set_enable_f();

	if (sd->sd_window_enable) {
		reg_val |= 	nvdisp_sd_hist_ctrl_set_window_enable_f();
	}

	tegra_dc_writel(dc, reg_val,
			nvdisp_sd_hist_ctrl_r());

	if (sd->sd_window_enable) {
	/*Adding the windowing function*/
		tegra_dc_writel(dc,
			nvdisp_sd_hist_win_pos_v_f(sd->sd_window.v_position) |
			nvdisp_sd_hist_win_pos_h_f(sd->sd_window.h_position),
			nvdisp_sd_hist_win_pos_r());

		tegra_dc_writel(dc,
			nvdisp_sd_hist_win_size_height_f(sd->sd_window.h_size) |
			nvdisp_sd_hist_win_size_width_f(sd->sd_window.v_size),
			nvdisp_sd_hist_win_size_r());
	}

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

	sd->enable = 0;
	/*Disable the gain function*/
	tegra_dc_writel(dc, 0x0,
		nvdisp_sd_gain_ctrl_r());

	/*Disable the interrupt bounds*/
	tegra_dc_writel(dc, 0x0,
		nvdisp_sd_hist_int_bounds_r());

	/*Disable the histogram function*/
	tegra_dc_writel(dc, 0, nvdisp_sd_hist_ctrl_r());

	//Set brightness back to 255
	if (_sd_brightness)
		atomic_set(_sd_brightness, 255);

	sd->old_backlight = 255;
	sd->new_backlight = 255;
	sd->phase_in_steps = 0;
	sd->last_phase_step = 0;

	//Update the backlight device
	if (sd->bl_device)
		backlight_update_status(sd->bl_device);

	return;
}

static ssize_t nvsd_settings_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct device *dev = container_of((kobj->parent), struct device, kobj);
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_sd_settings *sd;
	ssize_t res = 0;

	if (!dc)
		return res;

	sd = dc->out->sd_settings;

	mutex_lock(&dc->lock);
	if (sd) {
		if (IS_NVSD_ATTR(enable))
			res = snprintf(buf, PAGE_SIZE, "%d\n",
					sd->enable);
		else if (IS_NVSD_ATTR(aggressiveness))
			res = snprintf(buf, PAGE_SIZE, "%d\n",
					sd->aggressiveness);
		else if (IS_NVSD_ATTR(sd_window_enable))
			res = snprintf(buf, PAGE_SIZE, "%d\n",
					sd->sd_window_enable);
		else if (IS_NVSD_ATTR(sd_window))
			res = snprintf(buf, PAGE_SIZE,
					"x: %d, y: %d, w: %d, h: %d\n",
					sd->sd_window.h_position,
					sd->sd_window.v_position,
					sd->sd_window.h_size,
					sd->sd_window.v_size);
		else if (IS_NVSD_ATTR(debug_info))
			res = snprintf(buf, PAGE_SIZE,
					"Upper bound: %d\n"
					"Lower Bound: %d\n"
					"Oversaturated Pixels: %d\n"
					"oversaturated bin: %d\n"
					"Last Phase Step: %d\n"
					"Aggressiveness: %d\n"
					"Brightness: %d\n",
					sd->upper_bound,
					sd->lower_bound,
					sd->num_over_saturated_pixels,
					sd->over_saturated_bin,
					sd->last_phase_step,
					sd->aggressiveness,
					sd->old_backlight);
		else if (IS_NVSD_ATTR(sw_update_delay))
			res = snprintf(buf, PAGE_SIZE, "%d\n",
					sd->sw_update_delay);
		else if (IS_NVSD_ATTR(brightness)) {
			res = snprintf(buf, PAGE_SIZE,
					"%d\n",
					atomic_read(sd->sd_brightness));
		}
	} else {
		res = -EINVAL;
	}
	mutex_unlock(&dc->lock);

	return res;
}

#define nvsd_check_and_update(_min, _max, _varname) { \
	long int val; \
	int err = kstrtol(buf, 10, &val); \
	if (err) \
		return err; \
	if (val >= _min && val <= _max) { \
		if (sd->_varname != val) { \
		sd->_varname = val; \
		_varname##_updated = true; \
	} } }

#define nvsd_get_multi(_ele, _num, _act, _min, _max) { \
	char *b, *c, *orig_b; \
	b = orig_b = kstrdup(buf, GFP_KERNEL); \
	for (_act = 0; _act < _num; _act++) { \
		if (!b) \
			break; \
		b = strim(b); \
		c = strsep(&b, " "); \
		if (!strlen(c)) \
			break; \
		_ele[_act] = simple_strtol(c, NULL, 10); \
		if (_ele[_act] < _min || _ele[_act] > _max) \
			break; \
	} \
	if (orig_b) \
		kfree(orig_b); \
}

static ssize_t nvsd_settings_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct device *dev = container_of((kobj->parent), struct device, kobj);
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_sd_settings *sd;
	ssize_t res = count;
	bool enable_updated = false;
	bool aggressiveness_updated = false;
	bool sd_window_enable_updated = false;
	bool sw_update_delay_updated = false;
	bool old_backlight_updated = false;
	bool settings_updated = false;

	if (!dc)
		return res;

	sd = dc->out->sd_settings;

	mutex_lock(&dc->lock);
	if (sd) {
		if (IS_NVSD_ATTR(enable)) {
			nvsd_check_and_update(0, 1, enable);
		} else if (IS_NVSD_ATTR(aggressiveness)) {
			nvsd_check_and_update(1,5, aggressiveness);
		} else if (IS_NVSD_ATTR(sd_window_enable)) {
			nvsd_check_and_update(0, 1, sd_window_enable);
			settings_updated = true;
		} else if (IS_NVSD_ATTR(sd_window)) {
			int ele[4], i = 0, num = 4;
			nvsd_get_multi(ele, num, i, 0, LONG_MAX);
			if (i == num) {
				sd->sd_window.h_position = ele[0];
				sd->sd_window.v_position = ele[1];
				sd->sd_window.h_size = ele[2];
				sd->sd_window.v_size = ele[3];
				settings_updated = true;
			} else {
				res = -EINVAL;
			}
		} else if (IS_NVSD_ATTR(debug_info)) {
			mutex_unlock(&dc->lock);
			return res;
		} else if (IS_NVSD_ATTR(sw_update_delay)) {
			nvsd_check_and_update(0,3,sw_update_delay);
			mutex_unlock(&dc->lock);
			return res;
		} else if (IS_NVSD_ATTR(brightness)) {
			int check_val = sd->old_backlight;
			nvsd_check_and_update(0,255,old_backlight);
			if (check_val != sd->old_backlight) {
				atomic_set(sd->sd_brightness, sd->old_backlight);
				sd->new_backlight = sd->old_backlight;
				if (sd->bl_device)
					backlight_update_status(sd->bl_device);
			}
			mutex_unlock(&dc->lock);
			return res;
		}
	}

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return -ENODEV;
	}

	tegra_dc_get(dc);

	if (aggressiveness_updated)
		tegra_sd_set_over_saturated_pixels(dc);

	if ((enable_updated) || settings_updated) {
		if (sd->enable) {
				tegra_dc_unmask_interrupt(dc, SMARTDIM_INT);
				tegra_sd_stop(dc);
				tegra_sd_init(dc);
		} else
			tegra_sd_stop(dc);
	}

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

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

/*wrapper function used for disabling/enabling prism*/
void tegra_sd_enbl_dsbl_prism(struct device *dev, bool status)
{
	struct tegra_dc_sd_settings *sd;
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	if (!dc || !dc->enabled)
		return;

	sd = dc->out->sd_settings;
	if (!sd)
		return;

	if (sd->enable == status)
		return;

	if (status)
		tegra_sd_init(dc);
	else
		tegra_sd_stop(dc);

	return;
}
EXPORT_SYMBOL(tegra_sd_enbl_dsbl_prism);

/* wrapper to enable/disable prism based on brightness */
void tegra_sd_check_prism_thresh(struct device *dev, int brightness)
{
	struct tegra_dc *dc = dev_get_drvdata(dev);
	struct tegra_dc_sd_settings *sd = dc->out->sd_settings;

	if (brightness <= sd->turn_off_brightness)
		tegra_sd_enbl_dsbl_prism(dev, false);
	else if (brightness > sd->turn_on_brightness)
		tegra_sd_enbl_dsbl_prism(dev, true);
}
EXPORT_SYMBOL(tegra_sd_check_prism_thresh);
