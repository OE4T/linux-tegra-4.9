/*
 *
 * Tegra GK20A GPU Debugger Driver Register Ops
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/bsearch.h>
#include <linux/nvhost_dbg_gpu_ioctl.h>

#include "gk20a/gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/regops_gk20a.h"
#include "regops_gm20b.h"

static const struct regop_offset_range gm20b_global_whitelist_ranges[] = {
	{ 0x000004f0,   1 },
	{ 0x00001a00,   3 },
	{ 0x0000259c,   1 },
	{ 0x0000280c,   1 },
	{ 0x00009400,   1 },
	{ 0x00009410,   1 },
	{ 0x00020200,   1 },
	{ 0x00022430,   7 },
	{ 0x00022548,   1 },
	{ 0x00100c18,   3 },
	{ 0x00100c84,   1 },
	{ 0x00100cc4,   1 },
	{ 0x00106640,   1 },
	{ 0x0010a0a8,   1 },
	{ 0x0010a4f0,   1 },
	{ 0x0010e064,   1 },
	{ 0x0010e164,   1 },
	{ 0x0010e490,   1 },
	{ 0x00110100,   1 },
	{ 0x00140028,   1 },
	{ 0x001408dc,   1 },
	{ 0x00140a5c,   1 },
	{ 0x001410dc,   1 },
	{ 0x0014125c,   1 },
	{ 0x0017e028,   1 },
	{ 0x0017e8dc,   1 },
	{ 0x0017ea5c,   1 },
	{ 0x0017f0dc,   1 },
	{ 0x0017f25c,   1 },
	{ 0x00180000,  68 },
	{ 0x00180200,  68 },
	{ 0x001a0000,  68 },
	{ 0x001b0000,  68 },
	{ 0x001b0200,  68 },
	{ 0x001b0400,  68 },
	{ 0x001b0600,  68 },
	{ 0x001b4000,   3 },
	{ 0x001b4010,   3 },
	{ 0x001b4020,   3 },
	{ 0x001b4040,   3 },
	{ 0x001b4050,   3 },
	{ 0x001b4060,  16 },
	{ 0x001b40a4,   1 },
	{ 0x001b4100,   6 },
	{ 0x001b4124,   2 },
	{ 0x001b8000,   7 },
	{ 0x001bc000,   7 },
	{ 0x001be000,   7 },
	{ 0x00400500,   1 },
	{ 0x00400700,   1 },
	{ 0x0040415c,   1 },
	{ 0x00405850,   1 },
	{ 0x00405908,   1 },
	{ 0x00405b40,   1 },
	{ 0x00405b50,   1 },
	{ 0x00406024,   1 },
	{ 0x00407010,   1 },
	{ 0x00407808,   1 },
	{ 0x0040803c,   1 },
	{ 0x0040880c,   1 },
	{ 0x00408910,   1 },
	{ 0x00408984,   1 },
	{ 0x004090a8,   1 },
	{ 0x004098a0,   1 },
	{ 0x0041000c,   1 },
	{ 0x00410110,   1 },
	{ 0x00410184,   1 },
	{ 0x00418384,   1 },
	{ 0x004184a0,   1 },
	{ 0x00418604,   1 },
	{ 0x00418680,   1 },
	{ 0x00418714,   1 },
	{ 0x0041881c,   1 },
	{ 0x004188b0,   1 },
	{ 0x004188c8,   2 },
	{ 0x00418b04,   1 },
	{ 0x00418c04,   1 },
	{ 0x00418c64,   2 },
	{ 0x00418c88,   1 },
	{ 0x00418cb4,   2 },
	{ 0x00418d00,   1 },
	{ 0x00418d28,   2 },
	{ 0x00418e08,   1 },
	{ 0x00418e1c,   2 },
	{ 0x00418f08,   1 },
	{ 0x00418f20,   2 },
	{ 0x00419000,   1 },
	{ 0x0041900c,   1 },
	{ 0x00419018,   1 },
	{ 0x00419854,   1 },
	{ 0x00419ab0,   1 },
	{ 0x00419ab8,   3 },
	{ 0x00419ac8,   1 },
	{ 0x00419c0c,   1 },
	{ 0x00419c8c,   3 },
	{ 0x00419ca8,   1 },
	{ 0x00419d08,   2 },
	{ 0x00419e00,   1 },
	{ 0x00419e0c,   1 },
	{ 0x00419e14,   2 },
	{ 0x00419e24,   2 },
	{ 0x00419e34,   2 },
	{ 0x00419e44,   4 },
	{ 0x00419ea4,   1 },
	{ 0x00419eb0,   1 },
	{ 0x0041a0a0,   1 },
	{ 0x0041a0a8,   1 },
	{ 0x0041a17c,   1 },
	{ 0x0041a890,   2 },
	{ 0x0041a8a0,   3 },
	{ 0x0041a8b0,   2 },
	{ 0x0041b014,   1 },
	{ 0x0041b0a0,   1 },
	{ 0x0041b0cc,   1 },
	{ 0x0041b0e8,   2 },
	{ 0x0041b1dc,   1 },
	{ 0x0041b1f8,   2 },
	{ 0x0041be14,   1 },
	{ 0x0041bea0,   1 },
	{ 0x0041becc,   1 },
	{ 0x0041bee8,   2 },
	{ 0x0041bfdc,   1 },
	{ 0x0041bff8,   2 },
	{ 0x0041c054,   1 },
	{ 0x0041c2b0,   1 },
	{ 0x0041c2b8,   3 },
	{ 0x0041c2c8,   1 },
	{ 0x0041c40c,   1 },
	{ 0x0041c48c,   3 },
	{ 0x0041c48c,   1 },
	{ 0x0041c4a8,   1 },
	{ 0x0041c508,   2 },
	{ 0x0041c508,   2 },
	{ 0x0041c600,   1 },
	{ 0x0041c60c,   1 },
	{ 0x0041c614,   2 },
	{ 0x0041c624,   2 },
	{ 0x0041c634,   2 },
	{ 0x0041c644,   4 },
	{ 0x0041c6a4,   1 },
	{ 0x0041c6b0,   1 },
	{ 0x0041ce04,   6 },
	{ 0x0041ce24,   2 },
	{ 0x0041ce34,   2 },
	{ 0x0041ce44,   4 },
	{ 0x0041ce58,   6 },
	{ 0x0041ce74,   9 },
	{ 0x0041cea4,   1 },
	{ 0x0041ceac,   2 },
	{ 0x0041cf30,   8 },
	{ 0x0041cf58,   2 },
	{ 0x00500384,   1 },
	{ 0x005004a0,   1 },
	{ 0x00500604,   1 },
	{ 0x00500680,   1 },
	{ 0x00500714,   1 },
	{ 0x0050081c,   1 },
	{ 0x005008c8,   2 },
	{ 0x00500b04,   1 },
	{ 0x00500c04,   1 },
	{ 0x00500c64,   2 },
	{ 0x00500c88,   1 },
	{ 0x00500cb4,   2 },
	{ 0x00500d00,   1 },
	{ 0x00500d28,   2 },
	{ 0x00500e08,   1 },
	{ 0x00500e1c,   2 },
	{ 0x00500f08,   1 },
	{ 0x00500f20,   2 },
	{ 0x00501000,   1 },
	{ 0x0050100c,   1 },
	{ 0x00501018,   1 },
	{ 0x00501854,   1 },
	{ 0x00501ab0,   1 },
	{ 0x00501ab8,   3 },
	{ 0x00501ac8,   1 },
	{ 0x00501c0c,   1 },
	{ 0x00501c8c,   3 },
	{ 0x00501ca8,   1 },
	{ 0x00501d08,   2 },
	{ 0x00501e00,   1 },
	{ 0x00501e0c,   1 },
	{ 0x00501e14,   2 },
	{ 0x00501e24,   2 },
	{ 0x00501e34,   2 },
	{ 0x00501e44,   4 },
	{ 0x00501ea4,   1 },
	{ 0x00501eb0,   1 },
	{ 0x005020a0,   1 },
	{ 0x005020a8,   1 },
	{ 0x0050217c,   1 },
	{ 0x00502890,   2 },
	{ 0x005028a0,   3 },
	{ 0x005028b0,   2 },
	{ 0x00503014,   1 },
	{ 0x005030a0,   1 },
	{ 0x005030cc,   1 },
	{ 0x005030e8,   2 },
	{ 0x005031dc,   1 },
	{ 0x005031f8,   2 },
	{ 0x00503e14,   1 },
	{ 0x00503ea0,   1 },
	{ 0x00503ecc,   1 },
	{ 0x00503ee8,   2 },
	{ 0x00503fdc,   1 },
	{ 0x00503ff8,   2 },
	{ 0x00504054,   1 },
	{ 0x005042b0,   1 },
	{ 0x005042b8,   3 },
	{ 0x005042c8,   1 },
	{ 0x0050440c,   1 },
	{ 0x0050448c,   3 },
	{ 0x005044a8,   1 },
	{ 0x00504508,   2 },
	{ 0x00504600,   1 },
	{ 0x0050460c,   1 },
	{ 0x00504614,   2 },
	{ 0x00504624,   2 },
	{ 0x00504634,   2 },
	{ 0x00504644,   4 },
	{ 0x005046a4,   1 },
	{ 0x005046b0,   1 },
	{ 0x00504c8c,   1 },
	{ 0x00504d08,   2 },
	{ 0x00504d0c,   1 },
	{ 0x00504e04,   6 },
	{ 0x00504e0c,   1 },
	{ 0x00504e14,   1 },
	{ 0x00504e24,   1 },
	{ 0x00504e24,   2 },
	{ 0x00504e34,   1 },
	{ 0x00504e34,   2 },
	{ 0x00504e44,   4 },
	{ 0x00504e48,   1 },
	{ 0x00504e58,   6 },
	{ 0x00504e74,   9 },
	{ 0x00504ea4,   1 },
	{ 0x00504eac,   2 },
	{ 0x00504f30,   8 },
	{ 0x00504f5c,   2 },
};
static const u32 gm20b_global_whitelist_ranges_count =
	ARRAY_SIZE(gm20b_global_whitelist_ranges);

/* context */

static const struct regop_offset_range gm20b_context_whitelist_ranges[] = {
	{ 0x0000280c,   1 },
	{ 0x00021c00,   4 },
	{ 0x00021c14,   3 },
	{ 0x00021c24,   1 },
	{ 0x00021c2c,   5 },
	{ 0x00021cb8,   2 },
	{ 0x00021d3c,   1 },
	{ 0x00021d44,   1 },
	{ 0x00021d4c,   1 },
	{ 0x00021d54,   1 },
	{ 0x00021d5c,   1 },
	{ 0x00021d64,   2 },
	{ 0x00100cc4,   1 },
	{ 0x00400500,   1 },
	{ 0x00405b40,   1 },
	{ 0x00418e00,   1 },
	{ 0x00418e34,   1 },
	{ 0x00418e40,   2 },
	{ 0x00418e58,   2 },
	{ 0x00419000,   1 },
	{ 0x00419864,   1 },
	{ 0x00419c8c,   3 },
	{ 0x00419c8c,   1 },
	{ 0x00419d08,   2 },
	{ 0x00419e04,   3 },
	{ 0x00419e14,   2 },
	{ 0x00419e24,   2 },
	{ 0x00419e34,   2 },
	{ 0x00419e44,   4 },
	{ 0x00419e4c,   1 },
	{ 0x00419e54,   1 },
	{ 0x00419e58,   6 },
	{ 0x00419e84,   5 },
	{ 0x00419ea4,   1 },
	{ 0x00419eac,   2 },
	{ 0x00419eb0,   1 },
	{ 0x00419ee8,   1 },
	{ 0x00419f30,   8 },
	{ 0x0041b0e8,   2 },
	{ 0x0041bee8,   2 },
	{ 0x0041c48c,   3 },
	{ 0x0041c48c,   1 },
	{ 0x0041c508,   2 },
	{ 0x0041c604,   3 },
	{ 0x0041c614,   2 },
	{ 0x0041c624,   2 },
	{ 0x0041c634,   2 },
	{ 0x0041c644,   4 },
	{ 0x0041c64c,   1 },
	{ 0x0041c654,   1 },
	{ 0x0041c658,   6 },
	{ 0x0041c684,   5 },
	{ 0x0041c6a4,   1 },
	{ 0x0041c6ac,   2 },
	{ 0x0041c6e8,   1 },
	{ 0x0041c730,   8 },
	{ 0x0041cc8c,   1 },
	{ 0x0041ce4c,   1 },
	{ 0x0041ce54,   1 },
	{ 0x0041cee8,   1 },
	{ 0x00501000,   1 },
	{ 0x00501c8c,   3 },
	{ 0x00501c8c,   1 },
	{ 0x00501d08,   2 },
	{ 0x00501e04,   3 },
	{ 0x00501e14,   2 },
	{ 0x00501e24,   2 },
	{ 0x00501e34,   2 },
	{ 0x00501e44,   4 },
	{ 0x00501e4c,   1 },
	{ 0x00501e54,   1 },
	{ 0x00501e58,   6 },
	{ 0x00501e84,   5 },
	{ 0x00501ea4,   1 },
	{ 0x00501eac,   2 },
	{ 0x00501ee8,   1 },
	{ 0x00501f30,   8 },
	{ 0x005030e8,   2 },
	{ 0x00503ee8,   2 },
	{ 0x0050448c,   3 },
	{ 0x0050448c,   1 },
	{ 0x00504508,   2 },
	{ 0x0050450c,   1 },
	{ 0x00504604,   3 },
	{ 0x0050460c,   1 },
	{ 0x00504614,   2 },
	{ 0x00504614,   4 },
	{ 0x00504624,   2 },
	{ 0x00504624,   2 },
	{ 0x00504634,   2 },
	{ 0x00504634,   2 },
	{ 0x00504644,   4 },
	{ 0x00504644,   1 },
	{ 0x0050464c,   3 },
	{ 0x00504658,   6 },
	{ 0x00504684,   5 },
	{ 0x00504684,   4 },
	{ 0x00504698,   1 },
	{ 0x005046a4,   1 },
	{ 0x005046a4,   1 },
	{ 0x005046ac,   2 },
	{ 0x005046e8,   1 },
	{ 0x005046f0,  24 },
	{ 0x00504730,   8 },
	{ 0x00504750,   2 },
	{ 0x00504758,   2 },
	{ 0x00504c8c,   1 },
	{ 0x00504d0c,   1 },
	{ 0x00504e0c,   1 },
	{ 0x00504e14,   4 },
	{ 0x00504e24,   2 },
	{ 0x00504e34,   2 },
	{ 0x00504e44,   1 },
	{ 0x00504e4c,   3 },
	{ 0x00504e84,   4 },
	{ 0x00504e98,   1 },
	{ 0x00504ea4,   1 },
	{ 0x00504ee8,   1 },
	{ 0x00504ef0,  24 },
	{ 0x00504f50,   2 },
	{ 0x00504f58,   2 },
};
static const u32 gm20b_context_whitelist_ranges_count =
	ARRAY_SIZE(gm20b_context_whitelist_ranges);

/* runcontrol */
static const u32 gm20b_runcontrol_whitelist[] = {
	0x00419e10,
	0x0041c610,
	0x0041ce10,
	0x00501e10,
	0x00504610,
	0x00504e10,
};
static const u32 gm20b_runcontrol_whitelist_count =
	ARRAY_SIZE(gm20b_runcontrol_whitelist);

static const struct regop_offset_range gm20b_runcontrol_whitelist_ranges[] = {
	{ 0x00419e10,   1 },
	{ 0x0041c610,   1 },
	{ 0x0041ce10,   1 },
	{ 0x00501e10,   1 },
	{ 0x00504610,   1 },
	{ 0x00504e10,   1 },
};
static const u32 gm20b_runcontrol_whitelist_ranges_count =
	ARRAY_SIZE(gm20b_runcontrol_whitelist_ranges);


/* quad ctl */
static const u32 gm20b_qctl_whitelist[] = {
};
static const u32 gm20b_qctl_whitelist_count =
	ARRAY_SIZE(gm20b_qctl_whitelist);

static const struct regop_offset_range gm20b_qctl_whitelist_ranges[] = {
};
static const u32 gm20b_qctl_whitelist_ranges_count =
	ARRAY_SIZE(gm20b_qctl_whitelist_ranges);

const struct regop_offset_range *gm20b_get_global_whitelist_ranges(void)
{
	return gm20b_global_whitelist_ranges;
}

int gm20b_get_global_whitelist_ranges_count(void)
{
	return gm20b_global_whitelist_ranges_count;
}

const struct regop_offset_range *gm20b_get_context_whitelist_ranges(void)
{
	return gm20b_context_whitelist_ranges;
}

int gm20b_get_context_whitelist_ranges_count(void)
{
	return gm20b_context_whitelist_ranges_count;
}

const u32 *gm20b_get_runcontrol_whitelist(void)
{
	return gm20b_runcontrol_whitelist;
}

int gm20b_get_runcontrol_whitelist_count(void)
{
	return gm20b_runcontrol_whitelist_count;
}

const struct regop_offset_range *gm20b_get_runcontrol_whitelist_ranges(void)
{
	return gm20b_runcontrol_whitelist_ranges;
}

int gm20b_get_runcontrol_whitelist_ranges_count(void)
{
	return gm20b_runcontrol_whitelist_ranges_count;
}

const u32 *gm20b_get_qctl_whitelist(void)
{
	return gm20b_qctl_whitelist;
}

int gm20b_get_qctl_whitelist_count(void)
{
	return gm20b_qctl_whitelist_count;
}

const struct regop_offset_range *gm20b_get_qctl_whitelist_ranges(void)
{
	return gm20b_qctl_whitelist_ranges;
}

int gm20b_get_qctl_whitelist_ranges_count(void)
{
	return gm20b_qctl_whitelist_ranges_count;
}

void gm20b_init_regops(struct gpu_ops *gops)
{
	gops->regops.get_global_whitelist_ranges =
		gm20b_get_global_whitelist_ranges;
	gops->regops.get_global_whitelist_ranges_count =
		gm20b_get_global_whitelist_ranges_count;

	gops->regops.get_context_whitelist_ranges =
		gm20b_get_context_whitelist_ranges;
	gops->regops.get_context_whitelist_ranges_count =
		gm20b_get_context_whitelist_ranges_count;

	gops->regops.get_runcontrol_whitelist =
		gm20b_get_runcontrol_whitelist;
	gops->regops.get_runcontrol_whitelist_count =
		gm20b_get_runcontrol_whitelist_count;

	gops->regops.get_runcontrol_whitelist_ranges =
		gm20b_get_runcontrol_whitelist_ranges;
	gops->regops.get_runcontrol_whitelist_ranges_count =
		gm20b_get_runcontrol_whitelist_ranges_count;

	gops->regops.get_qctl_whitelist =
		gm20b_get_qctl_whitelist;
	gops->regops.get_qctl_whitelist_count =
		gm20b_get_qctl_whitelist_count;

	gops->regops.get_qctl_whitelist_ranges =
		gm20b_get_qctl_whitelist_ranges;
	gops->regops.get_qctl_whitelist_ranges_count =
		gm20b_get_qctl_whitelist_ranges_count;
}
