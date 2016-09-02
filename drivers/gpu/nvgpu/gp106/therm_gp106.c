/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include "therm_gp106.h"
#include <linux/debugfs.h>
#include "hw_therm_gp106.h"

#ifdef CONFIG_DEBUG_FS
static int therm_get_internal_sensor_curr_temp(void *data, u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	int err = 0;
	u32 readval;

	readval = gk20a_readl(g, therm_temp_sensor_tsense_r());

	if (!(therm_temp_sensor_tsense_state_v(readval) &
		therm_temp_sensor_tsense_state_valid_v())) {
		gk20a_err(dev_from_gk20a(g),
			"Attempt to read temperature while sensor is OFF!\n");
		err = -EINVAL;
	} else if (therm_temp_sensor_tsense_state_v(readval) &
		therm_temp_sensor_tsense_state_shadow_v()) {
		gk20a_err(dev_from_gk20a(g),
			"Reading temperature from SHADOWed sensor!\n");
	}

	// Convert from F9.5 -> F27.5 -> F24.8.
	readval &= therm_temp_sensor_tsense_fixed_point_m();

	*val = readval;

	return err;
}
DEFINE_SIMPLE_ATTRIBUTE(therm_ctrl_fops, therm_get_internal_sensor_curr_temp, NULL, "%llu\n");

static void gp106_therm_debugfs_init(struct gk20a *g) {
	struct gk20a_platform *platform = dev_get_drvdata(g->dev);
	struct dentry *dbgentry;

	dbgentry = debugfs_create_file(
		"temp", S_IRUGO, platform->debugfs, g, &therm_ctrl_fops);
	if (!dbgentry)
		gk20a_err(dev_from_gk20a(g), "debugfs entry create failed for therm_curr_temp");
}
#endif

void gp106_init_therm_ops(struct gpu_ops *gops) {
#ifdef CONFIG_DEBUG_FS
	gops->therm.therm_debugfs_init = gp106_therm_debugfs_init;
#endif
}
