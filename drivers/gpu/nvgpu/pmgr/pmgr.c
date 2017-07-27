/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "pwrdev.h"
#include "pmgrpmu.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include "common/linux/os_linux.h"
#endif

int pmgr_pwr_devices_get_power(struct gk20a *g, u32 *val)
{
	struct nv_pmu_pmgr_pwr_devices_query_payload payload;
	int status;

	status = pmgr_pmu_pwr_devices_query_blocking(g, 1, &payload);
	if (status)
		nvgpu_err(g, "pmgr_pwr_devices_get_current_power failed %x",
			status);

	*val = payload.devices[0].powerm_w;

	return status;
}

int pmgr_pwr_devices_get_current(struct gk20a *g, u32 *val)
{
	struct nv_pmu_pmgr_pwr_devices_query_payload payload;
	int status;

	status = pmgr_pmu_pwr_devices_query_blocking(g, 1, &payload);
	if (status)
		nvgpu_err(g, "pmgr_pwr_devices_get_current failed %x",
			status);

	*val = payload.devices[0].currentm_a;

	return status;
}

int pmgr_pwr_devices_get_voltage(struct gk20a *g, u32 *val)
{
	struct nv_pmu_pmgr_pwr_devices_query_payload payload;
	int status;

	status = pmgr_pmu_pwr_devices_query_blocking(g, 1, &payload);
	if (status)
		nvgpu_err(g, "pmgr_pwr_devices_get_current_voltage failed %x",
			status);

	*val = payload.devices[0].voltageu_v;

	return status;
}

#ifdef CONFIG_DEBUG_FS
static int pmgr_pwr_devices_get_power_u64(void *data, u64 *p)
{
	struct gk20a *g = (struct gk20a *)data;
	int err;
	u32 val;

	err = pmgr_pwr_devices_get_power(g, &val);
	*p = val;

	return err;
}

static int pmgr_pwr_devices_get_current_u64(void *data, u64 *p)
{
	struct gk20a *g = (struct gk20a *)data;
	int err;
	u32 val;

	err = pmgr_pwr_devices_get_current(g, &val);
	*p = val;

	return err;
}

static int pmgr_pwr_devices_get_voltage_u64(void *data, u64 *p)
{
	struct gk20a *g = (struct gk20a *)data;
	int err;
	u32 val;

	err = pmgr_pwr_devices_get_voltage(g, &val);
	*p = val;

	return err;
}

DEFINE_SIMPLE_ATTRIBUTE(
		pmgr_power_ctrl_fops, pmgr_pwr_devices_get_power_u64, NULL, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(
		pmgr_current_ctrl_fops, pmgr_pwr_devices_get_current_u64, NULL, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(
		pmgr_voltage_ctrl_fops, pmgr_pwr_devices_get_voltage_u64, NULL, "%llu\n");

static void pmgr_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *dbgentry;

	dbgentry = debugfs_create_file(
				"power", S_IRUGO, l->debugfs, g, &pmgr_power_ctrl_fops);
	if (!dbgentry)
		nvgpu_err(g, "debugfs entry create failed for power");

	dbgentry = debugfs_create_file(
				"current", S_IRUGO, l->debugfs, g, &pmgr_current_ctrl_fops);
	if (!dbgentry)
		nvgpu_err(g, "debugfs entry create failed for current");

	dbgentry = debugfs_create_file(
				"voltage", S_IRUGO, l->debugfs, g, &pmgr_voltage_ctrl_fops);
	if (!dbgentry)
		nvgpu_err(g, "debugfs entry create failed for voltage");
}
#endif

u32 pmgr_domain_sw_setup(struct gk20a *g)
{
	u32 status;

	status = pmgr_device_sw_setup(g);
	if (status) {
		nvgpu_err(g,
			"error creating boardobjgrp for pmgr devices, status - 0x%x",
			status);
		goto exit;
	}

	status = pmgr_monitor_sw_setup(g);
	if (status) {
		nvgpu_err(g,
			"error creating boardobjgrp for pmgr monitor, status - 0x%x",
			status);
		goto exit;
	}

	status = pmgr_policy_sw_setup(g);
	if (status) {
		nvgpu_err(g,
			"error creating boardobjgrp for pmgr policy, status - 0x%x",
			status);
		goto exit;
	}

#ifdef CONFIG_DEBUG_FS
	pmgr_debugfs_init(g);
#endif

exit:
	return status;
}

u32 pmgr_domain_pmu_setup(struct gk20a *g)
{
	return pmgr_send_pmgr_tables_to_pmu(g);
}
