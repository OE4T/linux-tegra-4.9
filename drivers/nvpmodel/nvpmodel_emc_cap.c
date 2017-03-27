/*
 * drivers/nvpmodel/nvpmodel_emc_cap.c
 *
 * NVIDIA Tegra Nvpmodel driver for Tegra chips
 *
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/bwmgr_mc.h>

#define AUTHOR "Terry Wang <terwang@nvidia.com>"
#define DESCRIPTION "Nvpmodel EMC cap driver"
#define MODULE_NAME "Nvpmodel_EMC_cap"
#define VERSION "1.0"

/* Module information */
MODULE_AUTHOR(AUTHOR);
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");

static struct kobject *emc_iso_cap_kobject;
static unsigned long emc_iso_cap;

/* bandwidth manager handle */
struct tegra_bwmgr_client *bwmgr_handle;

static ssize_t emc_iso_cap_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", emc_iso_cap);
}

static ssize_t emc_iso_cap_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf,
				size_t count)
{
	int error = 0;
	sscanf(buf, "%lu", &emc_iso_cap);
	error = tegra_bwmgr_set_emc(bwmgr_handle, emc_iso_cap,
				TEGRA_BWMGR_SET_EMC_ISO_CAP);
	if (error)
		pr_warn("Nvpmodel failed to set EMC hz=%lu errno=%d\n",
			emc_iso_cap, error);

	return count;
}

static struct kobj_attribute emc_iso_cap_attribute =
	__ATTR(emc_iso_cap, 0660, emc_iso_cap_show, emc_iso_cap_store);

static int __init nvpmodel_emc_cap_init(void)
{
	int error = 0;

	emc_iso_cap_kobject = kobject_create_and_add("nvpmodel_emc_cap",
						 kernel_kobj);
	if (!emc_iso_cap_kobject)
		return -ENOMEM;
	error = sysfs_create_file(emc_iso_cap_kobject,
				&emc_iso_cap_attribute.attr);
	if (error) {
		pr_err("failed to create emc_iso_cap sysfs: error %d\n", error);
		kobject_put(emc_iso_cap_kobject);
		return error;
	}

	bwmgr_handle = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_NVPMODEL);
	if (IS_ERR_OR_NULL(bwmgr_handle)) {
		error = IS_ERR(bwmgr_handle) ?
			PTR_ERR(bwmgr_handle) : -ENODEV;
		pr_warn("Nvpmodel can't register EMC bwmgr (%d)\n", error);
		goto put_bwmgr;
	}

	pr_info("Module initialized successfully \n");
	return error;

put_bwmgr:
	if (!IS_ERR_OR_NULL(bwmgr_handle))
		tegra_bwmgr_unregister(bwmgr_handle);
	kobject_put(emc_iso_cap_kobject);

	return error;
}

static void __exit nvpmodel_emc_cap_exit(void)
{
	tegra_bwmgr_unregister(bwmgr_handle);
	kobject_put(emc_iso_cap_kobject);
	pr_info("Module exit successfully \n");
}

module_init(nvpmodel_emc_cap_init);
module_exit(nvpmodel_emc_cap_exit);
