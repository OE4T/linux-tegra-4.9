/*
 * drivers/virt/tegra/hvc_sysfs.c
 *
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
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
 */

#include <linux/errno.h>
#include <linux/tegra-soc.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/mm.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <soc/tegra/virt/syscalls.h>

/*
 * This file implements a hypervisor control driver that can be accessed
 * from user-space via the sysfs interface. Currently, the only supported
 * use case is retrieval of the HV trace log when it is available.
 */

static struct bin_attribute log_attr;
static uint64_t log_ipa;

/* Map the HV trace buffer to the calling user process */
static int hvc_sysfs_mmap(struct file *fp, struct kobject *ko,
	struct bin_attribute *attr, struct vm_area_struct *vma)
{
	if ((log_ipa == 0) || (attr->size == 0)) {
		return -EINVAL;
	}

	if ((vma->vm_end - vma->vm_start) != attr->size) {
		return -EINVAL;
	}

	return remap_pfn_range(vma, vma->vm_start, log_ipa >> PAGE_SHIFT,
		attr->size, vma->vm_page_prot);
}

/* Discover availability and placement of the HV trace buffer */
static int hvc_sysfs_log(struct kobject *kobj)
{
	struct hyp_info_page *info;
	uint64_t ipa;

	if (hyp_read_hyp_info(&ipa) != 0) {
		return -EINVAL;
	}

	sysfs_bin_attr_init(&log_attr);
	log_attr.attr.name = "log";
	log_attr.attr.mode = S_IRUSR | S_IRGRP;
	log_attr.mmap = hvc_sysfs_mmap;

	info = (struct hyp_info_page *)ioremap(ipa, sizeof(*info));
	if (info == NULL) {
		return -EFAULT;
	}

	log_ipa = info->log_ipa;
	log_attr.size = (size_t)info->log_size;

	iounmap(info);

	if ((log_ipa == 0) || (log_attr.size == 0)) {
		return -EINVAL;
	}

	return sysfs_create_bin_file(kobj, &log_attr);
}

/* Set up all relevant hypervisor control nodes */
static int __init hvc_sysfs_register(void)
{
	struct kobject *kobj;
	int ret;

	if (is_tegra_hypervisor_mode() == false) {
		pr_info("hvc_sysfs: hypervisor is not present\n");
		return -EPERM;
	}

	kobj = kobject_create_and_add("hvc", NULL);
	if (kobj == NULL) {
		pr_err("hvc_sysfs: failed to add kobject\n");
		return -ENOMEM;
	}

	ret = hvc_sysfs_log(kobj);
	if (ret == 0) {
		pr_info("hvc_sysfs: log is available\n");
	} else {
		pr_info("hvc_sysfs: log is unavailable\n");
	}

	return 0;
}

late_initcall(hvc_sysfs_register);
