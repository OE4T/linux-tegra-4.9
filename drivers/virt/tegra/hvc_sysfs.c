/*
 * drivers/virt/tegra/hvc_sysfs.c
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
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
#include <soc/tegra/chip-id.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/mm.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <soc/tegra/virt/syscalls.h>

#define TEGRA_HV_ERR(...) pr_err("hvc_sysfs: " __VA_ARGS__)
#define TEGRA_HV_INFO(...) pr_info("hvc_sysfs: " __VA_ARGS__)

#define SYSFS_OFFSET_RELAY_NUM_CHANNELS 0U
#define SYSFS_OFFSET_RELAY_LOGMSG_SIZE  4U

/*
 * This file implements a hypervisor control driver that can be accessed
 * from user-space via the sysfs interface. Currently, the only supported
 * use case is retrieval of the HV trace log when it is available.
 */

struct uart_relay_info_t {
	uint64_t num_channels;
	uint64_t max_msg_size;
};

static struct uart_relay_info_t uart_relay_info;

struct hyp_shared_memory_info {
	const char *node_name;
	struct bin_attribute attr;
	uint64_t ipa;
	unsigned long size;
	ssize_t (*read)(struct file *, struct kobject *, struct bin_attribute *,
			char *, loff_t, size_t);
};

enum HYP_SHM_ID {
	HYP_SHM_ID_LOG,
	HYP_SHM_ID_PCT,
	HYP_SHM_ID_UART_RELAY,

	HYP_SHM_ID_NUM
};

struct hyp_shared_memory_info hyp_shared_memory_attrs[HYP_SHM_ID_NUM];

static ssize_t uart_relay_read(struct file *filp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	uint32_t *dest = (uint32_t *)buf;
	struct uart_relay_info_t *src =
			(struct uart_relay_info_t *)attr->private;

	if (off == SYSFS_OFFSET_RELAY_NUM_CHANNELS)
		*dest = (uint32_t)src->num_channels;
	else if (off == SYSFS_OFFSET_RELAY_LOGMSG_SIZE)
		*dest = (uint32_t)src->max_msg_size;
	else
		*dest = 0U;

	return sizeof(uint32_t);
}

/* Map the HV trace buffer to the calling user process */
static int hvc_sysfs_mmap(struct file *fp, struct kobject *ko,
	struct bin_attribute *attr, struct vm_area_struct *vma)
{
	struct hyp_shared_memory_info *hyp_shm_info =
		container_of(attr, struct hyp_shared_memory_info, attr);

	if ((hyp_shm_info->ipa == 0) || (hyp_shm_info->size == 0))
		return -EINVAL;


	if ((vma->vm_end - vma->vm_start) != attr->size)
		return -EINVAL;

	return remap_pfn_range(
		vma,
		vma->vm_start,
		hyp_shm_info->ipa >> PAGE_SHIFT,
		hyp_shm_info->size,
		vma->vm_page_prot);
}

/* Discover availability and placement of the HV trace buffer */
static int hvc_create_sysfs(
	struct kobject *kobj,
	struct hyp_shared_memory_info *hyp_shm_info)
{
	sysfs_bin_attr_init((struct bin_attribute *)&hyp_shm_info->attr);

	hyp_shm_info->attr.attr.name = hyp_shm_info->node_name;
	hyp_shm_info->attr.attr.mode = S_IRUSR | S_IRGRP | S_IROTH;
	hyp_shm_info->attr.mmap = hvc_sysfs_mmap;
	hyp_shm_info->attr.size = (size_t)hyp_shm_info->size;

	if (hyp_shm_info->read != NULL)
		hyp_shm_info->attr.read = hyp_shm_info->read;

	if ((hyp_shm_info->ipa == 0) || (hyp_shm_info->size == 0))
		return -EINVAL;

	return sysfs_create_bin_file(kobj, &hyp_shm_info->attr);
}

/* Set up all relevant hypervisor control nodes */
static int __init hvc_sysfs_register(void)
{
	struct kobject *kobj;
	int ret;
	uint64_t ipa;
	uint64_t size;
	struct hyp_info_page *info;

	if (is_tegra_hypervisor_mode() == false) {
		TEGRA_HV_INFO("hypervisor is not present\n");
		return -EPERM;
	}

	kobj = kobject_create_and_add("hvc", NULL);
	if (kobj == NULL) {
		TEGRA_HV_INFO("failed to add kobject\n");
		return -ENOMEM;
	}

	if (hyp_read_hyp_info(&ipa) != 0)
		goto probe_uart_relay;

	info = (struct hyp_info_page *)ioremap(ipa, sizeof(*info));
	if (info == NULL)
		goto probe_uart_relay;

	hyp_shared_memory_attrs[HYP_SHM_ID_LOG].ipa = info->log_ipa;
	hyp_shared_memory_attrs[HYP_SHM_ID_LOG].size = (size_t)info->log_size;
	hyp_shared_memory_attrs[HYP_SHM_ID_LOG].node_name = "log";

	ret = hvc_create_sysfs(kobj, &hyp_shared_memory_attrs[HYP_SHM_ID_LOG]);
	if (ret == 0)
		TEGRA_HV_INFO("log is available\n");
	else
		TEGRA_HV_INFO("log is unavailable\n");

	hyp_shared_memory_attrs[HYP_SHM_ID_PCT].ipa = info->pct_ipa;
	hyp_shared_memory_attrs[HYP_SHM_ID_PCT].size = (size_t)info->pct_size;
	hyp_shared_memory_attrs[HYP_SHM_ID_PCT].node_name = "pct";

	ret = hvc_create_sysfs(kobj, &hyp_shared_memory_attrs[HYP_SHM_ID_PCT]);
	if (ret == 0)
		TEGRA_HV_INFO("pct is available\n");
	else
		TEGRA_HV_INFO("pct is unavailable\n");

	iounmap(info);

	/* Probe if Uart relay is available */
probe_uart_relay:
	if (hyp_read_uart_relay_info(&ipa, &size, &uart_relay_info.num_channels,
					&uart_relay_info.max_msg_size) != 0) {
		TEGRA_HV_INFO("uart_relay: Hypercall failed!\n");
		goto out;
	}

	hyp_shared_memory_attrs[HYP_SHM_ID_UART_RELAY].ipa = (uint64_t) ipa;
	hyp_shared_memory_attrs[HYP_SHM_ID_UART_RELAY].size = size;
	hyp_shared_memory_attrs[HYP_SHM_ID_UART_RELAY].node_name = "uart_relay";
	hyp_shared_memory_attrs[HYP_SHM_ID_UART_RELAY].attr.private =
		(void *)&uart_relay_info;
	hyp_shared_memory_attrs[HYP_SHM_ID_UART_RELAY].read = uart_relay_read;

	ret = hvc_create_sysfs(kobj,
		&hyp_shared_memory_attrs[HYP_SHM_ID_UART_RELAY]);
	if (ret == 0)
		TEGRA_HV_INFO("uart_relay is available.\n");
	else
		TEGRA_HV_INFO("uart_relay is unavailable.\n");

out:
	return 0;
}

late_initcall(hvc_sysfs_register);
