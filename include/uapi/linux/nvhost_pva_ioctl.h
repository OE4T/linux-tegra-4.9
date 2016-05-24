/*
 * Tegra PVA Driver ioctls
 *
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
 *
 * You should have received a copy of the GNU General Public License along with
 * this program;  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_NVHOST_PVA_IOCTL_H
#define __LINUX_NVHOST_PVA_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if !defined(__KERNEL__)
#define __user
#endif

#define NVHOST_PVA_IOCTL_MAGIC 'P'

/**
 * struct pva_characteristics_req - Request filling of characteristics struct
 *
 * @characteristics: pointer to be filled with characteristics
 * @characteristics_size: size in bytes
 * @characteristics_filled: reserved(set to zero)
 *
 */
struct pva_characteristics_req {
	__u64 characteristics;
	__u64 characteristics_size;
	__u64 characteristics_filled;
};

/**
 * struct pva_characteristics - the information of the pva cluster
 *
 * @num_vpu: number of vpu per pva
 * @vpu_generation: vpu hardware generation
 * @task_structure_version: highest supported task struct ver
 * @reserved: reserved for future use
 * @r5_ucode_version: R5 firmware version
 * @r5_ucode_earliest: 1st version compatible with current running fw
 * @r5_vpu_runtime_earliest: First supported vpu runtime version
 *
 */
struct pva_characteristics {
	__u8 num_vpu;
	__u8 vpu_generation;
	__u8 reserved[6];
	__u32 r5_ucode_version;
	__u32 r5_ucode_earliest;
	__u32 r5_vpu_runtime_earliest;
};


#define PVA_IOCTL_CHARACTERISTICS	\
	_IOWR(NVHOST_PVA_IOCTL_MAGIC, 1, struct pva_characteristics_req)


#define NVHOST_PVA_IOCTL_LAST _IOC_NR(PVA_IOCTL_CHARACTERISTICS)
#define NVHOST_PVA_IOCTL_MAX_ARG_SIZE sizeof(struct pva_characteristics_req)

#endif /* __LINUX_NVHOST_PVA_IOCTL_H */

