/*
 * drivers/video/tegra/host/pva/pva_cluster.h
 *
 * PVA cluster header
 *
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_PVA_CLUSTER_H__
#define __NVHOST_PVA_CLUSTER_H__

struct platform_device;

/**
 * pva_cluster_add() - Add PVA device to PVA Cluster
 *
 * @pdev: Pointer to PVA device
 * Return: 0 on Success or negative error code
 *
 * This function adds a device to PVA cluster.
 */
int pva_cluster_add(struct platform_device *pdev);

/**
 * pva_cluster_remove() - Remove PVA device from PVA Cluster
 *
 * @pdev: Pointer to PVA device
 * Return: 0 on Success or negative error code
 *
 * This function removes a device to PVA cluster.
 */
int pva_cluster_remove(struct platform_device *pdev);

#endif /* __NVHOST_PVA_CLUSTER_H__ */

