/*
 * Nvhost CV cluster PM
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_CV_CLUSTER_PM_H__
#define __NVHOST_CV_CLUSTER_PM_H__

struct platform_device;

/* Must be called BEFORE CV modules power on (reset asserted) */
void cv_cluster_unclamp(struct platform_device *pdev);

/* Must be called AFTER CV modules power down */
void cv_cluster_clamp(struct platform_device *pdev);

#endif
