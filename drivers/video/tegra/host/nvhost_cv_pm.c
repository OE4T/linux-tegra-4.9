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
#include <linux/mutex.h>
#include "t194/hardware_t194.h"
#include "nvhost_cv_pm.h"
#include "bus_client_t194.h"

/**
 * Tracks cluster lower power transitions. Last module initiates poweroff and
 * first module initiates poweron.
 */
static int ref;

/* Serializes CV cluster PM sequences */
static DEFINE_MUTEX(cv_cluster_lock);

/* Must be called BEFORE CV modules power on (reset asserted) */
void cv_cluster_unclamp(struct platform_device *pdev)
{
	mutex_lock(&cv_cluster_lock);

	/* Unclamp */
	if (!ref)
		host1x_hypervisor_writel(pdev,
			host1x_sync_scr_prot_common_cv_cluster_clamp_0_r(), 0);
	ref++;
	mutex_unlock(&cv_cluster_lock);
}

/* Must be called AFTER CV modules power down */
void cv_cluster_clamp(struct platform_device *pdev)
{
	mutex_lock(&cv_cluster_lock);

	/* Clamp */
	if (ref == 1)
		host1x_hypervisor_writel(pdev,
			host1x_sync_scr_prot_common_cv_cluster_clamp_0_r(), 1);
	WARN_ON(!ref);
	ref--;
	mutex_unlock(&cv_cluster_lock);
}
