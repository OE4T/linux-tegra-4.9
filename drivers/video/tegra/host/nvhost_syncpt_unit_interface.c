/*
 * Engine side synchronization support
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/nvhost.h>
#include <linux/errno.h>
#include <linux/io.h>

#include "bus_client_t194.h"
#include "nvhost_syncpt_unit_interface.h"

#define SYNCPT_SIZE		0x1000
#define SYNCPT_APERTURE_SIZE	0x400000

struct syncpt_unit_interface {
	dma_addr_t start;
};

/**
 * nvhost_syncpt_gos_supported() - Check if the syncpoint has GoS backing
 *
 * @engine_pdev:	Pointer to a host1x engine
 * @syncpt_id:		Syncpoint id to be checked
 *
 * Return:		True if syncpoint has a GoS backing. False otherwise
 *
 * This function can be used for checking if the syncpoint has GoS backing
 * to determine the required synchronization method (GoS vs syncpoint).
 *
 * TBD: This function returns currently always false to force the user
 * of the API to use the syncpoint<->MSS interface.
 */
bool nvhost_syncpt_gos_supported(struct platform_device *engine_pdev,
				 u32 syncpt_id)
{
	return false;
}

/**
 * nvhost_syncpt_gos_address() - Retrieve semaphore address in engine IOVA
 *
 * @engine_pdev:	Pointer to a host1x engine
 * @syncpt_id:		Syncpoint id to be checked
 * @gos_base:		Pointer for storing the GoS identifier
 * @gos_offset:		Pointer for storing the word offset within GoS
 *
 * Returns:		0 on success, a negative error code otherwise.
 *
 * This function determines the GoS that holds the backing for given
 * syncpoint and the offset within the GoS.
 *
 * TBD: This function is not yet implemented.
 */
int nvhost_syncpt_gos_address(struct platform_device *engine_pdev,
			      u32 syncpt_id,
			      u32 *gos_id,
			      u32 *gos_offset)
{
	return -ENOSYS;
}

/**
 * nvhost_syncpt_unit_interface_get_aperture() - Get syncpoint MSS aperture
 *
 * @host_pdev:	Host1x pdev
 * @base:	Pointer for storing the MMIO base address
 * @size:	Pointer for storing the aperture size
 *
 * Return:	0 on success, a negative error code otherwise
 *
 * This function returns the start and size of the MSS syncpoint aperture.
 * The function can be used in cases where the device is not an nvhost
 * device (e.g. GPU).
 */
int nvhost_syncpt_unit_interface_get_aperture(
					struct platform_device *host_pdev,
					phys_addr_t *base,
					size_t *size)
{
	struct resource *res;

	if (host_pdev == NULL || base == NULL || size == NULL)
		return -ENOSYS;

	res = platform_get_resource(host_pdev, IORESOURCE_MEM,
				    HOST1X_MSS_APERTURE);

	*base = (phys_addr_t)res->start;
	*size = (size_t)res->end - (size_t)res->start + 1;

	return 0;
}
EXPORT_SYMBOL(nvhost_syncpt_unit_interface_get_aperture);

/**
 * nvhost_syncpt_unit_interface_get_byte_offset() - Get syncpoint offset
 *
 * @syncpt_id:	Syncpoint id
 *
 * Return:	Offset to the syncpoint address within the shim
 *
 * This function returns the offset to the syncpoint address from
 * the syncpoint mss aperture base.
 */
u32 nvhost_syncpt_unit_interface_get_byte_offset(u32 syncpt_id)
{
	return syncpt_id * SYNCPT_SIZE;
}
EXPORT_SYMBOL(nvhost_syncpt_unit_interface_get_byte_offset);

/**
 * nvhost_syncpt_address() - Get syncpoint IOVA for a device
 *
 * @engine_pdev:	Engine platform device pointer
 * @syncpt_id:		Syncpoint id
 *
 * Return:		IOVA address to the syncpoint
 *
 * This function returns the IOVA to a syncpoint. It is assumed that the
 * pdev uses nvhost and nvhost_syncpt_unit_interface_init_engine() has been
 * called.
 */
dma_addr_t nvhost_syncpt_address(struct platform_device *engine_pdev, u32 id)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(engine_pdev);
	struct syncpt_unit_interface *syncpt_unit_interface =
			pdata->syncpt_unit_interface;

	return syncpt_unit_interface->start + SYNCPT_SIZE * id;
}

/**
 * nvhost_syncpt_unit_interface() - Initialize engine-side synchronization
 *
 * @engine_pdev:	Engine platform device pointer
 *
 * Return:		0 on success, a negative error code otherwise
 *
 * This function prepares engine to perform synchronization without
 * utilizing Host1x channels to perform syncpoint waits. This includes
 * initialization of the syncpoint<->MSS interface and mapping the GoS
 * for the device if needed.
 *
 * TBD: Currently this function only verifies that IOMMU is disabled. This
 * allows using the physical address of the MSS aperture directly without
 * worrying about the real mappings. In future this function should perform
 * actual mappings.
 */
int nvhost_syncpt_unit_interface_init(struct platform_device *engine_pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(engine_pdev);
	struct syncpt_unit_interface *syncpt_unit_interface;
	struct platform_device *host_pdev;
	dma_addr_t range_start;
	struct resource *res;
	size_t range_size;

	/* Only physical accesses are supported */
	if (engine_pdev->dev.archdata.iommu)
		return -ENOSYS;

	/* Get aperture and initialize range variables assuming physical
	 * addressing
	 */
	host_pdev = to_platform_device(engine_pdev->dev.parent);
	res = platform_get_resource(host_pdev, IORESOURCE_MEM,
				    HOST1X_MSS_APERTURE);
	range_start = (dma_addr_t)res->start;
	range_size = (size_t)res->end - (size_t)res->start + 1;

	/* Allocate space for storing the interface configuration */
	syncpt_unit_interface = devm_kzalloc(&engine_pdev->dev,
					     sizeof(*syncpt_unit_interface),
					     GFP_KERNEL);
	if (syncpt_unit_interface == NULL)
		return -ENOMEM;

	syncpt_unit_interface->start = range_start;
	pdata->syncpt_unit_interface = syncpt_unit_interface;

	return 0;
}
