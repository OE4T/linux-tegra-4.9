/*
 * include/linux/nvhost.h
 *
 * Tegra graphics host driver
 *
 * Copyright (c) 2009-2012, NVIDIA Corporation.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __LINUX_NVHOST_H
#define __LINUX_NVHOST_H

#include <linux/device.h>
#include <linux/types.h>

struct nvhost_master;

#define NVHOST_MODULE_MAX_CLOCKS 3
#define NVHOST_MODULE_MAX_POWERGATE_IDS 2
#define NVHOST_MODULE_NO_POWERGATE_IDS .powergate_ids = {-1, -1}
#define NVHOST_DEFAULT_CLOCKGATE_DELAY .clockgate_delay = 25

struct nvhost_clock {
	char *name;
	long default_rate;
};

enum nvhost_device_powerstate_t {
	NVHOST_POWER_STATE_DEINIT,
	NVHOST_POWER_STATE_RUNNING,
	NVHOST_POWER_STATE_CLOCKGATED,
	NVHOST_POWER_STATE_POWERGATED
};

struct nvhost_device {
	const char	*name;		/* Device name */
	struct device	dev;		/* Linux device struct */
	int		id;		/* Separates clients of same hw */
	u32		num_resources;	/* Number of resources following */
	struct resource	*resource;	/* Resources (IOMEM in particular) */
	struct resource	*reg_mem;
	void __iomem	*aperture;	/* Iomem mapped to kernel */

	u32		syncpts;	/* Bitfield of sync points used */
	u32		waitbases;	/* Bit field of wait bases */
	u32		modulemutexes;	/* Bit field of module mutexes */
	u32		moduleid;	/* Module id for user space API */

	u32		class;		/* Device class */
	bool		exclusive;	/* True if only one user at a time */
	bool		keepalive;	/* Do not power gate when opened */
	bool		waitbasesync;	/* Force sync of wait bases */

	int		powergate_ids[NVHOST_MODULE_MAX_POWERGATE_IDS];
	bool		can_powergate;	/* True if module can be power gated */
	int		clockgate_delay;/* Delay before clock gated */
	int		powergate_delay;/* Delay before power gated */
	struct nvhost_clock clocks[NVHOST_MODULE_MAX_CLOCKS];/* Clock names */

	struct delayed_work powerstate_down;/* Power state management */
	int		num_clks;	/* Number of clocks opened for dev */
	struct clk	*clk[NVHOST_MODULE_MAX_CLOCKS];
	struct mutex	lock;		/* Power management lock */
	int		powerstate;	/* Current power state */
	int		refcount;	/* Number of tasks active */
	wait_queue_head_t idle_wq;	/* Work queue for idle */
	struct list_head client_list;	/* List of clients and rate requests */

	struct nvhost_channel *channel;	/* Channel assigned for the module */

	/* Preparing for power off. Used for context save. */
	int (*prepare_poweroff)(struct nvhost_device *dev);
	/* Finalize power on. Can be used for context restore. */
	void (*finalize_poweron)(struct nvhost_device *dev);
	/* Device is busy. */
	void (*busy)(struct nvhost_device *);
	/* Device is idle. */
	void (*idle)(struct nvhost_device *);
	/* Device is going to be suspended */
	void (*suspend)(struct nvhost_device *);
	/* Device is initialized */
	void (*init)(struct nvhost_device *dev);
	/* Device is de-initialized. */
	void (*deinit)(struct nvhost_device *dev);
};

/* Register device to nvhost bus */
extern int nvhost_device_register(struct nvhost_device *);
/* Deregister device from nvhost bus */
extern void nvhost_device_unregister(struct nvhost_device *);

extern struct bus_type nvhost_bus_type;

struct nvhost_driver {
	int (*probe)(struct nvhost_device *);
	int (*remove)(struct nvhost_device *);
	void (*shutdown)(struct nvhost_device *);
	int (*suspend)(struct nvhost_device *, pm_message_t state);
	int (*resume)(struct nvhost_device *);
	struct device_driver driver;
};

extern int nvhost_driver_register(struct nvhost_driver *);
extern void nvhost_driver_unregister(struct nvhost_driver *);
extern struct resource *nvhost_get_resource(struct nvhost_device *,
		unsigned int, unsigned int);
extern int nvhost_get_irq(struct nvhost_device *, unsigned int);
extern struct resource *nvhost_get_resource_byname(struct nvhost_device *,
		unsigned int, const char *);
extern int nvhost_get_irq_byname(struct nvhost_device *, const char *);

#define to_nvhost_device(x) container_of((x), struct nvhost_device, dev)
#define to_nvhost_driver(drv)	(container_of((drv), struct nvhost_driver, \
				 driver))

#define nvhost_get_drvdata(_dev) dev_get_drvdata(&(_dev)->dev)
#define nvhost_set_drvdata(_dev, data) dev_set_drvdata(&(_dev)->dev, (data))
#define nvhost_get_host(_dev) ((struct nvhost_master *) \
		dev_get_drvdata((_dev)->dev.parent))

int nvhost_bus_add_host(struct nvhost_master *host);

#endif
