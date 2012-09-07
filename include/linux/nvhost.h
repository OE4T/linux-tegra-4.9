/*
 * include/linux/nvhost.h
 *
 * Tegra graphics host driver
 *
 * Copyright (c) 2009-2012, NVIDIA Corporation. All rights reserved.
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
#include <linux/devfreq.h>

struct nvhost_master;
struct nvhost_hwctx;
struct nvhost_device_power_attr;

#define NVHOST_MODULE_MAX_CLOCKS		3
#define NVHOST_MODULE_MAX_POWERGATE_IDS 	2
#define NVHOST_MODULE_NO_POWERGATE_IDS		.powergate_ids = {-1, -1}
#define NVHOST_DEFAULT_CLOCKGATE_DELAY		.clockgate_delay = 25
#define NVHOST_NAME_SIZE			24
#define NVSYNCPT_INVALID			(-1)

/* FIXME:
 * Sync point ids are now split into 2 files.
 * 1 if this one and other is in
 * drivers/video/tegra/host/host1x/host1x_syncpt.h
 * So if someone decides to add new sync point in future
 * please check both the header files
 */
#define NVSYNCPT_DISP0_D		(5)
#define NVSYNCPT_DISP0_H		(6)
#define NVSYNCPT_DISP1_H		(7)
#define NVSYNCPT_DISP0_A		(8)
#define NVSYNCPT_DISP1_A		(9)
#define NVSYNCPT_AVP_0			(10)
#define NVSYNCPT_DISP0_B		(20)
#define NVSYNCPT_DISP1_B		(21)
#define NVSYNCPT_DISP0_C		(24)
#define NVSYNCPT_DISP1_C		(25)
#define NVSYNCPT_VBLANK0		(26)
#define NVSYNCPT_VBLANK1		(27)
#define NVSYNCPT_DSI			(31)

enum nvhost_power_sysfs_attributes {
	NVHOST_POWER_SYSFS_ATTRIB_CLOCKGATE_DELAY = 0,
	NVHOST_POWER_SYSFS_ATTRIB_POWERGATE_DELAY,
	NVHOST_POWER_SYSFS_ATTRIB_REFCOUNT,
	NVHOST_POWER_SYSFS_ATTRIB_MAX
};

struct nvhost_device_id {
	char name[NVHOST_NAME_SIZE];
	unsigned long version;
};

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
	const char	*name;		/* device name */
	int		version;	/* ip version number of device */
	struct device	dev;		/* Linux device struct */
	int		id;		/* Separates clients of same hw */
	int		index;		/* Hardware channel number */
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
	bool		powerup_reset;	/* Do a reset after power un-gating */
	bool		serialize;	/* Serialize submits in the channel */

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
	struct kobject *power_kobj;	/* kobject to hold power sysfs entries */
	struct nvhost_device_power_attr *power_attrib;	/* sysfs attributes */
	struct devfreq	*power_manager;	/* Device power management */
	struct dentry *debugfs;		/* debugfs directory */
};

struct nvhost_device_power_attr {
	struct nvhost_device *ndev;
	struct kobj_attribute power_attr[NVHOST_POWER_SYSFS_ATTRIB_MAX];
};

struct nvhost_devfreq_ext_stat {
	int		busy;
	unsigned long	max_freq;
	unsigned long	min_freq;
};


/* Register devices to nvhost bus */
extern int nvhost_add_devices(struct nvhost_device **, int num);

/* Register device to nvhost bus */
extern int nvhost_device_register(struct nvhost_device *);

/* Deregister device from nvhost bus */
extern void nvhost_device_unregister(struct nvhost_device *);

extern struct bus_type nvhost_bus_type;

struct nvhost_driver {
	int (*probe)(struct nvhost_device *, struct nvhost_device_id *);
	int (*remove)(struct nvhost_device *);
	void (*shutdown)(struct nvhost_device *);
	int (*suspend)(struct nvhost_device *, pm_message_t state);
	int (*resume)(struct nvhost_device *);
	struct device_driver driver;

	struct nvhost_device_id *id_table;

	/* Finalize power on. Can be used for context restore. */
	void (*finalize_poweron)(struct nvhost_device *dev);

	/* Device is busy. */
	void (*busy)(struct nvhost_device *);

	/* Device is idle. */
	void (*idle)(struct nvhost_device *);

	/* Device is going to be suspended */
	void (*suspend_ndev)(struct nvhost_device *);

	/* Device is initialized */
	void (*init)(struct nvhost_device *dev);

	/* Device is de-initialized. */
	void (*deinit)(struct nvhost_device *dev);

	/* Preparing for power off. Used for context save. */
	int (*prepare_poweroff)(struct nvhost_device *dev);

	/* Allocates a context handler for the device */
	struct nvhost_hwctx_handler *(*alloc_hwctx_handler)(u32 syncpt,
			u32 waitbase, struct nvhost_channel *ch);

	/* Clock gating callbacks */
	int (*prepare_clockoff)(struct nvhost_device *dev);
	void (*finalize_clockon)(struct nvhost_device *dev);

	/* Read module register into memory */
	int (*read_reg)(struct nvhost_device *dev,
			struct nvhost_channel *ch,
			struct nvhost_hwctx *hwctx,
			u32 offset,
			u32 *value);
};

extern int nvhost_driver_register(struct nvhost_driver *);
extern void nvhost_driver_unregister(struct nvhost_driver *);
extern struct resource *nvhost_get_resource(struct nvhost_device *,
		unsigned int, unsigned int);
extern int nvhost_get_irq(struct nvhost_device *, unsigned int);
extern struct resource *nvhost_get_resource_byname(struct nvhost_device *,
		unsigned int, const char *);
extern int nvhost_get_irq_byname(struct nvhost_device *, const char *);
extern void nvhost_device_writel(struct nvhost_device *, u32 r, u32 v);
extern u32 nvhost_device_readl(struct nvhost_device *, u32 r);

#define to_nvhost_device(x)	container_of((x), struct nvhost_device, dev)
#define to_nvhost_driver(drv)	(container_of((drv), struct nvhost_driver, \
				 driver))

#define nvhost_get_drvdata(_dev)	dev_get_drvdata(&(_dev)->dev)
#define nvhost_set_drvdata(_dev, data)	dev_set_drvdata(&(_dev)->dev, (data))

int nvhost_bus_add_host(struct nvhost_master *host);

static inline struct nvhost_device *nvhost_get_parent(struct nvhost_device *_dev)
{
	return _dev->dev.parent ? to_nvhost_device(_dev->dev.parent) : NULL;
}

/* public host1x power management APIs */
bool nvhost_module_powered_ext(struct nvhost_device *dev);
void nvhost_module_busy_ext(struct nvhost_device *dev);
void nvhost_module_idle_ext(struct nvhost_device *dev);

/* public host1x sync-point management APIs */
u32 nvhost_syncpt_incr_max_ext(struct nvhost_device *dev, u32 id, u32 incrs);
void nvhost_syncpt_cpu_incr_ext(struct nvhost_device *dev, u32 id);
u32 nvhost_syncpt_read_ext(struct nvhost_device *dev, u32 id);
int nvhost_syncpt_wait_timeout_ext(struct nvhost_device *dev, u32 id, u32 thresh,
	u32 timeout, u32 *value);

void nvhost_scale3d_set_throughput_hint(int hint);

#endif
