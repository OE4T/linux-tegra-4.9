/*
 * include/linux/nvhost.h
 *
 * Tegra graphics host driver
 *
 * Copyright (c) 2009-2013, NVIDIA Corporation. All rights reserved.
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
#include <linux/platform_device.h>

struct nvhost_master;
struct nvhost_hwctx;
struct nvhost_device_power_attr;
struct mem_mgr;

#define NVHOST_MODULE_MAX_CLOCKS		3
#define NVHOST_MODULE_MAX_POWERGATE_IDS 	2
#define NVHOST_MODULE_MAX_SYNCPTS		8
#define NVHOST_MODULE_MAX_WAITBASES		3
#define NVHOST_MODULE_MAX_MODMUTEXES		5
#define NVHOST_MODULE_MAX_IORESOURCE_MEM	3
#define NVHOST_MODULE_NO_POWERGATE_IDS		.powergate_ids = {-1, -1}
#define NVHOST_DEFAULT_CLOCKGATE_DELAY		.clockgate_delay = 25
#define NVHOST_NAME_SIZE			24
#define NVSYNCPT_INVALID			(-1)

#define NVSYNCPT_GRAPHICS_HOST		(0)	/* t20, t30, t114, t148 */
#define NVSYNCPT_DISP0_D		(5)	/* t20, t30, t114, t148 */
#define NVSYNCPT_DISP0_H		(6)	/* t20, t30, t114, t148 */
#define NVSYNCPT_DISP1_H		(7)	/* t20, t30, t114, t148 */
#define NVSYNCPT_DISP0_A		(8)	/* t20, t30, t114, t148 */
#define NVSYNCPT_DISP1_A		(9)	/* t20, t30, t114, t148 */
#define NVSYNCPT_AVP_0			(10)	/* t20, t30, t114, t148 */
#define NVSYNCPT_CSI_VI_0		(11)	/* t20, t30, t114, t148 */
#define NVSYNCPT_CSI_VI_1		(12)	/* t20, t30, t114, t148 */
#define NVSYNCPT_VI_ISP_0		(13)	/* t20, t30, t114, t148 */
#define NVSYNCPT_VI_ISP_1		(14)	/* t20, t30, t114, t148 */
#define NVSYNCPT_VI_ISP_2		(15)	/* t20, t30, t114, t148 */
#define NVSYNCPT_VI_ISP_3		(16)	/* t20, t30, t114, t148 */
#define NVSYNCPT_VI_ISP_4		(17)	/* t20, t30, t114, t148 */
#define NVSYNCPT_2D_0			(18)	/* t20, t30, t114, t148 */
#define NVSYNCPT_2D_1			(19)	/* t20, t30, t114, t148 */
#define NVSYNCPT_DISP0_B		(20)	/* t20, t30, t114, t148 */
#define NVSYNCPT_DISP1_B		(21)	/* t20, t30, t114, t148 */
#define NVSYNCPT_3D			(22)	/* t20, t30, t114, t148 */
#define NVSYNCPT_MPE			(23)	/* t20, t30 */
#define NVSYNCPT_MSENC			(23)	/* t114, t148 */
#define NVSYNCPT_DISP0_C		(24)	/* t20, t30, t114, t148 */
#define NVSYNCPT_DISP1_C		(25)	/* t20, t30, t114, t148 */
#define NVSYNCPT_VBLANK0		(26)	/* t20, t30, t114, t148 */
#define NVSYNCPT_VBLANK1		(27)	/* t20, t30, t114, t148 */
#define NVSYNCPT_MPE_EBM_EOF		(28)	/* t20, t30 */
#define NVSYNCPT_TSEC			(28)	/* t114, t148 */
#define NVSYNCPT_MPE_WR_SAFE		(29)	/* t20, t30 */
#define NVSYNCPT_DSI			(31)	/* t20, t30, t114, t148 */

#define NVWAITBASE_2D_0			(1)	/* t20, t30, t114 */
#define NVWAITBASE_2D_1			(2)	/* t20, t30, t114 */
#define NVWAITBASE_3D			(3)	/* t20, t30, t114 */
#define NVWAITBASE_MPE			(4)	/* t20, t30 */
#define NVWAITBASE_MSENC		(4)	/* t114, t148 */
#define NVWAITBASE_TSEC			(5)	/* t114, t148 */

#define NVMODMUTEX_2D_FULL		(1)	/* t20, t30, t114, t148 */
#define NVMODMUTEX_2D_SIMPLE		(2)	/* t20, t30, t114, t148 */
#define NVMODMUTEX_2D_SB_A		(3)	/* t20, t30, t114, t148 */
#define NVMODMUTEX_2D_SB_B		(4)	/* t20, t30, t114, t148 */
#define NVMODMUTEX_3D			(5)	/* t20, t30, t114, t148 */
#define NVMODMUTEX_DISPLAYA		(6)	/* t20, t30, t114, t148 */
#define NVMODMUTEX_DISPLAYB		(7)	/* t20, t30, t114, t148 */
#define NVMODMUTEX_VI			(8)	/* t20, t30, t114 */
#define NVMODMUTEX_VI_0			(8)	/* t148 */
#define NVMODMUTEX_DSI			(9)	/* t20, t30, t114, t148 */
#define NVMODMUTEX_VI_1			(10)	/* t148 */

/* sync points that are wholly managed by the client */
#define NVSYNCPTS_CLIENT_MANAGED ( \
	BIT(NVSYNCPT_DISP0_A) | BIT(NVSYNCPT_DISP1_A) | \
	BIT(NVSYNCPT_DISP0_B) | BIT(NVSYNCPT_DISP1_B) | \
	BIT(NVSYNCPT_DISP0_C) | BIT(NVSYNCPT_DISP1_C) | \
	BIT(NVSYNCPT_DSI) | \
	BIT(NVSYNCPT_VBLANK0) | BIT(NVSYNCPT_VBLANK1) | \
	BIT(NVSYNCPT_CSI_VI_0) | BIT(NVSYNCPT_CSI_VI_1) | \
	BIT(NVSYNCPT_VI_ISP_1) | BIT(NVSYNCPT_VI_ISP_2) | \
	BIT(NVSYNCPT_VI_ISP_3) | BIT(NVSYNCPT_VI_ISP_4) | \
	BIT(NVSYNCPT_MPE_EBM_EOF) | BIT(NVSYNCPT_MPE_WR_SAFE) | \
	BIT(NVSYNCPT_2D_1) | BIT(NVSYNCPT_AVP_0))

enum nvhost_power_sysfs_attributes {
	NVHOST_POWER_SYSFS_ATTRIB_CLOCKGATE_DELAY = 0,
	NVHOST_POWER_SYSFS_ATTRIB_POWERGATE_DELAY,
	NVHOST_POWER_SYSFS_ATTRIB_REFCOUNT,
	NVHOST_POWER_SYSFS_ATTRIB_MAX
};

struct nvhost_clock {
	char *name;
	unsigned long default_rate;
	u32 moduleid;
	int reset;
	unsigned long devfreq_rate;
};

enum nvhost_device_powerstate_t {
	NVHOST_POWER_STATE_DEINIT,
	NVHOST_POWER_STATE_RUNNING,
	NVHOST_POWER_STATE_CLOCKGATED,
	NVHOST_POWER_STATE_POWERGATED
};

struct nvhost_device_data {
	int		version;	/* ip version number of device */
	int		id;		/* Separates clients of same hw */
	int		index;		/* Hardware channel number */
	void __iomem	*aperture[NVHOST_MODULE_MAX_IORESOURCE_MEM];

	u32		syncpts[NVHOST_MODULE_MAX_SYNCPTS];
	u32		waitbases[NVHOST_MODULE_MAX_WAITBASES];
	u32		modulemutexes[NVHOST_MODULE_MAX_MODMUTEXES];
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

	void *private_data;		/* private platform data */
	struct platform_device *pdev;	/* owner platform_device */

	/* Finalize power on. Can be used for context restore. */
	void (*finalize_poweron)(struct platform_device *dev);

	/* Device is busy. */
	void (*busy)(struct platform_device *);

	/* Device is idle. */
	void (*idle)(struct platform_device *);

	/* Device is going to be suspended */
	void (*suspend_ndev)(struct platform_device *);

	/* Scaling init is run on device registration */
	void (*scaling_init)(struct platform_device *dev);

	/* Scaling deinit is called on device unregistration */
	void (*scaling_deinit)(struct platform_device *dev);

	/* Device is initialized */
	void (*init)(struct platform_device *dev);

	/* Device is de-initialized. */
	void (*deinit)(struct platform_device *dev);

	/* Preparing for power off. Used for context save. */
	int (*prepare_poweroff)(struct platform_device *dev);

	/* Allocates a context handler for the device */
	struct nvhost_hwctx_handler *(*alloc_hwctx_handler)(u32 syncpt,
			u32 waitbase, struct nvhost_channel *ch);

	/* Clock gating callbacks */
	int (*prepare_clockoff)(struct platform_device *dev);
	void (*finalize_clockon)(struct platform_device *dev);

	/* Read module register into memory */
	int (*read_reg)(struct platform_device *dev,
			struct nvhost_channel *ch,
			struct nvhost_hwctx *hwctx,
			u32 offset,
			u32 *value);
};

struct nvhost_devfreq_ext_stat {
	int		busy;
	unsigned long	max_freq;
	unsigned long	min_freq;
};

struct nvhost_device_power_attr {
	struct platform_device *ndev;
	struct kobj_attribute power_attr[NVHOST_POWER_SYSFS_ATTRIB_MAX];
};

void nvhost_device_writel(struct platform_device *dev, u32 r, u32 v);
u32 nvhost_device_readl(struct platform_device *dev, u32 r);

/* public host1x power management APIs */
bool nvhost_module_powered_ext(struct platform_device *dev);
void nvhost_module_busy_ext(struct platform_device *dev);
void nvhost_module_idle_ext(struct platform_device *dev);

/* public host1x sync-point management APIs */
u32 nvhost_syncpt_incr_max_ext(struct platform_device *dev, u32 id, u32 incrs);
void nvhost_syncpt_cpu_incr_ext(struct platform_device *dev, u32 id);
u32 nvhost_syncpt_read_ext(struct platform_device *dev, u32 id);
int nvhost_syncpt_wait_timeout_ext(struct platform_device *dev, u32 id, u32 thresh,
	u32 timeout, u32 *value);

void nvhost_scale3d_set_throughput_hint(int hint);

#endif
