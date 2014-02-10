/*
 * drivers/video/tegra/host/gk20a/soc/platform_gk20a.h
 *
 * GK20A Platform (SoC) Interface
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _GK20A_PLATFORM_H_
#define _GK20A_PLATFORM_H_

#include <linux/platform_device.h>
#ifdef CONFIG_TEGRA_GK20A
#include <linux/nvhost.h>
#endif

struct gk20a;
struct channel_gk20a;
struct gr_ctx_buffer_desc;
struct gk20a_scale_profile;

struct gk20a_platform {
#ifdef CONFIG_TEGRA_GK20A
	/* We need to have nvhost_device_data at the beginning, because
	 * nvhost assumes that it owns the platform_data. We can store
	 * gk20a platform info after that though. */
	struct nvhost_device_data nvhost;
#endif
	/* Populated by the gk20a driver before probing the platform. */
	struct gk20a *g;

	/* Should be populated at probe. */
	bool can_railgate;

	/* Should be populated at probe. */
	bool has_syncpoints;

	/* Should be populated by probe. */
	struct dentry *debugfs;

	/* Clock configuration is stored here. Platform probe is responsible
	 * for filling this data. */
	struct clk *clk[3];
	int num_clks;

	/* Delay before rail gated */
	int railgate_delay;

	/* Delay before clock gated */
	int clockgate_delay;

	/* Initialize the platform interface of the gk20a driver.
	 *
	 * The platform implementation of this function must
	 *   - set the power and clocks of the gk20a device to a known
	 *     state, and
	 *   - populate the gk20a_platform structure (a pointer to the
	 *     structure can be obtained by calling gk20a_get_platform).
	 *
	 * After this function is finished, the driver will initialise
	 * pm runtime and genpd based on the platform configuration.
	 */
	int (*probe)(struct platform_device *dev);

	/* Second stage initialisation - called once all power management
	 * initialisations are done.
	 */
	int (*late_probe)(struct platform_device *dev);

	/* Called before submitting work to the gpu. The platform may use this
	 * hook to ensure that any other hw modules that the gpu depends on are
	 * powered. The platform implementation must count refs to this call. */
	int (*channel_busy)(struct platform_device *dev);

	/* Called after the work on the gpu is completed. The platform may use
	 * this hook to release power refs to any other hw modules that the gpu
	 * depends on. The platform implementation must count refs to this
	 * call. */
	void (*channel_idle)(struct platform_device *dev);

	/* This function is called to allocate secure memory (memory that the
	 * CPU cannot see). The function should fill the context buffer
	 * descriptor (especially fields destroy, sgt, size).
	 */
	int (*secure_alloc)(struct platform_device *dev,
			    struct gr_ctx_buffer_desc *desc,
			    size_t size);

	/* Device is going to be suspended */
	int (*suspend)(struct device *);

	/* Called to turn off the device */
	int (*railgate)(struct platform_device *dev);

	/* Called to turn on the device */
	int (*unrailgate)(struct platform_device *dev);

	/* Postscale callback is called after frequency change */
	void (*postscale)(struct platform_device *pdev,
			  unsigned long freq);

	/* Pre callback is called before frequency change */
	void (*prescale)(struct platform_device *pdev);

	/* Devfreq governor name. If scaling is enabled, we request
	 * this governor to be used in scaling */
	const char *devfreq_governor;

	/* Quality of service id. If this is set, the scaling routines
	 * will register a callback to id. Each time we receive a new value,
	 * the postscale callback gets called.  */
	int qos_id;
};

static inline struct gk20a_platform *gk20a_get_platform(
		struct platform_device *dev)
{
	return (struct gk20a_platform *)platform_get_drvdata(dev);
}

extern struct gk20a_platform gk20a_generic_platform;
#ifdef CONFIG_TEGRA_GK20A
extern struct gk20a_platform gk20a_tegra_platform;
#endif

static inline int gk20a_platform_channel_busy(struct platform_device *dev)
{
	struct gk20a_platform *p = gk20a_get_platform(dev);
	int ret = 0;
	if (p->channel_busy)
		ret = p->channel_busy(dev);

	return ret;
}

static inline void gk20a_platform_channel_idle(struct platform_device *dev)
{
	struct gk20a_platform *p = gk20a_get_platform(dev);
	if (p->channel_idle)
		p->channel_idle(dev);
}

static inline bool gk20a_platform_has_syncpoints(struct platform_device *dev)
{
	struct gk20a_platform *p = gk20a_get_platform(dev);
	return p->has_syncpoints;
}

#endif
