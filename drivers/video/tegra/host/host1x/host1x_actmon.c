/*
 * drivers/video/tegra/host/host1x/host1x_actmon.c
 *
 * Tegra Graphics Host Actmon support
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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

#include <linux/nvhost.h>
#include <linux/io.h>
#include "host1x_hardware.h"
#include "dev.h"

/* Set to 1 if actmon has been initialized */
static int host1x_actmon_initialized;
static int above_wmark;
static int below_wmark;

int host1x_actmon_init(struct nvhost_master *host)
{
	void __iomem *sync_regs = host->sync_aperture;
	u32 val;

	if (host1x_actmon_initialized)
		return 0;

	/* Initialize average */
	writel(0, sync_regs + HOST1X_SYNC_ACTMON_INIT_AVG_0);

	/* Default count weight - 1 for per unit actmons */
	writel(1, sync_regs + HOST1X_SYNC_ACTMON_COUNT_WEIGHT_0);

	/*  Write sample period */
	writel(HOST1X_CREATE(SYNC_ACTMON_STATUS, SAMPLE_PERIOD, 1),
			sync_regs + HOST1X_SYNC_ACTMON_STATUS_0);

	/* Clear interrupt status */
	writel(0xffffffff, sync_regs + HOST1X_SYNC_ACTMON_INTR_STATUS_0);

	/* Set watermarks - arbitrary for now */
	writel(0x100, sync_regs + HOST1X_SYNC_ACTMON_AVG_UPPER_WMARK_0);
	writel(0x50, sync_regs + HOST1X_SYNC_ACTMON_AVG_LOWER_WMARK_0);

	val = readl(sync_regs + HOST1X_SYNC_ACTMON_CTRL_0);
	/* Enable periodic mode */
	val |= HOST1X_CREATE(SYNC_ACTMON_CTRL, ENB_PERIODIC, 1);
	/* Enable watermark interrupts */
	val |= HOST1X_CREATE(SYNC_ACTMON_CTRL, AVG_ABOVE_WMARK_EN, 1);
	val |= HOST1X_CREATE(SYNC_ACTMON_CTRL, AVG_BELOW_WMARK_EN, 1);
	/* Number of upper wmark breaches before interrupt */
	val |= HOST1X_CREATE(SYNC_ACTMON_CTRL, CONSECUTIVE_ABOVE_WMARK_NUM, 1);
	/* Number of below wmark breaches before interrupt */
	val |= HOST1X_CREATE(SYNC_ACTMON_CTRL, CONSECUTIVE_BELOW_WMARK_NUM, 1);
	/* Moving avg IIR filter window size 2^6=128 */
	val |= HOST1X_CREATE(SYNC_ACTMON_CTRL, K_VAL, 6);
	/* Enable ACTMON */
	val |= HOST1X_CREATE(SYNC_ACTMON_CTRL, ENB, 1);
	writel(val, sync_regs + HOST1X_SYNC_ACTMON_CTRL_0);

	host1x_actmon_initialized = 1;
	return 0;
}

void host1x_actmon_deinit(struct nvhost_master *host)
{
	void __iomem *sync_regs = host->sync_aperture;
	u32 val;

	if (!host1x_actmon_initialized)
		return;

	/* Disable actmon */
	val = readl(sync_regs + HOST1X_SYNC_ACTMON_CTRL_0);
	val |= HOST1X_CREATE(SYNC_ACTMON_CTRL, ENB, 1);
	writel(val, sync_regs + HOST1X_SYNC_ACTMON_CTRL_0);

	/* Clear interrupt status */
	writel(0xffffffff, sync_regs + HOST1X_SYNC_ACTMON_INTR_STATUS_0);

	host1x_actmon_initialized = 0;
}

int host1x_actmon_avg(struct nvhost_master *host, u32 *val)
{
	void __iomem *sync_regs = host->sync_aperture;

	if (!host1x_actmon_initialized)
		return -ENODEV;

	*val = readl(sync_regs + HOST1X_SYNC_ACTMON_AVG_COUNT_0);
	rmb();

	return 0;
}

void host1x_actmon_intr_above_wmark(void)
{
	above_wmark++;
}

void host1x_actmon_intr_below_wmark(void)
{
	below_wmark++;
}

int host1x_actmon_above_wmark_count(void)
{
	return above_wmark;
}

int host1x_actmon_below_wmark_count(void)
{
	return below_wmark;
}
