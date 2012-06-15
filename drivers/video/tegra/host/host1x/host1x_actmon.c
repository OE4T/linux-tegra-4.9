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
	writel(0, sync_regs + host1x_sync_actmon_init_avg_r());

	/* Default count weight - 1 for per unit actmons */
	writel(1, sync_regs + host1x_sync_actmon_count_weight_r());

	/*  Write sample period */
	writel(host1x_sync_actmon_status_sample_period_f(1),
			sync_regs + host1x_sync_actmon_status_r());

	/* Clear interrupt status */
	writel(0xffffffff, sync_regs + host1x_sync_actmon_intr_status_r());

	/* Set watermarks - arbitrary for now */
	writel(0x100, sync_regs + host1x_sync_actmon_avg_upper_wmark_r());
	writel(0x50, sync_regs + host1x_sync_actmon_avg_lower_wmark_r());

	val = readl(sync_regs + host1x_sync_actmon_ctrl_r());
	/* Enable periodic mode */
	val |= host1x_sync_actmon_ctrl_enb_periodic_f(1);
	/* Enable watermark interrupts */
	val |= host1x_sync_actmon_ctrl_avg_above_wmark_en_f(1);
	val |= host1x_sync_actmon_ctrl_avg_below_wmark_en_f(1);
	/* Number of upper wmark breaches before interrupt */
	val |= host1x_sync_actmon_ctrl_consecutive_above_wmark_num_f(1);
	/* Number of below wmark breaches before interrupt */
	val |= host1x_sync_actmon_ctrl_consecutive_below_wmark_num_f(1);
	/* Moving avg IIR filter window size 2^6=128 */
	val |= host1x_sync_actmon_ctrl_k_val_f(6);
	/* Enable ACTMON */
	val |= host1x_sync_actmon_ctrl_enb_f(1);
	writel(val, sync_regs + host1x_sync_actmon_ctrl_r());

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
	val = readl(sync_regs + host1x_sync_actmon_ctrl_r());
	val |= host1x_sync_actmon_ctrl_enb_f(0);
	writel(val, sync_regs + host1x_sync_actmon_ctrl_r());

	/* Clear interrupt status */
	writel(0xffffffff, sync_regs + host1x_sync_actmon_intr_status_r());

	host1x_actmon_initialized = 0;
}

int host1x_actmon_avg(struct nvhost_master *host, u32 *val)
{
	void __iomem *sync_regs = host->sync_aperture;

	if (!host1x_actmon_initialized)
		return -ENODEV;

	*val = readl(sync_regs + host1x_sync_actmon_avg_count_r());
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
