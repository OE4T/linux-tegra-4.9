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
#include "dev.h"
#include "chip_support.h"

/* Set to 1 if actmon has been initialized */
static int host1x_actmon_initialized;
static int above_wmark;
static int below_wmark;

static int host1x_actmon_init(struct nvhost_master *host)
{
	void __iomem *sync_regs = host->sync_aperture;
	u32 val;

	if (host1x_actmon_initialized)
		return 0;

	/* Initialize average */
	writel(0, sync_regs + host1x_sync_actmon_init_avg_r());

	/* Default count weight - 1 for per unit actmons */
	writel(1, sync_regs + host1x_sync_actmon_count_weight_r());

	/* Wait for actmon to be disabled */
	do {
		val = readl(sync_regs + host1x_sync_actmon_status_r());
	} while (val & host1x_sync_actmon_status_gr3d_mon_act_f(1));

	/*  Write sample period */
	writel(host1x_sync_actmon_status_sample_period_f(0)
		| host1x_sync_actmon_status_status_source_f(
			host1x_sync_actmon_status_status_source_msec_v()),
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
	/* Moving avg IIR filter window size 2^6=128 */
	val |= host1x_sync_actmon_ctrl_k_val_f(6);
	/* Enable ACTMON */
	val |= host1x_sync_actmon_ctrl_enb_f(1);
	writel(val, sync_regs + host1x_sync_actmon_ctrl_r());

	host1x_actmon_initialized = 1;
	return 0;
}

static void host1x_actmon_deinit(struct nvhost_master *host)
{
	void __iomem *sync_regs = host->sync_aperture;
	u32 val;

	if (!host1x_actmon_initialized)
		return;

	/* Disable actmon */
	val = readl(sync_regs + host1x_sync_actmon_ctrl_r());
	val &= ~host1x_sync_actmon_ctrl_enb_m();
	val &= ~host1x_sync_actmon_ctrl_enb_periodic_m();
	val &= ~host1x_sync_actmon_ctrl_avg_above_wmark_en_m();
	val &= ~host1x_sync_actmon_ctrl_avg_below_wmark_en_m();
	writel(val, sync_regs + host1x_sync_actmon_ctrl_r());

	/*  Write sample period */
	writel(host1x_sync_actmon_status_sample_period_f(0)
		| host1x_sync_actmon_status_status_source_f(
			host1x_sync_actmon_status_status_source_usec_v()),
			sync_regs + host1x_sync_actmon_status_r());
	/* Clear interrupt status */
	writel(0xffffffff, sync_regs + host1x_sync_actmon_intr_status_r());

	host1x_actmon_initialized = 0;
}

static int host1x_actmon_avg(struct nvhost_master *host, u32 *val)
{
	void __iomem *sync_regs = host->sync_aperture;

	nvhost_module_busy(host->dev);
	*val = readl(sync_regs + host1x_sync_actmon_avg_count_r());
	nvhost_module_idle(host->dev);
	rmb();

	return 0;
}

static void host1x_actmon_intr_above_wmark(void)
{
	above_wmark++;
}

static void host1x_actmon_intr_below_wmark(void)
{
	below_wmark++;
}

static int host1x_actmon_above_wmark_count(struct nvhost_master *host)
{
	return above_wmark;
}

static int host1x_actmon_below_wmark_count(struct nvhost_master *host)
{
	return below_wmark;
}

static int host1x_actmon_process_isr(u32 hintstatus, void __iomem *sync_regs)
{
	if (host1x_sync_hintstatus_gr3d_actmon_intr_v(hintstatus)) {
		u32 actmon =
			readl(sync_regs + host1x_sync_actmon_intr_status_r());
		if (host1x_sync_actmon_intr_status_avg_below_wmark_v(actmon))
			host1x_actmon_intr_below_wmark();
		if (host1x_sync_actmon_intr_status_avg_above_wmark_v(actmon))
			host1x_actmon_intr_above_wmark();

		writel(actmon, sync_regs + host1x_sync_actmon_intr_status_r());
		return 1;
	} else
		return 0;
}

static const struct nvhost_actmon_ops host1x_actmon_ops = {
	.init = host1x_actmon_init,
	.deinit = host1x_actmon_deinit,
	.read_avg = host1x_actmon_avg,
	.above_wmark_count = host1x_actmon_above_wmark_count,
	.below_wmark_count = host1x_actmon_below_wmark_count,
	.isr = host1x_actmon_process_isr,
};
