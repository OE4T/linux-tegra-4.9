/*
 * drivers/video/tegra/dc/bandwidth.c
 *
 * Copyright (c) 2010-2013, NVIDIA CORPORATION, All rights reserved.
 *
 * Author: Jon Mayo <jmayo@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>

#include <mach/clk.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/mc.h>
#include <linux/nvhost.h>
#include <mach/latency_allowance.h>
#include <trace/events/display.h>

#include "dc_reg.h"
#include "dc_config.h"
#include "dc_priv.h"

static int use_dynamic_emc = 1;

module_param_named(use_dynamic_emc, use_dynamic_emc, int, S_IRUGO | S_IWUSR);

/* uses the larger of w->bandwidth or w->new_bandwidth */
static void tegra_dc_set_latency_allowance(struct tegra_dc *dc,
	struct tegra_dc_win *w)
{
	unsigned long bw;
	/* windows A, B, C for first and second display */
	static const enum tegra_la_id la_id_tab[2][DC_N_WINDOWS] = {
		/* first display */
		{
			TEGRA_LA_DISPLAY_0A,
			TEGRA_LA_DISPLAY_0B,
			TEGRA_LA_DISPLAY_0C,
#if defined(CONFIG_ARCH_TEGRA_14x_SOC)
			TEGRA_LA_DISPLAYD,
			TEGRA_LA_DISPLAY_HC,
#endif
		},
		/* second display */
		{
			TEGRA_LA_DISPLAY_0AB,
			TEGRA_LA_DISPLAY_0BB,
			TEGRA_LA_DISPLAY_0CB,
#if defined(CONFIG_ARCH_TEGRA_14x_SOC)
			0,
			TEGRA_LA_DISPLAY_HCB,
#endif
		},
	};
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
	/* window B V-filter tap for first and second display. */
	static const enum tegra_la_id vfilter_tab[2] = {
		TEGRA_LA_DISPLAY_1B, TEGRA_LA_DISPLAY_1BB,
	};
#endif

	BUG_ON(dc->ndev->id >= ARRAY_SIZE(la_id_tab));
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
	BUG_ON(dc->ndev->id >= ARRAY_SIZE(vfilter_tab));
#endif
	BUG_ON(w->idx >= ARRAY_SIZE(*la_id_tab));

	bw = max(w->bandwidth, w->new_bandwidth);

#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
	/* tegra_dc_get_bandwidth() treats V filter windows as double
	 * bandwidth, but LA has a seperate client for V filter */
	if (w->idx == 1 && win_use_v_filter(dc, w))
		bw /= 2;
#endif

	/* our bandwidth is in kbytes/sec, but LA takes MBps.
	 * round up bandwidth to next 1MBps */
	if (bw != ULONG_MAX)
		bw = bw / 1000 + 1;

	tegra_set_latency_allowance(la_id_tab[dc->ndev->id][w->idx], bw);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
	/* if window B, also set the 1B client for the 2-tap V filter. */
	if (w->idx == 1)
		tegra_set_latency_allowance(vfilter_tab[dc->ndev->id], bw);
#endif
}

static int tegra_dc_windows_is_overlapped(struct tegra_dc_win *a,
	struct tegra_dc_win *b)
{
	if (!WIN_IS_ENABLED(a) || !WIN_IS_ENABLED(b))
		return 0;

	/* because memory access to load the fifo can overlap, only care
	 * if windows overlap vertically */
	return ((a->out_y + a->out_h > b->out_y) && (a->out_y <= b->out_y)) ||
		((b->out_y + b->out_h > a->out_y) && (b->out_y <= a->out_y));
}

/* check overlapping window combinations to find the max bandwidth. */
static unsigned long tegra_dc_find_max_bandwidth(struct tegra_dc_win *wins[],
						 unsigned n)
{
	unsigned i;
	unsigned j;
	unsigned long bw;
	unsigned long max = 0;

	for (i = 0; i < n; i++) {
		bw = wins[i]->new_bandwidth;
		for (j = 0; j < n; j++)
			if (tegra_dc_windows_is_overlapped(wins[i], wins[j]))
				bw += wins[j]->new_bandwidth;
		if (max < bw)
			max = bw;
	}
	return max;
}

/*
 * Calculate peak EMC bandwidth for each enabled window =
 * pixel_clock * win_bpp * (use_v_filter ? 2 : 1)) * H_scale_factor *
 * (windows_tiling ? 2 : 1)
 *
 * note:
 * (*) We use 2 tap V filter on T2x/T3x, so need double BW if use V filter
 * (*) Tiling mode on T30 and DDR3 requires double BW
 *
 * return:
 * bandwidth in kBps
 */
static unsigned long tegra_dc_calc_win_bandwidth(struct tegra_dc *dc,
	struct tegra_dc_win *w)
{
	unsigned long ret;
	int tiled_windows_bw_multiplier;
	unsigned long bpp;
	unsigned in_w;

	if (!WIN_IS_ENABLED(w))
		return 0;

	if (dfixed_trunc(w->w) == 0 || dfixed_trunc(w->h) == 0 ||
	    w->out_w == 0 || w->out_h == 0)
		return 0;
	if (w->flags & TEGRA_WIN_FLAG_SCAN_COLUMN)
		/* rotated: PRESCALE_SIZE swapped, but WIN_SIZE is unchanged */
		in_w = dfixed_trunc(w->h);
	else
		in_w = dfixed_trunc(w->w); /* normal output, not rotated */

	tiled_windows_bw_multiplier =
		tegra_mc_get_tiled_memory_bandwidth_multiplier();

	/* all of tegra's YUV formats(420 and 422) fetch 2 bytes per pixel,
	 * but the size reported by tegra_dc_fmt_bpp for the planar version
	 * is of the luma plane's size only. */
	bpp = tegra_dc_is_yuv_planar(w->fmt) ?
		2 * tegra_dc_fmt_bpp(w->fmt) : tegra_dc_fmt_bpp(w->fmt);
	ret = dc->mode.pclk / 1000UL * bpp / 8 *
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
		(win_use_v_filter(dc, w) ? 2 : 1) *
#endif
		in_w / w->out_w * (WIN_IS_TILED(w) ?
		tiled_windows_bw_multiplier : 1);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	/*
	 * Assuming 60% efficiency: i.e. if we calculate we need 70MBps, we
	 * will request 117MBps from EMC.
	 */
	ret = ret + (17 * ret / 25);
#endif
	return ret;
}

static unsigned long tegra_dc_get_bandwidth(
	struct tegra_dc_win *windows[], int n)
{
	int i;

	BUG_ON(n > DC_N_WINDOWS);

	/* emc rate and latency allowance both need to know per window
	 * bandwidths */
	for (i = 0; i < n; i++) {
		struct tegra_dc_win *w = windows[i];

		if (w)
			w->new_bandwidth =
				tegra_dc_calc_win_bandwidth(w->dc, w);
	}

	return tegra_dc_find_max_bandwidth(windows, n);
}

#ifdef CONFIG_TEGRA_ISOMGR
/* to save power, call when display memory clients would be idle */
void tegra_dc_clear_bandwidth(struct tegra_dc *dc)
{
	int latency;

	trace_clear_bandwidth(dc);
	latency = tegra_isomgr_reserve(dc->isomgr_handle, 0, 1000);
	WARN_ONCE(!latency, "tegra_isomgr_reserve failed\n");
	if (latency) {
		latency = tegra_isomgr_realize(dc->isomgr_handle);
		WARN_ONCE(!latency, "tegra_isomgr_realize failed\n");
	} else {
		tegra_dc_ext_process_bandwidth_renegotiate(dc->ndev->id);
	}
	dc->bw_kbps = 0;
}
#else
/* to save power, call when display memory clients would be idle */
void tegra_dc_clear_bandwidth(struct tegra_dc *dc)
{
	trace_clear_bandwidth(dc);
	if (tegra_is_clk_enabled(dc->emc_clk))
		clk_disable_unprepare(dc->emc_clk);
	dc->bw_kbps = 0;
}

/* bw in kByte/second. returns Hz for EMC frequency */
static inline unsigned long tegra_dc_kbps_to_emc(unsigned long bw)
{
	unsigned long freq;

	if (bw == ULONG_MAX)
		return ULONG_MAX;

	freq = tegra_emc_bw_to_freq_req(bw);
	if (freq >= (ULONG_MAX / 1000))
		return ULONG_MAX; /* freq too big - clamp at max */

	if (WARN_ONCE((freq * 1000) < freq, "Bandwidth Overflow"))
		return ULONG_MAX; /* should never occur because of above. */
	return freq * 1000;
}
#endif

/* use the larger of dc->bw_kbps or dc->new_bw_kbps, and copies
 * dc->new_bw_kbps into dc->bw_kbps.
 * calling this function both before and after a flip is sufficient to select
 * the best possible frequency and latency allowance.
 * set use_new to true to force dc->new_bw_kbps programming.
 */
void tegra_dc_program_bandwidth(struct tegra_dc *dc, bool use_new)
{
	unsigned i;

	if (use_new || dc->bw_kbps != dc->new_bw_kbps) {
		long bw = max(dc->bw_kbps, dc->new_bw_kbps);

#ifdef CONFIG_TEGRA_ISOMGR
		int latency;

		latency = tegra_isomgr_reserve(dc->isomgr_handle, bw, 1000);
		if (latency) {
			latency = tegra_isomgr_realize(dc->isomgr_handle);
			WARN_ONCE(!latency, "tegra_isomgr_realize failed\n");
		} else {
			tegra_dc_ext_process_bandwidth_renegotiate(
				dc->ndev->id);
		}
#else /* EMC version */
		int emc_freq;

		/* going from 0 to non-zero */
		if (!dc->bw_kbps && dc->new_bw_kbps &&
			!tegra_is_clk_enabled(dc->emc_clk))
			clk_prepare_enable(dc->emc_clk);

		emc_freq = tegra_dc_kbps_to_emc(bw);
		clk_set_rate(dc->emc_clk, emc_freq);

		/* going from non-zero to 0 */
		if (dc->bw_kbps && !dc->new_bw_kbps &&
			tegra_is_clk_enabled(dc->emc_clk))
			clk_disable_unprepare(dc->emc_clk);
#endif
		dc->bw_kbps = dc->new_bw_kbps;
	}

	for (i = 0; i < DC_N_WINDOWS; i++) {
		struct tegra_dc_win *w = &dc->windows[i];

		if ((use_new || w->bandwidth != w->new_bandwidth) &&
			w->new_bandwidth != 0)
			tegra_dc_set_latency_allowance(dc, w);
		trace_program_bandwidth(dc);
		w->bandwidth = w->new_bandwidth;
	}
}

int tegra_dc_set_dynamic_emc(struct tegra_dc_win *windows[], int n)
{
	unsigned long new_rate;
	struct tegra_dc *dc;

	if (!use_dynamic_emc)
		return 0;

	dc = windows[0]->dc;

#ifdef CONFIG_TEGRA_ISOMGR
	new_rate = tegra_dc_get_bandwidth(windows, n);
#else
	if (tegra_dc_has_multiple_dc())
		new_rate = ULONG_MAX;
	else
		new_rate = tegra_dc_get_bandwidth(windows, n);
#endif

	dc->new_bw_kbps = new_rate;
	trace_set_dynamic_emc(dc);

	return 0;
}

/* return the minimum bandwidth in kbps for display to function */
long tegra_dc_calc_min_bandwidth(struct tegra_dc *dc)
{
	unsigned pclk = tegra_dc_get_out_max_pixclock(dc);

	if (WARN_ONCE(!dc, "dc is NULL") ||
		WARN_ONCE(!dc->out, "dc->out is NULL!"))
		return 0;

	if (!pclk && dc->out->type == TEGRA_DC_OUT_HDMI) {
		pclk = tegra_dc_get_out_max_pixclock(dc);
		if (!pclk) {
#if defined(CONFIG_ARCH_TEGRA_11x_SOC)
			pclk = 300000000; /* 300MHz max */
#else
			pclk = 150000000; /* 150MHz max */
#endif
		}
	} else {
		pclk = dc->mode.pclk;
	}
	return pclk / 1000 * 4; /* support a single 32bpp window */
}

#ifdef CONFIG_TEGRA_ISOMGR
void tegra_dc_bandwidth_renegotiate(void *p, u32 avail_bw)
{
	struct tegra_dc *dc = p;
	unsigned long bw;

	if (WARN_ONCE(!dc, "dc is NULL!"))
		return;
	tegra_dc_ext_process_bandwidth_renegotiate(dc->ndev->id);

	/* a bit of a hack, report the change in bandwidth before it
	 * really happens.
	 */
	bw = tegra_dc_calc_min_bandwidth(dc);
	if (tegra_isomgr_reserve(dc->isomgr_handle, 0, 1000))
		tegra_isomgr_realize(dc->isomgr_handle);
}
#endif
