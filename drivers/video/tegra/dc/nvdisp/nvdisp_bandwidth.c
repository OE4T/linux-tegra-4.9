/*
 * drivers/video/tegra/dc/nvdisp/nvdisp_bandwidth.c
 *
 * Copyright (c) 2016, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform/tegra/bwmgr_mc.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/isomgr.h>
#include <linux/platform/tegra/latency_allowance.h>

#include <mach/dc.h>
#include <mach/tegra_dc_ext.h>

#include "dc_priv.h"
#include "nvdisp.h"

#ifdef CONFIG_TEGRA_ISOMGR

/*
 * The following configs were calculated offline with these common settings:
 *
 * - 3 active heads:
 *	- 4096x2160@60p
 * - 1 active cursor per head:
 *	- 4BPP packed
 *	- Pitch
 *	- LUT disabled
 * - N active windows across all heads:
 *	- 4BPP packed
 *	- BLx4
 *	- LUT disabled
 *	- Horizontal/vertical scaling disabled
 *	- Compression disabled
 *	- Rotation disabled
 *	- Fullscreen
 */
static struct nvdisp_bandwidth_config max_bw_configs[] = {
	/* 1 window */
	{
		.iso_bw = 3136300,		/* 3136.3   MB/s */
		.total_bw = 2851200,		/* 2851.2   MB/s */
		.emc_la_floor = 102000000,	/* 102      MHz  */
		.hubclk = 169820000,		/* 169.82   MHz  */
	},
	/* 2 windows */
	{
		.iso_bw = 5924200,		/* 5924.2   MB/s */
		.total_bw = 5385600,		/* 5385.6   MB/s */
		.emc_la_floor = 204000000,	/* 204      MHz  */
		.hubclk = 207380000,		/* 207.38   MHz  */
	},
	/* 3 windows */
	{
		.iso_bw = 8363500,		/* 8363.5   MB/s */
		.total_bw = 7603200,		/* 7603.2   MB/s */
		.emc_la_floor = 332800000,	/* 332.8    MHz  */
		.hubclk = 244940000,		/* 244.94   MHz  */
	},
	/* 4 windows */
	{
		.iso_bw = 11151400,		/* 11151.14 MB/s */
		.total_bw = 10137600,		/* 10137.6  MB/s */
		.emc_la_floor = 332800000,	/* 332.8    MHz  */
		.hubclk = 282500000,		/* 282.5    MHz  */
	},
	/* 5 windows */
	{
		.iso_bw = 13939200,		/* 13939.2  MB/s */
		.total_bw = 12672000,		/* 12672    MB/s */
		.emc_la_floor = 531200000,	/* 531.2    MHz  */
		.hubclk = 320060000,		/* 320.06   MHz  */
	},
	/* 6 windows */
	{
		.iso_bw = 16727000,		/* 16727    MB/s */
		.total_bw = 15206400,		/* 15206.4  MB/s */
		.emc_la_floor = 665600000,	/* 665.6    MHz  */
		.hubclk = 357620000,		/* 357.62   MHz  */
	},
};

/* Output id that we pass to tegra_dc_ext_process_bandwidth_negotiate */
#define NVDISP_BW_OUTPUT_ID		0

/* Global bw info shared across all heads */
static struct nvdisp_isoclient_bw_info ihub_bw_info;

static u32 tegra_nvdisp_get_max_pending_bw(struct tegra_dc *dc)
{
	struct tegra_dc_imp_settings *settings;
	u32 max_pending_bw = 0;

	list_for_each_entry(settings, &nvdisp_imp_settings_queue, imp_node) {
		u32 pending_bw =
			settings->ext_settings.total_display_iso_bw_kbps;

		if (pending_bw > max_pending_bw)
			max_pending_bw = pending_bw;
	}

	return max_pending_bw;
}

static int tegra_nvdisp_set_latency_allowance(u32 bw, u32 emc_freq)
{
	struct dc_to_la_params disp_params;
	int ret = 0;

	if (bw == 0)
		return ret;

	/* Zero out this struct since it's ignored by the LA/PTSA driver. */
	memset(&disp_params, 0, sizeof(disp_params));

	/* Our bw is in KB/s, but LA takes MB/s. Round up to the next MB/s. */
	if (bw != U32_MAX)
		bw = bw / 1000 + 1;

	return tegra_set_disp_latency_allowance(TEGRA_LA_NVDISPLAYR,
					emc_freq,
					bw,
					disp_params);
}

int tegra_nvdisp_program_bandwidth(struct tegra_dc *dc,
				u32 new_iso_bw,
				u32 new_total_bw,
				u32 new_emc,
				u32 new_hubclk,
				bool before_win_update)
{
	/*
	 * This function is responsible for updating the ISO bw, EMC floor,
	 * LA/PTSA, and hubclk values both before and after the current window
	 * update. In the two below cases, let X represent the current frame
	 * configuration and X' represent the new proposed frame configuration.
	 *
	 * A) Before the window update actually occurs, display needs to ensure
	 *    that the four aforementioned values are compatible with both X and
	 *    X'. This is due to the fact that any updates to these values take
	 *    effect immediately and aren't latched to any kind of frame
	 *    boundary. In order to update these values appropriately, display
	 *    will simply program the max across both X and X'.
	 * B) After the window update occurs and the new state has promoted,
	 *    display can program all the new values associated with X' as long
	 *    as they don't violate the ISO bw requirements that are currently
	 *    in place.
	 */

	struct nvdisp_bandwidth_config *cur_config = &ihub_bw_info.cur_config;
	u32 final_iso_bw = 0;
	u32 final_total_bw = 0;
	u32 final_emc = 0;
	u32 final_hubclk = 0;
	bool update_la_ptsa = false;
	int ret = 0;

	if (IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle) ||
				IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle)) {
		ret = -EINVAL;
		goto exit;
	}

	final_iso_bw = cur_config->iso_bw;
	final_total_bw = cur_config->total_bw;
	final_emc = cur_config->emc_la_floor;
	final_hubclk = cur_config->hubclk;

	if (before_win_update) { /* Case A */
		bool update_bw = false;

		/*
		 * ISO clients can only realize exactly what they have already
		 * reserved. The ISO bw that display has currently reserved is
		 * always guaranteed to be at least the bw needed for the
		 * proposed configuration since we aggregate bw reservations
		 * during PROPOSE.
		 */
		if (new_iso_bw > cur_config->iso_bw) {
			final_iso_bw = ihub_bw_info.reserved_bw;
			update_bw = true;
		}

		final_emc = max(final_emc, new_emc);
		if (update_bw)
			final_emc = max(final_emc, ihub_bw_info.emc_at_res_bw);

		final_hubclk = max(final_hubclk, new_hubclk);
		if (update_bw)
			final_hubclk =
			max(final_hubclk, ihub_bw_info.hubclk_at_res_bw);
	} else { /* Case B */
		u32 max_bw = tegra_nvdisp_get_max_pending_bw(dc);
		if (new_iso_bw >= max_bw &&
					new_iso_bw < cur_config->iso_bw) {
			/*
			 * Client's latency tolerance is ignored by isomgr. Pass
			 * in a dummy value of 1000 usec.
			 */
			if (!tegra_isomgr_reserve(ihub_bw_info.isomgr_handle,
						new_iso_bw,
						1000)) {
				pr_err("%s: failed to reserve %u KB/s\n",
					__func__, new_iso_bw);
				ret = -EINVAL;
				goto exit;
			}

			ihub_bw_info.reserved_bw = new_iso_bw;
			ihub_bw_info.emc_at_res_bw = new_emc;
			ihub_bw_info.hubclk_at_res_bw = new_hubclk;
			cur_config->total_bw = new_total_bw;

			final_iso_bw = new_iso_bw;
			final_total_bw = new_total_bw;
			final_emc = new_emc;
			final_hubclk = new_hubclk;
		}
	}

	if (before_win_update && final_hubclk != cur_config->hubclk) {
		clk_set_rate(hubclk, final_hubclk);
		cur_config->hubclk = final_hubclk;
	}

	if (final_iso_bw != cur_config->iso_bw) {
		if (!tegra_isomgr_realize(ihub_bw_info.isomgr_handle)) {
			pr_err("%s: failed to realize %u KB/s\n", __func__,
				final_iso_bw);
			ret = -EINVAL;
			goto exit;
		}

		cur_config->iso_bw = final_iso_bw;
		update_la_ptsa = true;
	}

	if (final_emc != cur_config->emc_la_floor) {
		/*
		 * tegra_bwmgr_set_emc() takes in the DRAM frequency. We need
		 * to use the conversion factor to properly convert the required
		 * EMC frequency to its corresponding DRAM frequency.
		 */
		int freq_factor = bwmgr_get_emc_to_dram_freq_factor();

		ret = tegra_bwmgr_set_emc(ihub_bw_info.bwmgr_handle,
					final_emc * freq_factor,
					TEGRA_BWMGR_SET_EMC_FLOOR);
		if (ret) {
			pr_err("%s: failed to set EMC floor=%u Hz\n", __func__,
				final_emc);
			goto exit;
		}

		cur_config->emc_la_floor = final_emc;
		update_la_ptsa = true;
	}

	if (update_la_ptsa) {
		/*
		 * If either our ISO bw requirement or the EMC floor has
		 * changed, we need to update LA/PTSA. The bw value that we pass
		 * to the LA/PTSA driver should not include the catchup factor.
		 */
		ret = tegra_nvdisp_set_latency_allowance(final_total_bw,
								final_emc);
		if (ret) {
			pr_err("%s: LA/PTSA failed w/ bw=%u KB/s,freq=%u Hz\n",
				__func__, final_total_bw, final_emc);
			goto exit;
		}
	}

	if (!before_win_update && final_hubclk != cur_config->hubclk) {
		clk_set_rate(hubclk, final_hubclk);
		cur_config->hubclk = final_hubclk;
	}

exit:
	return ret;
}

void tegra_nvdisp_init_bandwidth(struct tegra_dc *dc)
{
	/*
	 * Use the max config settings. These values will eventually be adjusted
	 * through IMP, if the client supports it.
	 */
	struct nvdisp_bandwidth_config *max_bw_config = ihub_bw_info.max_config;
	u32 new_iso_bw = 0;
	u32 new_total_bw = 0;
	u32 new_emc = 0;
	u32 new_hubclk = 0;
	bool before_win_update = true;

	if (IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle) ||
		IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle) ||
		!max_bw_config)
		return;

	new_iso_bw = max_bw_config->iso_bw;
	new_total_bw = max_bw_config->total_bw;
	new_emc = max_bw_config->emc_la_floor;
	new_hubclk = max_bw_config->hubclk;

	tegra_nvdisp_negotiate_reserved_bw(dc,
				new_iso_bw,
				new_total_bw,
				new_emc,
				new_hubclk);
	tegra_nvdisp_program_bandwidth(dc,
				new_iso_bw,
				new_total_bw,
				new_emc,
				new_hubclk,
				before_win_update);
}

void tegra_nvdisp_clear_bandwidth(struct tegra_dc *dc)
{
	u32 new_iso_bw = 0;
	u32 new_total_bw = 0;
	u32 new_emc = 0;
	u32 new_hubclk = 0;
	bool before_win_update = false;

	tegra_nvdisp_program_bandwidth(dc,
				new_iso_bw,
				new_total_bw,
				new_emc,
				new_hubclk,
				before_win_update);
}

/*
 * tegra_dc_calc_min_bandwidth - returns the minimum dedicated ISO bw
 *
 * @dc		dc instance
 *
 * @retval	minimum dedicated ISO bw
 */
long tegra_dc_calc_min_bandwidth(struct tegra_dc *dc)
{
	long bw = 0;

	if (ihub_bw_info.max_config)
		bw = ihub_bw_info.max_config->iso_bw;

	return bw;
}

int tegra_nvdisp_negotiate_reserved_bw(struct tegra_dc *dc,
				u32 new_iso_bw,
				u32 new_total_bw,
				u32 new_emc,
				u32 new_hubclk)
{
	/*
	 * There are two possible cases:
	 * A) If the proposed ISO bw is greater than the available bw, return an
	 *    error.
	 * B) If the proposed ISO bw is greater than the bw that is currently
	 *    reserved, try to reserve the proposed bw. Else, the current
	 *    reserved bw is already sufficient.
	 *
	 * This function is only responsible for reserving bw, NOT realizing it.
	 */

	int ret = 0;

	if (IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle) ||
				IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle)) {
		ret = -EINVAL;
		goto exit;
	}

	if (new_iso_bw > ihub_bw_info.available_bw) { /* Case A */
		pr_err("%s: requested %u KB/s > available %u KB/s",
			__func__, new_iso_bw, ihub_bw_info.available_bw);
		ret = -E2BIG;
		goto exit;
	}

	if (new_iso_bw > ihub_bw_info.reserved_bw) { /* Case B */
		/*
		 * Client's latency tolerance is ignored by isomgr. Pass in a
		 * dummy value of 1000 usec.
		 */
		if (!tegra_isomgr_reserve(ihub_bw_info.isomgr_handle,
						new_iso_bw,
						1000)) {
			pr_err("%s: failed to reserve %u KB/s\n", __func__,
				new_iso_bw);
			ret = -EINVAL;
			goto exit;
		}

		ihub_bw_info.reserved_bw = new_iso_bw;
		ihub_bw_info.emc_at_res_bw = new_emc;
		ihub_bw_info.hubclk_at_res_bw = new_hubclk;
		ihub_bw_info.cur_config.total_bw = new_total_bw;
	}

exit:
	return ret;
}

static void tegra_nvdisp_bandwidth_renegotiate(void *p, u32 avail_bw)
{
	struct tegra_dc_bw_data data;
	struct nvdisp_isoclient_bw_info *bw_info = p;

	if (!bw_info) {
		pr_err("%s: bw_info is NULL!\n", __func__);
		return;
	}

	if (IS_ERR_OR_NULL(bw_info->isomgr_handle) ||
					IS_ERR_OR_NULL(bw_info->bwmgr_handle))
		return;

	mutex_lock(&tegra_nvdisp_lock);

	if (bw_info->available_bw == avail_bw) {
		mutex_unlock(&tegra_nvdisp_lock);
		return;
	}

	data.total_bw = tegra_isomgr_get_total_iso_bw();
	data.avail_bw = avail_bw;
	data.resvd_bw = bw_info->reserved_bw;

	tegra_dc_ext_process_bandwidth_renegotiate(NVDISP_BW_OUTPUT_ID, &data);

	bw_info->available_bw = avail_bw;

	mutex_unlock(&tegra_nvdisp_lock);
}

/*
 * tegra_nvdisp_bandwidth_attach - save reference to ihub_bw_info
 *
 * @dc	dc instance
 */
void tegra_nvdisp_bandwidth_attach(struct tegra_dc *dc)
{
	if (!dc) {
		pr_err("%s: dc is NULL!\n", __func__);
		return;
	}

	dc->ihub_bw_info = &ihub_bw_info;
}

static int tegra_nvdisp_bandwidth_register_max_config(
					enum tegra_iso_client iso_client)
{
	tegra_isomgr_handle isomgr_handle = NULL;
	u32 max_emc_rate, emc_to_dram_factor;
	u32 total_iso_bw;
	int ret = 0, i;

	/* DRAM frequency (Hz) */
	max_emc_rate = tegra_bwmgr_get_max_emc_rate();

	/* conversion factor */
	emc_to_dram_factor = bwmgr_get_emc_to_dram_freq_factor();

	total_iso_bw = tegra_isomgr_get_total_iso_bw();

	/*
	 * WAR: Return immediately if either the total ISO bw or the max EMC
	 * clock are reported as 0.
	 */
	if (max_emc_rate == 0 || total_iso_bw == 0)
		return 0;

	/*
	 * Start with the highest-bw config and continue to fallback until we
	 * find one that works.
	 */
	ihub_bw_info.max_config = NULL;
	for (i = ARRAY_SIZE(max_bw_configs) - 1; i >= 0; i--) {
		struct nvdisp_bandwidth_config *cfg = &max_bw_configs[i];
		u32 cfg_dram_freq = 0;

		/*
		 * Check that our dedicated request doesn't exceed the total ISO
		 * bw. Both operands are in KB/s.
		 */
		if (cfg->iso_bw > total_iso_bw) {
			ret = -E2BIG;
			continue;
		}

		/* Make sure isomgr registration succeeds. */
		isomgr_handle = tegra_isomgr_register(iso_client,
					cfg->iso_bw,
					tegra_nvdisp_bandwidth_renegotiate,
					&ihub_bw_info);
		if (IS_ERR_OR_NULL(isomgr_handle)) {
			ret = -ENOENT;
			continue;
		}

		/*
		 * Check that the required EMC floor doesn't exceed the max EMC
		 * rate allowed.
		 */
		cfg_dram_freq = cfg->emc_la_floor * emc_to_dram_factor;
		if (tegra_bwmgr_round_rate(cfg_dram_freq) > max_emc_rate) {
			tegra_isomgr_unregister(isomgr_handle);
			ret = -E2BIG;
			continue;
		}

		ihub_bw_info.isomgr_handle = isomgr_handle;
		ihub_bw_info.max_config = cfg;

		pr_info("%s: max config iso bw = %u KB/s\n",
			__func__, cfg->iso_bw);
		pr_info("%s: max config EMC floor = %u Hz\n",
			__func__, cfg->emc_la_floor);
		pr_info("%s: max config hubclk = %u Hz\n",
			__func__, cfg->hubclk);
	}

	if (ihub_bw_info.max_config)
		ret = 0;
	else
		pr_err("%s: couldn't find valid max config!\n", __func__);

	return ret;
}

/*
 * tegra_nvdisp_bandwidth_register - register the IHUB BW client
 *
 * @iso_client		ISO client id
 * @bwmgr_client	BWMGR client id
 * @udedi_bw		minimum dedicated ISO bw (KB/s) needed by client
 *
 * @retval		0 on success
 * @retval		-ENOENT on registration failures
 */
int tegra_nvdisp_bandwidth_register(enum tegra_iso_client iso_client,
				enum tegra_bwmgr_client_id bwmgr_client)
{
	int ret = 0;

	mutex_lock(&tegra_nvdisp_lock);
	memset(&ihub_bw_info, 0, sizeof(ihub_bw_info));

	ihub_bw_info.bwmgr_handle = tegra_bwmgr_register(bwmgr_client);
	if (IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle)) {
		ret = -ENOENT;
		goto unlock_and_ret;
	}

	ret = tegra_nvdisp_bandwidth_register_max_config(iso_client);
	if (ret)
		goto unlock_and_ret;

	/*
	 * Assume that all the bandwidth is currently available to us so that we
	 * can reserve as much as needed until we are told by isomgr to backoff.
	 */
	ihub_bw_info.available_bw = UINT_MAX;

unlock_and_ret:
	mutex_unlock(&tegra_nvdisp_lock);
	return ret;
}

/*
 * tegra_nvdisp_bandwidth_unregister - unregister the IHUB BW client
 */
void tegra_nvdisp_bandwidth_unregister(void)
{
	mutex_lock(&tegra_nvdisp_lock);

	if (!IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle))
		tegra_isomgr_unregister(ihub_bw_info.isomgr_handle);
	if (!IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle))
		tegra_bwmgr_unregister(ihub_bw_info.bwmgr_handle);

	mutex_unlock(&tegra_nvdisp_lock);
}
#else
int tegra_nvdisp_program_bandwidth(struct tegra_dc *dc, u32 new_iso_bw,
	u32 new_total_bw, u32 new_emc, u32 new_hubclk,
	bool before_win_update)
{
	return -ENOSYS;
}
int tegra_nvdisp_negotiate_reserved_bw(struct tegra_dc *dc, u32 new_iso_bw,
	u32 new_total_bw, u32 new_emc, u32 new_hubclk)
{
	return -ENOSYS;
}
void tegra_nvdisp_init_bandwidth(struct tegra_dc *dc) {}
void tegra_nvdisp_clear_bandwidth(struct tegra_dc *dc) {}
long tegra_dc_calc_min_bandwidth(struct tegra_dc *dc)
{
	return -ENOSYS;
}
#endif /* CONFIG_TEGRA_ISOMGR */

/* These functions are all NO-OPs in accordance with the new IMP model. */
void tegra_dc_clear_bandwidth(struct tegra_dc *dc) {}
void tegra_dc_program_bandwidth(struct tegra_dc *dc, bool use_new) {}
int tegra_dc_bandwidth_negotiate_bw(struct tegra_dc *dc,
			struct tegra_dc_win *windows[],
			int n)
{
	return 0;
}
EXPORT_SYMBOL(tegra_dc_bandwidth_negotiate_bw);

unsigned long tegra_dc_get_bandwidth(struct tegra_dc_win *windows[], int n)
{
	return -ENOSYS;
}
EXPORT_SYMBOL(tegra_dc_get_bandwidth);
