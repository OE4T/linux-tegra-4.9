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
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/isomgr.h>
#include <linux/platform/tegra/latency_allowance.h>

#include <mach/dc.h>
#include <mach/tegra_dc_ext.h>

#include "dc_priv.h"
#include "nvdisp.h"

#ifdef CONFIG_TEGRA_ISOMGR
/*
 * NVDISP_BW_DEDI_BW_KBPS was calculated with the following settings:
 * - 1 active head:
 *	- 4096x2160@60p
 * - 1 active window on that head:
 *	- Fullscreen
 *	- 4BPP packed
 *	- BLx4
 *	- LUT disabled
 *	- Scaling disabled
 *	- Rotation disabled
 * - Cursor active on that head:
 *	- 4BPP packed
 *	- Pitch
 *	- LUT disabled
 *
 * NVDISP_BW_MAX_BW_KBPS, NVDISP_BW_TOTAL_MC_LATENCY, and
 * NVDISP_BW_REQ_HUBCLK_HZ were calculated with these settings:
 * - 3 active heads:
 *	- 4096x2160@60p
 * - 2 active windows on each head:
 *	- Fullscreen
 *	- 4BPP packed
 *	- BLx4
 *	- LUT disabled
 *	- Scaling disabled
 *	- Rotation disabled
 * - Cursor active on each head:
 *	- 4BPP packed
 *	- Pitch
 *	- LUT disabled
 *
 * Refer to setDefaultSysParams in libnvimp/nvimp.c for the default sys param
 * values that were used.
 *
 * The below values are rounded to the nearest unit.
 */

/* Minimum dedicated bw that is allocated to display (KB/s) */
#define NVDISP_BW_DEDI_BW_KBPS		2535000

/* Maximum bw that display could potentially need (KB/s) */
#define NVDISP_BW_MAX_BW_KBPS		15207000

/* Total MC request latency that display can tolerate (usec) */
#define NVDISP_BW_TOTAL_MC_LATENCY	1

/* Required hubclk rate for max config (Hz) */
#define NVDISP_BW_REQ_HUBCLK_HZ		362000000

/* Output id that we pass to tegra_dc_ext_process_bandwidth_negotiate */
#define NVDISP_BW_OUTPUT_ID		0

/* Global bw info shared across all heads */
static struct nvdisp_isoclient_bw_info ihub_bw_info;

static u32 tegra_nvdisp_get_max_pending_bw(struct tegra_dc *dc)
{
	struct tegra_dc_imp_settings *settings;
	u32 max_pending_bw = 0;

	list_for_each_entry(settings, &nvdisp_imp_settings_queue, imp_node) {
		/*
		 * dc->ctrl_num should always be a valid idx since we copy
		 * results to all heads, whether they're active or inactive.
		 */
		u32 pending_bw =
		settings->imp_results[dc->ctrl_num].required_total_bw_kbps;
		if (pending_bw > max_pending_bw)
			max_pending_bw = pending_bw;
	}

	return max_pending_bw;
}

static void tegra_dc_set_latency_allowance(u32 bw)
{
	struct dc_to_la_params disp_params;
	unsigned long emc_freq_hz = tegra_bwmgr_get_emc_rate();

	/* Zero out this struct since it's ignored by the LA/PTSA driver. */
	memset(&disp_params, 0, sizeof(disp_params));

	/* Our bw is in KB/s, but LA takes MB/s. Round up to the next MB/s. */
	if (bw != U32_MAX)
		bw = bw / 1000 + 1;

	if (tegra_set_disp_latency_allowance(TEGRA_LA_NVDISPLAYR,
					emc_freq_hz,
					bw,
					disp_params))
		pr_err("Failed to set latency allowance\n");
}

void tegra_nvdisp_program_bandwidth(struct tegra_dc *dc,
				u32 proposed_bw,
				u32 proposed_latency,
				u32 proposed_hubclk,
				bool before_win_update)
{
	/*
	 * This function handles two cases:
	 * A) Bw and LA/PTSA changes take effect immediately. Before we program
	 *    the window state, we must make sure that the effective bw can
	 *    satisfy both the current and upcoming frames.
	 *
	 *    Note that the bw we might potentially realize during this step is
	 *    just whatever has been reserved at this point, and is not
	 *    necessarily the proposed bw. However, the reserved bw is
	 *    guaranteed to be sufficient since we always check against the max
	 *    pending bw.
	 *
	 *    If we end up increasing the realized bw, increase the hubclk rate
	 *    after.
	 * B) After the window state has promoted, we need to check if we can
	 *    decrease the reserved and realized bw. We can safely decrease
	 *    these values if the target bw is still enough to satisfy all the
	 *    pending requests.
	 *
	 *    If we end up decreasing the realized bw, decrease the hubclk rate
	 *    before.
	 */

	u32 realized_bw = ihub_bw_info.realized_bw_kbps;
	u32 new_bw_to_realize = 0;
	u32 latency = 0;
	bool update_realized_bw = false;

	if (IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle))
		return;

	if (before_win_update && proposed_bw > realized_bw) { /* Case A */
		new_bw_to_realize = ihub_bw_info.reserved_bw_kbps;
		update_realized_bw = true;
	} else if (!before_win_update) { /* Case B */
		u32 max_bw = tegra_nvdisp_get_max_pending_bw(dc);
		if (proposed_bw >= max_bw && proposed_bw < realized_bw) {
			if (!tegra_isomgr_reserve(ihub_bw_info.isomgr_handle,
							proposed_bw,
							proposed_latency)) {
				WARN_ONCE(1, "tegra_isomgr_reserve failed\n");
				return;
			}

			ihub_bw_info.reserved_bw_kbps = proposed_bw;
			new_bw_to_realize = proposed_bw;
			update_realized_bw = true;
		}
	}

	if (update_realized_bw) {
		/* Case B */
		if (new_bw_to_realize < realized_bw)
			clk_set_rate(hubclk, proposed_hubclk);

		latency = tegra_isomgr_realize(ihub_bw_info.isomgr_handle);
		if (!latency) {
			WARN_ONCE(!latency, "tegra_isomgr_realize failed\n");
			return;
		}
		tegra_dc_set_latency_allowance(new_bw_to_realize);
		ihub_bw_info.realized_bw_kbps = new_bw_to_realize;

		/* Case A */
		if (new_bw_to_realize > realized_bw)
			clk_set_rate(hubclk, proposed_hubclk);
	}
}

void tegra_nvdisp_init_bandwidth(struct tegra_dc *dc)
{
	/*
	 * Use the max config settings. These values will eventually be adjusted
	 * through IMP, if the client supports it.
	 */

	u32 proposed_bw = NVDISP_BW_MAX_BW_KBPS;
	u32 proposed_latency = NVDISP_BW_TOTAL_MC_LATENCY;
	u32 proposed_hubclk = NVDISP_BW_REQ_HUBCLK_HZ;
	bool before_win_update = true;

	tegra_nvdisp_negotiate_reserved_bw(dc, proposed_bw, proposed_latency);
	tegra_nvdisp_program_bandwidth(dc,
				proposed_bw,
				proposed_latency,
				proposed_hubclk,
				before_win_update);
}

void tegra_nvdisp_clear_bandwidth(struct tegra_dc *dc)
{
	u32 proposed_bw = 0;
	u32 proposed_latency = 1000;
	u32 proposed_hubclk = 0;
	bool before_win_update = false;

	tegra_nvdisp_program_bandwidth(dc,
				proposed_bw,
				proposed_latency,
				proposed_hubclk,
				before_win_update);
}

/*
 * tegra_dc_calc_min_bandwidth - returns the minimum dedicated bw
 *
 * @dc		dc instance
 *
 * @retval	minimum dedicated bw
 */
long tegra_dc_calc_min_bandwidth(struct tegra_dc *dc)
{
	return NVDISP_BW_DEDI_BW_KBPS;
}

int tegra_nvdisp_negotiate_reserved_bw(struct tegra_dc *dc,
				u32 proposed_bw,
				u32 proposed_latency)
{
	/*
	 * There are two possible cases:
	 * A) If the proposed bw is greater than the available bw, fail the
	 *    ioctl.
	 * B) If the proposed bw is greater than the max pending bw, try to
	 *    reserve the proposed bw. Else, the bw that we have currently
	 *    reserved is already sufficient.
	 *
	 * This function is only responsible for reserving bw, NOT realizing it.
	 */

	u32 max_pending_bw = 0;

	if (IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle))
		return -EINVAL;

	if (proposed_bw > ihub_bw_info.available_bw) /* Case A */
		return -E2BIG;

	max_pending_bw = tegra_nvdisp_get_max_pending_bw(dc);
	if (proposed_bw > max_pending_bw) { /* Case B */
		if (!tegra_isomgr_reserve(ihub_bw_info.isomgr_handle,
						proposed_bw,
						proposed_latency))
			return -EINVAL;

		ihub_bw_info.reserved_bw_kbps = proposed_bw;
	}

	return 0;
}

static void tegra_nvdisp_bandwidth_renegotiate(void *p, u32 avail_bw)
{
	struct tegra_dc_bw_data data;
	struct nvdisp_isoclient_bw_info *bw_info = p;

	if (WARN_ONCE(!bw_info, "bw_info is NULL!"))
		return;

	if (IS_ERR_OR_NULL(bw_info->isomgr_handle))
		return;

	mutex_lock(&tegra_nvdisp_lock);
	if (bw_info->available_bw == avail_bw)
		goto unlock_and_exit;

	data.total_bw = tegra_isomgr_get_total_iso_bw();
	data.avail_bw = avail_bw;
	data.resvd_bw = bw_info->reserved_bw_kbps;

	tegra_dc_ext_process_bandwidth_renegotiate(NVDISP_BW_OUTPUT_ID, &data);

	bw_info->available_bw = avail_bw;
unlock_and_exit:
	mutex_unlock(&tegra_nvdisp_lock);
}

/*
 * tegra_nvdisp_isomgr_attach - save reference to ihub_bw_info
 *
 * @dc	dc instance
 */
void tegra_nvdisp_isomgr_attach(struct tegra_dc *dc)
{
	if (WARN_ONCE(!dc, "dc is NULL!"))
		return;

	dc->ihub_bw_info = &ihub_bw_info;
}

/*
 * tegra_nvdisp_isomgr_register - register the IHUB ISO BW client
 *
 * Expected to be called only once in the init path.
 *
 * @client	ISO client id
 * @udedi_bw	dedicated bw needed by client
 *
 * @retval	0 on success
 * @retval	-ENOENT if unable to register with isomgr
 */
int tegra_nvdisp_isomgr_register(enum tegra_iso_client client, u32 udedi_bw)
{
	int err = 0;

	mutex_lock(&tegra_nvdisp_lock);

	ihub_bw_info.isomgr_handle = tegra_isomgr_register(client, udedi_bw,
					tegra_nvdisp_bandwidth_renegotiate,
					&ihub_bw_info);
	if (IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle)) {
		err = -ENOENT;
		goto unlock_and_ret;
	}

	/*
	 * Use the maximum value so that we can reserve as much as needed until
	 * we are told by isomgr to backoff.
	 */
	ihub_bw_info.available_bw = UINT_MAX;

unlock_and_ret:
	mutex_unlock(&tegra_nvdisp_lock);
	return err;
}

/*
 * tegra_nvdisp_isomgr_unregister - unregister the IHUB ISO BW client
 */
void tegra_nvdisp_isomgr_unregister(void)
{
	mutex_lock(&tegra_nvdisp_lock);

	if (!IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle))
		tegra_isomgr_unregister(ihub_bw_info.isomgr_handle);
	memset(&ihub_bw_info, 0, sizeof(ihub_bw_info));

	mutex_unlock(&tegra_nvdisp_lock);
}
#else
void tegra_nvdisp_program_bandwidth(struct tegra_dc *dc, u32 proposed_bw,
	u32 proposed_latency, u32 proposed_hubclk, bool before_win_update) {}
int tegra_nvdisp_negotiate_reserved_bw(struct tegra_dc *dc, u32 proposed_bw,
	u32 proposed_latency)
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
