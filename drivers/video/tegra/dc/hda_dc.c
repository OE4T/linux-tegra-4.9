/*
 * drivers/video/tegra/dc/hda_dc.c
 *
 * Copyright (c) 2015, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
 * Author: Rahul Mittal <rmittal@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/dc.h>
#include <video/tegra_hdmi_audio.h>

#include "dc_priv.h"
#include "sor.h"
#include "sor_regs.h"
#include "edid.h"
#include "hdmi2.0.h"
#include "dp.h"
#include "hda_dc.h"

#define to_hdmi(DATA)	((struct tegra_hdmi *)DATA)
#define to_dp(DATA)	((struct tegra_dc_dp_data *)DATA)

static struct tegra_dc_hda_data *hda;

static void tegra_hda_get_eld_header(u8 *eld_mem_block)
{
	struct tegra_edid_hdmi_eld *eld = hda->eld;

	eld->baseline_len = HDMI_ELD_MONITOR_NAME_STR + eld->mnl +
				eld->sad_count * 3 - HDMI_ELD_CEA_EDID_VER_MNL;

	eld_mem_block[HDMI_ELD_VER] = eld->eld_ver << 3;
	eld_mem_block[HDMI_ELD_BASELINE_ELD_LEN] =
			DIV_ROUND_UP(eld->baseline_len, 4);
}

static void tegra_hda_get_eld_baseline(u8 *eld_mem_block)
{
	struct tegra_edid_hdmi_eld *eld = hda->eld;
	u8 tmp;

	tmp = eld->mnl | (eld->cea_edid_ver << 5);
	eld_mem_block[HDMI_ELD_CEA_EDID_VER_MNL] = tmp;

	tmp = eld->support_hdcp | (eld->support_ai << 1) |
		(eld->conn_type << 2) | (eld->sad_count << 4);
	eld_mem_block[HDMI_ELD_SAD_CNT_CON_TYPE_S_AI_S_HDCP] = tmp;

	eld_mem_block[HDMI_ELD_AUDIO_SYNC_DELAY] = eld->aud_synch_delay;

	eld_mem_block[HDMI_ELD_RLRC_FLRC_RC_RLR_FC_LFE_FLR] = eld->spk_alloc;

	memcpy(&eld_mem_block[HDMI_ELD_PORT_ID], eld->port_id, 8);

	memcpy(&eld_mem_block[HDMI_ELD_MANUFACTURER_NAME],
					eld->manufacture_id, 2);

	memcpy(&eld_mem_block[HDMI_ELD_PRODUCT_CODE], eld->product_id, 2);

	memcpy(&eld_mem_block[HDMI_ELD_MONITOR_NAME_STR],
				eld->monitor_name, eld->mnl);

	memcpy(&eld_mem_block[HDMI_ELD_MONITOR_NAME_STR + eld->mnl],
						eld->sad, eld->sad_count * 3);
}

static void tegra_hda_get_eld_vendor(u8 *eld_mem_block)
{
	struct tegra_edid_hdmi_eld *eld = hda->eld;
	u32 vendor_block_index = 4 + eld->baseline_len; /* 4 byte header */

	if (!eld->baseline_len)
		dev_err(&hda->dc->ndev->dev,
			"hdm: eld baseline length not populated\n");

	memset(&eld_mem_block[vendor_block_index], 0,
		HDMI_ELD_BUF - vendor_block_index + 1);
}

static int tegra_hda_eld_config(void)
{
	u8 *eld_mem;
	int cnt;

	eld_mem = devm_kzalloc(&hda->dc->ndev->dev,
				HDMI_ELD_BUF, GFP_KERNEL);
	if (!eld_mem) {
		dev_warn(&hda->dc->ndev->dev,
			"hdmi: eld memory allocation failed\n");
		return -ENOMEM;
	}

	tegra_hda_get_eld_header(eld_mem);
	tegra_hda_get_eld_baseline(eld_mem);
	tegra_hda_get_eld_vendor(eld_mem);

	for (cnt = 0; cnt < HDMI_ELD_BUF; cnt++)
		tegra_sor_writel(hda->sor, NV_SOR_AUDIO_HDA_ELD_BUFWR,
				NV_SOR_AUDIO_HDA_ELD_BUFWR_INDEX(cnt) |
				NV_SOR_AUDIO_HDA_ELD_BUFWR_DATA(eld_mem[cnt]));

	devm_kfree(&hda->dc->ndev->dev, eld_mem);
	return 0;
}

/* Applicable for dp too, func name still uses hdmi as per generic hda driver */
int tegra_hdmi_setup_hda_presence(void)
{
	if (!hda)
		return -EAGAIN;

	if (hda->sink == SINK_HDMI && to_hdmi(hda->client_data)->dvi)
		return -ENODEV;

	if (*(hda->enabled) && *(hda->eld_valid)) {
		if (hda->sink == SINK_HDMI) {
			tegra_dc_unpowergate_locked(hda->dc);
			tegra_hdmi_get(hda->dc);
		}
		tegra_dc_io_start(hda->dc);

		/* remove hda presence while setting up eld */
		tegra_sor_writel(hda->sor, NV_SOR_AUDIO_HDA_PRESENCE, 0);

		tegra_hda_eld_config();
		tegra_sor_writel(hda->sor, NV_SOR_AUDIO_HDA_PRESENCE,
				NV_SOR_AUDIO_HDA_PRESENCE_ELDV(1) |
				NV_SOR_AUDIO_HDA_PRESENCE_PD(1));

		tegra_dc_io_end(hda->dc);
		if (hda->sink == SINK_HDMI) {
			tegra_hdmi_put(hda->dc);
			tegra_dc_powergate_locked(hda->dc);
		}
		return 0;
	}

	return -ENODEV;
}

static void tegra_hdmi_audio_infoframe(void)
{
	if (hda->sink == SINK_HDMI && to_hdmi(hda->client_data)->dvi)
		return;

	/* disable audio infoframe before configuring */
	tegra_sor_writel(hda->sor, NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL, 0);

	if (hda->sink == SINK_HDMI) {
		to_hdmi(hda->client_data)->audio.channel_cnt =
			HDMI_AUDIO_CHANNEL_CNT_2;
		tegra_hdmi_infoframe_pkt_write(to_hdmi(hda->client_data),
				NV_SOR_HDMI_AUDIO_INFOFRAME_HEADER,
				HDMI_INFOFRAME_TYPE_AUDIO,
				HDMI_INFOFRAME_VS_AUDIO,
				HDMI_INFOFRAME_LEN_AUDIO,
				&to_hdmi(hda->client_data)->audio,
				sizeof(to_hdmi(hda->client_data)->audio));
	}

	/* Send infoframe every frame, checksum hw generated */
	tegra_sor_writel(hda->sor, NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL,
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_ENABLE_YES |
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_OTHER_DISABLE |
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_SINGLE_DISABLE |
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_CHECKSUM_ENABLE);
}

/* HW generated CTS and N */
static void tegra_hdmi_audio_acr(u32 audio_freq)
{
#define GET_AVAL(n, fs_hz) ((24000 * n) / (128 * fs_hz / 1000))
	u32 val;

	tegra_sor_writel(hda->sor, NV_SOR_HDMI_ACR_CTRL, 0x0);

	val = NV_SOR_HDMI_SPARE_HW_CTS_ENABLE |
		NV_SOR_HDMI_SPARE_CTS_RESET_VAL(1) |
		NV_SOR_HDMI_SPARE_ACR_PRIORITY_HIGH;
	tegra_sor_writel(hda->sor, NV_SOR_HDMI_SPARE, val);

	tegra_sor_writel(hda->sor, NV_SOR_HDMI_ACR_0441_SUBPACK_LOW,
			NV_SOR_HDMI_ACR_SUBPACK_USE_HW_CTS);
	tegra_sor_writel(hda->sor, NV_SOR_HDMI_ACR_0441_SUBPACK_HIGH,
			NV_SOR_HDMI_ACR_SUBPACK_ENABLE);

	val = NV_SOR_HDMI_AUDIO_N_RESET_ASSERT |
		NV_SOR_HDMI_AUDIO_N_LOOKUP_ENABLE;
	tegra_sor_writel(hda->sor, NV_SOR_HDMI_AUDIO_N, val);

	/* N from table 7.1, 7.2, 7.3 hdmi spec v1.4 */
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_NVAL_0320, 4096);
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_AVAL_0320,
			GET_AVAL(4096, audio_freq));
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_NVAL_0441, 6272);
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_AVAL_0441,
			GET_AVAL(6272, audio_freq));
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_NVAL_0882, 12544);
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_AVAL_0882,
			GET_AVAL(12544, audio_freq));
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_NVAL_1764, 25088);
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_AVAL_1764,
			GET_AVAL(25088, audio_freq));
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_NVAL_0480, 6144);
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_AVAL_0480,
			GET_AVAL(6144, audio_freq));
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_NVAL_0960, 12288);
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_AVAL_0960,
			GET_AVAL(12288, audio_freq));
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_NVAL_1920, 24576);
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_AVAL_1920,
			GET_AVAL(24576, audio_freq));

	tegra_sor_write_field(hda->sor, NV_SOR_HDMI_AUDIO_N,
				NV_SOR_HDMI_AUDIO_N_RESET_ASSERT,
				NV_SOR_HDMI_AUDIO_N_RESET_DEASSERT);
#undef GET_AVAL
}

static void tegra_hda_audio_config(u32 audio_freq, u32 audio_src)
{
	u32 val;
	struct tegra_dc_dp_link_config *cfg = NULL;

	if (hda->sink == SINK_DP)
		cfg = &to_dp(hda->client_data)->link_cfg;

	if (hda->sink == SINK_HDMI && to_hdmi(hda->client_data)->dvi)
		return;

	/* hda is the only audio source */
	val = NV_SOR_AUDIO_CTRL_AFIFO_FLUSH |
		NV_SOR_AUDIO_CTRL_SRC_HDA;
	if (hda->null_sample_inject)
		val |= NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN;
	tegra_sor_writel(hda->sor, NV_SOR_AUDIO_CTRL, val);

	/* override to advertise HBR capability */
	tegra_sor_writel(hda->sor, NV_PDISP_SOR_AUDIO_SPARE0_0,
		(1 << HDMI_AUDIO_HBR_ENABLE_SHIFT) |
		tegra_sor_readl(hda->sor, NV_PDISP_SOR_AUDIO_SPARE0_0));

	if (hda->sink == SINK_DP) {
		tegra_dc_sor_set_dp_mode(hda->sor, cfg);
		val = NV_SOR_DP_AUDIO_CTRL_ENABLE |
			NV_SOR_DP_AUDIO_CTRL_NEW_SETTINGS_TRIGGER;
		tegra_sor_writel(hda->sor, NV_SOR_DP_AUDIO_CTRL, val);
	}

	if (hda->sink == SINK_HDMI) {
		tegra_hdmi_audio_acr(audio_freq);
		tegra_hdmi_audio_infoframe();
	}
}

/* Applicable for dp too, func name still uses hdmi as per generic hda driver */
int tegra_hdmi_setup_audio_freq_source(unsigned audio_freq,
					unsigned audio_source)
{
	bool valid_freq;

	if (!hda)
		return -ENODEV;

	if (hda->sink == SINK_HDMI && to_hdmi(hda->client_data)->dvi)
		return -ENODEV;

	valid_freq = AUDIO_FREQ_32K == audio_freq ||
			AUDIO_FREQ_44_1K == audio_freq ||
			AUDIO_FREQ_48K == audio_freq ||
			AUDIO_FREQ_88_2K == audio_freq ||
			AUDIO_FREQ_96K == audio_freq ||
			AUDIO_FREQ_176_4K == audio_freq ||
			AUDIO_FREQ_192K == audio_freq;

	if (valid_freq) {
		tegra_dc_io_start(hda->dc);
		if (hda->sink == SINK_HDMI)
			tegra_hdmi_get(hda->dc);

		tegra_hda_audio_config(audio_freq, audio_source);
		hda->audio_freq = audio_freq;

		if (hda->sink == SINK_HDMI)
			tegra_hdmi_put(hda->dc);
		tegra_dc_io_end(hda->dc);
	} else {
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(tegra_hdmi_setup_audio_freq_source);

/* Applicable for dp too, func name still uses hdmi as per generic hda driver */
int tegra_hdmi_audio_null_sample_inject(bool on)
{
	if (!hda)
		return -ENODEV;

	if (on && !hda->null_sample_inject)
		tegra_sor_write_field(hda->sor,
					NV_SOR_AUDIO_CTRL,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN);
	else if (!on && hda->null_sample_inject)
		tegra_sor_write_field(hda->sor,
					NV_SOR_AUDIO_CTRL,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_DIS);

	hda->null_sample_inject = on;
	return 0;
}
EXPORT_SYMBOL(tegra_hdmi_audio_null_sample_inject);

static void tegra_dc_hda_get_clocks(void)
{
	if (!hda)
		return;

	hda->hda_clk = clk_get_sys("tegra30-hda", "hda");
	if (IS_ERR_OR_NULL(hda->hda_clk)) {
		dev_err(&hda->dc->ndev->dev, "hda: can't get hda clock\n");
		goto err_get_clk;
	}

	hda->hda2codec_clk = clk_get_sys("tegra30-hda", "hda2codec_2x");
	if (IS_ERR_OR_NULL(hda->hda2codec_clk)) {
		dev_err(&hda->dc->ndev->dev,
			"hda: can't get hda2codec clock\n");
		goto err_get_clk;
	}

	hda->hda2hdmi_clk = clk_get_sys("tegra30-hda", "hda2hdmi");
	if (IS_ERR_OR_NULL(hda->hda2hdmi_clk)) {
		dev_err(&hda->dc->ndev->dev, "hda: can't get hda2hdmi clock\n");
		goto err_get_clk;
	}

	if (hda->sink == SINK_DP) {
		hda->pll_p_clk = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(hda->pll_p_clk)) {
			dev_err(&hda->dc->ndev->dev,
				"hda: can't get pll_p clock\n");
			goto err_get_clk;
		}

		hda->maud_clk = tegra_get_clock_by_name("maud");
		if (IS_ERR_OR_NULL(hda->maud_clk)) {
			dev_err(&hda->dc->ndev->dev,
				"hda: can't get maud clock\n");
			goto err_get_clk;
		}
		clk_set_parent(hda->maud_clk, hda->pll_p_clk);
	}

err_get_clk:
	if (!IS_ERR_OR_NULL(hda->hda_clk))
		clk_put(hda->hda_clk);
	if (!IS_ERR_OR_NULL(hda->hda2codec_clk))
		clk_put(hda->hda2codec_clk);
	if (!IS_ERR_OR_NULL(hda->hda2hdmi_clk))
		clk_put(hda->hda2hdmi_clk);
	if (hda->sink == SINK_DP) {
		if (!IS_ERR_OR_NULL(hda->pll_p_clk))
			clk_put(hda->pll_p_clk);
		if (!IS_ERR_OR_NULL(hda->maud_clk))
			clk_put(hda->maud_clk);
	}
}

static void tegra_dc_hda_put_clocks(void)
{
	if (!hda)
		return;

	if (!IS_ERR_OR_NULL(hda->hda_clk))
		clk_put(hda->hda_clk);
	if (!IS_ERR_OR_NULL(hda->hda2codec_clk))
		clk_put(hda->hda2codec_clk);
	if (!IS_ERR_OR_NULL(hda->hda2hdmi_clk))
		clk_put(hda->hda2hdmi_clk);

	if (hda->sink == SINK_DP) {
		if (!IS_ERR_OR_NULL(hda->pll_p_clk))
			clk_put(hda->pll_p_clk);
		if (!IS_ERR_OR_NULL(hda->maud_clk))
			clk_put(hda->maud_clk);
	}
}

static void tegra_dc_hda_enable_clocks(void)
{
	if (!hda)
		return;

	clk_prepare_enable(hda->hda_clk);
	clk_prepare_enable(hda->hda2codec_clk);
	clk_prepare_enable(hda->hda2hdmi_clk);

	if (hda->sink == SINK_DP) {
		clk_set_rate(hda->maud_clk, 102000000);
		clk_prepare_enable(hda->maud_clk);
	}
}

static void tegra_dc_hda_disable_clocks(void)
{
	if (!hda)
		return;

	if (hda->sink == SINK_DP)
		clk_disable_unprepare(hda->maud_clk);

	clk_disable_unprepare(hda->hda2hdmi_clk);
	clk_disable_unprepare(hda->hda2codec_clk);
	clk_disable_unprepare(hda->hda_clk);
}

void tegra_hda_set_data(void *data, int sink)
{
	hda = kzalloc(sizeof(struct tegra_dc_hda_data), GFP_KERNEL);
	hda->client_data = data;
	hda->sink = sink;

	if (hda->sink == SINK_HDMI) {
		hda->sor = to_hdmi(hda->client_data)->sor;
		hda->dc = to_hdmi(hda->client_data)->dc;
		hda->eld = &to_hdmi(hda->client_data)->eld;
		hda->enabled = &to_hdmi(hda->client_data)->enabled;
		hda->eld_valid = &to_hdmi(hda->client_data)->eld_valid;
	}

	if (hda->sink == SINK_DP) {
		hda->sor = to_dp(hda->client_data)->sor;
		hda->dc = to_dp(hda->client_data)->dc;
		hda->eld = &to_dp(hda->client_data)->hpd_data.eld;
		hda->enabled = &to_dp(hda->client_data)->enabled;
		hda->eld_valid =
			&to_dp(hda->client_data)->hpd_data.eld_retrieved;
		hda->eld->conn_type = 1; /* For DP, conn_type = 1 */
	}

	hda->null_sample_inject = false;
	tegra_dc_hda_get_clocks();
	tegra_dc_hda_enable_clocks();
}

void tegra_hda_reset_data(void)
{
	tegra_dc_hda_disable_clocks();
	tegra_dc_hda_put_clocks();
	kfree(hda);
}
