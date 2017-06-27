/*
 * hda_dc.c: tegra dc hda dc driver.
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION, All rights reserved.
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

#include <video/tegra_hdmi_audio.h>

#include "dc.h"
#include "dc_priv.h"
#include "sor.h"
#include "sor_regs.h"
#include "edid.h"
#include "hdmi2.0.h"
#include "dp.h"
#include "hda_dc.h"

static DEFINE_MUTEX(hda_lock);
static struct tegra_dc_hda_data *hda_inst;

#define to_hdmi(DATA)	((struct tegra_hdmi *)DATA)
#define to_dp(DATA)	((struct tegra_dc_dp_data *)DATA)

static void tegra_hda_get_eld_header(u8 *eld_mem_block,
					struct tegra_dc_hda_data *hda)
{
	struct tegra_edid_hdmi_eld *eld = hda->eld;

	eld->baseline_len = HDMI_ELD_MONITOR_NAME_STR + eld->mnl +
				eld->sad_count * 3 - HDMI_ELD_CEA_EDID_VER_MNL;

	eld_mem_block[HDMI_ELD_VER] = eld->eld_ver << 3;
	eld_mem_block[HDMI_ELD_BASELINE_ELD_LEN] =
			DIV_ROUND_UP(eld->baseline_len, 4);
}

static void tegra_hda_get_eld_baseline(u8 *eld_mem_block,
					struct tegra_dc_hda_data *hda)
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

static void tegra_hda_get_eld_vendor(u8 *eld_mem_block,
					struct tegra_dc_hda_data *hda)
{
	struct tegra_edid_hdmi_eld *eld = hda->eld;
	u32 vendor_block_index = 4 + eld->baseline_len; /* 4 byte header */

	if (!eld->baseline_len)
		dev_err(&hda->dc->ndev->dev,
			"hdm: eld baseline length not populated\n");

	memset(&eld_mem_block[vendor_block_index], 0,
		HDMI_ELD_BUF - vendor_block_index + 1);
}

static int tegra_hda_eld_config(struct tegra_dc_hda_data *hda)
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

	tegra_hda_get_eld_header(eld_mem, hda);
	tegra_hda_get_eld_baseline(eld_mem, hda);
	tegra_hda_get_eld_vendor(eld_mem, hda);

	for (cnt = 0; cnt < HDMI_ELD_BUF; cnt++)
		tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_HDA_ELD_BUFWR,
				NV_SOR_AUDIO_HDA_ELD_BUFWR_INDEX(cnt) |
				NV_SOR_AUDIO_HDA_ELD_BUFWR_DATA(eld_mem[cnt]));

	devm_kfree(&hda->dc->ndev->dev, eld_mem);
	return 0;
}

/* Applicable for dp too, func name still uses hdmi as per generic hda driver */
int tegra_hdmi_setup_hda_presence(int sor_num)
{
	struct tegra_dc_hda_data *hda;

	if (sor_num < 0 || sor_num >= tegra_dc_get_numof_dispsors())
		return -EINVAL;

	if (!hda_inst)
		return -EAGAIN;

	hda = &hda_inst[sor_num];
	if (!hda->valid)
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
		tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_HDA_PRESENCE, 0);

		tegra_hda_eld_config(hda);
		tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_HDA_PRESENCE,
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
EXPORT_SYMBOL(tegra_hdmi_setup_hda_presence);

static void tegra_hdmi_audio_infoframe(struct tegra_dc_hda_data *hda)
{
	if (hda->sink == SINK_HDMI && to_hdmi(hda->client_data)->dvi)
		return;

	/* disable audio infoframe before configuring */
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL, 0);

	if (hda->sink == SINK_HDMI) {
		to_hdmi(hda->client_data)->audio.channel_cnt =
			HDMI_AUDIO_CHANNEL_CNT_2;
		tegra_hdmi_infoframe_pkt_write(to_hdmi(hda->client_data),
				NV_SOR_HDMI_AUDIO_INFOFRAME_HEADER,
				HDMI_INFOFRAME_TYPE_AUDIO,
				HDMI_INFOFRAME_VS_AUDIO,
				HDMI_INFOFRAME_LEN_AUDIO,
				&to_hdmi(hda->client_data)->audio,
				sizeof(to_hdmi(hda->client_data)->audio),
				false);
	}

	/* Send infoframe every frame, checksum hw generated */
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL,
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_ENABLE_YES |
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_OTHER_DISABLE |
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_SINGLE_DISABLE |
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_CHECKSUM_ENABLE);
}

/* HW generated CTS and N */
static void tegra_hdmi_audio_acr(u32 audio_freq, struct tegra_dc_hda_data *hda)
{
#define GET_AVAL(n, fs_hz) ((24000 * n) / (128 * fs_hz / 1000))
	u32 val;

	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_ACR_CTRL, 0x0);

	val = NV_SOR_HDMI_SPARE_HW_CTS_ENABLE |
		NV_SOR_HDMI_SPARE_CTS_RESET_VAL(1) |
		NV_SOR_HDMI_SPARE_ACR_PRIORITY_HIGH;
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_SPARE, val);

	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_ACR_0441_SUBPACK_LOW,
			NV_SOR_HDMI_ACR_SUBPACK_USE_HW_CTS);
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_ACR_0441_SUBPACK_HIGH,
			NV_SOR_HDMI_ACR_SUBPACK_ENABLE);

	val = NV_SOR_HDMI_AUDIO_N_RESET_ASSERT |
		NV_SOR_HDMI_AUDIO_N_LOOKUP_ENABLE;
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_AUDIO_N, val);

	/* N from table 7.1, 7.2, 7.3 hdmi spec v1.4 */
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0320, 4096);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0320,
			GET_AVAL(4096, audio_freq));
	/*For Multiple of 44.1khz source some receiver cannot handle CTS value
	  which is a little far away with golden value,So for the
	  TMDS_clk=148.5Mhz case, we should keep AVAL as default value(20000),
	  and set N= 4704*2 and 4704*4 for the 88.2 and 176.4khz audio case */
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0441, 4704);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0441, 20000);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0882, 9408);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0882, 20000);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_1764, 18816);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_1764, 20000);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0480, 6144);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0480,
			GET_AVAL(6144, audio_freq));
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0960, 12288);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0960,
			GET_AVAL(12288, audio_freq));
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_1920, 24576);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_1920,
			GET_AVAL(24576, audio_freq));

	tegra_sor_write_field_ext(hda->sor, NV_SOR_HDMI_AUDIO_N,
				NV_SOR_HDMI_AUDIO_N_RESET_ASSERT,
				NV_SOR_HDMI_AUDIO_N_RESET_DEASSERT);
#undef GET_AVAL
}

static void tegra_hda_audio_config(u32 audio_freq, u32 audio_src,
					struct tegra_dc_hda_data *hda)
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
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_CTRL, val);

	/* override to advertise HBR capability */
	tegra_sor_writel_ext(hda->sor, NV_PDISP_SOR_AUDIO_SPARE0_0,
		(1 << HDMI_AUDIO_HBR_ENABLE_SHIFT) |
		tegra_sor_readl_ext(hda->sor, NV_PDISP_SOR_AUDIO_SPARE0_0));

	if (hda->sink == SINK_DP) {
		/* program h/vblank sym */
		tegra_sor_write_field_ext(hda->sor,
			NV_SOR_DP_AUDIO_HBLANK_SYMBOLS,
			NV_SOR_DP_AUDIO_HBLANK_SYMBOLS_MASK, cfg->hblank_sym);

		tegra_sor_write_field_ext(hda->sor,
			NV_SOR_DP_AUDIO_VBLANK_SYMBOLS,
			NV_SOR_DP_AUDIO_VBLANK_SYMBOLS_MASK, cfg->vblank_sym);

		val = NV_SOR_DP_AUDIO_CTRL_ENABLE |
			NV_SOR_DP_AUDIO_CTRL_NEW_SETTINGS_TRIGGER |
			NV_SOR_DP_AUDIO_CTRL_CA_SELECT_HW |
			NV_SOR_DP_AUDIO_CTRL_SS_SELECT_HW |
			NV_SOR_DP_AUDIO_CTRL_SF_SELECT_HW |
			NV_SOR_DP_AUDIO_CTRL_CC_SELECT_HW |
			NV_SOR_DP_AUDIO_CTRL_CT_SELECT_HW;
		tegra_sor_writel_ext(hda->sor, NV_SOR_DP_AUDIO_CTRL, val);

		/* make sure to disable overriding channel data */
		tegra_sor_write_field_ext(hda->sor,
			NV_SOR_DP_OUTPUT_CHANNEL_STATUS2,
			NV_SOR_DP_OUTPUT_CHANNEL_STATUS2_OVERRIDE_EN,
			NV_SOR_DP_OUTPUT_CHANNEL_STATUS2_OVERRIDE_DIS);
	}

	if (hda->sink == SINK_HDMI) {
		tegra_hdmi_audio_acr(audio_freq, hda);
		tegra_hdmi_audio_infoframe(hda);
	}
}

/* Applicable for dp too, func name still uses hdmi as per generic hda driver */
int tegra_hdmi_setup_audio_freq_source(unsigned audio_freq,
					unsigned audio_source,
					int sor_num)
{
	bool valid_freq;
	struct tegra_dc_hda_data *hda;

	if (sor_num < 0 || sor_num >= tegra_dc_get_numof_dispsors())
		return -EINVAL;

	if (!hda_inst)
		return -ENODEV;

	hda = &hda_inst[sor_num];
	if (!hda->valid)
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

		tegra_hda_audio_config(audio_freq, audio_source, hda);
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
int tegra_hdmi_audio_null_sample_inject(bool on,
					int sor_num)
{
	struct tegra_dc_hda_data *hda;

	if (sor_num < 0 || sor_num >= tegra_dc_get_numof_dispsors())
		return -EINVAL;

	if (!hda_inst)
		return -ENODEV;

	hda = &hda_inst[sor_num];
	if (!hda->valid)
		return -ENODEV;

	if (on && !hda->null_sample_inject)
		tegra_sor_write_field_ext(hda->sor,
					NV_SOR_AUDIO_CTRL,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN);
	else if (!on && hda->null_sample_inject)
		tegra_sor_write_field_ext(hda->sor,
					NV_SOR_AUDIO_CTRL,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_DIS);

	hda->null_sample_inject = on;
	return 0;
}
EXPORT_SYMBOL(tegra_hdmi_audio_null_sample_inject);

static void tegra_dc_hda_get_clocks(struct tegra_dc *dc,
					struct tegra_dc_hda_data *hda)
{
	struct device_node *np_sor = tegra_dc_get_conn_np(dc);

	if (!np_sor) {
		dev_err(&dc->ndev->dev, "%s: error getting connector np\n",
			__func__);
		return;
	}

	hda->hda_clk = tegra_disp_of_clk_get_by_name(np_sor, "hda");
	if (IS_ERR_OR_NULL(hda->hda_clk)) {
		dev_err(&dc->ndev->dev, "hda: can't get hda clock\n");
		goto err_get_clk;
	}
	hda->hda2codec_clk = tegra_disp_of_clk_get_by_name(np_sor,
							"hda2codec_2x");
	if (IS_ERR_OR_NULL(hda->hda2codec_clk)) {
		dev_err(&dc->ndev->dev,
			"hda: can't get hda2codec clock\n");
		goto err_get_clk;
	}
	hda->hda2hdmi_clk = tegra_disp_of_clk_get_by_name(np_sor, "hda2hdmi");
	if (IS_ERR_OR_NULL(hda->hda2hdmi_clk)) {
		dev_err(&dc->ndev->dev, "hda: can't get hda2hdmi clock\n");
		goto err_get_clk;
	}

	if (hda->sink == SINK_DP) {
#if defined(CONFIG_TEGRA_NVDISPLAY)
		hda->pll_p_clk = tegra_disp_of_clk_get_by_name(np_sor,
						"pllp_out0");
		if (IS_ERR_OR_NULL(hda->pll_p_clk)) {
			dev_err(&dc->ndev->dev,
				"hda: can't get pllp_out0 clock\n");
			goto err_get_clk;
		}

		hda->maud_clk = tegra_disp_of_clk_get_by_name(np_sor, "maud");
		if (IS_ERR_OR_NULL(hda->maud_clk)) {
			dev_err(&hda->dc->ndev->dev,
				"hda: can't get maud clock\n");
			goto err_get_clk;
		}
#else
		hda->pll_p_clk = tegra_disp_of_clk_get_by_name(np_sor,
								"pll_p");
		if (IS_ERR_OR_NULL(hda->pll_p_clk)) {
			dev_err(&dc->ndev->dev,
				"hda: can't get pll_p clock\n");
			goto err_get_clk;
		}
		hda->maud_clk = tegra_disp_of_clk_get_by_name(np_sor, "maud");
		if (IS_ERR_OR_NULL(hda->maud_clk)) {
			dev_err(&hda->dc->ndev->dev,
				"hda: can't get maud clock\n");
			goto err_get_clk;
		}
#endif
		clk_set_parent(hda->maud_clk, hda->pll_p_clk);
	}
	return;

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

	return;
}

static void tegra_dc_hda_put_clocks(struct tegra_dc_hda_data *hda)
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

static void tegra_dc_hda_enable_clocks(struct tegra_dc_hda_data *hda)
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

static void tegra_dc_hda_disable_clocks(struct tegra_dc_hda_data *hda)
{
	if (!hda)
		return;

	if (hda->sink == SINK_DP)
		clk_disable_unprepare(hda->maud_clk);

	clk_disable_unprepare(hda->hda2hdmi_clk);
	clk_disable_unprepare(hda->hda2codec_clk);
	clk_disable_unprepare(hda->hda_clk);
}

void *tegra_hda_set_data(struct tegra_dc *dc, void *data, int sink)
{
	int sor_num;
	struct tegra_dc_sor_data *sor;
	struct tegra_dc_hda_data *hda;

	if (sink == SINK_HDMI) {
		sor = to_hdmi(data)->sor;
		sor_num = sor->ctrl_num;
	} else if (sink == SINK_DP) {
		sor = to_dp(data)->sor;
		sor_num = sor->ctrl_num;
	} else {
		dev_err(&dc->ndev->dev, "%s:invalid sink:%d\n", __func__, sink);
		return NULL;
	}

	mutex_lock(&hda_lock);
	if (!hda_inst) {
		int size = tegra_dc_get_numof_dispsors() * sizeof(*hda_inst);

		hda_inst = kzalloc(size, GFP_KERNEL);
		if (!hda_inst) {
			mutex_unlock(&hda_lock);
			return NULL;
		}
	}

	hda = &hda_inst[sor_num];
	hda->valid = true;
	mutex_unlock(&hda_lock);

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
	tegra_dc_hda_get_clocks(dc, hda);
	tegra_dc_hda_enable_clocks(hda);

	tegra_hdmi_setup_hda_presence(sor_num);
	return (void *)hda;
}

void tegra_hda_reset_data(void *hda_handle)
{
	int i, sor_num;
	bool free_hda_mem = true;
	struct tegra_dc_sor_data *sor;
	struct tegra_dc_hda_data *hda = (struct tegra_dc_hda_data *)hda_handle;

	if (!hda || !hda_inst)
		return;

	if (hda->sink == SINK_HDMI) {
		sor = to_hdmi(hda->client_data)->sor;
		sor_num = sor->ctrl_num;
	} else if (hda->sink == SINK_DP) {
		sor = to_dp(hda->client_data)->sor;
		sor_num = sor->ctrl_num;
	} else {
		pr_err("%s:invalid sink:%d\n", __func__, hda->sink);
		return;
	}

	hda = &hda_inst[sor_num];
	tegra_dc_hda_disable_clocks(hda);
	tegra_dc_hda_put_clocks(hda);

	mutex_lock(&hda_lock);
	hda->valid = false;

	for (i = 0; i < tegra_dc_get_numof_dispsors(); i++) {
		if (hda_inst[i].valid) {
			free_hda_mem = false;
			break;
		}
	}
	if (free_hda_mem) {
		kfree(hda_inst);
		hda_inst = NULL;
	}
	mutex_unlock(&hda_lock);
}
