/*
 * tegra_asoc_util_virt_alt.c - Tegra xbar dai link for machine drivers
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/module.h>
#include <linux/platform_device.h>

#include "tegra_virt_alt_ivc.h"
#include "tegra_asoc_util_virt_alt.h"


const int tegra186_arad_mux_value[] = {
	-1, /* None */
	0, 1, 2, 3, 4, 5,	/* I2S1~6 */
	28, 29, 30, 31,	/* SPDIF_RX1,2 & SPDIF_TX1,2 */
};

const char * const tegra186_arad_mux_text[] = {
	"None",
	"I2S1",
	"I2S2",
	"I2S3",
	"I2S4",
	"I2S5",
	"I2S6",
	"SPDIF1_RX1",
	"SPDIF1_RX2",
	"SPDIF1_TX1",
	"SPDIF1_TX2",
};

const char * const tegra186_asrc_ratio_source_text[] = {
	"ARAD",
	"SW",
};

int tegra_virt_t210mixer_get_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_get_gain);

int tegra_virt_t210mixer_set_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_RX_GAIN;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.rx_idx = (int) reg;
	msg.params.amixer_info.gain =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_set_gain);

int tegra_virt_t210mixer_get_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_GET_TX_ADDER_CONFIG;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.adder_idx = (((int) reg) >>
				MIXER_CONFIG_SHIFT_VALUE) & 0xFFFF;
	msg.params.amixer_info.adder_rx_idx = ((int) reg) & 0xFFFF;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_receive\n", __func__);

	ucontrol->value.integer.value[0] =
		msg.params.amixer_info.adder_rx_idx_enable;

	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_get_adder_config);

int tegra_virt_t210mixer_set_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_TX_ADDER_CONFIG;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.adder_idx = (((int) reg) >>
				MIXER_CONFIG_SHIFT_VALUE) & 0xFFFF;
	msg.params.amixer_info.adder_rx_idx = ((int) reg) & 0xFFFF;
	msg.params.amixer_info.adder_rx_idx_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_set_adder_config);

int tegra_virt_t210mixer_get_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_GET_ENABLE;
	msg.params.amixer_info.id = 0;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_receive\n", __func__);

	ucontrol->value.integer.value[0] = msg.params.amixer_info.enable;

	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_get_enable);
int tegra_virt_t210mixer_set_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_ENABLE;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_set_enable);

int tegra_virt_t210sfc_get_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_GET_IN_FREQ;
	msg.params.sfc_info.id = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	ucontrol->value.integer.value[0] = msg.params.sfc_info.in_freq;

	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210sfc_get_in_freq);
int tegra_virt_t210sfc_set_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_SET_IN_FREQ;
	msg.params.sfc_info.id = reg;
	msg.params.sfc_info.in_freq =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210sfc_set_in_freq);

int tegra_virt_t210sfc_get_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_GET_OUT_FREQ;
	msg.params.sfc_info.id = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	ucontrol->value.integer.value[0] = msg.params.sfc_info.out_freq;

	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210sfc_get_out_freq);
int tegra_virt_t210sfc_set_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_SET_OUT_FREQ;
	msg.params.sfc_info.id = reg;
	msg.params.sfc_info.out_freq =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210sfc_set_out_freq);

int tegra186_virt_asrc_get_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mreg_control *mc =
		(struct soc_mreg_control *)kcontrol->private_value;
	unsigned int reg = mc->regbase;
	int err;
	uint64_t val;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	val = (uint64_t) msg.params.asrc_info.int_ratio << 32;
	val &= 0xffffffff00000000ULL;
	val |= msg.params.asrc_info.frac_ratio;
	ucontrol->value.integer64.value[0] = val;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_ratio);

int tegra186_virt_asrc_set_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mreg_control *mc =
		(struct soc_mreg_control *)kcontrol->private_value;
	unsigned int reg = mc->regbase;
	int err;
	uint64_t val;
	struct nvaudio_ivc_msg msg;

	val = ucontrol->value.integer64.value[0];

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.int_ratio =
		(val >> 32) & 0xffffffffULL;
	msg.params.asrc_info.frac_ratio =
		(val & 0xffffffffULL);

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_ratio);

int tegra186_virt_asrc_get_int_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_INT_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.int_ratio;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_int_ratio);

int tegra186_virt_asrc_set_int_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_INT_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.int_ratio =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_int_ratio);
int tegra186_virt_asrc_get_frac_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mreg_control *mc =
		(struct soc_mreg_control *)kcontrol->private_value;
	unsigned int reg = mc->regbase;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_FRAC_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.frac_ratio;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_frac_ratio);

int tegra186_virt_asrc_set_frac_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mreg_control *mc =
		(struct soc_mreg_control *)kcontrol->private_value;
	unsigned int reg = mc->regbase;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_FRAC_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.frac_ratio =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_frac_ratio);

int tegra186_virt_asrc_get_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_RATIO_SOURCE;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.ratio_source;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_ratio_source);

int tegra186_virt_asrc_set_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_RATIO_SOURCE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.ratio_source =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_ratio_source);

int tegra186_virt_asrc_get_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_STREAM_ENABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.stream_enable;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_stream_enable);

int tegra186_virt_asrc_set_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_STREAM_ENABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.stream_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_stream_enable);

int tegra186_virt_asrc_get_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_HWCOMP_DISABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.hwcomp_disable;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_hwcomp_disable);

int tegra186_virt_asrc_set_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_HWCOMP_DISABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.hwcomp_disable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_hwcomp_disable);

int tegra186_virt_asrc_get_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_INPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	ucontrol->value.integer.value[0] = msg.params.sfc_info.out_freq;

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.input_threshold;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_input_threshold);

int tegra186_virt_asrc_set_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_INPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.input_threshold =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_input_threshold);

int tegra186_virt_asrc_get_output_threshold(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_OUTPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] =
			msg.params.asrc_info.output_threshold;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_output_threshold);

int tegra186_virt_asrc_set_output_threshold(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_OUTPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.output_threshold =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_output_threshold);

int tegra_virt_t210_amx_get_input_stream_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210_amx_get_input_stream_enable);

int tegra_virt_t210_amx_set_input_stream_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMX_SET_INPUT_STREAM_ENABLE;
	msg.params.amx_info.amx_id = (((int) reg) >>
				MIXER_CONFIG_SHIFT_VALUE) & 0xFFFF;
	msg.params.amx_info.amx_stream_id = ((int) reg) & 0xFFFF;
	msg.params.amx_info.amx_stream_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210_amx_set_input_stream_enable);


















int tegra186_virt_arad_get_lane_source(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err, i;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_GET_LANE_SRC;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg % NUM_ARAD_LANES;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	/* numerator reg 0 to 5, denominator reg 6 to 11 */
	if (reg/NUM_ARAD_LANES) {
		for (i = 0; i < NUM_ARAD_SOURCES; i++) {
			if (e->values[i] ==
					msg.params.arad_info.den_source)
				break;
		}
		ucontrol->value.integer.value[0] = i;
	} else {
		for (i = 0; i < NUM_ARAD_SOURCES; i++) {
			if (e->values[i] ==
					msg.params.arad_info.num_source)
				break;
		}
		ucontrol->value.integer.value[0] = i;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_get_lane_source);

int tegra186_virt_arad_set_lane_source(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;
	int source;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_SET_LANE_SRC;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg % NUM_ARAD_LANES;

	/* numerator reg 0 to 5, denominator reg 6 to 11 */
	source = e->values[ucontrol->value.integer.value[0]];
	if (reg/NUM_ARAD_LANES) {
		msg.params.arad_info.num_source = -1;
		msg.params.arad_info.den_source = source;
	} else {
		msg.params.arad_info.num_source = source;
		msg.params.arad_info.den_source = -1;
	}

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_set_lane_source);

int tegra186_virt_arad_get_lane_prescalar(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_GET_PRESCALAR;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg % NUM_ARAD_LANES;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	/* numerator reg 0 to 5, denominator reg 6 to 11 */
	if (reg/NUM_ARAD_LANES)
		ucontrol->value.integer.value[0] =
			msg.params.arad_info.den_prescalar;
	else
		ucontrol->value.integer.value[0] =
			msg.params.arad_info.num_prescalar;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_get_lane_prescalar);

int tegra186_virt_arad_set_lane_prescalar(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_SET_PRESCALAR;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg % NUM_ARAD_LANES;

	/* numerator reg 0 to 5, denominator reg 6 to 11 */
	if (reg/NUM_ARAD_LANES) {
		msg.params.arad_info.num_prescalar = -1;
		msg.params.arad_info.den_prescalar =
			ucontrol->value.integer.value[0];
	} else {
		msg.params.arad_info.num_prescalar =
			ucontrol->value.integer.value[0];
		msg.params.arad_info.den_prescalar = -1;
	}

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_set_lane_prescalar);

int tegra186_virt_arad_get_lane_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_GET_LANE_ENABLE;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] =
			msg.params.arad_info.lane_enable;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_get_lane_enable);

int tegra186_virt_arad_set_lane_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_SET_LANE_ENABLE;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg;
	msg.params.arad_info.lane_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_set_lane_enable);

int tegra186_virt_arad_get_lane_ratio(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	uint64_t val;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_GET_LANE_RATIO;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	val = (uint64_t)msg.params.arad_info.int_ratio << 32;
	val &= 0xffffffff00000000ULL;
	val |= (uint64_t)msg.params.arad_info.frac_ratio;
	ucontrol->value.integer64.value[0] = val;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_get_lane_ratio);

int tegra_virt_i2s_get_loopback_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_I2S_GET_LOOPBACK_ENABLE;
	msg.params.i2s_info.i2s_id = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] =
			msg.params.i2s_info.i2s_loopback_enable;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_i2s_get_loopback_enable);

int tegra_virt_i2s_set_loopback_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_I2S_SET_LOOPBACK_ENABLE;
	msg.params.i2s_info.i2s_id = reg;
	msg.params.i2s_info.i2s_loopback_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_i2s_set_loopback_enable);

MODULE_AUTHOR("Dipesh Gandhi <dipeshg@nvidia.com>");
MODULE_DESCRIPTION("Tegra Virt ASoC utility code");
MODULE_LICENSE("GPL");
