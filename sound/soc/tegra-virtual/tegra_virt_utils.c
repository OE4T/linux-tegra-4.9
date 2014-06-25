/*
 * tegra_virt_utils.c - Utilities for tegra124_virt_apbif_slave
 *
 * Copyright (c) 2011-2014 NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/io.h>
#include "tegra_virt_utils.h"
#include <linux/string.h>

const resource_size_t apbif_phy_base[MAX_APBIF_IDS] = {
	TEGRA_APBIF_BASE_ADR(0),
	TEGRA_APBIF_BASE_ADR(1),
	TEGRA_APBIF_BASE_ADR(2),
	TEGRA_APBIF_BASE_ADR(3),
	TEGRA_APBIF_BASE_ADR(4),
	TEGRA_APBIF_BASE_ADR(5),
	TEGRA_APBIF_BASE_ADR(6),
	TEGRA_APBIF_BASE_ADR(7),
	TEGRA_APBIF_BASE_ADR(8),
	TEGRA_APBIF_BASE_ADR(9),
};

const resource_size_t amx_phy_base[AMX_MAX_INSTANCE] = {
	TEGRA_AMX_BASE(0),
	TEGRA_AMX_BASE(1),
};

const resource_size_t audio_amx_offset[AMX_TOTAL_CHANNEL] = {
	TEGRA_AUDIO_AMX_OFFSET(0),
	TEGRA_AUDIO_AMX_OFFSET(1),
	TEGRA_AUDIO_AMX_OFFSET(2),
	TEGRA_AUDIO_AMX_OFFSET(3),
	TEGRA_AUDIO_AMX_OFFSET(4),
	TEGRA_AUDIO_AMX_OFFSET(5),
	TEGRA_AUDIO_AMX_OFFSET(6),
	TEGRA_AUDIO_AMX_OFFSET(7),
};

/*
	reg_write: write value to address
	base_address: base_address
	reg:offset
	val:value to be written
 */
void reg_write(void *base_address,
				unsigned int reg, unsigned int val)
{
	writel(val, base_address+reg);
}

/*
	reg_read: read value from address
	base_address: base_address
	reg:offset
	return value read from the address
 */
unsigned int reg_read(void *base_address, unsigned int reg)
{
	unsigned int val = 0;

	val = readl(base_address + reg);

	return val;
}

/*
	create_ioremap: create ioremapped address for a particular device.
	slave_remap_add: structure storing ioremapped addresses.
 */
int create_ioremap(struct device *dev, struct slave_remap_add *phandle)
{
	int i;
	memset(phandle->apbif_base, 0,
		sizeof(phandle->apbif_base[0])*MAX_APBIF_IDS);
	memset(phandle->amx_base, 0,
		sizeof(phandle->amx_base[0])*AMX_MAX_INSTANCE);
	phandle->audio_amx_base = NULL;

	for (i = 0; i < MAX_APBIF_IDS; i++) {
		phandle->apbif_base[i] = devm_ioremap(dev, apbif_phy_base[i],
			TEGRA_ABPIF_UNIT_SIZE);
		if (phandle->apbif_base[i] == NULL)
			goto remap_fail;
	}

	for (i = 0; i < AMX_MAX_INSTANCE; i++) {
		phandle->amx_base[i] = devm_ioremap(dev, amx_phy_base[i],
			TEGRA_AMX_UNIT_SIZE);
		if (phandle->amx_base[i] == NULL)
			goto remap_fail;
	}

	phandle->audio_amx_base = devm_ioremap(dev, TEGRA_AUDIO_BASE,
					TEGRA_AUDIO_SIZE);
	if (phandle->audio_amx_base == NULL)
		goto remap_fail;

	return 0;

remap_fail:
	remove_ioremap(dev, phandle);
	return -1;
}

/*
remove_ioremap: unmap ioremapped addresses.
*/
void remove_ioremap(struct device *dev, struct slave_remap_add *phandle)
{
	int i;
	for (i = 0; i < MAX_APBIF_IDS; i++) {
		if (phandle->apbif_base[i] != NULL) {
			devm_iounmap(dev,
				(void __iomem *)(phandle->apbif_base[i]));
			phandle->apbif_base[i] = NULL;
		}
	}

	for (i = 0; i < AMX_MAX_INSTANCE; i++) {
		if (phandle->amx_base[i] != NULL) {
			devm_iounmap(dev,
				(void __iomem *)(phandle->amx_base[i]));
			phandle->amx_base[i] = NULL;
		}
	}

	if (phandle->audio_amx_base != NULL) {
		devm_iounmap(dev,
			(void __iomem *)(phandle->audio_amx_base));
		phandle->audio_amx_base = NULL;
	}

}

/*
tegra_find_amx_channel: Find amx_channel index based on
  xbar register value.
	value contains xbar register value for a given receiver
	amx	*/
static unsigned int tegra_find_amx_channel(unsigned int value)
{
	unsigned int  apbif_id = MAX_APBIF_IDS;
	switch (value) {
	case APBIF_TX0:
		apbif_id = APBIF_ID_0;
		break;
	case APBIF_TX1:
		apbif_id = APBIF_ID_1;
		break;
	case APBIF_TX2:
		apbif_id = APBIF_ID_2;
		break;
	case APBIF_TX3:
		apbif_id = APBIF_ID_3;
		break;
	case APBIF_TX4:
		apbif_id = APBIF_ID_4;
		break;
	case APBIF_TX5:
		apbif_id = APBIF_ID_5;
		break;
	case APBIF_TX6:
		apbif_id = APBIF_ID_6;
		break;
	case APBIF_TX7:
		apbif_id = APBIF_ID_7;
		break;
	case APBIF_TX8:
		apbif_id = APBIF_ID_8;
		break;
	case APBIF_TX9:
		apbif_id = APBIF_ID_9;
		break;
	case I2S0_TX0: /* TBD */
	case I2S1_TX0: /* TBD */
	case I2S2_TX0: /* TBD */
	case I2S3_TX0: /* TBD */
	case I2S4_TX0: /* TBD */
	case DAM0_TX0: /* TBD */
	case DAM1_TX0: /* TBD */
	case DAM2_TX0: /* TBD */
	case SPDIF_TX0: /* TBD */
	case SPDIF_TX1: /* TBD */
	case AMX0_TX0: /* TBD */
	case ADX0_TX0: /* TBD */
	case ADX0_TX1: /* TBD */
	case ADX0_TX2: /* TBD */
	case ADX0_TX3: /* TBD */
		pr_warn("AMX connection is not supported\n");
		break;
	default:
		break;
	}
	return apbif_id;
}

/*
tegra_find_amx_info: Find amx index and particular
amx channel index used by apbif_id under consideration
	 */
void tegra_find_amx_info(unsigned long arg)
{

	int amx_idx, ch_idx;
	unsigned int value;
	unsigned int reg;
	struct tegra_virt_utils_data *data =
			(struct tegra_virt_utils_data *) arg;
	struct slave_remap_add *phandle = &(data->phandle);

	for (amx_idx = 0; amx_idx < AMX_MAX_INSTANCE; amx_idx++) {
		for (ch_idx = 0; ch_idx < AMX_MAX_CHANNEL; ch_idx++) {
			reg =
			audio_amx_offset[AMX_MAX_CHANNEL*amx_idx + ch_idx];
			value = reg_read(phandle->audio_amx_base, reg);
			if (data->apbif_id == tegra_find_amx_channel(value))
				break;
		}
		if (ch_idx < AMX_MAX_CHANNEL)
			break;
	}

	data->amx_id = amx_idx;
	data->amx_in_channel = ch_idx;
	return;
}
