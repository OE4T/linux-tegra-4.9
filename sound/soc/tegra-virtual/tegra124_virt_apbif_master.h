/*
 * tegra124_virt_apbif_master.h - Header file for
 *		tegra124_virt_apbif_master driver
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

#ifndef __TEGRA124_VIRT_APBIF_MASTER_H__
#define __TEGRA124_VIRT_APBIF_MASTER_H__

int tegra30_apbif_i2s_rx_fifo_is_enabled(int i2s_id);
int tegra30_apbif_i2s_tx_fifo_is_enabled(int i2s_id);
int tegra30_apbif_i2s_rx_fifo_is_empty(int i2s_id);
int tegra30_apbif_i2s_tx_fifo_is_empty(int i2s_id);

struct tegra124_virt_apbif_soc_data {
		unsigned int num_ch;
		unsigned int clk_list_mask;
		void (*set_audio_cif)(struct regmap *map,
			unsigned int reg,
			struct tegra30_xbar_cif_conf *cif_conf);
};

struct tegra124_virt_apbif {
	struct clk *clk;
	/* regmap for APBIF */
	struct regmap *regmap[2];
	struct tegra_alt_pcm_dma_params *capture_dma_data;
	struct tegra_alt_pcm_dma_params *playback_dma_data;
	const struct tegra124_virt_apbif_soc_data *soc_data;
};

#endif
