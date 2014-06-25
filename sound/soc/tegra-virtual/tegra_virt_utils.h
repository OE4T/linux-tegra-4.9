/*
 * tegra_virt_utils.h - Utilities for tegra124_virt_apbif_slave
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
#ifndef __TEGRA_VIRT_UTILS_H__
#define __TEGRA_VIRT_UTILS_H__

#define TEGRA_APBIF0_BASE		0x70300000
#define TEGRA_APBIF4_BASE		0x70300200
#define TEGRA_AMX0_BASE			0x70303000
#define TEGRA_AUDIO_BASE		0x70300800
#define TEGRA_AUDIO_AMX0_OFFSET		0x5c
#define TEGRA_AUDIO_AMX1_OFFSET		0x78
#define TEGRA_AUDIO_AMX_UNIT_SIZE		0x004
#define TEGRA_AUDIO_SIZE				0x200

/* define the unit sizes */
#define TEGRA_ABPIF_UNIT_SIZE	0x20
#define TEGRA_AMX_UNIT_SIZE		0x100

#define TEGRA_APBIF_BASE_ADR(id)	\
	(resource_size_t)((id < 4) ? \
	(TEGRA_APBIF0_BASE + id * TEGRA_ABPIF_UNIT_SIZE) : \
	(TEGRA_APBIF4_BASE + (id - 4) * TEGRA_ABPIF_UNIT_SIZE))

#define TEGRA_AMX_BASE(id)		\
	(resource_size_t)(TEGRA_AMX0_BASE + id * TEGRA_AMX_UNIT_SIZE)
#define TEGRA_AUDIO_AMX_OFFSET(id)	\
	(resource_size_t)((id < 4) ? \
		(TEGRA_AUDIO_AMX0_OFFSET + id * TEGRA_AUDIO_AMX_UNIT_SIZE) : \
			(TEGRA_AUDIO_AMX1_OFFSET + \
			 (id - 4) * TEGRA_AUDIO_AMX_UNIT_SIZE))

#define AMX_MAX_CHANNEL   4
#define AMX_TOTAL_CHANNEL 8

/* Mask to find AMX Connections */
#define APBIF_TX0 0x00000001
#define APBIF_TX1 0x00000002
#define APBIF_TX2 0x00000004
#define APBIF_TX3 0x00000008
#define I2S0_TX0  0x00000010
#define I2S1_TX0  0x00000020
#define I2S2_TX0  0x00000040
#define I2S3_TX0  0x00000080
#define I2S4_TX0  0x00000100
#define DAM0_TX0  0x00000200
#define DAM1_TX0  0x00000400
#define DAM2_TX0  0x00000800
#define SPDIF_TX0 0x00001000
#define SPDIF_TX1 0x00002000
#define APBIF_TX4 0x00004000
#define APBIF_TX5 0x00008000
#define APBIF_TX6 0x00010000
#define APBIF_TX7 0x00020000
#define APBIF_TX8 0x00040000
#define APBIF_TX9 0x00080000
#define AMX0_TX0  0x00100000
#define ADX0_TX0  0x00200000
#define ADX0_TX1  0x00400000
#define ADX0_TX2  0x00800000
#define ADX0_TX3  0x01000000

/* AHUB modules to program */
enum {
	APBIF = 0,
	AMX,
	AUDIO_AMX
};

/* AMX ids */
enum {
	AMX_INSTANCE_0 = 0,
	AMX_INSTANCE_1,
	AMX_MAX_INSTANCE
};

/* APBIF ids */
enum {
	APBIF_ID_0 = 0,
	APBIF_ID_1,
	APBIF_ID_2,
	APBIF_ID_3,
	APBIF_ID_4,
	APBIF_ID_5,
	APBIF_ID_6,
	APBIF_ID_7,
	APBIF_ID_8,
	APBIF_ID_9,
	MAX_APBIF_IDS
};

/* utils data */
/* Audio cif definition */
struct tegra_virt_cif {
	unsigned int threshold;
	unsigned int audio_channels;
	unsigned int client_channels;
	unsigned int audio_bits;
	unsigned int client_bits;
	unsigned int expand;
	unsigned int stereo_conv;
	unsigned int replicate;
	unsigned int direction;
	unsigned int truncate;
	unsigned int mono_conv;
};

struct slave_remap_add {
	void *apbif_base[MAX_APBIF_IDS];
	void *amx_base[AMX_MAX_INSTANCE];
	void *audio_amx_base;
};

struct tegra_virt_utils_data {
	unsigned int apbif_id;
	unsigned int amx_id;
	unsigned int amx_in_channel;
	struct tegra_virt_cif cif;
	struct slave_remap_add phandle;
};

void reg_write(void *base_address,
				unsigned int reg, unsigned int val);
unsigned int reg_read(void *base_address,
				unsigned int reg);
void tegra_find_amx_info(unsigned long data);
int create_ioremap(struct device *dev, struct slave_remap_add *phandle);
void remove_ioremap(struct device *dev, struct slave_remap_add *phandle);

#endif
