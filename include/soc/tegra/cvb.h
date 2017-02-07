/*
 * Utility functions for parsing Tegra CVB voltage tables
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef __DRIVERS_CLK_TEGRA_CVB_H
#define __DRIVERS_CLK_TEGRA_CVB_H

#include <linux/types.h>

struct device;

#define MAX_DVFS_FREQS	40

struct rail_alignment {
	int offset_uv;
	int step_uv;
};

struct cvb_coefficients {
	int c0;
	int c1;
	int c2;
	int c3;
	int c4;
	int c5;
};

struct cvb_table_freq_entry {
	unsigned long freq;
	struct cvb_coefficients coefficients;
};

struct cvb_cpu_dfll_data {
	u32 tune0_low;
	u32 tune0_high;
	u32 tune1;
	unsigned int tune_high_min_millivolts;
};

struct thermal_coefficients {
	struct cvb_coefficients cvb_coef;
	int c3;
	int c4;
	int c5;
};

/* Thermal trips and voltages */
struct thermal_tv {
	int temp;
	unsigned int millivolts;
};

struct cvb_table {
	int speedo_id;
	int process_id;

	int min_millivolts;
	int max_millivolts;

	int speedo_scale;
	int voltage_scale;
	struct cvb_table_freq_entry cvb_table[MAX_DVFS_FREQS];
	struct cvb_cpu_dfll_data cpu_dfll_data;
};

struct thermal_table {
	struct thermal_tv *thermal_floor_table;
	unsigned int thermal_floor_table_size;
	struct thermal_coefficients coefficients;
	unsigned int speedo_scale;
	unsigned int voltage_scale;
	unsigned int temp_scale;

	const struct thermal_tv *thermal_cap_table;
	unsigned int thermal_cap_table_size;
};

#if defined(CONFIG_TEGRA_DVFS)
const struct cvb_table *tegra_cvb_build_opp_table(
		const struct cvb_table *cvb_tables,
		size_t sz,
		const struct rail_alignment *align,
		int process_id,
		int speedo_id,
		int speedo_value,
		unsigned long max_rate,
		struct device *opp_dev);
int tegra_round_voltage(int mv, const struct rail_alignment *align, int up);
#else
static inline const struct cvb_table *tegra_cvb_build_opp_table(
		const struct cvb_table *cvb_tables,
		size_t sz,
		const struct rail_alignment *align,
		int process_id,
		int speedo_id,
		int speedo_value,
		unsigned long max_rate,
		struct device *opp_dev)
{
	return ERR_PTR(-ENODEV);
}

static inline int tegra_round_voltage(
		int mv,
		const struct rail_alignment *align,
		int up)
{
	return mv;
}
#endif

int tegra_get_cvb_voltage(int speedo, int s_scale,
			  const struct cvb_coefficients *cvb);
int tegra_round_cvb_voltage(int mv, int v_scale,
			    const struct rail_alignment *align);
int tegra_get_cvb_t_voltage(int speedo, int s_scale, int t, int t_scale,
			    struct cvb_coefficients *cvb);
int tegra_cvb_build_thermal_table(const struct thermal_table *table,
		int speedo_value);

#endif
