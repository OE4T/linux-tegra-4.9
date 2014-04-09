
/*
 * drivers/video/tegra/dc/sor.h
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_SOR_H__
#define __DRIVERS_VIDEO_TEGRA_DC_SOR_H__

enum {
	TRAINING_PATTERN_DISABLE = 0,
	TRAINING_PATTERN_1 = 1,
	TRAINING_PATTERN_2 = 2,
	TRAINING_PATTERN_3 = 3,
};

enum tegra_dc_sor_protocol {
	SOR_DP,
	SOR_LVDS,
};

#define SOR_LINK_SPEED_G1_62	6
#define SOR_LINK_SPEED_G2_7	10
#define SOR_LINK_SPEED_G5_4	20
#define SOR_LINK_SPEED_LVDS	7

struct tegra_dc_dp_link_config {
	bool	is_valid;	/*
				 * True if link config adheres to dp spec.
				 * Does not guarantee link training success.
				 */

	/* Supported configuration */
	u8	max_link_bw;
	u8	max_lane_count;
	bool	downspread;
	bool	support_enhanced_framing;
	u32	bits_per_pixel;
	bool	alt_scramber_reset_cap; /* true for eDP */
	bool	only_enhanced_framing;	/* enhanced_frame_en ignored */
	bool	edp_cap;		/* eDP display control capable */
	bool	support_fast_lt;	/* Support fast link training */

	/* Actual configuration */
	u8	link_bw;
	u8	lane_count;
	bool	enhanced_framing;
	bool	scramble_ena;

	u32	activepolarity;
	u32	active_count;
	u32	tu_size;
	u32	active_frac;
	u32	watermark;

	s32	hblank_sym;
	s32	vblank_sym;

	bool	lt_data_valid;	/*
				 * True only if link training passed with this
				 * drive_current, preemphasis and postcursor.
				 */
	u32	drive_current[4];
	u32	preemphasis[4];
	u32	postcursor[4];

	bool	tps3_supported;
	u8	aux_rd_interval;
};

enum {
	TEGRA_SOR_SAFE_CLK = 1,
	TEGRA_SOR_LINK_CLK = 2,
};

struct tegra_dc_sor_data {
	struct tegra_dc	*dc;

	void __iomem	*base;
	struct resource	*base_res;
	struct clk	*sor_clk;

	u8					 portnum;	/* 0 or 1 */
	const struct tegra_dc_dp_link_config	*link_cfg;

	bool   power_is_up;

	u8	clk_type;
};

#define TEGRA_SOR_TIMEOUT_MS		1000
#define TEGRA_SOR_ATTACH_TIMEOUT_MS	100000
#define TEGRA_SOR_SEQ_BUSY_TIMEOUT_MS	10000
#define TEGRA_DC_POLL_TIMEOUT_MS       50

#define CHECK_RET(x)			\
	do {				\
		ret = (x);		\
		if (ret != 0)		\
			return ret;	\
	} while (0)


struct tegra_dc_sor_data *tegra_dc_sor_init(struct tegra_dc *dc,
	const struct tegra_dc_dp_link_config *cfg);

void tegra_dc_sor_destroy(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_enable_dp(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_attach(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_detach(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_enable_lvds(struct tegra_dc_sor_data *sor,
	bool balanced, bool conforming);
void tegra_dc_sor_disable(struct tegra_dc_sor_data *sor, bool is_lvds);

void tegra_dc_sor_set_internal_panel(struct tegra_dc_sor_data *sor,
	bool is_int);
void tegra_dc_sor_read_link_config(struct tegra_dc_sor_data *sor,
	u8 *link_bw, u8 *lane_count);
void tegra_dc_sor_set_link_bandwidth(struct tegra_dc_sor_data *sor,
	u8 link_bw);
void tegra_dc_sor_set_lane_count(struct tegra_dc_sor_data *sor, u8 lane_count);
void tegra_sor_pad_cal_power(struct tegra_dc_sor_data *sor, bool power_up);
void tegra_dc_sor_set_pwm(struct tegra_dc_sor_data *sor, u32 pwm_div,
	u32 pwm_dutycycle, u32 pwm_clksrc);
void tegra_dc_sor_set_dp_lanedata(struct tegra_dc_sor_data *sor,
	u32 lane, u32 pre_emphasis, u32 drive_current, u32 tx_pu);
void tegra_dc_sor_set_dp_linkctl(struct tegra_dc_sor_data *sor, bool ena,
	u8 training_pattern, const struct tegra_dc_dp_link_config *cfg);
void tegra_dc_sor_set_dp_mode(struct tegra_dc_sor_data *sor,
	const struct tegra_dc_dp_link_config *cfg);
void tegra_sor_setup_clk(struct tegra_dc_sor_data *sor, struct clk *clk,
	bool is_lvds);
void tegra_sor_precharge_lanes(struct tegra_dc_sor_data *sor);
int tegra_dc_sor_set_power_state(struct tegra_dc_sor_data *sor,
	int pu_pd);
void tegra_dc_sor_modeset_notifier(struct tegra_dc_sor_data *sor,
	bool is_lvds);
void tegra_sor_tpg(struct tegra_dc_sor_data *sor, u32 tp, u32 n_lanes);
void tegra_sor_port_enable(struct tegra_dc_sor_data *sor, bool enb);
int tegra_sor_power_dp_lanes(struct tegra_dc_sor_data *sor,
					u32 lane_count, bool pu);
void tegra_sor_config_dp_clk(struct tegra_dc_sor_data *sor);
void tegra_sor_stop_dc(struct tegra_dc_sor_data *sor);
void tegra_sor_start_dc(struct tegra_dc_sor_data *sor);

static inline u32 tegra_sor_readl(struct tegra_dc_sor_data *sor, u32 reg)
{
	u32 reg_val = readl(sor->base + reg * 4);
	return reg_val;
}

static inline void tegra_sor_writel(struct tegra_dc_sor_data *sor,
	u32 reg, u32 val)
{
	writel(val, sor->base + reg * 4);
}

static inline void tegra_sor_write_field(struct tegra_dc_sor_data *sor,
	u32 reg, u32 mask, u32 val)
{
	u32 reg_val = tegra_sor_readl(sor, reg);
	reg_val &= ~mask;
	reg_val |= val;
	tegra_sor_writel(sor, reg, reg_val);
}

static inline void tegra_sor_clk_enable(struct tegra_dc_sor_data *sor)
{
	clk_prepare_enable(sor->sor_clk);
}

static inline void tegra_sor_clk_disable(struct tegra_dc_sor_data *sor)
{
	clk_disable_unprepare(sor->sor_clk);
}

static inline int lt_param_idx(int link_bw)
{
	int idx;
	switch (link_bw) {
	case SOR_LINK_SPEED_G1_62:
		idx = 0;
		break;
	case SOR_LINK_SPEED_G2_7:
		idx = 1;
		break;
	case SOR_LINK_SPEED_G5_4:
		idx = 2;
		break;
	default:
		/* Error BW */
		BUG_ON(1);
	}
	return idx;
}

#endif
