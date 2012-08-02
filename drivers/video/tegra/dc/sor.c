/*
 * drivers/video/tegra/dc/sor.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/nvhost.h>
#include <linux/kernel.h>
#include <linux/fb.h>

#include <mach/clk.h>
#include <mach/dc.h>

#include "sor.h"
#include "sor_regs.h"
#include "dc_priv.h"


struct tegra_dc_sor_data *tegra_dc_sor_init(struct tegra_dc *dc,
	const struct tegra_dc_dp_link_config *cfg)
{
	struct tegra_dc_sor_data	*sor;
	struct resource			*res;
	struct resource			*base_res;
	void __iomem			*base;
	struct clk			*clk;
	int				 err;

	sor = kzalloc(sizeof(*sor), GFP_KERNEL);
	if (!sor) {
		err = -ENOMEM;
		goto err_allocate;
	}

	res = nvhost_get_resource_byname(dc->ndev, IORESOURCE_MEM, "sor");
	if (!res) {
		dev_err(&dc->ndev->dev, "sor: no mem resource\n");
		err = -ENOENT;
		goto err_free_sor;
	}

	base_res = request_mem_region(res->start, resource_size(res),
		dc->ndev->name);
	if (!base_res) {
		dev_err(&dc->ndev->dev, "sor: request_mem_region failed\n");
		err = -EBUSY;
		goto err_free_sor;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&dc->ndev->dev, "sor: registers can't be mapped\n");
		err = -ENOENT;
		goto err_release_resource_reg;
	}

	clk = clk_get(&dc->ndev->dev, "sor");
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&dc->ndev->dev, "sor: can't get clock\n");
		err = -ENOENT;
		goto err_iounmap_reg;
	}

	sor->dc	      = dc;
	sor->base     = base;
	sor->base_res = base_res;
	sor->sor_clk  = clk;
	sor->link_cfg = cfg;

	return sor;


err_iounmap_reg:
	iounmap(base);
err_release_resource_reg:
	release_resource(base_res);
err_free_sor:
	kfree(sor);
err_allocate:
	return ERR_PTR(err);
}

static inline unsigned long tegra_sor_readl(struct tegra_dc_sor_data *sor,
	unsigned long reg)
{
	return readl(sor->base + reg * 4);
}

static inline void tegra_sor_writel(struct tegra_dc_sor_data *sor,
	unsigned long reg, unsigned long val)
{
	writel(val, sor->base + reg * 4);
}


static unsigned long tegra_dc_sor_poll_register(struct tegra_dc_sor_data *sor,
	u32 reg, u32 mask, u32 exp_val, u32 poll_interval_us, u32 timeout_ms)
{
	unsigned long timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);
	u32 reg_val = 0;

	do {
		usleep(poll_interval_us);
		reg_val = tegra_sor_readl(sor, reg);
	} while (((reg_val & mask) != exp_val) &&
		time_after(timeout_jf, jiffies));

	if ((reg_val & mask) == exp_val)
		return 0;	/* success */
	dev_dbg(&sor->dc->ndev->dev,
		"sor_poll_register 0x%x: timeout\n", reg);
	return jiffies - timeout_jf + 1;
}


static int tegra_dc_sor_set_power_state(struct tegra_dc_sor_data *sor,
	int pu_pd)
{
	unsigned long reg_val;
	unsigned long orig_val;

	if (tegra_dc_sor_poll_register(sor, NV_SOR_SEQ_CTL,
			NV_SOR_SEQ_CTL_STATUS_MASK,
			NV_SOR_SEQ_CTL_STATUS_STOPPED,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for SOR_SEQ_CT = STATUS_STOPPED\n");
		return -EFAULT;
	}

	orig_val = tegra_sor_readl(sor, NV_SOR_PWR);

	reg_val = pu_pd ? NV_SOR_PWR_NORMAL_STATE_PU :
		NV_SOR_PWR_NORMAL_STATE_PD; /* normal state only */

	if (reg_val == orig_val)
		return 0;	/* No update needed */

	reg_val |= NV_SOR_PWR_SETTING_NEW_TRIGGER;
	tegra_sor_writel(sor, NV_SOR_PWR, reg_val);
	return 0;
}

void tegra_dc_sor_disable(struct tegra_dc_sor_data *sor)
{
	/* Power down the SOR sequencer */
	if ((tegra_dc_sor_set_power_state(sor, 0))) {
		dev_err(&sor->dc->ndev->dev,
			"Failed to power down SOR sequencer\n");
		return;
	}

	/* Power down DP lanes */
	if (tegra_dc_sor_powerdown_dplanes(sor, 0)) {
		dev_err(&sor->dc->ndev->dev,
			"Failed to power down dp lanes\n");
		return;
	}
}
EXPORT_SYMBOL(tegra_dc_sor_disable);

void tegra_dc_sor_destroy(struct tegra_dc_sor_data *sor)
{
	clk_put(sor->sor_clk);
	iounmap(sor->base);
	release_resource(sor->base_res);
	kfree(sor);
}
EXPORT_SYMBOL(tegra_dc_sor_destroy);

void tegra_dc_sor_set_dp_linkctl(struct tegra_dc_sor_data *sor, bool ena,
	u8 training_pattern, const struct tegra_dc_dp_link_config *cfg,
	bool use_scramble)
{
	unsigned long reg_val;

	reg_val = tegra_sor_readl(sor, NV_SOR_DP_LINKCTL(sor->portnum));

	if (ena)
		reg_val |= NV_SOR_DP_LINKCTL_ENABLE_YES;
	else
		reg_val &= NV_SOR_DP_LINKCTL_ENABLE_NO;

	reg_val |= (cfg->tu_size << NV_SOR_DP_LINKCTL_TUSIZE_SHIFT);

	/* !!!TODO: fix the scrambler enabling */
	if (use_scramble)
		reg_val |= cfg->scramble_ena <<
			NV_SOR_DP_LINKCTL_SCRAMBLEREN_SHIFT;

	if (cfg->enhanced_framing)
		reg_val |= NV_SOR_DP_LINKCTL_ENHANCEDFRAME_ENABLE;
	reg_val |= (training_pattern <<
		NV_SOR_DP_LINKCTL_TRAININGPTTRN_SHIFT);

	tegra_sor_writel(sor, NV_SOR_DP_LINKCTL(sor->portnum),
		reg_val);
}
EXPORT_SYMBOL(tegra_dc_sor_set_dp_linkctl);

void tegra_dc_sor_set_dp_lanedata(struct tegra_dc_sor_data *sor,
	u32 lane, u32 pre_emphasis, u32 drive_current, u32 tx_pu)
{
	unsigned long d_cur;
	unsigned long p_emp;
	unsigned long p_ctl;


	d_cur = tegra_sor_readl(sor, NV_SOR_DC(sor->portnum));
	p_emp = tegra_sor_readl(sor, NV_SOR_PR(sor->portnum));
	p_ctl = tegra_sor_readl(sor, NV_SOR_DP_PADCTL(sor->portnum));

	switch (lane) {
	case 0:
		p_emp &= ~NV_SOR_PR_LANE2_DP_LANE0_MASK;
		p_emp |= (pre_emphasis <<
			NV_SOR_PR_LANE2_DP_LANE0_SHIFT);
		d_cur &= ~NV_SOR_DC_LANE2_DP_LANE0_MASK;
		d_cur |= (drive_current <<
			NV_SOR_DC_LANE2_DP_LANE0_SHIFT);
		break;
	case 1:
		p_emp &= ~NV_SOR_PR_LANE1_DP_LANE1_MASK;
		p_emp |= (pre_emphasis <<
			NV_SOR_PR_LANE1_DP_LANE1_SHIFT);
		d_cur &= ~NV_SOR_DC_LANE1_DP_LANE1_MASK;
		d_cur |= (drive_current <<
			NV_SOR_DC_LANE1_DP_LANE1_SHIFT);
		break;
	case 2:
		p_emp &= ~NV_SOR_PR_LANE0_DP_LANE2_MASK;
		p_emp |= (pre_emphasis <<
			NV_SOR_PR_LANE0_DP_LANE2_SHIFT);
		d_cur &= ~NV_SOR_DC_LANE0_DP_LANE2_MASK;
		d_cur |= (drive_current <<
			NV_SOR_DC_LANE0_DP_LANE2_SHIFT);
		break;
	case 3:
		p_emp &= ~NV_SOR_PR_LANE3_DP_LANE3_MASK;
		p_emp |= (pre_emphasis <<
			NV_SOR_PR_LANE3_DP_LANE3_SHIFT);
		d_cur &= ~NV_SOR_DC_LANE3_DP_LANE3_MASK;
		d_cur |= (drive_current <<
			NV_SOR_DC_LANE3_DP_LANE3_SHIFT);
	default:
		dev_err(&sor->dc->ndev->dev,
			"dp: sor lane count %d is invalid\n", lane);
	}

	p_ctl &= ~NV_SOR_DP_PADCTL_TX_PU_VALUE_DEFAULT_MASK;
	p_ctl |= (tx_pu << NV_SOR_DP_PADCTL_TX_PU_VALUE_SHIFT);

	tegra_sor_writel(sor, NV_SOR_DP_LINKCTL(sor->portnum), d_cur);
	tegra_sor_writel(sor, NV_SOR_PR(sor->portnum),
		p_emp);
	tegra_sor_writel(sor, NV_SOR_DP_PADCTL(sor->portnum),
		p_ctl);
}
EXPORT_SYMBOL(tegra_dc_sor_set_dp_lanedata);


int tegra_dc_sor_powerdown_dplanes(struct tegra_dc_sor_data *sor,
	u32 lane_count)
{
	unsigned long reg_val;

	reg_val = tegra_sor_readl(sor, NV_SOR_DP_PADCTL(sor->portnum));

	if (lane_count < 4)
		reg_val &= (NV_SOR_DP_PADCTL_PD_TXD_3_YES |
			NV_SOR_DP_PADCTL_PD_TXD_2_YES);
	if (lane_count < 2)
		reg_val &= NV_SOR_DP_PADCTL_PD_TXD_1_YES;
	if (lane_count < 1)
		reg_val &= NV_SOR_DP_PADCTL_PD_TXD_0_YES;
	tegra_sor_writel(sor, NV_SOR_DP_PADCTL(sor->portnum), reg_val);

	/* SOR lane sequencer */
	reg_val = tegra_sor_readl(sor, NV_SOR_LANE_SEQ_CTL);

	reg_val |= NV_SOR_LANE_SEQ_CTL_SETTING_NEW_TRIGGER;
	tegra_sor_writel(sor, NV_SOR_LANE_SEQ_CTL, reg_val);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_LANE_SEQ_CTL,
			NV_SOR_LANE_SEQ_CTL_SETTING_MASK, 0,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_dbg(&sor->dc->ndev->dev,
			"dp: timeout while waiting for SOR lane sequencer "
			"to power down langes\n");
		return -EFAULT;
	}
	return 0;
}
EXPORT_SYMBOL(tegra_dc_sor_powerdown_dplanes);


void tegra_dc_sor_set_panel_power(struct tegra_dc_sor_data *sor,
	bool power_up)
{
	unsigned long reg_val;

	/* !!TODO: need to enable panel power through GPIO operations */
	/* Check bug 790854 for HW progress */

	reg_val = tegra_sor_readl(sor, NV_SOR_DP_PADCTL(sor->portnum));

	if (power_up)
		reg_val |= NV_SOR_DP_PADCTL_PAD_CAL_PD_POWERUP;
	else
		reg_val &= ~NV_SOR_DP_PADCTL_PAD_CAL_PD_POWERUP;

	tegra_sor_writel(sor, NV_SOR_DP_PADCTL(sor->portnum), reg_val);
}
EXPORT_SYMBOL(tegra_dc_sor_set_panel_power);


void tegra_dc_sor_config_pwm(struct tegra_dc_sor_data *sor, u32 pwm_div,
	u32 pwm_dutycycle, u32 pwm_clksrc)
{
	unsigned long reg_val;

	tegra_sor_writel(sor, NV_SOR_PWM_DIV, pwm_div);

	reg_val = pwm_dutycycle & NV_SOR_PWM_CTL_DUTY_CYCLE_MASK;
	reg_val |= (pwm_clksrc << NV_SOR_PWM_CTL_DUTY_CYCLE_SHIFT);
	reg_val |= NV_SOR_PWM_CTL_SETTING_NEW_TRIGGER;
	tegra_sor_writel(sor, NV_SOR_PWM_CTL, reg_val);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWM_CTL,
			NV_SOR_PWM_CTL_SETTING_NEW_SHIFT,
			NV_SOR_PWM_CTL_SETTING_NEW_DONE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_dbg(&sor->dc->ndev->dev,
			"dp: timeout while waiting for SOR PWM setting\n");
	}
}
EXPORT_SYMBOL(tegra_dc_sor_config_pwm);

static void tegra_dc_sor_set_dp_mode(struct tegra_dc_sor_data *sor,
	const struct tegra_dc_dp_link_config *cfg)
{
	unsigned long reg_val;

	reg_val = NV_SOR_CLK_CNTRL_DP_CLK_SEL_DIFF_DPCLK;
	reg_val |= (cfg->link_bw << NV_SOR_CLK_CNTRL_DP_LINK_SPEED_SHIFT);
	tegra_sor_writel(sor, NV_SOR_CLK_CNTRL, reg_val);

	/* TODO: SOR_NV_PDISP_SOR_REFCL */
	/* tegra_dc_sor_set_dp_linkctl(sor, true,
	   trainingPattern_Disabled, true); */
	reg_val = tegra_sor_readl(sor,
		NV_SOR_DP_CONFIG(sor->portnum));
	reg_val &= NV_SOR_DP_CONFIG_WATERMARK_MASK;
	reg_val |= cfg->watermark;
	reg_val &= NV_SOR_DP_CONFIG_ACTIVESYM_COUNT_MASK;
	reg_val |= (cfg->active_count <<
		NV_SOR_DP_CONFIG_ACTIVESYM_COUNT_SHIFT);
	reg_val &= NV_SOR_DP_CONFIG_ACTIVESYM_FRAC_MASK;
	reg_val |= (cfg->active_frac <<
		NV_SOR_DP_CONFIG_ACTIVESYM_FRAC_SHIFT);
	if (cfg->activepolarity)
		reg_val |= NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_POSITIVE;
	else
		reg_val &= ~NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_POSITIVE;
	reg_val |= NV_SOR_DP_CONFIG_ACTIVESYM_CNTL_ENABLE;

	tegra_sor_writel(sor, NV_SOR_DP_CONFIG(sor->portnum),
		reg_val);

	/* program h/vblank sym */
	reg_val = tegra_sor_readl(sor,
		NV_SOR_DP_LINKCTL(sor->portnum));

	reg_val = cfg->hblank_sym & NV_SOR_DP_AUDIO_HBLANK_SYMBOLS_MASK;
	tegra_sor_writel(sor, NV_SOR_DP_AUDIO_HBLANK_SYMBOLS,
		reg_val);

	reg_val = cfg->vblank_sym & NV_SOR_DP_AUDIO_VBLANK_SYMBOLS_MASK;
	tegra_sor_writel(sor, NV_SOR_DP_AUDIO_VBLANK_SYMBOLS,
		reg_val);
}

void tegra_dc_sor_enable(struct tegra_dc_sor_data *sor)
{
	tegra_dc_sor_set_dp_mode(sor, sor->link_cfg);
}
EXPORT_SYMBOL(tegra_dc_sor_enable);


int tegra_dc_sor_set_dp_packet(struct tegra_dc_sor_data *sor,
	u8 *packet)
{
	/* No need to set the infoframe yet as there is no audio or
	   stereo support. This is a placeholder for now */
	return 0;
}


void tegra_dc_sor_set_internal_panel(struct tegra_dc_sor_data *sor, bool is_int)
{
	unsigned long reg_val;

	reg_val = tegra_sor_readl(sor, NV_SOR_DP_SPARE(sor->portnum));
	if (is_int)
		reg_val |= NV_SOR_DP_SPARE_PANEL_INTERNAL;
	else
		reg_val &= ~NV_SOR_DP_SPARE_PANEL_INTERNAL;
	tegra_sor_writel(sor, NV_SOR_DP_SPARE(sor->portnum), reg_val);
}
EXPORT_SYMBOL(tegra_dc_sor_set_internal_panel);

void tegra_dc_sor_read_link_config(struct tegra_dc_sor_data *sor, u8 *link_bw,
				   u8 *lane_count)
{
	unsigned long reg_val;

	reg_val = tegra_sor_readl(sor, NV_SOR_CLK_CNTRL);
	*link_bw = (reg_val & NV_SOR_CLK_CNTRL_DP_LINK_SPEED_MASK)
		>> NV_SOR_CLK_CNTRL_DP_LINK_SPEED_SHIFT;
	reg_val = tegra_sor_readl(sor,
		NV_SOR_DP_LINKCTL(sor->portnum));

	switch (reg_val & NV_SOR_DP_LINKCTL_LANECOUNT_MASK) {
	case NV_SOR_DP_LINKCTL_LANECOUNT_ZERO:
		*lane_count = 0;
		break;
	case NV_SOR_DP_LINKCTL_LANECOUNT_ONE:
		*lane_count = 1;
		break;
	case NV_SOR_DP_LINKCTL_LANECOUNT_TWO:
		*lane_count = 2;
		break;
	case NV_SOR_DP_LINKCTL_LANECOUNT_FOUR:
		*lane_count = 4;
		break;
	default:
		dev_err(&sor->dc->ndev->dev, "Unknown lane count\n");
	}
}
EXPORT_SYMBOL(tegra_dc_sor_read_link_config);

void tegra_dc_sor_set_link_bandwidth(struct tegra_dc_sor_data *sor, u8 link_bw)
{
	unsigned long reg_val;

	/* FIXME!!! make sure the clk (single/diff dpclk) is programmed */
	reg_val = tegra_sor_readl(sor, NV_SOR_CLK_CNTRL);
	reg_val &= ~NV_SOR_CLK_CNTRL_DP_LINK_SPEED_MASK;
	reg_val |= link_bw << NV_SOR_CLK_CNTRL_DP_LINK_SPEED_SHIFT;
	tegra_sor_writel(sor, NV_SOR_CLK_CNTRL, reg_val);
}
EXPORT_SYMBOL(tegra_dc_sor_set_link_bandwidth);

void tegra_dc_sor_set_lane_count(struct tegra_dc_sor_data *sor, u8 lane_count)
{
	unsigned long reg_val;

	reg_val = tegra_sor_readl(sor, NV_SOR_DP_LINKCTL(sor->portnum));
	reg_val &= ~NV_SOR_DP_LINKCTL_LANECOUNT_MASK;
	switch (lane_count) {
	case 0:
		break;
	case 1:
		reg_val |= NV_SOR_DP_LINKCTL_LANECOUNT_ONE;
		break;
	case 2:
		reg_val |= NV_SOR_DP_LINKCTL_LANECOUNT_TWO;
		break;
	case 4:
		reg_val |= NV_SOR_DP_LINKCTL_LANECOUNT_FOUR;
		break;
	default:
		/* 0 should be handled earlier. */
		dev_err(&sor->dc->ndev->dev, "dp: Invalid lane count %d\n",
			lane_count);
		return;
	}
	reg_val |= (lane_count << NV_SOR_DP_LINKCTL_LANECOUNT_SHIFT);
	tegra_sor_writel(sor, NV_SOR_DP_LINKCTL(sor->portnum),
		reg_val);
}
EXPORT_SYMBOL(tegra_dc_sor_set_lane_count);
