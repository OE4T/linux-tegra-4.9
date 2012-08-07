/*
 * drivers/video/tegra/dc/sor.c
 *
 * Copyright (c) 2011-2102, NVIDIA Corporation.
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
#include <linux/delay.h>

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

	clk = clk_get_sys("had2codec", "dpaux");
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

static inline void tegra_sor_write_field(struct tegra_dc_sor_data *sor,
	unsigned long reg, u32 mask, u32 val)
{
	u32 reg_val = tegra_sor_readl(sor, reg);
	reg_val &= ~mask;
	reg_val |= val;
	tegra_sor_writel(sor, reg, reg_val);
}

static unsigned long tegra_dc_sor_poll_register(struct tegra_dc_sor_data *sor,
	u32 reg, u32 mask, u32 exp_val, u32 poll_interval_us, u32 timeout_ms)
{
	unsigned long timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);
	u32 reg_val = 0;

	do {
		usleep_range(poll_interval_us, poll_interval_us << 1);
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

	/* Poll to confirm it is done */
	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWR,
			NV_SOR_PWR_SETTING_NEW_DEFAULT_MASK,
			NV_SOR_PWR_SETTING_NEW_DONE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for SOR_PWR = NEW_DONE\n");
		return -EFAULT;
	}
	return 0;
}


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


int tegra_dc_sor_power_dplanes(struct tegra_dc_sor_data *sor,
	u32 lane_count)
{
	unsigned long reg_val;

	reg_val = tegra_sor_readl(sor, NV_SOR_DP_PADCTL(sor->portnum)) | 0xf;

	switch (lane_count) {
	case 4:
		reg_val &= (NV_SOR_DP_PADCTL_PD_TXD_3_YES |
			NV_SOR_DP_PADCTL_PD_TXD_2_YES);
		/* fall through */
	case 2:
		reg_val &= NV_SOR_DP_PADCTL_PD_TXD_1_YES;
	case 1:
		reg_val &= NV_SOR_DP_PADCTL_PD_TXD_0_YES;
		break;
	default:
		dev_dbg(&sor->dc->ndev->dev,
			"dp: invalid lane number %d\n", lane_count);
		return -EFAULT;
	}

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
EXPORT_SYMBOL(tegra_dc_sor_power_dplanes);


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


static void tegra_dc_sor_config_pwm(struct tegra_dc_sor_data *sor, u32 pwm_div,
	u32 pwm_dutycycle)
{
	tegra_sor_writel(sor, NV_SOR_PWM_DIV, pwm_div);
	tegra_sor_writel(sor, NV_SOR_PWM_CTL,
		(pwm_dutycycle & NV_SOR_PWM_CTL_DUTY_CYCLE_MASK) |
		NV_SOR_PWM_CTL_SETTING_NEW_TRIGGER);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWM_CTL,
			NV_SOR_PWM_CTL_SETTING_NEW_SHIFT,
			NV_SOR_PWM_CTL_SETTING_NEW_DONE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_dbg(&sor->dc->ndev->dev,
			"dp: timeout while waiting for SOR PWM setting\n");
	}
}

static void tegra_dc_sor_set_dp_mode(struct tegra_dc_sor_data *sor,
	const struct tegra_dc_dp_link_config *cfg)
{
	unsigned long reg_val;

	tegra_dc_sor_set_link_bandwidth(sor, cfg->link_bw);

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

static void tegra_dc_sor_poweron(struct tegra_dc_sor_data *sor, u32 vdd_mode)
{
	/* enable PLL */
	tegra_sor_writel(sor, NV_SOR_PLL2,
		NV_SOR_PLL2_AUX1_SEQ_PLLCAPPD_OVERRIDE |
		NV_SOR_PLL2_AUX2_OVERRIDE_POWERDOWN |
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_DISABLE |
		NV_SOR_PLL2_AUX7_PORT_POWERDOWN_DISABLE |
		NV_SOR_PLL2_DCIR_PLL_RESET_OVERRIDE |
		NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_DISABLE);
	tegra_sor_writel(sor, NV_SOR_PLL0,
		NV_SOR_PLL0_VCOPD_RESCIND |
		NV_SOR_PLL0_PWR_ON);
	tegra_sor_writel(sor, NV_SOR_PLL3, vdd_mode);
}

static inline void tegra_dc_sor_super_update(struct tegra_dc_sor_data *sor)
{
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE0, 0);
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE0, 1);
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE0, 0);
}

static inline void tegra_dc_sor_update(struct tegra_dc_sor_data *sor)
{
	tegra_sor_writel(sor, NV_SOR_STATE0, 0);
	tegra_sor_writel(sor, NV_SOR_STATE0, 1);
	tegra_sor_writel(sor, NV_SOR_STATE0, 0);
}

static inline void tegra_dc_sor_enable_sequencer(struct tegra_dc_sor_data *sor)
{
	tegra_sor_writel(sor, NV_SOR_DP_SPARE(sor->portnum),
		NV_SOR_DP_SPARE_SEQ_ENABLE_YES);

	tegra_sor_writel(sor, NV_SOR_SEQ_CTL,
		(8 << NV_SOR_SEQ_CTL_PD_PC_SHIFT) |
		(8 << NV_SOR_SEQ_CTL_PD_PC_ALT_SHIFT));

	tegra_sor_write_field(sor, NV_SOR_LANE_SEQ_CTL,
		NV_SOR_LANE_SEQ_CTL_DELAY_DEFAULT_MASK, 0);
}

static void tegra_dc_sor_sequencer_dp_start(struct tegra_dc_sor_data *sor)
{
	tegra_dc_sor_enable_sequencer(sor);

	tegra_sor_writel(sor, NV_SOR_SEQ_INST(0),
		NV_SOR_SEQ_INST_WAIT_UNITS_VSYNC |
		NV_SOR_SEQ_INST_HALT_TRUE |
		NV_SOR_SEQ_INST_DRIVE_PWM_OUT_LO_TRUE |
		NV_SOR_SEQ_INST_LANE_SEQ_RUN);

	tegra_sor_writel(sor, NV_SOR_SEQ_INST(8),
		NV_SOR_SEQ_INST_WAIT_UNITS_VSYNC |
		NV_SOR_SEQ_INST_HALT_TRUE |
		NV_SOR_SEQ_INST_DRIVE_PWM_OUT_LO_TRUE |
		NV_SOR_SEQ_INST_LANE_SEQ_RUN |
		NV_SOR_SEQ_INST_SEQUENCE_DOWN);
}

static void tegra_dc_sor_sequencer_lvds_start(struct tegra_dc_sor_data *sor)
{
	tegra_dc_sor_enable_sequencer(sor);

	tegra_sor_writel(sor, NV_SOR_SEQ_INST(0),
		NV_SOR_SEQ_INST_POWERDOWN_MACRO_POWERDOWN |
		NV_SOR_SEQ_INST_BLANK_V_NORMAL |
		NV_SOR_SEQ_INST_BLANK_H_NORMAL |
		NV_SOR_SEQ_INST_BLANK_DE_NORMAL |
		NV_SOR_SEQ_INST_BLACK_DATA_BLACK |
		NV_SOR_SEQ_INST_TRISTATE_IOS_TRISTATE |
		NV_SOR_SEQ_INST_DRIVE_PWM_OUT_LO_TRUE |
		NV_SOR_SEQ_INST_PIN_B_LOW |
		NV_SOR_SEQ_INST_PIN_A_LOW |
		NV_SOR_SEQ_INST_SEQUENCE_UP |
		NV_SOR_SEQ_INST_LANE_SEQ_STOP |
		NV_SOR_SEQ_INST_PDPORT_NO |
		NV_SOR_SEQ_INST_PDPLL_NO |
		NV_SOR_SEQ_INST_HALT_FALSE |
		NV_SOR_SEQ_INST_WAIT_UNITS_US |
		0 << NV_SOR_SEQ_INST_WAIT_TIME_SHIFT);


	tegra_sor_writel(sor, NV_SOR_SEQ_INST(1),
		NV_SOR_SEQ_INST_POWERDOWN_MACRO_POWERDOWN |
		NV_SOR_SEQ_INST_BLANK_V_NORMAL |
		NV_SOR_SEQ_INST_BLANK_H_NORMAL |
		NV_SOR_SEQ_INST_BLANK_DE_NORMAL |
		NV_SOR_SEQ_INST_BLACK_DATA_BLACK |
		NV_SOR_SEQ_INST_TRISTATE_IOS_ENABLE_PINS |
		NV_SOR_SEQ_INST_DRIVE_PWM_OUT_LO_TRUE |
		NV_SOR_SEQ_INST_PIN_B_LOW |
		NV_SOR_SEQ_INST_PIN_A_HIGH |
		NV_SOR_SEQ_INST_SEQUENCE_UP |
		NV_SOR_SEQ_INST_LANE_SEQ_STOP |
		NV_SOR_SEQ_INST_PDPORT_NO |
		NV_SOR_SEQ_INST_PDPLL_NO |
		NV_SOR_SEQ_INST_HALT_FALSE |
		NV_SOR_SEQ_INST_WAIT_UNITS_US |
		1 << NV_SOR_SEQ_INST_WAIT_TIME_SHIFT);


	tegra_sor_writel(sor, NV_SOR_SEQ_INST(2),
		NV_SOR_SEQ_INST_POWERDOWN_MACRO_NORMAL |
		NV_SOR_SEQ_INST_BLANK_V_NORMAL |
		NV_SOR_SEQ_INST_BLANK_H_NORMAL |
		NV_SOR_SEQ_INST_BLANK_DE_NORMAL |
		NV_SOR_SEQ_INST_BLACK_DATA_NORMAL |
		NV_SOR_SEQ_INST_TRISTATE_IOS_ENABLE_PINS |
		NV_SOR_SEQ_INST_DRIVE_PWM_OUT_LO_TRUE |
		NV_SOR_SEQ_INST_PIN_B_LOW |
		NV_SOR_SEQ_INST_PIN_A_HIGH |
		NV_SOR_SEQ_INST_SEQUENCE_UP |
		NV_SOR_SEQ_INST_LANE_SEQ_RUN |
		NV_SOR_SEQ_INST_PDPORT_NO |
		NV_SOR_SEQ_INST_PDPLL_NO |
		NV_SOR_SEQ_INST_HALT_FALSE |
		NV_SOR_SEQ_INST_WAIT_UNITS_US |
		33 << NV_SOR_SEQ_INST_WAIT_TIME_SHIFT);

	tegra_sor_writel(sor, NV_SOR_SEQ_INST(3),
		NV_SOR_SEQ_INST_POWERDOWN_MACRO_NORMAL |
		NV_SOR_SEQ_INST_BLANK_V_NORMAL |
		NV_SOR_SEQ_INST_BLANK_H_NORMAL |
		NV_SOR_SEQ_INST_BLANK_DE_NORMAL |
		NV_SOR_SEQ_INST_BLACK_DATA_NORMAL |
		NV_SOR_SEQ_INST_TRISTATE_IOS_ENABLE_PINS |
		NV_SOR_SEQ_INST_DRIVE_PWM_OUT_LO_TRUE |
		NV_SOR_SEQ_INST_PIN_B_LOW |
		NV_SOR_SEQ_INST_PIN_A_HIGH |
		NV_SOR_SEQ_INST_SEQUENCE_UP |
		NV_SOR_SEQ_INST_LANE_SEQ_STOP |
		NV_SOR_SEQ_INST_PDPORT_NO |
		NV_SOR_SEQ_INST_PDPLL_NO |
		NV_SOR_SEQ_INST_HALT_FALSE |
		NV_SOR_SEQ_INST_WAIT_UNITS_US |
		205 << NV_SOR_SEQ_INST_WAIT_TIME_SHIFT);

	tegra_sor_writel(sor, NV_SOR_SEQ_INST(4),
		NV_SOR_SEQ_INST_POWERDOWN_MACRO_NORMAL |
		NV_SOR_SEQ_INST_BLANK_V_NORMAL |
		NV_SOR_SEQ_INST_BLANK_H_NORMAL |
		NV_SOR_SEQ_INST_BLANK_DE_NORMAL |
		NV_SOR_SEQ_INST_BLACK_DATA_NORMAL |
		NV_SOR_SEQ_INST_TRISTATE_IOS_ENABLE_PINS |
		NV_SOR_SEQ_INST_DRIVE_PWM_OUT_LO_FALSE |
		NV_SOR_SEQ_INST_PIN_B_HIGH |
		NV_SOR_SEQ_INST_PIN_A_HIGH |
		NV_SOR_SEQ_INST_SEQUENCE_UP |
		NV_SOR_SEQ_INST_LANE_SEQ_STOP |
		NV_SOR_SEQ_INST_PDPORT_NO |
		NV_SOR_SEQ_INST_PDPLL_NO |
		NV_SOR_SEQ_INST_HALT_FALSE |
		NV_SOR_SEQ_INST_WAIT_UNITS_VSYNC |
		0 << NV_SOR_SEQ_INST_WAIT_TIME_SHIFT);

	tegra_sor_writel(sor, NV_SOR_SEQ_INST(5),
		NV_SOR_SEQ_INST_POWERDOWN_MACRO_NORMAL |
		NV_SOR_SEQ_INST_BLANK_V_NORMAL |
		NV_SOR_SEQ_INST_BLANK_H_NORMAL |
		NV_SOR_SEQ_INST_BLANK_DE_NORMAL |
		NV_SOR_SEQ_INST_BLACK_DATA_NORMAL |
		NV_SOR_SEQ_INST_TRISTATE_IOS_ENABLE_PINS |
		NV_SOR_SEQ_INST_DRIVE_PWM_OUT_LO_FALSE |
		NV_SOR_SEQ_INST_PIN_B_HIGH |
		NV_SOR_SEQ_INST_PIN_A_HIGH |
		NV_SOR_SEQ_INST_SEQUENCE_UP |
		NV_SOR_SEQ_INST_LANE_SEQ_STOP |
		NV_SOR_SEQ_INST_PDPORT_NO |
		NV_SOR_SEQ_INST_PDPLL_NO |
		NV_SOR_SEQ_INST_HALT_TRUE |
		NV_SOR_SEQ_INST_WAIT_UNITS_US |
		0 << NV_SOR_SEQ_INST_WAIT_TIME_SHIFT);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWR,
			NV_SOR_PWR_SETTING_NEW_DEFAULT_MASK,
			NV_SOR_PWR_SETTING_NEW_DONE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for SOR_PWR = DONE\n");
		return;
	}
}

static void tegra_dc_sor_sequencer_lvds_stop(struct tegra_dc_sor_data *sor)
{
	tegra_dc_sor_enable_sequencer(sor);

	tegra_sor_writel(sor, NV_SOR_SEQ_INST(0),
		NV_SOR_SEQ_INST_WAIT_UNITS_VSYNC |
		1 << NV_SOR_SEQ_INST_WAIT_TIME_SHIFT);
	tegra_sor_writel(sor, NV_SOR_SEQ_INST(1),
		NV_SOR_SEQ_INST_BLANK_V_INACTIVE |
		NV_SOR_SEQ_INST_BLANK_H_INACTIVE |
		NV_SOR_SEQ_INST_BLANK_DE_INACTIVE |
		NV_SOR_SEQ_INST_BLACK_DATA_BLACK |
		NV_SOR_SEQ_INST_WAIT_UNITS_VSYNC |
		1 << NV_SOR_SEQ_INST_WAIT_TIME_SHIFT);
	tegra_sor_writel(sor, NV_SOR_SEQ_INST(2),
		NV_SOR_SEQ_INST_PIN_B_LOW |
		NV_SOR_SEQ_INST_TRISTATE_IOS_TRISTATE);
	tegra_sor_writel(sor, NV_SOR_SEQ_INST(3),
		NV_SOR_SEQ_INST_LANE_SEQ_RUN |
		NV_SOR_SEQ_INST_SEQUENCE_DOWN);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWR,
			NV_SOR_PWR_SETTING_NEW_DEFAULT_MASK,
			NV_SOR_PWR_SETTING_NEW_DONE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for SOR_PWR = DONE\n");
		return;
	}
}

static void tegra_dc_sor_config_panel(struct tegra_dc_sor_data *sor,
	bool is_lvds)
{
	const struct tegra_dc_out_pin	*pins	  = sor->dc->out->out_pins;
	const struct tegra_dc_mode	*dc_mode  = sor->dc->out->modes;

	const int	head_num = sor->dc->ndev->id;
	u32		reg_val  = NV_SOR_STATE1_ASY_OWNER_HEAD0 << head_num;
	u32		vtotal, htotal;
	u32		vsync_end, hsync_end;
	u32		vblank_end, hblank_end;
	u32		vblank_start, hblank_start;
	int		i;

	reg_val |= is_lvds ? NV_SOR_STATE1_ASY_PROTOCOL_LVDS_CUSTOM :
		NV_SOR_STATE1_ASY_PROTOCOL_DP_A;
	reg_val |= NV_SOR_STATE1_ASY_SUBOWNER_BOTH;

	for (i = 0; pins && (i < sor->dc->out->n_out_pins); i++) {
		switch (pins[i].name) {
		case TEGRA_DC_OUT_PIN_DATA_ENABLE:
			if (pins[i].pol == TEGRA_DC_OUT_PIN_POL_LOW)
				reg_val |=
				NV_SOR_STATE1_ASY_DEPOL_NEGATIVE_TRUE;
			break;
		case TEGRA_DC_OUT_PIN_H_SYNC:
			if (pins[i].pol == TEGRA_DC_OUT_PIN_POL_LOW)
				reg_val |=
				NV_SOR_STATE1_ASY_HSYNCPOL_NEGATIVE_TRUE;
			break;
		case TEGRA_DC_OUT_PIN_V_SYNC:
			if (pins[i].pol == TEGRA_DC_OUT_PIN_POL_LOW)
				reg_val |=
				NV_SOR_STATE1_ASY_VSYNCPOL_NEGATIVE_TRUE;
			break;
		default:	/* Ignore other pin setting */
			break;
		}
	}

	reg_val |= (sor->dc->pdata->fb->bits_per_pixel > 16) ?
		NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_24_444 :
		NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_18_444;

	BUG_ON(!dc_mode);
	vtotal = dc_mode->v_sync_width + dc_mode->v_back_porch +
		dc_mode->v_active + dc_mode->v_front_porch;
	htotal = dc_mode->h_sync_width + dc_mode->h_back_porch +
		dc_mode->h_active + dc_mode->h_front_porch;
	tegra_sor_writel(sor, NV_HEAD_STATE1(head_num),
		vtotal << NV_HEAD_STATE1_VTOTAL_SHIFT |
		htotal << NV_HEAD_STATE1_HTOTAL_SHIFT);

	vsync_end = dc_mode->v_sync_width - dc_mode->v_ref_to_sync;
	hsync_end = dc_mode->h_sync_width - dc_mode->h_ref_to_sync;
	tegra_sor_writel(sor, NV_HEAD_STATE2(head_num),
		vsync_end << NV_HEAD_STATE2_VSYNC_END_SHIFT |
		hsync_end << NV_HEAD_STATE2_HSYNC_END_SHIFT);

	vblank_end = dc_mode->v_sync_width + dc_mode->v_back_porch
		- dc_mode->v_ref_to_sync;
	hblank_end = dc_mode->h_sync_width + dc_mode->h_back_porch
		- dc_mode->h_ref_to_sync;
	tegra_sor_writel(sor, NV_HEAD_STATE3(head_num),
		vblank_end << NV_HEAD_STATE3_VBLANK_END_SHIFT |
		hblank_end << NV_HEAD_STATE3_HBLANK_END_SHIFT);

	vblank_start = vblank_end + dc_mode->v_active;
	hblank_start = hblank_end + dc_mode->h_active;
	tegra_sor_writel(sor, NV_HEAD_STATE4(head_num),
		vblank_start << NV_HEAD_STATE4_VBLANK_START_SHIFT |
		hblank_start << NV_HEAD_STATE4_HBLANK_START_SHIFT);

	/* TODO: setup rotclk */

	tegra_dc_sor_config_pwm(sor, 1024, 1024);
}

void tegra_dc_sor_enable_dp(struct tegra_dc_sor_data *sor)
{
	unsigned long reg_val;

	tegra_dc_sor_poweron(sor, NV_SOR_PLL3_PLLVDD_MODE_V3_3);
	tegra_dc_sor_power_dplanes(sor, sor->link_cfg->lane_count);
	tegra_dc_sor_set_dp_mode(sor, sor->link_cfg);

	tegra_dc_sor_sequencer_dp_start(sor);
	tegra_dc_sor_set_power_state(sor, 1);
	tegra_dc_sor_config_panel(sor, false);
	tegra_dc_sor_update(sor);

	reg_val = NV_SOR_SUPER_STATE1_ASY_HEAD_OP_AWAKE |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_NORMAL |
		NV_SOR_SUPER_STATE1_ATTACHED_NO;
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1, reg_val);
	tegra_dc_sor_super_update(sor);

	reg_val |= NV_SOR_SUPER_STATE1_ATTACHED_YES;
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1, reg_val);
	tegra_dc_sor_super_update(sor);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_TEST,
			NV_SOR_TEST_ATTACHED_DEFAULT_MASK,
			NV_SOR_TEST_ATTACHED_TRUE,
			100, TEGRA_SOR_ATTACH_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for ATTACHED = TRUE\n");
	}
}
EXPORT_SYMBOL(tegra_dc_sor_enable_dp);

void tegra_dc_sor_enable_lvds(struct tegra_dc_sor_data *sor)
{
	tegra_dc_sor_poweron(sor, NV_SOR_PLL3_PLLVDD_MODE_V1_8);
	tegra_sor_write_field(sor, NV_SOR_DP_SPARE(sor->portnum),
		NV_SOR_DP_SPARE_SOR_CLK_SEL_DEFAULT_MASK,
		NV_SOR_DP_SPARE_SOR_CLK_SEL_MACRO_SORCLK);

	/* TODO: check if balanced mode is needed
		 check ROTDAT setting */
	tegra_sor_writel(sor, NV_SOR_LVDS,
		NV_SOR_LVDS_LINKACTB_DISABLE |
		NV_SOR_LVDS_PD_TXCB_DISABLE |
		NV_SOR_LVDS_PD_TXDB_3_DISABLE |
		NV_SOR_LVDS_PD_TXDB_2_DISABLE |
		NV_SOR_LVDS_PD_TXDB_1_DISABLE |
		NV_SOR_LVDS_PD_TXDB_0_DISABLE |
		NV_SOR_LVDS_PD_TXDA_2_ENABLE |
		NV_SOR_LVDS_PD_TXDA_1_ENABLE |
		NV_SOR_LVDS_PD_TXDA_0_ENABLE);

	tegra_dc_sor_set_link_bandwidth(sor, SOR_LINK_SPEED_LVDS);
	tegra_dc_sor_config_panel(sor, true);
	tegra_dc_sor_update(sor);

	/* Attaching */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_SLEEP |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_SAFE |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_TEST,
			NV_SOR_TEST_ATTACHED_DEFAULT_MASK,
			NV_SOR_TEST_ATTACHED_TRUE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for ATTACHED = TRUE\n");
		return;
	}

	/* OR mode: normal */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_SLEEP |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_NORMAL |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);

	/* then awake */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_AWAKE |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_NORMAL |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);

	/* TODO: check if display controller needs to start now*/
	tegra_dc_sor_sequencer_lvds_start(sor);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWR,
			NV_SOR_PWR_MODE_DEFAULT_MASK,
			NV_SOR_PWR_MODE_NORMAL,
			100, TEGRA_SOR_ATTACH_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for ATTACHED = TRUE\n");
		return;
	}
}
EXPORT_SYMBOL(tegra_dc_sor_enable_lvds);

void tegra_dc_sor_disable(struct tegra_dc_sor_data *sor, bool is_lvds)
{
	/* #1: safe mode */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_AWAKE |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_SAFE |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWR,
			NV_SOR_PWR_MODE_DEFAULT_MASK,
			NV_SOR_PWR_MODE_SAFE,
			100, TEGRA_SOR_ATTACH_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for SOR_PWR = NEW_DONE\n");
		return;
	}

	if (is_lvds)
		tegra_dc_sor_sequencer_lvds_stop(sor);

	/* #2: sleep */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_SLEEP |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_SAFE |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_TEST,
			NV_SOR_TEST_ACT_HEAD_OPMODE_DEFAULT_MASK,
			NV_SOR_TEST_ACT_HEAD_OPMODE_SLEEP,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for ACT_HEAD_OPMODE = SLEEP\n");
		return;
	}

	/* #3: detach */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_SLEEP |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_SAFE |
		NV_SOR_SUPER_STATE1_ATTACHED_NO);
	tegra_dc_sor_super_update(sor);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_TEST,
			NV_SOR_TEST_ATTACHED_DEFAULT_MASK,
			NV_SOR_TEST_ATTACHED_FALSE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for ATTACHED = FALSE\n");
		return;
	}

	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_STATE1_ASY_OWNER_NONE |
		NV_SOR_STATE1_ASY_SUBOWNER_NONE |
		NV_SOR_STATE1_ASY_PROTOCOL_LVDS_CUSTOM);
	tegra_dc_sor_update(sor);

	/* TODO: need to program:
	   DISP_WIN_OPTIONS(SOR_DISABLE)
	   DISPLAY_COMMAND(DISPLAY_CTRL_MODE_STOP)
	   DISPLAY_POWER_CONTROL(POWER_ENABLE_DISABLE)
	 */

	/* Power down the SOR sequencer */
	if ((tegra_dc_sor_set_power_state(sor, 0))) {
		dev_err(&sor->dc->ndev->dev,
			"Failed to power down SOR sequencer\n");
		return;
	}

	/* Power down DP lanes */
	if (!is_lvds && tegra_dc_sor_power_dplanes(sor, 0)) {
		dev_err(&sor->dc->ndev->dev,
			"Failed to power down dp lanes\n");
		return;
	}

	clk_disable(sor->sor_clk);
}
EXPORT_SYMBOL(tegra_dc_sor_disable);

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
