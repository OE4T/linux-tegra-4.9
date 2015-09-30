/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * Authors:
 *      VenkataJagadish.p	<vjagadish@nvidia.com>
 *      Naveen Kumar Arepalli	<naveenk@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/time.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/tegra-soc.h>
#include <linux/reset.h>
#include <linux/tegra-pmc.h>

#include "ufshcd.h"
#include "unipro.h"
#include "ufs-tegra.h"
#include "ufshci.h"



/*
 * ufs_tegra_ufs_pwrcntrl_update - To config UFSHC_PWR_CNTRL_0
 */

static void ufs_tegra_ufs_pwrcntrl_update(bool psw_on)
{
	if (psw_on) {
		tegra_pmc_ufs_pwrcntrl_update(UFSHC_PWR_CNTRL_0_LP_ISOL_EN_MASK,
			UFSHC_PWR_CNTRL_0_LP_ISOL_EN_ENABLE);
		tegra_pmc_ufs_pwrcntrl_update(UFSHC_PWR_CNTRL_0_LP_PWR_RDY_MASK,
			UFSHC_PWR_CNTRL_0_LP_PWR_RDY_DISABLE);
	} else {
		tegra_pmc_ufs_pwrcntrl_update(UFSHC_PWR_CNTRL_0_LP_PWR_RDY_MASK,
			UFSHC_PWR_CNTRL_0_LP_PWR_RDY_ENABLE);
		tegra_pmc_ufs_pwrcntrl_update(UFSHC_PWR_CNTRL_0_LP_ISOL_EN_MASK,
			UFSHC_PWR_CNTRL_0_LP_ISOL_EN_DISABLE);
	}
}

static int ufs_tegra_host_regulator_get(struct device *dev,
		const char *name, struct regulator **regulator_out)
{
	int err = 0;
	struct regulator *regulator;

	regulator = devm_regulator_get(dev, name);
	if (IS_ERR(regulator)) {
		err = PTR_ERR(regulator);
		dev_err(dev, "%s: failed to get %s err %d",
				__func__, name, err);
	} else {
		*regulator_out = regulator;
	}

	return err;
}

static int ufs_tegra_init_regulators(struct ufs_tegra_host *ufs_tegra)
{
	int err = 0;
	struct device *dev = ufs_tegra->hba->dev;

	err = ufs_tegra_host_regulator_get(dev, "vddio-ufs",
			&ufs_tegra->vddio_ufs);
	if (err)
		return err;

	ufs_tegra_host_regulator_get(dev, "vddio-ufs-ap",
			&ufs_tegra->vddio_ufs_ap);

	return err;
}

static int ufs_tegra_enable_regulators(struct ufs_tegra_host *ufs_tegra)
{
	int err = 0;
	struct device *dev = ufs_tegra->hba->dev;

	if (ufs_tegra->vddio_ufs) {
		err = regulator_enable(ufs_tegra->vddio_ufs);
		if (err) {
			dev_err(dev, "%s: vddio-ufs enable failed, err=%d\n",
					__func__, err);
			goto out;
		}
	}
	if (ufs_tegra->vddio_ufs_ap) {
		err = regulator_enable(ufs_tegra->vddio_ufs_ap);
		if (err) {
			dev_err(dev, "%s: vddio-ufs-ap enable failed err = %d\n",
					__func__, err);
			goto disable_vddio_ufs;
		}
	}
	return err;

disable_vddio_ufs:
	regulator_disable(ufs_tegra->vddio_ufs);
out:
	return err;
}

static int ufs_tegra_disable_regulators(struct ufs_tegra_host *ufs_tegra)
{
	int err = 0;
	struct device *dev = ufs_tegra->hba->dev;

	if (ufs_tegra->vddio_ufs) {
		err = regulator_disable(ufs_tegra->vddio_ufs);
		if (err) {
			dev_err(dev, "%s: vddio-ufs disable failed, err=%d\n",
					__func__, err);
			return err;
		}
	}
	if (ufs_tegra->vddio_ufs_ap) {
		err = regulator_disable(ufs_tegra->vddio_ufs_ap);
		if (err) {
			dev_err(dev, "%s: vddio-ufs-ap disable failed err = %d\n",
					__func__, err);
			return err;
		}
	}
	return err;
}

static int ufs_tegra_host_clk_get(struct device *dev,
		const char *name, struct clk **clk_out)
{
	struct clk *clk;
	int err = 0;

	clk = devm_clk_get(dev, name);
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		dev_err(dev, "%s: failed to get %s err %d",
				__func__, name, err);
	} else {
		*clk_out = clk;
	}

	return err;
}

static int ufs_tegra_host_clk_enable(struct device *dev,
		const char *name, struct clk *clk)
{
	int err = 0;

	err = clk_prepare_enable(clk);
	if (err)
		dev_err(dev, "%s: %s enable failed %d\n", __func__, name, err);

	return err;
}

/**
 * ufs_tegra_mphy_receiver_calibration
 * @ufs_tegra: ufs_tegra_host controller instance
 *
 * Implements MPhy Receiver Calibration Sequence
 *
 * Returns -1 if receiver calibration fails
 * and returns zero on success.
 */
static int ufs_tegra_mphy_receiver_calibration(struct ufs_tegra_host *ufs_tegra)
{
	struct device *dev = ufs_tegra->hba->dev;
	u32 mphy_rx_vendor2;

	mphy_update(ufs_tegra->mphy_l0_base, MPHY_RX_APB_VENDOR2_0_RX_CAL_EN,
			MPHY_RX_APB_VENDOR2_0);
	/*Wait 100 us, to complete receiver calibration*/
	udelay(100);
	mphy_rx_vendor2 = mphy_readl(ufs_tegra->mphy_l0_base,
				MPHY_RX_APB_VENDOR2_0);
	if (!(mphy_rx_vendor2 & MPHY_RX_APB_VENDOR2_0_RX_CAL_DONE)) {
		dev_err(dev, "Receiver Calibration failed for Lane0\n");
		return -1;
	}

	if (ufs_tegra->x2config) {
		mphy_update(ufs_tegra->mphy_l1_base,
			MPHY_RX_APB_VENDOR2_0_RX_CAL_EN, MPHY_RX_APB_VENDOR2_0);
		/*Wait 100 us, to complete receiver calibration*/
		udelay(100);
		mphy_rx_vendor2 = mphy_readl(ufs_tegra->mphy_l1_base,
					MPHY_RX_APB_VENDOR2_0);
		if (!(mphy_rx_vendor2 & MPHY_RX_APB_VENDOR2_0_RX_CAL_DONE)) {
			dev_err(dev, "Receiver Calibration failed for Lane1\n");
			return -1;
		}
	}
	return 0;
}

static void ufs_tegra_disable_mphylane_clks(struct ufs_tegra_host *host)
{
	if (!host->is_lane_clks_enabled)
		return;

	clk_disable_unprepare(host->mphy_core_pll_fixed);
	clk_disable_unprepare(host->mphy_l0_tx_symb);
	clk_disable_unprepare(host->mphy_tx_1mhz_ref);
	clk_disable_unprepare(host->mphy_l0_rx_ana);
	clk_disable_unprepare(host->mphy_l0_rx_symb);
	clk_disable_unprepare(host->mphy_l0_tx_ls_3xbit);
	clk_disable_unprepare(host->mphy_l0_rx_ls_bit);

	if (host->x2config)
		clk_disable_unprepare(host->mphy_l1_rx_ana);

	host->is_lane_clks_enabled = false;
}

static int ufs_tegra_enable_mphylane_clks(struct ufs_tegra_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	if (host->is_lane_clks_enabled)
		return 0;

	err = ufs_tegra_host_clk_enable(dev, "mphy_core_pll_fixed",
		host->mphy_core_pll_fixed);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_tx_symb",
		host->mphy_l0_tx_symb);
	if (err)
		goto disable_l0_tx_symb;

	err = ufs_tegra_host_clk_enable(dev, "mphy_tx_1mhz_ref",
		host->mphy_tx_1mhz_ref);
	if (err)
		goto disable_tx_1mhz_ref;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_rx_ana",
		host->mphy_l0_rx_ana);
	if (err)
		goto disable_l0_rx_ana;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_rx_symb",
		host->mphy_l0_rx_symb);
	if (err)
		goto disable_l0_rx_symb;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_tx_ls_3xbit",
		host->mphy_l0_tx_ls_3xbit);
	if (err)
		goto disable_l0_tx_ls_3xbit;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_rx_ls_bit",
		host->mphy_l0_rx_ls_bit);
	if (err)
		goto disable_l0_rx_ls_bit;

	if (host->x2config) {
		err = ufs_tegra_host_clk_enable(dev, "mphy_l1_rx_ana",
			host->mphy_l1_rx_ana);
		if (err)
			goto disable_l1_rx_ana;
	}

	host->is_lane_clks_enabled = true;
	goto out;

disable_l1_rx_ana:
	clk_disable_unprepare(host->mphy_l0_rx_ls_bit);
disable_l0_rx_ls_bit:
	clk_disable_unprepare(host->mphy_l0_tx_ls_3xbit);
disable_l0_tx_ls_3xbit:
	clk_disable_unprepare(host->mphy_l0_rx_symb);
disable_l0_rx_symb:
	clk_disable_unprepare(host->mphy_l0_rx_ana);
disable_l0_rx_ana:
	clk_disable_unprepare(host->mphy_tx_1mhz_ref);
disable_tx_1mhz_ref:
	clk_disable_unprepare(host->mphy_l0_tx_symb);
disable_l0_tx_symb:
	clk_disable_unprepare(host->mphy_core_pll_fixed);
out:
	return err;
}

static int ufs_tegra_init_mphy_lane_clks(struct ufs_tegra_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	err = ufs_tegra_host_clk_get(dev,
			"mphy_core_pll_fixed", &host->mphy_core_pll_fixed);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev,
			"mphy_l0_tx_symb", &host->mphy_l0_tx_symb);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_tx_1mhz_ref",
		&host->mphy_tx_1mhz_ref);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_rx_ana",
		&host->mphy_l0_rx_ana);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_rx_symb",
		&host->mphy_l0_rx_symb);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_tx_ls_3xbit",
		&host->mphy_l0_tx_ls_3xbit);
	if (err)
		goto out;

	if (host->x2config)
		err = ufs_tegra_host_clk_get(dev, "mphy_l1_rx_ana",
			&host->mphy_l1_rx_ana);
out:
	return err;
}

static int ufs_tegra_init_ufs_clks(struct ufs_tegra_host *ufs_tegra)
{
	int err = 0;
	struct device *dev = ufs_tegra->hba->dev;

	err = ufs_tegra_host_clk_get(dev,
		"pll_p", &ufs_tegra->ufshc_parent);
	if (err)
		goto out;
	err = ufs_tegra_host_clk_get(dev,
		"ufshc", &ufs_tegra->ufshc_clk);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev,
		"clk_m", &ufs_tegra->ufsdev_parent);
	if (err)
		goto out;
	err = ufs_tegra_host_clk_get(dev,
		"ufsdev_ref", &ufs_tegra->ufsdev_ref_clk);
	if (err)
		goto out;

out:
	return err;
}

static int ufs_tegra_enable_ufs_clks(struct ufs_tegra_host *ufs_tegra)
{
	struct device *dev = ufs_tegra->hba->dev;
	int err = 0;

	err = ufs_tegra_host_clk_enable(dev, "ufshc",
		ufs_tegra->ufshc_clk);
	if (err)
		goto out;
	err = clk_set_parent(ufs_tegra->ufshc_clk,
				ufs_tegra->ufshc_parent);
	if (err)
		goto out;
	err = clk_set_rate(ufs_tegra->ufshc_clk, UFSHC_CLK_FREQ);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_enable(dev, "ufsdev_ref",
		ufs_tegra->ufsdev_ref_clk);
	if (err)
		goto disable_ufshc;
	err = clk_set_parent(ufs_tegra->ufsdev_ref_clk,
				ufs_tegra->ufsdev_parent);
	if (err)
		goto disable_ufshc;
	err = clk_set_rate(ufs_tegra->ufsdev_ref_clk, UFSDEV_CLK_FREQ);
	if (err)
		goto disable_ufshc;

	ufs_tegra->hba->clk_gating.state = CLKS_ON;

	return err;

disable_ufshc:
	clk_disable_unprepare(ufs_tegra->ufshc_clk);
out:
	return err;
}

static void ufs_tegra_disable_ufs_clks(struct ufs_tegra_host *ufs_tegra)
{
	if (ufs_tegra->hba->clk_gating.state == CLKS_OFF)
		return;

	clk_disable_unprepare(ufs_tegra->ufshc_clk);
	clk_disable_unprepare(ufs_tegra->ufsdev_ref_clk);

	ufs_tegra->hba->clk_gating.state = CLKS_OFF;
}


static int ufs_tegra_ufs_reset_init(struct ufs_tegra_host *ufs_tegra)
{
	struct device *dev = ufs_tegra->hba->dev;
	int ret = 0;

	ufs_tegra->ufs_rst = devm_reset_control_get(dev, "ufs_rst");
	if (IS_ERR(ufs_tegra->ufs_rst)) {
		ret = PTR_ERR(ufs_tegra->ufs_rst);
		dev_err(dev,
			"Reset control for ufs_rst not found: %d\n", ret);
	}
	ufs_tegra->ufs_axi_m_rst = devm_reset_control_get(dev, "ufs_axi_m_rst");
	if (IS_ERR(ufs_tegra->ufs_axi_m_rst)) {
		ret = PTR_ERR(ufs_tegra->ufs_axi_m_rst);
		dev_err(dev,
			"Reset control for ufs_axi_m_rst not found: %d\n", ret);
	}
	ufs_tegra->ufshc_lp_rst = devm_reset_control_get(dev, "ufshc_lp_rst");
	if (IS_ERR(ufs_tegra->ufshc_lp_rst)) {
		ret = PTR_ERR(ufs_tegra->ufshc_lp_rst);
		dev_err(dev,
			"Reset control for ufshc_lp_rst not found: %d\n", ret);
	}
	return ret;
}

static void ufs_tegra_ufs_deassert_reset(struct ufs_tegra_host *ufs_tegra)
{
	reset_control_deassert(ufs_tegra->ufs_rst);
	reset_control_deassert(ufs_tegra->ufs_axi_m_rst);
	reset_control_deassert(ufs_tegra->ufshc_lp_rst);
}

static int ufs_tegra_mphy_reset_init(struct ufs_tegra_host *ufs_tegra)
{
	struct device *dev = ufs_tegra->hba->dev;
	int ret = 0;

	ufs_tegra->mphy_l0_rx_rst =
				devm_reset_control_get(dev, "mphy_l0_rx_rst");
	if (IS_ERR(ufs_tegra->mphy_l0_rx_rst)) {
		ret = PTR_ERR(ufs_tegra->mphy_l0_rx_rst);
		dev_err(dev,
			"Reset control for mphy_l0_rx_rst not found: %d\n",
									ret);
	}

	ufs_tegra->mphy_l0_tx_rst =
				devm_reset_control_get(dev, "mphy_l0_tx_rst");
	if (IS_ERR(ufs_tegra->mphy_l0_tx_rst)) {
		ret = PTR_ERR(ufs_tegra->mphy_l0_tx_rst);
		dev_err(dev,
			"Reset control for mphy_l0_tx_rst not found: %d\n",
									ret);
	}

	ufs_tegra->mphy_clk_ctl_rst =
				devm_reset_control_get(dev, "mphy_clk_ctl_rst");
	if (IS_ERR(ufs_tegra->mphy_clk_ctl_rst)) {
		ret = PTR_ERR(ufs_tegra->mphy_clk_ctl_rst);
		dev_err(dev,
			"Reset control for mphy_clk_ctl_rst not found: %d\n",
									ret);
	}

	if (ufs_tegra->x2config) {
		ufs_tegra->mphy_l1_rx_rst =
				devm_reset_control_get(dev, "mphy_l1_rx_rst");
		if (IS_ERR(ufs_tegra->mphy_l1_rx_rst)) {
			ret = PTR_ERR(ufs_tegra->mphy_l1_rx_rst);
			dev_err(dev,
			"Reset control for mphy_l1_rx_rst not found: %d\n",
									ret);
		}

		ufs_tegra->mphy_l1_tx_rst =
				devm_reset_control_get(dev, "mphy_l1_tx_rst");
		if (IS_ERR(ufs_tegra->mphy_l1_tx_rst)) {
			ret = PTR_ERR(ufs_tegra->mphy_l1_tx_rst);
			dev_err(dev,
			"Reset control for mphy_l0_tx_rst not found: %d\n",
									ret);
		}
	}

	return ret;
}

static void ufs_tegra_mphy_assert_reset(struct ufs_tegra_host *ufs_tegra)
{
	reset_control_assert(ufs_tegra->mphy_l0_rx_rst);
	reset_control_assert(ufs_tegra->mphy_l0_tx_rst);
	reset_control_assert(ufs_tegra->mphy_clk_ctl_rst);
	if (ufs_tegra->x2config) {
		reset_control_assert(ufs_tegra->mphy_l1_rx_rst);
		reset_control_assert(ufs_tegra->mphy_l1_tx_rst);
	}
}

static void ufs_tegra_mphy_deassert_reset(struct ufs_tegra_host *ufs_tegra)
{
	reset_control_deassert(ufs_tegra->mphy_l0_rx_rst);
	reset_control_deassert(ufs_tegra->mphy_l0_tx_rst);
	reset_control_deassert(ufs_tegra->mphy_clk_ctl_rst);
	if (ufs_tegra->x2config) {
		reset_control_deassert(ufs_tegra->mphy_l1_rx_rst);
		reset_control_deassert(ufs_tegra->mphy_l1_tx_rst);
	}
}

void ufs_tegra_disable_mphy_slcg(struct ufs_tegra_host *ufs_tegra)
{
	u32 val = 0;

	val = (MPHY_TX_CLK_EN_SYMB | MPHY_TX_CLK_EN_SLOW |
			MPHY_TX_CLK_EN_FIXED | MPHY_TX_CLK_EN_3X);
	mphy_writel(ufs_tegra->mphy_l0_base, val, MPHY_TX_APB_TX_CG_OVR0_0);

	if (ufs_tegra->x2config)
		mphy_writel(ufs_tegra->mphy_l1_base, val,
						MPHY_TX_APB_TX_CG_OVR0_0);

}

void ufs_tegra_mphy_advgran(struct ufs_tegra_host *ufs_tegra)
{
	u32 val = 0;

	val = (TX_ADVANCED_GRANULARITY | TX_ADVANCED_GRANULARITY_SETTINGS);
	mphy_update(ufs_tegra->mphy_l0_base, val,
					MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
	mphy_writel(ufs_tegra->mphy_l0_base, MPHY_GO_BIT,
						MPHY_TX_APB_TX_VENDOR0_0);

	if (ufs_tegra->x2config) {
		mphy_update(ufs_tegra->mphy_l1_base, val,
					MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
		mphy_writel(ufs_tegra->mphy_l1_base, MPHY_GO_BIT,
						MPHY_TX_APB_TX_VENDOR0_0);
	}

}

void ufs_tegra_ufs_aux_ref_clk_enable(struct ufs_tegra_host *ufs_tegra)
{
	ufs_aux_update(ufs_tegra->ufs_aux_base, UFSHC_DEV_CLK_EN,
						UFSHC_AUX_UFSHC_DEV_CTRL_0);
}

void ufs_tegra_ufs_aux_ref_clk_disable(struct ufs_tegra_host *ufs_tegra)
{
	ufs_aux_clear_bits(ufs_tegra->ufs_aux_base, UFSHC_DEV_CLK_EN,
						UFSHC_AUX_UFSHC_DEV_CTRL_0);
}

void ufs_tegra_ufs_aux_prog(struct ufs_tegra_host *ufs_tegra)
{
	u32 val;

	/*
	 * Release the reset to UFS device on pin ufs_rst_n
	 */

	if (ufs_tegra->ufshc_state == UFSHC_INIT)
		ufs_aux_update(ufs_tegra->ufs_aux_base, UFSHC_DEV_RESET,
						UFSHC_AUX_UFSHC_DEV_CTRL_0);


	if (ufs_tegra->ufshc_state == UFSHC_SUSPEND) {
		/*
		 * Disable reference clock to Device
		 */
		ufs_tegra_ufs_aux_ref_clk_disable(ufs_tegra);
		/*
		 * UFSHC clock gating control register programing
		 */
		val = 0;

	} else {
		/*
		 * Enable reference clock to Device
		 */
		ufs_tegra_ufs_aux_ref_clk_enable(ufs_tegra);
		/*
		 * UFSHC clock gating control register programing
		 */
		val = (UFSHC_CLK_OVR_ON | UFSHC_HCLK_OVR_ON |
			UFSHC_LP_CLK_T_CLK_OVR_ON | UFSHC_CLK_T_CLK_OVR_ON |
			UFSHC_CG_SYS_CLK_OVR_ON | UFSHC_TX_SYMBOL_CLK_OVR_ON |
			UFSHC_RX_SYMBOLCLKSELECTED_CLK_OVR_ON |
							UFSHC_PCLK_OVR_ON);
	}

	ufs_aux_writel(ufs_tegra->ufs_aux_base, val,
				UFSHC_AUX_UFSHC_SW_EN_CLK_SLCG_0);
}

static void ufs_tegra_context_save(struct ufs_tegra_host *ufs_tegra)
{
	u32 reg_len = 0;
	u32 *mphy_context_save = ufs_tegra->mphy_context;

	reg_len = ARRAY_SIZE(mphy_rx_apb);
	/*
	 * Save mphy_rx_apb lane0 and lane1 context
	 */
	ufs_save_regs(ufs_tegra->mphy_l0_base, &mphy_context_save,
							mphy_rx_apb, reg_len);

	if (ufs_tegra->x2config)
		ufs_save_regs(ufs_tegra->mphy_l1_base, &mphy_context_save,
							mphy_rx_apb, reg_len);

	reg_len = ARRAY_SIZE(mphy_tx_apb);
	/*
	 * Save mphy_tx_apb lane0 and lane1 context
	 */
	ufs_save_regs(ufs_tegra->mphy_l0_base, &mphy_context_save,
							mphy_tx_apb, reg_len);
	if (ufs_tegra->x2config)
		ufs_save_regs(ufs_tegra->mphy_l1_base, &mphy_context_save,
							mphy_tx_apb, reg_len);
}

static void ufs_tegra_context_restore(struct ufs_tegra_host *ufs_tegra)
{
	u32 reg_len = 0;
	u32 *mphy_context_restore = ufs_tegra->mphy_context;

	reg_len = ARRAY_SIZE(mphy_rx_apb);
	/*
	 * Restore mphy_rx_apb lane0 and lane1 context
	 */
	ufs_restore_regs(ufs_tegra->mphy_l0_base, &mphy_context_restore,
							mphy_rx_apb, reg_len);
	if (ufs_tegra->x2config)
		ufs_restore_regs(ufs_tegra->mphy_l1_base, &mphy_context_restore,
							mphy_rx_apb, reg_len);

	reg_len = ARRAY_SIZE(mphy_tx_apb);
	/*
	 * Restore mphy_tx_apb lane0 and lane1 context
	 */
	ufs_restore_regs(ufs_tegra->mphy_l0_base, &mphy_context_restore,
							mphy_tx_apb, reg_len);
	if (ufs_tegra->x2config)
		ufs_restore_regs(ufs_tegra->mphy_l1_base, &mphy_context_restore,
							mphy_tx_apb, reg_len);
}

static int ufs_tegra_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	u32 val;
	int ret = 0;

	ufs_tegra->ufshc_state = UFSHC_SUSPEND;

	/*
	 * TODO Call PMC driver Function calls
	 */

	val = ufs_aux_readl(ufs_tegra->ufs_aux_base, UFSHC_AUX_UFSHC_STATUS_0);
	if (val & UFSHC_HIBERNATE_STATUS) {
		/*
		 * Save all armphy_rx_apb and armphy_tx_apb registers
		 */
		ufs_tegra_context_save(ufs_tegra);

		ufs_tegra_ufs_pwrcntrl_update(true);
	}

	/*
	 * Disable mphy tx/rx lane clocks if they are on
	 * and assert the reset
	 */
	ufs_tegra_disable_mphylane_clks(ufs_tegra);
	ufs_tegra_mphy_assert_reset(ufs_tegra);
	ufs_tegra_ufs_aux_prog(ufs_tegra);

	/*
	 * Disable mphy tx/rx lane clocks if they are on
	 * and powerdown UPHY
	 */
	phy_power_off(ufs_tegra->u_phy);

	return ret;
}

static int ufs_tegra_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	int ret = 0;

	ufs_tegra->ufshc_state = UFSHC_RESUME;

	/*
	 * Power on UPHY
	 */
	ret = phy_power_on(ufs_tegra->u_phy);
	if (ret)
		goto out;

	/*
	 * TODO Enable Required regulators
	 */
	/*
	 * Enable mphy lane
	 */
	ret = ufs_tegra_enable_mphylane_clks(ufs_tegra);
	if (ret)
		goto out_mphy_exit;

	ufs_tegra_mphy_deassert_reset(ufs_tegra);
	ret = ufs_tegra_mphy_receiver_calibration(ufs_tegra);
	if (ret < 0)
		goto out_disable_mphylane_clks;

	ufs_tegra_ufs_pwrcntrl_update(false);
	ufs_tegra_ufs_aux_prog(ufs_tegra);
	ufs_tegra_disable_mphy_slcg(ufs_tegra);
	ufs_tegra_context_restore(ufs_tegra);
	ufs_tegra_mphy_advgran(ufs_tegra);

	return ret;

out_disable_mphylane_clks:
	ufs_tegra_disable_mphylane_clks(ufs_tegra);
out_mphy_exit:
	phy_power_off(ufs_tegra->u_phy);
out:
	phy_exit(ufs_tegra->u_phy);
	return ret;
}


/**
 * ufs_tegra_cfg_vendor_registers
 * @hba: host controller instance
 */
static void ufs_tegra_cfg_vendor_registers(struct ufs_hba *hba)
{
	ufshcd_writel(hba, UFS_VNDR_HCLKDIV_1US_TICK, REG_UFS_VNDR_HCLKDIV);
}

static int ufs_tegra_context_save_init(struct ufs_tegra_host *ufs_tegra)
{
	u32 context_save_size = 0;
	struct device *dev = ufs_tegra->hba->dev;
	int err = 0;

	context_save_size = ARRAY_SIZE(mphy_rx_apb) + ARRAY_SIZE(mphy_tx_apb);
	context_save_size *= sizeof(u32);

	ufs_tegra->mphy_context = devm_kzalloc(dev, context_save_size,
								GFP_KERNEL);
	if (!ufs_tegra->mphy_context) {
		err = -ENOMEM;
		dev_err(dev, "no memory for tegra ufs mphy_context\n");
	}
	return err;
}

/**
 * ufs_tegra_init - bind phy with controller
 * @hba: host controller instance
 *
 * Binds PHY with controller and powers up UPHY enabling clocks
 * and regulators.
 *
 * Returns -EPROBE_DEFER if binding fails, returns negative error
 * on phy power up failure and returns zero on success.
 */
static int ufs_tegra_init(struct ufs_hba *hba)
{
	struct ufs_tegra_host *ufs_tegra;
	struct device *dev = hba->dev;
	int err = 0;

	ufs_tegra = devm_kzalloc(dev, sizeof(*ufs_tegra), GFP_KERNEL);
	if (!ufs_tegra) {
		err = -ENOMEM;
		dev_err(dev, "no memory for tegra ufs host\n");
		goto out;
	}
	ufs_tegra->ufshc_state = UFSHC_INIT;
	ufs_tegra->hba = hba;
	hba->priv = (void *)ufs_tegra;

	ufs_tegra->ufs_aux_base = devm_ioremap(dev,
			NV_ADDRESS_MAP_UFSHC_AUX_BASE, UFS_AUX_ADDR_RANGE);
	if (!ufs_tegra->ufs_aux_base) {
		err = -ENOMEM;
		dev_err(dev, "ufs_aux_base ioremap failed\n");
		goto out;
	}

	if (tegra_platform_is_silicon()) {
		ufs_tegra->mphy_l0_base = devm_ioremap(dev,
				NV_ADDRESS_MAP_MPHY_L0_BASE, MPHY_ADDR_RANGE);
		if (!ufs_tegra->ufs_aux_base) {
			err = -ENOMEM;
			dev_err(dev, "mphy_l0_base ioremap failed\n");
			goto out;
		}
		ufs_tegra->mphy_l1_base = devm_ioremap(dev,
				NV_ADDRESS_MAP_MPHY_L1_BASE, MPHY_ADDR_RANGE);
		if (!ufs_tegra->ufs_aux_base) {
			err = -ENOMEM;
			dev_err(dev, "mphy_l1_base ioremap failed\n");
			goto out;
		}

		err = ufs_tegra_context_save_init(ufs_tegra);
		if (err)
			goto out;

		ufs_tegra->u_phy = devm_phy_get(dev, "uphy");

		if (IS_ERR(ufs_tegra->u_phy)) {
			err = PTR_ERR(ufs_tegra->u_phy);
			dev_err(dev, "PHY get failed %d\n", err);
			goto out_host_free;
		}

		err = phy_init(ufs_tegra->u_phy);
		if (err)
			goto out_host_free;
		err = phy_power_on(ufs_tegra->u_phy);
		if (err)
			goto out_phy_exit;
	}

	if (tegra_platform_is_silicon()) {
		err = ufs_tegra_init_regulators(ufs_tegra);
		if (err) {
			goto out_disable_uphy;
		} else {
			err = ufs_tegra_enable_regulators(ufs_tegra);
			if (err)
				goto out_disable_uphy;
		}
		err = ufs_tegra_init_ufs_clks(ufs_tegra);
		if (err)
			goto out_disable_regulators;

		err = ufs_tegra_enable_ufs_clks(ufs_tegra);
		if (err)
			goto out_disable_regulators;

		err = ufs_tegra_init_mphy_lane_clks(ufs_tegra);
		if (err)
			goto out_disable_ufs_clks;

		err = ufs_tegra_enable_mphylane_clks(ufs_tegra);
		if (err)
			goto out_disable_ufs_clks;

		err = ufs_tegra_mphy_reset_init(ufs_tegra);
		if (err)
			goto out_disable_mphylane_clks;
		ufs_tegra_mphy_deassert_reset(ufs_tegra);
		err = ufs_tegra_mphy_receiver_calibration(ufs_tegra);
		if (err < 0)
			goto out_disable_mphylane_clks;
		err = ufs_tegra_ufs_reset_init(ufs_tegra);
		if (err)
			goto out_disable_mphylane_clks;
		ufs_tegra_ufs_deassert_reset(ufs_tegra);
		ufs_tegra_ufs_pwrcntrl_update(false);
		ufs_tegra_disable_mphy_slcg(ufs_tegra);
		ufs_tegra_ufs_aux_prog(ufs_tegra);
		ufs_tegra_cfg_vendor_registers(hba);
	}
	return err;

out_disable_mphylane_clks:
	if (tegra_platform_is_silicon())
		ufs_tegra_disable_mphylane_clks(ufs_tegra);
out_disable_ufs_clks:
	ufs_tegra_disable_ufs_clks(ufs_tegra);
out_disable_regulators:
	if (tegra_platform_is_silicon())
		ufs_tegra_disable_regulators(ufs_tegra);
out_disable_uphy:
	if (tegra_platform_is_silicon())
		phy_power_off(ufs_tegra->u_phy);
out_phy_exit:
	if (tegra_platform_is_silicon())
		phy_exit(ufs_tegra->u_phy);
out_host_free:
	if (tegra_platform_is_silicon())
		hba->priv = NULL;
out:
	return err;
}

static void ufs_tegra_exit(struct ufs_hba *hba)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;

	if (tegra_platform_is_silicon()) {
		ufs_tegra_disable_mphylane_clks(ufs_tegra);
		phy_power_off(ufs_tegra->u_phy);
		phy_exit(ufs_tegra->u_phy);

	}
}

/**
 * struct ufs_hba_tegra_vops - UFS TEGRA specific variant operations
 *
 * The variant operations configure the necessary controller and PHY
 * handshake during initialization.
 */
struct ufs_hba_variant_ops ufs_hba_tegra_vops = {
	.name                   = "ufs-tegra",
	.init                   = ufs_tegra_init,
	.exit                   = ufs_tegra_exit,
	.suspend		= ufs_tegra_suspend,
	.resume			= ufs_tegra_resume,
};
