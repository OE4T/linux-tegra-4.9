/*
 * dpaux.c: dpaux function definitions.
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
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

#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/tegra_prod.h>

#include "dpaux_regs.h"
#include "dc_priv.h"
#include "dpaux.h"
#include "dp.h"
#include "hdmi2.0.h"
#include "../../../../arch/arm/mach-tegra/iomap.h"

static DEFINE_MUTEX(dpaux_lock);

static inline void tegra_dpaux_get_name(char *buf, size_t buf_len, int sor_num)
{
	if (sor_num > 0)
		snprintf(buf, buf_len, "dpaux%d", sor_num);
	else
		snprintf(buf, buf_len, "dpaux");
}

int tegra_dpaux_readl(struct tegra_dc_dpaux_data *dpaux, u32 reg)
{
	return readl(dpaux->base + reg * 4);
}

void tegra_dpaux_writel(struct tegra_dc_dpaux_data *dpaux, u32 reg, u32 val)
{
	writel(val, dpaux->base + reg * 4);
}

void tegra_dpaux_write_field(struct tegra_dc_dpaux_data *dpaux, u32 reg,
				u32 mask, u32 val)
{
	u32 reg_val = tegra_dpaux_readl(dpaux, reg);

	reg_val = (reg_val & ~mask) | (val & mask);
	tegra_dpaux_writel(dpaux, reg, reg_val);
}

int tegra_dpaux_clk_en(struct tegra_dc_dpaux_data *dpaux)
{
	return tegra_disp_clk_prepare_enable(dpaux->clk);
}

void tegra_dpaux_clk_dis(struct tegra_dc_dpaux_data *dpaux)
{
	tegra_disp_clk_disable_unprepare(dpaux->clk);
}

void tegra_dpaux_int_toggle(struct tegra_dc_dpaux_data *dpaux, u32 intr,
				bool enable)
{
	u32 reg_val = tegra_dpaux_readl(dpaux, DPAUX_INTR_EN_AUX);

	if (enable)
		reg_val |= intr;
	else
		reg_val &= ~intr;

	tegra_dpaux_writel(dpaux, DPAUX_INTR_EN_AUX, reg_val);
}

static inline void _tegra_dpaux_pad_power(struct tegra_dc_dpaux_data *dpaux,
					bool on)
{
	tegra_dpaux_writel(dpaux,
			DPAUX_HYBRID_SPARE,
			(on ? DPAUX_HYBRID_SPARE_PAD_PWR_POWERUP :
			DPAUX_HYBRID_SPARE_PAD_PWR_POWERDOWN));
}

void tegra_dpaux_pad_power(struct tegra_dc_dpaux_data *dpaux, bool on)
{
	struct tegra_dc *dc;

	if (!dpaux || !dpaux->dc) {
		pr_err("%s: dpaux and dc must both be non-NULL", __func__);
		return;
	}

	dc = dpaux->dc;
	tegra_dpaux_clk_en(dpaux);

	tegra_dc_io_start(dc);

	mutex_lock(&dpaux_lock);
	_tegra_dpaux_pad_power(dpaux, on);
	mutex_unlock(&dpaux_lock);

	tegra_dc_io_end(dc);
	tegra_dpaux_clk_dis(dpaux);
}

static inline void _tegra_dpaux_config_pad_mode(
					struct tegra_dc_dpaux_data *dpaux,
					enum tegra_dpaux_pad_mode mode)
{
	u32 val = 0;

	val = tegra_dpaux_readl(dpaux, DPAUX_HYBRID_PADCTL);

	val &= ~(DPAUX_HYBRID_PADCTL_I2C_SDA_INPUT_RCV_ENABLE |
		DPAUX_HYBRID_PADCTL_I2C_SCL_INPUT_RCV_ENABLE |
		DPAUX_HYBRID_PADCTL_MODE_I2C);
	val |= mode ? (DPAUX_HYBRID_PADCTL_I2C_SDA_INPUT_RCV_ENABLE |
		DPAUX_HYBRID_PADCTL_I2C_SCL_INPUT_RCV_ENABLE |
		mode) : 0;

	tegra_dpaux_writel(dpaux, DPAUX_HYBRID_PADCTL, val);
}

void tegra_dpaux_config_pad_mode(struct tegra_dc_dpaux_data *dpaux,
			enum tegra_dpaux_pad_mode mode)
{
	struct tegra_dc *dc;

	if (!dpaux || !dpaux->dc) {
		pr_err("%s: dpaux and dc must both be non-NULL", __func__);
		return;
	}

	dc = dpaux->dc;
	tegra_dc_unpowergate_locked(dc);
	tegra_dpaux_clk_en(dpaux);
	tegra_dc_io_start(dc);
	mutex_lock(&dpaux_lock);

	/*
	 * Make sure to configure the pad mode before we power it on. If not
	 * done in this order, there is a chance that the pad will run in the
	 * default mode for a while before switching to the requested mode. This
	 * could cause intermittent glitches on the physical lines.
	 */
	_tegra_dpaux_config_pad_mode(dpaux, mode);
	_tegra_dpaux_pad_power(dpaux, true);

	mutex_unlock(&dpaux_lock);
	tegra_dc_io_end(dc);
	tegra_dpaux_clk_dis(dpaux);
	tegra_dc_powergate_locked(dc);
}

void tegra_dpaux_prod_set(struct tegra_dc_dpaux_data *dpaux)
{
	struct tegra_dc *dc;

	if (!dpaux || !dpaux->dc) {
		pr_err("%s: dpaux and dc must both be non-NULL", __func__);
		return;
	}

	/* Only HDMI, DP, and fakeDP use DPAUX. */
	dc = dpaux->dc;
	if (dc->out->type != TEGRA_DC_OUT_HDMI &&
		dc->out->type != TEGRA_DC_OUT_DP &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		pr_err("%s: dc output type must be HDMI, DP, or fakeDP\n",
			__func__);
		return;
	}

	tegra_dc_unpowergate_locked(dc);
	tegra_dpaux_clk_en(dpaux);
	tegra_dc_io_start(dc);

	if (!IS_ERR_OR_NULL(dpaux->prod_list)) {
		char *prod_string = NULL;

		prod_string = dc->out->type == TEGRA_DC_OUT_HDMI ?
				"prod_c_dpaux_hdmi" : "prod_c_dpaux_dp";

		if (tegra_prod_set_by_name(&dpaux->base, prod_string,
							dpaux->prod_list)) {
			dev_warn(&dc->ndev->dev, "%s: dpaux prod set failed\n",
				__func__);
		}
	}

	tegra_dc_io_end(dc);
	tegra_dpaux_clk_dis(dpaux);
	tegra_dc_powergate_locked(dc);
}

struct tegra_dc_dpaux_data *tegra_dpaux_init_data(struct tegra_dc *dc)
{
	struct tegra_dc_dpaux_data *dpaux = NULL;
	struct device_node *np_dpaux = NULL;
	void __iomem *base = NULL;
	struct clk *clk = NULL;
	struct reset_control *rst = NULL;
	struct tegra_prod *prod_list = NULL;
	int sor_num = -1;
	int err = 0;
	char dpaux_name[CHAR_BUF_SIZE_MAX] = {0};
	bool need_rst = true;

	if (!dc) {
		pr_err("%s: dc must be non-NULL\n", __func__);
		return NULL;
	}

	/* Allocate memory for the dpaux struct. */
	dpaux = devm_kzalloc(&dc->ndev->dev, sizeof(*dpaux), GFP_KERNEL);
	if (!dpaux)
		return ERR_PTR(-ENOMEM);

	sor_num = tegra_dc_which_sor(dc);
	tegra_dpaux_get_name(dpaux_name, CHAR_BUF_SIZE_MAX, sor_num);

	/* Find the DPAUX node in DT based on the SOR instance. */
	np_dpaux = sor_num ? of_find_node_by_path(DPAUX1_NODE) :
				of_find_node_by_path(DPAUX_NODE);
	if (!np_dpaux || ((!of_device_is_available(np_dpaux)) &&
				(dc->out->type != TEGRA_DC_OUT_FAKE_DP))) {
		dev_err(&dc->ndev->dev, "%s: no dpaux node found\n", __func__);
		err = -ENODEV;

		goto err_put_dpaux_node;
	}

	/* ioremap the memory region for the DPAUX registers. */
	base = of_iomap(np_dpaux, 0);
	if (!base) {
		dev_err(&dc->ndev->dev, "%s: dpaux regs can't be mapped\n",
			__func__);
		err = -ENOENT;

		goto err_put_dpaux_node;
	}

	/* Query the DPAUX clock. */
#ifdef CONFIG_TEGRA_NVDISPLAY
	clk = tegra_disp_of_clk_get_by_name(np_dpaux, dpaux_name);
#else
	clk = clk_get_sys(NULL, dpaux_name);
#endif
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&dc->ndev->dev, "%s: %s clk unavailable\n", __func__,
			dpaux_name);
		err = -ENOENT;

		goto err_unmap_region;
	}

#ifndef CONFIG_TEGRA_NVDISPLAY
	/*
	 * All accesses to the DPAUX reset signal in the DP driver are wrapped
	 * in CONFIG_TEGRA_NVDISPLAY sections. Until that's cleaned up, set the
	 * "need_rst" flag to false to skip querying the reset for DP on
	 * pre-NVDISPLAY platforms.
	 */
	if (dc->out->type == TEGRA_DC_OUT_DP ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DP) {
		need_rst = false;
	}
#endif

	/* Extract the reset entry from the DT node. */
	if (tegra_bpmp_running() && need_rst) {
		rst = of_reset_control_get(np_dpaux, dpaux_name);
		if (IS_ERR_OR_NULL(rst)) {
			dev_err(&dc->ndev->dev,
				"%s: Unable to get %s reset control\n",
				__func__, dpaux_name);
			err = -ENOENT;

			goto err_put_clk;
		}

		reset_control_deassert(rst);
	}

	/* Get the PROD value lists that are defined for the DPAUX node. */
	prod_list = devm_tegra_prod_get_from_node(&dc->ndev->dev, np_dpaux);
	if (IS_ERR_OR_NULL(prod_list)) {
		dev_err(&dc->ndev->dev,
			"%s: prod list init failed for dpaux with error %ld\n",
			__func__, PTR_ERR(prod_list));
		err = -EINVAL;

		goto err_put_rst;
	}

	dpaux->dc = dc;
	dpaux->base = base;
	dpaux->clk = clk;
	dpaux->rst = rst;
	dpaux->prod_list = prod_list;

	return dpaux;

err_put_rst:
	if (rst)
		reset_control_put(rst);
err_put_clk:
#ifndef CONFIG_TEGRA_NVDISPLAY
	clk_put(clk);
#endif
err_unmap_region:
	iounmap(base);
err_put_dpaux_node:
	of_node_put(np_dpaux);
	devm_kfree(&dc->ndev->dev, dpaux);

	return ERR_PTR(err);
}

void tegra_dpaux_destroy_data(struct tegra_dc_dpaux_data *dpaux)
{
	struct device_node *np_dpaux;
	struct tegra_dc *dc;
	int sor_num = -1;

	if (!dpaux || !dpaux->dc) {
		pr_err("%s: dc and dpaux must be non-NULL\n", __func__);
		return;
	}

	dc = dpaux->dc;
	sor_num = tegra_dc_which_sor(dc);
	np_dpaux = sor_num ? of_find_node_by_path(DPAUX1_NODE) :
			of_find_node_by_path(DPAUX_NODE);

	dpaux->prod_list = NULL;
	if (dpaux->rst)
		reset_control_put(dpaux->rst);

#ifndef CONFIG_TEGRA_NVDISPLAY
	clk_put(dpaux->clk);
#endif

	iounmap(dpaux->base);

	devm_kfree(&dc->ndev->dev, dpaux);
	of_node_put(np_dpaux);
}
