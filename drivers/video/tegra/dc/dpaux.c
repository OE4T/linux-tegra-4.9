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
#include <linux/of_irq.h>
#include <linux/tegra_pm_domains.h>

#include "dpaux_regs.h"
#include "dc_priv.h"
#include "dpaux.h"
#include "dp.h"
#include "hdmi2.0.h"

static struct of_device_id tegra_dpaux_pd[] = {
	{ .compatible = "nvidia,tegra210-sor-pd", },
	{ .compatible = "nvidia,tegra186-disa-pd", },
	{},
};

static DEFINE_MUTEX(dpaux_lock);

static inline void tegra_dpaux_get_name(char *buf, size_t buf_len, int ctrl_num)
{
	if (ctrl_num > 0)
		snprintf(buf, buf_len, "dpaux%d", ctrl_num);
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
	tegra_unpowergate_partition(dpaux->powergate_id);
	tegra_dpaux_clk_en(dpaux);
	tegra_dc_io_start(dc);
	mutex_lock(&dpaux_lock);
	tegra_dpaux_prod_set(dpaux);
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
	tegra_powergate_partition(dpaux->powergate_id);
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

	tegra_unpowergate_partition(dpaux->powergate_id);
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
	tegra_powergate_partition(dpaux->powergate_id);
}

struct tegra_dc_dpaux_data *tegra_dpaux_init_data(struct tegra_dc *dc,
		struct device_node *sor_np)
{
	u32 temp;
	int err = 0;
	char dpaux_name[CHAR_BUF_SIZE_MAX] = {0};
	void __iomem *base = NULL;
	struct clk *clk = NULL;
	struct device_node *dpaux_np = NULL;
	struct reset_control *rst = NULL;
	struct tegra_prod *prod_list = NULL;
	struct tegra_dc_dpaux_data *dpaux = NULL;

	if (!dc || !sor_np) {
		pr_err("%s: err: %s cannot be NULL\n", __func__,
				!dc ? "dc" : "sor_np");
		return NULL;
	}

	dpaux_np = of_parse_phandle(sor_np, "nvidia,dpaux", 0);
	if (IS_ERR_OR_NULL(dpaux_np)) {
		dev_err(&dc->ndev->dev, "%s: could not find %s property for %s\n",
			__func__, "nvidia,dpaux", of_node_full_name(sor_np));
		return NULL;
	}

	if (!of_device_is_available(dpaux_np)) {
		dev_err(&dc->ndev->dev, "%s: %s present but disabled\n",
				__func__, of_node_full_name(dpaux_np));
		err = -ENODEV;
		goto exit;
	}

	/* Allocate memory for the dpaux struct. */
	dpaux = devm_kzalloc(&dc->ndev->dev, sizeof(*dpaux), GFP_KERNEL);
	if (!dpaux) {
		err = -ENOMEM;
		goto exit;
	}

	if (!of_property_read_u32(dpaux_np, "nvidia,dpaux-ctrlnum", &temp)) {
		dpaux->ctrl_num = (unsigned long)temp;
	} else {
		dev_err(&dc->ndev->dev, "mandatory property %s for %s not found\n",
				"nvidia,dpaux-ctrlnum",
				of_node_full_name(dpaux_np));
		goto release_mem;
	}

	tegra_dpaux_get_name(dpaux_name, CHAR_BUF_SIZE_MAX, dpaux->ctrl_num);

	/* ioremap the memory region for the DPAUX registers. */
	base = of_iomap(dpaux_np, 0);
	if (!base) {
		dev_err(&dc->ndev->dev, "%s: %s regs can't be mapped\n",
				__func__, of_node_full_name(dpaux_np));
		err = -ENOENT;
		goto release_mem;
	}

	/* Query the DPAUX clock. */
#ifdef CONFIG_TEGRA_NVDISPLAY
	clk = tegra_disp_of_clk_get_by_name(dpaux_np, dpaux_name);
#else
	clk = clk_get_sys(NULL, dpaux_name);
#endif
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&dc->ndev->dev, "%s: %s clk unavailable\n", __func__,
			dpaux_name);
		err = -ENOENT;

		goto err_unmap_region;
	}

	/* Extract the reset entry from the DT node. */
	if (tegra_bpmp_running()) {
		rst = of_reset_control_get(dpaux_np, dpaux_name);
		if (IS_ERR_OR_NULL(rst)) {
			dev_err(&dc->ndev->dev,
				"%s: Unable to get %s reset control\n",
				__func__, dpaux_name);
			err = -ENOENT;
			goto err_put_clk;
		}
		reset_control_deassert(rst);
	}

	if (!tegra_platform_is_sim()) {
		prod_list = devm_tegra_prod_get_from_node(
				&dc->ndev->dev, dpaux_np);
		if (IS_ERR_OR_NULL(prod_list)) {
			dev_err(&dc->ndev->dev,
				"%s: prod list init failed for dpaux with error %ld\n",
				__func__, PTR_ERR(prod_list));
			err = -EINVAL;
			goto err_put_rst;
		}
	}

	dpaux->powergate_id = tegra_pd_get_powergate_id(tegra_dpaux_pd);
	dpaux->dc = dc;
	dpaux->base = base;
	dpaux->clk = clk;
	dpaux->rst = rst;
	dpaux->prod_list = prod_list;
	dpaux->np = dpaux_np;

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
release_mem:
	devm_kfree(&dc->ndev->dev, dpaux);
exit:
	of_node_put(dpaux_np);
	return ERR_PTR(err);
}

int tegra_dpaux_get_irq(struct tegra_dc_dpaux_data *dpaux)
{
	int irq;

	if (!dpaux)
		return 0; /* return 0 for an error */

	irq = of_irq_to_resource(dpaux->np, 0, NULL);
	if (!irq)
		pr_err("%s: error getting irq\n", __func__);

	return irq;
}

struct clk *tegra_dpaux_get_clk(struct tegra_dc_dpaux_data *dpaux,
		const char *clk_name)
{
	if (!dpaux || !clk_name)
		return NULL;

#ifdef CONFIG_TEGRA_NVDISPLAY
	return tegra_disp_of_clk_get_by_name(dpaux->np, clk_name);
#elif defined(CONFIG_ARCH_TEGRA_210_SOC)
	return clk_get_sys(NULL, clk_name);
#endif
}

void tegra_dpaux_destroy_data(struct tegra_dc_dpaux_data *dpaux)
{
	struct tegra_dc *dc;

	if (!dpaux || !dpaux->dc) {
		pr_err("%s: %s must be non-NULL\n", __func__,
				!dpaux ? "dpaux" : "dc");
		return;
	}

	dc = dpaux->dc;
	if (dpaux->rst)
		reset_control_put(dpaux->rst);

#ifndef CONFIG_TEGRA_NVDISPLAY
	clk_put(dpaux->clk);
#endif

	iounmap(dpaux->base);

	devm_kfree(&dc->ndev->dev, dpaux);
}
