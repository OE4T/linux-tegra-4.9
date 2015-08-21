/*
 * eqos_ape_amisc.c
 *
 * AMISC register handling for eavb ape synchronization
 *
 * Copyright (C) 2016, NVIDIA Corporation. All rights reserved.
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


#include "eqos_ape_global.h"

void amisc_clk_init(struct eqos_drvdata *eqos_ape_drv_data)
{
	int ret = 0;
	int rate;
	struct platform_device *pdev = eqos_ape_drv_data->pdev;

	eqos_ape_drv_data->ape_clk = devm_clk_get(&pdev->dev, "eqos_ape.ape");
	if (IS_ERR_OR_NULL(eqos_ape_drv_data->ape_clk)) {
		dev_err(&pdev->dev, "Failed to find adsp.ape clk\n");
		ret = -EINVAL;
	}
	eqos_ape_drv_data->pll_a_out0_clk =
			devm_clk_get(&pdev->dev, "pll_a_out0");
	if (IS_ERR_OR_NULL(eqos_ape_drv_data->pll_a_out0_clk)) {
		dev_err(&pdev->dev, "Failed to find pll_a_out0_clk clk\n");
		ret = -EINVAL;
	}
	eqos_ape_drv_data->pll_a_clk = devm_clk_get(&pdev->dev, "pll_a");
	if (IS_ERR_OR_NULL(eqos_ape_drv_data->pll_a_clk)) {
		dev_err(&pdev->dev, "Failed to find pll_a clk\n");
		ret = -EINVAL;
	}

	eqos_ape_drv_data->ape_clk_parent =
			clk_get_parent(eqos_ape_drv_data->ape_clk);
	ret = clk_set_parent(eqos_ape_drv_data->ape_clk,
			eqos_ape_drv_data->pll_a_out0_clk);
	if (ret)
		dev_err(&pdev->dev, "Failed to set parent\n");

	ret = clk_prepare_enable(eqos_ape_drv_data->ape_clk);
	if (ret)
		dev_err(&pdev->dev, "Failed to enable ape_clk\n");

	eqos_ape_drv_data->pll_a_clk_rate =
			clk_get_rate(eqos_ape_drv_data->pll_a_clk);
	dev_dbg(&pdev->dev, "ape rate %d\n", eqos_ape_drv_data->pll_a_clk_rate);

	rate =  clk_get_rate(eqos_ape_drv_data->ape_clk);
	dev_dbg(&pdev->dev, "ape rate %d\n", rate);
}

void amisc_clk_deinit(struct eqos_drvdata *eqos_ape_drv_data)
{
	int err;
	struct platform_device *pdev = eqos_ape_drv_data->pdev;

	err = clk_set_rate(eqos_ape_drv_data->pll_a_clk,
			eqos_ape_drv_data->pll_a_clk_rate);

	err = clk_set_parent(eqos_ape_drv_data->ape_clk,
			eqos_ape_drv_data->ape_clk_parent);

	clk_disable_unprepare(eqos_ape_drv_data->ape_clk);

	devm_clk_put(&pdev->dev, eqos_ape_drv_data->pll_a_clk);

	devm_clk_put(&pdev->dev, eqos_ape_drv_data->pll_a_out0_clk);

	devm_clk_put(&pdev->dev, eqos_ape_drv_data->ape_clk);
}

int amisc_ape_get_rate(struct eqos_drvdata *eqos_ape_drv_data)
{
	int rate;

	rate = clk_get_rate(eqos_ape_drv_data->ape_clk);
	return rate;
}

void amisc_ape_set_rate(struct eqos_drvdata *eqos_ape_drv_data, int rate)
{
	int err;

	err = clk_set_rate(eqos_ape_drv_data->ape_clk, rate);
}

int amisc_plla_get_rate(struct eqos_drvdata *eqos_ape_drv_data)
{
	int rate;

	rate = clk_get_rate(eqos_ape_drv_data->pll_a_clk);
	return rate;
}

void amisc_plla_set_rate(struct eqos_drvdata *eqos_ape_drv_data, int rate)
{
	int err;

	err = clk_set_rate(eqos_ape_drv_data->pll_a_clk, rate);
}

