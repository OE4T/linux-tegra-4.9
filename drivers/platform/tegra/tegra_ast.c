/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION. All rights reserved.
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

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/tegra_ast.h>

#define TEGRA_APS_AST_CONTROL			0
#define TEGRA_APS_AST_STREAMID_CTL		0x20
#define TEGRA_APS_AST_REGION_0_SLAVE_BASE_LO	0x100
#define TEGRA_APS_AST_REGION_0_SLAVE_BASE_HI	0x104
#define TEGRA_APS_AST_REGION_0_MASK_LO		0x108
#define TEGRA_APS_AST_REGION_0_MASK_HI		0x10c
#define TEGRA_APS_AST_REGION_0_MASTER_BASE_LO	0x110
#define TEGRA_APS_AST_REGION_0_MASTER_BASE_HI	0x114
#define TEGRA_APS_AST_REGION_0_CONTROL		0x118
#define TEGRA_APS_AST_REGION_1_MASK_LO		0x128

#define AST_MAX_REGION			7
#define AST_ADDR_MASK			0xfffff000

/* TEGRA_APS_AST_CONTROL register fields */
#define AST_MATCH_ERR_CTRL		0x2

/* TEGRA_APS_AST_REGION_<x>_CONTROL register fieds */
#define AST_RGN_CTRL_NON_SECURE		0x8
#define AST_RGN_CTRL_SNOOP		0x4

#define AST_MAX_VMINDEX			15
#define AST_MAX_STREAMID		255
#define AST_STREAMID(id)		((id) << 8)
#define AST_STREAMID_CTL_ENABLE		0x01

/* TEGRA_APS_AST_REGION_<x>_SLAVE_BASE_LO register fields */
#define AST_SLV_BASE_LO_ENABLE		1

struct tegra_ast {
	struct device *dev;
	void __iomem *ast_base;
};

static inline u32 tegra_ast_region_offset(u32 region)
{
	const u32 region_stride =
		TEGRA_APS_AST_REGION_1_MASK_LO - TEGRA_APS_AST_REGION_0_MASK_LO;

	return region * region_stride;
}

struct tegra_ast *tegra_ast_add_ref(const struct device_node *client_np,
			const char *ast_prop_name, u32 ast_prop_index)
{
	struct device_node *ast_node;
	struct platform_device *pdev;
	struct tegra_ast* ast;

	if (!client_np || !ast_prop_name)
		return ERR_PTR(-EINVAL);

	ast_node = of_parse_phandle(client_np, ast_prop_name, ast_prop_index);
	if (!ast_node)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(ast_node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	ast = dev_get_drvdata(&pdev->dev);
	if (!ast)
		return ERR_PTR(-EPROBE_DEFER);

	return ast;
}
EXPORT_SYMBOL(tegra_ast_add_ref);

int tegra_ast_del_ref(void)
{
	return 0;
}
EXPORT_SYMBOL(tegra_ast_del_ref);

int tegra_ast_set_streamid(struct tegra_ast *ast,
				u32 vmindex, u32 stream_id)
{
	if (!ast)
		return -EINVAL;

	if (vmindex > AST_MAX_VMINDEX) {
		dev_err(ast->dev,
			"%s: Invalid VM index %u\n",
			__func__, vmindex);
		return -EINVAL;
	}

	if (stream_id > AST_MAX_STREAMID) {
		dev_err(ast->dev,
			"%s: Invalid streamID %u\n",
			__func__, stream_id);
		return -EINVAL;
	}

	writel(AST_STREAMID(stream_id) | AST_STREAMID_CTL_ENABLE,
		ast->ast_base + TEGRA_APS_AST_STREAMID_CTL + (vmindex * 4));

	return 0;
}
EXPORT_SYMBOL(tegra_ast_set_streamid);

int tegra_ast_region_enable(struct tegra_ast *ast, u32 region,
			u32 slave_base, u32 mask, u64 master_base)
{
	u32 roffset;
	void __iomem *ast_base;

	if (!ast)
		return -EINVAL;

	if (region > AST_MAX_REGION) {
		dev_err(ast->dev,
			"%s: Invalid region\n",
			__func__);
		return -EINVAL;
	}

	roffset = tegra_ast_region_offset(region);
	ast_base = ast->ast_base;

	writel(AST_MATCH_ERR_CTRL, ast_base + TEGRA_APS_AST_CONTROL);

	writel(0, ast_base + TEGRA_APS_AST_REGION_0_MASK_HI + roffset);

	writel(mask & AST_ADDR_MASK,
	       ast_base + TEGRA_APS_AST_REGION_0_MASK_LO + roffset);

	writel(0, ast_base + TEGRA_APS_AST_REGION_0_MASTER_BASE_HI + roffset);

	writel(master_base & AST_ADDR_MASK,
		ast_base + TEGRA_APS_AST_REGION_0_MASTER_BASE_LO + roffset);

	writel(AST_RGN_CTRL_NON_SECURE | AST_RGN_CTRL_SNOOP,
		ast_base + TEGRA_APS_AST_REGION_0_CONTROL + roffset);

	writel(0, ast_base + TEGRA_APS_AST_REGION_0_SLAVE_BASE_HI + roffset);

	writel((slave_base & AST_ADDR_MASK) | AST_SLV_BASE_LO_ENABLE,
		ast_base + TEGRA_APS_AST_REGION_0_SLAVE_BASE_LO + roffset);

	return 0;
}
EXPORT_SYMBOL(tegra_ast_region_enable);

int tegra_ast_region_disable(struct tegra_ast *ast, u32 region)
{
	u32 roffset;
	void __iomem *ast_base;

	if (!ast)
		return -EINVAL;

	if (region > AST_MAX_REGION) {
		dev_err(ast->dev,
			"%s: Invalid region\n",
			__func__);
		return -EINVAL;
	}

	roffset = tegra_ast_region_offset(region);
	ast_base = ast->ast_base;

	writel(0, ast_base + TEGRA_APS_AST_REGION_0_SLAVE_BASE_LO + roffset);

	return 0;
}
EXPORT_SYMBOL(tegra_ast_region_disable);

static int tegra_ast_probe(struct platform_device *pdev)
{
	struct tegra_ast *ast;

	dev_dbg(&pdev->dev, "AST driver probe Start\n");

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev,
			"%s: No DT node found for AST\n",
			__func__);
		return -ENODEV;
	}

	ast = devm_kzalloc(&pdev->dev, sizeof(*ast), GFP_KERNEL);
	if (!ast)
		return -ENOMEM;

	ast->ast_base = of_iomap(pdev->dev.of_node, 0);
	dev_set_drvdata(&pdev->dev, ast);
	ast->dev = &pdev->dev;

	dev_dbg(&pdev->dev, "AST driver probe() OK\n");

	return 0;
}

static int tegra_ast_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id tegra_ast_of_match[] = {
	{
		.compatible = "nvidia,tegra186-ast",
	},
	{},
};
MODULE_DEVICE_TABLE(of, tegra_ast_of_match);

static struct platform_driver tegra_ast_driver = {
	.driver = {
		.name   = "tegra-ast",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_ast_of_match),
	},
	.probe = tegra_ast_probe,
	.remove = tegra_ast_remove,
};
module_platform_driver(tegra_ast_driver);
