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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/tegra_ast.h>
#include <linux/io.h>

#define TEGRA_APS_AST_STREAMID_CTL		0x20
#define TEGRA_APS_AST_REGION_0_SLAVE_BASE_LO	0x100
#define TEGRA_APS_AST_REGION_0_SLAVE_BASE_HI	0x104
#define TEGRA_APS_AST_REGION_0_MASK_LO		0x108
#define TEGRA_APS_AST_REGION_0_MASK_HI		0x10c
#define TEGRA_APS_AST_REGION_0_MASTER_BASE_LO	0x110
#define TEGRA_APS_AST_REGION_0_MASTER_BASE_HI	0x114
#define TEGRA_APS_AST_REGION_0_CONTROL		0x118

#define TEGRA_APS_AST_REGION_STRIDE		0x20

#define AST_MAX_REGION			7
#define AST_ADDR_MASK			0xfffff000
#define AST_ADDR_MASK64			(~0xfffULL)

/* TEGRA_APS_AST_REGION_<x>_CONTROL register fields */
#define AST_RGN_CTRL_VM_INDEX		15
#define AST_RGN_CTRL_NON_SECURE		0x8
#define AST_RGN_CTRL_SNOOP		0x4

#define AST_MAX_VMINDEX			15
#define AST_MAX_STREAMID		255
#define AST_STREAMID(id)		((id) << 8u)
#define AST_STREAMID_CTL_ENABLE		0x01

/* TEGRA_APS_AST_REGION_<x>_SLAVE_BASE_LO register fields */
#define AST_SLV_BASE_LO_ENABLE		1

int tegra_ast_map(struct device *dev, const char *name,
			unsigned count, void __iomem *bases[])
{
	int err, i;

	if (dev->of_node == NULL)
		return -EINVAL;

	/* AST regions 0 and 1 are used for DRAM and SYSRAM carveouts */
	for (i = 0; i < count; i++) {
		struct device_node *node;
		void __iomem *base;
		struct resource res;

		node = of_parse_phandle(dev->of_node, name, i);
		if (node == NULL) {
			err = -EINVAL;
			goto error;
		}

		err = of_address_to_resource(node, 0, &res);
		if (err)
			goto error;

		base = devm_ioremap(dev, res.start, resource_size(&res));
		if (base == NULL) {
			err = -ENOMEM;
			goto error;
		}

		bases[i] = base;
	}
	return 0;

error:
	tegra_ast_unmap(dev, i, bases);
	return err;
}
EXPORT_SYMBOL(tegra_ast_map);

void tegra_ast_unmap(struct device *dev,
			unsigned count, void __iomem *const bases[])
{
	while (count > 0)
		devm_iounmap(dev, bases[--count]);
}
EXPORT_SYMBOL(tegra_ast_unmap);

int tegra_ast_region_enable(unsigned count, void __iomem *const bases[],
				u32 region, u32 slave_base, u32 size,
				u64 master_base, u32 stream_id)
{
	u32 offset = region * TEGRA_APS_AST_REGION_STRIDE;
	u32 mask = size - 1;
	u32 ast_sid = AST_STREAMID(stream_id);
	u32 vmidx;
	unsigned i;

	if (region > AST_MAX_REGION || stream_id > AST_MAX_STREAMID)
		return -EINVAL;
	if (size & mask)
		return -EOPNOTSUPP;

	for (i = 0; i < count; i++) {
		for (vmidx = 0; vmidx <= AST_MAX_VMINDEX; vmidx++) {
			u32 r = readl(bases[i] + TEGRA_APS_AST_STREAMID_CTL +
					(4 * vmidx));
			if ((r & 0x0000ff00) == ast_sid)
				break;
		}

		if (vmidx > AST_MAX_VMINDEX) {
			/*
			 * TBD: will be replaced with return error once MB1
			 * sets stream_id at least one of vmidx table
			 */
			for (vmidx = 0; vmidx <= AST_MAX_VMINDEX; vmidx++) {
				u32 r = readl(bases[i] +
						TEGRA_APS_AST_STREAMID_CTL +
						(4 * vmidx));
				if (r & 0x0000ff00)
					break;
			}

			if (vmidx > AST_MAX_VMINDEX)
				goto error;

			writel(ast_sid | AST_STREAMID_CTL_ENABLE, bases[i] +
				TEGRA_APS_AST_STREAMID_CTL + (4 * vmidx));
		}

		writel(0, bases[i] + TEGRA_APS_AST_REGION_0_MASK_HI + offset);
		writel(mask & AST_ADDR_MASK, bases[i] +
			TEGRA_APS_AST_REGION_0_MASK_LO + offset);

		writel(0, bases[i] +
			TEGRA_APS_AST_REGION_0_MASTER_BASE_HI + offset);
		writel(master_base & AST_ADDR_MASK, bases[i] +
			TEGRA_APS_AST_REGION_0_MASTER_BASE_LO + offset);

		writel(AST_RGN_CTRL_NON_SECURE | AST_RGN_CTRL_SNOOP |
				(vmidx << AST_RGN_CTRL_VM_INDEX),
			bases[i] + TEGRA_APS_AST_REGION_0_CONTROL + offset);

		writel(0, bases[i] +
			TEGRA_APS_AST_REGION_0_SLAVE_BASE_HI + offset);
		writel((slave_base & AST_ADDR_MASK) | AST_SLV_BASE_LO_ENABLE,
			bases[i] +
			TEGRA_APS_AST_REGION_0_SLAVE_BASE_LO + offset);
	}

	return 0;
error:
	tegra_ast_region_disable(i, bases, region);
	return -EBUSY;
}
EXPORT_SYMBOL(tegra_ast_region_enable);

void tegra_ast_region_disable(unsigned count, void __iomem *const bases[],
				u32 region)
{
	u32 offset = region * TEGRA_APS_AST_REGION_STRIDE;

	while (count > 0)
		writel(0, bases[--count] +
			TEGRA_APS_AST_REGION_0_SLAVE_BASE_LO + offset);
}
EXPORT_SYMBOL(tegra_ast_region_disable);

void tegra_ast_get_region_info(void __iomem *base,
			u32 region,
			struct tegra_ast_region_info *info)
{
	u32 offset = region * TEGRA_APS_AST_REGION_STRIDE;
	u32 vmidx, stream_id, control;
	u64 lo, hi;

	control = readl(base + TEGRA_APS_AST_REGION_0_CONTROL + offset);
	info->control = control;

	info->lock = (control & BIT(0)) != 0;
	info->snoop = (control & BIT(2)) != 0;
	info->non_secure = (control & BIT(3)) != 0;
	info->ns_passthru = (control & BIT(4)) != 0;
	info->carveout_id = (control >> 5) & (0x1f);
	info->carveout_al = (control >> 10) & 0x3;
	info->vpr_rd = (control & BIT(12)) != 0;
	info->vpr_wr = (control & BIT(13)) != 0;
	info->vpr_passthru = (control & BIT(14)) != 0;
	vmidx = (control >> AST_RGN_CTRL_VM_INDEX) & 0xf;
	info->vm_index = vmidx;
	info->physical = (control & BIT(19)) != 0;
	stream_id = readl(base + TEGRA_APS_AST_STREAMID_CTL + (4 * vmidx));
	info->stream_id = stream_id >> 8;
	info->stream_id_enabled = (stream_id & BIT(0)) != 0;

	lo = readl(base + TEGRA_APS_AST_REGION_0_SLAVE_BASE_LO + offset);
	hi = readl(base + TEGRA_APS_AST_REGION_0_SLAVE_BASE_HI + offset);

	info->slave = ((hi << 32U) + lo) & AST_ADDR_MASK64;
	info->enabled = (lo & BIT(0)) != 0;

	hi = readl(base + TEGRA_APS_AST_REGION_0_MASK_HI + offset);
	lo = readl(base + TEGRA_APS_AST_REGION_0_MASK_LO + offset);

	info->mask = ((hi << 32) + lo) | ~AST_ADDR_MASK64;

	hi = readl(base + TEGRA_APS_AST_REGION_0_MASTER_BASE_HI + offset);
	lo = readl(base + TEGRA_APS_AST_REGION_0_MASTER_BASE_LO + offset);

	info->master = ((hi << 32U) + lo) & AST_ADDR_MASK64;
}
EXPORT_SYMBOL(tegra_ast_get_region_info);
