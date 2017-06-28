/*
 * SLVS-EC driver for T194
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
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

#include "slvsec.h"

#include <asm/ioctls.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <media/mc_common.h>
#include <media/csi.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t194/t194.h"

#define SLVSEC_CORE_INTR_STATUS			U32_C(0x84)
#define SLVSEC_CORE_INTR_STATUS_SW		BIT(4)
#define SLVSEC_CORE_INTR_STATUS_HOST1X		BIT(3)
#define SLVSEC_CORE_INTR_STATUS_CIL		BIT(2)
#define SLVSEC_CORE_INTR_STATUS_STRM_1		BIT(1)
#define SLVSEC_CORE_INTR_STATUS_STRM_0		BIT(0)

#define SLVSEC_CORE_HOST1X_INTR_STATUS		U32_C(0xb0)
#define SLVSEC_CORE_HOST1X_INTR_MASK		U32_C(0xb4)

#define SLVSEC_CORE_SW_DEBUG_INTR_STATUS	U32_C(0xb8)
#define SLVSEC_CORE_SW_DEBUG_INTR_TRIG		U32_C(0xbc)

#define SLVSEC_CORE_STRM0_INTR_STATUS_CH0	U32_C(0x10048)
#define SLVSEC_CORE_STRM0_INTR_STATUS_CH1	U32_C(0x1004c)
#define SLVSEC_CORE_STRM0_INTR_STATUS		U32_C(0x10050)

#define SLVSEC_CORE_STRM1_INTR_STATUS_CH0	U32_C(0x20048)
#define SLVSEC_CORE_STRM1_INTR_STATUS_CH1	U32_C(0x2004c)
#define SLVSEC_CORE_STRM1_INTR_STATUS		U32_C(0x20050)

#define SLVSEC_CIL_STRM0_INTR_STATUS		U32_C(0x30828)
#define SLVSEC_CIL_STRM1_INTR_STATUS		U32_C(0x31028)

struct slvsec {
	struct platform_device *pdev;
	struct regulator *regulator;
	int irq;

	/* Debugfs */
	struct slvsec_debug {
		struct debugfs_regset32 core;
		struct debugfs_regset32 core_stream0;
		struct debugfs_regset32 core_stream1;
		struct debugfs_regset32 cil;
	} debug;
};

static int slvsec_init_debugfs(struct slvsec *slvsec);
static void slvsec_remove_debugfs(struct slvsec *slvsec);

static irqreturn_t slvsec_isr(int irq, void *arg)
{
	struct platform_device *pdev = arg;
	u32 status, val;

	status = host1x_readl(pdev, SLVSEC_CORE_INTR_STATUS);
	dev_info(&pdev->dev, "INTR: %08x\n", status);

	if (status & SLVSEC_CORE_INTR_STATUS_SW) {
		val = host1x_readl(pdev, SLVSEC_CORE_SW_DEBUG_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CORE_SW_DEBUG_INTR_STATUS, val);
		dev_info(&pdev->dev, "SW_DEBUG_INTR: %08x\n", val);
	}

	if (status & SLVSEC_CORE_INTR_STATUS_HOST1X) {
		val = host1x_readl(pdev, SLVSEC_CORE_HOST1X_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CORE_HOST1X_INTR_STATUS, val);
		dev_info(&pdev->dev, "HOST1X_INTR: %08x\n", val);
	}

	if (status & SLVSEC_CORE_INTR_STATUS_CIL) {
		val = host1x_readl(pdev, SLVSEC_CIL_STRM0_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CIL_STRM0_INTR_STATUS, val);
		dev_info(&pdev->dev, "CIL_STRM0_INTR: %08x\n", val);
		val = host1x_readl(pdev, SLVSEC_CIL_STRM1_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CIL_STRM1_INTR_STATUS, val);
		dev_info(&pdev->dev, "CIL_STRM1_INTR: %08x\n", val);
	}

	if (status & SLVSEC_CORE_INTR_STATUS_STRM_1) {
		val = host1x_readl(pdev, SLVSEC_CORE_STRM1_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CORE_STRM1_INTR_STATUS, val);
		dev_info(&pdev->dev, "CORE_STRM1_INTR: %08x\n", val);

		val = host1x_readl(pdev, SLVSEC_CORE_STRM1_INTR_STATUS_CH0);
		host1x_writel(pdev, SLVSEC_CORE_STRM1_INTR_STATUS_CH0, val);
		dev_info(&pdev->dev, "CORE_STRM1_CH0_INTR: %08x\n", val);

		val = host1x_readl(pdev, SLVSEC_CORE_STRM1_INTR_STATUS_CH1);
		host1x_writel(pdev, SLVSEC_CORE_STRM1_INTR_STATUS_CH1, val);
		dev_info(&pdev->dev, "CORE_STRM1_CH1_INTR: %08x\n", val);
	}

	if (status & SLVSEC_CORE_INTR_STATUS_STRM_0) {
		val = host1x_readl(pdev, SLVSEC_CORE_STRM0_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CORE_STRM0_INTR_STATUS, val);
		dev_info(&pdev->dev, "CORE_STRM0_INTR: %08x\n", val);

		val = host1x_readl(pdev, SLVSEC_CORE_STRM0_INTR_STATUS_CH0);
		host1x_writel(pdev, SLVSEC_CORE_STRM0_INTR_STATUS_CH0, val);
		dev_info(&pdev->dev, "CORE_STRM0_CH0_INTR: %08x\n", val);

		val = host1x_readl(pdev, SLVSEC_CORE_STRM0_INTR_STATUS_CH1);
		host1x_writel(pdev, SLVSEC_CORE_STRM0_INTR_STATUS_CH1, val);
		dev_info(&pdev->dev, "CORE_STRM0_CH1_INTR: %08x\n", val);
	}

	return IRQ_HANDLED;
}

int slvsec_finalize_poweron(struct platform_device *pdev)
{
	struct slvsec *slvsec = nvhost_get_private_data(pdev);
	int ret;

	if (!slvsec->regulator)
		return 0;

	ret = regulator_enable(slvsec->regulator);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable slvs-ec regulator.");
		return ret;
	}

	return 0;
}

int slvsec_prepare_poweroff(struct platform_device *pdev)
{
	struct slvsec *slvsec = nvhost_get_private_data(pdev);
	int ret;

	if (!slvsec->regulator)
		return 0;

	ret = regulator_disable(slvsec->regulator);
	if (ret)
		dev_err(&pdev->dev, "failed to disable slvs-ec regulator.");

	return 0;
}

static int slvsec_probe_regulator(struct slvsec *slvsec)
{
	struct device *dev = &slvsec->pdev->dev;
	const char *name;
	struct regulator *regulator;
	int err;

	err = of_property_read_string(dev->of_node, "nvidia,regulator", &name);
	if (err == -EINVAL)
		return 0;
	if (err)
		return err;

	regulator = devm_regulator_get(dev, name);
	if (IS_ERR(regulator))
		return PTR_ERR(regulator);

	slvsec->regulator = regulator;

	return 0;
}

static int slvsec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *info;
	struct slvsec *slvsec;
	int err = 0;

	info = (void *)of_device_get_match_data(&pdev->dev);
	if (unlikely(info == NULL)) {
		dev_WARN(&pdev->dev, "no platform data\n");
		return -ENODATA;
	}

	slvsec = devm_kzalloc(dev, sizeof(*slvsec), GFP_KERNEL);
	if (!slvsec)
		return -ENOMEM;

	slvsec->pdev = pdev;
	info->pdev = pdev;
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);
	info->private_data = slvsec;

	err = slvsec_probe_regulator(slvsec);
	if (err)
		dev_info(dev, "failed to get regulator (%d)\n", err);

	slvsec->irq = of_irq_get_byname(dev->of_node, "slvs-ec");
	if (slvsec->irq <= 0) {
		dev_err(dev, "No IRQ available\n");
		slvsec->irq = -ENXIO;
	} else {
		err = devm_request_threaded_irq(dev, slvsec->irq,
						NULL, slvsec_isr, IRQF_ONESHOT,
						dev_name(dev), pdev);
		if (err) {
			dev_err(dev, "Cannot setup irq\n");
			slvsec->irq = -ENXIO;
		}
	}

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		return err;

	err = nvhost_module_init(pdev);
	if (err)
		return err;

	err = nvhost_client_device_init(pdev);
	if (err)
		goto deinit;

	slvsec_init_debugfs(slvsec);

	dev_info(dev, "clearing pending interrupts\n");
	slvsec_isr(slvsec->irq, pdev);

	dev_info(dev, "probed\n");

	return 0;

deinit:
	dev_err(dev, "probe failed: %d\n", err);
	nvhost_module_deinit(pdev);
	return err;
}

static int __exit slvsec_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct slvsec *slvsec = (struct slvsec *)pdata->private_data;

	slvsec_remove_debugfs(slvsec);

	return 0;
}

static const struct of_device_id tegra_slvsec_of_match[] = {
	{
		.compatible = "nvidia,tegra-slvs-ec",
		.data = &t19_slvsec_info,
	},
	{ },
};

static struct platform_driver slvsec_driver = {
	.probe = slvsec_probe,
	.remove = __exit_p(slvsec_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra-slvs-ec",
#ifdef CONFIG_OF
		.of_match_table = tegra_slvsec_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

static const struct debugfs_reg32 slvsec_core_regs[] = {
	{ .name = "slvsec_id", 0x80 },
	{ .name = "intr_status", 0x84 },
	{ .name = "dbg_cnt0_ctrl", 0x8c },
	{ .name = "dbg_counter0", 0x90 },
	{ .name = "dbg_cnt1_ctrl", 0x94 },
	{ .name = "dbg_counter1", 0x98 },
	{ .name = "dbg_cnt2_ctrl", 0x9C },
	{ .name = "dbg_counter2", 0xA0 },
	{ .name = "dbg_cnt3_ctrl", 0xA4 },
	{ .name = "dbg_counter3", 0xA8 },
};

static const struct debugfs_reg32 slvsec_core_strm_regs[] = {
	{ .name = "ctrl", 0x0 },
	{ .name = "rst_ctrl", 0x4 },
	{ .name = "clk_ctrl", 0x8 },
	{ .name = "ch0_cfg", 0x0c },
	{ .name = "ch1_cfg", 0x10 },
	{ .name = "ch0_embd_cfg", 0x14 },
	{ .name = "ch1_embd_cfg", 0x18 },
	{ .name = "timeout_ctrl", 0x1C },
	{ .name = "ph_rx_crc0", 0x20 },
	{ .name = "ph_rx_crc1", 0x24 },
	{ .name = "ph_rx_crc2", 0x28 },
	{ .name = "pd_rx_crc", 0x2c },
	{ .name = "pd_cal_crc", 0x30 },
	{ .name = "vi_err_mask_ch0", 0x34 },
	{ .name = "vi_err_mask_ch1", 0x38 },
	{ .name = "intr_mask_ch0", 0x3c },
	{ .name = "intr_mask_ch1", 0x40 },
	{ .name = "intr_mask", 0x44 },
	{ .name = "intr_status_ch0", 0x48 },
	{ .name = "intr_status_ch1", 0x4c },
	{ .name = "intr_status", 0x50 },
};

static const struct debugfs_reg32 slvsec_cil_regs[] = {
	{ .name = "ctrl", 0x0 },
	{ .name = "lane_swizzle", 0x4 },
	{ .name = "uphy_ctrl0", 0x8 },
	{ .name = "uphy_ctrl1", 0xC },
	{ .name = "uphy_ctrl2", 0x10 },
	{ .name = "timeout_ctrl", 0x14 },
	{ .name = "padctrl_rst_ctrl", 0x18 },
};

static int slvsec_sw_debug_intr_show(void *data, u64 *val)
{
	struct platform_device *pdev = data;

	*val = host1x_readl(pdev, SLVSEC_CORE_SW_DEBUG_INTR_STATUS);

	return 0;
}

static int slvsec_sw_debug_intr_store(void *data, u64 val)
{
	struct platform_device *pdev = data;

	if (val > (u32)~U32_C(0))
		return -EINVAL;

	host1x_writel(pdev, SLVSEC_CORE_SW_DEBUG_INTR_TRIG, (u32)val);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(slvsec_sw_debug_intr_fops,
			slvsec_sw_debug_intr_show,
			slvsec_sw_debug_intr_store,
			"%llu\n");

static int slvsec_init_debugfs(struct slvsec *slvsec)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(slvsec->pdev);
	struct dentry *dir = pdata->debugfs;
	struct slvsec_debug *debug = &slvsec->debug;

	/* Trigger or clear SW interrupts */
	debugfs_create_file("sw-intr", S_IRUGO | S_IWUSR, dir, slvsec->pdev,
			&slvsec_sw_debug_intr_fops);

	debug->core.base = pdata->aperture[0];
	debug->core.regs = slvsec_core_regs;
	debug->core.nregs = ARRAY_SIZE(slvsec_core_regs);
	debugfs_create_regset32("core", S_IRUGO, dir, &debug->core);

	debug->core_stream0.base = pdata->aperture[0] + 0x10000;
	debug->core_stream0.regs = slvsec_core_strm_regs;
	debug->core_stream0.nregs = ARRAY_SIZE(slvsec_core_strm_regs);
	debugfs_create_regset32("stream0", S_IRUGO, dir, &debug->core_stream0);

	debug->core_stream1.base = pdata->aperture[0] + 0x20000;
	debug->core_stream1.regs = slvsec_core_strm_regs;
	debug->core_stream1.nregs = ARRAY_SIZE(slvsec_core_strm_regs);
	debugfs_create_regset32("stream1", S_IRUGO, dir, &debug->core_stream1);

	debug->cil.base = pdata->aperture[0] + 0x30000;
	debug->cil.regs = slvsec_cil_regs;
	debug->cil.nregs = ARRAY_SIZE(slvsec_cil_regs);
	debugfs_create_regset32("cil", S_IRUGO, dir, &debug->cil);

	return 0;
}

static void slvsec_remove_debugfs(struct slvsec *slvsec)
{
}

#ifdef CONFIG_PM_GENERIC_DOMAINS
static struct of_device_id tegra_slvsec_domain_match[] = {
	{.compatible = "nvidia,tegra186-ve-pd",
	.data = &t19_slvsec_info},
	{},
};
#endif

static int __init slvsec_init(void)
{
#ifdef CONFIG_PM_GENERIC_DOMAINS
	int ret;

	ret = nvhost_domain_init(tegra_slvsec_domain_match);
	if (ret)
		return ret;
#endif

	return platform_driver_register(&slvsec_driver);
}

static void __exit slvsec_exit(void)
{
	platform_driver_unregister(&slvsec_driver);
}

late_initcall(slvsec_init);
module_exit(slvsec_exit);
