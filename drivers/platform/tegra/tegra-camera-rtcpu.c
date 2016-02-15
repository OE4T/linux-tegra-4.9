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
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/tegra-ivc-bus.h>

#include <dt-bindings/memory/tegra-swgroup.h>
#include <dt-bindings/memory/tegra186-swgroup.h>

/* Register specifics */
#define TEGRA_APS_FRSC_SC_CTL_0			0x0
#define TEGRA_SCE_APS_FRSC_SC_MODEIN_0		0x14
#define TEGRA_SCE_EVP_RESET_ADDR_0		0x20
#define TEGRA_SCEPM_R5_CTRL_0			0x40

#define TEGRA_SCE_R5R_SC_DISABLE		0x5
#define TEGRA_SCE_FN_MODEIN			0x29527
#define TEGRA_SCE_FWLOADDONE			0x2

static const char * const sce_clock_names[] = {
	"sce-apb",
};

static const char * const sce_reset_names[] = {
	"sce-apb",
	"sce-nsysporeset",
	"sce-nreset",
	"sce-dbgresetn",
	"sce-presetdbgn",
	"sce-actmon",
	"sce-pm",
	"sce-dma",
	"sce-hsp",
	"tsctnsce",
	"sce-tke",
	"sce-gte",
	"sce-cfg",
};

static const char * const ape_clock_names[] = {
	"ahub",
	"apb2ape",
	"ape",
	"adsp",
	"adspneon",
};

static const char * const ape_reset_names[] = {
	"adspdbg",
	"adspintf",
	"adspneon",
	"adspperiph",
	"adspscu",
	"adspwdt",
	"ape",
	"adsp",
};

static struct clk *sce_clocks[ARRAY_SIZE(sce_clock_names)];
static struct clk *ape_clocks[ARRAY_SIZE(sce_clock_names)];
static struct reset_control *sce_resets[ARRAY_SIZE(sce_reset_names)];
static struct reset_control *ape_resets[ARRAY_SIZE(ape_reset_names)];

enum tegra_cam_rtcpu_id {
	TEGRA_CAM_RTCPU_SCE,
	TEGRA_CAM_RTCPU_APE,
};

struct tegra_cam_rtcpu_pdata {
	const char *rtcpu_name;
	const char * const *clock_names;
	const char * const *reset_names;
	struct clk **clocks;
	struct reset_control **resets;
	enum tegra_cam_rtcpu_id id;
	u32 sid;
	u32 num_clocks;
	u32 num_resets;
};

static const struct tegra_cam_rtcpu_pdata sce_pdata = {
	.rtcpu_name = "sce",
	.clock_names = sce_clock_names,
	.reset_names = sce_reset_names,
	.clocks = sce_clocks,
	.resets = sce_resets,
	.id = TEGRA_CAM_RTCPU_SCE,
	.sid = TEGRA_SID_SCE,
	.num_clocks = ARRAY_SIZE(sce_clock_names),
	.num_resets = ARRAY_SIZE(sce_reset_names),
};

static const struct tegra_cam_rtcpu_pdata ape_pdata = {
	.rtcpu_name = "ape",
	.clock_names = ape_clock_names,
	.reset_names = ape_reset_names,
	.clocks = ape_clocks,
	.resets = ape_resets,
	.id = TEGRA_CAM_RTCPU_APE,
	.sid = TEGRA_SID_APE,
	.num_clocks = ARRAY_SIZE(ape_clock_names),
	.num_resets = ARRAY_SIZE(ape_reset_names),
};

#define NV(p) "nvidia," #p

static const struct of_device_id tegra_cam_rtcpu_of_match[] = {
	{
		.compatible = NV(tegra186-sce-ivc), .data = &sce_pdata
	},
	{
		.compatible = NV(tegra186-ape-ivc), .data = &ape_pdata
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_cam_rtcpu_of_match);

struct tegra_cam_rtcpu {
	struct tegra_ivc_bus *ivc;
	union {
		struct {
			void __iomem *sce_cfg_base;
			void __iomem *sce_arsce_evp;
			void __iomem *sce_pm_base;
		} __packed rtcpu_sce;
	};
	const struct tegra_cam_rtcpu_pdata *rtcpu_pdata;
};

static int tegra_cam_rtcpu_get_clks_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	int i;

	cam_rtcpu = dev_get_drvdata(dev);
	pdata = cam_rtcpu->rtcpu_pdata;

	/* Get clocks and resets */
	for (i = 0; i < pdata->num_clocks; i++) {
		pdata->clocks[i] = devm_clk_get(dev, pdata->clock_names[i]);
		if (IS_ERR(pdata->clocks[i])) {
			dev_err(dev, "clock %s not found: %ld\n",
				pdata->clock_names[i],
				PTR_ERR(pdata->clocks[i]));
			if (PTR_ERR(pdata->clocks[i]) == -EPROBE_DEFER)
				return PTR_ERR(pdata->clocks[i]);
		}
	}

	for (i = 0; i < pdata->num_resets; i++) {
		pdata->resets[i] =
			devm_reset_control_get(dev, pdata->reset_names[i]);
		if (IS_ERR(pdata->resets[i])) {
			dev_err(dev, "reset %s not found: %ld\n",
				pdata->reset_names[i],
				PTR_ERR(pdata->resets[i]));
			if (PTR_ERR(pdata->resets[i]) == -EPROBE_DEFER)
				return PTR_ERR(pdata->resets[i]);
		}
	}

	return 0;
}

static int tegra_cam_rtcpu_apply_clks(struct device *dev,
				int (*func)(struct clk *clk))
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	int i, ret;

	cam_rtcpu = dev_get_drvdata(dev);
	pdata = cam_rtcpu->rtcpu_pdata;

	for (i = 0; i < pdata->num_clocks; i++) {
		if (IS_ERR(pdata->clocks[i]))
			continue;
		ret = (*func)(pdata->clocks[i]);
		if (ret) {
			dev_err(dev, "clock %s failed: %d\n",
				pdata->clock_names[i], ret);
		}
	}

	return 0;
}

static int tegra_cam_rtcpu_apply_resets(struct device *dev,
			int (*func)(struct reset_control *))
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	int i, ret;

	cam_rtcpu = dev_get_drvdata(dev);
	pdata = cam_rtcpu->rtcpu_pdata;

	for (i = 0; i < pdata->num_resets; i++) {
		if (IS_ERR(pdata->resets[i]))
			continue;
		ret = (*func)(pdata->resets[i]);
		if (ret) {
			dev_err(dev, "reset %s failed: %d\n",
				pdata->reset_names[i], ret);
		}
	}

	return 0;
}

static void tegra_cam_rtcpu_boot(struct device *dev)
{
	struct tegra_cam_rtcpu *cam_rtcpu = dev_get_drvdata(dev);
	u32 reg_val;

	if (cam_rtcpu->rtcpu_pdata->id == TEGRA_CAM_RTCPU_SCE) {
		/* Disable SCE R5R and smartcomp in camera mode */
		writel(TEGRA_SCE_R5R_SC_DISABLE,
			cam_rtcpu->rtcpu_sce.sce_cfg_base +
					TEGRA_APS_FRSC_SC_CTL_0);

		/* Enable JTAG/Coresight */
		writel(TEGRA_SCE_FN_MODEIN,
			cam_rtcpu->rtcpu_sce.sce_cfg_base +
					TEGRA_SCE_APS_FRSC_SC_MODEIN_0);

		/* Set FW load done bit */
		reg_val = readl(cam_rtcpu->rtcpu_sce.sce_pm_base +
				TEGRA_SCEPM_R5_CTRL_0);
		writel(TEGRA_SCE_FWLOADDONE | reg_val,
			cam_rtcpu->rtcpu_sce.sce_pm_base +
			TEGRA_SCEPM_R5_CTRL_0);

		dev_info(dev, "booting sce");
	} /* else the rtcpu is ape. */
}

static int tegra_cam_rtcpu_remove(struct platform_device *pdev);

static int tegra_cam_rtcpu_probe(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	struct device *dev = &pdev->dev;
	struct device_node *dev_node = dev->of_node;
	int ret;
	const struct of_device_id *match;

	dev_dbg(dev, "probing\n");

	cam_rtcpu = devm_kzalloc(dev, sizeof(*cam_rtcpu), GFP_KERNEL);
	if (!cam_rtcpu)
		return -ENOMEM;
	platform_set_drvdata(pdev, cam_rtcpu);

	match = of_match_device(tegra_cam_rtcpu_of_match, dev);
	if (match == NULL) {
		dev_err(dev, "Device match not found\n");
		return -ENODEV;
	}

	cam_rtcpu->rtcpu_pdata = match->data;

	if (cam_rtcpu->rtcpu_pdata->id == TEGRA_CAM_RTCPU_SCE) {
		/* RTCPU base addr / SCE EVP addr */
		cam_rtcpu->rtcpu_sce.sce_arsce_evp = of_iomap(dev_node, 0);
		if (!cam_rtcpu->rtcpu_sce.sce_arsce_evp) {
			dev_err(dev, "failed to map SCE EVP space.\n");
			return -EINVAL;
		}

		/* SCE PM addr */
		cam_rtcpu->rtcpu_sce.sce_pm_base = of_iomap(dev_node, 1);
		if (!cam_rtcpu->rtcpu_sce.sce_pm_base) {
			dev_err(dev, "failed to map SCE PM space.\n");
			return -EINVAL;
		}

		/* SCE CFG addr */
		cam_rtcpu->rtcpu_sce.sce_cfg_base = of_iomap(dev_node, 2);
		if (!cam_rtcpu->rtcpu_sce.sce_cfg_base) {
			dev_err(dev, "failed to map SCE CFG space.\n");
			return -EINVAL;
		}
	}

	ret = tegra_cam_rtcpu_get_clks_resets(dev);
	if (ret) {
		dev_err(dev, "failed to get clocks/resets: %d\n", ret);
		return ret;
	}

	cam_rtcpu->ivc = tegra_ivc_bus_create(dev,
						cam_rtcpu->rtcpu_pdata->sid);
	if (IS_ERR(cam_rtcpu->ivc))
		return PTR_ERR(cam_rtcpu->ivc);

	ret = tegra_cam_rtcpu_apply_clks(dev, clk_prepare_enable);
	if (ret) {
		dev_err(dev, "failed to turn on clocks: %d\n", ret);
		goto fail;
	}

	tegra_cam_rtcpu_apply_resets(dev, reset_control_deassert);

	tegra_cam_rtcpu_boot(dev);

	dev_dbg(dev, "probe successful\n");

	return 0;

fail:
	tegra_ivc_bus_destroy(cam_rtcpu->ivc);
	return ret;
}

static int tegra_cam_rtcpu_remove(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *cam_rtcpu = platform_get_drvdata(pdev);

	tegra_ivc_bus_destroy(cam_rtcpu->ivc);
	return 0;
}


static int tegra_cam_rtcpu_resume(struct device *dev)
{
	/*
	 * TODO:
	 * Call tegra_cam_rtcpu_clks_resets_on() and tegra_cam_rtcpu_boot()?
	 */

	return 0;
}

static int tegra_cam_rtcpu_suspend(struct device *dev)
{
	/*
	 * TODO:
	 *
	 * 1. Signal(through new control channel) RTCPU to finish off pending
	 *    tasks and assert standbyWFI.
	 * 2. Wait until acknowledgement from RTCPU for standbyWFI assert.
	 * 3. Once ack recieved check SCEPM_PWR_STATUS_0 register for
	 *    WFIPIPESTOPPED to confirm standbyWFI assert is done.
	 * 4. If OK proceed for SC7.
	 * 5. Disable clocks and assert resets.
	 */

	return 0;
}

static const struct dev_pm_ops tegra_cam_rtcpu_pm_ops = {
	.suspend = tegra_cam_rtcpu_suspend,
	.resume = tegra_cam_rtcpu_resume,
};

static struct platform_driver tegra_cam_rtcpu_driver = {
	.driver = {
		.name	= "tegra186-cam-rtcpu",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_cam_rtcpu_of_match),
#ifdef CONFIG_PM
		.pm = &tegra_cam_rtcpu_pm_ops,
#endif
	},
	.probe = tegra_cam_rtcpu_probe,
	.remove = tegra_cam_rtcpu_remove,
};
module_platform_driver(tegra_cam_rtcpu_driver);

MODULE_DESCRIPTION("CAMERA RTCPU driver");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL v2");
