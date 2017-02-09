/*
 * Copyright (c) 2014-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <dt-bindings/soc/tegra-io-pads.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/padctrl/padctrl.h>
#include <linux/slab.h>
#include <soc/tegra/pmc.h>

struct tegra_io_pad_info {
	const char *name;
	int id;
	bool dynamic_pad_voltage;
};

#define TEGRA_IO_PAD_INFO(_id, _name)		\
{						\
	.name = #_name,				\
	.id = TEGRA_IO_PAD_GROUP_##_id,		\
}

static struct tegra_io_pad_info tegra_io_pads_info[] = {
	TEGRA_IO_PAD_INFO(AUDIO, audio),
	TEGRA_IO_PAD_INFO(AUDIO_HV, audio-hv),
	TEGRA_IO_PAD_INFO(CAM, cam),
	TEGRA_IO_PAD_INFO(CONN, conn),
	TEGRA_IO_PAD_INFO(CSIA, csia),
	TEGRA_IO_PAD_INFO(CSIB, csib),
	TEGRA_IO_PAD_INFO(CSIC, csic),
	TEGRA_IO_PAD_INFO(CSID, csid),
	TEGRA_IO_PAD_INFO(CSIE, csie),
	TEGRA_IO_PAD_INFO(CSIF, csif),
	TEGRA_IO_PAD_INFO(DBG, dbg),
	TEGRA_IO_PAD_INFO(DEBUG_NONAO, debug-nonao),
	TEGRA_IO_PAD_INFO(DMIC, dmic),
	TEGRA_IO_PAD_INFO(DMIC_HV, dmic-hv),
	TEGRA_IO_PAD_INFO(DP, dp),
	TEGRA_IO_PAD_INFO(DSI, dsi),
	TEGRA_IO_PAD_INFO(DSIB, dsib),
	TEGRA_IO_PAD_INFO(DSIC, dsic),
	TEGRA_IO_PAD_INFO(DSID, dsid),
	TEGRA_IO_PAD_INFO(EMMC, emmc),
	TEGRA_IO_PAD_INFO(EMMC2, emmc2),
	TEGRA_IO_PAD_INFO(GPIO, gpio),
	TEGRA_IO_PAD_INFO(EDP, edp),
	TEGRA_IO_PAD_INFO(HDMI, hdmi),
	TEGRA_IO_PAD_INFO(HDMI_DP0, hdmi-dp0),
	TEGRA_IO_PAD_INFO(HDMI_DP1, hdmi-dp1),
	TEGRA_IO_PAD_INFO(HSIC, hsic),
	TEGRA_IO_PAD_INFO(LVDS, lvds),
	TEGRA_IO_PAD_INFO(MIPI_BIAS, mipi-bias),
	TEGRA_IO_PAD_INFO(PEX_BIAS, pex-bias),
	TEGRA_IO_PAD_INFO(PEX_CLK_BIAS, pex-clk-bias),
	TEGRA_IO_PAD_INFO(PEX_CLK1, pex-clk1),
	TEGRA_IO_PAD_INFO(PEX_CLK2, pex-clk2),
	TEGRA_IO_PAD_INFO(PEX_CLK3, pex-clk3),
	TEGRA_IO_PAD_INFO(PEX_CTRL, pex-ctrl),
	TEGRA_IO_PAD_INFO(SDMMC1, sdmmc1),
	TEGRA_IO_PAD_INFO(SDMMC1_HV, sdmmc1-hv),
	TEGRA_IO_PAD_INFO(SDMMC2_HV, sdmmc2-hv),
	TEGRA_IO_PAD_INFO(SDMMC3, sdmmc3),
	TEGRA_IO_PAD_INFO(SDMMC3_HV, sdmmc3-hv),
	TEGRA_IO_PAD_INFO(SDMMC4, sdmmc4),
	TEGRA_IO_PAD_INFO(SPI, spi),
	TEGRA_IO_PAD_INFO(SPI_HV, spi-hv),
	TEGRA_IO_PAD_INFO(UART, uart),
	TEGRA_IO_PAD_INFO(USB_BIAS, usb-bias),
	TEGRA_IO_PAD_INFO(USB0, usb0),
	TEGRA_IO_PAD_INFO(USB1, usb1),
	TEGRA_IO_PAD_INFO(USB2, usb2),
	TEGRA_IO_PAD_INFO(USB3, usb3),
	TEGRA_IO_PAD_INFO(BB, bb),
	TEGRA_IO_PAD_INFO(SYS, sys),
	TEGRA_IO_PAD_INFO(HV, hv),
	TEGRA_IO_PAD_INFO(UFS, ufs),
	TEGRA_IO_PAD_INFO(AO_HV, ao-hv),
	TEGRA_IO_PAD_INFO(DDR_DVI, ddr_dvi),
	TEGRA_IO_PAD_INFO(DDR_GMI, ddr-gmi),
	TEGRA_IO_PAD_INFO(DDR_SDMMC2, ddr-sdmmc2),
	TEGRA_IO_PAD_INFO(DDR_SPI, ddr-spi),
};

struct tegra_io_pads_padcontrol {
	struct device *dev;
	struct padctrl_dev *pad_dev;
};

static struct tegra_io_pad_info *tegra_get_io_pad_info(int id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tegra_io_pads_info); ++i) {
		if (tegra_io_pads_info[i].id == id)
			break;
	}

	if (i == ARRAY_SIZE(tegra_io_pads_info))
		return NULL;

	return &tegra_io_pads_info[i];
}

static int tegra_io_pad_set_voltage(struct padctrl_dev *pad_dev,
				    int id, u32 voltage)
{
	struct tegra_io_pads_padcontrol *padctrl = padctrl_get_drvdata(pad_dev);
	struct tegra_io_pad_info *pad;
	u32 curr_volt;
	int ret;

	switch (voltage) {
	case 1200000:
	case 1800000:
	case 3300000:
		break;
	default:
		dev_err(padctrl->dev, "Pad voltage %u is not valid\n", voltage);
		return -EINVAL;
	}

	pad = tegra_get_io_pad_info(id);
	if (!pad) {
		dev_err(padctrl->dev, "Pad ID %d not identified\n", id);
		return -EINVAL;
	}

	ret = tegra_pmc_io_pad_get_voltage(pad->name);
	if (ret < 0)
		return ret;

	curr_volt = ret;
	if (voltage == curr_volt)
		return 0;

	ret = tegra_pmc_io_pad_set_voltage(pad->name, voltage);
	if (!ret)
		udelay(100);

	return ret;
}

static int tegra_io_pad_get_voltage(struct padctrl_dev *pad_dev,
				    int id, u32 *voltage)
{
	struct tegra_io_pads_padcontrol *padctrl = padctrl_get_drvdata(pad_dev);
	struct tegra_io_pad_info *pad;
	int ret;

	pad = tegra_get_io_pad_info(id);
	if (!pad) {
		dev_err(padctrl->dev, "Pad ID %d not identified\n", id);
		return -EINVAL;
	}

	ret = tegra_pmc_io_pad_get_voltage(pad->name);
	if (ret < 0)
		return ret;

	*voltage = ret;

	return 0;
}

static int tegra_io_pad_set_power(struct padctrl_dev *pad_dev,
				  int id, u32 enable)
{
	struct tegra_io_pads_padcontrol *padctrl = padctrl_get_drvdata(pad_dev);
	struct tegra_io_pad_info *pad;

	pad = tegra_get_io_pad_info(id);
	if (!pad) {
		dev_err(padctrl->dev, "Pad ID %d not identified\n", id);
		return -EINVAL;
	}

	if (enable)
		return tegra_pmc_io_pad_low_power_disable(pad->name);

	return tegra_pmc_io_pad_low_power_enable(pad->name);
}

static int tegra_io_pad_power_enable(struct padctrl_dev *pad_dev, int id)
{
	return tegra_io_pad_set_power(pad_dev, id, 1);
}

static int tegra_io_pad_power_disable(struct padctrl_dev *pad_dev,
					      int id)
{
	return tegra_io_pad_set_power(pad_dev, id, 0);
}

static struct padctrl_ops tegra_io_pad_padctrl_ops = {
	.set_voltage = &tegra_io_pad_set_voltage,
	.get_voltage = &tegra_io_pad_get_voltage,
	.power_enable = &tegra_io_pad_power_enable,
	.power_disable = &tegra_io_pad_power_disable,
};

static struct padctrl_desc tegra_io_pads_padctrl_desc = {
	.name = "tegra-pmc-padctrl",
	.ops = &tegra_io_pad_padctrl_ops,
};

static int tegra_io_pad_parse_dt(struct tegra_io_pads_padcontrol *padctrl)

{
	struct padctrl_dev *pad_dev = padctrl->pad_dev;
	struct device *dev = padctrl->dev;
	struct device_node *pad_np, *child;
	struct tegra_io_pad_info *pad;
	u32 pval;
	int id;
	const char *pad_name, *name;
	bool dpd_en, dpd_dis, pad_en, pad_dis, io_dpd_en, io_dpd_dis;
	bool dyn_pad_volt;
	int n_child;
	u32 *volt_configs, *iodpd_configs;
	int i, vcount, dpd_count, pindex;
	int ret;

	pad_np = of_get_child_by_name(dev->of_node, "io-pad-defaults");
	if (!pad_np)
		return 0;

	/* Ignore the nodes if disabled */
	ret = of_device_is_available(pad_np);
	if (!ret)
		return 0;

	n_child = of_get_available_child_count(pad_np);
	if (!n_child)
		return 0;

	n_child *= 2;
	volt_configs = kzalloc(n_child * sizeof(*volt_configs), GFP_KERNEL);
	if (!volt_configs)
		return -ENOMEM;

	iodpd_configs = kzalloc(n_child * sizeof(*iodpd_configs), GFP_KERNEL);
	if (!iodpd_configs) {
		kfree(volt_configs);
		return -ENOMEM;
	}

	vcount = 0;
	dpd_count = 0;
	for_each_available_child_of_node(pad_np, child) {
		name = of_get_property(child, "nvidia,pad-name", NULL);
		if (!name)
			name = child->name;

		for (i = 0; i < ARRAY_SIZE(tegra_io_pads_info); ++i) {
			pad = &tegra_io_pads_info[i];
			if (strcmp(name, pad->name))
				continue;

			ret = of_property_read_u32(child,
					"nvidia,io-pad-init-voltage", &pval);
			if (!ret) {
				volt_configs[vcount++] = i;
				volt_configs[vcount++] = pval;
			}

			pad->dynamic_pad_voltage = of_property_read_bool(child,
					"nvidia,enable-dynamic-pad-voltage");

			dpd_en = of_property_read_bool(child,
						"nvidia,deep-power-down-enable");
			dpd_dis = of_property_read_bool(child,
						"nvidia,deep-power-down-disable");
			pad_en = of_property_read_bool(child,
						"nvidia,io-pad-power-enable");
			pad_dis = of_property_read_bool(child,
						"nvidia,io-pad-power-disable");

			io_dpd_en = dpd_en | pad_dis;
			io_dpd_dis = dpd_dis | pad_en;

			if ((dpd_en && pad_en)	|| (dpd_dis && pad_dis) ||
					(io_dpd_en & io_dpd_dis)) {
				pr_err("PMC: Conflict on io-pad %s config\n",
					name);
				continue;
			}
			if (io_dpd_en || io_dpd_dis) {
				iodpd_configs[dpd_count++] = i;
				iodpd_configs[dpd_count++] = !!io_dpd_dis;
			}
		}
	}

	for (i = 0; i < vcount; i += 2) {
		if (!volt_configs[i + 1])
			continue;

		pindex = volt_configs[i];
		id = tegra_io_pads_info[volt_configs[i]].id;
		pad_name = tegra_io_pads_info[volt_configs[i]].name;

		dyn_pad_volt = tegra_io_pads_info[pindex].dynamic_pad_voltage;
		tegra_io_pads_info[pindex].dynamic_pad_voltage = true;
		ret = tegra_io_pad_set_voltage(pad_dev, id,
						    volt_configs[i + 1]);
		if (ret < 0) {
			dev_warn(dev, "PMC: IO pad %s voltage config failed: %d\n",
				 pad_name, ret);
			WARN_ON(1);
		} else {
			dev_info(dev, "PMC: IO pad %s voltage is %d\n",
				 pad_name, volt_configs[i + 1]);
		}
		tegra_io_pads_info[pindex].dynamic_pad_voltage = dyn_pad_volt;
	}

	for (i = 0; i < dpd_count; i += 2) {
		id = tegra_io_pads_info[iodpd_configs[i]].id;
		pad_name = tegra_io_pads_info[iodpd_configs[i]].name;

		ret = tegra_io_pad_set_power(pad_dev, id,
						  iodpd_configs[i + 1]);
		if (ret < 0) {
			dev_warn(dev, "PMC: IO pad %s power config failed: %d\n",
				 pad_name, ret);
			WARN_ON(1);
		} else {
			dev_info(dev, "PMC: IO pad %s power is %s\n",
				 pad_name, (iodpd_configs[i + 1]) ?
				 "enable" : "disable");
		}
	}

	kfree(volt_configs);
	kfree(iodpd_configs);
	return 0;
}

int tegra_io_pads_padctrl_init(struct device *dev)
{
	struct tegra_io_pads_padcontrol *padctrl;
	struct padctrl_config config = { };
	int ret;

	padctrl = devm_kzalloc(dev, sizeof(*padctrl), GFP_KERNEL);
	if (!padctrl)
		return -ENOMEM;

	config.of_node = dev->of_node;
	padctrl->dev = dev;
	padctrl->pad_dev = padctrl_register(dev, &tegra_io_pads_padctrl_desc,
					    &config);
	if (IS_ERR(padctrl->pad_dev)) {
		ret = PTR_ERR(padctrl->pad_dev);
		dev_err(dev, "Failed to register padcontrol: %d\n", ret);
		return ret;
	}

	padctrl_set_drvdata(padctrl->pad_dev, padctrl);

	/* Clear all DPD */
	if (of_property_read_bool(dev->of_node, "clear-all-io-pads-dpd"))
		tegra_pmc_io_dpd_clear();

	tegra_io_pad_parse_dt(padctrl);

	dev_info(dev, "IO padctrl driver initialized\n");

	return 0;
}
