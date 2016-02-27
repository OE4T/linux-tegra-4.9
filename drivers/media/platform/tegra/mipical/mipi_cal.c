 /*
 * drivers/misc/mipi_cal.c
 *
 * Copyright (c) 2016, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/reset.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/clk/tegra.h>
#include <linux/pm.h>


#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
#include "mipi_cal_t21x.h"
#endif

#include "mipi_cal.h"
#define MIPI_DEBUG 0
#define DRV_NAME "tegra_mipi_cal"
#define MIPI_CAL_TIMEOUT_MSEC 1

struct tegra_mipi_bias {
	/* BIAS_PAD_CFG0 */
	u8 pad_pdvclamp;
	u8 e_vclamp_ref;
	/* BIAS_PAD_CFG1 */
	u8 pad_driv_up_ref;
	u8 pad_driv_dn_ref;
	/* BIAS_PAD_CFG2 */
	u8 pad_vclamp_level;
	u8 pad_vauxp_level;
};

struct tegra_mipi_prod_csi {
	struct tegra_mipi_bias bias_csi;
	u8 overide_x;
	u8 termos_x;
	u8 termos_x_clk;
};

struct tegra_mipi_prod_dsi {
	struct tegra_mipi_bias bias_dsi;
	u8 overide_x;
	u8 hspdos_x;
	u8 hspuos_x;
	u8 termos_x;
	u8 clk_overide_x;
	u8 clk_hspdos_x;
	u8 clk_hspuos_x;
};

struct tegra_mipi;
struct tegra_mipi_ops {
	int (*pad_enable)(struct tegra_mipi *mipi);
	int (*pad_disable)(struct tegra_mipi *mipi);
	int (*calibrate)(struct tegra_mipi *mipi, int lanes);
	int (*set_mode)(struct tegra_mipi *mipi, int mode);
	int (*parse_cfg)(struct platform_device *pdev, struct tegra_mipi *mipi);
};

struct tegra_mipi_data {
	struct tegra_mipi_prod_csi csi;
	struct tegra_mipi_prod_dsi dsi;
	struct tegra_mipi_ops ops;
};

struct tegra_mipi {
	struct device *dev;
	struct clk *mipi_cal_clk;
	struct clk *mipi_cal_fixed;
	struct reset_control *rst;
	struct regmap *regmap;
	struct mutex lock;
	struct tegra_mipi_prod_csi *prod_csi;
	struct tegra_mipi_prod_dsi *prod_dsi;
	struct tegra_mipi_ops *ops;
	atomic_t refcount;
};

static const struct regmap_config t210_mipi_cal_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.cache_type = REGCACHE_NONE,
	.fast_io = 1,
};

static struct tegra_mipi *get_mipi(void)
{
	struct device_node *np;
	struct platform_device *dev;
	struct tegra_mipi *mipi;

	np = of_find_node_by_name(NULL, "mipical");
	if (!np) {
		pr_err("%s: Can not find mipical node\n", __func__);
		return NULL;
	}
	dev = of_find_device_by_node(np);
	if (!dev) {
		pr_err("%s:Can not find device\n", __func__);
		return NULL;
	}
	mipi = platform_get_drvdata(dev);
	if (!mipi) {
		pr_err("%s:Can not find device\n", __func__);
		return NULL;
	}
	return mipi;
}

static void tegra_mipi_print(struct tegra_mipi *mipi) __maybe_unused;
static void tegra_mipi_print(struct tegra_mipi *mipi)
{
#if MIPI_DEBUG
	int val;
	unsigned long rate;
#define pr_reg(a)						\
	do {							\
		regmap_read(mipi->regmap, a, &val);		\
		dev_info(mipi->dev, "%-30s %#04x %#010x\n",		\
			#a, a, val);				\
	} while (0)

	rate = clk_get_rate(mipi->mipi_cal_fixed);
	dev_dbg(mipi->dev, "Fixed clk %luMHz\n", rate/1000000);

	pr_reg(MIPI_CAL_CTRL);
	pr_reg(CIL_MIPI_CAL_STATUS);
	pr_reg(CIL_MIPI_CAL_STATUS_2);
	pr_reg(CILA_MIPI_CAL_CONFIG);
	pr_reg(CILB_MIPI_CAL_CONFIG);
	pr_reg(CILC_MIPI_CAL_CONFIG);
	pr_reg(CILD_MIPI_CAL_CONFIG);
	pr_reg(CILE_MIPI_CAL_CONFIG);
	pr_reg(CILF_MIPI_CAL_CONFIG);
	pr_reg(DSIA_MIPI_CAL_CONFIG);
	pr_reg(DSIB_MIPI_CAL_CONFIG);
	pr_reg(DSIC_MIPI_CAL_CONFIG);
	pr_reg(DSID_MIPI_CAL_CONFIG);
	pr_reg(MIPI_BIAS_PAD_CFG0);
	pr_reg(MIPI_BIAS_PAD_CFG1);
	pr_reg(MIPI_BIAS_PAD_CFG2);
	pr_reg(DSIA_MIPI_CAL_CONFIG_2);
	pr_reg(DSIB_MIPI_CAL_CONFIG_2);
	pr_reg(DSIC_MIPI_CAL_CONFIG_2);
	pr_reg(DSID_MIPI_CAL_CONFIG_2);
#undef pr_reg

#endif
}
static int tegra_mipi_wait(struct tegra_mipi *mipi, int lanes)
{
	unsigned long timeout;
	int val;

	regmap_write(mipi->regmap, CIL_MIPI_CAL_STATUS, 0xffffffff);
	regmap_write(mipi->regmap, CIL_MIPI_CAL_STATUS_2, 0xffffffff);
	regmap_update_bits(mipi->regmap, MIPI_CAL_CTRL, STARTCAL, 0x1);

	timeout = jiffies + msecs_to_jiffies(MIPI_CAL_TIMEOUT_MSEC);
	while (time_before(jiffies, timeout)) {
		regmap_read(mipi->regmap, CIL_MIPI_CAL_STATUS, &val);
		if (((val & lanes) == lanes) && ((val & CAL_ACTIVE) == 0))
			return 0;
		usleep_range(10, 50);
	}
	/* Sometimes there is false timeout. Sleep past the timeout and did
	 * not check the status again.
	 * Later status register dump shows no timeout.
	 * Add another check here in case sleep past the timeout.
	 */
	regmap_read(mipi->regmap, CIL_MIPI_CAL_STATUS, &val);
	if (((val & lanes) == lanes) && ((val & CAL_ACTIVE) == 0))
		return 0;
	dev_err(mipi->dev, "Mipi cal timeout,val:%x, lanes:%x\n", val, lanes);
	tegra_mipi_print(mipi);
	return -ETIMEDOUT;

}

static int tegra_mipi_apply_bias_prod(struct regmap *reg,
				struct tegra_mipi_bias *bias)
{
	int ret;
	unsigned int val;

	val = (bias->pad_pdvclamp << PDVCLAMP_SHIFT) |
		(bias->e_vclamp_ref << E_VCLAMP_REF_SHIFT);
	ret = regmap_write(reg, MIPI_BIAS_PAD_CFG0, val);
	if (ret)
		return ret;
	val = (bias->pad_driv_up_ref << PAD_DRIV_UP_REF_SHIFT) |
		(bias->pad_driv_dn_ref << PAD_DRIV_DN_REF_SHIFT);
	ret = regmap_write(reg, MIPI_BIAS_PAD_CFG1, val);
	if (ret)
		return ret;
	val = (bias->pad_vclamp_level << PAD_VCLAMP_LEVEL_SHIFT) |
		(bias->pad_vauxp_level << PAD_VAUXP_LEVEL_SHIFT);
	ret = regmap_write(reg, MIPI_BIAS_PAD_CFG2, val);

	return ret;
}
static void tegra_mipi_apply_csi_prod(struct regmap *reg,
				struct tegra_mipi_prod_csi *prod_csi,
				int lanes)
{
	tegra_mipi_apply_bias_prod(reg, &prod_csi->bias_csi);
}

static void tegra_mipi_apply_dsi_prod(struct regmap *reg,
				struct tegra_mipi_prod_dsi *prod_dsi,
				int lanes)
{
	int val, clk_val;

	tegra_mipi_apply_bias_prod(reg, &prod_dsi->bias_dsi);
	val = (prod_dsi->hspuos_x << HSPUOSDSIA_SHIFT);
	clk_val = (prod_dsi->clk_hspuos_x << HSCLKPUOSDSIA_SHIFT);
	if (lanes & DSIA) {
		regmap_update_bits(reg, DSIA_MIPI_CAL_CONFIG, HSPUOSDSIA, val);
		regmap_update_bits(reg, DSIB_MIPI_CAL_CONFIG, HSPUOSDSIB, val);
		regmap_update_bits(reg, DSIA_MIPI_CAL_CONFIG_2,
				HSCLKPUOSDSIA, clk_val);
		regmap_update_bits(reg, DSIB_MIPI_CAL_CONFIG_2,
				HSCLKPUOSDSIB, clk_val);
	}
	if (lanes & DSIC) {
		regmap_update_bits(reg, DSIC_MIPI_CAL_CONFIG, HSPUOSDSIC, val);
		regmap_update_bits(reg, DSID_MIPI_CAL_CONFIG, HSPUOSDSID, val);
		regmap_update_bits(reg, DSIC_MIPI_CAL_CONFIG_2,
				HSCLKPUOSDSIC, clk_val);
		regmap_update_bits(reg, DSID_MIPI_CAL_CONFIG_2,
				HSCLKPUOSDSID, clk_val);
	}

}
static int _tegra_mipi_bias_pad_enable(struct tegra_mipi *mipi)
{
	if (atomic_read(&mipi->refcount) < 0) {
		WARN_ON(1);
		return -EINVAL;
	}
	if (atomic_inc_return(&mipi->refcount) == 1)
		return regmap_update_bits(mipi->regmap,
				MIPI_BIAS_PAD_CFG2, PDVREG, 0);
	return 0;
}
int tegra_mipi_bias_pad_enable(void)
{
	struct tegra_mipi *mipi;

	mipi = get_mipi();
	if (!mipi)
		return -EPROBE_DEFER;
	dev_dbg(mipi->dev, "%s", __func__);

	if (mipi->ops->pad_enable)
		return mipi->ops->pad_enable(mipi);
	else
		return 0;
}
EXPORT_SYMBOL(tegra_mipi_bias_pad_enable);

static int _tegra_mipi_bias_pad_disable(struct tegra_mipi *mipi)
{
	if (atomic_read(&mipi->refcount) < 1) {
		WARN_ON(1);
		return -EINVAL;
	}
	if (atomic_dec_return(&mipi->refcount) == 0)
		return regmap_update_bits(mipi->regmap,
				MIPI_BIAS_PAD_CFG2, PDVREG, PDVREG);
	return 0;
}

int tegra_mipi_bias_pad_disable(void)
{
	struct tegra_mipi *mipi;

	mipi = get_mipi();
	if (!mipi)
		return -ENODEV;
	dev_dbg(mipi->dev, "%s", __func__);

	if (mipi->ops->pad_disable)
		return mipi->ops->pad_disable(mipi);
	else
		return 0;
}
EXPORT_SYMBOL(tegra_mipi_bias_pad_disable);

static int tegra_mipi_clk_enable(struct tegra_mipi *mipi)
{
	int err;

	err = clk_prepare_enable(mipi->mipi_cal_fixed);
	if (err) {
		dev_err(mipi->dev, "Fail to enable uart_mipi_cal clk\n");
		return err;
	}
	mdelay(1);
	err = clk_prepare_enable(mipi->mipi_cal_clk);
	if (err) {
		dev_err(mipi->dev, "Fail to enable mipi_cal clk\n");
		goto err_mipi_cal_clk;
	}
	return 0;

err_mipi_cal_clk:
	clk_disable_unprepare(mipi->mipi_cal_fixed);
	return err;
}

static void tegra_mipi_clk_disable(struct tegra_mipi *mipi)
{
	clk_disable_unprepare(mipi->mipi_cal_clk);
	clk_disable_unprepare(mipi->mipi_cal_fixed);
}

static void select_lanes(struct tegra_mipi *mipi, int lanes)
{
	regmap_update_bits(mipi->regmap, CILA_MIPI_CAL_CONFIG, SELA,
			((lanes & CSIA) > 0 ? SELA : 0));
	regmap_update_bits(mipi->regmap, CILB_MIPI_CAL_CONFIG, SELB,
			((lanes & CSIB) > 0 ? SELB : 0));
	regmap_update_bits(mipi->regmap, CILC_MIPI_CAL_CONFIG, SELC,
			((lanes & CSIC) > 0 ? SELC : 0));
	regmap_update_bits(mipi->regmap, CILD_MIPI_CAL_CONFIG, SELD,
			((lanes & CSID) > 0 ? SELD : 0));
	regmap_update_bits(mipi->regmap, CILE_MIPI_CAL_CONFIG, SELE,
			((lanes & CSIE) > 0 ? SELE : 0));
	regmap_update_bits(mipi->regmap, CILF_MIPI_CAL_CONFIG, SELF,
			((lanes & CSIF) > 0 ? SELF : 0));

	regmap_update_bits(mipi->regmap, DSIA_MIPI_CAL_CONFIG, SELDSIA,
			((lanes & DSIA) > 0 ? SELDSIA : 0));
	regmap_update_bits(mipi->regmap, DSIB_MIPI_CAL_CONFIG, SELDSIB,
			((lanes & DSIB) > 0 ? SELDSIB : 0));
	regmap_update_bits(mipi->regmap, DSIC_MIPI_CAL_CONFIG, SELDSIC,
			((lanes & DSIC) > 0 ? SELDSIC : 0));
	regmap_update_bits(mipi->regmap, DSID_MIPI_CAL_CONFIG, SELDSID,
			((lanes & DSID) > 0 ? SELDSID : 0));
	regmap_update_bits(mipi->regmap, DSIA_MIPI_CAL_CONFIG_2, CLKSELDSIA,
			((lanes & DSIA) > 0 ? CLKSELDSIA : 0));
	regmap_update_bits(mipi->regmap, DSIB_MIPI_CAL_CONFIG_2, CLKSELDSIB,
			((lanes & DSIB) > 0 ? CLKSELDSIB : 0));
	regmap_update_bits(mipi->regmap, DSIC_MIPI_CAL_CONFIG_2, CLKSELDSIC,
			((lanes & DSIC) > 0 ? CLKSELDSIC : 0));
	regmap_update_bits(mipi->regmap, DSID_MIPI_CAL_CONFIG_2, CLKSELDSID,
			((lanes & DSID) > 0 ? CLKSELDSID : 0));

}

static void clear_all(struct tegra_mipi *mipi)
{
	regmap_write(mipi->regmap, CILA_MIPI_CAL_CONFIG, 0);
	regmap_write(mipi->regmap, CILB_MIPI_CAL_CONFIG, 0);
	regmap_write(mipi->regmap, CILC_MIPI_CAL_CONFIG, 0);
	regmap_write(mipi->regmap, CILD_MIPI_CAL_CONFIG, 0);
	regmap_write(mipi->regmap, CILE_MIPI_CAL_CONFIG, 0);
	regmap_write(mipi->regmap, CILF_MIPI_CAL_CONFIG, 0);

	regmap_write(mipi->regmap, DSIA_MIPI_CAL_CONFIG, 0);
	regmap_write(mipi->regmap, DSIB_MIPI_CAL_CONFIG, 0);
	regmap_write(mipi->regmap, DSIC_MIPI_CAL_CONFIG, 0);
	regmap_write(mipi->regmap, DSID_MIPI_CAL_CONFIG, 0);
	regmap_write(mipi->regmap, DSIA_MIPI_CAL_CONFIG_2, 0);
	regmap_write(mipi->regmap, DSIB_MIPI_CAL_CONFIG_2, 0);
	regmap_write(mipi->regmap, DSIC_MIPI_CAL_CONFIG_2, 0);
	regmap_write(mipi->regmap, DSID_MIPI_CAL_CONFIG_2, 0);

	regmap_write(mipi->regmap, MIPI_BIAS_PAD_CFG0, 0);
	regmap_write(mipi->regmap, MIPI_BIAS_PAD_CFG1, 0);
	regmap_write(mipi->regmap, MIPI_BIAS_PAD_CFG2, 0);
}

#ifdef CONFIG_DEBUG_FS
static u32 mipical_status;
static u32 timeout_ct;
static u32 counts;

static int dbgfs_mipi_init(struct tegra_mipi *mipi)
{
	struct dentry *dir;
	struct dentry *val;

	dir = debugfs_create_dir("tegra_mipical", NULL);
	if (!dir)
		return -ENOMEM;

	val = debugfs_create_x32("LAST_STATUS", S_IRUGO, dir, &mipical_status);
	if (!val)
		return -ENOMEM;
	val = debugfs_create_u32("COUNT", S_IRUGO | S_IWUGO, dir, &counts);
	if (!val)
		return -ENOMEM;
	val = debugfs_create_u32("TIMEOUTS", S_IRUGO | S_IWUGO, dir,
				&timeout_ct);
	if (!val)
		return -ENOMEM;
	return 0;
}
#endif
static int _tegra_mipi_calibration(struct tegra_mipi *mipi, int lanes)
{
	int err;

	mutex_lock(&mipi->lock);
	err = tegra_mipi_clk_enable(mipi);
	if (err)
		goto err_unlock;

	/* clean up lanes */
	clear_all(mipi);

	/* Apply MIPI_CAL PROD_Set */
	if (lanes & (CSIA|CSIB|CSIC|CSID|CSIE|CSIF))
		tegra_mipi_apply_csi_prod(mipi->regmap, mipi->prod_csi,
						lanes);
	else
		tegra_mipi_apply_dsi_prod(mipi->regmap, mipi->prod_dsi,
						lanes);

	/*Select lanes */
	select_lanes(mipi, lanes);
	/* Start calibration */
	err = tegra_mipi_wait(mipi, lanes);

#ifdef CONFIG_DEBUG_FS
	regmap_read(mipi->regmap, CIL_MIPI_CAL_STATUS, &mipical_status);
	counts++;
	if (err)
		timeout_ct++;
#endif
	tegra_mipi_clk_disable(mipi);
err_unlock:
	mutex_unlock(&mipi->lock);
	return err;
}

int tegra_mipi_calibration(int lanes)
{
	struct tegra_mipi *mipi;

	mipi = get_mipi();
	if (!mipi)
		return -ENODEV;
	dev_dbg(mipi->dev, "%s", __func__);
	if (mipi->ops->calibrate)
		return mipi->ops->calibrate(mipi, lanes);
	else
		return 0;

}
EXPORT_SYMBOL(tegra_mipi_calibration);

int tegra_mipi_select_mode(int mode)
{
	struct tegra_mipi *mipi;

	mipi = get_mipi();
	if (!mipi)
		return -ENODEV;
	dev_dbg(mipi->dev, "%s", __func__);
	if (mipi->ops->set_mode)
		return mipi->ops->set_mode(mipi, mode);
	else
		return 0;
}
EXPORT_SYMBOL(tegra_mipi_select_mode);

static void parse_bias_prod(struct device_node *np,
			    struct tegra_mipi_bias *bias)
{
	int ret;
	unsigned int v;

	ret = of_property_read_u32(np, "bias-pad-cfg0", &v);
	if (!ret) {
		bias->pad_pdvclamp = (v & PDVCLAMP) >> PDVCLAMP_SHIFT;
		bias->e_vclamp_ref = (v & E_VCLAMP_REF) >> E_VCLAMP_REF_SHIFT;
	}

	ret = of_property_read_u32(np, "bias-pad-cfg1", &v);
	if (!ret) {
		bias->pad_driv_up_ref =
				(v & PAD_DRIV_UP_REF) >> PAD_DRIV_UP_REF_SHIFT;
		bias->pad_driv_dn_ref =
				(v & PAD_DRIV_DN_REF) >> PAD_DRIV_DN_REF_SHIFT;
	}

	ret = of_property_read_u32(np, "bias-pad-cfg2", &v);
	if (!ret) {
		bias->pad_vclamp_level =
			(v & PAD_VCLAMP_LEVEL) >> PAD_VCLAMP_LEVEL_SHIFT;
		bias->pad_vauxp_level =
			(v & PAD_VAUXP_LEVEL) >> PAD_VAUXP_LEVEL_SHIFT;
	}
}

static void parse_dsi_prod(struct device_node *np,
			   struct tegra_mipi_prod_dsi *dsi)
{
	int ret;
	unsigned int v;

	ret = of_property_read_u32(np, "dsix-cfg", &v);
	if (!ret) {
		dsi->overide_x = (v & OVERIDEDSIA) >> OVERIDEDSIA_SHIFT;
		dsi->hspdos_x = (v & HSPDOSDSIA) >> HSPDOSDSIA_SHIFT;
		dsi->hspuos_x = (v & HSPUOSDSIA) >> HSPUOSDSIA_SHIFT;
		dsi->termos_x = (v & TERMOSDSIA) >> TERMOSDSIA_SHIFT;
	}
	ret = of_property_read_u32(np, "dsix-cfg2", &v);
	if (!ret) {
		dsi->clk_overide_x =
				(v & CLKOVERIDEDSIA) >> CLKOVERIDEDSIA_SHIFT;
		dsi->clk_hspdos_x = (v & HSCLKPDOSDSIA) >> HSCLKPDOSDSIA_SHIFT;
		dsi->clk_hspuos_x = (v & HSCLKPUOSDSIA) >> HSCLKPUOSDSIA_SHIFT;
	}
	parse_bias_prod(np, &dsi->bias_dsi);
}
static void parse_csi_prod(struct device_node *np,
			   struct tegra_mipi_prod_csi *csi)
{
	int ret;
	unsigned int v;

	ret = of_property_read_u32(np, "cilx-cfg", &v);
	if (!ret) {
		csi->overide_x = (v & OVERIDEA) >> OVERIDEA_SHIFT;
		csi->termos_x = (v & TERMOSA) >> TERMOSA_SHIFT;
	}
	parse_bias_prod(np, &csi->bias_csi);

}
static int tegra_mipi_parse_config_t210(struct platform_device *pdev,
					struct tegra_mipi *mipi)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *next;

	if (!np)
		return -ENODEV;
	next = of_get_child_by_name(np, "dsi");
	if (next)
		parse_dsi_prod(next, mipi->prod_dsi);
	next = of_get_child_by_name(np, "csi");
	if (next)
		parse_csi_prod(next, mipi->prod_csi);
	return 0;
}
const struct tegra_mipi_data t210_mipi_data = {
	.dsi = {
		.hspuos_x = 0x2,
		.clk_hspuos_x = 0x2,
		.bias_dsi = {
			.pad_driv_up_ref = 0x3,
			.pad_vauxp_level = 0x1,
			.pad_vclamp_level = 0x1,
		},
	},
	.ops = {
		.pad_enable = _tegra_mipi_bias_pad_enable,
		.pad_disable = _tegra_mipi_bias_pad_disable,
		.calibrate = _tegra_mipi_calibration,
		.parse_cfg = tegra_mipi_parse_config_t210,
	},
};

static const struct of_device_id tegra_mipi_of_match[] = {
	{ .compatible = "nvidia,tegra210-mipical", .data = &t210_mipi_data},
};


static int tegra_mipi_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct tegra_mipi *mipi;
	struct tegra_mipi_data *data_ptr;
	struct resource *mem, *memregion;
	void __iomem *regs;
	int err;

	err = 0;
	match = of_match_device(tegra_mipi_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "No device match found\n");
		return -ENODEV;
	}

	mipi = devm_kzalloc(&pdev->dev, sizeof(*mipi), GFP_KERNEL);
	if (!mipi)
		return -ENOMEM;
	data_ptr = devm_kzalloc(&pdev->dev, sizeof(*data_ptr), GFP_KERNEL);
	memcpy(data_ptr, match->data, sizeof(*data_ptr));

	mipi->prod_csi = &data_ptr->csi;
	mipi->prod_dsi = &data_ptr->dsi;
	mipi->ops = &data_ptr->ops;
	mipi->dev = &pdev->dev;

	if (mipi->ops->parse_cfg)
		err = mipi->ops->parse_cfg(pdev, mipi);
	if (err)
		return err;

	dev_dbg(&pdev->dev, "Mipi cal start probing...\n");
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		return -EINVAL;
	}
	memregion = devm_request_mem_region(&pdev->dev, mem->start,
			resource_size(mem), pdev->name);
	if (!memregion) {
		dev_err(&pdev->dev, "Cannot request mem region\n");
		return -EBUSY;
	}
	regs = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	mipi->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
			&t210_mipi_cal_regmap_config);
	if (IS_ERR(mipi->regmap)) {
		dev_err(&pdev->dev, "Fai to initialize regmap\n");
		return PTR_ERR(mipi->regmap);
	}

	regcache_cache_only(mipi->regmap, false);

	mipi->mipi_cal_clk = devm_clk_get(&pdev->dev, "mipi_cal");
	if (IS_ERR(mipi->mipi_cal_clk))
		return PTR_ERR(mipi->mipi_cal_clk);

	mipi->mipi_cal_fixed = devm_clk_get(&pdev->dev, "uart_mipi_cal");
	if (IS_ERR(mipi->mipi_cal_fixed))
		return PTR_ERR(mipi->mipi_cal_fixed);

	mutex_init(&mipi->lock);
	atomic_set(&mipi->refcount, 0);

	platform_set_drvdata(pdev, mipi);
#ifdef CONFIG_DEBUG_FS
	err = dbgfs_mipi_init(mipi);
	if (err)
		dev_err(&pdev->dev, "Fail to create debugfs\n");
#endif
	dev_dbg(&pdev->dev, "Mipi cal done probing...\n");
	return err;
}

struct platform_driver tegra_mipi_cal_platform_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra_mipi_of_match,
	},
	.probe = tegra_mipi_probe,
};

static int __init tegra_mipi_module_init(void)
{
	return platform_driver_register(&tegra_mipi_cal_platform_driver);
}

static void __exit tegra_mipi_module_exit(void)
{
	platform_driver_unregister(&tegra_mipi_cal_platform_driver);
}
subsys_initcall(tegra_mipi_module_init);
module_exit(tegra_mipi_module_exit);

MODULE_AUTHOR("Wenjia Zhou <wenjiaz@nvidia.com>");
MODULE_DESCRIPTION("Common MIPI calibration driver for CSI and DSI");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
