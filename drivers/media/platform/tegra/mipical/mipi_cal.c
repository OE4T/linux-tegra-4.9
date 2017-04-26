/*
 * mipi_cal.c
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include <soc/tegra/fuse.h>
#include <soc/tegra/tegra_powergate.h>
#include <linux/tegra_prod.h>
#include <uapi/misc/tegra_mipi_ioctl.h>

#include "registers.h"
#include "mipi_cal.h"
#include "vmipi/vmipi.h"

#define CREATE_TRACE_POINTS
#include <trace/events/mipical.h>

#define DRV_NAME "tegra_mipi_cal"
#define MIPI_CAL_TIMEOUT_MSEC 500

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
	u8 clk_hstermos_x;
};

struct tegra_mipi;
struct tegra_mipi {
	struct device *dev;
	struct clk *mipi_cal_clk;
	struct clk *mipi_cal_fixed;
	struct reset_control *rst;
	struct regmap *regmap;
	struct mutex lock;
	/* Legacy way of storing mipical reg config */
	struct tegra_mipi_prod_csi *prod_csi;
	struct tegra_mipi_prod_dsi *prod_dsi;
	/* If use tegra_prod framework */
	struct tegra_prod *prod_list;
	const struct tegra_mipi_soc *soc;
	void __iomem *io;
	atomic_t refcount;
	struct miscdevice misc_dev;
};

static struct tegra_mipi *mipi;
static const struct regmap_config t210_mipi_cal_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.cache_type = REGCACHE_NONE,
	.fast_io = 1,
};

#define ADDR(x) (x + mipi->soc->addr_offset)
#define dump_register(nm)	\
{				\
	.name = #nm,		\
	.offset = nm,		\
}
struct tegra_mipi_soc {
	int powergate_id;
	unsigned int total_dsilanes;
	unsigned int total_cillanes;
	char addr_offset;
	char ppsb_war;
	int (*pad_enable)(struct tegra_mipi *mipi);
	int (*pad_disable)(struct tegra_mipi *mipi);
	int (*calibrate)(struct tegra_mipi *mipi, int lanes);
	int (*parse_cfg)(struct platform_device *pdev, struct tegra_mipi *mipi);
	u8 virtual_dev;
};

static int tegra_mipical_prodset_helper(struct tegra_mipi *mipi,
					char *prod_node);

static int mipical_write(struct regmap *map,
		unsigned int reg, unsigned int val)
{
	int ret, rb_val;

	ret = regmap_write(map, reg, val);
	if (mipi->soc->ppsb_war)
	/* Read back register to make sure that register writes completed */
		regmap_read(map, reg, &rb_val);

	return ret;
}

static int mipical_update_bits(struct regmap *map, unsigned int reg,
		unsigned int mask, unsigned int val)
{
	int ret, rb_val;

	ret = regmap_update_bits(map, reg, mask, val);
	if (mipi->soc->ppsb_war)
	/* Read back register to make sure that register writes completed */
		regmap_read(map, reg, &rb_val);

	return ret;
}
static int tegra_mipi_clk_enable(struct tegra_mipi *mipi)
{
	int err;

	err = tegra_unpowergate_partition(mipi->soc->powergate_id);
	if (err) {
		dev_err(mipi->dev, "Fail to unpowergate SOR\n");
		return err;
	}

	err = clk_prepare_enable(mipi->mipi_cal_fixed);
	if (err) {
		dev_err(mipi->dev, "Fail to enable uart_mipi_cal clk\n");
		goto err_fixed_clk;
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
err_fixed_clk:
	tegra_powergate_partition(mipi->soc->powergate_id);

	return err;
}

static void tegra_mipi_clk_disable(struct tegra_mipi *mipi)
{
	clk_disable_unprepare(mipi->mipi_cal_clk);
	clk_disable_unprepare(mipi->mipi_cal_fixed);
	tegra_powergate_partition(mipi->soc->powergate_id);
}

static void tegra_mipi_print(struct tegra_mipi *mipi) __maybe_unused;
static void tegra_mipi_print(struct tegra_mipi *mipi)
{
	int val;
	unsigned long rate;
#define pr_reg(a)						\
	do {							\
		regmap_read(mipi->regmap, ADDR(a), &val);		\
		dev_info(mipi->dev, "%-30s %#04x %#010x\n",		\
			#a, a + mipi->soc->addr_offset, val);		\
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
}
static int tegra_mipi_wait(struct tegra_mipi *mipi, int lanes)
{
	unsigned long timeout;
	int val;

	mipical_write(mipi->regmap, ADDR(CIL_MIPI_CAL_STATUS), 0xffffffff);
	mipical_write(mipi->regmap, ADDR(CIL_MIPI_CAL_STATUS_2), 0xffffffff);
	mipical_update_bits(mipi->regmap, ADDR(MIPI_CAL_CTRL), STARTCAL, 0x1);

	timeout = jiffies + msecs_to_jiffies(MIPI_CAL_TIMEOUT_MSEC);
	while (time_before(jiffies, timeout)) {
		regmap_read(mipi->regmap, ADDR(CIL_MIPI_CAL_STATUS), &val);
		if (((val & lanes) == lanes) && ((val & CAL_ACTIVE) == 0)) {
			trace_mipical_result("CIL_MIPI_CAL_STATUS", val);
			return 0;
		}
		usleep_range(10, 50);
	}
	/* Sometimes there is false timeout. Sleep past the timeout and did
	 * not check the status again.
	 * Later status register dump shows no timeout.
	 * Add another check here in case sleep past the timeout.
	 */
	regmap_read(mipi->regmap, ADDR(CIL_MIPI_CAL_STATUS), &val);
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
	ret = mipical_write(reg, ADDR(MIPI_BIAS_PAD_CFG0), val);
	if (ret)
		return ret;
	val = (bias->pad_driv_up_ref << PAD_DRIV_UP_REF_SHIFT) |
		(bias->pad_driv_dn_ref << PAD_DRIV_DN_REF_SHIFT);
	ret = mipical_write(reg, ADDR(MIPI_BIAS_PAD_CFG1), val);
	if (ret)
		return ret;
	val = (bias->pad_vclamp_level << PAD_VCLAMP_LEVEL_SHIFT) |
		(bias->pad_vauxp_level << PAD_VAUXP_LEVEL_SHIFT);
	ret = mipical_write(reg, ADDR(MIPI_BIAS_PAD_CFG2), val);

	return ret;
}
static void tegra_mipi_apply_csi_prod(struct regmap *reg,
				struct tegra_mipi_prod_csi *prod_csi,
				int lanes)
{
	int val;

	tegra_mipi_apply_bias_prod(reg, &prod_csi->bias_csi);
	val = (prod_csi->termos_x << TERMOSA_SHIFT) |
		(prod_csi->termos_x_clk << TERMOSA_CLK_SHIFT);

	if (lanes & CSIA)
		mipical_write(reg, ADDR(CILA_MIPI_CAL_CONFIG), val);
	if (lanes & CSIB)
		mipical_write(reg, ADDR(CILB_MIPI_CAL_CONFIG), val);
	if (lanes & CSIC)
		mipical_write(reg, ADDR(CILC_MIPI_CAL_CONFIG), val);
	if (lanes & CSID)
		mipical_write(reg, ADDR(CILD_MIPI_CAL_CONFIG), val);
	if (lanes & CSIE)
		mipical_write(reg, ADDR(CILE_MIPI_CAL_CONFIG), val);
	if (lanes & CSIF)
		mipical_write(reg, ADDR(CILF_MIPI_CAL_CONFIG), val);
}

static void tegra_mipi_apply_dsi_prod(struct regmap *reg,
				struct tegra_mipi_prod_dsi *prod_dsi,
				int lanes)
{
	int val, clk_val;

	tegra_mipi_apply_bias_prod(reg, &prod_dsi->bias_dsi);
	val = (prod_dsi->hspuos_x << HSPUOSDSIA_SHIFT) |
		(prod_dsi->termos_x << TERMOSDSIA_SHIFT);
	clk_val = (prod_dsi->clk_hspuos_x << HSCLKPUOSDSIA_SHIFT) |
		(prod_dsi->clk_hstermos_x << HSCLKTERMOSDSIA_SHIFT);
	if (lanes & DSIA) {
		mipical_write(reg, ADDR(DSIA_MIPI_CAL_CONFIG), val);
		mipical_write(reg, ADDR(DSIA_MIPI_CAL_CONFIG_2), clk_val);
	}
	if (lanes & DSIB) {
		mipical_write(reg, ADDR(DSIB_MIPI_CAL_CONFIG), val);
		mipical_write(reg, ADDR(DSIB_MIPI_CAL_CONFIG_2), clk_val);
	}
	if (lanes & DSIC) {
		mipical_write(reg, ADDR(DSIC_MIPI_CAL_CONFIG), val);
		mipical_write(reg, ADDR(DSIC_MIPI_CAL_CONFIG_2), clk_val);
	}
	if (lanes & DSID) {
		mipical_write(reg, ADDR(DSID_MIPI_CAL_CONFIG), val);
		mipical_write(reg, ADDR(DSID_MIPI_CAL_CONFIG_2), clk_val);
	}

}

static int _t18x_tegra_mipi_bias_pad_enable(struct tegra_mipi *mipi)
{
	if (atomic_read(&mipi->refcount) < 0) {
		WARN_ON(1);
		return -EINVAL;
	}
	if (atomic_inc_return(&mipi->refcount) == 1) {
		tegra_mipi_clk_enable(mipi);
		mipical_update_bits(mipi->regmap, ADDR(MIPI_BIAS_PAD_CFG0),
				PDVCLAMP, 0 << PDVCLAMP_SHIFT);
		mipical_update_bits(mipi->regmap, ADDR(MIPI_BIAS_PAD_CFG2),
				PDVREG, 0 << PDVREG_SHIFT);
	}
	return 0;
}

static int _t18x_tegra_mipi_bias_pad_disable(struct tegra_mipi *mipi)
{
	if (atomic_read(&mipi->refcount) < 1) {
		WARN_ON(1);
		return -EINVAL;
	}
	if (atomic_dec_return(&mipi->refcount) == 0) {
		mipical_update_bits(mipi->regmap, ADDR(MIPI_BIAS_PAD_CFG0),
				PDVCLAMP, 1 << PDVCLAMP_SHIFT);
		mipical_update_bits(mipi->regmap, ADDR(MIPI_BIAS_PAD_CFG2),
				PDVREG, 1 << PDVREG_SHIFT);
		tegra_mipi_clk_disable(mipi);
	}
	return 0;
}

static int _t21x_tegra_mipi_bias_pad_enable(struct tegra_mipi *mipi)
{
	if (atomic_read(&mipi->refcount) < 0) {
		WARN_ON(1);
		return -EINVAL;
	}
	if (atomic_inc_return(&mipi->refcount) == 1) {
		tegra_mipi_clk_enable(mipi);
		return mipical_update_bits(mipi->regmap,
				ADDR(MIPI_BIAS_PAD_CFG2), PDVREG, 0);
	}
	return 0;
}

int tegra_mipi_bias_pad_enable(void)
{
	int ret = 0;

	if (!mipi)
		return -EPROBE_DEFER;
	dev_dbg(mipi->dev, "%s", __func__);

	if (mipi->soc->pad_enable)
		ret = mipi->soc->pad_enable(mipi);
	else
		ret = -EINVAL;

	trace_pad_enable("ref", atomic_read(&mipi->refcount));
	return ret;
}
EXPORT_SYMBOL(tegra_mipi_bias_pad_enable);

static int _t21x_tegra_mipi_bias_pad_disable(struct tegra_mipi *mipi)
{
	if (atomic_read(&mipi->refcount) < 1) {
		WARN_ON(1);
		return -EINVAL;
	}
	if (atomic_dec_return(&mipi->refcount) == 0) {
		mipical_update_bits(mipi->regmap,
				ADDR(MIPI_BIAS_PAD_CFG2), PDVREG, PDVREG);
		tegra_mipi_clk_disable(mipi);
	}
	return 0;
}

int tegra_mipi_bias_pad_disable(void)
{
	int ret = 0;

	if (!mipi)
		return -ENODEV;
	dev_dbg(mipi->dev, "%s", __func__);

	if (mipi->soc->pad_disable)
		ret = mipi->soc->pad_disable(mipi);
	else
		ret = -EINVAL;

	trace_pad_disable("ref", atomic_read(&mipi->refcount));
	return ret;
}
EXPORT_SYMBOL(tegra_mipi_bias_pad_disable);

static void select_lanes(struct tegra_mipi *mipi, int lanes)
{
	mipical_update_bits(mipi->regmap, ADDR(CILA_MIPI_CAL_CONFIG), SELA,
			((lanes & CSIA) != 0 ? SELA : 0));
	mipical_update_bits(mipi->regmap, ADDR(CILB_MIPI_CAL_CONFIG), SELA,
			((lanes & CSIB) != 0 ? SELA : 0));
	mipical_update_bits(mipi->regmap, ADDR(CILC_MIPI_CAL_CONFIG), SELA,
			((lanes & CSIC) != 0 ? SELA : 0));
	mipical_update_bits(mipi->regmap, ADDR(CILD_MIPI_CAL_CONFIG), SELA,
			((lanes & CSID) != 0 ? SELA : 0));
	mipical_update_bits(mipi->regmap, ADDR(CILE_MIPI_CAL_CONFIG), SELA,
			((lanes & CSIE) != 0 ? SELA : 0));
	mipical_update_bits(mipi->regmap, ADDR(CILF_MIPI_CAL_CONFIG), SELA,
			((lanes & CSIF) != 0 ? SELA : 0));

	mipical_update_bits(mipi->regmap, ADDR(DSIA_MIPI_CAL_CONFIG), SELDSIA,
			((lanes & DSIA) != 0 ? SELDSIA : 0));
	mipical_update_bits(mipi->regmap, ADDR(DSIB_MIPI_CAL_CONFIG), SELDSIA,
			((lanes & DSIB) != 0 ? SELDSIA : 0));
	mipical_update_bits(mipi->regmap, ADDR(DSIC_MIPI_CAL_CONFIG), SELDSIA,
			((lanes & DSIC) != 0 ? SELDSIA : 0));
	mipical_update_bits(mipi->regmap, ADDR(DSID_MIPI_CAL_CONFIG), SELDSIA,
			((lanes & DSID) != 0 ? SELDSIA : 0));
	mipical_update_bits(mipi->regmap, ADDR(DSIA_MIPI_CAL_CONFIG_2),
			CLKSELDSIA, ((lanes & DSIA) != 0 ? CLKSELDSIA : 0));
	mipical_update_bits(mipi->regmap, ADDR(DSIB_MIPI_CAL_CONFIG_2),
			CLKSELDSIA, ((lanes & DSIB) != 0 ? CLKSELDSIA : 0));
	mipical_update_bits(mipi->regmap, ADDR(DSIC_MIPI_CAL_CONFIG_2),
			CLKSELDSIA, ((lanes & DSIC) != 0 ? CLKSELDSIA : 0));
	mipical_update_bits(mipi->regmap, ADDR(DSID_MIPI_CAL_CONFIG_2),
			CLKSELDSIA, ((lanes & DSID) != 0 ? CLKSELDSIA : 0));
}

static void clear_all(struct tegra_mipi *mipi)
{
	mipical_write(mipi->regmap, ADDR(CILA_MIPI_CAL_CONFIG), 0);
	mipical_write(mipi->regmap, ADDR(CILB_MIPI_CAL_CONFIG), 0);
	mipical_write(mipi->regmap, ADDR(CILC_MIPI_CAL_CONFIG), 0);
	mipical_write(mipi->regmap, ADDR(CILD_MIPI_CAL_CONFIG), 0);
	mipical_write(mipi->regmap, ADDR(CILE_MIPI_CAL_CONFIG), 0);
	mipical_write(mipi->regmap, ADDR(CILF_MIPI_CAL_CONFIG), 0);

	mipical_write(mipi->regmap, ADDR(DSIA_MIPI_CAL_CONFIG), 0);
	mipical_write(mipi->regmap, ADDR(DSIB_MIPI_CAL_CONFIG), 0);
	mipical_write(mipi->regmap, ADDR(DSIC_MIPI_CAL_CONFIG), 0);
	mipical_write(mipi->regmap, ADDR(DSID_MIPI_CAL_CONFIG), 0);
	mipical_write(mipi->regmap, ADDR(DSIA_MIPI_CAL_CONFIG_2), 0);
	mipical_write(mipi->regmap, ADDR(DSIB_MIPI_CAL_CONFIG_2), 0);
	mipical_write(mipi->regmap, ADDR(DSIC_MIPI_CAL_CONFIG_2), 0);
	mipical_write(mipi->regmap, ADDR(DSID_MIPI_CAL_CONFIG_2), 0);
}

#ifdef CONFIG_DEBUG_FS
static struct debugfs_reg32 mipical_regs[] = {
	dump_register(MIPI_CAL_CTRL),
	dump_register(MIPI_CAL_AUTOCAL_CTRL0),
	dump_register(CIL_MIPI_CAL_STATUS),
	dump_register(CIL_MIPI_CAL_STATUS_2),
	dump_register(CILA_MIPI_CAL_CONFIG),
	dump_register(CILB_MIPI_CAL_CONFIG),
	dump_register(CILC_MIPI_CAL_CONFIG),
	dump_register(CILD_MIPI_CAL_CONFIG),
	dump_register(CILE_MIPI_CAL_CONFIG),
	dump_register(CILF_MIPI_CAL_CONFIG),
	dump_register(DSIA_MIPI_CAL_CONFIG),
	dump_register(DSIB_MIPI_CAL_CONFIG),
	dump_register(DSIC_MIPI_CAL_CONFIG),
	dump_register(DSID_MIPI_CAL_CONFIG),
	dump_register(MIPI_BIAS_PAD_CFG0),
	dump_register(MIPI_BIAS_PAD_CFG1),
	dump_register(MIPI_BIAS_PAD_CFG2),
	dump_register(DSIA_MIPI_CAL_CONFIG_2),
	dump_register(DSIB_MIPI_CAL_CONFIG_2),
	dump_register(DSIC_MIPI_CAL_CONFIG_2),
	dump_register(DSID_MIPI_CAL_CONFIG_2),
};

static u32 mipical_status;
static u32 timeout_ct;
static u32 counts;

static int dbgfs_show_regs(struct seq_file *s, void *data)
{
	struct tegra_mipi *mipi = s->private;
	int err;

	err = tegra_unpowergate_partition(mipi->soc->powergate_id);
	if (err) {
		dev_err(mipi->dev, "Fail to unpowergate SOR\n");
		return err;
	}

	err = tegra_mipi_clk_enable(mipi);
	if (err) {
		dev_err(mipi->dev, "Fail to enable mipi clk\n");
		goto clk_err;
	}
	debugfs_print_regs32(s, mipical_regs, ARRAY_SIZE(mipical_regs),
				mipi->io, "");
	tegra_mipi_clk_disable(mipi);
	err = 0;

clk_err:
	tegra_powergate_partition(mipi->soc->powergate_id);
	return err;
}

static int dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbgfs_show_regs, inode->i_private);
}

static const struct file_operations dbgfs_ops = {
	.open		= dbgfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

static int dbgfs_mipi_init(struct tegra_mipi *mipi)
{
	struct dentry *dir;
	struct dentry *val;
	int i;

	dir = debugfs_create_dir(DRV_NAME, NULL);
	if (!dir)
		return -ENOMEM;

	val = debugfs_create_x32("LAST_STATUS", S_IRUGO, dir, &mipical_status);
	if (!val)
		goto err;
	val = debugfs_create_u32("COUNT", S_IRUGO | S_IWUGO, dir, &counts);
	if (!val)
		goto err;
	val = debugfs_create_u32("TIMEOUTS", S_IRUGO | S_IWUGO, dir,
				&timeout_ct);
	if (!val)
		goto err;

	val = debugfs_create_file("regs", S_IRUGO, dir, mipi, &dbgfs_ops);
	if (!val)
		goto err;

	/* Assign register address with adjusted offset */
	for (i = 0; i < ARRAY_SIZE(mipical_regs); ++i)
		mipical_regs[i].offset += mipi->soc->addr_offset;
	return 0;
err:
	dev_err(mipi->dev, "%s:Fail to create debugfs\n", __func__);
	debugfs_remove_recursive(dir);
	return -ENODEV;
}
#endif
static int _tegra_mipi_calibration(struct tegra_mipi *mipi, int lanes)
{
	int err;

	mutex_lock(&mipi->lock);
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
	regmap_read(mipi->regmap, ADDR(CIL_MIPI_CAL_STATUS), &mipical_status);
	counts++;
	if (err)
		timeout_ct++;
#endif
	mutex_unlock(&mipi->lock);
	return err;
}

static int tegra_mipical_using_prod(struct tegra_mipi *mipi, int lanes)
{
	int err = -1, val = 0;

	mutex_lock(&mipi->lock);

	/* clean up lanes */
	clear_all(mipi);

	/* Apply MIPI_CAL PROD_Set */
	regmap_read(mipi->regmap, ADDR(MIPI_CAL_MODE), &val);

	if (val & SEL_DPHY_CPHY) { /*CPHY*/

		err = tegra_mipical_prodset_helper(mipi, "prod");
		if (err)
			goto prod_set_fail;

		err = tegra_mipical_prodset_helper(mipi, "prod_c_cphy_csi");
		if (err)
			goto prod_set_fail;

	} else { /*DPHY*/

		if (lanes & (CSIA|CSIB|CSIC|CSID|CSIE|CSIF)) { /*DPHY_CSI*/
			err = tegra_mipical_prodset_helper(mipi, "prod");
			if (err)
				goto prod_set_fail;

			err = tegra_mipical_prodset_helper(mipi,
									"prod_c_dphy_csi");
			if (err)
				goto prod_set_fail;

		} else { /*DPHY_DSI*/

			err = tegra_mipical_prodset_helper(mipi, "prod");
			if (err)
				goto prod_set_fail;

			err = tegra_mipical_prodset_helper(mipi,
									"prod_c_dphy_dsi");
			if (err)
				goto prod_set_fail;

		}

	}

	/*Select lanes */
	select_lanes(mipi, lanes);
	/* Start calibration */
	err = tegra_mipi_wait(mipi, lanes);

#ifdef CONFIG_DEBUG_FS
	regmap_read(mipi->regmap, ADDR(CIL_MIPI_CAL_STATUS), &mipical_status);
	counts++;
	if (err)
		timeout_ct++;
#endif
prod_set_fail:
	mutex_unlock(&mipi->lock);
	return err;

}
int tegra_mipi_calibration(int lanes)
{
	if (!mipi)
		return -ENODEV;
	trace_mipical("lanes", lanes);
	if (mipi->soc->calibrate)
		return mipi->soc->calibrate(mipi, lanes);
	else
		return 0;

}
EXPORT_SYMBOL(tegra_mipi_calibration);

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

	if (!np)
		return;
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
		dsi->clk_hstermos_x =
				(v & HSCLKTERMOSDSIA) >> HSCLKTERMOSDSIA_SHIFT;
	}
	parse_bias_prod(np, &dsi->bias_dsi);
}
static void parse_csi_prod(struct device_node *np,
			   struct tegra_mipi_prod_csi *csi)
{
	int ret;
	unsigned int v;

	if (!np)
		return;
	ret = of_property_read_u32(np, "cilx-cfg", &v);
	if (!ret) {
		csi->overide_x = (v & OVERIDEA) >> OVERIDEA_SHIFT;
		csi->termos_x = (v & TERMOSA) >> TERMOSA_SHIFT;
		csi->termos_x_clk = (v & TERMOSA_CLK) >> TERMOSA_CLK_SHIFT;
	}
	parse_bias_prod(np, &csi->bias_csi);

}
static void parse_prod(struct device_node *np, struct tegra_mipi *mipi)
{
	struct device_node *next;

	if (!np)
		return;
	next = of_get_child_by_name(np, "dsi");
	parse_dsi_prod(next, mipi->prod_dsi);
	next = of_get_child_by_name(np, "csi");
	parse_csi_prod(next, mipi->prod_csi);
}

static void print_config(struct tegra_mipi *mipi)
{
	struct tegra_mipi_prod_csi *csi = mipi->prod_csi;
	struct tegra_mipi_prod_dsi *dsi = mipi->prod_dsi;
#define pr_val(x)	dev_dbg(mipi->dev, "%s:%x", #x, x)

	pr_val(csi->overide_x);
	pr_val(csi->termos_x);
	pr_val(csi->termos_x_clk);
	pr_val(dsi->overide_x);
	pr_val(dsi->hspdos_x);
	pr_val(dsi->hspuos_x);
	pr_val(dsi->termos_x);
	pr_val(dsi->clk_overide_x);
	pr_val(dsi->clk_hspdos_x);
	pr_val(dsi->clk_hspuos_x);
	pr_val(dsi->clk_hstermos_x);
#undef pr_val
}

static int tegra_prod_get_config(struct platform_device *pdev,
				struct tegra_mipi *mipi)
{
	if (mipi->prod_list != NULL)
		return 0;

	mipi->prod_list = devm_tegra_prod_get(&pdev->dev);

	if (IS_ERR(mipi->prod_list)) {
		pr_err("%s: Can not find mipical prod node\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int tegra_mipical_prodset_helper(struct tegra_mipi *mipi,
					char *prod_node)
{
	int err = -ENODEV;
	bool supported_prod = false;

	if (IS_ERR(mipi->prod_list) || !prod_node)
		return -EINVAL;

	supported_prod = tegra_prod_by_name_supported(mipi->prod_list,
						prod_node);

	if (supported_prod == true) {
		err = tegra_prod_set_by_name(&mipi->io, prod_node,
						mipi->prod_list);
		if (err) {
			dev_err(mipi->dev, "%s: prod set (%s) fail (err=%d)\n",
						__func__, prod_node, err);
		}
	}

	return err;
}

static int tegra_mipi_parse_config(struct platform_device *pdev,
		struct tegra_mipi *mipi)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *next;

	if (!np)
		return -ENODEV;

	if (of_device_is_compatible(np, "nvidia,tegra210-mipical"))
		parse_prod(np, mipi);

	if (of_device_is_compatible(np, "nvidia, tegra186-mipical")) {
		if (tegra_chip_get_revision() == TEGRA186_REVISION_A01) {
			dev_dbg(mipi->dev, "T186-A01\n");
			next = of_get_child_by_name(np, "a01");
			parse_prod(next, mipi);
		} else {
			dev_dbg(mipi->dev, "T186-A02\n");
			next = of_get_child_by_name(np, "a02");
			parse_prod(next, mipi);
		}
	}
	print_config(mipi);
	return 0;
}
static const struct tegra_mipi_soc tegra21x_mipi_soc = {
	.total_dsilanes = 4,
	.total_cillanes = 6,
	.addr_offset = 0,
	.ppsb_war = 1,
	.pad_enable = &_t21x_tegra_mipi_bias_pad_enable,
	.pad_disable = &_t21x_tegra_mipi_bias_pad_disable,
	.calibrate = &_tegra_mipi_calibration,
	.parse_cfg = &tegra_mipi_parse_config,
	.powergate_id = TEGRA210_POWER_DOMAIN_SOR,
};

static const struct tegra_mipi_soc tegra18x_mipi_soc = {
	.total_dsilanes = 4,
	.total_cillanes = 6,
	.addr_offset = 4,
	.ppsb_war = 0,
	.pad_enable = &_t18x_tegra_mipi_bias_pad_enable,
	.pad_disable = &_t18x_tegra_mipi_bias_pad_disable,
	.calibrate = &tegra_mipical_using_prod,
	.parse_cfg = &tegra_prod_get_config,
	.powergate_id = TEGRA186_POWER_DOMAIN_DISP,
};

static const struct tegra_mipi_soc tegra_vmipi_soc = {
	.pad_enable = &tegra_vmipi_bias_pad_enable,
	.pad_disable = &tegra_vmipi_bias_pad_disable,
	.calibrate = &tegra_vmipi_calibration,
	.virtual_dev = 1,
};

static const struct of_device_id tegra_mipi_of_match[] = {
	{
		.compatible = "nvidia,tegra210-mipical",
		.data = &tegra21x_mipi_soc,
	}, {
		.compatible = "nvidia, tegra186-mipical",
		.data = &tegra18x_mipi_soc,
	}, {
		.compatible = "nvidia, tegra186-mipical-shared-multi-os",
		.data = &tegra18x_mipi_soc,
	}, {
		.compatible = "nvidia,tegra-mipical-hv",
		.data = &tegra_vmipi_soc,
	}, {
	}
};
MODULE_DEVICE_TABLE(of, tegra_mipi_of_match);

static int tegra_mipi_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int tegra_mipi_release(struct inode *inode, struct file *file)
{
	return 0;
}
static long mipi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tegra_mipi *mipi = container_of(file->private_data,
					struct tegra_mipi, misc_dev);
	if (_IOC_TYPE(cmd) != TEGRA_MIPI_IOCTL_MAGIC)
		return -EINVAL;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(TEGRA_MIPI_IOCTL_BIAS_PAD_CTRL): {
		unsigned int enable = 0;
		int err;

		if (copy_from_user(&enable, (const void __user *)arg,
					sizeof(unsigned int))) {
			dev_err(mipi->dev, "Fail to get user data\n");
			return -EFAULT;
		}
		if (enable)
			err = tegra_mipi_bias_pad_enable();
		else
			err = tegra_mipi_bias_pad_disable();
		return  err;
	}
	case _IOC_NR(TEGRA_MIPI_IOCTL_CAL): {
		int lanes = 0;

		if (copy_from_user(&lanes, (const void __user *)arg,
					sizeof(int))) {
			dev_err(mipi->dev, "Fail to get user data\n");
			return -EFAULT;
		}
		if (lanes)
			return tegra_mipi_calibration(lanes);

		dev_err(mipi->dev, "Selected lane %x, skip mipical\n", lanes);
		return 0;
	}
	default:
		dev_err(mipi->dev, "Unknown ioctl\n");
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations tegra_mipi_miscdev_fops = {
	.owner = THIS_MODULE,
	.open = tegra_mipi_open,
	.release = tegra_mipi_release,
	.unlocked_ioctl = mipi_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mipi_ioctl,
#endif
};

static int tegra_mipi_misc_register(struct tegra_mipi *mipi)
{
	int err;

	if (!mipi)
		return -EINVAL;

	mipi->misc_dev.minor = MISC_DYNAMIC_MINOR;
	mipi->misc_dev.name = DRV_NAME;
	mipi->misc_dev.fops = &tegra_mipi_miscdev_fops;
	err = misc_register(&mipi->misc_dev);
	if (err)
		dev_err(mipi->dev, "Fail to register misc dev\n");

	return err;
}

static int tegra_vmipi_probe(struct platform_device *pdev)
{
	int err;

	err = tegra_vmipi_init(pdev);
	if (err) {
		dev_err(&pdev->dev, "Mipi cal virtual dev probe failed\n");
		return err;
	}

	mutex_init(&mipi->lock);

	err = tegra_mipi_misc_register(mipi);
	if (err)
		tegra_vmipi_deinit();

	return err;
}

static int tegra_mipi_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct resource *mem, *memregion;
	void __iomem *regs;
	int err = 0;
	struct device_node *np;
	const struct tegra_mipi_soc *cdata = NULL;

	np = pdev->dev.of_node;
	match = of_match_device(tegra_mipi_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "No device match found\n");
		return -ENODEV;
	}
	cdata = match->data;
	mipi = devm_kzalloc(&pdev->dev, sizeof(*mipi), GFP_KERNEL);
	if (!mipi)
		return -ENOMEM;

	mipi->dev = &pdev->dev;
	mipi->soc = cdata;
	platform_set_drvdata(pdev, mipi);

	if (mipi->soc->virtual_dev)
		return tegra_vmipi_probe(pdev);

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
	regs = devm_ioremap_nocache(&pdev->dev, mem->start, resource_size(mem));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	mipi->io = regs;
	mipi->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
			&t210_mipi_cal_regmap_config);
	if (IS_ERR(mipi->regmap)) {
		dev_err(&pdev->dev, "Fai to initialize regmap\n");
		return PTR_ERR(mipi->regmap);
	}

	mipi->mipi_cal_clk = devm_clk_get(&pdev->dev, "mipi_cal");
	if (IS_ERR(mipi->mipi_cal_clk))
		return PTR_ERR(mipi->mipi_cal_clk);

	if (of_device_is_compatible(np, "nvidia,tegra210-mipical")) {
		mipi->mipi_cal_fixed = devm_clk_get(&pdev->dev,
				"uart_mipi_cal");
		if (IS_ERR(mipi->mipi_cal_fixed))
			return PTR_ERR(mipi->mipi_cal_fixed);
		mipi->prod_csi = devm_kzalloc(&pdev->dev,
				sizeof(*mipi->prod_csi), GFP_KERNEL);
		mipi->prod_dsi = devm_kzalloc(&pdev->dev,
				sizeof(*mipi->prod_dsi), GFP_KERNEL);
	} else if (of_device_is_compatible(np, "nvidia, tegra186-mipical")) {
		mipi->mipi_cal_fixed = devm_clk_get(&pdev->dev,
				"uart_fs_mipi_cal");
		if (IS_ERR(mipi->mipi_cal_fixed))
			return PTR_ERR(mipi->mipi_cal_fixed);
		mipi->rst = devm_reset_control_get(mipi->dev, "mipi_cal");
		reset_control_deassert(mipi->rst);
		/* Bug 200224083 requires both register fields set to 1
		 * after de-asserted
		 */
		tegra_mipi_clk_enable(mipi);
		mipical_update_bits(mipi->regmap, ADDR(MIPI_BIAS_PAD_CFG0),
				PDVCLAMP, 1 << PDVCLAMP_SHIFT);
		mipical_update_bits(mipi->regmap, ADDR(MIPI_BIAS_PAD_CFG2),
				PDVREG, 1 << PDVREG_SHIFT);
		tegra_mipi_clk_disable(mipi);
	} else if (of_device_is_compatible(np, "nvidia, tegra186-mipical-shared-multi-os")) {
		mipi->mipi_cal_fixed = devm_clk_get(&pdev->dev,
				"uart_fs_mipi_cal");
		if (IS_ERR(mipi->mipi_cal_fixed))
			return PTR_ERR(mipi->mipi_cal_fixed);
		mipi->rst = devm_reset_control_get(mipi->dev, "mipi_cal");
		reset_control_deassert(mipi->rst);
	}

	if (mipi->soc->parse_cfg)
		err = mipi->soc->parse_cfg(pdev, mipi);
	if (err)
		return err;

	mutex_init(&mipi->lock);
	atomic_set(&mipi->refcount, 0);

	err = tegra_mipi_misc_register(mipi);
	if (err)
		return err;

#ifdef CONFIG_DEBUG_FS
	err = dbgfs_mipi_init(mipi);
	if (err)
		dev_err(&pdev->dev, "Fail to create debugfs\n");
#endif

	dev_dbg(&pdev->dev, "Mipi cal done probing...\n");
	return err;
}

static struct platform_driver tegra_mipi_cal_platform_driver = {
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
