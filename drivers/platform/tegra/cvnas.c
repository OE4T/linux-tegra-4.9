/*
 * drivers/platform/tegra/cvnas.c
 *
 * Copyright (C) 2017, NVIDIA Corporation.  All rights reserved.
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

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "cvnas: %s,%d" fmt, __func__, __LINE__

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <soc/tegra/chip-id.h>

static int cvnas_debug;
module_param(cvnas_debug, int, 0644);

#define CVSRAM_MEM_INIT_OFFSET		0x00
#define CVSRAM_MEM_INIT_START		BIT(0)
#define CVSRAM_MEM_INIT_STATUS		BIT(1)

#define CVSRAM_RD_COUNT_OFFSET		0x008
#define CVSRAM_WR_COUNT_OFFSET		0x00B
#define CVSRAM_STALLED_RD_COUNT_OFFSET	0x010
#define CVSRAM_STALLED_WR_COUNT_OFFSET	0x014

#define CVSRAM_PWR_CTRL_OFFSET		0x018

#define CVSRAM_EC_MERR_ENABLE_OFFSET	0x838
#define CVSRAM_EC_MERR_ENABLE		0x7FFFFF

#define CVSRAM_EC_MERR_UNLOCK_OFFSET	0x824
#define CVSRAM_EC_MERR_UNLOCK_INIT	0xE1
#define CVSRAM_EC_MERR_UNLOCK_FINISH	0x0

#define CVSRAM_EC_MERR_FORCE_OFFSET	0x83C
#define CVSRAM_EC_MERR_ECC_INJECT	0x3FFFFC

#define CVNAS_EC_MERR_ENABLE_OFFSET	0xF130
#define CVNAS_EC_MERR_ENABLE		0x3FFF

#define CVNAS_EC_MERR_UNLOCK_OFFSET	0xF11C
#define CVNAS_EC_MERR_UNLOCK_INIT	0xE1
#define CVNAS_EC_MERR_UNLOCK_FINISH	0x0

#define CVNAS_EC_MERR_FORCE_OFFSET	0xF134
#define CVNAS_EC_MERR_ECC_INJECT	0x1FE

#define RST_DEV_CVNAS			0x00
#define RST_DEV_CVNAS_SET		0x04
#define RST_DEV_CVNAS_CLR		0x08
#define RST_DEV_CVNAS_FCM		0x0c
#define RST_DEV_CVNAS_FCM_SET		0x10
#define RST_DEV_CVNAS_FCM_CLR		0x14
#define CLK_OUT_ENB_CVNAS		0x00
#define CLK_OUT_ENB_CVNAS_SET		0x04
#define CLK_OUT_ENB_CVNAS_CLR		0x08

#define SET_CLK_ENB_CVNAS		0x1
#define CLR_CLK_ENB_CVNAS		0x0
#define DEASSERT_CVNAS_RST		0x1
#define ASSERT_CVNAS_RST		0x1
#define DEASSERT_CVNAS_FCM_RST		0x1
#define ASSERT_CVNAS_FCM_RST		0x1

#define HSM_CVSRAM_ECC_CORRECT_OFFSET	0x18A
#define HSM_CVSRAM_ECC_DED_OFFSET_0	0x180
#define HSM_CVSRAM_ECC_DED_OFFSET_1	0x184

#define HSM_CVSRAM_ECC_CORRECT_MASK	0xF000000
#define HSM_CVSRAM_ECC_DED_MASK_0	0x1
#define HSM_CVSRAM_ECC_DED_MASK_1	0x7

struct cvnas_device {
	struct dentry *debugfs_root;

	void __iomem *cvsram_iobase;
	void __iomem *cvreg_iobase;
	void __iomem *car_iobase;
	void __iomem *hsm_iobase;

	struct device dma_dev;

	int nslices;
	int slice_size;
	phys_addr_t cvsram_base;
	size_t cvsram_size;
};

static u32 nvcvnas_car_readl(struct cvnas_device *dev, u32 reg)
{
	return readl(dev->car_iobase + reg);
}

static void nvcvnas_car_writel(struct cvnas_device *dev, u32 val, u32 reg)
{
	writel(val, dev->car_iobase + reg);
}

static u32 nvcvsram_readl(struct cvnas_device *dev, int sid, u32 reg)
{
	return readl(dev->cvsram_iobase + dev->slice_size * sid + reg);
}

static void nvcvsram_writel(struct cvnas_device *dev, int sid, u32 val, u32 reg)
{
	writel(val, dev->cvsram_iobase + dev->slice_size * sid + reg);
}

static u32 nvcvreg_readl(struct cvnas_device *dev, u32 reg)
{
	return readl(dev->cvreg_iobase + reg);
}

static void nvcvreg_writel(struct cvnas_device *dev, u32 val, u32 reg)
{
	writel(val, dev->cvreg_iobase + reg);
}

static u32 nvhsm_readl(struct cvnas_device *dev, u32 reg)
{
	return readl(dev->hsm_iobase + reg);
}

static int cvsram_perf_counters_show(struct seq_file *s, void *data)
{
	struct cvnas_device *dev = (struct cvnas_device *)data;
	int i;
	u32 val;

	if (!dev) {
		seq_printf(s, "Invalid cvnas device!\n");
		return -EINVAL;
	}

	seq_printf(s, "RD:  ");
	for (i = 0; i < dev->nslices; i++) {
		val = nvcvsram_readl(dev, i, CVSRAM_RD_COUNT_OFFSET);
		seq_printf(s, "%x ", val);
	}
	seq_printf(s, "\nWR:  ");
	for (i = 0; i < dev->nslices; i++) {
		val = nvcvsram_readl(dev, i, CVSRAM_WR_COUNT_OFFSET);
		seq_printf(s, "%x ", val);
	}
	seq_printf(s, "\nSRD: ");
	for (i = 0; i < dev->nslices; i++) {
		val = nvcvsram_readl(dev, i, CVSRAM_STALLED_RD_COUNT_OFFSET);
		seq_printf(s, "%x ", val);
	}
	seq_printf(s, "\nSWR: ");
	for (i = 0; i < dev->nslices; i++) {
		val = nvcvsram_readl(dev, i, CVSRAM_STALLED_WR_COUNT_OFFSET);
		seq_printf(s, "%x ", val);
	}
	seq_printf(s, "\n");
	return 0;
}

static int cvsram_perf_counter_open(struct inode *inode, struct file *file)
{
	return single_open(file, cvsram_perf_counters_show,
				inode->i_private);
}

static const struct file_operations cvsram_perf_fops = {
	.open = cvsram_perf_counter_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int cvsram_ecc_err_inject(struct seq_file *s, void *data)
{
	struct cvnas_device *dev = (struct cvnas_device *)s->private;
	int i;
	u32 val;

	if (!dev) {
		seq_printf(s, "Invalid cvnas device!\n");
		return -EINVAL;
	}

	nvcvreg_writel(dev, CVNAS_EC_MERR_ENABLE, CVNAS_EC_MERR_ENABLE_OFFSET);
	if (cvnas_debug) {
		val = nvcvreg_readl(dev, CVNAS_EC_MERR_ENABLE_OFFSET);
		seq_printf(s, "CVNAS_EC_MERR_ENABLE_OFFSET: %x : %x\n",
				CVNAS_EC_MERR_ENABLE_OFFSET, val);
	}
	for (i = 0; i < dev->nslices; i++) {
		nvcvsram_writel(dev, i, CVSRAM_EC_MERR_ENABLE,
					CVSRAM_EC_MERR_ENABLE_OFFSET);
		if (cvnas_debug) {
			val = nvcvreg_readl(dev, CVSRAM_EC_MERR_ENABLE_OFFSET);
			seq_printf(s, "CVSRAM_EC_MERR_ENABLE_OFFSET: %x : %x\n",
					CVSRAM_EC_MERR_ENABLE_OFFSET, val);
		}
	}

	nvcvreg_writel(dev, CVNAS_EC_MERR_UNLOCK_INIT,
				CVNAS_EC_MERR_UNLOCK_OFFSET);
	if (cvnas_debug) {
		val = nvcvreg_readl(dev, CVNAS_EC_MERR_UNLOCK_OFFSET);
		seq_printf(s, "CVNAS_EC_MERR_UNLOCK_OFFSET: %x : %x\n",
				CVNAS_EC_MERR_UNLOCK_OFFSET, val);
	}
	for (i = 0; i < dev->nslices; i++) {
		nvcvsram_writel(dev, i, CVSRAM_EC_MERR_UNLOCK_INIT,
					CVSRAM_EC_MERR_UNLOCK_OFFSET);
		if (cvnas_debug) {
			val = nvcvreg_readl(dev, CVSRAM_EC_MERR_UNLOCK_OFFSET);
			seq_printf(s, "CVSRAM_EC_MERR_UNLOCK_OFFSET: %x : %x\n",
					CVSRAM_EC_MERR_UNLOCK_OFFSET, val);
		}
	}

	nvcvreg_writel(dev, CVNAS_EC_MERR_ECC_INJECT, CVNAS_EC_MERR_FORCE_OFFSET);
	if (cvnas_debug) {
		val = nvcvreg_readl(dev, CVNAS_EC_MERR_FORCE_OFFSET);
		seq_printf(s, "CVNAS_EC_MERR_FORCE_OFFSET: %x : %x\n",
				CVNAS_EC_MERR_FORCE_OFFSET, val);
	}
	for (i = 0; i < dev->nslices; i++) {
		nvcvsram_writel(dev, i, CVSRAM_EC_MERR_ECC_INJECT,
					CVSRAM_EC_MERR_FORCE_OFFSET);
		if (cvnas_debug) {
			val = nvcvreg_readl(dev, CVSRAM_EC_MERR_FORCE_OFFSET);
			seq_printf(s, "CVSRAM_EC_MERR_FORCE_OFFSET: %x : %x\n",
					CVSRAM_EC_MERR_FORCE_OFFSET, val);
		}
	}

	nvcvreg_writel(dev, CVNAS_EC_MERR_UNLOCK_FINISH, CVNAS_EC_MERR_UNLOCK_OFFSET);
	if (cvnas_debug) {
		val = nvcvreg_readl(dev, CVNAS_EC_MERR_UNLOCK_OFFSET);
		seq_printf(s, "CVNAS_EC_MERR_UNLOCK_OFFSET: %x : %x\n",
				CVNAS_EC_MERR_UNLOCK_OFFSET, val);
	}
	for (i = 0; i < dev->nslices; i++) {
		nvcvsram_writel(dev, i, CVSRAM_EC_MERR_UNLOCK_FINISH,
					CVSRAM_EC_MERR_UNLOCK_OFFSET);
		if (cvnas_debug) {
			val = nvcvreg_readl(dev, CVSRAM_EC_MERR_UNLOCK_OFFSET);
			seq_printf(s, "CVSRAM_EC_MERR_UNLOCK_OFFSET: %x : %x\n",
					CVSRAM_EC_MERR_UNLOCK_OFFSET, val);
		}
	}

	val = nvhsm_readl(dev, HSM_CVSRAM_ECC_CORRECT_OFFSET);
	if (val & HSM_CVSRAM_ECC_CORRECT_MASK) {
		seq_printf(s, "HSM received ECC corrected SEC error\n");
	}
	val = nvhsm_readl(dev, HSM_CVSRAM_ECC_DED_OFFSET_0);
	if (val & HSM_CVSRAM_ECC_DED_MASK_0) {
		seq_printf(s, "HSM received ECC DED_0 error\n");
	}
	val = nvhsm_readl(dev, HSM_CVSRAM_ECC_DED_OFFSET_1);
	if (val & HSM_CVSRAM_ECC_DED_MASK_1) {
		seq_printf(s, "HSM received ECC DED_1 error\n");
	}

	return 0;
}

static int cvsram_ecc_err_open(struct inode *inode, struct file *file)
{
	return single_open(file, cvsram_ecc_err_inject,
				inode->i_private);
}

static const struct file_operations cvsram_ecc_err_fops = {
	.open = cvsram_ecc_err_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int nvcvnas_debugfs_init(struct cvnas_device *dev)
{
	struct dentry *root;

	root = debugfs_create_dir("cvnas", NULL);
	if (!root)
		return PTR_ERR(root);

	debugfs_create_x64("cvsram_base", S_IRUGO, root, &dev->cvsram_base);
	debugfs_create_size_t("cvsram_size", S_IRUGO, root, &dev->cvsram_size);
	debugfs_create_file("cvsram_perf_counters", S_IRUGO, root, dev, &cvsram_perf_fops);
	debugfs_create_file("inject_cvsram_ecc_error", S_IRUGO, root, dev, &cvsram_ecc_err_fops);
	dev->debugfs_root = root;
	return 0;
}

static int nvcvsram_ecc_setup(struct cvnas_device *dev)
{
	u32 mem_init = 0;
	int i;

	if (tegra_platform_is_sim())
		return 0;

	/* enable clock if disabled */

	for (i = 0; i < dev->nslices; i++) {
		int retry = 10;

		mem_init = nvcvsram_readl(dev, i, CVSRAM_MEM_INIT_OFFSET);
		if (mem_init & CVSRAM_MEM_INIT_STATUS)
			return 0;
		mem_init |= CVSRAM_MEM_INIT_START;
		nvcvsram_writel(dev, i, mem_init, CVSRAM_MEM_INIT_OFFSET);

		do {
			mem_init = nvcvsram_readl(dev, i,
					CVSRAM_MEM_INIT_OFFSET);
			/* FIXME: Use CCF to make sure clock runs
			 * at fixed frequency and wait for just
			 * that much time.
			 */
			usleep_range(1000, 2000);
			if (!retry--)
				break;
		} while (!(mem_init & CVSRAM_MEM_INIT_STATUS));
	}

	if (mem_init & CVSRAM_MEM_INIT_STATUS)
		return 0;
	return -EBUSY;
}

static int nvcvnas_power_on(struct cvnas_device *cvnas_dev)
{
	int val, i,j;
	u32 fcm_upg_seq[] =
		{0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00};

	if (!tegra_platform_is_qt())
		return 0;

	pr_info("initializing cvnas hardware\n");

	val = nvcvnas_car_readl(cvnas_dev, RST_DEV_CVNAS);
	if (val == 0) {
		pr_err("cvnas is not in assert!!!\n");
		return -ENODEV;
	}

	/* skip changing clock source and divider */

	nvcvnas_car_writel(cvnas_dev, SET_CLK_ENB_CVNAS, CLK_OUT_ENB_CVNAS_SET);
	val = nvcvnas_car_readl(cvnas_dev, CLK_OUT_ENB_CVNAS);
	if (val != 0) {
		pr_err("cvnas clock enable failed\n");
		return -ENODEV;
	}

	nvcvnas_car_writel(cvnas_dev, DEASSERT_CVNAS_RST, RST_DEV_CVNAS_CLR);
	val = nvcvnas_car_readl(cvnas_dev, RST_DEV_CVNAS);
	if (val != 0) {
		pr_err("cvnas deassert reset failed\n");
		return -ENODEV;
	}

	pr_info("initializing cvsram FCMs\n");
	for (i = 0; i < ARRAY_SIZE(fcm_upg_seq); i++) {
		for (j = 0; j < cvnas_dev->nslices; j++) {
			nvcvsram_writel(cvnas_dev, j, fcm_upg_seq[i],
					CVSRAM_PWR_CTRL_OFFSET);
			if (cvnas_debug) {
				u32 val = nvcvsram_readl(cvnas_dev, j,
						CVSRAM_PWR_CTRL_OFFSET);
				pr_info("Set SRAM%d_CVSRAM_PWR_CTRL %x to %x\n",
					j, CVSRAM_PWR_CTRL_OFFSET, val);
			}
		}
	}

	nvcvnas_car_writel(cvnas_dev, DEASSERT_CVNAS_FCM_RST, RST_DEV_CVNAS_FCM_CLR);
	val = nvcvnas_car_readl(cvnas_dev, RST_DEV_CVNAS_FCM);
	if (val != 0) {
		pr_err("cvnas fcm deassert reset failed\n");
		return -ENODEV;
	}

	val = nvcvsram_ecc_setup(cvnas_dev);
	if (val)
		pr_err("ECC init failed\n");
	return val;
}

static int nvcvnas_power_off(struct cvnas_device *cvnas_dev)
{
	int val, i, j;
	u32 fcm_pg_seq[] =
		{0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF};

	if (tegra_platform_is_vdk())
		return 0;

	nvcvnas_car_writel(cvnas_dev, ASSERT_CVNAS_FCM_RST, RST_DEV_CVNAS_FCM_SET);
	val = nvcvnas_car_readl(cvnas_dev, RST_DEV_CVNAS_FCM);
	if (val != ASSERT_CVNAS_FCM_RST) {
		pr_err("cvnas fcm assert reset failed\n");
		return -ENODEV;
	}

	/* FCM low power mode */
	for (i = 0; i < ARRAY_SIZE(fcm_pg_seq); i++) {
		for (j = 0; j < cvnas_dev->nslices; j++) {
			nvcvsram_writel(cvnas_dev, j, fcm_pg_seq[i],
					CVSRAM_PWR_CTRL_OFFSET);
			if (cvnas_debug) {
				val = nvcvsram_readl(cvnas_dev, j,
						CVSRAM_PWR_CTRL_OFFSET);
				pr_info("Set SRAM%d_CVSRAM_PWR_CTRL %x to %x\n",
					j, CVSRAM_PWR_CTRL_OFFSET, val);
			}
		}
	}

	nvcvnas_car_writel(cvnas_dev, ASSERT_CVNAS_RST, RST_DEV_CVNAS_SET);
	val = nvcvnas_car_readl(cvnas_dev, RST_DEV_CVNAS);
	if (val != ASSERT_CVNAS_RST) {
		pr_err("cvnas assert reset failed\n");
		return -ENODEV;
	}

	nvcvnas_car_writel(cvnas_dev, CLR_CLK_ENB_CVNAS, CLK_OUT_ENB_CVNAS_CLR);
	val = nvcvnas_car_readl(cvnas_dev, CLK_OUT_ENB_CVNAS);
	if (val == 0) {
		pr_err("cvnas clock disable failed\n");
		return -ENODEV;
	}
	return 0;
}

int nvmap_register_cvsram_carveout(struct device *dma_dev,
		phys_addr_t base, size_t size);

static int nvcvnas_probe(struct platform_device *pdev)
{
	struct cvnas_device *cvnas_dev;
	int ret;
	u32 cvsram_slice_data[2];
	u32 cvsram_reg_data[4];

	cvnas_dev = (struct cvnas_device *)kzalloc(
			sizeof(*cvnas_dev), GFP_KERNEL);
	if (!cvnas_dev)
		return -ENOMEM;

	cvnas_dev->cvreg_iobase = of_iomap(pdev->dev.of_node, 0);
	if (!cvnas_dev->cvreg_iobase) {
		dev_err(&pdev->dev, "No cvnas reg property found\n");
		ret = PTR_ERR(cvnas_dev->cvreg_iobase);
		goto err_of_iomap;
	}

	cvnas_dev->cvsram_iobase = of_iomap(pdev->dev.of_node, 1);
	if (!cvnas_dev->cvsram_iobase) {
		dev_err(&pdev->dev, "No cvsram reg property found\n");
		ret = PTR_ERR(cvnas_dev->cvsram_iobase);
		goto err_cvsram_of_iomap;
	}

	cvnas_dev->car_iobase = of_iomap(pdev->dev.of_node, 2);
	if (!cvnas_dev->car_iobase) {
		dev_err(&pdev->dev, "No cvnas car reg property found\n");
		ret = PTR_ERR(cvnas_dev->car_iobase);
		goto err_car_of_iomap;
	}

	cvnas_dev->hsm_iobase = of_iomap(pdev->dev.of_node, 2);
	if (!cvnas_dev->hsm_iobase) {
		dev_err(&pdev->dev, "No hsm reg property found\n");
		ret = PTR_ERR(cvnas_dev->hsm_iobase);
		goto err_hsm_of_iomap;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
		"cvsramslice", cvsram_slice_data,
		ARRAY_SIZE(cvsram_slice_data));
	if (ret) {
		dev_err(&pdev->dev, "no cvsramslice property found\n");
		goto err_cvsram_get_slice_data;
	}
	cvnas_dev->nslices = cvsram_slice_data[0];
	cvnas_dev->slice_size = cvsram_slice_data[1];

	ret = of_property_read_u32_array(pdev->dev.of_node,
		"cvsram-reg", cvsram_reg_data,
		ARRAY_SIZE(cvsram_reg_data));
	if (ret) {
		dev_err(&pdev->dev, "no cvsram-reg property found\n");
		goto err_cvsram_get_reg_data;
	}

	cvnas_dev->cvsram_base = ((u64)cvsram_reg_data[0]) << 32;
	cvnas_dev->cvsram_base |= cvsram_reg_data[1];
	cvnas_dev->cvsram_size = ((u64)cvsram_reg_data[2]) << 32;
	cvnas_dev->cvsram_size |= cvsram_reg_data[3];

	ret = nvcvnas_power_on(cvnas_dev);
	if (ret) {
		dev_err(&pdev->dev, "ECC init failed. ret=%d\n", ret);
		goto err_cvsram_ecc_setup;
	}

	ret = nvcvnas_debugfs_init(cvnas_dev);
	if (ret) {
		dev_err(&pdev->dev, "debugfs init failed. ret=%d\n", ret);
		goto err_cvnas_debugfs_init;
	}

	ret = nvmap_register_cvsram_carveout(&cvnas_dev->dma_dev,
			cvnas_dev->cvsram_base, cvnas_dev->cvsram_size);
	if (ret) {
		dev_err(&pdev->dev,
			"nvmap cvsram register failed. ret=%d\n", ret);
		goto err_cvsram_nvmap_heap_register;
	}

	dev_set_drvdata(&pdev->dev, cvnas_dev);

	/* TODO: Add interrupt handler */

	return 0;
err_cvsram_nvmap_heap_register:
	debugfs_remove(cvnas_dev->debugfs_root);
err_cvnas_debugfs_init:
err_cvsram_ecc_setup:
err_cvsram_get_reg_data:
err_cvsram_get_slice_data:
	iounmap(cvnas_dev->hsm_iobase);
err_hsm_of_iomap:
	iounmap(cvnas_dev->car_iobase);
err_car_of_iomap:
	iounmap(cvnas_dev->cvsram_iobase);
err_cvsram_of_iomap:
	iounmap(cvnas_dev->cvreg_iobase);
err_of_iomap:
	kfree(cvnas_dev);
	return ret;
}

static int nvcvnas_remove(struct platform_device *pdev)
{
	struct cvnas_device *cvnas_dev;
	int ret;

	cvnas_dev = dev_get_drvdata(&pdev->dev);
	if (!cvnas_dev)
		return -ENODEV;

	ret = nvcvnas_power_off(cvnas_dev);
	if (ret)
		return ret;
	debugfs_remove(cvnas_dev->debugfs_root);
	of_reserved_mem_device_release(&pdev->dev);
	iounmap(cvnas_dev->car_iobase);
	iounmap(cvnas_dev->cvsram_iobase);
	iounmap(cvnas_dev->cvreg_iobase);
	kfree(cvnas_dev);
	return 0;
}

static void nvcvnas_shutdown(struct platform_device *pdev)
{
	struct cvnas_device *cvnas_dev;
	int ret;

	cvnas_dev = dev_get_drvdata(&pdev->dev);
	if (!cvnas_dev) {
		dev_err(&pdev->dev, "shutdown fail\n");
		return;
	}

	ret = nvcvnas_power_off(cvnas_dev);
	if (ret)
		dev_err(&pdev->dev, "power off fail\n");
}

/* TODO: Add runtime power management */
#ifdef CONFIG_PM_SLEEP
static int nvcvnas_suspend(struct device *dev)
{
	struct cvnas_device *cvnas_dev;

	cvnas_dev = dev_get_drvdata(dev);
	if (!cvnas_dev)
		return -ENODEV;

	return nvcvnas_power_off(cvnas_dev);
}

static int nvcvnas_resume(struct device *dev)
{
	struct cvnas_device *cvnas_dev;
	int ret;

	cvnas_dev = dev_get_drvdata(dev);
	if (!cvnas_dev) {
		dev_err(dev, "empty drvdata!\n");
		return -ENODEV;
	}

	ret = nvcvnas_power_on(cvnas_dev);
	if (ret) {
		dev_err(dev, "cvnas power on failed\n");
		return ret;
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(nvcvnas_pm_ops, nvcvnas_suspend, nvcvnas_resume);
#define NVCVNAS_PM_OPS (&nvcvnas_pm_ops)
#else
#define NVCVNAs_PM_OPS NULL
#endif

static const struct of_device_id nvcvnas_of_ids[] = {
	{ .compatible = "nvidia,tegra-cvnas" },
	{ }
};

static struct platform_driver nvcvnas_driver = {
	.driver = {
		.name	= "tegra-cvnas",
		.owner	= THIS_MODULE,
		.of_match_table = nvcvnas_of_ids,
		.pm = NVCVNAS_PM_OPS,
	},

	.probe		= nvcvnas_probe,
	.remove		= nvcvnas_remove,
	.shutdown	= nvcvnas_shutdown,
};

static int __init nvcvnas_init(void)
{
	int ret;

	ret = platform_driver_register(&nvcvnas_driver);
	if (ret)
		return ret;

	return 0;
}
module_init(nvcvnas_init);

static void __exit nvcvnas_exit(void)
{
}
module_exit(nvcvnas_exit);
