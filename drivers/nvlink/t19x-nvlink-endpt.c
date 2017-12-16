/*
 * t19x-nvlink-endpt.c:
 * This is the NVLINK endpoint driver for the Tegra NVLINK controller.
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/of.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/mc-regs-t19x.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/tegra_pm_domains.h>
#include <linux/tegra-powergate.h>

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"

#define PLLNVHS_FREQ_150MHZ	(150 * 1000 * 1000)

/* NVLINK TOM is the top of the NVLINK aperture */
#define NVLINK_TOM_GB	512

/* Convert the NVLINK TOM value from GB to MB for register programming */
#define NVLINK_TOM_MB	(((NVLINK_TOM_GB) * 1024) - 1)

static struct of_device_id t19x_nvlink_controller_of_match[] = {
	{
		.compatible     = "nvidia,t19x-nvlink-controller",
	}, {
	},
};

#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
static struct of_device_id tegra_nvl_pd[] = {
	{ .compatible = "nvidia,tegra194-nvl-pd", },
	{},
};
#endif

MODULE_DEVICE_TABLE(of, t19x_nvlink_controller_of_match);

u32 nvlw_tioctrl_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(ndev->nvlw_tioctrl_base + reg);
}

void nvlw_tioctrl_writel(struct nvlink_device *ndev, u32 reg, u32 val)
{
	writel(val, ndev->nvlw_tioctrl_base + reg);
}

u32 nvlw_nvlipt_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(ndev->nvlw_nvlipt_base + reg);
}

void nvlw_nvlipt_writel(struct nvlink_device *ndev, u32 reg, u32 val)
{
	writel(val, ndev->nvlw_nvlipt_base + reg);
}

u32 nvlw_nvl_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(ndev->link.nvlw_nvl_base + reg);
}

void nvlw_nvl_writel(struct nvlink_device *ndev, u32 reg, u32 val)
{
	writel(val, ndev->link.nvlw_nvl_base + reg);
}

u32 nvlw_sync2x_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(((struct tegra_nvlink_device *)(ndev->priv))
						->nvlw_sync2x_base + reg);
}

void nvlw_sync2x_writel(struct nvlink_device *ndev, u32 reg, u32 val)
{
	writel(val, ((struct tegra_nvlink_device *)(ndev->priv))
						->nvlw_sync2x_base + reg);
}

u32 nvlw_nvltlc_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(ndev->link.nvlw_nvltlc_base + reg);
}

void nvlw_nvltlc_writel(struct nvlink_device *ndev, u32 reg, u32 val)
{
	writel(val, ndev->link.nvlw_nvltlc_base + reg);
}

static inline u32 mssnvlink_0_readl(struct nvlink_device *ndev, u32 reg)
{
	struct tegra_nvlink_link *priv =
			(struct tegra_nvlink_link *)(ndev->link.priv);
	return readl(priv->mssnvlink_0_base + reg);
}

static inline void mssnvlink_0_writel(struct nvlink_device *ndev, u32 reg,
									u32 val)
{
	struct tegra_nvlink_link *priv =
			(struct tegra_nvlink_link *)(ndev->link.priv);
	writel(val, priv->mssnvlink_0_base + reg);
}

/* TODO: Remove all non-NVLINK MMIO register writes from the driver */
static inline void non_nvlink_writel(u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(reg, 0x4);

	__raw_writel(val, ptr);
	iounmap(ptr);
}

/*
 * Wait for a bit to be set or cleared in an NVLINK register. If the desired bit
 * condition doesn't happen in a certain amount of time, a timeout will happen.
 */
int wait_for_reg_cond_nvlink(
			struct nvlink_device *ndev,
			u32 reg,
			u32 bit,
			int bit_set,
			char *bit_name,
			u32 (*reg_readl)(struct nvlink_device *, u32),
			u32 *reg_val)
{
	u32 elapsed_us = 0;

	do {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US*2);
		elapsed_us += DEFAULT_LOOP_SLEEP_US;

		*reg_val = reg_readl(ndev, reg);
		if ((bit_set && (*reg_val & BIT(bit))) ||
		    (!bit_set && ((*reg_val & BIT(bit)) == 0)))
			break;
	} while (elapsed_us < DEFAULT_LOOP_TIMEOUT_US);
	if (elapsed_us >= DEFAULT_LOOP_TIMEOUT_US) {
		if (bit_set) {
			nvlink_err("Timeout waiting for the %s bit to get set",
				bit_name);
		} else {
			nvlink_err(
				"Timeout waiting for the %s bit to get cleared",
				bit_name);
		}
		return -1;
	}

	return 0;
}

/*
 * tegra_nvlink_car_init(): initializes UPHY mgmt and sys clk
 * clears the resets to uphy
 */
static int tegra_nvlink_car_enable(struct nvlink_device *ndev)
{
	struct tegra_nvlink_device *tnvlink_dev = ndev->priv;
	int ret;
	unsigned long clk_rate = 0;
	u32 reg_val = 0;

	/* enable the Management clock */
	ret = clk_prepare_enable(tnvlink_dev->clk_nvhs_pll0_mgmt);
	if (ret < 0) {
		nvlink_err("nvlink mgmt clock enable failed : %d", ret);
		goto fail;
	}
	/* Clear reset for UPHY PM */
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_pm);
	/* Enable clock for nvlink_sys */
	ret = clk_prepare_enable(tnvlink_dev->clk_nvlink_sys);
	if (ret < 0) {
		nvlink_err("nvlink sys clock enable failed : %d", ret);
		goto nvlink_sys_fail;
	}

#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
	ret = tegra_unpowergate_partition(tnvlink_dev->pgid_nvl);
	if (ret < 0) {
		nvlink_err("Couldn't unpowergate : %d", ret);
		goto unpowergate_partition_fail;
	}
#endif
	/* Take link out of reset */
	reg_val = nvlw_tioctrl_readl(ndev, NVLW_RESET) |
			BIT(NVLW_RESET_LINKRESET);
	nvlw_tioctrl_writel(ndev, NVLW_RESET, reg_val);
	udelay(NVLW_POST_RESET_DELAY_US);

	/* Reset persistent HW state for this link */
	reg_val = nvlw_tioctrl_readl(ndev, NVLW_DEBUG_RESET) &
			~BIT(NVLW_DEBUG_RESET_LINK);
	nvlw_tioctrl_writel(ndev, NVLW_DEBUG_RESET, reg_val);
	udelay(NVLW_POST_RESET_DELAY_US);
	reg_val = nvlw_tioctrl_readl(ndev, NVLW_DEBUG_RESET) |
					BIT(NVLW_DEBUG_RESET_LINK) |
					BIT(NVLW_DEBUG_RESET_COMMON);
	nvlw_tioctrl_writel(ndev, NVLW_DEBUG_RESET, reg_val);
	udelay(NVLW_POST_RESET_DELAY_US);

	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_l0);
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_l1);
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_l2);
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_l3);
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_l4);
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_l5);
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_l6);
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_l7);
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy);
	reset_control_deassert(tnvlink_dev->rst_nvhs_uphy_pll0);

	ret = clk_set_rate(tnvlink_dev->clk_pllnvhs, PLLNVHS_FREQ_150MHZ);
	if (ret < 0) {
		nvlink_err("nvlink pllnvhs setrate failed : %d", ret);
		goto pllnvhs_fail;
	} else {
		clk_rate = clk_get_rate(tnvlink_dev->clk_pllnvhs);
		if (clk_rate != PLLNVHS_FREQ_150MHZ) {
			nvlink_err("clk_pllnvhs rate = %lu", clk_rate);
			ret = -EINVAL;
			goto pllnvhs_fail;
		}
		ret = clk_prepare_enable(tnvlink_dev->clk_pllnvhs);
		if (ret < 0) {
			nvlink_err("pllnvhs clock enable failed : %d", ret);
			goto pllnvhs_fail;
		}
	}
	return ret;

pllnvhs_fail:
#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
	tegra_powergate_partition(tnvlink_dev->pgid_nvl);
#endif
unpowergate_partition_fail:
	clk_disable_unprepare(tnvlink_dev->clk_nvlink_sys);
nvlink_sys_fail:
	clk_disable_unprepare(tnvlink_dev->clk_nvhs_pll0_mgmt);
fail:
	return ret;
}

static int switch_to_tx_ref_clk(struct nvlink_device *ndev)
{
	int ret = 0;
	struct tegra_nvlink_device *tnvlink_dev = ndev->priv;

	nvlink_dbg("NVLIPT - Switching to TXREFCLK");

	ret = clk_prepare_enable(tnvlink_dev->clk_txclk_ctrl);
	if (ret < 0)
		nvlink_err("nvlink txclk_control enable failed : %d\n", ret);
	return ret;
}

static int init_nvhs(struct nvlink_device *ndev)
{
	int ret = 0;

	nvlink_dbg("Initializing NVHS");

	ret = switch_to_tx_ref_clk(ndev);
	if (ret < 0)
		goto fail;

	goto success;

fail:
	nvlink_err("NVHS init failed!");
success:
	return ret;
}

static void init_dlpl(struct nvlink_device *ndev)
{
	u32 reg_val = 0;

	nvlink_dbg("Initializing DLPL");

	/* Enable link */
	reg_val = nvlw_nvl_readl(ndev, NVL_LINK_CONFIG) |
			BIT(NVL_LINK_CONFIG_LINK_EN) | BIT(3);
	nvlw_nvl_writel(ndev, NVL_LINK_CONFIG, reg_val);

	nvlw_nvl_writel(ndev, NVL_SL0_TRAIN0_TX, 0x63);
	nvlw_nvl_writel(ndev, NVL_SL0_TRAIN1_TX, 0xf0);

	nvlw_nvl_writel(ndev, NVL_SL0_SAFE_CTRL2_TX, 0x2f53);

	nvlw_nvl_writel(ndev, NVL_SL1_CONFIG_RX, 0x70001000);

	nvlw_nvl_writel(ndev, NVL_SUBLINK_CHANGE, 0x200000);

	nvlw_nvl_writel(ndev, NVL_SL1_RXSLSM_TIMEOUT_2, 0xfa0);
}

static void init_tlc_buffers(struct nvlink_device *ndev)
{
	nvlink_dbg("Initializing TLC buffers");

	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC0, 0x10000C0);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC1, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC2, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC3, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC4, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC5, 0x10000C0);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC6, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC7, 0x0);

	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC0, 0xff00bf);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC1, 0xff00bf);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC2, 0xff00bf);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC3, 0xff00bf);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC4, 0xff00bf);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC5, 0x1ff017f);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC6, 0x1ff017f);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC7, 0x1ff017f);

	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC0, 0x800040);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC1, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC2, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC3, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC4, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC5, 0x800040);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC6, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC7, 0x0);

	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC0, 0x7f003f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC1, 0x7f003f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC2, 0x7f003f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC3, 0x7f003f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC4, 0x7f003f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC5, 0xff007f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC6, 0xff007f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC7, 0xff007f);

	nvlw_nvltlc_writel(ndev, NVLTLC_TX_ERR_LOG_EN_0, 0x3ffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_ERR_REPORT_EN_0, 0x3ffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_ERR_CONTAIN_EN_0, 0x3ffffff);

	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_LOG_EN_0, 0xffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_REPORT_EN_0, 0xffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_CONTAIN_EN_0, 0xffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_LOG_EN_1, 0x3fffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_REPORT_EN_1, 0x3fffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_CONTAIN_EN_1, 0x3fffff);

	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_READY, 0x1);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_READY, 0x1);
}

/*
 * mssnvlink_init:
 * Do the folllwing to initialize MSSNVLINK. This initialization is required to
 * allow traffic to flow between the NVLINK controller and MSSNVLINK:
 *    - Program the upper limit of the NVLINK aperture in MSSNVLINK. The bottom
 *      of the aperture is fixed at 128 GB. So we don't need to program that.
 *    - Release MSSNVLINK header and data credits to the NVLINK controller.
 *
 * TODO: Convert the magic values being programmed below into something that's
 * more understandable.
 */
static void mssnvlink_init(struct nvlink_device *ndev)
{
	/* Program the upper limit of the NVLINK aperture in MSSNVLINK */
	nvlink_dbg("Programming MSSNVLINK_TOM to %u GB", NVLINK_TOM_GB);
	non_nvlink_writel(MCB_BASE + MC_MSSNVLINK_TOM, NVLINK_TOM_MB);
	non_nvlink_writel(MCB_BASE + MC_MSSNVLINK_REG_CTRL, 0x1);

	/* MSSNVLINK credit programming */
	mssnvlink_0_writel(ndev, MSSNVLINK_MASTER_CREDIT_TRANSINFO, 0x15455000);
	mssnvlink_0_writel(ndev, MSSNVLINK_MASTER_CREDIT_INGR_DATA, 0x8020000);
	mssnvlink_0_writel(ndev, MSSNVLINK_SLAVE_CREDIT_TRANSINFO, 0x14050000);
	mssnvlink_0_writel(ndev, MSSNVLINK_SLAVE_CREDIT_INGR_DATA, 0x300c0000);

	/*
	 * Performance settings for balancing request and response bandwidth
	 * across NVLINK
	 */
	non_nvlink_writel(MCB_BASE + MC_MCF_IREQX_VCARB_CONFIG, 0x8f0);
	non_nvlink_writel(MCB_BASE + MC_MCF_OREQX_VCARB_CONFIG, 0x8f0);
}

/*
 * Program the upper limit of the NVLINK aperture in SCF.
 * The bottom of the aperture is fixed at 128 GB. So we don't need to program
 * that.
 */
static inline void program_scf_tom(void)
{
	u32 reg_val = SCF_NVLINK_CFG_TOM_MB_F(NVLINK_TOM_MB) |
			BIT(SCF_NVLINK_CFG_EN);

	nvlink_dbg("Programming SCF TOM to %u GB", NVLINK_TOM_GB);
	asm volatile("msr s3_0_c15_c0_3, %0" : : "r" (reg_val));
}

/* Initialize the link and transition to SAFE mode */
int t19x_nvlink_endpt_enable_link(struct nvlink_device *ndev)
{
	int ret = 0;

	nvlink_dbg("Initializing link ...");
	tegra_nvlink_car_enable(ndev);
	minion_boot(ndev);
	nvlink_config_common_intr(ndev);
	ret = init_nvhs(ndev);
	if (ret < 0)
		goto fail;
	udelay(1);

	init_dlpl(ndev);
	ret = go_to_safe_mode(ndev);
	if (ret < 0)
		goto fail;

	nvlink_enable_link_interrupts(ndev);

	init_tlc_buffers(ndev);
	mssnvlink_init(ndev);
	program_scf_tom();

	nvlink_dbg("Link initialization succeeded!");
	goto success;

fail:
	nvlink_err("Link initialization failed!");
success:
	return ret;
}

static int tegra_nvlink_clk_rst_init(struct nvlink_device *ndev)
{
	struct tegra_nvlink_device *tnvlink_dev = ndev->priv;

	/* clocks */
	tnvlink_dev->clk_nvhs_pll0_mgmt = devm_clk_get(ndev->dev,
			"nvhs_pll0_mgmt");
	if (IS_ERR(tnvlink_dev->clk_nvhs_pll0_mgmt)) {
		nvlink_err("missing mgmt clock");
		return PTR_ERR(tnvlink_dev->clk_nvhs_pll0_mgmt);
	}

	tnvlink_dev->clk_nvlink_sys = devm_clk_get(ndev->dev,
			"nvlink_sys");
	if (IS_ERR(tnvlink_dev->clk_nvlink_sys)) {
		nvlink_err("missing sys clock");
		return PTR_ERR(tnvlink_dev->clk_nvlink_sys);
	}

	tnvlink_dev->clk_pllnvhs = devm_clk_get(ndev->dev,
			"pllnvhs");
	if (IS_ERR(tnvlink_dev->clk_pllnvhs)) {
		nvlink_err("missing pllnvhs clock");
		return PTR_ERR(tnvlink_dev->clk_pllnvhs);
	}

	tnvlink_dev->clk_txclk_ctrl = devm_clk_get(ndev->dev,
			"txclk_ctrl");
	if (IS_ERR(tnvlink_dev->clk_txclk_ctrl)) {
		nvlink_err("missing txclk_ctrl clock");
		return PTR_ERR(tnvlink_dev->clk_txclk_ctrl);
	}

	/* Resets */
	tnvlink_dev->rst_nvhs_uphy_pm = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_pm");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_pm)) {
		nvlink_err("missing rst_nvhs_uphy_pm reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_pm);
	}
	tnvlink_dev->rst_nvhs_uphy = devm_reset_control_get(ndev->dev,
			"nvhs_uphy");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy)) {
		nvlink_err("missing rst_nvhs_uphy reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy);
	}
	tnvlink_dev->rst_nvhs_uphy_pll0 = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_pll0");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_pll0)) {
		nvlink_err("missing rst_nvhs_uphy_pll0 reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_pll0);
	}
	tnvlink_dev->rst_nvhs_uphy_l0 = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_l0");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_l0)) {
		nvlink_err("missing rst_nvhs_uphy_l0 reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_l0);
	}
	tnvlink_dev->rst_nvhs_uphy_l1 = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_l1");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_l1)) {
		nvlink_err("missing rst_nvhs_uphy_l1 reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_l1);
	}
	tnvlink_dev->rst_nvhs_uphy_l2 = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_l2");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_l2)) {
		nvlink_err("missing rst_nvhs_uphy_l2 reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_l2);
	}
	tnvlink_dev->rst_nvhs_uphy_l3 = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_l3");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_l3)) {
		nvlink_err("missing rst_nvhs_uphy_l3 reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_l3);
	}
	tnvlink_dev->rst_nvhs_uphy_l4 = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_l4");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_l4)) {
		nvlink_err("missing rst_nvhs_uphy_l4 reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_l4);
	}
	tnvlink_dev->rst_nvhs_uphy_l5 = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_l5");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_l5)) {
		nvlink_err("missing rst_nvhs_uphy_l5 reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_l5);
	}
	tnvlink_dev->rst_nvhs_uphy_l6 = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_l6");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_l6)) {
		nvlink_err("missing rst_nvhs_uphy_l6 reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_l6);
	}
	tnvlink_dev->rst_nvhs_uphy_l7 = devm_reset_control_get(ndev->dev,
			"nvhs_uphy_l7");
	if (IS_ERR(tnvlink_dev->rst_nvhs_uphy_l7)) {
		nvlink_err("missing rst_nvhs_uphy_l7 reset");
		return PTR_ERR(tnvlink_dev->rst_nvhs_uphy_l7);
	}

#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
	tnvlink_dev->pgid_nvl = tegra_pd_get_powergate_id(tegra_nvl_pd);
#endif

	return 0;
}

static void tegra_nvlink_clk_rst_deinit(struct nvlink_device *ndev)
{
	struct tegra_nvlink_device *tnvlink_dev = ndev->priv;

	/* clocks */
	if (tnvlink_dev->clk_nvhs_pll0_mgmt)
		devm_clk_put(ndev->dev, tnvlink_dev->clk_nvhs_pll0_mgmt);

	if (tnvlink_dev->clk_nvlink_sys)
		devm_clk_put(ndev->dev, tnvlink_dev->clk_nvlink_sys);

	if (tnvlink_dev->clk_pllnvhs)
		devm_clk_put(ndev->dev, tnvlink_dev->clk_pllnvhs);

	if (tnvlink_dev->clk_txclk_ctrl)
		devm_clk_put(ndev->dev, tnvlink_dev->clk_txclk_ctrl);

	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_pm);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_pll0);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_l0);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_l1);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_l2);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_l3);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_l4);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_l5);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_l6);
	reset_control_assert(tnvlink_dev->rst_nvhs_uphy_l7);

}

static int t19x_nvlink_endpt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct nvlink_device *ndev;
	struct tegra_nvlink_link *tegra_link = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *endpt_dt_node = NULL;
	struct device *dev = NULL;

	if (!np) {
		nvlink_err("Invalid device_node");
		ret = -ENODEV;
		goto err_dt_node;
	}

	ndev = kzalloc(sizeof(struct nvlink_device), GFP_KERNEL);
	if (!ndev) {
		nvlink_err("Couldn't allocate memory for t19x device struct");
		ret = -ENOMEM;
		goto err_alloc_device;
	}

	ndev->priv =
		(void*) kzalloc(sizeof(struct tegra_nvlink_device), GFP_KERNEL);
	if (!ndev->priv) {
		nvlink_err("Couldn't allocate memory for tegra_nvlink_device");
		ret = -ENOMEM;
		goto err_alloc_priv;
	}

	ndev->link.priv =
		(void *) kzalloc(sizeof(struct tegra_nvlink_link), GFP_KERNEL);
	if (!ndev->link.priv) {
		nvlink_err("Couldn't allocate memory for link's private data");
		ret = -ENOMEM;
		goto err_alloc_link_priv;
	}
	tegra_link = (struct tegra_nvlink_link *)(ndev->link.priv);

	ndev->dev = &pdev->dev;
	ndev->class.owner = THIS_MODULE;
	ndev->class.name = NVLINK_DRV_NAME;
	platform_set_drvdata(pdev, ndev);

	ret = tegra_nvlink_clk_rst_init(ndev);
	if (ret)
		goto err_clk_rst;

	/* Map NVLINK apertures listed in device tree node */
	ndev->nvlw_tioctrl_base =
				of_io_request_and_map(np, 0,
						"NVLW_TIOCTRL aperture");
	if (IS_ERR(ndev->nvlw_tioctrl_base)) {
		nvlink_err("Couldn't map the NVLW_TIOCTRL aperture");
		ret = PTR_ERR(ndev->nvlw_tioctrl_base);
		goto err_mapping;
	}

	ndev->nvlw_nvlipt_base =
				of_io_request_and_map(np, 1,
						"NVLW_NVLIPT aperture");
	if (IS_ERR(ndev->nvlw_nvlipt_base)) {
		nvlink_err("Couldn't map the NVLW_NVLIPT aperture");
		ret = PTR_ERR(ndev->nvlw_nvlipt_base);
		goto err_mapping;
	}

	ndev->nvlw_minion_base =
				of_io_request_and_map(np, 2,
						"NVLW_MINION aperture");
	if (IS_ERR(ndev->nvlw_minion_base)) {
		nvlink_err("Couldn't map the NVLW_MINION aperture");
		ret = PTR_ERR(ndev->nvlw_minion_base);
		goto err_mapping;
	}

	ndev->link.nvlw_nvl_base =
				of_io_request_and_map(np, 3,
						"NVLW_NVL aperture");
	if (IS_ERR(ndev->link.nvlw_nvl_base)) {
		nvlink_err("Couldn't map the NVLW_NVL aperture");
		ret = PTR_ERR(ndev->link.nvlw_nvl_base);
		goto err_mapping;
	}

	((struct tegra_nvlink_device *)(ndev->priv))->
			nvlw_sync2x_base = of_io_request_and_map(np, 4,
							"NVLW_SYNC2X aperture");
	if (IS_ERR(((struct tegra_nvlink_device *)(ndev->priv))->
							nvlw_sync2x_base)) {
		nvlink_err("Couldn't map the NVLW_SYNC2X aperture");
		ret = PTR_ERR(((struct tegra_nvlink_device *)(ndev->priv))->
							nvlw_sync2x_base);
		goto err_mapping;
	}

	ndev->link.nvlw_nvltlc_base =
				of_io_request_and_map(np, 5,
						"NVLW_NVLTLC aperture");
	if (IS_ERR(ndev->link.nvlw_nvltlc_base)) {
		nvlink_err("Couldn't map the NVLW_NVLTLC aperture");
		ret = PTR_ERR(ndev->link.nvlw_nvltlc_base);
		goto err_mapping;
	}

	tegra_link->mssnvlink_0_base = of_io_request_and_map(np, 6,
							"MSSNVLINK_0 aperture");
	if (IS_ERR(tegra_link->mssnvlink_0_base)) {
		nvlink_err("Couldn't map the MSSNVLINK_0 aperture");
		ret = PTR_ERR(tegra_link->mssnvlink_0_base);
		goto err_mapping;
	}

	/* Read NVLINK topology information in device tree */
	endpt_dt_node = of_get_child_by_name(np, "endpoint");
	of_property_read_u32(endpt_dt_node, "local_dev_id", &ndev->device_id);
	of_property_read_u32(endpt_dt_node, "local_link_id", &ndev->link.link_id);
	ndev->is_master = of_property_read_bool(endpt_dt_node, "is_master");
	of_property_read_u32(endpt_dt_node, "remote_dev_id", &ndev->link.remote_dev_info.device_id);
	of_property_read_u32(endpt_dt_node, "remote_link_id", &ndev->link.remote_dev_info.link_id);

	nvlink_dbg("Device Tree Topology Information:");
	nvlink_dbg("  - Local Device: Device ID = %d, Link ID = %d, Is master? = %s",
		ndev->device_id,
		ndev->link.link_id,
		ndev->is_master ? "True" : "False");
	nvlink_dbg("  - Remote Device: Device ID = %d, Link ID = %d",
		ndev->link.remote_dev_info.device_id,
		ndev->link.remote_dev_info.link_id);

	/* Fill in the link struct */
	ndev->link.device_id = ndev->device_id;
	ndev->link.link_ops.enable_link = t19x_nvlink_endpt_enable_link;
	ndev->link.link_ops.get_link_mode = t19x_nvlink_get_link_mode;
	ndev->link.link_ops.set_link_mode = t19x_nvlink_set_link_mode;
	ndev->link.link_ops.get_sublink_mode = t19x_nvlink_get_sublink_mode;
	ndev->link.link_ops.set_sublink_mode = t19x_nvlink_set_sublink_mode;
	ndev->link.link_ops.get_link_state = t19x_nvlink_get_link_state;
	ndev->link.link_ops.get_tx_sublink_state =
					t19x_nvlink_get_tx_sublink_state;
	ndev->link.link_ops.get_rx_sublink_state =
					t19x_nvlink_get_rx_sublink_state;

	/* Create device node */
	ret = class_register(&ndev->class);
	if (ret) {
		nvlink_err("Failed to register class");
		goto err_mapping;
	}

	ret = alloc_chrdev_region(&ndev->dev_t, 0, 1, dev_name(ndev->dev));
	if (ret) {
		nvlink_err("Failed to allocate dev_t");
		goto err_chrdev_region;
	}

	cdev_init(&ndev->cdev, &t19x_nvlink_endpt_ops);
	ndev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&ndev->cdev, ndev->dev_t, 1);
	if (ret) {
		nvlink_err("Failed to add cdev");
		goto err_cdev;
	}

	dev = device_create(&ndev->class,
				NULL,
				ndev->dev_t,
				NULL,
				NVLINK_DRV_NAME);
	if (IS_ERR(dev)) {
		nvlink_err("Failed to create device");
		ret = PTR_ERR(dev);
		goto err_device;
	}

	/* Register device with core driver*/
	ret = nvlink_register_device(ndev);
	if (ret < 0) {
		goto err_ndev_register;
	}

	/* Register link with core driver */
	ret = nvlink_register_link(&ndev->link);
	if (ret < 0) {
		goto err_nlink_register;
	}

	t19x_nvlink_endpt_debugfs_init(ndev);

	nvlink_dbg("Probe successful!");
	goto success;

err_nlink_register:
	nvlink_unregister_device(ndev);
err_ndev_register:
	device_destroy(&ndev->class, ndev->dev_t);
err_device:
	cdev_del(&ndev->cdev);
err_cdev:
	unregister_chrdev_region(ndev->dev_t, 1);
err_chrdev_region:
	class_unregister(&ndev->class);
err_mapping:
	if (!IS_ERR(ndev->nvlw_tioctrl_base))
		iounmap(ndev->nvlw_tioctrl_base);

	if (!IS_ERR(ndev->nvlw_nvlipt_base))
		iounmap(ndev->nvlw_nvlipt_base);

	if (!IS_ERR(ndev->nvlw_minion_base))
		iounmap(ndev->nvlw_minion_base);

	if (!IS_ERR(ndev->link.nvlw_nvl_base))
		iounmap(ndev->link.nvlw_nvl_base);

	if (!IS_ERR(((struct tegra_nvlink_device *)(ndev->priv))->
							nvlw_sync2x_base))
		iounmap(((struct tegra_nvlink_device *)(ndev->priv))->
							nvlw_sync2x_base);

	if (!IS_ERR(ndev->link.nvlw_nvltlc_base))
		iounmap(ndev->link.nvlw_nvltlc_base);

	if (!IS_ERR(tegra_link->mssnvlink_0_base))
		iounmap(tegra_link->mssnvlink_0_base);

err_clk_rst:
	tegra_nvlink_clk_rst_deinit(ndev);
	kfree(tegra_link);
err_alloc_link_priv:
	kfree(ndev->priv);
err_alloc_priv:
	kfree(ndev);
err_alloc_device:
	nvlink_err("Probe failed!");
err_dt_node:
success:
	return ret;
}

static int t19x_nvlink_endpt_remove(struct platform_device *pdev)
{
	struct nvlink_device *ndev = platform_get_drvdata(pdev);
	struct tegra_nvlink_link *tegra_link =
				(struct tegra_nvlink_link *)(ndev->link.priv);

	t19x_nvlink_endpt_debugfs_deinit(ndev);
	nvlink_unregister_link(&ndev->link);
	nvlink_unregister_device(ndev);
	device_destroy(&ndev->class, ndev->dev_t);
	cdev_del(&ndev->cdev);
	unregister_chrdev_region(ndev->dev_t, 1);
	class_unregister(&ndev->class);

	tegra_nvlink_clk_rst_deinit(ndev);

	iounmap(ndev->nvlw_tioctrl_base);
	iounmap(ndev->nvlw_nvlipt_base);
	iounmap(ndev->nvlw_minion_base);
	iounmap(ndev->link.nvlw_nvl_base);
	iounmap(((struct tegra_nvlink_device *)(ndev->priv))->
							nvlw_sync2x_base);
	iounmap(ndev->link.nvlw_nvltlc_base);
	iounmap(tegra_link->mssnvlink_0_base);
	kfree(tegra_link);
	kfree(ndev->priv);
	kfree(ndev);

	return 0;
}

static struct platform_driver t19x_nvlink_endpt_pdrv = {
	.probe		= t19x_nvlink_endpt_probe,
	.remove		= t19x_nvlink_endpt_remove,
	.driver		= {
		.name	= NVLINK_DRV_NAME,
		.of_match_table = of_match_ptr(t19x_nvlink_controller_of_match),
	},
};

static int __init t19x_nvlink_endpt_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&t19x_nvlink_endpt_pdrv);
	if (ret < 0)
		nvlink_err("Platform driver register failed");

	return ret;
}

static void __exit t19x_nvlink_endpt_exit(void)
{
	nvlink_dbg("Unloading the T19x NVLINK endpoint driver");
	platform_driver_unregister(&t19x_nvlink_endpt_pdrv);
}

module_init(t19x_nvlink_endpt_init);
module_exit(t19x_nvlink_endpt_exit);

MODULE_ALIAS(NVLINK_DRV_NAME);
MODULE_DESCRIPTION("T19x NVLINK Endpoint Driver");
MODULE_AUTHOR("Adeel Raza <araza@nvidia.com>");
MODULE_LICENSE("GPL v2");
