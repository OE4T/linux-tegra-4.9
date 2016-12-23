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

#include "nvlink.h"
#include "nvlink-hw.h"

#define NVLINK_DRV_NAME	"t19x-nvlink-endpt"

/* Main structure for the driver */
struct t19x_nvlink_endpt {
	/* NVLINK apertures */
	void __iomem *nvlw_tioctrl_base;
	void __iomem *nvlw_nvlipt_base;
	void __iomem *nvlw_minion_base;
	void __iomem *nvlw_nvl_base;
	void __iomem *nvlw_sync2x_base;
	void __iomem *nvlw_nvltlc_base;
	void __iomem *mssnvlink_0_base;
	void __iomem *mssnvlink_1_base;
	void __iomem *mssnvlink_2_base;
	void __iomem *mssnvlink_3_base;
	void __iomem *mssnvlink_4_base;

	/* struct for registering this endpoint driver with the NVLINK core
	   driver */
	struct nvlink_endpt_drv endpt_drv;

	struct class class;
	dev_t dev_t;
	struct cdev cdev;
	struct device *dev;
};

static struct of_device_id t19x_nvlink_controller_of_match[] = {
	{
		.compatible     = "nvidia,t19x-nvlink-controller",
	}, {
	},
};

MODULE_DEVICE_TABLE(of, t19x_nvlink_controller_of_match);

static inline u32 nvlw_tioctrl_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->nvlw_tioctrl_base + reg);
}

static inline void nvlw_tioctrl_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->nvlw_tioctrl_base + reg);
}

static inline u32 nvlw_nvlipt_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->nvlw_nvlipt_base + reg);
}

static inline void nvlw_nvlipt_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->nvlw_nvlipt_base + reg);
}

static inline u32 nvlw_nvl_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->nvlw_nvl_base + reg);
}

static inline void nvlw_nvl_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->nvlw_nvl_base + reg);
}

static inline u32 nvlw_sync2x_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->nvlw_sync2x_base + reg);
}

static inline void nvlw_sync2x_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->nvlw_sync2x_base + reg);
}

static inline u32 nvlw_nvltlc_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->nvlw_nvltlc_base + reg);
}

static inline void nvlw_nvltlc_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->nvlw_nvltlc_base + reg);
}

static inline u32 mssnvlink_0_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->mssnvlink_0_base + reg);
}

static inline void mssnvlink_0_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->mssnvlink_0_base + reg);
}

static inline u32 mssnvlink_1_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->mssnvlink_1_base + reg);
}

static inline void mssnvlink_1_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->mssnvlink_1_base + reg);
}

static inline u32 mssnvlink_2_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->mssnvlink_2_base + reg);
}

static inline void mssnvlink_2_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->mssnvlink_2_base + reg);
}

static inline u32 mssnvlink_3_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->mssnvlink_3_base + reg);
}

static inline void mssnvlink_3_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->mssnvlink_3_base + reg);
}

static inline u32 mssnvlink_4_readl(struct t19x_nvlink_endpt *nvlink,
					u32 reg)
{
	return readl(nvlink->mssnvlink_4_base + reg);
}

static inline void mssnvlink_4_writel(struct t19x_nvlink_endpt *nvlink,
					u32 reg,
					u32 val)
{
	writel(val, nvlink->mssnvlink_4_base + reg);
}

/* TODO: Remove all non-NVLINK reads from the driver. */
static inline u32 non_nvlink_readl(u32 reg)
{
	void __iomem *ptr = ioremap(reg, 0x4);
	u32 val = __raw_readl(ptr);

	iounmap(ptr);
	return val;
}

/* TODO: Remove all non-NVLINK writes from the driver. */
static inline void non_nvlink_writel(u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(reg, 0x4);

	__raw_writel(val, ptr);
	iounmap(ptr);
}

#define DEFAULT_LOOP_SLEEP_US	100
#define DEFAULT_LOOP_TIMEOUT_US	1000000

/*
 * Wait for a bit to be set or cleared in a non-NVLINK register. If the desired
 * bit condition doesn't happen in a certain amount of time, a timeout will
 * happen.
 */
/* TODO: Remove all non-NVLINK register accesses from the driver. */
static int wait_for_reg_cond_non_nvlink(u32 reg,
					u32 bit,
					int bit_set,
					char *bit_name)
{
	u32 elapsed_us = 0;

	do {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US*2);
		elapsed_us += DEFAULT_LOOP_SLEEP_US;

		if ((bit_set && (non_nvlink_readl(reg) & BIT(bit))) ||
			(!bit_set && ((non_nvlink_readl(reg) & BIT(bit)) == 0)))
				break;
	} while (elapsed_us < DEFAULT_LOOP_TIMEOUT_US);
	if (elapsed_us >= DEFAULT_LOOP_TIMEOUT_US) {
		nvlink_err("%s timeout", bit_name);
		return -1;
	}

	return 0;
}

/* Initialize the NVLIPT clock control regsiters */
static void init_sysclk(struct t19x_nvlink_endpt *nvlink)
{
	nvlink_dbg("Initializing NVLINK_SYSCLK");

	non_nvlink_writel(CAR_CLK_OUT_ENB_NVLINK_SYSCLK, 0x1);
	non_nvlink_writel(CAR_CLK_OUT_ENB_NVLINK_SYSCLK, 0x0);

	non_nvlink_writel(CAR_CLOCK_SOURCE_NVLINK_SYSCLK, 0x0);
	non_nvlink_writel(CAR_CLOCK_SOURCE_NVLINK_SYSCLK, 0x40000000);

	non_nvlink_writel(CAR_CLK_OUT_ENB_NVLINK_SYSCLK, 0x1);
	udelay(1);
}

static void release_resets(struct t19x_nvlink_endpt *nvlink)
{
	u32 reg_val = 0;

	nvlink_dbg("Releasing resets");

	/* De-assert the NVHS rail reset */
	non_nvlink_writel(CAR_RST_DEV_NVHS_RAIL_CLR, 0x1);
	/*
	 * FIXME: The NVLINK HW test does a read of a CAR NVHS rail reset
	 *        register to introduce a delay before accessing nvlink core CAR
	 *        registers. But this is a really ambiguous way of adding a
	 *        delay. I am approximating the delay caused by a register read
	 *        to be 100us. But we need a more precise delay value from HW.
	 */
	usleep_range(100, 200);

	non_nvlink_writel(CAR_RST_DEV_NVLINK, 0x0);
	udelay(1);

	/* Take link out of reset */
	reg_val = nvlw_tioctrl_readl(nvlink, NVLW_RESET) |
			BIT(NVLW_RESET_LINKRESET);
	nvlw_tioctrl_writel(nvlink, NVLW_RESET, reg_val);
	udelay(NVLW_POST_RESET_DELAY_US);

	/* Reset persistent HW state for this link */
	reg_val = nvlw_tioctrl_readl(nvlink, NVLW_DEBUG_RESET) &
			~BIT(NVLW_DEBUG_RESET_LINK);
	nvlw_tioctrl_writel(nvlink, NVLW_DEBUG_RESET, reg_val);
	udelay(NVLW_POST_RESET_DELAY_US);
	reg_val = nvlw_tioctrl_readl(nvlink, NVLW_DEBUG_RESET) |
					BIT(NVLW_DEBUG_RESET_LINK) |
					BIT(NVLW_DEBUG_RESET_COMMON);
	nvlw_tioctrl_writel(nvlink, NVLW_DEBUG_RESET, reg_val);
	udelay(NVLW_POST_RESET_DELAY_US);
}

static void init_nvhs_pll(struct t19x_nvlink_endpt *nvlink)
{
	nvlink_dbg("Initializing PLLNVHS");

	non_nvlink_writel(CAR_PLLNVHS_MISC, 0xfa00);
	non_nvlink_writel(CAR_PLLNVHS_BASE, 0x0);
	non_nvlink_writel(CAR_PLLNVHS_BASE1, 0x0);
	non_nvlink_writel(CAR_PLLNVHS_MISC, 0xe000);
	non_nvlink_writel(CAR_PLLNVHS_MISC1, 0x0);
	non_nvlink_writel(CAR_PLLNVHS_MISC, 0xd800);
	udelay(4);

	non_nvlink_writel(CAR_PLLNVHS_BASE, 0x7d01);
	non_nvlink_writel(CAR_PLLNVHS_BASE1, 0x18);
	non_nvlink_writel(CAR_PLLNVHS_SS_CNTL, 0x1c00);
	non_nvlink_writel(CAR_PLLNVHS_SS_CNTL1, 0x0);
	non_nvlink_writel(CAR_PLLNVHS_SS_CNTL2, 0x0);
	non_nvlink_writel(CAR_PLLNVHS_MISC, 0xc200);
	non_nvlink_writel(CAR_PLLNVHS_MISC1, 0x0);
	non_nvlink_writel(CAR_PLLNVHS_BASE, 0x7d01);
	non_nvlink_writel(CAR_PLLNVHS_BASE1, 0x18);
	non_nvlink_writel(CAR_PLLNVHS_BASE, 0x80007d01);
	non_nvlink_writel(CAR_PLLNVHS_BASE1, 0x18);
	non_nvlink_writel(CAR_PLLNVHS_MISC, 0x5a00);

	non_nvlink_writel(CAR_CLK_SOURCE_NVHS_PLL0_MGMT, 0x106);
}

static int switch_to_tx_ref_clk(struct t19x_nvlink_endpt *nvlink)
{
	int ret = 0;

	nvlink_dbg("NVLIPT - Switching to TXREFCLK");

	non_nvlink_writel(CAR_NVLINK_CLK_CTRL, 0x101);

	/* Wait for NVLINK_TXCLK_STS bit to be set */
	ret = wait_for_reg_cond_non_nvlink(CAR_NVLINK_CLK_CTRL,
					CAR_NVLINK_CLK_CTRL_NVLINK_TXCLK_STS,
					1,
					"NVLINK_TXCLK_STS");
	if (ret < 0)
		nvlink_err("NVLIPT - Switching to TXREFCLK failed!");

	return ret;
}

static void enable_hw_sequencer(struct t19x_nvlink_endpt *nvlink)
{
	nvlink_dbg("NVLIPT - enabling hardware sequencer");
	non_nvlink_writel(CAR_NVHS_UPHY_PLL0_CFG0, 0x7493804);
}

static int init_nvhs(struct t19x_nvlink_endpt *nvlink)
{
	int ret = 0;

	nvlink_dbg("Initializing NVHS");

	init_nvhs_pll(nvlink);

	ret = switch_to_tx_ref_clk(nvlink);
	if (ret < 0)
		goto fail;

	enable_hw_sequencer(nvlink);

	goto success;

fail:
	nvlink_err("NVHS init failed!");
success:
	return ret;
}

static void init_dlpl(struct t19x_nvlink_endpt *nvlink)
{
	u32 reg_val = 0;

	nvlink_dbg("Initializing DLPL");

	/* Enable link */
	reg_val = nvlw_nvl_readl(nvlink, NVL_LINK_CONFIG) |
			BIT(NVL_LINK_CONFIG_LINK_EN) | BIT(3);
	nvlw_nvl_writel(nvlink, NVL_LINK_CONFIG, reg_val);

	nvlw_nvl_writel(nvlink, NVL_SL0_TRAIN0_TX, 0x63);
	nvlw_nvl_writel(nvlink, NVL_SL0_TRAIN1_TX, 0xf0);

	nvlw_nvl_writel(nvlink, NVL_SL0_SAFE_CTRL2_TX, 0x2f53);

	nvlw_nvl_writel(nvlink, NVL_SL1_CONFIG_RX, 0x70001000);

	nvlw_nvl_writel(nvlink, NVL_SUBLINK_CHANGE, 0x200000);

	nvlw_nvl_writel(nvlink, NVL_SL1_RXSLSM_TIMEOUT_2, 0xfa0);
}

static int go_to_safe_mode(struct t19x_nvlink_endpt *nvlink)
{
	u32 reg_val = 0;
	u32 state = 0;

	nvlink_dbg("Transitioning to SAFE mode ...");

	nvlw_nvl_writel(nvlink, NVL_LINK_CHANGE, 0x14);
	usleep_range(1000, 2000);
	reg_val = nvlw_nvl_readl(nvlink, NVL_LINK_STATE);
	state = reg_val & NVL_LINK_STATE_STATE_MASK;
	if (state != NVL_LINK_STATE_STATE_SWCFG) {
		nvlink_err("Failed to transition to SAFE mode");
		return -1;
	}

	nvlink_dbg("Successfully transitioned to SAFE mode");
	return 0;
}

static void init_tlc_buffers(struct t19x_nvlink_endpt *nvlink)
{
	nvlink_dbg("Initializing TLC buffers");

	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_SZ_VC0, 0x7f003f);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_SZ_VC1, 0x7f005f);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_SZ_VC2, 0x7f007f);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_SZ_VC3, 0x7f009f);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_SZ_VC4, 0x7f00bf);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_SZ_VC5, 0xff007f);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_SZ_VC6, 0xff007f);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_SZ_VC7, 0xff007f);

	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC0, 0x800040);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC1, 0x20);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC2, 0x20);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC3, 0x20);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC4, 0x20);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC5, 0x800040);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC6, 0x0);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC7, 0x0);

	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_ERR_LOG_EN_0, 0x3ffffff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_ERR_REPORT_EN_0, 0x3ffffff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_ERR_CONTAIN_EN_0, 0x3ffffff);

	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_SZ_VC0, 0xff007f);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_SZ_VC1, 0xff00bf);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_SZ_VC2, 0xff00ff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_SZ_VC3, 0xff013f);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_SZ_VC4, 0xff017f);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_SZ_VC5, 0x1ff01ff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_SZ_VC6, 0x1ff01ff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_SZ_VC7, 0x1ff01ff);

	nvlw_nvltlc_writel(nvlink,
			NVLTLC_RX_CTRL_BUFFER_CREDITS_VC0,
			0x1000080);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC1, 0x40);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC2, 0x40);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC3, 0x40);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC4, 0x40);
	nvlw_nvltlc_writel(nvlink,
			NVLTLC_RX_CTRL_BUFFER_CREDITS_VC5,
			0x1000080);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC6, 0x0);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC7, 0x0);

	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_ERR_LOG_EN_0, 0xffffff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_ERR_REPORT_EN_0, 0xffffff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_ERR_CONTAIN_EN_0, 0xffffff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_ERR_LOG_EN_1, 0x3fffff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_ERR_REPORT_EN_1, 0x3fffff);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_ERR_CONTAIN_EN_1, 0x3fffff);

	nvlw_nvltlc_writel(nvlink, NVLTLC_TX_CTRL_BUFFER_READY, 0x1);
	nvlw_nvltlc_writel(nvlink, NVLTLC_RX_CTRL_BUFFER_READY, 0x1);
}

/* Program the upper limit of the MSSNVLINK address space */
static int program_mssnvlink_tom(struct t19x_nvlink_endpt *nvlink)
{
	/* Program MSSNVLINK TOM to 512 GB */
	nvlink_dbg("Programming MSSNVLINK_TOM to 0x7ffff (i.e 512 GB)");
	mc_writel(0x7ffff, MC_MSSNVLINK_TOM);

	/* FIXME: Do we need this read or was it only useful for debugging? */
	if (mc_readl(MC_MSSNVLINK_TOM) != 0x7ffff) {
		nvlink_err("Failed to configure MSSNVLINK_TOM");
		return -1;
	}

	return 0;
}

/* Write to the MSSNVLINK registers to release header and data credits */
static int program_mssnvlink_hub_credits(struct t19x_nvlink_endpt *nvlink)
{
	u32 reg_val = 0;

	nvlink_dbg("Programming MSSNVLINK HUB credits");

	/*
	 * FIXME: Do we need the reads to ensure the value was written
	 * correctly? Or were these reads only useful for debugging?
	 */
	mssnvlink_0_writel(nvlink,
			MSSNVLINK_MASTER_CREDIT_TRANSINFO,
			0x14050000);
	reg_val = mssnvlink_0_readl(nvlink, MSSNVLINK_MASTER_CREDIT_TRANSINFO);
	reg_val &= 0x7fffffff;
	if (reg_val != 0x14050000) {
		nvlink_err("MSSNVLINK HUB credits programming failed");
		return -1;
	}

	mssnvlink_0_writel(nvlink,
			MSSNVLINK_MASTER_CREDIT_INGR_DATA,
			0x8020000);
	reg_val = mssnvlink_0_readl(nvlink, MSSNVLINK_MASTER_CREDIT_INGR_DATA);
	reg_val &= 0x7fffffff;
	if (reg_val != 0x8020000) {
		nvlink_err("MSSNVLINK HUB credits programming failed");
		return -1;
	}

	mssnvlink_0_writel(nvlink,
			MSSNVLINK_SLAVE_CREDIT_TRANSINFO,
			0x14050000);
	reg_val = mssnvlink_0_readl(nvlink, MSSNVLINK_SLAVE_CREDIT_TRANSINFO);
	reg_val &= 0x7fffffff;
	if (reg_val != 0x14050000) {
		nvlink_err("MSSNVLINK HUB credits programming failed");
		return -1;
	}

	mssnvlink_0_writel(nvlink,
			MSSNVLINK_SLAVE_CREDIT_INGR_DATA,
			0x300c0000);
	reg_val = mssnvlink_0_readl(nvlink, MSSNVLINK_SLAVE_CREDIT_INGR_DATA);
	reg_val &= 0x7fffffff;
	if (reg_val != 0x300c0000) {
		nvlink_err("MSSNVLINK HUB credits programming failed");
		return -1;
	}

	mc_writel(0x8f0, MC_MCF_IREQX_VCARB_CONFIG);
	mc_writel(0x8f0, MC_MCF_OREQX_VCARB_CONFIG);

	return 0;
}

/* Initialize the link and transition to SAFE mode */
int t19x_nvlink_endpt_enable_link(struct nvlink_endpt_drv *drv)
{
	int ret = 0;
	struct t19x_nvlink_endpt *nvlink = container_of(drv,
						struct t19x_nvlink_endpt,
						endpt_drv);

	nvlink_dbg("Initializing link ...");

	init_sysclk(nvlink);
	release_resets(nvlink);
	ret = init_nvhs(nvlink);
	if (ret < 0)
		goto fail;
	udelay(1);

	init_dlpl(nvlink);
	ret = go_to_safe_mode(nvlink);
	if (ret < 0)
		goto fail;

	init_tlc_buffers(nvlink);
	ret = program_mssnvlink_tom(nvlink);
	if (ret < 0)
		goto fail;

	ret = program_mssnvlink_hub_credits(nvlink);
	if (ret < 0)
		goto fail;

	nvlink_dbg("Link initialization succeeded!");
	goto success;

fail:
	nvlink_err("Link initialization failed!");
success:
	return ret;
}

static int t19x_nvlink_endpt_open(struct inode *in, struct file *filp)
{
	int ret = 0;
	unsigned int minor = iminor(in);
	struct t19x_nvlink_endpt *nvlink = container_of(in->i_cdev,
						struct t19x_nvlink_endpt,
						cdev);

	if (minor > 0) {
		nvlink_err("Incorrect minor number");
		return -EBADFD;
	}

	ret = nvlink_register_endpt_drv(&nvlink->endpt_drv);
	if (ret) {
		nvlink_err("Failed to register with the NVLINK core driver");
		return ret;
	}

	ret = nvlink_init_link(&nvlink->endpt_drv);

	return ret;
}

static int t19x_nvlink_endpt_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* File ops for device node */
static const struct file_operations t19x_nvlink_endpt_ops = {
	.owner = THIS_MODULE,
	.open = t19x_nvlink_endpt_open,
	.release = t19x_nvlink_endpt_release,
};

static int t19x_nvlink_endpt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct t19x_nvlink_endpt *nvlink;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *local_endpoint = NULL;
	struct device_node *remote_parent = NULL;
	const void *compat_prop = NULL;
	struct device *dev = NULL;

	nvlink = kzalloc(sizeof(struct t19x_nvlink_endpt), GFP_KERNEL);
	if (!nvlink) {
		nvlink_err("Couldn't allocate struct t19x_nvlink_endpt");
		ret = -ENOMEM;
		goto err_alloc;
	}

	nvlink->dev = &pdev->dev;
	nvlink->class.owner = THIS_MODULE;
	nvlink->class.name = NVLINK_DRV_NAME;
	nvlink->endpt_drv.local_endpt =	NVLINK_ENDPT_TEGRA;
	nvlink->endpt_drv.enable_link =	t19x_nvlink_endpt_enable_link;
	platform_set_drvdata(pdev, nvlink);

	if (!np) {
		nvlink_err("Invalid device_node");
		ret = -ENODEV;
		goto err_dt_node;
	}


	/* Map NVLINK apertures listed in device tree node */
	nvlink->nvlw_tioctrl_base =
				of_io_request_and_map(np, 0,
						"NVLW_TIOCTRL aperture");
	if (IS_ERR(nvlink->nvlw_tioctrl_base)) {
		nvlink_err("Couldn't map the NVLW_TIOCTRL aperture");
		ret = PTR_ERR(nvlink->nvlw_tioctrl_base);
		goto err_mapping;
	}

	nvlink->nvlw_nvlipt_base =
				of_io_request_and_map(np, 1,
						"NVLW_NVLIPT aperture");
	if (IS_ERR(nvlink->nvlw_nvlipt_base)) {
		nvlink_err("Couldn't map the NVLW_NVLIPT aperture");
		ret = PTR_ERR(nvlink->nvlw_nvlipt_base);
		goto err_mapping;
	}

	nvlink->nvlw_minion_base =
				of_io_request_and_map(np, 2,
						"NVLW_MINION aperture");
	if (IS_ERR(nvlink->nvlw_minion_base)) {
		nvlink_err("Couldn't map the NVLW_MINION aperture");
		ret = PTR_ERR(nvlink->nvlw_minion_base);
		goto err_mapping;
	}

	nvlink->nvlw_nvl_base =
				of_io_request_and_map(np, 3,
						"NVLW_NVL aperture");
	if (IS_ERR(nvlink->nvlw_nvl_base)) {
		nvlink_err("Couldn't map the NVLW_NVL aperture");
		ret = PTR_ERR(nvlink->nvlw_nvl_base);
		goto err_mapping;
	}

	nvlink->nvlw_sync2x_base =
				of_io_request_and_map(np, 4,
						"NVLW_SYNC2X aperture");
	if (IS_ERR(nvlink->nvlw_sync2x_base)) {
		nvlink_err("Couldn't map the NVLW_SYNC2X aperture");
		ret = PTR_ERR(nvlink->nvlw_sync2x_base);
		goto err_mapping;
	}

	nvlink->nvlw_nvltlc_base =
				of_io_request_and_map(np, 5,
						"NVLW_NVLTLC aperture");
	if (IS_ERR(nvlink->nvlw_nvltlc_base)) {
		nvlink_err("Couldn't map the NVLW_NVLTLC aperture");
		ret = PTR_ERR(nvlink->nvlw_nvltlc_base);
		goto err_mapping;
	}

	nvlink->mssnvlink_0_base =
				of_io_request_and_map(np, 6,
						"MSSNVLINK_0 aperture");
	if (IS_ERR(nvlink->mssnvlink_0_base)) {
		nvlink_err("Couldn't map the MSSNVLINK_0 aperture");
		ret = PTR_ERR(nvlink->mssnvlink_0_base);
		goto err_mapping;
	}

	nvlink->mssnvlink_1_base =
				of_io_request_and_map(np, 7,
						"MSSNVLINK_1 aperture");
	if (IS_ERR(nvlink->mssnvlink_1_base)) {
		nvlink_err("Couldn't map the MSSNVLINK_1 aperture");
		ret = PTR_ERR(nvlink->mssnvlink_1_base);
		goto err_mapping;
	}

	nvlink->mssnvlink_2_base =
				of_io_request_and_map(np, 8,
						"MSSNVLINK_2 aperture");
	if (IS_ERR(nvlink->mssnvlink_2_base)) {
		nvlink_err("Couldn't map the MSSNVLINK_2 aperture");
		ret = PTR_ERR(nvlink->mssnvlink_2_base);
		goto err_mapping;
	}

	nvlink->mssnvlink_3_base =
				of_io_request_and_map(np, 9,
						"MSSNVLINK_3 aperture");
	if (IS_ERR(nvlink->mssnvlink_3_base)) {
		nvlink_err("Couldn't map the MSSNVLINK_3 aperture");
		ret = PTR_ERR(nvlink->mssnvlink_3_base);
		goto err_mapping;
	}

	nvlink->mssnvlink_4_base =
				of_io_request_and_map(np, 10,
						"MSSNVLINK_4 aperture");
	if (IS_ERR(nvlink->mssnvlink_4_base)) {
		nvlink_err("Couldn't map the MSSNVLINK_4 aperture");
		ret = PTR_ERR(nvlink->mssnvlink_4_base);
		goto err_mapping;
	}


	/* Read NVLINK topology information in device tree */
	local_endpoint = of_graph_get_next_endpoint(np, NULL);
	remote_parent = of_graph_get_remote_port_parent(local_endpoint);
	compat_prop = of_get_property(remote_parent, "compatible", NULL);
	if (compat_prop) {
		if (strcmp(compat_prop,
			t19x_nvlink_controller_of_match[0].compatible) == 0) {
			nvlink_dbg("Loopback topology detected!");
			nvlink->endpt_drv.remote_endpt = NVLINK_ENDPT_TEGRA;
		} else {
			nvlink_err("Invalid topology info in device tree");
			ret = -1;
			goto err_mapping;
		}
	} else {
		nvlink_err("Invalid topology info in device tree");
		ret = -1;
		goto err_mapping;
	}


	/* Create device node */
	ret = class_register(&nvlink->class);
	if (ret) {
		nvlink_err("Failed to register class");
		goto err_mapping;
	}

	ret = alloc_chrdev_region(&nvlink->dev_t, 0, 1, dev_name(nvlink->dev));
	if (ret) {
		nvlink_err("Failed to allocate dev_t");
		goto err_chrdev_region;
	}

	cdev_init(&nvlink->cdev, &t19x_nvlink_endpt_ops);
	nvlink->cdev.owner = THIS_MODULE;

	ret = cdev_add(&nvlink->cdev, nvlink->dev_t, 1);
	if (ret) {
		nvlink_err("Failed to add cdev");
		goto err_cdev;
	}

	dev = device_create(&nvlink->class,
				NULL,
				nvlink->dev_t,
				NULL,
				NVLINK_DRV_NAME);
	if (IS_ERR(dev)) {
		nvlink_err("Failed to create device");
		ret = PTR_ERR(dev);
		goto err_device;
	}

	nvlink_dbg("Probe successful!");
	goto success;

err_device:
	cdev_del(&nvlink->cdev);
err_cdev:
	unregister_chrdev_region(nvlink->dev_t, 1);
err_chrdev_region:
	class_unregister(&nvlink->class);
err_mapping:
	if (!IS_ERR(nvlink->nvlw_tioctrl_base))
		iounmap(nvlink->nvlw_tioctrl_base);

	if (!IS_ERR(nvlink->nvlw_nvlipt_base))
		iounmap(nvlink->nvlw_nvlipt_base);

	if (!IS_ERR(nvlink->nvlw_minion_base))
		iounmap(nvlink->nvlw_minion_base);

	if (!IS_ERR(nvlink->nvlw_nvl_base))
		iounmap(nvlink->nvlw_nvl_base);

	if (!IS_ERR(nvlink->nvlw_sync2x_base))
		iounmap(nvlink->nvlw_sync2x_base);

	if (!IS_ERR(nvlink->nvlw_nvltlc_base))
		iounmap(nvlink->nvlw_nvltlc_base);

	if (!IS_ERR(nvlink->mssnvlink_0_base))
		iounmap(nvlink->mssnvlink_0_base);

	if (!IS_ERR(nvlink->mssnvlink_1_base))
		iounmap(nvlink->mssnvlink_1_base);

	if (!IS_ERR(nvlink->mssnvlink_2_base))
		iounmap(nvlink->mssnvlink_2_base);

	if (!IS_ERR(nvlink->mssnvlink_3_base))
		iounmap(nvlink->mssnvlink_3_base);

	if (!IS_ERR(nvlink->mssnvlink_4_base))
		iounmap(nvlink->mssnvlink_4_base);
err_dt_node:
	kfree(nvlink);
err_alloc:
	nvlink_err("Probe failed!");
success:
	return ret;
}

static int t19x_nvlink_endpt_remove(struct platform_device *pdev)
{
	struct t19x_nvlink_endpt *nvlink = platform_get_drvdata(pdev);

	device_destroy(&nvlink->class, nvlink->dev_t);
	cdev_del(&nvlink->cdev);
	unregister_chrdev_region(nvlink->dev_t, 1);
	class_unregister(&nvlink->class);
	iounmap(nvlink->nvlw_tioctrl_base);
	iounmap(nvlink->nvlw_nvlipt_base);
	iounmap(nvlink->nvlw_minion_base);
	iounmap(nvlink->nvlw_nvl_base);
	iounmap(nvlink->nvlw_sync2x_base);
	iounmap(nvlink->nvlw_nvltlc_base);
	iounmap(nvlink->mssnvlink_0_base);
	iounmap(nvlink->mssnvlink_1_base);
	iounmap(nvlink->mssnvlink_2_base);
	iounmap(nvlink->mssnvlink_3_base);
	iounmap(nvlink->mssnvlink_4_base);
	kfree(nvlink);

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
