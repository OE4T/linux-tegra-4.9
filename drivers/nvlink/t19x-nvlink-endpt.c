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

static struct of_device_id t19x_nvlink_controller_of_match[] = {
	{
		.compatible     = "nvidia,t19x-nvlink-controller",
	}, {
	},
};

MODULE_DEVICE_TABLE(of, t19x_nvlink_controller_of_match);

static inline u32 nvlw_tioctrl_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(ndev->nvlw_tioctrl_base + reg);
}

static inline void nvlw_tioctrl_writel(struct nvlink_device *ndev, u32 reg,
									u32 val)
{
	writel(val, ndev->nvlw_tioctrl_base + reg);
}

static inline u32 nvlw_nvlipt_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(ndev->nvlw_nvlipt_base + reg);
}

static inline void nvlw_nvlipt_writel(struct nvlink_device *ndev, u32 reg,
									u32 val)
{
	writel(val, ndev->nvlw_nvlipt_base + reg);
}

static inline u32 nvlw_nvl_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(ndev->links[0].nvlw_nvl_base + reg);
}

static inline void nvlw_nvl_writel(struct nvlink_device *ndev, u32 reg,
									u32 val)
{
	writel(val, ndev->links[0].nvlw_nvl_base + reg);
}

static inline u32 nvlw_sync2x_readl(struct nvlink_device *ndev,	u32 reg)
{
	return readl(((struct tegra_nvlink_device *)(ndev->priv))
						->nvlw_sync2x_base + reg);
}

static inline void nvlw_sync2x_writel(struct nvlink_device *ndev, u32 reg,
									u32 val)
{
	writel(val, ((struct tegra_nvlink_device *)(ndev->priv))
						->nvlw_sync2x_base + reg);
}

static inline u32 nvlw_nvltlc_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(ndev->links[0].nvlw_nvltlc_base + reg);
}

static inline void nvlw_nvltlc_writel(struct nvlink_device *ndev, u32 reg,
									u32 val)
{
	writel(val, ndev->links[0].nvlw_nvltlc_base + reg);
}

static inline u32 mssnvlink_0_readl(struct nvlink_device *ndev, u32 reg)
{
	return readl(((struct tegra_nvlink_link *)(ndev->links[0].priv))
						->mssnvlink_0_base + reg);
}

static inline void mssnvlink_0_writel(struct nvlink_device *ndev, u32 reg,
									u32 val)
{
	writel(val, ((struct tegra_nvlink_link *)(ndev->links[0].priv))
						->mssnvlink_0_base + reg);
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
static int wait_for_reg_cond_non_nvlink(u32 reg, u32 bit, int bit_set,
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
static void init_sysclk(struct nvlink_device *ndev)
{
	nvlink_dbg("Initializing NVLINK_SYSCLK");

	non_nvlink_writel(CAR_CLK_OUT_ENB_NVLINK_SYSCLK, 0x1);
	non_nvlink_writel(CAR_CLK_OUT_ENB_NVLINK_SYSCLK, 0x0);

	non_nvlink_writel(CAR_CLOCK_SOURCE_NVLINK_SYSCLK, 0x0);
	non_nvlink_writel(CAR_CLOCK_SOURCE_NVLINK_SYSCLK, 0x40000000);

	non_nvlink_writel(CAR_CLK_OUT_ENB_NVLINK_SYSCLK, 0x1);
	udelay(1);
}

static void release_resets(struct nvlink_device *ndev)
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
}

static void init_nvhs_pll(struct nvlink_device *ndev)
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

static int switch_to_tx_ref_clk(struct nvlink_device *ndev)
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

static void enable_hw_sequencer(struct nvlink_device *ndev)
{
	nvlink_dbg("NVLIPT - enabling hardware sequencer");
	non_nvlink_writel(CAR_NVHS_UPHY_PLL0_CFG0, 0x7493804);
}

static int init_nvhs(struct nvlink_device *ndev)
{
	int ret = 0;

	nvlink_dbg("Initializing NVHS");

	init_nvhs_pll(ndev);

	ret = switch_to_tx_ref_clk(ndev);
	if (ret < 0)
		goto fail;

	enable_hw_sequencer(ndev);

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

static int go_to_safe_mode(struct nvlink_device *ndev)
{
	u32 reg_val = 0;
	u32 state = 0;

	nvlink_dbg("Transitioning to SAFE mode ...");

	nvlw_nvl_writel(ndev, NVL_LINK_CHANGE, 0x14);
	usleep_range(1000, 2000);
	reg_val = nvlw_nvl_readl(ndev, NVL_LINK_STATE);
	state = reg_val & NVL_LINK_STATE_STATE_MASK;
	if (state != NVL_LINK_STATE_STATE_SWCFG) {
		nvlink_err("Failed to transition to SAFE mode");
		return -1;
	}

	nvlink_dbg("Successfully transitioned to SAFE mode");
	return 0;
}

static void init_tlc_buffers(struct nvlink_device *ndev)
{
	nvlink_dbg("Initializing TLC buffers");

	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC0, 0x7f003f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC1, 0x7f005f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC2, 0x7f007f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC3, 0x7f009f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC4, 0x7f00bf);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC5, 0xff007f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC6, 0xff007f);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_SZ_VC7, 0xff007f);

	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC0, 0x800040);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC1, 0x20);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC2, 0x20);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC3, 0x20);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC4, 0x20);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC5, 0x800040);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC6, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC7, 0x0);

	nvlw_nvltlc_writel(ndev, NVLTLC_TX_ERR_LOG_EN_0, 0x3ffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_ERR_REPORT_EN_0, 0x3ffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_TX_ERR_CONTAIN_EN_0, 0x3ffffff);

	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC0, 0xff007f);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC1, 0xff00bf);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC2, 0xff00ff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC3, 0xff013f);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC4, 0xff017f);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC5, 0x1ff01ff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC6, 0x1ff01ff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_SZ_VC7, 0x1ff01ff);

	nvlw_nvltlc_writel(ndev,
			NVLTLC_RX_CTRL_BUFFER_CREDITS_VC0,
			0x1000080);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC1, 0x40);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC2, 0x40);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC3, 0x40);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC4, 0x40);
	nvlw_nvltlc_writel(ndev,
			NVLTLC_RX_CTRL_BUFFER_CREDITS_VC5,
			0x1000080);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC6, 0x0);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC7, 0x0);

	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_LOG_EN_0, 0xffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_REPORT_EN_0, 0xffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_CONTAIN_EN_0, 0xffffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_LOG_EN_1, 0x3fffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_REPORT_EN_1, 0x3fffff);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_ERR_CONTAIN_EN_1, 0x3fffff);

	nvlw_nvltlc_writel(ndev, NVLTLC_TX_CTRL_BUFFER_READY, 0x1);
	nvlw_nvltlc_writel(ndev, NVLTLC_RX_CTRL_BUFFER_READY, 0x1);
}

/* Program the upper limit of the MSSNVLINK address space */
static int program_mssnvlink_tom(struct nvlink_device *ndev)
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
static int program_mssnvlink_hub_credits(struct nvlink_device *ndev)
{
	u32 reg_val = 0;

	nvlink_dbg("Programming MSSNVLINK HUB credits");

	/*
	 * FIXME: Do we need the reads to ensure the value was written
	 * correctly? Or were these reads only useful for debugging?
	 */
	mssnvlink_0_writel(ndev, MSSNVLINK_MASTER_CREDIT_TRANSINFO, 0x14050000);
	reg_val = mssnvlink_0_readl(ndev, MSSNVLINK_MASTER_CREDIT_TRANSINFO);
	reg_val &= 0x7fffffff;
	if (reg_val != 0x14050000) {
		nvlink_err("MSSNVLINK HUB credits programming failed");
		return -1;
	}

	mssnvlink_0_writel(ndev, MSSNVLINK_MASTER_CREDIT_INGR_DATA, 0x8020000);
	reg_val = mssnvlink_0_readl(ndev, MSSNVLINK_MASTER_CREDIT_INGR_DATA);
	reg_val &= 0x7fffffff;
	if (reg_val != 0x8020000) {
		nvlink_err("MSSNVLINK HUB credits programming failed");
		return -1;
	}

	mssnvlink_0_writel(ndev, MSSNVLINK_SLAVE_CREDIT_TRANSINFO, 0x14050000);
	reg_val = mssnvlink_0_readl(ndev, MSSNVLINK_SLAVE_CREDIT_TRANSINFO);
	reg_val &= 0x7fffffff;
	if (reg_val != 0x14050000) {
		nvlink_err("MSSNVLINK HUB credits programming failed");
		return -1;
	}

	mssnvlink_0_writel(ndev, MSSNVLINK_SLAVE_CREDIT_INGR_DATA, 0x300c0000);
	reg_val = mssnvlink_0_readl(ndev, MSSNVLINK_SLAVE_CREDIT_INGR_DATA);
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
int t19x_nvlink_endpt_enable_link(struct nvlink_device *ndev)
{
	int ret = 0;

	nvlink_dbg("Initializing link ...");

	init_sysclk(ndev);
	release_resets(ndev);
	ret = init_nvhs(ndev);
	if (ret < 0)
		goto fail;
	udelay(1);

	init_dlpl(ndev);
	ret = go_to_safe_mode(ndev);
	if (ret < 0)
		goto fail;

	init_tlc_buffers(ndev);
	ret = program_mssnvlink_tom(ndev);
	if (ret < 0)
		goto fail;

	ret = program_mssnvlink_hub_credits(ndev);
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
	struct nvlink_device *ndev = container_of(in->i_cdev,
						struct nvlink_device,
						cdev);

	if (minor > 0) {
		nvlink_err("Incorrect minor number");
		return -EBADFD;
	}

	ret = nvlink_register_endpt_drv(&ndev->links[0]);
	if (ret) {
		nvlink_err("Failed to register with the NVLINK core driver");
		return ret;
	}

	ret = nvlink_init_link(ndev);

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
	struct nvlink_device *ndev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *local_endpoint = NULL;
	struct device_node *remote_parent = NULL;
	const void *compat_prop = NULL;
	struct device *dev = NULL;

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

	ndev->links = (struct nvlink_link *) kzalloc(T19X_MAX_NVLINK_SUPPORTED *
					sizeof(struct nvlink_link), GFP_KERNEL);
	if (!ndev->links) {
		nvlink_err("Couldn't allocate memory for links");
		ret = -ENOMEM;
		goto err_alloc_links;
	}

	ndev->dev = &pdev->dev;
	ndev->class.owner = THIS_MODULE;
	ndev->class.name = NVLINK_DRV_NAME;
	ndev->device_id = NVLINK_ENDPT_T19X;
	ndev->number_of_links = T19X_MAX_NVLINK_SUPPORTED;
	ndev->links[0].link_ops.enable_link = t19x_nvlink_endpt_enable_link;
	platform_set_drvdata(pdev, ndev);

	if (!np) {
		nvlink_err("Invalid device_node");
		ret = -ENODEV;
		goto err_dt_node;
	}


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

	ndev->links[0].nvlw_nvl_base =
				of_io_request_and_map(np, 3,
						"NVLW_NVL aperture");
	if (IS_ERR(ndev->links[0].nvlw_nvl_base)) {
		nvlink_err("Couldn't map the NVLW_NVL aperture");
		ret = PTR_ERR(ndev->links[0].nvlw_nvl_base);
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

	ndev->links[0].nvlw_nvltlc_base =
				of_io_request_and_map(np, 5,
						"NVLW_NVLTLC aperture");
	if (IS_ERR(ndev->links[0].nvlw_nvltlc_base)) {
		nvlink_err("Couldn't map the NVLW_NVLTLC aperture");
		ret = PTR_ERR(ndev->links[0].nvlw_nvltlc_base);
		goto err_mapping;
	}

	((struct tegra_nvlink_link *)(ndev->links[0].priv))->
			mssnvlink_0_base = of_io_request_and_map(np, 6,
							"MSSNVLINK_0 aperture");
	if (IS_ERR(((struct tegra_nvlink_link *)(ndev->links[0].priv))->
							mssnvlink_0_base)) {
		nvlink_err("Couldn't map the MSSNVLINK_0 aperture");
		ret = PTR_ERR(((struct tegra_nvlink_link *)
						(ndev->links[0].priv))->
							mssnvlink_0_base);
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
			ndev->links[0].remote_device_info.device_id =
							NVLINK_ENDPT_T19X;
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

	nvlink_dbg("Probe successful!");
	goto success;

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

	if (!IS_ERR(ndev->links[0].nvlw_nvl_base))
		iounmap(ndev->links[0].nvlw_nvl_base);

	if (!IS_ERR(((struct tegra_nvlink_device *)(ndev->priv))->
							nvlw_sync2x_base))
		iounmap(((struct tegra_nvlink_device *)(ndev->priv))->
							nvlw_sync2x_base);

	if (!IS_ERR(ndev->links[0].nvlw_nvltlc_base))
		iounmap(ndev->links[0].nvlw_nvltlc_base);

	if (!IS_ERR(((struct tegra_nvlink_link *)(ndev->links[0].priv))->
							mssnvlink_0_base))
		iounmap(((struct tegra_nvlink_link *)(ndev->links[0].priv))->
							mssnvlink_0_base);

err_dt_node:
	kfree(ndev->links);
err_alloc_links:
	kfree(ndev->priv);
err_alloc_priv:
	kfree(ndev);
err_alloc_device:
	nvlink_err("Probe failed!");
success:
	return ret;
}

static int t19x_nvlink_endpt_remove(struct platform_device *pdev)
{
	struct nvlink_device *ndev = platform_get_drvdata(pdev);

	device_destroy(&ndev->class, ndev->dev_t);
	cdev_del(&ndev->cdev);
	unregister_chrdev_region(ndev->dev_t, 1);
	class_unregister(&ndev->class);

	iounmap(ndev->nvlw_tioctrl_base);
	iounmap(ndev->nvlw_nvlipt_base);
	iounmap(ndev->nvlw_minion_base);
	iounmap(ndev->links[0].nvlw_nvl_base);
	iounmap(((struct tegra_nvlink_device *)(ndev->priv))->
							nvlw_sync2x_base);
	iounmap(ndev->links[0].nvlw_nvltlc_base);
	iounmap(((struct tegra_nvlink_link *)(ndev->links[0].priv))->
							mssnvlink_0_base);
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
