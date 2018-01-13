/*
 * Copyright (c) 2017 - 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/resource.h>
#include <soc/tegra/chip-id.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include <linux/random.h>

#include "pcie-designware.h"

#define to_tegra_pcie(x)	container_of(x, struct tegra_pcie_dw, pp)

#define CTRL_0	(0)
#define CTRL_1	(1)
#define CTRL_2	(2)
#define CTRL_3	(3)
#define CTRL_4	(4)
#define CTRL_5	(5)

#define APPL_PINMUX				(0X0)
#define APPL_PINMUX_PEX_RST			BIT(0)
#define APPL_PINMUX_CLKREQ_OUT_OVRD_EN		BIT(9)
#define APPL_PINMUX_CLKREQ_OUT_OVRD		BIT(10)

#define APPL_CTRL				(0X4)
#define APPL_CTRL_LTSSM_EN			BIT(7)
#define APPL_CTRL_SYS_PRE_DET_STATE		BIT(6)

#define APPL_INTR_EN_L0_0			0x8
#define APPL_INTR_EN_L0_0_SYS_MSI_INTR_EN	BIT(31)
#define APPL_INTR_EN_L0_0_SYS_INTR_EN		BIT(30)
#define APPL_INTR_EN_L0_0_CDM_REG_CHK_INT_EN	BIT(19)
#define APPL_INTR_EN_L0_0_AXI_APB_ERR_INT_EN	BIT(17)
#define APPL_INTR_EN_L0_0_CPL_TIMEOUT_INT_EN	BIT(13)
#define APPL_INTR_EN_L0_0_INT_INT_EN		BIT(8)
#define APPL_INTR_EN_L0_0_MSI_RCV_INT_EN	BIT(4)
#define APPL_INTR_EN_L0_0_ERROR_INT_EN		BIT(1)
#define APPL_INTR_EN_L0_0_LINK_STATE_INT_EN	BIT(0)

#define APPL_INTR_STATUS_L0			0xC
#define APPL_INTR_STATUS_L0_CDM_REG_CHK_INT	BIT(18)
#define APPL_INTR_STATUS_L0_INT_INT		BIT(8)
#define APPL_INTR_STATUS_L0_LINK_STATE_INT	BIT(0)

#define APPL_INTR_EN_L1_0_0			0x1C
#define APPL_INTR_EN_L1_0_0_LINK_REQ_RST_NOT_INT_EN	BIT(1)

#define APPL_INTR_STATUS_L1_0_0			0x20
#define APPL_INTR_STATUS_L1_0_0_LINK_REQ_RST_NOT_CHGED	BIT(1)
#define APPL_INTR_STATUS_L1_0_0_SURPRISE_DOWN_ERR_STATE	BIT(4)

#define APPL_INTR_STATUS_L1_1			0x2C
#define APPL_INTR_STATUS_L1_2			0x30
#define APPL_INTR_STATUS_L1_3			0x34
#define APPL_INTR_STATUS_L1_6			0x3C
#define APPL_INTR_STATUS_L1_7			0x40

#define APPL_INTR_EN_L1_8_0			0x44
#define APPL_INTR_EN_L1_8_AER_INT_EN		BIT(15)
#define APPL_INTR_EN_L1_8_INTX_EN		BIT(11)
#define APPL_INTR_EN_L1_8_EDMA_INT_EN		BIT(6)
#define APPL_INTR_EN_L1_8_AUTO_BW_INT_EN		BIT(3)

#define APPL_INTR_STATUS_L1_8_0			0x4C
#define APPL_INTR_STATUS_L1_8_0_EDMA_INT_MASK	0xFC0
#define APPL_INTR_STATUS_L1_8_0_AUTO_BW_INT_STS	BIT(3)

#define APPL_INTR_STATUS_L1_9			0x54
#define APPL_INTR_STATUS_L1_10			0x58
#define APPL_INTR_STATUS_L1_11			0x64
#define APPL_INTR_STATUS_L1_13			0x74
#define APPL_INTR_STATUS_L1_14			0x78
#define APPL_INTR_STATUS_L1_15			0x7C
#define APPL_INTR_STATUS_L1_17			0x88

#define APPL_INTR_EN_L1_18			0x90
#define APPL_INTR_EN_L1_18_CDM_REG_CHK_CMPLT		BIT(2)
#define APPL_INTR_EN_L1_18_CDM_REG_CHK_CMP_ERR		BIT(1)
#define APPL_INTR_EN_L1_18_CDM_REG_CHK_LOGIC_ERR	BIT(0)

#define APPL_INTR_STATUS_L1_18			0x94
#define APPL_INTR_STATUS_L1_18_CDM_REG_CHK_CMPLT	BIT(2)
#define APPL_INTR_STATUS_L1_18_CDM_REG_CHK_CMP_ERR	BIT(1)
#define APPL_INTR_STATUS_L1_18_CDM_REG_CHK_LOGIC_ERR	BIT(0)

#define APPL_LINK_STATUS			0xcc
#define APPL_LINK_STATUS_RDLH_LINK_UP		BIT(0)

#define APPL_APPL_DEBUG				0xD0
#define APPL_APPL_DEBUG_PM_LINKST_IN_L2_LAT	BIT(21)
#define APPL_APPL_DEBUG_SMLH_LTSSM_STATE_MASK	GENMASK(8, 3)
#define APPL_APPL_DEBUG_SMLH_LTSSM_STATE_SHIFT	3

#define APPL_RADM_STATUS			0xE4
#define APPL_PM_XMT_TURNOFF_STATE		BIT(0)

#define APPL_DM_TYPE				0x100
#define APPL_DM_TYPE_RP				0x4

#define APPL_CFG_BASE_ADDR			0x104
#define APPL_CFG_BASE_ADDR_MASK			0xFFFFF000

#define APPL_CFG_IATU_DMA_BASE_ADDR		0x108
#define APPL_CFG_IATU_DMA_BASE_ADDR_MASK	0xFFFC0000

#define APPL_CFG_SLCG_OVERRIDE			0x114
#define APPL_CFG_SLCG_OVERRIDE_SLCG_EN_MASTER	BIT(0)

#define APPL_CAR_RESET_OVRD				0x12C
#define APPL_CAR_RESET_OVRD_CYA_OVERRIDE_CORE_RST_N	BIT(0)

#define APPL_GTH_PHY			0x138
#define APPL_GTH_PHY_RST		0x1

#define PCIE_ATU_REGION_INDEX0	0 /* used for EXT-CFG accesses */
#define PCIE_ATU_REGION_INDEX1	1 /* used for IO accesses */
#define PCIE_ATU_REGION_INDEX2	2 /* used for Non-Prefetchable MEM accesses */
#define PCIE_ATU_REGION_INDEX3	3 /* used for Prefetchable MEM accesses */

#define PCIE_ATU_CR1			0x0
#define PCIE_ATU_TYPE_MEM		(0x0 << 0)
#define PCIE_ATU_TYPE_IO		(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0		(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1		(0x5 << 0)
#define PCIE_ATU_INCREASE_REGION_SIZE	BIT(13)
#define PCIE_ATU_CR2			0x4
#define PCIE_ATU_ENABLE			(0x1 << 31)
#define PCIE_ATU_LOWER_BASE		0x8
#define PCIE_ATU_UPPER_BASE		0xC
#define PCIE_ATU_LIMIT			0x10
#define PCIE_ATU_LOWER_TARGET		0x14
#define PCIE_ATU_UPPER_TARGET		0x18
#define PCIE_ATU_UPPER_LIMIT		0x20

#define PCIE_ATU_BUS(x)			(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)			(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)		(((x) & 0x7) << 16)

#define CFG_LINK_CAP			0x7C
#define CFG_LINK_CAP_MAX_LINK_SPEED_MASK	0xF

#define CFG_LINK_STATUS_CONTROL		0x80
#define CFG_LINK_STATUS_DLL_ACTIVE	BIT(29)
#define CFG_LINK_STATUS_LT		BIT(27)
#define CFG_LINK_CONTROL_LT		BIT(5)

#define CFG_LINK_STATUS_CONTROL_2	0xA0
#define CFG_LINK_STATUS_CONTROL_2_PCIE_CAP_EQ_CPL	BIT(17)
#define CFG_LINK_STATUS_CONTROL_2_TARGET_LS_MASK	0xF

#define CFG_LINK_CAP_L1SUB		0x154

#define CAP_PL16G_STATUS_REG		0x164
#define CAP_PL16G_STATUS_REG_EQ_16G_CPL	BIT(0)

#define CFG_TIMER_CTRL_MAX_FUNC_NUM_OFF	0x718
#define CFG_TIMER_CTRL_ACK_NAK_SHIFT	(19)

#define EVENT_COUNTER_CONTROL_REG	0x168
#define EVENT_COUNTER_ALL_CLEAR		0x3
#define EVENT_COUNTER_ENABLE_ALL	0x7
#define EVENT_COUNTER_ENABLE_SHIFT	2
#define EVENT_COUNTER_EVENT_SEL_MASK	0xFF
#define EVENT_COUNTER_EVENT_SEL_SHIFT	16
#define EVENT_COUNTER_EVENT_Tx_L0S	0x2
#define EVENT_COUNTER_EVENT_Rx_L0S	0x3
#define EVENT_COUNTER_EVENT_L1		0x5
#define EVENT_COUNTER_EVENT_L1_1	0x7
#define EVENT_COUNTER_EVENT_L1_2	0x8
#define EVENT_COUNTER_GROUP_SEL_SHIFT	24
#define EVENT_COUNTER_GROUP_5		0x5

#define EVENT_COUNTER_DATA_REG		0x16C

#define DL_FEATURE_EXCHANGE_EN		BIT(31)

#define PORT_LOGIC_GEN2_CTRL		0x80C
#define PORT_LOGIC_GEN2_CTRL_DIRECT_SPEED_CHANGE	BIT(17)

#define GEN3_EQ_CONTROL_OFF	0x8a8
#define GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_SHIFT	8
#define GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_MASK	GENMASK(23, 8)
#define GEN3_EQ_CONTROL_OFF_FB_MODE_MASK	GENMASK(3, 0)

#define GEN3_RELATED_OFF	0x890
#define GEN3_RELATED_OFF_GEN3_EQ_DISABLE	BIT(16)
#define GEN3_RELATED_OFF_RATE_SHADOW_SEL_SHIFT	24
#define GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK	GENMASK(25, 24)

#define PORT_LOGIC_MSIX_DOORBELL			0x948

#define PORT_LOGIC_PL_CHK_REG_CONTROL_STATUS		0xB20
#define PORT_LOGIC_PL_CHK_REG_CHK_REG_START		BIT(0)
#define PORT_LOGIC_PL_CHK_REG_CHK_REG_CONTINUOUS	BIT(1)
#define PORT_LOGIC_PL_CHK_REG_CHK_REG_COMPARISON_ERROR	BIT(16)
#define PORT_LOGIC_PL_CHK_REG_CHK_REG_LOGIC_ERROR	BIT(17)
#define PORT_LOGIC_PL_CHK_REG_CHK_REG_COMPLETE		BIT(18)

#define PORT_LOGIC_PL_CHK_REG_ERR_ADDR			0xB28

#define CAP_SPCIE_CAP_OFF	0x154
#define CAP_SPCIE_CAP_OFF_DSP_TX_PRESET0_MASK	GENMASK(3, 0)

#define PL16G_CAP_OFF		0x188
#define PL16G_CAP_OFF_DSP_16G_TX_PRESET_MASK	GENMASK(3, 0)

#define AUX_CLK_FREQ			0xB40

#define DMA_RD_CHNL_NUM			2
#define DMA_WR_CHNL_NUM			4

#define LINK_RETRAIN_TIMEOUT HZ

/* DMA Common Registers */
#define DMA_WRITE_ENGINE_EN_OFF		0xC
#define DMA_WRITE_ENGINE_EN_OFF_ENABLE	BIT(0)

#define DMA_WRITE_DOORBELL_OFF		0x10
#define DMA_WRITE_DOORBELL_OFF_WR_STOP	BIT(31)

#define DMA_READ_ENGINE_EN_OFF		0x2C
#define DMA_READ_ENGINE_EN_OFF_ENABLE	BIT(0)

#define DMA_READ_DOORBELL_OFF		0x30
#define DMA_READ_DOORBELL_OFF_RD_STOP	BIT(31)

#define DMA_WRITE_INT_STATUS_OFF	0x4C
#define DMA_WRITE_INT_MASK_OFF		0x54
#define DMA_WRITE_INT_CLEAR_OFF		0x58

#define DMA_WRITE_DONE_IMWR_LOW_OFF	0x60
#define DMA_WRITE_DONE_IMWR_HIGH_OFF	0x64
#define DMA_WRITE_ABORT_IMWR_LOW_OFF	0x68
#define DMA_WRITE_ABORT_IMWR_HIGH_OFF	0x6C

#define DMA_WRITE_IMWR_DATA_OFF_BASE	0x70

#define DMA_READ_INT_STATUS_OFF		0xA0
#define DMA_READ_INT_MASK_OFF		0xA8
#define DMA_READ_INT_CLEAR_OFF		0xAC

#define DMA_READ_DONE_IMWR_LOW_OFF	0xCC
#define DMA_READ_DONE_IMWR_HIGH_OFF	0xD0
#define DMA_READ_ABORT_IMWR_LOW_OFF	0xD4
#define DMA_READ_ABORT_IMWR_HIGH_OFF	0xD8

#define DMA_READ_IMWR_DATA_OFF_BASE	0xDC

/* Channel specific registers */
#define DMA_CH_CONTROL1_OFF_WRCH	0x0
#define DMA_CH_CONTROL1_OFF_WRCH_LLE	BIT(9)
#define DMA_CH_CONTROL1_OFF_WRCH_RIE	BIT(4)
#define DMA_CH_CONTROL1_OFF_WRCH_LIE	BIT(3)
#define DMA_CH_CONTROL1_OFF_WRCH_LLP	BIT(2)
#define DMA_TRANSFER_SIZE_OFF_WRCH	0x8
#define DMA_SAR_LOW_OFF_WRCH		0xC
#define DMA_SAR_HIGH_OFF_WRCH		0x10
#define DMA_DAR_LOW_OFF_WRCH		0x14
#define DMA_DAR_HIGH_OFF_WRCH		0x18
#define DMA_LLP_LOW_OFF_WRCH		0x1C
#define DMA_LLP_HIGH_OFF_WRCH		0x20

#define DMA_CH_CONTROL1_OFF_RDCH	(0x0 + 0x100)
#define DMA_CH_CONTROL1_OFF_RDCH_LLE	BIT(9)
#define DMA_CH_CONTROL1_OFF_RDCH_RIE	BIT(4)
#define DMA_CH_CONTROL1_OFF_RDCH_LIE	BIT(3)
#define DMA_CH_CONTROL1_OFF_RDCH_LLP	BIT(2)
#define DMA_TRANSFER_SIZE_OFF_RDCH	(0x8 + 0x100)
#define DMA_SAR_LOW_OFF_RDCH		(0xC + 0x100)
#define DMA_SAR_HIGH_OFF_RDCH		(0x10 + 0x100)
#define DMA_DAR_LOW_OFF_RDCH		(0x14 + 0x100)
#define DMA_DAR_HIGH_OFF_RDCH		(0x18 + 0x100)
#define DMA_LLP_LOW_OFF_RDCH		(0x1C + 0x100)
#define DMA_LLP_HIGH_OFF_RDCH		(0x20 + 0x100)

#define PME_ACK_TIMEOUT 10000

struct tegra_pcie_dw {
	struct device *dev;
	struct resource	*dbi_res;
	struct resource	*atu_dma_res;
	void __iomem		*appl_base;
	void __iomem		*atu_dma_base;
	struct clk		*core_clk;
	struct reset_control	*core_apb_rst;
	struct reset_control	*core_rst;
	struct pcie_port	pp;

	int			phy_count;	/* DT phy-names count */
	struct phy		**phy;

	struct dentry *debugfs;
	u32 target_speed;
	dma_addr_t dma_addr;
	void *cpu_virt_addr;
	bool disable_clock_request;
	bool power_down_en;
	u8 init_link_width;

	/* DMA operation */
	u64 src;
	u64 dst;
	u32 size;
	u8 channel;
	/* lock for write DMA channel */
	struct mutex wr_lock[DMA_WR_CHNL_NUM];
	/* lock for read DMA channel */
	struct mutex rd_lock[DMA_RD_CHNL_NUM];
	struct completion wr_cpl[DMA_WR_CHNL_NUM];
	struct completion rd_cpl[DMA_RD_CHNL_NUM];
	unsigned long wr_busy;
	unsigned long rd_busy;

	u32 cfg_link_cap_l1sub;
	u32 cap_pl16g_status;
	u32 cap_pl16g_cap_off;
	u32 event_cntr_ctrl;
	u32 event_cntr_data;
	u32 dl_feature_cap;

	u32 num_lanes;
	u32 max_speed;
	bool cdm_check;
	u32 cid;

	struct regulator *pex_ctl_reg;
};

struct dma_tx {
	u64 src;
	u64 dst;
	u32 size;
	u8 channel;
	bool ll;
};

struct dma_ll_element_1 {
	u32 cb:1;
	u32 tcb:1;
	u32 llp:1;
	u32 lie:1;
	u32 rie:1;
};

struct dma_ll {
	struct dma_ll_element_1 ele_1;
	u32 size;
	u32 sar_low;
	u32 sar_high;
	u32 dar_low;
	u32 dar_high;
};

static inline void dma_common_wr16(void __iomem *p, u32 val, u32 offset)
{
	writew(val, 0x20000 + offset + p);
}

static inline u16 dma_common_rd16(void __iomem *p, u32 offset)
{
	return readw(0x20000 + offset + p);
}

static inline void dma_common_wr(void __iomem *p, u32 val, u32 offset)
{
	writel(val, 0x20000 + offset + p);
}

static inline u32 dma_common_rd(void __iomem *p, u32 offset)
{
	return readl(0x20000 + offset + p);
}

static inline void dma_channel_wr(void __iomem *p, u8 channel, u32 val,
				  u32 offset)
{
	writel(val, 0x20000 + (0x200 * (channel + 1)) + offset + p);
}

static inline u32 dma_channel_rd(void __iomem *p, u8 channel, u32 offset)
{
	return readl(0x20000 + (0x200 * (channel + 1)) + offset + p);
}

static void check_apply_link_bad_war(struct pcie_port *pp)
{
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);
	u32 val;

	dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL, 4, &val);
	if ((val >> 16) & PCI_EXP_LNKSTA_LBMS) {
		if (pcie->init_link_width >
		    ((val >> 16) & PCI_EXP_LNKSTA_NLW) >>
		    PCI_EXP_LNKSTA_NLW_SHIFT) {
			dev_warn(pp->dev, "PCIe link is bad, width reduced\n");
			dw_pcie_cfg_read(pcie->pp.dbi_base +
					 CFG_LINK_STATUS_CONTROL_2, 4, &val);
			val &= ~PCI_EXP_LNKSTA_CLS;
			val |= PCI_EXP_LNKSTA_CLS_2_5GB;
			dw_pcie_cfg_write(pcie->pp.dbi_base +
					  CFG_LINK_STATUS_CONTROL_2, 4, val);

			dw_pcie_cfg_read(pcie->pp.dbi_base +
					 CFG_LINK_STATUS_CONTROL, 4, &val);
			val |= CFG_LINK_CONTROL_LT;
			dw_pcie_cfg_write(pcie->pp.dbi_base +
					  CFG_LINK_STATUS_CONTROL, 4, val);
			/* NOTE:- Since this scenario is uncommon and link as
			 * such is not stable anyway, not waiting to confirm
			 * if link is really transiting to Gen-2 speed
			 */
		}
	}
}

static irqreturn_t tegra_pcie_irq_handler(int irq, void *arg)
{
	u32 val = 0, bit = 0, tmp = 0;
	int handled = 0;
	struct pcie_port *pp = (struct pcie_port *)arg;
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);

	handled = 1;

	val = readl(pcie->appl_base + APPL_INTR_STATUS_L0);
	dev_dbg(pp->dev, "APPL_INTR_STATUS_L0 = 0x%08X\n", val);
	if (val & APPL_INTR_STATUS_L0_INT_INT) {
		val = readl(pcie->appl_base + APPL_INTR_STATUS_L1_8_0);
		dev_dbg(pp->dev, "APPL_INTR_STATUS_L1_8_0 = 0x%08X\n", val);
		if (val & APPL_INTR_STATUS_L1_8_0_EDMA_INT_MASK) {
			val = dma_common_rd(pcie->atu_dma_base,
					    DMA_WRITE_INT_STATUS_OFF);
			/* check the status of all busy marked channels */
			for_each_set_bit(bit, &pcie->wr_busy,
					 DMA_WR_CHNL_NUM) {
				if (BIT(bit) & val) {
					dma_common_wr(pcie->atu_dma_base,
						      BIT(bit),
						      DMA_WRITE_INT_CLEAR_OFF);
					/* send completion to channel */
					complete(&pcie->wr_cpl[bit]);
					/* clear status */
					pcie->wr_busy &= ~(BIT(bit));
				}
			}

			val = dma_common_rd(pcie->atu_dma_base,
					    DMA_READ_INT_STATUS_OFF);
			/* check the status of all busy marked channels */
			for_each_set_bit(bit, &pcie->rd_busy,
					 DMA_RD_CHNL_NUM) {
				if (BIT(bit) & val) {
					dma_common_wr(pcie->atu_dma_base,
						      BIT(bit),
						      DMA_READ_INT_CLEAR_OFF);
					/* send completion to channel */
					complete(&pcie->rd_cpl[bit]);
					/* clear status */
					pcie->rd_busy &= ~(BIT(bit));
				}
			}
		}
		if (val & APPL_INTR_STATUS_L1_8_0_AUTO_BW_INT_STS) {
			writel(APPL_INTR_STATUS_L1_8_0_AUTO_BW_INT_STS,
			       pcie->appl_base + APPL_INTR_STATUS_L1_8_0);
			check_apply_link_bad_war(pp);
		}
	} else if (val & APPL_INTR_STATUS_L0_LINK_STATE_INT) {
		val = readl(pcie->appl_base + APPL_INTR_STATUS_L1_0_0);
		dev_dbg(pp->dev, "APPL_INTR_STATUS_L1_0_0 = 0x%08X\n", val);
		if ((val & APPL_INTR_STATUS_L1_0_0_LINK_REQ_RST_NOT_CHGED) &&
		    !(val & APPL_INTR_STATUS_L1_0_0_SURPRISE_DOWN_ERR_STATE)) {
			writel(val, pcie->appl_base + APPL_INTR_STATUS_L1_0_0);

			/* SBR WAR */
			val = readl(pcie->appl_base + APPL_CAR_RESET_OVRD);
			val &= ~APPL_CAR_RESET_OVRD_CYA_OVERRIDE_CORE_RST_N;
			writel(val, pcie->appl_base + APPL_CAR_RESET_OVRD);
			udelay(1);
			val = readl(pcie->appl_base + APPL_CAR_RESET_OVRD);
			val |= APPL_CAR_RESET_OVRD_CYA_OVERRIDE_CORE_RST_N;
			writel(val, pcie->appl_base + APPL_CAR_RESET_OVRD);

			dw_pcie_cfg_read(pp->dbi_base + PORT_LOGIC_GEN2_CTRL, 4,
					 &val);
			val |= PORT_LOGIC_GEN2_CTRL_DIRECT_SPEED_CHANGE;
			dw_pcie_cfg_write(pp->dbi_base + PORT_LOGIC_GEN2_CTRL,
					  4, val);
		}
	}
	val = readl(pcie->appl_base + APPL_INTR_STATUS_L0);
	if (val & APPL_INTR_STATUS_L0_CDM_REG_CHK_INT) {
		val = readl(pcie->appl_base + APPL_INTR_STATUS_L1_18);
		dw_pcie_cfg_read(pp->dbi_base +
				 PORT_LOGIC_PL_CHK_REG_CONTROL_STATUS, 4, &tmp);
		dev_dbg(pp->dev, "APPL_INTR_STATUS_L1_18 = 0x%08X\n", val);
		if (val & APPL_INTR_STATUS_L1_18_CDM_REG_CHK_CMPLT) {
			dev_err(pp->dev, "CDM check complete\n");
			tmp |= PORT_LOGIC_PL_CHK_REG_CHK_REG_COMPLETE;
		}
		if (val & APPL_INTR_STATUS_L1_18_CDM_REG_CHK_CMP_ERR) {
			dev_err(pp->dev, "CDM comparison mismatch\n");
			tmp |= PORT_LOGIC_PL_CHK_REG_CHK_REG_COMPARISON_ERROR;
		}
		if (val & APPL_INTR_STATUS_L1_18_CDM_REG_CHK_LOGIC_ERR) {
			dev_err(pp->dev, "CDM Logic error\n");
			tmp |= PORT_LOGIC_PL_CHK_REG_CHK_REG_LOGIC_ERROR;
		}
		dw_pcie_cfg_write(pp->dbi_base +
				 PORT_LOGIC_PL_CHK_REG_CONTROL_STATUS, 4, tmp);
		dw_pcie_cfg_read(pp->dbi_base +
				 PORT_LOGIC_PL_CHK_REG_ERR_ADDR, 4, &tmp);
		dev_err(pp->dev, "CDM Error Address Offset = 0x%08X\n", tmp);
	}

	return IRQ_RETVAL(handled);
}

static int bpmp_send_uphy_message_atomic(struct mrq_uphy_request *req, int size,
					 struct mrq_uphy_response *reply,
					 int reply_size)
{
	unsigned long flags;
	int err;

	local_irq_save(flags);
	err = tegra_bpmp_send_receive_atomic(MRQ_UPHY, req, size, reply,
					     reply_size);
	local_irq_restore(flags);

	return err;
}

static int bpmp_send_uphy_message(struct mrq_uphy_request *req, int size,
				  struct mrq_uphy_response *reply,
				  int reply_size)
{
	int err;

	err = tegra_bpmp_send_receive(MRQ_UPHY, req, size, reply, reply_size);
	if (err != -EAGAIN)
		return err;

	/*
	 * in case the mail systems worker threads haven't been started yet,
	 * use the atomic send/receive interface. This happens because the
	 * clocks are initialized before the IPC mechanism.
	 */
	return bpmp_send_uphy_message_atomic(req, size, reply, reply_size);
}

static int uphy_bpmp_pcie_controller_state_set(int controller, int enable)
{
	struct mrq_uphy_request req;
	struct mrq_uphy_response resp;

	req.cmd = CMD_UPHY_PCIE_CONTROLLER_STATE;
	req.controller_state.pcie_controller = controller;
	req.controller_state.enable = enable;

	return bpmp_send_uphy_message(&req, sizeof(req), &resp, sizeof(resp));
}

static irqreturn_t tegra_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}

static inline void prog_atu(struct pcie_port *pp, int i, u32 val, u32 reg)
{
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);

	writel(val, pcie->atu_dma_base + (i * 0x200) + reg);
}

static void outbound_atu(struct pcie_port *pp, int i, int type, u64 cpu_addr,
			 u64 pci_addr, u64 size)
{
	prog_atu(pp, i, lower_32_bits(cpu_addr), PCIE_ATU_LOWER_BASE);
	prog_atu(pp, i, upper_32_bits(cpu_addr), PCIE_ATU_UPPER_BASE);
	prog_atu(pp, i, lower_32_bits(cpu_addr + size - 1), PCIE_ATU_LIMIT);
	prog_atu(pp, i, upper_32_bits(cpu_addr + size - 1),
		 PCIE_ATU_UPPER_LIMIT);
	prog_atu(pp, i, lower_32_bits(pci_addr), PCIE_ATU_LOWER_TARGET);
	prog_atu(pp, i, upper_32_bits(pci_addr), PCIE_ATU_UPPER_TARGET);
	prog_atu(pp, i, type | PCIE_ATU_INCREASE_REGION_SIZE, PCIE_ATU_CR1);
	prog_atu(pp, i, PCIE_ATU_ENABLE, PCIE_ATU_CR2);
}

static int tegra_pcie_dw_rd_own_conf(struct pcie_port *pp, int where, int size,
				     u32 *val)
{
	/* This is EP specific register and system hangs when it is
	 * accessed with link being in ASPM-L1 state.
	 * So skip accessing it altogether
	 */
	if (where == PORT_LOGIC_MSIX_DOORBELL) {
		*val = 0x00000000;
		return PCIBIOS_SUCCESSFUL;
	} else {
		return dw_pcie_cfg_read(pp->dbi_base + where, size, val);
	}
}

static int tegra_pcie_dw_rd_other_conf(struct pcie_port *pp,
				       struct pci_bus *bus, unsigned int devfn,
				       int where, int size, u32 *val)
{
	int ret, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr)
		type = PCIE_ATU_TYPE_CFG0;
	else
		type = PCIE_ATU_TYPE_CFG1;

	cpu_addr = pp->cfg1_base;
	cfg_size = pp->cfg1_size;
	va_cfg_base = pp->va_cfg1_base;

	outbound_atu(pp, PCIE_ATU_REGION_INDEX0, type, cpu_addr, busdev,
		     cfg_size);
	ret = dw_pcie_cfg_read(va_cfg_base + where, size, val);
	return ret;
}

static int dma_write(struct tegra_pcie_dw *pcie, struct dma_tx *tx)
{
	u32 val = 0;
	int ret = 0;

	if (tx->channel > 3) {
		dev_err(pcie->dev,
			"Invalid Channel number. Should be with in [0~3]\n");
		return -EINVAL;
	}

	/* acquire lock for channel HW */
	mutex_lock(&pcie->wr_lock[tx->channel]);

	/* program registers */
	/* Enable Write Engine */
	dma_common_wr(pcie->atu_dma_base, DMA_WRITE_ENGINE_EN_OFF_ENABLE,
		      DMA_WRITE_ENGINE_EN_OFF);

	/* Un Mask DONE and ABORT interrupts */
	val = dma_common_rd(pcie->atu_dma_base, DMA_WRITE_INT_MASK_OFF);
	val &= ~(1 << tx->channel);		/* DONE */
	val &= ~(1 << ((tx->channel) + 16));	/* ABORT */
	dma_common_wr(pcie->atu_dma_base, val, DMA_WRITE_INT_MASK_OFF);

	val = dma_channel_rd(pcie->atu_dma_base, tx->channel,
			     DMA_CH_CONTROL1_OFF_WRCH);
	if (tx->ll)
		val = DMA_CH_CONTROL1_OFF_WRCH_LLE;
	else
		val = DMA_CH_CONTROL1_OFF_WRCH_LIE;
	dma_channel_wr(pcie->atu_dma_base, tx->channel, val,
		       DMA_CH_CONTROL1_OFF_WRCH);

	if (tx->ll) {
		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       (tx->src & 0xFFFFFFFF),
			       DMA_LLP_LOW_OFF_WRCH);
		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       ((tx->src >> 32) & 0xFFFFFFFF),
			       DMA_LLP_HIGH_OFF_WRCH);
	} else {
		dma_channel_wr(pcie->atu_dma_base, tx->channel, tx->size,
			       DMA_TRANSFER_SIZE_OFF_WRCH);

		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       (tx->src & 0xFFFFFFFF),
			       DMA_SAR_LOW_OFF_WRCH);
		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       ((tx->src >> 32) & 0xFFFFFFFF),
			       DMA_SAR_HIGH_OFF_WRCH);

		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       (tx->dst & 0xFFFFFFFF),
			       DMA_DAR_LOW_OFF_WRCH);
		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       ((tx->dst >> 32) & 0xFFFFFFFF),
			       DMA_DAR_HIGH_OFF_WRCH);
	}
	/* acquire lock for busy-data and mark it as busy and then release */
	pcie->wr_busy |= 1 << tx->channel;

	/* start DMA (ring the door bell) */
	/* ring the door bell with channel number */
	dma_common_wr(pcie->atu_dma_base, pcie->channel,
		      DMA_WRITE_DOORBELL_OFF);

	/* wait for completion or timeout */
	ret = wait_for_completion_timeout(&pcie->wr_cpl[tx->channel],
					  msecs_to_jiffies(5000));
	if (ret == 0) {
		dev_err(pcie->dev,
			"DMA write operation timed out and no interrupt\n");
		ret = -ETIMEDOUT;
		/* if timeout, clear the mess, sanitize channel & return err */
		dma_common_wr(pcie->atu_dma_base,
			      DMA_WRITE_DOORBELL_OFF_WR_STOP |  pcie->channel,
			      DMA_WRITE_DOORBELL_OFF);
		goto exit;
	}

exit:
	mutex_unlock(&pcie->wr_lock[tx->channel]);
	return ret;
}

static int dma_read(struct tegra_pcie_dw *pcie, struct dma_tx *tx)
{
	u32 val = 0;
	int ret = 0;

	if (tx->channel > 1) {
		dev_err(pcie->dev,
			"Invalid Channel number. Should be with in [0~1]\n");
		return -EINVAL;
	}

	/* acquire lock for channel HW */
	mutex_lock(&pcie->rd_lock[tx->channel]);

	/* program registers */
	/* Enable Read Engine */
	dma_common_wr(pcie->atu_dma_base, DMA_READ_ENGINE_EN_OFF_ENABLE,
		      DMA_READ_ENGINE_EN_OFF);

	/* Un Mask DONE and ABORT interrupts */
	val = dma_common_rd(pcie->atu_dma_base, DMA_READ_INT_MASK_OFF);
	val &= ~(1 << tx->channel);		/* DONE */
	val &= ~(1 << ((tx->channel) + 16));	/* ABORT */
	dma_common_wr(pcie->atu_dma_base, val, DMA_READ_INT_MASK_OFF);

	val = dma_channel_rd(pcie->atu_dma_base, tx->channel,
			     DMA_CH_CONTROL1_OFF_RDCH);
	if (tx->ll)
		val = DMA_CH_CONTROL1_OFF_RDCH_LLE;
	else
		val = DMA_CH_CONTROL1_OFF_RDCH_LIE;
	dma_channel_wr(pcie->atu_dma_base, tx->channel, val,
		       DMA_CH_CONTROL1_OFF_RDCH);

	if (tx->ll) {
		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       (tx->src & 0xFFFFFFFF),
			       DMA_LLP_LOW_OFF_RDCH);
		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       ((tx->src >> 32) & 0xFFFFFFFF),
			       DMA_LLP_HIGH_OFF_RDCH);
	} else {
		dma_channel_wr(pcie->atu_dma_base, tx->channel, tx->size,
			       DMA_TRANSFER_SIZE_OFF_RDCH);

		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       (tx->src & 0xFFFFFFFF),
			       DMA_SAR_LOW_OFF_RDCH);
		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       ((tx->src >> 32) & 0xFFFFFFFF),
			       DMA_SAR_HIGH_OFF_RDCH);

		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       (tx->dst & 0xFFFFFFFF),
			       DMA_DAR_LOW_OFF_RDCH);
		dma_channel_wr(pcie->atu_dma_base, tx->channel,
			       ((tx->dst >> 32) & 0xFFFFFFFF),
			       DMA_DAR_HIGH_OFF_RDCH);
	}

	/* acquire lock for busy-data and mark it as busy and then release */
	pcie->rd_busy |= 1 << tx->channel;

	/* start DMA (ring the door bell) */
	/* ring the door bell with channel number */
	dma_common_wr(pcie->atu_dma_base, pcie->channel,
		      DMA_READ_DOORBELL_OFF);

	/* wait for completion or timeout */
	ret = wait_for_completion_timeout(&pcie->rd_cpl[tx->channel],
					  msecs_to_jiffies(5000));
	if (ret == 0) {
		dev_err(pcie->dev,
			"DMA read operation timed out and no interrupt\n");
		ret = -ETIMEDOUT;
		/* if timeout, clear the mess, sanitize channel & return err */
		dma_common_wr(pcie->atu_dma_base,
			      DMA_READ_DOORBELL_OFF_RD_STOP | pcie->channel,
			      DMA_READ_DOORBELL_OFF);
		goto exit;
	}

exit:
	mutex_unlock(&pcie->rd_lock[tx->channel]);
	return ret;
}

static int write(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw *pcie = (struct tegra_pcie_dw *)(s->private);
	struct dma_tx tx;
	int ret = 0;
	void __iomem *dst_cpu_virt;

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = pcie->src;
	tx.dst = pcie->dst;
	tx.size = pcie->size;
	tx.channel = pcie->channel;

	dst_cpu_virt = ioremap_nocache(pcie->dst, pcie->size);

	/* fill source with random data */
	get_random_bytes(pcie->cpu_virt_addr, pcie->size);

	ret = dma_write(pcie, &tx);
	if (ret < 0) {
		dev_err(pcie->dev, "DMA-Write test FAILED\n");
		ret = -EIO;
		goto err_out;
	}

	/* compare copied data */
	if (!memcmp(pcie->cpu_virt_addr, dst_cpu_virt, pcie->size))
		dev_info(pcie->dev, "DMA-Write test PASSED\n");
	else
		dev_info(pcie->dev, "DMA-Write test FAILED\n");

err_out:
	iounmap(dst_cpu_virt);
	return ret;
}

static int write_ll(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw *pcie = (struct tegra_pcie_dw *)(s->private);
	struct dma_tx tx;
	int ret = 0;
	struct dma_ll *ll;
	void __iomem *dst_cpu_virt;

	dst_cpu_virt = ioremap_nocache(pcie->dst, 6 * 64 * 1024);

	/* create linked list */
	ll = (struct dma_ll *)(pcie->cpu_virt_addr);

	/* leave first 64K for LL element preparation */
	memset((ll + 0), 0x0, sizeof(struct dma_ll));
	(ll + 0)->size = (64 * 1024);
	(ll + 0)->sar_low = pcie->src + (64 * 1024);
	(ll + 0)->dar_low = (pcie->dst + (64 * 1024)) & 0xFFFFFFFF;
	(ll + 0)->dar_high = ((pcie->dst + (64 * 1024)) >> 32) & 0xFFFFFFFF;
	get_random_bytes((u8 *)pcie->cpu_virt_addr + (64 * 1024), 64 * 1024);

	memset((ll + 1), 0x0, sizeof(struct dma_ll));
	(ll + 1)->size = (64 * 1024);
	(ll + 1)->sar_low = pcie->src + (64 * 1024 * 2);
	(ll + 1)->dar_low = (pcie->dst + (64 * 1024 * 2)) & 0xFFFFFFFF;
	(ll + 1)->dar_high = ((pcie->dst + (64 * 1024 * 2)) >> 32) & 0xFFFFFFFF;
	get_random_bytes((u8 *)pcie->cpu_virt_addr + (64 * 1024 * 2),
			 64 * 1024);

	memset((ll + 2), 0x0, sizeof(struct dma_ll));
	(ll + 2)->ele_1.llp = 1;
	(ll + 2)->sar_low = (4 * sizeof(struct dma_ll)) + pcie->src;

	memset((ll + 4), 0x0, sizeof(struct dma_ll));
	(ll + 4)->ele_1.lie = 1;
	(ll + 4)->size = (64 * 1024);
	(ll + 4)->sar_low = pcie->src + (64 * 1024 * 4);
	(ll + 4)->dar_low = (pcie->dst + (64 * 1024 * 4)) & 0xFFFFFFFF;
	(ll + 4)->dar_high = ((pcie->dst + (64 * 1024 * 4)) >> 32) & 0xFFFFFFFF;
	get_random_bytes((u8 *)pcie->cpu_virt_addr + (64 * 1024 * 4),
			 64 * 1024);

	memset((ll + 5), 0x0, sizeof(struct dma_ll));
	(ll + 5)->ele_1.llp = 1;
	(ll + 5)->ele_1.tcb = 1;

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = pcie->src;
	tx.channel = pcie->channel;
	tx.ll = 1;
	ret = dma_write(pcie, &tx);
	if (ret < 0) {
		dev_err(pcie->dev, "DMA-Write-LL FAILED\n");
		ret = -EIO;
		goto err_out;
	}

	/* compare copied data */
	if (memcmp((void *)((u8 *)pcie->cpu_virt_addr + (64 * 1024 * 1)),
		   (u8 *)dst_cpu_virt + (64 * 1024 * 1),  64 * 1024)) {
		dev_err(pcie->dev, "DMA-Write-LL Chunk-1 FAILED\n");
		goto err_out;
	}
	if (memcmp((void *)((u8 *)pcie->cpu_virt_addr + (64 * 1024 * 2)),
		   (u8 *)dst_cpu_virt + (64 * 1024 * 2), 64 * 1024)) {
		dev_err(pcie->dev, "DMA-Write-LL Chunk-2 FAILED\n");
		goto err_out;
	}
	if (memcmp((void *)((u8 *)pcie->cpu_virt_addr + (64 * 1024 * 4)),
		   (u8 *)dst_cpu_virt + (64 * 1024 * 4),  64 * 1024)) {
		dev_err(pcie->dev, "DMA-Write-LL Chunk-3 FAILED\n");
		goto err_out;
	}
	dev_err(pcie->dev, "DMA-Write-LL PASSED\n");

err_out:
	iounmap(dst_cpu_virt);
	return ret;
}

static int read(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw *pcie = (struct tegra_pcie_dw *)(s->private);
	struct dma_tx tx;
	int ret = 0;
	void __iomem *dst_cpu_virt;

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = pcie->src;
	tx.dst = pcie->dst;
	tx.size = pcie->size;
	tx.channel = pcie->channel;

	dst_cpu_virt = ioremap_nocache(pcie->src, pcie->size);
	/* fill source with random data */
	get_random_bytes(dst_cpu_virt, pcie->size);

	ret = dma_read(pcie, &tx);
	if (ret < 0) {
		dev_err(pcie->dev, "DMA-Read test FAILED\n");
		ret = -EIO;
		goto err_out;
	}

	/* compare copied data */
	if (!memcmp(dst_cpu_virt, pcie->cpu_virt_addr, pcie->size))
		dev_info(pcie->dev, "DMA-Read test PASSED\n");
	else
		dev_info(pcie->dev, "DMA-Read test FAILED\n");

err_out:
	iounmap(dst_cpu_virt);
	return ret;
}

static int read_ll(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw *pcie = (struct tegra_pcie_dw *)(s->private);
	struct dma_tx tx;
	int ret = 0;
	struct dma_ll *ll;
	void __iomem *dst_cpu_virt;

	dst_cpu_virt = ioremap_nocache(pcie->src, 6 * 64 * 1024);

	/* create linked list to be sent to ep's local memory */
	ll = (struct dma_ll *)(pcie->cpu_virt_addr);

	/* leave first 64K for LL element preparation */
	memset((ll + 0), 0x0, sizeof(struct dma_ll));
	(ll + 0)->size = (64 * 1024);
	(ll + 0)->sar_low = pcie->src + (64 * 1024 * 1);
	(ll + 0)->sar_high = ((pcie->src + (64 * 1024 * 1)) >> 32) & 0xFFFFFFFF;
	(ll + 0)->dar_low = pcie->dst + (64 * 1024 * 1);
	get_random_bytes((u8 *)dst_cpu_virt + (64 * 1024 * 1), 64 * 1024);

	memset((ll + 1), 0x0, sizeof(struct dma_ll));
	(ll + 1)->size = (64 * 1024);
	(ll + 1)->sar_low = pcie->src + (64 * 1024 * 2);
	(ll + 1)->sar_high = ((pcie->src + (64 * 1024 * 2)) >> 32) & 0xFFFFFFFF;
	(ll + 1)->dar_low = pcie->dst + (64 * 1024 * 2);
	get_random_bytes((u8 *)dst_cpu_virt + (64 * 1024 * 2), 64 * 1024);

	memset((ll + 2), 0x0, sizeof(struct dma_ll));
	(ll + 2)->ele_1.llp = 1;
	(ll + 2)->sar_low = (4 * sizeof(struct dma_ll)) + pcie->dst;

	memset((ll + 4), 0x0, sizeof(struct dma_ll));
	(ll + 4)->ele_1.lie = 1;
	(ll + 4)->size = (64 * 1024);
	(ll + 4)->sar_low = pcie->src + (64 * 1024 * 4);
	(ll + 4)->sar_high = ((pcie->src + (64 * 1024 * 4)) >> 32) & 0xFFFFFFFF;
	(ll + 4)->dar_low = pcie->dst + (64 * 1024 * 4);
	get_random_bytes((u8 *)dst_cpu_virt + (64 * 1024 * 4), 64 * 1024);

	memset((ll + 5), 0x0, sizeof(struct dma_ll));
	(ll + 5)->ele_1.llp = 1;
	(ll + 5)->ele_1.tcb = 1;

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = pcie->dst;
	tx.channel = pcie->channel;
	tx.ll = 1;
	ret = dma_read(pcie, &tx);
	if (ret < 0) {
		dev_err(pcie->dev, "DMA-Read-LL FAILED\n");
		ret = -EIO;
		goto err_out;
	}

	/* compare copied data */
	if (memcmp((void *)((u8 *)pcie->cpu_virt_addr + (64 * 1024 * 1)),
		   (u8 *)dst_cpu_virt + (64 * 1024 * 1), 64 * 1024)) {
		dev_err(pcie->dev, "DMA-Read-LL Chunk-1 FAILED\n");
		goto err_out;
	}
	if (memcmp((void *)((u8 *)pcie->cpu_virt_addr + (64 * 1024 * 2)),
		   (u8 *)dst_cpu_virt + (64 * 1024 * 2), 64 * 1024)) {
		dev_err(pcie->dev, "DMA-Read-LL Chunk-2 FAILED\n");
		goto err_out;
	}
	if (memcmp((void *)((u8 *)pcie->cpu_virt_addr + (64 * 1024 * 4)),
		   (u8 *)dst_cpu_virt + (64 * 1024 * 4), 64 * 1024)) {
		dev_err(pcie->dev, "DMA-Read-LL Chunk-3 FAILED\n");
		goto err_out;
	}
	dev_err(pcie->dev, "DMA-Read-LL PASSED\n");

err_out:
	iounmap(dst_cpu_virt);
	return ret;
}

static int apply_speed_change(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw *pcie = (struct tegra_pcie_dw *)(s->private);
	unsigned long start_jiffies;
	u32 val = 0;

	if (pcie->target_speed > (PCI_EXP_LNKSTA_CLS_8_0GB + 1)) {
		seq_puts(s, "Invalid target speed. Should be 1 ~ 4\n");
		return 0;
	}

	dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL, 4,
			 &val);
	if (((val >> 16) & PCI_EXP_LNKSTA_CLS) == pcie->target_speed) {
		seq_puts(s, "Link speed is already the target speed...!\n");
		return 0;
	}

	if (!tegra_platform_is_fpga() && (pcie->target_speed == 4)) {
		u32 temp1 = 0, temp2 = 0;

		dw_pcie_cfg_read(pcie->pp.dbi_base + pcie->cap_pl16g_status,
				 4, &val);
		dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL,
				 4, &temp1);
		dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL_2,
				 4, &temp2);

		if (!((val & CAP_PL16G_STATUS_REG_EQ_16G_CPL) ||
		      (((temp1 & (PCI_EXP_LNKSTA_CLS << 16)) ==
			 (PCI_EXP_LNKSTA_CLS_8_0GB << 16)) &&
			 temp2 & CFG_LINK_STATUS_CONTROL_2_PCIE_CAP_EQ_CPL))) {
			seq_puts(s, "Gen-3/4 Equalization is not complete\n");
			return 0;
		}
	}

	dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL_2, 4,
			 &val);
	val &= ~PCI_EXP_LNKSTA_CLS;
	val |= pcie->target_speed;
	dw_pcie_cfg_write(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL_2, 4,
			  val);

	dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL, 4,
			 &val);
	val |= CFG_LINK_CONTROL_LT;
	dw_pcie_cfg_write(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL, 4,
			  val);

	/* Wait for link training end. Break out after waiting for timeout */
	start_jiffies = jiffies;
	for (;;) {
		dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL,
				 4, &val);
		if (!(val & CFG_LINK_STATUS_LT))
			break;
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT))
			break;
		usleep_range(1000, 1100);
	}
	if (val & CFG_LINK_STATUS_LT) {
		seq_puts(s, "Link Re-training failed after speed change\n");
	} else {
		dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL,
				 4, &val);
		if (((val >> 16) & PCI_EXP_LNKSTA_CLS) == pcie->target_speed) {
			seq_puts(s, "Link speed is successful...!\n");
		} else {
			seq_puts(s, "Link speed change failed...");
			seq_printf(s, "Settled for Gen-%u\n", (val >> 16) &
				   PCI_EXP_LNKSTA_CLS);
		}
	}

	return 0;
}

static int apply_pme_turnoff(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw *pcie = (struct tegra_pcie_dw *)(s->private);
	u32 val = 0;

	val = readl(pcie->appl_base + APPL_RADM_STATUS);
	val |= APPL_PM_XMT_TURNOFF_STATE;
	writel(val, pcie->appl_base + APPL_RADM_STATUS);

	mdelay(1000);

	val = readl(pcie->appl_base + APPL_APPL_DEBUG);
	if (val & APPL_APPL_DEBUG_PM_LINKST_IN_L2_LAT)
		seq_puts(s, "PME_TurnOff sent and Link is in L2 state\n");
	else
		seq_puts(s, "PME_TurnOff failed\n");

	return 0;
}

static int apply_sbr(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw *pcie = (struct tegra_pcie_dw *)(s->private);
	u32 val = 0;

	dw_pcie_cfg_read(pcie->pp.dbi_base + PCI_BRIDGE_CONTROL, 2, &val);
	val |= PCI_BRIDGE_CTL_BUS_RESET;
	dw_pcie_cfg_write(pcie->pp.dbi_base + PCI_BRIDGE_CONTROL, 2, val);
	mdelay(1);
	dw_pcie_cfg_read(pcie->pp.dbi_base + PCI_BRIDGE_CONTROL, 2, &val);
	val &= ~PCI_BRIDGE_CTL_BUS_RESET;
	dw_pcie_cfg_write(pcie->pp.dbi_base + PCI_BRIDGE_CONTROL, 2, val);

	seq_puts(s, "Secondary Bus Reset applied successfully...\n");

	return 0;
}

static inline u32 event_counter_prog(struct tegra_pcie_dw *pcie, u32 event)
{
	u32 val = 0;

	dw_pcie_cfg_read(pcie->pp.dbi_base + pcie->event_cntr_ctrl, 4, &val);
	val &= ~(EVENT_COUNTER_EVENT_SEL_MASK << EVENT_COUNTER_EVENT_SEL_SHIFT);
	val |= EVENT_COUNTER_GROUP_5 << EVENT_COUNTER_GROUP_SEL_SHIFT;
	val |= event << EVENT_COUNTER_EVENT_SEL_SHIFT;
	val |= EVENT_COUNTER_ENABLE_ALL << EVENT_COUNTER_ENABLE_SHIFT;
	dw_pcie_cfg_write(pcie->pp.dbi_base + pcie->event_cntr_ctrl, 4, val);
	dw_pcie_cfg_read(pcie->pp.dbi_base + pcie->event_cntr_data, 4, &val);
	return val;
}

static int aspm_state_cnt(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw *pcie = (struct tegra_pcie_dw *)(s->private);
	u32 val = 0;

	seq_printf(s, "Tx L0s entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_Tx_L0S));

	seq_printf(s, "Rx L0s entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_Rx_L0S));

	seq_printf(s, "Link L1 entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_L1));

	seq_printf(s, "Link L1.1 entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_L1_1));

	seq_printf(s, "Link L1.2 entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_L1_2));

	/* Clear all counters */
	dw_pcie_cfg_write(pcie->pp.dbi_base + pcie->event_cntr_ctrl, 4,
			  EVENT_COUNTER_ALL_CLEAR);

	/* Re-enable counting */
	val = EVENT_COUNTER_ENABLE_ALL << EVENT_COUNTER_ENABLE_SHIFT;
	val |= EVENT_COUNTER_GROUP_5 << EVENT_COUNTER_GROUP_SEL_SHIFT;
	dw_pcie_cfg_write(pcie->pp.dbi_base + pcie->event_cntr_ctrl, 4, val);

	return 0;
}

#define DEFINE_ENTRY(__name)	\
static int __name ## _open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, __name, inode->i_private); \
}									\
static const struct file_operations __name ## _fops = {	\
	.open		= __name ## _open,	\
	.read		= seq_read,	\
	.llseek		= seq_lseek,	\
	.release	= single_release,	\
}

/* common */
DEFINE_ENTRY(write);
DEFINE_ENTRY(write_ll);
DEFINE_ENTRY(read);
DEFINE_ENTRY(read_ll);
DEFINE_ENTRY(apply_speed_change);
DEFINE_ENTRY(apply_pme_turnoff);
DEFINE_ENTRY(apply_sbr);
DEFINE_ENTRY(aspm_state_cnt);

static int init_debugfs(struct tegra_pcie_dw *pcie)
{
	struct dentry *d;

	/* alloc memory required for RP-DMA testing */
	pcie->cpu_virt_addr = dma_alloc_coherent(pcie->dev, 0x100000,
					    &pcie->dma_addr, GFP_KERNEL);
	if (!pcie->cpu_virt_addr) {
		dev_err(pcie->dev,
			"Memory allocation for DMA failed...! exiting...!");
		return -ENOMEM;
	}
	dev_info(pcie->dev,
		 "---> Allocated memory for DMA @ 0x%llX\n", pcie->dma_addr);

	d = debugfs_create_x64("src", 0644, pcie->debugfs, &pcie->src);
	if (!d)
		dev_err(pcie->dev, "debugfs for src addr failed\n");

	d = debugfs_create_x64("dst", 0644, pcie->debugfs, &pcie->dst);
	if (!d)
		dev_err(pcie->dev, "debugfs for dst addr failed\n");

	d = debugfs_create_x32("size", 0644, pcie->debugfs, &pcie->size);
	if (!d)
		dev_err(pcie->dev, "debugfs for size failed\n");

	d = debugfs_create_x8("channel", 0644, pcie->debugfs, &pcie->channel);
	if (!d)
		dev_err(pcie->dev, "debugfs for channel failed\n");

	d = debugfs_create_file("write", 0444, pcie->debugfs, (void *)pcie,
				&write_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for write failed\n");

	d = debugfs_create_file("write_ll", 0444, pcie->debugfs, (void *)pcie,
				&write_ll_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for write failed\n");

	d = debugfs_create_file("read", 0444, pcie->debugfs, (void *)pcie,
				&read_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for read failed\n");

	d = debugfs_create_file("read_ll", 0444, pcie->debugfs, (void *)pcie,
				&read_ll_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for read failed\n");

	d = debugfs_create_u32("target_speed", 0644, pcie->debugfs,
			       &pcie->target_speed);
	if (!d)
		dev_err(pcie->dev, "debugfs for target_speed failed\n");

	d = debugfs_create_file("apply_speed_change", 0444, pcie->debugfs,
				(void *)pcie, &apply_speed_change_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for apply_speed_change failed\n");

	d = debugfs_create_file("apply_pme_turnoff", 0444, pcie->debugfs,
				(void *)pcie, &apply_pme_turnoff_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for apply_pme_turnoff failed\n");

	d = debugfs_create_file("apply_sbr", 0444, pcie->debugfs,
				(void *)pcie, &apply_sbr_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for apply_sbr failed\n");

	d = debugfs_create_file("aspm_state_cnt", 0444, pcie->debugfs,
				(void *)pcie, &aspm_state_cnt_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for aspm_state_cnt failed\n");

	return 0;
}

static int tegra_pcie_dw_wr_own_conf(struct pcie_port *pp, int where, int size,
				     u32 val)
{
	/* This is EP specific register and system hangs when it is
	 * accessed with link being in ASPM-L1 state.
	 * So skip accessing it altogether
	 */
	if (where == PORT_LOGIC_MSIX_DOORBELL)
		return PCIBIOS_SUCCESSFUL;
	else
		return dw_pcie_cfg_write(pp->dbi_base + where, size, val);
}

static int tegra_pcie_dw_wr_other_conf(struct pcie_port *pp,
				       struct pci_bus *bus, unsigned int devfn,
				       int where, int size, u32 val)
{
	int ret, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr)
		type = PCIE_ATU_TYPE_CFG0;
	else
		type = PCIE_ATU_TYPE_CFG1;

	cpu_addr = pp->cfg1_base;
	cfg_size = pp->cfg1_size;
	va_cfg_base = pp->va_cfg1_base;

	outbound_atu(pp, PCIE_ATU_REGION_INDEX0, type, cpu_addr, busdev,
		     cfg_size);
	ret = dw_pcie_cfg_write(va_cfg_base + where, size, val);

	return ret;
}

static void tegra_pcie_enable_system_interrupts(struct pcie_port *pp)
{
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);
	u32 val;

	val = readl(pcie->appl_base + APPL_INTR_EN_L0_0);
	val |= APPL_INTR_EN_L0_0_LINK_STATE_INT_EN;
	writel(val, pcie->appl_base + APPL_INTR_EN_L0_0);

	val = readl(pcie->appl_base + APPL_INTR_EN_L1_0_0);
	val |= APPL_INTR_EN_L1_0_0_LINK_REQ_RST_NOT_INT_EN;
	writel(val, pcie->appl_base + APPL_INTR_EN_L1_0_0);

	if (pcie->cdm_check) {
		val = readl(pcie->appl_base + APPL_INTR_EN_L0_0);
		val |= APPL_INTR_EN_L0_0_CDM_REG_CHK_INT_EN;
		writel(val, pcie->appl_base + APPL_INTR_EN_L0_0);

		val = readl(pcie->appl_base + APPL_INTR_EN_L1_18);
		val |= APPL_INTR_EN_L1_18_CDM_REG_CHK_CMP_ERR;
		val |= APPL_INTR_EN_L1_18_CDM_REG_CHK_LOGIC_ERR;
		writel(val, pcie->appl_base + APPL_INTR_EN_L1_18);
	}

	dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL, 4, &val);
	pcie->init_link_width = ((val >> 16) & PCI_EXP_LNKSTA_NLW) >>
				PCI_EXP_LNKSTA_NLW_SHIFT;
	val |= PCI_EXP_LNKCTL_LBMIE;
	dw_pcie_cfg_write(pcie->pp.dbi_base + CFG_LINK_STATUS_CONTROL, 2, val);
}

static void tegra_pcie_enable_legacy_interrupts(struct pcie_port *pp)
{
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);
	u32 val;

	/* enable legacy interrupt generation */
	val = readl(pcie->appl_base + APPL_INTR_EN_L0_0);
	val |= APPL_INTR_EN_L0_0_SYS_INTR_EN;
	val |= APPL_INTR_EN_L0_0_INT_INT_EN;
	writel(val, pcie->appl_base + APPL_INTR_EN_L0_0);

	val = readl(pcie->appl_base + APPL_INTR_EN_L1_8_0);
	val |= APPL_INTR_EN_L1_8_INTX_EN;
	val |= APPL_INTR_EN_L1_8_AUTO_BW_INT_EN;
	if (IS_ENABLED(CONFIG_PCIEAER))
		val |= APPL_INTR_EN_L1_8_AER_INT_EN;
	writel(val, pcie->appl_base + APPL_INTR_EN_L1_8_0);

	/* Enable Interrupt for DMA completion */
	val = readl(pcie->appl_base + APPL_INTR_EN_L1_8_0);
	val |= APPL_INTR_EN_L1_8_EDMA_INT_EN;
	writel(val, pcie->appl_base + APPL_INTR_EN_L1_8_0);
}

static void tegra_pcie_enable_msi_interrupts(struct pcie_port *pp)
{
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);
	u32 val;

	dw_pcie_msi_init(pp);

	/* enable MSI interrupt generation */
	val = readl(pcie->appl_base + APPL_INTR_EN_L0_0);
	val |= APPL_INTR_EN_L0_0_SYS_MSI_INTR_EN;
	val |= APPL_INTR_EN_L0_0_MSI_RCV_INT_EN;
	writel(val, pcie->appl_base + APPL_INTR_EN_L0_0);
}

static void tegra_pcie_enable_interrupts(struct pcie_port *pp)
{
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);

	/* Clear interrupt statuses before enabling interrupts */
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L0);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_0_0);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_1);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_2);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_3);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_6);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_7);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_8_0);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_9);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_10);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_11);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_13);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_14);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_15);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_17);

	tegra_pcie_enable_system_interrupts(pp);
	tegra_pcie_enable_legacy_interrupts(pp);
	if (IS_ENABLED(CONFIG_PCI_MSI))
		tegra_pcie_enable_msi_interrupts(pp);
}

static void disable_aspm_l0s(struct tegra_pcie_dw *pcie)
{
	u32 val = 0;

	dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_CAP, 4, &val);
	val &= ~(PCI_EXP_LNKCTL_ASPM_L0S << 10);
	dw_pcie_cfg_write(pcie->pp.dbi_base + CFG_LINK_CAP, 4, val);
}

static void disable_aspm_l10(struct tegra_pcie_dw *pcie)
{
	u32 val = 0;

	dw_pcie_cfg_read(pcie->pp.dbi_base + CFG_LINK_CAP, 4, &val);
	val &= ~(PCI_EXP_LNKCTL_ASPM_L1 << 10);
	dw_pcie_cfg_write(pcie->pp.dbi_base + CFG_LINK_CAP, 4, val);
}

static void disable_aspm_l11(struct tegra_pcie_dw *pcie)
{
	u32 val = 0;

	dw_pcie_cfg_read(pcie->pp.dbi_base + pcie->cfg_link_cap_l1sub,
			 4, &val);
	val &= ~PCI_L1SS_CAP_ASPM_L11S;
	dw_pcie_cfg_write(pcie->pp.dbi_base + pcie->cfg_link_cap_l1sub,
			  4, val);
}

static void disable_aspm_l12(struct tegra_pcie_dw *pcie)
{
	u32 val = 0;

	dw_pcie_cfg_read(pcie->pp.dbi_base + pcie->cfg_link_cap_l1sub,
			 4, &val);
	val &= ~PCI_L1SS_CAP_ASPM_L12S;
	dw_pcie_cfg_write(pcie->pp.dbi_base + pcie->cfg_link_cap_l1sub,
			  4, val);
}

static void program_gen3_gen4_eq_presets(struct pcie_port *pp)
{
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);
	int i, init_preset = 5;
	u32 val;

	/* program init preset */
	if (init_preset < 11) {
		for (i = 0; i < pcie->num_lanes; i++) {
			dw_pcie_cfg_read(pp->dbi_base + CAP_SPCIE_CAP_OFF
					 + (i * 2), 2, &val);
			val &= ~CAP_SPCIE_CAP_OFF_DSP_TX_PRESET0_MASK;
			val |= init_preset;
			dw_pcie_cfg_write(pp->dbi_base + CAP_SPCIE_CAP_OFF
					 + (i* 2), 2, val);

			dw_pcie_cfg_read(pp->dbi_base + pcie->cap_pl16g_cap_off
					 + i, 1, &val);
			val &= ~PL16G_CAP_OFF_DSP_16G_TX_PRESET_MASK;
			val |= init_preset;
			dw_pcie_cfg_write(pp->dbi_base + pcie->cap_pl16g_cap_off
					 + i , 1, val);
		}
	}

	dw_pcie_cfg_read(pp->dbi_base + GEN3_RELATED_OFF, 4, &val);
	val &= ~GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK;
	dw_pcie_cfg_write(pp->dbi_base + GEN3_RELATED_OFF, 4, val);

	dw_pcie_cfg_read(pp->dbi_base + GEN3_EQ_CONTROL_OFF, 4, &val);
	val &= ~GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_MASK;
	val |= (0x3ff << GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_SHIFT);
	val &= ~GEN3_EQ_CONTROL_OFF_FB_MODE_MASK;
	dw_pcie_cfg_write(pp->dbi_base + GEN3_EQ_CONTROL_OFF, 4, val);

	dw_pcie_cfg_read(pp->dbi_base + GEN3_RELATED_OFF, 4, &val);
	val &= ~GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK;
	val |= (0x1 << GEN3_RELATED_OFF_RATE_SHADOW_SEL_SHIFT);
	dw_pcie_cfg_write(pp->dbi_base + GEN3_RELATED_OFF, 4, val);

	dw_pcie_cfg_read(pp->dbi_base + GEN3_EQ_CONTROL_OFF, 4, &val);
	val &= ~GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_MASK;
	val |= (0x270 << GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_SHIFT);
	val &= ~GEN3_EQ_CONTROL_OFF_FB_MODE_MASK;
	val |= 0x1;
	dw_pcie_cfg_write(pp->dbi_base + GEN3_EQ_CONTROL_OFF, 4, val);

	dw_pcie_cfg_read(pp->dbi_base + GEN3_RELATED_OFF, 4, &val);
	val &= ~GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK;
	dw_pcie_cfg_write(pp->dbi_base + GEN3_RELATED_OFF, 4, val);
}

static void tegra_pcie_dw_host_init(struct pcie_port *pp)
{
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);
	struct device_node *np = pp->dev->of_node;
	struct resource_entry *win;
	u32 val, tmp;
	int err, count = 100;

	/* Get the I/O and memory ranges from DT */
	resource_list_for_each_entry(win, &pp->res) {
		if (win->res->flags & IORESOURCE_IO) {
			/* program iATU for IO mapping */
			outbound_atu(pp, PCIE_ATU_REGION_INDEX1,
				     PCIE_ATU_TYPE_IO, pp->io_base,
				     win->res->start - win->offset,
				     resource_size(win->res));
		} else if (win->res->flags & IORESOURCE_PREFETCH) {
			/* program iATU for Non-prefetchable MEM mapping */
			outbound_atu(pp, PCIE_ATU_REGION_INDEX3,
				     PCIE_ATU_TYPE_MEM, win->res->start,
				     win->res->start, resource_size(win->res));
		} else if (win->res->flags & IORESOURCE_MEM) {
			/* program iATU for Non-prefetchable MEM mapping */
			outbound_atu(pp, PCIE_ATU_REGION_INDEX2,
				     PCIE_ATU_TYPE_MEM, win->res->start,
				     win->res->start, resource_size(win->res));
		}
	}

	dw_pcie_setup_rc(pp);

	if (tegra_platform_is_fpga()) {
		/* Program correct VID and DID on FPGA */
		dw_pcie_cfg_write(pp->dbi_base + PCI_VENDOR_ID, 2, 0x10DE);
		dw_pcie_cfg_write(pp->dbi_base + PCI_DEVICE_ID, 2, 0x1AD1);

		/* Required for L1.1 working on FPGA */
		val = readl(pcie->appl_base + APPL_GTH_PHY);
		val &= 0xFFFF0003;
		val &= ~(0x2);
		val |= 0x7F4;
		writel(val, pcie->appl_base + APPL_GTH_PHY);

		/* Program correct L0s and L1 exit latencies */
		dw_pcie_cfg_read(pp->dbi_base + CFG_LINK_CAP, 4, &tmp);
		tmp &= ~PCI_EXP_LNKCAP_L0SEL;
		tmp |= 0x4; /* 512 ns to less than 1us */
		tmp &= ~PCI_EXP_LNKCAP_L1EL;
		tmp |= 0x6; /* 32us to 64us */
		dw_pcie_cfg_write(pp->dbi_base + CFG_LINK_CAP, 4, tmp);

		dw_pcie_cfg_read(pp->dbi_base + AUX_CLK_FREQ, 4, &tmp);
		tmp &= ~(0x3FF);
		tmp |= 0x6;
		dw_pcie_cfg_write(pp->dbi_base + AUX_CLK_FREQ, 4, tmp);
	} else {
		dw_pcie_cfg_read(pp->dbi_base + AUX_CLK_FREQ, 4, &tmp);
		tmp &= ~(0x3FF);
		/* CHECK: Confirm this value for Silicon */
		tmp |= 19;
		dw_pcie_cfg_write(pp->dbi_base + AUX_CLK_FREQ, 4, tmp);
	}

	/* Configure Max Speed from DT */
	dw_pcie_cfg_read(pp->dbi_base + CFG_LINK_CAP, 4, &tmp);
	tmp &= ~CFG_LINK_CAP_MAX_LINK_SPEED_MASK;
	tmp |= pcie->max_speed;
	dw_pcie_cfg_write(pp->dbi_base + CFG_LINK_CAP, 4, tmp);
	dw_pcie_cfg_read(pp->dbi_base + CFG_LINK_STATUS_CONTROL_2, 4, &tmp);
	tmp &= ~CFG_LINK_STATUS_CONTROL_2_TARGET_LS_MASK;
	tmp |= pcie->max_speed;
	dw_pcie_cfg_write(pp->dbi_base + CFG_LINK_STATUS_CONTROL_2, 4, tmp);

	program_gen3_gen4_eq_presets(pp);

	/* Program what ASPM states sould get advertised */
	err = of_property_read_u32(np, "nvidia,disable-aspm-states", &val);
	if (!err) {
		if (val & 0x1)
			disable_aspm_l0s(pcie); /* Disable L0s */
		if (val & 0x2) {
			disable_aspm_l10(pcie); /* Disable L1 */
			disable_aspm_l11(pcie); /* Disable L1.1 */
			disable_aspm_l12(pcie); /* Disable L1.2 */
		}
		if (val & 0x4)
			disable_aspm_l11(pcie); /* Disable L1.1 */
		if (val & 0x8)
			disable_aspm_l12(pcie); /* Disable L1.2 */
	}

	if (of_property_read_bool(np, "nvidia,update_fc_fixup")) {
		dw_pcie_cfg_read(pp->dbi_base +
				 CFG_TIMER_CTRL_MAX_FUNC_NUM_OFF, 4, &tmp);
		tmp |= 0x1 << CFG_TIMER_CTRL_ACK_NAK_SHIFT;
		dw_pcie_cfg_write(pp->dbi_base +
				  CFG_TIMER_CTRL_MAX_FUNC_NUM_OFF, 4, tmp);
	}

	/* CDM check enable */
	pcie->cdm_check = of_property_read_bool(np, "nvidia,cdm_check");
	if (pcie->cdm_check) {
		dw_pcie_cfg_read(pp->dbi_base +
				 PORT_LOGIC_PL_CHK_REG_CONTROL_STATUS, 4, &tmp);
		tmp |= PORT_LOGIC_PL_CHK_REG_CHK_REG_CONTINUOUS;
		tmp |= PORT_LOGIC_PL_CHK_REG_CHK_REG_START;
		dw_pcie_cfg_write(pp->dbi_base +
				  PORT_LOGIC_PL_CHK_REG_CONTROL_STATUS, 4, tmp);
	}

	/* FPGA specific PHY initialization */
	if (tegra_platform_is_fpga()) {
		val = readl(pcie->appl_base + APPL_GTH_PHY);
		val |= APPL_GTH_PHY_RST;
		writel(val, pcie->appl_base + APPL_GTH_PHY);
	}

	/* assert RST */
	val = readl(pcie->appl_base + APPL_PINMUX);
	val &= ~APPL_PINMUX_PEX_RST;
	writel(val, pcie->appl_base + APPL_PINMUX);

	usleep_range(100, 200);

	/* enable LTSSM */
	val = readl(pcie->appl_base + APPL_CTRL);
	val |= APPL_CTRL_LTSSM_EN;
	writel(val, pcie->appl_base + APPL_CTRL);

	/* de-assert RST */
	val = readl(pcie->appl_base + APPL_PINMUX);
	val |= APPL_PINMUX_PEX_RST;
	writel(val, pcie->appl_base + APPL_PINMUX);

	msleep(100);

	val = readl(pp->dbi_base + CFG_LINK_STATUS_CONTROL);
	while (!(val & CFG_LINK_STATUS_DLL_ACTIVE)) {
		if (!count) {
			val = readl(pcie->appl_base + APPL_APPL_DEBUG);
			val &= APPL_APPL_DEBUG_SMLH_LTSSM_STATE_MASK;
			val >>= APPL_APPL_DEBUG_SMLH_LTSSM_STATE_SHIFT;
			tmp = readl(pcie->appl_base + APPL_LINK_STATUS);
			tmp &= APPL_LINK_STATUS_RDLH_LINK_UP;
			if ((val == 0x11) && !tmp) {
				dev_info(pp->dev, "link is down in DLL, "
					 "try again with DLFE disabled");
				/* disable LTSSM */
				val = readl(pcie->appl_base + APPL_CTRL);
				val &= ~APPL_CTRL_LTSSM_EN;
				writel(val, pcie->appl_base + APPL_CTRL);

				reset_control_assert(pcie->core_rst);
				reset_control_deassert(pcie->core_rst);

				dw_pcie_cfg_read(pp->dbi_base +
						 pcie->dl_feature_cap,
						 4, &val);
				val &= ~DL_FEATURE_EXCHANGE_EN;
				dw_pcie_cfg_write(pp->dbi_base +
						  pcie->dl_feature_cap,
						  4, val);

				tegra_pcie_dw_host_init(&pcie->pp);
				return;
			}
			dev_info(pp->dev, "link is down\n");
			return;
		}
		dev_dbg(pp->dev, "polling for link up\n");
		usleep_range(1000, 2000);
		val = readl(pp->dbi_base + CFG_LINK_STATUS_CONTROL);
		count--;
	}
	dev_info(pp->dev, "link is up\n");

	tegra_pcie_enable_interrupts(pp);
}

static int tegra_pcie_dw_link_up(struct pcie_port *pp)
{
	u32 val = readl(pp->dbi_base + CFG_LINK_STATUS_CONTROL);

	return !!(val & CFG_LINK_STATUS_DLL_ACTIVE);
}

static void tegra_pcie_dw_scan_bus(struct pcie_port *pp)
{
	struct pci_dev *pdev = NULL;
	u16 val = 0;
	u32 data = 0, pos = 0;
	struct pci_bus *child;

	list_for_each_entry(child, &pp->bus->children, node) {
		/* L1SS programming only for immediate downstream devices */
		if (child->parent == pp->bus) {
			pdev = pci_get_slot(child, PCI_DEVFN(0, 0));
			if (!pdev)
				break;
			pos = pci_find_ext_capability(pdev,
						      PCI_EXT_CAP_ID_L1SS);
			if (!pos)
				continue;
			pci_read_config_dword(pdev, pos + PCI_L1SS_CAP, &data);
			if (!((data & PCI_L1SS_CAP_ASPM_L12S) ||
			      (data & PCI_L1SS_CAP_PM_L12S)))
				continue;
			pcie_capability_read_dword(pdev, PCI_EXP_DEVCAP2,
						   &data);
			if (data & PCI_EXP_DEVCAP2_LTR) {
				pcie_capability_read_word(pdev,
							  PCI_EXP_DEVCTL2,
							  &val);
				val |= PCI_EXP_DEVCTL2_LTR_EN;
				pcie_capability_write_word(pdev,
							   PCI_EXP_DEVCTL2,
							   val);
			}
		}
	}
}

static struct pcie_host_ops tegra_pcie_dw_host_ops = {
	.rd_own_conf = tegra_pcie_dw_rd_own_conf,
	.wr_own_conf = tegra_pcie_dw_wr_own_conf,
	.rd_other_conf = tegra_pcie_dw_rd_other_conf,
	.wr_other_conf = tegra_pcie_dw_wr_other_conf,
	.link_up = tegra_pcie_dw_link_up,
	.host_init = tegra_pcie_dw_host_init,
	.scan_bus = tegra_pcie_dw_scan_bus,
};

static int tegra_add_pcie_port(struct pcie_port *pp,
			       struct platform_device *pdev)
{
	struct tegra_pcie_dw *pcie = to_tegra_pcie(pp);
	int ret;

	pp->irq = platform_get_irq_byname(pdev, "intr");
	if (!pp->irq) {
		dev_err(pp->dev, "failed to get intr interrupt\n");
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, pp->irq, tegra_pcie_irq_handler,
			       IRQF_SHARED, "tegra-pcie-intr", pp);
	if (ret) {
		dev_err(pp->dev, "failed to request \"intr\" irq\n");
		return ret;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (!pp->msi_irq) {
			dev_err(pp->dev, "failed to get msi interrupt\n");
			ret = -ENODEV;
			goto fail_get_msi;
		}

		ret = devm_request_irq(&pdev->dev, pp->msi_irq,
				       tegra_pcie_msi_irq_handler,
				       IRQF_SHARED | IRQF_NO_THREAD,
				       "tegra-pcie-msi", pp);
		if (ret) {
			dev_err(pp->dev, "failed to request \"msi\" irq\n");
			goto fail_get_msi;
		}
	}

	/* program to use MPS of 256 whereever possible */
	pcie_bus_config = PCIE_BUS_PERFORMANCE;

	pp->root_bus_nr = -1;
	pp->ops = &tegra_pcie_dw_host_ops;

	/* Disable MSI interrupts for PME messages */
	pcie_pme_disable_msi();

	ret = dw_pcie_host_init(pp);

	if (ret == -ENOMEDIUM && pcie->power_down_en) {
		if (pcie->cid != CTRL_5) {
			if (uphy_bpmp_pcie_controller_state_set(pcie->cid,
								false))
				dev_err(pcie->dev,
					"Disabling controller-%d failed\n",
					pcie->cid);
		}
		goto fail_host_init;
	} else if (ret && ret != -ENOMEDIUM) {
		dev_err(pp->dev, "failed to initialize host\n");
		goto fail_host_init;
	}
	ret = 0;

	return ret;

fail_host_init:
	if (IS_ENABLED(CONFIG_PCI_MSI))
		devm_free_irq(&pdev->dev, pp->msi_irq, pp);
fail_get_msi:
	devm_free_irq(&pdev->dev, pp->irq, pp);
	return ret;
}

static void tegra_pcie_disable_phy(struct tegra_pcie_dw *pcie)
{
	int phy_count = pcie->phy_count;

	while (phy_count--) {
		phy_power_off(pcie->phy[phy_count]);
		phy_exit(pcie->phy[phy_count]);
	}
}

static int tegra_pcie_enable_phy(struct tegra_pcie_dw *pcie)
{
	int phy_count = pcie->phy_count;
	int ret;
	int i;

	for (i = 0; i < phy_count; i++) {
		ret = phy_init(pcie->phy[i]);
		if (ret < 0)
			goto err_phy_init;

		ret = phy_power_on(pcie->phy[i]);
		if (ret < 0) {
			phy_exit(pcie->phy[i]);
			goto err_phy_power_on;
		}
	}

	return 0;

	while (i >= 0) {
		phy_power_off(pcie->phy[i]);
err_phy_power_on:
		phy_exit(pcie->phy[i]);
err_phy_init:
		i--;
	}

	return ret;
}

static int tegra_pcie_dw_probe(struct platform_device *pdev)
{
	struct tegra_pcie_dw *pcie;
	struct pcie_port *pp;
	struct device_node *np = pdev->dev.of_node;
	struct phy **phy;
	struct resource *appl_res;
	struct resource	*dbi_res;
	struct resource	*atu_dma_res;
	struct pinctrl *pin = NULL;
	struct pinctrl_state *pin_state = NULL;
	char *name;
	int phy_count;
	int ret, i = 0;
	u32 val = 0;

	pcie = devm_kzalloc(&pdev->dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pp = &pcie->pp;

	pp->dev = &pdev->dev;
	pcie->dev = &pdev->dev;

	ret = of_property_read_u32(np, "nvidia,cfg-link-cap-l1sub",
				   &pcie->cfg_link_cap_l1sub);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read cfg-link-cap-l1sub: %d\n",
			ret);
		pcie->cfg_link_cap_l1sub = CFG_LINK_CAP_L1SUB;
	}
	of_property_read_u32(np, "nvidia,cap-pl16g-status",
			     &pcie->cap_pl16g_status);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read cap-pl16g-status: %d\n", ret);
		pcie->cap_pl16g_status = CAP_PL16G_STATUS_REG;
	}
	of_property_read_u32(np, "nvidia,cap-pl16g-cap-off",
			     &pcie->cap_pl16g_cap_off);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read cap-pl16g-cap-off: %d\n", ret);
		pcie->cap_pl16g_cap_off = PL16G_CAP_OFF;
	}
	of_property_read_u32(np, "nvidia,event-cntr-ctrl",
			     &pcie->event_cntr_ctrl);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read event-cntr-ctrl: %d\n", ret);
		pcie->event_cntr_ctrl = EVENT_COUNTER_CONTROL_REG;
	}
	of_property_read_u32(np, "nvidia,event-cntr-data",
			     &pcie->event_cntr_data);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read event-cntr-data: %d\n", ret);
		pcie->event_cntr_data = EVENT_COUNTER_DATA_REG;
	}

	of_property_read_u32(np, "nvidia,dl-feature-cap",
			     &pcie->dl_feature_cap);
	if (ret < 0)
		dev_err(pcie->dev, "fail to read dl_feature_cap: %d\n", ret);

	ret = of_property_read_u32(np, "num-lanes", &pcie->num_lanes);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read num-lanes: %d\n", ret);
		pcie->num_lanes = 0;
	}
	ret = of_property_read_u32(np, "nvidia,max-speed", &pcie->max_speed);
	if ((ret < 0) | (val < 0 && val > 4)) {
		dev_err(pcie->dev, "invalid max-speed (err=%d), set to Gen-1\n",
			ret);
		pcie->max_speed = 1;
	}

	pcie->power_down_en = of_property_read_bool(pcie->dev->of_node,
		"nvidia,enable-power-down");
	pp->skip_enum = pcie->power_down_en;

	if (tegra_platform_is_fpga()) {
		pcie->cfg_link_cap_l1sub = CFG_LINK_CAP_L1SUB;
		pcie->event_cntr_ctrl = EVENT_COUNTER_CONTROL_REG;
		pcie->event_cntr_data = EVENT_COUNTER_DATA_REG;
	}

	ret = of_property_read_u32(np, "nvidia,controller-id", &pcie->cid);
	if (ret) {
		dev_err(pcie->dev, "Controller-ID is missing in DT: %d\n", ret);
		return ret;
	}

	if (pcie->cid != CTRL_5) {
		ret = uphy_bpmp_pcie_controller_state_set(pcie->cid, true);
		if (ret) {
			dev_err(pcie->dev, "Enabling controller-%d failed:%d\n",
				pcie->cid, ret);
			return ret;
		}
	}

	pcie->pex_ctl_reg = devm_regulator_get(&pdev->dev, "vddio-pex-ctl");
	if (IS_ERR(pcie->pex_ctl_reg)) {
		dev_err(&pdev->dev, "fail to get regulator: %ld\n",
			PTR_ERR(pcie->pex_ctl_reg));
		return PTR_ERR(pcie->pex_ctl_reg);
	}
	ret = regulator_enable(pcie->pex_ctl_reg);
	if (ret < 0) {
		dev_err(&pdev->dev, "regulator enable failed: %d\n", ret);
		return ret;
	}

	pin = devm_pinctrl_get(pcie->dev);
	if (IS_ERR(pin)) {
		ret = PTR_ERR(pin);
		dev_err(pcie->dev, "pinctrl_get failed: %d\n", ret);
		return ret;
	}
	pin_state = pinctrl_lookup_state(pin, "pex_rst");
	if (!IS_ERR(pin_state)) {
		ret = pinctrl_select_state(pin, pin_state);
		if (ret < 0) {
			dev_err(pcie->dev, "setting pex_rst state fail: %d\n",
				ret);
			return ret;
		}
	}
	pin_state = pinctrl_lookup_state(pin, "clkreq");
	if (!IS_ERR(pin_state)) {
		ret = pinctrl_select_state(pin, pin_state);
		if (ret < 0) {
			dev_err(pcie->dev, "setting clkreq state fail: %d\n",
				ret);
			return ret;
		}
	}

	pcie->core_clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(pcie->core_clk)) {
		dev_err(&pdev->dev, "Failed to get core clock\n");
		ret = PTR_ERR(pcie->core_clk);
		goto fail_core_clk;
	}
	ret = clk_prepare_enable(pcie->core_clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable core clock\n");
		goto fail_core_clk;
	}

	appl_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "appl");
	if (!appl_res) {
		dev_err(&pdev->dev, "missing appl space\n");
		ret = PTR_ERR(appl_res);
		goto fail_appl_res;
	}
	pcie->appl_base = devm_ioremap_resource(&pdev->dev, appl_res);
	if (IS_ERR(pcie->appl_base)) {
		dev_err(&pdev->dev, "mapping appl space failed\n");
		ret = PTR_ERR(pcie->appl_base);
		goto fail_appl_res;
	}

	pcie->core_apb_rst = devm_reset_control_get(pcie->dev, "core_apb_rst");
	if (IS_ERR(pcie->core_apb_rst)) {
		dev_err(pcie->dev, "PCIE : core_apb_rst reset is missing\n");
		ret = PTR_ERR(pcie->core_apb_rst);
		goto fail_appl_res;
	}

	reset_control_deassert(pcie->core_apb_rst);

	phy_count = of_property_count_strings(np, "phy-names");
	if (phy_count < 0) {
		dev_err(pcie->dev, "unable to find phy entries\n");
		ret = phy_count;
		goto fail_phy;
	}

	phy = devm_kcalloc(pcie->dev, phy_count, sizeof(*phy), GFP_KERNEL);
	if (!phy) {
		ret = PTR_ERR(phy);
		goto fail_phy;
	}

	for (i = 0; i < phy_count; i++) {
		name = kasprintf(GFP_KERNEL, "pcie-p2u-%u", i);
		phy[i] = devm_phy_get(pcie->dev, name);
		kfree(name);
		if (IS_ERR(phy[i])) {
			ret = PTR_ERR(phy[i]);
			dev_err(pcie->dev, "phy_get error: %d\n", ret);
			goto fail_phy;
		}
	}

	pcie->phy_count = phy_count;
	pcie->phy = phy;

	ret = tegra_pcie_enable_phy(pcie);
	if (ret) {
		dev_err(pcie->dev, "failed to enable phy\n");
		goto fail_phy;
	}

	dbi_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	if (!dbi_res) {
		dev_err(&pdev->dev, "missing config space\n");
		ret = PTR_ERR(dbi_res);
		goto fail_dbi_res;
	}
	pcie->dbi_res = dbi_res;
	pp->dbi_base = devm_ioremap_resource(&pdev->dev, dbi_res);
	if (IS_ERR(pp->dbi_base)) {
		dev_err(&pdev->dev, "mapping dbi space failed\n");
		ret = PTR_ERR(pp->dbi_base);
		goto fail_dbi_res;
	}
	pp->va_cfg0_base = pp->dbi_base;
	pp->va_cfg1_base = pp->dbi_base + resource_size(dbi_res) / 2;

	/* Disable SLCG */
	/* NOTE:- This needs to be removed after initial bringup */
	val = readl(pcie->appl_base + APPL_CFG_SLCG_OVERRIDE);
	writel(val | APPL_CFG_SLCG_OVERRIDE_SLCG_EN_MASTER,
	       pcie->appl_base + APPL_CFG_SLCG_OVERRIDE);

	/* update CFG base address */
	writel(dbi_res->start & APPL_CFG_BASE_ADDR_MASK,
	       pcie->appl_base + APPL_CFG_BASE_ADDR);

	/* configure this core for RP mode operation */
	writel(APPL_DM_TYPE_RP, pcie->appl_base + APPL_DM_TYPE);

	val = readl(pcie->appl_base + APPL_CTRL);
	writel(val | APPL_CTRL_SYS_PRE_DET_STATE, pcie->appl_base + APPL_CTRL);

	pcie->disable_clock_request = of_property_read_bool(pcie->dev->of_node,
		"nvidia,disable-clock-request");
	if (pcie->disable_clock_request) {
		val = readl(pcie->appl_base + APPL_PINMUX);
		val |= APPL_PINMUX_CLKREQ_OUT_OVRD_EN;
		val |= APPL_PINMUX_CLKREQ_OUT_OVRD;
		writel(val, pcie->appl_base + APPL_PINMUX);

		/* Disable ASPM-L1SS adv as there is no CLKREQ routing */
		disable_aspm_l11(pcie); /* Disable L1.1 */
		disable_aspm_l12(pcie); /* Disable L1.2 */
	}

	atu_dma_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "atu_dma");
	if (!atu_dma_res) {
		dev_err(&pdev->dev, "missing atu_dma space\n");
		ret = PTR_ERR(atu_dma_res);
		goto fail_dbi_res;
	}
	pcie->atu_dma_res = atu_dma_res;
	pcie->atu_dma_base = devm_ioremap_resource(&pdev->dev, atu_dma_res);
	if (IS_ERR(pcie->atu_dma_base)) {
		dev_err(&pdev->dev, "mapping atu_dma space failed\n");
		ret = PTR_ERR(pcie->atu_dma_base);
		goto fail_dbi_res;
	}

	/* update iATU_DMA base address */
	writel(atu_dma_res->start & APPL_CFG_IATU_DMA_BASE_ADDR_MASK,
	       pcie->appl_base + APPL_CFG_IATU_DMA_BASE_ADDR);

	pcie->core_rst = devm_reset_control_get(pcie->dev, "core_rst");
	if (IS_ERR(pcie->core_rst)) {
		dev_err(pcie->dev, "PCIE : core_rst reset is missing\n");
		ret = PTR_ERR(pcie->core_rst);
		goto fail_dbi_res;
	}

	reset_control_deassert(pcie->core_rst);

	ret = tegra_add_pcie_port(pp, pdev);
	if (ret < 0) {
		if (ret != -ENOMEDIUM)
			dev_err(pcie->dev, "PCIE : Add PCIe port failed: %d\n",
				ret);
		goto fail_add_port;
	}

	platform_set_drvdata(pdev, pcie);

	/* Enable DMA processing engines */
	for (i = 0; i < DMA_WR_CHNL_NUM; i++) {
		mutex_init(&pcie->wr_lock[i]);
		init_completion(&pcie->wr_cpl[i]);
	}

	for (i = 0; i < DMA_RD_CHNL_NUM; i++) {
		mutex_init(&pcie->rd_lock[i]);
		init_completion(&pcie->rd_cpl[i]);
	}

	/* Enable ASPM counters */
	val = EVENT_COUNTER_ENABLE_ALL << EVENT_COUNTER_ENABLE_SHIFT;
	val |= EVENT_COUNTER_GROUP_5 << EVENT_COUNTER_GROUP_SEL_SHIFT;
	dw_pcie_cfg_write(pcie->pp.dbi_base + pcie->event_cntr_ctrl, 4, val);

	pcie->debugfs = debugfs_create_dir(pdev->dev.of_node->name, NULL);
	if (!pcie->debugfs)
		dev_err(pcie->dev, "debugfs creation failed\n");
	else
		init_debugfs(pcie);

	return 0;

fail_add_port:
	reset_control_assert(pcie->core_rst);
fail_dbi_res:
	tegra_pcie_disable_phy(pcie);
fail_phy:
	reset_control_assert(pcie->core_apb_rst);
fail_appl_res:
	clk_disable_unprepare(pcie->core_clk);
fail_core_clk:
	regulator_disable(pcie->pex_ctl_reg);
	return ret;
}

static void tegra_pcie_dw_pme_turnoff(struct tegra_pcie_dw *pcie)
{
	u32 val;
	int err;

	if (!tegra_pcie_dw_link_up(&pcie->pp))
		return;

	val = readl(pcie->appl_base + APPL_RADM_STATUS);
	val |= APPL_PM_XMT_TURNOFF_STATE;
	writel(val, pcie->appl_base + APPL_RADM_STATUS);

	err = readl_poll_timeout(pcie->appl_base + APPL_APPL_DEBUG, val,
				 val & APPL_APPL_DEBUG_PM_LINKST_IN_L2_LAT,
				 1, PME_ACK_TIMEOUT);
	if (err)
		dev_err(pcie->dev, "PME_TurnOff failed: %d\n", err);
}

static int tegra_pcie_dw_remove(struct platform_device *pdev)
{
	struct tegra_pcie_dw *pcie = platform_get_drvdata(pdev);
	int i;

	debugfs_remove_recursive(pcie->debugfs);

	for (i = 0; i < DMA_WR_CHNL_NUM; i++)
		mutex_destroy(&pcie->wr_lock[i]);

	for (i = 0; i < DMA_RD_CHNL_NUM; i++)
		mutex_destroy(&pcie->rd_lock[i]);

	dw_pcie_host_deinit(&pcie->pp);
	tegra_pcie_dw_pme_turnoff(pcie);

	reset_control_assert(pcie->core_rst);
	tegra_pcie_disable_phy(pcie);
	reset_control_assert(pcie->core_apb_rst);
	clk_disable_unprepare(pcie->core_clk);
	regulator_disable(pcie->pex_ctl_reg);

	return 0;
}

static int tegra_pcie_dw_suspend_noirq(struct device *dev)
{
	struct tegra_pcie_dw *pcie = dev_get_drvdata(dev);
	int ret = 0;

	/*TODO re verify the sequence with IAS*/
	tegra_pcie_dw_pme_turnoff(pcie);
	reset_control_assert(pcie->core_rst);
	tegra_pcie_disable_phy(pcie);
	reset_control_assert(pcie->core_apb_rst);
	clk_disable_unprepare(pcie->core_clk);
	regulator_disable(pcie->pex_ctl_reg);
	if (pcie->cid != CTRL_5) {
		ret = uphy_bpmp_pcie_controller_state_set(pcie->cid, false);
		if (ret) {
			dev_err(pcie->dev, "Disabling ctrl-%d failed:%d\n",
				pcie->cid, ret);
			return ret;
		}
	}

	return ret;
}

static int tegra_pcie_dw_resume_noirq(struct device *dev)
{
	struct tegra_pcie_dw *pcie = dev_get_drvdata(dev);
	int ret;
	u32 val;

	/*TODO re verify the sequence with IAS*/
	if (pcie->cid != CTRL_5) {
		ret = uphy_bpmp_pcie_controller_state_set(pcie->cid, true);
		if (ret) {
			dev_err(pcie->dev, "Enabling controller-%d failed:%d\n",
				pcie->cid, ret);
			return ret;
		}
	}

	ret = regulator_enable(pcie->pex_ctl_reg);
	if (ret < 0) {
		dev_err(dev, "regulator enable failed: %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(pcie->core_clk);
	if (ret) {
		dev_err(dev, "Failed to enable core clock\n");
		goto fail_core_clk;
	}
	reset_control_deassert(pcie->core_apb_rst);
	ret = tegra_pcie_enable_phy(pcie);
	if (ret) {
		dev_err(dev, "failed to enable phy\n");
		goto fail_phy;
	}
	writel(pcie->dbi_res->start & APPL_CFG_BASE_ADDR_MASK,
	       pcie->appl_base + APPL_CFG_BASE_ADDR);

	/* configure this core for RP mode operation */
	writel(APPL_DM_TYPE_RP, pcie->appl_base + APPL_DM_TYPE);

	val = readl(pcie->appl_base + APPL_CTRL);
	writel(val | APPL_CTRL_SYS_PRE_DET_STATE, pcie->appl_base + APPL_CTRL);

	if (pcie->disable_clock_request) {
		val = readl(pcie->appl_base + APPL_PINMUX);
		val |= APPL_PINMUX_CLKREQ_OUT_OVRD_EN;
		val |= APPL_PINMUX_CLKREQ_OUT_OVRD;
		writel(val, pcie->appl_base + APPL_PINMUX);

		/* Disable ASPM-L1SS adv as there is no CLKREQ routing */
		disable_aspm_l11(pcie); /* Disable L1.1 */
		disable_aspm_l12(pcie); /* Disable L1.2 */
	}

	/* update iATU_DMA base address */
	writel(pcie->atu_dma_res->start & APPL_CFG_IATU_DMA_BASE_ADDR_MASK,
	       pcie->appl_base + APPL_CFG_IATU_DMA_BASE_ADDR);

	reset_control_deassert(pcie->core_rst);

	tegra_pcie_dw_host_init(&pcie->pp);

	/* Enable ASPM counters */
	val = EVENT_COUNTER_ENABLE_ALL << EVENT_COUNTER_ENABLE_SHIFT;
	val |= EVENT_COUNTER_GROUP_5 << EVENT_COUNTER_GROUP_SEL_SHIFT;
	dw_pcie_cfg_write(pcie->pp.dbi_base + pcie->event_cntr_ctrl, 4, val);

	return 0;
fail_phy:
	reset_control_assert(pcie->core_apb_rst);
	clk_disable_unprepare(pcie->core_clk);
fail_core_clk:
	regulator_disable(pcie->pex_ctl_reg);
	return ret;
}

static const struct of_device_id tegra_pcie_dw_of_match[] = {
	{ .compatible = "nvidia,tegra194-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_pcie_dw_of_match);

static const struct dev_pm_ops tegra_pcie_dw_pm_ops = {
	.suspend_noirq = tegra_pcie_dw_suspend_noirq,
	.resume_noirq = tegra_pcie_dw_resume_noirq,
};

static struct platform_driver tegra_pcie_dw_driver = {
	.probe = tegra_pcie_dw_probe,
	.remove = tegra_pcie_dw_remove,
	.driver = {
		.name	= "tegra-pcie-dw",
#ifdef CONFIG_PM
		.pm = &tegra_pcie_dw_pm_ops,
#endif
		.of_match_table = tegra_pcie_dw_of_match,
	},
};

static int __init tegra_pcie_rp_init(void)
{
	return platform_driver_register(&tegra_pcie_dw_driver);
}

#if IS_MODULE(CONFIG_PCIE_TEGRA_DW)
static void __exit tegra_pcie_rp_deinit(void)
{
	platform_driver_unregister(&tegra_pcie_dw_driver);
}

module_init(tegra_pcie_rp_init);
module_exit(tegra_pcie_rp_deinit);
#else
late_initcall(tegra_pcie_rp_init);
#endif

MODULE_AUTHOR("Vidya Sagar <vidyas@nvidia.com>");
MODULE_DESCRIPTION("Nvidia PCIe host controller driver");
MODULE_LICENSE("GPL v2");
