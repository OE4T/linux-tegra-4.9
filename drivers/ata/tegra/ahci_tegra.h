/*
 * drivers/ata/ahci_tegra.h
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _AHCI_TEGRA_H
#define _AHCI_TEGRA_H

#include <linux/ahci_platform.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <scsi/scsi_device.h>
#include <linux/tegra-powergate.h>
#include <linux/tegra-soc.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

#include "../ahci.h"

/* AUX Registers */
#define SATA_AUX_RX_STAT_INT_0				0xc
#define SATA_DEVSLP					BIT(7)

#define SATA_AUX_MISC_CNTL_1_0				0x8
#define DEVSLP_OVERRIDE					BIT(17)

/* IPFS Register Space */
#define SATA_CONFIGURATION_0				0x180
#define SATA_CONFIGURATION_0_EN_FPCI			BIT(0)
#define SATA_CONFIGURATION_CLK_OVERRIDE			BIT(31)

#define SATA_FPCI_BAR5_0				0x94
#define FPCI_BAR5_START_MASK				(0xFFFFFFF << 4)
#define FPCI_BAR5_START					(0x0040020 << 4)
#define FPCI_BAR5_ACCESS_TYPE				(0x1)

#define SATA_INTR_MASK_0				0x188
#define IP_INT_MASK					BIT(16)

/* Configuration Register Space */
#define T_SATA_BKDOOR_CC				0x4A4
#define T_SATA_BKDOOR_CC_CLASS_CODE_MASK		(0xFFFF << 16)
#define T_SATA_BKDOOR_CC_CLASS_CODE			(0x0106 << 16)
#define T_SATA_BKDOOR_CC_PROG_IF_MASK			(0xFF << 8)
#define T_SATA_BKDOOR_CC_PROG_IF			(0x01 << 8)

#define T_SATA_CFG_SATA					0x54C
#define T_SATA_CFG_SATA_BACKDOOR_PROG_IF_EN		BIT(12)

#define T_SATA_CHX_PHY_CTRL17				0x6E8
#define T_SATA_CHX_PHY_CTRL17_RX_EQ_CTRL_L_GEN1		0x55010000

#define T_SATA_CHX_PHY_CTRL18				0x6EC
#define T_SATA_CHX_PHY_CTRL18_RX_EQ_CTRL_L_GEN2		0x55010000

#define T_SATA_CHX_PHY_CTRL20				0x6F4
#define T_SATA_CHX_PHY_CTRL20_RX_EQ_CTRL_H_GEN1		0x01

#define T_SATA_CHX_PHY_CTRL21				0x6F8
#define T_SATA_CHX_PHY_CTRL21_RX_EQ_CTRL_H_GEN2		0x01

#define T_SATA0_AHCI_HBA_CAP_BKDR			0x300
#define T_SATA0_AHCI_HBA_CAP_BKDR_PARTIAL_ST_CAP	BIT(13)
#define T_SATA0_AHCI_HBA_CAP_BKDR_SLUMBER_ST_CAP	BIT(14)
#define T_SATA0_AHCI_HBA_CAP_BKDR_SALP			BIT(26)
#define T_SATA0_AHCI_HBA_CAP_BKDR_SUPP_PM		BIT(17)

#define T_SATA_CFG_PHY_0				0x120
#define T_SATA_CFG_PHY_0_MASK_SQUELCH			BIT(24)
#define T_SATA_CFG_PHY_0_USE_7BIT_ALIGN_DET_FOR_SPD	BIT(11)

#define T_SATA0_NVOOB					0x114
#define T_SATA0_NVOOB_COMMA_CNT_MASK			(0XFF << 16)
#define T_SATA0_NVOOB_COMMA_CNT				(0x07 << 16)
#define T_SATA0_NVOOB_SQUELCH_FILTER_LENGTH_MASK	(0x3 << 26)
#define T_SATA0_NVOOB_SQUELCH_FILTER_LENGTH		(0x3 << 26)
#define T_SATA0_NVOOB_SQUELCH_FILTER_MODE_MASK		(0x3 << 24)
#define T_SATA0_NVOOB_SQUELCH_FILTER_MODE		(0x1 << 24)

#define T_SATA_CFG_1					0x4
#define T_SATA_CFG_1_IO_SPACE				BIT(0)
#define T_SATA_CFG_1_MEMORY_SPACE			BIT(1)
#define T_SATA_CFG_1_BUS_MASTER				BIT(2)
#define T_SATA_CFG_1_SERR				BIT(8)

#define T_SATA_CFG_9					0x24
#define T_SATA_CFG_9_BASE_ADDRESS			0x40020000

#define T_SATA0_CFG_35					0x94
#define T_SATA0_CFG_35_IDP_INDEX_MASK			(0x7FF << 2)
#define T_SATA0_CFG_35_IDP_INDEX			(0x2A << 2)

#define T_SATA0_AHCI_IDP1				0x98
#define T_SATA0_AHCI_IDP1_DATA				(0x400040)

#define T_SATA0_CFG_PHY_1				0x12C
#define T_SATA0_CFG_PHY_1_PADS_IDDQ_EN			BIT(23)
#define T_SATA0_CFG_PHY_1_PAD_PLL_IDDQ_EN		BIT(22)

#define T_SATA0_INDEX					0x680
#define T_SATA0_INDEX_NONE_SELECTED                     0
#define T_SATA0_INDEX_CH1				BIT(0)


/* AHCI registers */

/* AHCI Port Registers */
#define T_AHCI_PORT_PXSSTS				0x128

#define TEGRA_SATA_CORE_CLOCK_FREQ_HZ			(102*1000*1000)
#define TEGRA_SATA_OOB_CLOCK_FREQ_HZ			(204*1000*1000)

#define TEGRA_AHCI_MAX_CLKS				2
#define TEGRA_AHCI_DEFAULT_IDLE_TIME			10000

enum tegra_sata_bars {
	TEGRA_SATA_IPFS = 0,
	TEGRA_SATA_CONFIG,
	TEGRA_SATA_AHCI,
	TEGRA_SATA_AUX,
	TEGRA_SATA_BARS_MAX,
};

enum tegra_ahci_port_runtime_status {
	TEGRA_AHCI_PORT_RUNTIME_ACTIVE	= 1,
	TEGRA_AHCI_PORT_RUNTIME_PARTIAL	= 2,
	TEGRA_AHCI_PORT_RUNTIME_SLUMBER	= 6,
	TEGRA_AHCI_PORT_RUNTIME_DEVSLP	= 8,
};

struct tegra_ahci_priv {
	struct platform_device	   *pdev;
	void __iomem               *base_list[TEGRA_SATA_BARS_MAX];
	struct resource		   *res[TEGRA_SATA_BARS_MAX];
	struct regulator_bulk_data *supplies;
	struct tegra_ahci_soc_data *soc_data;
	void			   *pg_save;
	struct reset_control	   *sata_rst;
	struct reset_control	   *sata_cold_rst;
	struct clk		   *sata_clk;
	struct clk		   *sata_oob_clk;
	struct clk		   *pllp_clk; /* sata_oob clk parent */
	struct clk		   *pllp_uphy_clk; /* sata_clk parent */
	bool			   devslp_override;
};

struct tegra_ahci_ops {
	int (*tegra_ahci_power_on)(struct ahci_host_priv *);
	void (*tegra_ahci_power_off)(struct ahci_host_priv *);
	int (*tegra_ahci_quirks)(struct ahci_host_priv *);
	struct ahci_host_priv * (*tegra_ahci_platform_get_resources)
					(struct tegra_ahci_priv *);
};

struct tegra_ahci_soc_data {
	char    * const *sata_regulator_names;
	int     num_sata_regulators;
	struct tegra_ahci_ops ops;
	void	*data;
};

#ifdef CONFIG_PM
static u32 pg_save_bar5_registers[] = {
	0x018,  /* T_AHCI_HBA_CCC_PORTS */
	0x004,  /* T_AHCI_HBA_GHC */
	0x014,  /* T_AHCI_HBA_CCC_CTL - OP (optional) */
	0x01C,  /* T_AHCI_HBA_EM_LOC */
	0x020   /* T_AHCI_HBA_EM_CTL - OP */
};

static u32 pg_save_bar5_port_registers[] = {
	0x100,  /* T_AHCI_PORT_PXCLB */
	0x104,  /* T_AHCI_PORT_PXCLBU */
	0x108,  /* T_AHCI_PORT_PXFB */
	0x10C,  /* T_AHCI_PORT_PXFBU */
	0x114,  /* T_AHCI_PORT_PXIE */
	0x118,  /* T_AHCI_PORT_PXCMD */
	0x12C,  /* T_AHCI_PORT_PXSCTL */
	0x144   /* T_AHCI_PORT_PXDEVSLP */
};

/*
 * pg_save_bar5_bkdr_registers:
 *    These registers in BAR5 are read only.
 * To restore back those register values, write the saved value
 *    to the registers specified in pg_restore_bar5_bkdr_registers[].
 *    These pg_restore_bar5_bkdr_registers[] are in SATA_CONFIG space.
 */
static u32 pg_save_bar5_bkdr_registers[] = {
	/* Save and restore via bkdr writes */
	0x000,  /* T_AHCI_HBA_CAP */
	0x00C,  /* T_AHCI_HBA_PI */
	0x024   /* T_AHCI_HBA_CAP2 */
};

static u32 pg_restore_bar5_bkdr_registers[] = {
	/* Save and restore via bkdr writes */
	0x300,  /* BKDR of T_AHCI_HBA_CAP */
	0x33c,  /* BKDR of T_AHCI_HBA_PI */
	0x330   /* BKDR of T_AHCI_HBA_CAP2 */
};

/* These registers are saved for each port */
static u32 pg_save_bar5_bkdr_port_registers[] = {
	0x120,  /* NV_PROJ__SATA0_CHX_AHCI_PORT_PXTFD  */
	0x124,  /* NV_PROJ__SATA0_CHX_AHCI_PORT_PXSIG */
	0x128   /* NV_PROJ__SATA0_CHX_AHCI_PORT_PXSSTS */
};

static u32 pg_restore_bar5_bkdr_port_registers[] = {
	/* Save and restore via bkdr writes */
	0x790,  /* BKDR of NV_PROJ__SATA0_CHX_AHCI_PORT_PXTFD  */
	0x794,  /* BKDR of NV_PROJ__SATA0_CHX_AHCI_PORT_PXSIG */
	0x798   /* BKDR of NV_PROJ__SATA0_CHX_AHCI_PORT_PXSSTS */
};

static u32 pg_save_config_registers[] = {
	0x004,  /* T_SATA0_CFG_1 */
	0x00C,  /* T_SATA0_CFG_3 */
	0x024,  /* T_SATA0_CFG_9 */
	0x028,  /* T_SATA0_CFG_10 */
	0x030,  /* T_SATA0_CFG_12 */
	0x034,  /* T_SATA0_CFG_13 */
	0x038,  /* T_SATA0_CFG_14 */
	0x03C,  /* T_SATA0_CFG_15 */
	0x040,  /* T_SATA0_CFG_16 */
	0x044,  /* T_SATA0_CFG_17 */
	0x048,  /* T_SATA0_CFG_18 */
	0x0B0,  /* T_SATA0_MSI_CTRL */
	0x0B4,  /* T_SATA0_MSI_ADDR1 */
	0x0B8,  /* T_SATA0_MSI_ADDR2 */
	0x0BC,  /* T_SATA0_MSI_DATA */
	0x0C0,  /* T_SATA0_MSI_QUEUE */
	0x0EC,  /* T_SATA0_MSI_MAP */
	0x124,  /* T_SATA0_CFG_PHY_POWER */
	0x128,  /* T_SATA0_CFG_PHY_POWER_1 */
	0x12C,  /* T_SATA0_CFG_PHY_1 */
	0x174,  /* T_SATA0_CFG_LINK_0 */
	0x178,  /* T_SATA0_CFG_LINK_1 */
	0x1D0,  /* MCP_SATA0_CFG_TRANS_0 */
	0x238,  /* T_SATA0_ALPM_CTRL */
	0x30C,  /* T_SATA0_AHCI_HBA_CYA_0 */
	0x320,  /* T_SATA0_AHCI_HBA_SPARE_1 */
	0x324,  /* T_SATA0_AHCI_HBA_SPARE_2 */
	0x328,  /* T_SATA0_AHCI_HBA_DYN_CLK_CLAMP */
	0x32C,  /* T_SATA0_AHCI_CFG_ERR_CTRL */
	0x338,  /* T_SATA0_AHCI_HBA_CYA_1 */
	0x340,  /* T_SATA0_AHCI_HBA_PRE_STAGING_CONTROL */
	0x430,  /* T_SATA0_CFG_FPCI_0 */
	0x494,  /* T_SATA0_CFG_ESATA_CTRL */
	0x4A0,  /* T_SATA0_CYA1 */
	0x4B0,  /* T_SATA0_CFG_GLUE */
	0x534,  /* T_SATA0_PHY_CTRL */
	0x540,  /* T_SATA0_CTRL */
	0x550,  /* T_SATA0_DBG0 */
	0x554   /* T_SATA0_LOW_POWER_COUNT */
};

static u32 pg_save_config_port_registers[] = {
	/* Save and restore per port */
	/* need to have port selected */
	0x530,  /* T_SATA0_CHXCFG1 */
	0x684,  /* T_SATA0_CHX_MISC */
	0x700,  /* T_SATA0_CHXCFG3 */
	0x704,  /* T_SATA0_CHXCFG4_CHX */
	0x690,  /* T_SATA0_CHX_PHY_CTRL1_GEN1 */
	0x694,  /* T_SATA0_CHX_PHY_CTRL1_GEN2 */
	0x698,  /* T_SATA0_CHX_PHY_CTRL1_GEN3 */
	0x69C,  /* T_SATA0_CHX_PHY_CTRL_2 */
	0x6B0,  /* T_SATA0_CHX_PHY_CTRL_3 */
	0x6B4,  /* T_SATA0_CHX_PHY_CTRL_4 */
	0x6B8,  /* T_SATA0_CHX_PHY_CTRL_5 */
	0x6BC,  /* T_SATA0_CHX_PHY_CTRL_6 */
	0x714,  /* T_SATA0_PRBS_CHX - OP */
	0x750,  /* T_SATA0_CHX_LINK0 */
	0x7F0   /* T_SATA0_CHX_GLUE */
};

static u32 pg_save_ipfs_registers[] = {
	0x094,  /* SATA_FPCI_BAR5_0 */
	0x0C0,  /* SATA_MSI_BAR_SZ_0 */
	0x0C4,  /* SATA_MSI_AXI_BAR_ST_0 */
	0x0C8,  /* SATA_MSI_FPCI_BAR_ST_0 */
	0x140,  /* SATA_MSI_EN_VEC0_0 */
	0x144,  /* SATA_MSI_EN_VEC1_0 */
	0x148,  /* SATA_MSI_EN_VEC2_0 */
	0x14C,  /* SATA_MSI_EN_VEC3_0 */
	0x150,  /* SATA_MSI_EN_VEC4_0 */
	0x154,  /* SATA_MSI_EN_VEC5_0 */
	0x158,  /* SATA_MSI_EN_VEC6_0 */
	0x15C,  /* SATA_MSI_EN_VEC7_0 */
	0x180,  /* SATA_CONFIGURATION_0 */
	0x184,  /* SATA_FPCI_ERROR_MASKS_0 */
	0x188,  /* SATA_INTR_MASK_0 */
	0x1A0,  /* SATA_CFG_REVID_0 */
	0x198,  /* SATA_IPFS_INTR_ENABLE_0 */
	0x1BC,  /* SATA_CLKGATE_HYSTERSIS_0 */
	0x1DC   /* SATA_SATA_MCCIF_FIFOCTRL_0 */
};

static inline void tegra_ahci_save_regs(u32 **save_addr,
		void __iomem *reg_base,
		u32 reg_array[],
		u32 regs)
{
	u32 i;
	u32 *dest = *save_addr;

	for (i = 0; i < regs; ++i, ++dest)
		*dest = readl(reg_base + reg_array[i]);
	*save_addr = dest;
}

static inline void tegra_ahci_restore_regs(void **save_addr,
		void __iomem *reg_base,
		u32 reg_array[],
		u32 regs)
{
	u32 i;
	u32 *src = *save_addr;

	for (i = 0; i < regs; ++i, ++src)
		writel(*src, reg_base + reg_array[i]);
	*save_addr = src;
}
#endif
static inline u32 tegra_ahci_bar5_readl(struct ahci_host_priv *hpriv,
								u32 offset)
{
	struct tegra_ahci_priv *tegra = hpriv->plat_data;
	u32 rval = 0;

	rval = readl(tegra->base_list[TEGRA_SATA_AHCI] + offset);
	return rval;
}

static inline void tegra_ahci_sata_update(struct ahci_host_priv *hpriv, u32 val,
					u32 mask, u32 offset)
{
	struct tegra_ahci_priv *tegra = hpriv->plat_data;
	u32 rval = 0;

	rval = readl(tegra->base_list[TEGRA_SATA_IPFS] + offset);
	rval = (rval & ~mask) | (val & mask);
	writel(rval, tegra->base_list[TEGRA_SATA_IPFS] + offset);
	rval = readl(tegra->base_list[TEGRA_SATA_IPFS] + offset);
}

static inline void tegra_ahci_scfg_writel(struct ahci_host_priv *hpriv, u32 val,
					u32 offset)
{
	struct tegra_ahci_priv *tegra = hpriv->plat_data;

	writel(val, tegra->base_list[TEGRA_SATA_CONFIG] + offset);
	readl(tegra->base_list[TEGRA_SATA_CONFIG] + offset);
}

static inline void tegra_ahci_scfg_update(struct ahci_host_priv *hpriv, u32 val,
					u32 mask, u32 offset)
{
	struct tegra_ahci_priv *tegra = hpriv->plat_data;
	u32 rval = 0;

	rval = readl(tegra->base_list[TEGRA_SATA_CONFIG] + offset);
	rval = (rval & ~mask) | (val & mask);
	writel(rval, tegra->base_list[TEGRA_SATA_CONFIG] + offset);
	rval = readl(tegra->base_list[TEGRA_SATA_CONFIG] + offset);
}

static inline u32 tegra_ahci_aux_readl(struct ahci_host_priv *hpriv, u32 offset)
{
	struct tegra_ahci_priv *tegra = hpriv->plat_data;
	u32 rval = 0;

	rval = readl(tegra->base_list[TEGRA_SATA_AUX] + offset);
	return rval;
}

static inline void tegra_ahci_aux_update(struct ahci_host_priv *hpriv, u32 val,
					u32 mask, u32 offset)
{
	struct tegra_ahci_priv *tegra = hpriv->plat_data;
	u32 rval = 0;

	rval = readl(tegra->base_list[TEGRA_SATA_AUX] + offset);
	rval = (rval & ~mask) | (val & mask);
	writel(rval, tegra->base_list[TEGRA_SATA_AUX] + offset);
	rval = readl(tegra->base_list[TEGRA_SATA_AUX] + offset);
}
#endif
