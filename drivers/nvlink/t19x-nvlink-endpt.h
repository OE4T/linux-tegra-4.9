/*
 * t19x-nvlink-endpt.h:
 * This header contains the structures and APIs needed by the Tegra NVLINK
 * endpoint driver.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef T19X_NVLINK_ENDPT_H
#define T19X_NVLINK_ENDPT_H

#include "nvlink.h"
#include <linux/tegra_prod.h>

#define NVLINK_DRV_NAME		"t19x-nvlink-endpt"
#define DEFAULT_LOOP_SLEEP_US	100
#define DEFAULT_LOOP_TIMEOUT_US	1000000

struct tegra_nvlink_device {
	/* base address of SYNC2X */
	void __iomem *nvlw_sync2x_base;

#ifdef CONFIG_DEBUG_FS
	/* This is the debugfs directory for the Tegra endpoint driver */
	struct dentry *tegra_debugfs;
#endif /* CONFIG_DEBUG_FS  */
	/* clocks */
	struct clk *clk_nvhs_pll0_mgmt;
	struct clk *clk_nvlink_sys;
	struct clk *clk_pllnvhs;
	struct clk *clk_txclk_ctrl;
	/* resets */
	struct reset_control *rst_nvhs_uphy_pm;
	struct reset_control *rst_nvhs_uphy;
	struct reset_control *rst_nvhs_uphy_pll0;
	struct reset_control *rst_nvhs_uphy_l0;
	struct reset_control *rst_nvhs_uphy_l1;
	struct reset_control *rst_nvhs_uphy_l2;
	struct reset_control *rst_nvhs_uphy_l3;
	struct reset_control *rst_nvhs_uphy_l4;
	struct reset_control *rst_nvhs_uphy_l5;
	struct reset_control *rst_nvhs_uphy_l6;
	struct reset_control *rst_nvhs_uphy_l7;
	/* Powergate id */
	int pgid_nvl;
	struct tegra_prod *prod_list;
};

struct tegra_nvlink_link {
	/* base address of MSSNVLINK */
	void __iomem *mssnvlink_0_base;
};

extern const struct file_operations t19x_nvlink_endpt_ops;

u32 nvlw_tioctrl_readl(struct nvlink_device *ndev, u32 reg);
void nvlw_tioctrl_writel(struct nvlink_device *ndev, u32 reg, u32 val);

u32 nvlw_nvlipt_readl(struct nvlink_device *ndev, u32 reg);
void nvlw_nvlipt_writel(struct nvlink_device *ndev, u32 reg, u32 val);

u32 nvlw_minion_readl(struct nvlink_device *ndev, u32 reg);
void nvlw_minion_writel(struct nvlink_device *ndev, u32 reg, u32 val);

u32 nvlw_nvl_readl(struct nvlink_device *ndev, u32 reg);
void nvlw_nvl_writel(struct nvlink_device *ndev, u32 reg, u32 val);

u32 nvlw_sync2x_readl(struct nvlink_device *ndev, u32 reg);
void nvlw_sync2x_writel(struct nvlink_device *ndev, u32 reg, u32 val);

u32 nvlw_nvltlc_readl(struct nvlink_device *ndev, u32 reg);
void nvlw_nvltlc_writel(struct nvlink_device *ndev, u32 reg, u32 val);

int wait_for_reg_cond_nvlink(
			struct nvlink_device *ndev,
			u32 reg,
			u32 bit,
			int bit_set,
			char *bit_name,
			u32 (*reg_readl)(struct nvlink_device *, u32),
			u32 *reg_val);

int minion_boot(struct nvlink_device *ndev);
int init_nvhs_phy(struct nvlink_device *ndev);

void nvlink_config_common_intr(struct nvlink_device *ndev);
void nvlink_enable_link_interrupts(struct nvlink_device *ndev);
void minion_service_falcon_intr(struct nvlink_device *ndev);
irqreturn_t t19x_nvlink_endpt_isr(int irq, void *dev_id);

int go_to_safe_mode(struct nvlink_device *ndev);
u32 t19x_nvlink_get_link_state(struct nvlink_device *ndev);
u32 t19x_nvlink_get_link_mode(struct nvlink_device *ndev);
u32 t19x_nvlink_get_sublink_mode(struct nvlink_device *ndev,
				bool is_rx_sublink);
void t19x_nvlink_get_tx_sublink_state(struct nvlink_device *ndev,
				u32 *tx_sublink_state);
void t19x_nvlink_get_rx_sublink_state(struct nvlink_device *ndev,
				u32 *rx_sublink_state);
int t19x_nvlink_poll_link_state(struct nvlink_device *ndev, u32 link_state,
				u32 timeout);
int t19x_nvlink_poll_tx_sublink_state(struct nvlink_device *ndev,
				u32 tx_sublink_state, u32 timeout);
int t19x_nvlink_poll_rx_sublink_state(struct nvlink_device *ndev,
				u32 rx_sublink_state, u32 timeout);
int t19x_nvlink_poll_sublink_state(struct nvlink_device *ndev0,
				u32 tx_sublink_state,
				struct nvlink_device *ndev1,
				u32 rx_sublink_state,
				u32 timeout);
int t19x_nvlink_set_sublink_mode(struct nvlink_device *ndev, bool is_rx_sublink,
				u32 mode);
int t19x_nvlink_set_link_mode(struct nvlink_device *ndev, u32 mode);
int nvlink_train_intranode_conn_to_hs(struct nvlink_intranode_conn *conn);
void nvlink_enable_AN0_packets(struct nvlink_device *ndev);

#ifdef CONFIG_DEBUG_FS
void t19x_nvlink_endpt_debugfs_init(struct nvlink_device *ndev);
void t19x_nvlink_endpt_debugfs_deinit(struct nvlink_device *ndev);
#else
static inline void t19x_nvlink_endpt_debugfs_init(struct nvlink_device *ndev) {}
static inline void t19x_nvlink_endpt_debugfs_deinit(
						struct nvlink_device *ndev) {}
#endif /* CONFIG_DEBUG_FS  */

#endif /* T19X_NVLINK_ENDPT_H */
