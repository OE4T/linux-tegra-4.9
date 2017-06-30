/*
 * drivers/video/tegra/host/nvcsi/nvcsi.h
 *
 * Tegra Graphics Host NVCSI
 *
 * Copyright (c) 2015-2017 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_NVCSI_H__
#define __NVHOST_NVCSI_H__

#include <media/csi.h>

/* STREAM REGISTERS */
#define NVCSI_STREAM_0_ERROR_STATUS2VI_MASK		0x10090
#define NVCSI_STREAM_1_ERROR_STATUS2VI_MASK		0x10890
#define CFG_ERR_STATUS2VI_MASK_VC3			(0x1 << 24)
#define CFG_ERR_STATUS2VI_MASK_VC2			(0x1 << 16)
#define CFG_ERR_STATUS2VI_MASK_VC1			(0x1 << 8)
#define CFG_ERR_STATUS2VI_MASK_VC0			(0x1 << 0)
/* PHY REGISTERS */
/* PHY INTERRUPTS REGISTERS */
#define NVCSI_PHY_0_CILA_INTR_STATUS			0x10400
#define intr_dphy_cil_deskew_calib_err_ctrl		(1 << 27)
#define intr_dphy_cil_deskew_calib_err_lane1		(1 << 26)
#define intr_dphy_cil_deskew_calib_err_lane0		(1 << 25)
#define intr_dphy_cil_deskew_calib_done_ctrl		(1 << 24)
#define intr_dphy_cil_deskew_calib_done_lane1		(1 << 23)
#define intr_dphy_cil_deskew_calib_done_lane0		(1 << 22)
#define NVCSI_PHY_0_CILA_INTR_MASK			0x10404
#define NVCSI_PHY_0_CILA_ERR_INTR_STATUS		0x10408
#define NVCSI_PHY_0_CILA_ERR_INTR_MASK			0x1040c

#define NVCSI_PHY_0_CILB_INTR_STATUS			0x10c00
#define NVCSI_PHY_0_CILB_INTR_MASK			0x10c04
#define NVCSI_PHY_0_CILB_ERR_INTR_STATUS		0x10c08
#define NVCSI_PHY_0_CILB_ERR_INTR_MASK			0x10c0c


/* CIL REGISTERS
 * XXX_OFFSET: address offset from NVCSI_CIL_PHY_CTRL_0
 */
#define NVCSI_PHY_0_NVCSI_CIL_PHY_CTRL_0		0x18000
#define NVCSI_CIL_A_SW_RESET_0_OFFSET			0x18
#define NVCSI_CIL_A_CLK_DESKEW_CTRL_0_OFFSET            0x2c
#define NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET		0x24
#define SW_SET_DPHY_INADJ_CLK				(0x1 << 22)
#define DPHY_INADJ_CLK					(0x3f << 16)
#define DPHY_INADJ_CLK_SHIFT				16
#define SW_SET_DPHY_INADJ_IO1				(0x1 << 14)
#define DPHY_INADJ_IO1					(0x3f << 8)
#define DPHY_INADJ_IO1_SHIFT				8
#define SW_SET_DPHY_INADJ_IO0				(0x1 << 6)
#define DPHY_INADJ_IO0					0x3f
#define DPHY_INADJ_IO0_SHIFT				0
#define CLK_INADJ_SWEEP_CTRL				(0x1 << 15)
#define NVCSI_CIL_A_DATA_DESKEW_CTRL_0_OFFSET		0x30
#define DATA_INADJ_SWEEP_CTRL1				(0x1 << 31)
#define DATA_INADJ_SWEEP_CTRL0				(0x1 << 15)
#define NVCSI_CIL_A_DPHY_DESKEW_STATUS_0_OFFSET		0x34
#define DPHY_CALIB_ERR_IO1				(0x1 << 15)
#define DPHY_CALIB_DONE_IO1				(0x1 << 14)
#define DPHY_CALIB_ERR_IO0				(0x1 << 7)
#define DPHY_CALIB_DONE_IO0				(0x1 << 6)
#define NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_LOW_0_0_OFFSET	0x38
#define NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_HIGH_0_0_OFFSET	0x3c
#define NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_LOW_1_0_OFFSET	0x40
#define NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_HIGH_1_0_OFFSET	0x44
#define NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_LOW_0_0_OFFSET		0x48
#define NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_HIGH_0_0_OFFSET	0x4c
#define NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_LOW_1_0_OFFSET		0x50
#define NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_HIGH_1_0_OFFSET	0x54

#define NVCSI_CIL_A_CONTROL_0_OFFSET			0x5c
#define	DESKEW_COMPARE				(0xf << 20)
#define DESKEW_COMPARE_SHIFT			20
#define DESKEW_SETTLE				(0xf << 16)
#define DESKEW_SETTLE_SHIFT			16
#define CLK_SETTLE				(0x3f << 8)
#define CLK_SETTLE_SHIFT			8
#define THS_SETTLE				0x7f
#define THS_SETTLE_SHIFT			0

#define NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET		0x88
#define NVCSI_CIL_B_CLK_DESKEW_CTRL_0_OFFSET            0x90
#define NVCSI_CIL_B_DATA_DESKEW_CTRL_0_OFFSET		0x94
#define NVCSI_CIL_B_DPHY_DESKEW_STATUS_0_OFFSET		0x98
#define NVCSI_CIL_B_CONTROL_0_OFFSET			0xc0

#define NVCSI_PHY_OFFSET			0x10000
#define NVCSI_CIL_B_OFFSET			0x64
extern const struct file_operations tegra_nvcsi_ctrl_ops;

struct nvcsi {
	struct platform_device *pdev;
	struct regulator *regulator;
	struct tegra_csi_device csi;
	struct dentry *dir;
	struct mutex deskew_lock;
	int irq;
};

int nvcsi_finalize_poweron(struct platform_device *pdev);
int nvcsi_prepare_poweroff(struct platform_device *pdev);
int nvcsi_deskew_apply_check(unsigned int active_lanes);
int nvcsi_deskew_setup(unsigned int active_lanes);

struct tegra_csi_device *tegra_get_mc_csi(void);
#endif
