/*
 * PVA mailbox header
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#ifndef __PVA_MAILBOX_H__
#define __PVA_MAINBOX_H__

/* Total CCQ status registers */
#define PVA_CCQ_STATUS_REGS	9

/* Symbolic definitions of the CCQ status registers */
#define PVA_CCQ_STATUS0_INDEX	0
#define PVA_CCQ_STATUS1_INDEX	1
#define PVA_CCQ_STATUS2_INDEX	2
#define PVA_CCQ_STATUS3_INDEX	3
#define PVA_CCQ_STATUS4_INDEX	4
#define PVA_CCQ_STATUS5_INDEX	5
#define PVA_CCQ_STATUS6_INDEX	6
#define PVA_CCQ_STATUS7_INDEX	7
#define PVA_CCQ_STATUS8_INDEX	8

struct pva_mbox_status {
	uint32_t	cmd;
	uint32_t	error;
	uint32_t	status[PVA_CCQ_STATUS_REGS];
};

#define VALID_MB_INPUT_REGS 4

u32 pva_send_mbox_cmd(struct platform_device *pdev,
				struct pva_cmd *cmd, u32 nregs);
void pva_mbox_poll_status(struct platform_device *pdev);
u32 pva_read_mbox_status(struct platform_device *pdev,
			int int_status,
			struct pva_mbox_status *mb_status);

#endif /*__PVA_MAINBOX_H__*/
