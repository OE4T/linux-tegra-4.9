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

/* Number of valid MBOX registers used for sending commands */
#define VALID_MB_INPUT_REGS 4

extern struct pva_status_lookup pva_noop_cmnd;
extern struct pva_status_lookup pva_cmnd_nosupport;

/**
 * struct pva_mbox_status - Handle the MBOX status based on ISR
 *
 * @cmd:		Holds the current MBOX command
 * @error:		Holds the any error shown through ISR
 * @status:		Holds the status of all CCQ registers
 *
 */
struct pva_mbox_status {
	uint32_t	cmd;
	uint32_t	error;
	uint32_t	status[PVA_CCQ_STATUS_REGS];
};

/**
 * struct pva_status_lookup - Process the MBOX status
 *
 * @process_mbox_status:	Callback function to process the CCQ status
 *
 */
struct pva_status_lookup {
	int	(*process_mbox)(const struct pva_mbox_status *const mb_status);
};

/**
 * pva_send_mbox_cmd() - Functions to send MBOX commands
 *
 * @pdev:	Pointer to PVA device
 * @pva_cmd:	Pointer to the pva command struct
 * @nregs:	Number of valid mailbox registers for the command
 *
 * Return:	0 on Success or negative error code
 *
 * This function called by OS to pass the mailbox commands to
 * the PVA uCode.
 */
int pva_send_mbox_cmd(struct platform_device *pdev,
				struct pva_cmd *cmd, u32 nregs);

/**
 * pva_mbox_poll_status() - Functions to poll the mailbox status
 *
 * @pdev:	Pointer to PVA device
 *
 * Return:	0 on Success or negative error code
 *
 * This function can be used by kernel driver to poll the mbox status
 * register to check whether the commands is being processed or not.
 */
void pva_mbox_poll_status(struct platform_device *pdev);

/**
 * pva_read_mbox_status() - Functions to read the mailbox status registers
 *
 * @pdev:	Pointer to PVA device
 * @int_status:	Interrupt status value read from ISR
 * @mb_status:	Pointer to pva_mbox_status struct
 *
 * Return:	0 on Success or negative error code
 *
 * This function is used to read the CCQ status registers based on
 * the status set in mbox7 by the PVA uCode.
 */
int pva_read_mbox_status(struct platform_device *pdev,
			int int_status,
			struct pva_mbox_status *mb_status);

/**
 * pva_process_mbox_status() - Functions to Process the status registers
 *
 * @pdev:	Pointer to PVA device
 * @mb_status:	Pointer to pva_mbox_status struct
 *
 * Return:	0 on Success or negative error code
 *
 * This function is used to intepret the CCQ status register value based on
 * command being passed to the PVA uCode.
 */
int
pva_process_mbox_status(struct platform_device *pdev,
			const struct pva_mbox_status *const mb_status);

#endif /*__PVA_MAINBOX_H__*/
