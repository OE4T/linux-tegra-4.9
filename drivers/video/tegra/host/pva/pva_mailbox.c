/*
 * PVA mailbox code
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

#include <linux/export.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "dev.h"
#include "nvhost_acm.h"
#include "pva_regs.h"

/* Change this define as more cmd get supported */
#define SUPPORTED_CMD 1
static struct pva_status_lookup *commands[SUPPORTED_CMD] = {
	[CMD_NOOP]			= &pva_noop_cmnd
};

static u32 pva_get_mb_reg(u32 i)
{
	u32 mb_reg[VALID_MB_INPUT_REGS] = {
		hsp_sm0_r(),
		hsp_sm1_r(),
		hsp_sm2_r(),
		hsp_sm3_r()
	};

	return mb_reg[i];
}

/* Function to notify unsupported commands */
static int mbox_cmd_nosupport(struct platform_device *pdev,
			const struct pva_mbox_status *const mb_status)
{
	nvhost_err(&pdev->dev, "mbox cmd 0x%x missing code support\n",
			PVA_GET_COMMAND(mb_status->cmd));
	return -EINVAL;
}


int pva_send_mbox_cmd(struct platform_device *pdev,
				struct pva_cmd *cmd, u32 nregs)
{
	u32	reg, status;
	s32	i;
	int	err = 0;

	if (nregs > VALID_MB_INPUT_REGS) {
		pr_err("%s nregs %d more than expected\n", __func__, nregs);
		return -EINVAL;
	}

	/*
	 * Make sure the state is what we expect it to be.
	 */
	status = host1x_readl(pdev, hsp_sm7_r());

	WARN_ON((status & PVA_INT_PENDING));
	WARN_ON((status & PVA_READY) == 0);
	WARN_ON((status & PVA_BUSY));

	/*
	 * Write all of the other command mailbox
	 * registers before writing mailbox 0.
	 */
	for (i = (nregs - 1); i >= 0; i--) {
		reg = pva_get_mb_reg(i);
		host1x_writel(pdev, reg, cmd->mbox[i]);
	}

	return err;
}

void pva_mbox_poll_status(struct platform_device *pdev)
{
	u32 status = host1x_readl(pdev, hsp_sm7_r());

	/* check for the int_pending bit to set */
	while ((status & PVA_BUSY) || (!(status & PVA_INT_PENDING))) {
		usleep_range(10, 20);
		status = host1x_readl(pdev, hsp_sm7_r());
	}
	WARN_ON((status & PVA_READY) == 0);
	/* check whether PVA_BUSY is cleared */
	WARN_ON((status & PVA_BUSY));

}

int pva_read_mbox_status(struct platform_device *pdev,
			int int_status,
			struct pva_mbox_status *mb_status)
{
	u32 clear_status = PVA_INT_PENDING;

	WARN_ON((int_status & PVA_INT_PENDING) == 0);

	/* save the current command and subcommand for later processing */
	mb_status->cmd = host1x_readl(pdev, hsp_sm0_r());

	/* Get all the valid status register data */
	if (int_status & PVA_VALID_STATUS3) {
		mb_status->status[PVA_CCQ_STATUS3_INDEX] =
				 host1x_readl(pdev, cfg_ccq_status3_r());
		clear_status |= PVA_VALID_STATUS3;
		mb_status->error = PVA_GET_ERROR_CODE(
				mb_status->status[PVA_CCQ_STATUS3_INDEX]);
	}

	if (int_status & PVA_VALID_STATUS4) {
		mb_status->status[PVA_CCQ_STATUS4_INDEX] =
				 host1x_readl(pdev, cfg_ccq_status4_r());
		clear_status |= PVA_VALID_STATUS4;
	}

	if (int_status & PVA_VALID_STATUS5) {
		mb_status->status[PVA_CCQ_STATUS5_INDEX] =
				 host1x_readl(pdev, cfg_ccq_status5_r());
		clear_status |= PVA_VALID_STATUS5;
	}

	if (int_status & PVA_VALID_STATUS6) {
		mb_status->status[PVA_CCQ_STATUS6_INDEX] =
				 host1x_readl(pdev, cfg_ccq_status6_r());
		clear_status |= PVA_VALID_STATUS6;
	}

	if (int_status & PVA_VALID_STATUS7) {
		mb_status->status[PVA_CCQ_STATUS7_INDEX] =
				 host1x_readl(pdev, cfg_ccq_status7_r());
		clear_status |= PVA_VALID_STATUS7;
	}

	if (int_status & PVA_CMD_COMPLETE)
		clear_status |= PVA_CMD_COMPLETE;

	if (int_status & PVA_CMD_ERROR)
		clear_status |= PVA_CMD_ERROR;

	/* Acknowledge the interrupt */
	host1x_writel(pdev, hsp_sm7_r(), (int_status & ~clear_status));

	return int_status;
}

int
pva_process_mbox_status(struct platform_device *pdev,
		const struct pva_mbox_status *const mb_status)
{
	const struct pva_status_lookup *lookup;
	u32 cmd = PVA_GET_COMMAND(mb_status->cmd);
	int err = -EINVAL;

	if (cmd < SUPPORTED_CMD) {
		lookup = commands[cmd];
		if (lookup &&
			(lookup->process_mbox != NULL)) {
			err = lookup->process_mbox(mb_status);
		}
	} else {
		err = mbox_cmd_nosupport(pdev, mb_status);
	}

	return err;
}
