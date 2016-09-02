/*
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

#include <linux/platform_device.h>
#include "dev.h"
#include "pva_regs.h"

#define PVA_CMD_NOOP_MIN_REG_INDEX 3
#define PVA_CMD_NOOP_MAX_REG_INDEX 7

/*
 * Process the CMD_NOOP command.
 */
static int
process_noop(const struct pva_mbox_status *const mb_status)
{
	int err = 0;
	u32 flags = mb_status->cmd & PVA_MASK(30, 27);
	u32 reg =  PVA_GET_SUBCOMMAND(mb_status->cmd, uint8_t);


	if (flags & PVA_CMD_FL_NOOP_ERROR) {
		nvhost_dbg_info("noop cmd error: 0x%x",
					mb_status->error);
	}

	if (flags & PVA_CMD_FL_NOOP_ECHO) {
		if ((reg < PVA_CMD_NOOP_MIN_REG_INDEX) ||
			(reg > PVA_CMD_NOOP_MAX_REG_INDEX)) {
			nvhost_dbg_info("Wrong noop subcmd: 0x%x", reg);
		} else {
			nvhost_dbg_info("noop cmd echo data: 0x%x",
					mb_status->status[reg]);
		}
	}

	return err;
}


struct pva_status_lookup	pva_noop_cmnd = {
	.process_mbox	= process_noop,
};
