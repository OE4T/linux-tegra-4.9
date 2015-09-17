 /*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/reboot.h>
#include <linux/of.h>
#include <linux/bitops.h>

#include <asm/io.h>

#include "iomap.h"

#define NV_ADDRESS_MAP_PMC_IMPL_BASE	0x0c360000
#define PMC_IMPL_CNTRL_0		(NV_ADDRESS_MAP_PMC_IMPL_BASE + 0x0)
#define NV_ADDRESS_MAP_SCRATCH_BASE	0x0c390000
#define SCRATCH_SECURE_BL_SCRATCH_0	(NV_ADDRESS_MAP_SCRATCH_BASE + 0x0)

#define NEVER_RESET		0
#define RECOVERY_MODE		BIT(31)
#define BOOTLOADER_MODE		BIT(30)
#define FORCED_RECOVERY_MODE	BIT(1)

#define SYS_RST_OK		1

static int program_reboot_reason(const char *cmd)
{
	void __iomem *scratch = ioremap(SCRATCH_SECURE_BL_SCRATCH_0, 0x1000);
	u32 reg;

	/* clean up */
	reg = readl_relaxed(scratch);
	reg &= ~(BOOTLOADER_MODE | RECOVERY_MODE | FORCED_RECOVERY_MODE);
	writel_relaxed(reg, scratch);

	/* valid command? */
	if (!cmd || (strlen(cmd) == 0))
		return SYS_RST_OK;

	/* Writing recovery kernel or Bootloader mode in SCRATCH0 31:30:1 */
	if (!strcmp(cmd, "recovery"))
		reg |= RECOVERY_MODE;
	else if (!strcmp(cmd, "bootloader"))
		reg |= BOOTLOADER_MODE;
	else if (!strcmp(cmd, "forced-recovery"))
		reg |= FORCED_RECOVERY_MODE;

	/* write the restart command */
	writel_relaxed(reg, scratch);

	return 0;
}

static int tegra_restart_notify(struct notifier_block *nb,
			    unsigned long action, void *data)
{
	const char *cmd = (char *)data;
	void __iomem *pmc = ioremap(PMC_IMPL_CNTRL_0, 0x1000);
	u32 reg;

	/*
	 * program reboot reason for the bootloader
	 * remove this once we have got ammendment to PSCI
	 * to pass reset reason
	 */
	program_reboot_reason(cmd);

	/* write reset, remove this from here once DMCE issue is fixed */
	reg = readl_relaxed(pmc);
	reg |= (1 << 4);
	writel_relaxed(reg, pmc);
	/* read barrier */
	readl_relaxed(pmc);

	return NOTIFY_OK;
};

static struct notifier_block tegra_restart_nb = {
	.notifier_call = tegra_restart_notify,
	.priority = 129, /* greater than default priority */
};

static int tegra_register_restart_notifier(void)
{
	pr_info("Tegra restart notifier registered.\n");
	return register_restart_handler(&tegra_restart_nb);
}
arch_initcall(tegra_register_restart_notifier);
