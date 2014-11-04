/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_TEGRA_HSP_H
#define _LINUX_TEGRA_HSP_H

enum tegra_hsp_db_master {
	HSP_DB_MASTER_CCPLEX,
	HSP_DB_MASTER_DPMU,
	HSP_DB_MASTER_BPMP,
	HSP_DB_MASTER_SPE,
	HSP_DB_MASTER_SCE = 9,
	HSP_DB_MASTER_APE,
	HSP_DB_NR_MASTERS,
};

enum tegra_hsp_doorbell {
	HSP_DB_DPMU,
	HSP_DB_CCPLEX,
	HSP_DB_CCPLEX_TZ,
	HSP_DB_BPMP,
	HSP_DB_SPE,
	HSP_DB_SCE,
	HSP_DB_APE,
	HSP_NR_DBS,
};

enum tegra_hsp_init_status {
	HSP_INIT_PENDING,
	HSP_INIT_FAILED,
	HSP_INIT_OKAY,
};

int tegra_hsp_db_enable_master(enum tegra_hsp_db_master master);

int tegra_hsp_db_ring(enum tegra_hsp_doorbell dbell);

int tegra_hsp_db_enabled(enum tegra_hsp_doorbell dbell);

int tegra_hsp_db_listen(irq_handler_t callback);

u32 tegra_hsp_db_get_pending(void);

void tegra_hsp_db_clr_pending(u32 mask);

enum tegra_hsp_init_status tegra_hsp_get_init_status(void);

#define tegra_hsp_db_pending(mask, master)	((mask) & (1 << (master)))

#endif
