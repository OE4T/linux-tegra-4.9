/*
 * Copyright (c) 2014-2016 NVIDIA CORPORATION.  All rights reserved.
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

enum tegra_hsp_master {
	HSP_FIRST_MASTER = 1,

	/* secure */
	HSP_MASTER_SECURE_CCPLEX = HSP_FIRST_MASTER,
	HSP_MASTER_SECURE_DPMU,
	HSP_MASTER_SECURE_BPMP,
	HSP_MASTER_SECURE_SPE,
	HSP_MASTER_SECURE_SCE,
	HSP_MASTER_SECURE_DMA,
	HSP_MASTER_SECURE_TSECA,
	HSP_MASTER_SECURE_TSECB,
	HSP_MASTER_SECURE_JTAGM,
	HSP_MASTER_SECURE_CSITE,
	HSP_MASTER_SECURE_APE,

	/* non-secure */
	HSP_MASTER_CCPLEX = HSP_FIRST_MASTER + 16,
	HSP_MASTER_DPMU,
	HSP_MASTER_BPMP,
	HSP_MASTER_SPE,
	HSP_MASTER_SCE,
	HSP_MASTER_DMA,
	HSP_MASTER_TSECA,
	HSP_MASTER_TSECB,
	HSP_MASTER_JTAGM,
	HSP_MASTER_CSITE,
	HSP_MASTER_APE,

	HSP_LAST_MASTER = HSP_MASTER_APE,
};

enum tegra_hsp_doorbell {
	HSP_FIRST_DB = 0,
	HSP_DB_DPMU = HSP_FIRST_DB,
	HSP_DB_CCPLEX,
	HSP_DB_CCPLEX_TZ,
	HSP_DB_BPMP,
	HSP_DB_SPE,
	HSP_DB_SCE,
	HSP_DB_APE,
	HSP_LAST_DB = HSP_DB_APE,
	HSP_NR_DBS,
};

typedef void (*db_handler_t)(void *data);

int tegra_hsp_init(void);

int tegra_hsp_db_enable_master(enum tegra_hsp_master master);

int tegra_hsp_db_get_enabled_masters(void);

int tegra_hsp_db_ring(enum tegra_hsp_doorbell dbell);

int tegra_hsp_db_can_ring(enum tegra_hsp_doorbell dbell);

int tegra_hsp_db_add_handler(int master, db_handler_t handler, void *data);

int tegra_hsp_db_del_handler(int master);

int tegra_hsp_db_get_pending(void);

int tegra_hsp_db_clr_pending(u32 mask);

int tegra_hsp_db_get_raw(void);

int tegra_hsp_db_clr_raw(u32 mask);

#define tegra_hsp_find_master(mask, master)	((mask) & (1 << (master)))

#endif
