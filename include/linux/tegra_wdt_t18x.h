/*
 * tegra186_wdt_t18x.h - Definitions to change T186 watchdog state
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA186_WDT_T18X_H__
#define __TEGRA186_WDT_T18X_H__

#ifdef CONFIG_TEGRA18X_WATCHDOG
extern void tegra_wdt_t18x_debug_reset(bool state);
extern void tegra_wdt_t18x_por_reset(bool state);
extern void tegra_wdt_t18x_disable_all(void);
#else
static inline void tegra_wdt_t18x_debug_reset(bool state) { return; }
static inline void tegra_wdt_t18x_por_reset(bool state) { return; }
static inline void tegra_wdt_t18x_disable_all(void) { return; }
#endif

#endif
