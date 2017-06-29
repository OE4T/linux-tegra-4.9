/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef CLK_ARB_GP106_H
#define CLK_ARB_GP106_H

u32 gp106_get_arbiter_clk_domains(struct gk20a *g);
int gp106_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz);
int gp106_get_arbiter_clk_default(struct gk20a *g, u32 api_domain,
		u16 *default_mhz);

#endif /* CLK_ARB_GP106_H */
