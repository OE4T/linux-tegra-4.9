/*
 * GP10B Graphics Context
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __GR_CTX_GM10B_H__
#define __GR_CTX_GM10B_H__

#include "gk20a/gr_ctx_gk20a.h"

/* production netlist, one and only one from below */
#define GP10B_NETLIST_IMAGE_FW_NAME GK20A_NETLIST_IMAGE_A

int gr_gp10b_get_netlist_name(struct gk20a *g, int index, char *name);
bool gr_gp10b_is_firmware_defined(void);

#endif /*__GR_CTX_GP10B_H__*/
