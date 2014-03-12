/*
 * GK20A Debug functionality
 *
 * Copyright (C) 2011-2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _DEBUG_GK20A_H_
#define _DEBUG_GK20A_H_

extern unsigned int gk20a_debug_trace_cmdbuf;

void gk20a_debug_dump(struct platform_device *pdev);
void gk20a_debug_init(struct platform_device *pdev);

#endif
