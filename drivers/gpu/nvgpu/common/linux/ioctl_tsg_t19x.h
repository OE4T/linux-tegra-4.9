/*
 * GV11B TSG IOCTL handler
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _NVGPU_IOCTL_TSG_T19X
#define _NVGPU_IOCTL_TSG_T19X

int t19x_tsg_ioctl_handler(struct gk20a *g, struct tsg_gk20a *tsg,
				unsigned int cmd, u8 *arg);
#endif
