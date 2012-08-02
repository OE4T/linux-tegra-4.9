/*
 * drivers/video/tegra/host/t124/syncpt_t124.h
 *
 * Tegra Graphics Host Syncpoints for T124
 *
 * Copyright (c) 2011, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __NVHOST_SYNCPT_T124_H
#define __NVHOST_SYNCPT_T124_H

#define NVSYNCPT_VI_0_0   1
#define NVSYNCPT_VI_0_1   2
#define NVSYNCPT_VI_0_2   3
#define NVSYNCPT_VI_0_3   4
#define NVSYNCPT_VI_0_4   5
#define NVSYNCPT_VI_1_0   6
#define NVSYNCPT_VI_1_1   7
#define NVSYNCPT_VI_1_2   8
#define NVSYNCPT_VI_1_3   9
#define NVSYNCPT_VI_1_4  10
#define NVSYNCPT_ISP_0_0 11
#define NVSYNCPT_ISP_0_1 12
#define NVSYNCPT_ISP_0_2 13
#define NVSYNCPT_ISP_0_3 14
#define NVSYNCPT_ISP_1_0 15
#define NVSYNCPT_ISP_1_1 16
#define NVSYNCPT_ISP_1_2 17
#define NVSYNCPT_ISP_1_3 18
#define NVSYNCPT_TSEC	 19
#define NVSYNCPT_3D	 20
#define NVSYNCPT_MSENC	 21
#define NVSYNCPT_DISP0	 22
#define NVSYNCPT_DISP1	 23
#define NVSYNCPT_VBLANK0 24
#define NVSYNCPT_VBLANK1 25
#define NVSYNCPT_DSI     26

#ifdef CONFIG_ARCH_TEGRA_VIC
#define NVSYNCPT_VIC     27
#endif


#define NVSYNCPT_GK20A_BASE 32
/* following is base + number of gk20a channels. TODO: remove magic */
#define NVSYNCPT_GK20A_LAST (NVSYNCPT_GK20A_BASE + 127)


#define NV_VI_0_SYNCPTS ( \
	BIT(NVSYNCPT_VI_0_0) | \
	BIT(NVSYNCPT_VI_0_1) | \
	BIT(NVSYNCPT_VI_0_2) | \
	BIT(NVSYNCPT_VI_0_3) | \
	BIT(NVSYNCPT_VI_0_4) | \
	0 )

#define NV_VI_1_SYNCPTS ( \
	BIT(NVSYNCPT_VI_1_0) | \
	BIT(NVSYNCPT_VI_1_1) | \
	BIT(NVSYNCPT_VI_1_2) | \
	BIT(NVSYNCPT_VI_1_3) | \
	BIT(NVSYNCPT_VI_1_4) | \
	0 )

#define NV_ISP_0_SYNCPTS ( \
	BIT(NVSYNCPT_ISP_0_0) | \
	BIT(NVSYNCPT_ISP_0_1) | \
	BIT(NVSYNCPT_ISP_0_2) | \
	BIT(NVSYNCPT_ISP_0_3) | \
	0 )

#define NV_ISP_1_SYNCPTS ( \
	BIT(NVSYNCPT_ISP_1_0) | \
	BIT(NVSYNCPT_ISP_1_1) | \
	BIT(NVSYNCPT_ISP_1_2) | \
	BIT(NVSYNCPT_ISP_1_3) | \
	0 )


#define NVCAMERA_MANAGED_SYNCPTS ( \
	NV_VI_0_SYNCPTS  | NV_VI_1_SYNCPTS  | \
	NV_ISP_0_SYNCPTS | NV_ISP_1_SYNCPTS | \
	0 )

/* sync points that are wholly managed by the client */
#define NVSYNCPTS_CLIENT_MANAGED ( \
	BIT(NVSYNCPT_DISP0) | BIT(NVSYNCPT_DISP1) | \
	BIT(NVSYNCPT_DSI) |			    \
	NVCAMERA_MANAGED_SYNCPTS | \
   0 )


#define NVWAITBASE_3D   (3)
#define NVWAITBASE_MSENC  (4)
#define NVWAITBASE_TSEC   (5)

int nvhost_t124_init_syncpt(struct nvhost_master *host);

#endif /* __NVHOST_SYNCPT_T124_H */
