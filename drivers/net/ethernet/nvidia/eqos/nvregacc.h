/*
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
 */
#ifndef __NV__REGACC__H__

#define __NV__REGACC__H__

#define CLK_CRTL0_RgOffAddr ((volatile ULONG *)(BASE_ADDRESS + 0x8000))

#define CLK_CRTL0_RgWr(data) do {\
		iowrite32(data, (void *)CLK_CRTL0_RgOffAddr);\
} while(0)

#define CLK_CRTL0_RgRd(data) do {\
		(data) = ioread32((void *)CLK_CRTL0_RgOffAddr);\
} while(0)

/*#define  CLK_CRTL0_TX_CLK_Mask (ULONG)(~(~0<<(1))) << (30)))*/

#define CLK_CRTL0_TX_CLK_Mask (ULONG)(0x1)

/*#define CLK_CRTL0_TX_CLK_Wr_Mask (unsigned long)(~((~(~0 << (1))) << (30)))*/

#define CLK_CRTL0_TX_CLK_Wr_Mask (ULONG)(0xbfffffff)

#define CLK_CRTL0_TX_CLK_UdfWr(data) do {\
		ULONG v;\
		CLK_CRTL0_RgRd(v);\
		v = ((v & CLK_CRTL0_TX_CLK_Wr_Mask) | ((data & CLK_CRTL0_TX_CLK_Mask)<<30));\
		CLK_CRTL0_RgWr(v);\
} while(0)

#define CLK_CRTL0_TX_CLK_UdfRd(data) do {\
		CLK_CRTL0_RgRd(data);\
		data = ((data >> 30) & CLK_CRTL0_TX_CLK_Mask);\
} while(0)

#endif
