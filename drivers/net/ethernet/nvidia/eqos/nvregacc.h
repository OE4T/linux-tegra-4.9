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

#define VIRT_INTR_CH0_CRTL0_RgOffAddr ((volatile ULONG *)(BASE_ADDRESS + 0x8600))

#define VIRT_INTR_CH0_CRTL0_RgWr(data) do {\
		iowrite32(data, (void *)VIRT_INTR_CH0_CRTL0_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH0_CRTL0_RgRd(data) do {\
		(data) = ioread32((void *)VIRT_INTR_CH0_CRTL0_RgOffAddr);\
} while(0)

/*#define  VIRT_INTR_CH0_CRTL0_TX_Mask (ULONG)(~(~0<<(1))) << (0)))*/

#define VIRT_INTR_CH0_CRTL0_TX_Mask (ULONG)(0x1)

/*#define VIRT_INTR_CH0_CRTL0_TX_Wr_Mask (unsigned long)(~((~(~0 << (1))) << (0)))*/

#define VIRT_INTR_CH0_CRTL0_TX_Wr_Mask (ULONG)(0xfffffffe)

#define VIRT_INTR_CH0_CRTL0_TX_UdfWr(data) do {\
		ULONG v;\
		VIRT_INTR_CH0_CRTL0_RgRd(v);\
		v = ((v & VIRT_INTR_CH0_CRTL0_TX_Wr_Mask) | ((data & VIRT_INTR_CH0_CRTL0_TX_Mask)<<0));\
		VIRT_INTR_CH0_CRTL0_RgWr(v);\
} while(0)

#define VIRT_INTR_CH0_CRTL0_TX_UdfRd(data) do {\
		VIRT_INTR_CH0_CRTL0_RgRd(data);\
		data = ((data >> 0) & VIRT_INTR_CH0_CRTL0_TX_Mask);\
} while(0)

/*#define  VIRT_INTR_CH0_CRTL0_RX_Mask (ULONG)(~(~0<<(1))) << (1)))*/

#define VIRT_INTR_CH0_CRTL0_RX_Mask (ULONG)(0x1)

/*#define VIRT_INTR_CH0_CRTL0_RX_Wr_Mask (unsigned long)(~((~(~0 << (1))) << (1)))*/

#define VIRT_INTR_CH0_CRTL0_RX_Wr_Mask (ULONG)(0xfffffffd)

#define VIRT_INTR_CH0_CRTL0_RX_UdfWr(data) do {\
		ULONG v;\
		VIRT_INTR_CH0_CRTL0_RgRd(v);\
		v = ((v & VIRT_INTR_CH0_CRTL0_RX_Wr_Mask) | ((data & VIRT_INTR_CH0_CRTL0_RX_Mask)<<1));\
		VIRT_INTR_CH0_CRTL0_RgWr(v);\
} while(0)

#define VIRT_INTR_CH0_CRTL0_RX_UdfRd(data) do {\
		VIRT_INTR_CH0_CRTL0_RgRd(data);\
		data = ((data >> 1) & VIRT_INTR_CH0_CRTL0_RX_Mask);\
} while(0)


#define VIRT_INTR_CH1_CRTL0_RgOffAddr ((volatile ULONG *)(BASE_ADDRESS + 0x8608))

#define VIRT_INTR_CH1_CRTL0_RgWr(data) do {\
		iowrite32(data, (void *)VIRT_INTR_CH1_CRTL0_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH1_CRTL0_RgRd(data) do {\
		(data) = ioread32((void *)VIRT_INTR_CH1_CRTL0_RgOffAddr);\
} while(0)

/*#define  VIRT_INTR_CH1_CRTL0_TX_Mask (ULONG)(~(~0<<(1))) << (0)))*/

#define VIRT_INTR_CH1_CRTL0_TX_Mask (ULONG)(0x1)

/*#define VIRT_INTR_CH1_CRTL0_TX_Wr_Mask (unsigned long)(~((~(~0 << (1))) << (0)))*/

#define VIRT_INTR_CH1_CRTL0_TX_Wr_Mask (ULONG)(0xfffffffe)

#define VIRT_INTR_CH1_CRTL0_TX_UdfWr(data) do {\
		ULONG v;\
		VIRT_INTR_CH1_CRTL0_RgRd(v);\
		v = ((v & VIRT_INTR_CH1_CRTL0_TX_Wr_Mask) | ((data & VIRT_INTR_CH1_CRTL0_TX_Mask)<<0));\
		VIRT_INTR_CH1_CRTL0_RgWr(v);\
} while(0)

#define VIRT_INTR_CH1_CRTL0_TX_UdfRd(data) do {\
		VIRT_INTR_CH1_CRTL0_RgRd(data);\
		data = ((data >> 0) & VIRT_INTR_CH1_CRTL0_TX_Mask);\
} while(0)

/*#define  VIRT_INTR_CH1_CRTL0_RX_Mask (ULONG)(~(~0<<(1))) << (1)))*/

#define VIRT_INTR_CH1_CRTL0_RX_Mask (ULONG)(0x1)

/*#define VIRT_INTR_CH1_CRTL0_RX_Wr_Mask (unsigned long)(~((~(~0 << (1))) << (1)))*/

#define VIRT_INTR_CH1_CRTL0_RX_Wr_Mask (ULONG)(0xfffffffd)

#define VIRT_INTR_CH1_CRTL0_RX_UdfWr(data) do {\
		ULONG v;\
		VIRT_INTR_CH1_CRTL0_RgRd(v);\
		v = ((v & VIRT_INTR_CH1_CRTL0_RX_Wr_Mask) | ((data & VIRT_INTR_CH1_CRTL0_RX_Mask)<<1));\
		VIRT_INTR_CH1_CRTL0_RgWr(v);\
} while(0)

#define VIRT_INTR_CH1_CRTL0_RX_UdfRd(data) do {\
		VIRT_INTR_CH1_CRTL0_RgRd(data);\
		data = ((data >> 1) & VIRT_INTR_CH1_CRTL0_RX_Mask);\
} while(0)


#define VIRT_INTR_CH2_CRTL0_RgOffAddr ((volatile ULONG *)(BASE_ADDRESS + 0x8610))

#define VIRT_INTR_CH2_CRTL0_RgWr(data) do {\
		iowrite32(data, (void *)VIRT_INTR_CH2_CRTL0_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH2_CRTL0_RgRd(data) do {\
		(data) = ioread32((void *)VIRT_INTR_CH2_CRTL0_RgOffAddr);\
} while(0)

/*#define  VIRT_INTR_CH2_CRTL0_TX_Mask (ULONG)(~(~0<<(1))) << (0)))*/

#define VIRT_INTR_CH2_CRTL0_TX_Mask (ULONG)(0x1)

/*#define VIRT_INTR_CH2_CRTL0_TX_Wr_Mask (unsigned long)(~((~(~0 << (1))) << (0)))*/

#define VIRT_INTR_CH2_CRTL0_TX_Wr_Mask (ULONG)(0xfffffffe)

#define VIRT_INTR_CH2_CRTL0_TX_UdfWr(data) do {\
		ULONG v;\
		VIRT_INTR_CH2_CRTL0_RgRd(v);\
		v = ((v & VIRT_INTR_CH2_CRTL0_TX_Wr_Mask) | ((data & VIRT_INTR_CH2_CRTL0_TX_Mask)<<0));\
		VIRT_INTR_CH2_CRTL0_RgWr(v);\
} while(0)

#define VIRT_INTR_CH2_CRTL0_TX_UdfRd(data) do {\
		VIRT_INTR_CH2_CRTL0_RgRd(data);\
		data = ((data >> 0) & VIRT_INTR_CH2_CRTL0_TX_Mask);\
} while(0)

/*#define  VIRT_INTR_CH2_CRTL0_RX_Mask (ULONG)(~(~0<<(1))) << (1)))*/

#define VIRT_INTR_CH2_CRTL0_RX_Mask (ULONG)(0x1)

/*#define VIRT_INTR_CH2_CRTL0_RX_Wr_Mask (unsigned long)(~((~(~0 << (1))) << (1)))*/

#define VIRT_INTR_CH2_CRTL0_RX_Wr_Mask (ULONG)(0xfffffffd)

#define VIRT_INTR_CH2_CRTL0_RX_UdfWr(data) do {\
		ULONG v;\
		VIRT_INTR_CH2_CRTL0_RgRd(v);\
		v = ((v & VIRT_INTR_CH2_CRTL0_RX_Wr_Mask) | ((data & VIRT_INTR_CH2_CRTL0_RX_Mask)<<1));\
		VIRT_INTR_CH2_CRTL0_RgWr(v);\
} while(0)

#define VIRT_INTR_CH2_CRTL0_RX_UdfRd(data) do {\
		VIRT_INTR_CH2_CRTL0_RgRd(data);\
		data = ((data >> 1) & VIRT_INTR_CH2_CRTL0_RX_Mask);\
} while(0)


#define VIRT_INTR_CH3_CRTL0_RgOffAddr ((volatile ULONG *)(BASE_ADDRESS + 0x8618))

#define VIRT_INTR_CH3_CRTL0_RgWr(data) do {\
		iowrite32(data, (void *)VIRT_INTR_CH3_CRTL0_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH3_CRTL0_RgRd(data) do {\
		(data) = ioread32((void *)VIRT_INTR_CH3_CRTL0_RgOffAddr);\
} while(0)

/*#define  VIRT_INTR_CH3_CRTL0_TX_Mask (ULONG)(~(~0<<(1))) << (0)))*/

#define VIRT_INTR_CH3_CRTL0_TX_Mask (ULONG)(0x1)

/*#define VIRT_INTR_CH3_CRTL0_TX_Wr_Mask (unsigned long)(~((~(~0 << (1))) << (0)))*/

#define VIRT_INTR_CH3_CRTL0_TX_Wr_Mask (ULONG)(0xfffffffe)

#define VIRT_INTR_CH3_CRTL0_TX_UdfWr(data) do {\
		ULONG v;\
		VIRT_INTR_CH3_CRTL0_RgRd(v);\
		v = ((v & VIRT_INTR_CH3_CRTL0_TX_Wr_Mask) | ((data & VIRT_INTR_CH3_CRTL0_TX_Mask)<<0));\
		VIRT_INTR_CH3_CRTL0_RgWr(v);\
} while(0)

#define VIRT_INTR_CH3_CRTL0_TX_UdfRd(data) do {\
		VIRT_INTR_CH3_CRTL0_RgRd(data);\
		data = ((data >> 0) & VIRT_INTR_CH3_CRTL0_TX_Mask);\
} while(0)

/*#define  VIRT_INTR_CH3_CRTL0_RX_Mask (ULONG)(~(~0<<(1))) << (1)))*/

#define VIRT_INTR_CH3_CRTL0_RX_Mask (ULONG)(0x1)

/*#define VIRT_INTR_CH3_CRTL0_RX_Wr_Mask (unsigned long)(~((~(~0 << (1))) << (1)))*/

#define VIRT_INTR_CH3_CRTL0_RX_Wr_Mask (ULONG)(0xfffffffd)

#define VIRT_INTR_CH3_CRTL0_RX_UdfWr(data) do {\
		ULONG v;\
		VIRT_INTR_CH3_CRTL0_RgRd(v);\
		v = ((v & VIRT_INTR_CH3_CRTL0_RX_Wr_Mask) | ((data & VIRT_INTR_CH3_CRTL0_RX_Mask)<<1));\
		VIRT_INTR_CH3_CRTL0_RgWr(v);\
} while(0)

#define VIRT_INTR_CH3_CRTL0_RX_UdfRd(data) do {\
		VIRT_INTR_CH3_CRTL0_RgRd(data);\
		data = ((data >> 1) & VIRT_INTR_CH3_CRTL0_RX_Mask);\
} while(0)


#define VIRT_INTR_CH0_STAT_RgOffAddr ((volatile ULONG *)(BASE_ADDRESS + 0x8604))

#define VIRT_INTR_CH0_STAT_RgWr(data) do {\
		iowrite32(data, (void *)VIRT_INTR_CH0_STAT_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH0_STAT_RgRd(data) do {\
		(data) = ioread32((void *)VIRT_INTR_CH0_STAT_RgOffAddr);\
} while(0)


#define VIRT_INTR_CH1_STAT_RgOffAddr ((volatile ULONG *)(BASE_ADDRESS + 0x860c))

#define VIRT_INTR_CH1_STAT_RgWr(data) do {\
		iowrite32(data, (void *)VIRT_INTR_CH1_STAT_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH1_STAT_RgRd(data) do {\
		(data) = ioread32((void *)VIRT_INTR_CH1_STAT_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH2_STAT_RgOffAddr ((volatile ULONG *)(BASE_ADDRESS + 0x8614))

#define VIRT_INTR_CH2_STAT_RgWr(data) do {\
		iowrite32(data, (void *)VIRT_INTR_CH2_STAT_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH2_STAT_RgRd(data) do {\
		(data) = ioread32((void *)VIRT_INTR_CH2_STAT_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH3_STAT_RgOffAddr ((volatile ULONG *)(BASE_ADDRESS + 0x861c))

#define VIRT_INTR_CH3_STAT_RgWr(data) do {\
		iowrite32(data, (void *)VIRT_INTR_CH3_STAT_RgOffAddr);\
} while(0)

#define VIRT_INTR_CH3_STAT_RgRd(data) do {\
		(data) = ioread32((void *)VIRT_INTR_CH3_STAT_RgOffAddr);\
} while(0)


#define VIRT_INTR_CH_CRTL_TX_Wr_Mask (ULONG)(1 << 0)
#define VIRT_INTR_CH_CRTL_RX_Wr_Mask (ULONG)(1 << 1)

#define VIRT_INTR_CH_CRTL_RgWr(chan, data) do {\
		iowrite32(data, (void *)(volatile ULONG*)(BASE_ADDRESS + 0x8600 + (chan * 8)));\
} while(0)

#define VIRT_INTR_CH_CRTL_RgRd(chan, data) do {\
		(data) = ioread32((void *)(volatile ULONG*)(BASE_ADDRESS+0x8600 + (chan * 8)));\
} while(0)

#define VIRT_INTR_CH_STAT_RgWr(chan, data) do {\
		iowrite32(data, (void *)(volatile ULONG*)(BASE_ADDRESS+0x8604 + (chan * 8)));\
} while(0)

#define VIRT_INTR_CH_STAT_RgRd(chan, data) do {\
		(data) = ioread32((void *)(volatile ULONG*)(BASE_ADDRESS+0x8604 + (chan * 8)));\
} while(0)

#endif
