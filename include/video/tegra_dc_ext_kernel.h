/*
 * tegra_dc_ext_kernel.h: tegra dc ext kernel module interface.
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION, All rights reserved.
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

#ifndef VIDEO_DC_EXT_KERNEL_H
#define VIDEO_DC_EXT_KERNEL_H

#include <linux/types.h>
#include <drm/drm_fixed.h>

#define DC_N_WINDOWS		6

struct tegra_dc;
struct nvmap_handle_ref;

#define TEGRA_WIN_FLAG_ENABLED		(1 << 0)
#define TEGRA_WIN_FLAG_BLEND_PREMULT	(1 << 1)
#define TEGRA_WIN_FLAG_BLEND_COVERAGE	(1 << 2)
#define TEGRA_WIN_FLAG_INVERT_H		(1 << 3)
#define TEGRA_WIN_FLAG_INVERT_V		(1 << 4)
#define TEGRA_WIN_FLAG_TILED		(1 << 5)
#define TEGRA_WIN_FLAG_H_FILTER		(1 << 6)
#define TEGRA_WIN_FLAG_V_FILTER		(1 << 7)
#define TEGRA_WIN_FLAG_BLOCKLINEAR	(1 << 8)
#define TEGRA_WIN_FLAG_SCAN_COLUMN	(1 << 9)
#define TEGRA_WIN_FLAG_INTERLACE	(1 << 10)
#define TEGRA_WIN_FLAG_FB		(1 << 11)
#define TEGRA_WIN_FLAG_INPUT_RANGE_MASK	(3 << 12)
#define TEGRA_WIN_FLAG_INPUT_RANGE_FULL	(0 << 12)
#define TEGRA_WIN_FLAG_INPUT_RANGE_LIMITED	(1 << 12)
#define TEGRA_WIN_FLAG_INPUT_RANGE_BYPASS	(2 << 12)
#define TEGRA_WIN_FLAG_CS_MASK		(7 << 14)
#define TEGRA_WIN_FLAG_CS_DEFAULT	(0 << 14)
#define TEGRA_WIN_FLAG_CS_REC601	(1 << 14)
#define TEGRA_WIN_FLAG_CS_REC709	(2 << 14)
#define TEGRA_WIN_FLAG_CS_REC2020	(4 << 14)
#define TEGRA_WIN_FLAG_DEGAMMA_MASK	(15 << 16)
#define TEGRA_WIN_FLAG_DEGAMMA_DEFAULT	(0 << 16) /* driver selects */
#define TEGRA_WIN_FLAG_DEGAMMA_NONE	(1 << 16)
#define TEGRA_WIN_FLAG_DEGAMMA_SRGB	(2 << 16)
#define TEGRA_WIN_FLAG_DEGAMMA_YUV_8_10	(4 << 16)
#define TEGRA_WIN_FLAG_DEGAMMA_YUV_12	(8 << 16)
#define TEGRA_WIN_FLAG_INVALID		(1 << 31) /* window does not exist. */

#define TEGRA_WIN_BLEND_FLAGS_MASK \
	(TEGRA_WIN_FLAG_BLEND_PREMULT | TEGRA_WIN_FLAG_BLEND_COVERAGE)

#if defined(CONFIG_TEGRA_CSC_V2)
struct tegra_dc_csc_v2 {
	u32 r2r;
	u32 g2r;
	u32 b2r;
	u32 const2r;
	u32 r2g;
	u32 g2g;
	u32 b2g;
	u32 const2g;
	u32 r2b;
	u32 g2b;
	u32 b2b;
	u32 const2b;
	u32 csc_enable;
};
#endif

struct tegra_dc_csc {
	unsigned short yof;
	unsigned short kyrgb;
	unsigned short kur;
	unsigned short kvr;
	unsigned short kug;
	unsigned short kvg;
	unsigned short kub;
	unsigned short kvb;
};

#if defined(CONFIG_TEGRA_LUT)
/* palette lookup table */
struct tegra_dc_lut {
	u8 r[256];
	u8 g[256];
	u8 b[256];
};
#endif

#if defined(CONFIG_TEGRA_LUT_V2)
/* palette lookup table */
struct tegra_dc_lut {
	u64 *rgb;
	dma_addr_t phy_addr;
	size_t size;
};
#endif

struct tegra_dc_win {
	u8			idx;
	u8			ppflags; /* see TEGRA_WIN_PPFLAG* */
	u8			global_alpha;
	u32			fmt;
	u32			flags;

	void			*virt_addr;
	dma_addr_t		phys_addr;
	dma_addr_t		phys_addr_u;
	dma_addr_t		phys_addr_v;
#if defined(CONFIG_TEGRA_DC_INTERLACE)
	/* field 2 starting address */
	dma_addr_t		phys_addr2;
	dma_addr_t		phys_addr_u2;
	dma_addr_t		phys_addr_v2;
#endif
	unsigned		stride;
	unsigned		stride_uv;
	fixed20_12		x;
	fixed20_12		y;
	fixed20_12		w;
	fixed20_12		h;
	unsigned		out_x;
	unsigned		out_y;
	unsigned		out_w;
	unsigned		out_h;
	unsigned		z;

#if defined(CONFIG_TEGRA_CSC_V2)
	struct tegra_dc_csc_v2	csc;
	bool force_user_csc;
#else
	struct tegra_dc_csc	csc;
#endif
	bool			csc_dirty;

	int			dirty;
	int			underflows;
	struct tegra_dc		*dc;

	struct nvmap_handle_ref	*cur_handle;
	unsigned		bandwidth;
	unsigned		new_bandwidth;
	struct tegra_dc_lut	lut;
#if defined(CONFIG_TEGRA_DC_BLOCK_LINEAR)
	u8	block_height_log2;
#endif
#if defined(CONFIG_TEGRA_DC_CDE)
	struct {
		dma_addr_t cde_addr;
		unsigned offset_x;
		unsigned offset_y;
		u32 zbc_color;
		unsigned ctb_entry;
	} cde;
#endif
	struct {
		u32			id;
		u32			min;
		u32			max;
	} syncpt;

	bool		is_scaler_coeff_set;
	bool		color_expand_enable;

#ifdef CONFIG_TEGRA_NVDISPLAY
	bool		precomp_caps_read;
	u32		precomp_capc;
	u32		precomp_cape;
#endif
};

struct tegra_dc *tegra_dc_get_dc(unsigned idx);
#ifdef CONFIG_TEGRA_ISOMGR
int tegra_dc_bandwidth_negotiate_bw(struct tegra_dc *dc,
			struct tegra_dc_win *windows[], int n);
#endif
int tegra_dc_get_numof_dispheads(void);
int tegra_dc_get_numof_dispwindows(void);

/* needed by tegra-throughput */
int tegra_dc_set_flip_callback(void (*callback)(void));
int tegra_dc_unset_flip_callback(void);
int tegra_dc_get_panel_sync_rate(void);

/* needed by tegra-mods */
unsigned long tegra_dc_readl_exported(struct tegra_dc *, unsigned long);
void tegra_dc_writel_exported(struct tegra_dc *, unsigned long, unsigned long);

#endif
