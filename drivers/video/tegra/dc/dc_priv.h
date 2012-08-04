/*
 * drivers/video/tegra/dc/dc_priv.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *
 * Copyright (c) 2010-2012, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_DC_PRIV_H
#define __DRIVERS_VIDEO_TEGRA_DC_DC_PRIV_H

#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/switch.h>
#include <linux/nvhost.h>

#include <mach/dc.h>

#include <mach/tegra_dc_ext.h>
#include <mach/clk.h>

#include "dc_reg.h"

#define WIN_IS_TILED(win)	((win)->flags & TEGRA_WIN_FLAG_TILED)
#define WIN_IS_ENABLED(win)	((win)->flags & TEGRA_WIN_FLAG_ENABLED)

#define NEED_UPDATE_EMC_ON_EVERY_FRAME (windows_idle_detection_time == 0)

/* DDR: 8 bytes transfer per clock */
#define DDR_BW_TO_FREQ(bw) ((bw) / 8)

#if defined(CONFIG_TEGRA_EMC_TO_DDR_CLOCK)
#define EMC_BW_TO_FREQ(bw) (DDR_BW_TO_FREQ(bw) * CONFIG_TEGRA_EMC_TO_DDR_CLOCK)
#else
#define EMC_BW_TO_FREQ(bw) (DDR_BW_TO_FREQ(bw) * 2)
#endif

#ifndef CONFIG_TEGRA_FPGA_PLATFORM
#define ALL_UF_INT (WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT)
#else
/* ignore underflows when on simulation and fpga platform */
#define ALL_UF_INT (0)
#endif

struct tegra_dc;

struct tegra_dc_blend {
	unsigned z[DC_N_WINDOWS];
	unsigned flags[DC_N_WINDOWS];
};

struct tegra_dc_out_ops {
	/* initialize output.  dc clocks are not on at this point */
	int (*init)(struct tegra_dc *dc);
	/* destroy output.  dc clocks are not on at this point */
	void (*destroy)(struct tegra_dc *dc);
	/* detect connected display.  can sleep.*/
	bool (*detect)(struct tegra_dc *dc);
	/* enable output.  dc clocks are on at this point */
	void (*enable)(struct tegra_dc *dc);
	/* disable output.  dc clocks are on at this point */
	void (*disable)(struct tegra_dc *dc);
	/* hold output.  keeps dc clocks on. */
	void (*hold)(struct tegra_dc *dc);
	/* release output.  dc clocks may turn off after this. */
	void (*release)(struct tegra_dc *dc);
	/* idle routine of output.  dc clocks may turn off after this. */
	void (*idle)(struct tegra_dc *dc);
	/* suspend output.  dc clocks are on at this point */
	void (*suspend)(struct tegra_dc *dc);
	/* resume output.  dc clocks are on at this point */
	void (*resume)(struct tegra_dc *dc);
	/* mode filter. to provide a list of supported modes*/
	bool (*mode_filter)(const struct tegra_dc *dc,
			struct fb_videomode *mode);
};

struct tegra_dc {
	struct nvhost_device		*ndev;
	struct tegra_dc_platform_data	*pdata;

	struct resource			*base_res;
	void __iomem			*base;
	int				irq;

	struct clk			*clk;
	struct clk			*emc_clk;
	int				emc_clk_rate;
	int				new_emc_clk_rate;
	u32				shift_clk_div;

	bool				connected;
	bool				enabled;
	bool				suspended;

	struct tegra_dc_out		*out;
	struct tegra_dc_out_ops		*out_ops;
	void				*out_data;

	struct tegra_dc_mode		mode;
	s64				frametime_ns;

	struct tegra_dc_win		windows[DC_N_WINDOWS];
	struct tegra_dc_blend		blend;
	int				n_windows;
#ifdef CONFIG_TEGRA_DC_CMU
	struct tegra_dc_cmu		cmu;
#endif
	wait_queue_head_t		wq;
	wait_queue_head_t		timestamp_wq;

	struct mutex			lock;
	struct mutex			one_shot_lock;

	struct resource			*fb_mem;
	struct tegra_fb_info		*fb;

	struct {
		u32			id;
		u32			min;
		u32			max;
	} syncpt[DC_N_WINDOWS];
	u32				vblank_syncpt;

	unsigned long			underflow_mask;
	struct work_struct		reset_work;

#ifdef CONFIG_SWITCH
	struct switch_dev		modeset_switch;
#endif

	struct completion		frame_end_complete;

	struct work_struct		vblank_work;
	long				vblank_ref_count;

	struct {
		u64			underflows;
		u64			underflows_a;
		u64			underflows_b;
		u64			underflows_c;
	} stats;

	struct tegra_dc_ext		*ext;

	struct tegra_dc_feature		*feature;
	int				gen1_blend_num;

#ifdef CONFIG_DEBUG_FS
	struct dentry			*debugdir;
#endif
	struct tegra_dc_lut		fb_lut;
	struct delayed_work		underflow_work;
	u32				one_shot_delay_ms;
	struct delayed_work		one_shot_work;
	s64				frame_end_timestamp;
};

#define print_mode_info(dc, mode) do {					\
	trace_printk("%s:Mode settings: "				\
			"ref_to_sync: H = %d V = %d, "			\
			"sync_width: H = %d V = %d, "			\
			"back_porch: H = %d V = %d, "			\
			"active: H = %d V = %d, "			\
			"front_porch: H = %d V = %d, "			\
			"pclk = %d, stereo mode = %d\n",		\
			dc->ndev->name,					\
			mode.h_ref_to_sync, mode.v_ref_to_sync,		\
			mode.h_sync_width, mode.v_sync_width,		\
			mode.h_back_porch, mode.v_back_porch,		\
			mode.h_active, mode.v_active,			\
			mode.h_front_porch, mode.v_front_porch,		\
			mode.pclk, mode.stereo_mode);			\
	} while (0)

static inline void tegra_dc_io_start(struct tegra_dc *dc)
{
	nvhost_module_busy_ext(nvhost_get_parent(dc->ndev));
}

static inline void tegra_dc_io_end(struct tegra_dc *dc)
{
	nvhost_module_idle_ext(nvhost_get_parent(dc->ndev));
}

static inline unsigned long tegra_dc_readl(struct tegra_dc *dc,
					   unsigned long reg)
{
	unsigned long ret;

	BUG_ON(!nvhost_module_powered_ext(to_nvhost_device(dc->ndev->dev.parent)));
	if (!tegra_is_clk_enabled(dc->clk))
		WARN(1, "DC is clock-gated.\n");

	ret = readl(dc->base + reg * 4);
	trace_printk("readl %p=%#08lx\n", dc->base + reg * 4, ret);
	return ret;
}

static inline void tegra_dc_writel(struct tegra_dc *dc, unsigned long val,
				   unsigned long reg)
{
	BUG_ON(!nvhost_module_powered_ext(to_nvhost_device(dc->ndev->dev.parent)));
	if (!tegra_is_clk_enabled(dc->clk))
		WARN(1, "DC is clock-gated.\n");

	trace_printk("writel %p=%#08lx\n", dc->base + reg * 4, val);
	writel(val, dc->base + reg * 4);
}

static inline void _tegra_dc_write_table(struct tegra_dc *dc, const u32 *table,
					 unsigned len)
{
	int i;

	for (i = 0; i < len; i++)
		tegra_dc_writel(dc, table[i * 2 + 1], table[i * 2]);
}

#define tegra_dc_write_table(dc, table)		\
	_tegra_dc_write_table(dc, table, ARRAY_SIZE(table) / 2)

static inline void tegra_dc_set_outdata(struct tegra_dc *dc, void *data)
{
	dc->out_data = data;
}

static inline void *tegra_dc_get_outdata(struct tegra_dc *dc)
{
	return dc->out_data;
}

static inline unsigned long tegra_dc_get_default_emc_clk_rate(
	struct tegra_dc *dc)
{
	return dc->pdata->emc_clk_rate ? dc->pdata->emc_clk_rate : ULONG_MAX;
}

static inline int tegra_dc_fmt_bpp(int fmt)
{
	switch (fmt) {
	case TEGRA_WIN_FMT_P1:
		return 1;

	case TEGRA_WIN_FMT_P2:
		return 2;

	case TEGRA_WIN_FMT_P4:
		return 4;

	case TEGRA_WIN_FMT_P8:
		return 8;

	case TEGRA_WIN_FMT_B4G4R4A4:
	case TEGRA_WIN_FMT_B5G5R5A:
	case TEGRA_WIN_FMT_B5G6R5:
	case TEGRA_WIN_FMT_AB5G5R5:
		return 16;

	case TEGRA_WIN_FMT_B8G8R8A8:
	case TEGRA_WIN_FMT_R8G8B8A8:
	case TEGRA_WIN_FMT_B6x2G6x2R6x2A8:
	case TEGRA_WIN_FMT_R6x2G6x2B6x2A8:
		return 32;

	/* for planar formats, size of the Y plane, 8bit */
	case TEGRA_WIN_FMT_YCbCr420P:
	case TEGRA_WIN_FMT_YUV420P:
	case TEGRA_WIN_FMT_YCbCr422P:
	case TEGRA_WIN_FMT_YUV422P:
	case TEGRA_WIN_FMT_YCbCr422R:
	case TEGRA_WIN_FMT_YUV422R:
	case TEGRA_WIN_FMT_YCbCr422RA:
	case TEGRA_WIN_FMT_YUV422RA:
		return 8;

	/* YUYV packed into 32-bits */
	case TEGRA_WIN_FMT_YCbCr422:
	case TEGRA_WIN_FMT_YUV422:
		return 16;
	}
	return 0;
}

static inline bool tegra_dc_is_yuv(int fmt)
{
	switch (fmt) {
	case TEGRA_WIN_FMT_YUV420P:
	case TEGRA_WIN_FMT_YCbCr420P:
	case TEGRA_WIN_FMT_YCbCr422P:
	case TEGRA_WIN_FMT_YUV422P:
	case TEGRA_WIN_FMT_YCbCr422:
	case TEGRA_WIN_FMT_YUV422:
	case TEGRA_WIN_FMT_YCbCr422R:
	case TEGRA_WIN_FMT_YUV422R:
	case TEGRA_WIN_FMT_YCbCr422RA:
	case TEGRA_WIN_FMT_YUV422RA:
		return true;
	}
	return false;
}

static inline bool tegra_dc_is_yuv_planar(int fmt)
{
	switch (fmt) {
	case TEGRA_WIN_FMT_YUV420P:
	case TEGRA_WIN_FMT_YCbCr420P:
	case TEGRA_WIN_FMT_YCbCr422P:
	case TEGRA_WIN_FMT_YUV422P:
	case TEGRA_WIN_FMT_YCbCr422R:
	case TEGRA_WIN_FMT_YUV422R:
	case TEGRA_WIN_FMT_YCbCr422RA:
	case TEGRA_WIN_FMT_YUV422RA:
		return true;
	}
	return false;
}

static inline void tegra_dc_unmask_interrupt(struct tegra_dc *dc, u32 int_val)
{
	u32 val;

	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	val |= int_val;
	tegra_dc_writel(dc, val, DC_CMD_INT_MASK);
}

static inline void tegra_dc_mask_interrupt(struct tegra_dc *dc, u32 int_val)
{
	u32 val;

	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	val &= ~int_val;
	tegra_dc_writel(dc, val, DC_CMD_INT_MASK);
}

static inline unsigned long tegra_dc_clk_get_rate(struct tegra_dc *dc)
{
#ifdef CONFIG_TEGRA_SILICON_PLATFORM
	return clk_get_rate(dc->clk);
#else
	return dc->mode.pclk;
#endif
}

extern struct tegra_dc_out_ops tegra_dc_rgb_ops;
extern struct tegra_dc_out_ops tegra_dc_hdmi_ops;
extern struct tegra_dc_out_ops tegra_dc_dsi_ops;

/* defined in dc_sysfs.c, used by dc.c */
void tegra_dc_remove_sysfs(struct device *dev);
void tegra_dc_create_sysfs(struct device *dev);

/* defined in dc.c, used by dc_sysfs.c */
void tegra_dc_stats_enable(struct tegra_dc *dc, bool enable);
bool tegra_dc_stats_get(struct tegra_dc *dc);

/* defined in dc.c, used by dc_sysfs.c */
u32 tegra_dc_read_checksum_latched(struct tegra_dc *dc);
void tegra_dc_enable_crc(struct tegra_dc *dc);
void tegra_dc_disable_crc(struct tegra_dc *dc);

void tegra_dc_set_out_pin_polars(struct tegra_dc *dc,
				const struct tegra_dc_out_pin *pins,
				const unsigned int n_pins);
/* defined in dc.c, used in bandwidth.c and ext/dev.c */
unsigned int tegra_dc_has_multiple_dc(void);

/* defined in dc.c, used in dsi.c */
void tegra_dc_clk_enable(struct tegra_dc *dc);
void tegra_dc_clk_disable(struct tegra_dc *dc);

/* defined in dc.c, used in nvsd.c and dsi.c */
void tegra_dc_hold_dc_out(struct tegra_dc *dc);
void tegra_dc_release_dc_out(struct tegra_dc *dc);

/* defined in bandwidth.c, used in dc.c */
void tegra_dc_clear_bandwidth(struct tegra_dc *dc);
void tegra_dc_program_bandwidth(struct tegra_dc *dc, bool use_new);
int tegra_dc_set_dynamic_emc(struct tegra_dc_win *windows[], int n);

/* defined in mode.c, used in dc.c */
int tegra_dc_program_mode(struct tegra_dc *dc, struct tegra_dc_mode *mode);
int tegra_dc_calc_refresh(const struct tegra_dc_mode *m);

/* defined in clock.c, used in dc.c, dsi.c and hdmi.c */
void tegra_dc_setup_clk(struct tegra_dc *dc, struct clk *clk);
unsigned long tegra_dc_pclk_round_rate(struct tegra_dc *dc, int pclk);

/* defined in lut.c, used in dc.c */
void tegra_dc_init_lut_defaults(struct tegra_dc_lut *lut);
void tegra_dc_set_lut(struct tegra_dc *dc, struct tegra_dc_win *win);

/* defined in csc.c, used in dc.c */
void tegra_dc_init_csc_defaults(struct tegra_dc_csc *csc);
void tegra_dc_set_csc(struct tegra_dc *dc, struct tegra_dc_csc *csc);

/* defined in window.c, used in dc.c */
void tegra_dc_trigger_windows(struct tegra_dc *dc);

#endif
