/*
 * drivers/video/tegra/dc/dc.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/backlight.h>

#include <mach/clk.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/mc.h>
#include <mach/nvhost.h>

#include "dc_reg.h"
#include "dc_priv.h"
#include "overlay.h"
#include "nvsd.h"

static int no_vsync;

module_param_named(no_vsync, no_vsync, int, S_IRUGO | S_IWUSR);

struct tegra_dc *tegra_dcs[TEGRA_MAX_DC];

DEFINE_MUTEX(tegra_dc_lock);
DEFINE_MUTEX(shared_lock);

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
		return 8;

	case TEGRA_WIN_FMT_YCbCr422:
	case TEGRA_WIN_FMT_YUV422:
	case TEGRA_WIN_FMT_YCbCr422R:
	case TEGRA_WIN_FMT_YUV422R:
	case TEGRA_WIN_FMT_YCbCr422RA:
	case TEGRA_WIN_FMT_YUV422RA:
		/* FIXME: need to know the bpp of these formats */
		return 0;
	}
	return 0;
}

static inline bool tegra_dc_is_yuv_planar(int fmt)
{
	switch (fmt) {
	case TEGRA_WIN_FMT_YUV420P:
	case TEGRA_WIN_FMT_YCbCr420P:
	case TEGRA_WIN_FMT_YCbCr422P:
	case TEGRA_WIN_FMT_YUV422P:
		return true;
	}
	return false;
}

#define DUMP_REG(a) do {			\
	snprintf(buff, sizeof(buff), "%-32s\t%03x\t%08lx\n", \
		 #a, a, tegra_dc_readl(dc, a));		      \
	print(data, buff);				      \
	} while (0)

static void _dump_regs(struct tegra_dc *dc, void *data,
		       void (* print)(void *data, const char *str))
{
	int i;
	char buff[256];

	tegra_dc_io_start(dc);
	clk_enable(dc->clk);

	DUMP_REG(DC_CMD_DISPLAY_COMMAND_OPTION0);
	DUMP_REG(DC_CMD_DISPLAY_COMMAND);
	DUMP_REG(DC_CMD_SIGNAL_RAISE);
	DUMP_REG(DC_CMD_INT_STATUS);
	DUMP_REG(DC_CMD_INT_MASK);
	DUMP_REG(DC_CMD_INT_ENABLE);
	DUMP_REG(DC_CMD_INT_TYPE);
	DUMP_REG(DC_CMD_INT_POLARITY);
	DUMP_REG(DC_CMD_SIGNAL_RAISE1);
	DUMP_REG(DC_CMD_SIGNAL_RAISE2);
	DUMP_REG(DC_CMD_SIGNAL_RAISE3);
	DUMP_REG(DC_CMD_STATE_ACCESS);
	DUMP_REG(DC_CMD_STATE_CONTROL);
	DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
	DUMP_REG(DC_CMD_REG_ACT_CONTROL);

	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS0);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS1);
	DUMP_REG(DC_DISP_DISP_WIN_OPTIONS);
	DUMP_REG(DC_DISP_MEM_HIGH_PRIORITY);
	DUMP_REG(DC_DISP_MEM_HIGH_PRIORITY_TIMER);
	DUMP_REG(DC_DISP_DISP_TIMING_OPTIONS);
	DUMP_REG(DC_DISP_REF_TO_SYNC);
	DUMP_REG(DC_DISP_SYNC_WIDTH);
	DUMP_REG(DC_DISP_BACK_PORCH);
	DUMP_REG(DC_DISP_DISP_ACTIVE);
	DUMP_REG(DC_DISP_FRONT_PORCH);
	DUMP_REG(DC_DISP_H_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_D);
	DUMP_REG(DC_DISP_V_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE3_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE3_POSITION_A);
	DUMP_REG(DC_DISP_M0_CONTROL);
	DUMP_REG(DC_DISP_M1_CONTROL);
	DUMP_REG(DC_DISP_DI_CONTROL);
	DUMP_REG(DC_DISP_PP_CONTROL);
	DUMP_REG(DC_DISP_PP_SELECT_A);
	DUMP_REG(DC_DISP_PP_SELECT_B);
	DUMP_REG(DC_DISP_PP_SELECT_C);
	DUMP_REG(DC_DISP_PP_SELECT_D);
	DUMP_REG(DC_DISP_DISP_CLOCK_CONTROL);
	DUMP_REG(DC_DISP_DISP_INTERFACE_CONTROL);
	DUMP_REG(DC_DISP_DISP_COLOR_CONTROL);
	DUMP_REG(DC_DISP_SHIFT_CLOCK_OPTIONS);
	DUMP_REG(DC_DISP_DATA_ENABLE_OPTIONS);
	DUMP_REG(DC_DISP_SERIAL_INTERFACE_OPTIONS);
	DUMP_REG(DC_DISP_LCD_SPI_OPTIONS);
	DUMP_REG(DC_DISP_BORDER_COLOR);
	DUMP_REG(DC_DISP_COLOR_KEY0_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY0_UPPER);
	DUMP_REG(DC_DISP_COLOR_KEY1_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY1_UPPER);
	DUMP_REG(DC_DISP_CURSOR_FOREGROUND);
	DUMP_REG(DC_DISP_CURSOR_BACKGROUND);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR_NS);
	DUMP_REG(DC_DISP_CURSOR_POSITION);
	DUMP_REG(DC_DISP_CURSOR_POSITION_NS);
	DUMP_REG(DC_DISP_INIT_SEQ_CONTROL);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_A);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_B);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_C);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_D);
	DUMP_REG(DC_DISP_DC_MCCIF_FIFOCTRL);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0A_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0B_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0C_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY1B_HYST);
	DUMP_REG(DC_DISP_DAC_CRT_CTRL);
	DUMP_REG(DC_DISP_DISP_MISC_CONTROL);


	for (i = 0; i < 3; i++) {
		print(data, "\n");
		snprintf(buff, sizeof(buff), "WINDOW %c:\n", 'A' + i);
		print(data, buff);

		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
		DUMP_REG(DC_WIN_WIN_OPTIONS);
		DUMP_REG(DC_WIN_BYTE_SWAP);
		DUMP_REG(DC_WIN_BUFFER_CONTROL);
		DUMP_REG(DC_WIN_COLOR_DEPTH);
		DUMP_REG(DC_WIN_POSITION);
		DUMP_REG(DC_WIN_SIZE);
		DUMP_REG(DC_WIN_PRESCALED_SIZE);
		DUMP_REG(DC_WIN_H_INITIAL_DDA);
		DUMP_REG(DC_WIN_V_INITIAL_DDA);
		DUMP_REG(DC_WIN_DDA_INCREMENT);
		DUMP_REG(DC_WIN_LINE_STRIDE);
		DUMP_REG(DC_WIN_BUF_STRIDE);
		DUMP_REG(DC_WIN_UV_BUF_STRIDE);
		DUMP_REG(DC_WIN_BLEND_NOKEY);
		DUMP_REG(DC_WIN_BLEND_1WIN);
		DUMP_REG(DC_WIN_BLEND_2WIN_X);
		DUMP_REG(DC_WIN_BLEND_2WIN_Y);
		DUMP_REG(DC_WIN_BLEND_3WIN_XY);
		DUMP_REG(DC_WINBUF_START_ADDR);
		DUMP_REG(DC_WINBUF_START_ADDR_U);
		DUMP_REG(DC_WINBUF_START_ADDR_V);
		DUMP_REG(DC_WINBUF_ADDR_H_OFFSET);
		DUMP_REG(DC_WINBUF_ADDR_V_OFFSET);
		DUMP_REG(DC_WINBUF_UFLOW_STATUS);
		DUMP_REG(DC_WIN_CSC_YOF);
		DUMP_REG(DC_WIN_CSC_KYRGB);
		DUMP_REG(DC_WIN_CSC_KUR);
		DUMP_REG(DC_WIN_CSC_KVR);
		DUMP_REG(DC_WIN_CSC_KUG);
		DUMP_REG(DC_WIN_CSC_KVG);
		DUMP_REG(DC_WIN_CSC_KUB);
		DUMP_REG(DC_WIN_CSC_KVB);
	}

	DUMP_REG(DC_CMD_DISPLAY_POWER_CONTROL);
	DUMP_REG(DC_COM_PIN_OUTPUT_ENABLE2);
	DUMP_REG(DC_COM_PIN_OUTPUT_POLARITY2);
	DUMP_REG(DC_COM_PIN_OUTPUT_DATA2);
	DUMP_REG(DC_COM_PIN_INPUT_ENABLE2);
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT5);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS0);
	DUMP_REG(DC_DISP_M1_CONTROL);
	DUMP_REG(DC_COM_PM1_CONTROL);
	DUMP_REG(DC_COM_PM1_DUTY_CYCLE);
	DUMP_REG(DC_DISP_SD_CONTROL);

	clk_disable(dc->clk);
	tegra_dc_io_end(dc);
}

#undef DUMP_REG

#ifdef DEBUG
static void dump_regs_print(void *data, const char *str)
{
	struct tegra_dc *dc = data;
	dev_dbg(&dc->ndev->dev, "%s", str);
}

static void dump_regs(struct tegra_dc *dc)
{
	_dump_regs(dc, dc, dump_regs_print);
}
#else

static void dump_regs(struct tegra_dc *dc) {}

#endif

#ifdef CONFIG_DEBUG_FS

static void dbg_regs_print(void *data, const char *str)
{
	struct seq_file *s = data;

	seq_printf(s, "%s", str);
}

#undef DUMP_REG

static int dbg_dc_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;

	_dump_regs(dc, s, dbg_regs_print);

	return 0;
}


static int dbg_dc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_show, inode->i_private);
}

static const struct file_operations dbg_fops = {
	.open		= dbg_dc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void tegra_dc_dbg_add(struct tegra_dc *dc)
{
	char name[32];

	snprintf(name, sizeof(name), "tegra_dc%d_regs", dc->ndev->id);
	(void) debugfs_create_file(name, S_IRUGO, NULL, dc, &dbg_fops);
}
#else
static void tegra_dc_dbg_add(struct tegra_dc *dc) {}

#endif


static int tegra_dc_add(struct tegra_dc *dc, int index)
{
	int ret = 0;

	mutex_lock(&tegra_dc_lock);
	if (index >= TEGRA_MAX_DC) {
		ret = -EINVAL;
		goto out;
	}

	if (tegra_dcs[index] != NULL) {
		ret = -EBUSY;
		goto out;
	}

	tegra_dcs[index] = dc;

out:
	mutex_unlock(&tegra_dc_lock);

	return ret;
}

struct tegra_dc *tegra_dc_get_dc(unsigned idx)
{
	if (idx < TEGRA_MAX_DC)
		return tegra_dcs[idx];
	else
		return NULL;
}
EXPORT_SYMBOL(tegra_dc_get_dc);

struct tegra_dc_win *tegra_dc_get_window(struct tegra_dc *dc, unsigned win)
{
	if (win >= dc->n_windows)
		return NULL;

	return &dc->windows[win];
}
EXPORT_SYMBOL(tegra_dc_get_window);

static int get_topmost_window(u32 *depths, unsigned long *wins)
{
	int idx, best = -1;

	for_each_set_bit(idx, wins, DC_N_WINDOWS) {
		if (best == -1 || depths[idx] < depths[best])
			best = idx;
	}
	clear_bit(best, wins);
	return best;
}

static u32 blend_topwin(u32 flags)
{
	if (flags & TEGRA_WIN_FLAG_BLEND_COVERAGE)
		return BLEND(NOKEY, ALPHA, 0xff, 0xff);
	else if (flags & TEGRA_WIN_FLAG_BLEND_PREMULT)
		return BLEND(NOKEY, PREMULT, 0xff, 0xff);
	else
		return BLEND(NOKEY, FIX, 0xff, 0xff);
}

static u32 blend_2win(int idx, unsigned long behind_mask, u32* flags, int xy)
{
	int other;

	for (other = 0; other < DC_N_WINDOWS; other++) {
		if (other != idx && (xy-- == 0))
			break;
	}
	if (BIT(other) & behind_mask)
		return blend_topwin(flags[idx]);
	else if (flags[other])
		return BLEND(NOKEY, DEPENDANT, 0x00, 0x00);
	else
		return BLEND(NOKEY, FIX, 0x00, 0x00);
}

static u32 blend_3win(int idx, unsigned long behind_mask, u32* flags)
{
	unsigned long infront_mask;
	int first;

	infront_mask = ~(behind_mask | BIT(idx));
	infront_mask &= (BIT(DC_N_WINDOWS) - 1);
	first = ffs(infront_mask) - 1;

	if (!infront_mask)
		return blend_topwin(flags[idx]);
	else if (behind_mask && first != -1 && flags[first])
		return BLEND(NOKEY, DEPENDANT, 0x00, 0x00);
	else
		return BLEND(NOKEY, FIX, 0x0, 0x0);
}

static void tegra_dc_set_blending(struct tegra_dc *dc, struct tegra_dc_blend *blend)
{
	unsigned long mask = BIT(DC_N_WINDOWS) - 1;

	while (mask) {
		int idx = get_topmost_window(blend->z, &mask);

		tegra_dc_writel(dc, WINDOW_A_SELECT << idx,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		tegra_dc_writel(dc, BLEND(NOKEY, FIX, 0xff, 0xff),
				DC_WIN_BLEND_NOKEY);
		tegra_dc_writel(dc, BLEND(NOKEY, FIX, 0xff, 0xff),
				DC_WIN_BLEND_1WIN);
		tegra_dc_writel(dc, blend_2win(idx, mask, blend->flags, 0),
				DC_WIN_BLEND_2WIN_X);
		tegra_dc_writel(dc, blend_2win(idx, mask, blend->flags, 1),
				DC_WIN_BLEND_2WIN_Y);
		tegra_dc_writel(dc, blend_3win(idx, mask, blend->flags),
				DC_WIN_BLEND_3WIN_XY);
	}
}

static void tegra_dc_set_csc(struct tegra_dc *dc)
{
	tegra_dc_writel(dc, 0x00f0, DC_WIN_CSC_YOF);
	tegra_dc_writel(dc, 0x012a, DC_WIN_CSC_KYRGB);
	tegra_dc_writel(dc, 0x0000, DC_WIN_CSC_KUR);
	tegra_dc_writel(dc, 0x0198, DC_WIN_CSC_KVR);
	tegra_dc_writel(dc, 0x039b, DC_WIN_CSC_KUG);
	tegra_dc_writel(dc, 0x032f, DC_WIN_CSC_KVG);
	tegra_dc_writel(dc, 0x0204, DC_WIN_CSC_KUB);
	tegra_dc_writel(dc, 0x0000, DC_WIN_CSC_KVB);
}

static void tegra_dc_set_scaling_filter(struct tegra_dc *dc)
{
	unsigned i;
	unsigned v0 = 128;
	unsigned v1 = 0;
	/* linear horizontal and vertical filters */
	for (i = 0; i < 16; i++) {
		tegra_dc_writel(dc, (v1 << 16) | (v0 << 8),
				DC_WIN_H_FILTER_P(i));

		tegra_dc_writel(dc, v0,
				DC_WIN_V_FILTER_P(i));
		v0 -= 8;
		v1 += 8;
	}
}

/* does not support updating windows on multiple dcs in one call */
int tegra_dc_update_windows(struct tegra_dc_win *windows[], int n)
{
	struct tegra_dc *dc;
	unsigned long update_mask = GENERAL_ACT_REQ;
	unsigned long val;
	bool update_blend = false;
	int i;

	dc = windows[0]->dc;

	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return -EFAULT;
	}

	if (no_vsync)
		tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE, DC_CMD_STATE_ACCESS);
	else
		tegra_dc_writel(dc, WRITE_MUX_ASSEMBLY | READ_MUX_ASSEMBLY, DC_CMD_STATE_ACCESS);

	for (i = 0; i < n; i++) {
		struct tegra_dc_win *win = windows[i];
		unsigned h_dda;
		unsigned v_dda;
		unsigned h_offset;
		unsigned v_offset;
		bool invert_h = (win->flags & TEGRA_WIN_FLAG_INVERT_H) != 0;
		bool invert_v = (win->flags & TEGRA_WIN_FLAG_INVERT_V) != 0;
		bool yuvp = tegra_dc_is_yuv_planar(win->fmt);

		if (win->z != dc->blend.z[win->idx]) {
			dc->blend.z[win->idx] = win->z;
			update_blend = true;
		}
		if ((win->flags & TEGRA_WIN_BLEND_FLAGS_MASK) !=
			dc->blend.flags[win->idx]) {
			dc->blend.flags[win->idx] =
				win->flags & TEGRA_WIN_BLEND_FLAGS_MASK;
			update_blend = true;
		}

		tegra_dc_writel(dc, WINDOW_A_SELECT << win->idx,
				DC_CMD_DISPLAY_WINDOW_HEADER);

		if (!no_vsync)
			update_mask |= WIN_A_ACT_REQ << win->idx;

		if (!(win->flags & TEGRA_WIN_FLAG_ENABLED)) {
			tegra_dc_writel(dc, 0, DC_WIN_WIN_OPTIONS);
			continue;
		}

		tegra_dc_writel(dc, win->fmt, DC_WIN_COLOR_DEPTH);
		tegra_dc_writel(dc, 0, DC_WIN_BYTE_SWAP);

		tegra_dc_writel(dc,
				V_POSITION(win->out_y) | H_POSITION(win->out_x),
				DC_WIN_POSITION);
		tegra_dc_writel(dc,
				V_SIZE(win->out_h) | H_SIZE(win->out_w),
				DC_WIN_SIZE);
		tegra_dc_writel(dc,
				V_PRESCALED_SIZE(win->h) |
				H_PRESCALED_SIZE(win->w * tegra_dc_fmt_bpp(win->fmt) / 8),
				DC_WIN_PRESCALED_SIZE);

		h_dda = ((win->w - 1) * 0x1000) / max_t(int, win->out_w - 1, 1);
		v_dda = ((win->h - 1) * 0x1000) / max_t(int, win->out_h - 1, 1);
		tegra_dc_writel(dc, V_DDA_INC(v_dda) | H_DDA_INC(h_dda),
				DC_WIN_DDA_INCREMENT);
		tegra_dc_writel(dc, 0, DC_WIN_H_INITIAL_DDA);
		tegra_dc_writel(dc, 0, DC_WIN_V_INITIAL_DDA);

		tegra_dc_writel(dc, 0, DC_WIN_BUF_STRIDE);
		tegra_dc_writel(dc, 0, DC_WIN_UV_BUF_STRIDE);
		tegra_dc_writel(dc,
				(unsigned long)win->phys_addr +
				(unsigned long)win->offset,
				DC_WINBUF_START_ADDR);

		if (!yuvp) {
			tegra_dc_writel(dc, win->stride, DC_WIN_LINE_STRIDE);
		} else {
			tegra_dc_writel(dc,
					(unsigned long)win->phys_addr +
					(unsigned long)win->offset_u,
					DC_WINBUF_START_ADDR_U);
			tegra_dc_writel(dc,
					(unsigned long)win->phys_addr +
					(unsigned long)win->offset_v,
					DC_WINBUF_START_ADDR_V);
			tegra_dc_writel(dc,
					LINE_STRIDE(win->stride) |
					UV_LINE_STRIDE(win->stride_uv),
					DC_WIN_LINE_STRIDE);
		}

		h_offset = win->x;
		if (invert_h) {
			h_offset += win->w - 1;
		}
		h_offset *= tegra_dc_fmt_bpp(win->fmt) / 8;

		v_offset = win->y;
		if (invert_v) {
			v_offset += win->h - 1;
		}

		tegra_dc_writel(dc, h_offset, DC_WINBUF_ADDR_H_OFFSET);
		tegra_dc_writel(dc, v_offset, DC_WINBUF_ADDR_V_OFFSET);

		if (win->flags & TEGRA_WIN_FLAG_TILED)
			tegra_dc_writel(dc,
					DC_WIN_BUFFER_ADDR_MODE_TILE |
					DC_WIN_BUFFER_ADDR_MODE_TILE_UV,
					DC_WIN_BUFFER_ADDR_MODE);
		else
			tegra_dc_writel(dc,
					DC_WIN_BUFFER_ADDR_MODE_LINEAR |
					DC_WIN_BUFFER_ADDR_MODE_LINEAR_UV,
					DC_WIN_BUFFER_ADDR_MODE);

		val = WIN_ENABLE;
		if (yuvp)
			val |= CSC_ENABLE;
		else if (tegra_dc_fmt_bpp(win->fmt) < 24)
			val |= COLOR_EXPAND;

		if (win->w != win->out_w)
			val |= H_FILTER_ENABLE;
		if (win->h != win->out_h)
			val |= V_FILTER_ENABLE;

		if (invert_h)
			val |= H_DIRECTION_DECREMENT;
		if (invert_v)
			val |= V_DIRECTION_DECREMENT;

		tegra_dc_writel(dc, val, DC_WIN_WIN_OPTIONS);

		win->dirty = no_vsync ? 0 : 1;
	}

	if (update_blend) {
		tegra_dc_set_blending(dc, &dc->blend);
		for (i = 0; i < DC_N_WINDOWS; i++) {
			if (!no_vsync)
				dc->windows[i].dirty = 1;
			update_mask |= WIN_A_ACT_REQ << i;
		}
	}

	tegra_dc_writel(dc, update_mask << 8, DC_CMD_STATE_CONTROL);

	if (!no_vsync) {
		val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
		val |= FRAME_END_INT;
		tegra_dc_writel(dc, val, DC_CMD_INT_ENABLE);

		val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
		val |= FRAME_END_INT;
		tegra_dc_writel(dc, val, DC_CMD_INT_MASK);
	}

	tegra_dc_writel(dc, update_mask, DC_CMD_STATE_CONTROL);

	mutex_unlock(&dc->lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_update_windows);

u32 tegra_dc_get_syncpt_id(const struct tegra_dc *dc)
{
	return dc->syncpt_id;
}
EXPORT_SYMBOL(tegra_dc_get_syncpt_id);

u32 tegra_dc_incr_syncpt_max(struct tegra_dc *dc)
{
	u32 max;

	mutex_lock(&dc->lock);
	max = nvhost_syncpt_incr_max(&dc->ndev->host->syncpt, dc->syncpt_id,
					((dc->enabled) ? 1 : 0) );
	dc->syncpt_max = max;
	mutex_unlock(&dc->lock);

	return max;
}

void tegra_dc_incr_syncpt_min(struct tegra_dc *dc, u32 val)
{
	mutex_lock(&dc->lock);
	if ( dc->enabled )
		while (dc->syncpt_min < val) {
			dc->syncpt_min++;
			nvhost_syncpt_cpu_incr(&dc->ndev->host->syncpt,
				dc->syncpt_id);
		}
	mutex_unlock(&dc->lock);
}

static bool tegra_dc_windows_are_clean(struct tegra_dc_win *windows[],
					     int n)
{
	int i;

	for (i = 0; i < n; i++) {
		if (windows[i]->dirty)
			return false;
	}

	return true;
}

/* does not support syncing windows on multiple dcs in one call */
int tegra_dc_sync_windows(struct tegra_dc_win *windows[], int n)
{
	if (n < 1 || n > DC_N_WINDOWS)
		return -EINVAL;

	if (!windows[0]->dc->enabled)
		return -EFAULT;

	return wait_event_interruptible_timeout(windows[0]->dc->wq,
					 tegra_dc_windows_are_clean(windows, n),
					 HZ);
}
EXPORT_SYMBOL(tegra_dc_sync_windows);

static unsigned long tegra_dc_clk_get_rate(struct tegra_dc *dc)
{
#ifdef CONFIG_TEGRA_FPGA_PLATFORM
	return 27000000;
#else
	return clk_get_rate(dc->clk);
#endif
}

static unsigned long tegra_dc_pclk_round_rate(struct tegra_dc *dc, int pclk)
{
	unsigned long rate;
	unsigned long div;

	rate = tegra_dc_clk_get_rate(dc);

	div = DIV_ROUND_CLOSEST(rate * 2, pclk);

	if (div < 2)
		return 0;

	return rate * 2 / div;
}

void tegra_dc_setup_clk(struct tegra_dc *dc, struct clk *clk)
{
	int pclk;

	if (dc->out->type == TEGRA_DC_OUT_RGB) {
		struct clk *parent_clk =
			clk_get_sys(NULL, dc->out->parent_clk ? : "pll_p");

		if (clk_get_parent(clk) != parent_clk)
			clk_set_parent(clk, parent_clk);
	}

	if (dc->out->type == TEGRA_DC_OUT_HDMI) {
		unsigned long rate;
		struct clk *pll_d_out0_clk =
			clk_get_sys(NULL, "pll_d_out0");
		struct clk *pll_d_clk =
			clk_get_sys(NULL, "pll_d");

		if (dc->mode.pclk > 70000000)
			rate = 594000000;
		else
			rate = 216000000;

		if (rate != clk_get_rate(pll_d_clk))
			clk_set_rate(pll_d_clk, rate);

		if (clk_get_parent(clk) != pll_d_out0_clk)
			clk_set_parent(clk, pll_d_out0_clk);
	}

	if (dc->out->type == TEGRA_DC_OUT_DSI) {
		unsigned long rate;
		struct clk *pll_d_out0_clk;
		struct clk *pll_d_clk;

		if (clk == dc->clk) {
			pll_d_out0_clk = clk_get_sys(NULL, "pll_d_out0");
			pll_d_clk = clk_get_sys(NULL, "pll_d");
		} else {
			if (dc->pdata->default_out->dsi->dsi_instance) {
				pll_d_out0_clk = clk_get_sys(NULL, "pll_d2_out0");
				pll_d_clk = clk_get_sys(NULL, "pll_d2");
				tegra_clk_cfg_ex(pll_d_clk, TEGRA_CLK_PLLD_CSI_OUT_ENB, 1);
			} else {
				pll_d_out0_clk = clk_get_sys(NULL, "pll_d_out0");
				pll_d_clk = clk_get_sys(NULL, "pll_d");
			}
		}

		rate = dc->mode.pclk;
		if (rate != clk_get_rate(pll_d_clk))
			clk_set_rate(pll_d_clk, rate);

		if (clk_get_parent(clk) != pll_d_out0_clk)
			clk_set_parent(clk, pll_d_out0_clk);
	}

	pclk = tegra_dc_pclk_round_rate(dc, dc->mode.pclk);
	tegra_dvfs_set_rate(clk, pclk);
}

/* return non-zero if constraint is violated */
static int calc_h_ref_to_sync(const struct tegra_dc_mode *mode, int *href)
{
	long a, b;

	/* Constraint 5: H_REF_TO_SYNC >= 0 */
	a = 0;

	/* Constraint 6: H_FRONT_PORT >= (H_REF_TO_SYNC + 1) */
	b = mode->h_front_porch - 1;

	/* Constraint 1: H_REF_TO_SYNC + H_SYNC_WIDTH + H_BACK_PORCH > 11 */
	if (a + mode->h_sync_width + mode->h_back_porch <= 11)
		a = 1 + 11 - mode->h_sync_width - mode->h_back_porch;
	/* check Constraint 1 and 6 */
	if (a > b)
		return 1;

	/* Constraint 4: H_SYNC_WIDTH >= 1 */
	if (mode->h_sync_width < 1)
		return 4;

	/* Constraint 7: H_DISP_ACTIVE >= 16 */
	if (mode->h_active < 16)
		return 7;

	if (href) {
		if (b > a && a % 2)
			*href = a + 1; /* use smallest even value */
		else
			*href = a; /* even or only possible value */
	}

	return 0;
}

static int calc_v_ref_to_sync(const struct tegra_dc_mode *mode, int *vref)
{
	long a;
	a = 1; /* Constraint 5: V_REF_TO_SYNC >= 1 */

	/* Constraint 2: V_REF_TO_SYNC + V_SYNC_WIDTH + V_BACK_PORCH > 1 */
	if (a + mode->v_sync_width + mode->v_back_porch <= 1)
		a = 1 + 1 - mode->v_sync_width - mode->v_back_porch;

	/* Constraint 6 */
	if (mode->v_front_porch < a + 1)
		a = mode->v_front_porch - 1;

	/* Constraint 4: V_SYNC_WIDTH >= 1 */
	if (mode->v_sync_width < 1)
		return 4;

	/* Constraint 7: V_DISP_ACTIVE >= 16 */
	if (mode->v_active < 16)
		return 7;

        if (vref)
                *vref = a;
	return 0;
}

static int calc_ref_to_sync(struct tegra_dc_mode *mode)
{
	int ret;
	ret = calc_h_ref_to_sync(mode, &mode->h_ref_to_sync);
	if (ret)
		return ret;
	ret = calc_v_ref_to_sync(mode, &mode->v_ref_to_sync);
	if (ret)
		return ret;

	return 0;
}

#ifdef DEBUG
/* return in 1000ths of a Hertz */
static int calc_refresh(const struct tegra_dc_mode *m)
{
	long h_total, v_total, refresh;
	h_total = m->h_active + m->h_front_porch + m->h_back_porch +
		m->h_sync_width;
	v_total = m->v_active + m->v_front_porch + m->v_back_porch +
		m->v_sync_width;
	refresh = m->pclk / h_total;
	refresh *= 1000;
	refresh /= v_total;
	return refresh;
}

static void print_mode(struct tegra_dc *dc,
			const struct tegra_dc_mode *mode, const char *note)
{
	if (mode) {
		int refresh = calc_refresh(mode);
		dev_info(&dc->ndev->dev, "%s():MODE:%dx%d@%d.%03uHz pclk=%d\n",
			note ? note : "",
			mode->h_active, mode->v_active,
			refresh / 1000, refresh % 1000,
			mode->pclk);
	}
}
#else
static inline void print_mode(struct tegra_dc *dc,
			const struct tegra_dc_mode *mode, const char *note) { }
#endif

static int tegra_dc_program_mode(struct tegra_dc *dc, struct tegra_dc_mode *mode)
{
	unsigned long val;
	unsigned long rate;
	unsigned long div;
	unsigned long pclk;

	print_mode(dc, mode, __func__);

	tegra_dc_writel(dc, 0x0, DC_DISP_DISP_TIMING_OPTIONS);
	tegra_dc_writel(dc, mode->h_ref_to_sync | (mode->v_ref_to_sync << 16),
			DC_DISP_REF_TO_SYNC);
	tegra_dc_writel(dc, mode->h_sync_width | (mode->v_sync_width << 16),
			DC_DISP_SYNC_WIDTH);
	tegra_dc_writel(dc, mode->h_back_porch | (mode->v_back_porch << 16),
			DC_DISP_BACK_PORCH);
	tegra_dc_writel(dc, mode->h_active | (mode->v_active << 16),
			DC_DISP_DISP_ACTIVE);
	tegra_dc_writel(dc, mode->h_front_porch | (mode->v_front_porch << 16),
			DC_DISP_FRONT_PORCH);

	tegra_dc_writel(dc, DE_SELECT_ACTIVE | DE_CONTROL_NORMAL,
			DC_DISP_DATA_ENABLE_OPTIONS);

	val = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_POLARITY1);
	if (mode->flags & TEGRA_DC_MODE_FLAG_NEG_V_SYNC)
		val |= PIN1_LVS_OUTPUT;
	else
		val &= ~PIN1_LVS_OUTPUT;

	if (mode->flags & TEGRA_DC_MODE_FLAG_NEG_H_SYNC)
		val |= PIN1_LHS_OUTPUT;
	else
		val &= ~PIN1_LHS_OUTPUT;
	tegra_dc_writel(dc, val, DC_COM_PIN_OUTPUT_POLARITY1);

	/* TODO: MIPI/CRT/HDMI clock cals */

	val = DISP_DATA_FORMAT_DF1P1C;

	if (dc->out->align == TEGRA_DC_ALIGN_MSB)
		val |= DISP_DATA_ALIGNMENT_MSB;
	else
		val |= DISP_DATA_ALIGNMENT_LSB;

	if (dc->out->order == TEGRA_DC_ORDER_RED_BLUE)
		val |= DISP_DATA_ORDER_RED_BLUE;
	else
		val |= DISP_DATA_ORDER_BLUE_RED;

	tegra_dc_writel(dc, val, DC_DISP_DISP_INTERFACE_CONTROL);

	rate = tegra_dc_clk_get_rate(dc);

	pclk = tegra_dc_pclk_round_rate(dc, mode->pclk);
	if (pclk < (mode->pclk / 100 * 99) ||
	    pclk > (mode->pclk / 100 * 109)) {
		dev_err(&dc->ndev->dev,
			"can't divide %ld clock to %d -1/+9%% %ld %d %d\n",
			rate, mode->pclk,
			pclk, (mode->pclk / 100 * 99),
			(mode->pclk / 100 * 109));
		return -EINVAL;
	}

	div = (rate * 2 / pclk) - 2;

	tegra_dc_writel(dc, 0x00010001,
			DC_DISP_SHIFT_CLOCK_OPTIONS);
	tegra_dc_writel(dc, PIXEL_CLK_DIVIDER_PCD1 | SHIFT_CLK_DIVIDER(div),
			DC_DISP_DISP_CLOCK_CONTROL);

	return 0;
}


int tegra_dc_set_mode(struct tegra_dc *dc, const struct tegra_dc_mode *mode)
{
	memcpy(&dc->mode, mode, sizeof(dc->mode));

	print_mode(dc, mode, __func__);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_set_mode);

int tegra_dc_set_fb_mode(struct tegra_dc *dc,
		const struct fb_videomode *fbmode, bool stereo_mode)
{
	struct tegra_dc_mode mode;

	mode.pclk = PICOS2KHZ(fbmode->pixclock) * 1000;
	mode.h_sync_width = fbmode->hsync_len;
	mode.v_sync_width = fbmode->vsync_len;
	mode.h_back_porch = fbmode->left_margin;
	mode.v_back_porch = fbmode->upper_margin;
	mode.h_active = fbmode->xres;
	mode.v_active = fbmode->yres;
	mode.h_front_porch = fbmode->right_margin;
	mode.v_front_porch = fbmode->lower_margin;
	mode.stereo_mode = stereo_mode;
	if (calc_ref_to_sync(&mode)) {
		dev_err(&dc->ndev->dev, "bad href/vref values, overriding.\n");
		mode.h_ref_to_sync = 11;
		mode.v_ref_to_sync = 1;
	}
	dev_info(&dc->ndev->dev, "Using mode %dx%d pclk=%d href=%d vref=%d\n",
		mode.h_active, mode.v_active, mode.pclk,
		mode.h_ref_to_sync, mode.v_ref_to_sync
	);

	if (mode.stereo_mode) {
		mode.pclk *= 2;
		/* total v_active = yres*2 + activespace */
		mode.v_active = fbmode->yres*2 +
				fbmode->vsync_len +
				fbmode->upper_margin +
				fbmode->lower_margin;
	}

	mode.flags = 0;

	if (!(fbmode->sync & FB_SYNC_HOR_HIGH_ACT))
		mode.flags |= TEGRA_DC_MODE_FLAG_NEG_H_SYNC;

	if (!(fbmode->sync & FB_SYNC_VERT_HIGH_ACT))
		mode.flags |= TEGRA_DC_MODE_FLAG_NEG_V_SYNC;

	return tegra_dc_set_mode(dc, &mode);
}
EXPORT_SYMBOL(tegra_dc_set_fb_mode);

void
tegra_dc_config_pwm(struct tegra_dc *dc, struct tegra_dc_pwm_params *cfg)
{
	unsigned int ctrl;
	unsigned long out_sel;
	unsigned long cmd_state;

	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	ctrl = ((cfg->period << PM_PERIOD_SHIFT) |
		(cfg->clk_div << PM_CLK_DIVIDER_SHIFT) |
		cfg->clk_select);

	/* The new value should be effected immediately */
	cmd_state = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
	tegra_dc_writel(dc, (cmd_state | (1 << 2)), DC_CMD_STATE_ACCESS);

	switch (cfg->which_pwm) {
	case TEGRA_PWM_PM0:
		/* Select the LM0 on PM0 */
		out_sel = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_SELECT5);
		out_sel &= ~(7 << 0);
		out_sel |= (3 << 0);
		tegra_dc_writel(dc, out_sel, DC_COM_PIN_OUTPUT_SELECT5);
		tegra_dc_writel(dc, ctrl, DC_COM_PM0_CONTROL);
		tegra_dc_writel(dc, cfg->duty_cycle, DC_COM_PM0_DUTY_CYCLE);
		break;
	case TEGRA_PWM_PM1:
		/* Select the LM1 on PM1 */
		out_sel = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_SELECT5);
		out_sel &= ~(7 << 4);
		out_sel |= (3 << 4);
		tegra_dc_writel(dc, out_sel, DC_COM_PIN_OUTPUT_SELECT5);
		tegra_dc_writel(dc, ctrl, DC_COM_PM1_CONTROL);
		tegra_dc_writel(dc, cfg->duty_cycle, DC_COM_PM1_DUTY_CYCLE);
		break;
	default:
		dev_err(&dc->ndev->dev, "Error\n");
		break;
	}
	tegra_dc_writel(dc, cmd_state, DC_CMD_STATE_ACCESS);
	mutex_unlock(&dc->lock);
}
EXPORT_SYMBOL(tegra_dc_config_pwm);

static void tegra_dc_set_out_pin_polars(struct tegra_dc *dc,
				const struct tegra_dc_out_pin *pins,
				const unsigned int n_pins)
{
	unsigned int i;

	int name;
	int pol;

	u32 pol1, pol3;

	u32 set1, unset1;
	u32 set3, unset3;

	set1 = set3 = unset1 = unset3 = 0;

	for (i = 0; i < n_pins; i++) {
		name = (pins + i)->name;
		pol  = (pins + i)->pol;

		/* set polarity by name */
		switch (name) {
		case TEGRA_DC_OUT_PIN_DATA_ENABLE:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set3 |= LSPI_OUTPUT_POLARITY_LOW;
			else
				unset3 |= LSPI_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_H_SYNC:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LHS_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LHS_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_V_SYNC:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LVS_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LVS_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_PIXEL_CLOCK:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LSC0_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LSC0_OUTPUT_POLARITY_LOW;
			break;
		default:
			printk("Invalid argument in function %s\n",
			       __FUNCTION__);
			break;
		}
	}

	pol1 = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_POLARITY1);
	pol3 = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_POLARITY3);

	pol1 |= set1;
	pol1 &= ~unset1;

	pol3 |= set3;
	pol3 &= ~unset3;

	tegra_dc_writel(dc, pol1, DC_COM_PIN_OUTPUT_POLARITY1);
	tegra_dc_writel(dc, pol3, DC_COM_PIN_OUTPUT_POLARITY3);
}

static void tegra_dc_set_out(struct tegra_dc *dc, struct tegra_dc_out *out)
{
	dc->out = out;

	if (out->n_modes > 0)
		tegra_dc_set_mode(dc, &dc->out->modes[0]);

	switch (out->type) {
	case TEGRA_DC_OUT_RGB:
		dc->out_ops = &tegra_dc_rgb_ops;
		break;

	case TEGRA_DC_OUT_HDMI:
		dc->out_ops = &tegra_dc_hdmi_ops;
		break;

	case TEGRA_DC_OUT_DSI:
		dc->out_ops = &tegra_dc_dsi_ops;
		break;

	default:
		dc->out_ops = NULL;
		break;
	}

	if (dc->out_ops && dc->out_ops->init)
		dc->out_ops->init(dc);

}

unsigned tegra_dc_get_out_height(struct tegra_dc *dc)
{
	if (dc->out)
		return dc->out->height;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_get_out_height);

unsigned tegra_dc_get_out_width(struct tegra_dc *dc)
{
	if (dc->out)
		return dc->out->width;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_get_out_width);

static void tegra_dc_vblank(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(work, struct tegra_dc, vblank_work);
	bool nvsd_updated = false;

	mutex_lock(&dc->lock);

	/* Update the SD brightness */
	nvsd_updated = nvsd_update_brightness(dc);

	mutex_unlock(&dc->lock);

	/* Do the actual brightness update outside of the mutex */
	if (nvsd_updated && dc->out->sd_settings &&
	    dc->out->sd_settings->bl_device) {

		struct platform_device *pdev = dc->out->sd_settings->bl_device;
		struct backlight_device *bl = platform_get_drvdata(pdev);
		if (bl)
			backlight_update_status(bl);
	}
}

static irqreturn_t tegra_dc_irq(int irq, void *ptr)
{
#ifndef CONFIG_TEGRA_FPGA_PLATFORM
	struct tegra_dc *dc = ptr;
	unsigned long status;
	unsigned long val;
	unsigned long underflow_mask;
	int i;

	status = tegra_dc_readl(dc, DC_CMD_INT_STATUS);
	tegra_dc_writel(dc, status, DC_CMD_INT_STATUS);

	/*
	 * Overlays can get thier internal state corrupted during and underflow
	 * condition.  The only way to fix this state is to reset the DC.
	 * if we get 4 consecutive frames with underflows, assume we're
	 * hosed and reset.
	 */
	underflow_mask = status & (WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT);
	if (underflow_mask) {
		val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
		val |= V_BLANK_INT;
		tegra_dc_writel(dc, val, DC_CMD_INT_ENABLE);
		dc->underflow_mask |= underflow_mask;
	}

	if (status & V_BLANK_INT) {
		int i;

		/* Check for any underflow reset conditions */
		for (i = 0; i< DC_N_WINDOWS; i++) {
			if (dc->underflow_mask & (WIN_A_UF_INT <<i)) {
				dc->windows[i].underflows++;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
				if (dc->windows[i].underflows > 4)
					schedule_work(&dc->reset_work);
#endif
			} else {
				dc->windows[i].underflows = 0;
			}
		}

		if (!dc->underflow_mask) {
			/* If we have no underflow to check, go ahead
			   and disable the interrupt */
			val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
			val &= ~V_BLANK_INT;
			tegra_dc_writel(dc, val, DC_CMD_INT_ENABLE);
		}

		/* Clear the underflow mask now that we've checked it. */
		dc->underflow_mask = 0;

		/* Schedule any additional bottom-half vblank actvities. */
		schedule_work(&dc->vblank_work);

		/* Mark the vblank as complete. */
		complete(&dc->vblank_complete);
	}

	if (status & FRAME_END_INT) {
		int completed = 0;
		int dirty = 0;

		val = tegra_dc_readl(dc, DC_CMD_STATE_CONTROL);
		for (i = 0; i < DC_N_WINDOWS; i++) {
			if (!(val & (WIN_A_UPDATE << i))) {
				dc->windows[i].dirty = 0;
				completed = 1;
			} else {
				dirty = 1;
			}
		}

		if (!dirty) {
			val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
			val &= ~FRAME_END_INT;
			tegra_dc_writel(dc, val, DC_CMD_INT_ENABLE);
		}

		if (completed) {
			if (!dirty) {
				/* With the last completed window, go ahead
				   and enable the vblank interrupt for nvsd. */
				val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
				val |= V_BLANK_INT;
				tegra_dc_writel(dc, val, DC_CMD_INT_ENABLE);

				val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
				val |= V_BLANK_INT;
				tegra_dc_writel(dc, val, DC_CMD_INT_MASK);
			}

			/* Wake up the workqueue regardless. */
			wake_up(&dc->wq);
		}
	}

	return IRQ_HANDLED;
#else
	return IRQ_NONE;
#endif
}

static void tegra_dc_set_color_control(struct tegra_dc *dc)
{
	u32 color_control;

	switch (dc->out->depth) {
	case 3:
		color_control = BASE_COLOR_SIZE111;
		break;

	case 6:
		color_control = BASE_COLOR_SIZE222;
		break;

	case 8:
		color_control = BASE_COLOR_SIZE332;
		break;

	case 9:
		color_control = BASE_COLOR_SIZE333;
		break;

	case 12:
		color_control = BASE_COLOR_SIZE444;
		break;

	case 15:
		color_control = BASE_COLOR_SIZE555;
		break;

	case 16:
		color_control = BASE_COLOR_SIZE565;
		break;

	case 18:
		color_control = BASE_COLOR_SIZE666;
		break;

	default:
		color_control = BASE_COLOR_SIZE888;
		break;
	}

	switch (dc->out->dither) {
	case TEGRA_DC_DISABLE_DITHER:
		color_control |= DITHER_CONTROL_DISABLE;
		break;
	case TEGRA_DC_ORDERED_DITHER:
		color_control |= DITHER_CONTROL_ORDERED;
		break;
	case TEGRA_DC_ERRDIFF_DITHER:
		/* The line buffer for error-diffusion dither is limited
		 * to 640 pixels per line. This limits the maximum
		 * horizontal active area size to 640 pixels when error
		 * diffusion is enabled.
		 */
		BUG_ON(dc->mode.h_active > 640);
		color_control |= DITHER_CONTROL_ERRDIFF;
		break;
	}

	tegra_dc_writel(dc, color_control, DC_DISP_DISP_COLOR_CONTROL);
}

static void tegra_dc_init(struct tegra_dc *dc)
{
	u32 disp_syncpt = 0;
	u32 vblank_syncpt = 0;
	int i;

	tegra_dc_writel(dc, 0x00000100, DC_CMD_GENERAL_INCR_SYNCPT_CNTRL);
	if (dc->ndev->id == 0) {
		disp_syncpt = NVSYNCPT_DISP0;
		vblank_syncpt = NVSYNCPT_VBLANK0;

		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0A,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0B,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0C,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY1B,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAYHC,
				      TEGRA_MC_PRIO_HIGH);
	} else if (dc->ndev->id == 1) {
		disp_syncpt = NVSYNCPT_DISP1;
		vblank_syncpt = NVSYNCPT_VBLANK1;

		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0AB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0BB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0CB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY1BB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAYHCB,
				      TEGRA_MC_PRIO_HIGH);
	}
	tegra_dc_writel(dc, 0x00000100 | vblank_syncpt, DC_CMD_CONT_SYNCPT_VSYNC);
	tegra_dc_writel(dc, 0x00004700, DC_CMD_INT_TYPE);
	tegra_dc_writel(dc, 0x0001c700, DC_CMD_INT_POLARITY);
	tegra_dc_writel(dc, 0x00202020, DC_DISP_MEM_HIGH_PRIORITY);
	tegra_dc_writel(dc, 0x00010101, DC_DISP_MEM_HIGH_PRIORITY_TIMER);

	tegra_dc_writel(dc, (FRAME_END_INT |
			     V_BLANK_INT |
			     WIN_A_UF_INT |
			     WIN_B_UF_INT |
			     WIN_C_UF_INT), DC_CMD_INT_MASK);
	tegra_dc_writel(dc, (WIN_A_UF_INT |
			     WIN_B_UF_INT |
			     WIN_C_UF_INT), DC_CMD_INT_ENABLE);

	tegra_dc_writel(dc, 0x00000000, DC_DISP_BORDER_COLOR);

	tegra_dc_set_color_control(dc);
	for (i = 0; i < DC_N_WINDOWS; i++) {
		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		tegra_dc_set_csc(dc);
		tegra_dc_set_scaling_filter(dc);
	}


	dc->syncpt_id = disp_syncpt;

	dc->syncpt_min = dc->syncpt_max =
		nvhost_syncpt_read(&dc->ndev->host->syncpt, disp_syncpt);

	print_mode(dc, &dc->mode, __func__);

	if (dc->mode.pclk)
		tegra_dc_program_mode(dc, &dc->mode);

	/* Initialize SD AFTER the modeset.
	   nvsd_init handles the sd_settings = NULL case. */
	nvsd_init(dc, dc->out->sd_settings);
}

static bool _tegra_dc_controller_enable(struct tegra_dc *dc)
{
	if (dc->out->enable)
		dc->out->enable();

	tegra_dc_setup_clk(dc, dc->clk);

	clk_enable(dc->clk);
	clk_enable(dc->emc_clk);
	enable_irq(dc->irq);

	tegra_dc_init(dc);

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	if (dc->out->out_pins)
		tegra_dc_set_out_pin_polars(dc, dc->out->out_pins,
					    dc->out->n_out_pins);

	if (dc->out->postpoweron)
		dc->out->postpoweron();

	/* force a full blending update */
	dc->blend.z[0] = -1;

	return true;
}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static bool _tegra_dc_controller_reset_enable(struct tegra_dc *dc)
{
	if (dc->out->enable)
		dc->out->enable();

	tegra_dc_setup_clk(dc, dc->clk);
	clk_enable(dc->clk);
	clk_enable(dc->emc_clk);

	if (dc->ndev->id == 0 && tegra_dcs[1] != NULL) {
		mutex_lock(&tegra_dcs[1]->lock);
		disable_irq(tegra_dcs[1]->irq);
	} else if (dc->ndev->id == 1 && tegra_dcs[0] != NULL) {
		mutex_lock(&tegra_dcs[0]->lock);
		disable_irq(tegra_dcs[0]->irq);
	}

	msleep(5);
	tegra_periph_reset_assert(dc->clk);
	msleep(2);
#ifndef CONFIG_TEGRA_FPGA_PLATFORM
	tegra_periph_reset_deassert(dc->clk);
	msleep(1);
#endif

	if (dc->ndev->id == 0 && tegra_dcs[1] != NULL) {
		enable_irq(tegra_dcs[1]->irq);
		mutex_unlock(&tegra_dcs[1]->lock);
	} else if (dc->ndev->id == 1 && tegra_dcs[0] != NULL) {
		enable_irq(tegra_dcs[0]->irq);
		mutex_unlock(&tegra_dcs[0]->lock);
	}

	enable_irq(dc->irq);

	tegra_dc_init(dc);

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	if (dc->out->out_pins)
		tegra_dc_set_out_pin_polars(dc, dc->out->out_pins,
					    dc->out->n_out_pins);

	if (dc->out->postpoweron)
		dc->out->postpoweron();

	/* force a full blending update */
	dc->blend.z[0] = -1;

	return true;
}
#endif

static bool _tegra_dc_enable(struct tegra_dc *dc)
{
	if (dc->mode.pclk == 0)
		return false;

	if (!dc->out)
		return false;

	tegra_dc_io_start(dc);

	return _tegra_dc_controller_enable(dc);
}

void tegra_dc_enable(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);

	if (!dc->enabled)
		dc->enabled = _tegra_dc_enable(dc);

	mutex_unlock(&dc->lock);
}

static void _tegra_dc_controller_disable(struct tegra_dc *dc)
{
	disable_irq(dc->irq);

	if (dc->out_ops && dc->out_ops->disable)
		dc->out_ops->disable(dc);

	clk_disable(dc->emc_clk);
	clk_disable(dc->clk);
	tegra_dvfs_set_rate(dc->clk, 0);

	if (dc->out && dc->out->disable)
		dc->out->disable();

	/* flush any pending syncpt waits */
	while (dc->syncpt_min < dc->syncpt_max) {
		dc->syncpt_min++;
		nvhost_syncpt_cpu_incr(&dc->ndev->host->syncpt, dc->syncpt_id);
	}
}

static void _tegra_dc_disable(struct tegra_dc *dc)
{
	_tegra_dc_controller_disable(dc);
	tegra_dc_io_end(dc);
}

void tegra_dc_disable(struct tegra_dc *dc)
{
	if (dc->overlay)
		tegra_overlay_disable(dc->overlay);

	mutex_lock(&dc->lock);

	if (dc->enabled) {
		dc->enabled = false;

		if (!dc->suspended)
			_tegra_dc_disable(dc);
	}

	mutex_unlock(&dc->lock);
}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static void tegra_dc_reset_worker(struct work_struct *work)
{
	struct tegra_dc *dc =
		container_of(work, struct tegra_dc, reset_work);

	unsigned long val = 0;

	dev_warn(&dc->ndev->dev, "overlay stuck in underflow state.  resetting.\n");

	mutex_lock(&shared_lock);
	mutex_lock(&dc->lock);

	if (dc->enabled == false || dc->suspended)
		goto unlock;

	dc->enabled = false;

	/*
	 * off host read bus
	 */
	val = tegra_dc_readl(dc, DC_CMD_CONT_SYNCPT_VSYNC);
	val &= ~(0x00000100);
	tegra_dc_writel(dc, val, DC_CMD_CONT_SYNCPT_VSYNC);

	/*
	 * set DC to STOP mode
	 */
	tegra_dc_writel(dc, DISP_CTRL_MODE_STOP, DC_CMD_DISPLAY_COMMAND);

	msleep(10);

	_tegra_dc_controller_disable(dc);

	/* _tegra_dc_controller_reset_enable deasserts reset */
	_tegra_dc_controller_reset_enable(dc);

	dc->enabled = true;
unlock:
	mutex_unlock(&dc->lock);
	mutex_unlock(&shared_lock);
}
#endif


static int tegra_dc_probe(struct nvhost_device *ndev)
{
	struct tegra_dc *dc;
	struct clk *clk;
	struct clk *emc_clk;
	struct resource	*res;
	struct resource *base_res;
	struct resource *fb_mem = NULL;
	int ret = 0;
	void __iomem *base;
	int irq;
	int i;
	unsigned long emc_clk_rate;

	if (!ndev->dev.platform_data) {
		dev_err(&ndev->dev, "no platform data\n");
		return -ENOENT;
	}

	dc = kzalloc(sizeof(struct tegra_dc), GFP_KERNEL);
	if (!dc) {
		dev_err(&ndev->dev, "can't allocate memory for tegra_dc\n");
		return -ENOMEM;
	}

	irq = nvhost_get_irq_byname(ndev, "irq");
	if (irq <= 0) {
		dev_err(&ndev->dev, "no irq\n");
		ret = -ENOENT;
		goto err_free;
	}

	res = nvhost_get_resource_byname(ndev, IORESOURCE_MEM, "regs");
	if (!res) {
		dev_err(&ndev->dev, "no mem resource\n");
		ret = -ENOENT;
		goto err_free;
	}

	base_res = request_mem_region(res->start, resource_size(res), ndev->name);
	if (!base_res) {
		dev_err(&ndev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto err_free;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&ndev->dev, "registers can't be mapped\n");
		ret = -EBUSY;
		goto err_release_resource_reg;
	}

	fb_mem = nvhost_get_resource_byname(ndev, IORESOURCE_MEM, "fbmem");

	clk = clk_get(&ndev->dev, NULL);
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&ndev->dev, "can't get clock\n");
		ret = -ENOENT;
		goto err_iounmap_reg;
	}

	emc_clk = clk_get(&ndev->dev, "emc");
	if (IS_ERR_OR_NULL(emc_clk)) {
		dev_err(&ndev->dev, "can't get emc clock\n");
		ret = -ENOENT;
		goto err_put_clk;
	}

	dc->clk = clk;
	dc->emc_clk = emc_clk;
	dc->base_res = base_res;
	dc->base = base;
	dc->irq = irq;
	dc->ndev = ndev;
	dc->pdata = ndev->dev.platform_data;

	/*
	 * The emc is a shared clock, it will be set based on
	 * the requirements for each user on the bus.
	 */
	emc_clk_rate = dc->pdata->emc_clk_rate;
	clk_set_rate(emc_clk, emc_clk_rate ? emc_clk_rate : ULONG_MAX);

	if (dc->pdata->flags & TEGRA_DC_FLAG_ENABLED)
		dc->enabled = true;

	mutex_init(&dc->lock);
	init_completion(&dc->vblank_complete);
	init_waitqueue_head(&dc->wq);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	INIT_WORK(&dc->reset_work, tegra_dc_reset_worker);
#endif
	INIT_WORK(&dc->vblank_work, tegra_dc_vblank);

	dc->n_windows = DC_N_WINDOWS;
	for (i = 0; i < dc->n_windows; i++) {
		dc->windows[i].idx = i;
		dc->windows[i].dc = dc;
	}

	if (request_irq(irq, tegra_dc_irq, IRQF_DISABLED,
			dev_name(&ndev->dev), dc)) {
		dev_err(&ndev->dev, "request_irq %d failed\n", irq);
		ret = -EBUSY;
		goto err_put_emc_clk;
	}

	/* hack to ballence enable_irq calls in _tegra_dc_enable() */
	disable_irq(dc->irq);

	ret = tegra_dc_add(dc, ndev->id);
	if (ret < 0) {
		dev_err(&ndev->dev, "can't add dc\n");
		goto err_free_irq;
	}

	nvhost_set_drvdata(ndev, dc);

	if (dc->pdata->default_out)
		tegra_dc_set_out(dc, dc->pdata->default_out);
	else
		dev_err(&ndev->dev, "No default output specified.  Leaving output disabled.\n");

	mutex_lock(&dc->lock);
	if (dc->enabled)
		_tegra_dc_enable(dc);
	mutex_unlock(&dc->lock);

	tegra_dc_dbg_add(dc);

	dev_info(&ndev->dev, "probed\n");

	if (dc->pdata->fb) {
		if (dc->pdata->fb->bits_per_pixel == -1) {
			unsigned long fmt;
			tegra_dc_writel(dc,
					WINDOW_A_SELECT << dc->pdata->fb->win,
					DC_CMD_DISPLAY_WINDOW_HEADER);

			fmt = tegra_dc_readl(dc, DC_WIN_COLOR_DEPTH);
			dc->pdata->fb->bits_per_pixel =
				tegra_dc_fmt_bpp(fmt);
		}

		dc->fb = tegra_fb_register(ndev, dc, dc->pdata->fb, fb_mem);
		if (IS_ERR_OR_NULL(dc->fb))
			dc->fb = NULL;
	}

	if (dc->fb) {
		dc->overlay = tegra_overlay_register(ndev, dc);
		if (IS_ERR_OR_NULL(dc->overlay))
			dc->overlay = NULL;
	}

	if (dc->out && dc->out->hotplug_init)
		dc->out->hotplug_init();

	if (dc->out_ops && dc->out_ops->detect)
		dc->out_ops->detect(dc);

	tegra_dc_create_sysfs(&dc->ndev->dev);

	return 0;

err_free_irq:
	free_irq(irq, dc);
err_put_emc_clk:
	clk_put(emc_clk);
err_put_clk:
	clk_put(clk);
err_iounmap_reg:
	iounmap(base);
	if (fb_mem)
		release_resource(fb_mem);
err_release_resource_reg:
	release_resource(base_res);
err_free:
	kfree(dc);

	return ret;
}

static int tegra_dc_remove(struct nvhost_device *ndev)
{
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);

	tegra_dc_remove_sysfs(&dc->ndev->dev);

	if (dc->overlay) {
		tegra_overlay_unregister(dc->overlay);
	}

	if (dc->fb) {
		tegra_fb_unregister(dc->fb);
		if (dc->fb_mem)
			release_resource(dc->fb_mem);
	}


	if (dc->enabled)
		_tegra_dc_disable(dc);

	free_irq(dc->irq, dc);
	clk_put(dc->emc_clk);
	clk_put(dc->clk);
	iounmap(dc->base);
	if (dc->fb_mem)
		release_resource(dc->base_res);
	kfree(dc);
	return 0;
}

#ifdef CONFIG_PM
static int tegra_dc_suspend(struct nvhost_device *ndev, pm_message_t state)
{
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);

	dev_info(&ndev->dev, "suspend\n");

	if (dc->overlay)
		tegra_overlay_disable(dc->overlay);

	mutex_lock(&dc->lock);

	if (dc->out_ops && dc->out_ops->suspend)
		dc->out_ops->suspend(dc);

	if (dc->enabled) {
		tegra_fb_suspend(dc->fb);
		_tegra_dc_disable(dc);

		dc->suspended = true;
	}

	if (dc->out && dc->out->postsuspend) {
		dc->out->postsuspend();
		msleep(100); /* avoid resume event due to voltage falling */
	}

	mutex_unlock(&dc->lock);

	return 0;
}

static int tegra_dc_resume(struct nvhost_device *ndev)
{
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);

	dev_info(&ndev->dev, "resume\n");

	mutex_lock(&dc->lock);
	dc->suspended = false;

	if (dc->enabled)
		_tegra_dc_enable(dc);

	if (dc->out && dc->out->hotplug_init)
		dc->out->hotplug_init();

	if (dc->out_ops && dc->out_ops->resume)
		dc->out_ops->resume(dc);
	mutex_unlock(&dc->lock);

	return 0;
}

#endif

extern int suspend_set(const char *val, struct kernel_param *kp)
{
	if (!strcmp(val, "dump"))
		dump_regs(tegra_dcs[0]);
#ifdef CONFIG_PM
	else if (!strcmp(val, "suspend"))
		tegra_dc_suspend(tegra_dcs[0]->ndev, PMSG_SUSPEND);
	else if (!strcmp(val, "resume"))
		tegra_dc_resume(tegra_dcs[0]->ndev);
#endif

	return 0;
}

extern int suspend_get(char *buffer, struct kernel_param *kp)
{
	return 0;
}

int suspend;

module_param_call(suspend, suspend_set, suspend_get, &suspend, 0644);

struct nvhost_driver tegra_dc_driver = {
	.driver = {
		.name = "tegradc",
		.owner = THIS_MODULE,
	},
	.probe = tegra_dc_probe,
	.remove = tegra_dc_remove,
#ifdef CONFIG_PM
	.suspend = tegra_dc_suspend,
	.resume = tegra_dc_resume,
#endif
};

static int __init tegra_dc_module_init(void)
{
	return nvhost_driver_register(&tegra_dc_driver);
}

static void __exit tegra_dc_module_exit(void)
{
	nvhost_driver_unregister(&tegra_dc_driver);
}

module_exit(tegra_dc_module_exit);
module_init(tegra_dc_module_init);
