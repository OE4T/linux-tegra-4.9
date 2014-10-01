/*
 * drivers/video/tegra/dc/nvdisplay/nvdis_win.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All rights reserved.
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


#include <mach/dc.h>

#include "nvdisp.h"
#include "nvdisp_priv.h"
#include "nvdisp_reg.h"

#include "dc_reg.h"
#include "dc_config.h"
#include "dc_priv.h"

static int tegra_nvdisp_blend(struct tegra_dc_win *win)
{
	if (!(win->flags & TEGRA_WIN_BLEND_FLAGS_MASK)) {
		/* Set Bypassing if no blending is required */
		nvdisp_win_write(win, BLEND_BYPASS,
			DC_WIN_BLEND_LAYER_CONTROL);
		return 0;
	}

	/* TODO: to support different blending mode */
	return 0;
}

static int tegra_nvdisp_scaling(struct tegra_dc_win *win)
{
	/* TODO */
	return 0;
}

static int tegra_nvdisp_enable_cde(struct tegra_dc_win *win)
{
	/* TODO */
	return 0;
}

static int tegra_nvdisp_win_attribute(struct tegra_dc_win *win)
{
	u32 win_options;
	unsigned bpp = tegra_dc_fmt_bpp(win->fmt) / 8;

	nvdisp_win_write(win, tegra_dc_fmt(win->fmt), DC_WIN_COLOR_DEPTH);
	nvdisp_win_write(win,
		V_POSITION(win->out_y) | H_POSITION(win->out_x),
		DC_WIN_POSITION);

	/* TODO: interlace size is different */
	nvdisp_win_write(win, V_SIZE(win->out_h) | H_SIZE(win->out_w),
		DC_WIN_SIZE);

	win_options = WIN_ENABLE;
	if (win->flags & TEGRA_WIN_FLAG_SCAN_COLUMN)
		win_options |= WIN_SCAN_COLUMN;
	if (win->flags & TEGRA_WIN_FLAG_INVERT_H)
		win_options |= WIN_INVERT_H;
	if (win->flags & TEGRA_WIN_FLAG_INVERT_V)
		win_options |= WIN_INVERT_V;
	if (tegra_dc_fmt_bpp(win->fmt) < 24)
		win_options |= COLOR_EXPAND;
	if (win->ppflags & TEGRA_WIN_PPFLAG_CP_ENABLE)
		win_options |= CP_ENABLE;
	nvdisp_win_write(win, win_options, DC_WIN_WIN_OPTIONS);

	nvdisp_win_write(win,
			V_PRESCALED_SIZE(dfixed_trunc(win->h)) |
			H_PRESCALED_SIZE(dfixed_trunc(win->w) * bpp),
			WIN_PCALC_WINDOW_SET_CROPPED_SIZE_IN);

	nvdisp_win_write(win, tegra_dc_reg_l32(win->phys_addr),
		DC_WINBUF_START_ADDR);
	nvdisp_win_write(win, tegra_dc_reg_h32(win->phys_addr),
		DC_WINBUF_START_ADDR_HI);
	/* Change to WIN_SET_PLANAR_STORAGE later instead of  line_stride*/
	/*nvdisp_win_write(win, (win->stride>>6), WIN_SET_PLANAR_STORAGE);*/

	nvdisp_win_write(win, win->stride, DC_WIN_LINE_STRIDE);
	/* TODO: program related YUV registers as well */

	/* TODO: confirm ADDR_H/V_OFFSET programming not needed anymore */
	if (WIN_IS_BLOCKLINEAR(win)) {
		nvdisp_win_write(win, DC_WIN_BUFFER_SURFACE_BL_16B2 |
			(win->block_height_log2 << BLOCK_HEIGHT_SHIFT),
			DC_WIN_BUFFER_SURFACE_KIND);
	} else if (WIN_IS_TILED(win)) {
		nvdisp_win_write(win, DC_WIN_BUFFER_SURFACE_TILED,
			DC_WIN_BUFFER_SURFACE_KIND);
	} else {
		nvdisp_win_write(win, DC_WIN_BUFFER_SURFACE_PITCH,
			DC_WIN_BUFFER_SURFACE_KIND);
	}

	return 0;
}

int tegra_nvdisp_get_linestride(struct tegra_dc *dc, int win)
{
	/* Change to WIN_SET_PLANAR_STORAGE later instead of  line_stride*/
	/*return nvdisp_win_read(tegra_dc_get_window(dc, win),
					WIN_SET_PLANAR_STORAGE);*/
	return nvdisp_win_read(tegra_dc_get_window(dc, win),
					DC_WIN_LINE_STRIDE);
}

int tegra_nvdisp_update_windows(struct tegra_dc *dc,
	struct tegra_dc_win *windows[], int n,
	u16 *dirty_rect, bool wait_for_vblank)
{
	int i;
	u32 update_mask = GENERAL_ACT_REQ;
	u32 act_control = 0;

	for (i = 0; i < n; i++) {
		struct tegra_dc_win *win = windows[i];
		struct tegra_dc_win *dc_win = tegra_dc_get_window(dc, win->idx);

		if (!win || !dc_win) {
			dev_err(&dc->ndev->dev, "Invalid window %d to update\n",
				n);
			return -EINVAL;
		}
		if (!WIN_IS_ENABLED(win)) {
			dc_win->dirty = no_vsync ? 0 : 1;
			/* TODO: disable this window */
			continue;
		}

		update_mask |= WIN_A_ACT_REQ << win->idx;

		if (wait_for_vblank)
			act_control &= ~WIN_ACT_CNTR_SEL_HCOUNTER(win->idx);
		else
			act_control |= WIN_ACT_CNTR_SEL_HCOUNTER(win->idx);

		tegra_nvdisp_blend(win);
		tegra_nvdisp_scaling(win);
		if (win->cde.cde_addr)
			tegra_nvdisp_enable_cde(win);

		/* if (do_partial_update) { */
			/* /\* calculate the xoff, yoff etc values *\/ */
			/* tegra_dc_win_partial_update(dc, win, xoff, yoff, */
			/* 	width, height); */
		/* } */

		tegra_nvdisp_win_attribute(win);

		dc_win->dirty = 1;
		win->dirty = 1;

		trace_window_update(dc, win);
	}

	if (tegra_cpu_is_asim())
		tegra_dc_writel(dc, FRAME_END_INT | V_BLANK_INT,
			DC_CMD_INT_STATUS);

	tegra_dc_writel(dc, update_mask << 8, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, act_control, DC_CMD_REG_ACT_CONTROL);

	if (wait_for_vblank) {
		/* Use the interrupt handler.  ISR will clear the dirty flags
		   when the flip is completed */
		set_bit(V_BLANK_FLIP, &dc->vblank_ref_count);
		tegra_dc_unmask_interrupt(dc,
			FRAME_END_INT | V_BLANK_INT | ALL_UF_INT());
	}

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) {
		schedule_delayed_work(&dc->one_shot_work,
				msecs_to_jiffies(dc->one_shot_delay_ms));
	}
	dc->crc_pending = true;

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		update_mask |= NC_HOST_TRIG;

	tegra_dc_writel(dc, update_mask, DC_CMD_STATE_CONTROL);

	return 0;
}


/* detach window idx from current head */
int tegra_nvdisp_detach_win(struct tegra_dc *dc, unsigned idx)
{
	struct tegra_dc_win *win = tegra_dc_get_window(dc, idx);

	if (!win || win->dc != dc) {
		dev_err(&dc->ndev->dev,
			"%s: window %d does not belong to head %d\n",
			__func__, idx, dc->ctrl_num);
		return -EINVAL;
	}

	mutex_lock(&tegra_nvdisp_lock);

	/* detach window idx */
	nvdisp_win_write(win,
		SET_CONTROL_NONE,
		WIN_CORE_WINDOWGROUP_SET_CONTROL);


	dc->valid_windows &= ~(0x1 << idx);
	win->dc = NULL;
	mutex_unlock(&tegra_nvdisp_lock);
	return 0;
}


/* Assign window idx to head dc */
int tegra_nvdisp_assign_win(struct tegra_dc *dc, unsigned idx)
{
	struct tegra_dc_win *win = tegra_dc_get_window(dc, idx);

	if (win && win->dc == dc) /* already assigned to current head */
		return 0;

	mutex_lock(&tegra_nvdisp_lock);
	dc->valid_windows |= 0x1 << idx;

	win = tegra_dc_get_window(dc, idx);

	if (win->dc) {		/* window is owned by another head */
		dev_err(&dc->ndev->dev,
			"%s: cannot assign win %d to head %d, it owned by %d\n",
			__func__, idx, dc->ctrl_num, win->dc->ctrl_num);
		dc->valid_windows &= ~(0x1 << idx);
		mutex_unlock(&tegra_nvdisp_lock);
		return -EINVAL;
	}

	win->dc = dc;

	/* attach window idx */
	nvdisp_win_write(win, dc->ctrl_num, WIN_CORE_WINDOWGROUP_SET_CONTROL);

	mutex_unlock(&tegra_nvdisp_lock);
	return 0;
}
