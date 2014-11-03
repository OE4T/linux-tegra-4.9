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
#include "hw_nvdisp_nvdisp.h"
#include "hw_win_nvdisp.h"

#include "dc_config.h"
#include "dc_priv.h"

static int tegra_nvdisp_blend(struct tegra_dc_win *win)
{
	if (!(win->flags & TEGRA_WIN_BLEND_FLAGS_MASK)) {
		/* Set Bypassing if no blending is required */
		nvdisp_win_write(win,
			win_blend_layer_control_blend_enable_bypass_f(),
			win_blend_layer_control_r());
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
	bool yuvp = tegra_dc_is_yuv_planar(win->fmt);
	bool yuvsp = tegra_dc_is_yuv_semi_planar(win->fmt);

	nvdisp_win_write(win, tegra_dc_fmt(win->fmt), win_color_depth_r());
	nvdisp_win_write(win,
		win_position_v_position_f(win->out_y) |
		win_position_h_position_f(win->out_x),
		win_position_r());

	/* TODO: interlace size is different */
	nvdisp_win_write(win, win_size_v_size_f(win->out_h) |
		win_size_h_size_f(win->out_w),
		win_size_r());

	win_options = win_options_win_enable_enable_f();
	if (win->flags & TEGRA_WIN_FLAG_SCAN_COLUMN)
		win_options |= win_options_scan_column_enable_f();
	if (win->flags & TEGRA_WIN_FLAG_INVERT_H)
		win_options |= win_options_h_direction_decrement_f();
	if (win->flags & TEGRA_WIN_FLAG_INVERT_V)
		win_options |= win_options_v_direction_decrement_f();
	if (tegra_dc_fmt_bpp(win->fmt) < 24)
		win_options |= win_options_color_expand_enable_f();
	if (win->ppflags & TEGRA_WIN_PPFLAG_CP_ENABLE)
		win_options |= win_options_cp_enable_enable_f();
	nvdisp_win_write(win, win_options, win_options_r());

	nvdisp_win_write(win,
		win_set_cropped_size_in_height_f(dfixed_trunc(win->h)) |
		win_set_cropped_size_in_width_f(dfixed_trunc(win->w)),
		win_set_cropped_size_in_r());

	nvdisp_win_write(win, tegra_dc_reg_l32(win->phys_addr),
		win_start_addr_r());
	nvdisp_win_write(win, tegra_dc_reg_h32(win->phys_addr),
		win_start_addr_hi_r());

	/*	pitch is in 64B chunks	*/
	nvdisp_win_write(win, (win->stride>>6),
		win_set_planar_storage_r());

	if (yuvp) {
		nvdisp_win_write(win, tegra_dc_reg_l32(win->phys_addr_u),
			win_start_addr_u_r());
		nvdisp_win_write(win, tegra_dc_reg_h32(win->phys_addr_u),
			win_start_addr_hi_u_r());
		nvdisp_win_write(win, tegra_dc_reg_l32(win->phys_addr_v),
			win_start_addr_v_r());
		nvdisp_win_write(win, tegra_dc_reg_h32(win->phys_addr_v),
			win_start_addr_hi_v_r());

		nvdisp_win_write(win,
			win_set_planar_storage_uv_uv0_f(win->stride_uv>>6) |
			win_set_planar_storage_uv_uv1_f(win->stride_uv>>6),
			win_set_planar_storage_uv_r());
	} else if (yuvsp) {
		nvdisp_win_write(win, tegra_dc_reg_l32(win->phys_addr_u),
			win_start_addr_u_r());
		nvdisp_win_write(win, tegra_dc_reg_h32(win->phys_addr_u),
			win_start_addr_hi_u_r());

		nvdisp_win_write(win,
			win_set_planar_storage_uv_uv0_f(win->stride_uv>>6),
			win_set_planar_storage_uv_r());
	}

	if (WIN_IS_BLOCKLINEAR(win)) {
		nvdisp_win_write(win, win_surface_kind_kind_bl_f() |
			win_surface_kind_block_height_f(win->block_height_log2),
			win_surface_kind_r());
	} else if (WIN_IS_TILED(win)) {
		nvdisp_win_write(win, win_surface_kind_kind_tiled_f(),
			win_surface_kind_r());
	} else {
		nvdisp_win_write(win, win_surface_kind_kind_pitch_f(),
			win_surface_kind_r());
	}

	return 0;
}

int tegra_nvdisp_get_linestride(struct tegra_dc *dc, int win)
{
	return nvdisp_win_read(tegra_dc_get_window(dc, win),
		win_set_planar_storage_r()) << 6;
}

int tegra_nvdisp_update_windows(struct tegra_dc *dc,
	struct tegra_dc_win *windows[], int n,
	u16 *dirty_rect, bool wait_for_vblank)
{
	int i;
	u32 update_mask = nvdisp_cmd_state_ctrl_general_act_req_enable_f();
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

		update_mask |=
			nvdisp_cmd_state_ctrl_a_act_req_enable_f() << win->idx;

		if (wait_for_vblank)
			act_control = win_act_control_ctrl_sel_vcounter_f();
		else
			act_control = win_act_control_ctrl_sel_hcounter_f();

		nvdisp_win_write(win, act_control, win_act_control_r());

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
			nvdisp_cmd_int_status_r());

	tegra_dc_writel(dc, update_mask << 8 |
		nvdisp_cmd_state_ctrl_common_act_update_enable_f(),
		nvdisp_cmd_state_ctrl_r());

	if (wait_for_vblank) {
		/* Use the interrupt handler.  ISR will clear the dirty flags
		   when the flip is completed */
		set_bit(V_BLANK_FLIP, &dc->vblank_ref_count);
		tegra_dc_unmask_interrupt(dc,
			FRAME_END_INT | V_BLANK_INT | HEAD_UF_INT);
	}

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) {
		schedule_delayed_work(&dc->one_shot_work,
				msecs_to_jiffies(dc->one_shot_delay_ms));
	}
	dc->crc_pending = true;

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		update_mask |= (nvdisp_cmd_state_ctrl_host_trig_enable_f() |
			nvdisp_cmd_state_ctrl_common_act_req_enable_f());

	tegra_dc_writel(dc, update_mask, nvdisp_cmd_state_ctrl_r());

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
		win_set_control_owner_none_f(), win_set_control_r());

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
	nvdisp_win_write(win, dc->ctrl_num, win_set_control_r());

	mutex_unlock(&tegra_nvdisp_lock);
	return 0;
}
