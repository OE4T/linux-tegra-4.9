/*
 * drivers/video/tegra/dc/ext/cursor.c
 *
 * Copyright (c) 2011-2012, NVIDIA CORPORATION, All rights reserved.
 *
 * Author: Robert Morell <rmorell@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <video/tegra_dc_ext.h>

#include "tegra_dc_ext_priv.h"

/* ugh */
#include "../dc_priv.h"
#include "../dc_reg.h"

int tegra_dc_ext_get_cursor(struct tegra_dc_ext_user *user)
{
	struct tegra_dc_ext *ext = user->ext;
	int ret = 0;

	mutex_lock(&ext->cursor.lock);

	if (!ext->cursor.user)
		ext->cursor.user = user;
	else if (ext->cursor.user != user)
		ret = -EBUSY;

	mutex_unlock(&ext->cursor.lock);

	return ret;
}

int tegra_dc_ext_put_cursor(struct tegra_dc_ext_user *user)
{
	struct tegra_dc_ext *ext = user->ext;
	int ret = 0;

	mutex_lock(&ext->cursor.lock);

	if (ext->cursor.user == user)
		ext->cursor.user = 0;
	else
		ret = -EACCES;

	mutex_unlock(&ext->cursor.lock);

	return ret;
}

static void set_cursor_image_hw(struct tegra_dc *dc,
				struct tegra_dc_ext_cursor_image *args,
				dma_addr_t phys_addr)
{
	int clip_win;

	tegra_dc_writel(dc,
		CURSOR_COLOR(args->foreground.r,
			     args->foreground.g,
			     args->foreground.b),
		DC_DISP_CURSOR_FOREGROUND);
	tegra_dc_writel(dc,
		CURSOR_COLOR(args->background.r,
			     args->background.g,
			     args->background.b),
		DC_DISP_CURSOR_BACKGROUND);

	BUG_ON(phys_addr & ~CURSOR_START_ADDR_MASK);

	/* Get the cursor clip window number */
	clip_win = CURSOR_CLIP_GET_WINDOW(tegra_dc_readl(dc,
					  DC_DISP_CURSOR_START_ADDR));

	tegra_dc_writel(dc, CURSOR_CLIP_SHIFT_BITS(clip_win) |
		CURSOR_START_ADDR(((unsigned long) phys_addr)) |
		((args->flags & TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_64x64) ?
			CURSOR_SIZE_64 : 0),
		DC_DISP_CURSOR_START_ADDR);
}

int tegra_dc_ext_set_cursor_image(struct tegra_dc_ext_user *user,
				  struct tegra_dc_ext_cursor_image *args)
{
	struct tegra_dc_ext *ext = user->ext;
	struct tegra_dc *dc = ext->dc;
	struct nvmap_handle_ref *handle, *old_handle;
	dma_addr_t phys_addr;
	u32 size;
	int ret;

	if (!user->nvmap)
		return -EFAULT;

	size = args->flags & (TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_32x32 |
			      TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_64x64);

	if (size != TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_32x32 &&
	    size !=  TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_64x64)
		return -EINVAL;

	mutex_lock(&ext->cursor.lock);

	if (ext->cursor.user != user) {
		ret = -EACCES;
		goto unlock;
	}

	if (!ext->enabled) {
		ret = -ENXIO;
		goto unlock;
	}

	old_handle = ext->cursor.cur_handle;

	ret = tegra_dc_ext_pin_window(user, args->buff_id, &handle, &phys_addr);
	if (ret)
		goto unlock;

	ext->cursor.cur_handle = handle;

	mutex_lock(&dc->lock);
	tegra_dc_io_start(dc);
	tegra_dc_hold_dc_out(dc);

	set_cursor_image_hw(dc, args, phys_addr);

	tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_dc_release_dc_out(dc);
	tegra_dc_io_end(dc);
	/* XXX sync here? */

	mutex_unlock(&dc->lock);

	mutex_unlock(&ext->cursor.lock);

	if (old_handle) {
		nvmap_unpin(ext->nvmap, old_handle);
		nvmap_free(ext->nvmap, old_handle);
	}

	return 0;

unlock:
	mutex_unlock(&ext->cursor.lock);

	return ret;
}

int tegra_dc_ext_set_cursor(struct tegra_dc_ext_user *user,
			    struct tegra_dc_ext_cursor *args)
{
	struct tegra_dc_ext *ext = user->ext;
	struct tegra_dc *dc = ext->dc;
	u32 win_options;
	bool enable;
	int ret;

	mutex_lock(&ext->cursor.lock);

	if (ext->cursor.user != user) {
		ret = -EACCES;
		goto unlock;
	}

	if (!ext->enabled) {
		ret = -ENXIO;
		goto unlock;
	}

	enable = !!(args->flags & TEGRA_DC_EXT_CURSOR_FLAGS_VISIBLE);

	mutex_lock(&dc->lock);
	tegra_dc_io_start(dc);
	tegra_dc_hold_dc_out(dc);

	win_options = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	if (!!(win_options & CURSOR_ENABLE) != enable) {
		win_options &= ~CURSOR_ENABLE;
		if (enable)
			win_options |= CURSOR_ENABLE;
		tegra_dc_writel(dc, win_options, DC_DISP_DISP_WIN_OPTIONS);
	}

	tegra_dc_writel(dc, CURSOR_POSITION(args->x, args->y),
		DC_DISP_CURSOR_POSITION);

	tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	/* TODO: need to sync here?  hopefully can avoid this, but need to
	 * figure out interaction w/ rest of GENERAL_ACT_REQ */

	tegra_dc_release_dc_out(dc);
	tegra_dc_io_end(dc);
	mutex_unlock(&dc->lock);

	mutex_unlock(&ext->cursor.lock);

	return 0;

unlock:
	mutex_unlock(&ext->cursor.lock);

	return ret;
}

int tegra_dc_ext_cursor_clip(struct tegra_dc_ext_user *user,
			    int *args)
{
	struct tegra_dc_ext *ext = user->ext;
	struct tegra_dc *dc = ext->dc;
	int ret;
	unsigned long reg_val;

	mutex_lock(&ext->cursor.lock);

	if (ext->cursor.user != user) {
		ret = -EACCES;
		goto unlock;
	}

	if (!ext->enabled) {
		ret = -ENXIO;
		goto unlock;
	}

	mutex_lock(&dc->lock);
	tegra_dc_io_start(dc);
	tegra_dc_hold_dc_out(dc);

	reg_val = tegra_dc_readl(dc, DC_DISP_CURSOR_START_ADDR);
	reg_val &= ~CURSOR_CLIP_SHIFT_BITS(3); /* Clear out the old value */
	tegra_dc_writel(dc, reg_val | CURSOR_CLIP_SHIFT_BITS(*args),
			DC_DISP_CURSOR_START_ADDR);

	tegra_dc_release_dc_out(dc);
	tegra_dc_io_end(dc);
	mutex_unlock(&dc->lock);

	mutex_unlock(&ext->cursor.lock);

	return 0;

unlock:
	mutex_unlock(&ext->cursor.lock);

	return ret;
}
