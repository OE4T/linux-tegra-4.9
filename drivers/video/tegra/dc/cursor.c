/*
 * drivers/video/tegra/dc/ext/cursor.c
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION, All rights reserved.
 *
 * Author:
 *  Robert Morell <rmorell@nvidia.com>
 *  Jon Mayo <jmayo@nvidia.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>

#include "dc_priv.h"
#include "dc_reg.h"

/* modify val with cursor field set for a given size.
 * ignore val if it is NULL.
 * return non-zero on error, and clear val. */
static inline int cursor_size_value(enum tegra_dc_cursor_size size, u32 *val)
{
	u32 scratch = 0;
	if (!val)
		val = &scratch;
	switch (size) {
	case TEGRA_DC_CURSOR_SIZE_32X32:
		*val |= CURSOR_SIZE_32;
		return 0;
	case TEGRA_DC_CURSOR_SIZE_64X64:
		*val |= CURSOR_SIZE_64;
		return 0;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	case TEGRA_DC_CURSOR_SIZE_128X128:
		*val |= CURSOR_SIZE_128;
		return 0;
	case TEGRA_DC_CURSOR_SIZE_256X256:
		*val |= CURSOR_SIZE_256;
		return 0;
#endif
	}
	*val = 0;
	return -EINVAL;
}

/* modify val with cursor format.
 * ignore val if it is NULL.
 * return non-zero on error, and clear val. */
static inline u32 cursor_format_value(enum tegra_dc_cursor_format format,
	u32 *val)
{
	u32 scratch = 0;
	if (!val)
		val = &scratch;
	switch (format) {
	case TEGRA_DC_CURSOR_FORMAT_2BIT_LEGACY:
		*val |= CURSOR_MODE_SELECT(0);
		return 0;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_3x_SOC)
	case TEGRA_DC_CURSOR_FORMAT_RGBA_NON_PREMULT_ALPHA:
		*val |= CURSOR_MODE_SELECT(1);
# if !defined(CONFIG_ARCH_TEGRA_11x_SOC)
		*val |= CURSOR_ALPHA(255) | CURSOR_DST_BLEND_FACTOR_SELECT(2);
		*val |= CURSOR_SRC_BLEND_FACTOR_SELECT(1);
# endif
		return 0;
#endif
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_3x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_11x_SOC)
	case TEGRA_DC_CURSOR_FORMAT_RGBA_PREMULT_ALPHA:
		*val |= CURSOR_MODE_SELECT(1);
		*val |= CURSOR_ALPHA(255) | CURSOR_DST_BLEND_FACTOR_SELECT(2);
		*val |= CURSOR_SRC_BLEND_FACTOR_SELECT(0);
		return 0;
#endif
	}
	*val = 0;
	return -EINVAL;
}

static unsigned int set_cursor_start_addr(struct tegra_dc *dc,
	enum tegra_dc_cursor_size size, dma_addr_t phys_addr)
{
	u32 val = 0;
	int clip_win;

	BUG_ON(phys_addr & ~CURSOR_START_ADDR_MASK);

	/* this should not fail, as tegra_dc_cursor_image() checks the size */
	if (WARN(cursor_size_value(size, &val), "invalid cursor size."))
		return 0;

	/* Get the cursor clip window number */
	/* TODO: load clip setting from data structure */
	clip_win = CURSOR_CLIP_GET_WINDOW(
		tegra_dc_readl(dc, DC_DISP_CURSOR_START_ADDR));
	val |= CURSOR_CLIP_SHIFT_BITS(clip_win);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC) || \
	defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
	tegra_dc_writel(dc, val | CURSOR_START_ADDR(((unsigned long)phys_addr)),
		DC_DISP_CURSOR_START_ADDR);
#else
	/* TODO: check calculation with HW */
	tegra_dc_writel(dc, (u32)(CURSOR_START_ADDR_HI(phys_addr)),
		DC_DISP_CURSOR_START_ADDR_HI);
	tegra_dc_writel(dc, (u32)(val | CURSOR_START_ADDR_LOW(phys_addr)),
		DC_DISP_CURSOR_START_ADDR);
#endif

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_3x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_11x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_14x_SOC)
	tegra_dc_writel(dc, CURSOR_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, CURSOR_ACT_REQ, DC_CMD_STATE_CONTROL);
	return 0;
#else
	return 1;
#endif
}

static int set_cursor_position(struct tegra_dc *dc, s16 x, s16 y)
{
	tegra_dc_writel(dc, CURSOR_POSITION(x, y), DC_DISP_CURSOR_POSITION);

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_3x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_11x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_14x_SOC)
	tegra_dc_writel(dc, CURSOR_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, CURSOR_ACT_REQ, DC_CMD_STATE_CONTROL);
	return 0;
#else
	return 1;
#endif
}

static int set_cursor_activation_control(struct tegra_dc *dc)
{
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_3x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_11x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_14x_SOC)
	u32 reg = tegra_dc_readl(dc, DC_CMD_REG_ACT_CONTROL);

	if ((reg & (1 << CURSOR_ACT_CNTR_SEL)) ==
	    (CURSOR_ACT_CNTR_SEL_V << CURSOR_ACT_CNTR_SEL)) {
		reg &= ~(1 << CURSOR_ACT_CNTR_SEL);
		reg |= (CURSOR_ACT_CNTR_SEL_V << CURSOR_ACT_CNTR_SEL);
		tegra_dc_writel(dc, reg, DC_CMD_REG_ACT_CONTROL);
		return 1;
	}
#endif
	return 0;
}

static int set_cursor_enable(struct tegra_dc *dc, bool enable)
{
	u32 val = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	if (!!(val & CURSOR_ENABLE) != enable) {
		val &= ~CURSOR_ENABLE;
		if (enable)
			val |= CURSOR_ENABLE;
		tegra_dc_writel(dc, val, DC_DISP_DISP_WIN_OPTIONS);
		return 1;
	}
	return 0;
}

static int set_cursor_blend(struct tegra_dc *dc, u32 format)
{
	u32 val = tegra_dc_readl(dc, DC_DISP_BLEND_CURSOR_CONTROL);

	u32 newval = WINH_CURS_SELECT(0);

	/* this should not fail, as tegra_dc_cursor_image() checks the format */
	if (WARN(cursor_format_value(format, &newval), "invalid cursor format"))
		return 0;

	if (val != newval) {
		tegra_dc_writel(dc, newval, DC_DISP_BLEND_CURSOR_CONTROL);
		return 1;
	}

	return 0;
}

static int set_cursor_fg_bg(struct tegra_dc *dc, u32 fg, u32 bg)
{
	int general_update_needed = 0;
	/* TODO: check fg/bg against data structure, don't read the HW */
	if (fg != tegra_dc_readl(dc, DC_DISP_CURSOR_FOREGROUND)) {
		tegra_dc_writel(dc, fg, DC_DISP_CURSOR_FOREGROUND);
		general_update_needed |= 1;
	}

	if (bg != tegra_dc_readl(dc, DC_DISP_CURSOR_BACKGROUND)) {
		tegra_dc_writel(dc, bg, DC_DISP_CURSOR_BACKGROUND);
		general_update_needed |= 1;
	}

	return general_update_needed;
}

int tegra_dc_cursor_image(struct tegra_dc *dc,
	enum tegra_dc_cursor_format format, enum tegra_dc_cursor_size size,
	u32 fg, u32 bg, dma_addr_t phys_addr)
{
	int need_general_update = 0;

	if (cursor_size_value(size, NULL))
		return -EINVAL;

	if (cursor_format_value(format, NULL))
		return -EINVAL;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	need_general_update |= set_cursor_start_addr(dc, size, phys_addr);

	need_general_update |= set_cursor_fg_bg(dc, fg, bg);

	need_general_update |= set_cursor_blend(dc, format);

	if (need_general_update) {
		tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
		tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	}

	tegra_dc_put(dc);
	/* XXX sync here? */

	mutex_unlock(&dc->lock);

	return 0;
}

int tegra_dc_cursor_set(struct tegra_dc *dc, bool enable, int x, int y)
{
	int need_general_update = 0;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	need_general_update |= set_cursor_enable(dc, enable);

	need_general_update |= set_cursor_position(dc, x, y);

	need_general_update |= set_cursor_activation_control(dc);

	if (need_general_update) {
		tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
		tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	}

	/* TODO: need to sync here?  hopefully can avoid this, but need to
	 * figure out interaction w/ rest of GENERAL_ACT_REQ */

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return 0;
}

/* clip:
 * 0 - display
 * 1 - window A
 * 2 - window B
 * 3 - window C
 */
int tegra_dc_cursor_clip(struct tegra_dc *dc, unsigned clip)
{
	unsigned long reg_val;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	/* TODO: load start address from data structure */
	reg_val = tegra_dc_readl(dc, DC_DISP_CURSOR_START_ADDR);
	reg_val &= ~CURSOR_CLIP_SHIFT_BITS(3); /* Clear out the old value */
	tegra_dc_writel(dc, reg_val | CURSOR_CLIP_SHIFT_BITS(clip),
		DC_DISP_CURSOR_START_ADDR);

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_3x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_11x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_14x_SOC)
	tegra_dc_writel(dc, CURSOR_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, CURSOR_ACT_REQ, DC_CMD_STATE_CONTROL);
#else
	tegra_dc_writel(dc, GENERAL_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
#endif

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return 0;
}
