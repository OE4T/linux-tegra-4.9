/*
 * drivers/video/tegra/dc/ext/tegra_dc_ext_priv.h
 *
 * Copyright (C) 2011, NVIDIA Corporation
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

#ifndef __TEGRA_DC_EXT_PRIV_H
#define __TEGRA_DC_EXT_PRIV_H

#include <linux/cdev.h>
#include <linux/mutex.h>

#include <mach/dc.h>
#include <mach/nvmap.h>

#include <video/tegra_dc_ext.h>

struct tegra_dc_ext;

struct tegra_dc_ext_user {
	struct tegra_dc_ext	*ext;
	struct nvmap_client	*nvmap;
};

struct tegra_dc_ext_win {
	struct tegra_dc_ext	*ext;

	int			idx;

	struct tegra_dc_ext_user *user;

	struct mutex		lock;

	struct nvmap_handle_ref	*cur_handle;

	struct workqueue_struct	*flip_wq;
};

struct tegra_dc_ext {
	struct tegra_dc			*dc;

	struct cdev			cdev;
	struct device			*dev;

	struct nvmap_client		*nvmap;

	struct tegra_dc_ext_win		win[DC_N_WINDOWS];

	struct {
		struct tegra_dc_ext_user	*user;
		struct nvmap_handle_ref		*cur_handle;
		struct mutex			lock;
	} cursor;
};

extern int tegra_dc_ext_pin_window(struct tegra_dc_ext_user *user, u32 id,
				   struct nvmap_handle_ref **handle,
				   dma_addr_t *phys_addr);

extern int tegra_dc_ext_get_cursor(struct tegra_dc_ext_user *user);
extern int tegra_dc_ext_put_cursor(struct tegra_dc_ext_user *user);
extern int tegra_dc_ext_set_cursor_image(struct tegra_dc_ext_user *user,
					 struct tegra_dc_ext_cursor_image *);
extern int tegra_dc_ext_set_cursor(struct tegra_dc_ext_user *user,
				   struct tegra_dc_ext_cursor *);

#endif /* __TEGRA_DC_EXT_PRIV_H */
