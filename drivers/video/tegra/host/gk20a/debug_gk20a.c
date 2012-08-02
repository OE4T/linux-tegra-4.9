/*
 * drivers/video/tegra/host/t20/debug_gk20a.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/io.h>

#include "../dev.h"
#include "../debug.h"
#include "../nvhost_cdma.h"

#include "gk20a.h"


void gk20a_debug_show_channel_cdma(struct nvhost_master *m,
	struct nvhost_channel *ch, struct output *o, int chid)
{
}

void gk20a_debug_show_channel_fifo(struct nvhost_master *m,
	struct nvhost_channel *ch, struct output *o, int chid)
{
}
