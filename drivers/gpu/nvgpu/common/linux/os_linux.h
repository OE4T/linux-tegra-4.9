/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef NVGPU_OS_LINUX_H
#define NVGPU_OS_LINUX_H

#include <linux/cdev.h>

#include "gk20a/gk20a.h"

struct nvgpu_os_linux {
	struct gk20a g;

	struct {
		struct cdev cdev;
		struct device *node;
	} channel;

	struct {
		struct cdev cdev;
		struct device *node;
	} ctrl;

	struct {
		struct cdev cdev;
		struct device *node;
	} as_dev;

	struct {
		struct cdev cdev;
		struct device *node;
	} dbg;

	struct {
		struct cdev cdev;
		struct device *node;
	} prof;

	struct {
		struct cdev cdev;
		struct device *node;
	} tsg;

	struct {
		struct cdev cdev;
		struct device *node;
	} ctxsw;

	struct {
		struct cdev cdev;
		struct device *node;
	} sched;

	dev_t cdev_region;
};

static inline struct nvgpu_os_linux *nvgpu_os_linux_from_gk20a(struct gk20a *g)
{
	return container_of(g, struct nvgpu_os_linux, g);
}

#define INTERFACE_NAME "nvhost%s-gpu"

#endif
