/**
 * Copyright (c) 2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef __MAX9295_H__
#define __MAX9295_H__

#include <media/gmsl-link.h>

int max9295_poweron(struct device *dev);

int max9295_poweroff(struct device *dev);

int max9295_dev_pair(struct device *dev, struct gmsl_link_data *pinfo);

int max9295_dev_unpair(struct device *dev, struct device *s_dev);

int max9295_stream_setup(struct device *dev);

int max9295_streamon(struct device *dev);

int max9295_streamoff(struct device *dev);

#endif  /* __MAX9295_H__ */
