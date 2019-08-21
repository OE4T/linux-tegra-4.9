/*
 * NVDLA OS Interface
 *
 * Copyright (c) 2016-2019, NVIDIA Corporation.  All rights reserved.
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

#ifndef DLA_FW_VERSION_H
#define DLA_FW_VERSION_H

#define FIRMWARE_VERSION_MAJOR		0x1UL
#define FIRMWARE_VERSION_MINOR		0x1UL
#define FIRMWARE_VERSION_SUBMINOR	0x3UL

static inline uint32_t dla_version(void)
{
	return (((FIRMWARE_VERSION_MAJOR & 0xffU) << 16) |
			((FIRMWARE_VERSION_MINOR & 0xffU) << 8) |
			((FIRMWARE_VERSION_SUBMINOR & 0xffU)));
}

#endif
