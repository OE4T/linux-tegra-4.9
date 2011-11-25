/*
 * drivers/video/tegra/host/t20/cdma_t20.h
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __NVHOST_CDMA_T20_H
#define __NVHOST_CDMA_T20_H

/* Size of the sync queue. If it is too small, we won't be able to queue up
 * many command buffers. If it is too large, we waste memory. */
#define NVHOST_SYNC_QUEUE_SIZE 512

/* Number of gathers we allow to be queued up per channel. Must be a
 * power of two. Currently sized such that pushbuffer is 4KB (512*8B). */
#define NVHOST_GATHER_QUEUE_SIZE 512

/* 8 bytes per slot. (This number does not include the final RESTART.) */
#define PUSH_BUFFER_SIZE (NVHOST_GATHER_QUEUE_SIZE * 8)

/* 4K page containing GATHERed methods to increment channel syncpts
 * and replaces the original timed out contexts GATHER slots */
#define SYNCPT_INCR_BUFFER_SIZE_WORDS   (4096 / sizeof(u32))

#endif
