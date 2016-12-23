/*
 * nvlink.h:
 * This header contains the structures and APIs needed by the NVLINK core and
 * endpoint drivers for interacting with each other.
 *
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

#ifndef NVLINK_H
#define NVLINK_H

enum nvlink_log_categories {
	nvlink_log_err	= BIT(0),	/* Error prints - these will be printed
					   unconditionally */
	nvlink_log_dbg	= BIT(1),	/* Debug prints */
};

extern u32 nvlink_log_mask;

#define NVLINK_DEFAULT_LOG_MASK	nvlink_log_err

#define nvlink_print(log_mask, fmt, arg...)		\
	do {						\
		if ((log_mask) & nvlink_log_mask)	\
			printk("%s: %s: %d: " fmt "\n",	\
				NVLINK_DRV_NAME,	\
				__func__,		\
				__LINE__,		\
				##arg);			\
	} while (0)

#define nvlink_err(fmt, arg...)	nvlink_print(nvlink_log_err, fmt, ##arg)
#define nvlink_dbg(fmt, arg...)	nvlink_print(nvlink_log_dbg, fmt, ##arg)

enum nvlink_endpt {
	NVLINK_ENDPT_TEGRA
};

/* Structure needed by endpoint drivers for registering with the core driver */
struct nvlink_endpt_drv {
	enum nvlink_endpt local_endpt;
	enum nvlink_endpt remote_endpt;

	/* Function pointers used by the core driver for controlling endpoint
	   drivers */
	int (*enable_link)(struct nvlink_endpt_drv *drv);
};

/* APIs used by endpoint drivers for interfacing with the core driver */
int nvlink_register_endpt_drv(struct nvlink_endpt_drv *drv);

int nvlink_init_link(struct nvlink_endpt_drv *drv);

#endif /* NVLINK_H */
