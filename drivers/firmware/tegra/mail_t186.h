/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef MAIL_T186_H
#define MAIL_T186_H

#include <linux/kernel.h>

struct ivc;

struct mail_ops {
	int (*init_prepare)(void);
	int (*init_irq)(void);
	int (*iomem_init)(void);
	int (*handshake)(void);
	int (*channel_init)(void);
	struct ivc *(*ivc_obj)(int ch);
	void (*resume)(void);
	void (*ring_doorbell)(int ch);
};

#endif
