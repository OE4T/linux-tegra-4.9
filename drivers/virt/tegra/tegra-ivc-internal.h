/*
 * Inter-VM Commuinication
 *
 * Copyright (C) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef __tegra_ivc_internal_h__
#define  __tegra_ivc_internal_h__

#include <linux/tegra-ivc.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>

struct ivc;
struct cbuf;

struct ivc {
	struct cbuf *rx_cbuf, *tx_cbuf;

	void (*notify)(struct ivc *);
	unsigned nframes, frame_size;

	struct device *peer_device;
	dma_addr_t rx_handle, tx_handle;
};


int tegra_ivc_init(struct ivc *ivc, uintptr_t queue_base, unsigned nframes,
		unsigned frame_size, unsigned total_queue_size, int rx_first,
		struct device *peer_device, void (*notify)(struct ivc *));
int tegra_ivc_init_shared_memory(uintptr_t queue_base, unsigned nframes,
		unsigned frame_size, unsigned total_queue_size);
unsigned tegra_ivc_total_queue_size(unsigned queue_size);
int tegra_ivc_write_user(struct ivc *ivc, const void __user *user_buf,
		int size);
int tegra_ivc_read_user(struct ivc *ivc, void __user *buf, int max_read);

#endif /* __tegra_ivc_internal_h__ */
