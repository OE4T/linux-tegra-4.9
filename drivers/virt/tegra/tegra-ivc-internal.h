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
struct ivc_channel_header;

struct ivc {
	struct ivc_channel_header *rx_channel, *tx_channel;
	uint32_t w_pos, r_pos;

	void (*notify)(struct ivc *);
	uint32_t nframes, frame_size;

	struct device *peer_device;
	dma_addr_t rx_handle, tx_handle;
};


int tegra_ivc_init(struct ivc *ivc, uintptr_t rx_base, uintptr_t tx_base,
		unsigned nframes, unsigned frame_size,
		struct device *peer_device, void (*notify)(struct ivc *));
int tegra_ivc_init_shared_memory(uintptr_t rx_base, uintptr_t tx_base,
		unsigned nframes, unsigned frame_size);
unsigned tegra_ivc_total_queue_size(unsigned queue_size);
int tegra_ivc_write_user(struct ivc *ivc, const void __user *user_buf,
		size_t size);
int tegra_ivc_read_user(struct ivc *ivc, void __user *buf, size_t max_read);
size_t tegra_ivc_align(size_t size);

#ifdef CONFIG_SMP

static inline void ivc_rmb(void)
{
	smp_rmb();
}

static inline void ivc_wmb(void)
{
	smp_wmb();
}

static inline void ivc_mb(void)
{
	smp_mb();
}

#else

static inline void ivc_rmb(void)
{
	rmb();
}

static inline void ivc_wmb(void)
{
	wmb();
}

static inline void ivc_mb(void)
{
	mb();
}

#endif

#endif /* __tegra_ivc_internal_h__ */
