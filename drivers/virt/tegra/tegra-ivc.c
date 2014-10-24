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

#include <linux/tegra-ivc.h>
#include "tegra-ivc-internal.h"
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <asm/compiler.h>

#define IVC_ALIGN 64

struct ivc_channel_header {
	union {
		struct {
			uint32_t w_count;
			uint32_t state;
		};
		uint8_t w_align[IVC_ALIGN];
	};
	union {
		uint32_t r_count;
		uint8_t r_align[IVC_ALIGN];
	};
};

static inline void ivc_invalidate_counter(struct ivc *ivc,
		dma_addr_t handle)
{
	if (!ivc->peer_device)
		return;
	dma_sync_single_for_cpu(ivc->peer_device, handle, IVC_ALIGN,
			DMA_FROM_DEVICE);
}

static inline void ivc_flush_counter(struct ivc *ivc, dma_addr_t handle)
{
	if (!ivc->peer_device)
		return;
	dma_sync_single_for_device(ivc->peer_device, handle, IVC_ALIGN,
			DMA_TO_DEVICE);
}

static inline int ivc_channel_empty(struct ivc_channel_header *ch)
{
	return ACCESS_ONCE(ch->w_count) == ACCESS_ONCE(ch->r_count);
}

static inline int ivc_channel_full(struct ivc *ivc,
		struct ivc_channel_header *ch)
{
	/*
	 * Invalid cases where the counters indicate that the queue is over
	 * capacity also appear full.
	 */
	return ACCESS_ONCE(ch->w_count) - ACCESS_ONCE(ch->r_count)
		>= ivc->nframes;
}

static inline void ivc_advance_tx(struct ivc *ivc)
{
	ACCESS_ONCE(ivc->tx_channel->w_count) =
		ACCESS_ONCE(ivc->tx_channel->w_count) + 1;

	if (ivc->w_pos == ivc->nframes - 1)
		ivc->w_pos = 0;
	else
		ivc->w_pos++;
}

static inline void ivc_advance_rx(struct ivc *ivc)
{
	ACCESS_ONCE(ivc->rx_channel->r_count) =
		ACCESS_ONCE(ivc->rx_channel->r_count) + 1;

	if (ivc->r_pos == ivc->nframes - 1)
		ivc->r_pos = 0;
	else
		ivc->r_pos++;
}

/*
 * These helpers avoid unnecessary invalidations when performing repeated
 * accesses to an IVC channel by checking the old queue pointers first.
 * Synchronization is only necessary when these pointers indicate empty/full.
 */
static inline int ivc_rx_empty(struct ivc *ivc)
{
	if (ivc_channel_empty(ivc->rx_channel)) {
		ivc_invalidate_counter(ivc, ivc->rx_handle +
				offsetof(struct ivc_channel_header, r_count));
		return ivc_channel_empty(ivc->rx_channel);
	}

	return 0;
}

static inline int ivc_tx_full(struct ivc *ivc)
{
	if (ivc_channel_full(ivc, ivc->tx_channel)) {
		ivc_invalidate_counter(ivc, ivc->tx_handle +
				offsetof(struct ivc_channel_header, w_count));
		return ivc_channel_full(ivc, ivc->tx_channel);
	}

	return 0;
}

int tegra_ivc_can_read(struct ivc *ivc)
{
	return !ivc_rx_empty(ivc);
}
EXPORT_SYMBOL(tegra_ivc_can_read);

int tegra_ivc_can_write(struct ivc *ivc)
{
	return !ivc_tx_full(ivc);
}
EXPORT_SYMBOL(tegra_ivc_can_write);

int tegra_ivc_tx_empty(struct ivc *ivc)
{
	ivc_invalidate_counter(ivc, ivc->tx_handle +
			offsetof(struct ivc_channel_header, w_count));
	return ivc_channel_empty(ivc->tx_channel);
}
EXPORT_SYMBOL(tegra_ivc_tx_empty);

static void *ivc_frame_pointer(struct ivc *ivc, struct ivc_channel_header *ch,
		uint32_t frame)
{
	BUG_ON(frame >= ivc->nframes);
	return (void *)((uintptr_t)(ch + 1) + ivc->frame_size * frame);
}

static inline dma_addr_t ivc_frame_handle(struct ivc *ivc,
		dma_addr_t channel_handle, uint32_t frame)
{
	BUG_ON(!ivc->peer_device);
	BUG_ON(frame >= ivc->nframes);
	return channel_handle + ivc->frame_size * frame;
}

static inline void ivc_invalidate_frame(struct ivc *ivc,
		dma_addr_t channel_handle, unsigned frame, int offset, int len)
{
	if (!ivc->peer_device)
		return;
	dma_sync_single_for_cpu(ivc->peer_device,
			ivc_frame_handle(ivc, channel_handle, frame) + offset,
			len, DMA_FROM_DEVICE);
}

static inline void ivc_flush_frame(struct ivc *ivc, dma_addr_t channel_handle,
		unsigned frame, int offset, int len)
{
	if (!ivc->peer_device)
		return;
	dma_sync_single_for_device(ivc->peer_device,
			ivc_frame_handle(ivc, channel_handle, frame) + offset,
			len, DMA_TO_DEVICE);
}

static int ivc_read_frame(struct ivc *ivc, void *buf, void __user *user_buf,
		size_t max_read)
{
	const void *src;

	BUG_ON(buf && user_buf);

	if (max_read > ivc->frame_size)
		return -E2BIG;

	if (ivc_rx_empty(ivc))
		return -ENOMEM;

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	ivc_rmb();

	ivc_invalidate_frame(ivc, ivc->rx_handle, ivc->r_pos, 0, max_read);
	src = ivc_frame_pointer(ivc, ivc->rx_channel, ivc->r_pos);

	/*
	 * When compiled with optimizations, different versions of this
	 * function should be inlined into tegra_ivc_read_frame() or
	 * tegra_ivc_read_frame_user(). This should ensure that the user
	 * version does not add overhead to the kernel version.
	 */
	if (buf) {
		memcpy(buf, src, max_read);
	} else if (user_buf) {
		if (copy_to_user(user_buf, src, max_read))
			return -EFAULT;
	} else
		BUG();

	ivc_advance_rx(ivc);
	ivc_flush_counter(ivc, ivc->rx_handle);
	ivc->notify(ivc);

	return (int)max_read;
}

int tegra_ivc_read(struct ivc *ivc, void *buf, size_t max_read)
{
	return ivc_read_frame(ivc, buf, NULL, max_read);
}
EXPORT_SYMBOL(tegra_ivc_read);

int tegra_ivc_read_user(struct ivc *ivc, void __user *buf, size_t max_read)
{
	return ivc_read_frame(ivc, NULL, buf, max_read);
}
EXPORT_SYMBOL(tegra_ivc_read_user);

/* peek in the next rx buffer at offset off, the count bytes */
int tegra_ivc_read_peek(struct ivc *ivc, void *buf, size_t off, size_t count)
{
	const void *src;

	if (off > ivc->frame_size || off + count > ivc->frame_size)
		return -E2BIG;

	if (ivc_rx_empty(ivc))
		return -ENOMEM;

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	ivc_rmb();

	ivc_invalidate_frame(ivc, ivc->rx_handle, ivc->r_pos, off, count);
	src = ivc_frame_pointer(ivc, ivc->rx_channel, ivc->r_pos);

	memcpy(buf, (void *)((uintptr_t)src + off), count);

	/* note, no interrupt is generated */

	return (int)count;
}
EXPORT_SYMBOL(tegra_ivc_read_peek);

/* directly peek at the next frame rx'ed */
void *tegra_ivc_read_get_next_frame(struct ivc *ivc)
{
	if (ivc_rx_empty(ivc))
		return ERR_PTR(-ENOMEM);

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	ivc_rmb();

	ivc_invalidate_frame(ivc, ivc->rx_handle, ivc->r_pos, 0,
			ivc->frame_size);
	return ivc_frame_pointer(ivc, ivc->rx_channel, ivc->r_pos);
}
EXPORT_SYMBOL(tegra_ivc_read_get_next_frame);

int tegra_ivc_read_advance(struct ivc *ivc)
{
	/*
	 * No read barriers or synchronization here: the caller is expected to
	 * have already observed the channel non-empty. This check is just to
	 * catch programming errors.
	 */
	if (ivc_channel_empty(ivc->rx_channel))
		return -ENOMEM;

	ivc_advance_rx(ivc);
	ivc_flush_counter(ivc, ivc->rx_handle);
	ivc->notify(ivc);

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_read_advance);

static int ivc_write_frame(struct ivc *ivc, const void *buf,
		const void __user *user_buf, size_t size)
{
	void *p;

	BUG_ON(buf && user_buf);

	if (size > ivc->frame_size)
		return -E2BIG;

	if (ivc_tx_full(ivc))
		return -ENOMEM;

	p = ivc_frame_pointer(ivc, ivc->tx_channel, ivc->w_pos);

	/*
	 * When compiled with optimizations, different versions of this
	 * function should be inlined into tegra_ivc_write_frame() or
	 * tegra_ivc_write_frame_user(). This should ensure that the user
	 * version does not add overhead to the kernel version.
	 */
	if (buf) {
		memcpy(p, buf, size);
	} else if (user_buf) {
		if (copy_from_user(p, user_buf, size))
			return -EFAULT;
	} else
		BUG();

	memset(p + size, 0, ivc->frame_size - size);
	ivc_flush_frame(ivc, ivc->tx_handle, ivc->w_pos, 0, size);

	/*
	 * Ensure that updated data is visible before the w_pos counter
	 * indicates that it is ready.
	 */
	ivc_wmb();

	ivc_advance_tx(ivc);
	ivc_flush_counter(ivc, ivc->tx_handle);
	ivc->notify(ivc);

	return (int)size;
}

int tegra_ivc_write(struct ivc *ivc, const void *buf, size_t size)
{
	return ivc_write_frame(ivc, buf, NULL, size);
}
EXPORT_SYMBOL(tegra_ivc_write);

int tegra_ivc_write_user(struct ivc *ivc, const void __user *user_buf,
		size_t size)
{
	return ivc_write_frame(ivc, NULL, user_buf, size);
}
EXPORT_SYMBOL(tegra_ivc_write_user);

/* poke in the next tx buffer at offset off, the count bytes */
int tegra_ivc_write_poke(struct ivc *ivc, const void *buf, size_t off,
		size_t count)
{
	void *dest;

	if (off > ivc->frame_size || off + count > ivc->frame_size)
		return -E2BIG;

	if (ivc_channel_full(ivc, ivc->tx_channel))
		return -ENOMEM;

	dest = ivc_frame_pointer(ivc, ivc->tx_channel, ivc->w_pos);
	memcpy(dest + off, buf, count);

	return (int)count;
}
EXPORT_SYMBOL(tegra_ivc_write_poke);

/* directly poke at the next frame to be tx'ed */
void *tegra_ivc_write_get_next_frame(struct ivc *ivc)
{
	if (ivc_tx_full(ivc))
		return ERR_PTR(-ENOMEM);

	return ivc_frame_pointer(ivc, ivc->tx_channel, ivc->w_pos);
}
EXPORT_SYMBOL(tegra_ivc_write_get_next_frame);

/* advance the tx buffer */
int tegra_ivc_write_advance(struct ivc *ivc)
{
	if (ivc_channel_full(ivc, ivc->tx_channel))
		return -ENOMEM;

	ivc_flush_frame(ivc, ivc->tx_handle, ivc->w_pos, 0, ivc->frame_size);

	/*
	 * Order any possible stores to the frame before update of w_pos.
	 */
	ivc_wmb();

	ivc_advance_tx(ivc);
	ivc_flush_counter(ivc, ivc->tx_handle);
	ivc->notify(ivc);

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_write_advance);

size_t tegra_ivc_align(size_t size)
{
	return (size + (IVC_ALIGN - 1)) & ~(IVC_ALIGN - 1);
}
EXPORT_SYMBOL(tegra_ivc_align);

unsigned tegra_ivc_total_queue_size(unsigned queue_size)
{
	if (queue_size & (IVC_ALIGN - 1)) {
		pr_err("%s: queue_size (%u) must be %u-byte aligned\n",
				__func__, queue_size, IVC_ALIGN);
		return 0;
	}
	return queue_size + sizeof(struct ivc_channel_header);
}
EXPORT_SYMBOL(tegra_ivc_total_queue_size);

int tegra_ivc_init_shared_memory(uintptr_t queue_base1, uintptr_t queue_base2,
		unsigned nframes, unsigned frame_size)
{
	BUG_ON(offsetof(struct ivc_channel_header, w_count) & (IVC_ALIGN - 1));
	BUG_ON(offsetof(struct ivc_channel_header, r_count) & (IVC_ALIGN - 1));
	BUG_ON(sizeof(struct ivc_channel_header) & (IVC_ALIGN - 1));

	/*
	 * The headers must at least be aligned enough for counters
	 * to be accessed atomically.
	 */
	if (queue_base1 & (IVC_ALIGN - 1)) {
		pr_err("ivc channel start not aligned: %lx\n", queue_base1);
		return -EINVAL;
	}
	if (queue_base2 & (IVC_ALIGN - 1)) {
		pr_err("ivc channel start not aligned: %lx\n", queue_base2);
		return -EINVAL;
	}

	if (frame_size & (IVC_ALIGN - 1)) {
		pr_err("frame size not adequately aligned: %u\n", frame_size);
		return -EINVAL;
	}

	if (queue_base1 < queue_base2) {
		if (queue_base1 + frame_size * nframes > queue_base2) {
			pr_err("queue regions overlap: %lx + %x, %x\n",
					queue_base1, frame_size,
					frame_size * nframes);
			return -EINVAL;
		}
	} else {
		if (queue_base2 + frame_size * nframes > queue_base1) {
			pr_err("queue regions overlap: %lx + %x, %x\n",
					queue_base2, frame_size,
					frame_size * nframes);
			return -EINVAL;
		}
	}

	memset((void *)queue_base1, 0, sizeof(struct ivc_channel_header) +
			frame_size * nframes);
	memset((void *)queue_base2, 0, sizeof(struct ivc_channel_header) +
			frame_size * nframes);

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_init_shared_memory);

int tegra_ivc_init(struct ivc *ivc, uintptr_t rx_base, uintptr_t tx_base,
		unsigned nframes, unsigned frame_size,
		struct device *peer_device, void (*notify)(struct ivc *))
{
	BUG_ON(!ivc);
	BUG_ON((uint64_t)nframes * (uint64_t)frame_size >= 0x100000000);
	BUG_ON(!notify);

	/*
	 * All sizes that can be returned by communication functions should
	 * fit in an int.
	 */
	if (frame_size > INT_MAX)
		return -E2BIG;

	ivc->rx_channel = (struct ivc_channel_header *)rx_base;
	ivc->tx_channel = (struct ivc_channel_header *)tx_base;

	if (peer_device) {
		ivc->rx_handle = dma_map_single(peer_device, ivc->rx_channel,
				nframes * frame_size, DMA_BIDIRECTIONAL);
		if (ivc->rx_handle == DMA_ERROR_CODE)
			return -ENOMEM;

		ivc->tx_handle = dma_map_single(peer_device, ivc->tx_channel,
				nframes * frame_size, DMA_BIDIRECTIONAL);
		if (ivc->tx_handle == DMA_ERROR_CODE)
			return -ENOMEM;
	}

	ivc->notify = notify;
	ivc->frame_size = frame_size;
	ivc->nframes = nframes;

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_init);
