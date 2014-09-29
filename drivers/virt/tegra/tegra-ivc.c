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
#include "cbuf.h"
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/err.h>

static inline void ivc_invalidate_cbuf(struct ivc *ivc, dma_addr_t cbuf_handle)
{
	if (!ivc->peer_device)
		return;
	dma_sync_single_for_cpu(ivc->peer_device, cbuf_handle,
			sizeof(struct cbuf), DMA_FROM_DEVICE);
	/* Order synchronization before subsequent loads. */
	cbuf_rmb();
}

static inline void ivc_flush_cbuf(struct ivc *ivc, dma_addr_t cbuf_handle)
{
	if (!ivc->peer_device)
		return;
	/* Order cbuf updates before synchronization. */
	cbuf_wmb();
	dma_sync_single_for_device(ivc->peer_device, cbuf_handle,
			sizeof(struct cbuf), DMA_TO_DEVICE);
}

/*
 * These helpers avoid unnecessary invalidations when performing repeated
 * accesses to an IVC channel by checking the old queue pointers first.
 * Synchronization is only necessary when these pointers indicate empty/full.
 */
static inline int ivc_rx_empty(struct ivc *ivc)
{
	if (cbuf_is_empty(ivc->rx_cbuf)) {
		ivc_invalidate_cbuf(ivc, ivc->rx_handle);
		return cbuf_is_empty(ivc->rx_cbuf);
	}

	return 0;
}

static inline int ivc_tx_full(struct ivc *ivc)
{
	if (cbuf_is_full(ivc->tx_cbuf)) {
		ivc_invalidate_cbuf(ivc, ivc->tx_handle);
		return cbuf_is_full(ivc->tx_cbuf);
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
	ivc_invalidate_cbuf(ivc, ivc->tx_handle);
	return cbuf_is_empty(ivc->tx_cbuf);
}
EXPORT_SYMBOL(tegra_ivc_tx_empty);

static void *ivc_frame_pointer(struct ivc *ivc, struct cbuf *cb, unsigned frame)
{
	return &cb->buf[ivc->frame_size * frame];
}

static inline dma_addr_t ivc_frame_handle(struct ivc *ivc,
		dma_addr_t cbuf_handle, unsigned frame)
{
	BUG_ON(!ivc->peer_device);
	BUG_ON(frame >= ivc->nframes);
	return cbuf_handle + ivc->frame_size * frame;
}

static inline void ivc_invalidate_frame(struct ivc *ivc, dma_addr_t cbuf_handle,
		unsigned frame, int offset, int len)
{
	if (!ivc->peer_device)
		return;
	dma_sync_single_for_cpu(ivc->peer_device,
			ivc_frame_handle(ivc, cbuf_handle, frame) + offset,
			len, DMA_FROM_DEVICE);
}

static inline void ivc_flush_frame(struct ivc *ivc, dma_addr_t cbuf_handle,
		unsigned frame, int offset, int len)
{
	if (!ivc->peer_device)
		return;
	dma_sync_single_for_device(ivc->peer_device,
			ivc_frame_handle(ivc, cbuf_handle, frame) + offset,
			len, DMA_TO_DEVICE);
}

static int ivc_read_frame(struct ivc *ivc, void *buf, void __user *user_buf,
		int max_read)
{
	struct cbuf *cb = ivc->rx_cbuf;
	int ret, chunk, left;
	const void *src;
	unsigned frame;
	/* Use the frame_size value that was previously validated. */
	size_t struct_size = ivc->frame_size;

	BUG_ON(buf && user_buf);

	if (ivc_rx_empty(ivc))
		return -ENOMEM;

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	cbuf_rmb();

	if (max_read > struct_size) {
		chunk = struct_size;
		left = max_read - chunk;
	} else {
		chunk = max_read;
		left = 0;
	}

	frame = ACCESS_ONCE(cb->r_pos);
	if (frame >= ivc->nframes)
		return -EINVAL;

	ivc_invalidate_frame(ivc, ivc->rx_handle, frame, 0, max_read);
	src = ivc_frame_pointer(ivc, cb, frame);

	/*
	 * When compiled with optimizations, different versions of this
	 * function should be inlined into tegra_ivc_read_frame() or
	 * tegra_ivc_read_frame_user(). This should ensure that the user
	 * version does not add overhead to the kernel version.
	 */
	if (buf) {
		memcpy(buf, src, chunk);
		memset(buf + chunk, 0, left);
	} else if (user_buf) {
		if (copy_to_user(user_buf, src, chunk))
			return -EFAULT;
		if (left > 0 && clear_user(user_buf + chunk, left))
			return -EFAULT;
	} else
		BUG();

	cbuf_advance_r_pos(cb);
	ivc_flush_cbuf(ivc, ivc->rx_handle);
	ivc->notify(ivc);

	ret = chunk;

	return ret;
}

int tegra_ivc_read(struct ivc *ivc, void *buf, int max_read)
{
	return ivc_read_frame(ivc, buf, NULL, max_read);
}
EXPORT_SYMBOL(tegra_ivc_read);

int tegra_ivc_read_user(struct ivc *ivc, void __user *buf, int max_read)
{
	return ivc_read_frame(ivc, NULL, buf, max_read);
}
EXPORT_SYMBOL(tegra_ivc_read_user);

/* peek in the next rx buffer at offset off, the count bytes */
int tegra_ivc_read_peek(struct ivc *ivc, void *buf, int off, int count)
{
	struct cbuf *cb = ivc->rx_cbuf;
	int chunk, rem;
	const void *src;
	unsigned frame;
	/* Use the frame_size value that was previously validated. */
	size_t struct_size = ivc->frame_size;

	if (off > ivc->frame_size || off + count > ivc->frame_size)
		return -EINVAL;

	if (ivc_rx_empty(ivc))
		return -ENOMEM;

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	cbuf_rmb();

	/* get maximum available number of bytes */
	rem = struct_size - off;
	chunk = count;

	/* if request is for more than rem, return only rem */
	if (chunk > rem)
		chunk = rem;

	frame = ACCESS_ONCE(cb->r_pos);
	if (frame >= ivc->nframes)
		return -EINVAL;

	ivc_invalidate_frame(ivc, ivc->rx_handle, frame, off, count);
	src = ivc_frame_pointer(ivc, cb, frame);

	memcpy(buf, src + off, chunk);

	/* note, no interrupt is generated */

	return chunk;
}
EXPORT_SYMBOL(tegra_ivc_read_peek);

/* directly peek at the next frame rx'ed */
void *tegra_ivc_read_get_next_frame(struct ivc *ivc)
{
	struct cbuf *cb = ivc->rx_cbuf;
	unsigned frame;

	if (ivc_rx_empty(ivc))
		return ERR_PTR(-ENOMEM);

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	cbuf_rmb();

	frame = ACCESS_ONCE(cb->r_pos);
	if (frame >= ivc->nframes)
		return ERR_PTR(-EINVAL);

	ivc_invalidate_frame(ivc, ivc->rx_handle, frame, 0, ivc->frame_size);
	return ivc_frame_pointer(ivc, cb, frame);
}
EXPORT_SYMBOL(tegra_ivc_read_get_next_frame);

int tegra_ivc_read_advance(struct ivc *ivc)
{
	struct cbuf *cb = ivc->rx_cbuf;

	/*
	 * No read barriers or synchronization here: the caller is expected to
	 * have already observed the cbuf non-empty. This check is just to
	 * catch programming errors.
	 */
	if (cbuf_is_empty(cb))
		return -ENOMEM;

	cbuf_advance_r_pos(cb);
	ivc_flush_cbuf(ivc, ivc->rx_handle);
	ivc->notify(ivc);

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_read_advance);

static int ivc_write_frame(struct ivc *ivc, const void *buf,
		const void __user *user_buf, int size)
{
	struct cbuf *cb = ivc->tx_cbuf;
	void *p;
	int ret, left, chunk;
	unsigned frame;
	/* Use the frame_size value that was previously validated. */
	size_t struct_size = ivc->frame_size;

	BUG_ON(buf && user_buf);

	if (ivc_tx_full(ivc))
		return -ENOMEM;

	if (size > struct_size) {
		chunk = struct_size;
		left = size - struct_size;
	} else {
		chunk = size;
		left = struct_size - chunk;
	}

	frame = ACCESS_ONCE(cb->w_pos);
	if (frame >= ivc->nframes)
		return -EINVAL;
	p = ivc_frame_pointer(ivc, cb, frame);

	/*
	 * When compiled with optimizations, different versions of this
	 * function should be inlined into tegra_ivc_write_frame() or
	 * tegra_ivc_write_frame_user(). This should ensure that the user
	 * version does not add overhead to the kernel version.
	 */
	if (buf) {
		memcpy(p, buf, chunk);
	} else if (user_buf) {
		if (copy_from_user(p, user_buf, chunk))
			return -EFAULT;
	} else
		BUG();

	memset(p + chunk, 0, left);
	ivc_flush_frame(ivc, ivc->tx_handle, frame, 0, size);

	/*
	 * Ensure that updated data is visible before the w_pos counter
	 * indicates that it is ready.
	 */
	cbuf_wmb();

	cbuf_advance_w_pos(cb);
	ivc_flush_cbuf(ivc, ivc->tx_handle);
	ivc->notify(ivc);

	ret = chunk;

	return size;
}

int tegra_ivc_write(struct ivc *ivc, const void *buf, int size)
{
	return ivc_write_frame(ivc, buf, NULL, size);
}
EXPORT_SYMBOL(tegra_ivc_write);

int tegra_ivc_write_user(struct ivc *ivc, const void __user *user_buf, int size)
{
	return ivc_write_frame(ivc, NULL, user_buf, size);
}
EXPORT_SYMBOL(tegra_ivc_write_user);

/* poke in the next tx buffer at offset off, the count bytes */
int tegra_ivc_write_poke(struct ivc *ivc, const void *buf, int off, int count)
{
	struct cbuf *cb = ivc->tx_cbuf;
	int rem, chunk;
	void *dest;
	unsigned frame;

	if (cbuf_is_full(cb))
		return -ENOMEM;

	rem = ivc->frame_size + off;
	chunk = count;
	if (chunk > rem)
		chunk = rem;

	frame = ACCESS_ONCE(cb->w_pos);
	if (frame >= ivc->nframes)
		return -EINVAL;
	dest = ivc_frame_pointer(ivc, cb, frame);
	memcpy(dest + off, buf, chunk);

	return chunk;
}
EXPORT_SYMBOL(tegra_ivc_write_poke);

/* directly poke at the next frame to be tx'ed */
void *tegra_ivc_write_get_next_frame(struct ivc *ivc)
{
	struct cbuf *cb = ivc->tx_cbuf;
	unsigned frame;

	if (ivc_tx_full(ivc))
		return ERR_PTR(-ENOMEM);

	frame = ACCESS_ONCE(cb->w_pos);
	if (frame >= ivc->nframes)
		return ERR_PTR(-EINVAL);

	return ivc_frame_pointer(ivc, cb, frame);
}
EXPORT_SYMBOL(tegra_ivc_write_get_next_frame);

/* advance the tx buffer */
int tegra_ivc_write_advance(struct ivc *ivc)
{
	struct cbuf *cb = ivc->tx_cbuf;
	unsigned frame;

	if (cbuf_is_full(cb))
		return -ENOMEM;

	frame = ACCESS_ONCE(cb->w_pos);
	if (frame >= ivc->nframes)
		return -EINVAL;

	ivc_flush_frame(ivc, ivc->tx_handle, frame, 0, ivc->frame_size);

	/*
	 * Order any possible stores to the frame before update of w_pos.
	 */
	cbuf_wmb();

	cbuf_advance_w_pos(cb);
	ivc_flush_cbuf(ivc, ivc->tx_handle);
	ivc->notify(ivc);

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_write_advance);

unsigned tegra_ivc_total_queue_size(unsigned queue_size)
{
	return queue_size + sizeof(struct cbuf);
}
EXPORT_SYMBOL(tegra_ivc_total_queue_size);

int tegra_ivc_init_shared_memory(uintptr_t queue_base, unsigned nframes,
		unsigned frame_size, unsigned queue_size)
{
	/*
	 * The first cbuf is guaranteed to be adequately aligned given
	 * that it follows a struct placed at the start of a page.
	 */
	BUG_ON(queue_base & (sizeof(int) - 1));
	cbuf_init((void *)queue_base, frame_size, nframes);
	BUG_ON(queue_base + queue_size < queue_base);
	queue_base += queue_size;

	/*
	 * The cbuf must at least be aligned enough for its counters
	 * to be accessed atomically.
	 */
	if (queue_base & (sizeof(int) - 1)) {
		pr_err("rx_cbuf start not aligned: %lx\n", queue_base);
		return -EINVAL;
	}
	cbuf_init((void *)queue_base, frame_size, nframes);

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_init_shared_memory);

int tegra_ivc_init(struct ivc *ivc, uintptr_t queue_base, unsigned nframes,
		unsigned frame_size, unsigned queue_size, int rx_first,
		struct device *peer_device, void (*notify)(struct ivc *))
{
	BUG_ON(!ivc);
	BUG_ON((uint64_t)nframes * (uint64_t)frame_size >= 0x100000000);
	BUG_ON(nframes * frame_size > queue_size);
	BUG_ON(!notify);

	/*
	 * cbufs are already initialized
	 * we only have to assign rx/tx
	 * peer[0] it's rx, tx
	 * peer[1] it's tx, rx
	 * both queues are of the same size
	 */
	if (rx_first) {
		ivc->rx_cbuf = (struct cbuf *)queue_base;
		ivc->tx_cbuf = (struct cbuf *)(queue_base + queue_size);
	} else {
		ivc->tx_cbuf = (struct cbuf *)queue_base;
		ivc->rx_cbuf = (struct cbuf *)(queue_base + queue_size);
	}

	if (peer_device) {
		ivc->rx_handle = dma_map_single(peer_device, ivc->rx_cbuf,
				queue_size, DMA_BIDIRECTIONAL);
		if (ivc->rx_handle == DMA_ERROR_CODE)
			return -ENOMEM;

		ivc->tx_handle = dma_map_single(peer_device, ivc->tx_cbuf,
				queue_size, DMA_BIDIRECTIONAL);
		if (ivc->tx_handle == DMA_ERROR_CODE)
			return -ENOMEM;
	}

	ivc->notify = notify;
	ivc->frame_size = frame_size;
	ivc->nframes = nframes;

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_init);
