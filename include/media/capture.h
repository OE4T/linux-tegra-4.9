/*
 * Tegra Video Input capture operations
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: David Wang <davidw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VI_CAPTURE_H__
#define __VI_CAPTURE_H__

#if defined(__KERNEL__)
#include <linux/compiler.h>
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include <linux/ioctl.h>
#include <linux/videodev2.h>

#define __VI_CAPTURE_ALIGN __aligned(8)

#define VIDIOC_CAPTURE_BASE BASE_VIDIOC_PRIVATE
#define VIDIOC_CAPTURE_SETUP	\
		_IOW('V', VIDIOC_CAPTURE_BASE, struct vi_capture_setup)
#define VIDIOC_CAPTURE_RELEASE	\
		_IOW('V', VIDIOC_CAPTURE_BASE + 1, __u32)
#define VIDIOC_CAPTURE_RESET	\
		_IOW('V', VIDIOC_CAPTURE_BASE + 2, __u32)
#define VIDIOC_CAPTURE_GET_INFO	\
		_IOR('V', VIDIOC_CAPTURE_BASE + 3, struct vi_capture_info)
#define VIDIOC_CAPTURE_SET_CONFIG	\
		_IOW('V', VIDIOC_CAPTURE_BASE + 4, struct vi_capture_control_msg)
#define VIDIOC_CAPTURE_REQUEST	\
		_IOW('V', VIDIOC_CAPTURE_BASE + 5, struct vi_capture_req)
#define VIDIOC_CAPTURE_STATUS	\
		_IOW('V', VIDIOC_CAPTURE_BASE + 6, __u32)

struct tegra_channel;

struct vi_capture_setup {
	uint32_t channel_flags;
	uint32_t __pad_flags;
	uint64_t vi_channel_mask;
	uint32_t queue_depth;
	uint32_t request_size;
	uint32_t mem;
	uint32_t __pad_mem;
} __VI_CAPTURE_ALIGN;

struct vi_capture_info {
	struct vi_capture_syncpts {
		uint32_t progress_syncpt;
		uint32_t progress_syncpt_val;
		uint32_t emb_data_syncpt;
		uint32_t emb_data_syncpt_val;
		uint32_t line_timer_syncpt;
		uint32_t line_timer_syncpt_val;
	} syncpts;
} __VI_CAPTURE_ALIGN;

struct vi_capture_control_msg {
	uint64_t ptr;
	uint32_t size;
	uint32_t __pad;
	uint64_t response;
} __VI_CAPTURE_ALIGN;

struct vi_capture_req {
	uint32_t buffer_index;
	uint32_t num_relocs;
	uint64_t reloc_relatives;
} __VI_CAPTURE_ALIGN;

int vi_capture_init(struct tegra_channel *chan);
void vi_capture_shutdown(struct tegra_channel *chan);
int vi_capture_setup(struct tegra_channel *chan,
		struct vi_capture_setup *setup);
int vi_capture_reset(struct tegra_channel *chan,
		uint32_t reset_flags);
int vi_capture_release(struct tegra_channel *chan,
		uint32_t reset_flags);
int vi_capture_get_info(struct tegra_channel *chan,
		struct vi_capture_info *info);
int vi_capture_control_message(struct tegra_channel *chan,
		struct vi_capture_control_msg *msg);
int vi_capture_request(struct tegra_channel *chan,
		struct vi_capture_req *req);
int vi_capture_status(struct tegra_channel *chan,
		int32_t timeout_ms);
long vi_capture_ioctl(struct file *file, void *fh,
		bool use_prio, unsigned int cmd, void *arg);
#endif

