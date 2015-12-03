/*
 * drivers/video/tegra/host/vi/vi.h
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2012-2015, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __NVHOST_VI_H__
#define __NVHOST_VI_H__

#include <linux/platform/tegra/isomgr.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-core.h>

#include "core.h"

#include "camera_priv_defs.h"
#include "chip_support.h"

#define MAX_CHAN_NUM	8
#define MAX_FORMAT_NUM	64
#define MAX_SUBDEVICES 4

#define VI_CFG_INTERRUPT_MASK_0				0x8c
#define VI_CFG_INTERRUPT_STATUS_0			0x98

#define CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK_0		0x850
#define CSI_CSI_PIXEL_PARSER_A_STATUS_0			0x854
#define PPA_FIFO_OVRF					(1 << 5)

#define CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK_0		0x884
#define CSI_CSI_PIXEL_PARSER_B_STATUS_0			0x888
#define PPB_FIFO_OVRF					(1 << 5)

#define VI_CSI_0_ERROR_STATUS					0x184
#define VI_CSI_1_ERROR_STATUS					0x284
#define VI_CSI_0_WD_CTRL						0x18c
#define VI_CSI_1_WD_CTRL						0x28c

#ifdef TEGRA_21X_OR_HIGHER_CONFIG
#define VI_CSI_2_ERROR_STATUS					0x384
#define VI_CSI_3_ERROR_STATUS					0x484
#define VI_CSI_2_WD_CTRL						0x38c
#define VI_CSI_3_WD_CTRL						0x48c

#define NUM_VI_WATCHDOG							4
#else
#define NUM_VI_WATCHDOG							2
#endif

typedef void (*callback)(void *);

struct tegra_vi_stats {
	atomic_t overflow;
};

/**
 * struct tegra_channel_buffer - video channel buffer
 * @buf: vb2 buffer base object
 * @queue: buffer list entry in the channel queued buffers list
 * @chan: channel that uses the buffer
 * @addr: Tegra IOVA buffer address for VI output
 */
struct tegra_channel_buffer {
	struct vb2_buffer buf;
	struct list_head queue;
	struct tegra_channel *chan;

	dma_addr_t addr;
};

#define to_tegra_channel_buffer(vb) \
	container_of(vb, struct tegra_channel_buffer, buf)

/**
 * struct tegra_vi_graph_entity - Entity in the video graph
 * @list: list entry in a graph entities list
 * @node: the entity's DT node
 * @entity: media entity, from the corresponding V4L2 subdev
 * @asd: subdev asynchronous registration information
 * @subdev: V4L2 subdev
 */
struct tegra_vi_graph_entity {
	struct list_head list;
	struct device_node *node;
	struct media_entity *entity;

	struct v4l2_async_subdev asd;
	struct v4l2_subdev *subdev;
};

/**
 * struct tegra_channel - Tegra video channel
 * @list: list entry in a composite device dmas list
 * @video: V4L2 video device associated with the video channel
 * @video_lock:
 * @pad: media pad for the video device entity
 * @pipe: pipeline belonging to the channel
 *
 * @vi: composite device DT node port number for the channel
 *
 * @kthread_capture: kernel thread task structure of this video channel
 * @wait: wait queue structure for kernel thread
 *
 * @format: active V4L2 pixel format
 * @fmtinfo: format information corresponding to the active @format
 *
 * @queue: vb2 buffers queue
 * @alloc_ctx: allocation context for the vb2 @queue
 * @sequence: V4L2 buffers sequence number
 *
 * @capture: list of queued buffers for capture
 * @queued_lock: protects the buf_queued list
 *
 * @csi: CSI register bases
 * @align: channel buffer alignment, default is 64
 * @port: CSI port of this video channel
 * @io_id: Tegra IO rail ID of this video channel
 *
 * @fmts_bitmap: a bitmap for formats supported
 * @bypass: bypass flag for VI bypass mode
 */
struct tegra_channel {
	struct list_head list;
	struct video_device video;
	struct media_pad pad;
	struct media_pipeline pipe;

	struct vi *vi;
	struct v4l2_subdev *subdev[MAX_SUBDEVICES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_pix_format format;
	const struct tegra_video_format *fmtinfo;

	struct vb2_queue queue;
	void *alloc_ctx;

	void __iomem *csi;
	unsigned int align;
	unsigned int port;
	unsigned int io_id;
	unsigned int num_subdevs;

	DECLARE_BITMAP(fmts_bitmap, MAX_FORMAT_NUM);
	bool bypass;
};

#define to_tegra_channel(vdev) \
	container_of(vdev, struct tegra_channel, video)

enum tegra_vi_pg_mode {
	TEGRA_VI_PG_DISABLED = 0,
	TEGRA_VI_PG_DIRECT,
	TEGRA_VI_PG_PATCH,
};

/**
 * struct tegra_vi - NVIDIA Tegra Video Input device structure
 * @v4l2_dev: V4L2 device
 * @media_dev: media device
 * @dev: device struct
 * @tegra_camera: tegra camera structure
 * @nvhost_device_data: NvHost VI device information
 *
 * @notifier: V4L2 asynchronous subdevs notifier
 * @entities: entities in the graph as a list of tegra_vi_graph_entity
 * @num_subdevs: number of subdevs in the pipeline
 *
 * @channels: list of channels at the pipeline output and input
 *
 * @ctrl_handler: V4L2 control handler
 * @pattern: test pattern generator V4L2 control
 * @pg_mode: test pattern generator mode (disabled/direct/patch)
 * @tpg_fmts_bitmap: a bitmap for formats in test pattern generator mode
 *
 * @has_sensors: a flag to indicate whether is a real sensor connecting
 */
struct vi {
	struct tegra_camera *camera;
	struct platform_device *ndev;
	struct v4l2_device v4l2_dev;
	struct media_device media_dev;
	struct device *dev;
	struct nvhost_device_data *ndata;

	struct v4l2_async_notifier notifier;
	struct list_head entities;
	unsigned int num_subdevs;

	struct tegra_channel chans[MAX_CHAN_NUM];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pattern;
	enum tegra_vi_pg_mode pg_mode;
	DECLARE_BITMAP(tpg_fmts_bitmap, MAX_FORMAT_NUM);

	struct regulator *reg;
	struct dentry *debugdir;
	struct tegra_vi_stats vi_out;
	struct workqueue_struct *vi_workqueue;
	struct work_struct stats_work;
	struct work_struct mfi_cb_work;
#if defined(CONFIG_TEGRA_ISOMGR)
	tegra_isomgr_handle isomgr_handle;
#endif
	int vi_irq;
	uint vi_bw;
	uint max_bw;
	bool has_sensors;
	bool master_deinitialized;
};

extern const struct file_operations tegra_vi_ctrl_ops;
int nvhost_vi_prepare_poweroff(struct platform_device *);
int nvhost_vi_finalize_poweron(struct platform_device *);

void nvhost_vi_reset_all(struct platform_device *);

#ifdef CONFIG_VIDEO_TEGRA_VI
int tegra_vi_register_mfi_cb(callback cb, void *cb_arg);
int tegra_vi_unregister_mfi_cb(void);
#else
static inline int tegra_vi_register_mfi_cb(callback cb, void *cb_arg)
{
	return -ENOSYS;
}
static inline int tegra_vi_unregister_mfi_cb(void)
{
	return -ENOSYS;
}
#endif
#endif
