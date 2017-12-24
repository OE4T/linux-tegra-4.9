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

#include <linux/of.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/mc-regs-t19x.h>

#define NVLINK_MAX_DEVICES			2
#define NVLINK_MAX_LINKS			2
#define MINION_BYTES_PER_BLOCK		256
#define MINION_WORD_SIZE		4

struct nvlink_link;
struct nvlink_device;

enum nvlink_log_categories {
	nvlink_log_err	= BIT(0),	/* Error prints - these will be printed
					   unconditionally */
	nvlink_log_dbg	= BIT(1),	/* Debug prints */
};

extern u32 nvlink_log_mask;
#ifdef CONFIG_DEBUG_FS
/* This is the root debugfs directory for the entire NVLINK driver stack */
extern struct dentry *nvlink_debugfs;
#endif /* CONFIG_DEBUG_FS */

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
	NVLINK_ENDPT_T19X,
	NVLINK_ENDPT_GV100
};

enum link_mode {
	NVLINK_LINK_OFF,
	NVLINK_LINK_HS,
	NVLINK_LINK_SAFE,
	NVLINK_LINK_FAULT,
	NVLINK_LINK_RECOVERY,
	NVLINK_LINK_DETECT,
	NVLINK_LINK_RESET,
	NVLINK_LINK_ENABLE_PM,
	NVLINK_LINK_DISABLE_PM,
	NVLINK_LINK_DISABLE_ERR_DETECT,
	NVLINK_LINK_LANE_DISABLE,
	NVLINK_LINK_LANE_SHUTDOWN
};

enum tx_mode {
	NVLINK_TX_HS,
	NVLINK_TX_SINGLE_LANE,
	NVLINK_TX_SAFE,
	NVLINK_TX_OFF,
	NVLINK_TX_COMMON,
	NVLINK_TX_COMMON_DISABLE,
	NVLINK_TX_DATA_READY,
	NVLINK_TX_PRBS_EN,
};

enum rx_mode {
	NVLINK_RX_HS,
	NVLINK_RX_SINGLE_LANE,
	NVLINK_RX_SAFE,
	NVLINK_RX_OFF,
	NVLINK_RX_RXCAL,
};

enum nvlink_speed {
	NVLINK_SPEED_20,
	NVLINK_SPEED_25
};

enum device_state {
	NVLINK_DEVICE_RESET,
	NVLINK_DEVICE_INIT_IN_PROGRESS,
	NVLINK_DEVICE_LINK_READY_FOR_INIT
};

struct link_operations {
	int (*enable_link)(struct nvlink_device *ndev);
	u32 (*get_link_mode)(struct nvlink_device *ndev);
	int (*set_link_mode)(struct nvlink_device *ndev, u32 mode);
	u32 (*get_sublink_mode)(struct nvlink_device *ndev, bool is_rx_sublink);
	int (*set_sublink_mode)(struct nvlink_device *ndev, bool is_rx_sublink,
				u32 mode);
	u32 (*get_link_state)(struct nvlink_device *ndev);
	void (*get_tx_sublink_state)(struct nvlink_device *ndev,
				u32 *tx_sublink_state);
	void (*get_rx_sublink_state)(struct nvlink_device *ndev,
				u32 *rx_sublink_state);
};

struct device_operations {
	int (*dev_early_init)(struct nvlink_device *ndev);
	int (*dev_interface_init)(struct nvlink_device *ndev);
	int (*dev_shutdown)(struct nvlink_device *ndev);
};

struct remote_device_info {
	/* Device id of device connected - to be filled from device tree */
	enum nvlink_endpt device_id;
	/* Link id of the link connected - to be filled from device tree */
	u32 link_id;
};

struct nvlink_link {
	/* Instance# of link under same device */
	u32 link_id;
	/* ID of the device that this link belongs to */
	enum nvlink_endpt device_id;
	/* link State */
	enum link_mode mode;
	/* Nvlink Speed */
	enum nvlink_speed speed;
	/* base address of DLPL */
	void __iomem *nvlw_nvl_base;
	/* base address of TL */
	void __iomem *nvlw_nvltlc_base;
	/* bit index of enable bit within nvlink enable_register */
	u8 intr_bit_idx;
	/* bit index of reset bit within nvlink reset_register */
	u8 reset_bit_idx;
	/* is the link connected to an endpt - to be filled from device tree */
	bool is_connected;
	/* Pointer to device info of connected end point */
	struct remote_device_info remote_dev_info;
	/* Pointer to struct containing callback functions to do link specific
	 * operation from core driver
	 */
	struct link_operations link_ops;
	/* Pointer to implementations specific private data */
	void *priv;
};

/* Structure representing the MINION ucode header */
struct minion_hdr {
	u32 os_code_offset;
	u32 os_code_size;
	u32 os_data_offset;
	u32 os_data_size;
	u32 num_apps;
	u32 *app_code_offsets;
	u32 *app_code_sizes;
	u32 *app_data_offsets;
	u32 *app_data_sizes;
	u32 ovl_offset;
	u32 ovl_size;
	u32 ucode_img_size;
};

struct nvlink_device {
	/* device_id */
	enum nvlink_endpt device_id;
	/* device state */
	enum device_state state;
	/* if true, then ONLY the driver of this device can initiate enumeration
	* and data transfer on nvlink
	*/
	bool is_master;
	/* base address of NVLIPT */
	void __iomem *nvlw_nvlipt_base;
	/* base address of minion */
	void __iomem *nvlw_minion_base;
	/* base address of IOCTRL */
	void __iomem *nvlw_tioctrl_base;
	struct class class;
	dev_t dev_t;
	struct cdev cdev;
	struct device *dev;
	/*nvlink link data*/
	struct nvlink_link link;
	/* Pointer to struct containing callback functions to do device specific
	* operation from core driver
	*/
	struct device_operations dev_ops;
	/* pointer to private data of this device */
	/* MINION FW - contains both the ucode header and image */
	const struct firmware *minion_fw;
	/* MINION ucode header */
	struct minion_hdr minion_hdr;
	/* MINION ucode image */
	const u8 *minion_img;
	void *priv;
};

/* APIs used by endpoint drivers for interfacing with the core driver */
int nvlink_register_device(struct nvlink_device* device);
int nvlink_register_link(struct nvlink_link* link);
int nvlink_unregister_device(struct nvlink_device* device);
int nvlink_unregister_link(struct nvlink_link* link);
int nvlink_init_link(struct nvlink_device *ndev);

#endif /* NVLINK_H */
