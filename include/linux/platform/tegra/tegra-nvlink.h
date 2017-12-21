/*
 * tegra-nvlink.h:
 * This header contains the structures and APIs needed by Tegra NVLINK core and
 * endpoint drivers for interacting with each other.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef TEGRA_NVLINK_H
#define TEGRA_NVLINK_H

#include <linux/device.h>
#include <linux/cdev.h>

#define NVLINK_MAX_DEVICES			2
#define NVLINK_MAX_LINKS			2

struct nvlink_link;
struct nvlink_device;

enum nvlink_log_categories {
	nvlink_log_err	= BIT(0),	/* Error prints - these will be printed
					   unconditionally */
	nvlink_log_dbg	= BIT(1),	/* Debug prints */
};

#ifdef CONFIG_DEBUG_FS
/* This is the root debugfs directory for the entire NVLINK driver stack */
extern struct dentry *nvlink_debugfs;
#endif /* CONFIG_DEBUG_FS */

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

/* Enum nvlink_endpt will be used to initialize device ID in device struct */
enum nvlink_endpt {
	NVLINK_ENDPT_T19X,
	NVLINK_ENDPT_GV100
};

/*
 * Link modes are SW defined. Some modes map to HW link state, while some
 * falicitate transiting to a power state or off state.
 */
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

/*
 * TX_mode and RX_mode contains SW defined sublink modes. Some modes map to HW
 * sublink states while some are intermediate states needed for link training
 * and other sequences.
 */
enum tx_mode {
	NVLINK_TX_HS,
	NVLINK_TX_ENABLE_PM,
	NVLINK_TX_DISABLE_PM,
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
	NVLINK_RX_ENABLE_PM,
	NVLINK_RX_DISABLE_PM,
	NVLINK_RX_SINGLE_LANE,
	NVLINK_RX_SAFE,
	NVLINK_RX_OFF,
	NVLINK_RX_RXCAL,
};

/* Enum to represent link speed. Nvlink 2.0 can support below 2 speeds */
enum nvlink_speed {
	NVLINK_SPEED_20,
	NVLINK_SPEED_25
};

enum nvlink_refclk {
	NVLINK_REFCLK_150,
	NVLINK_REFCLK_156
};

/*
 * During nvlink device initialization, we use enum init_state to keep track of
 * what state we have reached. Based on the init state and the link mode, only
 * necessary steps will be executed. This allows us to call enumerate function
 * multiple times.
 *
 * NVLINK_DEV_OFF : The device is off and no part of nvlink controller hardware
 *		is out of reset and clocked.
 *
 * NVLINK_DEV_EARLY_INIT_DONE: The clocks are up, all resets deasserted, the
 * 		minion has booted and device level interrupts are initialized.
 *
 * NVLINK_LINK_EARLY_INIT_DONE: The link level initialization is done like -
 *		initialization of PHY, link interrupts and TLC buffers.
 *
 * NVLINK_DEV_INTERFACE_INIT_DONE: The memory interface is initialized.
 *
 * NVLINK_REG_INIT_DONE: The prod settings are incorporated. At this point
 * 		the link is ready to transition to safe mode and eventually to
 * 		High-Speed mode.
 */
enum init_state {
	NVLINK_DEV_OFF,
	NVLINK_DEV_EARLY_INIT_DONE,
	NVLINK_LINK_EARLY_INIT_DONE,
	NVLINK_DEV_INTERFACE_INIT_DONE,
	NVLINK_DEV_REG_INIT_DONE,
	NVLINK_INIT_STATE_INVALID
};

/*
 * These callbacks should be registered with core-driver during link
 * registration. These link_ops allow core-driver to enquire/set link and
 * sublink modes. Some help during link initializatiion.
 *
 * TODO: Pass struct nvlink_link as argument to below link_ops instead of using
 * 	struct nvlink_device. All the link level readl/writel functions need to
 * 	use link struct instead of device struct for above change.
 */
struct link_operations {
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
	int (*link_early_init)(struct nvlink_device *ndev);
	int (*link_interface_init)(struct nvlink_device *ndev);
};

/* These dev_ops expose interface between the core driver and endpoint device */
struct device_operations {
	int (*dev_early_init)(struct nvlink_device *ndev);
	int (*dev_interface_init)(struct nvlink_device *ndev);
	int (*dev_reg_init)(struct nvlink_device *ndev);
	int (*dev_shutdown)(struct nvlink_device *ndev);
};

/*
 * The core-driver maintains the topology information. The endpoint can also
 * keep a record of same in remote_device_info struct. Note: We do not save
 * pointers to remote device and link.
 */
struct remote_device_info {
	/* Device id of  remote device connected */
	enum nvlink_endpt device_id;
	/* Link id of the remote link connected */
	u32 link_id;
};

/*
 * This structure is used for storing parameters which describe the Single-Lane
 * (SL / 1/8th) mode policy. A few acronyms that are used in this structure are
 * as follows:
 *    - SL = Single-Lane / 1/8th mode - sublink low power mode where only 1 of
 *           the 8 lanes is used
 *    - FB = Full Bandwidth (i.e. HISPEED mode)
 *    - LP = Low Power (i.e. SL / 1/8th mode)
 *    - IC = Idle Counter - the idle counter is used to monitor traffic per
 *           sub-link
 */
struct single_lane_params {
	/* Is Single-Lane (SL) mode enabled? */
	bool enabled;

	/* Idle counter increment in FB */
	u16 fb_ic_inc;

	/* Idle counter increment in LP */
	u16 lp_ic_inc;

	/* Idle counter decrement in FB */
	u16 fb_ic_dec;

	/* Idle counter decrement in LP */
	u16 lp_ic_dec;

	/* SL entry threshold */
	u32 enter_thresh;

	/* SL exit threshold */
	u32 exit_thresh;

	/* Idle counter saturation limit */
	u32 ic_limit;
};

/* nvlink_link struct stores all link specific data. */
struct nvlink_link {
	/*
	 * The link id is unique across the entire nvlink system. Same link_id
	 * should not be used in different device structs. This is a HACK we
	 * need while we hardcode the topology in device tree.
	 * TODO: Add an enum for link_id like we have for device_id.
	 */
	u32 link_id;
	/* ID of the device that this link belongs to */
	enum nvlink_endpt device_id;
	/* link mode TODO: Add locks to protect the link_mode changes */
	enum link_mode mode;
	/* base address of DLPL */
	void __iomem *nvlw_nvl_base;
	/* base address of TL */
	void __iomem *nvlw_nvltlc_base;
	/* bit index of enable bit within nvlink enable_register */
	u8 intr_bit_idx;
	/* bit index of reset bit within nvlink reset_register */
	u8 reset_bit_idx;
	/*
	 * is the link connected to an endpt. Useful for devices with multiple
	 * links. Currenly unused on Tegra.
	 * TODO: Set this before registering the link
	 */
	bool is_connected;
	/* Pointer to device info of connected end point */
	struct remote_device_info remote_dev_info;
	/*
	 * Pointer to struct containing callback functions to do link specific
	 * operation from core driver
	 */
	struct link_operations link_ops;
	/* Pointer to implementations specific private data */
	void *priv;
	/* TLC errors status. TODO: Add more description here */
	u32 tlc_tx_err_status0;
	u32 tlc_rx_err_status0;
	u32 tlc_rx_err_status1;
	/* Successful error recoveries */
	u32 error_recoveries;
	/* Parameters which describe the selected Single-Lane policy */
	struct single_lane_params sl_params;
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

/* nvlink_device struct stores all device specific data. */
struct nvlink_device {
	/* device_id */
	enum nvlink_endpt device_id;
	/* init state */
	enum init_state init_state;
	/* Mutex to protect init_state access */
	struct mutex init_state_mutex;
	/*
	 * Only the master device can initiate enumeration and data transfer
	 * on nvlink. bool to check this device is master.
	 */
	bool is_master;
	/* base address of NVLIPT */
	void __iomem *nvlw_nvlipt_base;
	/* base address of minion */
	void __iomem *nvlw_minion_base;
	/* base address of IOCTRL */
	void __iomem *nvlw_tioctrl_base;
	/* TODO: Add more information here */
	int irq;
	struct class class;
	dev_t dev_t;
	struct cdev cdev;
	struct device *dev;
	/*nvlink link data. We assume there is single link per device*/
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
	/* Nvlink Speed */
	enum nvlink_speed speed;
	/* Nvlink refclk*/
	enum nvlink_refclk refclk;
};

/* Struct used for passing around error masks in error handling functions */
struct nvlink_link_error_masks {
	u32 dl;
	u32 tl;
	u32 tl_injected;
	u32 tlc_rx0;
	u32 tlc_rx0_injected;
	u32 tlc_rx1;
	u32 tlc_rx1_injected;
	u32 tlc_tx;
	u32 tlc_tx_injected;
};

/* Fatal Errors */
enum inforom_nvlink_fatal_err {
	/* NVLink 2.0 */
	TLC_RX_DL_DATA_PARITY,
	TLC_RX_DL_CTRL_PARITY,
	TLC_RX_RAM_DATA_PARITY,
	TLC_RX_RAM_HDR_PARITY,
	TLC_RX_DATA_POISONED_PKT_RCVD,
	TLC_TX_RAM_DATA_PARITY,
	TLC_TX_RAM_HDR_PARITY,
	TLC_TX_DL_FLOW_CONTROL_PARITY,
	DL_TX_RECOVERY_LONG,
	DL_TX_FAULT_RAM,
	DL_TX_FAULT_INTERFACE,
	DL_TX_FAULT_SUBLINK_CHANGE,
	DL_RX_FAULT_SUBLINK_CHANGE,
	DL_RX_FAULT_DL_PROTOCOL,
	DL_LTSSM_FAULT,
	TLC_RX_DL_HDR_PARITY,
	TLC_RX_INVALID_AE_FLIT_RCVD,
	TLC_RX_INVALID_BE_FLIT_RCVD,
	TLC_RX_INVALID_ADDR_ALIGN,
	TLC_RX_PKT_LEN,
	TLC_RX_RSVD_CMD_ENC,
	TLC_RX_RSVD_DAT_LEN_ENC,
	TLC_RX_RSVD_ADDR_TYPE,
	TLC_RX_RSVD_RSP_STATUS,
	TLC_RX_RSVD_PKT_STATUS,
	TLC_RX_RSVD_CACHE_ATTR_ENC_IN_PROBE_REQ,
	TLC_RX_RSVD_CACHE_ATTR_ENC_IN_PROBE_RESP,
	TLC_RX_DAT_LEN_GT_ATOMIC_REQ_MAX_SIZE,
	TLC_RX_DAT_LEN_GT_RMW_REQ_MAX_SIZE,
	TLC_RX_DAT_LEN_LT_ATR_RESP_MIN_SIZE,
	TLC_RX_INVALID_PO_FOR_CACHE_ATTR,
	TLC_RX_INVALID_COMPRESSED_RESP,
	TLC_RX_RESP_STATUS_TARGET,
	TLC_RX_RESP_STATUS_UNSUPPORTED_REQUEST,
	TLC_RX_HDR_OVERFLOW,
	TLC_RX_DATA_OVERFLOW,
	TLC_RX_STOMPED_PKT_RCVD,
	TLC_RX_CORRECTABLE_INTERNAL,
	TLC_RX_UNSUPPORTED_VC_OVERFLOW,
	TLC_RX_UNSUPPORTED_NVLINK_CREDIT_RELEASE,
	TLC_RX_UNSUPPORTED_NCISOC_CREDIT_RELEASE,
	TLC_TX_HDR_CREDIT_OVERFLOW,
	TLC_TX_DATA_CREDIT_OVERFLOW,
	TLC_TX_DL_REPLAY_CREDIT_OVERFLOW,
	TLC_TX_UNSUPPORTED_VC_OVERFLOW,
	TLC_TX_STOMPED_PKT_SENT,
	TLC_TX_DATA_POISONED_PKT_SENT,
	TLC_TX_RESP_STATUS_TARGET,
	TLC_TX_RESP_STATUS_UNSUPPORTED_REQUEST,
};

/* APIs used by endpoint drivers for interfacing with the core driver */
void nvlink_print_topology(void);
int nvlink_register_device(struct nvlink_device* device);
int nvlink_register_link(struct nvlink_link* link);
int nvlink_unregister_device(struct nvlink_device* device);
int nvlink_unregister_link(struct nvlink_link* link);
int nvlink_get_init_state(struct nvlink_device *ndev, enum init_state *state);
int nvlink_set_init_state(struct nvlink_device *ndev, enum init_state state);
int nvlink_enumerate(struct nvlink_device *ndev);
int nvlink_transition_intranode_conn_off_to_safe(struct nvlink_device *ndev);
int nvlink_train_intranode_conn_safe_to_hs(struct nvlink_device *ndev);
int nvlink_transition_intranode_conn_hs_to_safe(struct nvlink_device *ndev);
#endif /* TEGRA_NVLINK_H */
