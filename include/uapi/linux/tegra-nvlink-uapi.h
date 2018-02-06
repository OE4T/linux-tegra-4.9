/*
 * tegra-nvlink-uapi.h:
 * This header contains the structures and variables needed for
 * the NVLINK userspace APIs exported by the Tegra NVLINK endpoint driver.
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef TEGRA_NVLINK_UAPI_H
#define TEGRA_NVLINK_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if defined(__KERNEL__)
#include <linux/bitops.h>
#else
#define __user
#ifndef BIT
#define BIT(b) (1UL << (b))
#endif
#endif

/* TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS */

#define TEGRA_NVLINK_VERSION_10				0x00000001
#define TEGRA_NVLINK_VERSION_20				0x00000002
#define TEGRA_NVLINK_VERSION_22				0x00000004

#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_INVALID	(0x00000000)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_1_0	(0x00000001)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_0	(0x00000002)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_2	(0x00000004)

#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_INVALID	(0x00000000)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_0		(0x00000002)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_2		(0x00000004)

#define TEGRA_CTRL_NVLINK_CAPS_SUPPORTED		BIT(0)
#define TEGRA_CTRL_NVLINK_CAPS_P2P_SUPPORTED		BIT(1)
#define TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ACCESS		BIT(2)
#define TEGRA_CTRL_NVLINK_CAPS_P2P_ATOMICS		BIT(3)
#define TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ATOMICS		BIT(4)
#define TEGRA_CTRL_NVLINK_CAPS_PEX_TUNNELING		BIT(5)
#define TEGRA_CTRL_NVLINK_CAPS_SLI_BRIDGE		BIT(6)
#define TEGRA_CTRL_NVLINK_CAPS_SLI_BRIDGE_SENSABLE	BIT(7)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0		BIT(8)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L1		BIT(9)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L2		BIT(10)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L3		BIT(11)
#define TEGRA_CTRL_NVLINK_CAPS_VALID			BIT(12)

struct tegra_nvlink_caps {
	__u16 nvlink_caps;

	__u8 lowest_nvlink_version;
	__u8 highest_nvlink_version;
	__u8 lowest_nci_version;
	__u8 highest_nci_version;

	__u32 discovered_link_mask;
	__u32 enabled_link_mask;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_STATUS */

/* NVLink link states */
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_INIT		(0x00000000)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_HWCFG		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_SWCFG		(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_ACTIVE		(0x00000003)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_FAULT		(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_RECOVERY		(0x00000006)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_INVALID		(0xFFFFFFFF)

/* NVLink Tx sublink states */
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_HIGH_SPEED_1	(0x00000000)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_SINGLE_LANE	(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_TRAINING	(0x00000005)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_SAFE_MODE	(0x00000006)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_OFF		(0x00000007)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_INVALID	(0x000000FF)

/* NVLink Rx sublink states */
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_HIGH_SPEED_1	(0x00000000)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_SINGLE_LANE	(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_TRAINING	(0x00000005)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_SAFE_MODE	(0x00000006)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_OFF		(0x00000007)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_INVALID	(0x000000FF)

#define TEGRA_CTRL_NVLINK_STATUS_PHY_NVHS			(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_PHY_GRS			(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_PHY_INVALID			(0x000000FF)

/* Version information */
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_0		(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_2		(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_INVALID	(0x000000FF)

#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_0		(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_2		(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_INVALID		(0x000000FF)

#define TEGRA_CTRL_NVLINK_STATUS_NVHS_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_NVHS_VERSION_INVALID		(0x000000FF)

#define TEGRA_CTRL_NVLINK_STATUS_GRS_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_GRS_VERSION_INVALID		(0x000000FF)

/* Connection properties */
#define TEGRA_CTRL_NVLINK_STATUS_CONNECTED_TRUE		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_CONNECTED_FALSE		(0x00000000)

#define TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_LOOPBACK	(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_LOOPOUT		(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_NONE		(0x00000000)

#define TEGRA_CTRL_NVLINK_STATUS_REMOTE_LINK_NUMBER_INVALID	(0x000000FF)

/* NVLink REFCLK types */
#define TEGRA_CTRL_NVLINK_REFCLK_TYPE_INVALID			(0x00)
#define TEGRA_CTRL_NVLINK_REFCLK_TYPE_NVHS			(0x01)
#define TEGRA_CTRL_NVLINK_REFCLK_TYPE_PEX			(0x02)

#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_ID_FLAGS_NONE	(0x00000000)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_ID_FLAGS_PCI	(0x00000001)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_ID_FLAGS_UUID	(0x00000002)

#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_EBRIDGE	(0x00000000)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_NPU		(0x00000001)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_GPU		(0x00000002)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_SWITCH	(0x00000003)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_TEGRA	(0x00000004)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_NONE		(0x000000FF)

#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_UUID_INVALID	(0xFFFFFFFF)

struct tegra_nvlink_device_info {
	/* ID Flags */
	__u32 device_id_flags;

	/* PCI Information */
	__u16 domain;
	__u16 bus;
	__u16 device;
	__u16 function;
	__u32 pci_device_id;

	/* Device Type */
	__u64 device_type;

	/* Device UUID */
	__u8 device_uuid[16];
};

struct tegra_nvlink_link_status_info {
	/* Top level capablilites */
	__u16 caps;

	__u8 phy_type;
	__u8 sublink_width;

	/* Link and sublink states */
	__u32 link_state;
	__u8 rx_sublink_status;
	__u8 tx_sublink_status;

	/* Indicates that lane reveral is in effect on this link */
	bool bLane_reversal;

	__u8 nvlink_version;
	__u8 nci_version;
	__u8 phy_version;

	/* Clock information */
	__u32 nvlink_link_clockKHz;
	__u32 nvlink_common_clock_speedKHz;
	__u32 nvlink_ref_clk_speedKHz;
	__u8 nvlink_ref_clk_type;

	__u32 nvlink_link_clockMhz;
	__u32 nvlink_common_clock_speedMhz;
	__u32 nvlink_ref_clk_speedMhz;

	/* Connection information */
	bool connected;
	__u8 loop_property;
	__u8 remote_device_link_number;
	__u8 local_device_link_number;

	struct tegra_nvlink_device_info remote_device_info;
	struct tegra_nvlink_device_info local_device_info;
};

struct tegra_nvlink_status {
	__u32 enabled_link_mask;
	struct tegra_nvlink_link_status_info link_info;
};

/* TEGRA_CTRL_CMD_NVLINK_CLEAR_COUNTERS */

/* These are the bitmask definitions for different counter types */
#define TEGRA_CTRL_NVLINK_COUNTER_INVALID			0x00000000

#define TEGRA_CTRL_NVLINK_COUNTER_TL_TX0			0x00000001
#define TEGRA_CTRL_NVLINK_COUNTER_TL_TX1			0x00000002
#define TEGRA_CTRL_NVLINK_COUNTER_TL_RX0			0x00000004
#define TEGRA_CTRL_NVLINK_COUNTER_TL_RX1			0x00000008

#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT		0x00010000

#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L(i)	(1 << (i + 17))
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_SIZE	8
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L0	0x00020000
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L1	0x00040000
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L2	0x00080000
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L3	0x00100000
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L4	0x00200000
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L5	0x00400000
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L6	0x00800000
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L7	0x01000000

#define TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY		0x02000000
#define TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY		0x04000000

#define TEGRA_CTRL_NVLINK_COUNTER_MAX_TYPES			32

/*
 * Return index of the bit that is set in 'n'. This assumes there is only
 * one such set bit in 'n'. Even if multiple bits are set,
 * result is in range of 0-31.
 */
#define TEGRA_BIT_IDX_32(n)						\
			((((n) & 0xFFFF0000) ? 0x10 : 0) |	\
			(((n) & 0xFF00FF00) ? 0x08 : 0) |	\
			(((n) & 0xF0F0F0F0) ? 0x04 : 0) |	\
			(((n) & 0xCCCCCCCC) ? 0x02 : 0) |	\
			(((n) & 0xAAAAAAAA) ? 0x01 : 0))

struct tegra_nvlink_clear_counters {
	__u32 link_mask;
	__u32 counter_mask;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_COUNTERS */
#define tegra_nvlink_counter(x)	\
	TEGRA_BIT_IDX_32(TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L(x))

struct tegra_nvlink_get_counters {
	__u8 link_id;
	__u32 counter_mask;
	bool bTx0_tl_counter_overflow;
	bool bTx1_tl_counter_overflow;
	bool bRx0_tl_counter_overflow;
	bool bRx1_tl_counter_overflow;
	__u64 nvlink_counters[TEGRA_CTRL_NVLINK_COUNTER_MAX_TYPES];
};

/* TEGRA_CTRL_CMD_NVLINK_GET_ERR_INFO */
struct tegra_nvlink_err_info {
	__u32 tl_err_log;
	__u32 tl_intr_en;
	__u32 tlc_tx_err_status0;
	__u32 tlc_rx_err_status0;
	__u32 tlc_rx_err_status1;
	__u32 tlc_tx_err_log_en0;
	__u32 tlc_rx_err_log_en0;
	__u32 tlc_rx_err_log_en1;
	__u32 mif_tx_err_status0;
	__u32 mif_rx_err_status0;
	__u32 dl_speed_status_tx;
	__u32 dl_speed_status_rx;
	bool bExcess_error_dl;
};

struct tegra_nvlink_get_err_info {
	__u32 link_mask;
	struct tegra_nvlink_err_info link_err_info;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_ERROR_RECOVERIES */
struct tegra_nvlink_get_error_recoveries {
	__u32 link_mask;
	__u32 num_recoveries;
};

/* TEGRA_CTRL_CMD_NVLINK_SETUP_EOM */
struct tegra_nvlink_setup_eom {
	__u8 link_id;
	__u32 params;
};

/* TEGRA_CTRL_NVLINK_TRAIN_INTRANODE_CONN */
enum tegra_ctrl_link_mode {
	TEGRA_CTRL_NVLINK_LINK_OFF,
	TEGRA_CTRL_NVLINK_LINK_HS,
	TEGRA_CTRL_NVLINK_LINK_SAFE,
	TEGRA_CTRL_NVLINK_LINK_FAULT,
	TEGRA_CTRL_NVLINK_LINK_RECOVERY,
	TEGRA_CTRL_NVLINK_LINK_DETECT,
	TEGRA_CTRL_NVLINK_LINK_RESET,
	TEGRA_CTRL_NVLINK_LINK_ENABLE_PM,
	TEGRA_CTRL_NVLINK_LINK_DISABLE_PM,
	TEGRA_CTRL_NVLINK_LINK_DISABLE_ERR_DETECT,
	TEGRA_CTRL_NVLINK_LINK_LANE_DISABLE,
	TEGRA_CTRL_NVLINK_LINK_LANE_SHUTDOWN
};

enum tegra_ctrl_tx_mode {
	TEGRA_CTRL_NVLINK_TX_HS,
	TEGRA_CTRL_NVLINK_TX_ENABLE_PM,
	TEGRA_CTRL_NVLINK_TX_DISABLE_PM,
	TEGRA_CTRL_NVLINK_TX_SINGLE_LANE,
	TEGRA_CTRL_NVLINK_TX_SAFE,
	TEGRA_CTRL_NVLINK_TX_OFF,
	TEGRA_CTRL_NVLINK_TX_COMMON,
	TEGRA_CTRL_NVLINK_TX_COMMON_DISABLE,
	TEGRA_CTRL_NVLINK_TX_DATA_READY,
	TEGRA_CTRL_NVLINK_TX_PRBS_EN,
};

enum tegra_ctrl_rx_mode {
	TEGRA_CTRL_NVLINK_RX_HS,
	TEGRA_CTRL_NVLINK_RX_ENABLE_PM,
	TEGRA_CTRL_NVLINK_RX_DISABLE_PM,
	TEGRA_CTRL_NVLINK_RX_SINGLE_LANE,
	TEGRA_CTRL_NVLINK_RX_SAFE,
	TEGRA_CTRL_NVLINK_RX_OFF,
	TEGRA_CTRL_NVLINK_RX_RXCAL,
};

struct tegra_nvlink_pci_dev_info {
	__u16 domain;
	__u8 bus;
	__u8 device;
	__u8 function;
};

struct tegra_nvlink_endpoint {
	__u16 node_id;
	__u32 link_index;
	struct tegra_nvlink_pci_dev_info pci_info;
};

/* link and sublink state of an nvlink endpoint */
struct tegra_nvlink_link_state {
	__u64 link_mode;
	__u64 tx_sublink_mode;
	__u64 rx_sublink_mode;
};

enum tegra_nvlink_conn_train_type {
	tegra_nvlink_train_conn_off_to_swcfg = 0,
	tegra_nvlink_train_conn_swcfg_to_active,
	tegra_nvlink_train_conn_to_off,
	tegra_nvlink_train_conn_active_to_swcfg,
	tegra_nvlink_train_conn_swcfg_to_off,
};

struct tegra_nvlink_train_intranode_conn {
	/* input fields */
	enum tegra_nvlink_conn_train_type train_to;
	struct tegra_nvlink_endpoint src_end_point;
	struct tegra_nvlink_endpoint dst_end_point;

	/* output fields */
	int status;
	struct tegra_nvlink_link_state src_end_state;
	struct tegra_nvlink_link_state dst_end_state;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_LP_COUNTERS */
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_NVHS		0
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_EIGHTH	1
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_OTHER	2
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_NUM_TX_LP_ENTER	3
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_NUM_TX_LP_EXIT	4
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_MAX_COUNTERS		5

struct tegra_nvlink_get_lp_counters {
	/* input field */
	__u32 link_id;
	/* input, output field */
	__u32 counter_valid_mask;
	/* output field */
	__u32 counter_values[TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_MAX_COUNTERS];
};

/* TEGRA_CTRL_CMD_NVLINK_CLEAR_LP_COUNTERS */
struct tegra_nvlink_clear_lp_counters {
	__u32 link_id;
};

/* Enum to represent IOCTLs inside the Tegra NVLINK driver */
enum tnvlink_ioctl_num {
	TNVLINK_IOCTL_GET_NVLINK_CAPS,
	TNVLINK_IOCTL_GET_NVLINK_STATUS,
	TNVLINK_IOCTL_CLEAR_COUNTERS,
	TNVLINK_IOCTL_GET_COUNTERS,
	TNVLINK_IOCTL_GET_ERR_INFO,
	TNVLINK_IOCTL_GET_ERROR_RECOVERIES,
	TNVLINK_IOCTL_SETUP_EOM,
	TNVLINK_IOCTL_TRAIN_INTRANODE_CONN,
	TNVLINK_IOCTL_GET_LP_COUNTERS,
	TNVLINK_IOCTL_CLEAR_LP_COUNTERS,
	TNVLINK_IOCTL_NUM_IOCTLS
};

/* TODO: choose a unique MAGIC number for ioctl implementation */
#define TEGRA_NVLINK_IOC_MAGIC	  'T'
#define	TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS				\
			_IOR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_NVLINK_CAPS,		\
				struct tegra_nvlink_caps)
#define TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_STATUS				\
			_IOR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_NVLINK_STATUS,	\
				struct tegra_nvlink_status)
#define TEGRA_CTRL_CMD_NVLINK_CLEAR_COUNTERS				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_CLEAR_COUNTERS,		\
				struct tegra_nvlink_clear_counters)
#define TEGRA_CTRL_CMD_NVLINK_GET_COUNTERS				\
			_IOWR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_COUNTERS,		\
				struct tegra_nvlink_get_counters)
#define TEGRA_CTRL_CMD_NVLINK_GET_ERR_INFO				\
			_IOR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_ERR_INFO,		\
				struct tegra_nvlink_get_err_info)
#define TEGRA_CTRL_CMD_NVLINK_GET_ERROR_RECOVERIES			\
		_IOWR(TEGRA_NVLINK_IOC_MAGIC,				\
			TNVLINK_IOCTL_GET_ERROR_RECOVERIES,		\
			struct tegra_nvlink_get_error_recoveries)
#define TEGRA_CTRL_CMD_NVLINK_SETUP_EOM					\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_SETUP_EOM,		\
				struct tegra_nvlink_setup_eom)
#define TEGRA_CTRL_NVLINK_TRAIN_INTRANODE_CONN				\
		_IOWR(TEGRA_NVLINK_IOC_MAGIC,				\
			TNVLINK_IOCTL_TRAIN_INTRANODE_CONN,		\
			struct tegra_nvlink_train_intranode_conn)
#define TEGRA_CTRL_CMD_NVLINK_GET_LP_COUNTERS				\
			_IOWR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_LP_COUNTERS,		\
				struct tegra_nvlink_get_lp_counters)
#define TEGRA_CTRL_CMD_NVLINK_CLEAR_LP_COUNTERS				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_CLEAR_LP_COUNTERS,	\
				struct tegra_nvlink_clear_lp_counters)

#endif /* TEGRA_NVLINK_UAPI_H */
