/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2017 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

/* File hw_atl_b0_internal.h: Definition of Atlantic B0 chip specific
 * constants.
 */

#ifndef HW_ATL_B0_INTERNAL_H
#define HW_ATL_B0_INTERNAL_H

#include "../aq_common.h"

#define HW_ATL_B0_MTU_JUMBO (16352U - ETH_FCS_LEN)
#define HW_ATL_B0_MTU        1514U

#define HW_ATL_B0_TX_RINGS 4U
#define HW_ATL_B0_RX_RINGS 4U

#define HW_ATL_B0_RINGS_MAX 32U
#define HW_ATL_B0_TXD_SIZE       (16U)
#define HW_ATL_B0_RXD_SIZE       (16U)

#define HW_ATL_B0_MAC      0U
#define HW_ATL_B0_MAC_MIN  1U
#define HW_ATL_B0_MAC_MAX  33U

/* UCAST/MCAST filters */
#define HW_ATL_B0_UCAST_FILTERS_MAX 38
#define HW_ATL_B0_MCAST_FILTERS_MAX 8

/* interrupts */
#define HW_ATL_B0_ERR_INT 8U
#define HW_ATL_B0_INT_MASK  (0xFFFFFFFFU)

#define HW_ATL_B0_MPI_CONTROL_ADR       0x0368U
#define HW_ATL_B0_MPI_STATE_ADR         0x036CU

#define HW_ATL_B0_MPI_SPEED_MSK         0xFFFFU
#define HW_ATL_B0_MPI_SPEED_SHIFT       16U

#define HW_ATL_B0_RATE_10G              BIT(0)
#define HW_ATL_B0_RATE_5G               BIT(1)
#define HW_ATL_B0_RATE_2G5              BIT(3)
#define HW_ATL_B0_RATE_1G               BIT(4)
#define HW_ATL_B0_RATE_100M             BIT(5)

#define HW_ATL_B0_TXBUF_MAX  160U
#define HW_ATL_B0_RXBUF_MAX  320U

#define HW_ATL_B0_RSS_REDIRECTION_MAX 64U
#define HW_ATL_B0_RSS_REDIRECTION_BITS 3U
#define HW_ATL_B0_RSS_HASHKEY_BITS 320U

#define HW_ATL_B0_TCRSS_4_8  1
#define HW_ATL_B0_TC_MAX 1U
#define HW_ATL_B0_RSS_MAX 8U

#define HW_ATL_B0_LRO_RXD_MAX 2U
#define HW_ATL_B0_RS_SLIP_ENABLED  0U

/* (256k -1(max pay_len) - 54(header)) */
#define HAL_ATL_B0_LSO_MAX_SEGMENT_SIZE 262089U

/* (256k -1(max pay_len) - 74(header)) */
#define HAL_ATL_B0_LSO_IPV6_MAX_SEGMENT_SIZE 262069U

#define HW_ATL_B0_CHIP_REVISION_B0      0xA0U
#define HW_ATL_B0_CHIP_REVISION_UNKNOWN 0xFFU

#define HW_ATL_B0_FW_SEMA_RAM           0x2U

#define L2_FILTER_ACTION_DISCARD (0x0)
#define L2_FILTER_ACTION_HOST    (0x1)

#define HW_ATL_B0_UCP_0X370_REG  (0x370)

#define HW_ATL_B0_FLUSH() AQ_HW_READ_REG(self, 0x10)

#define HW_ATL_B0_FW_VER_EXPECTED 0x01050006U



/* HW layer capabilities */
static struct aq_hw_caps_s hw_atl_b0_hw_caps_ = {
	.ports = 1U,
	.is_64_dma = true,
	.msix_irqs = 4U,
	.irq_mask = ~0U,
	.vecs = HW_ATL_B0_RSS_MAX,
	.tcs = HW_ATL_B0_TC_MAX,
	.rxd_alignment = 1U,
	.rxd_size = HW_ATL_B0_RXD_SIZE,
	.rxds = 8U * 1024U,
	.txd_alignment = 1U,
	.txd_size = HW_ATL_B0_TXD_SIZE,
	.txds = 8U * 1024U,
	.txhwb_alignment = 4096U,
	.tx_rings = HW_ATL_B0_TX_RINGS,
	.rx_rings = HW_ATL_B0_RX_RINGS,
	.hw_features = NETIF_F_HW_CSUM |
			NETIF_F_RXCSUM |
			NETIF_F_RXHASH |
			NETIF_F_SG |
			NETIF_F_TSO |
			NETIF_F_LRO,
	.hw_priv_flags = IFF_UNICAST_FLT,
	.media_type = AQ_HW_MEDIA_TYPE_TP,
	.link_speed_msk = (HW_ATL_B0_RATE_10G |
			HW_ATL_B0_RATE_5G |
			HW_ATL_B0_RATE_2G5 |
			HW_ATL_B0_RATE_1G |
			HW_ATL_B0_RATE_100M),
	.flow_control = true,
	.mtu = HW_ATL_B0_MTU_JUMBO,
	.mac_regs_count = 88,
	.fw_ver_expected = HW_ATL_B0_FW_VER_EXPECTED,
};

#endif /* HW_ATL_B0_INTERNAL_H */
