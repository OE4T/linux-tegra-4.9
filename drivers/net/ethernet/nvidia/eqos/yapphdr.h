/* =========================================================================
 * The Synopsys DWC ETHER QOS Software Driver and documentation (hereinafter
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto.  Permission is hereby granted,
 * free of charge, to any person obtaining a copy of this software annotated
 * with this license and the Software, to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================= */
/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef __EQOS_YAPPHDR_H__

#define __EQOS_YAPPHDR_H__

#define EQOS_MAX_TX_QUEUE_CNT 8
#define EQOS_MAX_RX_QUEUE_CNT 8

/* Private IOCTL for handling device specific task */
#define EQOS_PRV_IOCTL	SIOCDEVPRIVATE

#define EQOS_POWERUP_MAGIC_CMD	1
#define EQOS_POWERDOWN_MAGIC_CMD	2
#define EQOS_POWERUP_REMOTE_WAKEUP_CMD	3
#define EQOS_POWERDOWN_REMOTE_WAKEUP_CMD	4

/* for TX and RX threshold configures */
#define EQOS_RX_THRESHOLD_CMD	5
#define EQOS_TX_THRESHOLD_CMD	6

/* for TX and RX Store and Forward mode configures */
#define EQOS_RSF_CMD	7
#define EQOS_TSF_CMD	8

/* for TX DMA Operate on Second Frame mode configures */
#define EQOS_OSF_CMD	9

/* for TX and RX PBL configures */
#define EQOS_TX_PBL_CMD	10
#define EQOS_RX_PBL_CMD	11

/* INCR and INCRX mode */
#define EQOS_INCR_INCRX_CMD	12

/* for MAC Double VLAN Processing config */
#define EQOS_DVLAN_TX_PROCESSING_CMD		13
#define EQOS_DVLAN_RX_PROCESSING_CMD		14
#define EQOS_SVLAN_CMD				15


//Manju: Remove the below defines
/* RX/TX VLAN */
//#define EQOS_RX_OUTER_VLAN_STRIPPING_CMD	13
//#define EQOS_RX_INNER_VLAN_STRIPPING_CMD	14
//#define EQOS_TX_VLAN_DESC_CMD	15
//#define EQOS_TX_VLAN_REG_CMD	16

/* SA on TX */
#define EQOS_SA0_DESC_CMD	17
#define EQOS_SA1_DESC_CMD	18
#define EQOS_SA0_REG_CMD		19
#define EQOS_SA1_REG_CMD		20

/* CONTEX desc setup control */
#define EQOS_SETUP_CONTEXT_DESCRIPTOR 21

/* Packet generation */
#define EQOS_PG_TEST		22

/* TX/RX channel/queue count */
#define EQOS_GET_TX_QCNT		23
#define EQOS_GET_RX_QCNT		24

/* Line speed */
#define EQOS_GET_CONNECTED_SPEED		25

/* DCB/AVB algorithm */
#define EQOS_DCB_ALGORITHM		26
#define EQOS_AVB_ALGORITHM		27

/* RX split header */
#define EQOS_RX_SPLIT_HDR_CMD		28

/* L3/L4 filter */
#define EQOS_L3_L4_FILTER_CMD		29
/* IPv4/6 and TCP/UDP filtering */
#define EQOS_IPV4_FILTERING_CMD		30
#define EQOS_IPV6_FILTERING_CMD		31
#define EQOS_UDP_FILTERING_CMD		32
#define EQOS_TCP_FILTERING_CMD		33
/* VLAN filtering */
#define EQOS_VLAN_FILTERING_CMD		34
/* L2 DA filtering */
#define EQOS_L2_DA_FILTERING_CMD		35
/* ARP Offload */
#define EQOS_ARP_OFFLOAD_CMD		36
/* for AXI PBL configures */
#define EQOS_AXI_PBL_CMD			37
/* for AXI Write Outstanding Request Limit configures */
#define EQOS_AXI_WORL_CMD		38
/* for AXI Read Outstanding Request Limit configures */
#define EQOS_AXI_RORL_CMD		39
/* for MAC LOOPBACK configuration */
#define EQOS_MAC_LOOPBACK_MODE_CMD	40
/* PFC(Priority Based Flow Control) mode */
#define EQOS_PFC_CMD			41
/* for PTP OFFLOADING configuration */
#define EQOS_PTPOFFLOADING_CMD			42
#define EQOS_CSR_ISO_TEST	43
#define EQOS_MEM_ISO_TEST	44
#define EQOS_PHY_LOOPBACK 45

#define EQOS_RWK_FILTER_LENGTH	8

/* List of command errors driver can set */
#define	EQOS_NO_HW_SUPPORT	-1
#define	EQOS_CONFIG_FAIL	-3
#define	EQOS_CONFIG_SUCCESS	0

/* RX THRESHOLD operations */
#define EQOS_RX_THRESHOLD_32	0x1
#define EQOS_RX_THRESHOLD_64	0x0
#define EQOS_RX_THRESHOLD_96	0x2
#define EQOS_RX_THRESHOLD_128	0x3

/* TX THRESHOLD operations */
#define EQOS_TX_THRESHOLD_32	0x1
#define EQOS_TX_THRESHOLD_64	0x0
#define EQOS_TX_THRESHOLD_96	0x2
#define EQOS_TX_THRESHOLD_128	0x3
#define EQOS_TX_THRESHOLD_192	0x4
#define EQOS_TX_THRESHOLD_256	0x5
#define EQOS_TX_THRESHOLD_384	0x6
#define EQOS_TX_THRESHOLD_512	0x7

/* TX and RX Store and Forward Mode operations */
#define EQOS_RSF_DISABLE	0x0
#define EQOS_RSF_ENABLE	0x1

#define EQOS_TSF_DISABLE	0x0
#define EQOS_TSF_ENABLE	0x1

/* TX DMA Operate on Second Frame operations */
#define EQOS_OSF_DISABLE	0x0
#define EQOS_OSF_ENABLE	0x1

/* INCR and INCRX mode */
#define EQOS_INCR_ENABLE		0x1
#define EQOS_INCRX_ENABLE	0x0

/* TX and RX PBL operations */
#define EQOS_PBL_1	1
#define EQOS_PBL_2	2
#define EQOS_PBL_4	4
#define EQOS_PBL_8	8
#define EQOS_PBL_16	16
#define EQOS_PBL_32	32
#define EQOS_PBL_64	64	/* 8 x 8 */
#define EQOS_PBL_128	128	/* 8 x 16 */
#define EQOS_PBL_256	256	/* 8 x 32 */

/* AXI operations */
#define EQOS_AXI_PBL_4	0x2
#define EQOS_AXI_PBL_8	0x6
#define EQOS_AXI_PBL_16	0xE
#define EQOS_AXI_PBL_32	0x1E
#define EQOS_AXI_PBL_64	0x3E
#define EQOS_AXI_PBL_128	0x7E
#define EQOS_AXI_PBL_256	0xFE

#define EQOS_MAX_AXI_WORL 31
#define EQOS_MAX_AXI_RORL 31

/* RX VLAN operations */
/* Do not strip VLAN tag from received pkt */
#define EQOS_RX_NO_VLAN_STRIP	0x0
/* Strip VLAN tag if received pkt pass VLAN filter */
#define EQOS_RX_VLAN_STRIP_IF_FILTER_PASS  0x1
/* Strip VLAN tag if received pkt fial VLAN filter */
#define EQOS_RX_VLAN_STRIP_IF_FILTER_FAIL  0x2
/* Strip VALN tag always from received pkt */
#define EQOS_RX_VLAN_STRIP_ALWAYS	0x3

/* TX VLAN operations */
/* Do not add a VLAN tag dring pkt transmission */
#define EQOS_TX_VLAN_TAG_NONE	0x0
/* Remove the VLAN tag from the pkt before transmission */
#define EQOS_TX_VLAN_TAG_DELETE	0x1
/* Insert the VLAN tag into pkt to be transmitted */
#define EQOS_TX_VLAN_TAG_INSERT	0x2
/* Replace the VLAN tag into pkt to be transmitted */
#define EQOS_TX_VLAN_TAG_REPLACE	0x3

/* RX split header operations */
#define EQOS_RX_SPLIT_HDR_DISABLE 0x0
#define EQOS_RX_SPLIT_HDR_ENABLE 0x1

/* L3/L4 filter operations */
#define EQOS_L3_L4_FILTER_DISABLE 0x0
#define EQOS_L3_L4_FILTER_ENABLE 0x1

/* Loopback mode */
#define EQOS_MAC_LOOPBACK_DISABLE 0x0
#define EQOS_MAC_LOOPBACK_ENABLE 0x1

/* PFC(Priority Based Flow Control) mode */
#define EQOS_PFC_DISABLE 0x0
#define EQOS_PFC_ENABLE 0x1

#define EQOS_MAC0REG 0
#define EQOS_MAC1REG 1

#define EQOS_SA0_NONE		((EQOS_MAC0REG << 2) | 0) /* Do not include the SA */
#define EQOS_SA0_DESC_INSERT	((EQOS_MAC0REG << 2) | 1) /* Include/Insert the SA with value given in MAC Addr 0 Reg */
#define EQOS_SA0_DESC_REPLACE	((EQOS_MAC0REG << 2) | 2) /* Replace the SA with the value given in MAC Addr 0 Reg */
#define EQOS_SA0_REG_INSERT	((EQOS_MAC0REG << 2) | 2) /* Include/Insert the SA with value given in MAC Addr 0 Reg */
#define EQOS_SA0_REG_REPLACE	((EQOS_MAC0REG << 2) | 3) /* Replace the SA with the value given in MAC Addr 0 Reg */

#define EQOS_SA1_NONE		((EQOS_MAC1REG << 2) | 0) /* Do not include the SA */
#define EQOS_SA1_DESC_INSERT	((EQOS_MAC1REG << 2) | 1) /* Include/Insert the SA with value given in MAC Addr 1 Reg */
#define EQOS_SA1_DESC_REPLACE	((EQOS_MAC1REG << 2) | 2) /* Replace the SA with the value given in MAC Addr 1 Reg */
#define EQOS_SA1_REG_INSERT	((EQOS_MAC1REG << 2) | 2) /* Include/Insert the SA with value given in MAC Addr 1 Reg */
#define EQOS_SA1_REG_REPLACE	((EQOS_MAC1REG << 2) | 3) /* Replace the SA with the value given in MAC Addr 1 Reg */

#define EQOS_MAX_WFQ_WEIGHT	0X7FFF /* value for bandwidth calculation */

#define EQOS_MAX_INT_FRAME_SIZE (1024* 64)

typedef enum {
	EQOS_DMA_TX_FP = 0,
	EQOS_DMA_TX_WSP = 1,
	EQOS_DMA_TX_WRR = 2,
} e_eqos_dma_tx_arb_algo;

typedef enum {
	EQOS_DCB_WRR = 0,
	EQOS_DCB_WFQ = 1,
	EQOS_DCB_DWRR = 2,
	EQOS_DCB_SP = 3,
} e_eqos_dcb_algorithm;

typedef enum {
	EQOS_AVB_SP = 0,
	EQOS_AVB_CBS = 1,
} e_eqos_avb_algorithm;

typedef enum {
	EQOS_QDISABLED = 0x0,
	EQOS_QAVB,
	EQOS_QDCB,
	EQOS_QGENERIC
} eqos_queue_operating_mode;

/* common data structure between driver and application for
 * sharing info through ioctl
 * */
struct ifr_data_struct {
	unsigned int flags;
	unsigned int qinx; /* dma channel no to be configured */
	unsigned int cmd;
	unsigned int context_setup;
	unsigned int connected_speed;
	unsigned int rwk_filter_values[EQOS_RWK_FILTER_LENGTH];
	unsigned int rwk_filter_length;
	int command_error;
	int test_done;
	void *ptr;
};

struct eqos_dcb_algorithm {
	unsigned int qinx;
	unsigned int algorithm;
	unsigned int weight;
	eqos_queue_operating_mode op_mode;
};

struct eqos_avb_algorithm {
	unsigned int qinx;
	unsigned int algorithm;
	unsigned int cc;
	unsigned int idle_slope;
	unsigned int send_slope;
	unsigned int hi_credit;
	unsigned int low_credit;
	eqos_queue_operating_mode op_mode;
};

struct eqos_l3_l4_filter {
	/* 0, 1,2,3,4,5,6 or 7*/
	int filter_no;
	/* 0 - disable and 1 - enable */
	int filter_enb_dis;
	/* 0 - src addr/port and 1- dst addr/port match */
	int src_dst_addr_match;
	/* 0 - perfect and 1 - inverse match filtering */
	int perfect_inverse_match;
	/* To hold source/destination IPv4 addresses */
	unsigned char ip4_addr[4];
	/* holds single IPv6 addresses */
	unsigned short ip6_addr[8];

	/* TCP/UDP src/dst port number */
	unsigned short port_no;
};

struct eqos_vlan_filter {
	/* 0 - disable and 1 - enable */
	int filter_enb_dis;
	/* 0 - perfect and 1 - hash filtering */
	int perfect_hash;
	/* 0 - perfect and 1 - inverse matching */
	int perfect_inverse_match;
};

struct eqos_l2_da_filter {
	/* 0 - perfect and 1 - hash filtering */
	int perfect_hash;
	/* 0 - perfect and 1 - inverse matching */
	int perfect_inverse_match;
};

struct eqos_arp_offload {
	unsigned char ip_addr[4];
};

#define EQOS_VIA_REG	0
#define EQOS_VIA_DESC	1

/* for MAC Double VLAN Processing config */
#define EQOS_DVLAN_OUTER	(1)
#define EQOS_DVLAN_INNER	(1 << 1)
#define EQOS_DVLAN_BOTH	(EQOS_DVLAN_OUTER | EQOS_DVLAN_INNER)

#define EQOS_DVLAN_NONE	0
#define EQOS_DVLAN_DELETE	1
#define EQOS_DVLAN_INSERT	2
#define EQOS_DVLAN_REPLACE	3

#define EQOS_DVLAN_DISABLE	0
#define EQOS_DVLAN_ENABLE	1

struct eqos_config_dvlan {
	int inner_vlan_tag;
	int outer_vlan_tag;
	/* op_type will be
 	 * 0/1/2/3 for none/delet/insert/replace respectively
 	 * */
	int op_type;
	/* in_out will be
 	 * 1/2/3 for outer/inner/both respectively.
 	 * */
	int in_out;
	/* 0 for via registers and 1 for via descriptor */
	int via_reg_or_desc;
};


/* for PTP offloading configuration */
#define EQOS_PTP_OFFLOADING_DISABLE 		0
#define EQOS_PTP_OFFLOADING_ENABLE	 		1

#define EQOS_PTP_ORDINARY_SLAVE	 		1
#define EQOS_PTP_ORDINARY_MASTER	 		2
#define EQOS_PTP_TRASPARENT_SLAVE	 		3
#define EQOS_PTP_TRASPARENT_MASTER	 		4
#define EQOS_PTP_PEER_TO_PEER_TRANSPARENT	5

struct eqos_config_ptpoffloading {
	int en_dis;
	int mode;
	int domain_num;
    int mc_uc; 
};


#ifdef EQOS_CONFIG_PGTEST

/* uncomment below macro to enable application
 * to record all run reports to file */
//#define PGTEST_LOGFILE

/* TX DMA CHANNEL Weights */
#define EQOS_TX_CH_WEIGHT1	0x0
#define EQOS_TX_CH_WEIGHT2	0x1
#define EQOS_TX_CH_WEIGHT3	0x2
#define EQOS_TX_CH_WEIGHT4	0x3
#define EQOS_TX_CH_WEIGHT5	0x4
#define EQOS_TX_CH_WEIGHT6	0x5
#define EQOS_TX_CH_WEIGHT7	0x6
#define EQOS_TX_CH_WEIGHT8	0x7

/* PG test sub commands macro's */
#define EQOS_PG_SET_CONFIG	0x1
#define EQOS_PG_CONFIG_HW	  0x2
#define EQOS_PG_RUN_TEST		0x3
#define EQOS_PG_GET_RESULT	0x4
#define EQOS_PG_TEST_DONE	0x5


/* DMA channel bandwidth allocation parameters */
struct eqos_pg_user_ch_input {
	unsigned char ch_arb_weight;	/* Channel weights(1/2/3/4/5/6/7/8) for arbitration */
	unsigned int ch_fr_size;	/* Channel Frame size */
	unsigned char ch_bw_alloc;	/* The percentage bandwidth allocation for ch */

	unsigned char ch_use_slot_no_check;	/* Should Ch use slot number checking ? */
	unsigned char ch_use_adv_slot_no_check;
	unsigned char ch_slot_count_to_use;	/* How many slot used to report pg bits per slot value */

	unsigned char ch_use_credit_shape;	/* Should Ch use Credid shape algorithm for traffic shaping ? */
	unsigned char ch_credit_control;	/* Sould Ch use Credit Control algorithm for traffic shaping ? */

	unsigned char ch_tx_desc_slot_no_start;
	unsigned char ch_tx_desc_slot_no_skip;
	unsigned char ch_operating_mode;
	unsigned long ch_avg_bits;
	unsigned long ch_avg_bits_interrupt_count;
	unsigned char ch_avb_algorithm;
	unsigned char ch_debug_mode; /* enable/disable debug mode */
	unsigned int ch_max_tx_frame_cnt; /* maximum pkts to be sent on this channel, can be used for debug purpose */
};

struct eqos_pg_user_input {
	unsigned char duration_of_exp;
	/* enable bits for DMA. bit0=>ch0, bit1=>ch1, bit2=>ch2 */
	unsigned char dma_ch_en;

	unsigned char ch_tx_rx_arb_scheme;	/* Should Ch use Weighted RR policy with Rx:Tx/Tx:Rx or Fixed Priority */
	unsigned char ch_use_tx_high_prio;	/* Should Ch Tx have High priority over Rx */
	unsigned char ch_tx_rx_prio_ratio;	/* For RR what is the ratio between Tx:Rx/Rx:Tx */
	unsigned char dma_tx_arb_algo; /* Refer DMA Mode register TAA field */

	unsigned char queue_dcb_algorithm;

	unsigned char mac_lb_mode; /* 0 => No MAC Loopback; 1 => MAC Loopback On */
	unsigned int speed_100M_1G; /* 0 => No MAC Loopback; 1 => MAC Loopback On */

	struct eqos_pg_user_ch_input ch_input[EQOS_MAX_TX_QUEUE_CNT];
};

#define copy_pg_ch_input_members(to, from) do { \
	(to)->interrupt_prints = (from)->interrupt_prints; \
	(to)->tx_interrupts = (from)->tx_interrupts; \
	(to)->ch_arb_weight = (from)->ch_arb_weight; \
	(to)->ch_queue_weight = (from)->ch_queue_weight; \
	(to)->ch_bw = (from)->ch_bw; \
	(to)->ch_frame_size = (from)->ch_frame_size; \
	(to)->ch_enable_slot_check = (from)->ch_enable_slot_check; \
	(to)->ch_enable_adv_slot_check = (from)->ch_enable_adv_slot_check; \
	(to)->ch_avb_algorithm = (from)->ch_avb_algorithm; \
	(to)->ch_slot_count = (from)->ch_slot_count; \
	(to)->ch_avg_bits = (from)->ch_avg_bits; \
	(to)->ch_avg_bits_interrupt_count = (from)->ch_avg_bits_interrupt_count; \
	(to)->ch_credit_control = (from)->ch_credit_control; \
	(to)->ch_tx_desc_slot_no_start = (from)->ch_tx_desc_slot_no_start; \
	(to)->ch_tx_desc_slot_no_skip = (from)->ch_tx_desc_slot_no_skip; \
	(to)->ch_send_slope = (from)->ch_send_slope; \
	(to)->ch_idle_slope = (from)->ch_idle_slope; \
	(to)->ch_hi_credit = (from)->ch_hi_credit; \
	(to)->ch_lo_credit = (from)->ch_lo_credit; \
	(to)->ch_framecount_tx = (from)->ch_framecount_tx; \
	(to)->ch_framecount_rx = (from)->ch_framecount_rx; \
	(to)->ch_operating_mode = (from)->ch_operating_mode; \
	(to)->ch_debug_mode = (from)->ch_debug_mode;\
	(to)->ch_max_tx_frame_cnt = (from)->ch_max_tx_frame_cnt;\
} while (0)

struct eqos_pg_ch_input {
	unsigned int interrupt_prints;
	unsigned int tx_interrupts;
	unsigned char ch_arb_weight;
	unsigned int ch_queue_weight;
	unsigned char ch_bw;
	unsigned int ch_frame_size;
	unsigned char ch_enable_slot_check;	/* Enable checking of slot numbers programmed in the Tx Desc */
	unsigned char ch_enable_adv_slot_check;	/* When Set Data fetched for current slot and for next 2 slots in advance
						When reset data fetched for current slot and in advance for next slot*/

	unsigned char ch_avb_algorithm;
	unsigned char ch_slot_count;	/* Over which transmiteed bits per slot needs to be computed (Only for Credit based shaping) */
	unsigned long ch_avg_bits;
	unsigned long ch_avg_bits_interrupt_count;

	unsigned char ch_credit_control;	/* Will be zero (Not used) */

	unsigned char ch_tx_desc_slot_no_start;
	unsigned char ch_tx_desc_slot_no_skip;

	unsigned int ch_send_slope;
	unsigned int ch_idle_slope;
	unsigned int ch_hi_credit;
	unsigned int ch_lo_credit;

	unsigned long ch_framecount_tx;	/* No of Frames Transmitted on Channel 1 */
	unsigned long ch_framecount_rx;	/* No of Frames Received on Channel 1 */
	unsigned char ch_operating_mode;

	unsigned char ch_debug_mode; /* enable/disable debug mode */
	unsigned int ch_max_tx_frame_cnt; /* maximum pkts to be sent on this channel, can be used for debug purpose */
	unsigned int ch_desc_prepare; /* max packets which will be reprepared in Tx-interrupt
 																	 do not copy contents to app-copy, only driver should use this variable*/
};

#define copy_pg_struct_members(to, from)	do { \
	(to)->ch_sel_mask = (from)->ch_sel_mask; \
	(to)->duration_of_exp = (from)->duration_of_exp; \
	(to)->prio_tag_for_av = (from)->prio_tag_for_av; \
	(to)->queue_dcb_algorithm = (from)->queue_dcb_algorithm; \
	(to)->ch_tx_rx_arb_scheme = (from)->ch_tx_rx_arb_scheme; \
	(to)->ch_use_tx_high_prio = (from)->ch_use_tx_high_prio; \
	(to)->ch_tx_rx_prio_ratio = (from)->ch_tx_rx_prio_ratio; \
	(to)->dma_tx_arb_algo = (from)->dma_tx_arb_algo; \
	(to)->mac_lb_mode = (from)->mac_lb_mode; \
} while (0)

struct eqos_pg_struct {
	/* This gives which DMA channel is enabled and which is disabled
	 * Bit0 for Ch0
	 * Bit1 for Ch1
	 * Bit2 for Ch2 and so on
	 * Bit7 for Ch7
	 * */
	unsigned char ch_sel_mask;

	/* Duration for which experiment should be conducted in minutes - Default 2 Minutes */
	unsigned char duration_of_exp;

	/* Used when more than One channel enabled in Rx path (Not Used)
	 * for only CH1 Enabled:
	 * Frames sith Priority > Value programmed, frames sent to CH1
	 * Frames with priority < Value programmed are sent to CH0
	 *
	 * For both CH1 and CH2 Enabled:
	 * Frames sith Priority > Value programmed, frames sent to CH2
	 * Frames with priority < Value programmed are sent to CH
	 * */
	unsigned char prio_tag_for_av;

	unsigned char queue_dcb_algorithm;

	unsigned char ch_tx_rx_arb_scheme;	/* Should Ch use Weighted RR policy with Rx:Tx/Tx:Rx or Fixed Priority */
	unsigned char ch_use_tx_high_prio;	/* Should Ch Tx have High priority over Rx */
	unsigned char ch_tx_rx_prio_ratio;	/* For RR what is the ratio between Tx:Rx/Rx:Tx */
	unsigned char dma_tx_arb_algo; /* Refer DMA Mode register TAA field */

	unsigned char mac_lb_mode; /* 0 => No MAC Loopback; 1 => MAC Loopback On */
    unsigned int speed_100M_1G;
	struct eqos_pg_ch_input pg_ch_input[EQOS_MAX_TX_QUEUE_CNT];
	unsigned char channel_running[EQOS_MAX_TX_QUEUE_CNT];
};
#endif /* end of EQOS_CONFIG_PGTEST */

#endif
