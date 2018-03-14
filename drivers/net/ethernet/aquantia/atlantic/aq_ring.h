/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2017 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

/* File aq_ring.h: Declaration of functions for Rx/Tx rings. */

#ifndef AQ_RING_H
#define AQ_RING_H

#include "aq_common.h"

struct page;

#define AQ_HW_RXD_WB_CHECKSUM_VALID BIT(3)

#define AQ_HW_TXD_DESC_TYPE_TXD   (0x1)
#define AQ_HW_TXD_DESC_TYPE_TXC   (0x2)


/* Hardware tx descriptor */

struct __packed hw_atl_txd_s {
        u64 buf_addr;

	union {
		struct __packed {
			u16 desc_type:3;
			u16:1;
			u16 buf_len:16;
			u16 dd:1;
			u16 eop:1;
			u16 tx_cmd:8;
			u16:14;
			u16 ct_idx:1;
			u16 ct_en:1;
			u32 pay_len:18;
		};
		u64 flags;
	};
};

/* Hardware tx context descriptor */
union hw_atl_txc_s {
	struct __packed {
		u64:40;
		u16 tun_len:8;
		u16 out_len:16;
		u16 desc_type:3;
		u32 ct_idx:1;
		u16 vlan_tag:16;
		u16 ct_cmd:4;
		u16 l2_len:7;
		u16 l3_len:9;
		u16 l4_len:8;
		u16 mss_len:16;
	};
	struct __packed {
		u64 flags1;
		u64 flags2;
	};	
};

union hw_atl_rxd_s {
    /* Hardware rx descriptor */
    struct __packed {
        u64 buf_addr;
        u64 hdr_addr;
    } read;

	/* Hardware rx descriptor writeback */
	struct __packed {
		u16 rss_type:4;
		u16 pkt_type:8;
		u16 rdm_err:1;
		u16:6;
		u16 rx_cntl:2;
		u16 sph:1;
		u16 hdr_len:10;
		u32 rss_hash;
		u16 dd:1;
		u16 eop:1;
		u16 stat:4;
		u16 estat:6;
		u16 rsc_cnt:4;
		u16 pkt_len;
		u16 next_desc_ptr;
		u16 vlan;
	} wb;
};

struct aq_cb {
    dma_addr_t dma;
    u16 append_cnt; /* number of skb's appended */
    bool page_released;
};

#define AQ_CB(skb) ((struct aq_cb *)(skb)->cb)

struct aq_ring_rx_buff_s {
	struct sk_buff *skb;
	dma_addr_t dma;
	struct page *page;
	unsigned int page_offset;
};

struct aq_ring_tx_buff_s {
	struct hw_atl_txd_s *next_to_watch; //TX pending work
	struct sk_buff *skb;
	unsigned int byte_count;
	unsigned short gso_segs;
	__be16 protocol;
	DEFINE_DMA_UNMAP_ADDR(dma);
	DEFINE_DMA_UNMAP_LEN(len);
	int vlan_tag;
};

struct aq_ring_stats_rx_s {
	u64 errors;
	u64 allocation_fails;
	u64 packets;
	u64 bytes;
	u64 lro_packets;
	u64 rsc_count;
	u64 rsc_flush;
	u64 non_eop;
	u64 jumbo_packets;
};

struct aq_ring_stats_tx_s {
	u64 errors;
	u64 packets;
	u64 bytes;
	u64 restart_queue;
	u64 busy_count;
};

union aq_ring_stats_s {
	struct aq_ring_stats_rx_s rx;
	struct aq_ring_stats_tx_s tx;
};

struct aq_ring_s {
	struct aq_obj_s header;

	struct aq_ring_rx_buff_s *rx_buff_info;
	struct aq_ring_tx_buff_s *tx_buff_info;


	u8 *dx_ring;		/* descriptors ring, dma shared mem */
	struct aq_nic_s *aq_nic;
	struct aq_hw_s *aq_hw;
	unsigned int idx;	/* for HW layer registers operations */

	unsigned int dx_mem_size;
	unsigned int size;	/* descriptors number */
	unsigned int dx_size;	/* TX or RX descriptor size,  */
	union aq_ring_stats_s stats;

	dma_addr_t dma;

	u16 next_to_use;
	u16 next_to_clean;
	u16 next_to_alloc;
};

struct aq_ring_param_s {
	unsigned int vec_idx;
	unsigned int cpu;
	cpumask_t affinity_mask;
};


struct aq_ring_s *aq_ring_tx_alloc(struct aq_ring_s *self,
				   struct aq_nic_s *aq_nic,
				   struct aq_hw_s *aq_hw,
				   unsigned int idx,
				   struct aq_nic_cfg_s *aq_nic_cfg);
struct aq_ring_s *aq_ring_rx_alloc(struct aq_ring_s *self,
				   struct aq_nic_s *aq_nic,
				   struct aq_hw_s *aq_hw,
				   unsigned int idx,
				   struct aq_nic_cfg_s *aq_nic_cfg);
int aq_ring_init(struct aq_ring_s *self);

void aq_ring_free_tx(struct aq_ring_s *self);
void aq_ring_free_rx(struct aq_ring_s *self);
bool aq_ring_tx_clean_irq(struct aq_ring_s *self, int budget);
int aq_ring_rx_clean_irq(struct aq_ring_s *self, struct napi_struct *napi, int budget);

netdev_tx_t aq_ring_transmit_skb(struct aq_ring_s *self, struct sk_buff *skb);

#endif /* AQ_RING_H */
