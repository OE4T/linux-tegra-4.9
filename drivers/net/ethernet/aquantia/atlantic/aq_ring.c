/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2017 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */


#include "aq_ring.h"
#include "aq_nic.h"
#include "aq_hw.h"
#include "./hw_atl/hw_atl_llh.h"

#include <linux/if.h>
#include <linux/if_vlan.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/timer.h>
#include <linux/cpu.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <net/ipv6.h>
#include <net/busy_poll.h>
#include <linux/vmalloc.h>

#define AQ_RING_2K 2048
#define AQ_RING_HEADER_SIZE 256
#define AQ_RING_TX_BUDGET 256
#define AQ_MAX_TXD_PWR	   14
#define AQ_MAX_DATA_PER_TXD  (1u << AQ_MAX_TXD_PWR)

/* Tx Descriptors needed, worst case */
#define TXD_USE_COUNT(S) DIV_ROUND_UP((S), AQ_MAX_DATA_PER_TXD)

#define DESC_NEEDED (MAX_SKB_FRAGS + 4)
#define TX_WAKE_THRESHOLD (DESC_NEEDED * 2)

static u16 aq_ring_desc_unused(struct aq_ring_s *self)
{
	u16 ntc = self->next_to_clean;
	u16 ntu = self->next_to_use;

	return ((ntc > ntu) ? 0 : self->size) + ntc - ntu -1;
}

static inline struct netdev_queue *txring_txq(struct aq_ring_s *self)
{
	struct net_device *ndev = aq_nic_get_ndev(self->aq_nic);

	return netdev_get_tx_queue(ndev, self->idx);
}

static bool aq_ring_alloc_mapped_page(struct aq_ring_s *self,
									  struct aq_ring_rx_buff_s *bi)
{
	struct page *page = bi->page;
	dma_addr_t dma;
	struct device *dev = aq_nic_get_dev(self->aq_nic);

	if (likely(page))
		return true;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	page = dev_alloc_pages(0);
#else
	page = __skb_alloc_pages(GFP_ATOMIC | __GFP_COLD | __GFP_COMP, bi->skb, 0);
#endif
	if (unlikely(!page)) {
		self->stats.rx.allocation_fails++;
		return false;
	}

	dma = dma_map_page(dev, page, 0, PAGE_SIZE, DMA_FROM_DEVICE);

	if (dma_mapping_error(dev, dma)) {
		__free_pages(page, 0);

		self->stats.rx.allocation_fails++;
		return false;
	}

	bi->dma = dma;
	bi->page = page;
	bi->page_offset = 0;

	return true;
}

static void aq_ring_alloc_rx_buffers(struct aq_ring_s *self, u16 cleaned_count)
{
	union hw_atl_rxd_s *rx_desc;
	struct aq_ring_rx_buff_s *bi;
	u16 i = self->next_to_use;

	if (!self->rx_buff_info) {
		return;
	}

	if (!cleaned_count) {
		return;
	}

	rx_desc = (union hw_atl_rxd_s *)&self->dx_ring[i * sizeof(union hw_atl_rxd_s)];
	bi = &self->rx_buff_info[i];
	i -= self->size;

	do {
		if (!aq_ring_alloc_mapped_page(self, bi))
			break;

		rx_desc->read.buf_addr = bi->dma + bi->page_offset;
		rx_desc->read.hdr_addr = 0U;

		rx_desc++;
		bi++;
		i++;

		if (unlikely(!i)) {
			rx_desc = (union hw_atl_rxd_s *)self->dx_ring;
			bi = self->rx_buff_info;
			i -= self->size;
		}

		cleaned_count--;
	} while (cleaned_count);

	i += self->size;

	if (self->next_to_use != i) {
		self->next_to_use = i;

	   	self->next_to_alloc = i;

		wmb();
		reg_rx_dma_desc_tail_ptr_set(self->aq_hw, i, self->idx);
	}
}

struct aq_ring_s *aq_ring_tx_alloc(struct aq_ring_s *self,
				   struct aq_nic_s *aq_nic,
				   struct aq_hw_s *aq_hw,
				   unsigned int idx,
				   struct aq_nic_cfg_s *aq_nic_cfg)
{
	struct device *dev = aq_nic_get_dev(aq_nic);

	self->aq_nic = aq_nic;
	self->aq_hw = aq_hw;
	self->idx = idx;
	self->size = aq_nic_cfg->txds;
	self->dx_size = aq_nic_cfg->aq_hw_caps->txd_size;

	self->tx_buff_info = vzalloc(sizeof(struct aq_ring_tx_buff_s) * self->size);

	if (!self->tx_buff_info)
		goto err;

	/* round up to nearest 4K */
	self->dx_mem_size = self->size * sizeof(struct hw_atl_txd_s);
	self->dx_mem_size = ALIGN(self->dx_mem_size, 4096);

	self->dx_ring = dma_alloc_coherent(dev, self->dx_mem_size, &self->dma, GFP_KERNEL);

	if (!self->dx_ring)
		goto err;

	self->next_to_use = 0;
	self->next_to_clean = 0;
	return self;

err:
	vfree(self->tx_buff_info);
	self->tx_buff_info = NULL;
	return NULL;
}

struct aq_ring_s *aq_ring_rx_alloc(struct aq_ring_s *self,
				   struct aq_nic_s *aq_nic,
				   struct aq_hw_s *aq_hw,
				   unsigned int idx,
				   struct aq_nic_cfg_s *aq_nic_cfg)
{
	struct device *dev = aq_nic_get_dev(aq_nic);

	self->aq_nic = aq_nic;
	self->aq_hw = aq_hw;
	self->idx = idx;
	self->size = aq_nic_cfg->rxds;
	self->dx_size = aq_nic_cfg->aq_hw_caps->rxd_size;

	self->rx_buff_info = vzalloc(sizeof(struct aq_ring_rx_buff_s) * self->size);

	if (!self->rx_buff_info)
		goto err;

	/* Round up to nearest 4K */
	self->dx_mem_size = self->size * sizeof(union hw_atl_rxd_s);
	self->dx_mem_size = ALIGN(self->dx_mem_size, 4096);

	self->dx_ring = dma_alloc_coherent(dev, self->dx_mem_size, &self->dma, GFP_KERNEL);

	if (!self->dx_ring)
		goto err;

	self->next_to_clean = 0;
	self->next_to_use = 0;

	aq_ring_alloc_rx_buffers(self, aq_ring_desc_unused(self));

	return self;

err:
	vfree(self->rx_buff_info);
	self->rx_buff_info = NULL;
	return NULL;
}

int aq_ring_init(struct aq_ring_s *self)
{
	self->next_to_use = 0U;
	self->next_to_clean = 0U;

	aq_ring_alloc_rx_buffers(self, aq_ring_desc_unused(self));
	return 0;
}

bool aq_ring_is_non_eop(struct aq_ring_s *self, union hw_atl_rxd_s *rx_desc, struct sk_buff *skb)
{
	u32 ntc = self->next_to_clean + 1;
	struct aq_ring_rx_buff_s *bi = NULL;

	ntc = (ntc < self->size) ? ntc : 0;
	self->next_to_clean = ntc;

	prefetch((void*)&self->dx_ring[ntc * sizeof(union hw_atl_rxd_s)]);

	if (unlikely(rx_desc->wb.next_desc_ptr)) {
		AQ_CB(skb)->append_cnt += rx_desc->wb.rsc_cnt - 1;

		ntc = rx_desc->wb.next_desc_ptr;
	}

	if (likely(rx_desc->wb.eop)) {
		return false;
	}

	bi = &self->rx_buff_info[ntc];
	bi->skb = skb;
	self->stats.rx.non_eop++;

	return true;
}

bool aq_ring_tx_clean_irq(struct aq_ring_s *self, int napi_budget)
{
	struct aq_ring_tx_buff_s *tx_buffer;
	struct hw_atl_txd_s *tx_desc;
	u32 total_bytes = 0U, total_packets = 0U;
	unsigned int budget = AQ_RING_TX_BUDGET;
	unsigned int i = self->next_to_clean;
	struct net_device *ndev = aq_nic_get_ndev(self->aq_nic);
	struct device *dev = aq_nic_get_dev(self->aq_nic);

	tx_buffer = &self->tx_buff_info[i];
	tx_desc = (struct hw_atl_txd_s*)&self->dx_ring[i * sizeof(struct hw_atl_txd_s)];
	i -= self->size;

	do {
		struct hw_atl_txd_s *eop_desc = tx_buffer->next_to_watch;
		/* if next_to_watch is not set then there is no work pending */
		if (!eop_desc)
			break;

		/* prevent any other reads prior to eop_desc */
		read_barrier_depends();

		if (!eop_desc->dd)
			break;

		/* clear next_to_watch to prevent false hangs */
		tx_buffer->next_to_watch = NULL;

		/* update the statistics for this packet */
		total_bytes += tx_buffer->byte_count;
		total_packets += tx_buffer->gso_segs;

		/* free the skb */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
		napi_consume_skb(tx_buffer->skb, napi_budget);
#else
		dev_consume_skb_any(tx_buffer->skb);
#endif

		/* unmap skb header data */
		dma_unmap_single(dev,
						 dma_unmap_addr(tx_buffer, dma),
						 dma_unmap_len(tx_buffer, len),
						 DMA_TO_DEVICE);

		/* clear tx_buffer data */
		tx_buffer->skb = NULL;
		dma_unmap_len_set(tx_buffer, len, 0);

		while (tx_desc != eop_desc) {
			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(!i)) {
				i -= self->size;
				tx_buffer = self->tx_buff_info;
				tx_desc = (struct hw_atl_txd_s*)self->dx_ring;
			}

			if (dma_unmap_len(tx_buffer, len)) {
				dma_unmap_page(dev,
							   dma_unmap_addr(tx_buffer, dma),
							   dma_unmap_len(tx_buffer, len),
							   DMA_TO_DEVICE);
				dma_unmap_len_set(tx_buffer, len, 0);
				tx_buffer->dma = 0;
			}
		}

		/* move us one more past the eop_desc for start of next pkt */
		tx_buffer++;
		tx_desc++;
		i++;
		if (unlikely(!i)) {
			i -= self->size;
			tx_buffer = self->tx_buff_info;
			tx_desc = (struct hw_atl_txd_s*)self->dx_ring;
		}

		prefetch(tx_desc);

		budget--;
	} while (likely(budget));

	i += self->size;
	self->next_to_clean = i;

	self->stats.tx.packets += total_packets;
	self->stats.tx.bytes += total_bytes;

	netdev_tx_completed_queue(txring_txq(self), total_packets, total_bytes);

	if (unlikely(total_packets && netif_carrier_ok(ndev) &&
		(aq_ring_desc_unused(self) >= TX_WAKE_THRESHOLD))) {
		smp_mb();
		if (__netif_subqueue_stopped(ndev, self->idx)) {
			netif_wake_subqueue(ndev, self->idx);
			++self->stats.tx.restart_queue;
		}
	}

	return !!budget;
}

static inline void aq_ring_rx_checksum(struct aq_ring_s *self,
								union hw_atl_rxd_s *rx_desc,
								struct sk_buff *skb)
{
	struct net_device *ndev = aq_nic_get_ndev(self->aq_nic);

	skb_checksum_none_assert(skb);

	if (!(ndev->features & NETIF_F_RXCSUM))
		return;

	if (rx_desc->wb.rx_cntl & 0x3U) {// ip cso and tcp/udp cso enable
		if ((rx_desc->wb.pkt_type & 0x3U) == 0) { //packet is not ARP

			if ((rx_desc->wb.stat & BIT(2)) == 0) { //no cso err
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
				__skb_incr_checksum_unnecessary(skb);

				if (rx_desc->wb.pkt_type & BIT(2) || rx_desc->wb.pkt_type & BIT(3)) {
					__skb_incr_checksum_unnecessary(skb);
				}
#else
				skb->ip_summed = CHECKSUM_UNNECESSARY;
#endif
			} else {
				skb->ip_summed = CHECKSUM_NONE;
			}
		} else {
			skb->ip_summed = CHECKSUM_NONE;
		}
	} else {
		skb->ip_summed = CHECKSUM_NONE;
	}

	if (rx_desc->wb.pkt_len <= 60) {
		skb->ip_summed = CHECKSUM_NONE;
	}
}

static void aq_ring_process_skb_fields(struct aq_ring_s *self,
								union hw_atl_rxd_s *rx_desc,
								struct sk_buff *skb)
{
	struct net_device *ndev = aq_nic_get_ndev(self->aq_nic);
	u16 hdr_len = 0U;

	/* RSC stats */
	if (AQ_CB(skb)->append_cnt) {
		hdr_len = skb_headlen(skb);

		skb_shinfo(skb)->gso_size = DIV_ROUND_UP((skb->len - hdr_len),
							AQ_CB(skb)->append_cnt);

		skb_shinfo(skb)->gso_type = SKB_GSO_TCPV4;

		self->stats.rx.rsc_count += AQ_CB(skb)->append_cnt;
		self->stats.rx.rsc_flush++;

		AQ_CB(skb)->append_cnt = 0;
	}

	/* RSS hash */
	if ((ndev->features & NETIF_F_RXHASH) && rx_desc->wb.rss_type) {
		skb_set_hash(skb, le32_to_cpu(rx_desc->wb.rss_hash),
				 (rx_desc->wb.rss_type == 0x4 || rx_desc->wb.rss_type == 0x5) ?
				 PKT_HASH_TYPE_L4 : PKT_HASH_TYPE_L3);
	}

	aq_ring_rx_checksum(self, rx_desc, skb);

	if ((ndev->features & NETIF_F_HW_VLAN_CTAG_RX) && rx_desc->wb.pkt_type & BIT(5)) { //vlan tagged
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), rx_desc->wb.vlan);
	}

	skb_record_rx_queue(skb, self->idx);
	skb->protocol = eth_type_trans(skb, ndev);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
static unsigned int aq_ring_get_headlen(unsigned char *data,
				      unsigned int max_len)
{
	union {
		unsigned char *network;
		struct ethhdr *eth;
		struct vlan_hdr *vlan;
		struct iphdr *ipv4;
		struct ipv6hdr *ipv6;
	} hdr;
	__be16 protocol;
	u8 nexthdr = 0;	/* default to not TCP */
	u8 hlen;

	if (max_len < ETH_HLEN)
		return max_len;

	hdr.network = data;
	protocol = hdr.eth->h_proto;
	hdr.network += ETH_HLEN;

	/* handle any vlan tag if present */
	if (protocol == htons(ETH_P_8021Q)) {
		if ((hdr.network - data) > (max_len - VLAN_HLEN))
			return max_len;

		protocol = hdr.vlan->h_vlan_encapsulated_proto;
		hdr.network += VLAN_HLEN;
	}

	/* handle L3 protocols */
	if (protocol == htons(ETH_P_IP)) {
		if ((hdr.network - data) > (max_len - sizeof(struct iphdr)))
			return max_len;

		/* access ihl as a u8 to avoid unaligned access on ia64 */
		hlen = (hdr.network[0] & 0x0F) << 2;

		/* verify hlen meets minimum size requirements */
		if (hlen < sizeof(struct iphdr))
			return hdr.network - data;

		/* record next protocol if header is present */
		if (!(hdr.ipv4->frag_off & htons(IP_OFFSET)))
			nexthdr = hdr.ipv4->protocol;
	} else if (protocol == htons(ETH_P_IPV6)) {
		if ((hdr.network - data) > (max_len - sizeof(struct ipv6hdr)))
			return max_len;

		/* record next protocol */
		nexthdr = hdr.ipv6->nexthdr;
		hlen = sizeof(struct ipv6hdr);
	} else {
		return hdr.network - data;
	}

	hdr.network += hlen;

	if (nexthdr == IPPROTO_TCP) {
		if ((hdr.network - data) > (max_len - sizeof(struct tcphdr)))
			return max_len;

		hlen = (hdr.network[12] & 0xF0) >> 2;

		if (hlen < sizeof(struct tcphdr))
			return hdr.network - data;

		hdr.network += hlen;
	} else if (nexthdr == IPPROTO_UDP) {
		if ((hdr.network - data) > (max_len - sizeof(struct udphdr)))
			return max_len;

		hdr.network += sizeof(struct udphdr);
	}

	if ((hdr.network - data) < max_len)
		return hdr.network - data;
	else
		return max_len;
}
#endif

static void aq_ring_pull_tail(struct aq_ring_s *self, struct sk_buff *skb)
{
	struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[0];
	unsigned char *va;
	unsigned int pull_len;

	va = skb_frag_address(frag);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	pull_len = eth_get_headlen(va, AQ_RING_HEADER_SIZE);
#else
	pull_len = aq_ring_get_headlen(va, AQ_RING_HEADER_SIZE);
#endif

	skb_copy_to_linear_data(skb, va, ALIGN(pull_len, sizeof(long)));

	skb_frag_size_sub(frag, pull_len);
	frag->page_offset += pull_len;
	skb->data_len -= pull_len;
	skb->tail += pull_len;
}

static bool aq_ring_cleanup_headers(struct aq_ring_s *self,
									union hw_atl_rxd_s *rx_desc,
									struct sk_buff *skb)
{
	struct net_device *ndev = aq_nic_get_ndev(self->aq_nic);

	if (unlikely(rx_desc->wb.stat & BIT(0)) && !(ndev->features & NETIF_F_RXALL)) {
		dev_kfree_skb_any(skb);
		return true;
	}

	if (skb_is_nonlinear(skb))
		aq_ring_pull_tail(self, skb);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	if (eth_skb_pad(skb))
		return true;
#else
	if (unlikely(skb->len < ETH_ZLEN)) {
		int pad_len = ETH_ZLEN - skb->len;
		if (skb_pad(skb, pad_len))
			return true;

		__skb_put(skb, pad_len);
	}
#endif

	return false;
}

static void aq_ring_rx_skb(struct napi_struct *napi, struct sk_buff *skb)
{
	skb_mark_napi_id(skb, napi);
	napi_gro_receive(napi, skb);
}

static void aq_ring_dma_sync_frag(struct aq_ring_s *self, struct sk_buff *skb)
{
	struct device *dev = aq_nic_get_dev(self->aq_nic);

	if (unlikely(AQ_CB(skb)->page_released)) {
		if (AQ_CB(skb)->dma) {
			dma_unmap_page(dev, AQ_CB(skb)->dma, PAGE_SIZE, DMA_FROM_DEVICE);
		}
		AQ_CB(skb)->page_released = false;
	} else {
		struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[0];

		dma_sync_single_range_for_cpu(dev,
			AQ_CB(skb)->dma,
			frag->page_offset,
			skb_frag_size(frag),
			DMA_FROM_DEVICE);
	}

	AQ_CB(skb)->dma = 0;
}

static void aq_ring_reuse_rx_page(struct aq_ring_s *self,
								  struct aq_ring_rx_buff_s *old_buff)
{
	struct device *dev = aq_nic_get_dev(self->aq_nic);
	u16 next_to_alloc = self->next_to_alloc;
	struct aq_ring_rx_buff_s *new_buff;

	new_buff = &self->rx_buff_info[next_to_alloc];

	next_to_alloc++;

	self->next_to_alloc = (next_to_alloc < self->size) ? next_to_alloc : 0;

	new_buff->dma = old_buff->dma;
	new_buff->page = old_buff->page;
	new_buff->page_offset = old_buff->page_offset;

	dma_sync_single_range_for_device(dev,
									 new_buff->dma,
									 new_buff->page_offset,
									 AQ_RING_2K,
									 DMA_FROM_DEVICE);
}

static bool aq_ring_add_rx_frag(struct aq_ring_s *self,
				struct aq_ring_rx_buff_s *rx_buff,
				union hw_atl_rxd_s *rx_desc,
				struct sk_buff *skb)
{
	unsigned int size = rx_desc->wb.pkt_len;
	struct device *dev = aq_nic_get_dev(self->aq_nic);

#if (PAGE_SIZE < 8192)
	unsigned int truesize = AQ_RING_2K;
#else
	unsigned int truesize = SKB_DATA_ALIGN(size);
	unsigned last_offset = PAGE_SIZE - AQ_RING_2K;
#endif

	BUG_ON(rx_buff->page == NULL);

	if ((size <= AQ_RING_HEADER_SIZE) && !skb_is_nonlinear(skb)) {
		unsigned char *va = page_address(rx_buff->page) + rx_buff->page_offset;

		memcpy(__skb_put(skb, size), va, ALIGN(size, sizeof(long)));

		if (unlikely(page_to_nid(rx_buff->page) != numa_node_id()))
			return true;

		if (rx_buff->dma) {
			dma_unmap_page(dev, rx_buff->dma, PAGE_SIZE, DMA_FROM_DEVICE);
			rx_buff->dma = 0;
		}

		__free_pages(rx_buff->page, 0);
		rx_buff->page = NULL;

		return false;
	}

	skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags, rx_buff->page,
					rx_buff->page_offset, size, truesize);
	if (unlikely(page_to_nid(rx_buff->page) != numa_node_id()))
		return false;

#if (PAGE_SIZE < 8192)
	if (unlikely(page_count(rx_buff->page) != 1))
		return false;

	rx_buff->page_offset ^= truesize;
#else
	rx_buff->page_offset += truesize;

	if (rx_buff->page_offset > last_offset)
		return false;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
	page_ref_inc(rx_buff->page);
#else
	atomic_inc(&rx_buff->page->_count);
#endif

	return true;
}

static struct sk_buff *aq_ring_fetch_rx_buffer(struct aq_ring_s *self,
					struct napi_struct *napi,
					union hw_atl_rxd_s *rx_desc)
{
	struct aq_ring_rx_buff_s *rx_buff;
	struct device *dev = aq_nic_get_dev(self->aq_nic);
	struct sk_buff *skb;
	struct page *page;

	rx_buff = &self->rx_buff_info[self->next_to_clean];
	page = rx_buff->page;
	prefetchw(page);

	skb = rx_buff->skb;

	if (likely(!skb)) {
		void *page_addr = page_address(page) + rx_buff->page_offset;

		prefetch(page_addr);

#if L1_CACHE_BYTES < 128
		prefetch(page_addr + L1_CACHE_BYTES);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
		skb = napi_alloc_skb(napi, AQ_RING_HEADER_SIZE);
#else
		skb = netdev_alloc_skb_ip_align(aq_nic_get_ndev(self->aq_nic), AQ_RING_HEADER_SIZE);
#endif

		if (unlikely(!skb)) {
			self->stats.rx.allocation_fails++;
			return NULL;
		}

		prefetchw(skb->data);

		if (likely(rx_desc->wb.eop))
			goto dma_sync;

		AQ_CB(skb)->dma = rx_buff->dma;
	} else {
		if (rx_desc->wb.eop)
			aq_ring_dma_sync_frag(self, skb);

dma_sync:
		dma_sync_single_range_for_cpu(dev,
					rx_buff->dma,
					rx_buff->page_offset,
					rx_desc->wb.pkt_len,
					DMA_FROM_DEVICE);
		rx_buff->skb = NULL;
	}

	if (aq_ring_add_rx_frag(self, rx_buff, rx_desc, skb)) {
		/* hand second half of page back to the ring */
		aq_ring_reuse_rx_page(self, rx_buff);
	} else if (AQ_CB(skb)->dma == rx_buff->dma) {
		/* the page has been released from the ring */
		AQ_CB(skb)->page_released = true;
	} else {
		/* we are not reusing the buffer so unmap it */
		if (rx_buff->dma) {
			dma_unmap_page(dev, rx_buff->dma, PAGE_SIZE, DMA_FROM_DEVICE);
			rx_buff->dma = 0;
		}
	}

	rx_buff->page = NULL;
	return skb;
}

int aq_ring_rx_clean_irq(struct aq_ring_s *self, struct napi_struct *napi, int budget)
{
	u32 err = 0;
	unsigned int total_rx_bytes =0, total_rx_packets = 0;
	u16 cleaned_count = aq_ring_desc_unused(self);

	while (likely(total_rx_packets < budget)) {
		union hw_atl_rxd_s *rx_desc;
		struct sk_buff *skb;

		if (cleaned_count >= 16) {
			aq_ring_alloc_rx_buffers(self, cleaned_count);
			cleaned_count = 0;
		}

		rx_desc = (union hw_atl_rxd_s *)&self->dx_ring[self->next_to_clean * sizeof(union hw_atl_rxd_s)];
		
		if (!(rx_desc->wb.dd)) {
			break;
		}
		
		err = rx_desc->wb.stat;
		err &= ~(AQ_HW_RXD_WB_CHECKSUM_VALID);
		
		rmb();

		skb = aq_ring_fetch_rx_buffer(self, napi, rx_desc);

		if (!skb)
			break;

		cleaned_count++;

		if (aq_ring_is_non_eop(self, rx_desc, skb))
			continue;

		if (aq_ring_cleanup_headers(self, rx_desc, skb))
			continue;

		total_rx_bytes += skb->len;

		aq_ring_process_skb_fields(self, rx_desc, skb);

		aq_ring_rx_skb(napi, skb);

		total_rx_packets++;
	}
	
	self->stats.rx.packets += total_rx_packets;
	self->stats.rx.bytes += total_rx_bytes;

	return total_rx_packets;
}

static bool skb_is_tcp(struct sk_buff *skb) {
	if (ip_hdr(skb)->version == 4)
		return ip_hdr(skb)->protocol == IPPROTO_TCP;
	else if (ip_hdr(skb)->version == 6) 
		return ipv6_hdr(skb)->nexthdr == NEXTHDR_TCP;
	return false;
}

static bool skb_is_udp(struct sk_buff *skb) {
	if (ip_hdr(skb)->version == 4)
		return ip_hdr(skb)->protocol == IPPROTO_UDP;
	else if (ip_hdr(skb)->version == 6)
		return  ipv6_hdr(skb)->nexthdr == NEXTHDR_UDP;
	return false;
}

static int aq_ring_tso(struct aq_ring_s *self,
					   struct aq_ring_tx_buff_s *first,
					   u8 *hdr_len)
{
	struct sk_buff *skb = first->skb;
	union hw_atl_txc_s *desc;
	u16 i = self->next_to_use;

	if (unlikely(!skb_is_gso(skb))) {
		return 0;
	}

	desc = (union hw_atl_txc_s *)&self->dx_ring[i * sizeof(union hw_atl_txc_s)];
	i++;
	self->next_to_use = (i < self->size) ? i : 0;

	desc->flags1 = 0U;
	desc->flags2 = 0U;

	desc->l2_len = ETH_HLEN;;
	desc->l3_len = ip_hdrlen(skb);
	desc->l4_len = tcp_hdrlen(skb);
	desc->mss_len = skb_shinfo(skb)->gso_size;

	desc->desc_type = AQ_HW_TXD_DESC_TYPE_TXC;

	if (ip_hdr(skb)->version == 6)
		desc->ct_cmd |= BIT(1); //IPv6

	desc->ct_cmd = skb_is_tcp(skb) ? BIT(2) : 0U;

	*hdr_len = desc->l2_len + desc->l3_len + desc->l4_len;

	return 1;
}

static inline int aq_ring_maybe_stop_tx(struct aq_ring_s *self, u16 size)
{
	struct net_device *ndev = aq_nic_get_ndev(self->aq_nic);
	
	if (likely(aq_ring_desc_unused(self) >= size))
		return 0;

	netif_stop_subqueue(ndev, self->idx);

	smp_mb();

	/* We need to check again in a case another CPU has just made room available. */
	if (likely(aq_ring_desc_unused(self) < size))
		return -EBUSY;

	/* A reprieve! - use start_queue because it doesn't call schedule */
	netif_start_subqueue(ndev, self->idx);
	++self->stats.tx.restart_queue;

	return 0;
}

void aq_ring_unmap_and_free_tx_resource(struct aq_ring_s *self,
										struct aq_ring_tx_buff_s *buff)
{
	struct device *dev = aq_nic_get_dev(self->aq_nic);

	if (buff->skb) {
		dev_kfree_skb_any(buff->skb);
		if (dma_unmap_len(buff, len))
			dma_unmap_single(dev,
							 dma_unmap_addr(buff, dma),
							 dma_unmap_len(buff, len),
							 DMA_TO_DEVICE);
	} else if (dma_unmap_len(buff, len)) {
		dma_unmap_page(dev,
						   dma_unmap_addr(buff, dma),
						   dma_unmap_len(buff, len),
						   DMA_TO_DEVICE);
	}

	buff->dma = 0;
	buff->next_to_watch = NULL;
	buff->skb = NULL;
	dma_unmap_len_set(buff, len, 0);
}

static void aq_ring_tx_map(struct aq_ring_s *self, struct aq_ring_tx_buff_s *first, const u8 hdr_len)
{
	struct sk_buff *skb = first->skb;
	struct aq_ring_tx_buff_s *tx_buffer;
	struct hw_atl_txd_s *tx_desc;
	struct skb_frag_struct *frag;
	dma_addr_t dma;
	u32 data_len, size;
	u16 i = self->next_to_use;
	struct device *dev = aq_nic_get_dev(self->aq_nic);

	tx_desc = (struct hw_atl_txd_s *)&self->dx_ring[i * sizeof(struct hw_atl_txd_s)];

	size = skb_headlen(skb);
	data_len = skb->data_len;

	dma = dma_map_single(dev, skb->data, size, DMA_TO_DEVICE);

	tx_buffer = first;

	tx_desc->flags = 0U;
	tx_desc->pay_len = skb->len - hdr_len;

	if (hdr_len) {
		tx_desc->tx_cmd |= BIT(4); //LSO
		tx_desc->ct_en = 1U; //enable context
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		if (skb->protocol == htons(ETH_P_IP))
			tx_desc->tx_cmd |= BIT(2);// ip CSO

		if (skb_is_tcp(skb) || skb_is_udp(skb))
			tx_desc->tx_cmd |= BIT(3); // enable tcp/udp cso
	}

	for (frag = &skb_shinfo(skb)->frags[0];; frag++) {
		if (dma_mapping_error(dev, dma))
			goto dma_error;

		/* record length, and DMA address */
		dma_unmap_len_set(tx_buffer, len, size);
		dma_unmap_addr_set(tx_buffer, dma, dma);

		tx_desc->buf_addr = cpu_to_le64(dma);

		while (unlikely(size > AQ_CFG_TX_FRAME_MAX)) {
			tx_desc->buf_len = AQ_CFG_TX_FRAME_MAX;
			tx_desc->desc_type = 1U; //tx descriptor
			
			i++;
			tx_desc++;

			if (i == self->size) {
				tx_desc = (struct hw_atl_txd_s *)self->dx_ring;
				i = 0;
			}

			dma += AQ_CFG_TX_FRAME_MAX;
			size -= AQ_CFG_TX_FRAME_MAX;

			tx_desc->buf_addr = cpu_to_le64(dma);
			tx_desc->flags = 0U;
		}

		if (likely(!data_len))
			break;

		tx_desc->buf_len = size;
		tx_desc->desc_type = AQ_HW_TXD_DESC_TYPE_TXD;
		tx_desc->eop = 0U;

		i++;
		tx_desc++;

		if (i == self->size) {
			tx_desc = (struct hw_atl_txd_s *)self->dx_ring;
			i = 0;
		}

		tx_desc->flags = 0U;
 
		size = skb_frag_size(frag);
		data_len -= size;

		dma = skb_frag_dma_map(dev, frag, 0, size, DMA_TO_DEVICE);

		tx_buffer = &self->tx_buff_info[i];
	}

	/* write last descriptor with RS and EOP bits */
	tx_desc->desc_type = AQ_HW_TXD_DESC_TYPE_TXD; //tx descriptor
	tx_desc->eop = 1U;
	tx_desc->tx_cmd |= 0x20; //enable wb;
	tx_desc->buf_len = size;

	netdev_tx_sent_queue(txring_txq(self), first->byte_count);

	wmb();

	/* set next_to_watch value indicating a packet is present */
	first->next_to_watch = tx_desc;

	i++;
	if (i == self->size) {
		i = 0;
	}	

	self->next_to_use = i;

	aq_ring_maybe_stop_tx(self, DESC_NEEDED);

	//if (netif_xmit_stopped(txring_txq(self)) || !skb->xmit_more) {
		reg_tx_dma_desc_tail_ptr_set(self->aq_hw, i, self->idx);
		mmiowb();
	//}

	return;

dma_error:

	/* clear dma mappings for failed tx_buffer_info map */
	for (;;) {
		tx_buffer = &self->tx_buff_info[i];
		aq_ring_unmap_and_free_tx_resource(self, tx_buffer);
		if (tx_buffer == first)
			break;
		if (i == 0)
			i = self->size;
		i--;
	}

	self->next_to_use = i;
}

netdev_tx_t aq_ring_transmit_skb(struct aq_ring_s *self, struct sk_buff *skb)
{
	struct aq_ring_tx_buff_s *first;
	unsigned short f;
	u16 count = TXD_USE_COUNT(skb_headlen(skb));
	__be16 protocol = skb->protocol;
	u8 hdr_len = 0;

	for (f = 0; f < skb_shinfo(skb)->nr_frags; f++)
		count += TXD_USE_COUNT(skb_shinfo(skb)->frags[f].size);

	if (aq_ring_maybe_stop_tx(self, count + 3)) {
		self->stats.tx.busy_count++;
		return NETDEV_TX_BUSY;
	}

	/* record the location of the first descriptor for this packet */
	first = &self->tx_buff_info[self->next_to_use];
	first->skb = skb;
	first->byte_count = skb->len;
	first->gso_segs = 1;

	first->vlan_tag = -1;

	protocol = vlan_get_protocol(skb);

	first->protocol = protocol;

	aq_ring_tso(self, first, &hdr_len);
		
	aq_ring_tx_map(self, first, hdr_len);

	return NETDEV_TX_OK;
}

static void aq_ring_clean_rx(struct aq_ring_s *self)
{
	struct device *dev = aq_nic_get_dev(self->aq_nic);
	unsigned long size;
	u16 i;

	if (!self->rx_buff_info) {
		return;
	}

	for (i = 0; i < self->size; i++) {
		struct aq_ring_rx_buff_s *buff = &self->rx_buff_info[i];

		if (buff->skb) {
			struct sk_buff *skb = buff->skb;
			if (AQ_CB(skb)->page_released) {
				dma_unmap_page(dev,
							   AQ_CB(skb)->dma,
							   AQ_RING_2K,
							   DMA_FROM_DEVICE);
				AQ_CB(skb)->dma = 0;
			}

			dev_kfree_skb(skb);
			buff->skb = NULL;
		}

		if (!buff->page)
			continue;

		dma_unmap_page(dev, buff->dma, PAGE_SIZE, DMA_FROM_DEVICE);
		buff->dma = 0;
		__free_pages(buff->page, 0);

		buff->page = NULL;
	}
 
	size = sizeof(struct aq_ring_rx_buff_s) * self->size;
	memset(self->rx_buff_info, 0, size);

	memset(self->dx_ring, 0, self->dx_mem_size);

	self->next_to_alloc = 0;
	self->next_to_clean = 0;
	self->next_to_use = 0;
}

static void aq_ring_clean_tx(struct aq_ring_s *self)
{
	struct aq_ring_tx_buff_s *buff;
	unsigned long size;
	u16 i;

	if (!self->tx_buff_info)
		return;

	for (i = 0; i < self->size; i++) {
		buff = &self->tx_buff_info[i];
		aq_ring_unmap_and_free_tx_resource(self, buff);
	}

	netdev_tx_reset_queue(txring_txq(self));

	size = sizeof(struct aq_ring_tx_buff_s) * self->size;
	memset(self->tx_buff_info, 0, size);

	memset(self->dx_ring, 0, self->dx_mem_size);

	self->next_to_use = 0;
	self->next_to_clean = 0;
}

void aq_ring_free_tx(struct aq_ring_s *self)
{
	aq_ring_clean_tx(self);

	vfree(self->tx_buff_info);
	self->tx_buff_info = NULL;

	if (!self->dx_ring)
		return;

	dma_free_coherent(aq_nic_get_dev(self->aq_nic),
			self->size * self->dx_size,
			self->dx_ring, self->dma);

	self->dx_ring = NULL;
}

void aq_ring_free_rx(struct aq_ring_s *self)
{
	aq_ring_clean_rx(self);

	vfree(self->rx_buff_info);
	self->rx_buff_info = NULL;

	if (!self->dx_ring)
		return;

	dma_free_coherent(aq_nic_get_dev(self->aq_nic),
			self->size * self->dx_size,
			self->dx_ring, self->dma);
 
	self->dx_ring = NULL;
}
