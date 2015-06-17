/*
 * "drivers/net/can/m_ttcan/m_ttcan_linux.c"
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * References are taken from "Bosch C_CAN controller" at
 * "drivers/net/can/c_can/c_can.c"
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

#include "hw/m_ttcan.h"
#include <linux/platform_device.h>

#ifndef CONFIG_OF
#define NV_ADDRESS_MAP_CAN1_0_INTR_ID (40 + 32)
#define NV_ADDRESS_MAP_CAN2_0_INTR_ID (42 + 32)
#define NV_ADDRESS_MAP_CAN1_BASE 0x0c310000
#define NV_ADDRESS_MAP_CAN2_BASE 0x0c320000
#define NV_ADDRESS_MAP_CAR_BASE 0x05000000
#define CAN_GLUE_ADDR 0x1000
#define CAN_MES_RAM_BASE_ADDR 0x2000

#define CAR_CAN1_SET_OFF 0xb10004
#define CAR_CAN1_CLR_OFF 0xb10008
#define CAR_CAN2_SET_OFF 0xb20004
#define CAR_CAN2_CLR_OFF 0xb20008

#endif

#define CAN_HOST_CLK_RATE 20000000
#define MTTCAN_POLL_TIME 50

#define TX_CONF_BUF 0x1
#define TX_CONF_Q 0x2
#define TX_CONF_MODE 0x4
/* Tx buffer configuration tx_conf
 * bit(0) = tx_buffer elements enabled
 * bit(1) = tx_fifo/queue elements enabled
 * bit(2) = fifo/queue mode. Fifo = 0, Queue = 1
 * default: Tx buffers and Tx Queue enabled
 */
static int tx_conf = 0x3;
module_param(tx_conf, int, 0644);
MODULE_PARM_DESC(tx_conf, "Tx Buffer config");

static bool tt_en;
module_param(tt_en, bool, 0644);
MODULE_PARM_DESC(tt_en, "TT CAN enabled");

static bool mttcan_poll;
module_param(mttcan_poll, bool, 0444);
MODULE_PARM_DESC(mttcan_poll, "Poll instead of interrupt");

static bool car_present = 1;
module_param(car_present, bool, 0444);
MODULE_PARM_DESC(car_present, "Clock and Reset present in bitstream");

static unsigned int tt_stop;
module_param(tt_stop, uint, 0644);
MODULE_PARM_DESC(tt_stop, "Stop after tt_stop TT interrupts");

static unsigned int bs_clock = 10000000;
module_param(bs_clock, uint, 0644);
MODULE_PARM_DESC(bs_clock, "Bitstream clock");

static __init int mttcan_hw_init(const struct mttcan_priv *priv)
{
	int err = 0;
	int tx_buf = 0, tx_q = 0, tx_mode = 0;
	u32 ie = 0, ttie = 0, gfc_reg = 0;
	struct ttcan_controller *ttcan = priv->ttcan;

	if (car_present)
		ttcan_reset_controller(ttcan);

	ttcan_set_ok(ttcan);

	ttcan_power_down(ttcan, 0);

	if (!mttcan_poll) {
		ie = 0x3BBFF7FF;
		ttie = 0x50C03;
	}
	err = ttcan_controller_init(ttcan, ie, ttie);
	if (err)
		return err;

	err = ttcan_mesg_ram_config(ttcan);
	if (err)
		return err;

	if (tx_conf) {
		if (tx_conf & TX_CONF_BUF)
			tx_buf = CONF_TX_BUFFER_ELEMS;
		if (tx_conf & TX_CONF_Q)
			tx_q = CONF_TX_FIFO_ELEMS;
		if (tx_conf & TX_CONF_MODE)
			tx_mode = 1;
	}

	ttcan_set_config_change_enable(ttcan);

	/* Accept unmatched in Rx FIFO0 and reject all remote frame */
	gfc_reg = (GFC_ANFS_RXFIFO_0 | GFC_ANFE_RXFIFO_0 | GFC_RRFS_REJECT |
	     GFC_RRFE_REJECT);

	err = ttcan_set_gfc(ttcan, gfc_reg);
	if (err)
		return err;

	/* Reset XIDAM to default */
	ttcan_set_xidam(ttcan, DEF_MTTCAN_XIDAM);

	/* Rx buffers set */
	ttcan_set_rx_buffer_addr(ttcan);
	ttcan_set_rx_fifo0(ttcan, CONF_RX_FIFO_0_ELEMS,
		(CONF_RX_FIFO_0_ELEMS/2));
	ttcan_set_rx_fifo1(ttcan, CONF_RX_FIFO_1_ELEMS,
		(CONF_RX_FIFO_1_ELEMS/2));

	ttcan_config_rx_data_elem_sizes(ttcan, BYTE64, BYTE64, BYTE64);

	ttcan_set_std_id_filter_addr(ttcan, CONF_11_BIT_FILTER_ELEMS);
	ttcan_reset_std_id_filter(ttcan, CONF_11_BIT_FILTER_ELEMS);
	ttcan_set_xtd_id_filter_addr(ttcan, CONF_29_BIT_FILTER_ELEMS);
	ttcan_reset_xtd_id_filter(ttcan, CONF_29_BIT_FILTER_ELEMS);


	ttcan_set_time_stamp_conf(ttcan, 9, TS_INTERNAL);
	ttcan_set_txevt_fifo_conf(ttcan, CONF_TX_EVENT_FIFO_ELEMS/2,
		CONF_TX_EVENT_FIFO_ELEMS);

	/* All Tx buffers as Tx Fifo (mode 0) Tx Queue (mode 1) */
	ttcan_set_tx_buffer_addr(ttcan, tx_buf, tx_q, BYTE64, tx_mode);

	if (tt_en) {
		dev_info(priv->device, "TTCAN Enabled\n");
		ttcan_disable_auto_retransmission(ttcan);
		ttcan_set_trigger_mem_conf(ttcan, CONF_TRIG_ELEMS);
		ttcan_reset_trigger_mem(ttcan, CONF_TRIG_ELEMS);
		ttcan_set_tur_config(ttcan, 0x0800, 0x0000, 1);
	}

	ttcan_clear_intr(ttcan);
	ttcan_clear_tt_intr(ttcan);

	ttcan_print_version(ttcan);

	return err;
}

static const struct can_bittiming_const mttcan_normal_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 255,
	.tseg2_min = 0,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 127,
	.sjw_max = 127,
	.brp_min = 1,
	.brp_max = 511,
	.brp_inc = 1,
};

static const struct can_bittiming_const mttcan_data_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 1,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 31,
	.tseg2_min = 0,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 15,
	.sjw_max = 15,
	.brp_min = 1,
	.brp_max = 15,
	.brp_inc = 1,
};

#if 0
static const struct ttcan_bittiming data_bt_config = {
	/* 1Mbit @ 20MHz */
	.bitrate = 0,
	.sampling_point = 0,
	.tq = 0,
	.brp = 1,
	.prop_seg = 1,
	.phase_seg1 = 12,
	.phase_seg2 = 6,
	.sjw = 6,
	.tdc = 0,
	.tdc_offset = 14,
	.tdc_filter_window = 0,
};

static const struct ttcan_bittiming nominal_bt_config = {
	/* 0.5Mbit @ 10MHz */
	.bitrate = 0,
	.sampling_point = 0,
	.tq = 0,
	.brp = 1,
	.prop_seg = 1,
	.phase_seg1 = 13,
	.phase_seg2 = 5,
	.sjw = 5,
	.tdc = 1,
	.tdc_offset = 14,
	.tdc_filter_window = 0,
};

static const struct ttcan_bittiming data_bt_config = {
	/* 2Mbit @ 40MHz */
	.bitrate = 0,
	.sampling_point = 0,
	.tq = 0,
	.brp = 1,
	.prop_seg = 1,
	.phase_seg1 = 12,
	.phase_seg2 = 6,
	.sjw = 6,
	.tdc = 0,
	.tdc_offset = 14,
	.tdc_filter_window = 0,
};

static const struct ttcan_bittiming nominal_bt_config = {
	/* 0.5Mbit @ 40MHz */
	.bitrate = 0,
	.sampling_point = 0,
	.tq = 0,
	.brp = 1,
	.prop_seg = 47,
	.phase_seg1 = 16,
	.phase_seg2 = 16,
	.sjw = 16,
	.tdc = 1,
	.tdc_offset = 14,
	.tdc_filter_window = 0,
};
#endif
static struct platform_device_id mttcan_id_table[] = {
	[0] = {
	       .name = "mttcan",
	       .driver_data = 0,
	       }, {
		   }
};

MODULE_DEVICE_TABLE(platform, mttcan_id_table);

static const struct of_device_id mttcan_of_table[] = {
	{ .compatible = "bosch,mttcan", .data = &mttcan_id_table[0]},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, mttcan_of_table);

static inline void mttcan_pm_runtime_enable(const struct mttcan_priv *priv)
{
	if (priv->device)
		pm_runtime_enable(priv->device);
}

static inline void mttcan_pm_runtime_disable(const struct mttcan_priv *priv)
{
	if (priv->device)
		pm_runtime_disable(priv->device);
}

static inline void mttcan_pm_runtime_get_sync(const struct mttcan_priv *priv)
{
	if (priv->device)
		pm_runtime_get_sync(priv->device);
}

static inline void mttcan_pm_runtime_put_sync(const struct mttcan_priv *priv)
{
	if (priv->device)
		pm_runtime_put_sync(priv->device);
}

static void mttcan_handle_lost_frame(struct net_device *dev, int fifo_num)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	u32 ack_ir;
	struct sk_buff *skb;
	struct can_frame *frame;

	if (fifo_num)
		ack_ir = MTT_IR_RF1L_MASK;
	else
		ack_ir = MTT_IR_RF0L_MASK;
	ttcan_ir_write(priv->ttcan, ack_ir);

	skb = alloc_can_err_skb(dev, &frame);
	if (unlikely(!skb))
		return;

	frame->can_id |= CAN_ERR_CRTL;
	frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
	stats->rx_errors++;
	stats->rx_over_errors++;
	netif_receive_skb(skb);
}

static int mttcan_do_receive(struct net_device *dev,
			     struct ttcanfd_frame *canfd)
{
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct canfd_frame *frame;

	skb = alloc_canfd_skb(dev, &frame);
	if (!skb) {
		stats->rx_dropped++;
		return 0;
	}

	memcpy(frame, canfd, sizeof(struct canfd_frame));

	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += frame->len;

	return 1;
}

static int mttcan_read_rcv_list(struct net_device *dev,
				struct list_head *rcv,
				enum ttcan_rx_type rx_type,
				int rec_msgs, int quota)
{
	unsigned int pushed;
	unsigned long flags;
	struct mttcan_priv *priv = netdev_priv(dev);
	struct ttcan_rx_msg_list *rx;
	struct net_device_stats *stats = &dev->stats;
	struct list_head *cur, *next, rx_q;

	if (list_empty(rcv))
		return 0;

	INIT_LIST_HEAD(&rx_q);

	spin_lock_irqsave(&priv->ttcan->lock, flags);
	switch (rx_type) {
	case BUFFER:
		priv->ttcan->rxb_mem = 0;
		priv->ttcan->list_status &= ~(BUFFER & 0xFF);
		break;
	case FIFO_0:
		priv->ttcan->rxq0_mem = 0;
		priv->ttcan->list_status &= ~(FIFO_0 & 0xFF);
		break;
	case FIFO_1:
		priv->ttcan->rxq1_mem = 0;
		priv->ttcan->list_status &= ~(FIFO_1 & 0xFF);
	default:
		break;
	}
	list_splice_init(rcv, &rx_q);
	spin_unlock_irqrestore(&priv->ttcan->lock, flags);

	pushed = rec_msgs;
	list_for_each_safe(cur, next, &rx_q) {
		struct sk_buff *skb;
		struct canfd_frame *fd_frame;
		struct can_frame *frame;
		if (!quota--)
			break;
		list_del_init(cur);

		rx = list_entry(cur, struct ttcan_rx_msg_list, recv_list);
		if (rx->msg.flags & CAN_FD_FLAG) {
			skb = alloc_canfd_skb(dev, &fd_frame);
			if (!skb) {
				stats->rx_dropped += pushed;
				return 0;
			}
			memcpy(fd_frame, &rx->msg, sizeof(struct canfd_frame));
			stats->rx_bytes += fd_frame->len;
		} else {
			skb = alloc_can_skb(dev, &frame);
			if (!skb) {
				stats->rx_dropped += pushed;
				return 0;
			}
			frame->can_id =  rx->msg.can_id;
			frame->can_dlc = rx->msg.d_len;
			memcpy(frame->data, &rx->msg.data, frame->can_dlc);
			stats->rx_bytes += frame->can_dlc;
		}

		kfree(rx);
		netif_receive_skb(skb);
		stats->rx_packets++;
		pushed--;
	}
	return rec_msgs - pushed;
}

static int mttcan_state_change(struct net_device *dev,
			       enum can_state error_type)
{
	u32 ecr;
	struct mttcan_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct can_berr_counter bec;

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	ecr = ttcan_read_ecr(priv->ttcan);
	bec.rxerr = (ecr & MTT_ECR_REC_MASK) >> MTT_ECR_REC_SHIFT;
	bec.txerr = (ecr & MTT_ECR_TEC_MASK) >> MTT_ECR_TEC_SHIFT;

	switch (error_type) {
	case CAN_STATE_ERROR_WARNING:
		/* error warning state */
		priv->can.can_stats.error_warning++;
		priv->can.state = CAN_STATE_ERROR_WARNING;
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = (bec.txerr > bec.rxerr) ?
		    CAN_ERR_CRTL_TX_WARNING : CAN_ERR_CRTL_RX_WARNING;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;

		break;
	case CAN_STATE_ERROR_PASSIVE:
		/* error passive state */
		priv->can.can_stats.error_passive++;
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		cf->can_id |= CAN_ERR_CRTL;
		if (ecr & MTT_ECR_RP_MASK)
			cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		if (bec.txerr > 127)
			cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;

		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case CAN_STATE_BUS_OFF:
		/* bus-off state */
		priv->can.state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		/*
		 * disable all interrupts in bus-off mode to ensure that
		 * the CPU is not hogged down
		 */
		ttcan_enable_all_interrupts(priv->ttcan, 0);
		can_bus_off(dev);
		break;
	default:
		break;
	}
	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 1;
}

static int mttcan_handle_bus_err(struct net_device *dev,
				 enum ttcan_lec_type lec_type)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;

	if (lec_type == LEC_NO_CHANGE || lec_type == LEC_NO_ERROR)
		return 0;
	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	/* common for all type of bus errors */
	priv->can.can_stats.bus_error++;
	stats->rx_errors++;
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
	cf->data[2] |= CAN_ERR_PROT_UNSPEC;

	switch (lec_type) {
	case LEC_STUFF_ERROR:
		netdev_warn(dev, "Stuff Error Detected\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		break;
	case LEC_FORM_ERROR:
		netdev_warn(dev, "Format Error Detected\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		break;
	case LEC_ACK_ERROR:
		netdev_warn(dev, "Acknowledgement Error Detected\n");
		cf->data[3] |= (CAN_ERR_PROT_LOC_ACK |
				CAN_ERR_PROT_LOC_ACK_DEL);
		break;
	case LEC_BIT1_ERROR:
		netdev_warn(dev, "Bit1 Error Detected\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		break;
	case LEC_BIT0_ERROR:
		netdev_warn(dev, "Bit0 Error Detected\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		break;
	case LEC_CRC_ERROR:
		netdev_warn(dev, "CRC Error Detected\n");
		cf->data[3] |= (CAN_ERR_PROT_LOC_CRC_SEQ |
				CAN_ERR_PROT_LOC_CRC_DEL);
		break;
	default:
		break;
	}

	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 1;
}

static u64 mttcan_ts_value(struct ttcan_controller *ttcan, unsigned short ts)
{
	return ((u64)ts * ttcan->ts_prescalar * 1000000 /
			ttcan->bt_config.nominal.bitrate) +
			ttcan->ts_counter;
}

static void mttcan_tx_event(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	struct ttcan_txevt_msg_list *evt;
	struct list_head *cur, *next, evt_q;
	struct mttcan_tx_evt_element txevt;
	u32 xtd, id;
	unsigned long flags;

	INIT_LIST_HEAD(&evt_q);

	spin_lock_irqsave(&priv->ttcan->lock, flags);

	if (list_empty(&priv->ttcan->tx_evt)) {
		spin_unlock_irqrestore(&priv->ttcan->lock, flags);
		return;
	}

	priv->ttcan->evt_mem = 0;
	priv->ttcan->list_status &= ~(TX_EVT & 0xFF);
	list_splice_init(&priv->ttcan->tx_evt, &evt_q);
	spin_unlock_irqrestore(&priv->ttcan->lock, flags);

	list_for_each_safe(cur, next, &evt_q) {
		list_del_init(cur);

		evt = list_entry(cur, struct ttcan_txevt_msg_list, txevt_list);
		memcpy(&txevt, &evt->txevt, sizeof(struct canfd_frame));
		kfree(evt);
		xtd = (txevt.f0 & MTT_TXEVT_ELE_F0_XTD_MASK) >>
			MTT_TXEVT_ELE_F0_XTD_SHIFT;
		id = (txevt.f0 & MTT_TXEVT_ELE_F0_ID_MASK) >>
			MTT_TXEVT_ELE_F0_ID_SHIFT;
		pr_info("%s:TS %llu:(index %u) ID %x(%s %s %s) Evt_Type %02d\n",
			__func__, mttcan_ts_value(priv->ttcan, txevt.f1 &
			MTT_TXEVT_ELE_F1_TXTS_MASK), (txevt.f1 &
			MTT_TXEVT_ELE_F1_MM_MASK) >> MTT_TXEVT_ELE_F1_MM_SHIFT,
			xtd ? id : id >> 18, xtd ? "XTD" : "STD",
			txevt.f1 & MTT_TXEVT_ELE_F1_FDF_MASK ? "FD" : "NON-FD",
			txevt.f1 & MTT_TXEVT_ELE_F1_BRS_MASK ? "BRS" : "NOBRS",
			(txevt.f1 & MTT_TXEVT_ELE_F1_ET_MASK)
			>> MTT_TXEVT_ELE_F1_ET_SHIFT);
	}
}

static void mttcan_echo_mesg(struct net_device *dev, u32 msg_no)
{
	struct can_priv *prv = netdev_priv(dev);
	struct sk_buff *skb = prv->echo_skb[msg_no];
	if (skb != NULL)
		can_get_echo_skb(dev, msg_no);
}

static void mttcan_tx_complete(struct net_device *dev)
{
	u32 completed_tx;
	u32 msg_no;

	struct mttcan_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;

	completed_tx = ttcan_read_tx_complete_reg(priv->ttcan);

	/*TODO:- fix how to handle wrap around of tx_next and echo */
	for (; (priv->tx_next - priv->tx_echo) > 0; priv->tx_echo++) {
		if (completed_tx != 0 && priv->tx_object != 0) {
			msg_no = ffs(priv->tx_object) - 1;
			if (completed_tx & (1 << (msg_no))) {
				/* Tx completed for this object */
				priv->tx_object &= ~(1 << msg_no);
				stats->tx_bytes +=
					priv->ttcan->tx_buf_dlc[msg_no];
				stats->tx_packets++;
				mttcan_echo_mesg(dev, msg_no);
				can_led_event(dev, CAN_LED_EVENT_TX);
			}
		} else {
			pr_info("%s TC %x priv->tx_object %x\n",
				__func__, completed_tx, priv->tx_object);
			break;
		}
	}

	if (netif_queue_stopped(dev))
		netif_start_queue(dev);

	if ((priv->tx_next - priv->tx_echo) > 0)
		netif_wake_queue(dev);
}

static void mttcan_tx_cancelled(struct net_device *dev)
{
	u32 msg_no;
	u32 buff_bit;
	u32 cancelled_msg;
	struct mttcan_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;


	msg_no = ttcan_read_tx_cancelled_reg(priv->ttcan);

	/* cancelled_msg are newly cancelled message for current interrupt */
	cancelled_msg = (priv->tx_obj_cancelled ^ msg_no) &
		~(priv->tx_obj_cancelled);
	priv->tx_obj_cancelled = msg_no;

	msg_no = ffs(cancelled_msg);
	while (msg_no) {
		buff_bit = (1 << (msg_no - 1));
		if (priv->tx_object & buff_bit) {
			priv->tx_object &= ~(buff_bit);
			cancelled_msg &= ~(buff_bit);
			stats->tx_aborted_errors++;
			can_free_echo_skb(dev, (msg_no - 1));
		} else {
			pr_info("%s TCF %x priv->tx_object %x\n", __func__,
					cancelled_msg, priv->tx_object);
			break;
		}
		priv->tx_echo++;
		msg_no = ffs(cancelled_msg);
	}
}

static int mttcan_poll_ir(struct napi_struct *napi, int quota)
{
	int work_done = 0;
	int rec_msgs = 0;
	struct net_device *dev = napi->dev;
	struct mttcan_priv *priv = netdev_priv(dev);
	u32 ir, ack, ttir, ttack, psr;
	ir = priv->irqstatus;
	ttir = priv->tt_irqstatus;

	netdev_dbg(dev, "IR %x\n", ir);
	if (!ir && !ttir)
		goto end;

	if (ir) {
		if (ir & MTTCAN_ERR_INTR) {
			psr = ttcan_read_psr(priv->ttcan);
			priv->ttcan->proto_state = psr;
			if ((ir & MTT_IR_EW_MASK) && (psr & MTT_PSR_EW_MASK)) {
				work_done += mttcan_state_change(dev,
					CAN_STATE_ERROR_WARNING);
				netdev_warn(dev,
					    "entered error warning state\n");
			}
			if ((ir & MTT_IR_EP_MASK) && (psr & MTT_PSR_EP_MASK)) {
				work_done += mttcan_state_change(dev,
					CAN_STATE_ERROR_PASSIVE);
				netdev_warn(dev,
					    "entered error passive state\n");
			}
			if ((ir & MTT_IR_BO_MASK) && (psr & MTT_PSR_BO_MASK)) {
				work_done +=
				    mttcan_state_change(dev, CAN_STATE_BUS_OFF);
				netdev_warn(dev, "entered bus off state\n");
			}
			if (((ir & MTT_IR_EP_MASK) && !(psr & MTT_PSR_EP_MASK))
				|| ((ir & MTT_IR_EW_MASK) &&
				!(psr & MTT_PSR_EW_MASK))) {
				if (ir & MTT_IR_EP_MASK)
					netdev_dbg(dev,
						"left error passive state\n");
				else
					netdev_dbg(dev,
						"left error warning state\n");

				priv->can.state = CAN_STATE_ERROR_ACTIVE;
			}

			/* Handle Bus error change */
			if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) {
				if ((ir & MTT_IR_PED_MASK) ||
					(ir & MTT_IR_PEA_MASK)) {
					enum ttcan_lec_type lec =
						(psr & MTT_PSR_LEC_MASK)
						>> MTT_PSR_LEC_SHIFT;
					work_done +=
					    mttcan_handle_bus_err(dev, lec);
					netdev_err(dev,	"IR = 0x%x PSR 0x%x\n",
						ir, psr);
				}
			}
			if (ir & MTT_IR_WDI_MASK)
				netdev_warn(dev,
					"Message RAM watchdog not handled\n");
			ack = MTTCAN_ERR_INTR;
			ttcan_ir_write(priv->ttcan, ack);
		}

		/* High Priority Message */
		if (ir & MTTCAN_RX_HP_INTR) {
			struct ttcanfd_frame ttcanfd;
			ack = MTT_IR_HPM_MASK;
			ttcan_ir_write(priv->ttcan, ack);
			if (ttcan_read_hp_mesgs(priv->ttcan, &ttcanfd))
				work_done += mttcan_do_receive(dev, &ttcanfd);
			pr_info("%s: hp mesg received\n", __func__);
		}

		/* Handle dedicated buffer */
		if (ir & MTT_IR_DRX_MASK) {
			ack = MTT_IR_DRX_MASK;
			ttcan_ir_write(priv->ttcan, ack);
			rec_msgs = ttcan_read_rx_buffer(priv->ttcan);
			work_done +=
			    mttcan_read_rcv_list(dev, &priv->ttcan->rx_b,
						 BUFFER, rec_msgs,
						 quota - work_done);
			pr_info("%s: buffer mesg received\n", __func__);

		}
		/* Handle RX Fifo interrupt */
		if (ir & MTTCAN_RX_FIFO_INTR) {
			if (ir & MTT_IR_RF1L_MASK) {
				mttcan_handle_lost_frame(dev, 1);
				work_done++;
				ack = MTT_IR_RF1L_MASK;
				ttcan_ir_write(priv->ttcan, ack);
			}
			if (ir & MTT_IR_RF0L_MASK) {
				mttcan_handle_lost_frame(dev, 0);
				work_done++;
				ack = MTT_IR_RF0L_MASK;
				ttcan_ir_write(priv->ttcan, ack);
			}

			if (ir & (MTT_IR_RF1F_MASK | MTT_IR_RF1W_MASK |
				MTT_IR_RF1N_MASK)) {
				ack = MTT_IR_RF1F_MASK | MTT_IR_RF1W_MASK |
					MTT_IR_RF1N_MASK;
				ttcan_ir_write(priv->ttcan, ack);

				rec_msgs = ttcan_read_rx_fifo1(priv->ttcan);
				work_done +=
				    mttcan_read_rcv_list(dev,
							 &priv->ttcan->rx_q1,
							 FIFO_1, rec_msgs,
							 quota - work_done);
				pr_info("%s: msg received in Q1\n", __func__);
			}
			if (ir & (MTT_IR_RF0F_MASK | MTT_IR_RF0W_MASK |
				MTT_IR_RF0N_MASK)) {
				ack = MTT_IR_RF0F_MASK | MTT_IR_RF0W_MASK |
					MTT_IR_RF0N_MASK;
				ttcan_ir_write(priv->ttcan, ack);
				rec_msgs = ttcan_read_rx_fifo0(priv->ttcan);
				work_done +=
				    mttcan_read_rcv_list(dev,
							 &priv->ttcan->rx_q0,
							 FIFO_0, rec_msgs,
							 quota - work_done);
				pr_info("%s: msg received in Q0\n", __func__);
			}
		}
		/* Handle Tx Event */
		if (ir & MTTCAN_TX_EV_FIFO_INTR) {
			/* New Tx Event */
			if ((ir & MTT_IR_TEFN_MASK) ||
				(ir & MTT_IR_TEFW_MASK)) {
				ttcan_read_txevt_fifo(priv->ttcan);
				mttcan_tx_event(dev);
			}

			if ((ir & MTT_IR_TEFL_MASK) &&
				priv->ttcan->tx_config.evt_q_size)
				pr_err("%s: Tx event lost\n", __func__);

			ack = MTTCAN_TX_EV_FIFO_INTR;
			ttcan_ir_write(priv->ttcan, ack);
		}

		/* Handle Timer wrap around */
		if (ir & MTT_IR_TSW_MASK) {
			/* milliseconds */
			priv->ttcan->ts_counter += priv->ttcan->ts_prescalar *
				65536 * 1000000 /
				priv->ttcan->bt_config.nominal.bitrate;
			ack = MTT_IR_TSW_MASK;
			ttcan_ir_write(priv->ttcan, ack);
		}

		/* Handle TX complete */
		if (ir & MTTCAN_TX_INTR) {
			/* Transmission cancellation finished */
			if (ir & MTT_IR_TCF_MASK)
				mttcan_tx_cancelled(dev);

			if (ir & MTT_IR_TC_MASK)
				mttcan_tx_complete(dev);

			/* if (ir & MTT_IR_TFE_MASK)
				netdev_warn(dev, "Tx Fifo Empty\n"); */

			ack = MTTCAN_TX_INTR;
			ttcan_ir_write(priv->ttcan, ack);
		}

	}

	if (ttir) {
		/* Handle CAN TT interrupts */
		unsigned int tt_err = 0;
		unsigned int ttost = 0;

		if (ttir & 0x7B100) {
			tt_err = 1;
			ttost = ttcan_get_ttost(priv->ttcan);
		}
		if (ttir & MTT_TTIR_CER_MASK)
			netdev_warn(dev, "TT Configuration Error\n");
		if (ttir & MTT_TTIR_AW_MASK)
			netdev_warn(dev, "TT Application wdt triggered\n");
		if (ttir & MTT_TTIR_WT_MASK)
			netdev_warn(dev, "TT Referrence Mesg missing\n");
		if (ttir & MTT_TTIR_IWT_MASK)
			netdev_warn(dev, "TT Initialization Watch Triggered\n");
		if (ttir & MTT_TTIR_SE2_MASK || ttir & MTT_TTIR_SE1_MASK)
			netdev_warn(dev, "TT Scheduling error SE%d\n",
				(ttir & MTT_TTIR_SE1_MASK) ? 1 : 2);
		if (ttir & MTT_TTIR_TXO_MASK)
			netdev_warn(dev, "TT Tx count overflow\n");
		if (ttir & MTT_TTIR_TXU_MASK)
			netdev_warn(dev, "TT Tx count underflow\n");
		if (ttir & MTT_TTIR_GTE_MASK)
			netdev_warn(dev, "TT Global timer error\n");
		if (ttir & MTT_TTIR_GTD_MASK)
			netdev_warn(dev, "TT Global time discontinuity\n");
		if (ttir & MTT_TTIR_GTW_MASK)
			netdev_info(dev, "TT Global time wrapped\n");
		if (ttir & MTT_TTIR_SWE_MASK)
			netdev_info(dev, "TT Stop watch event\n");
		if (ttir & MTT_TTIR_TTMI_MASK)
			netdev_warn(dev, "TT TMI event (int)\n");
		if (ttir & MTT_TTIR_RTMI_MASK)
			netdev_warn(dev, "TT Register TMI\n");
		if (ttir & MTT_TTIR_SOG_MASK)
			netdev_info(dev, "TT Start of Gap\n");
		if (ttir & MTT_TTIR_SMC_MASK)
			netdev_info(dev, "TT Start of Matrix Cycle\n");
		if (ttir & MTT_TTIR_SBC_MASK)
			netdev_info(dev, "TT Start of Basic Cycle\n");
		if (tt_err)
			netdev_err(dev, "TTOST 0x%x\n", ttost);
		ttack = 0xFFFFFFFF;
		ttcan_ttir_write(priv->ttcan, ttack);
	}
end:
	if (work_done < quota) {
		napi_complete(napi);
		ttcan_enable_all_interrupts(priv->ttcan, 1);
	}

	return work_done;
}

static int mttcan_get_berr_counter(const struct net_device *dev,
				   struct can_berr_counter *bec)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	u32 ecr;

	mttcan_pm_runtime_get_sync(priv);

	ecr = ttcan_read_ecr(priv->ttcan);
	bec->rxerr = (ecr & MTT_ECR_REC_MASK) >> MTT_ECR_REC_SHIFT;
	bec->txerr = (ecr & MTT_ECR_TEC_MASK) >> MTT_ECR_TEC_SHIFT;
	mttcan_pm_runtime_put_sync(priv);

	return 0;
}

static int mttcan_do_set_bittiming(struct net_device *dev)
{
	int err = 0;
	struct mttcan_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	const struct can_bittiming *dbt = &priv->can.data_bittiming;

	memcpy(&priv->ttcan->bt_config.nominal, bt,
		sizeof(struct can_bittiming));
	memcpy(&priv->ttcan->bt_config.data, dbt,
		sizeof(struct can_bittiming));

	/* memcpy(&priv->ttcan->bt_config.nominal, &nominal_bt_config,
	       sizeof(struct ttcan_bittiming)); */
	/* memcpy(&priv->ttcan->bt_config.data, &data_bt_config,
	       sizeof(struct ttcan_bittiming)); */
	if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		priv->ttcan->bt_config.fd_flags = CAN_FD_FLAG  | CAN_BRS_FLAG;
	else
		priv->ttcan->bt_config.fd_flags = 0;

	err = ttcan_set_bitrate(priv->ttcan);
	if (err) {
		netdev_err(dev, "Unable to set bitrate\n");
		return err;
	}

	netdev_info(dev, "Bitrate set\n");
	return 0;
}

static void mttcan_controller_config(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);

	/* set CCCR.INIT and then CCCR.CCE */
	ttcan_set_config_change_enable(priv->ttcan);

	pr_debug("%s: ctrlmode %x\n", __func__, priv->can.ctrlmode);
	/* enable automatic retransmission */
	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		ttcan_disable_auto_retransmission(priv->ttcan);

	if ((priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) &&
	    (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)) {
		/* internal loopback mode : useful for self-test function */
		ttcan_set_bus_monitoring_mode(priv->ttcan);
		ttcan_set_loopback(priv->ttcan);

	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* external loopback mode : useful for self-test function */
		ttcan_set_loopback(priv->ttcan);

	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* silent mode : bus-monitoring mode */
		ttcan_set_bus_monitoring_mode(priv->ttcan);
	} else
		/* clear bus montor or external loopback mode */
		ttcan_set_normal_mode(priv->ttcan);

	/* set bit timing and start controller */
	mttcan_do_set_bittiming(dev);
}

static void mttcan_start(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	u32 psr;

	mttcan_controller_config(dev);

	priv->tx_next = priv->tx_echo = 0;

	ttcan_enable_all_interrupts(priv->ttcan, 1);

	/* Set current state of CAN controller *
	 * It is assumed the controller is reset durning probing time
	 * It should be in sane state at first start but not guaranteed
	 */
	if (priv->ttcan->proto_state)
		psr = priv->ttcan->proto_state;
	else
		psr = ttcan_read_psr(priv->ttcan);

	if (psr & MTT_PSR_BO_MASK) {
		/* Bus off */
		priv->can.state = CAN_STATE_BUS_OFF;
		ttcan_enable_all_interrupts(priv->ttcan, 0);
		can_bus_off(dev);
	} else if (psr & MTT_PSR_EP_MASK) {
		/* Error Passive */
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
	} else if (psr * MTT_PSR_EW_MASK) {
		/* Error Warning */
		priv->can.state = CAN_STATE_ERROR_WARNING;
	} else {
		/* Error Active */
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
	}

	/* start Tx/Rx and enable protected mode */
	if (!tt_en)
		ttcan_reset_init(priv->ttcan);

	if (mttcan_poll)
		schedule_delayed_work(&priv->can_work,
			msecs_to_jiffies(MTTCAN_POLL_TIME));
}

static void mttcan_stop(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	ttcan_enable_all_interrupts(priv->ttcan, 0);

	priv->can.state = CAN_STATE_STOPPED;

	ttcan_set_config_change_enable(priv->ttcan);
}

static int mttcan_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		mttcan_start(dev);
		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

struct net_device *alloc_mttcan_dev(void)
{
	struct net_device *dev;
	struct mttcan_priv *priv;

	dev = alloc_candev(sizeof(struct mttcan_priv), MTT_CAN_TX_OBJ_NUM);
	if (!dev)
		return NULL;

	/* TODO:- check if we need to disable local loopback */
	dev->flags = (IFF_NOARP | IFF_ECHO);

	priv = netdev_priv(dev);

	priv->dev = dev;
	priv->can.bittiming_const = &mttcan_normal_bittiming_const;
	priv->can.data_bittiming_const = &mttcan_data_bittiming_const;
	priv->can.do_set_bittiming = mttcan_do_set_bittiming;
	priv->can.do_set_mode = mttcan_set_mode;
	priv->can.do_get_berr_counter = mttcan_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
	    CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_FD |
	    CAN_CTRLMODE_BERR_REPORTING | CAN_CTRLMODE_ONE_SHOT;

	if (can_change_mtu(dev, CANFD_MTU)) {
		netdev_err(dev, "unable to change MTU\n");
		free_candev(dev);
		return NULL;
	}

	netif_napi_add(dev, &priv->napi, mttcan_poll_ir, MTT_CAN_NAPI_WEIGHT);

	return dev;
}

static irqreturn_t mttcan_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct mttcan_priv *priv = netdev_priv(dev);

	priv->irqstatus = ttcan_read_ir(priv->ttcan);
	priv->tt_irqstatus = ttcan_read_ttir(priv->ttcan);

	if (!priv->irqstatus && !priv->tt_irqstatus)
		return IRQ_NONE;

	/* If tt_stop > 0, then stop when TT interrupt count > tt_stop */
	if (tt_stop && priv->tt_irqstatus)
		if (priv->tt_intrs++ > tt_stop)
			ttcan_set_config_change_enable(priv->ttcan);

	/* disable and clear all interrupts */
	ttcan_enable_all_interrupts(priv->ttcan, 0);

	/* schedule the NAPI */
	napi_schedule(&priv->napi);
	return IRQ_HANDLED;
}

static void mttcan_work(struct work_struct *work)
{
	struct mttcan_priv *priv = container_of(to_delayed_work(work),
						struct mttcan_priv, can_work);

	priv->irqstatus = ttcan_read_ir(priv->ttcan);
	priv->tt_irqstatus = ttcan_read_ttir(priv->ttcan);

	if (priv->irqstatus || priv->tt_irqstatus) {
		/* disable and clear all interrupts */
		ttcan_enable_all_interrupts(priv->ttcan, 0);

		/* schedule the NAPI */
		napi_schedule(&priv->napi);
	}
	schedule_delayed_work(&priv->can_work,
		msecs_to_jiffies(MTTCAN_POLL_TIME));
}

static int mttcan_open(struct net_device *dev)
{
	int err;
	struct mttcan_priv *priv = netdev_priv(dev);

	mttcan_pm_runtime_get_sync(priv);

	err = open_candev(dev);
	if (err) {
		netdev_err(dev, "failed to open can device\n");
		goto exit_open_fail;
	}

	err = devm_request_irq(priv->device, dev->irq, mttcan_isr, IRQF_SHARED,
			dev->name, dev);
	if (err < 0) {
		netdev_err(dev, "failed to request interrupt\n");
		goto fail;
	}

	napi_enable(&priv->napi);
	can_led_event(dev, CAN_LED_EVENT_OPEN);

	mttcan_start(dev);
	netif_start_queue(dev);

	return 0;

fail:
	close_candev(dev);
exit_open_fail:
	mttcan_pm_runtime_put_sync(priv);
	return err;
}

static int mttcan_close(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	mttcan_stop(dev);
	close_candev(dev);

	mttcan_pm_runtime_put_sync(priv);

	can_led_event(dev, CAN_LED_EVENT_STOP);
	return 0;
}

static netdev_tx_t mttcan_start_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	int msg_no = -1;
	struct mttcan_priv *priv = netdev_priv(dev);
	struct canfd_frame *frame = (struct canfd_frame *)skb->data;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	if (can_is_canfd_skb(skb))
		frame->flags |= CAN_FD_FLAG;

	if (tx_conf & TX_CONF_BUF)
		msg_no = ttcan_tx_msg_buffer_write(priv->ttcan,
				(struct ttcanfd_frame *)frame, tt_en);

	if ((tx_conf & TX_CONF_Q) && (msg_no < 0))
		msg_no = ttcan_tx_fifo_queue_msg(priv->ttcan,
				(struct ttcanfd_frame *)frame);

	if (msg_no < 0) {
		netdev_err(dev, "Tx message queue full\n");
		kfree_skb(skb);
		dev->stats.tx_dropped++;
		goto stopq;
	}

	can_put_echo_skb(skb, dev, msg_no);

	priv->tx_next++;
	priv->tx_object |= 1 << msg_no;
	priv->tx_obj_cancelled &= ~(1 << msg_no);

stopq:	/*if next tx is not possible stop the queue */
	if (ttcan_tx_fifo_full(priv->ttcan)) {
		netdev_err(dev, "Tx message queue full\n");
		netif_stop_queue(dev);
	}

	return NETDEV_TX_OK;
}

static int mttcan_change_mtu(struct net_device *dev, int new_mtu)
{
	if (dev->flags & IFF_UP)
		return -EBUSY;

	if (new_mtu != CANFD_MTU)
		dev->mtu = new_mtu;
	return 0;
}

static const struct net_device_ops mttcan_netdev_ops = {
	.ndo_open = mttcan_open,
	.ndo_stop = mttcan_close,
	.ndo_start_xmit = mttcan_start_xmit,
	.ndo_change_mtu = mttcan_change_mtu,
};

int register_mttcan_dev(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	int err;

	mttcan_pm_runtime_enable(priv);

	dev->netdev_ops = &mttcan_netdev_ops;

	err = register_candev(dev);
	if (err)
		mttcan_pm_runtime_disable(priv);
	else
		devm_can_led_init(dev);

	return err;
}

void unregister_mttcan_dev(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	unregister_candev(dev);
	mttcan_pm_runtime_disable(priv);
}

void free_mttcan_dev(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	netif_napi_del(&priv->napi);
	free_candev(dev);
}

static int mttcan_power_down(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);

	if (!(dev->flags & IFF_UP))
		return 0;

	if (ttcan_power_down(priv->ttcan, 1))
		return -ETIMEDOUT;

	mttcan_pm_runtime_put_sync(priv);

	return 0;
}

static int mttcan_probe(struct platform_device *pdev)
{
	int ret = 0;
	void __iomem *regs = NULL, *xregs = NULL;
	void __iomem *car_addr = NULL, *mram_addr = NULL;
	struct net_device *dev;
	struct mttcan_priv *priv;
	const struct platform_device_id *id;
	struct pinctrl *pinctrl;
	struct resource *mesg_ram, *ctrl_res;
	struct resource *car_res, *ext_res;
	int irq;
	struct clk *clk;
#ifdef CONFIG_OF
	const struct of_device_id *match;

	if (pdev->dev.of_node) {
		match = of_match_device(mttcan_of_table, &pdev->dev);
		if (!match) {
			dev_err(&pdev->dev, "Failed to find matching dt id\n");
			dev_err(&pdev->dev, "probe failed\n");
			return -EINVAL;
		}
		id = match->data;
	} else
#endif
		id = platform_get_device_id(pdev);


	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev, "failed to configure pins from driver\n");

	/* get the appropriate clk */
	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		/* ret = -ENODEV;
		goto exit; */
	}

	/* get the platform data */
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "IRQ not defined\n");
		goto exit_free_clk;
	}

	dev = alloc_mttcan_dev();
	if (!dev) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "CAN device allocation failed\n");
		goto exit_free_clk;
	}

	priv = netdev_priv(dev);

	/* mem0 Controller Register Space
	 * mem1 Controller Extra Registers Space
	 * mem2 Controller Messege RAM Space
	 */
	ctrl_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ext_res  = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mesg_ram = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!ctrl_res || !ext_res || !mesg_ram) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "Resource allocation failed\n");
		goto exit_free_clk;
	}

	if (car_present) {
		car_res  = platform_get_resource(pdev, IORESOURCE_MEM, 3);
		if (!car_res) {
			ret = -ENODEV;
			dev_err(&pdev->dev, "Resource allocation failed\n");
			goto exit_free_clk;
		}
		pr_debug("%s: car %llx\n", __func__, car_res->start);

		car_addr = devm_ioremap_resource(&pdev->dev, car_res);
		if (!car_addr) {
			dev_err(&pdev->dev, "failed to map can port\n");
			ret = -ENOMEM;
			goto exit_free_clk;
		}
		pr_debug("remapped : car_addr %p\n", car_addr);
	}

	regs = devm_ioremap_resource(&pdev->dev, ctrl_res);
	xregs = devm_ioremap_resource(&pdev->dev, ext_res);
	mram_addr = devm_ioremap_resource(&pdev->dev, mesg_ram);

	pr_debug("%s: res %llx ext_res %llx  mem %llx\n", __func__,
			ctrl_res->start, ext_res->start,
			mesg_ram->start);
	pr_debug("remapped : regs %p xregs %p mram_addr %p\n",
			regs, xregs, mram_addr);

	if (!mram_addr || !xregs || !regs) {
		dev_err(&pdev->dev, "failed to map can port\n");
		ret = -ENOMEM;
		goto exit_free_clk;
	}

	/* allocate the mttcan device */
	if (pdev->dev.of_node)
		priv->instance = of_alias_get_id(pdev->dev.of_node, "mttcan");
	else
		priv->instance = pdev->id;

	dev->irq = irq;
	priv->device = &pdev->dev;
	priv->can.clock.freq = bs_clock;/* clk_get_rate(clk) */;
	priv->clk = clk;
	priv->ttcan =
	    devm_kzalloc(priv->device, sizeof(struct ttcan_controller),
			 GFP_KERNEL);
	if (!priv->ttcan) {
		dev_err(priv->device,
			"cannot allocate memory for ttcan_controller\n");
		goto exit_free_device;
	}
	memset(priv->ttcan, 0, sizeof(struct ttcan_controller));
	priv->ttcan->base = regs;
	priv->ttcan->xbase = xregs;
	if (car_present)
		priv->ttcan->cbase = car_addr;
	priv->ttcan->id = priv->instance;
	priv->ttcan->mram_sa.virt_base = mram_addr;
	priv->ttcan->mram_sa.base = mesg_ram->start;
	INIT_LIST_HEAD(&priv->ttcan->rx_q0);
	INIT_LIST_HEAD(&priv->ttcan->rx_q1);
	INIT_LIST_HEAD(&priv->ttcan->rx_b);
	INIT_LIST_HEAD(&priv->ttcan->tx_evt);

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	if (mttcan_poll) {
		dev_info(&pdev->dev, "Polling Mode enabled\n");
		INIT_DELAYED_WORK(&priv->can_work, mttcan_work);
	}
	ret = register_mttcan_dev(dev);
	if (ret) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
			KBUILD_MODNAME, ret);
		goto exit_free_device;
	}

	ret = mttcan_hw_init(priv);
	if (ret)
		goto exit_free_device;

	ret = mttcan_create_sys_files(&dev->dev);
	if (ret)
		goto exit_free_device;

	dev_info(&dev->dev, "%s device registered (regs=%p, irq=%d)\n",
		 KBUILD_MODNAME, priv->ttcan->base, dev->irq);
	return 0;

exit_free_device:
	platform_set_drvdata(pdev, NULL);
	free_mttcan_dev(dev);
exit_free_clk:
	clk_put(clk);
	dev_err(&pdev->dev, "probe failed\n");
	return ret;
}

static int mttcan_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct mttcan_priv *priv = netdev_priv(dev);
	if (mttcan_poll)
		cancel_delayed_work_sync(&priv->can_work);

	dev_info(&dev->dev, "%s\n", __func__);

	mttcan_delete_sys_files(&dev->dev);
	mttcan_power_down(dev);

	mttcan_stop(dev);

	unregister_mttcan_dev(dev);

	platform_set_drvdata(pdev, NULL);

	free_mttcan_dev(dev);

	clk_put(priv->clk);
	return 0;
}

#ifdef CONFIG_PM
static int mttcan_power_up(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);

	mttcan_pm_runtime_get_sync(priv);

	return ttcan_power_down(priv->ttcan, 0);
}

static int mttcan_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret;
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct mttcan_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
	}
	ret = mttcan_power_down(ndev);
	if (ret) {
		netdev_err(ndev, "failed to enter power down mode\n");
		return ret;
	}

	mttcan_stop(dev);

	priv->can.state = CAN_STATE_SLEEPING;
	return 0;
}

static int mttcan_resume(struct platform_device *pdev)
{
	int ret;
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct mttcan_priv *priv = netdev_priv(ndev);

	if (!(dev->flags & IFF_UP))
		netdev_err(ndev, "Still in power down mode\n");
		return -EBUSY;
	}

	ret = mttcan_power_up(ndev);
	if (ret)
		return ret;
	mttcan_start(dev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	if (netif_running(ndev)) {
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}
	return 0;
}
#endif

static struct platform_driver mttcan_plat_driver = {
	.driver = {
		   .name = KBUILD_MODNAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = of_match_ptr(mttcan_of_table),
#endif
		   },
	.probe = mttcan_probe,
	.remove = mttcan_remove,
#ifdef CONFIG_PM
	.suspend = mttcan_suspend,
	.resume = mttcan_resume,
#endif
#ifdef CONFIG_OF
	.id_table = mttcan_id_table,
#endif
};

#ifndef CONFIG_OF
static struct resource res_can0[] = {
	[0] = {
		.start = NV_ADDRESS_MAP_CAN1_0_INTR_ID,
		.end = NV_ADDRESS_MAP_CAN1_0_INTR_ID,
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		/* Map CAN Controller */
		.start = NV_ADDRESS_MAP_CAN1_BASE,
		.end = NV_ADDRESS_MAP_CAN1_BASE + 1020,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		/* Map CAN Glue */
		.start = NV_ADDRESS_MAP_CAN1_BASE + CAN_GLUE_ADDR,
		.end = NV_ADDRESS_MAP_CAN1_BASE + CAN_GLUE_ADDR + 46,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		/* Map the size of messege RAM */
		.start = NV_ADDRESS_MAP_CAN1_BASE + CAN_MES_RAM_BASE_ADDR,
		.end = NV_ADDRESS_MAP_CAN1_BASE + CAN_MES_RAM_BASE_ADDR + 4092,
		.flags = IORESOURCE_MEM,
	},
	[4] = {
		/* CAR registers for CAN1 */
		.start = NV_ADDRESS_MAP_CAR_BASE + CAR_CAN1_SET_OFF,
		.end = NV_ADDRESS_MAP_CAR_BASE + CAR_CAN1_SET_OFF + 8,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource res_can1[] = {
	[0] = {
		.start = NV_ADDRESS_MAP_CAN2_0_INTR_ID,
		.end = NV_ADDRESS_MAP_CAN2_0_INTR_ID,
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		/* Map CAN Controller */
		.start = NV_ADDRESS_MAP_CAN2_BASE,
		.end = NV_ADDRESS_MAP_CAN2_BASE + 1020,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		/* Map GLUE Controller */
		.start = NV_ADDRESS_MAP_CAN2_BASE + CAN_GLUE_ADDR,
		.end = NV_ADDRESS_MAP_CAN2_BASE + CAN_GLUE_ADDR + 46,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		/* Map the size of messege RAM */
		.start = NV_ADDRESS_MAP_CAN2_BASE + CAN_MES_RAM_BASE_ADDR,
		.end = NV_ADDRESS_MAP_CAN2_BASE + CAN_MES_RAM_BASE_ADDR + 4092,
		.flags = IORESOURCE_MEM,
	},
	[4] = {
		/* CAR registers for CAN2 */
		.start = NV_ADDRESS_MAP_CAR_BASE + CAR_CAN2_SET_OFF,
		.end = NV_ADDRESS_MAP_CAR_BASE + CAR_CAN2_SET_OFF + 8,
		.flags = IORESOURCE_MEM,
	}
};

struct platform_device *mttcan_pdev0 = NULL, *mttcan_pdev1 = NULL;

static int __init mttcan_init(void)
{
	int err;

	mttcan_pdev0  = platform_device_alloc("mttcan", 0);
	if (mttcan_pdev0) {
		err = platform_device_add_resources(mttcan_pdev0, res_can0,
					ARRAY_SIZE(res_can0));
		if (err == 0)
			err = platform_device_add(mttcan_pdev0);
	} else {
		err = -ENOMEM;
	}
	if (err) {
		platform_device_put(mttcan_pdev0);
		return err;
	}
	mttcan_pdev1  = platform_device_alloc("mttcan", 1);
	if (mttcan_pdev1) {
		err = platform_device_add_resources(mttcan_pdev1, res_can1,
					ARRAY_SIZE(res_can1));
		if (err == 0)
			err = platform_device_add(mttcan_pdev1);
	} else {
		err = -ENOMEM;
	}
	if (err) {
		platform_device_put(mttcan_pdev1);
		platform_device_unregister(mttcan_pdev0);
		return err;
	}

	pr_info("mttcan plat_device registered\n");
	err = platform_driver_register(&mttcan_plat_driver);
	if (err) {
		platform_device_unregister(mttcan_pdev0);
		platform_device_unregister(mttcan_pdev1);
	}
	return err;
}

static void __exit mttcan_exit(void)
{
	platform_device_unregister(mttcan_pdev0);
	platform_device_unregister(mttcan_pdev1);
	platform_driver_unregister(&mttcan_plat_driver);
}
module_init(mttcan_init);
module_exit(mttcan_exit);
#else
module_platform_driver(mttcan_plat_driver);
#endif
MODULE_AUTHOR("Manoj Chourasia <mchourasia@nvidia.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Platform CAN bus driver for Bosch M_TTCAN controller");
