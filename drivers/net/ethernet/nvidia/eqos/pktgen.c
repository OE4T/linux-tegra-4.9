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
/*!@file: DWC_ETH_QOS_drv.c
 * @brief: Driver functions.
 */

#include "yheader.h"
#include "yapphdr.h"
#include "pktgen.h"

extern ULONG dwc_eth_qos_base_addr;
#include "yregacc.h"

static INT DWC_ETH_QOS_GStatus;

static int DWC_ETH_QOS_FRAME_PATTERN_CH[8] = {
	0x11111111,
	0x22222222,
	0x33333333,
	0x44444444,
	0x55555555,
	0x66666666,
	0x77777777,
	0x88888888,
};

static int DWC_ETH_QOS_frame_hdrs[8][4] = {
	/* for channel 0 : Non tagged header
	 * Dst addr : 0x00:0x0D:0x56:0x73:0xD0:0xF3
	 * Src addr : 0x00:0x55:0x7B:0xB5:0x7D:0xF7
	 * Type/Length : 0x800
	 * */
	{0x73560D00, 0x5500F3D0, 0xF77DB57B, 0x00000008},

	/* for channel 1 : VLAN tagged header with priority 1
	 * Dst addr : 0x00:0x0D:0x56:0x73:0xD0:0xF3
	 * Src addr : 0x00:0x55:0x7B:0xB5:0x7D:0xF7
	 * Type/Length : 0x8100
	 * */
	{0x73560D00, 0x5500F3D0, 0xF77DB57B, 0x64200081},

	/* for channel 2 : VLAN tagged header with priority 2
	 * Dst addr : 0x00:0x0D:0x56:0x73:0xD0:0xF3
	 * Src addr : 0x00:0x55:0x7B:0xB5:0x7D:0xF7
	 * Type/Length : 0x8100
	 * */
	{0x73560D00, 0x5500F3D0, 0xF77DB57B, 0x64400081},

	/* for channel 3 : VLAN tagged header with priority 3
	 * Dst addr : 0x00:0x0D:0x56:0x73:0xD0:0xF3
	 * Src addr : 0x00:0x55:0x7B:0xB5:0x7D:0xF7
	 * Type/Length : 0x8100
	 * */
	{0x73560D00, 0x5500F3D0, 0xF77DB57B, 0x64600081},

	/* for channel 4 : VLAN tagged header with priority 4
	 * Dst addr : 0x00:0x0D:0x56:0x73:0xD0:0xF3
	 * Src addr : 0x00:0x55:0x7B:0xB5:0x7D:0xF7
	 * Type/Length : 0x8100
	 * */
	{0x73560D00, 0x5500F3D0, 0xF77DB57B, 0x64800081},

	/* for channel 5 : VLAN tagged header with priority 5
	 * Dst addr : 0x00:0x0D:0x56:0x73:0xD0:0xF3
	 * Src addr : 0x00:0x55:0x7B:0xB5:0x7D:0xF7
	 * Type/Length : 0x8100
	 * */
	{0x73560D00, 0x5500F3D0, 0xF77DB57B, 0x64A00081},

	/* for channel 6 : VLAN tagged header with priority 6
	 * Dst addr : 0x00:0x0D:0x56:0x73:0xD0:0xF3
	 * Src addr : 0x00:0x55:0x7B:0xB5:0x7D:0xF7
	 * Type/Length : 0x8100
	 * */
	{0x73560D00, 0x5500F3D0, 0xF77DB57B, 0x64C00081},

	/* for channel 7 : VLAN tagged header with priority 7
	 * Dst addr : 0x00:0x0D:0x56:0x73:0xD0:0xF3
	 * Src addr : 0x00:0x55:0x7B:0xB5:0x7D:0xF7
	 * Type/Length : 0x8100
	 * */
	{0x73560D00, 0x5500F3D0, 0xF77DB57B, 0x64E00081},
};


/*!
* \brief API to receiv the data from device.
*
* \details This function reads as many packets are possible from
* device, reinitialize the descriptor buffer pointers and other
* control bits such that device owns the descriptor. It also does
* some housekeeping work to manage the descriptors.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] qInx - DMA channel/queue no. to be checked for packet.
*
* \return integer
*
* \retval number of packets received.
*/
static int DWC_ETH_QOS_poll_pg_sq(struct DWC_ETH_QOS_prv_data *pdata,
				unsigned int qInx)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data =
		GET_RX_WRAPPER_DESC(qInx);
	struct net_device *dev = pdata->dev;
	struct s_RX_NORMAL_DESC *RX_NORMAL_DESC = NULL;
	struct DWC_ETH_QOS_rx_buffer *buffer = NULL;
	unsigned int varrx_error_counters = 0;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input =
		&(pdata->pg->pg_ch_input[qInx]);
	int received = 0;
	struct hw_if_struct *hw_if = &pdata->hw_if;
	struct sk_buff *skb = NULL;

	DBGPR_PG("-->DWC_ETH_QOS_poll_pg_sq: qInx = %u\n", qInx);

	while (1) {
		DBGPR_PG("cur_rx = %d\n", desc_data->cur_rx);
		RX_NORMAL_DESC = GET_RX_DESC_PTR(qInx, desc_data->cur_rx);
		buffer = GET_RX_BUF_PTR(qInx, desc_data->cur_rx);

		/* reset rx packets attributes */
		memset(&(pdata->rx_pkt_features), 0,
		       sizeof(struct s_rx_pkt_features));
		/* reset error counters */
		memset(&(pdata->rx_error_counters), 0,
		       sizeof(struct s_rx_error_counters));
		buffer->len = 0;

		hw_if->dev_read(pdata, qInx);

		varrx_error_counters = pdata->rx_error_counters.rx_errors;
		/* no more data to read */
		if ((buffer->len == 0x0) && (varrx_error_counters == 0x0))
			break;

		/* assign it to new skb */
		skb = buffer->skb;
		/* good packet */
		if (varrx_error_counters == 0) {
			dev->last_rx = jiffies;
			/* update the statistics */
			dev->stats.rx_packets++;
			dev->stats.rx_bytes += buffer->len;
			pg_ch_input->ch_FramecountRx++;

			dma_sync_single_for_cpu(&pdata->pdev->dev, buffer->dma,
					DWC_ETH_QOS_PG_FRAME_SIZE, DMA_FROM_DEVICE);
#ifdef DWC_ETH_QOS_ENABLE_RX_PKT_DUMP
			if (0 == (pg_ch_input->ch_FramecountRx % 500)) {
				//print_pkt(skb, buffer->len, 0, (desc_data->cur_rx));
				dump_rx_desc(qInx, RX_NORMAL_DESC, desc_data->cur_rx);
			}
#endif
		} else {
			DBGPR_PG("Error in received pkt, hence failed to pass it to upper layer\n");
			dev->stats.rx_errors++;
			DWC_ETH_QOS_update_rx_errors(dev, varrx_error_counters);
		}

		/* Reassign same buffer pointer and give ownership to DMA */
		//memset(buffer->skb->data, 0, buffer->len);
		/* update buffer 1 address pointer */
		RX_NORMAL_DESC_RDES0_Ml_Wr(RX_NORMAL_DESC->RDES0, buffer->dma);
		/* set to zero */
		RX_NORMAL_DESC_RDES1_Ml_Wr(RX_NORMAL_DESC->RDES1, 0);
		/* set buffer 2 address pointer to zero */
		RX_NORMAL_DESC_RDES2_Ml_Wr(RX_NORMAL_DESC->RDES2, 0);
		/* set control bits - OWN, INTE and BUF1V */
		RX_NORMAL_DESC_RDES3_Ml_Wr(RX_NORMAL_DESC->RDES3, (0xc1000000));

		/* update the Rx Tail Pointer Register with address of
		 * descriptors from which data is read */
		DMA_RDTP_RPDR_RgWr(qInx, GET_RX_DESC_DMA_ADDR(qInx, desc_data->cur_rx));

		received++;
		INCR_RX_DESC_INDEX(desc_data->cur_rx, 1);
	}

	DBGPR_PG("<--DWC_ETH_QOS_poll_pg_sq: received = %d\n", received);

	return received;
}

/*!
* \brief API to receiv the data from device.
* 
* \details This function is called from ISR upon receive interrupt.
* This function will call other helper function to read the packets
* from all DMA channel.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void
*/
static void DWC_ETH_QOS_poll_pg(struct DWC_ETH_QOS_prv_data *pdata)
{
	unsigned int qInx;
	int received = 0;

	DBGPR_PG("-->DWC_ETH_QOS_poll_pg\n");

	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++) {
		received = DWC_ETH_QOS_poll_pg_sq(pdata, qInx);
		DBGPR_PG("Received %d packets from RX queue %u\n",
			received, qInx);
	}

	/* Enable all ch RX interrupt */
	DWC_ETH_QOS_enable_all_ch_rx_interrpt(pdata);

	DBGPR_PG("<--DWC_ETH_QOS_poll_pg\n");
}


/*!
* \brief API to update the tx status.
*
* \details This function is called from ISR upon transmit complete
* interrupt to check the status of packet transmitted by device. It
* also updates the private data structure fields.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] qInx - DMA channel number.
*
* \return void
*/
static void DWC_ETH_QOS_tx_interrupt_pg(struct DWC_ETH_QOS_prv_data *pdata,
				     UINT qInx)
{
	struct net_device *dev = pdata->dev;
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data =
	    GET_TX_WRAPPER_DESC(qInx);
	struct s_TX_NORMAL_DESC *txptr = NULL;
	struct DWC_ETH_QOS_tx_buffer *buffer = NULL;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int err_incremented;
	UINT reg_tail_ptr = 0, var_tail_ptr = 0, tail_ptr = 0, head_ptr = 0;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input =
		&(pdata->pg->pg_ch_input[qInx]);

	DBGPR_PG("-->DWC_ETH_QOS_tx_interrupt_pg: dirty_tx = %d, qInx = %u\n",
			desc_data->dirty_tx, qInx);

	while (1) {
		txptr = GET_TX_DESC_PTR(qInx, desc_data->dirty_tx);
		buffer = GET_TX_BUF_PTR(qInx, desc_data->dirty_tx);

		if (!hw_if->tx_complete(txptr))
			break;

#ifdef DWC_ETH_QOS_ENABLE_TX_DESC_DUMP
		dump_tx_desc(pdata, desc_data->dirty_tx, desc_data->dirty_tx,
			     0, qInx);
#endif

		dev->stats.tx_bytes += buffer->len;

		/* update the tx error if any by looking at last segment
		 * for NORMAL descriptors
		 * */
		if ((hw_if->get_tx_desc_ls(txptr)) && !(hw_if->get_tx_desc_ctxt(txptr))) {
			err_incremented = 0;
			if (txptr->TDES3 & 0x8000)
				dump_tx_desc(pdata, desc_data->dirty_tx, desc_data->dirty_tx, 0, qInx);

			if (hw_if->tx_window_error) {
				if (hw_if->tx_window_error(txptr)) {
					err_incremented = 1;
					dev->stats.tx_window_errors++;
				}
			}
			if (hw_if->tx_aborted_error) {
				if (hw_if->tx_aborted_error(txptr)) {
					err_incremented = 1;
					dev->stats.tx_aborted_errors++;
					if (hw_if->tx_handle_aborted_error)
						hw_if->tx_handle_aborted_error(txptr);
				}
			}
			if (hw_if->tx_carrier_lost_error) {
				if (hw_if->tx_carrier_lost_error(txptr)) {
					err_incremented = 1;
					dev->stats.tx_carrier_errors++;
				}
			}
			if (hw_if->tx_fifo_underrun) {
				if (hw_if->tx_fifo_underrun(txptr)) {
					dev->stats.tx_fifo_errors++;
					if (hw_if->tx_update_fifo_threshold)
						hw_if->tx_update_fifo_threshold(txptr);
				}
			}
			if (hw_if->tx_get_collision_count)
				dev->stats.collisions +=
				    hw_if->tx_get_collision_count(txptr);

			if (err_incremented == 1) {
				dev->stats.tx_errors++;
				printk(KERN_ALERT "Error in transmission of packet\n");
			}
			dev->stats.tx_packets++;
			pg_ch_input->ch_FramecountTx++;
		}
		else {
			if (!hw_if->get_tx_desc_ls(txptr))
				printk(KERN_ALERT "LS not set for %d\n", desc_data->dirty_tx);
			if (hw_if->get_tx_desc_ctxt(txptr))
				printk(KERN_ALERT "Context desc in %d\n", desc_data->dirty_tx);
		}

		/* reset the descriptor so that driver/host can reuse it */
		hw_if->tx_desc_reset(desc_data->dirty_tx, pdata, qInx);

		if ((pdata->prepare_pg_packet == Y_TRUE)) {
			if (pg_ch_input->ch_debug_mode == 0) {
				/* reassign the same buffer pointers and make it ready for transmission */
				DWC_ETH_QOS_prepare_desc(pdata, txptr, buffer, desc_data->dirty_tx, qInx);
				/* issue a poll command to Tx DMA by writing address
				 * of next immediate free descriptor */
				tail_ptr = GET_TX_DESC_DMA_ADDR(qInx, desc_data->dirty_tx);
				DMA_TDTP_TPDR_RgWr(qInx, tail_ptr);
			} else {
				/* DEBUG ON */
				if (pg_ch_input->ch_FramecountTx <= pg_ch_input->ch_desc_prepare) {
					/* reassign the same buffer pointers and make it ready for transmission */
					DWC_ETH_QOS_prepare_desc(pdata, txptr, buffer, desc_data->dirty_tx, qInx);
					/* issue a poll command to Tx DMA by writing address
					* of next immediate free descriptor */
					tail_ptr = GET_TX_DESC_DMA_ADDR(qInx, desc_data->dirty_tx);
					DMA_TDTP_TPDR_RgWr(qInx, tail_ptr);
				}
			}
		}

		INCR_TX_DESC_INDEX(desc_data->dirty_tx, 1);
	}

	/* debug print */
	if (pg_ch_input->interrupt_prints && !(pg_ch_input->tx_interrupts % 1) &&
			(pg_ch_input->ch_FramecountTx <= 8 /*|| pg_ch_input->ch_FramecountTx >= TX_DESC_CNT*/)) {
		var_tail_ptr = GET_TX_DESC_DMA_ADDR(qInx, desc_data->dirty_tx);
		DMA_TDTP_TPDR_RgRd(qInx, reg_tail_ptr);
		DMA_CHTDR_RgRd(qInx, head_ptr);
		printk(KERN_ALERT
				"%d] Tail @ run     [%3llu]%#x,r%#x\n"
				"    Head @ run     [%3llu]%#x\n"
				"    dirty_tx @ run  %d\n"
				"    ch_FramecountTx %lu\n\n",
				qInx,
				GET_TX_DESC_IDX(qInx, var_tail_ptr), var_tail_ptr, reg_tail_ptr,
				GET_TX_DESC_IDX(qInx, head_ptr), head_ptr,
				desc_data->dirty_tx,
				pg_ch_input->ch_FramecountTx);
		pg_ch_input->interrupt_prints--;
	}
	pg_ch_input->tx_interrupts++;

	DBGPR_PG("<--DWC_ETH_QOS_tx_interrupt_pg\n");
}


static void DWC_ETH_QOS_save_abs_count(struct DWC_ETH_QOS_prv_data *pdata,
		UINT qInx)
{
	struct DWC_ETH_QOS_PGStruct *pg_struct = pdata->pg;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	ULONG varMTL_QESR = 0;

	MTL_QESR_RgRd(qInx, varMTL_QESR);
	if (varMTL_QESR & 0x1000000) {
		switch (pg_ch_input[qInx].ch_operating_mode) {
		case eDWC_ETH_QOS_QDCB:
			pg_ch_input[qInx].ch_AvgBits += (varMTL_QESR & 0xffffff);
			break;

		case eDWC_ETH_QOS_QAVB:
			// TODO: calculation pending
			pg_ch_input[qInx].ch_AvgBits += (varMTL_QESR & 0xffffff);
			break;
		}
		pg_ch_input[qInx].ch_AvgBits_interrupt_count++;
	}

	return;
}

/*!
* \brief Interrupt Service Routine
*
* \details This function is invoked by Linux when there is any interrupt
* from GMAC. This function will check for all interrupts and call
* appropriate functions to service them.
*
* \param[in] irq         - interrupt number for particular device
* \param[in] device_id   - pointer to device structure
*
* \return integer
*
* \retval IRQ_HANDLED if inerrupt is handled successfully and
*         IRQ_NONE if interrupt is not ours.
*/
irqreturn_t DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS_pg(int irq, void *device_id)
{
	ULONG varDMA_ISR;
	ULONG varDMA_SR;
	ULONG varDMA_IER;
	struct DWC_ETH_QOS_prv_data *pdata =
	    (struct DWC_ETH_QOS_prv_data *)device_id;
	UINT qInx;

	DBGPR_PG("-->DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS_pg\n");

	DMA_ISR_RgRd(varDMA_ISR);
	if (varDMA_ISR == 0x0)
		return IRQ_NONE;

	DBGPR_PG("DMA_ISR = %#lx\n", varDMA_ISR);

	/* Handle DMA interrupts */
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		DMA_SR_RgRd(qInx, varDMA_SR);
		DMA_IER_RgRd(qInx, varDMA_IER);

		/* handle only those DMA interrupts which are enabled */
		varDMA_SR = (varDMA_SR & varDMA_IER);

		DBGPR_PG("DMA_SR[%d] = %#lx\n", qInx, varDMA_SR);

		if (varDMA_SR == 0)
			continue;

		if (GET_VALUE(varDMA_SR, DMA_SR_TI_LPOS, DMA_SR_TI_HPOS) & 1) {
			DWC_ETH_QOS_tx_interrupt_pg(pdata, qInx);
			DMA_SR_TI_UdfWr(qInx, 1);
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_TPS_LPOS, DMA_SR_TPS_HPOS) & 1) {
			DWC_ETH_QOS_GStatus = -E_DMA_SR_TPS;
			printk(KERN_ALERT "%d] TPS Interrupt\n", qInx);
			DMA_SR_TPS_UdfWr(qInx, 1);
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_TBU_LPOS, DMA_SR_TBU_HPOS) & 1) {
			DWC_ETH_QOS_GStatus = -E_DMA_SR_TBU;
			//if (pdata->prepare_pg_packet == Y_FALSE) {
				printk(KERN_ALERT "%d] TBU Interrupt\n", qInx);
				pdata->pg->channel_running[qInx] = Y_FALSE;
			//}
			DMA_SR_TBU_UdfWr(qInx, 1);
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_RI_LPOS, DMA_SR_RI_HPOS) & 1) {
			DWC_ETH_QOS_disable_all_ch_rx_interrpt(pdata);
			DWC_ETH_QOS_poll_pg(pdata);
			DMA_SR_RI_UdfWr(qInx, 1);
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_RBU_LPOS, DMA_SR_RBU_HPOS) & 1) {
			DWC_ETH_QOS_GStatus = -E_DMA_SR_RBU;
			DMA_SR_RBU_UdfWr(qInx, 1);
			printk(KERN_ALERT "RBU Interrupt\n");
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_RPS_LPOS, DMA_SR_RPS_HPOS) & 1) {
			DWC_ETH_QOS_GStatus = -E_DMA_SR_RPS;
			DMA_SR_RPS_UdfWr(qInx, 1);
			printk(KERN_ALERT "RPS Interrupt\n");
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_RWT_LPOS, DMA_SR_RWT_HPOS) & 1) {
			DWC_ETH_QOS_GStatus = S_DMA_SR_RWT;
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_FBE_LPOS, DMA_SR_FBE_HPOS) & 1) {
			DWC_ETH_QOS_GStatus = -E_DMA_SR_FBE;
			DMA_SR_FBE_UdfWr(qInx, 1);
			DBGPR_PG("FATAL bus error interrupt\n");
		}
	}

	/* MTL Interrupt handler */
	if (GET_VALUE(varDMA_ISR, DMA_ISR_MTLIS_LPOS, DMA_ISR_MTLIS_HPOS) & 1) {
		for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
			/* ABS Interrupt handler */
			DWC_ETH_QOS_save_abs_count(pdata, qInx);
		}
	}

	DBGPR_PG("<--DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS_pg\n");

	return IRQ_HANDLED;

}


/*!
 * \brief api to initialize default values.
 *
 * \details This function is used to initialize differnet parameters to
 * default values which are common parameters between Tx and Rx path.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */
void DWC_ETH_QOS_default_confs(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_tx_wrapper_descriptor *tx_desc_data = NULL;
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data = NULL;
	UINT qInx;

	pdata->incr_incrx = DWC_ETH_QOS_INCR_ENABLE;
	pdata->flow_ctrl = 0;
	pdata->oldflow_ctrl = 0;

	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		tx_desc_data = GET_TX_WRAPPER_DESC(qInx);

		tx_desc_data->tx_threshold_val = DWC_ETH_QOS_TX_THRESHOLD_32;
		tx_desc_data->tsf_on = DWC_ETH_QOS_TSF_ENABLE;
		tx_desc_data->osf_on = DWC_ETH_QOS_OSF_ENABLE;
		tx_desc_data->tx_pbl = DWC_ETH_QOS_PBL_16;
	}

	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++) {
		rx_desc_data = GET_RX_WRAPPER_DESC(qInx);

		rx_desc_data->rx_threshold_val = DWC_ETH_QOS_RX_THRESHOLD_64;
		rx_desc_data->rsf_on = DWC_ETH_QOS_RSF_DISABLE;
		rx_desc_data->rx_pbl = DWC_ETH_QOS_PBL_16;
	}
}

/*!
* \brief API to prepare descriptor.
*
* \details This function is called by other driver function to prepare
* the transmit descriptor.
*
* \param[in] pdata   - pointer to private data structure
* \param[in] txptr   - pointer to transmit descriptor structure
* \param[in] buffer  - pointer to transmit buffer structure
* \param[in] i       - descriptor index
* \param[in] qInx    - DMA channel number
*
* \return void
*/
static void DWC_ETH_QOS_prepare_desc(struct DWC_ETH_QOS_prv_data *pdata,
				struct s_TX_NORMAL_DESC *txptr,
				struct DWC_ETH_QOS_tx_buffer *buffer,
				int i,
				unsigned int qInx)
{
	//DBGPR_PG("-->DWC_ETH_QOS_prepare_desc\n");

	/* update packet address */
	TX_NORMAL_DESC_TDES0_Ml_Wr(txptr->TDES0, buffer->dma);
	/* update the packet length */
	TX_NORMAL_DESC_TDES2_HL_B1L_Mlf_Wr(txptr->TDES2, buffer->len);
	/* update the frame length */
	TX_NORMAL_DESC_TDES3_FL_Mlf_Wr(txptr->TDES3, buffer->len);
	/* set Interrupt on Completion for last descriptor */
	TX_NORMAL_DESC_TDES2_IC_Mlf_Wr(txptr->TDES2, 0x1);
	/* Mark it as First Descriptor */
	TX_NORMAL_DESC_TDES3_FD_Mlf_Wr(txptr->TDES3, 0x1);
	/* Mark it as LAST descriptor */
	TX_NORMAL_DESC_TDES3_LD_Mlf_Wr(txptr->TDES3, 0x1);
	/* Disable CRC and Pad Insertion */
	TX_NORMAL_DESC_TDES3_CPC_Mlf_Wr(txptr->TDES3, 0);
	/* Mark it as NORMAL descriptor */
	TX_NORMAL_DESC_TDES3_CTXT_Mlf_Wr(txptr->TDES3, 0);
	/* set slot number */
	TX_NORMAL_DESC_TDES3_SLOTNUM_TCPHDRLEN_Mlf_Wr(txptr->TDES3, buffer->slot_number);
	/* set OWN bit at end to avoid race condition */
	TX_NORMAL_DESC_TDES3_OWN_Mlf_Wr(txptr->TDES3, 0x1);

#ifdef DWC_ETH_QOS_ENABLE_TX_DESC_DUMP
	dump_tx_desc(pdata, i, i, 1, qInx);
#endif

	//DBGPR_PG("<--DWC_ETH_QOS_prepare_desc\n");
}


/*!
* \brief API to prepare tx descriptor.
*
* \details This function will prepare all DMA channel Tx descriptor
* in advance before starting the Tx DMA engine.
*
* \param[in] pdata   - pointer to private data structure
*
* \return void
*/
static void DWC_ETH_QOS_prepare_tx_packets_for_pg_test(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data = NULL;
	struct DWC_ETH_QOS_PGStruct *pg_struct = pdata->pg;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	UINT head_ptr = 0, tail_ptr = 0, desc_ring_ptr = 0, i, qInx, frame_size;
	unsigned short ch_tx_desc_slot_no_start = 0;
	unsigned short ch_tx_desc_slot_no_skip = 0;
	unsigned int tx_pkt_cnt = 0;
	int desc_idx = 0;

	DBGPR_PG("-->DWC_ETH_QOS_prepare_tx_packets_for_pg_test\n");

	/* Descriptor memory allocation for transmission */
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		struct s_TX_NORMAL_DESC *txptr = NULL;
		struct DWC_ETH_QOS_tx_buffer *buffer = NULL;
		unsigned int *skb_data = NULL;
		unsigned int avtype = DWC_ETH_QOS_AVTYPE;
		int payload_cnt = 0;
		desc_data = GET_TX_WRAPPER_DESC(qInx);

		/* for channel 0 no slot number checking */
		if (qInx > 0) {
			ch_tx_desc_slot_no_start = pg_ch_input[qInx].ch_tx_desc_slot_no_start;
			ch_tx_desc_slot_no_skip = pg_ch_input[qInx].ch_tx_desc_slot_no_skip;
		}

		if (pg_ch_input[qInx].ch_debug_mode == 1) {
			DMA_CHTDR_RgRd(qInx, head_ptr);
			desc_idx = GET_TX_DESC_IDX(qInx, head_ptr);
			if (pg_ch_input[qInx].ch_max_tx_frame_cnt >= TX_DESC_CNT) {
				tx_pkt_cnt = TX_DESC_CNT;
				pg_ch_input[qInx].ch_desc_prepare =
					(pg_ch_input[qInx].ch_max_tx_frame_cnt - TX_DESC_CNT);
				pg_ch_input[qInx].ch_desc_prepare++;
			}
			else {
				pg_ch_input[qInx].ch_desc_prepare = 0;
				tx_pkt_cnt = (pg_ch_input[qInx].ch_max_tx_frame_cnt % TX_DESC_CNT);
				/* DUT stops when tail & head become equal,
					 prepare one extra packet for transmit */
				tx_pkt_cnt++;
			}
		} else {
			tx_pkt_cnt = TX_DESC_CNT;
			desc_idx = 0;
		}

		for (i = 0; i < tx_pkt_cnt; i++) {
			txptr = GET_TX_DESC_PTR(qInx, desc_idx);
			buffer = GET_TX_BUF_PTR(qInx, desc_idx);
			skb_data = (unsigned int *)buffer->skb->data;

			if (!skb_data) {
				printk(KERN_ALERT "ERROR: No SKB Allocated for channel\n");
				break;
			}

			/* populate tx pg data */
			frame_size = pg_ch_input[qInx].ch_frame_size;
			if (qInx == 0) {
				/* Add Ethernet header */
				*skb_data++ = DWC_ETH_QOS_frame_hdrs[qInx][0];
				*skb_data++ = DWC_ETH_QOS_frame_hdrs[qInx][1];
				*skb_data++ = DWC_ETH_QOS_frame_hdrs[qInx][2];
				*skb_data++ = DWC_ETH_QOS_frame_hdrs[qInx][3];
				/* Add payload */
				for (payload_cnt = 0; payload_cnt < frame_size;) {
					*skb_data++ = DWC_ETH_QOS_FRAME_PATTERN_CH[qInx];
					/* increment by 4 since we are writing
					 * one dword at a time */
					payload_cnt += 4;
				}
				buffer->len = frame_size;
			} else {
				/* Add Ethernet header */
				*skb_data++ = DWC_ETH_QOS_frame_hdrs[qInx][0];
				*skb_data++ = DWC_ETH_QOS_frame_hdrs[qInx][1];
				*skb_data++ = DWC_ETH_QOS_frame_hdrs[qInx][2];
				*skb_data++ = DWC_ETH_QOS_frame_hdrs[qInx][3];

				if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB) {
					avtype = ((avtype << 8) | (avtype >> 8)) & 0x0000FFFF;
					*skb_data++ = (DWC_ETH_QOS_FRAME_PATTERN_CH[qInx] << 16) | avtype;
					payload_cnt = 4;
				} else {
					payload_cnt = 0;
				}

				/* Add payload */
				while (payload_cnt < frame_size) {
					*skb_data++ = DWC_ETH_QOS_FRAME_PATTERN_CH[qInx];
					/* increment by 4 since we are writing
					 * one dword at a time */
					payload_cnt += 4;
				}
				buffer->len = frame_size;
			}

			dma_sync_single_for_device(&pdata->pdev->dev, buffer->dma,
					DWC_ETH_QOS_PG_FRAME_SIZE, DMA_TO_DEVICE);

			if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB &&
					pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QDCB) {
				/* slot number preparation */
				if (ch_tx_desc_slot_no_skip != 0) {
					if ((desc_idx % ch_tx_desc_slot_no_skip) == 0) {
						ch_tx_desc_slot_no_start++;
						/* max value of slot number is 15 */
						ch_tx_desc_slot_no_start &= 0xF;
					}
				}
				buffer->slot_number = ch_tx_desc_slot_no_start;
			}
			else {
				buffer->slot_number = 0;
			}

			/* prepare descriptor for transmission */
			DWC_ETH_QOS_prepare_desc(pdata, txptr, buffer, desc_idx, qInx);
#ifdef DWC_ETH_QOS_ENABLE_TX_PKT_DUMP
			if (desc_idx < 1)
				print_pkt(buffer->skb, buffer->len, 1, desc_idx);
#endif
			INCR_TX_DESC_INDEX(desc_idx, 1);
		}

		if (pg_ch_input[qInx].ch_debug_mode == 1) {
			DECR_TX_DESC_INDEX(desc_idx);
			tail_ptr = GET_TX_DESC_DMA_ADDR(qInx, desc_idx);
			printk(KERN_ALERT "ch_desc_prepare     %d\n",
					pg_ch_input[qInx].ch_desc_prepare);
		}
		else {
			/* Updating tail pointer to one descriptor behind head pointer */
			DMA_CHTDR_RgRd(qInx, head_ptr);
			DMA_TDLAR_RgRd(qInx, desc_ring_ptr);
			if ((head_ptr == 0) || (desc_ring_ptr == head_ptr)) {
				tail_ptr = GET_TX_DESC_DMA_ADDR(qInx, tx_pkt_cnt - 1);
			}
			else {
				tail_ptr = (head_ptr - sizeof(struct s_TX_NORMAL_DESC));
			}
		}
		DMA_TDTP_TPDR_RgWr(qInx, tail_ptr);
		desc_data->dirty_tx = GET_TX_DESC_IDX(qInx, head_ptr);

		printk(KERN_ALERT
				"%d] Tail @ init    [%3llu]%#x\n"
				"    Head @ init    [%3llu]%#x\n"
				"    dirty_tx @ init %d\n\n",
				qInx, GET_TX_DESC_IDX(qInx, tail_ptr), tail_ptr,
				GET_TX_DESC_IDX(qInx, head_ptr), head_ptr,
				desc_data->dirty_tx);
	}

	DBGPR_PG("<--DWC_ETH_QOS_prepare_tx_packets_for_pg_test\n");
}


/*!
* \brief API to configure HW for PG test.
*
* \details This function will configures all the TX DMA channels for
* packet generator module.
*
* \param[in] pdata   - pointer to private data structure
*
* \return void
*/
static void DWC_ETH_QOS_prepare_hw_for_pg_test(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_PGStruct *pg_struct = pdata->pg;
	unsigned int qInx;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct timespec now;
	UINT varMAC_TCR = 0;

	DBGPR_PG("-->DWC_ETH_QOS_prepare_hw_for_pg_test\n");

	hw_if->set_tx_rx_prio_policy(pg_struct->ch_tx_rx_arb_scheme);
	hw_if->set_tx_rx_prio(pg_struct->ch_use_tx_high_prio);
	hw_if->set_tx_rx_prio_ratio(pg_struct->ch_tx_rx_prio_ratio);
	hw_if->set_dma_tx_arb_algorithm(pg_struct->dma_tx_arb_algo);
	hw_if->set_dcb_algorithm(pg_struct->queue_dcb_algorithm);
    hw_if->config_mac_loopback_mode(pg_struct->mac_lb_mode);

	/* Timer programming */
	hw_if->config_sub_second_increment(DWC_ETH_QOS_SYSCLOCK);

	varMAC_TCR = (MAC_TCR_TSENA | MAC_TCR_TSCTRLSSR);
	hw_if->config_hw_time_stamping(varMAC_TCR);

	getnstimeofday(&now);
	hw_if->init_systime(now.tv_sec, now.tv_nsec);

	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		hw_if->set_tx_queue_operating_mode(qInx, pg_ch_input[qInx].ch_operating_mode);

		/* DCB parameters */
		hw_if->set_dcb_queue_weight(qInx, pg_ch_input[qInx].ch_queue_weight);
		hw_if->set_ch_arb_weights(qInx, pg_ch_input[qInx].ch_arb_weight);

		/* Slot parameters */
		hw_if->config_slot_num_check(qInx, pg_ch_input[qInx].ch_EnableSlotCheck);
		hw_if->config_slot_interrupt(qInx, pg_ch_input[qInx].ch_EnableSlotCheck);
		hw_if->set_slot_count(qInx, pg_ch_input[qInx].ch_SlotCount);
		hw_if->config_advance_slot_num_check(qInx, pg_ch_input[qInx].ch_EnableAdvSlotCheck);

		/* AVB parameters */
		if ((pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB) && (qInx > 0)) {
			hw_if->set_avb_algorithm(qInx, pg_ch_input[qInx].ch_avb_algorithm);
			if (pg_ch_input[qInx].ch_avb_algorithm == eDWC_ETH_QOS_AVB_CBS) {
				hw_if->config_credit_control(qInx, pg_ch_input[qInx].ch_CreditControl);
				hw_if->config_send_slope(qInx, pg_ch_input[qInx].ch_SendSlope);
				hw_if->config_idle_slope(qInx, pg_ch_input[qInx].ch_IdleSlope);
				hw_if->config_high_credit(qInx, pg_ch_input[qInx].ch_HiCredit);
				hw_if->config_low_credit(qInx, pg_ch_input[qInx].ch_LoCredit);
			}
		}
	}

	DBGPR_PG("<--DWC_ETH_QOS_prepare_hw_for_pg_test\n");
}


/*!
* \brief timer function to stop tx DMA engine.
*
* \details This function will stop all TX DMA engine.
*
* \param[in] data   - pointer to private data structure
*
* \return void
*/
static void DWC_ETH_QOS_pg_timer_fun(unsigned long data)
{
	struct DWC_ETH_QOS_prv_data *pdata = (struct DWC_ETH_QOS_prv_data *)data;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data = NULL;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input = NULL;
	UINT qInx, head_ptr = 0, tail_ptr = 0, dma_dsr = 0;

	printk(KERN_ALERT "-->DWC_ETH_QOS_pg_timer_fun\n");

	/* allow device to transmit pending prepared packets */
	pdata->prepare_pg_packet = Y_FALSE;

	mdelay(500);

	DMA_DSR0_RgRd(dma_dsr);
	printk(KERN_ALERT "DMA Channel state: %#x\n", dma_dsr);
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		pg_ch_input = &(pdata->pg->pg_ch_input[qInx]);
		desc_data = GET_TX_WRAPPER_DESC(qInx);
		/* disable only that channel for which ENABLE bit is set */
		if ((pdata->pg->ch_SelMask) & (1 << qInx)) {
			hw_if->stop_dma_tx(qInx);
		}
		DMA_TDTP_TPDR_RgRd(qInx, tail_ptr);
		DMA_CHTDR_RgRd(qInx, head_ptr);
		printk(KERN_ALERT
				"%d] Tail @ stop     [%3llu]%#x\n"
				"    Head @ stop     [%3llu]%#x\n"
				"    dirty_tx @ stop  %d\n"
				"    ch_FramecountTx  %lu\n"
				"    Total interrupts %d\n\n",
				qInx, GET_TX_DESC_IDX(qInx, tail_ptr), tail_ptr,
				GET_TX_DESC_IDX(qInx, head_ptr), head_ptr,
				desc_data->dirty_tx,
				pg_ch_input->ch_FramecountTx,
				pg_ch_input->tx_interrupts);
	}
	// TODO: add code to disable slot interrupt except channel 0

	pdata->run_test = Y_FALSE;

	printk(KERN_ALERT "PG Experiment is completed ....\n"\
		"You can retrieve the Report\n");
	DBGPR_PG("PG Experiment is completed ....\n"\
		"You can retrieve the Report\n");

	printk(KERN_ALERT "<--DWC_ETH_QOS_pg_timer_fun\n");
}


/*!
* \brief APT to start tx DMA engine.
*
* \details This function will prepare all TX DMA engine in advance
* for data transfer and start all DMA TX engine.
*
* \param[in] data   - pointer to private data structure
*
* \return void
*/
static void DWC_ETH_QOS_pg_run(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned int qInx;

	DBGPR_PG("-->DWC_ETH_QOS_pg_run\n");

	DWC_ETH_QOS_prepare_tx_packets_for_pg_test(pdata);

	pdata->run_test = Y_TRUE;
	pdata->prepare_pg_packet = Y_TRUE;

	/* start pg timer before enabling the DMA's */
	add_timer(&pdata->pg_timer);

	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		/* enable only that channel for which ENABLE bit is set */
		if ((pdata->pg->ch_SelMask) & (1 << qInx)) {
			pdata->pg->channel_running[qInx] = Y_TRUE;
			hw_if->start_dma_tx(qInx);
		}
		else {
			pdata->pg->channel_running[qInx] = Y_FALSE;
		}
	}

	DBGPR_PG("<--DWC_ETH_QOS_pg_run\n");
}


/*!
* \brief APT to setup krnel timer function.
*
* \details This function setup a kernel timer function for packet
* generator test.
*
* \param[in] data   - pointer to private data structure
*
* \return void
*/
static void DWC_ETH_QOS_setup_timer(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR_PG("-->DWC_ETH_QOS_setup_timer\n");

	init_timer(&pdata->pg_timer);
	pdata->pg_timer.expires = (HZ * (pdata->pg->DurationOfExp) + jiffies);
	pdata->pg_timer.data = (unsigned long)pdata;
	pdata->pg_timer.function = DWC_ETH_QOS_pg_timer_fun;

	printk(KERN_ALERT "Test will expire at %d\n\n", (INT)pdata->pg_timer.expires);

	DBGPR_PG("<--DWC_ETH_QOS_setup_timer\n");
}


/*!
* \brief APT to start PG test.
*
* \details This function will start the packet generator test. It calls
* other driver functions which steup kernel timer, prepares descriptor
* and start the Tx DMA.
*
* \param[in] pdata   - pointer to private data structure
*
* \return void
*/
static void DWC_ETH_QOS_pg_run_test(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR_PG("-->DWC_ETH_QOS_pg_run_test\n");

	printk(KERN_ALERT "PG Experiment has started ....\n");
	DWC_ETH_QOS_setup_timer(pdata);
	DWC_ETH_QOS_pg_run(pdata);

	DBGPR_PG("<--DWC_ETH_QOS_pg_run_test\n");
}


/*!
* \brief APT to display PG data structure.
*
* \details This function will display the packet generator data structure
* for debugging purpose. The display shows what are the configurations are
* enabled in the device.
*
* \param[in] pdata   - pointer to private data structure
*
* \return void
*/
static void DWC_ETH_QOS_print_pg_struct(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_PGStruct *pg_struct = pdata->pg;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	unsigned int qInx;
	char *space = "                               ", *strptr = NULL;
	unsigned char display_avb_params = 0, display_dcb_params = 0;

	DBGPR_PG("-->DWC_ETH_QOS_print_pg_struct\n");

	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB) {
			display_avb_params = 1;
			break;
		}
	}
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QDCB) {
			display_dcb_params = 1;
			break;
		}
	}

	printk(KERN_ALERT "DurationOfExp                       = %#02x\n",
		pg_struct->DurationOfExp);
	printk(KERN_ALERT "ch_SelMask                          = %#02x\n",
		pg_struct->ch_SelMask);
	printk(KERN_ALERT "ch_tx_rx_arb_scheme                 = %#02x\n",
		pg_struct->ch_tx_rx_arb_scheme);
	printk(KERN_ALERT "Ch_use_tx_high_prio                 = %#02x\n",
		pg_struct->ch_use_tx_high_prio);
	printk(KERN_ALERT "ch_tx_rx_prio_ratio                 = %#02x\n",
		pg_struct->ch_tx_rx_prio_ratio);
	printk(KERN_ALERT "dma_tx_arb_algo                     = %#02x\n",
			pg_struct->dma_tx_arb_algo);
    printk(KERN_ALERT "mac_lb_mode                         = %#02x\n",
			pg_struct->mac_lb_mode);

	if (display_dcb_params) {
		switch (pg_struct->queue_dcb_algorithm) {
			case eDWC_ETH_QOS_DCB_WRR:
				strptr = "WRR (Weighted Round Robin)";
				break;
			case eDWC_ETH_QOS_DCB_WFQ:
				strptr = "WFQ (Weighted Fair Queuing)";
				break;
			case eDWC_ETH_QOS_DCB_DWRR:
				strptr = "DWRR (Deficit Weighted Round Robin)";
				break;
			case eDWC_ETH_QOS_DCB_SP:
				strptr = "SP (Strict Priority)";
				break;
		}
		printk(KERN_ALERT "queue_dcb_algorithm                 = %s\n",
				strptr);
	}

	printk(KERN_ALERT "PrioTagForAV (not used)             = %#02x\n",
		pg_struct->PrioTagForAV);

	printk(KERN_ALERT "ch_operating_mode\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		switch (pg_ch_input[qInx].ch_operating_mode) {
		case eDWC_ETH_QOS_QDISABLED:
			printk(KERN_ALERT "%s[Ch%d] = Disabled\n", space, qInx);
			break;
		case eDWC_ETH_QOS_QAVB:
			printk(KERN_ALERT "%s[Ch%d] = AVB\n", space, qInx);
			break;
		case eDWC_ETH_QOS_QDCB:
			printk(KERN_ALERT "%s[Ch%d] = DCB\n", space, qInx);
			break;
		case eDWC_ETH_QOS_QGENERIC:
			printk(KERN_ALERT "%s[Ch%d] = Generic\n", space, qInx);
			break;
		}
	}

	printk(KERN_ALERT "ch_arb_weight [DMA]\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %#01x\n", space, qInx,
				pg_ch_input[qInx].ch_arb_weight);
	}
	printk(KERN_ALERT "ch_bw\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %d%%\n", space, qInx,
			pg_ch_input[qInx].ch_bw);
	}

	printk(KERN_ALERT "ch_queue_weight\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %d\n", space, qInx,
			pg_ch_input[qInx].ch_queue_weight);
	}

	printk(KERN_ALERT "ch_frame_size\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %d\n", space, qInx,
			pg_ch_input[qInx].ch_frame_size);
	}

	printk(KERN_ALERT "ch_EnableSlotCheck\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %s\n", space, qInx,
			pg_ch_input[qInx].ch_EnableSlotCheck ? "YES" : "NO");
	}

	printk(KERN_ALERT "ch_EnableAdvSlotCheck\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %s\n", space, qInx,
			pg_ch_input[qInx].ch_EnableAdvSlotCheck ? "YES" : "NO");
	}

	printk(KERN_ALERT "ch_SlotCount\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %d\n", space, qInx,
			pg_ch_input[qInx].ch_SlotCount);
	}

	printk(KERN_ALERT "ch_tx_desc_slot_no_start\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %#01x\n", space, qInx,
			pg_ch_input[qInx].ch_tx_desc_slot_no_start);
	}

	printk(KERN_ALERT "ch_tx_desc_slot_no_skip\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %#01x\n", space, qInx,
			pg_ch_input[qInx].ch_tx_desc_slot_no_skip);
	}

	printk(KERN_ALERT "ch_AvgBits\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %lu\n", space, qInx,
			pg_ch_input[qInx].ch_AvgBits);
	}

	printk(KERN_ALERT "ch_AvgBits_interrupt_count\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %lu\n", space, qInx,
			pg_ch_input[qInx].ch_AvgBits_interrupt_count);
	}

	if (display_avb_params) {
		printk(KERN_ALERT "ch_avb_algorithm\n");
		for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
			if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %s\n", space, qInx,
						(pg_ch_input[qInx].ch_avb_algorithm == eDWC_ETH_QOS_AVB_SP ?
						 "Strict Priority": "Credit Based Shaper"));
		}

		printk(KERN_ALERT "ch_CreditControl\n");
		for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
			if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %s\n", space, qInx,
						pg_ch_input[qInx].ch_CreditControl ? "YES" : "NO");
		}

		printk(KERN_ALERT "ch_SendSlope\n");
		for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
			if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %#08x\n", space, qInx,
						pg_ch_input[qInx].ch_SendSlope);
		}

		printk(KERN_ALERT "ch_IdleSlope\n");
		for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
			if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %#08x\n", space, qInx,
						pg_ch_input[qInx].ch_IdleSlope);
		}

		printk(KERN_ALERT "ch_HiCredit\n");
		for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
			if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %#08x\n", space, qInx,
						pg_ch_input[qInx].ch_HiCredit);
		}

		printk(KERN_ALERT "ch_LoCredit\n");
		for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
			if (pg_ch_input[qInx].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %#08x\n", space, qInx,
						pg_ch_input[qInx].ch_LoCredit);
		}
	}

	printk(KERN_ALERT "ch_FramecountTx\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %lu\n", space, qInx,
			pg_ch_input[qInx].ch_FramecountTx);
	}

	printk(KERN_ALERT "ch_FramecountRx\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[Ch%d] = %lu\n", space, qInx,
			pg_ch_input[qInx].ch_FramecountRx);
	}

	printk(KERN_ALERT "Debug mode\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[CH%d]  %s\n",
			space, qInx, (pg_ch_input[qInx].ch_debug_mode ? "YES" : "NO"));
	}

	printk(KERN_ALERT "Maximum Tx packet count\n");
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		printk(KERN_ALERT "%s[CH%d]  %d\n",
			space, qInx, pg_ch_input[qInx].ch_max_tx_frame_cnt);
	}

	DBGPR_PG("<--DWC_ETH_QOS_print_pg_struct\n");
}


/*!
* \brief APT to sync the kernel and user data structure.
*
* \details This function is invoked by IOCTL function when the user
* issues a command to synchronize the user space data structure to
* kernel space data structure.
*
* \param[in] pdata   - pointer to private data structure
* \param[in] req   - pointer to IOCTL specific data structure
*
* \return void
*/
static void DWC_ETH_QOS_pg_set_config(struct DWC_ETH_QOS_prv_data *pdata,
					struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_PGStruct l_pg_struct;
	struct DWC_ETH_QOS_PGStruct *user_pg_struct =
		(struct DWC_ETH_QOS_PGStruct *)req->ptr;
	struct DWC_ETH_QOS_PGStruct *pg_struct = pdata->pg;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input =
		pg_struct->pg_ch_input;
	unsigned int qInx;

	DBGPR_PG("-->DWC_ETH_QOS_pg_set_config\n");

	/* First, copy contents of user info in a local structure */
	if(copy_from_user(&l_pg_struct, user_pg_struct,
				sizeof(struct DWC_ETH_QOS_PGStruct)))
		printk(KERN_ALERT "Failed to fetch PG Struct info from user\n");

	/* Second, copy required members into kernel structure */
	copy_PGStruct_members(pg_struct, &l_pg_struct);
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		copy_pg_ch_input_members(&(pg_ch_input[qInx]), &(l_pg_struct.pg_ch_input[qInx]));
	}

	DWC_ETH_QOS_print_pg_struct(pdata);

	DBGPR_PG("<--DWC_ETH_QOS_pg_set_config\n");
}


/*!
* \brief APT to sync the kernel and user data structure.
*
* \details This function is invoked by IOCTL function when the user
* issue a command to get the packet generator data. This function
* will copy kernel data structure into user data structure.
*
* \param[in] pdata   - pointer to private data structure
* \param[in] req   - pointer to IOCTL specific data structure
*
* \return void
*/
static void DWC_ETH_QOS_pg_get_result(struct DWC_ETH_QOS_prv_data *pdata,
					struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_PGStruct l_pg_struct;
	struct DWC_ETH_QOS_PGStruct *user_pg_struct =
		(struct DWC_ETH_QOS_PGStruct *)req->ptr;
	struct DWC_ETH_QOS_PGStruct *pg_struct = pdata->pg;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input =
		pg_struct->pg_ch_input;
	unsigned int qInx;

	DBGPR_PG("-->DWC_ETH_QOS_pg_get_result\n");
	copy_PGStruct_members(&l_pg_struct, pg_struct);
    l_pg_struct.speed_100M_1G = pdata->speed;/* Update the speed information */
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		copy_pg_ch_input_members(&(l_pg_struct.pg_ch_input[qInx]), &(pg_ch_input[qInx]));
	}
	if (copy_to_user(user_pg_struct, &l_pg_struct, sizeof(struct DWC_ETH_QOS_PGStruct)))
		printk(KERN_ALERT "Failed to send PG Struct info to user\n");

	DBGPR_PG("<--DWC_ETH_QOS_pg_get_result\n");
}

/*!
* \brief IOCTL function to handle user request.
*
* \details This function is invoked by IOCTL function when the user
* issues a command to get the packet generator data. This function
* will copy kernel data structure into user data structure.
*
* \param[in] pdata   - pointer to private data structure
* \param[in] ptr   - pointer to IOCTL specific data structure
*
* \return zero on success and -ve number on failure.
*/
int DWC_ETH_QOS_handle_pg_ioctl(struct DWC_ETH_QOS_prv_data *pdata,
				void *ptr)
{
	struct ifr_data_struct *req = ptr;
	int ret = 0;

	DBGPR_PG("-->DWC_ETH_QOS_handle_pg_ioctl\n");

	switch (req->flags) {
	case DWC_ETH_QOS_PG_SET_CONFIG:
		DWC_ETH_QOS_pg_set_config(pdata, req);
		break;
	case DWC_ETH_QOS_PG_CONFIG_HW:
		DWC_ETH_QOS_prepare_hw_for_pg_test(pdata);
		DWC_ETH_QOS_prepare_tx_packets_for_pg_test(pdata);
		printk(KERN_ALERT "\nCONFIGURING THE HW FOR PG ....\n");
		break;
	case DWC_ETH_QOS_PG_RUN_TEST:
		DWC_ETH_QOS_pg_run_test(pdata);
		printk(KERN_ALERT "PG RUN TEST STARTED ....\n");
		break;
	case DWC_ETH_QOS_PG_GET_RESULT:
		DWC_ETH_QOS_pg_get_result(pdata, req);
		break;
	case DWC_ETH_QOS_PG_TEST_DONE:
		req->test_done = pdata->run_test;
		break;
	default:
		printk(KERN_ALERT "Wrong Parameter for PG TEST\n");
		ret = -EINVAL;
	}

	DBGPR_PG("<--DWC_ETH_QOS_handle_pg_ioctl\n");

	return ret;
}


/*!
* \brief API to allocate local data structure.
*
* \details This function is used to allocate local data structure
* for handling packet generator module.
*
* \param[in] pdata   - pointer to private data structure
*
* \return zero on success and -ve number on failure.
*/
int DWC_ETH_QOS_alloc_pg(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR_PG("-->DWC_ETH_QOS_alloc_pg\n");

	pdata->pg = kzalloc(sizeof(struct DWC_ETH_QOS_PGStruct), GFP_KERNEL);
	if (pdata->pg == NULL) {
		printk(KERN_ALERT "%s:Unable to alloc pg structure\n", DEV_NAME);
		return -ENOMEM;
	}

	DBGPR_PG("<--DWC_ETH_QOS_alloc_pg\n");

	return 0;
}


/*!
* \brief API to free local data structure.
*
* \details This function is used to free the local data structure
* which is allocated for handling packet generator module.
*
* \param[in] pdata   - pointer to private data structure
*
* \return void
*/
void DWC_ETH_QOS_free_pg(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR_PG("-->DWC_ETH_QOS_free_pg\n");

	//kfree(pdata->pg->pg_ch_input);
	kfree(pdata->pg);

	DBGPR_PG("<--DWC_ETH_QOS_free_pg\n");
}


/*!
* \brief API to allocate tx buffer.
*
* \details This function is used to allocate tx buffer for data
* transmission.
*
* \param[in] pdata   - pointer to private data structure
* \param[in] buffer  - pointer to tx buffer data structure
* \param[in] gfp     - type of memory allocation.
*
* \return zero on success and -ve number on failure.
*/
int DWC_ETH_QOS_alloc_tx_buf_pg(struct DWC_ETH_QOS_prv_data *pdata,
				struct DWC_ETH_QOS_tx_buffer *buffer,
				gfp_t gfp)
{
	struct sk_buff *skb = NULL;

	//DBGPR_PG("-->DWC_ETH_QOS_alloc_tx_buf_pg\n");

	skb = dev_alloc_skb(DWC_ETH_QOS_PG_FRAME_SIZE);
	if (skb == NULL) {
		printk(KERN_ALERT "Failed to allocate tx skb\n");
		return -ENOMEM;
	}
	buffer->skb = skb;
	buffer->len = DWC_ETH_QOS_PG_FRAME_SIZE;
	buffer->dma = dma_map_single(&pdata->pdev->dev, skb->data,
				     DWC_ETH_QOS_PG_FRAME_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(&pdata->pdev->dev, buffer->dma))
		printk(KERN_ALERT "failed to do the TX dma map\n");

	buffer->buf1_mapped_as_page = Y_FALSE;

	//DBGPR_PG("<--DWC_ETH_QOS_alloc_tx_buf_pg\n");

	return 0;
}


/*!
* \brief API to allocate rx buffer.
*
* \details This function is used to allocate rx buffer for data
* receive by device.
*
* \param[in] pdata   - pointer to private data structure
* \param[in] buffer  - pointer to rx buffer data structure
* \param[in] gfp     - type of memory allocation.
*
* \return zero on success and -ve number on failure.
*/
int DWC_ETH_QOS_alloc_rx_buf_pg(struct DWC_ETH_QOS_prv_data *pdata,
				struct DWC_ETH_QOS_rx_buffer *buffer,
				gfp_t gfp)
{
	struct sk_buff *skb = NULL;

	//DBGPR_PG("-->DWC_ETH_QOS_alloc_rx_buf_pg\n");

	skb = dev_alloc_skb(DWC_ETH_QOS_PG_FRAME_SIZE);
	if (skb == NULL) {
		printk(KERN_ALERT "Failed to allocate tx skb\n");
		return -ENOMEM;
	}
	buffer->skb = skb;
	buffer->len = DWC_ETH_QOS_PG_FRAME_SIZE;
	buffer->dma = dma_map_single(&pdata->pdev->dev, skb->data,
				     DWC_ETH_QOS_PG_FRAME_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(&pdata->pdev->dev, buffer->dma))
		printk(KERN_ALERT "failed to do the RX dma map\n");

	buffer->mapped_as_page = Y_FALSE;
	wmb();

	//DBGPR_PG("<--DWC_ETH_QOS_alloc_rx_buf_pg\n");

	return 0;
}

