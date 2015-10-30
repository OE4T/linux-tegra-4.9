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
/*!@file: eqos_drv.c
 * @brief: Driver functions.
 */

#include "yheader.h"
#include "yapphdr.h"
#include "pktgen.h"

extern ULONG eqos_base_addr;
#include "yregacc.h"

static INT eqos_status;

static int EQOS_FRAME_PATTERN_CH[8] = {
	0x11111111,
	0x22222222,
	0x33333333,
	0x44444444,
	0x55555555,
	0x66666666,
	0x77777777,
	0x88888888,
};

static int eqos_frame_hdrs[8][4] = {
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
* \param[in] qinx - DMA channel/queue no. to be checked for packet.
*
* \return integer
*
* \retval number of packets received.
*/
static int eqos_poll_pg_sq(struct eqos_prv_data *pdata,
				unsigned int qinx)
{
	struct eqos_rx_wrapper_descriptor *desc_data =
		GET_RX_WRAPPER_DESC(qinx);
	struct net_device *dev = pdata->dev;
	struct s_rx_normal_desc *rx_normal_desc = NULL;
	struct eqos_rx_buffer *buffer = NULL;
	unsigned int varrx_error_counters = 0;
	struct eqos_pg_ch_input *pg_ch_input =
		&(pdata->pg->pg_ch_input[qinx]);
	int received = 0;
	struct hw_if_struct *hw_if = &pdata->hw_if;
	struct sk_buff *skb = NULL;

	DBGPR_PG("-->eqos_poll_pg_sq: qinx = %u\n", qinx);

	while (1) {
		DBGPR_PG("cur_rx = %d\n", desc_data->cur_rx);
		rx_normal_desc = GET_RX_DESC_PTR(qinx, desc_data->cur_rx);
		buffer = GET_RX_BUF_PTR(qinx, desc_data->cur_rx);

		/* reset rx packets attributes */
		memset(&(pdata->rx_pkt_features), 0,
		       sizeof(struct s_rx_pkt_features));
		/* reset error counters */
		memset(&(pdata->rx_error_counters), 0,
		       sizeof(struct s_rx_error_counters));
		buffer->len = 0;

		hw_if->dev_read(pdata, qinx);

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
			pg_ch_input->ch_framecount_rx++;

			dma_sync_single_for_cpu(&pdata->pdev->dev, buffer->dma,
					EQOS_PG_FRAME_SIZE, DMA_FROM_DEVICE);
#ifdef EQOS_ENABLE_RX_PKT_DUMP
			if (0 == (pg_ch_input->ch_framecount_rx % 500)) {
				//print_pkt(skb, buffer->len, 0, (desc_data->cur_rx));
				dump_rx_desc(qinx, rx_normal_desc, desc_data->cur_rx);
			}
#endif
		} else {
			DBGPR_PG("Error in received pkt, hence failed to pass it to upper layer\n");
			dev->stats.rx_errors++;
			eqos_update_rx_errors(dev, varrx_error_counters);
		}

		/* Reassign same buffer pointer and give ownership to DMA */
		//memset(buffer->skb->data, 0, buffer->len);
		/* update buffer 1 address pointer */
		RX_NORMAL_DESC_RDES0_WR(rx_normal_desc->rdes0, buffer->dma);
		/* set to zero */
		RX_NORMAL_DESC_RDES1_WR(rx_normal_desc->rdes1, 0);
		/* set buffer 2 address pointer to zero */
		RX_NORMAL_DESC_RDES2_WR(rx_normal_desc->rdes2, 0);
		/* set control bits - OWN, INTE and BUF1V */
		RX_NORMAL_DESC_RDES3_WR(rx_normal_desc->rdes3, (0xc1000000));

		/* update the Rx Tail Pointer Register with address of
		 * descriptors from which data is read */
		DMA_RDTP_RPDR_WR(qinx, GET_RX_DESC_DMA_ADDR(qinx, desc_data->cur_rx));

		received++;
		INCR_RX_DESC_INDEX(desc_data->cur_rx, 1);
	}

	DBGPR_PG("<--eqos_poll_pg_sq: received = %d\n", received);

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
static void eqos_poll_pg(struct eqos_prv_data *pdata)
{
	unsigned int qinx;
	int received = 0;

	DBGPR_PG("-->eqos_poll_pg\n");

	for (qinx = 0; qinx < EQOS_RX_QUEUE_CNT; qinx++) {
		received = eqos_poll_pg_sq(pdata, qinx);
		DBGPR_PG("Received %d packets from RX queue %u\n",
			received, qinx);
	}

	/* Enable all ch RX interrupt */
	eqos_enable_all_ch_rx_interrpt(pdata);

	DBGPR_PG("<--eqos_poll_pg\n");
}


/*!
* \brief API to update the tx status.
*
* \details This function is called from ISR upon transmit complete
* interrupt to check the status of packet transmitted by device. It
* also updates the private data structure fields.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] qinx - DMA channel number.
*
* \return void
*/
static void eqos_tx_interrupt_pg(struct eqos_prv_data *pdata,
				     UINT qinx)
{
	struct net_device *dev = pdata->dev;
	struct eqos_tx_wrapper_descriptor *desc_data =
	    GET_TX_WRAPPER_DESC(qinx);
	struct s_tx_normal_desc *txptr = NULL;
	struct eqos_tx_buffer *buffer = NULL;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int err_incremented;
	UINT reg_tail_ptr = 0, var_tail_ptr = 0, tail_ptr = 0, head_ptr = 0;
	struct eqos_pg_ch_input *pg_ch_input =
		&(pdata->pg->pg_ch_input[qinx]);

	DBGPR_PG("-->eqos_tx_interrupt_pg: dirty_tx = %d, qinx = %u\n",
			desc_data->dirty_tx, qinx);

	while (1) {
		txptr = GET_TX_DESC_PTR(qinx, desc_data->dirty_tx);
		buffer = GET_TX_BUF_PTR(qinx, desc_data->dirty_tx);

		if (!hw_if->tx_complete(txptr))
			break;

#ifdef EQOS_ENABLE_TX_DESC_DUMP
		dump_tx_desc(pdata, desc_data->dirty_tx, desc_data->dirty_tx,
			     0, qinx);
#endif

		dev->stats.tx_bytes += buffer->len;

		/* update the tx error if any by looking at last segment
		 * for NORMAL descriptors
		 * */
		if ((hw_if->get_tx_desc_ls(txptr)) && !(hw_if->get_tx_desc_ctxt(txptr))) {
			err_incremented = 0;
			if (txptr->tdes3 & 0x8000)
				dump_tx_desc(pdata, desc_data->dirty_tx, desc_data->dirty_tx, 0, qinx);

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
			pg_ch_input->ch_framecount_tx++;
		}
		else {
			if (!hw_if->get_tx_desc_ls(txptr))
				printk(KERN_ALERT "LS not set for %d\n", desc_data->dirty_tx);
			if (hw_if->get_tx_desc_ctxt(txptr))
				printk(KERN_ALERT "Context desc in %d\n", desc_data->dirty_tx);
		}

		/* reset the descriptor so that driver/host can reuse it */
		hw_if->tx_desc_reset(desc_data->dirty_tx, pdata, qinx);

		if ((pdata->prepare_pg_packet == Y_TRUE)) {
			if (pg_ch_input->ch_debug_mode == 0) {
				/* reassign the same buffer pointers and make it ready for transmission */
				eqos_prepare_desc(pdata, txptr, buffer, desc_data->dirty_tx, qinx);
				/* issue a poll command to Tx DMA by writing address
				 * of next immediate free descriptor */
				tail_ptr = GET_TX_DESC_DMA_ADDR(qinx, desc_data->dirty_tx);
				DMA_TDTP_TPDR_WR(qinx, tail_ptr);
			} else {
				/* DEBUG ON */
				if (pg_ch_input->ch_framecount_tx <= pg_ch_input->ch_desc_prepare) {
					/* reassign the same buffer pointers and make it ready for transmission */
					eqos_prepare_desc(pdata, txptr, buffer, desc_data->dirty_tx, qinx);
					/* issue a poll command to Tx DMA by writing address
					* of next immediate free descriptor */
					tail_ptr = GET_TX_DESC_DMA_ADDR(qinx, desc_data->dirty_tx);
					DMA_TDTP_TPDR_WR(qinx, tail_ptr);
				}
			}
		}

		INCR_TX_DESC_INDEX(desc_data->dirty_tx, 1);
	}

	/* debug print */
	if (pg_ch_input->interrupt_prints && !(pg_ch_input->tx_interrupts % 1) &&
			(pg_ch_input->ch_framecount_tx <= 8 /*|| pg_ch_input->ch_framecount_tx >= TX_DESC_CNT*/)) {
		var_tail_ptr = GET_TX_DESC_DMA_ADDR(qinx, desc_data->dirty_tx);
		DMA_TDTP_TPDR_RD(qinx, reg_tail_ptr);
		DMA_CHTDR_RD(qinx, head_ptr);
		printk(KERN_ALERT
				"%d] Tail @ run     [%3llu]%#x,r%#x\n"
				"    Head @ run     [%3llu]%#x\n"
				"    dirty_tx @ run  %d\n"
				"    ch_framecount_tx %lu\n\n",
				qinx,
				GET_TX_DESC_IDX(qinx, var_tail_ptr), var_tail_ptr, reg_tail_ptr,
				GET_TX_DESC_IDX(qinx, head_ptr), head_ptr,
				desc_data->dirty_tx,
				pg_ch_input->ch_framecount_tx);
		pg_ch_input->interrupt_prints--;
	}
	pg_ch_input->tx_interrupts++;

	DBGPR_PG("<--eqos_tx_interrupt_pg\n");
}


static void eqos_save_abs_count(struct eqos_prv_data *pdata,
		UINT qinx)
{
	struct eqos_pg_struct *pg_struct = pdata->pg;
	struct eqos_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	ULONG mtl_qesr = 0;

	MTL_QESR_RD(qinx, mtl_qesr);
	if (mtl_qesr & 0x1000000) {
		switch (pg_ch_input[qinx].ch_operating_mode) {
		case EQOS_QDCB:
			pg_ch_input[qinx].ch_avg_bits += (mtl_qesr & 0xffffff);
			break;

		case EQOS_QAVB:
			// TODO: calculation pending
			pg_ch_input[qinx].ch_avg_bits += (mtl_qesr & 0xffffff);
			break;
		}
		pg_ch_input[qinx].ch_avg_bits_interrupt_count++;
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
irqreturn_t eqos_pg_isr(int irq, void *device_id)
{
	ULONG dma_isr;
	ULONG dma_sr;
	ULONG dma_ier;
	struct eqos_prv_data *pdata =
	    (struct eqos_prv_data *)device_id;
	UINT qinx;

	DBGPR_PG("-->eqos_pg_isr\n");

	DMA_ISR_RD(dma_isr);
	if (dma_isr == 0x0)
		return IRQ_NONE;

	DBGPR_PG("DMA_ISR = %#lx\n", dma_isr);

	/* Handle DMA interrupts */
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		DMA_SR_RD(qinx, dma_sr);
		DMA_IER_RD(qinx, dma_ier);

		/* handle only those DMA interrupts which are enabled */
		dma_sr = (dma_sr & dma_ier);

		DBGPR_PG("DMA_SR[%d] = %#lx\n", qinx, dma_sr);

		if (dma_sr == 0)
			continue;

		if (GET_VALUE(dma_sr, DMA_SR_TI_LPOS, DMA_SR_TI_HPOS) & 1) {
			eqos_tx_interrupt_pg(pdata, qinx);
			DMA_SR_TI_WR(qinx, 1);
		}
		if (GET_VALUE(dma_sr, DMA_SR_TPS_LPOS, DMA_SR_TPS_HPOS) & 1) {
			eqos_status = -E_DMA_SR_TPS;
			printk(KERN_ALERT "%d] TPS Interrupt\n", qinx);
			DMA_SR_TPS_WR(qinx, 1);
		}
		if (GET_VALUE(dma_sr, DMA_SR_TBU_LPOS, DMA_SR_TBU_HPOS) & 1) {
			eqos_status = -E_DMA_SR_TBU;
			//if (pdata->prepare_pg_packet == Y_FALSE) {
				printk(KERN_ALERT "%d] TBU Interrupt\n", qinx);
				pdata->pg->channel_running[qinx] = Y_FALSE;
			//}
			DMA_SR_TBU_WR(qinx, 1);
		}
		if (GET_VALUE(dma_sr, DMA_SR_RI_LPOS, DMA_SR_RI_HPOS) & 1) {
			eqos_disable_all_ch_rx_interrpt(pdata);
			eqos_poll_pg(pdata);
			DMA_SR_RI_WR(qinx, 1);
		}
		if (GET_VALUE(dma_sr, DMA_SR_RBU_LPOS, DMA_SR_RBU_HPOS) & 1) {
			eqos_status = -E_DMA_SR_RBU;
			DMA_SR_RBU_WR(qinx, 1);
			printk(KERN_ALERT "RBU Interrupt\n");
		}
		if (GET_VALUE(dma_sr, DMA_SR_RPS_LPOS, DMA_SR_RPS_HPOS) & 1) {
			eqos_status = -E_DMA_SR_RPS;
			DMA_SR_RPS_WR(qinx, 1);
			printk(KERN_ALERT "RPS Interrupt\n");
		}
		if (GET_VALUE(dma_sr, DMA_SR_RWT_LPOS, DMA_SR_RWT_HPOS) & 1) {
			eqos_status = S_DMA_SR_RWT;
		}
		if (GET_VALUE(dma_sr, DMA_SR_FBE_LPOS, DMA_SR_FBE_HPOS) & 1) {
			eqos_status = -E_DMA_SR_FBE;
			DMA_SR_FBE_WR(qinx, 1);
			DBGPR_PG("FATAL bus error interrupt\n");
		}
	}

	/* MTL Interrupt handler */
	if (GET_VALUE(dma_isr, DMA_ISR_MTLIS_LPOS, DMA_ISR_MTLIS_HPOS) & 1) {
		for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
			/* ABS Interrupt handler */
			eqos_save_abs_count(pdata, qinx);
		}
	}

	DBGPR_PG("<--eqos_pg_isr\n");

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
void eqos_default_confs(struct eqos_prv_data *pdata)
{
	struct eqos_tx_wrapper_descriptor *tx_desc_data = NULL;
	struct eqos_rx_wrapper_descriptor *rx_desc_data = NULL;
	UINT qinx;

	pdata->incr_incrx = EQOS_INCR_ENABLE;
	pdata->flow_ctrl = 0;
	pdata->oldflow_ctrl = 0;

	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		tx_desc_data = GET_TX_WRAPPER_DESC(qinx);

		tx_desc_data->tx_threshold_val = EQOS_TX_THRESHOLD_32;
		tx_desc_data->tsf_on = EQOS_TSF_ENABLE;
		tx_desc_data->osf_on = EQOS_OSF_ENABLE;
		tx_desc_data->tx_pbl = EQOS_PBL_16;
	}

	for (qinx = 0; qinx < EQOS_RX_QUEUE_CNT; qinx++) {
		rx_desc_data = GET_RX_WRAPPER_DESC(qinx);

		rx_desc_data->rx_threshold_val = EQOS_RX_THRESHOLD_64;
		rx_desc_data->rsf_on = EQOS_RSF_DISABLE;
		rx_desc_data->rx_pbl = EQOS_PBL_16;
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
* \param[in] qinx    - DMA channel number
*
* \return void
*/
static void eqos_prepare_desc(struct eqos_prv_data *pdata,
				struct s_tx_normal_desc *txptr,
				struct eqos_tx_buffer *buffer,
				int i,
				unsigned int qinx)
{
	//DBGPR_PG("-->eqos_prepare_desc\n");

	/* update packet address */
	TX_NORMAL_DESC_TDES0_WR(txptr->tdes0, buffer->dma);
	/* update the packet length */
	TX_NORMAL_DESC_TDES2_HL_B1L_WR(txptr->tdes2, buffer->len);
	/* update the frame length */
	TX_NORMAL_DESC_TDES3_FL_WR(txptr->tdes3, buffer->len);
	/* set Interrupt on Completion for last descriptor */
	TX_NORMAL_DESC_TDES2_IC_WR(txptr->tdes2, 0x1);
	/* Mark it as First Descriptor */
	TX_NORMAL_DESC_TDES3_FD_WR(txptr->tdes3, 0x1);
	/* Mark it as LAST descriptor */
	TX_NORMAL_DESC_TDES3_LD_WR(txptr->tdes3, 0x1);
	/* Disable CRC and Pad Insertion */
	TX_NORMAL_DESC_TDES3_CPC_WR(txptr->tdes3, 0);
	/* Mark it as NORMAL descriptor */
	TX_NORMAL_DESC_TDES3_CTXT_WR(txptr->tdes3, 0);
	/* set slot number */
	TX_NORMAL_DESC_TDES3_SLOTNUM_TCPHDRLEN_WR(txptr->tdes3, buffer->slot_number);
	/* set OWN bit at end to avoid race condition */
	TX_NORMAL_DESC_TDES3_OWN_WR(txptr->tdes3, 0x1);

#ifdef EQOS_ENABLE_TX_DESC_DUMP
	dump_tx_desc(pdata, i, i, 1, qinx);
#endif

	//DBGPR_PG("<--eqos_prepare_desc\n");
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
static void eqos_prepare_tx_packets_for_pg_test(struct eqos_prv_data *pdata)
{
	struct eqos_tx_wrapper_descriptor *desc_data = NULL;
	struct eqos_pg_struct *pg_struct = pdata->pg;
	struct eqos_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	UINT head_ptr = 0, tail_ptr = 0, desc_ring_ptr = 0, i, qinx, frame_size;
	unsigned short ch_tx_desc_slot_no_start = 0;
	unsigned short ch_tx_desc_slot_no_skip = 0;
	unsigned int tx_pkt_cnt = 0;
	int desc_idx = 0;

	DBGPR_PG("-->eqos_prepare_tx_packets_for_pg_test\n");

	/* Descriptor memory allocation for transmission */
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		struct s_tx_normal_desc *txptr = NULL;
		struct eqos_tx_buffer *buffer = NULL;
		unsigned int *skb_data = NULL;
		unsigned int avtype = EQOS_AVTYPE;
		int payload_cnt = 0;
		desc_data = GET_TX_WRAPPER_DESC(qinx);

		/* for channel 0 no slot number checking */
		if (qinx > 0) {
			ch_tx_desc_slot_no_start = pg_ch_input[qinx].ch_tx_desc_slot_no_start;
			ch_tx_desc_slot_no_skip = pg_ch_input[qinx].ch_tx_desc_slot_no_skip;
		}

		if (pg_ch_input[qinx].ch_debug_mode == 1) {
			DMA_CHTDR_RD(qinx, head_ptr);
			desc_idx = GET_TX_DESC_IDX(qinx, head_ptr);
			if (pg_ch_input[qinx].ch_max_tx_frame_cnt >= TX_DESC_CNT) {
				tx_pkt_cnt = TX_DESC_CNT;
				pg_ch_input[qinx].ch_desc_prepare =
					(pg_ch_input[qinx].ch_max_tx_frame_cnt - TX_DESC_CNT);
				pg_ch_input[qinx].ch_desc_prepare++;
			}
			else {
				pg_ch_input[qinx].ch_desc_prepare = 0;
				tx_pkt_cnt = (pg_ch_input[qinx].ch_max_tx_frame_cnt % TX_DESC_CNT);
				/* DUT stops when tail & head become equal,
					 prepare one extra packet for transmit */
				tx_pkt_cnt++;
			}
		} else {
			tx_pkt_cnt = TX_DESC_CNT;
			desc_idx = 0;
		}

		for (i = 0; i < tx_pkt_cnt; i++) {
			txptr = GET_TX_DESC_PTR(qinx, desc_idx);
			buffer = GET_TX_BUF_PTR(qinx, desc_idx);
			skb_data = (unsigned int *)buffer->skb->data;

			if (!skb_data) {
				printk(KERN_ALERT "ERROR: No SKB Allocated for channel\n");
				break;
			}

			/* populate tx pg data */
			frame_size = pg_ch_input[qinx].ch_frame_size;
			if (qinx == 0) {
				/* Add Ethernet header */
				*skb_data++ = eqos_frame_hdrs[qinx][0];
				*skb_data++ = eqos_frame_hdrs[qinx][1];
				*skb_data++ = eqos_frame_hdrs[qinx][2];
				*skb_data++ = eqos_frame_hdrs[qinx][3];
				/* Add payload */
				for (payload_cnt = 0; payload_cnt < frame_size;) {
					*skb_data++ = EQOS_FRAME_PATTERN_CH[qinx];
					/* increment by 4 since we are writing
					 * one dword at a time */
					payload_cnt += 4;
				}
				buffer->len = frame_size;
			} else {
				/* Add Ethernet header */
				*skb_data++ = eqos_frame_hdrs[qinx][0];
				*skb_data++ = eqos_frame_hdrs[qinx][1];
				*skb_data++ = eqos_frame_hdrs[qinx][2];
				*skb_data++ = eqos_frame_hdrs[qinx][3];

				if (pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB) {
					avtype = ((avtype << 8) | (avtype >> 8)) & 0x0000FFFF;
					*skb_data++ = (EQOS_FRAME_PATTERN_CH[qinx] << 16) | avtype;
					payload_cnt = 4;
				} else {
					payload_cnt = 0;
				}

				/* Add payload */
				while (payload_cnt < frame_size) {
					*skb_data++ = EQOS_FRAME_PATTERN_CH[qinx];
					/* increment by 4 since we are writing
					 * one dword at a time */
					payload_cnt += 4;
				}
				buffer->len = frame_size;
			}

			dma_sync_single_for_device(&pdata->pdev->dev, buffer->dma,
					EQOS_PG_FRAME_SIZE, DMA_TO_DEVICE);

			if (pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB &&
					pg_ch_input[qinx].ch_operating_mode == EQOS_QDCB) {
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
			eqos_prepare_desc(pdata, txptr, buffer, desc_idx, qinx);
#ifdef EQOS_ENABLE_TX_PKT_DUMP
			if (desc_idx < 1)
				print_pkt(buffer->skb, buffer->len, 1, desc_idx);
#endif
			INCR_TX_DESC_INDEX(desc_idx, 1);
		}

		if (pg_ch_input[qinx].ch_debug_mode == 1) {
			DECR_TX_DESC_INDEX(desc_idx);
			tail_ptr = GET_TX_DESC_DMA_ADDR(qinx, desc_idx);
			printk(KERN_ALERT "ch_desc_prepare     %d\n",
					pg_ch_input[qinx].ch_desc_prepare);
		}
		else {
			/* Updating tail pointer to one descriptor behind head pointer */
			DMA_CHTDR_RD(qinx, head_ptr);
			DMA_TDLAR_RD(qinx, desc_ring_ptr);
			if ((head_ptr == 0) || (desc_ring_ptr == head_ptr)) {
				tail_ptr = GET_TX_DESC_DMA_ADDR(qinx, tx_pkt_cnt - 1);
			}
			else {
				tail_ptr = (head_ptr - sizeof(struct s_tx_normal_desc));
			}
		}
		DMA_TDTP_TPDR_WR(qinx, tail_ptr);
		desc_data->dirty_tx = GET_TX_DESC_IDX(qinx, head_ptr);

		printk(KERN_ALERT
				"%d] Tail @ init    [%3llu]%#x\n"
				"    Head @ init    [%3llu]%#x\n"
				"    dirty_tx @ init %d\n\n",
				qinx, GET_TX_DESC_IDX(qinx, tail_ptr), tail_ptr,
				GET_TX_DESC_IDX(qinx, head_ptr), head_ptr,
				desc_data->dirty_tx);
	}

	DBGPR_PG("<--eqos_prepare_tx_packets_for_pg_test\n");
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
static void eqos_prepare_hw_for_pg_test(struct eqos_prv_data *pdata)
{
	struct eqos_pg_struct *pg_struct = pdata->pg;
	unsigned int qinx;
	struct eqos_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct timespec now;
	UINT mac_tcr = 0;

	DBGPR_PG("-->eqos_prepare_hw_for_pg_test\n");

	hw_if->set_tx_rx_prio_policy(pg_struct->ch_tx_rx_arb_scheme);
	hw_if->set_tx_rx_prio(pg_struct->ch_use_tx_high_prio);
	hw_if->set_tx_rx_prio_ratio(pg_struct->ch_tx_rx_prio_ratio);
	hw_if->set_dma_tx_arb_algorithm(pg_struct->dma_tx_arb_algo);
	hw_if->set_dcb_algorithm(pg_struct->queue_dcb_algorithm);
    hw_if->config_mac_loopback_mode(pg_struct->mac_lb_mode);

	/* Timer programming */
	hw_if->config_sub_second_increment(EQOS_SYSCLOCK);

	mac_tcr = (MAC_TCR_TSENA | MAC_TCR_TSCTRLSSR);
	hw_if->config_hw_time_stamping(mac_tcr);

	getnstimeofday(&now);
	hw_if->init_systime(now.tv_sec, now.tv_nsec);

	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		hw_if->set_tx_queue_operating_mode(qinx, pg_ch_input[qinx].ch_operating_mode);

		/* DCB parameters */
		hw_if->set_dcb_queue_weight(qinx, pg_ch_input[qinx].ch_queue_weight);
		hw_if->set_ch_arb_weights(qinx, pg_ch_input[qinx].ch_arb_weight);

		/* Slot parameters */
		hw_if->config_slot_num_check(qinx, pg_ch_input[qinx].ch_enable_slot_check);
		hw_if->config_slot_interrupt(qinx, pg_ch_input[qinx].ch_enable_slot_check);
		hw_if->set_slot_count(qinx, pg_ch_input[qinx].ch_slot_count);
		hw_if->config_advance_slot_num_check(qinx, pg_ch_input[qinx].ch_enable_adv_slot_check);

		/* AVB parameters */
		if ((pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB) && (qinx > 0)) {
			hw_if->set_avb_algorithm(qinx, pg_ch_input[qinx].ch_avb_algorithm);
			if (pg_ch_input[qinx].ch_avb_algorithm == EQOS_AVB_CBS) {
				hw_if->config_credit_control(qinx, pg_ch_input[qinx].ch_credit_control);
				hw_if->config_send_slope(qinx, pg_ch_input[qinx].ch_send_slope);
				hw_if->config_idle_slope(qinx, pg_ch_input[qinx].ch_idle_slope);
				hw_if->config_high_credit(qinx, pg_ch_input[qinx].ch_hi_credit);
				hw_if->config_low_credit(qinx, pg_ch_input[qinx].ch_lo_credit);
			}
		}
	}

	DBGPR_PG("<--eqos_prepare_hw_for_pg_test\n");
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
static void eqos_pg_timer_fun(unsigned long data)
{
	struct eqos_prv_data *pdata = (struct eqos_prv_data *)data;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct eqos_tx_wrapper_descriptor *desc_data = NULL;
	struct eqos_pg_ch_input *pg_ch_input = NULL;
	UINT qinx, head_ptr = 0, tail_ptr = 0, dma_dsr = 0;

	printk(KERN_ALERT "-->eqos_pg_timer_fun\n");

	/* allow device to transmit pending prepared packets */
	pdata->prepare_pg_packet = Y_FALSE;

	mdelay(500);

	DMA_DSR0_RD(dma_dsr);
	printk(KERN_ALERT "DMA Channel state: %#x\n", dma_dsr);
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		pg_ch_input = &(pdata->pg->pg_ch_input[qinx]);
		desc_data = GET_TX_WRAPPER_DESC(qinx);
		/* disable only that channel for which ENABLE bit is set */
		if ((pdata->pg->ch_sel_mask) & (1 << qinx)) {
			hw_if->stop_dma_tx(pdata, qinx);
		}
		DMA_TDTP_TPDR_RD(qinx, tail_ptr);
		DMA_CHTDR_RD(qinx, head_ptr);
		printk(KERN_ALERT
				"%d] Tail @ stop     [%3llu]%#x\n"
				"    Head @ stop     [%3llu]%#x\n"
				"    dirty_tx @ stop  %d\n"
				"    ch_framecount_tx  %lu\n"
				"    Total interrupts %d\n\n",
				qinx, GET_TX_DESC_IDX(qinx, tail_ptr), tail_ptr,
				GET_TX_DESC_IDX(qinx, head_ptr), head_ptr,
				desc_data->dirty_tx,
				pg_ch_input->ch_framecount_tx,
				pg_ch_input->tx_interrupts);
	}
	// TODO: add code to disable slot interrupt except channel 0

	pdata->run_test = Y_FALSE;

	printk(KERN_ALERT "PG Experiment is completed ....\n"\
		"You can retrieve the Report\n");
	DBGPR_PG("PG Experiment is completed ....\n"\
		"You can retrieve the Report\n");

	printk(KERN_ALERT "<--eqos_pg_timer_fun\n");
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
static void eqos_pg_run(struct eqos_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned int qinx;

	DBGPR_PG("-->eqos_pg_run\n");

	eqos_prepare_tx_packets_for_pg_test(pdata);

	pdata->run_test = Y_TRUE;
	pdata->prepare_pg_packet = Y_TRUE;

	/* start pg timer before enabling the DMA's */
	add_timer(&pdata->pg_timer);

	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		/* enable only that channel for which ENABLE bit is set */
		if ((pdata->pg->ch_sel_mask) & (1 << qinx)) {
			pdata->pg->channel_running[qinx] = Y_TRUE;
			hw_if->start_dma_tx(qinx);
		}
		else {
			pdata->pg->channel_running[qinx] = Y_FALSE;
		}
	}

	DBGPR_PG("<--eqos_pg_run\n");
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
static void eqos_setup_timer(struct eqos_prv_data *pdata)
{
	DBGPR_PG("-->eqos_setup_timer\n");

	init_timer(&pdata->pg_timer);
	pdata->pg_timer.expires = (HZ * (pdata->pg->duration_of_exp) + jiffies);
	pdata->pg_timer.data = (unsigned long)pdata;
	pdata->pg_timer.function = eqos_pg_timer_fun;

	printk(KERN_ALERT "Test will expire at %d\n\n", (INT)pdata->pg_timer.expires);

	DBGPR_PG("<--eqos_setup_timer\n");
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
static void eqos_pg_run_test(struct eqos_prv_data *pdata)
{
	DBGPR_PG("-->eqos_pg_run_test\n");

	printk(KERN_ALERT "PG Experiment has started ....\n");
	eqos_setup_timer(pdata);
	eqos_pg_run(pdata);

	DBGPR_PG("<--eqos_pg_run_test\n");
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
static void eqos_print_pg_struct(struct eqos_prv_data *pdata)
{
	struct eqos_pg_struct *pg_struct = pdata->pg;
	struct eqos_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	unsigned int qinx;
	char *space = "                               ", *strptr = NULL;
	unsigned char display_avb_params = 0, display_dcb_params = 0;

	DBGPR_PG("-->eqos_print_pg_struct\n");

	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		if (pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB) {
			display_avb_params = 1;
			break;
		}
	}
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		if (pg_ch_input[qinx].ch_operating_mode == EQOS_QDCB) {
			display_dcb_params = 1;
			break;
		}
	}

	printk(KERN_ALERT "duration_of_exp                       = %#02x\n",
		pg_struct->duration_of_exp);
	printk(KERN_ALERT "ch_sel_mask                          = %#02x\n",
		pg_struct->ch_sel_mask);
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
			case EQOS_DCB_WRR:
				strptr = "WRR (Weighted Round Robin)";
				break;
			case EQOS_DCB_WFQ:
				strptr = "WFQ (Weighted Fair Queuing)";
				break;
			case EQOS_DCB_DWRR:
				strptr = "DWRR (Deficit Weighted Round Robin)";
				break;
			case EQOS_DCB_SP:
				strptr = "SP (Strict Priority)";
				break;
		}
		printk(KERN_ALERT "queue_dcb_algorithm                 = %s\n",
				strptr);
	}

	printk(KERN_ALERT "prio_tag_for_av (not used)             = %#02x\n",
		pg_struct->prio_tag_for_av);

	printk(KERN_ALERT "ch_operating_mode\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		switch (pg_ch_input[qinx].ch_operating_mode) {
		case EQOS_QDISABLED:
			printk(KERN_ALERT "%s[Ch%d] = Disabled\n", space, qinx);
			break;
		case EQOS_QAVB:
			printk(KERN_ALERT "%s[Ch%d] = AVB\n", space, qinx);
			break;
		case EQOS_QDCB:
			printk(KERN_ALERT "%s[Ch%d] = DCB\n", space, qinx);
			break;
		case EQOS_QGENERIC:
			printk(KERN_ALERT "%s[Ch%d] = Generic\n", space, qinx);
			break;
		}
	}

	printk(KERN_ALERT "ch_arb_weight [DMA]\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %#01x\n", space, qinx,
				pg_ch_input[qinx].ch_arb_weight);
	}
	printk(KERN_ALERT "ch_bw\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %d%%\n", space, qinx,
			pg_ch_input[qinx].ch_bw);
	}

	printk(KERN_ALERT "ch_queue_weight\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %d\n", space, qinx,
			pg_ch_input[qinx].ch_queue_weight);
	}

	printk(KERN_ALERT "ch_frame_size\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %d\n", space, qinx,
			pg_ch_input[qinx].ch_frame_size);
	}

	printk(KERN_ALERT "ch_enable_slot_check\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %s\n", space, qinx,
			pg_ch_input[qinx].ch_enable_slot_check ? "YES" : "NO");
	}

	printk(KERN_ALERT "ch_enable_adv_slot_check\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %s\n", space, qinx,
			pg_ch_input[qinx].ch_enable_adv_slot_check ? "YES" : "NO");
	}

	printk(KERN_ALERT "ch_slot_count\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %d\n", space, qinx,
			pg_ch_input[qinx].ch_slot_count);
	}

	printk(KERN_ALERT "ch_tx_desc_slot_no_start\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %#01x\n", space, qinx,
			pg_ch_input[qinx].ch_tx_desc_slot_no_start);
	}

	printk(KERN_ALERT "ch_tx_desc_slot_no_skip\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %#01x\n", space, qinx,
			pg_ch_input[qinx].ch_tx_desc_slot_no_skip);
	}

	printk(KERN_ALERT "ch_avg_bits\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %lu\n", space, qinx,
			pg_ch_input[qinx].ch_avg_bits);
	}

	printk(KERN_ALERT "ch_avg_bits_interrupt_count\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %lu\n", space, qinx,
			pg_ch_input[qinx].ch_avg_bits_interrupt_count);
	}

	if (display_avb_params) {
		printk(KERN_ALERT "ch_avb_algorithm\n");
		for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
			if (pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %s\n", space, qinx,
						(pg_ch_input[qinx].ch_avb_algorithm == EQOS_AVB_SP ?
						 "Strict Priority": "Credit Based Shaper"));
		}

		printk(KERN_ALERT "ch_credit_control\n");
		for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
			if (pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %s\n", space, qinx,
						pg_ch_input[qinx].ch_credit_control ? "YES" : "NO");
		}

		printk(KERN_ALERT "ch_send_slope\n");
		for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
			if (pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %#08x\n", space, qinx,
						pg_ch_input[qinx].ch_send_slope);
		}

		printk(KERN_ALERT "ch_idle_slope\n");
		for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
			if (pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %#08x\n", space, qinx,
						pg_ch_input[qinx].ch_idle_slope);
		}

		printk(KERN_ALERT "ch_hi_credit\n");
		for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
			if (pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %#08x\n", space, qinx,
						pg_ch_input[qinx].ch_hi_credit);
		}

		printk(KERN_ALERT "ch_lo_credit\n");
		for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
			if (pg_ch_input[qinx].ch_operating_mode == EQOS_QAVB)
				printk(KERN_ALERT "%s[Ch%d] = %#08x\n", space, qinx,
						pg_ch_input[qinx].ch_lo_credit);
		}
	}

	printk(KERN_ALERT "ch_framecount_tx\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %lu\n", space, qinx,
			pg_ch_input[qinx].ch_framecount_tx);
	}

	printk(KERN_ALERT "ch_framecount_rx\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[Ch%d] = %lu\n", space, qinx,
			pg_ch_input[qinx].ch_framecount_rx);
	}

	printk(KERN_ALERT "Debug mode\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[CH%d]  %s\n",
			space, qinx, (pg_ch_input[qinx].ch_debug_mode ? "YES" : "NO"));
	}

	printk(KERN_ALERT "Maximum Tx packet count\n");
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		printk(KERN_ALERT "%s[CH%d]  %d\n",
			space, qinx, pg_ch_input[qinx].ch_max_tx_frame_cnt);
	}

	DBGPR_PG("<--eqos_print_pg_struct\n");
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
static void eqos_pg_set_config(struct eqos_prv_data *pdata,
					struct ifr_data_struct *req)
{
	struct eqos_pg_struct l_pg_struct;
	struct eqos_pg_struct *user_pg_struct =
		(struct eqos_pg_struct *)req->ptr;
	struct eqos_pg_struct *pg_struct = pdata->pg;
	struct eqos_pg_ch_input *pg_ch_input =
		pg_struct->pg_ch_input;
	unsigned int qinx;

	DBGPR_PG("-->eqos_pg_set_config\n");

	/* First, copy contents of user info in a local structure */
	if(copy_from_user(&l_pg_struct, user_pg_struct,
				sizeof(struct eqos_pg_struct)))
		printk(KERN_ALERT "Failed to fetch PG Struct info from user\n");

	/* Second, copy required members into kernel structure */
	copy_pg_struct_members(pg_struct, &l_pg_struct);
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		copy_pg_ch_input_members(&(pg_ch_input[qinx]), &(l_pg_struct.pg_ch_input[qinx]));
	}

	eqos_print_pg_struct(pdata);

	DBGPR_PG("<--eqos_pg_set_config\n");
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
static void eqos_pg_get_result(struct eqos_prv_data *pdata,
					struct ifr_data_struct *req)
{
	struct eqos_pg_struct l_pg_struct;
	struct eqos_pg_struct *user_pg_struct =
		(struct eqos_pg_struct *)req->ptr;
	struct eqos_pg_struct *pg_struct = pdata->pg;
	struct eqos_pg_ch_input *pg_ch_input =
		pg_struct->pg_ch_input;
	unsigned int qinx;

	DBGPR_PG("-->eqos_pg_get_result\n");
	copy_pg_struct_members(&l_pg_struct, pg_struct);
    l_pg_struct.speed_100M_1G = pdata->speed;/* Update the speed information */
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		copy_pg_ch_input_members(&(l_pg_struct.pg_ch_input[qinx]), &(pg_ch_input[qinx]));
	}
	if (copy_to_user(user_pg_struct, &l_pg_struct, sizeof(struct eqos_pg_struct)))
		printk(KERN_ALERT "Failed to send PG Struct info to user\n");

	DBGPR_PG("<--eqos_pg_get_result\n");
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
int eqos_handle_pg_ioctl(struct eqos_prv_data *pdata,
				void *ptr)
{
	struct ifr_data_struct *req = ptr;
	int ret = 0;

	DBGPR_PG("-->eqos_handle_pg_ioctl\n");

	switch (req->flags) {
	case EQOS_PG_SET_CONFIG:
		eqos_pg_set_config(pdata, req);
		break;
	case EQOS_PG_CONFIG_HW:
		eqos_prepare_hw_for_pg_test(pdata);
		eqos_prepare_tx_packets_for_pg_test(pdata);
		printk(KERN_ALERT "\nCONFIGURING THE HW FOR PG ....\n");
		break;
	case EQOS_PG_RUN_TEST:
		eqos_pg_run_test(pdata);
		printk(KERN_ALERT "PG RUN TEST STARTED ....\n");
		break;
	case EQOS_PG_GET_RESULT:
		eqos_pg_get_result(pdata, req);
		break;
	case EQOS_PG_TEST_DONE:
		req->test_done = pdata->run_test;
		break;
	default:
		printk(KERN_ALERT "Wrong Parameter for PG TEST\n");
		ret = -EINVAL;
	}

	DBGPR_PG("<--eqos_handle_pg_ioctl\n");

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
int eqos_alloc_pg(struct eqos_prv_data *pdata)
{
	DBGPR_PG("-->eqos_alloc_pg\n");

	pdata->pg = kzalloc(sizeof(struct eqos_pg_struct), GFP_KERNEL);
	if (pdata->pg == NULL) {
		printk(KERN_ALERT "%s:Unable to alloc pg structure\n", DEV_NAME);
		return -ENOMEM;
	}

	DBGPR_PG("<--eqos_alloc_pg\n");

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
void eqos_free_pg(struct eqos_prv_data *pdata)
{
	DBGPR_PG("-->eqos_free_pg\n");

	//kfree(pdata->pg->pg_ch_input);
	kfree(pdata->pg);

	DBGPR_PG("<--eqos_free_pg\n");
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
int eqos_alloc_tx_buf_pg(struct eqos_prv_data *pdata,
				struct eqos_tx_buffer *buffer,
				gfp_t gfp)
{
	struct sk_buff *skb = NULL;

	//DBGPR_PG("-->eqos_alloc_tx_buf_pg\n");

	skb = dev_alloc_skb(EQOS_PG_FRAME_SIZE);
	if (skb == NULL) {
		printk(KERN_ALERT "Failed to allocate tx skb\n");
		return -ENOMEM;
	}
	buffer->skb = skb;
	buffer->len = EQOS_PG_FRAME_SIZE;
	buffer->dma = dma_map_single(&pdata->pdev->dev, skb->data,
				     EQOS_PG_FRAME_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(&pdata->pdev->dev, buffer->dma))
		printk(KERN_ALERT "failed to do the TX dma map\n");

	buffer->buf1_mapped_as_page = Y_FALSE;

	//DBGPR_PG("<--eqos_alloc_tx_buf_pg\n");

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
int eqos_alloc_rx_buf_pg(struct eqos_prv_data *pdata,
				struct eqos_rx_buffer *buffer,
				gfp_t gfp)
{
	struct sk_buff *skb = NULL;

	//DBGPR_PG("-->eqos_alloc_rx_buf_pg\n");

	skb = dev_alloc_skb(EQOS_PG_FRAME_SIZE);
	if (skb == NULL) {
		printk(KERN_ALERT "Failed to allocate tx skb\n");
		return -ENOMEM;
	}
	buffer->skb = skb;
	buffer->len = EQOS_PG_FRAME_SIZE;
	buffer->dma = dma_map_single(&pdata->pdev->dev, skb->data,
				     EQOS_PG_FRAME_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(&pdata->pdev->dev, buffer->dma))
		printk(KERN_ALERT "failed to do the RX dma map\n");

	buffer->mapped_as_page = Y_FALSE;
	wmb();

	//DBGPR_PG("<--eqos_alloc_rx_buf_pg\n");

	return 0;
}

