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
#include "drv.h"

extern ULONG dwc_eth_qos_base_addr;
#include "yregacc.h"
#include <linux/inet_lro.h>

static INT DWC_ETH_QOS_GStatus;

/* SA(Source Address) operations on TX */
unsigned char mac_addr0[6] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55 };
unsigned char mac_addr1[6] = { 0x00, 0x66, 0x77, 0x88, 0x99, 0xaa };

/* module parameters for configuring the queue modes
 * set default mode as GENERIC
 * */
/* Value of "2" enables mtl tx q */
static int q_op_mode[DWC_ETH_QOS_MAX_TX_QUEUE_CNT] = {
	2,
	2,
	2,
	2,
	2,
	2,
	2,
	2
};
module_param_array(q_op_mode, int, NULL, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(q_op_mode,
	"MTL queue operation mode [0-DISABLED, 1-AVB, 2-DCB, 3-GENERIC]");

void DWC_ETH_QOS_stop_all_ch_tx_dma(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT qInx;

	DBGPR("-->DWC_ETH_QOS_stop_all_ch_tx_dma\n");

	for(qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++)
		hw_if->stop_dma_tx(qInx);

	DBGPR("<--DWC_ETH_QOS_stop_all_ch_tx_dma\n");
}

static void DWC_ETH_QOS_stop_all_ch_rx_dma(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT qInx;

	DBGPR("-->DWC_ETH_QOS_stop_all_ch_rx_dma\n");

	for(qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++)
		hw_if->stop_dma_rx(qInx);

	DBGPR("<--DWC_ETH_QOS_stop_all_ch_rx_dma\n");
}

static void DWC_ETH_QOS_start_all_ch_tx_dma(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT i;

	DBGPR("-->DWC_ETH_QOS_start_all_ch_tx_dma\n");

	for(i = 0; i < DWC_ETH_QOS_TX_QUEUE_CNT; i++)
		hw_if->start_dma_tx(i);

	DBGPR("<--DWC_ETH_QOS_start_all_ch_tx_dma\n");
}

static void DWC_ETH_QOS_start_all_ch_rx_dma(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT i;

	DBGPR("-->DWC_ETH_QOS_start_all_ch_rx_dma\n");

	for(i = 0; i < DWC_ETH_QOS_RX_QUEUE_CNT; i++)
		hw_if->start_dma_rx(i);

	DBGPR("<--DWC_ETH_QOS_start_all_ch_rx_dma\n");
}

static void DWC_ETH_QOS_napi_enable_mq(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_rx_queue *rx_queue = NULL;
	int qInx;

	DBGPR("-->DWC_ETH_QOS_napi_enable_mq\n");

	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++) {
		rx_queue = GET_RX_QUEUE_PTR(qInx);
		napi_enable(&rx_queue->napi);
	}

	DBGPR("<--DWC_ETH_QOS_napi_enable_mq\n");
}

static void DWC_ETH_QOS_all_ch_napi_disable(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_rx_queue *rx_queue = NULL;
	int qInx;

	DBGPR("-->DWC_ETH_QOS_napi_enable\n");

	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++) {
		rx_queue = GET_RX_QUEUE_PTR(qInx);
		napi_disable(&rx_queue->napi);
	}

	DBGPR("<--DWC_ETH_QOS_napi_enable\n");
}

/*!
 * \details This function is invoked to stop device operation
 * Following operations are performed in this function.
 * - Stop the queue.
 * - Stops DMA TX and RX.
 * - Free the TX and RX skb's.
 * - Issues soft reset to device.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void DWC_ETH_QOS_stop_dev(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct desc_if_struct *desc_if = &(pdata->desc_if);

	DBGPR("-->DWC_ETH_QOS_stop_dev\n");

	netif_tx_disable(pdata->dev);

	DWC_ETH_QOS_all_ch_napi_disable(pdata);

	/* stop DMA TX/RX */
	DWC_ETH_QOS_stop_all_ch_tx_dma(pdata);
	DWC_ETH_QOS_stop_all_ch_rx_dma(pdata);

	/* issue software reset to device */
	hw_if->exit();

	/* free tx skb's */
	desc_if->tx_skb_free_mem(pdata, DWC_ETH_QOS_TX_QUEUE_CNT);
	/* free rx skb's */
	desc_if->rx_skb_free_mem(pdata, DWC_ETH_QOS_RX_QUEUE_CNT);

	DBGPR("<--DWC_ETH_QOS_stop_dev\n");
}


static void DWC_ETH_QOS_tx_desc_mang_ds_dump(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_tx_wrapper_descriptor *tx_desc_data = NULL;
	struct s_TX_NORMAL_DESC *tx_desc = NULL;
	int qInx, i;

#ifndef YDEBUG
	return;
#endif
	printk(KERN_ALERT "/**** TX DESC MANAGEMENT DATA STRUCTURE DUMP ****/\n");

	printk(KERN_ALERT "TX_DESC_QUEUE_CNT = %d\n", DWC_ETH_QOS_TX_QUEUE_CNT);
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		tx_desc_data = GET_TX_WRAPPER_DESC(qInx);

		printk(KERN_ALERT "DMA CHANNEL = %d\n", qInx);

		printk(KERN_ALERT "\tcur_tx           = %d\n",
			tx_desc_data->cur_tx);
		printk(KERN_ALERT "\tdirty_tx         = %d\n",
			tx_desc_data->dirty_tx);
		printk(KERN_ALERT "\tfree_desc_cnt    = %d\n",
			tx_desc_data->free_desc_cnt);
		printk(KERN_ALERT "\ttx_pkt_queued    = %d\n",
			tx_desc_data->tx_pkt_queued);
		printk(KERN_ALERT "\tqueue_stopped    = %d\n",
			tx_desc_data->queue_stopped);
		printk(KERN_ALERT "\tpacket_count     = %d\n",
			tx_desc_data->packet_count);
		printk(KERN_ALERT "\ttx_threshold_val = %d\n",
			tx_desc_data->tx_threshold_val);
		printk(KERN_ALERT "\ttsf_on           = %d\n",
			tx_desc_data->tsf_on);
		printk(KERN_ALERT "\tosf_on           = %d\n",
			tx_desc_data->osf_on);
		printk(KERN_ALERT "\ttx_pbl           = %d\n",
			tx_desc_data->tx_pbl);

		printk(KERN_ALERT "\t[<desc_add>  <dma_add> <index >] = <TDES0> : <TDES1> : <TDES2> : <TDES3>\n");
		for (i = 0; i < TX_DESC_CNT; i++) {
			tx_desc = GET_TX_DESC_PTR(qInx, i);
			printk(KERN_ALERT "\t[%4p %4p %03d] = %#x : %#x : %#x : %#x\n",
				tx_desc, (void *)(GET_TX_DESC_DMA_ADDR(qInx, i)), i, tx_desc->TDES0, tx_desc->TDES1,
				tx_desc->TDES2, tx_desc->TDES3);
		}
	}

	printk(KERN_ALERT "/************************************************/\n");
}


static void DWC_ETH_QOS_rx_desc_mang_ds_dump(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data = NULL;
	struct s_RX_NORMAL_DESC *rx_desc = NULL;
	int qInx, i;

#ifndef YDEBUG
	return;
#endif
	printk(KERN_ALERT "/**** RX DESC MANAGEMENT DATA STRUCTURE DUMP ****/\n");

	printk(KERN_ALERT "RX_DESC_QUEUE_CNT = %d\n", DWC_ETH_QOS_RX_QUEUE_CNT);
	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++) {
		rx_desc_data = GET_RX_WRAPPER_DESC(qInx);

		printk(KERN_ALERT "DMA CHANNEL = %d\n", qInx);

		printk(KERN_ALERT "\tcur_rx                = %d\n",
			rx_desc_data->cur_rx);
		printk(KERN_ALERT "\tdirty_rx              = %d\n",
			rx_desc_data->dirty_rx);
		printk(KERN_ALERT "\tpkt_received          = %d\n",
			rx_desc_data->pkt_received);
		printk(KERN_ALERT "\tskb_realloc_idx       = %d\n",
			rx_desc_data->skb_realloc_idx);
		printk(KERN_ALERT "\tskb_realloc_threshold = %d\n",
			rx_desc_data->skb_realloc_threshold);
		printk(KERN_ALERT "\tuse_riwt              = %d\n",
			rx_desc_data->use_riwt);
		printk(KERN_ALERT "\trx_riwt               = %d\n",
			rx_desc_data->rx_riwt);
		printk(KERN_ALERT "\trx_coal_frames        = %d\n",
			rx_desc_data->rx_coal_frames);
		printk(KERN_ALERT "\trx_threshold_val      = %d\n",
			rx_desc_data->rx_threshold_val);
		printk(KERN_ALERT "\trsf_on                = %d\n",
			rx_desc_data->rsf_on);
		printk(KERN_ALERT "\trx_pbl                = %d\n",
			rx_desc_data->rx_pbl);

		printk(KERN_ALERT "\t[<desc_add> <dma_add> <index >] = <RDES0> : <RDES1> : <RDES2> : <RDES3>\n");
		for (i = 0; i < RX_DESC_CNT; i++) {
			rx_desc = GET_RX_DESC_PTR(qInx, i);
			printk(KERN_ALERT "\t[%4p %4p %03d] = %#x : %#x : %#x : %#x\n",
				rx_desc, (void *)(GET_RX_DESC_DMA_ADDR(qInx, i)), i, rx_desc->RDES0, rx_desc->RDES1,
				rx_desc->RDES2, rx_desc->RDES3);
		}
	}

	printk(KERN_ALERT "/************************************************/\n");
}


static void DWC_ETH_QOS_restart_phy(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR("-->DWC_ETH_QOS_restart_phy\n");

	pdata->oldlink = 0;
	pdata->speed = 0;
	pdata->oldduplex = -1;

	if (pdata->phydev)
		phy_start_aneg(pdata->phydev);

	DBGPR("<--DWC_ETH_QOS_restart_phy\n");
}


/*!
 * \details This function is invoked to start the device operation
 * Following operations are performed in this function.
 * - Initialize software states
 * - Initialize the TX and RX descriptors queue.
 * - Initialize the device to know state
 * - Start the queue.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void DWC_ETH_QOS_start_dev(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct desc_if_struct *desc_if = &(pdata->desc_if);

	DBGPR("-->DWC_ETH_QOS_start_dev\n");

	/* reset all variables */
	DWC_ETH_QOS_default_common_confs(pdata);
	DWC_ETH_QOS_default_tx_confs(pdata);
	DWC_ETH_QOS_default_rx_confs(pdata);

	DWC_ETH_QOS_configure_rx_fun_ptr(pdata);

	DWC_ETH_QOS_napi_enable_mq(pdata);

	/* reinit descriptor */
	desc_if->wrapper_tx_desc_init(pdata);
	desc_if->wrapper_rx_desc_init(pdata);

	DWC_ETH_QOS_tx_desc_mang_ds_dump(pdata);
	DWC_ETH_QOS_rx_desc_mang_ds_dump(pdata);

	/* initializes MAC and DMA */
	hw_if->init(pdata);

	if (pdata->vlan_hash_filtering)
		hw_if->update_vlan_hash_table_reg(pdata->vlan_ht_or_id);
	else
		hw_if->update_vlan_id(pdata->vlan_ht_or_id);

	DWC_ETH_QOS_restart_phy(pdata);

#ifdef HWA_NV_1618922
#else
	pdata->eee_enabled = DWC_ETH_QOS_eee_init(pdata);
#endif

	netif_tx_wake_all_queues(pdata->dev);

	DBGPR("<--DWC_ETH_QOS_start_dev\n");
}

/*!
 * \details This function is invoked by isr handler when device issues an FATAL
 * bus error interrupt.  Following operations are performed in this function.
 * - Stop the device.
 * - Start the device
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] qInx – queue number.
 *
 * \return void
 */

static void DWC_ETH_QOS_restart_dev(struct DWC_ETH_QOS_prv_data *pdata,
					UINT qInx)
{
	struct desc_if_struct *desc_if = &(pdata->desc_if);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct DWC_ETH_QOS_rx_queue *rx_queue = GET_RX_QUEUE_PTR(qInx);

	DBGPR("-->DWC_ETH_QOS_restart_dev\n");

	netif_stop_subqueue(pdata->dev, qInx);
	napi_disable(&rx_queue->napi);

	/* stop DMA TX/RX */
	hw_if->stop_dma_tx(qInx);
	hw_if->stop_dma_rx(qInx);

	/* free tx skb's */
	desc_if->tx_skb_free_mem_single_q(pdata, qInx);
	/* free rx skb's */
	desc_if->rx_skb_free_mem_single_q(pdata, qInx);

	if ((DWC_ETH_QOS_TX_QUEUE_CNT == 0) &&
		(DWC_ETH_QOS_RX_QUEUE_CNT == 0)) {
		/* issue software reset to device */
		hw_if->exit();

		DWC_ETH_QOS_configure_rx_fun_ptr(pdata);
		DWC_ETH_QOS_default_common_confs(pdata);
	}
	/* reset all variables */
	DWC_ETH_QOS_default_tx_confs_single_q(pdata, qInx);
	DWC_ETH_QOS_default_rx_confs_single_q(pdata, qInx);

	/* reinit descriptor */
	desc_if->wrapper_tx_desc_init_single_q(pdata, qInx);
	desc_if->wrapper_rx_desc_init_single_q(pdata, qInx);

	napi_enable(&rx_queue->napi);

	/* initializes MAC and DMA
	 * NOTE : Do we need to init only one channel
	 * which generate FBE*/
	hw_if->init(pdata);

	DWC_ETH_QOS_restart_phy(pdata);

	netif_wake_subqueue(pdata->dev, qInx);

	DBGPR("<--DWC_ETH_QOS_restart_dev\n");
}

void DWC_ETH_QOS_disable_all_ch_rx_interrpt(
			struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT qInx;

	DBGPR("-->DWC_ETH_QOS_disable_all_ch_rx_interrpt\n");

	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++)
		hw_if->disable_rx_interrupt(qInx);

	DBGPR("<--DWC_ETH_QOS_disable_all_ch_rx_interrpt\n");
}

void DWC_ETH_QOS_enable_all_ch_rx_interrpt(
			struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT qInx;

	DBGPR("-->DWC_ETH_QOS_enable_all_ch_rx_interrpt\n");

	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++)
		hw_if->enable_rx_interrupt(qInx);

	DBGPR("<--DWC_ETH_QOS_enable_all_ch_rx_interrpt\n");
}


/*!
* \brief Interrupt Service Routine
* \details Interrupt Service Routine
*
* \param[in] irq         - interrupt number for particular device
* \param[in] device_id   - pointer to device structure
* \return returns positive integer
* \retval IRQ_HANDLED
*/

irqreturn_t DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS(int irq, void *device_id)
{
	ULONG varDMA_ISR;
	ULONG varDMA_SR;
	ULONG varMAC_ISR;
	ULONG varMAC_IMR;
	ULONG varMAC_PMTCSR;
	ULONG varDMA_IER;
	struct DWC_ETH_QOS_prv_data *pdata =
	    (struct DWC_ETH_QOS_prv_data *)device_id;
	struct net_device *dev = pdata->dev;
#ifndef HWA_NV_1637630
	struct hw_if_struct *hw_if = &(pdata->hw_if);
#endif
	UINT qInx;
	int napi_sched = 0;
	struct DWC_ETH_QOS_rx_queue *rx_queue = NULL;
	ULONG varMAC_ANS = 0;
	ULONG varMAC_PCS = 0;

	DBGPR("-->DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS\n");

	DMA_ISR_RgRd(varDMA_ISR);
	if (varDMA_ISR == 0x0)
		return IRQ_NONE;

	MAC_ISR_RgRd(varMAC_ISR);

	DBGPR("DMA_ISR = %#lx, MAC_ISR = %#lx\n", varDMA_ISR, varMAC_ISR);

	/* Handle DMA interrupts */
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		rx_queue = GET_RX_QUEUE_PTR(qInx);

		DMA_SR_RgRd(qInx, varDMA_SR);

		DMA_IER_RgRd(qInx, varDMA_IER);

		/* clear and process only those interrupts which we
		 * have enabled.
		 */
		varDMA_SR = (varDMA_SR & varDMA_IER);
		DMA_SR_RgWr(qInx, varDMA_SR);

		DBGPR("DMA_SR[%d] = %#lx\n", qInx, varDMA_SR);

		if (varDMA_SR == 0)
			continue;

		if ((GET_VALUE(varDMA_SR, DMA_SR_RI_LPOS, DMA_SR_RI_HPOS) & 1) ||
			(GET_VALUE(varDMA_SR, DMA_SR_RBU_LPOS, DMA_SR_RBU_HPOS) & 1)) {
			if (!napi_sched) {
				napi_sched = 1;
				if (likely(napi_schedule_prep(&rx_queue->napi))) {
					DWC_ETH_QOS_disable_all_ch_rx_interrpt(pdata);
					__napi_schedule(&rx_queue->napi);
				} else {
					printk(KERN_ALERT "driver bug! Rx interrupt while in poll\n");
					DWC_ETH_QOS_disable_all_ch_rx_interrpt(pdata);
				}

				if ((GET_VALUE(varDMA_SR, DMA_SR_RI_LPOS, DMA_SR_RI_HPOS) & 1))
					pdata->xstats.rx_normal_irq_n[qInx]++;
				else
					pdata->xstats.rx_buf_unavailable_irq_n[qInx]++;
			}
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_TI_LPOS, DMA_SR_TI_HPOS) & 1) {
			pdata->xstats.tx_normal_irq_n[qInx]++;
			DWC_ETH_QOS_tx_interrupt(dev, pdata, qInx);
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_TPS_LPOS, DMA_SR_TPS_HPOS) & 1) {
			pdata->xstats.tx_process_stopped_irq_n[qInx]++;
			DWC_ETH_QOS_GStatus = -E_DMA_SR_TPS;
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_TBU_LPOS, DMA_SR_TBU_HPOS) & 1) {
			pdata->xstats.tx_buf_unavailable_irq_n[qInx]++;
			DWC_ETH_QOS_GStatus = -E_DMA_SR_TBU;
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_RPS_LPOS, DMA_SR_RPS_HPOS) & 1) {
			pdata->xstats.rx_process_stopped_irq_n[qInx]++;
			DWC_ETH_QOS_GStatus = -E_DMA_SR_RPS;
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_RWT_LPOS, DMA_SR_RWT_HPOS) & 1) {
			pdata->xstats.rx_watchdog_irq_n++;
			DWC_ETH_QOS_GStatus = S_DMA_SR_RWT;
		}
		if (GET_VALUE(varDMA_SR, DMA_SR_FBE_LPOS, DMA_SR_FBE_HPOS) & 1) {
			pdata->xstats.fatal_bus_error_irq_n++;
			DWC_ETH_QOS_GStatus = -E_DMA_SR_FBE;
			DWC_ETH_QOS_restart_dev(pdata, qInx);
		}
	}

	/* Handle MAC interrupts */
	if (GET_VALUE(varDMA_ISR, DMA_ISR_MACIS_LPOS, DMA_ISR_MACIS_HPOS) & 1) {
		/* handle only those MAC interrupts which are enabled */
		MAC_IMR_RgRd(varMAC_IMR);
		varMAC_ISR = (varMAC_ISR & varMAC_IMR);

		/* PMT interrupt
		 * RemoteWake and MagicPacket events will be received by PHY supporting
		 * these features on silicon and can be used to wake up Tegra.
		 * Still let the below code be here in case we ever get this interrupt.
		 */
		if (GET_VALUE(varMAC_ISR, MAC_ISR_PMTIS_LPOS, MAC_ISR_PMTIS_HPOS) & 1) {
			pdata->xstats.pmt_irq_n++;
			DWC_ETH_QOS_GStatus = S_MAC_ISR_PMTIS;
			MAC_PMTCSR_RgRd(varMAC_PMTCSR);
			pr_info("commonisr: PMTCSR : %#lx\n", varMAC_PMTCSR);
			if (pdata->power_down)
				DWC_ETH_QOS_powerup(pdata->dev, DWC_ETH_QOS_IOCTL_CONTEXT);
		}

		/* RGMII/SMII interrupt */
		if (GET_VALUE(varMAC_ISR, MAC_ISR_RGSMIIS_LPOS, MAC_ISR_RGSMIIS_HPOS) & 1) {
			MAC_PCS_RgRd(varMAC_PCS);
			printk(KERN_ALERT "RGMII/SMII interrupt: MAC_PCS = %#lx\n", varMAC_PCS);
#ifdef HWA_NV_1637630

#else
			/* Comment out this block of code(1637630)
	 		 * as it was preventing 10mb to work.
			 */
			if ((varMAC_PCS & 0x80000) == 0x80000) {
				pdata->pcs_link = 1;
				netif_carrier_on(dev);
				if ((varMAC_PCS & 0x10000) == 0x10000) {
					pdata->pcs_duplex = 1;
					hw_if->set_full_duplex(); //TODO: may not be required
				} else {
					pdata->pcs_duplex = 0;
					hw_if->set_half_duplex(); //TODO: may not be required
				}

				if ((varMAC_PCS & 0x60000) == 0x0) {
					pdata->pcs_speed = SPEED_10;
					hw_if->set_mii_speed_10(); //TODO: may not be required
				} else if ((varMAC_PCS & 0x60000) == 0x20000) {
					pdata->pcs_speed = SPEED_100;
					hw_if->set_mii_speed_100(); //TODO: may not be required
				} else if ((varMAC_PCS & 0x60000) == 0x30000) {
					pdata->pcs_speed = SPEED_1000;
					hw_if->set_gmii_speed(); //TODO: may not be required
				}
				printk(KERN_ALERT "Link is UP:%dMbps & %s duplex\n",
					pdata->pcs_speed, pdata->pcs_duplex ? "Full" : "Half");
			} else {
				printk(KERN_ALERT "Link is Down\n");
				pdata->pcs_link = 0;
				netif_carrier_off(dev);
			}
#endif
		}

		/* PCS Link Status interrupt */
		if (GET_VALUE(varMAC_ISR, MAC_ISR_PCSLCHGIS_LPOS, MAC_ISR_PCSLCHGIS_HPOS) & 1) {
			printk(KERN_ALERT "PCS Link Status interrupt\n");
			MAC_ANS_RgRd(varMAC_ANS);
			if (GET_VALUE(varMAC_ANS, MAC_ANS_LS_LPOS, MAC_ANS_LS_HPOS) & 1) {
				printk(KERN_ALERT "Link: Up\n");
				netif_carrier_on(dev);
				pdata->pcs_link = 1;
			} else {
				printk(KERN_ALERT "Link: Down\n");
				netif_carrier_off(dev);
				pdata->pcs_link = 0;
			}
		}

		/* PCS Auto-Negotiation Complete interrupt */
		if (GET_VALUE(varMAC_ISR, MAC_ISR_PCSANCIS_LPOS, MAC_ISR_PCSANCIS_HPOS) & 1) {
			printk(KERN_ALERT "PCS Auto-Negotiation Complete interrupt\n");
			MAC_ANS_RgRd(varMAC_ANS);
		}

		/* EEE interrupts */
		if (GET_VALUE(varMAC_ISR, MAC_ISR_LPI_LPOS, MAC_ISR_LPI_HPOS) & 1) {
			DWC_ETH_QOS_handle_eee_interrupt(pdata);
		}
	}

	DBGPR("<--DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS\n");

	return IRQ_HANDLED;

}

/*!
* \brief API to get all hw features.
*
* \details This function is used to check what are all the different
* features the device supports.
*
* \param[in] pdata - pointer to driver private structure
*
* \return none
*/

void DWC_ETH_QOS_get_all_hw_features(struct DWC_ETH_QOS_prv_data *pdata)
{
	unsigned int varMAC_HFR0;
	unsigned int varMAC_HFR1;
	unsigned int varMAC_HFR2;

	DBGPR("-->DWC_ETH_QOS_get_all_hw_features\n");

	MAC_HFR0_RgRd(varMAC_HFR0);
	MAC_HFR1_RgRd(varMAC_HFR1);
	MAC_HFR2_RgRd(varMAC_HFR2);

	memset(&pdata->hw_feat, 0, sizeof(pdata->hw_feat));
	pdata->hw_feat.mii_sel = ((varMAC_HFR0 >> 0) & MAC_HFR0_MIISEL_Mask);
	pdata->hw_feat.gmii_sel = ((varMAC_HFR0 >> 1) & MAC_HFR0_GMIISEL_Mask);
	pdata->hw_feat.hd_sel = ((varMAC_HFR0 >> 2) & MAC_HFR0_HDSEL_Mask);
	pdata->hw_feat.pcs_sel = ((varMAC_HFR0 >> 3) & MAC_HFR0_PCSSEL_Mask);
	pdata->hw_feat.vlan_hash_en =
	    ((varMAC_HFR0 >> 4) & MAC_HFR0_VLANHASEL_Mask);
	pdata->hw_feat.sma_sel = ((varMAC_HFR0 >> 5) & MAC_HFR0_SMASEL_Mask);
	pdata->hw_feat.rwk_sel = ((varMAC_HFR0 >> 6) & MAC_HFR0_RWKSEL_Mask);
	pdata->hw_feat.mgk_sel = ((varMAC_HFR0 >> 7) & MAC_HFR0_MGKSEL_Mask);
	pdata->hw_feat.mmc_sel = ((varMAC_HFR0 >> 8) & MAC_HFR0_MMCSEL_Mask);
	pdata->hw_feat.arp_offld_en =
	    ((varMAC_HFR0 >> 9) & MAC_HFR0_ARPOFFLDEN_Mask);
	pdata->hw_feat.ts_sel =
	    ((varMAC_HFR0 >> 12) & MAC_HFR0_TSSSEL_Mask);
	pdata->hw_feat.eee_sel = ((varMAC_HFR0 >> 13) & MAC_HFR0_EEESEL_Mask);
	pdata->hw_feat.tx_coe_sel =
	    ((varMAC_HFR0 >> 14) & MAC_HFR0_TXCOESEL_Mask);
	pdata->hw_feat.rx_coe_sel =
	    ((varMAC_HFR0 >> 16) & MAC_HFR0_RXCOE_Mask);
	pdata->hw_feat.mac_addr16_sel =
	    ((varMAC_HFR0 >> 18) & MAC_HFR0_ADDMACADRSEL_Mask);
	pdata->hw_feat.mac_addr32_sel =
	    ((varMAC_HFR0 >> 23) & MAC_HFR0_MACADR32SEL_Mask);
	pdata->hw_feat.mac_addr64_sel =
	    ((varMAC_HFR0 >> 24) & MAC_HFR0_MACADR64SEL_Mask);
	pdata->hw_feat.tsstssel =
	    ((varMAC_HFR0 >> 25) & MAC_HFR0_TSINTSEL_Mask);
	pdata->hw_feat.sa_vlan_ins =
	    ((varMAC_HFR0 >> 27) & MAC_HFR0_SAVLANINS_Mask);
	pdata->hw_feat.act_phy_sel =
	    ((varMAC_HFR0 >> 28) & MAC_HFR0_ACTPHYSEL_Mask);

	pdata->hw_feat.rx_fifo_size =
	    ((varMAC_HFR1 >> 0) & MAC_HFR1_RXFIFOSIZE_Mask);
	    //8;
	pdata->hw_feat.tx_fifo_size =
	    ((varMAC_HFR1 >> 6) & MAC_HFR1_TXFIFOSIZE_Mask);
	    //8;
	pdata->hw_feat.adv_ts_hword =
	    ((varMAC_HFR1 >> 13) & MAC_HFR1_ADVTHWORD_Mask);
	pdata->hw_feat.dcb_en = ((varMAC_HFR1 >> 16) & MAC_HFR1_DCBEN_Mask);
	pdata->hw_feat.sph_en = ((varMAC_HFR1 >> 17) & MAC_HFR1_SPHEN_Mask);
	pdata->hw_feat.tso_en = ((varMAC_HFR1 >> 18) & MAC_HFR1_TSOEN_Mask);
	pdata->hw_feat.dma_debug_gen =
	    ((varMAC_HFR1 >> 19) & MAC_HFR1_DMADEBUGEN_Mask);
	pdata->hw_feat.av_sel = ((varMAC_HFR1 >> 20) & MAC_HFR1_AVSEL_Mask);
	pdata->hw_feat.lp_mode_en =
	    ((varMAC_HFR1 >> 23) & MAC_HFR1_LPMODEEN_Mask);
	pdata->hw_feat.hash_tbl_sz =
	    ((varMAC_HFR1 >> 24) & MAC_HFR1_HASHTBLSZ_Mask);
	pdata->hw_feat.l3l4_filter_num =
	    ((varMAC_HFR1 >> 27) & MAC_HFR1_L3L4FILTERNUM_Mask);

	pdata->hw_feat.rx_q_cnt = ((varMAC_HFR2 >> 0) & MAC_HFR2_RXQCNT_Mask);
	pdata->hw_feat.tx_q_cnt = ((varMAC_HFR2 >> 6) & MAC_HFR2_TXQCNT_Mask);
	pdata->hw_feat.rx_ch_cnt =
	    ((varMAC_HFR2 >> 12) & MAC_HFR2_RXCHCNT_Mask);
	pdata->hw_feat.tx_ch_cnt =
	    ((varMAC_HFR2 >> 18) & MAC_HFR2_TXCHCNT_Mask);
	pdata->hw_feat.pps_out_num =
	    ((varMAC_HFR2 >> 24) & MAC_HFR2_PPSOUTNUM_Mask);
	pdata->hw_feat.aux_snap_num =
	    ((varMAC_HFR2 >> 28) & MAC_HFR2_AUXSNAPNUM_Mask);

	DBGPR("<--DWC_ETH_QOS_get_all_hw_features\n");
}


/*!
* \brief API to print all hw features.
*
* \details This function is used to print all the device feature.
*
* \param[in] pdata - pointer to driver private structure
*
* \return none
*/

void DWC_ETH_QOS_print_all_hw_features(struct DWC_ETH_QOS_prv_data *pdata)
{
	char *str = NULL;

	DBGPR("-->DWC_ETH_QOS_print_all_hw_features\n");

	printk(KERN_ALERT "\n");
	printk(KERN_ALERT "=====================================================/\n");
	printk(KERN_ALERT "\n");
	printk(KERN_ALERT "10/100 Mbps Support                         : %s\n",
		pdata->hw_feat.mii_sel ? "YES" : "NO");
	printk(KERN_ALERT "1000 Mbps Support                           : %s\n",
		pdata->hw_feat.gmii_sel ? "YES" : "NO");
	printk(KERN_ALERT "Half-duplex Support                         : %s\n",
		pdata->hw_feat.hd_sel ? "YES" : "NO");
	printk(KERN_ALERT "PCS Registers(TBI/SGMII/RTBI PHY interface) : %s\n",
		pdata->hw_feat.pcs_sel ? "YES" : "NO");
	printk(KERN_ALERT "VLAN Hash Filter Selected                   : %s\n",
		pdata->hw_feat.vlan_hash_en ? "YES" : "NO");
	pdata->vlan_hash_filtering = pdata->hw_feat.vlan_hash_en;
	printk(KERN_ALERT "SMA (MDIO) Interface                        : %s\n",
		pdata->hw_feat.sma_sel ? "YES" : "NO");
	printk(KERN_ALERT "PMT Remote Wake-up Packet Enable            : %s\n",
		pdata->hw_feat.rwk_sel ? "YES" : "NO");
	printk(KERN_ALERT "PMT Magic Packet Enable                     : %s\n",
		pdata->hw_feat.mgk_sel ? "YES" : "NO");
	printk(KERN_ALERT "RMON/MMC Module Enable                      : %s\n",
		pdata->hw_feat.mmc_sel ? "YES" : "NO");
	printk(KERN_ALERT "ARP Offload Enabled                         : %s\n",
		pdata->hw_feat.arp_offld_en ? "YES" : "NO");
	printk(KERN_ALERT "IEEE 1588-2008 Timestamp Enabled            : %s\n",
		pdata->hw_feat.ts_sel ? "YES" : "NO");
	printk(KERN_ALERT "Energy Efficient Ethernet Enabled           : %s\n",
		pdata->hw_feat.eee_sel ? "YES" : "NO");
	printk(KERN_ALERT "Transmit Checksum Offload Enabled           : %s\n",
		pdata->hw_feat.tx_coe_sel ? "YES" : "NO");
	printk(KERN_ALERT "Receive Checksum Offload Enabled            : %s\n",
		pdata->hw_feat.rx_coe_sel ? "YES" : "NO");
	printk(KERN_ALERT "MAC Addresses 16–31 Selected                : %s\n",
		pdata->hw_feat.mac_addr16_sel ? "YES" : "NO");
	printk(KERN_ALERT "MAC Addresses 32–63 Selected                : %s\n",
		pdata->hw_feat.mac_addr32_sel ? "YES" : "NO");
	printk(KERN_ALERT "MAC Addresses 64–127 Selected               : %s\n",
		pdata->hw_feat.mac_addr64_sel ? "YES" : "NO");

	if (pdata->hw_feat.mac_addr64_sel)
		pdata->max_addr_reg_cnt = 128;
	else if (pdata->hw_feat.mac_addr32_sel)
		pdata->max_addr_reg_cnt = 64;
	else if (pdata->hw_feat.mac_addr16_sel)
		pdata->max_addr_reg_cnt = 32;
	else
		pdata->max_addr_reg_cnt = 1;

	switch(pdata->hw_feat.tsstssel) {
	case 0:
		str = "RESERVED";
		break;
	case 1:
		str = "INTERNAL";
		break;
	case 2:
		str = "EXTERNAL";
		break;
	case 3:
		str = "BOTH";
		break;
	}
	printk(KERN_ALERT "Timestamp System Time Source                : %s\n",
		str);
	printk(KERN_ALERT "Source Address or VLAN Insertion Enable     : %s\n",
		pdata->hw_feat.sa_vlan_ins ? "YES" : "NO");

	switch (pdata->hw_feat.act_phy_sel) {
	case 0:
		str = "GMII/MII";
		break;
	case 1:
		str = "RGMII";
		break;
	case 2:
		str = "SGMII";
		break;
	case 3:
		str = "TBI";
		break;
	case 4:
		str = "RMII";
		break;
	case 5:
		str = "RTBI";
		break;
	case 6:
		str = "SMII";
		break;
	case 7:
		str = "RevMII";
		break;
	default:
		str = "RESERVED";
	}
	printk(KERN_ALERT "Active PHY Selected                         : %s\n",
		str);

	switch(pdata->hw_feat.rx_fifo_size) {
	case 0:
		str = "128 bytes";
		break;
	case 1:
		str = "256 bytes";
		break;
	case 2:
		str = "512 bytes";
		break;
	case 3:
		str = "1 KBytes";
		break;
	case 4:
		str = "2 KBytes";
		break;
	case 5:
		str = "4 KBytes";
		break;
	case 6:
		str = "8 KBytes";
		break;
	case 7:
		str = "16 KBytes";
		break;
	case 8:
		str = "32 kBytes";
		break;
	case 9:
		str = "64 KBytes";
		break;
	case 10:
		str = "128 KBytes";
		break;
	case 11:
		str = "256 KBytes";
		break;
	default:
		str = "RESERVED";
	}
	printk(KERN_ALERT "MTL Receive FIFO Size                       : %s\n",
		str);

	switch(pdata->hw_feat.tx_fifo_size) {
	case 0:
		str = "128 bytes";
		break;
	case 1:
		str = "256 bytes";
		break;
	case 2:
		str = "512 bytes";
		break;
	case 3:
		str = "1 KBytes";
		break;
	case 4:
		str = "2 KBytes";
		break;
	case 5:
		str = "4 KBytes";
		break;
	case 6:
		str = "8 KBytes";
		break;
	case 7:
		str = "16 KBytes";
		break;
	case 8:
		str = "32 kBytes";
		break;
	case 9:
		str = "64 KBytes";
		break;
	case 10:
		str = "128 KBytes";
		break;
	case 11:
		str = "256 KBytes";
		break;
	default:
		str = "RESERVED";
	}
	printk(KERN_ALERT "MTL Transmit FIFO Size                       : %s\n",
		str);
	printk(KERN_ALERT "IEEE 1588 High Word Register Enable          : %s\n",
		pdata->hw_feat.adv_ts_hword ? "YES" : "NO");
	printk(KERN_ALERT "DCB Feature Enable                           : %s\n",
		pdata->hw_feat.dcb_en ? "YES" : "NO");
	printk(KERN_ALERT "Split Header Feature Enable                  : %s\n",
		pdata->hw_feat.sph_en ? "YES" : "NO");
	printk(KERN_ALERT "TCP Segmentation Offload Enable              : %s\n",
		pdata->hw_feat.tso_en ? "YES" : "NO");
	printk(KERN_ALERT "DMA Debug Registers Enabled                  : %s\n",
		pdata->hw_feat.dma_debug_gen ? "YES" : "NO");
	printk(KERN_ALERT "AV Feature Enabled                           : %s\n",
		pdata->hw_feat.av_sel ? "YES" : "NO");
	printk(KERN_ALERT "Low Power Mode Enabled                       : %s\n",
		pdata->hw_feat.lp_mode_en ? "YES" : "NO");

	switch(pdata->hw_feat.hash_tbl_sz) {
	case 0:
		str = "No hash table selected";
		pdata->max_hash_table_size = 0;
		break;
	case 1:
		str = "64";
		pdata->max_hash_table_size = 64;
		break;
	case 2:
		str = "128";
		pdata->max_hash_table_size = 128;
		break;
	case 3:
		str = "256";
		pdata->max_hash_table_size = 256;
		break;
	}
	printk(KERN_ALERT "Hash Table Size                              : %s\n",
		str);
	printk(KERN_ALERT "Total number of L3 or L4 Filters             : %d L3/L4 Filter\n",
		pdata->hw_feat.l3l4_filter_num);
	printk(KERN_ALERT "Number of MTL Receive Queues                 : %d\n",
		(pdata->hw_feat.rx_q_cnt + 1));
	printk(KERN_ALERT "Number of MTL Transmit Queues                : %d\n",
		(pdata->hw_feat.tx_q_cnt + 1));
	printk(KERN_ALERT "Number of DMA Receive Channels               : %d\n",
		(pdata->hw_feat.rx_ch_cnt + 1));
	printk(KERN_ALERT "Number of DMA Transmit Channels              : %d\n",
		(pdata->hw_feat.tx_ch_cnt + 1));

	switch(pdata->hw_feat.pps_out_num) {
	case 0:
		str = "No PPS output";
		break;
	case 1:
		str = "1 PPS output";
		break;
	case 2:
		str = "2 PPS output";
		break;
	case 3:
		str = "3 PPS output";
		break;
	case 4:
		str = "4 PPS output";
		break;
	default:
		str = "RESERVED";
	}
	printk(KERN_ALERT "Number of PPS Outputs                        : %s\n",
		str);

	switch(pdata->hw_feat.aux_snap_num) {
	case 0:
		str = "No auxillary input";
		break;
	case 1:
		str = "1 auxillary input";
		break;
	case 2:
		str = "2 auxillary input";
		break;
	case 3:
		str = "3 auxillary input";
		break;
	case 4:
		str = "4 auxillary input";
		break;
	default:
		str = "RESERVED";
	}
	printk(KERN_ALERT "Number of Auxiliary Snapshot Inputs          : %s",
		str);

	printk(KERN_ALERT "\n");
	printk(KERN_ALERT "=====================================================/\n");

	DBGPR("<--DWC_ETH_QOS_print_all_hw_features\n");
}

/*!
 * \brief allcation of Rx skb's for split header feature.
 *
 * \details This function is invoked by other api's for
 * allocating the Rx skb's if split header feature is enabled.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] buffer – pointer to wrapper receive buffer data structure.
 * \param[in] gfp – the type of memory allocation.
 *
 * \return int
 *
 * \retval 0 on success and -ve number on failure.
 */

static int DWC_ETH_QOS_alloc_split_hdr_rx_buf(
		struct DWC_ETH_QOS_prv_data *pdata,
		struct DWC_ETH_QOS_rx_buffer *buffer,
		gfp_t gfp)
{
	struct sk_buff *skb = buffer->skb;

	DBGPR("-->DWC_ETH_QOS_alloc_split_hdr_rx_buf\n");

	if (skb) {
		skb_trim(skb, 0);
		goto check_page;
	}

	buffer->rx_hdr_size = DWC_ETH_QOS_MAX_HDR_SIZE;
	/* allocate twice the maximum header size */
	skb = __netdev_alloc_skb_ip_align(pdata->dev,
			(2*buffer->rx_hdr_size),
			gfp);
	if (skb == NULL) {
		printk(KERN_ALERT "Failed to allocate skb\n");
		return -ENOMEM;
	}
	buffer->skb = skb;
	DBGPR("Maximum header buffer size allocated = %d\n",
		buffer->rx_hdr_size);
 check_page:
	if (!buffer->dma)
		buffer->dma = dma_map_single(&pdata->pdev->dev,
					buffer->skb->data,
					(2*buffer->rx_hdr_size),
					DMA_FROM_DEVICE);
	buffer->len = buffer->rx_hdr_size;

	/* allocate a new page if necessary */
	if (buffer->page2 == NULL) {
		buffer->page2 = alloc_page(gfp);
		if (unlikely(!buffer->page2)) {
			printk(KERN_ALERT
			"Failed to allocate page for second buffer\n");
			return -ENOMEM;
		}
	}
	if (!buffer->dma2)
		buffer->dma2 = dma_map_page(&pdata->pdev->dev,
				    buffer->page2, 0,
				    PAGE_SIZE, DMA_FROM_DEVICE);
	buffer->len2 = PAGE_SIZE;
	buffer->mapped_as_page = Y_TRUE;

	DBGPR("<--DWC_ETH_QOS_alloc_split_hdr_rx_buf\n");

	return 0;
}

/*!
 * \brief allcation of Rx skb's for jumbo frame.
 *
 * \details This function is invoked by other api's for
 * allocating the Rx skb's if jumbo frame is enabled.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] buffer – pointer to wrapper receive buffer data structure.
 * \param[in] gfp – the type of memory allocation.
 *
 * \return int
 *
 * \retval 0 on success and -ve number on failure.
 */

static int DWC_ETH_QOS_alloc_jumbo_rx_buf(struct DWC_ETH_QOS_prv_data *pdata,
					  struct DWC_ETH_QOS_rx_buffer *buffer,
					  gfp_t gfp)
{
	struct sk_buff *skb = buffer->skb;
	unsigned int bufsz = (256 - 16);	/* for skb_reserve */

	DBGPR("-->DWC_ETH_QOS_alloc_jumbo_rx_buf\n");

	if (skb) {
		skb_trim(skb, 0);
		goto check_page;
	}

	skb = __netdev_alloc_skb_ip_align(pdata->dev, bufsz, gfp);
	if (skb == NULL) {
		printk(KERN_ALERT "Failed to allocate skb\n");
		return -ENOMEM;
	}
	buffer->skb = skb;
 check_page:
	/* allocate a new page if necessary */
	if (buffer->page == NULL) {
		buffer->page = alloc_page(gfp);
		if (unlikely(!buffer->page)) {
			printk(KERN_ALERT "Failed to allocate page\n");
			return -ENOMEM;
		}
	}
	if (!buffer->dma)
		buffer->dma = dma_map_page(&pdata->pdev->dev,
					   buffer->page, 0,
					   PAGE_SIZE, DMA_FROM_DEVICE);
	buffer->len = PAGE_SIZE;

	if (buffer->page2 == NULL) {
		buffer->page2 = alloc_page(gfp);
		if (unlikely(!buffer->page2)) {
			printk(KERN_ALERT
			       "Failed to allocate page for second buffer\n");
			return -ENOMEM;
		}
	}
	if (!buffer->dma2)
		buffer->dma2 = dma_map_page(&pdata->pdev->dev,
					    buffer->page2, 0,
					    PAGE_SIZE, DMA_FROM_DEVICE);
	buffer->len2 = PAGE_SIZE;

	buffer->mapped_as_page = Y_TRUE;

	DBGPR("<--DWC_ETH_QOS_alloc_jumbo_rx_buf\n");

	return 0;
}

/*!
 * \brief allcation of Rx skb's for default rx mode.
 *
 * \details This function is invoked by other api's for
 * allocating the Rx skb's with default Rx mode ie non-jumbo
 * and non-split header mode.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] buffer – pointer to wrapper receive buffer data structure.
 * \param[in] gfp – the type of memory allocation.
 *
 * \return int
 *
 * \retval 0 on success and -ve number on failure.
 */

static int DWC_ETH_QOS_alloc_rx_buf(struct DWC_ETH_QOS_prv_data *pdata,
				    struct DWC_ETH_QOS_rx_buffer *buffer,
				    gfp_t gfp)
{
	struct sk_buff *skb = buffer->skb;

	DBGPR("-->DWC_ETH_QOS_alloc_rx_buf\n");

	if (skb) {
		skb_trim(skb, 0);
		goto map_skb;
	}

	skb = __netdev_alloc_skb_ip_align(pdata->dev, pdata->rx_buffer_len, gfp);
	if (skb == NULL) {
		printk(KERN_ALERT "Failed to allocate skb\n");
		return -ENOMEM;
	}
	buffer->skb = skb;
	buffer->len = pdata->rx_buffer_len;
 map_skb:
	buffer->dma = dma_map_single(&pdata->pdev->dev, skb->data,
				     pdata->rx_buffer_len, DMA_FROM_DEVICE);
	if (dma_mapping_error(&pdata->pdev->dev, buffer->dma))
		printk(KERN_ALERT "failed to do the RX dma map\n");

	buffer->mapped_as_page = Y_FALSE;

	DBGPR("<--DWC_ETH_QOS_alloc_rx_buf\n");

	return 0;
}


/*!
 * \brief api to configure Rx function pointer after reset.
 *
 * \details This function will initialize the receive function pointers
 * which are used for allocating skb's and receiving the packets based
 * Rx mode - default/jumbo/split header.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void DWC_ETH_QOS_configure_rx_fun_ptr(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR("-->DWC_ETH_QOS_configure_rx_fun_ptr\n");

	if (pdata->rx_split_hdr) {
		pdata->clean_rx = DWC_ETH_QOS_clean_split_hdr_rx_irq;
		pdata->alloc_rx_buf = DWC_ETH_QOS_alloc_split_hdr_rx_buf;
	}
	else if (pdata->dev->mtu > DWC_ETH_QOS_ETH_FRAME_LEN) {
		pdata->clean_rx = DWC_ETH_QOS_clean_jumbo_rx_irq;
		pdata->alloc_rx_buf = DWC_ETH_QOS_alloc_jumbo_rx_buf;
	} else {
		pdata->rx_buffer_len = DWC_ETH_QOS_ETH_FRAME_LEN;
		pdata->clean_rx = DWC_ETH_QOS_clean_rx_irq;
		pdata->alloc_rx_buf = DWC_ETH_QOS_alloc_rx_buf;
	}

	DBGPR("<--DWC_ETH_QOS_configure_rx_fun_ptr\n");
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

static void DWC_ETH_QOS_default_common_confs(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR("-->DWC_ETH_QOS_default_common_confs\n");

	pdata->drop_tx_pktburstcnt = 1;
	pdata->mac_enable_count = 0;
	pdata->incr_incrx = DWC_ETH_QOS_INCR_ENABLE;
	pdata->flow_ctrl = DWC_ETH_QOS_FLOW_CTRL_TX_RX;
	pdata->oldflow_ctrl = DWC_ETH_QOS_FLOW_CTRL_TX_RX;
	pdata->power_down = 0;
	pdata->tx_sa_ctrl_via_desc = DWC_ETH_QOS_SA0_NONE;
	pdata->tx_sa_ctrl_via_reg = DWC_ETH_QOS_SA0_NONE;
	pdata->hwts_tx_en = 0;
	pdata->hwts_rx_en = 0;
	pdata->l3_l4_filter = 0;
	pdata->l2_filtering_mode = !!pdata->hw_feat.hash_tbl_sz;
	pdata->tx_path_in_lpi_mode = 0;
	pdata->use_lpi_tx_automate = true;
	pdata->eee_active = 0;
	pdata->one_nsec_accuracy = 1;

	DBGPR("<--DWC_ETH_QOS_default_common_confs\n");
}


/*!
 * \brief api to initialize Tx parameters.
 *
 * \details This function is used to initialize all Tx
 * parameters to default values on reset.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] qInx – DMA channel/queue number to be initialized.
 *
 * \return void
 */

static void DWC_ETH_QOS_default_tx_confs_single_q(
		struct DWC_ETH_QOS_prv_data *pdata,
		UINT qInx)
{
	struct DWC_ETH_QOS_tx_queue *queue_data = GET_TX_QUEUE_PTR(qInx);
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data =
		GET_TX_WRAPPER_DESC(qInx);

	DBGPR("-->DWC_ETH_QOS_default_tx_confs_single_q\n");

	queue_data->q_op_mode = q_op_mode[qInx];

	desc_data->tx_threshold_val = DWC_ETH_QOS_TX_THRESHOLD_32;
	desc_data->tsf_on = DWC_ETH_QOS_TSF_ENABLE;
	desc_data->osf_on = DWC_ETH_QOS_OSF_ENABLE;
	desc_data->tx_pbl = DWC_ETH_QOS_PBL_16;
	desc_data->tx_vlan_tag_via_reg = Y_FALSE;
	desc_data->tx_vlan_tag_ctrl = DWC_ETH_QOS_TX_VLAN_TAG_INSERT;
	desc_data->vlan_tag_present = 0;
	desc_data->context_setup = 0;
	desc_data->default_mss = 0;

	DBGPR("<--DWC_ETH_QOS_default_tx_confs_single_q\n");
}


/*!
 * \brief api to initialize Rx parameters.
 *
 * \details This function is used to initialize all Rx
 * parameters to default values on reset.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] qInx – DMA queue/channel number to be initialized.
 *
 * \return void
 */

static void DWC_ETH_QOS_default_rx_confs_single_q(
		struct DWC_ETH_QOS_prv_data *pdata,
		UINT qInx)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data =
		GET_RX_WRAPPER_DESC(qInx);

	DBGPR("-->DWC_ETH_QOS_default_rx_confs_single_q\n");

	desc_data->rx_threshold_val = DWC_ETH_QOS_RX_THRESHOLD_64;
	desc_data->rsf_on = DWC_ETH_QOS_RSF_DISABLE;
	desc_data->rx_pbl = DWC_ETH_QOS_PBL_16;
	desc_data->rx_outer_vlan_strip = DWC_ETH_QOS_RX_VLAN_STRIP_ALWAYS;
	desc_data->rx_inner_vlan_strip = DWC_ETH_QOS_RX_VLAN_STRIP_ALWAYS;

	DBGPR("<--DWC_ETH_QOS_default_rx_confs_single_q\n");
}

static void DWC_ETH_QOS_default_tx_confs(struct DWC_ETH_QOS_prv_data *pdata)
{
	UINT qInx;

	DBGPR("-->DWC_ETH_QOS_default_tx_confs\n");

	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		DWC_ETH_QOS_default_tx_confs_single_q(pdata, qInx);
	}

	DBGPR("<--DWC_ETH_QOS_default_tx_confs\n");
}

static void DWC_ETH_QOS_default_rx_confs(struct DWC_ETH_QOS_prv_data *pdata)
{
	UINT qInx;

	DBGPR("-->DWC_ETH_QOS_default_rx_confs\n");

	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++) {
		DWC_ETH_QOS_default_rx_confs_single_q(pdata, qInx);
	}

	DBGPR("<--DWC_ETH_QOS_default_rx_confs\n");
}


/*!
* \brief API to open a deivce for data transmission & reception.
*
* \details Opens the interface. The interface is opned whenever
* ifconfig activates it. The open method should register any
* system resource it needs like I/O ports, IRQ, DMA, etc,
* turn on the hardware, and perform any other setup your device requires.
*
* \param[in] dev - pointer to net_device structure
*
* \return integer
*
* \retval 0 on success & negative number on failure.
*/

static int DWC_ETH_QOS_open(struct net_device *dev)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	int ret = Y_SUCCESS;
	struct hw_if_struct *hw_if = &pdata->hw_if;
	struct desc_if_struct *desc_if = &pdata->desc_if;

#ifdef HWA_NV_1617267
	ULONG lo, hi;
#endif
	DBGPR("-->DWC_ETH_QOS_open\n");

#ifdef HWA_NV_1617267

	/* temp code for bringup only.  Ensure mac address is not default */
	MAC_MA0LR_RgRd(lo);
	MAC_MA0HR_RgRd(hi);
	if ((lo == 0xffffffff) && (hi == 0x8000ffff)) {
		printk(KERN_ALERT
	       		"%s(): ERROR-MAC address needs to be changed\n",
			 __func__);
		return  -EINVAL;
	}

	/* save mac addr in sw copy.  Later in this execution thread
	 * configure_mac() is called, and it will program hw mac_addr0 with 
	 * dev_addr[].
	 */
	pdata->dev->dev_addr[0] = (lo & 0xff);
	pdata->dev->dev_addr[1] = ((lo >> 8) & 0xff);
	pdata->dev->dev_addr[2] = ((lo >> 16) & 0xff);
	pdata->dev->dev_addr[3] = ((lo >> 24) & 0xff);
	pdata->dev->dev_addr[4] = (hi & 0xff);
	pdata->dev->dev_addr[5] = ((hi >> 8) & 0xff);
#endif
	pdata->irq_number = dev->irq;
#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	ret = request_irq(pdata->irq_number, DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS_pg,
			  IRQF_SHARED, DEV_NAME, pdata);
#else
	ret = request_irq(pdata->irq_number, DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS,
			  IRQF_SHARED, DEV_NAME, pdata);
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */
	if (ret != 0) {
		printk(KERN_ALERT "Unable to register IRQ %d\n",
		       pdata->irq_number);
		ret = -EBUSY;
		goto err_irq_0;
	}
	ret = desc_if->alloc_buff_and_desc(pdata);
	if (ret < 0) {
		printk(KERN_ALERT
		       "failed to allocate buffer/descriptor memory\n");
		ret = -ENOMEM;
		goto err_out_desc_buf_alloc_failed;
	}

	/* default configuration */
#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	DWC_ETH_QOS_default_confs(pdata);
#else
	DWC_ETH_QOS_default_common_confs(pdata);
	DWC_ETH_QOS_default_tx_confs(pdata);
	DWC_ETH_QOS_default_rx_confs(pdata);
	DWC_ETH_QOS_configure_rx_fun_ptr(pdata);

	DWC_ETH_QOS_napi_enable_mq(pdata);
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */

	DWC_ETH_QOS_set_rx_mode(dev);
	desc_if->wrapper_tx_desc_init(pdata);
	desc_if->wrapper_rx_desc_init(pdata);

	DWC_ETH_QOS_tx_desc_mang_ds_dump(pdata);
	DWC_ETH_QOS_rx_desc_mang_ds_dump(pdata);

	DWC_ETH_QOS_mmc_setup(pdata);

	/* initializes MAC and DMA */
	hw_if->init(pdata);

	if (pdata->hw_feat.pcs_sel)
		hw_if->control_an(1, 0);

#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	hw_if->prepare_dev_pktgen(pdata);
#endif

	if (pdata->phydev)
		phy_start(pdata->phydev);

#ifdef HWA_NV_1618922
	pdata->eee_enabled = false;
#else
	pdata->eee_enabled = DWC_ETH_QOS_eee_init(pdata);
#endif

#ifndef DWC_ETH_QOS_CONFIG_PGTEST
	if (pdata->phydev)
		netif_tx_start_all_queues(dev);
#else
	netif_tx_disable(dev);
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */

	DBGPR("<--DWC_ETH_QOS_open\n");

	return ret;

 err_out_desc_buf_alloc_failed:
	free_irq(pdata->irq_number, pdata);
	pdata->irq_number = 0;

 err_irq_0:
	DBGPR("<--DWC_ETH_QOS_open\n");
	return ret;
}

/*!
* \brief API to close a device.
*
* \details Stops the interface. The interface is stopped when it is brought
* down. This function should reverse operations performed at open time.
*
* \param[in] dev - pointer to net_device structure
*
* \return integer
*
* \retval 0 on success & negative number on failure.
*/

static int DWC_ETH_QOS_close(struct net_device *dev)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &pdata->hw_if;
	struct desc_if_struct *desc_if = &pdata->desc_if;

	DBGPR("-->DWC_ETH_QOS_close\n");

	if (pdata->eee_enabled)
		del_timer_sync(&pdata->eee_ctrl_timer);

	if (pdata->phydev)
		phy_stop(pdata->phydev);

#ifndef DWC_ETH_QOS_CONFIG_PGTEST
	netif_tx_disable(dev);
	DWC_ETH_QOS_all_ch_napi_disable(pdata);
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */

	/* issue software reset to device */
	hw_if->exit();

#ifdef HWA_NV_1617267
	/* Temp hack.  Preserve MAC address so user does not have to 
  	 * reprogram it.
	 */
	MAC_MA0HR_RgWr(((pdata->dev->dev_addr[5] << 8) |
			(pdata->dev->dev_addr[4])));
	MAC_MA0LR_RgWr(((pdata->dev->dev_addr[3] << 24) |
			(pdata->dev->dev_addr[2] << 16) |
			(pdata->dev->dev_addr[1] << 8) |
			(pdata->dev->dev_addr[0])));
#endif

	desc_if->tx_free_mem(pdata);
	desc_if->rx_free_mem(pdata);
	if (pdata->irq_number != 0) {
		free_irq(pdata->irq_number, pdata);
		pdata->irq_number = 0;
	}
#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	del_timer(&pdata->pg_timer);
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */

	DBGPR("<--DWC_ETH_QOS_close\n");

	return Y_SUCCESS;
}


/*!
* \brief API to configure the multicast address in device.
*
* \details This function collects all the multicast addresse
* and updates the device.
*
* \param[in] dev - pointer to net_device structure.
*
* \retval 0 if perfect filtering is seleted & 1 if hash
* filtering is seleted.
*/
static int DWC_ETH_QOS_prepare_mc_list(struct net_device *dev)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	u32 mc_filter[DWC_ETH_QOS_HTR_CNT];
	struct netdev_hw_addr *ha = NULL;
	int crc32_val = 0;
	int ret = 0, i = 1;

	DBGPR_FILTER("-->DWC_ETH_QOS_prepare_mc_list\n");

	if (pdata->l2_filtering_mode) {
		DBGPR_FILTER("select HASH FILTERING for mc addresses: mc_count = %d\n",
				netdev_mc_count(dev));
		ret = 1;
		memset(mc_filter, 0, sizeof(mc_filter));

		if (pdata->max_hash_table_size == 64) {
			netdev_for_each_mc_addr(ha, dev) {
				DBGPR_FILTER("mc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",i++,
						ha->addr[0], ha->addr[1], ha->addr[2],
						ha->addr[3], ha->addr[4], ha->addr[5]);
				/* The upper 6 bits of the calculated CRC are used to
				 * index the content of the Hash Table Reg 0 and 1.
				 * */
				crc32_val =
					(bitrev32(~crc32_le(~0, ha->addr, 6)) >> 26);
				/* The most significant bit determines the register
				 * to use (Hash Table Reg X, X = 0 and 1) while the
				 * other 5(0x1F) bits determines the bit within the
				 * selected register
				 * */
				mc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
			}
		} else if (pdata->max_hash_table_size == 128) {
			netdev_for_each_mc_addr(ha, dev) {
				DBGPR_FILTER("mc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",i++,
						ha->addr[0], ha->addr[1], ha->addr[2],
						ha->addr[3], ha->addr[4], ha->addr[5]);
				/* The upper 7 bits of the calculated CRC are used to
				 * index the content of the Hash Table Reg 0,1,2 and 3.
				 * */
				crc32_val =
					(bitrev32(~crc32_le(~0, ha->addr, 6)) >> 25);

				printk(KERN_ALERT "crc_le = %#x, crc_be = %#x\n",
						bitrev32(~crc32_le(~0, ha->addr, 6)),
						bitrev32(~crc32_be(~0, ha->addr, 6)));

				/* The most significant 2 bits determines the register
				 * to use (Hash Table Reg X, X = 0,1,2 and 3) while the
				 * other 5(0x1F) bits determines the bit within the
				 * selected register
				 * */
				mc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
			}
		} else if (pdata->max_hash_table_size == 256) {
			netdev_for_each_mc_addr(ha, dev) {
				DBGPR_FILTER("mc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",i++,
						ha->addr[0], ha->addr[1], ha->addr[2],
						ha->addr[3], ha->addr[4], ha->addr[5]);
				/* The upper 8 bits of the calculated CRC are used to
				 * index the content of the Hash Table Reg 0,1,2,3,4,
				 * 5,6, and 7.
				 * */
				crc32_val =
					(bitrev32(~crc32_le(~0, ha->addr, 6)) >> 24);
				/* The most significant 3 bits determines the register
				 * to use (Hash Table Reg X, X = 0,1,2,3,4,5,6 and 7) while
				 * the other 5(0x1F) bits determines the bit within the
				 * selected register
				 * */
				mc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
			}
		}

		for (i = 0; i < DWC_ETH_QOS_HTR_CNT; i++)
			hw_if->update_hash_table_reg(i, mc_filter[i]);

	} else {
		DBGPR_FILTER("select PERFECT FILTERING for mc addresses, mc_count = %d, max_addr_reg_cnt = %d\n",
				netdev_mc_count(dev), pdata->max_addr_reg_cnt);

		netdev_for_each_mc_addr(ha, dev) {
			DBGPR_FILTER("mc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n", i,
					ha->addr[0], ha->addr[1], ha->addr[2],
					ha->addr[3], ha->addr[4], ha->addr[5]);
			if (i < 32)
				hw_if->update_mac_addr1_31_low_high_reg(i, ha->addr);
			else
				hw_if->update_mac_addr32_127_low_high_reg(i, ha->addr);
			i++;
		}
	}

	DBGPR_FILTER("<--DWC_ETH_QOS_prepare_mc_list\n");

	return ret;
}

/*!
* \brief API to configure the unicast address in device.
*
* \details This function collects all the unicast addresses
* and updates the device.
*
* \param[in] dev - pointer to net_device structure.
*
* \retval 0 if perfect filtering is seleted  & 1 if hash
* filtering is seleted.
*/
static int DWC_ETH_QOS_prepare_uc_list(struct net_device *dev)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	u32 uc_filter[DWC_ETH_QOS_HTR_CNT];
	struct netdev_hw_addr *ha = NULL;
	int crc32_val = 0;
	int ret = 0, i = 1;

	DBGPR_FILTER("-->DWC_ETH_QOS_prepare_uc_list\n");

	if (pdata->l2_filtering_mode) {
		DBGPR_FILTER("select HASH FILTERING for uc addresses: uc_count = %d\n",
				netdev_uc_count(dev));
		ret = 1;
		memset(uc_filter, 0, sizeof(uc_filter));

		if (pdata->max_hash_table_size == 64) {
			netdev_for_each_uc_addr(ha, dev) {
				DBGPR_FILTER("uc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",i++,
						ha->addr[0], ha->addr[1], ha->addr[2],
						ha->addr[3], ha->addr[4], ha->addr[5]);
				crc32_val =
					(bitrev32(~crc32_le(~0, ha->addr, 6)) >> 26);
				uc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
			}
		} else if (pdata->max_hash_table_size == 128) {
			netdev_for_each_uc_addr(ha, dev) {
				DBGPR_FILTER("uc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",i++,
						ha->addr[0], ha->addr[1], ha->addr[2],
						ha->addr[3], ha->addr[4], ha->addr[5]);
				crc32_val =
					(bitrev32(~crc32_le(~0, ha->addr, 6)) >> 25);
				uc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
			}
		} else if (pdata->max_hash_table_size == 256) {
			netdev_for_each_uc_addr(ha, dev) {
				DBGPR_FILTER("uc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",i++,
						ha->addr[0], ha->addr[1], ha->addr[2],
						ha->addr[3], ha->addr[4], ha->addr[5]);
				crc32_val =
					(bitrev32(~crc32_le(~0, ha->addr, 6)) >> 24);
				uc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
			}
		}

		/* configure hash value of real/default interface also */
		DBGPR_FILTER("real/default dev_addr = %#x:%#x:%#x:%#x:%#x:%#x\n",
				dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
				dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

		if (pdata->max_hash_table_size == 64) {
			crc32_val =
				(bitrev32(~crc32_le(~0, dev->dev_addr, 6)) >> 26);
			uc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
		} else if (pdata->max_hash_table_size == 128) {
			crc32_val =
				(bitrev32(~crc32_le(~0, dev->dev_addr, 6)) >> 25);
			uc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));

		} else if (pdata->max_hash_table_size == 256) {
			crc32_val =
				(bitrev32(~crc32_le(~0, dev->dev_addr, 6)) >> 24);
			uc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
		}

		for (i = 0; i < DWC_ETH_QOS_HTR_CNT; i++)
			hw_if->update_hash_table_reg(i, uc_filter[i]);

	} else {
		DBGPR_FILTER("select PERFECT FILTERING for uc addresses: uc_count = %d\n",
				netdev_uc_count(dev));

		netdev_for_each_uc_addr(ha, dev) {
			DBGPR_FILTER("uc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n", i,
					ha->addr[0], ha->addr[1], ha->addr[2],
					ha->addr[3], ha->addr[4], ha->addr[5]);
			if (i < 32)
				hw_if->update_mac_addr1_31_low_high_reg(i, ha->addr);
			else
				hw_if->update_mac_addr32_127_low_high_reg(i, ha->addr);
			i++;
		}
	}

	DBGPR_FILTER("<--DWC_ETH_QOS_prepare_uc_list\n");

	return ret;
}

/*!
* \brief API to set the device receive mode
*
* \details The set_multicast_list function is called when the multicast list
* for the device changes and when the flags change.
*
* \param[in] dev - pointer to net_device structure.
*
* \return void
*/
static void DWC_ETH_QOS_set_rx_mode(struct net_device *dev)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned long flags;
	unsigned char pr_mode = 0;
	unsigned char huc_mode = 0;
	unsigned char hmc_mode = 0;
	unsigned char pm_mode = 0;
	unsigned char hpf_mode = 0;
	int mode, i;

	DBGPR_FILTER("-->DWC_ETH_QOS_set_rx_mode\n");

	spin_lock_irqsave(&pdata->lock, flags);

#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	DBGPR("PG Test running, no parameters will be changed\n");
	spin_unlock_irqrestore(&pdata->lock, flags);
	return;
#endif

	if (dev->flags & IFF_PROMISC) {
		DBGPR_FILTER("PROMISCUOUS MODE (Accept all packets irrespective of DA)\n");
		pr_mode = 1;
	} else if ((dev->flags & IFF_ALLMULTI) ||
			(netdev_mc_count(dev) > (pdata->max_hash_table_size))) {
		DBGPR_FILTER("pass all multicast pkt\n");
		pm_mode = 1;
		if (pdata->max_hash_table_size) {
			for (i = 0; i < DWC_ETH_QOS_HTR_CNT; i++)
				hw_if->update_hash_table_reg(i, 0xffffffff);
		}
	} else if (!netdev_mc_empty(dev)) {
		DBGPR_FILTER("pass list of multicast pkt\n");
		if ((netdev_mc_count(dev) > (pdata->max_addr_reg_cnt - 1)) &&
			(!pdata->max_hash_table_size)) {
			/* switch to PROMISCUOUS mode */
			pr_mode = 1;
		} else {
			mode = DWC_ETH_QOS_prepare_mc_list(dev);
			if (mode) {
				/* Hash filtering for multicast */
				hmc_mode = 1;
			} else {
				/* Perfect filtering for multicast */
				hmc_mode = 0;
				hpf_mode = 1;
			}
		}
	}

	/* Handle multiple unicast addresses */
	if ((netdev_uc_count(dev) > (pdata->max_addr_reg_cnt - 1)) &&
			(!pdata->max_hash_table_size)) {
		/* switch to PROMISCUOUS mode */
		pr_mode = 1;
	} else if (!netdev_uc_empty(dev)) {
		mode = DWC_ETH_QOS_prepare_uc_list(dev);
		if (mode) {
			/* Hash filtering for unicast */
			huc_mode = 1;
		} else {
			/* Perfect filtering for unicast */
			huc_mode = 0;
			hpf_mode = 1;
		}
	}

	if (!SIM_WORLD)
	hw_if->config_mac_pkt_filter_reg(pr_mode, huc_mode,
		hmc_mode, pm_mode, hpf_mode);

	spin_unlock_irqrestore(&pdata->lock, flags);

	DBGPR("<--DWC_ETH_QOS_set_rx_mode\n");
}

/*!
* \brief API to calculate number of descriptor.
*
* \details This function is invoked by start_xmit function. This function
* calculates number of transmit descriptor required for a given transfer.
*
* \param[in] pdata - pointer to private data structure
* \param[in] skb - pointer to sk_buff structure
* \param[in] qInx - Queue number.
*
* \return integer
*
* \retval number of descriptor required.
*/

UINT DWC_ETH_QOS_get_total_desc_cnt(struct DWC_ETH_QOS_prv_data *pdata,
		struct sk_buff *skb, UINT qInx)
{
	UINT count = 0, size = 0;
	INT length = 0;
#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	struct hw_if_struct *hw_if = &pdata->hw_if;
	struct s_tx_pkt_features *tx_pkt_features = GET_TX_PKT_FEATURES_PTR;
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data =
	    GET_TX_WRAPPER_DESC(qInx);
#endif

	/* SG fragment count */
	count += skb_shinfo(skb)->nr_frags;

	/* descriptors required based on data limit per descriptor */
	length = (skb->len - skb->data_len);
	while (length) {
		size = min(length, DWC_ETH_QOS_MAX_DATA_PER_TXD);
		count++;
		length = length - size;
	}

	/* we need one context descriptor to carry tso details */
	if (skb_shinfo(skb)->gso_size != 0)
		count++;

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	desc_data->vlan_tag_present = 0;
	if (vlan_tx_tag_present(skb)) {
		USHORT vlan_tag = vlan_tx_tag_get(skb);
		vlan_tag |= (qInx << 13);
		desc_data->vlan_tag_present = 1;
		if (vlan_tag != desc_data->vlan_tag_id ||
				desc_data->context_setup == 1) {
			desc_data->vlan_tag_id = vlan_tag;
			if (Y_TRUE == desc_data->tx_vlan_tag_via_reg) {
				printk(KERN_ALERT "VLAN control info update via register\n\n");
				hw_if->enable_vlan_reg_control(desc_data);
			} else {
				hw_if->enable_vlan_desc_control(pdata);
				TX_PKT_FEATURES_PKT_ATTRIBUTES_VLAN_PKT_Mlf_Wr
				    (tx_pkt_features->pkt_attributes, 1);
				TX_PKT_FEATURES_VLAN_TAG_VT_Mlf_Wr
					(tx_pkt_features->vlan_tag, vlan_tag);
				/* we need one context descriptor to carry vlan tag info */
				count++;
			}
		}
		pdata->xstats.tx_vlan_pkt_n++;
	}
#endif
#ifdef DWC_ETH_QOS_ENABLE_DVLAN
	if (pdata->via_reg_or_desc == DWC_ETH_QOS_VIA_DESC) {
		/* we need one context descriptor to carry vlan tag info */
		count++;
	}
#endif /* End of DWC_ETH_QOS_ENABLE_DVLAN */

	return count;
}


/*!
* \brief API to transmit the packets
*
* \details The start_xmit function initiates the transmission of a packet.
* The full packet (protocol headers and all) is contained in a socket buffer
* (sk_buff) structure.
*
* \param[in] skb - pointer to sk_buff structure
* \param[in] dev - pointer to net_device structure
*
* \return integer
*
* \retval 0
*/

static int DWC_ETH_QOS_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	UINT qInx = skb_get_queue_mapping(skb);
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data =
	    GET_TX_WRAPPER_DESC(qInx);
	struct s_tx_pkt_features *tx_pkt_features = GET_TX_PKT_FEATURES_PTR;
	unsigned long flags;
	unsigned int desc_count = 0;
	unsigned int count = 0;
	struct hw_if_struct *hw_if = &pdata->hw_if;
	struct desc_if_struct *desc_if = &pdata->desc_if;
	INT retval = NETDEV_TX_OK;
#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	UINT varvlan_pkt;
#endif
	int tso;

	DBGPR("-->DWC_ETH_QOS_start_xmit: skb->len = %d, qInx = %u\n",
		skb->len, qInx);

	spin_lock_irqsave(&pdata->tx_lock, flags);

#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	retval = NETDEV_TX_BUSY;
	goto tx_netdev_return;
#endif

	if (skb->len <= 0) {
		dev_kfree_skb_any(skb);
		printk(KERN_ERR "%s : Empty skb received from stack\n",
			dev->name);
		goto tx_netdev_return;
	}

	if ((skb_shinfo(skb)->gso_size == 0) &&
		(skb->len > DWC_ETH_QOS_MAX_SUPPORTED_MTU)) {
		printk(KERN_ERR "%s : big packet = %d\n", dev->name,
			(u16)skb->len);
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		goto tx_netdev_return;
	}

	if ((pdata->eee_enabled) && (pdata->tx_path_in_lpi_mode) &&
		(!pdata->use_lpi_tx_automate))
		DWC_ETH_QOS_disable_eee_mode(pdata);

	memset(&pdata->tx_pkt_features, 0, sizeof(pdata->tx_pkt_features));

	/* check total number of desc required for current xfer */
	desc_count = DWC_ETH_QOS_get_total_desc_cnt(pdata, skb, qInx);
	if (desc_data->free_desc_cnt < desc_count) {
		desc_data->queue_stopped = 1;
		netif_stop_subqueue(dev, qInx);
		DBGPR("stopped TX queue(%d) since there are no sufficient "
			"descriptor available for the current transfer\n",
			qInx);
		retval = NETDEV_TX_BUSY;
		goto tx_netdev_return;
	}

	/* check for hw tstamping */
	if (pdata->hw_feat.tsstssel && pdata->hwts_tx_en) {
		if(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) {
			/* declare that device is doing timestamping */
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			TX_PKT_FEATURES_PKT_ATTRIBUTES_PTP_ENABLE_Mlf_Wr(tx_pkt_features->pkt_attributes, 1);
			DBGPR_PTP("Got PTP pkt to transmit [qInx = %d, cur_tx = %d]\n",
				qInx, desc_data->cur_tx);
		}
	}

	tso = desc_if->handle_tso(dev, skb);
	if (tso < 0) {
		printk(KERN_ALERT "Unable to handle TSO\n");
		dev_kfree_skb_any(skb);
		retval = NETDEV_TX_OK;
		goto tx_netdev_return;
	}
	if (tso) {
		pdata->xstats.tx_tso_pkt_n++;
		TX_PKT_FEATURES_PKT_ATTRIBUTES_TSO_ENABLE_Mlf_Wr(tx_pkt_features->pkt_attributes, 1);
	} else if (skb->ip_summed == CHECKSUM_PARTIAL) {
		TX_PKT_FEATURES_PKT_ATTRIBUTES_CSUM_ENABLE_Mlf_Wr(tx_pkt_features->pkt_attributes, 1);
	}

	count = desc_if->map_tx_skb(dev, skb);
	if (count == 0) {
		dev_kfree_skb_any(skb);
		retval = NETDEV_TX_OK;
		goto tx_netdev_return;
	}

	desc_data->packet_count = count;

	if (tso && (desc_data->default_mss != tx_pkt_features->mss))
		count++;

	dev->trans_start = jiffies;

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	TX_PKT_FEATURES_PKT_ATTRIBUTES_VLAN_PKT_Mlf_Rd
		(tx_pkt_features->pkt_attributes, varvlan_pkt);
	if (varvlan_pkt == 0x1)
		count++;
#endif
#ifdef DWC_ETH_QOS_ENABLE_DVLAN
	if (pdata->via_reg_or_desc == DWC_ETH_QOS_VIA_DESC) {
		count++;
	}
#endif /* End of DWC_ETH_QOS_ENABLE_DVLAN */

	desc_data->free_desc_cnt -= count;
	desc_data->tx_pkt_queued += count;

#ifdef DWC_ETH_QOS_ENABLE_TX_PKT_DUMP
	print_pkt(skb, skb->len, 1, (desc_data->cur_tx - 1));
#endif

	/* fallback to software time stamping if core doesn't
	 * support hardware time stamping */
	if ((pdata->hw_feat.tsstssel == 0) || (pdata->hwts_tx_en == 0))
		skb_tx_timestamp(skb);

	/* configure required descriptor fields for transmission */
	hw_if->pre_xmit(pdata, qInx);

tx_netdev_return:
	spin_unlock_irqrestore(&pdata->tx_lock, flags);

	DBGPR("<--DWC_ETH_QOS_start_xmit\n");

	return retval;
}

static void DWC_ETH_QOS_print_rx_tstamp_info(struct s_RX_NORMAL_DESC *rxdesc,
	unsigned int qInx)
{
	u32 ptp_status = 0;
	u32 pkt_type = 0;
	char *tstamp_dropped = NULL;
	char *tstamp_available = NULL;
	char *ptp_version = NULL;
	char *ptp_pkt_type = NULL;
	char *ptp_msg_type = NULL;

	DBGPR_PTP("-->DWC_ETH_QOS_print_rx_tstamp_info\n");

	/* status in RDES1 is not valid */
	if (!(rxdesc->RDES3 & DWC_ETH_QOS_RDESC3_RS1V))
		return;

	ptp_status = rxdesc->RDES1;
	tstamp_dropped = ((ptp_status & 0x8000) ? "YES" : "NO");
	tstamp_available = ((ptp_status & 0x4000) ? "YES" : "NO");
	ptp_version = ((ptp_status & 0x2000) ? "v2 (1588-2008)" : "v1 (1588-2002)");
	ptp_pkt_type = ((ptp_status & 0x1000) ? "ptp over Eth" : "ptp over IPv4/6");

	pkt_type = ((ptp_status & 0xF00) > 8);
	switch (pkt_type) {
	case 0:
		ptp_msg_type = "NO PTP msg received";
		break;
	case 1:
		ptp_msg_type = "SYNC";
		break;
	case 2:
		ptp_msg_type = "Follow_Up";
		break;
	case 3:
		ptp_msg_type = "Delay_Req";
		break;
	case 4:
		ptp_msg_type = "Delay_Resp";
		break;
	case 5:
		ptp_msg_type = "Pdelay_Req";
		break;
	case 6:
		ptp_msg_type = "Pdelay_Resp";
		break;
	case 7:
		ptp_msg_type = "Pdelay_Resp_Follow_up";
		break;
	case 8:
		ptp_msg_type = "Announce";
		break;
	case 9:
		ptp_msg_type = "Management";
		break;
	case 10:
		ptp_msg_type = "Signaling";
		break;
	case 11:
	case 12:
	case 13:
	case 14:
		ptp_msg_type = "Reserved";
		break;
	case 15:
		ptp_msg_type = "PTP pkr with Reserved Msg Type";
		break;
	}

	DBGPR_PTP("Rx timestamp detail for queue %d\n"
			"tstamp dropped    = %s\n"
			"tstamp available  = %s\n"
			"PTP version       = %s\n"
			"PTP Pkt Type      = %s\n"
			"PTP Msg Type      = %s\n",
			qInx, tstamp_dropped, tstamp_available,
			ptp_version, ptp_pkt_type, ptp_msg_type);

	DBGPR_PTP("<--DWC_ETH_QOS_print_rx_tstamp_info\n");
}


/*!
* \brief API to get rx time stamp value.
*
* \details This function will read received packet's timestamp from
* the descriptor and pass it to stack and also perform some sanity checks.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] skb - pointer to sk_buff structure.
* \param[in] desc_data - pointer to wrapper receive descriptor structure.
* \param[in] qInx - Queue/Channel number.
*
* \return integer
*
* \retval 0 if no context descriptor
* \retval 1 if timestamp is valid
* \retval 2 if time stamp is corrupted
*/

static unsigned char DWC_ETH_QOS_get_rx_hwtstamp(
	struct DWC_ETH_QOS_prv_data *pdata,
	struct sk_buff *skb,
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data,
	unsigned int qInx)
{
	struct s_RX_NORMAL_DESC *rx_normal_desc =
		GET_RX_DESC_PTR(qInx, desc_data->cur_rx);
	struct s_RX_CONTEXT_DESC *rx_context_desc = NULL;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct skb_shared_hwtstamps *shhwtstamp = NULL;
	u64 ns;
	int retry, ret;

	DBGPR_PTP("-->DWC_ETH_QOS_get_rx_hwtstamp\n");

	DWC_ETH_QOS_print_rx_tstamp_info(rx_normal_desc, qInx);

	desc_data->dirty_rx++;
	INCR_RX_DESC_INDEX(desc_data->cur_rx, 1);
	rx_context_desc = GET_RX_DESC_PTR(qInx, desc_data->cur_rx);

	DBGPR_PTP("\nRX_CONTEX_DESC[%d %4p %d RECEIVED FROM DEVICE]"\
			" = %#x:%#x:%#x:%#x",
			qInx, rx_context_desc, desc_data->cur_rx, rx_context_desc->RDES0,
			rx_context_desc->RDES1,
			rx_context_desc->RDES2, rx_context_desc->RDES3);

	/* check rx tsatmp */
	for (retry = 0; retry < 10; retry++) {
		ret = hw_if->get_rx_tstamp_status(rx_context_desc);
		if (ret == 1) {
			/* time stamp is valid */
			break;
		} else if (ret == 0) {
			printk(KERN_ALERT "Device has not yet updated the context "
				"desc to hold Rx time stamp(retry = %d)\n", retry);
		} else {
			printk(KERN_ALERT "Error: Rx time stamp is corrupted(retry = %d)\n", retry);
			return 2;
		}
	}

	if (retry == 10) {
			printk(KERN_ALERT "Device has not yet updated the context "
				"desc to hold Rx time stamp(retry = %d)\n", retry);
			desc_data->dirty_rx--;
			DECR_RX_DESC_INDEX(desc_data->cur_rx);
			return 0;
	}

	pdata->xstats.rx_timestamp_captured_n++;
	/* get valid tstamp */
	ns = hw_if->get_rx_tstamp(rx_context_desc);

	shhwtstamp = skb_hwtstamps(skb);
	memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamp->hwtstamp = ns_to_ktime(ns);

	DBGPR_PTP("<--DWC_ETH_QOS_get_rx_hwtstamp\n");

	return 1;
}


/*!
* \brief API to get tx time stamp value.
*
* \details This function will read timestamp from the descriptor
* and pass it to stack and also perform some sanity checks.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] txdesc - pointer to transmit descriptor structure.
* \param[in] skb - pointer to sk_buff structure.
*
* \return integer
*
* \retval 1 if time stamp is taken
* \retval 0 if time stamp in not taken/valid
*/

static unsigned int DWC_ETH_QOS_get_tx_hwtstamp(
	struct DWC_ETH_QOS_prv_data *pdata,
	struct s_TX_NORMAL_DESC *txdesc,
	struct sk_buff *skb)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct skb_shared_hwtstamps shhwtstamp;
	u64 ns;

	DBGPR_PTP("-->DWC_ETH_QOS_get_tx_hwtstamp\n");

	if (hw_if->drop_tx_status_enabled() == 0) {
		/* check tx tstamp status */
		if (!hw_if->get_tx_tstamp_status(txdesc)) {
			printk(KERN_ALERT "tx timestamp is not captured for this packet\n");
			return 0;
		}

		/* get the valid tstamp */
		ns = hw_if->get_tx_tstamp(txdesc);
	} else {
		/* drop tx status mode is enabled, hence read time
		 * stamp from register instead of descriptor */

		/* check tx tstamp status */
		if (!hw_if->get_tx_tstamp_status_via_reg()) {
			printk(KERN_ALERT "tx timestamp is not captured for this packet\n");
			return 0;
		}

		/* get the valid tstamp */
		ns = hw_if->get_tx_tstamp_via_reg();
	}

	pdata->xstats.tx_timestamp_captured_n++;
	memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamp.hwtstamp = ns_to_ktime(ns);
	/* pass tstamp to stack */
	skb_tstamp_tx(skb, &shhwtstamp);

	DBGPR_PTP("<--DWC_ETH_QOS_get_tx_hwtstamp\n");

	return 1;
}


/*!
* \brief API to update the tx status.
*
* \details This function is called in isr handler once after getting
* transmit complete interrupt to update the transmited packet status
* and it does some house keeping work like updating the
* private data structure variables.
*
* \param[in] dev - pointer to net_device structure
* \param[in] pdata - pointer to private data structure.
*
* \return void
*/

static void DWC_ETH_QOS_tx_interrupt(struct net_device *dev,
				     struct DWC_ETH_QOS_prv_data *pdata,
				     UINT qInx)
{
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data =
	    GET_TX_WRAPPER_DESC(qInx);
	struct s_TX_NORMAL_DESC *txptr = NULL;
	struct DWC_ETH_QOS_tx_buffer *buffer = NULL;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct desc_if_struct *desc_if = &(pdata->desc_if);
#ifndef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT
	int err_incremented;
#endif
	unsigned int tstamp_taken = 0;
	unsigned long flags;

	DBGPR("-->DWC_ETH_QOS_tx_interrupt: desc_data->tx_pkt_queued = %d"
		" dirty_tx = %d, qInx = %u\n",
		desc_data->tx_pkt_queued, desc_data->dirty_tx, qInx);

	spin_lock_irqsave(&pdata->tx_lock, flags);

	pdata->xstats.tx_clean_n[qInx]++;
	while (desc_data->tx_pkt_queued > 0) {
		txptr = GET_TX_DESC_PTR(qInx, desc_data->dirty_tx);
		buffer = GET_TX_BUF_PTR(qInx, desc_data->dirty_tx);
		tstamp_taken = 0;

		if (!hw_if->tx_complete(txptr))
			break;

#ifdef DWC_ETH_QOS_ENABLE_TX_DESC_DUMP
		dump_tx_desc(pdata, desc_data->dirty_tx, desc_data->dirty_tx,
			     0, qInx);
#endif

#ifndef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT
		/* update the tx error if any by looking at last segment
		 * for NORMAL descriptors
		 * */
		if ((hw_if->get_tx_desc_ls(txptr)) && !(hw_if->get_tx_desc_ctxt(txptr))) {
			/* check whether skb support hw tstamp */
			if ((pdata->hw_feat.tsstssel) &&
				(skb_shinfo(buffer->skb)->tx_flags & SKBTX_IN_PROGRESS)) {
				tstamp_taken = DWC_ETH_QOS_get_tx_hwtstamp(pdata,
					txptr, buffer->skb);
				if (tstamp_taken) {
					//dump_tx_desc(pdata, desc_data->dirty_tx, desc_data->dirty_tx,
					//		0, qInx);
					DBGPR_PTP("passed tx timestamp to stack[qInx = %d, dirty_tx = %d]\n",
						qInx, desc_data->dirty_tx);
				}
			}

			err_incremented = 0;
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
					err_incremented = 1;
					dev->stats.tx_fifo_errors++;
					if (hw_if->tx_update_fifo_threshold)
						hw_if->tx_update_fifo_threshold(txptr);
				}
			}
			if (hw_if->tx_get_collision_count)
				dev->stats.collisions +=
				    hw_if->tx_get_collision_count(txptr);

			if (err_incremented == 1)
				dev->stats.tx_errors++;

			pdata->xstats.q_tx_pkt_n[qInx]++;
			pdata->xstats.tx_pkt_n++;
			dev->stats.tx_packets++;
		}
#else
		if ((hw_if->get_tx_desc_ls(txptr)) && !(hw_if->get_tx_desc_ctxt(txptr))) {
			/* check whether skb support hw tstamp */
			if ((pdata->hw_feat.tsstssel) &&
				(skb_shinfo(buffer->skb)->tx_flags & SKBTX_IN_PROGRESS)) {
				tstamp_taken = DWC_ETH_QOS_get_tx_hwtstamp(pdata,
					txptr, buffer->skb);
				if (tstamp_taken) {
					dump_tx_desc(pdata, desc_data->dirty_tx, desc_data->dirty_tx,
							0, qInx);
					DBGPR_PTP("passed tx timestamp to stack[qInx = %d, dirty_tx = %d]\n",
						qInx, desc_data->dirty_tx);
				}
			}
		}
#endif
		dev->stats.tx_bytes += buffer->len;
		dev->stats.tx_bytes += buffer->len2;
		desc_if->unmap_tx_skb(pdata, buffer);

		/* reset the descriptor so that driver/host can reuse it */
		hw_if->tx_desc_reset(desc_data->dirty_tx, pdata, qInx);

		INCR_TX_DESC_INDEX(desc_data->dirty_tx, 1);
		desc_data->free_desc_cnt++;
		desc_data->tx_pkt_queued--;
	}

	if ((desc_data->queue_stopped == 1) && (desc_data->free_desc_cnt > 0)) {
		desc_data->queue_stopped = 0;
		netif_wake_subqueue(dev, qInx);
	}
#ifdef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT
	/* DMA has finished Transmitting data to MAC Tx-Fifo */
	MAC_MCR_TE_UdfWr(1);
#endif

	if ((pdata->eee_enabled) && (!pdata->tx_path_in_lpi_mode) &&
		(!pdata->use_lpi_tx_automate)) {
		DWC_ETH_QOS_enable_eee_mode(pdata);
		mod_timer(&pdata->eee_ctrl_timer,
			DWC_ETH_QOS_LPI_TIMER(DWC_ETH_QOS_DEFAULT_LPI_TIMER));
	}

	spin_unlock_irqrestore(&pdata->tx_lock, flags);

	DBGPR("<--DWC_ETH_QOS_tx_interrupt: desc_data->tx_pkt_queued = %d\n",
	      desc_data->tx_pkt_queued);
}

#ifdef YDEBUG_FILTER
static void DWC_ETH_QOS_check_rx_filter_status(struct s_RX_NORMAL_DESC *RX_NORMAL_DESC)
{
	u32 rdes2 = RX_NORMAL_DESC->RDES2;
	u32 rdes3 = RX_NORMAL_DESC->RDES3;

	/* Receive Status RDES2 Valid ? */
	if ((rdes3 & 0x8000000) == 0x8000000) {
		if ((rdes2 & 0x400) == 0x400)
			printk(KERN_ALERT "ARP pkt received\n");
		if ((rdes2 & 0x800) == 0x800)
			printk(KERN_ALERT "ARP reply not generated\n");
		if ((rdes2 & 0x8000) == 0x8000)
			printk(KERN_ALERT "VLAN pkt passed VLAN filter\n");
		if ((rdes2 & 0x10000) == 0x10000)
			printk(KERN_ALERT "SA Address filter fail\n");
		if ((rdes2 & 0x20000) == 0x20000)
			printk(KERN_ALERT "DA Addess filter fail\n");
		if ((rdes2 & 0x40000) == 0x40000)
			printk(KERN_ALERT "pkt passed the HASH filter in MAC and HASH value = %#x\n",
					(rdes2 >> 19) & 0xff);
		if ((rdes2 & 0x8000000) == 0x8000000)
			printk(KERN_ALERT "L3 filter(%d) Match\n", ((rdes2 >> 29) & 0x7));
		if ((rdes2 & 0x10000000) == 0x10000000)
			printk(KERN_ALERT "L4 filter(%d) Match\n", ((rdes2 >> 29) & 0x7));
	}
}
#endif /* YDEBUG_FILTER */


/* pass skb to upper layer */
static void DWC_ETH_QOS_receive_skb(struct DWC_ETH_QOS_prv_data *pdata,
				    struct net_device *dev, struct sk_buff *skb,
				    UINT qInx)
{
	struct DWC_ETH_QOS_rx_queue *rx_queue = GET_RX_QUEUE_PTR(qInx);

	skb_record_rx_queue(skb, qInx);
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);

	if (dev->features & NETIF_F_GRO) {
		napi_gro_receive(&rx_queue->napi, skb);
	} else if ((dev->features & NETIF_F_LRO) &&
		(skb->ip_summed == CHECKSUM_UNNECESSARY)) {
		lro_receive_skb(&rx_queue->lro_mgr, skb, (void *)pdata);
		rx_queue->lro_flush_needed = 1;
	} else {
		netif_receive_skb(skb);
	}
}

static void DWC_ETH_QOS_consume_page(struct DWC_ETH_QOS_rx_buffer *buffer,
				     struct sk_buff *skb,
				     u16 length, u16 buf2_used)
{
	buffer->page = NULL;
	if (buf2_used)
		buffer->page2 = NULL;
	skb->len += length;
	skb->data_len += length;
	skb->truesize += length;
}

static void DWC_ETH_QOS_consume_page_split_hdr(
				struct DWC_ETH_QOS_rx_buffer *buffer,
				struct sk_buff *skb,
				u16 length,
				USHORT page2_used)
{
	if (page2_used)
		buffer->page2 = NULL;

	skb->len += length;
	skb->data_len += length;
	skb->truesize += length;
}

/* Receive Checksum Offload configuration */
static inline void DWC_ETH_QOS_config_rx_csum(struct DWC_ETH_QOS_prv_data *pdata,
		struct sk_buff *skb,
		struct s_RX_NORMAL_DESC *rx_normal_desc)
{
	UINT varRDES1;

	skb->ip_summed = CHECKSUM_NONE;

	if ((pdata->dev_state & NETIF_F_RXCSUM) == NETIF_F_RXCSUM) {
		/* Receive Status RDES1 Valid ? */
		if ((rx_normal_desc->RDES3 & DWC_ETH_QOS_RDESC3_RS1V)) {
			/* check(RDES1.IPCE bit) whether device has done csum correctly or not */
			RX_NORMAL_DESC_RDES1_Ml_Rd(rx_normal_desc->RDES1, varRDES1);
			if ((varRDES1 & 0xC8) == 0x0)
				skb->ip_summed = CHECKSUM_UNNECESSARY;	/* csum done by device */
		}
	}
}

static inline void DWC_ETH_QOS_get_rx_vlan(struct DWC_ETH_QOS_prv_data *pdata,
			struct sk_buff *skb,
			struct s_RX_NORMAL_DESC *rx_normal_desc)
{
	USHORT vlan_tag = 0;

	if ((pdata->dev_state & NETIF_F_HW_VLAN_CTAG_RX) == NETIF_F_HW_VLAN_CTAG_RX) {
		/* Receive Status RDES0 Valid ? */
		if ((rx_normal_desc->RDES3 & DWC_ETH_QOS_RDESC3_RS0V)) {
			/* device received frame with VLAN Tag or
			 * double VLAN Tag ? */
			if (((rx_normal_desc->RDES3 & DWC_ETH_QOS_RDESC3_LT) == 0x40000)
				|| ((rx_normal_desc->RDES3 & DWC_ETH_QOS_RDESC3_LT) == 0x50000)) {
				vlan_tag = rx_normal_desc->RDES0 & 0xffff;
				/* insert VLAN tag into skb */
				__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vlan_tag);
				pdata->xstats.rx_vlan_pkt_n++;
			}
		}
	}
}

/* This api check for payload type and returns
 * 1 if payload load is TCP else returns 0;
 * */
static int DWC_ETH_QOS_check_for_tcp_payload(struct s_RX_NORMAL_DESC *rxdesc)
{
		u32 pt_type = 0;
		int ret = 0;

		if (rxdesc->RDES3 & DWC_ETH_QOS_RDESC3_RS1V) {
				pt_type = rxdesc->RDES1 & DWC_ETH_QOS_RDESC1_PT;
				if (pt_type == DWC_ETH_QOS_RDESC1_PT_TCP)
						ret = 1;
		}

		return ret;
}

/*!
* \brief API to pass the Rx packets to stack if split header
* feature is enabled.
*
* \details This function is invoked by main NAPI function if RX
* split header feature is enabled. This function checks the device
* descriptor for the packets and passes it to stack if any packtes
* are received by device.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] quota - maximum no. of packets that we are allowed to pass
* to into the kernel.
* \param[in] qInx - DMA channel/queue no. to be checked for packet.
*
* \return integer
*
* \retval number of packets received.
*/

static int DWC_ETH_QOS_clean_split_hdr_rx_irq(
			struct DWC_ETH_QOS_prv_data *pdata,
			int quota,
			UINT qInx)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data =
	    GET_RX_WRAPPER_DESC(qInx);
	struct net_device *dev = pdata->dev;
	struct desc_if_struct *desc_if = &pdata->desc_if;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct sk_buff *skb = NULL;
	int received = 0;
	struct DWC_ETH_QOS_rx_buffer *buffer = NULL;
	struct s_RX_NORMAL_DESC *RX_NORMAL_DESC = NULL;
	u16 pkt_len;
	unsigned short hdr_len = 0;
	unsigned short payload_len = 0;
	unsigned char intermediate_desc_cnt = 0;
	unsigned char buf2_used = 0;
	int ret;

#ifdef HWA_NV_1618922
	UINT ErrBits = 0x1200000;
#endif
	DBGPR("-->DWC_ETH_QOS_clean_split_hdr_rx_irq: qInx = %u, quota = %d\n",
		qInx, quota);

	while (received < quota) {
		buffer = GET_RX_BUF_PTR(qInx, desc_data->cur_rx);
		RX_NORMAL_DESC = GET_RX_DESC_PTR(qInx, desc_data->cur_rx);

		/* check for data availability */
		if (!(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_OWN)) {
#ifdef DWC_ETH_QOS_ENABLE_RX_DESC_DUMP
			dump_rx_desc(qInx, RX_NORMAL_DESC, desc_data->cur_rx);
#endif
			/* assign it to new skb */
			skb = buffer->skb;
			buffer->skb = NULL;

			/* first buffer pointer */
			dma_unmap_single(&pdata->pdev->dev, buffer->dma,
				       (2*buffer->rx_hdr_size), DMA_FROM_DEVICE);
			buffer->dma = 0;

			/* second buffer pointer */
			dma_unmap_page(&pdata->pdev->dev, buffer->dma2,
				       PAGE_SIZE, DMA_FROM_DEVICE);
			buffer->dma2 = 0;

			/* get the packet length */
			pkt_len =
			    (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_PL);

			/* FIRST desc and Receive Status RDES2 Valid ? */
			if ((RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_FD) &&
				(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_RS2V)) {
				/* get header length */
				hdr_len = (RX_NORMAL_DESC->RDES2 & DWC_ETH_QOS_RDESC2_HL);
				DBGPR("Device has %s HEADER SPLIT: hdr_len = %d\n",
						(hdr_len ? "done" : "not done"), hdr_len);
				if (hdr_len)
					pdata->xstats.rx_split_hdr_pkt_n++;
			}

			/* check for bad packet,
			 * error is valid only for last descriptor(OWN + LD bit set).
			 * */
#ifdef HWA_NV_1618922
			if ((RX_NORMAL_DESC->RDES3 & ErrBits) &&
			    (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_LD)) {
#else
			if ((RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_ES) &&
			    (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_LD)) {
#endif
				DBGPR("Error in rcved pkt, failed to pass it to upper layer\n");
				dump_rx_desc(qInx, RX_NORMAL_DESC, desc_data->cur_rx);
				dev->stats.rx_errors++;
				DWC_ETH_QOS_update_rx_errors(dev,
					RX_NORMAL_DESC->RDES3);

				/* recycle both page/buff and skb */
				buffer->skb = skb;
				if (desc_data->skb_top)
					dev_kfree_skb_any(desc_data->skb_top);

				desc_data->skb_top = NULL;
				goto next_desc;
			}

			if (!(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_LD)) {
				intermediate_desc_cnt++;
				buf2_used = 1;
				/* this descriptor is only the beginning/middle */
				if (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_FD) {
					/* this is the beginning of a chain */

					/* here skb/skb_top may contain
					 * if (device done split header)
					 *	only header
					 * else
					 *	header/(header + payload)
					 * */
					desc_data->skb_top = skb;
					/* page2 always contain only payload */
					if (hdr_len) {
						/* add header len to first skb->len */
						skb_put(skb, hdr_len);
						payload_len = pdata->rx_buffer_len;
						skb_fill_page_desc(skb, 0,
							buffer->page2, 0,
							payload_len);
					} else {
						/* add header len to first skb->len */
						skb_put(skb, buffer->rx_hdr_size);
						/* No split header, hence
						 * pkt_len = (payload + hdr_len)
						 * */
						payload_len = (pkt_len - buffer->rx_hdr_size);
						skb_fill_page_desc(skb, 0,
							buffer->page2, 0,
							payload_len);
					}
				} else {
					/* this is the middle of a chain */
					payload_len = pdata->rx_buffer_len;
					skb_fill_page_desc(desc_data->skb_top,
						skb_shinfo(desc_data->skb_top)->nr_frags,
						buffer->page2, 0,
						payload_len);

					/* re-use this skb, as consumed only the page */
					buffer->skb = skb;
				}
				DWC_ETH_QOS_consume_page_split_hdr(buffer,
							 desc_data->skb_top,
							 payload_len, buf2_used);
				goto next_desc;
			} else {
				if (!(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_FD)) {
					buf2_used = 1;
					/* end of the chain */
					if (hdr_len) {
						payload_len = (pkt_len -
							(pdata->rx_buffer_len * intermediate_desc_cnt) -
							hdr_len);
					} else {
						payload_len = (pkt_len -
							(pdata->rx_buffer_len * intermediate_desc_cnt) -
							buffer->rx_hdr_size);
					}

					skb_fill_page_desc(desc_data->skb_top,
						skb_shinfo(desc_data->skb_top)->nr_frags,
						buffer->page2, 0,
						payload_len);

					/* re-use this skb, as consumed only the page */
					buffer->skb = skb;
					skb = desc_data->skb_top;
					desc_data->skb_top = NULL;
					DWC_ETH_QOS_consume_page_split_hdr(buffer, skb,
								 payload_len, buf2_used);
				} else {
					/* no chain, got both FD + LD together */
					if (hdr_len) {
						buf2_used = 1;
						/* add header len to first skb->len */
						skb_put(skb, hdr_len);

						payload_len = pkt_len - hdr_len;
						skb_fill_page_desc(skb, 0,
							buffer->page2, 0,
							payload_len);
					} else {
						/* No split header, hence
						 * payload_len = (payload + hdr_len)
						 * */
						if (pkt_len > buffer->rx_hdr_size) {
							buf2_used = 1;
							/* add header len to first skb->len */
							skb_put(skb, buffer->rx_hdr_size);

							payload_len = (pkt_len - buffer->rx_hdr_size);
							skb_fill_page_desc(skb, 0,
								buffer->page2, 0,
								payload_len);
						} else {
							buf2_used = 0;
							/* add header len to first skb->len */
							skb_put(skb, pkt_len);
							payload_len = 0; /* no data in page2 */
						}
					}
					DWC_ETH_QOS_consume_page_split_hdr(buffer,
							skb, payload_len,
							buf2_used);
				}
				/* reset for next new packet/frame */
				intermediate_desc_cnt = 0;
				hdr_len = 0;
			}

			DWC_ETH_QOS_config_rx_csum(pdata, skb, RX_NORMAL_DESC);

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
			DWC_ETH_QOS_get_rx_vlan(pdata, skb, RX_NORMAL_DESC);
#endif

#ifdef YDEBUG_FILTER
			DWC_ETH_QOS_check_rx_filter_status(RX_NORMAL_DESC);
#endif

			if ((pdata->hw_feat.tsstssel) && (pdata->hwts_rx_en)) {
				/* get rx tstamp if available */
				if (hw_if->rx_tstamp_available(RX_NORMAL_DESC)) {
					ret = DWC_ETH_QOS_get_rx_hwtstamp(pdata,
							skb, desc_data, qInx);
					if (ret == 0) {
						/* device has not yet updated the CONTEXT desc to hold the
						 * time stamp, hence delay the packet reception
						 * */
						buffer->skb = skb;
						buffer->dma = dma_map_single(&pdata->pdev->dev, skb->data,
								pdata->rx_buffer_len, DMA_FROM_DEVICE);
						if (dma_mapping_error(&pdata->pdev->dev, buffer->dma))
							printk(KERN_ALERT "failed to do the RX dma map\n");

						goto rx_tstmp_failed;
					}
				}
			}

			if (!(dev->features & NETIF_F_GRO) &&
						(dev->features & NETIF_F_LRO)) {
					pdata->tcp_pkt =
							DWC_ETH_QOS_check_for_tcp_payload(RX_NORMAL_DESC);
			}

			dev->last_rx = jiffies;
			/* update the statistics */
			dev->stats.rx_packets++;
			dev->stats.rx_bytes += skb->len;
			DWC_ETH_QOS_receive_skb(pdata, dev, skb, qInx);
			received++;
 next_desc:
			desc_data->dirty_rx++;
			if (desc_data->dirty_rx >= desc_data->skb_realloc_threshold)
				desc_if->realloc_skb(pdata, qInx);

			INCR_RX_DESC_INDEX(desc_data->cur_rx, 1);
			buf2_used = 0;
		} else {
			/* no more data to read */
			break;
		}
	}

rx_tstmp_failed:

	if (desc_data->dirty_rx)
		desc_if->realloc_skb(pdata, qInx);

	DBGPR("<--DWC_ETH_QOS_clean_split_hdr_rx_irq: received = %d\n",
		received);

	return received;
}


/*!
* \brief API to pass the Rx packets to stack if jumbo frame
* is enabled.
*
* \details This function is invoked by main NAPI function if Rx
* jumbe frame is enabled. This function checks the device descriptor
* for the packets and passes it to stack if any packtes are received
* by device.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] quota - maximum no. of packets that we are allowed to pass
* to into the kernel.
* \param[in] qInx - DMA channel/queue no. to be checked for packet.
*
* \return integer
*
* \retval number of packets received.
*/

static int DWC_ETH_QOS_clean_jumbo_rx_irq(struct DWC_ETH_QOS_prv_data *pdata,
					  int quota,
					  UINT qInx)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data =
	    GET_RX_WRAPPER_DESC(qInx);
	struct net_device *dev = pdata->dev;
	struct desc_if_struct *desc_if = &pdata->desc_if;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct sk_buff *skb = NULL;
	int received = 0;
	struct DWC_ETH_QOS_rx_buffer *buffer = NULL;
	struct s_RX_NORMAL_DESC *RX_NORMAL_DESC = NULL;
	u16 pkt_len;
	UCHAR intermediate_desc_cnt = 0;
	unsigned int buf2_used;
	int ret;
#ifdef HWA_NV_1618922
	UINT ErrBits = 0x1200000;
#endif
	DBGPR("-->DWC_ETH_QOS_clean_jumbo_rx_irq: qInx = %u, quota = %d\n",
		qInx, quota);

	while (received < quota) {
		buffer = GET_RX_BUF_PTR(qInx, desc_data->cur_rx);
		RX_NORMAL_DESC = GET_RX_DESC_PTR(qInx, desc_data->cur_rx);

		/* check for data availability */
		if (!(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_OWN)) {
#ifdef DWC_ETH_QOS_ENABLE_RX_DESC_DUMP
			dump_rx_desc(qInx, RX_NORMAL_DESC, desc_data->cur_rx);
#endif
			/* assign it to new skb */
			skb = buffer->skb;
			buffer->skb = NULL;

			/* first buffer pointer */
			dma_unmap_page(&pdata->pdev->dev, buffer->dma,
				       PAGE_SIZE, DMA_FROM_DEVICE);
			buffer->dma = 0;

			/* second buffer pointer */
			dma_unmap_page(&pdata->pdev->dev, buffer->dma2,
				       PAGE_SIZE, DMA_FROM_DEVICE);
			buffer->dma2 = 0;

			/* get the packet length */
			pkt_len =
				(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_PL);

			/* check for bad packet,
			 * error is valid only for last descriptor (OWN + LD bit set).
			 * */
#ifdef HWA_NV_1618922
			if ((RX_NORMAL_DESC->RDES3 & ErrBits) &&
			    (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_LD)) {
#else
			if ((RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_ES) &&
			    (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_LD)) {
#endif
				DBGPR("Error in rcved pkt, failed to pass it to upper layer\n");
				dump_rx_desc(qInx, RX_NORMAL_DESC, desc_data->cur_rx);
				dev->stats.rx_errors++;
				DWC_ETH_QOS_update_rx_errors(dev,
					RX_NORMAL_DESC->RDES3);

				/* recycle both page and skb */
				buffer->skb = skb;
				if (desc_data->skb_top)
					dev_kfree_skb_any(desc_data->skb_top);

				desc_data->skb_top = NULL;
				goto next_desc;
			}

			if (!(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_LD)) {
				intermediate_desc_cnt++;
				buf2_used = 1;
				/* this descriptor is only the beginning/middle */
				if (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_FD) {
					/* this is the beginning of a chain */
					desc_data->skb_top = skb;
					skb_fill_page_desc(skb, 0,
						buffer->page, 0,
						pdata->rx_buffer_len);

					DBGPR("RX: pkt in second buffer pointer\n");
					skb_fill_page_desc(
						desc_data->skb_top,
						skb_shinfo(desc_data->skb_top)->nr_frags,
						buffer->page2, 0,
						pdata->rx_buffer_len);
				} else {
					/* this is the middle of a chain */
					skb_fill_page_desc(desc_data->skb_top,
						skb_shinfo(desc_data->skb_top)->nr_frags,
						buffer->page, 0,
						pdata->rx_buffer_len);

					DBGPR("RX: pkt in second buffer pointer\n");
					skb_fill_page_desc(desc_data->skb_top,
						skb_shinfo(desc_data->skb_top)->nr_frags,
						buffer->page2, 0,
						pdata->rx_buffer_len);
					/* re-use this skb, as consumed only the page */
					buffer->skb = skb;
				}
				DWC_ETH_QOS_consume_page(buffer,
							 desc_data->skb_top,
							 (pdata->rx_buffer_len * 2),
							 buf2_used);
				goto next_desc;
			} else {
				if (!(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_FD)) {
					/* end of the chain */
					pkt_len =
						(pkt_len - (pdata->rx_buffer_len * intermediate_desc_cnt));
					if (pkt_len > pdata->rx_buffer_len) {
						skb_fill_page_desc(desc_data->skb_top,
							skb_shinfo(desc_data->skb_top)->nr_frags,
							buffer->page, 0,
							pdata->rx_buffer_len);

						DBGPR("RX: pkt in second buffer pointer\n");
						skb_fill_page_desc(desc_data->skb_top,
							skb_shinfo(desc_data->skb_top)->nr_frags,
							buffer->page2, 0,
							(pkt_len - pdata->rx_buffer_len));
						buf2_used = 1;
					} else {
						skb_fill_page_desc(desc_data->skb_top,
							skb_shinfo(desc_data->skb_top)->nr_frags,
							buffer->page, 0,
							pkt_len);
						buf2_used = 0;
					}
					/* re-use this skb, as consumed only the page */
					buffer->skb = skb;
					skb = desc_data->skb_top;
					desc_data->skb_top = NULL;
					DWC_ETH_QOS_consume_page(buffer, skb,
								 pkt_len,
								 buf2_used);
				} else {
					/* no chain, got both FD + LD together */

					/* code added for copybreak, this should improve
					 * performance for small pkts with large amount
					 * of reassembly being done in the stack
					 * */
					if ((pkt_len <= DWC_ETH_QOS_COPYBREAK_DEFAULT)
					    && (skb_tailroom(skb) >= pkt_len)) {
						u8 *vaddr;
						vaddr =
						    kmap_atomic(buffer->page);
						memcpy(skb_tail_pointer(skb),
						       vaddr, pkt_len);
						kunmap_atomic(vaddr);
						/* re-use the page, so don't erase buffer->page/page2 */
						skb_put(skb, pkt_len);
					} else {
						if (pkt_len > pdata->rx_buffer_len) {
							skb_fill_page_desc(skb,
								0, buffer->page,
								0,
								pdata->rx_buffer_len);

							DBGPR ("RX: pkt in second buffer pointer\n");
							skb_fill_page_desc(skb,
								skb_shinfo(skb)->nr_frags, buffer->page2,
								0,
								(pkt_len - pdata->rx_buffer_len));
							buf2_used = 1;
						} else {
							skb_fill_page_desc(skb,
								0, buffer->page,
								0,
								pkt_len);
							buf2_used = 0;
						}
						DWC_ETH_QOS_consume_page(buffer,
								skb,
								pkt_len,
								buf2_used);
					}
				}
				intermediate_desc_cnt = 0;
			}

			DWC_ETH_QOS_config_rx_csum(pdata, skb, RX_NORMAL_DESC);

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
			DWC_ETH_QOS_get_rx_vlan(pdata, skb, RX_NORMAL_DESC);
#endif

#ifdef YDEBUG_FILTER
			DWC_ETH_QOS_check_rx_filter_status(RX_NORMAL_DESC);
#endif

			if ((pdata->hw_feat.tsstssel) && (pdata->hwts_rx_en)) {
				/* get rx tstamp if available */
				if (hw_if->rx_tstamp_available(RX_NORMAL_DESC)) {
					ret = DWC_ETH_QOS_get_rx_hwtstamp(pdata,
							skb, desc_data, qInx);
					if (ret == 0) {
						/* device has not yet updated the CONTEXT desc to hold the
						 * time stamp, hence delay the packet reception
						 * */
						buffer->skb = skb;
						buffer->dma = dma_map_single(&pdata->pdev->dev, skb->data,
								pdata->rx_buffer_len, DMA_FROM_DEVICE);
						if (dma_mapping_error(&pdata->pdev->dev, buffer->dma))
							printk(KERN_ALERT "failed to do the RX dma map\n");

						goto rx_tstmp_failed;
					}
				}
			}

			if (!(dev->features & NETIF_F_GRO) &&
						(dev->features & NETIF_F_LRO)) {
					pdata->tcp_pkt =
							DWC_ETH_QOS_check_for_tcp_payload(RX_NORMAL_DESC);
			}

			dev->last_rx = jiffies;
			/* update the statistics */
			dev->stats.rx_packets++;
			dev->stats.rx_bytes += skb->len;

			/* eth type trans needs skb->data to point to something */
			if (!pskb_may_pull(skb, ETH_HLEN)) {
				printk(KERN_ALERT "pskb_may_pull failed\n");
				dev_kfree_skb_any(skb);
				goto next_desc;
			}

			DWC_ETH_QOS_receive_skb(pdata, dev, skb, qInx);
			received++;
 next_desc:
			desc_data->dirty_rx++;
			if (desc_data->dirty_rx >= desc_data->skb_realloc_threshold)
				desc_if->realloc_skb(pdata, qInx);

			INCR_RX_DESC_INDEX(desc_data->cur_rx, 1);
		} else {
			/* no more data to read */
			break;
		}
	}

rx_tstmp_failed:

	if (desc_data->dirty_rx)
		desc_if->realloc_skb(pdata, qInx);

	DBGPR("<--DWC_ETH_QOS_clean_jumbo_rx_irq: received = %d\n", received);

	return received;
}

/*!
* \brief API to pass the Rx packets to stack if default mode
* is enabled.
*
* \details This function is invoked by main NAPI function in default
* Rx mode(non jumbo and non split header). This function checks the
* device descriptor for the packets and passes it to stack if any packtes
* are received by device.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] quota - maximum no. of packets that we are allowed to pass
* to into the kernel.
* \param[in] qInx - DMA channel/queue no. to be checked for packet.
*
* \return integer
*
* \retval number of packets received.
*/

static int DWC_ETH_QOS_clean_rx_irq(struct DWC_ETH_QOS_prv_data *pdata,
				    int quota,
				    UINT qInx)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data =
	    GET_RX_WRAPPER_DESC(qInx);
	struct net_device *dev = pdata->dev;
	struct desc_if_struct *desc_if = &pdata->desc_if;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct sk_buff *skb = NULL;
	int received = 0;
	struct DWC_ETH_QOS_rx_buffer *buffer = NULL;
	struct s_RX_NORMAL_DESC *RX_NORMAL_DESC = NULL;
	UINT pkt_len;
#ifdef HWA_NV_1618922
	UINT ErrBits = 0x1200000;
#endif
	int ret;

	DBGPR("-->DWC_ETH_QOS_clean_rx_irq: qInx = %u, quota = %d\n",
		qInx, quota);

	while (received < quota) {
		buffer = GET_RX_BUF_PTR(qInx, desc_data->cur_rx);
		RX_NORMAL_DESC = GET_RX_DESC_PTR(qInx, desc_data->cur_rx);

		/* check for data availability */
		if (!(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_OWN)) {
#ifdef DWC_ETH_QOS_ENABLE_RX_DESC_DUMP
			dump_rx_desc(qInx, RX_NORMAL_DESC, desc_data->cur_rx);
#endif
			/* assign it to new skb */
			skb = buffer->skb;
			buffer->skb = NULL;
			dma_unmap_single(&pdata->pdev->dev, buffer->dma,
					 pdata->rx_buffer_len, DMA_FROM_DEVICE);
			buffer->dma = 0;

			/* get the packet length */
			pkt_len =
			    (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_PL);

#ifdef DWC_ETH_QOS_ENABLE_RX_PKT_DUMP
			print_pkt(skb, pkt_len, 0, (desc_data->cur_rx));
#endif
			/* check for bad/oversized packet,
			 * error is valid only for last descriptor (OWN + LD bit set).
			 * */
#ifdef HWA_NV_1618922
			if (!(RX_NORMAL_DESC->RDES3 & ErrBits) &&
			    (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_LD)) {
#else
			if (!(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_ES) &&
			    (RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_LD)) {
#endif
				/* pkt_len = pkt_len - 4; */ /* CRC stripping */

				/* code added for copybreak, this should improve
				 * performance for small pkts with large amount
				 * of reassembly being done in the stack
				 * */
				if (pkt_len < DWC_ETH_QOS_COPYBREAK_DEFAULT) {
					struct sk_buff *new_skb =
					    netdev_alloc_skb_ip_align(dev,
								      pkt_len);
					if (new_skb) {
						skb_copy_to_linear_data_offset(new_skb,
							-NET_IP_ALIGN,
							(skb->data - NET_IP_ALIGN),
							(pkt_len + NET_IP_ALIGN));
						/* recycle actual desc skb */
						buffer->skb = skb;
						skb = new_skb;
					} else {
						/* just continue with the old skb */
					}
				}
				skb_put(skb, pkt_len);

				DWC_ETH_QOS_config_rx_csum(pdata, skb,
							RX_NORMAL_DESC);

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
				DWC_ETH_QOS_get_rx_vlan(pdata, skb, RX_NORMAL_DESC);
#endif

#ifdef YDEBUG_FILTER
				DWC_ETH_QOS_check_rx_filter_status(RX_NORMAL_DESC);
#endif

				if ((pdata->hw_feat.tsstssel) && (pdata->hwts_rx_en)) {
					/* get rx tstamp if available */
					if (hw_if->rx_tstamp_available(RX_NORMAL_DESC)) {
						ret = DWC_ETH_QOS_get_rx_hwtstamp(pdata,
								skb, desc_data, qInx);
						if (ret == 0) {
							/* device has not yet updated the CONTEXT desc to hold the
							 * time stamp, hence delay the packet reception
							 * */
							buffer->skb = skb;
							buffer->dma = dma_map_single(&pdata->pdev->dev, skb->data,
									pdata->rx_buffer_len, DMA_FROM_DEVICE);
							if (dma_mapping_error(&pdata->pdev->dev, buffer->dma))
								printk(KERN_ALERT "failed to do the RX dma map\n");

							goto rx_tstmp_failed;
						}
					}
				}


				if (!(dev->features & NETIF_F_GRO) &&
						(dev->features & NETIF_F_LRO)) {
						pdata->tcp_pkt =
								DWC_ETH_QOS_check_for_tcp_payload(RX_NORMAL_DESC);
				}

				dev->last_rx = jiffies;
				/* update the statistics */
				dev->stats.rx_packets++;
				dev->stats.rx_bytes += skb->len;
				DWC_ETH_QOS_receive_skb(pdata, dev, skb, qInx);
				received++;
			} else {
				dump_rx_desc(qInx, RX_NORMAL_DESC, desc_data->cur_rx);
				if (!(RX_NORMAL_DESC->RDES3 & DWC_ETH_QOS_RDESC3_LD))
					DBGPR("Received oversized pkt, spanned across multiple desc\n");

				/* recycle skb */
				buffer->skb = skb;
				dev->stats.rx_errors++;
				DWC_ETH_QOS_update_rx_errors(dev,
					RX_NORMAL_DESC->RDES3);
			}

			desc_data->dirty_rx++;
			if (desc_data->dirty_rx >= desc_data->skb_realloc_threshold)
				desc_if->realloc_skb(pdata, qInx);

			INCR_RX_DESC_INDEX(desc_data->cur_rx, 1);
		} else {
			/* no more data to read */
			break;
		}
	}

rx_tstmp_failed:

	if (desc_data->dirty_rx)
		desc_if->realloc_skb(pdata, qInx);

	DBGPR("<--DWC_ETH_QOS_clean_rx_irq: received = %d\n", received);

	return received;
}

/*!
* \brief API to update the rx status.
*
* \details This function is called in poll function to update the
* status of received packets.
*
* \param[in] dev - pointer to net_device structure.
* \param[in] rx_status - value of received packet status.
*
* \return void.
*/

void DWC_ETH_QOS_update_rx_errors(struct net_device *dev,
				 unsigned int rx_status)
{
	DBGPR("-->DWC_ETH_QOS_update_rx_errors\n");

	/* received pkt with crc error */
	if ((rx_status & 0x1000000))
		dev->stats.rx_crc_errors++;

	/* received frame alignment */
	if ((rx_status & 0x100000))
		dev->stats.rx_frame_errors++;

	/* receiver fifo overrun */
	if ((rx_status & 0x200000))
		dev->stats.rx_fifo_errors++;

	DBGPR("<--DWC_ETH_QOS_update_rx_errors\n");
}

/*!
* \brief API to pass the received packets to stack
*
* \details This function is provided by NAPI-compliant drivers to operate
* the interface in a polled mode, with interrupts disabled.
*
* \param[in] napi - pointer to napi_stuct structure.
* \param[in] budget - maximum no. of packets that we are allowed to pass
* to into the kernel.
*
* \return integer
*
* \retval number of packets received.
*/

int DWC_ETH_QOS_poll_mq(struct napi_struct *napi, int budget)
{
	struct DWC_ETH_QOS_rx_queue *rx_queue =
		container_of(napi, struct DWC_ETH_QOS_rx_queue, napi);
	struct DWC_ETH_QOS_prv_data *pdata = rx_queue->pdata;
	/* divide the budget evenly among all the queues */
	int per_q_budget = budget / DWC_ETH_QOS_RX_QUEUE_CNT;
	int qInx = 0;
	int received = 0, per_q_received = 0;

	DBGPR("-->DWC_ETH_QOS_poll_mq: budget = %d\n", budget);

	pdata->xstats.napi_poll_n++;
	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++) {
		rx_queue = GET_RX_QUEUE_PTR(qInx);

#ifdef DWC_ETH_QOS_TXPOLLING_MODE_ENABLE
		/* check for tx descriptor status */
		DWC_ETH_QOS_tx_interrupt(pdata->dev, pdata, qInx);
#endif
		rx_queue->lro_flush_needed = 0;

#ifdef RX_OLD_CODE
		per_q_received = DWC_ETH_QOS_poll(pdata, per_q_budget, qInx);
#else
		per_q_received = pdata->clean_rx(pdata, per_q_budget, qInx);
#endif
		received += per_q_received;
		pdata->xstats.rx_pkt_n += per_q_received;
		pdata->xstats.q_rx_pkt_n[qInx] += per_q_received;

		if (rx_queue->lro_flush_needed)
			lro_flush_all(&rx_queue->lro_mgr);
	}

	/* If we processed all pkts, we are done;
	 * tell the kernel & re-enable interrupt */
	if (received < budget) {
		if (pdata->dev->features & NETIF_F_GRO) {
			/* to turn off polling */
			napi_complete(napi);
			/* Enable all ch RX interrupt */
			DWC_ETH_QOS_enable_all_ch_rx_interrpt(pdata);
		} else {
			unsigned long flags;

			spin_lock_irqsave(&pdata->lock, flags);
			__napi_complete(napi);
			/* Enable all ch RX interrupt */
			DWC_ETH_QOS_enable_all_ch_rx_interrpt(pdata);
			spin_unlock_irqrestore(&pdata->lock, flags);
		}
	}

	DBGPR("<--DWC_ETH_QOS_poll_mq\n");

	return received;
}

/*!
* \brief API to return the device/interface status.
*
* \details The get_stats function is called whenever an application needs to
* get statistics for the interface. For example, this happend when ifconfig
* or netstat -i is run.
*
* \param[in] dev - pointer to net_device structure.
*
* \return net_device_stats structure
*
* \retval net_device_stats - returns pointer to net_device_stats structure.
*/

static struct net_device_stats *DWC_ETH_QOS_get_stats(struct net_device *dev)
{

	return &dev->stats;
}

#ifdef CONFIG_NET_POLL_CONTROLLER

/*!
* \brief API to receive packets in polling mode.
*
* \details This is polling receive function used by netconsole and other
* diagnostic tool to allow network i/o with interrupts disabled.
*
* \param[in] dev - pointer to net_device structure
*
* \return void
*/

static void DWC_ETH_QOS_poll_controller(struct net_device *dev)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);

	DBGPR("-->DWC_ETH_QOS_poll_controller\n");

	disable_irq(pdata->irq_number);
	DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS(pdata->irq_number, pdata);
	enable_irq(pdata->irq_number);

	DBGPR("<--DWC_ETH_QOS_poll_controller\n");
}

#endif	/*end of CONFIG_NET_POLL_CONTROLLER */

/*!
 * \brief User defined parameter setting API
 *
 * \details This function is invoked by kernel to update the device
 * configuration to new features. This function supports enabling and
 * disabling of TX and RX csum features.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] features – device feature to be enabled/disabled.
 *
 * \return int
 *
 * \retval 0
 */

static int DWC_ETH_QOS_set_features(struct net_device *dev, netdev_features_t features)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT dev_rxcsum_enable;
#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	UINT dev_rxvlan_enable, dev_txvlan_enable;
#endif

	if (pdata->hw_feat.rx_coe_sel) {
		dev_rxcsum_enable = !!(pdata->dev_state & NETIF_F_RXCSUM);

		if (((features & NETIF_F_RXCSUM) == NETIF_F_RXCSUM)
		    && !dev_rxcsum_enable) {
			hw_if->enable_rx_csum();
			pdata->dev_state |= NETIF_F_RXCSUM;
			printk(KERN_ALERT "State change - rxcsum enable\n");
		} else if (((features & NETIF_F_RXCSUM) == 0)
			   && dev_rxcsum_enable) {
			hw_if->disable_rx_csum();
			pdata->dev_state &= ~NETIF_F_RXCSUM;
			printk(KERN_ALERT "State change - rxcsum disable\n");
		}
	}
#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	dev_rxvlan_enable = !!(pdata->dev_state & NETIF_F_HW_VLAN_CTAG_RX);
	if (((features & NETIF_F_HW_VLAN_CTAG_RX) == NETIF_F_HW_VLAN_CTAG_RX)
	    && !dev_rxvlan_enable) {
		pdata->dev_state |= NETIF_F_HW_VLAN_CTAG_RX;
		hw_if->config_rx_outer_vlan_stripping(DWC_ETH_QOS_RX_VLAN_STRIP_ALWAYS);
		printk(KERN_ALERT "State change - rxvlan enable\n");
	} else if (((features & NETIF_F_HW_VLAN_CTAG_RX) == 0) &&
			dev_rxvlan_enable) {
		pdata->dev_state &= ~NETIF_F_HW_VLAN_CTAG_RX;
		hw_if->config_rx_outer_vlan_stripping(DWC_ETH_QOS_RX_NO_VLAN_STRIP);
		printk(KERN_ALERT "State change - rxvlan disable\n");
	}

	dev_txvlan_enable = !!(pdata->dev_state & NETIF_F_HW_VLAN_CTAG_TX);
	if (((features & NETIF_F_HW_VLAN_CTAG_TX) == NETIF_F_HW_VLAN_CTAG_TX)
	    && !dev_txvlan_enable) {
		pdata->dev_state |= NETIF_F_HW_VLAN_CTAG_TX;
		printk(KERN_ALERT "State change - txvlan enable\n");
	} else if (((features & NETIF_F_HW_VLAN_CTAG_TX) == 0) &&
			dev_txvlan_enable) {
		pdata->dev_state &= ~NETIF_F_HW_VLAN_CTAG_TX;
		printk(KERN_ALERT "State change - txvlan disable\n");
	}
#endif	/* DWC_ETH_QOS_ENABLE_VLAN_TAG */

	DBGPR("<--DWC_ETH_QOS_set_features\n");

	return 0;
}


/*!
 * \brief User defined parameter setting API
 *
 * \details This function is invoked by kernel to adjusts the requested
 * feature flags according to device-specific constraints, and returns the
 * resulting flags. This API must not modify the device state.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] features – device supported features.
 *
 * \return u32
 *
 * \retval modified flag
 */

static netdev_features_t DWC_ETH_QOS_fix_features(struct net_device *dev, netdev_features_t features)
{
#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
#endif
	DBGPR("-->DWC_ETH_QOS_fix_features: %#llx\n", features);

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	if (pdata->rx_split_hdr) {
		/* The VLAN tag stripping must be set for the split function.
		 * For instance, the DMA separates the header and payload of
		 * an untagged packet only. Hence, when a tagged packet is
		 * received, the QOS must be programmed such that the VLAN
		 * tags are deleted/stripped from the received packets.
		 * */
		features |= NETIF_F_HW_VLAN_CTAG_RX;
	}
#endif /* end of DWC_ETH_QOS_ENABLE_VLAN_TAG */

	DBGPR("<--DWC_ETH_QOS_fix_features: %#llx\n", features);

	return features;
}


/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to enable/disable receive split header mode.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] flags – flag to indicate whether RX split to be
 *                  enabled/disabled.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_rx_split_hdr_mode(struct net_device *dev,
		unsigned int flags)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned int qInx;
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_config_rx_split_hdr_mode\n");

	if (flags && pdata->rx_split_hdr) {
		printk(KERN_ALERT
			"Rx Split header mode is already enabled\n");
		return -EINVAL;
	}

	if (!flags && !pdata->rx_split_hdr) {
		printk(KERN_ALERT
			"Rx Split header mode is already disabled\n");
		return -EINVAL;
	}

	DWC_ETH_QOS_stop_dev(pdata);

	/* If split header mode is disabled(ie flags == 0)
	 * then RX will be in default/jumbo mode based on MTU
	 * */
	pdata->rx_split_hdr = !!flags;

	DWC_ETH_QOS_start_dev(pdata);

	hw_if->config_header_size(DWC_ETH_QOS_MAX_HDR_SIZE);
	/* enable/disable split header for all RX DMA channel */
	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++)
		hw_if->config_split_header_mode(qInx, pdata->rx_split_hdr);

	printk(KERN_ALERT "Succesfully %s Rx Split header mode\n",
		(flags ? "enabled" : "disabled"));

	DBGPR("<--DWC_ETH_QOS_config_rx_split_hdr_mode\n");

	return ret;
}


/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to enable/disable L3/L4 filtering.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] flags – flag to indicate whether L3/L4 filtering to be
 *                  enabled/disabled.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_l3_l4_filtering(struct net_device *dev,
		unsigned int flags)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int ret = 0;

	DBGPR_FILTER("-->DWC_ETH_QOS_config_l3_l4_filtering\n");

	if (flags && pdata->l3_l4_filter) {
		printk(KERN_ALERT
			"L3/L4 filtering is already enabled\n");
		return -EINVAL;
	}

	if (!flags && !pdata->l3_l4_filter) {
		printk(KERN_ALERT
			"L3/L4 filtering is already disabled\n");
		return -EINVAL;
	}

	pdata->l3_l4_filter = !!flags;
	hw_if->config_l3_l4_filter_enable(pdata->l3_l4_filter);

	DBGPR_FILTER("Succesfully %s L3/L4 filtering\n",
		(flags ? "ENABLED" : "DISABLED"));

	DBGPR_FILTER("<--DWC_ETH_QOS_config_l3_l4_filtering\n");

	return ret;
}


/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure L3(IPv4) filtering. This function does following,
 * - enable/disable IPv4 filtering.
 * - select source/destination address matching.
 * - select perfect/inverse matching.
 * - Update the IPv4 address into MAC register.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_ip4_filters(struct net_device *dev,
		struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct DWC_ETH_QOS_l3_l4_filter *u_l3_filter =
		(struct DWC_ETH_QOS_l3_l4_filter *)req->ptr;
	struct DWC_ETH_QOS_l3_l4_filter l_l3_filter;
	int ret = 0;

	DBGPR_FILTER("-->DWC_ETH_QOS_config_ip4_filters\n");

	if (pdata->hw_feat.l3l4_filter_num == 0)
		return DWC_ETH_QOS_NO_HW_SUPPORT;

	if (copy_from_user(&l_l3_filter, u_l3_filter,
		sizeof(struct DWC_ETH_QOS_l3_l4_filter)))
		return -EFAULT;

	if ((l_l3_filter.filter_no + 1) > pdata->hw_feat.l3l4_filter_num) {
		printk(KERN_ALERT "%d filter is not supported in the HW\n",
			l_l3_filter.filter_no);
		return DWC_ETH_QOS_NO_HW_SUPPORT;
	}

	if (!pdata->l3_l4_filter) {
		hw_if->config_l3_l4_filter_enable(1);
		pdata->l3_l4_filter = 1;
	}

	/* configure the L3 filters */
	hw_if->config_l3_filters(l_l3_filter.filter_no,
			l_l3_filter.filter_enb_dis, 0,
			l_l3_filter.src_dst_addr_match,
			l_l3_filter.perfect_inverse_match);

	if (!l_l3_filter.src_dst_addr_match)
		hw_if->update_ip4_addr0(l_l3_filter.filter_no,
				l_l3_filter.ip4_addr);
	else
		hw_if->update_ip4_addr1(l_l3_filter.filter_no,
				l_l3_filter.ip4_addr);

	DBGPR_FILTER("Successfully %s IPv4 %s %s addressing filtering on %d filter\n",
		(l_l3_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
		(l_l3_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"),
		(l_l3_filter.src_dst_addr_match ? "DESTINATION" : "SOURCE"),
		l_l3_filter.filter_no);

	DBGPR_FILTER("<--DWC_ETH_QOS_config_ip4_filters\n");

	return ret;
}


/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure L3(IPv6) filtering. This function does following,
 * - enable/disable IPv6 filtering.
 * - select source/destination address matching.
 * - select perfect/inverse matching.
 * - Update the IPv6 address into MAC register.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_ip6_filters(struct net_device *dev,
		struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct DWC_ETH_QOS_l3_l4_filter *u_l3_filter =
		(struct DWC_ETH_QOS_l3_l4_filter *)req->ptr;
	struct DWC_ETH_QOS_l3_l4_filter l_l3_filter;
	int ret = 0;

	DBGPR_FILTER("-->DWC_ETH_QOS_config_ip6_filters\n");

	if (pdata->hw_feat.l3l4_filter_num == 0)
		return DWC_ETH_QOS_NO_HW_SUPPORT;

	if (copy_from_user(&l_l3_filter, u_l3_filter,
		sizeof(struct DWC_ETH_QOS_l3_l4_filter)))
		return -EFAULT;

	if ((l_l3_filter.filter_no + 1) > pdata->hw_feat.l3l4_filter_num) {
		printk(KERN_ALERT "%d filter is not supported in the HW\n",
			l_l3_filter.filter_no);
		return DWC_ETH_QOS_NO_HW_SUPPORT;
	}

	if (!pdata->l3_l4_filter) {
		hw_if->config_l3_l4_filter_enable(1);
		pdata->l3_l4_filter = 1;
	}

	/* configure the L3 filters */
	hw_if->config_l3_filters(l_l3_filter.filter_no,
			l_l3_filter.filter_enb_dis, 1,
			l_l3_filter.src_dst_addr_match,
			l_l3_filter.perfect_inverse_match);

	hw_if->update_ip6_addr(l_l3_filter.filter_no,
			l_l3_filter.ip6_addr);

	DBGPR_FILTER("Successfully %s IPv6 %s %s addressing filtering on %d filter\n",
		(l_l3_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
		(l_l3_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"),
		(l_l3_filter.src_dst_addr_match ? "DESTINATION" : "SOURCE"),
		l_l3_filter.filter_no);

	DBGPR_FILTER("<--DWC_ETH_QOS_config_ip6_filters\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure L4(TCP/UDP) filtering. This function does following,
 * - enable/disable L4 filtering.
 * - select TCP/UDP filtering.
 * - select source/destination port matching.
 * - select perfect/inverse matching.
 * - Update the port number into MAC register.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 * \param[in] tcp_udp – flag to indicate TCP/UDP filtering.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_tcp_udp_filters(struct net_device *dev,
		struct ifr_data_struct *req,
		int tcp_udp)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct DWC_ETH_QOS_l3_l4_filter *u_l4_filter =
		(struct DWC_ETH_QOS_l3_l4_filter *)req->ptr;
	struct DWC_ETH_QOS_l3_l4_filter l_l4_filter;
	int ret = 0;

	DBGPR_FILTER("-->DWC_ETH_QOS_config_tcp_udp_filters\n");

	if (pdata->hw_feat.l3l4_filter_num == 0)
		return DWC_ETH_QOS_NO_HW_SUPPORT;

	if (copy_from_user(&l_l4_filter, u_l4_filter,
		sizeof(struct DWC_ETH_QOS_l3_l4_filter)))
		return -EFAULT;

	if ((l_l4_filter.filter_no + 1) > pdata->hw_feat.l3l4_filter_num) {
		printk(KERN_ALERT "%d filter is not supported in the HW\n",
			l_l4_filter.filter_no);
		return DWC_ETH_QOS_NO_HW_SUPPORT;
	}

	if (!pdata->l3_l4_filter) {
		hw_if->config_l3_l4_filter_enable(1);
		pdata->l3_l4_filter = 1;
	}

	/* configure the L4 filters */
	hw_if->config_l4_filters(l_l4_filter.filter_no,
			l_l4_filter.filter_enb_dis,
			tcp_udp,
			l_l4_filter.src_dst_addr_match,
			l_l4_filter.perfect_inverse_match);

	if (l_l4_filter.src_dst_addr_match)
		hw_if->update_l4_da_port_no(l_l4_filter.filter_no,
				l_l4_filter.port_no);
	else
		hw_if->update_l4_sa_port_no(l_l4_filter.filter_no,
				l_l4_filter.port_no);

	DBGPR_FILTER("Successfully %s %s %s %s Port number filtering on %d filter\n",
		(l_l4_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
		(tcp_udp ? "UDP" : "TCP"),
		(l_l4_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"),
		(l_l4_filter.src_dst_addr_match ? "DESTINATION" : "SOURCE"),
		l_l4_filter.filter_no);

	DBGPR_FILTER("<--DWC_ETH_QOS_config_tcp_udp_filters\n");

	return ret;
}


/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure VALN filtering. This function does following,
 * - enable/disable VLAN filtering.
 * - select perfect/hash filtering.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_vlan_filter(struct net_device *dev,
		struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct DWC_ETH_QOS_vlan_filter *u_vlan_filter =
		(struct DWC_ETH_QOS_vlan_filter *)req->ptr;
	struct DWC_ETH_QOS_vlan_filter l_vlan_filter;
	int ret = 0;

	DBGPR_FILTER("-->DWC_ETH_QOS_config_vlan_filter\n");

	if (copy_from_user(&l_vlan_filter, u_vlan_filter,
		sizeof(struct DWC_ETH_QOS_vlan_filter)))
		return -EFAULT;

	if ((l_vlan_filter.perfect_hash) &&
		(pdata->hw_feat.vlan_hash_en == 0)) {
		printk(KERN_ALERT "VLAN HASH filtering is not supported\n");
		return DWC_ETH_QOS_NO_HW_SUPPORT;
	}

	/* configure the vlan filter */
	hw_if->config_vlan_filtering(l_vlan_filter.filter_enb_dis,
					l_vlan_filter.perfect_hash,
					l_vlan_filter.perfect_inverse_match);
	pdata->vlan_hash_filtering = l_vlan_filter.perfect_hash;

	DBGPR_FILTER("Successfully %s VLAN %s filtering and %s matching\n",
		(l_vlan_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
		(l_vlan_filter.perfect_hash ? "HASH" : "PERFECT"),
		(l_vlan_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"));

	DBGPR_FILTER("<--DWC_ETH_QOS_config_vlan_filter\n");

	return ret;
}


/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to enable/disable ARP offloading feature.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_arp_offload(struct net_device *dev,
		struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct DWC_ETH_QOS_arp_offload *u_arp_offload =
		(struct DWC_ETH_QOS_arp_offload *)req->ptr;
	struct DWC_ETH_QOS_arp_offload l_arp_offload;
	int ret = 0;

	printk(KERN_ALERT "-->DWC_ETH_QOS_config_arp_offload\n");

	if (pdata->hw_feat.arp_offld_en == 0)
		return DWC_ETH_QOS_NO_HW_SUPPORT;

	if (copy_from_user(&l_arp_offload, u_arp_offload,
		sizeof(struct DWC_ETH_QOS_arp_offload)))
		return -EFAULT;

	/* configure the L3 filters */
	hw_if->config_arp_offload(req->flags);
	hw_if->update_arp_offload_ip_addr(l_arp_offload.ip_addr);
	pdata->arp_offload = req->flags;

	printk(KERN_ALERT "Successfully %s arp Offload\n",
		(req->flags ? "ENABLED" : "DISABLED"));

	printk(KERN_ALERT "<--DWC_ETH_QOS_config_arp_offload\n");

	return ret;
}


/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure L2 destination addressing filtering mode. This
 * function dose following,
 * - selects perfect/hash filtering.
 * - selects perfect/inverse matching.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_confing_l2_da_filter(struct net_device *dev,
		struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct DWC_ETH_QOS_l2_da_filter *u_l2_da_filter =
	  (struct DWC_ETH_QOS_l2_da_filter *)req->ptr;
	struct DWC_ETH_QOS_l2_da_filter l_l2_da_filter;
	int ret = 0;

	DBGPR_FILTER("-->DWC_ETH_QOS_confing_l2_da_filter\n");

	if (copy_from_user(&l_l2_da_filter, u_l2_da_filter,
	      sizeof(struct DWC_ETH_QOS_l2_da_filter)))
		return - EFAULT;

	if (l_l2_da_filter.perfect_hash) {
		if (pdata->hw_feat.hash_tbl_sz > 0)
			pdata->l2_filtering_mode = 1;
		else
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
	} else {
		if (pdata->max_addr_reg_cnt > 1)
			pdata->l2_filtering_mode = 0;
		else
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
	}

	/* configure L2 DA perfect/inverse_matching */
	hw_if->config_l2_da_perfect_inverse_match(l_l2_da_filter.perfect_inverse_match);

	DBGPR_FILTER("Successfully selected L2 %s filtering and %s DA matching\n",
		(l_l2_da_filter.perfect_hash ? "HASH" : "PERFECT"),
		(l_l2_da_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"));

	DBGPR_FILTER("<--DWC_ETH_QOS_confing_l2_da_filter\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to enable/disable mac loopback mode.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] flags – flag to indicate whether mac loopback mode to be
 *                  enabled/disabled.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_mac_loopback_mode(struct net_device *dev,
		unsigned int flags)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_config_mac_loopback_mode\n");

	if (flags && pdata->mac_loopback_mode) {
		printk(KERN_ALERT
			"MAC loopback mode is already enabled\n");
		return -EINVAL;
	}
	if (!flags && !pdata->mac_loopback_mode) {
		printk(KERN_ALERT
			"MAC loopback mode is already disabled\n");
		return -EINVAL;
	}
	pdata->mac_loopback_mode = !!flags;
	hw_if->config_mac_loopback_mode(flags);

	printk(KERN_ALERT "Succesfully %s MAC loopback mode\n",
		(flags ? "enabled" : "disabled"));

	DBGPR("<--DWC_ETH_QOS_config_mac_loopback_mode\n");

	return ret;
}

#ifdef DWC_ETH_QOS_ENABLE_DVLAN
static INT config_tx_dvlan_processing_via_reg(struct DWC_ETH_QOS_prv_data *pdata,
						UINT flags)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);

	printk(KERN_ALERT "--> config_tx_dvlan_processing_via_reg()\n");

	if (pdata->in_out & DWC_ETH_QOS_DVLAN_OUTER)
		hw_if->config_tx_outer_vlan(pdata->op_type,
					pdata->outer_vlan_tag);

	if (pdata->in_out & DWC_ETH_QOS_DVLAN_INNER)
		hw_if->config_tx_inner_vlan(pdata->op_type,
					pdata->inner_vlan_tag);

	if (flags == DWC_ETH_QOS_DVLAN_DISABLE)
		hw_if->config_mac_for_vlan_pkt(); /* restore default configurations */
	else
		hw_if->config_dvlan(1);

	printk(KERN_ALERT "<-- config_tx_dvlan_processing_via_reg()\n");

	return Y_SUCCESS;
}

static int config_tx_dvlan_processing_via_desc(struct DWC_ETH_QOS_prv_data *pdata,
						UINT flags)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);

	printk(KERN_ALERT "-->config_tx_dvlan_processing_via_desc\n");

	if (flags == DWC_ETH_QOS_DVLAN_DISABLE) {
		hw_if->config_mac_for_vlan_pkt(); /* restore default configurations */
		pdata->via_reg_or_desc = 0;
	} else {
		hw_if->config_dvlan(1);
	}

	if (pdata->in_out & DWC_ETH_QOS_DVLAN_INNER)
			MAC_IVLANTIRR_VLTI_UdfWr(1);

	if (pdata->in_out & DWC_ETH_QOS_DVLAN_OUTER)
			MAC_VLANTIRR_VLTI_UdfWr(1);

	printk(KERN_ALERT "<--config_tx_dvlan_processing_via_desc\n");

	return Y_SUCCESS;
}

/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to configure mac double vlan processing feature.
 *
 * \param[in] pdata - pointer to private data structure.
 * \param[in] flags – Each bit in this variable carry some information related
 *		      double vlan processing.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_tx_dvlan_processing(
		struct DWC_ETH_QOS_prv_data *pdata,
		struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_config_dvlan l_config_doubule_vlan,
					  *u_config_doubule_vlan = req->ptr;
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_config_tx_dvlan_processing\n");

	if(copy_from_user(&l_config_doubule_vlan, u_config_doubule_vlan,
				sizeof(struct DWC_ETH_QOS_config_dvlan))) {
		printk(KERN_ALERT "Failed to fetch Double vlan Struct info from user\n");
		return DWC_ETH_QOS_CONFIG_FAIL;
	}

	pdata->inner_vlan_tag = l_config_doubule_vlan.inner_vlan_tag;
	pdata->outer_vlan_tag = l_config_doubule_vlan.outer_vlan_tag;
	pdata->op_type = l_config_doubule_vlan.op_type;
	pdata->in_out = l_config_doubule_vlan.in_out;
	pdata->via_reg_or_desc = l_config_doubule_vlan.via_reg_or_desc;

	if (pdata->via_reg_or_desc == DWC_ETH_QOS_VIA_REG)
		ret = config_tx_dvlan_processing_via_reg(pdata, req->flags);
	else
		ret = config_tx_dvlan_processing_via_desc(pdata, req->flags);

	DBGPR("<--DWC_ETH_QOS_config_tx_dvlan_processing\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to configure mac double vlan processing feature.
 *
 * \param[in] pdata - pointer to private data structure.
 * \param[in] flags – Each bit in this variable carry some information related
 *		      double vlan processing.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_rx_dvlan_processing(
		struct DWC_ETH_QOS_prv_data *pdata, unsigned int flags)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_config_rx_dvlan_processing\n");

	hw_if->config_dvlan(1);
	if (flags == DWC_ETH_QOS_DVLAN_NONE) {
		hw_if->config_dvlan(0);
		hw_if->config_rx_outer_vlan_stripping(DWC_ETH_QOS_RX_NO_VLAN_STRIP);
		hw_if->config_rx_inner_vlan_stripping(DWC_ETH_QOS_RX_NO_VLAN_STRIP);
	} else if (flags == DWC_ETH_QOS_DVLAN_INNER) {
		hw_if->config_rx_outer_vlan_stripping(DWC_ETH_QOS_RX_NO_VLAN_STRIP);
		hw_if->config_rx_inner_vlan_stripping(DWC_ETH_QOS_RX_VLAN_STRIP_ALWAYS);
	} else if (flags == DWC_ETH_QOS_DVLAN_OUTER) {
		hw_if->config_rx_outer_vlan_stripping(DWC_ETH_QOS_RX_VLAN_STRIP_ALWAYS);
		hw_if->config_rx_inner_vlan_stripping(DWC_ETH_QOS_RX_NO_VLAN_STRIP);
	} else if (flags == DWC_ETH_QOS_DVLAN_BOTH) {
		hw_if->config_rx_outer_vlan_stripping(DWC_ETH_QOS_RX_VLAN_STRIP_ALWAYS);
		hw_if->config_rx_inner_vlan_stripping(DWC_ETH_QOS_RX_VLAN_STRIP_ALWAYS);
	} else {
		printk(KERN_ALERT "ERROR : double VLAN Rx configuration - Invalid argument");
		ret = DWC_ETH_QOS_CONFIG_FAIL;
	}

	DBGPR("<--DWC_ETH_QOS_config_rx_dvlan_processing\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to configure mac double vlan (svlan) processing feature.
 *
 * \param[in] pdata - pointer to private data structure.
 * \param[in] flags – Each bit in this variable carry some information related
 *		      double vlan processing.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_svlan(struct DWC_ETH_QOS_prv_data *pdata,
					unsigned int flags)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_config_svlan\n");

	ret = hw_if->config_svlan(flags);
	if (ret == Y_FAILURE)
		ret = DWC_ETH_QOS_CONFIG_FAIL;

	DBGPR("<--DWC_ETH_QOS_config_svlan\n");

	return ret;
}
#endif /* end of DWC_ETH_QOS_ENABLE_DVLAN */

static VOID DWC_ETH_QOS_config_timer_registers(
				struct DWC_ETH_QOS_prv_data *pdata)
{
		struct timespec now;
		struct hw_if_struct *hw_if = &(pdata->hw_if);
		u64 temp;

		DBGPR("-->DWC_ETH_QOS_config_timer_registers\n");

		/* program Sub Second Increment Reg */
		hw_if->config_sub_second_increment(DWC_ETH_QOS_SYSCLOCK);

		/* formula is :
		 * addend = 2^32/freq_div_ratio;
		 *
		 * where, freq_div_ratio = DWC_ETH_QOS_SYSCLOCK/50MHz
		 *
		 * hence, addend = ((2^32) * 50MHz)/DWC_ETH_QOS_SYSCLOCK;
		 *
		 * NOTE: DWC_ETH_QOS_SYSCLOCK should be >= 50MHz to
		 *       achive 20ns accuracy.
		 *
		 * 2^x * y == (y << x), hence
		 * 2^32 * 50000000 ==> (50000000 << 32)
		 * */
		temp = (u64)(50000000ULL << 32);
		pdata->default_addend = div_u64(temp, 62500000);

		hw_if->config_addend(pdata->default_addend);

		/* initialize system time */
		getnstimeofday(&now);
		hw_if->init_systime(now.tv_sec, now.tv_nsec);

		DBGPR("-->DWC_ETH_QOS_config_timer_registers\n");
}

/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to configure PTP offloading feature.
 *
 * \param[in] pdata - pointer to private data structure.
 * \param[in] flags – Each bit in this variable carry some information related
 *		      double vlan processing.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_ptpoffload(
		struct DWC_ETH_QOS_prv_data *pdata,
		struct DWC_ETH_QOS_config_ptpoffloading *u_conf_ptp)
{
	UINT pto_cntrl;
	UINT varMAC_TCR;
	struct DWC_ETH_QOS_config_ptpoffloading l_conf_ptp;
	struct hw_if_struct *hw_if = &(pdata->hw_if);


	if(copy_from_user(&l_conf_ptp, u_conf_ptp,
				sizeof(struct DWC_ETH_QOS_config_ptpoffloading))) {
		printk(KERN_ALERT "Failed to fetch Double vlan Struct info from user\n");
		return DWC_ETH_QOS_CONFIG_FAIL;
	}

	printk(KERN_ALERT"-->DWC_ETH_QOS_config_ptpoffload - %d\n",l_conf_ptp.mode);

	pto_cntrl = MAC_PTOCR_PTOEN; /* enable ptp offloading */
	varMAC_TCR = MAC_TCR_TSENA | MAC_TCR_TSIPENA | MAC_TCR_TSVER2ENA
			| MAC_TCR_TSCFUPDT | MAC_TCR_TSCTRLSSR;
	if (l_conf_ptp.mode == DWC_ETH_QOS_PTP_ORDINARY_SLAVE) {

		varMAC_TCR |= MAC_TCR_TSEVENTENA;
		pdata->ptp_offloading_mode = DWC_ETH_QOS_PTP_ORDINARY_SLAVE;

	} else if (l_conf_ptp.mode == DWC_ETH_QOS_PTP_TRASPARENT_SLAVE) {

		pto_cntrl |= MAC_PTOCR_APDREQEN;
		varMAC_TCR |= MAC_TCR_TSEVENTENA;
		varMAC_TCR |= MAC_TCR_SNAPTYPSEL_1;
		pdata->ptp_offloading_mode =
			DWC_ETH_QOS_PTP_TRASPARENT_SLAVE;

	} else if (l_conf_ptp.mode == DWC_ETH_QOS_PTP_ORDINARY_MASTER) {

		pto_cntrl |= MAC_PTOCR_ASYNCEN;
		varMAC_TCR |= MAC_TCR_TSEVENTENA;
		varMAC_TCR |= MAC_TCR_TSMASTERENA;
		pdata->ptp_offloading_mode = DWC_ETH_QOS_PTP_ORDINARY_MASTER;

	} else if(l_conf_ptp.mode == DWC_ETH_QOS_PTP_TRASPARENT_MASTER) {

		pto_cntrl |= MAC_PTOCR_ASYNCEN | MAC_PTOCR_APDREQEN;
		varMAC_TCR |= MAC_TCR_SNAPTYPSEL_1;
		varMAC_TCR |= MAC_TCR_TSEVENTENA;
		varMAC_TCR |= MAC_TCR_TSMASTERENA;
		pdata->ptp_offloading_mode =
			DWC_ETH_QOS_PTP_TRASPARENT_MASTER;

	} else if (l_conf_ptp.mode == DWC_ETH_QOS_PTP_PEER_TO_PEER_TRANSPARENT) {

		pto_cntrl |= MAC_PTOCR_APDREQEN;
		varMAC_TCR |= MAC_TCR_SNAPTYPSEL_3;
		pdata->ptp_offloading_mode =
			DWC_ETH_QOS_PTP_PEER_TO_PEER_TRANSPARENT;
	}

	pdata->ptp_offload = 1;
	if (l_conf_ptp.en_dis == DWC_ETH_QOS_PTP_OFFLOADING_DISABLE) {
		pto_cntrl = 0;
		varMAC_TCR = 0;
		pdata->ptp_offload = 0;
	}

	pto_cntrl |= (l_conf_ptp.domain_num << 8);
	hw_if->config_hw_time_stamping(varMAC_TCR);
	DWC_ETH_QOS_config_timer_registers(pdata);
	hw_if->config_ptpoffload_engine(pto_cntrl, l_conf_ptp.mc_uc);

	printk(KERN_ALERT"<--DWC_ETH_QOS_config_ptpoffload\n");

	return Y_SUCCESS;
}









/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to enable/disable pfc.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] flags – flag to indicate whether pfc to be enabled/disabled.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int DWC_ETH_QOS_config_pfc(struct net_device *dev,
		unsigned int flags)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_config_pfc\n");

	if (!pdata->hw_feat.dcb_en) {
		printk(KERN_ALERT "PFC is not supported\n");
		return DWC_ETH_QOS_NO_HW_SUPPORT;
	}

	hw_if->config_pfc(flags);

	printk(KERN_ALERT "Succesfully %s PFC(Priority Based Flow Control)\n",
		(flags ? "enabled" : "disabled"));

	DBGPR("<--DWC_ETH_QOS_config_pfc\n");

	return ret;
}

/*!
 * \brief Driver IOCTL routine
 *
 * \details This function is invoked by main ioctl function when
 * users request to configure various device features like,
 * PMT module, TX and RX PBL, TX and RX FIFO threshold level,
 * TX and RX OSF mode, SA insert/replacement, L2/L3/L4 and
 * VLAN filtering, AVB/DCB algorithm etc.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] req – pointer to ioctl structure.
 *
 * \return int
 *
 * \retval 0 - success
 * \retval negative - failure
 */

static int DWC_ETH_QOS_handle_prv_ioctl(struct DWC_ETH_QOS_prv_data *pdata,
					struct ifr_data_struct *req)
{
	unsigned int qInx = req->qInx;
	struct DWC_ETH_QOS_tx_wrapper_descriptor *tx_desc_data =
	    GET_TX_WRAPPER_DESC(qInx);
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data =
	    GET_RX_WRAPPER_DESC(qInx);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct net_device *dev = pdata->dev;
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_handle_prv_ioctl\n");

	if (qInx > DWC_ETH_QOS_QUEUE_CNT) {
		printk(KERN_ALERT "Queue number %d is invalid\n" \
				"Hardware has only %d Tx/Rx Queues\n",
				qInx, DWC_ETH_QOS_QUEUE_CNT);
		ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		return ret;
	}

	switch (req->cmd) {
	case DWC_ETH_QOS_POWERUP_MAGIC_CMD:
		if (pdata->hw_feat.mgk_sel) {
			ret = DWC_ETH_QOS_powerup(dev, DWC_ETH_QOS_IOCTL_CONTEXT);
			if (ret == 0)
				ret = DWC_ETH_QOS_CONFIG_SUCCESS;
			else
				ret = DWC_ETH_QOS_CONFIG_FAIL;
		} else {
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_POWERDOWN_MAGIC_CMD:
		if (pdata->hw_feat.mgk_sel) {
			ret =
			  DWC_ETH_QOS_powerdown(dev,
			    DWC_ETH_QOS_MAGIC_WAKEUP, DWC_ETH_QOS_IOCTL_CONTEXT);
			if (ret == 0)
				ret = DWC_ETH_QOS_CONFIG_SUCCESS;
			else
				ret = DWC_ETH_QOS_CONFIG_FAIL;
		} else {
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_POWERUP_REMOTE_WAKEUP_CMD:
		if (pdata->hw_feat.rwk_sel) {
			ret = DWC_ETH_QOS_powerup(dev, DWC_ETH_QOS_IOCTL_CONTEXT);
			if (ret == 0)
				ret = DWC_ETH_QOS_CONFIG_SUCCESS;
			else
				ret = DWC_ETH_QOS_CONFIG_FAIL;
		} else {
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_POWERDOWN_REMOTE_WAKEUP_CMD:
		if (pdata->hw_feat.rwk_sel) {
			ret = DWC_ETH_QOS_configure_remotewakeup(dev, req);
			if (ret == 0)
				ret = DWC_ETH_QOS_CONFIG_SUCCESS;
			else
				ret = DWC_ETH_QOS_CONFIG_FAIL;
		} else {
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_RX_THRESHOLD_CMD:
		rx_desc_data->rx_threshold_val = req->flags;
		hw_if->config_rx_threshold(qInx,
					rx_desc_data->rx_threshold_val);
		printk(KERN_ALERT "Configured Rx threshold with %d\n",
		       rx_desc_data->rx_threshold_val);
		break;

	case DWC_ETH_QOS_TX_THRESHOLD_CMD:
		tx_desc_data->tx_threshold_val = req->flags;
		hw_if->config_tx_threshold(qInx,
					tx_desc_data->tx_threshold_val);
		printk(KERN_ALERT "Configured Tx threshold with %d\n",
		       tx_desc_data->tx_threshold_val);
		break;

	case DWC_ETH_QOS_RSF_CMD:
		rx_desc_data->rsf_on = req->flags;
		hw_if->config_rsf_mode(qInx, rx_desc_data->rsf_on);
		printk(KERN_ALERT "Receive store and forward mode %s\n",
		       (rx_desc_data->rsf_on) ? "enabled" : "disabled");
		break;

	case DWC_ETH_QOS_TSF_CMD:
		tx_desc_data->tsf_on = req->flags;
		hw_if->config_tsf_mode(qInx, tx_desc_data->tsf_on);
		printk(KERN_ALERT "Transmit store and forward mode %s\n",
		       (tx_desc_data->tsf_on) ? "enabled" : "disabled");
		break;

	case DWC_ETH_QOS_OSF_CMD:
		tx_desc_data->osf_on = req->flags;
		hw_if->config_osf_mode(qInx, tx_desc_data->osf_on);
		printk(KERN_ALERT "Transmit DMA OSF mode is %s\n",
		       (tx_desc_data->osf_on) ? "enabled" : "disabled");
		break;

	case DWC_ETH_QOS_INCR_INCRX_CMD:
		pdata->incr_incrx = req->flags;
		hw_if->config_incr_incrx_mode(pdata->incr_incrx);
		printk(KERN_ALERT "%s mode is enabled\n",
		       (pdata->incr_incrx) ? "INCRX" : "INCR");
		break;

	case DWC_ETH_QOS_RX_PBL_CMD:
		rx_desc_data->rx_pbl = req->flags;
		DWC_ETH_QOS_config_rx_pbl(pdata, rx_desc_data->rx_pbl, qInx);
		break;

	case DWC_ETH_QOS_TX_PBL_CMD:
		tx_desc_data->tx_pbl = req->flags;
		DWC_ETH_QOS_config_tx_pbl(pdata, tx_desc_data->tx_pbl, qInx);
		break;

#ifdef DWC_ETH_QOS_ENABLE_DVLAN
	case DWC_ETH_QOS_DVLAN_TX_PROCESSING_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			ret = DWC_ETH_QOS_config_tx_dvlan_processing(pdata, req);
		} else {
			printk(KERN_ALERT "No HW support for Single/Double VLAN\n");
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;
	case DWC_ETH_QOS_DVLAN_RX_PROCESSING_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			ret = DWC_ETH_QOS_config_rx_dvlan_processing(pdata, req->flags);
		} else {
			printk(KERN_ALERT "No HW support for Single/Double VLAN\n");
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;
	case DWC_ETH_QOS_SVLAN_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			ret = DWC_ETH_QOS_config_svlan(pdata, req->flags);
		} else {
			printk(KERN_ALERT "No HW support for Single/Double VLAN\n");
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;
#endif /* end of DWC_ETH_QOS_ENABLE_DVLAN */
	case DWC_ETH_QOS_PTPOFFLOADING_CMD:
		if (pdata->hw_feat.tsstssel) {
			ret = DWC_ETH_QOS_config_ptpoffload(pdata,
					req->ptr);
		} else {
			printk(KERN_ALERT "No HW support for PTP\n");
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_SA0_DESC_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			pdata->tx_sa_ctrl_via_desc = req->flags;
			pdata->tx_sa_ctrl_via_reg = DWC_ETH_QOS_SA0_NONE;
			if (req->flags == DWC_ETH_QOS_SA0_NONE) {
				memcpy(pdata->mac_addr, pdata->dev->dev_addr,
				       DWC_ETH_QOS_MAC_ADDR_LEN);
			} else {
				memcpy(pdata->mac_addr, mac_addr0,
				       DWC_ETH_QOS_MAC_ADDR_LEN);
			}
			hw_if->configure_mac_addr0_reg(pdata->mac_addr);
			hw_if->configure_sa_via_reg(pdata->tx_sa_ctrl_via_reg);
			printk(KERN_ALERT
			       "SA will use MAC0 with descriptor for configuration %d\n",
			       pdata->tx_sa_ctrl_via_desc);
		} else {
			printk(KERN_ALERT
			       "Device doesn't supports SA Insertion/Replacement\n");
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_SA1_DESC_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			pdata->tx_sa_ctrl_via_desc = req->flags;
			pdata->tx_sa_ctrl_via_reg = DWC_ETH_QOS_SA1_NONE;
			if (req->flags == DWC_ETH_QOS_SA1_NONE) {
				memcpy(pdata->mac_addr, pdata->dev->dev_addr,
				       DWC_ETH_QOS_MAC_ADDR_LEN);
			} else {
				memcpy(pdata->mac_addr, mac_addr1,
				       DWC_ETH_QOS_MAC_ADDR_LEN);
			}
			hw_if->configure_mac_addr1_reg(pdata->mac_addr);
			hw_if->configure_sa_via_reg(pdata->tx_sa_ctrl_via_reg);
			printk(KERN_ALERT
			       "SA will use MAC1 with descriptor for configuration %d\n",
			       pdata->tx_sa_ctrl_via_desc);
		} else {
			printk(KERN_ALERT
			       "Device doesn't supports SA Insertion/Replacement\n");
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_SA0_REG_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			pdata->tx_sa_ctrl_via_reg = req->flags;
			pdata->tx_sa_ctrl_via_desc = DWC_ETH_QOS_SA0_NONE;
			if (req->flags == DWC_ETH_QOS_SA0_NONE) {
				memcpy(pdata->mac_addr, pdata->dev->dev_addr,
				       DWC_ETH_QOS_MAC_ADDR_LEN);
			} else {
				memcpy(pdata->mac_addr, mac_addr0,
				       DWC_ETH_QOS_MAC_ADDR_LEN);
			}
			hw_if->configure_mac_addr0_reg(pdata->mac_addr);
			hw_if->configure_sa_via_reg(pdata->tx_sa_ctrl_via_reg);
			printk(KERN_ALERT
			       "SA will use MAC0 with register for configuration %d\n",
			       pdata->tx_sa_ctrl_via_desc);
		} else {
			printk(KERN_ALERT
			       "Device doesn't supports SA Insertion/Replacement\n");
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_SA1_REG_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			pdata->tx_sa_ctrl_via_reg = req->flags;
			pdata->tx_sa_ctrl_via_desc = DWC_ETH_QOS_SA1_NONE;
			if (req->flags == DWC_ETH_QOS_SA1_NONE) {
				memcpy(pdata->mac_addr, pdata->dev->dev_addr,
				       DWC_ETH_QOS_MAC_ADDR_LEN);
			} else {
				memcpy(pdata->mac_addr, mac_addr1,
				       DWC_ETH_QOS_MAC_ADDR_LEN);
			}
			hw_if->configure_mac_addr1_reg(pdata->mac_addr);
			hw_if->configure_sa_via_reg(pdata->tx_sa_ctrl_via_reg);
			printk(KERN_ALERT
			       "SA will use MAC1 with register for configuration %d\n",
			       pdata->tx_sa_ctrl_via_desc);
		} else {
			printk(KERN_ALERT
			       "Device doesn't supports SA Insertion/Replacement\n");
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_SETUP_CONTEXT_DESCRIPTOR:
		if (pdata->hw_feat.sa_vlan_ins) {
			tx_desc_data->context_setup = req->context_setup;
			if (tx_desc_data->context_setup == 1) {
				printk(KERN_ALERT "Context descriptor will be transmitted"\
						" with every normal descriptor on %d DMA Channel\n",
						qInx);
			}
			else {
				printk(KERN_ALERT "Context descriptor will be setup"\
						" only if VLAN id changes %d\n", qInx);
			}
		}
		else {
			printk(KERN_ALERT
			       "Device doesn't support VLAN operations\n");
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;

	case DWC_ETH_QOS_GET_RX_QCNT:
		req->qInx = DWC_ETH_QOS_RX_QUEUE_CNT;
		break;

	case DWC_ETH_QOS_GET_TX_QCNT:
		req->qInx = DWC_ETH_QOS_TX_QUEUE_CNT;
		break;

	case DWC_ETH_QOS_GET_CONNECTED_SPEED:
		req->connected_speed = pdata->speed;
		break;

	case DWC_ETH_QOS_DCB_ALGORITHM:
		DWC_ETH_QOS_program_dcb_algorithm(pdata, req);
		break;

	case DWC_ETH_QOS_AVB_ALGORITHM:
		DWC_ETH_QOS_program_avb_algorithm(pdata, req);
		break;

	case DWC_ETH_QOS_RX_SPLIT_HDR_CMD:
		if (pdata->hw_feat.sph_en) {
			ret = DWC_ETH_QOS_config_rx_split_hdr_mode(dev, req->flags);
			if (ret == 0)
				ret = DWC_ETH_QOS_CONFIG_SUCCESS;
			else
				ret = DWC_ETH_QOS_CONFIG_FAIL;
		} else {
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;
	case DWC_ETH_QOS_L3_L4_FILTER_CMD:
		if (pdata->hw_feat.l3l4_filter_num > 0) {
			ret = DWC_ETH_QOS_config_l3_l4_filtering(dev, req->flags);
			if (ret == 0)
				ret = DWC_ETH_QOS_CONFIG_SUCCESS;
			else
				ret = DWC_ETH_QOS_CONFIG_FAIL;
		} else {
			ret = DWC_ETH_QOS_NO_HW_SUPPORT;
		}
		break;
	case DWC_ETH_QOS_IPV4_FILTERING_CMD:
		ret = DWC_ETH_QOS_config_ip4_filters(dev, req);
		break;
	case DWC_ETH_QOS_IPV6_FILTERING_CMD:
		ret = DWC_ETH_QOS_config_ip6_filters(dev, req);
		break;
	case DWC_ETH_QOS_UDP_FILTERING_CMD:
		ret = DWC_ETH_QOS_config_tcp_udp_filters(dev, req, 1);
		break;
	case DWC_ETH_QOS_TCP_FILTERING_CMD:
		ret = DWC_ETH_QOS_config_tcp_udp_filters(dev, req, 0);
		break;
	case DWC_ETH_QOS_VLAN_FILTERING_CMD:
		ret = DWC_ETH_QOS_config_vlan_filter(dev, req);
		break;
	case DWC_ETH_QOS_L2_DA_FILTERING_CMD:
		ret = DWC_ETH_QOS_confing_l2_da_filter(dev, req);
		break;
	case DWC_ETH_QOS_ARP_OFFLOAD_CMD:
		ret = DWC_ETH_QOS_config_arp_offload(dev, req);
		break;
	case DWC_ETH_QOS_AXI_PBL_CMD:
		pdata->axi_pbl = req->flags;
		hw_if->config_axi_pbl_val(pdata->axi_pbl);
		printk(KERN_ALERT "AXI PBL value: %d\n", pdata->axi_pbl);
		break;
	case DWC_ETH_QOS_AXI_WORL_CMD:
		pdata->axi_worl = req->flags;
		hw_if->config_axi_worl_val(pdata->axi_worl);
		printk(KERN_ALERT "AXI WORL value: %d\n", pdata->axi_worl);
		break;
	case DWC_ETH_QOS_AXI_RORL_CMD:
		pdata->axi_rorl = req->flags;
		hw_if->config_axi_rorl_val(pdata->axi_rorl);
		printk(KERN_ALERT "AXI RORL value: %d\n", pdata->axi_rorl);
		break;
	case DWC_ETH_QOS_MAC_LOOPBACK_MODE_CMD:
		ret = DWC_ETH_QOS_config_mac_loopback_mode(dev, req->flags);
		if (ret == 0)
			ret = DWC_ETH_QOS_CONFIG_SUCCESS;
		else
			ret = DWC_ETH_QOS_CONFIG_FAIL;
		break;
	case DWC_ETH_QOS_PFC_CMD:
		ret = DWC_ETH_QOS_config_pfc(dev, req->flags);
		break;
#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	case DWC_ETH_QOS_PG_TEST:
		ret = DWC_ETH_QOS_handle_pg_ioctl(pdata, (void *)req);
		break;
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */
	case DWC_ETH_QOS_PHY_LOOPBACK:
		ret = DWC_ETH_QOS_handle_phy_loopback(pdata, (void *)req);
		break;
	case DWC_ETH_QOS_MEM_ISO_TEST:
		ret = DWC_ETH_QOS_handle_mem_iso_ioctl(pdata, (void *)req);
		break;
	case DWC_ETH_QOS_CSR_ISO_TEST:
		ret = DWC_ETH_QOS_handle_csr_iso_ioctl(pdata, (void *)req);
		break;
	default:
		ret = -EOPNOTSUPP;
		printk(KERN_ALERT "Unsupported command call\n");
	}

	DBGPR("<--DWC_ETH_QOS_handle_prv_ioctl\n");

	return ret;
}


/*!
 * \brief control hw timestamping.
 *
 * \details This function is used to configure the MAC to enable/disable both
 * outgoing(Tx) and incoming(Rx) packets time stamping based on user input.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] ifr – pointer to IOCTL specific structure.
 *
 * \return int
 *
 * \retval 0 - success
 * \retval negative - failure
 */

static int DWC_ETH_QOS_handle_hwtstamp_ioctl(struct DWC_ETH_QOS_prv_data *pdata,
	struct ifreq *ifr)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct hwtstamp_config config;
	u32 ptp_v2 = 0;
	u32 tstamp_all = 0;
	u32 ptp_over_ipv4_udp = 0;
	u32 ptp_over_ipv6_udp = 0;
	u32 ptp_over_ethernet = 0;
	u32 snap_type_sel = 0;
	u32 ts_master_en = 0;
	u32 ts_event_en = 0;
	u32 av_8021asm_en = 0;
	u32 varMAC_TCR = 0;
	u64 temp = 0;
	struct timespec now;

	DBGPR_PTP("-->DWC_ETH_QOS_handle_hwtstamp_ioctl\n");

	if (!pdata->hw_feat.tsstssel) {
		printk(KERN_ALERT "No hw timestamping is available in this core\n");
		return -EOPNOTSUPP;
	}

	if (copy_from_user(&config, ifr->ifr_data,
		sizeof(struct hwtstamp_config)))
		return -EFAULT;

	DBGPR_PTP("config.flags = %#x, tx_type = %#x, rx_filter = %#x\n",
		config.flags, config.tx_type, config.rx_filter);

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		pdata->hwts_tx_en = 0;
		break;
	case HWTSTAMP_TX_ON:
		pdata->hwts_tx_en = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	/* time stamp no incoming packet at all */
	case HWTSTAMP_FILTER_NONE:
		config.rx_filter = HWTSTAMP_FILTER_NONE;
		break;

	/* PTP v1, UDP, any kind of event packet */
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
		/* take time stamp for all event messages */
		snap_type_sel = MAC_TCR_SNAPTYPSEL_1;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

	/* PTP v1, UDP, Sync packet */
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_SYNC;
		/* take time stamp for SYNC messages only */
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

	/* PTP v1, UDP, Delay_req packet */
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ;
		/* take time stamp for Delay_Req messages only */
		ts_master_en = MAC_TCR_TSMASTERENA;
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

	/* PTP v2, UDP, any kind of event packet */
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for all event messages */
		snap_type_sel = MAC_TCR_SNAPTYPSEL_1;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

	/* PTP v2, UDP, Sync packet */
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_SYNC;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for SYNC messages only */
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

	/* PTP v2, UDP, Delay_req packet */
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for Delay_Req messages only */
		ts_master_en = MAC_TCR_TSMASTERENA;
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

	/* PTP v2/802.AS1, any layer, any kind of event packet */
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for all event messages */
		snap_type_sel = MAC_TCR_SNAPTYPSEL_1;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		ptp_over_ethernet = MAC_TCR_TSIPENA;
		av_8021asm_en = MAC_TCR_AV8021ASMEN;
		break;

	/* PTP v2/802.AS1, any layer, Sync packet */
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for SYNC messages only */
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		ptp_over_ethernet = MAC_TCR_TSIPENA;
		av_8021asm_en = MAC_TCR_AV8021ASMEN;
		break;

	/* PTP v2/802.AS1, any layer, Delay_req packet */
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_DELAY_REQ;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for Delay_Req messages only */
		ts_master_en = MAC_TCR_TSMASTERENA;
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		ptp_over_ethernet = MAC_TCR_TSIPENA;
		av_8021asm_en = MAC_TCR_AV8021ASMEN;
		break;

	/* time stamp any incoming packet */
	case HWTSTAMP_FILTER_ALL:
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		tstamp_all = MAC_TCR_TSENALL;
		break;

	default:
		return -ERANGE;
	}
	pdata->hwts_rx_en = ((config.rx_filter == HWTSTAMP_FILTER_NONE) ? 0 : 1);

	if (!pdata->hwts_tx_en && !pdata->hwts_rx_en) {
		/* disable hw time stamping */
		hw_if->config_hw_time_stamping(varMAC_TCR);
	} else {
		varMAC_TCR = (MAC_TCR_TSENA | MAC_TCR_TSCFUPDT | MAC_TCR_TSCTRLSSR |
				tstamp_all | ptp_v2 | ptp_over_ethernet | ptp_over_ipv6_udp |
				ptp_over_ipv4_udp | ts_event_en | ts_master_en |
				snap_type_sel | av_8021asm_en);

		if (!pdata->one_nsec_accuracy)
			varMAC_TCR &= ~MAC_TCR_TSCTRLSSR;

		hw_if->config_hw_time_stamping(varMAC_TCR);

		/* program Sub Second Increment Reg */
		hw_if->config_sub_second_increment(DWC_ETH_QOS_SYSCLOCK);

		/* formula is :
		 * addend = 2^32/freq_div_ratio;
		 *
		 * where, freq_div_ratio = DWC_ETH_QOS_SYSCLOCK/50MHz
		 *
		 * hence, addend = ((2^32) * 50MHz)/DWC_ETH_QOS_SYSCLOCK;
		 *
		 * NOTE: DWC_ETH_QOS_SYSCLOCK should be >= 50MHz to
		 *       achive 20ns accuracy.
		 *
		 * 2^x * y == (y << x), hence
		 * 2^32 * 50000000 ==> (50000000 << 32)
		 * */
		temp = (u64)(50000000ULL << 32);
		pdata->default_addend = div_u64(temp, 62500000);

		hw_if->config_addend(pdata->default_addend);

		/* initialize system time */
		getnstimeofday(&now);
		hw_if->init_systime(now.tv_sec, now.tv_nsec);
	}

	DBGPR_PTP("config.flags = %#x, tx_type = %#x, rx_filter = %#x\n",
		config.flags, config.tx_type, config.rx_filter);

	DBGPR_PTP("<--DWC_ETH_QOS_handle_hwtstamp_ioctl\n");

	return (copy_to_user(ifr->ifr_data, &config,
		sizeof(struct hwtstamp_config))) ? -EFAULT : 0;
}


/*!
 * \brief Driver IOCTL routine
 *
 * \details This function is invoked by kernel when a user request an ioctl
 * which can't be handled by the generic interface code. Following operations
 * are performed in this functions.
 * - Configuring the PMT module.
 * - Configuring TX and RX PBL.
 * - Configuring the TX and RX FIFO threshold level.
 * - Configuring the TX and RX OSF mode.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] ifr – pointer to IOCTL specific structure.
 * \param[in] cmd – IOCTL command.
 *
 * \return int
 *
 * \retval 0 - success
 * \retval negative - failure
 */

static int DWC_ETH_QOS_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct ifr_data_struct *req = ifr->ifr_ifru.ifru_data;
	struct mii_ioctl_data *data = if_mii(ifr);
	unsigned int reg_val = 0;
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_ioctl\n");

#ifndef DWC_ETH_QOS_CONFIG_PGTEST
#ifdef AR_XXX
	if ((!netif_running(dev)) || (!pdata->phydev)) {
#else //Else for AR_XXX - We do not have PHY for now
	if (!netif_running(dev)) {
#endif
		DBGPR("<--DWC_ETH_QOS_ioctl - error\n");
		return -EINVAL;
	}
#endif

	spin_lock(&pdata->lock);
	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = pdata->phyaddr;
		printk(KERN_ALERT "PHY ID: SIOCGMIIPHY\n");
		break;

	case SIOCGMIIREG:
		ret =
		    DWC_ETH_QOS_mdio_read_direct(pdata, pdata->phyaddr,
				(data->reg_num & 0x1F), &reg_val);
		if (ret)
			ret = -EIO;

		data->val_out = reg_val;
		printk(KERN_ALERT "PHY ID: SIOCGMIIREG reg:%#x reg_val:%#x\n",
		       (data->reg_num & 0x1F), reg_val);
		break;

	case SIOCSMIIREG:
		printk(KERN_ALERT "PHY ID: SIOCSMIIPHY\n");
		break;

	case DWC_ETH_QOS_PRV_IOCTL:
		ret = DWC_ETH_QOS_handle_prv_ioctl(pdata, req);
		req->command_error = ret;
		break;

	case SIOCSHWTSTAMP:
		ret = DWC_ETH_QOS_handle_hwtstamp_ioctl(pdata, ifr);
		break;

	default:
		ret = -EOPNOTSUPP;
		printk(KERN_ALERT "Unsupported IOCTL call\n");
	}
	spin_unlock(&pdata->lock);

	DBGPR("<--DWC_ETH_QOS_ioctl\n");

	return ret;
}

/*!
* \brief API to change MTU.
*
* \details This function is invoked by upper layer when user changes
* MTU (Maximum Transfer Unit). The MTU is used by the Network layer
* to driver packet transmission. Ethernet has a default MTU of
* 1500Bytes. This value can be changed with ifconfig -
* ifconfig <interface_name> mtu <new_mtu_value>
*
* \param[in] dev - pointer to net_device structure
* \param[in] new_mtu - the new MTU for the device.
*
* \return integer
*
* \retval 0 - on success and -ve on failure.
*/

static INT DWC_ETH_QOS_change_mtu(struct net_device *dev, INT new_mtu)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	int max_frame = (new_mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN);

	DBGPR("-->DWC_ETH_QOS_change_mtu: new_mtu:%d\n", new_mtu);

#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	printk(KERN_ALERT "jumbo frames not supported with PG test\n");
	return -EOPNOTSUPP;
#endif
	if (dev->mtu == new_mtu) {
		printk(KERN_ALERT "%s: is already configured to %d mtu\n",
		       dev->name, new_mtu);
		return 0;
	}

	/* Supported frame sizes */
	if ((new_mtu < DWC_ETH_QOS_MIN_SUPPORTED_MTU) ||
	    (max_frame > DWC_ETH_QOS_MAX_SUPPORTED_MTU)) {
		printk(KERN_ALERT
		       "%s: invalid MTU, min %d and max %d MTU are supported\n",
		       dev->name, DWC_ETH_QOS_MIN_SUPPORTED_MTU,
		       DWC_ETH_QOS_MAX_SUPPORTED_MTU);
		return -EINVAL;
	}

	printk(KERN_ALERT "changing MTU from %d to %d\n", dev->mtu, new_mtu);

	DWC_ETH_QOS_stop_dev(pdata);

	if (max_frame <= 2048)
		pdata->rx_buffer_len = 2048;
	else
		pdata->rx_buffer_len = PAGE_SIZE; /* in case of JUMBO frame,
						max buffer allocated is
						PAGE_SIZE */

	if ((max_frame == ETH_FRAME_LEN + ETH_FCS_LEN) ||
	    (max_frame == ETH_FRAME_LEN + ETH_FCS_LEN + VLAN_HLEN))
		pdata->rx_buffer_len =
		    DWC_ETH_QOS_ETH_FRAME_LEN;

	dev->mtu = new_mtu;

	DWC_ETH_QOS_start_dev(pdata);

	DBGPR("<--DWC_ETH_QOS_change_mtu\n");

	return 0;
}

#ifdef DWC_ETH_QOS_QUEUE_SELECT_ALGO
u16	DWC_ETH_QOS_select_queue(struct net_device *dev,
			struct sk_buff *skb)
{
	static u16 txqueue_select = 0;

	DBGPR("-->DWC_ETH_QOS_select_queue\n");

	txqueue_select = skb_tx_hash(dev, skb);

	DBGPR("<--DWC_ETH_QOS_select_queue txqueue-select:%d\n",
		txqueue_select);

	return txqueue_select;
}
#endif

unsigned int crc32_snps_le(unsigned int initval, unsigned char *data, unsigned int size)
{
	unsigned int crc = initval;
	unsigned int poly = 0x04c11db7;
	unsigned int temp = 0;
	unsigned char my_data = 0;
	int bit_count;
	for(bit_count = 0; bit_count < size; bit_count++) {
		if((bit_count % 8) == 0) my_data = data[bit_count/8];
		DBGPR_FILTER("%s my_data = %x crc=%x\n", __func__, my_data,crc);
		temp = ((crc >> 31) ^  my_data) &  0x1;
		crc <<= 1;
		if(temp != 0) crc ^= poly;
		my_data >>=1;
	}
		DBGPR_FILTER("%s my_data = %x crc=%x\n", __func__, my_data,crc);
	return ~crc;
}



/*!
* \brief API to delete vid to HW filter.
*
* \details This function is invoked by upper layer when a VLAN id is removed.
* This function deletes the VLAN id from the HW filter.
* vlan id can be removed with vconfig -
* vconfig rem <interface_name > <vlan_id>
*
* \param[in] dev - pointer to net_device structure
* \param[in] vid - vlan id to be removed.
*
* \return void
*/
static int DWC_ETH_QOS_vlan_rx_kill_vid(struct net_device *dev, __be16 proto, u16 vid)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned short new_index, old_index;
	int crc32_val = 0;
	unsigned int enb_12bit_vhash;

	printk(KERN_ALERT "-->DWC_ETH_QOS_vlan_rx_kill_vid: vid = %d\n",
		vid);

	if (pdata->vlan_hash_filtering) {
		crc32_val = (bitrev32(~crc32_le(~0, (unsigned char *)&vid, 2)) >> 28);

		enb_12bit_vhash = hw_if->get_vlan_tag_comparison();
		if (enb_12bit_vhash) {
			/* neget 4-bit crc value for 12-bit VLAN hash comparison */
			new_index = (1 << (~crc32_val & 0xF));
		} else {
			new_index = (1 << (crc32_val & 0xF));
		}

		old_index = hw_if->get_vlan_hash_table_reg();
		old_index &= ~new_index;
		hw_if->update_vlan_hash_table_reg(old_index);
		pdata->vlan_ht_or_id = old_index;
	} else {
		/* By default, receive only VLAN pkt with VID = 1
		 * becasue writting 0 will pass all VLAN pkt */
		hw_if->update_vlan_id(1);
		pdata->vlan_ht_or_id = 1;
	}

	printk(KERN_ALERT "<--DWC_ETH_QOS_vlan_rx_kill_vid\n");

	//FIXME: Check if any errors need to be returned in case of a failure.
	return 0;
}


/*!
* \brief API to add vid to HW filter.
*
* \details This function is invoked by upper layer when a new VALN id is
* registered. This function updates the HW filter with new VLAN id.
* New vlan id can be added with vconfig -
* vconfig add <interface_name > <vlan_id>
*
* \param[in] dev - pointer to net_device structure
* \param[in] vid - new vlan id.
*
* \return void
*/
static int DWC_ETH_QOS_vlan_rx_add_vid(struct net_device *dev, __be16 proto, u16 vid)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned short new_index, old_index;
	int crc32_val = 0;
	unsigned int enb_12bit_vhash;

	printk(KERN_ALERT "-->DWC_ETH_QOS_vlan_rx_add_vid: vid = %d\n",
		vid);

	if (pdata->vlan_hash_filtering) {
		/* The upper 4 bits of the calculated CRC are used to
		 * index the content of the VLAN Hash Table Reg.
		 * */
		crc32_val = (bitrev32(~crc32_le(~0, (unsigned char *)&vid, 2)) >> 28);

		/* These 4(0xF) bits determines the bit within the
		 * VLAN Hash Table Reg 0
		 * */
		enb_12bit_vhash = hw_if->get_vlan_tag_comparison();
		if (enb_12bit_vhash) {
			/* neget 4-bit crc value for 12-bit VLAN hash comparison */
			new_index = (1 << (~crc32_val & 0xF));
		} else {
			new_index = (1 << (crc32_val & 0xF));
		}

		old_index = hw_if->get_vlan_hash_table_reg();
		old_index |= new_index;
		hw_if->update_vlan_hash_table_reg(old_index);
		pdata->vlan_ht_or_id = old_index;
	} else {
		hw_if->update_vlan_id(vid);
		pdata->vlan_ht_or_id = vid;
	}

	printk(KERN_ALERT "<--DWC_ETH_QOS_vlan_rx_add_vid\n");

	//FIXME: Check if any errors need to be returned in case of a failure.
	return 0;
}

/*!
 * \brief API called to put device in powerdown mode
 *
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to move the device to power down state. Following operations
 * are performed in this function.
 * - stop the phy.
 * - stop the queue.
 * - Disable napi.
 * - Stop DMA TX and RX process.
 * - Enable power down mode using PMT module.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] wakeup_type – remote wake-on-lan or magic packet.
 * \param[in] caller – netif_detach gets called conditionally based
 *                     on caller, IOCTL or DRIVER-suspend
 *
 * \return int
 *
 * \retval zero on success and -ve number on failure.
 */

INT DWC_ETH_QOS_powerdown(struct net_device *dev, UINT wakeup_type,
		UINT caller)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	ULONG flags;

	DBGPR(KERN_ALERT "-->DWC_ETH_QOS_powerdown\n");

	if (!dev || !netif_running(dev) ||
	    (caller == DWC_ETH_QOS_IOCTL_CONTEXT && pdata->power_down)) {
		printk(KERN_ALERT
		       "Device is already powered down and will powerup for %s\n",
		       DWC_ETH_QOS_POWER_DOWN_TYPE(pdata));
		DBGPR("<--DWC_ETH_QOS_powerdown\n");
		return -EINVAL;
	}

	if (pdata->phydev)
		phy_stop(pdata->phydev);

	spin_lock_irqsave(&pdata->pmt_lock, flags);

	if (caller == DWC_ETH_QOS_DRIVER_CONTEXT)
		netif_device_detach(dev);

	netif_tx_disable(dev);
	DWC_ETH_QOS_all_ch_napi_disable(pdata);

	/* stop DMA TX/RX */
	DWC_ETH_QOS_stop_all_ch_tx_dma(pdata);
	DWC_ETH_QOS_stop_all_ch_rx_dma(pdata);

	/* enable power down mode by programming the PMT regs */
	if (wakeup_type & DWC_ETH_QOS_REMOTE_WAKEUP)
		hw_if->enable_remote_pmt();
	if (wakeup_type & DWC_ETH_QOS_MAGIC_WAKEUP)
		hw_if->enable_magic_pmt();
	pdata->power_down_type = wakeup_type;

	if (caller == DWC_ETH_QOS_IOCTL_CONTEXT)
		pdata->power_down = 1;

	spin_unlock_irqrestore(&pdata->pmt_lock, flags);

	DBGPR("<--DWC_ETH_QOS_powerdown\n");

	return 0;
}

/*!
 * \brief API to powerup the device
 *
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to move the device to out of power down state. Following
 * operations are performed in this function.
 * - Wakeup the device using PMT module if supported.
 * - Starts the phy.
 * - Enable MAC and DMA TX and RX process.
 * - Enable napi.
 * - Starts the queue.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] caller – netif_attach gets called conditionally based
 *                     on caller, IOCTL or DRIVER-suspend
 *
 * \return int
 *
 * \retval zero on success and -ve number on failure.
 */

INT DWC_ETH_QOS_powerup(struct net_device *dev, UINT caller)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	ULONG flags;

	DBGPR("-->DWC_ETH_QOS_powerup\n");

	if (!dev || !netif_running(dev) ||
	    (caller == DWC_ETH_QOS_IOCTL_CONTEXT && !pdata->power_down)) {
		printk(KERN_ALERT "Device is already powered up\n");
		DBGPR(KERN_ALERT "<--DWC_ETH_QOS_powerup\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&pdata->pmt_lock, flags);

	if (pdata->power_down_type & DWC_ETH_QOS_MAGIC_WAKEUP) {
		hw_if->disable_magic_pmt();
		pdata->power_down_type &= ~DWC_ETH_QOS_MAGIC_WAKEUP;
	}

	if (pdata->power_down_type & DWC_ETH_QOS_REMOTE_WAKEUP) {
		hw_if->disable_remote_pmt();
		pdata->power_down_type &= ~DWC_ETH_QOS_REMOTE_WAKEUP;
	}

	pdata->power_down = 0;

	if (pdata->phydev)
		phy_start(pdata->phydev);

	/* enable MAC TX/RX */
	hw_if->start_mac_tx_rx();

	/* enable DMA TX/RX */
	DWC_ETH_QOS_start_all_ch_tx_dma(pdata);
	DWC_ETH_QOS_start_all_ch_rx_dma(pdata);

	if (caller == DWC_ETH_QOS_DRIVER_CONTEXT)
		netif_device_attach(dev);

	DWC_ETH_QOS_napi_enable_mq(pdata);

	netif_tx_start_all_queues(dev);

	spin_unlock_irqrestore(&pdata->pmt_lock, flags);

	DBGPR("<--DWC_ETH_QOS_powerup\n");

	return 0;
}

/*!
 * \brief API to configure remote wakeup
 *
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to move the device to power down state using remote wakeup.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to ioctl data structure.
 *
 * \return int
 *
 * \retval zero on success and -ve number on failure.
 */

INT DWC_ETH_QOS_configure_remotewakeup(struct net_device *dev,
				       struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);

	if (!dev || !netif_running(dev) || !pdata->hw_feat.rwk_sel
	    || pdata->power_down) {
		printk(KERN_ALERT
		       "Device is already powered down and will powerup for %s\n",
		       DWC_ETH_QOS_POWER_DOWN_TYPE(pdata));
		return -EINVAL;
	}

	hw_if->configure_rwk_filter(req->rwk_filter_values,
				    req->rwk_filter_length);

	DWC_ETH_QOS_powerdown(dev, DWC_ETH_QOS_REMOTE_WAKEUP,
			DWC_ETH_QOS_IOCTL_CONTEXT);

	return 0;
}

/*!
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to change the RX DMA PBL value. This function will program
 * the device to configure the user specified RX PBL value.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] rx_pbl – RX DMA pbl value to be programmed.
 *
 * \return void
 *
 * \retval none
 */

static void DWC_ETH_QOS_config_rx_pbl(struct DWC_ETH_QOS_prv_data *pdata,
				      UINT rx_pbl,
				      UINT qInx)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT pblx8_val = 0;

	DBGPR("-->DWC_ETH_QOS_config_rx_pbl: %d\n", rx_pbl);

	switch (rx_pbl) {
	case DWC_ETH_QOS_PBL_1:
	case DWC_ETH_QOS_PBL_2:
	case DWC_ETH_QOS_PBL_4:
	case DWC_ETH_QOS_PBL_8:
	case DWC_ETH_QOS_PBL_16:
	case DWC_ETH_QOS_PBL_32:
		hw_if->config_rx_pbl_val(qInx, rx_pbl);
		hw_if->config_pblx8(qInx, 0);
		break;
	case DWC_ETH_QOS_PBL_64:
	case DWC_ETH_QOS_PBL_128:
	case DWC_ETH_QOS_PBL_256:
		hw_if->config_rx_pbl_val(qInx, rx_pbl / 8);
		hw_if->config_pblx8(qInx, 1);
		pblx8_val = 1;
		break;
	}

	switch (pblx8_val) {
		case 0:
			printk(KERN_ALERT "Tx PBL[%d] value: %d\n",
					qInx, hw_if->get_tx_pbl_val(qInx));
			printk(KERN_ALERT "Rx PBL[%d] value: %d\n",
					qInx, hw_if->get_rx_pbl_val(qInx));
			break;
		case 1:
			printk(KERN_ALERT "Tx PBL[%d] value: %d\n",
					qInx, (hw_if->get_tx_pbl_val(qInx) * 8));
			printk(KERN_ALERT "Rx PBL[%d] value: %d\n",
					qInx, (hw_if->get_rx_pbl_val(qInx) * 8));
			break;
	}

	DBGPR("<--DWC_ETH_QOS_config_rx_pbl\n");
}

/*!
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to change the TX DMA PBL value. This function will program
 * the device to configure the user specified TX PBL value.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] tx_pbl – TX DMA pbl value to be programmed.
 *
 * \return void
 *
 * \retval none
 */

static void DWC_ETH_QOS_config_tx_pbl(struct DWC_ETH_QOS_prv_data *pdata,
				      UINT tx_pbl,
				      UINT qInx)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT pblx8_val = 0;

	DBGPR("-->DWC_ETH_QOS_config_tx_pbl: %d\n", tx_pbl);

	switch (tx_pbl) {
	case DWC_ETH_QOS_PBL_1:
	case DWC_ETH_QOS_PBL_2:
	case DWC_ETH_QOS_PBL_4:
	case DWC_ETH_QOS_PBL_8:
	case DWC_ETH_QOS_PBL_16:
	case DWC_ETH_QOS_PBL_32:
		hw_if->config_tx_pbl_val(qInx, tx_pbl);
		hw_if->config_pblx8(qInx, 0);
		break;
	case DWC_ETH_QOS_PBL_64:
	case DWC_ETH_QOS_PBL_128:
	case DWC_ETH_QOS_PBL_256:
		hw_if->config_tx_pbl_val(qInx, tx_pbl / 8);
		hw_if->config_pblx8(qInx, 1);
		pblx8_val = 1;
		break;
	}

	switch (pblx8_val) {
		case 0:
			printk(KERN_ALERT "Tx PBL[%d] value: %d\n",
					qInx, hw_if->get_tx_pbl_val(qInx));
			printk(KERN_ALERT "Rx PBL[%d] value: %d\n",
					qInx, hw_if->get_rx_pbl_val(qInx));
			break;
		case 1:
			printk(KERN_ALERT "Tx PBL[%d] value: %d\n",
					qInx, (hw_if->get_tx_pbl_val(qInx) * 8));
			printk(KERN_ALERT "Rx PBL[%d] value: %d\n",
					qInx, (hw_if->get_rx_pbl_val(qInx) * 8));
			break;
	}

	DBGPR("<--DWC_ETH_QOS_config_tx_pbl\n");
}


/*!
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to select the DCB algorithm.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] req – pointer to ioctl data structure.
 *
 * \return void
 *
 * \retval none
 */

static void DWC_ETH_QOS_program_dcb_algorithm(struct DWC_ETH_QOS_prv_data *pdata,
		struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_dcb_algorithm l_dcb_struct, *u_dcb_struct =
		(struct DWC_ETH_QOS_dcb_algorithm *)req->ptr;
	struct hw_if_struct *hw_if = &pdata->hw_if;

	DBGPR("-->DWC_ETH_QOS_program_dcb_algorithm\n");

	if(copy_from_user(&l_dcb_struct, u_dcb_struct,
				sizeof(struct DWC_ETH_QOS_dcb_algorithm)))
		printk(KERN_ALERT "Failed to fetch DCB Struct info from user\n");

	hw_if->set_tx_queue_operating_mode(l_dcb_struct.qInx,
		(UINT)l_dcb_struct.op_mode);
	hw_if->set_dcb_algorithm(l_dcb_struct.algorithm);
	hw_if->set_dcb_queue_weight(l_dcb_struct.qInx, l_dcb_struct.weight);

	DBGPR("<--DWC_ETH_QOS_program_dcb_algorithm\n");

	return;
}


/*!
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to select the AVB algorithm. This function also configures other
 * parameters like send and idle slope, high and low credit.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] req – pointer to ioctl data structure.
 *
 * \return void
 *
 * \retval none
 */

static void DWC_ETH_QOS_program_avb_algorithm(struct DWC_ETH_QOS_prv_data *pdata,
		struct ifr_data_struct *req)
{
	struct DWC_ETH_QOS_avb_algorithm l_avb_struct, *u_avb_struct =
		(struct DWC_ETH_QOS_avb_algorithm *)req->ptr;
	struct hw_if_struct *hw_if = &pdata->hw_if;

	DBGPR("-->DWC_ETH_QOS_program_avb_algorithm\n");

	if(copy_from_user(&l_avb_struct, u_avb_struct,
				sizeof(struct DWC_ETH_QOS_avb_algorithm)))
		printk(KERN_ALERT "Failed to fetch AVB Struct info from user\n");

	hw_if->set_tx_queue_operating_mode(l_avb_struct.qInx,
		(UINT)l_avb_struct.op_mode);
	hw_if->set_avb_algorithm(l_avb_struct.qInx, l_avb_struct.algorithm);
	hw_if->config_credit_control(l_avb_struct.qInx, l_avb_struct.cc);
	hw_if->config_send_slope(l_avb_struct.qInx, l_avb_struct.send_slope);
	hw_if->config_idle_slope(l_avb_struct.qInx, l_avb_struct.idle_slope);
	hw_if->config_high_credit(l_avb_struct.qInx, l_avb_struct.hi_credit);
	hw_if->config_low_credit(l_avb_struct.qInx, l_avb_struct.low_credit);

	DBGPR("<--DWC_ETH_QOS_program_avb_algorithm\n");

	return;
}

/*!
* \brief API to read the registers & prints the value.
* \details This function will read all the device register except
* data register & prints the values.
*
* \return none
*/
#if 0
void dbgpr_regs(void)
{
	UINT val0;
	UINT val1;
	UINT val2;
	UINT val3;
	UINT val4;
	UINT val5;

	MAC_PMTCSR_RgRd(val0);
	MMC_RXICMP_ERR_OCTETS_RgRd(val1);
	MMC_RXICMP_GD_OCTETS_RgRd(val2);
	MMC_RXTCP_ERR_OCTETS_RgRd(val3);
	MMC_RXTCP_GD_OCTETS_RgRd(val4);
	MMC_RXUDP_ERR_OCTETS_RgRd(val5);

	DBGPR("dbgpr_regs: MAC_PMTCSR:%#x\n"
	      "dbgpr_regs: MMC_RXICMP_ERR_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXICMP_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXTCP_ERR_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXTCP_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXUDP_ERR_OCTETS:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXUDP_GD_OCTETS_RgRd(val0);
	MMC_RXIPV6_NOPAY_OCTETS_RgRd(val1);
	MMC_RXIPV6_HDRERR_OCTETS_RgRd(val2);
	MMC_RXIPV6_GD_OCTETS_RgRd(val3);
	MMC_RXIPV4_UDSBL_OCTETS_RgRd(val4);
	MMC_RXIPV4_FRAG_OCTETS_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_RXUDP_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_NOPAY_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_HDRERR_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_UDSBL_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_FRAG_OCTETS:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXIPV4_NOPAY_OCTETS_RgRd(val0);
	MMC_RXIPV4_HDRERR_OCTETS_RgRd(val1);
	MMC_RXIPV4_GD_OCTETS_RgRd(val2);
	MMC_RXICMP_ERR_PKTS_RgRd(val3);
	MMC_RXICMP_GD_PKTS_RgRd(val4);
	MMC_RXTCP_ERR_PKTS_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_RXIPV4_NOPAY_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_HDRERR_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXICMP_ERR_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXICMP_GD_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXTCP_ERR_PKTS:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXTCP_GD_PKTS_RgRd(val0);
	MMC_RXUDP_ERR_PKTS_RgRd(val1);
	MMC_RXUDP_GD_PKTS_RgRd(val2);
	MMC_RXIPV6_NOPAY_PKTS_RgRd(val3);
	MMC_RXIPV6_HDRERR_PKTS_RgRd(val4);
	MMC_RXIPV6_GD_PKTS_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_RXTCP_GD_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXUDP_ERR_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXUDP_GD_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_NOPAY_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_HDRERR_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_GD_PKTS:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXIPV4_UBSBL_PKTS_RgRd(val0);
	MMC_RXIPV4_FRAG_PKTS_RgRd(val1);
	MMC_RXIPV4_NOPAY_PKTS_RgRd(val2);
	MMC_RXIPV4_HDRERR_PKTS_RgRd(val3);
	MMC_RXIPV4_GD_PKTS_RgRd(val4);
	MMC_RXCTRLPACKETS_G_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_RXIPV4_UBSBL_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_FRAG_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_NOPAY_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_HDRERR_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_GD_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXCTRLPACKETS_G:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXRCVERROR_RgRd(val0);
	MMC_RXWATCHDOGERROR_RgRd(val1);
	MMC_RXVLANPACKETS_GB_RgRd(val2);
	MMC_RXFIFOOVERFLOW_RgRd(val3);
	MMC_RXPAUSEPACKETS_RgRd(val4);
	MMC_RXOUTOFRANGETYPE_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_RXRCVERROR:%#x\n"
	      "dbgpr_regs: MMC_RXWATCHDOGERROR:%#x\n"
	      "dbgpr_regs: MMC_RXVLANPACKETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RXFIFOOVERFLOW:%#x\n"
	      "dbgpr_regs: MMC_RXPAUSEPACKETS:%#x\n"
	      "dbgpr_regs: MMC_RXOUTOFRANGETYPE:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXLENGTHERROR_RgRd(val0);
	MMC_RXUNICASTPACKETS_G_RgRd(val1);
	MMC_RX1024TOMAXOCTETS_GB_RgRd(val2);
	MMC_RX512TO1023OCTETS_GB_RgRd(val3);
	MMC_RX256TO511OCTETS_GB_RgRd(val4);
	MMC_RX128TO255OCTETS_GB_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_RXLENGTHERROR:%#x\n"
	      "dbgpr_regs: MMC_RXUNICASTPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_RX1024TOMAXOCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RX512TO1023OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RX256TO511OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RX128TO255OCTETS_GB:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RX65TO127OCTETS_GB_RgRd(val0);
	MMC_RX64OCTETS_GB_RgRd(val1);
	MMC_RXOVERSIZE_G_RgRd(val2);
	MMC_RXUNDERSIZE_G_RgRd(val3);
	MMC_RXJABBERERROR_RgRd(val4);
	MMC_RXRUNTERROR_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_RX65TO127OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RX64OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RXOVERSIZE_G:%#x\n"
	      "dbgpr_regs: MMC_RXUNDERSIZE_G:%#x\n"
	      "dbgpr_regs: MMC_RXJABBERERROR:%#x\n"
	      "dbgpr_regs: MMC_RXRUNTERROR:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXALIGNMENTERROR_RgRd(val0);
	MMC_RXCRCERROR_RgRd(val1);
	MMC_RXMULTICASTPACKETS_G_RgRd(val2);
	MMC_RXBROADCASTPACKETS_G_RgRd(val3);
	MMC_RXOCTETCOUNT_G_RgRd(val4);
	MMC_RXOCTETCOUNT_GB_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_RXALIGNMENTERROR:%#x\n"
	      "dbgpr_regs: MMC_RXCRCERROR:%#x\n"
	      "dbgpr_regs: MMC_RXMULTICASTPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_RXBROADCASTPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_RXOCTETCOUNT_G:%#x\n"
	      "dbgpr_regs: MMC_RXOCTETCOUNT_GB:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXPACKETCOUNT_GB_RgRd(val0);
	MMC_TXOVERSIZE_G_RgRd(val1);
	MMC_TXVLANPACKETS_G_RgRd(val2);
	MMC_TXPAUSEPACKETS_RgRd(val3);
	MMC_TXEXCESSDEF_RgRd(val4);
	MMC_TXPACKETSCOUNT_G_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_RXPACKETCOUNT_GB:%#x\n"
	      "dbgpr_regs: MMC_TXOVERSIZE_G:%#x\n"
	      "dbgpr_regs: MMC_TXVLANPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_TXPAUSEPACKETS:%#x\n"
	      "dbgpr_regs: MMC_TXEXCESSDEF:%#x\n"
	      "dbgpr_regs: MMC_TXPACKETSCOUNT_G:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_TXOCTETCOUNT_G_RgRd(val0);
	MMC_TXCARRIERERROR_RgRd(val1);
	MMC_TXEXESSCOL_RgRd(val2);
	MMC_TXLATECOL_RgRd(val3);
	MMC_TXDEFERRED_RgRd(val4);
	MMC_TXMULTICOL_G_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_TXOCTETCOUNT_G:%#x\n"
	      "dbgpr_regs: MMC_TXCARRIERERROR:%#x\n"
	      "dbgpr_regs: MMC_TXEXESSCOL:%#x\n"
	      "dbgpr_regs: MMC_TXLATECOL:%#x\n"
	      "dbgpr_regs: MMC_TXDEFERRED:%#x\n"
	      "dbgpr_regs: MMC_TXMULTICOL_G:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_TXSINGLECOL_G_RgRd(val0);
	MMC_TXUNDERFLOWERROR_RgRd(val1);
	MMC_TXBROADCASTPACKETS_GB_RgRd(val2);
	MMC_TXMULTICASTPACKETS_GB_RgRd(val3);
	MMC_TXUNICASTPACKETS_GB_RgRd(val4);
	MMC_TX1024TOMAXOCTETS_GB_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_TXSINGLECOL_G:%#x\n"
	      "dbgpr_regs: MMC_TXUNDERFLOWERROR:%#x\n"
	      "dbgpr_regs: MMC_TXBROADCASTPACKETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TXMULTICASTPACKETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TXUNICASTPACKETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX1024TOMAXOCTETS_GB:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_TX512TO1023OCTETS_GB_RgRd(val0);
	MMC_TX256TO511OCTETS_GB_RgRd(val1);
	MMC_TX128TO255OCTETS_GB_RgRd(val2);
	MMC_TX65TO127OCTETS_GB_RgRd(val3);
	MMC_TX64OCTETS_GB_RgRd(val4);
	MMC_TXMULTICASTPACKETS_G_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_TX512TO1023OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX256TO511OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX128TO255OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX65TO127OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX64OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TXMULTICASTPACKETS_G:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_TXBROADCASTPACKETS_G_RgRd(val0);
	MMC_TXPACKETCOUNT_GB_RgRd(val1);
	MMC_TXOCTETCOUNT_GB_RgRd(val2);
	MMC_IPC_INTR_RX_RgRd(val3);
	MMC_IPC_INTR_MASK_RX_RgRd(val4);
	MMC_INTR_MASK_TX_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_TXBROADCASTPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_TXPACKETCOUNT_GB:%#x\n"
	      "dbgpr_regs: MMC_TXOCTETCOUNT_GB:%#x\n"
	      "dbgpr_regs: MMC_IPC_INTR_RX:%#x\n"
	      "dbgpr_regs: MMC_IPC_INTR_MASK_RX:%#x\n"
	      "dbgpr_regs: MMC_INTR_MASK_TX:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_INTR_MASK_RX_RgRd(val0);
	MMC_INTR_TX_RgRd(val1);
	MMC_INTR_RX_RgRd(val2);
	MMC_CNTRL_RgRd(val3);
	MAC_MA1LR_RgRd(val4);
	MAC_MA1HR_RgRd(val5);

	DBGPR("dbgpr_regs: MMC_INTR_MASK_RX:%#x\n"
	      "dbgpr_regs: MMC_INTR_TX:%#x\n"
	      "dbgpr_regs: MMC_INTR_RX:%#x\n"
	      "dbgpr_regs: MMC_CNTRL:%#x\n"
	      "dbgpr_regs: MAC_MA1LR:%#x\n"
	      "dbgpr_regs: MAC_MA1HR:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MAC_MA0LR_RgRd(val0);
	MAC_MA0HR_RgRd(val1);
	MAC_GPIOR_RgRd(val2);
	MAC_GMIIDR_RgRd(val3);
	MAC_GMIIAR_RgRd(val4);
	MAC_HFR2_RgRd(val5);

	DBGPR("dbgpr_regs: MAC_MA0LR:%#x\n"
	      "dbgpr_regs: MAC_MA0HR:%#x\n"
	      "dbgpr_regs: MAC_GPIOR:%#x\n"
	      "dbgpr_regs: MAC_GMIIDR:%#x\n"
	      "dbgpr_regs: MAC_GMIIAR:%#x\n"
	      "dbgpr_regs: MAC_HFR2:%#x\n", val0, val1, val2, val3, val4, val5);

	MAC_HFR1_RgRd(val0);
	MAC_HFR0_RgRd(val1);
	MAC_MDR_RgRd(val2);
	MAC_VR_RgRd(val3);
	MAC_HTR7_RgRd(val4);
	MAC_HTR6_RgRd(val5);

	DBGPR("dbgpr_regs: MAC_HFR1:%#x\n"
	      "dbgpr_regs: MAC_HFR0:%#x\n"
	      "dbgpr_regs: MAC_MDR:%#x\n"
	      "dbgpr_regs: MAC_VR:%#x\n"
	      "dbgpr_regs: MAC_HTR7:%#x\n"
	      "dbgpr_regs: MAC_HTR6:%#x\n", val0, val1, val2, val3, val4, val5);

	MAC_HTR5_RgRd(val0);
	MAC_HTR4_RgRd(val1);
	MAC_HTR3_RgRd(val2);
	MAC_HTR2_RgRd(val3);
	MAC_HTR1_RgRd(val4);
	MAC_HTR0_RgRd(val5);

	DBGPR("dbgpr_regs: MAC_HTR5:%#x\n"
	      "dbgpr_regs: MAC_HTR4:%#x\n"
	      "dbgpr_regs: MAC_HTR3:%#x\n"
	      "dbgpr_regs: MAC_HTR2:%#x\n"
	      "dbgpr_regs: MAC_HTR1:%#x\n"
	      "dbgpr_regs: MAC_HTR0:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_RIWTR7_RgRd(val0);
	DMA_RIWTR6_RgRd(val1);
	DMA_RIWTR5_RgRd(val2);
	DMA_RIWTR4_RgRd(val3);
	DMA_RIWTR3_RgRd(val4);
	DMA_RIWTR2_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_RIWTR7:%#x\n"
	      "dbgpr_regs: DMA_RIWTR6:%#x\n"
	      "dbgpr_regs: DMA_RIWTR5:%#x\n"
	      "dbgpr_regs: DMA_RIWTR4:%#x\n"
	      "dbgpr_regs: DMA_RIWTR3:%#x\n"
	      "dbgpr_regs: DMA_RIWTR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RIWTR1_RgRd(val0);
	DMA_RIWTR0_RgRd(val1);
	DMA_RDRLR7_RgRd(val2);
	DMA_RDRLR6_RgRd(val3);
	DMA_RDRLR5_RgRd(val4);
	DMA_RDRLR4_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_RIWTR1:%#x\n"
	      "dbgpr_regs: DMA_RIWTR0:%#x\n"
	      "dbgpr_regs: DMA_RDRLR7:%#x\n"
	      "dbgpr_regs: DMA_RDRLR6:%#x\n"
	      "dbgpr_regs: DMA_RDRLR5:%#x\n"
	      "dbgpr_regs: DMA_RDRLR4:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RDRLR3_RgRd(val0);
	DMA_RDRLR2_RgRd(val1);
	DMA_RDRLR1_RgRd(val2);
	DMA_RDRLR0_RgRd(val3);
	DMA_TDRLR7_RgRd(val4);
	DMA_TDRLR6_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_RDRLR3:%#x\n"
	      "dbgpr_regs: DMA_RDRLR2:%#x\n"
	      "dbgpr_regs: DMA_RDRLR1:%#x\n"
	      "dbgpr_regs: DMA_RDRLR0:%#x\n"
	      "dbgpr_regs: DMA_TDRLR7:%#x\n"
	      "dbgpr_regs: DMA_TDRLR6:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_TDRLR5_RgRd(val0);
	DMA_TDRLR4_RgRd(val1);
	DMA_TDRLR3_RgRd(val2);
	DMA_TDRLR2_RgRd(val3);
	DMA_TDRLR1_RgRd(val4);
	DMA_TDRLR0_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_TDRLR5:%#x\n"
	      "dbgpr_regs: DMA_TDRLR4:%#x\n"
	      "dbgpr_regs: DMA_TDRLR3:%#x\n"
	      "dbgpr_regs: DMA_TDRLR2:%#x\n"
	      "dbgpr_regs: DMA_TDRLR1:%#x\n"
	      "dbgpr_regs: DMA_TDRLR0:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RDTP_RPDR7_RgRd(val0);
	DMA_RDTP_RPDR6_RgRd(val1);
	DMA_RDTP_RPDR5_RgRd(val2);
	DMA_RDTP_RPDR4_RgRd(val3);
	DMA_RDTP_RPDR3_RgRd(val4);
	DMA_RDTP_RPDR2_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_RDTP_RPDR7:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR6:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR5:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR4:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR3:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RDTP_RPDR1_RgRd(val0);
	DMA_RDTP_RPDR0_RgRd(val1);
	DMA_TDTP_TPDR7_RgRd(val2);
	DMA_TDTP_TPDR6_RgRd(val3);
	DMA_TDTP_TPDR5_RgRd(val4);
	DMA_TDTP_TPDR4_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_RDTP_RPDR1:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR0:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR7:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR6:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR5:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR4:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_TDTP_TPDR3_RgRd(val0);
	DMA_TDTP_TPDR2_RgRd(val1);
	DMA_TDTP_TPDR1_RgRd(val2);
	DMA_TDTP_TPDR0_RgRd(val3);
	DMA_RDLAR7_RgRd(val4);
	DMA_RDLAR6_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_TDTP_TPDR3:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR2:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR1:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR0:%#x\n"
	      "dbgpr_regs: DMA_RDLAR7:%#x\n"
	      "dbgpr_regs: DMA_RDLAR6:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RDLAR5_RgRd(val0);
	DMA_RDLAR4_RgRd(val1);
	DMA_RDLAR3_RgRd(val2);
	DMA_RDLAR2_RgRd(val3);
	DMA_RDLAR1_RgRd(val4);
	DMA_RDLAR0_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_RDLAR5:%#x\n"
	      "dbgpr_regs: DMA_RDLAR4:%#x\n"
	      "dbgpr_regs: DMA_RDLAR3:%#x\n"
	      "dbgpr_regs: DMA_RDLAR2:%#x\n"
	      "dbgpr_regs: DMA_RDLAR1:%#x\n"
	      "dbgpr_regs: DMA_RDLAR0:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_TDLAR7_RgRd(val0);
	DMA_TDLAR6_RgRd(val1);
	DMA_TDLAR5_RgRd(val2);
	DMA_TDLAR4_RgRd(val3);
	DMA_TDLAR3_RgRd(val4);
	DMA_TDLAR2_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_TDLAR7:%#x\n"
	      "dbgpr_regs: DMA_TDLAR6:%#x\n"
	      "dbgpr_regs: DMA_TDLAR5:%#x\n"
	      "dbgpr_regs: DMA_TDLAR4:%#x\n"
	      "dbgpr_regs: DMA_TDLAR3:%#x\n"
	      "dbgpr_regs: DMA_TDLAR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_TDLAR1_RgRd(val0);
	DMA_TDLAR0_RgRd(val1);
	DMA_IER7_RgRd(val2);
	DMA_IER6_RgRd(val3);
	DMA_IER5_RgRd(val4);
	DMA_IER4_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_TDLAR1:%#x\n"
	      "dbgpr_regs: DMA_TDLAR0:%#x\n"
	      "dbgpr_regs: DMA_IER7:%#x\n"
	      "dbgpr_regs: DMA_IER6:%#x\n"
	      "dbgpr_regs: DMA_IER5:%#x\n"
	      "dbgpr_regs: DMA_IER4:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_IER3_RgRd(val0);
	DMA_IER2_RgRd(val1);
	DMA_IER1_RgRd(val2);
	DMA_IER0_RgRd(val3);
	MAC_IMR_RgRd(val4);
	MAC_ISR_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_IER3:%#x\n"
	      "dbgpr_regs: DMA_IER2:%#x\n"
	      "dbgpr_regs: DMA_IER1:%#x\n"
	      "dbgpr_regs: DMA_IER0:%#x\n"
	      "dbgpr_regs: MAC_IMR:%#x\n"
	      "dbgpr_regs: MAC_ISR:%#x\n", val0, val1, val2, val3, val4, val5);

	MTL_ISR_RgRd(val0);
	DMA_SR7_RgRd(val1);
	DMA_SR6_RgRd(val2);
	DMA_SR5_RgRd(val3);
	DMA_SR4_RgRd(val4);
	DMA_SR3_RgRd(val5);

	DBGPR("dbgpr_regs: MTL_ISR:%#x\n"
	      "dbgpr_regs: DMA_SR7:%#x\n"
	      "dbgpr_regs: DMA_SR6:%#x\n"
	      "dbgpr_regs: DMA_SR5:%#x\n"
	      "dbgpr_regs: DMA_SR4:%#x\n"
	      "dbgpr_regs: DMA_SR3:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_SR2_RgRd(val0);
	DMA_SR1_RgRd(val1);
	DMA_SR0_RgRd(val2);
	DMA_ISR_RgRd(val3);
	DMA_DSR2_RgRd(val4);
	DMA_DSR1_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_SR2:%#x\n"
	      "dbgpr_regs: DMA_SR1:%#x\n"
	      "dbgpr_regs: DMA_SR0:%#x\n"
	      "dbgpr_regs: DMA_ISR:%#x\n"
	      "dbgpr_regs: DMA_DSR2:%#x\n"
	      "dbgpr_regs: DMA_DSR1:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_DSR0_RgRd(val0);
	MTL_Q0RDR_RgRd(val1);
	MTL_Q0ESR_RgRd(val2);
	MTL_Q0TDR_RgRd(val3);
	DMA_CHRBAR7_RgRd(val4);
	DMA_CHRBAR6_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_DSR0:%#x\n"
	      "dbgpr_regs: MTL_Q0RDR:%#x\n"
	      "dbgpr_regs: MTL_Q0ESR:%#x\n"
	      "dbgpr_regs: MTL_Q0TDR:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR7:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR6:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHRBAR5_RgRd(val0);
	DMA_CHRBAR4_RgRd(val1);
	DMA_CHRBAR3_RgRd(val2);
	DMA_CHRBAR2_RgRd(val3);
	DMA_CHRBAR1_RgRd(val4);
	DMA_CHRBAR0_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_CHRBAR5:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR4:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR3:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR2:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR1:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR0:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHTBAR7_RgRd(val0);
	DMA_CHTBAR6_RgRd(val1);
	DMA_CHTBAR5_RgRd(val2);
	DMA_CHTBAR4_RgRd(val3);
	DMA_CHTBAR3_RgRd(val4);
	DMA_CHTBAR2_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_CHTBAR7:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR6:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR5:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR4:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR3:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHTBAR1_RgRd(val0);
	DMA_CHTBAR0_RgRd(val1);
	DMA_CHRDR7_RgRd(val2);
	DMA_CHRDR6_RgRd(val3);
	DMA_CHRDR5_RgRd(val4);
	DMA_CHRDR4_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_CHTBAR1:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR0:%#x\n"
	      "dbgpr_regs: DMA_CHRDR7:%#x\n"
	      "dbgpr_regs: DMA_CHRDR6:%#x\n"
	      "dbgpr_regs: DMA_CHRDR5:%#x\n"
	      "dbgpr_regs: DMA_CHRDR4:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHRDR3_RgRd(val0);
	DMA_CHRDR2_RgRd(val1);
	DMA_CHRDR1_RgRd(val2);
	DMA_CHRDR0_RgRd(val3);
	DMA_CHTDR7_RgRd(val4);
	DMA_CHTDR6_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_CHRDR3:%#x\n"
	      "dbgpr_regs: DMA_CHRDR2:%#x\n"
	      "dbgpr_regs: DMA_CHRDR1:%#x\n"
	      "dbgpr_regs: DMA_CHRDR0:%#x\n"
	      "dbgpr_regs: DMA_CHTDR7:%#x\n"
	      "dbgpr_regs: DMA_CHTDR6:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHTDR5_RgRd(val0);
	DMA_CHTDR4_RgRd(val1);
	DMA_CHTDR3_RgRd(val2);
	DMA_CHTDR2_RgRd(val3);
	DMA_CHTDR1_RgRd(val4);
	DMA_CHTDR0_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_CHTDR5:%#x\n"
	      "dbgpr_regs: DMA_CHTDR4:%#x\n"
	      "dbgpr_regs: DMA_CHTDR3:%#x\n"
	      "dbgpr_regs: DMA_CHTDR2:%#x\n"
	      "dbgpr_regs: DMA_CHTDR1:%#x\n"
	      "dbgpr_regs: DMA_CHTDR0:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_SFCSR7_RgRd(val0);
	DMA_SFCSR6_RgRd(val1);
	DMA_SFCSR5_RgRd(val2);
	DMA_SFCSR4_RgRd(val3);
	DMA_SFCSR3_RgRd(val4);
	DMA_SFCSR2_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_SFCSR7:%#x\n"
	      "dbgpr_regs: DMA_SFCSR6:%#x\n"
	      "dbgpr_regs: DMA_SFCSR5:%#x\n"
	      "dbgpr_regs: DMA_SFCSR4:%#x\n"
	      "dbgpr_regs: DMA_SFCSR3:%#x\n"
	      "dbgpr_regs: DMA_SFCSR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_SFCSR1_RgRd(val0);
	DMA_SFCSR0_RgRd(val1);
	MAC_IVLANTIRR_RgRd(val2);
	MAC_VLANTIRR_RgRd(val3);
	MAC_VLANHTR_RgRd(val4);
	MAC_VLANTR_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_SFCSR1:%#x\n"
	      "dbgpr_regs: DMA_SFCSR0:%#x\n"
	      "dbgpr_regs: MAC_IVLANTIRR:%#x\n"
	      "dbgpr_regs: MAC_VLANTIRR:%#x\n"
	      "dbgpr_regs: MAC_VLANHTR:%#x\n"
	      "dbgpr_regs: MAC_VLANTR:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_SBUS_RgRd(val0);
	DMA_BMR_RgRd(val1);
	MTL_Q0RCR_RgRd(val2);
	MTL_Q0OCR_RgRd(val3);
	MTL_Q0ROMR_RgRd(val4);
	MTL_Q0QR_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_SBUS:%#x\n"
	      "dbgpr_regs: DMA_BMR:%#x\n"
	      "dbgpr_regs: MTL_Q0RCR:%#x\n"
	      "dbgpr_regs: MTL_Q0OCR:%#x\n"
	      "dbgpr_regs: MTL_Q0ROMR:%#x\n"
	      "dbgpr_regs: MTL_Q0QR:%#x\n", val0, val1, val2, val3, val4, val5);

	MTL_Q0ECR_RgRd(val0);
	MTL_Q0UCR_RgRd(val1);
	MTL_Q0TOMR_RgRd(val2);
	MTL_RQDCM1R_RgRd(val3);
	MTL_RQDCM0R_RgRd(val4);
	MTL_FDDR_RgRd(val5);

	DBGPR("dbgpr_regs: MTL_Q0ECR:%#x\n"
	      "dbgpr_regs: MTL_Q0UCR:%#x\n"
	      "dbgpr_regs: MTL_Q0TOMR:%#x\n"
	      "dbgpr_regs: MTL_RQDCM1R:%#x\n"
	      "dbgpr_regs: MTL_RQDCM0R:%#x\n"
	      "dbgpr_regs: MTL_FDDR:%#x\n", val0, val1, val2, val3, val4, val5);

	MTL_FDACS_RgRd(val0);
	MTL_OMR_RgRd(val1);
	MAC_RQC1R_RgRd(val2);
	MAC_RQC0R_RgRd(val3);
	MAC_TQPM1R_RgRd(val4);
	MAC_TQPM0R_RgRd(val5);

	DBGPR("dbgpr_regs: MTL_FDACS:%#x\n"
	      "dbgpr_regs: MTL_OMR:%#x\n"
	      "dbgpr_regs: MAC_RQC1R:%#x\n"
	      "dbgpr_regs: MAC_RQC0R:%#x\n"
	      "dbgpr_regs: MAC_TQPM1R:%#x\n"
	      "dbgpr_regs: MAC_TQPM0R:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MAC_RFCR_RgRd(val0);
	MAC_QTFCR7_RgRd(val1);
	MAC_QTFCR6_RgRd(val2);
	MAC_QTFCR5_RgRd(val3);
	MAC_QTFCR4_RgRd(val4);
	MAC_QTFCR3_RgRd(val5);

	DBGPR("dbgpr_regs: MAC_RFCR:%#x\n"
	      "dbgpr_regs: MAC_QTFCR7:%#x\n"
	      "dbgpr_regs: MAC_QTFCR6:%#x\n"
	      "dbgpr_regs: MAC_QTFCR5:%#x\n"
	      "dbgpr_regs: MAC_QTFCR4:%#x\n"
	      "dbgpr_regs: MAC_QTFCR3:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MAC_QTFCR2_RgRd(val0);
	MAC_QTFCR1_RgRd(val1);
	MAC_Q0TFCR_RgRd(val2);
	DMA_AXI4CR7_RgRd(val3);
	DMA_AXI4CR6_RgRd(val4);
	DMA_AXI4CR5_RgRd(val5);

	DBGPR("dbgpr_regs: MAC_QTFCR2:%#x\n"
	      "dbgpr_regs: MAC_QTFCR1:%#x\n"
	      "dbgpr_regs: MAC_Q0TFCR:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR7:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR6:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR5:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_AXI4CR4_RgRd(val0);
	DMA_AXI4CR3_RgRd(val1);
	DMA_AXI4CR2_RgRd(val2);
	DMA_AXI4CR1_RgRd(val3);
	DMA_AXI4CR0_RgRd(val4);
	DMA_RCR7_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_AXI4CR4:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR3:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR2:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR1:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR0:%#x\n"
	      "dbgpr_regs: DMA_RCR7:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_RCR6_RgRd(val0);
	DMA_RCR5_RgRd(val1);
	DMA_RCR4_RgRd(val2);
	DMA_RCR3_RgRd(val3);
	DMA_RCR2_RgRd(val4);
	DMA_RCR1_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_RCR6:%#x\n"
	      "dbgpr_regs: DMA_RCR5:%#x\n"
	      "dbgpr_regs: DMA_RCR4:%#x\n"
	      "dbgpr_regs: DMA_RCR3:%#x\n"
	      "dbgpr_regs: DMA_RCR2:%#x\n"
	      "dbgpr_regs: DMA_RCR1:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_RCR0_RgRd(val0);
	DMA_TCR7_RgRd(val1);
	DMA_TCR6_RgRd(val2);
	DMA_TCR5_RgRd(val3);
	DMA_TCR4_RgRd(val4);
	DMA_TCR3_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_RCR0:%#x\n"
	      "dbgpr_regs: DMA_TCR7:%#x\n"
	      "dbgpr_regs: DMA_TCR6:%#x\n"
	      "dbgpr_regs: DMA_TCR5:%#x\n"
	      "dbgpr_regs: DMA_TCR4:%#x\n"
	      "dbgpr_regs: DMA_TCR3:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_TCR2_RgRd(val0);
	DMA_TCR1_RgRd(val1);
	DMA_TCR0_RgRd(val2);
	DMA_CR7_RgRd(val3);
	DMA_CR6_RgRd(val4);
	DMA_CR5_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_TCR2:%#x\n"
	      "dbgpr_regs: DMA_TCR1:%#x\n"
	      "dbgpr_regs: DMA_TCR0:%#x\n"
	      "dbgpr_regs: DMA_CR7:%#x\n"
	      "dbgpr_regs: DMA_CR6:%#x\n"
	      "dbgpr_regs: DMA_CR5:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_CR4_RgRd(val0);
	DMA_CR3_RgRd(val1);
	DMA_CR2_RgRd(val2);
	DMA_CR1_RgRd(val3);
	DMA_CR0_RgRd(val4);
	MAC_WTR_RgRd(val5);

	DBGPR("dbgpr_regs: DMA_CR4:%#x\n"
	      "dbgpr_regs: DMA_CR3:%#x\n"
	      "dbgpr_regs: DMA_CR2:%#x\n"
	      "dbgpr_regs: DMA_CR1:%#x\n"
	      "dbgpr_regs: DMA_CR0:%#x\n"
	      "dbgpr_regs: MAC_WTR:%#x\n", val0, val1, val2, val3, val4, val5);

	MAC_MPFR_RgRd(val0);
	MAC_MECR_RgRd(val1);
	MAC_MCR_RgRd(val2);

	DBGPR("dbgpr_regs: MAC_MPFR:%#x\n"
	      "dbgpr_regs: MAC_MECR:%#x\n"
	      "dbgpr_regs: MAC_MCR:%#x\n", val0, val1, val2);

	return;
}
#endif

/*!
 * \details This function is invoked by DWC_ETH_QOS_start_xmit and
 * DWC_ETH_QOS_tx_interrupt function for dumping the TX descriptor contents
 * which are prepared for packet transmission and which are transmitted by
 * device. It is mainly used during development phase for debug purpose. Use
 * of these function may affect the performance during normal operation.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] first_desc_idx – first descriptor index for the current
 *		transfer.
 * \param[in] last_desc_idx – last descriptor index for the current transfer.
 * \param[in] flag – to indicate from which function it is called.
 *
 * \return void
 */

void dump_tx_desc(struct DWC_ETH_QOS_prv_data *pdata, int first_desc_idx,
		  int last_desc_idx, int flag, UINT qInx)
{
	int i;
	struct s_TX_NORMAL_DESC *desc = NULL;
	UINT varCTXT;

	if (first_desc_idx == last_desc_idx) {
		desc = GET_TX_DESC_PTR(qInx, first_desc_idx);

		TX_NORMAL_DESC_TDES3_CTXT_Mlf_Rd(desc->TDES3, varCTXT);

		printk(KERN_ALERT "\n%s[%02d %4p %03d %s] = %#x:%#x:%#x:%#x",
		       (varCTXT == 1) ? "TX_CONTXT_DESC" : "TX_NORMAL_DESC",
		       qInx, desc, first_desc_idx,
		       ((flag == 1) ? "QUEUED FOR TRANSMISSION" :
			((flag == 0) ? "FREED/FETCHED BY DEVICE" : "DEBUG DESC DUMP")),
			desc->TDES0, desc->TDES1,
			desc->TDES2, desc->TDES3);
	} else {
		int lp_cnt;
		if (first_desc_idx > last_desc_idx)
			lp_cnt = last_desc_idx + TX_DESC_CNT - first_desc_idx;
		else
			lp_cnt = last_desc_idx - first_desc_idx;

		for (i = first_desc_idx; lp_cnt >= 0; lp_cnt--) {
			desc = GET_TX_DESC_PTR(qInx, i);

			TX_NORMAL_DESC_TDES3_CTXT_Mlf_Rd(desc->TDES3, varCTXT);

			printk(KERN_ALERT "\n%s[%02d %4p %03d %s] = %#x:%#x:%#x:%#x",
			       (varCTXT == 1) ? "TX_CONTXT_DESC" : "TX_NORMAL_DESC",
			       qInx, desc, i,
			       ((flag == 1) ? "QUEUED FOR TRANSMISSION" :
				"FREED/FETCHED BY DEVICE"), desc->TDES0,
			       desc->TDES1, desc->TDES2, desc->TDES3);
			INCR_TX_DESC_INDEX(i, 1);
		}
	}
}

/*!
 * \details This function is invoked by poll function for dumping the
 * RX descriptor contents. It is mainly used during development phase for
 * debug purpose. Use of these function may affect the performance during
 * normal operation
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

void dump_rx_desc(UINT qInx, struct s_RX_NORMAL_DESC *desc, int desc_idx)
{
	printk(KERN_ALERT "\nRX_NORMAL_DESC[%02d %4p %03d RECEIVED FROM DEVICE]"\
		" = %#x:%#x:%#x:%#x",
		qInx, desc, desc_idx, desc->RDES0, desc->RDES1,
		desc->RDES2, desc->RDES3);
}

/*!
 * \details This function is invoked by start_xmit and poll function for
 * dumping the content of packet to be transmitted by device or received
 * from device. It is mainly used during development phase for debug purpose.
 * Use of these functions may affect the performance during normal operation.
 *
 * \param[in] skb – pointer to socket buffer structure.
 * \param[in] len – length of packet to be transmitted/received.
 * \param[in] tx_rx – packet to be transmitted or received.
 * \param[in] desc_idx – descriptor index to be used for transmission or
 *			reception of packet.
 *
 * \return void
 */

void print_pkt(struct sk_buff *skb, int len, bool tx_rx, int desc_idx)
{
	int i, j = 0;
	unsigned char *buf = skb->data;

	printk(KERN_ALERT
	       "\n\n/***********************************************************/\n");

	printk(KERN_ALERT "%s pkt of %d Bytes [DESC index = %d]\n\n",
	       (tx_rx ? "TX" : "RX"), len, desc_idx);
	printk(KERN_ALERT "Dst MAC addr(6 bytes)\n");
	for (i = 0; i < 6; i++)
		printk("%#.2x%s", buf[i], (((i == 5) ? "" : ":")));
	printk(KERN_ALERT "\nSrc MAC addr(6 bytes)\n");
	for (i = 6; i <= 11; i++)
		printk("%#.2x%s", buf[i], (((i == 11) ? "" : ":")));
	i = (buf[12] << 8 | buf[13]);
	printk(KERN_ALERT "\nType/Length(2 bytes)\n%#x", i);

	printk(KERN_ALERT "\nPay Load : %d bytes\n", (len - 14));
	for (i = 14, j = 1; i < len; i++, j++) {
		printk("%#.2x%s", buf[i], (((i == (len - 1)) ? "" : ":")));
		if ((j % 16) == 0)
			printk(KERN_ALERT "");
	}

	printk(KERN_ALERT
	       "/*************************************************************/\n\n");
}


/*!
 * \details This function is invoked by probe function. This function will
 * initialize default receive coalesce parameters and sw timer value and store
 * it in respective receive data structure.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

void DWC_ETH_QOS_init_rx_coalesce(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data = NULL;
	UINT i;

	DBGPR("-->DWC_ETH_QOS_init_rx_coalesce\n");

	for (i = 0; i < DWC_ETH_QOS_RX_QUEUE_CNT; i++) {
		rx_desc_data = GET_RX_WRAPPER_DESC(i);

		rx_desc_data->use_riwt = 1;
		rx_desc_data->rx_coal_frames = DWC_ETH_QOS_RX_MAX_FRAMES;
		rx_desc_data->rx_riwt =
			DWC_ETH_QOS_usec2riwt(DWC_ETH_QOS_OPTIMAL_DMA_RIWT_USEC, pdata);
	}

	DBGPR("<--DWC_ETH_QOS_init_rx_coalesce\n");
}


/*!
 * \details This function is invoked by open() function. This function will
 * clear MMC structure.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void DWC_ETH_QOS_mmc_setup(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR("-->DWC_ETH_QOS_mmc_setup\n");

	if (pdata->hw_feat.mmc_sel) {
		memset(&pdata->mmc, 0, sizeof(struct DWC_ETH_QOS_mmc_counters));
	} else
		printk(KERN_ALERT "No MMC/RMON module available in the HW\n");

	DBGPR("<--DWC_ETH_QOS_mmc_setup\n");
}

inline unsigned int DWC_ETH_QOS_reg_read(volatile ULONG *ptr)
{
		return ioread32((void *)ptr);
}


/*!
 * \details This function is invoked by ethtool function when user wants to
 * read MMC counters. This function will read the MMC if supported by core
 * and store it in DWC_ETH_QOS_mmc_counters structure. By default all the
 * MMC are programmed "read on reset" hence all the fields of the
 * DWC_ETH_QOS_mmc_counters are incremented.
 *
 * open() function. This function will
 * initialize MMC control register ie it disable all MMC interrupt and all
 * MMC register are configured to clear on read.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

void DWC_ETH_QOS_mmc_read(struct DWC_ETH_QOS_mmc_counters *mmc)
{
	DBGPR("-->DWC_ETH_QOS_mmc_read\n");

	/* MMC TX counter registers */
	mmc->mmc_tx_octetcount_gb += DWC_ETH_QOS_reg_read(MMC_TXOCTETCOUNT_GB_RgOffAddr);
	mmc->mmc_tx_framecount_gb += DWC_ETH_QOS_reg_read(MMC_TXPACKETCOUNT_GB_RgOffAddr);
	mmc->mmc_tx_broadcastframe_g += DWC_ETH_QOS_reg_read(MMC_TXBROADCASTPACKETS_G_RgOffAddr);
	mmc->mmc_tx_multicastframe_g += DWC_ETH_QOS_reg_read(MMC_TXMULTICASTPACKETS_G_RgOffAddr);
	mmc->mmc_tx_64_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX64OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_65_to_127_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX65TO127OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_128_to_255_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX128TO255OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_256_to_511_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX256TO511OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_512_to_1023_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX512TO1023OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_1024_to_max_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX1024TOMAXOCTETS_GB_RgOffAddr);
	mmc->mmc_tx_unicast_gb += DWC_ETH_QOS_reg_read(MMC_TXUNICASTPACKETS_GB_RgOffAddr);
	mmc->mmc_tx_multicast_gb += DWC_ETH_QOS_reg_read(MMC_TXMULTICASTPACKETS_GB_RgOffAddr);
	mmc->mmc_tx_broadcast_gb += DWC_ETH_QOS_reg_read(MMC_TXBROADCASTPACKETS_GB_RgOffAddr);
	mmc->mmc_tx_underflow_error += DWC_ETH_QOS_reg_read(MMC_TXUNDERFLOWERROR_RgOffAddr);
	mmc->mmc_tx_singlecol_g += DWC_ETH_QOS_reg_read(MMC_TXSINGLECOL_G_RgOffAddr);
	mmc->mmc_tx_multicol_g += DWC_ETH_QOS_reg_read(MMC_TXMULTICOL_G_RgOffAddr);
	mmc->mmc_tx_deferred += DWC_ETH_QOS_reg_read(MMC_TXDEFERRED_RgOffAddr);
	mmc->mmc_tx_latecol += DWC_ETH_QOS_reg_read(MMC_TXLATECOL_RgOffAddr);
	mmc->mmc_tx_exesscol += DWC_ETH_QOS_reg_read(MMC_TXEXESSCOL_RgOffAddr);
	mmc->mmc_tx_carrier_error += DWC_ETH_QOS_reg_read(MMC_TXCARRIERERROR_RgOffAddr);
	mmc->mmc_tx_octetcount_g += DWC_ETH_QOS_reg_read(MMC_TXOCTETCOUNT_G_RgOffAddr);
	mmc->mmc_tx_framecount_g += DWC_ETH_QOS_reg_read(MMC_TXPACKETSCOUNT_G_RgOffAddr);
	mmc->mmc_tx_excessdef += DWC_ETH_QOS_reg_read(MMC_TXEXCESSDEF_RgOffAddr);
	mmc->mmc_tx_pause_frame += DWC_ETH_QOS_reg_read(MMC_TXPAUSEPACKETS_RgOffAddr);
	mmc->mmc_tx_vlan_frame_g += DWC_ETH_QOS_reg_read(MMC_TXVLANPACKETS_G_RgOffAddr);
	mmc->mmc_tx_osize_frame_g += DWC_ETH_QOS_reg_read(MMC_TXOVERSIZE_G_RgOffAddr);

	/* MMC RX counter registers */
	mmc->mmc_rx_framecount_gb += DWC_ETH_QOS_reg_read(MMC_RXPACKETCOUNT_GB_RgOffAddr);
	mmc->mmc_rx_octetcount_gb += DWC_ETH_QOS_reg_read(MMC_RXOCTETCOUNT_GB_RgOffAddr);
	mmc->mmc_rx_octetcount_g += DWC_ETH_QOS_reg_read(MMC_RXOCTETCOUNT_G_RgOffAddr);
	mmc->mmc_rx_broadcastframe_g += DWC_ETH_QOS_reg_read(MMC_RXBROADCASTPACKETS_G_RgOffAddr);
	mmc->mmc_rx_multicastframe_g += DWC_ETH_QOS_reg_read(MMC_RXMULTICASTPACKETS_G_RgOffAddr);
	mmc->mmc_rx_crc_errror += DWC_ETH_QOS_reg_read(MMC_RXCRCERROR_RgOffAddr);
	mmc->mmc_rx_align_error += DWC_ETH_QOS_reg_read(MMC_RXALIGNMENTERROR_RgOffAddr);
	mmc->mmc_rx_run_error += DWC_ETH_QOS_reg_read(MMC_RXRUNTERROR_RgOffAddr);
	mmc->mmc_rx_jabber_error += DWC_ETH_QOS_reg_read(MMC_RXJABBERERROR_RgOffAddr);
	mmc->mmc_rx_undersize_g += DWC_ETH_QOS_reg_read(MMC_RXUNDERSIZE_G_RgOffAddr);
	mmc->mmc_rx_oversize_g += DWC_ETH_QOS_reg_read(MMC_RXOVERSIZE_G_RgOffAddr);
	mmc->mmc_rx_64_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX64OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_65_to_127_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX65TO127OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_128_to_255_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX128TO255OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_256_to_511_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX256TO511OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_512_to_1023_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX512TO1023OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_1024_to_max_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX1024TOMAXOCTETS_GB_RgOffAddr);
	mmc->mmc_rx_unicast_g += DWC_ETH_QOS_reg_read(MMC_RXUNICASTPACKETS_G_RgOffAddr);
	mmc->mmc_rx_length_error += DWC_ETH_QOS_reg_read(MMC_RXLENGTHERROR_RgOffAddr);
	mmc->mmc_rx_outofrangetype += DWC_ETH_QOS_reg_read(MMC_RXOUTOFRANGETYPE_RgOffAddr);
	mmc->mmc_rx_pause_frames += DWC_ETH_QOS_reg_read(MMC_RXPAUSEPACKETS_RgOffAddr);
	mmc->mmc_rx_fifo_overflow += DWC_ETH_QOS_reg_read(MMC_RXFIFOOVERFLOW_RgOffAddr);
	mmc->mmc_rx_vlan_frames_gb += DWC_ETH_QOS_reg_read(MMC_RXVLANPACKETS_GB_RgOffAddr);
	mmc->mmc_rx_watchdog_error += DWC_ETH_QOS_reg_read(MMC_RXWATCHDOGERROR_RgOffAddr);
	mmc->mmc_rx_receive_error += DWC_ETH_QOS_reg_read(MMC_RXRCVERROR_RgOffAddr);
	mmc->mmc_rx_ctrl_frames_g += DWC_ETH_QOS_reg_read(MMC_RXCTRLPACKETS_G_RgOffAddr);

	/* IPC */
	mmc->mmc_rx_ipc_intr_mask += DWC_ETH_QOS_reg_read(MMC_IPC_INTR_MASK_RX_RgOffAddr);
	mmc->mmc_rx_ipc_intr += DWC_ETH_QOS_reg_read(MMC_IPC_INTR_RX_RgOffAddr);

	/* IPv4 */
	mmc->mmc_rx_ipv4_gd += DWC_ETH_QOS_reg_read(MMC_RXIPV4_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv4_hderr += DWC_ETH_QOS_reg_read(MMC_RXIPV4_HDRERR_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv4_nopay += DWC_ETH_QOS_reg_read(MMC_RXIPV4_NOPAY_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv4_frag += DWC_ETH_QOS_reg_read(MMC_RXIPV4_FRAG_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv4_udsbl += DWC_ETH_QOS_reg_read(MMC_RXIPV4_UBSBL_PKTS_RgOffAddr);

	/* IPV6 */
	mmc->mmc_rx_ipv6_gd += DWC_ETH_QOS_reg_read(MMC_RXIPV6_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv6_hderr += DWC_ETH_QOS_reg_read(MMC_RXIPV6_HDRERR_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv6_nopay += DWC_ETH_QOS_reg_read(MMC_RXIPV6_NOPAY_PKTS_RgOffAddr);

	/* Protocols */
	mmc->mmc_rx_udp_gd += DWC_ETH_QOS_reg_read(MMC_RXUDP_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_udp_err += DWC_ETH_QOS_reg_read(MMC_RXUDP_ERR_PKTS_RgOffAddr);
	mmc->mmc_rx_tcp_gd += DWC_ETH_QOS_reg_read(MMC_RXTCP_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_tcp_err += DWC_ETH_QOS_reg_read(MMC_RXTCP_ERR_PKTS_RgOffAddr);
	mmc->mmc_rx_icmp_gd += DWC_ETH_QOS_reg_read(MMC_RXICMP_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_icmp_err += DWC_ETH_QOS_reg_read(MMC_RXICMP_ERR_PKTS_RgOffAddr);

	/* IPv4 */
	mmc->mmc_rx_ipv4_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv4_hderr_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_HDRERR_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv4_nopay_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_NOPAY_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv4_frag_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_FRAG_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv4_udsbl_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_UDSBL_OCTETS_RgOffAddr);

	/* IPV6 */
	mmc->mmc_rx_ipv6_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV6_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv6_hderr_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV6_HDRERR_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv6_nopay_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV6_NOPAY_OCTETS_RgOffAddr);

	/* Protocols */
	mmc->mmc_rx_udp_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXUDP_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_udp_err_octets += DWC_ETH_QOS_reg_read(MMC_RXUDP_ERR_OCTETS_RgOffAddr);
	mmc->mmc_rx_tcp_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXTCP_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_tcp_err_octets += DWC_ETH_QOS_reg_read(MMC_RXTCP_ERR_OCTETS_RgOffAddr);
	mmc->mmc_rx_icmp_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXICMP_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_icmp_err_octets += DWC_ETH_QOS_reg_read(MMC_RXICMP_ERR_OCTETS_RgOffAddr);

	DBGPR("<--DWC_ETH_QOS_mmc_read\n");
}


phy_interface_t DWC_ETH_QOS_get_phy_interface(struct DWC_ETH_QOS_prv_data *pdata)
{
	phy_interface_t ret = PHY_INTERFACE_MODE_MII;

	DBGPR("-->DWC_ETH_QOS_get_phy_interface\n");

	if (pdata->hw_feat.act_phy_sel == DWC_ETH_QOS_GMII_MII) {
		if (pdata->hw_feat.gmii_sel)
			ret = PHY_INTERFACE_MODE_GMII;
		else if (pdata->hw_feat.mii_sel)
			ret = PHY_INTERFACE_MODE_MII;
	} else if (pdata->hw_feat.act_phy_sel == DWC_ETH_QOS_RGMII) {
		ret = PHY_INTERFACE_MODE_RGMII;
	} else if (pdata->hw_feat.act_phy_sel == DWC_ETH_QOS_SGMII) {
		ret = PHY_INTERFACE_MODE_SGMII;
	} else if (pdata->hw_feat.act_phy_sel == DWC_ETH_QOS_TBI) {
		ret = PHY_INTERFACE_MODE_TBI;
	} else if (pdata->hw_feat.act_phy_sel == DWC_ETH_QOS_RMII) {
		ret = PHY_INTERFACE_MODE_RMII;
	} else if (pdata->hw_feat.act_phy_sel == DWC_ETH_QOS_RTBI) {
		ret = PHY_INTERFACE_MODE_RTBI;
	} else if (pdata->hw_feat.act_phy_sel == DWC_ETH_QOS_SMII) {
		ret = PHY_INTERFACE_MODE_SMII;
	} else if (pdata->hw_feat.act_phy_sel == DWC_ETH_QOS_RevMII) {
		//what to return ?
	} else {
		printk(KERN_ALERT "Missing interface support between"\
		    "PHY and MAC\n\n");
		ret = PHY_INTERFACE_MODE_NA;
	}

	DBGPR("<--DWC_ETH_QOS_get_phy_interface\n");

	return ret;
}

static const struct net_device_ops DWC_ETH_QOS_netdev_ops = {
	.ndo_open = DWC_ETH_QOS_open,
	.ndo_stop = DWC_ETH_QOS_close,
	.ndo_start_xmit = DWC_ETH_QOS_start_xmit,
	.ndo_get_stats = DWC_ETH_QOS_get_stats,
	.ndo_set_rx_mode = DWC_ETH_QOS_set_rx_mode,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = DWC_ETH_QOS_poll_controller,
#endif				/*end of CONFIG_NET_POLL_CONTROLLER */
	.ndo_set_features = DWC_ETH_QOS_set_features,
	.ndo_fix_features = DWC_ETH_QOS_fix_features,
	.ndo_do_ioctl = DWC_ETH_QOS_ioctl,
	.ndo_change_mtu = DWC_ETH_QOS_change_mtu,
#ifdef DWC_ETH_QOS_QUEUE_SELECT_ALGO
	.ndo_select_queue = DWC_ETH_QOS_select_queue,
#endif
	.ndo_vlan_rx_add_vid = DWC_ETH_QOS_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = DWC_ETH_QOS_vlan_rx_kill_vid,
};

struct net_device_ops *DWC_ETH_QOS_get_netdev_ops(void)
{
	return (struct net_device_ops *)&DWC_ETH_QOS_netdev_ops;
}


