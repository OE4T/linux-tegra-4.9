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
 *
 * ========================================================================= */

/*!@file: dev.c
 * @brief: Driver functions.
 */
#include "yheader.h"
#include "yapphdr.h"

extern ULONG dwc_eth_qos_pci_base_addr;

#include "yregacc.h"

#ifdef DWC_ETH_QOS_CONFIG_PGTEST

static INT prepare_dev_pktgen(struct DWC_ETH_QOS_prv_data *pdata)
{
	UINT qInx = 0;

	/* set MAC loop back mode */
	MAC_MCR_LM_UdfWr(0x1);

	/* Do not strip received VLAN tag */
	MAC_VLANTR_EVLS_UdfWr(0x0);

	/* set promiscuous mode */
	MAC_MPFR_PR_UdfWr(0x1);

	/* disable autopad or CRC stripping */
	MAC_MCR_ACS_UdfWr(0);

	/* enable drop tx status */
	MTL_OMR_DTXSTS_UdfWr(0x1);

	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		/* enable avg bits per slot interrupt */
		MTL_QECR_ABPSSIE_UdfWr(qInx, 0x1);

		/* enable OSF mode */
		DMA_TCR_OSP_UdfWr(qInx, 0x1);

		/* disable slot checks */
		DMA_SFCSR_RgWr(qInx, 0);
	}

	return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure slot count The software
* can program the number of slots(of duration 125us) over which the
* average transmitted bits per slot need to be computed for
* channel 1 to 7 when CBS alogorithm is enabled.
* \param[in] qInx
* \param[in] slot_count
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_slot_count(UINT qInx,
                          UCHAR slot_count)
{

  if (slot_count == 1) {
    MTL_QECR_SLC_UdfWr(qInx, 0);
  }
  else if (slot_count == 2) {
    MTL_QECR_SLC_UdfWr(qInx, 0x1);
  }
  else if (slot_count == 4) {
    MTL_QECR_SLC_UdfWr(qInx, 0x3);
  }
  else if (slot_count == 8) {
    MTL_QECR_SLC_UdfWr(qInx, 0x4);
  }
  else if (slot_count == 16) {
    MTL_QECR_SLC_UdfWr(qInx, 0x5);
  }

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to enable/disable slot interrupt:
* When this bit is set,the MAC asserts an interrupt when the average
* bits per slot status is updated for channel 1 to 7.
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_slot_interrupt(UINT qInx, UCHAR slot_int)
{

  MTL_QECR_ABPSSIE_UdfWr(qInx, slot_int);

  return Y_SUCCESS;
}





/*!
* \brief This sequence is used to configure DMA Tx:Rx/Rx:Tx
* Priority Ratio These bits control the priority ratio in WRR
* arbitration between the TX and RX DAM.
* \param[in] prio_ratio
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_tx_rx_prio_ratio(UCHAR prio_ratio)
{

  DMA_BMR_PR_UdfWr(prio_ratio);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure DMA Transmit Arbitration algorithm
* \param[in] arb_algo
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_dma_tx_arb_algorithm(UCHAR arb_algo)
{

  DMA_BMR_TAA_UdfWr(arb_algo);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure DMA Tx Priority When this
* bit is set, it indicates that the TX DMA has higher priority than
* the RX DMA during arbitration for the system-side bus.
* \param[in] prio
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_tx_rx_prio(UCHAR prio)
{

  DMA_BMR_TXPR_UdfWr(prio);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure DMA Tx/Rx Arbitration Scheme
* This bit specifies the arbitration scheme between the Tx and Rx paths
* of all channels.
* \param[in] prio_policy
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_tx_rx_prio_policy(UCHAR prio_policy)
{

  DMA_BMR_DA_UdfWr(prio_policy);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure TX Channel Weight
* \param[in] qInx
* \param[in] weight
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_ch_arb_weights(UINT qInx,
                               UCHAR weight)
{

  if (weight == 1) {
    DMA_TCR_TCW_UdfWr(qInx, 0);
  }
  else if (weight == 2) {
    DMA_TCR_TCW_UdfWr(qInx, 0x1);
  }
  else if (weight == 3) {
    DMA_TCR_TCW_UdfWr(qInx, 0x2);
  }
  else if (weight == 4) {
    DMA_TCR_TCW_UdfWr(qInx, 0x3);
  }
  else if (weight == 5) {
    DMA_TCR_TCW_UdfWr(qInx, 0x4);
  }
  else if (weight == 6) {
    DMA_TCR_TCW_UdfWr(qInx, 0x5);
  }
  else if (weight == 7) {
    DMA_TCR_TCW_UdfWr(qInx, 0x6);
  }
  else if (weight == 8) {
    DMA_TCR_TCW_UdfWr(qInx, 0x7);
  }

  return Y_SUCCESS;
}
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */



/*!
* \brief This sequence is used to enable/disable MAC loopback mode
* \param[in] enb_dis
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_mac_loopback_mode(UINT enb_dis)
{

  MAC_MCR_LM_UdfWr(enb_dis);

  return Y_SUCCESS;
}


/* enable/disable PFC(Priority Based Flow Control) */
static void config_pfc(int enb_dis)
{
	MAC_RFCR_PFCE_UdfWr(enb_dis);
}

/*!
* \brief This sequence is used to configure mac double vlan processing feature.
* \param[in] enb_dis
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_tx_outer_vlan(UINT op_type, UINT outer_vlt)
{
	printk(KERN_ALERT "--> config_tx_outer_vlan()\n");

	MAC_VLANTIRR_VLTI_UdfWr(0x0);
	MAC_VLANTIRR_VLT_UdfWr(outer_vlt);
	MAC_VLANTIRR_VLP_UdfWr(0x1);
	MAC_VLANTIRR_VLC_UdfWr(op_type);

	if (op_type == DWC_ETH_QOS_DVLAN_NONE) {
		MAC_VLANTIRR_VLP_UdfWr(0x0);
		MAC_VLANTIRR_VLT_UdfWr(0x0);
	}

	printk(KERN_ALERT "<-- config_tx_outer_vlan()\n");

	return Y_SUCCESS;
}

static INT config_tx_inner_vlan(UINT op_type, UINT inner_vlt)
{
	printk(KERN_ALERT "--> config_tx_inner_vlan()\n");

	MAC_IVLANTIRR_VLTI_UdfWr(0x0);
	MAC_IVLANTIRR_VLT_UdfWr(inner_vlt);
	MAC_IVLANTIRR_VLP_UdfWr(0x1);
	MAC_IVLANTIRR_VLC_UdfWr(op_type);

	if (op_type == DWC_ETH_QOS_DVLAN_NONE) {
		MAC_IVLANTIRR_VLP_UdfWr(0x0);
		MAC_IVLANTIRR_VLT_UdfWr(0x0);
	}

	printk(KERN_ALERT "<-- config_tx_inner_vlan()\n");

	return Y_SUCCESS;
}

static INT config_svlan(UINT flags)
{
	INT ret = Y_SUCCESS;

	printk(KERN_ALERT "--> config_svlan()\n");

	MAC_VLANTR_ESVL_UdfWr(1);
	if (flags == DWC_ETH_QOS_DVLAN_NONE) {
		MAC_VLANTR_ESVL_UdfWr(0);
		MAC_IVLANTIRR_CSVL_UdfWr(0);
		MAC_VLANTIRR_CSVL_UdfWr(0);
	} else if (flags == DWC_ETH_QOS_DVLAN_INNER) {
		MAC_IVLANTIRR_CSVL_UdfWr(1);
	} else if (flags == DWC_ETH_QOS_DVLAN_OUTER) {
		MAC_VLANTIRR_CSVL_UdfWr(1);
	} else if (flags == DWC_ETH_QOS_DVLAN_BOTH) {
		MAC_IVLANTIRR_CSVL_UdfWr(1);
		MAC_VLANTIRR_CSVL_UdfWr(1);
	} else {
		printk(KERN_ALERT "ERROR : double VLAN enable SVLAN configuration - Invalid argument");
		ret = Y_FAILURE;
	}

	printk(KERN_ALERT "<-- config_svlan()\n");

	return ret;
}

static VOID config_dvlan(bool enb_dis)
{
	MAC_VLANTR_EDVLP_UdfWr(enb_dis);
}


/*!
* \brief This sequence is used to enable/disable ARP offload
* \param[in] enb_dis
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int config_arp_offload(int enb_dis)
{

  MAC_MCR_ARPEN_UdfWr(enb_dis);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to update the IP addr into MAC ARP Add reg,
* which is used by MAC for replying to ARP packets
* \param[in] addr
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int update_arp_offload_ip_addr(UCHAR addr[])
{

  MAC_ARPA_RgWr((addr[3] | (addr[2] << 8) | (addr[1] << 16) | addr[0] << 24));

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to get the status of LPI/EEE mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static u32 get_lpi_status(void)
{
  u32 varmac_lps;

  MAC_LPS_RgRd(varmac_lps);

  return varmac_lps;
}




/*!
* \brief This sequence is used to enable EEE mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int set_eee_mode(void)
{

  MAC_LPS_LPIEN_UdfWr(0x1);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to disable EEE mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int reset_eee_mode(void)
{

  MAC_LPS_LPITXA_UdfWr(0);
  MAC_LPS_LPIEN_UdfWr(0);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to set PLS bit
* \param[in] phy_link
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int set_eee_pls(int phy_link)
{

  if (phy_link == 1) {
    MAC_LPS_PLS_UdfWr(0x1);
  }
  else {
    MAC_LPS_PLS_UdfWr(0);
  }

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to set EEE timer values
* \param[in] lpi_lst
* \param[in] lpi_twt
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int set_eee_timer(int lpi_lst,
                         int lpi_twt)
{

  /* mim time(us) for which the MAC waits after it stops transmitting */
  /* the LPI pattern to the PHY and before it resumes the normal transmission. */
  MAC_LPC_TWT_UdfWr(lpi_twt);
  /* mim time(ms) for which the link status from the PHY should be Up before */
  /* the LPI pattern can be transmitted to the PHY. */
  MAC_LPC_TLPIEX_UdfWr(lpi_lst);

  return Y_SUCCESS;
}




static int set_lpi_tx_automate(void)
{
	MAC_LPS_LPITXA_UdfWr(0x1);

	return Y_SUCCESS;
}




/*!
* \brief This sequence is used to enable/disable Auto-Negotiation
* and restart the autonegotiation
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int control_an(bool enable, bool restart)
{

  MAC_ANC_ANE_UdfWr(enable);
  MAC_ANC_RAN_UdfWr(restart);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to get Auto-Negotiation advertisment
* pause parameter
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int get_an_adv_pause_param(void)
{
  unsigned long varmac_aad;

  MAC_AAD_RgRd(varmac_aad);

  return GET_VALUE(varmac_aad, MAC_AAD_PSE_LPOS, MAC_AAD_PSE_HPOS);
}




/*!
* \brief This sequence is used to get Auto-Negotiation advertisment
* duplex parameter. Returns one if Full duplex mode is selected
* else returns zero
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int get_an_adv_duplex_param(void)
{
  unsigned long varmac_aad;

  MAC_AAD_RgRd(varmac_aad);
  if (GET_VALUE(varmac_aad, MAC_AAD_FD_LPOS, MAC_AAD_FD_HPOS) == 1) {
    return 1;
  }
  else {
    return 0;
  }
}




/*!
* \brief This sequence is used to get Link partner Auto-Negotiation
* advertisment pause parameter
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int get_lp_an_adv_pause_param(void)
{
  unsigned long varmac_alpa;

  MAC_ALPA_RgRd(varmac_alpa);

  return GET_VALUE(varmac_alpa, MAC_ALPA_PSE_LPOS, MAC_ALPA_PSE_HPOS);
}




/*!
* \brief This sequence is used to get Link partner Auto-Negotiation
* advertisment duplex parameter. Returns one if Full duplex mode
* is selected else returns zero
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int get_lp_an_adv_duplex_param(void)
{
  unsigned long varmac_alpa;

  MAC_ALPA_RgRd(varmac_alpa);
  if (GET_VALUE(varmac_alpa, MAC_ALPA_FD_LPOS, MAC_ALPA_FD_HPOS) == 1) {
    return 1;
  }
  else {
    return 0;
  }
}


static UINT get_vlan_tag_comparison(void)
{
	UINT etv;

	MAC_VLANTR_ETV_UdfRd(etv);

	return etv;
}

/*!
* \brief This sequence is used to enable/disable VLAN filtering and
* also selects VLAN filtering mode- perfect/hash
* \param[in] filter_enb_dis
* \param[in] perfect_hash
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_vlan_filtering(INT filter_enb_dis,
                                 INT perfect_hash_filtering,
				 INT perfect_inverse_match)
{
  MAC_MPFR_VTFE_UdfWr(filter_enb_dis);
  MAC_VLANTR_VTIM_UdfWr(perfect_inverse_match);
  MAC_VLANTR_VTHM_UdfWr(perfect_hash_filtering);
  /* To enable only HASH filtering then VL/VID
   * should be > zero. Hence we are writting 1 into VL.
   * It also means that MAC will always receive VLAN pkt with
   * VID = 1 if inverse march is not set.
   * */
  if (perfect_hash_filtering)
    MAC_VLANTR_VL_UdfWr(0x1);

  /* By default enable MAC to calculate vlan hash on
   * only 12-bits of received VLAN tag (ie only on
   * VLAN id and ignore priority and other fields)
   * */
  if (perfect_hash_filtering)
	  MAC_VLANTR_ETV_UdfWr(0x1);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to update the VLAN ID for perfect filtering
* \param[in] vid
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_vlan_id(USHORT vid)
{

  MAC_VLANTR_VL_UdfWr(vid);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to update the VLAN Hash Table reg with new VLAN ID
* \param[in] data
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_vlan_hash_table_reg(USHORT data)
{

  MAC_VLANHTR_VLHT_UdfWr(data);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to get the content of VLAN Hash Table reg
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT get_vlan_hash_table_reg(void)
{
  ULONG varMAC_VLANHTR;

  MAC_VLANHTR_RgRd(varMAC_VLANHTR);

  return GET_VALUE(varMAC_VLANHTR, MAC_VLANHTR_VLHT_LPOS, MAC_VLANHTR_VLHT_HPOS);
}




/*!
* \brief This sequence is used to update Destination Port Number for
* L4(TCP/UDP) layer filtering
* \param[in] filter_no
* \param[in] port_no
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_l4_da_port_no(INT filter_no,
                                USHORT port_no)
{

  MAC_L4AR_L4DP0_UdfWr(filter_no, port_no);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to update Source Port Number for
* L4(TCP/UDP) layer filtering
* \param[in] filter_no
* \param[in] port_no
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_l4_sa_port_no(INT filter_no,
                                USHORT port_no)
{

  MAC_L4AR_L4SP0_UdfWr(filter_no, port_no);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure L4(TCP/UDP) filters for
* SA and DA Port Number matching
* \param[in] filter_no
* \param[in] tcp_udp_match
* \param[in] src_dst_port_match
* \param[in] perfect_inverse_match
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_l4_filters(INT filter_no,
		                         INT enb_dis,
                             INT tcp_udp_match,
                             INT src_dst_port_match,
                             INT perfect_inverse_match)
{

  MAC_L3L4CR_L4PEN0_UdfWr(filter_no, tcp_udp_match);

  if (src_dst_port_match == 0) {
	if (enb_dis == 1) {
		/* Enable L4 filters for SOURCE Port No matching */
		MAC_L3L4CR_L4SPM0_UdfWr(filter_no, 0x1);
		MAC_L3L4CR_L4SPIM0_UdfWr(filter_no, perfect_inverse_match);
    }
    else {
		/* Disable L4 filters for SOURCE Port No matching */
		MAC_L3L4CR_L4SPM0_UdfWr(filter_no, 0x0);
		MAC_L3L4CR_L4SPIM0_UdfWr(filter_no, 0x0);
	}
  }
  else {
	if (enb_dis == 1) {
		/* Enable L4 filters for DESTINATION port No matching */
		MAC_L3L4CR_L4DPM0_UdfWr(filter_no, 0x1);
		MAC_L3L4CR_L4DPIM0_UdfWr(filter_no, perfect_inverse_match);
	}
	else {
		/* Disable L4 filters for DESTINATION port No matching */
		MAC_L3L4CR_L4DPM0_UdfWr(filter_no, 0x0);
		MAC_L3L4CR_L4DPIM0_UdfWr(filter_no, 0x0);
	}
  }

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to update IPv6 source/destination Address for L3 layer filtering
* \param[in] filter_no
* \param[in] addr
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_ip6_addr(INT filter_no,
                           USHORT addr[])
{
  /* update Bits[31:0] of 128-bit IP addr */
  MAC_L3A0R_RgWr(filter_no, (addr[7] | (addr[6] << 16)));
  /* update Bits[63:32] of 128-bit IP addr */
  MAC_L3A1R_RgWr(filter_no, (addr[5] | (addr[4] << 16)));
  /* update Bits[95:64] of 128-bit IP addr */
  MAC_L3A2R_RgWr(filter_no, (addr[3] | (addr[2] << 16)));
  /* update Bits[127:96] of 128-bit IP addr */
  MAC_L3A3R_RgWr(filter_no, (addr[1] | (addr[0] << 16)));

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to update IPv4 destination Address for L3 layer filtering
* \param[in] filter_no
* \param[in] addr
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_ip4_addr1(INT filter_no,
                            UCHAR addr[])
{
  MAC_L3A1R_RgWr(filter_no, (addr[3] | (addr[2] << 8) | (addr[1] << 16) | (addr[0] << 24)));

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to update IPv4 source Address for L3 layer filtering
* \param[in] filter_no
* \param[in] addr
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_ip4_addr0(INT filter_no,
                            UCHAR addr[])
{
  MAC_L3A0R_RgWr(filter_no, (addr[3] | (addr[2] << 8) | (addr[1] << 16) | (addr[0] << 24)));

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure L3(IPv4/IPv6) filters
* for SA/DA Address matching
* \param[in] filter_no
* \param[in] ipv4_ipv6_match
* \param[in] src_dst_addr_match
* \param[in] perfect_inverse_match
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_l3_filters(INT filter_no,
		                         INT enb_dis,
                             INT ipv4_ipv6_match,
                             INT src_dst_addr_match,
                             INT perfect_inverse_match)
{
	MAC_L3L4CR_L3PEN0_UdfWr(filter_no, ipv4_ipv6_match);

	/* For IPv6 either SA/DA can be checked, not both */
	if (ipv4_ipv6_match == 1) {
		if (enb_dis == 1) {
			if (src_dst_addr_match == 0) {
				/* Enable L3 filters for IPv6 SOURCE addr matching */
				MAC_L3L4CR_L3SAM0_UdfWr(filter_no, 0x1);
				MAC_L3L4CR_L3SAIM0_UdfWr(filter_no, perfect_inverse_match);
				MAC_L3L4CR_L3DAM0_UdfWr(filter_no, 0x0);
				MAC_L3L4CR_L3DAIM0_UdfWr(filter_no, 0x0);
			}
			else {
				/* Enable L3 filters for IPv6 DESTINATION addr matching */
				MAC_L3L4CR_L3SAM0_UdfWr(filter_no, 0x0);
				MAC_L3L4CR_L3SAIM0_UdfWr(filter_no, 0x0);
				MAC_L3L4CR_L3DAM0_UdfWr(filter_no, 0x1);
				MAC_L3L4CR_L3DAIM0_UdfWr(filter_no, perfect_inverse_match);
			}
		}
		else {
			/* Disable L3 filters for IPv6 SOURCE/DESTINATION addr matching */
			MAC_L3L4CR_L3PEN0_UdfWr(filter_no, 0x0);
			MAC_L3L4CR_L3SAM0_UdfWr(filter_no, 0x0);
			MAC_L3L4CR_L3SAIM0_UdfWr(filter_no, 0x0);
			MAC_L3L4CR_L3DAM0_UdfWr(filter_no, 0x0);
			MAC_L3L4CR_L3DAIM0_UdfWr(filter_no, 0x0);
		}
	}
	else {
		if (src_dst_addr_match == 0) {
			if (enb_dis == 1) {
				/* Enable L3 filters for IPv4 SOURCE addr matching */
				MAC_L3L4CR_L3SAM0_UdfWr(filter_no, 0x1);
				MAC_L3L4CR_L3SAIM0_UdfWr(filter_no, perfect_inverse_match);
			}
			else {
				/* Disable L3 filters for IPv4 SOURCE addr matching */
				MAC_L3L4CR_L3SAM0_UdfWr(filter_no, 0x0);
				MAC_L3L4CR_L3SAIM0_UdfWr(filter_no, 0x0);
			}
		}
		else {
			if (enb_dis == 1) {
				/* Enable L3 filters for IPv4 DESTINATION addr matching */
				MAC_L3L4CR_L3DAM0_UdfWr(filter_no, 0x1);
				MAC_L3L4CR_L3DAIM0_UdfWr(filter_no, perfect_inverse_match);
			}
			else {
				/* Disable L3 filters for IPv4 DESTINATION addr matching */
				MAC_L3L4CR_L3DAM0_UdfWr(filter_no, 0x0);
				MAC_L3L4CR_L3DAIM0_UdfWr(filter_no, 0x0);
			}
		}
	}

  return Y_SUCCESS;
}



/*!
* \brief This sequence is used to configure MAC in differnet pkt processing
* modes like promiscuous, multicast, unicast, hash unicast/multicast.
* \param[in] pr_mode
* \param[in] huc_mode
* \param[in] hmc_mode
* \param[in] pm_mode
* \param[in] hpf_mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_mac_pkt_filter_reg(UCHAR pr_mode,
                                     UCHAR huc_mode,
                                     UCHAR hmc_mode,
                                     UCHAR pm_mode,
                                     UCHAR hpf_mode)
{
  ULONG varMAC_MPFR;

  /* configure device in differnet modes */
  /* promiscuous, hash unicast, hash multicast, */
  /* all multicast and perfect/hash filtering mode. */
  MAC_MPFR_RgRd(varMAC_MPFR);
  varMAC_MPFR = varMAC_MPFR & (ULONG)(0x803103e8);
  varMAC_MPFR = varMAC_MPFR | ((pr_mode) << 0) | ((huc_mode) << 1) | ((hmc_mode) << 2) |
                ((pm_mode) << 4) | ((hpf_mode) << 10);
  MAC_MPFR_RgWr(varMAC_MPFR);


  return Y_SUCCESS;
}



/*!
* \brief This sequence is used to enable/disable L3 and L4 filtering
* \param[in] filter_enb_dis
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_l3_l4_filter_enable(INT filter_enb_dis)
{

  MAC_MPFR_IPFE_UdfWr(filter_enb_dis);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to select perfect/inverse matching for L2 DA
* \param[in] perfect_inverse_match
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_l2_da_perfect_inverse_match(INT perfect_inverse_match)
{

  MAC_MPFR_DAIF_UdfWr(perfect_inverse_match);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to update the MAC address in last 96 MAC
* address Low and High register(32-127) for L2 layer filtering
* \param[in] idx
* \param[in] addr
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_mac_addr32_127_low_high_reg(INT idx,
                                              UCHAR addr[])
{

  MAC_MA32_127LR_RgWr(idx, (addr[0] | (addr[1] << 8) | (addr[2] << 16) | (addr[3] << 24)));
  MAC_MA32_127HR_ADDRHI_UdfWr(idx, (addr[4] | (addr[5] << 8)));
  MAC_MA32_127HR_AE_UdfWr(idx, 0x1);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to update the MAC address in first 31 MAC
* address Low and High register(1-31) for L2 layer filtering
* \param[in] idx
* \param[in] addr
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_mac_addr1_31_low_high_reg(INT idx,
                                            UCHAR addr[])
{

  MAC_MA1_31LR_RgWr(idx, (addr[0] | (addr[1] << 8) | (addr[2] << 16) | (addr[3] << 24)));
  MAC_MA1_31HR_ADDRHI_UdfWr(idx, (addr[4] | (addr[5] << 8)));
  MAC_MA1_31HR_AE_UdfWr(idx, 0x1);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure hash table register for
* hash address filtering
* \param[in] idx
* \param[in] data
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT update_hash_table_reg(INT idx,
                                 UINT data)
{

  MAC_HTR_RgWr(idx, data);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used check whether Tx drop status in the
* MTL is enabled or not, returns 1 if it is enabled and 0 if
* it is disabled.
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT drop_tx_status_enabled(void)
{
  ULONG varMTL_OMR;

  MTL_OMR_RgRd(varMTL_OMR);

  return GET_VALUE(varMTL_OMR, MTL_OMR_DTXSTS_LPOS, MTL_OMR_DTXSTS_HPOS);
}




/*!
* \brief This sequence is used configure MAC SSIR
* \param[in] ptp_clock
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_sub_second_increment(ULONG ptp_clock)
{
  ULONG val;
  ULONG varMAC_TCR;

  MAC_TCR_RgRd(varMAC_TCR);

  /* convert the PTP_CLOCK to nano second */
  /*  formula is : ((1/ptp_clock) * 1000000000) */
	/*  where, ptp_clock = 50MHz if FINE correction */
	/*  and ptp_clock = DWC_ETH_QOS_SYSCLOCK if COARSE correction */
  if (GET_VALUE(varMAC_TCR, MAC_TCR_TSCFUPDT_LPOS, MAC_TCR_TSCFUPDT_HPOS) == 1) {
    val = ((1 * 1000000000ull) / 50000000);
  }
  else {
    val = ((1 * 1000000000ull) / ptp_clock);
  }

  /* 0.465ns accurecy */
  if (GET_VALUE(varMAC_TCR, MAC_TCR_TSCTRLSSR_LPOS, MAC_TCR_TSCTRLSSR_HPOS) == 0) {
    val = (val * 1000) / 465;
  }
  MAC_SSIR_SSINC_UdfWr(val);

  return Y_SUCCESS;
}



/*!
* \brief This sequence is used get 64-bit system time in nano sec
* \return (unsigned long long) on success
* \retval ns
*/

static ULONG_LONG get_systime(void)
{
  ULONG_LONG ns;
  ULONG varmac_stnsr;
  ULONG varmac_stsr;

  MAC_STNSR_RgRd(varmac_stnsr);
  ns = GET_VALUE(varmac_stnsr, MAC_STNSR_TSSS_LPOS, MAC_STNSR_TSSS_HPOS);
  /* convert sec/high time value to nanosecond */
  MAC_STSR_RgRd(varmac_stsr);
  ns = ns + (varmac_stsr * 1000000000ull);

  return ns;
}






/*!
* \brief This sequence is used to adjust/update the system time
* \param[in] sec
* \param[in] nsec
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT adjust_systime(UINT sec,
						UINT nsec,
			  			INT add_sub,
						bool one_nsec_accuracy)
{
  ULONG retryCount = 100000;
  ULONG vy_count;
  volatile ULONG varMAC_TCR;

  /* wait for previous(if any) time adjust/update to complete. */

  /*Poll*/
  vy_count = 0;
  while(1){
    if(vy_count > retryCount) {
      return -Y_FAILURE;
    }

    MAC_TCR_RgRd(varMAC_TCR);
    if (GET_VALUE(varMAC_TCR, MAC_TCR_TSUPDT_LPOS, MAC_TCR_TSUPDT_HPOS) == 0) {
      break;
    }
    vy_count++;
		mdelay(1);
  }

  if (add_sub) {
    /* If the new sec value needs to be subtracted with
     * the system time, then MAC_STSUR reg should be
     * programmed with (2^32 – <new_sec_value>)
     * */
    sec = (0x100000000ull - sec);

    /* If the new nsec value need to be subtracted with
     * the system time, then MAC_STNSUR.TSSS field should be
     * programmed with,
     * (10^9 - <new_nsec_value>) if MAC_TCR.TSCTRLSSR is set or
     * (2^31 - <new_nsec_value> if MAC_TCR.TSCTRLSSR is reset)
     * */
  	if (one_nsec_accuracy)
      nsec = (0x3B9ACA00 - nsec);
   	else
      nsec = (0x80000000 - nsec);
  }

  MAC_STSUR_RgWr(sec);
  MAC_STNSUR_TSSS_UdfWr(nsec);
  MAC_STNSUR_ADDSUB_UdfWr(add_sub);

  /* issue command to initialize system time with the value */
  /* specified in MAC_STSUR and MAC_STNSUR. */
  MAC_TCR_TSUPDT_UdfWr(0x1);
  /* wait for present time initialize to complete. */

  /*Poll*/
  vy_count = 0;
  while(1){
    if(vy_count > retryCount) {
      return -Y_FAILURE;
    }

    MAC_TCR_RgRd(varMAC_TCR);
    if (GET_VALUE(varMAC_TCR, MAC_TCR_TSUPDT_LPOS, MAC_TCR_TSUPDT_HPOS) == 0) {
      break;
    }
    vy_count++;
		mdelay(1);
  }

  return Y_SUCCESS;
}



/*!
* \brief This sequence is used to adjust the ptp operating frequency.
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_addend(UINT data)
{
  ULONG retryCount = 100000;
  ULONG vy_count;
  volatile ULONG varMAC_TCR;

  /* wait for previous(if any) added update to complete. */

  /*Poll*/
  vy_count = 0;
  while(1){
    if(vy_count > retryCount) {
      return -Y_FAILURE;
    }

    MAC_TCR_RgRd(varMAC_TCR);
    if (GET_VALUE(varMAC_TCR, MAC_TCR_TSADDREG_LPOS, MAC_TCR_TSADDREG_HPOS) == 0) {
      break;
    }
    vy_count++;
    mdelay(1);
  }

  MAC_TAR_RgWr(data);
  /* issue command to update the added value */
  MAC_TCR_TSADDREG_UdfWr(0x1);
  /* wait for present added update to complete. */

  /*Poll*/
  vy_count = 0;
  while(1){
    if(vy_count > retryCount) {
      return -Y_FAILURE;
    }
    MAC_TCR_RgRd(varMAC_TCR);
    if (GET_VALUE(varMAC_TCR, MAC_TCR_TSADDREG_LPOS, MAC_TCR_TSADDREG_HPOS) == 0) {
      break;
    }
    vy_count++;
    mdelay(1);
  }

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to initialize the system time
* \param[in] sec
* \param[in] nsec
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT init_systime(UINT sec,
                        UINT nsec)
{
  ULONG retryCount = 100000;
  ULONG vy_count;
  volatile ULONG varMAC_TCR;

  /* wait for previous(if any) time initialize to complete. */

  /*Poll*/
  vy_count = 0;
  while(1){
    if(vy_count > retryCount) {
      return -Y_FAILURE;
    }

    MAC_TCR_RgRd(varMAC_TCR);
    if (GET_VALUE(varMAC_TCR, MAC_TCR_TSINIT_LPOS, MAC_TCR_TSINIT_HPOS) == 0) {
      break;
    }
    vy_count++;
    mdelay(1);
  }
  MAC_STSUR_RgWr(sec);
  MAC_STNSUR_RgWr(nsec);
  /* issue command to initialize system time with the value */
  /* specified in MAC_STSUR and MAC_STNSUR. */
  MAC_TCR_TSINIT_UdfWr(0x1);
  /* wait for present time initialize to complete. */

  /*Poll*/
  vy_count = 0;
  while(1){
    if(vy_count > retryCount) {
      return -Y_FAILURE;
    }

    MAC_TCR_RgRd(varMAC_TCR);
    if (GET_VALUE(varMAC_TCR, MAC_TCR_TSINIT_LPOS, MAC_TCR_TSINIT_HPOS) == 0) {
      break;
    }
    vy_count++;
    mdelay(1);
  }

  return Y_SUCCESS;
}





/*!
* \brief This sequence is used to enable HW time stamping
* and receive frames
* \param[in] count
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_hw_time_stamping(UINT config_val)
{

  MAC_TCR_RgWr(config_val);

  return Y_SUCCESS;
}





/*!
* \brief This sequence is used get the 64-bit of the timestamp
* captured by the device for the corresponding received packet
* in nanosecond.
* \param[in] rxdesc
* \return (unsigned long long) on success
* \retval ns
*/

static ULONG_LONG get_rx_tstamp(t_RX_CONTEXT_DESC *rxdesc)
{
  ULONG_LONG ns;
  ULONG varrdes1;

  RX_CONTEXT_DESC_RDES0_Ml_Rd(rxdesc->RDES0, ns);
  RX_CONTEXT_DESC_RDES1_Ml_Rd(rxdesc->RDES1, varrdes1);
  ns = ns + (varrdes1 * 1000000000ull);

  return ns;
}





/*!
* \brief This sequence is used to check whether the captured timestamp
* for the corresponding received packet is valid or not.
* Returns 0 if no context descriptor
* Returns 1 if timestamp is valid
* Returns 2 if time stamp is corrupted
* \param[in] rxdesc
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static UINT get_rx_tstamp_status(t_RX_CONTEXT_DESC *rxdesc)
{
  UINT varOWN;
  UINT varCTXT;
  UINT varRDES0;
  UINT varRDES1;

  /* check for own bit and CTXT bit */
  RX_CONTEXT_DESC_RDES3_OWN_Mlf_Rd(rxdesc->RDES3, varOWN);
  RX_CONTEXT_DESC_RDES3_CTXT_Mlf_Rd(rxdesc->RDES3, varCTXT);
  if ((varOWN == 0) && (varCTXT == 0x1)) {
    RX_CONTEXT_DESC_RDES0_Ml_Rd(rxdesc->RDES0, varRDES0);
    RX_CONTEXT_DESC_RDES1_Ml_Rd(rxdesc->RDES1, varRDES1);
    if ((varRDES0 == 0xffffffff) && (varRDES1 == 0xffffffff)) {
      /* time stamp is corrupted */
      return 2;
    }
    else {
      /* time stamp is valid */
      return 1;
    }
  }
  else {
    /* no CONTEX desc to hold time stamp value */
    return 0;
  }
}




/*!
* \brief This sequence is used to check whether the timestamp value
* is available in a context descriptor or not. Returns 1 if timestamp
* is available else returns 0
* \param[in] rxdesc
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static UINT rx_tstamp_available(t_RX_NORMAL_DESC *rxdesc)
{
  UINT varRS1V;
  UINT varTSA;

  RX_NORMAL_DESC_RDES3_RS1V_Mlf_Rd(rxdesc->RDES3, varRS1V);
  if (varRS1V == 1) {
    RX_NORMAL_DESC_RDES1_TSA_Mlf_Rd(rxdesc->RDES1, varTSA);
    return varTSA;
  }
  else {
    return 0;
  }
}




/*!
* \brief This sequence is used get the least 64-bit of the timestamp
* captured by the device for the corresponding transmit packet in nanosecond
* \return (unsigned long long) on success
* \retval ns
*/

static ULONG_LONG get_tx_tstamp_via_reg(void)
{
  ULONG_LONG ns;
  ULONG varmac_ttn;

  MAC_TTSN_TXTSSTSLO_UdfRd(ns);
  MAC_TTN_TXTSSTSHI_UdfRd(varmac_ttn);
  ns = ns + (varmac_ttn * 1000000000ull);

  return ns;
}






/*!
* \brief This sequence is used to check whether a timestamp has been
* captured for the corresponding transmit packet. Returns 1 if
* timestamp is taken else returns 0
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static UINT get_tx_tstamp_status_via_reg(void)
{
  ULONG varMAC_TCR;
  ULONG varMAC_TTSN;

  /* device is configured to overwrite the timesatmp of */
  /* eariler packet if driver has not yet read it. */
  MAC_TCR_RgRd(varMAC_TCR);
  if (GET_VALUE(varMAC_TCR, MAC_TCR_TXTSSTSM_LPOS, MAC_TCR_TXTSSTSM_HPOS) == 1) {
    /* nothing to do */
  }
  else {
    /* timesatmp of the current pkt is ignored or not captured */
    MAC_TTSN_RgRd(varMAC_TTSN);
    if (GET_VALUE(varMAC_TTSN, MAC_TTSN_TXTSSTSMIS_LPOS, MAC_TTSN_TXTSSTSMIS_HPOS) == 1) {
      return 0;
    }
    else {
      return 1;
    }
  }

  return 0;
}





/*!
* \brief This sequence is used get the 64-bit of the timestamp captured
* by the device for the corresponding transmit packet in nanosecond.
* \param[in] txdesc
* \return (unsigned long long) on success
* \retval ns
*/

static ULONG_LONG get_tx_tstamp(t_TX_NORMAL_DESC *txdesc)
{
  ULONG_LONG ns;
  ULONG vartdes1;

  TX_NORMAL_DESC_TDES0_Ml_Rd(txdesc->TDES0, ns);
  TX_NORMAL_DESC_TDES1_Ml_Rd(txdesc->TDES1, vartdes1);
  ns = ns + (vartdes1 * 1000000000ull);

  return ns;
}




/*!
* \brief This sequence is used to check whether a timestamp has been
* captured for the corresponding transmit packet. Returns 1 if
* timestamp is taken else returns 0
* \param[in] txdesc
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static UINT get_tx_tstamp_status(t_TX_NORMAL_DESC *txdesc)
{
  UINT varTDES3;

  TX_NORMAL_DESC_TDES3_Ml_Rd(txdesc->TDES3, varTDES3);

  return (varTDES3 & 0x20000);
}




/*!
* \brief This sequence is used to enable/disable split header feature
* \param[in] qInx
* \param[in] sph_en
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_split_header_mode(UINT qInx,
                             USHORT sph_en)
{

  DMA_CR_SPH_UdfWr(qInx, sph_en);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure header size in case of split header feature
* \param[in] header_size
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_header_size(USHORT header_size)
{

  if (header_size == 64) {
    MAC_MECR_HDSMS_UdfWr(0);
  }
  else if (header_size == 128) {
    MAC_MECR_HDSMS_UdfWr(0x1);
  }
  else if (header_size == 256) {
    MAC_MECR_HDSMS_UdfWr(0x2);
  }
  else if (header_size == 512) {
    MAC_MECR_HDSMS_UdfWr(0x3);
  }
  else {
    MAC_MECR_HDSMS_UdfWr(0x4);
  }

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to set tx queue operating mode for Queue[0 - 7]
* \param[in] qInx
* \param[in] q_mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_tx_queue_operating_mode(UINT qInx,
                                       UINT q_mode)
{

  MTL_QTOMR_TXQEN_UdfWr(qInx, q_mode);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to select Tx Scheduling Algorithm for AVB feature for Queue[1 - 7]
* \param[in] avb_algo
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_avb_algorithm(UINT qInx, UCHAR avb_algo)
{

  MTL_QECR_AVALG_UdfWr(qInx, avb_algo);

  return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure credit-control for Queue[1 - 7]
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_credit_control(UINT qInx, UINT cc)
{

  MTL_QECR_CC_UdfWr(qInx, cc);

  return Y_SUCCESS;
}



/*!
* \brief This sequence is used to configure send slope credit value
* required for the credit-based shaper alogorithm for Queue[1 - 7]
* \param[in] qInx
* \param[in] sendSlope
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_send_slope(UINT qInx,
                          UINT sendSlope)
{

  MTL_QSSCR_SSC_UdfWr(qInx, sendSlope);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure idle slope credit value
* required for the credit-based shaper alogorithm for Queue[1 - 7]
* \param[in] qInx
* \param[in] idleSlope
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_idle_slope(UINT qInx,
                          UINT idleSlope)
{

  MTL_QW_ISCQW_UdfWr(qInx, idleSlope);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure low credit value
* required for the credit-based shaper alogorithm for Queue[1 - 7]
* \param[in] qInx
* \param[in] lowCredit
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_low_credit(UINT qInx,
			UINT lowCredit)
{
	INT lowCredit_neg = lowCredit;
	printk(KERN_CRIT "lowCreidt = %08x lowCredit_neg:%08x\n",
			lowCredit, lowCredit_neg);
	MTL_QLCR_LC_UdfWr(qInx, lowCredit_neg);

  MTL_QLCR_LC_UdfWr(qInx, lowCredit);

  return Y_SUCCESS;
}


/*!
* \brief This sequence is used to enable/disable slot number check When set,
* this bit enables the checking of the slot number programmed in the TX
* descriptor with the current reference given in the RSN field. The DMA fetches
* the data from the corresponding buffer only when the slot number is: equal to
* the reference slot number or  ahead of the reference slot number by one.
*
* \param[in] qInx
* \param[in] slot_check
*
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_slot_num_check(UINT qInx, UCHAR slot_check)
{

  DMA_SFCSR_ESC_UdfWr(qInx, slot_check);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to enable/disable advance slot check When set,
* this bit enables the DAM to fetch the data from the buffer when the slot
* number programmed in TX descriptor is equal to the reference slot number
* given in RSN field or ahead of the reference number by upto two slots
*
* \param[in] qInx
* \param[in] adv_slot_check
*
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_advance_slot_num_check(UINT qInx, UCHAR adv_slot_check)
{

  DMA_SFCSR_ASC_UdfWr(qInx, adv_slot_check);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure high credit value required
* for the credit-based shaper alogorithm for Queue[1 - 7]
* \param[in] qInx
* \param[in] hiCredit
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_high_credit(UINT qInx,
                           UINT hiCredit)
{

  MTL_QHCR_HC_UdfWr(qInx, hiCredit);

  return Y_SUCCESS;
}

/*!
* \brief This sequence is used to set weights for DCB feature for Queue[0 - 7]
* \param[in] qInx
* \param[in] q_weight
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_dcb_queue_weight(UINT qInx,
                                UINT q_weight)
{

  MTL_QW_ISCQW_UdfWr(qInx, q_weight);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to select Tx Scheduling Algorithm for DCB feature
* \param[in] dcb_algo
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_dcb_algorithm(UCHAR dcb_algo)
{

  MTL_OMR_SCHALG_UdfWr(dcb_algo);

  return Y_SUCCESS;
}

/*!
* \brief This sequence is used to get Tx queue count
* \param[in] count
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

UCHAR get_tx_queue_count(void)
{
	UCHAR count;
  ULONG varMAC_HFR2;

  MAC_HFR2_RgRd(varMAC_HFR2);
  count = GET_VALUE(varMAC_HFR2, MAC_HFR2_TXQCNT_LPOS, MAC_HFR2_TXQCNT_HPOS);

  return (count + 1);
}




/*!
* \brief This sequence is used to get Rx queue count
* \param[in] count
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

UCHAR get_rx_queue_count(void)
{
	UCHAR count;
  ULONG varMAC_HFR2;

  MAC_HFR2_RgRd(varMAC_HFR2);
  count = GET_VALUE(varMAC_HFR2, MAC_HFR2_RXQCNT_LPOS, MAC_HFR2_RXQCNT_HPOS);

  return (count + 1);
}




/*!
* \brief This sequence is used to disables all Tx/Rx MMC interrupts
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT disable_mmc_interrupts(void)
{

  /* disable all TX interrupts */
  MMC_INTR_MASK_TX_RgWr(0xffffffff);
  /* disable all RX interrupts */
  MMC_INTR_MASK_RX_RgWr(0xffffffff);
  MMC_IPC_INTR_MASK_RX_RgWr(0xffffffff); /* Disable MMC Rx Interrupts for IPC */
  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to configure MMC counters
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_mmc_counters(void)
{
  ULONG varMMC_CNTRL;

  /* set COUNTER RESET */
  /* set RESET ON READ */
  /* set COUNTER PRESET */
  /* set FULL_HALF PRESET */
  MMC_CNTRL_RgRd(varMMC_CNTRL);
  varMMC_CNTRL = varMMC_CNTRL & (ULONG)(0x10a);
  varMMC_CNTRL = varMMC_CNTRL | ((0x1) << 0) | ((0x1) << 2) | ((0x1) << 4) |
                ((0x1) << 5);
  MMC_CNTRL_RgWr(varMMC_CNTRL);


  return Y_SUCCESS;
}



/*!
* \brief This sequence is used to disable given DMA channel rx interrupts
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT disable_rx_interrupt(UINT qInx)
{

  DMA_IER_RBUE_UdfWr(qInx, 0);
  DMA_IER_RIE_UdfWr(qInx, 0);

  return Y_SUCCESS;
}




/*!
* \brief This sequence is used to enable given DMA channel rx interrupts
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT enable_rx_interrupt(UINT qInx)
{

  DMA_IER_RBUE_UdfWr(qInx, 0x1);
  DMA_IER_RIE_UdfWr(qInx, 0x1);

  return Y_SUCCESS;
}


static VOID configure_sa_via_reg(u32 cmd)
{
	MAC_MCR_SARC_UdfWr(cmd);
}

static VOID configure_mac_addr1_reg(UCHAR *mac_addr)
{
	MAC_MA1HR_RgWr(((mac_addr[5] << 8) | (mac_addr[4])));
	MAC_MA1LR_RgWr(((mac_addr[3] << 24) | (mac_addr[2] << 16) |
			(mac_addr[1] << 8) | (mac_addr[0])));
}

static VOID configure_mac_addr0_reg(UCHAR *mac_addr)
{
	MAC_MA0HR_RgWr(((mac_addr[5] << 8) | (mac_addr[4])));
	MAC_MA0LR_RgWr(((mac_addr[3] << 24) | (mac_addr[2] << 16) |
			(mac_addr[1] << 8) | (mac_addr[0])));
}

static VOID config_rx_outer_vlan_stripping(u32 cmd)
{
	MAC_VLANTR_EVLS_UdfWr(cmd);
}

static VOID config_rx_inner_vlan_stripping(u32 cmd)
{
	MAC_VLANTR_EIVLS_UdfWr(cmd);
}

static VOID config_ptpoffload_engine(UINT pto_cr, UINT mc_uc)
{
	MAC_PTO_CR_RgWr(pto_cr);
    MAC_TCR_TSENMACADDR_UdfWr(mc_uc);
}


static VOID configure_reg_vlan_control(struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data)
{
	USHORT vlan_id = desc_data->vlan_tag_id;
	UINT vlan_control = desc_data->tx_vlan_tag_ctrl;

	MAC_VLANTIRR_RgWr(((1 << 18) | (vlan_control << 16) | (vlan_id << 0)));
}

static VOID configure_desc_vlan_control(struct DWC_ETH_QOS_prv_data *pdata)
{
	MAC_VLANTIRR_RgWr((1 << 20));
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT configure_mac_for_vlan_pkt(void)
{

	/* Enable VLAN Tag stripping always */
	MAC_VLANTR_EVLS_UdfWr(0x3);
	/* Enable operation on the outer VLAN Tag, if present */
	MAC_VLANTR_ERIVLT_UdfWr(0);
	/* Disable double VLAN Tag processing on TX and RX */
	MAC_VLANTR_EDVLP_UdfWr(0);
	/* Enable VLAN Tag in RX Status. */
	MAC_VLANTR_EVLRXS_UdfWr(0x1);
	/* Disable VLAN Type Check */
	MAC_VLANTR_DOVLTC_UdfWr(0x1);

	/* configure MAC to get VLAN Tag to be inserted/replaced from */
	/* TX descriptor(context desc) */
	MAC_VLANTIRR_VLTI_UdfWr(0x1);
	/* insert/replace C_VLAN in 13th ans 14th bytes of transmitted frames */
	MAC_VLANTIRR_CSVL_UdfWr(0);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_pblx8(UINT qInx, UINT val)
{
	DMA_CR_PBLx8_UdfWr(qInx, val);

	return Y_SUCCESS;
}

/*!
* \return INT
* \retval programmed Tx PBL value
*/

static INT get_tx_pbl_val(UINT qInx)
{
	UINT tx_pbl;

	DMA_TCR_PBL_UdfRd(qInx, tx_pbl);

	return tx_pbl;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_tx_pbl_val(UINT qInx, UINT tx_pbl)
{
	DMA_TCR_PBL_UdfWr(qInx, tx_pbl);

	return Y_SUCCESS;
}

/*!
* \return INT
* \retval programmed Rx PBL value
*/

static INT get_rx_pbl_val(UINT qInx)
{
	UINT rx_pbl;

	DMA_RCR_PBL_UdfRd(qInx, rx_pbl);

	return rx_pbl;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_rx_pbl_val(UINT qInx, UINT rx_pbl)
{
	DMA_RCR_PBL_UdfWr(qInx, rx_pbl);

	return Y_SUCCESS;
}


/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_axi_rorl_val(UINT axi_rorl)
{
	DMA_SBUS_RD_OSR_LMT_UdfWr(axi_rorl);

	return Y_SUCCESS;
}


/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_axi_worl_val(UINT axi_worl)
{
	DMA_SBUS_WR_OSR_LMT_UdfWr(axi_worl);

	return Y_SUCCESS;
}


/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_axi_pbl_val(UINT axi_pbl)
{
	UINT varDMA_SBUS;

	DMA_SBUS_RgRd(varDMA_SBUS);
	varDMA_SBUS &= ~DMA_SBUS_AXI_PBL_MASK;
	varDMA_SBUS |= axi_pbl;
	DMA_SBUS_RgWr(varDMA_SBUS);

	return Y_SUCCESS;
}


/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_incr_incrx_mode(UINT val)
{
	DMA_SBUS_UNDEF_FB_UdfWr(val);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_osf_mode(UINT qInx, UINT val)
{
	DMA_TCR_OSP_UdfWr(qInx, val);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_rsf_mode(UINT qInx, UINT val)
{
	//if (qInx == 0) {
		//MTL_Q0ROMR_RSF_UdfWr(val);
	//}
	//else {
		MTL_QROMR_RSF_UdfWr(qInx, val);
	//}

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_tsf_mode(UINT qInx, UINT val)
{
	MTL_QTOMR_TSF_UdfWr(qInx, val);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_rx_threshold(UINT qInx, UINT val)
{
	MTL_QROMR_RTC_UdfWr(qInx, val);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_tx_threshold(UINT qInx, UINT val)
{
	MTL_QTOMR_TTC_UdfWr(qInx, val);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT config_rx_watchdog_timer(UINT qInx, u32 riwt)
{
	DMA_RIWTR_RWT_UdfWr(qInx, riwt);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT enable_magic_pmt_operation(void)
{
	MAC_PMTCSR_MGKPKTEN_UdfWr(0x1);
	MAC_PMTCSR_PWRDWN_UdfWr(0x1);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT disable_magic_pmt_operation(void)
{
	UINT varPMTCSR_PWRDWN;

	MAC_PMTCSR_MGKPKTEN_UdfWr(0x0);
	MAC_PMTCSR_PWRDWN_UdfRd(varPMTCSR_PWRDWN);
	if (varPMTCSR_PWRDWN == 0x1) {
		MAC_PMTCSR_PWRDWN_UdfWr(0x0);
	}

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT enable_remote_pmt_operation(void)
{
	MAC_PMTCSR_RWKPKTEN_UdfWr(0x1);
	MAC_PMTCSR_PWRDWN_UdfWr(0x1);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT disable_remote_pmt_operation(void)
{
	UINT varPMTCSR_PWRDWN;

	MAC_PMTCSR_RWKPKTEN_UdfWr(0x0);
	MAC_PMTCSR_PWRDWN_UdfRd(varPMTCSR_PWRDWN);
	if (varPMTCSR_PWRDWN == 0x1) {
		MAC_PMTCSR_PWRDWN_UdfWr(0x0);
	}

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT configure_rwk_filter_registers(UINT *value, UINT count)
{
	UINT i;

	MAC_PMTCSR_RWKFILTRST_UdfWr(1);
	for (i = 0; i < count; i++)
		MAC_RWPFFR_RgWr(value[i]);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT disable_tx_flow_ctrl(UINT qInx)
{

	MAC_QTFCR_TFE_UdfWr(qInx, 0);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT enable_tx_flow_ctrl(UINT qInx)
{

	MAC_QTFCR_TFE_UdfWr(qInx, 1);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT disable_rx_flow_ctrl(void)
{

	MAC_RFCR_RFE_UdfWr(0);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT enable_rx_flow_ctrl(void)
{

	MAC_RFCR_RFE_UdfWr(0x1);

	return Y_SUCCESS;
}

/*!
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT stop_dma_rx(UINT qInx)
{
  ULONG retryCount = 10;
  ULONG vy_count;
  volatile ULONG varDMA_DSR0;
  volatile ULONG varDMA_DSR1;
  volatile ULONG varDMA_DSR2;

  /* issue Rx dma stop command */
  DMA_RCR_ST_UdfWr(qInx, 0);

  /* wait for Rx DMA to stop, ie wait till Rx DMA
   * goes in either Running or Suspend state.
   * */
  if (qInx == 0) {

    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
        printk(KERN_ALERT "ERROR: Rx Channel 0 stop failed, DSR0 = %#lx\n",
			varDMA_DSR0);
        return -Y_FAILURE;
      }

      DMA_DSR0_RgRd(varDMA_DSR0);
      if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS0_LPOS, DMA_DSR0_RPS0_HPOS) == 0x3)
	|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS0_LPOS, DMA_DSR0_RPS0_HPOS) == 0x4)
	|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS0_LPOS, DMA_DSR0_RPS0_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 1) {

    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
        printk(KERN_ALERT "ERROR: Rx Channel 1 stop failed, DSR0 = %#lx\n",
			varDMA_DSR0);
        return -Y_FAILURE;
      }

      DMA_DSR0_RgRd(varDMA_DSR0);
      if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS1_LPOS, DMA_DSR0_RPS1_HPOS) == 0x3)
	|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS1_LPOS, DMA_DSR0_RPS1_HPOS) == 0x4)
	|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS1_LPOS, DMA_DSR0_RPS1_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 2) {

    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
        printk(KERN_ALERT "ERROR: Rx Channel 2 stop failed, DSR0 = %#lx\n",
			varDMA_DSR0);
        return -Y_FAILURE;
      }

      DMA_DSR0_RgRd(varDMA_DSR0);
      if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS2_LPOS, DMA_DSR0_RPS2_HPOS) == 0x3)
	|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS2_LPOS, DMA_DSR0_RPS2_HPOS) == 0x4)
	|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS2_LPOS, DMA_DSR0_RPS2_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 3) {

    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
        printk(KERN_ALERT "ERROR: Rx Channel 3 stop failed, DSR0 = %#lx\n",
			varDMA_DSR1);
        return -Y_FAILURE;
      }

      DMA_DSR1_RgRd(varDMA_DSR1);
      if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS3_LPOS, DMA_DSR1_RPS3_HPOS) == 0x3)
	|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS3_LPOS, DMA_DSR1_RPS3_HPOS) == 0x4)
	|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS3_LPOS, DMA_DSR1_RPS3_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 4) {

    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
        printk(KERN_ALERT "ERROR: Rx Channel 4 stop failed, DSR0 = %#lx\n",
			varDMA_DSR1);
        return -Y_FAILURE;
      }

      DMA_DSR1_RgRd(varDMA_DSR1);
      if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS4_LPOS, DMA_DSR1_RPS4_HPOS) == 0x3)
	|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS4_LPOS, DMA_DSR1_RPS4_HPOS) == 0x4)
	|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS4_LPOS, DMA_DSR1_RPS4_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 5) {

    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
        printk(KERN_ALERT "ERROR: Rx Channel 5 stop failed, DSR0 = %#lx\n",
			varDMA_DSR1);
        return -Y_FAILURE;
      }

      DMA_DSR1_RgRd(varDMA_DSR1);
      if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS5_LPOS, DMA_DSR1_RPS5_HPOS) == 0x3)
	|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS5_LPOS, DMA_DSR1_RPS5_HPOS) == 0x4)
	|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS5_LPOS, DMA_DSR1_RPS5_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 6) {

    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
        printk(KERN_ALERT "ERROR: Rx Channel 6 stop failed, DSR0 = %#lx\n",
			varDMA_DSR1);
        return -Y_FAILURE;
      }

      DMA_DSR1_RgRd(varDMA_DSR1);
      if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS6_LPOS, DMA_DSR1_RPS6_HPOS) == 0x3)
	|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS6_LPOS, DMA_DSR1_RPS6_HPOS) == 0x4)
	|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS6_LPOS, DMA_DSR1_RPS6_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 7) {

    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
        printk(KERN_ALERT "ERROR: Rx Channel 7 stop failed, DSR0 = %#lx\n",
			varDMA_DSR2);
        return -Y_FAILURE;
      }

      DMA_DSR2_RgRd(varDMA_DSR2);
      if ((GET_VALUE(varDMA_DSR2, DMA_DSR2_RPS7_LPOS, DMA_DSR2_RPS7_HPOS) == 0x3)
	|| (GET_VALUE(varDMA_DSR2, DMA_DSR2_RPS7_LPOS, DMA_DSR2_RPS7_HPOS) == 0x4)
	|| (GET_VALUE(varDMA_DSR2, DMA_DSR2_RPS7_LPOS, DMA_DSR2_RPS7_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  }

  return Y_SUCCESS;
}


/*!
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT start_dma_rx(UINT qInx)
{

  DMA_RCR_ST_UdfWr(qInx, 0x1);

  return Y_SUCCESS;
}

/*!
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT stop_dma_tx(UINT qInx)
{
  ULONG retryCount = 10;
  ULONG vy_count;
  volatile ULONG varDMA_DSR0;
  volatile ULONG varDMA_DSR1;
  volatile ULONG varDMA_DSR2;

  /* issue Tx dma stop command */
  DMA_TCR_ST_UdfWr(qInx, 0);

  /* wait for Tx DMA to stop, ie wait till Tx DMA
   * goes in Suspend state or stopped state.
   */
  if (qInx == 0) {
    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
	printk(KERN_ALERT "ERROR: Channel 0 stop failed, DSR0 = %lx\n",
	  varDMA_DSR0);
        return -Y_FAILURE;
      }

      DMA_DSR0_RgRd(varDMA_DSR0);
      if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS0_LPOS, DMA_DSR0_TPS0_HPOS) == 0x6) ||
        (GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS0_LPOS, DMA_DSR0_TPS0_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 1) {
    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
	printk(KERN_ALERT "ERROR: Channel 1 stop failed, DSR0 = %lx\n",
	  varDMA_DSR0);
        return -Y_FAILURE;
      }

      DMA_DSR0_RgRd(varDMA_DSR0);
      if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS1_LPOS, DMA_DSR0_TPS1_HPOS) == 0x6) ||
        (GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS1_LPOS, DMA_DSR0_TPS1_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 2) {
    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
	printk(KERN_ALERT "ERROR: Channel 2 stop failed, DSR0 = %lx\n",
	  varDMA_DSR0);
        return -Y_FAILURE;
      }

      DMA_DSR0_RgRd(varDMA_DSR0);
      if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS2_LPOS, DMA_DSR0_TPS2_HPOS) == 0x6) ||
        (GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS2_LPOS, DMA_DSR0_TPS2_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 3) {
    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
	printk(KERN_ALERT "ERROR: Channel 3 stop failed, DSR0 = %lx\n",
	  varDMA_DSR1);
        return -Y_FAILURE;
      }

      DMA_DSR1_RgRd(varDMA_DSR1);
      if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS3_LPOS, DMA_DSR1_TPS3_HPOS) == 0x6) ||
        (GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS3_LPOS, DMA_DSR1_TPS3_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 4) {
    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
	printk(KERN_ALERT "ERROR: Channel 4 stop failed, DSR0 = %lx\n",
	  varDMA_DSR1);
        return -Y_FAILURE;
      }

      DMA_DSR1_RgRd(varDMA_DSR1);
      if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS4_LPOS, DMA_DSR1_TPS4_HPOS) == 0x6) ||
        (GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS4_LPOS, DMA_DSR1_TPS4_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 5) {
    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
	printk(KERN_ALERT "ERROR: Channel 5 stop failed, DSR0 = %lx\n",
	  varDMA_DSR1);
        return -Y_FAILURE;
      }

      DMA_DSR1_RgRd(varDMA_DSR1);
      if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS5_LPOS, DMA_DSR1_TPS5_HPOS) == 0x6) ||
        (GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS5_LPOS, DMA_DSR1_TPS5_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 6) {
    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
	printk(KERN_ALERT "ERROR: Channel 6 stop failed, DSR0 = %lx\n",
	  varDMA_DSR1);
        return -Y_FAILURE;
      }

      DMA_DSR1_RgRd(varDMA_DSR1);
      if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS6_LPOS, DMA_DSR1_TPS6_HPOS) == 0x6) ||
        (GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS6_LPOS, DMA_DSR1_TPS6_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  } else if (qInx == 7) {
    /*Poll*/
    vy_count = 0;
    while(1){
      if(vy_count > retryCount) {
	printk(KERN_ALERT "ERROR: Channel 7 stop failed, DSR0 = %lx\n",
	  varDMA_DSR2);
        return -Y_FAILURE;
      }

      DMA_DSR2_RgRd(varDMA_DSR2);
      if ((GET_VALUE(varDMA_DSR2, DMA_DSR2_TPS7_LPOS, DMA_DSR2_TPS7_HPOS) == 0x6) ||
        (GET_VALUE(varDMA_DSR2, DMA_DSR2_TPS7_LPOS, DMA_DSR2_TPS7_HPOS) == 0x0)) {
        break;
      }
      vy_count++;
      mdelay(1);
    }
  }

  return Y_SUCCESS;
}

/*!
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT start_dma_tx(UINT qInx)
{

  DMA_TCR_ST_UdfWr(qInx, 0x1);

  return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT stop_mac_tx_rx(void)
{
	ULONG varMAC_MCR;

	MAC_MCR_RgRd(varMAC_MCR);
	varMAC_MCR = varMAC_MCR & (ULONG) (0xffffff7c);
	varMAC_MCR = varMAC_MCR | ((0) << 1) | ((0) << 0);
	MAC_MCR_RgWr(varMAC_MCR);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT start_mac_tx_rx(void)
{
	ULONG varMAC_MCR;

	MAC_MCR_RgRd(varMAC_MCR);
	varMAC_MCR = varMAC_MCR & (ULONG) (0xffffff7c);
	varMAC_MCR = varMAC_MCR | ((0x1) << 1) | ((0x1) << 0);
	MAC_MCR_RgWr(varMAC_MCR);

	return Y_SUCCESS;
}


/*!
* \brief This sequence is used to enable DMA interrupts
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT enable_dma_interrupts(UINT qInx)
{
	UINT tmp;
	ULONG varDMA_SR;
	ULONG varDMA_IER;

	/* clear all the interrupts which are set */
	DMA_SR_RgRd(qInx, varDMA_SR);
	tmp = varDMA_SR;
	DMA_SR_RgWr(qInx, tmp);
	/* Enable following interrupts for Queue 0 */
	/* TXSE - Transmit Stoppped Enable */
	/* RIE - Receive Interrupt Enable */
	/* RBUE - Receive Buffer Unavailable Enable  */
	/* RSE - Receive Stoppped Enable */
	/* AIE - Abnormal Interrupt Summary Enable */
	/* NIE - Normal Interrupt Summary Enable */
	/* FBE - Fatal Bus Error Enable */
	DMA_IER_RgRd(qInx, varDMA_IER);
	varDMA_IER = varDMA_IER & (ULONG) (0x2e00);
#ifdef DWC_ETH_QOS_VER_4_0    
	varDMA_IER = varDMA_IER | ((0x1) << 1) |
	    ((0x1) << 6) | ((0x1) << 7) | ((0x1) << 8) | ((0x1) << 15) |
	    ((0x1) << 16) | ((0x1) << 12);
#else
	varDMA_IER = varDMA_IER | ((0x1) << 1) | ((0x1) << 2) |
	    ((0x1) << 6) | ((0x1) << 7) | ((0x1) << 8) | ((0x1) << 14) |
	    ((0x1) << 15) | ((0x1) << 12);
#endif 
#ifndef DWC_ETH_QOS_TXPOLLING_MODE_ENABLE
	/* TIE - Transmit Interrupt Enable */
	/* TBUE - Transmit Buffer Unavailable Enable */
	varDMA_IER |= ((0x1) << 0) | ((0x1) << 2);
#endif
	DMA_IER_RgWr(qInx, varDMA_IER);

	return Y_SUCCESS;
}


/*!
* \brief This sequence is used to configure the MAC registers for
* GMII-1000Mbps speed
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_gmii_speed(void)
{

	MAC_MCR_PS_UdfWr(0);
	MAC_MCR_FES_UdfWr(0);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the MAC registers for
* MII-10Mpbs speed
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_mii_speed_10(void)
{

	MAC_MCR_PS_UdfWr(0x1);
	MAC_MCR_FES_UdfWr(0);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the MAC registers for
* MII-100Mpbs speed
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_mii_speed_100(void)
{

	MAC_MCR_PS_UdfWr(0x1);
	MAC_MCR_FES_UdfWr(0x1);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the MAC registers for
* half duplex mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_half_duplex(void)
{

	MAC_MCR_DM_UdfWr(0);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the MAC registers for
* full duplex mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_full_duplex(void)
{

	MAC_MCR_DM_UdfWr(0x1);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the device in list of
* multicast mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_multicast_list_mode(void)
{

	MAC_MPFR_HMC_UdfWr(0);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the device in unicast mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_unicast_mode(void)
{

	MAC_MPFR_HUC_UdfWr(0);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the device in all multicast mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_all_multicast_mode(void)
{

	MAC_MPFR_PM_UdfWr(0x1);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the device in promiscuous mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT set_promiscuous_mode(void)
{

	MAC_MPFR_PR_UdfWr(0x1);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to write into phy registers
* \param[in] phy_id
* \param[in] phy_reg
* \param[in] phy_reg_data
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT write_phy_regs(INT phy_id, INT phy_reg, INT phy_reg_data)
{
	ULONG retryCount = 1000;
	ULONG vy_count;
	volatile ULONG varMAC_GMIIAR;

	/* wait for any previous MII read/write operation to complete */

	/*Poll Until Poll Condition */
	vy_count = 0;
	while (1) {
		if (vy_count > retryCount) {
			return -Y_FAILURE;
		} else {
			vy_count++;
			mdelay(1);
		}
		MAC_GMIIAR_RgRd(varMAC_GMIIAR);
		if (GET_VALUE(varMAC_GMIIAR, MAC_GMIIAR_GB_LPOS, MAC_GMIIAR_GB_HPOS) == 0) {
			break;
		}
	}
	/* write the data */
	MAC_GMIIDR_GD_UdfWr(phy_reg_data);
	/* initiate the MII write operation by updating desired */
	/* phy address/id (0 - 31) */
	/* phy register offset */
	/* CSR Clock Range (20 - 35MHz) */
	/* Select write operation */
	/* set busy bit */
	MAC_GMIIAR_RgRd(varMAC_GMIIAR);
	varMAC_GMIIAR = varMAC_GMIIAR & (ULONG) (0x12);
	varMAC_GMIIAR =
	    varMAC_GMIIAR | ((phy_id) << 21) | ((phy_reg) << 16) | ((0x2) << 8)
	    | ((0x1) << 2) | ((0x1) << 0);
	MAC_GMIIAR_RgWr(varMAC_GMIIAR);

	/*DELAY IMPLEMENTATION USING udelay() */
	udelay(10);
	/* wait for MII write operation to complete */

	/*Poll Until Poll Condition */
	vy_count = 0;
	while (1) {
		if (vy_count > retryCount) {
			return -Y_FAILURE;
		} else {
			vy_count++;
			mdelay(1);
		}
		MAC_GMIIAR_RgRd(varMAC_GMIIAR);
		if (GET_VALUE(varMAC_GMIIAR, MAC_GMIIAR_GB_LPOS, MAC_GMIIAR_GB_HPOS) == 0) {
			break;
		}
	}

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to read the phy registers
* \param[in] phy_id
* \param[in] phy_reg
* \param[out] phy_reg_data
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT read_phy_regs(INT phy_id, INT phy_reg, INT *phy_reg_data)
{
	ULONG retryCount = 1000;
	ULONG vy_count;
	volatile ULONG varMAC_GMIIAR;
	ULONG varMAC_GMIIDR;

	/* wait for any previous MII read/write operation to complete */

	/*Poll Until Poll Condition */
	vy_count = 0;
	while (1) {
		if (vy_count > retryCount) {
			return -Y_FAILURE;
		} else {
			vy_count++;
			mdelay(1);
		}
		MAC_GMIIAR_RgRd(varMAC_GMIIAR);
		if (GET_VALUE(varMAC_GMIIAR, MAC_GMIIAR_GB_LPOS, MAC_GMIIAR_GB_HPOS) == 0) {
			break;
		}
	}
	/* initiate the MII read operation by updating desired */
	/* phy address/id (0 - 31) */
	/* phy register offset */
	/* CSR Clock Range (20 - 35MHz) */
	/* Select read operation */
	/* set busy bit */
	MAC_GMIIAR_RgRd(varMAC_GMIIAR);
	varMAC_GMIIAR = varMAC_GMIIAR & (ULONG) (0x12);
	varMAC_GMIIAR =
	    varMAC_GMIIAR | ((phy_id) << 21) | ((phy_reg) << 16) | ((0x2) << 8)
	    | ((0x3) << 2) | ((0x1) << 0);
	MAC_GMIIAR_RgWr(varMAC_GMIIAR);

	/*DELAY IMPLEMENTATION USING udelay() */
	udelay(10);
	/* wait for MII write operation to complete */

	/*Poll Until Poll Condition */
	vy_count = 0;
	while (1) {
		if (vy_count > retryCount) {
			return -Y_FAILURE;
		} else {
			vy_count++;
			mdelay(1);
		}
		MAC_GMIIAR_RgRd(varMAC_GMIIAR);
		if (GET_VALUE(varMAC_GMIIAR, MAC_GMIIAR_GB_LPOS, MAC_GMIIAR_GB_HPOS) == 0) {
			break;
		}
	}
	/* read the data */
	MAC_GMIIDR_RgRd(varMAC_GMIIDR);
	*phy_reg_data =
	    GET_VALUE(varMAC_GMIIDR, MAC_GMIIDR_GD_LPOS, MAC_GMIIDR_GD_HPOS);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to check whether transmitted pkts have
* fifo under run loss error or not, returns 1 if fifo under run error
* else returns 0
* \param[in] txdesc
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT tx_fifo_underrun(t_TX_NORMAL_DESC *txdesc)
{
	UINT varTDES3;

	/* check TDES3.UF bit */
	TX_NORMAL_DESC_TDES3_Ml_Rd(txdesc->TDES3, varTDES3);
	if ((varTDES3 & 0x4) == 0x4) {
		return 1;
	} else {
		return 0;
	}
}

/*!
* \brief This sequence is used to check whether transmitted pkts have
* carrier loss error or not, returns 1 if carrier loss error else returns 0
* \param[in] txdesc
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT tx_carrier_lost_error(t_TX_NORMAL_DESC *txdesc)
{
	UINT varTDES3;

	/* check TDES3.LoC and TDES3.NC bits */
	TX_NORMAL_DESC_TDES3_Ml_Rd(txdesc->TDES3, varTDES3);
	if (((varTDES3 & 0x800) == 0x800) || ((varTDES3 & 0x400) == 0x400)) {
		return 1;
	} else {
		return 0;
	}
}

/*!
* \brief This sequence is used to check whether transmission is aborted
* or not returns 1 if transmission is aborted else returns 0
* \param[in] txdesc
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT tx_aborted_error(t_TX_NORMAL_DESC *txdesc)
{
	UINT varTDES3;

	/* check for TDES3.LC and TDES3.EC */
	TX_NORMAL_DESC_TDES3_Ml_Rd(txdesc->TDES3, varTDES3);
	if (((varTDES3 & 0x200) == 0x200) || ((varTDES3 & 0x100) == 0x100)) {
		return 1;
	} else {
		return 0;
	}
}

/*!
* \brief This sequence is used to check whether the pkt transmitted is
* successfull or not, returns 1 if transmission is success else returns 0
* \param[in] txdesc
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT tx_complete(t_TX_NORMAL_DESC *txdesc)
{
	UINT varOWN;

	TX_NORMAL_DESC_TDES3_OWN_Mlf_Rd(txdesc->TDES3, varOWN);
	if (varOWN == 0) {
		return 1;
	} else {
		return 0;
	}
}

/*!
* \brief This sequence is used to check whethet rx csum is enabled/disabled
* returns 1 if rx csum is enabled else returns 0
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT get_rx_csum_status(void)
{
	ULONG varMAC_MCR;

	MAC_MCR_RgRd(varMAC_MCR);
	if (GET_VALUE(varMAC_MCR, MAC_MCR_IPC_LPOS, MAC_MCR_IPC_HPOS) == 0x1) {
		return 1;
	} else {
		return 0;
	}
}

/*!
* \brief This sequence is used to disable the rx csum
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT disable_rx_csum(void)
{

	/* enable rx checksum */
	MAC_MCR_IPC_UdfWr(0);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to enable the rx csum
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT enable_rx_csum(void)
{

	/* enable rx checksum */
	MAC_MCR_IPC_UdfWr(0x1);

	return Y_SUCCESS;
}


/*!
* \brief This sequence is used to reinitialize the TX descriptor fields,
* so that device can reuse the descriptors
* \param[in] idx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT tx_descriptor_reset(UINT idx, struct DWC_ETH_QOS_prv_data *pdata,
				UINT qInx)
{
	struct s_TX_NORMAL_DESC *TX_NORMAL_DESC =
		GET_TX_DESC_PTR(qInx, idx);

	DBGPR("-->tx_descriptor_reset\n");

	/* update buffer 1 address pointer to zero */
	TX_NORMAL_DESC_TDES0_Ml_Wr(TX_NORMAL_DESC->TDES0, 0);
	/* update buffer 2 address pointer to zero */
	TX_NORMAL_DESC_TDES1_Ml_Wr(TX_NORMAL_DESC->TDES1, 0);
	/* set all other control bits (IC, TTSE, B2L & B1L) to zero */
	TX_NORMAL_DESC_TDES2_Ml_Wr(TX_NORMAL_DESC->TDES2, 0);
	/* set all other control bits (OWN, CTXT, FD, LD, CPC, CIC etc) to zero */
	TX_NORMAL_DESC_TDES3_Ml_Wr(TX_NORMAL_DESC->TDES3, 0);

	DBGPR("<--tx_descriptor_reset\n");

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to reinitialize the RX descriptor fields,
* so that device can reuse the descriptors
* \param[in] idx
* \param[in] pdata
*/

static void rx_descriptor_reset(UINT idx,
				struct DWC_ETH_QOS_prv_data *pdata,
				unsigned int inte,
				UINT qInx)
{
	struct DWC_ETH_QOS_rx_buffer *buffer = GET_RX_BUF_PTR(qInx, idx);
	struct s_RX_NORMAL_DESC *RX_NORMAL_DESC = GET_RX_DESC_PTR(qInx, idx);

	DBGPR("-->rx_descriptor_reset\n");

	memset(RX_NORMAL_DESC, 0, sizeof(struct s_RX_NORMAL_DESC));
	/* update buffer 1 address pointer */
	RX_NORMAL_DESC_RDES0_Ml_Wr(RX_NORMAL_DESC->RDES0, buffer->dma);
	/* set to zero */
	RX_NORMAL_DESC_RDES1_Ml_Wr(RX_NORMAL_DESC->RDES1, 0);

	if ((pdata->dev->mtu > DWC_ETH_QOS_ETH_FRAME_LEN) ||
			(pdata->rx_split_hdr == 1)) {
		/* update buffer 2 address pointer */
		RX_NORMAL_DESC_RDES2_Ml_Wr(RX_NORMAL_DESC->RDES2, buffer->dma2);
		/* set control bits - OWN, INTE, BUF1V and BUF2V */
		RX_NORMAL_DESC_RDES3_Ml_Wr(RX_NORMAL_DESC->RDES3,
					   (0x83000000 | inte));
	} else {
		/* set buffer 2 address pointer to zero */
		RX_NORMAL_DESC_RDES2_Ml_Wr(RX_NORMAL_DESC->RDES2, 0);
		/* set control bits - OWN, INTE and BUF1V */
		RX_NORMAL_DESC_RDES3_Ml_Wr(RX_NORMAL_DESC->RDES3,
					   (0x81000000 | inte));
	}

	DBGPR("<--rx_descriptor_reset\n");
}

/*!
* \brief This sequence is used to initialize the rx descriptors.
* \param[in] pdata
*/

static void rx_descriptor_init(struct DWC_ETH_QOS_prv_data *pdata, UINT qInx)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data =
	    GET_RX_WRAPPER_DESC(qInx);
	struct DWC_ETH_QOS_rx_buffer *buffer =
	    GET_RX_BUF_PTR(qInx, rx_desc_data->cur_rx);
	struct s_RX_NORMAL_DESC *RX_NORMAL_DESC =
	    GET_RX_DESC_PTR(qInx, rx_desc_data->cur_rx);
	INT i;
	INT start_index = rx_desc_data->cur_rx;
	INT last_index;

	DBGPR("-->rx_descriptor_init\n");

	/* initialize all desc */

	for (i = 0; i < RX_DESC_CNT; i++) {
		memset(RX_NORMAL_DESC, 0, sizeof(struct s_RX_NORMAL_DESC));
		/* update buffer 1 address pointer */
		RX_NORMAL_DESC_RDES0_Ml_Wr(RX_NORMAL_DESC->RDES0, buffer->dma);
		/* set to zero  */
		RX_NORMAL_DESC_RDES1_Ml_Wr(RX_NORMAL_DESC->RDES1, 0);

		if ((pdata->dev->mtu > DWC_ETH_QOS_ETH_FRAME_LEN) ||
			(pdata->rx_split_hdr == 1)) {
			/* update buffer 2 address pointer */
			RX_NORMAL_DESC_RDES2_Ml_Wr(RX_NORMAL_DESC->RDES2,
						   buffer->dma2);
			/* set control bits - OWN, INTE, BUF1V and BUF2V */
			RX_NORMAL_DESC_RDES3_Ml_Wr(RX_NORMAL_DESC->RDES3,
						   (0xc3000000));
		} else {
			/* set buffer 2 address pointer to zero */
			RX_NORMAL_DESC_RDES2_Ml_Wr(RX_NORMAL_DESC->RDES2, 0);
			/* set control bits - OWN, INTE and BUF1V */
			RX_NORMAL_DESC_RDES3_Ml_Wr(RX_NORMAL_DESC->RDES3,
						   (0xc1000000));
		}
		buffer->inte = (1 << 30);

		/* reconfigure INTE bit if RX watchdog timer is enabled */
		if (rx_desc_data->use_riwt) {
			if ((i % rx_desc_data->rx_coal_frames) != 0) {
				UINT varRDES3 = 0;
				RX_NORMAL_DESC_RDES3_Ml_Rd(RX_NORMAL_DESC->RDES3,
					varRDES3);
				/* reset INTE */
				RX_NORMAL_DESC_RDES3_Ml_Wr(RX_NORMAL_DESC->RDES3,
						(varRDES3 & ~(1 << 30)));
				buffer->inte = 0;
			}
		}

		INCR_RX_DESC_INDEX(rx_desc_data->cur_rx, 1);
		RX_NORMAL_DESC =
			GET_RX_DESC_PTR(qInx, rx_desc_data->cur_rx);
		buffer = GET_RX_BUF_PTR(qInx, rx_desc_data->cur_rx);
	}
	/* update the total no of Rx descriptors count */
	DMA_RDRLR_RgWr(qInx, (RX_DESC_CNT - 1));
	/* update the Rx Descriptor Tail Pointer */
	last_index = GET_CURRENT_RCVD_LAST_DESC_INDEX(start_index, 0);
	DMA_RDTP_RPDR_RgWr(qInx, GET_RX_DESC_DMA_ADDR(qInx, last_index));
	/* update the starting address of desc chain/ring */
	DMA_RDLAR_RgWr(qInx, GET_RX_DESC_DMA_ADDR(qInx, start_index));

	DBGPR("<--rx_descriptor_init\n");
}

/*!
* \brief This sequence is used to initialize the tx descriptors.
* \param[in] pdata
*/

static void tx_descriptor_init(struct DWC_ETH_QOS_prv_data *pdata,
				UINT qInx)
{
	struct DWC_ETH_QOS_tx_wrapper_descriptor *tx_desc_data =
		GET_TX_WRAPPER_DESC(qInx);
	struct s_TX_NORMAL_DESC *TX_NORMAL_DESC =
		GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
	INT i;
	INT start_index = tx_desc_data->cur_tx;

	DBGPR("-->tx_descriptor_init\n");

	/* initialze all descriptors. */

	for (i = 0; i < TX_DESC_CNT; i++) {
		/* update buffer 1 address pointer to zero */
		TX_NORMAL_DESC_TDES0_Ml_Wr(TX_NORMAL_DESC->TDES0, 0);
		/* update buffer 2 address pointer to zero */
		TX_NORMAL_DESC_TDES1_Ml_Wr(TX_NORMAL_DESC->TDES1, 0);
		/* set all other control bits (IC, TTSE, B2L & B1L) to zero */
		TX_NORMAL_DESC_TDES2_Ml_Wr(TX_NORMAL_DESC->TDES2, 0);
		/* set all other control bits (OWN, CTXT, FD, LD, CPC, CIC etc) to zero */
		TX_NORMAL_DESC_TDES3_Ml_Wr(TX_NORMAL_DESC->TDES3, 0);

		INCR_TX_DESC_INDEX(tx_desc_data->cur_tx, 1);
		TX_NORMAL_DESC = GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
	}
	/* update the total no of Tx descriptors count */
	DMA_TDRLR_RgWr(qInx, (TX_DESC_CNT - 1));
	/* update the starting address of desc chain/ring */
	DMA_TDLAR_RgWr(qInx, GET_TX_DESC_DMA_ADDR(qInx, start_index));

	DBGPR("<--tx_descriptor_init\n");
}


/*!
* \brief This sequence is used to prepare tx descriptor for
* packet transmission and issue the poll demand command to TxDMA
*
* \param[in] pdata
*/

static void pre_transmit(struct DWC_ETH_QOS_prv_data *pdata,
				UINT qInx)
{
	struct DWC_ETH_QOS_tx_wrapper_descriptor *tx_desc_data =
	    GET_TX_WRAPPER_DESC(qInx);
	struct DWC_ETH_QOS_tx_buffer *buffer =
	    GET_TX_BUF_PTR(qInx, tx_desc_data->cur_tx);
	struct s_TX_NORMAL_DESC *TX_NORMAL_DESC =
	    GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
	struct s_TX_CONTEXT_DESC *TX_CONTEXT_DESC =
	    GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
	UINT varcsum_enable;
	UINT varvlan_pkt;
	UINT varvt;
	INT i;
	INT start_index = tx_desc_data->cur_tx;
	INT last_index, original_start_index = tx_desc_data->cur_tx;
	struct s_tx_pkt_features *tx_pkt_features = GET_TX_PKT_FEATURES_PTR;
#ifdef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT
	INT update_tail = 0;
	UINT varQTDR;
#endif
	UINT vartso_enable = 0;
	UINT varmss = 0;
	UINT varpay_len = 0;
	UINT vartcp_hdr_len = 0;
	UINT varptp_enable = 0;
	INT total_len = 0;

	DBGPR("-->pre_transmit: qInx = %u\n", qInx);

#ifdef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT
	if (qInx == 0)
		MTL_Q0TDR_TXQSTS_UdfRd(varQTDR);
	else
		MTL_QTDR_TXQSTS_UdfRd(qInx, varQTDR);

	/* No activity on MAC Tx-Fifo and fifo is empty */
	if (0 == varQTDR) {
		/* disable MAC Transmit */
		MAC_MCR_TE_UdfWr(0);
		update_tail = 1;
	}
#endif

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	TX_PKT_FEATURES_PKT_ATTRIBUTES_VLAN_PKT_Mlf_Rd(
		tx_pkt_features->pkt_attributes, varvlan_pkt);
	if (varvlan_pkt == 0x1) {
		/* put vlan tag in contex descriptor and set other control
		 * bits accordingly */
		TX_PKT_FEATURES_VLAN_TAG_VT_Mlf_Rd(tx_pkt_features->vlan_tag,
						   varvt);
		TX_CONTEXT_DESC_TDES3_VT_Mlf_Wr(TX_CONTEXT_DESC->TDES3, varvt);
		TX_CONTEXT_DESC_TDES3_VLTV_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);
		TX_CONTEXT_DESC_TDES3_CTXT_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);
		TX_CONTEXT_DESC_TDES3_OWN_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);

		original_start_index = tx_desc_data->cur_tx;
		INCR_TX_DESC_INDEX(tx_desc_data->cur_tx, 1);
		start_index = tx_desc_data->cur_tx;
		TX_NORMAL_DESC =
			GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
		buffer = GET_TX_BUF_PTR(qInx, tx_desc_data->cur_tx);
	}
#endif	/* DWC_ETH_QOS_ENABLE_VLAN_TAG */
#ifdef DWC_ETH_QOS_ENABLE_DVLAN
	if (pdata->via_reg_or_desc == DWC_ETH_QOS_VIA_DESC) {
		/* put vlan tag in contex descriptor and set other control
		 * bits accordingly */

		if (pdata->in_out & DWC_ETH_QOS_DVLAN_OUTER) {
			TX_CONTEXT_DESC_TDES3_VT_Mlf_Wr(TX_CONTEXT_DESC->TDES3,
					pdata->outer_vlan_tag);
			TX_CONTEXT_DESC_TDES3_VLTV_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);
			/* operation (insertion/replacement/deletion/none) will be
 			 * specified in normal descriptor TDES2
 			 * */
		}
		if (pdata->in_out & DWC_ETH_QOS_DVLAN_INNER) {
			TX_CONTEXT_DESC_TDES2_IVT_Mlf_Wr(TX_CONTEXT_DESC->TDES2,
								pdata->inner_vlan_tag);
			TX_CONTEXT_DESC_TDES3_IVLTV_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);
			TX_CONTEXT_DESC_TDES3_IVTIR_Mlf_Wr(TX_CONTEXT_DESC->TDES3,
								pdata->op_type);
		}
		TX_CONTEXT_DESC_TDES3_CTXT_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);
		TX_CONTEXT_DESC_TDES3_OWN_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);

		original_start_index = tx_desc_data->cur_tx;
		INCR_TX_DESC_INDEX(tx_desc_data->cur_tx, 1);
		start_index = tx_desc_data->cur_tx;
		TX_NORMAL_DESC = GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
		buffer = GET_TX_BUF_PTR(qInx, tx_desc_data->cur_tx);
	}
#endif /* End of DWC_ETH_QOS_ENABLE_DVLAN */

	/* prepare CONTEXT descriptor for TSO */
	TX_PKT_FEATURES_PKT_ATTRIBUTES_TSO_ENABLE_Mlf_Rd(
		tx_pkt_features->pkt_attributes, vartso_enable);
	if (vartso_enable && (tx_pkt_features->mss != tx_desc_data->default_mss)) {
		/* get MSS and update */
		TX_PKT_FEATURES_MSS_MSS_Mlf_Rd(tx_pkt_features->mss, varmss);
		TX_CONTEXT_DESC_TDES2_MSS_Mlf_Wr(TX_CONTEXT_DESC->TDES2, varmss);
		/* set MSS valid, CTXT and OWN bits */
		TX_CONTEXT_DESC_TDES3_TCMSSV_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);
		TX_CONTEXT_DESC_TDES3_CTXT_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);
		TX_CONTEXT_DESC_TDES3_OWN_Mlf_Wr(TX_CONTEXT_DESC->TDES3, 0x1);

		/* DMA uses the MSS value programed in DMA_CR if driver
		 * doesn't provided the CONTEXT descriptor */
		DMA_CR_MSS_UdfWr(qInx, tx_pkt_features->mss);

		tx_desc_data->default_mss = tx_pkt_features->mss;

		original_start_index = tx_desc_data->cur_tx;
		INCR_TX_DESC_INDEX(tx_desc_data->cur_tx, 1);
		start_index = tx_desc_data->cur_tx;
		TX_NORMAL_DESC = GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
		buffer = GET_TX_BUF_PTR(qInx, tx_desc_data->cur_tx);
	}

	/* update the first buffer pointer and length */
	TX_NORMAL_DESC_TDES0_Ml_Wr(TX_NORMAL_DESC->TDES0, buffer->dma);
	TX_NORMAL_DESC_TDES2_HL_B1L_Mlf_Wr(TX_NORMAL_DESC->TDES2, buffer->len);
	if (buffer->dma2 != 0) {
		/* update the second buffer pointer and length */
		TX_NORMAL_DESC_TDES1_Ml_Wr(TX_NORMAL_DESC->TDES1, buffer->dma2);
		TX_NORMAL_DESC_TDES2_B2L_Mlf_Wr(TX_NORMAL_DESC->TDES2, buffer->len2);
	}

	if (vartso_enable) {
		/* update TCP payload length (only for the descriptor with FD set) */
		TX_PKT_FEATURES_PAY_LEN_Ml_Rd(tx_pkt_features->pay_len, varpay_len);
		/* TDES3[17:0] will be TCP payload length */
		TX_NORMAL_DESC->TDES3 |= varpay_len;
	} else {
		/* update total length of packet */
		GET_TX_TOT_LEN(GET_TX_BUF_PTR(qInx, 0), tx_desc_data->cur_tx,
				GET_CURRENT_XFER_DESC_CNT(qInx), total_len);
		TX_NORMAL_DESC_TDES3_FL_Mlf_Wr(TX_NORMAL_DESC->TDES3, total_len);
	}

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	/* Insert a VLAN tag with a tag value programmed in MAC Reg 24 or
	 * CONTEXT descriptor
	 * */
	if (tx_desc_data->vlan_tag_present && Y_FALSE == tx_desc_data->tx_vlan_tag_via_reg) {
		//printk(KERN_ALERT "VLAN control info update via descriptor\n\n");
		TX_NORMAL_DESC_TDES2_VTIR_Mlf_Wr(TX_NORMAL_DESC->TDES2,
						 tx_desc_data->tx_vlan_tag_ctrl);
	}
#endif	/* DWC_ETH_QOS_ENABLE_VLAN_TAG */

#ifdef DWC_ETH_QOS_ENABLE_DVLAN
	if (pdata->via_reg_or_desc == DWC_ETH_QOS_VIA_DESC) {
		if (pdata->in_out & DWC_ETH_QOS_DVLAN_OUTER) {
			TX_NORMAL_DESC_TDES2_VTIR_Mlf_Wr(TX_NORMAL_DESC->TDES2,
								pdata->op_type);
		}
	}
#endif /* End of DWC_ETH_QOS_ENABLE_DVLAN */


	/* Mark it as First Descriptor */
	TX_NORMAL_DESC_TDES3_FD_Mlf_Wr(TX_NORMAL_DESC->TDES3, 0x1);
	/* Enable CRC and Pad Insertion (NOTE: set this only
	 * for FIRST descriptor) */
	TX_NORMAL_DESC_TDES3_CPC_Mlf_Wr(TX_NORMAL_DESC->TDES3, 0);
	/* Mark it as NORMAL descriptor */
	TX_NORMAL_DESC_TDES3_CTXT_Mlf_Wr(TX_NORMAL_DESC->TDES3, 0);
	/* Enable HW CSUM */
	TX_PKT_FEATURES_PKT_ATTRIBUTES_CSUM_ENABLE_Mlf_Rd(tx_pkt_features->pkt_attributes,
		varcsum_enable);
	if (varcsum_enable == 0x1) {
		TX_NORMAL_DESC_TDES3_CIC_Mlf_Wr(TX_NORMAL_DESC->TDES3, 0x3);
	}
	/* configure SA Insertion Control */
	TX_NORMAL_DESC_TDES3_SAIC_Mlf_Wr(TX_NORMAL_DESC->TDES3,
					 pdata->tx_sa_ctrl_via_desc);
	if (vartso_enable) {
		/* set TSE bit */
		TX_NORMAL_DESC_TDES3_TSE_Mlf_Wr(TX_NORMAL_DESC->TDES3, 0x1);

		/* update tcp data offset or tcp hdr len */
		TX_PKT_FEATURES_TCP_HDR_LEN_Ml_Rd(tx_pkt_features->tcp_hdr_len, vartcp_hdr_len);
		/* convert to bit value */
		vartcp_hdr_len = vartcp_hdr_len/4;
		TX_NORMAL_DESC_TDES3_SLOTNUM_TCPHDRLEN_Mlf_Wr(TX_NORMAL_DESC->TDES3, vartcp_hdr_len);
	}

	/* enable timestamping */
	TX_PKT_FEATURES_PKT_ATTRIBUTES_PTP_ENABLE_Mlf_Rd(tx_pkt_features->pkt_attributes, varptp_enable);
	if (varptp_enable) {
		TX_NORMAL_DESC_TDES2_TTSE_Mlf_Wr(TX_NORMAL_DESC->TDES2, 0x1);
	}

	INCR_TX_DESC_INDEX(tx_desc_data->cur_tx, 1);
	TX_NORMAL_DESC = GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
	buffer = GET_TX_BUF_PTR(qInx, tx_desc_data->cur_tx);

	for (i = 1; i < GET_CURRENT_XFER_DESC_CNT(qInx); i++) {
		/* update the first buffer pointer and length */
		TX_NORMAL_DESC_TDES0_Ml_Wr(TX_NORMAL_DESC->TDES0, buffer->dma);
		TX_NORMAL_DESC_TDES2_HL_B1L_Mlf_Wr(TX_NORMAL_DESC->TDES2, buffer->len);
		if (buffer->dma2 != 0) {
			/* update the second buffer pointer and length */
			TX_NORMAL_DESC_TDES1_Ml_Wr(TX_NORMAL_DESC->TDES1, buffer->dma2);
			TX_NORMAL_DESC_TDES2_B2L_Mlf_Wr(TX_NORMAL_DESC->TDES2, buffer->len2);
		}

		/* set own bit */
		TX_NORMAL_DESC_TDES3_OWN_Mlf_Wr(TX_NORMAL_DESC->TDES3, 0x1);
		/* Mark it as NORMAL descriptor */
		TX_NORMAL_DESC_TDES3_CTXT_Mlf_Wr(TX_NORMAL_DESC->TDES3, 0);

		INCR_TX_DESC_INDEX(tx_desc_data->cur_tx, 1);
		TX_NORMAL_DESC = GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
		buffer = GET_TX_BUF_PTR(qInx, tx_desc_data->cur_tx);
	}
	/* Mark it as LAST descriptor */
	last_index =
		GET_CURRENT_XFER_LAST_DESC_INDEX(qInx, start_index, 0);
	TX_NORMAL_DESC = GET_TX_DESC_PTR(qInx, last_index);
	TX_NORMAL_DESC_TDES3_LD_Mlf_Wr(TX_NORMAL_DESC->TDES3, 0x1);
	/* set Interrupt on Completion for last descriptor */
#ifdef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT
	pdata->mac_enable_count += 1;
	if ((pdata->mac_enable_count % pdata->drop_tx_pktburstcnt) == 0)
		TX_NORMAL_DESC_TDES2_IC_Mlf_Wr(TX_NORMAL_DESC->TDES2, 0x1);
#else
	TX_NORMAL_DESC_TDES2_IC_Mlf_Wr(TX_NORMAL_DESC->TDES2, 0x1);
#endif

	/* set OWN bit of FIRST descriptor at end to avoid race condition */
	TX_NORMAL_DESC = GET_TX_DESC_PTR(qInx, start_index);
	TX_NORMAL_DESC_TDES3_OWN_Mlf_Wr(TX_NORMAL_DESC->TDES3, 0x1);

#ifdef DWC_ETH_QOS_ENABLE_TX_DESC_DUMP
	dump_tx_desc(pdata, original_start_index, (tx_desc_data->cur_tx - 1),
			1, qInx);
#endif

#ifdef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT
	/* updating descriptor tail pointer for DMA Transmit under two conditions,
	 * 1. if burst number of packets are present in descriptor list
	 * 2. MAC has no activity on Tx fifo
	 * */
	if ((pdata->mac_enable_count >= pdata->drop_tx_pktburstcnt)
		&& (1 == update_tail)) {
		pdata->mac_enable_count -= pdata->drop_tx_pktburstcnt;
		/* issue a poll command to Tx DMA by writing address
		 * of next immediate free descriptor */
		last_index = GET_CURRENT_XFER_LAST_DESC_INDEX(qInx, start_index, 1);
		DMA_TDTP_TPDR_RgWr(qInx,
				GET_TX_DESC_DMA_ADDR(qInx, last_index));
	}
#else
	/* issue a poll command to Tx DMA by writing address
	 * of next immediate free descriptor */
	last_index = GET_CURRENT_XFER_LAST_DESC_INDEX(qInx, start_index, 1);
	DMA_TDTP_TPDR_RgWr(qInx,
			GET_TX_DESC_DMA_ADDR(qInx, last_index));
#endif

	if (pdata->eee_enabled) {
		/* restart EEE timer */
		mod_timer(&pdata->eee_ctrl_timer,
			DWC_ETH_QOS_LPI_TIMER(DWC_ETH_QOS_DEFAULT_LPI_TIMER));
	}
  
	DBGPR("<--pre_transmit\n");
}

/*!
* \brief This sequence is used to read data from device,
* it checks whether data is good or bad and updates the errors appropriately
* \param[in] pdata
*/

static void device_read(struct DWC_ETH_QOS_prv_data *pdata, UINT qInx)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data =
	    GET_RX_WRAPPER_DESC(qInx);
	struct s_RX_NORMAL_DESC *RX_NORMAL_DESC =
	    GET_RX_DESC_PTR(qInx, rx_desc_data->cur_rx);
	UINT varOWN;
	UINT varES;
	struct DWC_ETH_QOS_rx_buffer *buffer =
	    GET_RX_BUF_PTR(qInx, rx_desc_data->cur_rx);
	UINT varRS1V;
	UINT varIPPE;
	UINT varIPCB;
	UINT varIPHE;
	struct s_rx_pkt_features *rx_pkt_features = GET_RX_PKT_FEATURES_PTR;
	UINT varRS0V;
	UINT varLT;
	UINT varRDES0;
	UINT varOE;
	struct s_rx_error_counters *rx_error_counters =
	    GET_RX_ERROR_COUNTERS_PTR;
	UINT varCE;
	UINT varRE;
	UINT varLD;

	DBGPR("-->device_read: cur_rx = %d\n", rx_desc_data->cur_rx);

	/* check for data availability */
	RX_NORMAL_DESC_RDES3_OWN_Mlf_Rd(RX_NORMAL_DESC->RDES3, varOWN);
	if (varOWN == 0) {
		/* check whether it is good packet or bad packet */
		RX_NORMAL_DESC_RDES3_ES_Mlf_Rd(RX_NORMAL_DESC->RDES3, varES);
		RX_NORMAL_DESC_RDES3_LD_Mlf_Rd(RX_NORMAL_DESC->RDES3, varLD);
#ifdef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT_HALFDUPLEX
		/* Synopsys testing and debugging purposes only */
		if (varES == 1 && varLD == 1) {
			varES = 0;
			DBGPR("Forwarding error packets as good packets to stack\n");
		}
#endif
		if ((varES == 0) && (varLD == 1)) {
			/* get the packet length */
			RX_NORMAL_DESC_RDES3_FL_Mlf_Rd(RX_NORMAL_DESC->RDES3, buffer->len);
			RX_NORMAL_DESC_RDES3_RS1V_Mlf_Rd(RX_NORMAL_DESC->RDES3, varRS1V);
			if (varRS1V == 0x1) {
				/* check whether device has done csum correctly or not */
				RX_NORMAL_DESC_RDES1_IPPE_Mlf_Rd(RX_NORMAL_DESC->RDES1, varIPPE);
				RX_NORMAL_DESC_RDES1_IPCB_Mlf_Rd(RX_NORMAL_DESC->RDES1, varIPCB);
				RX_NORMAL_DESC_RDES1_IPHE_Mlf_Rd(RX_NORMAL_DESC->RDES1, varIPHE);
				if ((varIPPE == 0) && (varIPCB == 0) && (varIPHE == 0)) {
					/* IPC Checksum done */
					RX_PKT_FEATURES_PKT_ATTRIBUTES_CSUM_DONE_Mlf_Wr(
						rx_pkt_features->pkt_attributes, 0x1);
				}
			}
#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
			RX_NORMAL_DESC_RDES3_RS0V_Mlf_Rd(RX_NORMAL_DESC->RDES3,
							 varRS0V);
			if (varRS0V == 0x1) {
				/*  device received frame with VLAN Tag or double VLAN Tag ? */
				RX_NORMAL_DESC_RDES3_LT_Mlf_Rd(RX_NORMAL_DESC->RDES3, varLT);
				if ((varLT == 0x4) || (varLT == 0x5)) {
					RX_PKT_FEATURES_PKT_ATTRIBUTES_VLAN_PKT_Mlf_Wr(
						rx_pkt_features->pkt_attributes, 0x1);
					/* get the VLAN Tag */
					RX_NORMAL_DESC_RDES0_Ml_Rd(RX_NORMAL_DESC->RDES0, varRDES0);
					RX_PKT_FEATURES_VLAN_TAG_VT_Mlf_Wr(rx_pkt_features->vlan_tag,
						(varRDES0 & 0xffff));
				}
			}
#endif
		} else {
#ifdef DWC_ETH_QOS_ENABLE_RX_DESC_DUMP
			dump_rx_desc(qInx, RX_NORMAL_DESC, rx_desc_data->cur_rx);
#endif
			/* not a good packet, hence check for appropriate errors. */
			RX_NORMAL_DESC_RDES3_OE_Mlf_Rd(RX_NORMAL_DESC->RDES3, varOE);
			if (varOE == 1) {
				RX_ERROR_COUNTERS_RX_ERRORS_OVERRUN_ERROR_Mlf_Wr(rx_error_counters->rx_errors, 1);
			}
			RX_NORMAL_DESC_RDES3_CE_Mlf_Rd(RX_NORMAL_DESC->RDES3, varCE);
			if (varCE == 1) {
				RX_ERROR_COUNTERS_RX_ERRORS_CRC_ERROR_Mlf_Wr(rx_error_counters->rx_errors, 1);
			}
			RX_NORMAL_DESC_RDES3_RE_Mlf_Rd(RX_NORMAL_DESC->RDES3, varRE);
			if (varRE == 1) {
				RX_ERROR_COUNTERS_RX_ERRORS_FRAME_ERROR_Mlf_Wr(rx_error_counters->rx_errors, 1);
			}
			RX_NORMAL_DESC_RDES3_LD_Mlf_Rd(RX_NORMAL_DESC->RDES3, varLD);
			if (varRE == 0) {
				RX_ERROR_COUNTERS_RX_ERRORS_OVERRUN_ERROR_Mlf_Wr(rx_error_counters->rx_errors, 1);
			}
		}
	}

	DBGPR("<--device_read: cur_rx = %d\n", rx_desc_data->cur_rx);
}

static void update_rx_tail_ptr(unsigned int qInx, unsigned int dma_addr)
{
	DMA_RDTP_RPDR_RgWr(qInx, dma_addr);
}

/*!
* \brief This sequence is used to check whether CTXT bit is
* set or not returns 1 if CTXT is set else returns zero
* \param[in] txdesc
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT get_tx_descriptor_ctxt(t_TX_NORMAL_DESC *txdesc)
{
	ULONG varCTXT;

	/* check TDES3.CTXT bit */
	TX_NORMAL_DESC_TDES3_CTXT_Mlf_Rd(txdesc->TDES3, varCTXT);
	if (varCTXT == 1) {
		return 1;
	} else {
		return 0;
	}
}

/*!
* \brief This sequence is used to check whether LD bit is set or not
* returns 1 if LD is set else returns zero
* \param[in] txdesc
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static INT get_tx_descriptor_last(t_TX_NORMAL_DESC *txdesc)
{
	ULONG varLD;

	/* check TDES3.LD bit */
	TX_NORMAL_DESC_TDES3_LD_Mlf_Rd(txdesc->TDES3, varLD);
	if (varLD == 1) {
		return 1;
	} else {
		return 0;
	}
}


/*!
* \brief Exit routine
* \details Exit function that unregisters the device, deallocates buffers,
* unbinds the driver from controlling the device etc.
*
* \return Returns successful execution of the routine
* \retval Y_SUCCESS Function executed successfully
*/

static INT DWC_ETH_QOS_yexit(void)
{
	ULONG retryCount = 1000;
	ULONG vy_count;
	volatile ULONG varDMA_BMR;

	DBGPR("-->DWC_ETH_QOS_yexit\n");

	/*issue a software reset */
	DMA_BMR_SWR_UdfWr(0x1);
	/*DELAY IMPLEMENTATION USING udelay() */
	udelay(10);

	/*Poll Until Poll Condition */
	vy_count = 0;
	while (1) {
		if (vy_count > retryCount) {
			return -Y_FAILURE;
		} else {
			vy_count++;
			mdelay(1);
		}
		DMA_BMR_RgRd(varDMA_BMR);
		if (GET_VALUE(varDMA_BMR, DMA_BMR_SWR_LPOS, DMA_BMR_SWR_HPOS) == 0) {
			break;
		}
	}

	DBGPR("<--DWC_ETH_QOS_yexit\n");

	return Y_SUCCESS;
}


/*!
* \details This API will calculate per queue FIFO size.
*
* \param[in] fifo_size - total fifo size in h/w register
* \param[in] queue_count - total queue count
*
* \return returns integer
* \retval - fifo size per queue.
*/
static UINT calculate_per_queue_fifo(ULONG fifo_size, UCHAR queue_count)
{
	ULONG q_fifo_size = 0;	/* calculated fifo size per queue */
	ULONG p_fifo = eDWC_ETH_QOS_256; /* per queue fifo size programmable value */

	/* calculate Tx/Rx fifo share per queue */
	switch (fifo_size) {
	case 0:
		q_fifo_size = FIFO_SIZE_B(128);
		break;
	case 1:
		q_fifo_size = FIFO_SIZE_B(256);
		break;
	case 2:
		q_fifo_size = FIFO_SIZE_B(512);
		break;
	case 3:
		q_fifo_size = FIFO_SIZE_KB(1);
		break;
	case 4:
		q_fifo_size = FIFO_SIZE_KB(2);
		break;
	case 5:
		q_fifo_size = FIFO_SIZE_KB(4);
		break;
	case 6:
		q_fifo_size = FIFO_SIZE_KB(8);
		break;
	case 7:
		q_fifo_size = FIFO_SIZE_KB(16);
		break;
	case 8:
		q_fifo_size = FIFO_SIZE_KB(32);
		break;
	case 9:
		q_fifo_size = FIFO_SIZE_KB(64);
		break;
	case 10:
		q_fifo_size = FIFO_SIZE_KB(128);
		break;
	case 11:
		q_fifo_size = FIFO_SIZE_KB(256);
		break;
	}

	q_fifo_size = q_fifo_size/queue_count;

	if (q_fifo_size >= FIFO_SIZE_KB(32)) {
		p_fifo = eDWC_ETH_QOS_32k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(16)) {
		p_fifo = eDWC_ETH_QOS_16k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(8)) {
		p_fifo = eDWC_ETH_QOS_8k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(4)) {
		p_fifo = eDWC_ETH_QOS_4k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(2)) {
		p_fifo = eDWC_ETH_QOS_2k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(1)) {
		p_fifo = eDWC_ETH_QOS_1k;
	} else if (q_fifo_size >= FIFO_SIZE_B(512)) {
		p_fifo = eDWC_ETH_QOS_512;
	} else if (q_fifo_size >= FIFO_SIZE_B(256)) {
		p_fifo = eDWC_ETH_QOS_256;
	}

	return p_fifo;
}

static INT configure_mtl_queue(UINT qInx, struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_tx_queue *queue_data = GET_TX_QUEUE_PTR(qInx);
	ULONG retryCount = 1000;
	ULONG vy_count;
	volatile ULONG varMTL_QTOMR;
	UINT p_rx_fifo = eDWC_ETH_QOS_256, p_tx_fifo = eDWC_ETH_QOS_256;

	DBGPR("-->configure_mtl_queue\n");

	/*Flush Tx Queue */
	MTL_QTOMR_FTQ_UdfWr(qInx, 0x1);

	/*Poll Until Poll Condition */
	vy_count = 0;
	while (1) {
		if (vy_count > retryCount) {
			return -Y_FAILURE;
		} else {
			vy_count++;
			mdelay(1);
		}
		MTL_QTOMR_RgRd(qInx, varMTL_QTOMR);
		if (GET_VALUE(varMTL_QTOMR, MTL_QTOMR_FTQ_LPOS, MTL_QTOMR_FTQ_HPOS)
				== 0) {
			break;
		}
	}


	/*Enable Store and Forward mode for TX */
	MTL_QTOMR_TSF_UdfWr(qInx, 0x1);
	/* Program Tx operating mode */
	MTL_QTOMR_TXQEN_UdfWr(qInx, queue_data->q_op_mode);
	/* Transmit Queue weight */
	MTL_QW_ISCQW_UdfWr(qInx, (0x10 + qInx));

	MTL_QROMR_FEP_UdfWr(qInx, 0x1);

	/* Configure for Jumbo frame in MTL */
	if (pdata->dev->mtu > DWC_ETH_QOS_ETH_FRAME_LEN) {
		/* Disable RX Store and Forward mode */
		MTL_QROMR_RSF_UdfWr(qInx, 0x0);
		printk(KERN_ALERT "RX is configured in threshold mode and threshold = 64Byte\n");
	}

	p_rx_fifo = calculate_per_queue_fifo(pdata->hw_feat.rx_fifo_size, DWC_ETH_QOS_RX_QUEUE_CNT);
	p_tx_fifo = calculate_per_queue_fifo(pdata->hw_feat.tx_fifo_size, DWC_ETH_QOS_TX_QUEUE_CNT);

	/* Transmit/Receive queue fifo size programmed */
	MTL_QROMR_RQS_UdfWr(qInx, p_rx_fifo);
	MTL_QTOMR_TQS_UdfWr(qInx, p_tx_fifo);
	printk(KERN_ALERT "Queue%d Tx fifo size %d, Rx fifo size %d\n",
			qInx, ((p_tx_fifo + 1) * 256), ((p_rx_fifo + 1) * 256));

	/* flow control will be used only if
	 * each channel gets 8KB or more fifo */
	if (p_rx_fifo >= eDWC_ETH_QOS_4k) {
		/* Enable Rx FLOW CTRL in MTL and MAC
			 Programming is valid only if Rx fifo size is greater than
			 or equal to 8k */
		if ((pdata->flow_ctrl & DWC_ETH_QOS_FLOW_CTRL_TX) ==
			DWC_ETH_QOS_FLOW_CTRL_TX) {

			MTL_QROMR_EHFC_UdfWr(qInx, 0x1);

#ifdef DWC_ETH_QOS_VER_4_0
			if (p_rx_fifo == eDWC_ETH_QOS_4k) {
				/* This violates the above formula because of FIFO size limit
				 * therefore overflow may occur inspite of this
				 * */
				MTL_QROMR_RFD_UdfWr(qInx, 0x2);
				MTL_QROMR_RFA_UdfWr(qInx, 0x1);
			}
			else if (p_rx_fifo == eDWC_ETH_QOS_8k) {
				MTL_QROMR_RFD_UdfWr(qInx, 0x4);
				MTL_QROMR_RFA_UdfWr(qInx, 0x2);
			}
			else if (p_rx_fifo == eDWC_ETH_QOS_16k) {
				MTL_QROMR_RFD_UdfWr(qInx, 0x5);
				MTL_QROMR_RFA_UdfWr(qInx, 0x2);
			}
			else if (p_rx_fifo == eDWC_ETH_QOS_32k) {
				MTL_QROMR_RFD_UdfWr(qInx, 0x7);
				MTL_QROMR_RFA_UdfWr(qInx, 0x2);
			}
#else
			/* Set Threshold for Activating Flow Contol space for min 2 frames
			 * ie, (1500 * 1) = 1500 bytes
			 *
			 * Set Threshold for Deactivating Flow Contol for space of
			 * min 1 frame (frame size 1500bytes) in receive fifo */
			if (p_rx_fifo == eDWC_ETH_QOS_4k) {
				/* This violates the above formula because of FIFO size limit
				 * therefore overflow may occur inspite of this
				 * */
				MTL_QROMR_RFD_UdfWr(qInx, 0x3); //Full - 3K
				MTL_QROMR_RFA_UdfWr(qInx, 0x1); //Full - 1.5K
			}
			else if (p_rx_fifo == eDWC_ETH_QOS_8k) {
				MTL_QROMR_RFD_UdfWr(qInx, 0x6); //Full - 4K
				MTL_QROMR_RFA_UdfWr(qInx, 0xA); //Full - 6K
			}
			else if (p_rx_fifo == eDWC_ETH_QOS_16k) {
				MTL_QROMR_RFD_UdfWr(qInx, 0x6); //Full - 4K
				MTL_QROMR_RFA_UdfWr(qInx, 0x12); //Full - 10K
			}
			else if (p_rx_fifo == eDWC_ETH_QOS_32k) {
				MTL_QROMR_RFD_UdfWr(qInx, 0x6); //Full - 4K
				MTL_QROMR_RFA_UdfWr(qInx, 0x1E); //Full - 16K
			}

#endif
		}
	}

	DBGPR("<--configure_mtl_queue\n");

	return Y_SUCCESS;
}


static INT configure_dma_channel(UINT qInx,
			struct DWC_ETH_QOS_prv_data *pdata)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data =
		GET_RX_WRAPPER_DESC(qInx);

	DBGPR("-->configure_dma_channel\n");

	/*Enable OSF mode */
	DMA_TCR_OSP_UdfWr(qInx, 0x1);

	/*Select Rx Buffer size = 2048bytes */
	switch (pdata->rx_buffer_len) {
	case 16384:
		DMA_RCR_RBSZ_UdfWr(qInx, 16384);
		break;
	case 8192:
		DMA_RCR_RBSZ_UdfWr(qInx, 8192);
		break;
	case 4096:
		DMA_RCR_RBSZ_UdfWr(qInx, 4096);
		break;
	default:		/* default is 2K */
		DMA_RCR_RBSZ_UdfWr(qInx, 2048);
		break;
	}
	/* program RX watchdog timer */
	if (rx_desc_data->use_riwt) {
		DMA_RIWTR_RWT_UdfWr(qInx, rx_desc_data->rx_riwt);
	}
	else {
		DMA_RIWTR_RWT_UdfWr(qInx, 0);
	}
	printk(KERN_ALERT "%s Rx watchdog timer\n",
		(rx_desc_data->use_riwt ? "Enabled" : "Disabled"));

	enable_dma_interrupts(qInx);
	/* set PBLx8 */
	DMA_CR_PBLx8_UdfWr(qInx, 0x1);
	/* set TX PBL = 256 */
	DMA_TCR_PBL_UdfWr(qInx, 32);
	/* set RX PBL = 256 */
	DMA_RCR_PBL_UdfWr(qInx, 32);

    /* To get Best Performance */
    DMA_SBUS_BLEN16_UdfWr(1);
    DMA_SBUS_BLEN8_UdfWr(1);
    DMA_SBUS_BLEN4_UdfWr(1);
    DMA_SBUS_RD_OSR_LMT_UdfWr(2);

	/* enable TSO if HW supports */
	if (pdata->hw_feat.tso_en)
		DMA_TCR_TSE_UdfWr(qInx, 0x1);
	printk(KERN_ALERT "%s TSO\n",
		(pdata->hw_feat.tso_en ? "Enabled" : "Disabled"));

	/* program split header mode */
	DMA_CR_SPH_UdfWr(qInx, pdata->rx_split_hdr);
	printk(KERN_ALERT "%s Rx Split header mode\n",
		(pdata->rx_split_hdr ? "Enabled" : "Disabled"));

	/*
	 * For PG don't start TX DMA now.
	 */
#ifndef DWC_ETH_QOS_CONFIG_PGTEST
	/* start TX DMA */
	DMA_TCR_ST_UdfWr(qInx, 0x1);
#endif
	/* start RX DMA */
	DMA_RCR_ST_UdfWr(qInx, 0x1);

	DBGPR("<--configure_dma_channel\n");

	return Y_SUCCESS;
}




/*!
* \brief This sequence is used to enable MAC interrupts
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

static int enable_mac_interrupts(void)
{
  unsigned long varmac_imr;

  /* Enable following interrupts */
  /* RGSMIIIM - RGMII/SMII interrupt Enable */
  /* PCSLCHGIM -  PCS Link Status Interrupt Enable */
  /* PCSANCIM - PCS AN Completion Interrupt Enable */
  /* PMTIM - PMT Interrupt Enable */
  /* LPIIM - LPI Interrupt Enable */
  MAC_IMR_RgRd(varmac_imr);
  varmac_imr = varmac_imr & (unsigned long)(0x1008);
  varmac_imr = varmac_imr | ((0x1) << 0) | ((0x1) << 1) | ((0x1) << 2) |
                ((0x1) << 4) | ((0x1) << 5);
  MAC_IMR_RgWr(varmac_imr);


  return Y_SUCCESS;
}



static INT configure_mac(struct DWC_ETH_QOS_prv_data *pdata)
{
	ULONG varMAC_MCR;
	UINT qInx;

	DBGPR("-->configure_mac\n");

	for (qInx = 0; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++) {
		MAC_RQC0R_RXQEN_UdfWr(qInx, 0x2);
	}

	/* Set Tx flow control parameters */
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		/* set Pause Time */
		MAC_QTFCR_PT_UdfWr(qInx, 0xffff);
		/* Assign priority for RX flow control */
		/* Assign priority for TX flow control */
		switch(qInx) {
		case 0:
			MAC_TQPM0R_PSTQ0_UdfWr(0);
			MAC_RQC2R_PSRQ0_UdfWr(0x1 << qInx);
			break;
		case 1:
			MAC_TQPM0R_PSTQ1_UdfWr(1);
			MAC_RQC2R_PSRQ1_UdfWr(0x1 << qInx);
			break;
		case 2:
			MAC_TQPM0R_PSTQ2_UdfWr(2);
			MAC_RQC2R_PSRQ2_UdfWr(0x1 << qInx);
			break;
		case 3:
			MAC_TQPM0R_PSTQ3_UdfWr(3);
			MAC_RQC2R_PSRQ3_UdfWr(0x1 << qInx);
			break;
		case 4:
			MAC_TQPM1R_PSTQ4_UdfWr(4);
			MAC_RQC3R_PSRQ4_UdfWr(0x1 << qInx);
			break;
		case 5:
			MAC_TQPM1R_PSTQ5_UdfWr(5);
			MAC_RQC3R_PSRQ5_UdfWr(0x1 << qInx);
			break;
		case 6:
			MAC_TQPM1R_PSTQ6_UdfWr(6);
			MAC_RQC3R_PSRQ6_UdfWr(0x1 << qInx);
			break;
		case 7:
			MAC_TQPM1R_PSTQ7_UdfWr(7);
			MAC_RQC3R_PSRQ7_UdfWr(0x1 << qInx);
			break;
		}

		if ((pdata->flow_ctrl & DWC_ETH_QOS_FLOW_CTRL_TX) == DWC_ETH_QOS_FLOW_CTRL_TX)
			enable_tx_flow_ctrl(qInx);
		else
			disable_tx_flow_ctrl(qInx);
	}

	/* Set Rx flow control parameters */
	if ((pdata->flow_ctrl & DWC_ETH_QOS_FLOW_CTRL_RX) == DWC_ETH_QOS_FLOW_CTRL_RX)
		enable_rx_flow_ctrl();
	else
		disable_rx_flow_ctrl();

	/* Configure for Jumbo frame in MAC */
	if (pdata->dev->mtu > DWC_ETH_QOS_ETH_FRAME_LEN) {
		if (pdata->dev->mtu < DWC_ETH_QOS_MAX_GPSL) {
			MAC_MCR_JE_UdfWr(0x1);
			MAC_MCR_WD_UdfWr(0x0);
			MAC_MCR_GPSLCE_UdfWr(0x0);
			MAC_MCR_JD_UdfWr(0x0);
		} else {
			MAC_MCR_JE_UdfWr(0x0);
			MAC_MCR_WD_UdfWr(0x1);
			MAC_MCR_GPSLCE_UdfWr(0x1);
			MAC_MECR_GPSL_UdfWr(DWC_ETH_QOS_MAX_SUPPORTED_MTU);
			MAC_MCR_JD_UdfWr(0x1);
			printk(KERN_ALERT "Configured Gaint Packet Size Limit to %d\n",
				DWC_ETH_QOS_MAX_SUPPORTED_MTU);
		}
		printk(KERN_ALERT "Enabled JUMBO pkt\n");
	} else {
		MAC_MCR_JE_UdfWr(0x0);
		MAC_MCR_WD_UdfWr(0x0);
		MAC_MCR_GPSLCE_UdfWr(0x0);
		MAC_MCR_JD_UdfWr(0x0);
		printk(KERN_ALERT "Disabled JUMBO pkt\n");
	}

	/* update the MAC address */
	MAC_MA0HR_RgWr(((pdata->dev->dev_addr[5] << 8) |
			(pdata->dev->dev_addr[4])));
	MAC_MA0LR_RgWr(((pdata->dev->dev_addr[3] << 24) |
			(pdata->dev->dev_addr[2] << 16) |
			(pdata->dev->dev_addr[1] << 8) |
			(pdata->dev->dev_addr[0])));

	/*Enable MAC Transmit process */
	/*Enable MAC Receive process */
	/*Enable padding - disabled */
	/*Enable CRC stripping - disabled */
	MAC_MCR_RgRd(varMAC_MCR);
	varMAC_MCR = varMAC_MCR & (ULONG) (0xffcfff7c);
	varMAC_MCR = varMAC_MCR | ((0x1) << 0) | ((0x1) << 20) | ((0x1) << 21);
#ifndef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT
	varMAC_MCR |= ((0x1) << 1);
#endif
	MAC_MCR_RgWr(varMAC_MCR);

	if (pdata->hw_feat.rx_coe_sel &&
	     ((pdata->dev_state & NETIF_F_RXCSUM) == NETIF_F_RXCSUM))
		MAC_MCR_IPC_UdfWr(0x1);

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	configure_mac_for_vlan_pkt();
	if (pdata->hw_feat.vlan_hash_en)
			config_vlan_filtering(1, 1, 0);
#endif

	if (pdata->hw_feat.mmc_sel) {
		/* disable all MMC intterrupt as MMC are managed in SW and
		 * registers are cleared on each READ eventually
		 * */
		disable_mmc_interrupts();
		config_mmc_counters();
	}

	enable_mac_interrupts();

	DBGPR("<--configure_mac\n");

	return Y_SUCCESS;
}

/*!
* \brief Initialises device registers.
* \details This function initialises device registers.
*
* \return none
*/

static INT DWC_ETH_QOS_yinit(struct DWC_ETH_QOS_prv_data *pdata)
{
	UINT qInx;

	DBGPR("-->DWC_ETH_QOS_yinit\n");

	/* reset mmc counters */
	MMC_CNTRL_RgWr(0x1);

	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		configure_mtl_queue(qInx, pdata);
	}
	//Mapping MTL Rx queue and DMA Rx channel.
	MTL_RQDCM0R_RgWr(0x3020100);
	MTL_RQDCM1R_RgWr(0x7060504);
#ifdef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT
	/* enable tx drop status */
	MTL_OMR_DTXSTS_UdfWr(0x1);
#endif

	configure_mac(pdata);

	/* Setting INCRx */
	DMA_SBUS_RgWr(0x0);
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		configure_dma_channel(qInx, pdata);
	}

#ifdef DWC_ETH_QOS_CERTIFICATION_PKTBURSTCNT_HALFDUPLEX
	MTL_Q0ROMR_FEP_UdfWr(0x1);
	MAC_MPFR_RA_UdfWr(0x1);
	MAC_MCR_BE_UdfWr(0x1);
#endif

	DBGPR("<--DWC_ETH_QOS_yinit\n");

	return Y_SUCCESS;
}

#ifdef DWC_ETH_QOS_CONFIG_PGTEST

/*!
* \brief This sequence is used to initialize the tx descriptors.
* \param[in] pdata
*/

static void tx_descriptor_init_pg(struct DWC_ETH_QOS_prv_data *pdata,
					UINT qInx)
{
	struct DWC_ETH_QOS_tx_wrapper_descriptor *tx_desc_data =
		GET_TX_WRAPPER_DESC(qInx);
	struct s_TX_NORMAL_DESC *TX_NORMAL_DESC =
		GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
	struct DWC_ETH_QOS_tx_buffer *buffer =
		GET_TX_BUF_PTR(qInx, tx_desc_data->cur_tx);
	INT i;
	INT start_index = tx_desc_data->cur_tx;

	DBGPR("-->tx_descriptor_init_pg\n");

	/* initialze all descriptors. */

	for (i = 0; i < TX_DESC_CNT; i++) {
		/* update buffer 1 address pointer to zero */
		TX_NORMAL_DESC_TDES0_Ml_Wr(TX_NORMAL_DESC->TDES0, 0);
		/* update buffer 2 address pointer to zero */
		TX_NORMAL_DESC_TDES1_Ml_Wr(TX_NORMAL_DESC->TDES1, 0);
		/* set all other control bits (IC, TTSE, B2L & B1L) to zero */
		TX_NORMAL_DESC_TDES2_Ml_Wr(TX_NORMAL_DESC->TDES2, 0);
		/* set all other control bits (OWN, CTXT, FD, LD, CPC, CIC etc) to zero */
		TX_NORMAL_DESC_TDES3_Ml_Wr(TX_NORMAL_DESC->TDES3, 0);

		INCR_TX_DESC_INDEX(tx_desc_data->cur_tx, 1);
		TX_NORMAL_DESC = GET_TX_DESC_PTR(qInx, tx_desc_data->cur_tx);
		buffer = GET_TX_BUF_PTR(qInx, tx_desc_data->cur_tx);
	}
	/* update the total no of Tx descriptors count */
	DMA_TDRLR_RgWr(qInx, (TX_DESC_CNT - 1));
	/* update the starting address of desc chain/ring */
	DMA_TDLAR_RgWr(qInx, GET_TX_DESC_DMA_ADDR(qInx, start_index));

	DBGPR("<--tx_descriptor_init_pg\n");
}

/*!
* \brief This sequence is used to initialize the rx descriptors.
* \param[in] pdata
*/

static void rx_descriptor_init_pg(struct DWC_ETH_QOS_prv_data *pdata, UINT qInx)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data =
	    GET_RX_WRAPPER_DESC(qInx);
	struct DWC_ETH_QOS_rx_buffer *buffer =
	    GET_RX_BUF_PTR(qInx, rx_desc_data->cur_rx);
	struct s_RX_NORMAL_DESC *RX_NORMAL_DESC =
	    GET_RX_DESC_PTR(qInx, rx_desc_data->cur_rx);
	INT i;
	INT start_index = rx_desc_data->cur_rx;
	INT last_index;

	DBGPR("-->rx_descriptor_init_pg\n");

	/* initialize all desc */

	for (i = 0; i < RX_DESC_CNT; i++) {
		memset(RX_NORMAL_DESC, 0, sizeof(struct s_RX_NORMAL_DESC));
		/* update buffer 1 address pointer */
		RX_NORMAL_DESC_RDES0_Ml_Wr(RX_NORMAL_DESC->RDES0, buffer->dma);
		/* set to zero  */
		RX_NORMAL_DESC_RDES1_Ml_Wr(RX_NORMAL_DESC->RDES1, 0);

		/* set buffer 2 address pointer to zero */
		RX_NORMAL_DESC_RDES2_Ml_Wr(RX_NORMAL_DESC->RDES2, 0);
		/* set control bits - OWN, INTE and BUF1V */
		RX_NORMAL_DESC_RDES3_Ml_Wr(RX_NORMAL_DESC->RDES3, (0xc1000000));

		INCR_RX_DESC_INDEX(rx_desc_data->cur_rx, 1);
		RX_NORMAL_DESC =
			GET_RX_DESC_PTR(qInx, rx_desc_data->cur_rx);
		buffer = GET_RX_BUF_PTR(qInx, rx_desc_data->cur_rx);
	}
	/* update the total no of Rx descriptors count */
	DMA_RDRLR_RgWr(qInx, (RX_DESC_CNT - 1));
	/* update the Rx Descriptor Tail Pointer */
	last_index = GET_CURRENT_RCVD_LAST_DESC_INDEX(start_index, 0);
	DMA_RDTP_RPDR_RgWr(qInx, GET_RX_DESC_DMA_ADDR(qInx, last_index));
	/* update the starting address of desc chain/ring */
	DMA_RDLAR_RgWr(qInx, GET_RX_DESC_DMA_ADDR(qInx, start_index));

	DBGPR("<--rx_descriptor_init_pg\n");
}

#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */


/*!
* \brief API to initialize the function pointers.
*
* \details This function is called in probe to initialize all the
* function pointers which are used in other functions to capture
* the different device features.
*
* \param[in] hw_if - pointer to hw_if_struct structure.
*
* \return void.
*/

void DWC_ETH_QOS_init_function_ptrs_dev(struct hw_if_struct *hw_if)
{

	DBGPR("-->DWC_ETH_QOS_init_function_ptrs_dev\n");

	hw_if->tx_complete = tx_complete;
	hw_if->tx_window_error = NULL;
	hw_if->tx_aborted_error = tx_aborted_error;
	hw_if->tx_carrier_lost_error = tx_carrier_lost_error;
	hw_if->tx_fifo_underrun = tx_fifo_underrun;
	hw_if->tx_get_collision_count = NULL;
	hw_if->tx_handle_aborted_error = NULL;
	hw_if->tx_update_fifo_threshold = NULL;
	hw_if->tx_config_threshold = NULL;

	hw_if->set_promiscuous_mode = set_promiscuous_mode;
	hw_if->set_all_multicast_mode = set_all_multicast_mode;
	hw_if->set_multicast_list_mode = set_multicast_list_mode;
	hw_if->set_unicast_mode = set_unicast_mode;

	hw_if->enable_rx_csum = enable_rx_csum;
	hw_if->disable_rx_csum = disable_rx_csum;
	hw_if->get_rx_csum_status = get_rx_csum_status;

	hw_if->write_phy_regs = write_phy_regs;
	hw_if->read_phy_regs = read_phy_regs;
	hw_if->set_full_duplex = set_full_duplex;
	hw_if->set_half_duplex = set_half_duplex;
	hw_if->set_mii_speed_100 = set_mii_speed_100;
	hw_if->set_mii_speed_10 = set_mii_speed_10;
	hw_if->set_gmii_speed = set_gmii_speed;
	/* for PMT */
	hw_if->start_dma_rx = start_dma_rx;
	hw_if->stop_dma_rx = stop_dma_rx;
	hw_if->start_dma_tx = start_dma_tx;
	hw_if->stop_dma_tx = stop_dma_tx;
	hw_if->start_mac_tx_rx = start_mac_tx_rx;
	hw_if->stop_mac_tx_rx = stop_mac_tx_rx;

	hw_if->pre_xmit = pre_transmit;
	hw_if->dev_read = device_read;
	hw_if->init = DWC_ETH_QOS_yinit;
	hw_if->exit = DWC_ETH_QOS_yexit;
	/* Descriptor related Sequences have to be initialized here */
	hw_if->tx_desc_init = tx_descriptor_init;
	hw_if->rx_desc_init = rx_descriptor_init;
	hw_if->rx_desc_reset = rx_descriptor_reset;
	hw_if->tx_desc_reset = tx_descriptor_reset;
	hw_if->get_tx_desc_ls = get_tx_descriptor_last;
	hw_if->get_tx_desc_ctxt = get_tx_descriptor_ctxt;
	hw_if->update_rx_tail_ptr = update_rx_tail_ptr;

	/* for FLOW ctrl */
	hw_if->enable_rx_flow_ctrl = enable_rx_flow_ctrl;
	hw_if->disable_rx_flow_ctrl = disable_rx_flow_ctrl;
	hw_if->enable_tx_flow_ctrl = enable_tx_flow_ctrl;
	hw_if->disable_tx_flow_ctrl = disable_tx_flow_ctrl;

	/* for PMT operation */
	hw_if->enable_magic_pmt = enable_magic_pmt_operation;
	hw_if->disable_magic_pmt = disable_magic_pmt_operation;
	hw_if->enable_remote_pmt = enable_remote_pmt_operation;
	hw_if->disable_remote_pmt = disable_remote_pmt_operation;
	hw_if->configure_rwk_filter = configure_rwk_filter_registers;

    /* for TX vlan control */
    hw_if->enable_vlan_reg_control = configure_reg_vlan_control;
    hw_if->enable_vlan_desc_control = configure_desc_vlan_control;



	/* for rx vlan stripping */
	hw_if->config_rx_outer_vlan_stripping =
	    config_rx_outer_vlan_stripping;
	hw_if->config_rx_inner_vlan_stripping =
	    config_rx_inner_vlan_stripping;

	/* for sa(source address) insert/replace */
	hw_if->configure_mac_addr0_reg = configure_mac_addr0_reg;
	hw_if->configure_mac_addr1_reg = configure_mac_addr1_reg;
	hw_if->configure_sa_via_reg = configure_sa_via_reg;

	/* for RX watchdog timer */
	hw_if->config_rx_watchdog = config_rx_watchdog_timer;

	/* for RX and TX threshold config */
	hw_if->config_rx_threshold = config_rx_threshold;
	hw_if->config_tx_threshold = config_tx_threshold;

	/* for RX and TX Store and Forward Mode config */
	hw_if->config_rsf_mode = config_rsf_mode;
	hw_if->config_tsf_mode = config_tsf_mode;

	/* for TX DMA Operating on Second Frame config */
	hw_if->config_osf_mode = config_osf_mode;

	/* for INCR/INCRX config */
	hw_if->config_incr_incrx_mode = config_incr_incrx_mode;
	/* for AXI PBL config */
	hw_if->config_axi_pbl_val = config_axi_pbl_val;
	/* for AXI WORL config */
	hw_if->config_axi_worl_val = config_axi_worl_val;
	/* for AXI RORL config */
	hw_if->config_axi_rorl_val = config_axi_rorl_val;

	/* for RX and TX PBL config */
	hw_if->config_rx_pbl_val = config_rx_pbl_val;
	hw_if->get_rx_pbl_val = get_rx_pbl_val;
	hw_if->config_tx_pbl_val = config_tx_pbl_val;
	hw_if->get_tx_pbl_val = get_tx_pbl_val;
	hw_if->config_pblx8 = config_pblx8;

	hw_if->disable_rx_interrupt = disable_rx_interrupt;
	hw_if->enable_rx_interrupt = enable_rx_interrupt;

	/* for handling MMC */
	hw_if->disable_mmc_interrupts = disable_mmc_interrupts;
	hw_if->config_mmc_counters = config_mmc_counters;

	/* for handling split header */
	hw_if->config_split_header_mode = config_split_header_mode;
	hw_if->config_header_size = config_header_size;

	hw_if->set_dcb_algorithm = set_dcb_algorithm;
	hw_if->set_dcb_queue_weight = set_dcb_queue_weight;

	hw_if->set_tx_queue_operating_mode = set_tx_queue_operating_mode;
	hw_if->set_avb_algorithm = set_avb_algorithm;
	hw_if->config_credit_control = config_credit_control;
	hw_if->config_send_slope = config_send_slope;
	hw_if->config_idle_slope = config_idle_slope;
	hw_if->config_high_credit = config_high_credit;
	hw_if->config_low_credit = config_low_credit;
	hw_if->config_slot_num_check = config_slot_num_check;
	hw_if->config_advance_slot_num_check = config_advance_slot_num_check;
#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	hw_if->tx_desc_init_pg = tx_descriptor_init_pg;
	hw_if->rx_desc_init_pg = rx_descriptor_init_pg;
	hw_if->set_ch_arb_weights = set_ch_arb_weights;
	hw_if->config_slot_interrupt = config_slot_interrupt;
	hw_if->set_slot_count = set_slot_count;
	hw_if->set_tx_rx_prio_policy = set_tx_rx_prio_policy;
	hw_if->set_tx_rx_prio = set_tx_rx_prio;
	hw_if->set_tx_rx_prio_ratio = set_tx_rx_prio_ratio;
	hw_if->set_dma_tx_arb_algorithm = set_dma_tx_arb_algorithm;
	hw_if->prepare_dev_pktgen = prepare_dev_pktgen;
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */

	/* for hw time stamping */
	hw_if->config_hw_time_stamping = config_hw_time_stamping;
	hw_if->config_sub_second_increment = config_sub_second_increment;
	hw_if->init_systime = init_systime;
	hw_if->config_addend = config_addend;
	hw_if->adjust_systime = adjust_systime;
	hw_if->get_systime = get_systime;
	hw_if->get_tx_tstamp_status = get_tx_tstamp_status;
	hw_if->get_tx_tstamp = get_tx_tstamp;
	hw_if->get_tx_tstamp_status_via_reg = get_tx_tstamp_status_via_reg;
	hw_if->get_tx_tstamp_via_reg = get_tx_tstamp_via_reg;
	hw_if->rx_tstamp_available = rx_tstamp_available;
	hw_if->get_rx_tstamp_status = get_rx_tstamp_status;
	hw_if->get_rx_tstamp = get_rx_tstamp;
	hw_if->drop_tx_status_enabled = drop_tx_status_enabled;

	/* for l3 and l4 layer filtering */
	hw_if->config_l2_da_perfect_inverse_match = config_l2_da_perfect_inverse_match;
	hw_if->update_mac_addr32_127_low_high_reg = update_mac_addr32_127_low_high_reg;
	hw_if->update_mac_addr1_31_low_high_reg = update_mac_addr1_31_low_high_reg;
	hw_if->update_hash_table_reg = update_hash_table_reg;
	hw_if->config_mac_pkt_filter_reg = config_mac_pkt_filter_reg;
	hw_if->config_l3_l4_filter_enable = config_l3_l4_filter_enable;
	hw_if->config_l3_filters = config_l3_filters;
	hw_if->update_ip4_addr0 = update_ip4_addr0;
	hw_if->update_ip4_addr1 = update_ip4_addr1;
	hw_if->update_ip6_addr = update_ip6_addr;
	hw_if->config_l4_filters = config_l4_filters;
	hw_if->update_l4_sa_port_no = update_l4_sa_port_no;
	hw_if->update_l4_da_port_no = update_l4_da_port_no;

	/* for VLAN filtering */
	hw_if->get_vlan_hash_table_reg = get_vlan_hash_table_reg;
	hw_if->update_vlan_hash_table_reg = update_vlan_hash_table_reg;
	hw_if->update_vlan_id = update_vlan_id;
	hw_if->config_vlan_filtering = config_vlan_filtering;
   	hw_if->config_mac_for_vlan_pkt = configure_mac_for_vlan_pkt;
    hw_if->get_vlan_tag_comparison = get_vlan_tag_comparison;

	/* for differnet PHY interconnect */
	hw_if->control_an = control_an;
	hw_if->get_an_adv_pause_param = get_an_adv_pause_param;
	hw_if->get_an_adv_duplex_param = get_an_adv_duplex_param;
	hw_if->get_lp_an_adv_pause_param = get_lp_an_adv_pause_param;
	hw_if->get_lp_an_adv_duplex_param = get_lp_an_adv_duplex_param;

	/* for EEE */
	hw_if->set_eee_mode = set_eee_mode;
	hw_if->reset_eee_mode = reset_eee_mode;
	hw_if->set_eee_pls = set_eee_pls;
	hw_if->set_eee_timer = set_eee_timer;
	hw_if->get_lpi_status = get_lpi_status;
	hw_if->set_lpi_tx_automate = set_lpi_tx_automate;

	/* for ARP */
	hw_if->config_arp_offload = config_arp_offload;
	hw_if->update_arp_offload_ip_addr = update_arp_offload_ip_addr;

	/* for MAC loopback */
	hw_if->config_mac_loopback_mode = config_mac_loopback_mode;

	/* for PFC */
	hw_if->config_pfc = config_pfc;


    /* for MAC Double VLAN Processing config */
	hw_if->config_tx_outer_vlan = config_tx_outer_vlan;
	hw_if->config_tx_inner_vlan = config_tx_inner_vlan;
	hw_if->config_svlan = config_svlan;
	hw_if->config_dvlan = config_dvlan;
	hw_if->config_rx_outer_vlan_stripping = config_rx_outer_vlan_stripping;
	hw_if->config_rx_inner_vlan_stripping = config_rx_inner_vlan_stripping;

	/* for PTP Offloading */
	hw_if->config_ptpoffload_engine = config_ptpoffload_engine;



	DBGPR("<--DWC_ETH_QOS_init_function_ptrs_dev\n");
}
