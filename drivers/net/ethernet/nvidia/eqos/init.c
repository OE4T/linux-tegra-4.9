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


/*!@file: DWC_ETH_QOS_init.c
 * @brief: Driver functions.
 */
#include "yheader.h"
#include "init.h"
#include "yregacc.h"
#include "nvregacc.h"
#include <linux/platform_device.h>
#include <linux/of_device.h>

#define LP_SUPPORTED 1
static UCHAR dev_addr[6] = {0, 0x55, 0x7b, 0xb5, 0x7d, 0xf7};
static const struct of_device_id dwc_eth_qos_of_match[] = {
	{ .compatible = "synopsys,dwc_eth_qos" },
	{},
};
MODULE_DEVICE_TABLE(of, dwc_eth_qos_of_match);

ULONG dwc_eth_qos_base_addr;

void DWC_ETH_QOS_init_all_fptrs(struct DWC_ETH_QOS_prv_data *pdata)
{
	DWC_ETH_QOS_init_function_ptrs_dev(&pdata->hw_if);
	DWC_ETH_QOS_init_function_ptrs_desc(&pdata->desc_if);
}

/*!
* \brief POWER Interrupt Service Routine
* \details POWER Interrupt Service Routine
*
* \param[in] irq         - interrupt number for particular device
* \param[in] device_id   - pointer to device structure
* \return returns positive integer
* \retval IRQ_HANDLED
*/
irqreturn_t DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS_POWER(int irq, void *device_id)
{
	struct DWC_ETH_QOS_prv_data *pdata = (struct DWC_ETH_QOS_prv_data *)device_id;
	ULONG varMAC_ISR;
	ULONG varMAC_IMR;
	ULONG varMAC_PMTCSR;

#ifdef HWA_FPGA_ONLY
	ULONG varCLK_CTRL;
	CLK_CRTL0_RgRd(varCLK_CTRL);
	if (varCLK_CTRL & BIT(31)) {
		pr_info("power_isr: phy_intr received\n");
		return IRQ_NONE;
	} else {
#endif
		MAC_ISR_RgRd(varMAC_ISR);
		MAC_IMR_RgRd(varMAC_IMR);
		pr_info("power_isr: power_intr received, MAC_ISR =%#lx, MAC_IMR =%#lx\n",
				varMAC_ISR, varMAC_IMR);

		varMAC_ISR = (varMAC_ISR & varMAC_IMR);

		/* RemoteWake and MagicPacket events will be received by PHY supporting
		 * these features on silicon and can be used to wake up Tegra.
		 * Still let the below code be there in case we ever get this interrupt.
		 */
		if (GET_VALUE(varMAC_ISR, MAC_ISR_PMTIS_LPOS, MAC_ISR_PMTIS_HPOS) & 1) {
			pdata->xstats.pmt_irq_n++;
			MAC_PMTCSR_RgRd(varMAC_PMTCSR);
			pr_info("power_isr: PMTCSR : %#lx\n", varMAC_PMTCSR);
			if (pdata->power_down)
				DWC_ETH_QOS_powerup(pdata->dev, DWC_ETH_QOS_IOCTL_CONTEXT);
		}

		/* RxLPI exit EEE interrupts */
		if (GET_VALUE(varMAC_ISR, MAC_ISR_LPI_LPOS, MAC_ISR_LPI_HPOS) & 1) {
			pr_info("power_isr: LPI intr received\n");
			DWC_ETH_QOS_handle_eee_interrupt(pdata);
#ifdef HWA_NV_1650337
		/* FIXME: remove this once root cause of HWA_NV_1650337 is known */
		} else {
			/* We have seen power_intr flood without LPIIS set in MAC_ISR
			 * and need to still read MAC_LPI_CONTROL_STS register to
			 * get rid of interrupt storm issue.
			 */
			pr_info("power_isr: LPIIS not set in MAC_ISR but still reading MAC_LPI_CONTROL_STS\n");
			DWC_ETH_QOS_handle_eee_interrupt(pdata);
#endif
		}

		return IRQ_HANDLED;
#ifdef HWA_FPGA_ONLY
	}
#endif
}

/*!
* \brief API to initialize the device.
*
* \details This probing function gets called (during execution of
* pci_register_driver() for already existing devices or later if a
* new device gets inserted) for all PCI devices which match the ID table
* and are not "owned" by the other drivers yet. This function gets passed
* a "struct pci_dev *" for each device whose entry in the ID table matches
* the device. The probe function returns zero when the driver chooses to take
* "ownership" of the device or an error code (negative number) otherwise.
* The probe function always gets called from process context, so it can sleep.
*
* \param[in] pdev - pointer to pci_dev structure.
* \param[in] id   - pointer to table of device ID/ID's the driver is inerested.
*
* \return integer
*
* \retval 0 on success & -ve number on failure.
*/

int DWC_ETH_QOS_probe(struct platform_device *pdev)
{

	struct DWC_ETH_QOS_prv_data *pdata = NULL;
	struct net_device *ndev = NULL;
	int i, ret = 0;
	int irq, power_irq;
	int phyirq;
	struct hw_if_struct *hw_if = NULL;
	struct desc_if_struct *desc_if = NULL;
	UCHAR tx_q_count = 0, rx_q_count = 0;
	struct resource *res;
	const struct of_device_id *match;

	DBGPR("***EQOS DRIVER COMPILED ON %s AT %s***\n", __DATE__, __TIME__);

	match = of_match_device(dwc_eth_qos_of_match, &pdev->dev);
	if(!match)
		return -EINVAL;

  /* get base addr */
  res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (unlikely(res == NULL)) {
  	dev_err(&pdev->dev, "invalid resource\n");
  	return -EINVAL;
	}

	/*get IRQ*/
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	power_irq = platform_get_irq(pdev, 1);
	if (power_irq < 0)
		return power_irq;

#if defined(CONFIG_PHYS_ADDR_T_64BIT)
	DBGPR("res->start = 0x%lx \n", (unsigned long)res->start);
	DBGPR("res->end = 0x%lx \n", (unsigned long)res->end);
#else
	DBGPR("res->start = 0x%x \n", (unsigned int)res->start);
	DBGPR("res->end = 0x%x \n", (unsigned int)res->end);
#endif
	DBGPR("irq = %d \n", irq);
	DBGPR("power_irq = %d \n", power_irq);

#ifdef HWA_FPGA_ONLY
	/* PMT and PHY irqs are shared on FPGA system */
	phyirq = power_irq;
#else
	/* On silicon the phy_intr line is handled through a wake capable
	 * GPIO input. DMIC4_CLK is the GPIO input port.
	 * Refer Bug 1626763
	 */
#endif
	DBGPR("phyirq = %d\n", phyirq);

	DBGPR("==========================================================\n");
	DBGPR("Sizeof rx context desc is %lu\n", sizeof(struct s_RX_CONTEXT_DESC));
	DBGPR("Sizeof tx context desc is %lu\n", sizeof(struct s_TX_CONTEXT_DESC));
	DBGPR("Sizeof rx normal desc is %lu\n", sizeof(struct s_RX_NORMAL_DESC));
	DBGPR("Sizeof tx normal desc is %lu\n\n", sizeof(struct s_TX_NORMAL_DESC));
	DBGPR("==========================================================\n");

	/*remap base address*/
	dwc_eth_qos_base_addr = (ULONG)ioremap_nocache(res->start, (res->end - res->start) + 1);

	/* queue count */
	tx_q_count = get_tx_queue_count();
	rx_q_count = get_rx_queue_count();

	/* allocate and set up the ethernet device*/
	ndev = alloc_etherdev_mqs(sizeof(struct DWC_ETH_QOS_prv_data),
				tx_q_count, rx_q_count);
	if (ndev == NULL) {
		printk(KERN_ALERT "%s:Unable to alloc new net device\n",
		    DEV_NAME);
		ret = -ENOMEM;
		goto err_out_dev_failed;
	}

	/* Set up MAC address */
	ndev->dev_addr[0] = dev_addr[0];
	ndev->dev_addr[1] = dev_addr[1];
	ndev->dev_addr[2] = dev_addr[2];
	ndev->dev_addr[3] = dev_addr[3];
	ndev->dev_addr[4] = dev_addr[4];
	ndev->dev_addr[5] = dev_addr[5];

	ndev->base_addr = dwc_eth_qos_base_addr;
	SET_NETDEV_DEV(ndev, &pdev->dev);
	pdata = netdev_priv(ndev);
	DWC_ETH_QOS_init_all_fptrs(pdata);
	hw_if = &(pdata->hw_if);
	desc_if = &(pdata->desc_if);

	platform_set_drvdata(pdev, ndev);
	pdata->pdev = pdev;

	pdata->dev = ndev;
	pdata->tx_queue_cnt = tx_q_count;
	pdata->rx_queue_cnt = rx_q_count;

#ifdef DWC_ETH_QOS_CONFIG_DEBUGFS
	/* to give prv data to debugfs */
	DWC_ETH_QOS_get_pdata(pdata);
#endif

	/* issue software reset to device */
	if (!SIM_WORLD)
		hw_if->exit();

	ndev->irq = irq;
	pdata->power_irq = power_irq;
	pdata->phyirq = phyirq;

	if (!SIM_WORLD) {
		DWC_ETH_QOS_get_all_hw_features(pdata);
		DWC_ETH_QOS_print_all_hw_features(pdata);
	}

	ret = desc_if->alloc_queue_struct(pdata);
	if (ret < 0) {
		printk(KERN_ALERT "ERROR: Unable to alloc Tx/Rx queue\n");
		goto err_out_q_alloc_failed;
	}

	ndev->netdev_ops = DWC_ETH_QOS_get_netdev_ops();

	pdata->interface = DWC_ETH_QOS_get_phy_interface(pdata);
	/* Bypass PHYLIB for TBI, RTBI and SGMII interface */
	if (1 == pdata->hw_feat.sma_sel) {
		ret = DWC_ETH_QOS_mdio_register(ndev);
		if (ret < 0) {
			printk(KERN_ALERT "MDIO bus (id %d) registration failed\n",
			       pdata->bus_id);
#ifndef AR_XXX
			printk(KERN_ALERT "********** CAUTION:IGNORING ERRORS FROM MDIO BUS REGISTRATION FOR NOW *********\n");
#else
			goto err_out_mdio_reg;
#endif
		}
	} else {
		printk(KERN_ALERT "%s: MDIO is not present\n\n", DEV_NAME);
	}

#ifndef DWC_ETH_QOS_CONFIG_PGTEST
	/* enabling and registration of irq with magic wakeup */
	if (1 == pdata->hw_feat.mgk_sel) {
		device_set_wakeup_capable(&pdev->dev, 1);
		pdata->wolopts = WAKE_MAGIC;
		enable_irq_wake(ndev->irq);
	}

	for (i = 0; i < DWC_ETH_QOS_RX_QUEUE_CNT; i++) {
		struct DWC_ETH_QOS_rx_queue *rx_queue = GET_RX_QUEUE_PTR(i);

		netif_napi_add(ndev, &rx_queue->napi, DWC_ETH_QOS_poll_mq,
				(64 * DWC_ETH_QOS_RX_QUEUE_CNT));
	}

	ndev->ethtool_ops = (DWC_ETH_QOS_get_ethtool_ops());

	if (pdata->hw_feat.tso_en) {
		ndev->hw_features = NETIF_F_TSO;
		ndev->hw_features |= NETIF_F_SG;
		ndev->hw_features |= NETIF_F_IP_CSUM;
		ndev->hw_features |= NETIF_F_IPV6_CSUM;
		printk(KERN_ALERT "Supports TSO, SG and TX COE\n");
	}
	else if (pdata->hw_feat.tx_coe_sel) {
		ndev->hw_features = NETIF_F_IP_CSUM ;
		ndev->hw_features |= NETIF_F_IPV6_CSUM;
		printk(KERN_ALERT "Supports TX COE\n");
	}

	if (pdata->hw_feat.rx_coe_sel) {
		ndev->hw_features |= NETIF_F_RXCSUM;
		ndev->hw_features |= NETIF_F_LRO;
		printk(KERN_ALERT "Supports RX COE and LRO\n");
	}
#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	ndev->vlan_features |= ndev->hw_features;
	ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_RX;
	if (pdata->hw_feat.sa_vlan_ins) {
		ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_TX;
		printk(KERN_ALERT "VLAN Feature enabled\n");
	}
	if (pdata->hw_feat.vlan_hash_en) {
		ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_FILTER;
		printk(KERN_ALERT "VLAN HASH Filtering enabled\n");
	}
#endif /* end of DWC_ETH_QOS_ENABLE_VLAN_TAG */
	ndev->features |= ndev->hw_features;
	pdata->dev_state |= ndev->features;

	DWC_ETH_QOS_init_rx_coalesce(pdata);

#ifdef DWC_ETH_QOS_CONFIG_PTP
	DWC_ETH_QOS_ptp_init(pdata);
#endif	/* end of DWC_ETH_QOS_CONFIG_PTP */

#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */

	spin_lock_init(&pdata->lock);
	spin_lock_init(&pdata->tx_lock);
	spin_lock_init(&pdata->pmt_lock);

#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	ret = DWC_ETH_QOS_alloc_pg(pdata);
	if (ret < 0) {
		printk(KERN_ALERT "ERROR:Unable to allocate PG memory\n");
		goto err_out_pg_failed;
	}
	printk(KERN_ALERT "\n");
	printk(KERN_ALERT "/*******************************************\n");
	printk(KERN_ALERT "*\n");
	printk(KERN_ALERT "* PACKET GENERATOR MODULE ENABLED IN DRIVER\n");
	printk(KERN_ALERT "*\n");
	printk(KERN_ALERT "*******************************************/\n");
	printk(KERN_ALERT "\n");
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */

	ret = register_netdev(ndev);
	if (ret) {
		printk(KERN_ALERT "%s: Net device registration failed\n",
		    DEV_NAME);
		goto err_out_netdev_failed;
	}

	DBGPR("<-- DWC_ETH_QOS_probe\n");

	if (pdata->hw_feat.pcs_sel) {
		netif_carrier_off(ndev);
		printk(KERN_ALERT "carrier off till LINK is up\n");
	}
	else
		printk(KERN_ALERT "Net device registration sucessful\n");

	ret = request_irq(power_irq, DWC_ETH_QOS_ISR_SW_DWC_ETH_QOS_POWER,
		IRQF_SHARED, DEV_NAME, pdata);

	if (ret != 0) {
		printk(KERN_ALERT "Unable to register PMT IRQ %d\n", power_irq);
		ret = -EBUSY;
		goto err_out_netdev_failed;
	}

	return 0;

 err_out_netdev_failed:
#ifdef DWC_ETH_QOS_CONFIG_PTP
	DWC_ETH_QOS_ptp_remove(pdata);
#endif	/* end of DWC_ETH_QOS_CONFIG_PTP */

#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	DWC_ETH_QOS_free_pg(pdata);
 err_out_pg_failed:
#endif
	if (1 == pdata->hw_feat.sma_sel)
		DWC_ETH_QOS_mdio_unregister(ndev);

#ifndef AR_XXX
#else
 err_out_mdio_reg:
#endif
	desc_if->free_queue_struct(pdata);

 err_out_q_alloc_failed:
	free_netdev(ndev);
	platform_set_drvdata(pdev, NULL);

err_out_dev_failed:
	iounmap((void *)dwc_eth_qos_base_addr);

	return ret;
}

/*!
* \brief API to release all the resources from the driver.
*
* \details The remove function gets called whenever a device being handled
* by this driver is removed (either during deregistration of the driver or
* when it is manually pulled out of a hot-pluggable slot). This function
* should reverse operations performed at probe time. The remove function
* always gets called from process context, so it can sleep.
*
* \param[in] pdev - pointer to pci_dev structure.
*
* \return void
*/

int DWC_ETH_QOS_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct desc_if_struct *desc_if = &(pdata->desc_if);
	int ret_val = 0;

	DBGPR("--> DWC_ETH_QOS_remove\n");

	if (pdev == NULL) {
		DBGPR("Remove called on invalid device\n");
		return -1;
	}

	if (pdata->power_irq != 0) {
		free_irq(pdata->power_irq, pdata);
		pdata->power_irq = 0;
	}
	if (pdata->irq_number != 0) {
		free_irq(pdata->irq_number, pdata);
		pdata->irq_number = 0;
	}

	if (1 == pdata->hw_feat.sma_sel)
		DWC_ETH_QOS_mdio_unregister(dev);

#ifdef DWC_ETH_QOS_CONFIG_PTP
	DWC_ETH_QOS_ptp_remove(pdata);
#endif /* end of DWC_ETH_QOS_CONFIG_PTP */

	unregister_netdev(dev);

#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	DWC_ETH_QOS_free_pg(pdata);
#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */

	desc_if->free_queue_struct(pdata);

	free_netdev(dev);

	platform_set_drvdata(pdev, NULL);
	iounmap((void *)dwc_eth_qos_base_addr);

	DBGPR("<-- DWC_ETH_QOS_remove\n");

	return ret_val;
}

static struct platform_driver DWC_ETH_QOS_driver = {

	.probe = DWC_ETH_QOS_probe,
	.remove = DWC_ETH_QOS_remove,
	.shutdown = DWC_ETH_QOS_shutdown,
#if 0
	.suspend_late = DWC_ETH_QOS_suspend_late,
	.resume_early = DWC_ETH_QOS_resume_early,
#endif
#ifdef CONFIG_PM
	.suspend = DWC_ETH_QOS_suspend,
	.resume = DWC_ETH_QOS_resume,
#endif
	.driver = {
		   .name = DEV_NAME,
		   .owner = THIS_MODULE,
			 .of_match_table = dwc_eth_qos_of_match,
	},
};

static void DWC_ETH_QOS_shutdown(struct platform_device *pdev)
{
	printk(KERN_ALERT "-->DWC_ETH_QOS_shutdown\n");
	printk(KERN_ALERT "Handle the shutdown\n");
	printk(KERN_ALERT ">--DWC_ETH_QOS_shutdown\n");

	return;
}

#if 0
static INT DWC_ETH_QOS_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	printk(KERN_ALERT "-->DWC_ETH_QOS_suspend_late\n");
	printk(KERN_ALERT "Handle the suspend_late\n");
	printk(KERN_ALERT "<--DWC_ETH_QOS_suspend_late\n");

	return 0;
}

static INT DWC_ETH_QOS_resume_early(struct platform_device *pdev)
{
	printk(KERN_ALERT "-->DWC_ETH_QOS_resume_early\n");
	printk(KERN_ALERT "Handle the resume_early\n");
	printk(KERN_ALERT "<--DWC_ETH_QOS_resume_early\n");

	return 0;
}

#endif

#ifdef CONFIG_PM

/*!
 * \brief Routine to put the device in suspend mode
 *
 * \details This function gets called by PCI core when the device is being
 * suspended. The suspended state is passed as input argument to it.
 * Following operations are performed in this function,
 * - stop the phy.
 * - detach the device from stack.
 * - stop the queue.
 * - Disable napi.
 * - Stop DMA TX and RX process.
 * - Enable power down mode using PMT module or disable MAC TX and RX process.
 * - Save the pci state.
 *
 * \param[in] pdev – pointer to pci device structure.
 * \param[in] state – suspend state of device.
 *
 * \return int
 *
 * \retval 0
 */

static INT DWC_ETH_QOS_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	INT ret, pmt_flags = 0;
	unsigned int rwk_filter_values[] = {
		/* for filter 0 CRC is computed on 0 - 7 bytes from offset */
		0x000000ff,

		/* for filter 1 CRC is computed on 0 - 7 bytes from offset */
		0x000000ff,

		/* for filter 2 CRC is computed on 0 - 7 bytes from offset */
		0x000000ff,

		/* for filter 3 CRC is computed on 0 - 31 bytes from offset */
		0x000000ff,

		/* filter 0, 1 independently enabled and would apply for
		 * unicast packet only filter 3, 2 combined as,
		 * "Filter-3 pattern AND NOT Filter-2 pattern" */
		0x03050101,

		/* filter 3, 2, 1 and 0 offset is 50, 58, 66, 74 bytes
		 * from start */
		0x4a423a32,

		/* pattern for filter 1 and 0, "0x55", "11", repeated 8 times */
		0xe7b77eed,

		/* pattern for filter 3 and 4, "0x44", "33", repeated 8 times */
		0x9b8a5506,
	};

	DBGPR("-->DWC_ETH_QOS_suspend\n");

	if (!dev || !netif_running(dev) || (!pdata->hw_feat.mgk_sel &&
			!pdata->hw_feat.rwk_sel)) {
		DBGPR("<--DWC_ETH_QOS_dev_suspend\n");
		return -EINVAL;
	}

	if (pdata->hw_feat.rwk_sel && (pdata->wolopts & WAKE_UCAST)) {
		pmt_flags |= DWC_ETH_QOS_REMOTE_WAKEUP;
		hw_if->configure_rwk_filter(rwk_filter_values, 8);
	}

	if (pdata->hw_feat.mgk_sel && (pdata->wolopts & WAKE_MAGIC))
		pmt_flags |= DWC_ETH_QOS_MAGIC_WAKEUP;

	ret = DWC_ETH_QOS_powerdown(dev, pmt_flags, DWC_ETH_QOS_DRIVER_CONTEXT);
#if (LP_SUPPORTED)
	pci_save_state(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));
#endif

	DBGPR("<--DWC_ETH_QOS_suspend\n");

	return ret;
}

/*!
 * \brief Routine to resume device operation
 *
 * \details This function gets called by PCI core when the device is being
 * resumed. It is always called after suspend has been called. These function
 * reverse operations performed at suspend time. Following operations are
 * performed in this function,
 * - restores the saved pci power state.
 * - Wakeup the device using PMT module if supported.
 * - Starts the phy.
 * - Enable MAC and DMA TX and RX process.
 * - Attach the device to stack.
 * - Enable napi.
 * - Starts the queue.
 *
 * \param[in] pdev – pointer to pci device structure.
 *
 * \return int
 *
 * \retval 0
 */

static INT DWC_ETH_QOS_resume(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	INT ret;

	DBGPR("-->DWC_ETH_QOS_resume\n");

	if (!dev || !netif_running(dev)) {
		DBGPR("<--DWC_ETH_QOS_dev_resume\n");
		return -EINVAL;
	}
#if (LP_SUPPORTED)
	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
#endif

	ret = DWC_ETH_QOS_powerup(dev, DWC_ETH_QOS_DRIVER_CONTEXT);

	DBGPR("<--DWC_ETH_QOS_resume\n");

	return ret;
}

#endif	/* CONFIG_PM */

/*!
* \brief API to register the driver.
*
* \details This is the first function called when the driver is loaded.
* It register the driver with PCI sub-system
*
* \return void.
*/

static int DWC_ETH_QOS_init_module(void)
{
	INT ret = 0;

	DBGPR("-->DWC_ETH_QOS_init_module\n");

	ret = platform_driver_register(&DWC_ETH_QOS_driver);
	if (ret < 0) {
		printk(KERN_ALERT "DWC_ETH_QOS:driver registration failed");
		return ret;
	}
	printk(KERN_ALERT "DWC_ETH_QOS:driver registration sucessful");

#ifdef DWC_ETH_QOS_CONFIG_DEBUGFS
	create_debug_files();
#endif

	DBGPR("<--DWC_ETH_QOS_init_module\n");

	return ret;
}

/*!
* \brief API to unregister the driver.
*
* \details This is the first function called when the driver is removed.
* It unregister the driver from PCI sub-system
*
* \return void.
*/

static void __exit DWC_ETH_QOS_exit_module(void)
{
	DBGPR("-->DWC_ETH_QOS_exit_module\n");

#ifdef DWC_ETH_QOS_CONFIG_DEBUGFS
	remove_debug_files();
#endif

	platform_driver_unregister(&DWC_ETH_QOS_driver);

	DBGPR("<--DWC_ETH_QOS_exit_module\n");
}

/*!
* \brief Macro to register the driver registration function.
*
* \details A module always begin with either the init_module or the function
* you specify with module_init call. This is the entry function for modules;
* it tells the kernel what functionality the module provides and sets up the
* kernel to run the module's functions when they're needed. Once it does this,
* entry function returns and the module does nothing until the kernel wants
* to do something with the code that the module provides.
*/
module_init(DWC_ETH_QOS_init_module);

/*!
* \brief Macro to register the driver un-registration function.
*
* \details All modules end by calling either cleanup_module or the function
* you specify with the module_exit call. This is the exit function for modules;
* it undoes whatever entry function did. It unregisters the functionality
* that the entry function registered.
*/
module_exit(DWC_ETH_QOS_exit_module);

/*!
* \brief Macro to declare the module author.
*
* \details This macro is used to declare the module's authore.
*/
MODULE_AUTHOR("Synopsys India Pvt Ltd");

/*!
* \brief Macro to describe what the module does.
*
* \details This macro is used to describe what the module does.
*/
MODULE_DESCRIPTION("DWC_ETH_QOS Driver");

/*!
* \brief Macro to describe the module license.
*
* \details This macro is used to describe the module license.
*/
MODULE_LICENSE("GPL");
