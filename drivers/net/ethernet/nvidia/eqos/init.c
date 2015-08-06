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
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/tegra-soc.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#define LP_SUPPORTED 0
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
	ULONG varCLK_CTRL = 0;

	if (tegra_platform_is_unit_fpga())
		CLK_CRTL0_RgRd(varCLK_CTRL);

	if (varCLK_CTRL & BIT(31)) {
		pr_info("power_isr: phy_intr received\n");
		return IRQ_NONE;
	} else {
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
	}
}

void get_dt_u32(struct device_node *pnode, char *pdt_prop, u32 *pval,
		u32 val_def, u32 val_max)
{
	int ret = 0;

	ret = of_property_read_u32(pnode, pdt_prop, pval);

	if (ret < 0)
		printk(KERN_ALERT "%s(): \"%s\" read failed %d. Using default\n",
			__func__, pdt_prop, ret);

	if (*pval > val_max) {
		printk(KERN_ALERT
			"%s(): %d is invalid value for \"%s\".  Using default.\n",
			__func__, *pval, pdt_prop);
		*pval = val_def;
	}
	printk(KERN_ALERT "%s(): \"%s\"=%d\n", __func__,
			pdt_prop, *pval);
}


void get_dt_u32_array(struct device_node *pnode, char *pdt_prop, u32 *pval,
			u32 val_def, u32 val_max, u32 num_entries)
{
	int i, ret = 0;

	ret = of_property_read_u32_array(pnode, pdt_prop, pval, num_entries);

	if (ret < 0) {
		printk(KERN_ALERT "%s(): \"%s\" read failed %d. Using default\n",
			__func__, pdt_prop, ret);
		for (i = 0; i < num_entries; i++)
			pval[i] = val_def;
	}
	for (i = 0; i < num_entries; i++) {
		if (pval[i] > val_max) {
			printk(KERN_ALERT "%d is invalid value for \"%s[%d]\"."
				"  Using default.\n",
				pval[i], pdt_prop, i);
			pval[i] = val_def;
		}
	}
	printk(KERN_ALERT "%s(): \"%s\" = 0x%x 0x%x 0x%x 0x%x\n", __func__,
		pdt_prop, pval[0], pval[1], pval[2], pval[3]);
}

static int eqos_pad_calibrate(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	int ret;
	uint i;
	u32 hwreg;

	DBGPR("-->%s()\n", __func__);

	/* 1. Set field PAD_E_INPUT_OR_E_PWRD in
	 * reg ETHER_QOS_SDMEMCOMPPADCTRL_0
	 */
	PAD_CRTL_E_INPUT_OR_E_PWRD_UdfWr(1);

	/* 2. delay for 1 usec */
	usleep_range(1, 3);

	/* 3. Set AUTO_CAL_ENABLE and AUTO_CAL_START in
	 * reg ETHER_QOS_AUTO_CAL_CONFIG_0.
	 */
	PAD_AUTO_CAL_CFG_RgRd(hwreg);
	hwreg |=
		((PAD_AUTO_CAL_CFG_START_MASK) |
			(PAD_AUTO_CAL_CFG_ENABLE_MASK));

	PAD_AUTO_CAL_CFG_RgWr(hwreg);

	/* 4. Wait on AUTO_CAL_ACTIVE until it is 1. 10us timeout */
	i = 10;
	while (i--) {
		usleep_range(1, 3);
		PAD_AUTO_CAL_STAT_RgRd(hwreg);

		/* calibration started when CAL_STAT_ACTIVE is set */
		if (hwreg & PAD_AUTO_CAL_STAT_ACTIVE_MASK)
			break;
	}
	if (!i) {
		ret = -1;
		dev_err(&pdev->dev,
			"eqos pad calibration took too long to start\n");
		goto calibration_failed;
	}

	/* 5. Wait on AUTO_CAL_ACTIVE until it is 0. 200us timeout */
	i = 10;
	while (i--) {
		usleep_range(20, 30);
		PAD_AUTO_CAL_STAT_RgRd(hwreg);

		/* calibration done when CAL_STAT_ACTIVE is zero */
		if (!(hwreg & PAD_AUTO_CAL_STAT_ACTIVE_MASK))
			break;
	}
	if (!i) {
		ret = -1;
		dev_err(&pdev->dev,
			"eqos pad calibration took too long to complete\n");
		goto calibration_failed;
	}
	ret = 0;

calibration_failed:
	/* 6. Disable field PAD_E_INPUT_OR_E_PWRD in
	 * reg ETHER_QOS_SDMEMCOMPPADCTRL_0 to save power.
	 */
	PAD_CRTL_E_INPUT_OR_E_PWRD_UdfWr(0);

	DBGPR("<--%s()\n", __func__);

	return ret;
}

static void eqos_clock_deinit(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;

	clk_disable(pdata->tx_clk);
	clk_disable(pdata->ptp_ref_clk);
	clk_disable(pdata->rx_clk);
	clk_disable(pdata->axi_clk);
	clk_disable(pdata->axi_cbb_clk);

	devm_clk_put(&pdev->dev, pdata->tx_clk);
	devm_clk_put(&pdev->dev, pdata->ptp_ref_clk);
	devm_clk_put(&pdev->dev, pdata->rx_clk);
	devm_clk_put(&pdev->dev, pdata->axi_clk);
	devm_clk_put(&pdev->dev, pdata->axi_cbb_clk);
}

static int eqos_clock_init(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	struct device_node *node = pdev->dev.of_node;
	u32 ptp_ref_clock_speed;
	int ret;

	pdata->axi_cbb_clk = devm_clk_get(&pdev->dev, "axi_cbb");
	if (IS_ERR(pdata->axi_cbb_clk)) {
		ret = PTR_ERR(pdata->axi_cbb_clk);
		dev_err(&pdev->dev, "can't get axi_cbb clk (%d)\n", ret);
		return ret;
	}
	pdata->axi_clk = devm_clk_get(&pdev->dev, "eqos_axi");
	if (IS_ERR(pdata->axi_clk)) {
		ret = PTR_ERR(pdata->axi_clk);
		dev_err(&pdev->dev, "can't get eqos_axi clk (%d)\n", ret);
		goto axi_get_fail;
	}
	pdata->rx_clk = devm_clk_get(&pdev->dev, "eqos_rx");
	if (IS_ERR(pdata->rx_clk)) {
		ret = PTR_ERR(pdata->rx_clk);
		dev_err(&pdev->dev, "can't get eqos_rx clk (%d)\n", ret);
		goto rx_get_fail;
	}
	pdata->ptp_ref_clk = devm_clk_get(&pdev->dev, "eqos_ptp_ref");
	if (IS_ERR(pdata->ptp_ref_clk)) {
		ret = PTR_ERR(pdata->ptp_ref_clk);
		dev_err(&pdev->dev, "can't get eqos_ptp_ref clk (%d)\n", ret);
		goto ptp_ref_get_fail;
	}
	pdata->tx_clk = devm_clk_get(&pdev->dev, "eqos_tx");
	if (IS_ERR(pdata->tx_clk)) {
		ret = PTR_ERR(pdata->tx_clk);
		dev_err(&pdev->dev, "can't get eqos_tx clk (%d)\n", ret);
		goto tx_get_fail;
	}

	ret = clk_enable(pdata->axi_cbb_clk);
	if (ret < 0)
		goto axi_cbb_en_fail;

	ret = clk_enable(pdata->axi_clk);
	if (ret < 0)
		goto axi_en_fail;

	ret = clk_enable(pdata->rx_clk);
	if (ret < 0)
		goto rx_en_fail;

	ret = clk_enable(pdata->ptp_ref_clk);
	if (ret < 0)
		goto ptp_ref_en_fail;

	/* set ptp_ref_clk freq default 62.5Mhz */
	ret = of_property_read_u32(node, "nvidia,ptp_ref_clock_speed",
			&ptp_ref_clock_speed);
	if (ret < 0) {
		dev_err(&pdev->dev,
		"ptp_ref_clk read failed %d, setting default to 125MHz\n", ret);
		/* take default as 125MHz */
		ptp_ref_clock_speed = 125;
	}

	ret = clk_set_rate(pdata->ptp_ref_clk, ptp_ref_clock_speed * 1000000);
	if (ret) {
		dev_err(&pdev->dev, "ptp_ref clk set rate failed (%d)\n", ret);
		goto ptp_ref_set_rate_failed;
	}

	ret = clk_enable(pdata->tx_clk);
	if (ret < 0)
		goto tx_en_fail;

	dev_info(&pdev->dev, "axi_cbb/axi/rx/ptp/tx = %ld/%ld/%ld/%ld/%ld\n",
		clk_get_rate(pdata->axi_cbb_clk),
		clk_get_rate(pdata->axi_clk), clk_get_rate(pdata->rx_clk),
		clk_get_rate(pdata->ptp_ref_clk), clk_get_rate(pdata->tx_clk));

	return 0;

tx_en_fail:
ptp_ref_set_rate_failed:
	clk_disable(pdata->ptp_ref_clk);
ptp_ref_en_fail:
	clk_disable(pdata->rx_clk);
rx_en_fail:
	clk_disable(pdata->axi_clk);
axi_en_fail:
	clk_disable(pdata->axi_cbb_clk);
axi_cbb_en_fail:
	devm_clk_put(&pdev->dev, pdata->tx_clk);
tx_get_fail:
	devm_clk_put(&pdev->dev, pdata->ptp_ref_clk);
ptp_ref_get_fail:
	devm_clk_put(&pdev->dev, pdata->rx_clk);
rx_get_fail:
	devm_clk_put(&pdev->dev, pdata->axi_clk);
axi_get_fail:
	devm_clk_put(&pdev->dev, pdata->axi_cbb_clk);
	return ret;
}

static void eqos_regulator_deinit(struct DWC_ETH_QOS_prv_data *pdata)
{
	regulator_disable(pdata->phy_pllvdd);
	regulator_disable(pdata->phy_ovdd_rgmii);
	regulator_disable(pdata->phy_vdd_1v8);
	devm_regulator_put(pdata->phy_pllvdd);
	devm_regulator_put(pdata->phy_ovdd_rgmii);
	devm_regulator_put(pdata->phy_vdd_1v8);
}

static int eqos_regulator_init(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	int ret = 0;

	pdata->phy_vdd_1v8 = devm_regulator_get(&pdev->dev, "phy_vdd_1v8");
	if (IS_ERR(pdata->phy_vdd_1v8)) {
		ret = PTR_ERR(pdata->phy_vdd_1v8);
		dev_err(&pdev->dev, "phy_vdd_1v8 get failed %d\n", ret);
		return ret;
	}

	pdata->phy_ovdd_rgmii =
		devm_regulator_get(&pdev->dev, "phy_ovdd_rgmii");
	if (IS_ERR(pdata->phy_ovdd_rgmii)) {
		ret = PTR_ERR(pdata->phy_ovdd_rgmii);
		dev_err(&pdev->dev, "phy_ovdd_rgmii get failed %d\n", ret);
		goto phy_ovdd_rgmii_get_failed;
	}

	pdata->phy_pllvdd = devm_regulator_get(&pdev->dev,
		"phy_pllvdd");
	if (IS_ERR(pdata->phy_pllvdd)) {
		ret = PTR_ERR(pdata->phy_pllvdd);
		dev_err(&pdev->dev, "phy_pllvdd get failed %d\n", ret);
		goto phy_pllvdd_get_failed;
	}

	ret = regulator_enable(pdata->phy_vdd_1v8);
	if (ret) {
		dev_err(&pdev->dev, "phy_vdd_1v8 enable failed %d\n", ret);
		goto phy_vdd_1v8_enable_failed;
	}

	ret = regulator_enable(pdata->phy_ovdd_rgmii);
	if (ret) {
		dev_err(&pdev->dev, "phy_ovdd_rgmii enable failed %d\n", ret);
		goto phy_ovdd_rgmii_enable_failed;
	}

	ret = regulator_enable(pdata->phy_pllvdd);
	if (ret) {
		dev_err(&pdev->dev, "phy_pllvdd enable failed %d\n", ret);
		goto phy_pllvdd_enable_failed;
	}

	return 0;

phy_pllvdd_enable_failed:
	regulator_disable(pdata->phy_ovdd_rgmii);
phy_ovdd_rgmii_enable_failed:
	regulator_disable(pdata->phy_vdd_1v8);
phy_vdd_1v8_enable_failed:
	devm_regulator_put(pdata->phy_pllvdd);
phy_pllvdd_get_failed:
	devm_regulator_put(pdata->phy_ovdd_rgmii);
phy_ovdd_rgmii_get_failed:
	devm_regulator_put(pdata->phy_vdd_1v8);
	return ret;
}

static int eqos_get_phyreset_from_gpio(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	struct device_node *node = pdev->dev.of_node;
	int ret;

	pdata->phy_reset_gpio =
		of_get_named_gpio(node, "nvidia,phy-reset-gpio", 0);
	if (pdata->phy_reset_gpio < 0) {
		dev_err(&pdev->dev, "failed to read phy_reset_gpio\n");
		return -ENODEV;
	}
	if (gpio_is_valid(pdata->phy_reset_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, pdata->phy_reset_gpio,
				GPIOF_OUT_INIT_HIGH, "eqos_phy_reset");
		if (ret < 0) {
			dev_err(&pdev->dev, "phy_reset gpio_request failed\n");
			return ret;
		}
	} else {
		dev_err(&pdev->dev, "invalid phy_reset_gpio\n");
		return -ENODEV;
	}
	return 0;
}

static int eqos_get_phyirq_from_gpio(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	struct device_node *node = pdev->dev.of_node;
	int ret;

	pdata->phy_intr_gpio =
			of_get_named_gpio(node, "nvidia,phy-intr-gpio", 0);
	if (pdata->phy_intr_gpio < 0) {
		dev_err(&pdev->dev, "failed to read phy_intr_gpio\n");
		return -ENODEV;
	}
	if (gpio_is_valid(pdata->phy_intr_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, pdata->phy_intr_gpio,
				GPIOF_IN, "eqos_phy_intr");
		if (ret < 0) {
			dev_err(&pdev->dev, "phy_intr gpio_request failed\n");
			return ret;
		}
		ret = gpio_to_irq(pdata->phy_intr_gpio);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"gpio_to_irq failed for phy_intr\n");
			return ret;
		}
	} else {
		dev_err(&pdev->dev, "invalid phy_intr_gpio\n");
		return -ENODEV;
	}
	return ret;
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
	int i, j, ret = 0;
	int irq, power_irq;
	int phyirq;
	int rx_irqs[MAX_CHANS];
	int tx_irqs[MAX_CHANS];
	struct hw_if_struct *hw_if = NULL;
	struct desc_if_struct *desc_if = NULL;
	UCHAR tx_q_count = 0, rx_q_count = 0;
	struct resource *res;
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	u32 csr_clock_speed;
	u32 mac_addr[6];

	struct eqos_cfg *pdt_cfg;
	struct chan_data *pchinfo;

	DBGPR("-->%s()\n", __func__);

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

	for (i = IRQ_CHAN0_RX_IDX, j = 0; i <= IRQ_MAX_IDX; i += 2, j++) {
		rx_irqs[j] = platform_get_irq(pdev, i);
		if (rx_irqs[j] < 0)
			return rx_irqs[j];

		tx_irqs[j] = platform_get_irq(pdev, i+1);
		if (tx_irqs[j] < 0)
			return tx_irqs[j];
	}

#if defined(CONFIG_PHYS_ADDR_T_64BIT)
	DBGPR("res->start = 0x%lx\n", (unsigned long)res->start);
	DBGPR("res->end = 0x%lx\n", (unsigned long)res->end);
#else
	DBGPR("res->start = 0x%x\n", (unsigned int)res->start);
	DBGPR("res->end = 0x%x\n", (unsigned int)res->end);
#endif
	DBGPR("irq = %d\n", irq);
	DBGPR("power_irq = %d\n", power_irq);

	for (j = 0; j < MAX_CHANS; j++)
		DBGPR("rx_irq[%d]=%d, tx_irq[%d]=%d\n",
			j, rx_irqs[j], j, tx_irqs[j]);

	DBGPR("==========================================================\n");
	DBGPR("Sizeof rx context desc is %lu\n", sizeof(struct s_RX_CONTEXT_DESC));
	DBGPR("Sizeof tx context desc is %lu\n", sizeof(struct s_TX_CONTEXT_DESC));
	DBGPR("Sizeof rx normal desc is %lu\n", sizeof(struct s_RX_NORMAL_DESC));
	DBGPR("Sizeof tx normal desc is %lu\n\n", sizeof(struct s_TX_NORMAL_DESC));
	DBGPR("==========================================================\n");

	/*remap base address*/
	dwc_eth_qos_base_addr = (ULONG)ioremap_nocache(res->start, (res->end - res->start) + 1);

	/* allocate and set up the ethernet device*/
	ndev = alloc_etherdev_mqs(sizeof(struct DWC_ETH_QOS_prv_data),
				MAX_CHANS, MAX_CHANS);
	if (ndev == NULL) {
		printk(KERN_ALERT "%s:Unable to alloc new net device\n",
		    DEV_NAME);
		ret = -ENOMEM;
		goto err_out_dev_failed;
	}

	ndev->base_addr = dwc_eth_qos_base_addr;
	SET_NETDEV_DEV(ndev, &pdev->dev);
	pdata = netdev_priv(ndev);
	DWC_ETH_QOS_init_all_fptrs(pdata);
	hw_if = &(pdata->hw_if);
	desc_if = &(pdata->desc_if);

	platform_set_drvdata(pdev, ndev);
	pdata->pdev = pdev;

	pdata->dev = ndev;

	/* PMT and PHY irqs are shared on FPGA system */
	if (tegra_platform_is_unit_fpga()) {
		phyirq = power_irq;
	} else {
		/* regulator init */
		ret = eqos_regulator_init(pdata);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to enable regulator %d\n", ret);
			goto err_out_regulator_en_failed;
		}

		/* On silicon the phy_intr line is handled through a wake
		 * capable GPIO input. DMIC4_CLK is the GPIO input port.
		 */
		phyirq = eqos_get_phyirq_from_gpio(pdata);
		if (phyirq < 0) {
			dev_err(&pdev->dev, "get_phyirq_from_gpio failed\n");
			goto err_out_phyirq_failed;
		}

		/* setup PHY reset gpio */
		ret = eqos_get_phyreset_from_gpio(pdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "get_phyreset_from_gpio failed\n");
			goto err_out_phyreset_failed;
		}

		/* reset the PHY Broadcom PHY needs minimum of 2us delay */
		gpio_set_value(pdata->phy_reset_gpio, 0);
		usleep_range(10, 11);
		gpio_set_value(pdata->phy_reset_gpio, 1);

		/* CAR reset */
		pdata->eqos_rst =
			devm_reset_control_get(&pdev->dev, "eqos_rst");
		if (IS_ERR_OR_NULL(pdata->eqos_rst)) {
			ret = PTR_ERR(pdata->eqos_rst);
			dev_err(&pdev->dev,
				"failed to get eqos reset %d\n", ret);
			goto err_out_reset_get_failed;
		} else {
			reset_control_deassert(pdata->eqos_rst);
		}

		/* clock initialization */
		ret = eqos_clock_init(pdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "eqos_clock_init failed\n");
			goto err_out_clock_init_failed;
		}
	}
	DBGPR("phyirq = %d\n", phyirq);



	/* queue count */
	tx_q_count = get_tx_queue_count();
	rx_q_count = get_rx_queue_count();

	pdata->tx_queue_cnt = tx_q_count;
	pdata->rx_queue_cnt = rx_q_count;

	/* calibrate pad */
	if (!tegra_platform_is_unit_fpga()) {
		ret = eqos_pad_calibrate(pdata);
		if (ret < 0)
			goto err_out_dev_failed;
	}

#ifdef DWC_ETH_QOS_CONFIG_DEBUGFS
	/* to give prv data to debugfs */
	DWC_ETH_QOS_get_pdata(pdata);
#endif

	/* issue software reset to device */
	hw_if->exit();

	/* ack and disable wrapper tx/rx irq ints */
	i = (VIRT_INTR_CH_CRTL_RX_Wr_Mask | VIRT_INTR_CH_CRTL_TX_Wr_Mask);

	for (j = 0; j < MAX_CHANS; j++) {
		VIRT_INTR_CH_STAT_RgWr(j, i);
		VIRT_INTR_CH_CRTL_RgWr(j, ~i);
	}

	ndev->irq = irq;
	pdata->common_irq = irq;

	pdata->power_irq = power_irq;
	pdata->phyirq = phyirq;

	for (j = 0; j < MAX_CHANS; j++) {
		pdata->rx_irqs[j] = rx_irqs[j];
		pdata->tx_irqs[j] = tx_irqs[j];
	}

	DWC_ETH_QOS_get_all_hw_features(pdata);
	DWC_ETH_QOS_print_all_hw_features(pdata);

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
			goto err_out_mdio_reg;
		}
	} else {
		printk(KERN_ALERT "%s: MDIO is not present\n\n", DEV_NAME);
	}

	pdata->ptp_cfg.use_tagged_ptp = of_property_read_bool(node,
			"nvidia,use_tagged_ptp");
	get_dt_u32(node, "nvidia,ptp_dma_ch",
		&(pdata->ptp_cfg.ptp_dma_ch_id),
		PTP_DMA_CH_DEFAULT, PTP_DMA_CH_MAX);

	pdt_cfg = (struct eqos_cfg *)&pdata->dt_cfg;
	get_dt_u32(node, "nvidia,intr_mode", &pdt_cfg->intr_mode,
			INTR_MODE_DEFAULT, MODE_MAX);
	get_dt_u32(node, "nvidia,pause_frames", &pdt_cfg->pause_frames,
			PAUSE_FRAMES_DEFAULT, PAUSE_FRAMES_MAX);
	get_dt_u32_array(node, "nvidia,chan_mode", pdt_cfg->chan_mode,
			CHAN_MODE_DEFAULT, CHAN_MODE_MAX, 4);
	get_dt_u32_array(node, "nvidia,chan_napi_quota",
			pdt_cfg->chan_napi_quota,
			CHAN_NAPI_QUOTA_DEFAULT, CHAN_NAPI_QUOTA_MAX, 4);
	get_dt_u32_array(node, "nvidia,rxq_enable_ctrl", pdt_cfg->rxq_ctrl,
			RXQ_CTRL_DEFAULT, RXQ_CTRL_MAX, 4);

	for (i = 0; i < MAX_CHANS; i++) {
		pchinfo = &pdata->chinfo[i];
		pchinfo->chan_num = i;
		pchinfo->poll_interval = 1000;
	}

	if (pdata->dt_cfg.intr_mode == MODE_COMMON_IRQ) {
		pdata->dt_cfg.chan_mode[0] = CHAN_MODE_NONE;
		pdata->dt_cfg.chan_mode[1] = CHAN_MODE_NONE;
		pdata->dt_cfg.chan_mode[2] = CHAN_MODE_NONE;
		pdata->dt_cfg.chan_mode[3] = CHAN_MODE_NONE;
	}

	for (i = 0; i < MAX_CHANS; i++)
		pdata->napi_quota_all_chans += pdt_cfg->chan_napi_quota[i];

	ret = of_property_read_u32(node, "nvidia,csr_clock_speed",
			&csr_clock_speed);
	if (ret < 0)
		printk(KERN_ALERT "csr_clock_speed read failed %d\n", ret);

	MAC_1US_TIC_RgWr(csr_clock_speed - 1);

	ret = of_property_read_u32_array(node, "nvidia,local-mac-address",
			mac_addr, sizeof(mac_addr)/sizeof(u32));
	if (ret < 0) {
		printk(KERN_ALERT "local-mac-address read failed %d\n", ret);
		goto err_out_mac_read_failed;
	} else if (mac_addr[0] == 0x0 && mac_addr[1] == 0x0 &&
		mac_addr[2] == 0x0 && mac_addr[3] == 0x0 &&
		mac_addr[4] == 0x0 && mac_addr[5] == 0x0) {
		printk(KERN_ALERT "ERROR!! local-mac-address is all zeros.\n");
		printk(KERN_ALERT "Update DT with unique MAC address\n");
		goto err_out_mac_read_failed;
	} else {
		printk(KERN_ALERT "Setting local MAC: %x %x %x %x %x %x\n",
			mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3],
			mac_addr[4], mac_addr[5]);
		/* Set up MAC address */
		ndev->dev_addr[0] = mac_addr[0];
		ndev->dev_addr[1] = mac_addr[1];
		ndev->dev_addr[2] = mac_addr[2];
		ndev->dev_addr[3] = mac_addr[3];
		ndev->dev_addr[4] = mac_addr[4];
		ndev->dev_addr[5] = mac_addr[5];
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

		if (pdata->dt_cfg.intr_mode == MODE_MULTI_IRQ) {
			netif_napi_add(ndev, &rx_queue->napi,
					DWC_ETH_QOS_poll_mq_napi,
					pdata->napi_quota_all_chans);
		} else
			netif_napi_add(ndev, &rx_queue->napi,
					DWC_ETH_QOS_poll_mq,
					pdata->napi_quota_all_chans);
		rx_queue->chan_num = i;
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

	for (i = 0; i < MAX_CHANS; i++)
		spin_lock_init(&pdata->chinfo[i].chan_lock);

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
		goto err_out_pmt_irq_failed;
	}

	return 0;

 err_out_pmt_irq_failed:
	unregister_netdev(ndev);
 err_out_netdev_failed:
#ifdef DWC_ETH_QOS_CONFIG_PTP
	DWC_ETH_QOS_ptp_remove(pdata);
#endif	/* end of DWC_ETH_QOS_CONFIG_PTP */

#ifdef DWC_ETH_QOS_CONFIG_PGTEST
	DWC_ETH_QOS_free_pg(pdata);
 err_out_pg_failed:
#endif
 err_out_mac_read_failed:
	if (1 == pdata->hw_feat.sma_sel)
		DWC_ETH_QOS_mdio_unregister(ndev);

 err_out_mdio_reg:
	desc_if->free_queue_struct(pdata);

 err_out_q_alloc_failed:
	if (!tegra_platform_is_unit_fpga())
		eqos_clock_deinit(pdata);
 err_out_clock_init_failed:
	if (!tegra_platform_is_unit_fpga() &&
		!IS_ERR_OR_NULL(pdata->eqos_rst))
		reset_control_assert(pdata->eqos_rst);
 err_out_reset_get_failed:
 err_out_phyreset_failed:
 err_out_phyirq_failed:
	if (!tegra_platform_is_unit_fpga())
		eqos_regulator_deinit(pdata);
 err_out_regulator_en_failed:
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

	if (!tegra_platform_is_unit_fpga())
		eqos_clock_deinit(pdata);

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
	if (!tegra_platform_is_unit_fpga() &&
		!IS_ERR_OR_NULL(pdata->eqos_rst))
		reset_control_assert(pdata->eqos_rst);
	if (!tegra_platform_is_unit_fpga())
		eqos_regulator_deinit(pdata);
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
