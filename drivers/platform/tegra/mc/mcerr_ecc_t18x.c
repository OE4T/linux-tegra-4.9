/*
 * Tegra 18x SoC-specific DRAM ECC Error handling code.
 *
 * Copyright (c) 2016, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define pr_fmt(fmt) "ecc-err: " fmt

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/mcerr_ecc_t18x.h>
#include <linux/platform/tegra/tegra18_emc.h>

static struct mc_ecc_err_log ecc_log;
void __iomem *emc;
void __iomem *emc_regs[MAX_CHANNELS];
u32 emc_int_status[MAX_CHANNELS];
u32 gbl_int_status;
u32 ecc_int_mask;
u32 ecc_err_silenced;

#define ecc_err_pr(fmt, ...)					\
	do {							\
		if (!ecc_err_silenced) {			\
			trace_printk(fmt, ##__VA_ARGS__);	\
			pr_err(fmt, ##__VA_ARGS__);		\
		}						\
	} while (0)

static int mc_check_ebe(struct mc_ecc_err_log *log)
{
	if ((log->ecc_eerr_par_sp0) || (log->ecc_eerr_par_sp1))
		return 1;

	return 0;
}

static int mc_check_sbe(struct mc_ecc_err_log *log)
{
	if ((log->ecc_derr_par_sp0 == 1) || (log->ecc_derr_par_sp1 == 1))
		return 1;

	return 0;
}

static int mc_check_dbe(struct mc_ecc_err_log *log)
{
	if ((log->ecc_derr_par_sp0 == 2) || (log->ecc_derr_par_sp1 == 2))
		return 1;

	return 0;
}

static int mc_check_poison(struct mc_ecc_err_log *log)
{
	if ((log->ecc_err_poison_sp0 == 1) || (log->ecc_err_poison_sp1 == 1))
		return 1;

	return 0;
}

static void mc_ecc_dump_regs(struct mc_ecc_err_log *log)
{
	ecc_err_pr("EMC_ECC_ERR_REQ = 0x%08x\n", log->emc_ecc_err_req);
	ecc_err_pr("EMC_ECC_ERR_SP0 = 0x%08x\n", log->emc_ecc_err_sp0);
	ecc_err_pr("EMC_ECC_ERR_SP1 = 0x%08x\n", log->emc_ecc_err_sp1);
	ecc_err_pr("EMC_ECC_ERR_ADDR = 0x%08x\n", log->ecc_err_addr);

	ecc_err_pr("D.X.R.B.C.S.L:%d.%d.0x%x.%d.0x%x.%d.0\n", log->ecc_err_dev,
			log->ecc_err_ch, log->row, log->bank, log->col,
			log->subp);
}

static void mc_ecc_dump_log(struct mc_ecc_err_log *log)
{
	pr_debug("emc_ecc_err_sp0 - reg value = 0x%08x\n",
		log->emc_ecc_err_sp0);
	pr_debug("emc_ecc_err_sp1 - reg value = 0x%08x\n",
		log->emc_ecc_err_sp1);
	pr_debug("ncol = 0x%08x, gob=0x%08x, col_sp0=0x%08x, err_seq=0x%08x\n",
		log->col, log->gob, log->col_sp0, log->err_seq);
	pr_debug("ecc_err_cgid = %d, ecc_err_ch = %d\n", log->ecc_err_cgid,
		log->ecc_err_ch);
	pr_debug("ecc_err_dev = %d, ecc_err_size = %d\n", log->ecc_err_dev,
		log->ecc_err_size);
	pr_debug("ecc_err_swap = %d\n", log->ecc_err_swap);
	pr_debug("ecc_eerr_par_sp0 = %d, ecc_eerr_par_sp1 = %d\n",
		log->ecc_eerr_par_sp0, log->ecc_eerr_par_sp1);
	pr_debug("ecc_derr_par_sp0 = %d, ecc_derr_par_sp1 = %d\n",
		log->ecc_derr_par_sp0, log->ecc_derr_par_sp1);
	pr_debug("ecc_err_poison_sp0 = %d,ecc_err_poison_sp1 = %d\n",
		log->ecc_err_poison_sp0, log->ecc_err_poison_sp1);
	pr_debug("ecc_err_addr = 0x%08x\n", log->ecc_err_addr);
	pr_debug("row = 0x%08x, bank = 0x%08x\n", log->row, log->bank);
	pr_debug("D.X.R.B.C.S.L:%d.%d.0x%x.%d.0x%x.%d.0\n", log->ecc_err_dev,
			log->ecc_err_ch, log->row, log->bank, log->col,
			log->subp);
}

static int mc_check_subp1_err(struct mc_ecc_err_log *log)
{
	if (((log->ecc_derr_par_sp1 == 1)  ||  (log->ecc_derr_par_sp1 == 2)) &&
		(!((log->ecc_derr_par_sp0 == 1) ||
		(log->ecc_derr_par_sp0 == 2))))
		return 1; /*Error only in subp1*/
	else
		return 0; /*Error on both Subpartitions or only Subp0*/
}

static void mc_ecc_read_log(struct mc_ecc_err_log *log, u32 ch)
{
	u32 val;

	memset((void *)log, 0, sizeof(struct mc_ecc_err_log));

	val = __emc_readl(ch, EMC_ECC_CONTROL);
	val |= (1 << ERR_BUFFER_LOAD_SHIFT);
	__emc_writel(ch, val, EMC_ECC_CONTROL);

	val = __emc_readl(ch, EMC_ECC_ERR_REQ);
	log->emc_ecc_err_req = val;
	log->ecc_err_cgid = ((val >> ECC_ERR_CGID_SHIFT) & ECC_ERR_CGID_MASK);
	log->ecc_err_ch = ((val >> ECC_ERR_EMC_ID_SHIFT) & ECC_ERR_EMC_ID_MASK);

	log->ecc_err_dev = ((val >> ECC_ERR_DEVICE_SHIFT) &
						ECC_ERR_DEVICE_MASK);
	log->ecc_err_size = ((val >> ECC_ERR_SIZE_SHIFT) & ECC_ERR_SIZE_MASK);
	log->ecc_err_swap = ((val >> ECC_ERR_SWAP_SHIFT) &  ECC_ERR_SWAP_MASK);
	log->col_sp0 = ((val >> ECC_ERR_COL_SP0_SHIFT) & ECC_ERR_COL_SP0_MASK);
	log->col_sp1 = ((val >> ECC_ERR_COL_SP1_SHIFT) & ECC_ERR_COL_SP1_MASK);
	log->err_seq = ((val >> ECC_ERR_SEQ_SHIFT) & ECC_ERR_SEQ_MASK);

	val = __emc_readl(ch, EMC_ECC_ERR_SP0);
	log->emc_ecc_err_sp0 = val;
	log->ecc_eerr_par_sp0 = ((val >> ECC_EERR_PAR_SHIFT) &
						ECC_EERR_PAR_MASK);
	log->ecc_derr_par_sp0 = ((val >> ECC_DERR_PAR_SHIFT) &
						ECC_DERR_PAR_MASK);
	log->ecc_err_poison_sp0 = ((val >> ECC_ERR_POISON_SHIFT) &
						ECC_ERR_POISON_MASK);
	log->ecc_err_bit_sp0 = (((val >> ECC_DERR_SYNDROME_SHIFT) &
					ECC_DERR_SYNDROME_MASK) & ~(0x200));


	val = __emc_readl(ch, EMC_ECC_ERR_SP1);
	log->emc_ecc_err_sp1 = val;
	log->ecc_eerr_par_sp1 = ((val >> ECC_EERR_PAR_SHIFT) &
						ECC_EERR_PAR_MASK);
	log->ecc_derr_par_sp1 = ((val >> ECC_DERR_PAR_SHIFT) &
						ECC_DERR_PAR_MASK);
	log->ecc_err_poison_sp1 = ((val >> ECC_ERR_POISON_SHIFT) &
						ECC_ERR_POISON_MASK);
	log->ecc_err_bit_sp1 = (((val >> ECC_DERR_SYNDROME_SHIFT) &
					ECC_DERR_SYNDROME_MASK) & ~(0x200));

	val = __emc_readl(ch, EMC_ECC_ERR_ADDR);
	log->ecc_err_addr = val;
	log->row = ((val >> ECC_ERR_ROW_SHIFT) & ECC_ERR_ROW_MASK);
	log->bank = ((val >> ECC_ERR_BANK_SHIFT) & ECC_ERR_BANK_MASK);
	log->gob = ((val >> ECC_ERR_GOB_SHIFT) & ECC_ERR_GOB_MASK);
	log->col = log->gob;

	if ((log->ecc_err_size == 1) || (log->ecc_err_size == 2)) {
		/* Read size is 32bytes or 16bytes */
		log->subp = log->ecc_err_swap;
	} else {
		/* Read size is 64bytes */
		if (mc_check_poison(log))
			log->subp = 0;
		else if (mc_check_subp1_err(log))
			log->subp = 1;
		else
			log->subp = 0;
	}

	if (log->subp)
		log->col = (log->col << 3) | log->col_sp1;
	else
		log->col = (log->col << 3) | log->col_sp0;

	log->col = (log->col << 2) | log->err_seq;
	log->col = (log->col << 2);

	pr_debug("D.X.R.B.C.S.L:%d.%d.0x%x.%d.0x%x.%d.0\n", log->ecc_err_dev,
			log->ecc_err_ch, log->row, log->bank, log->col,
			log->subp);
}

static void mc_ecc_dump_intr_mask(void)
{
	pr_info("EMC_INTMASK = 0x%08x\n",
			__emc_readl(0, EMC_INTMASK));
	pr_info("EMC_NONCRITICAL_INTMASK = 0x%08x\n",
			__emc_readl(0, EMC_NONCRITICAL_INTMASK));
	pr_info("EMC_CRITICAL_INTMASK = 0x%08x\n",
			__emc_readl(0, EMC_CRITICAL_INTMASK));
}

static void mc_config_ecc_log(uint32_t mode, uint32_t depth)
{
	uint32_t val;

	val = __emc_readl(0, EMC_ECC_CONTROL);

	val &= ~(ERR_BUFFER_MODE_MASK << ERR_BUFFER_MODE_SHIFT);
	val |= (mode << ERR_BUFFER_MODE_SHIFT);

	val &= ~(ERR_BUFFER_LIMIT_MASK << ERR_BUFFER_LIMIT_SHIFT);
	val |= (depth << ERR_BUFFER_LIMIT_SHIFT);

	emc_writel(val, EMC_ECC_CONTROL);
}

static void mc_ecc_clear_all_intr(void)
{
	emc_writel(0xFFFFFFFF, EMC_INTSTATUS);
	emc_writel(0xF, EMC_MCH_GLOBAL_INTSTATUS);
	emc_writel(0xF, EMC_MCH_GLOBAL_NONCRITICAL_INTSTATUS);
	emc_writel(0xF, EMC_MCH_GLOBAL_CRITICAL_INTSTATUS);
}

static void mc_ecc_dump_ch_logs(uint32_t ch)
{
	uint32_t val, err_buff_count;
	struct mc_ecc_err_log *pecclog = &ecc_log;

	val = __emc_readl(ch, EMC_ECC_STATUS);
	err_buff_count = ((val >> ERR_BUFFER_CNT_SHIFT) &
					ERR_BUFFER_CNT_MASK);

	while (err_buff_count) {
		pr_debug("----ch%d---Error log(%d)---<Start>-----\n", ch,
				err_buff_count);
		mc_ecc_read_log(pecclog, ch);
		mc_ecc_dump_log(pecclog);
		pr_debug("----ch%d---Error log(%d)---<End>-----\n", ch,
				err_buff_count);
		val = __emc_readl(ch, EMC_ECC_STATUS);
		err_buff_count = ((val >> ERR_BUFFER_CNT_SHIFT) &
					ERR_BUFFER_CNT_MASK);
	}
}

static void mc_ecc_dump_status(void)
{
	u32 val;
	u32 err_buff_count;
	u32 buff_ovf_intr;
	u32 corr_err_intr;
	u32 uncorr_err_intr;
	u32 ch = 0;

	do {
		val = __emc_readl(ch, EMC_ECC_STATUS);
		err_buff_count = ((val >> ERR_BUFFER_CNT_SHIFT) &
					ERR_BUFFER_CNT_MASK);

		val = __emc_readl(ch, EMC_INTSTATUS);
		buff_ovf_intr = ((val >> ECC_ERR_BUF_OVF_INT_SHIFT) &
				ECC_ERR_BUF_OVF_INT_MASK);
		corr_err_intr = ((val >>  ECC_CORR_ERR_INT_SHIFT) &
				ECC_CORR_ERR_INT_MASK);
		uncorr_err_intr = ((val >> ECC_UNCORR_ERR_INT_SHIFT) &
				ECC_UNCORR_ERR_INT_MASK);

		pr_debug("##### EMC Channel %d #####\n", ch);
		pr_debug("err_buff_count = %d,buff_ovf_intr = %d\n",
			err_buff_count, buff_ovf_intr);
		pr_debug("corr_err_intr = %d,uncorr_err_intr = %d\n",
			corr_err_intr, uncorr_err_intr);

		mc_ecc_dump_ch_logs(ch);

		ch++;

	} while (ch < MAX_CHANNELS);

	pr_debug("EMC_MCH_GLOBAL_INTSTATUS = 0x%08x\n",
				emc_readl(EMC_MCH_GLOBAL_INTSTATUS));
	pr_debug("EMC_MCH_GLOBAL_CRITICAL_INTSTATUS = 0x%08x\n",
		emc_readl(EMC_MCH_GLOBAL_CRITICAL_INTSTATUS));
	pr_debug("EMC_MCH_GLOBAL_NONCRITICAL_INTSTATUS = 0x%08x\n",
		emc_readl(EMC_MCH_GLOBAL_NONCRITICAL_INTSTATUS));
}

static void mc_increase_ecc_errors(void)
{
	uint32_t val;

	mc_writel(0x02, MC_TIMING_CONTROL_DBG);
	mc_writel(0x40040001, MC_EMEM_ARB_CFG);
	mc_writel(0x00400b39, MC_EMEM_ARB_MISC1);
	mc_writel(0x0, MC_TIMING_CONTROL_DBG);

	val = mc_readl(MC_EMEM_ARB_CFG);
	pr_debug("MC_EMEM_ARB_CFG = 0x%x\n", val);
	val = mc_readl(MC_EMEM_ARB_MISC1);
	pr_debug("MC_EMEM_ARB_MISC1 = 0x%x\n", val);
}

static int mc_handle_err_intr_ch(u32 ch)
{
	u32 val, err_buff_count;
	struct mc_ecc_err_log *pecclog = &ecc_log;
	u32 err = false;

	if (ch >= MAX_CHANNELS) {
		pr_err("!ERROR:Invalid channel number, ch :%d\n", ch);
		return -EINVAL;
	}
	/* Process the new interrupts while at bottom half, at the same time
		preserve the ch status which is read from top half
	*/
	emc_int_status[ch] |= __emc_readl(ch, EMC_INTSTATUS);
	pr_debug("ch:%d, emc_int_status:0x%08x\n", ch, emc_int_status[ch]);

	val = __emc_readl(ch, EMC_ECC_STATUS);
	err_buff_count = ((val >> ERR_BUFFER_CNT_SHIFT) &
					ERR_BUFFER_CNT_MASK);
	while (err_buff_count) {
		mc_ecc_read_log(pecclog, ch);

		mc_ecc_dump_log(pecclog);

		if (mc_check_poison(pecclog)) {
			ecc_err_pr("POISON Bit ERR\n");
			err = true;
		}
		if (mc_check_ebe(pecclog)) {
			ecc_err_pr("ECC Bit ERR\n");
			err = true;
		}
		if (mc_check_sbe(pecclog)) {
			ecc_err_pr("SBE ERR\n");
			err = true;
		}
		if (mc_check_dbe(pecclog)) {
			ecc_err_pr("DBE ERR\n");
			err = true;
		}

		if (err)
			mc_ecc_dump_regs(pecclog);

		val = __emc_readl(ch, EMC_ECC_STATUS);
		err_buff_count = ((val >> ERR_BUFFER_CNT_SHIFT) &
					ERR_BUFFER_CNT_MASK);
	}
	if (emc_int_status[ch] & (1 << ECC_ERR_BUF_OVF_INT_SHIFT))
			ecc_err_pr("BUFF OVER FLOW ERR\n");

	/* clear the channel local interrupts */
	__emc_writel(ch, emc_int_status[ch], EMC_INTSTATUS);

	return 0;
}
static void mc_handle_err_intr(void)
{
	u32 ch = 0;
	u32 ch_mask = 1;
	u32 val;
	u32 err_buff_count;

	/* Process the new interrupts while at bottom half, at the same time
		preserve the status which is read from top half
	*/
	gbl_int_status |= emc_readl(EMC_MCH_GLOBAL_INTSTATUS);

	pr_debug("gbl_int_status = 0x%08x\n", gbl_int_status);

	do {
		if (gbl_int_status & ch_mask) {
			mc_handle_err_intr_ch(ch);

			/* Again check for new errors in the channel,
				before clearing global bit */
			val = __emc_readl(ch, EMC_ECC_STATUS);
			err_buff_count = ((val >> ERR_BUFFER_CNT_SHIFT) &
					ERR_BUFFER_CNT_MASK);

			if (err_buff_count == 0) {
				/*Clear the global interrupt bit
					for that channel*/
				emc_writel((1 << ch),
					EMC_MCH_GLOBAL_INTSTATUS);
				gbl_int_status &= ~(ch_mask);
			}
		}

		gbl_int_status |= emc_readl(
				EMC_MCH_GLOBAL_INTSTATUS);

		if (!(gbl_int_status & ch_mask)) {
			/* Current channel no errors, move to next
				channel */
			ch_mask = (ch_mask << 1);
			ch++;
		}

	} while (gbl_int_status);
}

irqreturn_t tegra_ecc_error_thread(int irq, void *data)
{
	mc_handle_err_intr();

	/* Unmask the interrupt back */
	emc_writel(ecc_int_mask, EMC_INTMASK);

	return IRQ_HANDLED;
}
irqreturn_t tegra_ecc_error_hard_irq(int irq, void *data)
{
	u32 ch = 0;

	pr_debug("ECC ERR detected.\n");
	gbl_int_status = emc_readl(EMC_MCH_GLOBAL_INTSTATUS);

	if (!gbl_int_status) {
		pr_err("ECC Err unknown source (global status = 0x%08x)\n",
			     gbl_int_status);
		return IRQ_NONE;
	}

	/* prevent new interrupts until we've handled this one */
	emc_writel(0, EMC_INTMASK);

	do {
		emc_int_status[ch] = __emc_readl(ch, EMC_INTSTATUS);
		__emc_writel(ch, emc_int_status[ch], EMC_INTSTATUS);
		ch++;
	} while (ch < MAX_CHANNELS);

	emc_writel(gbl_int_status, EMC_MCH_GLOBAL_INTSTATUS);

	return IRQ_WAKE_THREAD;
}
static int mc_ecc_err_init(struct platform_device *pdev)
{
	struct resource *res;
	u32 reg_index = DT_REG_INDEX_EMC_BROADCAST;
	u32 ch = 0;
	const void *prop;
	u32 ecc_err_irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, reg_index);
	emc = devm_ioremap_resource(&pdev->dev, res);
	if (!emc) {
		dev_err(&pdev->dev, "emc map failed\n");
		return -ENOMEM;
	}
	reg_index++;
	do {
		res = platform_get_resource(pdev, IORESOURCE_MEM, reg_index);
		emc_regs[ch] = devm_ioremap_resource(&pdev->dev, res);
		if (!emc_regs[ch]) {
			dev_err(&pdev->dev, "emc_regs[%d] map failed\n",
								reg_index);
			return -ENOMEM;
		}
		reg_index++;
		ch++;
	} while (ch < MAX_CHANNELS);

	ecc_err_irq = irq_of_parse_and_map(pdev->dev.of_node, 1);
	pr_info("ecc_err_irq = %d\n", ecc_err_irq);
	if (ecc_err_irq < 0) {
		pr_err("Unable to parse/map ECC interrupt\n");
		return -EINVAL;
	}

	if (request_threaded_irq(ecc_err_irq, tegra_ecc_error_hard_irq,
					tegra_ecc_error_thread, 0,
					"ecc_irq_status", NULL)) {
		pr_err("Unable to register ecc err interrupt\n");
		return -EINVAL;
	}

	prop = of_get_property(pdev->dev.of_node, "ecc_int_mask", NULL);
	if (!prop) {
		pr_err("No int_mask prop for mcerr!\n");
		return -EINVAL;
	}
	ecc_int_mask = be32_to_cpup(prop);

	mc_ecc_dump_status();

	mc_config_ecc_log(MC_ECC_LOG_RING_MODE, MC_ECC_LOG_BUFF_DEPTH);

	mc_ecc_clear_all_intr();

	mc_ecc_dump_status();

	emc_writel(ecc_int_mask, EMC_INTMASK);

	mc_ecc_dump_intr_mask();

	mc_increase_ecc_errors();

	return 0;
}

void tegra_emcerr_init(struct platform_device *pdev)
{
	u32 mc_ecc_control = mc_readl(MC_ECC_CONTROL);

	if (mc_ecc_control & 1) {
		pr_info("dram ecc enabled-MC_ECC_CONTROL:0x%08x\n",
							mc_ecc_control);
		if (mc_ecc_err_init(pdev))
			pr_err("ecc error init failed\n");

	} else {
		pr_info("dram ecc disabled-MC_ECC_CONTROL:0x%08x\n",
							mc_ecc_control);
	}
}
