/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
 * The driver handles Error's from Control Backbone(CBB) generated due to
 * illegal accesses. When an error is reported from a NOC within CBB,
 * the driver checks ErrVld status of all three Error Logger's of that NOC.
 * It then prints debug information about failed transaction using ErrLog
 * registers of error logger which has ErrVld set. Currently, SLV, DEC,
 * TMO, SEC, UNS are the only codes which are supported by CBB.
 */

#include <asm/traps.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <soc/tegra/chip-id.h>
#include <linux/platform/tegra/tegra19x_cbb.h>

static LIST_HEAD(cbb_noc_list);
static DEFINE_RAW_SPINLOCK(cbb_noc_lock);

static struct tegra_noc_errors noc_errors[] = {
	{.errcode = "SLV",
	.src = "Target",
	.type = "Target error detected by CBB slave"
	},
	{.errcode = "DEC",
	.src = "Initiator NIU",
	.type = "Address decode error"
	},
	{.errcode = "UNS",
	.src = "Target NIU",
	.type = "Unsupported request. Not a valid transaction"
	},
	{.errcode = "DISC",			/* Not Supported by CBB */
	.src = "Power Disconnect",
	.type = "Disconnected target or domain"
	},
	{.errcode = "SEC",
	.src = "Initiator NIU or Firewall",
	.type = "Security violation. Firewall error"
	},
	{.errcode = "HIDE",			/* Not Supported by CBB */
	.src = "Firewall",
	.type = "Hidden security violation, reported as OK to initiator"
	},
	{.errcode = "TMO",
	.src = "Target NIU",
	.type = "Target time-out error"
	},
	{.errcode = "RSV",
	.src = "None",
	.type = "Reserved"
	}
};

static char *tegra_noc_opc_trantype[] = {
	"RD  - Read, Incrementing",
	"RDW - Read, Wrap",			/* Not Supported by CBB */
	"RDX - Exclusive Read",			/* Not Supported by CBB */
	"RDL - Linked Read",			/* Not Supported by CBB */
	"WR  - Write, Incrementing",
	"WRW - Write, Wrap",			/* Not Supported by CBB */
	"WRC - Exclusive Write",		/* Not Supported by CBB */
	"PRE - Preamble Sequence for Fixed Accesses"
};

static char *tegra_axi2apb_errors[] = {
	"SFIFONE - Status FIFO Not Empty interrupt",
	"SFIFOF - Status FIFO Full interrupt",
	"TIM - Timer(Timeout) interrupt",
	"SLV - SLVERR interrupt",
	"NULL",
	"ERBF - Early response buffer Full interrupt",
	"NULL",
	"RDFIFOF - Read Response FIFO Full interrupt",
	"WRFIFOF - Write Response FIFO Full interrupt",
	"CH0DFIFOF - Ch0 Data FIFO Full interrupt",
	"CH1DFIFOF - Ch1 Data FIFO Full interrupt",
	"CH2DFIFOF - Ch2 Data FIFO Full interrupt",
	"UAT - Unsupported alignment type error",
	"UBS - Unsupported burst size error",
	"UBE - Unsupported Byte Enable error",
	"UBT - Unsupported burst type error",
	"BFS - Block Firewall security error",
	"ARFS - Address Range Firewall security error",
	"CH0RFIFOF - Ch0 Request FIFO Full interrupt",
	"CH1RFIFOF - Ch1 Request FIFO Full interrupt",
	"CH2RFIFOF - Ch2 Request FIFO Full interrupt"
};

static void print_cbb_err(struct seq_file *file, const char *fmt, ...)
{
	va_list args;
	struct va_format vaf;

	va_start(args, fmt);

	if (file) {
		seq_vprintf(file, fmt, args);
	} else {
		vaf.fmt = fmt;
		vaf.va = &args;
		pr_crit("%pV", &vaf);
	}

	va_end(args);
}

static void cbbcentralnoc_parse_routeid
		(struct tegra_lookup_noc_aperture *noc_trans_info, u64 routeid)
{
	noc_trans_info->initflow = get_cbb_routeid_initflow(routeid, 23, 20);
	noc_trans_info->targflow = get_cbb_routeid_targflow(routeid, 19, 16);
	noc_trans_info->targ_subrange =	get_cbb_routeid_targsubrange
							(routeid, 15, 9);
	noc_trans_info->seqid = get_cbb_routeid_seqid(routeid, 8, 0);
}

static void bpmpnoc_parse_routeid
		(struct tegra_lookup_noc_aperture *noc_trans_info, u64 routeid)
{
	noc_trans_info->initflow = get_cbb_routeid_initflow(routeid, 20, 18);
	noc_trans_info->targflow = get_cbb_routeid_targflow(routeid, 17, 13);
	noc_trans_info->targ_subrange =	get_cbb_routeid_targsubrange
							(routeid, 12, 9);
	noc_trans_info->seqid = get_cbb_routeid_seqid(routeid, 8, 0);
}

static void aonnoc_parse_routeid
		(struct tegra_lookup_noc_aperture *noc_trans_info, u64 routeid)
{
	noc_trans_info->initflow = get_cbb_routeid_initflow(routeid, 22, 21);
	noc_trans_info->targflow = get_cbb_routeid_targflow(routeid, 20, 15);
	noc_trans_info->targ_subrange = get_cbb_routeid_targsubrange
							(routeid, 14, 9);
	noc_trans_info->seqid = get_cbb_routeid_seqid(routeid, 8, 0);
}

static void scenoc_parse_routeid
		(struct tegra_lookup_noc_aperture *noc_trans_info, u64 routeid)
{
	noc_trans_info->initflow = get_cbb_routeid_initflow(routeid, 21, 19);
	noc_trans_info->targflow = get_cbb_routeid_targflow(routeid, 18, 14);
	noc_trans_info->targ_subrange = get_cbb_routeid_targsubrange
							(routeid, 13, 9);
	noc_trans_info->seqid = get_cbb_routeid_seqid(routeid, 8, 0);
}


static void cbb_errlogger_faulten(void __iomem *addr)
{
	writel(1, addr+OFF_ERRLOGGER_0_FAULTEN_0);
	writel(1, addr+OFF_ERRLOGGER_1_FAULTEN_0);
	writel(1, addr+OFF_ERRLOGGER_2_FAULTEN_0);
}


static void cbb_errlogger_stallen(void __iomem *addr)
{
	writel(1, addr+OFF_ERRLOGGER_0_STALLEN_0);
	writel(1, addr+OFF_ERRLOGGER_1_STALLEN_0);
	writel(1, addr+OFF_ERRLOGGER_2_STALLEN_0);
}


static void cbb_errlogger_errclr(void __iomem *addr)
{
	writel(1, addr+OFF_ERRLOGGER_0_ERRCLR_0);
	writel(1, addr+OFF_ERRLOGGER_1_ERRCLR_0);
	writel(1, addr+OFF_ERRLOGGER_2_ERRCLR_0);
}


static unsigned int cbb_errlogger_errvld(void __iomem *addr)
{
	unsigned int errvld_status = 0;

	errvld_status = readl(addr+OFF_ERRLOGGER_0_ERRVLD_0);
	errvld_status |= (readl(addr+OFF_ERRLOGGER_1_ERRVLD_0) << 1);
	errvld_status |= (readl(addr+OFF_ERRLOGGER_2_ERRVLD_0) << 2);

	return errvld_status;
}


/*
 * Fetch InitlocalAddress from NOC Aperture lookup table
 * using Targflow, Targsubrange
 */
static int get_init_localaddress(
		struct tegra_lookup_noc_aperture *noc_trans_info,
		struct tegra_lookup_noc_aperture *lookup_noc_aperture,
		int max_cnt)
{
	int targ_f = 0, targ_sr = 0;
	unsigned long long init_localaddress = 0;
	int targflow = noc_trans_info->targflow;
	int targ_subrange = noc_trans_info->targ_subrange;

	for (targ_f = 0; targ_f < max_cnt; targ_f++) {
		if (lookup_noc_aperture[targ_f].targflow == targflow) {
			targ_sr = targ_f;
			do {
				if (lookup_noc_aperture[targ_sr].targ_subrange == targ_subrange) {
					init_localaddress = lookup_noc_aperture[targ_sr].init_localaddress;
					return init_localaddress;
				}
				if (targ_sr >= max_cnt)
					return 0;
				targ_sr++;
			} while (lookup_noc_aperture[targ_sr].targflow == lookup_noc_aperture[targ_sr-1].targflow);
			targ_f = targ_sr;
		}
	}

	return init_localaddress;
}


static void print_cache(struct seq_file *file, u32 cache)
{
	if ((cache & 0x3) == 0x0) {
		print_cbb_err(file, "\t  Cache\t\t\t: 0x%x -- "
				"Non-cacheable/Non-Bufferable)\n", cache);
		return;
	}
	if ((cache & 0x3) == 0x1) {
		print_cbb_err(file, "\t  Cache\t\t\t: 0x%x -- Device\n",
				cache);
		return;
	}

	switch (cache) {
	case 0x2:
		print_cbb_err(file,
		"\t  Cache\t\t\t: 0x%x -- Cacheable/Non-Bufferable\n", cache);
			break;
	case 0x3:
		print_cbb_err(file,
		"\t  Cache\t\t\t: 0x%x -- Cacheable/Bufferable\n", cache);
		break;
	default:
		print_cbb_err(file, "\t  Cache\t\t\t: 0x%x -- Cacheable\n",
				cache);
	}
}


static void print_prot(struct seq_file *file, u32 prot)
{
	char *data_str;
	char *secure_str;
	char *priv_str;

	data_str = (prot & 0x4) ? "Instruction" : "Data";
	secure_str = (prot & 0x2) ? "Non-Secure" : "Secure";
	priv_str = (prot & 0x1) ? "Privileged" : "Unprivileged";

	print_cbb_err(file, "\t  Protection\t\t: 0x%x -- %s, %s, %s Access\n",
			prot, priv_str, secure_str, data_str);
}

static unsigned int tegra_axi2apb_errstatus(void __iomem *addr)
{
	unsigned int error_status;

	error_status = readl(addr+DMAAPB_X_RAW_INTERRUPT_STATUS);
	writel(0xFFFFFFFF, addr+DMAAPB_X_RAW_INTERRUPT_STATUS);
	return error_status;
}


static void print_errlog5(struct seq_file *file,
				struct tegra_cbb_errlog_record *errlog)
{
	u8 mstr_id = 0, vqc = 0, grpsec = 0, falconsec = 0;
	u8 axi_id = 0, axcache = 0, axprot = 0, non_mod = 0;
	u32 errlog5 = errlog->errlog5;

	axcache     =   get_cbb_errlog5_axcache(errlog5);
	non_mod     =   get_cbb_errlog5_non_modify(errlog5);
	axprot      =   get_cbb_errlog5_axprot(errlog5);
	vqc         =   get_cbb_errlog5_vqc(errlog5);
	grpsec      =   get_cbb_errlog5_grpsec(errlog5);
	falconsec   =   get_cbb_errlog5_falconsec(errlog5);
	mstr_id     =   get_cbb_errlog5_mstr_id(errlog5)-1;
	axi_id      =   get_cbb_errlog5_axi_id(errlog5);

	print_cbb_err(file, "\t  Master ID\t\t: %s\n",
					errlog->tegra_cbb_master_id[mstr_id]);
	print_cbb_err(file, "\t  Non-Modify\t\t: 0x%x\n", non_mod);
	print_cbb_err(file, "\t  AXI ID\t\t: 0x%x\n", axi_id);
	print_cbb_err(file, "\t  Security Group(GRPSEC): 0x%x\n", grpsec);
	print_cache(file, axcache);
	print_prot(file, axprot);
	print_cbb_err(file, "\t  FALCONSEC\t\t: 0x%x\n", falconsec);
	print_cbb_err(file, "\t  Virtual Queuing Channel(VQC): 0x%x\n", vqc);
}


/*
 *  Fetch Base Address/InitlocalAddress from NOC aperture lookup table
 *  using TargFlow & Targ_subRange extracted from RouteId.
 *  Perform address reconstruction as below:
 *		Address = Base Address + (ErrLog3+ErrLog4)
 */
static void print_errlog3_4(struct seq_file *file, u32 errlog3, u32 errlog4,
		struct tegra_lookup_noc_aperture *noc_trans_info,
		struct tegra_lookup_noc_aperture *noc_aperture,
		int max_noc_aperture)
{
	struct resource *res = NULL;
	u64 addr = 0;

	addr = errlog4;
	addr = (addr << 32) | errlog3;

	/*
	 * if errlog4[7]="1", then it's a joker entry.
	 * joker entry is a rare phenomenon and address is not reliable.
	 * debug should be done using the routeid information alone.
	 */
	if (errlog4 & 0x80)
		print_cbb_err(file, "\t  debug using routeid alone as below"
				" address is a joker entry and not-reliable.");

	addr += get_init_localaddress(noc_trans_info, noc_aperture,
							max_noc_aperture);

	res = locate_resource(&iomem_resource, addr);
	if (res == NULL)
		print_cbb_err(file, "\t  Address\t\t: 0x%llx"
				" (unknown device)\n", addr);
	else
		print_cbb_err(file, "\t  Address\t\t: "
				"0x%llx -- %s + 0x%llx\n", addr, res->name,
				addr - res->start);
}

/*
 *  Get RouteId from ErrLog1+ErrLog2 registers and fetch values of
 *  InitFlow, TargFlow, Targ_subRange and SeqId values from RouteId
 */
static void print_errlog1_2(struct seq_file *file,
		struct tegra_cbb_errlog_record *errlog,
		struct tegra_lookup_noc_aperture *noc_trans_info)
{
	u64	routeid = 0;
	u32	seqid = 0;

	routeid = errlog->errlog2;
	routeid = (routeid<<32)|errlog->errlog1;
	print_cbb_err(file, "\t  RouteId\t\t: 0x%lx\n", routeid);
	errlog->tegra_noc_parse_routeid(noc_trans_info, routeid);

	print_cbb_err(file, "\t  InitFlow\t\t: %s\n",
		errlog->tegra_noc_routeid_initflow[noc_trans_info->initflow]);
	print_cbb_err(file, "\t  Targflow\t\t: %s\n",
		errlog->tegra_noc_routeid_targflow[noc_trans_info->targflow]);
	print_cbb_err(file, "\t  TargSubRange\t\t: %d\n",
			noc_trans_info->targ_subrange);
	print_cbb_err(file, "\t  SeqId\t\t\t: %d\n", seqid);
}

/*
 * Print transcation type, error code and description from ErrLog0 for all
 * errors. For NOC slave errors, all relevant error info is printed using
 * ErrLog0 only. But additional information is printed for errors from
 * APB slaves because for them:
 *  - All errors are logged as SLV(slave) errors in errlog0 due to APB having
 *    only single bit pslverr to report all errors.
 *  - Exact cause is printed by reading DMAAPB_X_RAW_INTERRUPT_STATUS register.
 *  - The driver prints information showing AXI2APB bridge and exact error
 *    only if there is error in any AXI2APB slave.
 *  - There is still no way to disambiguate a DEC error from SLV error type.
 */
static void print_errlog0(struct seq_file *file,
		struct tegra_cbb_errlog_record *errlog)
{
	struct tegra_noc_packet_header hdr;

	hdr.lock    = errlog->errlog0 & 0x1;
	hdr.opc     = get_cbb_errlog0_trans_opc(errlog->errlog0);
	hdr.errcode = get_cbb_errlog0_code(errlog->errlog0);
	hdr.len1    = get_cbb_errlog0_src(errlog->errlog0);
	hdr.format  = (errlog->errlog0>>31);

	print_cbb_err(file, "\t  Transaction Type\t: %s\n",
			tegra_noc_opc_trantype[hdr.opc]);
	print_cbb_err(file, "\t  Error Code\t\t: %s\n",
			noc_errors[hdr.errcode].errcode);
	print_cbb_err(file, "\t  Error Source\t\t: %s\n",
			noc_errors[hdr.errcode].src);
	print_cbb_err(file, "\t  Error Description\t: %s\n",
			noc_errors[hdr.errcode].type);

	if (!strcmp(noc_errors[hdr.errcode].errcode, "SLV")) {
		int i = 0, j = 0;
		int max_axi2apb_err = ARRAY_SIZE(tegra_axi2apb_errors);
		u32 bus_status = 0;

		/* For all SLV errors, read DMAAPB_X_RAW_INTERRUPT_STATUS
		 * register to get error status for all AXI2APB bridges and
		 * print only if a bit set for any bridge due to error in
		 * a APB slave. For other NOC slaves, no bit will be set.
		 * So, below line won't get printed.
		 */

		for (i = 0; i < errlog->apb_bridge_cnt; i++) {
			bus_status = tegra_axi2apb_errstatus(
				(void __iomem *)errlog->axi2abp_bases[i]);

			if (bus_status) {
				for(j = 0; j < max_axi2apb_err; j++) {
					if ( bus_status & (1<<j) )
						print_cbb_err(file, "\t  "
						"AXI2APB_%d bridge error: %s"
						, i, tegra_axi2apb_errors[j]);
				}
			}
		}
	}
	print_cbb_err(file, "\t  Packet header Lock\t: %d\n", hdr.lock);
	print_cbb_err(file, "\t  Packet header Len1\t: %d\n", hdr.len1);
	if (hdr.format)
		print_cbb_err(file, "\t  NOC protocol version\t: %s\n",
				"version >= 2.7");
	else
		print_cbb_err(file, "\t  NOC protocol version\t: %s\n",
				"version < 2.7");
}


/*
 * Print debug information about failed transaction using
 * ErrLog registers of error loggger having ErrVld set
 */
static void print_errloggerX_info(
			struct seq_file *file,
			struct tegra_cbb_errlog_record *errlog, int errloggerX)
{
	struct tegra_lookup_noc_aperture noc_trans_info = {0,};

	print_cbb_err(file, "\tError Logger\t\t: %d\n", errloggerX);
	if (errloggerX == 0) {
		errlog->errlog0 = readl(errlog->vaddr+OFF_ERRLOGGER_0_ERRLOG0_0);
		errlog->errlog1 = readl(errlog->vaddr+OFF_ERRLOGGER_0_ERRLOG1_0);
		errlog->errlog2 = readl(errlog->vaddr+OFF_ERRLOGGER_0_RESERVED_00_0);
		errlog->errlog3 = readl(errlog->vaddr+OFF_ERRLOGGER_0_ERRLOG3_0);
		errlog->errlog4 = readl(errlog->vaddr+OFF_ERRLOGGER_0_ERRLOG4_0);
		errlog->errlog5 = readl(errlog->vaddr+OFF_ERRLOGGER_0_ERRLOG5_0);
	} else if (errloggerX == 1) {
		errlog->errlog0 = readl(errlog->vaddr+OFF_ERRLOGGER_1_ERRLOG0_0);
		errlog->errlog1 = readl(errlog->vaddr+OFF_ERRLOGGER_1_ERRLOG1_0);
		errlog->errlog2 = readl(errlog->vaddr+OFF_ERRLOGGER_1_RESERVED_00_0);
		errlog->errlog3 = readl(errlog->vaddr+OFF_ERRLOGGER_1_ERRLOG3_0);
		errlog->errlog4 = readl(errlog->vaddr+OFF_ERRLOGGER_1_ERRLOG4_0);
		errlog->errlog5 = readl(errlog->vaddr+OFF_ERRLOGGER_1_ERRLOG5_0);
	} else if (errloggerX == 2) {
		errlog->errlog0 = readl(errlog->vaddr+OFF_ERRLOGGER_2_ERRLOG0_0);
		errlog->errlog1 = readl(errlog->vaddr+OFF_ERRLOGGER_2_ERRLOG1_0);
		errlog->errlog2 = readl(errlog->vaddr+OFF_ERRLOGGER_2_RESERVED_00_0);
		errlog->errlog3 = readl(errlog->vaddr+OFF_ERRLOGGER_2_ERRLOG3_0);
		errlog->errlog4 = readl(errlog->vaddr+OFF_ERRLOGGER_2_ERRLOG4_0);
		errlog->errlog5 = readl(errlog->vaddr+OFF_ERRLOGGER_2_ERRLOG5_0);
	}

	print_cbb_err(file, "\tErrLog0\t\t\t: 0x%x\n", errlog->errlog0);
	print_errlog0(file, errlog);

	print_cbb_err(file, "\tErrLog1\t\t\t: 0x%x\n", errlog->errlog1);
	print_cbb_err(file, "\tErrLog2\t\t\t: 0x%x\n", errlog->errlog2);
	print_errlog1_2(file, errlog, &noc_trans_info);

	print_cbb_err(file, "\tErrLog3\t\t\t: 0x%x\n", errlog->errlog3);
	print_cbb_err(file, "\tErrLog4\t\t\t: 0x%x\n", errlog->errlog4);
	print_errlog3_4(file, errlog->errlog3, errlog->errlog4,
				&noc_trans_info, errlog->noc_aperture,
				errlog->max_noc_aperture);

	print_cbb_err(file, "\tErrLog5\t\t\t: 0x%x\n", errlog->errlog5);
	print_errlog5(file, errlog);
}


static void print_errlog(struct seq_file *file,
			struct tegra_cbb_errlog_record *errlog,
			int errvld_status)
{
	pr_crit("**************************************\n");
	pr_crit("* For more Internal Decode Help\n");
	pr_crit("*     http://nv/cbberr\n");
	pr_crit("* NVIDIA userID is required to access\n");
	pr_crit("**************************************\n");
	pr_crit("CPU:%d, Error:%s\n", smp_processor_id(), errlog->name);

	if (errvld_status & 0x1)
		print_errloggerX_info(file, errlog, 0);
	else if (errvld_status & 0x2)
		print_errloggerX_info(file, errlog, 1);
	else if (errvld_status & 0x4)
		print_errloggerX_info(file, errlog, 2);

	errlog->errclr(errlog->vaddr);
	print_cbb_err(file, "\t**************************************\n");
}


static int cbb_serr_callback(struct pt_regs *regs, int reason,
				unsigned int esr, void *priv)
{
	unsigned int errvld_status = 0;
	int retval = 1;
	struct tegra_cbb_errlog_record *errlog = priv;

	errvld_status = errlog->errvld(errlog->vaddr);

	if (errvld_status) {
		print_errlog(NULL, errlog, errvld_status);
		retval = 0;
	}

	return retval;
}


static struct tegra_cbb_noc_data tegra194_cbb_central_noc_data = {
	.name	= "CBB-NOC",
	.errvld = cbb_errlogger_errvld,
	.faulten = cbb_errlogger_faulten,
	.stallen = cbb_errlogger_stallen,
	.errclr	= cbb_errlogger_errclr,
	.tegra_cbb_master_id = t194_master_id,
	.noc_aperture = t194_cbbcentralnoc_aperture_lookup,
	.max_noc_aperture = ARRAY_SIZE(t194_cbbcentralnoc_aperture_lookup),
	.tegra_noc_routeid_initflow = t194_cbbcentralnoc_routeid_initflow,
	.tegra_noc_routeid_targflow = t194_cbbcentralnoc_routeid_targflow,
	.tegra_noc_parse_routeid = cbbcentralnoc_parse_routeid
};

static struct tegra_cbb_noc_data tegra194_aon_noc_data = {
	.name	= "AON-NOC",
	.errvld = cbb_errlogger_errvld,
	.faulten = cbb_errlogger_faulten,
	.stallen = cbb_errlogger_stallen,
	.errclr	= cbb_errlogger_errclr,
	.tegra_cbb_master_id = t194_master_id,
	.noc_aperture = t194_aonnoc_aperture_lookup,
	.max_noc_aperture = ARRAY_SIZE(t194_aonnoc_aperture_lookup),
	.tegra_noc_routeid_initflow = t194_aonnoc_routeid_initflow,
	.tegra_noc_routeid_targflow = t194_aonnoc_routeid_targflow,
	.tegra_noc_parse_routeid = aonnoc_parse_routeid
};

static struct tegra_cbb_noc_data tegra194_bpmp_noc_data = {
	.name   = "BPMP-NOC",
	.errvld = cbb_errlogger_errvld,
	.faulten = cbb_errlogger_faulten,
	.stallen = cbb_errlogger_stallen,
	.errclr = cbb_errlogger_errclr,
	.tegra_cbb_master_id = t194_master_id,
	.noc_aperture = t194_bpmpnoc_aperture_lookup,
	.max_noc_aperture = ARRAY_SIZE(t194_bpmpnoc_aperture_lookup),
	.tegra_noc_routeid_initflow = t194_bpmpnoc_routeid_initflow,
	.tegra_noc_routeid_targflow = t194_bpmpnoc_routeid_targflow,
	.tegra_noc_parse_routeid = bpmpnoc_parse_routeid
};

static struct tegra_cbb_noc_data tegra194_rce_noc_data = {
	.name   = "RCE-NOC",
	.errvld = cbb_errlogger_errvld,
	.faulten = cbb_errlogger_faulten,
	.stallen = cbb_errlogger_stallen,
	.errclr = cbb_errlogger_errclr,
	.tegra_cbb_master_id = t194_master_id,
	.noc_aperture = t194_scenoc_aperture_lookup,
	.max_noc_aperture = ARRAY_SIZE(t194_scenoc_aperture_lookup),
	.tegra_noc_routeid_initflow = t194_scenoc_routeid_initflow,
	.tegra_noc_routeid_targflow = t194_scenoc_routeid_targflow,
	.tegra_noc_parse_routeid = scenoc_parse_routeid
};

static struct tegra_cbb_noc_data tegra194_sce_noc_data = {
	.name   = "SCE-NOC",
	.errvld = cbb_errlogger_errvld,
	.faulten = cbb_errlogger_faulten,
	.stallen = cbb_errlogger_stallen,
	.errclr = cbb_errlogger_errclr,
	.tegra_cbb_master_id = t194_master_id,
	.noc_aperture = t194_scenoc_aperture_lookup,
	.max_noc_aperture = ARRAY_SIZE(t194_scenoc_aperture_lookup),
	.tegra_noc_routeid_initflow = t194_scenoc_routeid_initflow,
	.tegra_noc_routeid_targflow = t194_scenoc_routeid_targflow,
	.tegra_noc_parse_routeid = scenoc_parse_routeid
};

static const struct of_device_id axi2apb_match[] = {
	{ .compatible = "nvidia,tegra194-AXI2APB-bridge", },
	{},
};

static struct of_device_id tegra_cbb_match[] = {
	{.compatible    = "nvidia,tegra194-CBBNOCAXI-bridge",
		.data = &tegra194_cbb_central_noc_data},
	{.compatible    = "nvidia,tegra194-CBBNOCAON-bridge",
		.data = &tegra194_aon_noc_data},
	{.compatible    = "nvidia,tegra194-CBBNOCBPMP-bridge",
		.data = &tegra194_bpmp_noc_data},
	{.compatible    = "nvidia,tegra194-CBBNOCRCE-bridge",
		.data = &tegra194_rce_noc_data},
	{.compatible    = "nvidia,tegra194-CBBNOCSCE-bridge",
		.data = &tegra194_sce_noc_data},
	{},
};

MODULE_DEVICE_TABLE(of, tegra_cbb_match);


#ifdef CONFIG_DEBUG_FS
static DEFINE_MUTEX(cbb_err_mutex);
static int created_root;


static int cbb_err_show(struct seq_file *file, void *data)
{
	struct tegra_cbb_errlog_record *errlog;
	unsigned int errvld_status = 0;

	mutex_lock(&cbb_err_mutex);

	list_for_each_entry(errlog, &cbb_noc_list, node) {

		errvld_status = errlog->errvld(errlog->vaddr);

		if (errvld_status) {
			print_errlog(file, errlog, errvld_status);
		}
	}

	mutex_unlock(&cbb_err_mutex);
	return 0;
}


static int cbb_err_open(struct inode *inode, struct file *file)
{
	return single_open(file, cbb_err_show, inode->i_private);
}


static const struct file_operations cbb_err_fops = {
	.open = cbb_err_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};


static int cbb_noc_dbgfs_init(void)
{
	struct dentry *d;
	if (!created_root) {
		d = debugfs_create_file("tegra_cbb_err",
				S_IRUGO, NULL, NULL, &cbb_err_fops);
		if (IS_ERR_OR_NULL(d)) {
			pr_err("%s: could not create 'tegra_cbb_err' node\n",
					__func__);
			return PTR_ERR(d);
		}
		created_root = true;
	}
	return 0;
}

#else
static int cbb_noc_dbgfs_init(void) { return 0; }
#endif

/*
 * Handler for CBB errors from masters other than CCPLEX
 */
static irqreturn_t tegra_cbb_error_isr(int irq, void *dev_id)
{
	struct tegra_cbb_errlog_record *errlog;
	unsigned int errvld_status = 0;
	unsigned long flags;

	raw_spin_lock_irqsave(&cbb_noc_lock, flags);

	list_for_each_entry(errlog, &cbb_noc_list, node) {
		errvld_status = errlog->errvld(errlog->vaddr);

		if (errvld_status && ((irq == errlog->noc_secure_irq)
				|| (irq == errlog->noc_nonsecure_irq))) {
			print_cbb_err(NULL, "CPU:%d, Error:%s@0x%llx, irq=%d\n",
			smp_processor_id(), errlog->name, errlog->start, irq);

			print_errlog(NULL, errlog, errvld_status);
		}
	}
	raw_spin_unlock_irqrestore(&cbb_noc_lock, flags);

	return IRQ_HANDLED;
}

/*
 * Register handler for CBB_NONSECURE & CBB_SECURE interrupts due to
 * CBB errors from masters other than CCPLEX
 */
static int cbb_register_isr(struct platform_device *pdev,
				struct tegra_cbb_errlog_record *errlog)
{
	int err = 0;

	errlog->noc_nonsecure_irq = platform_get_irq(pdev, 0);
	if (errlog->noc_nonsecure_irq <= 0) {
		dev_err(&pdev->dev, "can't get irq (%d)\n",
					errlog->noc_nonsecure_irq);
		err = -ENOENT;
		goto isr_err;
	}

	errlog->noc_secure_irq = platform_get_irq(pdev, 1);
	if (errlog->noc_secure_irq <= 0) {
		dev_err(&pdev->dev, "can't get irq (%d)\n",
						errlog->noc_secure_irq);
		err = -ENOENT;
		goto isr_err;
	}
	dev_info(&pdev->dev, "noc_secure_irq = %d, noc_nonsecure_irq = %d>\n",
			errlog->noc_secure_irq, errlog->noc_nonsecure_irq);

	if (request_irq(errlog->noc_nonsecure_irq, tegra_cbb_error_isr, 0,
				"noc_nonsecure_irq", pdev)) {
		dev_err(&pdev->dev, "%s: Unable to register (%d) interrupt\n",
				__func__, errlog->noc_nonsecure_irq);
		goto isr_err;
	}
	if (request_irq(errlog->noc_secure_irq, tegra_cbb_error_isr, 0,
				"noc_secure_irq", pdev)) {
		dev_err(&pdev->dev, "%s: Unable to register (%d) interrupt\n",
				__func__, errlog->noc_secure_irq);
		goto isr_err_free_irq;
	}
	return 0;

isr_err_free_irq:
	free_irq(errlog->noc_nonsecure_irq, pdev);
isr_err:
	return err;
}

static int tegra_cbb_probe(struct platform_device *pdev)
{
	struct resource *res_base;
	struct tegra_cbb_errlog_record *errlog;
	const struct tegra_cbb_noc_data *bdata;
	struct serr_hook *callback;
	unsigned long flags;
	struct device_node *np;
	int i = 0;
	int err = 0;

	/*
	 * CBB don't exist on the simulator
	 */
	if (tegra_cpu_is_asim())
		return 0;

	bdata = of_device_get_match_data(&pdev->dev);
	if (!bdata) {
		dev_err(&pdev->dev, "No device match found\n");
		return -EINVAL;
	}

	res_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_base) {
		dev_err(&pdev->dev, "Could not find base address");
		return -ENOENT;
	}

	err = cbb_noc_dbgfs_init();
	if (err)
		return err;

	errlog = devm_kzalloc(&pdev->dev, sizeof(*errlog), GFP_KERNEL);
	if (!errlog)
		return -ENOMEM;

	errlog->start = res_base->start;
	errlog->vaddr = devm_ioremap_resource(&pdev->dev, res_base);
	if (IS_ERR(errlog->vaddr))
		return -EPERM;

	np = of_find_matching_node(NULL, axi2apb_match);
	if (!np) {
		dev_info(&pdev->dev, "No match found for axi2apb\n");
		return -ENOENT;
	}
	errlog->apb_bridge_cnt =
		(of_property_count_elems_of_size(np, "reg", sizeof(u32)))/4;

	errlog->axi2abp_bases = devm_kzalloc(&pdev->dev,
			sizeof(u64)*errlog->apb_bridge_cnt, GFP_KERNEL);
	if (errlog->axi2abp_bases == NULL)
		return -ENOMEM;

	for (i = 0; i < errlog->apb_bridge_cnt; i++) {
		void __iomem *base = of_iomap(np, i);

		if (!base) {
			dev_err(&pdev->dev,
					"failed to map axi2apb range\n");
			return -ENOENT;
		}
		errlog->axi2abp_bases[i] = (u64)base;
	}

	errlog->name      = bdata->name;
	errlog->errvld    = bdata->errvld;
	errlog->errclr    = bdata->errclr;
	errlog->faulten   = bdata->faulten;
	errlog->stallen	  = bdata->stallen;
	errlog->noc_aperture = bdata->noc_aperture;
	errlog->max_noc_aperture = bdata->max_noc_aperture;
	errlog->tegra_noc_routeid_initflow = bdata->tegra_noc_routeid_initflow;
	errlog->tegra_noc_routeid_targflow = bdata->tegra_noc_routeid_targflow;
	errlog->tegra_noc_parse_routeid = bdata->tegra_noc_parse_routeid;
	errlog->tegra_cbb_master_id = bdata->tegra_cbb_master_id;

	callback = devm_kzalloc(&pdev->dev, sizeof(*callback), GFP_KERNEL);
	callback->fn = cbb_serr_callback;
	callback->priv = errlog;
	errlog->callback = callback;

	raw_spin_lock_irqsave(&cbb_noc_lock, flags);
	list_add(&errlog->node, &cbb_noc_list);
	raw_spin_unlock_irqrestore(&cbb_noc_lock, flags);

	/* set “StallEn=1” to enable queuing of error packets till
	 * first is served & cleared
	 */
	errlog->stallen(errlog->vaddr);

	/* register handler for CBB errors due to CCPLEX master*/
	register_serr_hook(callback);

	/* register handler for CBB errors due to masters other than CCPLEX*/
	err = cbb_register_isr(pdev, errlog);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to register CBB Interrupt ISR");
		return err;
	}

	/* set “FaultEn=1” to enable error reporting signal “Fault” */
	errlog->faulten(errlog->vaddr);

	dev_info(&pdev->dev, "cbb NOC probed OK\n");

	return 0;
}


static int tegra_cbb_remove(struct platform_device *pdev)
{
	struct resource *res_base;
	struct tegra_cbb_errlog_record *errlog;
	unsigned long flags;

	res_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_base)
		return 0;

	raw_spin_lock_irqsave(&cbb_noc_lock, flags);
	list_for_each_entry(errlog, &cbb_noc_list, node) {
		if (errlog->start == res_base->start) {
			unregister_serr_hook(errlog->callback);
			list_del(&errlog->node);
			break;
		}
	}
	raw_spin_unlock_irqrestore(&cbb_noc_lock, flags);

	return 0;
}


static struct platform_driver tegra_cbb_driver = {
	.probe          = tegra_cbb_probe,
	.remove         = tegra_cbb_remove,
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "tegra-cbb",
		.of_match_table = of_match_ptr(tegra_cbb_match),
	},
};

static int __init tegra_cbb_init(void)
{
	return platform_driver_register(&tegra_cbb_driver);
}

static void __exit tegra_cbb_exit(void)
{
	platform_driver_unregister(&tegra_cbb_driver);
}

arch_initcall(tegra_cbb_init);
module_exit(tegra_cbb_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SError handler for NOC errors within Control Backbone");
