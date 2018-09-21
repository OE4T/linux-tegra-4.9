/*
 * Copyright (c) 2018 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/io.h>

struct arm_smmu_device {
	void __iomem *base;
	int size;

	u32 num_context_banks;
	unsigned long pgshift;

	struct debugfs_regset32 *regset;
	struct debugfs_regset32 *perf_regset;
	struct dentry *debugfs_root;
	struct dentry *cb_root;
	struct dentry *masters_root;
};

static struct arm_smmu_device *smmu_handle;

/* Identification registers */
#define ARM_SMMU_GR0_sCR0		0x0
#define ARM_SMMU_GR0_ID0		0x20
#define ARM_SMMU_GR0_ID1		0x24
#define ARM_SMMU_GR0_ID2		0x28
#define ARM_SMMU_GR0_ID3		0x2c
#define ARM_SMMU_GR0_ID4		0x30
#define ARM_SMMU_GR0_ID5		0x34
#define ARM_SMMU_GR0_ID6		0x38
#define ARM_SMMU_GR0_ID7		0x3c
#define ARM_SMMU_GR0_sGFSR		0x48
#define ARM_SMMU_GR0_sGFSYNR0		0x50
#define ARM_SMMU_GR0_sGFSYNR1		0x54
#define ARM_SMMU_GR0_sGFSYNR2		0x58
#define ARM_SMMU_GR0_nsCR0		0x400
#define ARM_SMMU_GR0_nsGFSR		0x448
#define ARM_SMMU_GR0_nsGFSYNR0		0x450
#define ARM_SMMU_GR0_nsGFSYNR1		0x454
#define ARM_SMMU_GR0_nsGFSYNR2		0x458
#define ARM_SMMU_GR0_PIDR0		0xfe0
#define ARM_SMMU_GR0_PIDR1		0xfe4
#define ARM_SMMU_GR0_PIDR2		0xfe8

#define ARM_SMMU_GR0_S2CR(n)		(0xc00 + ((n) << 2))

/* Stream mapping registers */
#define ARM_SMMU_GR0_SMR(n)		(0x800 + ((n) << 2))

/* Context bank attribute registers */
#define ARM_SMMU_GR1_CBAR(n)		(0x0 + ((n) << 2))
#define ARM_SMMU_GR1_CBA2R(n)		(0x800 + ((n) << 2))

/* Global TLB invalidation */
#define ARM_SMMU_GR0_STLBIALL		0x60
#define ARM_SMMU_GR0_TLBIVMID		0x64
#define ARM_SMMU_GR0_TLBIALLNSNH	0x68
#define ARM_SMMU_GR0_TLBIALLH		0x6c
#define ARM_SMMU_GR0_sTLBGSYNC		0x70
#define ARM_SMMU_GR0_sTLBGSTATUS	0x74
#define ARM_SMMU_GR0_nsTLBGSYNC		0x470
#define ARM_SMMU_GR0_nsTLBGSTATUS	0x474

/* Perf Monitor registers */
#define ARM_SMMU_GNSR0_PMCNTENSET_0	0xc00
#define ARM_SMMU_GNSR0_PMCNTENCLR_0	0xc20
#define ARM_SMMU_GNSR0_PMINTENSET_0	0xc40
#define ARM_SMMU_GNSR0_PMINTENCLR_0	0xc60
#define ARM_SMMU_GNSR0_PMOVSCLR_0	0xc80
#define ARM_SMMU_GNSR0_PMOVSSET_0	0xcc0
#define ARM_SMMU_GNSR0_PMCFGR_0		0xe00
#define ARM_SMMU_GNSR0_PMCR_0		0xe04
#define ARM_SMMU_GNSR0_PMCEID0_0	0xe20
#define ARM_SMMU_GNSR0_PMAUTHSTATUS_0	0xfb8
#define ARM_SMMU_GNSR0_PMDEVTYPE_0	0xfcc

#define ARM_SMMU_GNSR0_PMEVTYPER(n)	(0x400 + ((n) << 2))
#define ARM_SMMU_GNSR0_PMEVCNTR(n)	(0x0 + ((n) << 2))
#define ARM_SMMU_GNSR0_PMCGCR(n)	(0x800 + ((n) << 2))
#define ARM_SMMU_GNSR0_PMCGSMR(n)	(0xa00 + ((n) << 2))

/* Translation context bank */
#define ARM_SMMU_CB_BASE(smmu)		((smmu)->base + ((smmu)->size >> 1))
#define ARM_SMMU_CB(smmu, n)		((n) * (1 << (smmu)->pgshift))

#define ARM_SMMU_CB_SCTLR		0x0
#define ARM_SMMU_CB_RESUME		0x8
#define ARM_SMMU_CB_TTBCR2		0x10
#define ARM_SMMU_CB_TTBR0_LO		0x20
#define ARM_SMMU_CB_TTBR0_HI		0x24
#define ARM_SMMU_CB_TTBCR		0x30
#define ARM_SMMU_CB_S1_MAIR0		0x38
#define ARM_SMMU_CB_FSR			0x58
#define ARM_SMMU_CB_FAR_LO		0x60
#define ARM_SMMU_CB_FAR_HI		0x64
#define ARM_SMMU_CB_FSYNR0		0x68
#define ARM_SMMU_CB_S1_TLBIASID		0x610
#define ARM_SMMU_CB_S1_TLBIVA		0x600
#define ARM_SMMU_CB_S1_TLBIVAL		0x620
#define ARM_SMMU_CB_S2_TLBIIPAS2	0x630
#define ARM_SMMU_CB_S2_TLBIIPAS2L	0x638
#define ARM_SMMU_CB_TLBSYNC		0x7f0
#define ARM_SMMU_CB_TLBSTATUS		0x7f4

/* Counter group registers */
#define PMCG_SIZE	32
/* Event Counter registers */
#define PMEV_SIZE	8

#define defreg(_name)				\
	{					\
		.name = __stringify(_name),	\
		.offset = ARM_SMMU_ ## _name,	\
	}
#define defreg_gr0(_name) defreg(GR0_ ## _name)

static const struct debugfs_reg32 arm_smmu_gr0_regs[] = {
	defreg_gr0(sCR0),
	defreg_gr0(ID0),
	defreg_gr0(ID1),
	defreg_gr0(ID2),
	defreg_gr0(sGFSR),
	defreg_gr0(sGFSYNR0),
	defreg_gr0(sGFSYNR1),
	defreg_gr0(sTLBGSTATUS),
	defreg_gr0(nsCR0),
	defreg_gr0(nsGFSR),
	defreg_gr0(nsGFSYNR0),
	defreg_gr0(nsGFSYNR1),
	defreg_gr0(nsTLBGSTATUS),
	defreg_gr0(PIDR2),
};

#define defreg_gnsr0(_name) defreg(GNSR0_ ## _name)

static const struct debugfs_reg32 arm_smmu_gnsr0_regs[] = {
	defreg_gnsr0(PMCNTENSET_0),
	defreg_gnsr0(PMCNTENCLR_0),
	defreg_gnsr0(PMINTENSET_0),
	defreg_gnsr0(PMINTENCLR_0),
	defreg_gnsr0(PMOVSCLR_0),
	defreg_gnsr0(PMOVSSET_0),
	defreg_gnsr0(PMCFGR_0),
	defreg_gnsr0(PMCR_0),
	defreg_gnsr0(PMCEID0_0),
	defreg_gnsr0(PMAUTHSTATUS_0),
	defreg_gnsr0(PMDEVTYPE_0)
};

#define defreg_cb(_name)			\
	{					\
		.name = __stringify(_name),	\
		.offset = ARM_SMMU_CB_ ## _name,\
	}

static const struct debugfs_reg32 arm_smmu_cb_regs[] = {
	defreg_cb(SCTLR),
	defreg_cb(TTBCR2),
	defreg_cb(TTBR0_LO),
	defreg_cb(TTBR0_HI),
	defreg_cb(TTBCR),
	defreg_cb(S1_MAIR0),
	defreg_cb(FSR),
	defreg_cb(FAR_LO),
	defreg_cb(FAR_HI),
	defreg_cb(FSYNR0),
};

static void debugfs_create_smmu_cb(struct arm_smmu_device *smmu, u8 cbndx)
{
	struct dentry *dent;
	char name[] = "cb000";
	struct debugfs_regset32	*cb;

	sprintf(name, "cb%03d", cbndx);
	dent = debugfs_create_dir(name, smmu->cb_root);
	if (!dent)
		return;

	cb = smmu->regset + 1 + cbndx;
	cb->regs = arm_smmu_cb_regs;
	cb->nregs = ARRAY_SIZE(arm_smmu_cb_regs);
	cb->base = smmu->base + (smmu->size >> 1) +
		cbndx * (1 << smmu->pgshift);
	debugfs_create_regset32("regdump", S_IRUGO, dent, cb);
}

static int smmu_reg32_debugfs_set(void *data, u64 val)
{
	struct debugfs_reg32 *regs = (struct debugfs_reg32 *)data;

	writel(val, (smmu_handle->base + regs->offset));
	return 0;
}

static int smmu_reg32_debugfs_get(void *data, u64 *val)
{
	struct debugfs_reg32 *regs = (struct debugfs_reg32 *)data;

	*val = readl(smmu_handle->base + regs->offset);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(smmu_reg32_debugfs_fops,
			smmu_reg32_debugfs_get,
			smmu_reg32_debugfs_set, "%08llx\n");

static int smmu_perf_regset_debugfs_set(void *data, u64 val)
{
	struct debugfs_reg32 *regs = (struct debugfs_reg32 *)data;

	writel(val, (smmu_handle->perf_regset->base + regs->offset));
	return 0;
}

static int smmu_perf_regset_debugfs_get(void *data, u64 *val)
{
	struct debugfs_reg32 *regs = (struct debugfs_reg32 *)data;

	*val = readl(smmu_handle->perf_regset->base + regs->offset);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(smmu_perf_regset_debugfs_fops,
			smmu_perf_regset_debugfs_get,
			smmu_perf_regset_debugfs_set, "%08llx\n");

static void arm_smmu_debugfs_delete(struct arm_smmu_device *smmu)
{
	int i;

	if (smmu->regset) {
		const struct debugfs_reg32 *regs = smmu->regset->regs;

		regs += ARRAY_SIZE(arm_smmu_gr0_regs);
		for (i = 0; i < 4 * smmu->num_context_banks; i++)
			kfree(regs[i].name);

		kfree(smmu->regset);
	}

	if (smmu->perf_regset) {
		const struct debugfs_reg32 *regs = smmu->perf_regset->regs;

		i = ARRAY_SIZE(arm_smmu_gnsr0_regs);
		for (; i < smmu->perf_regset->nregs ; i++)
			kfree(regs[i].name);

		kfree(smmu->perf_regset);
		smmu->perf_regset = NULL;
	}

	debugfs_remove_recursive(smmu->debugfs_root);
}

static int arm_smmu_debugfs_create(struct arm_smmu_device *smmu)
{
	int i;
	struct debugfs_reg32 *regs;
	size_t bytes;
	struct dentry *dent_gr, *dent_gnsr;

	smmu->debugfs_root = debugfs_create_dir("smmu_regs", NULL);
	if (!smmu->debugfs_root)
		return -1;

	dent_gr = debugfs_create_dir("gr", smmu->debugfs_root);
	if (!dent_gr)
		goto err_out;

	dent_gnsr = debugfs_create_dir("gnsr", smmu->debugfs_root);
	if (!dent_gnsr)
		goto err_out;

	smmu->masters_root = debugfs_create_dir("masters", smmu->debugfs_root);
	if (!smmu->masters_root)
		goto err_out;

	smmu->cb_root = debugfs_create_dir("context_banks", smmu->debugfs_root);
	if (!smmu->cb_root)
		goto err_out;

	bytes = (smmu->num_context_banks + 1) * sizeof(*smmu->regset);
	bytes += ARRAY_SIZE(arm_smmu_gr0_regs) * sizeof(*regs);
	bytes += 4 * smmu->num_context_banks * sizeof(*regs);
	smmu->regset = kzalloc(bytes, GFP_KERNEL);
	if (!smmu->regset)
		goto err_out;

	smmu->regset->base = smmu->base;
	smmu->regset->nregs = ARRAY_SIZE(arm_smmu_gr0_regs) +
		4 * smmu->num_context_banks;
	smmu->regset->regs = (struct debugfs_reg32 *)(smmu->regset +
						smmu->num_context_banks + 1);
	regs = (struct debugfs_reg32 *)smmu->regset->regs;
	for (i = 0; i < ARRAY_SIZE(arm_smmu_gr0_regs); i++) {
		regs->name = arm_smmu_gr0_regs[i].name;
		regs->offset = arm_smmu_gr0_regs[i].offset;
		regs++;
	}

	for (i = 0; i < smmu->num_context_banks; i++) {
		regs->name = kasprintf(GFP_KERNEL, "GR0_SMR%03d", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GR0_SMR(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GR0_S2CR%03d", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GR0_S2CR(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GR1_CBAR%03d", i);
		if (!regs->name)
			goto err_out;
		regs->offset = (1 << smmu->pgshift) + ARM_SMMU_GR1_CBAR(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GR1_CBA2R%03d", i);
		if (!regs->name)
			goto err_out;
		regs->offset = (1 << smmu->pgshift) + ARM_SMMU_GR1_CBA2R(i);
		regs++;
	}

	regs = (struct debugfs_reg32 *)smmu->regset->regs;
	for (i = 0; i < smmu->regset->nregs; i++) {
		debugfs_create_file(regs->name, S_IRUGO | S_IWUSR,
				dent_gr, regs, &smmu_reg32_debugfs_fops);
		regs++;
	}

	debugfs_create_regset32("regdump", S_IRUGO, smmu->debugfs_root,
				smmu->regset);

	bytes = sizeof(*smmu->perf_regset);
	bytes += ARRAY_SIZE(arm_smmu_gnsr0_regs) * sizeof(*regs);
	/*
	 * Account the number of bytes for two sets of
	 * counter group registers
	 */
	bytes += 2 * PMCG_SIZE * sizeof(*regs);
	/*
	 * Account the number of bytes for two sets of
	 * event counter registers
	 */
	bytes += 2 * PMEV_SIZE * sizeof(*regs);

	/* Allocate memory for Perf Monitor registers */
	smmu->perf_regset =  kzalloc(bytes, GFP_KERNEL);
	if (!smmu->perf_regset)
		goto err_out;

	/*
	 * perf_regset base address is placed at offset (3 * smmu_pagesize)
	 * from smmu->base address
	 */
	smmu->perf_regset->base = smmu->base + 3 * (1 << smmu->pgshift);
	smmu->perf_regset->nregs = ARRAY_SIZE(arm_smmu_gnsr0_regs) +
		2 * PMCG_SIZE + 2 * PMEV_SIZE;
	smmu->perf_regset->regs =
		(struct debugfs_reg32 *)(smmu->perf_regset + 1);

	regs = (struct debugfs_reg32 *)smmu->perf_regset->regs;

	for (i = 0; i < ARRAY_SIZE(arm_smmu_gnsr0_regs); i++) {
		regs->name = arm_smmu_gnsr0_regs[i].name;
		regs->offset = arm_smmu_gnsr0_regs[i].offset;
		regs++;
	}

	for (i = 0; i < PMEV_SIZE; i++) {
		regs->name = kasprintf(GFP_KERNEL, "GNSR0_PMEVTYPER%d_0", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GNSR0_PMEVTYPER(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GNSR0_PMEVCNTR%d_0", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GNSR0_PMEVCNTR(i);
		regs++;
	}

	for (i = 0; i < PMCG_SIZE; i++) {
		regs->name = kasprintf(GFP_KERNEL, "GNSR0_PMCGCR%d_0", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GNSR0_PMCGCR(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GNSR0_PMCGSMR%d_0", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GNSR0_PMCGSMR(i);
		regs++;
	}

	regs = (struct debugfs_reg32 *)smmu->perf_regset->regs;
	for (i = 0; i < smmu->perf_regset->nregs; i++) {
		debugfs_create_file(regs->name, S_IRUGO | S_IWUSR,
			dent_gnsr, regs, &smmu_perf_regset_debugfs_fops);
		regs++;
	}

	for (i = 0; i < smmu->num_context_banks; i++)
		debugfs_create_smmu_cb(smmu, i);

	return 0;

err_out:
	arm_smmu_debugfs_delete(smmu);
	return -1;
}

static int __init smmu_regs_init(void)
{
	int ret = 0;

	smmu_handle = kzalloc(sizeof(struct arm_smmu_device), GFP_KERNEL);
	if (smmu_handle == NULL)
		return -ENOMEM;

	/* Get the smmu base register */
	smmu_handle->base = ioremap(0x12000000, 0x800000);
	smmu_handle->size = 0x800000;
	smmu_handle->pgshift = PAGE_SHIFT;
	smmu_handle->num_context_banks = 64;

	if (IS_ERR(smmu_handle->base))
		return PTR_ERR(smmu_handle->base);

	ret = arm_smmu_debugfs_create(smmu_handle);
	if (ret != 0)
		pr_debug("arm_smmu_debugfs_create failed\n");

	return 0;
}

static void __exit smmu_regs_exit(void)
{
	iounmap(smmu_handle->base);
	arm_smmu_debugfs_delete(smmu_handle);
	kfree(smmu_handle);
}

module_init(smmu_regs_init);
module_exit(smmu_regs_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("arm-smmu regdump");
