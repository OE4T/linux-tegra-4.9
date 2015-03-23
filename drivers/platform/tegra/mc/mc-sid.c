/*
 * MC StreamID configuration
 *
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
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define pr_fmt(fmt)	"%s(): " fmt, __func__

#include <linux/err.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <dt-bindings/memory/tegra-swgroup.h>

#define SCEW_STREAMID_WRITE_ACCESS_DISABLED	BIT(16)
#define SCEW_STREAMID_OVERRIDE		BIT(8)
#define SCEW_NS				BIT(0)

#define MC_SMMU_BYPASS_CONFIG_0		0x1820
#define TBU_BYPASS_SID			2

enum override_id {
	PTCR,
	AFIR,
	HDAR,
	HOST1XDMAR,
	NVENCSRD,
	SATAR,
	MPCORER,
	NVENCSWR,
	AFIW,
	HDAW,
	MPCOREW,
	SATAW,
	ISPRA,
	ISPWA,
	ISPWB,
	XUSB_HOSTR,
	XUSB_HOSTW,
	XUSB_DEVR,
	XUSB_DEVW,
	TSECSRD,
	TSECSWR,
	GPUSRD,
	GPUSWR,
	SDMMCRA,
	SDMMCRAA,
	SDMMCR,
	SDMMCRAB,
	SDMMCWA,
	SDMMCWAA,
	SDMMCW,
	SDMMCWAB,
	VICSRD,
	VICSWR,
	VIW,
	NVDECSRD,
	NVDECSWR,
	APER,
	APEW,
	NVJPGSRD,
	NVJPGSWR,
	SESRD,
	SESWR,
	ETRR,
	ETRW,
	TSECSRDB,
	TSECSWRB,
	GPUSRD2,
	GPUSWR2,
	AXISR,
	AXISW,
	EQOSR,
	EQOSW,
	UFSHCR,
	UFSHCW,
	NVDISPLAYR,
	BPMPR,
	BPMPW,
	BPMPDMAR,
	BPMPDMAW,
	AONR,
	AONW,
	AONDMAR,
	AONDMAW,
	SCER,
	SCEW,
	SCEDMAR,
	SCEDMAW,
	APEDMAR,
	APEDMAW,
	NVDISPLAYR1,
	VICSRD1,
	NVDECSRD1,
	MAX_OID,
};

struct sid_override_reg {
	const char *name;
	int offs;
};

#define DEFREG(__name, __offset)		\
	[__name] = {				\
		.name = __stringify(__name),	\
		.offs = __offset,		\
	}

static struct sid_override_reg sid_override_reg[] = {
	DEFREG(PTCR, 0x000),
	DEFREG(AFIR, 0x070),
	DEFREG(HDAR, 0x0a8),
	DEFREG(HOST1XDMAR, 0x0b0),
	DEFREG(NVENCSRD, 0x0e0),
	DEFREG(SATAR, 0x0f8),
	DEFREG(MPCORER, 0x138),
	DEFREG(NVENCSWR, 0x158),
	DEFREG(AFIW, 0x188),
	DEFREG(HDAW, 0x1a8),
	DEFREG(MPCOREW, 0x1c8),
	DEFREG(SATAW, 0x1e8),
	DEFREG(ISPRA, 0x220),
	DEFREG(ISPWA, 0x230),
	DEFREG(ISPWB, 0x238),
	DEFREG(XUSB_HOSTR, 0x250),
	DEFREG(XUSB_HOSTW, 0x258),
	DEFREG(XUSB_DEVR, 0x260),
	DEFREG(XUSB_DEVW, 0x268),
	DEFREG(TSECSRD, 0x2a0),
	DEFREG(TSECSWR, 0x2a8),
	DEFREG(GPUSRD, 0x2c0),
	DEFREG(GPUSWR, 0x2c8),
	DEFREG(SDMMCRA, 0x300),
	DEFREG(SDMMCRAA, 0x308),
	DEFREG(SDMMCR, 0x310),
	DEFREG(SDMMCRAB, 0x318),
	DEFREG(SDMMCWA, 0x320),
	DEFREG(SDMMCWAA, 0x328),
	DEFREG(SDMMCW, 0x330),
	DEFREG(SDMMCWAB, 0x338),
	DEFREG(VICSRD, 0x360),
	DEFREG(VICSWR, 0x368),
	DEFREG(VIW, 0x390),
	DEFREG(NVDECSRD, 0x3c0),
	DEFREG(NVDECSWR, 0x3c8),
	DEFREG(APER, 0x3d0),
	DEFREG(APEW, 0x3d8),
	DEFREG(NVJPGSRD, 0x3f0),
	DEFREG(NVJPGSWR, 0x3f8),
	DEFREG(SESRD, 0x400),
	DEFREG(SESWR, 0x408),
	DEFREG(ETRR, 0x420),
	DEFREG(ETRW, 0x428),
	DEFREG(TSECSRDB, 0x430),
	DEFREG(TSECSWRB, 0x438),
	DEFREG(GPUSRD2, 0x440),
	DEFREG(GPUSWR2, 0x448),
	DEFREG(AXISR, 0x460),
	DEFREG(AXISW, 0x468),
	DEFREG(EQOSR, 0x470),
	DEFREG(EQOSW, 0x478),
	DEFREG(UFSHCR, 0x480),
	DEFREG(UFSHCW, 0x488),
	DEFREG(NVDISPLAYR, 0x490),
	DEFREG(BPMPR, 0x498),
	DEFREG(BPMPW, 0x4a0),
	DEFREG(BPMPDMAR, 0x4a8),
	DEFREG(BPMPDMAW, 0x4b0),
	DEFREG(AONR, 0x4b8),
	DEFREG(AONW, 0x4c0),
	DEFREG(AONDMAR, 0x4c8),
	DEFREG(AONDMAW, 0x4d0),
	DEFREG(SCER, 0x4d8),
	DEFREG(SCEW, 0x4e0),
	DEFREG(SCEDMAR, 0x4e8),
	DEFREG(SCEDMAW, 0x4f0),
	DEFREG(APEDMAR, 0x4f8),
	DEFREG(APEDMAW, 0x500),
	DEFREG(NVDISPLAYR1, 0x508),
	DEFREG(VICSRD1, 0x510),
	DEFREG(NVDECSRD1, 0x518),
};
#undef DEFREG

#define MAX_OIDS_IN_SID 5
struct sid_to_oids
{
	int sid;			/* StreamID */
	int noids;			/* # of override IDs */
	int oid[MAX_OIDS_IN_SID];	/* Override IDs */
	bool mc_overrides;		/* MC or Device overrides SID? */
};

static struct sid_to_oids sid_to_oids[] = {
	{
		.sid	= TEGRA_SWGROUP_AFI,
		.noids	= 2,
		.oid	= {
			AFIR,
			AFIW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_HDA,
		.noids	= 2,
		.oid	= {
			HDAR,
			HDAW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_SATA2,
		.noids	= 2,
		.oid	= {
			SATAR,
			SATAW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_XUSB_HOST,
		.noids	= 2,
		.oid	= {
			XUSB_HOSTR,
			XUSB_HOSTW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_XUSB_DEV,
		.noids	= 2,
		.oid	= {
			XUSB_DEVR,
			XUSB_DEVW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_TSEC,
		.noids	= 2,
		.oid	= {
			TSECSRD,
			TSECSWR,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_GPUB,
		.noids	= 4,
		.oid	= {
			GPUSRD,
			GPUSWR,
			GPUSRD2,
			GPUSWR2,
		},
		.mc_overrides = false,
	},
	{
		.sid	= TEGRA_SWGROUP_SDMMC1A,
		.noids	= 2,
		.oid	= {
			SDMMCRA,
			SDMMCWA,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_SDMMC2A,
		.noids	= 2,
		.oid	= {
			SDMMCRAA,
			SDMMCWAA,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_SDMMC3A,
		.noids	= 2,
		.oid	= {
			SDMMCR,
			SDMMCW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_SDMMC4A,
		.noids	= 2,
		.oid	= {
			SDMMCRAB,
			SDMMCWAB,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_APE,
		.noids	= 4,
		.oid	= {
			APER,
			APEW,
			APEDMAR,
			APEDMAW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_SE,
		.noids	= 2,
		.oid	= {
			SESRD,
			SESWR,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_ETR,
		.noids	= 2,
		.oid	= {
			ETRR,
			ETRW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_TSECB,
		.noids	= 2,
		.oid	= {
			TSECSRDB,
			TSECSWRB,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_AXIS,
		.noids	= 2,
		.oid	= {
			AXISR,
			AXISW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_EQOS,
		.noids	= 2,
		.oid	= {
			EQOSR,
			EQOSW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_UFSHC,
		.noids	= 2,
		.oid	= {
			UFSHCR,
			UFSHCW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_NVDISPLAY,
		.noids	= 1,
		.oid	= {
			NVDISPLAYR,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_BPMP,
		.noids	= 4,
		.oid	= {
			BPMPR,
			BPMPW,
			BPMPDMAR,
			BPMPDMAW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_AON,
		.noids	= 4,
		.oid	= {
			AONR,
			AONW,
			AONDMAR,
			AONDMAW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_SCE,
		.noids	= 4,
		.oid	= {
			SCER,
			SCEW,
			SCEDMAR,
			SCEDMAW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_HC,
		.noids	= 1,
		.oid	= {
			HOST1XDMAR,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_VIC,
		.noids	= 2,
		.oid = {
			VICSRD,
			VICSWR,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_VI,
		.noids	= 1,
		.oid	= {
			VIW,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_ISP,
		.noids	= 3,
		.oid	= {
			ISPRA,
			ISPWA,
			ISPWB,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_NVDEC,
		.noids	= 2,
		.oid	= {
			NVDECSRD,
			NVDECSWR,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_NVENC,
		.noids	= 2,
		.oid	= {
			NVENCSRD,
			NVENCSWR,
		},
		.mc_overrides = true,
	},
	{
		.sid	= TEGRA_SWGROUP_NVJPG,
		.noids	= 2,
		.oid	= {
			NVJPGSRD,
			NVJPGSWR,
		},
		.mc_overrides = true,
	},
};

#define TO_MC_SID_STREAMID_SECURITY_CONFIG(addr)	(addr + sizeof(u32))

static void __iomem *mc_sid_base;
static void __iomem *mc_base;

static struct of_device_id mc_sid_of_match[] = {
	{ .compatible = "nvidia,tegra-mc-sid", .data = (void *)0, },
	{ .compatible = "nvidia,tegra-mc-sid-cl34000094", .data = (void *)1, },
	{},
};
MODULE_DEVICE_TABLE(of, mc_sid_of_match);

static long ms_sid_is_cl34000094; /* support for obsolete cl34000094 */

static void __mc_override_sid(int sid, int oid)
{
	const char *name = sid_override_reg[oid].name;
	int offs = sid_override_reg[oid].offs;
	volatile void __iomem *addr;
	u32 val;

	BUG_ON(oid >= MAX_OID);

	if (ms_sid_is_cl34000094) {
		addr = mc_sid_base + offs / 2;
		val = 0x80010000 | sid;
		writel_relaxed(val, addr);
	} else {
		addr = mc_sid_base + offs;
		val = readl_relaxed(TO_MC_SID_STREAMID_SECURITY_CONFIG(addr));

		if (val & SCEW_STREAMID_OVERRIDE) {
			/* OK */;
		} else if (val & SCEW_STREAMID_WRITE_ACCESS_DISABLED) {
			pr_info("Cann't OVERRIDE SID for MC_SID_STRAMID_SECURITY_CONFIG_%s(%x) %x\n",
				name, offs, val);
			return;
		}

		if ((val & SCEW_NS) == 0)
			pr_info("SECURE for MC_SID_STRAMID_SECURITY_CONFIG_%s(%x) %x\n",
				name, offs, val);

		val = SCEW_STREAMID_OVERRIDE | SCEW_NS;
		writel_relaxed(val, addr);

		addr = mc_sid_base + offs;
		writel_relaxed(sid, addr);
	}

	pr_debug("override sid=%d oid=%d at offset=%x\n", sid, oid, offs);
}

void platform_override_streamid(int sid)
{
	int i;

	if (!mc_sid_base) {
		pr_err("mc-sid isn't populated\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(sid_to_oids); i++) {
		struct sid_to_oids *conf;
		int j;

		conf = &sid_to_oids[i];
		BUG_ON(conf->noids >= MAX_OIDS_IN_SID);

		if (sid != conf->sid)
			continue;

		if (!conf->mc_overrides)
			continue;

		for (j = 0; j < conf->noids; j++)
			__mc_override_sid(sid, conf->oid[j]);
	}
}

#if defined(CONFIG_DEBUG_FS)
enum { ORD, SEC, TXN, MAX_REGS_TYPE};
static const char *mc_regs_type[] = { "ord", "sec", "txn", };
static struct debugfs_reg32 mc_regs[MAX_REGS_TYPE * MAX_OID];
static struct debugfs_regset32 mc_regset[MAX_REGS_TYPE];

static void mc_sid_debugfs(void)
{
	int i, j;
	struct dentry *dent;

	dent = debugfs_create_dir("mc_sid", NULL);
	if (!dent)
		return;

	for (i = 0; i < MAX_REGS_TYPE; i++) {
		struct debugfs_regset32 *set = mc_regset + i;
		struct debugfs_reg32 *reg = mc_regs + i * MAX_OID;
		int diff = 0;

		set->regs = reg;
		set->nregs = MAX_OID;
		set->base = mc_sid_base;
		if (i == SEC) {
			diff = sizeof(u32);
		} else if (i == TXN) {
			set->base = mc_base;
			diff = 0x1000;
		}

		for (j = 0; j < MAX_OID; j++, reg++) {
			reg->name = (char *)sid_override_reg[j].name;
			reg->offset = sid_override_reg[j].offs;
			reg->offset += diff;
		}

		debugfs_create_regset32(mc_regs_type[i], S_IRUGO, dent, set);
	}
}
#else
static inline void mc_sid_debugfs(void)
{
}
#endif	/* CONFIG_DEBUG_FS */

static int mc_sid_probe(struct platform_device *pdev)
{
	int i;
	struct resource *res;
	static void __iomem *addr;
	const struct of_device_id *id;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr))
		return PTR_ERR(addr);
	mc_sid_base = addr;

	id = of_match_device(mc_sid_of_match, &pdev->dev);
	ms_sid_is_cl34000094 = (long)id->data;

	for (i = 0; i < ARRAY_SIZE(sid_override_reg); i++)
		__mc_override_sid(0x7f, i);

	/* FIXME: wait for MC driver */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr))
		return PTR_ERR(addr);
	mc_base = addr;
	writel_relaxed(TBU_BYPASS_SID, mc_base + MC_SMMU_BYPASS_CONFIG_0);

	mc_sid_debugfs();
	return 0;
}

static struct platform_driver mc_sid_driver = {
	.probe	= mc_sid_probe,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "mc-sid",
		.of_match_table	= of_match_ptr(mc_sid_of_match),
	},
};

static int __init mc_sid_init(void)
{
	return platform_driver_register(&mc_sid_driver);
}
arch_initcall_sync(mc_sid_init); /* FIXME: population order */

MODULE_DESCRIPTION("MC StreamID configuration");
MODULE_AUTHOR("Hiroshi DOYU <hdoyu@nvidia.com>");
MODULE_LICENSE("GPL v2");
