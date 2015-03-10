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
#include <linux/of.h>
#include <linux/platform_device.h>

#include <dt-bindings/memory/tegra-swgroup.h>

#define SCEW_STREAMID_WRITE_ACCESS	BIT(16)
#define SCEW_STREAMID_OVERRIDE		BIT(8)
#define SCEW_NS				BIT(0)

#define MC_SMMU_BYPASS_CONFIG_0		0x1820
#define TBU_BYPASS_SID			2

/* FIXME: Move to dt-bindings/memory/tegra-swgroup.h */
#define TEGRA_SWGROUP_SCE		52

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

/* Generated from sim.t186/linsim/include_chip/collector/t186/armc_sid.h */
static int sid_override_offset[] = {
	[PTCR]		= 0x000,
	[AFIR]		= 0x070,
	[HDAR]		= 0x0a8,
	[HOST1XDMAR]	= 0x0b0,
	[NVENCSRD]	= 0x0e0,
	[SATAR]		= 0x0f8,
	[MPCORER]	= 0x138,
	[NVENCSWR]	= 0x158,
	[AFIW]		= 0x188,
	[HDAW]		= 0x1a8,
	[MPCOREW]	= 0x1c8,
	[SATAW]		= 0x1e8,
	[ISPRA]		= 0x220,
	[ISPWA]		= 0x230,
	[ISPWB]		= 0x238,
	[XUSB_HOSTR]	= 0x250,
	[XUSB_HOSTW]	= 0x258,
	[XUSB_DEVR]	= 0x260,
	[XUSB_DEVW]	= 0x268,
	[TSECSRD]	= 0x2a0,
	[TSECSWR]	= 0x2a8,
	[GPUSRD]	= 0x2c0,
	[GPUSWR]	= 0x2c8,
	[SDMMCRA]	= 0x300,
	[SDMMCRAA]	= 0x308,
	[SDMMCR]	= 0x310,
	[SDMMCRAB]	= 0x318,
	[SDMMCWA]	= 0x320,
	[SDMMCWAA]	= 0x328,
	[SDMMCW]	= 0x330,
	[SDMMCWAB]	= 0x338,
	[VICSRD]	= 0x360,
	[VICSWR]	= 0x368,
	[VIW]		= 0x390,
	[NVDECSRD]	= 0x3c0,
	[NVDECSWR]	= 0x3c8,
	[APER]		= 0x3d0,
	[APEW]		= 0x3d8,
	[NVJPGSRD]	= 0x3f0,
	[NVJPGSWR]	= 0x3f8,
	[SESRD]		= 0x400,
	[SESWR]		= 0x408,
	[ETRR]		= 0x420,
	[ETRW]		= 0x428,
	[TSECSRDB]	= 0x430,
	[TSECSWRB]	= 0x438,
	[GPUSRD2]	= 0x440,
	[GPUSWR2]	= 0x448,
	[AXISR]		= 0x460,
	[AXISW]		= 0x468,
	[EQOSR]		= 0x470,
	[EQOSW]		= 0x478,
	[UFSHCR]	= 0x480,
	[UFSHCW]	= 0x488,
	[NVDISPLAYR]	= 0x490,
	[BPMPR]		= 0x498,
	[BPMPW]		= 0x4a0,
	[BPMPDMAR]	= 0x4a8,
	[BPMPDMAW]	= 0x4b0,
	[AONR]		= 0x4b8,
	[AONW]		= 0x4c0,
	[AONDMAR]	= 0x4c8,
	[AONDMAW]	= 0x4d0,
	[SCER]		= 0x4d8,
	[SCEW]		= 0x4e0,
	[SCEDMAR]	= 0x4e8,
	[SCEDMAW]	= 0x4f0,
	[APEDMAR]	= 0x4f8,
	[APEDMAW]	= 0x500,
	[NVDISPLAYR1]	= 0x508,
	[VICSRD1]	= 0x510,
	[NVDECSRD1]	= 0x518,
};

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
		.mc_overrides = true,
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

static void __iomem *mc_sid_base;

static struct of_device_id mc_sid_of_match[] = {
	{ .compatible = "nvidia,tegra-mc-sid", },
}
MODULE_DEVICE_TABLE(of, mc_sid_of_match);

static void __mc_override_sid(int sid, int oid)
{
	volatile void __iomem *addr;
	u32 val;

	BUG_ON(oid >= MAX_OID);

	addr = mc_sid_base + sid_override_offset[oid];
	addr += sizeof(u32); /* MC_SID_STREAMID_SECURITY_CONFIG_* */
	val = SCEW_STREAMID_OVERRIDE | SCEW_NS;
	writel_relaxed(val, addr);

	addr = mc_sid_base + sid_override_offset[oid];
	writel_relaxed(sid, addr);

	pr_debug("override sid=%d oid=%d at offset=%x\n",
		 sid, oid, sid_override_offset[oid]);
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

static int mc_sid_probe(struct platform_device *pdev)
{
	int i;
	struct resource *res;
	static void __iomem *addr;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr))
		return PTR_ERR(addr);
	mc_sid_base = addr;

	for (i = 0; i < ARRAY_SIZE(sid_override_offset); i++)
		__mc_override_sid(0x7f, i);

	/* FIXME: wait for MC driver */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr))
		return PTR_ERR(addr);

	writel_relaxed(TBU_BYPASS_SID, addr + MC_SMMU_BYPASS_CONFIG_0);
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
