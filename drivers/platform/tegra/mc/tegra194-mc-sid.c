/*
 * Tegra194 MC StreamID configuration
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt)	"%s(): " fmt, __func__

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <linux/platform/tegra/tegra-mc-sid.h>
#include <dt-bindings/memory/tegra-swgroup.h>
#include <dt-bindings/memory/tegra194-swgroup.h>

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
	DLA0RDA,
	DLA0FALRDB,
	DLA0WRA,
	DLA0FALWRB,
	DLA1RDA,
	DLA1FALRDB,
	DLA1WRA,
	DLA1FALWRB,
	PVA0RDA,
	PVA0RDB,
	PVA0RDC,
	PVA0WRA,
	PVA0WRB,
	PVA0WRC,
	PVA1RDA,
	PVA1RDB,
	PVA1RDC,
	PVA1WRA,
	PVA1WRB,
	PVA1WRC,
	DLA0RDA1,
	DLA1RDA1,
	PVA0RDA1,
	PVA0RDB1,
	PVA1RDA1,
	PVA1RDB1,
	MAX_OID,
};

static struct sid_override_reg sid_override_reg[] = {
	DEFREG(PTCR,		0x000),
	DEFREG(AFIR,		0x070),
	DEFREG(HDAR,		0x0a8),
	DEFREG(HOST1XDMAR,	0x0b0),
	DEFREG(NVENCSRD,	0x0e0),
	DEFREG(SATAR,		0x0f8),
	DEFREG(MPCORER,		0x138),
	DEFREG(NVENCSWR,	0x158),
	DEFREG(AFIW,		0x188),
	DEFREG(HDAW,		0x1a8),
	DEFREG(MPCOREW,		0x1c8),
	DEFREG(SATAW,		0x1e8),
	DEFREG(ISPRA,		0x220),
	DEFREG(ISPWA,		0x230),
	DEFREG(ISPWB,		0x238),
	DEFREG(XUSB_HOSTR,	0x250),
	DEFREG(XUSB_HOSTW,	0x258),
	DEFREG(XUSB_DEVR,	0x260),
	DEFREG(XUSB_DEVW,	0x268),
	DEFREG(TSECSRD,		0x2a0),
	DEFREG(TSECSWR,		0x2a8),
	DEFREG(GPUSRD,		0x2c0),
	DEFREG(GPUSWR,		0x2c8),
	DEFREG(SDMMCRA,		0x300),
	DEFREG(SDMMCRAA,	0x308),
	DEFREG(SDMMCR,		0x310),
	DEFREG(SDMMCRAB,	0x318),
	DEFREG(SDMMCWA,		0x320),
	DEFREG(SDMMCWAA,	0x328),
	DEFREG(SDMMCW,		0x330),
	DEFREG(SDMMCWAB,	0x338),
	DEFREG(VICSRD,		0x360),
	DEFREG(VICSWR,		0x368),
	DEFREG(VIW,		0x390),
	DEFREG(NVDECSRD,	0x3c0),
	DEFREG(NVDECSWR,	0x3c8),
	DEFREG(APER,		0x3d0),
	DEFREG(APEW,		0x3d8),
	DEFREG(NVJPGSRD,	0x3f0),
	DEFREG(NVJPGSWR,	0x3f8),
	DEFREG(SESRD,		0x400),
	DEFREG(SESWR,		0x408),
	DEFREG(ETRR,		0x420),
	DEFREG(ETRW,		0x428),
	DEFREG(TSECSRDB,	0x430),
	DEFREG(TSECSWRB,	0x438),
	DEFREG(GPUSRD2,		0x440),
	DEFREG(GPUSWR2,		0x448),
	DEFREG(AXISR,		0x460),
	DEFREG(AXISW,		0x468),
	DEFREG(EQOSR,		0x470),
	DEFREG(EQOSW,		0x478),
	DEFREG(UFSHCR,		0x480),
	DEFREG(UFSHCW,		0x488),
	DEFREG(NVDISPLAYR,	0x490),
	DEFREG(BPMPR,		0x498),
	DEFREG(BPMPW,		0x4a0),
	DEFREG(BPMPDMAR,	0x4a8),
	DEFREG(BPMPDMAW,	0x4b0),
	DEFREG(AONR,		0x4b8),
	DEFREG(AONW,		0x4c0),
	DEFREG(AONDMAR,		0x4c8),
	DEFREG(AONDMAW,		0x4d0),
	DEFREG(SCER,		0x4d8),
	DEFREG(SCEW,		0x4e0),
	DEFREG(SCEDMAR,		0x4e8),
	DEFREG(SCEDMAW,		0x4f0),
	DEFREG(APEDMAR,		0x4f8),
	DEFREG(APEDMAW,		0x500),
	DEFREG(NVDISPLAYR1,	0x508),
	DEFREG(VICSRD1,		0x510),
	DEFREG(NVDECSRD1,	0x518),
	DEFREG(DLA0RDA,		0x5f0),
	DEFREG(DLA0FALRDB,	0x5f8),
	DEFREG(DLA0WRA,		0x600),
	DEFREG(DLA0FALWRB,	0x608),
	DEFREG(DLA1RDA,		0x610),
	DEFREG(DLA1FALRDB,	0x618),
	DEFREG(DLA1WRA,		0x620),
	DEFREG(DLA1FALWRB,	0x628),
	DEFREG(PVA0RDA,		0x630),
	DEFREG(PVA0RDB,		0x638),
	DEFREG(PVA0RDC,		0x640),
	DEFREG(PVA0WRA,		0x648),
	DEFREG(PVA0WRB,		0x650),
	DEFREG(PVA0WRC,		0x658),
	DEFREG(PVA1RDA,		0x660),
	DEFREG(PVA1RDB,		0x668),
	DEFREG(PVA1RDC,		0x670),
	DEFREG(PVA1WRA,		0x678),
	DEFREG(PVA1WRB,		0x680),
	DEFREG(PVA1WRC,		0x688),
	DEFREG(DLA0RDA1,	0x748),
	DEFREG(DLA1RDA1,	0x750),
	DEFREG(PVA0RDA1,	0x758),
	DEFREG(PVA0RDB1,	0x760),
	DEFREG(PVA1RDA1,	0x768),
	DEFREG(PVA1RDB1,	0x770),
};

static struct sid_to_oids sid_to_oids[] = {
	{
		.sid	= TEGRA_SID_AFI,
		.noids	= 2,
		.oid	= {
			AFIR,
			AFIW,
		},
		.ord = OVERRIDE,
		.name = "AFI",
	},
	{
		.sid	= TEGRA_SID_HDA,
		.noids	= 2,
		.oid	= {
			HDAR,
			HDAW,
		},
		.ord = OVERRIDE,
		.name = "HDA",
	},
	{
		.sid	= TEGRA_SID_SATA2,
		.noids	= 2,
		.oid	= {
			SATAR,
			SATAW,
		},
		.ord = OVERRIDE,
		.name = "SATA2",
	},
	{
		.sid	= TEGRA_SID_XUSB_HOST,
		.noids	= 2,
		.oid	= {
			XUSB_HOSTR,
			XUSB_HOSTW,
		},
		.ord = OVERRIDE,
		.name = "XUSB_HOST",
	},
	{
		.sid	= TEGRA_SID_XUSB_DEV,
		.noids	= 2,
		.oid	= {
			XUSB_DEVR,
			XUSB_DEVW,
		},
		.ord = OVERRIDE,
		.name = "XUSB_DEV",
	},
	{
		.sid	= TEGRA_SID_TSEC,
		.noids	= 2,
		.oid	= {
			TSECSRD,
			TSECSWR,
		},
		.ord = SIM_OVERRIDE,
		.name = "TSEC",
	},
	{
		.sid	= TEGRA_SID_GPUB,
		.noids	= 4,
		.oid	= {
			GPUSRD,
			GPUSWR,
			GPUSRD2,
			GPUSWR2,
		},
		.ord = NO_OVERRIDE,
		.name = "GPU",
	},
	{
		.sid	= TEGRA_SID_SDMMC1A,
		.noids	= 2,
		.oid	= {
			SDMMCRA,
			SDMMCWA,
		},
		.ord = OVERRIDE,
		.name = "SDMMC1A",
	},
	{
		.sid	= TEGRA_SID_SDMMC2A,
		.noids	= 2,
		.oid	= {
			SDMMCRAA,
			SDMMCWAA,
		},
		.ord = OVERRIDE,
		.name = "SDMMC2A",
	},
	{
		.sid	= TEGRA_SID_SDMMC3A,
		.noids	= 2,
		.oid	= {
			SDMMCR,
			SDMMCW,
		},
		.ord = OVERRIDE,
		.name = "SDMMC3A",
	},
	{
		.sid	= TEGRA_SID_SDMMC4A,
		.noids	= 2,
		.oid	= {
			SDMMCRAB,
			SDMMCWAB,
		},
		.ord = OVERRIDE,
		.name = "SDMMC4A",
	},
	{
		.sid	= TEGRA_SID_APE,
		.noids	= 4,
		.oid	= {
			APER,
			APEW,
			APEDMAR,
			APEDMAW,
		},
		.ord = NO_OVERRIDE,
		.name = "APE",
	},
	{
		.sid	= TEGRA_SID_SE,
		.noids	= 2,
		.oid	= {
			SESRD,
			SESWR,
		},
		.ord = NO_OVERRIDE,
		.name = "SE",
	},
	{
		.sid	= TEGRA_SID_ETR,
		.noids	= 2,
		.oid	= {
			ETRR,
			ETRW,
		},
		.ord = OVERRIDE,
		.name = "ETR",
	},
	{
		.sid	= TEGRA_SID_TSECB,
		.noids	= 2,
		.oid	= {
			TSECSRDB,
			TSECSWRB,
		},
		.ord = SIM_OVERRIDE,
		.name = "TSECB",
	},
	{
		.sid	= TEGRA_SID_GPCDMA_0, /* AXIS */
		.noids	= 2,
		.oid	= {
			AXISR,
			AXISW,
		},
		.ord = NO_OVERRIDE,
		.name = "GPCDMA",
	},
	{
		.sid	= TEGRA_SID_EQOS,
		.noids	= 2,
		.oid	= {
			EQOSR,
			EQOSW,
		},
		.ord = OVERRIDE,
		.name = "EQOS",
	},
	{
		.sid	= TEGRA_SID_UFSHC,
		.noids	= 2,
		.oid	= {
			UFSHCR,
			UFSHCW,
		},
		.ord = OVERRIDE,
		.name = "UFSHC",
	},
	{
		.sid	= TEGRA_SID_NVDISPLAY,
		.noids	= 2,
		.oid	= {
			NVDISPLAYR,
			NVDISPLAYR1,
		},
		.ord = OVERRIDE,
		.name = "NVDISPLAY",
	},
	{
		.sid	= TEGRA_SID_BPMP,
		.noids	= 4,
		.oid	= {
			BPMPR,
			BPMPW,
			BPMPDMAR,
			BPMPDMAW,
		},
		.ord = NO_OVERRIDE,
		.name = "BPMP",
	},
	{
		.sid	= TEGRA_SID_AON,
		.noids	= 4,
		.oid	= {
			AONR,
			AONW,
			AONDMAR,
			AONDMAW,
		},
		.ord = NO_OVERRIDE,
		.name = "AON",
	},
	{
		.sid	= TEGRA_SID_SCE,
		.noids	= 4,
		.oid	= {
			SCER,
			SCEW,
			SCEDMAR,
			SCEDMAW,
		},
		.ord = NO_OVERRIDE,
		.name = "SCE",
	},
	{
		.sid	= TEGRA_SID_HC,
		.noids	= 1,
		.oid	= {
			HOST1XDMAR,
		},
		.ord = SIM_OVERRIDE,
		.name = "HC",
	},
	{
		.sid	= TEGRA_SID_VIC,
		.noids	= 3,
		.oid = {
			VICSRD1,
			VICSRD,
			VICSWR,
		},
		.ord = SIM_OVERRIDE,
		.name = "VIC",
	},
	{
		.sid	= TEGRA_SID_VI,
		.noids	= 1,
		.oid	= {
			VIW,
		},
		.ord = OVERRIDE,
		.name = "VI",
	},
	{
		.sid	= TEGRA_SID_ISP,
		.noids	= 3,
		.oid	= {
			ISPRA,
			ISPWA,
			ISPWB,
		},
		.ord = OVERRIDE,
		.name = "ISP",
	},
	{
		.sid	= TEGRA_SID_NVDEC,
		.noids	= 3,
		.oid	= {
			NVDECSRD1,
			NVDECSRD,
			NVDECSWR,
		},
		.ord = SIM_OVERRIDE,
		.name = "NVDEC",
	},
	{
		.sid	= TEGRA_SID_NVENC,
		.noids	= 2,
		.oid	= {
			NVENCSRD,
			NVENCSWR,
		},
		.ord = SIM_OVERRIDE,
		.name = "NVENC",
	},
	{
		.sid	= TEGRA_SID_NVJPG,
		.noids	= 2,
		.oid	= {
			NVJPGSRD,
			NVJPGSWR,
		},
		.ord = SIM_OVERRIDE,
		.name = "NVJPG",
	},
	{
		.sid	= TEGRA_SID_NVDLA0,
		.noids	= 5,
		.oid	= {
			DLA0RDA,
			DLA0FALRDB,
			DLA0WRA,
			DLA0FALWRB,
			DLA0RDA1,
		},
		.ord = SIM_OVERRIDE,
		.name = "NVDLA0",
	},
	{
		.sid	= TEGRA_SID_NVDLA1,
		.noids	= 5,
		.oid	= {
			DLA1RDA,
			DLA1FALRDB,
			DLA1WRA,
			DLA1FALWRB,
			DLA1RDA1,
		},
		.ord = SIM_OVERRIDE,
		.name = "NVDLA1",
	},
	{
		.sid	= TEGRA_SID_PVA0,
		.noids	= 8,
		.oid	= {
			PVA0RDA,
			PVA0RDB,
			PVA0RDC,
			PVA0WRA,
			PVA0WRB,
			PVA0WRC,
			PVA0RDA1,
			PVA0RDB1,
		},
		.ord = SIM_OVERRIDE,
		.name = "PVA0",
	},
	{
		.sid	= TEGRA_SID_PVA1,
		.noids	= 8,
		.oid	= {
			PVA1RDA,
			PVA1RDB,
			PVA1RDC,
			PVA1WRA,
			PVA1WRB,
			PVA1WRC,
			PVA1RDA1,
			PVA1RDB1,
		},
		.ord = SIM_OVERRIDE,
		.name = "PVA1",
	},
};

static const struct tegra_mc_sid_soc_data tegra194_mc_soc_data = {
	.sid_override_reg = sid_override_reg,
	.nsid_override_reg = ARRAY_SIZE(sid_override_reg),
	.sid_to_oids = sid_to_oids,
	.nsid_to_oids = ARRAY_SIZE(sid_to_oids),
	.max_oids = MAX_OID,
};

static int tegra194_mc_sid_probe(struct platform_device *pdev)
{
	return tegra_mc_sid_probe(pdev, &tegra194_mc_soc_data);
}

static const struct of_device_id tegra194_mc_sid_of_match[] = {
	{ .compatible = "nvidia,tegra194-mc-sid", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra194_mc_sid_of_match);

static struct platform_driver tegra194_mc_sid_driver = {
	.probe	= tegra194_mc_sid_probe,
	.remove = tegra_mc_sid_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "tegra194-mc-sid",
		.of_match_table	= of_match_ptr(tegra194_mc_sid_of_match),
	},
};

static int __init tegra194_mc_sid_init(void)
{
	struct device_node *np;
	struct platform_device *pdev = NULL;

	np = of_find_compatible_node(NULL, NULL, "nvidia,tegra194-mc-sid");
	if (np) {
		pdev = of_platform_device_create(np, NULL,
						 platform_bus_type.dev_root);
		of_node_put(np);
	}

	if (IS_ERR_OR_NULL(pdev))
		return -ENODEV;

	return platform_driver_register(&tegra194_mc_sid_driver);
}
arch_initcall(tegra194_mc_sid_init);

MODULE_DESCRIPTION("MC StreamID configuration");
MODULE_AUTHOR("Hiroshi DOYU <hdoyu@nvidia.com>, Pritesh Raithatha <praithatha@nvidia.com>");
MODULE_LICENSE("GPL v2");
