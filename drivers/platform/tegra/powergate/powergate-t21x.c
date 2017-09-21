/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <dt-bindings/clock/tegra210-car.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <soc/tegra/tegra_powergate.h>
#include <soc/tegra/tegra-powergate-driver.h>
#include <soc/tegra/chip-id.h>
#include <linux/tegra_soctherm.h>
#include <soc/tegra/tegra-dvfs.h>
#include <trace/events/power.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clk/tegra.h>
#include <soc/tegra/reset.h>
#include <soc/tegra/common.h>
#include <soc/tegra/fuse.h>
#include <linux/platform/tegra/mc.h>

#define MAX_CLK_EN_NUM			15
#define MAX_HOTRESET_CLIENT_NUM		4

enum clk_type {
	CLK_AND_RST,
	RST_ONLY,
	CLK_ONLY,
};

struct partition_clk_info {
	const char *clk_name;
	enum clk_type clk_type;
	struct clk *clk_ptr;
};

struct powergate_partition_info {
	const char *name;
	struct partition_clk_info clk_info[MAX_CLK_EN_NUM];
	struct partition_clk_info slcg_info[MAX_CLK_EN_NUM];
	unsigned long reset_id[MAX_CLK_EN_NUM];
	int reset_id_num;
	struct raw_notifier_head slcg_notifier;
	int refcount;
	bool disable_after_boot;
	struct mutex pg_mutex;
	bool skip_reset;
};

#define EMULATION_MC_FLUSH_TIMEOUT 100
#define TEGRA210_POWER_DOMAIN_NVENC TEGRA210_POWER_DOMAIN_MPE

enum mc_client_type {
	MC_CLIENT_AFI		= 0,
	MC_CLIENT_AVPC		= 1,
	MC_CLIENT_DC		= 2,
	MC_CLIENT_DCB		= 3,
	MC_CLIENT_HC		= 6,
	MC_CLIENT_HDA		= 7,
	MC_CLIENT_ISP2		= 8,
	MC_CLIENT_NVENC		= 11,
	MC_CLIENT_SATA		= 15,
	MC_CLIENT_VI		= 17,
	MC_CLIENT_VIC		= 18,
	MC_CLIENT_XUSB_HOST	= 19,
	MC_CLIENT_XUSB_DEV	= 20,
	MC_CLIENT_BPMP		= 21,
	MC_CLIENT_TSEC		= 22,
	MC_CLIENT_SDMMC1	= 29,
	MC_CLIENT_SDMMC2	= 30,
	MC_CLIENT_SDMMC3	= 31,
	MC_CLIENT_SDMMC4	= 32,
	MC_CLIENT_ISP2B		= 33,
	MC_CLIENT_GPU		= 34,
	MC_CLIENT_NVDEC		= 37,
	MC_CLIENT_APE		= 38,
	MC_CLIENT_SE		= 39,
	MC_CLIENT_NVJPG		= 40,
	MC_CLIENT_TSECB		= 45,
	MC_CLIENT_LAST		= -1,
};

struct tegra210_mc_client_info {
	enum mc_client_type hot_reset_clients[MAX_HOTRESET_CLIENT_NUM];
};

static struct tegra210_mc_client_info tegra210_pg_mc_info[] = {
	[TEGRA210_POWER_DOMAIN_CRAIL] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_VENC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_ISP2,
			[1] = MC_CLIENT_VI,
			[2] = MC_CLIENT_LAST,
		},
	},
#ifdef CONFIG_ARCH_TEGRA_HAS_PCIE
	[TEGRA210_POWER_DOMAIN_PCIE] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_AFI,
			[1] = MC_CLIENT_LAST,
		},
	},
#endif
#ifdef CONFIG_ARCH_TEGRA_HAS_SATA
	[TEGRA210_POWER_DOMAIN_SATA] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_SATA,
			[1] = MC_CLIENT_LAST,
		},
	},
#endif
	[TEGRA210_POWER_DOMAIN_NVENC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_NVENC,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_SOR] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_DISA] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_DC,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_DISB] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_DCB,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_XUSBA] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_XUSBB] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_XUSB_DEV,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_XUSBC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_XUSB_HOST,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_VIC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_VIC,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_NVDEC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_NVDEC,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_NVJPG] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_NVJPG,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_APE] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_APE,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_VE2] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_ISP2B,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA210_POWER_DOMAIN_GPU] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_GPU,
			[1] = MC_CLIENT_LAST,
		},
	},
};

static struct powergate_partition_info tegra210_pg_partition_info[] = {
	[TEGRA210_POWER_DOMAIN_VENC] = {
		.name = "ve",
		.clk_info = {
			[0] = { .clk_name = "ispa", .clk_type = CLK_ONLY },
			[1] = { .clk_name = "vi", .clk_type = CLK_ONLY },
			[2] = { .clk_name = "csi", .clk_type = CLK_ONLY },
			[3] = { .clk_name = "vii2c", .clk_type = CLK_ONLY },
			[4] = { .clk_name = "cilab", .clk_type = CLK_ONLY },
			[5] = { .clk_name = "cilcd", .clk_type = CLK_ONLY },
			[6] = { .clk_name = "cile", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "host1x" },
			[5] = { .clk_name = "vi_slcg_ovr" },
			[6] = { .clk_name = "ispa_slcg_ovr" },
		},
		.reset_id = { TEGRA210_CLK_ISPA, TEGRA210_CLK_VI,
			      TEGRA210_CLK_CSI, TEGRA210_CLK_VI_I2C },
		.reset_id_num = 4,
	},
#ifdef CONFIG_ARCH_TEGRA_HAS_PCIE
	[TEGRA210_POWER_DOMAIN_PCIE] = {
		.name = "pcie",
		.clk_info = {
			[0] = { .clk_name = "afi", .clk_type = CLK_ONLY },
			[1] = { .clk_name = "pcie", .clk_type = CLK_ONLY },
		},
		.reset_id = { TEGRA210_CLK_AFI, TEGRA210_CLK_PCIE,
			      TEGRA210_CLK_PCIEX },
		.reset_id_num = 3,
		.skip_reset = true,
	},
#endif
#ifdef CONFIG_ARCH_TEGRA_HAS_SATA
	[TEGRA210_POWER_DOMAIN_SATA] = {
		.name = "sata",
		.disable_after_boot = false,
		.clk_info = {
			[0] = { .clk_name = "sata_oob", .clk_type = CLK_ONLY },
			[1] = { .clk_name = "cml1", .clk_type = CLK_ONLY },
			[3] = { .clk_name = "sata_aux", .clk_type = CLK_ONLY },
			[4] = { .clk_name = "sata", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "sata_slcg_fpci" },
			[5] = { .clk_name = "sata_slcg_ipfs" },
			[6] = { .clk_name = "sata_slcg_ovr" },
		},
		.reset_id = { TEGRA210_CLK_SATA_OOB, TEGRA210_CLK_SATA_COLD,
			      TEGRA210_CLK_SATA },
		.reset_id_num = 3,
	},
#endif
	[TEGRA210_POWER_DOMAIN_NVENC] = {
		.name = "nvenc",
		.clk_info = {
			[0] = { .clk_name = "nvenc", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "nvenc_slcg_ovr" },
		},
		.reset_id = { TEGRA210_CLK_NVENC },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_SOR] = {
		.name = "sor",
		.clk_info = {
			[0] = { .clk_name = "sor0", .clk_type = CLK_ONLY },
			[1] = { .clk_name = "dsia", .clk_type = CLK_ONLY },
			[2] = { .clk_name = "dsib", .clk_type = CLK_ONLY },
			[3] = { .clk_name = "sor1", .clk_type = CLK_ONLY },
			[4] = { .clk_name = "mipi-cal", .clk_type = CLK_ONLY },
			[5] = { .clk_name = "dpaux", .clk_type = CLK_ONLY },
			[6] = { .clk_name = "dpaux1", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "hda2hdmi" },
			[5] = { .clk_name = "hda2codec_2x" },
			[6] = { .clk_name = "disp1" },
			[7] = { .clk_name = "disp2" },
			[8] = { .clk_name = "disp1_slcg_ovr" },
			[9] = { .clk_name = "disp2_slcg_ovr" },
		},
		.reset_id = { TEGRA210_CLK_SOR0, TEGRA210_CLK_DISP1,
			      TEGRA210_CLK_DSIB, TEGRA210_CLK_SOR1,
			      TEGRA210_CLK_MIPI_CAL },
		.reset_id_num = 5,
	},
	[TEGRA210_POWER_DOMAIN_DISA] = {
		.name = "disa",
		.clk_info = {
			[0] = { .clk_name = "disp1", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "la" },
			[5] = { .clk_name = "host1x" },
			[6] = { .clk_name = "disp1_slcg_ovr" },
		},
		.reset_id = { TEGRA210_CLK_DISP1 },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_DISB] = {
		.name = "disb",
		.disable_after_boot = true,
		.clk_info = {
			[0] = { .clk_name = "disp2", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "la" },
			[5] = { .clk_name = "host1x" },
			[6] = { .clk_name = "disp2_slcg_ovr" },
		},
		.reset_id = { TEGRA210_CLK_DISP2 },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_XUSBA] = {
		.name = "xusba",
		.clk_info = {
			[0] = { .clk_name = "xusb_ss", .clk_type = CLK_ONLY },
			[1] = { .clk_name = "xusb_ssp_src", .clk_type = CLK_ONLY },
			[2] = { .clk_name = "xusb_hs_src", .clk_type = CLK_ONLY },
			[3] = { .clk_name = "xusb_fs_src", .clk_type = CLK_ONLY },
			[4] = { .clk_name = "xusb_dev_src", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "xusb_host" },
			[5] = { .clk_name = "xusb_dev" },
			[6] = { .clk_name = "xusb_host_slcg" },
			[7] = { .clk_name = "xusb_dev_slcg" },
		},
		.reset_id = { TEGRA210_CLK_XUSB_SS },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_XUSBB] = {
		.name = "xusbb",
		.clk_info = {
			[0] = { .clk_name = "xusb_dev", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "xusb_ss" },
			[5] = { .clk_name = "xusb_host" },
			[6] = { .clk_name = "xusb_host_slcg" },
			[7] = { .clk_name = "xusb_dev_slcg" },
		},
		.reset_id = { 95 },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_XUSBC] = {
		.name = "xusbc",
		.clk_info = {
			[0] = { .clk_name = "xusb_host", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "xusb_ss" },
			[5] = { .clk_name = "xusb_dev" },
			[6] = { .clk_name = "xusb_dev_slcg" },
			[7] = { .clk_name = "xusb_host_slcg" },
		},
		.reset_id = { TEGRA210_CLK_XUSB_HOST },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_VIC] = {
		.name = "vic",
		.clk_info = {
			[0] = { .clk_name = "vic03", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "vic03_slcg_ovr" },
			[5] = { .clk_name = "host1x" },
		},
		.reset_id = { TEGRA210_CLK_VIC03 },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_NVDEC] = {
		.name = "nvdec",
		.clk_info = {
			[0] = { .clk_name = "nvdec", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "nvdec_slcg_ovr" },
			[5] = { .clk_name = "nvjpg" },
			[6] = { .clk_name = "nvjpg_slcg_ovr" },
		},
		.reset_id = { TEGRA210_CLK_NVDEC },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_NVJPG] = {
		.name = "nvjpg",
		.clk_info = {
			[0] = { .clk_name = "nvjpg", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "nvjpg_slcg_ovr" },
			[5] = { .clk_name = "nvdec" },
			[6] = { .clk_name = "nvdec_slcg_ovr" },
		},
		.reset_id = { TEGRA210_CLK_NVJPG },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_APE] = {
		.name = "ape",
		.clk_info = {
			[0] = { .clk_name = "ape", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "aclk" },
			[5] = { .clk_name = "i2s0" },
			[6] = { .clk_name = "i2s1" },
			[7] = { .clk_name = "i2s2" },
			[8] = { .clk_name = "i2s3" },
			[9] = { .clk_name = "i2s4" },
			[10] = { .clk_name = "spdif_out" },
			[11] = { .clk_name = "d_audio" },
			[12] = { .clk_name = "ape_slcg_ovr" },
			[13] = { .clk_name = "aclk_slcg_ovr" },
			[14] = { .clk_name = "daudio_slcg_ovr" },

		},
		.reset_id = { TEGRA210_CLK_APE },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_VE2] = {
		.name = "ve2",
		.clk_info = {
			[0] = { .clk_name = "ispb", .clk_type = CLK_ONLY },
		},
		.slcg_info = {
			[0] = { .clk_name = "mc_capa" },
			[1] = { .clk_name = "mc_cbpa" },
			[2] = { .clk_name = "mc_ccpa" },
			[3] = { .clk_name = "mc_cdpa" },
			[4] = { .clk_name = "ispb_slcg_ovr" },
		},
		.reset_id = { TEGRA210_CLK_ISPB },
		.reset_id_num = 1,
	},
	[TEGRA210_POWER_DOMAIN_GPU] = {
		.name = "gpu",
		.clk_info = {
			[0] = { .clk_name = "gpu_gate", .clk_type = CLK_AND_RST },
			[1] = { .clk_name = "gpu_ref", .clk_type = CLK_ONLY },
			[2] = { .clk_name = "pll_p_out5", .clk_type = CLK_ONLY },
		},
	},
};

struct mc_client_hotreset_reg {
	u32 control_reg;
	u32 status_reg;
};

struct tegra210_powergate_info {
	bool valid;
	unsigned int mask;
	int part_id;
	struct tegra210_mc_client_info *mc_info;
	struct powergate_partition_info *part_info;
};

#define T210_POWERGATE_INFO(_pg_id, _pg_bit, _part_id, _mc_id)		\
[TEGRA210_POWER_DOMAIN_##_pg_id] = {						\
	.valid = true,							\
	.mask = BIT(_pg_bit),						\
	.part_id = TEGRA210_POWER_DOMAIN_##_part_id,				\
	.mc_info = &tegra210_pg_mc_info[TEGRA210_POWER_DOMAIN_##_mc_id],	\
	.part_info = &tegra210_pg_partition_info[TEGRA210_POWER_DOMAIN_##_part_id], \
}

static struct tegra210_powergate_info t210_pg_info[TEGRA210_POWER_DOMAIN_MAX] = {
	T210_POWERGATE_INFO(CRAIL, 0, CRAIL, CRAIL),
	T210_POWERGATE_INFO(GPU, 1, GPU, GPU),
	T210_POWERGATE_INFO(VENC, 2, VENC, VENC),
	T210_POWERGATE_INFO(PCIE, 3, PCIE, PCIE),
	T210_POWERGATE_INFO(VDEC, 4, VDEC, VDEC),
	T210_POWERGATE_INFO(L2, 5, L2, L2),
	T210_POWERGATE_INFO(MPE, 6, MPE, MPE),
	T210_POWERGATE_INFO(HEG, 7, HEG, HEG),
	T210_POWERGATE_INFO(SATA, 8, SATA, SATA),
	T210_POWERGATE_INFO(CPU1, 9, CPU1, CPU1),
	T210_POWERGATE_INFO(CPU2, 10, CPU2, CPU2),
	T210_POWERGATE_INFO(CPU3, 11, CPU3, CPU3),
	T210_POWERGATE_INFO(CELP, 12, CELP, CELP),
	T210_POWERGATE_INFO(3D1, 13, 3D1, 3D1),
	T210_POWERGATE_INFO(CPU0, 14, CPU0, CPU0),
	T210_POWERGATE_INFO(C0NC, 15, C0NC, C0NC),
	T210_POWERGATE_INFO(C1NC, 16, C1NC, C1NC),
	T210_POWERGATE_INFO(SOR, 17, SOR, SOR),
	T210_POWERGATE_INFO(DISA, 18, DISA, DISA),
	T210_POWERGATE_INFO(DISB, 19, DISB, DISB),
	T210_POWERGATE_INFO(XUSBA, 20, XUSBA, XUSBA),
	T210_POWERGATE_INFO(XUSBB, 21, XUSBB, XUSBB),
	T210_POWERGATE_INFO(XUSBC, 22, XUSBC, XUSBC),
	T210_POWERGATE_INFO(VIC, 23, VIC, VIC),
	T210_POWERGATE_INFO(NVDEC, 25, NVDEC, NVDEC),
	T210_POWERGATE_INFO(NVJPG, 26, NVJPG, NVJPG),
	T210_POWERGATE_INFO(APE, 27, APE, APE),
	T210_POWERGATE_INFO(VE2, 29, VE2, VE2),
};

#define PMC_GPU_RG_CONTROL		0x2d4

#define PWRGATE_CLAMP_STATUS		0x2c
#define PWRGATE_TOGGLE			0x30
#define PWRGATE_TOGGLE_START		(1 << 8)
#define REMOVE_CLAMPING			0x34
#define PWRGATE_STATUS			0x38

static DEFINE_SPINLOCK(tegra210_pg_lock);

static struct dvfs_rail *gpu_rail;
static void __iomem *tegra_mc;
static void __iomem *tegra_pmc;

static int tegra210_pg_powergate_partition(int id);
static int tegra210_pg_unpowergate_partition(int id);
static int tegra210_pg_mc_flush(int id);
static int tegra210_pg_mc_flush_done(int id);

static u32  __maybe_unused mc_read(unsigned long reg)
{
        return readl(tegra_mc + reg);
}

/* PMC register read/write */
static u32 pmc_read(unsigned long reg)
{
        return readl(tegra_pmc + reg);
}

static void pmc_write(u32 val, unsigned long reg)
{
        writel_relaxed(val, tegra_pmc + reg);
}

#define HOTRESET_READ_COUNTS		5

static const char *tegra210_pg_get_name(int id)
{
	return t210_pg_info[id].part_info->name;
}

static spinlock_t *tegra210_pg_get_lock(void)
{
	return &tegra210_pg_lock;
}

static bool tegra210_pg_skip(int id)
{
	u32 hid, chipid, major;

	hid = tegra_read_chipid();
	chipid = tegra_hidrev_get_chipid(hid);
	major = tegra_hidrev_get_majorrev(hid);

	switch (t210_pg_info[id].part_id) {
	case TEGRA210_POWER_DOMAIN_GPU:
		return true;
	case TEGRA210_POWER_DOMAIN_VENC:
	case TEGRA210_POWER_DOMAIN_VE2:
		/* T214 has SE2 in place of ISP2 and powergate
		 * is not supported for SE2.
		 */
		if (chipid == TEGRA210B01 && major >= 2)
			return true;
	default:
		return false;
	}
}

static bool tegra210_pg_is_powered(int id)
{
	u32 status = 0;

	if (id == TEGRA210_POWER_DOMAIN_GPU) {
		if (gpu_rail)
			status = tegra_dvfs_is_rail_up(gpu_rail);
	} else {
		status = pmc_read(PWRGATE_STATUS) & t210_pg_info[id].mask;
	}

	return !!status;
}

static int tegra_powergate_set(int id, bool new_state)
{
	bool status;
	unsigned long flags;
	spinlock_t *lock;
	u32 reg;

	/* 10us timeout for toggle operation if it takes affect*/
	int toggle_timeout = 10;

	/* 100 * 10 = 1000us timeout for toggle command to take affect in case
	   of contention with h/w initiated CPU power gating */
	int contention_timeout = 100;

	if (tegra_cpu_is_asim())
		return 0;

	lock = tegra210_pg_get_lock();

	spin_lock_irqsave(lock, flags);

	status = !!(pmc_read(PWRGATE_STATUS) & t210_pg_info[id].mask);

	if (status == new_state) {
		spin_unlock_irqrestore(lock, flags);
		return 0;
	}

	switch (id) {
	case TEGRA210_POWER_DOMAIN_CPU0:
	case TEGRA210_POWER_DOMAIN_CPU1:
	case TEGRA210_POWER_DOMAIN_CPU2:
	case TEGRA210_POWER_DOMAIN_CPU3:
		/* CPU ungated in s/w only during boot/resume with outer
		   waiting loop and no contention from other CPUs */
		pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);
		pmc_read(PWRGATE_TOGGLE);
		spin_unlock_irqrestore(lock, flags);
		return 0;

	default:
		break;
	}

	/* Wait if PMC is already processing some other power gating request */
	do {
		udelay(1);
		reg = pmc_read(PWRGATE_TOGGLE);
		contention_timeout--;
	} while ((contention_timeout > 0) && (reg & PWRGATE_TOGGLE_START));

	if (contention_timeout <= 0)
		pr_err(" Timed out waiting for PMC to submit \
				new power gate request \n");
	contention_timeout = 100;

	/* Submit power gate request */
	pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);

	/* Wait while PMC accepts the request */
	do {
		udelay(1);
		reg = pmc_read(PWRGATE_TOGGLE);
		contention_timeout--;
	} while ((contention_timeout > 0) && (reg & PWRGATE_TOGGLE_START));

	if (contention_timeout <= 0)
		pr_err(" Timed out waiting for PMC to accept \
				new power gate request \n");
	contention_timeout = 100;

	/* Check power gate status */
	do {
		do {
			udelay(1);
			status = !!(pmc_read(PWRGATE_STATUS) &
				    t210_pg_info[id].mask);

			toggle_timeout--;
		} while ((status != new_state) && (toggle_timeout > 0));

		toggle_timeout = 10;
		contention_timeout--;
	} while ((status != new_state) && (contention_timeout > 0));

	spin_unlock_irqrestore(lock, flags);

	if (status != new_state) {
		WARN(1, "Could not set powergate %d to %d", id, new_state);
		return -EBUSY;
	}

	trace_power_domain_target(tegra210_pg_get_name(id), new_state,
			raw_smp_processor_id());

	return 0;
}

static const char *clk_get_name(struct clk *clk)
{
	return __clk_get_name(clk);
}

static int powergate_module(int id)
{
	tegra210_pg_mc_flush(id);

	return tegra_powergate_set(id, false);
}

static int partition_clk_enable(struct powergate_partition_info *pg_info)
{
	int ret;
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *clk_info;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &pg_info->clk_info[idx];
		clk = clk_info->clk_ptr;
		if (IS_ERR(clk) || !clk)
			break;

		if (clk_info->clk_type != RST_ONLY) {
			ret = clk_prepare_enable(clk);
			if (ret)
				goto err_clk_en;
		}
	}

	return 0;

err_clk_en:
	WARN(1, "Could not enable clk %s, error %d", clk_get_name(clk), ret);
	while (idx--) {
		clk_info = &pg_info->clk_info[idx];
		if (clk_info->clk_type != RST_ONLY)
			clk_disable_unprepare(clk_info->clk_ptr);
	}

	return ret;
}

static void partition_clk_disable(struct powergate_partition_info *pg_info)
{
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *clk_info;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &pg_info->clk_info[idx];
		clk = clk_info->clk_ptr;

		if (IS_ERR(clk) || !clk)
			break;

		if (clk_info->clk_type != RST_ONLY)
			clk_disable_unprepare(clk);
	}
}

static void get_clk_info(struct powergate_partition_info *pg_info)
{
	int idx;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		if (!pg_info->clk_info[idx].clk_name)
			break;

		pg_info->clk_info[idx].clk_ptr =
			clk_get_sys(NULL, pg_info->clk_info[idx].clk_name);

		if (IS_ERR_OR_NULL(pg_info->clk_info[idx].clk_ptr))
			WARN(1, "Could not find clock %s for %s partition\n",
				pg_info->clk_info[idx].clk_name,
				pg_info->name);
	}
}

static void powergate_partition_assert_reset(struct powergate_partition_info *pg_info)
{
	tegra_rst_assertv(&pg_info->reset_id[0], pg_info->reset_id_num);
}

static void powergate_partition_deassert_reset(struct powergate_partition_info *pg_info)
{
	tegra_rst_deassertv(&pg_info->reset_id[0], pg_info->reset_id_num);
}

static int slcg_clk_enable(struct powergate_partition_info *pg_info)
{
	int ret;
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *slcg_info;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		slcg_info = &pg_info->slcg_info[idx];
		clk = slcg_info->clk_ptr;
		if (IS_ERR(clk) || !clk)
			break;

		ret = clk_prepare_enable(clk);
		if (ret)
			goto err_clk_en;
	}

	return 0;

err_clk_en:
	WARN(1, "Could not enable clk %s, error %d", __clk_get_name(clk), ret);
	while (idx--) {
		slcg_info = &pg_info->slcg_info[idx];
		clk_disable_unprepare(slcg_info->clk_ptr);
	}

	return ret;
}

static void slcg_clk_disable(struct powergate_partition_info *pg_info)
{
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *slcg_info;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		slcg_info = &pg_info->slcg_info[idx];
		clk = slcg_info->clk_ptr;

		if (IS_ERR(clk) || !clk)
			break;

		clk_disable_unprepare(clk);
	}
}

static void get_slcg_info(struct powergate_partition_info *pg_info)
{
	int idx;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		if (!pg_info->slcg_info[idx].clk_name)
			break;

		pg_info->slcg_info[idx].clk_ptr =
			clk_get_sys(NULL, pg_info->slcg_info[idx].clk_name);

		if (IS_ERR_OR_NULL(pg_info->slcg_info[idx].clk_ptr))
			pr_err("### Could not find clock %s for %s partition\n",
				pg_info->slcg_info[idx].clk_name,
				pg_info->name);
	}
}

static int tegra210_slcg_register_notifier(int id, struct notifier_block *nb)
{
	struct powergate_partition_info *pg_info = t210_pg_info[id].part_info;

	if (!pg_info || !nb)
		return -EINVAL;

	return raw_notifier_chain_register(&pg_info->slcg_notifier, nb);
}

static int tegra210_slcg_unregister_notifier(int id, struct notifier_block *nb)
{
	struct powergate_partition_info *pg_info = t210_pg_info[id].part_info;

	if (!pg_info || !nb)
		return -EINVAL;

	return raw_notifier_chain_unregister(&pg_info->slcg_notifier, nb);
}

static int tegra210_powergate_remove_clamping(int id)
{
	u32 mask;
	int contention_timeout = 100;

	/*
	 * PCIE and VDE clamping masks are swapped with respect to their
	 * partition ids
	 */
	if (id ==  TEGRA210_POWER_DOMAIN_VDEC)
		mask = t210_pg_info[TEGRA210_POWER_DOMAIN_PCIE].mask;
	else if (id == TEGRA210_POWER_DOMAIN_PCIE)
		mask = t210_pg_info[TEGRA210_POWER_DOMAIN_VDEC].mask;
	else
		mask = t210_pg_info[id].mask;

	pmc_write(mask, REMOVE_CLAMPING);
	/* Wait until clamp is removed */
	do {
		udelay(1);
		contention_timeout--;
	} while ((contention_timeout > 0)
			&& (pmc_read(PWRGATE_CLAMP_STATUS) & BIT(id)));

	WARN(contention_timeout <= 0, "Couldn't remove clamping");

	return 0;
}

static int __tegra1xx_powergate(int id, struct powergate_partition_info *pg_info,
				bool clk_enable)
{
	int ret;

	/* If first clk_ptr is null, fill clk info for the partition */
	if (!pg_info->clk_info[0].clk_ptr)
		get_clk_info(pg_info);

	if (clk_enable) {
		/*
		 * Enable clocks only if clocks are not expected to
		 * be off when power gating is done
		 */
		ret = partition_clk_enable(pg_info);
		if (ret) {
			WARN(1, "Couldn't enable clock");
			return ret;
		}

		udelay(10);
	}

	tegra210_pg_mc_flush(id);

	udelay(10);

	powergate_partition_assert_reset(pg_info);

	udelay(10);

	/* Powergating is done only if refcnt of all clks is 0 */
	partition_clk_disable(pg_info);

	udelay(10);

	ret = tegra_powergate_set(id, false);
	if (ret)
		goto err_power_off;

	return 0;

err_power_off:
	WARN(1, "Could not Powergate Partition %d", id);
	return ret;
}

static int tegra1xx_powergate(int id, struct powergate_partition_info *pg_info)
{
	return __tegra1xx_powergate(id, pg_info, true);
}

static int __tegra1xx_unpowergate(int id, struct powergate_partition_info *pg_info,
				bool clk_disable)
{
	int ret;

	/* If first clk_ptr is null, fill clk info for the partition */
	if (!pg_info->clk_info[0].clk_ptr)
		get_clk_info(pg_info);

	if (!pg_info->slcg_info[0].clk_ptr)
		get_slcg_info(pg_info);

	if (tegra210_pg_is_powered(id)) {
		if (!clk_disable) {
			ret = partition_clk_enable(pg_info);
			if (ret)
				return ret;
			if (!pg_info->skip_reset) {
				powergate_partition_assert_reset(pg_info);
				udelay(10);
				powergate_partition_deassert_reset(pg_info);
			}
		}
		return 0;
	}

	ret = tegra_powergate_set(id, true);
	if (ret)
		goto err_power;

	udelay(10);

	/* Un-Powergating fails if all clks are not enabled */
	ret = partition_clk_enable(pg_info);
	if (ret)
		goto err_clk_on;

	powergate_partition_assert_reset(pg_info);

	udelay(10);

	ret = tegra210_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;

	udelay(10);

	if (!pg_info->skip_reset) {
		powergate_partition_deassert_reset(pg_info);

		udelay(10);
	}

	tegra210_pg_mc_flush_done(id);

	udelay(10);

	slcg_clk_enable(pg_info);

	raw_notifier_call_chain(&pg_info->slcg_notifier, 0, NULL);

	slcg_clk_disable(pg_info);

	/* Disable all clks enabled earlier. Drivers should enable clks */
	if (clk_disable)
		partition_clk_disable(pg_info);

	return 0;

err_clamp:
	partition_clk_disable(pg_info);
err_clk_on:
	powergate_module(id);
err_power:
	WARN(1, "Could not Un-Powergate %d", id);
	return ret;
}

static int tegra1xx_unpowergate(int id,
				struct powergate_partition_info *pg_info)
{
	return __tegra1xx_unpowergate(id, pg_info, true);
}

static int tegra1xx_powergate_partition_with_clk_off(int id,
		struct powergate_partition_info *pg_info)
{
	int ret = 0;

	ret = __tegra1xx_powergate(id, pg_info, false);
	if (ret)
		WARN(1, "Could not Powergate Partition %d", id);

	return ret;
}

static int tegra1xx_unpowergate_partition_with_clk_on(int id,
	struct powergate_partition_info *pg_info)
{
	int ret = 0;

	ret = __tegra1xx_unpowergate(id, pg_info, false);
	if (ret)
		WARN(1, "Could not Un-Powergate %d", id);

	return ret;
}

static int tegra210_pg_mc_flush(int id)
{
	u32 idx;
	enum mc_client_type mc_client_bit;
	int ret = EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mc_client_bit =
			t210_pg_info[id].mc_info->hot_reset_clients[idx];
		if (mc_client_bit == MC_CLIENT_LAST)
			break;
		ret = tegra_mc_flush(mc_client_bit);
		if (ret)
			break;
	}

	return ret;
}

static int tegra210_pg_mc_flush_done(int id)
{
	u32 idx;
	enum mc_client_type mc_client_bit;
	int ret = -EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mc_client_bit =
			t210_pg_info[id].mc_info->hot_reset_clients[idx];
		if (mc_client_bit == MC_CLIENT_LAST)
			break;
		ret = tegra_mc_flush_done(mc_client_bit);
		if (ret)
			break;
	}
	wmb();

	return ret;
}

static int tegra210_pg_powergate(int id)
{
	struct powergate_partition_info *partition = t210_pg_info[id].part_info;
	int ret = 0;

	trace_powergate(__func__, tegra210_pg_get_name(id), id, 1, 0);
	mutex_lock(&partition->pg_mutex);

	if (--partition->refcount > 0)
		goto exit_unlock;

	if ((partition->refcount < 0) || !tegra210_pg_is_powered(id)) {
		WARN(1, "Partition %s already powergated, refcount and status mismatch\n",
		     partition->name);
		goto exit_unlock;
	}

	ret = tegra1xx_powergate(id, partition);

exit_unlock:
	mutex_unlock(&partition->pg_mutex);
	trace_powergate(__func__, tegra210_pg_get_name(id), id, 0, ret);
	return ret;
}

static int tegra210_pg_unpowergate(int id)
{
	struct powergate_partition_info *partition = t210_pg_info[id].part_info;
	int ret = 0;

	trace_powergate(__func__, tegra210_pg_get_name(id), id, 1, 0);
	mutex_lock(&partition->pg_mutex);

	if (partition->refcount++ > 0)
		goto exit_unlock;

	if (tegra210_pg_is_powered(id)) {
		WARN(1, "Partition %s is already unpowergated, refcount and status mismatch\n",
		     partition->name);
		goto exit_unlock;
	}

	ret = tegra1xx_unpowergate(id, partition);

exit_unlock:
	mutex_unlock(&partition->pg_mutex);
	trace_powergate(__func__, tegra210_pg_get_name(id), id, 0, ret);
	return ret;
}

static int tegra210_pg_gpu_powergate(int id)
{
	int ret = 0;
	struct powergate_partition_info *partition = t210_pg_info[id].part_info;

	trace_powergate(__func__, tegra210_pg_get_name(id), id, 1, 0);
	mutex_lock(&partition->pg_mutex);

	if (--partition->refcount > 0)
		goto exit_unlock;

	if (!tegra210_pg_is_powered(id)) {
		WARN(1, "GPU rail is already off, refcount and status mismatch\n");
		goto exit_unlock;
	}

	if (!partition->clk_info[0].clk_ptr)
		get_clk_info(partition);

	tegra210_pg_mc_flush(id);

	udelay(10);

	powergate_partition_assert_reset(partition);

	udelay(10);

	pmc_write(0x1, PMC_GPU_RG_CONTROL);
	pmc_read(PMC_GPU_RG_CONTROL);

	udelay(10);

	partition_clk_disable(partition);

	udelay(10);

	tegra_soctherm_gpu_tsens_invalidate(1);

	if (gpu_rail) {
		ret = tegra_dvfs_rail_power_down(gpu_rail);
		if (ret) {
			WARN(1, "Could not power down GPU rail\n");
			goto exit_unlock;
		}
	} else {
		pr_info("No GPU regulator?\n");
	}

exit_unlock:
	mutex_unlock(&partition->pg_mutex);
	trace_powergate(__func__, tegra210_pg_get_name(id), id, 0, ret);
	return ret;
}

static int tegra210_pg_gpu_unpowergate(int id)
{
	int ret = 0;
	bool first = false;
	struct powergate_partition_info *partition = t210_pg_info[id].part_info;

	trace_powergate(__func__, tegra210_pg_get_name(id), id, 1, 0);
	mutex_lock(&partition->pg_mutex);

	if (partition->refcount++ > 0)
		goto exit_unlock;

	if (!gpu_rail) {
		gpu_rail = tegra_dvfs_get_rail_by_name("vdd_gpu");
		if (IS_ERR_OR_NULL(gpu_rail)) {
			WARN(1, "No GPU regulator?\n");
			goto err_power;
		}
		first = true;
	}

	if (tegra210_pg_is_powered(id)) {
		WARN(1, "GPU rail is already on, refcount and status mismatch\n");
		goto exit_unlock;
	}

	ret = tegra_dvfs_rail_power_up(gpu_rail);
	if (ret) {
		WARN(1, "Could not turn on GPU rail\n");
		goto err_power;
	}

	tegra_soctherm_gpu_tsens_invalidate(0);

	if (!partition->clk_info[0].clk_ptr)
		get_clk_info(partition);

	if (!first) {
		ret = partition_clk_enable(partition);
		if (ret) {
			WARN(1, "Could not turn on partition clocks\n");
			goto err_clk_on;
		}
	}

	udelay(10);

	powergate_partition_assert_reset(partition);

	udelay(10);

	pmc_write(0, PMC_GPU_RG_CONTROL);
	pmc_read(PMC_GPU_RG_CONTROL);

	udelay(10);

	/*
	 * Make sure all clok branches into GPU, except reference clock are
	 * gated across resert de-assertion.
	 */
	clk_disable_unprepare(partition->clk_info[0].clk_ptr);
	powergate_partition_deassert_reset(partition);
	clk_prepare_enable(partition->clk_info[0].clk_ptr);

	/* Flush MC after boot/railgate/SC7 */
	tegra210_pg_mc_flush(id);

	udelay(10);

	tegra210_pg_mc_flush_done(id);

	udelay(10);

exit_unlock:
	mutex_unlock(&partition->pg_mutex);
	trace_powergate(__func__, tegra210_pg_get_name(id), id, 0, ret);
	return ret;

err_clk_on:
	powergate_module(id);
err_power:
	mutex_unlock(&partition->pg_mutex);

	trace_powergate(__func__, tegra210_pg_get_name(id), id, 0, ret);
	return ret;
}

static int tegra210_pg_powergate_sor(int id)
{
	int ret;

	ret = tegra210_pg_powergate(id);
	if (ret)
		return ret;

	ret = tegra210_pg_powergate_partition(TEGRA210_POWER_DOMAIN_SOR);
	if (ret)
		return ret;

	return 0;
}

static int tegra210_pg_unpowergate_sor(int id)
{
	int ret;

	ret = tegra210_pg_unpowergate_partition(TEGRA210_POWER_DOMAIN_SOR);
	if (ret)
		return ret;

	ret = tegra210_pg_unpowergate(id);
	if (ret) {
		tegra210_pg_powergate_partition(TEGRA210_POWER_DOMAIN_SOR);
		return ret;
	}

	return 0;
}

static int tegra210_pg_nvdec_powergate(int id)
{
	tegra210_pg_powergate(TEGRA210_POWER_DOMAIN_NVDEC);
	tegra210_pg_powergate_partition(TEGRA210_POWER_DOMAIN_NVJPG);

	return 0;
}

static int tegra210_pg_nvdec_unpowergate(int id)
{
	tegra210_pg_unpowergate_partition(TEGRA210_POWER_DOMAIN_NVJPG);
	tegra210_pg_unpowergate(TEGRA210_POWER_DOMAIN_NVDEC);

	return 0;
}

static int tegra210_pg_sata_powergate(int id)
{
	tegra210_set_sata_pll_seq_sw(true);
	tegra210_pg_powergate(TEGRA210_POWER_DOMAIN_SATA);

	return 0;
}

static int tegra210_pg_sata_unpowergate(int id)
{
	tegra210_pg_unpowergate(TEGRA210_POWER_DOMAIN_SATA);
	tegra210_set_sata_pll_seq_sw(false);

	return 0;
}

static int tegra210_pg_powergate_partition(int id)
{
	int ret;

	switch (t210_pg_info[id].part_id) {
		case TEGRA210_POWER_DOMAIN_GPU:
			ret = tegra210_pg_gpu_powergate(id);
			break;
		case TEGRA210_POWER_DOMAIN_DISA:
		case TEGRA210_POWER_DOMAIN_DISB:
		case TEGRA210_POWER_DOMAIN_VENC:
			ret = tegra210_pg_powergate_sor(id);
			break;
		case TEGRA210_POWER_DOMAIN_NVDEC:
			ret = tegra210_pg_nvdec_powergate(id);
			break;
		case TEGRA210_POWER_DOMAIN_SATA:
			ret = tegra210_pg_sata_powergate(id);
			break;
		default:
			ret = tegra210_pg_powergate(id);
	}

	return ret;
}

static int tegra210_pg_unpowergate_partition(int id)
{
	int ret;

	switch (t210_pg_info[id].part_id) {
		case TEGRA210_POWER_DOMAIN_GPU:
			ret = tegra210_pg_gpu_unpowergate(id);
			break;
		case TEGRA210_POWER_DOMAIN_DISA:
		case TEGRA210_POWER_DOMAIN_DISB:
		case TEGRA210_POWER_DOMAIN_VENC:
			ret = tegra210_pg_unpowergate_sor(id);
			break;
		case TEGRA210_POWER_DOMAIN_NVDEC:
			ret = tegra210_pg_nvdec_unpowergate(id);
			break;
		case TEGRA210_POWER_DOMAIN_SATA:
			ret = tegra210_pg_sata_unpowergate(id);
			break;
		default:
			ret = tegra210_pg_unpowergate(id);
	}

	return ret;
}

static int tegra210_pg_powergate_clk_off(int id)
{
	int ret = 0;
	struct powergate_partition_info *partition = t210_pg_info[id].part_info;

	BUG_ON(id == TEGRA210_POWER_DOMAIN_GPU);

	trace_powergate(__func__, tegra210_pg_get_name(id), id, 1, 0);
	mutex_lock(&partition->pg_mutex);

	if (--partition->refcount > 0)
		goto exit_unlock;

	if ((partition->refcount < 0) || !tegra210_pg_is_powered(id)) {
		WARN(1, "Partition %s already powergated, refcount and status mismatch\n",
		     partition->name);
		goto exit_unlock;
	}

	if (t210_pg_info[id].part_id == TEGRA210_POWER_DOMAIN_SATA)
		tegra210_set_sata_pll_seq_sw(true);

	ret = tegra1xx_powergate_partition_with_clk_off(id,
			t210_pg_info[id].part_info);

exit_unlock:
	mutex_unlock(&partition->pg_mutex);
	trace_powergate(__func__, tegra210_pg_get_name(id), id, 0, ret);

	return ret;
}

static int tegra210_pg_unpowergate_clk_on(int id)
{
	int ret = 0;
	struct powergate_partition_info *partition = t210_pg_info[id].part_info;

	BUG_ON(id == TEGRA210_POWER_DOMAIN_GPU);
	trace_powergate(__func__, tegra210_pg_get_name(id), id, 1, 0);
	mutex_lock(&partition->pg_mutex);

	if (partition->refcount++ > 0)
		goto exit_unlock;

	ret = tegra1xx_unpowergate_partition_with_clk_on(id,
			t210_pg_info[id].part_info);

	if (t210_pg_info[id].part_id == TEGRA210_POWER_DOMAIN_SATA)
		tegra210_set_sata_pll_seq_sw(false);

exit_unlock:
	mutex_unlock(&partition->pg_mutex);
	trace_powergate(__func__, tegra210_pg_get_name(id), id, 0, ret);

	return ret;
}

static int tegra210_pg_init_refcount(void)
{
	int i;

	for (i = 0; i < TEGRA210_POWER_DOMAIN_MAX; i++) {
		if (!t210_pg_info[i].valid)
			continue;

		/* Consider main partion ID only */
		if (i != t210_pg_info[i].part_id)
			continue;

		if (tegra210_pg_is_powered(i))
			t210_pg_info[i].part_info->refcount = 1;
		else
			t210_pg_info[i].part_info->disable_after_boot = 0;

		mutex_init(&t210_pg_info[i].part_info->pg_mutex);
	}

	/* SOR refcount depends on other units */
	t210_pg_info[TEGRA210_POWER_DOMAIN_SOR].part_info->refcount =
		(tegra210_pg_is_powered(TEGRA210_POWER_DOMAIN_DISA) ? 1 : 0) +
		(tegra210_pg_is_powered(TEGRA210_POWER_DOMAIN_DISB) ? 1 : 0) +
		(tegra210_pg_is_powered(TEGRA210_POWER_DOMAIN_VENC) ? 1 : 0);

	tegra210_pg_powergate_partition(TEGRA210_POWER_DOMAIN_XUSBA);
	tegra210_pg_powergate_partition(TEGRA210_POWER_DOMAIN_XUSBB);
	tegra210_pg_powergate_partition(TEGRA210_POWER_DOMAIN_XUSBC);

	return 0;
}

static bool tegra210_powergate_id_is_valid(int id)
{
	if ((id < 0) || (id >= TEGRA210_POWER_DOMAIN_MAX))
		return false;

	return t210_pg_info[id].valid;
}

static int tegra210_powergate_cpuid_to_powergate_id(int cpu)
{
	switch (cpu) {
	case 0:
		return TEGRA210_POWER_DOMAIN_CPU0;
	case 1:
		return TEGRA210_POWER_DOMAIN_CPU1;
	case 2:
		return TEGRA210_POWER_DOMAIN_CPU2;
	case 3:
		return TEGRA210_POWER_DOMAIN_CPU3;
	default:
		return -1;
	}

	return -1;
}

static struct tegra_powergate_driver_ops tegra210_pg_ops = {
	.soc_name = "tegra210",

	.num_powerdomains = TEGRA210_POWER_DOMAIN_MAX,
	.powergate_id_is_soc_valid = tegra210_powergate_id_is_valid,
	.powergate_cpuid_to_powergate_id =
				tegra210_powergate_cpuid_to_powergate_id,

	.get_powergate_lock = tegra210_pg_get_lock,
	.get_powergate_domain_name = tegra210_pg_get_name,

	.powergate_partition = tegra210_pg_powergate_partition,
	.unpowergate_partition = tegra210_pg_unpowergate_partition,

	.powergate_partition_with_clk_off = tegra210_pg_powergate_clk_off,
	.unpowergate_partition_with_clk_on = tegra210_pg_unpowergate_clk_on,

	.powergate_mc_flush = tegra210_pg_mc_flush,
	.powergate_mc_flush_done = tegra210_pg_mc_flush_done,

	.powergate_skip = tegra210_pg_skip,

	.powergate_is_powered = tegra210_pg_is_powered,

	.powergate_init_refcount = tegra210_pg_init_refcount,
	.powergate_remove_clamping = tegra210_powergate_remove_clamping,
	.slcg_register_notifier = tegra210_slcg_register_notifier,
	.slcg_unregister_notifier = tegra210_slcg_unregister_notifier,
};

#define TEGRA_PMC_BASE  0x7000E400
#define TEGRA_MC_BASE   0x70019000
struct tegra_powergate_driver_ops *tegra210_powergate_init_chip_support(void)
{
	tegra_pmc = ioremap(TEGRA_PMC_BASE, 4096);
	tegra_mc = ioremap(TEGRA_MC_BASE, 4096);

	return &tegra210_pg_ops;
}

static int __init tegra210_disable_boot_partitions(void)
{
	int i;

	if (!soc_is_tegra210_n_before())
		return 0;

	pr_info("Disable partitions left on by BL\n");
	for (i = 0; i < TEGRA210_POWER_DOMAIN_MAX; i++) {
		if (!t210_pg_info[i].valid)
			continue;

		/* consider main partion ID only */
		if (i != t210_pg_info[i].part_id)
			continue;

		if (t210_pg_info[i].part_info->disable_after_boot &&
			(t210_pg_info[i].part_id != TEGRA210_POWER_DOMAIN_GPU)) {
			pr_info("  %s\n", t210_pg_info[i].part_info->name);
			tegra210_pg_powergate_partition(i);
		}
	}

	return 0;
}
late_initcall(tegra210_disable_boot_partitions);
