/*
 * drivers/soc/tegra/pmc.c
 *
 * Copyright (c) 2010 Google, Inc
 * Copyright (c) 2012-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#define pr_fmt(fmt) "tegra-pmc: " fmt

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/psci.h>
#include <linux/reboot.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tegra_prod.h>
#include <linux/tegra-soc.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>

#include <soc/tegra/common.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/pmc.h>

#include <asm/system_misc.h>

#include <dt-bindings/pinctrl/pinctrl-tegra-io-pad.h>

#define PMC_CNTRL			0x0
#define  PMC_CNTRL_SYSCLK_POLARITY	(1 << 10)  /* sys clk polarity */
#define  PMC_CNTRL_SYSCLK_OE		(1 << 11)  /* system clock enable */
#define  PMC_CNTRL_SIDE_EFFECT_LP0	(1 << 14)  /* LP0 when CPU pwr gated */
#define  PMC_CNTRL_CPU_PWRREQ_POLARITY	(1 << 15)  /* CPU pwr req polarity */
#define  PMC_CNTRL_CPU_PWRREQ_OE	(1 << 16)  /* CPU pwr req enable */
#define  PMC_CNTRL_INTR_POLARITY	(1 << 17)  /* inverts INTR polarity */
#define  PMC_CNTRL_MAIN_RST		(1 <<  4)

#define DPD_SAMPLE			0x020
#define  DPD_SAMPLE_ENABLE		(1 << 0)
#define  DPD_SAMPLE_DISABLE		(0 << 0)

#define PWRGATE_TOGGLE			0x30
#define  PWRGATE_TOGGLE_START		(1 << 8)

#define REMOVE_CLAMPING			0x34

#define PWRGATE_STATUS			0x38

#define PMC_PWR_DET_ENABLE		0x48

#define PMC_SCRATCH0			0x50
#define  PMC_SCRATCH0_MODE_RECOVERY	(1 << 31)
#define  PMC_SCRATCH0_MODE_BOOTLOADER	(1 << 30)
#define  PMC_SCRATCH0_MODE_RCM		(1 << 1)
#define  PMC_SCRATCH0_MODE_MASK		(PMC_SCRATCH0_MODE_RECOVERY | \
					 PMC_SCRATCH0_MODE_BOOTLOADER | \
					 PMC_SCRATCH0_MODE_RCM)

#define PMC_CPUPWRGOOD_TIMER		0xc8
#define PMC_CPUPWROFF_TIMER		0xcc

#define PMC_PWR_DET_VAL			0xe4

#define PMC_SCRATCH41			0x140

#define PMC_SENSOR_CTRL			0x1b0
#define PMC_SENSOR_CTRL_SCRATCH_WRITE	(1 << 2)
#define PMC_SENSOR_CTRL_ENABLE_RST	(1 << 1)

#define PMC_RST_STATUS			0x1b4
#define  PMC_RST_STATUS_POR		0
#define  PMC_RST_STATUS_WATCHDOG	1
#define  PMC_RST_STATUS_SENSOR		2
#define  PMC_RST_STATUS_SW_MAIN		3
#define  PMC_RST_STATUS_LP0		4
#define  PMC_RST_STATUS_AOTAG		5

#define IO_DPD_REQ			0x1b8
#define  IO_DPD_REQ_CODE_IDLE		(0 << 30)
#define  IO_DPD_REQ_CODE_OFF		(1 << 30)
#define  IO_DPD_REQ_CODE_ON		(2 << 30)
#define  IO_DPD_REQ_CODE_MASK		(3 << 30)
#define  IO_DPD_ENABLE_LSB		30

#define IO_DPD_STATUS			0x1bc

#define IO_DPD2_REQ			0x1c0
#define  IO_DPD2_ENABLE_LSB		30

#define IO_DPD2_STATUS			0x1c4
#define SEL_DPD_TIM			0x1c8

#define PMC_SCRATCH54			0x258
#define PMC_SCRATCH54_DATA_SHIFT	8
#define PMC_SCRATCH54_ADDR_SHIFT	0

#define PMC_SCRATCH55			0x25c
#define PMC_SCRATCH55_RESET_TEGRA	(1 << 31)
#define PMC_SCRATCH55_CNTRL_ID_SHIFT	27
#define PMC_SCRATCH55_PINMUX_SHIFT	24
#define PMC_SCRATCH55_16BITOP		(1 << 15)
#define PMC_SCRATCH55_CHECKSUM_SHIFT	16
#define PMC_SCRATCH55_I2CSLV1_SHIFT	0

#define GPU_RG_CNTRL			0x2d4

struct tegra_powergate {
	struct generic_pm_domain genpd;
	struct tegra_pmc *pmc;
	unsigned int id;
	struct clk **clks;
	unsigned int num_clks;
	struct reset_control **resets;
	unsigned int num_resets;
};

#define PMC_FUSE_CTRL                   0x450
#define PMC_FUSE_CTRL_ENABLE_REDIRECTION	(1 << 0)
#define PMC_FUSE_CTRL_DISABLE_REDIRECTION	(1 << 1)
#define PMC_FUSE_CTRL_PS18_LATCH_SET    (1 << 8)
#define PMC_FUSE_CTRL_PS18_LATCH_CLEAR  (1 << 9)

/* Scratch 250: Bootrom i2c command base */
#define PMC_BR_COMMAND_BASE		0x908

/* USB2 SLEEPWALK registers */
#define UTMIP(_port, _offset1, _offset2) \
		(((_port) <= 2) ? (_offset1) : (_offset2))

#define APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(x)	UTMIP(x, 0x1fc, 0x4d0)
#define   UTMIP_MASTER_ENABLE(x)		UTMIP(x, BIT(8 * (x)), BIT(0))
#define   UTMIP_FSLS_USE_PMC(x)			UTMIP(x, BIT(8 * (x) + 1), \
							BIT(1))
#define   UTMIP_PCTRL_USE_PMC(x)		UTMIP(x, BIT(8 * (x) + 2), \
							BIT(2))
#define   UTMIP_TCTRL_USE_PMC(x)		UTMIP(x, BIT(8 * (x) + 3), \
							BIT(3))
#define   UTMIP_WAKE_VAL(_port, _value)		(((_value) & 0xf) << \
					(UTMIP(_port, 8 * (_port) + 4, 4)))
#define   UTMIP_WAKE_VAL_NONE(_port)		UTMIP_WAKE_VAL(_port, 12)
#define   UTMIP_WAKE_VAL_ANY(_port)		UTMIP_WAKE_VAL(_port, 15)

#define APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG1	(0x4d0)
#define   UTMIP_RPU_SWITC_LOW_USE_PMC_PX(x)	BIT((x) + 8)
#define   UTMIP_RPD_CTRL_USE_PMC_PX(x)		BIT((x) + 16)

#define APBDEV_PMC_UTMIP_MASTER_CONFIG		(0x274)
#define   UTMIP_PWR(x)				UTMIP(x, BIT(x), BIT(4))
#define   UHSIC_PWR(x)				BIT(3)

#define APBDEV_PMC_USB_DEBOUNCE_DEL		(0xec)
#define   DEBOUNCE_VAL(x)			(((x) & 0xffff) << 0)
#define   UTMIP_LINE_DEB_CNT(x)			(((x) & 0xf) << 16)
#define   UHSIC_LINE_DEB_CNT(x)			(((x) & 0xf) << 20)

#define APBDEV_PMC_UTMIP_UHSIC_FAKE(x)		UTMIP(x, 0x218, 0x294)
#define   UTMIP_FAKE_USBOP_VAL(x)		UTMIP(x, BIT(4 * (x)), BIT(8))
#define   UTMIP_FAKE_USBON_VAL(x)		UTMIP(x, BIT(4 * (x) + 1), \
							BIT(9))
#define   UTMIP_FAKE_USBOP_EN(x)		UTMIP(x, BIT(4 * (x) + 2), \
							BIT(10))
#define   UTMIP_FAKE_USBON_EN(x)		UTMIP(x, BIT(4 * (x) + 3), \
							BIT(11))

#define APBDEV_PMC_UTMIP_UHSIC_SLEEPWALK_CFG(x)	UTMIP(x, 0x200, 0x288)
#define   UTMIP_WAKE_WALK_EN(x)			UTMIP(x, BIT(8 * (x) + 6), \
							BIT(14))
#define   UTMIP_LINEVAL_WALK_EN(x)		UTMIP(x, BIT(8 * (x) + 7), \
							BIT(15))

#define APBDEV_PMC_USB_AO			(0xf0)
#define   USBOP_VAL_PD(x)			UTMIP(x, BIT(4 * (x)), BIT(20))
#define   USBON_VAL_PD(x)			UTMIP(x, BIT(4 * (x) + 1), \
							BIT(21))
#define   STROBE_VAL_PD(x)			BIT(12)
#define   DATA0_VAL_PD(x)			BIT(13)
#define   DATA1_VAL_PD				BIT(24)

#define APBDEV_PMC_UTMIP_UHSIC_SAVED_STATE(x)	UTMIP(x, 0x1f0, 0x280)
#define   SPEED(_port, _value)			(((_value) & 0x3) << \
						(UTMIP(_port, 8 * (_port), 8)))
#define   UTMI_HS(_port)			SPEED(_port, 0)
#define   UTMI_FS(_port)			SPEED(_port, 1)
#define   UTMI_LS(_port)			SPEED(_port, 2)
#define   UTMI_RST(_port)			SPEED(_port, 3)

#define APBDEV_PMC_UTMIP_UHSIC_TRIGGERS		(0x1ec)
#define   UTMIP_CLR_WALK_PTR(x)			UTMIP(x, BIT(x), BIT(16))
#define   UTMIP_CAP_CFG(x)			UTMIP(x, BIT((x) + 4), BIT(17))
#define   UTMIP_CLR_WAKE_ALARM(x)		UTMIP(x, BIT((x) + 12), \
							BIT(19))
#define   UHSIC_CLR_WALK_PTR			BIT(3)
#define   UHSIC_CLR_WAKE_ALARM			BIT(15)

#define APBDEV_PMC_UTMIP_SLEEPWALK_PX(x)	UTMIP(x, 0x204 + (4 * (x)), \
							0x4e0)
/* phase A */
#define   UTMIP_USBOP_RPD_A			BIT(0)
#define   UTMIP_USBON_RPD_A			BIT(1)
#define   UTMIP_AP_A				BIT(4)
#define   UTMIP_AN_A				BIT(5)
#define   UTMIP_HIGHZ_A				BIT(6)
/* phase B */
#define   UTMIP_USBOP_RPD_B			BIT(8)
#define   UTMIP_USBON_RPD_B			BIT(9)
#define   UTMIP_AP_B				BIT(12)
#define   UTMIP_AN_B				BIT(13)
#define   UTMIP_HIGHZ_B				BIT(14)
/* phase C */
#define   UTMIP_USBOP_RPD_C			BIT(16)
#define   UTMIP_USBON_RPD_C			BIT(17)
#define   UTMIP_AP_C				BIT(20)
#define   UTMIP_AN_C				BIT(21)
#define   UTMIP_HIGHZ_C				BIT(22)
/* phase D */
#define   UTMIP_USBOP_RPD_D			BIT(24)
#define   UTMIP_USBON_RPD_D			BIT(25)
#define   UTMIP_AP_D				BIT(28)
#define   UTMIP_AN_D				BIT(29)
#define   UTMIP_HIGHZ_D				BIT(30)

#define APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP	(0x26c)
#define   UTMIP_LINE_WAKEUP_EN(x)		UTMIP(x, BIT(x), BIT(4))
#define   UHSIC_LINE_WAKEUP_EN			BIT(3)

#define APBDEV_PMC_UTMIP_TERM_PAD_CFG		(0x1f8)
#define   PCTRL_VAL(x)				(((x) & 0x3f) << 1)
#define   TCTRL_VAL(x)				(((x) & 0x3f) << 7)

#define APBDEV_PMC_UTMIP_PAD_CFGX(x)		(0x4c0 + (4 * (x)))
#define   RPD_CTRL_PX(x)			(((x) & 0x1f) << 22)

#define APBDEV_PMC_UHSIC_SLEEP_CFG		\
		APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(0)
#define   UHSIC_MASTER_ENABLE			BIT(24)
#define   UHSIC_WAKE_VAL(_value)		(((_value) & 0xf) << 28)
#define   UHSIC_WAKE_VAL_SD10			UHSIC_WAKE_VAL(2)
#define   UHSIC_WAKE_VAL_NONE			UHSIC_WAKE_VAL(12)

#define APBDEV_PMC_UHSIC_FAKE			APBDEV_PMC_UTMIP_UHSIC_FAKE(0)
#define   UHSIC_FAKE_STROBE_VAL			BIT(12)
#define   UHSIC_FAKE_DATA_VAL			BIT(13)
#define   UHSIC_FAKE_STROBE_EN			BIT(14)
#define   UHSIC_FAKE_DATA_EN			BIT(15)

#define APBDEV_PMC_UHSIC_SAVED_STATE		\
		APBDEV_PMC_UTMIP_UHSIC_SAVED_STATE(0)
#define   UHSIC_MODE(_value)			(((_value) & 0x1) << 24)
#define   UHSIC_HS				UHSIC_MODE(0)
#define   UHSIC_RST				UHSIC_MODE(1)

#define APBDEV_PMC_UHSIC_SLEEPWALK_CFG		\
		APBDEV_PMC_UTMIP_UHSIC_SLEEPWALK_CFG(0)
#define   UHSIC_WAKE_WALK_EN			BIT(30)
#define   UHSIC_LINEVAL_WALK_EN			BIT(31)

#define APBDEV_PMC_UHSIC_SLEEPWALK_P0		(0x210)
#define   UHSIC_DATA0_RPD_A			BIT(1)
#define   UHSIC_DATA0_RPU_B			BIT(11)
#define   UHSIC_DATA0_RPU_C			BIT(19)
#define   UHSIC_DATA0_RPU_D			BIT(27)
#define   UHSIC_STROBE_RPU_A			BIT(2)
#define   UHSIC_STROBE_RPD_B			BIT(8)
#define   UHSIC_STROBE_RPD_C			BIT(16)
#define   UHSIC_STROBE_RPD_D			BIT(24)

/* io dpd off request code */
#define IO_DPD_CODE_OFF		1

/*** Tegra186 register definitions **/
#define PMC_IMPL_HALT_IN_FIQ_MASK		BIT(28)

#define PMC_RST_LEVEL_MASK			0x3
#define PMC_RST_LEVEL_SHIFT			0x0
#define PMC_RST_SOURCE_MASK			0x3C
#define PMC_RST_SOURCE_SHIFT			0x2

#define T186_PMC_IO_DPD_CSIA_MASK		BIT(0)
#define T186_PMC_IO_DPD_CSIB_MASK		BIT(1)
#define T186_PMC_IO_DPD_CODE_DPD_OFF		BIT(30)
#define T186_PMC_IO_DPD_CODE_DPD_ON		BIT(31)

#define T186_PMC_IO_DPD2_CSIC_MASK		BIT(11)
#define T186_PMC_IO_DPD2_CSID_MASK		BIT(12)
#define T186_PMC_IO_DPD2_CSIE_MASK		BIT(13)
#define T186_PMC_IO_DPD2_CSIF_MASK		BIT(14)

#define T186_PMC_IO_DPD_REQ			0x74
#define T186_PMC_IO_DPD_STATUS			0x78

struct io_dpd_reg_info {
	u32 req_reg_off;
	u8 dpd_code_lsb;
};

static struct io_dpd_reg_info t3_io_dpd_req_regs[] = {
	{0x1b8, 30},
	{0x1c0, 30},
};

enum pmc_regs {
	TEGRA_PMC_CNTRL,
	TEGRA_PMC_WAKE_MASK,
	TEGRA_PMC_WAKE_LEVEL,
	TEGRA_PMC_WAKE_STATUS,
	TEGRA_PMC_WAKE_DELAY,
	TEGRA_PMC_SW_WAKE_STATUS,
	TEGRA_PMC_WAKE2_MASK,
	TEGRA_PMC_WAKE2_LEVEL,
	TEGRA_PMC_WAKE2_STATUS,
	TEGRA_PMC_SW_WAKE2_STATUS,
	TEGRA_PMC_IO_DPD_SAMPLE,
	TEGRA_PMC_IO_DPD_ENABLE,
	TEGRA_PMC_IO_DPD_REQ,
	TEGRA_PMC_IO_DPD_STATUS,
	TEGRA_PMC_IO_DPD2_REQ,
	TEGRA_PMC_IO_DPD2_STATUS,
	TEGRA_PMC_SEL_DPD_TIM,
	TEGRA_PMC_PWR_NO_IOPOWER,
	TEGRA_PMC_PWR_DET_ENABLE,
	TEGRA_PMC_PWR_DET_VAL,
	TEGRA_PMC_REMOVE_CLAMPING,
	TEGRA_PMC_PWRGATE_TOGGLE,
	TEGRA_PMC_PWRGATE_STATUS,
	TEGRA_PMC_COREPWRGOOD_TIMER,
	TEGRA_PMC_CPUPWRGOOD_TIMER,
	TEGRA_PMC_CPUPWROFF_TIMER,
	TEGRA_PMC_COREPWROFF_TIMER,
	TEGRA_PMC_SENSOR_CTRL,
	TEGRA_PMC_GPU_RG_CNTRL,
	TEGRA_PMC_FUSE_CTRL,
	TEGRA_PMC_BR_COMMAND_BASE,
	TEGRA_PMC_SCRATCH0,
	TEGRA_PMC_SCRATCH1,
	TEGRA_PMC_SCRATCH41,
	TEGRA_PMC_SCRATCH54,
	TEGRA_PMC_SCRATCH55,
	TEGRA_PMC_RST_STATUS,
	TEGRA_PMC_IMPL_RAMDUMP_CTL_STATUS,
	TEGRA_PMC_SATA_PWRGT_0,
	TEGRA_PMC_UFSHC_PWR_CNTRL_0,
	TEGRA_PMC_E_33V_PWR,
	TEGRA_PMC_E_18V_PWR,

	/* Last entry */
	TEGRA_PMC_MAX_REG,
};

static struct tegra_prod *prod_list;

#ifdef CONFIG_TEGRA210_BOOTROM_PMC
extern int tegra210_boorom_pmc_init(struct device *dev);
#endif

#define PMC_PWR_NO_IOPOWER	0x44

static DEFINE_SPINLOCK(pwr_lock);

struct tegra_pmc_io_pad_soc {
	const char *name;
	unsigned int dpd;
	unsigned int voltage;
	unsigned int io_power;
	const unsigned int pins[1];
	unsigned int npins;
	int dpd_req_reg;
	int dpd_status_reg;
	int dpd_timer_reg;
	int dpd_sample_reg;
	bool bdsdmem_cfc;
	unsigned int io_pad_pwr_det_enable_reg;
	unsigned int io_pad_pwr_det_val_reg;
	int pad_uv_0;
	int pad_uv_1;
};

struct tegra_pmc_soc {
	unsigned int num_powergates;
	const char *const *powergates;
	unsigned int num_cpu_powergates;
	const u8 *cpu_powergates;
	const struct tegra_pmc_io_pad_soc *io_pads;
	unsigned int num_io_pads;
	const struct pinctrl_pin_desc *descs;
	unsigned int num_descs;
	const unsigned long *rmap;

	bool has_tsense_reset;
	bool has_gpu_clamps;
	bool has_ps18;
	bool has_pclk_clock;
	bool has_interrupt_polarity_support;
	bool has_reboot_base_address;
	bool show_reset_status;
	bool skip_lp0_vector_setup;
	bool skip_legacy_pmc_init;
	bool skip_power_gate_debug_fs_init;
	bool skip_restart_register;
	bool skip_arm_pm_restart;
};

struct tegra_io_pad_regulator {
	const struct tegra_pmc_io_pad_soc *pad;
	struct regulator *regulator;
	struct notifier_block nb;
};

/**
 * struct tegra_pmc - NVIDIA Tegra PMC
 * @dev: pointer to PMC device structure
 * @base: pointer to I/O remapped register region
 * @reboot_base: pointer to I/O remapped register region for reboot address
 * @clk: pointer to pclk clock
 * @soc: pointer to SoC data structure
 * @debugfs: pointer to debugfs entry
 * @rate: currently configured rate of pclk
 * @suspend_mode: lowest suspend mode available
 * @cpu_good_time: CPU power good time (in microseconds)
 * @cpu_off_time: CPU power off time (in microsecends)
 * @core_osc_time: core power good OSC time (in microseconds)
 * @core_pmu_time: core power good PMU time (in microseconds)
 * @core_off_time: core power off time (in microseconds)
 * @corereq_high: core power request is active-high
 * @sysclkreq_high: system clock request is active-high
 * @combined_req: combined power request for CPU & core
 * @cpu_pwr_good_en: CPU power good signal is enabled
 * @lp0_vec_phys: physical base address of the LP0 warm boot code
 * @lp0_vec_size: size of the LP0 warm boot code
 * @powergates_available: Bitmap of available power gates
 * @powergates_lock: mutex for power gate register access
 * @pctl: pinctrl handle which is returned after registering pinctrl
 * @pinctrl_desc: Pincontrol descriptor for IO pads
 */
struct tegra_pmc {
	struct device *dev;
	void __iomem *base;
	void __iomem *reboot_base;
	struct clk *clk;
	struct dentry *debugfs;

	const struct tegra_pmc_soc *soc;

	unsigned long rate;

	enum tegra_suspend_mode suspend_mode;
	u32 cpu_good_time;
	u32 cpu_off_time;
	u32 core_osc_time;
	u32 core_pmu_time;
	u32 core_off_time;
	bool corereq_high;
	bool sysclkreq_high;
	bool combined_req;
	bool cpu_pwr_good_en;
	u32 lp0_vec_phys;
	u32 lp0_vec_size;
	DECLARE_BITMAP(powergates_available, TEGRA_POWERGATE_MAX);

	struct mutex powergates_lock;
	struct pinctrl_dev *pctl;
	struct pinctrl_desc pinctrl_desc;
};

static struct tegra_pmc *pmc = &(struct tegra_pmc) {
	.base = NULL,
	.suspend_mode = TEGRA_SUSPEND_NONE,
};

static inline struct tegra_powergate *
to_powergate(struct generic_pm_domain *domain)
{
	return container_of(domain, struct tegra_powergate, genpd);
}

/* PMC register read/write/update with offset from the base */
static u32 _tegra_pmc_readl(unsigned long offset)
{
	return readl(pmc->base + offset);
}

static void _tegra_pmc_writel(u32 value, unsigned long offset)
{
	writel(value, pmc->base + offset);
}

#ifndef CONFIG_TEGRA186_PMC
static void _tegra_pmc_register_update(int offset,
		unsigned long mask, unsigned long val)
{
	u32 pmc_reg;

	pmc_reg = _tegra_pmc_readl(offset);
	pmc_reg = (pmc_reg & ~mask) | (val & mask);
	_tegra_pmc_writel(pmc_reg, offset);
}

static void _tegra_pmc_register_update_lock(int offset,
		unsigned long mask, unsigned long val)
{
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);
	_tegra_pmc_register_update(offset, mask, val);
	spin_unlock_irqrestore(&pwr_lock, flags);
}
#endif

/* PMC register read/write/update with pmc register enums */
static u32 tegra_pmc_readl(enum pmc_regs reg)
{
	return readl(pmc->base + pmc->soc->rmap[reg]);
}

static void tegra_pmc_writel(u32 value, enum pmc_regs reg)
{
	writel(value, pmc->base + pmc->soc->rmap[reg]);
}

static void tegra_pmc_register_update(enum pmc_regs reg,
                                      unsigned long mask,
                                      unsigned long val)
{
	u32 pmc_reg;

	pmc_reg = tegra_pmc_readl(reg);
	pmc_reg = (pmc_reg & ~mask) | (val & mask);
	tegra_pmc_writel(pmc_reg, reg);
}

#ifndef CONFIG_TEGRA_POWERGATE
static inline bool tegra_powergate_state(int id)
{
	if (id == TEGRA_POWERGATE_3D && pmc->soc->has_gpu_clamps)
		return (_tegra_pmc_readl(GPU_RG_CNTRL) & 0x1) == 0;
	else
		return (_tegra_pmc_readl(PWRGATE_STATUS) & BIT(id)) != 0;
}

static inline bool tegra_powergate_is_valid(int id)
{
	return (pmc->soc && pmc->soc->powergates[id]);
}

static inline bool tegra_powergate_is_available(int id)
{
	return test_bit(id, pmc->powergates_available);
}

static int tegra_powergate_lookup(struct tegra_pmc *pmc, const char *name)
{
	unsigned int i;

	if (!pmc || !pmc->soc || !name)
		return -EINVAL;

	for (i = 0; i < pmc->soc->num_powergates; i++) {
		if (!tegra_powergate_is_valid(i))
			continue;

		if (!strcmp(name, pmc->soc->powergates[i]))
			return i;
	}

	dev_err(pmc->dev, "powergate %s not found\n", name);

	return -ENODEV;
}

/**
 * tegra_powergate_set() - set the state of a partition
 * @id: partition ID
 * @new_state: new state of the partition
 */
static int tegra_powergate_set(unsigned int id, bool new_state)
{
	bool status;
	int err;

	if (id == TEGRA_POWERGATE_3D && pmc->soc->has_gpu_clamps)
		return -EINVAL;

	mutex_lock(&pmc->powergates_lock);

	if (tegra_powergate_state(id) == new_state) {
		mutex_unlock(&pmc->powergates_lock);
		return 0;
	}

	tegra_pmc_writel(PWRGATE_TOGGLE_START | id, TEGRA_PMC_PWRGATE_TOGGLE);

	err = readx_poll_timeout(tegra_powergate_state, id, status,
				 status == new_state, 10, 100000);

	mutex_unlock(&pmc->powergates_lock);

	return err;
}

static int __tegra_powergate_remove_clamping(unsigned int id)
{
	u32 mask;

	mutex_lock(&pmc->powergates_lock);

	/*
	 * On Tegra124 and later, the clamps for the GPU are controlled by a
	 * separate register (with different semantics).
	 */
	if (id == TEGRA_POWERGATE_3D) {
		if (pmc->soc->has_gpu_clamps) {
			_tegra_pmc_writel(0, GPU_RG_CNTRL);
			goto out;
		}
	}

	/*
	 * Tegra 2 has a bug where PCIE and VDE clamping masks are
	 * swapped relatively to the partition ids
	 */
	if (id == TEGRA_POWERGATE_VDEC)
		mask = (1 << TEGRA_POWERGATE_PCIE);
	else if (id == TEGRA_POWERGATE_PCIE)
		mask = (1 << TEGRA_POWERGATE_VDEC);
	else
		mask = (1 << id);

	_tegra_pmc_writel(mask, REMOVE_CLAMPING);

out:
	mutex_unlock(&pmc->powergates_lock);

	return 0;
}

static void tegra_powergate_disable_clocks(struct tegra_powergate *pg)
{
	unsigned int i;

	for (i = 0; i < pg->num_clks; i++)
		clk_disable_unprepare(pg->clks[i]);
}

static int tegra_powergate_enable_clocks(struct tegra_powergate *pg)
{
	unsigned int i;
	int err;

	for (i = 0; i < pg->num_clks; i++) {
		err = clk_prepare_enable(pg->clks[i]);
		if (err)
			goto out;
	}

	return 0;

out:
	while (i--)
		clk_disable_unprepare(pg->clks[i]);

	return err;
}

static int tegra_powergate_reset_assert(struct tegra_powergate *pg)
{
	unsigned int i;
	int err;

	for (i = 0; i < pg->num_resets; i++) {
		err = reset_control_assert(pg->resets[i]);
		if (err)
			return err;
	}

	return 0;
}

static int tegra_powergate_reset_deassert(struct tegra_powergate *pg)
{
	unsigned int i;
	int err;

	for (i = 0; i < pg->num_resets; i++) {
		err = reset_control_deassert(pg->resets[i]);
		if (err)
			return err;
	}

	return 0;
}

static int tegra_powergate_power_up(struct tegra_powergate *pg,
				    bool disable_clocks)
{
	int err;

	err = tegra_powergate_reset_assert(pg);
	if (err)
		return err;

	usleep_range(10, 20);

	err = tegra_powergate_set(pg->id, true);
	if (err < 0)
		return err;

	usleep_range(10, 20);

	err = tegra_powergate_enable_clocks(pg);
	if (err)
		goto disable_clks;

	usleep_range(10, 20);

	err = __tegra_powergate_remove_clamping(pg->id);
	if (err)
		goto disable_clks;

	usleep_range(10, 20);

	err = tegra_powergate_reset_deassert(pg);
	if (err)
		goto powergate_off;

	usleep_range(10, 20);

	if (disable_clocks)
		tegra_powergate_disable_clocks(pg);

	return 0;

disable_clks:
	tegra_powergate_disable_clocks(pg);
	usleep_range(10, 20);

powergate_off:
	tegra_powergate_set(pg->id, false);

	return err;
}

static int tegra_powergate_power_down(struct tegra_powergate *pg)
{
	int err;

	err = tegra_powergate_enable_clocks(pg);
	if (err)
		return err;

	usleep_range(10, 20);

	err = tegra_powergate_reset_assert(pg);
	if (err)
		goto disable_clks;

	usleep_range(10, 20);

	tegra_powergate_disable_clocks(pg);

	usleep_range(10, 20);

	err = tegra_powergate_set(pg->id, false);
	if (err)
		goto assert_resets;

	return 0;

assert_resets:
	tegra_powergate_enable_clocks(pg);
	usleep_range(10, 20);
	tegra_powergate_reset_deassert(pg);
	usleep_range(10, 20);

disable_clks:
	tegra_powergate_disable_clocks(pg);

	return err;
}

static int tegra_genpd_power_on(struct generic_pm_domain *domain)
{
	struct tegra_powergate *pg = to_powergate(domain);
	struct tegra_pmc *pmc = pg->pmc;
	int err;

	err = tegra_powergate_power_up(pg, true);
	if (err)
		dev_err(pmc->dev, "failed to turn on PM domain %s: %d\n",
			pg->genpd.name, err);

	return err;
}

static int tegra_genpd_power_off(struct generic_pm_domain *domain)
{
	struct tegra_powergate *pg = to_powergate(domain);
	struct tegra_pmc *pmc = pg->pmc;
	int err;

	err = tegra_powergate_power_down(pg);
	if (err)
		dev_err(pmc->dev, "failed to turn off PM domain %s: %d\n",
			pg->genpd.name, err);

	return err;
}

/**
 * tegra_powergate_power_on() - power on partition
 * @id: partition ID
 */
int tegra_powergate_power_on(unsigned int id)
{
	if (!tegra_powergate_is_available(id))
		return -EINVAL;

	return tegra_powergate_set(id, true);
}

/**
 * tegra_powergate_power_off() - power off partition
 * @id: partition ID
 */
int tegra_powergate_power_off(unsigned int id)
{
	if (!tegra_powergate_is_available(id))
		return -EINVAL;

	return tegra_powergate_set(id, false);
}
EXPORT_SYMBOL(tegra_powergate_power_off);

/**
 * tegra_powergate_is_powered() - check if partition is powered
 * @id: partition ID
 */
int tegra_powergate_is_powered(unsigned int id)
{
	int status;

	if (!tegra_powergate_is_valid(id))
		return -EINVAL;

	mutex_lock(&pmc->powergates_lock);
	status = tegra_powergate_state(id);
	mutex_unlock(&pmc->powergates_lock);

	return status;
}

/**
 * tegra_powergate_remove_clamping() - remove power clamps for partition
 * @id: partition ID
 */
int tegra_powergate_remove_clamping(unsigned int id)
{
	if (!tegra_powergate_is_available(id))
		return -EINVAL;

	return __tegra_powergate_remove_clamping(id);
}
EXPORT_SYMBOL(tegra_powergate_remove_clamping);

/**
 * tegra_powergate_sequence_power_up() - power up partition
 * @id: partition ID
 * @clk: clock for partition
 * @rst: reset for partition
 *
 * Must be called with clk disabled, and returns with clk enabled.
 */
int tegra_powergate_sequence_power_up(unsigned int id, struct clk *clk,
				      struct reset_control *rst)
{
	struct tegra_powergate pg;
	int err;

	if (!tegra_powergate_is_available(id))
		return -EINVAL;

	pg.id = id;
	pg.clks = &clk;
	pg.num_clks = 1;
	pg.resets = &rst;
	pg.num_resets = 1;

	err = tegra_powergate_power_up(&pg, false);
	if (err)
		pr_err("failed to turn on partition %d: %d\n", id, err);

	return err;
}
EXPORT_SYMBOL(tegra_powergate_sequence_power_up);

#ifdef CONFIG_SMP
/**
 * tegra_get_cpu_powergate_id() - convert from CPU ID to partition ID
 * @cpuid: CPU partition ID
 *
 * Returns the partition ID corresponding to the CPU partition ID or a
 * negative error code on failure.
 */
static int tegra_get_cpu_powergate_id(unsigned int cpuid)
{
	if (pmc->soc && cpuid < pmc->soc->num_cpu_powergates)
		return pmc->soc->cpu_powergates[cpuid];

	return -EINVAL;
}

/**
 * tegra_pmc_cpu_is_powered() - check if CPU partition is powered
 * @cpuid: CPU partition ID
 */
bool tegra_pmc_cpu_is_powered(unsigned int cpuid)
{
	int id;

	id = tegra_get_cpu_powergate_id(cpuid);
	if (id < 0)
		return false;

	return tegra_powergate_is_powered(id);
}

/**
 * tegra_pmc_cpu_power_on() - power on CPU partition
 * @cpuid: CPU partition ID
 */
int tegra_pmc_cpu_power_on(unsigned int cpuid)
{
	int id;

	id = tegra_get_cpu_powergate_id(cpuid);
	if (id < 0)
		return id;

	return tegra_powergate_set(id, true);
}

/**
 * tegra_pmc_cpu_remove_clamping() - remove power clamps for CPU partition
 * @cpuid: CPU partition ID
 */
int tegra_pmc_cpu_remove_clamping(unsigned int cpuid)
{
	int id;

	id = tegra_get_cpu_powergate_id(cpuid);
	if (id < 0)
		return id;

	return tegra_powergate_remove_clamping(id);
}
#endif /* CONFIG_SMP */
#endif /* CONFIG_TEGRA_POWERGATE */

static void tegra_pmc_program_reboot_reason(const char *cmd)
{
	u32 value;

	value = tegra_pmc_readl(TEGRA_PMC_SCRATCH0);
	value &= ~PMC_SCRATCH0_MODE_MASK;

	if (cmd) {
		if (strcmp(cmd, "recovery") == 0)
			value |= PMC_SCRATCH0_MODE_RECOVERY;

		if (strcmp(cmd, "bootloader") == 0)
			value |= PMC_SCRATCH0_MODE_BOOTLOADER;

		if (strcmp(cmd, "forced-recovery") == 0)
			value |= PMC_SCRATCH0_MODE_RCM;
	}

	tegra_pmc_writel(value, TEGRA_PMC_SCRATCH0);
}

static int tegra_pmc_restart_notify(struct notifier_block *this,
				    unsigned long action, void *data)
{
	const char *cmd = data;
	u32 value;

	tegra_pmc_program_reboot_reason(cmd);

	/* reset everything but PMC_SCRATCH0 and PMC_RST_STATUS */
	value = tegra_pmc_readl(TEGRA_PMC_CNTRL);
	value |= PMC_CNTRL_MAIN_RST;
	tegra_pmc_writel(value, TEGRA_PMC_CNTRL);

	return NOTIFY_DONE;
}

static struct notifier_block tegra_pmc_restart_handler = {
	.notifier_call = tegra_pmc_restart_notify,
	.priority = 128,
};

#ifndef CONFIG_TEGRA_POWERGATE
static int powergate_show(struct seq_file *s, void *data)
{
	unsigned int i;
	int status;

	seq_printf(s, " powergate powered\n");
	seq_printf(s, "------------------\n");

	for (i = 0; i < pmc->soc->num_powergates; i++) {
		status = tegra_powergate_is_powered(i);
		if (status < 0)
			continue;

		seq_printf(s, " %9s %7s\n", pmc->soc->powergates[i],
			   status ? "yes" : "no");
	}

	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open = powergate_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int tegra_powergate_debugfs_init(void)
{
	if (pmc->soc->skip_power_gate_debug_fs_init)
		return 0;

	pmc->debugfs = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
					   &powergate_fops);
	if (!pmc->debugfs)
		return -ENOMEM;

	return 0;
}

static int tegra_powergate_of_get_clks(struct tegra_powergate *pg,
				       struct device_node *np)
{
	struct clk *clk;
	unsigned int i, count;
	int err;

	count = of_count_phandle_with_args(np, "clocks", "#clock-cells");
	if (count == 0)
		return -ENODEV;

	pg->clks = kcalloc(count, sizeof(clk), GFP_KERNEL);
	if (!pg->clks)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		pg->clks[i] = of_clk_get(np, i);
		if (IS_ERR(pg->clks[i])) {
			err = PTR_ERR(pg->clks[i]);
			goto err;
		}
	}

	pg->num_clks = count;

	return 0;

err:
	while (i--)
		clk_put(pg->clks[i]);

	kfree(pg->clks);

	return err;
}

static int tegra_powergate_of_get_resets(struct tegra_powergate *pg,
					 struct device_node *np, bool off)
{
	struct reset_control *rst;
	unsigned int i, count;
	int err;

	count = of_count_phandle_with_args(np, "resets", "#reset-cells");
	if (count == 0)
		return -ENODEV;

	pg->resets = kcalloc(count, sizeof(rst), GFP_KERNEL);
	if (!pg->resets)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		pg->resets[i] = of_reset_control_get_by_index(np, i);
		if (IS_ERR(pg->resets[i])) {
			err = PTR_ERR(pg->resets[i]);
			goto error;
		}

		if (off)
			err = reset_control_assert(pg->resets[i]);
		else
			err = reset_control_deassert(pg->resets[i]);

		if (err) {
			reset_control_put(pg->resets[i]);
			goto error;
		}
	}

	pg->num_resets = count;

	return 0;

error:
	while (i--)
		reset_control_put(pg->resets[i]);

	kfree(pg->resets);

	return err;
}

static void tegra_powergate_add(struct tegra_pmc *pmc, struct device_node *np)
{
	struct tegra_powergate *pg;
	int id, err;
	bool off;

	pg = kzalloc(sizeof(*pg), GFP_KERNEL);
	if (!pg)
		return;

	id = tegra_powergate_lookup(pmc, np->name);
	if (id < 0) {
		dev_err(pmc->dev, "powergate lookup failed for %s: %d\n",
			np->name, id);
		goto free_mem;
	}

	/*
	 * Clear the bit for this powergate so it cannot be managed
	 * directly via the legacy APIs for controlling powergates.
	 */
	clear_bit(id, pmc->powergates_available);

	pg->id = id;
	pg->genpd.name = np->name;
	pg->genpd.power_off = tegra_genpd_power_off;
	pg->genpd.power_on = tegra_genpd_power_on;
	pg->pmc = pmc;

	off = !tegra_powergate_is_powered(pg->id);

	err = tegra_powergate_of_get_clks(pg, np);
	if (err < 0) {
		dev_err(pmc->dev, "failed to get clocks for %s: %d\n",
			np->name, err);
		goto set_available;
	}

	err = tegra_powergate_of_get_resets(pg, np, off);
	if (err < 0) {
		dev_err(pmc->dev, "failed to get resets for %s: %d\n",
			np->name, err);
		goto remove_clks;
	}

	if (!IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS))
		goto power_on_cleanup;

	/*
	 * FIXME: If XHCI is enabled for Tegra, then power-up the XUSB
	 * host and super-speed partitions. Once the XHCI driver
	 * manages the partitions itself this code can be removed. Note
	 * that we don't register these partitions with the genpd core
	 * to avoid it from powering down the partitions as they appear
	 * to be unused.
	 */
	if (IS_ENABLED(CONFIG_USB_XHCI_TEGRA) &&
	    (id == TEGRA_POWERGATE_XUSBA || id == TEGRA_POWERGATE_XUSBC))
		goto power_on_cleanup;

	pm_genpd_init(&pg->genpd, NULL, off);

	err = of_genpd_add_provider_simple(np, &pg->genpd);
	if (err < 0) {
		dev_err(pmc->dev, "failed to add genpd provider for %s: %d\n",
			np->name, err);
		goto remove_resets;
	}

	dev_dbg(pmc->dev, "added power domain %s\n", pg->genpd.name);

	return;

power_on_cleanup:
	if (off)
		WARN_ON(tegra_powergate_power_up(pg, true));

remove_resets:
	while (pg->num_resets--)
		reset_control_put(pg->resets[pg->num_resets]);

	kfree(pg->resets);

remove_clks:
	while (pg->num_clks--)
		clk_put(pg->clks[pg->num_clks]);

	kfree(pg->clks);

set_available:
	set_bit(id, pmc->powergates_available);

free_mem:
	kfree(pg);
}
#else
static int tegra_powergate_debugfs_init(void)
{
	return 0;
}
#endif

#ifndef CONFIG_TEGRA_POWERGATE
static void tegra_powergate_init(struct tegra_pmc *pmc,
				 struct device_node *parent)
{
	struct device_node *np, *child;
	unsigned int i;

	/* Create a bitmap of the available and valid partitions */
	for (i = 0; i < pmc->soc->num_powergates; i++)
		if (pmc->soc->powergates[i])
			set_bit(i, pmc->powergates_available);

	np = of_get_child_by_name(parent, "powergates");
	if (!np)
		return;

	for_each_child_of_node(np, child) {
		tegra_powergate_add(pmc, child);
		of_node_put(child);
	}

	of_node_put(np);
}

/* Legacy APIs for IO pad enable/disable */
int tegra_io_rail_power_on(unsigned int id)
{
	switch (id) {
	case TEGRA_IO_RAIL_LVDS:
		return tegra_pmc_io_pad_low_power_disable("lvds");
	case TEGRA_IO_RAIL_HDMI:
		return tegra_pmc_io_pad_low_power_disable("hdmi");
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(tegra_io_rail_power_on);

int tegra_io_rail_power_off(unsigned int id)
{
	switch (id) {
	case TEGRA_IO_RAIL_LVDS:
		return tegra_pmc_io_pad_low_power_enable("lvds");
	case TEGRA_IO_RAIL_HDMI:
		return tegra_pmc_io_pad_low_power_enable("hdmi");
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(tegra_io_rail_power_off);
#endif /* CONFIG_TEGRA_POWERGATE */

void tegra_pmc_write_bootrom_command(u32 command_offset, unsigned long val)
{
	_tegra_pmc_writel(val, command_offset + PMC_BR_COMMAND_BASE);
}
EXPORT_SYMBOL(tegra_pmc_write_bootrom_command);

void tegra_pmc_reset_system(void)
{
	u32 val;

	val = tegra_pmc_readl(TEGRA_PMC_CNTRL);
	val |= 0x10;
	tegra_pmc_writel(val, TEGRA_PMC_CNTRL);
}
EXPORT_SYMBOL(tegra_pmc_reset_system);

#ifndef CONFIG_TEGRA186_PMC
int tegra_pmc_clear_reboot_reason(u32 reason)
{
	u32 val;

	val = readl_relaxed(pmc->reboot_base +
			    pmc->soc->rmap[TEGRA_PMC_SCRATCH0]);
	val &= ~reason;
	writel_relaxed(val, pmc->reboot_base +
		       pmc->soc->rmap[TEGRA_PMC_SCRATCH0]);
	return 0;
}
EXPORT_SYMBOL(tegra_pmc_clear_reboot_reason);

int tegra_pmc_set_reboot_reason(u32 reason)
{
	u32 val;

	val = readl_relaxed(pmc->reboot_base +
			    pmc->soc->rmap[TEGRA_PMC_SCRATCH0]);
	val |= reason;
	writel_relaxed(val, pmc->reboot_base +
		       pmc->soc->rmap[TEGRA_PMC_SCRATCH0]);
	return 0;
}
EXPORT_SYMBOL(tegra_pmc_set_reboot_reason);

/* UFS power gate control */
void tegra_pmc_ufs_pwrcntrl_update(unsigned long mask, unsigned long val)
{
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);
	tegra_pmc_register_update(TEGRA_PMC_UFSHC_PWR_CNTRL_0, mask, val);
	spin_unlock_irqrestore(&pwr_lock, flags);
}
EXPORT_SYMBOL(tegra_pmc_ufs_pwrcntrl_update);

unsigned long tegra_pmc_ufs_pwrcntrl_get(void)
{
	return tegra_pmc_readl(TEGRA_PMC_UFSHC_PWR_CNTRL_0);
}
EXPORT_SYMBOL(tegra_pmc_ufs_pwrcntrl_get);

/* SATA power gate control */
void tegra_pmc_sata_pwrgt_update(unsigned long mask, unsigned long val)
{
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);
	tegra_pmc_register_update(TEGRA_PMC_SATA_PWRGT_0, mask, val);
	spin_unlock_irqrestore(&pwr_lock, flags);
}
EXPORT_SYMBOL(tegra_pmc_sata_pwrgt_update);

unsigned long tegra_pmc_sata_pwrgt_get(void)
{
	return tegra_pmc_readl(TEGRA_PMC_SATA_PWRGT_0);
}
EXPORT_SYMBOL(tegra_pmc_sata_pwrgt_get);

void tegra_pmc_iopower_enable(int reg, u32 bit_mask)
{
	_tegra_pmc_register_update_lock(reg, bit_mask, 0);
}
EXPORT_SYMBOL(tegra_pmc_iopower_enable);

void  tegra_pmc_iopower_disable(int reg, u32 bit_mask)
{
	_tegra_pmc_register_update_lock(reg, bit_mask, bit_mask);
}
EXPORT_SYMBOL(tegra_pmc_iopower_disable);

int tegra_pmc_iopower_get_status(int reg, u32 bit_mask)
{
	unsigned int no_iopower;

	no_iopower = _tegra_pmc_readl(reg);
	if (no_iopower & bit_mask)
		return 0;
	else
		return 1;
}
EXPORT_SYMBOL(tegra_pmc_iopower_get_status);
#endif

/* cleans io dpd settings from bootloader during kernel init */
static void _tegra_bl_io_dpd_cleanup(void)
{
	int i;
	unsigned int dpd_mask;
	unsigned int dpd_status;

	pr_info("Clear bootloader IO dpd settings\n");
	/* clear all dpd requests from bootloader */
	for (i = 0; i < ARRAY_SIZE(t3_io_dpd_req_regs); i++) {
		dpd_mask = ((1 << t3_io_dpd_req_regs[i].dpd_code_lsb) - 1);
		dpd_mask |= (IO_DPD_CODE_OFF <<
			t3_io_dpd_req_regs[i].dpd_code_lsb);
		_tegra_pmc_writel(dpd_mask, t3_io_dpd_req_regs[i].req_reg_off);
		/* dpd status register is next to req reg in tegra3 */
		dpd_status =
			_tegra_pmc_readl(t3_io_dpd_req_regs[i].req_reg_off + 4);
	}
	return;
}

void tegra_pmc_io_dpd_clear(void)
{
	_tegra_bl_io_dpd_cleanup();
}
EXPORT_SYMBOL(tegra_pmc_io_dpd_clear);

/* T210 USB2 SLEEPWALK APIs */
int tegra_pmc_utmi_phy_enable_sleepwalk(int port, enum usb_device_speed speed,
					struct tegra_utmi_pad_config *config)
{
	u32 reg;

	pr_info("PMC %s : port %d, speed %d\n", __func__, port, speed);

	/* ensure sleepwalk logic is disabled */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	reg &= ~UTMIP_MASTER_ENABLE(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	/* ensure sleepwalk logics are in low power mode */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_MASTER_CONFIG);
	reg |= UTMIP_PWR(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_MASTER_CONFIG);

	/* set debounce time */
	reg = _tegra_pmc_readl(APBDEV_PMC_USB_DEBOUNCE_DEL);
	reg &= ~UTMIP_LINE_DEB_CNT(~0);
	reg |= UTMIP_LINE_DEB_CNT(0x1);
	_tegra_pmc_writel(reg, APBDEV_PMC_USB_DEBOUNCE_DEL);

	/* ensure fake events of sleepwalk logic are desiabled */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_FAKE(port));
	reg &= ~(UTMIP_FAKE_USBOP_VAL(port) | UTMIP_FAKE_USBON_VAL(port) |
			UTMIP_FAKE_USBOP_EN(port) | UTMIP_FAKE_USBON_EN(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_FAKE(port));

	/* ensure wake events of sleepwalk logic are not latched */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);
	reg &= ~UTMIP_LINE_WAKEUP_EN(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);

	/* disable wake event triggers of sleepwalk logic */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	reg &= ~UTMIP_WAKE_VAL(port, ~0);
	reg |= UTMIP_WAKE_VAL_NONE(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	/* power down the line state detectors of the pad */
	reg = _tegra_pmc_readl(APBDEV_PMC_USB_AO);
	reg |= (USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_USB_AO);

	/* save state per speed */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SAVED_STATE(port));
	reg &= ~SPEED(port, ~0);
	if (speed == USB_SPEED_HIGH)
		reg |= UTMI_HS(port);
	else if (speed == USB_SPEED_FULL)
		reg |= UTMI_FS(port);
	else if (speed == USB_SPEED_LOW)
		reg |= UTMI_LS(port);
	else
		reg |= UTMI_RST(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SAVED_STATE(port));

	/* enable the trigger of the sleepwalk logic */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));
	reg |= (UTMIP_WAKE_WALK_EN(port) | UTMIP_LINEVAL_WALK_EN(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));

	/* reset the walk pointer and clear the alarm of the sleepwalk logic,
	 * as well as capture the configuration of the USB2.0 pad
	 */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_TRIGGERS);
	reg |= (UTMIP_CLR_WALK_PTR(port) | UTMIP_CLR_WAKE_ALARM(port) |
		UTMIP_CAP_CFG(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_TRIGGERS);

	/* program electrical parameters read from XUSB PADCTL */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_TERM_PAD_CFG);
	reg &= ~(TCTRL_VAL(~0) | PCTRL_VAL(~0));
	reg |= (TCTRL_VAL(config->tctrl) | PCTRL_VAL(config->pctrl));
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_TERM_PAD_CFG);

	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_PAD_CFGX(port));
	reg &= ~RPD_CTRL_PX(~0);
	reg |= RPD_CTRL_PX(config->rpd_ctrl);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_PAD_CFGX(port));

	/* setup the pull-ups and pull-downs of the signals during the four
	 * stages of sleepwalk.
	 * if device is connected, program sleepwalk logic to maintain a J and
	 * keep driving K upon seeing remote wake.
	 */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_SLEEPWALK_PX(port));
	reg = (UTMIP_USBOP_RPD_A | UTMIP_USBOP_RPD_B | UTMIP_USBOP_RPD_C |
		UTMIP_USBOP_RPD_D);
	reg |= (UTMIP_USBON_RPD_A | UTMIP_USBON_RPD_B | UTMIP_USBON_RPD_C |
		UTMIP_USBON_RPD_D);
	if (speed == USB_SPEED_UNKNOWN) {
		reg |= (UTMIP_HIGHZ_A | UTMIP_HIGHZ_B | UTMIP_HIGHZ_C |
			UTMIP_HIGHZ_D);
	} else if ((speed == USB_SPEED_HIGH) || (speed == USB_SPEED_FULL)) {
		/* J state: D+/D- = high/low, K state: D+/D- = low/high */
		reg |= UTMIP_HIGHZ_A;
		reg |= UTMIP_AP_A;
		reg |= (UTMIP_AN_B | UTMIP_AN_C | UTMIP_AN_D);
	} else if (speed == USB_SPEED_LOW) {
		/* J state: D+/D- = low/high, K state: D+/D- = high/low */
		reg |= UTMIP_HIGHZ_A;
		reg |= UTMIP_AN_A;
		reg |= (UTMIP_AP_B | UTMIP_AP_C | UTMIP_AP_D);
	}
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_SLEEPWALK_PX(port));

	/* power up the line state detectors of the pad */
	reg = _tegra_pmc_readl(APBDEV_PMC_USB_AO);
	reg &= ~(USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_USB_AO);

	usleep_range(50, 100);

	/* switch the electric control of the USB2.0 pad to PMC */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	reg |= (UTMIP_FSLS_USE_PMC(port) | UTMIP_PCTRL_USE_PMC(port) |
			UTMIP_TCTRL_USE_PMC(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG1);
	reg |= (UTMIP_RPD_CTRL_USE_PMC_PX(port) |
			UTMIP_RPU_SWITC_LOW_USE_PMC_PX(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG1);

	/* set the wake signaling trigger events */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	reg &= ~UTMIP_WAKE_VAL(port, ~0);
	reg |= UTMIP_WAKE_VAL_ANY(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	/* enable the wake detection */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	reg |= UTMIP_MASTER_ENABLE(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);
	reg |= UTMIP_LINE_WAKEUP_EN(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);

	return 0;
}
EXPORT_SYMBOL(tegra_pmc_utmi_phy_enable_sleepwalk);

int tegra_pmc_utmi_phy_disable_sleepwalk(int port)
{
	u32 reg;

	pr_info("PMC %s : port %dn", __func__, port);

	/* disable the wake detection */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	reg &= ~UTMIP_MASTER_ENABLE(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);
	reg &= ~UTMIP_LINE_WAKEUP_EN(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);

	/* switch the electric control of the USB2.0 pad to XUSB or USB2 */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	reg &= ~(UTMIP_FSLS_USE_PMC(port) | UTMIP_PCTRL_USE_PMC(port) |
			UTMIP_TCTRL_USE_PMC(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG1);
	reg &= ~(UTMIP_RPD_CTRL_USE_PMC_PX(port) |
			UTMIP_RPU_SWITC_LOW_USE_PMC_PX(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG1);

	/* disable wake event triggers of sleepwalk logic */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	reg &= ~UTMIP_WAKE_VAL(port, ~0);
	reg |= UTMIP_WAKE_VAL_NONE(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	/* power down the line state detectors of the port */
	reg = _tegra_pmc_readl(APBDEV_PMC_USB_AO);
	reg |= (USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	_tegra_pmc_writel(reg, APBDEV_PMC_USB_AO);

	/* clear alarm of the sleepwalk logic */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_TRIGGERS);
	reg |= UTMIP_CLR_WAKE_ALARM(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_TRIGGERS);

	return 0;
}
EXPORT_SYMBOL(tegra_pmc_utmi_phy_disable_sleepwalk);

int tegra_pmc_hsic_phy_enable_sleepwalk(int port)
{
	u32 reg;

	pr_info("PMC %s : port %dn", __func__, port);

	/* ensure sleepwalk logic is disabled */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_SLEEP_CFG);
	reg &= ~UHSIC_MASTER_ENABLE;
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_SLEEP_CFG);

	/* ensure sleepwalk logics are in low power mode */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_MASTER_CONFIG);
	reg |= UHSIC_PWR(port);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_MASTER_CONFIG);

	/* set debounce time */
	reg = _tegra_pmc_readl(APBDEV_PMC_USB_DEBOUNCE_DEL);
	reg &= ~UHSIC_LINE_DEB_CNT(~0);
	reg |= UHSIC_LINE_DEB_CNT(0x1);
	_tegra_pmc_writel(reg, APBDEV_PMC_USB_DEBOUNCE_DEL);

	/* ensure fake events of sleepwalk logic are desiabled */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_FAKE);
	reg &= ~(UHSIC_FAKE_STROBE_VAL | UHSIC_FAKE_DATA_VAL |
			UHSIC_FAKE_STROBE_EN | UHSIC_FAKE_DATA_EN);
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_FAKE);

	/* ensure wake events of sleepwalk logic are not latched */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);
	reg &= ~UHSIC_LINE_WAKEUP_EN;
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);

	/* disable wake event triggers of sleepwalk logic */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_SLEEP_CFG);
	reg &= ~UHSIC_WAKE_VAL(~0);
	reg |= UHSIC_WAKE_VAL_NONE;
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_SLEEP_CFG);

	/* power down the line state detectors of the port */
	reg = _tegra_pmc_readl(APBDEV_PMC_USB_AO);
	reg |= (STROBE_VAL_PD(port) | DATA0_VAL_PD(port) | DATA1_VAL_PD);
	_tegra_pmc_writel(reg, APBDEV_PMC_USB_AO);

	/* save state, HSIC always comes up as HS */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_SAVED_STATE);
	reg &= ~UHSIC_MODE(~0);
	reg |= UHSIC_HS;
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_SAVED_STATE);

	/* enable the trigger of the sleepwalk logic */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_SLEEPWALK_CFG);
	reg |= (UHSIC_WAKE_WALK_EN | UHSIC_LINEVAL_WALK_EN);
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_SLEEPWALK_CFG);

	/* reset the walk pointer and clear the alarm of the sleepwalk logic,
	 * as well as capture the configuration of the USB2.0 port
	 */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_TRIGGERS);
	reg |= (UHSIC_CLR_WALK_PTR | UHSIC_CLR_WAKE_ALARM);
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_TRIGGERS);

	/* setup the pull-ups and pull-downs of the signals during the four
	 * stages of sleepwalk.
	 * maintain a HSIC IDLE and keep driving HSIC RESUME upon remote wake
	 */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_SLEEPWALK_P0);
	reg = (UHSIC_DATA0_RPD_A | UHSIC_DATA0_RPU_B | UHSIC_DATA0_RPU_C |
		UHSIC_DATA0_RPU_D);
	reg |= (UHSIC_STROBE_RPU_A | UHSIC_STROBE_RPD_B | UHSIC_STROBE_RPD_C |
		UHSIC_STROBE_RPD_D);
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_SLEEPWALK_P0);

	/* power up the line state detectors of the port */
	reg = _tegra_pmc_readl(APBDEV_PMC_USB_AO);
	reg &= ~(STROBE_VAL_PD(port) | DATA0_VAL_PD(port) | DATA1_VAL_PD);
	_tegra_pmc_writel(reg, APBDEV_PMC_USB_AO);

	usleep_range(50, 100);

	/* set the wake signaling trigger events */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_SLEEP_CFG);
	reg &= ~UHSIC_WAKE_VAL(~0);
	reg |= UHSIC_WAKE_VAL_SD10;
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_SLEEP_CFG);

	/* enable the wake detection */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_SLEEP_CFG);
	reg |= UHSIC_MASTER_ENABLE;
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_SLEEP_CFG);

	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);
	reg |= UHSIC_LINE_WAKEUP_EN;
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);

	return 0;
}
EXPORT_SYMBOL(tegra_pmc_hsic_phy_enable_sleepwalk);

int tegra_pmc_hsic_phy_disable_sleepwalk(int port)
{
	u32 reg;

	pr_info("PMC %s : port %dn", __func__, port);

	/* disable the wake detection */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_SLEEP_CFG);
	reg &= ~UHSIC_MASTER_ENABLE;
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_SLEEP_CFG);

	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);
	reg &= ~UHSIC_LINE_WAKEUP_EN;
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP);

	/* disable wake event triggers of sleepwalk logic */
	reg = _tegra_pmc_readl(APBDEV_PMC_UHSIC_SLEEP_CFG);
	reg &= ~UHSIC_WAKE_VAL(~0);
	reg |= UHSIC_WAKE_VAL_NONE;
	_tegra_pmc_writel(reg, APBDEV_PMC_UHSIC_SLEEP_CFG);

	/* power down the line state detectors of the port */
	reg = _tegra_pmc_readl(APBDEV_PMC_USB_AO);
	reg |= (STROBE_VAL_PD(port) | DATA0_VAL_PD(port) | DATA1_VAL_PD);
	_tegra_pmc_writel(reg, APBDEV_PMC_USB_AO);


	/* clear alarm of the sleepwalk logic */
	reg = _tegra_pmc_readl(APBDEV_PMC_UTMIP_UHSIC_TRIGGERS);
	reg |= UHSIC_CLR_WAKE_ALARM;
	_tegra_pmc_writel(reg, APBDEV_PMC_UTMIP_UHSIC_TRIGGERS);

	return 0;
}
EXPORT_SYMBOL(tegra_pmc_hsic_phy_disable_sleepwalk);

#ifndef CONFIG_TEGRA186_PMC
void tegra_pmc_fuse_disable_mirroring(void)
{
	u32 val;

	val = tegra_pmc_readl(TEGRA_PMC_FUSE_CTRL);
	if (val & PMC_FUSE_CTRL_ENABLE_REDIRECTION) {
		val &= ~PMC_FUSE_CTRL_ENABLE_REDIRECTION;
		tegra_pmc_writel(val, TEGRA_PMC_FUSE_CTRL);
	}
}
EXPORT_SYMBOL(tegra_pmc_fuse_disable_mirroring);

void tegra_pmc_fuse_enable_mirroring(void)
{
	u32 val;

	val = tegra_pmc_readl(TEGRA_PMC_FUSE_CTRL);
	if (!(val & PMC_FUSE_CTRL_ENABLE_REDIRECTION)) {
		val |= PMC_FUSE_CTRL_ENABLE_REDIRECTION;
		tegra_pmc_writel(val, TEGRA_PMC_FUSE_CTRL);
	}
}
EXPORT_SYMBOL(tegra_pmc_fuse_enable_mirroring);

void tegra_pmc_fuse_control_ps18_latch_set(void)
{
	u32 val;

	if (!pmc->soc->has_ps18)
		return;

	val = tegra_pmc_readl(TEGRA_PMC_FUSE_CTRL);
	val &= ~(PMC_FUSE_CTRL_PS18_LATCH_CLEAR);
	tegra_pmc_writel(val, TEGRA_PMC_FUSE_CTRL);
	mdelay(1);
	val |= PMC_FUSE_CTRL_PS18_LATCH_SET;
	tegra_pmc_writel(val, TEGRA_PMC_FUSE_CTRL);
	mdelay(1);
}
EXPORT_SYMBOL(tegra_pmc_fuse_control_ps18_latch_set);

void tegra_pmc_fuse_control_ps18_latch_clear(void)
{
	u32 val;

	if (!pmc->soc->has_ps18)
		return;

	val = tegra_pmc_readl(TEGRA_PMC_FUSE_CTRL);
	val &= ~(PMC_FUSE_CTRL_PS18_LATCH_SET);
	tegra_pmc_writel(val, TEGRA_PMC_FUSE_CTRL);
	mdelay(1);
	val |= PMC_FUSE_CTRL_PS18_LATCH_CLEAR;
	tegra_pmc_writel(val, TEGRA_PMC_FUSE_CTRL);
	mdelay(1);
}
EXPORT_SYMBOL(tegra_pmc_fuse_control_ps18_latch_clear);
#endif

#ifdef CONFIG_PM_SLEEP
enum tegra_suspend_mode tegra_pmc_get_suspend_mode(void)
{
	return pmc->suspend_mode;
}

void tegra_pmc_set_suspend_mode(enum tegra_suspend_mode mode)
{
	if (mode < TEGRA_SUSPEND_NONE || mode >= TEGRA_MAX_SUSPEND_MODE)
		return;

	pmc->suspend_mode = mode;
}

void tegra_pmc_enter_suspend_mode(enum tegra_suspend_mode mode)
{
	unsigned long long rate = 0;
	u32 value;

	switch (mode) {
	case TEGRA_SUSPEND_LP1:
		rate = 32768;
		break;

	case TEGRA_SUSPEND_LP2:
		rate = clk_get_rate(pmc->clk);
		break;

	default:
		break;
	}

	if (WARN_ON_ONCE(rate == 0))
		rate = 100000000;

	if (rate != pmc->rate) {
		u64 ticks;

		ticks = pmc->cpu_good_time * rate + USEC_PER_SEC - 1;
		do_div(ticks, USEC_PER_SEC);
		tegra_pmc_writel(ticks, TEGRA_PMC_CPUPWRGOOD_TIMER);

		ticks = pmc->cpu_off_time * rate + USEC_PER_SEC - 1;
		do_div(ticks, USEC_PER_SEC);
		tegra_pmc_writel(ticks, TEGRA_PMC_CPUPWROFF_TIMER);

		wmb();

		pmc->rate = rate;
	}

	value = tegra_pmc_readl(TEGRA_PMC_CNTRL);
	value &= ~PMC_CNTRL_SIDE_EFFECT_LP0;
	value |= PMC_CNTRL_CPU_PWRREQ_OE;
	tegra_pmc_writel(value, TEGRA_PMC_CNTRL);
}

#else
static inline void set_core_power_timers(void) { }
#endif

/* IO Pads configurations */
static int tegra_pmc_io_pad_prepare(const struct tegra_pmc_io_pad_soc *pad)
{
	unsigned long rate, value;

	if (pad->dpd == UINT_MAX)
		return -ENOTSUPP;

	if (!pmc->clk)
		return 0;

	rate = clk_get_rate(pmc->clk);
	if (!rate) {
		dev_err(pmc->dev, "Failed to get clock rate\n");
		return -ENODEV;
	}

	tegra_pmc_writel(DPD_SAMPLE_ENABLE, pad->dpd_sample_reg);

	/* must be at least 200 ns, in APB (PCLK) clock cycles */
	value = DIV_ROUND_UP(1000000000, rate);
	value = DIV_ROUND_UP(200, value);
	tegra_pmc_writel(value, pad->dpd_timer_reg);

	return 0;
}

static int tegra_pmc_io_pad_poll(const struct tegra_pmc_io_pad_soc *pad,
				 u32 val, unsigned long timeout)
{
	u32 mask = BIT(pad->dpd);
	u32 value;

	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_after(timeout, jiffies)) {
		value = tegra_pmc_readl(pad->dpd_status_reg);
		if ((value & mask) == val)
			return 0;

		usleep_range(250, 1000);
	}

	return -ETIMEDOUT;
}

static void tegra_pmc_io_pad_unprepare(const struct tegra_pmc_io_pad_soc *pad)
{
	if (!pmc->clk)
		return;

	tegra_pmc_writel(DPD_SAMPLE_DISABLE, pad->dpd_sample_reg);
}

/**
 * tegra_pmc_io_pad_power_enable() - enable power to I/O pad
 * @pad: Tegra I/O pad SOC data for which to enable power
 *
 * Returns: 0 on success or a negative error code on failure.
 */
static int tegra_pmc_io_pad_power_enable(const struct tegra_pmc_io_pad_soc *pad)
{
	int err;

	mutex_lock(&pmc->powergates_lock);

	err = tegra_pmc_io_pad_prepare(pad);
	if (err < 0) {
		dev_err(pmc->dev, "Failed to prepare I/O pad %s: %d\n",
			pad->name, err);
		goto unlock;
	}

	tegra_pmc_writel(IO_DPD_REQ_CODE_OFF | BIT(pad->dpd), pad->dpd_req_reg);

	err = tegra_pmc_io_pad_poll(pad, 0, 250);
	if (err < 0) {
		dev_err(pmc->dev, "Failed to enable I/O pad %s: %d\n",
			pad->name, err);
		goto unlock;
	}

	tegra_pmc_io_pad_unprepare(pad);

unlock:
	mutex_unlock(&pmc->powergates_lock);
	return err;
}

/**
 * tegra_pmc_io_pad_power_disable() - disable power to I/O pad
 * @pad: Tegra I/O pad SOC data for which to disable power
 *
 * Returns: 0 on success or a negative error code on failure.
 */
static int tegra_pmc_io_pad_power_disable(
			const struct tegra_pmc_io_pad_soc *pad)
{
	int err;

	mutex_lock(&pmc->powergates_lock);

	err = tegra_pmc_io_pad_prepare(pad);
	if (err < 0) {
		dev_err(pmc->dev, "Failed to prepare I/O pad %s: %d\n",
			pad->name, err);
		goto unlock;
	}

	tegra_pmc_writel(IO_DPD_REQ_CODE_ON | BIT(pad->dpd), pad->dpd_req_reg);

	err = tegra_pmc_io_pad_poll(pad, BIT(pad->dpd), 250);
	if (err < 0) {
		dev_err(pmc->dev, "Failed to disable I/O pad %s: %d\n",
			pad->name, err);
		goto unlock;
	}

	tegra_pmc_io_pad_unprepare(pad);

unlock:
	mutex_unlock(&pmc->powergates_lock);
	return err;
}

static int _tegra_pmc_io_pad_set_voltage(const struct tegra_pmc_io_pad_soc *pad,
					 int io_pad_uv)
{
	u32 value;

	if (pad->voltage == UINT_MAX)
		return -ENOTSUPP;

	if ((io_pad_uv != pad->pad_uv_0) &&
	    (io_pad_uv != pad->pad_uv_1))
		return -EINVAL;

	mutex_lock(&pmc->powergates_lock);

	/* write-enable PMC_PWR_DET_VALUE[pad->voltage] */
	if (pad->io_pad_pwr_det_enable_reg != UINT_MAX) {
		value = tegra_pmc_readl(pad->io_pad_pwr_det_enable_reg);
		value |= BIT(pad->voltage);
		tegra_pmc_writel(value, pad->io_pad_pwr_det_enable_reg);
	}

	/* update I/O voltage */
	value = tegra_pmc_readl(pad->io_pad_pwr_det_val_reg);

	if (io_pad_uv == pad->pad_uv_0)
		value &= ~BIT(pad->voltage);
	else
		value |= BIT(pad->voltage);

	tegra_pmc_writel(value, pad->io_pad_pwr_det_val_reg);

	mutex_unlock(&pmc->powergates_lock);

	usleep_range(100, 250);

	return 0;
}

static int _tegra_pmc_io_pad_get_voltage(const struct tegra_pmc_io_pad_soc *pad)
{
	u32 value;

	if (pad->voltage == UINT_MAX)
		return -ENOTSUPP;

	value = tegra_pmc_readl(pad->io_pad_pwr_det_val_reg);

	if ((value & BIT(pad->voltage)) == 0)
		return pad->pad_uv_0;

	return pad->pad_uv_1;
}

/**
 * tegra_pmc_io_pad_is_powered() - check if IO pad is powered
 * @pad: Tegra I/O pad SOC data for which power status need to check
 *
 * Return 1 if power-ON, 0 if power OFF and error number in
 * negative if pad ID is not valid or power down not supported
 * on given IO pad.
 */
static int tegra_pmc_io_pad_is_powered(const struct tegra_pmc_io_pad_soc *pad)
{
	u32 value;

	if (pad->dpd == UINT_MAX)
		return -ENOTSUPP;

	value = tegra_pmc_readl(pad->dpd_status_reg);

	return !(value & BIT(pad->dpd));
}

#ifndef CONFIG_TEGRA186_PMC
int tegra186_pmc_io_dpd_enable(int reg, int bit_pos)
{
	unsigned int enable_mask;
	unsigned int dpd_status;
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);

	enable_mask = ((1 << bit_pos) | IO_DPD_REQ_CODE_ON);

	_tegra_pmc_writel(enable_mask, (T186_PMC_IO_DPD_REQ + reg * 8));
	udelay(7);

	dpd_status = _tegra_pmc_readl(T186_PMC_IO_DPD_STATUS + reg * 8);
	if (!(dpd_status & (1 << bit_pos))) {
		pr_info("Error: dpd%d enable failed, status=%#x\n",
				(reg + 1), dpd_status);
	}
	spin_unlock_irqrestore(&pwr_lock, flags);

	return 0;
}
EXPORT_SYMBOL(tegra186_pmc_io_dpd_enable);

int tegra186_pmc_io_dpd_disable(int reg, int bit_pos)
{
	unsigned int enable_mask;
	unsigned int dpd_status;
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);

	enable_mask = ((1 << bit_pos) | IO_DPD_REQ_CODE_OFF);

	_tegra_pmc_writel(enable_mask, T186_PMC_IO_DPD_REQ + reg * 8);
	udelay(7);

	dpd_status = _tegra_pmc_readl(T186_PMC_IO_DPD_STATUS + reg * 8);
	if (dpd_status & (1 << bit_pos)) {
		pr_info("Error: dpd%d disable failed, status=%#x\n",
				(reg + 1), dpd_status);
	}
	spin_unlock_irqrestore(&pwr_lock, flags);
	return 0;
}
EXPORT_SYMBOL(tegra186_pmc_io_dpd_disable);

int tegra186_pmc_io_dpd_get_status(int reg, int bit_pos)
{
	unsigned int dpd_status;

	dpd_status = _tegra_pmc_readl(T186_PMC_IO_DPD_STATUS + reg * 8);
	if (dpd_status & BIT(bit_pos))
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(tegra186_pmc_io_dpd_get_status);

void tegra_pmc_pad_voltage_update(unsigned int reg,
		unsigned long mask, unsigned long val)
{
	_tegra_pmc_register_update_lock(reg, mask, val);
}
EXPORT_SYMBOL(tegra_pmc_pad_voltage_update);

unsigned long tegra_pmc_pad_voltage_get(unsigned int reg)
{
	return _tegra_pmc_readl(reg);
}
EXPORT_SYMBOL(tegra_pmc_pad_voltage_get);
#endif

static int tegra_pmc_io_pads_pinctrl_get_groups_count(
			struct pinctrl_dev *pctldev)
{
	struct tegra_pmc *tpmc = pinctrl_dev_get_drvdata(pctldev);

	return tpmc->soc->num_io_pads;
}

static const char *tegra_pmc_io_pads_pinctrl_get_group_name(
		struct pinctrl_dev *pctldev, unsigned int group)
{
	struct tegra_pmc *tpmc = pinctrl_dev_get_drvdata(pctldev);

	return tpmc->soc->io_pads[group].name;
}

static int tegra_pmc_io_pads_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
						    unsigned int group,
						    const unsigned int **pins,
						    unsigned int *num_pins)
{
	struct tegra_pmc *tpmc = pinctrl_dev_get_drvdata(pctldev);

	*pins = tpmc->soc->io_pads[group].pins;
	*num_pins = tpmc->soc->io_pads[group].npins;

	return 0;
}

enum tegra_io_rail_pads_params {
	TEGRA_IO_PAD_POWER_SOURCE_VOLTAGE = PIN_CONFIG_END + 1,
};

static const struct pinconf_generic_params tegra_io_pads_cfg_params[] = {
	{
		.property = "nvidia,power-source-voltage",
		.param = TEGRA_IO_PAD_POWER_SOURCE_VOLTAGE,
	},
};

static const struct pinctrl_ops tegra_pmc_io_pads_pinctrl_ops = {
	.get_groups_count = tegra_pmc_io_pads_pinctrl_get_groups_count,
	.get_group_name	= tegra_pmc_io_pads_pinctrl_get_group_name,
	.get_group_pins	= tegra_pmc_io_pads_pinctrl_get_group_pins,
	.dt_node_to_map	= pinconf_generic_dt_node_to_map_pin,
	.dt_free_map	= pinconf_generic_dt_free_map,
};

static int tegra_pmc_io_pads_pinconf_get(struct pinctrl_dev *pctldev,
					 unsigned int pin,
					 unsigned long *config)
{
	struct tegra_pmc *tpmc = pinctrl_dev_get_drvdata(pctldev);
	u16 param = pinconf_to_config_param(*config);
	const struct tegra_pmc_io_pad_soc *pad = &tpmc->soc->io_pads[pin];
	u16 arg = 0;
	int ret;

	switch (param) {
	case PIN_CONFIG_LOW_POWER_MODE:
		ret = tegra_pmc_io_pad_is_powered(pad);
		if (ret < 0)
			return ret;
		arg = !ret;
		break;

	case TEGRA_IO_PAD_POWER_SOURCE_VOLTAGE:
		if (pmc->soc->io_pads[pin].voltage == UINT_MAX)
			return -EINVAL;

		ret = _tegra_pmc_io_pad_get_voltage(pad);
		if (ret < 0)
			return ret;
		arg = ret;

		break;

	default:
		dev_dbg(tpmc->dev, "I/O pad %s does not support param %d\n",
			pad->name, param);
		return -EINVAL;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int tegra_pmc_io_pads_pinconf_set(struct pinctrl_dev *pctldev,
					 unsigned int pin,
					 unsigned long *configs,
					 unsigned int num_configs)
{
	struct tegra_pmc *tpmc = pinctrl_dev_get_drvdata(pctldev);
	const struct tegra_pmc_io_pad_soc *pad = &tpmc->soc->io_pads[pin];
	unsigned int i;

	for (i = 0; i < num_configs; i++) {
		int ret;
		u16 param_val = pinconf_to_config_argument(configs[i]);
		u16 param = pinconf_to_config_param(configs[i]);

		switch (param) {
		case PIN_CONFIG_LOW_POWER_MODE:
			if (param_val)
				ret = tegra_pmc_io_pad_power_disable(pad);
			else
				ret = tegra_pmc_io_pad_power_enable(pad);
			if (ret < 0) {
				dev_err(tpmc->dev,
					"Failed to set low power %s of I/O pad %s: %d\n",
					(param_val) ? "disable" : "enable",
					pad->name, ret);
				return ret;
			}

			break;

		case TEGRA_IO_PAD_POWER_SOURCE_VOLTAGE:
			if (pmc->soc->io_pads[pin].voltage == UINT_MAX)
				return -EINVAL;

			ret = _tegra_pmc_io_pad_set_voltage(pad, param_val);
			if (ret < 0) {
				dev_err(tpmc->dev,
					"Failed to set voltage %d of pin %u: %d\n",
					param_val, pin, ret);
				return ret;
			}

			break;

		default:
			dev_err(tpmc->dev, "I/O pad %s does not support param %d\n",
				pad->name, param);
			return -EINVAL;
		}
	}

	return 0;
}

static const struct pinconf_ops tegra_pmc_io_pads_pinconf_ops = {
	.pin_config_get = tegra_pmc_io_pads_pinconf_get,
	.pin_config_set = tegra_pmc_io_pads_pinconf_set,
	.is_generic = true,
};

static int tegra_pmc_io_pads_pinctrl_init(struct tegra_pmc *pmc)
{
	if (!pmc->soc->num_descs)
		return 0;

	pmc->pinctrl_desc.name = "pinctr-pmc-io-pads";
	pmc->pinctrl_desc.pctlops = &tegra_pmc_io_pads_pinctrl_ops;
	pmc->pinctrl_desc.confops = &tegra_pmc_io_pads_pinconf_ops;
	pmc->pinctrl_desc.pins = pmc->soc->descs;
	pmc->pinctrl_desc.npins = pmc->soc->num_descs;
	pmc->pinctrl_desc.custom_params = tegra_io_pads_cfg_params;
	pmc->pinctrl_desc.num_custom_params =
				ARRAY_SIZE(tegra_io_pads_cfg_params);

	pmc->pctl = devm_pinctrl_register(pmc->dev, &pmc->pinctrl_desc, pmc);
	if (IS_ERR(pmc->pctl)) {
		int ret = PTR_ERR(pmc->pctl);

		dev_err(pmc->dev, "Failed to register pinctrl-io-pad: %d\n",
			ret);
		return ret;
	}

	return 0;
}

static const struct tegra_pmc_io_pad_soc *tegra_pmc_get_pad_by_name(
				const char *pad_name)
{
	unsigned int i;

	for (i = 0; i < pmc->soc->num_io_pads; ++i) {
		if (!strcmp(pad_name, pmc->soc->io_pads[i].name))
			return &pmc->soc->io_pads[i];
	}

	return NULL;
}

int tegra_pmc_io_pad_low_power_enable(const char *pad_name)
{
	const struct tegra_pmc_io_pad_soc *pad;

	pad = tegra_pmc_get_pad_by_name(pad_name);
	if (!pad) {
		dev_err(pmc->dev, "IO Pad %s not found\n", pad_name);
		return -EINVAL;
	}

	return tegra_pmc_io_pad_power_disable(pad);
}
EXPORT_SYMBOL(tegra_pmc_io_pad_low_power_enable);

int tegra_pmc_io_pad_low_power_disable(const char *pad_name)
{
	const struct tegra_pmc_io_pad_soc *pad;

	pad = tegra_pmc_get_pad_by_name(pad_name);
	if (!pad) {
		dev_err(pmc->dev, "IO Pad %s not found\n", pad_name);
		return -EINVAL;
	}

	return tegra_pmc_io_pad_power_enable(pad);
}
EXPORT_SYMBOL(tegra_pmc_io_pad_low_power_disable);

int tegra_pmc_io_pad_set_voltage(const char *pad_name, unsigned int pad_uv)
{
	int io_pad_uv;
	const struct tegra_pmc_io_pad_soc *pad;

	pad = tegra_pmc_get_pad_by_name(pad_name);
	if (!pad) {
		dev_err(pmc->dev, "IO Pad %s not found\n", pad_name);
		return -EINVAL;
	}

	switch (pad_uv) {
	case 1800000:
		io_pad_uv = TEGRA_IO_PAD_VOLTAGE_1800000UV;
		break;

	case 3300000:
		io_pad_uv = TEGRA_IO_PAD_VOLTAGE_3300000UV;
		break;

	default:
		return -EINVAL;
	}

	return _tegra_pmc_io_pad_set_voltage(pad, io_pad_uv);
}
EXPORT_SYMBOL(tegra_pmc_io_pad_set_voltage);

int tegra_pmc_io_pad_get_voltage(const char *pad_name)
{
	int io_pad_uv;
	const struct tegra_pmc_io_pad_soc *pad;

	pad = tegra_pmc_get_pad_by_name(pad_name);
	if (!pad) {
		dev_err(pmc->dev, "IO Pad %s not found\n", pad_name);
		return -EINVAL;
	}

	io_pad_uv = _tegra_pmc_io_pad_get_voltage(pad);
	if (io_pad_uv < 0)
		return -EINVAL;

	switch (io_pad_uv) {
	case TEGRA_IO_PAD_VOLTAGE_1800000UV:
		return 1800000;

	case TEGRA_IO_PAD_VOLTAGE_3300000UV:
		return 3300000;

	default:
		break;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_pmc_io_pad_get_voltage);

#ifndef CONFIG_TEGRA186_PMC
void tegra_pmc_nvcsi_ab_brick_update(unsigned long mask, unsigned long val)
{
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);
	tegra_pmc_register_update(TEGRA_PMC_IO_DPD_REQ, mask, val);
	spin_unlock_irqrestore(&pwr_lock, flags);
}
EXPORT_SYMBOL(tegra_pmc_nvcsi_ab_brick_update);

unsigned long tegra_pmc_nvcsi_ab_brick_getstatus(void)
{
	return tegra_pmc_readl(TEGRA_PMC_IO_DPD_REQ);
}
EXPORT_SYMBOL(tegra_pmc_nvcsi_ab_brick_getstatus);

void tegra_pmc_nvcsi_cdef_brick_update(unsigned long mask, unsigned long val)
{
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);
	tegra_pmc_register_update(TEGRA_PMC_IO_DPD2_REQ, mask, val);
	spin_unlock_irqrestore(&pwr_lock, flags);
}
EXPORT_SYMBOL(tegra_pmc_nvcsi_cdef_brick_update);

unsigned long tegra_pmc_nvcsi_cdef_brick_getstatus(void)
{
	return tegra_pmc_readl(TEGRA_PMC_IO_DPD2_REQ);
}
EXPORT_SYMBOL(tegra_pmc_nvcsi_cdef_brick_getstatus);

void tegra186_pmc_enable_nvcsi_brick_dpd(void)
{
	u32 val;

	val = tegra_pmc_readl(TEGRA_PMC_IO_DPD_REQ);
	val |= T186_PMC_IO_DPD_CSIA_MASK;
	val |= T186_PMC_IO_DPD_CSIB_MASK;
	tegra_pmc_writel(val, TEGRA_PMC_IO_DPD_REQ);

	val = tegra_pmc_readl(TEGRA_PMC_IO_DPD2_REQ);
	val |= (T186_PMC_IO_DPD2_CSIC_MASK | T186_PMC_IO_DPD2_CSID_MASK |
		T186_PMC_IO_DPD2_CSIE_MASK | T186_PMC_IO_DPD2_CSIF_MASK);
	tegra_pmc_writel(val, TEGRA_PMC_IO_DPD2_REQ);
}
EXPORT_SYMBOL(tegra186_pmc_enable_nvcsi_brick_dpd);

void tegra186_pmc_disable_nvcsi_brick_dpd(void)
{
	u32 val;

	val = tegra_pmc_readl(TEGRA_PMC_IO_DPD_REQ);
	val &= ~(T186_PMC_IO_DPD_CSIA_MASK | T186_PMC_IO_DPD_CSIB_MASK);
	tegra_pmc_writel(val, TEGRA_PMC_IO_DPD_REQ);

	val = tegra_pmc_readl(TEGRA_PMC_IO_DPD2_REQ);
	val &= ~(T186_PMC_IO_DPD2_CSIC_MASK | T186_PMC_IO_DPD2_CSID_MASK |
		 T186_PMC_IO_DPD2_CSIE_MASK | T186_PMC_IO_DPD2_CSIF_MASK);
	tegra_pmc_writel(val, TEGRA_PMC_IO_DPD2_REQ);
}
EXPORT_SYMBOL(tegra186_pmc_disable_nvcsi_brick_dpd);
#endif

static int tegra_pmc_parse_dt(struct tegra_pmc *pmc, struct device_node *np)
{
	u32 value, values[2];

	if (of_property_read_u32(np, "nvidia,suspend-mode", &value)) {
	} else {
		switch (value) {
		case 0:
			pmc->suspend_mode = TEGRA_SUSPEND_LP0;
			break;

		case 1:
			pmc->suspend_mode = TEGRA_SUSPEND_LP1;
			break;

		case 2:
			pmc->suspend_mode = TEGRA_SUSPEND_LP2;
			break;

		default:
			pmc->suspend_mode = TEGRA_SUSPEND_NONE;
			break;
		}
	}

	pmc->suspend_mode = tegra_pm_validate_suspend_mode(pmc->suspend_mode);

	if (of_property_read_u32(np, "nvidia,cpu-pwr-good-time", &value))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->cpu_good_time = value;

	if (of_property_read_u32(np, "nvidia,cpu-pwr-off-time", &value))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->cpu_off_time = value;

	if (of_property_read_u32_array(np, "nvidia,core-pwr-good-time",
				       values, ARRAY_SIZE(values)))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->core_osc_time = values[0];
	pmc->core_pmu_time = values[1];

	if (of_property_read_u32(np, "nvidia,core-pwr-off-time", &value))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->core_off_time = value;

	pmc->corereq_high = of_property_read_bool(np,
				"nvidia,core-power-req-active-high");

	pmc->sysclkreq_high = of_property_read_bool(np,
				"nvidia,sys-clock-req-active-high");

	pmc->combined_req = of_property_read_bool(np,
				"nvidia,combined-power-req");

	pmc->cpu_pwr_good_en = of_property_read_bool(np,
				"nvidia,cpu-pwr-good-en");

	if (of_property_read_u32_array(np, "nvidia,lp0-vec", values,
				       ARRAY_SIZE(values)))
		if (pmc->suspend_mode == TEGRA_SUSPEND_LP0)
			pmc->suspend_mode = TEGRA_SUSPEND_LP1;

	pmc->lp0_vec_phys = values[0];
	pmc->lp0_vec_size = values[1];

	return 0;
}

static void tegra_pmc_init(struct tegra_pmc *pmc)
{
	u32 value;

	if (pmc->soc->skip_legacy_pmc_init)
		return;

	/* Always enable CPU power request */
	value = tegra_pmc_readl(TEGRA_PMC_CNTRL);
	value |= PMC_CNTRL_CPU_PWRREQ_OE;
	tegra_pmc_writel(value, TEGRA_PMC_CNTRL);

	value = tegra_pmc_readl(TEGRA_PMC_CNTRL);

	if (pmc->sysclkreq_high)
		value &= ~PMC_CNTRL_SYSCLK_POLARITY;
	else
		value |= PMC_CNTRL_SYSCLK_POLARITY;

	/* configure the output polarity while the request is tristated */
	tegra_pmc_writel(value, TEGRA_PMC_CNTRL);

	/* now enable the request */
	value = tegra_pmc_readl(TEGRA_PMC_CNTRL);
	value |= PMC_CNTRL_SYSCLK_OE;
	tegra_pmc_writel(value, TEGRA_PMC_CNTRL);
}

static void tegra_pmc_init_tsense_reset(struct tegra_pmc *pmc)
{
	static const char disabled[] = "emergency thermal reset disabled";
	u32 pmu_addr, ctrl_id, reg_addr, reg_data, pinmux;
	struct device *dev = pmc->dev;
	struct device_node *np;
	u32 value, checksum;

	if (!pmc->soc->has_tsense_reset)
		return;

	np = of_find_node_by_name(pmc->dev->of_node, "i2c-thermtrip");
	if (!np) {
		dev_warn(dev, "i2c-thermtrip node not found, %s.\n", disabled);
		return;
	}

	if (of_property_read_u32(np, "nvidia,i2c-controller-id", &ctrl_id)) {
		dev_err(dev, "I2C controller ID missing, %s.\n", disabled);
		goto out;
	}

	if (of_property_read_u32(np, "nvidia,bus-addr", &pmu_addr)) {
		dev_err(dev, "nvidia,bus-addr missing, %s.\n", disabled);
		goto out;
	}

	if (of_property_read_u32(np, "nvidia,reg-addr", &reg_addr)) {
		dev_err(dev, "nvidia,reg-addr missing, %s.\n", disabled);
		goto out;
	}

	if (of_property_read_u32(np, "nvidia,reg-data", &reg_data)) {
		dev_err(dev, "nvidia,reg-data missing, %s.\n", disabled);
		goto out;
	}

	if (of_property_read_u32(np, "nvidia,pinmux-id", &pinmux))
		pinmux = 0;

	value = tegra_pmc_readl(TEGRA_PMC_SENSOR_CTRL);
	value |= PMC_SENSOR_CTRL_SCRATCH_WRITE;
	tegra_pmc_writel(value, TEGRA_PMC_SENSOR_CTRL);

	value = (reg_data << PMC_SCRATCH54_DATA_SHIFT) |
		(reg_addr << PMC_SCRATCH54_ADDR_SHIFT);
	tegra_pmc_writel(value, TEGRA_PMC_SCRATCH54);

	value = PMC_SCRATCH55_RESET_TEGRA;
	value |= ctrl_id << PMC_SCRATCH55_CNTRL_ID_SHIFT;
	value |= pinmux << PMC_SCRATCH55_PINMUX_SHIFT;
	value |= pmu_addr << PMC_SCRATCH55_I2CSLV1_SHIFT;

	/*
	 * Calculate checksum of SCRATCH54, SCRATCH55 fields. Bits 23:16 will
	 * contain the checksum and are currently zero, so they are not added.
	 */
	checksum = reg_addr + reg_data + (value & 0xff) + ((value >> 8) & 0xff)
		+ ((value >> 24) & 0xff);
	checksum &= 0xff;
	checksum = 0x100 - checksum;

	value |= checksum << PMC_SCRATCH55_CHECKSUM_SHIFT;

	tegra_pmc_writel(value, TEGRA_PMC_SCRATCH55);

	value = tegra_pmc_readl(TEGRA_PMC_SENSOR_CTRL);
	value |= PMC_SENSOR_CTRL_ENABLE_RST;
	tegra_pmc_writel(value, TEGRA_PMC_SENSOR_CTRL);

	dev_info(pmc->dev, "emergency thermal reset enabled\n");

out:
	of_node_put(np);
}

#ifdef CONFIG_DEBUG_FS
struct tegra_pmc_scratch_export_info {
	const char **reg_names;
	u32 *reg_offset;
	int cnt_reg_offset;
	int cnt_reg_names;
};
static struct tegra_pmc_scratch_export_info scratch_info;

static inline u32 tegra_pmc_debug_scratch_readl(u32 reg)
{
	return readl(pmc->reboot_base + reg);
}

static inline void tegra_pmc_debug_scratch_writel(u32 val, u32 reg)
{
	writel(val, pmc->reboot_base + reg);
}

static ssize_t tegra_pmc_debug_scratch_reg_read(struct file *file,
						char __user *user_buf,
						size_t count, loff_t *ppos)
{
	char buf[64] = {};
	unsigned char *dfsname = file->f_path.dentry->d_iname;
	ssize_t ret;
	u32 value;
	int id;

	for (id = 0; id < scratch_info.cnt_reg_offset; id++) {
		if (!strcmp(dfsname, scratch_info.reg_names[id]))
			break;
	}

	if (id == scratch_info.cnt_reg_offset)
		return -EINVAL;

	value = tegra_pmc_debug_scratch_readl(scratch_info.reg_offset[id]);
	ret = snprintf(buf, sizeof(buf), "Reg: 0x%x : Value: 0x%x\n",
				scratch_info.reg_offset[id], value);

	return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

static ssize_t tegra_pmc_debug_scratch_reg_write(struct file *file,
						 const char __user *user_buf,
						 size_t count, loff_t *ppos)
{
	char buf[64] = { };
	unsigned char *dfsname = file->f_path.dentry->d_iname;
	ssize_t buf_size;
	u32 value = 0;
	int id;

	for (id = 0; id < scratch_info.cnt_reg_offset; id++) {
		if (!strcmp(dfsname, scratch_info.reg_names[id]))
			break;
	}
	if (id == scratch_info.cnt_reg_offset)
		return -EINVAL;

	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (!sscanf(buf, "%x\n", &value))
		return -EINVAL;

	pr_info("PMC reg: 0x%x Value: 0x%x\n", scratch_info.reg_offset[id], value);
	tegra_pmc_debug_scratch_writel(value, scratch_info.reg_offset[id]);

	return count;
}

static const struct file_operations pmc_debugfs_fops = {
	.open		= simple_open,
	.write		= tegra_pmc_debug_scratch_reg_write,
	.read		= tegra_pmc_debug_scratch_reg_read,
};

static int tegra_pmc_debug_scratch_reg_init(struct tegra_pmc *pmc)
{
	struct device_node *np = pmc->dev->of_node;
	const char *srname;
	struct property *prop;
	int count, i;
	int ret;
	int cnt_reg_names, cnt_reg_offset;
	struct dentry *dbgfs_root;

	cnt_reg_offset = of_property_count_u32_elems(np,
					"export-pmc-scratch-reg-offset");
	if (cnt_reg_offset <= 0) {
		dev_info(pmc->dev, "scratch reg offset dts data not present\n");
		return -EINVAL;
	}

	scratch_info.cnt_reg_offset = cnt_reg_offset;

	cnt_reg_names = of_property_count_strings(np,
				"export-pmc-scratch-reg-name");
	if (cnt_reg_names < 0 || (cnt_reg_offset != cnt_reg_names)) {
		dev_info(pmc->dev, "reg offset and names count not matching\n");
		return -EINVAL;
	}

	scratch_info.cnt_reg_names = cnt_reg_names;
	scratch_info.reg_names = devm_kzalloc(pmc->dev, (cnt_reg_offset + 1) *
					   sizeof(*scratch_info.reg_names),
					   GFP_KERNEL);
	if (!scratch_info.reg_names)
		return -ENOMEM;

	count = 0;
	of_property_for_each_string(np, "export-pmc-scratch-reg-name",
				    prop, srname)
		scratch_info.reg_names[count++] = srname;

	scratch_info.reg_names[count] = NULL;

	scratch_info.reg_offset = devm_kzalloc(pmc->dev, sizeof(u32) *
					    cnt_reg_offset, GFP_KERNEL);
	if (!scratch_info.reg_offset)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "export-pmc-scratch-reg-offset",
					 scratch_info.reg_offset, cnt_reg_offset);
	if (ret < 0)
		return -ENODEV;

	dbgfs_root = debugfs_create_dir("PMC", NULL);
	if (!dbgfs_root) {
		dev_info(pmc->dev, "PMC:Failed to create debugfs dir\n");
		return -ENOMEM;
	}

	for (i = 0; i < cnt_reg_offset; i++) {
		debugfs_create_file(scratch_info.reg_names[i], S_IRUGO | S_IWUSR,
				    dbgfs_root, NULL, &pmc_debugfs_fops);
		dev_info(pmc->dev, "create /sys/kernel/debug/%s/%s\n",
			 dbgfs_root->d_name.name, scratch_info.reg_names[i]);
	}

	return 0;
}

#else
static int tegra_pmc_debug_scratch_reg_init(struct device_node *np)
{
	return 0;
}
#endif

#ifndef CONFIG_TEGRA186_PMC
bool tegra_pmc_is_halt_in_fiq(void)
{
	return !!(PMC_IMPL_HALT_IN_FIQ_MASK &
		tegra_pmc_readl(TEGRA_PMC_IMPL_RAMDUMP_CTL_STATUS));
}
EXPORT_SYMBOL(tegra_pmc_is_halt_in_fiq);
#endif

static void tegra_pmc_halt_in_fiq_init(struct tegra_pmc *pmc)
{
	struct device_node *np = pmc->dev->of_node;

	if (!of_property_read_bool(np, "nvidia,enable-halt-in-fiq"))
		return;

	tegra_pmc_register_update(TEGRA_PMC_IMPL_RAMDUMP_CTL_STATUS,
				  PMC_IMPL_HALT_IN_FIQ_MASK,
				  PMC_IMPL_HALT_IN_FIQ_MASK);
}

static void tegra_pmc_show_reset_status(void)
{
	u32 val, rst_src, rst_lvl;
	char *reset_source[] = {
		"Power on reset",
		"AOWDT",
		"Denvor watchdog time out",
		"BPMPWDT",
		"SCEWDT",
		"SPEWDT",
		"APEWDT",
		"A57 watchdog time out",
		"SENSOR",
		"AOTAG",
		"VFSENSOR",
		"Software reset",
		"SC7",
		"HSM",
		"CSITE",
	};
	char *reset_level[] = {
		"L0",
		"L1",
		"L2",
		"WARM",
	};

	val = tegra_pmc_readl(TEGRA_PMC_RST_STATUS);
	rst_src = (val & PMC_RST_SOURCE_MASK) >> PMC_RST_SOURCE_SHIFT;
	rst_lvl = (val & PMC_RST_LEVEL_MASK) >> PMC_RST_LEVEL_SHIFT;
	pr_info("### PMC reset source: %s\n", reset_source[rst_src]);
	pr_info("### PMC reset level: %s\n", reset_level[rst_lvl]);
	pr_info("### PMC reset status reg: 0x%x\n", val);
}

static int tegra_pmc_probe(struct platform_device *pdev)
{
	void __iomem *base;
	struct resource *res;
	int err;

	/*
	 * Early initialisation should have configured an initial
	 * register mapping and setup the soc data pointer. If these
	 * are not valid then something went badly wrong!
	 */
	if (WARN_ON(!pmc->base || !pmc->soc))
		return -ENODEV;

	err = tegra_pmc_parse_dt(pmc, pdev->dev.of_node);
	if (err < 0)
		return err;

	/* take over the memory region from the early initialization */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	if (pmc->soc->has_reboot_base_address) {
		base =  pmc->reboot_base;
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		pmc->reboot_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(pmc->reboot_base))
			return PTR_ERR(pmc->reboot_base);

		iounmap(base);
	} else {
		pmc->reboot_base = pmc->base;
	}

	if (pmc->soc->has_pclk_clock) {
		pmc->clk = devm_clk_get(&pdev->dev, "pclk");
		if (IS_ERR(pmc->clk)) {
			err = PTR_ERR(pmc->clk);
			dev_err(&pdev->dev, "failed to get pclk: %d\n", err);
			return err;
		}
	}

	pmc->dev = &pdev->dev;

	tegra_pmc_init(pmc);

	tegra_pmc_init_tsense_reset(pmc);

	tegra_pmc_halt_in_fiq_init(pmc);

	tegra_pmc_debug_scratch_reg_init(pmc);

	if (pmc->soc->show_reset_status)
		tegra_pmc_show_reset_status();

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_powergate_debugfs_init();
		if (err < 0)
			return err;
	}

	if (!pmc->soc->skip_restart_register) {
		err = register_restart_handler(&tegra_pmc_restart_handler);
		if (err) {
			debugfs_remove(pmc->debugfs);
			dev_err(&pdev->dev, "unable to register restart handler, %d\n",
				err);
			return err;
		}
	}

	mutex_lock(&pmc->powergates_lock);
	iounmap(pmc->base);
	pmc->base = base;
	mutex_unlock(&pmc->powergates_lock);

	/* Prod setting like platform specific rails */
	prod_list = devm_tegra_prod_get(&pdev->dev);
	if (IS_ERR(prod_list)) {
		err = PTR_ERR(prod_list);
		dev_info(&pdev->dev, "prod list not found: %d\n",
			 err);
		prod_list = NULL;
	} else {
		err = tegra_prod_set_by_name(&base,
				"prod_c_platform_pad_rail", prod_list);
		if (err < 0) {
			dev_info(&pdev->dev,
				 "prod setting for rail not found\n");
		} else {
			dev_info(&pdev->dev,
				 "POWER_DET: 0x%08x, POWR_VAL: 0x%08x\n",
				 _tegra_pmc_readl(PMC_PWR_DET_ENABLE),
				 _tegra_pmc_readl(PMC_PWR_DET_VAL));
		}
	}

	err = tegra_pmc_io_pads_pinctrl_init(pmc);
	if (err < 0)
		return err;

	/* Register as pad controller */
	err = tegra_io_pads_padctrl_init(&pdev->dev);
	if (err)
		pr_err("ERROR: Pad control driver init failed: %d\n", err);

#ifdef CONFIG_TEGRA210_BOOTROM_PMC
	err = tegra210_boorom_pmc_init(&pdev->dev);
	if (err < 0)
		pr_err("ERROR: Bootrom PMC config failed: %d\n", err);
#endif

	/* handle PMC reboot reason with PSCI */
	if (!pmc->soc->skip_arm_pm_restart && arm_pm_restart)
		psci_handle_reboot_cmd = tegra_pmc_program_reboot_reason;

	return 0;
}

#if defined(CONFIG_PM_SLEEP) && defined(CONFIG_ARM)
static int tegra_pmc_suspend(struct device *dev)
{
	_tegra_pmc_writel(virt_to_phys(tegra_resume), PMC_SCRATCH41);

	return 0;
}

static int tegra_pmc_resume(struct device *dev)
{
	_tegra_pmc_writel(0x0, PMC_SCRATCH41);

	return 0;
}

static SIMPLE_DEV_PM_OPS(tegra_pmc_pm_ops, tegra_pmc_suspend, tegra_pmc_resume);

#endif

#ifdef CONFIG_TEGRA_POWERGATE
#define TEGRA_POWERGATE_CPU     0
#define TEGRA_POWERGATE_3D      1
#define TEGRA_POWERGATE_VENC    2
#define TEGRA_POWERGATE_PCIE    3
#define TEGRA_POWERGATE_VDEC    4
#define TEGRA_POWERGATE_L2      5
#define TEGRA_POWERGATE_MPE     6
#define TEGRA_POWERGATE_HEG     7
#define TEGRA_POWERGATE_SATA    8
#define TEGRA_POWERGATE_CPU1    9
#define TEGRA_POWERGATE_CPU2    10
#define TEGRA_POWERGATE_CPU3    11
#define TEGRA_POWERGATE_CELP    12
#define TEGRA_POWERGATE_3D1     13
#define TEGRA_POWERGATE_CPU0    14
#define TEGRA_POWERGATE_C0NC    15
#define TEGRA_POWERGATE_C1NC    16
#define TEGRA_POWERGATE_SOR     17
#define TEGRA_POWERGATE_DIS     18
#define TEGRA_POWERGATE_DISB    19
#define TEGRA_POWERGATE_XUSBA   20
#define TEGRA_POWERGATE_XUSBB   21
#define TEGRA_POWERGATE_XUSBC   22
#define TEGRA_POWERGATE_VIC     23
#define TEGRA_POWERGATE_IRAM    24
#define TEGRA_POWERGATE_NVDEC   25
#define TEGRA_POWERGATE_NVJPG   26
#define TEGRA_POWERGATE_AUD     27
#define TEGRA_POWERGATE_DFD     28
#define TEGRA_POWERGATE_VE2     29
#endif

static const char * const tegra20_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "cpu",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_L2] = "l2",
	[TEGRA_POWERGATE_MPE] = "mpe",
};

static const struct tegra_pmc_soc tegra20_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra20_powergates),
	.powergates = tegra20_powergates,
	.num_cpu_powergates = 0,
	.cpu_powergates = NULL,
	.has_tsense_reset = false,
	.has_gpu_clamps = false,
};

static const char * const tegra30_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "cpu0",
	[TEGRA_POWERGATE_3D] = "3d0",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_L2] = "l2",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_SATA] = "sata",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_3D1] = "3d1",
};

static const u8 tegra30_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra30_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra30_powergates),
	.powergates = tegra30_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra30_cpu_powergates),
	.cpu_powergates = tegra30_cpu_powergates,
	.has_tsense_reset = true,
	.has_gpu_clamps = false,
};

static const char * const tegra114_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "crail",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_CPU0] = "cpu0",
	[TEGRA_POWERGATE_C0NC] = "c0nc",
	[TEGRA_POWERGATE_C1NC] = "c1nc",
	[TEGRA_POWERGATE_DIS] = "dis",
	[TEGRA_POWERGATE_DISB] = "disb",
	[TEGRA_POWERGATE_XUSBA] = "xusba",
	[TEGRA_POWERGATE_XUSBB] = "xusbb",
	[TEGRA_POWERGATE_XUSBC] = "xusbc",
};

static const u8 tegra114_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra114_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra114_powergates),
	.powergates = tegra114_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra114_cpu_powergates),
	.cpu_powergates = tegra114_cpu_powergates,
	.has_tsense_reset = true,
	.has_gpu_clamps = false,
};

static const char * const tegra124_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "crail",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_SATA] = "sata",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_CPU0] = "cpu0",
	[TEGRA_POWERGATE_C0NC] = "c0nc",
	[TEGRA_POWERGATE_C1NC] = "c1nc",
	[TEGRA_POWERGATE_SOR] = "sor",
	[TEGRA_POWERGATE_DIS] = "dis",
	[TEGRA_POWERGATE_DISB] = "disb",
	[TEGRA_POWERGATE_XUSBA] = "xusba",
	[TEGRA_POWERGATE_XUSBB] = "xusbb",
	[TEGRA_POWERGATE_XUSBC] = "xusbc",
	[TEGRA_POWERGATE_VIC] = "vic",
	[TEGRA_POWERGATE_IRAM] = "iram",
};

static const u8 tegra124_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra124_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra124_powergates),
	.powergates = tegra124_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra124_cpu_powergates),
	.cpu_powergates = tegra124_cpu_powergates,
	.has_tsense_reset = true,
	.has_gpu_clamps = true,
};

static const unsigned long tegra210_register_map[TEGRA_PMC_MAX_REG] = {
	[TEGRA_PMC_CNTRL]		=  0x00,
	[TEGRA_PMC_WAKE_MASK]		=  0x0c,
	[TEGRA_PMC_WAKE_LEVEL]		=  0x10,
	[TEGRA_PMC_WAKE_STATUS]		=  0x14,
	[TEGRA_PMC_SW_WAKE_STATUS]	=  0x18,
	[TEGRA_PMC_WAKE_DELAY]		=  0xe0,
	[TEGRA_PMC_WAKE2_MASK]		=  0x160,
	[TEGRA_PMC_WAKE2_LEVEL]		=  0x164,
	[TEGRA_PMC_WAKE2_STATUS]	=  0x168,
	[TEGRA_PMC_SW_WAKE2_STATUS]	=  0x16c,
	[TEGRA_PMC_IO_DPD_SAMPLE]	=  0x20,
	[TEGRA_PMC_IO_DPD_ENABLE]	=  0x24,
	[TEGRA_PMC_IO_DPD_REQ]		=  0x1b8,
	[TEGRA_PMC_IO_DPD_STATUS]	=  0x1bc,
	[TEGRA_PMC_IO_DPD2_REQ]		=  0x1c0,
	[TEGRA_PMC_IO_DPD2_STATUS]	=  0x1c4,
	[TEGRA_PMC_SEL_DPD_TIM]		=  0x1c8,
	[TEGRA_PMC_PWR_NO_IOPOWER]	=  0x44,
	[TEGRA_PMC_PWR_DET_ENABLE]	=  0x48,
	[TEGRA_PMC_PWR_DET_VAL]		=  0xe4,
	[TEGRA_PMC_REMOVE_CLAMPING]	=  0x34,
	[TEGRA_PMC_PWRGATE_TOGGLE]	=  0x30,
	[TEGRA_PMC_PWRGATE_STATUS]	=  0x38,
	[TEGRA_PMC_COREPWRGOOD_TIMER]	=  0x3c,
	[TEGRA_PMC_COREPWROFF_TIMER]	=  0xe0,
	[TEGRA_PMC_CPUPWRGOOD_TIMER]	=  0xc8,
	[TEGRA_PMC_CPUPWROFF_TIMER]	=  0xcc,
	[TEGRA_PMC_SENSOR_CTRL]		=  0x1b0,
	[TEGRA_PMC_GPU_RG_CNTRL]	=  0x2d4,
	[TEGRA_PMC_FUSE_CTRL]		=  0x450,
	[TEGRA_PMC_BR_COMMAND_BASE]	=  0x908,
	[TEGRA_PMC_SCRATCH0]		=  0x50,
	[TEGRA_PMC_SCRATCH1]		=  0x54,
	[TEGRA_PMC_SCRATCH41]		=  0x140,
	[TEGRA_PMC_SCRATCH54]		=  0x258,
	[TEGRA_PMC_SCRATCH55]		=  0x25c,
};

static const char * const tegra210_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "crail",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_SATA] = "sata",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CPU0] = "cpu0",
	[TEGRA_POWERGATE_C0NC] = "c0nc",
	[TEGRA_POWERGATE_SOR] = "sor",
	[TEGRA_POWERGATE_DIS] = "dis",
	[TEGRA_POWERGATE_DISB] = "disb",
	[TEGRA_POWERGATE_XUSBA] = "xusba",
	[TEGRA_POWERGATE_XUSBB] = "xusbb",
	[TEGRA_POWERGATE_XUSBC] = "xusbc",
	[TEGRA_POWERGATE_VIC] = "vic",
	[TEGRA_POWERGATE_IRAM] = "iram",
	[TEGRA_POWERGATE_NVDEC] = "nvdec",
	[TEGRA_POWERGATE_NVJPG] = "nvjpg",
	[TEGRA_POWERGATE_AUD] = "aud",
	[TEGRA_POWERGATE_DFD] = "dfd",
	[TEGRA_POWERGATE_VE2] = "ve2",
};

static const u8 tegra210_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

#define TEGRA210_IO_PAD_CONFIG(_pin, _npins, _name, _dpd,	\
			       _vbit, _iopower, _reg)		\
	{							\
		.name =  #_name,				\
		.pins = {(_pin)},				\
		.npins = _npins,				\
		.dpd = _dpd,					\
		.voltage = _vbit,				\
		.io_power = _iopower,				\
		.dpd_req_reg = TEGRA_PMC_IO_##_reg##_REQ,	\
		.dpd_status_reg = TEGRA_PMC_IO_##_reg##_STATUS,	\
		.dpd_timer_reg = TEGRA_PMC_SEL_DPD_TIM,		\
		.dpd_sample_reg = TEGRA_PMC_IO_DPD_SAMPLE,	\
		.bdsdmem_cfc = false,				\
		.io_pad_pwr_det_enable_reg = TEGRA_PMC_PWR_DET_ENABLE, \
		.io_pad_pwr_det_val_reg = TEGRA_PMC_PWR_DET_VAL, \
		.pad_uv_0 = TEGRA_IO_PAD_VOLTAGE_1800000UV,	\
		.pad_uv_1 = TEGRA_IO_PAD_VOLTAGE_3300000UV,	\
	},

/**
 * All IO pads of Tegra SoCs do not support the low power and multi level
 * voltage configurations for its pads.
 * Defining macros for different cases as follows:
 * TEGRA_IO_PAD_LPONLY : IO pad which support low power state but
 *			 operate in single level of IO voltage.
 * TEGRA_IO_PAD_LP_N_PV: IO pad which support low power state as well as
 *			 it can operate in multi-level voltages.
 * TEGRA_IO_PAD_PVONLY:  IO pad which does not support low power state but
 *			 it can operate in multi-level voltages.
 */
#define TEGRA210_IO_PAD_LPONLY(_pin, _name, _dpd, _reg)	\
	TEGRA210_IO_PAD_CONFIG(_pin, 1, _name, _dpd, UINT_MAX, UINT_MAX, _reg)

#define TEGRA210_IO_PAD_LP_N_PV(_pin, _name, _dpd, _vbit, _io , _reg)  \
	TEGRA210_IO_PAD_CONFIG(_pin, 1, _name, _dpd, _vbit, _io, _reg)

#define TEGRA210_IO_PAD_PVONLY(_pin, _name, _vbit, _io, _reg)	\
	TEGRA210_IO_PAD_CONFIG(_pin, 0, _name, UINT_MAX, _vbit, _io, _reg)

#define TEGRA210_IO_PAD_DESC_LP(_pin, _name, _dpd, _reg)	\
	{					\
		.number = _pin,			\
		.name = #_name,			\
	},
#define TEGRA210_IO_PAD_DESC_LP_N_PV(_pin, _name, _dpd, _vbit, _io, _reg) \
	TEGRA210_IO_PAD_DESC_LP(_pin, _name, _dpd, _reg)

#define TEGRA210_IO_PAD_DESC_PV(_pin, _name, _vbit, _io, _reg) \
	TEGRA210_IO_PAD_DESC_LP(_pin, _name, UINT_MAX, _reg)

#define TEGRA210_IO_PAD_TABLE(_lponly_, _pvonly_, _lp_n_pv_)	\
	_lp_n_pv_(0, audio, 17, 5, 5, DPD)		\
	_lp_n_pv_(1, audio-hv, 29, 18, 18, DPD2)	\
	_lp_n_pv_(2, cam, 4, 10, 10, DPD2)		\
	_lponly_(3, csia, 0, DPD)			\
	_lponly_(4, csib, 1, DPD)			\
	_lponly_(5, csic, 10, DPD2)			\
	_lponly_(6, csid, 11, DPD2)			\
	_lponly_(7, csie, 12, DPD2)			\
	_lponly_(8, csif, 13, DPD2)			\
	_lp_n_pv_(9, dbg, 25, 19, 19, DPD)		\
	_lponly_(10, debug-nonao, 26, DPD)		\
	_lp_n_pv_(11, dmic, 18, 20, 20, DPD2)		\
	_lponly_(12, dp, 19, DPD2)			\
	_lponly_(13, dsi, 2, DPD)			\
	_lponly_(14, dsib, 7, DPD2)			\
	_lponly_(15, dsic, 8, DPD2)			\
	_lponly_(16, dsid, 9, DPD2)			\
	_lponly_(17, emmc, 3, DPD2)			\
	_lponly_(18, emmc2, 5, DPD2)			\
	_lp_n_pv_(19, gpio, 27, 21, 21, DPD)		\
	_lponly_(20, hdmi, 28, DPD)			\
	_lponly_(21, hsic, 19, DPD)			\
	_lponly_(22, lvds, 25, DPD2)			\
	_lponly_(23, mipi-bias, 3, DPD)			\
	_lponly_(24, pex-bias, 4, DPD)			\
	_lponly_(25, pex-clk1, 5, DPD)			\
	_lponly_(26, pex-clk2, 6, DPD)			\
	_pvonly_(27, pex-ctrl, 0, 11, DPD2)		\
	_lp_n_pv_(28, sdmmc1, 1, 12, 12, DPD2)		\
	_lp_n_pv_(29, sdmmc3, 2, 13, 13, DPD2)		\
	_lp_n_pv_(30, spi, 14, 22, 22, DPD2)		\
	_lp_n_pv_(31, spi-hv, 15, 23, 23, DPD2)		\
	_lp_n_pv_(32, uart, 14, 2, 2, DPD)		\
	_lponly_(33, usb0, 9, DPD)			\
	_lponly_(34, usb1, 10, DPD)			\
	_lponly_(35, usb2, 11, DPD)			\
	_lponly_(36, usb3, 18, DPD)			\
	_lponly_(37, usb-bias, 12, DPD)

static const struct tegra_pmc_io_pad_soc tegra210_io_pads[] = {
	TEGRA210_IO_PAD_TABLE(TEGRA210_IO_PAD_LPONLY, TEGRA210_IO_PAD_PVONLY,
			      TEGRA210_IO_PAD_LP_N_PV)
};

static const struct pinctrl_pin_desc tegra210_io_pads_pinctrl_desc[] = {
	TEGRA210_IO_PAD_TABLE(TEGRA210_IO_PAD_DESC_LP, TEGRA210_IO_PAD_DESC_PV,
			      TEGRA210_IO_PAD_DESC_LP_N_PV)
};

static const struct tegra_pmc_soc tegra210_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra210_powergates),
	.powergates = tegra210_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra210_cpu_powergates),
	.cpu_powergates = tegra210_cpu_powergates,
	.has_tsense_reset = true,
	.has_gpu_clamps = true,
	.has_ps18 = true,
	.has_pclk_clock = true,
	.has_interrupt_polarity_support = true,
	.show_reset_status = false,
	.has_reboot_base_address = false,
	.skip_lp0_vector_setup = false,
	.skip_legacy_pmc_init = false,
	.skip_power_gate_debug_fs_init = false,
	.skip_restart_register = false,
	.skip_arm_pm_restart = false,
	.num_io_pads = ARRAY_SIZE(tegra210_io_pads),
	.io_pads = tegra210_io_pads,
	.num_descs = ARRAY_SIZE(tegra210_io_pads_pinctrl_desc),
	.descs = tegra210_io_pads_pinctrl_desc,
	.rmap = tegra210_register_map,
};

/* Tegra 186 register map */
static const unsigned long tegra186_register_map[TEGRA_PMC_MAX_REG] = {
	[TEGRA_PMC_FUSE_CTRL]			= 0x100,
	[TEGRA_PMC_IMPL_RAMDUMP_CTL_STATUS]	= 0x10C,
	[TEGRA_PMC_E_18V_PWR]			= 0x3C,
	[TEGRA_PMC_E_33V_PWR]			= 0x40,
	[TEGRA_PMC_PWR_NO_IOPOWER]		= 0x44,
	[TEGRA_PMC_RST_STATUS]			= 0x70,
	[TEGRA_PMC_SATA_PWRGT_0]		= 0x68,
	[TEGRA_PMC_UFSHC_PWR_CNTRL_0]		= 0xF4,
	[TEGRA_PMC_IO_DPD_REQ]			= 0x74,
	[TEGRA_PMC_IO_DPD_STATUS]		= 0x78,
	[TEGRA_PMC_IO_DPD2_REQ]			= 0x7C,
	[TEGRA_PMC_IO_DPD2_STATUS]		= 0x80,
	[TEGRA_PMC_SCRATCH0]			= 0x2000,
};

#define TEGRA186_IO_PAD_CONFIG(_pin, _npins, _name, _dpd_reg, _dpd_bit,     \
			       _padv_reg, _padv_bit, _v0, _v1, _iopwr_bit,  \
			       _bds)  \
	{							\
		.name =  #_name,				\
		.pins = {(_pin)},				\
		.npins = _npins,				\
		.dpd_req_reg = TEGRA_PMC_IO_##_dpd_reg##_REQ,	\
		.dpd_status_reg = TEGRA_PMC_IO_##_dpd_reg##_STATUS,	\
		.dpd_timer_reg = TEGRA_PMC_SEL_DPD_TIM,		\
		.dpd_sample_reg = TEGRA_PMC_IO_DPD_SAMPLE,	\
		.dpd = _dpd_bit,				\
		.io_pad_pwr_det_val_reg = TEGRA_PMC_##_padv_reg##_PWR, \
		.io_pad_pwr_det_enable_reg = UINT_MAX,		\
		.pad_uv_0 = TEGRA_IO_PAD_VOLTAGE_##_v0##000UV,	\
		.pad_uv_1 = TEGRA_IO_PAD_VOLTAGE_##_v1##000UV,	\
		.voltage = _padv_bit,				\
		.io_power = _iopwr_bit,				\
		.bdsdmem_cfc = _bds,				\
	},

#define TEGRA186_IO_PAD_LPONLY(_pin, _name, _dpd_reg, _dpd_bit, _iopwr_bit, \
			       _bds)					    \
	TEGRA186_IO_PAD_CONFIG(_pin, 1, _name, _dpd_reg, _dpd_bit,	    \
			       E_33V, UINT_MAX, 1200, 1200, _iopwr_bit, _bds)

#define TEGRA186_IO_PAD_LP_N_PV(_pin, _name, _dpd_reg, _dpd_bit, _padv_reg, \
				_padv_bit, _v0, _v1, _iopwr_bit, _bds)	    \
	TEGRA186_IO_PAD_CONFIG(_pin, 1, _name, _dpd_reg, _dpd_bit,	    \
			       _padv_reg, _padv_bit, _v0, _v1, _iopwr_bit,  \
			       _bds)

#define TEGRA186_IO_PAD_PVONLY(_pin, _name, _padv_reg, _padv_bit, _v0, _v1, \
			       _iopwr_bit, _bds)			    \
	TEGRA186_IO_PAD_CONFIG(_pin, 1, _name, DPD, UINT_MAX, _padv_reg,    \
			       _padv_bit, _v0, _v1, _iopwr_bit, _bds)

#define TEGRA186_IO_PAD_DESC_LP(_pin, _name, _dpd_reg, _dpd_bit, _iopwr_bit, \
				_bds)					     \
	{								\
		.number = _pin,						\
		.name = #_name,						\
	},

#define TEGRA186_IO_PAD_DESC_LP_N_PV(_pin, _name, _dpd_reg, _dpd_bit,	\
				     _padv_reg, _padv_bit, _v0, _v1,	\
				     _iopwr_bit, _bds)			\
	TEGRA186_IO_PAD_DESC_LP(_pin, _name, _dpd_reg, _dpd_bit, _iopwr_bit, _bds)

#define TEGRA186_IO_PAD_DESC_PV(_pin, _name, _padv_reg, _padv_bit, _v0, _v1, \
			       _iopwr_bit, _bds)			    \
	TEGRA186_IO_PAD_DESC_LP(_pin, _name, UINT_MAX, UINT_MAX, UINT_MAX, UINT_MAX)

#define TEGRA186_IO_PAD_TABLE(_lponly_, _pvonly_, _lp_n_pv_)		\
        _lponly_(0, csia, DPD, 0, UINT_MAX, false)			\
        _lponly_(1, csib, DPD, 1, UINT_MAX, false)			\
        _lponly_(2, dsi, DPD, 2, UINT_MAX, false)			\
        _lponly_(3, mipi-bias, DPD, 3, 9, false)			\
        _lponly_(4, pex-clk-bias, DPD, 4, UINT_MAX, false)		\
        _lponly_(5, pex-clk3, DPD, 5, UINT_MAX, false)			\
        _lponly_(6, pex-clk2, DPD, 6, UINT_MAX, false)			\
        _lponly_(7, pex-clk1, DPD, 7, UINT_MAX, false)			\
        _lponly_(8, usb0, DPD, 9, UINT_MAX, false)			\
        _lponly_(9, usb1, DPD, 10, UINT_MAX, false)			\
        _lponly_(10, usb2, DPD, 11, UINT_MAX, false)			\
        _lponly_(11, usb-bias, DPD, 12, UINT_MAX, false)		\
        _lponly_(12, uart, DPD, 14, 2, false)				\
        _lponly_(13, audio, DPD, 17, 5, false)				\
        _lponly_(14, hsic, DPD, 19, UINT_MAX, false)			\
        _lp_n_pv_(15, dbg, DPD, 25, E_18V, 4, 1200, 1800, 19, false)	\
        _lponly_(16, hdmi-dp0, DPD, 28, UINT_MAX, false)		\
        _lponly_(17, hdmi-dp1, DPD, 29, UINT_MAX, false)		\
        _lponly_(18, pex-ctrl, DPD2, 0, 11, false)			\
        _lp_n_pv_(19, sdmmc2-hv, DPD2, 2, E_33V, 5, 1800, 3300, 30, true) \
        _lponly_(20, sdmmc4, DPD2, 4, 14, false)			\
        _lponly_(21, cam, DPD2, 6, 10, false)				\
        _lponly_(22, dsib, DPD2, 8, UINT_MAX, false)			\
        _lponly_(23, dsic, DPD2, 9, UINT_MAX, false)			\
        _lponly_(24, dsid, DPD2, 10, UINT_MAX, false)			\
        _lponly_(25, csic, DPD2, 11, UINT_MAX, false)			\
        _lponly_(26, csid, DPD2, 12, UINT_MAX, false)			\
        _lponly_(27, csie, DPD2, 13, UINT_MAX, false)			\
        _lponly_(28, csif, DPD2, 14, UINT_MAX, false)			\
        _lp_n_pv_(29, spi, DPD2, 15, E_18V, 5, 1200, 1800, 22, false)	\
        _lp_n_pv_(30, ufs, DPD2, 17, E_18V, 0, 1200, 1800, 6, false)	\
        _lp_n_pv_(31, dmic-hv, DPD2, 20, E_33V, 2, 1800, 3300, 28, true)   \
        _lponly_(32, edp, DPD2, 21, 4, false)				\
        _lp_n_pv_(33, sdmmc1-hv, DPD2, 23, E_33V, 4, 1800, 3300, 15, true) \
        _lp_n_pv_(34, sdmmc3-hv, DPD2, 24, E_33V, 6, 1800, 3300, 31, true) \
        _lponly_(35, conn, DPD2, 28, 3, false)				\
        _lp_n_pv_(36, audio-hv, DPD2, 29, E_33V, 1, 1800, 3300, 18, true) \
	_pvonly_(37, ao-hv, E_33V, 0, 1800, 3300, 27, true)

static const struct tegra_pmc_io_pad_soc tegra186_io_pads[] = {
	TEGRA186_IO_PAD_TABLE(TEGRA186_IO_PAD_LPONLY, TEGRA186_IO_PAD_PVONLY,
			      TEGRA186_IO_PAD_LP_N_PV)
};

static const struct pinctrl_pin_desc tegra186_io_pads_pinctrl_desc[] = {
	TEGRA186_IO_PAD_TABLE(TEGRA186_IO_PAD_DESC_LP, TEGRA186_IO_PAD_DESC_PV,
			      TEGRA186_IO_PAD_DESC_LP_N_PV)
};

static const struct tegra_pmc_soc tegra186_pmc_soc = {
	.has_tsense_reset = false,
	.has_pclk_clock = false,
	.has_interrupt_polarity_support = false,
	.show_reset_status = true,
	.has_reboot_base_address = true,
	.skip_lp0_vector_setup = true,
	.skip_legacy_pmc_init = true,
	.skip_power_gate_debug_fs_init = true,
	.skip_restart_register = true,
	.skip_arm_pm_restart = true,
	.num_io_pads = ARRAY_SIZE(tegra186_io_pads),
	.io_pads = tegra186_io_pads,
	.num_descs = ARRAY_SIZE(tegra186_io_pads_pinctrl_desc),
	.descs = tegra186_io_pads_pinctrl_desc,
	.rmap = tegra186_register_map,
};

static const struct of_device_id tegra_pmc_match[] = {
	{ .compatible = "nvidia,tegra186-pmc", .data = &tegra186_pmc_soc },
	{ .compatible = "nvidia,tegra210-pmc", .data = &tegra210_pmc_soc },
	{ .compatible = "nvidia,tegra132-pmc", .data = &tegra124_pmc_soc },
	{ .compatible = "nvidia,tegra124-pmc", .data = &tegra124_pmc_soc },
	{ .compatible = "nvidia,tegra114-pmc", .data = &tegra114_pmc_soc },
	{ .compatible = "nvidia,tegra30-pmc", .data = &tegra30_pmc_soc },
	{ .compatible = "nvidia,tegra20-pmc", .data = &tegra20_pmc_soc },
	{ }
};

static struct platform_driver tegra_pmc_driver = {
	.driver = {
		.name = "tegra-pmc",
		.suppress_bind_attrs = true,
		.of_match_table = tegra_pmc_match,
#if defined(CONFIG_PM_SLEEP) && defined(CONFIG_ARM)
		.pm = &tegra_pmc_pm_ops,
#endif
	},
	.probe = tegra_pmc_probe,
};

static int __init _tegra_pmc_driver_init(void)
{
	return platform_driver_register(&tegra_pmc_driver);
}
arch_initcall(_tegra_pmc_driver_init);

/*
 * Early initialization to allow access to registers in the very early boot
 * process.
 */
static int __init tegra_pmc_early_init(void)
{
	const struct of_device_id *match;
	struct device_node *np;
	struct resource regs;
	bool invert;
	u32 value;

	mutex_init(&pmc->powergates_lock);

	np = of_find_matching_node_and_match(NULL, tegra_pmc_match, &match);
	if (!np) {
		/*
		 * Fall back to legacy initialization for 32-bit ARM only. All
		 * 64-bit ARM device tree files for Tegra are required to have
		 * a PMC node.
		 *
		 * This is for backwards-compatibility with old device trees
		 * that didn't contain a PMC node. Note that in this case the
		 * SoC data can't be matched and therefore powergating is
		 * disabled.
		 */
		if (IS_ENABLED(CONFIG_ARM) && soc_is_tegra()) {
			pr_warn("DT node not found, powergating disabled\n");

			regs.start = 0x7000e400;
			regs.end = 0x7000e7ff;
			regs.flags = IORESOURCE_MEM;

			pr_warn("Using memory region %pR\n", &regs);
		} else {
			/*
			 * At this point we're not running on Tegra, so play
			 * nice with multi-platform kernels.
			 */
			return 0;
		}
	} else {
		/*
		 * Extract information from the device tree if we've found a
		 * matching node.
		 */
		if (of_address_to_resource(np, 0, &regs) < 0) {
			pr_err("failed to get PMC registers\n");
			of_node_put(np);
			return -ENXIO;
		}
	}

	pmc->base = ioremap_nocache(regs.start, resource_size(&regs));
	if (!pmc->base) {
		pr_err("failed to map PMC registers\n");
		of_node_put(np);
		return -ENXIO;
	}

	if (np) {
		pmc->soc = match->data;

#ifndef CONFIG_TEGRA_POWERGATE
		tegra_powergate_init(pmc, np);
#endif

	if (pmc->soc && pmc->soc->has_reboot_base_address) {
		pmc->reboot_base = of_iomap(np, 1);
		if (!pmc->reboot_base) {
			pr_err("Failed to map reboot PMC registers\n");
			return -ENXIO;
		}
	} else {
		pmc->reboot_base = pmc->base;
	}

		/*
		 * Invert the interrupt polarity if a PMC device tree node
		 * exists and contains the nvidia,invert-interrupt property.
		 */
		invert = of_property_read_bool(np, "nvidia,invert-interrupt");

		value = tegra_pmc_readl(TEGRA_PMC_CNTRL);

		if (invert)
			value |= PMC_CNTRL_INTR_POLARITY;
		else
			value &= ~PMC_CNTRL_INTR_POLARITY;

		tegra_pmc_writel(value, TEGRA_PMC_CNTRL);

		of_node_put(np);
	}

	return 0;
}
early_initcall(tegra_pmc_early_init);

static void pmc_iopower_enable(const struct tegra_pmc_io_pad_soc *pad)
{
	if (pad->io_power == UINT_MAX)
		return;

	tegra_pmc_register_update(TEGRA_PMC_PWR_NO_IOPOWER,
				  BIT(pad->io_power), 0);
}

static void pmc_iopower_disable(const struct tegra_pmc_io_pad_soc *pad)
{
	if (pad->io_power == UINT_MAX)
		return;

	tegra_pmc_register_update(TEGRA_PMC_PWR_NO_IOPOWER, BIT(pad->io_power),
				  BIT(pad->io_power));
}

static int pmc_iopower_get_status(const struct tegra_pmc_io_pad_soc *pad)
{
	unsigned int no_iopower;

	if (pad->io_power == UINT_MAX)
		return 1;

	no_iopower = tegra_pmc_readl(TEGRA_PMC_PWR_NO_IOPOWER);

	return !(no_iopower & BIT(pad->io_power));
}


static int tegra_pmc_io_rail_change_notify_cb(struct notifier_block *nb,
					      unsigned long event, void *v)
{
	struct tegra_io_pad_regulator *tip_reg;
	const struct tegra_pmc_io_pad_soc *pad;
	unsigned long flags;

	if (!(event & (REGULATOR_EVENT_POST_ENABLE |
		       REGULATOR_EVENT_PRE_DISABLE |
		       REGULATOR_EVENT_PRE_ENABLE |
		       REGULATOR_EVENT_DISABLE)))
		return NOTIFY_OK;

	tip_reg = container_of(nb, struct tegra_io_pad_regulator, nb);
	pad = tip_reg->pad;

	spin_lock_irqsave(&pwr_lock, flags);

	if (pad->bdsdmem_cfc) {
		if (event & REGULATOR_EVENT_PRE_ENABLE)
			pmc_iopower_enable(pad);

		if (event & REGULATOR_EVENT_DISABLE)
			pmc_iopower_disable(pad);
	} else {
		if (event & REGULATOR_EVENT_POST_ENABLE)
			pmc_iopower_enable(pad);

		if (event & REGULATOR_EVENT_PRE_DISABLE)
			pmc_iopower_disable(pad);
	}

	dev_dbg(pmc->dev, "tegra-iopower: %s: event 0x%08lx state: %d\n",
		pad->name, event, pmc_iopower_get_status(pad));

	spin_unlock_irqrestore(&pwr_lock, flags);

	return NOTIFY_OK;
}

static int tegra_pmc_io_power_init_one(struct device *dev,
				       const struct tegra_pmc_io_pad_soc *pad,
				       u32 *disabled_mask,
				       bool enable_pad_volt_config)
{
	struct tegra_io_pad_regulator *tip_reg;
	char regname[32]; /* 32 is max size of property name */
	char *prefix;
	int curr_io_uv;
	int ret;

	prefix = "vddio";
	snprintf(regname, 32, "%s-%s-supply", prefix, pad->name);
	if (!of_find_property(dev->of_node, regname, NULL)) {
		prefix = "iopower";
		snprintf(regname, 32, "%s-%s-supply", prefix, pad->name);
		if (!of_find_property(dev->of_node, regname, NULL)) {
			dev_info(dev, "Regulator supply %s not available\n",
				 regname);
			return 0;
		}
	}

	tip_reg = devm_kzalloc(dev, sizeof(*tip_reg), GFP_KERNEL);
	if (!tip_reg)
		return -ENOMEM;

	tip_reg->pad = pad;

	snprintf(regname, 32, "%s-%s", prefix, pad->name);
	tip_reg->regulator = devm_regulator_get(dev, regname);
	if (IS_ERR(tip_reg->regulator)) {
		ret = PTR_ERR(tip_reg->regulator);
		dev_err(dev, "Failed to get regulator %s: %d\n", regname, ret);
		return ret;
	}

	if (!enable_pad_volt_config)
		goto skip_pad_config;

	ret = regulator_get_voltage(tip_reg->regulator);
	if (ret < 0) {
		dev_err(dev, "Failed to get IO rail %s voltage: %d\n",
			regname, ret);
		return ret;
	}

	if (ret == 1200000)
		curr_io_uv = TEGRA_IO_PAD_VOLTAGE_1200000UV;
	else if (ret == 1800000)
		curr_io_uv = TEGRA_IO_PAD_VOLTAGE_1800000UV;
	else
		curr_io_uv = TEGRA_IO_PAD_VOLTAGE_3300000UV;

	ret = _tegra_pmc_io_pad_set_voltage(pad, curr_io_uv);
	if (ret < 0) {
		dev_err(dev, "Failed to set voltage %duV of I/O pad %s: %d\n",
			curr_io_uv, pad->name, ret);
		return ret;
	}

skip_pad_config:
	tip_reg->nb.notifier_call = tegra_pmc_io_rail_change_notify_cb;
	ret = devm_regulator_register_notifier(tip_reg->regulator,
					       &tip_reg->nb);
	if (ret < 0) {
		dev_err(dev, "Failed to register regulator %s notifier: %d\n",
			regname, ret);
		return ret;
	}

	if (regulator_is_enabled(tip_reg->regulator)) {
		pmc_iopower_enable(pad);
	} else {
		*disabled_mask |= BIT(pad->io_power);
		pmc_iopower_disable(pad);
	}

	return 0;
}

static int tegra_pmc_iopower_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	bool enable_pad_volt_config = false;
	u32 pwrio_disabled_mask = 0;
	int i, ret;

	if (!pmc->base) {
		dev_err(dev, "PMC Driver is not ready\n");
		return -EPROBE_DEFER;
	}

	enable_pad_volt_config = of_property_read_bool(dev->of_node,
					"nvidia,auto-pad-voltage-config");

	for (i = 0; i < pmc->soc->num_io_pads; ++i) {
		if (pmc->soc->io_pads[i].io_power == UINT_MAX)
			continue;

		ret = tegra_pmc_io_power_init_one(&pdev->dev,
						  &pmc->soc->io_pads[i],
						  &pwrio_disabled_mask,
						  enable_pad_volt_config);
		if (ret < 0)
			dev_info(dev, "io-power cell %s init failed: %d\n",
				 pmc->soc->io_pads[i].name, ret);
	}

	dev_info(dev, "NO_IOPOWER setting 0x%x\n", pwrio_disabled_mask);
	return 0;
}

static const struct of_device_id tegra_pmc_iopower_match[] = {
	{ .compatible = "nvidia,tegra186-pmc-iopower", },
	{ .compatible = "nvidia,tegra210-pmc-iopower", },
	{ }
};

static struct platform_driver tegra_pmc_iopower_driver = {
	.probe   = tegra_pmc_iopower_probe,
	.driver  = {
		.name  = "tegra-pmc-iopower",
		.owner = THIS_MODULE,
		.of_match_table = tegra_pmc_iopower_match,
	},
};

builtin_platform_driver(tegra_pmc_iopower_driver);
