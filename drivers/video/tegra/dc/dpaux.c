/*
 * drivers/video/tegra/dc/dpaux.c
 *
 * Copyright (c) 2014 - 2015, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
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

#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/err.h>

#include "dpaux_regs.h"
#include "dc_priv.h"
#include "dpaux.h"
#include "../../../../arch/arm/mach-tegra/iomap.h"

static const char *const dpaux_clks[TEGRA_DPAUX_INSTANCE_N] = {
	 "dpaux",
	 "dpaux1",
};

#if !defined(CONFIG_TEGRA_NVDISPLAY)
static unsigned long dpaux_base_addr[TEGRA_DPAUX_INSTANCE_N] = {
	TEGRA_DPAUX_BASE,
	TEGRA_DPAUX1_BASE,
};
#endif

static void __iomem *dpaux_baseaddr[TEGRA_DPAUX_INSTANCE_N];

static DEFINE_MUTEX(dpaux_lock);

static inline struct clk *tegra_dpaux_clk_get(enum tegra_dpaux_instance id)
{
	if (id >= TEGRA_DPAUX_INSTANCE_N)
		return ERR_PTR(-EINVAL);

	return clk_get_sys(dpaux_clks[id], NULL);
}

int tegra_dpaux_clk_en(enum tegra_dpaux_instance id)
{
	return clk_prepare_enable(tegra_dpaux_clk_get(id));
}

void tegra_dpaux_clk_dis(enum tegra_dpaux_instance id)
{
	clk_disable_unprepare(tegra_dpaux_clk_get(id));
}

static inline void _tegra_dpaux_pad_power(struct tegra_dc *dc,
					enum tegra_dpaux_instance id, bool on)
{
	void __iomem *regaddr;
#if !defined(CONFIG_TEGRA_NVDISPLAY)
	regaddr = IO_ADDRESS(dpaux_base_addr[id] + DPAUX_HYBRID_PADCTL * 4);
#else
	regaddr = dpaux_baseaddr[id] + DPAUX_HYBRID_PADCTL * 4;
#endif
	writel((on ? DPAUX_HYBRID_SPARE_PAD_PWR_POWERUP :
		DPAUX_HYBRID_SPARE_PAD_PWR_POWERDOWN),
		regaddr);
}

__maybe_unused
void tegra_dpaux_pad_power(struct tegra_dc *dc,
				enum tegra_dpaux_instance id, bool on)
{
	if (!tegra_platform_is_linsim())
		tegra_dpaux_clk_en(id);

	tegra_dc_io_start(dc);

	mutex_lock(&dpaux_lock);
	_tegra_dpaux_pad_power(dc, id, on);
	mutex_unlock(&dpaux_lock);

	tegra_dc_io_end(dc);
	if (!tegra_platform_is_linsim())
		tegra_dpaux_clk_dis(id);
}

static inline void _tegra_dpaux_config_pad_mode(struct tegra_dc *dc,
					enum tegra_dpaux_instance id,
					enum tegra_dpaux_pad_mode mode)
{
	u32 val;
	void __iomem *regaddr;
#if !defined(CONFIG_TEGRA_NVDISPLAY)
	regaddr = IO_ADDRESS(dpaux_base_addr[id] + DPAUX_HYBRID_PADCTL * 4);
#else
	regaddr = dpaux_baseaddr[id] + DPAUX_HYBRID_PADCTL * 4;
#endif

	val = readl(regaddr);
	val &= ~(DPAUX_HYBRID_PADCTL_I2C_SDA_INPUT_RCV_ENABLE |
		DPAUX_HYBRID_PADCTL_I2C_SCL_INPUT_RCV_ENABLE |
		DPAUX_HYBRID_PADCTL_MODE_I2C);
	val |= mode ? (DPAUX_HYBRID_PADCTL_I2C_SDA_INPUT_RCV_ENABLE |
		DPAUX_HYBRID_PADCTL_I2C_SCL_INPUT_RCV_ENABLE |
		mode) : 0;
	writel(val, regaddr);
}

__maybe_unused
void tegra_dpaux_config_pad_mode(struct tegra_dc *dc,
					enum tegra_dpaux_instance id,
					enum tegra_dpaux_pad_mode mode)
{
	tegra_dpaux_clk_en(id);
	tegra_dc_io_start(dc);

	mutex_lock(&dpaux_lock);
	_tegra_dpaux_pad_power(dc, id, true);
	_tegra_dpaux_config_pad_mode(dc, id, mode);
	mutex_unlock(&dpaux_lock);

	tegra_dc_io_end(dc);
	tegra_dpaux_clk_dis(id);
}

__maybe_unused
void tegra_set_dpaux_addr(void __iomem *dpaux_base,
			enum tegra_dpaux_instance id)
{
	dpaux_baseaddr[id] = dpaux_base;
}
