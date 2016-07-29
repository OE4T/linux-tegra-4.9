/*
 * Copyright (C) 2014-2017 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SOC_TEGRA_COMMON_H__
#define __SOC_TEGRA_COMMON_H__

#ifdef CONFIG_ARCH_TEGRA
bool soc_is_tegra210_n_before(void);
bool soc_is_tegra186_n_later(void);
#else
static inline bool soc_is_tegra210_n_before(void)
{
	return false;
}
static inline bool soc_is_tegra186_n_later(void)
{
	return false;
}
#endif

static inline bool soc_is_tegra(void)
{
	return soc_is_tegra210_n_before() || soc_is_tegra186_n_later();
}

int tegra_get_usb_port_owner_info(void);

#endif /* __SOC_TEGRA_COMMON_H__ */
