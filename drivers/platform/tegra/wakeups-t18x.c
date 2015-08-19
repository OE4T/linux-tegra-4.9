/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irqchip/tegra.h>

#include <mach/irqs.h>

#include "gpio-names.h"
#include "iomap.h"

static int tegra_gpio_wakes[] = {
	TEGRA_WAKE_GPIO_PA6,    /* wake 0 */
	TEGRA_WAKE_GPIO_PA2,    /* wake 1 */
	TEGRA_WAKE_GPIO_PS2,    /* wake 2 */
	TEGRA_WAKE_GPIO_PH2,    /* wake 3 */
	TEGRA_WAKE_GPIO_PJ5,    /* wake 4 */
	TEGRA_WAKE_GPIO_PJ6,    /* wake 5 */
	TEGRA_WAKE_GPIO_PJ7,    /* wake 6 */
	TEGRA_WAKE_GPIO_PK0,    /* wake 7 */
	TEGRA_WAKE_GPIO_PQ1,    /* wake 8 */
	TEGRA_WAKE_GPIO_PF4,    /* wake 9 */
	TEGRA_WAKE_GPIO_PM5,    /* wake 10 */
	TEGRA_WAKE_GPIO_PP0,    /* wake 11 */
	TEGRA_WAKE_GPIO_PA5,    /* wake 12 */
	TEGRA_WAKE_GPIO_PP2,    /* wake 13 */
	TEGRA_WAKE_GPIO_PP1,    /* wake 14 */
	TEGRA_WAKE_GPIO_PO3,    /* wake 15 */
	TEGRA_WAKE_GPIO_PR5,    /* wake 16 */
	-EINVAL,		/*Need to revisit 17 */
	TEGRA_WAKE_GPIO_PS3,    /* wake 18 */
	TEGRA_WAKE_GPIO_PS4,    /* wake 19 */
	TEGRA_WAKE_GPIO_PS1,    /* wake 20 */
	TEGRA_WAKE_GPIO_PF2,    /* wake 21 */
	TEGRA_WAKE_GPIO_PFF0,    /* wake 22 */
	TEGRA_WAKE_GPIO_PD3,    /* wake 23 */
	TEGRA_WAKE_GPIO_PFF4,    /* wake 24 */
	TEGRA_WAKE_GPIO_PC6,    /* wake 25 */
	TEGRA_WAKE_GPIO_PW2,    /* wake 26 */
	TEGRA_WAKE_GPIO_PW5,    /* wake 27 */
	TEGRA_WAKE_GPIO_PW1,    /* wake 28 */
	TEGRA_WAKE_GPIO_PV0,    /* wake 29 */
	TEGRA_WAKE_GPIO_PV1,    /* wake 30 */
	TEGRA_WAKE_GPIO_PV2,    /* wake 31 */
	TEGRA_WAKE_GPIO_PV3,    /* wake 32 */
	TEGRA_WAKE_GPIO_PV4,    /* wake 33 */
	TEGRA_WAKE_GPIO_PE3,    /* wake 34 */
	TEGRA_WAKE_GPIO_PV5,    /* wake 35 */
	TEGRA_WAKE_GPIO_PEE0,    /* wake 36 */
	TEGRA_WAKE_GPIO_PZ1,    /* wake 37 */
	TEGRA_WAKE_GPIO_PZ3,    /* wake 38 */
	TEGRA_WAKE_GPIO_PAA0,    /* wake 39 */
	TEGRA_WAKE_GPIO_PAA1,    /* wake 40 */
	TEGRA_WAKE_GPIO_PAA2,    /* wake 41 */
	TEGRA_WAKE_GPIO_PAA3,    /* wake 42 */
	TEGRA_WAKE_GPIO_PAA4,    /* wake 43 */
	TEGRA_WAKE_GPIO_PAA5,    /* wake 44 */
	TEGRA_WAKE_GPIO_PG3,    /* wake 45 */
	TEGRA_WAKE_GPIO_PAA6,    /* wake 46 */
	TEGRA_WAKE_GPIO_PAA7,    /* wake 47 */
	TEGRA_WAKE_GPIO_PX3,    /* wake 48 */
	TEGRA_WAKE_GPIO_PX7,    /* wake 49 */
	TEGRA_WAKE_GPIO_PY0,    /* wake 50 */
	TEGRA_WAKE_GPIO_PY1,    /* wake 51 */
	TEGRA_WAKE_GPIO_PY2,    /* wake 52 */
	TEGRA_WAKE_GPIO_PY5,    /* wake 53 */
	TEGRA_WAKE_GPIO_PY6,    /* wake 54 */
	TEGRA_WAKE_GPIO_PL1,    /* wake 55 */
	-EINVAL,		/*Need to revisit 56 */
	TEGRA_WAKE_GPIO_PL3,    /* wake 57 */
	TEGRA_WAKE_GPIO_PL4,    /* wake 58 */
	TEGRA_WAKE_GPIO_PL5,    /* wake 59 */
	TEGRA_WAKE_GPIO_PI4,    /* wake 60 */
	TEGRA_WAKE_GPIO_PI6,    /* wake 61 */
	TEGRA_WAKE_GPIO_PZ0,   /* wake 62 */
	TEGRA_WAKE_GPIO_PZ2,    /* wake 63 */
	TEGRA_WAKE_GPIO_PFF1,    /* wake 64 */
	TEGRA_WAKE_GPIO_PFF2,    /* wake 65 */
	TEGRA_WAKE_GPIO_PFF3,    /* wake 66 */
	TEGRA_WAKE_GPIO_PB3,    /* wake 67 */
	TEGRA_WAKE_GPIO_PH3,    /* wake 68 */
	TEGRA_WAKE_GPIO_PP5,    /* wake 69 */
	TEGRA_WAKE_GPIO_PB5,    /* wake 70 */
	TEGRA_WAKE_GPIO_PC0,    /* wake 71 */
	-EINVAL,				/* wake72 */
	-EINVAL,				/* wake73 */
	-EINVAL,				/* wake74 */
	-EINVAL,				/* wake75 */
	-EINVAL,				/* wake76 */
	-EINVAL,				/* wake77 */
	-EINVAL,				/* wake78 */
	-EINVAL,				/* wake79 */
	-EINVAL,				/* wake80 */
	-EINVAL,				/* wake81 */
	-EINVAL,				/* wake82 */
	-EINVAL,				/* wake83 */
	-EINVAL,				/* wake84 */
	-EINVAL,				/* wake85 */
	-EINVAL,				/* wake86 */
	-EINVAL,				/* wake87 */
	-EINVAL,				/* wake88 */
	-EINVAL,				/* wake89 */
	-EINVAL,				/* wake90 */
	-EINVAL,				/* wake91 */
	-EINVAL,				/* wake92 */
	-EINVAL,				/* wake93 */
	-EINVAL,				/* wake94 */
	-EINVAL,				/* wake95 */
};

static int tegra_wake_event_irq[] = {
	-EINVAL,				/* wake0 */
	-EINVAL,				/* wake1 */
	-EINVAL,				/* wake2 */
	-EINVAL,				/* wake3 */
	-EINVAL,				/* wake4 */
	-EINVAL,				/* wake5 */
	-EINVAL,				/* wake6 */
	-EINVAL,				/* wake7 */
	-EINVAL,				/* wake8 */
	-EINVAL,				/* wake9 */
	-EINVAL,				/* wake10 */
	-EINVAL,				/* wake11 */
	-EINVAL,				/* wake12 */
	-EINVAL,				/* wake13 */
	-EINVAL,				/* wake14 */
	-EINVAL,				/* wake15 */
	-EINVAL,				/* wake16 */
	-EINVAL, 				/* wake17 */
	-EINVAL,				/* wake18 */
	-EINVAL,				/* wake19 */
	-EINVAL,				/* wake20 */
	-EINVAL,				/* wake21 */
	-EINVAL,				/* wake22 */
	-EINVAL,				/* wake23 */
	-EINVAL,				/* wake24 */
	-EINVAL,				/* wake25 */
	-EINVAL,				/* wake26 */
	-EINVAL,				/* wake27 */
	-EINVAL,				/* wake28 */
	-EINVAL,				/* wake29 */
	-EINVAL,				/* wake30 */
	-EINVAL,				/* wake31 */
	-EINVAL,				/* wake32 */
	-EINVAL,				/* wake33 */
	-EINVAL,				/* wake34 */
	-EINVAL,				/* wake35 */
	-EINVAL,				/* wake36 */
	-EINVAL,				/* wake37 */
	-EINVAL,				/* wake38 */
	-EINVAL,				/* wake39 */
	-EINVAL,				/* wake40 */
	-EINVAL,				/* wake41 */
	-EINVAL,				/* wake42 */
	-EINVAL,				/* wake43 */
	-EINVAL,				/* wake44 */
	-EINVAL,				/* wake45 */
	-EINVAL,				/* wake46 */
	-EINVAL,				/* wake47 */
	-EINVAL,				/* wake48 */
	-EINVAL,				/* wake49 */
	-EINVAL,				/* wake50 */
	-EINVAL,				/* wake51 */
	-EINVAL,				/* wake52 */
	-EINVAL,				/* wake53 */
	-EINVAL,				/* wake54 */
	-EINVAL,				/* wake55 */
	-EINVAL,				/* wake56 */
	-EINVAL,				/* wake57 */
	-EINVAL,				/* wake58 */
	-EINVAL,				/* wake59 */
	-EINVAL,				/* wake60 */
	-EINVAL,				/* wake61 */
	-EINVAL,				/* wake62 */
	-EINVAL,				/* wake63 */
	-EINVAL,				/* wake64 */
	-EINVAL,				/* wake65 */
	-EINVAL,				/* wake66 */
	-EINVAL,				/* wake67 */
	-EINVAL,				/* wake68 */
	-EINVAL,				/* wake69 */
	-EINVAL,				/* wake70 */
	-EINVAL,				/* wake71 */
	-EINVAL,				/* wake72 */
	-EINVAL,				/* wake73 */
	-EINVAL,				/* wake74 */
	-EINVAL,				/* wake75 */
	-EINVAL,				/* wake76 */
	-EINVAL,				/* wake77 */
	-EINVAL,				/* wake78 */
	-EINVAL,				/* wake79 */
	-EINVAL,				/* wake80 */
	-EINVAL,				/* wake81 */
	-EINVAL,				/* wake82 */
	-EINVAL,				/* wake83 */
	-EINVAL,				/* wake84 */
	-EINVAL,				/* wake85 */
	-EINVAL,				/* wake86 */
	-EINVAL,				/* wake87 */
	-EINVAL,				/* wake88 */
	-EINVAL,				/* wake89 */
	-EINVAL,				/* wake90 */
	-EINVAL,				/* wake91 */
	-EINVAL,				/* wake92 */
	-EINVAL,				/* wake93 */
	-EINVAL,				/* wake94 */
	-EINVAL,				/* wake95 */
};

static int __init tegra18x_wakeup_table_init(void)
{
	tegra_gpio_wake_table = tegra_gpio_wakes;
	tegra_irq_wake_table = tegra_wake_event_irq;
	tegra_wake_table_len = ARRAY_SIZE(tegra_gpio_wakes);
	return 0;
}

int __init tegra_wakeup_table_init(void)
{
	return tegra18x_wakeup_table_init();
}
