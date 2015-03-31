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
	-EINVAL,				/* wake17 */
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
