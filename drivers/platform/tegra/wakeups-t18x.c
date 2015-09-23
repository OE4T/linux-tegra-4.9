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

#include "iomap.h"

#define TEGRA_GPIO_BANK_ID_A 0
#define TEGRA_GPIO_BANK_ID_B 1
#define TEGRA_GPIO_BANK_ID_C 2
#define TEGRA_GPIO_BANK_ID_D 3
#define TEGRA_GPIO_BANK_ID_E 4
#define TEGRA_GPIO_BANK_ID_F 5
#define TEGRA_GPIO_BANK_ID_G 6
#define TEGRA_GPIO_BANK_ID_H 7
#define TEGRA_GPIO_BANK_ID_I 8
#define TEGRA_GPIO_BANK_ID_J 9
#define TEGRA_GPIO_BANK_ID_K 10
#define TEGRA_GPIO_BANK_ID_L 11
#define TEGRA_GPIO_BANK_ID_M 12
#define TEGRA_GPIO_BANK_ID_N 13
#define TEGRA_GPIO_BANK_ID_O 14
#define TEGRA_GPIO_BANK_ID_P 15
#define TEGRA_GPIO_BANK_ID_Q 16
#define TEGRA_GPIO_BANK_ID_R 17
#define TEGRA_GPIO_BANK_ID_S 18
#define TEGRA_GPIO_BANK_ID_T 19
#define TEGRA_GPIO_BANK_ID_U 20
#define TEGRA_GPIO_BANK_ID_V 21
#define TEGRA_GPIO_BANK_ID_W 22
#define TEGRA_GPIO_BANK_ID_X 23
#define TEGRA_GPIO_BANK_ID_Y 24
#define TEGRA_GPIO_BANK_ID_Z 25
#define TEGRA_GPIO_BANK_ID_AA 26
#define TEGRA_GPIO_BANK_ID_BB 27
#define TEGRA_GPIO_BANK_ID_CC 28
#define TEGRA_GPIO_BANK_ID_DD 29
#define TEGRA_GPIO_BANK_ID_EE 30
#define TEGRA_GPIO_BANK_ID_FF 31

#define TEGRA_GPIO(bank, offset) \
	((TEGRA_GPIO_BANK_ID_##bank * 8) + offset)

static int tegra_gpio_wakes[] = {
	TEGRA_GPIO(A, 6),		/* wake0 */
	TEGRA_GPIO(A, 2),		/* wake1 */
	TEGRA_GPIO(A, 5),		/* wake2 */
	TEGRA_GPIO(D, 3),		/* wake3 */
	TEGRA_GPIO(E, 3),		/* wake4 */
	TEGRA_GPIO(G, 3),		/* wake5 */
	-EINVAL,			/* wake6 */
	TEGRA_GPIO(B, 3),		/* wake7 */
	TEGRA_GPIO(B, 5),		/* wake8 */
	TEGRA_GPIO(C, 0),		/* wake9 */
	TEGRA_GPIO(S, 2),		/* wake10 */
	TEGRA_GPIO(H, 2),		/* wake11 */
	TEGRA_GPIO(J, 5),		/* wake12 */
	TEGRA_GPIO(J, 6),		/* wake13 */
	TEGRA_GPIO(J, 7),		/* wake14 */
	TEGRA_GPIO(K, 0),		/* wake15 */
	TEGRA_GPIO(Q, 1),		/* wake16 */
	TEGRA_GPIO(F, 4),		/* wake17 */
	TEGRA_GPIO(M, 5),		/* wake18 */
	TEGRA_GPIO(P, 0),		/* wake19 */
	TEGRA_GPIO(P, 2),		/* wake20 */
	TEGRA_GPIO(P, 1),		/* wake21 */
	TEGRA_GPIO(O, 3),		/* wake22 */
	TEGRA_GPIO(R, 5),		/* wake23 */
	-EINVAL,			/* wake24 */
	TEGRA_GPIO(S, 3),		/* wake25 */
	TEGRA_GPIO(S, 4),		/* wake26 */
	TEGRA_GPIO(S, 1),		/* wake27 */
	TEGRA_GPIO(F, 2),		/* wake28 */
	TEGRA_GPIO(FF, 0),		/* wake29 */
	TEGRA_GPIO(FF, 4),		/* wake30 */
	TEGRA_GPIO(C, 6),		/* wake31 */
	TEGRA_GPIO(W, 2),		/* wake32 */
	TEGRA_GPIO(W, 5),		/* wake33 */
	TEGRA_GPIO(W, 1),		/* wake34 */
	TEGRA_GPIO(V, 0),		/* wake35 */
	TEGRA_GPIO(V, 1),		/* wake36 */
	TEGRA_GPIO(V, 2),		/* wake37 */
	TEGRA_GPIO(V, 3),		/* wake38 */
	TEGRA_GPIO(V, 4),		/* wake39 */
	TEGRA_GPIO(V, 5),		/* wake40 */
	TEGRA_GPIO(EE, 0),		/* wake41 */
	TEGRA_GPIO(Z, 1),		/* wake42 */
	TEGRA_GPIO(Z, 3),		/* wake43 */
	TEGRA_GPIO(AA, 0),		/* wake44 */
	TEGRA_GPIO(AA, 1),		/* wake45 */
	TEGRA_GPIO(AA, 2),		/* wake46 */
	TEGRA_GPIO(AA, 3),		/* wake47 */
	TEGRA_GPIO(AA, 4),		/* wake48 */
	TEGRA_GPIO(AA, 5),		/* wake49 */
	TEGRA_GPIO(AA, 6),		/* wake50 */
	TEGRA_GPIO(AA, 7),		/* wake51 */
	TEGRA_GPIO(X, 3),		/* wake52 */
	TEGRA_GPIO(X, 7),		/* wake53 */
	TEGRA_GPIO(Y, 0),		/* wake54 */
	TEGRA_GPIO(Y, 1),		/* wake55 */
	TEGRA_GPIO(Y, 2),		/* wake56 */
	TEGRA_GPIO(Y, 5),		/* wake57 */
	TEGRA_GPIO(Y, 6),		/* wake58 */
	TEGRA_GPIO(L, 1),		/* wake59 */
	TEGRA_GPIO(L, 3),		/* wake60 */
	TEGRA_GPIO(L, 4),		/* wake61 */
	TEGRA_GPIO(L, 5),		/* wake62 */
	TEGRA_GPIO(I, 4),		/* wake63 */
	TEGRA_GPIO(I, 6),		/* wake64 */
	TEGRA_GPIO(Z, 0),		/* wake65 */
	TEGRA_GPIO(Z, 2),		/* wake66 */
	TEGRA_GPIO(FF, 1),		/* wake67 */
	TEGRA_GPIO(FF, 2),		/* wake68 */
	TEGRA_GPIO(FF, 3),		/* wake69 */
	TEGRA_GPIO(H, 3),		/* wake70 */
	TEGRA_GPIO(P, 5),		/* wake71 */
	-EINVAL,		/* wake72 */
	-EINVAL,		/* wake73 */
	-EINVAL,		/* wake74 */
	-EINVAL,		/* wake75 */
	-EINVAL,		/* wake76 */
	-EINVAL,		/* wake77 */
	-EINVAL,		/* wake78 */
	-EINVAL,		/* wake79 */
	-EINVAL,		/* wake80 */
	-EINVAL,		/* wake81 */
	-EINVAL,		/* wake82 */
	-EINVAL,		/* wake83 */
	-EINVAL,		/* wake84 */
	-EINVAL,		/* wake85 */
	-EINVAL,		/* wake86 */
	-EINVAL,		/* wake87 */
	-EINVAL,		/* wake88 */
	-EINVAL,		/* wake89 */
	-EINVAL,		/* wake90 */
	-EINVAL,		/* wake91 */
	-EINVAL,		/* wake92 */
	-EINVAL,		/* wake93 */
	-EINVAL,		/* wake94 */
	-EINVAL,		/* wake95 */
};

static int tegra_wake_event_irq[] = {
	-EINVAL,		/* wake0 */
	-EINVAL,		/* wake1 */
	-EINVAL,		/* wake2 */
	-EINVAL,		/* wake3 */
	-EINVAL,		/* wake4 */
	-EINVAL,		/* wake5 */
	-EINVAL,		/* wake6 */
	-EINVAL,		/* wake7 */
	-EINVAL,		/* wake8 */
	-EINVAL,		/* wake9 */
	-EINVAL,		/* wake10 */
	-EINVAL,		/* wake11 */
	-EINVAL,		/* wake12 */
	-EINVAL,		/* wake13 */
	-EINVAL,		/* wake14 */
	-EINVAL,		/* wake15 */
	-EINVAL,		/* wake16 */
	-EINVAL,		/* wake17 */
	-EINVAL,		/* wake18 */
	-EINVAL,		/* wake19 */
	-EINVAL,		/* wake20 */
	-EINVAL,		/* wake21 */
	-EINVAL,		/* wake22 */
	-EINVAL,		/* wake23 */
	-EINVAL,		/* wake24 */
	-EINVAL,		/* wake25 */
	-EINVAL,		/* wake26 */
	-EINVAL,		/* wake27 */
	-EINVAL,		/* wake28 */
	-EINVAL,		/* wake29 */
	-EINVAL,		/* wake30 */
	-EINVAL,		/* wake31 */
	-EINVAL,		/* wake32 */
	-EINVAL,		/* wake33 */
	-EINVAL,		/* wake34 */
	-EINVAL,		/* wake35 */
	-EINVAL,		/* wake36 */
	-EINVAL,		/* wake37 */
	-EINVAL,		/* wake38 */
	-EINVAL,		/* wake39 */
	-EINVAL,		/* wake40 */
	-EINVAL,		/* wake41 */
	-EINVAL,		/* wake42 */
	-EINVAL,		/* wake43 */
	-EINVAL,		/* wake44 */
	-EINVAL,		/* wake45 */
	-EINVAL,		/* wake46 */
	-EINVAL,		/* wake47 */
	-EINVAL,		/* wake48 */
	-EINVAL,		/* wake49 */
	-EINVAL,		/* wake50 */
	-EINVAL,		/* wake51 */
	-EINVAL,		/* wake52 */
	-EINVAL,		/* wake53 */
	-EINVAL,		/* wake54 */
	-EINVAL,		/* wake55 */
	-EINVAL,		/* wake56 */
	-EINVAL,		/* wake57 */
	-EINVAL,		/* wake58 */
	-EINVAL,		/* wake59 */
	-EINVAL,		/* wake60 */
	-EINVAL,		/* wake61 */
	-EINVAL,		/* wake62 */
	-EINVAL,		/* wake63 */
	-EINVAL,		/* wake64 */
	-EINVAL,		/* wake65 */
	-EINVAL,		/* wake66 */
	-EINVAL,		/* wake67 */
	-EINVAL,		/* wake68 */
	-EINVAL,		/* wake69 */
	-EINVAL,		/* wake70 */
	-EINVAL,		/* wake71 */
	-EINVAL,		/* wake72 */
	-EINVAL,		/* wake73 */
	-EINVAL,		/* wake74 */
	-EINVAL,		/* wake75 */
	-EINVAL,		/* wake76 */
	-EINVAL,		/* wake77 */
	-EINVAL,		/* wake78 */
	-EINVAL,		/* wake79 */
	-EINVAL,		/* wake80 */
	-EINVAL,		/* wake81 */
	-EINVAL,		/* wake82 */
	-EINVAL,		/* wake83 */
	-EINVAL,		/* wake84 */
	-EINVAL,		/* wake85 */
	-EINVAL,		/* wake86 */
	-EINVAL,		/* wake87 */
	-EINVAL,		/* wake88 */
	-EINVAL,		/* wake89 */
	-EINVAL,		/* wake90 */
	-EINVAL,		/* wake91 */
	-EINVAL,		/* wake92 */
	-EINVAL,		/* wake93 */
	-EINVAL,		/* wake94 */
	-EINVAL,		/* wake95 */
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
