/*
 * GK20A ECC
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#ifndef ECC_GK20A_H
#define ECC_GK20A_H

#include <uapi/linux/nvgpu.h>

struct gk20a_ecc_stat {
	char **names;
	u32 *counters;
	struct hlist_node hash_node;
};

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "ecc_t18x.h"
#endif
#ifdef CONFIG_TEGRA_19x_GPU
#include "ecc_t19x.h"
#endif

struct ecc_gk20a {
	/* Stats per engine */
	struct {
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
		struct ecc_gr_t18x t18x;
#endif
#ifdef CONFIG_TEGRA_19x_GPU
		struct ecc_gr_t19x t19x;
#endif
	} gr;

	struct {
#ifdef CONFIG_TEGRA_19x_GPU
		struct ecc_ltc_t19x t19x;
#endif
	} ltc;

	struct {
#ifdef CONFIG_TEGRA_19x_GPU
		struct ecc_eng_t19x t19x;
#endif
	} eng;

};

#endif /*__ECC_GK20A_H__*/
