/*
 * drivers/video/tegra/host/gk20a/kind_gk20a.h
 *
 * GK20A memory kind management
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __KIND_GK20A_H__
#define __KIND_GK20A_H__

#include <nvgpu/bitops.h>

void gk20a_init_uncompressed_kind_map(void);
void gk20a_init_kind_attr(void);

extern u16 gk20a_kind_attr[];
#define NV_KIND_ATTR_SIZE		256
#define NV_KIND_DEFAULT		     -1

#define GK20A_KIND_ATTR_SUPPORTED    BIT(0)
#define GK20A_KIND_ATTR_COMPRESSIBLE BIT(1)
#define GK20A_KIND_ATTR_Z            BIT(2)
#define GK20A_KIND_ATTR_C            BIT(3)
#define GK20A_KIND_ATTR_ZBC          BIT(4)

/* TBD: not sure on the work creation for gk20a, doubtful */
static inline bool gk20a_kind_work_creation_sked(u8 k)
{
	return false;
}
static inline bool gk20a_kind_work_creation_host(u8 k)
{
	return false;
}

static inline bool gk20a_kind_work_creation(u8 k)
{
	return gk20a_kind_work_creation_sked(k) ||
		gk20a_kind_work_creation_host(k);
}

static inline bool gk20a_kind_is_supported(u8 k)
{
	return !!(gk20a_kind_attr[k] & GK20A_KIND_ATTR_SUPPORTED);
}
static inline bool gk20a_kind_is_compressible(u8 k)
{
	return !!(gk20a_kind_attr[k] & GK20A_KIND_ATTR_COMPRESSIBLE);
}

static inline bool gk20a_kind_is_z(u8 k)
{
	return !!(gk20a_kind_attr[k] & GK20A_KIND_ATTR_Z);
}

static inline bool gk20a_kind_is_c(u8 k)
{
	return !!(gk20a_kind_attr[k] & GK20A_KIND_ATTR_C);
}
static inline bool gk20a_kind_is_zbc(u8 k)
{
	return !!(gk20a_kind_attr[k] & GK20A_KIND_ATTR_ZBC);
}

/* maps kind to its uncompressed version */
extern u8 gk20a_uc_kind_map[];
static inline u8 gk20a_get_uncompressed_kind(u8 k)
{
	return gk20a_uc_kind_map[k];
}

#endif /* __KIND_GK20A_H__ */
