/*
 * drivers/video/tegra/host/gk20a/gk20a.h
 *
 * GK20A Graphics
 *
 * Copyright (c) 2011, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef _NVHOST_GK20A_H_
#define _NVHOST_GK20A_H_


#if defined(CONFIG_ARCH_TEGRA_12x_SOC) && defined(CONFIG_TEGRA_SIMULATION_PLATFORM)
#define CONFIG_GK20A_SIM 1
#else
#define CONFIG_GK20A_SIM 0
#endif

struct gk20a;
struct fifo_gk20a;
struct channel_gk20a;
struct gr_gk20a;
struct sim_gk20a;

#include <mach/hardware.h>
#include "clk_gk20a.h"
#include "fifo_gk20a.h"
#include "gr_gk20a.h"
#include "sim_gk20a.h"
#include "intr_gk20a.h"
#include "pmu_gk20a.h"
#include "priv_ring_gk20a.h"
#include "therm_gk20a.h"


extern struct platform_device tegra_gk20a_device;

struct gk20a {
	struct nvhost_master *host;
	struct platform_device *dev;

	struct resource *reg_mem;
	void __iomem *regs;

	struct resource *bar1_mem;
	void __iomem *bar1;

	bool power_on;
	bool irq_requested;

	struct clk_gk20a clk;
	struct fifo_gk20a fifo;
	struct gr_gk20a gr;
	struct sim_gk20a sim;
	struct mm_gk20a mm;
	struct pmu_gk20a pmu;

	void (*remove_support)(struct platform_device *);
};

static inline struct gk20a *get_gk20a(struct platform_device *dev)
{
	return (struct gk20a *)nvhost_get_private_data(dev);
}

extern const struct nvhost_as_moduleops gk20a_as_moduleops;

/* register accessors */
static inline void gk20a_writel(struct gk20a *g, u32 r, u32 v)
{
	nvhost_dbg(dbg_reg, " r=0x%x v=0x%x", r, v);
	writel(v, g->regs + r);
}
static inline u32 gk20a_readl(struct gk20a *g, u32 r)
{
	u32 v = readl(g->regs + r);
	nvhost_dbg(dbg_reg, " r=0x%x v=0x%x", r, v);
	return v;
}

static inline void gk20a_bar1_writel(struct gk20a *g, u32 b, u32 v)
{
	nvhost_dbg(dbg_reg, " b=0x%x v=0x%x", b, v);
	writel(v, g->bar1 + b);
}

static inline u32 gk20a_bar1_readl(struct gk20a *g, u32 b)
{
	u32 v = readl(g->bar1 + b);
	nvhost_dbg(dbg_reg, " b=0x%x v=0x%x", b, v);
	return v;
}

/* convenience */
static inline struct device *dev_from_gk20a(struct gk20a *g)
{
	return &g->dev->dev;
}
static inline struct nvhost_syncpt *syncpt_from_gk20a(struct gk20a* g)
{
	return &(nvhost_get_host(g->dev)->syncpt);
}
static inline struct mem_mgr *mem_mgr_from_g(struct gk20a* g)
{
	return nvhost_get_host(g->dev)->memmgr;
}

static inline u32 u64_hi32(u64 n)
{
	return (u32)((n >> 32) & ~(u32)0);
}

static inline u32 u64_lo32(u64 n)
{
	return (u32)(n & ~(u32)0);
}

static inline u32 set_field(u32 val, u32 mask, u32 field)
{
	return ((val & ~mask) | field);
}

/* invalidate channel lookup tlb */
static inline void gk20a_gr_flush_channel_tlb(struct gr_gk20a *gr)
{
	memset(gr->chid_tlb, 0,
		sizeof(struct gr_channel_map_tlb_entry) *
		GR_CHANNEL_MAP_TLB_SIZE);
}

/* This function can be called from two places, whichever comes first.
 * 1. nvhost calls this for gk20a driver init when client opens first gk20a channel.
 * 2. client opens gk20a ctrl node.
 */
void nvhost_gk20a_init(struct platform_device *dev);

/* classes that the device supports */
/* TBD: get these from an open-sourced SDK? */
enum {
	KEPLER_C                  = 0xA297,
	FERMI_TWOD_A              = 0x902D,
	KEPLER_COMPUTE_A          = 0xA0C0,
	KEPLER_INLINE_TO_MEMORY_A = 0xA040,
	KEPLER_DMA_COPY_A         = 0xA0B5, /*not sure about this one*/
};

/* TBD: these should come from tegra iomap.h &&|| be in the device resources */
#define TEGRA_GK20A_BAR0_BASE  0x57000000
#define TEGRA_GK20A_BAR0_SIZE  0x01000000
#define TEGRA_GK20A_BAR1_BASE  0x58000000
#define TEGRA_GK20A_BAR1_SIZE  0x01000000

#if defined (CONFIG_TEGRA_GK20A_PMU)
static inline int support_gk20a_pmu(void)
{
	return tegra_platform_is_qt() ? 0 : 1;
}
#else
static inline int support_gk20a_pmu(void){return 0;}
#endif


#endif /* _NVHOST_GK20A_H_ */
