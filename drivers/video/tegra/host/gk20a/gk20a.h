/*
 * drivers/video/tegra/host/gk20a/gk20a.h
 *
 * GK20A Graphics
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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


struct gk20a;
struct fifo_gk20a;
struct channel_gk20a;
struct gr_gk20a;
struct sim_gk20a;

#include "dev.h"

#include <linux/tegra-soc.h>
#include <linux/spinlock.h>
#include <linux/nvhost_gpu_ioctl.h>
#include "as_gk20a.h"
#include "clk_gk20a.h"
#include "fifo_gk20a.h"
#include "gr_gk20a.h"
#include "sim_gk20a.h"
#include "pmu_gk20a.h"
#include "priv_ring_gk20a.h"
#include "therm_gk20a.h"
#include "platform_gk20a.h"

#include "../../../../../arch/arm/mach-tegra/iomap.h"

extern struct platform_device tegra_gk20a_device;

bool is_gk20a_module(struct platform_device *dev);

struct cooling_device_gk20a {
	struct thermal_cooling_device *gk20a_cooling_dev;
	unsigned int gk20a_freq_state;
	unsigned int gk20a_freq_table_size;
	struct gk20a *g;
};

struct gpu_ops {
	struct {
		int (*determine_L2_size_bytes)(struct gk20a *gk20a);
		void (*set_max_ways_evict_last)(struct gk20a *g, u32 max_ways);
		int (*init_comptags)(struct gk20a *g, struct gr_gk20a *gr);
		int (*clear_comptags)(struct gk20a *g, u32 min, u32 max);
		void (*set_zbc_color_entry)(struct gk20a *g,
					    struct zbc_entry *color_val,
					    u32 index);
		void (*set_zbc_depth_entry)(struct gk20a *g,
					    struct zbc_entry *depth_val,
					    u32 index);
		void (*clear_zbc_color_entry)(struct gk20a *g, u32 index);
		void (*clear_zbc_depth_entry)(struct gk20a *g, u32 index);
		int  (*init_zbc)(struct gk20a *g, struct gr_gk20a *gr);
		void (*init_cbc)(struct gk20a *g, struct gr_gk20a *gr);
		void (*sync_debugfs)(struct gk20a *g);
		void (*elpg_flush)(struct gk20a *g);
	} ltc;
	struct {
		int (*init_fs_state)(struct gk20a *g);
		void (*access_smpc_reg)(struct gk20a *g, u32 quad, u32 offset);
		void (*bundle_cb_defaults)(struct gk20a *g);
		void (*cb_size_default)(struct gk20a *g);
		int (*calc_global_ctx_buffer_size)(struct gk20a *g);
		void (*commit_global_attrib_cb)(struct gk20a *g,
						struct channel_ctx_gk20a *ch_ctx,
						u64 addr, bool patch);
		void (*commit_global_bundle_cb)(struct gk20a *g,
						struct channel_ctx_gk20a *ch_ctx,
						u64 addr, u64 size, bool patch);
		int (*commit_global_cb_manager)(struct gk20a *g,
						struct channel_gk20a *ch,
						bool patch);
		void (*commit_global_pagepool)(struct gk20a *g,
					       struct channel_ctx_gk20a *ch_ctx,
					       u64 addr, u32 size, bool patch);
		void (*init_gpc_mmu)(struct gk20a *g);
		int (*handle_sw_method)(struct gk20a *g, u32 addr,
					 u32 class_num, u32 offset, u32 data);
		void (*set_alpha_circular_buffer_size)(struct gk20a *g,
					               u32 data);
		void (*set_circular_buffer_size)(struct gk20a *g, u32 data);
		void (*enable_hww_exceptions)(struct gk20a *g);
		bool (*is_valid_class)(struct gk20a *g, u32 class_num);
		void (*get_sm_dsm_perf_regs)(struct gk20a *g,
						  u32 *num_sm_dsm_perf_regs,
						  u32 **sm_dsm_perf_regs,
						  u32 *perf_register_stride);
		void (*get_sm_dsm_perf_ctrl_regs)(struct gk20a *g,
						  u32 *num_sm_dsm_perf_regs,
						  u32 **sm_dsm_perf_regs,
						  u32 *perf_register_stride);
		void (*set_hww_esr_report_mask)(struct gk20a *g);
		int (*setup_alpha_beta_tables)(struct gk20a *g,
					      struct gr_gk20a *gr);
	} gr;
	const char *name;
	struct {
		void (*init_fs_state)(struct gk20a *g);
		void (*reset)(struct gk20a *g);
		void (*init_uncompressed_kind_map)(struct gk20a *g);
		void (*init_kind_attr)(struct gk20a *g);
	} fb;
};

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
	struct cooling_device_gk20a gk20a_cdev;

	/* Save pmu fw here so that it lives cross suspend/resume.
	   pmu suspend destroys all pmu sw/hw states. Loading pmu
	   fw in resume crashes when the resume is from sys_exit. */
	const struct firmware *pmu_fw;

	u32 gr_idle_timeout_default;
	u32 timeouts_enabled;

	bool slcg_enabled;
	bool blcg_enabled;
	bool elcg_enabled;
	bool elpg_enabled;
	bool aelpg_enabled;

#ifdef CONFIG_DEBUG_FS
	spinlock_t debugfs_lock;
	struct dentry *debugfs_ltc_enabled;
	struct dentry *debugfs_timeouts_enabled;
	struct dentry *debugfs_gr_idle_timeout_default;
#endif
	struct gk20a_ctxsw_ucode_info ctxsw_ucode_info;

	/* held while manipulating # of debug/profiler sessions present */
	/* also prevents debug sessions from attaching until released */
	struct mutex dbg_sessions_lock;
	int dbg_sessions; /* number attached */
	int dbg_powergating_disabled_refcount; /*refcount for pg disable */

	void (*remove_support)(struct platform_device *);

	u64 pg_ingating_time_us;
	u64 pg_ungating_time_us;
	u32 pg_gating_cnt;

	spinlock_t mc_enable_lock;

	struct nvhost_gpu_characteristics gpu_characteristics;

	struct {
		struct cdev cdev;
		struct device *node;
	} channel;

	struct gk20a_as as;

	struct {
		struct cdev cdev;
		struct device *node;
	} ctrl;

	struct {
		struct cdev cdev;
		struct device *node;
	} dbg;

	struct {
		struct cdev cdev;
		struct device *node;
	} prof;

	struct mutex client_lock;
	int client_refcount; /* open channels and ctrl nodes */

	dev_t cdev_region;
	struct class *class;

	struct gpu_ops ops;

	int irq_stall;
	int irq_nonstall;

	struct generic_pm_domain pd;

	struct devfreq *devfreq;

	struct gk20a_scale_profile *scale_profile;

	struct device_dma_parameters dma_parms;
};

static inline unsigned long gk20a_get_gr_idle_timeout(struct gk20a *g)
{
	return g->timeouts_enabled ?
		g->gr_idle_timeout_default : MAX_SCHEDULE_TIMEOUT;
}

static inline struct gk20a *get_gk20a(struct platform_device *dev)
{
	return gk20a_get_platform(dev)->g;
}

enum BAR0_DEBUG_OPERATION {
	BARO_ZERO_NOP = 0,
	OP_END = 'DONE',
	BAR0_READ32 = '0R32',
	BAR0_WRITE32 = '0W32',
};

struct share_buffer_head {
	enum BAR0_DEBUG_OPERATION operation;
/* size of the operation item */
	u32 size;
	u32 completed;
	u32 failed;
	u64 context;
	u64 completion_callback;
};

struct gk20a_cyclestate_buffer_elem {
	struct share_buffer_head	head;
/* in */
	u64 p_data;
	u64 p_done;
	u32 offset_bar0;
	u16 first_bit;
	u16 last_bit;
/* out */
/* keep 64 bits to be consistent */
	u64 data;
};

/* debug accessories */

#ifdef CONFIG_DEBUG_FS
    /* debug info, default is compiled-in but effectively disabled (0 mask) */
    #define GK20A_DEBUG
    /*e.g: echo 1 > /d/tegra_host/dbg_mask */
    #define GK20A_DEFAULT_DBG_MASK 0
#else
    /* manually enable and turn it on the mask */
    /*#define NVHOST_DEBUG*/
    #define GK20A_DEFAULT_DBG_MASK (dbg_info)
#endif

enum gk20a_dbg_categories {
	gpu_dbg_info    = BIT(0),  /* lightly verbose info */
	gpu_dbg_fn      = BIT(2),  /* fn name tracing */
	gpu_dbg_reg     = BIT(3),  /* register accesses, very verbose */
	gpu_dbg_pte     = BIT(4),  /* gmmu ptes */
	gpu_dbg_intr    = BIT(5),  /* interrupts */
	gpu_dbg_pmu     = BIT(6),  /* gk20a pmu */
	gpu_dbg_clk     = BIT(7),  /* gk20a clk */
	gpu_dbg_map     = BIT(8),  /* mem mappings */
	gpu_dbg_gpu_dbg = BIT(9),  /* gpu debugger/profiler */
	gpu_dbg_mem     = BIT(31), /* memory accesses, very verbose */
};

#if defined(GK20A_DEBUG)
extern u32 gk20a_dbg_mask;
extern u32 gk20a_dbg_ftrace;
#define gk20a_dbg(dbg_mask, format, arg...)				\
do {									\
	if (unlikely((dbg_mask) & gk20a_dbg_mask)) {		\
		if (nvhost_dbg_ftrace)					\
			trace_printk(format "\n", ##arg);		\
		else							\
			pr_info("gk20a %s: " format "\n",		\
					__func__, ##arg);		\
	}								\
} while (0)

#else /* GK20A_DEBUG */
#define gk20a_dbg(dbg_mask, format, arg...)				\
do {									\
	if (0)								\
		pr_info("gk20a %s: " format "\n", __func__, ##arg);\
} while (0)

#endif

#define gk20a_err(d, fmt, arg...) \
	dev_err(d, "%s: " fmt "\n", __func__, ##arg)

#define gk20a_warn(d, fmt, arg...) \
	dev_warn(d, "%s: " fmt "\n", __func__, ##arg)

#define gk20a_dbg_fn(fmt, arg...) \
	gk20a_dbg(gpu_dbg_fn, fmt, ##arg)

#define gk20a_dbg_info(fmt, arg...) \
	gk20a_dbg(gpu_dbg_info, fmt, ##arg)

/* mem access with dbg_mem logging */
static inline u8 gk20a_mem_rd08(void *ptr, int b)
{
	u8 _b = ((const u8 *)ptr)[b];
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u8)*b, _b);
#endif
	return _b;
}
static inline u16 gk20a_mem_rd16(void *ptr, int s)
{
	u16 _s = ((const u16 *)ptr)[s];
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u16)*s, _s);
#endif
	return _s;
}
static inline u32 gk20a_mem_rd32(void *ptr, int w)
{
	u32 _w = ((const u32 *)ptr)[w];
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr + sizeof(u32)*w, _w);
#endif
	return _w;
}
static inline void gk20a_mem_wr08(void *ptr, int b, u8 data)
{
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u8)*b, data);
#endif
	((u8 *)ptr)[b] = data;
}
static inline void gk20a_mem_wr16(void *ptr, int s, u16 data)
{
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u16)*s, data);
#endif
	((u16 *)ptr)[s] = data;
}
static inline void gk20a_mem_wr32(void *ptr, int w, u32 data)
{
#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
	gk20a_dbg(gpu_dbg_mem, " %p = 0x%x", ptr+sizeof(u32)*w, data);
#endif
	((u32 *)ptr)[w] = data;
}

/* register accessors */
static inline void gk20a_writel(struct gk20a *g, u32 r, u32 v)
{
	gk20a_dbg(gpu_dbg_reg, " r=0x%x v=0x%x", r, v);
	writel(v, g->regs + r);
}
static inline u32 gk20a_readl(struct gk20a *g, u32 r)
{
	u32 v = readl(g->regs + r);
	gk20a_dbg(gpu_dbg_reg, " r=0x%x v=0x%x", r, v);
	return v;
}

static inline void gk20a_bar1_writel(struct gk20a *g, u32 b, u32 v)
{
	gk20a_dbg(gpu_dbg_reg, " b=0x%x v=0x%x", b, v);
	writel(v, g->bar1 + b);
}

static inline u32 gk20a_bar1_readl(struct gk20a *g, u32 b)
{
	u32 v = readl(g->bar1 + b);
	gk20a_dbg(gpu_dbg_reg, " b=0x%x v=0x%x", b, v);
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
static inline struct gk20a *gk20a_from_as(struct gk20a_as *as)
{
	return container_of(as, struct gk20a, as);
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
	spin_lock(&gr->ch_tlb_lock);
	memset(gr->chid_tlb, 0,
		sizeof(struct gr_channel_map_tlb_entry) *
		GR_CHANNEL_MAP_TLB_SIZE);
	spin_unlock(&gr->ch_tlb_lock);
}

/* classes that the device supports */
/* TBD: get these from an open-sourced SDK? */
enum {
	KEPLER_C                  = 0xA297,
	FERMI_TWOD_A              = 0x902D,
	KEPLER_COMPUTE_A          = 0xA0C0,
	KEPLER_INLINE_TO_MEMORY_A = 0xA040,
	KEPLER_DMA_COPY_A         = 0xA0B5, /*not sure about this one*/
};

#if defined(CONFIG_GK20A_PMU)
static inline int support_gk20a_pmu(void)
{
	return 1;
}
#else
static inline int support_gk20a_pmu(void){return 0;}
#endif

void gk20a_create_sysfs(struct platform_device *dev);

#ifdef CONFIG_DEBUG_FS
int clk_gk20a_debugfs_init(struct platform_device *dev);
#endif

#define GK20A_BAR0_IORESOURCE_MEM 0
#define GK20A_BAR1_IORESOURCE_MEM 1
#define GK20A_SIM_IORESOURCE_MEM 2

void gk20a_busy_noresume(struct platform_device *pdev);
int gk20a_busy(struct platform_device *pdev);
void gk20a_idle(struct platform_device *pdev);
int gk20a_channel_busy(struct platform_device *pdev);
void gk20a_channel_idle(struct platform_device *pdev);
void gk20a_disable(struct gk20a *g, u32 units);
void gk20a_enable(struct gk20a *g, u32 units);
void gk20a_reset(struct gk20a *g, u32 units);
int gk20a_get_client(struct gk20a *g);
void gk20a_put_client(struct gk20a *g);

const struct firmware *
gk20a_request_firmware(struct gk20a *g, const char *fw_name);

#define NVHOST_GPU_ARCHITECTURE_SHIFT 4

/* constructs unique and compact GPUID from nvhost_gpu_characteristics
 * arch/impl fields */
#define GK20A_GPUID(arch, impl) ((u32) ((arch) | (impl)))

#define GK20A_GPUID_GK20A \
	GK20A_GPUID(NVHOST_GPU_ARCH_GK100, NVHOST_GPU_IMPL_GK20A)

int gk20a_init_gpu_characteristics(struct gk20a *g);

#endif /* _NVHOST_GK20A_H_ */
