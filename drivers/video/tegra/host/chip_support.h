/*
 * drivers/video/tegra/host/chip_support.h
 *
 * Tegra Graphics Host Chip Support
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef _NVHOST_CHIP_SUPPORT_H_
#define _NVHOST_CHIP_SUPPORT_H_

struct output;
struct nvhost_waitchk;

struct nvhost_chip_support {
	struct {
		int (*init)(struct nvhost_channel *,
			    struct nvhost_master *,
			    int chid);
		int (*submit)(struct nvhost_channel *,
			      struct nvhost_hwctx *,
			      struct nvmap_client *,
			      u32 *gather,
			      u32 *gather_end,
			      struct nvhost_waitchk *waitchk,
			      struct nvhost_waitchk *waitchk_end,
			      u32 waitchk_mask,
			      struct nvmap_handle **unpins,
			      int nr_unpins,
			      u32 syncpt_id,
			      u32 syncpt_incrs,
			      u32 *syncpt_value,
			      bool null_kickoff);
		int (*read3dreg)(struct nvhost_channel *channel,
				struct nvhost_hwctx *hwctx,
				u32 offset,
				u32 *value);
	} channel;

	struct {
		void (*start)(struct nvhost_cdma *);
		void (*stop)(struct nvhost_cdma *);
		void (*kick)(struct  nvhost_cdma *);
	} cdma;

	struct {
		void (*reset)(struct push_buffer *);
		int (*init)(struct push_buffer *);
		void (*destroy)(struct push_buffer *);
		void (*push_to)(struct push_buffer *,
				struct nvmap_handle *,
				u32 op1, u32 op2);
		void (*pop_from)(struct push_buffer *,
				 unsigned int slots);
		u32 (*space)(struct push_buffer *);
		u32 (*putptr)(struct push_buffer *);
	} push_buffer;

	struct {
		void (*show_channel_cdma)(struct nvhost_master *,
					  struct output *,
					  int chid);
		void (*show_channel_fifo)(struct nvhost_master *,
					  struct output *,
					  int chid);
		void (*show_mlocks)(struct nvhost_master *m,
				    struct output *o);

	} debug;

	struct {
		void (*reset)(struct nvhost_syncpt *, u32 id);
		void (*reset_wait_base)(struct nvhost_syncpt *, u32 id);
		void (*read_wait_base)(struct nvhost_syncpt *, u32 id);
		u32 (*update_min)(struct nvhost_syncpt *, u32 id);
		void (*cpu_incr)(struct nvhost_syncpt *, u32 id);
		int (*wait_check)(struct nvhost_syncpt *sp,
				  struct nvmap_client *nvmap,
				  u32 waitchk_mask,
				  struct nvhost_waitchk *wait,
				  struct nvhost_waitchk *waitend);
		void (*debug)(struct nvhost_syncpt *);
		const char * (*name)(struct nvhost_syncpt *, u32 id);
	} syncpt;

	struct {
		void (*init_host_sync)(struct nvhost_intr *);
		void (*set_host_clocks_per_usec)(
		        struct nvhost_intr *, u32 clocks);
		void (*set_syncpt_threshold)(
		        struct nvhost_intr *, u32 id, u32 thresh);
		void (*enable_syncpt_intr)(struct nvhost_intr *, u32 id);
		void (*disable_all_syncpt_intrs)(struct nvhost_intr *);
		int  (*request_host_general_irq)(struct nvhost_intr *);
		void (*free_host_general_irq)(struct nvhost_intr *);
	} intr;

	struct {
		int (*mutex_try_lock)(struct nvhost_cpuaccess *,
				      unsigned int idx);
		void (*mutex_unlock)(struct nvhost_cpuaccess *,
				     unsigned int idx);
	} cpuaccess;
};


int nvhost_init_t20_support(struct nvhost_master *host);
int nvhost_init_t30_support(struct nvhost_master *host);


/* place holder for chip id assumed to live in kernel/arch/arm/mach-tegra */
struct tegra_chip_info {
#define TEGRA_SOC_CHIP_ARCH_T20 0
#define TEGRA_SOC_CHIP_IMPL_T20 0
	u16 arch;
#define TEGRA_SOC_CHIP_ARCH_T30 1
#define TEGRA_SOC_CHIP_IMPL_T30 0
	u16 impl;
};

#if 0
extern int tegra_get_chip_info(struct tegra_chip_info *);
#else
static inline int tegra_get_chip_info(struct tegra_chip_info *ci)
{
#if defined(CONFIG_ARCH_TEGRA_3x_SOC)
	ci->arch = TEGRA_SOC_CHIP_ARCH_T30;
	ci->impl = TEGRA_SOC_CHIP_IMPL_T30;

#elif defined(CONFIG_ARCH_TEGRA_2x_SOC)
	ci->arch = TEGRA_SOC_CHIP_ARCH_T20;
	ci->impl = TEGRA_SOC_CHIP_IMPL_T20;

#else
	return -ENODEV;
#endif

	return 0;
}
#endif

#endif /* _NVHOST_CHIP_SUPPORT_H_ */
