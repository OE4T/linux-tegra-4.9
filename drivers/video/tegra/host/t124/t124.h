/*
 * drivers/video/tegra/host/t124/t124.h
 *
 * Tegra Graphics Chip support for T124
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _NVHOST_T124_H_
#define _NVHOST_T124_H_

/* HACK.  Get this from auto-generated hardware def'n instead... */
#define T124_NVHOST_NUMCHANNELS 12
#define NVHOST_CHANNEL_BASE 0

/*
 * NOTE: currently we have user-mode code which is relying upon
 * this *exact* ordering of module mutexes across *all* SOCs.
 * For now just deal with it as we have the space to accomodate.
 * The entries marked "n/a" below can be removed when that issue is
 * fixed (and the entries could then be re-ordered/packed).
 */
#define NVMODMUTEX_2D_FULL_X   (1) /* n/a */
#define NVMODMUTEX_2D_SIMPLE_X (2) /* n/a */
#define NVMODMUTEX_2D_SB_A_X   (3) /* n/a */
#define NVMODMUTEX_2D_SB_B_X   (4) /* n/a */
#define NVMODMUTEX_3D        (5)
#define NVMODMUTEX_DISPLAYA  (6)
#define NVMODMUTEX_DISPLAYB  (7)
#define NVMODMUTEX_VI_0      (8)
#define NVMODMUTEX_DSI       (9)
#ifdef CONFIG_ARCH_TEGRA_VIC
#define NVMODMUTEX_VIC       (10)
#endif
#define NVMODMUTEX_VI_1      (11)
#define NVMODMUTEX_ISP_0     (1) /* above "1" isn't used in practice on t124 */
#define NVMODMUTEX_ISP_1     (2) /* above "2" isn't used in practice on t124 */

struct nvhost_chip_support;

int nvhost_init_t124_support(struct nvhost_master *,
		struct nvhost_chip_support *);
int nvhost_init_t124_channel_support(struct nvhost_master *,
		struct nvhost_chip_support *);
int nvhost_init_t124_cdma_support(struct nvhost_chip_support *);
int nvhost_init_t124_debug_support(struct nvhost_chip_support *);
int nvhost_init_t124_syncpt_support(struct nvhost_master *,
		struct nvhost_chip_support *);
int nvhost_init_t124_intr_support(struct nvhost_chip_support *);
int nvhost_init_t124_cpuaccess_support(struct nvhost_master *,
		struct nvhost_chip_support *);
int nvhost_init_t124_as_support(struct nvhost_chip_support *);

/* these sort of stick out, per module support */
int t124_nvhost_hwctx_handler_init(struct nvhost_channel *ch);

struct gk20a;

struct t124 {
	struct nvhost_master *host;
	struct gk20a *gk20a;
};

#endif /* _NVHOST_T124_H_ */
