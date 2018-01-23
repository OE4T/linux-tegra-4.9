/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_NVLINK_GV100_H
#define NVGPU_NVLINK_GV100_H

struct gk20a;

#define MINION_REG_RD32(g, off) gk20a_readl(g, g->nvlink.minion_base + (off))
#define MINION_REG_WR32(g, off, v) gk20a_writel(g, g->nvlink.minion_base + (off), (v))
#define IOCTRL_REG_RD32(g, off) gk20a_readl(g, g->nvlink.ioctrl_base + (off))
#define IOCTRL_REG_WR32(g, off, v) gk20a_writel(g, g->nvlink.ioctrl_base + (off), (v));
#define MIF_REG_RD32(g, id, off) gk20a_readl(g, g->nvlink.links[(id)].mif_base + (off))
#define MIF_REG_WR32(g, id, off, v) gk20a_writel(g, g->nvlink.links[(id)].mif_base + (off), (v))
#define IPT_REG_RD32(g, off) gk20a_readl(g, g->nvlink.ipt_base + (off))
#define IPT_REG_WR32(g, off, v) gk20a_writel(g, g->nvlink.ipt_base + (off), (v))
#define TLC_REG_RD32(g, id, off) gk20a_readl(g, g->nvlink.links[(id)].tl_base + (off))
#define TLC_REG_WR32(g, id, off, v) gk20a_writel(g, g->nvlink.links[(id)].tl_base + (off), (v))
#define DLPL_REG_RD32(g, id, off) gk20a_readl(g, g->nvlink.links[(id)].dlpl_base + (off))
#define DLPL_REG_WR32(g, id, off, v) gk20a_writel(g, g->nvlink.links[(id)].dlpl_base + (off), (v))

int gv100_nvlink_discover_ioctrl(struct gk20a *g);
int gv100_nvlink_discover_link(struct gk20a *g);
int gv100_nvlink_init(struct gk20a *g);
int gv100_nvlink_isr(struct gk20a *g);
/* API */
int gv100_nvlink_link_early_init(struct gk20a *g, unsigned long mask);
u32 gv100_nvlink_link_get_mode(struct gk20a *g, u32 link_id);
u32 gv100_nvlink_link_get_state(struct gk20a *g, u32 link_id);
int gv100_nvlink_link_set_mode(struct gk20a *g, u32 link_id, u32 mode);
u32 gv100_nvlink_link_get_sublink_mode(struct gk20a *g, u32 link_id,
	bool is_rx_sublink);
u32 gv100_nvlink_link_get_tx_sublink_state(struct gk20a *g, u32 link_id);
u32 gv100_nvlink_link_get_rx_sublink_state(struct gk20a *g, u32 link_id);
int gv100_nvlink_link_set_sublink_mode(struct gk20a *g, u32 link_id,
	bool is_rx_sublink, u32 mode);
int gv100_nvlink_interface_init(struct gk20a *g);
int gv100_nvlink_reg_init(struct gk20a *g);
int gv100_nvlink_shutdown(struct gk20a *g);
int gv100_nvlink_early_init(struct gk20a *g);
#endif
