/*
 * GP106 Clocks
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include <nvgpu/kmem.h>

#include "gk20a/gk20a.h"
#include "common/linux/os_linux.h"

#include "clk_gp106.h"
#include "clk/clk_arb.h"

#include "gp106/mclk_gp106.h"

#include <nvgpu/hw/gp106/hw_trim_gp106.h>

#define gk20a_dbg_clk(fmt, arg...) \
	gk20a_dbg(gpu_dbg_clk, fmt, ##arg)

#ifdef CONFIG_DEBUG_FS
static int clk_gp106_debugfs_init(struct gk20a *g);
#endif

#define NUM_NAMEMAPS	4
#define XTAL4X_KHZ 108000


static u32 gp106_get_rate_cntr(struct gk20a *g, struct namemap_cfg *);
u32 gp106_crystal_clk_hz(struct gk20a *g)
{
	return (XTAL4X_KHZ * 1000);
}

unsigned long gp106_clk_measure_freq(struct gk20a *g, u32 api_domain)
{
	struct clk_gk20a *clk = &g->clk;
	u32 freq_khz;
	u32 i;
	struct namemap_cfg *c = NULL;

	for (i = 0; i < clk->namemap_num; i++) {
		if (api_domain == clk->namemap_xlat_table[i]) {
			c = &clk->clk_namemap[i];
			break;
		}
	}

	if (!c)
		return 0;

	freq_khz = c->is_counter ? c->scale * gp106_get_rate_cntr(g, c) :
		0; /* TODO: PLL read */

	/* Convert to HZ */
	return freq_khz * 1000UL;
}

int gp106_init_clk_support(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	u32 err = 0;

	gk20a_dbg_fn("");

	err = nvgpu_mutex_init(&clk->clk_mutex);
	if (err)
		return err;

	clk->clk_namemap = (struct namemap_cfg *)
		nvgpu_kzalloc(g, sizeof(struct namemap_cfg) * NUM_NAMEMAPS);

	if (!clk->clk_namemap) {
		nvgpu_mutex_destroy(&clk->clk_mutex);
		return -ENOMEM;
	}

	clk->namemap_xlat_table = nvgpu_kcalloc(g, NUM_NAMEMAPS, sizeof(u32));

	if (!clk->namemap_xlat_table) {
		nvgpu_kfree(g, clk->clk_namemap);
		nvgpu_mutex_destroy(&clk->clk_mutex);
		return -ENOMEM;
	}

	clk->clk_namemap[0] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_GPC2CLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr.reg_ctrl_addr = trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_r(),
		.cntr.reg_ctrl_idx  =
			trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_source_gpc2clk_f(),
		.cntr.reg_cntr_addr = trim_gpc_bcast_clk_cntr_ncgpcclk_cnt_r(),
		.name = "gpc2clk",
		.scale = 1
	};
	clk->namemap_xlat_table[0] = CTRL_CLK_DOMAIN_GPC2CLK;
	clk->clk_namemap[1] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_SYS2CLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr.reg_ctrl_addr = trim_sys_clk_cntr_ncsyspll_cfg_r(),
		.cntr.reg_ctrl_idx  = trim_sys_clk_cntr_ncsyspll_cfg_source_sys2clk_f(),
		.cntr.reg_cntr_addr = trim_sys_clk_cntr_ncsyspll_cnt_r(),
		.name = "sys2clk",
		.scale = 1
	};
	clk->namemap_xlat_table[1] = CTRL_CLK_DOMAIN_SYS2CLK;
	clk->clk_namemap[2] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_XBAR2CLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr.reg_ctrl_addr = trim_sys_clk_cntr_ncltcpll_cfg_r(),
		.cntr.reg_ctrl_idx  = trim_sys_clk_cntr_ncltcpll_cfg_source_xbar2clk_f(),
		.cntr.reg_cntr_addr = trim_sys_clk_cntr_ncltcpll_cnt_r(),
		.name = "xbar2clk",
		.scale = 1
	};
	clk->namemap_xlat_table[2] = CTRL_CLK_DOMAIN_XBAR2CLK;
	clk->clk_namemap[3] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_DRAMCLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr.reg_ctrl_addr = trim_fbpa_bcast_clk_cntr_ncltcclk_cfg_r(),
		.cntr.reg_ctrl_idx  =
			trim_fbpa_bcast_clk_cntr_ncltcclk_cfg_source_dramdiv4_rec_clk1_f(),
		.cntr.reg_cntr_addr = trim_fbpa_bcast_clk_cntr_ncltcclk_cnt_r(),
		.name = "dramdiv2_rec_clk1",
		.scale = 2
	};
	clk->namemap_xlat_table[3] = CTRL_CLK_DOMAIN_MCLK;

	clk->namemap_num = NUM_NAMEMAPS;

	clk->g = g;

#ifdef CONFIG_DEBUG_FS
	if (!clk->debugfs_set) {
		if (!clk_gp106_debugfs_init(g))
			clk->debugfs_set = true;
	}
#endif
	return err;
}

static u32 gp106_get_rate_cntr(struct gk20a *g, struct namemap_cfg *c) {
	u32 save_reg;
	u32 retries;
	u32 cntr = 0;

	struct clk_gk20a *clk = &g->clk;

	if (!c || !c->cntr.reg_ctrl_addr || !c->cntr.reg_cntr_addr)
		return 0;

	nvgpu_mutex_acquire(&clk->clk_mutex);

	/* Save the register */
	save_reg = gk20a_readl(g, c->cntr.reg_ctrl_addr);

	/* Disable and reset the current clock */
	gk20a_writel(g, c->cntr.reg_ctrl_addr,
				 trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_reset_asserted_f() |
				 trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_enable_deasserted_f());

	/* Force wb() */
	gk20a_readl(g, c->cntr.reg_ctrl_addr);

	/* Wait for reset to happen */
	retries = CLK_DEFAULT_CNTRL_SETTLE_RETRIES;
	do {
		nvgpu_udelay(CLK_DEFAULT_CNTRL_SETTLE_USECS);
	} while ((--retries) && (cntr = gk20a_readl(g, c->cntr.reg_cntr_addr)));

	if (!retries) {
		nvgpu_err(g, "unable to settle counter reset, bailing");
		goto read_err;
	}
	/* Program counter */
	gk20a_writel(g, c->cntr.reg_ctrl_addr,
					trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_reset_deasserted_f()          |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_enable_asserted_f()           |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_write_en_asserted_f()         |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_write_en_asserted_f()         |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_write_en_asserted_f()         |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_noofipclks_f(XTAL_CNTR_CLKS)  |
					c->cntr.reg_ctrl_idx);
	gk20a_readl(g, c->cntr.reg_ctrl_addr);

	nvgpu_udelay(XTAL_CNTR_DELAY);

	cntr = XTAL_SCALE_TO_KHZ * gk20a_readl(g, c->cntr.reg_cntr_addr);

read_err:
	/* reset and restore control register */
	gk20a_writel(g, c->cntr.reg_ctrl_addr,
				 trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_reset_asserted_f() |
				 trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_enable_deasserted_f());
	gk20a_readl(g, c->cntr.reg_ctrl_addr);
	gk20a_writel(g, c->cntr.reg_ctrl_addr, save_reg);
	gk20a_readl(g, c->cntr.reg_ctrl_addr);
	nvgpu_mutex_release(&clk->clk_mutex);

	return cntr;

}

#ifdef CONFIG_DEBUG_FS
static int gp106_get_rate_show(void *data , u64 *val) {
	struct namemap_cfg *c = (struct namemap_cfg *) data;
	struct gk20a *g = c->g;

	*val = c->is_counter ? gp106_get_rate_cntr(g, c) : 0 /* TODO PLL read */;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(get_rate_fops, gp106_get_rate_show, NULL, "%llu\n");

static int sys_cfc_read(void *data , u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	bool bload = boardobjgrpmask_bitget(
		&g->clk_pmu.clk_freq_controllers.freq_ctrl_load_mask.super,
		CTRL_CLK_CLK_FREQ_CONTROLLER_ID_SYS);

	/* val = 1 implies CLFC is loaded or enabled */
	*val = bload ? 1 : 0;
	return 0;
}
static int sys_cfc_write(void *data , u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	int status;
	/* val = 1 implies load or enable the CLFC */
	bool bload = val ? true : false;

	nvgpu_clk_arb_pstate_change_lock(g, true);
	status = clk_pmu_freq_controller_load(g, bload,
					CTRL_CLK_CLK_FREQ_CONTROLLER_ID_SYS);
	nvgpu_clk_arb_pstate_change_lock(g, false);

	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(sys_cfc_fops, sys_cfc_read, sys_cfc_write, "%llu\n");

static int ltc_cfc_read(void *data , u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	bool bload = boardobjgrpmask_bitget(
		&g->clk_pmu.clk_freq_controllers.freq_ctrl_load_mask.super,
		CTRL_CLK_CLK_FREQ_CONTROLLER_ID_LTC);

	/* val = 1 implies CLFC is loaded or enabled */
	*val = bload ? 1 : 0;
	return 0;
}
static int ltc_cfc_write(void *data , u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	int status;
	/* val = 1 implies load or enable the CLFC */
	bool bload = val ? true : false;

	nvgpu_clk_arb_pstate_change_lock(g, true);
	status = clk_pmu_freq_controller_load(g, bload,
					CTRL_CLK_CLK_FREQ_CONTROLLER_ID_LTC);
	nvgpu_clk_arb_pstate_change_lock(g, false);

	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(ltc_cfc_fops, ltc_cfc_read, ltc_cfc_write, "%llu\n");

static int xbar_cfc_read(void *data , u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	bool bload = boardobjgrpmask_bitget(
		&g->clk_pmu.clk_freq_controllers.freq_ctrl_load_mask.super,
		CTRL_CLK_CLK_FREQ_CONTROLLER_ID_XBAR);

	/* val = 1 implies CLFC is loaded or enabled */
	*val = bload ? 1 : 0;
	return 0;
}
static int xbar_cfc_write(void *data , u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	int status;
	/* val = 1 implies load or enable the CLFC */
	bool bload = val ? true : false;

	nvgpu_clk_arb_pstate_change_lock(g, true);
	status = clk_pmu_freq_controller_load(g, bload,
			CTRL_CLK_CLK_FREQ_CONTROLLER_ID_XBAR);
	nvgpu_clk_arb_pstate_change_lock(g, false);

	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(xbar_cfc_fops, xbar_cfc_read,
			xbar_cfc_write, "%llu\n");

static int gpc_cfc_read(void *data , u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	bool bload = boardobjgrpmask_bitget(
		&g->clk_pmu.clk_freq_controllers.freq_ctrl_load_mask.super,
		CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC0);

	/* val = 1 implies CLFC is loaded or enabled */
	*val = bload ? 1 : 0;
	return 0;
}
static int gpc_cfc_write(void *data , u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	int status;
	/* val = 1 implies load or enable the CLFC */
	bool bload = val ? true : false;

	nvgpu_clk_arb_pstate_change_lock(g, true);
	status = clk_pmu_freq_controller_load(g, bload,
			CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC0);
	nvgpu_clk_arb_pstate_change_lock(g, false);

	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(gpc_cfc_fops, gpc_cfc_read, gpc_cfc_write, "%llu\n");

static int clk_gp106_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *gpu_root = l->debugfs;
	struct dentry *clocks_root, *clk_freq_ctlr_root;
	struct dentry *d;
	unsigned int i;

	if (NULL == (clocks_root = debugfs_create_dir("clocks", gpu_root)))
		return -ENOMEM;

	clk_freq_ctlr_root = debugfs_create_dir("clk_freq_ctlr", gpu_root);
	if (clk_freq_ctlr_root == NULL)
		return -ENOMEM;

	d = debugfs_create_file("sys", S_IRUGO | S_IWUSR, clk_freq_ctlr_root,
				g, &sys_cfc_fops);
	d = debugfs_create_file("ltc", S_IRUGO | S_IWUSR, clk_freq_ctlr_root,
				g, &ltc_cfc_fops);
	d = debugfs_create_file("xbar", S_IRUGO | S_IWUSR, clk_freq_ctlr_root,
				g, &xbar_cfc_fops);
	d = debugfs_create_file("gpc", S_IRUGO | S_IWUSR, clk_freq_ctlr_root,
				g, &gpc_cfc_fops);

	gk20a_dbg(gpu_dbg_info, "g=%p", g);

	for (i = 0; i < g->clk.namemap_num; i++) {
		if (g->clk.clk_namemap[i].is_enable) {
			d = debugfs_create_file(
				g->clk.clk_namemap[i].name,
				S_IRUGO,
				clocks_root,
				&g->clk.clk_namemap[i],
				&get_rate_fops);
			if (!d)
				goto err_out;
		}
	}
	return 0;

err_out:
	pr_err("%s: Failed to make debugfs node\n", __func__);
	debugfs_remove_recursive(clocks_root);
	return -ENOMEM;
}
#endif /* CONFIG_DEBUG_FS */

int gp106_suspend_clk_support(struct gk20a *g)
{
	nvgpu_mutex_destroy(&g->clk.clk_mutex);
	return 0;
}
