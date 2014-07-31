/*
 * drivers/video/tegra/host/gk20a/ltc_common.c
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include "gk20a.h"
#include "gr_gk20a.h"

static int gk20a_determine_L2_size_bytes(struct gk20a *g)
{
	const u32 gpuid = GK20A_GPUID(g->gpu_characteristics.arch,
				      g->gpu_characteristics.impl);
	u32 lts_per_ltc;
	u32 ways;
	u32 sets;
	u32 bytes_per_line;
	u32 active_ltcs;
	u32 cache_size;

	u32 tmp;
	u32 active_sets_value;

	tmp = gk20a_readl(g, ltc_ltc0_lts0_tstg_cfg1_r());
	ways = hweight32(ltc_ltc0_lts0_tstg_cfg1_active_ways_v(tmp));

	active_sets_value = ltc_ltc0_lts0_tstg_cfg1_active_sets_v(tmp);
	if (active_sets_value == ltc_ltc0_lts0_tstg_cfg1_active_sets_all_v()) {
		sets = 64;
	} else if (active_sets_value ==
		 ltc_ltc0_lts0_tstg_cfg1_active_sets_half_v()) {
		sets = 32;
	} else if (active_sets_value ==
		 ltc_ltc0_lts0_tstg_cfg1_active_sets_quarter_v()) {
		sets = 16;
	} else {
		dev_err(dev_from_gk20a(g),
			"Unknown constant %u for active sets",
		       (unsigned)active_sets_value);
		sets = 0;
	}

	active_ltcs = g->gr.num_fbps;

	/* chip-specific values */
	switch (gpuid) {
	case GK20A_GPUID_GK20A:
		lts_per_ltc = 1;
		bytes_per_line = 128;
		break;

	case GK20A_GPUID_GM20B:
		lts_per_ltc = 2;
		bytes_per_line = 128;
		break;

	default:
		dev_err(dev_from_gk20a(g), "Unknown GPU id 0x%02x\n",
			(unsigned)gpuid);
		lts_per_ltc = 0;
		bytes_per_line = 0;
	}

	cache_size = active_ltcs * lts_per_ltc * ways * sets * bytes_per_line;

	return cache_size;
}

/*
 * Set the maximum number of ways that can have the "EVIST_LAST" class.
 */
static void gk20a_ltc_set_max_ways_evict_last(struct gk20a *g, u32 max_ways)
{
	u32 mgmt_reg;

	mgmt_reg = gk20a_readl(g, ltc_ltcs_ltss_tstg_set_mgmt_r()) &
		~ltc_ltcs_ltss_tstg_set_mgmt_max_ways_evict_last_f(~0);
	mgmt_reg |= ltc_ltcs_ltss_tstg_set_mgmt_max_ways_evict_last_f(max_ways);

	gk20a_writel(g, ltc_ltcs_ltss_tstg_set_mgmt_r(), mgmt_reg);
}

/*
 * Sets the ZBC color for the passed index.
 */
static void gk20a_ltc_set_zbc_color_entry(struct gk20a *g,
					  struct zbc_entry *color_val,
					  u32 index)
{
	u32 i;
	u32 real_index = index + GK20A_STARTOF_ZBC_TABLE;

	gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_index_r(),
		     ltc_ltcs_ltss_dstg_zbc_index_address_f(real_index));

	for (i = 0;
	     i < ltc_ltcs_ltss_dstg_zbc_color_clear_value__size_1_v(); i++)
		gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_color_clear_value_r(i),
			     color_val->color_l2[i]);
}

/*
 * Sets the ZBC depth for the passed index.
 */
static void gk20a_ltc_set_zbc_depth_entry(struct gk20a *g,
					  struct zbc_entry *depth_val,
					  u32 index)
{
	u32 real_index = index + GK20A_STARTOF_ZBC_TABLE;

	gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_index_r(),
		     ltc_ltcs_ltss_dstg_zbc_index_address_f(real_index));

	gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_depth_clear_value_r(),
		     depth_val->depth);
}

static int gk20a_ltc_alloc_phys_cbc(struct gk20a *g,
				    size_t compbit_backing_size)
{
	struct gr_gk20a *gr = &g->gr;
	int order = ffs(compbit_backing_size >> PAGE_SHIFT);
	struct page *pages;
	struct sg_table *sgt;
	int err = 0;

	/* allocate few pages */
	pages = alloc_pages(GFP_KERNEL, order);
	if (!pages) {
		gk20a_dbg(gpu_dbg_pte, "alloc_pages failed\n");
		err = -ENOMEM;
		goto err_alloc_pages;
	}

	/* clean up the pages */
	memset(page_address(pages), 0, compbit_backing_size);

	/* allocate room for placing the pages pointer.. */
	gr->compbit_store.pages =
		kzalloc(sizeof(*gr->compbit_store.pages), GFP_KERNEL);
	if (!gr->compbit_store.pages) {
		gk20a_dbg(gpu_dbg_pte, "failed to allocate pages struct");
		err = -ENOMEM;
		goto err_alloc_compbit_store;
	}

	err = gk20a_get_sgtable_from_pages(&g->dev->dev, &sgt, &pages, 0,
					   compbit_backing_size);
	if (err) {
		gk20a_dbg(gpu_dbg_pte, "could not get sg table for pages\n");
		goto err_alloc_sg_table;
	}

	/* store the parameters to gr structure */
	*gr->compbit_store.pages = pages;
	gr->compbit_store.base_iova = sg_phys(sgt->sgl);
	gr->compbit_store.size = compbit_backing_size;
	gr->compbit_store.sgt = sgt;

	return 0;

err_alloc_sg_table:
	kfree(gr->compbit_store.pages);
	gr->compbit_store.pages = NULL;
err_alloc_compbit_store:
	__free_pages(pages, order);
err_alloc_pages:
	return err;
}

static int gk20a_ltc_alloc_virt_cbc(struct gk20a *g,
				    size_t compbit_backing_size)
{
	struct device *d = dev_from_gk20a(g);
	struct gr_gk20a *gr = &g->gr;
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t iova;
	int err;

	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);

	gr->compbit_store.pages =
		dma_alloc_attrs(d, compbit_backing_size, &iova,
				GFP_KERNEL, &attrs);
	if (!gr->compbit_store.pages) {
		gk20a_err(dev_from_gk20a(g), "failed to allocate backing store for compbit : size %zu",
				  compbit_backing_size);
		return -ENOMEM;
	}

	gr->compbit_store.base_iova = iova;
	gr->compbit_store.size = compbit_backing_size;
	err = gk20a_get_sgtable_from_pages(d,
				   &gr->compbit_store.sgt,
				   gr->compbit_store.pages, iova,
				   compbit_backing_size);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "failed to allocate sgt for backing store");
		return err;
	}

	return 0;
}

static void gk20a_ltc_init_cbc(struct gk20a *g, struct gr_gk20a *gr)
{
	u32 max_size = gr->max_comptag_mem;
	u32 max_comptag_lines = max_size << 3;

	u32 compbit_base_post_divide;
	u64 compbit_base_post_multiply64;
	u64 compbit_store_base_iova;
	u64 compbit_base_post_divide64;

	if (IS_ENABLED(CONFIG_GK20A_PHYS_PAGE_TABLES))
		compbit_store_base_iova = gr->compbit_store.base_iova;
	else
		compbit_store_base_iova = NV_MC_SMMU_VADDR_TRANSLATE(
			gr->compbit_store.base_iova);

	compbit_base_post_divide64 = compbit_store_base_iova >>
		ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	do_div(compbit_base_post_divide64, g->ltc_count);
	compbit_base_post_divide = u64_lo32(compbit_base_post_divide64);

	compbit_base_post_multiply64 = ((u64)compbit_base_post_divide *
		g->ltc_count) << ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	if (compbit_base_post_multiply64 < compbit_store_base_iova)
		compbit_base_post_divide++;

	/* Bug 1477079 indicates sw adjustment on the posted divided base. */
	if (g->ops.ltc.cbc_fix_config)
		compbit_base_post_divide =
			g->ops.ltc.cbc_fix_config(g, compbit_base_post_divide);

	gk20a_writel(g, ltc_ltcs_ltss_cbc_base_r(),
		compbit_base_post_divide);

	gk20a_dbg(gpu_dbg_info | gpu_dbg_map | gpu_dbg_pte,
		   "compbit base.pa: 0x%x,%08x cbc_base:0x%08x\n",
		   (u32)(compbit_store_base_iova >> 32),
		   (u32)(compbit_store_base_iova & 0xffffffff),
		   compbit_base_post_divide);

	g->ops.ltc.cbc_ctrl(g, gk20a_cbc_op_invalidate,
			    0, max_comptag_lines - 1);

}
