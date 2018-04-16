/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <gk20a/gk20a.h>
#include <nvgpu/nvlink.h>
#include <nvgpu/enabled.h>
#include "module.h"

#ifdef CONFIG_TEGRA_NVLINK
int nvgpu_nvlink_read_dt_props(struct gk20a *g)
{
	struct device_node *np;
	struct nvlink_device *ndev = g->nvlink.priv;
	u32 local_dev_id;
	u32 local_link_id;
	u32 remote_dev_id;
	u32 remote_link_id;
	u32 physical_link;
	bool is_master;

	/* Parse DT */
	np = nvgpu_get_node(g);
	if (!np)
		goto fail;

	np = of_get_child_by_name(np, "nvidia,nvlink");
	if (!np)
		goto fail;

	np = of_get_child_by_name(np, "endpoint");
	if (!np)
		goto fail;

	/* Parse DT structure to detect endpoint topology */
	of_property_read_u32(np, "local_dev_id", &local_dev_id);
	of_property_read_u32(np, "local_link_id", &local_link_id);
	of_property_read_u32(np, "remote_dev_id", &remote_dev_id);
	of_property_read_u32(np, "remote_link_id", &remote_link_id);
	of_property_read_u32(np, "physical_link", &physical_link);
	is_master = of_property_read_bool(np, "is_master");

	/* Check that we are in dGPU mode */
	if (local_dev_id != NVLINK_ENDPT_GV100) {
		nvgpu_err(g, "Local nvlink device is not dGPU");
		return -EINVAL;
	}

	ndev->is_master = is_master;
	ndev->device_id = local_dev_id;
	ndev->link.link_id = local_link_id;
	ndev->link.remote_dev_info.device_id = remote_dev_id;
	ndev->link.remote_dev_info.link_id = remote_link_id;

	g->nvlink.connected_links = BIT(physical_link);
	return 0;

fail:
	nvgpu_info(g, "nvlink endpoint not found or invaling in DT");
	return -ENODEV;
}
#endif /* CONFIG_TEGRA_NVLINK */

void nvgpu_mss_nvlink_init_credits(struct gk20a *g)
{
		/* MSS_NVLINK_1_BASE */
		void __iomem *soc1 = ioremap(0x01f20010, 4096);
		/* MSS_NVLINK_2_BASE */
		void __iomem *soc2 = ioremap(0x01f40010, 4096);
		/* MSS_NVLINK_3_BASE */
		void __iomem *soc3 = ioremap(0x01f60010, 4096);
		/* MSS_NVLINK_4_BASE */
		void __iomem *soc4 = ioremap(0x01f80010, 4096);
		u32 val;

		nvgpu_log(g, gpu_dbg_info, "init nvlink soc credits");

		val = readl_relaxed(soc1);
		writel_relaxed(val, soc1);
		val = readl_relaxed(soc1 + 4);
		writel_relaxed(val, soc1 + 4);

		val = readl_relaxed(soc2);
		writel_relaxed(val, soc2);
		val = readl_relaxed(soc2 + 4);
		writel_relaxed(val, soc2 + 4);

		val = readl_relaxed(soc3);
		writel_relaxed(val, soc3);
		val = readl_relaxed(soc3 + 4);
		writel_relaxed(val, soc3 + 4);

		val = readl_relaxed(soc4);
		writel_relaxed(val, soc4);
		val = readl_relaxed(soc4 + 4);
		writel_relaxed(val, soc4 + 4);
}
