/*
 * Copyright (c) 2013-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>
#include <linux/tegra-firmwares.h>
#include <linux/tegra-ivc.h>
#include <linux/uaccess.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/tegra_pasr.h>
#include <soc/tegra/tegra_powergate.h>
#include "../../../arch/arm/mach-tegra/iomap.h"
#include "bpmp.h"

static void *hv_virt_base;
static struct device *device;
char firmware_tag[32];

static int bpmp_get_fwtag(void)
{
	const size_t sz = sizeof(firmware_tag) + 1;
	dma_addr_t phys;
	char *virt;
	int r;

	virt = tegra_bpmp_alloc_coherent(sz, &phys, GFP_KERNEL);
	if (!virt)
		return -ENOMEM;

	r = tegra_bpmp_send_receive(MRQ_QUERY_TAG,
			&phys, sizeof(phys), NULL, 0);
	if (r)
		goto exit;

	memcpy(firmware_tag, virt, sz - 1);

	virt[sz - 1] = 0;
	dev_info(device, "firmware tag is %s\n", virt);

exit:
	tegra_bpmp_free_coherent(sz, virt, phys);

	return r;
}

static ssize_t bpmp_version(struct device *dev, char *data, size_t size)
{
	return snprintf(data, size, "firmware tag %*.*s",
		 (int)sizeof(firmware_tag), (int)sizeof(firmware_tag),
		 firmware_tag);
}

static void *bpmp_get_virt_for_alloc(void *virt, dma_addr_t phys)
{
	if (hv_virt_base)
		return hv_virt_base + phys;

	return virt;
}

void *tegra_bpmp_alloc_coherent(size_t size, dma_addr_t *phys,
		gfp_t flags)
{
	void *virt;

	if (!device)
		return NULL;

	virt = dma_alloc_coherent(device, size, phys,
			flags);

	virt = bpmp_get_virt_for_alloc(virt, *phys);

	return virt;
}
EXPORT_SYMBOL(tegra_bpmp_alloc_coherent);

static void *bpmp_get_virt_for_free(void *virt, dma_addr_t phys)
{
	if (hv_virt_base)
		return (void *)(virt - hv_virt_base);

	return virt;
}

void tegra_bpmp_free_coherent(size_t size, void *vaddr,
		dma_addr_t phys)
{
	if (!device) {
		pr_err("device not found\n");
		return;
	}

	vaddr = bpmp_get_virt_for_free(vaddr, phys);

	dma_free_coherent(device, size, vaddr, phys);
}
EXPORT_SYMBOL(tegra_bpmp_free_coherent);

static struct syscore_ops bpmp_syscore_ops = {
	.resume = tegra_bpmp_resume,
};

static int bpmp_do_ping(void)
{
	unsigned long flags;
	int ret;

	local_irq_save(flags);
	ret = __bpmp_do_ping();
	local_irq_restore(flags);
	pr_info("bpmp: ping status is %d\n", ret);

	return ret;
}

static struct platform_device bpmp_tty = {
	.name = "tegra-bpmp-tty",
	.id = -1,
};

static int bpmp_init_powergate(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 pd_cells;

	if (of_property_read_u32(np, "#power-domain-cells", &pd_cells))
		return 0;

	if (pd_cells != 1) {
		dev_err(&pdev->dev, "%s #power-domain-cells must be 1\n",
			np->full_name);
		return -ENODEV;
	}

	return tegra_bpmp_init_powergate(pdev);
}

static int bpmp_setup_allocator(struct device *dev)
{
	uint32_t mempool_id;
	int ret;
	struct tegra_hv_ivm_cookie *ivm;
	void *virt_base;

	ret = of_property_read_u32_index(dev->of_node, "mempool", 0,
			&mempool_id);
	if (ret) {
		dev_err(dev, "failed to read mempool id (%d)\n", ret);
		return ret;
	}

	ivm = tegra_hv_mempool_reserve(dev->of_node, mempool_id);
	if (IS_ERR_OR_NULL(ivm)) {
		if (!IS_ERR(ivm))
			dev_err(dev, "No mempool found\n");
		return -ENOMEM;
	}

	dev_info(dev, "Found mempool with id %u\n", mempool_id);
	dev_info(dev, "ivm %pa\n", &ivm->ipa);

	virt_base = ioremap_cache(ivm->ipa, ivm->size);

	ret = dma_declare_coherent_memory(dev, ivm->ipa, 0, ivm->size,
			DMA_MEMORY_NOMAP | DMA_MEMORY_EXCLUSIVE);

	if (!(ret & DMA_MEMORY_NOMAP)) {
		dev_err(dev, "dma_declare_coherent_memory failed (%x)\n", ret);
		return ret;
	}

	hv_virt_base = virt_base;

	return 0;
}

static int bpmp_clk_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk *sclk;
	struct clk *emc_clk;

	sclk = devm_clk_get(dev, "sclk");
	if (IS_ERR(sclk)) {
		dev_err(dev, "cannot get avp sclk\n");
		return -ENODEV;
	}

	emc_clk = devm_clk_get(dev, "emc");
	if (IS_ERR(emc_clk)) {
		dev_err(dev, "cannot get avp emc clk\n");
		return -ENODEV;
	}

	clk_prepare_enable(sclk);
	clk_prepare_enable(emc_clk);

	if (tegra21_pasr_init(&pdev->dev))
		dev_err(&pdev->dev, "PASR init failed\n");

	return 0;
}

static int bpmp_linear_map_init(struct platform_device *pdev)
{
	struct device_node *node;
	DEFINE_DMA_ATTRS(attrs);
	uint32_t of_start;
	uint32_t of_size;
	int ret;

	node = pdev->dev.of_node;

	ret = of_property_read_u32(node, "carveout-start", &of_start);
	if (ret)
		return ret;

	ret = of_property_read_u32(node, "carveout-size", &of_size);
	if (ret)
		return ret;

	dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, &attrs);
	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	ret = dma_map_linear_attrs(&pdev->dev, of_start, of_size, 0, &attrs);
	if (ret == DMA_ERROR_CODE)
		return -ENOMEM;

	return 0;
}

struct dentry * __weak bpmp_init_debug(struct platform_device *pdev)
{
	return NULL;
}

int __weak bpmp_init_cpuidle_debug(struct dentry *root)
{
	return 0;
}

struct pconfig {
	uint8_t clk;
	uint8_t cpuidle;
	uint8_t hv;
	uint8_t lin_map;
};

static int bpmp_probe(struct platform_device *pdev)
{
	const struct pconfig *cfg;
	struct dentry *root;
	int r;

	device = &pdev->dev;

	cfg = of_device_get_match_data(&pdev->dev);
	if (!cfg) {
		r = -ENODEV;
		goto err_out;
	}

	if (cfg->hv) {
		r = bpmp_setup_allocator(&pdev->dev);
		if (r)
			goto err_out;
	}

	if (cfg->lin_map) {
		r = bpmp_linear_map_init(pdev);
		if (r)
			goto err_out;
	}

	if (cfg->clk) {
		r = bpmp_clk_init(pdev);
		if (r)
			goto err_out;
	}

	root = bpmp_init_debug(pdev);
	if (!root) {
		r = -ENOMEM;
		goto err_out;
	}

	if (cfg->cpuidle) {
		r = bpmp_init_cpuidle_debug(root);
		if (r)
			goto err_out;
	}

	bpmp_tty.dev.platform_data = root;

	r = bpmp_do_ping();
	r = r ?: bpmp_get_fwtag();
	r = r ?: of_platform_populate(device->of_node, NULL, NULL, device);
	r = r ?: platform_device_register(&bpmp_tty);
	if (r)
		goto err_out;

	register_syscore_ops(&bpmp_syscore_ops);

	devm_tegrafw_register(device, "bpmp", TFW_NORMAL, bpmp_version, NULL);

	r = bpmp_init_powergate(pdev);
	if (r) {
		dev_err(device, "powergating init failed (%d)\n", r);
		goto err_out;
	}

	dev_info(device, "probe ok\n");

	return 0;

err_out:
	dev_err(device, "probe failed (%d)\n", r);

	return r;
}

static const struct pconfig t210_cfg = {
	.clk = 1,
	.cpuidle = 1,
	.lin_map = 1
};

static const struct pconfig t186_native_cfg;

static const struct pconfig t186_hv_cfg = {
	.hv = 1
};

static const struct of_device_id bpmp_of_matches[] = {
	{ .compatible = "nvidia,tegra186-bpmp", .data = &t186_native_cfg },
	{ .compatible = "nvidia,tegra186-bpmp-hv", .data = &t186_hv_cfg },
	{ .compatible = "nvidia,tegra210-bpmp", .data = &t210_cfg },
	{}
};

static struct platform_driver bpmp_driver = {
	.probe = bpmp_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "bpmp",
		.of_match_table = of_match_ptr(bpmp_of_matches)
	}
};

static __init int bpmp_init(void)
{
	struct device_node *np;
	int r = -ENODEV;

	np = of_find_matching_node(NULL, bpmp_of_matches);
	if (!np || !of_device_is_available(np))
		goto out;

	r = bpmp_mail_init(&chip_mail_ops, np);
	if (r)
		goto out;

	r = platform_driver_register(&bpmp_driver);

out:
	of_node_put(np);

	return r;
}
postcore_initcall(bpmp_init);
