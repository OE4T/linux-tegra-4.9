/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_DMA_MAPPING_H
#define __ASM_DMA_MAPPING_H

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/vmalloc.h>

#include <xen/xen.h>
#include <asm/xen/hypervisor.h>

#define DMA_ERROR_CODE	(~(dma_addr_t)0)
extern struct dma_map_ops dummy_dma_ops;

static inline struct dma_map_ops *__generic_dma_ops(struct device *dev)
{
	if (dev && dev->archdata.dma_ops)
		return dev->archdata.dma_ops;

	/*
	 * We expect no ISA devices, and all other DMA masters are expected to
	 * have someone call arch_setup_dma_ops at device creation time.
	 */
	return &dummy_dma_ops;
}

static inline struct dma_map_ops *get_dma_ops(struct device *dev)
{
	if (xen_initial_domain())
		return xen_dma_ops;
	else
		return __generic_dma_ops(dev);
}

void arch_setup_dma_ops(struct device *dev, u64 dma_base, u64 size,
			const struct iommu_ops *iommu, bool coherent);
#define arch_setup_dma_ops	arch_setup_dma_ops

#ifdef CONFIG_IOMMU_DMA
void arch_teardown_dma_ops(struct device *dev);
#define arch_teardown_dma_ops	arch_teardown_dma_ops
#endif

/* do not use this function in a driver */
static inline bool is_device_dma_coherent(struct device *dev)
{
	if (!dev)
		return false;
	return dev->archdata.dma_coherent;
}

static inline dma_addr_t phys_to_dma(struct device *dev, phys_addr_t paddr)
{
	dma_addr_t dev_addr = (dma_addr_t)paddr;

	return dev_addr - ((dma_addr_t)dev->dma_pfn_offset << PAGE_SHIFT);
}

static inline phys_addr_t dma_to_phys(struct device *dev, dma_addr_t dev_addr)
{
	phys_addr_t paddr = (phys_addr_t)dev_addr;

	return paddr + ((phys_addr_t)dev->dma_pfn_offset << PAGE_SHIFT);
}

static inline bool dma_capable(struct device *dev, dma_addr_t addr, size_t size)
{
	if (!dev->dma_mask)
		return false;

	return addr + size - 1 <= *dev->dma_mask;
}

static inline void dma_mark_clean(void *addr, size_t size)
{
}

/* Override for dma_max_pfn() */
static inline unsigned long dma_max_pfn(struct device *dev)
{
	dma_addr_t dma_max = (dma_addr_t)*dev->dma_mask;

	return (ulong)dma_to_phys(dev, dma_max) >> PAGE_SHIFT;
}
#define dma_max_pfn(dev) dma_max_pfn(dev)

static inline dma_addr_t dma_iova_alloc_at(struct device *dev, dma_addr_t *addr,
					   size_t size, struct dma_attrs *attrs)
{
	struct dma_map_ops *ops = get_dma_ops(dev);
	BUG_ON(!ops);

	return ops->iova_alloc_at(dev, addr, size, attrs);
}

static inline dma_addr_t
dma_map_linear_attrs(struct device *dev, phys_addr_t pa, size_t size,
			enum dma_data_direction dir, struct dma_attrs *attrs)
{
	dma_addr_t da, req = pa;
	void *va = phys_to_virt(pa);
	DEFINE_DMA_ATTRS(_attrs);
	struct dma_map_ops *ops = get_dma_ops(dev);
	dma_addr_t addr;

	if (ops && ops->linear_map)
		return ops->linear_map(dev, pa, size, dir, attrs);

	da = dma_iova_alloc_at(dev, &req, size, attrs);
	if (da == DMA_ERROR_CODE) {
		struct dma_iommu_mapping *map;
		dma_addr_t end = pa + size;
		size_t bytes = 0;

		switch (req) {
		case -ENXIO:
			map = to_dma_iommu_mapping(dev);
			dev_info(dev, "Trying to IOVA linear map %pa-%pa outside of as:%pa-%pa\n",
				 &pa, &end, &map->base, &map->end);

			if ((pa >= map->base) && (pa < map->end)) {
				req = pa;
				bytes = map->end - pa;
			} else if ((end > map->base) && (end <= map->end)) {
				req = map->base;
				bytes = end - map->base;
			}

			/* Partially reserve within IOVA map */
			if (bytes) {
				da = dma_iova_alloc_at(dev, &req, bytes, attrs);
				if (da == DMA_ERROR_CODE)
					return DMA_ERROR_CODE;

				if (!dma_get_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs))
					dma_sync_single_for_device(NULL, da, bytes, dir);
			}

			/* Allow to map outside of map */
			if (!attrs)
				attrs = &_attrs;
			dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, attrs);
			da = (dma_addr_t)pa;
			break;
		case -EINVAL:
		default:
			return DMA_ERROR_CODE;
		}
	}

	kmemcheck_mark_initialized(va, size);
	BUG_ON(!valid_dma_direction(dir));
	addr = ops->map_page_at(dev, virt_to_page(va), da,
			     (unsigned long)va & ~PAGE_MASK, size,
			     dir, attrs);
	debug_dma_map_page(dev, virt_to_page(va),
			   (unsigned long)va & ~PAGE_MASK, size,
			   dir, addr, true);
	return addr;
}

#endif	/* __KERNEL__ */
#endif	/* __ASM_DMA_MAPPING_H */
