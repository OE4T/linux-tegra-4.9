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

#include <asm-generic/dma-coherent.h>

#include <asm/dma-iommu.h>

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

/* do not use this function in a driver */
static inline bool skip_device_cache_sync(struct device *dev, unsigned long attrs)
{
	if (DMA_ATTR_SKIP_CPU_SYNC & attrs)
		return true;
	return is_device_dma_coherent(dev);
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

#define dma_alloc_at_coherent(d, s, h, f) dma_alloc_at_attrs(d, s, h, f, 0)

static inline void *dma_alloc_at_attrs(struct device *dev, size_t size,
				       dma_addr_t *dma_handle, gfp_t flags,
				       unsigned long attrs)
{
	struct dma_map_ops *ops = get_dma_ops(dev);
	void *vaddr;

	vaddr = ops->alloc(dev, size, dma_handle, flags, attrs);
	debug_dma_alloc_coherent(dev, size, *dma_handle, vaddr);
	return vaddr;
}

static inline dma_addr_t
dma_map_linear_attrs(struct device *dev, phys_addr_t pa, size_t size,
			enum dma_data_direction dir, unsigned long attrs)
{
	void *va = phys_to_virt(pa);
	struct dma_map_ops *ops = get_dma_ops(dev);
	dma_addr_t addr = DMA_ERROR_CODE;

	if (!ops || !ops->map_at) {
		WARN(1, "map_at is not supported\n");
		return DMA_ERROR_CODE;
	}

	addr = ops->map_at(dev, pa, pa, size, dir, attrs);

	if (addr != DMA_ERROR_CODE) {
		kmemcheck_mark_initialized(va, size);
		debug_dma_map_page(dev, virt_to_page(va),
				   (unsigned long)va & ~PAGE_MASK, size,
				   dir, addr, true);
	}
	return addr;
}

#define CONFIG_ARM_DMA_USE_IOMMU

#ifdef CONFIG_ARM_DMA_USE_IOMMU
extern bool device_is_iommuable(struct device *dev);
#else
static inline bool device_is_iommuable(struct device *dev)
{
	return false;
}
#endif

static inline void set_dma_ops(struct device *dev, struct dma_map_ops *ops)
{
	dev->archdata.dma_ops = ops;
}

#endif	/* __KERNEL__ */
#endif	/* __ASM_DMA_MAPPING_H */
