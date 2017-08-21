#ifndef _LINUX_DMA_MAPPING_OVERRIDE_H
#define _LINUX_DMA_MAPPING_OVERRIDE_H

#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>

#define DMA_FOR_NVLINK	(1 << 5)

#define NVLINK_PHY_BIT	37

void dma_qualify_ioprot(enum dma_data_direction dir, unsigned long *ioprot);

void dma_marshal_handle(enum dma_data_direction dir, dma_addr_t *handle);

void dma_unmarshal_handle(enum dma_data_direction dir, dma_addr_t *handle);

#endif

