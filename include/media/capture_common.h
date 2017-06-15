/*
 * Tegra capture common operations
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Sudhir Vyas <svyas@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <media/mc_common.h>

/* buffer details including dma_buf and iova etc. */
struct capture_common_buf {
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t iova;
};

/* unpin details for a capture channel, per request */
struct capture_common_unpins {
	uint32_t num_unpins;
	struct capture_common_buf data[];
};

struct capture_common_relocs {
	uint32_t num_relocs;
	uint32_t reloc_relatives[];
};

struct capture_common_pin_req {
	struct device *dev;
	struct device *rtcpu_dev;
	struct capture_common_unpins *unpins;
	struct capture_common_buf *requests;
	struct capture_common_buf *requests_dev;
	uint32_t request_size;
	uint32_t request_offset;
	uint32_t requests_mem;
	struct capture_common_relocs *relocs;
};

int capture_common_pin_memory(struct device *dev,
		uint32_t mem, struct capture_common_buf *unpin_data);

void capture_common_unpin_memory(struct capture_common_buf *unpin_data);

int capture_common_request_pin_and_reloc(struct capture_common_pin_req *req);
