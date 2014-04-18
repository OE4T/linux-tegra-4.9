/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This header is BSD licensed so anyone can use the definitions to implement
 * compatible drivers/servers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of NVIDIA CORPORATION nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NVIDIA CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __TEGRA_IVC_CONFIG_H
#define __TEGRA_IVC_CONFIG_H

#include <linux/types.h>

/* 8 x 4 = 32 bytes */
struct tegra_hv_queue_data {
	uint32_t	id;	/* IVC id */
	uint32_t	peers[2];
	uint32_t	size;
	uint32_t	nframes;
	uint32_t	frame_size;
	uint32_t	offset;
	uint32_t	flags;
};

/* Queue flags */
#define TQF_STREAM_MODE		(1 << 0)
#define TQF_PAGE_ALIGNED	(1 << 1)

struct tegra_hv_shared_data {
	uint32_t	magic;
	uint32_t	sum;
	uint32_t	nr_queues;
	uint32_t	flags;
	/* Do not use fields; only one is possible */
	/* struct tegra_hv_queue_data	queue_data[]; */
};

#define TEGRA_HV_SHD_MAGIC	0xf00fbaaf

#define tegra_hv_shd_to_queue_data(_shd) \
	((struct tegra_hv_queue_data *) \
		((void *)(_shd) + sizeof(struct tegra_hv_shared_data)))


#define tegra_hv_server_data_size(nr_queues) \
	(sizeof(struct tegra_hv_shared_data) + \
	 sizeof(struct tegra_hv_queue_data) * (nr_queues))

#endif
