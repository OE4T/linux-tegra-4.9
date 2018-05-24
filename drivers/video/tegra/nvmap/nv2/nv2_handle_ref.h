/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __NVMAP2_HANDLE_REF_H
#define __NVMAP2_HANDLE_REF_H

struct nvmap_handle_ref *NVMAP2_handle_ref_create(struct nvmap_handle *handle);
void NVMAP2_handle_ref_free(struct nvmap_handle_ref *ref);

int NVMAP2_handle_ref_get(struct nvmap_handle_ref *ref);
int NVMAP2_handle_ref_put(struct nvmap_handle_ref *ref);

#endif /* __NVMAP2_HANDLE_REF_H */
