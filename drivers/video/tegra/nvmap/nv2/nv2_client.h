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

#ifndef __NVMAP2_CLIENT_H
#define __NVMAP2_CLIENT_H

/*
 * CLIENT
 */
void NVMAP2_client_remove_ref(struct nvmap_client *client,
					struct nvmap_handle_ref *ref);
void NVMAP2_client_add_ref(struct nvmap_client *client,
			   struct nvmap_handle_ref *ref);
struct nvmap_handle_ref *NVMAP2_client_to_handle_ref(struct nvmap_client *client,
					struct nvmap_handle *handle);

int NVMAP2_client_add_handle(struct nvmap_client *client,
			   struct nvmap_handle *handle);
void NVMAP2_client_remove_handle(struct nvmap_client *client,
			   struct nvmap_handle *handle);

int NVMAP2_client_create_handle(struct nvmap_client *client, size_t size);

int NVMAP2_client_create_fd(struct nvmap_client *client);

void NVMAP2_client_warn_if_no_tag(struct nvmap_client *client,
					unsigned int flags);

#endif /* __NVMAP2_CLIENT_H */
