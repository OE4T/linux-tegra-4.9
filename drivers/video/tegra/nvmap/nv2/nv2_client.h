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

struct nvmap_client *NVMAP2_client_create(struct list_head *dev_client_list,
						const char *name);
void NVMAP2_client_destroy(struct nvmap_client *client);

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


void NVMAP2_client_warn_if_bad_heap(struct nvmap_client *client,
				u32 heap_type, u32 userflags);
void NVMAP2_client_warn_if_no_tag(struct nvmap_client *client,
					unsigned int flags);

pid_t NVMAP2_client_pid(struct nvmap_client *client);

void NVMAP2_client_stats_alloc(struct nvmap_client *client, size_t size);

const char *NVMAP2_client_name(struct nvmap_client *client);

void NVMAP2_client_stringify(struct nvmap_client *client, struct seq_file *s);
void NVMAP2_client_allocations_stringify(struct nvmap_client *client,
				  struct seq_file *s, u32 heap_type);
void NVMAP2_client_maps_stringify(struct nvmap_client *client,
				struct seq_file *s, u32 heap_type);
u64 NVMAP2_client_calc_mss(struct nvmap_client *client, u32 heap_type);
int NVMAP2_client_show_by_pid(struct nvmap_client *client, struct seq_file *s,
				pid_t pid);
u64 NVMAP2_handle_procrank_walk(struct nvmap_handle *h, struct mm_walk *walk,
		pid_t client_pid);
void NVMAP2_client_calc_iovmm_mss(struct nvmap_client *client, u64 *pss,
				   u64 *total);

struct nvmap_client *NVMAP2_client_from_list(struct list_head *n);
void NVMAP2_client_del_list(struct nvmap_client *client);

#endif /* __NVMAP2_CLIENT_H */
