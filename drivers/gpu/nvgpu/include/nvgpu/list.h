/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_LIST_H
#define NVGPU_LIST_H

struct nvgpu_list_node {
	struct nvgpu_list_node *prev;
	struct nvgpu_list_node *next;
};

static inline void nvgpu_init_list_node(struct nvgpu_list_node *node)
{
	node->prev = node;
	node->next = node;
}

static inline void nvgpu_list_add(struct nvgpu_list_node *new_node, struct nvgpu_list_node *head)
{
	new_node->next = head->next;
	new_node->next->prev = new_node;
	new_node->prev = head;
	head->next = new_node;
}

static inline void nvgpu_list_add_tail(struct nvgpu_list_node *new_node, struct nvgpu_list_node *head)
{
	new_node->prev = head->prev;
	new_node->prev->next = new_node;
	new_node->next = head;
	head->prev = new_node;
}

static inline void nvgpu_list_del(struct nvgpu_list_node *node)
{
	node->prev->next = node->next;
	node->next->prev = node->prev;
	nvgpu_init_list_node(node);
}

static inline int nvgpu_list_empty(struct nvgpu_list_node *head)
{
	return head->next == head;
}

static inline void nvgpu_list_move(struct nvgpu_list_node *node, struct nvgpu_list_node *head)
{
	nvgpu_list_del(node);
	nvgpu_list_add(node, head);
}

static inline void nvgpu_list_replace_init(struct nvgpu_list_node *old_node, struct nvgpu_list_node *new_node)
{
	new_node->next = old_node->next;
	new_node->next->prev = new_node;
	new_node->prev = old_node->prev;
	new_node->prev->next = new_node;
	nvgpu_init_list_node(old_node);
}

#define nvgpu_list_entry(ptr, type, member)	\
	type ## _from_ ## member(ptr)

#define nvgpu_list_next_entry(pos, type, member)	\
	nvgpu_list_entry((pos)->member.next, type, member)

#define nvgpu_list_first_entry(ptr, type, member)	\
	nvgpu_list_entry((ptr)->next, type, member)

#define nvgpu_list_for_each_entry(pos, head, type, member)	\
	for (pos = nvgpu_list_first_entry(head, type, member);	\
		&pos->member != (head);				\
		pos = nvgpu_list_next_entry(pos, type, member))

#define nvgpu_list_for_each_entry_safe(pos, n, head, type, member)	\
	for (pos = nvgpu_list_first_entry(head, type, member),		\
			n = nvgpu_list_next_entry(pos, type, member);	\
		&pos->member != (head);					\
		pos = n, n = nvgpu_list_next_entry(n, type, member))

#endif /* NVGPU_LIST_H */
