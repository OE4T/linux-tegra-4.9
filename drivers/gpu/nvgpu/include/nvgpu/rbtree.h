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

#ifndef __NVGPU_RBTREE_H__
#define __NVGPU_RBTREE_H__

#include <nvgpu/types.h>

struct nvgpu_rbtree_node {
	u64 key_start;
	u64 key_end;

	bool is_red; /* !IsRed == IsBlack */

	struct nvgpu_rbtree_node *parent;
	struct nvgpu_rbtree_node *left;
	struct nvgpu_rbtree_node *right;
};

/**
 * nvgpu_rbtree_insert - insert a new node into rbtree
 *
 * @new_node	Pointer to new node.
 * @root	Pointer to root of tree
 *
 * Nodes with duplicate key_start and overlapping ranges
 * are not allowed
 */
void nvgpu_rbtree_insert(struct nvgpu_rbtree_node *new_node,
		    struct nvgpu_rbtree_node **root);

/**
 * nvgpu_rbtree_unlink - delete a node from rbtree
 *
 * @node	Pointer to node to be deleted
 * @root	Pointer to root of tree
 */
void nvgpu_rbtree_unlink(struct nvgpu_rbtree_node *node,
		    struct nvgpu_rbtree_node **root);

/**
 * nvgpu_rbtree_search - search a given key in rbtree
 *
 * @key_start	Key to be searched in rbtree
 * @node	Node pointer to be returned
 * @root	Pointer to root of tree
 *
 * This API will match given key against key_start of each node
 * In case of a hit, node points to a node with given key
 * In case of a miss, node is NULL
 */
void nvgpu_rbtree_search(u64 key_start, struct nvgpu_rbtree_node **node,
			     struct nvgpu_rbtree_node *root);

/**
 * nvgpu_rbtree_range_search - search a node with key falling in range
 *
 * @key		Key to be searched in rbtree
 * @node	Node pointer to be returned
 * @root	Pointer to root of tree
 *
 * This API will match given key and find a node where key value
 * falls within range of {start, end} keys
 * In case of a hit, node points to a node with given key
 * In case of a miss, node is NULL
 */
void nvgpu_rbtree_range_search(u64 key,
			       struct nvgpu_rbtree_node **node,
			       struct nvgpu_rbtree_node *root);

/**
 * nvgpu_rbtree_less_than_search - search a node with key lesser than given key
 *
 * @key_start	Key to be searched in rbtree
 * @node	Node pointer to be returned
 * @root	Pointer to root of tree
 *
 * This API will match given key and find a node with highest
 * key value lesser than given key
 * In case of a hit, node points to a node with given key
 * In case of a miss, node is NULL
 */
void nvgpu_rbtree_less_than_search(u64 key_start,
			       struct nvgpu_rbtree_node **node,
			       struct nvgpu_rbtree_node *root);

/**
 * nvgpu_rbtree_enum_start - enumerate tree starting at the node with specified value
 *
 * @key_start	Key value to begin enumeration from
 * @node	Pointer to first node in the tree
 * @root	Pointer to root of tree
 *
 * This API returns node pointer pointing to first node in the rbtree
 */
void nvgpu_rbtree_enum_start(u64 key_start,
			struct nvgpu_rbtree_node **node,
			struct nvgpu_rbtree_node *root);

/**
 * nvgpu_rbtree_enum_next - find next node in enumeration
 *
 * @node	Pointer to next node in the tree
 * @root	Pointer to root of tree
 *
 * This API returns node pointer pointing to next node in the rbtree
 */
void nvgpu_rbtree_enum_next(struct nvgpu_rbtree_node **node,
		       struct nvgpu_rbtree_node *root);

#endif /* __NVGPU_RBTREE_H__ */
