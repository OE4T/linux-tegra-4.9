/*
 * GK20A color decompression engine support
 *
 * Copyright (c) 2014, NVIDIA Corporation.  All rights reserved.
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

#ifndef _CDE_GK20A_H_
#define _CDE_GK20A_H_

#include "mm_gk20a.h"

#define MAX_CDE_BUFS		10
#define MAX_CDE_PARAMS		64
#define MAX_CDE_USER_PARAMS	32
#define MAX_CDE_OBJ_IDS		4

struct dma_buf;
struct gk20a;

/*
 * this element defines a buffer that is allocated and mapped into gpu address
 * space. data_byte_offset defines the beginning of the buffer inside the
 * firmare. num_bytes defines how many bytes the firmware contains.
 *
 * If data_byte_offset is zero, we allocate an empty buffer.
 */

struct gk20a_cde_hdr_buf {
	u64 data_byte_offset;
	u64 num_bytes;
};

/*
 * this element defines a constant patching in buffers. It basically
 * computes physical address to <source_buf>+source_byte_offset. The
 * address is then modified into patch value as per:
 *    value = (current_value & ~mask) | (address << shift) & mask .
 *
 * The type field defines the register size as:
 *  0=u32,
 *  1=u64 (little endian),
 *  2=u64 (big endian)
 */

struct gk20a_cde_hdr_replace {
	u32 target_buf;
	u32 source_buf;
	s32 shift;
	u32 type;
	s64 target_byte_offset;
	s64 source_byte_offset;
	u64 mask;
};

enum {
	TYPE_PARAM_TYPE_U32 = 0,
	TYPE_PARAM_TYPE_U64_LITTLE,
	TYPE_PARAM_TYPE_U64_BIG
};

/*
 * this element defines a runtime patching in buffers. Parameters with id from
 * 0 to 1024 are reserved for special usage as follows:
 *   0 = comptags_per_cacheline,
 *   1 = slices_per_fbp,
 *   2 = num_fbps
 *   3 = source buffer first page offset
 *   4 = source buffer block height log2
 *   5 = backing store memory address
 *   6 = destination memory address
 *   7 = destination size (bytes)
 *   8 = backing store size (bytes)
 *   9 = cache line size
 *
 * Parameters above id 1024 are user-specified. I.e. they determine where a
 * parameters from user space should be placed in buffers, what is their
 * type, etc.
 *
 * Once the value is available, we add data_offset to the value.
 *
 * The value address is then modified into patch value as per:
 *    value = (current_value & ~mask) | (address << shift) & mask .
 *
 * The type field defines the register size as:
 *  0=u32,
 *  1=u64 (little endian),
 *  2=u64 (big endian)
 */

struct gk20a_cde_hdr_param {
	u32 id;
	u32 target_buf;
	s32 shift;
	u32 type;
	s64 data_offset;
	s64 target_byte_offset;
	u64 mask;
};

enum {
	TYPE_PARAM_COMPTAGS_PER_CACHELINE = 0,
	TYPE_PARAM_GPU_CONFIGURATION,
	TYPE_PARAM_FIRSTPAGEOFFSET,
	TYPE_PARAM_NUMPAGES,
	TYPE_PARAM_BACKINGSTORE,
	TYPE_PARAM_DESTINATION,
	TYPE_PARAM_DESTINATION_SIZE,
	TYPE_PARAM_BACKINGSTORE_SIZE,
	TYPE_PARAM_SOURCE_SMMU_ADDR,
	NUM_RESERVED_PARAMS = 1024,
};

/*
 * This header element defines a command. The op field determines whether the
 * element is defining an init (0) or convert command (1). data_byte_offset
 * denotes the beginning address of command elements in the file.
 */

struct gk20a_cde_hdr_command {
	u32 op;
	u32 num_entries;
	u64 data_byte_offset;
};

enum {
	TYPE_BUF_COMMAND_INIT = 0,
	TYPE_BUF_COMMAND_CONVERT
};

/*
 * This is a command element defines one entry inside push buffer. target_buf
 * defines the buffer including the pushbuffer entries, target_byte_offset the
 * offset inside the buffer and num_bytes the number of words in the buffer.
 */

struct gk20a_cde_cmd_elem {
	u32 target_buf;
	u32 padding;
	u64 target_byte_offset;
	u64 num_bytes;
};

/*
 * Following defines a single header element. Each element has a type and
 * some of the data structures.
 */

struct gk20a_cde_hdr_elem {
	u32 type;
	u32 padding;
	union {
		struct gk20a_cde_hdr_buf buf;
		struct gk20a_cde_hdr_replace replace;
		struct gk20a_cde_hdr_param param;
		u32 required_class;
		struct gk20a_cde_hdr_command command;
	};
};

enum {
	TYPE_BUF = 0,
	TYPE_REPLACE,
	TYPE_PARAM,
	TYPE_REQUIRED_CLASS,
	TYPE_COMMAND
};

struct gk20a_cde_mem_desc {
	struct sg_table *sgt;
	dma_addr_t iova;
	void *cpuva;
	size_t num_bytes;
	u64 gpu_va;
};

struct gk20a_cde_param {
	u32 id;
	u32 padding;
	u64 value;
};

struct gk20a_cde_ctx {
	struct gk20a *g;
	struct platform_device *pdev;

	/* channel related data */
	struct channel_gk20a *ch;
	struct vm_gk20a *vm;

	/* buf converter configuration */
	struct gk20a_cde_mem_desc mem[MAX_CDE_BUFS];
	int num_bufs;

	/* buffer patching params (where should patching be done) */
	struct gk20a_cde_hdr_param params[MAX_CDE_PARAMS];
	int num_params;

	/* storage for user space parameter values */
	u32 user_param_values[MAX_CDE_USER_PARAMS];

	u64 src_smmu_addr;
	u32 src_param_offset;
	u32 src_param_lines;

	u64 src_vaddr;

	u64 dest_vaddr;
	u64 dest_size;

	u32 obj_ids[MAX_CDE_OBJ_IDS];
	int num_obj_ids;

	u64 backing_store_vaddr;

	struct nvhost_gpfifo *init_cmd;
	int init_cmd_num_entries;

	struct nvhost_gpfifo *convert_cmd;
	int convert_cmd_num_entries;

	struct kobj_attribute attr;
};

struct gk20a_cde_app {
	bool initialised;
	struct mutex mutex;
	struct vm_gk20a *vm;

	struct gk20a_cde_ctx cde_ctx[1];
	int cde_ctx_ptr;
};

int gk20a_cde_destroy(struct gk20a *g);
int gk20a_init_cde_support(struct gk20a *g);
int gk20a_cde_reload(struct gk20a *g);
int gk20a_cde_convert(struct gk20a *g, struct dma_buf *src, struct dma_buf *dst,
		      s32 dst_kind, u64 dst_word_offset,
		      u32 dst_size, struct nvhost_fence *fence,
		      u32 __flags, struct gk20a_cde_param *params,
		      int num_params, struct gk20a_fence **fence_out);
void gk20a_cde_debugfs_init(struct platform_device *dev);

int gk20a_prepare_compressible_read(
		struct gk20a *g, u32 buffer_fd, u32 request, u64 offset,
		u64 compbits_hoffset, u64 compbits_voffset,
		u32 width, u32 height, u32 block_height_log2,
		u32 submit_flags, struct nvhost_fence *fence,
		u32 *valid_compbits, struct gk20a_fence **fence_out);
int gk20a_mark_compressible_write(
		struct gk20a *g, u32 buffer_fd, u32 valid_compbits, u64 offset);

#endif
