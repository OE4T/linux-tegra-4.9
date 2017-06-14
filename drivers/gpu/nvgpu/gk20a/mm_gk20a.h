/*
 * GK20A memory management
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef MM_GK20A_H
#define MM_GK20A_H

#include <linux/scatterlist.h>
#include <linux/iommu.h>
#include <linux/version.h>
#include <asm/dma-iommu.h>
#include <asm/cacheflush.h>

#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/allocator.h>
#include <nvgpu/vm.h>
#include <nvgpu/list.h>
#include <nvgpu/rbtree.h>
#include <nvgpu/kref.h>

struct nvgpu_pd_cache;

#ifdef CONFIG_ARM64
#define outer_flush_range(a, b)
#define __cpuc_flush_dcache_area __flush_dcache_area
#endif

#define FLUSH_CPU_DCACHE(va, pa, size)	\
	do {	\
		__cpuc_flush_dcache_area((void *)(va), (size_t)(size));	\
		outer_flush_range(pa, pa + (size_t)(size));		\
	} while (0)

struct gpfifo_desc {
	struct nvgpu_mem mem;
	u32 entry_num;

	u32 get;
	u32 put;

	bool wrap;

	/* if gpfifo lives in vidmem or is forced to go via PRAMIN, first copy
	 * from userspace to pipe and then from pipe to gpu buffer */
	void *pipe;
};

struct patch_desc {
	struct nvgpu_mem mem;
	u32 data_count;
};

struct zcull_ctx_desc {
	u64 gpu_va;
	u32 ctx_attr;
	u32 ctx_sw_mode;
};

struct pm_ctx_desc {
	struct nvgpu_mem mem;
	u32 pm_mode;
};

struct compbit_store_desc {
	struct nvgpu_mem mem;

	/* The value that is written to the hardware. This depends on
	 * on the number of ltcs and is not an address. */
	u64 base_hw;
};

struct gk20a_buffer_state {
	struct nvgpu_list_node list;

	/* The valid compbits and the fence must be changed atomically. */
	struct nvgpu_mutex lock;

	/* Offset of the surface within the dma-buf whose state is
	 * described by this struct (one dma-buf can contain multiple
	 * surfaces with different states). */
	size_t offset;

	/* A bitmask of valid sets of compbits (0 = uncompressed). */
	u32 valid_compbits;

	/* The ZBC color used on this buffer. */
	u32 zbc_color;

	/* This struct reflects the state of the buffer when this
	 * fence signals. */
	struct gk20a_fence *fence;
};

static inline struct gk20a_buffer_state *
gk20a_buffer_state_from_list(struct nvgpu_list_node *node)
{
	return (struct gk20a_buffer_state *)
		((uintptr_t)node - offsetof(struct gk20a_buffer_state, list));
};

struct gk20a_comptags {
	u32 offset;
	u32 lines;
	u32 allocated_lines;
	bool user_mappable;
};

struct priv_cmd_queue {
	struct nvgpu_mem mem;
	u32 size;	/* num of entries in words */
	u32 put;	/* put for priv cmd queue */
	u32 get;	/* get for priv cmd queue */
};

struct priv_cmd_entry {
	bool valid;
	struct nvgpu_mem *mem;
	u32 off;	/* offset in mem, in u32 entries */
	u64 gva;
	u32 get;	/* start of entry in queue */
	u32 size;	/* in words */
};

struct gk20a;
struct channel_gk20a;

int gk20a_init_mm_support(struct gk20a *g);
int gk20a_init_mm_setup_sw(struct gk20a *g);
int gk20a_init_mm_setup_hw(struct gk20a *g);
void gk20a_init_mm_ce_context(struct gk20a *g);

int gk20a_mm_fb_flush(struct gk20a *g);
void gk20a_mm_l2_flush(struct gk20a *g, bool invalidate);
void gk20a_mm_cbc_clean(struct gk20a *g);
void gk20a_mm_l2_invalidate(struct gk20a *g);

#define FAULT_TYPE_NUM		2	/* replay and nonreplay faults */

struct mmu_fault_info {
	u64	inst_ptr;
	u32	inst_aperture;
	u64	fault_addr;
	u32	fault_addr_aperture;
	u32	timestamp_lo;
	u32	timestamp_hi;
	u32	mmu_engine_id;
	u32	gpc_id;
	u32	client_type;
	u32	client_id;
	u32	fault_type;
	u32	access_type;
	u32	protected_mode;
	u32	replayable_fault;
	u32	replay_fault_en;
	u32	valid;
	u32	faulted_pbdma;
	u32	faulted_engine;
	u32	faulted_subid;
	u32	chid;
	struct channel_gk20a *refch;
	const char *client_type_desc;
	const char *fault_type_desc;
	const char *client_id_desc;
};

struct mm_gk20a {
	struct gk20a *g;

	/* GPU VA default sizes address spaces for channels */
	struct {
		u64 user_size;   /* userspace-visible GPU VA region */
		u64 kernel_size; /* kernel-only GPU VA region */
	} channel;

	struct {
		u32 aperture_size;
		struct vm_gk20a *vm;
		struct nvgpu_mem inst_block;
	} bar1;

	struct {
		u32 aperture_size;
		struct vm_gk20a *vm;
		struct nvgpu_mem inst_block;
	} bar2;

	struct {
		u32 aperture_size;
		struct vm_gk20a *vm;
		struct nvgpu_mem inst_block;
	} pmu;

	struct {
		/* using pmu vm currently */
		struct nvgpu_mem inst_block;
	} hwpm;

	struct {
		struct vm_gk20a *vm;
		struct nvgpu_mem inst_block;
	} perfbuf;

	struct {
		struct vm_gk20a *vm;
	} cde;

	struct {
		struct vm_gk20a *vm;
	} ce;

	struct nvgpu_pd_cache *pd_cache;

	struct nvgpu_mutex l2_op_lock;
	struct nvgpu_mutex tlb_lock;
	struct nvgpu_mutex priv_lock;

	struct nvgpu_mem bar2_desc;

#ifdef CONFIG_TEGRA_19x_GPU
	struct nvgpu_mem hw_fault_buf[FAULT_TYPE_NUM];
	unsigned int hw_fault_buf_status[FAULT_TYPE_NUM];
	struct mmu_fault_info *fault_info[FAULT_TYPE_NUM];
	struct nvgpu_mutex hub_isr_mutex;
	u32    hub_intr_types;
#endif
	/*
	 * Separate function to cleanup the CE since it requires a channel to
	 * be closed which must happen before fifo cleanup.
	 */
	void (*remove_ce_support)(struct mm_gk20a *mm);
	void (*remove_support)(struct mm_gk20a *mm);
	bool sw_ready;
	int physical_bits;
	bool use_full_comp_tag_line;
	bool ltc_enabled_current;
	bool ltc_enabled_target;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
	u32 bypass_smmu;
	u32 disable_bigpage;
#else
	bool bypass_smmu;
	bool disable_bigpage;
#endif
	bool has_physical_mode;

	struct nvgpu_mem sysmem_flush;

	u32 pramin_window;
	struct nvgpu_spinlock pramin_window_lock;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
	u32 force_pramin; /* via debugfs */
#else
	bool force_pramin; /* via debugfs */
#endif

	struct {
		size_t size;
		u64 base;
		size_t bootstrap_size;
		u64 bootstrap_base;

		struct nvgpu_allocator allocator;
		struct nvgpu_allocator bootstrap_allocator;

		u32 ce_ctx_id;
		volatile bool cleared;
		struct nvgpu_mutex first_clear_mutex;

		struct nvgpu_list_node clear_list_head;
		struct nvgpu_mutex clear_list_mutex;

		struct work_struct clear_mem_worker;
		atomic64_t bytes_pending;
	} vidmem;
};

int gk20a_mm_init(struct mm_gk20a *mm);

#define gk20a_from_mm(mm) ((mm)->g)
#define gk20a_from_vm(vm) ((vm)->mm->g)

#define dev_from_vm(vm) dev_from_gk20a(vm->mm->g)

#define DEFAULT_ALLOC_ALIGNMENT (4*1024)

static inline int bar1_aperture_size_mb_gk20a(void)
{
	return 16; /* 16MB is more than enough atm. */
}

/* The maximum GPU VA range supported */
#define NV_GMMU_VA_RANGE          38

/* The default userspace-visible GPU VA size */
#define NV_MM_DEFAULT_USER_SIZE   (1ULL << 37)

/* The default kernel-reserved GPU VA size */
#define NV_MM_DEFAULT_KERNEL_SIZE (1ULL << 32)

/*
 * When not using unified address spaces, the bottom 56GB of the space are used
 * for small pages, and the remaining high memory is used for large pages.
 */
static inline u64 __nv_gmmu_va_small_page_limit(void)
{
	return ((u64)SZ_1G * 56);
}

enum gmmu_pgsz_gk20a __get_pte_size_fixed_map(struct vm_gk20a *vm,
					      u64 base, u64 size);
enum gmmu_pgsz_gk20a __get_pte_size(struct vm_gk20a *vm, u64 base, u64 size);

void set_vidmem_page_alloc(struct scatterlist *sgl, u64 addr);
bool is_vidmem_page_alloc(u64 addr);
struct nvgpu_page_alloc *get_vidmem_page_alloc(struct scatterlist *sgl);

#if 0 /*related to addr bits above, concern below TBD on which is accurate */
#define bar1_instance_block_shift_gk20a() (max_physaddr_bits_gk20a() -\
					   bus_bar1_block_ptr_s())
#else
#define bar1_instance_block_shift_gk20a() bus_bar1_block_ptr_shift_v()
#endif

int gk20a_alloc_inst_block(struct gk20a *g, struct nvgpu_mem *inst_block);
void gk20a_free_inst_block(struct gk20a *g, struct nvgpu_mem *inst_block);
void gk20a_init_inst_block(struct nvgpu_mem *inst_block, struct vm_gk20a *vm,
		u32 big_page_size);
u64 gk20a_mm_inst_block_addr(struct gk20a *g, struct nvgpu_mem *mem);

void gk20a_mm_dump_vm(struct vm_gk20a *vm,
		u64 va_begin, u64 va_end, char *label);

int gk20a_mm_suspend(struct gk20a *g);

u64 gk20a_mm_gpu_phys_addr(struct gk20a *g, u64 phys, u32 flags);
u64 gk20a_mm_smmu_vaddr_translate(struct gk20a *g, dma_addr_t iova);

void gk20a_mm_ltc_isr(struct gk20a *g);

bool gk20a_mm_mmu_debug_mode_enabled(struct gk20a *g);

int gk20a_mm_mmu_vpr_info_fetch(struct gk20a *g);

static inline phys_addr_t gk20a_mem_phys(struct nvgpu_mem *mem)
{
	/* FIXME: the sgt/sgl may get null if this is accessed e.g. in an isr
	 * during channel deletion - attempt to fix at least null derefs */
	struct sg_table *sgt = mem->priv.sgt;

	if (sgt) {
		struct scatterlist *sgl = sgt->sgl;
		if (sgl)
			return sg_phys(sgl);
	}

	return 0;
}

u64 gk20a_locked_gmmu_map(struct vm_gk20a *vm,
			u64 map_offset,
			struct sg_table *sgt,
			u64 buffer_offset,
			u64 size,
			int pgsz_idx,
			u8 kind_v,
			u32 ctag_offset,
			u32 flags,
			int rw_flag,
			bool clear_ctags,
			bool sparse,
			bool priv,
			struct vm_gk20a_mapping_batch *batch,
			enum nvgpu_aperture aperture);

void gk20a_locked_gmmu_unmap(struct vm_gk20a *vm,
			u64 vaddr,
			u64 size,
			int pgsz_idx,
			bool va_allocated,
			int rw_flag,
			bool sparse,
			struct vm_gk20a_mapping_batch *batch);

struct sg_table *gk20a_mm_pin(struct device *dev, struct dma_buf *dmabuf);
void gk20a_mm_unpin(struct device *dev, struct dma_buf *dmabuf,
		    struct sg_table *sgt);

int nvgpu_vm_get_compbits_info(struct vm_gk20a *vm,
			       u64 mapping_gva,
			       u64 *compbits_win_size,
			       u32 *compbits_win_ctagline,
			       u32 *mapping_ctagline,
			       u32 *flags);

/* vm-as interface */
struct nvgpu_as_alloc_space_args;
struct nvgpu_as_free_space_args;
int gk20a_vm_release_share(struct gk20a_as_share *as_share);
int gk20a_vm_bind_channel(struct gk20a_as_share *as_share,
			  struct channel_gk20a *ch);
int __gk20a_vm_bind_channel(struct vm_gk20a *vm, struct channel_gk20a *ch);

int gk20a_vidmem_buf_alloc(struct gk20a *g, size_t bytes);
int gk20a_vidmem_get_space(struct gk20a *g, u64 *space);
int gk20a_vidbuf_access_memory(struct gk20a *g, struct dma_buf *dmabuf,
		void *buffer, u64 offset, u64 size, u32 cmd);

void gk20a_get_comptags(struct device *dev, struct dma_buf *dmabuf,
			struct gk20a_comptags *comptags);
dma_addr_t gk20a_mm_gpuva_to_iova_base(struct vm_gk20a *vm, u64 gpu_vaddr);

int gk20a_dmabuf_alloc_drvdata(struct dma_buf *dmabuf, struct device *dev);

int gk20a_dmabuf_get_state(struct dma_buf *dmabuf, struct gk20a *g,
			   u64 offset, struct gk20a_buffer_state **state);

void pde_range_from_vaddr_range(struct vm_gk20a *vm,
					      u64 addr_lo, u64 addr_hi,
					      u32 *pde_lo, u32 *pde_hi);
int gk20a_mm_pde_coverage_bit_count(struct vm_gk20a *vm);
u32 gk20a_mm_get_physical_addr_bits(struct gk20a *g);

const struct gk20a_mmu_level *gk20a_mm_get_mmu_levels(struct gk20a *g,
						      u32 big_page_size);
void gk20a_mm_init_pdb(struct gk20a *g, struct nvgpu_mem *mem,
		struct vm_gk20a *vm);

extern const struct gk20a_mmu_level gk20a_mm_levels_64k[];
extern const struct gk20a_mmu_level gk20a_mm_levels_128k[];

int gk20a_mm_get_buffer_info(struct device *dev, int dmabuf_fd,
			     u64 *buffer_id, u64 *buffer_len);
void gk20a_vm_unmap_locked_kref(struct kref *ref);

#endif /* MM_GK20A_H */
