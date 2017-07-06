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
 */

#ifndef __FALCON_H__
#define __FALCON_H__

#include <nvgpu/types.h>
#include <nvgpu/lock.h>

/*
 * Falcon Id Defines
 */
#define FALCON_ID_PMU       (0)
#define FALCON_ID_FECS      (2)
#define FALCON_ID_GPCCS     (3)
#define FALCON_ID_SEC2      (7)

/*
 * Falcon Base address Defines
 */
#define FALCON_PWR_BASE    0x0010a000
#define FALCON_SEC_BASE    0x00087000
#define FALCON_FECS_BASE   0x00409000
#define FALCON_GPCCS_BASE  0x0041a000

/* Falcon Register index */
#define FALCON_REG_R0		(0)
#define FALCON_REG_R1		(1)
#define FALCON_REG_R2		(2)
#define FALCON_REG_R3		(3)
#define FALCON_REG_R4		(4)
#define FALCON_REG_R5		(5)
#define FALCON_REG_R6		(6)
#define FALCON_REG_R7		(7)
#define FALCON_REG_R8		(8)
#define FALCON_REG_R9		(9)
#define FALCON_REG_R10		(10)
#define FALCON_REG_R11		(11)
#define FALCON_REG_R12		(12)
#define FALCON_REG_R13		(13)
#define FALCON_REG_R14		(14)
#define FALCON_REG_R15		(15)
#define FALCON_REG_IV0		(16)
#define FALCON_REG_IV1		(17)
#define FALCON_REG_UNDEFINED	(18)
#define FALCON_REG_EV		(19)
#define FALCON_REG_SP		(20)
#define FALCON_REG_PC		(21)
#define FALCON_REG_IMB		(22)
#define FALCON_REG_DMB		(23)
#define FALCON_REG_CSW		(24)
#define FALCON_REG_CCR		(25)
#define FALCON_REG_SEC		(26)
#define FALCON_REG_CTX		(27)
#define FALCON_REG_EXCI		(28)
#define FALCON_REG_RSVD0	(29)
#define FALCON_REG_RSVD1	(30)
#define FALCON_REG_RSVD2	(31)
#define FALCON_REG_SIZE		(32)

/*
 * Falcon HWCFG request read types defines
 */
enum flcn_hwcfg_read {
	FALCON_IMEM_SIZE = 0,
	FALCON_DMEM_SIZE,
	FALCON_CORE_REV,
	FALCON_SECURITY_MODEL,
	FLACON_MAILBOX_COUNT
};

/*
 * Falcon HWCFG request write types defines
 */
enum flcn_hwcfg_write {
	FALCON_STARTCPU = 0,
	FALCON_STARTCPU_SECURE,
	FALCON_BOOTVEC,
	FALCON_ITF_EN
};

#define FALCON_MEM_SCRUBBING_TIMEOUT_MAX 1000
#define FALCON_MEM_SCRUBBING_TIMEOUT_DEFAULT 10

enum flcn_dma_dir {
	DMA_TO_FB = 0,
	DMA_FROM_FB
};

enum flcn_mem_type {
	MEM_DMEM = 0,
	MEM_IMEM
};

struct nvgpu_falcon_dma_info {
	u32 fb_base;
	u32 fb_off;
	u32 flcn_mem_off;
	u32 size_in_bytes;
	enum flcn_dma_dir dir;
	u32 ctx_dma;
	enum flcn_mem_type flcn_mem;
	u32 is_wait_complete;
};

struct gk20a;
struct nvgpu_falcon;

struct nvgpu_falcon_version_ops {
	void (*start_cpu_secure)(struct nvgpu_falcon *flcn);
	void (*write_dmatrfbase)(struct nvgpu_falcon *flcn, u32 addr);
};

/* ops which are falcon engine specific */
struct nvgpu_falcon_engine_dependency_ops {
	int (*reset_eng)(struct gk20a *g);
};

struct nvgpu_falcon_ops {
	int (*reset)(struct nvgpu_falcon *flcn);
	void (*set_irq)(struct nvgpu_falcon *flcn, bool enable);
	bool (*clear_halt_interrupt_status)(struct nvgpu_falcon *flcn);
	bool (*is_falcon_cpu_halted)(struct nvgpu_falcon *flcn);
	bool (*is_falcon_idle)(struct nvgpu_falcon *flcn);
	bool (*is_falcon_scrubbing_done)(struct nvgpu_falcon *flcn);
	int (*copy_from_dmem)(struct nvgpu_falcon *flcn, u32 src, u8 *dst,
		u32 size, u8 port);
	int (*copy_to_dmem)(struct nvgpu_falcon *flcn, u32 dst, u8 *src,
		u32 size, u8 port);
	int (*copy_from_imem)(struct nvgpu_falcon *flcn, u32 src, u8 *dst,
		u32 size, u8 port);
	int (*copy_to_imem)(struct nvgpu_falcon *flcn, u32 dst, u8 *src,
		u32 size, u8 port, bool sec, u32 tag);
	int (*dma_copy)(struct nvgpu_falcon *flcn,
			struct nvgpu_falcon_dma_info *dma_info);
	u32 (*mailbox_read)(struct nvgpu_falcon *flcn, u32 mailbox_index);
	void (*mailbox_write)(struct nvgpu_falcon *flcn, u32 mailbox_index,
		u32 data);
	int (*bootstrap)(struct nvgpu_falcon *flcn, u32 boot_vector);
	void (*dump_falcon_stats)(struct nvgpu_falcon *flcn);
};

struct nvgpu_falcon {
	struct gk20a *g;
	u32 flcn_id;
	u32 flcn_base;
	u32 flcn_core_rev;
	bool is_falcon_supported;
	bool is_interrupt_enabled;
	u32 intr_mask;
	u32 intr_dest;
	bool isr_enabled;
	struct nvgpu_mutex isr_mutex;
	struct nvgpu_mutex copy_lock;
	struct nvgpu_falcon_ops flcn_ops;
	struct nvgpu_falcon_version_ops flcn_vops;
	struct nvgpu_falcon_engine_dependency_ops flcn_engine_dep_ops;
};

int nvgpu_flcn_wait_idle(struct nvgpu_falcon *flcn);
int nvgpu_flcn_wait_for_halt(struct nvgpu_falcon *flcn, unsigned int timeout);
int nvgpu_flcn_clear_halt_intr_status(struct nvgpu_falcon *flcn,
		unsigned int timeout);
int nvgpu_flcn_reset(struct nvgpu_falcon *flcn);
void nvgpu_flcn_set_irq(struct nvgpu_falcon *flcn, bool enable,
	u32 intr_mask, u32 intr_dest);
bool nvgpu_flcn_get_mem_scrubbing_status(struct nvgpu_falcon *flcn);
bool nvgpu_flcn_get_cpu_halted_status(struct nvgpu_falcon *flcn);
bool nvgpu_flcn_get_idle_status(struct nvgpu_falcon *flcn);
int nvgpu_flcn_copy_from_dmem(struct nvgpu_falcon *flcn,
	u32 src, u8 *dst, u32 size, u8 port);
int nvgpu_flcn_copy_to_dmem(struct nvgpu_falcon *flcn,
	u32 dst, u8 *src, u32 size, u8 port);
int nvgpu_flcn_copy_to_imem(struct nvgpu_falcon *flcn,
	u32 dst, u8 *src, u32 size, u8 port, bool sec, u32 tag);
int nvgpu_flcn_dma_copy(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_dma_info *dma_info);
u32 nvgpu_flcn_mailbox_read(struct nvgpu_falcon *flcn, u32 mailbox_index);
void nvgpu_flcn_mailbox_write(struct nvgpu_falcon *flcn, u32 mailbox_index,
	u32 data);
int nvgpu_flcn_bootstrap(struct nvgpu_falcon *flcn, u32 boot_vector);
void nvgpu_flcn_print_dmem(struct nvgpu_falcon *flcn, u32 src, u32 size);
void nvgpu_flcn_print_imem(struct nvgpu_falcon *flcn, u32 src, u32 size);
void nvgpu_flcn_dump_stats(struct nvgpu_falcon *flcn);

void nvgpu_flcn_sw_init(struct gk20a *g, u32 flcn_id);


#endif /* __FALCON_H__ */
