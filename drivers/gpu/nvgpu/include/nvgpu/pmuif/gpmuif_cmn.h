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
#ifndef _GPMUIFCMN_H_
#define _GPMUIFCMN_H_

/*
 * Defines the logical queue IDs that must be used when submitting
 * commands to the PMU
 */
/* write by sw, read by pmu, protected by sw mutex lock */
#define PMU_COMMAND_QUEUE_HPQ		0
/* write by sw, read by pmu, protected by sw mutex lock */
#define PMU_COMMAND_QUEUE_LPQ		1
/* read/write by sw/hw, protected by hw pmu mutex, id = 2 */
#define PMU_COMMAND_QUEUE_BIOS		2
/* read/write by sw/hw, protected by hw pmu mutex, id = 3 */
#define PMU_COMMAND_QUEUE_SMI		3
/* write by pmu, read by sw, accessed by interrupt handler, no lock */
#define PMU_MESSAGE_QUEUE		4
#define PMU_QUEUE_COUNT			5

#define PMU_IS_COMMAND_QUEUE(id)	\
		((id)  < PMU_MESSAGE_QUEUE)

#define PMU_IS_SW_COMMAND_QUEUE(id)	\
		(((id) == PMU_COMMAND_QUEUE_HPQ) || \
		 ((id) == PMU_COMMAND_QUEUE_LPQ))

#define  PMU_IS_MESSAGE_QUEUE(id)	\
		((id) == PMU_MESSAGE_QUEUE)

enum {
	OFLAG_READ = 0,
	OFLAG_WRITE
};

#define QUEUE_SET		(true)
#define QUEUE_GET		(false)

#define QUEUE_ALIGNMENT		(4)

/* An enumeration containing all valid logical mutex identifiers */
enum {
	PMU_MUTEX_ID_RSVD1 = 0,
	PMU_MUTEX_ID_GPUSER,
	PMU_MUTEX_ID_QUEUE_BIOS,
	PMU_MUTEX_ID_QUEUE_SMI,
	PMU_MUTEX_ID_GPMUTEX,
	PMU_MUTEX_ID_I2C,
	PMU_MUTEX_ID_RMLOCK,
	PMU_MUTEX_ID_MSGBOX,
	PMU_MUTEX_ID_FIFO,
	PMU_MUTEX_ID_PG,
	PMU_MUTEX_ID_GR,
	PMU_MUTEX_ID_CLK,
	PMU_MUTEX_ID_RSVD6,
	PMU_MUTEX_ID_RSVD7,
	PMU_MUTEX_ID_RSVD8,
	PMU_MUTEX_ID_RSVD9,
	PMU_MUTEX_ID_INVALID
};

#define PMU_MUTEX_ID_IS_VALID(id)	\
		((id) < PMU_MUTEX_ID_INVALID)

#define PMU_INVALID_MUTEX_OWNER_ID	(0)

/*
 * The PMU's frame-buffer interface block has several slots/indices
 * which can be bound to support DMA to various surfaces in memory
 */
enum {
	PMU_DMAIDX_UCODE         = 0,
	PMU_DMAIDX_VIRT          = 1,
	PMU_DMAIDX_PHYS_VID      = 2,
	PMU_DMAIDX_PHYS_SYS_COH  = 3,
	PMU_DMAIDX_PHYS_SYS_NCOH = 4,
	PMU_DMAIDX_RSVD          = 5,
	PMU_DMAIDX_PELPG         = 6,
	PMU_DMAIDX_END           = 7
};

/*
 * Falcon PMU DMA's minimum size in bytes.
 */
#define PMU_DMA_MIN_READ_SIZE_BYTES		16
#define PMU_DMA_MIN_WRITE_SIZE_BYTES	4

#define PMU_FB_COPY_RW_ALIGNMENT	\
	((PMU_DMA_MIN_READ_SIZE_BYTES > PMU_DMA_MIN_WRITE_SIZE_BYTES) ?	\
	PMU_DMA_MIN_READ_SIZE_BYTES : PMU_DMA_MIN_WRITE_SIZE_BYTES)

/*
 * Macros to make aligned versions of RM_PMU_XXX structures. PMU needs aligned
 * data structures to issue DMA read/write operations.
 */
#define NV_PMU_MAKE_ALIGNED_STRUCT(name, size)           \
union name##_aligned {		                         \
		struct name data;                        \
		u8 pad[ALIGN_UP(sizeof(struct name),     \
		(PMU_FB_COPY_RW_ALIGNMENT))];            \
}

#define NV_PMU_MAKE_ALIGNED_UNION(name, size)	         \
union name##_aligned {		                         \
		union name data;                         \
		u8 pad[ALIGN_UP(sizeof(union name),      \
		(PMU_FB_COPY_RW_ALIGNMENT))];            \
}

#endif /* _GPMUIFCMN_H_*/
